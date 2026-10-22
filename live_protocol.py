"""Protocol, decoding and persistence helpers for the live ESP32/ADS1299 recorder.

The firmware in this repository emulates a Cyton-style board (see
``docs/OPENBCI_COMPATIBILITY.md``). This module contains everything the live GUI
needs that is not Qt specific:

* building Cyton command bytes (``x...X``, ``~n``, ``b``/``s``, ``v`` ...)
* splitting the mixed serial stream into 33-byte data packets and text replies
* turning packets into microvolt sample blocks, including the interleaved
  Cyton+Daisy layout
* a streaming filter chain that mirrors the offline analyzer's filter options
* settings persistence, so the hardware configuration survives restarts

Keeping this Qt-free makes the whole data path testable without a display.
"""

from __future__ import annotations

import json
import queue
import re
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from scipy import signal

VREF_VOLTS = 4.5
ADC_FULL_SCALE = float((1 << 23) - 1)
PACKET_LENGTH = 33
PACKET_START_BYTE = 0xA0
PACKET_END_BYTE = 0xC0
AUX_OFFSET = 26

GAIN_SCALARS = (1, 2, 4, 6, 8, 12, 24)
GAIN_LABELS = ("x1", "x2", "x4", "x6", "x8", "x12", "x24")
INPUT_TYPE_LABELS = (
    "Normal",
    "Shorted",
    "Bias Meas",
    "MVDD",
    "Temp",
    "Test Sig",
    "BIAS DRP",
    "BIAS DRN",
)
SAMPLE_RATES = (
    (16000, 0),
    (8000, 1),
    (4000, 2),
    (2000, 3),
    (1000, 4),
    (500, 5),
    (250, 6),
)
SAMPLE_RATE_BY_CODE = {code: rate for rate, code in SAMPLE_RATES}
CODE_BY_SAMPLE_RATE = {rate: code for rate, code in SAMPLE_RATES}
BOARD_CHANNEL_CHOICES = (("Cyton + Daisy (16 channels)", 16), ("Cyton (8 channels)", 8))

# Cyton selectors for the second half of a 16-channel board.
DAISY_SELECTORS = ("Q", "W", "E", "R", "T", "Y", "U", "I")

# The LIS3DH is configured for +/-4 g and the firmware forwards the raw 16-bit
# register value, which is 12-bit data left justified. Stock OpenBCI uses the
# same conversion.
ACCEL_G_PER_COUNT = 0.002 / 16.0
ACCEL_COUNTS_PER_G = 1.0 / ACCEL_G_PER_COUNT


def gain_uv_per_count(gain: float) -> float:
    """Microvolts per ADS1299 LSB for the given PGA gain."""

    return VREF_VOLTS / (ADC_FULL_SCALE * float(gain)) * 1e6


def gain_scalar_from_ordinal(ordinal: int) -> int:
    if 0 <= ordinal < len(GAIN_SCALARS):
        return GAIN_SCALARS[ordinal]
    return GAIN_SCALARS[-1]


def max_sustained_packet_rate(baud: int) -> int:
    """Packets per second the serial link can carry with 10% headroom."""

    usable_bits = (max(int(baud), 9600) // 10) * 9
    return max(usable_bits // (PACKET_LENGTH * 10), 1)


def supported_packet_rates(baud: int) -> list[tuple[int, int]]:
    limit = max_sustained_packet_rate(baud)
    return [(rate, code) for rate, code in SAMPLE_RATES if rate <= limit]


def sample_rate_for(code: int, board_channels: int) -> float:
    """Per-channel sample rate; daisy mode halves the packet rate."""

    rate = float(SAMPLE_RATE_BY_CODE.get(int(code), 250))
    return rate / 2.0 if int(board_channels) > 8 else rate


def physical_channel_map(board_channels: int) -> list[Optional[int]]:
    """Map GUI channel index to the physical ADS channel index (None = padding).

    The ADS1299-4 build produces 8 real channels. The ADS1299-6 build produces
    12 real channels inside a 16-channel Cyton+Daisy layout, so GUI channels
    7, 8, 15 and 16 have no hardware behind them.
    """

    if int(board_channels) <= 8:
        return list(range(8))

    mapping: list[Optional[int]] = [None] * 16
    for slot in range(6):
        mapping[slot] = slot
        mapping[8 + slot] = 6 + slot
    return mapping


def real_channel_indices(board_channels: int) -> list[int]:
    return [index for index, real in enumerate(physical_channel_map(board_channels)) if real is not None]


def channel_selector(channel_index: int) -> str:
    """Return the ``x`` command selector for a zero-based GUI channel index."""

    if channel_index < 8:
        return str(channel_index + 1)
    if channel_index < 16:
        return DAISY_SELECTORS[channel_index - 8]
    raise ValueError(f"channel index out of range: {channel_index}")


@dataclass(slots=True)
class ChannelSetting:
    """One channel of ``CHnSET`` plus the bias/SRB switches it drives."""

    active: bool = True
    gain_ordinal: int = 6
    input_ordinal: int = 0
    bias: bool = True
    srb2: bool = True
    srb1: bool = False

    @property
    def gain_scalar(self) -> int:
        return gain_scalar_from_ordinal(self.gain_ordinal)

    def digits(self) -> str:
        return "".join(
            (
                "0" if self.active else "1",
                str(min(max(int(self.gain_ordinal), 0), 6)),
                str(min(max(int(self.input_ordinal), 0), 7)),
                "1" if self.bias else "0",
                "1" if self.srb2 else "0",
                "1" if self.srb1 else "0",
            )
        )

    def copy(self) -> "ChannelSetting":
        return ChannelSetting(
            active=self.active,
            gain_ordinal=self.gain_ordinal,
            input_ordinal=self.input_ordinal,
            bias=self.bias,
            srb2=self.srb2,
            srb1=self.srb1,
        )


def build_channel_command(channel_index: int, setting: ChannelSetting) -> bytes:
    """``x`` + selector + 6 digits + ``X`` as understood by the firmware."""

    return f"x{channel_selector(channel_index)}{setting.digits()}X".encode("ascii")


def build_sample_rate_command(code: int) -> bytes:
    return f"~{int(code)}".encode("ascii")


@dataclass(slots=True)
class TextLine:
    text: str
    kind: str = "info"


def classify_text(text: str) -> str:
    lowered = text.lower()
    if text.startswith("#"):
        if lowered.startswith("#err"):
            return "error"
        if lowered.startswith("#ok"):
            return "success"
        if lowered.startswith("#state"):
            return "state"
        return "debug"
    if "failure" in lowered or lowered.startswith("error"):
        return "error"
    if "success" in lowered:
        return "success"
    return "info"

class MixedStreamParser:
    """Split a Cyton serial stream into data packets and text replies.

    The firmware interleaves 33-byte binary packets with ASCII replies, and
    some replies end with ``$$$`` (Cyton style) while the boot log ends with
    newlines. Both forms are handled without dropping bytes.
    """

    def __init__(self) -> None:
        self._mode = "idle"
        self._packet = bytearray()
        self._text = bytearray()
        self.stray_bytes = 0
        self.packets = 0

    def reset(self) -> None:
        self._mode = "idle"
        self._packet.clear()
        self._text.clear()

    def feed(self, data: bytes) -> list[object]:
        events: list[object] = []
        for byte in data:
            if self._mode == "packet":
                self._packet.append(byte)
                if len(self._packet) == PACKET_LENGTH:
                    packet = bytes(self._packet)
                    self._packet.clear()
                    self._mode = "idle"
                    if packet[0] == PACKET_START_BYTE and packet[-1] == PACKET_END_BYTE:
                        self.packets += 1
                        events.append(packet)
                    else:
                        self._resync(packet)
                continue

            if self._mode == "text":
                if self._is_text_byte(byte):
                    self._text.append(byte)
                    self._maybe_flush_text(events)
                else:
                    self._flush_text(events)
                    self._handle_idle_byte(byte, events)
                continue

            self._handle_idle_byte(byte, events)

        return events

    def flush(self) -> list[object]:
        events: list[object] = []
        self._flush_text(events)
        return events

    @staticmethod
    def _is_text_byte(byte: int) -> bool:
        return byte in (9, 10, 13) or 32 <= byte <= 126

    def _handle_idle_byte(self, byte: int, events: list[object]) -> None:
        if byte == PACKET_START_BYTE:
            self._mode = "packet"
            self._packet = bytearray([byte])
            return
        if self._is_text_byte(byte):
            self._mode = "text"
            self._text = bytearray([byte])
            self._maybe_flush_text(events)
            return
        self.stray_bytes += 1

    def _resync(self, packet: bytes) -> None:
        """Attempt to recover a packet whose start byte was a false positive."""

        self.stray_bytes += len(packet)
        restart = packet.rfind(bytes([PACKET_START_BYTE]))
        if restart > 0:
            self._mode = "packet"
            self._packet = bytearray(packet[restart:])

    def _maybe_flush_text(self, events: list[object]) -> None:
        if self._text.endswith(b"$$$"):
            del self._text[-3:]
            self._flush_text(events)
            return
        if self._text.endswith(b"\n"):
            self._flush_text(events)

    def _flush_text(self, events: list[object]) -> None:
        self._mode = "idle"
        if not self._text:
            return
        text = self._text.decode("utf-8", errors="replace").replace("\r", "").strip()
        self._text = bytearray()
        if text:
            events.append(TextLine(text=text, kind=classify_text(text)))


def decode_i24(msb: int, mid: int, lsb: int) -> int:
    value = (msb << 16) | (mid << 8) | lsb
    if value & 0x800000:
        value -= 0x1000000
    return value


def decode_aux(packet: bytes) -> np.ndarray:
    raw = np.frombuffer(packet, dtype=np.uint8, count=6, offset=AUX_OFFSET).astype(np.int16)
    values = (raw[0::2] << 8) | (raw[1::2] & 0xFF)
    return values.astype(np.float64)


class PacketDecoder:
    """Convert 33-byte packets into microvolt sample vectors.

    Returns a vector sized to the GUI channel count so callers can append it to
    a ring buffer without further bookkeeping. In daisy mode the board and daisy
    halves arrive on alternating packets; one 16-channel sample is produced for
    each complete pair.
    """

    STALE_PAIR_SECONDS = 0.12

    def __init__(self, board_channels: int = 16) -> None:
        self.board_channels = int(board_channels)
        self._gains = [24.0] * (12 if self.board_channels > 8 else 8)
        self._board_slots: Optional[np.ndarray] = None
        self._board_time = 0.0
        self._daisy_slots: Optional[np.ndarray] = None
        self._daisy_time = 0.0
        self.last_aux = np.zeros(3, dtype=np.float64)
        self.packet_count = 0
        self.packet_loss = 0
        self._last_index: Optional[int] = None

    def set_gains(self, scalars: Sequence[float]) -> None:
        gains = [float(value) if value else 24.0 for value in scalars]
        if gains:
            self._gains = gains

    def _channel_scale(self, real_index: int) -> float:
        gain = self._gains[real_index] if real_index < len(self._gains) else 24.0
        return gain_uv_per_count(gain)

    def _decode_slots(self, packet: bytes, daisy: bool) -> np.ndarray:
        values = np.zeros(8, dtype=np.float64)
        for slot in range(8):
            offset = 2 + slot * 3
            counts = decode_i24(packet[offset], packet[offset + 1], packet[offset + 2])
            if self.board_channels > 8:
                if slot >= 6:
                    continue
                real = 6 + slot if daisy else slot
            else:
                real = slot
            values[slot] = counts * self._channel_scale(real)
        return values

    def push(self, packet: bytes) -> Optional[np.ndarray]:
        now = time.monotonic()
        self.packet_count += 1
        index = packet[1]
        if self._last_index is not None:
            delta = (index - self._last_index) & 0xFF
            if 1 < delta < 128:
                self.packet_loss += delta - 1
        self._last_index = index
        self.last_aux = decode_aux(packet)

        board_packet = (index & 1) != 0
        slots = self._decode_slots(packet, daisy=not board_packet)

        if self.board_channels <= 8:
            return slots

        if board_packet:
            self._board_slots = slots
            self._board_time = now
            return None

        self._daisy_slots = slots
        self._daisy_time = now
        if self._board_slots is None:
            return None
        if now - self._board_time > self.STALE_PAIR_SECONDS:
            return None
        return self._combine()

    def _combine(self) -> np.ndarray:
        assert self._board_slots is not None and self._daisy_slots is not None
        combined = np.zeros(16, dtype=np.float64)
        combined[0:6] = self._board_slots[0:6]
        combined[8:14] = self._daisy_slots[0:6]
        return combined

@dataclass(slots=True)
class FilterState:
    """Display filter configuration shared with the settings file."""

    notch_50: bool = False
    notch_100: bool = False
    quality_factor: float = 30.0
    high_pass_hz: float = 0.0
    low_pass_hz: float = 0.0

    def key(self) -> tuple[bool, bool, float, float, float]:
        return (
            self.notch_50,
            self.notch_100,
            round(float(self.quality_factor), 3),
            round(float(self.high_pass_hz), 3),
            round(float(self.low_pass_hz), 3),
        )

    def is_active(self, sample_rate: float) -> bool:
        if self.notch_50 and 50.0 < 0.49 * sample_rate:
            return True
        if self.notch_100 and 100.0 < 0.49 * sample_rate:
            return True
        nyquist = 0.5 * sample_rate
        if 0.0 < self.high_pass_hz < 0.99 * nyquist:
            return True
        if 0.0 < self.low_pass_hz < 0.99 * nyquist:
            return True
        return False


class StreamingFilter:
    """Causal filter chain applied sample-by-sample to the live display data."""

    def __init__(self, sample_rate: float, channel_count: int) -> None:
        self.sample_rate = float(sample_rate)
        self.channel_count = int(channel_count)
        self._sos: Optional[np.ndarray] = None
        self._zi: Optional[np.ndarray] = None
        self._key: Optional[tuple[bool, bool, float, float, float]] = None

    def configure(self, state: FilterState) -> None:
        key = state.key()
        if key == self._key:
            return
        self._key = key
        self._sos = self._build_sos(state)
        self.reset()

    def _build_sos(self, state: FilterState) -> Optional[np.ndarray]:
        fs = self.sample_rate
        nyquist = 0.5 * fs
        sections: list[np.ndarray] = []

        for enabled, frequency in ((state.notch_50, 50.0), (state.notch_100, 100.0)):
            if not enabled or frequency >= 0.49 * fs:
                continue
            b, a = signal.iirnotch(w0=frequency, Q=max(float(state.quality_factor), 1.0), fs=fs)
            sections.append(signal.tf2sos(b, a))

        high = max(float(state.high_pass_hz), 0.0)
        low = float(state.low_pass_hz)
        low = low if 0.0 < low < 0.99 * nyquist else 0.0
        high = high if 0.0 < high < 0.99 * nyquist else 0.0
        if high > 0.0 and 0.0 < low and high < low:
            sections.append(signal.butter(4, [high, low], btype="bandpass", fs=fs, output="sos"))
        elif high > 0.0 and low <= 0.0:
            sections.append(signal.butter(4, high, btype="highpass", fs=fs, output="sos"))
        elif low > 0.0 and high <= 0.0:
            sections.append(signal.butter(4, low, btype="lowpass", fs=fs, output="sos"))

        if not sections:
            return None
        return np.vstack(sections)

    def reset(self) -> None:
        self._zi = None

    def process(self, block: np.ndarray) -> np.ndarray:
        data = np.asarray(block, dtype=np.float64)
        if data.ndim != 2:
            return data
        if self._sos is None:
            return data
        if data.shape[1] != self.channel_count:
            self.channel_count = int(data.shape[1])
            self.reset()
        if self._zi is None:
            self._zi = np.zeros((self._sos.shape[0], 2, self.channel_count), dtype=np.float64)
        filtered, self._zi = signal.sosfilt(self._sos, data, axis=0, zi=self._zi)
        return filtered


class RingBuffer:
    """Fixed-capacity circular buffer for multichannel sample blocks."""

    def __init__(self, capacity: int, channels: int, dtype=np.float64) -> None:
        self._data = np.zeros((max(int(capacity), 1), int(channels)), dtype=dtype)
        self._capacity = self._data.shape[0]
        self._write = 0
        self._filled = 0
        self.total = 0

    @property
    def channels(self) -> int:
        return int(self._data.shape[1])

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def filled(self) -> int:
        return self._filled

    def clear(self) -> None:
        self._write = 0
        self._filled = 0

    def append(self, block: np.ndarray) -> None:
        data = np.asarray(block, dtype=self._data.dtype)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[0] == 0:
            return
        if data.shape[1] != self._data.shape[1]:
            raise ValueError(f"expected {self._data.shape[1]} channels, got {data.shape[1]}")
        if data.shape[0] >= self._capacity:
            data = data[-self._capacity :]
        count = data.shape[0]
        first = min(count, self._capacity - self._write)
        self._data[self._write : self._write + first] = data[:first]
        if count > first:
            self._data[: count - first] = data[first:]
        self._write = (self._write + count) % self._capacity
        self._filled = min(self._filled + count, self._capacity)
        self.total += count

    def last(self, count: int) -> Optional[np.ndarray]:
        if self._filled == 0:
            return None
        count = int(min(max(count, 1), self._filled))
        start = (self._write - count) % self._capacity
        if start + count <= self._capacity:
            return self._data[start : start + count]
        return np.vstack((self._data[start:], self._data[: (start + count) % self._capacity]))


@dataclass(slots=True)
class Annotation:
    label: str
    start_s: float
    end_s: float
    note: str = ""

    @property
    def duration_s(self) -> float:
        return max(0.0, float(self.end_s) - float(self.start_s))

    def as_dict(self) -> dict[str, object]:
        return {
            "label": self.label,
            "start_s": round(float(self.start_s), 4),
            "end_s": round(float(self.end_s), 4),
            "note": self.note,
        }


def format_duration(seconds: float) -> str:
    total = max(0.0, float(seconds))
    minutes, remainder = divmod(total, 60.0)
    hours, minutes = divmod(int(minutes), 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{remainder:05.2f}"
    return f"{minutes:02d}:{remainder:05.2f}"


def sanitize_filename(text: str, fallback: str = "recording") -> str:
    cleaned = re.sub(r"[^0-9A-Za-z._-]+", "_", (text or "").strip()).strip("._-")
    return cleaned or fallback

@dataclass(slots=True)
class HardwareSettings:
    """Persisted board configuration that is replayed on every connect."""

    board_channels: int = 16
    sample_rate_code: int = 6
    channels: list[ChannelSetting] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.ensure_channels()

    def ensure_channels(self) -> None:
        target = 16 if int(self.board_channels) > 8 else 8
        if len(self.channels) < target:
            self.channels.extend(ChannelSetting() for _ in range(target - len(self.channels)))
        while len(self.channels) > target:
            self.channels.pop()

    def set_board_channels(self, board_channels: int) -> None:
        self.board_channels = int(board_channels)
        self.ensure_channels()

    @property
    def sample_rate_hz(self) -> float:
        return sample_rate_for(self.sample_rate_code, self.board_channels)

    @property
    def packet_rate_hz(self) -> float:
        return float(SAMPLE_RATE_BY_CODE.get(int(self.sample_rate_code), 250))

    def physical_gains(self) -> list[float]:
        """PGA scalar per physical ADS channel, in decoder order."""

        mapping = physical_channel_map(self.board_channels)
        gains = [24.0] * (12 if self.board_channels > 8 else 8)
        for gui_index, real in enumerate(mapping):
            if real is None or gui_index >= len(self.channels):
                continue
            gains[real] = float(self.channels[gui_index].gain_scalar)
        return gains

    def as_dict(self) -> dict[str, object]:
        return {
            "board_channels": int(self.board_channels),
            "sample_rate_code": int(self.sample_rate_code),
            "channels": [
                {
                    "active": channel.active,
                    "gain_ordinal": int(channel.gain_ordinal),
                    "input_ordinal": int(channel.input_ordinal),
                    "bias": channel.bias,
                    "srb2": channel.srb2,
                    "srb1": channel.srb1,
                }
                for channel in self.channels
            ],
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "HardwareSettings":
        channels = [
            ChannelSetting(
                active=bool(item.get("active", True)),
                gain_ordinal=int(item.get("gain_ordinal", 6)),
                input_ordinal=int(item.get("input_ordinal", 0)),
                bias=bool(item.get("bias", True)),
                srb2=bool(item.get("srb2", True)),
                srb1=bool(item.get("srb1", False)),
            )
            for item in payload.get("channels", [])
            if isinstance(item, dict)
        ]
        return cls(
            board_channels=int(payload.get("board_channels", 16)),
            sample_rate_code=int(payload.get("sample_rate_code", 6)),
            channels=channels,
        )


@dataclass(slots=True)
class ViewSettings:
    """Persisted display configuration for the live plots."""

    vertical_scale_uv: float = 100.0
    window_seconds: float = 5.0
    visible_rows: int = 8
    center_channels: bool = True
    fft_points: int = 1024
    fft_window: str = "Hann"
    fft_smoothing: float = 0.0
    fft_scale_mode: float = 0.0
    fft_log_scale: bool = False
    fft_channel: str = "all"
    fft_end_hz: float = 0.0
    record_visible_only: bool = False
    accel_full_scale_g: float = 2.0
    accel_motion_threshold_g: float = 0.08
    accel_baseline_seconds: float = 2.0
    accel_counts_per_g: float = ACCEL_COUNTS_PER_G
    notch_50: bool = False
    notch_100: bool = False
    notch_q: float = 30.0
    high_pass_hz: float = 0.0
    low_pass_hz: float = 0.0
    visible_channels: list[bool] = field(default_factory=list)

    def ensure_visibility(self, channel_count: int) -> None:
        if len(self.visible_channels) < channel_count:
            self.visible_channels.extend([True] * (channel_count - len(self.visible_channels)))
        if len(self.visible_channels) > channel_count:
            del self.visible_channels[channel_count:]

    def filter_state(self) -> FilterState:
        return FilterState(
            notch_50=self.notch_50,
            notch_100=self.notch_100,
            quality_factor=self.notch_q,
            high_pass_hz=self.high_pass_hz,
            low_pass_hz=self.low_pass_hz,
        )

    def as_dict(self) -> dict[str, object]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "ViewSettings":
        defaults = cls()
        values: dict[str, object] = {}
        for name in defaults.as_dict():
            if name in payload:
                values[name] = payload[name]
        try:
            return cls(**values)  # type: ignore[arg-type]
        except TypeError:
            return defaults


@dataclass(slots=True)
class AppSettings:
    """Everything the live window remembers between sessions."""

    hardware: HardwareSettings = field(default_factory=HardwareSettings)
    view: ViewSettings = field(default_factory=ViewSettings)
    last_port: str = ""
    baud_rate: int = 115200
    session_name: str = "session"
    output_dir: str = "recordings"

    def as_dict(self) -> dict[str, object]:
        return {
            "version": 1,
            "hardware": self.hardware.as_dict(),
            "view": self.view.as_dict(),
            "connection": {"last_port": self.last_port, "baud_rate": int(self.baud_rate)},
            "recording": {"session_name": self.session_name, "output_dir": self.output_dir},
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "AppSettings":
        connection = payload.get("connection") if isinstance(payload.get("connection"), dict) else {}
        recording = payload.get("recording") if isinstance(payload.get("recording"), dict) else {}
        hardware_payload = payload.get("hardware") if isinstance(payload.get("hardware"), dict) else {}
        view_payload = payload.get("view") if isinstance(payload.get("view"), dict) else {}
        return cls(
            hardware=HardwareSettings.from_dict(hardware_payload),
            view=ViewSettings.from_dict(view_payload),
            last_port=str(connection.get("last_port", "")),
            baud_rate=int(connection.get("baud_rate", 115200)),
            session_name=str(recording.get("session_name", "session")),
            output_dir=str(recording.get("output_dir", "recordings")),
        )


class SettingsStore:
    """JSON-backed store for hardware and view settings."""

    def __init__(self, path: Optional[Path] = None) -> None:
        if path is None:
            path = Path(__file__).resolve().with_name("settings") / "live_recorder.json"
        self.path = Path(path)

    def load(self) -> AppSettings:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return AppSettings()
        if not isinstance(payload, dict):
            return AppSettings()
        return AppSettings.from_dict(payload)

    def save(self, settings: AppSettings) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(settings.as_dict(), indent=2), encoding="utf-8")
        temporary.replace(self.path)

def save_annotation_sidecar(recording_path: Path, annotations: Sequence[Annotation]) -> Path:
    """Rewrite the annotation sidecar of an existing recording."""

    recording_path = Path(recording_path)
    sidecar = recording_path.with_suffix(recording_path.suffix + ".annotations.json")
    try:
        payload = json.loads(sidecar.read_text(encoding="utf-8")) if sidecar.exists() else {}
    except (OSError, ValueError):
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    payload.setdefault("recording", str(recording_path))
    payload["annotations"] = [annotation.as_dict() for annotation in annotations]
    sidecar.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return sidecar


class OpenBCIRecording:
    """OpenBCI-compatible CSV writer with an annotation sidecar.

    The layout matches what OpenBCI GUI writes (comment header, EXG columns in
    microvolts, accelerometer in g, marker and timestamp) so the analysis window
    can load a recording without special casing.
    """

    def __init__(
        self,
        path: Path,
        channel_labels: Sequence[str],
        channel_map: Sequence[Optional[int]],
        sample_rate: float,
        metadata: Optional[dict[str, str]] = None,
    ) -> None:
        self.path = Path(path)
        self.channel_labels = list(channel_labels)
        self.channel_map = list(channel_map)
        self.sample_rate = float(sample_rate)
        self.metadata = dict(metadata or {})
        self._handle = None
        self._queue: "queue.Queue[Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]]" = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._rows_written = 0
        self._error: Optional[str] = None

    @property
    def rows_written(self) -> int:
        return self._rows_written

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def is_active(self) -> bool:
        return self._thread is not None

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8", newline="\n")
        self._write_header()
        self._thread = threading.Thread(target=self._run, name="recording-writer", daemon=True)
        self._thread.start()

    def _write_header(self) -> None:
        assert self._handle is not None
        lines = [
            "%OpenBCI Raw EXG Data",
            f"%Number of channels = {len(self.channel_labels)}",
            f"%Sample Rate = {self.sample_rate:g} Hz",
        ]
        for key, value in self.metadata.items():
            if value:
                lines.append(f"%{key} = {value}")
        lines.append("%Data Format = EXG channels in microvolts, accelerometer in g, timestamp in seconds")
        columns = [*self.channel_labels, "Accel Channel 0", "Accel Channel 1", "Accel Channel 2", "Marker", "Timestamp"]
        lines.append(",".join(columns))
        self._handle.write("\n".join(lines) + "\n")
        self._handle.flush()

    def append(self, times_s: np.ndarray, data_uv: np.ndarray, accel_g: np.ndarray) -> None:
        if self._thread is None:
            return
        times = np.asarray(times_s, dtype=np.float64)
        data = np.asarray(data_uv, dtype=np.float64)
        accel = np.asarray(accel_g, dtype=np.float64)
        if data.ndim != 2 or data.shape[0] == 0:
            return
        if accel.ndim != 2 or accel.shape[1] != 3:
            accel = np.zeros((data.shape[0], 3), dtype=np.float64)
        count = min(data.shape[0], times.shape[0], accel.shape[0])
        if count <= 0:
            return
        self._queue.put((times[:count].copy(), data[:count].copy(), accel[:count].copy()))

    def _run(self) -> None:
        assert self._handle is not None
        buffer: list[str] = []
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.25)
                except queue.Empty:
                    if buffer:
                        self._handle.writelines(buffer)
                        self._handle.flush()
                        buffer.clear()
                    continue
                if item is None:
                    break
                times, data, accel = item
                for row in range(data.shape[0]):
                    self._rows_written += 1
                    values = ",".join(f"{value:.4f}" for value in data[row])
                    accel_values = ",".join(f"{value:.5f}" for value in accel[row])
                    buffer.append(f"{self._rows_written},{values},{accel_values},0,{times[row]:.6f}\n")
                if len(buffer) >= 512:
                    self._handle.writelines(buffer)
                    self._handle.flush()
                    buffer.clear()
        except Exception as exc:  # noqa: BLE001 - surfaced through the error property
            self._error = str(exc)
        finally:
            if buffer and self._handle is not None:
                try:
                    self._handle.writelines(buffer)
                except OSError:
                    pass

    def stop(self, annotations: Sequence[Annotation] = (), extra_metadata: Optional[dict[str, str]] = None) -> Path:
        if self._thread is not None:
            self._queue.put(None)
            self._thread.join(timeout=5.0)
            self._thread = None
        annotation_list = list(annotations)
        if self._handle is not None:
            try:
                for annotation in annotation_list:
                    note = annotation.note.replace("\n", " ")
                    self._handle.write(
                        f"%Annotation,{annotation.start_s:.4f},{annotation.end_s:.4f},{annotation.label},{note}\n"
                    )
            finally:
                self._handle.close()
                self._handle = None
        self._write_sidecar(annotation_list, extra_metadata or {})
        return self.path

    def _write_sidecar(self, annotations: Sequence[Annotation], extra: dict[str, str]) -> None:
        sidecar = self.path.with_suffix(self.path.suffix + ".annotations.json")
        payload = {
            "recording": str(self.path),
            "sample_rate_hz": self.sample_rate,
            "channel_labels": self.channel_labels,
            "channel_map": self.channel_map,
            "metadata": {**self.metadata, **extra},
            "annotations": [annotation.as_dict() for annotation in annotations],
        }
        try:
            sidecar.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except OSError:
            pass

class DemoBoardPort:
    """Stand-in for ``serial.Serial`` that emulates the ESP32 firmware.

    Used by the "DEMO" source in the port list so the GUI can be exercised
    without hardware, and by the automated smoke test.
    """

    def __init__(self, board_channels: int = 16, sample_rate_code: int = 6) -> None:
        self.board_channels = int(board_channels)
        self.sample_rate_code = int(sample_rate_code)
        self.notify_boot = True
        self.port = "DEMO"
        self.baudrate = 115200
        self._lock = threading.Lock()
        self._rx = bytearray()
        self._tx = bytearray()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._gains = [24.0] * (12 if self.board_channels > 8 else 8)
        self._rng = np.random.default_rng(20261022)
        self._phases = np.linspace(0.0, np.pi, max(self.board_channels, 8), endpoint=False)
        self._counter = 0
        self._moving_until = 0.0
        self._streaming = False
        self._start_time = time.monotonic()
        self._stream_start = self._start_time
        self._stream_base = 0

    @property
    def is_open(self) -> bool:
        return self._thread is not None

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def open(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="demo-board", daemon=True)
        self._thread.start()

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + 0.02
        while True:
            with self._lock:
                if self._rx:
                    chunk = bytes(self._rx[:size])
                    del self._rx[:size]
                    return chunk
            if time.monotonic() > deadline:
                return b""
            time.sleep(0.002)

    def write(self, data: bytes) -> int:
        with self._lock:
            self._tx.extend(data)
        self._process_commands()
        return len(data)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _emit_text(self, text: str, terminator: str = "$$$") -> None:
        with self._lock:
            self._rx.extend(text.encode("ascii", errors="replace"))
            if terminator:
                self._rx.extend(terminator.encode("ascii"))

    def _process_commands(self) -> None:
        with self._lock:
            data = bytes(self._tx)
            self._tx.clear()
        index = 0
        while index < len(data):
            char = chr(data[index])
            if char == "x":
                end = data.find(b"X", index + 1)
                if end < 0:
                    break
                self._apply_channel_command(data[index + 1 : end].decode("ascii", errors="replace"))
                index = end + 1
                continue
            if char == "v":
                self._emit_text(self._version_text(), terminator="")
                index += 1
                continue
            if char == "d":
                self._emit_text("Success: updating channel settings to default")
            elif char == "b":
                self._streaming = True
                self._stream_base = self._counter
                self._stream_start = time.monotonic()
            elif char == "s":
                self._streaming = False
                self._emit_text("Success: Stream stopped (missed=0 dropped=0 corrupt=0)")
            elif char == "~":
                if index + 1 < len(data):
                    next_char = chr(data[index + 1])
                    if next_char.isdigit():
                        self.sample_rate_code = int(next_char)
                        rate = SAMPLE_RATE_BY_CODE.get(self.sample_rate_code, 250)
                        self._emit_text(f"Success: Sample rate is {rate}Hz")
                        index += 2
                        continue
                self._emit_text("Failure: sample rate out of bounds")
            elif char == "?":
                self._emit_text("Board ADS Registers\nCH1SET=0x61 CH2SET=0x61\n$$$", terminator="")
            index += 1

    def _apply_channel_command(self, payload: str) -> None:
        if len(payload) < 7:
            self._emit_text("Failure: invalid channel command length")
            return
        selector, digits = payload[0], payload[1:7]
        logical: Optional[int] = None
        if selector.isdigit():
            logical = int(selector) - 1
        elif selector.upper() in DAISY_SELECTORS:
            logical = 8 + DAISY_SELECTORS.index(selector.upper())
        mapping = physical_channel_map(self.board_channels)
        if logical is None or logical >= len(mapping) or mapping[logical] is None:
            self._emit_text("Failure: channel not available")
            return
        gain_ordinal = int(digits[1]) if digits[1].isdigit() else 6
        real = mapping[logical]
        assert real is not None
        if real < len(self._gains):
            self._gains[real] = float(gain_scalar_from_ordinal(gain_ordinal))
        self._emit_text(f"Success: Channel set for {logical + 1}")

    def _version_text(self) -> str:
        lines = ["OpenBCI V3 8-16 channel", "On Board ADS1299 Device ID: 0x3D"]
        if self.board_channels > 8:
            lines.append("On Daisy ADS1299 Device ID: 0x3D")
        lines.extend(["LIS3DH Device ID: 0x33", "SD card: present", "Firmware: v3.1.5", "$$$"])
        return "\n".join(lines) + "\n"

    def _run(self) -> None:
        period = 1.0 / 250.0
        if self.notify_boot:
            boot = (
                "=== OpenBCI ESP32 power-on self test ===\n"
                f"Hardware    : ADS1299 x2 = {self.board_channels} channels\n"
                "--- SPI bus probe ---\n"
                "[ADS1] SPI + device ID             OK : id=0x3D (expected 0x3D)\n"
                "[AUX]  LIS3DH present              OK : address=0x18 WHO_AM_I=0x33\n"
                "=== POST PASSED ===\n"
            )
            self._emit_text(boot, terminator="")
        while not self._stop.is_set():
            if self._streaming:
                now = time.monotonic()
                # Emit whole conversions on a wall-clock grid so the emulated
                # sample rate stays accurate despite OS sleep granularity.
                due = self._stream_base + int((now - self._stream_start) / period)
                packets = bytearray()
                emitted = 0
                while self._counter < due and emitted < 400:
                    packets.extend(self._build_packet(now - self._stream_start))
                    emitted += 1
                if packets:
                    with self._lock:
                        self._rx.extend(packets)
            time.sleep(0.007)

    def _build_packet(self, elapsed: float) -> bytes:
        index = self._counter & 0xFF
        self._counter += 1
        t = self._counter / 250.0
        slots = np.zeros(8, dtype=np.int64)
        mapping = physical_channel_map(self.board_channels)
        board_packet = (index & 1) != 0
        if self.board_channels <= 8:
            for slot in range(8):
                slots[slot] = self._cell_value(mapping[slot], t)
        elif board_packet:
            for slot in range(6):
                slots[slot] = self._cell_value(mapping[slot], t)
        else:
            for slot in range(6):
                slots[slot] = self._cell_value(mapping[8 + slot], t)

        packet = bytearray(PACKET_LENGTH)
        packet[0] = PACKET_START_BYTE
        packet[1] = index
        for slot in range(8):
            raw = int(slots[slot]) & 0x00FFFFFF
            packet[2 + slot * 3] = (raw >> 16) & 0xFF
            packet[3 + slot * 3] = (raw >> 8) & 0xFF
            packet[4 + slot * 3] = raw & 0xFF
        aux = self._accel_counts(elapsed)
        packet[AUX_OFFSET : AUX_OFFSET + 6] = aux.astype(">i2").tobytes()
        packet[32] = PACKET_END_BYTE
        return bytes(packet)

    def _cell_value(self, real_index: Optional[int], t: float) -> int:
        if real_index is None:
            return 0
        gain = self._gains[real_index] if real_index < len(self._gains) else 24.0
        uv_per_count = gain_uv_per_count(gain)
        phase = float(self._phases[real_index % len(self._phases)])
        alpha_uv = 18.0 * np.sin(2 * np.pi * 10.0 * t + phase)
        slow_uv = 25.0 * np.sin(2 * np.pi * 0.7 * t + phase * 0.5)
        pickup_uv = 6.0 * np.sin(2 * np.pi * 50.0 * t + phase)
        noise_uv = float(self._rng.normal(0.0, 2.0))
        microvolts = alpha_uv + slow_uv + pickup_uv + noise_uv
        return int(round(microvolts / uv_per_count))

    def _accel_counts(self, elapsed: float) -> np.ndarray:
        if elapsed > self._moving_until and (elapsed % 9.0) < 0.1:
            self._moving_until = elapsed + 1.2
        movement = np.zeros(3)
        if elapsed < self._moving_until:
            movement = np.array(
                [
                    900.0 * np.sin(2 * np.pi * 3.1 * elapsed),
                    700.0 * np.sin(2 * np.pi * 2.3 * elapsed + 0.7),
                    600.0 * np.sin(2 * np.pi * 4.4 * elapsed + 1.9),
                ]
            )
        noise = self._rng.normal(0.0, 6.0, size=3)
        base = np.array([0.0, 0.0, 998.0])
        return np.round(base + movement + noise).astype(np.int16)