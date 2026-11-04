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

# ADS1299 SPI bus. This has to match ADS_SPI_CLOCK_HZ in the firmware; the app
# only uses it to sanity-check the budget the board reports.
ADS_SPI_CLOCK_HZ = 2_000_000
ADS_SPI_CLOCK_MIN_HZ = 1_000_000
ADS_SPI_CLOCK_MAX_HZ = 8_000_000
SPI_CLOCK_CHOICES = (
    (1_000_000, "1 MHz"),
    (2_000_000, "2 MHz (default)"),
    (4_000_000, "4 MHz"),
    (6_000_000, "6 MHz"),
    (8_000_000, "8 MHz (max)"),
)
ADS_SPI_FRAME_BITS = 3 * 8 + 8 * 8 * 3  # 3 status bytes + 8 x 24-bit samples = 216

# BR/EDR transmit power levels the controller actually implements. The board snaps a
# request to the nearest of these and reports what it read back, so only real levels
# are offered here.
BLUETOOTH_POWER_CHOICES = (
    (-12, "-12 dBm (minimum)"),
    (-6, "-6 dBm"),
    (0, "0 dBm"),
    (3, "3 dBm"),
    (4, "4 dBm (default)"),
    (6, "6 dBm"),
    (9, "9 dBm (maximum)"),
)

# SPP write size. 330 is the Arduino library's own chunk, which queues and returns;
# anything larger hands the write to esp_spp_write() directly and waits for each
# acknowledgement. Measured on this board the library path is the faster one, which is
# why it is the default and the larger sizes are marked as the experimental path.
SPP_WRITE_CHOICES = (
    (330, "330 B (library queue - fastest measured)"),
    (990, "990 B (direct - experimental)"),
    (1980, "1980 B (direct - experimental)"),
    (4096, "4096 B (direct - experimental)"),
)

# Output decimation. The converter runs at the selected output data rate and the
# firmware block-averages N conversions before it publishes a packet, so the
# packet rate stays where the serial/BLE link is comfortable while the ADS1299
# third-order sinc corner (-3 dB at 0.262 x ODR) moves out of the measurement
# band: 500 SPS -> 131 Hz, 2000 SPS -> 524 Hz, 4000 SPS -> 1048 Hz.
DECIMATION_CHOICES = (1, 2, 4, 5, 8, 10, 16, 20, 25, 32)
DECIMATION_MIN = 1
DECIMATION_MAX = 32

# Sinc3 -3 dB corner as a fraction of the output data rate.
SINC3_CORNER_FRACTION = 0.262


def sinc3_corner_hz(data_rate_hz: float) -> float:
    """-3 dB corner of the ADS1299 digital filter at the given data rate."""

    return SINC3_CORNER_FRACTION * float(data_rate_hz)


def clamp_spi_clock(value: object, default: int = ADS_SPI_CLOCK_HZ) -> int:
    try:
        clock = int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return max(ADS_SPI_CLOCK_MIN_HZ, min(ADS_SPI_CLOCK_MAX_HZ, clock))


def clamp_decimation(value: object) -> int:
    try:
        factor = int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return 1
    return max(DECIMATION_MIN, min(DECIMATION_MAX, factor))

# Default display band. Matches what the offline analyzer is usually driven
# with, so the live traces and the analysis look at the same signal.
DEFAULT_HIGH_PASS_HZ = 0.1
DEFAULT_LOW_PASS_HZ = 150.0
# Bumped whenever a stored settings file needs to adopt a new default.
SETTINGS_VERSION = 6

# Reference ("SRB") routing, as the silicon actually implements it.
#
# * SRB1 - MISC1 bit 5 closes one switch inside the converter that ties the
#   **inverting** input (INxN) of every channel to the SRB1 pin. The datasheet's
#   referential-montage section (SBAS499C, "Referential montage") describes exactly
#   this: reference electrode to SRB1, every active electrode to INxP. This is the
#   pin that provides a shared *negative* reference.
# * SRB2 - CHnSET bit 3 closes a switch between the SRB2 pin and that channel's
#   **non-inverting** input (INxP). It is per channel and chip-mode independent,
#   but it does NOT refer the channel to a negative reference: closing it drives
#   INP from the SRB2 pin and leaves INN to whatever is wired there.
# * Differential - neither switch closed, each channel uses its own INP and INN.
#
# Keeping these two distinct matters: wiring a reference electrode to SRB2 and
# expecting a referential montage leaves INN floating, which is why an SRB2 setup
# that "should" work reads as saturation or as pure floating-input noise.
REFERENCE_MODE_DIFFERENTIAL = "diff"
REFERENCE_MODE_SRB1 = "srb1"
REFERENCE_MODE_SRB2 = "srb2"
REFERENCE_MODE_CHOICES = (
    ("True differential (each channel uses its own INP and INN)", REFERENCE_MODE_DIFFERENTIAL),
    ("SRB1 - reference electrode to every INxN (referential montage)", REFERENCE_MODE_SRB1),
    ("SRB2 - SRB2 pin drives INxP (non-inverting), per channel", REFERENCE_MODE_SRB2),
)
REFERENCE_MODE_CODES = {
    REFERENCE_MODE_DIFFERENTIAL: 0,
    REFERENCE_MODE_SRB1: 1,
    REFERENCE_MODE_SRB2: 2,
}
# Compact labels for combo boxes and one-line status text. `report_export`
# re-exports this tuple so the UI and the reports can never disagree.
REFERENCE_MODE_SHORT = (
    ("Differential", REFERENCE_MODE_DIFFERENTIAL),
    ("SRB1->INN", REFERENCE_MODE_SRB1),
    ("SRB2->INP", REFERENCE_MODE_SRB2),
)
# value -> label. ``dict(REFERENCE_MODE_SHORT)`` is the opposite direction (what a
# combo box needs), so looking a mode up in it silently fell back to the raw code.
REFERENCE_MODE_SHORT_LABELS = {value: label for label, value in REFERENCE_MODE_SHORT}

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
    """Packets per second the serial link can carry with 10% headroom.

    This board reaches the host through a UART bridge (CP2102N), not native USB, so
    the baud rate is real wire time: 8N1 is 10 bits per byte, and 33 bytes per packet.
    At 3 Mbaud that is 300 kB/s, or about 8.2 k packets/s once 10% is kept back.
    """

    baud = max(int(baud), 9600)
    usable_bits = (baud // 10) * 9
    return max(usable_bits // (PACKET_LENGTH * 10), 1)


def spi_frame_microseconds(spi_clock_hz: int = ADS_SPI_CLOCK_HZ, device_count: int = 2) -> float:
    """Time on the wire for the RDATAC frames of every converter.

    Each ADS1299 answers one 216-bit frame and both are read inside the same
    sample period because they share the START line.
    """

    clock = max(float(spi_clock_hz), 1.0)
    return ADS_SPI_FRAME_BITS * max(int(device_count), 1) / clock * 1e6


def max_sample_rate_from_spi(spi_clock_hz: int = ADS_SPI_CLOCK_HZ, device_count: int = 2) -> int:
    """Samples per second the ADS1299 SPI bus can serve with 10% headroom."""

    frame_us = spi_frame_microseconds(spi_clock_hz, device_count)
    if frame_us <= 0.0:
        return 0
    return max(int(900_000.0 / frame_us), 1)


def max_usable_sample_rate(baud: int, spi_clock_hz: int = ADS_SPI_CLOCK_HZ, device_count: int = 2) -> int:
    """Tighter of the serial and SPI limits, in board samples per second."""

    return min(max_sustained_packet_rate(baud), max_sample_rate_from_spi(spi_clock_hz, device_count))


def supported_packet_rates(baud: int) -> list[tuple[int, int]]:
    limit = max_sustained_packet_rate(baud)
    return [(rate, code) for rate, code in SAMPLE_RATES if rate <= limit]


def sample_rate_for(code: int, board_channels: int, decimation: int = 1) -> float:
    """Per-channel sample rate of one ADS1299, after the firmware's block averaging.

    This is the rate the converters actually run at divided by the averaging factor.
    It deliberately does **not** depend on the board profile: every ADS1299 samples
    its own inputs at the selected data rate, and the firmware publishes one packet
    per converter per conversion, so each channel is published once per conversion
    whether one converter or two are fitted.
    """

    del board_channels  # kept for call-site compatibility; the rate does not depend on it
    return float(SAMPLE_RATE_BY_CODE.get(int(code), 250)) / max(int(decimation), 1)


def output_packet_rate(code: int, decimation: int = 1, converters: int = 2) -> float:
    """Packets per second the firmware publishes for one data-rate code.

    One packet per converter per conversion (Cyton+Daisy), or one per conversion on
    a single-converter board.
    """

    per_converter = float(SAMPLE_RATE_BY_CODE.get(int(code), 250)) / max(int(decimation), 1)
    return per_converter * max(int(converters), 1)


def rate_fits_link(
    data_rate_hz: float,
    packet_rate_hz: float,
    baud: int,
    spi_clock_hz: int = ADS_SPI_CLOCK_HZ,
    device_count: int = 2,
) -> tuple[bool, str]:
    """Check a (data rate, decimated packet rate) pair against both budgets."""

    serial_limit = max_sustained_packet_rate(baud)
    spi_limit = max_sample_rate_from_spi(spi_clock_hz, device_count)
    if float(packet_rate_hz) > serial_limit:
        return False, (
            f"{packet_rate_hz:g} packets/s needs "
            f"{packet_rate_hz * PACKET_LENGTH * 10 / 1000:.0f} kbit/s, "
            f"the {baud / 1000:.0f} kbit/s link carries {serial_limit:g}"
        )
    if float(data_rate_hz) > spi_limit:
        return False, (
            f"{data_rate_hz:g} SPS needs {spi_frame_microseconds(spi_clock_hz, device_count):.0f} us "
            f"per sample, SCLK at {spi_clock_hz / 1e6:g} MHz serves {spi_limit:g} SPS"
        )
    return True, ""


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
    # CHnSET bit 3: closes the switch that ties this channel's **INxP**
    # (non-inverting input) to the SRB2 pin. It is *not* a negative reference.
    srb2: bool = False
    # Derived from the converter's mode; MISC1 bit 5 owns it for the whole chip.
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


def build_reference_command(mode: str) -> bytes:
    """``N<0|1|2>``: one board-wide reference routing for every converter."""

    code = REFERENCE_MODE_CODES.get(str(mode), REFERENCE_MODE_CODES[REFERENCE_MODE_SRB2])
    return f"N{code}".encode("ascii")


def build_chip_reference_command(device_index: int, mode: str) -> bytes:
    """``n<chip><0|1|2>``: reference routing for one ADS1299 only.

    SRB1 is one switch per converter, so a board that measures the common-mode
    voltage on a true-differential input *and* tests SRB1 needs the two halves to
    run different reference modes.
    """

    code = REFERENCE_MODE_CODES.get(str(mode), REFERENCE_MODE_CODES[REFERENCE_MODE_SRB2])
    return f"n{int(device_index) + 1}{code}".encode("ascii")


def build_spi_clock_command(spi_clock_hz: int) -> bytes:
    """``o<kHz>o``: runtime SCLK change."""

    kilohertz = max(int(round(float(spi_clock_hz) / 1000.0)), 1)
    return f"o{kilohertz}o".encode("ascii")


def build_decimation_command(decimation: int) -> bytes:
    """``m<n>m``: block-average factor applied before a packet is published."""

    factor = min(max(int(decimation), DECIMATION_MIN), DECIMATION_MAX)
    return f"m{factor}m".encode("ascii")


def build_repair_command() -> bytes:
    """``>``: re-program every register from the firmware's cached state and
    report the readback check. Named with a symbol because every letter is
    already a Cyton channel selector."""

    return b">"


def boxcar_corner_hz(data_rate_hz: float, decimation: int) -> float:
    """-3 dB point of the MCU's block average, for the operator to see.

    A block average of N samples at f_s has its first null at f_s/N and its
    -3 dB point at roughly 0.443 * f_s/N. This is why averaging cannot lower
    noise *inside* the measurement band: it only removes what is above this
    corner, and it is the ADS1299 sample rate that sets the noise bandwidth.
    """

    factor = max(int(decimation), 1)
    return 0.443 * float(data_rate_hz) / factor


def boxcar_null_hz(data_rate_hz: float, decimation: int) -> float:
    factor = max(int(decimation), 1)
    return float(data_rate_hz) / factor


def noise_bandwidth_hz(data_rate_hz: float) -> float:
    """Effective noise bandwidth of the ADS1299 sinc3 filter.

    The datasheet's noise table uses ENBW = 1.22 x the -3 dB corner for the
    third-order sinc, so this is the number that explains why a higher data rate
    is noisier for the same signal band.
    """

    return 1.22 * sinc3_corner_hz(data_rate_hz)


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
    # 500 SPS with Cyton+Daisy interleaving = 250 Hz per channel, the fastest
    # rate that still fits the serial link comfortably at 921600 baud.
    sample_rate_code: int = 5
    reference_mode: str = REFERENCE_MODE_SRB2
    # One reference mode per ADS1299. SRB1 is a single switch inside each chip,
    # so the two converters can run different references even though the
    # recording name can only describe one - this is what lets one converter stay
    # true differential (measuring the common-mode voltage directly) while the
    # other is in SRB1 or SRB2.
    chip_reference_modes: list[str] = field(default_factory=list)
    spi_clock_hz: int = ADS_SPI_CLOCK_HZ
    decimation: int = 1
    # Bluetooth link tuning. The power is the controller's BR/EDR level and is
    # applied to the board on connect; the SPP write size is a host-side choice that
    # decides which firmware write path a throughput test uses.
    bluetooth_power_dbm: int = 4
    spp_write_bytes: int = 330
    channels: list[ChannelSetting] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.ensure_channels()

    def ensure_channels(self) -> None:
        target = 16 if int(self.board_channels) > 8 else 8
        if len(self.channels) < target:
            self.channels.extend(ChannelSetting() for _ in range(target - len(self.channels)))
        while len(self.channels) > target:
            self.channels.pop()
        self.sync_reference()

    def set_board_channels(self, board_channels: int) -> None:
        self.board_channels = int(board_channels)
        self.ensure_channels()

    def normalised_reference_mode(self) -> str:
        mode = str(self.reference_mode or REFERENCE_MODE_SRB2)
        return mode if mode in REFERENCE_MODE_CODES else REFERENCE_MODE_SRB2

    def normalised_chip_modes(self) -> list[str]:
        """Exactly one reference mode per converter."""

        modes: list[str] = []
        for value in list(self.chip_reference_modes)[:2]:
            modes.append(value if value in REFERENCE_MODE_CODES else self.normalised_reference_mode())
        while len(modes) < 2:
            modes.append(self.normalised_reference_mode())
        return modes

    @property
    def device_count(self) -> int:
        """How many ADS1299 converters this board profile actually has."""

        return 2 if int(self.board_channels) > 8 else 1

    @property
    def chip_modes_differ(self) -> bool:
        modes = self.normalised_chip_modes()[: self.device_count]
        return len(set(modes)) > 1

    def chip_mode_for_real(self, real_channel: int) -> str:
        per_device = channels_per_device(self.board_channels)
        device = min(int(real_channel) // max(per_device, 1), 1)
        return self.normalised_chip_modes()[device]

    def chip_indices(self, device: int) -> list[int]:
        """GUI channel indices that belong to one converter and are powered up."""

        per_device = channels_per_device(self.board_channels)
        device = int(device)
        result: list[int] = []
        for gui_index, real in enumerate(physical_channel_map(self.board_channels)):
            if real is None or gui_index >= len(self.channels):
                continue
            if (int(real) // max(per_device, 1)) != device:
                continue
            if self.channels[gui_index].active:
                result.append(gui_index)
        return result

    def resolve_chip_modes(self) -> list[str]:
        """Derive each converter's mode from its channel rows.

        The rows are the master: this is exactly what the firmware will be told,
        and it is the only place the two converters' states come from.

        ``SRB1`` is a single switch per chip (``MISC1`` bit 5), so a converter can
        only be in SRB1 when **every** powered-up input of that converter asks for
        it. Anything else is a plain per-input choice: ``SRB2`` when at least one
        input still uses the SRB2 pin (the firmware must then be in SRB2 mode for
        those inputs to get ``CHnSET`` bit 3), otherwise every input is on its own
        INN.
        """

        modes: list[str] = []
        for device in range(2):
            inputs = self.chip_indices(device)
            on_srb1 = [index for index in inputs if self.channels[index].srb1]
            if inputs and len(on_srb1) == len(inputs):
                modes.append(REFERENCE_MODE_SRB1)
            elif any(self.channels[index].srb2 for index in inputs) or not inputs:
                modes.append(REFERENCE_MODE_SRB2)
            else:
                modes.append(REFERENCE_MODE_DIFFERENTIAL)
        return modes

    def reference_conflicts(self) -> list[str]:
        """Per-input choices that the silicon cannot honour, if any.

        Only SRB1 can conflict: it is chip-wide, so mixing it with the SRB2 pin or
        with own-INN inputs on the same converter is not programmable.
        """

        problems: list[str] = []
        for device in range(2):
            inputs = self.chip_indices(device)
            on_srb1 = [index for index in inputs if self.channels[index].srb1]
            if on_srb1 and len(on_srb1) != len(inputs):
                problems.append(
                    f"ADS{device + 1}: SRB1 is one switch for the whole converter, so it cannot be "
                    f"mixed with other references (CH "
                    + ", ".join(str(index + 1) for index in on_srb1)
                    + ")"
                )
        return problems

    def sync_reference(self) -> None:
        """Make the per-input choices self-consistent and derive the chip modes.

        SRB1 is chip-wide, so if any powered-up input of a converter asks for it,
        the whole converter is SRB1 - that is the only state the hardware can be
        in, and resolving it here means a file saved by an older version (which
        stored the mode separately) still lands on a real configuration.

        Everything else is left alone: ``srb2`` is the per-input ``CHnSET`` bit 3
        and stays the operator's choice even while the converter is in SRB1 or
        differential mode, so returning to SRB2 restores the mix instead of
        turning every input into "SRB2 pin".
        """

        self.spi_clock_hz = clamp_spi_clock(self.spi_clock_hz)
        self.decimation = clamp_decimation(self.decimation)
        for device in range(2):
            inputs = self.chip_indices(device)
            if any(self.channels[index].srb1 for index in inputs):
                for index in inputs:
                    self.channels[index].srb1 = True
        for gui_index, real in enumerate(physical_channel_map(self.board_channels)):
            if real is None and gui_index < len(self.channels):
                self.channels[gui_index].srb1 = False
        modes = self.resolve_chip_modes()
        self.chip_reference_modes = modes
        self.reference_mode = modes[0] if modes else self.normalised_reference_mode()

    def set_channel_reference(self, gui_index: int, mode: str) -> None:
        """Set one row's reference, honouring the chip-wide SRB1 switch.

        Choosing SRB1 therefore moves the whole converter to SRB1 (that is the
        only thing it can do), and choosing anything else clears SRB1 for the
        converter while leaving the other inputs' SRB2-pin / own-INN choices
        untouched.
        """

        if not (0 <= int(gui_index) < len(self.channels)):
            return
        value = mode if mode in REFERENCE_MODE_CODES else REFERENCE_MODE_SRB2
        device = self._device_of_gui(int(gui_index))
        inputs = self.chip_indices(device)
        if value == REFERENCE_MODE_SRB1:
            for index in inputs:
                self.channels[index].srb1 = True
        else:
            for index in inputs:
                self.channels[index].srb1 = False
            self.channels[int(gui_index)].srb2 = value == REFERENCE_MODE_SRB2
        self.sync_reference()

    def _device_of_gui(self, gui_index: int) -> int:
        per_device = channels_per_device(self.board_channels)
        real = physical_channel_map(self.board_channels)[int(gui_index)]
        if real is None:
            return 0
        return min(int(real) // max(per_device, 1), 1)

    def set_reference_mode(self, mode: str) -> None:
        """Bulk-set both converters: every powered-up input follows the mode."""

        value = mode if mode in REFERENCE_MODE_CODES else REFERENCE_MODE_SRB2
        for device in range(2):
            self._apply_mode_to_chip(device, value)
        self.sync_reference()

    def set_chip_reference_mode(self, device_index: int, mode: str) -> None:
        """Bulk-set one converter (the one-click control above the table)."""

        value = mode if mode in REFERENCE_MODE_CODES else REFERENCE_MODE_SRB2
        self._apply_mode_to_chip(int(device_index), value)
        self.sync_reference()

    def _apply_mode_to_chip(self, device: int, mode: str) -> None:
        for index in self.chip_indices(device):
            channel = self.channels[index]
            channel.srb1 = mode == REFERENCE_MODE_SRB1
            if mode == REFERENCE_MODE_SRB2:
                channel.srb2 = True
            elif mode == REFERENCE_MODE_DIFFERENTIAL:
                channel.srb2 = False
            # SRB1 mode keeps the srb2 intent, which is unused while the chip is
            # in SRB1 and comes back if the converter returns to SRB2.

    def channel_connects_srb2(self, gui_index: int) -> bool:
        """True when this input's INxP is tied to the SRB2 pin right now.

        This is the non-inverting input, so a channel with this closed is *not*
        referred to a negative reference: its INN still has to be wired.
        """

        if not (0 <= int(gui_index) < len(self.channels)):
            return False
        channel = self.channels[gui_index]
        real = physical_channel_map(self.board_channels)[int(gui_index)]
        if real is None or not channel.active:
            return False
        return bool(channel.srb2)

    def channel_reference_short(self, gui_index: int) -> str:
        """One-word description of how an input's two sides are connected."""

        real = physical_channel_map(self.board_channels)[int(gui_index)]
        if real is None:
            return "n/a"
        mode = self.chip_mode_for_real(real)
        if mode == REFERENCE_MODE_SRB1:
            return "SRB1"
        if self.channel_connects_srb2(gui_index):
            return "SRB2>INP"
        return "own INN"

    def reference_summary(self) -> str:
        """Human readable reference state for the dock, including per-input mix."""

        short = REFERENCE_MODE_SHORT_LABELS
        modes = self.normalised_chip_modes()[: self.device_count]
        parts: list[str] = []
        for device, mode in enumerate(modes):
            per_device = channels_per_device(self.board_channels)
            inputs = [
                gui_index
                for gui_index, real in enumerate(physical_channel_map(self.board_channels))
                if real is not None and (int(real) // max(per_device, 1)) == device
                and self.channels[gui_index].active
            ]
            label = short.get(mode, mode)
            if mode == REFERENCE_MODE_SRB2 and inputs:
                driven = [index for index in inputs if self.channels[index].srb2]
                if driven:
                    shown = ", ".join(str(index + 1) for index in driven[:6])
                    if len(driven) > 6:
                        shown += "..."
                    label = f"{label}, {len(driven)} INP driven by SRB2 ({shown})"
            parts.append(f"ADS{device + 1}: {label}")
        return "  |  ".join(parts)

    def bias_channels(self) -> list[int]:
        """Powered-up inputs currently feeding the bias/DRL amplifier."""

        result: list[int] = []
        for device in range(self.device_count):
            for index in self.chip_indices(device):
                if self.channels[index].bias:
                    result.append(index)
        return result

    def reference_risks(self) -> list[str]:
        """Reasons the current routing will read as floating or as saturation.

        This is the check that was missing when an SRB2 setup "should" have worked:
        SRB2 drives the non-inverting input, so a channel whose INN has no path is
        an open input no matter how the register looks.
        """

        risks: list[str] = []
        modes = self.resolve_chip_modes()
        for device in range(self.device_count):
            mode = modes[device]
            inputs = self.chip_indices(device)
            if mode == REFERENCE_MODE_SRB2:
                driven = [index for index in inputs if self.channels[index].srb2]
                if driven:
                    risks.append(
                        f"ADS{device + 1}: the SRB2 switch is closed on "
                        + ", ".join(str(index + 1) for index in driven[:6])
                        + ("..." if len(driven) > 6 else "")
                        + ". That ties INxP to the SRB2 pin - it does not refer the channel to a "
                        "negative reference. Those inputs measure SRB2 against their own INN, so INN "
                        "must still be wired; for a reference electrode on the inverting input use SRB1."
                    )
        if not self.bias_channels():
            active_reference = any(mode != REFERENCE_MODE_DIFFERENTIAL for mode in modes) or any(
                self.channels[index].srb2
                for device in range(self.device_count)
                for index in self.chip_indices(device)
            )
            if active_reference:
                risks.append(
                    "No input feeds the bias/DRL amplifier, so the inputs have no DC return path. "
                    "A referenced or SRB2-driven input without bias drifts to a rail and looks exactly "
                    "like a floating electrode. Enable bias on at least one input and connect the BIAS "
                    "electrode."
                )
        return risks

    @property
    def reference_mode_code(self) -> int:
        return REFERENCE_MODE_CODES[self.normalised_reference_mode()]

    @property
    def reference_mode_label(self) -> str:
        modes = self.normalised_chip_modes()[: self.device_count]
        short = REFERENCE_MODE_SHORT_LABELS
        if len(set(modes)) > 1:
            return " / ".join(
                f"ADS{device + 1} {short.get(mode, mode)}" for device, mode in enumerate(modes)
            )
        mode = self.normalised_reference_mode()
        for label, value in REFERENCE_MODE_CHOICES:
            if value == mode:
                return label
        return mode

    @property
    def data_rate_hz(self) -> float:
        """ADS1299 output data rate (before the firmware's block averaging)."""

        return float(SAMPLE_RATE_BY_CODE.get(int(self.sample_rate_code), 250))

    @property
    def sample_rate_hz(self) -> float:
        return sample_rate_for(self.sample_rate_code, self.board_channels, self.decimation)

    @property
    def packet_rate_hz(self) -> float:
        return float(SAMPLE_RATE_BY_CODE.get(int(self.sample_rate_code), 250))

    @property
    def output_packet_rate_hz(self) -> float:
        converters = 2 if int(self.board_channels) > 8 else 1
        return output_packet_rate(self.sample_rate_code, self.decimation, converters)

    @property
    def sinc_corner_hz(self) -> float:
        return sinc3_corner_hz(self.data_rate_hz)

    def physical_gains(self) -> list[float]:
        """PGA scalar per physical ADS channel, in decoder order."""

        mapping = physical_channel_map(self.board_channels)
        gains = [24.0] * (12 if self.board_channels > 8 else 8)
        for gui_index, real in enumerate(mapping):
            if real is None or gui_index >= len(self.channels):
                continue
            gains[real] = float(self.channels[gui_index].gain_scalar)
        return gains

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
        mode = payload.get("reference_mode")
        if not isinstance(mode, str) or mode not in REFERENCE_MODE_CODES:
            # Files written before the reference mode existed carried per-channel
            # SRB flags; recover the single mode they were aiming for.
            active = [channel for channel in channels if channel.active] or channels
            if active and all(channel.srb1 for channel in active):
                mode = REFERENCE_MODE_SRB1
            elif active and not any(channel.srb2 for channel in active):
                mode = REFERENCE_MODE_DIFFERENTIAL
            else:
                mode = REFERENCE_MODE_SRB2
        chip_modes = payload.get("chip_reference_modes")
        if not isinstance(chip_modes, list) or not chip_modes:
            chip_modes = [mode, mode]
        chip_modes = [str(value) for value in chip_modes][:2]
        # Migrate a saved per-converter mode into the channel rows, which are now
        # the master. Older versions kept the mode separately and could leave the
        # rows without the matching SRB1 flag, which would silently demote an SRB1
        # converter to per-input routing.
        per_device = channels_per_device(int(payload.get("board_channels", 16)))
        mapping = physical_channel_map(int(payload.get("board_channels", 16)))
        for device, chip_mode in enumerate(chip_modes):
            if chip_mode != REFERENCE_MODE_SRB1:
                continue
            for gui_index, real in enumerate(mapping):
                if real is None or gui_index >= len(channels):
                    continue
                if (int(real) // max(per_device, 1)) == device:
                    channels[gui_index].srb1 = True
        try:
            spi_clock_hz = int(payload.get("spi_clock_hz", ADS_SPI_CLOCK_HZ) or ADS_SPI_CLOCK_HZ)
        except (TypeError, ValueError):
            spi_clock_hz = ADS_SPI_CLOCK_HZ
        try:
            decimation = int(payload.get("decimation", 1) or 1)
        except (TypeError, ValueError):
            decimation = 1
        return cls(
            board_channels=int(payload.get("board_channels", 16)),
            sample_rate_code=int(payload.get("sample_rate_code", 6)),
            reference_mode=mode,
            chip_reference_modes=[str(value) for value in chip_modes][:2],
            spi_clock_hz=spi_clock_hz,
            decimation=decimation,
            channels=channels,
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "board_channels": int(self.board_channels),
            "sample_rate_code": int(self.sample_rate_code),
            "reference_mode": self.normalised_reference_mode(),
            "chip_reference_modes": self.normalised_chip_modes(),
            "spi_clock_hz": int(self.spi_clock_hz),
            "decimation": int(self.decimation),
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


@dataclass(slots=True)
class DeviceExpectation:
    """Register bytes the firmware should hold for one ADS1299 after a write."""

    name: str
    config1: int
    config2: int
    config3: int
    channel_regs: list[int]
    bias_sensp: int
    bias_sensn: int
    misc1: int

    def describe(self, local_index: int, reg: int) -> str:
        """Human readable reason for a CHnSET byte."""

        gain_ordinal = (reg >> 4) & 0x07
        return f"gain x{GAIN_SCALARS[min(gain_ordinal, 6)]}"


def channel_register_value(setting: "ChannelSetting", chip_mode: str) -> int:
    """The CHnSET byte the firmware should hold for one channel.

    Mirrors ``ADS1299::configureChannel()``: powered-down channels carry 0x80 and
    MUX = 001 (input shorted), the gain sits in bits 6:4, SRB2 is bit 3 and the
    input type is bits 2:0.

    Bit 3 closes the switch between the SRB2 pin and this channel's **INxP**
    (non-inverting input). It is suppressed only while the converter is in SRB1
    mode, where the whole chip is already referred to the SRB1 pin and closing
    both would measure one reference pin against the other.
    """

    gain = min(max(int(setting.gain_ordinal), 0), 6)
    if not setting.active:
        return 0x80 | (gain << 4) | 0x01
    srb2 = 0x08 if (chip_mode == REFERENCE_MODE_SRB2 and setting.srb2) else 0x00
    return (gain << 4) | srb2 | (int(setting.input_ordinal) & 0x07)


def channels_per_device(board_channels: int) -> int:
    """Real ADS channels behind each of the two converters."""

    return 6 if int(board_channels) > 8 else 4


# ID register value per channel count (datasheet: NU_CH bits 7..3).
DEVICE_ID_BY_CHANNELS = {4: 0x3C, 6: 0x3D, 8: 0x3E}


def expected_device_registers(hardware: "HardwareSettings") -> list[DeviceExpectation]:
    """Compute the register image the firmware should have after applying settings.

    Mirrors ADS1299::configureChannel() and applyCytonDefaults():
    CHnSET = gain<<4 | SRB2 | input (plus 0x80 and input 1 when the channel is
    powered down), BIAS_SENSP/N carry one bit per bias-included channel,
    MISC1 bit 5 enables the SRB1 routing and CONFIG1 carries DAISY_EN plus
    CLK_EN on the clock master.
    """

    per_device = channels_per_device(hardware.board_channels)
    device_count = 2
    mapping = physical_channel_map(hardware.board_channels)
    rate_bits = int(hardware.sample_rate_code) & 0x07

    expectations: list[DeviceExpectation] = []
    for device in range(device_count):
        channel_regs: list[int] = []
        bias_sensp = 0
        bias_sensn = 0
        chip_mode = hardware.chip_mode_for_real(device * per_device)
        for local in range(per_device):
            real = device * per_device + local
            gui_index = next((index for index, value in enumerate(mapping) if value == real), None)
            setting = None
            if gui_index is not None and gui_index < len(hardware.channels):
                setting = hardware.channels[gui_index]
            if setting is None:
                channel_regs.append(0x61)
                continue
            gain = min(max(int(setting.gain_ordinal), 0), 6)
            if not setting.active:
                channel_regs.append(0x80 | (gain << 4) | 0x01)
                continue
            # SRB2 is only in the CHnSET byte while the converter is in SRB2
            # mode, which is what keeps a true-differential monitor input
            # differential on an otherwise SRB2-referred chip.
            channel_regs.append(channel_register_value(setting, chip_mode))
            if setting.bias:
                bias_sensp |= 1 << local
                bias_sensn |= 1 << local
        expectations.append(
            DeviceExpectation(
                name=f"ADS{device + 1}",
                config1=0x90 | 0x40 | (0x20 if device == 0 else 0x00) | rate_bits,
                config2=0xC0,
                config3=0xEC,
                channel_regs=channel_regs,
                bias_sensp=bias_sensp,
                bias_sensn=bias_sensn,
                misc1=0x20 if chip_mode == REFERENCE_MODE_SRB1 else 0x00,
            )
        )
    return expectations


@dataclass(slots=True)
class RegisterCheck:
    """One comparison between a reported register byte and the expected one."""

    device: str
    label: str
    actual: int
    expected: int
    mask: int = 0xFF

    @property
    def matches(self) -> bool:
        return (self.actual & self.mask) == (self.expected & self.mask)

    def describe(self) -> str:
        return f"{self.device} {self.label} 0x{self.actual:02X}, expected 0x{self.expected:02X}"


# Reserved bits of CONFIG1 and CONFIG2 read back as 0 on this part - so does
# DAISY_EN (CONFIG1 b6) and INT_CAL (CONFIG2 b4) - while CONFIG3, MISC1 and the
# CHnSET bytes read back exactly. Comparing whole bytes therefore reported a
# mismatch on a converter that is configured correctly and is streaming happily,
# which is worse than useless. Only the bits that decide behaviour and that the
# readback actually reflects are compared:
#
#   CONFIG1: CLK_EN (b5) | DR[2:0]                          -> 0x27
#   CONFIG2: INT_CAL (b4) | CAL_AMP (b2) | CAL_FREQ[1:0]    -> 0x17
#
# DAISY_EN is verified through the ACK of the write and the fact that both
# converters deliver samples; it is shown raw in the board-report line instead.
COMPARE_MASKS = {0x01: 0x27, 0x02: 0x17}

# The test-signal settings only mean anything while INT_CAL is on, so a stale
# CAL_FREQ left over from a test-signal run must not look like a fault.
CONFIG2_CAL_MASK = 0x07


def compare_configuration(
    report: dict[str, list[int]],
    hardware: "HardwareSettings",
) -> tuple[list[RegisterCheck], list[str]]:
    """Compare a readback report with the settings that were written.

    ``report`` maps device name to the 24 register bytes (0x00..0x17). Returns
    the mismatching register checks and a list of plain notes (missing
    converter, unexpected device ID, and so on).
    """

    checks: list[RegisterCheck] = []
    notes: list[str] = []
    expected_id = DEVICE_ID_BY_CHANNELS.get(channels_per_device(hardware.board_channels), 0x3E)
    for expectation in expected_device_registers(hardware):
        regs = report.get(expectation.name)
        if not regs or len(regs) < 0x18:
            notes.append(f"{expectation.name} did not report registers")
            continue
        if regs[0] != expected_id:
            # 0x3C/0x3D/0x3E are the ADS1299-4/-6/-8 IDs. Anything else means the
            # chip did not answer, so say so instead of blaming a channel.
            notes.append(
                f"{expectation.name} reports device ID 0x{regs[0]:02X}, expected 0x{expected_id:02X} "
                f"for {channels_per_device(hardware.board_channels)} channels"
            )
        for address, expected in (
            (0x01, expectation.config1),
            (0x02, expectation.config2),
            (0x03, expectation.config3),
            (0x0D, expectation.bias_sensp),
            (0x0E, expectation.bias_sensn),
            (0x15, expectation.misc1),
        ):
            actual = regs[address]
            mask = COMPARE_MASKS.get(address, 0xFF)
            if address == 0x02 and not (actual & 0x10) and not (expected & 0x10):
                # Test signal off on both sides: CAL_AMP/CAL_FREQ are don't-care,
                # so a leftover CAL_FREQ from an earlier test-signal run must not
                # read as a configuration fault.
                mask = 0x10
            check = RegisterCheck(
                expectation.name, REGISTER_LABELS[address], actual, expected, mask
            )
            if not check.matches:
                checks.append(check)
        for local, expected in enumerate(expectation.channel_regs):
            actual = regs[0x05 + local]
            if actual != expected:
                checks.append(RegisterCheck(expectation.name, f"CH{local + 1}SET", actual, expected))
    return checks, notes


REGISTER_LABELS = {
    0x00: "ID",    0x01: "CONFIG1",
    0x02: "CONFIG2",
    0x03: "CONFIG3",
    0x04: "LOFF",
    0x05: "CH1SET",
    0x06: "CH2SET",
    0x07: "CH3SET",
    0x08: "CH4SET",
    0x09: "CH5SET",
    0x0A: "CH6SET",
    0x0B: "CH7SET",
    0x0C: "CH8SET",
    0x0D: "BIAS_SENSP",
    0x0E: "BIAS_SENSN",
    0x0F: "LOFF_SENSP",
    0x10: "LOFF_SENSN",
    0x11: "LOFF_FLIP",
    0x12: "LOFF_STATP",
    0x13: "LOFF_STATN",
    0x14: "GPIO",
    0x15: "MISC1",
    0x16: "MISC2",
    0x17: "CONFIG4",
}


@dataclass(slots=True)
class RepairReport:
    """Result of the firmware's own repair-and-verify pass (``>`` command)."""

    devices: list[dict[str, object]] = field(default_factory=list)
    bad_registers: list[str] = field(default_factory=list)
    problems: int = 0
    summary: dict[str, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.problems == 0 and bool(self.devices)

    def describe(self) -> str:
        if not self.devices:
            return "The board did not answer the repair command."
        parts = [
            f"{device.get('DEV', '?')} ID 0x{int(device.get('ID', '0'), 16):02X}, "
            f"{device.get('REGOK', '?')}/{device.get('CHECKED', '?')} registers match"
            if str(device.get("ID", "")).strip()
            else f"{device.get('DEV', '?')} not linked"
            for device in self.devices
        ]
        verdict = "configuration repaired and verified" if self.ok else f"{self.problems} register(s) still wrong"
        return f"{verdict}: " + "; ".join(parts)


def parse_repair_report(lines: Sequence[str]) -> RepairReport:
    """Collect the ``[CHK]`` lines the firmware prints for a repair pass."""

    report = RepairReport()
    for line in lines:
        text = str(line).strip()
        if not text.startswith("[CHK]"):
            continue
        fields = dict(token.split("=", 1) for token in text[len("[CHK]"):].split() if "=" in token)
        if "REASON" in fields:
            report.summary = fields
            try:
                report.problems = int(fields.get("PROBLEMS", "0"))
            except ValueError:
                report.problems = 0
        elif " BAD " in f" {text} ":
            report.bad_registers.append(
                f"{fields.get('DEV', '?')} {fields.get('REG', '?')} "
                f"0x{fields.get('GOT', '??')}, expected 0x{fields.get('WANT', '??')}"
            )
        elif "DEV" in fields:
            report.devices.append(fields)
    return report


def parse_configuration_links(lines: Sequence[str]) -> dict[str, bool]:
    """Which converters said they were on the SPI bus, from the ``[CFG]`` lines."""

    links: dict[str, bool] = {}
    for line in lines:
        text = line.strip()
        if not text.startswith("[CFG]"):
            continue
        fields = dict(
            token.split("=", 1) for token in text[len("[CFG]"):].split() if "=" in token
        )
        name = fields.get("DEV")
        if name:
            links[name] = fields.get("LINK", "1") != "0"
    return links


def parse_configuration_report(lines: Sequence[str]) -> dict[str, list[int]]:
    """Collect ``[CFG] DEV=ADS1 LINK=1 REGS=<hex>`` lines into register lists."""

    report: dict[str, list[int]] = {}
    for line in lines:
        text = line.strip()
        if not text.startswith("[CFG]"):
            continue
        fields = dict(
            token.split("=", 1) for token in text[len("[CFG]"):].split() if "=" in token
        )
        name = fields.get("DEV")
        regs_hex = fields.get("REGS")
        if not name or not regs_hex:
            continue
        regs_hex = regs_hex.strip()
        if len(regs_hex) % 2:
            continue
        try:
            report[name] = [int(regs_hex[index:index + 2], 16) for index in range(0, len(regs_hex), 2)]
        except ValueError:
            continue
    return report


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
    fft_log_scale: bool = True
    fft_channel: str = "all"
    fft_end_hz: float = 0.0
    record_visible_only: bool = True
    accel_full_scale_g: float = 2.0
    accel_motion_threshold_g: float = 0.08
    accel_baseline_seconds: float = 2.0
    accel_counts_per_g: float = ACCEL_COUNTS_PER_G
    # Channel wired across the applied stimulus. It gets its own plot/FFT with its
    # own scale, is excluded from the main views and forced to PGA x1.
    reference_channel: str = ""
    reference_fft_scale_uv: float = 1000.0
    # Which of the three lower-right views is in front (reference / accelerometer /
    # noise) and how often the noise table is refreshed.
    bottom_tab: str = "reference"
    noise_interval_ms: int = 500
    fft_freq_smoothing_bins: int = 1
    notch_50: bool = False
    notch_100: bool = False
    notch_q: float = 30.0
    high_pass_hz: float = DEFAULT_HIGH_PASS_HZ
    low_pass_hz: float = DEFAULT_LOW_PASS_HZ
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
    baud_rate: int = 3_000_000
    session_name: str = "session"
    output_dir: str = "recordings"
    auto_connect: bool = True
    measurement: dict[str, object] = field(default_factory=dict)
    # Everything the automated function-generator sweep remembers (resource,
    # amplitude, drive mode, frequency list, dwell rules, monitor channel).
    sweep: dict[str, object] = field(default_factory=dict)
    # Link capacity assumptions. The Bluetooth budget starts at the conservative
    # built-in default and only changes when the user adopts a measured value.
    link: dict[str, object] = field(default_factory=dict)
    version: int = SETTINGS_VERSION

    def as_dict(self) -> dict[str, object]:
        return {
            "version": SETTINGS_VERSION,
            "hardware": self.hardware.as_dict(),
            "view": self.view.as_dict(),
            "connection": {
                "last_port": self.last_port,
                "baud_rate": int(self.baud_rate),
                "auto_connect": bool(self.auto_connect),
            },
            "recording": {
                "session_name": self.session_name,
                "output_dir": self.output_dir,
                "measurement": dict(self.measurement),
            },
            "sweep": dict(self.sweep),
            "link": dict(self.link),
        }
    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "AppSettings":
        connection = payload.get("connection") if isinstance(payload.get("connection"), dict) else {}
        recording = payload.get("recording") if isinstance(payload.get("recording"), dict) else {}
        hardware_payload = payload.get("hardware") if isinstance(payload.get("hardware"), dict) else {}
        view_payload = payload.get("view") if isinstance(payload.get("view"), dict) else {}
        try:
            version = int(payload.get("version", 1) or 1)
        except (TypeError, ValueError):
            version = 1
        settings = cls(
            hardware=HardwareSettings.from_dict(hardware_payload),
            view=ViewSettings.from_dict(view_payload),
            last_port=str(connection.get("last_port", "")),
            baud_rate=int(connection.get("baud_rate", 3_000_000)),
            session_name=str(recording.get("session_name", "session")),
            output_dir=str(recording.get("output_dir", "recordings")),
            auto_connect=bool(connection.get("auto_connect", True)),
            measurement=dict(recording.get("measurement") or {}) if isinstance(recording.get("measurement"), dict) else {},
            sweep=dict(payload.get("sweep") or {}) if isinstance(payload.get("sweep"), dict) else {},
            link=dict(payload.get("link") or {}) if isinstance(payload.get("link"), dict) else {},
            version=version,
        )
        settings.migrate()
        return settings

    def migrate(self) -> None:
        """Adopt newer defaults once, without touching later user choices."""

        if self.version < 2:
            # Version 2 turned the display band on by default (0.1 Hz / 150 Hz).
            self.view.high_pass_hz = DEFAULT_HIGH_PASS_HZ
            self.view.low_pass_hz = DEFAULT_LOW_PASS_HZ
        if self.version < 4:
            # Version 4 changed the boot defaults: 921600 baud, a logarithmic
            # FFT axis on auto scale, recording only the ticked channels, and
            # 500 SPS (250 Hz per channel) at gain x24 with a normal input.
            self.baud_rate = 3_000_000
            self.view.fft_log_scale = True
            self.view.fft_scale_mode = 0.0
            self.view.record_visible_only = True
            self.hardware.sample_rate_code = 5
            for channel in self.hardware.channels:
                channel.active = True
                channel.gain_ordinal = 6
                channel.input_ordinal = 0
        if self.version < 5:
            # Version 5 added per-converter reference modes, a selectable SPI
            # clock and MCU-side block averaging. Existing files keep working at
            # 2 MHz / no averaging / the same reference on both chips.
            self.hardware.chip_reference_modes = [
                self.hardware.normalised_reference_mode(),
                self.hardware.normalised_reference_mode(),
            ]
            self.hardware.spi_clock_hz = ADS_SPI_CLOCK_HZ
            self.hardware.decimation = 1
        if self.version < 6:
            # Version 6 moved the USB link to 3 Mbaud (the firmware now boots at
            # that rate) and changed the sample-rate figure to mean the ADS1299
            # output data rate itself, not the per-channel rate after the
            # Cyton+Daisy interleave. Both are adopted so an existing settings file
            # cannot leave the app talking to a board at the wrong baud.
            self.baud_rate = 3_000_000
        self.hardware.sync_reference()
        self.version = SETTINGS_VERSION


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
        # Rows still to stamp with Marker=1. Set by mark(), consumed by the writer
        # thread, so a marker is the same instant in this file and on the card.
        self._mark_rows = 0
        self._mark_lock = threading.Lock()

    def mark(self, rows: int = 1) -> None:
        """Stamp the next ``rows`` data rows with Marker=1.

        The window sends the board's marker command at the same moment, so the two
        files end up holding one shared instant. Without it the comparison tool can
        only guess the offset by correlating the traces, which is ambiguous for any
        periodic signal.
        """

        with self._mark_lock:
            self._mark_rows = max(int(rows), 0)

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
                    with self._mark_lock:
                        marker = 1 if self._mark_rows > 0 else 0
                        if self._mark_rows > 0:
                            self._mark_rows -= 1
                    buffer.append(
                        f"{self._rows_written},{values},{accel_values},{marker},{times[row]:.6f}\n"
                    )
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
        self._settings = HardwareSettings(board_channels=self.board_channels, sample_rate_code=self.sample_rate_code)
        self._rng = np.random.default_rng(20261022)
        self._phases = np.linspace(0.0, np.pi, max(self.board_channels, 8), endpoint=False)
        self._counter = 0
        self._moving_until = 0.0
        self._streaming = False
        self._start_time = time.monotonic()
        self._stream_start = self._start_time
        self._stream_base = 0
        self.bt_buffer_bytes = 24576
        self.bluetooth_mode = "bt"
        self.bluetooth_client = True
        self.bluetooth_tx_dbm = 4
        self.bluetooth_sent = 0
        self.bluetooth_dropped = 0
        self.bluetooth_opens = 1
        self.bluetooth_closes = 0
        self.bluetooth_rejected = 0
        self.bluetooth_write_failures = 0

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
                        self._settings.sample_rate_code = self.sample_rate_code
                        rate = SAMPLE_RATE_BY_CODE.get(self.sample_rate_code, 250)
                        self._emit_text(f"Success: Sample rate is {rate}Hz")
                        index += 2
                        continue
                self._emit_text("Failure: sample rate out of bounds")
            elif char == "?":
                self._emit_text("Board ADS Registers\nCH1SET=0x61 CH2SET=0x61\n$$$", terminator="")
            elif char == "Q":
                end = data.find(b"Q", index + 1)
                if end < 0:
                    break
                digits = data[index + 1 : end].decode("ascii", errors="replace")
                if digits.isdigit() and 2 <= int(digits) <= 64:
                    self.bt_buffer_bytes = int(digits) * 1024
                    self._emit_text(f"Success: bluetooth buffer set to {int(digits)} kB")
                else:
                    self._emit_text("Failure: buffer size out of range (2..64 kB)")
                index = end + 1
                continue
            elif char == "T":
                self._emit_text(self.bluetooth_report_text())
            elif char == "B":
                self._emit_text(self.bluetooth_log_text(), terminator="")
            elif char == "P":
                self._emit_configuration_report()
            elif char == "N":
                if index + 1 < len(data):
                    digit = chr(data[index + 1])
                    if digit.isdigit() and int(digit) <= 2:
                        self._settings.set_reference_mode(
                            (
                                REFERENCE_MODE_DIFFERENTIAL,
                                REFERENCE_MODE_SRB1,
                                REFERENCE_MODE_SRB2,
                            )[int(digit)]
                        )
                        self._emit_text(
                            f"Success: Reference mode {self._settings.reference_mode}"
                        )
                        index += 2
                        continue
                self._emit_text("Failure: invalid reference mode (0=diff, 1=SRB1, 2=SRB2)")
            index += 1

    def bluetooth_health_text(self) -> str:
        """The ``#BT`` line the firmware emits every 500 ms."""

        used = min(self.bt_buffer_bytes // 8, self.bt_buffer_bytes)
        return (
            f"#BT mode={self.bluetooth_mode} client={1 if self.bluetooth_client else 0} "
            f"boots=1 up=42s opens={self.bluetooth_opens} closes={self.bluetooth_closes} "
            f"remote=0 rej={self.bluetooth_rejected} fail={self.bluetooth_write_failures} "
            f"heap=210000 name=SZD_EEG txdbm={self.bluetooth_tx_dbm} buf={used} "
            f"bufsize={self.bt_buffer_bytes} sent={self.bluetooth_sent} "
            f"dropped={self.bluetooth_dropped} crc=0x1A2B crc_n=32\n"
        )

    def bluetooth_report_text(self) -> str:
        return (
            "Success: BT name=SZD_EEG client=1 link=bt boots=1 uptime=42s "
            f"opens=1 closes=0 remote=0 rejected=0 writefail=0 heap=210000 "
            f"buffer=0/{self.bt_buffer_bytes} sent={self.bluetooth_sent} "
            f"dropped={self.bluetooth_dropped} crc=0x1A2B"
        )

    def acquisition_health_text(self) -> str:
        """The ``#DX`` line the firmware emits once a second while streaming."""

        rate = self._packet_rate_hz()
        return (
            f"#DX ch_rate={rate / 2.0:.1f} pkt_rate={rate:.1f} drdy_s={rate * 2:.1f} "
            f"pkts={self._counter} miss=0 late=0 gap_max_us=2000 corrupt=0 spi_us=110 spi_max_us=118 "
            f"usb_free=3900 bt_buf=0 core=1 prio=22\n"
        )

    def bluetooth_log_text(self) -> str:
        events = ["boot=1 reset=1(power-on)", "ready txdbm=4 power=ok heap=210000", "spp server started"]
        events.append("open handle=1 status=0")
        return "\n".join(f"#BTLOG {event}" for event in events) + "\n"

    def _emit_configuration_report(self) -> None:
        """Answer the firmware's ``P`` command with the same ``[CFG]`` lines."""

        lines = ["[CFG] BEGIN"]
        for expectation in expected_device_registers(self._settings):
            regs = [0x00] * 0x18
            regs[0x00] = 0x3E
            regs[0x01] = expectation.config1
            regs[0x02] = expectation.config2
            regs[0x03] = expectation.config3
            for local, value in enumerate(expectation.channel_regs):
                regs[0x05 + local] = value
            for local in range(len(expectation.channel_regs), 8):
                regs[0x05 + local] = 0x81
            regs[0x0D] = expectation.bias_sensp
            regs[0x0E] = expectation.bias_sensn
            regs[0x15] = expectation.misc1
            payload = "".join(f"{value:02X}" for value in regs)
            lines.append(f"[CFG] DEV={expectation.name} LINK=1 REGS={payload}")
        rate = SAMPLE_RATE_BY_CODE.get(self.sample_rate_code, 250)
        lines.append(
            f"[CFG] CODE={self.sample_rate_code} SPS={rate} SPICLK={ADS_SPI_CLOCK_HZ} "
            f"SPILIMIT={max_sample_rate_from_spi(ADS_SPI_CLOCK_HZ, 2)} "
            f"SERIALLIMIT={max_sustained_packet_rate(self.baudrate)} DONE=1"
        )
        self._emit_text("\n".join(lines) + "\n", terminator="")

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
        if logical < len(self._settings.channels):
            self._settings.channels[logical] = ChannelSetting(
                active=digits[0] == "0",
                gain_ordinal=gain_ordinal,
                input_ordinal=int(digits[2]) if digits[2].isdigit() else 0,
                bias=digits[3] == "1",
                srb2=digits[4] == "1",
                srb1=digits[5] == "1",
            )
            # The reference mode owns SRB1/SRB2, exactly like the firmware.
            self._settings.sync_reference()
        self._emit_text(f"Success: Channel set for {logical + 1}")

    def _version_text(self) -> str:
        lines = ["OpenBCI V3 8-16 channel", "On Board ADS1299 Device ID: 0x3D"]
        if self.board_channels > 8:
            lines.append("On Daisy ADS1299 Device ID: 0x3D")
        lines.extend(
            [
                "LIS3DH Device ID: 0x33",
                "SD card: present",
                "Firmware: v3.1.5",
                f"Serial link limit: {max_sustained_packet_rate(self.baudrate)} SPS",
                f"SPI clock: {ADS_SPI_CLOCK_HZ // 1000} kHz, frame "
                f"{spi_frame_microseconds(ADS_SPI_CLOCK_HZ, 2):.0f} us x 2 devices",
                f"SPI link limit: {max_sample_rate_from_spi(ADS_SPI_CLOCK_HZ, 2)} SPS",
                "$$$",
            ]
        )
        return "\n".join(lines) + "\n"

    def _packet_rate_hz(self) -> float:
        """Conversions per second on the wire (one packet each).

        In 16-channel daisy mode the two chips take turns, so this is twice the
        per-channel rate; with a single chip it is the per-channel rate.
        """

        return float(SAMPLE_RATE_BY_CODE.get(self.sample_rate_code, 250))

    def _run(self) -> None:
        period = 1.0 / self._packet_rate_hz()
        self._last_health = time.monotonic()
        self._last_dx = time.monotonic()
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
                    self.bluetooth_sent += emitted
                    with self._lock:
                        self._rx.extend(packets)
                if now - self._last_health >= 0.5:
                    self._last_health = now
                    self._emit_text(self.bluetooth_health_text(), terminator="")
                if now - self._last_dx >= 1.0:
                    self._last_dx = now
                    self._emit_text(self.acquisition_health_text(), terminator="")
            time.sleep(0.007)

    def _build_packet(self, elapsed: float) -> bytes:
        index = self._counter & 0xFF
        self._counter += 1
        t = self._counter / self._packet_rate_hz()
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