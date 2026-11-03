"""Bluetooth SPP throughput benchmark: host side of the synthetic packet stream.

The firmware generates frames in memory and pushes them through the real SPP path.
This module is the other end: it finds the frames in the byte stream coming out of
the serial port, verifies every one of them against the payload the firmware must
have produced for that sequence number, and measures what actually arrived.

Three quantities are deliberately kept apart, because conflating them is how a
throughput test ends up lying:

* **queued** - bytes the firmware put into its Bluetooth ring (reported by the
  firmware in a stats packet),
* **accepted** - bytes the Bluetooth stack took from that ring (also firmware
  side, and still not proof of delivery),
* **verified** - bytes this module received, parsed and proved byte-for-byte.

Only the last one is throughput. The other two are context: they say where the
bytes were lost when the numbers disagree.
"""

from __future__ import annotations

import json
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

BENCH_SYNC = b"\xa5\x5a\xc3\x3c"
BENCH_VERSION = 2
BENCH_TYPE_DATA = 1
BENCH_TYPE_STATS = 2
BENCH_HEADER_BYTES = 16
BENCH_TRAILER_BYTES = 4
BENCH_CRC_SPAN = BENCH_HEADER_BYTES - 4  # everything after the sync word
BENCH_STATS_BYTES = 64

_HEADER = struct.Struct("<BBIHI")  # version, type, sequence, payload length, firmware ms
PAYLOAD_CHOICES = (128, 256, 512, 990, 1024, 2048, 4096)
RATE_CHOICES = (
    (0, "Unlimited (as fast as the link takes it)"),
    (32 * 1024, "32 KiB/s"),
    (64 * 1024, "64 KiB/s"),
    (100 * 1024, "100 KiB/s"),
    (150 * 1024, "150 KiB/s"),
    (200 * 1024, "200 KiB/s"),
)
SWEEP_STEPS = (32 * 1024, 48 * 1024, 64 * 1024, 80 * 1024, 100 * 1024, 128 * 1024, 160 * 1024, 200 * 1024, 256 * 1024)

# A sweep step only counts as sustainable when the host verified nearly everything
# it should have received and the firmware queue was not climbing. A rate that
# only holds for a burst is not a sustainable rate.
SUSTAIN_MIN_VERIFIED_RATIO = 0.995
SUSTAIN_MAX_STALL_RATIO = 0.02
SUSTAIN_MAX_QUEUE_TREND = 0.25


def expected_payload(sequence: int, length: int) -> bytes:
    """Regenerate the payload the firmware produced for one packet.

    Mirrors ``benchFillPayload()`` in main.cpp: an elementwise mixing function of
    (sequence, index). Being able to recompute it is what makes integrity checkable
    without the firmware storing the data twice.

    Vectorised on purpose. The first version was a byte-at-a-time xorshift, which
    costs roughly a million interpreted iterations per second at line rate - enough to
    peg a core and freeze the GUI while a maximum-throughput test ran.
    """

    if length <= 0:
        return b""
    base = np.uint32((sequence * 2654435761) & 0xFFFFFFFF)
    index = np.arange(length, dtype=np.uint32)
    with np.errstate(over="ignore"):
        values = base + index * np.uint32(2246822519)
        values = values ^ (values >> np.uint32(15))
    return ((values >> np.uint32(13)) & np.uint32(0xFF)).astype(np.uint8).tobytes()


def build_frame(packet_type: int, sequence: int, payload: bytes, host_ms: int = 0) -> bytes:
    """Build one frame. Used by the tests and the loopback self-check."""

    body = struct.pack("<BBIHI", BENCH_VERSION, packet_type, sequence, len(payload), host_ms)
    crc = zlib.crc32(body + payload) & 0xFFFFFFFF
    return BENCH_SYNC + body + payload + struct.pack("<I", crc)


@dataclass(slots=True)
class FirmwareStats:
    """What the firmware says about its own side of the test."""

    queued_bytes: int = 0
    stack_bytes: int = 0
    queue_fill: int = 0
    peak_fill: int = 0
    stalls: int = 0
    uptime_ms: int = 0
    rate_bps: int = 0
    payload_bytes: int = 0
    frame_bytes: int = 0
    packets_queued: int = 0
    ring_bytes: int = 0
    congestion_events: int = 0
    ack_timeouts: int = 0
    spp_write_chunk: int = 0
    fast_path: int = 0
    fast_path_fallbacks: int = 0
    spp_handle: int = 0
    received_at: float = 0.0

    def absorb(self, payload: bytes, received_at: float) -> None:
        if len(payload) < BENCH_STATS_BYTES:
            return
        values = struct.unpack("<IIIIIIIHHIIIIIIII", payload[:BENCH_STATS_BYTES])
        (
            self.queued_bytes,
            self.stack_bytes,
            self.queue_fill,
            self.peak_fill,
            self.stalls,
            self.uptime_ms,
            self.rate_bps,
        ) = values[:7]
        self.payload_bytes = values[7]
        self.frame_bytes = values[8]
        self.packets_queued = values[9]
        self.ring_bytes = values[10]
        self.congestion_events = values[11]
        self.ack_timeouts = values[12]
        self.spp_write_chunk = values[13]
        self.fast_path = values[14]
        self.fast_path_fallbacks = values[15]
        self.spp_handle = values[16]
        self.received_at = received_at


@dataclass
class BenchmarkResult:
    """Everything one run produced, ready for the table or a CSV/JSON file."""

    started_at: str = ""
    duration_s: float = 0.0
    mode: str = ""
    rate_target_bps: int = 0
    payload_bytes: int = 0
    verified_bytes: int = 0
    received_bytes: int = 0
    packets: int = 0
    corrupt_packets: int = 0
    missing_packets: int = 0
    sequence_errors: int = 0
    peak_bytes_per_second: float = 0.0
    average_bytes_per_second: float = 0.0
    firmware_queued_bytes: int = 0
    firmware_stack_bytes: int = 0
    peak_queue_fill: int = 0
    ring_bytes: int = 0
    stalls: int = 0
    verdict: str = ""
    notes: list[str] = field(default_factory=list)
    samples: list[dict[str, float]] = field(default_factory=list)

    @property
    def average_kib_s(self) -> float:
        return self.average_bytes_per_second / 1024.0

    @property
    def average_kb_s(self) -> float:
        return self.average_bytes_per_second / 1000.0

    @property
    def average_mbit_s(self) -> float:
        return self.average_bytes_per_second * 8.0 / 1e6

    @property
    def peak_kib_s(self) -> float:
        return self.peak_bytes_per_second / 1024.0

    def as_dict(self) -> dict[str, object]:
        return {
            "started_at": self.started_at,
            "duration_s": round(self.duration_s, 3),
            "mode": self.mode,
            "rate_target_bps": self.rate_target_bps,
            "payload_bytes": self.payload_bytes,
            "average_bytes_per_second": round(self.average_bytes_per_second, 1),
            "average_kib_per_second": round(self.average_kib_s, 1),
            "average_kb_per_second": round(self.average_kb_s, 1),
            "average_mbit_per_second": round(self.average_mbit_s, 3),
            "peak_bytes_per_second": round(self.peak_bytes_per_second, 1),
            "peak_kib_per_second": round(self.peak_kib_s, 1),
            "verified_bytes": self.verified_bytes,
            "received_bytes": self.received_bytes,
            "packets": self.packets,
            "corrupt_packets": self.corrupt_packets,
            "missing_packets": self.missing_packets,
            "sequence_errors": self.sequence_errors,
            "firmware_queued_bytes": self.firmware_queued_bytes,
            "firmware_stack_bytes": self.firmware_stack_bytes,
            "peak_queue_fill": self.peak_queue_fill,
            "ring_bytes": self.ring_bytes,
            "stalls": self.stalls,
            "verdict": self.verdict,
            "notes": list(self.notes),
            "samples": list(self.samples),
        }

    def save_json(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(self.as_dict(), handle, indent=2)

    def save_csv(self, path: str) -> None:
        """One row per one-second sample; the run totals go in the header rows."""

        lines = [
            f"# started_at={self.started_at}",
            f"# mode={self.mode} rate_target_bps={self.rate_target_bps} payload_bytes={self.payload_bytes}",
            f"# average_kib_s={self.average_kib_s:.2f} peak_kib_s={self.peak_kib_s:.2f} "
            f"verified_bytes={self.verified_bytes} corrupt={self.corrupt_packets} "
            f"missing={self.missing_packets} verdict={self.verdict}",
            "Time (s),Bytes/s,kB/s,Mbit/s,Queue fill,Peak fill,Stalls",
        ]
        for sample in self.samples:
            lines.append(
                f"{sample.get('t', 0.0):.3f},{sample.get('bytes_per_second', 0.0):.0f},"
                f"{sample.get('kib_per_second', 0.0):.1f},{sample.get('mbit_per_second', 0.0):.3f},"
                f"{sample.get('queue_fill', 0)},{sample.get('peak_fill', 0)},{sample.get('stalls', 0)}"
            )
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")


class BenchmarkReceiver:
    """Incremental frame parser plus the statistics of one run.

    Fed raw bytes straight off the serial port, before the OpenBCI parser sees
    them. Anything that is not part of a benchmark frame is handed back, so text
    replies and ordinary packet traffic keep working during a test.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.active = False
        self.started_monotonic = 0.0
        self.started_wall = 0.0
        # Every byte handed to the parser, before any framing. If this stays at zero
        # the link is dead; if it climbs while verified stays zero, the framing is
        # wrong. Two very different faults that used to look identical.
        self.raw_bytes = 0
        self.received_bytes = 0
        self.verified_bytes = 0
        self.packets = 0
        self.corrupt_packets = 0
        self.missing_packets = 0
        self.sequence_errors = 0
        # What the sample size the run was configured with, for the result record.
        self.payload_bytes = 0
        self._next_sequence: Optional[int] = None
        self._last_sequence: Optional[int] = None
        self.firmware = FirmwareStats()
        self._window_start = 0.0
        self._window_bytes = 0
        self.current_bytes_per_second = 0.0
        self.peak_bytes_per_second = 0.0
        self._samples: list[dict[str, float]] = []
        self._last_sample_time = 0.0

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        self._buffer.clear()
        self.active = True
        self.started_monotonic = time.monotonic()
        self.started_wall = time.time()
        self.raw_bytes = 0
        self.received_bytes = 0
        self.verified_bytes = 0
        self.packets = 0
        self.corrupt_packets = 0
        self.missing_packets = 0
        self.sequence_errors = 0
        self._next_sequence = None
        self._last_sequence = None
        self.firmware = FirmwareStats()
        self._window_start = self.started_monotonic
        self._window_bytes = 0
        self.current_bytes_per_second = 0.0
        self.peak_bytes_per_second = 0.0
        self._samples = []
        self._last_sample_time = self.started_monotonic

    def stop(self) -> None:
        self.active = False

    @property
    def elapsed_s(self) -> float:
        if not self.started_monotonic:
            return 0.0
        return max(time.monotonic() - self.started_monotonic, 1e-9)

    # -- parsing -----------------------------------------------------------

    def feed(self, chunk: bytes) -> bytes:
        """Consume benchmark frames from ``chunk``; return the bytes that are not.

        The returned bytes are what the normal stream parser should see. They are
        collected in order, so an interleaved ``Success:`` line still arrives.
        """

        if not self.active or not chunk:
            return chunk
        self.raw_bytes += len(chunk)
        buffer = self._buffer
        buffer.extend(chunk)
        passthrough = bytearray()
        # One read offset with a single compaction at the end, instead of deleting
        # from the front once per frame: at a few hundred KiB/s the front deletions
        # are themselves a measurable cost.
        position = 0
        pending = len(buffer)
        while True:
            start = buffer.find(BENCH_SYNC, position)
            if start < 0:
                # No sync at all: keep a tail long enough to complete a split one.
                keep = len(BENCH_SYNC) - 1
                if pending - position > keep:
                    passthrough.extend(buffer[position : pending - keep])
                    position = pending - keep
                break
            if start > position:
                passthrough.extend(buffer[position:start])
                position = start
            if pending - position < BENCH_HEADER_BYTES:
                break
            version = buffer[position + 4]
            length = struct.unpack_from("<H", buffer, position + 10)[0]
            total = BENCH_HEADER_BYTES + length + BENCH_TRAILER_BYTES
            if version != BENCH_VERSION or length == 0 or total > 65536:
                # Not one of ours (or a truncated write): drop the sync and rescan.
                passthrough.append(buffer[position])
                position += 1
                continue
            if pending - position < total:
                break
            self._handle_frame(bytes(buffer[position : position + total]), length)
            position += total
        if position:
            del buffer[:position]
        return bytes(passthrough)

    def _handle_frame(self, frame: bytes, length: int) -> None:
        self.received_bytes += len(frame)
        self._window_bytes += len(frame)
        expected_crc = struct.unpack_from("<I", frame, BENCH_HEADER_BYTES + length)[0]
        payload = frame[BENCH_HEADER_BYTES : BENCH_HEADER_BYTES + length]
        actual_crc = zlib.crc32(frame[4 : BENCH_HEADER_BYTES + length]) & 0xFFFFFFFF
        packet_type = frame[5]
        sequence = struct.unpack_from("<I", frame, 6)[0]
        now = time.monotonic()
        if actual_crc != expected_crc:
            self.corrupt_packets += 1
            return
        if packet_type == BENCH_TYPE_STATS:
            self.firmware.absorb(payload, now)
            return
        if packet_type != BENCH_TYPE_DATA:
            return
        # Integrity: the payload is a pure function of the sequence number, so a
        # flipped byte anywhere in it is detectable even though the CRC passed.
        if payload != expected_payload(sequence, length):
            self.corrupt_packets += 1
            return
        self.verified_bytes += len(frame)
        self.packets += 1
        if self._next_sequence is not None:
            if sequence == self._next_sequence:
                pass
            elif sequence > self._next_sequence:
                self.missing_packets += int(sequence - self._next_sequence)
            else:
                self.sequence_errors += 1
        self._next_sequence = sequence + 1
        self._last_sequence = sequence

    # -- sampling ----------------------------------------------------------

    def poll(self) -> None:
        """Update the one-second figures. Call at a few Hz from the GUI."""

        if not self.active:
            return
        now = time.monotonic()
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self.current_bytes_per_second = self._window_bytes / elapsed
            if self.current_bytes_per_second > self.peak_bytes_per_second:
                self.peak_bytes_per_second = self.current_bytes_per_second
            self._window_bytes = 0
            self._window_start = now
        if now - self._last_sample_time >= 0.5:
            self._last_sample_time = now
            self._samples.append(
                {
                    "t": now - self.started_monotonic,
                    "bytes_per_second": self.current_bytes_per_second,
                    "kib_per_second": self.current_bytes_per_second / 1024.0,
                    "mbit_per_second": self.current_bytes_per_second * 8.0 / 1e6,
                    "queue_fill": float(self.firmware.queue_fill),
                    "peak_fill": float(self.firmware.peak_fill),
                    "stalls": float(self.firmware.stalls),
                }
            )
            # A long test must not grow without bound.
            del self._samples[:-3600]

    @property
    def average_bytes_per_second(self) -> float:
        return self.verified_bytes / self.elapsed_s if self.elapsed_s > 0 else 0.0

    @property
    def samples(self) -> list[dict[str, float]]:
        return self._samples

    def verdict(self) -> str:
        """Stable / congested / losing data, from the numbers rather than opinion."""

        if self.packets == 0:
            return "no data"
        loss = (self.missing_packets + self.corrupt_packets) / max(self.packets, 1)
        stalled = self.firmware.stalls
        fill = self.firmware.peak_fill
        ring = max(self.firmware.ring_bytes, 1)
        if self.corrupt_packets and loss > 0.01:
            return "corrupt"
        if loss > 0.01 or (self.sequence_errors and loss > 0.005):
            return "losing data"
        if stalled > 0 or fill > 0.9 * ring:
            # The ring is saturated. Perfectly normal at line rate, where the
            # generator deliberately outruns the radio; only suspicious if data is
            # also being lost, which the checks above already caught.
            return "saturated (all verified)" if loss == 0 else "congested"
        return "stable"

    def result(self, *, mode: str, rate_target_bps: int, payload_bytes: int) -> BenchmarkResult:
        started = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.started_wall or time.time()))
        return BenchmarkResult(
            started_at=started,
            duration_s=self.elapsed_s,
            mode=mode,
            rate_target_bps=int(rate_target_bps),
            payload_bytes=int(payload_bytes),
            verified_bytes=int(self.verified_bytes),
            received_bytes=int(self.received_bytes),
            packets=int(self.packets),
            corrupt_packets=int(self.corrupt_packets),
            missing_packets=int(self.missing_packets),
            sequence_errors=int(self.sequence_errors),
            peak_bytes_per_second=float(
                max(self.peak_bytes_per_second, self.current_bytes_per_second)
            ),
            average_bytes_per_second=float(self.average_bytes_per_second),
            firmware_queued_bytes=int(self.firmware.queued_bytes),
            firmware_stack_bytes=int(self.firmware.stack_bytes),
            peak_queue_fill=int(self.firmware.peak_fill),
            ring_bytes=int(self.firmware.ring_bytes),
            stalls=int(self.firmware.stalls),
            verdict=self.verdict(),
            samples=list(self._samples),
        )


def is_sustainable(result: BenchmarkResult) -> tuple[bool, str]:
    """Judge one run: verified almost everything, no queue growth, no stalls.

    Backpressure is judged only for a *rate-limited* run. An unlimited run fills the
    firmware ring by design - that is what "as fast as the link takes it" means - so
    counting its stalls against it would mark every maximum-throughput run as failed.
    """

    expected = result.packets + result.missing_packets + result.corrupt_packets
    if expected <= 0:
        return False, "no packets"
    ratio = result.packets / expected
    if ratio < SUSTAIN_MIN_VERIFIED_RATIO:
        return False, f"only {ratio * 100:.1f}% verified"
    if result.corrupt_packets:
        return False, f"{result.corrupt_packets} corrupt packet(s)"
    rate_limited = result.rate_target_bps > 0
    if not rate_limited:
        return True, "verified at line rate"
    if result.ring_bytes and result.peak_queue_fill > SUSTAIN_MAX_QUEUE_TREND * result.ring_bytes:
        return False, f"queue filled to {result.peak_queue_fill} of {result.ring_bytes} bytes"
    if result.packets and result.stalls > SUSTAIN_MAX_STALL_RATIO * result.packets:
        return False, f"{result.stalls} transmission stall(s)"
    return True, "verified, no queue growth"


def summarize_sweep(steps: Sequence[tuple[int, BenchmarkResult]]) -> str:
    """The highest rate that passed, plus the first one that did not."""

    best: Optional[int] = None
    failed: Optional[tuple[int, str]] = None
    for rate, result in steps:
        ok, reason = is_sustainable(result)
        if ok:
            best = rate
        elif failed is None:
            failed = (rate, reason)
    if best is None:
        return "No rate in the sweep was sustainable."
    text = f"Highest sustainable rate: {best / 1024:.0f} KiB/s ({best / 1000:.0f} kB/s, {best * 8 / 1e6:.2f} Mbit/s)."
    if failed is not None:
        text += f" Failed at {failed[0] / 1024:.0f} KiB/s ({failed[1]})."
    return text
