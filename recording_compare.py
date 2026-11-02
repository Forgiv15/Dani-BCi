"""Compare two recordings of the same measurement.

The board can record to its SD card while the live window records the same
stream over the cable, and the two files will never be identical: the card writer
starts a little before or after the host, USB or radio packets can be dropped, and
a card that cannot keep up skips samples. This module answers the three questions
that matter afterwards:

* when did each recording start, relative to the other (start offset),
* do the two timelines cover the same span (end offset / coverage), and
* is anything missing or corrupted inside that span (gaps, duplicate timestamps,
  non-monotonic timestamps).

It works from the recorded timestamps and the sample values, so it needs nothing
from the firmware beyond the OpenBCI-compatible file both sides already write.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from data_loader import load_openbci_csv, openbci_sample_rate, read_openbci_metadata

# A gap larger than this multiple of the nominal sample period counts as missing
# data rather than jitter. USB arrival jitter is a fraction of a period; a
# dropped packet is at least one whole one.
GAP_PERIOD_TOLERANCE = 1.6

# Below this correlation the two traces are not the same signal, so a time offset
# derived from them would be meaningless.
MIN_ALIGNMENT_CORRELATION = 0.3


@dataclass(slots=True)
class RecordingSummary:
    """What one file contains, independent of the other."""

    path: Path
    name: str
    sample_rate: float
    channels: int
    rows: int
    duration_s: float
    start_timestamp: Optional[float]
    end_timestamp: Optional[float]
    first_sample_index: Optional[int]
    timestamps_ok: bool
    duplicate_timestamps: int
    backwards_timestamps: int
    gaps: list[tuple[float, float]] = field(default_factory=list)
    missing_samples: int = 0
    notes: list[str] = field(default_factory=list)

    @property
    def coverage_s(self) -> float:
        return self.duration_s

    def describe(self) -> list[str]:
        lines = [
            f"{self.path.name}",
            f"  session            {self.name or '-'}",
            f"  sample rate        {self.sample_rate:g} Hz",
            f"  channels           {self.channels}",
            f"  rows               {self.rows:,}",
            f"  duration           {self.duration_s:.3f} s",
        ]
        if self.start_timestamp is not None and self.end_timestamp is not None:
            lines.append(
                f"  file timestamps    {self.start_timestamp:.6f} .. {self.end_timestamp:.6f} s"
            )
        if not self.timestamps_ok:
            lines.append("  timestamps         missing - coverage compared by row count only")
        if self.duplicate_timestamps:
            lines.append(f"  duplicate times    {self.duplicate_timestamps:,}")
        if self.backwards_timestamps:
            lines.append(f"  backwards times    {self.backwards_timestamps:,} (time jumped back)")
        if self.gaps:
            total = sum(end - start for start, end in self.gaps)
            lines.append(
                f"  gaps               {len(self.gaps):,} gap(s), {total:.3f} s missing "
                f"({self.missing_samples:,} samples)"
            )
            for start, end in self.gaps[:5]:
                lines.append(f"    at {start:.3f} s for {end - start:.3f} s")
            if len(self.gaps) > 5:
                lines.append(f"    ... {len(self.gaps) - 5} more")
        else:
            lines.append("  gaps               none")
        lines.extend(f"  note               {note}" for note in self.notes)
        return lines


@dataclass(slots=True)
class ComparisonResult:
    a: RecordingSummary
    b: RecordingSummary
    start_offset_s: Optional[float] = None
    end_offset_s: Optional[float] = None
    alignment_correlation: Optional[float] = None
    alignment_channel: str = ""
    verdict: str = ""
    problems: list[str] = field(default_factory=list)
    # Facts worth showing that are not faults, e.g. "the two files use different
    # clocks". Kept apart from ``problems`` so the verdict stays honest.
    notes: list[str] = field(default_factory=list)

    def report(self) -> str:
        lines = ["Recording A (reference)", *["  " + text for text in self.a.describe()]]
        lines.append("")
        lines.append("Recording B (compared)")
        lines.extend("  " + text for text in self.b.describe())
        lines.append("")
        lines.append("Comparison")
        if self.start_offset_s is not None:
            direction = "B starts later" if self.start_offset_s >= 0 else "B starts earlier"
            lines.append(
                f"  start offset       {abs(self.start_offset_s):.4f} s ({direction}), "
                "measured from where the waveforms line up"
            )
        else:
            lines.append("  start offset       could not be measured")
        if self.end_offset_s is not None:
            direction = "B ends later" if self.end_offset_s >= 0 else "B ends earlier"
            lines.append(f"  end offset         {abs(self.end_offset_s):.4f} s ({direction})")
        if self.alignment_correlation is not None:
            lines.append(
                f"  trace correlation  {self.alignment_correlation:.3f} on "
                f"{self.alignment_channel or 'a shared channel'}"
            )
        for note in self.notes:
            lines.append(f"  note               {note}")
        if self.problems:
            lines.append("  findings:")
            lines.extend(f"    - {problem}" for problem in self.problems)
        else:
            lines.append("  findings: the two recordings agree")
        lines.append("")
        lines.append(f"Verdict: {self.verdict}")
        return "\n".join(lines)


def _channel_names(path: Path) -> list[str]:
    import csv

    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for row in csv.reader(handle):
            if not row:
                continue
            first = row[0].strip()
            if first.startswith("%"):
                continue
            return [cell.strip() for cell in row]
    return []


def _signal_columns(names: Sequence[str]) -> list[str]:
    """Columns that carry a signal, i.e. everything but Marker and Timestamp."""

    return [
        name
        for name in names
        if name and name.lower() not in ("marker", "timestamp")
    ]


def _load_values(path: Path, sample_rate: float) -> np.ndarray:
    """Load every signal column of a recording as a 2-D array."""

    names = _channel_names(path)
    columns = _signal_columns(names)
    if not columns:
        raise ValueError("no signal columns found")
    lower = [name.lower() for name in names]
    timestamp_column = names[lower.index("timestamp")] if "timestamp" in lower else None
    dataset = load_openbci_csv(
        path,
        sample_rate=sample_rate or 250.0,
        channel_columns=columns,
        timestamp_column=timestamp_column,
    )
    return np.asarray(dataset.data, dtype=np.float64)


def summarise_recording(path: Path | str) -> RecordingSummary:
    """Read one file and describe its timeline and integrity."""

    path = Path(path)
    metadata = read_openbci_metadata(path)
    sample_rate = float(openbci_sample_rate(metadata) or 0.0)
    values = _load_values(path, sample_rate)
    rows = int(values.shape[0])
    names = _channel_names(path)
    exg_columns = [name for name in names if name.lower().startswith("exg")]
    channels = len(exg_columns) or int(values.shape[1])

    # The timestamp column is the last one in an OpenBCI file; when it is absent
    # or constant nothing about timing can be said, which is reported rather than
    # guessed.
    timestamps: Optional[np.ndarray] = None
    index_column: Optional[np.ndarray] = None
    try:
        import csv

        raw_times: list[float] = []
        raw_index: list[float] = []
        with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.reader(handle)
            header: list[str] = []
            for row in reader:
                if not row or row[0].strip().startswith("%"):
                    continue
                header = [cell.strip() for cell in row]
                break
            lower = [name.lower() for name in header]
            header_time_index = lower.index("timestamp") if "timestamp" in lower else -1
            for row in reader:
                if not row or len(row) < 2:
                    continue
                # OpenBCI files prefix every data row with the sample counter while
                # the header row does not, so the data row is one field longer.
                offset = 1 if len(row) == len(header) + 1 else 0
                if offset and row[0].strip():
                    try:
                        raw_index.append(float(row[0]))
                    except ValueError:
                        pass
                time_index = header_time_index + offset if header_time_index >= 0 else -1
                if 0 <= time_index < len(row):
                    try:
                        raw_times.append(float(row[time_index]))
                    except ValueError:
                        pass
        if raw_times and len(raw_times) >= rows // 2:
            timestamps = np.asarray(raw_times[:rows], dtype=np.float64)
        if raw_index:
            index_column = np.asarray(raw_index[:rows], dtype=np.float64)
    except OSError:
        timestamps = None

    summary = RecordingSummary(
        path=path,
        name=str(metadata.get("Session", "") or ""),
        sample_rate=sample_rate or 250.0,
        channels=channels,
        rows=rows,
        duration_s=rows / (sample_rate or 250.0),
        start_timestamp=None,
        end_timestamp=None,
        first_sample_index=int(index_column[0]) if index_column is not None and index_column.size else None,
        timestamps_ok=timestamps is not None,
        duplicate_timestamps=0,
        backwards_timestamps=0,
    )
    if timestamps is None:
        return summary

    period = 1.0 / (sample_rate or 250.0)
    summary.start_timestamp = float(timestamps[0])
    summary.end_timestamp = float(timestamps[-1])
    deltas = np.diff(timestamps)
    summary.duplicate_timestamps = int(np.count_nonzero(deltas == 0.0))
    summary.backwards_timestamps = int(np.count_nonzero(deltas < 0.0))

    # Gaps are measured against the nominal period, not against the median, so a
    # recording that dropped half its packets does not hide the loss.
    gap_indices = np.nonzero(deltas > period * GAP_PERIOD_TOLERANCE)[0]
    for index in gap_indices:
        start = float(timestamps[index])
        end = float(timestamps[index + 1])
        summary.gaps.append((start, end))
        summary.missing_samples += max(int(round((end - start) / period)) - 1, 0)
    return summary


def _marker_rows(path: Path, limit: int) -> list[int]:
    """Row indices whose Marker cell is non-zero, in file order.

    Both writers stamp the same board-side instant, so the first marker in each
    file names the same moment and gives an unambiguous offset - something
    correlation cannot do for a periodic trace.
    """

    import csv

    found: list[int] = []
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.reader(handle)
            header: list[str] = []
            for row in reader:
                if not row or row[0].strip().startswith("%"):
                    continue
                header = [cell.strip() for cell in row]
                break
            lower = [name.lower() for name in header]
            if "marker" not in lower:
                return []
            marker_index = lower.index("marker")
            for position, row in enumerate(reader):
                if position >= limit:
                    break
                if not row or len(row) < 2:
                    continue
                # Data rows carry the sample counter as an extra leading field.
                offset = 1 if len(row) == len(header) + 1 else 0
                index = marker_index + offset
                if 0 <= index < len(row):
                    try:
                        if float(row[index]) != 0.0:
                            found.append(position)
                    except ValueError:
                        continue
    except OSError:
        return []
    return found


def _best_lag(
    a: np.ndarray,
    b: np.ndarray,
    period: float,
    max_lag_s: float = 5.0,
    around: Optional[int] = None,
    span: Optional[int] = None,
) -> tuple[int, float]:
    """Offset, in samples, that best aligns B onto A.

    Positive means B holds that many samples of data from before A's first row,
    i.e. ``b[lag]`` corresponds to ``a[0]`` and B started earlier.

    Uses FFT cross-correlation over a decimated copy first, then refines over a
    narrow window: a direct loop over +/-1250 lags on a long recording would take
    seconds. When ``around`` is given (a marker-derived anchor) the search is
    restricted to ``span`` samples either side of it.
    """

    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a = a - np.mean(a)
    b = b - np.mean(b)
    if a.size < 32 or b.size < 32:
        return 0, 0.0

    # Decimate so the coarse search covers the whole +/-max_lag_s window cheaply.
    step = max(int(round(1.0 / max(period, 1e-9) / 200.0)), 1)  # work at ~200 Hz
    a_small = a[::step]
    b_small = b[::step]
    a_small = a_small - np.mean(a_small)
    b_small = b_small - np.mean(b_small)
    size = 1
    while size < a_small.size + b_small.size:
        size <<= 1
    spectrum = np.fft.rfft(b_small, size) * np.conj(np.fft.rfft(a_small, size))
    correlation = np.fft.irfft(spectrum, size)
    coarse_limit = int(min(max_lag_s / max(period * step, 1e-9), min(a_small.size, b_small.size) // 2))
    if coarse_limit <= 0:
        return 0, 0.0
    # window[i] is the correlation at lag (i - coarse_limit) * step.
    window = np.concatenate((correlation[-coarse_limit:], correlation[: coarse_limit + 1]))
    if around is not None:
        # A known anchor (the shared recording marker) narrows the search to a
        # span around it, so a periodic trace cannot pull the answer a period away.
        centre = int(round(around / max(step, 1)))
        half = max((span or 0) // max(step, 1), 2)
        low = max(centre - half, -coarse_limit)
        high = min(centre + half, coarse_limit)
        if low >= high:
            return int(around), -2.0
        # low/high are decimated lags; window is indexed from -coarse_limit, so the
        # slice offset has to be added back before subtracting coarse_limit again.
        sub = window[low + coarse_limit : high + coarse_limit + 1]
        coarse_index = int(np.argmax(sub)) + low + coarse_limit
    else:
        coarse_index = int(np.argmax(window))
    coarse_lag = coarse_index - coarse_limit
    best_lag = coarse_lag * step

    # Refine at full resolution around the coarse estimate.
    best = -2.0
    refined = best_lag
    reach = 2 * step if around is None else max(2 * step, span or 0)
    for lag in range(best_lag - reach, best_lag + reach + 1):
        if lag >= 0:
            x = a[: a.size - lag]
            y = b[lag:]
        else:
            x = a[-lag:]
            y = b[: b.size + lag]
        n = min(x.size, y.size)
        if n < 32:
            continue
        x = x[:n]
        y = y[:n]
        denominator = float(np.linalg.norm(x) * np.linalg.norm(y))
        if denominator <= 0.0:
            continue
        value = float(np.dot(x, y) / denominator)
        if value > best:
            best = value
            refined = lag
    return refined, best


def compare_recordings(
    path_a: Path | str,
    path_b: Path | str,
) -> ComparisonResult:
    """Compare two recordings of the same measurement."""

    summary_a = summarise_recording(path_a)
    summary_b = summarise_recording(path_b)
    result = ComparisonResult(a=summary_a, b=summary_b)

    if summary_a.sample_rate != summary_b.sample_rate:
        ratio = summary_b.sample_rate / max(summary_a.sample_rate, 1e-9)
        integer_ratio = round(ratio) if ratio >= 1.0 else round(1.0 / ratio)
        if ratio >= 1.0 and abs(ratio - integer_ratio) < 0.01 and integer_ratio >= 1:
            summary_a.notes.append(
                f"sample rates differ by x{ratio:g}: B keeps every conversion while A is the "
                "decimated link rate, so B is compared after averaging by that factor"
            )
        elif ratio < 1.0 and abs(1.0 / ratio - integer_ratio) < 0.01 and integer_ratio >= 1:
            summary_a.notes.append(
                f"sample rates differ by 1/{integer_ratio:g}: A keeps every conversion while B is "
                "the decimated link rate, so A is compared after averaging by that factor"
            )
        else:
            result.problems.append(
                f"sample rates differ ({summary_a.sample_rate:g} vs {summary_b.sample_rate:g} Hz); "
                "compare recordings taken with the same data rate"
            )

    # --- the two files do not share a clock --------------------------------
    #
    # The live file's Timestamp column is seconds since the host opened the file.
    # The card file's is the board's micros() since boot (69.7 s in a real
    # recording). Subtracting one from the other produced an offset that looked
    # like 69.773 s and made every comparison read as a failure, so the raw
    # timestamps are only ever used *within* one file. The offset between the two
    # files comes from where the waveforms actually line up.
    clocks_differ = (
        summary_a.start_timestamp is not None
        and summary_b.start_timestamp is not None
        and abs(float(summary_a.start_timestamp) - float(summary_b.start_timestamp)) > 1.0
    )
    if clocks_differ:
        result.notes.append(
            "the two files use different clocks (A starts at "
            f"{summary_a.start_timestamp:.3f} s, B at {summary_b.start_timestamp:.3f} s), so their "
            "timestamps are compared only within each file"
        )

    # --- waveform alignment -------------------------------------------------
    try:
        values_a = _load_values(Path(path_a), summary_a.sample_rate)
        values_b = _load_values(Path(path_b), summary_b.sample_rate)
        shared = min(values_a.shape[1], values_b.shape[1])
        if shared:
            channel = 0
            period = 1.0 / (summary_a.sample_rate or 250.0)
            result.alignment_channel = "column 1"
            # A gap in the middle shifts everything after it, so a single lag can
            # only be measured over the part before the first gap. Correlating
            # across a gap would report a poor match for a file that is perfect
            # apart from the gap itself.
            a_trace = values_a[:, channel]
            b_trace = values_b[:, channel]
            # Bring both traces onto A's time base. The SD file can legitimately
            # run at twice the live rate, because the firmware sends the two
            # converters in alternating packets while the card stores every
            # conversion.
            rate_a = summary_a.sample_rate or 250.0
            rate_b = summary_b.sample_rate or 250.0
            if rate_b > rate_a * 1.01:
                factor = int(round(rate_b / rate_a))
                if factor >= 2:
                    b_trace = b_trace[: (b_trace.size // factor) * factor].reshape(-1, factor).mean(axis=1)
                    period = 1.0 / rate_a
            elif rate_a > rate_b * 1.01:
                factor = int(round(rate_a / rate_b))
                if factor >= 2:
                    a_trace = a_trace[: (a_trace.size // factor) * factor].reshape(-1, factor).mean(axis=1)
                    period = 1.0 / rate_b
            # Trim to the part before B's first gap: a gap shifts everything after
            # it, and correlating across one would report a poor match for a file
            # that is perfect apart from the gap itself.
            if summary_b.gaps and summary_b.start_timestamp is not None:
                first_gap = summary_b.gaps[0][0]
                limit = int((first_gap - summary_b.start_timestamp) * (summary_b.sample_rate or 250.0))
                if limit > 64:
                    b_trace = b_trace[:limit]

            # A marker is the only unambiguous anchor: a shared instant that both
            # writers stamped. Correlation alone cannot resolve a periodic trace,
            # and a test tone or mains pickup is exactly that.
            markers_a = _marker_rows(Path(path_a), summary_a.rows)
            markers_b = _marker_rows(Path(path_b), summary_b.rows)
            expected_shift: Optional[int] = None
            if markers_a and markers_b:
                marker_a_s = markers_a[0] / (summary_a.sample_rate or 250.0)
                marker_b_s = markers_b[0] / (summary_b.sample_rate or 250.0)
                # A later marker position inside its own file means that file
                # started earlier, which is a positive lag.
                expected_shift = int(round((marker_b_s - marker_a_s) / period))
                result.notes.append(
                    "aligned on the shared recording marker: A at "
                    f"{marker_a_s:.3f} s, B at {marker_b_s:.3f} s"
                )
            elif markers_a or markers_b:
                result.notes.append(
                    "only one file carries a recording marker, so the offset comes from the waveforms"
                )

            if expected_shift is not None:
                # Search only around the marker so a periodic trace cannot pull the
                # answer a whole period away.
                window = max(int(round(0.2 / period)), 8)
                lag, correlation = _best_lag(
                    a_trace,
                    b_trace,
                    period,
                    max_lag_s=max(abs(expected_shift) * period + 0.5, 1.0),
                    around=expected_shift,
                    span=window,
                )
                if correlation < MIN_ALIGNMENT_CORRELATION:
                    result.alignment_correlation = correlation
                    result.problems.append(
                        f"the marker says the files are {abs(expected_shift) * period:.3f} s apart but "
                        f"the traces do not correlate there ({correlation:.2f})"
                    )
                else:
                    result.alignment_correlation = correlation
                    # `lag` positive means B holds that many samples of data from
                    # before A's first row, i.e. B started earlier.
                    result.start_offset_s = float(-lag * period)
                    result.end_offset_s = result.start_offset_s + (
                        summary_b.duration_s - summary_a.duration_s
                    )
            else:
                lag, correlation = _best_lag(a_trace, b_trace, period)
                result.alignment_correlation = correlation
                if correlation >= MIN_ALIGNMENT_CORRELATION:
                    # `lag` positive means B holds that many samples of data from
                    # before A's first row, i.e. B started earlier.
                    result.start_offset_s = float(-lag * period)
                    result.end_offset_s = result.start_offset_s + (
                        summary_b.duration_s - summary_a.duration_s
                    )
                else:
                    result.problems.append(
                        f"the traces do not correlate ({correlation:.2f}); they may not be the same "
                        "measurement, or one of them is noise"
                    )
    except (OSError, ValueError) as exc:
        result.problems.append(f"could not align the waveforms: {exc}")

    # --- integrity ----------------------------------------------------------
    for summary, label in ((summary_a, "A"), (summary_b, "B")):
        if summary.duplicate_timestamps:
            result.problems.append(
                f"{label} has {summary.duplicate_timestamps:,} duplicate timestamp(s) - packets were "
                "repeated or the writer stalled"
            )
        if summary.backwards_timestamps:
            result.problems.append(
                f"{label} has {summary.backwards_timestamps:,} backwards timestamp step(s) - the "
                "clock jumped, so its timeline cannot be trusted"
            )
        if summary.gaps:
            result.problems.append(
                f"{label} is missing {summary.missing_samples:,} sample(s) across "
                f"{len(summary.gaps):,} gap(s), {sum(e - s for s, e in summary.gaps):.3f} s in total"
            )

    # Coverage is reported as a note, not as a fault: see the verdict below. A
    # shorter file is normal when the two writers were started at different times.


    # --- verdict ------------------------------------------------------------
    #
    # A different start time is not a fault: the two writers are started by
    # different things, so a second of offset is normal. What matters is whether
    # the overlapping part is the same signal and whether either file lost data
    # inside its own span.
    lossless = not any(
        "gap" in problem or "missing" in problem or "duplicate" in problem or "backwards" in problem
        for problem in result.problems
    )
    correlated = (
        result.alignment_correlation is not None
        and result.alignment_correlation >= MIN_ALIGNMENT_CORRELATION
    )
    if result.start_offset_s is not None:
        b_start = result.start_offset_s
        b_end = result.start_offset_s + summary_b.duration_s
        a_start = 0.0
        a_end = summary_a.duration_s
        uncovered: list[str] = []
        if b_start > a_start + 0.05:
            uncovered.append(f"{b_start - a_start:.3f} s at the start")
        if b_end < a_end - 0.05:
            uncovered.append(f"{a_end - b_end:.3f} s at the end")
        if uncovered:
            result.notes.append(
                "B does not cover the whole of A: "
                + " and ".join(uncovered)
                + " of A is missing from B"
            )
        elif b_start < a_start - 0.05 or b_end > a_end + 0.05:
            result.notes.append(
                f"B covers A completely and has {abs(min(b_start, 0.0)):.3f} s extra at the start "
                f"and {max(b_end - a_end, 0.0):.3f} s extra at the end"
            )

    if not correlated:
        result.verdict = "The two traces do not line up, so they cannot be compared sample by sample."
    elif lossless:
        result.verdict = (
            "The two recordings carry the same signal"
            + (
                f", offset by {abs(result.start_offset_s):.3f} s."
                if result.start_offset_s is not None
                else "."
            )
            + " Neither file lost samples."
        )
    else:
        result.verdict = (
            "The two recordings carry the same signal, but at least one of them lost samples "
            "inside its own span - see the findings above."
        )
    return result
