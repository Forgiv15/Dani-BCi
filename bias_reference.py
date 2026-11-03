"""Frequency-dependent reference-voltage curve for bias CMRR analysis.

A *bias reference curve* answers one question: at this frequency, what voltage did
the generator actually deliver to the board? It is built from one or more
recordings in which the first channel is wired straight across the generator
output - no bias amplifier in the path - so what that channel measures is the real
applied voltage, not the attenuated copy the differential inputs see.

Why a curve rather than a number: the applied amplitude is not flat with frequency.
The source impedance, the cable, the input network and the ADC's sinc3 filter all
take their share, so a nominal 1 Vrms at 5 Hz is not 1 Vrms at 200 Hz. CMRR is
``20 log10(Vcm / Vout)``, and using the nominal number puts that entire loss into
the CMRR result.

Averaging several recordings suppresses the noise floor of any single run, and the
fitted curve turns the averaged points into a continuous ``Vcm(f)``.

The *frequency* used by a CMRR measurement still comes from the CMRR recording
itself - its monitor channel or its generator plan - never from this module. Only
the voltage is looked up here. That keeps the two recordings independent: the
sweep decides where it is, this curve decides how big the stimulus was when it got
there.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline, PchipInterpolator

from data_loader import load_openbci_csv, openbci_sample_rate, read_openbci_metadata, sniff_csv_layout
from fft_analysis import detect_tone_frequency, fit_tone_rms, parse_sweep_steps

CURVE_FILE_KIND = "dani-bci.bias-reference-curve"
CURVE_FILE_VERSION = 1

FIT_KINDS: tuple[tuple[str, str], ...] = (
    ("Polynomial in log10(f)", "poly"),
    ("Exact, monotone (PCHIP)", "pchip"),
    ("Exact, min curvature (spline)", "spline"),
    ("Linear between points", "linear"),
)
FIT_LABELS = {kind: label for label, kind in FIT_KINDS}

DEFAULT_POLY_ORDER = 3
DEFAULT_FIT_KIND = "pchip"
DEFAULT_WINDOW_S = 0.5
DEFAULT_BUCKET_PERCENT = 2.0
DEFAULT_MIN_RECORDINGS = 2

# Signals below this are treated as "the generator was not driving anything" and
# dropped, rather than averaged in as a near-zero voltage that would inflate every
# CMRR value computed from it.
MIN_REFERENCE_VRMS = 1e-6
# A generator that is between steps produces a broadband transient; keep only
# readings whose tone dominates its own window.
MIN_TONE_DOMINANCE_DB = 6.0

POINT_COLUMNS = [
    "Frequency (Hz)",
    "Reference (Vrms)",
    "Spread (Vrms)",
    "Recordings",
    "Gain (V/V)",
    "Gain (dB)",
]


@dataclass(slots=True)
class ReferenceReading:
    """One measured point: the voltage this recording saw at this frequency."""

    frequency_hz: float
    voltage_v: float
    source: str


@dataclass(slots=True)
class ReferencePoint:
    """The average of every reading that landed on one frequency."""

    frequency_hz: float
    voltage_v: float
    spread_v: float
    recordings: int


@dataclass(slots=True)
class RecordingSummary:
    """What one input file contributed, for the dialog's file list."""

    path: str
    sample_rate_hz: float
    setpoint_v: float
    readings: int
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error and self.readings > 0

    def describe(self) -> str:
        name = Path(self.path).name
        if self.error:
            return f"{name}: {self.error}"
        return f"{name}: {self.readings} point(s), {self.sample_rate_hz:g} SPS"


def metadata_applied_vrms(metadata: Mapping[str, str]) -> float:
    """Applied sine amplitude in Vrms, whichever key the recording carries."""

    def number(key: str) -> float:
        try:
            return float(str(metadata.get(key, "") or "").strip())
        except (TypeError, ValueError):
            return 0.0

    vrms = number("Sweep Amplitude (Vrms)")
    if vrms > 0:
        return vrms
    vpp = number("Sweep Amplitude (Vpp)")
    return vpp / (2.0 * np.sqrt(2.0)) if vpp > 0 else 0.0


# ---------------------------------------------------------------------------
# Reading one recording
# ---------------------------------------------------------------------------


def extract_reference_readings(
    values: np.ndarray,
    sample_rate: float,
    *,
    source: str = "",
    steps: Optional[Sequence[object]] = None,
    window_duration_s: float = DEFAULT_WINDOW_S,
    fft_length: int = 4096,
    window_name: str = "Hann",
    min_tone_dominance_db: float = MIN_TONE_DOMINANCE_DB,
) -> list[ReferenceReading]:
    """Cut one reference-channel trace into (frequency, voltage) readings.

    A recording that carries a generator plan is read at the frequency of every
    step inside the time that step held, which is exact. Otherwise the dominant
    tone of each window is taken and used as its own frequency - good enough for a
    flat, continuous sweep, and the only option for a manual one.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    if signal_values.size < 16 or sample_rate <= 0:
        return []
    readings: list[ReferenceReading] = []

    if steps:
        for step in steps:
            frequency_hz = float(getattr(step, "frequency_hz", 0.0))
            if frequency_hz <= 0:
                continue
            start = int(round(float(getattr(step, "usable_start_s", 0.0)) * sample_rate))
            end = int(round(float(getattr(step, "end_s", 0.0)) * sample_rate))
            start = max(start, 0)
            end = min(end, signal_values.size)
            if end - start < 16:
                continue
            segment = signal_values[start:end]
            voltage_v = fit_tone_rms(segment, sample_rate, frequency_hz) * 1e-6
            if not np.isfinite(voltage_v) or voltage_v <= MIN_REFERENCE_VRMS:
                continue
            readings.append(ReferenceReading(frequency_hz=frequency_hz, voltage_v=voltage_v, source=source))
        return readings

    window_size = max(int(round(max(window_duration_s, 0.05) * sample_rate)), 16)
    if signal_values.size < window_size:
        return []
    hop = max(window_size // 2, 1)
    for start in range(0, signal_values.size - window_size + 1, hop):
        segment = signal_values[start : start + window_size]
        frequency_hz = detect_tone_frequency(
            segment, sample_rate, fft_length=fft_length, window_name=window_name
        )
        if frequency_hz is None or frequency_hz <= 0:
            continue
        voltage_v = fit_tone_rms(segment, sample_rate, frequency_hz) * 1e-6
        if not np.isfinite(voltage_v) or voltage_v <= MIN_REFERENCE_VRMS:
            continue
        # Reject the sliding transition between two generator steps: there the
        # "dominant tone" is a transient, and its amplitude is meaningless.
        centered = segment - np.mean(segment)
        total_rms = float(np.sqrt(np.mean(centered**2))) * 1e-6
        if total_rms > 0:
            dominance_db = 20.0 * np.log10(max(voltage_v, 1e-15) / total_rms)
            if dominance_db < min_tone_dominance_db:
                continue
        readings.append(ReferenceReading(frequency_hz=float(frequency_hz), voltage_v=voltage_v, source=source))
    return readings


def load_reference_recording(
    file_path: str | Path,
    *,
    channel_name: Optional[str] = None,
    window_duration_s: float = DEFAULT_WINDOW_S,
    fft_length: int = 4096,
    window_name: str = "Hann",
) -> tuple[RecordingSummary, list[ReferenceReading]]:
    """Load one CSV and read its reference channel across frequency.

    The reference channel defaults to the first recommended channel, which is the
    convention these recordings are made under: channel 0 is the unbuffered tap.
    """

    path = str(file_path)
    try:
        metadata = read_openbci_metadata(path)
        sample_rate = openbci_sample_rate(metadata) or 0.0
        if sample_rate <= 0:
            return RecordingSummary(path, 0.0, 0.0, 0, "no sample rate in the header"), []
        preview = sniff_csv_layout(path)
        if channel_name and channel_name in preview.numeric_columns:
            column = channel_name
        elif preview.recommended_channels:
            column = preview.recommended_channels[0]
        else:
            return RecordingSummary(path, sample_rate, 0.0, 0, "no numeric channel found"), []
        dataset = load_openbci_csv(path, sample_rate, [column], preview=preview)
    except (OSError, ValueError) as error:
        return RecordingSummary(path, 0.0, 0.0, 0, str(error)), []

    steps = parse_sweep_steps(metadata)
    readings = extract_reference_readings(
        dataset.data[:, 0],
        dataset.sample_rate,
        source=path,
        steps=steps or None,
        window_duration_s=window_duration_s,
        fft_length=fft_length,
        window_name=window_name,
    )
    summary = RecordingSummary(
        path=path,
        sample_rate_hz=float(dataset.sample_rate),
        setpoint_v=metadata_applied_vrms(metadata),
        readings=len(readings),
        error="" if readings else "no usable tone found",
    )
    return summary, readings


def available_channels(file_path: str | Path) -> list[str]:
    """Channel names offered by one CSV, for the dialog's channel combo."""

    try:
        return list(sniff_csv_layout(file_path).numeric_columns)
    except (OSError, ValueError):
        return []


# ---------------------------------------------------------------------------
# Averaging readings into points
# ---------------------------------------------------------------------------


def average_readings(
    readings: Sequence[ReferenceReading],
    *,
    bucket_percent: float = DEFAULT_BUCKET_PERCENT,
    min_recordings: int = 1,
) -> list[ReferencePoint]:
    """Group readings by frequency and average them.

    Grouping is relative rather than absolute: 2 % of 5 Hz is 0.1 Hz while 2 % of
    200 Hz is 4 Hz, which is the resolution the frequency actually has at each end
    of a sweep. Each group records how many *distinct recordings* contributed, so a
    single noisy file cannot masquerade as a well-averaged point.
    """

    usable = [r for r in readings if np.isfinite(r.frequency_hz) and np.isfinite(r.voltage_v)]
    usable = [r for r in usable if r.frequency_hz > 0 and r.voltage_v > 0]
    if not usable:
        return []

    tolerance = max(float(bucket_percent), 0.0) / 100.0
    ordered = sorted(usable, key=lambda r: r.frequency_hz)
    groups: list[list[ReferenceReading]] = []
    for reading in ordered:
        if groups:
            center = float(np.median([r.frequency_hz for r in groups[-1]]))
            if center > 0 and abs(reading.frequency_hz - center) <= tolerance * center:
                groups[-1].append(reading)
                continue
        groups.append([reading])

    points: list[ReferencePoint] = []
    for group in groups:
        voltages = np.array([r.voltage_v for r in group], dtype=np.float64)
        frequencies = np.array([r.frequency_hz for r in group], dtype=np.float64)
        sources = {r.source for r in group}
        if len(sources) < max(int(min_recordings), 1):
            continue
        points.append(
            ReferencePoint(
                frequency_hz=float(np.mean(frequencies)),
                voltage_v=float(np.mean(voltages)),
                spread_v=float(np.std(voltages, ddof=0)),
                recordings=len(sources),
            )
        )
    points.sort(key=lambda p: p.frequency_hz)
    return points


# ---------------------------------------------------------------------------
# The curve
# ---------------------------------------------------------------------------


@dataclass
class ReferenceCurve:
    """Averaged reference points plus the fit that turns them into ``Vcm(f)``."""

    points: list[ReferencePoint] = field(default_factory=list)
    fit_kind: str = DEFAULT_FIT_KIND
    poly_order: int = DEFAULT_POLY_ORDER
    setpoint_v: float = 0.0
    sample_rate_hz: float = 0.0
    sources: list[str] = field(default_factory=list)
    note: str = ""
    min_recordings: int = DEFAULT_MIN_RECORDINGS
    bucket_percent: float = DEFAULT_BUCKET_PERCENT

    # -- basic accessors ---------------------------------------------------

    def __len__(self) -> int:
        return len(self.points)

    def frequencies(self) -> np.ndarray:
        return np.array([p.frequency_hz for p in self.points], dtype=np.float64)

    def voltages(self) -> np.ndarray:
        return np.array([p.voltage_v for p in self.points], dtype=np.float64)

    def spread(self) -> np.ndarray:
        return np.array([p.spread_v for p in self.points], dtype=np.float64)

    def counts(self) -> np.ndarray:
        return np.array([p.recordings for p in self.points], dtype=np.int64)

    def coverage_hz(self) -> Optional[tuple[float, float]]:
        if not self.points:
            return None
        frequencies = self.frequencies()
        return float(frequencies[0]), float(frequencies[-1])

    def gains(self) -> np.ndarray:
        if self.setpoint_v <= 0:
            return np.full(len(self.points), np.nan)
        return self.voltages() / float(self.setpoint_v)

    def to_frame(self) -> pd.DataFrame:
        if not self.points:
            return pd.DataFrame(columns=POINT_COLUMNS)
        frame = pd.DataFrame(
            {
                "Frequency (Hz)": self.frequencies(),
                "Reference (Vrms)": self.voltages(),
                "Spread (Vrms)": self.spread(),
                "Recordings": self.counts(),
            }
        )
        gains = self.gains()
        frame["Gain (V/V)"] = gains
        with np.errstate(divide="ignore", invalid="ignore"):
            frame["Gain (dB)"] = 20.0 * np.log10(np.where(gains > 0, gains, np.nan))
        return frame[POINT_COLUMNS]

    def is_usable(self) -> bool:
        return len(self.points) >= 1 and self.fit_kind in FIT_LABELS

    # -- fitting -----------------------------------------------------------

    def _log_grid(self) -> np.ndarray:
        return np.log10(self.frequencies())

    def evaluate(self, frequencies_hz: Sequence[float] | float) -> np.ndarray:
        """Curve voltage at the given frequencies, in Vrms.

        Frequencies outside the measured range come back as ``nan``: extrapolating
        a front-end response past the points that were measured would quietly
        invent the number the whole correction depends on.
        """

        queries = np.asarray(frequencies_hz, dtype=np.float64)
        result = np.full(queries.shape, np.nan, dtype=np.float64)
        if len(self.points) == 0:
            return result
        frequencies = self.frequencies()
        log_x = np.log10(frequencies)
        # One distinct frequency cannot be interpolated through; return it flat.
        if len(self.points) == 1 or np.unique(log_x).size == 1:
            inside = np.isfinite(queries) & (queries > 0)
            result[inside] = float(self.voltages()[0])
            return result

        values = self.voltages()
        inside = np.isfinite(queries) & (queries > 0)
        log_q = np.full(queries.shape, np.nan)
        log_q[inside] = np.log10(queries[inside])
        inside &= (log_q >= log_x[0] - 1e-12) & (log_q <= log_x[-1] + 1e-12)
        if not np.any(inside):
            return result

        query = log_q[inside]
        kind = self.fit_kind
        if kind == "poly":
            order = int(min(max(self.poly_order, 1), max(len(self.points) - 1, 1)))
            if len(self.points) >= order + 1 and np.unique(log_x).size >= order + 1:
                coefficients = np.polyfit(log_x, values, order)
                result[inside] = np.polyval(coefficients, query)
            else:
                result[inside] = np.interp(query, log_x, values)
        elif kind == "linear":
            result[inside] = np.interp(query, log_x, values)
        elif kind == "pchip":
            result[inside] = PchipInterpolator(log_x, values)(query)
        elif kind == "spline":
            result[inside] = CubicSpline(log_x, values, bc_type="natural")(query)
        else:
            result[inside] = np.interp(query, log_x, values)
        # A spline can ring slightly negative near a steep edge; a voltage cannot.
        result[inside] = np.clip(result[inside], np.finfo(np.float64).tiny, None)
        return result

    def fitted(self, point_count: int = 400) -> tuple[np.ndarray, np.ndarray]:
        """The curve on a dense grid, for plotting."""

        coverage = self.coverage_hz()
        if coverage is None:
            return np.array([]), np.array([])
        low, high = coverage
        if low <= 0 or high <= low:
            return self.frequencies(), self.voltages()
        grid = np.logspace(np.log10(low), np.log10(high), max(int(point_count), 8))
        return grid, self.evaluate(grid)

    def voltage_lookup(
        self,
        *,
        scale: float = 1.0,
        target_setpoint_v: Optional[float] = None,
        default_v: float = float("nan"),
    ):
        """Return ``lookup([f, ...]) -> Vrms`` for the analysis functions.

        ``scale`` multiplies the curve directly and is the preferred way to express
        "this run was driven at a different amplitude" - the caller works out the
        ratio from generator setpoints, which is the only meaningful comparison.

        ``target_setpoint_v`` is kept for callers that only know the target: it is
        converted to the same ratio here. It must be a *setpoint*, never a measured
        voltage, or the correction cancels itself out.
        """

        factor = float(scale) if np.isfinite(scale) and scale > 0 else 1.0
        if target_setpoint_v is not None and float(target_setpoint_v) > 0 and self.setpoint_v > 0:
            factor = float(target_setpoint_v) / float(self.setpoint_v)

        def lookup(frequencies_hz: Sequence[float] | float) -> np.ndarray:
            values = np.atleast_1d(self.evaluate(np.asarray(frequencies_hz, dtype=np.float64))) * factor
            if np.isfinite(default_v):
                values = np.where(np.isfinite(values), values, float(default_v))
            return values

        return lookup

    # -- description -------------------------------------------------------

    def summary(self) -> str:
        coverage = self.coverage_hz()
        if coverage is None:
            return "no points"
        label = FIT_LABELS.get(self.fit_kind, self.fit_kind)
        if self.fit_kind == "poly" and len(self.points) > 1:
            label = f"{label} {int(min(max(self.poly_order, 1), max(len(self.points) - 1, 1)))}"
        head = (
            f"{len(self.points)} point(s) from {len(self.sources) or '?'} recording(s), "
            f"{coverage[0]:g}-{coverage[1]:g} Hz, {label}"
        )
        if self.setpoint_v > 0:
            head += f", {self.setpoint_v:g} Vrms setpoint"
        return head

    # -- persistence -------------------------------------------------------

    def to_dict(self) -> dict[str, object]:
        return {
            "kind": CURVE_FILE_KIND,
            "version": CURVE_FILE_VERSION,
            "fit_kind": self.fit_kind,
            "poly_order": int(self.poly_order),
            "setpoint_v": float(self.setpoint_v),
            "sample_rate_hz": float(self.sample_rate_hz),
            "min_recordings": int(self.min_recordings),
            "bucket_percent": float(self.bucket_percent),
            "sources": list(self.sources),
            "note": self.note,
            "points": [
                {
                    "frequency_hz": float(p.frequency_hz),
                    "voltage_v": float(p.voltage_v),
                    "spread_v": float(p.spread_v),
                    "recordings": int(p.recordings),
                }
                for p in self.points
            ],
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "ReferenceCurve":
        if str(payload.get("kind", "")) != CURVE_FILE_KIND:
            raise ValueError("that file is not a bias reference curve")
        points: list[ReferencePoint] = []
        for item in payload.get("points", []) or []:
            entry = item if isinstance(item, Mapping) else {}
            points.append(
                ReferencePoint(
                    frequency_hz=float(entry.get("frequency_hz", 0.0) or 0.0),
                    voltage_v=float(entry.get("voltage_v", 0.0) or 0.0),
                    spread_v=float(entry.get("spread_v", 0.0) or 0.0),
                    recordings=int(entry.get("recordings", 1) or 1),
                )
            )
        points = [p for p in points if p.frequency_hz > 0 and p.voltage_v > 0]
        points.sort(key=lambda p: p.frequency_hz)
        return cls(
            points=points,
            fit_kind=str(payload.get("fit_kind", DEFAULT_FIT_KIND) or DEFAULT_FIT_KIND),
            poly_order=int(payload.get("poly_order", DEFAULT_POLY_ORDER) or DEFAULT_POLY_ORDER),
            setpoint_v=float(payload.get("setpoint_v", 0.0) or 0.0),
            sample_rate_hz=float(payload.get("sample_rate_hz", 0.0) or 0.0),
            sources=[str(item) for item in (payload.get("sources") or [])],
            note=str(payload.get("note", "") or ""),
            min_recordings=int(payload.get("min_recordings", DEFAULT_MIN_RECORDINGS) or 1),
            bucket_percent=float(payload.get("bucket_percent", DEFAULT_BUCKET_PERCENT) or DEFAULT_BUCKET_PERCENT),
        )

    def save(self, file_path: str | Path) -> None:
        Path(file_path).write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, file_path: str | Path) -> "ReferenceCurve":
        payload = json.loads(Path(file_path).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("that file is not a bias reference curve")
        return cls.from_dict(payload)


def build_curve(
    readings: Iterable[ReferenceReading],
    *,
    sources: Sequence[str] = (),
    fit_kind: str = DEFAULT_FIT_KIND,
    poly_order: int = DEFAULT_POLY_ORDER,
    setpoint_v: float = 0.0,
    sample_rate_hz: float = 0.0,
    bucket_percent: float = DEFAULT_BUCKET_PERCENT,
    min_recordings: int = DEFAULT_MIN_RECORDINGS,
    note: str = "",
) -> ReferenceCurve:
    """Average the readings and wrap them in a curve with the chosen fit."""

    points = average_readings(
        list(readings), bucket_percent=bucket_percent, min_recordings=min_recordings
    )
    return ReferenceCurve(
        points=points,
        fit_kind=fit_kind if fit_kind in FIT_LABELS else DEFAULT_FIT_KIND,
        poly_order=int(poly_order),
        setpoint_v=float(setpoint_v),
        sample_rate_hz=float(sample_rate_hz),
        sources=list(dict.fromkeys(str(s) for s in sources)),
        note=note,
        min_recordings=int(min_recordings),
        bucket_percent=float(bucket_percent),
    )
