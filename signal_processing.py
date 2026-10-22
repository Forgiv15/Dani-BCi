"""Time-domain utilities for selection handling, RMS measurements, and filtering."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from scipy import signal


@dataclass(slots=True)
class FilterSettings:
    notch_50_hz: bool = False
    notch_100_hz: bool = False
    quality_factor: float = 30.0

    def enabled_frequencies(self) -> list[float]:
        frequencies: list[float] = []
        if self.notch_50_hz:
            frequencies.append(50.0)
        if self.notch_100_hz:
            frequencies.append(100.0)
        return frequencies


def rms(values: np.ndarray, axis: int | None = None) -> np.ndarray:
    """Compute true RMS with finite-value masking."""

    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        return np.sqrt(np.nanmean(np.square(array), axis=axis))
    return np.sqrt(np.mean(np.square(array), axis=axis))


def apply_filters(data: np.ndarray, sample_rate: float, settings: FilterSettings) -> np.ndarray:
    """Apply optional 50 Hz and 100 Hz notch filters to a signal matrix."""

    frequencies = settings.enabled_frequencies()
    if not frequencies:
        return np.asarray(data, dtype=np.float64)

    filtered = np.asarray(data, dtype=np.float64).copy()
    for frequency in frequencies:
        if frequency >= 0.49 * sample_rate:
            continue
        b, a = signal.iirnotch(w0=frequency, Q=settings.quality_factor, fs=sample_rate)
        padlen = min(3 * (max(len(a), len(b)) - 1), filtered.shape[0] - 1)
        if padlen < 1:
            continue
        filtered = signal.filtfilt(b, a, filtered, axis=0, padlen=padlen)
    return filtered


def apply_bandpass_filter(
    data: np.ndarray,
    sample_rate: float,
    low_cut_hz: float,
    high_cut_hz: float,
    order: int = 4,
) -> np.ndarray:
    """Apply a zero-phase Butterworth bandpass to a signal vector or matrix."""

    filtered = np.asarray(data, dtype=np.float64)
    if filtered.size == 0 or filtered.shape[0] < 8 or sample_rate <= 0:
        return filtered.copy()

    nyquist_hz = 0.5 * float(sample_rate)
    low_hz = max(float(low_cut_hz), 0.0)
    high_hz = min(float(high_cut_hz), 0.95 * nyquist_hz)
    if high_hz <= 0:
        return filtered.copy()

    if 0.0 < low_hz < high_hz:
        critical = [low_hz, high_hz]
        filter_type = "bandpass"
    else:
        critical = high_hz
        filter_type = "lowpass"

    sos = signal.butter(order, critical, btype=filter_type, fs=sample_rate, output="sos")
    padlen = min(3 * (2 * sos.shape[0] + 1), filtered.shape[0] - 1)
    if padlen < 1:
        return filtered.copy()
    return signal.sosfiltfilt(sos, filtered, axis=0, padlen=padlen)


def extract_time_region(
    time_s: np.ndarray,
    data: np.ndarray,
    start_s: float,
    end_s: float,
    excluded_ranges: Sequence[tuple[float, float]] = (),
) -> tuple[np.ndarray, np.ndarray]:
    """Return the selected time region with exclusion segments removed."""

    if data.size == 0:
        return np.empty(0, dtype=np.float64), np.empty((0, data.shape[1] if data.ndim == 2 else 0), dtype=np.float64)

    lo, hi = sorted((float(start_s), float(end_s)))
    start_index = int(np.searchsorted(time_s, lo, side="left"))
    end_index = int(np.searchsorted(time_s, hi, side="right"))
    region_time = time_s[start_index:end_index]
    region_data = data[start_index:end_index]

    if region_time.size == 0:
        return region_time, region_data

    keep_mask = build_keep_mask(region_time, excluded_ranges)
    return region_time[keep_mask], region_data[keep_mask]


def build_keep_mask(time_s: np.ndarray, excluded_ranges: Sequence[tuple[float, float]]) -> np.ndarray:
    keep_mask = np.ones(time_s.shape[0], dtype=bool)
    for start_s, end_s in excluded_ranges:
        lo, hi = sorted((float(start_s), float(end_s)))
        keep_mask &= ~((time_s >= lo) & (time_s <= hi))
    return keep_mask


def hard_remove_ranges(
    time_s: np.ndarray,
    data: np.ndarray,
    excluded_ranges: Sequence[tuple[float, float]],
    sample_rate: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Physically remove excluded ranges and rebuild a contiguous time axis."""

    if data.size == 0 or not excluded_ranges:
        return np.asarray(time_s, dtype=np.float64), np.asarray(data, dtype=np.float64)

    keep_mask = build_keep_mask(np.asarray(time_s, dtype=np.float64), excluded_ranges)
    reduced_data = np.asarray(data, dtype=np.float64)[keep_mask]
    reduced_time = np.arange(reduced_data.shape[0], dtype=np.float64) / float(sample_rate)
    return reduced_time, reduced_data


def keep_only_range(
    time_s: np.ndarray,
    data: np.ndarray,
    start_s: float,
    end_s: float,
    sample_rate: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep only the selected range and rebuild a contiguous time axis."""

    if data.size == 0:
        return np.asarray(time_s, dtype=np.float64), np.asarray(data, dtype=np.float64)

    lo, hi = sorted((float(start_s), float(end_s)))
    start_index = int(np.searchsorted(time_s, lo, side="left"))
    end_index = int(np.searchsorted(time_s, hi, side="right"))
    reduced_data = np.asarray(data, dtype=np.float64)[start_index:end_index]
    reduced_time = np.arange(reduced_data.shape[0], dtype=np.float64) / float(sample_rate)
    return reduced_time, reduced_data


def merge_ranges(ranges: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    sorted_ranges = sorted((min(a, b), max(a, b)) for a, b in ranges)
    if not sorted_ranges:
        return []

    merged = [sorted_ranges[0]]
    for start_s, end_s in sorted_ranges[1:]:
        prev_start, prev_end = merged[-1]
        if start_s <= prev_end:
            merged[-1] = (prev_start, max(prev_end, end_s))
            continue
        merged.append((start_s, end_s))
    return merged