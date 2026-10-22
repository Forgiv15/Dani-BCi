"""Utilities for loading OpenBCI-style CSV recordings into numeric arrays.

The loader is intentionally conservative:
- metadata lines starting with '%' are skipped, matching OpenBCI GUI exports
- only user-selected numeric columns are treated as EEG/ADC channels
- timestamps are normalized to seconds when they match the sampling cadence

If timestamps are absent or ambiguous, the analysis time axis is regenerated from
the user-supplied sampling rate for reproducibility.
"""

from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

import numpy as np
import pandas as pd


@dataclass(slots=True)
class CsvPreview:
    columns: list[str]
    numeric_columns: list[str]
    recommended_timestamp: Optional[str]
    recommended_channels: list[str]
    delimiter: str
    has_header: bool
    format_description: str


@dataclass(slots=True)
class SignalDataset:
    file_path: Path
    sample_rate: float
    time_s: np.ndarray
    channel_names: list[str]
    data: np.ndarray
    source_timestamp: Optional[np.ndarray] = None

    @property
    def sample_count(self) -> int:
        return int(self.data.shape[0])

    @property
    def channel_count(self) -> int:
        return int(self.data.shape[1])

    @property
    def duration_s(self) -> float:
        if self.sample_count <= 1:
            return 0.0
        return float(self.time_s[-1] - self.time_s[0])


def sniff_csv_layout(file_path: str | Path, preview_rows: int = 512) -> CsvPreview:
    """Inspect a CSV file and recommend timestamp and channel columns."""

    delimiter, has_header = _sniff_delimiter_and_header(file_path)
    frame = _read_csv_frame(file_path, delimiter=delimiter, has_header=has_header, nrows=preview_rows)
    if not has_header:
        frame.columns = _generate_column_names(frame)
    if frame.empty:
        raise ValueError("The selected CSV file does not contain any data rows.")

    numeric_frame = frame.apply(pd.to_numeric, errors="coerce")
    numeric_ratio = numeric_frame.notna().mean(axis=0)
    numeric_columns = [column for column in frame.columns if numeric_ratio[column] >= 0.90]
    if not numeric_columns:
        raise ValueError("No numeric columns were detected in the selected CSV file.")

    recommended_timestamp = _recommend_timestamp_column(numeric_frame, numeric_columns)
    recommended_channels = _recommend_channel_columns(numeric_columns)

    return CsvPreview(
        columns=[str(column) for column in frame.columns],
        numeric_columns=[str(column) for column in numeric_columns],
        recommended_timestamp=recommended_timestamp,
        recommended_channels=recommended_channels,
        delimiter=delimiter,
        has_header=has_header,
        format_description=_describe_format(delimiter, has_header),
    )


def load_openbci_csv(
    file_path: str | Path,
    sample_rate: float,
    channel_columns: Sequence[str],
    timestamp_column: Optional[str] = None,
    preview: Optional[CsvPreview] = None,
) -> SignalDataset:
    """Load selected columns from a CSV file into a dense multichannel dataset."""

    if sample_rate <= 0:
        raise ValueError("Sampling rate must be positive.")
    if not channel_columns:
        raise ValueError("At least one channel must be selected.")

    csv_preview = preview or sniff_csv_layout(file_path)
    selected_columns = list(dict.fromkeys([*(channel_columns or []), *([timestamp_column] if timestamp_column else [])]))
    frame = _read_csv_frame(
        file_path,
        delimiter=csv_preview.delimiter,
        has_header=csv_preview.has_header,
    )
    if not csv_preview.has_header:
        frame.columns = csv_preview.columns
    missing_columns = [column for column in selected_columns if column not in frame.columns]
    if missing_columns:
        missing = ", ".join(missing_columns)
        raise ValueError(f"The following selected columns were not found in the file: {missing}")
    frame = frame[selected_columns]
    if frame.empty:
        raise ValueError("The selected CSV file does not contain any usable samples.")

    numeric = frame.apply(pd.to_numeric, errors="coerce")
    valid_mask = numeric[list(channel_columns)].notna().all(axis=1)
    if timestamp_column:
        valid_mask &= numeric[timestamp_column].notna()

    numeric = numeric.loc[valid_mask, selected_columns]
    if numeric.empty:
        raise ValueError("No rows remain after removing incomplete channel samples.")

    data = numeric[list(channel_columns)].to_numpy(dtype=np.float64, copy=True)
    source_timestamp = None
    if timestamp_column:
        source_timestamp = numeric[timestamp_column].to_numpy(dtype=np.float64, copy=True)
        time_s = _normalize_timebase(source_timestamp, sample_rate)
    else:
        time_s = np.arange(data.shape[0], dtype=np.float64) / float(sample_rate)

    channel_names = [_clean_channel_name(column, index) for index, column in enumerate(channel_columns)]
    return SignalDataset(
        file_path=Path(file_path),
        sample_rate=float(sample_rate),
        time_s=time_s,
        channel_names=channel_names,
        data=data,
        source_timestamp=source_timestamp,
    )


def _read_csv_frame(
    file_path: str | Path,
    delimiter: str,
    has_header: bool,
    nrows: Optional[int] = None,
) -> pd.DataFrame:
    return pd.read_csv(
        file_path,
        comment="%",
        sep=delimiter,
        header=0 if has_header else None,
        nrows=nrows,
        low_memory=False,
        skip_blank_lines=True,
    )


def _sniff_delimiter_and_header(file_path: str | Path) -> tuple[str, bool]:
    sample_lines = _read_sample_lines(file_path)
    if not sample_lines:
        raise ValueError("The selected CSV file does not contain any readable rows.")

    sample_text = "".join(sample_lines)
    delimiter = _detect_delimiter(sample_text)
    first_row = next(csv.reader([sample_lines[0]], delimiter=delimiter))
    has_header = not _row_is_mostly_numeric(first_row)
    return delimiter, has_header


def _read_sample_lines(file_path: str | Path, max_lines: int = 32) -> list[str]:
    lines: list[str] = []
    with Path(file_path).open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            lines.append(line)
            if len(lines) >= max_lines:
                break
    return lines


def _detect_delimiter(sample_text: str) -> str:
    try:
        dialect = csv.Sniffer().sniff(sample_text, delimiters=",;\t")
        return str(dialect.delimiter)
    except csv.Error:
        if "\t" in sample_text:
            return "\t"
        if ";" in sample_text:
            return ";"
        return ","


def _row_is_mostly_numeric(values: Sequence[str]) -> bool:
    cleaned = [str(value).strip() for value in values if str(value).strip()]
    if not cleaned:
        return False
    numeric_count = sum(_is_number(value) for value in cleaned)
    return numeric_count / len(cleaned) >= 0.9


def _is_number(text: str) -> bool:
    try:
        float(text)
    except ValueError:
        return False
    return True


def _generate_column_names(frame: pd.DataFrame) -> list[str]:
    column_count = int(frame.shape[1])
    names = [f"Column {index + 1}" for index in range(column_count)]
    if column_count >= 1:
        names[0] = "Sample Index"
    for index in range(1, min(column_count, 9)):
        names[index] = f"Ch {index}"
    accel_names = ["Accel X", "Accel Y", "Accel Z"]
    for index, name in enumerate(accel_names, start=9):
        if index < column_count:
            names[index] = name
    for index in range(12, column_count):
        names[index] = f"Aux {index - 11}"

    numeric_frame = frame.apply(pd.to_numeric, errors="coerce")
    timestamp_index = _recommend_timestamp_index(numeric_frame)
    if timestamp_index is not None:
        names[timestamp_index] = "Timestamp"
    if column_count >= 1 and names[-1].startswith("Aux"):
        names[-1] = "Marker"
    return names


def _recommend_timestamp_column(numeric_frame: pd.DataFrame, numeric_columns: Sequence[str]) -> Optional[str]:
    named_match = next(
        (
            column
            for column in numeric_columns
            if any(token in column.lower() for token in ("time", "timestamp", "elapsed"))
        ),
        None,
    )
    if named_match:
        return named_match

    timestamp_index = _recommend_timestamp_index(numeric_frame[numeric_columns])
    if timestamp_index is None:
        return None
    return str(list(numeric_columns)[timestamp_index])


def _recommend_timestamp_index(numeric_frame: pd.DataFrame) -> Optional[int]:
    best_index: Optional[int] = None
    best_score = float("-inf")
    for index, column in enumerate(numeric_frame.columns):
        values = numeric_frame[column].to_numpy(dtype=np.float64, copy=False)
        finite_values = values[np.isfinite(values)]
        if finite_values.size < 4:
            continue
        deltas = np.diff(finite_values)
        positive_ratio = float(np.mean(deltas > 0)) if deltas.size else 0.0
        if positive_ratio < 0.95:
            continue
        span = float(finite_values[-1] - finite_values[0])
        if span <= 0:
            continue
        score = positive_ratio * 100.0
        if np.nanmedian(finite_values) > 1e6:
            score += 1000.0
        if span > 1.0:
            score += min(np.log10(span + 1.0), 12.0)
        if score > best_score:
            best_score = score
            best_index = index
    return best_index


def _describe_format(delimiter: str, has_header: bool) -> str:
    delimiter_name = {",": "comma-separated", "\t": "tab-separated", ";": "semicolon-separated"}.get(delimiter, "delimited")
    header_text = "headered" if has_header else "headerless"
    return f"Detected {header_text} {delimiter_name} data"


def _recommend_channel_columns(numeric_columns: Sequence[str]) -> list[str]:
    direct_match = [column for column in numeric_columns if column.lower().startswith(("ch ", "ch", "eeg"))]
    if direct_match:
        return direct_match[:32]

    excluded_tokens = ("time", "timestamp", "elapsed", "sample", "index", "accel", "aux", "marker")
    preferred = [
        column
        for column in numeric_columns
        if not any(token in column.lower() for token in excluded_tokens)
    ]
    if preferred:
        return preferred[:32]

    fallback = [column for column in numeric_columns if "sample" not in column.lower()]
    return list(fallback[:32] if fallback else numeric_columns[:32])


def _normalize_timebase(timestamp_values: np.ndarray, sample_rate: float) -> np.ndarray:
    values = np.asarray(timestamp_values, dtype=np.float64)
    if values.size < 2 or not np.all(np.isfinite(values)):
        return np.arange(values.size, dtype=np.float64) / float(sample_rate)

    deltas = np.diff(values)
    positive_deltas = deltas[deltas > 0]
    if positive_deltas.size == 0:
        return np.arange(values.size, dtype=np.float64) / float(sample_rate)

    measured_dt = float(np.median(positive_deltas))
    expected_dt = 1.0 / float(sample_rate)
    scale = _infer_time_scale(measured_dt, expected_dt)
    if scale is None:
        return np.arange(values.size, dtype=np.float64) / float(sample_rate)

    normalized = (values - values[0]) * scale
    if np.any(np.diff(normalized) <= 0):
        return np.arange(values.size, dtype=np.float64) / float(sample_rate)
    return normalized


def _infer_time_scale(measured_dt: float, expected_dt: float) -> Optional[float]:
    candidates = (1.0, 1e-3, 1e-6, 1e-9)
    for scale in candidates:
        scaled_dt = measured_dt * scale
        if expected_dt == 0:
            continue
        relative_error = abs(scaled_dt - expected_dt) / expected_dt
        if relative_error <= 0.35:
            return scale
    return None


def _clean_channel_name(column_name: str, index: int) -> str:
    text = str(column_name).strip()
    if not text:
        return f"Ch {index + 1}"
    return text


def read_openbci_metadata(file_path: str | Path, max_lines: int = 128) -> dict[str, str]:
    """Parse the ``%Key = Value`` comment block of an OpenBCI-style CSV."""

    metadata: dict[str, str] = {}
    path = Path(file_path)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, line in enumerate(handle):
            if line_number >= max_lines:
                break
            if not line.startswith("%"):
                if line.strip():
                    break
                continue
            body = line[1:].strip()
            if not body:
                continue
            if "=" in body:
                key, value = body.split("=", 1)
                metadata[key.strip()] = value.strip()
            else:
                metadata.setdefault("format", body)
    return metadata


def openbci_sample_rate(metadata: Mapping[str, str]) -> Optional[float]:
    """Return the sample rate recorded in an OpenBCI header, if present."""

    value = ""
    for key, item in metadata.items():
        if key.lower() == "sample rate":
            value = item
            break
    match = re.search(r"[-+]?[0-9]*\.?[0-9]+", value)
    if not match:
        return None
    rate = float(match.group(0))
    return rate if rate > 0 else None
