"""Frequency-domain analysis with window-corrected RMS spectra.

The FFT scaling is designed so that a bin-centered sinusoid is reported in RMS
units directly from the spectrum, including correction for the selected window's
coherent gain.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace
from typing import Callable, Mapping, Optional, Sequence

import numpy as np
import pandas as pd
from scipy import signal

from signal_processing import FilterSettings, apply_bandpass_filter, apply_filters, rms


WINDOW_MAP = {
    "Hann": "hann",
    "Hamming": "hamming",
    "Blackman": "blackman",
    "Rectangular": "boxcar",
}


DEFAULT_INPUT_ATTENUATOR_RATIO = 100.0 / 1_000_100.0
# Band used for the input-referred noise metric: what the live recorder shows
# with its default 0.1 / 150 Hz display filters, capped at 100 Hz so mains
# harmonics above the EEG band do not inflate it.
BAND_NOISE_LOW_HZ = 0.1
BAND_NOISE_HIGH_HZ = 100.0
BAND_NOISE_LABEL = f"Band RMS {BAND_NOISE_LOW_HZ:g}-{BAND_NOISE_HIGH_HZ:g} Hz (uVrms)"
VREF_VOLTS = 4.5
GAIN_ERROR_LABEL = "Gain Error (dB)"
SNR_INPUT_LABEL = "SNR In-Referred (dB)"
ENOB_FS_LABEL = "ENOB @ FS (bits)"
CHANNEL_AVERAGE_LABEL = "All channels (mean)"
EXAMPLE_ADC_GAIN_RESPONSE_FREQ_HZ = np.asarray([5.0, 10.0, 20.0, 30.0, 40.0, 60.0, 70.0, 80.0, 90.0, 110.0], dtype=np.float64)
EXAMPLE_ADC_GAIN_RESPONSE_DB = np.asarray([0.0, 0.0, -0.2, -0.5, -1.0, -2.5, -3.5, -4.8, -6.0, -9.0], dtype=np.float64)


@dataclass(slots=True)
class FFTSettings:
    fft_length: int = 4096
    window_name: str = "Hann"
    manual_fundamental_hz: Optional[float] = None
    cmrr_frequency_hz: Optional[float] = None
    crosstalk_frequency_hz: Optional[float] = None
    cmrr_correction_enabled: bool = False
    cmrr_correction_fs_hz: float = 250.0
    # RMS of the signal the user applied to the inputs, in volts. Zero means
    # "not specified", in which case the input-referred columns stay empty.
    applied_input_rms_v: float = 0.0
    # Full-scale RMS of the channel (VREF / gain / sqrt(2)), in volts. Needed to
    # express ENOB against the converter's full scale.
    full_scale_rms_v: float = VREF_VOLTS / 24.0 / np.sqrt(2.0)


@dataclass(slots=True)
class SpectrumResult:
    frequencies_hz: np.ndarray
    amplitude_rms: np.ndarray
    fft_length: int
    window_name: str
    fundamental_frequency_hz: Optional[float]
    fundamental_rms: float
    signal_power: float
    noise_power: float
    snr_db: float
    thd_percent: float
    thd_db: float
    enob: float
    time_domain_rms: float
    selection_avg_rms: float
    band_rms_0p1_100hz: float
    gain_error_db: Optional[float]
    snr_input_db: Optional[float]
    enob_full_scale: Optional[float]
    pickup_50hz_rms: float
    cmrr_measured_db: Optional[float]
    cmrr_correction_gain_db: Optional[float]
    cmrr_db: Optional[float]
    cmrr_frequency_hz: Optional[float]
    cmrr_output_rms: Optional[float]
    fundamental_source: str


@dataclass(slots=True)
class CMRRSweepResult:
    dataframe: pd.DataFrame
    accepted_windows: int = 0
    rejected_windows: int = 0


@dataclass(slots=True)
class CrosstalkSweepResult:
    dataframe: pd.DataFrame
    accepted_windows: int = 0
    rejected_windows: int = 0


@dataclass(slots=True)
class GainSweepResult:
    dataframe: pd.DataFrame
    accepted_windows: int = 0
    rejected_windows: int = 0


def ads1299_sinc3_gain_db(frequencies_hz: np.ndarray | Sequence[float] | float, output_data_rate_hz: float) -> np.ndarray:
    r"""Return the ADS1299 sinc3 digital filter gain in dB.

    $$H(f)=\left(\frac{\sin(\pi f/f_s)}{\pi f/f_s}\right)^3$$
    $$G_{dB}(f)=20\log_{10}(|H(f)|)$$
    """

    frequencies = np.asarray(frequencies_hz, dtype=np.float64)
    if output_data_rate_hz <= 0:
        return np.zeros_like(frequencies)

    x = np.pi * frequencies / float(output_data_rate_hz)
    sinc_term = np.ones_like(frequencies)
    nonzero_mask = np.abs(x) > 1e-15
    sinc_term[nonzero_mask] = np.sin(x[nonzero_mask]) / x[nonzero_mask]
    gain_linear = np.abs(sinc_term) ** 3
    gain_linear = np.clip(gain_linear, np.finfo(np.float64).tiny, None)
    return 20.0 * np.log10(gain_linear)


def build_ads1299_sinc3_gain_response(
    output_data_rate_hz: float,
    max_frequency_hz: float,
    point_count: int = 4096,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a dense ADS1299 sinc3 gain-response table for interpolation."""

    max_frequency = max(0.0, min(float(max_frequency_hz), float(output_data_rate_hz)))
    if point_count < 2:
        point_count = 2
    response_frequencies = np.linspace(0.0, max_frequency, int(point_count), dtype=np.float64)
    response_gain_db = ads1299_sinc3_gain_db(response_frequencies, output_data_rate_hz)
    return response_frequencies, response_gain_db


def interpolate_gain_response_db(
    frequencies_hz: np.ndarray | Sequence[float] | float,
    response_frequencies_hz: np.ndarray | Sequence[float],
    response_gain_db: np.ndarray | Sequence[float],
) -> np.ndarray:
    """Interpolate a stored gain-response table in dB across frequency."""

    frequencies = np.asarray(frequencies_hz, dtype=np.float64)
    response_frequencies = np.asarray(response_frequencies_hz, dtype=np.float64)
    response_gain = np.asarray(response_gain_db, dtype=np.float64)
    if response_frequencies.size == 0 or response_gain.size == 0 or response_frequencies.size != response_gain.size:
        return np.zeros_like(frequencies)

    order = np.argsort(response_frequencies)
    return np.interp(frequencies, response_frequencies[order], response_gain[order], left=response_gain[order][0], right=response_gain[order][-1])


def compute_cmrr_correction_gain_db(
    frequencies_hz: np.ndarray | Sequence[float] | float,
    correction_enabled: bool,
    correction_fs_hz: float,
) -> np.ndarray:
    """Return the ADS1299 gain correction term used to compensate measured CMRR."""

    frequencies = np.asarray(frequencies_hz, dtype=np.float64)
    if not correction_enabled or correction_fs_hz <= 0:
        return np.zeros_like(frequencies)

    max_frequency_hz = float(np.max(np.abs(frequencies))) if frequencies.size else 0.0
    response_frequencies, response_gain_db = build_ads1299_sinc3_gain_response(correction_fs_hz, max_frequency_hz=max_frequency_hz)
    return interpolate_gain_response_db(np.abs(frequencies), response_frequencies, response_gain_db)


def analyze_multichannel(
    data: np.ndarray,
    channel_names: Sequence[str],
    sample_rate: float,
    fft_settings: FFTSettings,
    common_mode_voltage: Optional[float] = None,
    crosstalk_source_index: int = 0,
    signal_scale_to_volts: float = 1e-6,
    mean_channel_names: Optional[Sequence[str]] = None,
    channel_gains: Optional[Sequence[float]] = None,
) -> tuple[pd.DataFrame, pd.DataFrame, list[SpectrumResult]]:
    """Analyze each channel and derive a crosstalk summary.

    ``mean_channel_names`` restricts the "All channels (mean)" row to the
    channels the user actually has visible; it never changes the per-channel
    metrics.
    """

    spectra: list[SpectrumResult] = []
    rows: list[dict[str, object]] = []
    for index, channel_name in enumerate(channel_names):
        settings_for_channel = fft_settings
        if channel_gains is not None and index < len(channel_gains):
            # ENOB is referenced to the full scale of *this* channel's PGA gain.
            gain = max(float(channel_gains[index]), 1.0)
            settings_for_channel = replace(
                fft_settings, full_scale_rms_v=VREF_VOLTS / gain / np.sqrt(2.0)
            )
        result = analyze_channel(
            data[:, index],
            sample_rate=sample_rate,
            fft_settings=settings_for_channel,
            common_mode_voltage=common_mode_voltage,
            signal_scale_to_volts=signal_scale_to_volts,
        )
        spectra.append(result)
        rows.append(
            {
                "Channel": channel_name,
                "Selection Avg RMS (uVrms)": result.selection_avg_rms,
                BAND_NOISE_LABEL: result.band_rms_0p1_100hz,
                "Fundamental (Hz)": result.fundamental_frequency_hz,
                "Fundamental RMS (uVrms)": result.fundamental_rms,
                "Fundamental Source": result.fundamental_source,
                "50 Hz Pickup (uVrms)": result.pickup_50hz_rms,
                "SNR (dB)": result.snr_db,
                "THD (%)": result.thd_percent,
                "THD (dB)": result.thd_db,
                "ENOB (bits)": result.enob,
                ENOB_FS_LABEL: result.enob_full_scale,
                GAIN_ERROR_LABEL: result.gain_error_db,
                SNR_INPUT_LABEL: result.snr_input_db,
                "CMRR Freq (Hz)": result.cmrr_frequency_hz,
                "CMRR Output (uVrms)": result.cmrr_output_rms,
                "ADC Gain Corr (dB)": result.cmrr_correction_gain_db,
                "CMRR Measured (dB)": result.cmrr_measured_db,
                "CMRR (dB)": result.cmrr_db,
            }
        )

    metrics = pd.DataFrame(rows)
    metrics = _append_channel_average(metrics, mean_channel_names)
    crosstalk = compute_crosstalk(
        data=data,
        channel_names=channel_names,
        spectra=spectra,
        sample_rate=sample_rate,
        source_index=crosstalk_source_index,
        measurement_frequency_hz=fft_settings.crosstalk_frequency_hz,
        fft_length=fft_settings.fft_length,
    )
    return metrics, crosstalk, spectra


def _append_channel_average(
    metrics: pd.DataFrame,
    mean_channel_names: Optional[Sequence[str]] = None,
) -> pd.DataFrame:
    """Add an "All channels (mean)" row so one number summarises the selection.

    Only the rows in ``mean_channel_names`` are averaged, so channels the user
    switched off cannot move the summary value. The row label states the count
    so the report is unambiguous.
    """

    if metrics.empty or "Channel" not in metrics.columns:
        return metrics

    frame = metrics
    restricted = False
    if mean_channel_names is not None:
        wanted = [name for name in mean_channel_names if name in set(metrics["Channel"])]
        if wanted:
            frame = metrics[metrics["Channel"].isin(wanted)]
            restricted = len(frame.index) != len(metrics.index)

    label = CHANNEL_AVERAGE_LABEL
    if restricted:
        label = f"Mean of {len(frame.index)} visible"

    summary: dict[str, object] = {"Channel": label}
    for column in metrics.columns:
        if column == "Channel":
            continue
        series = frame[column]
        if pd.api.types.is_numeric_dtype(series):
            summary[column] = series.mean(skipna=True)
        else:
            summary[column] = ""
    average_row = pd.DataFrame([summary], columns=metrics.columns)
    return pd.concat([metrics, average_row], ignore_index=True)


def analyze_channel(
    values: np.ndarray,
    sample_rate: float,
    fft_settings: FFTSettings,
    common_mode_voltage: Optional[float] = None,
    signal_scale_to_volts: float = 1e-6,
) -> SpectrumResult:
    """Compute RMS, FFT, SNR, THD, ENOB, and optional CMRR for one signal."""

    signal_values = np.asarray(values, dtype=np.float64)
    if signal_values.size < 8:
        raise ValueError("At least 8 samples are required for spectral analysis.")

    time_domain_rms = float(rms(signal_values))
    selection_avg_values = apply_bandpass_filter(signal_values, sample_rate, low_cut_hz=0.5, high_cut_hz=100.0, order=4)
    selection_avg_rms = float(rms(selection_avg_values))
    band_values = apply_bandpass_filter(signal_values, sample_rate, low_cut_hz=BAND_NOISE_LOW_HZ, high_cut_hz=BAND_NOISE_HIGH_HZ, order=4)
    band_rms = float(rms(band_values))
    analysis_values = _prepare_fft_segment(signal_values - np.mean(signal_values), int(fft_settings.fft_length))
    frequencies_hz, amplitude_rms = _compute_rms_spectrum_from_segment(analysis_values, sample_rate, fft_settings.window_name)
    fundamental_index = detect_fundamental_index(
        frequencies_hz,
        amplitude_rms,
        manual_frequency_hz=fft_settings.manual_fundamental_hz,
    )
    fundamental_source = "Manual" if fft_settings.manual_fundamental_hz is not None else "Auto"
    pickup_50hz_rms = _fit_sine_component_rms(analysis_values, sample_rate, 50.0) if sample_rate > 100.0 else 0.0

    if fundamental_index is None:
        signal_power = 0.0
        noise_power = float(rms(analysis_values) ** 2)
        snr_db = float("-inf")
        thd_percent = 0.0
        thd_db = float("-inf")
        enob = float("-inf")
        fundamental_frequency_hz = None
        fundamental_rms = 0.0
    else:
        if fft_settings.manual_fundamental_hz is not None and fft_settings.manual_fundamental_hz > 0:
            fundamental_frequency_hz = float(fft_settings.manual_fundamental_hz)
        else:
            fundamental_frequency_hz = _refine_peak_frequency(frequencies_hz, amplitude_rms, fundamental_index)

        # Refine around the detected peak: one FFT bin is 0.12 Hz here, and a
        # fractional-bin error would otherwise dominate the residual.
        bin_hz = float(frequencies_hz[1] - frequencies_hz[0]) if len(frequencies_hz) > 1 else 0.0
        search_hz = bin_hz if fft_settings.manual_fundamental_hz is None else max(bin_hz * 0.5, 0.01)
        fundamental_rms, residual_rms, fitted_frequency_hz = fit_sine_refined(
            analysis_values, sample_rate, fundamental_frequency_hz, search_hz
        )
        if fitted_frequency_hz > 0:
            fundamental_frequency_hz = float(fitted_frequency_hz)
        signal_power = float(fundamental_rms**2)

        # Noise = everything the fitted fundamental does not explain. This is the
        # residual of the same least-squares fit, which stays accurate even when
        # the tone is far above the noise floor.
        noise_power = float(residual_rms**2)
        snr_db = _power_ratio_db(signal_power, noise_power)

        harmonic_power = 0.0
        for harmonic_number in range(2, 6):
            harmonic_frequency = fundamental_frequency_hz * harmonic_number
            if harmonic_frequency >= sample_rate / 2.0:
                continue
            harmonic_rms = _fit_sine_component_rms(analysis_values, sample_rate, harmonic_frequency)
            harmonic_power += float(harmonic_rms**2)

        thd_percent = float(100.0 * np.sqrt(harmonic_power / signal_power)) if signal_power > 0 else 0.0
        thd_db = 20.0 * np.log10(thd_percent / 100.0) if thd_percent > 0 else float("-inf")
        enob = (snr_db - 1.76) / 6.02 if np.isfinite(snr_db) else float("-inf")

    # Referred-to-input figures. These need the level that was actually applied,
    # because the measured fundamental also contains the ADC gain error.
    gain_error_db: Optional[float] = None
    snr_input_db: Optional[float] = None
    enob_full_scale: Optional[float] = None
    applied_rms_uv = float(fft_settings.applied_input_rms_v) * 1e6
    noise_rms_uv = float(np.sqrt(max(noise_power, 0.0)))
    if applied_rms_uv > 0.0 and fundamental_rms > 0.0:
        gain_error_db = float(20.0 * np.log10(fundamental_rms / applied_rms_uv))
        if noise_rms_uv > 0.0:
            snr_input_db = float(20.0 * np.log10(applied_rms_uv / noise_rms_uv))
    if np.isfinite(snr_db) and snr_db > 0.0 and fundamental_rms > 0.0:
        # ENOB is only defined for a full-scale input. Scale the measured SNR up
        # by the headroom between the measured fundamental and full scale.
        # A positive SNR is required: on a pure-noise record (shorted input) the
        # "fundamental" is just the largest noise bin, and scaling that up would
        # report an ENOB above the converter resolution.
        full_scale_uv = float(fft_settings.full_scale_rms_v) * 1e6
        if full_scale_uv > fundamental_rms > 0.0:
            snr_full_scale = snr_db + 20.0 * math.log10(full_scale_uv / fundamental_rms)
            candidate = (snr_full_scale - 1.76) / 6.02
            # Never report more bits than the converter has.
            if candidate <= 24.0:
                enob_full_scale = float(candidate)

    cmrr_measured_db: Optional[float] = None
    cmrr_correction_gain_db: Optional[float] = None
    cmrr_db: Optional[float] = None
    cmrr_frequency_hz: Optional[float] = None
    cmrr_output_rms: Optional[float] = None
    if fft_settings.cmrr_frequency_hz is not None and fft_settings.cmrr_frequency_hz > 0:
        cmrr_frequency_hz = float(fft_settings.cmrr_frequency_hz)
        cmrr_output_rms = _fit_sine_component_rms(analysis_values, sample_rate, cmrr_frequency_hz)
    elif fundamental_rms > 0 and fundamental_frequency_hz is not None:
        cmrr_frequency_hz = float(fundamental_frequency_hz)
        cmrr_output_rms = float(fundamental_rms)
    else:
        cmrr_output_rms = float(time_domain_rms)

    measured_output_volts = cmrr_output_rms * signal_scale_to_volts if cmrr_output_rms is not None else None
    if common_mode_voltage and common_mode_voltage > 0 and measured_output_volts and measured_output_volts > 0:
        cmrr_measured_db = 20.0 * np.log10(common_mode_voltage / measured_output_volts)
        if cmrr_frequency_hz is not None and cmrr_frequency_hz > 0:
            cmrr_correction_gain_db = float(
                compute_cmrr_correction_gain_db(
                    [cmrr_frequency_hz],
                    correction_enabled=fft_settings.cmrr_correction_enabled,
                    correction_fs_hz=fft_settings.cmrr_correction_fs_hz,
                )[0]
            )
        else:
            cmrr_correction_gain_db = 0.0
        cmrr_db = cmrr_measured_db + cmrr_correction_gain_db

    return SpectrumResult(
        frequencies_hz=frequencies_hz,
        amplitude_rms=amplitude_rms,
        fft_length=int(fft_settings.fft_length),
        window_name=fft_settings.window_name,
        fundamental_frequency_hz=fundamental_frequency_hz,
        fundamental_rms=fundamental_rms,
        signal_power=signal_power,
        noise_power=noise_power,
        snr_db=snr_db,
        thd_percent=thd_percent,
        thd_db=thd_db,
        enob=enob,
        time_domain_rms=time_domain_rms,
        selection_avg_rms=selection_avg_rms,
        band_rms_0p1_100hz=band_rms,
        gain_error_db=gain_error_db,
        snr_input_db=snr_input_db,
        enob_full_scale=enob_full_scale,
        pickup_50hz_rms=pickup_50hz_rms,
        cmrr_measured_db=cmrr_measured_db,
        cmrr_correction_gain_db=cmrr_correction_gain_db,
        cmrr_db=cmrr_db,
        cmrr_frequency_hz=cmrr_frequency_hz,
        cmrr_output_rms=cmrr_output_rms,
        fundamental_source=fundamental_source,
    )


def compute_rms_spectrum(
    values: np.ndarray,
    sample_rate: float,
    fft_settings: FFTSettings,
) -> tuple[np.ndarray, np.ndarray]:
    """Return a one-sided amplitude spectrum in RMS units."""

    signal_values = np.asarray(values, dtype=np.float64)
    prepared = _prepare_fft_segment(signal_values - np.mean(signal_values), int(fft_settings.fft_length))
    return _compute_rms_spectrum_from_segment(prepared, sample_rate, fft_settings.window_name)


def _compute_rms_spectrum_from_segment(
    prepared: np.ndarray,
    sample_rate: float,
    window_name: str,
) -> tuple[np.ndarray, np.ndarray]:
    window = signal.get_window(WINDOW_MAP[window_name], prepared.size, fftbins=True)
    windowed = prepared * window
    spectrum = np.fft.rfft(windowed)
    frequencies_hz = np.fft.rfftfreq(prepared.size, d=1.0 / float(sample_rate))

    coherent_gain = float(np.sum(window))
    amplitude_rms = np.abs(spectrum) / coherent_gain
    if amplitude_rms.size > 2:
        amplitude_rms[1:-1] *= np.sqrt(2.0)
    elif amplitude_rms.size == 2:
        amplitude_rms[1] *= np.sqrt(2.0)

    if prepared.size % 2 == 0 and amplitude_rms.size >= 2:
        amplitude_rms[-1] /= np.sqrt(2.0)

    amplitude_rms[0] = 0.0
    return frequencies_hz, amplitude_rms


def detect_fundamental_index(
    frequencies_hz: np.ndarray,
    amplitude_rms: np.ndarray,
    manual_frequency_hz: Optional[float] = None,
) -> Optional[int]:
    if amplitude_rms.size <= 1:
        return None
    positive_indices = np.where(frequencies_hz > 0.0)[0]
    if positive_indices.size == 0:
        return None
    if manual_frequency_hz is not None and manual_frequency_hz > 0:
        return int(positive_indices[np.argmin(np.abs(frequencies_hz[positive_indices] - manual_frequency_hz))])
    positive_amplitudes = amplitude_rms[positive_indices]
    if not np.any(positive_amplitudes > 0):
        return None
    return int(positive_indices[np.argmax(positive_amplitudes)])


def compute_crosstalk(
    data: np.ndarray,
    channel_names: Sequence[str],
    spectra: Sequence[SpectrumResult],
    sample_rate: float,
    source_index: int,
    measurement_frequency_hz: Optional[float] = None,
    fft_length: int = 4096,
) -> pd.DataFrame:
    """Measure per-channel RMS at one selected frequency relative to the source channel.

    Crosstalk is defined here as the ratio between the target channel RMS and the
    source channel RMS at the same selected frequency component, evaluated on the
    same FFT analysis segment used by the metrics table.
    """

    if not spectra:
        return pd.DataFrame(columns=["Source", "Target", "Frequency (Hz)", "Source RMS (uVrms)", "Target RMS (uVrms)", "Crosstalk (dB)"])

    source = spectra[source_index]
    selected_frequency_hz = float(measurement_frequency_hz) if measurement_frequency_hz is not None and measurement_frequency_hz > 0 else source.fundamental_frequency_hz
    if selected_frequency_hz is None or selected_frequency_hz <= 0:
        return pd.DataFrame(columns=["Source", "Target", "Frequency (Hz)", "Source RMS (uVrms)", "Target RMS (uVrms)", "Crosstalk (dB)"])

    source_segment = _prepare_measurement_segment(data[:, source_index], fft_length)
    source_rms = _fit_sine_component_rms(source_segment, sample_rate, selected_frequency_hz)
    if source_rms <= 0:
        return pd.DataFrame(columns=["Source", "Target", "Frequency (Hz)", "Source RMS (uVrms)", "Target RMS (uVrms)", "Crosstalk (dB)"])

    rows: list[dict[str, object]] = []
    for target_index, target_name in enumerate(channel_names):
        target_segment = _prepare_measurement_segment(data[:, target_index], fft_length)
        target_rms = _fit_sine_component_rms(target_segment, sample_rate, selected_frequency_hz)
        crosstalk_db = 20.0 * np.log10(target_rms / source_rms) if target_rms > 0 else float("-inf")
        rows.append(
            {
                "Source": channel_names[source_index],
                "Target": target_name,
                "Frequency (Hz)": selected_frequency_hz,
                "Source RMS (uVrms)": source_rms,
                "Target RMS (uVrms)": target_rms,
                "Crosstalk (dB)": crosstalk_db,
            }
        )
    return pd.DataFrame(rows)


def _prepare_measurement_segment(values: np.ndarray, fft_length: int) -> np.ndarray:
    signal_values = np.asarray(values, dtype=np.float64)
    prepared = _prepare_fft_segment(signal_values - np.mean(signal_values), int(fft_length))
    return prepared


def _fit_sine_component(values: np.ndarray, sample_rate: float, frequency_hz: float) -> tuple[float, float]:
    """Least-squares sine fit at one frequency.

    Returns ``(rms, residual_rms)``. The residual is the part of the signal the
    fitted tone and its DC term do not explain, i.e. the noise plus harmonics at
    that frequency - a numerically stable noise estimate (``total - signal``
    cancels catastrophically at high SNR).
    """

    if frequency_hz <= 0:
        return 0.0, 0.0
    signal_values = np.asarray(values, dtype=np.float64)
    sample_count = signal_values.size
    if sample_count == 0:
        return 0.0, 0.0

    time_s = np.arange(sample_count, dtype=np.float64) / float(sample_rate)
    omega_t = 2.0 * np.pi * frequency_hz * time_s
    design = np.column_stack((np.sin(omega_t), np.cos(omega_t), np.ones(sample_count, dtype=np.float64)))
    coefficients, _, _, _ = np.linalg.lstsq(design, signal_values, rcond=None)
    fitted = design @ coefficients
    residual = signal_values - fitted
    sine_coeff, cosine_coeff = float(coefficients[0]), float(coefficients[1])
    peak_amplitude = np.hypot(sine_coeff, cosine_coeff)
    return float(peak_amplitude / np.sqrt(2.0)), float(rms(residual))


def fit_sine_refined(
    values: np.ndarray,
    sample_rate: float,
    frequency_hz: float,
    search_hz: float,
) -> tuple[float, float, float]:
    """Fit a sine, refining the frequency around ``frequency_hz``.

    A parabolic FFT peak estimate is only accurate to a fraction of a bin; with a
    long window that tiny error leaves a residual proportional to the *signal*
    amplitude, which would make the noise (and therefore SNR and ENOB) look far
    worse than it is. This does a shrinking grid search for the frequency that
    minimises the least-squares residual and returns
    ``(rms, residual_rms, frequency_hz)``.
    """

    best_freq = float(frequency_hz)
    best_rms, best_residual = _fit_sine_component(values, sample_rate, best_freq)
    span = abs(float(search_hz))
    for _ in range(3):
        if span <= 0.0:
            break
        improved = False
        for step in range(-5, 6):
            if step == 0:
                continue
            candidate_freq = best_freq + span * step / 5.0
            if candidate_freq <= 0.0:
                continue
            candidate_rms, candidate_residual = _fit_sine_component(values, sample_rate, candidate_freq)
            if candidate_residual < best_residual:
                best_freq, best_rms, best_residual = candidate_freq, candidate_rms, candidate_residual
                improved = True
        span *= 0.25 if improved else 0.1
    return best_rms, best_residual, best_freq


def _fit_sine_component_rms(values: np.ndarray, sample_rate: float, frequency_hz: float) -> float:
    if frequency_hz <= 0:
        return 0.0
    signal_values = np.asarray(values, dtype=np.float64)
    sample_count = signal_values.size
    if sample_count == 0:
        return 0.0

    time_s = np.arange(sample_count, dtype=np.float64) / float(sample_rate)
    omega_t = 2.0 * np.pi * frequency_hz * time_s
    design = np.column_stack((np.sin(omega_t), np.cos(omega_t), np.ones(sample_count, dtype=np.float64)))
    coefficients, _, _, _ = np.linalg.lstsq(design, signal_values, rcond=None)
    sine_coeff, cosine_coeff = float(coefficients[0]), float(coefficients[1])
    peak_amplitude = np.hypot(sine_coeff, cosine_coeff)
    return float(peak_amplitude / np.sqrt(2.0))


def _refine_peak_frequency(frequencies_hz: np.ndarray, amplitude_rms: np.ndarray, peak_index: int) -> float:
    if peak_index <= 0 or peak_index >= len(amplitude_rms) - 1:
        return float(frequencies_hz[peak_index])

    left = amplitude_rms[peak_index - 1]
    center = amplitude_rms[peak_index]
    right = amplitude_rms[peak_index + 1]
    denominator = left - 2.0 * center + right
    if abs(denominator) < 1e-18:
        return float(frequencies_hz[peak_index])

    bin_offset = 0.5 * (left - right) / denominator
    bin_spacing = float(frequencies_hz[1] - frequencies_hz[0]) if len(frequencies_hz) > 1 else 0.0
    return float(frequencies_hz[peak_index] + bin_offset * bin_spacing)


def _prepare_fft_segment(values: np.ndarray, fft_length: int) -> np.ndarray:
    if fft_length <= 0:
        raise ValueError("FFT length must be positive.")
    if values.size == fft_length:
        return values
    if values.size > fft_length:
        start_index = (values.size - fft_length) // 2
        return values[start_index : start_index + fft_length]

    padded = np.zeros(fft_length, dtype=np.float64)
    start_index = (fft_length - values.size) // 2
    padded[start_index : start_index + values.size] = values
    return padded


def _band_radius_for_window(window_name: str) -> int:
    if window_name in {"Hann", "Hamming", "Blackman"}:
        return 1
    return 0


def _band_indices(center_index: int, radius: int, max_length: int) -> np.ndarray:
    start_index = max(center_index - radius, 0)
    end_index = min(center_index + radius + 1, max_length)
    return np.arange(start_index, end_index, dtype=int)


def _power_ratio_db(signal_power: float, noise_power: float) -> float:
    if signal_power <= 0:
        return float("-inf")
    if noise_power <= 0:
        return float("inf")
    return float(10.0 * np.log10(signal_power / noise_power))


def spectrum_peak_indices(
    frequencies_hz: np.ndarray,
    amplitude_rms: np.ndarray,
) -> list[int]:
    """Indices of the local maxima of a one-sided spectrum.

    Sweep analysis needs *candidates*, not just the global maximum: when the tone
    is small, the largest peak is often an intermodulation product rather than the
    tone itself.
    """

    amplitudes = np.asarray(amplitude_rms, dtype=np.float64)
    if amplitudes.size == 0:
        return []
    if amplitudes.size < 3:
        return [int(np.argmax(amplitudes))]
    middle = amplitudes[1:-1]
    is_peak = (middle > amplitudes[:-2]) & (middle >= amplitudes[2:]) & (middle > 0)
    indices = [int(position + 1) for position in np.flatnonzero(is_peak)]
    if not indices:
        indices = [int(np.argmax(amplitudes))]
    return indices


class SweepFundamentalTracker:
    """Follow a stepped, downward frequency sweep without being fooled by mixing.

    A bench CMRR/crosstalk sweep walks the generator down from ~110 Hz to a few
    hertz and holds each step for a second or more. Below roughly 70 Hz the
    differential signal is so small that the *intermodulation products* of the
    applied tone with the mains are louder than the tone itself: 15 Hz produces
    strong lines at 85 and 115 Hz, 30 Hz at 70 and 130 Hz. Picking the strongest
    peak then reports a frequency the generator was never set to.

    Three facts about a real sweep make the tone identifiable:

    * it only ever moves **down** between steps, while a mixing product of the
      current tone sits at ``n * 50 +/- f`` - at or above it, never below the
      previous step by much;
    * the previous tone **disappears** when the generator steps away, so a peak
      that is still at the old frequency at full strength means "not moved yet";
    * a step is only real once **several consecutive windows agree**, which is the
      same minimum-hold rule the sweep already uses to reject transitions.

    So: never accept a frequency above the last committed one, hold the committed
    frequency while it is still present, and only move the lock after
    ``windows_needed`` agreeing windows.
    """

    def __init__(
        self,
        window_boundary_hz: float = 0.5,
        windows_needed: int = 2,
        release_drop_db: float = 6.0,
        downward_only: bool = True,
        enabled: bool = True,
        mains_guard_hz: float = 2.0,
    ) -> None:
        self.window_boundary_hz = max(float(window_boundary_hz), 1e-3)
        self.windows_needed = max(int(windows_needed), 1)
        self.release_ratio = float(10.0 ** (-abs(float(release_drop_db)) / 20.0))
        self.downward_only = bool(downward_only)
        self.enabled = bool(enabled)
        self.mains_guard_hz = max(float(mains_guard_hz), 0.0)
        self.locked_hz: Optional[float] = None
        self.locked_amplitude = 0.0
        self.rejected_upward = 0
        self._pending: list[float] = []

    def choose(self, frequencies_hz: np.ndarray, amplitude_rms: np.ndarray) -> Optional[int]:
        """Index of the candidate peak that should be treated as the fundamental."""

        peaks = spectrum_peak_indices(frequencies_hz, amplitude_rms)
        if not peaks:
            return None
        peaks = self._without_mains(frequencies_hz, peaks)
        if not peaks:
            # Every candidate sits on a mains multiple, so there is no evidence of
            # the applied tone in this window; rejecting is honest.
            return None
        if not self.enabled or self.locked_hz is None:
            return int(max(peaks, key=lambda index: amplitude_rms[index]))

        allowed = peaks
        if self.downward_only:
            allowed = [
                index
                for index in peaks
                if frequencies_hz[index] <= self.locked_hz + self.window_boundary_hz
            ]
            self.rejected_upward += len(peaks) - len(allowed)
            if not allowed:
                return None

        if self.locked_hz is not None:
            held = [
                index
                for index in allowed
                if abs(frequencies_hz[index] - self.locked_hz) <= self.window_boundary_hz
            ]
            if held:
                best = int(max(held, key=lambda index: amplitude_rms[index]))
                # Still there at the level it had when it was committed: the
                # generator has not stepped yet, so keep reporting the same tone.
                if amplitude_rms[best] >= self.locked_amplitude * self.release_ratio:
                    return best
        return int(max(allowed, key=lambda index: amplitude_rms[index]))

    def _without_mains(self, frequencies_hz: np.ndarray, peaks: list[int]) -> list[int]:
        """Drop peaks sitting on a mains multiple when a better candidate exists.

        The generator is never set to exactly 50 or 100 Hz during a CMRR sweep -
        it would be indistinguishable from the mains it is measured against - so a
        peak there is pickup. Leaving it in is what made the tracker stop at
        100 Hz on its way down from 110 Hz.
        """

        if self.mains_guard_hz <= 0:
            return peaks
        return [
            index
            for index in peaks
            if all(abs(frequencies_hz[index] - mains) > self.mains_guard_hz for mains in MAINS_EXCLUDE_HZ[:2])
        ]

    def observe(self, frequency_hz: float, amplitude: float) -> Optional[float]:
        """Feed one accepted window; return the frequency if the lock moved."""

        if not self.enabled:
            return None
        if frequency_hz <= 0:
            return None
        if self._pending and abs(frequency_hz - float(np.median(self._pending))) <= self.window_boundary_hz:
            self._pending.append(float(frequency_hz))
        else:
            self._pending = [float(frequency_hz)]
        if len(self._pending) < self.windows_needed:
            return None
        committed = float(np.median(self._pending))
        if self.locked_hz is not None and abs(committed - self.locked_hz) <= self.window_boundary_hz:
            # Same step as before: refresh the reference level and stay put.
            self.locked_amplitude = float(amplitude)
            return None
        if self.downward_only and self.locked_hz is not None and committed > self.locked_hz + self.window_boundary_hz:
            return None
        self.locked_hz = committed
        self.locked_amplitude = float(amplitude)
        self._pending = []
        return committed


CMRR_SWEEP_COLUMNS = [
    "Frequency (Hz)",
    "Vcm Used (Vrms)",
    "CMRR Measured (dB)",
    "ADC Gain Corr (dB)",
    "CMRR (dB)",
    "CMRR Std (dB)",
    "Output RMS (uVrms)",
    "Windows",
]

MAINS_EXCLUDE_HZ = (50.0, 100.0)


def sweep_windows_needed(min_hold_s: float, hop_seconds: float, minimum: int = 2) -> int:
    """Windows that must share one frequency before it counts as a sweep point.

    A stepped sweep holds each frequency for a while before moving on, so two
    consecutive windows with the same tone are the smallest honest evidence that
    the generator was standing still rather than mid-transition.
    """

    if hop_seconds <= 0:
        return max(int(minimum), 2)
    needed = int(np.ceil(max(float(min_hold_s), 0.0) / hop_seconds - 1e-9))
    return max(int(minimum), needed, 2)


def _select_stable_sweep_windows(
    raw: pd.DataFrame,
    stability_tolerance_hz: float,
    windows_needed: int,
) -> tuple[pd.DataFrame, int]:
    """Drop windows that belong to a frequency the sweep only passed through."""

    stable_mask = np.zeros(len(raw), dtype=bool)
    rejected = 0
    run_start = 0
    while run_start < len(raw):
        run_end = run_start + 1
        run_frequencies = [float(raw.iloc[run_start]["Frequency (Hz)"])]
        while run_end < len(raw):
            candidate_frequency = float(raw.iloc[run_end]["Frequency (Hz)"])
            run_center = float(np.median(run_frequencies))
            if abs(candidate_frequency - run_center) > stability_tolerance_hz:
                break
            run_frequencies.append(candidate_frequency)
            run_end += 1
        if run_end - run_start >= windows_needed:
            stable_mask[run_start:run_end] = True
        else:
            rejected += run_end - run_start
        run_start = run_end
    return raw.loc[stable_mask].copy(), rejected


def _aggregate_sweep_windows(
    raw: pd.DataFrame,
    *,
    columns: Sequence[str],
    mean_columns: Sequence[str],
    value_column: str,
    std_column: str,
    stability_tolerance_hz: float,
    windows_needed: int,
    exclude_mains: bool,
    mains_tolerance_hz: float = 1.0,
    bucket_hz: Optional[float] = None,
) -> tuple[pd.DataFrame, int, int]:
    """Average the stable windows into one row per swept frequency."""

    stable, rejected = _select_stable_sweep_windows(raw, stability_tolerance_hz, windows_needed)
    if stable.empty:
        return pd.DataFrame(columns=columns), 0, rejected

    bucket_hz = max(bucket_hz or 0.0, stability_tolerance_hz, 0.05)
    stable["Frequency Bucket"] = (stable["Frequency (Hz)"] / bucket_hz).round() * bucket_hz
    aggregation = {column: "mean" for column in mean_columns}
    aggregation["Time (s)"] = "count"
    aggregated = stable.groupby("Frequency Bucket", as_index=False).agg(aggregation)
    aggregated = aggregated.rename(columns={"Time (s)": "Windows"})
    spread = stable.groupby("Frequency Bucket")[value_column].std(ddof=0).reset_index(name=std_column)
    aggregated = aggregated.merge(spread, on="Frequency Bucket", how="left")
    aggregated[std_column] = aggregated[std_column].fillna(0.0)
    aggregated = aggregated.drop(columns=["Frequency Bucket"])

    if exclude_mains:
        # 50 and 100 Hz are almost always the generator plus mains pickup, so a
        # sweep point there measures the mains, not the front end.
        keep = np.ones(len(aggregated), dtype=bool)
        for mains_hz in MAINS_EXCLUDE_HZ:
            keep &= np.abs(aggregated["Frequency (Hz)"].to_numpy() - mains_hz) > mains_tolerance_hz
        aggregated = aggregated.loc[keep]

    aggregated = aggregated[list(columns)].sort_values("Frequency (Hz)").reset_index(drop=True)
    return aggregated, int(len(stable.index)), rejected


# Maps a frequency in Hz to the common-mode voltage actually applied there, in
# Vrms. A bias reference curve supplies one of these; ``None`` means "use the
# single amplitude the user entered".
CommonModeLookup = Callable[[Sequence[float]], np.ndarray]


def resolve_common_mode_v(
    frequency_hz: float,
    scalar_v: float,
    lookup: Optional[CommonModeLookup] = None,
) -> float:
    """The common-mode voltage to use at one frequency.

    Without a lookup this is the plain scalar the user typed. With one - the bias
    reference curve - the voltage is read off the fitted ``Vcm(f)`` instead, which
    is the whole point of that curve: the applied amplitude is not flat with
    frequency, so a single number puts the front end's own roll-off into the CMRR.

    ``nan`` means "the curve does not cover this frequency", and the caller drops
    the point rather than silently substituting the nominal amplitude.
    """

    if lookup is None:
        return float(scalar_v)
    try:
        resolved = np.asarray(lookup([float(frequency_hz)]), dtype=np.float64).ravel()
    except (TypeError, ValueError):
        return float("nan")
    if resolved.size == 0:
        return float("nan")
    return float(resolved[0])


def compute_cmrr_sweep(
    values: np.ndarray,
    sample_rate: float,
    common_mode_voltage: float,
    fft_length: int,
    window_name: str,
    signal_scale_to_volts: float = 1e-6,
    correction_enabled: bool = False,
    correction_fs_hz: float = 250.0,
    window_duration_s: float = 0.75,
    hop_fraction: float = 0.5,
    min_frequency_hz: float = 1.0,
    min_output_rms_uv: float = 0.1,
    stability_tolerance_hz: float = 0.2,
    min_stable_run_windows: int = 2,
    min_local_snr_db: float = -20.0,
    min_hold_s: float = 0.0,
    exclude_mains: bool = False,
    mains_tolerance_hz: float = 1.0,
    downward_only: bool = True,
    lock_tolerance_hz: float = 0.5,
    lock_release_db: float = 6.0,
    common_mode_lookup: Optional[CommonModeLookup] = None,
) -> CMRRSweepResult:
    """Estimate CMRR across frequency for a swept-tone selection.

    The selected region is split into overlapping windows. For each window, the
    dominant positive-frequency tone is detected and its exact-frequency RMS is
    estimated with a sine fit. CMRR is then computed against the user-entered
    common-mode amplitude.

    ``min_hold_s`` is how long the generator has to hold a frequency before that
    step counts: at least two consecutive windows must agree on the tone, which
    is what rejects the sliding transition between two steps.

    ``downward_only`` follows the usual bench order (high frequency first, then
    stepping down). With it enabled the fundamental can never move *up*, so the
    intermodulation products of the tone with the mains - which sit at
    ``100 +/- f`` and are louder than the tone itself at low frequency - are not
    mistaken for the applied signal. ``lock_tolerance_hz`` is how far two windows
    may differ and still count as the same step, and ``lock_release_db`` is how
    far the previously committed tone has to fall before the tracker accepts that
    the generator has moved on.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    if window_duration_s <= 0:
        window_duration_s = 0.75
    if stability_tolerance_hz <= 0:
        stability_tolerance_hz = 0.2

    window_size = max(int(round(window_duration_s * sample_rate)), 16)
    if signal_values.size < window_size:
        empty = pd.DataFrame(columns=CMRR_SWEEP_COLUMNS)
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)
    if common_mode_voltage <= 0 and common_mode_lookup is None:
        empty = pd.DataFrame(columns=CMRR_SWEEP_COLUMNS)
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    # Sweep mode always uses notch cleanup to suppress fixed mains pickup.
    filtered_values = apply_filters(
        signal_values[:, np.newaxis],
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )[:, 0]

    zero_pad_length = max(int(fft_length), 8 * window_size)
    hop_size = max(int(window_size * hop_fraction), 1)
    tracker = SweepFundamentalTracker(
        window_boundary_hz=lock_tolerance_hz,
        windows_needed=sweep_windows_needed(min_hold_s, hop_size / float(sample_rate), min_stable_run_windows),
        release_drop_db=lock_release_db,
        downward_only=downward_only,
    )
    raw_rows: list[dict[str, float]] = []
    rejected_windows = 0
    for start_index in range(0, signal_values.size - window_size + 1, hop_size):
        segment = filtered_values[start_index : start_index + window_size]
        centered = segment - np.mean(segment)
        prepared = _prepare_fft_segment(centered, zero_pad_length)
        frequencies_hz, amplitude_rms = _compute_rms_spectrum_from_segment(prepared, sample_rate, window_name)
        positive_mask = frequencies_hz >= min_frequency_hz
        if not np.any(positive_mask):
            rejected_windows += 1
            continue
        masked_frequencies = frequencies_hz[positive_mask]
        masked_amplitudes = amplitude_rms[positive_mask]
        relative_index = tracker.choose(masked_frequencies, masked_amplitudes)
        if relative_index is None:
            rejected_windows += 1
            continue
        original_indices = np.flatnonzero(positive_mask)
        peak_index = int(original_indices[relative_index])
        frequency_hz = _refine_peak_frequency(frequencies_hz, amplitude_rms, peak_index)
        if frequency_hz <= 0:
            rejected_windows += 1
            continue
        output_rms = _fit_sine_component_rms(centered, sample_rate, frequency_hz)
        if output_rms <= min_output_rms_uv:
            rejected_windows += 1
            continue

        total_ac_power = float(rms(centered) ** 2)
        tone_power = float(output_rms**2)
        residual_power = max(total_ac_power - tone_power, 0.0)
        local_snr_db = _power_ratio_db(tone_power, residual_power)
        if local_snr_db < min_local_snr_db:
            rejected_windows += 1
            continue
        output_volts = output_rms * signal_scale_to_volts
        common_mode_here = resolve_common_mode_v(frequency_hz, common_mode_voltage, common_mode_lookup)
        if not np.isfinite(common_mode_here) or common_mode_here <= 0:
            # The reference curve does not reach this frequency, so the applied
            # amplitude is unknown here. Guessing it would fabricate the CMRR.
            rejected_windows += 1
            continue
        cmrr_measured_db = 20.0 * np.log10(common_mode_here / output_volts) if output_volts > 0 else float("inf")
        correction_gain_db = float(
            compute_cmrr_correction_gain_db(
                [frequency_hz],
                correction_enabled=correction_enabled,
                correction_fs_hz=correction_fs_hz,
            )[0]
        )
        cmrr_db = cmrr_measured_db + correction_gain_db
        tracker.observe(frequency_hz, output_rms)
        center_time_s = (start_index + 0.5 * window_size) / float(sample_rate)
        raw_rows.append(
            {
                "Time (s)": center_time_s,
                "Frequency (Hz)": frequency_hz,
                "Vcm Used (Vrms)": common_mode_here,
                "Output RMS (uVrms)": output_rms,
                "CMRR Measured (dB)": cmrr_measured_db,
                "ADC Gain Corr (dB)": correction_gain_db,
                "CMRR (dB)": cmrr_db,
                "Local SNR (dB)": local_snr_db,
            }
        )

    raw_dataframe = pd.DataFrame(raw_rows)
    if raw_dataframe.empty:
        empty = pd.DataFrame(columns=CMRR_SWEEP_COLUMNS)
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected_windows)

    aggregated, accepted, unstable = _aggregate_sweep_windows(
        raw_dataframe,
        columns=CMRR_SWEEP_COLUMNS,
        mean_columns=(
            "Frequency (Hz)",
            "Vcm Used (Vrms)",
            "CMRR Measured (dB)",
            "ADC Gain Corr (dB)",
            "CMRR (dB)",
            "Output RMS (uVrms)",
        ),
        value_column="CMRR (dB)",
        std_column="CMRR Std (dB)",
        stability_tolerance_hz=stability_tolerance_hz,
        windows_needed=sweep_windows_needed(min_hold_s, hop_size / float(sample_rate), min_stable_run_windows),
        exclude_mains=exclude_mains,
        mains_tolerance_hz=mains_tolerance_hz,
        bucket_hz=lock_tolerance_hz,
    )
    return CMRRSweepResult(
        dataframe=aggregated,
        accepted_windows=accepted,
        rejected_windows=int(rejected_windows + unstable),
    )


def compute_crosstalk_sweep(
    source_values: np.ndarray,
    target_values: np.ndarray,
    sample_rate: float,
    fft_length: int,
    window_name: str,
    window_duration_s: float = 0.75,
    hop_fraction: float = 0.5,
    min_frequency_hz: float = 1.0,
    min_source_rms_uv: float = 0.1,
    stability_tolerance_hz: float = 0.2,
    min_stable_run_windows: int = 2,
    min_local_snr_db: float = -20.0,
    min_hold_s: float = 0.0,
    exclude_mains: bool = False,
    mains_tolerance_hz: float = 1.0,
    downward_only: bool = True,
    lock_tolerance_hz: float = 0.5,
    lock_release_db: float = 6.0,
) -> CrosstalkSweepResult:
    """Estimate crosstalk across frequency for a swept-tone selection.

    The source channel is used to detect stable sweep windows and frequencies.
    For each accepted stable window, the target channel RMS is measured at the
    same exact frequency and reported relative to the source channel RMS.
    """

    columns = [
        "Frequency (Hz)",
        "Crosstalk (dB)",
        "Crosstalk Std (dB)",
        "Source RMS (uVrms)",
        "Target RMS (uVrms)",
        "Windows",
    ]
    source_signal = np.asarray(source_values, dtype=np.float64)
    target_signal = np.asarray(target_values, dtype=np.float64)
    if source_signal.size != target_signal.size:
        raise ValueError("Source and target signals must have the same sample count.")
    if window_duration_s <= 0:
        window_duration_s = 0.75
    if stability_tolerance_hz <= 0:
        stability_tolerance_hz = 0.2

    window_size = max(int(round(window_duration_s * sample_rate)), 16)
    if source_signal.size < window_size:
        empty = pd.DataFrame(columns=columns)
        return CrosstalkSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    filtered_matrix = apply_filters(
        np.column_stack((source_signal, target_signal)),
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )
    filtered_source = filtered_matrix[:, 0]
    filtered_target = filtered_matrix[:, 1]

    zero_pad_length = max(int(fft_length), 8 * window_size)
    hop_size = max(int(window_size * hop_fraction), 1)
    tracker = SweepFundamentalTracker(
        window_boundary_hz=lock_tolerance_hz,
        windows_needed=sweep_windows_needed(min_hold_s, hop_size / float(sample_rate), min_stable_run_windows),
        release_drop_db=lock_release_db,
        downward_only=downward_only,
    )
    raw_rows: list[dict[str, float]] = []
    rejected_windows = 0
    for start_index in range(0, source_signal.size - window_size + 1, hop_size):
        source_segment = filtered_source[start_index : start_index + window_size]
        target_segment = filtered_target[start_index : start_index + window_size]
        source_centered = source_segment - np.mean(source_segment)
        prepared = _prepare_fft_segment(source_centered, zero_pad_length)
        frequencies_hz, amplitude_rms = _compute_rms_spectrum_from_segment(prepared, sample_rate, window_name)
        positive_mask = frequencies_hz >= min_frequency_hz
        if not np.any(positive_mask):
            rejected_windows += 1
            continue
        masked_frequencies = frequencies_hz[positive_mask]
        masked_amplitudes = amplitude_rms[positive_mask]
        relative_index = tracker.choose(masked_frequencies, masked_amplitudes)
        if relative_index is None:
            rejected_windows += 1
            continue
        original_indices = np.flatnonzero(positive_mask)
        peak_index = int(original_indices[relative_index])
        frequency_hz = _refine_peak_frequency(frequencies_hz, amplitude_rms, peak_index)
        if frequency_hz <= 0:
            rejected_windows += 1
            continue

        source_rms = _fit_sine_component_rms(source_centered, sample_rate, frequency_hz)
        if source_rms <= min_source_rms_uv:
            rejected_windows += 1
            continue

        total_ac_power = float(rms(source_centered) ** 2)
        tone_power = float(source_rms**2)
        residual_power = max(total_ac_power - tone_power, 0.0)
        local_snr_db = _power_ratio_db(tone_power, residual_power)
        if local_snr_db < min_local_snr_db:
            rejected_windows += 1
            continue

        tracker.observe(frequency_hz, source_rms)
        target_centered = target_segment - np.mean(target_segment)
        target_rms = _fit_sine_component_rms(target_centered, sample_rate, frequency_hz)
        crosstalk_db = 20.0 * np.log10(target_rms / source_rms) if target_rms > 0 else float("-inf")
        center_time_s = (start_index + 0.5 * window_size) / float(sample_rate)
        raw_rows.append(
            {
                "Time (s)": center_time_s,
                "Frequency (Hz)": frequency_hz,
                "Source RMS (uVrms)": source_rms,
                "Target RMS (uVrms)": target_rms,
                "Crosstalk (dB)": crosstalk_db,
                "Local SNR (dB)": local_snr_db,
            }
        )

    raw_dataframe = pd.DataFrame(raw_rows)
    if raw_dataframe.empty:
        empty = pd.DataFrame(columns=columns)
        return CrosstalkSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected_windows)

    aggregated, accepted, unstable = _aggregate_sweep_windows(
        raw_dataframe,
        columns=columns,
        mean_columns=("Frequency (Hz)", "Crosstalk (dB)", "Source RMS (uVrms)", "Target RMS (uVrms)"),
        value_column="Crosstalk (dB)",
        std_column="Crosstalk Std (dB)",
        stability_tolerance_hz=stability_tolerance_hz,
        windows_needed=sweep_windows_needed(min_hold_s, hop_size / float(sample_rate), min_stable_run_windows),
        exclude_mains=exclude_mains,
        mains_tolerance_hz=mains_tolerance_hz,
        bucket_hz=lock_tolerance_hz,
    )
    return CrosstalkSweepResult(
        dataframe=aggregated,
        accepted_windows=accepted,
        rejected_windows=int(rejected_windows + unstable),
    )


def compute_gain_sweep(
    values: np.ndarray,
    sample_rate: float,
    input_before_attenuator_v: float,
    fft_length: int,
    window_name: str,
    attenuator_ratio: float = DEFAULT_INPUT_ATTENUATOR_RATIO,
    signal_scale_to_volts: float = 1e-6,
    window_duration_s: float = 0.75,
    hop_fraction: float = 0.5,
    min_frequency_hz: float = 1.0,
    min_output_rms_uv: float = 0.1,
    stability_tolerance_hz: float = 0.2,
    min_stable_run_windows: int = 2,
    min_local_snr_db: float = -20.0,
    min_hold_s: float = 0.0,
    exclude_mains: bool = False,
    mains_tolerance_hz: float = 1.0,
    downward_only: bool = True,
    lock_tolerance_hz: float = 0.5,
    lock_release_db: float = 6.0,
) -> GainSweepResult:
    """Estimate channel gain across frequency for a swept-tone selection.

    The user supplies the source amplitude before the fixed attenuator. The
    effective analyzer input is derived as:

        input_after_attenuator = input_before_attenuator * attenuator_ratio

    Gain is then reported as output / input_after_attenuator.
    """

    columns = [
        "Frequency (Hz)",
        "Gain (V/V)",
        "Gain (dB)",
        "Gain Std (dB)",
        "Applied (Vrms)",
        "At Amplifier Input (uVrms)",
        "Output RMS (uVrms)",
        "Windows",
    ]
    if input_before_attenuator_v <= 0 or attenuator_ratio <= 0:
        empty = pd.DataFrame(columns=columns)
        return GainSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    input_after_attenuator_v = float(input_before_attenuator_v) * float(attenuator_ratio)
    cmrr_like = compute_cmrr_sweep(
        values=values,
        sample_rate=sample_rate,
        common_mode_voltage=input_after_attenuator_v,
        fft_length=fft_length,
        window_name=window_name,
        signal_scale_to_volts=signal_scale_to_volts,
        window_duration_s=window_duration_s,
        hop_fraction=hop_fraction,
        min_frequency_hz=min_frequency_hz,
        min_output_rms_uv=min_output_rms_uv,
        stability_tolerance_hz=stability_tolerance_hz,
        min_stable_run_windows=min_stable_run_windows,
        min_local_snr_db=min_local_snr_db,
        min_hold_s=min_hold_s,
        exclude_mains=exclude_mains,
        mains_tolerance_hz=mains_tolerance_hz,
        downward_only=downward_only,
        lock_tolerance_hz=lock_tolerance_hz,
        lock_release_db=lock_release_db,
    )
    if cmrr_like.dataframe.empty:
        empty = pd.DataFrame(columns=columns)
        return GainSweepResult(
            dataframe=empty,
            accepted_windows=cmrr_like.accepted_windows,
            rejected_windows=cmrr_like.rejected_windows,
        )

    gain_dataframe = pd.DataFrame(
        {
            "Frequency (Hz)": cmrr_like.dataframe["Frequency (Hz)"],
            "Gain (V/V)": (cmrr_like.dataframe["Output RMS (uVrms)"] * signal_scale_to_volts) / input_after_attenuator_v,
            "Gain (dB)": -cmrr_like.dataframe["CMRR (dB)"],
            "Gain Std (dB)": cmrr_like.dataframe["CMRR Std (dB)"],
            "Applied (Vrms)": float(input_before_attenuator_v),
            "At Amplifier Input (uVrms)": input_after_attenuator_v / signal_scale_to_volts,
            "Output RMS (uVrms)": cmrr_like.dataframe["Output RMS (uVrms)"],
            "Windows": cmrr_like.dataframe["Windows"],
        }
    )
    gain_dataframe = gain_dataframe[columns].sort_values("Frequency (Hz)").reset_index(drop=True)
    return GainSweepResult(
        dataframe=gain_dataframe,
        accepted_windows=cmrr_like.accepted_windows,
        rejected_windows=cmrr_like.rejected_windows,
    )
# ---------------------------------------------------------------------------
# Planned (generator-synchronised) sweeps
#
# When the live window drives the function generator itself it knows the exact
# frequency *and* the exact time each step was applied, so nothing has to be
# guessed from the spectrum. That removes the whole problem the peak tracker had
# below ~70 Hz, where the intermodulation products of the tone with the mains are
# louder than the tone itself: the analyser simply looks at the frequency the
# generator was told to produce, inside the time window that step occupied.
# ---------------------------------------------------------------------------

PLANNED_SWEEP_COLUMNS = [
    "Frequency (Hz)",
    "Output RMS (uVrms)",
    "Windows",
    "Applied Vpp",
]

_SWEEP_STEP_PATTERN = re.compile(r"^Sweep Step (\d+)$")


@dataclass(slots=True)
class PlannedSweepStep:
    """One generator step as written into the recording header."""

    frequency_hz: float
    # Applied sine amplitude in Vrms (recordings older than the RMS change carry
    # Vpp here; parse_sweep_steps converts those using the metadata key).
    amplitude_vrms: float
    start_s: float
    settle_s: float
    end_s: float

    @property
    def usable_start_s(self) -> float:
        return max(self.settle_s, self.start_s)


def parse_sweep_steps(metadata: Mapping[str, str]) -> list[PlannedSweepStep]:
    """Recover the sweep schedule that the live window wrote into the header.

    The format is ``frequency|amplitude|start|settle|end`` in Hz, Vrms and
    seconds. Older recordings stored Vpp under ``Sweep Amplitude (Vpp)``; in that
    case the per-step amplitude is converted so it keeps the same meaning.
    """

    legacy_vpp = str((metadata or {}).get("Sweep Amplitude (Vpp)", "") or "").strip()
    convert = bool(legacy_vpp) and not str((metadata or {}).get("Sweep Amplitude (Vrms)", "") or "").strip()
    steps: list[tuple[int, PlannedSweepStep]] = []
    for key, value in (metadata or {}).items():
        match = _SWEEP_STEP_PATTERN.match(str(key).strip())
        if not match:
            continue
        fields = str(value).split("|")
        if len(fields) < 5:
            continue
        try:
            amplitude = float(fields[1])
            if convert:
                amplitude /= 2.0 * np.sqrt(2.0)
            step = PlannedSweepStep(
                frequency_hz=float(fields[0]),
                amplitude_vrms=amplitude,
                start_s=float(fields[2]),
                settle_s=float(fields[3]),
                end_s=float(fields[4]),
            )
        except ValueError:
            continue
        if step.frequency_hz > 0 and step.end_s > step.usable_start_s:
            steps.append((int(match.group(1)), step))
    steps.sort(key=lambda item: item[0])
    return [step for _index, step in steps]


def _segment_indices(step: PlannedSweepStep, sample_rate: float, total: int) -> tuple[int, int]:
    start = int(round(step.usable_start_s * sample_rate))
    end = int(round(step.end_s * sample_rate))
    return max(start, 0), min(end, total)


def compute_planned_cmrr_sweep(
    values: np.ndarray,
    sample_rate: float,
    steps: Sequence[PlannedSweepStep],
    *,
    common_mode_v_rms: float,
    signal_scale_to_volts: float = 1e-6,
    correction_enabled: bool = False,
    correction_fs_hz: float = 250.0,
    guard_fraction: float = 0.15,
    max_windows_per_step: int = 8,
    min_output_rms_uv: float = 0.02,
    common_mode_lookup: Optional[CommonModeLookup] = None,
) -> CMRRSweepResult:
    """CMRR per generator step, using the frequency the generator was set to.

    Each step is cut into as many whole windows as fit between the settled start
    and the end of the hold, so even a 1 s hold at 5 Hz produces a usable point
    without any peak picking.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    empty = pd.DataFrame(columns=CMRR_SWEEP_COLUMNS)
    if signal_values.size == 0 or sample_rate <= 0:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)
    if common_mode_v_rms <= 0 and common_mode_lookup is None:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    filtered = apply_filters(
        signal_values[:, np.newaxis],
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )[:, 0]

    rows: list[dict[str, float]] = []
    accepted = 0
    rejected = 0
    for step in steps:
        start_index, end_index = _segment_indices(step, sample_rate, signal_values.size)
        span = end_index - start_index
        if span < max(int(0.05 * sample_rate), 16):
            rejected += 1
            continue
        guard = int(round(span * max(min(guard_fraction, 0.45), 0.0)))
        usable = span - guard
        # At least two cycles per window, so the sine fit is not dominated by the
        # trend; a 5 Hz step held for one second still gets a usable window.
        min_window = max(int(round(max(0.15, 2.0 / max(step.frequency_hz, 1e-6)) * sample_rate)), 16)
        windows = int(min(max_windows_per_step, max(1, usable // min_window)))
        window_size = max(usable // windows, 16)
        step_rms: list[float] = []
        for index in range(windows):
            window_start = start_index + guard + index * window_size
            window_end = window_start + window_size
            if window_end > end_index:
                break
            segment = filtered[window_start:window_end]
            centered = segment - np.mean(segment)
            rms_uv = _fit_sine_component_rms(centered, sample_rate, step.frequency_hz)
            if rms_uv <= 0:
                continue
            step_rms.append(float(rms_uv))
        if not step_rms:
            rejected += 1
            continue
        mean_rms_uv = float(np.mean(step_rms))
        std_rms_uv = float(np.std(step_rms, ddof=0))
        if mean_rms_uv < min_output_rms_uv:
            # The tone is below the analyser floor; reporting a CMRR here would be
            # reporting the noise of the front end.
            rejected += 1
            continue
        output_volts = mean_rms_uv * signal_scale_to_volts
        common_mode_here = resolve_common_mode_v(
            float(step.frequency_hz), common_mode_v_rms, common_mode_lookup
        )
        if not np.isfinite(common_mode_here) or common_mode_here <= 0:
            rejected += 1
            continue
        cmrr_measured_db = 20.0 * np.log10(common_mode_here / output_volts) if output_volts > 0 else float("inf")
        correction_gain_db = float(
            compute_cmrr_correction_gain_db(
                [step.frequency_hz],
                correction_enabled=correction_enabled,
                correction_fs_hz=correction_fs_hz,
            )[0]
        )
        std_db = (
            20.0 * np.log10(mean_rms_uv / (mean_rms_uv - std_rms_uv))
            if mean_rms_uv > std_rms_uv > 0
            else 0.0
        )
        accepted += 1
        rows.append(
            {
                "Frequency (Hz)": float(step.frequency_hz),
                "Vcm Used (Vrms)": float(common_mode_here),
                "Output RMS (uVrms)": mean_rms_uv,
                "CMRR Measured (dB)": cmrr_measured_db,
                "ADC Gain Corr (dB)": correction_gain_db,
                "CMRR (dB)": cmrr_measured_db + correction_gain_db,
                "CMRR Std (dB)": float(std_db),
                "Windows": float(len(step_rms)),
            }
        )

    if not rows:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected)

    frame = pd.DataFrame(rows)
    frame = frame[CMRR_SWEEP_COLUMNS].sort_values("Frequency (Hz)").reset_index(drop=True)
    return CMRRSweepResult(dataframe=frame, accepted_windows=accepted, rejected_windows=rejected)


def fit_tone_rms(values: np.ndarray, sample_rate: float, frequency_hz: float) -> float:
    """Public wrapper: least-squares sine amplitude at an exact frequency.

    Used to read the reference channel's real amplitude at the frequency the
    generator was set to, which is what a "measured versus applied" figure needs.
    """

    return _fit_sine_component_rms(np.asarray(values, dtype=np.float64), sample_rate, frequency_hz)


def detect_tone_frequency(
    values: np.ndarray,
    sample_rate: float,
    *,
    fft_length: int = 4096,
    window_name: str = "Hann",
    min_frequency_hz: float = 1.0,
    mains_guard_hz: float = 2.0,
) -> Optional[float]:
    """Dominant tone of one segment, used to read the stimulus off a monitor channel.

    A channel wired straight across the common-mode voltage sees the applied sine
    at close to a volt, so its dominant peak *is* the generator frequency and the
    intermodulation products that confuse the differential channels are irrelevant.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    if signal_values.size < 16 or sample_rate <= 0:
        return None
    centered = signal_values - np.mean(signal_values)
    prepared = _prepare_fft_segment(centered, max(int(fft_length), 4096))
    frequencies_hz, amplitude_rms = _compute_rms_spectrum_from_segment(prepared, sample_rate, window_name)
    mask = frequencies_hz >= min_frequency_hz
    if not np.any(mask):
        return None
    masked_frequencies = frequencies_hz[mask]
    masked_amplitudes = amplitude_rms[mask]
    if masked_amplitudes.size == 0 or float(np.max(masked_amplitudes)) <= 0:
        return None
    best = int(np.argmax(masked_amplitudes))
    if mains_guard_hz > 0 and any(
        abs(float(masked_frequencies[best]) - mains) <= mains_guard_hz for mains in MAINS_EXCLUDE_HZ
    ):
        # A peak sitting exactly on a mains harmonic is pickup, not the stimulus.
        order = np.argsort(masked_amplitudes)[::-1]
        for candidate in order:
            frequency = float(masked_frequencies[int(candidate)])
            if all(abs(frequency - mains) > mains_guard_hz for mains in MAINS_EXCLUDE_HZ):
                best = int(candidate)
                break
    original_indices = np.flatnonzero(mask)
    refined = _refine_peak_frequency(frequencies_hz, amplitude_rms, int(original_indices[best]))
    return refined if refined > 0 else None

def compute_reference_cmrr_sweep(
    values: np.ndarray,
    reference_values: np.ndarray,
    sample_rate: float,
    *,
    common_mode_v_rms: float,
    window_duration_s: float = 0.75,
    hop_fraction: float = 0.5,
    fft_length: int = 4096,
    window_name: str = "Hann",
    signal_scale_to_volts: float = 1e-6,
    correction_enabled: bool = False,
    correction_fs_hz: float = 250.0,
    stability_tolerance_hz: float = 0.5,
    min_stable_run_windows: int = 1,
    min_output_rms_uv: float = 0.02,
    common_mode_lookup: Optional[CommonModeLookup] = None,
) -> CMRRSweepResult:
    """CMRR where the applied frequency is read off a monitor channel.

    The monitor input is wired straight across the common-mode voltage, so it sees
    the generator at close to a volt. Its dominant peak is therefore the applied
    frequency, and the differential channels are only ever evaluated at exactly
    that frequency. This is what makes the low-frequency end of a downward sweep
    usable: there is no peak to pick on the channel under test, so the
    intermodulation products that beat the real tone at 60 Hz and below can never
    be mistaken for it.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    monitor_values = np.asarray(reference_values, dtype=np.float64)
    empty = pd.DataFrame(columns=CMRR_SWEEP_COLUMNS)
    count = min(signal_values.size, monitor_values.size)
    if count == 0 or sample_rate <= 0:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)
    if common_mode_v_rms <= 0 and common_mode_lookup is None:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)
    if window_duration_s <= 0:
        window_duration_s = 0.75

    window_size = max(int(round(window_duration_s * sample_rate)), 16)
    if count < window_size:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    filtered = apply_filters(
        signal_values[:count, np.newaxis],
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )[:, 0]
    hop_size = max(int(window_size * hop_fraction), 1)
    raw_rows: list[dict[str, float]] = []
    rejected = 0
    for start_index in range(0, count - window_size + 1, hop_size):
        monitor_segment = monitor_values[start_index : start_index + window_size]
        frequency_hz = detect_tone_frequency(
            monitor_segment,
            sample_rate,
            fft_length=max(int(fft_length), 8 * window_size),
            window_name=window_name,
        )
        if frequency_hz is None:
            rejected += 1
            continue
        segment = filtered[start_index : start_index + window_size]
        centered = segment - np.mean(segment)
        output_rms = _fit_sine_component_rms(centered, sample_rate, frequency_hz)
        if output_rms <= min_output_rms_uv:
            rejected += 1
            continue
        output_volts = output_rms * signal_scale_to_volts
        common_mode_here = resolve_common_mode_v(float(frequency_hz), common_mode_v_rms, common_mode_lookup)
        if not np.isfinite(common_mode_here) or common_mode_here <= 0:
            rejected += 1
            continue
        cmrr_measured_db = 20.0 * np.log10(common_mode_here / output_volts) if output_volts > 0 else float("inf")
        correction_gain_db = float(
            compute_cmrr_correction_gain_db(
                [frequency_hz],
                correction_enabled=correction_enabled,
                correction_fs_hz=correction_fs_hz,
            )[0]
        )
        raw_rows.append(
            {
                "Time (s)": (start_index + 0.5 * window_size) / float(sample_rate),
                "Frequency (Hz)": float(frequency_hz),
                "Vcm Used (Vrms)": float(common_mode_here),
                "Output RMS (uVrms)": float(output_rms),
                "CMRR Measured (dB)": cmrr_measured_db,
                "ADC Gain Corr (dB)": correction_gain_db,
                "CMRR (dB)": cmrr_measured_db + correction_gain_db,
            }
        )

    frame = pd.DataFrame(raw_rows)
    if frame.empty:
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected)
    aggregated, accepted, unstable = _aggregate_sweep_windows(
        frame,
        columns=CMRR_SWEEP_COLUMNS,
        mean_columns=[
            "Frequency (Hz)",
            "Vcm Used (Vrms)",
            "Output RMS (uVrms)",
            "CMRR Measured (dB)",
            "ADC Gain Corr (dB)",
            "CMRR (dB)",
        ],
        value_column="CMRR (dB)",
        std_column="CMRR Std (dB)",
        stability_tolerance_hz=stability_tolerance_hz,
        windows_needed=max(int(min_stable_run_windows), 1),
        exclude_mains=False,
    )
    return CMRRSweepResult(
        dataframe=aggregated,
        accepted_windows=accepted,
        rejected_windows=rejected + unstable,
    )

CROSSTALK_SWEEP_COLUMNS = [
    "Frequency (Hz)",
    "Crosstalk (dB)",
    "Crosstalk Std (dB)",
    "Source RMS (uVrms)",
    "Target RMS (uVrms)",
    "Windows",
]

GAIN_SWEEP_COLUMNS = [
    "Frequency (Hz)",
    "Gain (V/V)",
    "Gain (dB)",
    "Gain Std (dB)",
    "Applied (Vrms)",
    "At Amplifier Input (uVrms)",
    "Output RMS (uVrms)",
    "Windows",
]


def _step_amplitudes(
    values: np.ndarray,
    sample_rate: float,
    steps: Sequence[PlannedSweepStep],
    *,
    guard_fraction: float = 0.15,
    max_windows_per_step: int = 8,
) -> dict[float, float]:
    """Sine-fit RMS at each step frequency, averaged over the windows that fit."""

    results: dict[float, float] = {}
    for step in steps:
        start_index, end_index = _segment_indices(step, sample_rate, values.size)
        span = end_index - start_index
        if span < max(int(0.05 * sample_rate), 16):
            continue
        guard = int(round(span * max(min(guard_fraction, 0.45), 0.0)))
        usable = span - guard
        min_window = max(int(round(max(0.15, 2.0 / max(step.frequency_hz, 1e-6)) * sample_rate)), 16)
        windows = int(min(max_windows_per_step, max(1, usable // min_window)))
        window_size = max(usable // windows, 16)
        measured: list[float] = []
        for index in range(windows):
            window_start = start_index + guard + index * window_size
            window_end = window_start + window_size
            if window_end > end_index:
                break
            segment = values[window_start:window_end]
            rms_uv = _fit_sine_component_rms(segment - np.mean(segment), sample_rate, step.frequency_hz)
            if rms_uv > 0:
                measured.append(float(rms_uv))
        if measured:
            results[step.frequency_hz] = float(np.mean(measured))
    return results


def compute_planned_crosstalk_sweep(
    source_values: np.ndarray,
    target_values: np.ndarray,
    sample_rate: float,
    steps: Sequence[PlannedSweepStep],
    *,
    min_source_rms_uv: float = 0.05,
) -> CrosstalkSweepResult:
    """Crosstalk per generator step, from the schedule the live window recorded."""

    source_signal = np.asarray(source_values, dtype=np.float64)
    target_signal = np.asarray(target_values, dtype=np.float64)
    count = min(source_signal.size, target_signal.size)
    empty = pd.DataFrame(columns=CROSSTALK_SWEEP_COLUMNS)
    if count == 0 or sample_rate <= 0 or not steps:
        return CrosstalkSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    filtered = apply_filters(
        np.column_stack((source_signal[:count], target_signal[:count])),
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )
    source_map = _step_amplitudes(filtered[:, 0], sample_rate, steps)
    target_map = _step_amplitudes(filtered[:, 1], sample_rate, steps)

    rows: list[dict[str, float]] = []
    rejected = 0
    for step in steps:
        source_rms = source_map.get(step.frequency_hz)
        target_rms = target_map.get(step.frequency_hz)
        if source_rms is None or target_rms is None or source_rms < min_source_rms_uv:
            rejected += 1
            continue
        ratio = target_rms / source_rms
        rows.append(
            {
                "Frequency (Hz)": float(step.frequency_hz),
                "Crosstalk (dB)": float(20.0 * np.log10(ratio)) if ratio > 0 else float("-inf"),
                "Crosstalk Std (dB)": 0.0,
                "Source RMS (uVrms)": source_rms,
                "Target RMS (uVrms)": target_rms,
                "Windows": 1.0,
            }
        )

    if not rows:
        return CrosstalkSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected)
    frame = pd.DataFrame(rows)[CROSSTALK_SWEEP_COLUMNS].sort_values("Frequency (Hz)").reset_index(drop=True)
    return CrosstalkSweepResult(dataframe=frame, accepted_windows=len(rows), rejected_windows=rejected)

def compute_planned_gain_sweep(
    values: np.ndarray,
    sample_rate: float,
    steps: Sequence[PlannedSweepStep],
    *,
    applied_vrms: float,
    signal_scale_to_volts: float = 1e-6,
    attenuator_ratio: float = 1.0,
    guard_fraction: float = 0.15,
    max_windows_per_step: int = 8,
    min_output_rms_uv: float = 0.02,
    notch: bool = False,
) -> GainSweepResult:
    """Gain per generator step, from the schedule the live window recorded.

    ``attenuator_ratio`` is the divider between the generator and the amplifier
    input, so the reference amplitude is what actually reached the electrode.
    ``notch`` is off by default: a gain step is a ratio of two known amplitudes,
    and notching 50/100 Hz would shave the tone itself whenever the generator sits
    near those frequencies.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    empty = pd.DataFrame(columns=GAIN_SWEEP_COLUMNS)
    if signal_values.size == 0 or sample_rate <= 0 or not steps:
        return GainSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    input_after_attenuator_v = max(float(applied_vrms), 0.0) * float(attenuator_ratio)
    if applied_vrms <= 0 or input_after_attenuator_v <= 0:
        return GainSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    if notch:
        filtered = apply_filters(
            signal_values[:, np.newaxis],
            sample_rate,
            FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
        )[:, 0]
    else:
        filtered = signal_values - np.mean(signal_values)

    rows: list[dict[str, float]] = []
    accepted = 0
    rejected = 0
    for step in steps:
        start_index, end_index = _segment_indices(step, sample_rate, signal_values.size)
        span = end_index - start_index
        if span < max(int(0.05 * sample_rate), 16):
            rejected += 1
            continue
        guard = int(round(span * max(min(guard_fraction, 0.45), 0.0)))
        usable = span - guard
        min_window = max(int(round(max(0.15, 2.0 / max(step.frequency_hz, 1e-6)) * sample_rate)), 16)
        windows = int(min(max_windows_per_step, max(1, usable // min_window)))
        window_size = max(usable // windows, 16)
        measured: list[float] = []
        for index in range(windows):
            window_start = start_index + guard + index * window_size
            window_end = window_start + window_size
            if window_end > end_index:
                break
            segment = filtered[window_start:window_end]
            rms_uv = _fit_sine_component_rms(segment - np.mean(segment), sample_rate, step.frequency_hz)
            if rms_uv > 0:
                measured.append(float(rms_uv))
        if not measured:
            rejected += 1
            continue
        mean_uv = float(np.mean(measured))
        if mean_uv < min_output_rms_uv:
            rejected += 1
            continue
        measured_v = mean_uv * signal_scale_to_volts
        gain = measured_v / input_after_attenuator_v
        accepted += 1
        rows.append(
            {
                "Frequency (Hz)": float(step.frequency_hz),
                "Gain (V/V)": gain,
                "Gain (dB)": float(20.0 * np.log10(gain)) if gain > 0 else float("-inf"),
                "Gain Std (dB)": 0.0,
                "Applied (Vrms)": float(applied_vrms),
                "At Amplifier Input (uVrms)": input_after_attenuator_v / signal_scale_to_volts,
                "Output RMS (uVrms)": mean_uv,
                "Windows": float(len(measured)),
            }
        )

    if not rows:
        return GainSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected)
    frame = pd.DataFrame(rows)[GAIN_SWEEP_COLUMNS].sort_values("Frequency (Hz)").reset_index(drop=True)
    return GainSweepResult(dataframe=frame, accepted_windows=accepted, rejected_windows=rejected)
