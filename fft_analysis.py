"""Frequency-domain analysis with window-corrected RMS spectra.

The FFT scaling is designed so that a bin-centered sinusoid is reported in RMS
units directly from the spectrum, including correction for the selected window's
coherent gain.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Sequence

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
    min_local_snr_db: float = 6.0,
) -> CMRRSweepResult:
    """Estimate CMRR across frequency for a swept-tone selection.

    The selected region is split into overlapping windows. For each window, the
    dominant positive-frequency tone is detected and its exact-frequency RMS is
    estimated with a sine fit. CMRR is then computed against the user-entered
    common-mode amplitude.
    """

    signal_values = np.asarray(values, dtype=np.float64)
    if window_duration_s <= 0:
        window_duration_s = 0.75
    if stability_tolerance_hz <= 0:
        stability_tolerance_hz = 0.2

    window_size = max(int(round(window_duration_s * sample_rate)), 16)
    if signal_values.size < window_size:
        empty = pd.DataFrame(columns=["Frequency (Hz)", "CMRR Measured (dB)", "ADC Gain Corr (dB)", "CMRR (dB)", "CMRR Std (dB)", "Output RMS (uVrms)", "Windows"])
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)
    if common_mode_voltage <= 0:
        empty = pd.DataFrame(columns=["Frequency (Hz)", "CMRR Measured (dB)", "ADC Gain Corr (dB)", "CMRR (dB)", "CMRR Std (dB)", "Output RMS (uVrms)", "Windows"])
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=0)

    # Sweep mode always uses notch cleanup to suppress fixed mains pickup.
    filtered_values = apply_filters(
        signal_values[:, np.newaxis],
        sample_rate,
        FilterSettings(notch_50_hz=True, notch_100_hz=True, quality_factor=30.0),
    )[:, 0]

    zero_pad_length = max(int(fft_length), 8 * window_size)
    hop_size = max(int(window_size * hop_fraction), 1)
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
        relative_index = detect_fundamental_index(masked_frequencies, masked_amplitudes)
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
        cmrr_measured_db = 20.0 * np.log10(common_mode_voltage / output_volts) if output_volts > 0 else float("inf")
        correction_gain_db = float(
            compute_cmrr_correction_gain_db(
                [frequency_hz],
                correction_enabled=correction_enabled,
                correction_fs_hz=correction_fs_hz,
            )[0]
        )
        cmrr_db = cmrr_measured_db + correction_gain_db
        center_time_s = (start_index + 0.5 * window_size) / float(sample_rate)
        raw_rows.append(
            {
                "Time (s)": center_time_s,
                "Frequency (Hz)": frequency_hz,
                "Output RMS (uVrms)": output_rms,
                "CMRR Measured (dB)": cmrr_measured_db,
                "ADC Gain Corr (dB)": correction_gain_db,
                "CMRR (dB)": cmrr_db,
                "Local SNR (dB)": local_snr_db,
            }
        )

    raw_dataframe = pd.DataFrame(raw_rows)
    if raw_dataframe.empty:
        empty = pd.DataFrame(columns=["Frequency (Hz)", "CMRR Measured (dB)", "ADC Gain Corr (dB)", "CMRR (dB)", "CMRR Std (dB)", "Output RMS (uVrms)", "Windows"])
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected_windows)

    stable_mask = np.zeros(len(raw_dataframe), dtype=bool)
    run_start = 0
    while run_start < len(raw_dataframe):
        run_end = run_start + 1
        run_frequencies = [float(raw_dataframe.iloc[run_start]["Frequency (Hz)"])]
        while run_end < len(raw_dataframe):
            candidate_frequency = float(raw_dataframe.iloc[run_end]["Frequency (Hz)"])
            run_center = float(np.median(run_frequencies))
            if abs(candidate_frequency - run_center) > stability_tolerance_hz:
                break
            run_frequencies.append(candidate_frequency)
            run_end += 1
        if run_end - run_start >= min_stable_run_windows:
            stable_mask[run_start:run_end] = True
        else:
            rejected_windows += run_end - run_start
        run_start = run_end

    stable_dataframe = raw_dataframe.loc[stable_mask].copy()
    if stable_dataframe.empty:
        empty = pd.DataFrame(columns=["Frequency (Hz)", "CMRR Measured (dB)", "ADC Gain Corr (dB)", "CMRR (dB)", "CMRR Std (dB)", "Output RMS (uVrms)", "Windows"])
        return CMRRSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected_windows)

    frequency_bucket_hz = max(stability_tolerance_hz, 0.05)
    stable_dataframe["Frequency Bucket"] = (stable_dataframe["Frequency (Hz)"] / frequency_bucket_hz).round() * frequency_bucket_hz
    aggregated = (
        stable_dataframe.groupby("Frequency Bucket", as_index=False)
        .agg(
            {
                "Frequency (Hz)": "mean",
                "CMRR Measured (dB)": "mean",
                "ADC Gain Corr (dB)": "mean",
                "CMRR (dB)": "mean",
                "Output RMS (uVrms)": "mean",
                "Time (s)": "count",
            }
        )
        .rename(columns={"Time (s)": "Windows"})
    )
    cmrr_std = stable_dataframe.groupby("Frequency Bucket")["CMRR (dB)"].std(ddof=0).reset_index(name="CMRR Std (dB)")
    aggregated = aggregated.merge(cmrr_std, on="Frequency Bucket", how="left")
    aggregated["CMRR Std (dB)"] = aggregated["CMRR Std (dB)"].fillna(0.0)
    aggregated = aggregated.drop(columns=["Frequency Bucket"])
    aggregated = aggregated[["Frequency (Hz)", "CMRR Measured (dB)", "ADC Gain Corr (dB)", "CMRR (dB)", "CMRR Std (dB)", "Output RMS (uVrms)", "Windows"]]
    aggregated = aggregated.sort_values("Frequency (Hz)").reset_index(drop=True)
    return CMRRSweepResult(
        dataframe=aggregated,
        accepted_windows=int(len(stable_dataframe.index)),
        rejected_windows=int(rejected_windows),
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
    min_local_snr_db: float = 6.0,
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
        relative_index = detect_fundamental_index(masked_frequencies, masked_amplitudes)
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

    stable_mask = np.zeros(len(raw_dataframe), dtype=bool)
    run_start = 0
    while run_start < len(raw_dataframe):
        run_end = run_start + 1
        run_frequencies = [float(raw_dataframe.iloc[run_start]["Frequency (Hz)"])]
        while run_end < len(raw_dataframe):
            candidate_frequency = float(raw_dataframe.iloc[run_end]["Frequency (Hz)"])
            run_center = float(np.median(run_frequencies))
            if abs(candidate_frequency - run_center) > stability_tolerance_hz:
                break
            run_frequencies.append(candidate_frequency)
            run_end += 1
        if run_end - run_start >= min_stable_run_windows:
            stable_mask[run_start:run_end] = True
        else:
            rejected_windows += run_end - run_start
        run_start = run_end

    stable_dataframe = raw_dataframe.loc[stable_mask].copy()
    if stable_dataframe.empty:
        empty = pd.DataFrame(columns=columns)
        return CrosstalkSweepResult(dataframe=empty, accepted_windows=0, rejected_windows=rejected_windows)

    frequency_bucket_hz = max(stability_tolerance_hz, 0.05)
    stable_dataframe["Frequency Bucket"] = (stable_dataframe["Frequency (Hz)"] / frequency_bucket_hz).round() * frequency_bucket_hz
    aggregated = (
        stable_dataframe.groupby("Frequency Bucket", as_index=False)
        .agg(
            {
                "Frequency (Hz)": "mean",
                "Crosstalk (dB)": "mean",
                "Source RMS (uVrms)": "mean",
                "Target RMS (uVrms)": "mean",
                "Time (s)": "count",
            }
        )
        .rename(columns={"Time (s)": "Windows"})
    )
    crosstalk_std = stable_dataframe.groupby("Frequency Bucket")["Crosstalk (dB)"].std(ddof=0).reset_index(name="Crosstalk Std (dB)")
    aggregated = aggregated.merge(crosstalk_std, on="Frequency Bucket", how="left")
    aggregated["Crosstalk Std (dB)"] = aggregated["Crosstalk Std (dB)"].fillna(0.0)
    aggregated = aggregated.drop(columns=["Frequency Bucket"])
    aggregated = aggregated[columns]
    aggregated = aggregated.sort_values("Frequency (Hz)").reset_index(drop=True)
    return CrosstalkSweepResult(
        dataframe=aggregated,
        accepted_windows=int(len(stable_dataframe.index)),
        rejected_windows=int(rejected_windows),
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
    min_local_snr_db: float = 6.0,
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
        "Input Before Attenuator (V)",
        "Input After Attenuator (uVrms)",
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
            "Input Before Attenuator (V)": float(input_before_attenuator_v),
            "Input After Attenuator (uVrms)": input_after_attenuator_v / signal_scale_to_volts,
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