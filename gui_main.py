"""PyQt6 desktop application for EEG and ADC signal analysis.

The GUI is designed for scientific reproducibility:
- the selected analysis region is always explicit and highlighted
- all spectral metrics operate on the selected region only
- optional 50 Hz and 100 Hz notch filters are user-controlled
- exported CSV/PDF outputs reflect the current selection and settings
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import pandas as pd
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt6.QtCore import QSize, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QDoubleSpinBox,
    QSpinBox,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from report_export import (
    ExportOptions,
    MeasurementNameDialog,
    ExportSelectionDialog,
    MeasurementInfo,
    ReportSettingsStore,
    ReportWriter,
    full_export_stem,
    install_wheel_guard,
    parse_voltage,
)
from data_loader import (
    CsvPreview,
    SignalDataset,
    load_openbci_csv,
    openbci_sample_rate,
    read_openbci_metadata,
    sniff_csv_layout,
)
from bias_reference import (
    DEFAULT_BUCKET_PERCENT,
    DEFAULT_FIT_KIND,
    DEFAULT_MIN_RECORDINGS,
    DEFAULT_POLY_ORDER,
    DEFAULT_WINDOW_S,
    FIT_KINDS,
    FIT_LABELS,
    MIN_REFERENCE_VRMS,
    POINT_COLUMNS,
    RecordingSummary,
    ReferenceCurve,
    ReferenceReading,
    available_channels,
    build_curve,
    load_reference_recording,
)
from fft_analysis import (
    CHANNEL_AVERAGE_LABEL,
    VREF_VOLTS,
    DEFAULT_INPUT_ATTENUATOR_RATIO,
    FFTSettings,
    SpectrumResult,
    WINDOW_MAP,
    ads1299_sinc3_gain_db,
    analyze_multichannel,
    compute_cmrr_sweep,
    compute_crosstalk_sweep,
    compute_gain_sweep,
    compute_planned_cmrr_sweep,
    compute_planned_crosstalk_sweep,
    compute_planned_gain_sweep,
    compute_reference_cmrr_sweep,
    detect_tone_frequency,
    fit_tone_rms,
    parse_sweep_steps,
)
from signal_processing import FilterSettings, apply_filters, extract_time_region, hard_remove_ranges, keep_only_range, merge_ranges
from recording_compare import compare_recordings


# Sweep curves were thin enough to lose behind the grid on a 2560-wide screen.
SWEEP_TRACE_WIDTH = 2.4
SWEEP_FIT_WIDTH = 2.0
SWEEP_FIT_COLOR = (70, 82, 100)
# Entry in the CMRR monitor combo that means "work the frequency out from the
# channel under test".
NO_MONITOR_CHANNEL = "(none)"

# How the common-mode voltage for a CMRR calculation is established. Each one is a
# different answer to "what voltage was really at the input", and they differ by tens
# of dB, so the choice is explicit rather than inferred.
VCM_FLAT = "flat"
VCM_SINC3 = "sinc3"
VCM_MONITOR = "monitor"
VCM_MONITOR_CURVE = "monitor_curve"
VCM_MODE_LABELS = {
    VCM_FLAT: "flat amplitude",
    VCM_SINC3: "sinc3-corrected amplitude",
    VCM_MONITOR: "CM monitor measured",
    VCM_MONITOR_CURVE: "CM monitor frequencies + bias curve voltage",
}


def resolve_channel_name(reference: str, channel_names: Sequence[str]) -> str:
    """Match a recorded channel label to one of the dataset's column names.

    The recorder names a reference as ``CH 1`` (its own label), while the CSV
    columns are ``EXG Channel 0`` and so on. Comparing the two literally never
    matches, which is why a recorded reference used to be ignored instead of
    excluded from the plots. This accepts either form, plus a bare number.
    """

    text = str(reference or "").strip()
    if not text:
        return ""
    names = [str(name) for name in channel_names]
    if text in names:
        return text
    lowered = {name.lower(): name for name in names}
    if text.lower() in lowered:
        return lowered[text.lower()]

    # "CH 3" -> third channel; "EXG Channel 2" -> third channel as well, since the
    # two conventions differ by one. Prefer the CH form because that is what the
    # recorder writes.
    digits = "".join(character for character in text if character.isdigit())
    if not digits:
        return ""
    number = int(digits)
    upper = text.upper()
    looks_one_based = upper.startswith("CH ") or upper.startswith("CHANNEL") or upper == f"CH{digits}"
    if looks_one_based:
        zero_based = number - 1
        if 0 <= zero_based < len(names):
            return names[zero_based]
        return ""
    # Already a column name with an index, e.g. "EXG Channel 2".
    for name in names:
        if name.lower().endswith(str(number)):
            return name
    return ""

# Sweep figures are drawn over the measurement band only. A fixed 0..125 Hz axis
# replaces the auto range, which used to leave half the plot empty (-50..250 Hz).
SWEEP_X_MAX_HZ = 125.0
# Height/width for the FFT figure in the report: a wide strip instead of a
# narrow column that came out of the tab layout.
FFT_FIGURE_ASPECT = 0.42


def _metadata_float(metadata: dict[str, str], key: str) -> float:
    try:
        return float(str(metadata.get(key, "") or "").strip())
    except (TypeError, ValueError):
        return 0.0


def _sweep_amplitude_vrms(metadata: dict[str, str]) -> float:
    """Applied sine amplitude in Vrms, whichever key the recording carries.

    Everything is Vrms now. Recordings made before that change stored Vpp, so the
    old key is converted rather than silently mistreated as an RMS value.
    """

    vrms = _metadata_float(metadata, "Sweep Amplitude (Vrms)")
    if vrms > 0:
        return vrms
    vpp = _metadata_float(metadata, "Sweep Amplitude (Vpp)")
    return vpp / (2.0 * np.sqrt(2.0)) if vpp > 0 else 0.0
SWEEP_FIT_ORDERS = (
    ("No fit", 0),
    ("Order 2", 2),
    ("Order 3", 3),
    ("Order 4", 4),
)
SWEEP_FIT_REFERENCE_HZ = 50.0
# Reported sweep figures are re-laid out wider than the on-screen widget so the
# legend and the swept points stay readable at page width.
SWEEP_FIGURE_ASPECT = 0.42
SWEEP_FIGURE_MAX_HEIGHT = 0.55
# Height of one sweep figure inside the scrollable area. Tall enough to read, short
# enough that four of them are reachable with one scroll gesture.
SWEEP_PLOT_HEIGHT = 260


class RecordingComparisonDialog(QDialog):
    """Compare a live recording with the board's own SD-card recording.

    Deliberately not part of the main layout: it answers a bench question ("did
    the card lose anything, and did the two recordings start together?"), not an
    analysis question, so it opens from the menu and closes again.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Compare two recordings")
        self.resize(940, 720)
        self.setWindowFlag(Qt.WindowType.Window, True)

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        intro = QLabel(
            "Pick the live recording and the copy the board wrote to its SD card. The comparison "
            "reports how far apart the two timelines are, whether either is missing samples, and "
            "whether the traces actually correlate - which is what tells you if the card kept up."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form = QGridLayout()
        self.file_a_edit = QLineEdit()
        self.file_b_edit = QLineEdit()
        for row, (label, edit) in enumerate(
            (("Recording A (live)", self.file_a_edit), ("Recording B (SD card)", self.file_b_edit))
        ):
            form.addWidget(QLabel(label), row, 0)
            form.addWidget(edit, row, 1)
            browse = QPushButton("Browse...")
            browse.clicked.connect(lambda _checked=False, target=edit: self._browse(target))
            form.addWidget(browse, row, 2)
        layout.addLayout(form)

        button_row = QHBoxLayout()
        self.compare_button = QPushButton("Compare")
        self.compare_button.clicked.connect(self._compare)
        button_row.addWidget(self.compare_button)
        self.swap_button = QPushButton("Swap")
        self.swap_button.setToolTip("Compare the other way round (start offset is signed)")
        self.swap_button.clicked.connect(self._swap)
        button_row.addWidget(self.swap_button)
        button_row.addStretch(1)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        button_row.addWidget(close_button)
        layout.addLayout(button_row)

        self.result_view = QPlainTextEdit()
        self.result_view.setReadOnly(True)
        self.result_view.setStyleSheet("font-family: Consolas; font-size: 12px;")
        self.result_view.setPlaceholderText("The comparison appears here.")
        layout.addWidget(self.result_view, 1)

    def set_first_file(self, path) -> None:
        self.file_a_edit.setText(str(path))

    def _browse(self, target: QLineEdit) -> None:
        start = target.text().strip() or str(Path.cwd())
        chosen, _filter = QFileDialog.getOpenFileName(
            self, "Choose a recording", start, "Recordings (*.csv *.CSV);;All files (*)"
        )
        if chosen:
            target.setText(chosen)

    def _swap(self) -> None:
        a, b = self.file_a_edit.text(), self.file_b_edit.text()
        self.file_a_edit.setText(b)
        self.file_b_edit.setText(a)

    def _compare(self) -> None:
        path_a = self.file_a_edit.text().strip()
        path_b = self.file_b_edit.text().strip()
        if not path_a or not path_b:
            QMessageBox.information(self, "Compare", "Choose both recordings first.")
            return
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            result = compare_recordings(path_a, path_b)
        except (OSError, ValueError) as exc:
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Compare", f"Could not compare the files.\\n\\n{exc}")
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.result_view.setPlainText(result.report())


class PlotScrollArea(QScrollArea):
    """Vertical scroll area whose plots zoom on the mouse wheel.

    Four stacked sweep figures do not fit on any monitor, so the area scrolls.
    The wheel zooms the plot under the cursor (which is what "zoomable" means
    here) and Shift+wheel scrolls the page, so both actions stay reachable
    without a modifier-free ambiguity.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt naming
        scrolling = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        plot = None if scrolling else self._plot_under(event)
        if plot is None:
            super().wheelEvent(event)
            return
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 0.82 if delta > 0 else 1.0 / 0.82
        view_box = plot.getPlotItem().getViewBox()
        view_box.scaleBy((factor, factor))
        # Auto-range would undo the zoom on the next redraw.
        view_box.disableAutoRange()
        event.accept()

    def _plot_under(self, event) -> Optional[pg.PlotWidget]:
        widget = self.widget()
        if widget is None:
            return None
        position = event.position().toPoint()
        child = widget.childAt(widget.mapFrom(self.viewport(), position))
        while child is not None:
            if isinstance(child, pg.PlotWidget):
                return child
            child = child.parentWidget()
        return None


@dataclass(slots=True)
class EditAction:
    action_type: str
    time_range: Optional[tuple[float, float]] = None


class SelectableViewBox(pg.ViewBox):
    selectionPreview = pyqtSignal(float, float)
    selectionFinished = pyqtSignal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.setMouseMode(pg.ViewBox.PanMode)
        self._selection_enabled = False
        self._drag_start_x: Optional[float] = None

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = bool(enabled)

    def mouseDragEvent(self, event, axis=None):  # type: ignore[override]
        if self._selection_enabled and event.button() == Qt.MouseButton.LeftButton:
            event.accept()
            if event.isStart():
                self._drag_start_x = float(self.mapToView(event.buttonDownPos()).x())
            current_x = float(self.mapToView(event.pos()).x())
            if self._drag_start_x is not None:
                start_x, end_x = sorted((self._drag_start_x, current_x))
                self.selectionPreview.emit(start_x, end_x)
            if event.isFinish() and self._drag_start_x is not None:
                start_x, end_x = sorted((self._drag_start_x, current_x))
                self.selectionFinished.emit(start_x, end_x)
                self._drag_start_x = None
            return
        super().mouseDragEvent(event, axis=axis)


class ImportOptionsDialog(QDialog):
    def __init__(
        self,
        file_path: Path,
        preview: CsvPreview,
        parent: Optional[QWidget] = None,
        default_sample_rate: Optional[float] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("CSV Import Options")
        self.resize(520, 620)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(f"File: {file_path.name}"))
        format_label = QLabel(preview.format_description)
        format_label.setWordWrap(True)
        layout.addWidget(format_label)

        form = QFormLayout()
        self.sample_rate_spin = QDoubleSpinBox()
        self.sample_rate_spin.setRange(0.1, 100_000.0)
        self.sample_rate_spin.setDecimals(3)
        self.sample_rate_spin.setValue(float(default_sample_rate) if default_sample_rate else 250.0)
        self.sample_rate_spin.setSuffix(" Hz")
        form.addRow("Sampling rate", self.sample_rate_spin)

        self.timestamp_combo = QComboBox()
        self.timestamp_combo.addItem("Generate from sampling rate", None)
        for column in preview.numeric_columns:
            self.timestamp_combo.addItem(column, column)
        if preview.recommended_timestamp:
            index = self.timestamp_combo.findData(preview.recommended_timestamp)
            if index >= 0:
                self.timestamp_combo.setCurrentIndex(index)
        form.addRow("Timestamp column", self.timestamp_combo)
        layout.addLayout(form)

        channel_group = QGroupBox("Channels")
        channel_layout = QVBoxLayout(channel_group)
        button_row = QHBoxLayout()
        select_all_button = QPushButton("Select All")
        clear_all_button = QPushButton("Clear All")
        button_row.addWidget(select_all_button)
        button_row.addWidget(clear_all_button)
        channel_layout.addLayout(button_row)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        self.channel_checks: dict[str, QCheckBox] = {}
        for column in preview.numeric_columns:
            checkbox = QCheckBox(column)
            checkbox.setChecked(column in preview.recommended_channels)
            self.channel_checks[column] = checkbox
            scroll_layout.addWidget(checkbox)
        scroll_layout.addStretch(1)
        scroll_area.setWidget(scroll_content)
        channel_layout.addWidget(scroll_area)
        layout.addWidget(channel_group)

        select_all_button.clicked.connect(lambda: self._set_all_channels(True))
        clear_all_button.clicked.connect(lambda: self._set_all_channels(False))
        self.timestamp_combo.currentIndexChanged.connect(self._sync_timestamp_channel)
        self._sync_timestamp_channel()

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._validate_and_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def sample_rate(self) -> float:
        return float(self.sample_rate_spin.value())

    def timestamp_column(self) -> Optional[str]:
        return self.timestamp_combo.currentData()

    def selected_channels(self) -> list[str]:
        return [name for name, checkbox in self.channel_checks.items() if checkbox.isChecked()]

    def _set_all_channels(self, checked: bool) -> None:
        timestamp_column = self.timestamp_column()
        for name, checkbox in self.channel_checks.items():
            if checkbox.isEnabled() and name != timestamp_column:
                checkbox.setChecked(checked)

    def _sync_timestamp_channel(self) -> None:
        timestamp_column = self.timestamp_column()
        for name, checkbox in self.channel_checks.items():
            enabled = name != timestamp_column
            checkbox.setEnabled(enabled)
            if not enabled:
                checkbox.setChecked(False)

    def _validate_and_accept(self) -> None:
        if not self.selected_channels():
            QMessageBox.warning(self, "Import", "Select at least one channel to load.")
            return
        self.accept()


METRICS_HELP_HTML = """
<style>
  h3 { margin: 14px 0 4px 0; font-size: 11pt; }
  p  { margin: 2px 0 4px 0; }
  code { font-family: Consolas, monospace; background: #f0f0f0; }
  table { border-collapse: collapse; margin-top: 4px; }
  td, th { border: 1px solid #ccc; padding: 3px 8px; font-size: 9.5pt; }
  th { background: #eef2f7; }
</style>
<h2>Metric definitions</h2>
<p>Amplitudes are always <b>uVrms</b>. Every derivative below is computed from the data inside the
selection region, using only the channels ticked in both channel lists.</p>

<h3>Fundamental</h3>
<p>The <b>largest spectral peak</b> inside the selected region (Auto detect), or the exact frequency you type
in Manual Hz. Its level comes from a least-squares sine fit at that frequency. The fit refines the frequency
around the peak first, because a peak estimate that is off by a fraction of a bin would leave a residue that
looks like noise and would make SNR and ENOB far too pessimistic.</p>
<p>Use Manual Hz when a mains harmonic or a drift peak is taller than the signal you care about.</p>

<h3>SNR (dB)</h3>
<p><code>10 log10(fundamental power / residual power)</code></p>
<p>The residual is everything the fitted fundamental does <i>not</i> explain: broadband noise, hum,
harmonics, drift. It needs no values from outside the recording - it describes the measured spectrum.</p>

<h3>THD (%) and THD (dB)</h3>
<p><code>THD% = 100 sqrt((h2^2 + h3^2 + h4^2 + h5^2) / fundamental^2)</code>,
<code>THD dB = 20 log10(THD% / 100)</code></p>
<p>Harmonics 2f..5f are each measured with the same sine fit. THD is relative to the detected fundamental,
so it never needs the applied signal either.</p>

<h3>ENOB (bits)</h3>
<p><code>(SNR - 1.76) / 6.02</code></p>
<p>This is the true resolution of the converter only when the input actually reaches full scale. For a small
input it describes the measured signal-to-noise ratio, not the ADC, so compare it with ENOB @ FS.</p>

<h3>Applied signal and the derived columns</h3>
<p>Enter the RMS you really fed into the board in <b>Derived Metrics</b>. The table then adds:</p>
<table>
<tr><th>Column</th><th>Formula</th><th>Meaning</th></tr>
<tr><td>Gain Error (dB)</td><td><code>20 log10(measured / applied)</code></td><td>Deviation of the measured
fundamental from the level you applied.</td></tr>
<tr><td>SNR In-Referred (dB)</td><td><code>20 log10(applied / residual)</code></td><td>SNR seen by the
source: noise referred to the input instead of to the measured fundamental.</td></tr>
<tr><td>ENOB @ FS (bits)</td><td><code>((SNR + 20 log10(FS / fundamental)) - 1.76) / 6.02</code></td>
<td>SNR scaled up to full scale using the channel PGA gain from the recording header, which is what makes
it comparable between gain settings. A pure-noise record reports none, because scaling up the largest noise
bin would exceed the converter resolution.</td></tr>
</table>

<h3>The other columns</h3>
<p><b>Selection Avg RMS</b> uses a fixed 0.5-100 Hz band; <b>Band RMS 0.1-100 Hz</b> is the input-referred
noise figure and is the one to compare between builds.</p>
<p><b>Mean rows</b> average only the channels ticked in both the time and the FFT lists, so a hidden channel
cannot move the summary.</p>
<p><b>50 Hz pickup</b> is the exact 50 Hz RMS. <b>CMRR</b> uses the selected CMRR frequency or the
fundamental; with the ADS1299 sinc3 correction enabled,
<code>CMRR corrected(f) = CMRR measured(f) + G_ADC(f)</code>. <b>Crosstalk</b> is the target/source ratio at
the selected frequency.</p>
"""


class BiasReferenceDialog(QDialog):
    """Build the bias reference curve from several unbuffered-channel recordings.

    Each input CSV is a run with the generator wired straight into channel 0 - no
    bias amplifier in the path - so that channel measures the voltage the
    generator really delivered at each frequency. Averaging the runs suppresses
    the noise of any one of them, and the fit turns the average into a continuous
    ``Vcm(f)`` that CMRR analysis can look a frequency up in.

    This is calibration, not measurement: the curve is reused across recordings,
    so it is saved to a JSON file rather than recomputed per analysis.
    """

    curveChanged = pyqtSignal(object)
    useCurveChanged = pyqtSignal(bool)

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        curve: Optional[ReferenceCurve] = None,
        use_curve: bool = False,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Bias reference curve")
        self.resize(1080, 760)
        self.setWindowFlag(Qt.WindowType.Window, True)

        self._files: list[str] = []
        self._summaries: list[RecordingSummary] = []
        self._readings: list[ReferenceReading] = []
        self._curve: Optional[ReferenceCurve] = None
        self._reloading = False

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        hint = QLabel(
            "Add two or more recordings of the same sweep. Channel 0 of each must be wired straight "
            "across the generator output, with no bias amplifier in the path. The averaged, fitted "
            "curve supplies the common-mode voltage for CMRR analysis; the frequency of a CMRR sweep "
            "still comes from that recording's own monitor channel."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #475569;")
        layout.addWidget(hint)

        layout.addWidget(self._build_files_group(), 0)
        layout.addWidget(self._build_settings_group(), 0)
        layout.addWidget(self._build_plots(), 1)

        self.summary_label = QLabel("No curve yet.")
        self.summary_label.setWordWrap(True)
        layout.addWidget(self.summary_label)

        buttons = QHBoxLayout()
        self.save_button = QPushButton("Save curve...")
        self.save_button.clicked.connect(self.save_curve)
        buttons.addWidget(self.save_button)
        self.load_button = QPushButton("Load curve...")
        self.load_button.clicked.connect(self.load_curve)
        buttons.addWidget(self.load_button)
        self.export_button = QPushButton("Export points CSV")
        self.export_button.clicked.connect(self.export_points)
        buttons.addWidget(self.export_button)
        buttons.addStretch(1)
        self.use_check = QCheckBox("Use this curve for CMRR")
        self.use_check.setChecked(bool(use_curve))
        self.use_check.setToolTip(
            "When ticked, the CMRR sweep reads the common-mode voltage from this curve at each "
            "measured frequency instead of using the single amplitude in the analysis panel."
        )
        self.use_check.toggled.connect(self.useCurveChanged.emit)
        buttons.addWidget(self.use_check)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        buttons.addWidget(close_button)
        layout.addLayout(buttons)

        if curve is not None:
            self.set_curve(curve)

    # -- construction ------------------------------------------------------

    def _build_files_group(self) -> QWidget:
        group = QGroupBox("Recordings")
        layout = QVBoxLayout(group)
        self.file_table = QTableWidget(0, 3)
        self.file_table.setHorizontalHeaderLabels(["File", "Sample rate", "Points"])
        self.file_table.verticalHeader().setVisible(False)
        self.file_table.setMaximumHeight(140)
        self.file_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.file_table.horizontalHeader().setStretchLastSection(False)
        self.file_table.setColumnWidth(0, 620)
        self.file_table.setColumnWidth(1, 110)
        layout.addWidget(self.file_table)

        row = QHBoxLayout()
        add_button = QPushButton("Add recordings...")
        add_button.clicked.connect(self.add_recordings)
        row.addWidget(add_button)
        remove_button = QPushButton("Remove selected")
        remove_button.clicked.connect(self.remove_selected)
        row.addWidget(remove_button)
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_recordings)
        row.addWidget(clear_button)
        row.addStretch(1)
        layout.addLayout(row)
        return group

    def _build_settings_group(self) -> QWidget:
        group = QGroupBox("Averaging and fit")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Reference channel"))
        self.channel_combo = QComboBox()
        self.channel_combo.setMinimumWidth(150)
        self.channel_combo.setToolTip(
            "Column read from every file. These recordings put the unbuffered tap on channel 0, so "
            "the first channel is the default."
        )
        self.channel_combo.currentIndexChanged.connect(self._on_channel_changed)
        layout.addWidget(self.channel_combo)

        layout.addWidget(QLabel("Window"))
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(0.05, 10.0)
        self.window_spin.setDecimals(2)
        self.window_spin.setSingleStep(0.05)
        self.window_spin.setValue(DEFAULT_WINDOW_S)
        self.window_spin.setSuffix(" s")
        self.window_spin.setToolTip(
            "Length of the piece each frequency is read from when a recording has no generator plan. "
            "Longer is quieter but blurs a fast sweep."
        )
        layout.addWidget(self.window_spin)

        layout.addWidget(QLabel("Bucket"))
        self.bucket_spin = QDoubleSpinBox()
        self.bucket_spin.setRange(0.0, 50.0)
        self.bucket_spin.setDecimals(1)
        self.bucket_spin.setSingleStep(0.5)
        self.bucket_spin.setValue(DEFAULT_BUCKET_PERCENT)
        self.bucket_spin.setSuffix(" %")
        self.bucket_spin.setToolTip(
            "How close two readings must be, relative to their frequency, to be averaged together. "
            "2 % is 0.1 Hz at 5 Hz and 4 Hz at 200 Hz."
        )
        layout.addWidget(self.bucket_spin)

        layout.addWidget(QLabel("Min recordings"))
        self.min_recordings_spin = QSpinBox()
        self.min_recordings_spin.setRange(1, 50)
        self.min_recordings_spin.setValue(DEFAULT_MIN_RECORDINGS)
        self.min_recordings_spin.setToolTip(
            "Drop a frequency that fewer than this many recordings contributed to, so a single noisy "
            "file cannot masquerade as a well-averaged point."
        )
        layout.addWidget(self.min_recordings_spin)

        layout.addWidget(QLabel("Fit"))
        self.fit_combo = QComboBox()
        for label, kind in FIT_KINDS:
            self.fit_combo.addItem(label, kind)
        self.fit_combo.setCurrentIndex(max(self.fit_combo.findData(DEFAULT_FIT_KIND), 0))
        self.fit_combo.setToolTip(
            "Polynomial fits the trend without chasing noise. The exact options pass through every "
            "averaged point, which is what to use when the points already are the measurement."
        )
        layout.addWidget(self.fit_combo)

        layout.addWidget(QLabel("Order"))
        self.order_spin = QSpinBox()
        self.order_spin.setRange(1, 20)
        self.order_spin.setValue(DEFAULT_POLY_ORDER)
        self.order_spin.setToolTip("Only used by the polynomial fit.")
        layout.addWidget(self.order_spin)

        build_button = QPushButton("Build curve")
        build_button.clicked.connect(self.build)
        layout.addWidget(build_button)
        layout.addStretch(1)

        self.fit_combo.currentIndexChanged.connect(self._on_fit_changed)
        self.order_spin.valueChanged.connect(self._on_fit_changed)
        self.bucket_spin.valueChanged.connect(self._mark_stale)
        self.min_recordings_spin.valueChanged.connect(self._mark_stale)
        return group

    def _build_plots(self) -> QWidget:
        holder = QWidget()
        layout = QVBoxLayout(holder)
        layout.setContentsMargins(0, 0, 0, 0)

        self.log_check = QCheckBox("Log frequency axis")
        self.log_check.setChecked(True)
        self.log_check.toggled.connect(self._apply_log_mode)
        layout.addWidget(self.log_check)

        heading = QLabel("Reference voltage actually measured, averaged over the recordings")
        heading.setStyleSheet("font-weight: 600; color: #1e293b;")
        layout.addWidget(heading)
        self.points_plot = pg.PlotWidget()
        self.points_plot.setMinimumHeight(230)
        self.points_plot.setLabel("bottom", "Frequency", units="Hz")
        self.points_plot.setLabel("left", "Reference", units="Vrms")
        self.points_plot.showGrid(x=True, y=True, alpha=0.28)
        self.points_plot.addLegend(offset=(-10, 10))
        layout.addWidget(self.points_plot)

        heading = QLabel("Transfer from the generator setpoint to the measured reference")
        heading.setStyleSheet("font-weight: 600; color: #1e293b;")
        layout.addWidget(heading)
        self.gain_plot = pg.PlotWidget()
        self.gain_plot.setMinimumHeight(230)
        self.gain_plot.setLabel("bottom", "Frequency", units="Hz")
        self.gain_plot.setLabel("left", "Gain", units="dB")
        self.gain_plot.showGrid(x=True, y=True, alpha=0.28)
        self.gain_plot.addLegend(offset=(-10, 10))
        layout.addWidget(self.gain_plot)

        self._apply_log_mode(True)
        return holder

    # -- file handling -----------------------------------------------------

    def add_recordings(self) -> None:
        paths, _filter = QFileDialog.getOpenFileNames(
            self, "Add reference recordings", "", "CSV files (*.csv);;All files (*)"
        )
        if not paths:
            return
        for path in paths:
            if path not in self._files:
                self._files.append(path)
        self._reload()

    def remove_selected(self) -> None:
        rows = sorted({index.row() for index in self.file_table.selectedIndexes()}, reverse=True)
        for row in rows:
            if 0 <= row < len(self._files):
                del self._files[row]
        self._reload()

    def clear_recordings(self) -> None:
        self._files.clear()
        self._reload()

    def _on_channel_changed(self, *_args) -> None:
        if not self._reloading:
            self._reload()

    def _populate_channels(self) -> None:
        """Offer the columns of the first file, keeping any existing choice."""

        previous = self.channel_combo.currentText()
        names: list[str] = []
        if self._files:
            names = available_channels(self._files[0])
        self._reloading = True
        self.channel_combo.clear()
        self.channel_combo.addItem("(first channel)", "")
        for name in names:
            self.channel_combo.addItem(name, name)
        position = self.channel_combo.findData(previous)
        self.channel_combo.setCurrentIndex(max(position, 0))
        self._reloading = False

    def _reload(self) -> None:
        """Re-read every file at the selected channel."""

        self._populate_channels()
        column = str(self.channel_combo.currentData() or "") or None
        self._summaries = []
        self._readings = []
        for path in self._files:
            summary, readings = load_reference_recording(
                path,
                channel_name=column,
                window_duration_s=float(self.window_spin.value()),
                fft_length=int(4096),
                window_name="Hann",
            )
            self._summaries.append(summary)
            self._readings.extend(readings)

        self.file_table.setRowCount(len(self._summaries))
        for row, summary in enumerate(self._summaries):
            name_item = QTableWidgetItem(Path(summary.path).name)
            name_item.setToolTip(summary.describe())
            self.file_table.setItem(row, 0, name_item)
            rate_item = QTableWidgetItem(f"{summary.sample_rate_hz:g} SPS" if summary.sample_rate_hz else "-")
            self.file_table.setItem(row, 1, rate_item)
            points_item = QTableWidgetItem(summary.error if summary.error else str(summary.readings))
            if summary.error:
                points_item.setToolTip(summary.error)
            self.file_table.setItem(row, 2, points_item)

        self.build()

    # -- building ----------------------------------------------------------

    def parameters(self) -> dict[str, object]:
        return {
            "fit_kind": str(self.fit_combo.currentData() or DEFAULT_FIT_KIND),
            "poly_order": int(self.order_spin.value()),
            "bucket_percent": float(self.bucket_spin.value()),
            "min_recordings": int(self.min_recordings_spin.value()),
        }

    def build(self) -> None:
        """Average the loaded readings and fit the curve."""

        setpoints = [s.setpoint_v for s in self._summaries if s.setpoint_v > 0]
        rates = [s.sample_rate_hz for s in self._summaries if s.sample_rate_hz > 0]
        parameters = self.parameters()
        self._curve = build_curve(
            self._readings,
            sources=self._files,
            fit_kind=str(parameters["fit_kind"]),
            poly_order=int(parameters["poly_order"]),
            # Median, not mean: the curve's setpoint is what a later run is scaled
            # *from*, so one recording with a missing or mistyped amplitude must not
            # drag the whole calibration off by a few percent.
            setpoint_v=float(np.median(setpoints)) if setpoints else 0.0,
            sample_rate_hz=float(np.median(rates)) if rates else 0.0,
            bucket_percent=float(parameters["bucket_percent"]),
            min_recordings=int(parameters["min_recordings"]),
        )
        self._draw()
        self.curveChanged.emit(self._curve)

    def _mark_stale(self, *_args) -> None:
        if not self._reloading and self._readings:
            self.build()

    def _on_fit_changed(self, *_args) -> None:
        # Re-fitting does not need the files re-read; only the curve changes.
        if self._curve is not None and len(self._curve.points) > 1:
            parameters = self.parameters()
            self._curve.fit_kind = str(parameters["fit_kind"])
            self._curve.poly_order = int(parameters["poly_order"])
            self._draw()
            self.curveChanged.emit(self._curve)

    def _apply_log_mode(self, enabled: bool) -> None:
        for plot in (self.points_plot, self.gain_plot):
            plot.setLogMode(x=bool(enabled), y=False)

    # -- drawing -----------------------------------------------------------

    def _draw(self) -> None:
        self.points_plot.clear()
        self.gain_plot.clear()
        curve = self._curve
        if curve is None or not len(curve):
            self.summary_label.setText(
                "Add recordings with a usable tone. No curve was built."
                if self._files
                else "No curve yet. Add the reference recordings first."
            )
            return

        frequencies = curve.frequencies()
        voltages = curve.voltages()
        spread = curve.spread()
        self.points_plot.plot(
            frequencies,
            voltages,
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush=(16, 120, 90),
            name="averaged points",
        )
        # The spread is the disagreement between runs, which is the only honest
        # error bar this data has: it is drawn, not fitted.
        if np.any(spread > 0):
            upper = voltages + spread
            lower = np.clip(voltages - spread, np.finfo(np.float64).tiny, None)
            self.points_plot.plot(frequencies, upper, pen=pg.mkPen((16, 120, 90), width=1, style=Qt.PenStyle.DotLine))
            self.points_plot.plot(
                frequencies,
                lower,
                pen=pg.mkPen((16, 120, 90), width=1, style=Qt.PenStyle.DotLine),
                name="+/- 1 SD",
            )
        if curve.setpoint_v > 0:
            self.points_plot.plot(
                frequencies,
                np.full(frequencies.shape, curve.setpoint_v),
                pen=pg.mkPen((190, 90, 60), width=1.4, style=Qt.PenStyle.DashLine),
                name="generator setpoint",
            )

        grid, fitted = curve.fitted()
        if grid.size and np.any(np.isfinite(fitted)):
            self.points_plot.plot(
                grid,
                fitted,
                pen=pg.mkPen((16, 120, 90), width=2.0, style=Qt.PenStyle.DashLine),
                name=FIT_LABELS.get(curve.fit_kind, curve.fit_kind),
            )
            if curve.setpoint_v > 0:
                with np.errstate(divide="ignore", invalid="ignore"):
                    gain_db = 20.0 * np.log10(np.where(fitted > 0, fitted / curve.setpoint_v, np.nan))
                self.gain_plot.plot(grid, gain_db, pen=pg.mkPen((37, 99, 235), width=2.0))
            self.gain_plot.plot([], [], pen=None, name=FIT_LABELS.get(curve.fit_kind, curve.fit_kind))
        gains = curve.gains()
        finite = np.isfinite(gains) & (gains > 0)
        if np.any(finite):
            with np.errstate(divide="ignore", invalid="ignore"):
                point_db = 20.0 * np.log10(gains[finite])
            self.gain_plot.plot(
                frequencies[finite],
                point_db,
                pen=None,
                symbol="o",
                symbolSize=5,
                symbolBrush=(37, 99, 235),
                name="per point",
            )
        if curve.fit_kind == "poly" and curve.setpoint_v > 0 and len(curve.points) > 1:
            self.gain_plot.plot([], [], pen=None, name="")

        self.summary_label.setText(curve.summary())

    def set_curve(self, curve: Optional[ReferenceCurve]) -> None:
        """Adopt a curve that was built or loaded elsewhere."""

        self._curve = curve
        if curve is not None:
            self.fit_combo.blockSignals(True)
            position = self.fit_combo.findData(curve.fit_kind)
            self.fit_combo.setCurrentIndex(max(position, 0))
            self.fit_combo.blockSignals(False)
            self.order_spin.blockSignals(True)
            self.order_spin.setValue(int(curve.poly_order))
            self.order_spin.blockSignals(False)
            self._files = list(curve.sources)
            self._summaries = []
            self._readings = []
            self.file_table.setRowCount(len(curve.sources))
            for row, source in enumerate(curve.sources):
                self.file_table.setItem(row, 0, QTableWidgetItem(Path(source).name))
                self.file_table.setItem(row, 1, QTableWidgetItem("-"))
                self.file_table.setItem(row, 2, QTableWidgetItem("(from file)"))
        self._draw()

    @property
    def curve(self) -> Optional[ReferenceCurve]:
        return self._curve

    def use_curve(self) -> bool:
        return bool(self.use_check.isChecked())

    def set_use_curve(self, enabled: bool) -> None:
        self.use_check.blockSignals(True)
        self.use_check.setChecked(bool(enabled))
        self.use_check.blockSignals(False)

    # -- persistence -------------------------------------------------------

    def save_curve(self) -> None:
        if self._curve is None or not len(self._curve):
            QMessageBox.information(self, "Bias reference curve", "Build a curve before saving it.")
            return
        file_name, _filter = QFileDialog.getSaveFileName(
            self, "Save bias reference curve", "bias_reference_curve.json", "Curve files (*.json)"
        )
        if not file_name:
            return
        try:
            self._curve.save(file_name)
        except OSError as error:
            QMessageBox.warning(self, "Bias reference curve", f"Could not write the file:\n{error}")
            return
        self.summary_label.setText(f"Saved to {file_name}. " + self._curve.summary())

    def load_curve(self) -> None:
        file_name, _filter = QFileDialog.getOpenFileName(
            self, "Load bias reference curve", "", "Curve files (*.json);;All files (*)"
        )
        if not file_name:
            return
        try:
            curve = ReferenceCurve.load(file_name)
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, "Bias reference curve", f"Could not read the file:\n{error}")
            return
        if not len(curve):
            QMessageBox.warning(self, "Bias reference curve", "That file contains no points.")
            return
        self.set_curve(curve)
        self.use_check.setChecked(True)
        self.curveChanged.emit(curve)

    def export_points(self) -> None:
        if self._curve is None or not len(self._curve):
            QMessageBox.information(self, "Bias reference curve", "There are no points to export.")
            return
        file_name, _filter = QFileDialog.getSaveFileName(
            self, "Export reference points", "bias_reference_points.csv", "CSV files (*.csv)"
        )
        if not file_name:
            return
        try:
            self._curve.to_frame().to_csv(file_name, index=False)
        except OSError as error:
            QMessageBox.warning(self, "Bias reference curve", f"Could not write the file:\n{error}")


class ScientificSignalAnalyzerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Scientific EEG/ADC Signal Analyzer")
        self.resize(self._default_window_size())

        self.dataset: Optional[SignalDataset] = None
        self.original_dataset: Optional[SignalDataset] = None
        self.excluded_ranges: list[tuple[float, float]] = []
        self.edit_history: list[EditAction] = []
        self.metrics_df = pd.DataFrame()
        self.crosstalk_df = pd.DataFrame()
        self.crosstalk_sweep_df = pd.DataFrame()
        self.gain_sweep_df = pd.DataFrame()
        self.cmrr_sweep_df = pd.DataFrame()
        self.crosstalk_sweep_fit_df = pd.DataFrame()
        self.gain_sweep_fit_df = pd.DataFrame()
        self.cmrr_sweep_fit_df = pd.DataFrame()
        self.cmrr_reference_df = pd.DataFrame()
        self.cmrr_mean_fit_df = pd.DataFrame()
        self.cmrr_mean_fit_hz = float("nan")
        self.cmrr_reference_fit_hz = float("nan")
        # The Vcm a bias reference curve supplied, and the amplitude it was scaled
        # to. Kept on the window so the reference figure and the results table are
        # always drawn from the same numbers.
        self.cmrr_bias_overlay: Optional[pd.DataFrame] = None
        self.cmrr_vcm_scale = 1.0
        self.cmrr_vcm_note = ""
        self.cmrr_monitor_table: Optional[pd.DataFrame] = None
        self.cmrr_reference_note = ""
        self.spectra: list[SpectrumResult] = []
        self.channel_checks: dict[str, QCheckBox] = {}
        self.channel_fft_checks: dict[str, QCheckBox] = {}
        self.channel_curves: dict[int, pg.PlotDataItem] = {}
        self.fft_curves: dict[int, pg.PlotDataItem] = {}
        self.crosstalk_sweep_curves: list[pg.PlotDataItem] = []
        self.gain_sweep_curves: list[pg.PlotDataItem] = []
        self.cmrr_sweep_curves: list[pg.PlotDataItem] = []
        self.exclusion_items: list[pg.LinearRegionItem] = []
        self._filter_cache_key: Optional[tuple[bool, bool, float]] = None
        self._filter_cache_data = None
        self._setting_region = False
        self._last_selection_matrix: Optional[np.ndarray] = None
        self._last_enabled_names: list[str] = []
        self._last_mean_names: list[str] = []
        self.source_metadata: dict[str, str] = {}
        self.recording_name = ""
        self.export_store = ReportSettingsStore()
        self.export_options = self.export_store.load()
        self.measurement = MeasurementInfo()
        # The bias reference curve is bench calibration, not per-recording data, so
        # it lives on the window and survives loading a different CSV.
        self.bias_reference_curve: Optional[ReferenceCurve] = None
        self._bias_dialog: Optional[BiasReferenceDialog] = None

        self._build_ui()
        self._build_menu()
        # Establish the initial state of the four-mode selector: mode 4 is only
        # offered once there is a curve, and the explanatory line is filled in.
        self._refresh_bias_curve_controls()

    @staticmethod
    def _default_window_size() -> "QSize":
        """Largest useful window that still fits the screen it opens on.

        A fixed 1680x980 does not fit a 150% scaled 1600p display, and the left
        panel then squeezes its rows on top of each other.
        """

        screen = QApplication.primaryScreen()
        if screen is None:
            return QSize(1400, 900)
        available = screen.availableGeometry()
        width = max(1180, min(available.width() - 40, 1720))
        height = max(720, min(available.height() - 70, 1060))
        return QSize(width, height)

    def _build_menu(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        load_action = QAction("Load CSV", self)
        load_action.triggered.connect(self.load_csv)
        file_menu.addAction(load_action)

        export_menu = self.menuBar().addMenu("Export")
        export_csv_action = QAction("Export Results CSV", self)
        export_csv_action.triggered.connect(self.export_csv)
        export_menu.addAction(export_csv_action)

        export_png_action = QAction("Export PNG", self)
        export_png_action.triggered.connect(self.export_png)
        export_menu.addAction(export_png_action)

        export_pdf_action = QAction("Export PDF", self)
        export_pdf_action.triggered.connect(self.export_pdf)
        export_menu.addAction(export_pdf_action)

        export_menu.addSeparator()
        export_report_action = QAction("Export report (choose sections)...", self)
        export_report_action.triggered.connect(lambda: self.open_export_dialog("pdf"))
        export_menu.addAction(export_report_action)

        edit_menu = self.menuBar().addMenu("Edit")
        undo_action = QAction("Undo Last Edit", self)
        undo_action.triggered.connect(self.undo_last_edit)
        edit_menu.addAction(undo_action)

        restore_action = QAction("Restore Original Data", self)
        restore_action.triggered.connect(self.restore_original_data)
        edit_menu.addAction(restore_action)

        window_menu = self.menuBar().addMenu("Window")
        live_action = QAction("Live recorder", self)
        live_action.triggered.connect(self.open_live_recorder)
        window_menu.addAction(live_action)

        # Comparing a live recording with the SD-card copy is a diagnostic tool,
        # not part of the normal analysis flow, so it lives here rather than in
        # the layout: it opens on demand and closes again.
        tools_menu = self.menuBar().addMenu("Tools")
        compare_action = QAction("Compare two recordings...", self)
        compare_action.setToolTip(
            "Check a live recording against the board's SD-card copy: start/end offset, "
            "missing samples and timeline damage."
        )
        compare_action.triggered.connect(self.open_recording_comparison)
        tools_menu.addAction(compare_action)

        # Calibration for the CMRR work, not a daily tool: it turns several
        # reference recordings into the Vcm(f) curve the sweep needs.
        bias_action = QAction("Bias reference curve...", self)
        bias_action.setToolTip(
            "Average the unbuffered channel of several recordings and fit the result, so CMRR "
            "analysis knows the voltage the generator really delivered at each frequency."
        )
        bias_action.triggered.connect(self.open_bias_reference)
        tools_menu.addAction(bias_action)

    def open_bias_reference(self) -> None:
        """Open the bias reference curve builder (own top-level window)."""

        if self._bias_dialog is None:
            self._bias_dialog = BiasReferenceDialog(
                self,
                curve=self.bias_reference_curve,
                use_curve=self.cmrr_vcm_mode() == VCM_MONITOR_CURVE,
            )
            self._bias_dialog.curveChanged.connect(self._on_bias_curve_changed)
            self._bias_dialog.useCurveChanged.connect(self._on_bias_curve_use_changed)
            self._bias_dialog.finished.connect(self._on_bias_dialog_closed)
        self._bias_dialog.show()
        self._bias_dialog.raise_()
        self._bias_dialog.activateWindow()

    def _on_bias_dialog_closed(self, *_args) -> None:
        self._bias_dialog = None

    def _on_bias_curve_changed(self, curve: object) -> None:
        if isinstance(curve, ReferenceCurve) and len(curve):
            self.bias_reference_curve = curve
        self._refresh_bias_curve_controls()

    def _on_bias_curve_use_changed(self, enabled: bool) -> None:
        """The dialog's tick is a shortcut for the fourth CM voltage mode."""

        if enabled:
            self._set_cmrr_vcm_mode(VCM_MONITOR_CURVE)
            self._mark_sweep_stale_for("cmrr")
            return
        # Unticking means "stop using the curve". Fall back to the monitor mode when
        # a monitor channel is selected, because that is the mode that needs no curve.
        monitor = self.cmrr_monitor_combo.currentText()
        fallback = VCM_MONITOR if monitor and monitor != NO_MONITOR_CHANNEL else VCM_FLAT
        self._set_cmrr_vcm_mode(fallback)
        self._mark_sweep_stale_for("cmrr")

    def _refresh_bias_curve_controls(self) -> None:
        """State which of the four CM voltage definitions is in force."""

        combo = getattr(self, "cmrr_vcm_mode_combo", None)
        label = getattr(self, "cmrr_vcm_label", None)
        if combo is None or label is None:
            return
        curve = self.bias_reference_curve
        usable = curve is not None and len(curve)
        # The curve mode is only offered once there is a curve to use; the others are
        # always available because they need nothing but the recording.
        model = combo.model()
        item = model.item(3) if model is not None and model.rowCount() > 3 else None
        if item is not None:
            item.setEnabled(bool(usable))
        if not usable and self.cmrr_vcm_mode() == VCM_MONITOR_CURVE:
            monitor = self.cmrr_monitor_combo.currentText()
            self._set_cmrr_vcm_mode(
                VCM_MONITOR if monitor and monitor != NO_MONITOR_CHANNEL else VCM_FLAT
            )
        mode = self.cmrr_vcm_mode()
        if mode == VCM_MONITOR_CURVE:
            coverage = curve.coverage_hz() or (0.0, 0.0)
            label.setText(
                f"Bias curve ready: {curve.summary()}. Vcm is read from it at this recording's own "
                f"frequencies ({coverage[0]:g}-{coverage[1]:g} Hz); outside that band points are "
                "dropped rather than extrapolated."
            )
        elif mode in (VCM_MONITOR, VCM_SINC3):
            label.setText(VCM_MODE_LABELS[mode] + " - the exact values appear after computing.")
        else:
            label.setText("Flat: one amplitude for the whole sweep.")

    def open_recording_comparison(self) -> None:
        """Open the live-versus-SD comparison window (own top-level dialog)."""

        dialog = RecordingComparisonDialog(self)
        if self.dataset is not None:
            dialog.set_first_file(self.dataset.file_path)
        dialog.exec()

    def open_live_recorder(self) -> None:
        """Open the acquisition window, reusing this analyzer for its results."""

        from live_gui import LiveRecorderWindow

        if getattr(self, "_live_window", None) is None:
            self._live_window = LiveRecorderWindow()
            self._live_window.attach_analyzer(self)
        self._live_window.show()
        self._live_window.raise_()
        self._live_window.activateWindow()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)

        main_layout = QHBoxLayout(root)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # The controls need more height than a laptop screen has, and a plain
        # widget squashes its rows instead of scrolling. Wrap it like the live
        # window does so every row keeps its natural height.
        self.controls_panel = self._build_controls_panel()
        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.controls_scroll.setWidget(self.controls_panel)
        self.controls_scroll.setMinimumWidth(300)
        self.controls_scroll.setMaximumWidth(430)
        splitter.addWidget(self.controls_scroll)

        right_splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(right_splitter)

        self.waveform_viewbox = SelectableViewBox()
        self.waveform_plot = pg.PlotWidget(viewBox=self.waveform_viewbox)
        self.waveform_plot.setLabel("bottom", "Time", units="s")
        self.waveform_plot.setLabel("left", "Channels")
        self.waveform_plot.showGrid(x=True, y=True, alpha=0.18)
        self.waveform_plot.setMenuEnabled(False)
        self.waveform_plot.getPlotItem().setClipToView(True)

        waveform_section = QWidget()
        waveform_layout = QVBoxLayout(waveform_section)
        waveform_layout.setContentsMargins(0, 0, 0, 0)
        waveform_layout.setSpacing(6)
        waveform_layout.addWidget(self.waveform_plot)
        waveform_layout.addWidget(self._build_waveform_footer())
        right_splitter.addWidget(waveform_section)

        self.selection_region = pg.LinearRegionItem(
            values=(0.0, 1.0),
            orientation=pg.LinearRegionItem.Vertical,
            movable=True,
            brush=pg.mkBrush(37, 99, 235, 45),
            pen=pg.mkPen("#2563eb", width=1.6),
        )
        self.selection_region.hide()
        self.selection_region.setZValue(30)
        self.selection_region.sigRegionChangeFinished.connect(self._on_selection_region_changed)
        self.waveform_plot.addItem(self.selection_region)

        self.waveform_viewbox.selectionPreview.connect(self._preview_selection)
        self.waveform_viewbox.selectionFinished.connect(self._commit_selection)

        bottom_splitter = QSplitter(Qt.Orientation.Horizontal)
        right_splitter.addWidget(bottom_splitter)

        self.fft_plot = pg.PlotWidget()
        self.fft_plot.setLabel("bottom", "Frequency", units="Hz")
        self.fft_plot.setLabel("left", "Amplitude", units="uVrms")
        self.fft_plot.showGrid(x=True, y=True, alpha=0.28)
        self.fft_plot.setMenuEnabled(False)
        self.fft_marker = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#dc2626", width=1.2, style=Qt.PenStyle.DashLine))
        self.fft_marker.hide()
        self.fft_plot.addItem(self.fft_marker)

        fft_section = QWidget()
        fft_layout = QVBoxLayout(fft_section)
        fft_layout.setContentsMargins(0, 0, 0, 0)
        fft_layout.setSpacing(6)
        fft_layout.addWidget(self.fft_plot)
        fft_layout.addWidget(self._build_fft_footer())
        bottom_splitter.addWidget(fft_section)

        result_tabs = QTabWidget()
        self.metrics_table = QTableWidget()
        self.crosstalk_table = QTableWidget()
        result_tabs.addTab(self._build_metrics_page(), "Metrics")
        result_tabs.addTab(self.crosstalk_table, "Crosstalk")
        result_tabs.addTab(self._build_crosstalk_sweep_page(), "Crosstalk Sweep")
        result_tabs.addTab(self._build_gain_sweep_page(), "Gain Sweep")
        result_tabs.addTab(self._build_cmrr_sweep_page(), "CMRR Sweep")
        bottom_splitter.addWidget(result_tabs)

        splitter.setSizes([360, self._default_window_size().width() - 380])
        # The sweep tabs stack several figures plus their controls, so the lower
        # half needs real height and width or every figure is a 70 px slit.
        right_splitter.setSizes([520, 430])
        bottom_splitter.setSizes([660, 620])

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Load an OpenBCI CSV recording to begin.")

    def _build_waveform_footer(self) -> QWidget:
        footer = QWidget()
        layout = QHBoxLayout(footer)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.setSpacing(8)

        layout.addWidget(QLabel("Time Editing"))
        exclude_button = QPushButton("Exclude")
        exclude_button.clicked.connect(self.exclude_selected_region)
        layout.addWidget(exclude_button)
        delete_button = QPushButton("Delete")
        delete_button.clicked.connect(self.delete_selected_region)
        layout.addWidget(delete_button)
        keep_button = QPushButton("Keep Only")
        keep_button.clicked.connect(self.keep_only_selected_region)
        layout.addWidget(keep_button)
        undo_button = QPushButton("Undo")
        undo_button.clicked.connect(self.undo_last_edit)
        layout.addWidget(undo_button)
        restore_button = QPushButton("Restore")
        restore_button.clicked.connect(self.restore_original_data)
        layout.addWidget(restore_button)

        layout.addSpacing(12)
        layout.addWidget(QLabel("Vertical scale"))
        self.waveform_scale_combo = QComboBox()
        self.waveform_scale_combo.addItems(["25 uV/div", "50 uV/div", "100 uV/div", "250 uV/div", "500 uV/div", "1000 uV/div"])
        self.waveform_scale_combo.setCurrentText("100 uV/div")
        self.waveform_scale_combo.currentIndexChanged.connect(self._on_waveform_display_changed)
        layout.addWidget(self.waveform_scale_combo)

        self.waveform_grid_check = QCheckBox("Grid")
        self.waveform_grid_check.setChecked(True)
        self.waveform_grid_check.toggled.connect(self._on_waveform_display_changed)
        layout.addWidget(self.waveform_grid_check)

        self.waveform_labels_check = QCheckBox("Labels")
        self.waveform_labels_check.setChecked(True)
        self.waveform_labels_check.toggled.connect(self._on_waveform_display_changed)
        layout.addWidget(self.waveform_labels_check)

        layout.addStretch(1)
        return footer

    def _build_fft_footer(self) -> QWidget:
        footer = QWidget()
        layout = QHBoxLayout(footer)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.setSpacing(8)

        layout.addWidget(QLabel("Amplitude scale"))
        self.fft_amplitude_mode_combo = QComboBox()
        self.fft_amplitude_mode_combo.addItems(["Log 100 uVrms", "Log 1000 uVrms", "Linear auto"])
        self.fft_amplitude_mode_combo.currentIndexChanged.connect(self._on_fft_display_changed)
        layout.addWidget(self.fft_amplitude_mode_combo)

        self.fft_grid_check = QCheckBox("Grid")
        self.fft_grid_check.setChecked(True)
        self.fft_grid_check.toggled.connect(self._on_fft_display_changed)
        layout.addWidget(self.fft_grid_check)

        fft_help = QLabel("One-sided FFT in uVrms")
        layout.addWidget(fft_help)
        layout.addStretch(1)
        return footer

    def show_metrics_description(self) -> None:
        """Open the metric definitions in a scrollable popup."""

        dialog = getattr(self, "_metrics_help_dialog", None)
        if dialog is None:
            dialog = QDialog(self)
            dialog.setWindowTitle("Metric definitions")
            dialog.resize(860, 640)
            dialog_layout = QVBoxLayout(dialog)
            browser = QTextBrowser(dialog)
            browser.setHtml(METRICS_HELP_HTML)
            dialog_layout.addWidget(browser, 1)
            buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
            buttons.rejected.connect(dialog.reject)
            dialog_layout.addWidget(buttons)
            self._metrics_help_dialog = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _build_metrics_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Fundamental"))
        self.fundamental_mode_combo = QComboBox()
        self.fundamental_mode_combo.addItems(["Auto detect", "Manual Hz"])
        self.fundamental_mode_combo.currentIndexChanged.connect(self._on_fundamental_mode_changed)
        controls.addWidget(self.fundamental_mode_combo)

        self.manual_fundamental_spin = QDoubleSpinBox()
        self.manual_fundamental_spin.setRange(0.01, 100000.0)
        self.manual_fundamental_spin.setDecimals(4)
        self.manual_fundamental_spin.setValue(10.0)
        self.manual_fundamental_spin.setSuffix(" Hz")
        self.manual_fundamental_spin.valueChanged.connect(self._refresh_analysis)
        controls.addWidget(self.manual_fundamental_spin)
        controls.addStretch(1)
        self.metrics_help_button = QPushButton("Description")
        self.metrics_help_button.setToolTip(
            "How every metric in this table is calculated. Amplitudes are uVrms throughout."
        )
        self.metrics_help_button.clicked.connect(self.show_metrics_description)
        controls.addWidget(self.metrics_help_button)
        layout.addLayout(controls)

        layout.addWidget(self.metrics_table)
        self._on_fundamental_mode_changed()
        return page

    # -- sweep options, fitting and legends -------------------------------

    def _scroll_row(self, row: QHBoxLayout) -> QScrollArea:
        """Let a wide control row scroll sideways instead of demanding a wide panel.

        These rows hold a dozen controls, so their combined minimum width used to
        force the whole results tab - and with it the right-hand column - to about
        2400 px, which does not fit any monitor. Scrolling the row keeps every
        control reachable at any panel width.
        """

        holder = QWidget()
        holder.setLayout(row)
        row.setContentsMargins(0, 0, 0, 0)
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.Shape.NoFrame)
        area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        area.setWidget(holder)
        area.setFixedHeight(holder.sizeHint().height() + 18)
        return area

    def _add_sweep_options(self, prefix: str, layout, plot: pg.PlotWidget) -> None:
        """Hold time, mains exclusion and curve fitting, shared by all sweeps."""

        row = QHBoxLayout()
        row.addWidget(QLabel("Min hold"))
        hold_spin = QDoubleSpinBox()
        hold_spin.setRange(0.0, 30.0)
        hold_spin.setDecimals(2)
        hold_spin.setSingleStep(0.1)
        hold_spin.setValue(1.0)
        hold_spin.setSuffix(" s")
        hold_spin.setToolTip(
            "How long the generator holds one frequency. A step only counts once at least two "
            "consecutive windows agree on the tone, so the transition between steps is ignored."
        )
        row.addWidget(hold_spin)

        mains_button = QPushButton("Exclude 50/100 Hz")
        mains_button.setCheckable(True)
        mains_button.setToolTip("Leave the mains points out of the sweep, curve fit and 50 Hz value")
        row.addWidget(mains_button)

        downward_check = QCheckBox("Downward sweep")
        downward_check.setChecked(True)
        downward_check.setToolTip(
            "Start high and step down. The fundamental can then never move up, so the intermodulation "
            "products of the tone with the mains (100 +/- f, which are louder than the tone itself at low "
            "frequency) are not mistaken for the applied signal."
        )
        row.addWidget(downward_check)

        row.addWidget(QLabel("Lock"))
        lock_spin = QDoubleSpinBox()
        lock_spin.setRange(0.01, 20.0)
        lock_spin.setDecimals(2)
        lock_spin.setSingleStep(0.1)
        lock_spin.setValue(0.5)
        lock_spin.setSuffix(" Hz")
        lock_spin.setToolTip(
            "How far two windows may differ and still count as the same step. The generator must hold a "
            "frequency within this band for the whole minimum hold before the step is locked in."
        )
        row.addWidget(lock_spin)

        row.addWidget(QLabel("Min SNR"))
        snr_spin = QDoubleSpinBox()
        snr_spin.setRange(-40.0, 30.0)
        snr_spin.setDecimals(1)
        snr_spin.setSingleStep(1.0)
        snr_spin.setValue(-20.0)
        snr_spin.setSuffix(" dB")
        snr_spin.setToolTip(
            "Reject a window whose tracked tone is weaker than this against everything else in the window. "
            "The default (-20 dB) effectively disables it, because a high-CMRR point at low frequency is "
            "exactly a small tone among louder interferers."
        )
        row.addWidget(snr_spin)

        row.addWidget(QLabel("Curve fit"))
        fit_combo = QComboBox()
        for label, order in SWEEP_FIT_ORDERS:
            fit_combo.addItem(label, order)
        # A third-order fit in log10(f) is the useful default: it follows the
        # rolled-off response without chasing single noisy points.
        fit_combo.setCurrentIndex(max(fit_combo.findData(3), 0))
        fit_combo.setToolTip("Polynomial in log10(frequency), fitted through the swept points")
        row.addWidget(fit_combo)

        fit_check = QCheckBox("Show fit")
        fit_check.setChecked(True)
        row.addWidget(fit_check)

        fit_label = QLabel("")
        # Wrapping keeps a long "mean (min .. max)" summary from widening the
        # window; the label only has to be readable, not on one line.
        fit_label.setWordWrap(True)
        fit_label.setMinimumWidth(160)
        row.addWidget(fit_label, 1)
        layout.addWidget(self._scroll_row(row))

        setattr(self, f"{prefix}_sweep_hold_spin", hold_spin)
        setattr(self, f"{prefix}_sweep_mains_button", mains_button)
        setattr(self, f"{prefix}_sweep_downward_check", downward_check)
        setattr(self, f"{prefix}_sweep_lock_spin", lock_spin)
        setattr(self, f"{prefix}_sweep_snr_spin", snr_spin)
        setattr(self, f"{prefix}_sweep_fit_combo", fit_combo)
        setattr(self, f"{prefix}_sweep_fit_check", fit_check)
        setattr(self, f"{prefix}_sweep_fit_label", fit_label)
        setattr(self, f"{prefix}_sweep_fit_curves", [])

        for widget in (hold_spin, lock_spin, snr_spin):
            widget.valueChanged.connect(lambda *_: self._mark_sweep_stale_for(prefix))
        downward_check.toggled.connect(lambda *_: self._mark_sweep_stale_for(prefix))
        mains_button.toggled.connect(lambda *_: self._mark_sweep_stale_for(prefix))
        fit_combo.currentIndexChanged.connect(self._on_sweep_fit_changed)
        fit_check.toggled.connect(self._on_sweep_fit_changed)

        legend = plot.addLegend(offset=(-10, 10), labelTextSize="11pt", brush=(255, 255, 255, 210))
        legend.setColumnCount(2)
        setattr(self, f"{prefix}_sweep_legend", legend)

    def _sweep_info_text(self, prefix: str, head: str, accepted: int, rejected: int) -> str:
        """One-line status under the sweep controls."""

        parts = [head, f"{accepted} ok", f"{rejected} rej"]
        if self._sweep_exclude_mains(prefix):
            parts.append("50/100 Hz excluded")
        if getattr(self, f"{prefix}_sweep_downward_check").isChecked():
            parts.append("downward lock")
        parts.append("notch on")
        return " | ".join(parts)

    def _sweep_hold_seconds(self, prefix: str) -> float:
        spin = getattr(self, f"{prefix}_sweep_hold_spin", None)
        return float(spin.value()) if spin is not None else 0.0

    def _sweep_tracking_options(self, prefix: str) -> dict[str, object]:
        """Fundamental-tracking settings shared by the three sweeps."""

        return {
            "downward_only": bool(getattr(self, f"{prefix}_sweep_downward_check").isChecked()),
            "lock_tolerance_hz": float(getattr(self, f"{prefix}_sweep_lock_spin").value()),
            "min_local_snr_db": float(getattr(self, f"{prefix}_sweep_snr_spin").value()),
        }

    def _sweep_exclude_mains(self, prefix: str) -> bool:
        button = getattr(self, f"{prefix}_sweep_mains_button", None)
        return bool(button.isChecked()) if button is not None else False

    def _sweep_fit_order(self, prefix: str) -> int:
        combo = getattr(self, f"{prefix}_sweep_fit_combo", None)
        return int(combo.currentData() or 0) if combo is not None else 0

    def _mark_sweep_stale_for(self, prefix: str) -> None:
        """Throw away one sweep's results after its options changed."""

        setattr(self, f"{prefix}_sweep_df", pd.DataFrame())
        setattr(self, f"{prefix}_sweep_fit_df", pd.DataFrame())
        getattr(self, f"_clear_{prefix}_sweep_plot")()
        getattr(self, f"{prefix}_sweep_info_label").setText("Sweep stale - recompute")

    def _on_sweep_fit_changed(self, *_args) -> None:
        for prefix, builder in (
            ("cmrr", self._redraw_cmrr_sweep_fit),
            ("crosstalk", self._redraw_crosstalk_sweep_fit),
            ("gain", self._redraw_gain_sweep_fit),
        ):
            builder(prefix)

    @staticmethod
    def _fit_sweep_curve(
        frequencies: np.ndarray,
        values: np.ndarray,
        order: int,
        reference_hz: float = SWEEP_FIT_REFERENCE_HZ,
    ) -> Optional[dict[str, object]]:
        """Fit a polynomial in log10(frequency) and evaluate it at the reference."""

        if order <= 0:
            return None
        frequencies = np.asarray(frequencies, dtype=np.float64)
        values = np.asarray(values, dtype=np.float64)
        finite = np.isfinite(frequencies) & np.isfinite(values) & (frequencies > 0)
        frequencies = frequencies[finite]
        values = values[finite]
        if frequencies.size < order + 1 or np.unique(frequencies).size < order + 1:
            return None
        log_frequency = np.log10(frequencies)
        coefficients = np.polyfit(log_frequency, values, order)
        grid = np.linspace(log_frequency.min(), log_frequency.max(), 200)
        fitted = np.polyval(coefficients, grid)
        residuals = values - np.polyval(coefficients, log_frequency)
        total = values - values.mean()
        total_sum = float(np.sum(total**2))
        r_squared = 1.0 - float(np.sum(residuals**2)) / total_sum if total_sum > 0 else 1.0
        return {
            "order": int(order),
            "frequencies": np.power(10.0, grid),
            "values": fitted,
            "at_reference": float(np.polyval(coefficients, np.log10(reference_hz))),
            "r_squared": float(r_squared),
            "points": int(frequencies.size),
        }

    def _draw_sweep_fit(
        self,
        prefix: str,
        plot: Optional[pg.PlotWidget],
        frequency_column: str,
        value_column: str,
        channel_labels: Sequence[str],
        channel_column: str = "Channel",
    ) -> tuple[pd.DataFrame, list[dict[str, object]]]:
        """Fit one curve per channel; return the summary and the fits.

        ``plot=None`` computes and returns the fits without drawing them on the
        measurement figure - the CMRR tab shows the measured points and the fits in
        separate figures so neither obscures the other.

        ``channel_column`` names the column that holds the channel of each point:
        the crosstalk table calls it ``Target``. Matching the wrong name silently
        fitted every point of every channel into one curve.
        """

        if plot is not None:
            for curve in getattr(self, f"{prefix}_sweep_fit_curves", []):
                plot.removeItem(curve)
        setattr(self, f"{prefix}_sweep_fit_curves", [])

        frame = getattr(self, f"{prefix}_sweep_df", pd.DataFrame())
        label = getattr(self, f"{prefix}_sweep_fit_label", None)
        order = self._sweep_fit_order(prefix)
        show = getattr(self, f"{prefix}_sweep_fit_check", None)
        visible = show is None or show.isChecked()
        rows: list[dict[str, object]] = []
        fits: list[dict[str, object]] = []
        empty = pd.DataFrame(columns=["Channel", "Fit Order", "Fit @ 50 Hz", "R2", "Points"])
        if frame.empty or order <= 0:
            if label is not None:
                label.setText("" if order <= 0 or frame.empty else "fit hidden")
            return empty, fits

        value_name = f"Fit @ {SWEEP_FIT_REFERENCE_HZ:g} Hz"
        curves = getattr(self, f"{prefix}_sweep_fit_curves", [])
        for index, channel in enumerate(channel_labels):
            if channel_column in frame:
                subset = frame.loc[frame[channel_column] == channel]
            else:
                subset = frame
            if subset.empty:
                continue
            fit = self._fit_sweep_curve(
                subset[frequency_column].to_numpy(), subset[value_column].to_numpy(), order
            )
            if fit is None:
                continue
            color = pg.intColor(index, hues=max(8, len(channel_labels)), values=1.0)
            if plot is not None and visible:
                # No ``name``: the fit must not double every legend entry.
                curves.append(
                    plot.plot(
                        np.asarray(fit["frequencies"]),
                        np.asarray(fit["values"]),
                        pen=pg.mkPen(color, width=SWEEP_FIT_WIDTH, style=Qt.PenStyle.DashLine),
                    )
                )
            fits.append({"channel": channel, "color": color, "fit": fit})
            rows.append(
                {
                    "Channel": channel,
                    "Fit Order": int(order),
                    value_name: float(fit["at_reference"]),
                    "R2": float(fit["r_squared"]),
                    "Points": int(fit["points"]),
                }
            )
        setattr(self, f"{prefix}_sweep_fit_curves", curves)

        summary = pd.DataFrame(rows)
        if label is not None:
            if summary.empty:
                label.setText(f"order {order}: not enough swept points")
            else:
                values = summary[value_name]
                if len(values) == 1:
                    label.setText(f"{value_name}: {values.iloc[0]:.2f} dB")
                else:
                    label.setText(
                        f"{value_name}: mean {values.mean():.2f} dB "
                        f"({values.min():.2f} .. {values.max():.2f})"
                    )
        return summary, fits

    def _refresh_cmrr_reference_figure(self, monitor_name: str, common_mode_vrms: float) -> None:
        """Plot what the reference input really measured, in Vrms.

        Comparing this with the amplitude the generator was set to is the whole
        point of the monitor channel: the difference is what the bias network and
        the sinc filter did to the common-mode voltage.
        """

        self.cmrr_reference_plot.clear()
        self.cmrr_reference_df = pd.DataFrame()
        if not monitor_name or monitor_name == NO_MONITOR_CHANNEL:
            return
        if monitor_name not in self._last_enabled_names or self._last_selection_matrix is None:
            return

        sample_rate = self.dataset.sample_rate if self.dataset else 1.0
        column = self._last_selection_matrix[:, self._last_enabled_names.index(monitor_name)]
        steps = parse_sweep_steps(self.source_metadata)
        rows: list[dict[str, float]] = []
        if steps:
            for step in steps:
                start = int(round(step.usable_start_s * sample_rate))
                end = int(round(step.end_s * sample_rate))
                if end <= start:
                    continue
                segment = np.asarray(column[start:end], dtype=np.float64)
                if segment.size < 8:
                    continue
                rms_uv = fit_tone_rms(segment, sample_rate, step.frequency_hz)
                rows.append(
                    {
                        "Frequency (Hz)": float(step.frequency_hz),
                        "Reference (Vrms)": rms_uv * 1e-6,
                        "Applied (Vrms)": float(step.amplitude_vrms),
                    }
                )
        else:
            # No recorded schedule: fall back to the dominant tone of the window.
            frame = self._last_selection_matrix
            window = max(int(round(sample_rate * 0.5)), 16)
            for start in range(0, max(frame.shape[0] - window, 1), window):
                segment = np.asarray(column[start : start + window], dtype=np.float64)
                frequency = detect_tone_frequency(segment, sample_rate)
                if frequency is None:
                    continue
                rms_uv = fit_tone_rms(segment, sample_rate, frequency)
                rows.append(
                    {
                        "Frequency (Hz)": float(frequency),
                        "Reference (Vrms)": rms_uv * 1e-6,
                        "Applied (Vrms)": float(common_mode_vrms),
                    }
                )
        if not rows:
            return

        self.cmrr_reference_df = pd.DataFrame(rows).sort_values("Frequency (Hz)").reset_index(drop=True)
        self.cmrr_reference_fit_hz = float("nan")
        order = self._sweep_fit_order("cmrr")
        fit = self._fit_sweep_curve(
            self.cmrr_reference_df["Frequency (Hz)"].to_numpy(),
            self.cmrr_reference_df["Reference (Vrms)"].to_numpy(),
            max(order, 2),
        )
        self.cmrr_reference_plot.plot(
            self.cmrr_reference_df["Frequency (Hz)"],
            self.cmrr_reference_df["Reference (Vrms)"],
            pen=pg.mkPen((16, 120, 90), width=SWEEP_TRACE_WIDTH),
            symbol="o",
            symbolSize=5,
            name=f"{monitor_name} measured",
        )
        self.cmrr_reference_plot.plot(
            self.cmrr_reference_df["Frequency (Hz)"],
            self.cmrr_reference_df["Applied (Vrms)"],
            pen=pg.mkPen((190, 90, 60), width=1.4, style=Qt.PenStyle.DashLine),
            name="generator set",
        )
        if fit is not None:
            # The fit is what makes the loss curve readable: the measured points
            # scatter, the trend is the response of the bias network plus sinc
            # filter. Its value at 50 Hz is also reported in the info line.
            self.cmrr_reference_fit_hz = float(fit["at_reference"])
            self.cmrr_reference_plot.plot(
                np.asarray(fit["frequencies"]),
                np.asarray(fit["values"]),
                pen=pg.mkPen((16, 120, 90), width=SWEEP_FIT_WIDTH, style=Qt.PenStyle.DashLine),
                name=f"fit (R^2 {float(fit['r_squared']):.3f})",
            )
            self.cmrr_reference_df["Reference fit (Vrms)"] = np.interp(
                self.cmrr_reference_df["Frequency (Hz)"].to_numpy(),
                np.asarray(fit["frequencies"]),
                np.asarray(fit["values"]),
            )

        self._overlay_bias_curve_vcm(common_mode_vrms)
        self.cmrr_reference_plot.setXRange(0.0, SWEEP_X_MAX_HZ, padding=0.0)
        legend = self.cmrr_reference_plot.addLegend(offset=(-10, 10))
        legend.setBrush(pg.mkBrush(255, 255, 255, 190))

    def _overlay_bias_curve_vcm(self, common_mode_vrms: float) -> None:
        """Draw the Vcm the chosen mode supplied, at the frequencies measured.

        This is the figure that answers "what did the correction do". The monitor
        channel shows what reached the input - small, when a bias network is in the
        way - while the blue trace is the voltage CMRR was actually divided by. The
        gap between them *is* the bias network's attenuation.
        """

        plot = self.cmrr_reference_plot
        curve = self.bias_reference_curve
        mode = self.cmrr_vcm_mode()
        self.cmrr_bias_overlay = None
        if mode not in (VCM_MONITOR_CURVE, VCM_MONITOR, VCM_SINC3):
            plot.setLogMode(x=False, y=False)
            return
        if mode == VCM_MONITOR_CURVE and (curve is None or not len(curve)):
            plot.setLogMode(x=False, y=False)
            return

        # The same factor the analysis used, so this figure cannot disagree with the
        # results table. Stored rather than recomputed: the two must not drift apart.
        scale = float(getattr(self, "cmrr_vcm_scale", 1.0) or 1.0)

        if mode == VCM_MONITOR_CURVE:
            # Dense curve across the measured band, so the underlying fit is visible.
            grid, values = curve.fitted()
            if grid.size:
                plot.plot(
                    grid,
                    np.asarray(values, dtype=float) * scale,
                    pen=pg.mkPen((37, 99, 235), width=SWEEP_FIT_WIDTH, style=Qt.PenStyle.DashLine),
                    name="bias curve",
                )

        # The exact value used at each swept frequency: the same numbers the CMRR
        # arithmetic used, so this figure cannot disagree with the results table.
        frame = self.cmrr_sweep_df
        overlay = pd.DataFrame()
        if not frame.empty and "Vcm Used (Vrms)" in frame.columns:
            overlay = (
                frame.groupby("Frequency (Hz)", as_index=False)["Vcm Used (Vrms)"]
                .mean()
                .sort_values("Frequency (Hz)")
                .reset_index(drop=True)
            )
        if overlay.empty:
            coverage = curve.coverage_hz()
            if coverage is not None:
                frequencies = np.linspace(coverage[0], coverage[1], 60)
                overlay = pd.DataFrame(
                    {
                        "Frequency (Hz)": frequencies,
                        "Vcm Used (Vrms)": curve.evaluate(frequencies) * scale,
                    }
                )
        if not overlay.empty:
            plot.plot(
                overlay["Frequency (Hz)"],
                overlay["Vcm Used (Vrms)"],
                pen=pg.mkPen((37, 99, 235), width=SWEEP_TRACE_WIDTH),
                symbol="s",
                symbolSize=6,
                name="Vcm used for CMRR",
            )
            self.cmrr_bias_overlay = overlay
            # Carry it in the reference table too, so the export and the report
            # show what each measured point was divided by.
            if not self.cmrr_reference_df.empty:
                self.cmrr_reference_df["Vcm from curve (Vrms)"] = np.interp(
                    self.cmrr_reference_df["Frequency (Hz)"].to_numpy(dtype=float),
                    overlay["Frequency (Hz)"].to_numpy(dtype=float),
                    overlay["Vcm Used (Vrms)"].to_numpy(dtype=float),
                )

        # Log y when the bias network attenuates hard: 0.03 V and 0.95 V cannot be
        # read on one linear axis, and hiding one of them is what made the curve
        # look like it did nothing.
        measured = self.cmrr_reference_df.get("Reference (Vrms)")
        ratio_db = 0.0
        if measured is not None and len(measured) and not overlay.empty:
            measured_values = np.asarray(measured, dtype=float)
            curve_values = np.asarray(overlay["Vcm Used (Vrms)"], dtype=float)
            count = min(measured_values.size, curve_values.size)
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = measured_values[:count] / curve_values[:count]
            finite = ratio[np.isfinite(ratio) & (ratio > 0)]
            if finite.size:
                ratio_db = float(np.median(20.0 * np.log10(finite)))
        plot.setLogMode(x=False, y=bool(ratio_db < -6.0))

        if abs(ratio_db) >= 0.5:
            axis = plot.getAxis("left")
            axis.setLabel("Reference", units="Vrms" + (" (log)" if ratio_db < -6.0 else ""))
            self.cmrr_reference_note = (
                f"Blue trace is the voltage CMRR was divided by, {abs(ratio_db):.1f} dB above what "
                f"reached the input ({10 ** (ratio_db / 20.0):.4g}x): that gap is the bias "
                "network's attenuation."
            )
        else:
            self.cmrr_reference_note = (
                "The measured reference and the blue trace agree to within "
                f"{abs(ratio_db):.1f} dB, so nothing is attenuating the common-mode voltage here."
            )
        label = getattr(self, "cmrr_reference_note_label", None)
        if label is not None:
            label.setText(self.cmrr_reference_note)

    def _redraw_cmrr_sweep_fit(self, prefix: str = "cmrr") -> None:
        # Fits go to their own figure, never on top of the measured points.
        summary, fits = self._draw_sweep_fit(
            "cmrr", None, "Frequency (Hz)", "CMRR (dB)", self._last_enabled_names
        )
        self.cmrr_sweep_fit_df = summary
        self._refresh_cmrr_fit_figures(fits)

    def _refresh_cmrr_fit_figures(self, fits: Sequence[dict[str, object]]) -> None:
        """Draw the fit-only figure, the mean-of-fits figure and the 50 Hz table.

        These are what the report shows: the fitted response on its own (without
        the measurement clutter), the average fitted response with the channel
        spread, and the numeric 50 Hz value per channel.
        """

        self.cmrr_fit_plot.clear()
        self.cmrr_fit_mean_plot.clear()
        self.cmrr_50hz_table.clear()
        show = getattr(self, "cmrr_sweep_fit_check", None)
        drawing = show is None or show.isChecked()
        if not fits or not drawing:
            return

        for entry in fits:
            fit = entry["fit"]
            color = entry["color"]
            curve = self.cmrr_fit_plot.plot(
                np.asarray(fit["frequencies"]),
                np.asarray(fit["values"]),
                pen=pg.mkPen(color, width=SWEEP_TRACE_WIDTH + 0.4),
                name=str(entry["channel"]),
            )
            curve.setToolTip(str(entry["channel"]))
        self.cmrr_fit_plot.setXRange(0.0, SWEEP_X_MAX_HZ, padding=0.0)
        legend = self.cmrr_fit_plot.addLegend(offset=(-10, 10))
        legend.setBrush(pg.mkBrush(255, 255, 255, 190))

        # Mean of the fits on a common grid. Each fit is only defined over its own
        # frequency range, so the mean is taken where at least one fit exists and
        # the spread is the min/max envelope of the fits that cover that point.
        greatest = max(np.asarray(entry["fit"]["frequencies"]).size for entry in fits)
        low = min(float(np.min(entry["fit"]["frequencies"])) for entry in fits)
        high = max(float(np.max(entry["fit"]["frequencies"])) for entry in fits)
        grid = np.logspace(np.log10(max(low, 1e-3)), np.log10(max(high, low * 1.01)), max(greatest, 64))
        stacked = np.full((len(fits), grid.size), np.nan)
        for row, entry in enumerate(fits):
            fitted = entry["fit"]
            stacked[row, :] = np.interp(
                grid,
                np.asarray(fitted["frequencies"]),
                np.asarray(fitted["values"]),
                left=np.nan,
                right=np.nan,
            )
        with np.errstate(invalid="ignore"):
            mean_curve = np.nanmean(stacked, axis=0)
            low_curve = np.nanmin(stacked, axis=0)
            high_curve = np.nanmax(stacked, axis=0)
        self.cmrr_fit_mean_plot.plot(
            grid, low_curve, pen=pg.mkPen((120, 130, 150), width=1.2, style=Qt.PenStyle.DotLine),
            name="min" if len(fits) > 1 else "",
        )
        self.cmrr_fit_mean_plot.plot(
            grid, high_curve, pen=pg.mkPen((120, 130, 150), width=1.2, style=Qt.PenStyle.DotLine)
        )
        self.cmrr_fit_mean_plot.plot(
            grid, mean_curve, pen=pg.mkPen((30, 90, 160), width=SWEEP_TRACE_WIDTH + 0.8), name="mean of fits"
        )
        self.cmrr_fit_mean_plot.setXRange(0.0, SWEEP_X_MAX_HZ, padding=0.0)
        mean_legend = self.cmrr_fit_mean_plot.addLegend(offset=(-10, 10))
        mean_legend.setBrush(pg.mkBrush(255, 255, 255, 190))

        self.cmrr_mean_fit_hz = float(
            np.interp(SWEEP_FIT_REFERENCE_HZ, grid, mean_curve)
            if np.isfinite(mean_curve).any()
            else float("nan")
        )
        self.cmrr_mean_fit_df = pd.DataFrame(
            {
                "Frequency (Hz)": grid,
                "Mean CMRR (dB)": mean_curve,
                "Min CMRR (dB)": low_curve,
                "Max CMRR (dB)": high_curve,
            }
        )

        value_name = f"Fit @ {SWEEP_FIT_REFERENCE_HZ:g} Hz"
        frame = self.cmrr_sweep_fit_df
        if frame.empty:
            return
        table = frame[["Channel", value_name, "R2", "Points"]].copy()
        mean_value = float(table[value_name].mean())
        table.loc[len(table)] = ["Mean of fits", mean_value, float(table["R2"].mean()), int(table["Points"].sum())]
        self._populate_table(self.cmrr_50hz_table, table)

    def _redraw_crosstalk_sweep_fit(self, prefix: str = "crosstalk") -> None:
        # _draw_sweep_fit returns (summary_frame, fits); assigning the tuple here
        # left this attribute unusable as a DataFrame.
        summary, _fits = self._draw_sweep_fit(
            "crosstalk",
            self.crosstalk_sweep_plot,
            "Frequency (Hz)",
            "Crosstalk (dB)",
            self._last_enabled_names,
            channel_column="Target",
        )
        self.crosstalk_sweep_fit_df = summary

    def _redraw_gain_sweep_fit(self, prefix: str = "gain") -> None:
        summary, _fits = self._draw_sweep_fit(
            "gain", self.gain_sweep_plot, "Frequency (Hz)", "Gain (dB)", self._last_enabled_names
        )
        self.gain_sweep_fit_df = summary

    def _build_cmrr_sweep_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        control_row = QHBoxLayout()
        compute_button = QPushButton("Compute Sweep From Selection")
        compute_button.clicked.connect(self.compute_cmrr_sweep_view)
        control_row.addWidget(compute_button)
        control_row.addWidget(QLabel("Window"))
        self.cmrr_sweep_window_spin = QDoubleSpinBox()
        self.cmrr_sweep_window_spin.setRange(0.01, 600.0)
        self.cmrr_sweep_window_spin.setDecimals(3)
        self.cmrr_sweep_window_spin.setSingleStep(0.05)
        self.cmrr_sweep_window_spin.setValue(0.75)
        self.cmrr_sweep_window_spin.setSuffix(" s")
        control_row.addWidget(self.cmrr_sweep_window_spin)
        control_row.addWidget(QLabel("Stability"))
        self.cmrr_sweep_tolerance_spin = QDoubleSpinBox()
        self.cmrr_sweep_tolerance_spin.setRange(0.0001, 1000.0)
        self.cmrr_sweep_tolerance_spin.setDecimals(4)
        self.cmrr_sweep_tolerance_spin.setSingleStep(0.01)
        self.cmrr_sweep_tolerance_spin.setValue(0.20)
        self.cmrr_sweep_tolerance_spin.setSuffix(" Hz")
        control_row.addWidget(self.cmrr_sweep_tolerance_spin)
        control_row.addWidget(QLabel("CM monitor"))
        self.cmrr_monitor_combo = QComboBox()
        self.cmrr_monitor_combo.setMinimumWidth(110)
        self.cmrr_monitor_combo.addItem(NO_MONITOR_CHANNEL)
        self.cmrr_monitor_combo.setToolTip(
            "Channel wired straight across the common-mode voltage. Its dominant peak is the applied "
            "generator frequency, so every other channel is evaluated at exactly that frequency. "
            "Without it the sweep has to guess the frequency from the channel under test, which fails "
            "below about 70 Hz where the mixing products are louder than the tone."
        )
        control_row.addWidget(self.cmrr_monitor_combo)
        self.cmrr_sweep_info_label = QLabel("Select region and set Vcm (Vrms)")
        self.cmrr_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.cmrr_sweep_info_label)
        control_row.addStretch(1)
        self.cmrr_reset_zoom_button = QPushButton("Reset zoom")
        self.cmrr_reset_zoom_button.setToolTip(
            "Undo any wheel zoom on the four CMRR figures below."
        )
        self.cmrr_reset_zoom_button.clicked.connect(self._reset_cmrr_zoom)
        control_row.addWidget(self.cmrr_reset_zoom_button)
        layout.addWidget(self._scroll_row(control_row))

        # How the common-mode voltage is established. Four distinct definitions,
        # because they answer four different questions and the difference between
        # them is tens of dB - not a setting to be guessed at.
        vcm_row = QHBoxLayout()
        vcm_row.addWidget(QLabel("CM voltage"))
        self.cmrr_vcm_mode_combo = QComboBox()
        self.cmrr_vcm_mode_combo.setMinimumWidth(330)
        self.cmrr_vcm_mode_combo.addItem("Flat: the constant amplitude above", VCM_FLAT)
        self.cmrr_vcm_mode_combo.addItem(
            "Generator amplitude with the ADS1299 sinc3 roll-off", VCM_SINC3
        )
        self.cmrr_vcm_mode_combo.addItem("Measured on the CM monitor channel", VCM_MONITOR)
        self.cmrr_vcm_mode_combo.addItem(
            "CM monitor frequencies, voltage from the bias curve", VCM_MONITOR_CURVE
        )
        self.cmrr_vcm_mode_combo.setToolTip(
            "Flat: one number for the whole sweep.\n"
            "Sinc3: that number reduced with frequency by the ADS1299 digital filter at the "
            "recording's data rate.\n"
            "Monitor: the voltage actually measured on the monitor channel, per frequency - the "
            "converter already measured it, so no sinc3 correction is applied.\n"
            "Monitor + bias curve: frequency and timing from the monitor channel, but the "
            "voltage taken from the fitted bias reference curve. This is the one to use when a "
            "bias network attenuates the stimulus: the sweep still knows where it is, and the "
            "voltage is the value the generator really delivered."
        )
        self.cmrr_vcm_mode_combo.currentIndexChanged.connect(self._on_vcm_mode_changed)
        vcm_row.addWidget(self.cmrr_vcm_mode_combo, 1)
        bias_button = QPushButton("Bias curve...")
        bias_button.setToolTip(
            "Average several unbuffered-channel recordings into the Vcm(f) curve used by the "
            "last mode."
        )
        bias_button.clicked.connect(self.open_bias_reference)
        vcm_row.addWidget(bias_button)
        layout.addWidget(self._scroll_row(vcm_row))

        self.cmrr_vcm_label = QLabel("")
        self.cmrr_vcm_label.setWordWrap(True)
        self.cmrr_vcm_label.setStyleSheet("color: #475569; font-size: 11px;")
        self.cmrr_vcm_label.setMinimumHeight(30)
        layout.addWidget(self.cmrr_vcm_label)

        # The four figures are stacked in a scroll area: four plots do not fit on a
        # screen at a readable size, and a scroll area keeps their height honest
        # instead of squeezing them into slivers.
        self.cmrr_figures = QWidget()
        figures = QVBoxLayout(self.cmrr_figures)
        figures.setContentsMargins(0, 0, 0, 0)
        figures.setSpacing(6)

        self.cmrr_sweep_plot = self._make_sweep_figure(
            figures,
            "Measured CMRR per channel",
            "Frequency",
            "Hz",
            "CMRR",
            "dB",
        )
        self.cmrr_fit_plot = self._make_sweep_figure(
            figures,
            "Fitted curves (one per channel)",
            "Frequency",
            "Hz",
            "CMRR fit",
            "dB",
        )
        self.cmrr_fit_mean_plot = self._make_sweep_figure(
            figures,
            "Mean of the fitted curves with the channel spread",
            "Frequency",
            "Hz",
            "Mean of fits",
            "dB",
        )
        self.cmrr_reference_plot = self._make_sweep_figure(
            figures,
            "Reference input actually measured, and its fit",
            "Frequency",
            "Hz",
            "Reference input",
            "Vrms",
        )
        self.cmrr_reference_note_label = QLabel("")
        self.cmrr_reference_note_label.setWordWrap(True)
        self.cmrr_reference_note_label.setStyleSheet("color: #475569; font-size: 11px;")
        figures.addWidget(self.cmrr_reference_note_label)
        figures.addStretch(1)

        self.cmrr_figure_scroll = PlotScrollArea()
        self.cmrr_figure_scroll.setWidget(self.cmrr_figures)
        self.cmrr_figure_scroll.setToolTip(
            "Mouse wheel zooms the figure under the cursor, Shift+wheel scrolls."
        )
        self._add_sweep_options("cmrr", layout, self.cmrr_sweep_plot)
        layout.addWidget(self.cmrr_figure_scroll, 1)

        layout.addWidget(QLabel(f"CMRR at {SWEEP_FIT_REFERENCE_HZ:g} Hz from each channel's fitted curve"))
        self.cmrr_50hz_table = QTableWidget()
        self.cmrr_50hz_table.setMaximumHeight(100)
        layout.addWidget(self.cmrr_50hz_table, 0)
        return page

    def _make_sweep_figure(
        self,
        layout: QVBoxLayout,
        title: str,
        bottom_label: str,
        bottom_units: str,
        left_label: str,
        left_units: str,
    ) -> pg.PlotWidget:
        """One titled, zoomable figure inside the scrollable sweep area."""

        heading = QLabel(title)
        heading.setStyleSheet("font-weight: 600; color: #1e293b;")
        layout.addWidget(heading)
        plot = pg.PlotWidget()
        plot.setMinimumHeight(SWEEP_PLOT_HEIGHT)
        plot.setLabel("bottom", bottom_label, units=bottom_units)
        plot.setLabel("left", left_label, units=left_units)
        plot.showGrid(x=True, y=True, alpha=0.28)
        plot.setXRange(0.0, SWEEP_X_MAX_HZ, padding=0.0)
        # Left-drag pans, right-drag scales; the wheel is handled by the scroll
        # area so it zooms instead of scrolling.
        plot.setMouseEnabled(x=True, y=True)
        layout.addWidget(plot)
        return plot

    def _reset_cmrr_zoom(self) -> None:
        for name in ("cmrr_sweep_plot", "cmrr_fit_plot", "cmrr_fit_mean_plot", "cmrr_reference_plot"):
            plot = getattr(self, name, None)
            if plot is None:
                continue
            plot.getPlotItem().getViewBox().enableAutoRange()
            plot.setXRange(0.0, SWEEP_X_MAX_HZ, padding=0.0)

    def _build_gain_sweep_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        control_row = QHBoxLayout()
        compute_button = QPushButton("Compute Sweep From Selection")
        compute_button.clicked.connect(self.compute_gain_sweep_view)
        control_row.addWidget(compute_button)

        control_row.addWidget(QLabel("Channel"))
        self.gain_sweep_channel_combo = QComboBox()
        self.gain_sweep_channel_combo.currentIndexChanged.connect(self._mark_gain_sweep_stale)
        control_row.addWidget(self.gain_sweep_channel_combo)

        control_row.addWidget(QLabel("Applied (Vrms, before attenuator)"))
        self.gain_sweep_input_spin = QDoubleSpinBox()
        self.gain_sweep_input_spin.setRange(0.0, 1_000_000_000.0)
        self.gain_sweep_input_spin.setDecimals(9)
        self.gain_sweep_input_spin.setValue(0.0)
        self.gain_sweep_input_spin.setSuffix(" Vrms")
        self.gain_sweep_input_spin.valueChanged.connect(self._mark_gain_sweep_stale)
        control_row.addWidget(self.gain_sweep_input_spin)

        control_row.addWidget(QLabel("Window"))
        self.gain_sweep_window_spin = QDoubleSpinBox()
        self.gain_sweep_window_spin.setRange(0.01, 600.0)
        self.gain_sweep_window_spin.setDecimals(3)
        self.gain_sweep_window_spin.setSingleStep(0.05)
        self.gain_sweep_window_spin.setValue(0.75)
        self.gain_sweep_window_spin.setSuffix(" s")
        self.gain_sweep_window_spin.valueChanged.connect(self._mark_gain_sweep_stale)
        control_row.addWidget(self.gain_sweep_window_spin)

        control_row.addWidget(QLabel("Stability"))
        self.gain_sweep_tolerance_spin = QDoubleSpinBox()
        self.gain_sweep_tolerance_spin.setRange(0.0001, 1000.0)
        self.gain_sweep_tolerance_spin.setDecimals(4)
        self.gain_sweep_tolerance_spin.setSingleStep(0.01)
        self.gain_sweep_tolerance_spin.setValue(0.20)
        self.gain_sweep_tolerance_spin.setSuffix(" Hz")
        self.gain_sweep_tolerance_spin.valueChanged.connect(self._mark_gain_sweep_stale)
        control_row.addWidget(self.gain_sweep_tolerance_spin)

        self.gain_attenuator_label = QLabel("")
        self.gain_attenuator_label.setToolTip("")
        control_row.addWidget(self.gain_attenuator_label)
        self._refresh_attenuator_label()

        self.gain_sweep_info_label = QLabel("Select region, input, channel")
        self.gain_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.gain_sweep_info_label)
        control_row.addStretch(1)
        layout.addWidget(self._scroll_row(control_row))

        self.gain_sweep_plot = pg.PlotWidget()
        self.gain_sweep_plot.setLabel("bottom", "Frequency", units="Hz")
        self.gain_sweep_plot.setLabel("left", "Gain", units="dB")
        self.gain_sweep_plot.showGrid(x=True, y=True, alpha=0.28)
        self._add_sweep_options("gain", layout, self.gain_sweep_plot)
        layout.addWidget(self._scrollable_figure(self.gain_sweep_plot), 3)

        return page

    def _scrollable_figure(self, plot: pg.PlotWidget, minimum_height: int = SWEEP_PLOT_HEIGHT):
        """Put one sweep figure in the same zoomable, scrolling frame as the CMRR tab."""

        plot.setMinimumHeight(minimum_height)
        holder = QWidget()
        holder_layout = QVBoxLayout(holder)
        holder_layout.setContentsMargins(0, 0, 0, 0)
        holder_layout.addWidget(plot)
        area = PlotScrollArea()
        area.setWidget(holder)
        area.setToolTip("Mouse wheel zooms the figure, Shift+wheel scrolls.")
        return area

    def _build_crosstalk_sweep_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        control_row = QHBoxLayout()
        compute_button = QPushButton("Compute Sweep From Selection")
        compute_button.clicked.connect(self.compute_crosstalk_sweep_view)
        control_row.addWidget(compute_button)
        control_row.addWidget(QLabel("Window"))
        self.crosstalk_sweep_window_spin = QDoubleSpinBox()
        self.crosstalk_sweep_window_spin.setRange(0.01, 600.0)
        self.crosstalk_sweep_window_spin.setDecimals(3)
        self.crosstalk_sweep_window_spin.setSingleStep(0.05)
        self.crosstalk_sweep_window_spin.setValue(0.75)
        self.crosstalk_sweep_window_spin.setSuffix(" s")
        control_row.addWidget(self.crosstalk_sweep_window_spin)
        control_row.addWidget(QLabel("Stability"))
        self.crosstalk_sweep_tolerance_spin = QDoubleSpinBox()
        self.crosstalk_sweep_tolerance_spin.setRange(0.0001, 1000.0)
        self.crosstalk_sweep_tolerance_spin.setDecimals(4)
        self.crosstalk_sweep_tolerance_spin.setSingleStep(0.01)
        self.crosstalk_sweep_tolerance_spin.setValue(0.20)
        self.crosstalk_sweep_tolerance_spin.setSuffix(" Hz")
        control_row.addWidget(self.crosstalk_sweep_tolerance_spin)
        control_row.addWidget(QLabel("Reference"))
        self.crosstalk_reference_combo = QComboBox()
        self.crosstalk_reference_combo.setMinimumWidth(110)
        self.crosstalk_reference_combo.addItem(NO_MONITOR_CHANNEL)
        self.crosstalk_reference_combo.setToolTip(
            "Channel that measures the applied stimulus. It is left out of the crosstalk curves, "
            "because its level is the stimulus itself and a crosstalk figure for it would be "
            "meaningless.\n"
            "Default: the channel the recorder marked as its reference."
        )
        self.crosstalk_reference_combo.currentIndexChanged.connect(
            lambda *_: self._mark_sweep_stale_for("crosstalk")
        )
        control_row.addWidget(self.crosstalk_reference_combo)
        self.crosstalk_sweep_info_label = QLabel("Select region and source")
        self.crosstalk_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.crosstalk_sweep_info_label)
        control_row.addStretch(1)
        layout.addWidget(self._scroll_row(control_row))

        self.crosstalk_sweep_plot = pg.PlotWidget()
        self.crosstalk_sweep_plot.setLabel("bottom", "Frequency", units="Hz")
        self.crosstalk_sweep_plot.setLabel("left", "Crosstalk", units="dB")
        self.crosstalk_sweep_plot.showGrid(x=True, y=True, alpha=0.28)
        self._add_sweep_options("crosstalk", layout, self.crosstalk_sweep_plot)
        layout.addWidget(self._scrollable_figure(self.crosstalk_sweep_plot), 3)

        return page

    def _build_controls_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMaximumWidth(290)
        layout = QVBoxLayout(panel)

        import_group = QGroupBox("Import")
        import_layout = QHBoxLayout(import_group)
        load_button = QPushButton("Load CSV")
        load_button.clicked.connect(self.load_csv)
        import_layout.addWidget(load_button)
        self.file_label = QLabel("No file")
        self.file_label.setWordWrap(False)
        import_layout.addWidget(self.file_label)
        layout.addWidget(import_group)

        session_group = QGroupBox("Recording")
        session_layout = QVBoxLayout(session_group)
        self.session_label = QLabel("Recording: -")
        self.session_label.setWordWrap(True)
        self.session_label.setStyleSheet("font-weight: 600;")
        session_layout.addWidget(self.session_label)
        self.measurement_label = QLabel("")
        self.measurement_label.setWordWrap(True)
        self.measurement_label.setStyleSheet("color: #475569; font-size: 11px;")
        session_layout.addWidget(self.measurement_label)
        measurement_button = QPushButton("Edit measurement details...")
        measurement_button.setToolTip(
            "Describe what you measured (noise / CMRR / crosstalk / gain, the connection and the "
            "applied voltage). Used for the report, the file names and the CSV export."
        )
        measurement_button.clicked.connect(self.edit_measurement)
        session_layout.addWidget(measurement_button)
        layout.addWidget(session_group)

        navigation_group = QGroupBox("Navigation")
        navigation_group.setMaximumHeight(150)
        navigation_layout = QVBoxLayout(navigation_group)
        self.selection_mode_button = QPushButton("Selection Mode")
        self.selection_mode_button.setCheckable(True)
        self.selection_mode_button.toggled.connect(self._toggle_selection_mode)
        navigation_layout.addWidget(self.selection_mode_button)
        fit_button = QPushButton("Fit Visible Window")
        fit_button.clicked.connect(self._fit_visible_window)
        navigation_layout.addWidget(fit_button)
        select_visible_button = QPushButton("Select Visible Window")
        select_visible_button.clicked.connect(self._select_visible_window)
        navigation_layout.addWidget(select_visible_button)
        self.selection_summary = QLabel("Selection: none")
        self.selection_summary.setWordWrap(True)
        navigation_layout.addWidget(self.selection_summary)
        layout.addWidget(navigation_group)

        channels_group = QGroupBox("Channels")
        channels_layout = QVBoxLayout(channels_group)
        self.channel_container = QWidget()
        self.channel_layout = QVBoxLayout(self.channel_container)
        self.channel_layout.addStretch(1)
        channel_scroll = QScrollArea()
        channel_scroll.setWidgetResizable(True)
        channel_scroll.setWidget(self.channel_container)
        channel_scroll.setMinimumHeight(180)
        header_row = QHBoxLayout()
        header_row.addWidget(QLabel("Time"))
        header_row.addSpacing(22)
        header_row.addWidget(QLabel("FFT"))
        header_row.addStretch(1)
        channels_layout.addLayout(header_row)
        channels_layout.addWidget(channel_scroll)
        layout.addWidget(channels_group)

        fft_group = QGroupBox("FFT")
        fft_layout = QFormLayout(fft_group)
        self.fft_length_spin = QSpinBox()
        self.fft_length_spin.setRange(128, 1_048_576)
        self.fft_length_spin.setSingleStep(256)
        self.fft_length_spin.setValue(4096)
        self.fft_length_spin.valueChanged.connect(self._refresh_analysis)
        fft_layout.addRow("FFT length", self.fft_length_spin)
        self.window_combo = QComboBox()
        self.window_combo.addItems(list(WINDOW_MAP.keys()))
        self.window_combo.setCurrentText("Hann")
        self.window_combo.currentIndexChanged.connect(self._refresh_analysis)
        fft_layout.addRow("Window", self.window_combo)
        layout.addWidget(fft_group)

        filter_group = QGroupBox("Filters")
        filter_layout = QFormLayout(filter_group)
        self.notch_50_check = QCheckBox("Enable 50 Hz notch")
        self.notch_50_check.toggled.connect(self._on_filter_changed)
        filter_layout.addRow(self.notch_50_check)
        self.notch_100_check = QCheckBox("Enable 100 Hz notch")
        self.notch_100_check.toggled.connect(self._on_filter_changed)
        filter_layout.addRow(self.notch_100_check)
        self.filter_q_spin = QDoubleSpinBox()
        self.filter_q_spin.setRange(2.0, 100.0)
        self.filter_q_spin.setValue(30.0)
        self.filter_q_spin.setDecimals(1)
        self.filter_q_spin.valueChanged.connect(self._on_filter_changed)
        filter_layout.addRow("Notch Q", self.filter_q_spin)
        layout.addWidget(filter_group)

        metrics_group = QGroupBox("Derived Metrics")
        metrics_layout = QFormLayout(metrics_group)
        self.common_mode_spin = QDoubleSpinBox()
        self.common_mode_spin.setRange(0.0, 1_000_000_000.0)
        self.common_mode_spin.setDecimals(9)
        self.common_mode_spin.setValue(0.0)
        self.common_mode_spin.setSuffix(" Vrms")
        self.common_mode_spin.valueChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("Common-mode input", self.common_mode_spin)
        # The applied signal makes SNR/THD/ENOB meaningful: without it the app can
        # only describe the measured spectrum, not refer it to what you fed in.
        self.input_amplitude_spin = QDoubleSpinBox()
        self.input_amplitude_spin.setRange(0.0, 1_000_000_000.0)
        self.input_amplitude_spin.setDecimals(9)
        self.input_amplitude_spin.setValue(0.0)
        self.input_amplitude_spin.setSuffix(" Vrms")
        self.input_amplitude_spin.setToolTip(
            "RMS of the signal you actually applied to the inputs. Used to report gain error, "
            "input-referred SNR and a full-scale-referenced ENOB."
        )
        self.input_amplitude_spin.valueChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("Applied signal", self.input_amplitude_spin)
        self.cmrr_frequency_mode_combo = QComboBox()
        self.cmrr_frequency_mode_combo.addItems(["Use fundamental", "Manual Hz"])
        self.cmrr_frequency_mode_combo.currentIndexChanged.connect(self._on_cmrr_mode_changed)
        metrics_layout.addRow("CMRR frequency", self.cmrr_frequency_mode_combo)
        self.cmrr_frequency_spin = QDoubleSpinBox()
        self.cmrr_frequency_spin.setRange(0.01, 100000.0)
        self.cmrr_frequency_spin.setDecimals(4)
        self.cmrr_frequency_spin.setValue(50.0)
        self.cmrr_frequency_spin.setSuffix(" Hz")
        self.cmrr_frequency_spin.valueChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("CMRR manual Hz", self.cmrr_frequency_spin)
        self.cmrr_correction_check = QCheckBox("Apply ADS1299 sinc3 correction")
        self.cmrr_correction_check.toggled.connect(self._on_cmrr_correction_changed)
        metrics_layout.addRow(self.cmrr_correction_check)
        self.cmrr_correction_fs_spin = QDoubleSpinBox()
        self.cmrr_correction_fs_spin.setRange(1.0, 100000.0)
        self.cmrr_correction_fs_spin.setDecimals(3)
        self.cmrr_correction_fs_spin.setValue(250.0)
        self.cmrr_correction_fs_spin.setSuffix(" Hz")
        self.cmrr_correction_fs_spin.valueChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("ADS1299 sinc fs", self.cmrr_correction_fs_spin)
        self.crosstalk_source_combo = QComboBox()
        self.crosstalk_source_combo.currentIndexChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("Crosstalk source", self.crosstalk_source_combo)
        self.crosstalk_frequency_mode_combo = QComboBox()
        self.crosstalk_frequency_mode_combo.addItems(["Use fundamental", "Manual Hz"])
        self.crosstalk_frequency_mode_combo.currentIndexChanged.connect(self._on_crosstalk_mode_changed)
        metrics_layout.addRow("Crosstalk freq", self.crosstalk_frequency_mode_combo)
        self.crosstalk_frequency_spin = QDoubleSpinBox()
        self.crosstalk_frequency_spin.setRange(0.01, 100000.0)
        self.crosstalk_frequency_spin.setDecimals(4)
        self.crosstalk_frequency_spin.setValue(10.0)
        self.crosstalk_frequency_spin.setSuffix(" Hz")
        self.crosstalk_frequency_spin.valueChanged.connect(self._refresh_analysis)
        metrics_layout.addRow("Crosstalk manual Hz", self.crosstalk_frequency_spin)
        self._on_cmrr_mode_changed()
        self._on_cmrr_correction_changed()
        self._on_crosstalk_mode_changed()
        layout.addWidget(metrics_group)

        layout.addStretch(1)
        return panel

    def load_csv(self) -> None:
        file_name, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV files (*.csv);;All files (*)")
        if not file_name:
            return
        self.open_csv_path(file_name)

    def _apply_header_stimulus(self, metadata: dict[str, str]) -> None:
        """Carry the applied voltage from the recording name into the metrics.

        The stimulus is typed once, in the recorder's naming dialog. Reading it
        back here is what lets gain error, input-referred SNR and ENOB at full
        scale fill in without retyping it.
        """

        spin = getattr(self, "input_amplitude_spin", None)
        if spin is None:
            return
        applied = float(self.measurement.stimulus_v or 0.0)
        if applied <= 0.0:
            for key in ("Applied Stimulus (V)", "Applied Stimulus"):
                parsed = parse_voltage(str(metadata.get(key, "") or ""))
                if parsed is not None and parsed > 0.0:
                    applied = parsed
                    break
        spin.blockSignals(True)
        spin.setValue(max(applied, 0.0))
        spin.blockSignals(False)
        self.measurement.stimulus_v = max(applied, 0.0)

    def open_csv_path(self, file_name: str | Path, selection: Optional[tuple[float, float]] = None) -> None:
        """Load a CSV by path, using the sample rate stored in the file header.

        ``selection`` optionally restricts the analysis region to the given time
        range (used to analyze a named section of a recording).
        """

        file_path = Path(file_name)
        try:
            preview = sniff_csv_layout(file_path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "CSV Import", f"Could not inspect the CSV file.\n\n{exc}")
            return

        try:
            metadata = read_openbci_metadata(file_path)
        except OSError:
            metadata = {}
        default_sample_rate = openbci_sample_rate(metadata)
        self.source_metadata = dict(metadata)
        self.recording_name = str(metadata.get("Session", "") or "").strip()
        self.measurement = MeasurementInfo.from_metadata(metadata)
        self._apply_header_stimulus(metadata)
        self._refresh_attenuator_label()
        if self.recording_name:
            self.export_options.last_stem = full_export_stem(self.recording_name, self.measurement)
        if str(file_path.parent) and not self.export_options.output_dir:
            self.export_options.output_dir = str(file_path.parent)

        dialog = ImportOptionsDialog(file_path, preview, self, default_sample_rate=default_sample_rate)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            dataset = load_openbci_csv(
                file_path=file_path,
                sample_rate=dialog.sample_rate(),
                channel_columns=dialog.selected_channels(),
                timestamp_column=dialog.timestamp_column(),
                preview=preview,
            )
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "CSV Import", f"Could not load the CSV data.\n\n{exc}")
            return
        finally:
            QApplication.restoreOverrideCursor()

        self.original_dataset = dataset
        self.dataset = dataset
        self.excluded_ranges = []
        self.edit_history = []
        self.metrics_df = pd.DataFrame()
        self.crosstalk_df = pd.DataFrame()
        self.crosstalk_sweep_df = pd.DataFrame()
        self.gain_sweep_df = pd.DataFrame()
        self.cmrr_sweep_df = pd.DataFrame()
        self.spectra = []
        self._last_selection_matrix = None
        self._last_enabled_names = []
        self._invalidate_filter_cache()
        self.file_label.setText(file_path.name)
        self.file_label.setToolTip(str(file_path))
        self._update_session_label()
        self._rebuild_channel_controls()
        # After the channel list exists: the combos are filled from the enabled
        # channels, which only exist once the checkboxes do.
        self._refresh_channel_combos()
        recorded_reference = str(metadata.get("Reference Channel", "") or "").strip()
        if recorded_reference:
            # Preselect the monitor the live window used, so the CMRR tab is ready
            # without hunting for the channel name. The recorder writes "CH 1"
            # while the columns are "EXG Channel 0", so the label is resolved
            # rather than matched literally.
            resolved = resolve_channel_name(recorded_reference, self._enabled_channel_names())
            for combo in (self.cmrr_monitor_combo, self.crosstalk_reference_combo):
                if not resolved:
                    continue
                position = combo.findText(resolved)
                if position >= 0:
                    combo.blockSignals(True)
                    combo.setCurrentIndex(position)
                    combo.blockSignals(False)
        self._rebuild_waveform_items()
        self._initialize_selection()

        section_text = ""
        if selection is not None and dataset.sample_count > 1:
            start_s, end_s = sorted((float(selection[0]), float(selection[1])))
            start_s = max(start_s, float(dataset.time_s[0]))
            end_s = min(end_s, float(dataset.time_s[-1]))
            if end_s > start_s:
                self._set_selection_region(start_s, end_s, refresh=False)
                self.waveform_plot.setXRange(start_s, end_s, padding=0.02)
                section_text = f" | section {start_s:.2f}-{end_s:.2f} s"

        self._refresh_analysis()
        self.statusBar().showMessage(
            f"Loaded {dataset.sample_count:,} samples across {dataset.channel_count} channel(s) "
            f"at {dataset.sample_rate:.3f} Hz.{section_text}"
        )

    def _rebuild_channel_controls(self) -> None:
        self.channel_checks.clear()
        self.channel_fft_checks.clear()
        while self.channel_layout.count() > 0:
            item = self.channel_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        if not self.dataset:
            self.channel_layout.addStretch(1)
            return

        for channel_name in self.dataset.channel_names:
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(6)

            time_checkbox = QCheckBox()
            time_checkbox.setChecked(True)
            time_checkbox.stateChanged.connect(self._on_channel_visibility_changed)
            row_layout.addWidget(time_checkbox)

            fft_checkbox = QCheckBox()
            fft_checkbox.setChecked(True)
            fft_checkbox.stateChanged.connect(self._on_channel_visibility_changed)
            row_layout.addWidget(fft_checkbox)

            label = QLabel(channel_name)
            row_layout.addWidget(label)
            row_layout.addStretch(1)

            self.channel_layout.addWidget(row_widget)
            self.channel_checks[channel_name] = time_checkbox
            self.channel_fft_checks[channel_name] = fft_checkbox
        self.channel_layout.addStretch(1)
        self._refresh_channel_combos()

    def _rebuild_waveform_items(self) -> None:
        for curve in self.channel_curves.values():
            self.waveform_plot.removeItem(curve)
        self.channel_curves.clear()
        for curve in self.fft_curves.values():
            self.fft_plot.removeItem(curve)
        self.fft_curves.clear()

        if not self.dataset:
            return

        for index, channel_name in enumerate(self.dataset.channel_names):
            curve = self.waveform_plot.plot(name=channel_name)
            pen = pg.mkPen(pg.intColor(index, hues=max(8, self.dataset.channel_count), values=1.0), width=1.0)
            curve.setPen(pen)
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            self.channel_curves[index] = curve

            fft_curve = self.fft_plot.plot(name=channel_name)
            fft_curve.setPen(pg.mkPen(pen.color(), width=1.4))
            self.fft_curves[index] = fft_curve
        self._redraw_waveform()

    def _initialize_selection(self) -> None:
        if not self.dataset or self.dataset.sample_count == 0:
            return

        selection_end = float(self.dataset.time_s[-1]) if self.dataset.sample_count > 1 else 0.0
        self._set_selection_region(0.0, selection_end, refresh=False)
        self.selection_region.show()
        visible_end = min(selection_end, 10.0) if selection_end > 0 else 1.0
        self.waveform_plot.setXRange(0.0, visible_end, padding=0.01)

    def _set_selection_region(self, start_s: float, end_s: float, refresh: bool) -> None:
        self._setting_region = True
        try:
            self.selection_region.setRegion((start_s, end_s))
        finally:
            self._setting_region = False
        if refresh:
            self._refresh_analysis()

    def _preview_selection(self, start_s: float, end_s: float) -> None:
        self._set_selection_region(start_s, end_s, refresh=False)

    def _commit_selection(self, start_s: float, end_s: float) -> None:
        self._set_selection_region(start_s, end_s, refresh=True)

    def _on_selection_region_changed(self) -> None:
        if self._setting_region:
            return
        self._refresh_analysis()

    def _toggle_selection_mode(self, enabled: bool) -> None:
        self.waveform_viewbox.set_selection_enabled(enabled)
        if enabled:
            self.statusBar().showMessage("Selection mode enabled. Drag on the waveform to define the analysis region.")
        else:
            self.statusBar().showMessage("Pan mode enabled. Drag to pan and use the mouse wheel to zoom.")

    def _fit_visible_window(self) -> None:
        if not self.dataset:
            return
        self.waveform_plot.autoRange()

    def _select_visible_window(self) -> None:
        if not self.dataset:
            return
        x_range, _ = self.waveform_plot.viewRange()
        start_s = max(float(x_range[0]), 0.0)
        end_s = min(float(x_range[1]), float(self.dataset.time_s[-1]) if self.dataset.sample_count else 0.0)
        self._set_selection_region(start_s, end_s, refresh=True)

    def _on_channel_visibility_changed(self) -> None:
        self._refresh_channel_combos()
        self._redraw_waveform()
        self._refresh_analysis()

    def _refresh_channel_combos(self) -> None:
        enabled_names = self._enabled_channel_names()
        current_source = self.crosstalk_source_combo.currentText()
        current_gain_channel = self.gain_sweep_channel_combo.currentText()
        current_monitor = self.cmrr_monitor_combo.currentText()
        current_reference = self.crosstalk_reference_combo.currentText()

        self.crosstalk_source_combo.blockSignals(True)
        self.gain_sweep_channel_combo.blockSignals(True)
        self.cmrr_monitor_combo.blockSignals(True)
        self.crosstalk_reference_combo.blockSignals(True)
        self.crosstalk_source_combo.clear()
        self.gain_sweep_channel_combo.clear()
        self.cmrr_monitor_combo.clear()
        self.crosstalk_reference_combo.clear()
        self.cmrr_monitor_combo.addItem(NO_MONITOR_CHANNEL)
        self.crosstalk_reference_combo.addItem(NO_MONITOR_CHANNEL)
        self.crosstalk_source_combo.addItems(enabled_names)
        self.gain_sweep_channel_combo.addItems(enabled_names)
        self.cmrr_monitor_combo.addItems(enabled_names)
        self.crosstalk_reference_combo.addItems(enabled_names)

        if current_source in enabled_names:
            self.crosstalk_source_combo.setCurrentText(current_source)
        if current_gain_channel in enabled_names:
            self.gain_sweep_channel_combo.setCurrentText(current_gain_channel)
        if current_monitor:
            self.cmrr_monitor_combo.setCurrentText(current_monitor)
        # The recorded reference is what the tab should default to; only override
        # it when the operator has chosen something that no longer exists. "(none)"
        # is a real choice and must survive a channel toggle, not be replaced by
        # the recorded channel again.
        if current_reference in enabled_names or current_reference == NO_MONITOR_CHANNEL:
            if current_reference:
                self.crosstalk_reference_combo.setCurrentText(current_reference)
        else:
            recorded = self.reference_channel_name(resolve=False)
            resolved = resolve_channel_name(recorded, enabled_names)
            if resolved:
                self.crosstalk_reference_combo.setCurrentText(resolved)
        self.crosstalk_source_combo.blockSignals(False)
        self.gain_sweep_channel_combo.blockSignals(False)
        self.cmrr_monitor_combo.blockSignals(False)
        self.crosstalk_reference_combo.blockSignals(False)

    def _enabled_channel_names(self) -> list[str]:
        if not self.dataset:
            return []
        enabled_names: list[str] = []
        for name in self.dataset.channel_names:
            checkbox = self.channel_checks.get(name)
            if checkbox is not None and checkbox.isChecked():
                enabled_names.append(name)
        return enabled_names

    def _enabled_channel_indices(self) -> list[int]:
        if not self.dataset:
            return []
        enabled_names = set(self._enabled_channel_names())
        return [index for index, name in enumerate(self.dataset.channel_names) if name in enabled_names]

    def _fft_visible_channel_indices(self) -> list[int]:
        if not self.dataset:
            return []
        fft_visible_names = {
            name
            for name in self.dataset.channel_names
            if self.channel_fft_checks.get(name) is not None and self.channel_fft_checks[name].isChecked()
        }
        enabled_names = set(self._enabled_channel_names())
        return [index for index, name in enumerate(self.dataset.channel_names) if name in enabled_names and name in fft_visible_names]

    def _default_fft_analysis_index(self, enabled_indices: list[int]) -> int:
        visible_fft_indices = set(self._fft_visible_channel_indices())
        for local_index, channel_index in enumerate(enabled_indices):
            if channel_index in visible_fft_indices:
                return local_index
        return 0

    def _current_filter_settings(self) -> FilterSettings:
        return FilterSettings(
            notch_50_hz=self.notch_50_check.isChecked(),
            notch_100_hz=self.notch_100_check.isChecked(),
            quality_factor=float(self.filter_q_spin.value()),
        )

    def _current_filter_key(self) -> tuple[bool, bool, float]:
        settings = self._current_filter_settings()
        return (settings.notch_50_hz, settings.notch_100_hz, settings.quality_factor)

    def _invalidate_filter_cache(self) -> None:
        self._filter_cache_key = None
        self._filter_cache_data = None

    def _get_filtered_data(self):
        if not self.dataset:
            return None

        key = self._current_filter_key()
        if not any(key[:2]):
            return self.dataset.data
        if key == self._filter_cache_key and self._filter_cache_data is not None:
            return self._filter_cache_data

        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            filtered = apply_filters(self.dataset.data, self.dataset.sample_rate, self._current_filter_settings())
        finally:
            QApplication.restoreOverrideCursor()
        self._filter_cache_key = key
        self._filter_cache_data = filtered
        return filtered

    def _on_filter_changed(self) -> None:
        self._invalidate_filter_cache()
        self._redraw_waveform()
        self._refresh_analysis()

    def _on_waveform_display_changed(self) -> None:
        self.waveform_plot.showGrid(
            x=self.waveform_grid_check.isChecked(),
            y=self.waveform_grid_check.isChecked(),
            alpha=0.18,
        )
        self._redraw_waveform()

    def _on_fft_display_changed(self) -> None:
        self.fft_plot.showGrid(
            x=self.fft_grid_check.isChecked(),
            y=self.fft_grid_check.isChecked(),
            alpha=0.28,
        )
        self._refresh_analysis()

    def _on_fundamental_mode_changed(self) -> None:
        manual_mode = self.fundamental_mode_combo.currentText() == "Manual Hz"
        self.manual_fundamental_spin.setEnabled(manual_mode)
        self._refresh_analysis()

    def _on_cmrr_mode_changed(self) -> None:
        manual_mode = self.cmrr_frequency_mode_combo.currentText() == "Manual Hz"
        self.cmrr_frequency_spin.setEnabled(manual_mode)
        self._refresh_analysis()

    def _on_cmrr_correction_changed(self) -> None:
        correction_enabled = self.cmrr_correction_check.isChecked()
        self.cmrr_correction_fs_spin.setEnabled(correction_enabled)
        self._refresh_analysis()

    def _on_crosstalk_mode_changed(self) -> None:
        manual_mode = self.crosstalk_frequency_mode_combo.currentText() == "Manual Hz"
        self.crosstalk_frequency_spin.setEnabled(manual_mode)
        self._refresh_analysis()

    def _manual_fundamental_hz(self) -> Optional[float]:
        if self.fundamental_mode_combo.currentText() != "Manual Hz":
            return None
        return float(self.manual_fundamental_spin.value())

    def _manual_cmrr_frequency_hz(self) -> Optional[float]:
        if self.cmrr_frequency_mode_combo.currentText() != "Manual Hz":
            return None
        return float(self.cmrr_frequency_spin.value())

    def _manual_crosstalk_frequency_hz(self) -> Optional[float]:
        if self.crosstalk_frequency_mode_combo.currentText() != "Manual Hz":
            return None
        return float(self.crosstalk_frequency_spin.value())

    def reference_channel_name(self, resolve: bool = True) -> str:
        """Channel the live window recorded as the applied-stimulus monitor.

        The recorder writes its own label (``CH 1``) while the CSV columns are
        ``EXG Channel 0``; ``resolve_channel_name()`` bridges the two so the
        reference really is matched instead of silently ignored.
        """

        combo = getattr(self, "cmrr_monitor_combo", None)
        manual = str(combo.currentText()).strip() if combo is not None else ""
        recorded = str(self.source_metadata.get("Reference Channel", "") or "").strip()
        chosen = manual if manual and manual != NO_MONITOR_CHANNEL else recorded
        if not chosen or not resolve:
            return chosen
        names = self.dataset.channel_names if self.dataset else []
        return resolve_channel_name(chosen, names) or chosen

    def crosstalk_reference_name(self) -> str:
        """Reference channel chosen on the Crosstalk Sweep tab itself.

        It is chosen there rather than taken from the recording so a single file
        can be re-examined with a different reference without editing anything
        else, and it defaults to the recorded reference.
        """

        combo = getattr(self, "crosstalk_reference_combo", None)
        if combo is None:
            return ""
        text = str(combo.currentText()).strip()
        return "" if text == NO_MONITOR_CHANNEL else text

    def _redraw_waveform(self) -> None:
        if not self.dataset:
            return

        display_data = self._get_filtered_data()
        if display_data is None:
            return

        enabled_indices = self._enabled_channel_indices()
        ticks = []
        if not enabled_indices:
            for curve in self.channel_curves.values():
                curve.hide()
            self.waveform_plot.getAxis("left").setTicks([[]])
            return

        # The reference channel is volts while the rest are microvolts, so it would
        # flatten every other trace. It gets its own figure (Reference channel in
        # the export), and is left out of the stacked view - unless it is the only
        # channel enabled, in which case showing nothing would be worse.
        reference_name = self.reference_channel_name()
        reference_index = (
            self.dataset.channel_names.index(reference_name)
            if reference_name and reference_name in self.dataset.channel_names
            else None
        )
        if reference_index is not None and len(enabled_indices) > 1:
            enabled_indices = [index for index in enabled_indices if index != reference_index]
        for index, curve in self.channel_curves.items():
            if index not in enabled_indices:
                curve.hide()
        if not enabled_indices:
            self.waveform_plot.getAxis("left").setTicks([[]])
            return

        sample_step = max(1, display_data.shape[0] // 5000)
        preview = display_data[::sample_step, enabled_indices]
        scale_uv = self._waveform_scale_uv_per_div()
        spacing = max(scale_uv * 4.0, 1.0)
        offset_map: dict[int, float] = {}
        major_ticks: list[tuple[float, str]] = []
        minor_ticks: list[tuple[float, str]] = []

        for order, channel_index in enumerate(enabled_indices):
            offset = float((len(enabled_indices) - 1 - order) * spacing)
            offset_map[channel_index] = offset
            ticks.append((offset, self.dataset.channel_names[channel_index]))
            major_ticks.append((offset, self.dataset.channel_names[channel_index]))
            for multiplier in (-2, -1, 1, 2):
                minor_ticks.append((offset + multiplier * scale_uv, f"{multiplier * scale_uv:.0f}"))

        for channel_index, curve in self.channel_curves.items():
            if channel_index not in enabled_indices:
                curve.hide()
                continue
            offset = offset_map[channel_index]
            curve.setData(self.dataset.time_s, display_data[:, channel_index] + offset)
            curve.show()

        if self.waveform_labels_check.isChecked():
            self.waveform_plot.getAxis("left").setTicks([major_ticks, minor_ticks])
        else:
            self.waveform_plot.getAxis("left").setTicks([[]])
        self.waveform_plot.setLabel("left", f"Channels, {scale_uv:.0f} uV/div")
        self.waveform_plot.setYRange(-spacing, max(offset_map.values()) + spacing, padding=0.02)
        self._refresh_exclusion_overlays()

    def _refresh_analysis(self) -> None:
        if not self.dataset:
            return

        enabled_indices = self._enabled_channel_indices()
        if not enabled_indices:
            self.metrics_table.clear()
            self.crosstalk_table.clear()
            self._mark_sweep_results_stale()
            for curve in self.fft_curves.values():
                curve.hide()
            self.fft_marker.hide()
            self.selection_summary.setText("Selection: no enabled channels")
            self._last_selection_matrix = None
            self._last_enabled_names = []
            self._last_mean_names = []
            return

        filtered_data = self._get_filtered_data()
        if filtered_data is None:
            return

        start_s, end_s = self.selection_region.getRegion()
        selection_time, selection_matrix = extract_time_region(
            self.dataset.time_s,
            filtered_data[:, enabled_indices],
            start_s,
            end_s,
            self.excluded_ranges,
        )
        if selection_matrix.shape[0] < 8:
            self.selection_summary.setText("Selection: choose a longer region after exclusions")
            self.metrics_table.clear()
            self.crosstalk_table.clear()
            self._mark_sweep_results_stale()
            for curve in self.fft_curves.values():
                curve.hide()
            self.fft_marker.hide()
            self._last_selection_matrix = None
            self._last_enabled_names = []
            self._last_mean_names = []
            return

        enabled_names = [self.dataset.channel_names[index] for index in enabled_indices]
        # The averaged row must only contain channels that are actually shown:
        # a channel switched off in either the time or the FFT list is not part
        # of the mean, so a hidden noisy channel cannot skew it.
        fft_visible = {
            self.dataset.channel_names[index] for index in self._fft_visible_channel_indices()
        }
        mean_names = [name for name in enabled_names if name in fft_visible] or list(enabled_names)
        self._last_selection_matrix = selection_matrix.copy()
        self._last_enabled_names = list(enabled_names)
        self._last_mean_names = list(mean_names)
        fft_length = min(int(self.fft_length_spin.value()), max(selection_matrix.shape[0], 128))
        applied_rms = float(self.input_amplitude_spin.value())
        channel_gains = self._header_channel_gains(enabled_names)
        fft_settings = FFTSettings(
            fft_length=fft_length,
            window_name=self.window_combo.currentText(),
            manual_fundamental_hz=self._manual_fundamental_hz(),
            cmrr_frequency_hz=self._manual_cmrr_frequency_hz(),
            crosstalk_frequency_hz=self._manual_crosstalk_frequency_hz(),
            cmrr_correction_enabled=self.cmrr_correction_check.isChecked(),
            cmrr_correction_fs_hz=float(self.cmrr_correction_fs_spin.value()),
            applied_input_rms_v=applied_rms,
            full_scale_rms_v=VREF_VOLTS / max(channel_gains[0], 1.0) / np.sqrt(2.0),
        )
        common_mode_voltage = float(self.common_mode_spin.value()) if self.common_mode_spin.value() > 0 else None
        source_name = self.crosstalk_source_combo.currentText()
        crosstalk_source_index = enabled_names.index(source_name) if source_name in enabled_names else 0

        try:
            metrics_df, crosstalk_df, spectra = analyze_multichannel(
                selection_matrix,
                enabled_names,
                sample_rate=self.dataset.sample_rate,
                fft_settings=fft_settings,
                common_mode_voltage=common_mode_voltage,
                crosstalk_source_index=crosstalk_source_index,
                signal_scale_to_volts=1e-6,
                mean_channel_names=mean_names,
                channel_gains=channel_gains,
            )
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "Analysis", f"Analysis failed for the current selection.\n\n{exc}")
            return

        self.metrics_df = metrics_df
        self.crosstalk_df = crosstalk_df
        self.spectra = spectra
        self._populate_table(self.metrics_table, metrics_df)
        self._populate_table(self.crosstalk_table, crosstalk_df)
        self._mark_sweep_results_stale()

        analysis_index = self._default_fft_analysis_index(enabled_indices)
        self._plot_fft(spectra, enabled_names, analysis_index, enabled_indices)

        raw_start = int(pd.Index(self.dataset.time_s).searchsorted(min(start_s, end_s), side="left"))
        raw_end = int(pd.Index(self.dataset.time_s).searchsorted(max(start_s, end_s), side="right"))
        raw_sample_count = max(raw_end - raw_start, 0)
        excluded_count = max(raw_sample_count - selection_time.size, 0)
        duration = selection_time[-1] - selection_time[0] if selection_time.size > 1 else 0.0
        self.selection_summary.setText(
            f"Selection: {duration:.3f} s, {selection_time.size:,} samples, {excluded_count:,} excluded sample(s)"
        )

    def _header_channel_gains(self, channel_names: Sequence[str]) -> list[float]:
        """PGA gain per channel, read from the recording header.

        The header stores ``Board Channel Gains = x24,x24,...`` for all real
        channels, so the full-scale reference for ENOB can be computed per
        channel instead of assuming x24.
        """

        raw = self.source_metadata.get("Board Channel Gains", "")
        parsed = [float(token.strip().lstrip("xX") or 24.0) for token in raw.split(",") if token.strip()]
        board_order = self._board_channel_order(channel_names)
        gains: list[float] = []
        for position, _name in enumerate(channel_names):
            index = board_order[position] if position < len(board_order) else position
            gains.append(parsed[index] if 0 <= index < len(parsed) else 24.0)
        return gains or [24.0] * len(channel_names)

    def _board_channel_order(self, channel_names: Sequence[str]) -> list[int]:
        """Map the analysed channels onto the ``Board Channel Gains`` order."""

        mapping = self.source_metadata.get("Channel Map", "")
        if not mapping:
            return list(range(len(channel_names)))
        numbers = [int(token.strip()) for token in mapping.split(",") if token.strip().isdigit()]
        order: list[int] = []
        for name in channel_names:
            digits = "".join(character for character in str(name) if character.isdigit())
            number = int(digits) if digits else 0
            order.append(numbers.index(number) if number in numbers else len(order))
        return order

    def _plot_fft(
        self,
        spectra: list[SpectrumResult],
        enabled_names: list[str],
        analysis_index: int,
        enabled_indices: list[int],
    ) -> None:
        axis = self.fft_plot.getAxis("left")
        amplitude_mode = self.fft_amplitude_mode_combo.currentText()
        visible_fft_indices = self._fft_visible_channel_indices()
        visible_amplitudes: list[np.ndarray] = []

        for local_index, channel_name in enumerate(enabled_names):
            channel_index = enabled_indices[local_index]
            curve = self.fft_curves.get(channel_index)
            if curve is None:
                continue
            spectrum = spectra[local_index]
            mask = spectrum.frequencies_hz > 0.0
            frequencies_hz = spectrum.frequencies_hz[mask]
            amplitude_uvrms = spectrum.amplitude_rms[mask]

            if channel_index not in visible_fft_indices:
                curve.hide()
                continue

            visible_amplitudes.append(amplitude_uvrms)
            if amplitude_mode == "Linear auto":
                curve.setData(frequencies_hz, amplitude_uvrms)
            else:
                max_scale = 100.0 if amplitude_mode == "Log 100 uVrms" else 1000.0
                min_scale = 0.1
                clipped = amplitude_uvrms.clip(min_scale, max_scale)
                curve.setData(frequencies_hz, np.log10(clipped))
            curve.show()

        if amplitude_mode == "Linear auto":
            combined = np.concatenate(visible_amplitudes) if visible_amplitudes else np.array([1.0])
            y_max = max(float(combined.max()) * 1.1 if combined.size else 1.0, 1.0)
            self.fft_plot.setYRange(0.0, y_max, padding=0.0)
            axis.setTicks([self._make_linear_ticks(y_max)])
        else:
            max_scale = 100.0 if amplitude_mode == "Log 100 uVrms" else 1000.0
            min_scale = 0.1
            self.fft_plot.setYRange(np.log10(min_scale), np.log10(max_scale), padding=0.0)
            axis.setTicks(self._make_log_ticks(max_scale))

        spectrum = spectra[analysis_index]

        if spectrum.fundamental_frequency_hz is not None:
            self.fft_marker.setValue(spectrum.fundamental_frequency_hz)
            self.fft_marker.show()
        else:
            self.fft_marker.hide()

        nyquist = self.dataset.sample_rate / 2.0 if self.dataset else 1.0
        self.fft_plot.setXRange(0.0, nyquist, padding=0.02)
        self.fft_plot.setTitle(f"FFT: {enabled_names[analysis_index]} ({spectrum.fundamental_source} fundamental)")

    def _waveform_scale_uv_per_div(self) -> float:
        text = self.waveform_scale_combo.currentText().split()[0]
        return float(text)

    @staticmethod
    def _make_linear_ticks(y_max: float) -> list[tuple[float, str]]:
        divisions = 5
        step = y_max / divisions if y_max > 0 else 1.0
        return [(step * index, f"{step * index:.3f}") for index in range(divisions + 1)]

    @staticmethod
    def _make_log_ticks(max_scale: float) -> list[list[tuple[float, str]]]:
        major_levels: list[tuple[float, str]] = []
        minor_levels: list[tuple[float, str]] = []
        decade = 0.1
        while decade <= max_scale + 1e-12:
            major_levels.append((float(np.log10(decade)), f"{decade:g}"))
            for multiplier in range(2, 10):
                value = decade * multiplier
                if value > max_scale + 1e-12:
                    break
                minor_levels.append((float(np.log10(value)), ""))
            decade *= 10.0
        return [major_levels, minor_levels]

    def _populate_table(self, table: QTableWidget, dataframe: pd.DataFrame) -> None:
        table.clear()
        table.setRowCount(len(dataframe.index))
        table.setColumnCount(len(dataframe.columns))
        table.setHorizontalHeaderLabels([str(column) for column in dataframe.columns])

        for row_index, (_, row) in enumerate(dataframe.iterrows()):
            for column_index, value in enumerate(row):
                table.setItem(row_index, column_index, QTableWidgetItem(self._format_table_value(value)))
        table.resizeColumnsToContents()

    def _mark_sweep_results_stale(self) -> None:
        self.crosstalk_sweep_df = pd.DataFrame()
        self._clear_crosstalk_sweep_plot()
        self.crosstalk_sweep_info_label.setText("Sweep stale")

        self._mark_gain_sweep_stale()

        self.cmrr_sweep_df = pd.DataFrame()
        self._clear_cmrr_sweep_plot()
        self.cmrr_sweep_info_label.setText("Sweep stale")

    def _mark_gain_sweep_stale(self, *args) -> None:
        self.gain_sweep_df = pd.DataFrame()
        self._clear_gain_sweep_plot()
        self.gain_sweep_info_label.setText("Sweep stale")

    def compute_crosstalk_sweep_view(self) -> None:
        if self._last_selection_matrix is None or not self._last_enabled_names:
            QMessageBox.information(self, "Crosstalk Sweep", "Select a valid time region first.")
            return

        source_name = self.crosstalk_source_combo.currentText()
        if source_name not in self._last_enabled_names:
            QMessageBox.information(self, "Crosstalk Sweep", "Select a valid crosstalk source first.")
            return

        source_index = self._last_enabled_names.index(source_name)
        source_values = self._last_selection_matrix[:, source_index]
        steps = parse_sweep_steps(self.source_metadata)
        # The reference measures the stimulus, so a crosstalk figure for it would
        # be the drive itself. It is skipped here rather than hidden afterwards, so
        # it contributes nothing to the fit or the 50 Hz summary either.
        reference_name = self.crosstalk_reference_name()
        skipped_reference = bool(reference_name) and reference_name in self._last_enabled_names
        self._clear_crosstalk_sweep_plot()
        combined_frames: list[pd.DataFrame] = []
        accepted_windows = 0
        rejected_windows = 0
        plotted_channels = 0

        for target_index, target_name in enumerate(self._last_enabled_names):
            if skipped_reference and target_name == reference_name:
                continue
            if steps:
                result = compute_planned_crosstalk_sweep(
                    source_values=source_values,
                    target_values=self._last_selection_matrix[:, target_index],
                    sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                    steps=steps,
                )
            else:
                result = compute_crosstalk_sweep(
                    source_values=source_values,
                    target_values=self._last_selection_matrix[:, target_index],
                    sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                    fft_length=int(self.fft_length_spin.value()),
                    window_name=self.window_combo.currentText(),
                    window_duration_s=float(self.crosstalk_sweep_window_spin.value()),
                    stability_tolerance_hz=float(self.crosstalk_sweep_tolerance_spin.value()),
                    min_hold_s=self._sweep_hold_seconds("crosstalk"),
                    exclude_mains=self._sweep_exclude_mains("crosstalk"),
                    **self._sweep_tracking_options("crosstalk"),
                )
            accepted_windows = result.accepted_windows
            rejected_windows = result.rejected_windows
            if result.dataframe.empty:
                continue

            plotted_channels += 1
            channel_frame = result.dataframe.copy()
            channel_frame.insert(0, "Target", target_name)
            channel_frame.insert(0, "Source", source_name)
            combined_frames.append(channel_frame)

            color = pg.intColor(target_index, hues=max(8, len(self._last_enabled_names)), values=1.0)
            curve = self.crosstalk_sweep_plot.plot(
                channel_frame["Frequency (Hz)"],
                channel_frame["Crosstalk (dB)"],
                pen=pg.mkPen(color, width=SWEEP_TRACE_WIDTH),
                symbol="o",
                symbolSize=4,
                name=target_name,
            )
            self.crosstalk_sweep_curves.append(curve)

        reference_note = f" | ref {reference_name} skipped" if skipped_reference else ""
        if not combined_frames:
            self.crosstalk_sweep_df = pd.DataFrame()
            self.crosstalk_sweep_info_label.setText(
                f"Src {source_name} | 0 ch | {accepted_windows} ok | {rejected_windows} rej{reference_note}"
            )
            return

        self.crosstalk_sweep_df = pd.concat(combined_frames, ignore_index=True)
        self.crosstalk_sweep_info_label.setText(
            self._sweep_info_text(
                "crosstalk",
                f"Src {source_name} | {plotted_channels} ch{reference_note}",
                accepted_windows,
                rejected_windows,
            )
        )
        self._redraw_crosstalk_sweep_fit()

    def _recorded_attenuator_ratio(self) -> float:
        """The input divider the live window wrote into this recording.

        Falls back to this module's default so older recordings, and manual files
        that never saw the app, still produce a number.
        """

        ratio = _metadata_float(self.source_metadata, "Input Attenuator Ratio")
        return ratio if 0.0 < ratio <= 1.0 else DEFAULT_INPUT_ATTENUATOR_RATIO

    def _refresh_attenuator_label(self) -> None:
        label = getattr(self, "gain_attenuator_label", None)
        if label is None:
            return
        ratio = self._recorded_attenuator_ratio()
        series = _metadata_float(self.source_metadata, "Input Attenuator Series (ohm)")
        shunt = _metadata_float(self.source_metadata, "Input Attenuator Shunt (ohm)")
        if series > 0 and shunt > 0:
            label.setText(f"Divider {shunt:g}/({series:g}+{shunt:g}) = {ratio:.4g}")
            label.setToolTip(
                f"Input at the amplifier = applied before the divider * {ratio:.6g}.\n"
                f"Recorded with {series:g} ohm in series and {shunt:g} ohm to ground."
            )
        else:
            label.setText(f"Divider {ratio:.4g} (default)")
            label.setToolTip(
                f"Input at the amplifier = applied before the divider * {ratio:.6g}.\n"
                "This recording carries no divider, so the default is used."
            )

    def compute_gain_sweep_view(self) -> None:
        if self._last_selection_matrix is None or not self._last_enabled_names:
            QMessageBox.information(self, "Gain Sweep", "Select a valid time region first.")
            return

        channel_name = self.gain_sweep_channel_combo.currentText()
        if channel_name not in self._last_enabled_names:
            QMessageBox.information(self, "Gain Sweep", "Select a valid gain sweep channel first.")
            return

        input_before_v = float(self.gain_sweep_input_spin.value())
        channel_index = self._last_enabled_names.index(channel_name)
        steps = parse_sweep_steps(self.source_metadata)
        metadata_vrms = _sweep_amplitude_vrms(self.source_metadata)
        # The recording carries the divider that was actually in front of the
        # amplifier, so a run with a different attenuator is converted correctly
        # instead of with this module's default.
        attenuator = self._recorded_attenuator_ratio()
        planned = bool(steps) and metadata_vrms > 0
        if not planned and input_before_v <= 0:
            # Only the manual path needs this: an automated run reads the applied
            # amplitude out of the recording, so asking for it again would be
            # both redundant and wrong.
            QMessageBox.information(
                self, "Gain Sweep", "Enter a positive input voltage before the attenuator first."
            )
            return

        if planned:
            # The live window drove the generator itself, so the applied amplitude
            # is known exactly and the gain comes straight out of the schedule.
            result = compute_planned_gain_sweep(
                values=self._last_selection_matrix[:, channel_index],
                sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                steps=steps,
                applied_vrms=metadata_vrms,
                signal_scale_to_volts=1e-6,
                attenuator_ratio=attenuator,
            )
        else:
            result = compute_gain_sweep(
                values=self._last_selection_matrix[:, channel_index],
                sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                input_before_attenuator_v=input_before_v,
                fft_length=int(self.fft_length_spin.value()),
                window_name=self.window_combo.currentText(),
                attenuator_ratio=attenuator,
                signal_scale_to_volts=1e-6,
                window_duration_s=float(self.gain_sweep_window_spin.value()),
                stability_tolerance_hz=float(self.gain_sweep_tolerance_spin.value()),
                min_hold_s=self._sweep_hold_seconds("gain"),
                exclude_mains=self._sweep_exclude_mains("gain"),
                **self._sweep_tracking_options("gain"),
            )

        self._clear_gain_sweep_plot()
        if result.dataframe.empty:
            self.gain_sweep_df = pd.DataFrame()
            self.gain_sweep_info_label.setText(
                f"Ch {channel_name} | 0 ok | {result.rejected_windows} rej | div {attenuator:.4g}"
            )
            return

        self.gain_sweep_df = result.dataframe.copy()
        self.gain_sweep_df.insert(0, "Channel", channel_name)

        curve = self.gain_sweep_plot.plot(
            self.gain_sweep_df["Frequency (Hz)"],
            self.gain_sweep_df["Gain (dB)"],
            pen=pg.mkPen(
                pg.intColor(channel_index, hues=max(8, len(self._last_enabled_names)), values=1.0),
                width=SWEEP_TRACE_WIDTH,
            ),
            symbol="o",
            symbolSize=4,
            name=channel_name,
        )
        self.gain_sweep_curves.append(curve)

        if planned:
            input_after_uvrms = metadata_vrms * attenuator / 1e-6
            source_tag = f"generator plan ({len(steps)} steps)"
        else:
            input_after_uvrms = input_before_v * attenuator / 1e-6
            source_tag = "manual"
        self.gain_sweep_info_label.setText(
            self._sweep_info_text(
                "gain",
                f"Ch {channel_name} | in {input_after_uvrms:.6g} uVrms | div {attenuator:.4g} | {source_tag}",
                result.accepted_windows,
                result.rejected_windows,
            )
        )
        self._redraw_gain_sweep_fit()

    def _on_vcm_mode_changed(self, *_args) -> None:
        mode = self.cmrr_vcm_mode()
        if self._bias_dialog is not None:
            self._bias_dialog.set_use_curve(mode == VCM_MONITOR_CURVE)
        self._refresh_bias_curve_controls()
        self._mark_sweep_stale_for("cmrr")

    def cmrr_vcm_mode(self) -> str:
        combo = getattr(self, "cmrr_vcm_mode_combo", None)
        return str(combo.currentData() or VCM_FLAT) if combo is not None else VCM_FLAT

    def _set_cmrr_vcm_mode(self, mode: str) -> None:
        combo = getattr(self, "cmrr_vcm_mode_combo", None)
        if combo is None:
            return
        position = combo.findData(mode)
        if position >= 0:
            combo.blockSignals(True)
            combo.setCurrentIndex(position)
            combo.blockSignals(False)
        self._refresh_bias_curve_controls()

    def _monitor_voltage_table(
        self,
        monitor_values: np.ndarray,
        steps,
        sample_rate: float,
    ) -> pd.DataFrame:
        """The voltage actually measured on the monitor channel, per frequency.

        This is the converter measuring the common-mode voltage itself, so it needs
        no sinc3 correction: the digital filter has already been applied to the number
        that comes out. It is returned as a frequency-keyed table so the analysis can
        look it up at whatever frequency it detected.
        """

        rows: list[dict[str, float]] = []
        values = np.asarray(monitor_values, dtype=np.float64)
        if steps:
            for step in steps:
                start = int(round(step.usable_start_s * sample_rate))
                end = int(round(step.end_s * sample_rate))
                start, end = max(start, 0), min(end, values.size)
                if end - start < 16:
                    continue
                rms_uv = fit_tone_rms(values[start:end], sample_rate, step.frequency_hz)
                if rms_uv > 0:
                    rows.append({"Frequency (Hz)": float(step.frequency_hz), "Reference (Vrms)": rms_uv * 1e-6})
        else:
            window = max(int(round(max(float(self.cmrr_sweep_window_spin.value()), 0.05) * sample_rate)), 16)
            if values.size >= window:
                hop = max(window // 2, 1)
                for start in range(0, values.size - window + 1, hop):
                    segment = values[start : start + window]
                    frequency = detect_tone_frequency(
                        segment,
                        sample_rate,
                        fft_length=max(int(self.fft_length_spin.value()), 4096),
                        window_name=self.window_combo.currentText(),
                    )
                    if frequency is None or frequency <= 0:
                        continue
                    rms_uv = fit_tone_rms(segment, sample_rate, frequency)
                    if rms_uv > 0:
                        rows.append({"Frequency (Hz)": float(frequency), "Reference (Vrms)": rms_uv * 1e-6})
        if not rows:
            return pd.DataFrame(columns=["Frequency (Hz)", "Reference (Vrms)"])

        frame = pd.DataFrame(rows)
        # Group readings that land on the same step: relative tolerance, because 2 %
        # of 5 Hz and 2 % of 200 Hz are very different absolute widths.
        frame["Bucket"] = np.round(np.log10(frame["Frequency (Hz)"].clip(lower=1e-9)) / 0.01)
        grouped = (
            frame.groupby("Bucket", as_index=False)
            .agg({"Frequency (Hz)": "mean", "Reference (Vrms)": "mean"})
            .sort_values("Frequency (Hz)")
            .reset_index(drop=True)
        )
        return grouped[["Frequency (Hz)", "Reference (Vrms)"]]

    def _cmrr_vcm_source(
        self,
        monitor_values: Optional[np.ndarray],
        steps,
        monitor_name: str,
        metadata_vrms: float,
    ) -> tuple[Optional[object], float, str]:
        """Resolve the four modes into (lookup, flat scalar, explanation).

        ``lookup`` wins when present: it maps frequency -> Vrms for this recording.
        ``flat`` is used only when there is no lookup. ``explanation`` goes on screen,
        because which of these is in force changes CMRR by tens of dB and that must
        never be implicit.
        """

        mode = self.cmrr_vcm_mode()
        sample_rate = self.dataset.sample_rate if self.dataset else 1.0
        amplitude = float(self.common_mode_spin.value())
        if amplitude <= 0 < metadata_vrms:
            amplitude = metadata_vrms

        if mode == VCM_FLAT:
            # One number for the whole sweep: no frequency dependence at all.
            return None, amplitude, (
                f"Flat {amplitude:g} Vrms across the whole sweep - no frequency dependence."
            )

        if mode == VCM_SINC3:
            if amplitude <= 0:
                return None, 0.0, "sinc3 roll-off needs an amplitude - set the CM voltage above."
            # The converter's output data rate is the rate the samples were recorded
            # at; the sinc3 response is defined against exactly that.
            data_rate = float(sample_rate) if sample_rate > 0 else 0.0
            if data_rate <= 0:
                return None, amplitude, "sinc3 roll-off needs a sample rate."

            def sinc3_lookup(frequencies):
                values = np.atleast_1d(np.asarray(frequencies, dtype=np.float64))
                gain_db = ads1299_sinc3_gain_db(values, data_rate)
                return amplitude * np.power(10.0, gain_db / 20.0)

            return (
                sinc3_lookup,
                amplitude,
                f"Sinc3 roll-off applied to {amplitude:g} Vrms at {data_rate:g} SPS: the applied "
                "voltage falls with frequency by the converter's own digital filter.",
            )

        if mode in (VCM_MONITOR, VCM_MONITOR_CURVE) and monitor_values is None:
            return (
                None,
                amplitude,
                "No CM monitor channel is selected, so the monitor-based mode cannot run - falling "
                "back to the flat amplitude above. Pick the channel wired across the common-mode "
                "voltage in the CM monitor box.",
            )

        if mode == VCM_MONITOR:
            table = self._monitor_voltage_table(monitor_values, steps, sample_rate)
            self.cmrr_monitor_table = table
            if table.empty:
                return (
                    None,
                    amplitude,
                    "No tone could be measured on the monitor channel - falling back to the flat "
                    "amplitude above.",
                )
            frequencies = table["Frequency (Hz)"].to_numpy(dtype=float)
            voltages = table["Reference (Vrms)"].to_numpy(dtype=float)

            def monitor_lookup(query):
                return np.interp(
                    np.atleast_1d(np.asarray(query, dtype=np.float64)), frequencies, voltages
                )

            return (
                monitor_lookup,
                amplitude,
                f"Vcm measured on {monitor_name}: "
                f"{voltages.min():.4g}-{voltages.max():.4g} Vrms across "
                f"{frequencies.min():g}-{frequencies.max():g} Hz, taken as-is (the converter "
                "already measured it, so no sinc3 correction is applied).",
            )

        # VCM_MONITOR_CURVE
        curve = self.bias_reference_curve
        if curve is None or not len(curve):
            return (
                None,
                amplitude,
                "No bias reference curve is loaded - falling back to the flat amplitude above. "
                "Build or load one with the Bias curve button.",
            )
        # Frequency and timing come from this recording; only the voltage comes from
        # the curve. Rescaled only if this run genuinely drove the generator at a
        # different setpoint than the curve was calibrated at - never by a measured
        # voltage, which would divide out the attenuation the curve exists to supply.
        scale = 1.0
        if metadata_vrms > 0 and curve.setpoint_v > 0:
            scale = float(metadata_vrms) / float(curve.setpoint_v)
        self.cmrr_vcm_scale = scale
        coverage = curve.coverage_hz() or (0.0, 0.0)
        note = (
            f"Vcm from the bias curve ({coverage[0]:g}-{coverage[1]:g} Hz), at this recording's "
            f"own frequencies: {curve.voltage_lookup(scale=scale)([coverage[0]])[0]:.4g}-"
            f"{curve.voltage_lookup(scale=scale)([coverage[1]])[0]:.4g} Vrms"
        )
        if abs(scale - 1.0) > 1e-9:
            note += f" (curve {curve.setpoint_v:g} Vrms scaled x{scale:.4g} to this run's {metadata_vrms:g} Vrms)"
        note += ". Points outside that range are dropped, not extrapolated."
        return curve.voltage_lookup(scale=scale), amplitude, note

    def compute_cmrr_sweep_view(self) -> None:
        if self._last_selection_matrix is None or not self._last_enabled_names:
            QMessageBox.information(self, "CMRR Sweep", "Select a valid time region first.")
            return

        # A recording made by the automated generator sweep carries the exact
        # schedule, so nothing has to be guessed: every step is evaluated at the
        # frequency the generator was told to produce, inside the time it held it.
        steps = parse_sweep_steps(self.source_metadata)
        metadata_vrms = _sweep_amplitude_vrms(self.source_metadata)
        monitor_name = self.cmrr_monitor_combo.currentText()
        monitor_values = None
        if monitor_name and monitor_name != NO_MONITOR_CHANNEL and monitor_name in self._last_enabled_names:
            monitor_values = self._last_selection_matrix[:, self._last_enabled_names.index(monitor_name)]

        # Everything is Vrms: the spin box, the recording metadata and the CMRR
        # definition all use the same unit, so nothing is converted by hand.
        #
        # The spin box is the amplitude of the stimulus. It is used as-is by the flat
        # mode, shaped by the sinc3 filter in the sinc3 mode, and ignored by the two
        # monitor modes, which take the voltage from the recording itself.
        spin_vrms = float(self.common_mode_spin.value())
        base_vrms = spin_vrms if spin_vrms > 0 else (metadata_vrms if steps else 0.0)
        self.cmrr_vcm_scale = 1.0
        common_mode_lookup, common_mode_voltage, vcm_note = self._cmrr_vcm_source(
            monitor_values, steps, monitor_name, metadata_vrms
        )
        self.cmrr_vcm_note = vcm_note
        label = getattr(self, "cmrr_vcm_label", None)
        if label is not None:
            label.setText(vcm_note)
        if common_mode_voltage <= 0 and common_mode_lookup is None:
            QMessageBox.information(
                self,
                "CMRR Sweep",
                "Enter a positive common-mode voltage in Vrms, or choose a CM voltage mode that "
                "reads it from the recording.",
            )
            return

        self._clear_cmrr_sweep_plot()
        self.cmrr_bias_overlay = None
        if getattr(self, "cmrr_reference_note_label", None) is not None:
            self.cmrr_reference_note_label.setText("")
        combined_frames: list[pd.DataFrame] = []
        accepted_total = 0
        rejected_total = 0
        plotted_channels = 0

        for channel_index, channel_name in enumerate(self._last_enabled_names):
            if monitor_values is not None and channel_name == monitor_name:                # The monitor channel measures the stimulus itself, so a CMRR
                # number for it would be meaningless.
                continue
            signal_values = self._last_selection_matrix[:, channel_index]
            if steps:
                result = compute_planned_cmrr_sweep(
                    values=signal_values,
                    sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                    steps=steps,
                    common_mode_v_rms=common_mode_voltage,
                    signal_scale_to_volts=1e-6,
                    correction_enabled=self.cmrr_correction_check.isChecked(),
                    correction_fs_hz=float(self.cmrr_correction_fs_spin.value()),
                    common_mode_lookup=common_mode_lookup,
                )
            elif monitor_values is not None:
                result = compute_reference_cmrr_sweep(
                    values=signal_values,
                    reference_values=monitor_values,
                    sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                    common_mode_v_rms=common_mode_voltage,
                    window_duration_s=float(self.cmrr_sweep_window_spin.value()),
                    fft_length=int(self.fft_length_spin.value()),
                    window_name=self.window_combo.currentText(),
                    signal_scale_to_volts=1e-6,
                    correction_enabled=self.cmrr_correction_check.isChecked(),
                    correction_fs_hz=float(self.cmrr_correction_fs_spin.value()),
                    stability_tolerance_hz=max(float(self.cmrr_sweep_tolerance_spin.value()), 0.1),
                    common_mode_lookup=common_mode_lookup,
                )
            else:
                result = compute_cmrr_sweep(
                    values=signal_values,
                    sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                    common_mode_voltage=common_mode_voltage,
                    fft_length=int(self.fft_length_spin.value()),
                    window_name=self.window_combo.currentText(),
                    signal_scale_to_volts=1e-6,
                    correction_enabled=self.cmrr_correction_check.isChecked(),
                    correction_fs_hz=float(self.cmrr_correction_fs_spin.value()),
                    window_duration_s=float(self.cmrr_sweep_window_spin.value()),
                    stability_tolerance_hz=float(self.cmrr_sweep_tolerance_spin.value()),
                    min_hold_s=self._sweep_hold_seconds("cmrr"),
                    exclude_mains=self._sweep_exclude_mains("cmrr"),
                    **self._sweep_tracking_options("cmrr"),
                    common_mode_lookup=common_mode_lookup,
                )
            accepted_total += result.accepted_windows
            rejected_total += result.rejected_windows
            if result.dataframe.empty:
                continue

            plotted_channels += 1
            channel_frame = result.dataframe.copy()
            channel_frame.insert(0, "Channel", channel_name)
            combined_frames.append(channel_frame)

            color = pg.intColor(channel_index, hues=max(8, len(self._last_enabled_names)), values=1.0)
            curve = self.cmrr_sweep_plot.plot(
                channel_frame["Frequency (Hz)"],
                channel_frame["CMRR (dB)"],
                pen=pg.mkPen(color, width=SWEEP_TRACE_WIDTH),
                symbol="o",
                symbolSize=4,
                name=channel_name,
            )
            self.cmrr_sweep_curves.append(curve)

        if not combined_frames:
            self.cmrr_sweep_df = pd.DataFrame()
            self.cmrr_sweep_info_label.setText(f"0 ch | 0 ok | {rejected_total} rej")
            self.cmrr_sweep_fit_df = pd.DataFrame()
            return

        self.cmrr_sweep_df = pd.concat(combined_frames, ignore_index=True)
        correction_tag = "corr on" if self.cmrr_correction_check.isChecked() else "corr off"
        if steps:
            source_tag = f"generator plan ({len(steps)} steps)"
        elif monitor_values is not None:
            source_tag = f"from {monitor_name}"
        else:
            source_tag = "auto"
        if common_mode_lookup is not None:
            source_tag += f" | Vcm {self._vcm_used_summary()}"
        else:
            source_tag += f" | Vcm {VCM_MODE_LABELS.get(self.cmrr_vcm_mode(), '?')} {common_mode_voltage:.4g} Vrms"
        self.cmrr_sweep_info_label.setText(
            self._sweep_info_text(
                "cmrr",
                f"{plotted_channels} ch | {correction_tag} | f {source_tag}",
                accepted_total,
                rejected_total,
            )
        )
        self._refresh_cmrr_reference_figure(monitor_name, common_mode_voltage)
        self._redraw_cmrr_sweep_fit()

    def _vcm_used_summary(self) -> str:
        """Short description of the Vcm the curve actually supplied."""

        frame = self.cmrr_sweep_df
        if frame.empty or "Vcm Used (Vrms)" not in frame.columns:
            return "none"
        values = np.asarray(frame["Vcm Used (Vrms)"], dtype=float)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return "none"
        return f"{values.min():.4g}-{values.max():.4g} Vrms"

    def _clear_sweep_plot(self, prefix: str, plot: pg.PlotWidget) -> None:
        """Remove the curves, the fit lines and the stale legend entries."""

        for curve in getattr(self, f"{prefix}_sweep_curves", []):
            plot.removeItem(curve)
        getattr(self, f"{prefix}_sweep_curves", []).clear()
        for curve in getattr(self, f"{prefix}_sweep_fit_curves", []):
            plot.removeItem(curve)
        setattr(self, f"{prefix}_sweep_fit_curves", [])
        legend = getattr(self, f"{prefix}_sweep_legend", None)
        if legend is not None:
            # removeItem() leaves the legend label behind, so a re-run would
            # otherwise stack duplicate channel names on every sweep.
            legend.clear()

    def _clear_crosstalk_sweep_plot(self) -> None:
        self._clear_sweep_plot("crosstalk", self.crosstalk_sweep_plot)

    def _clear_gain_sweep_plot(self) -> None:
        self._clear_sweep_plot("gain", self.gain_sweep_plot)

    def _clear_cmrr_sweep_plot(self) -> None:
        self._clear_sweep_plot("cmrr", self.cmrr_sweep_plot)

    def _refresh_exclusion_overlays(self) -> None:
        for item in self.exclusion_items:
            self.waveform_plot.removeItem(item)
        self.exclusion_items.clear()

        for start_s, end_s in self.excluded_ranges:
            region_item = pg.LinearRegionItem(
                values=(start_s, end_s),
                orientation=pg.LinearRegionItem.Vertical,
                movable=False,
                brush=pg.mkBrush(220, 38, 38, 40),
                pen=pg.mkPen(220, 38, 38, 140),
            )
            region_item.setZValue(20)
            self.waveform_plot.addItem(region_item)
            self.exclusion_items.append(region_item)

    def exclude_selected_region(self) -> None:
        if not self.dataset:
            return
        selected_range = self._get_selected_range()
        if selected_range is None:
            return
        self.edit_history.append(EditAction("exclude", selected_range))
        self._replay_edit_history()

    def delete_selected_region(self) -> None:
        if not self.dataset:
            return
        selected_range = self._get_selected_range()
        if selected_range is None:
            return
        self.edit_history.append(EditAction("remove", selected_range))
        self._replay_edit_history()

    def keep_only_selected_region(self) -> None:
        if not self.dataset:
            return
        selected_range = self._get_selected_range()
        if selected_range is None:
            return
        self.edit_history.append(EditAction("keep", selected_range))
        self._replay_edit_history()

    def clear_exclusions(self) -> None:
        if not self.dataset or not self.excluded_ranges:
            return
        self.edit_history.append(EditAction("clear_exclusions"))
        self._replay_edit_history()

    def undo_last_edit(self) -> None:
        if not self.edit_history:
            QMessageBox.information(self, "Undo", "There is no edit history to undo.")
            return
        self.edit_history.pop()
        self._replay_edit_history()

    def restore_original_data(self) -> None:
        if not self.original_dataset:
            return
        self.edit_history = []
        self._replay_edit_history()

    def _get_selected_range(self) -> Optional[tuple[float, float]]:
        if not self.dataset or self.dataset.sample_count == 0:
            return None
        start_s, end_s = self.selection_region.getRegion()
        start_s, end_s = sorted((float(start_s), float(end_s)))
        if abs(end_s - start_s) <= 1e-12:
            QMessageBox.information(self, "Selection", "Select a non-zero time range first.")
            return None
        return (start_s, end_s)

    def _replay_edit_history(self) -> None:
        if not self.original_dataset:
            return

        dataset = self.original_dataset
        excluded_ranges: list[tuple[float, float]] = []

        for action in self.edit_history:
            if action.action_type == "exclude" and action.time_range is not None:
                excluded_ranges = merge_ranges([*excluded_ranges, action.time_range])
                continue

            if action.action_type == "clear_exclusions":
                excluded_ranges = []
                continue

            if action.time_range is None:
                continue

            if action.action_type == "remove":
                time_s, data = hard_remove_ranges(dataset.time_s, dataset.data, [action.time_range], dataset.sample_rate)
            elif action.action_type == "keep":
                time_s, data = keep_only_range(dataset.time_s, dataset.data, action.time_range[0], action.time_range[1], dataset.sample_rate)
            else:
                continue

            if data.shape[0] < 8:
                QMessageBox.warning(
                    self,
                    "Time Editing",
                    "That edit would leave fewer than 8 samples, so it was not applied.",
                )
                self.edit_history = [entry for entry in self.edit_history if entry is not action]
                return self._replay_edit_history()

            dataset = SignalDataset(
                file_path=dataset.file_path,
                sample_rate=dataset.sample_rate,
                time_s=time_s,
                channel_names=list(dataset.channel_names),
                data=data,
                source_timestamp=None,
            )
            excluded_ranges = []

        self.dataset = dataset
        self.excluded_ranges = excluded_ranges
        self.metrics_df = pd.DataFrame()
        self.crosstalk_df = pd.DataFrame()
        self.crosstalk_sweep_df = pd.DataFrame()
        self.gain_sweep_df = pd.DataFrame()
        self.spectra = []
        self._invalidate_filter_cache()
        self._rebuild_waveform_items()
        self._initialize_selection()
        self._refresh_analysis()

        if not self.edit_history:
            self.statusBar().showMessage("Original data restored.")
            return

        last_action = self.edit_history[-1].action_type
        action_message = {
            "exclude": "Selected range excluded from analysis.",
            "remove": "Selected range deleted and the time axis was rebuilt.",
            "keep": "Only the selected range was kept.",
            "clear_exclusions": "Excluded ranges were cleared.",
        }.get(last_action, "Edit applied.")
        self.statusBar().showMessage(action_message)

    # -- export ------------------------------------------------------------

    def edit_measurement(self) -> None:
        """Edit the measurement description that goes into every export."""

        dialog = MeasurementNameDialog(self.measurement, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.measurement = dialog.measurement()
        self.export_options.measurement = self.measurement
        if self.recording_name:
            self.export_options.last_stem = full_export_stem(self.recording_name, self.measurement)
        else:
            self.export_options.last_stem = self.measurement.slug()
        self.export_store.save(self.export_options)
        self._update_session_label()

    def _update_session_label(self) -> None:
        """Show which recording name exports will reuse."""

        label = getattr(self, "session_label", None)
        if label is None:
            return
        if self.recording_name:
            label.setText(f"Recording: {self.recording_name}")
        elif self.dataset is not None and self.dataset.file_path is not None:
            label.setText(f"Recording: {self.dataset.file_path.stem}")
        else:
            label.setText("Recording: -")
        label.setToolTip(
            "This name comes from the recording header (the name you gave it in the live recorder) "
            "and is used by default when exporting."
        )
        measurement_label = getattr(self, "measurement_label", None)
        if measurement_label is not None:
            measurement_label.setText("   |   ".join(self.measurement.describe_lines()))

    def _available_export_sections(self) -> list[str]:
        """Sections that currently have data behind them."""

        available = ["info"]
        if self.dataset is not None:
            available += ["waveform", "fft", "metrics", "crosstalk"]
        if not self.crosstalk_sweep_df.empty:
            available.append("crosstalk_sweep")
        if not self.gain_sweep_df.empty:
            available.append("gain_sweep")
        if not self.cmrr_sweep_df.empty:
            available.append("cmrr_sweep")
        if not self.cmrr_sweep_fit_df.empty:
            available += ["cmrr_fits", "cmrr_50hz"]
        if not self.cmrr_mean_fit_df.empty:
            available.append("cmrr_fit_mean")
        if not self.cmrr_reference_df.empty:
            available.append("reference")
        return available

    def open_export_dialog(self, focus_format: str = "pdf") -> None:
        """Ask what to include, remember the answer, then write the files."""

        if self.dataset is None:
            QMessageBox.information(self, "Export", "Load a recording before exporting.")
            return
        if self.metrics_df.empty:
            self._refresh_analysis()
        if self.metrics_df.empty:
            QMessageBox.information(self, "Export", "No analysis results are available to export.")
            return

        if not self.export_options.last_stem:
            self.export_options.last_stem = self._default_export_stem()
        dialog = ExportSelectionDialog(
            self.export_options,
            self,
            available=self._available_export_sections(),
            focus_format=focus_format,
        )
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        options = dialog.options()
        if not (options.pdf or options.png or options.csv):
            QMessageBox.information(self, "Export", "Pick at least one format (PDF, PNG or CSV).")
            return
        self.export_options = options
        self.measurement = options.measurement
        self.export_store.save(options)
        self._run_export(options)

    def _default_export_stem(self) -> str:
        if self.recording_name:
            return full_export_stem(self.recording_name, self.measurement)
        if self.dataset is not None and self.dataset.file_path is not None:
            return self.dataset.file_path.stem
        return "analysis"

    def _prompt_export_base(self, options: ExportOptions) -> Optional[Path]:
        stem = options.last_stem.strip() or self._default_export_stem()
        if self.dataset is not None and self.dataset.file_path is not None:
            default_path = Path(options.output_dir or self.dataset.file_path.parent) / stem
        else:
            default_path = Path(options.output_dir or Path.cwd()) / stem
        pattern = (
            "*.pdf" if options.pdf and not (options.png or options.csv)
            else "*.png" if options.png and not (options.pdf or options.csv)
            else "*.csv" if options.csv and not (options.pdf or options.png)
            else "*"
        )
        file_name, _ = QFileDialog.getSaveFileName(
            self, "Export report", str(default_path) + (".pdf" if options.pdf else ""), f"Files ({pattern})"
        )
        if not file_name:
            return None
        return Path(file_name)

    def _run_export(self, options: ExportOptions) -> None:
        base = self._prompt_export_base(options)
        if base is None:
            return
        options.output_dir = str(base.parent)
        options.last_stem = base.stem
        self.export_store.save(options)

        written: list[str] = []
        try:
            if options.pdf:
                written.append(str(self._write_pdf(base.with_suffix(".pdf"), options)))
            if options.png:
                written.extend(self._write_png(base, options))
            if options.csv:
                written.extend(self._write_csv(base, options))
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed.\n\n{exc}")
            return
        if not written:
            QMessageBox.information(self, "Export", "Nothing matched the selected sections.")
            return
        QMessageBox.information(self, "Export", "Saved:\n" + "\n".join(written))

    # -- report content ----------------------------------------------------

    def _report_title(self, options: ExportOptions) -> str:
        stem = options.last_stem or self._default_export_stem()
        return stem

    def _report_subtitle(self) -> str:
        parts: list[str] = []
        if self.dataset is not None:
            parts.append(f"File: {self.dataset.file_path.name}")
            parts.append(f"Sample rate: {self.dataset.sample_rate:.3f} Hz")
            parts.append(f"{self.dataset.channel_count} channel(s)")
        parts.append(f"Filters: 50 Hz={'on' if self.notch_50_check.isChecked() else 'off'}")
        parts.append(f"100 Hz={'on' if self.notch_100_check.isChecked() else 'off'}")
        return "   |   ".join(parts)

    def _analysis_settings_lines(self) -> list[str]:
        """Everything the user typed in, so the report documents the setup."""

        selection_text = self.selection_summary.text()
        lines = [selection_text if selection_text.lower().startswith("selection") else f"Selection: {selection_text}"]
        lines.append(f"FFT length: {int(self.fft_length_spin.value())}   Window: {self.window_combo.currentText()}")
        lines.append(
            f"Notch filters: 50 Hz={'on' if self.notch_50_check.isChecked() else 'off'}, "
            f"100 Hz={'on' if self.notch_100_check.isChecked() else 'off'} (Q={self.filter_q_spin.value():g})"
        )
        common_mode = float(self.common_mode_spin.value())
        lines.append(
            "Common-mode input: " + (f"{common_mode:.6g} V" if common_mode > 0 else "not specified")
        )
        if self.cmrr_frequency_mode_combo.currentIndex() == 1:
            lines.append(f"CMRR frequency: {self.cmrr_frequency_spin.value():g} Hz (manual)")
        else:
            lines.append("CMRR frequency: from the detected fundamental")
        lines.append(
            "ADS1299 sinc3 correction: "
            + (f"on (fs = {self.cmrr_correction_fs_spin.value():g} Hz)" if self.cmrr_correction_check.isChecked() else "off")
        )
        if self.gain_sweep_input_spin.value() > 0:
            lines.append(f"Gain-sweep stimulus: {self.gain_sweep_input_spin.value():.6g} V")
        ratio = self._recorded_attenuator_ratio()
        series = _metadata_float(self.source_metadata, "Input Attenuator Series (ohm)")
        shunt = _metadata_float(self.source_metadata, "Input Attenuator Shunt (ohm)")
        source = "recorded" if "Input Attenuator Ratio" in self.source_metadata else "default"
        divider = f"Input attenuator: {ratio:.6g}"
        if series > 0 and shunt > 0:
            divider += f" ({shunt:g} ohm / ({series:g} + {shunt:g}) ohm)"
        lines.append(f"{divider}, {source}")
        lines.append(
            f"Crosstalk source: {self.crosstalk_source_combo.currentText() or '-'}   "
            f"Channels in analysis: {len(self._last_enabled_names)}"
        )
        if self._last_mean_names:
            lines.append("Averaged channels: " + ", ".join(self._last_mean_names))
        return lines

    def _host_tab(self, widget: QWidget) -> Optional[tuple[QTabWidget, int]]:
        """Find the tab page that owns ``widget``, so a hidden one can be shown.

        QTabWidget inserts a QStackedWidget between itself and the pages, so the
        page is not the direct child of the tab widget.
        """

        chain: list[QWidget] = []
        page: Optional[QWidget] = widget
        while page is not None:
            chain.append(page)
            parent = page.parentWidget()
            if isinstance(parent, QTabWidget):
                for candidate in chain:
                    index = parent.indexOf(candidate)
                    if index >= 0:
                        return parent, index
                return None
            page = parent
        return None

    def _plot_pixmap(self, plot_widget, aspect: Optional[float] = None):
        """Render a plot for the report at high resolution.

        ``grab()`` copies whatever size the widget happens to have, so the report
        would depend on the window layout. ImageExporter re-renders the scene and
        keeps the plot's own proportions, which is what makes the figure legible
        instead of squashed. ``aspect`` (height / width) overrides those
        proportions so a wide on-screen plot can still fill a page in the report.
        """

        target_width = 2000
        width = max(plot_widget.width(), 1)
        height = max(plot_widget.height(), 1)
        restored = None
        shown_tab = None
        if aspect:
            # A widget on an inactive tab is still laid out at its minimum size,
            # so resizing it there changes nothing. Show its tab for the render.
            if not plot_widget.isVisible():
                host = self._host_tab(plot_widget)
                if host is not None:
                    tab, index = host
                    shown_tab = (tab, tab.currentIndex())
                    tab.setCurrentIndex(index)
                    QApplication.instance().processEvents()
            # The exporter keeps the scene's own proportions, so the only way to
            # get a different shape is to lay the plot out that way for the
            # render. Updates stay off so the reflow never reaches the screen.
            aspect = min(max(float(aspect), 0.15), 2.0)
            width = max(plot_widget.width(), 900)
            height = max(int(round(width * aspect)), 320)
            restored = (plot_widget.minimumSize(), plot_widget.maximumSize(), plot_widget.size())
            plot_widget.setUpdatesEnabled(False)
            plot_widget.setFixedSize(width, height)
            # ImageExporter derives its size from the scene rect, and a hidden
            # widget only re-lays-out when it is painted, so grab() here is what
            # actually applies the requested shape to the figure.
            plot_widget.grab()
            QApplication.instance().processEvents()
        exporter = pyqtgraph.exporters.ImageExporter(plot_widget.plotItem)
        params = exporter.parameters()
        try:
            # The exporter links width and height through the scene's own aspect
            # ratio, so setting both makes them fight. Width alone is enough and
            # keeps the figure's proportions consistent.
            params["width"] = target_width
            if not aspect:
                params["height"] = max(int(target_width * height / width), 600)
        except (KeyError, TypeError):
            pass
        rendered = exporter.export(toBytes=True)
        if restored is not None:
            plot_widget.setMinimumSize(restored[0])
            plot_widget.setMaximumSize(restored[1])
            plot_widget.resize(restored[2])
            plot_widget.setUpdatesEnabled(True)
            if shown_tab is not None:
                shown_tab[0].setCurrentIndex(shown_tab[1])
            QApplication.instance().processEvents()
            # The scene kept the temporary shape and pyqtgraph only re-lays out
            # when the view is painted, so paint once more at the real size to
            # leave the widget exactly as it was found.
            plot_widget.grab()
            QApplication.instance().processEvents()
        if isinstance(rendered, QImage):
            image = rendered
        else:
            image = QImage.fromData(rendered, "PNG")
        if image.isNull():
            return plot_widget.grab()
        return QPixmap.fromImage(image)

    def _write_pdf(self, path: Path, options: ExportOptions) -> Path:
        writer = ReportWriter(path, self._report_title(options), self._report_subtitle())
        if options.enabled("info"):
            writer.heading("Measurement")
            writer.paragraph(self.measurement.describe_lines())
            writer.heading("Analysis settings", level=2)
            writer.paragraph(self._analysis_settings_lines(), size=8.5)
            if self.source_metadata:
                writer.heading("Recording metadata", level=2)
                writer.paragraph(
                    [f"{key}: {value}" for key, value in self.source_metadata.items() if key != "format"],
                    size=8,
                )
        if options.enabled("waveform") and self.dataset is not None:
            # A tall waveform is what makes a multi-channel trace readable, and it
            # only fits on a page of its own.
            writer.new_page()
            writer.heading("Waveform", keep_with_next=0.86)
            writer.image(
                self._plot_pixmap(self.waveform_plot, aspect=0.62),
                "Selected analysis region",
                max_height=0.80,
            )
        if options.enabled("fft"):
            writer.heading("FFT", keep_with_next=0.54)
            # The FFT tab is a tall narrow column, so rendering it with its own
            # proportions produced a strip in the middle of the page. Force a wide
            # aspect: it is a spectrum, it belongs across the page.
            writer.image(
                self._plot_pixmap(self.fft_plot, aspect=FFT_FIGURE_ASPECT),
                "RMS spectrum of the analysed channel",
                max_height=0.55,
            )
        if options.enabled("metrics"):
            writer.heading("Metrics", keep_with_next=0.14)
            writer.table(self.metrics_df, caption="Per-channel metrics")
        if options.enabled("crosstalk") and not self.crosstalk_df.empty:
            writer.heading("Crosstalk", keep_with_next=0.14)
            writer.table(self.crosstalk_df, caption="Crosstalk from the selected source")
        if options.enabled("crosstalk_sweep") and not self.crosstalk_sweep_df.empty:
            writer.heading("Crosstalk sweep", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.crosstalk_sweep_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Crosstalk versus frequency",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        if options.enabled("gain_sweep") and not self.gain_sweep_df.empty:
            writer.heading("Gain sweep", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.gain_sweep_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Response versus applied level",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        # The CMRR figures are exported as one block that mirrors the tab: the
        # measured points, the fits on their own, the mean of the fits and the
        # reference input. Enabling the CMRR sweep is enough - the individual
        # section keys only add nothing extra, so a saved checklist from an older
        # version still produces the complete report.
        cmrr_figures = options.enabled("cmrr_sweep") or any(
            options.enabled(key) for key in ("cmrr_fits", "cmrr_fit_mean", "cmrr_50hz", "reference")
        )
        if cmrr_figures and not self.cmrr_sweep_df.empty:
            writer.heading("CMRR sweep", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.cmrr_sweep_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Measured CMRR versus frequency, one curve per channel",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        if cmrr_figures and not self.cmrr_sweep_fit_df.empty:
            writer.heading("CMRR fitted curves", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.cmrr_fit_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Polynomial fit of each channel in log10(frequency), without the measured points",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        if cmrr_figures and not self.cmrr_mean_fit_df.empty:
            writer.heading("Mean of the fitted curves", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.cmrr_fit_mean_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Average fitted CMRR across the plotted channels, with the channel spread",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        if cmrr_figures and not self.cmrr_reference_df.empty:
            writer.heading("Reference channel", keep_with_next=0.42)
            writer.image(
                self._plot_pixmap(self.cmrr_reference_plot, aspect=SWEEP_FIGURE_ASPECT),
                "Reference input actually measured (and its fit) against the amplitude the "
                "generator was set to",
                max_height=SWEEP_FIGURE_MAX_HEIGHT,
            )
        if cmrr_figures and not self.cmrr_sweep_fit_df.empty:
            writer.heading(f"CMRR at {SWEEP_FIT_REFERENCE_HZ:g} Hz", keep_with_next=0.14)
            writer.paragraph(
                [
                    f"Taken from each channel's fitted curve, not from a swept point, so no "
                    f"measurement sits exactly on {SWEEP_FIT_REFERENCE_HZ:g} Hz."
                ],
                size=8.5,
            )
            writer.table(self._cmrr_50hz_frame(), caption="Fitted CMRR at the reference frequency")
        return writer.close()

    def _cmrr_50hz_frame(self) -> pd.DataFrame:
        """The 50 Hz table as a frame, with the mean of the channel fits appended."""

        value_name = f"Fit @ {SWEEP_FIT_REFERENCE_HZ:g} Hz"
        frame = self.cmrr_sweep_fit_df
        if frame.empty or value_name not in frame:
            return pd.DataFrame()
        table = frame[["Channel", value_name, "R2", "Points"]].copy()
        table.loc[len(table)] = [
            "Mean of fits",
            float(frame[value_name].mean()),
            float(frame["R2"].mean()),
            int(frame["Points"].sum()),
        ]
        return table.rename(columns={value_name: f"CMRR @ {SWEEP_FIT_REFERENCE_HZ:g} Hz (dB)", "R2": "R^2"})

    def _write_png(self, base: Path, options: ExportOptions) -> list[str]:
        written: list[str] = []
        targets = (
            ("waveform", self.waveform_plot, "waveform"),
            ("fft", self.fft_plot, "fft"),
            ("crosstalk_sweep", self.crosstalk_sweep_plot, "crosstalk_sweep"),
            ("gain_sweep", self.gain_sweep_plot, "gain_sweep"),
            ("cmrr_sweep", self.cmrr_sweep_plot, "cmrr_sweep"),
            ("cmrr_fits", self.cmrr_fit_plot, "cmrr_fits"),
            ("cmrr_fit_mean", self.cmrr_fit_mean_plot, "cmrr_fit_mean"),
            ("reference", self.cmrr_reference_plot, "reference_channel"),
        )
        for key, plot_widget, suffix in targets:
            if not options.enabled(key):
                continue
            suffix_name = suffix if key in ("crosstalk_sweep", "gain_sweep", "cmrr_sweep") else suffix
            if key in ("crosstalk_sweep", "gain_sweep", "cmrr_sweep"):
                frame = {
                    "crosstalk_sweep": self.crosstalk_sweep_df,
                    "gain_sweep": self.gain_sweep_df,
                    "cmrr_sweep": self.cmrr_sweep_df,
                }[key]
                if frame.empty:
                    continue
            if key in ("cmrr_fits", "cmrr_50hz") and self.cmrr_sweep_fit_df.empty:
                continue
            if key == "cmrr_fit_mean" and self.cmrr_mean_fit_df.empty:
                continue
            if key == "reference" and self.cmrr_reference_df.empty:
                continue
            target = base.with_name(f"{base.stem}_{suffix_name}.png")
            exporter = pyqtgraph.exporters.ImageExporter(plot_widget.plotItem)
            try:
                exporter.parameters()["width"] = 2200
            except (KeyError, TypeError):
                pass
            exporter.export(str(target))
            written.append(str(target))
        return written

    def _write_csv(self, base: Path, options: ExportOptions) -> list[str]:
        written: list[str] = []

        def dump(frame: pd.DataFrame, suffix: str) -> None:
            if frame is None or frame.empty:
                return
            target = base.with_name(f"{base.stem}_{suffix}.csv")
            frame.to_csv(target, index=False)
            written.append(str(target))

        if options.enabled("info"):
            rows: list[dict[str, str]] = []
            for line in self.measurement.describe_lines():
                key, _, value = line.partition(":")
                rows.append({"Section": "Measurement", "Key": key.strip(), "Value": value.strip()})
            for line in self._analysis_settings_lines():
                key, _, value = line.partition(":")
                rows.append({"Section": "Analysis", "Key": key.strip(), "Value": value.strip()})
            for key, value in self.source_metadata.items():
                rows.append({"Section": "Recording", "Key": key, "Value": value})
            dump(pd.DataFrame(rows), "info")
        if options.enabled("metrics"):
            dump(self.metrics_df, "metrics")
        if options.enabled("crosstalk"):
            dump(self.crosstalk_df, "crosstalk")
        if options.enabled("crosstalk_sweep"):
            dump(self.crosstalk_sweep_df, "crosstalk_sweep")
        if options.enabled("gain_sweep"):
            dump(self.gain_sweep_df, "gain_sweep")
        if options.enabled("cmrr_sweep"):
            dump(self.cmrr_sweep_df, "cmrr_sweep")
        if options.enabled("cmrr_50hz"):
            dump(self._cmrr_50hz_frame(), "cmrr_50hz")
        if options.enabled("cmrr_fit_mean"):
            dump(self.cmrr_mean_fit_df, "cmrr_fit_mean")
        if options.enabled("reference"):
            dump(self.cmrr_reference_df, "reference_channel")
        return written

    # -- menu entry points (kept for muscle memory) ------------------------

    def export_csv(self) -> None:
        self.open_export_dialog("csv")

    def export_png(self) -> None:
        self.open_export_dialog("png")

    def export_pdf(self) -> None:
        self.open_export_dialog("pdf")

    @staticmethod
    def _format_table_value(value: object) -> str:
        if value is None:
            return ""
        if isinstance(value, float):
            if pd.isna(value):
                return ""
            if value in (float("inf"), float("-inf")):
                return str(value)
            return f"{value:.6f}"
        return str(value)


def main() -> None:
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    app = QApplication(sys.argv)
    install_wheel_guard(app)
    if "--live" in sys.argv:
        from live_gui import LiveRecorderWindow

        window = LiveRecorderWindow()
    else:
        window = ScientificSignalAnalyzerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()