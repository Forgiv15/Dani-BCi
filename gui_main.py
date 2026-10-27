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
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
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
)
from data_loader import (
    CsvPreview,
    SignalDataset,
    load_openbci_csv,
    openbci_sample_rate,
    read_openbci_metadata,
    sniff_csv_layout,
)
from fft_analysis import (
    CHANNEL_AVERAGE_LABEL,
    VREF_VOLTS,
    DEFAULT_INPUT_ATTENUATOR_RATIO,
    FFTSettings,
    SpectrumResult,
    WINDOW_MAP,
    analyze_multichannel,
    compute_cmrr_sweep,
    compute_crosstalk_sweep,
    compute_gain_sweep,
)
from signal_processing import FilterSettings, apply_filters, extract_time_region, hard_remove_ranges, keep_only_range, merge_ranges


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

        self._build_ui()
        self._build_menu()

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
        right_splitter.setSizes([600, 340])
        bottom_splitter.setSizes([740, 540])

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
        self.cmrr_sweep_info_label = QLabel("Select region and set Vcm")
        self.cmrr_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.cmrr_sweep_info_label)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        self.cmrr_sweep_plot = pg.PlotWidget()
        self.cmrr_sweep_plot.setLabel("bottom", "Frequency", units="Hz")
        self.cmrr_sweep_plot.setLabel("left", "CMRR", units="dB")
        self.cmrr_sweep_plot.showGrid(x=True, y=True, alpha=0.28)
        layout.addWidget(self.cmrr_sweep_plot)

        self.cmrr_sweep_table = QTableWidget()
        layout.addWidget(self.cmrr_sweep_table)
        return page

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

        control_row.addWidget(QLabel("Input before attenuator"))
        self.gain_sweep_input_spin = QDoubleSpinBox()
        self.gain_sweep_input_spin.setRange(0.0, 1_000_000_000.0)
        self.gain_sweep_input_spin.setDecimals(9)
        self.gain_sweep_input_spin.setValue(0.0)
        self.gain_sweep_input_spin.setSuffix(" V")
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

        attenuator_label = QLabel("Atten 100/1000100")
        attenuator_label.setToolTip("Input at ADC = input before attenuator * 100 / 1000100")
        control_row.addWidget(attenuator_label)

        self.gain_sweep_info_label = QLabel("Select region, input, channel")
        self.gain_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.gain_sweep_info_label)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        self.gain_sweep_plot = pg.PlotWidget()
        self.gain_sweep_plot.setLabel("bottom", "Frequency", units="Hz")
        self.gain_sweep_plot.setLabel("left", "Gain", units="dB")
        self.gain_sweep_plot.showGrid(x=True, y=True, alpha=0.28)
        layout.addWidget(self.gain_sweep_plot)

        self.gain_sweep_table = QTableWidget()
        layout.addWidget(self.gain_sweep_table)
        return page

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
        self.crosstalk_sweep_info_label = QLabel("Select region and source")
        self.crosstalk_sweep_info_label.setWordWrap(False)
        control_row.addWidget(self.crosstalk_sweep_info_label)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        self.crosstalk_sweep_plot = pg.PlotWidget()
        self.crosstalk_sweep_plot.setLabel("bottom", "Frequency", units="Hz")
        self.crosstalk_sweep_plot.setLabel("left", "Crosstalk", units="dB")
        self.crosstalk_sweep_plot.showGrid(x=True, y=True, alpha=0.28)
        layout.addWidget(self.crosstalk_sweep_plot)

        self.crosstalk_sweep_table = QTableWidget()
        layout.addWidget(self.crosstalk_sweep_table)
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
        self.common_mode_spin.setSuffix(" V")
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

        self.crosstalk_source_combo.blockSignals(True)
        self.gain_sweep_channel_combo.blockSignals(True)
        self.crosstalk_source_combo.clear()
        self.gain_sweep_channel_combo.clear()
        self.crosstalk_source_combo.addItems(enabled_names)
        self.gain_sweep_channel_combo.addItems(enabled_names)

        if current_source in enabled_names:
            self.crosstalk_source_combo.setCurrentText(current_source)
        if current_gain_channel in enabled_names:
            self.gain_sweep_channel_combo.setCurrentText(current_gain_channel)
        self.crosstalk_source_combo.blockSignals(False)
        self.gain_sweep_channel_combo.blockSignals(False)

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
        self.crosstalk_sweep_table.clear()
        self.crosstalk_sweep_info_label.setText("Sweep stale")

        self._mark_gain_sweep_stale()

        self.cmrr_sweep_df = pd.DataFrame()
        self._clear_cmrr_sweep_plot()
        self.cmrr_sweep_table.clear()
        self.cmrr_sweep_info_label.setText("Sweep stale")

    def _mark_gain_sweep_stale(self, *args) -> None:
        self.gain_sweep_df = pd.DataFrame()
        self._clear_gain_sweep_plot()
        self.gain_sweep_table.clear()
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
        self._clear_crosstalk_sweep_plot()
        combined_frames: list[pd.DataFrame] = []
        accepted_windows = 0
        rejected_windows = 0
        plotted_channels = 0

        for target_index, target_name in enumerate(self._last_enabled_names):
            result = compute_crosstalk_sweep(
                source_values=source_values,
                target_values=self._last_selection_matrix[:, target_index],
                sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
                fft_length=int(self.fft_length_spin.value()),
                window_name=self.window_combo.currentText(),
                window_duration_s=float(self.crosstalk_sweep_window_spin.value()),
                stability_tolerance_hz=float(self.crosstalk_sweep_tolerance_spin.value()),
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
                pen=pg.mkPen(color, width=1.8),
                symbol="o",
                symbolSize=4,
                name=target_name,
            )
            self.crosstalk_sweep_curves.append(curve)

        if not combined_frames:
            self.crosstalk_sweep_df = pd.DataFrame()
            self.crosstalk_sweep_table.clear()
            self.crosstalk_sweep_info_label.setText(f"Src {source_name} | 0 ch | {accepted_windows} ok | {rejected_windows} rej")
            return

        self.crosstalk_sweep_df = pd.concat(combined_frames, ignore_index=True)
        self._populate_table(self.crosstalk_sweep_table, self.crosstalk_sweep_df)
        self.crosstalk_sweep_info_label.setText(
            f"Src {source_name} | {plotted_channels} ch | {accepted_windows} ok | {rejected_windows} rej | notch on"
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
        if input_before_v <= 0:
            QMessageBox.information(self, "Gain Sweep", "Enter a positive input voltage before the attenuator first.")
            return

        channel_index = self._last_enabled_names.index(channel_name)
        result = compute_gain_sweep(
            values=self._last_selection_matrix[:, channel_index],
            sample_rate=self.dataset.sample_rate if self.dataset else 1.0,
            input_before_attenuator_v=input_before_v,
            fft_length=int(self.fft_length_spin.value()),
            window_name=self.window_combo.currentText(),
            attenuator_ratio=DEFAULT_INPUT_ATTENUATOR_RATIO,
            signal_scale_to_volts=1e-6,
            window_duration_s=float(self.gain_sweep_window_spin.value()),
            stability_tolerance_hz=float(self.gain_sweep_tolerance_spin.value()),
        )

        self._clear_gain_sweep_plot()
        if result.dataframe.empty:
            self.gain_sweep_df = pd.DataFrame()
            self.gain_sweep_table.clear()
            self.gain_sweep_info_label.setText(
                f"Ch {channel_name} | 0 ok | {result.rejected_windows} rej"
            )
            return

        self.gain_sweep_df = result.dataframe.copy()
        self.gain_sweep_df.insert(0, "Channel", channel_name)
        self._populate_table(self.gain_sweep_table, self.gain_sweep_df)

        curve = self.gain_sweep_plot.plot(
            self.gain_sweep_df["Frequency (Hz)"],
            self.gain_sweep_df["Gain (dB)"],
            pen=pg.mkPen(pg.intColor(channel_index, hues=max(8, len(self._last_enabled_names)), values=1.0), width=1.8),
            symbol="o",
            symbolSize=4,
            name=channel_name,
        )
        self.gain_sweep_curves.append(curve)

        input_after_uvrms = input_before_v * DEFAULT_INPUT_ATTENUATOR_RATIO / 1e-6
        self.gain_sweep_info_label.setText(
            f"Ch {channel_name} | in {input_after_uvrms:.6g} uVrms | {result.accepted_windows} ok | {result.rejected_windows} rej | notch on"
        )

    def compute_cmrr_sweep_view(self) -> None:
        if self._last_selection_matrix is None or not self._last_enabled_names:
            QMessageBox.information(self, "CMRR Sweep", "Select a valid time region first.")
            return
        common_mode_voltage = float(self.common_mode_spin.value())
        if common_mode_voltage <= 0:
            QMessageBox.information(self, "CMRR Sweep", "Enter a positive common-mode input voltage first.")
            return

        self._clear_cmrr_sweep_plot()
        combined_frames: list[pd.DataFrame] = []
        accepted_total = 0
        rejected_total = 0
        plotted_channels = 0

        for channel_index, channel_name in enumerate(self._last_enabled_names):
            signal_values = self._last_selection_matrix[:, channel_index]
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
                pen=pg.mkPen(color, width=1.8),
                symbol="o",
                symbolSize=4,
                name=channel_name,
            )
            self.cmrr_sweep_curves.append(curve)

        if not combined_frames:
            self.cmrr_sweep_df = pd.DataFrame()
            self.cmrr_sweep_table.clear()
            self.cmrr_sweep_info_label.setText(f"0 ch | 0 ok | {rejected_total} rej")
            return

        self.cmrr_sweep_df = pd.concat(combined_frames, ignore_index=True)
        self._populate_table(self.cmrr_sweep_table, self.cmrr_sweep_df)
        correction_tag = "corr on" if self.cmrr_correction_check.isChecked() else "corr off"
        self.cmrr_sweep_info_label.setText(f"{plotted_channels} ch | {accepted_total} ok | {rejected_total} rej | {correction_tag} | notch on")

    def _clear_crosstalk_sweep_plot(self) -> None:
        for curve in self.crosstalk_sweep_curves:
            self.crosstalk_sweep_plot.removeItem(curve)
        self.crosstalk_sweep_curves.clear()

    def _clear_gain_sweep_plot(self) -> None:
        for curve in self.gain_sweep_curves:
            self.gain_sweep_plot.removeItem(curve)
        self.gain_sweep_curves.clear()

    def _clear_cmrr_sweep_plot(self) -> None:
        for curve in self.cmrr_sweep_curves:
            self.cmrr_sweep_plot.removeItem(curve)
        self.cmrr_sweep_curves.clear()

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
        lines.append(
            f"Crosstalk source: {self.crosstalk_source_combo.currentText() or '-'}   "
            f"Channels in analysis: {len(self._last_enabled_names)}"
        )
        if self._last_mean_names:
            lines.append("Averaged channels: " + ", ".join(self._last_mean_names))
        return lines

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
        if aspect:
            # The exporter keeps the scene's own proportions, so the only way to
            # get a taller figure is to lay the plot out taller for the render.
            # Updates stay off so the reflow never reaches the screen.
            width = max(width, 640)
            restored = (plot_widget.minimumSize(), plot_widget.maximumSize(), plot_widget.size())
            plot_widget.setUpdatesEnabled(False)
            plot_widget.setFixedSize(width, max(int(width * float(aspect)), 480))
            QApplication.instance().processEvents()
            height = max(plot_widget.height(), 1)
        target_height = max(int(target_width * height / width), 600)
        exporter = pyqtgraph.exporters.ImageExporter(plot_widget.plotItem)
        params = exporter.parameters()
        try:
            # ParameterTree does not support membership tests, so assign directly.
            params["width"] = target_width
            params["height"] = target_height
        except (KeyError, TypeError):
            pass
        rendered = exporter.export(toBytes=True)
        if restored is not None:
            plot_widget.setMinimumSize(restored[0])
            plot_widget.setMaximumSize(restored[1])
            plot_widget.resize(restored[2])
            plot_widget.setUpdatesEnabled(True)
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
            writer.image(self._plot_pixmap(self.fft_plot), "RMS spectrum of the analysed channel", max_height=0.46)
        if options.enabled("metrics"):
            writer.heading("Metrics", keep_with_next=0.14)
            writer.table(self.metrics_df, caption="Per-channel metrics")
        if options.enabled("crosstalk") and not self.crosstalk_df.empty:
            writer.heading("Crosstalk", keep_with_next=0.14)
            writer.table(self.crosstalk_df, caption="Crosstalk from the selected source")
        if options.enabled("crosstalk_sweep") and not self.crosstalk_sweep_df.empty:
            writer.heading("Crosstalk sweep", keep_with_next=0.42)
            writer.image(self._plot_pixmap(self.crosstalk_sweep_plot), "Crosstalk versus frequency")
            writer.table(self.crosstalk_sweep_df, max_rows=400, caption="Crosstalk sweep data")
        if options.enabled("gain_sweep") and not self.gain_sweep_df.empty:
            writer.heading("Gain sweep", keep_with_next=0.42)
            writer.image(self._plot_pixmap(self.gain_sweep_plot), "Response versus applied level")
            writer.table(self.gain_sweep_df, max_rows=400, caption="Gain sweep data")
        if options.enabled("cmrr_sweep") and not self.cmrr_sweep_df.empty:
            writer.heading("CMRR sweep", keep_with_next=0.42)
            writer.image(self._plot_pixmap(self.cmrr_sweep_plot), "CMRR versus frequency")
            writer.table(self.cmrr_sweep_df, max_rows=400, caption="CMRR sweep data")
        return writer.close()

    def _write_png(self, base: Path, options: ExportOptions) -> list[str]:
        written: list[str] = []
        targets = (
            ("waveform", self.waveform_plot, "waveform"),
            ("fft", self.fft_plot, "fft"),
            ("crosstalk_sweep", self.crosstalk_sweep_plot, "crosstalk_sweep"),
            ("gain_sweep", self.gain_sweep_plot, "gain_sweep"),
            ("cmrr_sweep", self.cmrr_sweep_plot, "cmrr_sweep"),
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