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
from typing import Optional

import numpy as np
import pandas as pd
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt6.QtCore import QRect, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QFont, QPainter, QPageLayout, QPageSize, QPdfWriter
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
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
    QVBoxLayout,
    QWidget,
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


class ScientificSignalAnalyzerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Scientific EEG/ADC Signal Analyzer")
        self.resize(1680, 980)

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

        self._build_ui()
        self._build_menu()

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

        self.controls_panel = self._build_controls_panel()
        splitter.addWidget(self.controls_panel)

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

        splitter.setSizes([340, 1340])
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
        layout.addLayout(controls)

        self.metrics_formula_label = QLabel(
            "Units: EEG amplitudes are treated as uVrms. Selection Avg RMS is the RMS of each channel over the full selected region after exclusions, active filters, and a fixed 0.5-100 Hz bandpass filter. Auto mode finds the dominant spectral tone; Manual Hz measures the exact user-entered frequency with a sine-fit RMS estimator. "
            "50 Hz pickup is the exact 50 Hz RMS. SNR = 10 log10(Pfund / Pnoise). THD uses harmonics 2f through 5f. ENOB = (SNR - 1.76) / 6.02. "
            "CMRR uses the selected CMRR frequency or the fundamental if no CMRR frequency is specified. When ADS1299 sinc3 correction is enabled, the app applies the sinc3 gain to the user-entered common-mode input, so CMRR_corrected(f) = CMRR_measured(f) + G_ADC(f), where G_ADC(f) is typically negative at higher frequency. Crosstalk uses the selected Crosstalk frequency or the fundamental if no Crosstalk frequency is specified."
        )
        self.metrics_formula_label.setWordWrap(True)
        layout.addWidget(self.metrics_formula_label)

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
            default_sample_rate = openbci_sample_rate(read_openbci_metadata(file_path))
        except OSError:
            default_sample_rate = None

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
            return

        enabled_names = [self.dataset.channel_names[index] for index in enabled_indices]
        self._last_selection_matrix = selection_matrix.copy()
        self._last_enabled_names = list(enabled_names)
        fft_length = min(int(self.fft_length_spin.value()), max(selection_matrix.shape[0], 128))
        fft_settings = FFTSettings(
            fft_length=fft_length,
            window_name=self.window_combo.currentText(),
            manual_fundamental_hz=self._manual_fundamental_hz(),
            cmrr_frequency_hz=self._manual_cmrr_frequency_hz(),
            crosstalk_frequency_hz=self._manual_crosstalk_frequency_hz(),
            cmrr_correction_enabled=self.cmrr_correction_check.isChecked(),
            cmrr_correction_fs_hz=float(self.cmrr_correction_fs_spin.value()),
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

    def export_csv(self) -> None:
        if self.metrics_df.empty:
            self._refresh_analysis()
        if self.metrics_df.empty:
            QMessageBox.information(self, "Export", "No analysis results are available to export.")
            return

        file_name, _ = QFileDialog.getSaveFileName(self, "Export CSV", "analysis_results.csv", "CSV files (*.csv)")
        if not file_name:
            return

        base_path = Path(file_name)
        metrics_path = base_path.with_name(f"{base_path.stem}_metrics.csv")
        crosstalk_path = base_path.with_name(f"{base_path.stem}_crosstalk.csv")
        self.metrics_df.to_csv(metrics_path, index=False)
        self.crosstalk_df.to_csv(crosstalk_path, index=False)
        extra_paths = [str(metrics_path), str(crosstalk_path)]
        if not self.crosstalk_sweep_df.empty:
            crosstalk_sweep_path = base_path.with_name(f"{base_path.stem}_crosstalk_sweep.csv")
            self.crosstalk_sweep_df.to_csv(crosstalk_sweep_path, index=False)
            extra_paths.append(str(crosstalk_sweep_path))
        if not self.gain_sweep_df.empty:
            gain_sweep_path = base_path.with_name(f"{base_path.stem}_gain_sweep.csv")
            self.gain_sweep_df.to_csv(gain_sweep_path, index=False)
            extra_paths.append(str(gain_sweep_path))
        if not self.cmrr_sweep_df.empty:
            cmrr_sweep_path = base_path.with_name(f"{base_path.stem}_cmrr_sweep.csv")
            self.cmrr_sweep_df.to_csv(cmrr_sweep_path, index=False)
            extra_paths.append(str(cmrr_sweep_path))
        QMessageBox.information(self, "Export", "Saved:\n" + "\n".join(extra_paths))

    def export_png(self) -> None:
        if not self.dataset:
            QMessageBox.information(self, "Export", "Load data before exporting plots.")
            return

        file_name, _ = QFileDialog.getSaveFileName(self, "Export PNG", "analysis_plots.png", "PNG files (*.png)")
        if not file_name:
            return

        base_path = Path(file_name)
        waveform_path = base_path.with_name(f"{base_path.stem}_waveform.png")
        fft_path = base_path.with_name(f"{base_path.stem}_fft.png")
        extra_paths = [str(waveform_path), str(fft_path)]
        pyqtgraph.exporters.ImageExporter(self.waveform_plot.plotItem).export(str(waveform_path))
        pyqtgraph.exporters.ImageExporter(self.fft_plot.plotItem).export(str(fft_path))
        if not self.crosstalk_sweep_df.empty:
            crosstalk_sweep_path = base_path.with_name(f"{base_path.stem}_crosstalk_sweep.png")
            pyqtgraph.exporters.ImageExporter(self.crosstalk_sweep_plot.plotItem).export(str(crosstalk_sweep_path))
            extra_paths.append(str(crosstalk_sweep_path))
        if not self.gain_sweep_df.empty:
            gain_sweep_path = base_path.with_name(f"{base_path.stem}_gain_sweep.png")
            pyqtgraph.exporters.ImageExporter(self.gain_sweep_plot.plotItem).export(str(gain_sweep_path))
            extra_paths.append(str(gain_sweep_path))
        if not self.cmrr_sweep_df.empty:
            cmrr_sweep_path = base_path.with_name(f"{base_path.stem}_cmrr_sweep.png")
            pyqtgraph.exporters.ImageExporter(self.cmrr_sweep_plot.plotItem).export(str(cmrr_sweep_path))
            extra_paths.append(str(cmrr_sweep_path))
        QMessageBox.information(self, "Export", "Saved:\n" + "\n".join(extra_paths))

    def export_pdf(self) -> None:
        if self.metrics_df.empty:
            self._refresh_analysis()
        if self.metrics_df.empty:
            QMessageBox.information(self, "Export", "No analysis results are available to export.")
            return

        file_name, _ = QFileDialog.getSaveFileName(self, "Export PDF", "analysis_report.pdf", "PDF files (*.pdf)")
        if not file_name:
            return

        pdf_path = Path(file_name)
        writer = QPdfWriter(str(pdf_path))
        writer.setResolution(150)
        writer.setPageOrientation(QPageLayout.Orientation.Landscape)
        writer.setPageSize(QPageSize(QPageSize.PageSizeId.A4))

        painter = QPainter(writer)
        try:
            page_rect = writer.pageLayout().paintRectPixels(writer.resolution())
            left_margin = 40
            top_margin = 30
            content_width = page_rect.width() - 2 * left_margin

            painter.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
            painter.drawText(left_margin, top_margin + 20, "Scientific EEG/ADC Analysis Report")

            painter.setFont(QFont("Segoe UI", 9))
            metadata_line = (
                f"File: {self.dataset.file_path.name if self.dataset else 'N/A'}    "
                f"Sampling rate: {self.dataset.sample_rate:.3f} Hz    "
                f"Filters: 50 Hz={'on' if self.notch_50_check.isChecked() else 'off'}, "
                f"100 Hz={'on' if self.notch_100_check.isChecked() else 'off'}"
            )
            painter.drawText(left_margin, top_margin + 46, metadata_line)
            painter.drawText(left_margin, top_margin + 64, self.selection_summary.text())

            waveform_pixmap = self.waveform_plot.grab()
            fft_pixmap = self.fft_plot.grab()
            plot_top = top_margin + 78
            plot_height = 280
            half_width = (content_width - 20) // 2
            painter.drawPixmap(left_margin, plot_top, half_width, plot_height, waveform_pixmap)
            painter.drawPixmap(left_margin + half_width + 20, plot_top, half_width, plot_height, fft_pixmap)

            painter.setFont(QFont("Consolas", 8))
            metrics_text = self.metrics_df.round(6).to_string(index=False)
            crosstalk_text = self.crosstalk_df.round(6).to_string(index=False) if not self.crosstalk_df.empty else "No crosstalk result available"
            text_top = plot_top + plot_height + 25
            painter.drawText(left_margin, text_top, "Metrics")
            painter.drawText(
                QRect(left_margin, text_top + 16, content_width // 2 - 10, page_rect.height() - text_top - 40),
                int(Qt.TextFlag.TextExpandTabs | Qt.TextFlag.TextWordWrap),
                metrics_text,
            )
            painter.drawText(left_margin + content_width // 2, text_top, "Crosstalk")
            painter.drawText(
                QRect(left_margin + content_width // 2, text_top + 16, content_width // 2 - 10, page_rect.height() - text_top - 40),
                int(Qt.TextFlag.TextExpandTabs | Qt.TextFlag.TextWordWrap),
                crosstalk_text,
            )
        finally:
            painter.end()

        QMessageBox.information(self, "Export", f"Saved report to:\n{pdf_path}")

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
    if "--live" in sys.argv:
        from live_gui import LiveRecorderWindow

        window = LiveRecorderWindow()
    else:
        window = ScientificSignalAnalyzerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()