"""Live recorder window for the ESP32 + dual ADS1299 board.

This is the second window of the application: it talks to the board over
serial, replays the saved hardware settings on every connect, streams the data
into a per-channel time series and FFT view, shows the accelerometer with a
movement alarm, records OpenBCI-compatible CSV files with named sections, and
offers a serial monitor for the non-data messages of the firmware.

The analysis part lives in ``gui_main.py`` and is unchanged; recordings made
here can be opened there directly.
"""

from __future__ import annotations

import queue
import re
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import pyqtgraph as pg
from scipy import signal as scipy_signal
from PyQt6.QtCore import QEvent, QRect, QSignalBlocker, QSize, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - only hit without pyserial installed
    serial = None
    list_ports = None

from fft_analysis import WINDOW_MAP
from report_export import (
    REFERENCE_MODE_CODES,
    REFERENCE_MODE_SHORT,
    REFERENCE_MODE_SHORT_LABELS,
    MeasurementInfo,
    MeasurementNameDialog,
    install_wheel_guard,
)
from utg_control import (
    DEFAULT_SWEEP_FREQUENCIES_HZ,
    MIN_AMPLITUDE_VRMS,
    OUTPUT_DIFFERENTIAL,
    OUTPUT_OFF,
    OUTPUT_SINGLE,
    OUTPUT_ZERO,
    UTG932E,
    UTGError,
    GeneratorOutput,
    SweepDrive,
    SweepPlan,
    SweepSettings,
    SweepStep,
    dwell_seconds,
    list_resources as list_visa_resources,
    parse_frequency_list,
    per_channel_amplitude_vrms,
    selftest as run_generator_selftest,
)
from bluetooth_benchmark import (
    PAYLOAD_CHOICES,
    RATE_CHOICES,
    SWEEP_STEPS,
    BenchmarkReceiver,
    BenchmarkResult,
    is_sustainable,
    summarize_sweep,
)
from live_protocol import (
    ADS_SPI_CLOCK_HZ,
    ADS_SPI_CLOCK_MAX_HZ,
    BLUETOOTH_POWER_CHOICES,
    BOARD_CHANNEL_CHOICES,
    DECIMATION_CHOICES,
    DemoBoardPort,
    GAIN_LABELS,
    HardwareSettings,
    INPUT_TYPE_LABELS,
    MixedStreamParser,
    OpenBCIRecording,
    PacketDecoder,
    REFERENCE_MODE_DIFFERENTIAL,
    REFERENCE_MODE_SRB1,
    REFERENCE_MODE_SRB2,
    SPP_WRITE_CHOICES,
    SPI_CLOCK_CHOICES,
    RingBuffer,
    Annotation,
    SAMPLE_RATES,
    SAMPLE_RATE_BY_CODE,
    SettingsStore,
    VREF_VOLTS,
    build_chip_reference_command,
    build_decimation_command,
    build_repair_command,
    build_spi_clock_command,
    boxcar_corner_hz,
    boxcar_null_hz,
    channel_register_value,
    channels_per_device,
    clamp_decimation,
    clamp_spi_clock,
    max_sample_rate_from_spi,
    max_sustained_packet_rate,
    noise_bandwidth_hz,
    parse_repair_report,
    rate_fits_link,
    save_annotation_sidecar,
    sinc3_corner_hz,
    spi_frame_microseconds,
    StreamingFilter,
    TextLine,
    ChannelSetting,
    build_channel_command,
    build_sample_rate_command,
    compare_configuration,
    expected_device_registers,
    format_duration,
    parse_configuration_links,
    parse_configuration_report,
    physical_channel_map,
    real_channel_indices,
    sanitize_filename,
)

TRACE_SCALE_CHOICES = (25.0, 50.0, 100.0, 200.0, 400.0, 1000.0, 2000.0, 5000.0, 10000.0)
WINDOW_CHOICES = (1.0, 2.0, 5.0, 10.0, 20.0, 30.0)
FFT_POINTS_CHOICES = (256, 512, 1024, 2048, 4096, 8192, 16384)
SMOOTHING_CHOICES = (
    ("Off", 0.0),
    ("0.5", 0.5),
    ("0.75", 0.75),
    ("0.9", 0.9),
    ("0.95", 0.95),
    ("0.98", 0.98),
    ("0.99", 0.99),
    ("0.999", 0.999),
)
SPECTRUM_SCALE_CHOICES = (
    ("Auto", 0.0),
    ("0.01 uVrms", 0.01),
    ("0.1 uVrms", 0.1),
    ("1 uVrms", 1.0),
    ("10 uVrms", 10.0),
    ("100 uVrms", 100.0),
    ("1000 uVrms", 1000.0),
)
FFT_TRACE_PEN_WIDTH = 2.0
FFT_LOG_FLOOR_UV = 1.0
FFT_END_FREQ_CHOICES = (
    ("Auto", 0.0),
    ("25 Hz", 25.0),
    ("50 Hz", 50.0),
    ("100 Hz", 100.0),
    ("120 Hz", 120.0),
    ("150 Hz", 150.0),
    ("200 Hz", 200.0),
    ("250 Hz", 250.0),
)
HIGH_PASS_CHOICES = (
    (0.0, "Off"),
    (0.1, "0.1 Hz"),
    (0.3, "0.3 Hz"),
    (0.5, "0.5 Hz"),
    (1.0, "1 Hz"),
    (2.0, "2 Hz"),
    (5.0, "5 Hz"),
    (10.0, "10 Hz"),
    (20.0, "20 Hz"),
    (30.0, "30 Hz"),
    (40.0, "40 Hz"),
)
LOW_PASS_CHOICES = (
    (0.0, "Off"),
    (5.0, "5 Hz"),
    (10.0, "10 Hz"),
    (20.0, "20 Hz"),
    (30.0, "30 Hz"),
    (40.0, "40 Hz"),
    (50.0, "50 Hz"),
    (60.0, "60 Hz"),
    (80.0, "80 Hz"),
    (100.0, "100 Hz"),
    (150.0, "150 Hz"),
    (200.0, "200 Hz"),
)
TRACK_GUTTER_WIDTH = 72
BUFFER_SECONDS = 90.0
DEFAULT_BAUD = 115200
# A Bluetooth COM port can block for seconds while the driver talks to the radio,
# and Windows can block indefinitely on a stale pairing. The watchdog reports that
# instead of leaving the window looking idle, and the retries give a flaky pairing
# a chance to come up.
OPEN_WATCHDOG_MS = 12000
MAX_CONNECT_ATTEMPTS = 4
RECONNECT_DELAY_MS = 2000
# Bluetooth SPP is not a UART: the "baud" of a Bluetooth COM port is accepted by
# the driver and then ignored, because the bytes travel over RFCOMM at the radio's
# own rate. Any budget computed from the baud combo is therefore wrong for a
# Bluetooth port, and using it would throttle the sample rate for a limit that does
# not exist. This is the figure the budget uses instead: deliberately conservative
# for ESP32 classic SPP (measured rates are usually higher), and about twice what
# 12 channels at 500 SPS needs.
BLUETOOTH_SPP_BYTES_PER_SECOND = 32 * 1024
UI_INTERVAL_MS = 33
FFT_INTERVAL_TICKS = 4
VERIFY_TIMEOUT_MS = 4000
REPAIR_TIMEOUT_MS = 6000
# Corner of the running average shown as "DC" under every channel badge.
DC_CUTOFF_HZ = 0.1
# How long a channel may report exactly zero before it is called out.
STALL_SECONDS = 2.0
# Fraction of the ADC full scale above which an input counts as saturated, and
# how long it has to stay there before the row is flagged.
RAIL_FRACTION = 0.98
RAIL_SECONDS = 0.5
# Port tried on startup when nothing was used before.
AUTOCONNECT_FALLBACK_PORT = "COM5"
# How often to look for the board while auto-connect is on and nothing is open.
AUTOCONNECT_RETRY_MS = 2500
DEMO_PORT = "__DEMO__"
STREAM_HEALTH_PATTERN = re.compile(r"missed=(\d+)\s+dropped=(\d+)\s+corrupt=(\d+)")
BT_HEALTH_PATTERN = re.compile(r"^#BT\s+(.*)$")
BT_FIELD_PATTERN = re.compile(r"(\w+)=([^\s]+)")
BT_LOG_PATTERN = re.compile(r"^#BTLOG\s+(.*)$")
DX_PATTERN = re.compile(r"^#DX\s+(.*)$")
GAP_PATTERN = re.compile(r"^\[GAP\]\s+(.*)$")


def channel_label(index: int) -> str:
    return f"CH {index + 1}"


def channel_pen(index: int, width: float = 1.2) -> QPen:
    color = pg.intColor(index, hues=16, values=1, maxValue=210)
    return pg.mkPen(color, width=width)


def _shrinkable(label: QLabel) -> QLabel:
    """Let a wrapping label shrink below its text width.

    Without this a long line ("SPI link (2 MHz, ...) supports up to 4166 SPS")
    makes the whole side panel demand more width than the dock has, which pushes
    the buttons out of view.
    """

    label.setWordWrap(True)
    label.setMinimumWidth(0)
    label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    return label


def port_label(info) -> str:
    """Serial port label, flagging Bluetooth links so they are recognisable."""

    description = (info.description or "").strip()
    label = f"{info.device}  {description}" if description else info.device
    if port_is_bluetooth(info):
        # Windows keeps an incoming and an outgoing Bluetooth port and only one
        # of them carries data, so say what to do rather than what it is.
        label = f"{label}  (pair SZD_EEG; try the other BT port if silent)"
    return label


def port_is_bluetooth(info) -> bool:
    """True for a Windows Bluetooth SPP virtual COM port.

    Windows names these "Standard Serial over Bluetooth link" and the hardware id
    is a ``BTHENUM\\...`` path, which is the reliable discriminator: a plain USB
    bridge never reports either.
    """

    text = f"{getattr(info, 'description', '') or ''} {getattr(info, 'hwid', '') or ''}".lower()
    return "bluetooth" in text or "bthenum" in text


class PortScanner(QThread):
    """Enumerate serial ports off the GUI thread.

    ``list_ports.comports()`` asks Windows about every device, and with a
    Bluetooth COM port present (especially one whose link is down) that call can
    block for many seconds. On the GUI thread that is a frozen window with no CPU
    use, which is exactly how it behaves when a paired Bluetooth board misbehaves.
    """

    portsReady = pyqtSignal(object)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._request = threading.Event()
        self._stop = threading.Event()

    def request_scan(self) -> None:
        self._request.set()

    def stop(self) -> None:
        self._stop.set()
        self._request.set()
        self.wait(2000)

    def run(self) -> None:  # noqa: D401 - QThread entry point
        while not self._stop.is_set():
            # Idle until somebody asks: a scan is only worth doing on demand and
            # while auto-connect is hunting for the board.
            if not self._request.wait(0.25):
                continue
            self._request.clear()
            if self._stop.is_set():
                return
            if list_ports is None:
                self.portsReady.emit([])
                continue
            try:
                found = sorted(list_ports.comports(), key=lambda port: port.device)
                ports = [(port_label(info), info.device, port_is_bluetooth(info)) for info in found]
            except Exception:  # noqa: BLE001 - a driver fault must not kill the scan
                ports = []
            if not self._stop.is_set():
                self.portsReady.emit(ports)


class SerialWorker(QThread):
    """Owns the serial port and turns the byte stream into data and text.

    The transport may be passed in already open, or as a zero-argument callable
    that opens it. A callable is opened here, on this thread: opening a Bluetooth
    COM port takes seconds while the driver talks to the radio, and on the GUI
    thread that is another freeze.
    """

    batchReady = pyqtSignal(object)
    textReady = pyqtSignal(object)
    failed = pyqtSignal(str)
    opened = pyqtSignal()

    def __init__(self, transport, board_channels: int, gains: Sequence[float], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._transport = transport
        self._decoder = PacketDecoder(board_channels)
        self._decoder.set_gains(gains)
        self._parser = MixedStreamParser()
        self._commands: "queue.Queue[bytes]" = queue.Queue()
        self._stop_event = threading.Event()
        self._gain_lock = threading.Lock()
        # Optional raw-byte tap: a callable that consumes benchmark frames and
        # returns whatever was not part of one, so the normal parser still sees
        # text replies and packet traffic during a throughput test.
        self._benchmark_sink = None
        self._sink_lock = threading.Lock()

    def set_benchmark_sink(self, sink) -> None:
        with self._sink_lock:
            self._benchmark_sink = sink
        # Restart the text/packet parser so a half-consumed frame cannot leave it
        # hunting for a 0xFF that will never come.
        self._parser = MixedStreamParser()

    def send(self, payload: bytes) -> None:
        self._commands.put(payload)

    def set_gains(self, gains: Sequence[float]) -> None:
        with self._gain_lock:
            self._decoder.set_gains(gains)

    def stop(self) -> None:
        self._stop_event.set()
        try:
            if self._transport is not None:
                self._transport.close()
        except Exception:  # noqa: BLE001 - closing a dead port must not raise
            pass
        # A Bluetooth write can take up to its write timeout to fail, so allow a
        # moment longer than that instead of blocking the caller for seconds.
        self.wait(1500)

    def _open_transport(self) -> bool:
        if not callable(self._transport):
            return self._transport is not None
        try:
            self._transport = self._transport()
            self.opened.emit()
            return True
        except Exception as exc:  # noqa: BLE001 - surfaced to the user
            self._transport = None
            self.failed.emit(f"Could not open the port: {exc}")
            return False

    def run(self) -> None:  # noqa: D401 - QThread entry point
        samples: list[np.ndarray] = []
        aux_values: list[np.ndarray] = []
        texts: list[TextLine] = []
        last_emit = time.monotonic()
        try:
            if not self._open_transport():
                return
            while not self._stop_event.is_set():
                while True:
                    try:
                        payload = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._transport.write(payload)
                    except Exception as exc:  # noqa: BLE001
                        self.failed.emit(f"Serial write failed: {exc}")
                        return

                try:
                    waiting = int(getattr(self._transport, "in_waiting", 0) or 0)
                    chunk = self._transport.read(waiting if waiting > 0 else 1)
                except Exception as exc:  # noqa: BLE001
                    self.failed.emit(f"Serial read failed: {exc}")
                    return

                if chunk:
                    with self._sink_lock:
                        sink = self._benchmark_sink
                    if sink is not None:
                        try:
                            chunk = sink(chunk)
                        except Exception:  # noqa: BLE001 - a bad sink must not kill the link
                            chunk = b""
                    if chunk:
                        for event in self._parser.feed(chunk):
                            if isinstance(event, TextLine):
                                texts.append(event)
                                continue
                            with self._gain_lock:
                                block = self._decoder.push(event)
                                aux = self._decoder.last_aux.copy() if block is not None else None
                            if block is not None:
                                samples.append(block)
                                aux_values.append(aux if aux is not None else np.zeros(3))

                now = time.monotonic()
                if (samples or texts) and (now - last_emit) >= 0.02:
                    if samples:
                        self.batchReady.emit(
                            {
                                "samples": np.vstack(samples),
                                "aux": np.vstack(aux_values),
                                "packets": self._decoder.packet_count,
                                "loss": self._decoder.packet_loss,
                                "stray": self._parser.stray_bytes,
                            }
                        )
                        samples.clear()
                        aux_values.clear()
                    if texts:
                        self.textReady.emit(texts[:])
                        texts.clear()
                    last_emit = now
        except Exception as exc:  # noqa: BLE001 - report and let the GUI recover
            self.failed.emit(str(exc))
        finally:
            try:
                if self._transport is not None:
                    self._transport.close()
            except Exception:  # noqa: BLE001
                pass


class ReferenceChannelPanel(QWidget):
    """Dedicated view for the channel that measures the applied stimulus.

    A reference input wired across the generator sits at hundreds of millivolts to
    volts while the channels under test measure microvolts. Sharing one plot makes
    the interesting traces unreadable, so the reference channel gets its own time
    series and its own FFT with its own vertical scale, is excluded from the main
    views, and is forced to PGA x1 so a large stimulus cannot clip the ADC.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(250)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._label = ""
        self._sample_rate = 250.0
        self._waveform: list[float] = []
        self._fft_scale_uv = 1_000_000.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        header = QHBoxLayout()
        self.title_label = QLabel("Reference channel")
        self.title_label.setStyleSheet("font-weight: 600;")
        header.addWidget(self.title_label)
        header.addStretch(1)
        self.readout_label = QLabel("")
        self.readout_label.setStyleSheet("font-family: Consolas; font-size: 11px;")
        header.addWidget(self.readout_label)
        layout.addLayout(header)

        self.waveform_plot = pg.PlotWidget()
        self.waveform_plot.setLabel("bottom", "Time", units="s")
        self.waveform_plot.setLabel("left", "mV")
        self.waveform_plot.showGrid(x=True, y=True, alpha=0.2)
        self.waveform_plot.setMouseEnabled(x=False, y=False)
        self.waveform_curve = self.waveform_plot.plot(pen=pg.mkPen((16, 120, 90), width=1.4))
        layout.addWidget(self.waveform_plot, 3)

        self.fft_plot = pg.PlotWidget()
        self.fft_plot.setLabel("bottom", "Frequency", units="Hz")
        self.fft_plot.setLabel("left", "mVrms")
        self.fft_plot.showGrid(x=True, y=True, alpha=0.2)
        self.fft_plot.setMouseEnabled(x=False, y=False)
        self.fft_curve = self.fft_plot.plot(pen=pg.mkPen((30, 90, 160), width=1.4))
        layout.addWidget(self.fft_plot, 2)

    def set_sample_rate(self, sample_rate: float) -> None:
        self._sample_rate = max(float(sample_rate), 1.0)

    def set_channel(self, label: str) -> None:
        self._label = label
        self.title_label.setText(f"Reference channel: {label}" if label else "Reference channel: none")
        self._waveform.clear()
        self.waveform_curve.setData([], [])
        self.fft_curve.setData([], [])
        self.readout_label.setText("")

    def set_fft_scale(self, scale_uv: float) -> None:
        self._fft_scale_uv = max(float(scale_uv), 1e-6)

    def update_trace(self, values: np.ndarray, sample_rate: float) -> None:
        """Redraw from a block of microvolt samples of the reference channel."""

        block = np.asarray(values, dtype=np.float64).reshape(-1)
        if block.size < 4:
            return
        if not np.all(np.isfinite(block)):
            block = np.nan_to_num(block)
        # Milliovolts on the axis: the reference is 3-4 orders of magnitude larger
        # than an EEG channel, so microvolts would need six-digit tick labels.
        millivolts = block * 1e-3
        times = np.arange(block.size, dtype=np.float64) / max(sample_rate, 1.0)
        self.waveform_curve.setData(times, millivolts)
        span = float(np.max(np.abs(millivolts))) if millivolts.size else 0.0
        if span > 0:
            self.waveform_plot.setYRange(-span * 1.15, span * 1.15, padding=0.0)

        rms_uv = float(np.sqrt(np.mean(np.square(block - np.mean(block)))))
        peak = float(np.max(np.abs(block - np.mean(block))))
        self.readout_label.setText(f"RMS {rms_uv / 1000:.4g} mV  peak {peak / 1000:.4g} mV")

        length = min(block.size, 1024)
        window = np.hanning(length)
        spectrum = np.fft.rfft((block[-length:] - np.mean(block[-length:])) * window)
        # Amplitude-corrected RMS: |X| / (N * coherent gain of the window).
        amplitude = np.abs(spectrum) / (length * max(float(np.mean(window)), 1e-9)) / np.sqrt(2.0)
        frequencies = np.fft.rfftfreq(length, d=1.0 / max(sample_rate, 1.0))
        self.fft_curve.setData(frequencies[1:], amplitude[1:] * 1e-3)
        self.fft_plot.setXRange(0.0, min(self._sample_rate / 2.0, 130.0), padding=0.0)


class AccelerometerPanel(QWidget):
    """Three-axis readout with a movement alarm around the panel border."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(210)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._accel_g = np.zeros(3)
        self._baseline: Optional[np.ndarray] = None
        self._delta = 0.0
        self._peak = 0.0
        self._moving = False
        self._flash = False
        self._events = 0
        self._full_scale_g = 2.0
        self._threshold_g = 0.08
        self._baseline_seconds = 2.0
        self._last_update = time.monotonic()
        self._note = ""

    def configure(self, full_scale_g: float, threshold_g: float, baseline_seconds: float) -> None:
        self._full_scale_g = max(float(full_scale_g), 0.1)
        self._threshold_g = max(float(threshold_g), 1e-4)
        self._baseline_seconds = max(float(baseline_seconds), 0.2)
        self.update()

    def reset(self) -> None:
        self._baseline = None
        self._delta = 0.0
        self._peak = 0.0
        self._moving = False
        self._flash = False
        self._events = 0
        self._last_update = time.monotonic()
        self.update()

    def set_note(self, note: str) -> None:
        if note != self._note:
            self._note = note
            self.update()

    def set_flash(self, phase: bool) -> None:
        if phase != self._flash:
            self._flash = phase
            self.update()

    @property
    def is_moving(self) -> bool:
        return self._moving

    @property
    def movement_events(self) -> int:
        return self._events

    def set_reading(self, accel_g: np.ndarray) -> None:
        now = time.monotonic()
        dt = max(now - self._last_update, 1e-3)
        self._last_update = now
        values = np.asarray(accel_g, dtype=float).reshape(3)
        if self._baseline is None:
            self._baseline = values.copy()
        alpha = min(max(dt / self._baseline_seconds, 0.001), 1.0)
        self._baseline = self._baseline + alpha * (values - self._baseline)
        self._delta = float(np.linalg.norm(values - self._baseline))
        self._peak = max(self._peak, self._delta)
        if self._delta > self._threshold_g:
            if not self._moving:
                self._events += 1
            self._moving = True
        else:
            self._moving = False
        self._accel_g = values
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(2, 2, -2, -2)

        if self._moving:
            outline = QColor("#ef4444") if self._flash else QColor("#991b1b")
            fill = QColor(254, 226, 226, 90 if self._flash else 40)
        else:
            outline = QColor("#94a3b8")
            fill = QColor(248, 250, 252)

        painter.setPen(QPen(outline, 3.0 if self._moving else 1.6))
        painter.setBrush(fill)
        painter.drawRoundedRect(rect, 10, 10)

        title_font = QFont(self.font())
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.setPen(QColor("#0f172a"))
        title_rect = QRect(rect.left() + 12, rect.top() + 8, rect.width() - 24, 20)
        painter.drawText(title_rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, "Accelerometer")
        painter.setPen(QColor("#dc2626") if self._moving else QColor("#15803d"))
        status = "MOVING" if self._moving else "STILL"
        painter.drawText(title_rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, status)

        painter.setFont(self.font())
        body_top = rect.top() + 36
        body_bottom = rect.bottom() - 28
        row_height = max((body_bottom - body_top) / 3.0, 24.0)
        label_width = 22
        value_width = 84
        track_left = rect.left() + 12 + label_width
        track_width = max(rect.width() - 24 - label_width - value_width, 20)
        center_x = track_left + track_width / 2.0
        half = track_width / 2.0

        for axis, name in enumerate(("X", "Y", "Z")):
            value = float(self._accel_g[axis])
            top = int(body_top + axis * row_height)
            row = QRect(rect.left() + 12, top, rect.width() - 24, int(row_height))
            painter.setPen(QColor("#334155"))
            painter.drawText(QRect(row.left(), top, label_width, row.height()), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, name)

            bar_top = int(top + row.height() / 2 - 5)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(226, 232, 240))
            painter.drawRoundedRect(QRect(int(track_left), bar_top, int(track_width), 10), 4, 4)
            fraction = float(np.clip(value / self._full_scale_g, -1.0, 1.0))
            length = fraction * half
            bar_x = int(center_x) if length >= 0 else int(center_x + length)
            painter.setBrush(QColor("#2563eb") if axis < 2 else QColor("#0ea5e9"))
            painter.drawRoundedRect(QRect(bar_x, bar_top, int(abs(length)), 10), 4, 4)
            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRect(row.left() + row.width() - value_width, top, value_width, row.height()),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                f"{value:+.3f} g",
            )

        footer_font = QFont(self.font())
        footer_font.setPointSizeF(max(footer_font.pointSizeF() - 0.5, 7.0))
        painter.setFont(footer_font)
        footer_rect = rect.adjusted(12, 0, -12, -6)
        painter.setPen(QColor("#475569"))
        painter.drawText(
            footer_rect,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom,
            f"motion {self._delta:.3f} g | peak {self._peak:.3f} g | events {self._events}",
        )
        if self._note:
            painter.setPen(QColor("#b45309"))
            painter.drawText(footer_rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom, self._note)
        painter.end()

class TrackGutter(QWidget):
    """Left gutter of one trace row: channel badge, DC value and amplitude labels."""

    def __init__(self, index: int, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.index = index
        self._scale = 100.0
        self._dc_uv: Optional[float] = None
        self._state = "ok"
        self.setFixedWidth(TRACK_GUTTER_WIDTH)

    def set_scale(self, scale: float) -> None:
        if abs(float(scale) - self._scale) > 1e-9:
            self._scale = float(scale)
            self.update()

    def set_dc(self, dc_uv: Optional[float]) -> None:
        if dc_uv is None:
            changed = self._dc_uv is not None
            self._dc_uv = None
        else:
            changed = self._dc_uv is None or abs(dc_uv - self._dc_uv) >= 0.005
            self._dc_uv = float(dc_uv)
        if changed:
            self.update()

    def set_state(self, state: str) -> None:
        if state != self._state:
            self._state = state
            self.update()

    @property
    def state(self) -> str:
        return self._state

    @staticmethod
    def _format_dc(dc_uv: float) -> tuple[str, str]:
        """Value and the unit that goes underneath it."""

        magnitude = abs(dc_uv)
        if magnitude >= 1000.0:
            return f"{dc_uv / 1000.0:+.3f}", "mV"
        if magnitude >= 100.0:
            return f"{dc_uv:+.1f}", "uV"
        return f"{dc_uv:+.2f}", "uV"

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        width = self.width()
        height = self.height()

        badge = 18
        badge_y = 3
        if self._state == "stall":
            painter.setPen(QPen(QColor("#dc2626"), 2.0))
        elif self._state == "rail":
            painter.setPen(QPen(QColor("#f59e0b"), 2.0))
        else:
            painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(pg.intColor(self.index, hues=16, values=1, maxValue=210))
        painter.drawEllipse(4, badge_y, badge, badge)
        badge_font = QFont(self.font())
        badge_font.setBold(True)
        badge_font.setPointSizeF(8.0)
        painter.setFont(badge_font)
        painter.setPen(QColor("#ffffff"))
        painter.drawText(QRect(4, badge_y, badge, badge), Qt.AlignmentFlag.AlignCenter, str(self.index + 1))

        if self._state != "ok":
            painter.setPen(QColor("#dc2626") if self._state == "stall" else QColor("#b45309"))
            painter.drawText(
                QRect(24, badge_y, width - 26, badge),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                "!" if self._state == "stall" else "~",
            )

        # The DC number sits directly under the badge with its unit below it, so
        # the row never has to give up space to an explanatory word. The state is
        # carried by the colour and by the unit line, never by hiding the value.
        if self._dc_uv is None:
            value_text, unit_text = "-", "uV"
        else:
            value_text, unit_text = self._format_dc(self._dc_uv)
        if self._state == "stall":
            unit_text = "no data"
        elif self._state == "rail":
            unit_text = "saturated"

        value_font = QFont(self.font())
        value_font.setPointSizeF(7.5)
        value_font.setBold(self._state != "ok")
        painter.setFont(value_font)
        if self._state == "stall":
            painter.setPen(QColor("#dc2626"))
        elif self._state == "rail":
            painter.setPen(QColor("#b45309"))
        else:
            painter.setPen(QColor("#0f766e"))
        value_top = badge_y + badge + 1
        painter.drawText(
            QRect(2, value_top, width - 4, 13),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            value_text,
        )

        if height > value_top + 22:
            unit_font = QFont(self.font())
            unit_font.setPointSizeF(6.0)
            painter.setFont(unit_font)
            painter.setPen(QColor("#94a3b8") if self._state == "ok" else QColor("#dc2626"))
            painter.drawText(
                QRect(2, value_top + 12, width - 4, 10),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                unit_text,
            )

        label_font = QFont(self.font())
        label_font.setPointSizeF(6.5)
        painter.setFont(label_font)
        painter.setPen(QColor("#64748b"))
        top_margin = 7
        bottom_margin = 7
        usable = max(height - top_margin - bottom_margin, 1)
        values = (
            (0.0, self._scale),
            (0.25, self._scale * 0.5),
            (0.5, 0.0),
            (0.75, -self._scale * 0.5),
            (1.0, -self._scale),
        )
        for fraction, value in values:
            y = int(top_margin + fraction * usable)
            text = "0" if abs(value) < 1e-9 else f"{value:+.0f}"
            painter.drawText(
                QRect(40, y - 6, width - 44, 12),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                text,
            )
        painter.end()


class ChannelTrack(QWidget):
    """One channel row: fixed +/- vertical range, grid at 0 / half / full scale."""

    def __init__(self, index: int, label: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.index = index
        self.label = label
        self._scale = 100.0

        self.gutter = TrackGutter(index)
        self.plot = pg.PlotWidget()
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.hideAxis("left")
        bottom_axis = self.plot.getAxis("bottom")
        bottom_axis.setStyle(showValues=False)
        bottom_axis.setPen(pg.mkPen(None))
        bottom_axis.setHeight(0)
        self.plot.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.plot.setBackground("w")

        self.curve = self.plot.getPlotItem().plot()
        self.curve.setPen(channel_pen(index))
        self.curve.setClipToView(True)
        self.curve.setDownsampling(auto=True, method="peak")

        self._lines: list[pg.InfiniteLine] = []
        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            if fraction == 0.0:
                pen = pg.mkPen("#94a3b8", width=1.0)
            elif abs(fraction) == 1.0:
                pen = pg.mkPen("#cbd5e1", width=1.0)
            else:
                pen = pg.mkPen("#e2e8f0", width=1.0, style=Qt.PenStyle.DashLine)
            line = pg.InfiniteLine(angle=0, movable=False, pen=pen)
            line.setValue(fraction * self._scale)
            self.plot.addItem(line)
            self._lines.append(line)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.gutter)
        layout.addWidget(self.plot, 1)
        self.set_scale(self._scale)

    def set_scale(self, scale: float) -> None:
        self._scale = max(float(scale), 1e-3)
        self.gutter.set_scale(self._scale)
        for fraction, line in zip((-1.0, -0.5, 0.0, 0.5, 1.0), self._lines):
            line.setValue(fraction * self._scale)
        self.plot.setYRange(-self._scale, self._scale, padding=0.0)

    def set_data(self, times: np.ndarray, values: np.ndarray) -> None:
        self.curve.setData(times, values)

    def set_dc(self, dc_uv: Optional[float]) -> None:
        self.gutter.set_dc(dc_uv)

    def set_state(self, state: str) -> None:
        self.gutter.set_state(state)

    def clear(self) -> None:
        self.curve.setData([], [])
        self.gutter.set_dc(None)
        self.gutter.set_state("ok")


class TimeSeriesPanel(QWidget):
    """Scrollable stack of per-channel traces sharing one time axis."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.labels: list[str] = []
        self.tracks: dict[int, ChannelTrack] = {}
        self._visible: list[int] = []

        self.scale_combo = QComboBox()
        for value in TRACE_SCALE_CHOICES:
            self.scale_combo.addItem(f"{value:g} uV", value)
        self.scale_combo.setCurrentIndex(TRACE_SCALE_CHOICES.index(100.0))

        self.rows_combo = QComboBox()
        for rows in (4, 6, 8, 10, 12):
            self.rows_combo.addItem(f"{rows} rows", rows)
        self.rows_combo.setCurrentIndex(2)
        self.rows_combo.setToolTip(
            "How many channel traces fit on screen at once. Scroll the plot area for the rest; "
            "each row keeps the same height so the vertical scale stays comparable."
        )

        self.window_combo = QComboBox()
        for value in WINDOW_CHOICES:
            self.window_combo.addItem(f"{value:g} s", value)
        self.window_combo.setCurrentIndex(WINDOW_CHOICES.index(5.0))

        self.center_check = QCheckBox("Center")
        self.center_check.setChecked(True)
        self.center_check.setToolTip("Remove the mean of the visible window from each trace (display only)")
        self.pause_check = QCheckBox("Pause")

        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(4, 0, 4, 0)
        toolbar.setSpacing(6)
        self.scale_combo.setToolTip("Vertical span of every channel row (+/- this value)")
        toolbar.addWidget(QLabel("Scale"))
        toolbar.addWidget(self.scale_combo)
        rows_label = QLabel("Rows")
        rows_label.setToolTip("How many channel traces fit on screen at once")
        toolbar.addWidget(rows_label)
        toolbar.addWidget(self.rows_combo)
        toolbar.addWidget(QLabel("Window"))
        toolbar.addWidget(self.window_combo)
        toolbar.addWidget(self.center_check)
        toolbar.addWidget(self.pause_check)
        toolbar.addStretch(1)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.container = QWidget()
        self.track_layout = QVBoxLayout(self.container)
        self.track_layout.setContentsMargins(0, 0, 0, 0)
        self.track_layout.setSpacing(0)
        self.scroll.setWidget(self.container)

        self.axis_plot = pg.PlotWidget()
        self.axis_plot.setMenuEnabled(False)
        self.axis_plot.setMouseEnabled(x=False, y=False)
        self.axis_plot.hideAxis("left")
        self.axis_plot.setFixedHeight(40)
        self.axis_plot.setBackground("w")
        self.axis_plot.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.axis_plot.getPlotItem().setContentsMargins(0, 0, 0, 0)
        self.axis_plot.setLabel("bottom", "Time", units="s")
        self.axis_plot.setXRange(0.0, 1.0, padding=0.0)
        self.axis_plot.setYRange(0.0, 1.0, padding=0.0)

        axis_row = QHBoxLayout()
        axis_row.setContentsMargins(0, 0, 0, 0)
        axis_row.setSpacing(0)
        self.axis_left_spacer = QWidget()
        self.axis_left_spacer.setFixedWidth(TRACK_GUTTER_WIDTH)
        self.axis_right_spacer = QWidget()
        self.axis_right_spacer.setFixedWidth(0)
        axis_row.addWidget(self.axis_left_spacer)
        axis_row.addWidget(self.axis_plot, 1)
        axis_row.addWidget(self.axis_right_spacer)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addLayout(toolbar)
        # Full-width row: the state banner must never be squeezed into the
        # toolbar where a long "input saturated: CH 1, CH 3, ..." would clip.
        self.state_banner = QLabel("")
        self.state_banner.setWordWrap(True)
        self.state_banner.setStyleSheet(
            "color: #b91c1c; font-weight: 600; font-size: 11px; background: #fef2f2; "
            "border: 1px solid #fecaca; padding: 2px 6px;"
        )
        self.state_banner.setVisible(False)
        layout.addWidget(self.state_banner)
        self._states: dict[int, str] = {}
        layout.addWidget(self.scroll, 1)
        layout.addLayout(axis_row)

        self.scale_combo.currentIndexChanged.connect(self._on_scale_changed)
        self.rows_combo.currentIndexChanged.connect(self._apply_row_height)

    def _on_scale_changed(self) -> None:
        for track in self.tracks.values():
            track.set_scale(self.scale_uv)

    @property
    def paused(self) -> bool:
        return self.pause_check.isChecked()

    @property
    def window_seconds(self) -> float:
        return float(self.window_combo.currentData())

    @property
    def scale_uv(self) -> float:
        return float(self.scale_combo.currentData())

    def apply_settings(self, view: dict) -> None:
        scale = float(view.get("vertical_scale_uv", 100.0))
        index = min(range(len(TRACE_SCALE_CHOICES)), key=lambda i: abs(TRACE_SCALE_CHOICES[i] - scale))
        self.scale_combo.setCurrentIndex(index)
        window = float(view.get("window_seconds", 5.0))
        window_index = min(range(len(WINDOW_CHOICES)), key=lambda i: abs(WINDOW_CHOICES[i] - window))
        self.window_combo.setCurrentIndex(window_index)
        rows = int(view.get("visible_rows", 8))
        row_index = self.rows_combo.findData(rows)
        if row_index >= 0:
            self.rows_combo.setCurrentIndex(row_index)
        self.center_check.setChecked(bool(view.get("center_channels", True)))

    def ensure_channels(self, labels: Sequence[str]) -> None:
        labels = list(labels)
        if labels == self.labels:
            return
        for track in self.tracks.values():
            self.track_layout.removeWidget(track)
            track.deleteLater()
        self.tracks = {}
        self.labels = labels
        self._visible = []
        for index, _ in enumerate(labels):
            self.tracks[index] = ChannelTrack(index, channel_label(index))

    def set_visible(self, flags: Sequence[bool]) -> None:
        visible = [index for index, flag in enumerate(flags) if flag and index in self.tracks]
        if visible == self._visible:
            return
        self._visible = visible
        for track in self.tracks.values():
            self.track_layout.removeWidget(track)
            track.hide()
        for index in self._visible:
            self.track_layout.addWidget(self.tracks[index])
            self.tracks[index].show()
        self._apply_row_height()

    def _apply_row_height(self) -> None:
        rows = int(self.rows_combo.currentData() or 8)
        count = max(len(self._visible), 1)
        rows = max(min(rows, count), 1)
        viewport_height = max(self.scroll.viewport().height(), 120)
        row_height = max(viewport_height // rows, 52)
        for index in self._visible:
            self.tracks[index].setFixedHeight(row_height)
        self.container.setMinimumHeight(row_height * count)
        # Keep the shared time axis aligned with the trace area: reserve the
        # channel gutter on the left and the scrollbar width on the right.
        self.axis_left_spacer.setFixedWidth(TRACK_GUTTER_WIDTH)
        needs_scroll = row_height * count > viewport_height
        reserve = self.scroll.verticalScrollBar().sizeHint().width() if needs_scroll else 0
        self.axis_right_spacer.setFixedWidth(reserve)

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        super().resizeEvent(event)
        self._apply_row_height()

    def clear(self) -> None:
        for track in self.tracks.values():
            track.clear()

    def render(self, block: Optional[np.ndarray], sample_rate: float, end_time: float, visible: Sequence[bool]) -> None:
        self.set_visible(visible)
        if block is None or block.shape[0] < 2 or sample_rate <= 0 or not self._visible:
            return
        count = min(block.shape[0], max(int(self.window_seconds * sample_rate), 8))
        data = block[-count:]
        if self.center_check.isChecked():
            data = data - data.mean(axis=0, keepdims=True)
        times = end_time - (count - np.arange(count)) / float(sample_rate)
        for index in self._visible:
            track = self.tracks.get(index)
            if track is None or index >= data.shape[1]:
                continue
            track.set_data(times, data[:, index])
        x0 = float(times[0])
        x1 = float(times[-1])
        for index in self._visible:
            self.tracks[index].plot.setXRange(x0, x1, padding=0.0)
        self.axis_plot.setXRange(x0, x1, padding=0.0)

    def set_dc(self, values: dict[int, Optional[float]]) -> None:
        """Show the sub-0.1 Hz level under each channel badge."""

        for index, track in self.tracks.items():
            track.set_dc(values.get(index))

    def set_channel_states(self, states: dict[int, str]) -> None:
        """``ok`` / ``stall`` / ``rail`` per channel, plus a banner that says so."""

        self._states = dict(states)
        for index, track in self.tracks.items():
            track.set_state(self._states.get(index, "ok"))
        self._update_state_banner()

    def _update_state_banner(self) -> None:
        stalled = sorted(index for index, state in self._states.items() if state == "stall")
        railed = sorted(index for index, state in self._states.items() if state == "rail")
        parts: list[str] = []
        if stalled:
            parts.append("no data (0 V): " + ", ".join(channel_label(index) for index in stalled))
        if railed:
            parts.append("input saturated: " + ", ".join(channel_label(index) for index in railed))
        if not parts:
            self.state_banner.setText("")
            self.state_banner.setVisible(False)
            return
        self.state_banner.setText("   |   ".join(parts))
        self.state_banner.setVisible(True)

class SpectrumPanel(QWidget):
    """Live RMS spectrum with FFT length, window, smoothing, scale and cursor."""

    AVERAGE_KEY = -1

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.labels: list[str] = []
        self.curves: dict[int, pg.PlotDataItem] = {}
        self._visible: list[int] = []
        self._smoothed: dict[int, np.ndarray] = {}
        self._signature: Optional[tuple[int, float, float]] = None
        self._bins: Optional[tuple[np.ndarray, dict[int, np.ndarray]]] = None
        self._data_max: Optional[float] = None
        self._channel_mode = "all"
        self._end_hz = 0.0
        self._nyquist_hz = 0.0
        self._grid_lines: list[pg.InfiniteLine] = []

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMenuEnabled(False)
        self.plot_widget.setMouseEnabled(x=False, y=False)
        self.plot_widget.setLabel("bottom", "Frequency", units="Hz")
        self.plot_widget.setLabel("left", "Amplitude", units="uVrms")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.getPlotItem().hideButtons()
        self.plot_widget.installEventFilter(self)

        plot_item = self.plot_widget.getPlotItem()
        self.nyquist_line = pg.InfiniteLine(
            angle=90, movable=False, pen=pg.mkPen("#94a3b8", width=1.0, style=Qt.PenStyle.DashLine)
        )
        self.nyquist_line.setZValue(-1)
        self.nyquist_line.hide()
        self.plot_widget.addItem(self.nyquist_line, ignoreBounds=True)

        self.average_curve = plot_item.plot(name="Average")
        self.average_curve.setPen(pg.mkPen("#0f172a", width=FFT_TRACE_PEN_WIDTH))
        self.average_curve.hide()

        self.cursor_v = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#f59e0b", width=1.0))
        self.cursor_h = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen("#f59e0b", width=1.0))
        self.cursor_v.hide()
        self.cursor_h.hide()
        self.plot_widget.addItem(self.cursor_v, ignoreBounds=True)
        self.plot_widget.addItem(self.cursor_h, ignoreBounds=True)
        self.plot_widget.scene().sigMouseMoved.connect(self._on_mouse_moved)

        self.readout_item = pg.TextItem("", anchor=(0.0, 0.0), color="#0f172a", fill=pg.mkBrush(255, 255, 255, 190))
        self.readout_item.setZValue(20)
        self.plot_widget.addItem(self.readout_item, ignoreBounds=True)

        self.points_combo = QComboBox()
        for value in FFT_POINTS_CHOICES:
            self.points_combo.addItem(f"{value}", value)
        self.points_combo.setCurrentIndex(FFT_POINTS_CHOICES.index(1024))

        self.window_combo = QComboBox()
        self.window_combo.addItems(list(WINDOW_MAP.keys()))
        self.window_combo.setCurrentText("Hann")
        self.window_combo.setMaximumWidth(120)

        self.smoothing_combo = QComboBox()
        for label, value in SMOOTHING_CHOICES:
            self.smoothing_combo.addItem(label, value)

        self.scale_combo = QComboBox()
        for label, value in SPECTRUM_SCALE_CHOICES:
            self.scale_combo.addItem(label, value)
        self.scale_combo.setToolTip("Upper end of the amplitude axis (Auto follows the strongest bin)")
        self.scale_combo.setMaximumWidth(132)

        self.log_check = QCheckBox("Log")
        self.log_check.setToolTip(
            "Logarithmic amplitude axis with one grid line per decade: 1 uVrms up to the highest "
            "decade of the selected scale (10^3 for 1000 uVrms, 10^2 for 100 uVrms)."
        )
        self.pause_check = QCheckBox("Pause")

        self.channel_combo = QComboBox()
        self.channel_combo.setToolTip("Draw every visible channel, a single channel, or the average of the visible ones")
        self._rebuild_channel_choices()

        self.end_combo = QComboBox()
        for label, value in FFT_END_FREQ_CHOICES:
            self.end_combo.addItem(label, value)
        self.end_combo.setToolTip(
            "Right end of the frequency axis. Auto follows the Nyquist limit of the current sample rate. "
            "When the axis is wider than Nyquist the bins stop at the dashed line."
        )

        first_row = QHBoxLayout()
        first_row.setContentsMargins(4, 0, 4, 0)
        first_row.setSpacing(4)
        heading = QLabel("FFT")
        heading.setStyleSheet("font-weight: 600;")
        first_row.addWidget(heading)
        first_row.addSpacing(6)
        first_row.addWidget(QLabel("Points"))
        first_row.addWidget(self.points_combo)
        first_row.addWidget(QLabel("Window"))
        first_row.addWidget(self.window_combo)
        first_row.addWidget(QLabel("Smooth"))
        first_row.addWidget(self.smoothing_combo)
        first_row.addWidget(self.pause_check)
        first_row.addStretch(1)

        second_row = QHBoxLayout()
        second_row.setContentsMargins(4, 0, 4, 0)
        second_row.setSpacing(4)
        second_row.addWidget(QLabel("Trace"))
        second_row.addWidget(self.channel_combo)
        second_row.addWidget(QLabel("Scale"))
        second_row.addWidget(self.scale_combo)
        second_row.addWidget(self.log_check)
        second_row.addWidget(QLabel("End"))
        second_row.addWidget(self.end_combo)
        second_row.addStretch(1)

        self.readout_label = QLabel("")
        self.readout_label.setStyleSheet("color: #475569; font-size: 11px;")
        self.readout_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addLayout(first_row)
        layout.addLayout(second_row)
        layout.addWidget(self.plot_widget, 1)
        layout.addWidget(self.readout_label)

        self.log_check.toggled.connect(self._refresh_amplitude_axis)
        self.scale_combo.currentIndexChanged.connect(self._refresh_amplitude_axis)
        self.end_combo.currentIndexChanged.connect(self._refresh_x_axis)

        self.points_combo.currentIndexChanged.connect(self._reset_smoothing)
        self.window_combo.currentIndexChanged.connect(self._reset_smoothing)
        self.smoothing_combo.currentIndexChanged.connect(self._reset_smoothing)

    # -- channel selection -------------------------------------------------

    def channel_mode(self) -> str:
        return str(self.channel_combo.currentData() or "all")

    def set_channel_mode(self, mode: str) -> None:
        self._channel_mode = str(mode or "all")
        position = self.channel_combo.findData(self._channel_mode)
        if position < 0:
            position = 0
        if position != self.channel_combo.currentIndex():
            self.channel_combo.setCurrentIndex(position)

    def _rebuild_channel_choices(self) -> None:
        blocker = QSignalBlocker(self.channel_combo)
        self.channel_combo.clear()
        self.channel_combo.addItem("Visible", "all")
        self.channel_combo.addItem("Avg", "average")
        for index, _ in enumerate(self.labels):
            self.channel_combo.addItem(channel_label(index), str(index))
        position = self.channel_combo.findData(self._channel_mode)
        self.channel_combo.setCurrentIndex(position if position >= 0 else 0)
        del blocker

    def _selected_channels(self) -> list[int]:
        mode = self.channel_mode()
        if mode in ("all", "average"):
            return [index for index in self._visible if index in self.curves]
        try:
            index = int(mode)
        except ValueError:
            return []
        return [index] if index in self.curves else []

    def end_frequency(self) -> float:
        return float(self.end_combo.currentData() or 0.0)

    def set_end_frequency(self, value: float) -> None:
        best = min(
            range(self.end_combo.count()),
            key=lambda position: abs(float(self.end_combo.itemData(position) or 0.0) - float(value)),
        )
        self.end_combo.setCurrentIndex(best)
        self._end_hz = 0.0

    # -- plot events -------------------------------------------------------

    def eventFilter(self, obj, event):  # noqa: N802 - Qt naming
        if obj is self.plot_widget and event.type() == QEvent.Type.Leave:
            self._set_cursor_visible(False)
        return super().eventFilter(obj, event)

    def _set_cursor_visible(self, visible: bool) -> None:
        self.cursor_v.setVisible(visible)
        self.cursor_h.setVisible(visible)
        if not visible:
            self.readout_item.setText("")
            self._update_peak_label()

    def _on_mouse_moved(self, position) -> None:
        if self._bins is None or self._bins[0].size == 0:
            return
        frequencies, amplitudes = self._bins
        view_box = self.plot_widget.getPlotItem().vb
        if not view_box.sceneBoundingRect().contains(position):
            self._set_cursor_visible(False)
            return
        point = view_box.mapSceneToView(position)
        if point.x() < 0.0 or point.x() > float(frequencies[-1]) + 1e-9:
            self._set_cursor_visible(False)
            return
        index = int(np.clip(np.searchsorted(frequencies, point.x()), 0, frequencies.size - 1))
        if index > 0 and abs(frequencies[index - 1] - point.x()) < abs(frequencies[index] - point.x()):
            index -= 1
        best_channel = None
        best_value = -1.0
        for channel, values in amplitudes.items():
            if index < values.size and float(values[index]) > best_value:
                best_value = float(values[index])
                best_channel = channel
        if best_channel is None:
            return
        y_value = float(np.log10(max(best_value, 1e-6))) if self.log_check.isChecked() else best_value
        self.cursor_v.setPos(float(frequencies[index]))
        self.cursor_h.setPos(y_value)
        self._set_cursor_visible(True)
        name = "avg" if best_channel == self.AVERAGE_KEY else f"CH {best_channel + 1}"
        self._set_readout(f"{float(frequencies[index]):.2f} Hz    {best_value:.3f} uVrms    {name}")

    def _update_peak_label(self) -> None:
        if self._bins is None or self._bins[0].size == 0:
            self.readout_label.setText("")
            return
        frequencies, amplitudes = self._bins
        best_index = None
        best_value = 0.0
        for values in amplitudes.values():
            if values.size == 0:
                continue
            index = int(np.argmax(values))
            if best_index is None or float(values[index]) > best_value:
                best_index = index
                best_value = float(values[index])
        if best_index is None:
            self.readout_label.setText("")
            return
        self.readout_label.setText(f"peak {float(frequencies[best_index]):.2f} Hz / {best_value:.2f} uVrms")

    def _reset_smoothing(self) -> None:
        self._smoothed.clear()
        self._signature = None
        self._bins = None

    # -- configuration -----------------------------------------------------

    def apply_settings(self, view: dict) -> None:
        points = int(view.get("fft_points", 1024))
        index = min(range(len(FFT_POINTS_CHOICES)), key=lambda i: abs(FFT_POINTS_CHOICES[i] - points))
        self.points_combo.setCurrentIndex(index)
        window_name = str(view.get("fft_window", "Hann"))
        if window_name in WINDOW_MAP:
            self.window_combo.setCurrentText(window_name)
        smoothing = float(view.get("fft_smoothing", 0.0))
        smooth_index = min(range(len(SMOOTHING_CHOICES)), key=lambda i: abs(SMOOTHING_CHOICES[i][1] - smoothing))
        self.smoothing_combo.setCurrentIndex(smooth_index)
        scale = float(view.get("fft_scale_mode", 0.0) or 0.0)
        scale_index = min(range(len(SPECTRUM_SCALE_CHOICES)), key=lambda i: abs(SPECTRUM_SCALE_CHOICES[i][1] - scale))
        self.scale_combo.setCurrentIndex(scale_index)
        self.log_check.setChecked(bool(view.get("fft_log_scale", False)))
        self.set_channel_mode(str(view.get("fft_channel", "all")))
        self.set_end_frequency(float(view.get("fft_end_hz", 0.0) or 0.0))
        self._apply_amplitude_axis()
        self._refresh_x_axis()

    def ensure_channels(self, labels: Sequence[str]) -> None:
        labels = list(labels)
        if labels == self.labels:
            return
        plot_item = self.plot_widget.getPlotItem()
        for curve in self.curves.values():
            plot_item.removeItem(curve)
        self.curves = {}
        self.labels = labels
        self._visible = []
        self._reset_smoothing()
        for index, _ in enumerate(labels):
            curve = plot_item.plot(name=channel_label(index))
            curve.setPen(channel_pen(index, width=FFT_TRACE_PEN_WIDTH))
            curve.hide()
            self.curves[index] = curve
        self._rebuild_channel_choices()
        self._end_hz = 0.0

    def set_visible(self, flags: Sequence[bool]) -> None:
        self._visible = [index for index, flag in enumerate(flags) if flag and index in self.curves]

    def clear(self) -> None:
        for curve in self.curves.values():
            curve.setData([], [])
        self.average_curve.setData([], [])
        self._bins = None
        self._data_max = None
        self._set_cursor_visible(False)
        self.readout_label.setText("")
        self.readout_item.setText("")
        self._refresh_amplitude_axis()

    def _hide_all(self) -> None:
        for curve in self.curves.values():
            curve.hide()
        self.average_curve.hide()

    def _update_frequency_axis(self, end_hz: float, nyquist: float) -> None:
        if abs(end_hz - self._end_hz) > 1e-6:
            self._end_hz = end_hz
            for line in self._grid_lines:
                self.plot_widget.removeItem(line)
            self._grid_lines = []
            step = 10.0 if end_hz <= 130.0 else (25.0 if end_hz <= 260.0 else 50.0)
            position = step
            while position < end_hz - 1e-6:
                line = pg.InfiniteLine(pos=position, angle=90, movable=False, pen=pg.mkPen("#eef2f7", width=1.0))
                line.setZValue(-10)
                self.plot_widget.addItem(line, ignoreBounds=True)
                self._grid_lines.append(line)
                position += step
        if end_hz > nyquist + 1e-6:
            self.nyquist_line.setPos(nyquist)
            self.nyquist_line.show()
        else:
            self.nyquist_line.hide()

    def render(self, block: Optional[np.ndarray], sample_rate: float, visible: Sequence[bool]) -> None:
        self.set_visible(visible)
        if self.pause_check.isChecked():
            return
        if block is None or sample_rate <= 0 or not self.curves:
            return

        channels = self._selected_channels()
        if not channels:
            self._hide_all()
            self._bins = None
            return

        points = int(self.points_combo.currentData())
        count = min(points, block.shape[0])
        if count < 32:
            return

        window = WINDOW_MAP.get(self.window_combo.currentText(), "hann")
        weights = np.hanning(count) if window == "hann" else scipy_signal.get_window(window, count)
        coherent_gain = float(np.sum(weights))
        if coherent_gain <= 0:
            return

        signature = (count, float(sample_rate), float(round(coherent_gain, 6)))
        if signature != self._signature:
            self._signature = signature
            self._smoothed.clear()

        smoothing = float(self.smoothing_combo.currentData() or 0.0)
        frequencies = np.fft.rfftfreq(count, 1.0 / float(sample_rate))
        positive = frequencies > 0.0
        segment = block[-count:]

        stored: dict[int, np.ndarray] = {}
        for channel_index in channels:
            if channel_index >= segment.shape[1]:
                continue
            values = segment[:, channel_index] - float(np.mean(segment[:, channel_index]))
            spectrum = np.abs(np.fft.rfft(values * weights)) / coherent_gain
            if spectrum.size > 2:
                spectrum[1:-1] *= np.sqrt(2.0)
            elif spectrum.size == 2:
                spectrum[1] *= np.sqrt(2.0)

            if smoothing > 0.0:
                previous = self._smoothed.get(channel_index)
                if previous is not None and previous.shape == spectrum.shape:
                    spectrum = smoothing * previous + (1.0 - smoothing) * spectrum
                self._smoothed[channel_index] = spectrum
            else:
                self._smoothed.pop(channel_index, None)

            stored[channel_index] = spectrum[positive]

        if not stored:
            self._hide_all()
            self._bins = None
            return

        average = self.channel_mode() == "average"
        if average:
            stored = {self.AVERAGE_KEY: np.mean(np.stack(list(stored.values())), axis=0)}

        self._bins = (frequencies[positive], stored)

        nyquist = float(frequencies[-1]) if frequencies.size else 0.0
        self._nyquist_hz = nyquist
        end_hz = self.end_frequency() or max(nyquist, 1.0)
        self._update_frequency_axis(end_hz, nyquist)
        self.plot_widget.setXRange(0.0, end_hz, padding=0.0)

        combined = np.concatenate(list(stored.values()))
        self._data_max = float(np.max(combined)) if combined.size else None
        low, high = self._apply_amplitude_axis()

        for channel_index, curve in self.curves.items():
            values = stored.get(channel_index)
            if values is None:
                curve.hide()
                continue
            curve.setData(frequencies[positive], self._plot_values(values, low, high))
            curve.show()

        if average:
            self.average_curve.setData(
                frequencies[positive], self._plot_values(stored[self.AVERAGE_KEY], low, high)
            )
            self.average_curve.show()
        else:
            self.average_curve.hide()

        self._place_readout()
        if not self.cursor_v.isVisible():
            self._update_peak_label()

    # -- amplitude axis ----------------------------------------------------

    def _amplitude_bounds(self) -> tuple[float, float]:
        """Amplitude window implied by the Scale/Log controls (works without data)."""
        fixed = float(self.scale_combo.currentData() or 0.0)
        data_max = self._data_max
        if self.log_check.isChecked():
            floor = FFT_LOG_FLOOR_UV
            if data_max is not None and data_max < floor * 0.9:
                floor = 10.0 ** float(np.floor(np.log10(max(data_max, 1e-6))))
            if fixed > 0.0:
                ceiling = fixed
            elif data_max is not None:
                ceiling = 10.0 ** float(np.ceil(np.log10(max(data_max * 1.15, 1e-2))))
            else:
                ceiling = 100.0
            return floor, max(ceiling, floor * 10.0)
        if fixed > 0.0:
            return 0.0, fixed
        if data_max is not None:
            return 0.0, max(data_max * 1.15, 0.01)
        return 0.0, 100.0

    @staticmethod
    def _log_ticks(bottom: float, top: float) -> list[list[tuple[float, str]]]:
        """Decade labels plus the classic 2..9 minor lines of a log grid."""
        first = int(np.floor(bottom))
        last = int(np.ceil(top))
        major = [
            (float(power), "1" if power == 0 else f"10^{power}")
            for power in range(first, last + 1)
            if bottom - 0.05 <= power <= top + 0.05
        ]
        minor: list[tuple[float, str]] = []
        for power in range(first - 1, last + 1):
            for multiplier in range(2, 10):
                position = power + float(np.log10(multiplier))
                if bottom + 0.02 <= position <= top - 0.02:
                    minor.append((position, ""))
        return [major, minor]

    def _apply_amplitude_axis(self) -> tuple[float, float]:
        """Configure the amplitude axis; returns the amplitude clip bounds."""
        low, high = self._amplitude_bounds()
        axis = self.plot_widget.getPlotItem().getAxis("left")
        if self.log_check.isChecked():
            bottom = float(np.log10(low))
            top = float(np.log10(high))
            axis.setTicks(self._log_ticks(bottom, top))
            # A hair of room keeps the outer decade labels inside the frame.
            self.plot_widget.setYRange(bottom - 0.08, top + 0.08, padding=0.0)
        else:
            axis.setTicks(None)
            self.plot_widget.setYRange(0.0, high, padding=0.0)
        return low, high

    def _plot_values(self, values: np.ndarray, low: float, high: float) -> np.ndarray:
        """Clip to the visible window and, in log mode, work in log10 units."""
        clipped = np.clip(values, low, high)
        if self.log_check.isChecked():
            return np.log10(np.maximum(clipped, 1e-9))
        return clipped

    def _redraw_stored(self) -> None:
        if self._bins is None:
            return
        frequencies, stored = self._bins
        low, high = self._amplitude_bounds()
        for channel_index, curve in self.curves.items():
            values = stored.get(channel_index)
            if values is not None:
                curve.setData(frequencies, self._plot_values(values, low, high))
        if self.AVERAGE_KEY in stored:
            self.average_curve.setData(
                frequencies, self._plot_values(stored[self.AVERAGE_KEY], low, high)
            )

    def _refresh_amplitude_axis(self, *_args) -> None:
        """React to the Scale/Log controls immediately, even while paused."""
        self._apply_amplitude_axis()
        self._redraw_stored()
        self._place_readout()

    def set_sample_rate(self, sample_rate: float) -> None:
        """Keep the frequency axis meaningful before the first data block."""
        nyquist = max(float(sample_rate) / 2.0, 1.0) if sample_rate > 0 else 0.0
        if abs(nyquist - self._nyquist_hz) > 1e-9:
            self._nyquist_hz = nyquist
            self._refresh_x_axis()

    def _refresh_x_axis(self, *_args) -> None:
        nyquist = self._nyquist_hz
        end_hz = self.end_frequency() or nyquist or 50.0
        self._update_frequency_axis(end_hz, nyquist)
        self.plot_widget.setXRange(0.0, max(end_hz, 1.0), padding=0.0)
        self._place_readout()

    # -- cursor readout ----------------------------------------------------

    def _place_readout(self) -> None:
        """Keep the readout inside the plot, top left, whatever the axis mode."""
        view_box = self.plot_widget.getPlotItem().vb
        (x_low, x_high), (y_low, y_high) = view_box.viewRange()
        self.readout_item.setPos(
            x_low + 0.015 * (x_high - x_low), y_high - 0.02 * (y_high - y_low)
        )

    def _set_readout(self, text: str) -> None:
        self.readout_item.setText(text)
        self._place_readout()


class HardwareSettingsDialog(QDialog):
    """Bulk and per-channel ADS1299 configuration.

    Everything is staged locally and pushed to the board only when one of the
    apply buttons is pressed, so the firmware is never flooded with half-edited
    settings.
    """

    applyRequested = pyqtSignal(object, list)
    verifyRequested = pyqtSignal(object)

    HEADERS = ("", "Channel", "On", "Gain", "Input", "Bias", "Reference of INN", "Reg", "Status")

    def __init__(
        self,
        hardware: HardwareSettings,
        statuses: Optional[dict[int, str]] = None,
        parent: Optional[QWidget] = None,
        link_limit: Optional[int] = None,
        reference_channel: str = "",
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Hardware settings")
        self.resize(1040, 720)
        self._board_channels = int(hardware.board_channels)
        self._rate_code = int(hardware.sample_rate_code)
        self._channels = [channel.copy() for channel in hardware.channels]
        self._mapping = physical_channel_map(self._board_channels)
        self._rows: list[dict] = []
        self._statuses = dict(statuses or {})
        self._link_limit = int(link_limit) if link_limit else None
        self._reference_mode = hardware.normalised_reference_mode()
        self._chip_modes = hardware.normalised_chip_modes()
        self._spi_clock_hz = clamp_spi_clock(hardware.spi_clock_hz)
        self._bt_power_dbm = int(getattr(hardware, "bluetooth_power_dbm", 4) or 0)
        self._spp_write_bytes = int(getattr(hardware, "spp_write_bytes", 330) or 330)
        self._decimation = clamp_decimation(hardware.decimation)
        self._reference_channel = str(reference_channel or "").strip()
        self._connected = False

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        intro = QLabel(
            "Settings are saved automatically and replayed on every connect. "
            "Use the bulk row to set every channel at once, or edit single channels for mixed measurements. "
            "The reference mode is chosen per converter, because SRB1 is one switch inside each ADS1299."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form = QHBoxLayout()
        form.addWidget(QLabel("Board profile"))
        self.board_combo = QComboBox()
        for label, value in BOARD_CHANNEL_CHOICES:
            self.board_combo.addItem(label, value)
        self.board_combo.setCurrentIndex(max(self.board_combo.findData(self._board_channels), 0))
        form.addWidget(self.board_combo)
        form.addSpacing(12)
        form.addWidget(QLabel("Data rate"))
        self.rate_combo = QComboBox()
        for rate, code in SAMPLE_RATES:
            self.rate_combo.addItem(f"{rate} SPS", code)
        self.rate_combo.setCurrentIndex(max(self.rate_combo.findData(self._rate_code), 0))
        form.addWidget(self.rate_combo)
        form.addSpacing(12)
        form.addWidget(QLabel("Decimation"))
        self.decimation_combo = QComboBox()
        for factor in DECIMATION_CHOICES:
            self.decimation_combo.addItem(f"/{factor}", factor)
        self.decimation_combo.setCurrentIndex(
            max(self.decimation_combo.findData(int(hardware.decimation)), 0)
        )
        self.decimation_combo.setToolTip(
            "The converter runs at the data rate above and the firmware block-averages this many "
            "samples into one packet.\n\n"
            "What it does: moving the data rate up and averaging back down keeps the packet rate and "
            "pushes the ADS1299 sinc3 corner (0.262 x data rate) out of the measurement band, so the "
            "top of your band is no longer rolled off.\n\n"
            "What it does NOT do: it cannot lower noise inside the measurement band. An average of N "
            "samples at f_s only removes what is above ~0.443*f_s/N - at 4000 SPS that is 221 Hz for "
            "/8 - while the noise below that corner is what the ADC's own sinc3 already left there. "
            "For the lowest in-band noise, stay at 500 SPS or lower."
        )
        form.addWidget(self.decimation_combo)
        form.addSpacing(12)
        form.addWidget(QLabel("SPI clock"))
        self.spi_combo = QComboBox()
        for hz, label in SPI_CLOCK_CHOICES:
            self.spi_combo.addItem(label, hz)
        current_spi = clamp_spi_clock(hardware.spi_clock_hz)
        if self.spi_combo.findData(current_spi) < 0:
            self.spi_combo.addItem(f"{current_spi / 1e6:g} MHz", current_spi)
        self.spi_combo.setCurrentIndex(max(self.spi_combo.findData(current_spi), 0))
        self.spi_combo.setToolTip(
            "SCLK between the ESP32 and both ADS1299. A high data rate needs a fast bus: "
            "two 216-bit frames per sample must fit inside one sample period."
        )
        form.addWidget(self.spi_combo)
        form.addSpacing(12)
        form.addWidget(QLabel("Bluetooth power"))
        self.bt_power_combo = QComboBox()
        for dbm, label in BLUETOOTH_POWER_CHOICES:
            self.bt_power_combo.addItem(label, dbm)
        self.bt_power_combo.setCurrentIndex(
            max(self.bt_power_combo.findData(int(hardware.bluetooth_power_dbm)), 0)
        )
        self.bt_power_combo.setToolTip(
            "BR/EDR transmit power the board is asked for on connect, and read back from "
            "the controller. Higher survives a worse RF path; lower radiates less into the "
            "front end."
        )
        form.addWidget(self.bt_power_combo)
        form.addSpacing(12)
        form.addWidget(QLabel("Bluetooth SPP write"))
        self.spp_combo = QComboBox()
        for size, label in SPP_WRITE_CHOICES:
            self.spp_combo.addItem(label, size)
        self.spp_combo.setCurrentIndex(
            max(self.spp_combo.findData(int(hardware.spp_write_bytes)), 0)
        )
        self.spp_combo.setToolTip(
            "Bytes per SPP write for streaming and benchmark tests. 330 uses the Arduino "
            "library's own queue and is measured fastest on this board; above that the "
            "firmware writes to the stack directly, one acknowledged write at a time."
        )
        form.addWidget(self.spp_combo)
        form.addStretch(1)
        layout.addLayout(form)

        self.rate_note = QLabel("")
        self.rate_note.setWordWrap(True)
        self.rate_note.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.rate_note)

        self.reference_group = QGroupBox("Reference preset (fills the table below)")
        reference_layout = QVBoxLayout(self.reference_group)
        reference_row = QHBoxLayout()
        self.chip_ref_combos: list[QComboBox] = []
        self.chip_ref_buttons: list[QPushButton] = []
        staged_modes = hardware.normalised_chip_modes()
        for device in range(2):
            reference_row.addWidget(QLabel(f"ADS{device + 1}"))
            combo = QComboBox()
            for label, value in REFERENCE_MODE_SHORT:
                combo.addItem(label, value)
            combo.setCurrentIndex(max(combo.findData(staged_modes[device]), 0))
            combo.setToolTip(
                "Preset for this converter only. It writes the Reference column of every powered-up "
                "input of that converter; the table remains the master setting that is sent."
            )
            reference_row.addWidget(combo, 1)
            self.chip_ref_combos.append(combo)
            button = QPushButton(f"Set ADS{device + 1}")
            button.setToolTip(
                f"Write this reference into every powered-up input of ADS{device + 1}."
            )
            button.clicked.connect(lambda _checked=False, index=device: self._apply_chip_preset(index))
            reference_row.addWidget(button)
            self.chip_ref_buttons.append(button)
            if device == 0:
                reference_row.addSpacing(12)
        # Kept for the callers that still expect a single board-wide control.
        self.reference_combo = self.chip_ref_combos[0]
        reference_layout.addLayout(reference_row)

        all_row = QHBoxLayout()
        all_row.addStretch(1)
        self.reference_all_button = QPushButton("Set both converters")
        self.reference_all_button.setToolTip("Write the two presets into every powered-up input.")
        self.reference_all_button.clicked.connect(self._apply_all_presets)
        all_row.addWidget(self.reference_all_button)
        reference_layout.addLayout(all_row)

        self.reference_note = QLabel("")
        self.reference_note.setWordWrap(True)
        self.reference_note.setStyleSheet("color: #475569; font-size: 11px;")
        reference_layout.addWidget(self.reference_note)
        layout.addWidget(self.reference_group)

        bulk_group = QGroupBox("Set every selected channel to")
        bulk_layout = QHBoxLayout(bulk_group)
        self.bulk_gain = QComboBox()
        self.bulk_gain.addItems(list(GAIN_LABELS))
        self.bulk_gain.setCurrentIndex(len(GAIN_LABELS) - 1)
        self.bulk_input = QComboBox()
        self.bulk_input.addItems(list(INPUT_TYPE_LABELS))
        self.bulk_bias = QComboBox()
        self.bulk_bias.addItems(["Include", "Exclude"])
        for label, widget in (
            ("Gain", self.bulk_gain),
            ("Input", self.bulk_input),
            ("Bias", self.bulk_bias),
        ):
            bulk_layout.addWidget(QLabel(label))
            bulk_layout.addWidget(widget)
        bulk_layout.addSpacing(10)
        selected_button = QPushButton("Apply to selected")
        selected_button.clicked.connect(lambda: self._apply_bulk(selected_only=True))
        bulk_layout.addWidget(selected_button)
        all_button = QPushButton("Apply to all")
        all_button.clicked.connect(lambda: self._apply_bulk(selected_only=False))
        bulk_layout.addWidget(all_button)
        layout.addWidget(bulk_group)

        selection_row = QHBoxLayout()
        select_all = QPushButton("Select all")
        select_all.clicked.connect(lambda: self._set_selection(True))
        select_none = QPushButton("Select none")
        select_none.clicked.connect(lambda: self._set_selection(False))
        selection_row.addWidget(select_all)
        selection_row.addWidget(select_none)
        selection_row.addStretch(1)
        self.summary_label = QLabel("")
        selection_row.addWidget(self.summary_label)
        layout.addLayout(selection_row)

        self.table = QTableWidget(len(self._mapping), len(self.HEADERS))
        self.table.setHorizontalHeaderLabels(list(self.HEADERS))
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table, 1)
        self.floating_label = QLabel("")
        self.floating_label.setWordWrap(True)
        self.floating_label.setStyleSheet("color: #b45309; font-size: 11px;")
        layout.addWidget(self.floating_label)
        layout.addWidget(self._build_verify_group())

        buttons = QDialogButtonBox()
        defaults_button = QPushButton("Board defaults")
        defaults_button.setToolTip("Reset the staged values to the firmware defaults (gain x24, normal input, bias and SRB2 on)")
        defaults_button.clicked.connect(self._restore_defaults)
        buttons.addButton(defaults_button, QDialogButtonBox.ButtonRole.ResetRole)
        apply_all_button = QPushButton("Apply all channels")
        apply_all_button.clicked.connect(lambda: self._emit_apply(selected_only=False))
        buttons.addButton(apply_all_button, QDialogButtonBox.ButtonRole.ApplyRole)
        apply_selected_button = QPushButton("Apply selected")
        apply_selected_button.clicked.connect(lambda: self._emit_apply(selected_only=True))
        buttons.addButton(apply_selected_button, QDialogButtonBox.ButtonRole.ActionRole)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        buttons.addButton(close_button, QDialogButtonBox.ButtonRole.RejectRole)
        layout.addWidget(buttons)

        self.board_combo.currentIndexChanged.connect(self._on_board_changed)
        self.rate_combo.currentIndexChanged.connect(self._on_rate_changed)
        self.decimation_combo.currentIndexChanged.connect(self._on_rate_changed)
        self.spi_combo.currentIndexChanged.connect(self._on_rate_changed)
        self._populate_table()
        self._update_rate_note()
        self._update_reference_availability()
        self._update_reference_note()
        self._refresh_registers()

    # -- configuration check ----------------------------------------------

    def set_connected(self, connected: bool) -> None:
        self._connected = bool(connected)
        self.verify_button.setEnabled(self._connected)

    def _on_verify_clicked(self) -> None:
        # Stage the current dialog state first, so verification compares what
        # the board *should* hold after the next apply.
        self.verifyRequested.emit(self.collect())

    def set_verify_result(self, text: str, state: str = "info") -> None:
        colours = {"error": "#b91c1c", "success": "#15803d", "busy": "#b45309"}
        self.verify_result_label.setStyleSheet(
            f"color: {colours.get(state, '#475569')}; font-size: 11px;"
        )
        self.verify_result_label.setText(text)

    def set_verify_busy(self, busy: bool) -> None:
        self.verify_button.setEnabled(self._connected and not busy)
        self.verify_button.setText("Verifying..." if busy else "Verify board configuration")

    # -- table plumbing ----------------------------------------------------

    def _populate_table(self) -> None:
        self._rows = []
        self.table.clearContents()
        for row, real in enumerate(self._mapping):
            setting = self._channels[row] if row < len(self._channels) else None
            padding = real is None or setting is None
            select = QCheckBox()
            select.setChecked(not padding)
            self.table.setCellWidget(row, 0, select)

            label = QTableWidgetItem(f"CH {row + 1}" + ("  (padding)" if padding else ""))
            self.table.setItem(row, 1, label)

            active = QCheckBox()
            active.setChecked(bool(setting.active) if setting else False)
            gain = QComboBox()
            gain.addItems(list(GAIN_LABELS))
            gain.setCurrentIndex(setting.gain_ordinal if setting else 6)
            input_combo = QComboBox()
            input_combo.addItems(list(INPUT_TYPE_LABELS))
            input_combo.setCurrentIndex(setting.input_ordinal if setting else 0)
            bias = QComboBox()
            bias.addItems(["Include", "Exclude"])
            bias.setCurrentIndex(0 if (setting and setting.bias) else 1)
            # The row is the master: whatever is chosen here is what gets written.
            # SRB1 is the one switch inside the chip that ties every INxN to the
            # SRB1 pin, so choosing it moves the whole converter; SRB2 is a
            # per-channel switch onto INxP, so it can be mixed freely.
            reference = QComboBox()
            reference.addItem("SRB1 pin -> all INxN (reference)", REFERENCE_MODE_SRB1)
            reference.addItem("Differential (own INN)", REFERENCE_MODE_DIFFERENTIAL)
            reference.addItem("SRB2 pin -> this INxP", REFERENCE_MODE_SRB2)
            reference.setCurrentIndex(
                max(
                    reference.findData(
                        REFERENCE_MODE_SRB1
                        if (setting and setting.srb1)
                        else (REFERENCE_MODE_SRB2 if (setting and setting.srb2) else REFERENCE_MODE_DIFFERENTIAL)
                    ),
                    0,
                )
            )
            reference.setToolTip(
                "Where this input's two sides come from - this column is what is sent to the board.\n\n"
                "SRB1 pin -> all INxN: MISC1 bit 5 closes one switch that ties the INVERTING input of "
                "every channel of that converter to the SRB1 pin. This is the reference-electrode pin "
                "the datasheet's referential montage uses (reference to SRB1, electrodes to INxP). "
                "One switch per chip, so it takes the whole converter and cannot be mixed.\n\n"
                "Differential (own INN): nothing is switched; the channel measures its own INP against "
                "its own INN. Mixable with SRB2 rows on the same chip.\n\n"
                "SRB2 pin -> this INxP: CHnSET bit 3 closes a switch between the SRB2 pin and this "
                "channel's NON-INVERTING input. It drives INP - it does NOT give the channel a "
                "negative reference. With this closed the channel measures (SRB2 pin) against "
                "whatever is wired to INN, so INN must still be connected. A reference electrode "
                "belongs on SRB1, not here."
            )
            register_label = QLabel("")
            register_label.setStyleSheet("color: #475569; font-size: 11px; font-family: Consolas;")
            status = QLabel(self._status_text(self._statuses.get(row, "")))

            widgets = (select, active, gain, input_combo, bias, reference)
            if padding:
                for widget in widgets:
                    widget.setEnabled(False)
                label.setForeground(QColor("#94a3b8"))
            self.table.setCellWidget(row, 2, active)
            self.table.setCellWidget(row, 3, gain)
            self.table.setCellWidget(row, 4, input_combo)
            self.table.setCellWidget(row, 5, bias)
            self.table.setCellWidget(row, 6, reference)
            self.table.setCellWidget(row, 7, register_label)
            self.table.setCellWidget(row, 8, status)
            self.table.setRowHeight(row, 30)

            self._rows.append(
                {
                    "select": select,
                    "active": active,
                    "gain": gain,
                    "input": input_combo,
                    "bias": bias,
                    "reference": reference,
                    "register": register_label,
                    "status": status,
                    "padding": padding,
                    "real": real,
                }
            )
            for widget in (active, gain, input_combo, bias):
                if isinstance(widget, QComboBox):
                    widget.currentIndexChanged.connect(self._refresh_registers)
                else:
                    widget.toggled.connect(self._refresh_registers)
            # The Reference cell has its own handler: it has to enforce the
            # chip-wide SRB1 rule before anything is refreshed.
            reference.currentIndexChanged.connect(
                lambda _index, row=row: self._on_row_reference_changed(row)
            )
        self.table.setColumnWidth(0, 28)
        self.table.setColumnWidth(1, 120)
        self.table.setColumnWidth(2, 44)
        self._update_summary()

    def _build_verify_group(self) -> QGroupBox:
        verify_group = QGroupBox("Configuration check")
        verify_layout = QVBoxLayout(verify_group)
        verify_row = QHBoxLayout()
        self.verify_button = QPushButton("Verify configuration")
        self.verify_button.setToolTip(
            "Ask the board to clock every ADS1299 register back out over serial and compare it with "
            "the staged settings. Can be run as often as you like."
        )
        self.verify_button.setEnabled(False)
        self.verify_button.clicked.connect(self._on_verify_clicked)
        verify_row.addWidget(self.verify_button)
        verify_row.addStretch(1)
        verify_layout.addLayout(verify_row)
        self.verify_result_label = QLabel("Connect the board to enable the register readback.")
        self.verify_result_label.setWordWrap(True)
        self.verify_result_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.verify_result_label.setStyleSheet("color: #475569; font-size: 11px;")
        verify_layout.addWidget(self.verify_result_label)
        # What the board says it is actually doing, never what the app remembers.
        self.board_state_label = QLabel("Board state: not read yet.")
        self.board_state_label.setWordWrap(True)
        self.board_state_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.board_state_label.setStyleSheet("color: #1e293b; font-size: 11px; font-family: Consolas;")
        verify_layout.addWidget(self.board_state_label)
        return verify_group

    def set_board_state(self, text: str) -> None:
        self.board_state_label.setText(text or "Board state: not read yet.")

    def _sync_row(self, row: int) -> None:
        widgets = self._rows[row]
        setting = self._channels[row]
        for widget in (widgets["active"], widgets["gain"], widgets["input"], widgets["bias"], widgets["reference"]):
            widget.blockSignals(True)
        widgets["active"].setChecked(setting.active)
        widgets["gain"].setCurrentIndex(setting.gain_ordinal)
        widgets["input"].setCurrentIndex(setting.input_ordinal)
        widgets["bias"].setCurrentIndex(0 if setting.bias else 1)
        target = (
            REFERENCE_MODE_SRB1
            if setting.srb1
            else (REFERENCE_MODE_SRB2 if setting.srb2 else REFERENCE_MODE_DIFFERENTIAL)
        )
        widgets["reference"].setCurrentIndex(max(widgets["reference"].findData(target), 0))
        for widget in (widgets["active"], widgets["gain"], widgets["input"], widgets["bias"], widgets["reference"]):
            widget.blockSignals(False)
        self._refresh_registers()

    def _collect_from_rows(self) -> None:
        for row, widgets in enumerate(self._rows):
            if widgets["padding"] or row >= len(self._channels):
                continue
            setting = self._channels[row]
            setting.active = widgets["active"].isChecked()
            setting.gain_ordinal = int(widgets["gain"].currentIndex())
            setting.input_ordinal = int(widgets["input"].currentIndex())
            setting.bias = widgets["bias"].currentIndex() == 0
            mode = str(widgets["reference"].currentData() or REFERENCE_MODE_SRB2)
            if mode == REFERENCE_MODE_SRB1:
                # Keep the remembered SRB2-pin/own-INN choice: while a converter is
                # in SRB1 the column can only show SRB1, and overwriting the other
                # bit here would lose the mix the moment the converter is released.
                setting.srb1 = True
            else:
                setting.srb1 = False
                setting.srb2 = mode == REFERENCE_MODE_SRB2

    def collect(self) -> HardwareSettings:
        self._collect_from_rows()
        # The reference channel is excluded from the bias loop by rule, not by the
        # operator's choice, so enforce it on the staged copy as well.
        if self._reference_channel:
            for row in range(len(self._channels)):
                if channel_label(row) == self._reference_channel:
                    self._channels[row].bias = False
        hardware = HardwareSettings(
            board_channels=self._board_channels,
            sample_rate_code=self._rate_code,
            reference_mode=self._reference_mode,
            chip_reference_modes=list(self._chip_modes),
            spi_clock_hz=self._spi_clock_hz,
            decimation=self._decimation,
            bluetooth_power_dbm=self._bt_power_dbm,
            spp_write_bytes=self._spp_write_bytes,
            channels=[channel.copy() for channel in self._channels],
        )
        # The rows are the master, so the chip modes are derived from them rather
        # than stored alongside and allowed to disagree.
        hardware.sync_reference()
        return hardware

    def _staged_hardware(self) -> HardwareSettings:
        """The table's current state as a HardwareSettings, without touching chips."""

        return HardwareSettings(
            board_channels=self._board_channels,
            sample_rate_code=self._rate_code,
            reference_mode=self._reference_mode,
            spi_clock_hz=self._spi_clock_hz,
            decimation=self._decimation,
            bluetooth_power_dbm=self._bt_power_dbm,
            spp_write_bytes=self._spp_write_bytes,
            channels=[channel.copy() for channel in self._channels],
        )

    def _on_row_reference_changed(self, row: int) -> None:
        """Apply a row's reference, keeping SRB1 chip-wide.

        SRB1 is one switch per converter, so a row can never hold it alone: asking
        for it moves every powered-up input of that converter, and asking for
        anything else releases the converter. That is what makes mixing SRB1 with
        SRB2/differential inside one ADS1299 impossible rather than merely
        discouraged.
        """

        if row >= len(self._rows) or self._rows[row]["padding"]:
            return
        self._collect_from_rows()
        mode = str(self._rows[row]["reference"].currentData() or REFERENCE_MODE_SRB2)
        self._sync_rows_from_reference(row, mode)

    def _sync_rows_from_reference(self, row: int, mode: str) -> None:
        if row >= len(self._channels):
            return
        hardware = self._staged_hardware()
        hardware.set_channel_reference(row, mode)
        self._channels = [channel.copy() for channel in hardware.channels]
        self._chip_modes = hardware.normalised_chip_modes()
        self._reference_mode = self._chip_modes[0]
        self._push_reference_to_rows()
        self._update_reference_note()
        self._refresh_registers()

    def _push_reference_to_rows(self) -> None:
        """Refresh every Reference cell from the staged channel state."""

        for row, widgets in enumerate(self._rows):
            if widgets["padding"] or row >= len(self._channels):
                continue
            setting = self._channels[row]
            target = (
                REFERENCE_MODE_SRB1
                if setting.srb1
                else (REFERENCE_MODE_SRB2 if setting.srb2 else REFERENCE_MODE_DIFFERENTIAL)
            )
            combo = widgets["reference"]
            combo.blockSignals(True)
            combo.setCurrentIndex(max(combo.findData(target), 0))
            combo.blockSignals(False)

    def _apply_chip_preset(self, device: int) -> None:
        mode = str(self.chip_ref_combos[device].currentData() or REFERENCE_MODE_SRB2)
        hardware = self._staged_hardware()
        hardware.set_chip_reference_mode(device, mode)
        self._channels = [channel.copy() for channel in hardware.channels]
        self._chip_modes = hardware.normalised_chip_modes()
        self._reference_mode = self._chip_modes[0]
        self._push_reference_to_rows()
        self._update_reference_note()
        self._refresh_registers()

    def _apply_all_presets(self) -> None:
        hardware = self._staged_hardware()
        for device in range(2):
            hardware.set_chip_reference_mode(
                device, str(self.chip_ref_combos[device].currentData() or REFERENCE_MODE_SRB2)
            )
        self._channels = [channel.copy() for channel in hardware.channels]
        self._chip_modes = hardware.normalised_chip_modes()
        self._reference_mode = self._chip_modes[0]
        self._push_reference_to_rows()
        self._update_reference_note()
        self._refresh_registers()

    def _refresh_registers(self, *_args) -> None:
        """Show the CHnSET byte each row will produce and flag dead configurations.

        The register column is what makes the row authoritative: it shows the byte
        the firmware will actually hold for that input, so SRB1 versus SRB2-versus-
        INP versus own INN can be told apart at a glance.
        """

        self._collect_from_rows()
        hardware = self._staged_hardware()
        modes = hardware.resolve_chip_modes()
        self._chip_modes = modes
        self._reference_mode = modes[0]
        per_device = max(channels_per_device(self._board_channels), 1)
        notes: list[str] = []
        srb2_rows: list[str] = []
        for row, widgets in enumerate(self._rows):
            if widgets["padding"]:
                widgets["register"].setText("")
                widgets["reference"].setEnabled(False)
                continue
            setting = self._channels[row]
            real = self._mapping[row] if row < len(self._mapping) else None
            device = min(int(real) // per_device, 1) if real is not None else 0
            mode = modes[min(device, len(modes) - 1)]
            is_reference = self._reference_channel and channel_label(row) == self._reference_channel
            widgets["reference"].setEnabled(setting.active)
            # The reference input is a stimulus/reference node, so it never joins
            # the bias loop. Lock the cell rather than silently overriding an edit.
            widgets["bias"].setEnabled(setting.active and not is_reference)
            if is_reference:
                widgets["bias"].setToolTip(
                    "The reference channel is always excluded from the bias/DRL loop: it carries the "
                    "stimulus (or the shared reference), and feeding it to the bias amplifier would "
                    "drive the DRL electrode with the voltage under test."
                )
            register = channel_register_value(setting, mode)
            widgets["register"].setText(f"0x{register:02X}")
            if setting.active and setting.srb2 and mode != REFERENCE_MODE_SRB1:
                srb2_rows.append(channel_label(row))

        active_bias = any(
            self._channels[row].active and self._channels[row].bias
            for row, widgets in enumerate(self._rows)
            if not widgets["padding"] and row < len(self._channels)
        )
        if srb2_rows:
            shown = ", ".join(srb2_rows[:6]) + ("..." if len(srb2_rows) > 6 else "")
            notes.append(
                f"SRB2 switch closed on {shown}: this ties INxP to the SRB2 pin. That is not a "
                f"negative reference - those inputs still measure against their own INN, so INN has "
                f"to be wired. For a reference electrode on the inverting input use SRB1."
            )
        if not active_bias and (srb2_rows or any(mode != REFERENCE_MODE_DIFFERENTIAL for mode in modes)):
            notes.append(
                "No input feeds the bias/DRL amplifier, so there is no DC return path. A referenced "
                "input without bias drifts to a rail and looks like a floating electrode. Enable bias "
                "on at least one input and connect the BIAS electrode."
            )
        self.floating_label.setText("  ".join(notes))
        self._update_summary()

    def _update_reference_note(self) -> None:
        modes = list(self._chip_modes)
        self._reference_mode = modes[0] if modes[0] == modes[1] else modes[0]
        lines: list[str] = []
        for device, mode in enumerate(modes):
            if mode == REFERENCE_MODE_SRB1:
                lines.append(
                    f"ADS{device + 1}: SRB1 - MISC1 bit 5 closes one switch that ties the INVERTING "
                    "input of every channel of that converter to the SRB1 pin (referential montage)."
                )
            elif mode == REFERENCE_MODE_SRB2:
                lines.append(
                    f"ADS{device + 1}: some inputs have the SRB2 switch closed, which drives their "
                    "INxP from the SRB2 pin; the rest use their own INP/INN."
                )
            else:
                lines.append(
                    f"ADS{device + 1}: every input is true differential - no SRB routing at all."
                )
        lines.append(
            "SRB1 and SRB2 are different pins: SRB1 feeds INxN (a reference), SRB2 feeds INxP (a "
            "stimulus). The table is the master and is what gets written; the presets only fill it."
        )
        modes_equal = modes[0] == modes[1]
        if not modes_equal:
            presets = ", ".join(
                f"ADS{device + 1} {REFERENCE_MODE_SHORT_LABELS.get(mode, mode)}"
                for device, mode in enumerate(modes)
            )
            lines.append(f"Current: {presets}.")
        self.reference_note.setText("  ".join(lines))

    def _on_board_changed(self) -> None:
        board_channels = int(self.board_combo.currentData() or 16)
        if board_channels == self._board_channels:
            return
        self._collect_from_rows()
        self._board_channels = board_channels
        self._mapping = physical_channel_map(board_channels)
        target = 16 if board_channels > 8 else 8
        while len(self._channels) < target:
            from live_protocol import ChannelSetting

            self._channels.append(ChannelSetting())
        if len(self._channels) > target:
            del self._channels[target:]
        self._populate_table()
        self._update_rate_note()
        self._update_reference_availability()

    def _update_reference_availability(self) -> None:
        """Disable the second converter's preset and button on a single-chip board."""

        two_chips = self._board_channels > 8
        if len(self.chip_ref_combos) > 1:
            combo = self.chip_ref_combos[1]
            combo.setEnabled(two_chips)
            combo.setToolTip(
                "Preset for the second ADS1299."
                if two_chips
                else "This board profile has one converter, so ADS2 does not exist."
            )
        if len(self.chip_ref_buttons) > 1:
            self.chip_ref_buttons[1].setEnabled(two_chips)

    def _on_rate_changed(self) -> None:
        self._rate_code = int(self.rate_combo.currentData() if self.rate_combo.currentData() is not None else 6)
        self._decimation = clamp_decimation(self.decimation_combo.currentData() or 1)
        self._spi_clock_hz = clamp_spi_clock(self.spi_combo.currentData() or ADS_SPI_CLOCK_HZ)
        self._bt_power_dbm = int(self.bt_power_combo.currentData() or 0)
        self._spp_write_bytes = int(self.spp_combo.currentData() or 330)
        self._update_rate_note()

    def _update_rate_note(self) -> None:
        rate = SAMPLE_RATE_BY_CODE.get(self._rate_code, 250)
        decimation = max(int(self._decimation), 1)
        packets = rate / decimation
        per_channel = packets / 2.0 if self._board_channels > 8 else packets
        corner = sinc3_corner_hz(rate)
        average_corner = boxcar_corner_hz(rate, decimation)
        parts = [
            f"per channel: {per_channel:g} Hz",
            f"{packets:g} packets/s",
            f"ADC sinc3 -3 dB: {corner:g} Hz",
            f"noise bandwidth: {noise_bandwidth_hz(rate):g} Hz",
        ]
        if decimation > 1:
            # Be explicit about what averaging can and cannot do: it removes what
            # is above this corner, it cannot remove noise inside the band.
            parts.append(
                f"MCU average /{decimation}: -3 dB at {average_corner:g} Hz, "
                f"null at {boxcar_null_hz(rate, decimation):g} Hz"
            )
        parts.append(f"SPI {self._spi_clock_hz / 1e6:g} MHz serves {max_sample_rate_from_spi(self._spi_clock_hz, 2)} SPS")
        if self._link_limit:
            parts.append(f"serial carries {self._link_limit} packets/s")
        # Judged against the limit the window actually reports, not a hardcoded
        # 921600: assuming the slowest link made a 2 Mbaud port look over budget when
        # it was not, which is what pushed the data rate and the decimation around.
        serial_limit = self._link_limit or max_sustained_packet_rate(921600)
        fits = packets <= serial_limit and rate <= max_sample_rate_from_spi(self._spi_clock_hz, 2)
        if not fits:
            reasons = []
            if packets > serial_limit:
                reasons.append(f"{packets:g} packets/s exceeds the link's {serial_limit} packets/s")
            spi_limit = max_sample_rate_from_spi(self._spi_clock_hz, 2)
            if rate > spi_limit:
                reasons.append(f"{rate:g} SPS exceeds the bus limit of {spi_limit} SPS")
            parts.append("OVER BUDGET: " + "; ".join(reasons))
        self.rate_note.setText("  |  ".join(parts))
        self.rate_note.setStyleSheet(
            "color: #b91c1c; font-size: 11px;" if not fits else "color: #475569; font-size: 11px;"
        )

    def _set_selection(self, checked: bool) -> None:
        for row, widgets in enumerate(self._rows):
            if widgets["padding"]:
                continue
            widgets["select"].setChecked(checked)
        self._update_summary()

    def _update_summary(self) -> None:
        selected = sum(1 for widgets in self._rows if not widgets["padding"] and widgets["select"].isChecked())
        usable = sum(1 for rows in self._rows if not rows["padding"])
        self.summary_label.setText(f"{selected} of {usable} real channels selected")

    def _apply_bulk(self, selected_only: bool) -> None:
        gain = self.bulk_gain.currentIndex()
        input_ordinal = self.bulk_input.currentIndex()
        bias = self.bulk_bias.currentIndex() == 0
        for row, widgets in enumerate(self._rows):
            if widgets["padding"] or row >= len(self._channels):
                continue
            if selected_only and not widgets["select"].isChecked():
                continue
            setting = self._channels[row]
            setting.gain_ordinal = gain
            setting.input_ordinal = input_ordinal
            setting.bias = bias
            self._sync_row(row)

    def _restore_defaults(self) -> None:
        from live_protocol import ChannelSetting

        for row in range(len(self._channels)):
            self._channels[row] = ChannelSetting()
            self._sync_row(row)
        self._chip_modes = [REFERENCE_MODE_SRB2, REFERENCE_MODE_SRB2]
        self._reference_mode = REFERENCE_MODE_SRB2
        for combo in self.chip_ref_combos:
            combo.blockSignals(True)
            combo.setCurrentIndex(max(combo.findData(REFERENCE_MODE_SRB2), 0))
            combo.blockSignals(False)
        self._push_reference_to_rows()
        self.decimation_combo.setCurrentIndex(max(self.decimation_combo.findData(1), 0))
        self.spi_combo.setCurrentIndex(max(self.spi_combo.findData(ADS_SPI_CLOCK_HZ), 0))
        self._on_rate_changed()
        self._update_reference_note()

    def _emit_apply(self, selected_only: bool) -> None:
        self._collect_from_rows()
        hardware = self.collect()
        indices = [
            row
            for row, widgets in enumerate(self._rows)
            if not widgets["padding"] and (not selected_only or widgets["select"].isChecked())
        ]
        self.applyRequested.emit(hardware, indices)

    def update_statuses(self, statuses: dict[int, str]) -> None:
        self._statuses = dict(statuses)
        for row, widgets in enumerate(self._rows):
            widgets["status"].setText(self._status_text(self._statuses.get(row, "")))

    @staticmethod
    def _status_text(status: str) -> str:
        return {
            "ok": "written",
            "failed": "rejected",
            "timeout": "no reply",
            "pending": "sending...",
        }.get(status, "")

class SerialMonitorWindow(QDialog):
    """Waterfall of the non-data serial traffic plus a raw command sender."""

    sendRequested = pyqtSignal(bytes)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Serial monitor - board messages")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(980, 560)
        self._records: deque[tuple[str, str, str, str]] = deque(maxlen=20000)
        self._paused = False

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        toolbar = QHBoxLayout()
        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self._on_pause)
        toolbar.addWidget(self.pause_button)

        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self._on_clear)
        toolbar.addWidget(clear_button)

        save_button = QPushButton("Save log...")
        save_button.clicked.connect(self._on_save)
        toolbar.addWidget(save_button)

        toolbar.addSpacing(12)
        self.show_info = QCheckBox("Info")
        self.show_info.setChecked(True)
        self.show_info.toggled.connect(self._rebuild)
        toolbar.addWidget(self.show_info)
        self.show_success = QCheckBox("Success")
        self.show_success.setChecked(True)
        self.show_success.toggled.connect(self._rebuild)
        toolbar.addWidget(self.show_success)
        self.show_errors = QCheckBox("Errors")
        self.show_errors.setChecked(True)
        self.show_errors.toggled.connect(self._rebuild)
        toolbar.addWidget(self.show_errors)
        self.show_sent = QCheckBox("Sent")
        self.show_sent.setChecked(True)
        self.show_sent.toggled.connect(self._rebuild)
        toolbar.addWidget(self.show_sent)
        self.show_stats = QCheckBox("Data stats")
        self.show_stats.setChecked(True)
        self.show_stats.toggled.connect(self._rebuild)
        toolbar.addWidget(self.show_stats)
        toolbar.addStretch(1)
        self.stats_label = QLabel("waiting for board")
        toolbar.addWidget(self.stats_label)
        layout.addLayout(toolbar)

        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(40000)
        self.view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setPointSize(9)
        self.view.setFont(font)
        layout.addWidget(self.view, 1)

        send_row = QHBoxLayout()
        send_row.addWidget(QLabel("Send"))
        self.command_edit = QLineEdit()
        self.command_edit.setPlaceholderText("Cyton command bytes, e.g. v or x1060110X")
        self.command_edit.returnPressed.connect(self._send)
        send_row.addWidget(self.command_edit, 1)
        self.hex_check = QCheckBox("Hex")
        send_row.addWidget(self.hex_check)
        self.newline_check = QCheckBox("Append newline")
        send_row.addWidget(self.newline_check)
        send_button = QPushButton("Send")
        send_button.clicked.connect(self._send)
        send_row.addWidget(send_button)
        layout.addLayout(send_row)

    # -- content -----------------------------------------------------------

    def append_record(self, timestamp: str, direction: str, text: str, kind: str = "info") -> None:
        self._records.append((timestamp, direction, text, kind))
        if self._paused or not self._kind_visible(kind, direction):
            return
        self.view.appendHtml(self._format(timestamp, direction, text, kind))
        self.view.verticalScrollBar().setValue(self.view.verticalScrollBar().maximum())

    def set_stats(self, text: str) -> None:
        self.stats_label.setText(text)

    def _kind_visible(self, kind: str, direction: str) -> bool:
        if direction == "sent":
            return self.show_sent.isChecked()
        if kind == "stats":
            return self.show_stats.isChecked()
        if kind == "error":
            return self.show_errors.isChecked()
        if kind == "success":
            return self.show_success.isChecked()
        return self.show_info.isChecked()

    def _format(self, timestamp: str, direction: str, text: str, kind: str) -> str:
        from html import escape

        colors = {
            "info": "#334155",
            "success": "#15803d",
            "error": "#b91c1c",
            "debug": "#7c3aed",
            "sent": "#1d4ed8",
            "stats": "#64748b",
        }
        arrow = {"in": "&larr;", "out": "&rarr;", "sent": "&rarr;", "stats": "&middot;"}.get(direction, "&middot;")
        color = colors.get(kind if direction != "sent" else "sent", "#334155")
        return (
            f'<span style="color:#94a3b8;">{escape(timestamp)}</span> '
            f'<span style="color:#64748b;">{arrow}</span> '
            f'<span style="color:{color};">{escape(text)}</span>'
        )

    def _rebuild(self) -> None:
        self.view.clear()
        for timestamp, direction, text, kind in self._records:
            if self._kind_visible(kind, direction):
                self.view.appendHtml(self._format(timestamp, direction, text, kind))

    def _on_pause(self, paused: bool) -> None:
        self._paused = paused
        self.pause_button.setText("Resume" if paused else "Pause")
        if not paused:
            self._rebuild()

    def _on_clear(self) -> None:
        self._records.clear()
        self.view.clear()

    def _on_save(self) -> None:
        file_name, _ = QFileDialog.getSaveFileName(self, "Save serial log", "serial_log.txt", "Text files (*.txt);;All files (*)")
        if not file_name:
            return
        try:
            with open(file_name, "w", encoding="utf-8") as handle:
                for timestamp, direction, text, kind in self._records:
                    handle.write(f"{timestamp}\t{direction}\t{kind}\t{text}\n")
        except OSError as exc:
            QMessageBox.warning(self, "Save log", f"Could not write the log file.\n\n{exc}")

    def _send(self) -> None:
        text = self.command_edit.text()
        if not text:
            return
        if self.hex_check.isChecked():
            cleaned = text.replace(",", " ").replace("0x", " ").strip()
            try:
                payload = bytes.fromhex(cleaned)
            except ValueError:
                QMessageBox.warning(self, "Send", "The hexadecimal payload could not be parsed.")
                return
        else:
            payload = text.encode("utf-8")
            if self.newline_check.isChecked():
                payload += b"\n"
        self.sendRequested.emit(payload)
        self.command_edit.clear()

class GeneratorWorker(QThread):
    """Applies one manual generator state off the UI thread.

    Opening the USBTMC resource costs hundreds of milliseconds and a hung front
    panel can cost the whole 3 s SCPI timeout, so every command from the dock runs
    here instead of inside a click handler.
    """

    applied = pyqtSignal(object, list)
    failed = pyqtSignal(str)
    progress = pyqtSignal(str)

    def __init__(
        self,
        generator: UTG932E,
        output: GeneratorOutput,
        resource: str = "",
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.generator = generator
        self.output = output
        self.resource = resource

    def run(self) -> None:  # noqa: D401 - QThread entry point
        try:
            if not self.generator.is_open:
                self.progress.emit("connecting to the generator...")
                identity = self.generator.open(self.resource or None)
                self.generator.identity = identity
                self.progress.emit(f"connected: {identity}")
                self.generator.unlock_front_panel()
            notes = self.generator.program_output(self.output)
            self.applied.emit(self.output, notes)
        except UTGError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:  # noqa: BLE001 - never let the thread die silently
            self.failed.emit(f"{type(exc).__name__}: {exc}")


class AutoSweepWorker(QThread):
    """Drives the UTG932E through the frequency list while the recorder runs.

    Each step is applied and then held for its computed dwell, which is the
    larger of 300 ms and ten (above 50 Hz) or five (at or below 50 Hz) whole
    periods. The window records the moment every step was applied, so the
    analyzer knows which slice of the recording belongs to which frequency
    instead of having to guess it from the spectrum.
    """

    progress = pyqtSignal(str)
    stepApplied = pyqtSignal(int)
    warning = pyqtSignal(str)
    failed = pyqtSignal(str)
    completed = pyqtSignal()

    def __init__(
        self,
        generator: UTG932E,
        plan: SweepPlan,
        steps: Sequence[SweepStep],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.generator = generator
        self.plan = plan
        self.steps = list(steps)
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    def run(self) -> None:  # noqa: D401 - QThread entry point
        try:
            for index, step in enumerate(self.steps):
                if self._stop.is_set():
                    break
                started = time.monotonic()
                # Only the first step is read back. That proves the whole
                # programming path on this instrument, while the later steps are a
                # single batched frequency command each - a per-step readback would
                # add two query round trips to every point and make the frequency
                # change and the dwell start disagree.
                problems = self.generator.apply_step(
                    step, self.plan.drive, verify=index == 0
                )
                if problems:
                    # Never let a silently rejected write become a CMRR point.
                    self.warning.emit(f"{step.frequency_hz:g} Hz: " + "; ".join(problems))
                self.stepApplied.emit(index)
                self.progress.emit(
                    f"{step.frequency_hz:g} Hz held for {step.dwell_s:.2f} s "
                    f"({index + 1}/{len(self.steps)})"
                )
                while not self._stop.is_set():
                    remaining = step.dwell_s - (time.monotonic() - started)
                    if remaining <= 0:
                        break
                    time.sleep(min(remaining, 0.05))
            try:
                self.generator.all_off()
            except UTGError:
                pass
            self.completed.emit()
        except UTGError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:  # noqa: BLE001 - never let the thread die silently
            self.failed.emit(f"{type(exc).__name__}: {exc}")


class AutoSweepDialog(QDialog):
    """Automated CMRR / crosstalk sweep: generator settings and the plan."""

    startRequested = pyqtSignal(object)
    stopRequested = pyqtSignal()

    def __init__(
        self,
        settings: SweepSettings,
        channel_names: Sequence[str],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Automated measurement - function generator")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(720, 680)
        self._settings = SweepSettings.from_dict(settings.as_dict())
        self._running = False

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        generator_group = QGroupBox("Generator (UNI-T UTG932E over USBTMC)")
        generator_layout = QGridLayout(generator_group)
        self.resource_combo = QComboBox()
        self.resource_combo.setEditable(True)
        self.resource_combo.setMinimumWidth(320)
        if self._settings.resource:
            self.resource_combo.addItem(self._settings.resource)
        generator_layout.addWidget(QLabel("VISA resource"), 0, 0)
        generator_layout.addWidget(self.resource_combo, 0, 1, 1, 2)
        refresh_button = QPushButton("Find")
        refresh_button.setToolTip("List the VISA instruments that are plugged in")
        refresh_button.clicked.connect(self._refresh_resources)
        generator_layout.addWidget(refresh_button, 0, 3)

        self.test_button = QPushButton("Test generator")
        self.test_button.setToolTip(
            "Connect, program one sine, read the settings back and switch the output off again. "
            "Proves the VISA resource and that the firmware accepts typed SCPI before a sweep runs."
        )
        self.test_button.clicked.connect(self._test_generator)
        generator_layout.addWidget(self.test_button, 0, 4)

        self.amplitude_label = QLabel("Amplitude (Vrms)")
        generator_layout.addWidget(self.amplitude_label, 1, 0)
        self.amplitude_spin = QDoubleSpinBox()
        self.amplitude_spin.setRange(0.001, 20.0)
        self.amplitude_spin.setDecimals(4)
        self.amplitude_spin.setSuffix(" Vrms")
        self.amplitude_spin.setValue(float(self._settings.amplitude_vrms))
        self.amplitude_spin.setToolTip(
            "Level the amplifier should see. In differential mode the two outputs add, so each "
            "channel is programmed with half of this."
        )
        generator_layout.addWidget(self.amplitude_spin, 1, 1)
        generator_layout.addWidget(QLabel("Load"), 1, 2)
        self.load_spin = QSpinBox()
        self.load_spin.setRange(1, 1_000_000)
        self.load_spin.setSuffix(" ohm")
        self.load_spin.setValue(int(self._settings.load_ohms))
        self.load_spin.setToolTip(
            "Tell the generator the DUT is high impedance, otherwise it halves the amplitude it drives."
        )
        generator_layout.addWidget(self.load_spin, 1, 3)

        generator_layout.addWidget(QLabel("Drive"), 2, 0)
        self.drive_combo = QComboBox()
        self.drive_combo.addItem("Common mode - CH1 drives every input (CMRR)", "common")
        self.drive_combo.addItem("Differential - CH1 0 deg, CH2 180 deg (crosstalk)", "differential")
        self.drive_combo.addItem("Gain - CH1 into the input under test", "gain")
        self.drive_combo.setCurrentIndex(
            max(self.drive_combo.findData(self._settings.drive_kind), 0)
        )
        self.drive_combo.setToolTip(
            "Common mode: CMRR. The analyzer divides the applied common-mode amplitude by what each "
            "channel measured.\n"
            "Differential: a real differential stimulus for crosstalk, CH1 and CH2 phase-locked 180 "
            "degrees apart.\n"
            "Gain: CH1 drives the input under test against the board reference; the analyzer reports "
            "measured RMS / applied RMS per channel."
        )
        generator_layout.addWidget(self.drive_combo, 2, 1, 1, 3)

        generator_layout.addWidget(QLabel("CM monitor"), 3, 0)
        self.monitor_combo = QComboBox()
        self.monitor_combo.addItem("(none)", "")
        for name in channel_names:
            self.monitor_combo.addItem(name, name)
        position = self.monitor_combo.findData(self._settings.monitor_channel)
        if position >= 0:
            self.monitor_combo.setCurrentIndex(position)
        self.monitor_combo.setToolTip(
            "Channel wired straight across the common-mode voltage. The analyzer reads the applied "
            "frequency from it, which is what makes the low end of the sweep usable."
        )
        generator_layout.addWidget(self.monitor_combo, 3, 1, 1, 3)

        self.drive_hint_label = QLabel("")
        self.drive_hint_label.setWordWrap(True)
        self.drive_hint_label.setStyleSheet("color: #475569; font-size: 11px;")
        generator_layout.addWidget(self.drive_hint_label, 4, 0, 1, 5)
        self.drive_combo.currentIndexChanged.connect(self._refresh_drive_hint)
        self.amplitude_spin.valueChanged.connect(self._refresh_drive_hint)
        self._refresh_drive_hint()
        layout.addWidget(generator_group)

        plan_group = QGroupBox("Sweep plan")
        plan_layout = QVBoxLayout(plan_group)
        self.frequency_edit = QPlainTextEdit()
        self.frequency_edit.setPlaceholderText("one frequency per line, e.g. 110")
        self.frequency_edit.setPlainText(
            "\n".join(f"{value:g}" for value in self._settings.frequencies)
        )
        self.frequency_edit.setMinimumHeight(150)
        plan_layout.addWidget(QLabel("Frequencies (Hz), high to low"))
        plan_layout.addWidget(self.frequency_edit, 1)

        dwell_row = QHBoxLayout()
        dwell_row.addWidget(QLabel("Minimum dwell"))
        self.dwell_spin = QDoubleSpinBox()
        self.dwell_spin.setRange(0.05, 60.0)
        self.dwell_spin.setDecimals(2)
        self.dwell_spin.setSingleStep(0.05)
        self.dwell_spin.setSuffix(" s")
        self.dwell_spin.setValue(float(self._settings.min_dwell_s))
        dwell_row.addWidget(self.dwell_spin)
        dwell_row.addWidget(QLabel("Periods above 50 Hz"))
        self.periods_high = QSpinBox()
        self.periods_high.setRange(1, 100)
        self.periods_high.setValue(int(self._settings.periods_above))
        dwell_row.addWidget(self.periods_high)
        dwell_row.addWidget(QLabel("at or below 50 Hz"))
        self.periods_low = QSpinBox()
        self.periods_low.setRange(1, 100)
        self.periods_low.setValue(int(self._settings.periods_below))
        dwell_row.addWidget(self.periods_low)
        dwell_row.addStretch(1)
        plan_layout.addLayout(dwell_row)

        reset_row = QHBoxLayout()
        defaults_button = QPushButton("Default frequencies")
        defaults_button.clicked.connect(self._restore_default_frequencies)
        reset_row.addWidget(defaults_button)
        self.plan_summary = QLabel("")
        self.plan_summary.setWordWrap(True)
        self.plan_summary.setStyleSheet("color: #475569; font-size: 11px;")
        reset_row.addWidget(self.plan_summary, 1)
        plan_layout.addLayout(reset_row)
        layout.addWidget(plan_group, 1)

        self.status_label = QLabel("Idle. The sweep starts a recording and stops it when it finishes.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #1e293b;")
        layout.addWidget(self.status_label)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("Start sweep & record")
        self.start_button.clicked.connect(self._on_start)
        buttons.addWidget(self.start_button)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._on_stop)
        buttons.addWidget(self.stop_button)
        buttons.addStretch(1)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.reject)
        buttons.addWidget(close_button)
        layout.addLayout(buttons)

        for widget in (self.amplitude_spin, self.dwell_spin):
            widget.valueChanged.connect(self._update_summary)
        for widget in (self.periods_high, self.periods_low):
            widget.valueChanged.connect(self._update_summary)
        self.frequency_edit.textChanged.connect(self._update_summary)
        self._update_summary()

    # -- helpers -----------------------------------------------------------

    def _refresh_resources(self) -> None:
        try:
            resources = list_visa_resources()
        except UTGError as exc:
            self.status_label.setText(str(exc))
            return
        current = self.resource_combo.currentText()
        self.resource_combo.clear()
        self.resource_combo.addItems(resources)
        if current:
            self.resource_combo.setCurrentText(current)

    def _restore_default_frequencies(self) -> None:
        self.frequency_edit.setPlainText("\n".join(f"{value:g}" for value in DEFAULT_SWEEP_FREQUENCIES_HZ))

    def _test_generator(self) -> None:
        """Prove the generator link without starting a recording."""

        frequencies = self._frequencies()
        probe = frequencies[0] if frequencies else 110.0
        self.set_status(f"Testing the generator at {probe:g} Hz...")
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            lines = run_generator_selftest(
                self.resource_combo.currentText().strip(), probe, float(self.amplitude_spin.value())
            )
        except UTGError as exc:
            self.set_status(f"Generator test failed: {exc}", error=True)
            return
        finally:
            QApplication.restoreOverrideCursor()
        failed = any(line.startswith(("MISMATCH", "ERROR")) for line in lines)
        self.set_status("  ".join(lines), error=failed)

    def _frequencies(self) -> list[float]:
        return parse_frequency_list(self.frequency_edit.toPlainText())

    def _update_summary(self) -> None:
        frequencies = self._frequencies()
        if not frequencies:
            self.plan_summary.setText("No valid frequency in the list.")
            return
        holds = [
            dwell_seconds(
                value,
                float(self.dwell_spin.value()),
                int(self.periods_high.value()),
                int(self.periods_low.value()),
            )
            for value in frequencies
        ]
        total = sum(holds)
        self.plan_summary.setText(
            f"{len(frequencies)} steps, {total:.1f} s total "
            f"(shortest hold {min(holds):.2f} s, longest {max(holds):.2f} s)"
        )

    def _refresh_drive_hint(self) -> None:
        """Say what each output gets, so a differential level is never double-counted."""

        differential = str(self.drive_combo.currentData() or "") == "differential"
        amplitude = float(self.amplitude_spin.value())
        self.amplitude_label.setText("Amplitude (Vrms diff)" if differential else "Amplitude (Vrms)")
        if differential:
            each = per_channel_amplitude_vrms(amplitude, True)
            self.drive_hint_label.setText(
                f"CH1 {each:g} Vrms at 0 deg + CH2 {each:g} Vrms at 180 deg = "
                f"{amplitude:g} Vrms differential. Each output is programmed with half, because the "
                f"amplifier sees their difference."
            )
        else:
            self.drive_hint_label.setText(
                f"CH1 is programmed with the full {amplitude:g} Vrms; CH2 stays off."
            )

    def settings(self) -> SweepSettings:
        drive = str(self.drive_combo.currentData() or "common")
        return SweepSettings(
            resource=self.resource_combo.currentText().strip(),
            amplitude_vrms=float(self.amplitude_spin.value()),
            load_ohms=int(self.load_spin.value()),
            drive_kind=drive,
            frequencies=self._frequencies(),
            min_dwell_s=float(self.dwell_spin.value()),
            periods_above=int(self.periods_high.value()),
            periods_below=int(self.periods_low.value()),
            monitor_channel=str(self.monitor_combo.currentData() or ""),
            frequency_hz=float(self._settings.frequency_hz),
            output_mode=str(self._settings.output_mode),
            # Edited in the dock, carried through here so a sweep plan keeps the
            # divider that is actually in front of the amplifier.
            attenuator_series_ohms=float(self._settings.attenuator_series_ohms),
            attenuator_shunt_ohms=float(self._settings.attenuator_shunt_ohms),
        )

    def _on_start(self) -> None:
        settings = self.settings()
        if not settings.frequencies:
            self.status_label.setText("Add at least one frequency before starting.")
            return
        self.startRequested.emit(settings)

    def _on_stop(self) -> None:
        self.stopRequested.emit()

    # -- state pushed from the window --------------------------------------

    def set_running(self, running: bool) -> None:
        self._running = bool(running)
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.frequency_edit.setEnabled(not running)
        self.drive_combo.setEnabled(not running)
        self.amplitude_spin.setEnabled(not running)

    def set_status(self, text: str, error: bool = False) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet("color: #b91c1c;" if error else "color: #1e293b;")

    def update_channels(self, channel_names: Sequence[str]) -> None:
        current = str(self.monitor_combo.currentData() or "")
        self.monitor_combo.clear()
        self.monitor_combo.addItem("(none)", "")
        for name in channel_names:
            self.monitor_combo.addItem(name, name)
        position = self.monitor_combo.findData(current)
        self.monitor_combo.setCurrentIndex(position if position >= 0 else 0)

    def set_monitor_channel(self, name: str) -> None:
        """Follow the reference channel chosen in the dock."""

        position = self.monitor_combo.findData(str(name or ""))
        if position < 0:
            self.monitor_combo.blockSignals(True)
            self.monitor_combo.addItem(name, name)
            self.monitor_combo.blockSignals(False)
            position = self.monitor_combo.findData(str(name or ""))
        blocker = QSignalBlocker(self.monitor_combo)
        self.monitor_combo.setCurrentIndex(position if position >= 0 else 0)
        del blocker


class LiveRecorderWindow(QMainWindow):
    """Serial acquisition window: configure, record, annotate and monitor."""

    def __init__(self, settings_path: Optional[Path] = None, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Dani-BCi Live Recorder")
        self.resize(self._default_window_size())

        self.store = SettingsStore(settings_path)
        self.settings = self.store.load()
        self.hardware = self.settings.hardware
        self.view = self.settings.view
        self.channel_count = 16 if self.hardware.board_channels > 8 else 8
        self.view.ensure_visibility(self.channel_count)

        self.worker: Optional[SerialWorker] = None
        self.connected = False
        self.streaming = False
        self.board_info: dict[str, str] = {}
        self.info_received = False
        self.profile_checked = False
        self.probe_attempts = 0

        self.apply_status: dict[int, str] = {}
        self.apply_summary = ""
        self._apply_plan: list[tuple[Optional[int], bytes, str]] = []
        self._apply_index = 0
        self._apply_current: Optional[tuple[Optional[int], str]] = None
        self._apply_failures = 0
        self._resume_streaming = False
        self._settings_dialog: Optional[HardwareSettingsDialog] = None

        self.sample_rate = self.hardware.sample_rate_hz
        self.sample_total = 0
        self.packet_count = 0
        self.packet_loss = 0
        self._measured_rate = 0.0
        self._rate_history: deque[tuple[float, int]] = deque(maxlen=256)

        self.raw_ring = RingBuffer(int(BUFFER_SECONDS * self.sample_rate), self.channel_count)
        self.display_ring = RingBuffer(int(BUFFER_SECONDS * self.sample_rate), self.channel_count)
        self.accel_ring = RingBuffer(int(BUFFER_SECONDS * 20.0), 3)
        self._last_aux_counts: Optional[np.ndarray] = None
        self._accel_zero_ticks = 0
        # Slowly tracked mean of the raw signal: the part below ~0.1 Hz that the
        # display filters remove. alpha comes from the sample rate in _rebuild_buffers.
        self._dc_state: Optional[np.ndarray] = None
        self._dc_alpha = 1.0
        self._zero_seconds = np.zeros(self.channel_count, dtype=np.float64)
        self._rail_seconds = np.zeros(self.channel_count, dtype=np.float64)
        self._channel_states: dict[int, str] = {}
        self._stalled_channels: set[int] = set()
        self._real_mask: Optional[np.ndarray] = None
        self._rate_downgrade_note = ""
        self.stream_health: Optional[tuple[int, int, int]] = None
        self.bluetooth_health: dict[str, str] = {}
        self.bluetooth_log: list[str] = []
        self._gap_events = 0
        self._last_gap: dict[str, str] = {}
        self._power_dialog = None
        self.power_state: dict[str, str] = {}
        self._benchmark_dialog = None
        self._benchmark_receiver: Optional[BenchmarkReceiver] = None
        self.benchmark_reports: list[str] = []
        # Board-side acquisition health: what the ESP32 actually produced, as
        # opposed to what this window managed to receive.
        self.board_dx: dict[str, str] = {}
        self.filter = StreamingFilter(self.sample_rate, self.channel_count)
        self.filter.configure(self.view.filter_state())

        self.recording: Optional[OpenBCIRecording] = None
        self.record_path: Optional[Path] = None
        self._record_channel_indices: list[int] = list(real_channel_indices(self.hardware.board_channels))
        self.record_start_sample = 0
        self.record_started_at: Optional[float] = None
        self.measurement = MeasurementInfo.from_dict(self.settings.measurement)
        self.annotations: list[Annotation] = []
        self._open_annotation: Optional[Annotation] = None
        self.last_recording: Optional[Path] = None

        # Automated function-generator sweep state.
        self.sweep_settings = SweepSettings.from_dict(self.settings.sweep)
        self.generator = UTG932E(self.sweep_settings.resource)
        self._generator_worker: Optional[GeneratorWorker] = None
        self._sweep_dialog: Optional[AutoSweepDialog] = None
        self._sweep_worker: Optional[AutoSweepWorker] = None
        self._sweep_plan: Optional[SweepPlan] = None
        self._sweep_steps: list[SweepStep] = []
        self._sweep_warnings: list[str] = []
        self._sweep_owns_recording = False

        self.monitors: list[SerialMonitorWindow] = []
        self._text_backlog: deque[tuple[str, str, str, str]] = deque(maxlen=4000)
        self._analyzer_window = None

        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(UI_INTERVAL_MS)
        self._ui_timer.timeout.connect(self._on_ui_tick)
        self._ui_ticks = 0
        self._flash_phase = False

        self._apply_timer = QTimer(self)
        self._apply_timer.setSingleShot(True)
        self._apply_timer.timeout.connect(self._on_apply_timeout)

        self._verify_timer = QTimer(self)
        self._verify_timer.setSingleShot(True)
        self._verify_timer.timeout.connect(self._on_verify_timeout)

        self._repair_timer = QTimer(self)
        self._repair_timer.setSingleShot(True)
        self._repair_timer.timeout.connect(self._on_repair_timeout)
        self._verify_lines: list[str] = []
        self._verify_pending = False
        self._pending_verify_hardware: Optional[HardwareSettings] = None
        self._repair_lines: list[str] = []
        self._repair_pending = False
        self.verify_result_text = ""
        self.verify_result_state = "info"
        # Last "[CFG]" readback, so the left panel can show what the board says
        # it is running rather than what the app remembers.
        self.board_report_text = ""

        self._autoconnect_timer = QTimer(self)
        self._autoconnect_timer.setInterval(AUTOCONNECT_RETRY_MS)
        self._autoconnect_timer.timeout.connect(self._autoconnect)
        self._autoconnect_idle = True

        self._probe_timer = QTimer(self)
        self._probe_timer.setSingleShot(True)
        self._probe_timer.timeout.connect(self._probe_board)

        self._info_timer = QTimer(self)
        self._info_timer.setSingleShot(True)
        self._info_timer.timeout.connect(self._on_info_timeout)

        self._open_watchdog = QTimer(self)
        self._open_watchdog.setSingleShot(True)
        self._open_watchdog.setInterval(OPEN_WATCHDOG_MS)
        self._open_watchdog.timeout.connect(self._on_open_watchdog)

        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setSingleShot(True)
        self._reconnect_timer.timeout.connect(self._on_reconnect)
        self._connect_attempts = 0
        self._open_watchdog_ms = OPEN_WATCHDOG_MS

        self._build_ui()
        self._build_menus()
        self.setStatusBar(QStatusBar())
        self._load_settings_into_ui()
        # The scanner runs from here on; the port list is filled asynchronously so
        # a slow (Bluetooth) enumeration can never freeze the window at startup.
        self._port_scanner = PortScanner(self)
        self._port_scanner.portsReady.connect(self._apply_ports)
        self._port_scanner.start()
        self.port_combo.addItem("DEMO  synthetic board (no hardware)", DEMO_PORT)
        self._refresh_ports()
        self._set_connection_state(False, "Not connected")
        self._update_hardware_labels()
        self._rebuild_channel_checks()
        labels = [channel_label(index) for index in range(self.channel_count)]
        self.time_panel.ensure_channels(labels)
        self.spectrum_panel.ensure_channels(labels)
        self._refresh_annotations()
        self._ui_timer.start()
        QTimer.singleShot(400, self._start_autoconnect)

    # -- construction ------------------------------------------------------

    @staticmethod
    def _default_window_size() -> "QSize":
        """Largest useful window that still fits the screen it opens on."""
        screen = QApplication.primaryScreen()
        if screen is None:
            return QSize(1280, 820)
        available = screen.availableGeometry()
        width = max(1024, min(available.width() - 40, 1560))
        height = max(640, min(available.height() - 70, 1000))
        return QSize(width, height)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter)

        self.time_panel = TimeSeriesPanel()
        splitter.addWidget(self.time_panel)

        right = QSplitter(Qt.Orientation.Vertical)
        self.spectrum_panel = SpectrumPanel()
        # The reference channel takes the bottom quarter instead of the
        # accelerometer: motion is still reported in the dock, while a stimulus
        # input that is 1000x larger than the EEG channels needs its own axes.
        self.reference_panel = ReferenceChannelPanel()
        right.addWidget(self.spectrum_panel)
        right.addWidget(self.reference_panel)
        right.setSizes([470, 260])
        splitter.addWidget(right)
        splitter.setSizes([560, 520])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        self._build_dock()

    def _build_dock(self) -> None:
        dock = QDockWidget("Device and session", self)
        dock.setObjectName("device_dock")
        dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)
        layout.addWidget(self._build_connection_group())
        # The generator sits directly under the connection block: it drives the
        # input under test, so it is part of setting the bench up, not a dialog
        # buried behind a menu.
        layout.addWidget(self._build_generator_group())
        layout.addWidget(self._build_recording_group())
        layout.addWidget(self._build_sd_card_group())
        layout.addWidget(self._build_annotation_group())
        layout.addWidget(self._build_channel_group())
        layout.addWidget(self._build_hardware_group())
        layout.addWidget(self._build_reference_group())
        layout.addWidget(self._build_filter_group())
        layout.addWidget(self._build_accel_group())
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(container)
        scroll.setMinimumWidth(320)
        scroll.setMaximumWidth(370)
        dock.setWidget(scroll)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        self.device_dock = dock

    def _build_connection_group(self) -> QWidget:
        group = QGroupBox("Connection")
        layout = QVBoxLayout(group)

        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(110)
        port_row.addWidget(self.port_combo, 1)
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._refresh_ports)
        port_row.addWidget(self.refresh_button)
        layout.addLayout(port_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud"))
        self.baud_combo = QComboBox()
        for baud in (115200, 230400, 460800, 921600, 2000000):
            self.baud_combo.addItem(str(baud), baud)
        baud_row.addWidget(self.baud_combo)
        baud_row.addStretch(1)
        layout.addLayout(baud_row)
        # Says what the baud actually does for the selected port. A Bluetooth SPP
        # port accepts the value and ignores it, which is worth stating once rather
        # than leaving the number to imply a limit that does not exist.
        self.baud_note_label = _shrinkable(QLabel(""))
        self.baud_note_label.setWordWrap(True)
        self.baud_note_label.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.baud_note_label)

        button_row = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.setCheckable(True)
        self.connect_button.toggled.connect(self._on_connect_toggled)
        button_row.addWidget(self.connect_button)
        self.stream_button = QPushButton("Start stream")
        self.stream_button.setCheckable(True)
        self.stream_button.setEnabled(False)
        self.stream_button.toggled.connect(self._on_stream_toggled)
        button_row.addWidget(self.stream_button)
        layout.addLayout(button_row)

        self.auto_connect_check = QCheckBox("Auto-connect")
        self.auto_connect_check.setToolTip(
            "On startup, connect to the port used last time (or COM5) without asking."
        )
        layout.addWidget(self.auto_connect_check)

        self.connection_label = _shrinkable(QLabel("Not connected"))
        layout.addWidget(self.connection_label)

        self.board_info_label = _shrinkable(QLabel("Board details appear here after connect."))
        self.board_info_label.setStyleSheet("color: #475569;")
        layout.addWidget(self.board_info_label)

        self.warning_label = _shrinkable(QLabel(""))
        self.warning_label.setStyleSheet("color: #b45309; font-weight: 600;")
        layout.addWidget(self.warning_label)
        return group

    def _build_generator_group(self) -> QWidget:
        """Always-visible manual control of the UTG932E stimulus.

        Two jobs: park the outputs (off, or held at 0 V so a shorted-input noise
        run sees a real ground), and apply a plain sine single-ended or
        differentially.
        """

        group = QGroupBox("Function generator")
        layout = QVBoxLayout(group)

        self.generator_status_label = _shrinkable(QLabel("Not connected"))
        self.generator_status_label.setStyleSheet("color: #475569;")
        layout.addWidget(self.generator_status_label)

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)

        grid.addWidget(QLabel("Output"), 0, 0)
        self.generator_mode_combo = QComboBox()
        self.generator_mode_combo.addItem("Off (outputs open)", OUTPUT_OFF)
        self.generator_mode_combo.addItem("0 V (short to GND)", OUTPUT_ZERO)
        self.generator_mode_combo.addItem("Sine - CH1 only", OUTPUT_SINGLE)
        self.generator_mode_combo.addItem("Sine - differential", OUTPUT_DIFFERENTIAL)
        self.generator_mode_combo.setToolTip(
            "Off opens both output relays, so the input is left floating.\n"
            "0 V keeps both outputs enabled at the generator's minimum amplitude: the output holds "
            "the node at 0 V through its ~50 ohm source impedance, which is the closest a generator "
            "gets to shorting the input to ground (it cannot go below 1 mVrms).\n"
            "Sine - CH1 only drives one output against ground.\n"
            "Sine - differential drives CH1 at 0 deg and CH2 at 180 deg, phase locked: the two "
            "outputs add, so the amplitude here is the differential level and each channel gets half."
        )
        grid.addWidget(self.generator_mode_combo, 0, 1)
        # Restore the remembered mode before anything else reads the combo: the
        # hint refresh writes the current selection back into the settings, so a
        # later restore would only ever see the default.
        saved_mode = self.generator_mode_combo.findData(str(self.sweep_settings.output_mode))
        if saved_mode >= 0:
            self.generator_mode_combo.setCurrentIndex(saved_mode)
        layout.addLayout(grid)

        self.generator_frequency_spin = QDoubleSpinBox()
        self.generator_frequency_spin.setRange(0.01, 100_000.0)
        self.generator_frequency_spin.setDecimals(3)
        self.generator_frequency_spin.setSuffix(" Hz")
        self.generator_frequency_spin.setValue(float(self.sweep_settings.frequency_hz))

        self.generator_amplitude_spin = QDoubleSpinBox()
        self.generator_amplitude_spin.setRange(MIN_AMPLITUDE_VRMS, 20.0)
        self.generator_amplitude_spin.setDecimals(4)
        self.generator_amplitude_spin.setSuffix(" Vrms")
        self.generator_amplitude_spin.setValue(max(float(self.sweep_settings.amplitude_vrms), MIN_AMPLITUDE_VRMS))

        for row, (label, widget) in enumerate(
            (
                ("Frequency", self.generator_frequency_spin),
                ("Amplitude", self.generator_amplitude_spin),
            ),
            start=1,
        ):
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(widget, row, 1)

        self.generator_mode_combo.currentIndexChanged.connect(self._refresh_generator_hint)
        self.generator_frequency_spin.valueChanged.connect(self._refresh_generator_hint)
        self.generator_amplitude_spin.valueChanged.connect(self._refresh_generator_hint)

        self.generator_hint_label = _shrinkable(QLabel(""))
        self.generator_hint_label.setStyleSheet("color: #334155; font-size: 11px;")
        layout.addWidget(self.generator_hint_label)

        button_row = QHBoxLayout()
        self.generator_apply_button = QPushButton("Apply")
        self.generator_apply_button.setToolTip("Program the selected output state and read it back")
        self.generator_apply_button.clicked.connect(self._apply_generator_output)
        button_row.addWidget(self.generator_apply_button)
        self.generator_zero_button = QPushButton("0 V")
        self.generator_zero_button.setToolTip("Park both outputs at 0 V (shorted-input measurement)")
        self.generator_zero_button.clicked.connect(self._generator_set_zero)
        button_row.addWidget(self.generator_zero_button)
        self.generator_off_button = QPushButton("Off")
        self.generator_off_button.setToolTip("Open both output relays")
        self.generator_off_button.clicked.connect(self._generator_set_off)
        button_row.addWidget(self.generator_off_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self._refresh_generator_hint()
        return group

    def _generator_mode(self) -> str:
        return str(self.generator_mode_combo.currentData() or OUTPUT_OFF)

    def _generator_output(self) -> GeneratorOutput:
        return GeneratorOutput(
            mode=self._generator_mode(),
            frequency_hz=float(self.generator_frequency_spin.value()),
            amplitude_vrms=float(self.generator_amplitude_spin.value()),
            load_ohms=int(self.sweep_settings.load_ohms),
        )

    def _refresh_generator_hint(self) -> None:
        """Show the amplitude each output actually gets, and the divider result."""

        mode = self._generator_mode()
        sine = mode in (OUTPUT_SINGLE, OUTPUT_DIFFERENTIAL)
        for widget in (self.generator_frequency_spin, self.generator_amplitude_spin):
            widget.setEnabled(sine)
        self.generator_amplitude_spin.setSuffix(" Vrms diff" if mode == OUTPUT_DIFFERENTIAL else " Vrms")

        amplitude = float(self.generator_amplitude_spin.value())
        frequency = float(self.generator_frequency_spin.value())
        if mode == OUTPUT_OFF:
            hint = "Outputs open: the input is left floating."
        elif mode == OUTPUT_ZERO:
            hint = (
                f"Both outputs enabled at 0 V (generator minimum {MIN_AMPLITUDE_VRMS * 1e3:g} mVrms "
                f"through ~50 ohm)."
            )
        elif mode == OUTPUT_DIFFERENTIAL:
            each = per_channel_amplitude_vrms(amplitude, True)
            hint = (
                f"CH1 {each:g} Vrms at 0 deg + CH2 {each:g} Vrms at 180 deg "
                f"= {amplitude:g} Vrms differential at {frequency:g} Hz."
            )
        else:
            hint = f"CH1 {amplitude:g} Vrms at {frequency:g} Hz, CH2 off."
        self.generator_hint_label.setText(hint)
        self._store_generator_settings()

    def _store_generator_settings(self) -> None:
        """Keep the plan object and the saved settings in step with the widgets."""

        settings = self.sweep_settings
        settings.output_mode = self._generator_mode()
        settings.frequency_hz = float(self.generator_frequency_spin.value())
        settings.amplitude_vrms = float(self.generator_amplitude_spin.value())
        self.settings.sweep = settings.as_dict()

    def _generator_sync_from_settings(self) -> None:
        """Mirror a plan that changed elsewhere (the sweep dialog) into the dock."""

        settings = self.sweep_settings
        for widget, value in (
            (self.generator_frequency_spin, settings.frequency_hz),
            (self.generator_amplitude_spin, settings.amplitude_vrms),
        ):
            widget.blockSignals(True)
            widget.setValue(float(value))
            widget.blockSignals(False)
        position = self.generator_mode_combo.findData(settings.output_mode)
        if position >= 0:
            self.generator_mode_combo.blockSignals(True)
            self.generator_mode_combo.setCurrentIndex(position)
            self.generator_mode_combo.blockSignals(False)
        self._refresh_generator_hint()

    def _set_generator_busy(self, busy: bool) -> None:
        for widget in (
            self.generator_apply_button,
            self.generator_zero_button,
            self.generator_off_button,
            self.generator_mode_combo,
        ):
            widget.setEnabled(not busy)

    def _apply_generator_output(self) -> None:
        self._start_generator_command(self._generator_output())

    def _generator_set_zero(self) -> None:
        position = self.generator_mode_combo.findData(OUTPUT_ZERO)
        if position >= 0:
            self.generator_mode_combo.setCurrentIndex(position)
        self._start_generator_command(self._generator_output())

    def _generator_set_off(self) -> None:
        position = self.generator_mode_combo.findData(OUTPUT_OFF)
        if position >= 0:
            self.generator_mode_combo.setCurrentIndex(position)
        self._start_generator_command(self._generator_output())

    def _start_generator_command(self, output: GeneratorOutput) -> None:
        if self._generator_worker is not None:
            self.generator_status_label.setText("Busy: the previous generator command is still running.")
            self._set_sweep_status("The generator is still busy with the previous command.", error=True)
            return
        if self._sweep_worker is not None:
            self.generator_status_label.setText("Busy: an automated sweep is driving the generator.")
            self._set_sweep_status(
                "An automated sweep is driving the generator; stop it before using the manual controls.",
                error=True,
            )
            return
        self._store_generator_settings()
        self._save_settings()
        self._set_generator_busy(True)
        self.generator_status_label.setText(f"Applying: {output.describe()}")
        worker = GeneratorWorker(self.generator, output, self.sweep_settings.resource, self)
        worker.applied.connect(self._on_generator_applied)
        worker.failed.connect(self._on_generator_failed)
        worker.progress.connect(lambda text: self.generator_status_label.setText(text))
        worker.finished.connect(self._on_generator_finished)
        self._generator_worker = worker
        worker.start()

    def _on_generator_applied(self, output: GeneratorOutput, notes: list) -> None:
        self.generator_status_label.setText(f"Applied: {output.describe()}")
        self.generator_status_label.setStyleSheet("color: #15803d;")
        self._log("in", f"[generator] {output.describe()}", "info")
        for note in notes:
            self._log("in", f"[generator] {note}", "info")

    def _on_generator_failed(self, message: str) -> None:
        self.generator_status_label.setText(f"Generator error: {message}")
        self.generator_status_label.setStyleSheet("color: #b91c1c; font-weight: 600;")
        self._log("in", f"[generator] {message}", "error")

    def _on_generator_finished(self) -> None:
        worker = self._generator_worker
        self._generator_worker = None
        if worker is not None:
            worker.wait(2000)
        self._set_generator_busy(False)

    def _build_sd_card_group(self) -> QWidget:
        """SD card telemetry, naming and standalone recording control.

        The board can record on its own (record button or a host command), so this
        panel is a view onto state that also changes without the GUI: it reacts to
        the board's own ``EVENT=`` reports and never assumes. It deliberately does
        not poll on a timer - reading the card's free space walks its file system,
        and that is slow enough to be visible as a jump in the traces.
        """

        group = QGroupBox("SD card")
        layout = QVBoxLayout(group)

        self.sd_status_label = _shrinkable(QLabel("Not checked yet - connect the board and press Refresh."))
        self.sd_status_label.setWordWrap(True)
        self.sd_status_label.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.sd_status_label)

        self.sd_state_label = _shrinkable(QLabel(""))
        self.sd_state_label.setWordWrap(True)
        self.sd_state_label.setStyleSheet("color: #1e293b; font-size: 11px; font-weight: 600;")
        layout.addWidget(self.sd_state_label)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name"))
        self.sd_name_edit = QLineEdit()
        self.sd_name_edit.setPlaceholderText("sd session name")
        self.sd_name_edit.setToolTip(
            "Stored on the card and written into the recording header. The record button uses it "
            "when the board records without a host."
        )
        self.sd_name_edit.editingFinished.connect(self._send_sd_name)
        name_row.addWidget(self.sd_name_edit, 1)
        layout.addLayout(name_row)

        button_row = QHBoxLayout()
        self.sd_refresh_button = QPushButton("Refresh")
        self.sd_refresh_button.setToolTip(
            "Read the card's state and free space on demand. Reading free space walks the "
            "card's file system, so it is never done automatically during a measurement."
        )
        self.sd_refresh_button.clicked.connect(self.refresh_sd_status)
        button_row.addWidget(self.sd_refresh_button)
        self.sd_save_button = QPushButton("Save settings")
        self.sd_save_button.setToolTip(
            "Program the converters, then store that configuration on the card so the record "
            "button can apply it without a host."
        )
        self.sd_save_button.clicked.connect(self.save_settings_to_sd)
        button_row.addWidget(self.sd_save_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        record_row = QHBoxLayout()
        self.sd_start_button = QPushButton("Start")
        self.sd_start_button.setToolTip(
            "Start recording on the card; the live link keeps streaming and the live recording "
            "starts too, so both captures line up"
        )
        self.sd_start_button.clicked.connect(self.start_sd_recording)
        record_row.addWidget(self.sd_start_button)
        self.sd_stop_button = QPushButton("Stop")
        self.sd_stop_button.setToolTip("Stop the card recording and close the file")
        self.sd_stop_button.clicked.connect(self.stop_sd_recording)
        self.sd_stop_button.setEnabled(False)
        record_row.addWidget(self.sd_stop_button)
        record_row.addStretch(1)
        layout.addLayout(record_row)

        self.sd_scope_label = _shrinkable(QLabel(""))
        self.sd_scope_label.setWordWrap(True)
        self.sd_scope_label.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.sd_scope_label)

        # Live view of whether the card and the radio are keeping up. This is the
        # answer to "are both recordings fine at once", in numbers rather than by
        # watching the stream.
        self.sd_health_label = _shrinkable(QLabel(""))
        self.sd_health_label.setWordWrap(True)
        self.sd_health_label.setStyleSheet("color: #475569; font-size: 11px;")
        self.sd_health_label.setToolTip(
            "Card write headroom and the longest single card transaction, so a card that "
            "cannot keep up is visible before it drops samples."
        )
        layout.addWidget(self.sd_health_label)

        self._sd_state: dict[str, str] = {}
        self._sd_pending = False
        self._sd_save_after_apply = False
        # Which action the operator just asked for, so the reply can be turned into
        # a definite "started" / "failed" instead of a generic idle line.
        self._sd_pending_action: str | None = None
        # Set while an action message is showing so a later report cannot overwrite
        # "programming, then storing..." - but only until the deadline, so a board
        # that never confirms cannot leave the panel stuck.
        self._sd_action_hold = False
        self._sd_hold_until = 0.0
        # The card is read only when the operator asks (Refresh, or a start/stop
        # they initiated). SD.usedBytes() walks the FAT, which is slow enough to be
        # seen as a jump in the data, so nothing here runs on a timer.
        # A board that never answers (an older firmware, or the demo port) must not
        # leave the panel stuck on "Reading the card...".
        self._sd_reply_timer = QTimer(self)
        self._sd_reply_timer.setSingleShot(True)
        self._sd_reply_timer.setInterval(2500)
        self._sd_reply_timer.timeout.connect(self._on_sd_reply_timeout)
        self._refresh_sd_scope()
        return group

    def _on_sd_reply_timeout(self) -> None:
        if not self._sd_pending:
            return
        self._sd_pending = False
        pending = self._sd_pending_action
        self._sd_pending_action = None
        self._sd_action_hold = False
        self._sd_hold_until = 0.0
        if pending in ("start", "stop"):
            self.sd_state_label.setText(
                f"The board did not confirm the {pending} request. Check the serial log - "
                "an older firmware does not report card recording state."
            )
            self.sd_state_label.setStyleSheet("color: #b91c1c; font-size: 11px; font-weight: 600;")
            return
        self.sd_status_label.setText(
            "No reply to the card query. The firmware may predate SD support, or the card is not "
            "answering. Press Refresh to try again."
        )

    def _refresh_sd_scope(self) -> None:
        """Say which channels the card recording will contain."""

        label = getattr(self, "sd_scope_label", None)
        if label is None:
            return
        enabled = [
            index + 1
            for index in real_channel_indices(self.hardware.board_channels)
            if index < len(self.hardware.channels) and self.hardware.channels[index].active
        ]
        if not enabled:
            label.setText("Every channel is switched off, so the card would record nothing.")
            return
        label.setText(
            f"Records {len(enabled)} channel(s): "
            + ", ".join(str(number) for number in enabled)
        )

    def _send_sd_command(self, payload: bytes) -> bool:
        if not self.connected:
            self.sd_state_label.setText("Connect the board before controlling the card.")
            return False
        return self._send(payload)

    def _send_sd_name(self) -> None:
        if not self.connected:
            return
        name = self.sd_name_edit.text().strip()
        if not name:
            return
        self._send(f":n{name};".encode("ascii"))

    def refresh_sd_status(self) -> None:
        """Ask the board for its card state; the reply arrives on the serial link.

        This is the only thing in the window that reads the card's free space, and
        it runs only when the operator asks for it: the FAT walk behind
        ``SD.usedBytes()`` costs enough time to show up as a jump in the traces.
        """

        if not self.connected:
            self.sd_status_label.setText("Connect the board to read the card.")
            self.sd_state_label.setText("")
            return
        self._sd_pending = True
        self._sd_pending_action = None
        self.sd_status_label.setText("Reading the card...")
        self._sd_reply_timer.start()
        self._send(b":s")

    def start_sd_recording(self) -> None:
        """Start the card recording and, if needed, the matching live recording.

        The live recording is opened *before* the card one. Doing it the other way
        round left the card file holding however long the ``EVENT=start`` reply
        took to arrive over the link - 1.5 s over Bluetooth - so the two files
        began at completely different points of the stream. Card-first also means
        the card never opens on a stream that is not yet running.
        """

        if not self.connected:
            self.sd_state_label.setText("Connect the board before controlling the card.")
            return
        # Start the live capture first so both files begin at nearly the same
        # stream position; the reply then only has to confirm the card file.
        if self.recording is None:
            self.start_recording()
        self._sd_pending_action = "start"
        self._sd_action_hold = True
        self._sd_hold_until = time.monotonic() + 8.0
        if self._send_sd_command(b":b"):
            self.sd_state_label.setText("Starting the card recording...")
            self.sd_state_label.setStyleSheet("color: #475569; font-size: 11px; font-weight: 600;")
            # The board replies with an EVENT=start line once the file is open. Until
            # it does, keep asking, so a silent board cannot leave this blank.
            self._arm_sd_reply_timer()
        else:
            self._sd_pending_action = None
            self._sd_action_hold = False
            self._sd_hold_until = 0.0

    def stop_sd_recording(self) -> None:
        self._sd_pending_action = "stop"
        self._sd_action_hold = True
        self._sd_hold_until = time.monotonic() + 8.0
        if self._send_sd_command(b":e"):
            self.sd_state_label.setText("Stopping the card recording...")
            self.sd_state_label.setStyleSheet("color: #475569; font-size: 11px; font-weight: 600;")
            self._arm_sd_reply_timer()

    def _arm_sd_reply_timer(self) -> None:
        self._sd_pending = True
        self._sd_reply_timer.start()

    def _on_sd_action_reply(self, event: str, recording: bool) -> None:
        """Turn an ``EVENT=`` reply into a definite state message.

        A silent card start used to be indistinguishable from a working one, which
        is exactly what made "start recording" look like it had frozen the stream.
        """

        action = self._sd_pending_action
        self._sd_pending_action = None
        self._sd_action_hold = False
        self._sd_hold_until = 0.0
        if event == "start" and recording:
            self.sd_state_label.setText(f"Recording to the card: {self._sd_state.get('FILE', '?')}")
            self.sd_state_label.setStyleSheet("color: #15803d; font-size: 11px; font-weight: 600;")
            # Only the physical button can start a card recording without the
            # window having opened its own file first, so this is the fallback that
            # keeps a button-started pair comparable.
            if self.recording is None and self.connected:
                self.start_recording()
            # Both files exist now, so stamp the same instant into each: this is
            # what makes the offset between them measurable instead of guessed.
            self.send_recording_marker()
            return
        if event == "stop":
            self.sd_state_label.setText(
                f"Card recording stopped: {self._sd_state.get('FILE', '?')}  |  "
                f"{self._sd_state.get('SAMPLES', '0')} samples written"
            )
            self.sd_state_label.setStyleSheet("color: #475569; font-size: 11px; font-weight: 600;")
            # Both captures were started together, so they stop together too and
            # stay directly comparable.
            if self.recording is not None:
                self.stop_recording()
            return
        if event == "timeout":
            self.sd_state_label.setText(
                f"Card recording finished its time limit: {self._sd_state.get('FILE', '?')}"
            )
            self.sd_state_label.setStyleSheet("color: #475569; font-size: 11px; font-weight: 600;")
            if self.recording is not None:
                self.stop_recording()
            return
        if event == "fail":
            self.sd_state_label.setText(
                "The board refused to start the card recording - see the serial log for the reason."
            )
            self.sd_state_label.setStyleSheet("color: #b91c1c; font-size: 11px; font-weight: 600;")
            return
        if event == "busy":
            self.sd_state_label.setText("The card is already recording; press Stop first.")
            self.sd_state_label.setStyleSheet("color: #b45309; font-size: 11px; font-weight: 600;")
            return
        if event == "saved":
            self.sd_state_label.setText("Settings confirmed as written to the card.")
            self.sd_state_label.setStyleSheet("color: #15803d; font-size: 11px; font-weight: 600;")
            return
        if action == "start":
            self.sd_state_label.setText(
                "The card reported idle after a start request - the recording did not begin."
            )
            self.sd_state_label.setStyleSheet("color: #b91c1c; font-size: 11px; font-weight: 600;")

    def save_settings_to_sd(self) -> None:
        """Program the board first, then persist that exact configuration on the card."""

        if not self.connected:
            self.sd_state_label.setText("Connect the board before saving settings.")
            return
        if self.recording is not None:
            self.sd_state_label.setText("Stop the live recording before changing hardware settings.")
            return
        # The card must hold what the converters actually run, so the apply has to
        # finish before the file is written. _finish_apply() does the second half.
        self._sd_save_after_apply = True
        self._sd_action_hold = True
        self._sd_hold_until = time.monotonic() + 8.0
        self._sd_pending_action = "save"
        self.sd_state_label.setText("Programming the board, then storing the settings on the card...")
        if not self.apply_hardware():
            # The apply was refused (busy, or the link budget check failed). Say so
            # instead of leaving the panel on a message that will never resolve, and
            # still store whatever the board currently runs so the button does
            # something useful.
            self._sd_save_after_apply = False
            self._send_sd_command(b":w")
            self.sd_state_label.setText(
                "Could not reprogram the board just now; storing its current settings on the card instead."
            )

    def _on_sd_status_line(self, text: str) -> None:
        """Parse one ``[SD] ... DONE=1`` line."""

        if "DONE=1" not in text:
            return
        fields = dict(
            token.split("=", 1) for token in text[len("[SD]"):].split() if "=" in token
        )
        self._sd_state = fields
        self._sd_pending = False
        self._sd_reply_timer.stop()
        if self._sd_action_hold and time.monotonic() >= self._sd_hold_until:
            self._sd_action_hold = False
        present = fields.get("PRESENT") == "1"
        recording = fields.get("RECORDING") == "1"
        # "status" is the explicit Refresh answer; only real state changes get
        # treated as an action reply.
        event = fields.get("EVENT", "-")
        action_event = event not in ("-", "", "status")
        if action_event:
            self._on_sd_action_reply(event, recording)
        if not present:
            self.sd_status_label.setText("No card detected. Insert one and press Refresh.")
            if not action_event:
                self.sd_state_label.setText("not recording" if not recording else "recording?")
            self.sd_state_label.setStyleSheet("color: #b91c1c; font-size: 11px; font-weight: 600;")
            return

        def number(key: str) -> int:
            try:
                return int(fields.get(key, "0") or 0)
            except ValueError:
                return 0

        free_mb = number("FREE_MB")
        total_mb = number("TOTAL_MB")
        estimate = number("EST_SEC")
        channels = number("CHANNELS")
        rate = number("RATE")
        self.sd_status_label.setText(
            f"{free_mb / 1024.0:.2f} GB free of {total_mb / 1024.0:.2f} GB "
            f"({free_mb * 100 // max(total_mb, 1)}%)  |  {channels} ch at {rate} Hz  |  "
            f"~{format_duration(float(estimate))} of recording space"
            f"{'  |  session ' + fields.get('NAME', '') if fields.get('NAME') else ''}"
        )
        if recording:
            written = number("SAMPLES")
            dropped = number("DROPPED")
            line = f"Recording to {fields.get('FILE', '?')} - {written:,} samples"
            if dropped:
                line += f", {dropped:,} dropped"
            self.sd_state_label.setText(line)
            self.sd_state_label.setStyleSheet("color: #15803d; font-size: 11px; font-weight: 600;")
            self._sd_action_hold = False
            self._sd_hold_until = 0.0
        elif action_event:
            # An action event already produced its own message; a generic "Idle"
            # here would erase the confirmation the operator is waiting for.
            pass
        elif not self._sd_action_hold:
            # A report must not erase an action message such as "programming, then
            # storing the settings".
            self.sd_state_label.setText("Idle - not recording to the card.")
            self.sd_state_label.setStyleSheet("color: #475569; font-size: 11px; font-weight: 600;")
        self._refresh_sd_health(fields, number)
        name = str(fields.get("NAME", "") or "")
        if name and not self.sd_name_edit.text().strip():
            self.sd_name_edit.setText(name)
        self.sd_start_button.setEnabled(not recording)
        self.sd_stop_button.setEnabled(recording)

    def _refresh_sd_health(self, fields: dict[str, str], number) -> None:
        """Summarise the card write path: headroom, peak queue and longest stall.

        A card that is keeping up shows a queue peak well under the capacity and a
        write rate below the SPI clock; a card that is not shows a peak at the
        capacity and a rising drop count. Reporting both makes the difference
        visible instead of guessed at.
        """

        label = getattr(self, "sd_health_label", None)
        if label is None:
            return
        if "QPEAK" not in fields:
            # An older firmware reports none of this.
            label.setText("")
            return
        capacity = 512
        peak = number("QPEAK")
        write_ms = number("WRITE_MS")
        rate_kb = number("RATE_KB")
        spi_khz = number("SPIKHZ")
        dropped = number("DROPPED")
        parts = [
            f"Card: {rate_kb} kB/s, longest write {write_ms} ms, queue peak "
            f"{peak}/{capacity}, SPI {spi_khz} kHz"
        ]
        if dropped:
            parts.append(f"{dropped:,} sample(s) dropped - the card cannot keep up at this rate")
            colour = "#b91c1c"
        elif peak >= capacity * 0.75:
            parts.append("the queue is nearly full - little margin")
            colour = "#b45309"
        else:
            colour = "#475569"
        label.setText("  |  ".join(parts))
        label.setStyleSheet(f"color: {colour}; font-size: 11px;")

    def open_name_dialog(self) -> None:
        """Name the measurement with dropdowns instead of typing.

        The reference mode is part of the name, so picking a name also drives the
        hardware: choosing "true differential" here selects that reference mode
        and writes it to the board if one is connected.
        """

        # Start from the bias state the board is actually in, so the recorded
        # name cannot claim "bias off" while every channel has bias enabled.
        seed = MeasurementInfo.from_dict(self.measurement.as_dict())
        seed.bias_enabled = self._hardware_bias_active()
        dialog = MeasurementNameDialog(seed, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.measurement = dialog.measurement()
        self.session_name_edit.setText(dialog.name_stem())
        self.settings.session_name = self.session_name_edit.text()
        self.settings.measurement = self.measurement.as_dict()
        self._save_settings()
        self._apply_measurement_reference(self.measurement.reference_mode)
        self._update_measurement_label()

    def _hardware_bias_active(self) -> bool:
        """True when any real channel currently has bias connected."""

        indices = real_channel_indices(self.hardware.board_channels)
        for index in indices:
            if index < len(self.hardware.channels) and self.hardware.channels[index].bias:
                return True
        return False

    def _apply_measurement_reference(self, mode: str) -> None:
        """Record the reference the operator *named* - without touching hardware.

        The name is documentation and the hardware is state: forcing one from the
        other meant that naming a run could silently reprogram both converters and
        wipe the per-input reference of a mixed measurement. Changing the hardware
        therefore no longer rewrites the name either, and this only refreshes the
        label so a mismatch is visible instead of hidden.
        """

        if mode not in REFERENCE_MODE_CODES:
            return
        self.measurement.reference_mode = mode
        self._save_settings()
        self._update_measurement_label()

    def _refresh_session_name_reference(self) -> None:
        """Keep the measurement description honest after a hardware change.

        Deliberately does **not** rewrite the session name: the operator typed it,
        and renaming a run because a converter was switched would make the file
        name disagree with what they called the measurement. The live label and
        the recording metadata both carry the reference the board is actually in.
        """

        self.measurement.reference_mode = self.hardware.normalised_reference_mode()
        self._update_measurement_label()

    def _update_measurement_label(self) -> None:
        label = getattr(self, "measurement_label", None)
        if label is None:
            return
        info = self.measurement.normalised()
        info.reference_mode = self.hardware.normalised_reference_mode()
        text = "  |  ".join(info.describe_lines())
        note = self._reference_mismatch_note()
        if note:
            text = f"{text}   |   {note}"
        if not self.session_name_edit.text().strip():
            text = "Name this measurement with the Name... button.   |   " + text
        label.setText(text)

    def _reference_mismatch_note(self) -> str:
        """Say so when the name claims a different reference than the board runs."""

        named = str(self.measurement.reference_mode or "")
        short = REFERENCE_MODE_SHORT_LABELS
        if named in REFERENCE_MODE_CODES and named != self.hardware.normalised_reference_mode():
            return (
                f"name says {short.get(named, named)} but the board runs "
                f"{self.hardware.reference_mode_label} - the recording stores the board state"
            )
        if self.hardware.chip_modes_differ:
            return f"mixed reference: {self.hardware.reference_summary()}"
        return ""

    def _build_recording_group(self) -> QWidget:
        group = QGroupBox("Recording")
        layout = QVBoxLayout(group)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Session"))
        self.session_name_edit = QLineEdit()
        self.session_name_edit.setPlaceholderText("session name")
        self.session_name_edit.setToolTip("Used for the CSV file name")
        name_row.addWidget(self.session_name_edit, 1)
        self.name_button = QPushButton("Name...")
        self.name_button.setToolTip("Build the name from dropdowns")
        self.name_button.clicked.connect(self.open_name_dialog)
        name_row.addWidget(self.name_button)
        layout.addLayout(name_row)

        self.measurement_label = _shrinkable(QLabel(""))
        self.measurement_label.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.measurement_label)

        dir_row = QHBoxLayout()
        self.output_dir_label = QLabel("")
        self.output_dir_label.setWordWrap(True)
        dir_row.addWidget(self.output_dir_label, 1)
        browse = QPushButton("...")
        browse.setMaximumWidth(34)
        browse.clicked.connect(self._choose_output_dir)
        dir_row.addWidget(browse)
        layout.addLayout(dir_row)

        self.record_visible_check = QCheckBox("Only checked channels")
        self.record_visible_check.setToolTip("Write only the channels ticked in the 'Channels' group")
        layout.addWidget(self.record_visible_check)

        self.record_scope_label = _shrinkable(QLabel(""))
        self.record_scope_label.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.record_scope_label)

        self.record_button = QPushButton("Start recording")
        self.record_button.setCheckable(True)
        self.record_button.setEnabled(False)
        self.record_button.toggled.connect(self._on_record_toggled)
        layout.addWidget(self.record_button)

        self.record_time_label = QLabel("00:00.00")
        self.record_time_label.setStyleSheet("font-family: Consolas; font-size: 14px;")
        layout.addWidget(self.record_time_label)

        self.record_stats_label = QLabel("no recording yet")
        self.record_stats_label.setStyleSheet("color: #475569;")
        self.record_stats_label.setWordWrap(True)
        layout.addWidget(self.record_stats_label)

        self.analyze_button = QPushButton("Analyze last")
        self.analyze_button.setEnabled(False)
        self.analyze_button.clicked.connect(lambda: self.open_analyzer(load_last=True))
        layout.addWidget(self.analyze_button)
        return group

    def _build_annotation_group(self) -> QWidget:
        group = QGroupBox("Sections")
        layout = QVBoxLayout(group)

        self.current_section_label = QLabel("No section open")
        self.current_section_label.setWordWrap(True)
        self.current_section_label.setStyleSheet("color: #b45309; font-weight: 600;")
        layout.addWidget(self.current_section_label)

        self.annotation_list = QListWidget()
        self.annotation_list.setMaximumHeight(120)
        self.annotation_list.itemDoubleClicked.connect(lambda _: self.analyze_section())
        layout.addWidget(self.annotation_list)

        row = QHBoxLayout()
        self.add_section_button = QPushButton("Start section")
        self.add_section_button.setToolTip("Name a new section starting at the current recording time")
        self.add_section_button.clicked.connect(self.add_annotation)
        row.addWidget(self.add_section_button)
        self.end_section_button = QPushButton("End")
        self.end_section_button.setToolTip("Close the open section at the current recording time")
        self.end_section_button.clicked.connect(self.end_annotation)
        row.addWidget(self.end_section_button)
        layout.addLayout(row)

        row2 = QHBoxLayout()
        self.edit_section_button = QPushButton("Edit")
        self.edit_section_button.clicked.connect(self.edit_annotation)
        row2.addWidget(self.edit_section_button)
        self.remove_section_button = QPushButton("Delete")
        self.remove_section_button.clicked.connect(self.remove_annotation)
        row2.addWidget(self.remove_section_button)
        self.analyze_section_button = QPushButton("Analyze")
        self.analyze_section_button.setToolTip("Open the analyzer on exactly this section's time range")
        self.analyze_section_button.clicked.connect(self.analyze_section)
        row2.addWidget(self.analyze_section_button)
        layout.addLayout(row2)
        return group

    def _build_channel_group(self) -> QWidget:
        group = QGroupBox("Channels")
        layout = QVBoxLayout(group)
        self.channel_check_container = QWidget()
        self.channel_check_layout = QGridLayout(self.channel_check_container)
        self.channel_check_layout.setContentsMargins(0, 0, 0, 0)
        self.channel_check_layout.setSpacing(4)
        layout.addWidget(self.channel_check_container)

        row = QHBoxLayout()
        all_button = QPushButton("All")
        all_button.clicked.connect(lambda: self._set_all_channels(True))
        row.addWidget(all_button)
        none_button = QPushButton("None")
        none_button.clicked.connect(lambda: self._set_all_channels(False))
        row.addWidget(none_button)
        layout.addLayout(row)

        self.channel_warning_label = _shrinkable(QLabel(""))
        self.channel_warning_label.setStyleSheet("color: #b91c1c; font-size: 11px; font-weight: 600;")
        self.channel_warning_label.setVisible(False)
        layout.addWidget(self.channel_warning_label)

        self.channel_checkboxes: list[QCheckBox] = []
        return group

    def _build_hardware_group(self) -> QWidget:
        group = QGroupBox("Hardware settings")
        layout = QVBoxLayout(group)

        self.board_label = _shrinkable(QLabel(""))
        layout.addWidget(self.board_label)

        button_row = QHBoxLayout()
        self.hardware_button = QPushButton("Hardware settings...")
        self.hardware_button.setToolTip("Per-channel gain/input/bias/SRB2, reference mode, rate, decimation, SPI clock")
        self.hardware_button.clicked.connect(self.open_hardware_settings)
        button_row.addWidget(self.hardware_button)
        self.reapply_button = QPushButton("Re-apply saved")
        self.reapply_button.setToolTip("Send the settings held in this app again")
        self.reapply_button.clicked.connect(lambda: self.apply_hardware())
        button_row.addWidget(self.reapply_button)
        layout.addLayout(button_row)

        read_row = QHBoxLayout()
        self.read_board_button = QPushButton("Read board state")
        self.read_board_button.setToolTip("Read the ADS1299 registers back and report what the board runs")
        self.read_board_button.clicked.connect(self.read_board_state)
        read_row.addWidget(self.read_board_button)
        self.repair_button = QPushButton("Repair and verify")
        self.repair_button.setToolTip("Re-program both converters from the board's cached state, then check the readback")
        self.repair_button.clicked.connect(self.repair_board)
        read_row.addWidget(self.repair_button)
        layout.addLayout(read_row)

        self.board_report_label = _shrinkable(QLabel("Board reports: connect and press 'Read board state'."))
        self.board_report_label.setWordWrap(True)
        self.board_report_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.board_report_label.setStyleSheet("color: #1e293b; font-size: 11px; font-family: Consolas;")
        layout.addWidget(self.board_report_label)

        # Everything below only *shows* the configuration; the settings window is
        # the single place where anything can be changed.
        self.reference_note = _shrinkable(QLabel(""))
        self.reference_note.setWordWrap(True)
        self.reference_note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.reference_note)

        self.apply_status_label = _shrinkable(QLabel(""))
        self.apply_status_label.setStyleSheet("color: #475569;")
        layout.addWidget(self.apply_status_label)
        return group

    def _build_filter_group(self) -> QWidget:
        group = QGroupBox("Display filters")
        layout = QGridLayout(group)
        self.notch_50_check = QCheckBox("50 Hz notch")
        self.notch_50_check.toggled.connect(self._on_filter_changed)
        layout.addWidget(self.notch_50_check, 0, 0)
        self.notch_100_check = QCheckBox("100 Hz notch")
        self.notch_100_check.toggled.connect(self._on_filter_changed)
        layout.addWidget(self.notch_100_check, 0, 1)
        layout.addWidget(QLabel("Notch Q"), 1, 0)
        self.notch_q_spin = QDoubleSpinBox()
        self.notch_q_spin.setRange(2.0, 100.0)
        self.notch_q_spin.setDecimals(1)
        self.notch_q_spin.setValue(30.0)
        self.notch_q_spin.valueChanged.connect(self._on_filter_changed)
        layout.addWidget(self.notch_q_spin, 1, 1)
        layout.addWidget(QLabel("High-pass"), 2, 0)
        self.high_pass_combo = QComboBox()
        for value, label in HIGH_PASS_CHOICES:
            self.high_pass_combo.addItem(label, value)
        self.high_pass_combo.currentIndexChanged.connect(self._on_filter_changed)
        layout.addWidget(self.high_pass_combo, 2, 1)
        layout.addWidget(QLabel("Low-pass"), 3, 0)
        self.low_pass_combo = QComboBox()
        for value, label in LOW_PASS_CHOICES:
            self.low_pass_combo.addItem(label, value)
        self.low_pass_combo.currentIndexChanged.connect(self._on_filter_changed)
        layout.addWidget(self.low_pass_combo, 3, 1)
        return group
    # -- menus and settings ------------------------------------------------

    def _build_menus(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        analyzer_action = QAction("Open analyzer", self)
        analyzer_action.triggered.connect(lambda: self.open_analyzer(load_last=False))
        file_menu.addAction(analyzer_action)
        analyze_last_action = QAction("Analyze last recording", self)
        analyze_last_action.triggered.connect(lambda: self.open_analyzer(load_last=True))
        file_menu.addAction(analyze_last_action)
        file_menu.addSeparator()
        quit_action = QAction("Quit", self)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        device_menu = self.menuBar().addMenu("Device")
        monitor_action = QAction("Serial monitor...", self)
        monitor_action.triggered.connect(self.open_monitor)
        device_menu.addAction(monitor_action)
        bluetooth_action = QAction("Bluetooth report...", self)
        bluetooth_action.triggered.connect(self.show_bluetooth_report)
        device_menu.addAction(bluetooth_action)
        hardware_action = QAction("Hardware settings...", self)
        hardware_action.triggered.connect(self.open_hardware_settings)
        device_menu.addAction(hardware_action)
        power_action = QAction("Power control...", self)
        power_action.setToolTip("Switch converters, accelerometer, card, radio and sleep modes for current measurement")
        power_action.triggered.connect(self.open_power_control)
        device_menu.addAction(power_action)
        defaults_action = QAction("Restore board defaults", self)
        defaults_action.triggered.connect(self.restore_board_defaults)
        device_menu.addAction(defaults_action)
        device_menu.addSeparator()
        refresh_action = QAction("Refresh ports", self)
        refresh_action.triggered.connect(self._refresh_ports)
        device_menu.addAction(refresh_action)

        recording_menu = self.menuBar().addMenu("Recording")
        start_action = QAction("Start recording", self)
        start_action.triggered.connect(lambda: self.record_button.setChecked(True))
        recording_menu.addAction(start_action)
        stop_action = QAction("Stop recording", self)
        stop_action.triggered.connect(lambda: self.record_button.setChecked(False))
        recording_menu.addAction(stop_action)
        recording_menu.addSeparator()
        section_action = QAction("New section", self)
        section_action.triggered.connect(self.add_annotation)
        recording_menu.addAction(section_action)

        measure_menu = self.menuBar().addMenu("Measurement")
        sweep_action = QAction("Automated sweep (function generator)...", self)
        sweep_action.triggered.connect(self.open_auto_sweep)
        measure_menu.addAction(sweep_action)

        # Link diagnostics live apart from the measurement menus: a throughput
        # test is a bench question about the radio, not part of an experiment.
        tools_menu = self.menuBar().addMenu("Tools")
        benchmark_action = QAction("Bluetooth throughput benchmark...", self)
        benchmark_action.setToolTip(
            "Measure the real SPP throughput to this PC with synthetic frames. Nothing is read "
            "from the ADS1299, so the result is the link and only the link."
        )
        benchmark_action.triggered.connect(self.open_bluetooth_benchmark)
        tools_menu.addAction(benchmark_action)

    def _load_settings_into_ui(self) -> None:
        index = self.baud_combo.findData(self.settings.baud_rate)
        if index >= 0:
            self.baud_combo.setCurrentIndex(index)
        self.session_name_edit.setText(self.settings.session_name)
        self.measurement = MeasurementInfo.from_dict(self.settings.measurement)
        self.measurement.reference_mode = self.hardware.normalised_reference_mode()
        self._update_measurement_label()
        self._update_output_dir_label()

        view = self.view.as_dict()
        self.time_panel.apply_settings(view)
        self.spectrum_panel.apply_settings(view)
        self.accel_panel.configure(
            self.view.accel_full_scale_g,
            self.view.accel_motion_threshold_g,
            self.view.accel_baseline_seconds,
        )
        scale_index = self.accel_scale_combo.findData(self.view.accel_full_scale_g)
        if scale_index >= 0:
            self.accel_scale_combo.setCurrentIndex(scale_index)
        self.accel_threshold_spin.setValue(self.view.accel_motion_threshold_g)
        self.accel_baseline_spin.setValue(self.view.accel_baseline_seconds)
        self._update_accel_scale_label()
        # Reference channel: rebuild the list, then restore the saved choice.
        self._rebuild_reference_combo()
        self.reference_panel.set_channel(str(self.view.reference_channel or ""))
        self.reference_panel.set_fft_scale(self.view.reference_fft_scale_uv)
        fft_scale_mv = max(float(self.view.reference_fft_scale_uv) / 1000.0, 0.001)
        ref_position = self.reference_fft_scale_combo.findData(fft_scale_mv)
        if ref_position >= 0:
            blocker = QSignalBlocker(self.reference_fft_scale_combo)
            self.reference_fft_scale_combo.setCurrentIndex(ref_position)
            del blocker
        # Restore the filter controls with the change signals detached: each
        # handler writes the whole filter state back, so touching one widget
        # would overwrite the others with their not-yet-restored values.
        blockers = [
            QSignalBlocker(widget)
            for widget in (
                self.notch_50_check,
                self.notch_100_check,
                self.notch_q_spin,
                self.high_pass_combo,
                self.low_pass_combo,
                self.accel_scale_combo,
                self.accel_threshold_spin,
                self.accel_baseline_spin,
            )
        ]
        self.notch_50_check.setChecked(self.view.notch_50)
        self.notch_100_check.setChecked(self.view.notch_100)
        self.notch_q_spin.setValue(self.view.notch_q)
        self._select_combo_value(self.high_pass_combo, self.view.high_pass_hz)
        self._select_combo_value(self.low_pass_combo, self.view.low_pass_hz)
        del blockers
        self.record_visible_check.setChecked(bool(self.view.record_visible_only))
        self._update_record_scope_label()
        self.spectrum_panel.set_sample_rate(self.sample_rate)
        self.auto_connect_check.setChecked(bool(self.settings.auto_connect))
        self.auto_connect_check.toggled.connect(self._on_auto_connect_changed)
        self.hardware.sync_reference()
        # A saved file may still have bias on the reference input; enforce the
        # rule on load so the first write is already correct.
        if self._enforce_reference_bias_rule():
            self._save_settings()
        self._update_reference_note()

        self.time_panel.scale_combo.currentIndexChanged.connect(self._on_view_changed)
        self.time_panel.window_combo.currentIndexChanged.connect(self._on_view_changed)
        self.time_panel.rows_combo.currentIndexChanged.connect(self._on_view_changed)
        self.time_panel.center_check.toggled.connect(self._on_view_changed)
        self.spectrum_panel.points_combo.currentIndexChanged.connect(self._on_view_changed)
        self.spectrum_panel.window_combo.currentIndexChanged.connect(self._on_view_changed)
        self.spectrum_panel.smoothing_combo.currentIndexChanged.connect(self._on_view_changed)
        self.spectrum_panel.scale_combo.currentIndexChanged.connect(self._on_view_changed)
        self.spectrum_panel.log_check.toggled.connect(self._on_view_changed)
        self.spectrum_panel.channel_combo.currentIndexChanged.connect(self._on_view_changed)
        self.spectrum_panel.end_combo.currentIndexChanged.connect(self._on_view_changed)
        self.record_visible_check.toggled.connect(self._on_record_scope_changed)
        self.accel_scale_combo.currentIndexChanged.connect(self._on_accel_settings_changed)
        self.accel_threshold_spin.valueChanged.connect(self._on_accel_settings_changed)
        self.accel_baseline_spin.valueChanged.connect(self._on_accel_settings_changed)
        self.session_name_edit.editingFinished.connect(self._save_settings)
        self.baud_combo.currentIndexChanged.connect(self._save_settings)
        self.baud_combo.currentIndexChanged.connect(self._refresh_link_note)
        self.baud_combo.currentIndexChanged.connect(self._update_hardware_labels)
        self.port_combo.currentIndexChanged.connect(self._refresh_link_note)
        self.port_combo.currentIndexChanged.connect(self._update_hardware_labels)

    @staticmethod
    def _select_combo_value(combo: QComboBox, value: float) -> None:
        best = 0
        best_delta = None
        for index in range(combo.count()):
            data = float(combo.itemData(index) or 0.0)
            delta = abs(data - float(value))
            if best_delta is None or delta < best_delta:
                best = index
                best_delta = delta
        combo.setCurrentIndex(best)

    def _on_view_changed(self, *_args) -> None:
        self._save_settings()

    def _on_record_scope_changed(self, _checked: bool = False) -> None:
        self.view.record_visible_only = self.record_visible_check.isChecked()
        self._update_record_scope_label()
        self._save_settings()

    def _on_auto_connect_changed(self, checked: bool) -> None:
        self.settings.auto_connect = bool(checked)
        if checked:
            self._autoconnect_timer.start()
            if not self.connected:
                self._autoconnect()
        else:
            self._autoconnect_timer.stop()
        self._save_settings()

    def _update_reference_note(self) -> None:
        """Show what the reference routing actually is, and what will float.

        The risks list is the part that answers "why is my input saturated": SRB2
        drives INxP rather than referring the channel, and a referenced input with
        no bias path has no DC return.
        """

        text = self.hardware.reference_summary()
        if self.hardware.chip_modes_differ:
            text += "   (mixed reference; change it per converter in the settings window)"
        risks = self.hardware.reference_risks()
        if risks:
            text += "\n" + "  ".join(risks)
        self.reference_note.setText(text)
        self._update_measurement_label()

    def _record_indices(self) -> list[int]:
        """Physical channel indices that the next recording will store."""
        indices = [index for index in real_channel_indices(self.hardware.board_channels)
                   if index < self.channel_count]
        if not self.view.record_visible_only:
            return indices
        flags = [checkbox.isChecked() for checkbox in self.channel_checkboxes]
        checked = [index for index in indices if index < len(flags) and flags[index]]
        return checked or indices

    def _update_record_scope_label(self) -> None:
        label = getattr(self, "record_scope_label", None)
        if label is None:
            return
        available = [index for index in real_channel_indices(self.hardware.board_channels)
                     if index < self.channel_count]
        flags = [checkbox.isChecked() for checkbox in self.channel_checkboxes]
        checked = [index for index in available if index < len(flags) and flags[index]]
        if not self.view.record_visible_only:
            label.setText(f"records all {len(available)} channels")
        elif not checked:
            label.setText("no channel ticked - every channel will be recorded")
        else:
            label.setText(
                f"records {len(checked)} of {len(available)} channels: "
                + ", ".join(str(index + 1) for index in checked)
            )

    def _on_filter_changed(self, *_args) -> None:
        self.view.notch_50 = self.notch_50_check.isChecked()
        self.view.notch_100 = self.notch_100_check.isChecked()
        self.view.notch_q = float(self.notch_q_spin.value())
        self.view.high_pass_hz = float(self.high_pass_combo.currentData() or 0.0)
        self.view.low_pass_hz = float(self.low_pass_combo.currentData() or 0.0)
        self.filter.configure(self.view.filter_state())
        self._save_settings()

    def _save_settings(self, *_args) -> None:
        self.view.vertical_scale_uv = self.time_panel.scale_uv
        self.view.window_seconds = self.time_panel.window_seconds
        self.view.visible_rows = int(self.time_panel.rows_combo.currentData() or 8)
        self.view.center_channels = self.time_panel.center_check.isChecked()
        self.view.fft_points = int(self.spectrum_panel.points_combo.currentData() or 1024)
        self.view.fft_window = self.spectrum_panel.window_combo.currentText()
        self.view.fft_smoothing = float(self.spectrum_panel.smoothing_combo.currentData() or 0.0)
        self.view.fft_scale_mode = float(self.spectrum_panel.scale_combo.currentData() or 0.0)
        self.view.fft_log_scale = self.spectrum_panel.log_check.isChecked()
        self.view.fft_channel = self.spectrum_panel.channel_mode()
        self.view.fft_end_hz = self.spectrum_panel.end_frequency()
        self.view.record_visible_only = self.record_visible_check.isChecked()
        self.view.accel_full_scale_g = float(self.accel_scale_combo.currentData() or 2.0)
        self.view.accel_motion_threshold_g = float(self.accel_threshold_spin.value())
        self.view.accel_baseline_seconds = float(self.accel_baseline_spin.value())
        self.view.visible_channels = [checkbox.isChecked() for checkbox in self.channel_checkboxes]
        self.view.notch_50 = self.notch_50_check.isChecked()
        self.view.notch_100 = self.notch_100_check.isChecked()
        self.view.notch_q = float(self.notch_q_spin.value())
        self.view.high_pass_hz = float(self.high_pass_combo.currentData() or 0.0)
        self.view.low_pass_hz = float(self.low_pass_combo.currentData() or 0.0)
        self.settings.hardware = self.hardware
        self.settings.session_name = self.session_name_edit.text().strip() or "session"
        self.settings.measurement = self.measurement.as_dict()
        port = str(self.port_combo.currentData() or "")
        self.settings.last_port = "" if port == DEMO_PORT else port
        self.settings.baud_rate = int(self.baud_combo.currentData() or DEFAULT_BAUD)
        try:
            self.store.save(self.settings)
        except OSError as exc:
            self.statusBar().showMessage(f"Could not save settings: {exc}", 6000)

    def _update_output_dir_label(self) -> None:
        directory = self._output_directory()
        self.output_dir_label.setText(str(directory))
        self.output_dir_label.setToolTip(str(directory))

    def _output_directory(self) -> Path:
        directory = Path(self.settings.output_dir or "recordings")
        if not directory.is_absolute():
            directory = Path(__file__).resolve().parent / directory
        return directory

    def _choose_output_dir(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Recordings folder", str(self._output_directory()))
        if not directory:
            return
        self.settings.output_dir = directory
        self._update_output_dir_label()
        self._save_settings()

    # -- ports and connection ----------------------------------------------

    def _start_port_scan(self) -> None:
        """Ask the background scanner for a fresh list; never blocks."""

        scanner = getattr(self, "_port_scanner", None)
        if scanner is not None:
            scanner.request_scan()

    def _apply_ports(self, ports) -> None:
        """Rebuild the port list from a background scan result."""

        discovered = []
        self._port_is_bluetooth: dict[str, bool] = {}
        for entry in ports:
            label, device = entry[0], str(entry[1])
            is_bluetooth = bool(entry[2]) if len(entry) > 2 else False
            discovered.append((label, device))
            self._port_is_bluetooth[device] = is_bluetooth
        existing = [
            (self.port_combo.itemText(index), str(self.port_combo.itemData(index)))
            for index in range(self.port_combo.count())
            if self.port_combo.itemData(index) != DEMO_PORT
        ]
        if discovered and discovered == existing:
            self._refresh_link_note()
            self._refresh_hardware_labels_if_ready()
            return
        # Keep exactly what was selected. Rebuilding the list must never move the
        # selection, or a scan finishing in the background would silently change
        # the port the user picked.
        current = self.port_combo.currentData()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for label, device in discovered:
            self.port_combo.addItem(label, device)
        self.port_combo.addItem("DEMO  synthetic board (no hardware)", DEMO_PORT)
        target = current if current is not None else self.settings.last_port
        index = self.port_combo.findData(target) if target else -1
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)
        self.port_combo.setEnabled(not self.connected)
        self._refresh_link_note()
        self._refresh_hardware_labels_if_ready()
        if list_ports is None:
            self.connection_label.setText("pyserial is not installed - only the DEMO source is available")

    def _refresh_hardware_labels_if_ready(self) -> None:
        """Update the hardware summary only once the widgets it reads exist.

        A port scan can finish while the window is still being built, and the
        summary needs the baud combo and the board label.
        """

        if getattr(self, "baud_combo", None) is None or getattr(self, "board_label", None) is None:
            return
        self._update_hardware_labels()

    def _refresh_ports(self) -> None:
        """Refresh button: ask for a scan and keep the current selection."""

        if getattr(self, "_port_scanner", None) is None:
            self._apply_ports([])
        self._start_port_scan()
        self._save_settings()

    def _autoconnect_port(self) -> Optional[str]:
        """Port to open: the last one used, else COM5, else the only candidate.

        The board may be plugged in after the application started, so this is
        evaluated again on every retry rather than once at startup.
        """

        selected = self.port_combo.currentData()
        if selected == DEMO_PORT:
            # The demo source was chosen deliberately: never open a real port
            # instead, or the window would silently attach to hardware.
            return None
        if selected:
            # Whatever the list shows as selected is what the user expects to be
            # opened, even if it is not the remembered port.
            return str(selected)

        candidates: list[str] = []
        if self.settings.last_port and self.settings.last_port != DEMO_PORT:
            candidates.append(self.settings.last_port)
        candidates.append(AUTOCONNECT_FALLBACK_PORT)
        for candidate in candidates:
            if candidate and candidate != DEMO_PORT and self.port_combo.findData(candidate) >= 0:
                return candidate
        # Nothing preferred is present: if exactly one real port exists, that is
        # unambiguous enough to use.
        real_ports = [
            str(self.port_combo.itemData(index))
            for index in range(self.port_combo.count())
            if self.port_combo.itemData(index) != DEMO_PORT
        ]
        if len(real_ports) == 1:
            return real_ports[0]
        return None

    def _start_autoconnect(self) -> None:
        self._autoconnect()
        if self.auto_connect_check.isChecked():
            self._autoconnect_timer.start()

    def _autoconnect(self) -> None:
        """Try to open the board; keep retrying while auto-connect stays on."""

        if not self.auto_connect_check.isChecked():
            self._autoconnect_idle = True
            return
        if self.connected or self.worker is not None:
            self._autoconnect_idle = True
            return
        self._autoconnect_idle = False
        # Re-scan first: the board is often plugged in after the app starts.
        known = [self.port_combo.itemData(index) for index in range(self.port_combo.count())]
        self._refresh_ports_silent()
        port = self._autoconnect_port()
        if port is None:
            self.connection_label.setText(
                f"Waiting for the board (looking for {AUTOCONNECT_FALLBACK_PORT}) - it connects automatically"
            )
            self.connection_label.setStyleSheet("color: #b45309;")
            return
        if port not in known:
            self._log("in", f"[connection] found {port} on rescan", "info")
        index = self.port_combo.findData(port)
        if index < 0:
            return
        self.port_combo.setCurrentIndex(index)
        self.connection_label.setText(f"Auto-connecting to {port}...")
        self._log("in", f"[connection] auto-connecting to {port}", "info")
        self.connect_button.setChecked(True)

    def _refresh_ports_silent(self) -> None:
        """Ask for a fresh scan; results arrive on the GUI thread via _apply_ports.

        The enumeration itself happens on the scanner thread, because asking
        Windows about a Bluetooth COM port can block for seconds.
        """

        self._start_port_scan()

    def _on_connect_toggled(self, checked: bool) -> None:
        if checked:
            self._connect()
        else:
            self._disconnect()

    def _port_short_label(self) -> str:
        """Port name for status messages, without the pairing hint."""

        text = self.port_combo.currentText()
        return text.split("  (pair")[0].strip() or str(self.port_combo.currentData() or "")

    def _connect(self) -> None:
        if self.worker is not None:
            return
        key = self.port_combo.currentData()
        if not key:
            self._set_connection_state(False, "No serial port selected")
            return
        baud = int(self.baud_combo.currentData() or DEFAULT_BAUD)
        label = self._port_short_label()

        def open_transport():
            if key == DEMO_PORT:
                demo = DemoBoardPort(self.hardware.board_channels, self.hardware.sample_rate_code)
                demo.open()
                return demo
            if serial is None:
                raise RuntimeError("pyserial is not installed")
            # write_timeout matters: without it a write to a Bluetooth port whose
            # link has gone blocks for ever and takes the worker thread with it.
            try:
                return serial.Serial(key, baud, timeout=0.02, write_timeout=0.5)
            except (ValueError, TypeError) as exc:
                # A few drivers reject write_timeout outright; opening slowly is
                # better than not connecting at all.
                if "timeout" not in str(exc).lower():
                    raise
                return serial.Serial(key, baud, timeout=0.02)

        self._connect_attempts += 1
        self.worker = SerialWorker(open_transport, self.channel_count, self.hardware.physical_gains(), self)
        self.worker.batchReady.connect(self._on_batch)
        self.worker.textReady.connect(self._on_text)
        self.worker.failed.connect(self._on_worker_failed)
        self.worker.opened.connect(self._on_worker_opened)
        self.worker.start()

        self.connected = True
        self.info_received = False
        self.profile_checked = False
        self.probe_attempts = 0
        self.board_info = {}
        self.apply_status = {}
        # Opening a Bluetooth port can take seconds, and the OS can block on it
        # instead of failing. Saying "Connected" here would hide both cases, so the
        # label states what is actually happening until the open returns.
        attempt = f" (attempt {self._connect_attempts})" if self._connect_attempts > 1 else ""
        self._set_connection_state(True, f"Opening {label}{attempt}...")
        self.board_info_label.setText("Requesting board information...")
        self._set_apply_summary("Requesting board information...")
        self._save_settings()
        self._open_watchdog.start(self._open_watchdog_ms)

    def _on_worker_opened(self) -> None:
        self._open_watchdog.stop()
        if self.worker is None:
            return
        self._set_connection_state(True, f"Connected to {self._port_short_label()}")
        self._probe_timer.start(1200)
        # The benchmark's options depend on which transport this window now holds:
        # it must switch to "main window link" the moment Bluetooth is taken, or its
        # next connection attempt would break this one.
        if self._benchmark_dialog is not None:
            self._benchmark_dialog.refresh_link_state()

    def _on_open_watchdog(self) -> None:
        """The port open has neither succeeded nor failed - report and retry."""

        if self.worker is None:
            return
        label = self._port_short_label()
        self._log(
            "in",
            f"[link] opening {label} has not returned after {self._open_watchdog_ms / 1000:g} s",
            "error",
        )
        self._abandon_connection(f"{label} did not open within {self._open_watchdog_ms / 1000:g} s.")

    def _abandon_connection(self, reason: str) -> None:
        """Tear the attempt down without losing the operator's selection.

        The connect button is unchecking itself here, so ``_disconnect`` must not
        also start the auto-connect timer; retries are scheduled explicitly.
        """

        worker = self.worker
        self.worker = None
        self._open_watchdog.stop()
        self._probe_timer.stop()
        self._info_timer.stop()
        if worker is not None:
            worker.stop()
        self.connected = False
        self._set_streaming(False)
        # Uncheck without emitting: a signal would run _disconnect(), which resets
        # the attempt counter this method is in the middle of using.
        self.connect_button.blockSignals(True)
        self.connect_button.setChecked(False)
        self.connect_button.blockSignals(False)
        self._set_connection_state(False, "Not connected")
        self._schedule_reconnect(reason)

    def _schedule_reconnect(self, reason: str) -> None:
        if self._connect_attempts >= MAX_CONNECT_ATTEMPTS:
            self._set_connection_state(False, f"{reason} Gave up after {self._connect_attempts} attempts.")
            self._log("in", f"[link] {reason} no more retries", "error")
            self._connect_attempts = 0
            return
        delay = RECONNECT_DELAY_MS
        self._set_connection_state(False, f"{reason} Retrying in {delay / 1000:g} s...")
        self._log("in", f"[link] {reason} retrying in {delay / 1000:g} s", "info")
        if self.auto_connect_check.isChecked():
            self._autoconnect_timer.start()
        self._reconnect_timer.start(delay)

    def _on_reconnect(self) -> None:
        if self.worker is not None or self.connected:
            return
        if not self.port_combo.currentData():
            return
        self.connect_button.setChecked(True)

    def _disconnect(self) -> None:
        self._probe_timer.stop()
        self._info_timer.stop()
        self._apply_timer.stop()
        self._open_watchdog.stop()
        self._reconnect_timer.stop()
        self._connect_attempts = 0
        self._apply_current = None
        self._verify_timer.stop()
        self._verify_pending = False
        self._verify_lines = []
        self._set_dialog_connected(False)
        self._set_verify_result("")
        if self._benchmark_dialog is not None:
            # The benchmark may have been reading this link; it has to stop using it
            # rather than hold a handle to a port that is closing.
            self._benchmark_dialog.refresh_link_state()
        if self.recording is not None:
            self.stop_recording()
        worker = self.worker
        self.worker = None
        if worker is not None:
            try:
                worker.send(b"s")
                if self._selected_port_is_bluetooth():
                    # Windows can close the COM handle without telling the radio, and
                    # the board then keeps that client in its single slot - so the next
                    # connect is refused and the device still reads "connected". Ask the
                    # firmware to drop the client while the link is still open.
                    worker.send(b"|btdrop;")
                    time.sleep(0.15)
            except Exception:  # noqa: BLE001
                pass
            worker.stop()
        self.connected = False
        self._set_streaming(False)
        if self.auto_connect_check.isChecked():
            # Keep looking: the user unplugged the board or closed the port.
            self._autoconnect_timer.start()
        self._set_connection_state(False, "Not connected")
        self.board_info_label.setText("Board details appear here after connect.")

    def _port_busy_hint(self, message: str) -> str:
        """Explain the two failures that a retry cannot fix by itself."""

        text = message.lower()
        if "access is denied" in text or "permissionerror" in text or "busy" in text:
            return (
                " The port is held by another program - close any other serial monitor "
                "(or a second copy of this window) and remove and re-pair SZD_EEG."
            )
        if "could not open port" in text and "not" in text and "find" in text:
            return " The port disappeared; press Refresh and pick it again."
        return ""

    def _on_worker_failed(self, message: str) -> None:
        message += self._port_busy_hint(message)
        self._log("in", message, "error")
        self.statusBar().showMessage(message, 8000)
        if self.worker is None:
            return
        if not self.info_received:
            # The link never carried data, so this is a port problem. Retrying is
            # worth it (a stale Bluetooth pairing often works the second time) and
            # _schedule_reconnect owns the decision to stop trying.
            self._abandon_connection(message)
            return
        # The link had been working, so leave the reconnect decision to the operator.
        self._open_watchdog.stop()
        self._set_connection_state(False, message)
        self.connect_button.setChecked(False)

    def _probe_board(self) -> None:
        if not self.connected:
            return
        self.probe_attempts += 1
        self._send(b"v")
        self._info_timer.start(2600)

    def _on_info_timeout(self) -> None:
        if self.info_received or not self.connected:
            return
        if self.probe_attempts < 6:
            self.connection_label.setText(f"Waiting for the board... (attempt {self.probe_attempts + 1})")
            self._probe_timer.start(300)
        else:
            port_label = f"{self.port_combo.currentData() or ''} {self.port_combo.currentText()}".lower()
            if "bluetooth" in port_label:
                hint = (
                    "The Bluetooth port opened but the board stayed silent. Windows creates two ports per "
                    "Bluetooth service and only the outgoing one carries data - try the other 'Standard Serial "
                    "over Bluetooth link' port. If both are silent, open the USB port and look at the [BT] "
                    "lines: the board reports stale and rejected connections there. Removing SZD_EEG in "
                    "Settings -> Bluetooth & devices and pairing again clears a stuck pairing."
                )
            else:
                hint = "The board did not answer the version request. Check the port and that the firmware is running."
            self._set_apply_summary(hint, error=True)
            self._log("in", "No response to the version request", "error")

    def _absorb_board_info(self, text: str) -> None:
        stripped = text.strip()
        if stripped.startswith("On Board ADS1299"):
            self.board_info["board_id"] = stripped.split(":")[-1].strip()
        elif stripped.startswith("On Daisy ADS1299"):
            self.board_info["daisy_id"] = stripped.split(":")[-1].strip()
        elif stripped.startswith("LIS3DH Device ID"):
            self.board_info["accel"] = stripped.split(":")[-1].strip()
        elif stripped.startswith("SD card:"):
            self.board_info["sd"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("Firmware:"):
            self.board_info["firmware"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("SPI clock:"):
            self.board_info["spi"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("SPI link limit:"):
            self.board_info["spi_limit"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("=== POST"):
            self.board_info["post"] = stripped
        elif stripped.startswith("Hardware") and ":" in stripped:
            self.board_info["hardware"] = stripped.split(":", 1)[1].strip()
        else:
            return
        if "firmware" in self.board_info and not self.info_received:
            self._on_info_received()

    def _on_info_received(self) -> None:
        self.info_received = True
        self._info_timer.stop()
        self._probe_timer.stop()
        has_daisy = "daisy_id" in self.board_info
        detected = 16 if has_daisy else 8
        if detected != self.hardware.board_channels and not self.profile_checked and self.recording is None:
            self.profile_checked = True
            self._log("in", f"[settings] Board reports {detected} channels - switching the profile", "info")
            self._change_board_profile(detected)
            return
        self.profile_checked = True
        self._update_hardware_labels()
        self._update_board_info_label()
        self._set_connection_state(True, f"Connected to {self._port_short_label()}")
        QTimer.singleShot(250, self.apply_hardware)

    def _update_board_info_label(self) -> None:
        parts: list[str] = []
        if "board_id" in self.board_info:
            parts.append(f"ADS1299 {self.board_info['board_id']}")
        if "daisy_id" in self.board_info:
            parts.append(f"daisy {self.board_info['daisy_id']}")
        if "accel" in self.board_info:
            parts.append(f"LIS3DH {self.board_info['accel']}")
        if "sd" in self.board_info:
            parts.append(f"SD {self.board_info['sd']}")
        if "firmware" in self.board_info:
            parts.append(f"fw {self.board_info['firmware']}")
        if "spi_limit" in self.board_info:
            parts.append(f"SPI {self.board_info['spi_limit']}")
        self.board_info_label.setText(" | ".join(parts) if parts else "No board details received")
        self.board_info_label.setToolTip("\n".join(f"{key}: {value}" for key, value in self.board_info.items()))

    def _set_connection_state(self, connected: bool, message: str) -> None:
        self.connect_button.blockSignals(True)
        self.connect_button.setChecked(connected)
        self.connect_button.blockSignals(False)
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.stream_button.setEnabled(connected)
        self.record_button.setEnabled(connected)
        self._set_dialog_connected(connected)
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.connection_label.setText(message)
        self.connection_label.setStyleSheet("color: #15803d;" if connected else "color: #475569;")
        # The card is never read automatically: SD.usedBytes() walks the FAT and
        # that shows up as a jump in the traces, so the panel waits for Refresh.
        if self._power_dialog is not None:
            self._power_dialog.set_connected(connected)
        if not connected:
            self._sd_pending = False
            self.sd_status_label.setText("Connect the board to read the card.")
            self.sd_state_label.setText("")
            self.sd_health_label.setText("")
        elif not self._sd_state:
            self.sd_status_label.setText("Press Refresh to read the card.")
        self._update_status_bar()

    def _restart_worker(self) -> None:
        self._probe_timer.stop()
        self._info_timer.stop()
        worker = self.worker
        self.worker = None
        if worker is not None:
            try:
                worker.send(b"s")
            except Exception:  # noqa: BLE001
                pass
            worker.stop()
        self.connected = False
        self._set_streaming(False)
        self._set_connection_state(False, "Reconnecting with the new board profile...")
        QTimer.singleShot(200, self._reconnect)

    def _reconnect(self) -> None:
        if self.connect_button.isChecked():
            return
        self.connect_button.setChecked(True)
    def _build_reference_group(self) -> QWidget:
        """Choose and scale the channel that measures the applied stimulus."""

        group = QGroupBox("Reference channel")
        layout = QVBoxLayout(group)

        row = QHBoxLayout()
        row.addWidget(QLabel("Channel"))
        self.reference_channel_combo = QComboBox()
        self.reference_channel_combo.setToolTip(
            "Input wired across the generator. Forced to PGA x1 and kept out of the main plots/FFT."
        )
        self.reference_channel_combo.currentIndexChanged.connect(self._on_reference_channel_changed)
        row.addWidget(self.reference_channel_combo, 1)
        layout.addLayout(row)

        scale_row = QHBoxLayout()
        scale_row.addWidget(QLabel("FFT full scale"))
        self.reference_fft_scale_combo = QComboBox()
        for value in (1.0, 10.0, 100.0, 1000.0, 10000.0):
            self.reference_fft_scale_combo.addItem(f"{value:g} mVrms", value)
        self.reference_fft_scale_combo.currentIndexChanged.connect(self._on_reference_fft_scale_changed)
        scale_row.addWidget(self.reference_fft_scale_combo, 1)
        layout.addLayout(scale_row)
        return group

    def _on_reference_fft_scale_changed(self, *_args) -> None:
        self.view.reference_fft_scale_uv = float(
            (self.reference_fft_scale_combo.currentData() or 1000.0)
        ) * 1000.0
        self._save_settings()

    def _build_accel_group(self) -> QWidget:
        group = QGroupBox("Accelerometer")
        layout = QGridLayout(group)
        # The accelerometer moved out of the main view to leave room for the
        # reference channel; it lives here as a compact widget.
        self.accel_panel = AccelerometerPanel()
        self.accel_panel.setMinimumHeight(110)
        layout.addWidget(self.accel_panel, 0, 0, 1, 2)
        layout.addWidget(QLabel("Full scale"), 1, 0)
        self.accel_scale_combo = QComboBox()
        for value in (0.5, 1.0, 2.0, 4.0):
            self.accel_scale_combo.addItem(f"+/- {value:g} g", value)
        layout.addWidget(self.accel_scale_combo, 1, 1)
        layout.addWidget(QLabel("Motion alarm"), 2, 0)
        self.accel_threshold_spin = QDoubleSpinBox()
        self.accel_threshold_spin.setRange(0.005, 2.0)
        self.accel_threshold_spin.setSingleStep(0.01)
        self.accel_threshold_spin.setDecimals(3)
        self.accel_threshold_spin.setSuffix(" g")
        layout.addWidget(self.accel_threshold_spin, 2, 1)
        layout.addWidget(QLabel("Baseline"), 3, 0)
        self.accel_baseline_spin = QDoubleSpinBox()
        self.accel_baseline_spin.setRange(0.3, 10.0)
        self.accel_baseline_spin.setSingleStep(0.5)
        self.accel_baseline_spin.setDecimals(1)
        self.accel_baseline_spin.setSuffix(" s")
        layout.addWidget(self.accel_baseline_spin, 3, 1)
        self.accel_calibrate_button = QPushButton("Calibrate from rest")
        self.accel_calibrate_button.setToolTip("With the board lying still, set the scale so the current reading equals 1 g")
        self.accel_calibrate_button.clicked.connect(self._calibrate_accelerometer)
        layout.addWidget(self.accel_calibrate_button, 4, 0, 1, 2)
        self.accel_scale_label = QLabel("")
        self.accel_scale_label.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.accel_scale_label, 5, 0, 1, 2)
        return group

    def _on_accel_settings_changed(self, *_args) -> None:
        self.view.accel_full_scale_g = float(self.accel_scale_combo.currentData() or 2.0)
        self.view.accel_motion_threshold_g = float(self.accel_threshold_spin.value())
        self.view.accel_baseline_seconds = float(self.accel_baseline_spin.value())
        self.accel_panel.configure(
            self.view.accel_full_scale_g,
            self.view.accel_motion_threshold_g,
            self.view.accel_baseline_seconds,
        )
        self._save_settings()

    def _calibrate_accelerometer(self) -> None:
        counts = self._last_aux_counts
        if counts is None:
            QMessageBox.information(self, "Accelerometer", "Start the stream first, then keep the board still while calibrating.")
            return
        magnitude = float(np.linalg.norm(counts))
        if magnitude < 100.0:
            QMessageBox.information(self, "Accelerometer", "The accelerometer reading looks invalid - check the LIS3DH wiring.")
            return
        self.view.accel_counts_per_g = magnitude
        self._update_accel_scale_label()
        self._save_settings()

    def _update_accel_scale_label(self) -> None:
        self.accel_scale_label.setText(f"{self.view.accel_counts_per_g:,.0f} counts per g")
    def _render_reference_panel(self) -> None:
        index = self.reference_channel_index()
        self.reference_panel.set_fft_scale(self.view.reference_fft_scale_uv)
        if index is None:
            return
        window_count = int(self.time_panel.window_seconds * self.sample_rate)
        block = self.display_ring.last(window_count)
        if block is None or index >= block.shape[1]:
            return
        self.reference_panel.update_trace(block[:, index], self.sample_rate)

    def reference_channel_index(self) -> Optional[int]:
        """GUI index of the selected reference channel, or None."""

        name = str(getattr(self.view, "reference_channel", "") or "").strip()
        if not name:
            return None
        for index in range(self.channel_count):
            if channel_label(index) == name:
                return index
        return None

    def _on_reference_channel_changed(self, *_args) -> None:
        label = str(self.reference_channel_combo.currentData() or "")
        self.view.reference_channel = label
        self.reference_panel.set_channel(label)
        changed: list[int] = []
        if label:
            index = self.reference_channel_index()
            # A stimulus input is volts, not microvolts: PGA x24 would clip it.
            if index is not None and index < len(self.hardware.channels):
                setting = self.hardware.channels[index]
                if setting.gain_ordinal != 0:
                    setting.gain_ordinal = 0
                    changed.append(index)
                    self._log(
                        "in",
                        f"[reference] {label} gain set to x1 (a stimulus input must not be amplified)",
                        "info",
                    )
        changed.extend(self._enforce_reference_bias_rule())
        self.hardware.sync_reference()
        if changed:
            self._save_settings()
            self._update_hardware_labels()
            if self.connected and self.recording is None:
                self.apply_hardware(sorted(set(changed)))
        self._sync_sweep_monitor_channel()
        self._save_settings()

    def _enforce_reference_bias_rule(self) -> list[int]:
        """Keep the reference channel out of the bias/DRL loop.

        The reference input is tied to the stimulus (or to a shared reference
        electrode), so feeding it into the bias amplifier would drive the DRL
        electrode with the stimulus itself - and, during a CMRR sweep, with the
        very voltage under test. It is excluded unconditionally here rather than
        left as an operator choice, and any channel that had to be changed is
        returned so the caller can re-apply just those.

        Applies to every real channel except the selected reference, so switching
        the reference away restores bias on the old one.
        """

        reference = self.reference_channel_index()
        changed: list[int] = []
        for index in real_channel_indices(self.hardware.board_channels):
            if index >= self.channel_count or index >= len(self.hardware.channels):
                continue
            setting = self.hardware.channels[index]
            should_bias = index != reference
            if setting.bias != should_bias:
                setting.bias = should_bias
                changed.append(index)
        if reference is not None and reference in changed:
            self._log(
                "in",
                f"[reference] {channel_label(reference)} excluded from the bias circuit",
                "info",
            )
        return changed

    def _sync_sweep_monitor_channel(self) -> None:
        """One place decides the monitor channel for the automated sweeps."""

        label = str(getattr(self.view, "reference_channel", "") or "")
        self.sweep_settings.monitor_channel = label
        self.settings.sweep = self.sweep_settings.as_dict()
        if self._sweep_dialog is not None:
            self._sweep_dialog.set_monitor_channel(label)

    def _rebuild_reference_combo(self) -> None:
        combo = getattr(self, "reference_channel_combo", None)
        if combo is None:
            return
        current = str(getattr(self.view, "reference_channel", "") or "")
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("(none)", "")
        for index in real_channel_indices(self.hardware.board_channels):
            if index < self.channel_count:
                combo.addItem(channel_label(index), channel_label(index))
        position = combo.findData(current)
        combo.setCurrentIndex(position if position >= 0 else 0)
        combo.blockSignals(False)

    def _reset_reference_channel(self) -> None:
        self.view.reference_channel = ""
        self._rebuild_reference_combo()
        self.reference_panel.set_channel("")
        self._sync_sweep_monitor_channel()
        self._save_settings()

    # -- streaming and hardware --------------------------------------------

    def _on_stream_toggled(self, checked: bool) -> None:
        if checked:
            self.start_streaming()
        else:
            self.stop_streaming()

    def start_streaming(self) -> None:
        if not self.connected:
            self._set_streaming(False)
            return
        self._rebuild_buffers()
        self.accel_panel.reset()
        self._set_streaming(True)
        self._send(b"b")

    def stop_streaming(self) -> None:
        if self.connected:
            self._send(b"s")
        self._set_streaming(False)

    def _set_streaming(self, value: bool) -> None:
        self.streaming = bool(value)
        self.stream_button.blockSignals(True)
        self.stream_button.setChecked(self.streaming)
        self.stream_button.blockSignals(False)
        self.stream_button.setText("Stop stream" if self.streaming else "Start stream")
        self._update_status_bar()

    def _send(self, payload: bytes) -> bool:
        if self.worker is None:
            return False
        self.worker.send(payload)
        self._log("sent", self._describe_payload(payload), "sent")
        return True

    @staticmethod
    def _describe_payload(payload: bytes) -> str:
        try:
            text = payload.decode("ascii")
        except UnicodeDecodeError:
            return payload.hex(" ").upper()
        return text if text.isprintable() else payload.hex(" ").upper()

    def _log(self, direction: str, text: str, kind: str = "info") -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        record = (stamp, direction, text, kind)
        self._text_backlog.append(record)
        # Bounded: a long session with a chatty board must not grow without limit.
        if len(self._text_backlog) > 5000:
            del self._text_backlog[:-4000]
        for monitor in list(self.monitors):
            if monitor.isVisible():
                monitor.append_record(*record)

    def bluetooth_budget_bytes_per_second(self) -> float:
        """Planned SPP capacity, after the user's safety margin.

        The built-in 32 KiB/s is a conservative estimate, not a physical limit. A
        measured figure only replaces it when the user adopts one from the
        throughput benchmark, and the margin is subtracted on top, so a peak burst
        is never mistaken for a sustainable rate.
        """

        link = self.settings.link if isinstance(self.settings.link, dict) else {}
        try:
            stored = float(link.get("bluetooth_budget_bps", 0) or 0)
        except (TypeError, ValueError):
            stored = 0.0
        try:
            margin = float(link.get("bluetooth_margin_percent", 0) or 0)
        except (TypeError, ValueError):
            margin = 0.0
        base = stored if stored > 0 else float(BLUETOOTH_SPP_BYTES_PER_SECOND)
        margin = min(max(margin, 0.0), 90.0)
        return max(base * (1.0 - margin / 100.0), 1.0)

    def set_bluetooth_budget(self, bytes_per_second: Optional[float], margin_percent: float) -> None:
        """Adopt (or clear) a measured link capacity and keep it in the settings."""

        if bytes_per_second is not None and bytes_per_second > 0:
            self.settings.link["bluetooth_budget_bps"] = float(bytes_per_second)
            self.settings.link["bluetooth_margin_percent"] = float(margin_percent)
        else:
            self.settings.link.pop("bluetooth_budget_bps", None)
            self.settings.link.pop("bluetooth_margin_percent", None)
        self.store.save(self.settings)
        self._refresh_link_note()

    def _link_description(self, baud: int) -> str:
        """Name the transport in a budget message, without inventing a baud rate."""

        if self._selected_port_is_bluetooth():
            return (
                f"the Bluetooth link ({self.bluetooth_budget_bytes_per_second() / 1024:.0f} kB/s budget)"
            )
        return f"the {baud / 1000:.0f} kbit/s USB link"

    def _ensure_link_budget(self, baud: int) -> bool:
        """Make the saved rate fit both links, adjusting it if it does not.

        Raising SCLK is tried first because it costs nothing, then a larger
        decimation (which keeps the sinc3 corner where the operator wanted it),
        and only then a lower data rate.
        """

        data_rate = self.hardware.data_rate_hz
        fits, reason = rate_fits_link(
            data_rate, self.hardware.output_packet_rate_hz, baud, self.hardware.spi_clock_hz, 2
        )
        if fits:
            self._rate_downgrade_note = ""
            return True

        notes: list[str] = []
        spi_limit = max_sample_rate_from_spi(self.hardware.spi_clock_hz, 2)
        if data_rate > spi_limit:
            for hz, _label in SPI_CLOCK_CHOICES:
                if max_sample_rate_from_spi(hz, 2) >= data_rate:
                    if hz != self.hardware.spi_clock_hz:
                        self.hardware.spi_clock_hz = hz
                        notes.append(f"SPI clock raised to {hz / 1e6:g} MHz so {data_rate:g} SPS fits the bus")
                    break
            else:
                if self.hardware.spi_clock_hz != ADS_SPI_CLOCK_MAX_HZ:
                    self.hardware.spi_clock_hz = ADS_SPI_CLOCK_MAX_HZ
                    notes.append(f"SPI clock raised to {ADS_SPI_CLOCK_MAX_HZ / 1e6:g} MHz (the maximum)")

        serial_limit = max_sustained_packet_rate(baud)
        if data_rate / max(self.hardware.decimation, 1) > serial_limit:
            for factor in DECIMATION_CHOICES:
                if data_rate / factor <= serial_limit:
                    if factor != self.hardware.decimation:
                        notes.append(
                            f"decimation raised to /{factor} so {data_rate / factor:g} packets/s fits "
                            f"{self._link_description(baud)}"
                        )
                        self.hardware.decimation = factor
                    break

        spi_limit = max_sample_rate_from_spi(self.hardware.spi_clock_hz, 2)
        if data_rate > spi_limit:
            # SAMPLE_RATES is ordered fastest-first, so the first entry that fits
            # is the fastest usable rate.
            fallback = None
            for rate, code in SAMPLE_RATES:
                if rate <= spi_limit and rate / max(self.hardware.decimation, 1) <= serial_limit:
                    fallback = (rate, code)
                    break
            if fallback is not None:
                notes.append(f"data rate lowered to {fallback[0]:g} SPS")
                self.hardware.sample_rate_code = fallback[1]

        fits, reason = rate_fits_link(
            self.hardware.data_rate_hz,
            self.hardware.output_packet_rate_hz,
            baud,
            self.hardware.spi_clock_hz,
            2,
        )
        if not fits:
            message = f"The board configuration does not fit the link: {reason}."
            self._log("in", f"[settings] {message}", "error")
            self._set_apply_summary(message, error=True)
            return False

        message = "; ".join(notes) + "." if notes else ""
        self._rate_downgrade_note = message
        if message:
            self._log("in", f"[settings] {message}", "info")
            self._set_apply_summary(message)
        self._save_settings()
        self._update_hardware_labels()
        if abs(self.sample_rate - self.hardware.sample_rate_hz) > 1e-9:
            self._rebuild_buffers()
        return True

    def apply_hardware(
        self,
        channels: Optional[Sequence[int]] = None,
        hardware: Optional[HardwareSettings] = None,
    ) -> bool:
        """Write the saved sample rate and channel settings to the board.

        Returns True when a write actually started, so an action that depends on
        it (storing the settings on the SD card) can tell "queued" from "refused"
        instead of silently waiting forever.
        """

        if hardware is not None:
            self._set_hardware(hardware)
        if not self.connected or self.worker is None:
            self._set_apply_summary("Connect the board before writing settings.", error=True)
            return False
        if self.recording is not None:
            self._set_apply_summary("Stop the recording before changing hardware settings.", error=True)
            return False
        if self._apply_current is not None and self._apply_timer.isActive():
            self._set_apply_summary("A settings write is already in progress.", error=True)
            return False

        requested_rate = self.hardware.data_rate_hz
        packets = self.hardware.output_packet_rate_hz
        ba = self._effective_baud_for_budget()
        if not self._ensure_link_budget(ba):
            return False

        if abs(self.sample_rate - self.hardware.sample_rate_hz) > 1e-9:
            # The write above changed the rate, and the time base, FFT axis and
            # ring buffers are all sized from it.
            self._rebuild_buffers()

        self.hardware.sync_reference()
        # The reference input must never join the bias loop, whichever path got
        # here (dialog apply, re-apply saved, auto-connect, a sweep).
        self._enforce_reference_bias_rule()
        self.hardware.sync_reference()
        indices = list(channels) if channels is not None else real_channel_indices(self.hardware.board_channels)
        indices = [index for index in indices if index < len(self.hardware.channels)]
        chip_modes = self.hardware.normalised_chip_modes()
        plan: list[tuple[Optional[int], bytes, str]] = [
            (
                None,
                build_spi_clock_command(self.hardware.spi_clock_hz),
                f"SPI clock {self.hardware.spi_clock_hz / 1000:.0f} kHz",
            ),
            (
                None,
                f"|btpower={int(self.hardware.bluetooth_power_dbm)};".encode("ascii"),
                f"Bluetooth power {int(self.hardware.bluetooth_power_dbm)} dBm",
            ),
            (
                None,
                build_decimation_command(self.hardware.decimation),
                f"decimation /{self.hardware.decimation}",
            ),
            (
                None,
                build_sample_rate_command(self.hardware.sample_rate_code),
                f"data rate {requested_rate:g} SPS ({packets:g} packets/s)",
            ),
        ]
        for index in indices:
            plan.append((index, build_channel_command(index, self.hardware.channels[index]), f"channel {index + 1}"))
        # Last, so the SRB routing is always the final state on the chip and can
        # never be left half applied by a per-channel write. Programmed per
        # converter because SRB1 is one switch inside each ADS1299.
        for device, mode in enumerate(chip_modes):
            plan.append(
                (
                    None,
                    build_chip_reference_command(device, mode),
                    f"reference ADS{device + 1} {mode}",
                )
            )

        self._apply_plan = plan
        self._apply_index = 0
        self._apply_current = None
        self._apply_failures = 0
        self.apply_status = {index: "pending" for index in indices}
        self._resume_streaming = self.streaming
        if self.streaming:
            self._send(b"s")
            self._set_streaming(False)
        if self._settings_dialog is not None:
            self._settings_dialog.update_statuses(self.apply_status)
        self._set_apply_summary(f"Writing {len(indices)} channel(s) and the sample rate to the board...")
        self._send_next_apply()
        return True

    def _send_next_apply(self) -> None:
        if self._apply_index >= len(self._apply_plan):
            self._finish_apply()
            return
        channel, payload, label = self._apply_plan[self._apply_index]
        self._apply_current = (channel, label)
        if channel is not None:
            self.apply_status[channel] = "pending"
        self._send(payload)
        if self._settings_dialog is not None:
            self._settings_dialog.update_statuses(self.apply_status)
        self._apply_timer.start(900)

    def _on_apply_timeout(self) -> None:
        if self._apply_current is None:
            return
        channel, label = self._apply_current
        self._apply_current = None
        self._apply_failures += 1
        if channel is not None:
            self.apply_status[channel] = "timeout"
        self._log("in", f"[settings] no reply for {label}", "error")
        self._apply_index += 1
        self._send_next_apply()

    def _resolve_apply(self, ok: bool) -> None:
        if self._apply_current is None:
            return
        channel, _label = self._apply_current
        self._apply_current = None
        self._apply_timer.stop()
        if channel is not None:
            self.apply_status[channel] = "ok" if ok else "failed"
        if not ok:
            self._apply_failures += 1
        self._apply_index += 1
        self._send_next_apply()

    def _finish_apply(self) -> None:
        confirmed = sum(1 for status in self.apply_status.values() if status == "ok")
        total = len(self.apply_status)
        failed = total - confirmed
        if failed == 0 and total:
            text = f"Saved settings written to the board: {confirmed}/{total} channels confirmed."
            self._set_apply_summary(text, success=True)
        elif total:
            text = f"{confirmed}/{total} channels confirmed, {failed} problem(s). Open Hardware settings for details."
            self._set_apply_summary(text, error=True)
        else:
            text = "Nothing to write."
            self._set_apply_summary(text)
        note = self._rate_downgrade_note
        self._rate_downgrade_note = ""
        self.stream_health: Optional[tuple[int, int, int]] = None
        if note:
            text = f"{text} {note}"
            self._set_apply_summary(text, error=True)
        self._log("in", f"[settings] {text}", "info")
        if self._settings_dialog is not None:
            self._settings_dialog.update_statuses(self.apply_status)
        if self._resume_streaming and self.connected:
            self._resume_streaming = False
            self._send(b"b")
            self._set_streaming(True)
        if self._sd_save_after_apply:
            # The converters now hold the new settings, so the card can store them.
            self._sd_save_after_apply = False
            self._sd_pending_action = "save"
            self._send(b":w")
            self.sd_state_label.setText("Settings sent to the card - waiting for confirmation...")
            self._sd_action_hold = True
        self._sd_hold_until = time.monotonic() + 8.0

    def _set_apply_summary(self, text: str, error: bool = False, success: bool = False) -> None:
        self.apply_summary = text
        self.apply_status_label.setText(text)
        if error:
            color = "#b91c1c"
        elif success:
            color = "#15803d"
        else:
            color = "#475569"
        self.apply_status_label.setStyleSheet(f"color: {color};")

    # -- configuration verification ---------------------------------------

    def read_board_state(self) -> None:
        """Read and display the board's own view of its configuration."""

        if not self.connected or self.worker is None:
            QMessageBox.information(self, "Board state", "Connect the board first.")
            return
        if self.recording is not None:
            QMessageBox.information(self, "Board state", "Stop the recording first.")
            return
        self.verify_configuration()

    def repair_board(self) -> None:
        """Ask the firmware to re-program and self-check both converters.

        The firmware owns the register image, so this is the authoritative repair:
        it does not depend on the app being able to talk the settings in again.
        """

        if not self.connected or self.worker is None:
            QMessageBox.information(self, "Repair", "Connect the board first.")
            return
        if self.recording is not None:
            QMessageBox.information(self, "Repair", "Stop the recording first.")
            return
        if self._repair_pending:
            return
        self._repair_lines = []
        self._repair_pending = True
        self.repair_button.setEnabled(False)
        self.repair_button.setText("Repairing...")
        self._send(build_repair_command())
        self._repair_timer.start(REPAIR_TIMEOUT_MS)
        self._log("in", "[repair] re-programming both converters", "info")

    def _on_repair_line(self, text: str) -> bool:
        self._repair_lines.append(text)
        return "DONE=1" in text

    def _on_repair_timeout(self) -> None:
        if not self._repair_pending:
            return
        self._finish_repair("The board did not answer the repair command.", failed=True)

    def _finish_repair(self, text: str = "", failed: bool = False) -> None:
        self._repair_timer.stop()
        self._repair_pending = False
        self.repair_button.setEnabled(True)
        self.repair_button.setText("Repair and verify")
        report = parse_repair_report(self._repair_lines)
        details = report.describe() if report.devices else text or "No reply from the board."
        if report.bad_registers:
            details += "\n" + "\n".join(report.bad_registers[:8])
        self._set_verify_result(details, error=failed or not report.ok)
        self.board_report_text = details.split("\n")[0]
        reported = getattr(self, "board_report_label", None)
        if reported is not None:
            reported.setText(self.board_report_text)
        self._log("in", f"[repair] {details.splitlines()[0]}", "error" if (failed or not report.ok) else "info")

    def verify_configuration(self, hardware: Optional[HardwareSettings] = None) -> None:
        """Ask the board to read every ADS1299 register back and compare it."""
        if hardware is not None:
            self._pending_verify_hardware = hardware
        if not self.connected or self.worker is None:
            self._set_verify_result("Connect the board before verifying the configuration.", error=True)
            return
        if self.recording is not None:
            self._set_verify_result("Stop the recording before verifying the configuration.", error=True)
            return
        if self._verify_pending:
            return
        self._verify_lines = []
        self._verify_pending = True
        self._set_verify_busy(True)
        self._set_verify_result("Reading the ADS1299 registers back...", state="busy")
        self._send(b"P")
        self._verify_timer.start(VERIFY_TIMEOUT_MS)

    def _set_dialog_connected(self, connected: bool) -> None:
        dialog = self._settings_dialog
        if dialog is not None:
            dialog.set_connected(connected)

    def _set_verify_busy(self, busy: bool) -> None:
        dialog = self._settings_dialog
        if dialog is not None:
            dialog.set_verify_busy(busy)

    def _on_verify_line(self, text: str) -> bool:
        """Consume one ``[CFG]`` line; returns True when the report is complete."""

        self._verify_lines.append(text)
        return "DONE=1" in text

    def _on_verify_timeout(self) -> None:
        if not self._verify_pending:
            return
        self._verify_pending = False
        self._set_verify_busy(False)
        message = "The board did not answer the register readback."
        self._set_verify_result(message, error=True)
        self._log("in", f"[verify] {message}", "error")

    def _finish_verify(self) -> None:
        self._verify_timer.stop()
        self._verify_pending = False
        self._set_verify_busy(False)

        # Compare against the settings the verify was requested with (the dialog
        # stages its edits), falling back to the saved ones.
        expected = self._pending_verify_hardware or self.hardware
        self._pending_verify_hardware = None

        report = parse_configuration_report(self._verify_lines)
        rate_line = next((line for line in self._verify_lines if "DONE=1" in line), "")
        board_rate = self._int_field(rate_line, "SPS")
        board_code = self._int_field(rate_line, "CODE")
        board_spi = self._int_field(rate_line, "SPILIMIT")
        board_clock = self._int_field(rate_line, "SPICLK")
        board_decimation = self._int_field(rate_line, "DECIM")
        board_out = self._int_field(rate_line, "OUT")

        if not report:
            self._set_verify_result("No register data came back from the board.", error=True)
            return

        # What the board itself says, independent of what the app remembers.
        state_lines: list[str] = []
        for line in self._verify_lines:
            if not line.strip().startswith("[CFG]"):
                continue
            fields = dict(token.split("=", 1) for token in line[len("[CFG]"):].split() if "=" in token)
            if "REGS" in fields:
                state_lines.append(
                    f"{fields.get('DEV', '?')}: ref {fields.get('REF', '?')}, "
                    f"CONFIG1 0x{fields['REGS'][2:4]}, MISC1 0x{fields['REGS'][42:44]}"
                )
        summary_bits: list[str] = []
        if board_rate is not None:
            summary_bits.append(f"{board_rate} SPS")
        if board_decimation is not None:
            summary_bits.append(f"/{board_decimation}")
        if board_out is not None:
            summary_bits.append(f"{board_out} packets/s")
        if board_clock is not None:
            summary_bits.append(f"SCLK {board_clock / 1000:.0f} kHz")
        if "REF=" in rate_line:
            summary_bits.append("ref " + rate_line.split("REF=", 1)[1].split()[0])
        self.board_report_text = "Board reports: " + ", ".join(summary_bits)
        if state_lines:
            self.board_report_text += "\n" + "\n".join(state_lines)
        reported = getattr(self, "board_report_label", None)
        if reported is not None:
            reported.setText(self.board_report_text)
        if self._settings_dialog is not None:
            self._settings_dialog.set_board_state(self.board_report_text)

        checks, notes = compare_configuration(report, expected)
        # Say *why* a converter is missing instead of leaving a bare note: a link
        # that reports LINK=0 is an SPI problem, no line at all is a lost report.
        links = parse_configuration_links(self._verify_lines)
        rewritten: list[str] = []
        for note in notes:
            if "did not report registers" in note:
                name = note.split()[0]
                if name in links and not links[name]:
                    rewritten.append(
                        f"{name} reports LINK=0: it did not answer on the SPI bus. Check its CS "
                        f"line, DOUT and power before trusting anything else here."
                    )
                    continue
                if name not in links:
                    rewritten.append(
                        f"{name} sent no [CFG] line at all - the report was lost on the way back. "
                        f"Run the check again; if it repeats, the link is dropping text."
                    )
                    continue
            rewritten.append(note)
        notes = rewritten
        checked_channels = sum(
            len(expectation.channel_regs) for expectation in expected_device_registers(expected)
        )
        details: list[str] = []
        for expectation in expected_device_registers(expected):
            regs = report.get(expectation.name)
            if not regs:
                continue
            mismatched = [check for check in checks if check.device == expectation.name]
            channel_hits = sum(1 for check in mismatched if check.label.endswith("SET"))
            details.append(
                f"{expectation.name}: ID 0x{regs[0]:02X}, CONFIG1 0x{regs[0x01]:02X}, "
                f"{len(expectation.channel_regs) - channel_hits}/{len(expectation.channel_regs)} channels match"
            )

        problems: list[str] = []
        if board_code is not None and board_code != int(expected.sample_rate_code):
            problems.append(
                f"sample rate code is {board_code}, expected {int(expected.sample_rate_code)}"
            )
        if board_rate is not None and abs(board_rate - expected.data_rate_hz) > 1e-6:
            problems.append(f"board data rate is {board_rate} SPS")
        if board_decimation is not None and board_decimation != int(expected.decimation):
            problems.append(f"board decimation is /{board_decimation}, expected /{int(expected.decimation)}")
        if board_clock is not None and board_clock != int(expected.spi_clock_hz):
            problems.append(f"board SCLK is {board_clock / 1000:.0f} kHz, expected {expected.spi_clock_hz / 1000:.0f} kHz")
        if board_spi is not None and board_spi != max_sample_rate_from_spi(expected.spi_clock_hz, 2):
            notes.append(
                f"board SPI budget is {board_spi} SPS, this app computes "
                f"{max_sample_rate_from_spi(expected.spi_clock_hz, 2)} SPS at {expected.spi_clock_hz / 1e6:g} MHz"
            )

        all_good = not checks and not problems
        lines = details + notes + problems
        lines += [check.describe() for check in checks[:8]]
        if len(checks) > 8:
            lines.append(f"... {len(checks) - 8} more register mismatch(es), see the serial monitor")

        if all_good:
            summary = (
                f"Configuration verified: all {checked_channels} channels and both converters match "
                "the saved settings."
            )
            self._set_verify_result("\n".join([summary] + lines), success=True)
        else:
            summary = (
                f"Configuration mismatch: {len(checks)} register(s)"
                + (f" and {len(problems)} rate problem(s)" if problems else "")
                + " differ from the saved settings."
            )
            self._set_verify_result("\n".join([summary] + lines), error=True)
        self._log("in", f"[verify] {summary}", "info" if all_good else "error")

    @staticmethod
    def _int_field(line: str, key: str) -> Optional[int]:
        match = re.search(rf"\b{re.escape(key)}=(\d+)", line or "")
        return int(match.group(1)) if match else None

    def _set_verify_result(
        self, text: str, error: bool = False, success: bool = False, state: Optional[str] = None
    ) -> None:
        resolved = state or ("error" if error else ("success" if success else "info"))
        self.verify_result_text = text
        self.verify_result_state = resolved
        dialog = self._settings_dialog
        if dialog is not None:
            dialog.set_verify_result(text, state=resolved)

    def _set_hardware(self, hardware: HardwareSettings) -> None:
        rate_changed = abs(hardware.sample_rate_hz - self.sample_rate) > 1e-9
        board_changed = hardware.board_channels != self.hardware.board_channels
        hardware.sync_reference()
        self.hardware = hardware
        self.settings.hardware = hardware
        self.channel_count = 16 if hardware.board_channels > 8 else 8
        self.view.ensure_visibility(self.channel_count)
        self._update_reference_note()
        self._save_settings()
        if self.worker is not None:
            self.worker.set_gains(hardware.physical_gains())
        if rate_changed or board_changed:
            self._rebuild_buffers()
        self._update_hardware_labels()
        self._refresh_sd_scope()

    def _current_baud(self) -> int:
        return int(self.baud_combo.currentData() or DEFAULT_BAUD)

    def _selected_port_is_bluetooth(self) -> bool:
        """True when the selected port is a Bluetooth SPP virtual COM port."""

        device = self.port_combo.currentData()
        if not device:
            return False
        table = getattr(self, "_port_is_bluetooth", {})
        if table.get(str(device)):
            return True
        # A saved port that the current scan has not described yet: the label
        # carries the Bluetooth hint, which is the only evidence available.
        text = self.port_combo.currentText().lower()
        return "bluetooth" in text or "bt port" in text

    def _link_packet_rate(self) -> int:
        """Packets per second the live link can carry.

        A Bluetooth COM port has no baud rate - pyserial accepts the value and the
        driver ignores it - so the budget must not be derived from the combo in
        that case. Using the baud here would throttle the sample rate for a limit
        that does not exist, which is worse than being approximate.
        """

        if self._selected_port_is_bluetooth():
            return max(int(self.bluetooth_budget_bytes_per_second() / 33), 1)
        return max_sustained_packet_rate(self._current_baud())

    def _effective_baud_for_budget(self) -> int:
        """The baud the shared helpers need to represent the selected transport.

        ``max_sustained_packet_rate()`` is expressed in baud, so the Bluetooth case
        is converted back to the baud that would carry the same packet rate.
        """

        if not self._selected_port_is_bluetooth():
            return self._current_baud()
        # 10 bits per byte, 90 % usable, 33-byte packets - the inverse of
        # max_sustained_packet_rate().
        packet_rate = self.bluetooth_budget_bytes_per_second() / 33.0
        return int(packet_rate * 33 * 10 / 0.9)

    def _refresh_link_note(self) -> None:
        """Explain what the baud setting means for the selected port."""

        label = getattr(self, "baud_note_label", None)
        if label is None:
            return
        if self._selected_port_is_bluetooth():
            label.setText(
                f"Bluetooth SPP: the baud rate is not used. The link runs at the radio's rate "
                f"(budget {self.bluetooth_budget_bytes_per_second() / 1024:.0f} kB/s)."
            )
            label.setStyleSheet("color: #b45309; font-size: 11px;")
        elif self.port_combo.currentData() == DEMO_PORT:
            label.setText("Demo source: no link.")
            label.setStyleSheet("color: #475569; font-size: 11px;")
        else:
            label.setText(
                f"USB: must match the firmware ({921600} baud). "
                f"Carries {max_sustained_packet_rate(self._current_baud())} packets/s."
            )
            label.setStyleSheet("color: #475569; font-size: 11px;")

    def _spi_sample_rate(self) -> int:
        devices = 2
        return max_sample_rate_from_spi(self.hardware.spi_clock_hz, devices)

    def _update_hardware_labels(self) -> None:
        board = "16 channels (Cyton + Daisy)" if self.hardware.board_channels > 8 else "8 channels (Cyton)"
        spi_clock = self.hardware.spi_clock_hz
        frame_us = spi_frame_microseconds(spi_clock, 2)
        spi_limit = self._spi_sample_rate()
        serial_limit = self._link_packet_rate()
        binding = "SPI" if spi_limit < self.hardware.data_rate_hz else "serial"
        modes = self.hardware.normalised_chip_modes()
        short = REFERENCE_MODE_SHORT_LABELS
        reference = (
            short.get(modes[0], modes[0])
            if modes[0] == modes[1]
            else f"ADS1 {short.get(modes[0], modes[0])} / ADS2 {short.get(modes[1], modes[1])}"
        )
        # A Bluetooth COM port has no baud rate, so the last line must not claim
        # one. Keeping it a separate variable avoids the conditional swallowing the
        # whole multi-line label through implicit string concatenation.
        if self._selected_port_is_bluetooth():
            link_line = f"Bluetooth SPP up to {serial_limit:g} packets/s  |  limit in force: {binding}"
        else:
            link_line = (
                f"Serial {self._current_baud()} baud up to {serial_limit:g} packets/s  |  "
                f"limit in force: {binding}"
            )
        text = (
            f"Board: {board}  |  reference {reference}\n"
            f"Saved: {self.hardware.data_rate_hz:g} SPS /{self.hardware.decimation} = "
            f"{self.hardware.output_packet_rate_hz:g} packets/s, {self.hardware.sample_rate_hz:g} Hz per channel\n"
            f"sinc3 -3 dB at {self.hardware.sinc_corner_hz:g} Hz  |  "
            f"SPI {spi_clock / 1e6:g} MHz ({frame_us:.0f} us/sample) up to {spi_limit:g} SPS\n"
            f"{link_line}"
        )
        self.board_label.setText(text)
        self.board_label.setToolTip(
            "Saved values are replayed on every connect. The 'Board reports' block below shows what the "
            "board itself says it is running, which only changes after a successful write or a Verify."
        )
        reported = getattr(self, "board_report_label", None)
        if reported is not None:
            reported.setText(self.board_report_text or "Board reports: connect and press 'Read board state'.")

    def open_hardware_settings(self) -> None:
        if self._settings_dialog is not None:
            self._settings_dialog.raise_()
            self._settings_dialog.activateWindow()
            return
        dialog = HardwareSettingsDialog(
            self.hardware,
            self.apply_status,
            self,
            link_limit=self._link_packet_rate(),
            reference_channel=str(self.view.reference_channel or ""),
        )
        dialog.applyRequested.connect(self._on_dialog_apply)
        dialog.verifyRequested.connect(self._on_dialog_verify)
        dialog.finished.connect(self._on_settings_dialog_closed)
        dialog.set_connected(self.connected)
        if self.verify_result_text:
            dialog.set_verify_result(self.verify_result_text, state=self.verify_result_state)
        self._settings_dialog = dialog
        dialog.show()

    def _on_settings_dialog_closed(self, *_args) -> None:
        self._settings_dialog = None

    def open_power_control(self) -> None:
        """Open the current-consumption test panel."""

        if self._power_dialog is not None:
            self._power_dialog.raise_()
            self._power_dialog.activateWindow()
            return
        # Only the Bluetooth ports are offered for the panel's own link: the USB
        # port is already held by this window, and a second handle on one COM port
        # is not possible.
        table = getattr(self, "_port_is_bluetooth", {})
        bt_ports = []
        for index in range(self.port_combo.count()):
            device = str(self.port_combo.itemData(index) or "")
            if device and table.get(device):
                bt_ports.append((self.port_combo.itemText(index), device))
        dialog = PowerControlDialog(self._send, self.connected, self, bluetooth_ports=bt_ports)
        dialog.finished.connect(self._on_power_dialog_closed)
        self._power_dialog = dialog
        dialog.show()
        if self.connected:
            # A read only. Opening this window must not change the board - the whole
            # point of a current measurement is that nothing moves except the thing
            # being switched. Everything else in the panel is an explicit action, and
            # "Restore converters to normal" is there for when a chip really is left
            # in a test state.
            dialog.refresh()

    def _on_power_dialog_closed(self, *_args) -> None:
        if self._power_dialog is not None:
            self._power_dialog.shutdown()
        self._power_dialog = None

    # -- Bluetooth throughput benchmark ------------------------------------

    def _set_benchmark_receiver(self, receiver: Optional[BenchmarkReceiver]) -> None:
        """Point the reader thread at (or away from) a benchmark receiver.

        Installed and removed from the GUI thread only, so the worker never sees a
        sink appear halfway through a chunk.
        """

        self._benchmark_receiver = receiver
        worker = getattr(self, "worker", None)
        if worker is None:
            return
        if receiver is None:
            worker.set_benchmark_sink(None)
        else:
            worker.set_benchmark_sink(receiver.feed)

    def open_bluetooth_benchmark(self) -> None:
        """Open the SPP throughput benchmark window (own top-level dialog)."""

        if self._benchmark_dialog is None:
            self._benchmark_dialog = BluetoothBenchmarkDialog(self)
            self._benchmark_dialog.finished.connect(self._on_benchmark_dialog_closed)
        self._benchmark_dialog.refresh_link_state()
        if self.connected:
            # Ask once for the board's state, so the dialog can tell an image with
            # the benchmark from one that would silently ignore it.
            self._send(b"|status;")
        self._benchmark_dialog.show()
        self._benchmark_dialog.raise_()
        self._benchmark_dialog.activateWindow()

    def _on_benchmark_dialog_closed(self, *_args) -> None:
        if self._benchmark_dialog is not None:
            self._benchmark_dialog.shutdown()
        self._set_benchmark_receiver(None)
        self._benchmark_dialog = None

    @staticmethod
    def _benchmark_sustainable_rate(
        result: Optional[BenchmarkResult],
        sweep: Sequence[tuple[int, BenchmarkResult]] = (),
    ) -> float:
        """The verified rate worth adopting, never the peak.

        From a sweep that is the highest step that passed every check. From a
        single run it is only adoptable if that run itself was stable - adopting
        the average of a run that was losing packets would bake the failure into
        every later feasibility calculation.
        """

        if sweep:
            passing = [rate for rate, item in sweep if is_sustainable(item)[0]]
            return float(max(passing)) if passing else 0.0
        if result is None:
            return 0.0
        ok, _reason = is_sustainable(result)
        return result.average_bytes_per_second if ok else 0.0

    def _on_power_status_line(self, text: str) -> None:
        """Route one ``[PWR] ... DONE=1`` line into the power panel."""

        if "DONE=1" not in text:
            return
        fields = dict(token.split("=", 1) for token in text[len("[PWR]"):].split() if "=" in token)
        self.power_state = fields
        if self._power_dialog is not None:
            self._power_dialog.apply_state(fields)

    def _on_dialog_verify(self, hardware: HardwareSettings) -> None:
        if self.recording is not None:
            QMessageBox.information(
                self, "Verify", "Stop the recording before verifying the configuration."
            )
            return
        self.verify_configuration(hardware)

    def _on_dialog_apply(self, hardware: HardwareSettings, indices: list) -> None:
        if self.recording is not None:
            QMessageBox.information(self, "Hardware settings", "Stop the recording before changing hardware settings.")
            return
        profile_changed = int(hardware.board_channels) != int(self.hardware.board_channels)
        previous_reference = self.hardware.normalised_reference_mode()
        self._set_hardware(hardware)
        if hardware.normalised_reference_mode() != previous_reference:
            self._refresh_session_name_reference()
        if profile_changed:
            self._apply_profile_change()
            return
        self.apply_hardware(indices)

    def restore_board_defaults(self) -> None:
        if not self.connected:
            QMessageBox.information(self, "Board defaults", "Connect the board first.")
            return
        if self.recording is not None:
            QMessageBox.information(self, "Board defaults", "Stop the recording first.")
            return
        answer = QMessageBox.question(
            self,
            "Board defaults",
            "Send the firmware defaults (gain x24, normal input, bias and SRB2 on) to all channels?\n\n"
            "The saved configuration will be reset to match.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        for index in real_channel_indices(self.hardware.board_channels):
            self.hardware.channels[index] = ChannelSetting()
        self._set_hardware(self.hardware)
        self._send(b"d")
        self._set_apply_summary("Board defaults sent; saved settings were reset to match.", success=True)

    def _apply_profile_change(self) -> None:
        self._rebuild_channel_checks()
        labels = [channel_label(index) for index in range(self.channel_count)]
        self.time_panel.ensure_channels(labels)
        self.spectrum_panel.ensure_channels(labels)
        self._rebuild_reference_combo()
        self._rebuild_buffers()
        self._restart_worker()

    def _change_board_profile(self, channels: int) -> None:
        self.hardware.set_board_channels(channels)
        self._set_hardware(self.hardware)
        self._apply_profile_change()

    # -- channel visibility and buffers ------------------------------------

    def _rebuild_channel_checks(self) -> None:
        while self.channel_check_layout.count() > 0:
            item = self.channel_check_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.channel_checkboxes = []
        mapping = physical_channel_map(self.hardware.board_channels)
        for index in range(self.channel_count):
            checkbox = QCheckBox(channel_label(index))
            visible = self.view.visible_channels[index] if index < len(self.view.visible_channels) else True
            checkbox.setChecked(visible)
            if index < len(mapping) and mapping[index] is None:
                checkbox.setChecked(False)
                checkbox.setEnabled(False)
                checkbox.setToolTip("Padding channel - no ADS input in this build")
            checkbox.toggled.connect(self._on_channel_visibility_changed)
            self.channel_check_layout.addWidget(checkbox, index // 3, index % 3)
            self.channel_checkboxes.append(checkbox)
        # The checkboxes are the truth (padding channels were forced off above),
        # so publish their state. Without this, padding channels could stay
        # "visible" from an older settings file and would be averaged into the
        # FFT as flat zero traces.
        self.view.visible_channels = [checkbox.isChecked() for checkbox in self.channel_checkboxes]
        self._update_record_scope_label()
        self._update_measurement_label()

    def _on_channel_visibility_changed(self, *_args) -> None:
        self.view.visible_channels = [checkbox.isChecked() for checkbox in self.channel_checkboxes]
        self._update_record_scope_label()
        # Clear the stale badge/warning of a channel that was just hidden instead
        # of waiting for the next health update to age it out.
        for index, state in list(self._channel_states.items()):
            if state != "ok" and index < len(self.view.visible_channels) and not self.view.visible_channels[index]:
                self._channel_states[index] = "ok"
        self.time_panel.set_channel_states(self._channel_states)
        self._update_channel_warning()
        self._save_settings()

    def _set_all_channels(self, checked: bool) -> None:
        for checkbox in self.channel_checkboxes:
            if checkbox.isEnabled():
                checkbox.setChecked(checked)
        self._update_record_scope_label()
        self._save_settings()

    def _rebuild_buffers(self) -> None:
        self.sample_rate = self.hardware.sample_rate_hz
        panel = getattr(self, "spectrum_panel", None)
        if panel is not None:
            panel.set_sample_rate(self.sample_rate)
        self.channel_count = 16 if self.hardware.board_channels > 8 else 8
        capacity = max(int(BUFFER_SECONDS * self.sample_rate), 512)
        self.raw_ring = RingBuffer(capacity, self.channel_count)
        self.display_ring = RingBuffer(capacity, self.channel_count)
        self.accel_ring = RingBuffer(int(BUFFER_SECONDS * 20.0), 3)
        self.filter = StreamingFilter(self.sample_rate, self.channel_count)
        self.filter.configure(self.view.filter_state())
        self.sample_total = 0
        self.packet_count = 0
        self.packet_loss = 0
        self._measured_rate = 0.0
        self._rate_history.clear()
        self._dc_state = None
        self._zero_seconds = np.zeros(self.channel_count, dtype=np.float64)
        self._rail_seconds = np.zeros(self.channel_count, dtype=np.float64)
        self._channel_states = {}
        self._stalled_channels = set()
        self._real_mask = None
        self.time_panel.set_channel_states({})
        self._update_channel_warning()
        self.time_panel.clear()
        self.spectrum_panel.clear()
    # -- data flow ---------------------------------------------------------

    def _on_batch(self, batch: dict) -> None:
        samples = np.asarray(batch.get("samples"), dtype=np.float64)
        aux = np.asarray(batch.get("aux"), dtype=np.float64)
        if samples.ndim != 2 or samples.shape[0] == 0:
            return
        if not self.streaming:
            self._set_streaming(True)

        count = samples.shape[0]
        start_index = self.sample_total
        try:
            self.raw_ring.append(samples)
        except ValueError:
            self._rebuild_buffers()
            return
        self.display_ring.append(self.filter.process(samples))
        self._track_signal_health(samples)

        if aux.ndim != 2 or aux.shape[1] != 3:
            aux = np.zeros((count, 3), dtype=np.float64)
        counts_per_g = max(float(self.view.accel_counts_per_g), 1.0)
        accel_g = aux[:count] / counts_per_g
        self.accel_ring.append(accel_g)
        self._last_aux_counts = aux[-1].copy()

        self.sample_total += count
        self.packet_count = int(batch.get("packets", 0) or 0)
        self.packet_loss = int(batch.get("loss", 0) or 0)
        now = time.monotonic()
        self._rate_history.append((now, self.sample_total))
        while len(self._rate_history) > 2 and now - self._rate_history[0][0] > 2.0:
            self._rate_history.popleft()

        if self.recording is not None and self.recording.is_active:
            usable = [index for index in self._record_channel_indices if index < samples.shape[1]]
            times = (np.arange(count, dtype=np.float64) + start_index - self.record_start_sample) / self.sample_rate
            self.recording.append(times, samples[:, usable], accel_g)

    def _real_channel_mask(self) -> np.ndarray:
        """True for GUI channels that map to a real ADS input."""

        if self._real_mask is None or self._real_mask.shape[0] != self.channel_count:
            mapping = physical_channel_map(self.hardware.board_channels)
            mask = np.zeros(self.channel_count, dtype=bool)
            for index in range(min(len(mapping), self.channel_count)):
                mask[index] = mapping[index] is not None
            self._real_mask = mask
        return self._real_mask

    def _channel_full_scale(self) -> np.ndarray:
        """Full-scale amplitude in uV per GUI channel (VREF/gain)."""

        limits = np.full(self.channel_count, VREF_VOLTS * 1e6 / 1.0)
        mapping = physical_channel_map(self.hardware.board_channels)
        for gui_index, real in enumerate(mapping):
            if real is None or gui_index >= min(len(mapping), self.channel_count):
                continue
            if gui_index < len(self.hardware.channels):
                gain = max(float(self.hardware.channels[gui_index].gain_scalar), 1.0)
                limits[gui_index] = VREF_VOLTS * 1e6 / gain
        return limits

    def _track_signal_health(self, samples: np.ndarray) -> None:
        """Track the sub-0.1 Hz mean and flag dead or saturated inputs."""

        count = int(samples.shape[0])
        if count <= 0 or self.sample_rate <= 0:
            return
        channels = int(samples.shape[1])
        if self._dc_state is None or self._dc_state.shape[0] != channels:
            self._dc_state = samples.mean(axis=0)
            self._dc_alpha = 1.0 - float(np.exp(-2.0 * np.pi * DC_CUTOFF_HZ / self.sample_rate))
            self._zero_seconds = np.zeros(channels, dtype=np.float64)
            self._rail_seconds = np.zeros(channels, dtype=np.float64)
        # Exponential average with a 0.1 Hz corner: one update per sample, so a
        # whole batch is folded in with the closed-form weight.
        decay = float((1.0 - self._dc_alpha) ** count)
        batch_mean = samples.mean(axis=0)
        self._dc_state = decay * self._dc_state + (1.0 - decay) * batch_mean

        duration = count / float(self.sample_rate)
        # Only inputs that are actually shown (ticked in the dock) are checked.
        # An input the operator hid is often one they deliberately left
        # unconnected, and warning about its rail or its zero samples buries the
        # warnings that matter.
        checked = self.view.visible_channels
        shown = np.array(
            [bool(checked[index]) if index < len(checked) else True for index in range(channels)],
            dtype=bool,
        )
        real = self._real_channel_mask()[:channels] & shown
        flat = np.all(samples == 0.0, axis=0) & real
        limits = self._channel_full_scale()[:channels]
        saturated = np.all(np.abs(samples) >= (RAIL_FRACTION * limits), axis=0) & real
        self._zero_seconds = np.where(flat, self._zero_seconds + duration, 0.0)
        self._rail_seconds = np.where(saturated, self._rail_seconds + duration, 0.0)
        # A hidden or padding input keeps no history, so re-showing it starts clean
        # instead of instantly reporting a stall it accumulated while hidden.
        self._zero_seconds = np.where(real, self._zero_seconds, 0.0)
        self._rail_seconds = np.where(real, self._rail_seconds, 0.0)

        states: dict[int, str] = {}
        for index in range(self.channel_count):
            if index >= channels:
                states[index] = "ok"
            elif not real[index]:
                states[index] = "ok"
            elif self._zero_seconds[index] >= STALL_SECONDS:
                states[index] = "stall"
            elif self._rail_seconds[index] >= RAIL_SECONDS:
                states[index] = "rail"
            else:
                states[index] = "ok"
        stalled = {index for index, state in states.items() if state != "ok"}
        if states != self._channel_states or stalled != self._stalled_channels:
            self._channel_states = states
            self._stalled_channels = stalled
            self.time_panel.set_channel_states(states)
            self._update_channel_warning()

    def _update_channel_warning(self) -> None:
        label = getattr(self, "channel_warning_label", None)
        if label is None:
            return
        # Never report an input the operator has hidden: they unticked it, so its
        # state is not part of the measurement any more.
        checked = self.view.visible_channels
        stalled = sorted(
            index
            for index, state in self._channel_states.items()
            if state == "stall" and (index >= len(checked) or checked[index])
        )
        railed = sorted(
            index
            for index, state in self._channel_states.items()
            if state == "rail" and (index >= len(checked) or checked[index])
        )
        lines: list[str] = []
        if stalled:
            lines.append(
                "No data from "
                + ", ".join(channel_label(index) for index in stalled)
                + ": the samples stay at exactly zero, so that input is not reaching the ADC."
            )
        if railed:
            lines.append(
                "Input saturated on "
                + ", ".join(channel_label(index) for index in railed)
                + ": the differential input is at the ADC rail (no valid reference or electrode off)."
            )
        text = "\n".join(lines)
        label.setText(text)
        label.setVisible(bool(text))

    def _on_text(self, lines) -> None:
        for line in lines:
            self._log("in", line.text, line.kind)
            if line.text.strip().startswith("[SD]"):
                self._on_sd_status_line(line.text.strip())
                continue
            if line.text.strip().startswith("[PWR]"):
                self._on_power_status_line(line.text.strip())
                continue
            if line.text.strip().startswith("[BENCH]"):
                # The board's own verdict on the throughput test: started,
                # stopped with its counters, or refused and why.
                self.benchmark_reports.append(line.text.strip())
                del self.benchmark_reports[:-40]
                if self._benchmark_dialog is not None:
                    self._benchmark_dialog.on_firmware_report(line.text.strip())
                continue
            if self._verify_pending and line.text.strip().startswith("[CFG]"):
                if self._on_verify_line(line.text):
                    self._finish_verify()
                continue
            if self._repair_pending and line.text.strip().startswith("[CHK]"):
                if self._on_repair_line(line.text):
                    self._finish_repair()
                continue
            if line.text.strip().startswith("[CHK]"):
                # A stream start re-programs the converters, so the firmware can
                # print a check without anybody asking; keep it visible.
                self._log("in", line.text, "info")
                continue
            bluetooth = BT_HEALTH_PATTERN.match(line.text)
            if bluetooth:
                self.bluetooth_health = dict(BT_FIELD_PATTERN.findall(bluetooth.group(1)))
            dx = DX_PATTERN.match(line.text)
            if dx:
                self.board_dx = dict(BT_FIELD_PATTERN.findall(dx.group(1)))
            gap = GAP_PATTERN.match(line.text)
            if gap:
                # A firmware-side acquisition gap, reported the moment it happens.
                # The window shows it so a step in the traces can be attributed
                # immediately: a step with no [GAP] line is not a timing problem.
                fields = dict(BT_FIELD_PATTERN.findall(gap.group(1)))
                self._gap_events += 1
                self._last_gap = fields
                gap_us = int(fields.get("GAP_US", "0") or 0)
                self.statusBar().showMessage(
                    f"Acquisition gap #{self._gap_events}: {gap_us / 1000.0:.2f} ms "
                    f"(period {int(fields.get('PERIOD_US', '0') or 0) / 1000.0:.2f} ms) - "
                    "a step in the traces at this moment is a firmware stall, otherwise it is analog.",
                    10000,
                )
            log_line = BT_LOG_PATTERN.match(line.text)
            if log_line:
                self.bluetooth_log.append(log_line.group(1).strip())
                del self.bluetooth_log[:-60]
            self._absorb_board_info(line.text)
            health = STREAM_HEALTH_PATTERN.search(line.text)
            if health:
                missed, dropped, corrupt = (int(value) for value in health.groups())
                self.stream_health = (missed, dropped, corrupt)
                self.statusBar().showMessage(
                    f"Board counters: missed {missed}, dropped {dropped}, corrupt {corrupt} frames", 15000
                )
            if self._apply_current is not None and line.kind in ("success", "error"):
                self._resolve_apply(line.kind == "success")

    def _on_ui_tick(self) -> None:
        self._ui_ticks += 1
        if self.sample_rate <= 0:
            return

        visible = list(self.view.visible_channels)
        if len(visible) < self.channel_count:
            visible.extend([True] * (self.channel_count - len(visible)))
        # Padding channels of the daisy emulation carry no data; never let them
        # into the plots or into the FFT average.
        real_mask = self._real_channel_mask()
        visible = [flag and index < real_mask.size and bool(real_mask[index]) for index, flag in enumerate(visible)]

        # The reference channel must not share the main axes: it carries the
        # stimulus, so it would flatten every other trace.
        reference_index = self.reference_channel_index()
        if reference_index is not None and reference_index < len(visible):
            visible[reference_index] = False
        if self._dc_state is not None:
            mask = self._real_channel_mask()
            self.time_panel.set_dc(
                {
                    index: float(self._dc_state[index])
                    for index in range(min(len(self._dc_state), self.channel_count))
                    if index < mask.size and mask[index] and index != reference_index
                }
            )

        window_count = int(self.time_panel.window_seconds * self.sample_rate)
        end_time = self.sample_total / self.sample_rate
        if not self.time_panel.paused:
            self.time_panel.render(self.display_ring.last(window_count), self.sample_rate, end_time, visible)

        if self._ui_ticks % FFT_INTERVAL_TICKS == 0:
            points = int(self.spectrum_panel.points_combo.currentData() or 1024)
            self.spectrum_panel.render(self.display_ring.last(points), self.sample_rate, visible)

        self._render_reference_panel()

        latest_accel = self.accel_ring.last(1)
        if latest_accel is not None and self.streaming:
            self.accel_panel.set_reading(latest_accel[-1])

        if self._ui_ticks % 4 == 0:
            self._flash_phase = not self._flash_phase
            self.accel_panel.set_flash(self._flash_phase)

        if len(self._rate_history) >= 2:
            first_time, first_count = self._rate_history[0]
            last_time, last_count = self._rate_history[-1]
            span = last_time - first_time
            if span > 0.5:
                self._measured_rate = (last_count - first_count) / span

        if self.recording is not None:
            self.record_time_label.setText(format_duration(self._record_elapsed()))
            name = self.record_path.name if self.record_path else ""
            self.record_stats_label.setText(f"{self.recording.rows_written:,} rows -> {name}")

        self._update_current_section_label()

        if self.streaming and self.sample_rate > 0 and self._measured_rate > 0.0:
            if abs(self._measured_rate - self.sample_rate) > 0.1 * self.sample_rate:
                self.warning_label.setText(self._rate_warning_text())
            elif self.warning_label.text():
                self.warning_label.setText("")

        zero_aux = self._last_aux_counts is not None and not np.any(self._last_aux_counts)
        self._accel_zero_ticks = self._accel_zero_ticks + 1 if (self.streaming and zero_aux) else 0
        self.accel_panel.set_note(
            "no accelerometer data - check the LIS3DH" if self._accel_zero_ticks > 60 else ""
        )

        stats = self._stats_text()
        for monitor in list(self.monitors):
            if monitor.isVisible():
                monitor.set_stats(stats)
        self._update_status_bar()

    def _rate_warning_text(self) -> str:
        """Explain a rate shortfall using the board's own numbers.

        The board reports what it produced (``#DX``); this window reports what it
        received. Comparing the two is the only way to tell a slow ESP32 from a
        lossy link, which need opposite fixes.
        """

        head = f"Receiving {self._measured_rate:.0f} of {self.sample_rate:.0f} Hz."
        board_rate = self._as_float(self.board_dx.get("ch_rate"))
        corrupt = self._as_int(self.board_dx.get("corrupt"))
        missed = self._as_int(self.board_dx.get("miss"))
        spi_max = self._as_int(self.board_dx.get("spi_max_us"))
        period_us = 1e6 / max(self.hardware.packet_rate_hz, 1.0)
        if board_rate > 0 and abs(board_rate - self.sample_rate) > 0.05 * self.sample_rate:
            late = self._as_int(self.board_dx.get("late"))
            gap_max = self._as_int(self.board_dx.get("gap_max_us"))
            detail = (
                f" The board itself only produced {board_rate:.0f} Hz"
                f" (missed conversions {missed}, late conversions {late}"
                f" with the worst gap {gap_max} us against a {period_us:.0f} us period,"
                f" corrupt frames {corrupt}, worst SPI read {spi_max} us)."
            )
            if late > 0 and gap_max > period_us:
                detail += (
                    " The acquisition task is being delayed by something else on the chip"
                    " - see the 'core'/'prio' fields, or lower the ADS rate to 250 SPS."
                )
            elif spi_max > 0.8 * period_us:
                detail += " Its SPI reads are running out of time: lower the ADS rate."
            else:
                detail += (
                    " The converters or the firmware are the limit, not the link"
                    " - check the [BOOT]/[BT] lines and the input wiring."
                )
            return head + detail
        return (
            head
            + " The board reports producing the full rate, so the samples are being lost on the way here"
            + (" - over Bluetooth, lower the ADS rate or enlarge the buffer with Q48Q."
               if self._link_is_bluetooth()
               else " - check the baud rate and the cable.")
        )

    def _link_is_bluetooth(self) -> bool:
        if self.bluetooth_health.get("mode") in ("bt", "both"):
            return True
        label = f"{self.port_combo.currentData() or ''} {self.port_combo.currentText()}".lower()
        return "bluetooth" in label

    @staticmethod
    def _as_float(value: object) -> float:
        try:
            return float(str(value))
        except (TypeError, ValueError):
            return 0.0

    def _stats_text(self) -> str:
        parts = [f"{self._measured_rate:.1f} Hz", f"{self.sample_total:,} samples", f"lost {self.packet_loss:,}"]
        if not self.connected:
            return "offline"
        return " | ".join(parts)

    def _update_status_bar(self) -> None:
        parts = ["connected" if self.connected else "offline"]
        if self.streaming:
            parts.append("streaming")
        if self.sample_total:
            parts.append(f"{self.sample_total:,} samples")
        if self._measured_rate > 0.0:
            parts.append(f"{self._measured_rate:.1f} Hz")
        if self.packet_loss:
            parts.append(f"lost {self.packet_loss:,}")
        board = self._board_health_text()
        if board:
            parts.append(board)
        link = self._link_health_text()
        if link:
            parts.append(link)
        if self.recording is not None:
            parts.append("recording")
        self.statusBar().showMessage(" | ".join(parts))

    def _board_health_text(self) -> str:
        """What the ESP32 itself produced, so a slow board is visible at a glance."""

        dx = self.board_dx
        if not dx or not self.streaming:
            return ""
        rate = dx.get("ch_rate")
        parts = [f"board {rate} Hz" if rate else "board ?"]
        missed = self._as_int(dx.get("miss"))
        late = self._as_int(dx.get("late"))
        corrupt = self._as_int(dx.get("corrupt"))
        spi_max = self._as_int(dx.get("spi_max_us"))
        gap_max = self._as_int(dx.get("gap_max_us"))
        period_us = 1e6 / max(self.hardware.packet_rate_hz, 1.0)
        if missed:
            parts.append(f"missed {missed}")
        if late:
            # The acquisition task was kept off the CPU: this is the number that
            # says the scheduling fix worked or did not.
            parts.append(f"late {late} (gap {gap_max} us of {period_us:.0f})")
        if corrupt:
            parts.append(f"corrupt {corrupt}")
        if spi_max and period_us and spi_max > 0.8 * period_us:
            parts.append(f"SPI {spi_max}/{period_us:.0f} us (too slow)")
        core = dx.get("core")
        prio = dx.get("prio")
        if core is not None and prio is not None:
            parts.append(f"core {core} prio {prio}")
        return " ".join(parts)

    def _link_health_text(self) -> str:
        """One line about the transport: USB, or Bluetooth with its quality."""

        health = self.bluetooth_health
        if not health:
            return ""
        mode = health.get("mode", "")
        if mode not in ("bt", "both"):
            return ""
        if health.get("client") != "1":
            return "BT waiting for a host"
        parts = ["BT"]
        sent = self._as_int(health.get("sent"))
        dropped = self._as_int(health.get("dropped"))
        if sent:
            # PER is what the user can act on; the ESP32 stack does not expose
            # per-packet bit errors, so this is the honest link figure.
            error_rate = 100.0 * float(dropped) / float(max(sent, 1))
            parts.append(f"PER {error_rate:.2f}%")
            parts.append(f"quality {max(0.0, 100.0 - error_rate):.1f}%")
        buffer_used = self._as_int(health.get("buf"))
        buffer_size = self._as_int(health.get("bufsize"))
        if buffer_size:
            parts.append(f"buffer {100.0 * buffer_used / buffer_size:.0f}%")
        # Peak fill and the measured send rate answer "does the radio keep up",
        # which is the question that decides whether both recordings can run.
        peak = self._as_int(health.get("peak"))
        if buffer_size and peak:
            parts.append(f"peak {100.0 * peak / buffer_size:.0f}%")
        rate = self._as_int(health.get("rate"))
        need = self._as_int(health.get("need"))
        if rate:
            parts.append(f"{rate / 1024.0:.1f} kB/s")
            if need:
                parts.append(f"need {need / 1024.0:.1f} kB/s")
        if health.get("txdbm"):
            parts.append(f"{health['txdbm']} dBm")
        rejected = self._as_int(health.get("rej"))
        if rejected:
            # The board only rejects a second client when its single SPP slot was
            # still held by a dead one, which is the classic "pairs once" failure.
            parts.append(f"stale drops {rejected}")
        write_failures = self._as_int(health.get("fail"))
        if write_failures > 500:
            parts.append(f"write retries {write_failures}")
        return " ".join(parts)

    @staticmethod
    def _as_int(value: object) -> int:
        try:
            return int(str(value), 0)
        except (TypeError, ValueError):
            return 0

    # -- recording ---------------------------------------------------------

    def _on_record_toggled(self, checked: bool) -> None:
        if checked:
            self.start_recording()
        else:
            self.stop_recording()

    def _set_record_button(self, checked: bool) -> None:
        self.record_button.blockSignals(True)
        self.record_button.setChecked(checked)
        self.record_button.blockSignals(False)
        self.record_button.setText("Stop recording" if checked else "Start recording")

    def _record_elapsed(self) -> float:
        if self.recording is None or self.sample_rate <= 0:
            return 0.0
        return max(0.0, (self.sample_total - self.record_start_sample) / self.sample_rate)

    def start_recording(self) -> None:
        if self.recording is not None:
            return
        if not self.connected:
            self._set_record_button(False)
            QMessageBox.information(self, "Recording", "Connect the board before recording.")
            return
        if not self.streaming:
            self.start_streaming()

        name = self.session_name_edit.text().strip() or "session"
        directory = self._output_directory()
        try:
            directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._set_record_button(False)
            QMessageBox.critical(self, "Recording", f"Could not create the recordings folder.\n\n{exc}")
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = directory / f"{stamp}_{sanitize_filename(name)}.csv"
        indices = self._record_indices()
        self._record_channel_indices = list(indices)
        labels = [f"EXG Channel {index}" for index in range(len(indices))]
        channel_numbers = [index + 1 for index in indices]
        gains = ",".join(
            GAIN_LABELS[min(self.hardware.channels[index].gain_ordinal, len(GAIN_LABELS) - 1)]
            for index in indices
            if index < len(self.hardware.channels)
        )
        board = (
            "ESP32 + 2x ADS1299-6 (Cyton+Daisy emulation)"
            if self.hardware.board_channels > 8
            else "ESP32 + 2x ADS1299 (Cyton emulation)"
        )
        metadata = {
            "Board": board,
            "Firmware": self.board_info.get("firmware", "unknown"),
            "Port": self.port_combo.currentText(),
            "Session": name,
            "Started": datetime.now().isoformat(timespec="seconds"),
            "Channel Map": ",".join(str(number) for number in channel_numbers),
            "Board Channel Gains": gains,
        }
        # Document the bench setup in the file itself so the analyzer never has
        # to ask for it again.
        measurement = self.measurement.normalised()
        measurement.reference_mode = self.hardware.normalised_reference_mode()
        self.measurement = measurement
        metadata.update(measurement.metadata())
        # A mixed reference cannot be described by the single "Reference Mode"
        # key above, so record each converter and every input that does not follow
        # its converter. The analyzer and the report both read these.
        chip_modes = self.hardware.normalised_chip_modes()
        for device, mode in enumerate(chip_modes):
            metadata[f"Reference ADS{device + 1}"] = REFERENCE_MODE_SHORT_LABELS.get(mode, mode)
        overrides = []
        for index in real_channel_indices(self.hardware.board_channels):
            if index >= self.channel_count or index >= len(self.hardware.channels):
                continue
            if not self.hardware.channels[index].active:
                continue
            short = self.hardware.channel_reference_short(index)
            per_device = channels_per_device(self.hardware.board_channels)
            device = min(index // max(per_device, 1), 1)
            if short != REFERENCE_MODE_SHORT_LABELS.get(chip_modes[device], chip_modes[device]) and short != "n/a":
                overrides.append(f"CH{index + 1}:{short}")
        if overrides:
            metadata["Reference Overrides"] = ", ".join(overrides)
        # Record why a capture might read as floating, so it is visible after the
        # fact instead of being re-discovered from the trace.
        reference_risks = self.hardware.reference_risks()
        if reference_risks:
            metadata["Reference Warning"] = " | ".join(reference_risks)
        reference_label = str(self.view.reference_channel or "").strip()
        if reference_label:
            # The analyzer needs this to know which trace is the stimulus: that
            # channel is volts, not microvolts, and must not be averaged with the
            # channels under test.
            metadata["Reference Channel"] = reference_label
            metadata["Reference Channel Gain"] = "x1"
        # The input divider is part of the bench setup, not of the sweep: a gain
        # computed from a manual run needs it just as much as an automated one.
        metadata["Input Attenuator Series (ohm)"] = f"{self.sweep_settings.attenuator_series_ohms:.6g}"
        metadata["Input Attenuator Shunt (ohm)"] = f"{self.sweep_settings.attenuator_shunt_ohms:.6g}"
        metadata["Input Attenuator Ratio"] = f"{self.sweep_settings.attenuator_ratio:.6g}"
        generator_output = self._generator_output()
        if generator_output.mode != OUTPUT_OFF:
            metadata["Generator Output"] = generator_output.describe()
        self.recording = OpenBCIRecording(path, labels, channel_numbers, self.sample_rate, metadata)
        try:
            self.recording.start()
        except OSError as exc:
            self.recording = None
            self._set_record_button(False)
            QMessageBox.critical(self, "Recording", f"Could not open the recording file.\n\n{exc}")
            return

        self.record_path = path
        self.record_start_sample = self.sample_total
        self.record_started_at = time.monotonic()
        self.annotations = []
        self._open_annotation = None
        self._refresh_annotations()
        self._set_record_button(True)
        self.record_time_label.setText("00:00.00")
        self.record_stats_label.setText(f"writing {path.name}")
        self._log("in", f"[recording] started {path}", "info")

    def send_recording_marker(self) -> None:
        """Stamp the same instant into the live file and the card recording.

        The board writes Marker=1 into its next SD row, this file marks its own next
        rows, and the comparison tool then knows the offset between the two
        timelines without having to guess it from the waveforms. Correlation alone
        is ambiguous for anything periodic, which is exactly what a test tone or
        mains pickup is.
        """

        if self.recording is None:
            return
        # A few rows, not one: the live file is written from whole packets, so the
        # marked rows may straddle the instant the board actually acts on.
        self.recording.mark(2)
        if self.connected:
            self._send(b"`")
    def stop_recording(self) -> None:
        if self.recording is None:
            self._set_record_button(False)
            return
        if self._open_annotation is not None:
            self._open_annotation.end_s = self._record_elapsed()
            self._open_annotation = None
        elapsed = self._record_elapsed()
        recording = self.recording
        self.recording = None
        path = recording.stop(
            self.annotations,
            {
                "Stopped": datetime.now().isoformat(timespec="seconds"),
                "Duration": f"{elapsed:.2f} s",
                "Rows": str(recording.rows_written),
                # A generator sweep only knows its schedule once it has run, and
                # the CSV header is flushed before the first sample, so the times
                # travel in the sidecar. read_openbci_metadata() merges them.
                **self._sweep_metadata(),
            },
        )
        self.last_recording = path
        self.analyze_button.setEnabled(True)
        self._set_record_button(False)
        self.record_stats_label.setText(
            f"saved {path.name} - {recording.rows_written:,} rows in {format_duration(elapsed)}"
        )
        self._refresh_annotations()
        # The schedule has been written to the sidecar; a later recording must not
        # inherit it.
        self._sweep_plan = None
        self._sweep_steps = []
        self._log("in", f"[recording] saved {path}", "info")
        if recording.error:
            QMessageBox.warning(
                self,
                "Recording",
                f"The recording writer reported a problem:\n\n{recording.error}",
            )

    # -- annotations -------------------------------------------------------

    def add_annotation(self) -> None:
        if self.recording is None:
            QMessageBox.information(
                self,
                "Sections",
                "Start a recording first - sections document time ranges inside the recording.",
            )
            return
        label, accepted = QInputDialog.getText(
            self,
            "Start section",
            "Name this section (for example 'baseline', 'eyes closed', 'P300 block 1'):",
            text=f"section {len(self.annotations) + 1}",
        )
        if not accepted:
            return
        label = label.strip() or f"section {len(self.annotations) + 1}"
        now = self._record_elapsed()
        if self._open_annotation is not None:
            self._open_annotation.end_s = now
        annotation = Annotation(label=label, start_s=now, end_s=now)
        self.annotations.append(annotation)
        self._open_annotation = annotation
        self._refresh_annotations()
        self._log("in", f"[section] '{label}' starts at {format_duration(now)}", "info")

    def end_annotation(self) -> None:
        if self._open_annotation is None:
            return
        self._open_annotation.end_s = self._record_elapsed()
        self._open_annotation = None
        self._refresh_annotations()
        self._persist_annotations()

    def edit_annotation(self) -> None:
        row = self.annotation_list.currentRow()
        if row < 0 or row >= len(self.annotations):
            return
        annotation = self.annotations[row]
        dialog = AnnotationDialog(annotation, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            updated = dialog.values()
            annotation.label = updated.label
            annotation.note = updated.note
            annotation.start_s = updated.start_s
            annotation.end_s = updated.end_s
            self._refresh_annotations()
            self._persist_annotations()

    def remove_annotation(self) -> None:
        row = self.annotation_list.currentRow()
        if row < 0 or row >= len(self.annotations):
            return
        annotation = self.annotations.pop(row)
        if annotation is self._open_annotation:
            self._open_annotation = None
        self._refresh_annotations()
        self._persist_annotations()

    def _refresh_annotations(self) -> None:
        self.annotation_list.clear()
        for annotation in self.annotations:
            text = f"{format_duration(annotation.start_s)} - {format_duration(annotation.end_s)}   {annotation.label}"
            item = QListWidgetItem(text)
            if annotation.note:
                item.setToolTip(annotation.note)
            if annotation is self._open_annotation:
                item.setForeground(QColor("#b45309"))
                item.setToolTip((item.toolTip() + "\n" if item.toolTip() else "") + "section still open")
            self.annotation_list.addItem(item)
        recording = self.recording is not None
        self.add_section_button.setEnabled(recording)
        self.end_section_button.setEnabled(recording and self._open_annotation is not None)
        self.edit_section_button.setEnabled(bool(self.annotations))
        self.remove_section_button.setEnabled(bool(self.annotations))
        self.analyze_section_button.setEnabled(bool(self.annotations) and self.last_recording is not None and not recording)
        self._update_current_section_label()

    def _update_current_section_label(self) -> None:
        if self._open_annotation is None:
            if self.recording is not None:
                self.current_section_label.setText("No section open")
            else:
                self.current_section_label.setText("Sections document time ranges inside a recording")
            return
        elapsed = max(0.0, self._record_elapsed() - self._open_annotation.start_s)
        self.current_section_label.setText(
            f"Open section: {self._open_annotation.label}   ({format_duration(elapsed)})"
        )

    def _persist_annotations(self) -> None:
        if self.last_recording is None or self.recording is not None:
            return
        try:
            save_annotation_sidecar(self.last_recording, self.annotations)
        except OSError as exc:
            self.statusBar().showMessage(f"Could not save the sections file: {exc}", 6000)

    def analyze_section(self) -> None:
        row = self.annotation_list.currentRow()
        if row < 0 or row >= len(self.annotations):
            QMessageBox.information(self, "Sections", "Select a section in the list first.")
            return
        if self.recording is not None:
            QMessageBox.information(self, "Sections", "Stop the recording first so the file is complete.")
            return
        if self.last_recording is None:
            QMessageBox.information(self, "Sections", "Record something first.")
            return
        annotation = self.annotations[row]
        if annotation.end_s <= annotation.start_s:
            QMessageBox.information(self, "Sections", "This section has no duration yet - press End first.")
            return
        self.open_analyzer(load_last=True, selection=(annotation.start_s, annotation.end_s))

    # -- auxiliary windows -------------------------------------------------

    def attach_analyzer(self, window) -> None:
        """Share an existing analyzer window instead of opening another one."""

        self._analyzer_window = window

    def show_bluetooth_report(self) -> None:
        """Ask the board for its Bluetooth state and show it in one dialog.

        The board reports over the cable as well as the radio, so this works
        precisely when Bluetooth is the thing that is misbehaving.
        """

        if not self.connected:
            QMessageBox.information(
                self,
                "Bluetooth report",
                "Connect to the board first - over USB if Bluetooth is the problem, because the board "
                "reports Bluetooth state on both links.",
            )
            return
        self._send(b"T")
        self._send(b"B")
        QTimer.singleShot(900, self._present_bluetooth_report)

    def _present_bluetooth_report(self) -> None:
        health = dict(self.bluetooth_health)
        lines: list[str] = []
        if health:
            mode = health.get("mode", "?")
            lines.append(f"Mode: {mode}   Client attached: {'yes' if health.get('client') == '1' else 'no'}")
            lines.append(
                f"Confirmed pairs: opens {health.get('opens', '?')}, closes {health.get('closes', '?')}, "
                f"closed by the PC {health.get('remote', '?')}, rejected {health.get('rej', '?')}"
            )
            lines.append(
                f"Board boots {health.get('boots', '?')}, uptime {health.get('up', '?')}   "
                f"free heap {health.get('heap', '?')} bytes"
            )
            lines.append(
                f"Stalled writes {health.get('fail', '?')}, buffer {health.get('buf', '?')}/"
                f"{health.get('bufsize', '?')} bytes, sent {health.get('sent', '?')}, "
                f"dropped {health.get('dropped', '?')}"
            )
            lines.append(f"Name {health.get('name', '?')}, TX power {health.get('txdbm', '?')} dBm")
        if self.board_dx:
            dx = self.board_dx
            period_us = 1e6 / max(self.hardware.packet_rate_hz, 1.0)
            lines.append("")
            lines.append(
                f"Acquisition: {dx.get('ch_rate', '?')} Hz per channel, "
                f"{dx.get('drdy_s', '?')} converter edges/s"
            )
            lines.append(
                f"  missed conversions {dx.get('miss', '?')}, corrupt frames {dx.get('corrupt', '?')}, "
                f"worst SPI read {dx.get('spi_max_us', '?')} us of {period_us:.0f} us"
            )
            lines.append(
                f"  USB headroom {dx.get('usb_free', '?')} bytes, radio buffer {dx.get('bt_buf', '?')} bytes"
            )
            boots = self._as_int(health.get("boots"))
            if boots > 1:
                lines.append("")
                lines.append(
                    f"This boot is number {boots} since power was applied. If that number grows while you "
                    "connect over Bluetooth, the board is restarting - read the [BOOT] reason in the serial "
                    "monitor (BROWNOUT means the supply, PANIC means the firmware)."
                )
        else:
            lines.append("The board has not reported its Bluetooth state yet.")
            lines.append("Firmware older than the Bluetooth release does not report it at all.")
        if self.bluetooth_log:
            lines.append("")
            lines.append("Recent events (newest last):")
            lines.extend(f"  {entry}" for entry in self.bluetooth_log[-14:])
        else:
            lines.append("")
            lines.append("No events recorded yet. Reproduce the disconnect, then open this report again.")
        lines.append("")
        lines.append(
            "remote=1 on a close means the PC dropped the link; remote=0 means the board did. A rejected "
            "open means the board still tracked a dead client, which it repairs by itself."
        )
        dialog = QMessageBox(self)
        dialog.setWindowTitle("Bluetooth report")
        dialog.setIcon(QMessageBox.Icon.Information)
        dialog.setText("\n".join(lines))
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.exec()

    def open_monitor(self) -> None:
        self.monitors = [monitor for monitor in self.monitors if monitor.isVisible()]
        monitor = SerialMonitorWindow(self)
        monitor.sendRequested.connect(self._send)
        monitor.set_stats(self._stats_text())
        for record in self._text_backlog:
            monitor.append_record(*record)
        monitor.show()
        self.monitors.append(monitor)

    def open_analyzer(self, load_last: bool = False, selection: Optional[tuple[float, float]] = None) -> None:
        from gui_main import ScientificSignalAnalyzerWindow

        if self._analyzer_window is None:
            self._analyzer_window = ScientificSignalAnalyzerWindow()
        self._analyzer_window.show()
        self._analyzer_window.raise_()
        self._analyzer_window.activateWindow()
        if load_last and self.last_recording is not None:
            self._analyzer_window.open_csv_path(self.last_recording, selection=selection)

    # -- automated function-generator sweep -------------------------------

    def open_auto_sweep(self) -> None:
        if self._sweep_dialog is not None:
            self._sweep_dialog.raise_()
            self._sweep_dialog.activateWindow()
            return
        dialog = AutoSweepDialog(self.sweep_settings, self._measurement_channel_names(), self)
        # The dock owns the reference/monitor choice; the dialog mirrors it.
        dialog.set_monitor_channel(str(self.view.reference_channel or ""))
        dialog.startRequested.connect(self._start_auto_sweep)
        dialog.stopRequested.connect(self._stop_auto_sweep)
        dialog.finished.connect(self._on_sweep_dialog_closed)
        if self._sweep_worker is not None:
            dialog.set_running(True)
        self._sweep_dialog = dialog
        dialog.show()

    def _on_sweep_dialog_closed(self, *_args) -> None:
        self._sweep_dialog = None

    def _measurement_channel_names(self) -> list[str]:
        """Channel names offered as the common-mode monitor."""

        names: list[str] = []
        for index in real_channel_indices(self.hardware.board_channels):
            if index < self.channel_count:
                names.append(channel_label(index))
        return names

    def _start_auto_sweep(self, settings: SweepSettings) -> None:
        if self._sweep_worker is not None:
            self._set_sweep_status("A sweep is already running.", error=True)
            return
        if not self.connected or self.worker is None:
            self._set_sweep_status("Connect the board first: the sweep has to record while it runs.", error=True)
            return
        steps = settings.steps()
        if not steps:
            self._set_sweep_status("The frequency list is empty.", error=True)
            return

        # Save the plan first so the frequency list survives a restart.
        self.sweep_settings = settings
        # Keep the manual panel honest about what the sweep is doing to the
        # generator, so the dock and the instrument never disagree.
        settings.output_mode = OUTPUT_DIFFERENTIAL if SweepDrive(settings.drive_kind).uses_channel_2 else OUTPUT_SINGLE
        if steps:
            settings.frequency_hz = float(steps[0].frequency_hz)
        self.settings.sweep = settings.as_dict()
        self._save_settings()

        try:
            if not self.generator.is_open:
                identity = self.generator.open(settings.resource or None)
                self.sweep_settings.resource = self.generator.resource
                self.settings.sweep = self.sweep_settings.as_dict()
                self._save_settings()
                self._log("in", f"[sweep] generator connected: {identity}", "info")
                if self.generator.blind:
                    self._log(
                        "in",
                        "[sweep] the generator answers no queries; commands are applied but the "
                        "readback cannot be verified",
                        "error",
                    )
            self.generator.unlock_front_panel()
            self.generator.set_output_load(1, settings.load_ohms)
            if SweepDrive(settings.drive_kind).uses_channel_2:
                self.generator.set_output_load(2, settings.load_ohms)
                self.generator.set_phase_mode(True)
            else:
                self.generator.set_phase_mode(False)
        except UTGError as exc:
            self._set_sweep_status(f"Generator problem: {exc}", error=True)
            return

        drive = SweepDrive(kind=settings.drive_kind)
        self._sweep_plan = SweepPlan(
            drive=drive,
            amplitude_vrms=settings.amplitude_vrms,
            monitor_channel=settings.monitor_channel,
            steps=steps,
            attenuator_series_ohms=settings.attenuator_series_ohms,
            attenuator_shunt_ohms=settings.attenuator_shunt_ohms,
        )
        self._sweep_steps = steps

        # Recording first: the sweep schedule is expressed in recording time, so
        # the analyzer can cut exactly the slice each frequency occupied.
        if self.recording is None:
            self.start_recording()
            if self.recording is None:
                self._set_sweep_status("Could not start the recording.", error=True)
                return
            self._sweep_owns_recording = True
        else:
            self._sweep_owns_recording = False

        worker = AutoSweepWorker(self.generator, self._sweep_plan, steps, self)
        worker.progress.connect(self._on_sweep_progress)
        worker.stepApplied.connect(self._on_sweep_step_applied)
        worker.warning.connect(self._on_sweep_warning)
        worker.failed.connect(self._on_sweep_failed)
        worker.completed.connect(self._on_sweep_completed)
        self._sweep_worker = worker
        self._sweep_warnings = []
        if self._sweep_dialog is not None:
            self._sweep_dialog.set_running(True)
            self._sweep_dialog.set_status(
                f"Running {len(steps)} steps, {sum(step.dwell_s for step in steps):.1f} s total."
            )
        self._log(
            "in",
            f"[sweep] {drive.label}, {settings.amplitude_vrms:g} Vrms applied "
            f"({per_channel_amplitude_vrms(settings.amplitude_vrms, drive.uses_channel_2):g} Vrms per channel), "
            f"divider {settings.attenuator_ratio:.4g}, {len(steps)} steps",
            "info",
        )
        # The dock shows the same instrument, so keep its fields in step with the
        # plan the sweep just adopted.
        self.sweep_settings = settings
        self._generator_sync_from_settings()
        worker.start()

    def _stop_auto_sweep(self) -> None:
        worker = self._sweep_worker
        if worker is None:
            return
        worker.stop()
        self._set_sweep_status("Stopping the generator...")

    def _set_sweep_status(self, text: str, error: bool = False) -> None:
        if self._sweep_dialog is not None:
            self._sweep_dialog.set_status(text, error=error)
        self._log("in", f"[sweep] {text}", "error" if error else "info")

    def _on_sweep_progress(self, text: str) -> None:
        if self._sweep_dialog is not None:
            self._sweep_dialog.set_status(text)

    def _on_sweep_warning(self, message: str) -> None:
        """The generator's readback disagreed with what was just programmed."""

        self._sweep_warnings.append(message)
        self._log("in", f"[sweep] readback problem: {message}", "error")

    def _on_sweep_step_applied(self, index: int) -> None:
        """Stamp the moment a frequency reached the board, in recording time."""

        if index >= len(self._sweep_steps):
            return
        step = self._sweep_steps[index]
        started = self._record_elapsed()
        # The generator needs a moment to settle after the frequency change, so
        # the analyzer starts a little into the hold instead of on the edge.
        settle = min(0.15, 0.25 * step.dwell_s)
        step.start_s = started
        step.settle_s = started + settle
        step.end_s = started + step.dwell_s

    def _on_sweep_failed(self, message: str) -> None:
        self._set_sweep_status(f"Sweep failed: {message}", error=True)
        self._finish_sweep(failed=True)

    def _on_sweep_completed(self) -> None:
        warnings = list(self._sweep_warnings)
        if warnings:
            self._set_sweep_status(
                f"Sweep finished with {len(warnings)} generator readback problem(s): {warnings[0]}",
                error=True,
            )
        else:
            self._set_sweep_status("Sweep finished; every step was confirmed by the generator.")
        self._finish_sweep(failed=False)

    def _finish_sweep(self, failed: bool) -> None:
        worker = self._sweep_worker
        self._sweep_worker = None
        if worker is not None:
            worker.wait(3000)
        try:
            self.generator.all_off()
        except UTGError:
            pass
        if self._sweep_dialog is not None:
            self._sweep_dialog.set_running(False)
        owns = self._sweep_owns_recording
        self._sweep_owns_recording = False
        if owns and self.recording is not None:
            self.stop_recording()
        if failed:
            self._sweep_plan = None

    def _sweep_metadata(self) -> dict[str, str]:
        """Sweep schedule for the recording sidecar (the analyzer reads it back)."""

        plan = self._sweep_plan
        if plan is None or not self._sweep_steps:
            return {}
        plan.steps = [
            step for step in self._sweep_steps if step.end_s > step.start_s
        ]
        if not plan.steps:
            return {}
        return plan.metadata()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._autoconnect_timer.stop()
        self._open_watchdog.stop()
        self._reconnect_timer.stop()
        if self._power_dialog is not None:
            self._power_dialog.shutdown()
            self._power_dialog.close()
            self._power_dialog = None
        self._save_settings()
        if self._sweep_worker is not None:
            self._sweep_worker.stop()
            self._sweep_worker.wait(3000)
            self._sweep_worker = None
        if self._generator_worker is not None:
            self._generator_worker.wait(3000)
            self._generator_worker = None
        try:
            self.generator.all_off()
            self.generator.close()
        except UTGError:
            pass
        if self.recording is not None:
            self.stop_recording()
        if self.connected or self.worker is not None:
            # A worker can outlive `connected` (a failed or abandoned attempt), and
            # a port left open here makes the next run fail with "Access is denied".
            self._disconnect()
        for monitor in list(self.monitors):
            monitor.close()
        self._ui_timer.stop()
        scanner = getattr(self, "_port_scanner", None)
        if scanner is not None:
            scanner.stop()
        event.accept()


class BluetoothBenchmarkDialog(QDialog):
    """Measures the real application-level throughput of the Bluetooth SPP link.

    The firmware generates frames in memory and pushes them through the production
    transmission path; this window reads them off the same COM port the recorder
    uses, proves each one, and reports what actually arrived. Nothing here talks
    to the ADS1299: the converters are not read, so the number is the link and only
    the link.

    Three figures are shown side by side on purpose - verified, stack-accepted and
    queued - because a link that accepts bytes and delivers none is exactly the
    failure this test exists to catch.
    """

    MODE_FIXED = "fixed"
    MODE_MAX = "max"
    MODE_SWEEP = "sweep"

    def __init__(self, window: "LiveRecorderWindow") -> None:
        super().__init__(window)
        self.setWindowTitle("Bluetooth throughput benchmark")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(1080, 760)
        self._window = window
        self._receiver = BenchmarkReceiver()
        self._running = False
        self._run_mode = ""
        self._run_rate = 0
        self._run_payload = 0
        self._sweep_index = 0
        self._sweep_results: list[tuple[int, BenchmarkResult]] = []
        self._sweep_phase = "idle"
        self._sweep_phase_until = 0.0
        self._last_plot_s = 0.0
        self._plot_ymax = 0.0
        self._bt_link = None
        self._last_result: Optional[BenchmarkResult] = None
        self._relink_attempts = 0

        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._tick)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        self.link_label = QLabel("")
        self.link_label.setWordWrap(True)
        layout.addWidget(self.link_label)

        layout.addWidget(self._build_link_group())
        layout.addWidget(self._build_controls())
        layout.addWidget(self._build_stats(), 0)
        layout.addWidget(self._build_plot(), 1)

        self.summary_label = QLabel("Idle. Press Start to measure the link.")
        self.summary_label.setWordWrap(True)
        layout.addWidget(self.summary_label)

        # Live firmware diagnostics: one line, refreshed every second while a test
        # runs. Text is selectable so it can be pasted back verbatim.
        self.diag_label = QLabel("Board diagnostics appear here while a test runs.")
        self.diag_label.setWordWrap(True)
        self.diag_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.diag_label.setStyleSheet("color: #94a3b8; font-size: 11px;")
        layout.addWidget(self.diag_label)

        row = QHBoxLayout()
        self.save_json_button = QPushButton("Save JSON...")
        self.save_json_button.clicked.connect(lambda: self._save_result("json"))
        self.save_json_button.setEnabled(False)
        row.addWidget(self.save_json_button)
        self.save_csv_button = QPushButton("Save CSV...")
        self.save_csv_button.clicked.connect(lambda: self._save_result("csv"))
        self.save_csv_button.setEnabled(False)
        row.addWidget(self.save_csv_button)
        self.apply_budget_button = QPushButton("Use measured rate as link budget...")
        self.apply_budget_button.setToolTip(
            "Adopt the verified sustainable rate (with a safety margin) as the capacity the "
            "sample-rate planner assumes for Bluetooth. Peak throughput is never adopted."
        )
        self.apply_budget_button.clicked.connect(self._apply_measured_budget)
        self.apply_budget_button.setEnabled(False)
        row.addWidget(self.apply_budget_button)
        row.addStretch(1)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.close)
        row.addWidget(close_button)
        layout.addLayout(row)

        self.refresh_link_state()

    # -- construction ------------------------------------------------------

    def _build_link_group(self) -> QWidget:
        """The benchmark's own Bluetooth connection.

        The whole point of this group is that the cable stays free: holding USB for
        commands and Bluetooth for the measured data means the board is controllable
        while the radio is saturated, and a command can never queue behind 200 KiB/s
        of test traffic. It also keeps the board in Bluetooth mode, which it leaves
        when nothing is using the SPP link.
        """

        group = QGroupBox("Benchmark link")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Data link"))
        self.data_link_combo = QComboBox()
        self.data_link_combo.addItem("Main window link (already open)", "main")
        self.data_link_combo.addItem("This window's own Bluetooth port", "bt")
        self.data_link_combo.addItem("USB (reference only)", "usb")
        self.data_link_combo.setToolTip(
            "Where the benchmark bytes are received. The board serves one Bluetooth client at a "
            "time, so when the main window is already streaming over Bluetooth the test has to "
            "read from that same connection - opening a second one makes the firmware drop the "
            "first. Use this window's own port when the cable carries the commands."
        )
        self.data_link_combo.currentIndexChanged.connect(self._on_data_link_changed)
        layout.addWidget(self.data_link_combo)

        layout.addWidget(QLabel("Port"))
        self.bt_port_combo = QComboBox()
        self.bt_port_combo.setMinimumWidth(150)
        layout.addWidget(self.bt_port_combo)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_link)
        layout.addWidget(self.connect_button)

        self.link_state_label = QLabel("Not connected")
        self.link_state_label.setStyleSheet("color: #b91c1c;")
        layout.addWidget(self.link_state_label, 1)
        return group

    def _build_controls(self) -> QWidget:
        group = QGroupBox("Test")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Mode"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Maximum throughput", self.MODE_MAX)
        self.mode_combo.addItem("Fixed rate", self.MODE_FIXED)
        self.mode_combo.addItem("Automatic sweep", self.MODE_SWEEP)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        layout.addWidget(self.mode_combo)

        layout.addWidget(QLabel("Duration"))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(2.0, 600.0)
        self.duration_spin.setDecimals(0)
        self.duration_spin.setValue(30.0)
        self.duration_spin.setSuffix(" s")
        layout.addWidget(self.duration_spin)

        layout.addWidget(QLabel("Payload"))
        self.payload_combo = QComboBox()
        for size in PAYLOAD_CHOICES:
            self.payload_combo.addItem(f"{size} B", size)
        self.payload_combo.setCurrentIndex(max(self.payload_combo.findData(1024), 0))
        self.payload_combo.setToolTip(
            "Bytes of payload per frame. Larger frames mean fewer write calls; 990 is the "
            "classic SPP-friendly size, 1024 matches the firmware's flush chunk."
        )
        layout.addWidget(self.payload_combo)

        layout.addWidget(QLabel("Target rate"))
        self.rate_combo = QComboBox()
        for rate, label in RATE_CHOICES:
            self.rate_combo.addItem(label, rate)
        self.rate_combo.setCurrentIndex(max(self.rate_combo.findData(64 * 1024), 0))
        layout.addWidget(self.rate_combo)

        layout.addWidget(QLabel("Sweep"))
        self.sweep_settle_spin = QDoubleSpinBox()
        self.sweep_settle_spin.setRange(1.0, 60.0)
        self.sweep_settle_spin.setDecimals(0)
        self.sweep_settle_spin.setValue(3.0)
        self.sweep_settle_spin.setSuffix(" s settle")
        layout.addWidget(self.sweep_settle_spin)
        self.sweep_measure_spin = QDoubleSpinBox()
        self.sweep_measure_spin.setRange(1.0, 120.0)
        self.sweep_measure_spin.setDecimals(0)
        self.sweep_measure_spin.setValue(5.0)
        self.sweep_measure_spin.setSuffix(" s measure")
        layout.addWidget(self.sweep_measure_spin)

        layout.addWidget(QLabel("SPP write"))
        self.spp_chunk_combo = QComboBox()
        for size, label in SPP_WRITE_CHOICES:
            self.spp_chunk_combo.addItem(label, size)
        preferred = int(
            getattr(getattr(self._window, "hardware", None), "spp_write_bytes", 330) or 330
        )
        self.spp_chunk_combo.setCurrentIndex(max(self.spp_chunk_combo.findData(preferred), 0))
        self.spp_chunk_combo.setToolTip(
            "Bytes per SPP write. Measured on this board: 330 through the Arduino library "
            "sustained 88-99 KiB/s and lost under 4% of frames, while the direct write at "
            "990 B managed 1.3 KiB/s and at 4096 B about 11 KiB/s.\n\n"
            "The library queues each chunk and returns, so its own task keeps the radio fed "
            "while the sample ring refills - it is pipelined. The direct path has to wait for "
            "every acknowledgement before it may write again, so its chunk divided by that "
            "round trip is the whole ceiling.\n\n"
            "330 is therefore the default and the fastest option. The larger sizes are kept "
            "as an experimental path for links where the acknowledgement is much faster than "
            "this one's."
        )
        layout.addWidget(self.spp_chunk_combo)

        self.start_button = QPushButton("Start benchmark")
        self.start_button.clicked.connect(self.start_benchmark)
        layout.addWidget(self.start_button)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(lambda: self.stop_benchmark("stopped by user"))
        layout.addWidget(self.stop_button)
        layout.addStretch(1)

        self._on_mode_changed()
        return group

    def _build_stats(self) -> QWidget:
        group = QGroupBox("Live")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(14)
        self._stat_labels: dict[str, QLabel] = {}

        rows = [
            ("current", "Current"),
            ("average", "Average"),
            ("peak", "Peak (1 s)"),
            ("verified", "Verified payload"),
            ("received", "Received"),
            ("raw", "Raw bytes"),
            ("packets", "Data packets"),
            ("corrupt", "Corrupt / bad CRC"),
            ("missing", "Missing"),
            ("sequence", "Sequence errors"),
            ("queue", "Firmware queue"),
            ("queue_peak", "Peak queue"),
            ("stalls", "Transmission stalls"),
            ("congestion", "SPP congestion"),
            ("spopath", "Write path"),
            ("handle", "SPP handle"),
            ("elapsed", "Elapsed"),
            ("verdict", "Status"),
        ]
        for index, (key, label) in enumerate(rows):
            column = (index % 5) * 2
            row = index // 5
            grid.addWidget(QLabel(label), row, column)
            value = QLabel("-")
            value.setStyleSheet("font-weight: 600;")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            grid.addWidget(value, row, column + 1)
            self._stat_labels[key] = value
        return group

    def _build_plot(self) -> QWidget:
        self.plot = pg.PlotWidget()
        self.plot.setMinimumHeight(230)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Throughput", units="kB/s")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend(offset=(-10, 10))
        self.plot.setYRange(0, 100)
        self._rate_curve = self.plot.plot(
            [], [], pen=pg.mkPen((37, 99, 235), width=2),
            symbol="o", symbolSize=5, symbolBrush=(37, 99, 235), name="kB/s",
        )
        self._queue_curve = self.plot.plot(
            [], [], pen=pg.mkPen((190, 90, 60), width=1.4, style=Qt.PenStyle.DashLine),
            name="queue (KiB)",
        )
        # Fixed ranges with no autoranging: a live plot that rescales itself every
        # half second is unreadable, and with a single point in it pyqtgraph would
        # otherwise pick an arbitrary range and the trace would sit off-screen.
        self.plot.setXRange(0, 30, padding=0)
        self.plot.setYRange(0, 100, padding=0)
        self.plot.enableAutoRange(False)
        return self.plot

    # -- link description --------------------------------------------------

    def refresh_link_state(self) -> None:
        """Re-read the transport and offer only what is actually possible."""

        self.refresh_ports()
        self._refresh_link_label()

    def on_firmware_report(self, text: str) -> None:
        """Take one ``[BENCH] EVENT=...`` line from the board.

        A refusal is the single most misleading case: the firmware declines to
        start and stops answering, which looks exactly like a dead link. Showing
        the board's own reason turns that into a message.
        """

        fields = dict(token.split("=", 1) for token in text[len("[BENCH]"):].split() if "=" in token)
        event = fields.get("EVENT", "")
        reason = fields.get("REASON", "")
        if event == "failed":
            self._running = False
            self._timer.stop()
            self._install_data_sink(False)
            self._receiver.stop()
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            if reason in ("no-bluetooth-host", "heap-too-low") and self._relink_attempts < 2:
                # Do not tear the port down on the first refusal: the board can report
                # "no host" for a moment while its own connect bookkeeping settles, and
                # dropping a working link is a much worse outcome than waiting. Retry
                # first, and only release the port if the board still says no.
                self._relink_attempts += 1
                self.summary_label.setText(
                    f"The board was not ready ({reason.replace('-', ' ')}). Retrying start "
                    f"({self._relink_attempts}/2) - the link is left alone for now."
                )
                QTimer.singleShot(900, self.start_benchmark)
                return
            self._relink_attempts = 0
            if reason == "no-bluetooth-host" and self._data_link() == "bt":
                # The board no longer sees a client, even though this window still
                # holds the COM handle. That stale handle is what used to force a
                # manual disconnect-and-reconnect, so release it and reopen once.
                self._close_link()
                self.summary_label.setText(
                    "The board no longer sees a Bluetooth client, so the handle this window was "
                    "holding had gone stale. It has been released and is being reopened - press "
                    "Start again in a moment."
                )
                QTimer.singleShot(600, self.toggle_link)
                return
            if reason == "no-bluetooth-host" and self._data_link() == "main":
                self.summary_label.setText(
                    "The board no longer sees a Bluetooth client, so the main window's link has "
                    "gone stale. Disconnect and reconnect it in the main window, then press Start."
                )
                return
            self.summary_label.setText(
                f"The board refused to start the benchmark: {reason.replace('-', ' ')}.\n"
                "Nothing was measured. Check that the Bluetooth link is up, then press Start again."
            )
        elif event in ("diag", "started-diag"):
            # The board's own view of the generator. Kept on screen because it is the
            # only way to tell "the firmware never queued anything" from "the PC
            # never received anything".
            self.diag_label.setText(text)
            fields = dict(token.split("=", 1) for token in text[len("[BENCH]"):].split() if "=" in token)
            colour = "#475569"
            active = fields.get("ACTIVE") == "1"
            fast = fields.get("FAST") == "1"
            stack = int(fields.get("STACK", "0") or 0)
            api_errors = int(fields.get("ERR", "0") or 0)
            if active and not fast:
                colour = "#b45309"
                # Say which input failed, so this is diagnosable at a glance rather
                # than needing another reflash to find out.
                handle = fields.get("HANDLE", "0")
                spp = fields.get("SPP", "?")
                thresh = fields.get("THRESH", "?")
                why = (
                    "the board has no SPP handle yet" if handle == "0"
                    else f"write size {spp} is not above the library threshold {thresh}"
                )
                self.link_state_label.setText(
                    f"Benchmark running on the library write path ({why}): frames will be lost."
                )
                self.link_state_label.setStyleSheet("color: #b45309;")
            elif active and fast and stack == 0 and api_errors > 0:
                # The direct path is engaged but nothing is being accepted. Before the
                # retry logic was fixed this was the permanent state of every run, and
                # it looked identical to a dead link.
                colour = "#b45309"
                self.link_state_label.setText(
                    f"The direct write path is engaged but the stack has accepted nothing "
                    f"({api_errors} API errors, {fields.get('ACK', '0')} missed acknowledgements)."
                )
                self.link_state_label.setStyleSheet("color: #b45309;")
            self.diag_label.setStyleSheet(f"color: {colour}; font-size: 11px;")
        elif event == "stopped":
            explanations = {
                "no-progress": "the radio accepted no data, so this write size is too large for the link",
                "duration-elapsed": "the requested duration ran out",
                "host-request": "you pressed Stop",
                "bluetooth-stopped": "Bluetooth stopped",
                "host-disconnected": "the host disconnected",
            }
            detail = explanations.get(reason, reason.replace("-", " "))
            note = f"the board stopped the test ({detail})"
            if self._running:
                # The board ended the run itself - its progress watchdog, or a write
                # size the link stopped acknowledging. Ignoring this used to leave the
                # window waiting on a link that had already stopped, so Start never
                # came back and the only way out was to reconnect.
                self._end_run(note)
            else:
                self.summary_label.setText(self.summary_label.text() + f"\nBoard stopped the test: {detail}.")

    def _refresh_link_label(self) -> None:
        link = self._data_link()
        window = self._window
        if link == "bt":
            port = str(self.bt_port_combo.currentData() or "")
            if self._bt_link is not None:
                self.link_label.setText(
                    f"Measuring Bluetooth SPP on {port} over this window's own connection - the "
                    "cable stays free for commands. Payload throughput is application bytes the "
                    "PC verified; it is not the radio's PHY bitrate."
                )
                self.link_label.setStyleSheet("color: #166534;")
            elif not self._allows_own_link():
                self.link_label.setText(
                    "The main window is streaming over Bluetooth, so a second connection cannot "
                    "be opened - the board drops the first one. Switch Data link to "
                    "'Main window link' to measure the connection that is already running."
                )
                self.link_label.setStyleSheet("color: #b91c1c;")
            else:
                self.link_label.setText(
                    "Connect the Bluetooth port above to measure it. The cable can stay connected "
                    "for commands, which is what keeps the board controllable during the test."
                )
                self.link_label.setStyleSheet("color: #b45309;")
            return

        if link == "main":
            port = window.port_combo.currentData() if hasattr(window, "port_combo") else ""
            if not window.connected:
                self.link_label.setText(
                    "Connect the board in the main window first, then measure that same link."
                )
                self.link_label.setStyleSheet("color: #b91c1c;")
            elif window._selected_port_is_bluetooth():
                self.link_label.setText(
                    f"Measuring {port} over the main window's own Bluetooth connection. This is "
                    "the real SPP link, and no second connection is opened - the board only "
                    "serves one client."
                )
                self.link_label.setStyleSheet("color: #166534;")
            else:
                self.link_label.setText(
                    f"USB reference: {port} is a cable port, not Bluetooth. This must not be "
                    "reported as Bluetooth performance."
                )
                self.link_label.setStyleSheet("color: #b45309;")
            return

        port = window.port_combo.currentData() if hasattr(window, "port_combo") else ""
        if not window.connected:
            self.link_label.setText("Connect the board, or connect a Bluetooth port above.")
            self.link_label.setStyleSheet("color: #b91c1c;")
        elif window._selected_port_is_bluetooth():
            self.link_label.setText(
                f"Warning: the main link is Bluetooth ({port}), so a USB reference is not "
                "available here. Use 'Main window link' to measure it."
            )
            self.link_label.setStyleSheet("color: #b45309;")
        else:
            self.link_label.setText(
                f"USB reference on {port}: a useful baseline, but it is a different link and must "
                "not be reported as Bluetooth performance."
            )
            self.link_label.setStyleSheet("color: #b45309;")

    # -- the benchmark's own Bluetooth link --------------------------------

    def _data_link(self) -> str:
        return str(self.data_link_combo.currentData() or "bt")

    def _bluetooth_ports(self) -> list[tuple[str, str]]:
        """Bluetooth COM ports the main window knows about (label, device)."""

        table = getattr(self._window, "_port_is_bluetooth", {}) or {}
        wanted: list[tuple[str, str]] = []
        last = str(getattr(self._window.settings, "last_port", "") or "")
        combo = getattr(self._window, "port_combo", None)
        if combo is not None:
            for index in range(combo.count()):
                device = str(combo.itemData(index) or "")
                if device and table.get(device):
                    wanted.append((combo.itemText(index), device))
        if last and last not in [device for _label, device in wanted]:
            wanted.append((f"{last} (last used)", last))
        return wanted

    def _allows_own_link(self) -> bool:
        """Whether a second Bluetooth connection is possible right now.

        The board serves **one** SPP client. A second connection makes the firmware
        treat the existing one as stale: it disconnects the tracked handle and starts
        link recovery, which drops the live stream and leaves the port unusable until
        a reconnect. So the benchmark may only open its own link when the main window
        is not already using Bluetooth.
        """

        window = self._window
        if not window.connected:
            return True
        return not window._selected_port_is_bluetooth()

    def _default_data_link(self) -> str:
        return "main" if not self._allows_own_link() else "bt"

    def refresh_ports(self) -> None:
        previous = str(self.bt_port_combo.currentData() or "")
        held = ""
        self.bt_port_combo.blockSignals(True)
        self.bt_port_combo.clear()
        for label, device in self._bluetooth_ports():
            if self._window.connected and device == str(self._window.port_combo.currentData() or ""):
                held = device
                continue  # opening it again is the one thing that must not happen
            self.bt_port_combo.addItem(label, device)
        if self.bt_port_combo.count() == 0:
            self.bt_port_combo.addItem("(no free Bluetooth port)", "")
        position = self.bt_port_combo.findData(previous)
        self.bt_port_combo.setCurrentIndex(max(position, 0))
        self.bt_port_combo.blockSignals(False)

        # Pick the data link that is actually possible, so the default can never be
        # the option that breaks the connection.
        wanted = self._default_data_link()
        combo = self.data_link_combo
        combo.blockSignals(True)
        position = combo.findData(wanted)
        if position >= 0:
            combo.setCurrentIndex(position)
        combo.blockSignals(False)
        self._on_data_link_changed()
        if held:
            self.link_state_label.setText(
                f"{held} is held by the main window. The board accepts one Bluetooth "
                "client, so the test reads its data from that link instead."
            )
            self.link_state_label.setStyleSheet("color: #b45309;")

    def _on_data_link_changed(self, *_args) -> None:
        own = self._data_link() == "bt"
        self.bt_port_combo.setEnabled(own and self._bt_link is None)
        self.connect_button.setEnabled(own or self._bt_link is not None)
        if not own and self._bt_link is not None:
            # Leaving "own port" must release it: a dangling handle is exactly the
            # "port is always busy" symptom.
            self._close_link()
        self._refresh_link_label()

    def toggle_link(self) -> None:
        if self._bt_link is not None:
            self._close_link()
            return
        if not self._allows_own_link():
            QMessageBox.information(
                self,
                "Benchmark",
                "The main window is already streaming over Bluetooth, and the board accepts "
                "only one Bluetooth client at a time.\n\n"
                "Opening a second connection makes the firmware treat the first as stale and "
                "drop it, which is what leaves the port unusable afterwards.\n\n"
                "Set Data link to 'Main window link' to measure over the connection that is "
                "already open.",
            )
            return
        port = str(self.bt_port_combo.currentData() or "")
        if not port:
            QMessageBox.information(
                self, "Benchmark", "No Bluetooth COM port to connect to. Pair SZD_EEG first."
            )
            return
        if port == str(getattr(self._window, "port_combo", None) and self._window.port_combo.currentData() or ""):
            # The main window already owns that handle; a second open would fail
            # anyway, and the message is clearer than the driver error.
            QMessageBox.information(
                self,
                "Benchmark",
                f"{port} is already open in the main window. Either disconnect it there, or set "
                "the main window to the USB port so the benchmark can hold Bluetooth.",
            )
            return
        link = PowerSerialLink(port, parent=self)
        link.opened.connect(self._on_link_opened)
        link.failed.connect(self._on_link_failed)
        link.closed.connect(self._on_link_closed)
        link.lineReceived.connect(self._on_link_line)
        self._bt_link = link
        self.connect_button.setText("Disconnect")
        self.bt_port_combo.setEnabled(False)
        self.link_state_label.setText(f"Opening {port}...")
        self.link_state_label.setStyleSheet("color: #b45309;")
        link.start()

    def _close_link(self) -> None:
        """Release the port completely - the handle, the thread and the object.

        A COM port that is still held after the dialog closes is the difference
        between "the test is finished" and "Bluetooth is dead until a reboot", so
        this waits for the reader thread to exit rather than trusting it to.
        """

        link = self._bt_link
        self._bt_link = None
        if link is None:
            return
        try:
            link.set_byte_sink(None)
        except Exception:  # noqa: BLE001
            pass
        try:
            link.stop()
            # stop() asks and waits; this makes sure the OS handle is really closed
            # even if the thread was mid-read when it was interrupted.
            link.wait(2000)
        except Exception:  # noqa: BLE001
            pass
        try:
            link.deleteLater()
        except Exception:  # noqa: BLE001
            pass
        self.connect_button.setText("Connect")
        self.bt_port_combo.setEnabled(self._data_link() == "bt")
        self.link_state_label.setText("Not connected")
        self.link_state_label.setStyleSheet("color: #b91c1c;")
        self._refresh_link_label()

    def _on_link_opened(self, port: str) -> None:
        self.link_state_label.setText(f"Connected to {port}")
        self.link_state_label.setStyleSheet("color: #166534;")
        # A test in progress wants its bytes on this link immediately.
        if self._running:
            self._install_data_sink(True)
        self._refresh_link_label()

    def _on_link_failed(self, message: str) -> None:
        self.link_state_label.setText(message)
        self.link_state_label.setStyleSheet("color: #b91c1c;")
        # A failed open must not leave the button reading "Disconnect" over a handle
        # that does not exist, which is how the port looked permanently busy.
        self._close_link()

    def _on_link_closed(self) -> None:
        if self._running:
            self.stop_benchmark("Bluetooth link closed")
        self._close_link()

    def _on_link_line(self, text: str) -> None:
        if text.startswith("[BENCH]"):
            self.on_firmware_report(text)

    def _install_data_sink(self, active: bool) -> None:
        """Attach the receiver to exactly one link - never two, or it counts bytes twice."""

        link = self._data_link()
        use_main = active and link == "main"
        use_own = active and link == "bt"
        # The main window's reader is only tapped when it is the chosen source: its
        # text parser must keep working otherwise.
        self._window._set_benchmark_receiver(self._receiver if (use_main or link == "usb") else None)
        reader = self._bt_link
        if reader is not None:
            try:
                reader.set_byte_sink(self._receiver.feed if use_own else None)
            except Exception:  # noqa: BLE001
                pass

    def shutdown(self) -> None:
        """Release the link and any attached sink; safe to call twice."""

        try:
            self._install_data_sink(False)
        except Exception:  # noqa: BLE001
            pass
        self._close_link()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        if self._running:
            self.stop_benchmark("window closed")
        self.shutdown()
        super().closeEvent(event)

    def _on_mode_changed(self, *_args) -> None:
        mode = str(self.mode_combo.currentData())
        self.rate_combo.setEnabled(mode == self.MODE_FIXED)
        self.duration_spin.setEnabled(mode != self.MODE_SWEEP)
        self.sweep_settle_spin.setEnabled(mode == self.MODE_SWEEP)
        self.sweep_measure_spin.setEnabled(mode == self.MODE_SWEEP)

    # -- firmware commands -------------------------------------------------

    def _send(self, payload: bytes) -> bool:
        """Send a command, preferring the cable.

        Control must stay responsive while the radio is saturated, and the cable is
        never the link under test - so it is tried first whenever the main window
        holds it. Failing that, the command goes over whichever Bluetooth link is
        actually open, which is what makes the window usable on Bluetooth alone.
        """

        if self._window.connected:
            try:
                self._window._send(payload)
                return True
            except Exception:  # noqa: BLE001
                pass
        link = self._bt_link
        if link is not None:
            try:
                return link.send(payload)
            except Exception:  # noqa: BLE001
                return False
        return False

    def _effective_payload(self) -> tuple[int, str]:
        """Payload to use, plus a note when the write size forces it down.

        The Arduino library writes in 330-byte pieces and discards the rest of a
        piece when the stack congests, so a frame larger than one piece cannot
        arrive intact. That is exactly why a 990-byte payload on the 330-byte path
        produced thousands of received bytes and not one parseable frame.
        """

        chunk = int(self.payload_combo.currentData() or 1024)
        spp = int(self.spp_chunk_combo.currentData() or 990)
        limit = 330 - 20
        if spp <= 330 and chunk > limit:
            fits = [size for size in PAYLOAD_CHOICES if size <= limit]
            chunk = max(fits) if fits else PAYLOAD_CHOICES[0]
            return chunk, (
                f"The 330 B library write cannot carry a frame above 330 B, so the payload "
                f"was reduced to {chunk} B for this run."
            )
        return chunk, ""

    def _configure_firmware(self, rate_bps: int, duration_s: float) -> None:
        chunk = self._run_payload
        self._send(f"|benchchunk={chunk};".encode("ascii"))
        self._send(f"|benchspp={int(self.spp_chunk_combo.currentData() or 990)};".encode("ascii"))
        self._send(f"|benchrate={max(int(rate_bps), 0)};".encode("ascii"))
        self._send(f"|benchdur={max(int(duration_s * 1000), 1000)};".encode("ascii"))

    # -- run control -------------------------------------------------------

    def start_benchmark(self) -> None:
        link = self._data_link()
        if link == "main" and not self._window.connected:
            QMessageBox.information(
                self, "Benchmark", "Connect the board in the main window first."
            )
            return
        if link == "bt" and self._bt_link is None:
            QMessageBox.information(
                self, "Benchmark", "Connect the Bluetooth port above first, or switch Data link."
            )
            return
        if link == "usb" and not self._window.connected:
            QMessageBox.information(
                self, "Benchmark", "Connect the board over USB first."
            )
            return
        # The firmware advertises the benchmark in its status line. A board that
        # does not is running an older image that would accept the commands and
        # transmit nothing, which looks exactly like a broken link.
        reported = str(self._window.power_state.get("BENCH", "") or "")
        if reported and reported.isdigit() and int(reported) < 1:
            QMessageBox.warning(
                self,
                "Benchmark",
                "The board does not report benchmark support (BENCH). Reflash it: an older "
                "firmware image accepts these commands and sends nothing.",
            )
            return
        if self._window.recording is not None:
            QMessageBox.information(
                self, "Benchmark", "Stop the recording first: the test stops the sample stream."
            )
            return
        if self._window.streaming:
            # Stop the stream from here rather than letting the firmware do it behind
            # the GUI's back: the button state, the packet counters and the port all
            # have to agree that acquisition has stopped.
            self._window.stream_button.setChecked(False)

        mode = str(self.mode_combo.currentData())
        if mode == self.MODE_SWEEP:
            self._sweep_index = 0
            self._sweep_results = []
            self._start_sweep_step()
            return
        rate = 0 if mode == self.MODE_MAX else int(self.rate_combo.currentData() or 0)
        self._begin_run(rate, float(self.duration_spin.value()), mode)

    def _begin_run(self, rate_bps: int, duration_s: float, mode: str) -> None:
        self._run_payload, note = self._effective_payload()
        # The sink must be installed before the first frame can arrive.
        self._window._set_benchmark_receiver(None)
        self._receiver.payload_bytes = self._run_payload
        self._receiver.start()
        self._install_data_sink(True)
        self._configure_firmware(rate_bps, duration_s)
        self._send(b"|bench=1;")
        self._running = True
        self._run_mode = mode
        self._run_rate = rate_bps
        self._run_until = time.monotonic() + duration_s
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.save_json_button.setEnabled(False)
        self.save_csv_button.setEnabled(False)
        self.apply_budget_button.setEnabled(False)
        # Clear the traces, not the plot: plot.clear() removes the curve items too,
        # and every later setData() then writes to an orphaned item - which is why
        # the graph stayed blank through an entire run.
        self._rate_curve.setData([], [])
        self._queue_curve.setData([], [])
        self._plot_ymax = 0.0
        self.plot.setYRange(0, 100, padding=0)
        self.plot.setXRange(0, 30, padding=0)
        self.summary_label.setText(
            f"Running: {'unlimited' if rate_bps <= 0 else f'{rate_bps / 1024:.0f} KiB/s target'}, "
            f"{self._run_payload} B payload, {duration_s:.0f} s."
            + (f"\n{note}" if note else "")
        )
        self._timer.start()

    def _end_run(self, reason: str) -> None:
        """Finish a run that is already over, whoever ended it."""

        if self._run_mode == self.MODE_SWEEP and self._sweep_phase != "idle":
            # A sweep step ended early - the board's progress watchdog, or a link that
            # will not take that rate. That is a *result*, not a failure of the sweep:
            # record the step and move on instead of abandoning the whole run.
            self._complete_sweep_step(extra_note=reason)
            return
        self._timer.stop()
        self._running = False
        self._sweep_phase = "idle"
        self._install_data_sink(False)
        self._receiver.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        result = self._receiver.result(
            mode=self._run_mode,
            rate_target_bps=getattr(self, "_run_rate", 0),
            payload_bytes=getattr(self, "_run_payload", 0) or int(self.payload_combo.currentData() or 1024),
        )
        self._publish(result, reason)

    def stop_benchmark(self, reason: str) -> None:
        self._send(b"|bench=0;")
        self._end_run(reason)

    def _publish(self, result: BenchmarkResult, reason: str) -> None:
        self._last_result = result
        link = self._data_link()
        if link == "main":
            over_bluetooth = self._window._selected_port_is_bluetooth()
            label = "Bluetooth SPP (main window link)" if over_bluetooth else "USB reference (NOT Bluetooth)"
        elif link == "bt":
            label = "Bluetooth SPP (own port)"
        else:
            label = "USB reference (NOT Bluetooth)"
        ok, why = is_sustainable(result)
        self.summary_label.setText(
            f"{label} - {reason}.\n"
            f"Average {result.average_kib_s:.1f} KiB/s ({result.average_kb_s:.1f} kB/s, "
            f"{result.average_mbit_s:.2f} Mbit/s), peak {result.peak_kib_s:.1f} KiB/s.\n"
            f"Verified {result.verified_bytes} bytes in {result.packets} packets; "
            f"{result.missing_packets} missing, {result.corrupt_packets} corrupt, "
            f"{result.stalls} stalls, queue peak {result.peak_queue_fill}/{result.ring_bytes} bytes.\n"
            f"Verdict: {result.verdict} - {why}."
        )
        self.save_json_button.setEnabled(True)
        self.save_csv_button.setEnabled(True)
        self.apply_budget_button.setEnabled(result.average_bytes_per_second > 0)

    # -- sweep ------------------------------------------------------------

    def _start_sweep_step(self) -> None:
        if self._sweep_index >= len(SWEEP_STEPS):
            self._finish_sweep()
            return
        rate = SWEEP_STEPS[self._sweep_index]
        settle = float(self.sweep_settle_spin.value())
        measure = float(self.sweep_measure_spin.value())
        self._begin_run(rate, settle + measure, self.MODE_SWEEP)
        self._sweep_phase = "settle"
        self._sweep_phase_until = time.monotonic() + settle
        self._sweep_measure_s = measure
        self.summary_label.setText(
            f"Sweep step {self._sweep_index + 1}/{len(SWEEP_STEPS)}: {rate / 1024:.0f} KiB/s "
            f"({settle:.0f} s settle, {measure:.0f} s measure)."
        )

    def _complete_sweep_step(self, extra_note: str = "") -> None:
        """Record the current sweep step and either start the next or finish."""

        self._sweep_phase = "idle"
        self._send(b"|bench=0;")
        self._receiver.stop()
        result = self._receiver.result(
            mode=self.MODE_SWEEP,
            rate_target_bps=SWEEP_STEPS[self._sweep_index],
            payload_bytes=self._run_payload or int(self.payload_combo.currentData() or 1024),
        )
        self._sweep_results.append((SWEEP_STEPS[self._sweep_index], result))
        if extra_note:
            result.notes.append(extra_note)
        self._sweep_index += 1
        self._window._set_benchmark_receiver(None)
        if self._sweep_index >= len(SWEEP_STEPS):
            self._finish_sweep()
        else:
            self._start_sweep_step()

    def _finish_sweep(self) -> None:
        self._timer.stop()
        self._running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        text = summarize_sweep(self._sweep_results)
        lines = [text, ""]
        for rate, result in self._sweep_results:
            ok, why = is_sustainable(result)
            lines.append(
                f"  {rate / 1024:5.0f} KiB/s -> {result.average_kib_s:6.1f} KiB/s verified, "
                f"{result.missing_packets} missing, {result.corrupt_packets} corrupt, "
                f"{result.stalls} stalls  [{'OK' if ok else 'FAIL'}: {why}]"
            )
        self.summary_label.setText("\n".join(lines))
        best = max(
            (rate for rate, result in self._sweep_results if is_sustainable(result)[0]), default=0
        )
        if best:
            self._last_result = next(r for rate, r in self._sweep_results if rate == best)
            self.apply_budget_button.setEnabled(True)
        self.save_json_button.setEnabled(bool(self._sweep_results))
        self.save_csv_button.setEnabled(bool(self._sweep_results))

    # -- periodic update ---------------------------------------------------

    def _tick(self) -> None:
        self._receiver.poll()
        if self._sweep_phase == "settle" and time.monotonic() >= self._sweep_phase_until:
            # Restart the counters so the step is judged on the measurement window
            # only, not on the ramp into it.
            self._receiver.start()
            self._receiver.payload_bytes = self._run_payload or int(self.payload_combo.currentData() or 1024)
            self._sweep_phase = "measure"
            self._sweep_phase_until = time.monotonic() + self._sweep_measure_s
        elif self._sweep_phase == "measure" and time.monotonic() >= self._sweep_phase_until:
            self._complete_sweep_step()

        if self._running:
            if self._run_mode == self.MODE_SWEEP:
                # The sweep drives its own step boundaries; a duration stop here would
                # cut a step short and then fight the phase timer for the next one.
                pass
            elif time.monotonic() >= self._run_until:
                self.stop_benchmark("duration elapsed")
            elif self._receiver.raw_bytes == 0 and self._receiver.elapsed_s > 4.0:
                # Nothing at all has arrived - not even a firmware stats packet. Saying
                # so beats showing a row of zeros that looks like a measurement.
                self.summary_label.setText(
                    "Running, but nothing has arrived on this link after "
                    f"{self._receiver.elapsed_s:.0f} s - not even the firmware's once-per-second "
                    "stats packet. The board is not sending, so the problem is the connection "
                    "rather than the throughput. The board diagnostics line above shows whether "
                    "the generator is running and whether the ring is accepting data."
                )
            elif self._receiver.raw_bytes > 0 and self._receiver.packets == 0 and self._receiver.elapsed_s > 4.0:
                self.summary_label.setText(
                    f"{self._receiver.raw_bytes} bytes have arrived but none of them parsed as a "
                    "benchmark frame. The link is delivering data, so the fault is in the framing "
                    "- most likely a firmware image older than this window."
                )
        self._update_stats()
        self._update_plot()

    def _update_stats(self) -> None:
        receiver = self._receiver
        rate = receiver.current_bytes_per_second
        firmware = receiver.firmware
        values = {
            "current": f"{rate / 1024:.1f} KiB/s  ({rate / 1000:.1f} kB/s, {rate * 8 / 1e6:.2f} Mbit/s)",
            "average": f"{receiver.average_bytes_per_second / 1024:.1f} KiB/s",
            "peak": f"{receiver.peak_bytes_per_second / 1024:.1f} KiB/s",
            "verified": f"{receiver.verified_bytes} B",
            "received": f"{receiver.received_bytes} B",
            "raw": f"{receiver.raw_bytes} B",
            "packets": f"{receiver.packets}",
            "corrupt": f"{receiver.corrupt_packets}",
            "missing": f"{receiver.missing_packets}",
            "sequence": f"{receiver.sequence_errors}",
            "queue": f"{firmware.queue_fill} / {firmware.ring_bytes or '?'} B",
            "queue_peak": f"{firmware.peak_fill} B",
            "stalls": f"{firmware.stalls}",
            "congestion": (
                f"{firmware.congestion_events} (ack timeouts {firmware.ack_timeouts})"
            ),
            "spopath": (
                f"{'direct ' + str(firmware.spp_write_chunk) + ' B' if firmware.fast_path else 'library ' + str(firmware.spp_write_chunk) + ' B'}"
                + (f" - {firmware.fast_path_fallbacks} fallback(s)" if firmware.fast_path_fallbacks else "")
            ),
            "handle": f"{firmware.spp_handle}",
            "elapsed": f"{receiver.elapsed_s:.1f} s",
            "verdict": receiver.verdict() if receiver.packets else "-",
        }
        for key, text in values.items():
            label = self._stat_labels.get(key)
            if label is not None:
                label.setText(text)

    def _update_plot(self) -> None:
        samples = self._receiver.samples
        if not samples:
            return
        # Throttled and incremental: rebuilding the whole figure ten times a second
        # was a measurable part of why the window felt dead during a test.
        now = time.monotonic()
        if now - self._last_plot_s < 0.4:
            return
        self._last_plot_s = now
        times = np.fromiter((s["t"] for s in samples), dtype=float, count=len(samples))
        rates = np.fromiter((s["kib_per_second"] for s in samples), dtype=float, count=len(samples))
        fills = np.fromiter((s["queue_fill"] / 1024.0 for s in samples), dtype=float, count=len(samples))
        # setData on persistent curves: no item churn, no legend rebuild.
        self._rate_curve.setData(times, rates)
        self._queue_curve.setData(times, fills)
        # Grow the range to fit, and only ever grow: a shrinking axis during a run
        # makes a dip look like a collapse.
        span = max(float(times[-1]), 5.0)
        self.plot.setXRange(0, span * 1.05, padding=0)
        top = max(float(rates.max()) if rates.size else 0.0, float(fills.max()) if fills.size else 0.0)
        if top > self._plot_ymax:
            self._plot_ymax = top * 1.25
            self.plot.setYRange(0, self._plot_ymax, padding=0)

    # -- results -----------------------------------------------------------

    def _save_result(self, kind: str) -> None:
        if kind == "json" and self._sweep_results:
            results = [
                {"rate_bps": rate, "sustainable": is_sustainable(result)[0], **result.as_dict()}
                for rate, result in self._sweep_results
            ]
            file_name, _filter = QFileDialog.getSaveFileName(
                self, "Save sweep results", "bluetooth_sweep.json", "JSON files (*.json)"
            )
            if not file_name:
                return
            import json as _json

            with open(file_name, "w", encoding="utf-8") as handle:
                _json.dump(results, handle, indent=2)
            self.summary_label.setText(self.summary_label.text() + f"\nSaved to {file_name}.")
            return
        result = self._last_result
        if result is None:
            return
        if kind == "json":
            file_name, _filter = QFileDialog.getSaveFileName(
                self, "Save benchmark result", "bluetooth_benchmark.json", "JSON files (*.json)"
            )
            if file_name:
                result.save_json(file_name)
        else:
            file_name, _filter = QFileDialog.getSaveFileName(
                self, "Save benchmark samples", "bluetooth_benchmark.csv", "CSV files (*.csv)"
            )
            if file_name:
                result.save_csv(file_name)

    def _apply_measured_budget(self) -> None:
        rate = self._window._benchmark_sustainable_rate(self._last_result, self._sweep_results)
        if rate <= 0:
            QMessageBox.information(
                self, "Benchmark", "There is no verified sustainable rate to adopt yet."
            )
            return
        margin, ok = QInputDialog.getDouble(
            self,
            "Link budget",
            f"Adopt {rate / 1024:.1f} KiB/s as the Bluetooth capacity.\n"
            "Safety margin applied on top (%):",
            20.0,
            0.0,
            90.0,
            1,
        )
        if not ok:
            return
        self._window.set_bluetooth_budget(rate, margin)
        effective = self._window.bluetooth_budget_bytes_per_second()
        self.summary_label.setText(
            self.summary_label.text()
            + f"\nLink budget now {effective / 1024:.1f} KiB/s "
            f"({rate / 1024:.1f} KiB/s measured, {margin:.0f} % margin)."
        )
        self._refresh_link_label()


class PowerSerialLink(QThread):
    """A second serial port for the power panel.

    The board only stays in Bluetooth mode while a host is actually using the SPP
    link, so measuring "Bluetooth running" current needs a program holding that port
    open. The main window already owns the USB port, so the panel gets its own,
    completely separate connection - a different COM port, so there is no conflict
    with the cable.
    """

    lineReceived = pyqtSignal(str)
    opened = pyqtSignal(str)
    failed = pyqtSignal(str)
    closed = pyqtSignal()

    def __init__(self, port: str, baud: int = 115200, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._handle = None
        self._sink = None

    def set_byte_sink(self, sink) -> None:
        """Take raw bytes before they are split into lines.

        Called from this thread, so a high-rate consumer never has to cross a Qt
        queued connection - which would cap any throughput measurement at the event
        loop's rate. The callable returns the bytes that were not consumed; those
        are still parsed into lines, so replies keep working during a test.
        """

        self._sink = sink

    def stop(self) -> None:
        self._stop.set()
        handle = self._handle
        self._handle = None
        if handle is not None:
            # Cancel any pending read first: a Windows Bluetooth COM port can sit in
            # a read for a while, and close() will not take effect until it returns.
            try:
                handle.cancel_read()
            except Exception:  # noqa: BLE001 - not all backends support it
                pass
            try:
                handle.close()
            except Exception:  # noqa: BLE001 - closing a dead port must not raise
                pass
        self.wait(2500)

    def send(self, payload: bytes) -> bool:
        handle = self._handle
        if handle is None:
            return False
        try:
            handle.write(payload)
            return True
        except Exception as exc:  # noqa: BLE001
            self.failed.emit(f"Bluetooth write failed: {exc}")
            return False

    def run(self) -> None:  # noqa: D401 - QThread entry point
        if serial is None:
            self.failed.emit("pyserial is not installed")
            return
        try:
            # A Bluetooth COM port can take seconds to come up while the driver
            # talks to the radio, which is why this is on its own thread.
            open_kwargs = {"timeout": 0.05}
            try:
                handle = serial.Serial(self.port, self.baud, write_timeout=0.5, **open_kwargs)
            except (ValueError, TypeError):
                handle = serial.Serial(self.port, self.baud, **open_kwargs)
        except Exception as exc:  # noqa: BLE001
            self.failed.emit(f"Could not open {self.port}: {exc}")
            return
        self._handle = handle
        self.opened.emit(self.port)
        buffer = b""
        try:
            while not self._stop.is_set():
                try:
                    chunk = handle.read(max(int(getattr(handle, "in_waiting", 0) or 0), 1))
                except Exception as exc:  # noqa: BLE001
                    if not self._stop.is_set():
                        self.failed.emit(f"{self.port} read failed: {exc}")
                    break
                if not chunk:
                    continue
                sink = self._sink
                if sink is not None:
                    try:
                        chunk = sink(chunk)
                    except Exception:  # noqa: BLE001 - a bad sink must not kill the link
                        chunk = b""
                if not chunk:
                    continue
                buffer += chunk
                # Bound the line buffer. Benchmark bytes are binary: they contain no
                # newline, so without this the buffer grows without limit whenever
                # data arrives while no sink is attached - which is exactly what
                # happens between Stop and the firmware noticing. That was an
                # out-of-memory crash waiting to happen.
                if len(buffer) > 65536 and b"\n" not in buffer:
                    buffer = b""
                    continue
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    text = raw.decode("ascii", "replace").strip()
                    if text:
                        self.lineReceived.emit(text)
        finally:
            try:
                handle.close()
            except Exception:  # noqa: BLE001
                pass
            self._handle = None
            self.closed.emit()


class PowerControlDialog(QDialog):
    """Switch board subsystems on and off to measure their current draw.

    The panel only reports and switches; the bench supply does the measuring. It
    therefore does two things very deliberately:

    * **It never polls.** The state is read once when the window opens and again
      only when *Refresh* is pressed. A periodic query would put a burst of card
      and SPI traffic into every measurement - which is exactly the artefact this
      window exists to avoid.
    * **It states which pins are shared.** ``PWDN`` and ``RESET`` are wired to both
      ADS1299s on this board, so those two controls say so instead of pretending to
      be per-converter. Per-chip operating mode goes through the START/STOP/STANDBY
      SPI opcodes, which really are per device.
    """

    #: SPS choices as (label, data-rate bits). Matches the ADS1299 DR field.
    SPS_CHOICES = (
        ("16000 SPS", 0),
        ("8000 SPS", 1),
        ("4000 SPS", 2),
        ("2000 SPS", 3),
        ("1000 SPS", 4),
        ("500 SPS", 5),
        ("250 SPS", 6),
    )
    GAIN_CHOICES = (("x1", 0), ("x2", 1), ("x4", 2), ("x6", 3), ("x8", 4), ("x12", 5), ("x24", 6))
    MODE_CHOICES = (
        ("Run (converting, streaming)", "run"),
        ("Idle (clock on, no conversion)", "idle"),
        ("Standby (lowest per-chip current)", "standby"),
    )
    #: The BR/EDR steps the controller actually has.
    BT_POWER_CHOICES = tuple((f"{dbm:+d} dBm", dbm) for dbm in (-12, -9, -6, -3, 0, 3, 6, 9))

    def __init__(
        self,
        send,
        connected: bool = False,
        parent: Optional[QWidget] = None,
        bluetooth_ports: Optional[Sequence[tuple[str, str]]] = None,
    ) -> None:
        super().__init__(parent)
        self._send_command = send
        self._bt_link: Optional[PowerSerialLink] = None
        self._bt_ports = list(bluetooth_ports or [])
        self.setWindowTitle("Power control")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(620, 720)
        self._state: dict[str, str] = {}

        outer = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setSpacing(8)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)

        layout.addWidget(self._build_converter_group())
        layout.addWidget(self._build_link_group())
        layout.addWidget(self._build_peripheral_group())
        layout.addWidget(self._build_bluetooth_group())
        layout.addWidget(self._build_sleep_group())
        layout.addWidget(self._build_status_group())
        layout.addStretch(1)

        button_row = QHBoxLayout()
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setToolTip("Read the board's power state (nothing is polled automatically)")
        self.refresh_button.clicked.connect(self.refresh)
        button_row.addWidget(self.refresh_button)
        button_row.addStretch(1)
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        button_row.addWidget(close_button)
        outer.addLayout(button_row)

        # The board powers up with the converters, accelerometer, card and radio
        # enabled, so those boxes start checked and a Refresh corrects them. PWDN and
        # RESET are inverted (checked = held low = disabled), so they start clear.
        for widget in (
            self.lis3dh_check,
            self.sd_check,
            self.bt_check,
            self.ads_clkout_check,
        ):
            widget.setChecked(True)
        self._set_stream_button(False)
        self.state_label.setText("Press Refresh to read the board's current state.")
        self.set_connected(connected)

    # -- construction ------------------------------------------------------

    def _build_link_group(self) -> QWidget:
        group = QGroupBox("Command link")
        layout = QVBoxLayout(group)

        row = QHBoxLayout()
        row.addWidget(QLabel("Send over"))
        self.transport_combo = QComboBox()
        self.transport_combo.addItem("USB (main window link)", "usb")
        self.transport_combo.addItem("Bluetooth SPP", "bt")
        self.transport_combo.setToolTip(
            "USB uses the cable port the main window already holds. Bluetooth opens its own "
            "connection to the SPP COM port, which is what keeps the board in Bluetooth mode - "
            "it leaves Bluetooth when nothing is using the link."
        )
        row.addWidget(self.transport_combo, 1)
        layout.addLayout(row)

        port_row = QHBoxLayout()
        self.bt_port_combo = QComboBox()
        self.bt_port_combo.setMinimumWidth(150)
        for label, device in self._bt_ports:
            self.bt_port_combo.addItem(label, device)
        port_row.addWidget(self.bt_port_combo, 1)
        self.bt_connect_button = QPushButton("Connect")
        self.bt_connect_button.setCheckable(True)
        self.bt_connect_button.toggled.connect(self._on_bt_connect_toggled)
        port_row.addWidget(self.bt_connect_button)
        layout.addLayout(port_row)

        self.bt_link_label = QLabel(
            "No Bluetooth port found. Pair SZD_EEG first; the port appears as "
            "'Standard Serial over Bluetooth link'."
            if not self._bt_ports
            else "Not connected - press Connect to hold the SPP link open."
        )
        self.bt_link_label.setWordWrap(True)
        self.bt_link_label.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.bt_link_label)
        return group

    def _on_bt_connect_toggled(self, checked: bool) -> None:
        if not checked:
            link = self._bt_link
            self._bt_link = None
            if link is not None:
                link.stop()
            self.bt_link_label.setText("Disconnected.")
            self.bt_link_label.setStyleSheet("color: #475569; font-size: 11px;")
            # Without the extra transport, the panel needs the main link again.
            self.set_connected(bool(getattr(self.parent(), "connected", False)))
            return
        device = str(self.bt_port_combo.currentData() or "")
        if not device:
            self.bt_connect_button.blockSignals(True)
            self.bt_connect_button.setChecked(False)
            self.bt_connect_button.blockSignals(False)
            self.bt_link_label.setText("No Bluetooth port selected.")
            return
        link = PowerSerialLink(device, parent=self)
        link.lineReceived.connect(self._on_bt_line)
        link.opened.connect(
            lambda port: (
                self.bt_link_label.setText(f"Connected to {port} - the board stays in Bluetooth mode."),
                self.bt_link_label.setStyleSheet("color: #15803d; font-size: 11px;"),
                # Reading the state over the new link proves it works end to end.
                self.refresh(),
            )
        )
        link.failed.connect(self._on_bt_failed)
        link.closed.connect(
            lambda: self.bt_link_label.setText("Bluetooth link closed.")
        )
        self._bt_link = link
        self.bt_link_label.setText(f"Opening {device}...")
        self.bt_link_label.setStyleSheet("color: #475569; font-size: 11px;")
        link.start()
        # Enable the controls immediately; the send path reports if the open fails.
        self.set_connected(True)

    def telemetry(self) -> dict[str, str]:
        """The last ``[PWR]`` state, for anything that wants to log it."""

        return dict(self._state)

    def shutdown(self) -> None:
        link = self._bt_link
        self._bt_link = None
        if link is not None:
            link.stop()

    def _on_bt_failed(self, message: str) -> None:
        self.bt_link_label.setText(message)
        self.bt_link_label.setStyleSheet("color: #b91c1c; font-size: 11px;")
        self.bt_connect_button.blockSignals(True)
        self.bt_connect_button.setChecked(False)
        self.bt_connect_button.blockSignals(False)
        link = self._bt_link
        self._bt_link = None
        if link is not None:
            link.stop()

    def _on_bt_line(self, text: str) -> None:
        if text.startswith("[PWR]"):
            self.apply_state_dict(
                dict(token.split("=", 1) for token in text[len("[PWR]"):].split() if "=" in token)
            )
        elif text.startswith("Success:") or text.startswith("#OK"):
            pass
        else:
            # Anything else is worth showing: it is the board talking back on the
            # link the operator is trying to prove.
            self.link_trace.append(text)
            del self.link_trace[:-8]
            self.link_trace_label.setText("\n".join(self.link_trace[-4:]))

    def _dispatch(self, payload: bytes) -> bool:
        if str(self.transport_combo.currentData()) == "bt":
            link = self._bt_link
            if link is None:
                self.state_label.setText("Connect the Bluetooth link first, or send over USB.")
                return False
            return link.send(payload)
        return bool(self._send_command(payload))

    def _build_converter_group(self) -> QWidget:
        group = QGroupBox("ADS1299 converters")
        layout = QVBoxLayout(group)

        for index in range(2):
            row = QGridLayout()
            row.setHorizontalSpacing(6)
            row.addWidget(QLabel(f"ADS{index + 1}"), 0, 0)
            mode = QComboBox()
            for label, value in self.MODE_CHOICES:
                mode.addItem(label, value)
            row.addWidget(mode, 0, 1)
            sps = QComboBox()
            for label, value in self.SPS_CHOICES:
                sps.addItem(label, value)
            row.addWidget(sps, 0, 2)
            gain = QComboBox()
            for label, value in self.GAIN_CHOICES:
                gain.addItem(label, value)
            row.addWidget(gain, 0, 3)
            apply_button = QPushButton("Apply")
            apply_button.clicked.connect(
                lambda _=False, i=index: self._apply_converter(i)
            )
            row.addWidget(apply_button, 0, 4)
            row.setColumnStretch(1, 1)
            layout.addLayout(row)
            setattr(self, f"ads{index + 1}_mode", mode)
            setattr(self, f"ads{index + 1}_sps", sps)
            setattr(self, f"ads{index + 1}_gain", gain)

        note = QLabel(
            "PWDN and RESET are wired to both converters, so those two are board-wide; "
            "the mode above is per converter (SPI opcodes)."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(note)

        restore_row = QHBoxLayout()
        self.restore_button = QPushButton("Restore converters to normal")
        self.restore_button.setToolTip(
            "Re-detect both converters and re-apply the full configuration this app holds "
            "(reference, clock output, every channel, data rate). Use this after a power or "
            "reset test: a converter that comes back from PWDN or RESET answers on SPI with its "
            "register defaults, which means every channel is in power-down and nothing converts."
        )
        self.restore_button.clicked.connect(self.restore_converters)
        restore_row.addWidget(self.restore_button)
        restore_row.addStretch(1)
        layout.addLayout(restore_row)

        stream_row = QHBoxLayout()
        self.stream_button = QPushButton("Start stream")
        self.stream_button.setCheckable(True)
        self.stream_button.setToolTip(
            "Start and stop acquisition from here. 'Converters running with an idle link' and "
            "'link running with idle converters' draw very different current, so being able to "
            "switch the stream without leaving the panel is the difference between two runs."
        )
        self.stream_button.clicked.connect(self._apply_stream)
        stream_row.addWidget(self.stream_button)
        stream_row.addStretch(1)
        layout.addLayout(stream_row)

        shared = QGridLayout()
        shared.setHorizontalSpacing(6)
        self.ads_pwdn_check = QCheckBox("Hold PWDN low - both converters off")
        self.ads_pwdn_check.setToolTip(
            "Check this to drive PWDN low and shut both ADS1299s down. They are powered "
            "normally when it is clear. Releasing it re-detects and re-configures them."
        )
        self.ads_pwdn_check.clicked.connect(self._apply_ads_pwdn)
        shared.addWidget(self.ads_pwdn_check, 0, 0, 1, 2)
        self.ads_reset_check = QCheckBox("Hold RESET low - both converters in reset")
        self.ads_reset_check.setToolTip(
            "Check this to hold RESET low: no SPI access at all until it is released, which "
            "re-detects and re-configures both converters. They run normally when it is clear."
        )
        self.ads_reset_check.clicked.connect(self._apply_ads_reset)
        shared.addWidget(self.ads_reset_check, 1, 0, 1, 2)
        self.ads_clkout_check = QCheckBox("Internal oscillator CLK output enabled (ADS1)")
        self.ads_clkout_check.setToolTip(
            "CONFIG1 CLK_EN. This is the clock *output driver* on the CLK pin, which is what "
            "feeds the second converter - the oscillator itself always runs and cannot be "
            "switched off. Turning it off saves the output driver only, and stops ADS2."
        )
        self.ads_clkout_check.clicked.connect(self._apply_ads_clkout)
        shared.addWidget(self.ads_clkout_check, 2, 0, 1, 2)
        layout.addLayout(shared)
        return group

    def _build_peripheral_group(self) -> QWidget:
        group = QGroupBox("Peripherals")
        layout = QVBoxLayout(group)
        self.lis3dh_check = QCheckBox("Accelerometer (LIS3DH) active")
        self.lis3dh_check.setToolTip(
            "Unchecking writes CTRL_REG1 = 0, which stops the accelerometer's oscillator and "
            "takes it from ~11 uA at 100 Hz to ~0.5 uA."
        )
        self.lis3dh_check.clicked.connect(
            lambda: self._send_key("lis3dh", 1 if self.lis3dh_check.isChecked() else 0)
        )
        layout.addWidget(self.lis3dh_check)

        self.sd_check = QCheckBox("SD card mounted")
        self.sd_check.setToolTip(
            "Unchecking releases the card (SD.end(), CS high, bus parked) so the card drops to "
            "its own standby current. It is re-initialised when checked again."
        )
        self.sd_check.clicked.connect(
            lambda: self._send_key("sd", 1 if self.sd_check.isChecked() else 0)
        )
        layout.addWidget(self.sd_check)

        self.test_check = QCheckBox("Synthetic test stream (no converter traffic)")
        self.test_check.setToolTip(
            "Stream a generated signal instead of reading the converters. This isolates the "
            "link and the CPU from the analog front end, so the current draw of the radio and "
            "the ESP32 can be measured on their own."
        )
        self.test_check.clicked.connect(
            lambda: self._send_key("test", 1 if self.test_check.isChecked() else 0)
        )
        layout.addWidget(self.test_check)
        return group

    def _build_bluetooth_group(self) -> QWidget:
        group = QGroupBox("Bluetooth")
        layout = QVBoxLayout(group)
        self.bt_check = QCheckBox("Radio stack running")
        self.bt_check.setToolTip(
            "Unchecking stops the SPP stack. USB streaming is unaffected, so the board stays "
            "controllable over the cable while the radio is off."
        )
        self.bt_check.clicked.connect(
            lambda: self._send_key("bt", 1 if self.bt_check.isChecked() else 0)
        )
        layout.addWidget(self.bt_check)

        row = QHBoxLayout()
        row.addWidget(QLabel("TX power"))
        self.bt_power_combo = QComboBox()
        for label, value in self.BT_POWER_CHOICES:
            self.bt_power_combo.addItem(label, value)
        self.bt_power_combo.setToolTip(
            "BR/EDR transmit power as a runtime request. The controller snaps it to its own "
            "steps, the level it actually holds is read back and reported, and the reply shows "
            "both. Lower power draws less current and also couples less RF into the front end."
        )
        row.addWidget(self.bt_power_combo, 1)
        apply_button = QPushButton("Apply")
        apply_button.setToolTip("Send the level and read back what the controller actually holds.")
        apply_button.clicked.connect(self._apply_bt_power)
        row.addWidget(apply_button)
        layout.addLayout(row)

        readback_row = QHBoxLayout()
        readback_row.addWidget(QLabel("Controller holds"))
        self.bt_readback_label = QLabel("unknown - press Refresh")
        self.bt_readback_label.setStyleSheet("font-weight: 600;")
        self.bt_readback_label.setToolTip(
            "esp_bredr_tx_power_get() read from the controller after the last set. This is the "
            "authority: the request is only what was asked for."
        )
        readback_row.addWidget(self.bt_readback_label, 1)
        layout.addLayout(readback_row)

        tx_note = QLabel(
            "TX power only changes the reading while the radio is actually transmitting. With "
            "no Bluetooth host connected the radio idles and the current will not move; to "
            "measure a difference, hold the SPP link open and let data flow over it."
        )
        tx_note.setWordWrap(True)
        tx_note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(tx_note)

        self.bt_sleep_check = QCheckBox("Controller modem sleep")
        self.bt_sleep_check.setToolTip(
            "esp_bt_sleep_enable(). Experimental: some controller/host combinations reject it, "
            "and the reply says so rather than pretending it worked."
        )
        self.bt_sleep_check.clicked.connect(
            lambda: self._send_key("btsleep", 1 if self.bt_sleep_check.isChecked() else 0)
        )
        layout.addWidget(self.bt_sleep_check)
        return group

    def _build_sleep_group(self) -> QWidget:
        group = QGroupBox("ESP32 sleep")
        layout = QVBoxLayout(group)

        light_row = QHBoxLayout()
        light_row.addWidget(QLabel("Light sleep"))
        self.light_ms_spin = QSpinBox()
        self.light_spin = self.light_ms_spin
        self.light_ms_spin.setRange(50, 600_000)
        self.light_ms_spin.setSingleStep(500)
        self.light_ms_spin.setValue(3000)
        self.light_ms_spin.setSuffix(" ms")
        light_row.addWidget(self.light_ms_spin)
        light_button = QPushButton("Sleep")
        light_button.clicked.connect(lambda: self._sleep("light"))
        light_row.addWidget(light_button)
        light_row.addStretch(1)
        layout.addLayout(light_row)

        deep_row = QHBoxLayout()
        deep_row.addWidget(QLabel("Deep sleep"))
        self.deep_ms_spin = QSpinBox()
        self.deep_ms_spin.setRange(1000, 3_600_000)
        self.deep_ms_spin.setSingleStep(1000)
        self.deep_ms_spin.setValue(10_000)
        self.deep_ms_spin.setSuffix(" ms")
        deep_row.addWidget(self.deep_ms_spin)
        deep_button = QPushButton("Sleep (restarts)")
        deep_button.clicked.connect(lambda: self._sleep("deep"))
        deep_row.addWidget(deep_button)
        deep_row.addStretch(1)
        layout.addLayout(deep_row)

        note = QLabel(
            "Light sleep keeps RAM and the program counter, so the board continues on wake "
            "with no reset - but the radio must be off and the link is silent for the whole "
            "interval. Deep sleep has no way around a restart: the board reboots when the "
            "timer expires. Both stop streaming first."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(note)
        return group

    def _build_status_group(self) -> QWidget:
        group = QGroupBox("Board reports")
        layout = QVBoxLayout(group)
        self.state_label = QLabel("Not read yet - press Refresh.")
        self.state_label.setWordWrap(True)
        self.state_label.setStyleSheet("font-family: Consolas; font-size: 11px; color: #1e293b;")
        self.state_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.state_label)

        self.link_trace: list[str] = []
        self.link_trace_label = QLabel("")
        self.link_trace_label.setWordWrap(True)
        self.link_trace_label.setStyleSheet("font-family: Consolas; font-size: 10px; color: #475569;")
        layout.addWidget(self.link_trace_label)
        return group

    # -- behaviour ---------------------------------------------------------

    def set_connected(self, connected: bool, message: str = "") -> None:
        """Enable the controls that need the main (USB) link.

        The Bluetooth transport is deliberately *not* gated on this: its whole
        purpose is to be usable when the panel is talking over the SPP port instead,
        including before or without the main window's own connection.
        """

        for widget in (
            self.refresh_button,
            self.stream_button,
            self.lis3dh_check,
            self.sd_check,
            self.test_check,
            self.bt_check,
            self.bt_sleep_check,
            self.ads_pwdn_check,
            self.ads_reset_check,
            self.ads_clkout_check,
        ):
            widget.setEnabled(connected or self._bt_link is not None)
        for index in range(2):
            getattr(self, f"ads{index + 1}_mode").setEnabled(connected or self._bt_link is not None)
            getattr(self, f"ads{index + 1}_sps").setEnabled(connected or self._bt_link is not None)
            getattr(self, f"ads{index + 1}_gain").setEnabled(connected or self._bt_link is not None)
        if not connected and self._bt_link is None:
            self.state_label.setText(message or "Connect the board to control its power state.")

    def refresh(self) -> None:
        if not self._dispatch(b"|status;"):
            self.state_label.setText("Connect the board to read its power state.")

    def _send_key(self, key: str, value: object) -> bool:
        command = f"|{key}={value};".encode("ascii")
        if not self._dispatch(command):
            self.state_label.setText("Connect the board first.")
            return False
        return True

    def _apply_converter(self, index: int) -> None:
        mode = str(getattr(self, f"ads{index + 1}_mode").currentData())
        sps = int(getattr(self, f"ads{index + 1}_sps").currentData() or 0)
        gain = int(getattr(self, f"ads{index + 1}_gain").currentData() or 0)
        # Rate and gain first: the mode decides whether the chip is converting, and
        # setting it last means the new rate is in force when conversion starts.
        self._send_key(f"ads{index + 1}rate", sps)
        self._send_key(f"ads{index + 1}gain", gain)
        self._send_key(f"ads{index + 1}", mode)

    def restore_converters(self) -> bool:
        """Put both converters back to their normal, fully programmed state."""

        return self._send_key("restore", 1)

    def _apply_ads_pwdn(self) -> None:
        # Checked means "hold the pin low", i.e. shut both converters down.
        self._send_key("adspwdn", 0 if self.ads_pwdn_check.isChecked() else 1)

    def _apply_ads_reset(self) -> None:
        self._send_key("adsreset", 0 if self.ads_reset_check.isChecked() else 1)

    def _apply_ads_clkout(self) -> None:
        self._send_key("adsclkout", 1 if self.ads_clkout_check.isChecked() else 0)

    def _apply_bt_power(self) -> bool:
        """Request a TX power; the firmware replies with the level it really holds."""

        requested = int(self.bt_power_combo.currentData() or 0)
        self._bt_power_requested = requested
        if not self._send_key("btdbm", requested):
            self._bt_power_requested = None
            return False
        # Belt and braces: the firmware emits a status by itself, but ask again so
        # the read-back is refreshed even if that line is lost in a busy stream.
        self.refresh()
        return True

    def _apply_stream(self) -> None:
        self._send_key("stream", 1 if self.stream_button.isChecked() else 0)
        self._set_stream_button(None)
        self.refresh()

    def _set_stream_button(self, streaming: Optional[bool]) -> None:
        """Point the button at the next action, not the current state."""

        if streaming is not None:
            self.stream_button.blockSignals(True)
            self.stream_button.setChecked(streaming)
            self.stream_button.blockSignals(False)
        running = self.stream_button.isChecked()
        self.stream_button.setText("Stop stream" if running else "Start stream")

    def _sleep(self, mode: str) -> None:
        milliseconds = int(self.light_ms_spin.value()) if mode == "light" else int(self.deep_ms_spin.value())
        if self._send_key(mode, milliseconds):
            if mode == "deep":
                self.state_label.setText(
                    f"Deep sleep for {milliseconds} ms requested. The board powers down and "
                    "restarts when the timer expires - the link will drop and come back."
                )
            else:
                self.state_label.setText(
                    f"Light sleep for {milliseconds} ms requested. The link is silent until the "
                    "board wakes; it will report the wake cause then."
                )

    def _pin_level(self, fields: dict[str, str], key: str) -> Optional[bool]:
        """The physical level of a shared converter pin, as a bool (high = True).

        Firmware protocol 2 reports the level directly. Anything older reported the
        *flag* "this pin is being held low", so the value has to be inverted to mean
        the same thing. Getting this wrong is what made the panel show a healthy
        board as powered down, so the conversion lives in one place.
        """

        value = fields.get(key)
        if value is None:
            return None
        try:
            reported = int(value)
        except ValueError:
            return None
        legacy = self._legacy_firmware(fields)
        return (reported == 0) if legacy else (reported != 0)

    def _legacy_firmware(self, fields: dict[str, str]) -> bool:
        try:
            return int(fields.get("FW", "1")) < 2
        except ValueError:
            return True

    def apply_state(self, fields: dict[str, str]) -> None:
        """Reflect one ``[PWR]`` line into the widgets without signalling."""

        self.apply_state_dict(fields)

    def apply_state_dict(self, fields: dict[str, str]) -> None:
        self._state = dict(fields)
        if not hasattr(self, "link_trace"):
            self.link_trace: list[str] = []

        def flag(key: str) -> Optional[bool]:
            value = fields.get(key)
            if value is None:
                return None
            try:
                return int(value) > 0
            except ValueError:
                return None

        def set_check(widget: QCheckBox, value: Optional[bool]) -> None:
            if value is None:
                return
            widget.blockSignals(True)
            widget.setChecked(value)
            widget.blockSignals(False)

        for index in (0, 1):
            token = fields.get(f"ADS{index + 1}", "")
            combo = getattr(self, f"ads{index + 1}_mode")
            position = combo.findData(token)
            if position >= 0:
                combo.blockSignals(True)
                combo.setCurrentIndex(position)
                combo.blockSignals(False)

        set_check(self.lis3dh_check, flag("LIS3DH"))
        set_check(self.sd_check, flag("SD"))
        set_check(self.bt_check, flag("BT"))
        set_check(self.bt_sleep_check, flag("BTSLEEP"))
        set_check(self.test_check, flag("TEST"))
        # These two boxes are inverted: checked means the pin is driven low, so a 1
        # in the report ("pin is high") means the box is clear.
        pwdn_level = self._pin_level(fields, "PWDN")
        reset_level = self._pin_level(fields, "RESET")
        set_check(self.ads_pwdn_check, None if pwdn_level is None else not pwdn_level)
        set_check(self.ads_reset_check, None if reset_level is None else not reset_level)
        set_check(self.ads_clkout_check, flag("CLKOUT"))
        if hasattr(self, "stream_button"):
            self._set_stream_button(flag("STREAM"))

        dbm = fields.get("BTDBM")
        if dbm is not None:
            position = self.bt_power_combo.findData(int(dbm) if dbm.lstrip("-").isdigit() else 0)
            if position >= 0:
                self.bt_power_combo.blockSignals(True)
                self.bt_power_combo.setCurrentIndex(position)
                self.bt_power_combo.blockSignals(False)
            readback = getattr(self, "bt_readback_label", None)
            if readback is not None:
                try:
                    held = int(dbm)
                except ValueError:
                    held = 0
                # Compare against what was *requested*, which the combo no longer
                # holds: it has just been moved to the value read back.
                asked = getattr(self, "_bt_power_requested", None)
                if not flag("BT"):
                    readback.setText("n/a - the radio stack is stopped")
                    readback.setStyleSheet("font-weight: 600; color: #b91c1c;")
                elif asked is None:
                    readback.setText(f"{held:+d} dBm (read from the controller)")
                    readback.setStyleSheet("font-weight: 600; color: #166534;")
                elif held == asked:
                    readback.setText(f"{held:+d} dBm (confirmed by the controller)")
                    readback.setStyleSheet("font-weight: 600; color: #166534;")
                else:
                    readback.setText(f"{held:+d} dBm - the controller did not take {asked:+d} dBm")
                    readback.setStyleSheet("font-weight: 600; color: #b45309;")
                self._bt_power_requested = None

        def describe(key: str, suffix: str = "") -> str:
            return f"{fields.get(key, '?')}{suffix}"

        def raw(key: str) -> Optional[int]:
            value = fields.get(key)
            if value is None:
                return None
            try:
                return int(value)
            except ValueError:
                return None

        def chip_line(index: int) -> str:
            """One converter, including the proof that it is really converting."""

            name = f"ADS{index + 1}"
            link = raw(f"{name}_LINK")
            problems = raw(f"{name}_PROB")
            drdy = raw(f"{name}_DRDY")
            rate = raw("SPS")
            if not link:
                return f"{name} {describe(name):>9s}  not answering on SPI"
            # What DRDY *should* look like in the 60 ms window the firmware samples.
            expected = round((rate or 0) * 0.060)
            verdict = ""
            if drdy is not None and expected:
                if drdy == 0:
                    verdict = "  NO DATA"
                elif drdy >= expected * 0.5:
                    verdict = "  converting"
                else:
                    verdict = "  slow"
            problems_text = (
                "not checked while streaming"
                if problems is None or problems < 0
                else str(problems)
            )
            return (
                f"{name} {describe(name):>9s}  ID {describe(f'{name}_ID')}  "
                f"reg problems {problems_text}  "
                f"DRDY {drdy if drdy is not None else '?'}/{expected} in 60 ms{verdict}"
            )

        pwdn = self._pin_level(fields, "PWDN")
        reset = self._pin_level(fields, "RESET")
        lis_raw = raw("LIS3DH")
        fw = raw("FW")
        lines = [
            chip_line(0),
            chip_line(1),
            f"data rate {describe('SPS', ' SPS')}   "
            f"PWDN {('HIGH (converter on)' if pwdn else 'LOW (converter off)') if pwdn is not None else '?'}   "
            f"RESET {('released' if reset else 'held LOW') if reset is not None else '?'}   "
            f"CLKOUT {'on' if flag('CLKOUT') else 'off'}",
            f"LIS3DH {'not present' if lis_raw == -1 else ('on' if lis_raw else 'powered down')}   "
            f"SD {'mounted' if flag('SD') else 'released'}   "
            f"test stream {'on' if flag('TEST') else 'off'}",
            f"Bluetooth {'running' if flag('BT') else 'off'}   TX {describe('BTDBM', ' dBm')}"
            f"   modem sleep {'on' if flag('BTSLEEP') else 'off'}",
            f"streaming {'yes' if flag('STREAM') else 'no'}   packets sent {describe('TX')}"
            f"   free heap {describe('HEAP', ' B')}",
        ]
        if fw is None or fw < 2:
            # The conversion above already accounts for the older reporting
            # convention, so the state is right - but the board cannot report the
            # pin level itself, and the rest of this panel assumes a matching image.
            lines.append(
                "Board firmware is older than this panel (no FW tag). PWDN/RESET above are "
                "converted from the older convention; reflash to get pin-level reporting."
            )
        elif not flag("STREAM") and (pwdn is False or reset is False):
            lines.append("A converter pin is held low: clear its box again to bring the converters back.")
        elif not flag("STREAM"):
            for index in (0, 1):
                if raw(f"ADS{index + 1}_LINK") and raw(f"ADS{index + 1}_DRDY") == 0:
                    lines.append("Press Restore converters to normal to re-programme them.")
                    break
        self.state_label.setText("\n".join(lines))


class AnnotationDialog(QDialog):
    """Edit the label, note and time range of a recording section."""

    def __init__(self, annotation: Annotation, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Section")
        self.resize(420, 220)

        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.label_edit = QLineEdit(annotation.label)
        form.addRow("Label", self.label_edit)
        self.note_edit = QLineEdit(annotation.note)
        self.note_edit.setPlaceholderText("optional note, e.g. electrode montage or task")
        form.addRow("Note", self.note_edit)
        self.start_spin = QDoubleSpinBox()
        self.start_spin.setRange(0.0, 1_000_000.0)
        self.start_spin.setDecimals(2)
        self.start_spin.setSuffix(" s")
        self.start_spin.setValue(float(annotation.start_s))
        form.addRow("Start", self.start_spin)
        self.end_spin = QDoubleSpinBox()
        self.end_spin.setRange(0.0, 1_000_000.0)
        self.end_spin.setDecimals(2)
        self.end_spin.setSuffix(" s")
        self.end_spin.setValue(float(annotation.end_s))
        form.addRow("End", self.end_spin)
        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def values(self) -> Annotation:
        start = float(self.start_spin.value())
        end = float(self.end_spin.value())
        if end < start:
            start, end = end, start
        return Annotation(
            label=self.label_edit.text().strip() or "section",
            start_s=start,
            end_s=end,
            note=self.note_edit.text().strip(),
        )


def _install_fatal_log() -> None:
    """Record an exception that escapes a Qt callback before PyQt aborts.

    PyQt terminates the process when a Python exception escapes a slot or an
    event filter, and on a console build that shows up as a bare traceback that
    scrolls away. The hook keeps the window from dying silently: it writes the
    traceback next to the settings file so it can be read afterwards.
    """

    log_path = Path(__file__).resolve().parent / "settings" / "crash.log"

    def hook(exc_type, exc_value, exc_tb):
        if issubclass(exc_type, KeyboardInterrupt):
            return
        import traceback as _traceback

        text = "".join(_traceback.format_exception(exc_type, exc_value, exc_tb))
        sys.stderr.write(text)
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("a", encoding="utf-8") as handle:
                handle.write(f"\n=== {datetime.now().isoformat(timespec='seconds')} ===\n{text}")
        except OSError:
            pass

    sys.excepthook = hook


def main() -> int:
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    _install_fatal_log()
    app = QApplication.instance() or QApplication(sys.argv)
    install_wheel_guard(app)
    window = LiveRecorderWindow()
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())