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
    QScrollArea,
    QSizePolicy,
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
    MeasurementInfo,
    MeasurementNameDialog,
    install_wheel_guard,
)
from live_protocol import (
    ADS_SPI_CLOCK_HZ,
    BOARD_CHANNEL_CHOICES,
    CODE_BY_SAMPLE_RATE,
    DemoBoardPort,
    GAIN_LABELS,
    HardwareSettings,
    INPUT_TYPE_LABELS,
    MixedStreamParser,
    OpenBCIRecording,
    PacketDecoder,
    REFERENCE_MODE_SRB1,
    REFERENCE_MODE_SRB2,
    RingBuffer,
    Annotation,
    SAMPLE_RATES,
    SAMPLE_RATE_BY_CODE,
    SettingsStore,
    VREF_VOLTS,
    build_reference_command,
    max_sample_rate_from_spi,
    max_sustained_packet_rate,
    max_usable_sample_rate,
    save_annotation_sidecar,
    spi_frame_microseconds,
    supported_packet_rates,
    StreamingFilter,
    TextLine,
    ChannelSetting,
    build_channel_command,
    build_sample_rate_command,
    compare_configuration,
    expected_device_registers,
    format_duration,
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
UI_INTERVAL_MS = 33
FFT_INTERVAL_TICKS = 4
VERIFY_TIMEOUT_MS = 4000
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


class SerialWorker(QThread):
    """Owns the serial port and turns the byte stream into data and text."""

    batchReady = pyqtSignal(object)
    textReady = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, transport, board_channels: int, gains: Sequence[float], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._transport = transport
        self._decoder = PacketDecoder(board_channels)
        self._decoder.set_gains(gains)
        self._parser = MixedStreamParser()
        self._commands: "queue.Queue[bytes]" = queue.Queue()
        self._stop_event = threading.Event()
        self._gain_lock = threading.Lock()

    def send(self, payload: bytes) -> None:
        self._commands.put(payload)

    def set_gains(self, gains: Sequence[float]) -> None:
        with self._gain_lock:
            self._decoder.set_gains(gains)

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001 - closing a dead port must not raise
            pass
        self.wait(2000)

    def run(self) -> None:  # noqa: D401 - QThread entry point
        samples: list[np.ndarray] = []
        aux_values: list[np.ndarray] = []
        texts: list[TextLine] = []
        last_emit = time.monotonic()
        try:
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
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass


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

    HEADERS = ("", "Channel", "On", "Gain", "Input", "Bias", "Status")

    def __init__(
        self,
        hardware: HardwareSettings,
        statuses: Optional[dict[int, str]] = None,
        parent: Optional[QWidget] = None,
        link_limit: Optional[int] = None,
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
        self._connected = False

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        intro = QLabel(
            "Settings are saved automatically and replayed on every connect. "
            "Use the bulk row to set every channel at once, or edit single channels for mixed measurements. "
            "The reference mode is one setting for the whole board."
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
        form.addSpacing(16)
        form.addWidget(QLabel("ADS sample rate"))
        self.rate_combo = QComboBox()
        for rate, code in SAMPLE_RATES:
            label = f"{rate} SPS" + ("  (over the serial link)" if self._link_limit and rate > self._link_limit else "")
            self.rate_combo.addItem(label, code)
        self.rate_combo.setCurrentIndex(max(self.rate_combo.findData(self._rate_code), 0))
        form.addWidget(self.rate_combo)
        form.addStretch(1)
        self.rate_note = QLabel("")
        form.addWidget(self.rate_note)
        layout.addLayout(form)

        reference_group = QGroupBox("Reference / SRB")
        reference_layout = QVBoxLayout(reference_group)
        reference_row = QHBoxLayout()
        self.reference_combo = QComboBox()
        for label, value in REFERENCE_MODE_SHORT:
            self.reference_combo.addItem(label, value)
        self.reference_combo.setCurrentIndex(max(self.reference_combo.findData(self._reference_mode), 0))
        reference_row.addWidget(self.reference_combo, 1)
        reference_layout.addLayout(reference_row)
        self.reference_note = QLabel("")
        self.reference_note.setWordWrap(True)
        self.reference_note.setStyleSheet("color: #475569; font-size: 11px;")
        reference_layout.addWidget(self.reference_note)
        layout.addWidget(reference_group)

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
        self.reference_combo.currentIndexChanged.connect(self._on_reference_changed)
        self._populate_table()
        self._update_rate_note()
        self._update_reference_note()

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
            status = QLabel(self._status_text(self._statuses.get(row, "")))

            widgets = (select, active, gain, input_combo, bias)
            if padding:
                for widget in widgets:
                    widget.setEnabled(False)
                label.setForeground(QColor("#94a3b8"))
            self.table.setCellWidget(row, 2, active)
            self.table.setCellWidget(row, 3, gain)
            self.table.setCellWidget(row, 4, input_combo)
            self.table.setCellWidget(row, 5, bias)
            self.table.setCellWidget(row, 6, status)
            self.table.setRowHeight(row, 30)

            self._rows.append(
                {
                    "select": select,
                    "active": active,
                    "gain": gain,
                    "input": input_combo,
                    "bias": bias,
                    "status": status,
                    "padding": padding,
                }
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
        return verify_group

    def _sync_row(self, row: int) -> None:
        widgets = self._rows[row]
        setting = self._channels[row]
        widgets["active"].setChecked(setting.active)
        widgets["gain"].setCurrentIndex(setting.gain_ordinal)
        widgets["input"].setCurrentIndex(setting.input_ordinal)
        widgets["bias"].setCurrentIndex(0 if setting.bias else 1)

    def _collect_from_rows(self) -> None:
        for row, widgets in enumerate(self._rows):
            if widgets["padding"] or row >= len(self._channels):
                continue
            setting = self._channels[row]
            setting.active = widgets["active"].isChecked()
            setting.gain_ordinal = int(widgets["gain"].currentIndex())
            setting.input_ordinal = int(widgets["input"].currentIndex())
            setting.bias = widgets["bias"].currentIndex() == 0

    def collect(self) -> HardwareSettings:
        self._collect_from_rows()
        hardware = HardwareSettings(
            board_channels=self._board_channels,
            sample_rate_code=self._rate_code,
            reference_mode=self._reference_mode,
            channels=[channel.copy() for channel in self._channels],
        )
        hardware.sync_reference()
        return hardware

    def _on_reference_changed(self) -> None:
        self._reference_mode = str(self.reference_combo.currentData() or REFERENCE_MODE_SRB2)
        self._update_reference_note()

    def _update_reference_note(self) -> None:
        if self._reference_mode == REFERENCE_MODE_SRB1:
            text = (
                "SRB1 closes one switch inside each ADS1299 (MISC1 bit 5), so every channel's negative "
                "input goes to the SRB1 pin at once. Connect the reference electrode to SRB1."
            )
        elif self._reference_mode == REFERENCE_MODE_SRB2:
            text = (
                "SRB2 switches each channel's negative input to the SRB2 pin (CHnSET bit 3). "
                "Connect the reference electrode to SRB2."
            )
        else:
            text = (
                "True differential: no SRB routing at all. Every channel needs its own INP and INN "
                "electrode, and the bias/DRL electrode keeps the common mode in range."
            )
        self.reference_note.setText(text)

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

    def _on_rate_changed(self) -> None:
        self._rate_code = int(self.rate_combo.currentData() or 6)
        self._update_rate_note()

    def _update_rate_note(self) -> None:
        rate = SAMPLE_RATE_BY_CODE.get(self._rate_code, 250)
        if self._board_channels > 8:
            text = f"per channel: {rate / 2:g} Hz (Cyton+Daisy interleaving)"
        else:
            text = f"per channel: {rate:g} Hz"
        if self._link_limit:
            text += f"  |  serial link: up to {self._link_limit} SPS"
        text += f"  |  SPI link: up to {max_sample_rate_from_spi(ADS_SPI_CLOCK_HZ, 2)} SPS"
        self.rate_note.setText(text)

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
        self._reference_mode = REFERENCE_MODE_SRB2
        self.reference_combo.setCurrentIndex(max(self.reference_combo.findData(self._reference_mode), 0))
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
        self._verify_lines: list[str] = []
        self._verify_pending = False
        self._pending_verify_hardware: Optional[HardwareSettings] = None
        self.verify_result_text = ""
        self.verify_result_state = "info"

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

        self._build_ui()
        self._build_menus()
        self.setStatusBar(QStatusBar())
        self._load_settings_into_ui()
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
        self.accel_panel = AccelerometerPanel()
        right.addWidget(self.spectrum_panel)
        right.addWidget(self.accel_panel)
        right.setSizes([470, 280])
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
        layout.addWidget(self._build_recording_group())
        layout.addWidget(self._build_annotation_group())
        layout.addWidget(self._build_channel_group())
        layout.addWidget(self._build_hardware_group())
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
        for baud in (115200, 230400, 460800, 921600):
            self.baud_combo.addItem(str(baud), baud)
        baud_row.addWidget(self.baud_combo)
        baud_row.addStretch(1)
        layout.addLayout(baud_row)

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

    def open_name_dialog(self) -> None:
        """Name the measurement with dropdowns instead of typing.

        The reference mode is part of the name, so picking a name also drives the
        hardware: choosing "true differential" here selects that reference mode
        and writes it to the board if one is connected.
        """

        dialog = MeasurementNameDialog(self.measurement, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.measurement = dialog.measurement()
        self.session_name_edit.setText(dialog.name_stem())
        self.settings.session_name = self.session_name_edit.text()
        self.settings.measurement = self.measurement.as_dict()
        self._save_settings()
        self._apply_measurement_reference(self.measurement.reference_mode)
        self._update_measurement_label()

    def _apply_measurement_reference(self, mode: str) -> None:
        """Make the hardware reference follow the name (and the board if online)."""

        if mode not in REFERENCE_MODE_CODES:
            return
        changed = mode != self.hardware.normalised_reference_mode()
        self.hardware.set_reference_mode(mode)
        self.measurement.reference_mode = mode
        position = self.reference_combo.findData(mode)
        self.reference_combo.blockSignals(True)
        self.reference_combo.setCurrentIndex(position if position >= 0 else 0)
        self.reference_combo.blockSignals(False)
        self._update_reference_note()
        self._save_settings()
        if changed and self.connected and self.recording is None:
            self._log("in", f"[settings] reference mode {mode} from the recording name", "info")
            self.apply_hardware()

    def _refresh_session_name_reference(self) -> None:
        """Rewrite the ``ref-...`` token of the current name after a mode change."""

        mode = self.hardware.normalised_reference_mode()
        self.measurement.reference_mode = mode
        current = self.session_name_edit.text().strip()
        if not current:
            return
        token = f"ref-{mode}"
        parts = current.split("_")
        replaced = False
        for index, part in enumerate(parts):
            if part.startswith("ref-"):
                parts[index] = token
                replaced = True
                break
        updated = "_".join(parts) if replaced else f"{current}_{token}"
        if updated != current:
            self.session_name_edit.setText(updated)
            self.settings.session_name = updated

    def _update_measurement_label(self) -> None:
        label = getattr(self, "measurement_label", None)
        if label is None:
            return
        info = self.measurement.normalised()
        info.reference_mode = self.hardware.normalised_reference_mode()
        text = "  |  ".join(info.describe_lines())
        if not self.session_name_edit.text().strip():
            text = "Name this measurement with the Name... button.   |   " + text
        label.setText(text)

    def _build_recording_group(self) -> QWidget:
        group = QGroupBox("Recording")
        layout = QVBoxLayout(group)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Session"))
        self.session_name_edit = QLineEdit()
        self.session_name_edit.setPlaceholderText("session name")
        self.session_name_edit.setToolTip(
            "Used for the CSV file name and reused automatically when you export from the analyzer."
        )
        name_row.addWidget(self.session_name_edit, 1)
        self.name_button = QPushButton("Name...")
        self.name_button.setToolTip("Build the name from dropdowns: measurement, connection, stimulus and reference")
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
        self.record_visible_check.setToolTip(
            "Off (default): the CSV always stores every channel of the board.\n"
            "On: only the channels ticked in the 'Channels' group are written, which keeps "
            "files small when you only need a few inputs."
        )
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

        hint = _shrinkable(QLabel(
            "A section is a named time range inside the recording. Start one before each "
            "measurement condition, end it when you switch condition, then analyze a section on its own."
        ))
        hint.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(hint)

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
        hint = _shrinkable(QLabel("Tick a channel to show it in the plots. Recording follows this "
                                 "only when 'Record only checked channels' is enabled."))
        hint.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(hint)
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
        self.hardware_button = QPushButton("Hardware settings")
        self.hardware_button.setToolTip(
            "Per-channel gain/input/bias, the reference mode and the Verify button live there."
        )
        self.hardware_button.clicked.connect(self.open_hardware_settings)
        button_row.addWidget(self.hardware_button)
        self.reapply_button = QPushButton("Re-apply")
        self.reapply_button.setToolTip("Send the saved sample rate, reference mode and channel settings again")
        self.reapply_button.clicked.connect(lambda: self.apply_hardware())
        button_row.addWidget(self.reapply_button)
        layout.addLayout(button_row)

        reference_row = QHBoxLayout()
        reference_row.addWidget(QLabel("Reference"))
        self.reference_combo = QComboBox()
        for label, value in REFERENCE_MODE_SHORT:
            self.reference_combo.addItem(label, value)
        self.reference_combo.setToolTip(
            "One setting for the whole board.\n"
            "True differential: every channel measures its own INP- INN pair.\n"
            "SRB1: all negative inputs are tied to the SRB1 pin (one switch per ADS1299, "
            "so it can never be per channel).\n"
            "SRB2: every channel's negative input is switched to the SRB2 pin.\n"
            "The mode is applied last when the settings are written, so a wrong combination "
            "cannot be left on the chip."
        )
        self.reference_combo.currentIndexChanged.connect(self._on_reference_changed)
        reference_row.addWidget(self.reference_combo, 1)
        layout.addLayout(reference_row)

        self.reference_note = _shrinkable(QLabel(""))
        self.reference_note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.reference_note)

        quick = QGridLayout()
        quick.setHorizontalSpacing(4)
        quick.setVerticalSpacing(3)
        self.quick_gain = QComboBox()
        self.quick_gain.addItems(list(GAIN_LABELS))
        self.quick_gain.setCurrentIndex(len(GAIN_LABELS) - 1)
        self.quick_input = QComboBox()
        self.quick_input.addItems(list(INPUT_TYPE_LABELS))
        self.quick_bias = QComboBox()
        self.quick_bias.addItems(["Include", "Exclude"])
        quick.addWidget(QLabel("Gain"), 0, 0)
        quick.addWidget(self.quick_gain, 0, 1)
        quick.addWidget(QLabel("Input"), 1, 0)
        quick.addWidget(self.quick_input, 1, 1)
        quick.addWidget(QLabel("Bias"), 2, 0)
        quick.addWidget(self.quick_bias, 2, 1)
        quick.setColumnStretch(1, 1)
        layout.addLayout(quick)

        self.quick_apply_button = QPushButton("Apply to all")
        self.quick_apply_button.clicked.connect(self._apply_quick_settings)
        layout.addWidget(self.quick_apply_button)

        self.apply_status_label = _shrinkable(QLabel("Saved settings are written automatically after connect."))
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
        note = _shrinkable(QLabel("Filters affect the live traces and FFT only; recordings stay raw."))
        note.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(note, 4, 0, 1, 2)
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
        hardware_action = QAction("Hardware settings...", self)
        hardware_action.triggered.connect(self.open_hardware_settings)
        device_menu.addAction(hardware_action)
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
        position = self.reference_combo.findData(self.hardware.normalised_reference_mode())
        self.reference_combo.blockSignals(True)
        self.reference_combo.setCurrentIndex(position if position >= 0 else 0)
        self.reference_combo.blockSignals(False)
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

    def _on_reference_changed(self, *_args) -> None:
        mode = str(self.reference_combo.currentData() or REFERENCE_MODE_SRB2)
        if mode == self.hardware.reference_mode:
            self._update_reference_note()
            return
        if self.recording is not None:
            # Reject the change instead of leaving the combo and the board apart.
            self.reference_combo.blockSignals(True)
            self.reference_combo.setCurrentIndex(
                max(self.reference_combo.findData(self.hardware.reference_mode), 0)
            )
            self.reference_combo.blockSignals(False)
            QMessageBox.information(
                self, "Reference", "Stop the recording before changing the reference mode."
            )
            return
        self.hardware.set_reference_mode(mode)
        if self._settings_dialog is not None:
            position = self._settings_dialog.reference_combo.findData(mode)
            if position >= 0:
                self._settings_dialog.reference_combo.setCurrentIndex(position)
        self._update_reference_note()
        # The name carries the reference, so it has to follow a hardware change.
        self._refresh_session_name_reference()
        self._update_measurement_label()
        self._save_settings()
        if self.connected:
            self.apply_hardware()

    def _update_reference_note(self) -> None:
        mode = self.hardware.normalised_reference_mode()
        if mode == REFERENCE_MODE_SRB1:
            text = "SRB1 pin is the shared reference for every input of a converter."
        elif mode == REFERENCE_MODE_SRB2:
            text = "SRB2 pin is the shared reference (per-channel switch, applied to all)."
        else:
            text = "No SRB routing: every channel needs its own INP and INN."
        self.reference_note.setText(text)

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

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        if list_ports is not None:
            for info in sorted(list_ports.comports(), key=lambda port: port.device):
                description = (info.description or "").strip()
                label = f"{info.device}  {description}" if description else info.device
                self.port_combo.addItem(label, info.device)
        self.port_combo.addItem("DEMO  synthetic board (no hardware)", DEMO_PORT)
        target = self.settings.last_port or current
        index = self.port_combo.findData(target) if target else -1
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)
        if list_ports is None:
            self.connection_label.setText("pyserial is not installed - only the DEMO source is available")
        self._save_settings()

    def _autoconnect_port(self) -> Optional[str]:
        """Port to open: the last one used, else COM5, else the only candidate.

        The board may be plugged in after the application started, so this is
        evaluated again on every retry rather than once at startup.
        """

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
        """Rescan the serial ports without disturbing the current selection."""

        if list_ports is None:
            return
        current = self.port_combo.currentData()
        discovered: list[tuple[str, str]] = []
        for info in sorted(list_ports.comports(), key=lambda port: port.device):
            description = (info.description or "").strip()
            label = f"{info.device}  {description}" if description else info.device
            discovered.append((label, info.device))
        existing = [
            (self.port_combo.itemText(index), str(self.port_combo.itemData(index)))
            for index in range(self.port_combo.count())
            if self.port_combo.itemData(index) != DEMO_PORT
        ]
        if discovered == existing:
            return
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

    def _on_connect_toggled(self, checked: bool) -> None:
        if checked:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        if self.worker is not None:
            return
        key = self.port_combo.currentData()
        if not key:
            self._set_connection_state(False, "No serial port selected")
            return
        baud = int(self.baud_combo.currentData() or DEFAULT_BAUD)
        try:
            if key == DEMO_PORT:
                transport = DemoBoardPort(self.hardware.board_channels, self.hardware.sample_rate_code)
                transport.open()
            else:
                if serial is None:
                    raise RuntimeError("pyserial is not installed")
                transport = serial.Serial(key, baud, timeout=0.02)
        except Exception as exc:  # noqa: BLE001 - surfaced to the user
            self._set_connection_state(False, f"Could not open {key}: {exc}")
            self._log("in", f"Could not open {key}: {exc}", "error")
            return

        self.worker = SerialWorker(transport, self.channel_count, self.hardware.physical_gains(), self)
        self.worker.batchReady.connect(self._on_batch)
        self.worker.textReady.connect(self._on_text)
        self.worker.failed.connect(self._on_worker_failed)
        self.worker.start()

        self.connected = True
        self.info_received = False
        self.profile_checked = False
        self.probe_attempts = 0
        self.board_info = {}
        self.apply_status = {}
        self._set_connection_state(True, f"Connected to {self.port_combo.currentText()}")
        self.board_info_label.setText("Requesting board information...")
        self._set_apply_summary("Requesting board information...")
        self._save_settings()
        self._probe_timer.start(1200)

    def _disconnect(self) -> None:
        self._probe_timer.stop()
        self._info_timer.stop()
        self._apply_timer.stop()
        self._apply_current = None
        self._verify_timer.stop()
        self._verify_pending = False
        self._verify_lines = []
        self._set_dialog_connected(False)
        self._set_verify_result("")
        if self.recording is not None:
            self.stop_recording()
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
        if self.auto_connect_check.isChecked():
            # Keep looking: the user unplugged the board or closed the port.
            self._autoconnect_timer.start()
        self._set_connection_state(False, "Not connected")
        self.board_info_label.setText("Board details appear here after connect.")

    def _on_worker_failed(self, message: str) -> None:
        self._log("in", message, "error")
        self.statusBar().showMessage(message, 8000)
        if self.worker is not None:
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
            self._set_apply_summary(
                "The board did not answer the version request. Check the port and that the firmware is running.",
                error=True,
            )
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
        self._set_connection_state(True, f"Connected to {self.port_combo.currentText()}")
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
    def _build_accel_group(self) -> QWidget:
        group = QGroupBox("Accelerometer")
        layout = QGridLayout(group)
        layout.addWidget(QLabel("Full scale"), 0, 0)
        self.accel_scale_combo = QComboBox()
        for value in (0.5, 1.0, 2.0, 4.0):
            self.accel_scale_combo.addItem(f"+/- {value:g} g", value)
        layout.addWidget(self.accel_scale_combo, 0, 1)
        layout.addWidget(QLabel("Motion alarm"), 1, 0)
        self.accel_threshold_spin = QDoubleSpinBox()
        self.accel_threshold_spin.setRange(0.005, 2.0)
        self.accel_threshold_spin.setSingleStep(0.01)
        self.accel_threshold_spin.setDecimals(3)
        self.accel_threshold_spin.setSuffix(" g")
        layout.addWidget(self.accel_threshold_spin, 1, 1)
        layout.addWidget(QLabel("Baseline"), 2, 0)
        self.accel_baseline_spin = QDoubleSpinBox()
        self.accel_baseline_spin.setRange(0.3, 10.0)
        self.accel_baseline_spin.setSingleStep(0.5)
        self.accel_baseline_spin.setDecimals(1)
        self.accel_baseline_spin.setSuffix(" s")
        layout.addWidget(self.accel_baseline_spin, 2, 1)
        self.accel_calibrate_button = QPushButton("Calibrate from rest")
        self.accel_calibrate_button.setToolTip("With the board lying still, set the scale so the current reading equals 1 g")
        self.accel_calibrate_button.clicked.connect(self._calibrate_accelerometer)
        layout.addWidget(self.accel_calibrate_button, 3, 0, 1, 2)
        self.accel_scale_label = QLabel("")
        self.accel_scale_label.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(self.accel_scale_label, 4, 0, 1, 2)
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
        for monitor in list(self.monitors):
            if monitor.isVisible():
                monitor.append_record(*record)

    def apply_hardware(
        self,
        channels: Optional[Sequence[int]] = None,
        hardware: Optional[HardwareSettings] = None,
    ) -> None:
        """Write the saved sample rate and channel settings to the board."""

        if hardware is not None:
            self._set_hardware(hardware)
        if not self.connected or self.worker is None:
            self._set_apply_summary("Connect the board before writing settings.", error=True)
            return
        if self.recording is not None:
            self._set_apply_summary("Stop the recording before changing hardware settings.", error=True)
            return
        if self._apply_current is not None and self._apply_timer.isActive():
            self._set_apply_summary("A settings write is already in progress.", error=True)
            return

        requested_rate = self.hardware.packet_rate_hz
        ba = self._current_baud()
        serial_limit = max_sustained_packet_rate(ba)
        spi_limit = self._spi_sample_rate()
        limit = max_usable_sample_rate(ba, ADS_SPI_CLOCK_HZ, 2)
        if requested_rate > limit:
            supported = [rate for rate, _code in supported_packet_rates(ba) if rate <= spi_limit]
            if spi_limit < serial_limit:
                reason = (
                    f"{spi_limit:g} SPS is the SPI limit "
                    f"({spi_frame_microseconds(ADS_SPI_CLOCK_HZ, 2):.0f} us per sample at "
                    f"{ADS_SPI_CLOCK_HZ / 1e6:g} MHz)"
                )
            else:
                reason = f"{ba / 1000:.0f} kbit/s of serial carries {serial_limit:g} SPS"
            # SAMPLE_RATES is ordered fastest-first, so the first supported
            # entry is the fastest rate that still fits the link.
            fallback = max(supported) if supported else 250
            message = (
                f"{requested_rate:g} SPS is above the link budget: {reason}; using {fallback:g} SPS instead."
            )
            self._log("in", f"[settings] {message}", "error")
            self._set_apply_summary(message, error=True)
            self._rate_downgrade_note = message
            self.hardware.sample_rate_code = CODE_BY_SAMPLE_RATE[fallback]
            self._save_settings()
            self._update_hardware_labels()
            # The buffers, the FFT axis and the time base are sized from this.
            self._rebuild_buffers()

        if abs(self.sample_rate - self.hardware.sample_rate_hz) > 1e-9:
            # The write above changed the rate, and the time base, FFT axis and
            # ring buffers are all sized from it.
            self._rebuild_buffers()

        self.hardware.sync_reference()
        indices = list(channels) if channels is not None else real_channel_indices(self.hardware.board_channels)
        indices = [index for index in indices if index < len(self.hardware.channels)]
        plan: list[tuple[Optional[int], bytes, str]] = [
            (
                None,
                build_sample_rate_command(self.hardware.sample_rate_code),
                f"sample rate {self.hardware.packet_rate_hz:g} Hz",
            )
        ]
        for index in indices:
            plan.append((index, build_channel_command(index, self.hardware.channels[index]), f"channel {index + 1}"))
        # Last, so the SRB routing is always the final state on the chip and can
        # never be left half applied by a per-channel write.
        plan.append(
            (None, build_reference_command(self.hardware.reference_mode), f"reference {self.hardware.reference_mode}")
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

        if not report:
            self._set_verify_result("No register data came back from the board.", error=True)
            return

        checks, notes = compare_configuration(report, expected)
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
        if board_rate is not None and abs(board_rate - expected.packet_rate_hz) > 1e-6:
            problems.append(f"board sample rate is {board_rate} SPS")
        if board_spi is not None and board_spi != max_sample_rate_from_spi(ADS_SPI_CLOCK_HZ, 2):
            notes.append(f"board SPI budget is {board_spi} SPS, firmware constant here differs")

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
        position = self.reference_combo.findData(hardware.normalised_reference_mode())
        self.reference_combo.blockSignals(True)
        self.reference_combo.setCurrentIndex(position if position >= 0 else 0)
        self.reference_combo.blockSignals(False)
        self._update_reference_note()
        self._save_settings()
        if self.worker is not None:
            self.worker.set_gains(hardware.physical_gains())
        if rate_changed or board_changed:
            self._rebuild_buffers()
        self._update_hardware_labels()

    def _current_baud(self) -> int:
        return int(self.baud_combo.currentData() or DEFAULT_BAUD)

    def _link_packet_rate(self) -> int:
        return max_sustained_packet_rate(self._current_baud())

    def _spi_sample_rate(self) -> int:
        devices = 2
        return max_sample_rate_from_spi(ADS_SPI_CLOCK_HZ, devices)

    def _update_hardware_labels(self) -> None:
        board = "16 channels (Cyton + Daisy)" if self.hardware.board_channels > 8 else "8 channels (Cyton)"
        frame_us = spi_frame_microseconds(ADS_SPI_CLOCK_HZ, 2)
        spi_limit = self._spi_sample_rate()
        serial_limit = self._link_packet_rate()
        binding = "SPI" if spi_limit < serial_limit else "serial"
        text = (
            f"Board: {board}\n"
            f"ADS rate {self.hardware.packet_rate_hz:g} SPS, {self.hardware.sample_rate_hz:g} Hz per channel\n"
            f"SPI link ({ADS_SPI_CLOCK_HZ / 1e6:g} MHz, {frame_us:.0f} us/sample) supports up to {spi_limit:g} SPS\n"
            f"Serial link ({self._current_baud()} baud) supports up to {serial_limit:g} SPS\n"
            f"Limit in force: {binding}"
        )
        self.board_label.setText(text)
        self.board_label.setToolTip(
            "Each ADS1299 answers a 216-bit RDATAC frame and both converters are read inside the "
            "same sample period, so one sample costs two frames on the SPI bus. Both budgets keep "
            "10% headroom."
        )

    def _apply_quick_settings(self) -> None:
        if self.recording is not None:
            QMessageBox.information(self, "Hardware settings", "Stop the recording before changing hardware settings.")
            return
        gain = self.quick_gain.currentIndex()
        input_ordinal = self.quick_input.currentIndex()
        bias = self.quick_bias.currentIndex() == 0
        for index in real_channel_indices(self.hardware.board_channels):
            if index >= len(self.hardware.channels):
                continue
            setting = self.hardware.channels[index]
            setting.gain_ordinal = gain
            setting.input_ordinal = input_ordinal
            setting.bias = bias
        self.hardware.sync_reference()
        self._save_settings()
        self.apply_hardware()

    def open_hardware_settings(self) -> None:
        if self._settings_dialog is not None:
            self._settings_dialog.raise_()
            self._settings_dialog.activateWindow()
            return
        dialog = HardwareSettingsDialog(self.hardware, self.apply_status, self, link_limit=self._link_packet_rate())
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
        real = self._real_channel_mask()[:channels]
        flat = np.all(samples == 0.0, axis=0) & real
        limits = self._channel_full_scale()[:channels]
        saturated = np.all(np.abs(samples) >= (RAIL_FRACTION * limits), axis=0) & real
        self._zero_seconds = np.where(flat, self._zero_seconds + duration, 0.0)
        self._rail_seconds = np.where(saturated, self._rail_seconds + duration, 0.0)

        states: dict[int, str] = {}
        for index in range(self.channel_count):
            if index >= channels:
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
        stalled = sorted(index for index, state in self._channel_states.items() if state == "stall")
        railed = sorted(index for index, state in self._channel_states.items() if state == "rail")
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
            if self._verify_pending and line.text.strip().startswith("[CFG]"):
                if self._on_verify_line(line.text):
                    self._finish_verify()
                continue
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

        if self._dc_state is not None:
            mask = self._real_channel_mask()
            self.time_panel.set_dc(
                {
                    index: float(self._dc_state[index])
                    for index in range(min(len(self._dc_state), self.channel_count))
                    if index < mask.size and mask[index]
                }
            )

        window_count = int(self.time_panel.window_seconds * self.sample_rate)
        end_time = self.sample_total / self.sample_rate
        if not self.time_panel.paused:
            self.time_panel.render(self.display_ring.last(window_count), self.sample_rate, end_time, visible)

        if self._ui_ticks % FFT_INTERVAL_TICKS == 0:
            points = int(self.spectrum_panel.points_combo.currentData() or 1024)
            self.spectrum_panel.render(self.display_ring.last(points), self.sample_rate, visible)

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
                self.warning_label.setText(
                    f"Receiving {self._measured_rate:.0f} of {self.sample_rate:.0f} Hz - data is being lost. "
                    "Lower the ADS rate or raise the baud rate."
                )
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
        if self.recording is not None:
            parts.append("recording")
        self.statusBar().showMessage(" | ".join(parts))

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
            },
        )
        self.last_recording = path
        self.analyze_button.setEnabled(True)
        self._set_record_button(False)
        self.record_stats_label.setText(
            f"saved {path.name} - {recording.rows_written:,} rows in {format_duration(elapsed)}"
        )
        self._refresh_annotations()
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

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._autoconnect_timer.stop()
        self._save_settings()
        if self.recording is not None:
            self.stop_recording()
        if self.connected:
            self._disconnect()
        for monitor in list(self.monitors):
            monitor.close()
        self._ui_timer.stop()
        event.accept()


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


def main() -> int:
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    app = QApplication.instance() or QApplication(sys.argv)
    install_wheel_guard(app)
    window = LiveRecorderWindow()
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())