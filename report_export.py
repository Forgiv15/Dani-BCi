"""Shared reporting, export selection and measurement naming.

Both windows use this module:

* ``MeasurementInfo`` describes *what* was measured and which external stimulus
  was applied (kind, connection, voltage, reference mode). It is written into the
  recording header so the analyzer can document it without asking again.
* ``ExportOptions`` is the list of report sections the user ticked last time.
* ``ReportWriter`` renders a real table layout (header band, grid lines,
  alternating rows, pagination, page footer) into a PDF.

Nothing here imports the analyzer or the live window, so both can use it.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional, Sequence

import pandas as pd
from PyQt6.QtCore import QEvent, QObject, QPointF, QRectF, Qt
from PyQt6.QtGui import (
    QColor,
    QFont,
    QFontMetricsF,
    QPageLayout,
    QPageSize,
    QPainter,
    QPdfWriter,
    QPixmap,
)
from PyQt6.QtWidgets import (
    QAbstractScrollArea,
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


SETTINGS_VERSION = 1

# -- measurement naming ------------------------------------------------------

MEASUREMENT_KINDS = (
    ("Noise (input shorted to GND)", "noise"),
    ("CMRR", "cmrr"),
    ("Crosstalk", "crosstalk"),
    ("Gain", "gain"),
)
MEASUREMENT_KIND_CODES = {value: value for _label, value in MEASUREMENT_KINDS}
MEASUREMENT_KIND_LABELS = {value: label for label, value in MEASUREMENT_KINDS}

CONNECTION_MAIN = (
    ("Electrode", "electrode"),
    ("Coax", "coax"),
    ("IC (internal)", "ic"),
)
CONNECTION_MAIN_CODES = {value: value for _, value in CONNECTION_MAIN}

COAX_SUBTYPES = (
    ("GND", "gnd"),
    ("GND with OP", "gnd-op"),
    ("BIAS", "bias"),
    ("ACTIVE", "active"),
)
COAX_SUBTYPE_CODES = {value: value for _, value in COAX_SUBTYPES}
COAX_SUBTYPE_LABELS = {value: label for label, value in COAX_SUBTYPES}

REFERENCE_MODE_CHOICES = (
    ("True differential", "diff"),
    ("SRB1", "srb1"),
    ("SRB2", "srb2"),
)
REFERENCE_MODE_CODES = {value: value for _, value in REFERENCE_MODE_CHOICES}
REFERENCE_LABELS = {value: label for label, value in REFERENCE_MODE_CHOICES}
# Compact labels for combo boxes, so a long description cannot widen a narrow
# side panel. The full text stays available as a tooltip and in reports.
REFERENCE_MODE_SHORT = (
    ("True differential", "diff"),
    ("SRB1", "srb1"),
    ("SRB2", "srb2"),
)

# Kinds that are driven by an external stimulus voltage.
STIMULUS_KINDS = ("cmrr", "crosstalk", "gain")


def _slug(text: str) -> str:
    return re.sub(r"[^0-9A-Za-z._+-]+", "-", str(text or "").strip()).strip("-.")


@dataclass(slots=True)
class MeasurementInfo:
    """What was measured and which external input was applied."""

    kind: str = "noise"
    connection: str = "ic"
    coax_subtype: str = ""
    stimulus_v: float = 0.0
    stimulus_note: str = ""
    reference_mode: str = "srb2"
    note: str = ""
    operator_note: str = ""

    def normalised(self) -> "MeasurementInfo":
        kind = self.kind if self.kind in MEASUREMENT_KIND_CODES else "noise"
        connection = self.connection if self.connection in CONNECTION_MAIN_CODES else "electrode"
        subtype = self.coax_subtype if self.coax_subtype in COAX_SUBTYPE_CODES else ""
        if connection != "coax":
            subtype = ""
        reference = self.reference_mode if self.reference_mode in REFERENCE_MODE_CODES else "srb2"
        stimulus = float(self.stimulus_v) if kind in STIMULUS_KINDS else 0.0
        return MeasurementInfo(
            kind=kind,
            connection=connection,
            coax_subtype=subtype,
            stimulus_v=stimulus,
            stimulus_note=self.stimulus_note,
            reference_mode=reference,
            note=self.note,
            operator_note=self.operator_note,
        )

    # -- human readable forms ---------------------------------------------

    @property
    def kind_label(self) -> str:
        return MEASUREMENT_KIND_LABELS.get(self.normalised().kind, self.kind)

    @property
    def connection_label(self) -> str:
        info = self.normalised()
        if info.connection == "coax":
            subtype = COAX_SUBTYPE_LABELS.get(info.coax_subtype, info.coax_subtype or "GND")
            return f"Coax ({subtype})"
        if info.connection == "ic":
            return "IC (internal)"
        return "Electrode"

    @property
    def reference_label(self) -> str:
        return REFERENCE_LABELS.get(self.normalised().reference_mode, "SRB2")

    @property
    def stimulus_label(self) -> str:
        info = self.normalised()
        if info.kind not in STIMULUS_KINDS:
            return "-"
        return format_voltage(info.stimulus_v)

    def slug(self) -> str:
        """Filesystem-safe fragment describing the measurement."""

        info = self.normalised()
        parts = [info.kind]
        if info.connection == "coax":
            parts.append(f"coax-{info.coax_subtype or 'gnd'}")
        else:
            parts.append(info.connection)
        if info.kind in STIMULUS_KINDS:
            parts.append(_slug(format_voltage(info.stimulus_v).replace(" ", "")))
        parts.append(f"ref-{info.reference_mode}")
        return "_".join(_slug(part) for part in parts if part)

    def describe_lines(self) -> list[str]:
        """Multi-line description used in reports."""

        info = self.normalised()
        lines = [
            f"Measurement: {info.kind_label}",
            f"Connection: {info.connection_label}",
            f"Reference mode: {info.reference_label}",
        ]
        if info.kind in STIMULUS_KINDS:
            lines.append(f"Applied stimulus: {format_voltage(info.stimulus_v)}")
        if info.stimulus_note:
            lines.append(f"Stimulus detail: {info.stimulus_note}")
        if info.note:
            lines.append(f"Note: {info.note}")
        if info.operator_note:
            lines.append(f"Operator: {info.operator_note}")
        return lines

    def metadata(self) -> dict[str, str]:
        """Key/value pairs written into the CSV header block."""

        info = self.normalised()
        payload = {
            "Measurement": info.kind_label,
            "Measurement Kind": info.kind,
            "Connection": info.connection_label,
            "Reference Mode": info.reference_label,
        }
        if info.connection == "coax":
            payload["Connection Type"] = "coax"
            payload["Coax Subtype"] = COAX_SUBTYPE_LABELS.get(info.coax_subtype, info.coax_subtype or "GND")
        else:
            payload["Connection Type"] = info.connection
        if info.kind in STIMULUS_KINDS:
            payload["Applied Stimulus"] = format_voltage(info.stimulus_v)
            payload["Applied Stimulus (V)"] = f"{info.stimulus_v:.9g}"
        if info.stimulus_note:
            payload["Stimulus Source"] = info.stimulus_note
        if info.note:
            payload["Measurement Note"] = info.note
        return payload

    def as_dict(self) -> dict[str, object]:
        info = self.normalised()
        return {
            "kind": info.kind,
            "connection": info.connection,
            "coax_subtype": info.coax_subtype,
            "stimulus_v": info.stimulus_v,
            "stimulus_note": info.stimulus_note,
            "reference_mode": info.reference_mode,
            "note": info.note,
            "operator_note": info.operator_note,
        }

    @classmethod
    def from_dict(cls, payload: object) -> "MeasurementInfo":
        if not isinstance(payload, dict):
            return cls()
        defaults = cls()

        def text(key: str) -> str:
            value = payload.get(key)
            return str(value) if value is not None else getattr(defaults, key)

        try:
            stimulus = float(payload.get("stimulus_v", 0.0) or 0.0)
        except (TypeError, ValueError):
            stimulus = 0.0
        return cls(
            kind=text("kind"),
            connection=text("connection"),
            coax_subtype=text("coax_subtype"),
            stimulus_v=stimulus,
            stimulus_note=text("stimulus_note"),
            reference_mode=text("reference_mode"),
            note=text("note"),
            operator_note=text("operator_note"),
        ).normalised()

    @classmethod
    def from_metadata(cls, metadata: dict[str, str]) -> "MeasurementInfo":
        """Rebuild the description from a recording header."""

        info = cls()
        kind = (metadata.get("Measurement Kind") or "").strip()
        if kind in MEASUREMENT_KIND_CODES:
            info.kind = kind
        else:
            label = (metadata.get("Measurement") or "").strip().lower()
            for label_text, code in MEASUREMENT_KINDS:
                if label_text.lower().split(" (")[0] == label:
                    info.kind = code
                    break
        connection = (metadata.get("Connection Type") or "").strip()
        if connection in CONNECTION_MAIN_CODES:
            info.connection = connection
        subtype_raw = (metadata.get("Coax Subtype") or "").strip()
        subtype_key = subtype_raw.lower().replace(" ", "-")
        if subtype_key in COAX_SUBTYPE_CODES:
            info.coax_subtype = subtype_key
        else:
            for label_text, code in COAX_SUBTYPES:
                if label_text.lower() == subtype_raw.lower():
                    info.coax_subtype = code
                    break
        reference = (metadata.get("Reference Mode") or "").strip().lower()
        for label_text, code in REFERENCE_MODE_CHOICES:
            if label_text.lower() == reference or code == reference:
                info.reference_mode = code
                break
        for key in ("Applied Stimulus (V)", "Applied Stimulus"):
            value = (metadata.get(key) or "").strip()
            if value:
                parsed = parse_voltage(value)
                if parsed is not None:
                    info.stimulus_v = parsed
                    break
        info.stimulus_note = (metadata.get("Stimulus Source") or "").strip()
        info.note = (metadata.get("Measurement Note") or "").strip()
        return info.normalised()


_VOLTAGE_RE = re.compile(r"^\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*([munp]?)\s*V?\s*$")


def parse_voltage(text: str) -> Optional[float]:
    """Parse ``1``, ``1V``, ``1 mV``, ``500uV`` into volts."""

    match = _VOLTAGE_RE.match(str(text or ""))
    if not match:
        return None
    try:
        value = float(match.group(1))
    except ValueError:
        return None
    prefix = match.group(2)
    scale = {"": 1.0, "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12}.get(prefix, 1.0)
    return value * scale


def format_voltage(volts: float) -> str:
    """Pick a readable unit for a stimulus voltage."""

    value = float(volts or 0.0)
    magnitude = abs(value)
    if magnitude == 0.0:
        return "0 V"
    if magnitude >= 1.0:
        return f"{value:g} V"
    if magnitude >= 1e-3:
        return f"{value * 1e3:g} mV"
    if magnitude >= 1e-6:
        return f"{value * 1e6:g} uV"
    return f"{value * 1e9:g} nV"


def full_export_stem(stem: str, info: MeasurementInfo) -> str:
    """Add the measurement slug to an existing base name once."""

    slug = info.slug()
    if not slug or slug in stem:
        return stem
    return f"{stem}_{slug}"


# -- export selection --------------------------------------------------------

# (key, label, description). Order is the order used in the dialog and report.
EXPORT_SECTIONS = (
    ("info", "Measurement & settings", "What was measured, the applied stimulus and the analysis settings"),
    ("waveform", "Waveform plot", "Time-domain traces of the selected region"),
    ("fft", "FFT plot", "RMS spectrum of the analysed channel"),
    ("metrics", "Metrics table", "Per-channel noise, SNR, THD, ENOB and CMRR"),
    ("crosstalk", "Crosstalk table", "Crosstalk from the selected source channel"),
    ("crosstalk_sweep", "Crosstalk sweep", "Crosstalk versus frequency"),
    ("gain_sweep", "Gain sweep", "Response versus applied stimulus level"),
    ("cmrr_sweep", "CMRR sweep", "CMRR versus frequency"),
)
EXPORT_SECTION_KEYS = tuple(key for key, _label, _desc in EXPORT_SECTIONS)
DEFAULT_EXPORT_SECTIONS = ("info", "waveform", "fft", "metrics")


@dataclass(slots=True)
class ExportOptions:
    """Which report sections and formats to produce (persisted)."""

    sections: list[str] = field(default_factory=lambda: list(DEFAULT_EXPORT_SECTIONS))
    pdf: bool = True
    png: bool = False
    csv: bool = False
    measurement: MeasurementInfo = field(default_factory=MeasurementInfo)
    output_dir: str = ""
    last_stem: str = ""

    def enabled(self, key: str) -> bool:
        return key in self.sections

    def normalised(self) -> "ExportOptions":
        sections = [key for key in EXPORT_SECTION_KEYS if key in set(self.sections)]
        if not sections:
            sections = list(DEFAULT_EXPORT_SECTIONS)
        return ExportOptions(
            sections=sections,
            pdf=bool(self.pdf),
            png=bool(self.png),
            csv=bool(self.csv),
            measurement=self.measurement.normalised(),
            output_dir=str(self.output_dir or ""),
            last_stem=str(self.last_stem or ""),
        )

    def as_dict(self) -> dict[str, object]:
        options = self.normalised()
        return {
            "sections": list(options.sections),
            "formats": {"pdf": options.pdf, "png": options.png, "csv": options.csv},
            "measurement": options.measurement.as_dict(),
            "output_dir": options.output_dir,
            "last_stem": options.last_stem,
        }

    @classmethod
    def from_dict(cls, payload: object) -> "ExportOptions":
        defaults = cls()
        if not isinstance(payload, dict):
            return defaults
        sections = payload.get("sections")
        formats = payload.get("formats") if isinstance(payload.get("formats"), dict) else {}
        options = cls(
            sections=[str(item) for item in sections] if isinstance(sections, list) else list(defaults.sections),
            pdf=bool(formats.get("pdf", defaults.pdf)),
            png=bool(formats.get("png", defaults.png)),
            csv=bool(formats.get("csv", defaults.csv)),
            measurement=MeasurementInfo.from_dict(payload.get("measurement")),
            output_dir=str(payload.get("output_dir", "") or ""),
            last_stem=str(payload.get("last_stem", "") or ""),
        )
        return options.normalised()


class ReportSettingsStore:
    """JSON store for the analyzer export preferences."""

    def __init__(self, path: Optional[Path] = None) -> None:
        if path is None:
            path = Path(__file__).resolve().with_name("settings") / "analyzer.json"
        self.path = Path(path)

    def load(self) -> ExportOptions:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return ExportOptions().normalised()
        if not isinstance(payload, dict):
            return ExportOptions().normalised()
        return ExportOptions.from_dict(payload.get("export", payload))

    def save(self, options: ExportOptions) -> None:
        payload: dict[str, object] = {"version": SETTINGS_VERSION, "export": options.as_dict()}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(self.path)


# -- PDF rendering -----------------------------------------------------------

class ReportWriter:
    """Paginated PDF writer with real tables.

    Tables get a header band, grid lines, alternating row shading and per-column
    alignment, and they continue across pages with the header repeated, instead
    of dumping ``DataFrame.to_string()`` onto the page.
    """

    # Geometry is expressed in points and multiplied by ``scale`` so the layout
    # stays identical at any output resolution.
    RESOLUTION = 200
    MARGIN_X_PT = 30.0
    MARGIN_TOP_PT = 26.0
    MARGIN_BOTTOM_PT = 30.0
    HEADER_BAND = QColor("#1e3a8a")
    HEADER_TEXT = QColor("#ffffff")
    ROW_ALT = QColor("#f1f5f9")
    GRID = QColor("#cbd5e1")
    TEXT = QColor("#0f172a")
    MUTED = QColor("#475569")

    def __init__(self, path: Path, title: str, subtitle: str = "") -> None:
        self.path = Path(path)
        self.title = title
        self.subtitle = subtitle
        self.writer = QPdfWriter(str(self.path))
        self.writer.setResolution(self.RESOLUTION)
        self.writer.setPageSize(QPageSize(QPageSize.PageSizeId.A4))
        self.writer.setPageOrientation(QPageLayout.Orientation.Landscape)
        self.writer.setTitle(title)
        self.writer.setCreator("Dani-BCi analyzer")
        self.painter = QPainter(self.writer)
        self.painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self.painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)
        self.scale = self.writer.resolution() / 72.0
        rect = self.writer.pageLayout().paintRectPixels(self.writer.resolution())
        self.page_width = float(rect.width())
        self.page_height = float(rect.height())
        self.margin_x = self.MARGIN_X_PT * self.scale
        self.margin_top = self.MARGIN_TOP_PT * self.scale
        self.margin_bottom = self.MARGIN_BOTTOM_PT * self.scale
        self.content_width = self.page_width - 2 * self.margin_x
        self.y = self.margin_top
        self.page = 0
        self._header_drawn = False
        self.new_page()

    # -- primitives --------------------------------------------------------

    def _font(self, size: float, bold: bool = False) -> QFont:
        font = QFont("Segoe UI")
        font.setPointSizeF(max(size, 5.0))
        font.setBold(bold)
        return font

    def new_page(self) -> None:
        if self.page:
            self.writer.newPage()
        self.page += 1
        self.y = self.margin_top
        self._header_drawn = False

    def ensure(self, height: float) -> None:
        if self.y + height > self.page_height - self.margin_bottom:
            self.new_page()
            self._page_header()

    def _page_header(self) -> None:
        if self._header_drawn:
            return
        self._header_drawn = True
        title_height = 16.0 * self.scale
        self.painter.setFont(self._font(14, bold=True))
        self.painter.setPen(self.TEXT)
        self.painter.drawText(
            QRectF(self.margin_x, self.y, self.content_width, title_height),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            self.title,
        )
        self.y += title_height
        if self.page == 1 and self.subtitle:
            subtitle_height = 11.0 * self.scale
            self.painter.setFont(self._font(8.5))
            self.painter.setPen(self.MUTED)
            self.painter.drawText(
                QRectF(self.margin_x, self.y, self.content_width, subtitle_height),
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                self.subtitle,
            )
            self.y += subtitle_height
        pen = self.painter.pen()
        pen.setColor(self.GRID)
        pen.setWidthF(1.0)
        self.painter.setPen(pen)
        line_y = self.y + 2 * self.scale
        self.painter.drawLine(QPointF(self.margin_x, line_y), QPointF(self.page_width - self.margin_x, line_y))
        self.y += 8.0 * self.scale

    def heading(self, text: str, level: int = 1, keep_with_next: float = 0.0) -> None:
        """Draw a heading.

        ``keep_with_next`` is a fraction of the page height that must remain
        below the heading, so a heading never ends up orphaned at the bottom of
        a page while its table or figure sits on the next one.
        """

        size = 11.5 if level == 1 else 9.5
        height = size * 2.0 * self.scale
        self.ensure(height + 4 * self.scale + keep_with_next * self.page_height)
        self._page_header()
        self.painter.setFont(self._font(size, bold=True))
        self.painter.setPen(QColor("#1e3a8a") if level == 1 else self.TEXT)
        self.painter.drawText(
            QRectF(self.margin_x, self.y, self.content_width, height),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            text,
        )
        self.y += height

    def paragraph(self, lines: Iterable[str], size: float = 9.0) -> None:
        line_height = size * 1.5 * self.scale
        for line in lines:
            text = str(line)
            while text:
                self.ensure(line_height)
                self._page_header()
                metrics = QFontMetricsF(self._font(size))
                fitted, remaining = self._split_to_width(text, metrics, self.content_width)
                self.painter.setFont(self._font(size))
                self.painter.setPen(self.TEXT)
                self.painter.drawText(
                    QRectF(self.margin_x, self.y, self.content_width, line_height),
                    int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                    fitted,
                )
                self.y += line_height
                text = remaining

    @staticmethod
    def _split_to_width(text: str, metrics: QFontMetricsF, width: float) -> tuple[str, str]:
        if metrics.horizontalAdvance(text) <= width:
            return text, ""
        words = text.split(" ")
        current = ""
        while words:
            candidate = current + (" " if current else "") + words[0]
            if metrics.horizontalAdvance(candidate) > width and current:
                break
            current = candidate
            words.pop(0)
        if not current:
            current = text[: max(1, len(text) // 2)]
            return current, text[len(current):]
        return current, " ".join(words)

    def image(self, pixmap: QPixmap, caption: str = "", max_height: float = 0.34) -> None:
        if pixmap is None or pixmap.isNull():
            return
        caption_height = 9.0 * self.scale if caption else 0.0
        available = self.page_height - self.margin_top - self.margin_bottom - caption_height
        # A figure may use nearly the whole page: only the header band and the
        # caption are reserved, so a wide plot is not shrunk to half a page.
        max_h = min(self.page_height * max_height, available)
        ratio = min(self.content_width / max(pixmap.width(), 1), max_h / max(pixmap.height(), 1))
        target_w = max(pixmap.width() * ratio, 1.0)
        target_h = max(pixmap.height() * ratio, 1.0)
        self.ensure(target_h + caption_height + 4 * self.scale)
        self._page_header()
        x = self.margin_x + (self.content_width - target_w) / 2.0
        self.painter.drawPixmap(
            QRectF(x, self.y, target_w, target_h), pixmap, QRectF(0, 0, pixmap.width(), pixmap.height())
        )
        self.y += target_h + 3 * self.scale
        if caption:
            self.painter.setFont(self._font(8))
            self.painter.setPen(self.MUTED)
            self.painter.drawText(
                QRectF(self.margin_x, self.y, self.content_width, caption_height),
                int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
                caption,
            )
            self.y += caption_height
        self.y += 6 * self.scale

    MAX_DATA_COLUMNS = 6

    def table(
        self,
        dataframe: pd.DataFrame,
        columns: Optional[Sequence[str]] = None,
        font_size: float = 7.2,
        max_rows: Optional[int] = None,
        caption: str = "",
        max_data_columns: Optional[int] = None,
    ) -> None:
        """Draw a dataframe as one or more real tables.

        Wide frames are split into groups of ``max_data_columns`` after the first
        column, which is repeated in every group, so no header or value is ever
        clipped on the page.
        """

        if dataframe is None or dataframe.empty:
            return
        frame = dataframe
        if columns is not None:
            present = [column for column in columns if column in dataframe.columns]
            frame = dataframe[present] if present else dataframe
        if max_rows is not None and len(frame.index) > max_rows:
            frame = frame.iloc[:max_rows]
        if not len(frame.columns):
            return

        limit = max_data_columns or self.MAX_DATA_COLUMNS
        first = list(frame.columns[:1])
        rest = list(frame.columns[1:])
        if len(rest) <= limit:
            self._draw_table(frame, font_size, caption)
            return

        chunks = [rest[index:index + limit] for index in range(0, len(rest), limit)]
        for number, chunk in enumerate(chunks, start=1):
            label = f"{caption} ({number}/{len(chunks)})" if caption else f"Part {number}/{len(chunks)}"
            self._draw_table(frame[first + chunk], font_size, label)

    def _draw_table(self, frame: pd.DataFrame, font_size: float, caption: str = "") -> None:
        """Draw one table block."""

        if frame.empty:
            return
        headers = [str(column) for column in frame.columns]
        if not headers:
            return
        if caption:
            caption_height = font_size * 1.7 * self.scale
            self.ensure(caption_height)
            self._page_header()
            self.painter.setFont(self._font(font_size + 0.6, bold=True))
            self.painter.setPen(self.MUTED)
            self.painter.drawText(
                QRectF(self.margin_x, self.y, self.content_width, caption_height),
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                caption,
            )
            self.y += caption_height

        metrics_font = self._font(font_size)
        metrics_header = self._font(font_size, bold=True)
        cell_metrics = QFontMetricsF(metrics_font)
        header_metrics = QFontMetricsF(metrics_header)

        padding = 6.0 * self.scale
        raw_widths: list[float] = []
        for index, header in enumerate(headers):
            widest = header_metrics.horizontalAdvance(self._shorten(header, 42))
            sample = frame.iloc[: min(len(frame.index), 400), index]
            for value in sample:
                widest = max(widest, cell_metrics.horizontalAdvance(self._cell(value)))
            raw_widths.append(widest + 2 * padding)
        total = sum(raw_widths) or 1.0
        floor_width = 22.0 * self.scale
        widths = [max(raw * self.content_width / total, floor_width) for raw in raw_widths]
        # Never let one column eat the page: cap at 40% and renormalise.
        cap = self.content_width * 0.40
        widths = [min(width, cap) for width in widths]
        rescale = self.content_width / sum(widths)
        widths = [width * rescale for width in widths]

        row_height = font_size * 1.75 * self.scale
        header_height = row_height + 2 * self.scale
        left = self.margin_x
        right = self.margin_x + self.content_width

        def draw_header_band() -> None:
            self.ensure(header_height + row_height)
            self._page_header()
            self.painter.setPen(Qt.PenStyle.NoPen)
            self.painter.setBrush(self.HEADER_BAND)
            self.painter.drawRect(QRectF(left, self.y, self.content_width, header_height))
            self.painter.setBrush(Qt.BrushStyle.NoBrush)
            self.painter.setFont(metrics_header)
            self.painter.setPen(self.HEADER_TEXT)
            x = left
            for header, width in zip(headers, widths):
                self.painter.drawText(
                    QRectF(x + padding / 2, self.y, width - padding, header_height),
                    int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                    self._shorten(header, 42),
                )
                x += width
            self.y += header_height

        def draw_row_lines(top: float, height: float) -> None:
            grid_pen = self.painter.pen()
            grid_pen.setColor(self.GRID)
            grid_pen.setWidthF(0.5)
            self.painter.setPen(grid_pen)
            self.painter.setBrush(Qt.BrushStyle.NoBrush)
            self.painter.drawLine(QPointF(left, top + height), QPointF(right, top + height))
            x = left
            for width in widths[:-1]:
                x += width
                self.painter.drawLine(QPointF(x, top), QPointF(x, top + height))

        draw_header_band()
        block_top = self.y - header_height
        for position, (_, row) in enumerate(frame.iterrows()):
            if self.y + row_height > self.page_height - self.margin_bottom:
                self._close_table_block(block_top, headers, widths)
                self.new_page()
                draw_header_band()
                block_top = self.y - header_height
            top = self.y
            if position % 2 == 1:
                self.painter.setPen(Qt.PenStyle.NoPen)
                self.painter.setBrush(self.ROW_ALT)
                self.painter.drawRect(QRectF(left, top, self.content_width, row_height))
                self.painter.setBrush(Qt.BrushStyle.NoBrush)
            self.painter.setFont(metrics_font)
            self.painter.setPen(self.TEXT)
            x = left
            for value, width in zip(row, widths):
                text = self._cell(value)
                alignment = (
                    Qt.AlignmentFlag.AlignRight if self._is_number(value) else Qt.AlignmentFlag.AlignLeft
                )
                self.painter.drawText(
                    QRectF(x + padding / 2, top, width - padding, row_height),
                    int(alignment | Qt.AlignmentFlag.AlignVCenter),
                    self._shorten(text, 42),
                )
                x += width
            draw_row_lines(top, row_height)
            self.y = top + row_height
        self.painter.setBrush(Qt.BrushStyle.NoBrush)
        self._close_table_block(block_top, headers, widths)

    def _close_table_block(self, top: float, headers: Sequence[str], widths: Sequence[float]) -> None:
        """Draw the outer box around one page's share of a table."""

        left = self.margin_x
        box_pen = self.painter.pen()
        box_pen.setColor(self.GRID)
        box_pen.setWidthF(0.9)
        self.painter.setPen(box_pen)
        self.painter.setBrush(Qt.BrushStyle.NoBrush)
        self.painter.drawRect(QRectF(left, top, self.content_width, max(self.y - top, 1.0)))
        self.y += 8 * self.scale

    @staticmethod
    def _is_number(value: object) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool)

    @staticmethod
    def _cell(value: object) -> str:
        if value is None:
            return ""
        if isinstance(value, bool):
            return "yes" if value else "no"
        if isinstance(value, (int, float)):
            if isinstance(value, float) and pd.isna(value):
                return ""
            if value != 0 and (abs(value) >= 1e5 or abs(value) < 1e-4):
                return f"{value:.4g}"
            return f"{value:.4f}"
        try:
            if pd.isna(value):
                return ""
        except (TypeError, ValueError):
            pass
        return str(value)

    @staticmethod
    def _shorten(text: str, limit: int) -> str:
        return text if len(text) <= limit else text[: limit - 1] + "\u2026"

    def _footer(self) -> None:
        self.painter.setFont(self._font(7.5))
        self.painter.setPen(self.MUTED)
        self.painter.drawText(
            QRectF(
                self.margin_x,
                self.page_height - self.margin_bottom + 3 * self.scale,
                self.content_width,
                10 * self.scale,
            ),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            datetime.now().strftime("Dani-BCi report - generated %Y-%m-%d %H:%M"),
        )

    def close(self) -> Path:
        self._footer()
        self.painter.end()
        return self.path


# -- dialogs -----------------------------------------------------------------

class MeasurementNameDialog(QDialog):
    """Build the recording name from dropdowns instead of typing it.

    Every field maps to something the board or the bench setup actually is
    (measurement kind, connection, applied stimulus, reference mode), so the
    name documents itself and the analyzer can read the same facts back.
    """

    def __init__(self, info: Optional[MeasurementInfo] = None, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Name this measurement")
        self.setMinimumWidth(520)
        self._info = (info or MeasurementInfo()).normalised()

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        intro = QLabel(
            "Pick what you are measuring. The generated name becomes the recording name, is stored in the "
            "CSV header and is reused automatically when you export from the analyzer."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form = QFormLayout()
        form.setSpacing(8)
        self.kind_combo = QComboBox()
        for label, value in MEASUREMENT_KINDS:
            self.kind_combo.addItem(label, value)
        self.kind_combo.setCurrentIndex(max(self.kind_combo.findData(self._info.kind), 0))
        form.addRow("Measurement", self.kind_combo)

        self.connection_combo = QComboBox()
        for label, value in CONNECTION_MAIN:
            self.connection_combo.addItem(label, value)
        self.connection_combo.setCurrentIndex(max(self.connection_combo.findData(self._info.connection), 0))
        form.addRow("Connection", self.connection_combo)

        self.subtype_combo = QComboBox()
        for label, value in COAX_SUBTYPES:
            self.subtype_combo.addItem(label, value)
        self.subtype_combo.setCurrentIndex(max(self.subtype_combo.findData(self._info.coax_subtype), 0))
        self.subtype_label = QLabel("Coax type")
        form.addRow(self.subtype_label, self.subtype_combo)

        self.stimulus_spin = QDoubleSpinBox()
        self.stimulus_spin.setDecimals(6)
        self.stimulus_spin.setRange(0.0, 1000.0)
        self.stimulus_spin.setValue(max(self._info.stimulus_v, 0.0))
        self.stimulus_spin.setSuffix(" V")
        self.stimulus_spin.setSingleStep(0.1)
        self.stimulus_spin.setToolTip("Voltage you feed in externally (CMRR / crosstalk / gain)")
        self.stimulus_label = QLabel("Applied voltage")
        form.addRow(self.stimulus_label, self.stimulus_spin)

        self.stimulus_note_edit = QLineEdit(self._info.stimulus_note)
        self.stimulus_note_edit.setPlaceholderText("where the voltage comes from, e.g. function generator 50 Ohm")
        self.stimulus_note_label = QLabel("Source")
        form.addRow(self.stimulus_note_label, self.stimulus_note_edit)

        self.reference_combo = QComboBox()
        for label, value in REFERENCE_MODE_CHOICES:
            self.reference_combo.addItem(label, value)
        self.reference_combo.setCurrentIndex(max(self.reference_combo.findData(self._info.reference_mode), 0))
        form.addRow("Reference", self.reference_combo)

        self.note_edit = QLineEdit(self._info.note)
        self.note_edit.setPlaceholderText("optional free text")
        form.addRow("Note", self.note_edit)
        layout.addLayout(form)

        preview_group = QGroupBox("Result")
        preview_layout = QVBoxLayout(preview_group)
        self.preview_label = QLabel("")
        self.preview_label.setWordWrap(True)
        self.preview_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.preview_label.setStyleSheet("font-family: Consolas; font-size: 11px;")
        preview_layout.addWidget(self.preview_label)
        layout.addWidget(preview_group)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.kind_combo.currentIndexChanged.connect(self._refresh)
        self.connection_combo.currentIndexChanged.connect(self._refresh)
        self.subtype_combo.currentIndexChanged.connect(self._refresh)
        self.stimulus_spin.valueChanged.connect(self._refresh)
        self.reference_combo.currentIndexChanged.connect(self._refresh)
        self._refresh()

    def _refresh(self) -> None:
        kind = str(self.kind_combo.currentData() or "noise")
        coax = str(self.connection_combo.currentData() or "electrode") == "coax"
        self.subtype_combo.setVisible(coax)
        self.subtype_label.setVisible(coax)
        stimulus = kind in STIMULUS_KINDS
        for widget in (self.stimulus_spin, self.stimulus_label, self.stimulus_note_edit, self.stimulus_note_label):
            widget.setVisible(stimulus)
        info = self.measurement()
        self.preview_label.setText(f"{self.name_stem()}\n\n" + "\n".join(info.describe_lines()))

    def measurement(self) -> MeasurementInfo:
        return MeasurementInfo(
            kind=str(self.kind_combo.currentData() or "noise"),
            connection=str(self.connection_combo.currentData() or "electrode"),
            coax_subtype=str(self.subtype_combo.currentData() or "gnd"),
            stimulus_v=float(self.stimulus_spin.value()),
            stimulus_note=self.stimulus_note_edit.text().strip(),
            reference_mode=str(self.reference_combo.currentData() or "srb2"),
            note=self.note_edit.text().strip(),
            operator_note=self._info.operator_note,
        ).normalised()

    def name_stem(self) -> str:
        return self.measurement().slug()


class ExportSelectionDialog(QDialog):
    """Checklist of report sections plus the measurement facts to document."""

    def __init__(
        self,
        options: ExportOptions,
        parent: Optional[QWidget] = None,
        available: Optional[Iterable[str]] = None,
        focus_format: str = "pdf",
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Export report")
        self.setMinimumWidth(620)
        self._options = options.normalised()
        available_keys = set(available) if available is not None else set(EXPORT_SECTION_KEYS)

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        formats_group = QGroupBox("Format")
        formats_layout = QHBoxLayout(formats_group)
        self.pdf_check = QCheckBox("PDF report")
        self.png_check = QCheckBox("PNG images")
        self.csv_check = QCheckBox("CSV tables")
        self.pdf_check.setChecked(self._options.pdf)
        self.png_check.setChecked(self._options.png)
        self.csv_check.setChecked(self._options.csv)
        for widget in (self.pdf_check, self.png_check, self.csv_check):
            formats_layout.addWidget(widget)
        formats_layout.addStretch(1)
        layout.addWidget(formats_group)
        focus = focus_format.lower()
        if focus == "png":
            self.png_check.setChecked(True)
        elif focus == "csv":
            self.csv_check.setChecked(True)
        elif focus == "pdf":
            self.pdf_check.setChecked(True)

        content_group = QGroupBox("Include")
        content_layout = QGridLayout(content_group)
        content_layout.setVerticalSpacing(4)
        self.section_checks: dict[str, QCheckBox] = {}
        row = 0
        for key, label, description in EXPORT_SECTIONS:
            checkbox = QCheckBox(label)
            checkbox.setChecked(self._options.enabled(key) and key in available_keys)
            if key not in available_keys:
                checkbox.setChecked(False)
                checkbox.setEnabled(False)
                checkbox.setToolTip("Not available until the matching sweep has been computed")
            else:
                checkbox.setToolTip(description)
            hint = QLabel(description)
            hint.setStyleSheet("color: #64748b; font-size: 11px;")
            content_layout.addWidget(checkbox, row, 0)
            content_layout.addWidget(hint, row, 1)
            self.section_checks[key] = checkbox
            row += 1
        content_layout.setColumnStretch(1, 1)
        layout.addWidget(content_group)

        info_group = QGroupBox("Measurement (written into the report and the file name)")
        info_layout = QVBoxLayout(info_group)
        self.info_summary = QLabel("")
        self.info_summary.setWordWrap(True)
        self.info_summary.setStyleSheet("font-size: 11px;")
        info_layout.addWidget(self.info_summary)
        self.edit_measurement_button = QPushButton("Edit measurement details...")
        self.edit_measurement_button.clicked.connect(self._edit_measurement)
        info_layout.addWidget(self.edit_measurement_button)
        layout.addWidget(info_group)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("File name"))
        self.name_edit = QLineEdit(self._options.last_stem)
        self.name_edit.setPlaceholderText("base name for the exported files")
        name_row.addWidget(self.name_edit, 1)
        layout.addLayout(name_row)
        self._refresh_info()

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _edit_measurement(self) -> None:
        dialog = MeasurementNameDialog(self._options.measurement, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._options.measurement = dialog.measurement()
            self._refresh_info()
            if not self.name_edit.text().strip():
                self.name_edit.setText(dialog.name_stem())

    def _refresh_info(self) -> None:
        info = self._options.measurement
        self.info_summary.setText("  |  ".join(info.describe_lines()))

    def options(self) -> ExportOptions:
        sections = [key for key, checkbox in self.section_checks.items() if checkbox.isChecked()]
        return ExportOptions(
            sections=sections,
            pdf=self.pdf_check.isChecked(),
            png=self.png_check.isChecked(),
            csv=self.csv_check.isChecked(),
            measurement=self._options.measurement.normalised(),
            output_dir=self._options.output_dir,
            last_stem=self.name_edit.text().strip(),
        ).normalised()


class WheelGuard(QObject):
    """Stop the mouse wheel from editing values by accident.

    Qt changes a combo box or spin box on a wheel event even when it has no
    focus, so scrolling a long settings panel silently mutates whatever the
    pointer happens to be over. With this filter installed on the application,
    an unfocused value widget lets the wheel fall through to the enclosing
    scroll area instead; a click (which focuses it) restores normal wheel
    behaviour.
    """

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._value_types = (QComboBox, QAbstractSpinBox)

    def eventFilter(self, obj, event):  # noqa: N802 - Qt naming
        if event.type() != QEvent.Type.Wheel or not isinstance(obj, self._value_types):
            return False
        if obj.hasFocus():
            return False
        area = obj.parentWidget()
        while area is not None and not isinstance(area, QAbstractScrollArea):
            area = area.parentWidget()
        if area is not None:
            # Forward to the viewport so the panel scrolls exactly as if the
            # pointer had been over empty space.
            QApplication.sendEvent(area.viewport(), event)
        return True


def install_wheel_guard(application: QApplication) -> WheelGuard:
    """Install (once) the wheel guard and return it so the caller keeps a ref."""

    existing = getattr(application, "_dani_bci_wheel_guard", None)
    if existing is not None:
        return existing
    guard = WheelGuard()
    application.installEventFilter(guard)

    def _detach() -> None:
        # A Python-defined event filter still installed when the interpreter
        # finalises the application tears down in the wrong order and takes the
        # whole process down, so remove it while the application still exists.
        application.removeEventFilter(guard)
        setattr(application, "_dani_bci_wheel_guard", None)

    application.aboutToQuit.connect(_detach)
    setattr(application, "_dani_bci_wheel_guard", guard)
    return guard
