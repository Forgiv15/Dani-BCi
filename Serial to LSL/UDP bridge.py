#!/usr/bin/env python3

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
import serial
from serial.tools import list_ports
import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


APP_TITLE = "ESP32 ADS1299 Debug Tool"
DEFAULT_BAUD = 115200
CHANNEL_COUNT = 8
VREF = 4.5
ADC_SCALE = float((1 << 23) - 1)
BUFFER_SECONDS = 8
PLOT_SECONDS = 5
FFT_MAX_HZ = 100.0
PLOT_REFRESH_MS = 20
DEFAULT_TRACE_SCALE_UV = 100.0
TRACE_SCALE_OPTIONS_UV = [25, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]

STATUS_MESSAGES = {
    "#OK start": "Board accepted start streaming.",
    "#OK stop": "Board accepted stop streaming.",
    "#OK config": "Board accepted the channel settings update.",
    "#ERR config": "Board rejected the channel settings update.",
    "#OK imp": "Board accepted the impedance setting update.",
    "#ERR imp": "Board rejected the impedance setting update.",
    "#OK rate": "Board accepted the sample-rate change.",
    "#ERR rate": "Board rejected the sample-rate change.",
    "#OK defaults": "Board restored default settings.",
    "#ERR defaults": "Board failed to restore default settings.",
}

GAIN_LABELS = ["1x", "2x", "4x", "6x", "8x", "12x", "24x"]
GAIN_SCALARS = [1, 2, 4, 6, 8, 12, 24]
INPUT_TYPES = [
    "Normal",
    "Shorted",
    "Bias Meas",
    "MVDD",
    "Temp",
    "Test Sig",
    "Bias DRP",
    "Bias DRN",
]
TEST_MODES = [
    ("Normal", "normal"),
    ("Ground", "ground"),
    ("Pulse 1x Slow", "pulse1slow"),
    ("Pulse 1x Fast", "pulse1fast"),
    ("Pulse 2x Slow", "pulse2slow"),
    ("Pulse 2x Fast", "pulse2fast"),
    ("DC", "dc"),
]
SAMPLE_RATE_CHOICES = [
    (16000, 0),
    (8000, 1),
    (4000, 2),
    (2000, 3),
    (1000, 4),
    (500, 5),
    (250, 6),
]
BOARD_MODES = [
    ("Default", 0),
    ("Debug", 1),
    ("Analog", 2),
    ("Digital", 3),
    ("Marker", 4),
]

BOARD_MODE_DESCRIPTIONS = {
    "Default": "Normal EEG packet streaming mode. Use this for standard ADS1299 debugging with sample packets.",
    "Debug": "Firmware debug board mode flag. Keeps the debug side-band protocol active for this app.",
    "Analog": "Reserved OpenBCI-style board mode. Not used by this ESP32 hardware path right now.",
    "Digital": "Reserved OpenBCI-style board mode. Not used by this ESP32 hardware path right now.",
    "Marker": "Reserved marker mode. Present for protocol compatibility, not a separate acquisition path here.",
}

CONTROL_HELP_TEXT = (
    "Refresh Board State: ask the ESP32 for its current streaming, rate, channel, SRB, and impedance settings.\n"
    "Restore Defaults: apply Cyton-like defaults to all 8 channels.\n"
    "Input Mux: selects the ADS1299 MUX[2:0] source for that channel.\n"
    "SRB2: per-channel SRB2 switch. SRB1: shared common negative bus on each ADS1299.\n"
    "Imp P / Imp N: enable lead-off impedance excitation on the channel P or N electrode path.\n"
    "Apply Settings: send that row's channel configuration to the ESP32. Apply Impedance: send only the impedance toggles."
)


def decode_i24(msb: int, mid: int, lsb: int) -> int:
    value = (msb << 16) | (mid << 8) | lsb
    if value & 0x800000:
        value -= 0x1000000
    return value


def counts_to_uv(counts: int, gain_scalar: int) -> float:
    if gain_scalar <= 0:
        gain_scalar = 24
    return counts * (VREF / (gain_scalar * ADC_SCALE)) * 1e6


@dataclass
class Sample:
    index: int
    timestamp: float
    counts: list[int]
    microvolts: list[float]


class MixedSerialParser:
    def __init__(self) -> None:
        self.packet: bytearray | None = None
        self.text_mode = False
        self.text_buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[str, Any]]:
        events: list[tuple[str, Any]] = []

        for byte in data:
            if self.text_mode:
                self.text_buffer.append(byte)
                if byte == 0x0A:
                    line = self.text_buffer.decode("utf-8", errors="replace").strip()
                    self.text_buffer.clear()
                    self.text_mode = False
                    if line:
                        events.append(("line", line))
                continue

            if self.packet is not None:
                self.packet.append(byte)
                if len(self.packet) == 33:
                    packet = bytes(self.packet)
                    self.packet = None
                    if packet[0] == 0xA0 and packet[32] == 0xC0:
                        events.append(("packet", packet))
                continue

            if byte == 0x23:
                self.text_mode = True
                self.text_buffer = bytearray([byte])
            elif byte == 0xA0:
                self.packet = bytearray([byte])

        return events


class SerialDebugClient:
    def __init__(self) -> None:
        self.serial_port: serial.Serial | None = None
        self.port_name = ""
        self.baudrate = DEFAULT_BAUD
        self.event_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.command_queue: queue.Queue[bytes] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.parser = MixedSerialParser()

    @property
    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open

    def connect(self, port_name: str, baudrate: int = DEFAULT_BAUD) -> None:
        self.disconnect()
        self.serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.05)
        self.serial_port.timeout = 0.005
        self.port_name = port_name
        self.baudrate = baudrate
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.event_queue.put(("log", f"Connected to {port_name} @ {baudrate}"))

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None

    def send_debug_command(self, command: str) -> None:
        if not command.startswith(":"):
            command = ":" + command
        if not command.endswith("\n"):
            command += "\n"
        self.command_queue.put(command.encode("utf-8"))

    def _run(self) -> None:
        assert self.serial_port is not None

        while not self.stop_event.is_set():
            try:
                while True:
                    payload = self.command_queue.get_nowait()
                    self.serial_port.write(payload)
            except queue.Empty:
                pass
            except serial.SerialException as exc:
                self.event_queue.put(("error", f"Write error: {exc}"))
                break

            try:
                incoming = self.serial_port.read(4096)
            except serial.SerialException as exc:
                self.event_queue.put(("error", f"Read error: {exc}"))
                break

            if not incoming:
                continue

            for event in self.parser.feed(incoming):
                self.event_queue.put(event)

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass


class ChannelRow:
    def __init__(self, parent: ttk.Frame, channel_index: int, apply_callback, imp_callback) -> None:
        self.channel_index = channel_index
        self.apply_callback = apply_callback
        self.imp_callback = imp_callback

        row = channel_index + 2
        ttk.Label(parent, text=f"Ch {channel_index + 1}").grid(row=row, column=0, padx=4, pady=2, sticky="w")

        self.active_var = tk.BooleanVar(value=True)
        self.gain_var = tk.StringVar(value=GAIN_LABELS[-1])
        self.input_var = tk.StringVar(value=INPUT_TYPES[0])
        self.bias_var = tk.BooleanVar(value=True)
        self.srb2_var = tk.BooleanVar(value=True)
        self.srb1_var = tk.BooleanVar(value=False)
        self.imp_p_var = tk.BooleanVar(value=False)
        self.imp_n_var = tk.BooleanVar(value=False)
        self.dirty = False
        self._suspend_dirty_tracking = True
        self.state_var = tk.StringVar(value="Saved")

        ttk.Checkbutton(parent, variable=self.active_var).grid(row=row, column=1, padx=2)
        ttk.Combobox(parent, textvariable=self.gain_var, width=6, values=GAIN_LABELS, state="readonly").grid(row=row, column=2, padx=2)
        ttk.Combobox(parent, textvariable=self.input_var, width=10, values=INPUT_TYPES, state="readonly").grid(row=row, column=3, padx=2)
        ttk.Checkbutton(parent, variable=self.bias_var).grid(row=row, column=4, padx=2)
        ttk.Checkbutton(parent, variable=self.srb2_var).grid(row=row, column=5, padx=2)
        ttk.Checkbutton(parent, variable=self.srb1_var).grid(row=row, column=6, padx=2)
        ttk.Checkbutton(parent, variable=self.imp_p_var).grid(row=row, column=7, padx=2)
        ttk.Checkbutton(parent, variable=self.imp_n_var).grid(row=row, column=8, padx=2)
        tk.Label(parent, textvariable=self.state_var, width=10, anchor="w").grid(row=row, column=9, padx=2, sticky="w")
        ttk.Button(parent, text="Apply", command=self.apply, width=8).grid(row=row, column=10, padx=2)
        ttk.Button(parent, text="Set", command=self.apply_impedance, width=7).grid(row=row, column=11, padx=2)

        for variable in [
            self.active_var,
            self.gain_var,
            self.input_var,
            self.bias_var,
            self.srb2_var,
            self.srb1_var,
            self.imp_p_var,
            self.imp_n_var,
        ]:
            variable.trace_add("write", self._on_user_edit)

        self._suspend_dirty_tracking = False

    def _on_user_edit(self, *_args) -> None:
        if not self._suspend_dirty_tracking:
            self.dirty = True
            self.state_var.set("Unsaved")

    def apply(self) -> None:
        gain_index = GAIN_LABELS.index(self.gain_var.get())
        input_index = INPUT_TYPES.index(self.input_var.get())
        self.state_var.set("Sending...")
        self.apply_callback(
            self.channel_index,
            self.active_var.get(),
            gain_index,
            input_index,
            self.bias_var.get(),
            self.srb2_var.get(),
            self.srb1_var.get(),
        )

    def apply_impedance(self) -> None:
        self.imp_callback(self.channel_index, self.imp_p_var.get(), self.imp_n_var.get())

    def apply_state(self, state: dict[str, Any], force: bool = False) -> None:
        if self.dirty and not force:
            return
        self._suspend_dirty_tracking = True
        self.active_var.set(bool(state.get("active", 1)))
        self.gain_var.set(GAIN_LABELS[min(int(state.get("gain_ordinal", 6)), len(GAIN_LABELS) - 1)])
        self.input_var.set(INPUT_TYPES[min(int(state.get("input", 0)), len(INPUT_TYPES) - 1)])
        self.bias_var.set(bool(state.get("bias", 1)))
        self.srb2_var.set(bool(state.get("srb2", 1)))
        self.srb1_var.set(bool(state.get("srb1", 0)))
        self.imp_p_var.set(bool(state.get("imp_p", 0)))
        self.imp_n_var.set(bool(state.get("imp_n", 0)))
        self._suspend_dirty_tracking = False
        self.dirty = False
        self.state_var.set("Saved")

    def mark_clean(self) -> None:
        self.dirty = False
        self.state_var.set("Saved")

    def mark_failed(self) -> None:
        self.dirty = True
        self.state_var.set("Send failed")


class DebugApp:
    def __init__(self) -> None:
        self.client = SerialDebugClient()
        self.root = tk.Tk()
        self.root.title(APP_TITLE + " Controls")
        self.root.geometry("1260x780")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.data_window = tk.Toplevel(self.root)
        self.data_window.title(APP_TITLE + " Live Data")
        self.data_window.geometry("1280x820")
        self.data_window.protocol("WM_DELETE_WINDOW", self.on_close)

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.mode_var = tk.StringVar(value="Unknown")
        self.sample_rate_var = tk.StringVar(value="250 Hz")
        self.board_mode_var = tk.StringVar(value=BOARD_MODES[0][0])
        self.test_mode_var = tk.StringVar(value=TEST_MODES[0][0])
        self.board_mode_desc_var = tk.StringVar(value=BOARD_MODE_DESCRIPTIONS[BOARD_MODES[0][0]])
        self.trace_scale_var = tk.StringVar(value=f"{int(DEFAULT_TRACE_SCALE_UV)} uV")

        self.latest_state: dict[str, Any] = {}
        self.sample_rate_hz = 250
        self.packet_loss = 0
        self.packet_total = 0
        self.last_packet_index: int | None = None
        self.last_sample_time: float | None = None
        self.last_stats_time = time.time()
        self.samples_since_stats = 0
        self.current_sps = 0.0
        self.force_next_state_refresh = False
        self.pending_config_channel: int | None = None
        self.pending_impedance_channel: int | None = None

        self.timestamps: deque[float] = deque(maxlen=BUFFER_SECONDS * 1000)
        self.channel_buffers: list[deque[float]] = [deque(maxlen=BUFFER_SECONDS * 1000) for _ in range(CHANNEL_COUNT)]
        self.current_counts = [0] * CHANNEL_COUNT
        self.current_uv = [0.0] * CHANNEL_COUNT

        self.value_vars = [tk.StringVar(value="0 cnt / 0.00 uV") for _ in range(CHANNEL_COUNT)]
        self.stats_var = tk.StringVar(value="Samples: 0 | SPS: 0.0 | Lost: 0")

        self.channel_rows: list[ChannelRow] = []
        self.trace_lines = []

        self._build_controls()
        self._build_live_window()
        self.refresh_ports()
        self.root.after(50, self.poll_events)
        self.root.after(PLOT_REFRESH_MS, self.refresh_plots)

    def _build_controls(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Port").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left", padx=4)
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left", padx=4)
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=12)
        ttk.Label(top, text="Mode:").pack(side="left")
        ttk.Label(top, textvariable=self.mode_var).pack(side="left", padx=4)

        global_frame = ttk.LabelFrame(self.root, text="Acquisition", padding=8)
        global_frame.pack(fill="x", padx=8, pady=6)

        ttk.Button(global_frame, text="Start", command=lambda: self.send_command("start")).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(global_frame, text="Stop", command=lambda: self.send_command("stop")).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(global_frame, text="Refresh Board State", command=self.refresh_board_state).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(global_frame, text="Restore Defaults", command=self.restore_defaults).grid(row=0, column=3, padx=4, pady=4)

        ttk.Label(global_frame, text="Sample Rate").grid(row=0, column=4, padx=4, pady=4)
        ttk.Combobox(
            global_frame,
            textvariable=self.sample_rate_var,
            width=10,
            values=[f"{rate} Hz" for rate, _ in SAMPLE_RATE_CHOICES],
            state="readonly",
        ).grid(row=0, column=5, padx=4, pady=4)
        ttk.Button(global_frame, text="Apply Rate", command=self.apply_sample_rate).grid(row=0, column=6, padx=4, pady=4)

        ttk.Label(global_frame, text="Board Mode").grid(row=0, column=7, padx=4, pady=4)
        board_mode_combo = ttk.Combobox(
            global_frame,
            textvariable=self.board_mode_var,
            width=10,
            values=[name for name, _ in BOARD_MODES],
            state="readonly",
        )
        board_mode_combo.grid(row=0, column=8, padx=4, pady=4)
        board_mode_combo.bind("<<ComboboxSelected>>", lambda _event: self.update_board_mode_description())
        ttk.Button(global_frame, text="Apply Mode", command=self.apply_board_mode).grid(row=0, column=9, padx=4, pady=4)

        ttk.Label(global_frame, text="Global Test Mode").grid(row=1, column=0, padx=4, pady=4)
        ttk.Combobox(
            global_frame,
            textvariable=self.test_mode_var,
            width=14,
            values=[name for name, _ in TEST_MODES],
            state="readonly",
        ).grid(row=1, column=1, padx=4, pady=4, columnspan=2, sticky="w")
        ttk.Button(global_frame, text="Apply Test", command=self.apply_test_mode).grid(row=1, column=3, padx=4, pady=4)
        ttk.Button(global_frame, text="Read Registers", command=lambda: self.send_command("registers")).grid(row=1, column=4, padx=4, pady=4)
        ttk.Label(global_frame, textvariable=self.stats_var).grid(row=1, column=5, columnspan=5, padx=8, pady=4, sticky="w")
        ttk.Label(global_frame, textvariable=self.board_mode_desc_var, wraplength=920, justify="left").grid(row=2, column=0, columnspan=10, padx=4, pady=(6, 2), sticky="w")

        channels_frame = ttk.LabelFrame(self.root, text="Channel Controls", padding=8)
        channels_frame.pack(fill="both", expand=True, padx=8, pady=6)

        headers = ["Channel", "On", "Gain", "Input Mux", "Bias", "SRB2", "SRB1", "Imp P", "Imp N", "Status", "Apply Settings", "Apply Impedance"]
        for col, text in enumerate(headers):
            ttk.Label(channels_frame, text=text).grid(row=1, column=col, padx=4, pady=2, sticky="w")

        for index in range(CHANNEL_COUNT):
            self.channel_rows.append(ChannelRow(channels_frame, index, self.apply_channel_config, self.apply_impedance_config))

        help_frame = ttk.LabelFrame(self.root, text="Control Help", padding=8)
        help_frame.pack(fill="x", padx=8, pady=6)
        ttk.Label(help_frame, text=CONTROL_HELP_TEXT, wraplength=1180, justify="left").pack(anchor="w")

        log_frame = ttk.LabelFrame(self.root, text="Debug Log", padding=8)
        log_frame.pack(fill="both", expand=True, padx=8, pady=6)
        self.log_widget = ScrolledText(log_frame, height=14, wrap="word")
        self.log_widget.pack(fill="both", expand=True)
        self.log_widget.configure(state="disabled")

    def _build_live_window(self) -> None:
        plot_frame = ttk.Frame(self.data_window, padding=8)
        plot_frame.pack(fill="both", expand=True)

        self.figure = Figure(figsize=(12, 7), dpi=100)
        self.time_ax = self.figure.add_subplot(211)
        self.fft_ax = self.figure.add_subplot(212)
        self.figure.tight_layout(pad=2.5)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        for index in range(CHANNEL_COUNT):
            line, = self.time_ax.plot([], [], linewidth=0.8, label=f"Ch {index + 1}")
            self.trace_lines.append(line)

        self.time_ax.set_title("Rolling 5-Second Time Series")
        self.time_ax.set_xlabel("Time (s)")
        self.time_ax.set_ylabel("Active Channels")
        self.time_ax.grid(True, alpha=0.3)
        self.scale_text = self.time_ax.text(
            0.995,
            0.98,
            f"Row scale: +/- {int(DEFAULT_TRACE_SCALE_UV)} uV",
            transform=self.time_ax.transAxes,
            ha="right",
            va="top",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
        )

        self.fft_ax.set_title("FFT of Averaged Active Channels")
        self.fft_ax.set_xlabel("Frequency (Hz)")
        self.fft_ax.set_ylabel("Magnitude")
        self.fft_ax.grid(True, alpha=0.3)

        values_frame = ttk.LabelFrame(self.data_window, text="Latest Measurements", padding=8)
        values_frame.pack(fill="x", padx=8, pady=6)
        scale_frame = ttk.Frame(values_frame)
        scale_frame.grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 4))
        ttk.Label(scale_frame, text="Time-Series Row Scale").pack(side="left")
        scale_combo = ttk.Combobox(
            scale_frame,
            textvariable=self.trace_scale_var,
            width=10,
            values=[f"{value} uV" for value in TRACE_SCALE_OPTIONS_UV],
            state="readonly",
        )
        scale_combo.pack(side="left", padx=6)
        for index in range(CHANNEL_COUNT):
            ttk.Label(values_frame, text=f"Ch {index + 1}").grid(row=index // 4 + 1, column=(index % 4) * 2, padx=4, pady=2, sticky="w")
            ttk.Label(values_frame, textvariable=self.value_vars[index], width=28).grid(row=index // 4 + 1, column=(index % 4) * 2 + 1, padx=4, pady=2, sticky="w")

    def get_trace_scale_uv(self) -> float:
        label = self.trace_scale_var.get().replace(" uV", "").strip()
        try:
            return max(1.0, float(label))
        except ValueError:
            return DEFAULT_TRACE_SCALE_UV

    def refresh_ports(self) -> None:
        ports = sorted(port.device for port in list_ports.comports())
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror(APP_TITLE, "Select a COM port first.")
            return
        try:
            self.client.connect(port, DEFAULT_BAUD)
        except serial.SerialException as exc:
            messagebox.showerror(APP_TITLE, f"Failed to open {port}:\n{exc}")
            return
        self.status_var.set(f"Connected to {port}")
        self.send_command("ping")
        self.send_command("state")

    def disconnect(self) -> None:
        self.client.disconnect()
        self.status_var.set("Disconnected")

    def send_command(self, command: str) -> None:
        if not self.client.is_connected:
            return
        self.client.send_debug_command(command)

    def refresh_board_state(self) -> None:
        self.force_next_state_refresh = True
        self.send_command("state")

    def restore_defaults(self) -> None:
        self.force_next_state_refresh = True
        self.send_command("defaults")

    def apply_sample_rate(self) -> None:
        label = self.sample_rate_var.get().replace(" Hz", "")
        try:
            target_hz = int(label)
        except ValueError:
            return
        rate_code = next((code for hz, code in SAMPLE_RATE_CHOICES if hz == target_hz), None)
        if rate_code is not None:
            self.send_command(f"rate {rate_code}")

    def apply_board_mode(self) -> None:
        selected = self.board_mode_var.get()
        mode_value = next((value for name, value in BOARD_MODES if name == selected), None)
        if mode_value is not None:
            self.send_command(f"boardmode {mode_value}")

    def update_board_mode_description(self) -> None:
        selected = self.board_mode_var.get()
        self.board_mode_desc_var.set(BOARD_MODE_DESCRIPTIONS.get(selected, "No description available."))

    def apply_test_mode(self) -> None:
        selected = self.test_mode_var.get()
        mode_value = next((value for name, value in TEST_MODES if name == selected), None)
        if mode_value is not None:
            self.force_next_state_refresh = True
            self.send_command(f"test {mode_value}")

    def apply_channel_config(self, channel_index: int, active: bool, gain: int, input_index: int, bias: bool, srb2: bool, srb1: bool) -> None:
        self.pending_config_channel = channel_index
        self.send_command(
            f"config {channel_index + 1} {1 if active else 0} {gain} {input_index} {1 if bias else 0} {1 if srb2 else 0} {1 if srb1 else 0}"
        )

    def apply_impedance_config(self, channel_index: int, imp_p: bool, imp_n: bool) -> None:
        self.pending_impedance_channel = channel_index
        self.send_command(f"imp {channel_index + 1} {1 if imp_p else 0} {1 if imp_n else 0}")

    def append_log(self, message: str) -> None:
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def poll_events(self) -> None:
        while True:
            try:
                event_type, payload = self.client.event_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "packet":
                self.handle_packet(payload)
            elif event_type == "line":
                self.handle_line(payload)
            elif event_type == "log":
                self.append_log(str(payload))
            elif event_type == "error":
                self.append_log(str(payload))
                self.status_var.set("Disconnected")

        self.root.after(50, self.poll_events)

    def handle_line(self, line: str) -> None:
        if line.startswith("#HELLO "):
            self.mode_var.set("Debug")
            self.append_log(line)
            return

        if line.startswith("#STATE "):
            try:
                state = json.loads(line[7:])
            except json.JSONDecodeError:
                self.append_log(f"Bad state JSON: {line}")
                return
            self.apply_state(state)
            return

        if line in STATUS_MESSAGES:
            self.append_log(STATUS_MESSAGES[line])
            if line == "#OK config" and self.pending_config_channel is not None:
                self.channel_rows[self.pending_config_channel].mark_clean()
                self.pending_config_channel = None
            elif line == "#ERR config" and self.pending_config_channel is not None:
                self.channel_rows[self.pending_config_channel].mark_failed()
                self.pending_config_channel = None
            elif line == "#OK imp" and self.pending_impedance_channel is not None:
                self.channel_rows[self.pending_impedance_channel].mark_clean()
                self.pending_impedance_channel = None
            elif line == "#ERR imp":
                self.pending_impedance_channel = None
            return

        self.append_log(line)

    def apply_state(self, state: dict[str, Any]) -> None:
        self.latest_state = state
        self.sample_rate_hz = int(state.get("sample_rate_hz", 250))
        self.mode_var.set("Debug")
        self.status_var.set("Streaming" if state.get("streaming") else "Connected")

        self.sample_rate_var.set(f"{self.sample_rate_hz} Hz")
        board_mode_value = int(state.get("board_mode", 0))
        for name, value in BOARD_MODES:
            if value == board_mode_value:
                self.board_mode_var.set(name)
                break
        self.update_board_mode_description()

        force = self.force_next_state_refresh
        self.force_next_state_refresh = False

        channels = state.get("channels", [])
        for row, channel_state in zip(self.channel_rows, channels):
            row.apply_state(channel_state, force=force)

    def handle_packet(self, packet: bytes) -> None:
        counts = [decode_i24(packet[offset], packet[offset + 1], packet[offset + 2]) for offset in range(2, 26, 3)]
        state_channels = self.latest_state.get("channels", [])
        microvolts = []
        for index, count in enumerate(counts):
            gain_scalar = 24
            if index < len(state_channels):
                gain_scalar = int(state_channels[index].get("gain_scalar", 24))
            microvolts.append(counts_to_uv(count, gain_scalar))

        now = time.time()
        sample_index = packet[1]
        self.packet_total += 1
        delta_samples = 1
        if self.last_packet_index is not None:
            delta = (sample_index - self.last_packet_index) & 0xFF
            if delta > 1:
                self.packet_loss += delta - 1
            if delta > 0:
                delta_samples = delta
        self.last_packet_index = sample_index
        self.samples_since_stats += 1
        elapsed = now - self.last_stats_time
        if elapsed >= 1.0:
            self.current_sps = self.samples_since_stats / elapsed
            self.samples_since_stats = 0
            self.last_stats_time = now
            self.stats_var.set(f"Samples: {self.packet_total} | SPS: {self.current_sps:0.1f} | Lost: {self.packet_loss}")

        if self.last_sample_time is None or self.sample_rate_hz <= 0:
            sample_time = now
        else:
            sample_time = self.last_sample_time + (delta_samples / float(self.sample_rate_hz))
        self.last_sample_time = sample_time

        self.timestamps.append(sample_time)
        self.current_counts = counts
        self.current_uv = microvolts
        for index in range(CHANNEL_COUNT):
            self.channel_buffers[index].append(microvolts[index])
            self.value_vars[index].set(f"{counts[index]} cnt / {microvolts[index]:0.2f} uV")

    def refresh_plots(self) -> None:
        timestamps = np.array(self.timestamps, dtype=float)
        if timestamps.size > 1:
            time_axis = timestamps - timestamps[-1]
            mask = time_axis >= -PLOT_SECONDS
            plot_times = time_axis[mask]
            active_mask = [True] * CHANNEL_COUNT
            channels_state = self.latest_state.get("channels", [])
            if channels_state:
                active_mask = [bool(channel.get("active", 1)) for channel in channels_state]

            active_indices = [index for index, is_active in enumerate(active_mask) if is_active]
            if not active_indices:
                active_indices = list(range(CHANNEL_COUNT))

            centered_series: dict[int, np.ndarray] = {}
            for index in active_indices:
                channel_data = np.array(self.channel_buffers[index], dtype=float)
                if channel_data.size != timestamps.size or plot_times.size == 0:
                    continue
                plot_segment = channel_data[mask]
                if plot_segment.size == 0:
                    continue
                centered = plot_segment - np.median(plot_segment)
                centered_series[index] = centered

            trace_scale_uv = self.get_trace_scale_uv()
            base_spacing = trace_scale_uv * 3.0
            self.scale_text.set_text(f"Row scale: +/- {int(trace_scale_uv)} uV")

            y_tick_positions = []
            y_tick_labels = []

            for line in self.trace_lines:
                line.set_data([], [])
                line.set_visible(False)

            for order, index in enumerate(active_indices):
                line = self.trace_lines[index]
                centered = centered_series.get(index)
                if centered is None:
                    continue
                offset = order * base_spacing
                line.set_data(plot_times, centered + offset)
                line.set_visible(True)
                line.set_alpha(1.0)
                y_tick_positions.append(offset)
                y_tick_labels.append(f"Ch {index + 1}")

            if y_tick_positions:
                self.time_ax.set_xlim(-PLOT_SECONDS, 0.0)
                self.time_ax.set_ylim(-base_spacing * 0.75, y_tick_positions[-1] + base_spacing * 0.75)
                self.time_ax.set_yticks(y_tick_positions)
                self.time_ax.set_yticklabels(y_tick_labels)
            else:
                self.time_ax.set_xlim(-PLOT_SECONDS, 0.0)
                self.time_ax.set_ylim(-1.0, 1.0)
                self.time_ax.set_yticks([])

            active_arrays = []
            for index in range(CHANNEL_COUNT):
                if index < len(active_mask) and not active_mask[index]:
                    continue
                channel_data = np.array(self.channel_buffers[index], dtype=float)
                if channel_data.size >= 32:
                    active_arrays.append(channel_data)

            self.fft_ax.clear()
            self.fft_ax.set_title("FFT of Averaged Active Channels")
            self.fft_ax.set_xlabel("Frequency (Hz)")
            self.fft_ax.set_ylabel("Magnitude (log scale)")
            self.fft_ax.grid(True, alpha=0.3)
            self.fft_ax.set_yscale("log")

            if active_arrays and self.sample_rate_hz > 0:
                min_len = min(array.size for array in active_arrays)
                stacked = np.vstack([array[-min_len:] for array in active_arrays])
                averaged = stacked.mean(axis=0)
                averaged = averaged - averaged.mean()
                window = np.hanning(min_len)
                spectrum = np.fft.rfft(averaged * window)
                freqs = np.fft.rfftfreq(min_len, d=1.0 / self.sample_rate_hz)
                fft_mask = freqs <= FFT_MAX_HZ
                magnitudes = np.maximum(np.abs(spectrum)[fft_mask], 1e-3)
                self.fft_ax.plot(freqs[fft_mask], magnitudes, color="tab:orange", linewidth=1.0)
                self.fft_ax.set_xlim(0.0, FFT_MAX_HZ)
                self.fft_ax.set_ylim(bottom=1e-3)

        self.canvas.draw_idle()
        self.root.after(PLOT_REFRESH_MS, self.refresh_plots)

    def on_close(self) -> None:
        self.client.disconnect()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = DebugApp()
    app.run()


if __name__ == "__main__":
    main()