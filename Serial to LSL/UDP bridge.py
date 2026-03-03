#!/usr/bin/env python3
"""
GUI Serial -> LSL / UDP bridge for Cyton-style 8-channel packets.

Features:
- COM port selection + refresh
- Start/stop bridge with ESP32 stream control (s then b)
- Real-time 8-channel EEG graph
- 0-100 Hz FFT of averaged signal
- Optional LSL and UDP outputs

Expected incoming packet format (33 bytes):
  [0]    0xA0
  [1]    sample id (0..255)
  [2:26] 8 channels x signed 24-bit big-endian
  [26:32] aux bytes (ignored)
  [32]   0xC0
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Iterable, List, Optional

import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from serial.tools import list_ports

try:
    from pylsl import StreamInfo, StreamOutlet, local_clock
except ImportError:
    StreamInfo = None
    StreamOutlet = None
    local_clock = None


PACKET_START = 0xA0
PACKET_END = 0xC0
PACKET_LEN = 33
CHANNEL_COUNT = 8
DEFAULT_SAMPLE_RATE = 250


@dataclass
class BridgeConfig:
    port: str
    baud: int
    scale_uv: bool
    gain: float
    vref: float
    startup_delay: float
    enable_lsl: bool
    lsl_name: str
    lsl_type: str
    lsl_source_id: str
    enable_udp: bool
    udp_host: str
    udp_port: int


def signed24_be_to_int(b0: int, b1: int, b2: int) -> int:
    raw = (b0 << 16) | (b1 << 8) | b2
    if raw & 0x800000:
        raw -= 1 << 24
    return raw


def counts_to_uv(counts: int, vref: float, gain: float) -> float:
    scale = (vref / (gain * ((1 << 23) - 1))) * 1e6
    return counts * scale


def decode_packet(packet: bytes) -> tuple[int, List[int]]:
    sample_id = packet[1]
    channels = []
    offset = 2
    for _ in range(CHANNEL_COUNT):
        channels.append(
            signed24_be_to_int(packet[offset], packet[offset + 1], packet[offset + 2])
        )
        offset += 3
    return sample_id, channels


class PacketParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterable[bytes]:
        self._buf.extend(data)

        while True:
            start_idx = self._buf.find(bytes([PACKET_START]))
            if start_idx < 0:
                self._buf.clear()
                return

            if start_idx > 0:
                del self._buf[:start_idx]

            if len(self._buf) < PACKET_LEN:
                return

            packet = self._buf[:PACKET_LEN]
            if packet[-1] == PACKET_END:
                yield bytes(packet)
                del self._buf[:PACKET_LEN]
            else:
                del self._buf[0]


class SerialBridgeWorker:
    def __init__(self, config: BridgeConfig, output_queue: Queue):
        self.config = config
        self.output_queue = output_queue

        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._ser: Optional[serial.Serial] = None

        self._parser = PacketParser()
        self._lsl_outlet = None
        self._udp_sock: Optional[socket.socket] = None

        self.sample_count = 0
        self.lost_total = 0
        self.last_sample_id: Optional[int] = None
        self.t0 = 0.0

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _emit_status(self, msg: str) -> None:
        self.output_queue.put(("status", msg))

    def _emit_error(self, msg: str) -> None:
        self.output_queue.put(("error", msg))

    def _emit_sample(self, sample_id: int, sample: List[float]) -> None:
        self.output_queue.put(("sample", sample_id, sample, time.time()))

    def _setup_outputs(self) -> bool:
        if self.config.enable_lsl:
            if StreamInfo is None or StreamOutlet is None or local_clock is None:
                self._emit_error("pylsl missing. Install with: pip install pylsl")
                return False

            info = StreamInfo(
                self.config.lsl_name,
                self.config.lsl_type,
                CHANNEL_COUNT,
                DEFAULT_SAMPLE_RATE,
                "float32",
                self.config.lsl_source_id,
            )
            chns = info.desc().append_child("channels")
            for idx in range(CHANNEL_COUNT):
                ch = chns.append_child("channel")
                ch.append_child_value("label", f"CH{idx + 1}")
                ch.append_child_value("unit", "uV" if self.config.scale_uv else "counts")
                ch.append_child_value("type", "EEG")
            self._lsl_outlet = StreamOutlet(info)
            self._emit_status(f"LSL enabled: {self.config.lsl_name}")

        if self.config.enable_udp:
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._emit_status(
                f"UDP enabled: {self.config.udp_host}:{self.config.udp_port}"
            )

        return True

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baud,
                timeout=0.1,
            )
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._emit_status(
                f"Serial opened: {self.config.port} @ {self.config.baud}"
            )
            return True
        except Exception as exc:
            self._emit_error(f"Could not open serial port: {exc}")
            return False

    def _close_all(self) -> None:
        if self._ser is not None:
            try:
                self._ser.write(b"s")
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        if self._udp_sock is not None:
            try:
                self._udp_sock.close()
            except Exception:
                pass
            self._udp_sock = None

        self._lsl_outlet = None

    def _run(self) -> None:
        self.sample_count = 0
        self.lost_total = 0
        self.last_sample_id = None
        self.t0 = time.time()

        if not self._open_serial():
            self._running.clear()
            return

        if not self._setup_outputs():
            self._close_all()
            self._running.clear()
            return

        try:
            self._ser.write(b"s")
            time.sleep(self.config.startup_delay)
            self._ser.reset_input_buffer()
            self._ser.write(b"b")
            self._emit_status("Sent stream start command ('b').")

            while self._running.is_set():
                chunk = self._ser.read(512)
                if not chunk:
                    continue

                for packet in self._parser.feed(chunk):
                    sample_id, channel_counts = decode_packet(packet)

                    if self.last_sample_id is not None:
                        expected = (self.last_sample_id + 1) & 0xFF
                        if sample_id != expected:
                            self.lost_total += (sample_id - expected) & 0xFF
                    self.last_sample_id = sample_id

                    if self.config.scale_uv:
                        sample = [
                            counts_to_uv(x, self.config.vref, self.config.gain)
                            for x in channel_counts
                        ]
                    else:
                        sample = [float(x) for x in channel_counts]

                    if self._lsl_outlet is not None:
                        self._lsl_outlet.push_sample(sample, local_clock())

                    if self._udp_sock is not None:
                        payload = {
                            "sample_id": sample_id,
                            "timestamp": time.time(),
                            "channels": sample,
                        }
                        self._udp_sock.sendto(
                            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                            (self.config.udp_host, self.config.udp_port),
                        )

                    self.sample_count += 1
                    self._emit_sample(sample_id, sample)

        except Exception as exc:
            self._emit_error(f"Bridge runtime error: {exc}")
        finally:
            self._close_all()
            self._emit_status("Bridge stopped.")
            self._running.clear()


class BridgeApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 ADS1299 Bridge + Scope")
        self.root.geometry("1280x860")

        self.queue: Queue = Queue()
        self.worker: Optional[SerialBridgeWorker] = None

        self.sample_rate = DEFAULT_SAMPLE_RATE
        self.display_seconds = 4.0
        self.max_samples = int(self.sample_rate * self.display_seconds)

        self.ch_buffers = [deque(maxlen=self.max_samples) for _ in range(CHANNEL_COUNT)]
        self.t_buffer = deque(maxlen=self.max_samples)

        self.sample_total = 0
        self.last_sample_id: Optional[int] = None
        self.lost_total = 0
        self.start_time = time.time()

        self._build_ui()
        self.refresh_ports()
        self._schedule_update()

    def _build_ui(self) -> None:
        controls = ttk.Frame(self.root)
        controls.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)

        ttk.Label(controls, text="COM Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, padx=4)

        ttk.Button(controls, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=4
        )

        ttk.Label(controls, text="Baud:").grid(row=0, column=3, sticky="w", padx=(12, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(controls, textvariable=self.baud_var, width=9).grid(row=0, column=4, padx=4)

        self.scale_uv_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="Scale to uV", variable=self.scale_uv_var).grid(
            row=0, column=5, padx=(12, 4)
        )

        ttk.Label(controls, text="Gain:").grid(row=0, column=6, sticky="w")
        self.gain_var = tk.StringVar(value="24")
        ttk.Entry(controls, textvariable=self.gain_var, width=6).grid(row=0, column=7, padx=4)

        ttk.Label(controls, text="Vref:").grid(row=0, column=8, sticky="w")
        self.vref_var = tk.StringVar(value="4.5")
        ttk.Entry(controls, textvariable=self.vref_var, width=6).grid(row=0, column=9, padx=4)

        self.enable_lsl_var = tk.BooleanVar(value=True)
        self.enable_udp_var = tk.BooleanVar(value=True)

        ttk.Checkbutton(controls, text="LSL", variable=self.enable_lsl_var).grid(
            row=1, column=0, sticky="w", pady=(6, 0)
        )
        ttk.Checkbutton(controls, text="UDP", variable=self.enable_udp_var).grid(
            row=1, column=1, sticky="w", pady=(6, 0)
        )

        ttk.Label(controls, text="UDP Host:").grid(row=1, column=2, sticky="e", pady=(6, 0))
        self.udp_host_var = tk.StringVar(value="127.0.0.1")
        ttk.Entry(controls, textvariable=self.udp_host_var, width=14).grid(
            row=1, column=3, padx=4, pady=(6, 0)
        )

        ttk.Label(controls, text="UDP Port:").grid(row=1, column=4, sticky="e", pady=(6, 0))
        self.udp_port_var = tk.StringVar(value="12345")
        ttk.Entry(controls, textvariable=self.udp_port_var, width=8).grid(
            row=1, column=5, padx=4, pady=(6, 0)
        )

        self.start_button = ttk.Button(controls, text="Start Bridge", command=self.start_bridge)
        self.start_button.grid(row=1, column=8, padx=(18, 4), pady=(6, 0))

        self.stop_button = ttk.Button(
            controls,
            text="Stop",
            command=self.stop_bridge,
            state=tk.DISABLED,
        )
        self.stop_button.grid(row=1, column=9, padx=4, pady=(6, 0))

        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.TOP, fill=tk.X, padx=8)

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.status_var, foreground="#1f4b8f").pack(
            side=tk.LEFT, padx=6
        )

        self.stats_var = tk.StringVar(value="samples=0  sps=0.0  lost=0")
        ttk.Label(status_frame, textvariable=self.stats_var).pack(side=tk.RIGHT)

        values_frame = ttk.Frame(self.root)
        values_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(4, 0))
        self.value_vars = [tk.StringVar(value=f"CH{i+1}: 0.00") for i in range(CHANNEL_COUNT)]
        for idx, var in enumerate(self.value_vars):
            ttk.Label(values_frame, textvariable=var, width=14).grid(
                row=0, column=idx, padx=2, sticky="w"
            )

        fig = Figure(figsize=(12.5, 7.2), dpi=100)
        self.ax_time = fig.add_subplot(2, 1, 1)
        self.ax_fft = fig.add_subplot(2, 1, 2)

        self.ax_time.set_title("8-Channel EEG (offset traces)")
        self.ax_time.set_xlabel("Time (s)")
        self.ax_time.set_ylabel("Amplitude")
        self.ax_time.grid(True, alpha=0.25)

        self.ax_fft.set_title("Averaged Signal FFT (0-100 Hz)")
        self.ax_fft.set_xlabel("Frequency (Hz)")
        self.ax_fft.set_ylabel("Magnitude")
        self.ax_fft.grid(True, alpha=0.25)
        self.ax_fft.set_xlim(0, 100)

        self.lines = [
            self.ax_time.plot([], [], lw=1.0, label=f"CH{i+1}")[0]
            for i in range(CHANNEL_COUNT)
        ]
        self.fft_line = self.ax_fft.plot([], [], lw=1.4, color="tab:purple")[0]

        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.canvas = canvas

    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        labels = [p.device for p in ports]
        self.port_combo["values"] = labels
        if not labels:
            self.port_var.set("")
            return

        if self.port_var.get() not in labels:
            cp210 = None
            for p in ports:
                text = f"{p.device} {p.description} {p.hwid}".lower()
                if "cp210" in text or "silicon labs" in text or "10c4" in text:
                    cp210 = p.device
                    break
            self.port_var.set(cp210 if cp210 else labels[0])

    def _make_config(self) -> BridgeConfig:
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("Please select a COM port.")

        return BridgeConfig(
            port=port,
            baud=int(self.baud_var.get().strip()),
            scale_uv=self.scale_uv_var.get(),
            gain=float(self.gain_var.get().strip()),
            vref=float(self.vref_var.get().strip()),
            startup_delay=0.25,
            enable_lsl=self.enable_lsl_var.get(),
            lsl_name="OpenBCIEEG",
            lsl_type="EEG",
            lsl_source_id="ESP32_ADS1299_CYTON",
            enable_udp=self.enable_udp_var.get(),
            udp_host=self.udp_host_var.get().strip(),
            udp_port=int(self.udp_port_var.get().strip()),
        )

    def start_bridge(self) -> None:
        try:
            config = self._make_config()
        except Exception as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return

        if self.worker is not None:
            messagebox.showinfo("Bridge", "Bridge is already running.")
            return

        self._reset_buffers()
        self.worker = SerialBridgeWorker(config, self.queue)
        self.worker.start()

        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self.status_var.set("Starting...")

    def stop_bridge(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)
        self.status_var.set("Stopped")

    def _reset_buffers(self) -> None:
        for buf in self.ch_buffers:
            buf.clear()
        self.t_buffer.clear()
        self.sample_total = 0
        self.last_sample_id = None
        self.lost_total = 0
        self.start_time = time.time()

    def _process_queue(self) -> None:
        while True:
            try:
                item = self.queue.get_nowait()
            except Empty:
                break

            kind = item[0]
            if kind == "status":
                self.status_var.set(item[1])
            elif kind == "error":
                self.status_var.set(item[1])
                messagebox.showerror("Bridge error", item[1])
                self.stop_bridge()
            elif kind == "sample":
                sample_id = item[1]
                sample = item[2]
                tstamp = item[3]
                self._ingest_sample(sample_id, sample, tstamp)

    def _ingest_sample(self, sample_id: int, sample: List[float], tstamp: float) -> None:
        if self.last_sample_id is not None:
            expected = (self.last_sample_id + 1) & 0xFF
            if sample_id != expected:
                self.lost_total += (sample_id - expected) & 0xFF
        self.last_sample_id = sample_id

        self.t_buffer.append(tstamp)
        for i in range(CHANNEL_COUNT):
            self.ch_buffers[i].append(sample[i])
            self.value_vars[i].set(f"CH{i+1}: {sample[i]:8.2f}")

        self.sample_total += 1

    def _update_plots(self) -> None:
        if len(self.t_buffer) < 2:
            return

        t = np.array(self.t_buffer)
        t = t - t[-1]

        y_data = [np.array(buf) for buf in self.ch_buffers]
        if len(y_data[0]) < 2:
            return

        y_std = np.nanstd(np.concatenate(y_data))
        if not np.isfinite(y_std) or y_std < 1e-6:
            y_std = 1.0
        spacing = y_std * 6.0

        for i, line in enumerate(self.lines):
            y = y_data[i] + (CHANNEL_COUNT - 1 - i) * spacing
            line.set_data(t, y)

        self.ax_time.set_xlim(-self.display_seconds, 0)
        ymin = np.min([np.min(y) for y in y_data])
        ymax = np.max([np.max(y) for y in y_data]) + spacing * (CHANNEL_COUNT - 1)
        if np.isfinite(ymin) and np.isfinite(ymax):
            pad = max((ymax - ymin) * 0.05, 1.0)
            self.ax_time.set_ylim(ymin - pad, ymax + pad)

        min_len = min(len(buf) for buf in self.ch_buffers)
        if min_len >= 64:
            stack = np.vstack([np.array(buf)[-min_len:] for buf in self.ch_buffers])
            avg = np.mean(stack, axis=0)
            avg = avg - np.mean(avg)

            n = min(1024, len(avg))
            signal = avg[-n:]
            if n >= 16:
                window = np.hanning(n)
                fft_vals = np.fft.rfft(signal * window)
                freqs = np.fft.rfftfreq(n, d=1.0 / self.sample_rate)
                mags = np.abs(fft_vals)
                mask = freqs <= 100.0
                self.fft_line.set_data(freqs[mask], mags[mask])
                self.ax_fft.set_xlim(0, 100)
                if np.any(mask):
                    ymax_fft = np.max(mags[mask])
                    self.ax_fft.set_ylim(0, ymax_fft * 1.15 if ymax_fft > 0 else 1)

        self.canvas.draw_idle()

    def _update_stats(self) -> None:
        elapsed = max(time.time() - self.start_time, 1e-6)
        sps = self.sample_total / elapsed
        self.stats_var.set(
            f"samples={self.sample_total}  sps={sps:5.1f}  lost={self.lost_total}"
        )

    def _schedule_update(self) -> None:
        self._process_queue()
        self._update_plots()
        self._update_stats()
        self.root.after(40, self._schedule_update)


def main() -> int:
    root = tk.Tk()
    app = BridgeApp(root)

    def on_close() -> None:
        app.stop_bridge()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
