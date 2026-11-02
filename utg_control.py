"""UNI-T UTG932E function-generator control over USBTMC (SCPI).

The automated CMRR/crosstalk sweeps need the live window to know exactly which
sine frequency is being applied, so the generator is driven directly instead of
by hand. The UTG900E series accepts plain SCPI, which keeps this module small:

    :CHANnel<n>:BASE:WAVe SINe
    :CHANnel<n>:AMPLitude:UNIT VRMS
    :CHANnel<n>:BASE:AMPLitude <Vrms>   (0 V offset, so the sine is AC centred)
    :CHANnel<n>:BASE:OFFSet <V>
    :CHANnel<n>:BASE:FREQuency <Hz>
    :CHANnel<n>:BASE:PHASe <deg>
    :CHANnel<n>:LOAD <ohm>
    :CHANnel<n>:OUTPut ON|OFF
    :SYSTem:PHASe:MODe SYNChronization|INDependent

``pyvisa`` and a VISA backend are optional: when they are missing the class
still imports and every call raises :class:`UTGError`, so the rest of the
application keeps working on machines without a generator.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

try:  # pragma: no cover - depends on the test machine
    import pyvisa
except ImportError:  # pragma: no cover
    pyvisa = None

# UNI-T USB vendor id, used to recognise the generator among all VISA resources.
UNIT_USB_VID = "0x6656"


class UTGError(RuntimeError):
    """Any failure talking to the generator."""


_NUMBER_PATTERN = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")


def parse_scpi_float(text: object) -> Optional[float]:
    """Pull a number out of a SCPI answer like ``'1.000000e+03'`` or ``'2e+0\\n'``."""

    match = _NUMBER_PATTERN.search(str(text or ""))
    if not match:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


@dataclass(slots=True)
class SweepDrive:
    """How the stimulus is applied to the board."""

    # "common": CH1 feeds every input, which is a common-mode stimulus (CMRR).
    # "differential": CH1 and CH2 are phase locked 180 degrees apart, so the
    # board sees a real differential voltage (crosstalk / gain).
    # "gain": CH1 drives one input against the board reference; the measured
    # amplitude divided by the applied amplitude is the channel gain.
    kind: str = "common"

    @property
    def label(self) -> str:
        return {
            "common": "Common mode (CH1)",
            "differential": "Differential (CH1 0 deg / CH2 180 deg)",
            "gain": "Gain (CH1 into the input under test)",
        }.get(self.kind, self.kind)

    @property
    def short(self) -> str:
        return {"common": "common-mode", "differential": "differential"}.get(self.kind, "gain")

    @property
    def uses_channel_2(self) -> bool:
        return self.kind == "differential"


DEFAULT_SWEEP_FREQUENCIES_HZ: tuple[float, ...] = (
    110.0,
    95.0,
    90.0,
    85.0,
    80.0,
    70.0,
    60.0,
    55.0,
    45.0,
    40.0,
    35.0,
    25.0,
    20.0,
    15.0,
    10.0,
    7.0,
    5.0,
)

# Dwell rules: long enough for the front end to settle and for the FFT to see
# whole cycles. Above 50 Hz ten periods, at or below it five periods, and never
# less than 300 ms.
DEFAULT_MIN_DWELL_S = 0.3
DEFAULT_PERIODS_ABOVE_50HZ = 10
DEFAULT_PERIODS_BELOW_50HZ = 5

# The UTG932E clamps :BASE:AMPLitude at 1 mVrms (verified on the instrument: a
# write of 0 reads back as 1.000000e-03), so "0 V" is the smallest sine the
# generator can hold, driven through its ~50 ohm source impedance.
MIN_AMPLITUDE_VRMS = 1e-3

# The manual output modes. "zero" keeps the output enabled at that minimum
# amplitude, which is as close to a short to ground as a generator gets; "off"
# opens the output relay and leaves the input floating.
OUTPUT_OFF = "off"
OUTPUT_ZERO = "zero"
OUTPUT_SINGLE = "single"
OUTPUT_DIFFERENTIAL = "differential"
OUTPUT_MODES = (OUTPUT_OFF, OUTPUT_ZERO, OUTPUT_SINGLE, OUTPUT_DIFFERENTIAL)

# Resistive divider between the generator and the amplifier. The defaults are the
# 1 Mohm / 100 ohm divider of this front end, i.e. 100/1000100 = 9.999e-05.
DEFAULT_ATTENUATOR_SERIES_OHMS = 1_000_000.0
DEFAULT_ATTENUATOR_SHUNT_OHMS = 100.0


def attenuator_ratio(
    series_ohms: float = DEFAULT_ATTENUATOR_SERIES_OHMS,
    shunt_ohms: float = DEFAULT_ATTENUATOR_SHUNT_OHMS,
) -> float:
    """Divider ratio ``shunt / (series + shunt)`` from the two resistors."""

    series = max(float(series_ohms), 0.0)
    shunt = max(float(shunt_ohms), 0.0)
    total = series + shunt
    return shunt / total if total > 0 else 1.0


def per_channel_amplitude_vrms(total_vrms: float, differential: bool) -> float:
    """Amplitude each generator channel has to emit for a requested level.

    In differential mode the two outputs are in series as far as the amplifier is
    concerned, so CH1 0 deg and CH2 180 deg *add*: asking for 1 Vrms differential
    means 0.5 Vrms on each channel, not 1 Vrms on each. Single-ended uses CH1
    alone and takes the value as it is.
    """

    total = max(float(total_vrms), MIN_AMPLITUDE_VRMS)
    return total / 2.0 if differential else total


@dataclass(slots=True)
class GeneratorOutput:
    """A manual generator state: off, held at 0 V, or a plain sine."""

    mode: str = OUTPUT_OFF
    frequency_hz: float = 50.0
    amplitude_vrms: float = 0.2
    load_ohms: int = 10000

    @property
    def differential(self) -> bool:
        return self.mode == OUTPUT_DIFFERENTIAL

    @property
    def per_channel_vrms(self) -> float:
        return per_channel_amplitude_vrms(self.amplitude_vrms, self.differential)

    def describe(self) -> str:
        if self.mode == OUTPUT_OFF:
            return "output off"
        if self.mode == OUTPUT_ZERO:
            return f"0 V held on both channels ({MIN_AMPLITUDE_VRMS * 1e3:g} mVrms residual)"
        if self.differential:
            return (
                f"{self.amplitude_vrms:g} Vrms differential at {self.frequency_hz:g} Hz "
                f"(CH1 {self.per_channel_vrms:g} Vrms 0 deg, CH2 {self.per_channel_vrms:g} Vrms 180 deg)"
            )
        return f"{self.amplitude_vrms:g} Vrms single-ended on CH1 at {self.frequency_hz:g} Hz"

    def as_dict(self) -> dict[str, object]:
        return {
            "mode": self.mode,
            "frequency_hz": float(self.frequency_hz),
            "amplitude_vrms": float(self.amplitude_vrms),
            "load_ohms": int(self.load_ohms),
        }

    @classmethod
    def from_dict(cls, payload: object) -> "GeneratorOutput":
        data = payload if isinstance(payload, dict) else {}
        mode = str(data.get("mode", OUTPUT_OFF) or OUTPUT_OFF)
        try:
            frequency = float(data.get("frequency_hz", 50.0))
        except (TypeError, ValueError):
            frequency = 50.0
        try:
            amplitude = float(data.get("amplitude_vrms", 0.2))
        except (TypeError, ValueError):
            amplitude = 0.2
        try:
            load = int(data.get("load_ohms", 10000))
        except (TypeError, ValueError):
            load = 10000
        return cls(
            mode=mode if mode in OUTPUT_MODES else OUTPUT_OFF,
            frequency_hz=max(frequency, 1e-6),
            amplitude_vrms=max(amplitude, MIN_AMPLITUDE_VRMS),
            load_ohms=max(load, 1),
        )


def dwell_seconds(
    frequency_hz: float,
    min_dwell_s: float = DEFAULT_MIN_DWELL_S,
    periods_above: int = DEFAULT_PERIODS_ABOVE_50HZ,
    periods_below: int = DEFAULT_PERIODS_BELOW_50HZ,
) -> float:
    """Time the generator has to hold one frequency before the window counts."""

    frequency = max(float(frequency_hz), 1e-6)
    periods = periods_above if frequency > 50.0 else periods_below
    return max(float(min_dwell_s), periods / frequency)


@dataclass(slots=True)
class SweepStep:
    """One frequency of an automated sweep, with the times it was active."""

    frequency_hz: float
    amplitude_vrms: float
    dwell_s: float
    start_s: float = 0.0
    settle_s: float = 0.0
    end_s: float = 0.0

    def as_metadata(self) -> str:
        return (
            f"{self.frequency_hz:.6g}|{self.amplitude_vrms:.6g}|"
            f"{self.start_s:.6f}|{self.settle_s:.6f}|{self.end_s:.6f}"
        )


@dataclass(slots=True)
class SweepPlan:
    """Everything the analyzer needs to know about one automatic sweep."""

    drive: SweepDrive = field(default_factory=SweepDrive)
    amplitude_vrms: float = 0.2
    monitor_channel: str = ""
    steps: list[SweepStep] = field(default_factory=list)
    attenuator_series_ohms: float = DEFAULT_ATTENUATOR_SERIES_OHMS
    attenuator_shunt_ohms: float = DEFAULT_ATTENUATOR_SHUNT_OHMS

    @property
    def attenuator_ratio(self) -> float:
        return attenuator_ratio(self.attenuator_series_ohms, self.attenuator_shunt_ohms)

    def metadata(self) -> dict[str, str]:
        payload = {
            "Sweep Mode": self.drive.short,
            "Sweep Drive": self.drive.label,
            "Sweep Amplitude (Vrms)": f"{self.amplitude_vrms:.6g}",
            # The per-channel level is what the generator actually emits; the
            # differential level above is their sum.
            "Sweep Per-Channel Amplitude (Vrms)": (
                f"{per_channel_amplitude_vrms(self.amplitude_vrms, self.drive.uses_channel_2):.6g}"
            ),
            # The analyzer divides the applied level by this divider, so a gain
            # sweep is only right if the recording carries the ratio that was in
            # place when it ran.
            "Input Attenuator Series (ohm)": f"{self.attenuator_series_ohms:.6g}",
            "Input Attenuator Shunt (ohm)": f"{self.attenuator_shunt_ohms:.6g}",
            "Input Attenuator Ratio": f"{self.attenuator_ratio:.6g}",
            "Sweep Monitor Channel": self.monitor_channel or "none",
            "Sweep Step Count": str(len(self.steps)),
        }
        for index, step in enumerate(self.steps, start=1):
            payload[f"Sweep Step {index}"] = step.as_metadata()
        return payload


@dataclass(slots=True)
class SweepSettings:
    """Everything the automated-measurement window remembers between sessions."""

    resource: str = ""
    amplitude_vrms: float = 0.2
    load_ohms: int = 10000
    drive_kind: str = "common"
    frequencies: list[float] = field(default_factory=lambda: list(DEFAULT_SWEEP_FREQUENCIES_HZ))
    min_dwell_s: float = DEFAULT_MIN_DWELL_S
    periods_above: int = DEFAULT_PERIODS_ABOVE_50HZ
    periods_below: int = DEFAULT_PERIODS_BELOW_50HZ
    monitor_channel: str = ""
    output_enabled: bool = True
    # Manual generator panel state, kept next to the sweep plan because both
    # drive the same instrument.
    frequency_hz: float = 50.0
    output_mode: str = OUTPUT_OFF
    attenuator_series_ohms: float = DEFAULT_ATTENUATOR_SERIES_OHMS
    attenuator_shunt_ohms: float = DEFAULT_ATTENUATOR_SHUNT_OHMS

    @property
    def attenuator_ratio(self) -> float:
        return attenuator_ratio(self.attenuator_series_ohms, self.attenuator_shunt_ohms)

    def output(self) -> "GeneratorOutput":
        return GeneratorOutput(
            mode=self.output_mode,
            frequency_hz=self.frequency_hz,
            amplitude_vrms=self.amplitude_vrms,
            load_ohms=self.load_ohms,
        )

    def steps(self) -> list[SweepStep]:
        return build_steps(
            self.frequencies,
            self.amplitude_vrms,
            self.min_dwell_s,
            self.periods_above,
            self.periods_below,
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "resource": self.resource,
            "amplitude_vrms": float(self.amplitude_vrms),
            "load_ohms": int(self.load_ohms),
            "drive_kind": self.drive_kind,
            "frequencies": [float(value) for value in self.frequencies],
            "min_dwell_s": float(self.min_dwell_s),
            "periods_above": int(self.periods_above),
            "periods_below": int(self.periods_below),
            "monitor_channel": self.monitor_channel,
            "frequency_hz": float(self.frequency_hz),
            "output_mode": self.output_mode,
            "attenuator_series_ohms": float(self.attenuator_series_ohms),
            "attenuator_shunt_ohms": float(self.attenuator_shunt_ohms),
        }

    @classmethod
    def from_dict(cls, payload: object) -> "SweepSettings":
        data = payload if isinstance(payload, dict) else {}
        frequencies = data.get("frequencies")
        parsed = [float(value) for value in frequencies] if isinstance(frequencies, list) and frequencies else []
        try:
            # Older settings files stored Vpp under "amplitude_vpp". Convert it so
            # a stored 0.2 does not silently become 0.2 Vrms (2.8x the amplitude).
            legacy_vpp = data.get("amplitude_vpp")
            if legacy_vpp and not data.get("amplitude_vrms"):
                amplitude = float(legacy_vpp) / (2.0 * np.sqrt(2.0))
            else:
                amplitude = float(data.get("amplitude_vrms", 0.2) or 0.2)
        except (TypeError, ValueError):
            amplitude = 0.2
        return cls(
            resource=str(data.get("resource", "") or ""),
            amplitude_vrms=max(amplitude, 1e-4),
            load_ohms=int(data.get("load_ohms", 10000) or 10000),
            drive_kind=str(data.get("drive_kind", "common") or "common"),
            frequencies=parsed or list(DEFAULT_SWEEP_FREQUENCIES_HZ),
            min_dwell_s=float(data.get("min_dwell_s", DEFAULT_MIN_DWELL_S) or DEFAULT_MIN_DWELL_S),
            periods_above=int(data.get("periods_above", DEFAULT_PERIODS_ABOVE_50HZ) or DEFAULT_PERIODS_ABOVE_50HZ),
            periods_below=int(data.get("periods_below", DEFAULT_PERIODS_BELOW_50HZ) or DEFAULT_PERIODS_BELOW_50HZ),
            monitor_channel=str(data.get("monitor_channel", "") or ""),
            frequency_hz=float(data.get("frequency_hz", 50.0) or 50.0),
            output_mode=(
                str(data.get("output_mode", OUTPUT_OFF) or OUTPUT_OFF)
                if str(data.get("output_mode", OUTPUT_OFF) or OUTPUT_OFF) in OUTPUT_MODES
                else OUTPUT_OFF
            ),
            attenuator_series_ohms=float(
                data.get("attenuator_series_ohms", DEFAULT_ATTENUATOR_SERIES_OHMS)
                or DEFAULT_ATTENUATOR_SERIES_OHMS
            ),
            attenuator_shunt_ohms=float(
                data.get("attenuator_shunt_ohms", DEFAULT_ATTENUATOR_SHUNT_OHMS)
                or DEFAULT_ATTENUATOR_SHUNT_OHMS
            ),
        )


def list_resources() -> list[str]:
    """VISA resources that look like a UNI-T generator, then everything else."""

    if pyvisa is None:
        return []
    try:
        manager = pyvisa.ResourceManager()
    except Exception as exc:  # noqa: BLE001 - a missing VISA backend is normal
        raise UTGError(f"No VISA backend available: {exc}") from exc
    try:
        resources = list(manager.list_resources())
    except Exception as exc:  # noqa: BLE001
        raise UTGError(f"Could not list VISA resources: {exc}") from exc
    finally:
        try:
            manager.close()
        except Exception:  # noqa: BLE001
            pass
    preferred = [item for item in resources if UNIT_USB_VID.lower() in item.lower()]
    others = [item for item in resources if item not in preferred]
    return preferred + others


class UTG932E:
    """Minimal SCPI driver: sine frequency, amplitude, offset, phase, on/off."""

    # Read strategies tried in order. Some pyvisa-py USB builds only return data
    # when the termination is left to the device, and a few UNI-T firmwares answer
    # with a terminator the resource does not expect, which shows up as a plain
    # VI_ERROR_TMO even though every command is executed.
    READ_STYLES = ("term", "raw")

    def __init__(self, resource: str = "", timeout_ms: int = 3000) -> None:
        self.resource = resource
        self.timeout_ms = int(timeout_ms)
        self._instrument = None
        self._manager = None
        self._read_style = "term"
        # True once every read strategy has timed out: the sweep can then still
        # run (writes work), it just cannot confirm the readback.
        self.blind = False
        self.identity = ""
        # (differential, per-channel Vrms) last programmed by prepare_sweep, so a
        # sweep step can tell whether it only has to change the frequency.
        self._sweep_signature: Optional[tuple[bool, float]] = None

    # -- lifecycle ---------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._instrument is not None

    def open(self, resource: Optional[str] = None) -> str:
        if pyvisa is None:
            raise UTGError("pyvisa is not installed (pip install pyvisa pyvisa-py)")
        target = resource or self.resource
        if not target:
            candidates = list_resources()
            if not candidates:
                raise UTGError("No VISA instrument found; is the generator plugged in and switched on?")
            target = candidates[0]
        try:
            self._manager = pyvisa.ResourceManager()
            self._instrument = self._manager.open_resource(target)
            self._instrument.timeout = self.timeout_ms
            self._instrument.write_termination = "\n"
            self._instrument.read_termination = "\n"
        except Exception as exc:  # noqa: BLE001
            self.close()
            raise UTGError(f"Could not open {target}: {exc}") from exc
        self.resource = target
        # A failure to answer *IDN? is the single most useful diagnostic: the
        # port opened but the instrument is not talking. Try a wake-up write and
        # a retry, then fall back to blind mode rather than refusing to work at
        # all - writes are what the sweep actually needs.
        self.identity = ""
        try:
            self.identity = self.query("*IDN?").strip()
        except UTGError:
            try:
                self._instrument.write("*CLS")
                self._instrument.write("*IDN?")
                self.identity = self._read(self._read_style).strip()
            except Exception:  # noqa: BLE001 - blind operation is acceptable
                self.blind = True
                self.identity = ""
        if self.blind or not self.identity:
            self.blind = True
            self.identity = f"{target} (write-only: the generator accepts commands but answers no queries)"
        return self.identity

    def diagnose(self) -> list[str]:
        """Report raw traffic so a silent generator can be debugged.

        Prints every query it tries together with exactly what came back (or the
        exception), which is the difference between "writes go out, nothing comes
        back" and "the resource belongs to another device".
        """

        lines: list[str] = [f"Resource: {self.resource}"]
        for command in ("*IDN?", ":SYSTem:LOCK?", ":CHANnel1:OUTPut?", ":CHANnel1:BASE:FREQuency?"):
            try:
                answer = self.query(command)
                lines.append(f"{command} -> {answer!r}")
            except UTGError as exc:
                lines.append(f"{command} -> ERROR {exc}")
        return lines

    def close(self) -> None:
        for closer in (self._instrument, self._manager):
            if closer is None:
                continue
            try:
                closer.close()
            except Exception:  # noqa: BLE001
                pass
        self._instrument = None
        self._manager = None

    def __enter__(self) -> "UTG932E":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- low level ---------------------------------------------------------

    def write(self, command: str) -> None:
        if self._instrument is None:
            raise UTGError("Generator is not connected")
        try:
            self._instrument.write(command)
        except Exception as exc:  # noqa: BLE001
            raise UTGError(f"{command} failed: {exc}") from exc

    def write_many(self, commands: Sequence[str]) -> None:
        """Send several SCPI commands back to back, one message each.

        Chaining them into a single message (``:CHANnel1:...;:CHANnel2:...``) is
        *not* usable here: measured on the UTG900E (firmware 3.06), only the first
        command of a chained message takes effect and the instrument then stops
        answering queries until the link is reset. So the commands go out as
        separate writes - a few milliseconds apart over USB, which is far below any
        sweep dwell, and correct on this firmware.
        """

        for command in commands:
            cleaned = command.strip().rstrip(";")
            if cleaned:
                self.write(cleaned)

    def _read(self, style: str) -> str:
        instrument = self._instrument
        assert instrument is not None
        if style == "term":
            instrument.read_termination = "\n"
            return str(instrument.read())
        instrument.read_termination = None
        raw = instrument.read_raw()
        return raw.decode("ascii", "replace") if isinstance(raw, bytes) else str(raw)

    def query(self, command: str) -> str:
        """Send a query and read the answer, trying every read style once.

        A CLI-generated ``VI_ERROR_TMO`` means the command was accepted but no
        answer came back through the configured termination, which happens on some
        pyvisa-py USB builds even though the instrument is fine. Switching the read
        style usually fixes it; when none of them works the link is marked blind so
        callers can still program the generator.
        """

        if self._instrument is None:
            raise UTGError("Generator is not connected")
        last_error: Optional[Exception] = None
        # Start from whichever style last worked, so a link whose first style
        # times out does not pay that timeout on every single query (which would
        # make a sweep of 17 steps x 6 queries take minutes).
        order = list(self.READ_STYLES)
        if self._read_style in order:
            order.remove(self._read_style)
            order.insert(0, self._read_style)
        for style in order:
            try:
                self._instrument.write(command)
                answer = self._read(style)
                if self._read_style != style:
                    self._read_style = style
                return answer
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                error_code = str(exc)
                if "TMO" not in error_code and "timeout" not in error_code.lower():
                    # A real protocol error: do not paper over it.
                    raise UTGError(f"{command} failed: {exc}") from exc
                try:
                    self._instrument.clear()
                except Exception:  # noqa: BLE001
                    pass
        self.blind = True
        raise UTGError(f"{command} failed: {last_error}")

    # -- high level --------------------------------------------------------

    def set_output_load(self, channel: int, ohms: int = 10000) -> None:
        """Tell the generator the DUT is high impedance.

        With a 50 ohm assumption the generator halves the amplitude it actually
        drives; the EEG front end is a high impedance load, so the programmed Vrms
        has to be the open-circuit value.
        """

        self.write(f":CHANnel{int(channel)}:LOAD {max(int(ohms), 1)}")

    def sine_commands(
        self,
        channel: int,
        frequency_hz: float,
        amplitude_vrms: float,
        offset_v: float = 0.0,
        phase_deg: Optional[float] = None,
    ) -> list[str]:
        """The SCPI sequence for :meth:`set_sine`, without sending it."""

        channel = int(channel)
        commands = [
            f":CHANnel{channel}:MODe CONTINUE",
            f":CHANnel{channel}:BASE:WAVe SINe",
            f":CHANnel{channel}:AMPLitude:UNIT VRMS",
            f":CHANnel{channel}:BASE:AMPLitude {max(float(amplitude_vrms), 1e-4):.6g}",
            # Always 0 V: the stimulus must be AC centred, never 0..2*Vpeak.
            f":CHANnel{channel}:BASE:OFFSet 0",
            f":CHANnel{channel}:BASE:FREQuency {max(float(frequency_hz), 1e-6):.6g}",
        ]
        if phase_deg is not None:
            commands.append(f":CHANnel{channel}:BASE:PHASe {float(phase_deg):.6g}")
        return commands

    def set_sine(
        self,
        channel: int,
        frequency_hz: float,
        amplitude_vrms: float,
        offset_v: float = 0.0,
        phase_deg: Optional[float] = None,
    ) -> None:
        """Program a plain, unmodulated, AC-centred sine on one channel.

        Two things matter here:

        * ``:CHANnel<n>:MODe CONTINUE`` - the UTG900E keeps its *modulation*
          setting (AM/PM/FM/FSK/Line/Log) independently of the carrier, so a
          generator left in AM mode from the front panel emits an
          amplitude-modulated "sine" no matter what ``BASE:WAVe`` says.
        * the amplitude unit is **VRMS** and the offset is forced to 0 V, so the
          waveform always swings symmetrically around ground. A 0..3 V swing is
          not a valid biopotential stimulus: it would put a DC step of half the
          peak amplitude on the input and saturate the front end.
        """

        self.write_many(self.sine_commands(channel, frequency_hz, amplitude_vrms, offset_v, phase_deg))
        # A full (re)programming means a later sweep step cannot assume the
        # waveform and amplitude are what it left behind.
        self._sweep_signature = None

    def set_frequency(self, frequency_hz: float, channels: Sequence[int] = (1,)) -> None:
        """Change the frequency, and nothing else.

        A sweep step only varies the frequency, and a generator write is not free:
        each one is a USB round trip. Writing just this parameter is what keeps a
        step down to one message per channel instead of the whole waveform setup.
        """

        value = max(float(frequency_hz), 1e-6)
        self.write_many([f":CHANnel{int(ch):d}:BASE:FREQuency {value:.6g}" for ch in channels])

    def unlock_front_panel(self) -> None:
        """Release the key/touch lock another tool or the front panel may have set.

        A locked front panel also rejects the typed SCPI parameter commands on
        some firmware revisions, which looks exactly like "the generator does not
        respond".
        """

        for command in (":SYSTem:LOCK OFF", "System:LOCK off"):
            try:
                self.write(command)
                return
            except UTGError:
                continue

    def set_output(self, channel: int, enabled: bool) -> None:
        self.write(f":CHANnel{int(channel)}:OUTPut {'ON' if enabled else 'OFF'}")

    def set_phase_mode(self, synchronised: bool) -> None:
        """Lock the start phase of the two channels.

        Differential drive is only meaningful if CH1 and CH2 start together, so
        the sweep asks for the synchronised mode before it turns the outputs on.
        """

        self.write(f":SYSTem:PHASe:MODe {'SYNChronization' if synchronised else 'INDependent'}")

    def output_state(self, channel: int) -> bool:
        return self.query(f":CHANnel{int(channel)}:OUTPut?").strip().endswith("1")

    def set_zero(self, channel: int) -> float:
        """Hold the output at 0 V and report the amplitude the generator accepted.

        A pair of test points shorted to the board ground is what a "noise floor"
        measurement wants, and an *enabled* output at 0 V is the closest a signal
        generator gets: the output amplifier actively holds the node at 0 V through
        its ~50 ohm source impedance instead of a floating open relay. The
        instrument clamps the amplitude at 1 mVrms, so the reported value is
        normally 1e-3 - small enough to look like a short to a microvolt front end,
        but it is not a hard short.
        """

        channel = int(channel)
        self._sweep_signature = None
        self.write(f":CHANnel{channel}:MODe CONTINUE")
        self.write(f":CHANnel{channel}:BASE:OFFSet 0")
        self.write(f":CHANnel{channel}:BASE:AMPLitude 0")
        self.set_output(channel, True)
        try:
            answer = parse_scpi_float(self.query(f":CHANnel{channel}:BASE:AMPLitude?"))
        except UTGError:
            answer = None
        return float(answer) if answer is not None else MIN_AMPLITUDE_VRMS

    def set_sine_differential(
        self,
        frequency_hz: float,
        total_vrms: float,
        load_ohms: int = 10000,
    ) -> float:
        """Program a differential sine whose *sum* is ``total_vrms``.

        CH1 and CH2 are phase locked 180 degrees apart, so the amplifier sees the
        difference of the two outputs: 0.5 Vrms on each channel is 1 Vrms
        differential. Returns the per-channel amplitude that was written.
        """

        each = per_channel_amplitude_vrms(total_vrms, True)
        self.set_output_load(1, load_ohms)
        self.set_output_load(2, load_ohms)
        self.set_sine(1, frequency_hz, each, 0.0, 0.0)
        self.set_sine(2, frequency_hz, each, 0.0, 180.0)
        # Restart both channels together on the shared phase reference so the
        # 180 degree relationship actually holds.
        self.set_phase_mode(True)
        self.set_output(1, False)
        self.set_output(2, False)
        self.set_output(1, True)
        self.set_output(2, True)
        return each

    def program_output(self, output: GeneratorOutput, verify: bool = True) -> list[str]:
        """Apply one of the manual states and report anything that disagrees.

        Returns human readable notes/problems, never raises for a readback
        mismatch, so the button can show what the instrument actually holds.
        """

        notes: list[str] = []
        if output.mode == OUTPUT_OFF:
            self.all_off()
            return notes
        if output.mode == OUTPUT_ZERO:
            self.set_phase_mode(False)
            residual = self.set_zero(1)
            self.set_zero(2)
            notes.append(
                f"both outputs enabled at 0 V; the generator clamps the amplitude at "
                f"{residual * 1e3:g} mVrms, so this is a 0 V drive through ~50 ohm, not a hard short"
            )
            return notes
        if output.differential:
            each = self.set_sine_differential(output.frequency_hz, output.amplitude_vrms, output.load_ohms)
            notes.append(
                f"CH1 {each:g} Vrms 0 deg, CH2 {each:g} Vrms 180 deg = "
                f"{output.amplitude_vrms:g} Vrms differential"
            )
        else:
            self.set_phase_mode(False)
            self.set_output_load(1, output.load_ohms)
            self.set_sine(1, output.frequency_hz, output.amplitude_vrms, 0.0, 0.0)
            self.set_output(2, False)
            self.set_output(1, True)
            notes.append(f"CH1 {output.amplitude_vrms:g} Vrms at {output.frequency_hz:g} Hz, CH2 off")
        if not verify or self.blind:
            return notes
        channels = (1, 2) if output.differential else (1,)
        expected = output.per_channel_vrms if output.differential else output.amplitude_vrms
        for channel in channels:
            state = self.read_back(channel)
            frequency = state.get("frequency_hz")
            if isinstance(frequency, float) and abs(frequency - output.frequency_hz) > max(
                0.01, output.frequency_hz * 1e-3
            ):
                notes.append(f"CH{channel} reports {frequency:g} Hz, set to {output.frequency_hz:g} Hz")
            amplitude = state.get("amplitude_vrms")
            if isinstance(amplitude, float) and abs(amplitude - expected) > max(expected * 0.05, 1e-6):
                notes.append(f"CH{channel} reports {amplitude:g} Vrms, set to {expected:g} Vrms")
            if output.differential and channel == 2:
                phase = state.get("phase_deg")
                if isinstance(phase, float) and abs(abs(phase) - 180.0) > 5.0:
                    notes.append(f"CH2 reports {phase:g} deg, set to 180 deg")
            if state.get("output_on") is False:
                notes.append(f"CH{channel} reports its output is off")
        return notes

    def read_frequency(self, channel: int) -> float:
        try:
            return float(self.query(f":CHANnel{int(channel)}:BASE:FREQuency?").strip())
        except (TypeError, ValueError) as exc:
            raise UTGError(f"Could not read the frequency of channel {channel}") from exc

    def read_back(self, channel: int) -> dict[str, object]:
        """Everything the generator says it is doing on one channel.

        The sweep never trusts a blind write: a firmware that ignores a typed
        parameter command still answers ``Failure`` to nothing at all, so the only
        honest check is to ask the instrument what it holds. Keys that cannot be
        queried are reported as ``None`` instead of aborting.
        """

        channel = int(channel)
        result: dict[str, object] = {"channel": channel}
        if self.blind:
            return result
        for key, command in (
            ("wave", f":CHANnel{channel}:BASE:WAVe?"),
            ("frequency_hz", f":CHANnel{channel}:BASE:FREQuency?"),
            ("amplitude_vrms", f":CHANnel{channel}:BASE:AMPLitude?"),
            ("offset_v", f":CHANnel{channel}:BASE:OFFSet?"),
            ("phase_deg", f":CHANnel{channel}:BASE:PHASe?"),
            ("output_on", f":CHANnel{channel}:OUTPut?"),
        ):
            try:
                answer = self.query(command).strip()
            except UTGError:
                result[key] = None
                continue
            if key == "wave":
                result[key] = answer
            elif key == "output_on":
                result[key] = answer.endswith("1")
            else:
                result[key] = parse_scpi_float(answer)
        return result

    # -- sweep helpers -----------------------------------------------------

    def verify_step(self, step: SweepStep, drive: SweepDrive) -> list[str]:
        """Compare the generator's readback with what was just programmed.

        Returns human readable problems, so a mistyped or rejected write shows up
        in the sweep log instead of quietly producing a wrong CMRR point.
        """

        problems: list[str] = []
        if self.blind:
            # Nothing to compare against; saying so is better than pretending the
            # step was confirmed.
            return ["the generator answers no queries, so the readback could not be checked"]
        channels = (1, 2) if drive.uses_channel_2 else (1,)
        expected_vrms = per_channel_amplitude_vrms(step.amplitude_vrms, drive.uses_channel_2)
        for channel in channels:
            state = self.read_back(channel)
            frequency = state.get("frequency_hz")
            if isinstance(frequency, float) and frequency > 0:
                if abs(frequency - step.frequency_hz) > max(0.01, step.frequency_hz * 1e-3):
                    problems.append(
                        f"CH{channel} reports {frequency:g} Hz, set to {step.frequency_hz:g} Hz"
                    )
            elif frequency is None:
                problems.append(f"CH{channel} did not report its frequency")
            amplitude = state.get("amplitude_vrms")
            if isinstance(amplitude, float) and amplitude > 0:
                if abs(amplitude - expected_vrms) > max(expected_vrms * 0.05, 1e-6):
                    problems.append(
                        f"CH{channel} reports {amplitude:g} Vrms, set to {expected_vrms:g} Vrms "
                        f"per channel for {step.amplitude_vrms:g} Vrms applied"
                    )
            output = state.get("output_on")
            if output is False:
                problems.append(f"CH{channel} reports its output is off")
            if drive.uses_channel_2 and channel == 2:
                phase = state.get("phase_deg")
                if isinstance(phase, float) and abs(abs(phase) - 180.0) > 5.0:
                    problems.append(f"CH2 reports {phase:g} deg phase, set to 180 deg")
        return problems

    def apply_step(
        self,
        step: SweepStep,
        drive: SweepDrive,
        verify: bool = False,
        load_ohms: int = 10000,
    ) -> list[str]:
        """Drive the board with one sweep frequency.

        ``step.amplitude_vrms`` is the level the *amplifier* should see. In
        differential mode the two outputs add, so each channel gets half of it.

        Only the frequency changes between steps, so only the frequency is written
        - as one message covering both channels. The waveform, amplitude, offset,
        phase and load are programmed once, when they first need to be, and the
        outputs are never switched off and on again: a sweep is a continuous
        measurement, and toggling the output would put a step into every CMRR
        point. The two channels stay 180 degrees apart because the generator
        derives both from the same clock in the synchronised phase mode, which
        :meth:`prepare_sweep` sets up before the first step.
        """

        each = per_channel_amplitude_vrms(step.amplitude_vrms, drive.uses_channel_2)
        channels = (1, 2) if drive.uses_channel_2 else (1,)

        if not self._sweep_configured(drive, each):
            self.prepare_sweep(drive, each, step.frequency_hz, load_ohms)

        self.set_frequency(step.frequency_hz, channels)
        if not verify:
            return []
        return self.verify_step(step, drive)

    def prepare_sweep(
        self,
        drive: SweepDrive,
        per_channel_vrms: float,
        frequency_hz: float,
        load_ohms: int = 10000,
    ) -> None:
        """Program everything that stays constant for a whole sweep.

        One message for the waveform and one for the output state: with the output
        already on and the phase mode locked, every later step is a single
        frequency command.
        """

        phases = (0.0, 180.0) if drive.uses_channel_2 else (0.0,)
        channels = (1, 2) if drive.uses_channel_2 else (1,)
        self.set_phase_mode(drive.uses_channel_2)
        commands: list[str] = []
        # Whatever is not being driven must be off, and that comes first: turning an
        # unwanted output off after programming it would emit the wrong waveform.
        for channel in (1, 2):
            if channel not in channels:
                commands.append(f":CHANnel{channel}:OUTPut OFF")
        for channel in (1, 2):
            commands.append(f":CHANnel{channel}:LOAD {max(int(load_ohms), 1)}")
        for index, channel in enumerate(channels):
            commands.extend(
                self.sine_commands(channel, frequency_hz, per_channel_vrms, 0.0, phases[index])
            )
        self.write_many(commands)
        self.write_many([f":CHANnel{channel}:OUTPut ON" for channel in channels])
        self._sweep_signature = (drive.uses_channel_2, round(float(per_channel_vrms), 9))

    def _sweep_configured(self, drive: SweepDrive, per_channel_vrms: float) -> bool:
        return self._sweep_signature == (
            drive.uses_channel_2,
            round(float(per_channel_vrms), 9),
        )

    def all_off(self) -> None:
        self._sweep_signature = None
        for channel in (1, 2):
            try:
                self.set_output(channel, False)
            except UTGError:
                pass


def parse_frequency_list(text: str) -> list[float]:
    """Read a free-form frequency list, one value per line or comma separated."""

    values: list[float] = []
    for chunk in str(text or "").replace(",", "\n").replace(";", "\n").split("\n"):
        cleaned = chunk.strip()
        if not cleaned:
            continue
        try:
            value = float(cleaned)
        except ValueError:
            continue
        if value > 0:
            values.append(value)
    return values


def build_steps(
    frequencies_hz: Sequence[float],
    amplitude_vrms: float,
    min_dwell_s: float = DEFAULT_MIN_DWELL_S,
    periods_above: int = DEFAULT_PERIODS_ABOVE_50HZ,
    periods_below: int = DEFAULT_PERIODS_BELOW_50HZ,
) -> list[SweepStep]:
    steps: list[SweepStep] = []
    clock = 0.0
    for frequency in frequencies_hz:
        hold = dwell_seconds(frequency, min_dwell_s, periods_above, periods_below)
        steps.append(
            SweepStep(
                frequency_hz=float(frequency),
                amplitude_vrms=float(amplitude_vrms),
                dwell_s=hold,
                start_s=clock,
                settle_s=clock,
                end_s=clock + hold,
            )
        )
        clock += hold
    return steps


def mark_step_times(steps: Sequence[SweepStep], clock_offset_s: float) -> None:
    """Shift the planned times onto the recording clock, in place."""

    for step in steps:
        step.start_s += clock_offset_s
        step.settle_s += clock_offset_s
        step.end_s += clock_offset_s


def describe_readback(state: dict[str, object]) -> str:
    """One line describing what the generator reports for a channel."""

    def number(key: str, suffix: str) -> str:
        value = state.get(key)
        return f"{value:g} {suffix}" if isinstance(value, (int, float)) else f"? {suffix}"

    output = state.get("output_on")
    output_text = "on" if output is True else ("off" if output is False else "?")
    return (
        f"CH{state.get('channel', '?')}: {state.get('wave') or '?'}, "
        f"{number('frequency_hz', 'Hz')}, {number('amplitude_vrms', 'Vrms')}, "
        f"offset {number('offset_v', 'V')}, phase {number('phase_deg', 'deg')}, output {output_text}"
    )


def selftest(
    resource: str = "",
    frequency_hz: float = 110.0,
    amplitude_vrms: float = 0.1,
) -> list[str]:
    """Prove the whole chain without touching the board.

    Connects, programs one sine, reads it back and switches the output off again.
    Used by ``python utg_control.py selftest`` and by the *Test generator* button,
    so a wrong driver / resource / front-panel state is found before a sweep
    wastes a recording.
    """

    lines: list[str] = []
    generator = UTG932E(resource)
    try:
        identity = generator.open(resource or None)
        lines.append(f"Connected: {identity}")
        lines.append(f"Resource: {generator.resource}")
        generator.unlock_front_panel()
        if generator.blind:
            lines.append(
                "NOTE: the generator accepts commands but answers no queries. Programmed values are "
                "applied (including the plain-sine mode), yet the readback cannot be verified. If you "
                "need verification, install NI-VISA and select the same resource."
            )
        try:
            generator.set_output_load(1, 10000)
        except UTGError as exc:
            lines.append(f"Load setting not accepted ({exc}); continuing")
        step = SweepStep(frequency_hz=float(frequency_hz), amplitude_vrms=float(amplitude_vrms), dwell_s=1.0)
        generator.apply_step(step, SweepDrive("common"), verify=False)
        lines.append(describe_readback(generator.read_back(1)))
        problems = generator.verify_step(step, SweepDrive("common"))
        if problems:
            lines.append("MISMATCH: " + "; ".join(problems))
        else:
            lines.append(f"OK: CH1 holds a continuous, 0 V-centred {frequency_hz:g} Hz sine at {amplitude_vrms:g} Vrms")
        return lines
    except UTGError as exc:
        lines.append(f"ERROR: {exc}")
        # Only diagnose when the link is open at all; the failure above already
        # explains a closed one.
        if generator.is_open:
            lines.extend(generator.diagnose())
        return lines
    finally:
        try:
            generator.all_off()
        except UTGError:
            pass
        generator.close()


def _main(argv: Sequence[str] | None = None) -> int:
    """Small command line front end, so the generator can be proven on its own."""

    import argparse

    parser = argparse.ArgumentParser(
        description="Control a UNI-T UTG932E and check that it accepts typed SCPI."
    )
    parser.add_argument("--resource", default="", help="VISA resource string (default: first found)")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("list", help="list VISA resources")
    sub.add_parser("idn", help="connect and print *IDN?")

    sine = sub.add_parser("sine", help="apply a sine and report what the generator read back")
    sine.add_argument("--freq", type=float, default=110.0, help="frequency in Hz")
    sine.add_argument("--amp", type=float, default=0.1, help="amplitude in Vrms")
    sine.add_argument("--offset", type=float, default=0.0, help="DC offset in V (the interface always uses 0)")
    sine.add_argument("--phase", type=float, default=None, help="phase in degrees")
    sine.add_argument("--channel", type=int, default=1, choices=(1, 2))
    sine.add_argument("--output", default="on", choices=("on", "off"))

    test = sub.add_parser("selftest", help="connect, program one sine, read it back, switch off")
    test.add_argument("--freq", type=float, default=110.0)
    test.add_argument("--amp", type=float, default=0.1)

    args = parser.parse_args(argv)
    if not args.command:
        parser.print_help()
        return 2

    try:
        if args.command == "list":
            resources = list_resources()
            if not resources:
                print("No VISA instrument found. Is the generator plugged in and switched on?")
                return 1
            for item in resources:
                print(item)
            return 0
        if args.command == "idn":
            generator = UTG932E(args.resource)
            try:
                print(generator.open(args.resource or None))
                print(f"Resource: {generator.resource}")
            finally:
                generator.close()
            return 0
        if args.command == "sine":
            generator = UTG932E(args.resource)
            try:
                print(generator.open(args.resource or None))
                generator.set_output_load(args.channel, 10000)
                generator.set_sine(args.channel, args.freq, args.amp, args.offset, args.phase)
                generator.set_output(args.channel, args.output == "on")
                print(describe_readback(generator.read_back(args.channel)))
            finally:
                generator.close()
            return 0
        for line in selftest(args.resource, args.freq, args.amp):
            print(line)
        return 0
    except UTGError as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":  # pragma: no cover - manual entry point
    raise SystemExit(_main())
