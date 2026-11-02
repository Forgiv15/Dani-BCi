# ESP32 + Dual ADS1299 OpenBCI Compatibility Notes

## What this firmware emulates

This project emulates a **Cyton-style serial board** for OpenBCI GUI. The
channel count is selected at build time with `ADS1299_CHANNELS_PER_CHIP`
(see `src/main.cpp`):

- **ADS1299-4** (`ADS1299_CHANNELS_PER_CHIP = 4`): 8 EEG channels
  (4 from ADS1 + 4 from ADS2), sent as a standard **8-channel Cyton**.
- **ADS1299-6** (`ADS1299_CHANNELS_PER_CHIP = 6`): 12 EEG channels
  (6 from ADS1 + 6 from ADS2), sent as **16-channel Cyton+Daisy** using the
  official interleaved wire format (odd samples = board, even = daisy).

Both modes use the standard Cyton packet framing: `0xA0 ... 0xC0`, a 33-byte
packet, 24-bit signed channel payloads (big-endian per channel), and 3 aux
channels (accelerometer at 25 Hz).

## OpenBCI GUI transport expectations

### 1) Serial (recommended for this firmware)

OpenBCI GUI Cyton serial mode expects a USB serial port. This is exactly what this firmware targets.

- Use your USB-UART connection to the ESP32
- Baud configured in firmware: `115200`
- 8-channel build: set the GUI board type to **Cyton**
- 12-channel build: set the GUI board type to **Cyton + Daisy (16 ch)**

### 2) Bluetooth

OpenBCI GUI Bluetooth paths are primarily for Ganglion/BLE workflows, not generic Cyton-over-BLE serial emulation.

If you want BLE/Wi-Fi instead of USB serial, you have two robust options:

1. Keep this firmware as-is and bridge data to LSL/UDP via a host app.
2. Implement BrainFlow-compatible BLE/Wi-Fi transport profile expected by GUI modes.

For fastest stability, use USB serial first.

## Implemented command compatibility

Implemented in firmware command parser:

- `v` board/version info text
- `b` start streaming
- `s` stop streaming
- `d` apply default ADS settings
- `c` 8-channel mode (`daisy removed$$$` when daisy is active)
- `C` 16-channel mode (`16$$$` / `daisy attached16$$$` / `no daisy to attach!8$$$`)
- `~0..~6` data-rate selection (ADS1299 DR bits). The rate check is made on the
  *decimated output* rate for the serial link and on the raw rate for SCLK
- `x...X` per-channel ADS settings command (channels `1-8` = board, `Q I` = daisy)
- `?` dump every ADS1299 register as plain text
- `N<0|1|2>` board-wide reference routing (0 = true differential, 1 = SRB1,
  2 = SRB2). Applied to every converter at once; `x...X` no longer touches
  `MISC1`, because SRB1 is a single switch per chip and a per-channel request
  could silently move every other channel's negative input.
- `n<converter><0|1|2>` reference routing for **one** ADS1299 (`n10` = ADS1 true
  differential, `n21` = ADS2 SRB1). SRB1 is chip-wide, so this is what lets one
  converter run SRB1 while the other stays true differential and measures the
  common-mode voltage directly. In SRB2 mode the per-channel `CHnSET` bit 3 is
  honoured, so a single input can remain differential while the rest of that
  converter is referred to SRB2. The two converters and the per-channel bit are
  fully independent: changing one never rewrites the others, and an input's
  own-INN choice survives a trip through differential or SRB1 mode because the
  host keeps it as intent and the firmware gates it on the chip mode.
- `m<n>m` output decimation (`1..32`): the converters are sampled at the `~`
  data rate and `n` conversions are block-averaged into one published packet.  This moves the sinc3 -3 dB corner (`0.262 x f_MOD`, e.g. 1048 Hz at 4000 SPS)
  above the measurement band without raising the packet rate.
- `o<kHz>o` runtime SCLK change (`1000..8000`). Streaming is paused while the
  clock changes because both converters share SCLK.
- `>` repair and verify: re-programs every register of both converters from the
  cached state, then reads all 24 registers back and prints
  `[CHK] DEV=ADS1 ID=3D NCH=6 REGOK=12 CHECKED=12` plus one
  `[CHK] BAD DEV=.. REG=.. GOT=.. WANT=..` line per register that still differs and
  `[CHK] CODE=.. PROBLEMS=.. REASON=host DONE=1`. The same re-programming runs at
  every stream start, so a half-applied configuration is never streamed.
- `P` configuration readback for the GUI: one
  `[CFG] DEV=ADS1 LINK=1 REF=srb2 REGS=<48 hex chars>` line per converter
  (registers `0x00..0x17`) plus a
  `[CFG] CODE=.. SPS=.. DECIM=.. OUT=.. SPICLK=.. SPILIMIT=.. SERIALLIMIT=.. REF=.. DONE=1`
  line. Streaming is paused for the readback and restarted afterwards.
- `z...Z` impedance-mode style command (basic support)
- legacy channel toggles: `1..8` off, `!@#$%^&*` on, daisy `q..i` off / `Q..I` on

## Power-on self test

Every boot runs a self test (~6 s) before any streaming starts and prints the
result on the serial monitor at 115200 baud. Nothing else is needed to know
whether the board is healthy:

```
=== OpenBCI ESP32 power-on self test ===
Hardware    : ADS1299-6 x2 = 12 channels (Cyton+Daisy 16ch emulation)
Sample rate : 250 Hz
--- SPI bus probe ---
bus       : SCLK=18 MISO=19 MOSI=23 @ 1 MHz, mode 1 (CPOL=0 CPHA=1)
CS pins   : ADS1=5 ADS2=4 (must idle HIGH)
levels    : CS1=1 CS2=1 DRDY1=1 DRDY2=1 RESET=1 PWDN=1 START=0
idle MISO : 0xFF (no CS asserted) - floating/pulled high
[ADS1] RREG(0x00) raw: opcode=0x00 count=0x00 data=0x3D -> ID=0x3D
[ADS1] regs 00-17: 3D 96 C0 EC 00 61 61 61 61 61 61 00 00 00 00 00 00 00 00 00 0F 00 00 00
[ADS2] RREG(0x00) raw: opcode=0x00 count=0x00 data=0x3D -> ID=0x3D
[ADS2] regs 00-17: 3D 96 C0 EC 00 61 61 61 61 61 61 00 00 00 00 00 00 00 00 00 0F 00 00 00
[SPI]  CS isolates devices        OK : ADS1=0xA5/0x5A (want 0xA5/0x5A), ADS2=0x00/0x00
[ADS1] SPI + device ID             OK : id=0x3D (expected 0x3D)
[ADS1] CLK_EN (clock master)       OK : CONFIG1=0xF6, CLK_EN=1, expected=1
[ADS1] DRDY / clock present        OK : DRDY 250 Hz (expected 250 Hz)
[ADS1] RDATAC frame                OK : status=0xC00000 ch1=-3 (status nibble must be 0xC)
[ADS1] AFE shorted input           OK : worst mean=0.31uV rms=0.42uV railed=0.0%
[ADS1] AFE internal test signal    OK : amp 1861..1864uV (expect ~1875uV) railed=0.0%
[ADS2] SPI + device ID             OK : id=0x3D (expected 0x3D)
[ADS2] CLK_EN (clock master)       OK : CONFIG1=0xD6, CLK_EN=0, expected=0
[ADS2] DRDY / clock present        OK : DRDY 250 Hz (expected 250 Hz)
[ADS2] RDATAC frame                OK : status=0xC00000 ch1=-1 (status nibble must be 0xC)
[ADS2] AFE shorted input           OK : worst mean=0.28uV rms=0.39uV railed=0.0%
[ADS2] AFE internal test signal    OK : amp 1858..1863uV (expect ~1875uV) railed=0.0%
[AUX]  LIS3DH present              OK : address=0x18 WHO_AM_I=0x33
[AUX]  LIS3DH data read            OK : X=-12 Y=41 Z=998 (4g range, ~500 LSB per g)
[SD]   SD card present             OK : type=3 size=7624MB
=== POST PASSED (15 checks, 0 failed) ===
```

What each check proves:

| Check | Meaning |
|-------|---------|
| `--- SPI bus probe ---` | Raw view: pin levels, MISO idle byte, and the exact bytes clocked out of each part |
| `RREG(0x00) raw` | `data` is the ID register. `0xFF` = nothing driving MISO, `0x00` = line low/shorted |
| `CS isolates devices` | Two patterns written to ADS1's `LOFF_FLIP`; ADS2 must not see them |
| `SPI + device ID` | SPI wiring/CS is good and the fitted part reports the right channel count (`0x3C`=4ch, `0x3D`=6ch, `0x3E`=8ch) |
| `CLK_EN (clock master)` | `CONFIG1` bit 5 read back correctly for this device's role |
| `DRDY / clock present` | The device is actually converting at 250 SPS. For ADS2 this proves the clock is reaching it from ADS1 |
| `RDATAC frame` | A real 24-bit data frame was read and its status nibble is `0xC`, proving SPI framing and conversion |
| `AFE shorted input` | All channels powered up, gain 24, inputs internally shorted → offset and noise should be tiny |
| `AFE internal test signal` | Internal 1.875 mV square wave reproduced on every channel |
| `LIS3DH` | I²C address, `WHO_AM_I`, and live axis data |
| `SD card` | Card detected and readable |

When a check fails, the offending channels are listed individually underneath so
you can see exactly which input is at fault.

### Interpreting a non-responding ADS1299

| Symptom | Likely cause |
|---------|--------------|
| `idle MISO = 0x00`, `data=0x00` | MISO held low — short to GND, or the device never releases DOUT |
| `data=0xFF` | Nothing drives MISO — no power, `PWDN` low, or wrong `CS` pin |
| ID valid but `DRDY 0 Hz` and `RDATAC frame` fails | No clock: check `CLKSEL` straps and the `CLK` trace from ADS1 |
| `CS isolates devices` fails with ADS2 seeing ADS1's data | `CS1`/`CS2` shorted or one is stuck low |
| ID is `0x3C`/`0x3E` but you fitted a `-6` part | Wrong `ADS1299_CHANNELS_PER_CHIP` for the fitted chip |

If `idle MISO` reports `0xFF` but `data` is also `0xFF`, add a 10 kΩ pull-up from
MISO to 3.3 V; a completely floating MISO makes the raw byte meaningless.

## Clock wiring (very important)

The two ADS1299s are clocked from a single oscillator:

- **ADS1** has `CLKSEL` tied **high** and `CONFIG1.CLK_EN = 1`, so its internal
  oscillator is driven out on the `CLK` pin.
- **ADS2** has `CLKSEL` tied **low**, takes that signal on `CLK` as an external
  clock, and is configured with `CLK_EN = 0`.

This creates a bootstrapping order that the firmware handles explicitly: ADS2
**cannot answer on SPI at all** until ADS1 is driving `CLK`, and ADS1 only drives
`CLK` once `CLK_EN` is written. Bring-up therefore:

1. resets and probes ADS1, then immediately writes `CONFIG1` with `CLK_EN = 1`
   (`enableClockOutput()`), waits 10 ms,
2. only then probes ADS2,
3. re-probes anything that did not answer up to 5 more times,
4. and re-checks once more at stream start (`recoverMissingDevices()`), configuring
   anything that has appeared since.

A device that is detected late still gets the full `applyCytonDefaults()` write,
including `CONFIG3.PD_REFBUF = 1`. Skipping that leaves the part at its reset
value (`CONFIG3 = 0x60`), which means **the internal reference buffer is off** and
every sample is meaningless.

If ADS2 reports `DRDY / clock present` as `FAIL`, the clock is not arriving —
check the `CLK` trace and both `CLKSEL` straps.

## Acquisition timing and data integrity

The acquisition path is built around the ADS1299's `DRDY` edge, which is the
only thing that signals "a new conversion is ready":

1. The `DRDY` ISR does nothing but give a binary semaphore and bump a counter.
2. The acquisition task blocks on that semaphore (a real block, so the idle task
   and the task watchdog keep running — busy-waiting here starves `IDLE0` and
   makes the watchdog fire every 5 s, which corrupts the UART stream).
3. **Both** ADS1299s are read back-to-back immediately after the edge, so each
   read completes ~400 µs into a 4 ms conversion period. A converter's output
   register is therefore never read near the moment it is overwritten.
4. The 24-bit status word of each frame is validated (top nibble must be `1100b`).
   A misaligned frame is discarded and counted, never published.
5. In 16-channel daisy emulation each packet carries one converter and the value
   sent is the average of the last two conversions of that converter — the same
   decimation scheme the stock Cyton+Daisy firmware uses.
6. The UART write never blocks: if the host has stalled, the packet is dropped
   (and counted) instead of costing a conversion.

`s` reports the health counters when streaming stops:

```
Success: Stream stopped (missed=0 dropped=0 corrupt=0)$$$
```

- **`missed`** — conversions where no `DRDY` arrived within 50 ms, or where no
  converter produced a valid frame.
- **`dropped`** — packets skipped because the host was not reading fast enough.
- **`corrupt`** — frames rejected by the status-word alignment check.

All three should stay at 0 during normal operation.

## What the SPI bus does during streaming (this is normal)

In `RDATAC` mode the ADS1299 converts continuously on its own. The MCU does not
send anything meaningful to start a conversion:

- **`START` is driven HIGH once** when streaming begins and stays high. It is not
  toggled per sample.
- **`MOSI` carries only `0x00` dummy bytes**, so the line sits at 0 V with no
  visible transitions. Absence of MOSI activity during streaming is expected.
- **`SCLK`** runs at 2 MHz and shows two bursts of 27 clocks per conversion (one
  per ADS1299, the full status + 8-channel frame), ~108 µs each and ~216 µs
  total.
- **`CS1` and `CS2`** each pulse low once per conversion at 250 Hz for ~110 µs.
  A multimeter will show them as constantly high; use a scope.
- **`DOUT`** streams the data; **`DRDY`** falls once per conversion at 250 Hz.

### Bus budgets

The firmware reports both limits in the `v` reply:

```
Serial link limit: 2513 SPS
SPI clock: 2000 kHz, frame 216 us x 2 devices, SPI link limit: 4166 SPS
```

- **Serial (ESP32 → PC)**: 33 bytes per packet at 10 bits/byte, 10 % headroom.
  This is normally the binding limit.
- **SPI (ESP32 → 2x ADS1299)**: each converter answers a 216-bit `RDATAC` frame
  and both are read inside the same sample period (they share `START`), so one
  sample costs 432 bits = 216 µs at 2 MHz, i.e. at most 4166 SPS with 10 %
  headroom. A `~` rate request above either limit is refused with the reason.

## Reference / SRB routing

The two reference pins go to **different** inputs of the channel - this is the one
thing that is almost always assumed wrongly:

| Pin | Register | Connects to |
|-----|----------|-------------|
| SRB1 | `MISC1` bit 5, one switch for the whole chip | the **inverting** input (INxN) of *all* channels |
| SRB2 | `CHnSET` bit 3, per channel | that channel's **non-inverting** input (INxP) |

So SRB1 is the reference-electrode pin for a referential montage (datasheet
"Referential montage": reference on SRB1, active electrodes on INxP), while SRB2
*drives* INP and is not a negative reference. A channel with `CHnSET` bit 3 closed
measures *(SRB2 pin) - (INN)*, so INN still has to be wired.

The firmware keeps one mode per converter and derives both registers:

| Mode | `MISC1` | `CHnSET` bit 3 | Montage |
|------|---------|----------------|---------|
| 0 (differential) | `0x00` | per channel (0 for own INN) | each channel uses its own INP and INN; SRB2 rows drive INP from the SRB2 pin |
| 1 (SRB1) | `0x20` | 0 | every INN tied to the SRB1 pin |
| 2 (SRB2) | `0x00` | 1 on the selected channels | INP driven from the SRB2 pin |

`configureChannel()` only tracks the SRB1 flag; the routing itself is written by
`setSrb1Routing()` / `setSrb2ForAllChannels()`, and `applyCytonDefaults()` leaves
`MISC1` alone so a default pass cannot switch the reference off underneath a
running SRB1 configuration. The mode is re-applied after `d`/`v` and when a
converter is recovered at stream start.

The firmware default is **mode 0 with every SRB2 bit clear**. It used to default
to mode 2 with all SRB2 bits set, which shorted every INxP to the SRB2 pin - on a
board where SRB2 is not wired that leaves the positive input floating, and every
channel reads as saturation or as pure float noise.

`verifyDeviceRegisters()` prints two warnings that no register comparison can
catch, because the registers are *correct*:

* `[CHK] NOTE ... SRB2_SWITCH=CLOSED ... EFFECT=INxP_DRIVEN_BY_SRB2_NOT_A_NEGATIVE_REFERENCE`
* `[CHK] WARN ... REASON=NO_DC_RETURN BIAS_SENSP=0 BIAS_SENSN=0 EFFECT=inputs_float_and_rail`

If a channel reports full-scale input, check those two first. A referenced input
without a bias/DRL return path is a floating input, whatever the register map says.

### What the configuration check actually compares

The ADS1299 does not read back every bit you write: reserved bits of `CONFIG1` and
`CONFIG2` come back as 0, and so do `DAISY_EN` (`CONFIG1` b6) and `INT_CAL`
(`CONFIG2` b4), while `CONFIG3`, `MISC1`, `BIAS_SENSP/N` and every `CHnSET` read
back exactly. Comparing whole bytes therefore reported "CONFIG1 0x04, expected
0xD4" on a converter that was configured correctly and streaming - a false alarm.

The check now masks to the bits that decide behaviour and that the readback
reflects:

| Register | Compared bits |
|----------|---------------|
| `CONFIG1` | `CLK_EN` + `DR[2:0]` (`0x27`) - a wrong data rate is still caught |
| `CONFIG2` | `INT_CAL` only while the test signal is off; `INT_CAL`/`CAL_AMP`/`CAL_FREQ` when it is on |
| `CONFIG3`, `MISC1`, `BIAS_SENSP/N`, `CHnSET` | whole byte |

`DAISY_EN` is confirmed by the write acknowledgement and by both converters
actually delivering samples; its raw value stays visible in the *Board reports*
line. A converter that is missing from the report now says whether it answered
with `LINK=0` (an SPI problem) or sent no line at all (a lost report), instead of
the bare "did not report registers".

## SD card commands (`:` prefix)

The prefix exists because every letter and digit is already a Cyton channel
selector or an existing command.

| Command | Meaning |
|---------|---------|
| `:s` | Status line: `[SD] EVENT PRESENT RECORDING FILE NAME CHANNELS RATE TOTAL_MB FREE_MB FREE_BYTES EST_SEC SAMPLES DROPPED BYTES DONE=1` |
| `:b` | Start recording to the card. Starts the data stream if it is not already running |
| `:e` | Stop and close the file |
| `:p` | Toggle (what the GPIO35 button does) |
| `:w` | Write the current board configuration to `/SZD_CFG.TXT` |
| `:r` | Read that file and apply it (rate, decimation, SPI clock, converter reference modes, per-channel settings) |
| `:n<name>;` | Set the session name, terminated by `;` |

`EVENT` is `status` for an explicit Refresh and `start` / `stop` / `timeout` /
`fail` / `busy` / `saved` for a state change, so a host can confirm an action
without waiting for the next poll. There is no periodic poll: `:s` re-reads the
card's free space, and the FAT walk behind it is slow enough to disturb a running
measurement. Only `status` and `saved` measure it (`sdCapacityCached()`); the
automatic action replies reuse the cached value.

`QUEUED` / `QPEAK` / `QBUF` / `BUFKB` expose the RAM write buffering, and
`WRITE_MS` / `RATE_KB` / `SPIKHZ` the card's own performance: a `QPEAK` close to
512 together with a non-zero `DROPPED` means the card cannot keep up with the
sample rate.

`#BT ... peak= rate= need=` reports the radio side: `need` is what the live link
must carry at the current data rate (33 bytes per packet), `rate` what it is
actually sending, and `peak` the high-water mark of the transmit ring. Bluetooth
SPP has **no baud rate** - `btSerial.begin()` takes only the device name - so these
figures, not a serial speed, are what decide whether the link keeps up.

The OpenBCI marker command (`` ` ``) writes `Marker=1` into the next card row, so a
card recording and a live recording can be aligned on one shared instant rather
than by correlation, which is ambiguous for any periodic trace.

**Card recordings are fed by the conversion stream**, so `:b` starts the stream when
it is not running; a recording requested while stopped used to write a header and
no rows. Any host byte also counts as presence: while a card recording is active the
board restarts a stopped stream if the last host byte arrived within 50 ms, so a
dropped stream heals itself during a recording. `s` and `v` arm
`suppressAutoStream`, so a deliberate stop is never undone by the 5 s `:s` poll; a
`b` or a new recording clears it. A board with no host never self-starts.

Recordings are OpenBCI-compatible CSV at `/SZD_NNNN_<name>.CSV`, containing only
the channels that are switched on. The stored configuration is a plain text file:

```
SZDCFG 1
name=<session>
rate=<DR bits>
decim=<n>
spi=<Hz>
ref1=<0|1|2>
ref2=<0|1|2>
ch<0..11>=<active>,<gain>,<input>,<bias>,<srb2>
```

`:w` should be sent *after* the normal `x...X` / `N` / `n` / `~` / `m` / `o`
writes, so the card stores the configuration the converters are actually running.
The GPIO37 button reads that file and applies it before recording, which is what
makes a standalone recording reproduce the settings last reviewed in the GUI.

## ADS1299 register defaults applied (Cyton-like)

On startup, `d` command, and after the self test:

- `CONFIG1`: `DAISY_EN = 1` (multiple readback, since both devices share
  SCLK/DIN/DOUT with independent CS), `CLK_EN` per device role, `250 SPS`
- `CONFIG2`: internal test signal off
- `CONFIG3`: `PD_REFBUF = 1` (internal reference buffer on), `BIASREF_INT = 1`
  (bias reference from `(AVDD + AVSS)/2`), `PD_BIAS = 1` (bias amplifier on)
- `CHnSET`: gain = `x24`, input type = `NORMAL`, **SRB2 = OFF**, bias include = ON

## Channel mapping

ADS1299-4 (8-channel build):

- Channel 1..4 -> ADS1 CH1..CH4
- Channel 5..8 -> ADS2 CH1..CH4

ADS1299-6 (12-channel, 16-channel daisy emulation):

- Board channels 1..6 -> ADS1 CH1..CH6 (GUI channels 1-6)
- Daisy channels 9..14 -> ADS2 CH1..CH6 (GUI channels 9-14)
- GUI channels 7, 8, 15, 16 are zero-padded

## Packet format detail

Each sample packet (33 bytes):

1. Byte 0: `0xA0`
2. Byte 1: sample counter (`0..255`, wraps)
3. Bytes 2..25: 8 channels × 3 bytes signed 24-bit
4. Bytes 26..31: aux payload (6 bytes, accelerometer X/Y/Z)
5. Byte 32: `0xC0`

In 16-channel mode the 8 channel slots alternate: odd sample numbers carry the
board (ADS1) channels, even sample numbers carry the daisy (ADS2) channels. This
matches the official Cyton+Daisy wire format, so the stock GUI reconstructs both
halves without any parser changes.

## Practical setup checklist

1. Flash firmware and open serial monitor once to verify startup text.
2. In OpenBCI GUI select:
   - Data source: Cyton (live)
   - Transfer protocol: Serial
   - Correct COM port
   - 8-channel build: board type **Cyton** (8 channels)
   - 12-channel build: board type **Cyton + Daisy** (16 channels)
3. Start session, then stream.
4. Use GUI Hardware Settings to change gain/input/bias/SRB per channel (handled through `x...X`).

The repository also ships its own acquisition window that speaks this protocol
directly (bulk channel configuration, automatic settings replay on connect,
recording with named sections, FFT/accelerometer views and a serial monitor):
see [`docs/LIVE_RECORDER.md`](LIVE_RECORDER.md).

## EEG quality reminders

- Keep electrode impedance low and balanced.
- Use proper reference electrode and bias electrode contact.
- Keep cables short, shielded, and away from mains noise.
- Use battery/isolated power for low-noise and safety.
