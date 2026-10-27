# Live Recorder (ESP32 + dual ADS1299)

The Python application has two windows that share one process:

| Window | How to open | Purpose |
|--------|-------------|---------|
| `Scientific EEG/ADC Signal Analyzer` | `python gui_main.py` | Offline analysis of recordings |
| `Dani-BCi Live Recorder` | *Window -> Live recorder* in the analyzer, `python gui_main.py --live`, or `python live_gui.py` | Configure the board, stream, record, annotate, monitor |

"Analyze last recording" in the recorder opens the analyzer with the freshly
written CSV already loaded, so the round trip is: record -> label sections ->
analyze one section at a time.

## Quick start

```powershell
python -m pip install -r requirements.txt
python gui_main.py            # analyzer, then Window -> Live recorder
python gui_main.py --live     # start directly in the recorder
```

The **DEMO** entry in the port list is a synthetic board (no hardware) that
produces a realistic 12-channel Cyton+Daisy stream. Use it to try the UI, the
FFT, the accelerometer alarm and the recording path.

The window opens sized for the display it starts on (and remembers nothing
about size), so it fills a large monitor without exceeding it; the layout also
shrinks down to roughly 1300 px if you drag it narrow.

## Connecting

1. Pick the port and press **Connect**. With **Auto-connect on start** ticked
   (the default) the recorder opens the port used last time by itself, or
   `COM5` when nothing was used before; untick it to always start offline.
2. The recorder sends `v` (version request) and waits for the board answer.
   On a freshly opened port the ESP32 may still be running its ~6 s power-on
   self test; the request is retried until the board replies.
3. From the reply the recorder learns the ADS1299 IDs, the LIS3DH ID, the SD
   card state, the firmware version and the two link budgets. Those appear
   under the connection status and in the serial monitor.
4. The **saved** sample rate and per-channel settings are written back to the
   board automatically and every channel is confirmed individually.

If the reply shows a daisy device but the saved profile is 8 channels (or the
other way round) the profile is switched and the port re-opened once.

### Link budgets (serial and SPI)

Two independent limits decide the highest usable ADS rate, and the Hardware
settings group shows both:

| Link | Calculation | Limit at the defaults |
| --- | --- | --- |
| ESP32 → PC (serial) | 33 bytes/packet at 10 bits/byte, 10 % headroom | 314 SPS at 115200 baud, 2513 SPS at 921600 |
| ESP32 → 2x ADS1299 (SPI) | 216-bit frame per converter, both read per sample: 432 bits at 2 MHz = 216 µs | 4166 SPS |

One sample therefore costs 216 µs on the SPI bus; the bus would only become the
limiting factor above ~4 kHz per channel. The line *Limit in force* names the
tighter of the two.

* The rate list in *Hardware settings* marks rates above the link capacity and
  also shows the SPI budget.
* If a saved configuration asks for a faster rate, the recorder downgrades to
  the highest supported rate, says which link caused it, and applies the rest
  normally.
* The status bar warning *"Receiving 176 of 250 Hz"* means samples are being
  lost - lower the ADS rate or raise `SERIAL_BAUD` in the firmware and pick the
  same baud rate in the recorder.

### Verifying the board configuration

Write acknowledgements only say "the command arrived". **Verify board
configuration** reads the chips themselves:

1. The recorder sends `P`; the firmware pauses streaming, clocks registers
   `0x00..0x17` out of both converters and answers with one
   `[CFG] DEV=ADS1 LINK=1 REGS=...` line per chip plus the rate/clock summary.
2. The recorder rebuilds the register image the saved settings *should*
   produce (CHnSET = gain, SRB2, input type and the power-down bit; BIAS_SENSP/N
   one bit per bias-included channel; MISC1 bit 5 for SRB1; CONFIG1 with
   DAISY_EN, CLK_EN on the clock master and the DR bits) and compares.
3. The result is one green line - *"Configuration verified: all 12 channels and
   both converters match the saved settings"* - or a red list of the exact
   registers that differ, for example
   `ADS1 CH1SET 0x68, expected 0x08`.
4. Streaming resumes by itself; the raw report is in the serial monitor.

This catches the cases where a write silently did not stick: a stuck SPI line,
a channel that stayed powered down, a bias/SRB bit that never latched, or a
sample rate that was refused.

## Hardware settings

Settings are edited in two places:

* **Hardware settings...** opens the full table: one row per GUI channel with
  `On`, `Gain`, `Input`, `Bias`, plus a status column that shows whether the last
  write was confirmed by the firmware. The reference mode and the configuration
  check live in the same dialog.
* The compact quick row in the dock sets gain / input / bias for *all* channels in
  one click, which is the fast path for a uniform measurement. The reference mode
  is a separate one-line choice underneath it.

In the dialog every channel also has a selection checkbox, and the bulk row
("Set every selected channel to") applies to the ticked channels only. That
covers both "set all channels identically" and "keep single channel control for
mixed measurements".

| Field | Values | Notes |
|-------|--------|-------|
| Gain | x1 ... x24 | PGA gain; the host scales counts to microvolts with the same value |
| Input | Normal, Shorted, Bias Meas, MVDD, Temp, Test Sig, BIAS DRP, BIAS DRN | ADS1299 `CHnSET` MUX |
| Bias | Include / Exclude | channel participates in the bias (DRL) loop |

Board profile and ADS sample rate live at the top of the dialog. In the 16-channel
build GUI channels **7, 8, 15 and 16 are padding** (the ADS1299-6 only has six
inputs per chip); they are shown greyed out, cannot be selected, and are never
written or recorded.

Per-channel rate is the ADS rate for the 8-channel profile and **half** the ADS
rate for Cyton+Daisy, because the two converters are interleaved into one
16-channel frame.

### Reference / SRB (one setting, not per channel)

There is exactly **one** reference choice for the whole board - in the dock next
to *Apply these to all channels*, and at the top of the Hardware settings dialog:

| Mode | What the chip is programmed with | Wiring it expects |
|------|----------------------------------|-------------------|
| True differential | `MISC1` SRB1 = 0, every `CHnSET` SRB2 = 0 | each channel has its own INP **and** INN electrode; the bias/DRL electrode keeps the common mode in range |
| SRB1 | `MISC1` bit 5 = 1 on **every** converter, every `CHnSET` SRB2 = 0 | one reference electrode on the **SRB1** pin; every INN is internally tied to it |
| SRB2 | `MISC1` SRB1 = 0, `CHnSET` bit 3 = 1 on every active channel | one reference electrode on the **SRB2** pin |

Why it is not per channel: `MISC1` bit 5 is a single switch inside each
ADS1299 that connects the negative input of *all* channels at once. A
per-channel SRB1 checkbox could therefore never be honoured by the hardware - and
worse, turning it on for one channel silently moved every other channel's
reference onto the SRB1 node, which is exactly how an input ends up floating and
saturated. The recorder now derives the per-channel `CHnSET` SRB2 bit and the
`MISC1` byte from the single mode, so an inconsistent combination cannot be
written. The firmware enforces the same rule: `N<0|1|2>` is applied *after* the
per-channel writes, and `x...X` can no longer change `MISC1` at all.

If a channel still saturates (see the warning below), the mode does not match the
wiring - typically SRB2 selected while the reference electrode sits on SRB1, or a
differential setup with no bias/DRL electrode at all.

Board profile and ADS sample rate live at the top of the dialog. In the 16-channel
build GUI channels **7, 8, 15 and 16 are padding** (the ADS1299-6 only has six
inputs per chip); they are shown greyed out, cannot be selected, and are never
written or recorded.

Per-channel rate is the ADS rate for the 8-channel profile and **half** the ADS
rate for Cyton+Daisy, because the two converters are interleaved into one
16-channel frame.

### Automatic replay and verification

Every change is stored in `settings/live_recorder.json` immediately. After each
connection the recorder writes, one command at a time and waiting for the
firmware reply:

1. `~<code>` - the ADS sample rate,
2. `x<selector><6 digits>X` - one command per real channel.

A reply is `Success: ...` or `Failure: ...`. The status column shows
`written` / `rejected` / `no reply` per channel, and the dock shows a summary
such as `Saved settings written to the board: 12/12 channels confirmed.`
The measured packet rate is shown in the status bar, so a rate mismatch is
visible immediately.

Settings are never written while a recording is running; the recorder asks you
to stop first so the file keeps a single, consistent configuration.

## Live views

### Time series (left)

Each visible channel gets its **own row** with its own plot, stacked like the
OpenBCI time series panel:

* one shared time axis at the bottom of the view, in seconds since the stream
  started - it stays visible when you scroll;
* per row: the channel number badge, the **DC value** (see below), small
  `+max / +half / 0 / -half / -max` labels and grid lines at exactly those five
  levels, so a 400 uV scale shows lines at 0, +/-200 and +/-400 uV and the row
  edge *is* 400 uV;
* `Scale` sets that range for every row at once;
* `Rows` chooses how many rows fill the window (4, 6, 8, 10 or 12); with
  more channels than rows the stack scrolls with the mouse wheel;
* `Window` sets the visible time span (1 to 30 s);
* `Center` removes the mean of the visible window per channel (display only) so
  electrode offsets do not push a trace off its row;
* `Pause` freezes the traces while acquisition and recording continue.

### DC value under every channel number

Directly under each badge is the running mean of the **raw** signal with a 0.1 Hz
corner (an exponential average over roughly the last 1.6 s of samples), so it is
exactly the part the display high-pass removes: the electrode/offset potential of
that input. The number is shown on one line and its unit (`uV` / `mV`) on the line
below it, without any label in front, so it stays readable in a narrow gutter. It
is updated about 30 times per second and is coloured teal while the channel is
healthy.

A shorted input therefore reads a small value, an open electrode drifts, and a
channel pinned at `+187.500 mV` (gain x24) is sitting on the ADC rail.

### No-data and saturated-input warnings

Two failure modes are called out, each with a red banner across the top of the
time-series panel, a marked badge and a message in the dock:

* **no data (0 V)** - the samples stay at **exactly** zero for 2 s. The badge gets
  a red ring with `!` and the value reads `+0.00` with `no data` underneath.
  A shorted input is *not* this: it still produces amplifier noise. An exact run
  of zeros means the channel is powered down, the register write never reached
  the chip, or the electrode path is completely open.
* **input saturated** - every sample of the batch sits within 2 % of the ADC full
  scale (`VREF / gain` = 187.5 mV at x24) for 0.5 s. The badge gets an amber ring,
  the value turns amber and the unit line reads `saturated`. This is what a
  floating or wrongly referenced input looks like: the differential input is
  pinned at the rail, so the trace is either flat or full of rail-to-rail noise.

The banner lists the affected channels, for example

```
no data (0 V): CH 3   |   input saturated: CH 1, CH 2, CH 5
```

If several channels saturate at once, check the **reference mode** against the
wiring first - that is almost always the cause - then the bias/DRL electrode.

### Channels (dock)

The checkboxes in the dock decide what is **shown**: they hide a channel from the
time series, the FFT and the accelerometer-free part of the display. The `All` /
`None` buttons tick or untick everything, padding channels of the 16-channel
daisy emulation stay disabled (there is no ADS input behind them).

By default a recording still contains **every** channel of the board. Enable
`Record only checked channels` in the Recording group to make the CSV follow the
ticks - the label under it always spells out what the next file will contain
(for example `records 10 of 12 channels: 1, 3, 4, ...`). With nothing ticked the
option falls back to recording everything rather than producing an empty file.

### FFT (right, top)

* `Points`: 256 to 16384 samples per transform.
* `Window`: Hann, Hamming, Blackman, Rectangular.
* `Smooth`: exponential smoothing of the magnitude spectrum (Off ... 0.999).
* `Trace`: draw every channel that is ticked in the dock (`Visible`), one single
  channel (`CH 1` ... `CH 16`), or the mean of the visible channels (`Avg`).
* `Scale`: `Auto` or a fixed uVrms ceiling; this is the actual y-axis limit.
* `Log`: logarithmic amplitude axis. Each decade gets its own labelled grid line
  (`1`, `10^1`, `10^2`, ... up to the top decade of the selected scale) plus the
  faint 2..9 minor lines of a classic log grid, so `100 uVrms` shows
  `1 / 10^1 / 10^2` and `1000 uVrms` shows `1 / 10^1 / 10^2 / 10^3`. The axis is
  configured as soon as you touch the control, even with no data streaming; only
  when the whole spectrum stays below 1 uVrms does the floor follow the data
  down so nothing is lost.
* `End`: right end of the frequency axis. `Auto` follows the Nyquist limit of
  the current per-channel sample rate; 25, 50, 100, 120, 150, 200 and 250 Hz
  are fixed choices. When the chosen end is above Nyquist the bins stop at the
  dashed Nyquist line, because no real data exists beyond it.
* Faint vertical guide lines every 10 Hz, plus grid lines for both frequency
  and amplitude.
* `Pause` freezes the display.
* Hover the plot: a crosshair plus a readout inside the plot (top left) shows
  frequency, amplitude and the channel (or `avg`) the value belongs to. When the
  pointer leaves, the bottom right keeps showing the strongest peak.

### Accelerometer (right, bottom)

* Three axis bars with a movement alarm: the outline flashes red and the label
  switches to `MOVING` when the vector difference from a slow baseline exceeds
  the threshold.
* `Motion alarm` (g), `Baseline` (s) and `Full scale` (+/- 0.5 ... 4 g) are in
  the dock. The footer shows current motion, the peak since the stream started
  and the number of movement events.
* `Calibrate from rest`: with the board lying still, sets counts-per-g from the
  current reading (the firmware forwards raw LIS3DH counts).
* If the board reports zeros for more than two seconds the panel says
  *"no accelerometer data - check the LIS3DH"* instead of pretending the board
  is perfectly still.

### Filters

Dropdowns with an explicit **Off** entry:

* 50 Hz and 100 Hz notches with adjustable Q,
* Butterworth high-pass (0.1 ... 40 Hz) and low-pass (5 ... 200 Hz).

The **default band is 0.1 Hz high-pass / 150 Hz low-pass**, the same band the
analyzer is normally driven with. Existing settings files adopt this band once
(settings version 2); after that whatever you select is remembered.

Filters affect the traces and the FFT only; recordings stay raw so the offline
analyzer can apply its own zero-phase filters.

## Recording and sections

A **section** is a named time range inside one recording. It is the way to keep
"baseline", "eyes closed", "block 1", ... apart so each one can be analyzed on
its own.

1. Name the measurement with **Name...** (see below) or type a session name. It
   becomes the file name and is reused by every export.
2. **Start recording** - recording starts the stream if needed and writes
   `recordings/<YYYYmmdd_HHMMSS>_<session>.csv`. Unless `Record only checked
   channels` is enabled the file holds all real channels of the board.
3. **Start section** asks for a name (default `section N`) and opens a section at
   the current time. The dock shows `Open section: baseline (00:12.34)` and the
   duration keeps counting.
4. When the measurement condition changes, press **Start section** again (that
   closes the previous one) or **End** to close without starting a new one.
5. Select a section and press **Analyze** (or double-click it) to open the
   analyzer with the recording loaded and the selection set to exactly that
   time range. The metrics, FFT and exports then refer to that section only.
6. **Edit** changes label, note and start/end time; **Delete** removes it.
7. **Stop recording** flushes the file and writes the annotation sidecar.

Sections can still be edited after the recording stops; the sidecar is rewritten
each time.

### Naming a measurement

**Name...** next to the session field opens a builder instead of making you type
a name. Every field describes the bench setup, so the name documents itself:

| Field | Choices |
|-------|---------|
| Measurement | **Noise (input shorted to GND)**, **CMRR**, **Crosstalk**, **Gain** |
| Connection | **Electrode**, **Coax**, **IC (internal)** |
| Coax type (only for coax) | **GND**, **GND with OP**, **BIAS**, **ACTIVE** |
| Applied voltage (only for CMRR / crosstalk / gain) | the external stimulus, e.g. `1` or `0.0005` V, with an optional source note |
| Reference | **True differential**, **SRB1**, **SRB2** (pre-filled from the hardware setting) |
| Note | optional free text |

The dialog shows the resulting name live, for example:

```
cmrr_coax-gnd-op_1V_ref-srb1
noise_coax-active_ref-diff
gain_ic_2.5mV_ref-srb2
```

Those facts are written into the CSV header (`Measurement`, `Connection`,
`Reference Mode`, `Applied Stimulus`, ...), so the analyzer picks them up
automatically - you never retype them when exporting.

### Noise metric in the analyzer

The analyzer reports the input-referred noise in the band you actually measure
in: **`Band RMS 0.1-100 Hz (uVrms)`**, one value per channel (RMS of the selection
after a 0.1-100 Hz zero-phase Butterworth bandpass), plus the
**mean** row at the bottom of the metrics table (`All channels (mean)`, or
`Mean of N visible` when channels are hidden), whose cell in that column is the
single averaged 0.1-100 Hz noise figure of the selection.

A channel that is switched off in **either** the time or the FFT channel list is
excluded from that mean, so a noisy channel you are not looking at cannot move
the summary number. Use at least ~10 s of data so the 0.1 Hz corner can settle,
and analyze a section of shorted inputs to read the amplifier floor.

`Selection Avg RMS (uVrms)` is the older metric and uses a 0.5-100 Hz band; it is
still there, so the two can be compared directly.

## Exporting from the analyzer

`Export -> Export report...` (or any of the three older entries, which just
preselect a format) opens one dialog:

* **Format** - PDF report, PNG images, CSV tables. Any combination.
* **Include** - a checklist of sections: measurement & settings, waveform plot,
  FFT plot, metrics table, crosstalk table, crosstalk sweep, gain sweep, CMRR
  sweep. Sweeps are greyed out until you have computed them.
* **Measurement** - the description read from the recording header; *Edit
  measurement details...* changes it.
* **File name** - pre-filled with the recording name plus the measurement slug.

The choice is stored in `settings/analyzer.json`, so the next time you export the
same sections, formats and measurement text are already selected. The PDF is a
real report: title and metadata block, the applied stimulus and analysis settings,
figures at page width, and tables with a header band, grid lines, alternating row
shading and right-aligned numbers. Tables wider than six data columns are split
into labelled parts (1/2, 2/2) with the channel column repeated, so nothing is
ever clipped.

The page footer records when the report was generated.

## Files on disk

`recordings/20261022_160403_session.csv` - OpenBCI-style CSV:

```
%OpenBCI Raw EXG Data
%Number of channels = 12
%Sample Rate = 125 Hz
%Board = ESP32 + 2x ADS1299-6 (Cyton+Daisy emulation)
%Firmware = v3.1.5
%Port = COM5
%Session = session
%Started = 2026-10-22T16:04:03
%Channel Map = 1,2,3,4,5,6,9,10,11,12,13,14
%Board Channel Gains = x24,x24,...
%Data Format = EXG channels in microvolts, accelerometer in g, timestamp in seconds
Sample Index,EXG Channel 0,...,Accel Channel 0,Accel Channel 1,Accel Channel 2,Marker,Timestamp
1,12.3456,...,0.00000,0.00000,0.12475,0,0.000000
```

Only the **real** channels are recorded (12 columns for the ADS1299-6 build), in
GUI channel order. `Marker` is reserved and currently always `0`.

`recordings/<...>.csv.annotations.json` - metadata and sections:

```json
{
  "recording": "recordings/20261022_160403_session.csv",
  "sample_rate_hz": 125.0,
  "channel_labels": ["EXG Channel 0", "..."],
  "metadata": {"Board": "..."},
  "annotations": [
    {"label": "baseline", "start_s": 0.0, "end_s": 31.2, "note": "eyes open"}
  ]
}
```

The same annotations are appended as `%Annotation,...` comment lines at the end
of the CSV, so a single file already documents the measurement.

## Serial monitor

*Device -> Serial monitor...* opens a waterfall of every non-data message:

* boot log and power-on self test,
* `v` version replies (including the serial link limit reported by the firmware),
* `Success:` / `Failure:` replies for settings, rate and stream commands,
* everything you send, with an arrow and timestamp,
* a live `measured Hz / samples / lost` line (can be hidden).

Filters toggle info / success / error / sent / data-stat lines. The log can be
paused, cleared, saved to a text file, and the send box accepts ASCII commands
(`v`, `x1060110X`, `~6`, `b`, `s`, `d`, `?`) or hexadecimal payloads.

The monitor shows whatever the firmware prints. The shipped build runs the
standard Cyton protocol (`Success: ...$$$`); if you rebuild with
`USE_DEBUG_APP_MODE = true` the `#`-prefixed debug replies appear in the same
window, and the recorder keeps working because it understands both formats.

When you stop the stream the firmware reports `missed`, `dropped` and `corrupt`
counters; the recorder mirrors them in the status bar so frame integrity is
visible without opening the monitor.

## Saved settings

`settings/live_recorder.json` (git-ignored) stores:

* hardware: board profile, ADS rate, all per-channel fields (replayed on connect),
* connection: last port, baud rate and whether to auto-connect on start,
* display: scale, window, row height, FFT points/window/smoothing/scale/log,
  FFT trace selection and end frequency, filters, accelerometer settings,
  visible channels, record-only-checked-channels,
* recording: session name, output folder and the measurement description
  (kind, connection, stimulus, reference mode).

The analyzer keeps its own small store, `settings/analyzer.json`, holding the
export sections/formats and the measurement text.

The file carries a `version` field, so an older file adopts the newer defaults
once and then keeps your choices:

| Version | What it introduced |
|---------|--------------------|
| 2 | 0.1 / 150 Hz default band, auto-connect flag |
| 4 | 921600 baud, 500 SPS (250 Hz per channel), gain x24, normal input, SRB2, FFT log + auto scale, record only checked channels |

Delete the file to return to defaults.

### Boot defaults

A fresh install (and any file older than version 4) starts like this, because
this is the setup the amplifier is normally used in:

* **COM5, 921600 baud, auto-connect on.** The app scans the available ports,
  opens the remembered one (or COM5) as soon as it appears and keeps scanning
  every 2.5 s while it is disconnected, so plugging the board in later is enough.
* **500 SPS** from the ADS1299, which is **250 Hz per channel** in Cyton+Daisy
  mode. Gain **x24**, input **normal**, reference **SRB2**, 0.1 Hz high pass and
  150 Hz low pass enabled.
* **FFT** on the log amplitude axis with automatic full-scale tracking,
  **Time series** showing all 12 real channels.
* **Record only checked channels** enabled, so unticking a channel in the dock
  keeps it out of the recording (and out of the FFT average).
* Nothing is written to the board until you press **Write to board**, but the
  saved settings are replayed automatically the moment a connection is made, and
  the result of every write is shown as `confirmed` / `problem` per channel.

If a requested rate does not fit the link (see
[Link budgets](#link-budgets-serial-and-spi)) the app writes the fastest rate
that does fit and says so, instead of silently dropping to the slowest option.

### Scrolling cannot change a setting by accident

Scrolling a long panel used to be able to change whatever combo box or spin box
happened to be under the pointer. Now an unfocused value widget ignores the wheel
and lets the panel scroll instead; the value only responds to the wheel once you
have clicked it (so it has focus). This applies to both windows.

### Recording name and reference mode stay in sync

The reference mode is one hardware setting, and the measurement name records it,
so the two are kept equal in both directions:

* Choosing a name with a reference (**Name...** -> Reference) writes that
  reference into the hardware settings and sends it to the board, so a
  `ref-srb1` name always means the board really is in SRB1.
* Changing the reference in **Hardware settings** renames the current session
  automatically (`..._ref-srb1`), so a file cannot be labelled with a reference
  it was not recorded with.

## Firmware notes that matter for this app

| Command | Meaning |
|---------|---------|
| `v` | version / device inventory (also stops streaming and re-applies defaults) |
| `~<0-6>` | ADS data rate (16000 ... 250 SPS, rejected above the serial or SPI limit) |
| `x<selector><digits>X` | per-channel power/gain/input/bias (the last two digits are accepted but SRB is owned by `N`) |
| `N<0\|1\|2>` | board-wide reference mode: 0 = differential, 1 = SRB1, 2 = SRB2 |
| `b` / `s` | start / stop streaming |
| `d` | firmware defaults for all channels |
| `?` | register dump (serial monitor) |
| `P` | configuration readback for **Verify board configuration** (`[CFG]` lines) |

Selectors are `1`-`8` for the board channels and `Q W E R T Y U I` for channels
9-16 of the daisy half.

Three firmware behaviours exist specifically to keep the data spike-free:

1. **The full RDATAC frame is clocked** (status + 8 channel slots, 216 bits) even
   on a 6-channel part. Stopping after 6 channels leaves the device mid-frame
   and a read that crosses a conversion reload returns bit-shifted values.
2. **Torn frames are rejected**: the acquisition task compares the DRDY counter
   before and after each SPI read. If a conversion started while the frame was
   being clocked, the frame is discarded and re-read once, and `corrupt` is
   incremented instead of publishing a spike.
3. **Rates above the serial or SPI link capacity are refused** by `~`, so packets
   are never queued faster than the UART can drain them and both converters can
   still be read inside one sample period.

The SPI clock is `ADS_SPI_CLOCK_HZ` in `src/main.cpp` (currently 2 MHz); raising
it to 4 MHz is allowed by the ADS1299 datasheet and would double the SPI budget
to ~8300 SPS, but the serial link becomes the binding limit long before that.

## Troubleshooting

| Symptom | What to check |
|---------|---------------|
| Single-sample spikes on all channels | Those were torn RDATAC frames. Reflash the current firmware and keep the ADS rate within the link budget; check the `corrupt` counter after stopping the stream |
| "No data from CH n" warning | That channel reads exactly zero: press **Verify board configuration** and look for the CHnSET mismatch, or check that the channel is powered on / the electrode path is connected |
| "input saturated" warning | The differential input is at the ADC rail. Check that the reference mode matches the wiring (SRB2 selected but the reference on SRB1 is the classic mistake), then the bias/DRL electrode and the gain |
| Channel values frozen at one number | The firmware used to hold the last sample when a converter stopped answering. It now publishes zeros after four missed frames, so a dead SPI/DRDY path shows up as "no data" instead |
| "Configuration mismatch" after verify | The listed registers did not stick. A whole chip missing points at CS/SPI wiring; one channel points at that channel's input or power bit |
| "The board did not answer the version request" | wrong port, port already open in another program, or the board is still running the self test |
| Some channels show `rejected` | those channels do not exist on the fitted chip (padding), or the firmware refused the value |
| Traces flat / all zeros | input type set to `Shorted`, or channels powered down (`On` unchecked) |
| Accelerometer reads 0.000 g | LIS3DH not answering - the panel shows a note after two seconds |
| Accelerometer never alarms | press `Calibrate from rest`, then lower `Motion alarm` |
| Recording has fewer channels than expected | the CSV stores only real channels; padding channels are intentionally dropped |
| Rate drops back to 250 SPS | the selected rate needs more bandwidth than the serial link provides |
| Analyzer uses the wrong sample rate | the file header is read first; the import dialog shows the detected rate and lets you override it |