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

1. Pick the port and press **Connect**.
2. The recorder sends `v` (version request) and waits for the board answer.
   On a freshly opened port the ESP32 may still be running its ~6 s power-on
   self test; the request is retried until the board replies.
3. From the reply the recorder learns the ADS1299 IDs, the LIS3DH ID, the SD
   card state and the firmware version. Those appear under the connection
   status and in the serial monitor.
4. The **saved** sample rate and per-channel settings are written back to the
   board automatically and every channel is confirmed individually.

If the reply shows a daisy device but the saved profile is 8 channels (or the
other way round) the profile is switched and the port re-opened once.

### Serial link budget

One packet is 33 bytes on the wire, so the ADS rate is also the packet rate.
At 115200 baud the link carries about **314 packets/s** with the headroom the
firmware keeps, which means **250 SPS is the highest usable ADS rate** over the
default serial link (Cyton+Daisy then runs at 125 Hz per channel).

* The rate list in *Hardware settings* marks rates above the link capacity.
* If a saved configuration asks for a faster rate, the recorder downgrades to
  the highest supported rate, says so in the Hardware settings status and in
  the serial monitor, and applies the rest normally.
* The status bar warning *"Receiving 176 of 250 Hz"* means samples are being
  lost - lower the ADS rate or raise `SERIAL_BAUD` in the firmware and pick the
  same baud rate in the recorder.

## Hardware settings

Settings are edited in two places:

* **Hardware settings...** opens the full table: one row per GUI channel with
  `On`, `Gain`, `Input`, `Bias`, `SRB2`, `SRB1`, plus a status column that shows
  whether the last write was confirmed by the firmware.
* The compact quick row in the dock sets the five fields for *all* channels in
  one click, which is the fast path for a uniform measurement.

In the dialog every channel also has a selection checkbox, and the bulk row
("Set every selected channel to") applies to the ticked channels only. That
covers both "set all channels identically" and "keep single channel control for
mixed measurements".

| Field | Values | Notes |
|-------|--------|-------|
| Gain | x1 ... x24 | PGA gain; the host scales counts to microvolts with the same value |
| Input | Normal, Shorted, Bias Meas, MVDD, Temp, Test Sig, BIAS DRP, BIAS DRN | ADS1299 `CHnSET` MUX |
| Bias | Include / Exclude | channel participates in the bias (DRL) loop |
| SRB2 | On / Off | per-channel SRB2 switch |
| SRB1 | On / Off | shared SRB1 bus of each ADS1299 |

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
* per row: the channel number badge, small `+max / +half / 0 / -half / -max`
  labels and grid lines at exactly those five levels, so a 400 uV scale shows
  lines at 0, +/-200 and +/-400 uV and the row edge *is* 400 uV;
* `Scale` sets that range for every row at once;
* `Rows` chooses how many rows fill the window (4, 6, 8, 10 or 12); with
  more channels than rows the stack scrolls with the mouse wheel;
* `Window` sets the visible time span (1 to 30 s);
* `Center` removes the mean of the visible window per channel (display only) so
  electrode offsets do not push a trace off its row;
* `Pause` freezes the traces while acquisition and recording continue.

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

Filters affect the traces and the FFT only; recordings stay raw so the offline
analyzer can apply its own zero-phase filters.

## Recording and sections

A **section** is a named time range inside one recording. It is the way to keep
"baseline", "eyes closed", "block 1", ... apart so each one can be analyzed on
its own.

1. Type a session name (it becomes part of the filename).
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
* connection: last port and baud rate,
* display: scale, window, row height, FFT points/window/smoothing/scale/log,
  FFT trace selection and end frequency, filters, accelerometer settings,
  visible channels, record-only-checked-channels,
* recording: session name and output folder.

Delete the file to return to defaults.

## Firmware notes that matter for this app

| Command | Meaning |
|---------|---------|
| `v` | version / device inventory (also stops streaming and re-applies defaults) |
| `~<0-6>` | ADS data rate (16000 ... 250 SPS, rejected above the link capacity) |
| `x<selector><digits>X` | per-channel power/gain/input/bias/SRB2/SRB1 |
| `b` / `s` | start / stop streaming |
| `d` | firmware defaults for all channels |
| `?` | register dump (serial monitor) |

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
3. **Rates above the serial link capacity are refused** by `~`, so packets are
   never queued faster than the UART can drain them.

## Troubleshooting

| Symptom | What to check |
|---------|---------------|
| Single-sample spikes on all channels | Those were torn RDATAC frames. Reflash the current firmware and keep the ADS rate within the link budget; check the `corrupt` counter after stopping the stream |
| "The board did not answer the version request" | wrong port, port already open in another program, or the board is still running the self test |
| Some channels show `rejected` | those channels do not exist on the fitted chip (padding), or the firmware refused the value |
| Traces flat / all zeros | input type set to `Shorted`, or channels powered down (`On` unchecked) |
| Accelerometer reads 0.000 g | LIS3DH not answering - the panel shows a note after two seconds |
| Accelerometer never alarms | press `Calibrate from rest`, then lower `Motion alarm` |
| Recording has fewer channels than expected | the CSV stores only real channels; padding channels are intentionally dropped |
| Rate drops back to 250 SPS | the selected rate needs more bandwidth than the serial link provides |
| Analyzer uses the wrong sample rate | the file header is read first; the import dialog shows the detected rate and lets you override it |