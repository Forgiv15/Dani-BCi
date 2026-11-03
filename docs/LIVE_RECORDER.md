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

### Connecting over Bluetooth instead of the cable

The board exposes a classic-Bluetooth serial port called **SZD_EEG** next to the
USB one, so the same recorder works untethered.

1. On the board: nothing to do. Bluetooth starts at boot alongside USB. Touch
   **GPIO32** once to switch to Bluetooth; the status LED then blinks **four
   times faster** than in USB mode, and touching it again returns to automatic.
2. On the PC: **Settings -> Bluetooth & devices -> Add device** -> *SZD_EEG*.
   Windows creates *Standard Serial over Bluetooth link* ports (usually two;
   either works). No MAC address is needed and nothing has to be typed on the
   board.
3. In the recorder, pick that port - it is labelled
   `(pair with SZD_EEG, then use this port)` - and press **Connect**.

The board listens on **both** transports at boot. Whichever one delivers the
first byte becomes the active one, so unplugging the cable and pressing Connect
over Bluetooth just works, and plugging the cable back in switches back with no
command.

The baud rate is irrelevant over Bluetooth (it is a virtual COM port); what
matters is throughput. At 500 SPS the stream is ~165 kbit/s, about a third of
what SPP handles comfortably. If the status bar shows lost packets on the radio,
either lower the ADS rate to 250 SPS or raise the buffer with `Q<kB>Q` (see
below).

#### When it pairs but nothing works

This is the most common Bluetooth failure, and it is a Windows/driver behaviour
rather than a broken board:

* **Two ports appear, and only one carries data.** Windows creates a paired
  "incoming" and "outgoing" *Standard Serial over Bluetooth link*. Opening the
  incoming one succeeds but nothing flows. If a Bluetooth port opens and the
  board never answers, **try the other Bluetooth port** - the recorder now says
  so explicitly instead of "check the port".
* **Windows says "Connected" after pairing.** That only means the radio link
  (ACL) is up. It says nothing about the serial channel, so do not use it as a
  health indicator; watch the `BT` segment in the recorder's status bar instead.
* **A stale pairing.** If the board is reset or the link is dropped mid-session,
  Windows can keep a dead device record and every later attempt fails until it is
  removed: **Settings -> Bluetooth & devices -> SZD_EEG -> Remove device**, then
  pair again. The firmware also repairs the matching ESP32-side state by itself
  (see below), so a re-pair is usually enough.

The firmware supervises its own SPP link, because the Arduino Bluetooth library
keeps exactly one client handle and will refuse every later connection while
that handle is still set - which is precisely what "pairs once, then never
connects again" looks like from the PC. On a stale or non-draining link it
disconnects the dead client, and if the handle does not clear it restarts the
Bluetooth stack. Every step is reported on the **USB** port:

```
[BOOT] reset reason=1 (power-on), heap 234512 bytes
[BT] SPP 'SZD_EEG' ready, tx power request 4 dBm (accepted), heap 141832 bytes
[BT] open handle=1 status=0
[BT] close handle=1 stale=0
[BT] open handle=2 status=0 (rejected: a stale client was still tracked)
[BT] connection state needs repair (opens=2 closes=1 rejected=1 fail=0) - disconnecting the stale client
[BT] connection cleared, waiting for the host to reconnect
```

To read that log while Bluetooth is the broken thing, just connect the USB cable
and open the port: the board keeps both transports alive, and the `[BT]` lines
are printed to the cable only, so they never disturb the radio stream.

`[BOOT] reset reason` is the other answer you will want: if it does **not** say
`power-on` after you re-plug, the board rebooted, and the reason names the cause
(`PANIC - firmware crashed`, `BROWNOUT - the supply sagged`, `task watchdog`).
A brownout is worth knowing on a board that also drives two ADS1299s.

The `#BT` telemetry carries the same counters, which the recorder shows as
`stale drops N` when connections were rejected and `write retries N` when the
link stopped draining.

#### Reading the board's Bluetooth log in the app

**Device -> Bluetooth report...** asks the board for its state and shows it in one
window. It works over the **USB cable**, which is the point: the board reports
Bluetooth on both links, so the report is readable exactly when the radio is the
thing that is failing.

```
Mode: bt   Client attached: yes
Confirmed pairs: opens 1, closes 1, closed by the PC 1, rejected 0
Board boots 1, uptime 428s   free heap 141832 bytes
Stalled writes 0, buffer 2145/24576 bytes, sent 90210, dropped 0
Name SZD_EEG, TX power 4 dBm

Recent events (newest last):
  boot=1 reset=1(power-on)
  ready txdbm=4 power=ok heap=141832
  spp server started
  open handle=1 status=0
  close handle=1 remote=1 status=0 stale=0
```

How to read it:

| Field | Meaning |
|-------|---------|
| `remote=1` on a close | **The PC closed the link.** Points at Windows, the pairing or the driver - not at the firmware. |
| `remote=0` on a close | The board closed it, usually the stale-handle repair described above. |
| `rejected` | A second client arrived while a dead one was still tracked. The board repairs this itself. |
| `boots` | Increments on every reset and lives in RTC memory, so it survives a reset but not a power cycle. **If it grows while you connect over Bluetooth, the board is restarting** - read the `[BOOT]` reason next. |
| `uptime` | Seconds since that reset. A value that drops back to a few seconds during a session confirms a restart. |
| `heap` | Free heap. A number in the low thousands after a connection attempt means the stack is running out of memory. |
| `Stalled writes` | SPP writes that could not be queued. Rising fast means the link is not draining. |

The same information is printed on the USB port as `[BOOT]` and `[BT]` lines and
appears in the serial monitor; `B` in the monitor replays the event history and
`T` prints the summary.

#### Bluetooth debugging checklist (Windows 11)

1. **Is the port the right one?** PowerShell:
   `Get-PnpDevice -Class Ports | Format-Table Name,Status,FriendlyName`.
   Both Bluetooth ports show up; try the other one.
2. **Did the board reboot?** Reconnect over USB and read `[BOOT] reset reason`.
3. **Is the radio up at all?** Over USB, send `T` in the serial monitor. The
   answer includes `client=`, `opens=`, `closes=`, `rejected=`, `writefail=` and
   `heap=`. `client=1` with `rejected=0` means the ESP32 has a live client.
4. **Is the PC holding a phantom connection?** Remove the device in Settings,
   pair again, then open the port once.
5. **Interference.** Bluetooth and Wi-Fi share 2.4 GHz. A weak link shows up as
   climbing `write retries` and buffer fill rather than a clean disconnect.
6. **Power.** Two ADS1299 boards plus the radio draw more than USB alone may
   supply on a marginal hub; a brownout restarts the whole board and takes
   Bluetooth with it.

**Link health** is shown in the status bar while a Bluetooth host is connected:

```
BT PER 0.00% quality 100.0% buffer 12% 4 dBm crc 0x1A2B
```

| Field | Meaning |
|-------|---------|
| `PER` | Packet error rate: packets the board had to drop / packets sent. This is the number to watch - it is measured from the packet counter, so it counts real losses. |
| `quality` | `100 - PER`, a percentage that is easier to read at a glance |
| `buffer` | Fill level of the radio ring buffer. If this climbs, the link cannot keep up. |
| `dBm` | Configured BR/EDR TX power ceiling (`SZD_BT_TX_POWER_DBM`, default 4) |
| `crc` | CRC-16 over the last 32 packets. The PC recomputes it from what it received, so a value that stops matching means corruption rather than loss. |

RSSI itself is not reported: ESP-IDF exposes classic-Bluetooth RSSI only through
the GAP callback that the Arduino BluetoothSerial library owns, and taking that
over would break pairing. PER and the buffer fill describe the same problem from
the side you can act on.

The board learns how big its radio buffer should be with `Q<kbytes>Q`, e.g.
`Q32Q` for 32 kB (2-64 kB, default 24 kB). A bigger buffer rides out longer
stalls at the cost of latency: at 500 SPS, 33 bytes every 2 ms, so 32 kB is
about 2 s. Nothing is ever sent truncated - if the buffer would overflow, the
oldest whole packet is dropped and counted, because half a sample frame would
corrupt the stream instead of just leaving a gap.

### Keeping the acquisition on time

The converters raise one DRDY edge per conversion and the firmware must read both
of them inside that period. Missing one is silent, so the work is arranged so it
cannot happen:

| Core | Runs | Why |
|------|------|-----|
| **1** | `adsStream` (priority 22) | Nothing else. The DRDY interrupt is attached from `setup()`, which runs on this core, so the semaphore hand-off never crosses cores |
| **0** | Bluetooth flush, SD writer, accelerometer, and the Bluetooth radio stack itself | Everything that can block or belongs to the radio |

This matters because the ESP32's Bluetooth controller interrupts and the
Bluedroid `BTU`/`BTC` tasks live on core 0, and `BTU` sits at the same priority a
naive acquisition task would use. Sharing core 0 with the radio delays the
acquisition by whole milliseconds at a time, which at 500 SPS is a missed
conversion - and the resulting holes in the daisy pair are what the recorder
shows as a ragged or stepped trace.

Three other things were removed from the acquisition path for the same reason:

* the accelerometer mutex is taken with **timeout 0**, so a slow I2C read can
  never delay a conversion (the 25 Hz accelerometer values are cached instead);
* the CRC over the radio stream is computed by the flush task, not per packet in
  the acquisition loop;
* the **touch pad is polled once every two seconds while streaming** (50 ms when
  idle). It is only useful before a stream starts, and `touchRead()` costs
  milliseconds.

The `#DX` line makes all of this measurable instead of assumed:

```
#DX ch_rate=250.0 pkt_rate=500.0 drdy_s=1000.0 pkts=5001 miss=0 late=0 gap_max_us=2004
    corrupt=0 spi_us=110 spi_max_us=124 usb_free=3900 bt_buf=0 core=1 prio=22
```

| Field | Meaning |
|-------|---------|
| `late` | Conversions that arrived more than 1.5 periods after the previous one, measured with the microsecond clock. **This is the number that says whether the acquisition task is being delayed.** Any non-zero value here is a scheduling problem, not a converter problem. |
| `gap_max_us` | Worst gap between two conversions. Compare with the period (`2000` us at 500 SPS); a value near or above it explains a rate shortfall directly. |
| `miss` | Conversions lost entirely (synchronisation lost for a whole period). |
| `corrupt` | Frames rejected because a conversion boundary crossed the SPI read. |
| `core`/`prio` | Where the acquisition task is actually running, so the layout above can be checked on the board rather than trusted. |

### Finding where samples are lost

The status bar carries the board's own numbers next to the link's:

```
connected | streaming | 12 480 samples | 222.6 Hz | board 222.0 Hz missed 84 corrupt 12 | BT PER 0.00% ...
                                                ^^^^^^^^^^^^^ the ESP32's own count
```

`board <n> Hz` is what the **converter edges** say the ESP32 actually produced,
measured from its DRDY counters - not what this window managed to read. Comparing
it with the received rate is the only way to tell "the ESP32 is not keeping up"
from "the link is dropping data", because the two need opposite fixes:

| Received | Board | Meaning | What to do |
|----------|-------|---------|------------|
| 222 Hz | **222 Hz** | The converters are not being read fast enough | Look at `[BOOT]`/`[BT]` lines, the input wiring and the supply. If `SPI <n>/2000 us (too slow)` appears, the SPI reads are running out of time - lower the ADS rate |
| 222 Hz | **250 Hz** | The board produced full rate; samples were lost in transit | Over Bluetooth: lower the ADS rate or enlarge the buffer (`Q48Q`). Over USB: check the baud rate and the cable |
| 250 Hz | 250 Hz | Everything is fine | - |

The same figures are in the 1 Hz `#DX` line on the wire and in **Device ->
Bluetooth report**, which also shows missed conversions, torn frames and the
worst SPI read time in microseconds against the sample period.

`corrupt` counts frames rejected because a conversion boundary crossed the SPI
read - the guard that prevents spikes. A few are normal; a steady climb means the
SPI timing is marginal.

### Link budgets (serial and SPI)

Two independent limits decide the highest usable ADS rate, and the Hardware
settings window shows both:

| Link | Calculation | Limit at the defaults |
| --- | --- | --- |
| ESP32 → PC (serial) | 33 bytes/packet at 10 bits/byte, 10 % headroom | 314 packets/s at 115200 baud (**with decimation `/2`**) ... 2513 packets/s at 921600 |
| ESP32 → 2x ADS1299 (SPI) | 216-bit frame per converter, both read per sample: 432 bits at 2 MHz = 216 µs | 4166 SPS |

The serial limit applies to the **decimated packet rate** (data rate / decimation),
not to the converter's data rate, which is why 4000 SPS with `/8` fits comfortably
through a 921600 baud link. The SPI limit applies to the raw data rate. The line
*Limit in force* names the tighter of the two.

* The dialog shows the resulting packet rate, per-channel rate and sinc3 corner
  live, and marks the line red when the combination does not fit.
* If a saved configuration is over budget, the recorder raises SCLK, then the
  decimation, then lowers the data rate, and reports what it changed.
* The status bar warning *"Receiving 176 of 250 Hz"* means samples are being
  lost - lower the data rate or raise `SERIAL_BAUD` in the firmware and pick the
  same baud rate in the recorder.

### Verifying the board configuration

Write acknowledgements only say "the command arrived", and a configuration can
also be lost later (a failed write, a converter that was not linked when the
settings were replayed, a stray reset). Two independent checks exist:

**Repair and verify** (`>` on the wire) is the authoritative one, because the
firmware owns the register image:

1. the firmware re-programs CONFIG1/2/3, every `CHnSET`, both bias masks and
   `MISC1` of both converters from its cached state,
2. it reads all 24 registers back and compares them against what the cached state
   implies,
3. it answers with `[CHK] DEV=ADS1 ID=3D NCH=6 REGOK=12 CHECKED=12` per converter
   plus `[CHK] CODE=... PROBLEMS=0 REASON=host DONE=1`, and one
   `[CHK] BAD DEV=ADS1 REG=CH1SET GOT=61 WANT=60` line per register that is still
   wrong.

The same re-programming runs automatically at every stream start, so a broken
configuration can never be streamed as valid-looking data.

**Read board state** (`P`) only reads - it is the readback path used for the
*Board reports:* line in the dock, and for comparing against the app's saved
settings (register image rebuilt in `expected_device_registers()`).

```text
Board reports: 500 SPS, /1, 500 packets/s, SCLK 2000 kHz, ref srb2
ADS1: ref srb2, CONFIG1 0xD5, MISC1 0x00
ADS2: ref srb2, CONFIG1 0xD5, MISC1 0x00
```

`ID=3D` is correct for the ADS1299-6 (`3C`/`3D`/`3E` are the -4/-6/-8 parts), so
that value alone is not a fault.

## Hardware settings

Settings are edited in one place: the **Hardware settings...** popup. The dock
only *shows* them:

* **Hardware settings...** opens the popup: board profile, data rate, decimation,
  SPI clock, a reference preset row per converter, and a table with one row per
  GUI channel (`On`, `Gain`, `Input`, `Bias`, `Reference`) plus a status column
  that shows whether the last write was confirmed by the firmware. The channel
  table is the master setting - exactly what is sent. The configuration check
  lives in the same window.
* **Re-apply saved** pushes everything the app remembers back to the board: SCLK,
  decimation, data rate, every channel and both reference modes. It is the
  recovery button when the board was reset, or when a write failed halfway.
* **Read board state** asks the board what it is actually running.

In the popup every channel also has a selection checkbox, and the bulk row
("Set every selected channel to") applies to the ticked channels only. That
covers both "set all channels identically" and "keep single channel control for
mixed measurements".

| Field | Values | Notes |
|-------|--------|-------|
| Gain | x1 ... x24 | PGA gain; the host scales counts to microvolts with the same value |
| Input | Normal, Shorted, Bias Meas, MVDD, Temp, Test Sig, BIAS DRP, BIAS DRN | ADS1299 `CHnSET` MUX |
| Bias | Include / Exclude | channel participates in the bias (DRL) loop |
| Reference | SRB2 pin / Differential (own INN) / SRB1 (whole converter) | per-input `CHnSET` bit 3, or the chip-wide `MISC1` switch; see below |

Board profile and ADS sample rate live at the top of the dialog. In the 16-channel
build GUI channels **7, 8, 15 and 16 are padding** (the ADS1299-6 only has six
inputs per chip); they are shown greyed out, cannot be selected, and are never
written or recorded.

Per-channel rate is the ADS rate for the 8-channel profile and **half** the ADS
rate for Cyton+Daisy, because the two converters are interleaved into one
16-channel frame.

### Reference / SRB (the channel table is the master)

The two reference pins are **not** interchangeable, and getting them the wrong way
round is the single most common reason a referenced channel reads as saturation or
as pure floating-input noise:

| Pin | Register | Electrically |
|-----|----------|--------------|
| **SRB1** | `MISC1` bit 5, one switch for the whole converter | ties the **inverting** input (INxN) of **every** channel to the SRB1 pin |
| **SRB2** | `CHnSET` bit 3, one switch per channel | ties that channel's **non-inverting** input (INxP) to the SRB2 pin |

The datasheet's *Referential montage* section describes the SRB1 case directly:
put the reference electrode on SRB1, every active electrode on the INxP pins, and
each channel measures its own electrode against the shared reference.

**SRB2 is not a negative reference.** Closing `CHnSET` bit 3 drives INP from the
SRB2 pin; the channel then measures *(SRB2 pin) - (whatever is on INN)*. It is
useful for driving a common stimulus into every positive input (a common-mode
voltage, for instance), but if you wire a reference electrode to SRB2 and expect a
referential montage, INN is left open and the input floats.

Each channel row therefore offers three states, and **that table is what is
written**:

| Choice in the row | Register | Meaning |
|-------------------|----------|---------|
| `SRB1 pin -> all INxN (reference)` | `MISC1` bit 5 = 1 | the reference electrode feeds every channel's INN of that converter |
| `Differential (own INN)` | `CHnSET` bit 3 = 0 | the channel uses its own INP and INN pair |
| `SRB2 pin -> this INxP` | `CHnSET` bit 3 = 1 | the SRB2 pin drives this channel's INP (it still needs INN wired) |

`SRB1` is chip-wide, so picking it on any row moves every powered-up input of that
converter, and picking another state on any of them releases it. `SRB2` and
differential can be mixed freely inside one converter. The **Reg** column shows the
resulting `CHnSET` byte (`0x60` = own INN at gain 24, `0x68` = SRB2 switch closed).

The window also states the two configurations that are guaranteed to float, because
neither is visible in a register comparison:

* any input with the SRB2 switch closed, since INP is then a driven pin and the
  measurement still depends on INN being wired, and
* any referenced routing with **no input feeding the bias/DRL amplifier** - without
  that DC return path the input charges to a rail and looks like a floating
  electrode.

Both appear in the hardware window, in the dock, and as `Reference Warning` in the
recording header.

### The reference channel is always outside the bias loop

The channel selected as the **Reference channel** in the dock is excluded from the
bias/DRL circuit automatically: it carries the stimulus (or a shared reference
electrode), so including it would drive the DRL electrode with the voltage under
test - which during a CMRR sweep is exactly the signal being characterised.

This is a rule, not a preference:

* selecting a reference clears its bias flag and restores bias on the previous one,
* the Bias cell for that row is disabled in the hardware settings window and
  `collect()` forces it back to *Exclude* even if something sets it,
* `apply_hardware()` re-applies the rule on every write path (dialog apply,
  re-apply saved, auto-connect, and a sweep), and
* it is enforced when a settings file is loaded, so a file saved by an earlier
  version cannot start up with bias on the reference input.

### Reference presets (bulk fill)

Above the table there is one preset per converter plus **Set both converters**.
These do **not** program anything by themselves: they fill the Reference column of
every powered-up input of that converter, so a whole chip can be set with one
click and then adjusted row by row. The table stays the single source of truth for
what is sent, which is why the register preview always matches the row.

| Preset | Fills the rows with |
|--------|---------------------|
| True differential | `Differential (own INN)` for every input of that converter |
| SRB1 | `SRB1 pin -> all INxN (reference)` for every input of that converter |
| SRB2 | `SRB2 pin -> this INxP` for every input of that converter |

The second converter's preset and button are disabled on a single-chip (8-channel)
board profile.

| Resulting converter | What the chip is programmed with | Wiring it expects |
|------|----------------------------------|-------------------|
| all rows `Differential` | `MISC1` SRB1 = 0, every `CHnSET` SRB2 = 0 | each channel has its own INP **and** INN electrode; the bias/DRL electrode keeps the common mode in range |
| any row `SRB1` | `MISC1` bit 5 = 1 on that converter, every `CHnSET` SRB2 = 0 | reference electrode on the **SRB1** pin, active electrodes on INxP |
| `SRB2` and/or `Differential` rows | `MISC1` SRB1 = 0, `CHnSET` bit 3 per input | for the `SRB2` rows the **SRB2 pin drives INxP**; INN still has to be wired. The other rows use their own INP/INN |

The two converters are independent. That is what makes a direct CMRR measurement
possible: run **one input of the converter that stays differential** straight
across the common-mode voltage, and it measures the applied stimulus as a real
differential signal - so the generator frequency and amplitude are always known,
even at 5 Hz where the differential leakage on the other channels is far below the
mixing products. The second converter is then free to run SRB1 for the actual test.

`SRB1` clears every SRB2 bit on that converter on purpose: with both switch sets
closed the channel would measure one reference pin against the other.

The two converters are independent. That is what makes a direct CMRR measurement
possible: run **one input of the converter that stays differential** straight
across the common-mode voltage, and it measures the applied stimulus as a real
differential signal - so the generator frequency and amplitude are always known,
even at 5 Hz where the differential leakage on the other channels is far below the
mixing products. The second converter is then free to run SRB1 for the actual test.

Firmware commands:

* `n<converter><0|1|2>` - reference mode of one converter (`n10` = ADS1 true
  differential, `n21` = ADS2 SRB1), derived from that converter's rows.
* `N<0|1|2>` - both converters at once.
* `>` - **repair and verify** (see below).

The mode is always applied *after* the per-channel writes, and `x...X` can never
change `MISC1`, so an inconsistent combination cannot be left on the chip.

If a channel still saturates (see the warning below), the mode does not match the
wiring - typically SRB2 selected while the reference electrode sits on SRB1, or a
differential setup with no bias/DRL electrode at all.

### Data rate, decimation and SPI clock

The ADS1299 third-order sinc filter is **-3 dB at 0.262 x the output data rate**,
so at 500 SPS the passband ends at 131 Hz - right on top of a 100 Hz measurement
band. Two settings in the Hardware settings window change that:

| Control | Effect |
|---------|--------|
| Data rate | ADS1299 `DR` bits: 250 ... 16000 SPS. 2000 SPS puts the -3 dB point at 524 Hz, 4000 SPS at 1048 Hz |
| Decimation (`m<n>m`) | The firmware averages `n` conversions into one published packet. `/8` at 4000 SPS gives 500 packets/s and 250 Hz per channel |
| SPI clock (`o<kHz>o`) | SCLK for both converters. Two 216-bit frames per sample must fit inside one sample period: 4000 SPS needs ~1.73 Mbit/s, so 2 MHz is the minimum and 4 MHz leaves real headroom |

Raising the data rate and averaging back down keeps the host packet rate, the CSV
sample rate and the SPI/airtime budget where they were, while moving the ADC's
analog corner out of the band. The dialog shows the resulting packet rate, the
per-channel rate, the sinc3 corner, the effective noise bandwidth and - when
decimation is on - the average's own -3 dB point and null.

**Decimation does not lower noise inside the measurement band.** An average of N
samples at `f_s` removes only what is above roughly `0.443 * f_s / N`: at
4000 SPS that is 221 Hz for `/8`, 443 Hz for `/4`. The noise *below* that corner
is what the ADC already left there, and at a higher data rate the ADC's noise
bandwidth (`1.22 x 0.262 x f_s`) is wider, so `4000 SPS /8` is about 2 dB noisier
than `500 SPS /1`. Decimation is for **bandwidth**, not for noise; for the lowest
in-band noise stay at 500 SPS or lower. The real noise win in a sweep comes from
averaging the stable part of each frequency hold, which the analyzer already does.

If a saved configuration is over budget when it is written, the recorder tries to
fix it in this order and says what it did: raise SCLK, then raise the decimation,
then lower the data rate.

Board profile, data rate, decimation and SPI clock all live at the top of the
dialog. In the 16-channel build GUI channels **7, 8, 15 and 16 are padding** (the
ADS1299-6 only has six inputs per chip); they are shown greyed out, cannot be
selected, and are never written or recorded.

Per-channel rate is the decimated packet rate for the 8-channel profile and
**half** of it for Cyton+Daisy, because the two converters are interleaved into
one 16-channel frame.

### Automatic replay and verification

Every change is stored in `settings/live_recorder.json` immediately. After each
connection the recorder writes, one command at a time and waiting for the
firmware reply:

1. `o<kHz>o` - the SPI clock,
2. `m<n>m` - the decimation factor,
3. `~<code>` - the ADS data rate,
4. `x<selector><6 digits>X` - one command per real channel,
5. `n<converter><code>` - the reference mode of each converter, last.

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

Both checks apply **only to channels that are ticked** in the dock. Unticking an
input stops its state from being tracked and immediately clears any badge or
banner line it had, because an input the operator hid is often one they left
unconnected on purpose - and a warning about it would bury the ones that matter.
Tick it again and it starts from a clean count instead of instantly reporting a
stall it accumulated while hidden.

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
| Bias (only for CMRR / crosstalk / gain) | **On** or **Off**, pre-filled from whether any channel currently has bias connected |
| Note | optional free text |

The dialog shows the resulting name live, for example:

```
cmrr_coax-gnd-op_1V_ref-srb1_bias-on
noise_coax-active_ref-diff
gain_ic_2.5mV_ref-srb2_bias-off
```

Bias is recorded because the bias network loads the input: a CMRR figure taken
with bias on is not the same measurement as one taken with it off. Noise names
carry no bias token - the input is shorted to GND for those.

Those facts are written into the CSV header (`Measurement`, `Connection`,
`Reference Mode`, `Applied Stimulus`, `Bias`, ...), so the analyzer picks them up
automatically - you never retype them when exporting. **The applied voltage is
also carried into the analyzer's `Applied signal` field**, which is what fills in
`Gain Error (dB)`, `SNR In-Referred (dB)` and `ENOB @ FS (bits)` without typing
the stimulus twice.

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

### Sweeps (crosstalk, gain, CMRR)

All three sweep tabs share the same options, because they are all built from the
same kind of stepped measurement: hold a generator frequency, let it settle, step
down, repeat.

| Control | What it does |
|---------|--------------|
| **Window** | Length of one analysis window. Overlapping windows are measured inside the selection. |
| **Stability** | How far a window's detected tone may wander before it counts as a different frequency. |
| **Min hold** | How long the generator must hold a frequency before that step is kept. The default of 1 s means at least two consecutive windows have to agree on the tone, so the sliding transition between two steps is rejected instead of producing a bogus point halfway between them. Set 0 to accept whatever is detected. |
| **Downward sweep** (default on) | Tells the analysis that you start high and step down. See *Following a downward sweep* below. |
| **Lock** | How far two windows may differ and still count as the same step (default 0.5 Hz). Also the width a swept point is averaged over. |
| **Min SNR** | Rejects a window whose tracked tone is weaker than this against everything else in it. Default **-20 dB**, i.e. effectively off, because a high-CMRR point at low frequency *is* a small tone among louder interferers. Raise it if you want noisy windows dropped. |
| **Exclude 50/100 Hz** | Toggle button. Leaves the mains points out of the sweep, the curve fit and the 50 Hz value. Sweep mode already notches 50 and 100 Hz, so a generator step exactly there measures the notch, not the front end. |
| **Curve fit** | `No fit`, `Order 2`, `Order 3` (default) or `Order 4`: a polynomial in `log10(frequency)` through the swept points. |
| **Show fit** | Draws the fit as a dashed line in the channel's own colour (not repeated in the legend) and hides it again without recomputing. |
| **Fit @ 50 Hz** | The fitted value at 50 Hz, shown for the first channel or as mean and range across all plotted channels. Also written into the report as a per-channel table with `R^2` and the number of points. |
| **CM voltage** (CMRR tab) | How Vcm is established: flat amplitude, sinc3 roll-off, measured on the CM monitor channel, or monitor frequencies with the bias curve supplying the voltage. See *The four common-mode voltage definitions* below the export section. |

Every curve carries a legend with the channel name and matches the colour used
in the tables, and the same legend is part of the exported PDF/PNG figure.

The figures live in a scroll area instead of a fixed split: the mouse wheel zooms
the figure under the cursor, **Shift+wheel** scrolls the page, and **Reset zoom**
(CMRR tab) undoes every pan and zoom. Control rows scroll sideways, so the tabs
stay usable in a narrow panel instead of forcing the whole right-hand column to
about 2400 px. Only the output is shown - the per-frequency, per-channel and
window-count tables are deliberately gone.

#### Following a downward sweep

CMRR gets better as the frequency drops, so the differential signal you are
measuring gets smaller while the *intermodulation products* of the applied tone
with the mains do not. Those products sit at `n * 50 +/- f`, which means they end
up **louder than the tone itself**:

| Generator | Mixing products | Result with "strongest peak" |
|-----------|-----------------|------------------------------|
| 110 Hz | 10, 210 Hz | correct - the tone is still the loudest |
| 30 Hz | 70, 130 Hz | **70 or 130 picked**, 30 Hz lost |
| 15 Hz | 85, 115 Hz | **115 picked**, 15 Hz lost |

Picking the loudest peak therefore reports a frequency the generator was never
set to - which is why a sweep could show a point above the step you were on.

With **Downward sweep** on, the analysis uses three facts about a real bench
sweep instead:

1. the tone only ever moves **down**, while a mixing product sits at or above it;
2. the previous tone **disappears** when the generator steps away, so a peak still
   at the old frequency at full strength means "not moved yet";
3. a step is only real once the **minimum hold** of agreeing windows has passed.

It also ignores candidates sitting on a mains multiple (50/100/150 Hz) when any
other candidate exists, and rejects a window whose only candidate is mains
pickup. On a synthetic 110 -> 90 -> 70 -> 60 -> 30 -> 15 Hz sweep with loud
mixing products this recovers all six points, where "strongest peak" returns
`60, 70, 90, 110, 115` - inventing 115 Hz and losing 30 and 15 Hz.

Switch it off if your sweep goes *up* in frequency, or if you want the old
behaviour for comparison.

#### Reading the frequency off a monitor channel

The robust fix for the low end of a sweep is not to guess at all. Wire one input
straight across the common-mode voltage (it sees close to a volt instead of
microvolts) and pick it in **CM monitor** on the CMRR tab - or select it once in
the live window's **Reference channel** panel, which then preselects it here:

* that channel's dominant peak is the generator frequency, whatever the mixing
  products are doing;
* every other channel is then evaluated at exactly that frequency with a
  least-squares sine fit, so a 5 Hz point no longer depends on finding a 5 Hz
  peak in a spectrum dominated by 50/100 Hz;
* the monitor channel itself is skipped - a CMRR number for the channel that
  measures the stimulus would be meaningless;
* its measured amplitude is plotted as **Reference channel** (Vrms versus
  frequency) against the amplitude the generator was set to, so the loss caused by
  the bias network and the sinc filter is visible directly. That figure is also an
  export section.

#### Reference channel on the Crosstalk tab

The Crosstalk tab has its own **Reference** selector, next to *Window* and
*Stability*, so which channel is the stimulus monitor is decided there rather than
being inherited from the live window. That channel is skipped in the crosstalk
curves, the curve fits and the exported table: its level is the drive itself, so a
crosstalk figure for it is meaningless. The status line says so explicitly
(`ref EXG Channel 0 skipped`), and choosing `(none)` plots every channel as before.

The selector defaults to the channel the recording marks as its reference. Note
that the recorder writes its own label (`CH 1`) while the CSV columns are
`EXG Channel 0`; `resolve_channel_name()` maps between the two, so a recorded
reference is actually applied instead of silently ignored.

Give the monitor input its own converter, set that converter to **true
differential**, and let the other one run SRB1 or SRB2 (see *Reference / SRB*
above). With a single converter in SRB1 mode every input is tied to the SRB1 pin,
so a true-differential monitor input is only possible on the other chip.

#### Following a sweep that the recorder drove itself

If the recording was made with *Measurement -> Automated sweep*, the header (via
the sidecar) carries the exact schedule: which frequency, how long it was held,
and when. The CMRR tab then shows `f generator plan (17 steps)` and every point is
computed at the frequency the generator was told to produce, inside the time it
held it - no peak picking, no stability vote, and points at 5, 7 and 10 Hz come
out like any other.

**How the windows are chosen** (this is the part that decides whether a fast
sweep is usable):

* *Without a recorded schedule* the selection is cut into fixed-length,
  overlapping windows that simply follow each other. The generator's step
  boundaries are unknown, so a window can straddle a frequency change, and the
  analysis only recovers a point when several consecutive windows agree - which is
  why fast sweeps used to need a long hold.
* *With a recorded schedule* none of that applies: each step has its own
  `start|settle|end`, the analyzer takes only the settled part of the hold and
  averages as many whole windows as fit inside it. A step that is too short to
  contain a trustworthy window is reported as rejected instead of producing a
  garbage point.

The recorder enforces `max(min dwell, 10 periods above 50 Hz / 5 periods at or
below)` and defaults the minimum dwell to **1 s**, because a generator needs a
few cycles to settle and the ADC needs enough cycles for the sine fit.

The same applies to **Crosstalk** and **Gain**: both read the schedule and stop
guessing. Gain additionally takes the applied amplitude from the recording
(`Sweep Amplitude (Vrms)`) instead of asking for it again, divides by the
attenuator ratio and reports `measured RMS / applied RMS` per channel in V/V and
dB. Gain deliberately does **not** notch 50/100 Hz - a gain step is a ratio of two
known amplitudes, and a notch would shave the tone whenever the generator sits
near those frequencies.

## Manual generator control (left panel)

The dock carries a **Function generator** group directly under *Connection*, so
the stimulus is always one click away instead of living in a menu. It drives the
same UTG932E the automated sweeps use, and every command runs on a worker thread
(opening the USBTMC resource costs hundreds of milliseconds, and a hung front
panel costs the full 3 s SCPI timeout).

| Control | Meaning |
|---------|---------|
| **Output: Off (outputs open)** | Both output relays open, the input floats |
| **Output: 0 V (short to GND)** | Both outputs enabled at the generator's minimum amplitude. The instrument clamps `BASE:AMPLitude` at **1 mVrms** (a write of 0 reads back as `1.000000e-03`), so this is a 0 V drive through the ~50 ohm source impedance - electrically the closest a generator gets to a short. Use it for shorted-input noise runs, but remember the 1 mVrms residual |
| **Output: Sine - CH1 only** | Plain sine on CH1, CH2 off |
| **Output: Sine - differential** | CH1 at 0 deg and CH2 at 180 deg, phase-locked (`SYSTem:PHASe:MODe SYNChronization`), giving a real differential stimulus |
| **Frequency** | 0.01 Hz - 100 kHz, remembered between sessions |
| **Amplitude** | **Vrms the amplifier should see.** In differential mode the two outputs add, so each channel is programmed with **half** of it: 1 Vrms differential is written as 0.5 Vrms on CH1 and 0.5 Vrms on CH2, exactly as a 1 Vrms source in series with the two outputs would behave. The hint under the fields always shows the split |
| **Atten series / Atten shunt** | The resistive divider between generator and amplifier (defaults 1 Mohm / 100 ohm). The derived ratio `shunt / (series + shunt)` and the resulting level at the amplifier are shown live, so `1 Vrms` with the default divider reads `20 uVrms at the amplifier` |
| **Apply / 0 V / Off** | Program the selection and read the instrument back. Every mismatch (frequency, per-channel amplitude, phase, output state) is logged as a warning instead of being silently accepted |

The three output modes, verified against the instrument:

* 0 V → both outputs enabled, amplitude reads back 1 mVrms
* single-ended 0.4 Vrms at 37.5 Hz → CH1 `amp = 0.4`, `offset = 0`, `SINe`, CH2 off
* differential 1 Vrms → CH1 0.5 Vrms 0 deg, CH2 0.5 Vrms 180 deg

The divider is written into **every** recording (`Input Attenuator Series (ohm)`,
`Input Attenuator Shunt (ohm)`, `Input Attenuator Ratio`) together with the
generator state (`Generator Output`), and the analyzer converts the applied level
with the recorded ratio instead of its built-in default - so a gain sweep taken
with a different divider still reports the right gain. The gain tab shows the
divider in its control row (`Divider 470/(220000+470) = 0.002132`) and the report
prints it under *Analysis settings*.

Manual controls refuse to fight an automated sweep: while a sweep is running they
report that the generator is busy instead of overwriting its steps.

## Automated measurement (function generator)

*Measurement -> Automated sweep (function generator)...* opens its own window. It
drives a UNI-T **UTG932E** over USBTMC with plain SCPI (`:CHANnel<n>:BASE:FREQuency`,
`:BASE:AMPLitude`, `:OUTPut`, `:SYSTem:PHASe:MODe`) and records the board while it
steps through the frequency list.

| Field | Meaning |
|-------|---------|
| VISA resource | Auto-detected (`Find`), remembered between sessions |
| Amplitude | Generator **Vrms the amplifier should see**. The label reads *Amplitude (Vrms diff)* in differential mode, where each output is programmed with half of it |
| Load | Load the generator assumes. Default **10 kOhm** so the programmed Vrms is what the high-impedance front end actually sees; with the usual 50 ohm assumption the generator halves it |
| Drive | **Common mode** - CH1 into every input (CMRR). **Differential** - CH1 at 0 deg and CH2 at 180 deg, phase-locked, for a real differential stimulus (crosstalk). **Gain** - CH1 into the input under test; the analyzer reports measured/applied per channel |
| CM monitor | Channel wired across the common-mode voltage, recorded with the sweep. It is the same choice as the dock's *Reference channel* |
| Frequencies | One per line. Default `110, 95, 90, 85, 80, 70, 60, 55, 45, 40, 35, 25, 20, 15, 10, 7, 5` |
| Minimum dwell | Never shorter than this (default **1 s**; 300 ms is the floor the dwell rules allow) |
| Periods above / at or below 50 Hz | Default **10** and **5**, so the hold is `max(minimum dwell, N / f)` |

Every voltage in the application is an **RMS** value: the generator is programmed
in Vrms, the applied stimulus in the recording name is Vrms, the analyzer's
`Vcm` and gain-amplitude fields are Vrms, and the reports print `Vrms`/`mVrms`.
The generator output is additionally always **AC centred**: `BASE:OFFSet` is
written as 0 V, so a 1 Vrms sine swings +/-1.41 V around ground rather than 0..2.8 V.
A DC-shifted "sine" is not a valid biopotential stimulus - it puts a constant
offset on the input and saturates the front end.

Start records first and then drives the generator, so the schedule is expressed in
recording time. The recorder stamps the moment each frequency was applied, keeps a
short settle margin at the start of each hold, and writes
`Sweep Mode`, `Sweep Drive`, `Sweep Amplitude (Vrms)`,
`Sweep Per-Channel Amplitude (Vrms)`, the `Input Attenuator *` keys,
`Sweep Monitor Channel` and one
`Sweep Step <n> = frequency|amplitude|start|settle|end` line per step into the
annotation sidecar when the recording stops. `read_openbci_metadata()` merges those
keys, so the analyzer sees them exactly like header fields.

The generator's output is switched off and the recording stopped when the list
finishes or when *Stop* is pressed.

### What one sweep step actually sends

A sweep is a continuous measurement, so a step must be one small, quiet change.
`apply_step()` therefore writes **only the frequency** - one message per driven
channel - and nothing else:

| | Before | Now |
|---|---|---|
| Per-step writes | 14 (mode, wave, unit, amplitude, offset, frequency and phase on both channels) | **2** (CH1 frequency, CH2 frequency) |
| Per-step queries | 12 (full read-back of both channels) | 0 |
| Output toggling | CH1+CH2 off, then on, every step | none |

Everything that is constant for the whole sweep - phase mode, load, waveform,
amplitude, offset and phase, plus turning the outputs on - is programmed once by
`prepare_sweep()` (19 writes) before the first step. Measured on the bench: the
first step takes ~180 ms including that setup, every later step ~1 ms.

Two consequences worth knowing:

* **The outputs are never switched off and on during a sweep.** The old restart was
  there to re-establish the 180 degree relationship; in `SYSTem:PHASe:MODe
  SYNChronization` both channels derive from one clock and the phase offset is a
  programmed parameter (`BASE:PHASe 180`), so it survives a frequency change. The
  restart also put a step into every CMRR point, which is exactly what a
  continuous measurement must avoid.
* **Only the first step is read back.** That proves the whole programming path on
  this instrument; a read-back on every step would add two query round trips to
  every point and make the frequency change and the dwell start disagree. The
  value is still checked whenever the generator's state changes: a step whose
  amplitude differs from the previous one re-programs everything and verifies.

**SCPI commands are sent one message at a time, never chained.** Chaining
(`:CHANnel1:...;:CHANnel2:...`) looks like the obvious way to change both channels
in one write, but on the UTG900E (firmware 3.06) only the *first* command of a
chained message takes effect and the instrument then stops answering queries until
the link is reset. That was verified on the hardware: the second channel kept its
old frequency and the next query timed out.

### Using the UTG932E

**1. Install the driver stack.** The generator is driven over VISA:

```
pip install pyvisa pyvisa-py
```

`pyvisa-py` alone has no USBTMC backend on Windows, so pick one of:

* install the **NI-VISA** runtime (free, National Instruments) or **Keysight IO
  Libraries Suite** - then plain `USB0::...` resources work; or
* use the **LAN interface** of the generator - `TCPIP0::<ip>::INSTR`, which needs
  no extra driver and is the most reliable option; or
* keep `pyvisa-py` with `pip install pyusb libusb-package` **and** replace the
  USB driver of the instrument with WinUSB/libusb (Zadig). Only do this if you
  cannot use NI-VISA or LAN.

**2. Find the resource string.**

```
python utg_control.py list
```

prints every VISA resource; a UTG9xx over USB looks like
`USB0::0x6656::0x0834::<serial>::INSTR` (VID `6656` is UNI-T, so that one is
listed first). Then prove the whole chain:

```
python utg_control.py idn
python utg_control.py selftest --freq 110 --amp 0.1
```

`selftest` connects, programs one sine, **reads the settings back**, and switches
the output off again. A line like
`OK: CH1 holds 110 Hz at 0.1 Vpp` means the sweep will work; `MISMATCH: ...`
names exactly what the instrument did not accept.

You can also just apply a sine by hand:

```
python utg_control.py sine --freq 60 --amp 0.2 --output on
python utg_control.py sine --phase 180 --channel 2 --freq 60
```

**3. Run a sweep from the app.** *Measurement -> Automated sweep (function
generator)...*, then:

1. **Find** (or type the resource), **Test generator** - the status line shows the
   readback so you know the link is real.
2. Set **Amplitude** (Vpp) and **Load** (leave 10 kOhm for the high-impedance
   front end).
3. Pick **Drive**: *Common mode* for CMRR (CH1 only), *Differential* for crosstalk
   (CH1 0 deg + CH2 180 deg, phase-locked through
   `:SYSTem:PHASe:MODe SYNChronization`).
4. Pick the **CM monitor** channel if you have one wired across the common-mode
   voltage.
5. **Start sweep & record**. The board has to be connected; the recording starts
   first and stops by itself at the end.

**Every step is verified.** After programming a frequency the app asks the
instrument what it holds (`:BASE:FREQuency?`, `:BASE:AMPLitude?`, `:OUTPut?`,
`:BASE:PHASe?`) and reports a mismatch instead of silently producing a wrong CMRR
point; the problems are collected and shown in the sweep status and the serial
monitor log.

**The output is always a plain, unmodulated sine.** The UTG900E keeps its
modulation setting (AM/PM/FM/FSK/Line/Log) independently of the carrier, so a
generator left in AM mode from the front panel emits an amplitude-modulated
"sine" no matter what the carrier settings say. Each step therefore writes
`:CHANnel<n>:MODe CONTINUE` before anything else, and also releases a front-panel
key lock (`:SYSTem:LOCK OFF`) that some firmware revisions use to reject typed
commands.

| Symptom | Cause |
|---------|-------|
| `pyvisa is not installed` | `pip install pyvisa pyvisa-py` |
| `No VISA instrument found` | generator off, USB cable is charge-only, or no VISA backend (see step 1) |
| `Could not open USB0::...` on Windows | `pyvisa-py` without a USBTMC backend - install NI-VISA or switch to LAN |
| `(write-only: the generator accepts commands but answers no queries)` | the resource opened and commands execute, but every query timed out. The sweep still runs (it only needs writes); the readback check is skipped. Install NI-VISA to get verification back. The driver already tries a raw read as a fallback, which fixes most pyvisa-py USB setups |
| `MISMATCH: CH1 reports 1000 Hz, set to 110 Hz` | the write was rejected; check the front panel is not in a modal menu |
| amplitude read back ~half | the generator's load setting is 50 ohm; the sweep sets 10 kOhm, so set the same if you drive it by hand |
| `CH1 reports its output is off` | output was not enabled, or the channel is in a mode that gates the output |
| sine looks amplitude-modulated | the front panel is in an AM/FSK/Line mode the step did not clear - run *Test generator*, which applies `MODe CONTINUE` and reads the mode back |

> Why typed SCPI and not the `ebench`/`ebUnit` wrapper? `ebench` is a
> YAML/absl-driven bench-menu framework whose UTG driver
> (`ebench.Unit.UnitSignalGenerator`) works by emulating **front-panel key
> presses** (`KEY:CH1`, `KEY:NUM1`, ...), keeps a shadow copy of the channel
> on/off state because it cannot read it, waits 0.2 s per command, and has no
> `SYSTem:PHASe:MoDe` command at all - so it cannot do a phase-locked
> differential drive and cannot keep up with a 300 ms dwell. The UTG900E accepts
> typed parameter SCPI directly, which is what `utg_control.py` uses: a whole
> 17-step sweep is a few hundred milliseconds of serial traffic plus the dwell
> times. What *was* worth taking from that project is its "verify, never trust a
> blind write" habit, which is now `verify_step()` / the *Test generator* button.

## Bias reference curve (what voltage the generator actually delivered)

*Tools -> Bias reference curve...* answers a question CMRR cannot answer on its
own: at each frequency, how big was the common-mode voltage really?

`CMRR = 20 log10(Vcm / Vout)`. `Vcm` is usually typed in as one number - "I set the
generator to 1 Vrms" - but the source impedance, the cable, the input network and
the ADS1299's sinc3 filter all take a share, and the share grows with frequency.
Using the nominal number does not make that error go away, it books it as CMRR.

The curve is built from **reference recordings**: one or more runs with channel 0
wired straight across the generator output, no bias amplifier in the path. Channel
0 then measures the real delivered voltage, and averaging several such runs
suppresses each one's noise.

* **Add recordings...** - two or more CSVs. **Reference channel** defaults to the
  first channel, which is the convention these recordings are made under.
* A recording that carries a generator plan is read at the frequency of every
  step, inside the time that step held. Without a plan the dominant tone of each
  window is used, and windows whose tone does not dominate are dropped (those are
  the sliding transitions between steps).
* **Bucket** groups readings that are within that percentage of each other, so
  5 Hz is averaged within 0.1 Hz and 200 Hz within 4 Hz. **Min recordings** drops
  a frequency that too few files contributed to, so one noisy run cannot pass for
  a well-averaged point.
* **Fit** - a polynomial in log10(f) (least squares, order 1-20) or an exact fit
  through every averaged point: monotone PCHIP, natural cubic spline (minimum
  curvature), or straight lines. Exact fits are the honest choice once the points
  already are the measurement; the polynomial is for a trend through scatter.
* **Save curve...** writes a JSON file. This is calibration, not per-recording
  data - build it once and reuse it.

The figure shows the averaged points with their ±1 SD spread, the generator
setpoint, and the fitted curve; below it the same curve expressed as a gain in dB
against the setpoint, which is the front end's own response.

### The four common-mode voltage definitions

The CMRR Sweep tab has a **CM voltage** selector. These are four different answers to
"what voltage was really at the input", and they differ by tens of decibels, so the
choice is explicit and the resolved value is shown under the control:

| Mode | Vcm used | When to use it |
|------|----------|----------------|
| **Flat** | The constant amplitude in the Analysis panel, for every frequency | The stimulus is trustworthy as a number and the front end is flat enough across the band |
| **Sinc3 roll-off** | That same amplitude, reduced with frequency by the ADS1299 digital filter at the recording's data rate | You want the converter's own filter accounted for without a monitor channel. At 500 SPS the sinc3 response is 0.95x at 50 Hz and 0.78x at 110 Hz |
| **CM monitor measured** | The voltage actually measured on the monitor channel, per frequency - as-is | The converter measured the common-mode voltage itself, so no sinc3 correction is applied: it is already in the number |
| **CM monitor + bias curve** | Frequency and timing from the monitor channel; voltage from the fitted bias reference curve | A bias network attenuates the stimulus. The sweep still knows where it is (from the monitor), and the voltage is what the generator really delivered |

Mode 2 uses the recording's own sample rate as the converter data rate. In daisy mode
the per-channel rate is half the chip's output data rate, so check the note under the
selector before trusting the roll-off.

**Mode 4 is the one that answers "what would this have been without the bias amp".**
The monitor channel supplies the frequency and the timing - which is what it is good
at, and all that survives when the amplitude is attenuated to 25 mV - while the
voltage comes from the curve built out of bias-off runs:

```
bias on,  monitor measures   0.025 Vrms   ->  CMRR 105 dB
bias on,  curve supplies     0.85-0.94 Vrms ->  CMRR 136 dB      (+31 dB)
```

That 31 dB *is* the bias network's attenuation, and the figure caption states it.

Two rules the implementation follows, because each was a real bug:

* **the curve is never scaled by a measured voltage.** Dividing the curve by the
  attenuated measurement cancels the correction exactly. The only automatic scale is a
  ratio of generator *setpoints* - if the recording states it was driven at 0.5 Vrms
  and the curve was built at 1 Vrms, the curve is halved. No manual scale control
  exists, because there should be nothing to get wrong.
* **frequencies outside the curve's range are dropped, not extrapolated**, and appear
  in the rejected count.

Frequencies outside the curve's measured range are **dropped, not extrapolated**,
and they appear in the rejected count - a point whose Vcm is unknown produces a
made-up CMRR. The `Vcm Used (Vrms)` column records exactly which voltage every row
was computed against, and the sweep info line reports the range actually supplied
(for example `Vcm curve 0.86-0.94 Vrms`).

### Reading the reference figure

The **Reference input actually measured, and its fit** figure gains two traces when
a curve is active:

| Trace | What it is |
|-------|------------|
| green, `EXG Channel N measured` | what the monitor channel actually saw at the input - i.e. **after** the bias network |
| red dashed, `generator set` | the amplitude the generator was told to produce |
| blue, `Vcm used for CMRR` | the curve evaluated at each swept frequency, scaled - the number CMRR was divided by |
| blue dashed, `bias curve (scaled)` | the same curve as a continuous line |

The gap between the green and blue traces **is the bias network's attenuation**, and
the caption under the figure states it: *"Bias network attenuates the common-mode
voltage by 29.7 dB (0.0328x)"*. When the two traces differ by more than 6 dB the
figure switches to a logarithmic voltage axis, because 0.03 V and 0.95 V cannot both
be read on one linear axis - hiding one of them is exactly what made the curve look
like it was doing nothing.

The curve value is also written into the reference table as
`Vcm from curve (Vrms)`, so the exported CSV and the PDF report carry it.

### Why the CMRR number barely moves

A curve fitted to 1 Vrms recordings returns values close to 1 V at the swept
frequencies, so substituting it for a typed **1 Vrms** changes CMRR by about a
decibel. That is not a fault: the curve is a *better* 1 V, not a different quantity.

The case that matters is a **bias-on** run. There the monitor measures 0.02-0.03 Vrms,
and mode 4 supplies 0.85-0.95 Vrms - so CMRR moves by **30-31 dB**, which is the bias
network's attenuation. If it moves by 1-2 dB instead, the *CM voltage* selector is on
mode 3 (which uses the attenuated measurement) rather than mode 4.

The caption under the reference figure states the ratio directly, for example *"Blue
trace is the voltage CMRR was divided by, 31.3 dB above what reached the input
(0.0273x)"*, so both readings are always comparable.

**Record the calibration at the same output data rate as the CMRR runs.** Part of
what the curve measures is the ADS1299 sinc3 filter, and that response moves with
the ODR. A curve built at 500 SPS describes 500 SPS measurements; the sample rate
of every input file is listed in the file table so a mismatch is visible.

## Bluetooth throughput benchmark

*Tools -> Bluetooth throughput benchmark...* measures what the SPP link actually
delivers to this PC. It is a bench tool about the radio, not part of an
experiment, so it lives under Tools rather than beside the measurement controls.

The firmware generates frames in memory and pushes them through the **production**
transmission path - the same ring buffer, the same flush task on core 0, the same
SPP stack, the same Windows virtual COM port - so the number is the link and only
the link. **The ADS1299 is not read at all**: no SPI traffic, no conversions.

| Control | Meaning |
|---------|---------|
| **Mode** | *Maximum throughput* (as fast as the link takes it), *Fixed rate* (32/64/100/150/200 KiB/s targets), *Automatic sweep* (steps up 32, 48, 64 ... 256 KiB/s and reports the highest rate that passes) |
| **Duration** | Length of a single run, 30 s by default |
| **Payload** | Frame payload: 128 to 4096 bytes. 990 is the classic SPP-friendly size; 1024 matches the firmware's flush chunk |
| **SPP write** | Bytes per `esp_spp_write()`: 990 (default), 1980 or 4096 on the direct path, plus **330 B** which deliberately runs the Arduino library path as a lossy reference. See *Raising the throughput ceiling* |
| **Sweep settle / measure** | Per step: how long the rate is held before counting, then how long is measured. The measurement window is judged on its own counters, so the ramp into a rate cannot flatter it |

### Raising the throughput ceiling

A first run of this benchmark peaked at **31.3 KiB/s** and lost a few packets, with
nothing at all arriving above 48 KiB/s. That was not the radio's limit - it was the
Arduino `BluetoothSerial` wrapper:

* it splits every write into **330-byte** `esp_spp_write()` calls;
* the radio allows **only one outstanding write**, so throughput is
  `chunk / acknowledgement round-trip` - about 32 KiB/s at 330 bytes;
* when the stack reports congestion it **discards the rest of that chunk**, so the
  dropped bytes arrive at the PC as truncated frames. That is what the
  corrupt-packet count was showing, and at higher rates it shredded the stream
  completely, which is why the upper sweep steps verified zero packets.

The bulk stream now bypasses the wrapper. `btRawWrite()` calls `esp_spp_write()`
directly, with the rules the API imposes: one outstanding write at a time, nothing
sent while the link is congested until `ESP_SPP_CONG_EVT` clears it, and a 100 ms
acknowledgement wait instead of the library's 1000 ms. A chunk is never dropped
part-way - if the stack will not take it, it is retried whole.

**SPP write** selects the chunk size. Throughput is roughly `chunk / round-trip`, so
990 bytes is about 3x the old ceiling and 1980 about 6x, until something else becomes
the limit. Try 990, then 1980, then 4096, and keep the largest that stays stable.

The one size that does **not** use the direct path is **330 B**: that is the library
chunk, so selecting it runs the wrapper deliberately, as a reference point for how much
the direct path actually buys. Everything above 330 uses the direct
`esp_spp_write()` path, because the library path cannot carry a test: it copies every
chunk with a `malloc` and discards the remainder of a chunk when the stack is congested -
and a throughput test is nothing but congestion. Measured consequences of running a test
on it were a heap down to 10 kB, thousands of write failures, and 3.5 kB arriving at the
host with **zero** frames parseable out of it. The direct path takes no copy and never
drops a partial chunk.

It is active only while the test runs - the size resets the moment the test stops - so
ordinary streaming and every text reply keep using the library path that has always
worked. It also abandons itself after eight missed acknowledgements in a row and falls
back, reporting it on the serial monitor. The **Write path** field shows which is live
(`direct 990 B` / `library 330 B`), and the diagnostics line turns amber if a benchmark
finds itself on the library path.

The direct path needs two things, and the board prints both so a failure is never a
guess. The `EVENT=diag` line carries:

| Token | Meaning |
|-------|---------|
| `SPP` | The write size currently in force |
| `THRESH` | The library chunk it must exceed to use the direct path (330) |
| `HANDLE` | The SPP client handle the firmware is holding; `0` means the board has no client, so no direct write is possible |
| `FAST` | `1` when the direct path is live |
| `STACK` | Bytes the stack has **accepted** from the benchmark. This is the one that matters: if it stays at `0`, nothing is leaving the board whatever `QUEUED` says |
| `ERR` | `esp_spp_write()` calls the stack refused - normal at a low rate, a fault if `STACK` stays `0` |
| `ACK` | Completed writes whose acknowledgement never arrived |

`FAST=0` with `ACTIVE=1` is one diagnostic; the app turns the line amber and names
the input that failed - "no SPP handle yet" or "write size 330 is not above the
library threshold 330". `FAST=1` with `STACK=0` and a rising `ERR` is the other,
and it means the link will not take that write size at all.

### What was wrong, and why it needed a rewrite

Two defects in the write layer, both of which the numbers above make visible:

* **The in-flight flag latched.** The flag that stops a second write overlapping the
  first was being set from the *congestion* flag of the completed write. "This write
  congested the link" therefore read as "a write is still outstanding", and only
  another write could clear it - which was not allowed. One congested write stopped
  the stream permanently. The flag is now cleared by `ESP_SPP_WRITE_EVT`
  unconditionally, and by a 150 ms timeout if that event is ever lost. Congestion is
  tracked separately and also expires.
* **The flush task could spin.** When a status line could not be sent it retried
  with no delay, burning a core at ~200 000 iterations a second; the data ring was
  then never reached, which is why the diagnostics showed `STACK=0` with `FAILS`
  climbing into the hundreds of thousands. Text now yields, and a line that cannot be
  sent within a second is dropped - it is diagnostic output about a link that is
  already broken.

A benchmark also **ends itself** now. If the stack accepts no data for 2.5 s the
board stops the run and restores the library write path, so a failed test can never
leave the link engaged and the board needing a reconnect. The app treats that as the
end of the run and re-enables Start.

### The real limit was memory, not the radio

Every failed run in the field pointed at the same thing once the numbers were read
together: the board had **about 12 kB of heap free** during a test, and Bluedroid
cannot transmit without contiguous memory for each SPP write. Its own library
complained in plain text:

```
E[BluetoothSerial.cpp:165] _spp_queue_packet(): SPP TX Packet Malloc Failed!
```

### Measured: the library queue is the fast path

Once the memory problem was gone the numbers were unambiguous:

| Write path | Chunk | Verified throughput | Frames intact |
|-----------|-------|--------------------|---------------|
| library queue | 330 B | **88-99 KiB/s** | 96-98 % |
| direct `esp_spp_write` | 4096 B | 11.3 KiB/s | 74.8 % |
| direct `esp_spp_write` | 990 B | 1.3 KiB/s | 26 % |

The direct path is the *slow* one, and the reason is architectural rather than
tuning:

* **`BluetoothSerial::write()` queues and returns.** Its own `_spp_tx_task` fills a
  330-byte send buffer and blocks on the acknowledgement, so the sample ring refills
  while a write is in flight - the pipeline is two deep by construction.
* **`esp_spp_write()` permits exactly one outstanding write.** A producer that waits
  for each acknowledgement can only ever reach `chunk / round-trip`, and on this link
  the round trip is long enough that 990 bytes does worse than 330 queued.

So 330 through the library is the default and the recommended setting. The larger
sizes remain selectable so the comparison can be repeated on other hardware, where
the acknowledgement may be much faster.

### Where the remaining loss comes from

The library discards the remainder of a chunk when the stack reports congestion
(`if(!_spp_send_buffer()){ len = 0; }`). At a saturated offered rate that is normal,
and the host resynchronises on the frame sync word, so the cost is bounded to the
frame being carried. That is why "automatic sweep" is the honest way to find a
clean rate: it reports loss per step, and the verdict marks a rate as sustainable
only when almost everything verified.

Two things were making it worse than it needed to be:

* benchmark diagnostics went through `Serial`, which routes to Bluetooth whenever the
  host is on Bluetooth - so the once-per-second status line competed with the data it
  was measuring, and failed to send, and was retried. `FAILS=50186` on a 21 s run was
  almost entirely this. Diagnostics now always go to the cable.
* text replies were attempted during a benchmark at all. They are now suppressed
  while a test runs, so the radio carries data only.

Three buffers accounted for the memory that caused all of it:

| Buffer | Size | When it existed |
|--------|------|-----------------|
| Bluetooth transmit ring | 32 kB | always |
| **SD sample queue** | **32 kB** | **always, even with no card inserted** |
| Benchmark frame scratch | 4 kB (up to) | always, once a benchmark had ever run |

The SD queue was the waste: it was created in `setup()` and never released, for a
card that may not even be present. It is now created when a recording opens and
released as soon as the recording closes (the writer task owns the deletion, since
it is the only task blocked on it). The frame scratch is now sized to the frame in
use - 276 bytes for a 256-byte payload, not 4 kB - and freed when a run ends.

The transmit ring is sized per path, because the two need opposite things: **8 kB**
for the direct path, which blocks its producer and therefore needs the rest of the
memory in the heap for Bluedroid's per-write buffers; **32 kB** for the library path,
which needs no extra heap and benefits from the depth to ride out Windows' read
jitter. The original size is restored when the test stops.

A benchmark now refuses to start below **20 kB** of free heap rather than producing
thousands of errors and no throughput.

### Bluetooth controls in Hardware Settings

Two link settings are saved with the rest of the hardware state and applied on every
connect, so a board is never left on a stale radio configuration:

| Control | Meaning |
|---------|---------|
| **Bluetooth power** | BR/EDR transmit level from -12 to +9 dBm. Sent as `\|btpower=<dBm>;` and **read back from the controller** - the board reports what the controller accepted, not what was asked for, so a refused level cannot masquerade as applied. Also shown as `BTDBM=` in the `[PWR]` line |
| **Bluetooth SPP write** | Bytes per SPP write, 330 to 4096. 330 is the measured-fastest library queue path and is the default; the larger values select the direct write and are marked experimental |

### SPI clock

The bus clock is checked against two things before it is accepted, because both
failures look like "the stream turned to gibberish" rather than like an error:

* **Can the bus carry the selected rate?** Two 216-bit frames per sample must fit
  inside one sample period. At 1 MHz that is 432 us, so 2 kSPS would spend 86 % of
  every second on SPI, and the frame read overruns the next DRDY - every sample after
  that is taken from the wrong conversion. A clock below what the selected data rate
  needs is now **refused with the numbers**, rather than accepted and left to corrupt.
* **Do the converters still answer?** After the change the device ID is read back over
  the new clock. A wrong or absent answer reverts to the previous clock and says so.

The rate code `0` is a real value (16000 SPS) and was being read as "not set" and
replaced with 250 SPS, which made the fastest rate silently unselectable.


### Write size on the library path

`BT_LIBRARY_WRITE_CHUNK` is now exactly the library's own internal buffer (330), not
1024. Handing `BluetoothSerial` 1024 bytes makes it assemble 330 + 330 + 364 and
**discard the 364** on the first congestion, and it `malloc`s a whole copy per call.
At 330 the library's send buffer is emptied on every write, so there is never a
remainder to lose - which is why the one configuration that ever worked by accident
(payload 256 on the 330 path) was the one where a frame fit in a single piece.

The library writes in **330-byte** pieces and discards the rest of a piece when the
stack congests, so **a frame larger than 330 bytes cannot survive the 330-byte path**.
A 990-byte payload on a 330-byte write produced 18 kB of received bytes and not one
parseable frame. The firmware now never hands the library more than 330 bytes for
exactly that reason, and the app reduces the payload to fit when 330 B is selected and
says so in the run summary; 990 B and above use the direct path and take any payload
up to 4096 bytes.

### Benchmark link: the board serves one Bluetooth client

| Data link | What it reads |
|-----------|---------------|
| **Main window link** | The connection the recorder already holds. Used automatically when the main window is on Bluetooth |
| **This window's own Bluetooth port** | A second, separate SPP connection - only possible when the main window is on USB |
| **USB** | The cable, as a reference only. Never reported as Bluetooth performance |

**The board accepts one SPP client at a time.** A second connection makes the firmware
treat the existing one as stale: it disconnects the tracked handle and starts link
recovery, which drops the running stream and leaves the port unusable until the
adapter or the board is reset. So the window will:

* remove the port the main window is holding from its list;
* default to *Main window link* the moment the recorder connects over Bluetooth;
* refuse to open a second connection at all, and say why;
* release the handle - including on a failed open - so the port is never left busy.

That last point is what made the port look permanently "used by something else":
the guard now runs `cancel_read()` before `close()` (a Windows Bluetooth port can sit
in a read and ignore the close) and waits for the reader thread to exit.

**Commands always prefer the cable**, so with USB on commands and Bluetooth on data
the board stays fully controllable while the radio is saturated and **Stop** is never
queued behind test traffic.

Bytes are only ever tapped from **one** link - attaching the receiver to two would
count every packet twice.

### Why the throughput gain is opt-in

An earlier revision made the direct write path the default and doubled the transmit
ring to 64 kB. That produced the worst kind of failure: the PC saw `SZD_EEG`,
connected, and the board never answered. Two causes were possible - memory pressure
on the Bluetooth stack, or a handshake problem in the direct path - so both were
addressed rather than guessed between:

* the ring is back to **32 kB**, and it now refuses to allocate at all if it would
  leave less than **96 kB of heap** for Bluedroid. That reserve exists because the
  ring is allocated *before* the stack starts, and Bluedroid's own TX/RX buffers come
  out of what is left. This is the failure mode to protect against, because it looks
  like "advertises, connects, never answers" rather than like an allocation error.
* the direct path is now **opt-in** and **self-healing** (see *SPP write* above).

The boot log now states the outcome, so this is never a guess again:

```
[BT] transmit ring 32768 B, heap after 118000 B
```

If you want to prove where a Bluetooth failure sits, the discriminator is the USB
serial monitor: a board that answers there but not over SPP has a link problem, and
one that answers nowhere has not booted.

### What the numbers mean

Three different byte counts are shown, and they answer different questions:

* **Verified payload** - bytes this PC received, parsed and proved byte-for-byte
  against the payload the firmware must have generated for that sequence number.
  *This* is throughput.
* **Firmware accepted** - bytes the Bluetooth stack took from the firmware's ring.
  They can be accepted and still never arrive.
* **Firmware queue** and **queue fill** - what the generator produced versus how
  much is still waiting. A queue that keeps growing means the link is behind, not
  that the buffer is too small.

**Corrupt**, **Missing** and **Sequence errors** come from the CRC, from
regenerating the payload, and from gaps in the sequence numbering.

Three byte counts, three different faults:

| Field | If it is zero |
|-------|---------------|
| **Raw bytes** | Nothing reached this program: the link or the port is the problem |
| **Received** | Bytes arrived but none formed a benchmark frame: framing problem, usually an older firmware image |
| **Verified payload** | Frames arrived but failed their checks: corruption |

The **Board diagnostics** line under the summary shows the firmware's own view - whether
the generator is active, what the ring holds, how many bytes the stack accepted, and
whether `BluetoothSerial.write` is failing. It is the only way to tell "the firmware
never produced anything" from "the PC never received it", and it refreshes once a
second while a test runs.

The **Status** field is derived, not typed: *stable* means everything arrived and
the queue stayed flat; *congested* means the queue rode high or the firmware hit
transmission stalls; *losing data* means packets went missing.

**Payload throughput is not the radio's PHY bitrate.** Bluetooth Classic carries far
more on air than survives SPP framing, L2CAP fragmentation and the Windows driver.
The figure shown is the application bytes that verifiably crossed the boundary.

### Safety

The test stops the sample stream first and refuses to start if it does not stop,
because the generator and the acquisition path share one ring buffer. It refuses
to start when no Bluetooth host is connected, or when the board does not report
`BENCH` support - an older image accepts the commands and sends nothing, which
looks exactly like a dead link. The generator is bounded to 4 ms per `loop()`
iteration, so **Stop** always lands even at maximum speed. On stop the firmware
returns to its normal idle state and does not restart the stream by itself.

If the selected port is a cable rather than a Bluetooth port, the window says so in
amber and labels the result *USB reference (NOT Bluetooth)* - a USB measurement is
a useful reference but it is a different link.

Results save to **JSON** (full record, including the per-second samples) or **CSV**
(one row per second) for a paper.

## Bluetooth link budget

The recorder used to assume a fixed 32 KiB/s for planning sample rates over
Bluetooth. That figure is a conservative estimate, not a measured limit.

The benchmark's **Use measured rate as link budget...** adopts a *verified
sustainable* rate - never a peak - with a safety margin on top (20 % by default,
and applied only to an adopted value). The margin is 0 % until something measured
is adopted, so nothing changes for anyone who never runs the test, and clearing the
stored value returns to the built-in 32 KiB/s.

The sample-rate feasibility calculations, the packet-rate budget and the link note
all read that one budget. The distinction between ADS1299 sample rate, output
packet rate and per-channel rate in daisy mode is unchanged.

## Exporting from the analyzer

`Export -> Export report...` (or any of the three older entries, which just
preselect a format) opens one dialog:

* **Format** - PDF report, PNG images, CSV tables. Any combination.
* **Include** - a checklist of sections: measurement & settings, waveform plot,
  FFT plot, metrics table, crosstalk table, crosstalk sweep, gain sweep, CMRR
  sweep, fitted curves, mean of the fits, 50 Hz table and the reference channel
  figure. Sweeps are greyed out until you have computed them.
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

Sweep figures are drawn over the measurement band with a fixed **0-125 Hz** axis
and contain the curves only: the per-frequency, per-channel and window-count
tables are deliberately not in the report. What the report does carry is the
output: the measured points, the fitted curves on their own, the mean-of-fits
figure, the reference-input figure with its own fit, and a **50 Hz table** whose
values come from each channel's fitted curve (plus a *Mean of fits* row), so no
swept point has to land exactly on 50 Hz. The same table is shown under the CMRR
tab.

Figures are exported at a forced wide aspect (height/width 0.42) rather than at
whatever shape the on-screen plot happens to have, so each one fills the page
width. The render temporarily reshapes the plot, paints it twice (pyqtgraph only
re-lays out when painted) and puts the widget back exactly as it was, so
exporting never rearranges the window.

## SD card recording (standalone and alongside the live link)

The board can record to its own microSD card, either on command from the host or
completely standalone from the record button, while the live link keeps streaming.
The dock's **SD card** group is the control and telemetry surface:

| Control | What it does |
|---------|--------------|
| **Refresh** | Reads the card state and free space **on demand** - nothing is polled (see below) |
| **Name** | Session name stored on the card and written into the recording header |
| **Save settings** | Programs the converters with the current hardware settings **first**, then writes that exact configuration to `/SZD_CFG.TXT` |
| **Start / Stop** | `:b` / `:e`: start or stop a card recording. Starting also starts the live recording, so the two captures line up for comparison |

The status line reports what the board measured: free and total space in GB, the
active channel count and rate, and the recording time the remaining space buys at
that configuration (`EST_SEC`, computed from the real line length of the channel
set). While recording it also shows the file name and the written/dropped sample
counters.

### Confirmation, and why a recording needs the stream

A card recording is fed from `publishConversion()`, i.e. from the conversion
stream - not from a separate acquisition path. Starting a recording while the
stream was stopped therefore wrote the header and then **no data at all**, and the
host looked frozen because it was waiting for samples that could never arrive.
Two rules now prevent that:

* **Any recording start guarantees the stream.** `startSdRecordingFromBoard()`
  calls `startStreaming()` when it is not already running, whichever route asked
  for the recording (host command or record button).
* **A talking host gets the stream back while recording.** Every byte on the host
  link updates `lastHostByteMs`; after each command burst,
  `ensureStreamingForHost()` restarts a stopped stream when a byte arrived within
  `HOST_PRESENCE_WINDOW_MS` (50 ms) **and** a card recording is active. The
  recording scope is what makes it safe: a later `:s` refresh can never undo a
  deliberate "stop streaming", which arms `suppressAutoStream` until a `b` or a new
  recording clears it. A board with no host attached never self-starts.

Every state change is reported immediately with an `EVENT=` field, so the window
never has to guess:

```text
[SD] EVENT=start   PRESENT=1 RECORDING=1 FILE=/SZD_0002_run.CSV ... DONE=1
[SD] EVENT=stop    PRESENT=1 RECORDING=0 FILE=/SZD_0002_run.CSV ... DONE=1
[SD] EVENT=timeout PRESENT=1 RECORDING=0 ...          (reached its time limit)
[SD] EVENT=fail    ...                                (could not open/write)
[SD] EVENT=busy    ...                                (already recording)
[SD] EVENT=saved   ...                                (settings written by :w)
[SD] EVENT=status  ...                                (answer to an explicit Refresh)
```

`EVENT=status` is the Refresh answer and is deliberately *not* treated as an action
reply; every other value is. The line buffer is 384 bytes because the worst case
(long session name, 16 GB card, maximum counters) is 312 characters - the previous
256-byte buffer truncated `DONE=1`, and a line without it is ignored by the window.

### Nothing reads the card unless you ask

Reading the card's free space is not free: `SD.usedBytes()` walks the FAT's cluster
chain, which holds the card bus and the CPU for tens of milliseconds - long enough
to delay a conversion and show up as a visible jump in the traces. So the card is
never interrogated automatically:

* The window sends **no periodic `:s`**; it used to poll every 5 s.
* Connecting does not read the card. The panel says *Press Refresh to read the
  card.* until you do.
* `EVENT=start` / `stop` / `timeout` / `busy` / `fail` arrive from the firmware on
  their own - nothing is sent to trigger them, so a recording start or stop costs
  no extra card traffic.
* On the firmware side, only an explicit request measures free space. `emitSdStatus()`
  re-reads it for `status` and `saved` and reuses the cached value for every other
  report (`sdCapacityCached()`), so the automatic action replies do not touch the
  file system at all.

If no reply arrives within 2.5 s the window says which request went unanswered
instead of leaving the panel blank. When the board starts a recording on its own
(the record button), the window starts **its own live recording** in response, and
when the board stops it, the window stops its recording too - so a button-started
pair is always comparable in the analyzer.

### Steps in the traces: telling a firmware stall from an analog one

A rectangular step that appears on **every** channel at the same instant can come
from two very different places, and they need different investigations:

* **A firmware stall** - the acquisition task was kept off the CPU, so one or more
  conversions are missing and the trace is drawn across the gap.
* **The analog side** - the converters published every conversion on time, but the
  input, the reference or the supply moved (bias network, electrode, RF coupling).

The firmware now reports the first case the moment it happens, on the USB link and
in the window's status bar:

```text
[GAP] MS=12345 GAP_US=8200 PERIOD_US=1000 LATE=7 MISSED=3 REC=0 QUEUED=0 SPI_US=210 HEAP=98000
```

| Field | Meaning |
|-------|---------|
| `MS` | `millis()` when it happened - correlate with the trace |
| `GAP_US` | how long the conversion was late; `PERIOD_US` is one conversion period |
| `LATE` / `MISSED` | running totals of late conversions and missed ones |
| `REC` | whether a card recording was running |
| `QUEUED` | samples waiting for the card writer at that moment |
| `SPI_US` | duration of the previous ADS1299 read |
| `HEAP` | free heap, in case an allocation is involved |

**Read it as a discriminator:** a visible step **with** a `[GAP]` line at that
moment is a firmware stall; a visible step with **no** `[GAP]` line is analog. The
line is rate limited to 10 per second so a sustained stall cannot flood the link.

Work that used to run periodically during a capture has been removed, because each
of these could produce exactly that kind of step without any gap being recorded:

* No SD polling at all (see above).
* The **touch pad is no longer polled while streaming** - it needs an ADC1
  conversion, and the measurement owns the front end. It is read only when the
  stream is stopped.
* The **SD writer no longer wakes every 20 ms** when there is nothing to write
  (200 ms instead), so an idle card task costs 5 scheduler entries per second on
  the radio core instead of 50.

If steps persist with no `[GAP]` line, the cause is not the firmware's timing.
Two cheap experiments separate the remaining candidates:

1. **Build without Bluetooth** (`-DSZD_BT_ENABLED=0` in `platformio.ini`). The
   radio controller keeps running while the stack is enabled, and its RF bursts can
   couple into a high-impedance biopotential front end; if the steps vanish, that is
   the coupling path.
2. **Disconnect the card slot's ground and the generator** and repeat with shorted
   inputs. A step that survives with everything external removed points at the
   bias/reference network or the supply.

A gap figure of many seconds in the status bar used to be meaningless: `previousUs`
was a function-level static, so the first conversion after a stream restart counted
the whole time the stream had been stopped as one "gap". It is now reset when a
stream starts, so `late` and `gap` only describe the session you are looking at.

Capacity is taken from the card's own CSD (`SD.cardSize()`), not from
`SD.totalBytes()`. The latter is derived from the FAT volume's cluster chain and
under-reported a 16 GB card as 2.8 GB, which also made the free-space estimate
meaningless. All byte arithmetic is 64-bit for the same reason: 15,938,355,200
bytes does not fit in a `uint32_t`.

### What a card recording contains

The file is an **OpenBCI-compatible CSV**, the same layout the live window writes
(`%` header, EXG columns in microvolts, accelerometer, Marker, Timestamp), so
`gui_main.py` opens it with the normal loader and the two recordings can be
compared directly. Differences from a live file:

* **Switched-off channels are absent.** The header's `%Channel Map` lists the
  physical inputs that were on, so a mixed measurement costs space only for what
  it measures.
* It is written at the **converter rate** (before the Cyton+Daisy interleaving),
  so it holds every conversion while the live link carries half of them.
* `%Source = SD card (ESP32 internal recording)` marks where it came from.

### Record button (GPIO35)

A short press of a button on **GPIO35** (the `SWITCH` net) toggles card recording:
it loads `/SZD_CFG.TXT`, applies it to both ADS1299 (rate, decimation, SPI clock,
reference modes, per-channel gain/input/bias/SRB2, and which channels are on), then
starts recording. Pressing it again stops and closes the file. The session name from
the stored configuration is used for the file name.

The press is caught by a **rising-edge GPIO interrupt** (`attachInterrupt(...,
RISING)`), not by polling. A polled version could miss a press completely while the
radio flush task or the card writer held the CPU for tens of milliseconds, which is
why the button used to do nothing. The ISR only timestamps the edge and raises a
flag; debouncing is a 40 ms re-arm window inside the ISR, and all the SPI work
happens on the main loop, which owns the ADC bus.

If the button is already held at power-up there is no edge to catch, so the pin is
read once at start-up and the flag seeded from it.

**No external resistor is needed.** GPIO34-39 are input-only and have *no* pull
resistors *enabled at reset* (which matters for boot strapping), but they do
contain internal pull-up/pull-down resistors that firmware can switch on - so the
button uses `pinMode(35, INPUT_PULLDOWN)` and simply switches the pin to 3V3 when
pressed. The earlier note here claimed these pins cannot be pulled at all; that was
wrong. (GPIO37, the pin used previously, is not bonded out on a WROOM-32E at all.)

### Writing to the card without disturbing the radio

A card write is the only operation in the firmware that can block for tens of
milliseconds: the FAT layer may erase a sector, and a single 100-byte line can cost
10-50 ms. Three things keep that away from the Bluetooth link:

* **The writer runs below the radio.** `SD_TASK_PRIO` is 1 and `BT_FLUSH_TASK_PRIO`
  is 2, both on core 0, so the flush task always preempts the card writer. The
  writer previously sat at priority 5 and starved the radio until its ring was
  permanently 100 % - which is what made "record to the card" look like it had
  frozen the Bluetooth link.
* **Samples are buffered in RAM, twice.** A 512-sample queue absorbs conversion
  bursts (about half a second at 1 kHz) and a 4 kB byte buffer turns one card
  transaction per sample into one per ~40 samples. `queueSampleForSd()` never
  blocks: if the queue is full the sample is dropped and counted, never stalled.
* **The card clock is 10 MHz, not 4.** Transaction time, not byte count, dominates
  a card write, so the faster clock shrinks the stall the radio has to preempt.
* **The buffer is flushed when the queue runs dry** (20 ms of silence), so the tail
  of a recording reaches the card without waiting for 4 kB of company, and
  `closeSdFile()` flushes before closing.

The `[SD]` status line reports `QUEUED=` (samples waiting), `QPEAK=` (its
high-water mark), `QBUF=`/`BUFKB=` (bytes in the byte buffer and its capacity),
`WRITE_MS=` (the longest single card transaction), `RATE_KB=` (measured write
rate) and `SPIKHZ=`. A `QPEAK` near 512 with a non-zero `DROPPED` means the card
cannot keep up with the sample rate; the buffer is not the limit at that point.

### Tuning both paths at once (what does and does not help)

The two recordings are **independent consumers of the same stream**. The acquisition
task hands each conversion to both and never blocks on either, so "both fine
together" is a budget question, not a scheduling one. Measured from the firmware:

| Data rate (DR) | Live link must carry | Card must write (12 ch) |
|----------------|----------------------|--------------------------|
| 500 SPS  | 16.1 kB/s (132 kbit/s) | ~31 kB/s |
| 1000 SPS | 32.2 kB/s (264 kbit/s) | ~61 kB/s |
| 16000 SPS | 516 kB/s (4.2 Mbit/s) | way beyond the link |

**The baud rate is not a knob.** Bluetooth SPP is not a UART:
`btSerial.begin(SZD_BT_DEVICE_NAME)` takes no baud argument, and there is no
serial speed to raise. The Arduino API keeps the same shape as `Serial` for code
compatibility only.

### What the "Baud" dropdown in the window actually does

There are two different things called "baud" here, and only one of them is real:

| Port selected | What the baud affects |
|---------------|-----------------------|
| **USB** (`COM5  CP210x`, `CH340`, ...) | Real. It programs the USB-UART bridge and must match the firmware's `SERIAL_BAUD` (921600). |
| **Bluetooth** (`Standard Serial over Bluetooth link`) | **Nothing.** pyserial accepts the value and the Windows SPP driver discards it: the bytes travel over RFCOMM at the radio's rate, not at a bit clock. Any value works. |

The maximum in the list (921600) is therefore a *USB* limit, and it is chosen to
match the firmware. Setting a Bluetooth port to 115200 does not slow it down; the
data still goes as fast as the radio can carry it.

**This used to be a real bug.** The link budget
(`_ensure_link_budget()`) was computed from the selected baud whatever the port
was, so a Bluetooth port left on 115200 made the window believe the link could
carry only 314 packets/s and it would silently raise decimation - degrading the
measurement for a limit that did not exist. The window now detects a Bluetooth
port (`port_is_bluetooth()`, from the port's description or its `BTHENUM\...`
hardware id) and uses a byte-rate budget instead of a baud:

```python
BLUETOOTH_SPP_BYTES_PER_SECOND = 32 * 1024   # conservative ESP32 SPP figure
```

which works out to about **992 packets/s** - twice what 12 channels at 500 SPS
needs. The baud row also gains a note under it saying which of the two cases
applies, and the hardware summary line reads `Bluetooth SPP up to 992 packets/s`
instead of `Serial 921600 baud`.

### What the Bluetooth link's real maximum is

There is no baud number to quote because the limit is not a bit clock. The
physical ceiling is the radio mode:

| Layer | ESP32 classic Bluetooth |
|-------|--------------------------|
| Bluetooth version | 4.2, BR/EDR (Bluetooth Classic) |
| Air rate | 1 Mbps (basic rate), 2 Mbps (2-DH5 EDR), **3 Mbps (3-DH5 EDR)** |
| Ceiling for application data | roughly 2.1 Mbit/s (~260 kB/s), after L2CAP/RFCOMM framing |
| What the Arduino SPP path realistically sustains | well under that - the library's write path, the default 672-byte L2CAP MTU and the mutex per call dominate |

The honest answer for this board is therefore: **the radio can carry far more than
this application needs.** At 500 SPS the link must carry 16.1 kB/s; the ceiling is
over 200 kB/s. So the `rate=` and `need=` figures on the `#BT` health line are the
right thing to watch - they report what the link is *actually* doing on the
hardware in front of you, which is more useful than any nominal rate:

```
#BT mode=bt client=1 ... buf=2048 bufsize=32768 peak=9000 need=16500 rate=16200
```

`rate` at or above `need` with `peak` well below `bufsize` means the radio is
keeping up and there is nothing left to tune. If `peak` pins at `bufsize`, lower
the data rate, the channel count, or raise `SZD_BT_TX_POWER_DBM` for a better RF
link - not the baud.

What was actually done, and why:

* **SD SPI clock 4 MHz -> 10 MHz** (`SD_SPI_CLOCK_HZ`). This is the change that
  matters most. A card write is dominated by transaction time rather than byte
  count, so a faster clock directly shortens the window in which the writer holds
  the bus and the CPU - the exact contention with the radio. The SD specification
  allows up to 25 MHz in SPI mode. If a card refuses to initialise at 10 MHz the
  bring-up retries at 4 MHz and says so on the serial port, so a slow card costs
  speed, not the recording.
* **BT ring 24 kB -> 32 kB** (`SZD_BT_TX_BUFFER_BYTES`), which is about one second
  of the live link at 1 kSPS. This absorbs bursts - it does *not* raise average
  throughput. If the allocation fails the ring falls back through 24/16/8 kB
  instead of disabling Bluetooth. The host can still resize it at runtime with
  `Q<kb>Q`.
* **Flush chunk 512 -> 1024 bytes** (`BT_FLUSH_CHUNK`): half as many SPP write
  calls and congestion events for the same bytes.
* **Peak and rate reporting** on both paths (`#BT ... peak= rate= need=`, and the
  `[SD]` fields above), so "is it keeping up" is a number.

What will *not* help: enlarging the buffers further. Their only job is to ride out
a stall; once `peak` stays well under `bufsize` the link has headroom and extra
kilobytes buy nothing but heap pressure. The firmware now reports `HEAP=` on both
lines so a buffer that is too big for the remaining RAM is visible.

The live window shows both figures directly: the SD panel gains a line such as
`Card: 31 kB/s, longest write 11 ms, queue peak 40/512, SPI 10000 kHz`, and the
link line gains `peak 27% 31.2 kB/s need 32.2 kB/s`. Read them as:

* `queue peak` low and `DROPPED` zero -> the card is keeping up, whatever the rate.
* `queue peak` at the capacity -> lower the data rate or the channel count; a
  bigger queue only delays the first drop.
* `rate` at or above `need` with `peak` low -> the radio is keeping up.
* BT `buffer` or `peak` at 100 % -> the radio is the limit at that data rate.

### Aligning a card recording with a live one

Both files are written from the same stream, but they are opened by different things
so they never begin at the same instant. Opening the live file first narrows the gap
to a round trip, and a **shared marker** removes the guesswork entirely:

* the window sends the board's marker command (`` ` ``) as soon as both files exist,
* the board writes `Marker=1` into its next card row,
* the window writes `Marker=1` into its own next rows,
* **Tools -> Compare two recordings...** aligns on that marker and refines the offset
  by correlation within 200 ms of it.

Correlation alone cannot do this. A test tone or mains pickup correlates just as
well one period away, so a search over the whole file can report an offset that is
several periods wrong - which is what a 69.7 s "start offset" from two different
clocks looked like before.

The two files do not share a clock - the live file's Timestamp is seconds since the
host opened it, the card file's is the board's `micros()` since boot - so the
comparison never subtracts one from the other. It reports the offset measured from
the waveforms, and describes coverage ("B has 1.5 s extra at the start") instead of
calling a different start time a fault.

### SD card wiring

| Signal | GPIO | Note |
|--------|------|------|
| CS (DAT3) | 15 | MTDO strapping pin, but its internal pull-up is active at reset so CS idles high through boot |
| MOSI (CMD) | 13 | |
| SCLK | 14 | |
| MISO (DAT0) | **39** (SENSOR_VN) | input-only, not a strapping pin; internal pull-up enabled in firmware |

MISO must **not** be on GPIO12: that is MTDI, which latches the flash supply
voltage at reset (high selects 1.8 V flash for a 3.3 V part), and it is also why a
card on that pin caused intermittent "MD5 of file does not match data in flash"
upload failures. GPIO12 is now unused - leave it unconnected, and never pull it
high.

The SD specification asks for pull-ups on CS, CMD and DAT0. The ESP32 drives CS,
CMD and CLK as ordinary push-pull outputs, so those do not need external parts to
work; the resistors exist to give the bus a defined level before the firmware
configures the pins and while the card tri-states DAT0. On this board that is
covered by GPIO15's reset pull-up on CS and by the firmware-enabled internal
pull-up (≈45 kΩ) on MISO. If a particular card still refuses to initialise, add
10 kΩ from CS and MISO to 3V3 - those are the only two that matter.

### LED feedback while recording

The status LED becomes a recording indicator first and a transport indicator second,
because losing a recording matters more than not knowing which link is up:

| State | Pattern |
|-------|---------|
| Recording, USB host connected | **solid on** |
| Recording, Bluetooth host connected | on **950 ms**, off **50 ms** |
| Recording, no host connected | on 900 ms, off 50 ms, on 50 ms, off 50 ms (two quick off flashes) |
| Not recording, USB link | 500 ms square wave |
| Not recording, Bluetooth link | 125 ms square wave |

### Comparing the two recordings

`gui_main.py` -> **Tools -> Compare two recordings...** opens a window that takes
the live CSV and the card copy and reports:

* start and end offset between the two timelines (signed, in seconds), measured from
  where the waveforms line up, or from the shared marker when both files carry one,
* trace correlation at that offset, which is what proves both files carry the same
  measurement,
* any gap, duplicate timestamp or backwards time step in either file, with the
  missing sample count and the time of the first few gaps,
* whether B covers the whole of A, and how much extra data either file holds.

A different start time is reported as coverage, not as a fault: the two writers are
started by different things, so some offset is normal. The verdict only fails when
the traces do not correlate or a file lost samples inside its own span.

A sample-rate difference of an integer factor is handled rather than rejected: the
card copy keeps every conversion, so it is averaged down to the live rate before
the comparison. The tool is deliberately not part of the main layout - it answers a
bench question, so it opens on demand.

The board side is two commands plus a query; see the protocol notes.

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
GUI channel order. `Marker` is `0` except on the rows the window stamped when it
asked the board for a recording marker, which is what the two-recording comparison
aligns on.

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

* hardware: board profile, data rate, decimation, SPI clock, per-converter
  reference modes, all per-channel fields (replayed on connect),
* connection: last port, baud rate and whether to auto-connect on start,
* display: scale, window, row height, FFT points/window/smoothing/scale/log,
  FFT trace selection and end frequency, filters, accelerometer settings,
  visible channels, record-only-checked-channels,
* recording: session name, output folder and the measurement description
  (kind, connection, stimulus, reference mode),
* sweep: the generator VISA resource, amplitude, load, drive mode, frequency
  list, dwell rules and monitor channel, plus the manual panel's output mode,
  test frequency and the attenuator series/shunt resistors.

The analyzer keeps its own small store, `settings/analyzer.json`, holding the
export sections/formats and the measurement text.

### Reference channel (live window)

*Reference channel* in the dock selects the input wired across the applied
stimulus (a common-mode monitor, a crosstalk source or the gain stimulus). Choosing
one:

* gives it its own time series and FFT in the bottom-right panel, in **mV** with
  its own full-scale setting, instead of sharing axes with microvolt channels;
* **hides it from the main time series and the main FFT**, so it cannot flatten
  the traces you are actually looking at;
* forces that input to **PGA x1** - a volt-level stimulus on x24 clips the ADC;
* becomes the **CM monitor** of the automated sweep (one place decides it);
* is written into the recording as `Reference Channel` / `Reference Channel Gain`,
  so the analyzer preselects it and leaves it out of the stacked waveform.

The file carries a `version` field, so an older file adopts the newer defaults
once and then keeps your choices:

| Version | What it introduced |
|---------|--------------------|
| 2 | 0.1 / 150 Hz default band, auto-connect flag |
| 4 | 921600 baud, 500 SPS (250 Hz per channel), gain x24, normal input, SRB2, FFT log + auto scale, record only checked channels |
| 5 | per-converter reference modes, selectable SPI clock (2 MHz default) and output decimation (off by default) |

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

### Why the window never blocks on a port

Opening, listing and writing to a serial port all go through worker threads, and
that is deliberate:

* **Listing** runs in a background scanner. Asking Windows about a Bluetooth COM
  port can take seconds, and doing it on the GUI thread freezes the window with
  no CPU use at all.
* **Opening** happens on the reader thread, because a Bluetooth port's driver can
  take seconds to come up while it talks to the radio.
* **Writing** uses `write_timeout=0.5`, so a write to a port whose link has gone
  raises instead of blocking for ever. Without it, one dead Bluetooth link was
  enough to wedge the reader and freeze the whole window.
* **Auto-connect** never overrides a port you selected: if you pick the demo
  source or a specific port, that is what gets opened.

### What the connection label reports, and the retries

The window does not claim to be connected before the port is actually open. A
Bluetooth COM port can take seconds to come up, and Windows can block on a stale
pairing instead of failing, so:

| Label | Meaning |
|-------|---------|
| `Opening COM7 ...` | The open is in progress. Nothing is being claimed yet. |
| `Connected to COM7` | The driver returned a handle; the board is now probed for its version. |
| `<reason> Retrying in 2 s...` | The open failed or stalled, and another attempt is queued. |
| `<reason> Gave up after 4 attempts.` | No more automatic retries; the port is a real problem. |

* An **open watchdog** (`OPEN_WATCHDOG_MS`, 12 s) catches the case where the open
  neither succeeds nor fails. Without it the window simply looked idle.
* A **failed open is retried** up to `MAX_CONNECT_ATTEMPTS` (4) with
  `RECONNECT_DELAY_MS` (2 s) between attempts, because a stale Bluetooth pairing
  often succeeds on the second try. A failure *after* the board has answered is
  not retried - it means the link dropped mid-session.
* Two common failures get an explanation appended: `Access is denied` means
  another program holds the port (including a second copy of this window), and a
  missing port means the device disappeared and Refresh is needed.
* Closing the window always releases the port, even if the connect attempt failed
  or was abandoned - a port left open here is what makes the *next* run fail with
  "Access is denied".

### Scrolling cannot change a setting by accident

Scrolling a long panel used to be able to change whatever combo box or spin box
happened to be under the pointer. Now an unfocused value widget ignores the wheel
and lets the panel scroll instead; the value only responds to the wheel once you
have clicked it (so it has focus). This applies to both windows.

### Recording name and reference mode are independent

The reference is hardware state; the recording name is documentation. They are
deliberately **not** locked together, because a mixed reference (one converter
SRB1, the other true differential, plus per-input overrides) cannot be described
by one name token anyway, and forcing one from the other meant that naming a run
could silently reprogram the chip and wipe the per-input choices:

* Choosing a name with a reference (**Name...** -> Reference) records that
  reference for the report only. It does not write hardware and does not switch
  converters.
* Changing the reference in **Hardware settings** does not rename the session.
  The name stays exactly as typed.
* The measurement label shows the reference the **board** is actually in, and
  adds `name says X but the board runs Y - the recording stores the board state`
  when the two disagree, so a mismatch is visible instead of hidden.
* The recording header always carries the truth: `Reference Mode` plus
  `Reference ADS1`, `Reference ADS2` and `Reference Overrides` (for example
  `CH1:INN`) whenever the board is not uniform.

## Firmware notes that matter for this app

| Command | Meaning |
|---------|---------|
| `v` | version / device inventory (also stops streaming and re-applies defaults) |
| `~<0-6>` | ADS data rate (16000 ... 250 SPS, rejected above the serial or SPI limit) |
| `x<selector><digits>X` | per-channel power/gain/input/bias/SRB2 intent (SRB1 is owned by `MISC1`) |
| `N<0\|1\|2>` | board-wide reference mode: 0 = differential, 1 = SRB1, 2 = SRB2 |
| `n<converter><0\|1\|2>` | reference mode of one converter only |
| `b` / `s` | start / stop streaming |
| `d` | firmware defaults for all channels |
| `?` | register dump (serial monitor) |
| `P` | configuration readback for **Verify board configuration** (`[CFG]` lines) |
| `T` | Bluetooth status: device name, client, active link, TX power, buffer, counters, boots, uptime, CRC |
| `B` | Bluetooth event history (`#BTLOG` lines): boots, opens, closes with the `remote` flag |
| `Q<2-64>Q` | Bluetooth buffer size in kB, e.g. `Q32Q` |

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

### Power control (current-consumption testing)

*Device -> Power control...* opens a window that switches each subsystem on and off
so its current draw can be measured on the bench supply. The window only *switches*
and *reports*; the supply does the measuring.

| Group | What can be switched |
|-------|----------------------|
| **ADS1299** | Per converter: run / idle / standby, data rate, and PGA for every channel of that chip. Board-wide: hold `PWDN` low, hold `RESET` low, ADS1's clock output. Plus **Start/Stop stream** and **Restore converters to normal** |
| **Command link** | Send over the main USB link, or open the panel's **own** Bluetooth SPP connection |
| **Peripherals** | Accelerometer (power-down), SD card (released), synthetic test stream |
| **Bluetooth** | Stack on/off, transmit power in the controller's steps (-12 ... +9 dBm, read back from the controller), modem sleep |
| **ESP32 sleep** | Light sleep for N ms (continues, no reset) and deep sleep for N ms (restarts) |

Opening the panel **only reads**. It sends `|status;` and nothing else, so a current
measurement is never disturbed just by looking at the board. Anything that changes
state is an explicit click; **Restore converters to normal** (`restore=1`) is there
for when a converter really was left in a test state, and the panel prompts for it
when a linked converter reports no DRDY edges.

`PWDN` and `RESET` are **tick-to-disable**: clear means the pin is high and the
converters run normally, which is how the board boots. Ticking either box drives the
shared pin low. That is the only useful direction for a current test - nobody wants
to power the converters *on* by hand - and it makes the boxes read the same way as
the accelerometer, card and radio boxes next to them.

**The Bluetooth TX power only moves the current reading while the radio is
actually transmitting.** With no SPP host attached the controller idles; hold the
link open and let data flow over it, then compare `-12` with `+9` dBm.

The **Controller holds** line under the power combo is the read-back: Apply sends
the request and the board answers with `esp_bredr_tx_power_get()`, so the panel
shows *confirmed* when the controller took the level, and *"the controller did not
take +9 dBm"* when it clamped it. The level in the status line is that read-back
too, never the echo of the request.

**The panel does not trust a setting, it proves it.** The *Board reports* block shows
per converter: whether it answers on SPI, its device ID, how many registers differ
from what was programmed, and **how many DRDY edges it produced in a 60 ms window**
(`DRDY 59/60 in 60 ms  converting`, or `DRDY 0/60 ... NO DATA`). The packet counter
(`packets sent`) shows whether the stream is really delivering. A converter can be
perfectly configured and still produce nothing - `DRDY 0` is the field that
separates the two.

**The Bluetooth connection is the panel's own.** The board leaves Bluetooth mode when
nothing is using the SPP link, so measuring the radio's current means holding that
port open from a program. The panel therefore opens its own connection to the SPP
COM port - a different port from the cable, which the main window already holds - and
can send every command over either transport. It works with the USB link connected,
disconnected, or both.

Three things are deliberately explicit rather than hidden:

* **`PWDN` and `RESET` are wired to both converters on this board**, so those two
  are board-wide and the window says so instead of pretending to be per-chip. The
  per-chip mode is real: it goes through the `START`/`STOP`/`STANDBY` SPI opcodes,
  not the shared `START` pin.
* **`CLK_EN` is the clock output driver, not the oscillator.** The ADS1299's
  oscillator always runs; `CLK_EN` only drives it out on the CLK pin for the second
  converter. Turning it off stops ADS2 and saves the output driver, nothing more.
* **Light sleep can continue the session; deep sleep cannot.** Light sleep keeps
  RAM and the program counter, so the board resumes where it left off - the UART
  divisor is refreshed on wake because light sleep changes the APB clock. Deep sleep
  powers down everything outside the RTC domain and *always* restarts the chip when
  the timer expires.

Both sleep modes stop streaming and (for light sleep) stop the radio first: an
active Bluetooth controller wakes the chip immediately, so light sleep with the
radio running would measure nothing.

The window **never polls**. It reads the state once when it opens and again only on
*Refresh*, because a periodic query would put card and SPI traffic into the very
measurement the window exists to make. Every control acknowledges with a `[PWR]`
line, so the panel can show which request was refused and why.

The synthetic test stream is worth knowing about: with `test` on, the firmware
generates packets at the configured rate **without touching the converters**, and it
starts the stream itself. Without that the setting was accepted and nothing was ever
sent, because the generator lives inside the streaming task.

### Bluetooth in the firmware

The radio is a classic-Bluetooth SPP device called `SZD_EEG`, started next to the
USB port at boot. Everything the firmware prints goes through one `HostLink`
object, so ~120 existing `Serial.print` sites now reach whichever transport is
active without knowing Bluetooth exists.

| Build option | Default | Meaning |
|--------------|---------|---------|
| `SZD_BT_ENABLED` | `1` | Compile the radio in at all. `-DSZD_BT_ENABLED=0` (or `build_flags = -DSZD_BT_ENABLED=0` in `platformio.ini`) leaves the USB transport only; the firmware then uses 383 kB of flash instead of 1.20 MB (29% vs 92% of the app partition), which matters if you need the room |
| `SZD_BT_DEVICE_NAME` | `"SZD_EEG"` | Name the PC pairs with |
| `SZD_BT_TX_POWER_DBM` | `4` | BR/EDR power ceiling. The controller only has -12/-9/-6/-3/0/+3/+6/+9 steps, so 4 selects the +3 dBm step (the strongest one that does not exceed it) and 6 selects +6 dBm; the chip maximum +9 dBm is not needed for a bench link |
| `SZD_BT_TX_BUFFER_BYTES` | `32768` | Radio ring buffer at boot (runtime adjustable with `Q<kB>Q`, up to 64 kB). Falls back through 24/16/8 kB if the allocation fails |
| `SZD_BT_TOUCH_PIN` | `32` | Touch pad that switches to Bluetooth mode |
| `SZD_BT_TOUCH_TRIGGER_RATIO` | `0.75` | A touch is detected when the reading falls to this fraction of the calibrated baseline |

Three design points keep the radio from disturbing the converters:

1. **The conversion path never writes to the radio.** Packets go into a ring
   buffer; a separate `btFlush` task pinned to core 1 at priority 2 (the
   acquisition task runs on core 0 at priority 20) drains it. A congested link
   can only fill the buffer.
2. **Overflow drops whole packets, never partial ones.** Eviction happens in
   33-byte steps so the frame that remains in the buffer is always intact; the
   drop is counted and shows up as `PER` in the GUI and as a gap in the packet
   counter.
3. **Text replies bypass the ring** and are written straight to the stack. The
   ring therefore only ever contains fixed-size frames, which is what makes (2)
   safe.

The stream itself is still the plain 33-byte Cyton frame, identical to USB, so
the recorder, the CSV format and the analyzer are transport-agnostic. Integrity is
reported by the `#BT` line: the CRC-16 of the last 32 packets plus the drop
counter, both of which the PC can verify/recompute. SPP already retransmits lost
ACL packets at the baseband layer, so the only loss that can occur is the
buffer overflow described above - which is counted rather than hidden.

## Troubleshooting

| Symptom | What to check |
|---------|---------------|
| Single-sample spikes on all channels | Those were torn RDATAC frames. Reflash the current firmware and keep the ADS rate within the link budget; check the `corrupt` counter after stopping the stream |
| "No data from CH n" warning | That channel reads exactly zero: press **Verify board configuration** and look for the CHnSET mismatch, or check that the channel is powered on / the electrode path is connected |
| "input saturated" warning | The differential input is at the ADC rail. Check that the reference mode matches the wiring (SRB2 selected but the reference on SRB1 is the classic mistake), then the bias/DRL electrode and the gain |
| Channel values frozen at one number | The firmware used to hold the last sample when a converter stopped answering. It now publishes zeros after four missed frames, so a dead SPI/DRDY path shows up as "no data" instead |
| "Configuration mismatch" after verify | The listed registers did not stick. A whole chip missing points at CS/SPI wiring; one channel points at that channel's input or power bit |
| Bluetooth port does not appear | Pair with `SZD_EEG` first (Settings -> Bluetooth & devices -> Add device); the COM port only exists after pairing. Touch GPIO32 to force Bluetooth mode and confirm with the fast LED blink |
| Bluetooth pairs, then the port opens but the board never answers | Normal Windows behaviour for one of the two ports: Windows creates an incoming and an outgoing *Standard Serial over Bluetooth link*, and only the outgoing one carries data - try the other. If both stay silent, read the `[BT]` lines over the USB port; the recorder also reports `stale drops` when the board had to repair a dead client |
| Bluetooth device disappears and pairing must be redone | Remove `SZD_EEG` in Settings -> Bluetooth & devices and pair again. The firmware now clears a stuck SPP client by itself, and `[BOOT] reset reason` on the USB port tells you whether the board restarted (a brownout is the usual cause when the radio is involved) |
| `PER` climbs on Bluetooth | The radio cannot keep up: lower the ADS rate to 250 SPS or enlarge the buffer with `Q48Q`. Also check the distance/obstacles - and that the board is not sitting next to the 2.4 GHz of a Wi-Fi access point on the same channel |
| Status bar shows no `BT` segment | The board is on USB. It appears only while a Bluetooth host is connected and streaming |
| "The board did not answer the version request" | wrong port, port already open in another program, or the board is still running the self test |
| Some channels show `rejected` | those channels do not exist on the fitted chip (padding), or the firmware refused the value |
| Traces flat / all zeros | input type set to `Shorted`, or channels powered down (`On` unchecked) |
| Accelerometer reads 0.000 g | LIS3DH not answering - the panel shows a note after two seconds |
| Accelerometer never alarms | press `Calibrate from rest`, then lower `Motion alarm` |
| Recording has fewer channels than expected | the CSV stores only real channels; padding channels are intentionally dropped |
| Rate drops back to 250 SPS | the selected rate needs more bandwidth than the serial link provides |
| Analyzer uses the wrong sample rate | the file header is read first; the import dialog shows the detected rate and lets you override it |