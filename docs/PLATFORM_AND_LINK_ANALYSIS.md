# Platform and link analysis

Long-term reference for two questions that keep coming back:

1. **Is the Arduino framework holding this firmware back?** What would ESP-IDF
   actually buy, and would it enable a much higher per-converter rate?
2. **Should the stream go over Wi-Fi instead of the cable or Bluetooth?** Can it
   carry the peak packet rate, and what does it cost in RAM?

Every number below is labelled with where it comes from:

| Tag | Meaning |
|-----|---------|
| **[datasheet]** | from `docs/ADS1299_FULL_DATASHEET.txt` or the ESP32 documentation |
| **[core]** | read out of the installed framework/library source on this machine |
| **[measured]** | reported by this board (build output, `#DX`, `#BT`, `[SD]` lines) |
| **[derived]** | arithmetic on the above |
| **[estimate]** | a range I would expect but which has *not* been measured here - treat it as a hypothesis with a test attached |

---

## 1. What the firmware runs on today

| Item | Value | Source |
|------|-------|--------|
| Framework | Arduino-ESP32 **2.0.17** (`ESP_ARDUINO_VERSION` 2.0.17) | [core] |
| Underlying ESP-IDF | **4.4.7** (`ESP_IDF_VERSION_MAJOR/MINOR/PATCH`) | [core] |
| Target | `esp32dev`, ESP32-WROOM-32E class, 4 MB flash | [measured] `platformio.ini`, build |
| DRAM | 327 680 B, static use 56 736 B (17.3 %) | [measured] build |
| App partition | 1 310 720 B, used 1 250 685 B (**95.4 %**) | [measured] build |
| PSRAM | **none** on WROOM-32E (support is compiled into the core, the chip is absent) | [core] `CONFIG_ESP32_SPIRAM_SUPPORT=y`, [measured] DRAM only |
| Partition table | default (single 1.25 MB OTA-slot scheme) | [measured] no `partitions.csv` |
| Host link | CP2102N UART at **3 000 000 baud** | [measured] `SERIAL_BAUD` |
| Sample packet | **33 B**: start, sequence, 8 x 24-bit slots, aux, end | [measured] `publishConversion()` |
| ADS1299 interface | SPI, SCLK factory default 2 MHz, **capped at 8 MHz by firmware** | [measured] `ADS_SPI_CLOCK_MAX_HZ` |
| ADS1299 max rate | **16 kSPS** (data-rate code 000) | [datasheet] line 554 |
| ADS1299 max SCLK | **20 MHz** (`tSCLK` = 50 ns) | [datasheet] line 737 |

Two things follow immediately:

* The rate table already offers **16 000 SPS**, and the chip supports it
  [datasheet]. Nothing in the firmware forbids selecting it.
* The framework is **already ESP-IDF** - Arduino 2.0.17 is a set of C++ wrappers
  over IDF 4.4.7. "Switching to ESP-IDF" therefore means *dropping the wrappers and
  moving from IDF 4.4 to IDF 5.x*, not changing the layer underneath the scheduler.

---

## 2. Where the limits actually come from

### 2.1 The information floor

Twelve real channels at 24 bit are **36 B of payload per conversion** (one
conversion = one sample instant across both converters):

```
16 000 SPS  x 36 B = 576 000 B/s = 4.61 Mbit/s   <- irreducible at 16 kSPS
 8 000 SPS  x 36 B = 288 000 B/s = 2.30 Mbit/s
 4 000 SPS  x 36 B = 144 000 B/s = 1.15 Mbit/s
```

That floor is what every link below has to clear. No framing trick and no
framework gets under it.

### 2.2 The current wire format

Each 33-byte packet carries 8 slots x 3 B = 24 B of slot data plus 9 B of framing
(start, sequence, six aux bytes, end). Two packets - one per converter - are emitted
per conversion, so a conversion costs **66 B on the wire for 36 B of real signal**
(36 B of padding is the four unused slots of the daisy emulation).

```
 4 000 SPS/ADS ->  8 000 packets/s -> 264 000 B/s  [measured] pkt_rate=8000
```

### 2.3 Link budgets

| Link | Capacity | Packets/s at 33 B | SPS/ADS it can carry | Status |
|------|----------|-------------------|----------------------|--------|
| CP2102N UART, 3 Mbaud | 300 000 B/s raw, firmware budgets 90 % = 270 000 B/s | ~8 100 | **~4 000** | [measured] what you run today, 88 % duty |
| Bluetooth SPP, library queue at 330 B | **88-99 KiB/s** measured | 2 700-3 000 | ~1 400 | [measured] benchmark |
| SD card over SPI at 20 MHz | ~2.5 MB/s bus ceiling; the formatting CPU and FAT overhead bite first (see 2.6) | n/a | ~4 000-8 000 [estimate] |
| Wi-Fi 802.11n, UDP | 10-30 Mbit/s achievable payload = 38 000-113 000 packets/s | 38 000-113 000 | limited by the ADC, not the link | [estimate] needs measuring |
| Native USB (ESP32-S3 only) | > 10 Mbit/s | > 37 000 | 16 000 | [estimate] different MCU |

**This is the whole story of the last few sessions.** At 4 000 SPS/ADS the UART is
88 % full; at 8 000 SPS/ADS it would need 528 000 B/s and cannot be met; at
16 000 SPS/ADS it needs 1 056 000 B/s, i.e. **10.6 Mbaud**. The cable was never
going to carry it, which is why raising the SPI clock and the timing work stopped
paying off above 4 000.

The firmware already knows this: `maxSustainedPacketRate()` budgets 90 % of the baud
(2.7 Mbit/s) over 330 bits per packet = **8 181 packets/s**, and
`maxUsableSampleRate()` takes the smaller of that and the SPI limit - which at 8 MHz
is 16 666 SPS. The serial figure wins, so the ceiling is 8 181 / 2 = **4 090 SPS per
converter**, and the measured 4 000 sits exactly there [measured].

### 2.4 What the cable could carry with a tighter format

| Wire format | Bytes per conversion | Max SPS/ADS at 300 000 B/s |
|-------------|----------------------|----------------------------|
| Today: 2 x 33 B packets | 66 | 4 545 (you run 4 000) |
| 12 channels + aux, packed | 45 | 6 666 |
| 12 channels packed, aux on its own slow channel | 38 | 7 894 |
| Absolute floor (36 B, zero framing) | 36 | 8 333 |

So a **packed binary format is worth roughly a 1.7x rate increase over the existing
cable** without touching the hardware - and it is the only lever left that large on
this link. 16 kSPS/ADS needs **5.76 Mbaud minimum**, which the CP2102N cannot do
(3 Mbaud max); a CH343 or FT231X bridge (6 Mbaud) plus a packed format would just
barely reach it.

**The trade-off:** the 33-byte layout exists so the *stock OpenBCI GUI* and any other
OpenBCI tool can read the board unchanged (see `OPENBCI_COMPATIBILITY.md`). A packed
format would be this project's own protocol, understood only by `live_gui.py` /
`gui_main.py` unless a converter is written. The sensible shape is therefore two
modes: keep the 33-byte packets as the default (compatibility), and enable a packed
"raw fast" mode above ~4 kSPS where compatibility is already lost to the rate.

### 2.5 SPI duty per conversion

One conversion reads two frames of 216 bits (24 status + 8 x 24 data), so 432 bits
must be clocked inside one conversion period:

| SPS/ADS | Period | at 8 MHz | at 16 MHz | at 20 MHz (chip max) |
|---------|--------|----------|-----------|----------------------|
| 4 000 | 250 us | 54 us (22 %) | 27 us (11 %) | 21.6 us (8.6 %) |
| 8 000 | 125 us | 54 us (43 %) | 27 us (22 %) | 21.6 us (17 %) |
| **16 000** | **62.5 us** | **54 us (86 %)** | 27 us (43 %) | 21.6 us (35 %) |

At 8 MHz - the firmware's current ceiling - 16 kSPS would leave 8.5 us per period
for everything else. That cannot work. **The SPI clock cap is a firmware constant,
not a datasheet limit**: the ADS1299 accepts 20 MHz, which would leave 35 % duty at
16 kSPS. Raising `ADS_SPI_CLOCK_MAX_HZ` to 16-20 MHz is therefore a prerequisite for
any rate above ~6 kSPS, and it is independent of the framework.

### 2.6 The next wall: the SD text format

A CSV row for 12 channels is roughly **144 B**:

```
 4 000 SPS -> 576 kB/s   fine at 20 MHz SPI
16 000 SPS -> 2.3 MB/s   at the edge of the bus and of the formatting CPU
```

So even if the data never leaves the board, **16 kSPS/ADS needs a binary log, not
CSV**. The analyzer already parses raw int32 columns, so a sidecar binary format
with a CSV header remains loadable.

---

## 3. What rates are reachable, and what each one needs

| Target | SPI SCLK | Uplink | Storage |
|--------|----------|--------|---------|
| 4 000 SPS/ADS (today) | 8 MHz ok | 3 Mbaud UART at 88 % | CSV at 20 MHz SPI |
| 8 000 SPS/ADS | 16 MHz | **UART cannot** with today's format (528 kB/s); a packed 45 B format needs 360 kB/s = **3.6 Mbaud**, so a 6 Mbaud bridge (CH343/FT231X) covers it | CSV ok |
| 16 000 SPS/ADS (chip max) | 16-20 MHz | **UART cannot** even packed (576 kB/s = 5.76 Mbaud minimum); Wi-Fi / native USB / SD-only | binary log |

160 000 SPS per converter is not a thing on this part: the ADS1299 stops at
16 kSPS [datasheet]. Ten times that would be a different AFE entirely.

A note for measurements, not just throughput: the datasheet's own noise table at
code 000 (16 kSPS) is worse than at 250 SPS, because the digital filter bandwidth
widens. High rate is for timing/EMG-style work, not for noise characterisation.

---

## 4. ESP-IDF: what it would and would not give

### 4.1 Gains that map onto this codebase

| Area | Arduino 2.0.17 today | ESP-IDF 5.x | Value here |
|------|----------------------|-------------|------------|
| **SPI** | `spiTransferBytesNL()` is a **register loop**: 16 words into `SPI.data_buf`, busy-wait on `cmd.usr`. No DMA path in `SPIClass` at all [core] | `spi_master` with `SPI_DMA_CH_AUTO`, *and* the choice of `spi_device_polling_transmit()` for short frames | At 16 kSPS the acquisition task would spin 86 % of the time; with IDF it can block on a semaphore and let the other task on that core run. For the 27-byte frame, **polling is faster than DMA** - the win is that you choose per transfer instead of having no choice |
| **UART** | `HardwareSerial::write()` blocks on the 128-byte TX FIFO | `uart_driver_install()` with a TX ring and an event queue, `uart_wait_tx_done()` | Replaces the hand-rolled USB ring in `HostLink`; the same discipline, but with real backpressure instead of `availableForWrite()` polling |
| **SD** | `SD`/`File` over the same `SPIClass`, `transferBytes` + a software CRC16 per sector [core] | `esp_vfs_fat_sdspi_mount()` (DMA-backed `spi_master`) or `sdmmc_host_init()` | Cleaner, and DMA for free - but see 6 for why the pin map blocks the fast SDMMC mode |
| **Memory** | fixed `sdkconfig` baked into the prebuilt core; no `heap_caps` tracing UI | full Kconfig, `heap_caps_*`, `heap_trace_*`, per-capability pools | **The largest long-term win**, given that memory - not CPU - has caused every failure in the last three sessions |
| **Flash** | `CONFIG_BT_ENABLED=y` is compiled into the core whether you use it or not; app partition 1.25 MB, **95.4 % full** [measured] | `CONFIG_BT_ENABLED=n` when unused, custom partition table | Both frameworks allow a bigger partition (3 MB app on a 4 MB part) - that alone removes the flash pressure |
| **Bluetooth** | `BluetoothSerial` wrapper with its own quirks (330-byte `SPP_TX_MAX`, a malloc per chunk, congestion accounting you had to work around) | direct Bluedroid SPP API, or `CONFIG_BT_BLUEDROID_ENABLED=n` + NimBLE for BLE-only | Removes the layer whose bugs cost the most debugging time |
| **Diagnostics** | `Serial.printf`, no coredump | `idf.py monitor` with decoded backtraces, coredump-to-flash, `idf.py size-components` | Would have shortened several debug cycles here |
| **Timers/ISR** | available through IDF passthrough, undocumented in Arduino docs | `esp_timer` (us), `ESP_INTR_FLAG_IRAM`, task watchdog per task | Already used indirectly; no new capability |

### 4.2 Costs

* **Rewrite of the peripheral layer.** `setup()/loop()` become `app_main()`; `SPI`,
  `Wire`, `SD`, `BluetoothSerial`, `HardwareSerial`, `Print/Stream` all disappear.
  In this codebase that is the `HostLink`, `ADS1299` bus helpers, `bringUpSdCard`,
  the LIS3DH I2C path, the SD writer's `File` calls and the BT SPP plumbing -
  roughly 500-700 lines out of ~7 800, but they are the lines the timing work
  depends on.
* **Re-validation of the acquisition timing.** Every conclusion in *Acquisition
  timing* in `LIVE_RECORDER.md` would have to be re-measured. The scheduling
  structure (acquisition alone on core 1, everything blocking on core 0) survives
  unchanged, because it is FreeRTOS, not Arduino.
* **The rest is plain C++** and ports as-is: packet framing, register tables, the
  `|` command parser, the configuration state machine, the SD row formatter.
* PlatformIO supports it directly (`framework = espidf`), so tooling is not a
  blocker.
* Effort, honestly: **days to weeks**, dominated by re-verification on hardware,
  not by typing.

### 4.3 Does ESP-IDF enable 16 kSPS/ADS?

**No - not the framework, and not the rate.** The blockers are in this order:

1. **Uplink** - 16 kSPS/ADS needs 1.06 MB/s with the current format, 576 kB/s at the
   absolute floor, against a 300 kB/s cable. No framework changes that.
2. **SPI clock** - needs 16-20 MHz instead of the current 8 MHz. That is a
   one-constant firmware change plus verification of the register-writing holds
   that already exist.
3. **Storage format** - CSV at 16 kSPS is 2.3 MB/s. Needs a binary log.
4. **CPU/memory** - the only place IDF genuinely helps, and by then it is the
   smallest of the four.

So: **IDF is worth doing for memory control, SPI DMA choice, and getting rid of the
`BluetoothSerial` layer - but it does not unlock a higher rate by itself. The rate
is gated by the link and by the SD format.**

---

## 5. RAM: where it goes, and what can be done about it

### 5.1 The measured picture

| Consumer | Bytes | Source |
|----------|-------|--------|
| DRAM total | 327 680 | [measured] |
| Static (.data/.bss, including the 4 kB SD write buffer) | 56 736 | [measured] |
| Free heap at boot | ~228 000 | [derived] from 41 136 free while streaming |
| Bluedroid, client connected | **152 744** | [measured] |
| Bluetooth transmit ring | 8 192 with a card, up to 32 768 without | [measured] |
| USB transmit ring | 8 192 | [measured] |
| Task stacks (acq, SD, LIS3DH, BT flush, USB flush) | ~19 000 | [derived] |
| Free heap while streaming over USB | 41 136 | [measured] |
| Free heap while streaming over Bluetooth | 35 588 | [measured] |

Bluedroid alone is **two thirds of the free heap**. That is why every memory
problem has been a Bluetooth problem.

### 5.2 The board cannot be given more RAM

ESP32-WROOM-32E has **no PSRAM**. The Arduino core is already built with
`CONFIG_ESP32_SPIRAM_SUPPORT=y` [core], so a **WROVER module** (4 MB PSRAM) would
work by adding `-DBOARD_HAS_PSRAM` and `board_build.arduino.memory_type`. PSRAM on
the classic ESP32 is DMA-capable, so it could hold the SD queue, the Wi-Fi buffers
or a binary log buffer - but it cannot hold ISR code or be used at 20 MHz SPI
without contention.

### 5.3 What IDF gives you beyond that

* `heap_caps_malloc(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL, n)` - say *where* a buffer
  must live instead of hoping.
* `heap_caps_get_largest_free_block()` - already used via `ESP.getMaxAllocHeap()`;
  IDF exposes it per capability, so "26 kB free but 6 kB contiguous" becomes visible
  instead of mysterious.
* `heap_trace_start()` / `heap_trace_dump()` - answers "who took the 152 kB" with a
  call site, on the device, without guessing.
* **menuconfig**: turn off Bluetooth, trim lwIP, drop the log level, shrink the main
  task stack. The Arduino core's `sdkconfig` is fixed, so none of that is currently
  reachable.

---

## 6. DMA: exact status per peripheral

| Path | Arduino 2.0.17 | ESP-IDF 5.x | Does DMA help *here*? |
|------|----------------|-------------|----------------------|
| ADS1299 SPI (27 B/frame) | **No DMA.** `spiTransferBytesNL` copies 16 words into `SPI.data_buf` and busy-waits [core] | `spi_master` DMA or polling, per transfer | Marginally: the wire time is unchanged, only the CPU is freed. At 16 kSPS that is 43 % of a core at 16 MHz SCLK, which is worth having |
| SD over SPI (512 B sectors) | **No DMA.** Same `SPIClass`; plus a software CRC16 per sector [core] | `sdspi` on `spi_master` -> DMA | Not much: 512 B at 20 MHz is 205 us, one transaction per ~40 rows. At 4 000 SPS that is a few percent of the writer's core |
| SD over SDMMC (4-bit) | `SD_MMC` library exists | `sdmmc_host_init()` with IDMA | **Yes, enormously** (20 MB/s class instead of 2.5) - but see the pin conflict below |
| UART TX | FIFO + interrupt; `write()` blocks | Ring buffer + event queue; **still no public TX DMA on ESP32 classic** | No |
| I2C (LIS3DH) | interrupt-driven | interrupt-driven | No - 6 bytes at 25 Hz |
| Bluetooth | Controller DMA, opaque | Same | No |
| Wi-Fi | Driver DMA, opaque | Same | No |

**The SDMMC pin conflict.** SDMMC on the ESP32 uses *fixed* pins: 1-bit is
CLK=14, CMD=15, D0=2; 4-bit adds D1=4, D2=12, D3=13. This board's SD wiring already
sits on three of them - SCLK=14 (CLK), CS=15 (CMD), MOSI=13 (D3) [measured] - but
**D0 is GPIO2, which is `PIN_ADS_PWDN`**, and D1 would be GPIO4 (`PIN_ADS2_CS`).
So 4-bit SDMMC is impossible without a respin, and even 1-bit needs one added wire
plus moving ADS PWDN off GPIO2. SPI is the only option on this pin map, which caps
the card at roughly 2.5 MB/s - and that in turn caps the CSV rate near 8 kSPS per
converter.

---

## 7. Wi-Fi as the uplink

### 7.1 Throughput arithmetic

| Case | Payload needed | Datagrams/s at 1472 B | Airtime incl. IP/UDP/802.11 headers |
|------|----------------|----------------------|-------------------------------------|
| 4 000 SPS/ADS, current format | 264 000 B/s = 2.1 Mbit/s | 179 | ~2.2 Mbit/s |
| 16 000 SPS/ADS, current format | 1 056 000 B/s = 8.5 Mbit/s | 717 | ~8.8 Mbit/s |
| 16 000 SPS/ADS, packed (45 B/conv) | 720 000 B/s = 5.8 Mbit/s | 489 | ~6.0 Mbit/s |

Overhead per datagram is roughly 8 B LLC + 20 B IP + 8 B UDP + 24-34 B 802.11 MAC
(~60 B), plus an ACK per frame. Batching matters: sending one 33-byte packet per
datagram wastes more than half the airtime, so **batch 32-44 packets into one
datagram** (1056-1452 B) and 16 kSPS needs only ~700-1000 datagrams/s.

**Can it do 32 000 packets/s?** On arithmetic, yes: ~9 Mbit/s of payload against an
802.11n HT20 PHY of 72 Mbit/s, and ESP32 boards typically reach **10-30 Mbit/s of
UDP payload** [estimate]. That is a real margin, unlike Bluetooth's 88-99 KiB/s.
But it is not free:

* **UDP only.** TCP retransmits and head-of-line blocking turn a 62.5 us sampling
  cadence into jitter you cannot recover.
* **Modem sleep must be off** (`esp_wifi_set_ps(WIFI_PS_NONE)`), or latency jumps to
  beacon intervals (100 ms+).
* Single-stream HT20, MCS7, and a quiet channel. Expect to lose packets on a busy
  2.4 GHz band, and design the receiver to count gaps (the host already resyncs on
  the start byte).
* The receiver must handle ~1 000 datagrams/s without per-packet allocation.

### 7.2 RAM cost - Wi-Fi is *cheaper* than Bluetooth here

| Stack | Approximate heap | Source |
|-------|------------------|--------|
| Bluedroid + SPP, client connected | **152 744 B** | [measured] |
| Arduino core's Wi-Fi defaults: 8 static RX + 8 static TX buffers x 1600 B, 32 dynamic RX buffers, driver state | **40-70 kB** | [core] `sdkconfig` + [estimate] |

So switching the wireless link from Bluetooth to Wi-Fi **frees roughly 80-110 kB**,
which is two to three times the largest SD queue this board can currently afford.
Wi-Fi is not the RAM risk; running **both** is (152 + 45 kB and coexistence CPU) and
is not viable at high rates on this part.

One more caution: `CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED=y` and `RX_BA_WIN=6` are on in
the core [core], which is good for throughput and adds reorder buffering to design
for.

### 7.3 What to tune if you go this way

1. `esp_wifi_set_ps(WIFI_PS_NONE)` - mandatory.
2. Batch 32-44 packets per UDP datagram; a single `sendto` per sample will not do.
3. Static IP, no DHCP renewals mid-recording.
4. Send from a task on **core 0**; never from the acquisition task.
5. A ring buffer between acquisition and the socket task, sized like the SD queue
   (the same adaptive allocation applies).
6. Count and report drops per second (`#DX` style) - a wireless link that silently
   loses 2 % is worse than a slower wired one.

### 7.4 Risks for a microvolt front end

* 2.4 GHz bursts couple into a high-impedance input, and Wi-Fi transmits at up to
  +20 dBm versus Bluetooth's +9 dBm. Expect to see it in the noise floor.
* Peak current draw of a Wi-Fi TX burst (~200-300 mA peaks) will move the supply
  unless the regulator and decoupling are good; that shows up as a step at the
  transmit instant.
* Both are measurable with tools that already exist: the noise tab (0.1-250 Hz in
  three bands) and the accelerometer/`[GAP]` machinery.

### 7.5 Alternatives to Wi-Fi that are cheaper in every sense

| Option | Rate | Cost |
|--------|------|------|
| **Pack the wire format** (45 B/conv) | 6 600 SPS/ADS on the *existing* cable | firmware only, no hardware and no RF |
| **SD-only high-rate capture** | up to ~8 kSPS/ADS as CSV, more as binary | already implemented, no link involved |
| **ESP32-S3 with native USB** | > 30 000 packets/s | different MCU/module, rewrites the host link but keeps everything else |
| **6 Mbaud bridge (CH343/FT231X)** | 16 kSPS/ADS with a packed format | board respin of the USB-UART only |

---

## 8. Recommendation, in order

1. **Pack the wire format before touching the platform.** It is the only change that
   raises the achievable rate on the hardware you already have (~1.7x), and it is
   confined to `publishConversion()` plus the host parser.
2. **Raise the SPI clock cap to 16-20 MHz** and verify with the register read-back
   machinery that already exists. Without it nothing above ~6 kSPS is possible.
3. **Keep the CSV for ≤ 4 000 SPS; add a binary log for anything above.** CSV is
   2.3 MB/s at 16 kSPS and will not survive.
4. **Then choose the high-rate uplink:** Wi-Fi UDP if wireless is acceptable (and it
   *frees* RAM), S3 native USB if a module change is acceptable, SD-only if nothing
   has to leave the board live.
5. **Only then consider ESP-IDF** - and do it for the memory control, the SPI DMA
   choice and the removal of the `BluetoothSerial` layer, not because it raises a
   rate. Split it: the analyzer and the host protocol do not change at all.
6. Independent of all of the above: **use a bigger app partition** (3 MB instead of
   1.25 MB) so flash stops being at 95 %.

---

## 9. Experiments that would settle the open questions

Each is cheap enough to run before committing to a rewrite.

| Question | Test | What to record |
|----------|------|----------------|
| Is the UART really the 4 000 SPS wall? | Run 4 000 SPS for 60 s and read `usb_drop`/`dropped` in `#DX`; then rerun with the host reading *nothing* | Drops appear only when the host drains the port => the link, not the acquisition |
| Does Wi-Fi carry 32 000 packets/s? | UDP sink on the PC, 1 056 B datagrams, 1 000/s, 10 minutes | Loss %, jitter, and `esp_wifi` heap use from `[HEAP]` |
| What does Wi-Fi cost on this board? | 30-line sketch: `esp_wifi` STA + lwIP, no application | `ESP.getFreeHeap()` before/after - settles the 40-70 kB estimate |
| Does 16 MHz SCLK stay clean? | Set `ADS_SPI_CLOCK_MAX_HZ` to 16 MHz, run the existing read-back verification loop for an hour | Channel-1 register rejects, `corrupt`, `spi_max_us` |
| Is the SD card the limit at 4 kSPS? | 4 000 SPS to card only, no host | `RATE_KB`, `WRITE_MS`, `QPEAK` versus `QCAP` |

## 10. Summary

* The framework is **not** what limits the rate; **the 3 Mbaud cable, the 8 MHz SPI
  cap and the CSV format** are, in that order.
* ESP-IDF would give better memory control (`heap_caps`, tracing, menuconfig), a
  real choice of SPI DMA vs polling, a non-blocking UART driver and no
  `BluetoothSerial` layer. It would **not** by itself enable 16 kSPS/ADS.
* 16 kSPS/ADS is the chip's maximum [datasheet]; 160 kSPS is not available on an
  ADS1299 at any framework.
* DMA does not exist for SPI or the card in this framework version [core], and
  where it would help it is blocked by the pin map (SDMMC D0/D1 collide with ADS
  PWDN and ADS2 CS).
* Wi-Fi is the only wireless link with the margin for 32 000 packets/s, and it costs
  **less RAM than Bluetooth** (~40-70 kB versus 152 kB) - the obstacle is RF coupling
  into the front end, not memory.
