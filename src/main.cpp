#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "ADS1299.h"

// ---------------------------------------------------------------------------
// Host link: one USB UART plus classic-Bluetooth SPP.
//
// The board always shows up on both. Whichever transport delivers the first
// byte becomes the active one, which is what the GUI's connect button does, so
// nothing has to be configured to switch between cable and radio.
// ---------------------------------------------------------------------------
#ifndef SZD_BT_ENABLED
#define SZD_BT_ENABLED 1
#endif
#ifndef SZD_BT_DEVICE_NAME
#define SZD_BT_DEVICE_NAME "SZD_EEG"
#endif
// BR/EDR TX power ceiling in dBm. The controller only has -12/-9/-6/-3/0/+3/+6/+9
// steps, so 4 selects +3 dBm (the strongest step that does not exceed it) and
// 6 selects +6 dBm. +9 dBm is the chip maximum and is not needed for a bench link.
#ifndef SZD_BT_TX_POWER_DBM
#define SZD_BT_TX_POWER_DBM 4
#endif
// Ring buffer that decouples the conversion interrupt from the Bluetooth stack.
// Answer to "how big should the buffer be": at 500 SPS a 33-byte packet every
// 2 ms is 16.5 kB/s, so 24 kB absorbs about 1.5 s of stall.
#ifndef SZD_BT_TX_BUFFER_BYTES
// The radio ring holds the outgoing stream while the SPP stack is busy. It absorbs
// bursts; it does not raise the radio's average throughput. 32 kB is about one
// second of the live link at 1 kSPS (33 B/packet), which is enough to ride out a
// host that stalls momentarily - the case that used to end in dropped packets.
#define SZD_BT_TX_BUFFER_BYTES 32768
#endif
#ifndef SZD_BT_TOUCH_PIN
#define SZD_BT_TOUCH_PIN 32
#endif
// A finger on the pad pulls the reading well below its resting value; the
// threshold is a fraction of the calibrated baseline so it self-adjusts.
#ifndef SZD_BT_TOUCH_TRIGGER_RATIO
#define SZD_BT_TOUCH_TRIGGER_RATIO 0.75f
#endif

static constexpr int PIN_TOUCH_BLUETOOTH = SZD_BT_TOUCH_PIN;
static constexpr uint32_t BT_HEALTH_PERIOD_MS = 500;
// Larger chunks mean fewer SPP write calls and fewer congestion events for the
// same bytes: the per-call cost is what the Arduino SPP path spends most of its
// time on. 1024 is still well inside a normal L2CAP MTU once fragmentation is
// applied, so it does not add latency to a text reply waiting behind it.
static constexpr uint32_t BT_FLUSH_CHUNK = 1024;
static constexpr size_t BT_MAX_BUFFER_BYTES = 65536;
static constexpr UBaseType_t BT_FLUSH_TASK_PRIO = 2;
// Core 0, with the radio stack it writes to (see ACQUISITION_CORE for why).
static constexpr int BT_FLUSH_TASK_CORE = 0;
static constexpr uint32_t LED_PERIOD_USB_MS = 500;
static constexpr uint32_t LED_PERIOD_BT_MS = 125;  // 4x faster: Bluetooth mode

static constexpr int PIN_ADS1_DRDY = 34;
static constexpr int PIN_ADS1_CS = 5;
static constexpr int PIN_ADS2_DRDY = 16;
static constexpr int PIN_ADS2_CS = 4;
static constexpr int PIN_ADS_START = 25;
static constexpr int PIN_ADS_RESET = 17;
static constexpr int PIN_ADS_PWDN = 2;
static constexpr int PIN_STATUS_LED = 27;
static constexpr int PIN_LIS3DH_CS = 26;
static constexpr int PIN_LIS3DH_SDA = 21;
static constexpr int PIN_LIS3DH_SCL = 22;

static constexpr int PIN_SD_CS = 15;
static constexpr int PIN_SD_MOSI = 13;
// MISO is on SENSOR_VN (GPIO39). It used to be GPIO12, which is MTDI - the pin
// that latches the flash supply voltage at every reset (high = 1.8 V flash,
// which bricks the interface). GPIO39 is input-only, which is exactly what MISO
// needs, and it is not a strapping pin, so this line can carry a pull-up.
static constexpr int PIN_SD_MISO = 39;
static constexpr int PIN_SD_SCLK = 14;

// Record/stop button, wired to the 'SWITCH' net on GPIO35.
//
// GPIO34..39 are input-only. They have no pull resistors *enabled at reset*, but
// they do contain internal pull-up/pull-down resistors that firmware can switch
// on, so pinMode(35, INPUT_PULLDOWN) is valid here and no external resistor is
// needed. (GPIO37, which this used to point at, is not even bonded out on a
// WROOM-32E.)
static constexpr int PIN_RECORD_BUTTON = 35;
// An edge closer than this to the previous one is contact bounce, not a press.
static constexpr uint32_t RECORD_BUTTON_DEBOUNCE_MS = 40;

static constexpr int PIN_SPI_SCLK = 18;
static constexpr int PIN_SPI_MISO = 19;
static constexpr int PIN_SPI_MOSI = 23;

// ---------------------------------------------------------------------------
// Build-time hardware selection (change ADS1299_CHANNELS_PER_CHIP to switch
// between your two boards):
//   ADS1299-4 -> 4 channels/chip,  8 total -> plain 8-channel Cyton mode
//   ADS1299-6 -> 6 channels/chip, 12 total -> 16-channel Cyton+Daisy emulation
// You can also override from PlatformIO:
//   build_flags = -DADS1299_CHANNELS_PER_CHIP=4
// ---------------------------------------------------------------------------
#ifndef ADS1299_CHANNELS_PER_CHIP
#define ADS1299_CHANNELS_PER_CHIP 6
#endif

static constexpr uint8_t ADS_CHANNELS_PER_CHIP = ADS1299_CHANNELS_PER_CHIP;  // 4 or 6
static constexpr uint8_t TOTAL_ADS_CHANNELS = ADS_CHANNELS_PER_CHIP * 2;     // 8 or 12 real channels
static constexpr uint8_t WIRE_CHANNELS = 8;                                 // a Cyton packet always carries 8 slots
static constexpr uint8_t GUI_CHANNELS = (TOTAL_ADS_CHANNELS == 8) ? 8 : 16; // channels the GUI sees
static constexpr bool DAISY_MODE = (TOTAL_ADS_CHANNELS > 8);                // interleaved 16ch emulation

static constexpr uint32_t SERIAL_BAUD = 2000000;
static constexpr uint32_t SERIAL_TX_BUFFER = 4096;
static constexpr bool USE_DEBUG_APP_MODE = false;
static constexpr uint32_t LIS3DH_I2C_CLOCK_HZ = 400000;
// SD card clock. The SD specification allows up to 25 MHz in SPI mode, and the
// Arduino SD library defaults to 4 MHz. A card write is dominated by the
// transaction time rather than the byte count, so the clock rate is the single
// most effective way to cut the time the writer task holds the bus - which is what
// competes with the Bluetooth flush task for the CPU. If the card refuses to
// initialise at this rate the bring-up retries at the conservative default.
static constexpr uint32_t SD_SPI_CLOCK_HZ = 10000000;
static constexpr uint32_t SD_SPI_CLOCK_FALLBACK_HZ = 4000000;
static uint32_t sdSpiClockActive = SD_SPI_CLOCK_HZ;
// 24-bit two's complement full scale of the ADS1299, used for the microvolt
// conversion of SD samples (the host uses the same value).
static constexpr int32_t ADS_FULL_SCALE_COUNTS = (1 << 23) - 1;
// ADS1299 SCLK. The datasheet allows up to 4 MHz for reads, but everything on
// the shared bus is byte-banged through the ESP32 SPI peripheral, so 2 MHz
// halves the time spent inside a data frame compared to the old 1 MHz setting.
// The host may raise this at runtime ('o' command) so a 2000/4000 SPS output
// data rate still fits inside one sample period.
static constexpr uint32_t ADS_SPI_CLOCK_HZ = 2000000;
static constexpr uint32_t ADS_SPI_CLOCK_MIN_HZ = 1000000;
static constexpr uint32_t ADS_SPI_CLOCK_MAX_HZ = 8000000;
static uint32_t adsSpiClockHz = ADS_SPI_CLOCK_HZ;

// Output decimation. The ADS1299 third-order sinc filter is -3 dB at
// 0.262 * f_MOD, so at 500 SPS the passband ends at ~131 Hz. Running the
// converter faster moves that corner up (2000 SPS -> 524 Hz, 4000 SPS ->
// 1048 Hz); averaging the extra samples back down keeps the host packet rate
// and recovers the noise that the shorter integration window costs.
static constexpr uint8_t DECIM_MIN = 1;
static constexpr uint8_t DECIM_MAX = 32;
static uint8_t decimFactor = 1;

static constexpr uint8_t PACKET_START_BYTE = 0xA0;
static constexpr uint8_t PACKET_END_STANDARD = 0xC0;

// Consecutive failed frame reads before a converter's channels are published as
// zeros. Small enough to notice a dead path in well under a second, large
// enough to ride out a single torn frame.
static constexpr uint8_t STALE_FRAME_LIMIT = 4;

static constexpr uint8_t LOFF_MAG_6NA = 0x00;
static constexpr uint8_t LOFF_FREQ_31P2HZ = 0x02;
static constexpr uint8_t TESTSIG_AMP_1X = 0x00;
static constexpr uint8_t TESTSIG_AMP_2X = 0x04;
static constexpr uint8_t TESTSIG_FREQ_SLOW = 0x00;
static constexpr uint8_t TESTSIG_FREQ_FAST = 0x01;
static constexpr uint8_t TESTSIG_FREQ_DC = 0x03;

static constexpr uint8_t BOARD_MODE_DEFAULT = 0;
static constexpr uint8_t BOARD_MODE_DEBUG = 1;
static constexpr uint8_t BOARD_MODE_ANALOG = 2;
static constexpr uint8_t BOARD_MODE_DIGITAL = 3;
static constexpr uint8_t BOARD_MODE_MARKER = 4;

static constexpr uint8_t LIS3DH_ADDR_LOW = 0x18;
static constexpr uint8_t LIS3DH_ADDR_HIGH = 0x19;
static constexpr uint8_t LIS3DH_REG_STATUS = 0x27;
static constexpr uint8_t LIS3DH_REG_OUT_X_L = 0x28;
static constexpr uint8_t LIS3DH_REG_WHO_AM_I = 0x0F;
static constexpr uint8_t LIS3DH_REG_CTRL1 = 0x20;
static constexpr uint8_t LIS3DH_REG_CTRL4 = 0x23;
static constexpr uint8_t LIS3DH_WHO_AM_I_VALUE = 0x33;

static constexpr uint8_t LIS3DH_CTRL1_100HZ_XYZ = 0x57;
static constexpr uint8_t LIS3DH_CTRL4_BDU_HR_4G = 0x98;

static constexpr uint8_t SD_MODE_COUNT = 8;
static constexpr char SD_FILENAME_PREFIX[] = "/OBCI";
static constexpr char SD_FILENAME_SUFFIX[] = ".TXT";

// SD recordings are OpenBCI-compatible CSV so the analyzer opens them with the
// same loader as a live recording. One line per published sample.
static constexpr char SD_RECORD_EXT[] = ".CSV";
static constexpr char SD_CONFIG_PATH[] = "/SZD_CFG.TXT";
static constexpr uint8_t SD_CONFIG_VERSION = 1;
static constexpr size_t SD_SESSION_NAME_MAX = 40;
// 12 channels x (sign + 7 digits + comma) + aux + marker + timestamp is about
// 155 bytes; 320 leaves room to spare and the writer refuses rather than
// overflows if a value is ever longer than expected.
static constexpr size_t SD_LINE_BUFFER = 320;
static constexpr uint32_t SD_ESTIMATE_SAMPLE_RATE_FALLBACK = 250;

// ---------------------------------------------------------------------------
// Transport state
// ---------------------------------------------------------------------------
enum HostLinkMode : uint8_t {
  HOST_LINK_BOTH = 0,  // nothing has spoken yet: listen on cable and radio
  HOST_LINK_USB = 1,
  HOST_LINK_BT = 2,
};

static HardwareSerial& usbHostPort = Serial;

#if SZD_BT_ENABLED
#include <BluetoothSerial.h>
#include <esp_bt.h>
static BluetoothSerial btSerial;
#else
// Compile-time stub with the same surface, so the transport code below needs no
// #if of its own: begin() reports failure and btStackReady stays false.
struct BluetoothStub {
  bool begin(const char*) { return false; }
  void setTimeout(unsigned long) {}
  bool hasClient() { return false; }
  bool disconnect() { return true; }
  void end() {}
  size_t write(const uint8_t*, size_t) { return 0; }
  int available() { return 0; }
  int read() { return -1; }
};
static BluetoothStub btSerial;
#endif

static volatile uint8_t hostLinkMode = HOST_LINK_BOTH;
static volatile bool btStackReady = false;
static volatile bool btModeActive = false;      // radio owns the stream
static volatile bool btForceEnabled = false;    // set by the touch pad
static volatile bool btStreamToRadio = false;   // mirror the stream over SPP

// Ring buffer between the 500 SPS conversion path and the Bluetooth stack.
static uint8_t* btRing = nullptr;
static size_t btRingSize = 0;
static volatile size_t btRingHead = 0;  // written by the streaming task
static volatile size_t btRingTail = 0;  // advanced by the flush task
static portMUX_TYPE btRingLock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool btDroppedBuffers = false;
// Peak fill of the ring since boot. This is the number that says whether the link
// has headroom or is riding the limit: a peak at 100 % means the radio could not
// keep up and packets were dropped, not that the buffer was too small.
static volatile size_t btRingPeak = 0;

static volatile uint32_t btPacketsQueued = 0;
static volatile uint32_t btPacketsDropped = 0;
static volatile uint32_t btBytesSent = 0;
static volatile bool btClientConnected = false;

// Measured throughput, refreshed once a second by the flush task. The point is to
// compare what the link *has* to carry against what it actually carries: if the
// measured rate sits at the required rate with the ring never filling, the radio is
// keeping up and no amount of buffer tuning is needed.
static volatile uint32_t btBytesPerSecond = 0;
static uint32_t btRateWindowBytes = 0;
static uint32_t btRateWindowStartMs = 0;

// The same for the card. sdMaxWriteMs is the longest single card transaction seen:
// that is the stall the Bluetooth flush task has to be able to preempt.
static volatile uint32_t sdBytesPerSecond = 0;
static volatile uint32_t sdMaxFlushMs = 0;
static uint32_t sdRateWindowBytes = 0;
static uint32_t sdRateWindowStartMs = 0;
static volatile size_t sdQueuePeak = 0;

// Link supervision. The Arduino SPP server keeps exactly one client handle and
// refuses every later connection while that handle is still set, so a single
// half-torn connection would make the board unreachable until it reboots. These
// track the events needed to notice that and recover from it.
static volatile bool btLinkOpen = false;
static volatile uint32_t btOpenEvents = 0;
static volatile uint32_t btCloseEvents = 0;
static volatile uint32_t btRejectedOpens = 0;
static volatile uint32_t btRemoteCloses = 0;
static volatile uint32_t btLastWriteOkMs = 0;
static volatile uint32_t btWriteFailures = 0;
static volatile bool btNeedsRecovery = false;
static uint8_t btRecoveryStage = 0;
static uint32_t btRecoveryDeadline = 0;

// Serialises every call into the SPP API, so the flush task cannot be inside
// btSerial.write() while the supervisor tears the stack down.
static SemaphoreHandle_t btApiMutex = nullptr;

// Text replies get their own small ring: the packet ring is frame-aligned so an
// overflow can evict exactly one 33-byte packet, and mixing variable-length
// lines into it would allow a torn frame.
// Diagnostics are queued here from the Bluetooth callback (which must never
// block, and must never allocate) and flushed to the host by the loop task.
static constexpr size_t BT_LOG_SLOTS = 16;
static constexpr size_t BT_LOG_LINE = 72;
static char btLog[BT_LOG_SLOTS][BT_LOG_LINE];
static volatile size_t btLogHead = 0;
static volatile size_t btLogTail = 0;

// Survives a reset but not a power cycle, so an increment after a Bluetooth
// disconnect proves the board restarted rather than the link merely dropping.
RTC_DATA_ATTR static uint32_t bootCount = 0;

static constexpr size_t BT_TEXT_RING_BYTES = 4096;
static uint8_t btTextRing[BT_TEXT_RING_BYTES];
static volatile size_t btTextHead = 0;
static volatile size_t btTextTail = 0;


static uint16_t touchBaseline = 0;
static uint16_t touchThreshold = 0;
static uint8_t touchStableCount = 0;
static bool touchLatched = false;
static uint16_t touchRaw = 0;

SPIClass sdSpi(HSPI);

ADS1299 ads1;
ADS1299 ads2;

struct DeviceCtx {
  ADS1299* ads;
  const char* name;
  int drdyPin;
  int csPin;
  uint8_t channelOffset;
  uint8_t channelCount;
  uint8_t deviceId;
  bool clockMaster;
  bool linkOk;
  volatile bool drdyPending;
  volatile uint32_t drdyCount;
  uint8_t failedReads;
};

static DeviceCtx devices[] = {
    {&ads1, "ADS1", PIN_ADS1_DRDY, PIN_ADS1_CS, 0, ADS_CHANNELS_PER_CHIP, 0, true, false, false, 0, 0},
    {&ads2, "ADS2", PIN_ADS2_DRDY, PIN_ADS2_CS, ADS_CHANNELS_PER_CHIP, ADS_CHANNELS_PER_CHIP, 0, false, false,
     false, 0, 0},
};
static constexpr size_t DEVICE_COUNT = sizeof(devices) / sizeof(devices[0]);

struct ChannelSetting {
  bool active;
  uint8_t gainOrdinal;
  uint8_t inputTypeOrdinal;
  bool biasInclude;
  bool srb2Connect;
  bool srb1Connect;
};

static ChannelSetting channelSettings[TOTAL_ADS_CHANNELS];
static bool leadOffP[TOTAL_ADS_CHANNELS];
static bool leadOffN[TOTAL_ADS_CHANNELS];

// Reference ("SRB") mode, one per converter.
//
// The two reference pins are NOT interchangeable, which is the single most
// confusing thing about this chip:
//
//   SRB1 (MISC1 bit 5) closes one switch for the entire chip that ties the
//   *inverting* input (INxN) of every channel to the SRB1 pin. This is the pin
//   the datasheet's referential montage uses: reference electrode to SRB1, all
//   active electrodes to INxP.
//
//   SRB2 (CHnSET bit 3) closes a per-channel switch between the SRB2 pin and
//   that channel's *non-inverting* input (INxP). It therefore drives INP; it
//   does not give the channel a negative reference. Closing it on a board where
//   SRB2 is not wired floats INP, and the channel then reads as saturation or
//   as pure float noise.
//
// Because SRB1 is chip-wide, the mode is tracked per converter rather than for
// the whole board: one ADS1299 can stay true differential (so a single input
// pair can measure the common-mode voltage directly) while the other converter
// runs SRB1 or SRB2.
static constexpr uint8_t REFERENCE_MODE_DIFFERENTIAL = 0;  // INP/INN from electrodes
static constexpr uint8_t REFERENCE_MODE_SRB1 = 1;          // all INN -> SRB1 pin
static constexpr uint8_t REFERENCE_MODE_SRB2 = 2;          // INP -> SRB2 pin, per channel
static uint8_t chipReferenceMode[DEVICE_COUNT] = {REFERENCE_MODE_DIFFERENTIAL,
                                                  REFERENCE_MODE_DIFFERENTIAL};

static const char* referenceModeName(uint8_t mode) {
  switch (mode) {
    case REFERENCE_MODE_DIFFERENTIAL: return "differential";
    case REFERENCE_MODE_SRB1: return "SRB1";
    case REFERENCE_MODE_SRB2: return "SRB2";
    default: return "unknown";
  }
}

// True when both converters are programmed the same way.
static bool referenceModesAgree() {
  return chipReferenceMode[0] == chipReferenceMode[1];
}

// "SRB2" when both chips agree, "ADS1:SRB2,ADS2:differential" when they do not.
static void formatReferenceSummary(char* buffer, size_t length) {
  if (referenceModesAgree()) {
    snprintf(buffer, length, "%s", referenceModeName(chipReferenceMode[0]));
    return;
  }
  snprintf(buffer, length, "ADS1:%s,ADS2:%s", referenceModeName(chipReferenceMode[0]),
           referenceModeName(chipReferenceMode[1]));
}

static volatile bool streaming = false;
static volatile uint8_t sampleCounter = 0;
static uint8_t sampleRateCode = 0x06;
static uint8_t boardMode = BOARD_MODE_DEFAULT;
static volatile bool daisyActive = DAISY_MODE;
static bool awaitingSampleRate = false;
static bool awaitingBoardMode = false;
static bool awaitingMarker = false;
static bool awaitingReferenceMode = false;
static bool awaitingChipReferenceChip = false;
static bool awaitingChipReferenceMode = false;
// ':' prefixed SD-card controls. A prefix is used because every letter and digit
// is already a Cyton channel selector or an existing command.
static bool awaitingSdCommand = false;
static bool parsingSdName = false;
static char sdNamePayload[SD_SESSION_NAME_MAX + 1];
static int sdNameLen = 0;
static char chipReferenceChip = 0;
static bool parsingDecimation = false;
static char decimationPayload[4];
static int decimationLen = 0;
static bool parsingSpiClock = false;
static char spiClockPayload[8];
static int spiClockLen = 0;
static bool parsingBufferSize = false;
static char bufferSizePayload[8];
static int bufferSizeLen = 0;
static char xPayload[24];
static uint8_t xLen = 0;
static bool parsingX = false;
static char zPayload[8];
static uint8_t zLen = 0;
static bool parsingZ = false;
static bool awaitingRadioCommand = false;
static uint8_t radioCommand = 0;
static bool awaitingRadioChannelValue = false;
static bool parsingDebugLine = false;
static char debugLine[128];
static uint8_t debugLineLen = 0;
static bool lis3dhPresent = false;
static uint8_t lis3dhAddress = LIS3DH_ADDR_LOW;
static uint8_t lis3dhWhoAmI = 0x00;
static volatile int16_t gAuxData[3] = {0, 0, 0};
static SemaphoreHandle_t auxMutex = nullptr;
// Given from the DRDY ISR, taken by the acquisition task. Using a real
// blocking wait (instead of polling a flag) lets the idle task run, which keeps
// the task watchdog quiet and removes the multi-second jitter.
static SemaphoreHandle_t conversionSem = nullptr;
static volatile uint32_t missedConversions = 0;
// Worst time spent clocking both converters in one conversion. Comparing it with
// the sample period says whether the acquisition task is late or the transport
// is losing what it produced.
static volatile uint32_t spiReadMaxUs = 0;
static volatile uint32_t spiReadLastUs = 0;
// Conversions that actually became packets, so DRDY edges and published packets
// can be compared: edges above packets means conversions are being missed.
static volatile uint32_t publishedConversions = 0;
// Late conversions are the number that matters for this design: one conversion
// arriving more than 1.5 periods after the previous one means the acquisition
// task was delayed by somebody else. It is measured directly with the microsecond
// clock rather than inferred from counters.
static volatile uint32_t lateConversions = 0;
static volatile uint32_t worstGapUs = 0;
// CRC work for the radio, computed where the packets are sent instead of in the
// acquisition loop. A running CRC over the byte stream is enough for the host to
// verify integrity and costs nothing per conversion.
static volatile uint16_t btStreamCrc = 0xFFFF;
static volatile uint32_t droppedPackets = 0;
static volatile uint32_t corruptFrames = 0;
// When the last byte arrived from a host (USB or Bluetooth). Any command inside
// HOST_PRESENCE_WINDOW_MS of it means a host is actively talking to us, which is
// what the automatic stream restart keys off.
static volatile uint32_t lastHostByteMs = 0;
static constexpr uint32_t HOST_PRESENCE_WINDOW_MS = 50;
// Set by the explicit stop / reset commands so the automatic restart cannot fight
// a deliberate stop.
static bool suppressAutoStream = false;

// Set by the record-button interrupt, consumed by the main loop, which owns the
// ADC bus.
static volatile bool recordButtonPressed = false;
static volatile uint32_t recordButtonLastEdgeMs = 0;

// Bumped for every recording, so queued samples from a previous file are dropped.
static volatile uint32_t sdGeneration = 0;

static bool sdPresent = false;
static volatile bool sdRecording = false;
static File sdFile;
static char sdFileName[72] = {0};
static uint32_t sdRecordStartMs = 0;
static uint32_t sdRecordDurationMs = 0;
static SemaphoreHandle_t sdMutex = nullptr;
static QueueHandle_t sdQueue = nullptr;

// Session name used for the SD file and its header. Written into the config file
// so a button-started recording carries the same name the GUI would have used.
static char sdSessionName[SD_SESSION_NAME_MAX + 1] = "sd-session";
// Channel names/indices that were enabled when the recording opened, so the
// writer can skip powered-down inputs instead of spending space on them.
static uint8_t sdRecordedChannels[TOTAL_ADS_CHANNELS] = {0};
static uint8_t sdRecordedCount = 0;
// Counters that make a gap visible after the fact.
static volatile uint32_t sdSamplesWritten = 0;
static volatile uint32_t sdSamplesDropped = 0;
static volatile uint32_t sdBytesWritten = 0;
static bool sdHeaderWritten = false;
// Set when the host sends a marker (`` ` ``). The next SD row carries Marker=1,
// which is what lets the analyzer line the card recording up with the live file:
// both files then contain the same instant, whatever the link latency was.
static volatile bool sdMarkerPending = false;

// ---------------------------------------------------------------------------
// SD write buffering.
//
// A card write is not a memcpy: the FAT layer may erase a sector, and a single
// 100-byte line can cost 10-50 ms. Doing one card transaction per sample at
// 1000 SPS is impossible, and doing it at a priority above the Bluetooth flush
// task starved the radio until its ring was permanently 100 % - which is exactly
// the "recording to the card froze the Bluetooth link" failure.
//
// So samples are formatted into RAM and handed to the card in large batches:
//   * a queue absorbs conversion bursts (a card that pauses to erase a sector),
//   * this buffer turns thousands of small writes into a few large ones.
// The writer task runs *below* the radio flush task, so the card can never take
// the CPU the radio needs; it only uses what is left over.
// ---------------------------------------------------------------------------
static constexpr int SD_QUEUE_LENGTH = 512;
static constexpr size_t SD_WRITE_BUFFER_BYTES = 4096;
static uint8_t sdWriteBuffer[SD_WRITE_BUFFER_BYTES];
static size_t sdWriteBufferLength = 0;
// How many samples are currently represented in the byte buffer, so a failed card
// write reports the truth instead of "one drop".
static uint32_t sdWriteBufferSamples = 0;
static uint32_t sdBufferFlushes = 0;

// ---------------------------------------------------------------------------
// Core assignment.
//
// The ESP32 has two cores, and the radio is not a polite neighbour: the
// Bluetooth controller's interrupts and the Bluedroid BTU/BTC tasks are pinned
// to core 0 (and would sit at the same priority as a priority-20 acquisition
// task). Anything that shares core 0 with them gets delayed by whole
// milliseconds at a time, which at 500 SPS is a missed conversion.
//
// So the acquisition task gets core 1 as close to exclusively as possible:
//   core 1 - ADS1299 acquisition only (plus the Arduino loop task at priority 1)
//   core 0 - Bluetooth flush, SD writer, accelerometer, radio stack
//
// The DRDY interrupt is attached from setup(), which runs on core 1, so the
// semaphore hand-off also stays on one core instead of going through the
// cross-core IPC.
// ---------------------------------------------------------------------------
static constexpr int ACQUISITION_CORE = 1;
static constexpr int RADIO_CORE = 0;
static constexpr UBaseType_t STREAMING_TASK_PRIO = 22;
// Deliberately below BT_FLUSH_TASK_PRIO. The card writer is the only task that
// can block for tens of milliseconds at a time, so it must be the one that waits:
// the radio link always wins the CPU, and the SD queue absorbs the difference.
static constexpr UBaseType_t SD_TASK_PRIO = 1;
static constexpr UBaseType_t AUX_TASK_PRIO = 3;

static void ledBlinkTask(void* parameter) {
  (void)parameter;
  bool ledOn = false;

  for (;;) {
    // While the card is recording the LED is a recording indicator first and a
    // transport indicator second, because losing a recording is worse than not
    // knowing which link is up:
    //
    //   USB link       -> solid on
    //   Bluetooth link -> 950 ms on / 50 ms off (a deliberate "long on" blink)
    //   no host link   -> 900 ms on / 50 off / 50 on / 50 off (two quick off
    //                     flashes: recording, but nothing is listening)
    if (sdRecording) {
      const uint8_t phase = btModeActive ? 1 : 0;
      if (hostLinkMode == HOST_LINK_USB) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      if (phase == 1) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(950));
        digitalWrite(PIN_STATUS_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
      } else {
        digitalWrite(PIN_STATUS_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(900));
        digitalWrite(PIN_STATUS_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_STATUS_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_STATUS_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      continue;
    }

    ledOn = !ledOn;
    digitalWrite(PIN_STATUS_LED, ledOn ? HIGH : LOW);
    // Four times faster while the radio carries the stream, so the board says
    // which transport it is using without needing the GUI.
    vTaskDelay(pdMS_TO_TICKS(btModeActive ? LED_PERIOD_BT_MS : LED_PERIOD_USB_MS));
  }
}

struct SdModeSetting {
  char command;
  uint32_t durationMs;
  const char* label;
};

static const SdModeSetting SD_MODE_SETTINGS[SD_MODE_COUNT] = {
    {'A', 5UL * 60UL * 1000UL, "5 minute"},
    {'S', 15UL * 60UL * 1000UL, "15 minute"},
    {'F', 30UL * 60UL * 1000UL, "30 minute"},
    {'G', 60UL * 60UL * 1000UL, "1 hour"},
    {'H', 2UL * 60UL * 60UL * 1000UL, "2 hour"},
    {'J', 4UL * 60UL * 60UL * 1000UL, "4 hour"},
    {'K', 12UL * 60UL * 60UL * 1000UL, "12 hour"},
    {'L', 24UL * 60UL * 60UL * 1000UL, "24 hour"},
};

static void emitDebugHello();
static void emitDebugState();
static void handleDebugLine(const char* line);
static void emitSuccess(const char* msg);
static void emitFailure(const char* msg);
static uint16_t sampleRateFromCode(uint8_t code);
static bool stopStreaming();
static void startStreaming();
static void programAllDevices();
static void emitSdStatus(const char* event = nullptr);
static void saveSdConfiguration();
static bool loadSdConfiguration(bool applyToHardware);
static void startSdRecordingFromBoard(const char* reason);
static void stopSdRecordingFromBoard(const char* reason);
static void toggleSdRecordingFromButton();
static void ensureStreamingForHost();
static void streamingTask(void* parameter);
static void sdWriterTask(void* parameter);
static void lis3dhTask(void* parameter);
static void bringUpBluetooth();
static void IRAM_ATTR adsDrdyIsr0();
static void IRAM_ATTR adsDrdyIsr1();

// ---------------------------------------------------------------------------
// HostLink: the object the rest of the firmware talks to as ``Serial``.
//
// ``Print`` supplies print/println/printf, so every existing call site keeps
// working unchanged; only the transport underneath changes.
// ---------------------------------------------------------------------------
static uint16_t crc16Update(uint16_t crc, const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static size_t btRingFreeLocked() {
  const size_t head = btRingHead;
  const size_t tail = btRingTail;
  if (head >= tail) {
    return btRingSize - (head - tail) - 1;
  }
  return tail - head - 1;
}

static bool btRingPush(const uint8_t* data, size_t length, bool dropOldest) {
  if (btRing == nullptr || !btStreamToRadio || length == 0 || length >= btRingSize) {
    return false;
  }
  bool dropped = false;
  portENTER_CRITICAL(&btRingLock);
  if (dropOldest && length % 33 == 0) {
    // Keep whole packets: a truncated 33-byte frame is far worse than a gap the
    // host can see in the packet counter.
    while (btRingFreeLocked() < length) {
      btRingTail = (btRingTail + 33) % btRingSize;
      btDroppedBuffers = true;
      dropped = true;
    }
  }
  if (btRingFreeLocked() < length) {
    portEXIT_CRITICAL(&btRingLock);
    return false;
  }
  const size_t first = min(length, btRingSize - btRingHead);
  memcpy(btRing + btRingHead, data, first);
  if (first < length) {
    memcpy(btRing, data + first, length - first);
  }
  btRingHead = (btRingHead + length) % btRingSize;
  // Peak fill, measured while the ring is locked so it is exact.
  const size_t filled = (btRingHead >= btRingTail) ? (btRingHead - btRingTail)
                                                   : (btRingSize - btRingTail + btRingHead);
  if (filled > btRingPeak) {
    btRingPeak = filled;
  }
  portEXIT_CRITICAL(&btRingLock);
  if (dropped) {
    btPacketsDropped++;
  }
  return true;
}

static bool btHasClient() {
  return btStackReady && btSerial.hasClient();
}

// Safe from any task and from the BT callback: no allocation, no printing.
static void btLogEvent(const char* format, ...) {
  char line[BT_LOG_LINE];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  const size_t head = btLogHead;
  strncpy(btLog[head], line, BT_LOG_LINE - 1);
  btLog[head][BT_LOG_LINE - 1] = '\0';
  btLogHead = (head + 1) % BT_LOG_SLOTS;
}

static size_t btLogPendingCount() {
  const size_t head = btLogHead;
  const size_t tail = btLogTail;
  return (head >= tail) ? (head - tail) : (BT_LOG_SLOTS - tail + head);
}

static bool btLogPop(char* out, size_t length) {
  if (btLogPendingCount() == 0) {
    return false;
  }
  const size_t tail = btLogTail;
  strncpy(out, btLog[tail], length - 1);
  out[length - 1] = '\0';
  btLogTail = (tail + 1) % BT_LOG_SLOTS;
  return true;
}

static size_t btRingUsedBytes() {
  portENTER_CRITICAL(&btRingLock);
  const size_t used = (btRingHead >= btRingTail) ? (btRingHead - btRingTail)
                                                 : (btRingSize - btRingTail + btRingHead);
  portEXIT_CRITICAL(&btRingLock);
  return used;
}

static size_t btTextRingUsed() {
  portENTER_CRITICAL(&btRingLock);
  const size_t used = (btTextHead >= btTextTail) ? (btTextHead - btTextTail)
                                                 : (BT_TEXT_RING_BYTES - btTextTail + btTextHead);
  portEXIT_CRITICAL(&btRingLock);
  return used;
}

// Queue a text line for the flush task. Whole lines only: if the ring cannot
// take the line it is dropped, which is harmless for telemetry and keeps the
// flush task's output free of half-written replies.
static void btQueueText(const char* text, size_t length) {
  if (text == nullptr || length == 0 || length >= BT_TEXT_RING_BYTES) {
    return;
  }
  portENTER_CRITICAL(&btRingLock);
  const size_t used = (btTextHead >= btTextTail) ? (btTextHead - btTextTail)
                                                 : (BT_TEXT_RING_BYTES - btTextTail + btTextHead);
  bool queued = false;
  if (BT_TEXT_RING_BYTES - used - 1 >= length) {
    const size_t first = min(length, BT_TEXT_RING_BYTES - btTextHead);
    memcpy(btTextRing + btTextHead, text, first);
    if (first < length) {
      memcpy(btTextRing, text + first, length - first);
    }
    btTextHead = (btTextHead + length) % BT_TEXT_RING_BYTES;
    queued = true;
  }
  portEXIT_CRITICAL(&btRingLock);
  if (!queued) {
    btPacketsDropped++;
  }
}

// Copies without consuming: the flush task advances the tail only after the
// stack accepted the bytes, so a failed write loses nothing and cannot reorder
// the reply stream.
static size_t btTextRingPeek(char* out, size_t maxLength) {
  size_t taken = 0;
  portENTER_CRITICAL(&btRingLock);
  const size_t used = (btTextHead >= btTextTail) ? (btTextHead - btTextTail)
                                                 : (BT_TEXT_RING_BYTES - btTextTail + btTextHead);
  taken = min(used, maxLength);
  if (taken > 0) {
    const size_t first = min(taken, BT_TEXT_RING_BYTES - btTextTail);
    memcpy(out, btTextRing + btTextTail, first);
    if (first < taken) {
      memcpy(out + first, btTextRing, taken - first);
    }
  }
  portEXIT_CRITICAL(&btRingLock);
  return taken;
}

static void btTextRingAdvance(size_t count) {
  portENTER_CRITICAL(&btRingLock);
  btTextTail = (btTextTail + count) % BT_TEXT_RING_BYTES;
  portEXIT_CRITICAL(&btRingLock);
}

// Mirrors the stream into the radio ring. Never blocks: if the ring is full the
// oldest packet is dropped and counted, because stalling here would cost a
// conversion on the ADS1299 side.
static void btQueuePacket(const uint8_t* packet, size_t length) {
  if (!btStackReady || !btStreamToRadio || !btHasClient()) {
    return;
  }
  if (btRingPush(packet, length, true)) {
    // No CRC here on purpose: this runs in the acquisition task, and the flush
    // task folds the bytes into the stream CRC as it sends them.
    btPacketsQueued++;
  } else {
    btPacketsDropped++;
  }
}

// Only the flush task ever touches btSerial, so a reply queued from the loop
// task, from setup() or from the acquisition task is written by exactly one
// writer. Calling the SPP API from several tasks at once is what makes the
// Arduino library's congestion accounting unreliable.
static void btSendText(const char* text) {
  if (!btStackReady || !btStreamToRadio || !btHasClient()) {
    return;
  }
  btQueueText(text, strlen(text));
}

class HostLink : public Stream {
 public:
  void begin(uint32_t baud) {
    usbHostPort.begin(baud);
  }

  void setTxBufferSize(size_t size) {
    usbHostPort.setTxBufferSize(size);
  }

  size_t write(uint8_t value) override { return write(&value, 1); }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (buffer == nullptr || size == 0) {
      return 0;
    }
    if (btStreamToRadio) {
      if (size == 33 && buffer[0] == PACKET_START_BYTE) {
        btQueuePacket(buffer, size);
      } else {
        btQueueText(reinterpret_cast<const char*>(buffer), size);
      }
    }
    // The USB port is mirrored unconditionally, without ever blocking: if the
    // UART buffer cannot take the whole frame the copy is skipped and the packet
    // counter shows the gap. Replying to both transports is deliberate - the
    // cable being silent must never be the reason a reply disappears.
    if (usbHostPort.availableForWrite() >= static_cast<int>(size)) {
      usbHostPort.write(buffer, size);
    }
    return size;
  }

  int availableForWrite() override {
    // Report headroom whenever any transport can take the data, so the
    // acquisition task never drops a packet that the radio could have carried.
    if (btStreamToRadio && btHasClient()) {
      return max(usbHostPort.availableForWrite(), static_cast<int>(BT_FLUSH_CHUNK));
    }
    return usbHostPort.availableForWrite();
  }

  int available() override {
    int total = peeked >= 0 ? 1 : 0;
    total += usbHostPort.available();
    if (btStackReady) {
      total += btSerial.available();
    }
    return total;
  }

  int peek() override {
    if (peeked < 0) {
      peeked = read();
    }
    return peeked;
  }

  int read() override {
    if (peeked >= 0) {
      const int value = peeked;
      peeked = -1;
      return value;
    }
    // A byte from the radio is what switches the board to Bluetooth, and a byte
    // from the cable switches it back, so plugging in never needs a command.
    if (btStackReady && btSerial.available() > 0) {
      const int value = btSerial.read();
      if (value >= 0) {
        lastHostByteMs = millis();
        if (hostLinkMode != HOST_LINK_BT) {
          setMode(HOST_LINK_BT);
        }
      }
      return value;
    }
    if (usbHostPort.available() > 0) {
      const int value = usbHostPort.read();
      if (value >= 0) {
        lastHostByteMs = millis();
        if (hostLinkMode != HOST_LINK_USB) {
          setMode(HOST_LINK_USB);
        }
      }
      return value;
    }
    return -1;
  }

  void flush() {
    usbHostPort.flush();
  }

 private:
  int peeked = -1;

 public:

  static void setMode(uint8_t mode) {
    if (hostLinkMode == mode) {
      return;
    }
    hostLinkMode = mode;
    btModeActive = (mode == HOST_LINK_BT);
  }
};

static HostLink hostLink;

// Everything below this line talks to ``hostLink``. The alias keeps the ~120
// existing Serial.print/println/printf sites untouched.
#define Serial hostLink

static bool lis3dhWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lis3dhAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool lis3dhReadRegister(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(lis3dhAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(static_cast<int>(lis3dhAddress), 1);
  if (received != 1 || !Wire.available()) {
    return false;
  }

  value = Wire.read();
  return true;
}

static bool lis3dhReadRegisters(uint8_t startReg, uint8_t* buffer, size_t length) {
  if (buffer == nullptr || length == 0) {
    return false;
  }

  Wire.beginTransmission(lis3dhAddress);
  Wire.write(static_cast<uint8_t>(startReg | 0x80));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t requested = Wire.requestFrom(static_cast<int>(lis3dhAddress), static_cast<int>(length));
  if (requested != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = Wire.read();
  }
  return true;
}

static bool bringUpLis3dh() {
  pinMode(PIN_LIS3DH_CS, OUTPUT);
  digitalWrite(PIN_LIS3DH_CS, HIGH);

  Wire.begin(PIN_LIS3DH_SDA, PIN_LIS3DH_SCL, LIS3DH_I2C_CLOCK_HZ);

  for (uint8_t address : {LIS3DH_ADDR_LOW, LIS3DH_ADDR_HIGH}) {
    lis3dhAddress = address;
    uint8_t whoAmI = 0;
    if (lis3dhReadRegister(LIS3DH_REG_WHO_AM_I, whoAmI) && whoAmI == LIS3DH_WHO_AM_I_VALUE) {
      lis3dhWhoAmI = whoAmI;
      Serial.printf("[LIS3DH] found at 0x%02X, WHO_AM_I=0x%02X\n", address, whoAmI);

      const bool c1Wrote = lis3dhWriteRegister(LIS3DH_REG_CTRL1, LIS3DH_CTRL1_100HZ_XYZ);
      const bool c4Wrote = lis3dhWriteRegister(LIS3DH_REG_CTRL4, LIS3DH_CTRL4_BDU_HR_4G);

      uint8_t c1 = 0, c4 = 0;
      const bool c1Read = lis3dhReadRegister(LIS3DH_REG_CTRL1, c1);
      const bool c4Read = lis3dhReadRegister(LIS3DH_REG_CTRL4, c4);
      Serial.printf("[LIS3DH] CTRL1 write=%d readback=0x%02X (want 0x%02X)\n",
                    c1Wrote ? 1 : 0, c1, LIS3DH_CTRL1_100HZ_XYZ);
      Serial.printf("[LIS3DH] CTRL4 write=%d readback=0x%02X (want 0x%02X)\n",
                    c4Wrote ? 1 : 0, c4, LIS3DH_CTRL4_BDU_HR_4G);

      lis3dhPresent = c1Wrote && c4Wrote && c1Read && c4Read &&
                      c1 == LIS3DH_CTRL1_100HZ_XYZ &&
                      c4 == LIS3DH_CTRL4_BDU_HR_4G;
      return lis3dhPresent;
    }

    Serial.printf("[LIS3DH] no device at 0x%02X (WHO_AM_I=0x%02X)\n", address, whoAmI);
  }

  lis3dhPresent = false;
  lis3dhWhoAmI = 0x00;
  Serial.println("[LIS3DH] NOT FOUND - check I2C pull-ups, SA0 wiring and that CS is held HIGH");
  return false;
}

static bool readLis3dhAuxData(int16_t out[3]) {
  if (!lis3dhPresent) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return false;
  }

  uint8_t raw[6] = {0};
  if (!lis3dhReadRegisters(LIS3DH_REG_OUT_X_L, raw, sizeof(raw))) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return false;
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    const int16_t value = static_cast<int16_t>(static_cast<uint16_t>(raw[axis * 2]) | (static_cast<uint16_t>(raw[axis * 2 + 1]) << 8));
    out[axis] = value;
  }
  return true;
}

static bool bringUpSdCard() {
  // CS is driven high before the bus is configured, so the card sees a defined
  // deselect from the first clock edge. The ESP32's GPIO15 also has its internal
  // pull-up active at reset, which keeps CS high through the whole boot.
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  sdSpi.begin(PIN_SD_SCLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // The SD spec asks for a pull-up on DAT0 (MISO) so the line has a defined level
  // while the card tri-states it. GPIO39 has an internal one, so this costs no
  // external part; it changes only the pull, never the direction, so the SPI
  // input path is untouched.
  gpio_set_pull_mode(static_cast<gpio_num_t>(PIN_SD_MISO), GPIO_PULLUP_ONLY);

  sdSpiClockActive = SD_SPI_CLOCK_HZ;
  sdPresent = SD.begin(PIN_SD_CS, sdSpi, SD_SPI_CLOCK_HZ);
  if (!sdPresent) {
    // A slow or unbuffered card can refuse the first init at 10 MHz. Falling back
    // is much better than reporting "no card": the recording still works, just at
    // the library default.
    sdPresent = SD.begin(PIN_SD_CS, sdSpi, SD_SPI_CLOCK_FALLBACK_HZ);
    if (sdPresent) {
      sdSpiClockActive = SD_SPI_CLOCK_FALLBACK_HZ;
      usbHostPort.printf("[SD] card did not initialise at %lu kHz, using %lu kHz\n",
                         static_cast<unsigned long>(SD_SPI_CLOCK_HZ / 1000),
                         static_cast<unsigned long>(SD_SPI_CLOCK_FALLBACK_HZ / 1000));
    }
  }
  if (sdPresent) {
    usbHostPort.printf("[SD] card ready at %lu kHz SPI\n",
                       static_cast<unsigned long>(sdSpiClockActive / 1000));
  }
  return sdPresent;
}

static void closeSdFile();

static bool makeNextSdFileName(char* buffer, size_t length) {
  if (buffer == nullptr || length < sizeof("/OBCI_00.TXT")) {
    return false;
  }

  for (uint16_t index = 0; index < 100; ++index) {
    snprintf(buffer, length, "%s_%02u%s", SD_FILENAME_PREFIX, static_cast<unsigned>(index), SD_FILENAME_SUFFIX);
    if (!SD.exists(buffer)) {
      return true;
    }
  }

  return false;
}

static void writeHex24(File& file, int32_t value) {
  char buffer[7];
  snprintf(buffer, sizeof(buffer), "%06lX", static_cast<unsigned long>(static_cast<uint32_t>(value) & 0x00FFFFFFUL));
  file.print(buffer);
}

static void writeHex16(File& file, int16_t value) {
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%04X", static_cast<unsigned>(static_cast<uint16_t>(value)));
  file.print(buffer);
}

struct SdSample {
  uint8_t sampleIndex;
  int32_t channels[TOTAL_ADS_CHANNELS];
  int16_t aux[3];
  uint32_t micros;  // sample time, for the Timestamp column
  // Which recording this sample belongs to. A sample queued for one file must
  // never be written into the next one, and the writer can be mid-sample when a
  // recording is closed and a new one opened.
  uint32_t generation;
};

// Integer microvolt conversion: counts * VREF / (full-scale * gain). Done in
// 64-bit so a near-full-scale sample at gain x1 cannot overflow.
static int32_t countsToMicrovolts(int32_t counts, uint8_t gainOrdinal) {
  static const int32_t gainScalarLut[7] = {1, 2, 4, 6, 8, 12, 24};
  const int32_t gain = gainScalarLut[gainOrdinal > 6 ? 6 : gainOrdinal];
  const int64_t numerator = static_cast<int64_t>(counts) * 4500000LL;
  const int64_t denominator = static_cast<int64_t>(ADS_FULL_SCALE_COUNTS) * gain;
  return static_cast<int32_t>(numerator / denominator);
}

static char* appendUInt32(char* cursor, uint32_t value) {
  char digits[10];
  uint8_t count = 0;
  do {
    digits[count++] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && count < sizeof(digits));
  while (count > 0) {
    *cursor++ = digits[--count];
  }
  return cursor;
}

static char* appendInt32(char* cursor, int32_t value) {
  if (value < 0) {
    *cursor++ = '-';
    // Negate in 64-bit: -(-2147483648) is not representable in int32_t.
    return appendUInt32(cursor, static_cast<uint32_t>(-static_cast<int64_t>(value)));
  }
  return appendUInt32(cursor, static_cast<uint32_t>(value));
}

static char* appendFixed6(char* cursor, uint32_t value) {
  char digits[6];
  for (int i = 5; i >= 0; --i) {
    digits[i] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  }
  memcpy(cursor, digits, sizeof(digits));
  return cursor + sizeof(digits);
}

// Writes whatever is in the RAM buffer to the card. Caller must hold sdMutex.
// Returns true when the buffer was actually handed over.
static bool sdFlushWriteBufferLocked() {
  if (sdWriteBufferLength == 0) {
    return false;
  }
  if (!sdFile) {
    sdWriteBufferLength = 0;
    sdWriteBufferSamples = 0;
    return false;
  }
  // Time the transaction: this is the window in which the writer task holds the
  // bus and the CPU, i.e. exactly what the radio flush task has to compete with.
  const uint32_t startedMs = millis();
  const size_t written = sdFile.write(sdWriteBuffer, sdWriteBufferLength);
  const uint32_t elapsedMs = millis() - startedMs;
  if (elapsedMs > sdMaxFlushMs) {
    sdMaxFlushMs = elapsedMs;
  }
  if (written != sdWriteBufferLength) {
    // A short write means the card is gone or full. Report every sample that was
    // in the buffer, not just one: the whole batch is lost.
    sdSamplesDropped += sdWriteBufferSamples;
  }
  sdRateWindowBytes += static_cast<uint32_t>(written);
  const uint32_t nowMs = millis();
  if (sdRateWindowStartMs == 0) {
    sdRateWindowStartMs = nowMs;
  } else if (nowMs - sdRateWindowStartMs >= 1000) {
    sdBytesPerSecond = (sdRateWindowBytes * 1000U) / (nowMs - sdRateWindowStartMs);
    sdRateWindowBytes = 0;
    sdRateWindowStartMs = nowMs;
  }
  sdWriteBufferLength = 0;
  sdWriteBufferSamples = 0;
  sdBufferFlushes++;
  return true;
}

// Card write entry point used by the writer task when the queue runs dry, so the
// tail of a recording reaches the card without waiting for 4 kB of data.
static void sdFlushWriteBuffer() {
  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }
  if (sdRecording && sdFile) {
    sdFlushWriteBufferLocked();
    sdFile.flush();
  }
  xSemaphoreGive(sdMutex);
}

static void writeSampleToSd(const SdSample& sample) {
  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  if (!sdRecording || !sdFile || sdRecordedCount == 0) {
    xSemaphoreGive(sdMutex);
    return;
  }
  if (sample.generation != sdGeneration) {
    // Queued for a recording that has already been closed.
    xSemaphoreGive(sdMutex);
    return;
  }

  char line[SD_LINE_BUFFER];
  char* cursor = line;
  // Hard bound: a pathological value must cost a dropped sample, never a smashed
  // stack. Worst case is about 200 bytes, so this only fires on a real surprise.
  const auto fits = [&](size_t needed) {
    return static_cast<size_t>(cursor - line) + needed < SD_LINE_BUFFER;
  };

  // Sample index, then one column per *enabled* channel. Channels switched off in
  // the settings are not in the file at all, which is the whole point of the mask:
  // the space and the column count both follow what is actually measured.
  if (!fits(12)) {
    sdSamplesDropped++;
    xSemaphoreGive(sdMutex);
    return;
  }
  cursor = appendUInt32(cursor, static_cast<uint32_t>(sample.sampleIndex));
  for (uint8_t i = 0; i < sdRecordedCount; ++i) {
    if (!fits(13)) {
      sdSamplesDropped++;
      xSemaphoreGive(sdMutex);
      return;
    }
    const uint8_t channel = sdRecordedChannels[i];
    *cursor++ = ',';
    const uint8_t gainOrdinal =
        channel < TOTAL_ADS_CHANNELS ? channelSettings[channel].gainOrdinal : 6;
    cursor = appendInt32(cursor, countsToMicrovolts(sample.channels[channel], gainOrdinal));
  }

  if (!fits(32)) {
    sdSamplesDropped++;
    xSemaphoreGive(sdMutex);
    return;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    *cursor++ = ',';
    cursor = appendInt32(cursor, sample.aux[axis]);
  }

  *cursor++ = ',';
  *cursor++ = sdMarkerPending ? '1' : '0';
  // One row only: a marker marks an instant, not a region.
  sdMarkerPending = false;

  *cursor++ = ',';
  cursor = appendUInt32(cursor, sample.micros / 1000000UL);
  *cursor++ = '.';
  cursor = appendFixed6(cursor, sample.micros % 1000000UL);

  *cursor++ = '\r';
  *cursor++ = '\n';

  const size_t length = static_cast<size_t>(cursor - line);
  // Append to the RAM buffer and only touch the card when it is full. This is the
  // difference between one card transaction per sample and one per ~40 samples.
  if (sdWriteBufferLength + length > SD_WRITE_BUFFER_BYTES) {
    sdFlushWriteBufferLocked();
  }
  if (sdWriteBufferLength + length > SD_WRITE_BUFFER_BYTES) {
    // A single line larger than the whole buffer cannot happen (the bound above
    // caps it at SD_LINE_BUFFER), but dropping beats writing past the end.
    sdSamplesDropped++;
    xSemaphoreGive(sdMutex);
    return;
  }
  memcpy(sdWriteBuffer + sdWriteBufferLength, line, length);
  sdWriteBufferLength += length;
  sdWriteBufferSamples++;
  sdSamplesWritten++;
  sdBytesWritten += static_cast<uint32_t>(length);
  xSemaphoreGive(sdMutex);
}

static void queueSampleForSd(uint8_t packetIndex, const int32_t channels[TOTAL_ADS_CHANNELS], const int16_t aux[3]) {
  if (sdQueue == nullptr || !sdRecording) {
    return;
  }

  SdSample sample;
  sample.sampleIndex = packetIndex;
  memcpy(sample.channels, channels, sizeof(sample.channels));
  memcpy(sample.aux, aux, sizeof(sample.aux));
  sample.micros = micros();
  sample.generation = sdGeneration;

  // Non-blocking: if the SD writer can't keep up, drop the sample. The
  // streaming path must never stall behind the SD card.
  if (xQueueSend(sdQueue, &sample, 0) != pdTRUE) {
    sdSamplesDropped++;
  } else {
    const UBaseType_t waiting = uxQueueMessagesWaiting(sdQueue);
    if (waiting > sdQueuePeak) {
      sdQueuePeak = waiting;
    }
  }
}

// Builds "/SZD_0007_my_name.CSV" from the session name. Everything outside
// [A-Za-z0-9._-] becomes '-', so any name the GUI sends is a legal FAT name and
// the analyzer can still recover the original from the header.
static void sanitiseName(const char* source, char* target, size_t length) {
  size_t out = 0;
  bool lastDash = false;
  for (size_t i = 0; source != nullptr && source[i] != '\0' && out + 1 < length; ++i) {
    const char c = source[i];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '.' || c == '_';
    const char mapped = ok ? c : '-';
    if (mapped == '-') {
      if (lastDash || out == 0) {
        continue;
      }
      lastDash = true;
    } else {
      lastDash = false;
    }
    target[out++] = mapped;
  }
  if (out == 0) {
    const char* fallback = "session";
    for (size_t i = 0; fallback[i] != '\0' && out + 1 < length; ++i) {
      target[out++] = fallback[i];
    }
  }
  target[out] = '\0';
}

static bool makeSdRecordingName(char* buffer, size_t length) {
  char safe[SD_SESSION_NAME_MAX + 1];
  sanitiseName(sdSessionName, safe, sizeof(safe));

  for (uint16_t index = 1; index <= 9999; ++index) {
    snprintf(buffer, length, "/SZD_%04u_%s%s", static_cast<unsigned>(index), safe, SD_RECORD_EXT);
    if (!SD.exists(buffer)) {
      return true;
    }
  }
  return false;
}

// Which GUI channels are switched on right now. The SD file only carries these,
// so a mixed measurement costs space in proportion to what is measured.
static void captureSdChannelMask() {
  sdRecordedCount = 0;
  for (uint8_t channel = 0; channel < TOTAL_ADS_CHANNELS; ++channel) {
    if (!channelSettings[channel].active) {
      continue;
    }
    sdRecordedChannels[sdRecordedCount++] = channel;
  }
}

static int32_t sdGainScalar(uint8_t channel) {
  static const int32_t gainScalarLut[7] = {1, 2, 4, 6, 8, 12, 24};
  const uint8_t ordinal = channel < TOTAL_ADS_CHANNELS ? channelSettings[channel].gainOrdinal : 6;
  return gainScalarLut[ordinal > 6 ? 6 : ordinal];
}

// The CSV header mirrors an OpenBCI recording, so `data_loader` and the analyzer
// read an SD file exactly like a live one - which is what makes the two
// recordings comparable.
//
// Every line is built by writing the parts straight to the file rather than into
// a fixed buffer. The previous version formatted the channel-name row into a
// 224-byte buffer and then memcpy'd 68 more bytes into it: with 12 channels that
// overflowed by 13 bytes, smashed the stack and panicked (reset reason 4), which
// is why the file was created but stayed empty.
static void writeSdHeader(File& file, uint32_t rateHz, uint32_t decimation) {
  file.print("%OpenBCI Raw EXG Data\r\n");
  file.print("%Number of channels = ");
  file.print(static_cast<unsigned>(sdRecordedCount));
  file.print("\r\n");
  file.print("%Sample Rate = ");
  file.print(static_cast<unsigned long>(rateHz));
  file.print(" Hz\r\n");
  file.print("%Session = ");
  file.print(sdSessionName);
  file.print("\r\n");
  file.print("%Data Format = EXG channels in microvolts, accelerometer in g, timestamp in seconds\r\n");
  file.print("%Source = SD card (ESP32 internal recording)\r\n");
  file.print("%Decimation = /");
  file.print(static_cast<unsigned long>(decimation));
  file.print("\r\n");
  file.print("%Started = ");
  file.print(static_cast<unsigned long>(millis()));
  file.print(" ms since boot\r\n");

  // Channel map and gains: the analyzer needs both to know which physical input
  // each column came from and how to interpret the accelerometer columns.
  file.print("%Channel Map = ");
  for (uint8_t i = 0; i < sdRecordedCount; ++i) {
    if (i > 0) {
      file.print(',');
    }
    file.print(static_cast<unsigned>(sdRecordedChannels[i] + 1));
  }
  file.print("\r\n");

  file.print("%Board Channel Gains = ");
  for (uint8_t i = 0; i < sdRecordedCount; ++i) {
    if (i > 0) {
      file.print(',');
    }
    file.print(static_cast<long>(sdGainScalar(sdRecordedChannels[i])));
  }
  file.print("\r\n");

  file.print("%Reference ADS1 = ");
  file.print(referenceModeName(chipReferenceMode[0]));
  file.print("\r\n");
  file.print("%Reference ADS2 = ");
  file.print(referenceModeName(chipReferenceMode[1]));
  file.print("\r\n");

  // Channel-name row, exactly as an OpenBCI file has it, then the accelerometer
  // and bookkeeping columns.
  for (uint8_t i = 0; i < sdRecordedCount; ++i) {
    if (i > 0) {
      file.print(',');
    }
    file.print("EXG Channel ");
    file.print(static_cast<unsigned>(i));
  }
  file.print(",Accel Channel 0,Accel Channel 1,Accel Channel 2,Marker,Timestamp\r\n");
  file.flush();
}

static bool openSdFile(uint32_t durationMs, const char* label) {
  if (!sdPresent) {
    emitFailure("SD card not available");
    return false;
  }

  closeSdFile();
  captureSdChannelMask();
  if (sdRecordedCount == 0) {
    emitFailure("every channel is off, nothing to record");
    return false;
  }

  if (!makeSdRecordingName(sdFileName, sizeof(sdFileName))) {
    emitFailure("no SD filename available");
    return false;
  }

  if (sdMutex != nullptr) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
  }
  sdFile = SD.open(sdFileName, FILE_WRITE);
  if (!sdFile) {
    sdFileName[0] = '\0';
    if (sdMutex != nullptr) {
      xSemaphoreGive(sdMutex);
    }
    emitFailure("failed to open SD file");
    return false;
  }

  sdSamplesWritten = 0;
  sdSamplesDropped = 0;
  sdBytesWritten = 0;
  sdWriteBufferLength = 0;
  sdWriteBufferSamples = 0;
  sdBufferFlushes = 0;
  sdMarkerPending = false;
  // Per-recording quality figures: a peak from the previous file would hide a
  // problem in this one.
  sdQueuePeak = 0;
  sdMaxFlushMs = 0;
  sdBytesPerSecond = 0;
  sdRateWindowBytes = 0;
  sdRateWindowStartMs = 0;
  // Everything already in the queue belongs to the previous recording.  sdGeneration++;
  if (sdQueue != nullptr) {
    SdSample stale;
    while (xQueueReceive(sdQueue, &stale, 0) == pdTRUE) {
    }
  }
  writeSdHeader(sdFile, sampleRateFromCode(sampleRateCode) / max(decimFactor, static_cast<uint8_t>(1)),
                decimFactor);
  // The header must be on the card before a single sample is accepted: a file
  // that says how many channels follow is what makes it loadable later.
  if (sdFile.position() == 0) {
    sdFile.close();
    sdFileName[0] = '\0';
    if (sdMutex != nullptr) {
      xSemaphoreGive(sdMutex);
    }
    emitFailure("could not write the SD header - the card may be full or write-protected");
    return false;
  }
  sdBytesWritten = static_cast<uint32_t>(sdFile.position());
  sdHeaderWritten = true;

  sdRecording = true;
  sdRecordStartMs = millis();
  sdRecordDurationMs = durationMs;
  if (sdMutex != nullptr) {
    xSemaphoreGive(sdMutex);
  }

  char msg[96];
  snprintf(msg, sizeof(msg), "SD recording started: %s (%s)", sdFileName, label != nullptr ? label : "manual");
  emitSuccess(msg);
  return true;
}

static void closeSdFile() {
  if (sdMutex != nullptr) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
  }

  // The tail of the recording is still in RAM, so it has to reach the card before
  // the file is closed - otherwise the last few hundred milliseconds are lost.
  sdFlushWriteBufferLocked();
  if (sdFile) {
    sdFile.flush();
    sdFile.close();
  }

  sdRecording = false;
  sdRecordStartMs = 0;
  sdRecordDurationMs = 0;
  sdHeaderWritten = false;
  sdWriteBufferLength = 0;
  sdWriteBufferSamples = 0;

  if (sdMutex != nullptr) {
    xSemaphoreGive(sdMutex);
  }
}

static void updateSdRecordingTimeout() {
  if (!sdRecording || sdRecordDurationMs == 0) {
    return;
  }

  if ((millis() - sdRecordStartMs) >= sdRecordDurationMs) {
    char finishedFile[sizeof(sdFileName)];
    strncpy(finishedFile, sdFileName, sizeof(finishedFile) - 1);
    finishedFile[sizeof(finishedFile) - 1] = '\0';
    closeSdFile();

    char msg[112];
    snprintf(msg, sizeof(msg), "SD recording finished: %s", finishedFile[0] != '\0' ? finishedFile : "file closed");
    emitSuccess(msg);
    emitSdStatus("timeout");
  }
}

static const SdModeSetting* findSdMode(char command) {
  for (uint8_t i = 0; i < SD_MODE_COUNT; ++i) {
    if (SD_MODE_SETTINGS[i].command == command) {
      return &SD_MODE_SETTINGS[i];
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// SD card settings persistence
//
// The live window writes the board configuration into /SZD_CFG.TXT after it has
// programmed the converters, so the configuration that the record button applies
// is byte-for-byte the one the operator just reviewed - not a second, guessed one.
// ---------------------------------------------------------------------------

static void saveSdConfiguration() {
  if (!sdPresent) {
    emitFailure("SD card not available");
    return;
  }

  File file = SD.open(SD_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    emitFailure("could not open the SD settings file");
    return;
  }

  char line[96];
  file.print("SZDCFG 1\r\n");
  snprintf(line, sizeof(line), "name=%s\r\n", sdSessionName);
  file.print(line);
  snprintf(line, sizeof(line), "rate=%u\r\n", static_cast<unsigned>(sampleRateCode));
  file.print(line);
  snprintf(line, sizeof(line), "decim=%u\r\n", static_cast<unsigned>(decimFactor));
  file.print(line);
  snprintf(line, sizeof(line), "spi=%lu\r\n", static_cast<unsigned long>(adsSpiClockHz));
  file.print(line);
  snprintf(line, sizeof(line), "ref1=%u\r\n", static_cast<unsigned>(chipReferenceMode[0]));
  file.print(line);
  snprintf(line, sizeof(line), "ref2=%u\r\n", static_cast<unsigned>(chipReferenceMode[1]));
  file.print(line);

  // One line per converter channel: active, gain, input, bias, srb2.
  for (uint8_t channel = 0; channel < TOTAL_ADS_CHANNELS; ++channel) {
    const ChannelSetting& setting = channelSettings[channel];
    snprintf(line, sizeof(line), "ch%u=%u,%u,%u,%u,%u\r\n", static_cast<unsigned>(channel),
             setting.active ? 1U : 0U, static_cast<unsigned>(setting.gainOrdinal),
             static_cast<unsigned>(setting.inputTypeOrdinal), setting.biasInclude ? 1U : 0U,
             setting.srb2Connect ? 1U : 0U);
    file.print(line);
  }

  file.flush();
  file.close();

  char msg[96];
  snprintf(msg, sizeof(msg), "SD settings saved: %u channel(s), %.1f s of data per MB",
           static_cast<unsigned>(TOTAL_ADS_CHANNELS), 0.0);
  emitSuccess(msg);
  emitSdStatus("saved");
}

// Parses "key=value" lines. Returns true when at least one channel line matched,
// so a truncated file is reported instead of silently applied.
static bool loadSdConfiguration(bool applyToHardware) {
  if (!sdPresent) {
    emitFailure("SD card not available");
    return false;
  }
  if (!SD.exists(SD_CONFIG_PATH)) {
    emitFailure("no SD settings stored yet - send them from the live window first");
    return false;
  }

  File file = SD.open(SD_CONFIG_PATH, FILE_READ);
  if (!file) {
    emitFailure("could not open the SD settings file");
    return false;
  }

  uint8_t newRate = sampleRateCode;
  uint8_t newDecim = decimFactor;
  uint32_t newSpi = adsSpiClockHz;
  uint8_t refs[2] = {chipReferenceMode[0], chipReferenceMode[1]};
  bool active[TOTAL_ADS_CHANNELS];
  uint8_t gain[TOTAL_ADS_CHANNELS];
  uint8_t input[TOTAL_ADS_CHANNELS];
  bool bias[TOTAL_ADS_CHANNELS];
  bool srb2[TOTAL_ADS_CHANNELS];
  char name[SD_SESSION_NAME_MAX + 1];
  strncpy(name, sdSessionName, sizeof(name) - 1);
  name[sizeof(name) - 1] = '\0';
  for (uint8_t channel = 0; channel < TOTAL_ADS_CHANNELS; ++channel) {
    active[channel] = channelSettings[channel].active;
    gain[channel] = channelSettings[channel].gainOrdinal;
    input[channel] = channelSettings[channel].inputTypeOrdinal;
    bias[channel] = channelSettings[channel].biasInclude;
    srb2[channel] = channelSettings[channel].srb2Connect;
  }

  uint8_t channelLines = 0;
  char line[128];
  size_t length = 0;
  // Read one line at a time; the file is a few hundred bytes, so a character loop
  // is both simpler and cheaper than pulling it all into RAM.
  while (file.available()) {
    const char c = static_cast<char>(file.read());
    if (c == '\n') {
      line[length] = '\0';
      if (length > 0 && line[length - 1] == '\r') {
        line[--length] = '\0';
      }
      char* equals = strchr(line, '=');
      if (equals != nullptr) {
        *equals = '\0';
        const char* key = line;
        const char* value = equals + 1;
        if (strcmp(key, "name") == 0) {
          strncpy(name, value, sizeof(name) - 1);
          name[sizeof(name) - 1] = '\0';
        } else if (strcmp(key, "rate") == 0) {
          newRate = static_cast<uint8_t>(atoi(value) & 0x07);
        } else if (strcmp(key, "decim") == 0) {
          newDecim = static_cast<uint8_t>(atoi(value) < 1 ? 1 : atoi(value));
        } else if (strcmp(key, "spi") == 0) {
          newSpi = static_cast<uint32_t>(strtoul(value, nullptr, 10));
        } else if (strcmp(key, "ref1") == 0) {
          refs[0] = static_cast<uint8_t>(atoi(value) % 3);
        } else if (strcmp(key, "ref2") == 0) {
          refs[1] = static_cast<uint8_t>(atoi(value) % 3);
        } else if (key[0] == 'c' && key[1] == 'h') {
          const int channel = atoi(key + 2);
          if (channel >= 0 && channel < TOTAL_ADS_CHANNELS) {
            unsigned a = 1, g = 6, i = 0, b = 1, s = 0;
            if (sscanf(value, "%u,%u,%u,%u,%u", &a, &g, &i, &b, &s) == 5) {
              active[channel] = a != 0;
              gain[channel] = static_cast<uint8_t>(g > 6 ? 6 : g);
              input[channel] = static_cast<uint8_t>(i > 7 ? 0 : i);
              bias[channel] = b != 0;
              srb2[channel] = s != 0;
              channelLines++;
            }
          }
        }
      }
      length = 0;
      continue;
    }
    if (length + 1 < sizeof(line)) {
      line[length++] = c;
    }
  }
  file.close();

  if (channelLines == 0) {
    emitFailure("the SD settings file has no channel lines");
    return false;
  }

  strncpy(sdSessionName, name, sizeof(sdSessionName) - 1);
  sdSessionName[sizeof(sdSessionName) - 1] = '\0';

  if (!applyToHardware) {
    char msg[96];
    snprintf(msg, sizeof(msg), "SD settings read: %u channel(s), session %s",
             static_cast<unsigned>(channelLines), sdSessionName);
    emitSuccess(msg);
    return true;
  }

  // Apply exactly as a host command would, so the converters, the serial link and
  // the SD file all agree on what is being measured.
  const bool wasStreaming = stopStreaming();
  chipReferenceMode[0] = refs[0];
  chipReferenceMode[1] = refs[1];
  for (uint8_t channel = 0; channel < TOTAL_ADS_CHANNELS; ++channel) {
    ChannelSetting& setting = channelSettings[channel];
    setting.active = active[channel];
    setting.gainOrdinal = gain[channel];
    setting.inputTypeOrdinal = input[channel];
    setting.biasInclude = bias[channel] && active[channel];
    setting.srb2Connect = srb2[channel];
  }
  sampleRateCode = newRate;
  decimFactor = newDecim < 1 ? 1 : newDecim;
  if (newSpi >= 1000000UL && newSpi <= 8000000UL) {
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (devices[i].linkOk) {
        devices[i].ads->setSpiClock(newSpi);
      }
    }
    adsSpiClockHz = newSpi;
  }

  programAllDevices();
  if (wasStreaming) {
    startStreaming();
  }

  char msg[96];
  snprintf(msg, sizeof(msg), "SD settings applied: %u channel(s), %lu SPS, session %s",
           static_cast<unsigned>(channelLines),
           static_cast<unsigned long>(sampleRateFromCode(sampleRateCode)), sdSessionName);
  emitSuccess(msg);
  return true;
}

// Free space, total space and how long that lasts at the current settings. The
// estimate is deliberately computed from the real line length of the channel set
// rather than a bytes-per-channel guess.
//
// Byte counts are 64-bit throughout: a 16 GB card is 15,938,355,200 bytes, which
// does not fit in a uint32_t. The earlier version truncated it and reported
// 2.8 GB, which made the free-space estimate meaningless.
static uint32_t sdEstimatedSeconds(const uint64_t freeBytes) {
  captureSdChannelMask();
  if (sdRecordedCount == 0) {
    return 0;
  }
  // Approximate line length: per channel up to 8 characters (sign + 6 digits +
  // comma), then three accelerometer fields, marker and a 17-character timestamp.
  const uint64_t perSample = static_cast<uint64_t>(sdRecordedCount) * 8ULL + 40ULL;
  const uint32_t rate = sampleRateFromCode(sampleRateCode) / max(decimFactor, static_cast<uint8_t>(1));
  if (rate == 0) {
    return 0;
  }
  const uint64_t seconds = (freeBytes / perSample) / rate;
  return static_cast<uint32_t>(seconds > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : seconds);
}

// Total and free capacity of the card, in bytes.
//
// The total comes from the card's own CSD (SD.cardSize(), which reports 15200 MB
// for a 16 GB card) rather than from SD.totalBytes(). The latter is derived from
// the FAT volume's cluster chain and under-reports on large FAT32 cards; used
// space is still read from the FAT because that is the only place it exists.
static void sdCapacity(uint64_t& totalBytes, uint64_t& freeBytes) {
  totalBytes = 0;
  freeBytes = 0;
  if (!sdPresent) {
    return;
  }
  totalBytes = SD.cardSize();
  if (totalBytes == 0) {
    totalBytes = SD.totalBytes();
  }
  uint64_t used = SD.usedBytes();
  if (used > totalBytes) {
    // A misreported used count must not produce a negative free space.
    used = totalBytes;
  }
  freeBytes = totalBytes - used;
}

// SD.usedBytes() walks the FAT's cluster chain, which is the one card operation
// slow enough to be visible in the data: it holds the card bus and the CPU for
// tens of milliseconds, long enough to delay a conversion. So the figure is
// measured only when the host explicitly asks for it (or when it has never been
// measured) and reused for every other report, which keeps the automatic action
// replies - start, stop, timeout - free of it.
static uint64_t sdCachedTotalBytes = 0;
static uint64_t sdCachedFreeBytes = 0;
static bool sdCapacityValid = false;

static void sdCapacityCached(uint64_t& totalBytes, uint64_t& freeBytes, bool refresh) {
  if (refresh || !sdCapacityValid) {
    sdCapacity(sdCachedTotalBytes, sdCachedFreeBytes);
    sdCapacityValid = true;
  }
  totalBytes = sdCachedTotalBytes;
  freeBytes = sdCachedFreeBytes;
}

// The status line is a flat KEY=VALUE token list, so a session name containing a
// space would silently truncate NAME=. Spaces become '_' here; the file itself is
// already named from the sanitised form.
static void statusSafeName(const char* source, char* target, size_t length) {
  size_t out = 0;
  for (size_t i = 0; source != nullptr && source[i] != '\0' && out + 1 < length; ++i) {
    const char c = source[i];
    target[out++] = (c == ' ' || c == '\t') ? '_' : c;
  }
  target[out] = '\0';
}

// The line is 280+ characters in the worst case (long session name, 16 GB card,
// large byte counts), so the buffer must stay above that or DONE=1 gets cut off
// and the host silently ignores the report.
static void emitSdStatus(const char* event) {
  const bool present = sdPresent;
  // Only an explicit request re-reads the card's free space (see sdCapacityCached):
  // the automatic start/stop/timeout replies reuse the last measurement.
  const bool refreshCapacity = (event == nullptr) || (event[0] == '\0') ||
                               strcmp(event, "status") == 0 || strcmp(event, "saved") == 0;
  uint64_t totalBytes = 0;
  uint64_t freeBytes = 0;
  sdCapacityCached(totalBytes, freeBytes, refreshCapacity);
  const uint64_t totalMb = totalBytes / (1024ULL * 1024ULL);
  const uint64_t freeMb = freeBytes / (1024ULL * 1024ULL);
  const uint32_t rate = sampleRateFromCode(sampleRateCode) / max(decimFactor, static_cast<uint8_t>(1));
  captureSdChannelMask();
  char nameField[SD_SESSION_NAME_MAX + 1];
  statusSafeName(sdSessionName, nameField, sizeof(nameField));

  char line[448];
  snprintf(line, sizeof(line),
           "[SD] EVENT=%s PRESENT=%d RECORDING=%d FILE=%s NAME=%s CHANNELS=%u RATE=%lu "
           "TOTAL_MB=%lu FREE_MB=%lu FREE_BYTES=%lu EST_SEC=%lu SAMPLES=%lu DROPPED=%lu BYTES=%lu "
           "QUEUED=%u QPEAK=%u QBUF=%u BUFKB=%u WRITE_MS=%lu RATE_KB=%lu SPIKHZ=%lu HEAP=%u DONE=1\n",
           (event != nullptr && event[0] != '\0') ? event : "-", present ? 1 : 0, sdRecording ? 1 : 0,
           sdFileName[0] != '\0' ? sdFileName : "-", nameField,
           static_cast<unsigned>(sdRecordedCount), static_cast<unsigned long>(rate),
           static_cast<unsigned long>(totalMb), static_cast<unsigned long>(freeMb),
           static_cast<unsigned long>(freeBytes > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : freeBytes),
           static_cast<unsigned long>(sdEstimatedSeconds(freeBytes)),
           static_cast<unsigned long>(sdSamplesWritten), static_cast<unsigned long>(sdSamplesDropped),
           static_cast<unsigned long>(sdBytesWritten),
           // How much of the RAM buffering is in use. QUEUED is the sample queue
           // right now, QPEAK its high-water mark: a peak near the capacity means
           // the card cannot keep up with the sample rate, not that the buffer is
           // too small. WRITE_MS is the longest single card transaction, i.e. the
           // stall the Bluetooth flush task had to preempt.
           static_cast<unsigned>(sdQueue != nullptr ? uxQueueMessagesWaiting(sdQueue) : 0),
           static_cast<unsigned>(sdQueuePeak),
           static_cast<unsigned>(sdWriteBufferLength),
           static_cast<unsigned>(SD_WRITE_BUFFER_BYTES / 1024),
           static_cast<unsigned long>(sdMaxFlushMs),
           static_cast<unsigned long>(sdBytesPerSecond / 1024UL),
           static_cast<unsigned long>(sdSpiClockActive / 1000UL),
           static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.print(line);
}

// SD rows are produced by publishConversion(), which only runs while streaming.
// Starting a recording with the stream stopped therefore wrote a header and then
// nothing at all, and the host appeared to freeze because it was waiting for
// samples that could never arrive. Any recording start now guarantees the stream.
static void startSdRecordingFromBoard(const char* reason) {
  if (sdRecording) {
    emitFailure("SD recording already running");
    emitSdStatus("busy");
    return;
  }
  openSdFile(0, reason);
  if (!sdRecording) {
    emitSdStatus("fail");
    return;
  }
  // A recording is a new, explicit reason to stream, so a previous "stop
  // streaming" must not keep the conversion stream off.
  suppressAutoStream = false;
  if (!streaming) {
    startStreaming();
  }
  emitSdStatus("start");
}

static void stopSdRecordingFromBoard(const char* reason) {
  if (!sdRecording) {
    emitFailure("no SD recording is running");
    emitSdStatus("idle");
    return;
  }
  char finished[sizeof(sdFileName)];
  strncpy(finished, sdFileName, sizeof(finished) - 1);
  finished[sizeof(finished) - 1] = '\0';
  const uint32_t samples = sdSamplesWritten;
  const uint32_t dropped = sdSamplesDropped;
  closeSdFile();
  char msg[120];
  snprintf(msg, sizeof(msg), "SD recording stopped (%s): %s, %lu samples, %lu dropped",
           reason, finished[0] != '\0' ? finished : "file closed",
           static_cast<unsigned long>(samples), static_cast<unsigned long>(dropped));
  emitSuccess(msg);
  // Reported after closeSdFile() so FILE= still names the finished file while
  // RECORDING=0 tells the host the recording is over. closeSdFile() deliberately
  // keeps sdFileName so the finished name stays reportable.
  emitSdStatus("stop");
}

// While a card recording is running the stream must not stay stopped: without
// conversions the recording writes nothing, and the host looks frozen because it
// waits for samples that can never arrive. A host that has just spoken therefore
// gets the stream back. The rule is deliberately scoped to an active recording and
// gated on the explicit stop, so the 5 s status poll can never undo a deliberate
// "stop streaming".
static void ensureStreamingForHost() {
  if (streaming || suppressAutoStream || !sdRecording) {
    return;
  }
  const uint32_t last = lastHostByteMs;
  if (last == 0 || (millis() - last) > HOST_PRESENCE_WINDOW_MS) {
    return;
  }
  startStreaming();
}

// Record button (PIN_RECORD_BUTTON, GPIO35 on the current board). A press loads
// the configuration that the live window stored on the card, applies it, and then
// starts recording - so the physical button works standalone, without a host.
static void toggleSdRecordingFromButton() {
  if (sdRecording) {
    stopSdRecordingFromBoard("button");
    return;
  }
  if (!sdPresent) {
    emitFailure("record button pressed but no SD card is present");
    return;
  }
  // Apply the stored configuration first; without one, record with whatever is
  // already programmed so the button is never dead.
  loadSdConfiguration(true);
  startSdRecordingFromBoard("button");
}

// RISING-edge interrupt: the pin is pulled down, so a press drives it high and
// fires exactly one interrupt per press. The ISR only timestamps the edge;
// debouncing and the actual work stay out of interrupt context.
static void IRAM_ATTR recordButtonIsr() {
  const uint32_t now = millis();
  if (now - recordButtonLastEdgeMs < RECORD_BUTTON_DEBOUNCE_MS) {
    return;
  }
  recordButtonLastEdgeMs = now;
  recordButtonPressed = true;
}

static void beginRecordButton() {
  // Internal pull-down, so a released button reads 0 and a pressed one reads 1.
  // GPIO34..39 have no pull resistors active at reset, but they do have internal
  // ones firmware can enable, so no external resistor is required. Without the
  // pull-down the pin floats and a rising edge may never be seen at all.
  pinMode(PIN_RECORD_BUTTON, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_RECORD_BUTTON), recordButtonIsr, RISING);
  // A button already held at power-up produces no edge, so seed the flag from the
  // pin state and let the main loop act on it.
  if (digitalRead(PIN_RECORD_BUTTON) != 0) {
    recordButtonPressed = true;
  }
}

static const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_EXT:
      return "external pin";
    case ESP_RST_SW:
      return "software restart";
    case ESP_RST_PANIC:
      return "PANIC - firmware crashed";
    case ESP_RST_INT_WDT:
      return "interrupt watchdog - a task blocked interrupts";
    case ESP_RST_TASK_WDT:
      return "task watchdog - a task stopped yielding";
    case ESP_RST_WDT:
      return "other watchdog";
    case ESP_RST_DEEPSLEEP:
      return "deep sleep wake";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT - the supply sagged";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      return "unknown";
  }
}

static void emitDebugLine(const char* prefix, const char* payload) {
  if (!USE_DEBUG_APP_MODE) {
    return;
  }

  Serial.print(prefix);
  if (payload != nullptr && payload[0] != '\0') {
    Serial.print(payload);
  }
  Serial.print('\n');
}

static void emitRadioResponse(const char* msg) {
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#RADIO ", msg);
  } else {
    Serial.println(msg);
  }
}

static void emitRawResponse(const char* msg) {
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#RAW ", msg);
  } else {
    Serial.print(msg);
    Serial.print("$$$");
  }
}

static void emitSuccess(const char* msg) {
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#OK ", msg);
  } else {
    Serial.print("Success: ");
    Serial.print(msg);
    Serial.print("$$$");
  }
}

static void emitFailure(const char* msg) {
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#ERR ", msg);
  } else {
    Serial.print("Failure: ");
    Serial.print(msg);
    Serial.print("$$$");
  }
}

// One packet is 33 bytes on the wire (10 bits per byte). Keeping 10% headroom
// guarantees Serial.write() never blocks or truncates, so the acquisition task
// always reads the converters right after DRDY.
static uint32_t maxSustainedPacketRate() {
  constexpr uint32_t PACKET_WIRE_BYTES = 33;
  const uint32_t usableBitsPerSecond = (SERIAL_BAUD / 10UL) * 9UL;
  return usableBitsPerSecond / (PACKET_WIRE_BYTES * 10UL);
}

// SPI budget between the ESP32 and the converters. One RDATAC frame is always
// 3 status bytes + 8 channel slots = 27 bytes (216 bits), and both devices are
// read inside the same sample period because they share START, so every output
// sample costs two frames on the bus.
static uint32_t spiFrameTimeUs() {
  constexpr uint32_t SPI_FRAME_BITS = 3 * 8 + 8 * 8 * 3;  // 216
  const uint32_t totalBits = SPI_FRAME_BITS * DEVICE_COUNT;
  return (totalBits * 1000000UL + adsSpiClockHz - 1UL) / adsSpiClockHz;
}

// Same 10% headroom as the serial limit: the bus must carry both frames with
// room left for the register housekeeping the acquisition task does.
static uint32_t maxSampleRateFromSpi() {
  const uint32_t frameUs = spiFrameTimeUs();
  if (frameUs == 0) {
    return 0;
  }
  return 900000UL / frameUs;
}

static uint32_t maxUsableSampleRate() {
  const uint32_t serialLimit = maxSustainedPacketRate();
  const uint32_t spiLimit = maxSampleRateFromSpi();
  return (spiLimit != 0 && spiLimit < serialLimit) ? spiLimit : serialLimit;
}

static uint16_t sampleRateFromCode(uint8_t code) {
  static const uint16_t sampleRates[] = {16000, 8000, 4000, 2000, 1000, 500, 250};
  if (code > 6) {
    return 250;
  }
  return sampleRates[code];
}

static void emitDefaultSettingsReport() {
  emitRawResponse("060110");
}

static void emitVersionString() {
  emitRawResponse("v3.1.5");
}

static uint8_t gainScalarForOrdinal(uint8_t gainOrdinal) {
  static const uint8_t gainScalars[] = {1, 2, 4, 6, 8, 12, 24};
  if (gainOrdinal > 6) {
    return 24;
  }
  return gainScalars[gainOrdinal];
}

static void emitDebugHello() {
  char msg[96];
  snprintf(msg, sizeof(msg), "{\"mode\":\"debug\",\"channels\":%u,\"baud\":%lu}", GUI_CHANNELS, static_cast<unsigned long>(SERIAL_BAUD));
  emitDebugLine("#HELLO ", msg);
}

static void emitDebugState() {
  if (!USE_DEBUG_APP_MODE) {
    return;
  }

  Serial.print("#STATE {\"streaming\":");
  Serial.print(streaming ? '1' : '0');
  Serial.print(",\"sample_rate_hz\":");
  Serial.print(sampleRateFromCode(sampleRateCode));
  Serial.print(",\"sample_rate_code\":");
  Serial.print(sampleRateCode);
  Serial.print(",\"board_mode\":");
  Serial.print(boardMode);
  Serial.print(",\"channels\":[");

  for (uint8_t i = 0; i < TOTAL_ADS_CHANNELS; ++i) {
    const ChannelSetting& setting = channelSettings[i];
    if (i != 0) {
      Serial.print(',');
    }
    Serial.print("{\"index\":");
    Serial.print(i + 1);
    Serial.print(",\"active\":");
    Serial.print(setting.active ? '1' : '0');
    Serial.print(",\"gain_ordinal\":");
    Serial.print(setting.gainOrdinal);
    Serial.print(",\"gain_scalar\":");
    Serial.print(gainScalarForOrdinal(setting.gainOrdinal));
    Serial.print(",\"input\":");
    Serial.print(setting.inputTypeOrdinal);
    Serial.print(",\"bias\":");
    Serial.print(setting.biasInclude ? '1' : '0');
    Serial.print(",\"srb2\":");
    Serial.print(setting.srb2Connect ? '1' : '0');
    Serial.print(",\"srb1\":");
    Serial.print(setting.srb1Connect ? '1' : '0');
    Serial.print(",\"imp_p\":");
    Serial.print(leadOffP[i] ? '1' : '0');
    Serial.print(",\"imp_n\":");
    Serial.print(leadOffN[i] ? '1' : '0');
    Serial.print('}');
  }

  Serial.print("]}\n");
}

static void printVersionInfo() {
  Serial.println("OpenBCI V3 8-16 channel");
  Serial.print("On Board ADS1299 Device ID: 0x");
  if (devices[0].deviceId < 0x10) {
    Serial.print('0');
  }
  Serial.println(devices[0].deviceId, HEX);
  if (DAISY_MODE) {
    Serial.print("On Daisy ADS1299 Device ID: 0x");
    if (devices[1].deviceId < 0x10) {
      Serial.print('0');
    }
    Serial.println(devices[1].deviceId, HEX);
  }
  Serial.print("LIS3DH Device ID: 0x");
  if (lis3dhWhoAmI < 0x10) {
    Serial.print('0');
  }
  Serial.println(lis3dhWhoAmI, HEX);
  Serial.print("SD card: ");
  Serial.println(sdPresent ? "present" : "not present");
  Serial.println("Firmware: v3.1.5");
  Serial.print("Bluetooth: ");
  if (btStackReady) {
    Serial.print(SZD_BT_DEVICE_NAME);
    Serial.print(" SPP ready, tx ");
    Serial.print(SZD_BT_TX_POWER_DBM);
    Serial.print(" dBm, buffer ");
    Serial.print(static_cast<unsigned>(btRingSize / 1024));
    Serial.println(" kB");
  } else {
    Serial.println("off (USB only)");
  }
  Serial.print("Serial link limit: ");
  Serial.print(maxSustainedPacketRate());
  Serial.println(" SPS");
  Serial.print("SPI clock: ");
  Serial.print(adsSpiClockHz / 1000UL);
  Serial.print(" kHz, frame ");
  Serial.print(spiFrameTimeUs());
  Serial.print(" us x ");
  Serial.print(static_cast<unsigned>(DEVICE_COUNT));
  Serial.print(" devices, SPI link limit: ");
  Serial.print(maxSampleRateFromSpi());
  Serial.println(" SPS");
  Serial.print("Decimation: ");
  Serial.print(static_cast<unsigned>(decimFactor));
  Serial.print(" -> ");
  Serial.print(sampleRateFromCode(sampleRateCode) / decimFactor);
  Serial.println(" packets/s");
  Serial.print("Reference mode: ");
  {
    char referenceText[48];
    formatReferenceSummary(referenceText, sizeof(referenceText));
    Serial.println(referenceText);
  }
  Serial.print("$$$");
}

static int channelSelectorToIndex(char selector) {
  if (selector >= '1' && selector <= '8') {
    return selector - '1';
  }

  if (!daisyActive) {
    return -1;
  }

  switch (selector) {
    case 'Q': return 8;
    case 'W': return 9;
    case 'E': return 10;
    case 'R': return 11;
    case 'T': return 12;
    case 'Y': return 13;
    case 'U': return 14;
    case 'I': return 15;
    default: return -1;
  }
}

// Maps a GUI logical channel (0..GUI_CHANNELS-1) to a real channel index
// (0..TOTAL_ADS_CHANNELS-1), or -1 for the padding channels in daisy mode.
static int logicalToReal(int logical) {
  if (logical < 0 || logical >= GUI_CHANNELS) {
    return -1;
  }

  if (!daisyActive) {
    return logical;
  }

  if (logical < 8) {
    return (logical < ADS_CHANNELS_PER_CHIP) ? logical : -1;
  }

  const int daisyLocal = logical - 8;
  return (daisyLocal < ADS_CHANNELS_PER_CHIP) ? (ADS_CHANNELS_PER_CHIP + daisyLocal) : -1;
}

static bool setDataRateForAll(uint8_t drBits) {
  const bool wasStreaming = stopStreaming();

  bool ok = true;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }
    ok = devices[i].ads->setDataRateBits(drBits) && ok;
  }
  if (ok) {
    sampleRateCode = drBits;
  }

  if (wasStreaming) {
    startStreaming();
  }
  return ok;
}

static bool applyChannelToHardware(uint8_t channelIndex) {
  if (channelIndex >= TOTAL_ADS_CHANNELS) {
    return false;
  }

  const size_t deviceIndex = channelIndex / ADS_CHANNELS_PER_CHIP;
  DeviceCtx& dev = devices[deviceIndex];
  if (!dev.linkOk) {
    return false;
  }

  const uint8_t localIndex = channelIndex % ADS_CHANNELS_PER_CHIP;
  const ChannelSetting& setting = channelSettings[channelIndex];
  // SRB2 is the per-channel CHnSET bit, so it is only programmed while this
  // converter is in SRB2 mode and only for the inputs the host asked for. That
  // is what lets one input stay true differential (a common-mode monitor)
  // while the rest of the chip is referred to SRB2. SRB1 and differential mode
  // force it off, because MISC1 owns the SRB1 switch for the whole chip.
  const bool srb2 = setting.active && (chipReferenceMode[deviceIndex] == REFERENCE_MODE_SRB2) &&
                    setting.srb2Connect;
  bool ok = dev.ads->configureChannel(
      localIndex,
      setting.active,
      setting.gainOrdinal,
      setting.inputTypeOrdinal,
      setting.biasInclude,
      srb2,
      false);

  if (!setting.active) {
    leadOffP[channelIndex] = false;
    leadOffN[channelIndex] = false;
    ok = dev.ads->setLeadOffForChannel(localIndex, false, false) && ok;
  }

  return ok;
}

// Programs the reference routing of one converter and rewrites its CHnSET
// registers, because switching mode also changes the SRB2 bit of every channel.
static bool applyReferenceToDevice(size_t deviceIndex) {
  if (deviceIndex >= DEVICE_COUNT) {
    return false;
  }
  DeviceCtx& dev = devices[deviceIndex];
  if (!dev.linkOk) {
    return true;
  }

  const uint8_t mode = chipReferenceMode[deviceIndex];
  bool ok = dev.ads->setSrb1Routing(mode == REFERENCE_MODE_SRB1);
  if (mode != REFERENCE_MODE_SRB2) {
    // Drop every per-channel SRB2 switch before the channel rewrite.
    ok = dev.ads->setSrb2ForAllChannels(false) && ok;
  }
  for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
    ok = applyChannelToHardware(static_cast<uint8_t>(dev.channelOffset + ch)) && ok;
  }
  return ok;
}

// Sets one converter's reference mode. Used by the 'n' command so the two
// halves of the board can run different references.
static bool applyChipReferenceMode(size_t deviceIndex, uint8_t mode) {
  if (deviceIndex >= DEVICE_COUNT || mode > REFERENCE_MODE_SRB2) {
    return false;
  }

  const bool wasStreaming = stopStreaming();
  chipReferenceMode[deviceIndex] = mode;
  const bool ok = applyReferenceToDevice(deviceIndex);

  if (wasStreaming) {
    startStreaming();
  }
  return ok;
}

// Sets the reference mode on both converters ('N' command).
static bool applyReferenceMode(uint8_t mode) {
  if (mode > REFERENCE_MODE_SRB2) {
    return false;
  }

  const bool wasStreaming = stopStreaming();
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    chipReferenceMode[i] = mode;
  }

  bool ok = true;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    ok = applyReferenceToDevice(i) && ok;
  }

  if (wasStreaming) {
    startStreaming();
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Full configuration apply + readback verification
//
// Every setting command only touches the one register it owns, so a partially
// applied configuration (a write that did not stick, a converter that was not
// linked when the settings were replayed, a stray reset) could previously
// survive until the operator noticed wrong data. This function is the single
// authoritative "program everything from the cached state, then check what the
// silicon really holds" path:
//
//   * called from startStreaming(), so a broken configuration can never persist
//     into a new stream, and
//   * called by the 'Y' command, which the GUI exposes as "Repair & verify".
//
// The verification compares each register against the value derived from the
// cached state, so the report names the exact register that is wrong instead of
// leaving the operator to guess.
// ---------------------------------------------------------------------------

struct RegisterCheck {
  const char* name;
  uint8_t address;
  uint8_t actual;
  uint8_t expected;
};

static uint8_t expectedChannelRegister(uint8_t channelIndex) {
  const ChannelSetting& setting = channelSettings[channelIndex];
  const uint8_t gain = setting.gainOrdinal > 6 ? 6 : setting.gainOrdinal;
  if (!setting.active) {
    // Powered down, MUX = input shorted: no signal is routed.
    return static_cast<uint8_t>(0x80 | (gain << 4) | 0x01);
  }
  const size_t deviceIndex = channelIndex / ADS_CHANNELS_PER_CHIP;
  const bool srb2 = chipReferenceMode[deviceIndex] == REFERENCE_MODE_SRB2 && setting.srb2Connect;
  return static_cast<uint8_t>((gain << 4) | (srb2 ? 0x08 : 0x00) | (setting.inputTypeOrdinal & 0x07));
}

static uint8_t expectedBiasMask(size_t deviceIndex) {
  uint8_t mask = 0;
  for (uint8_t ch = 0; ch < devices[deviceIndex].channelCount; ++ch) {
    const uint8_t real = static_cast<uint8_t>(devices[deviceIndex].channelOffset + ch);
    if (channelSettings[real].active && channelSettings[real].biasInclude) {
      mask |= static_cast<uint8_t>(1U << ch);
    }
  }
  return mask;
}

static uint8_t expectedConfig1(size_t deviceIndex) {
  const uint8_t base = 0x90 | 0x40;  // reserved bits + DAISY_EN (multiple readback)
  const uint8_t clock = devices[deviceIndex].clockMaster ? 0x20 : 0x00;
  return static_cast<uint8_t>(base | clock | (sampleRateCode & 0x07));
}

static const char* const kChannelRegisterNames[8] = {"CH1SET", "CH2SET", "CH3SET", "CH4SET",
                                                     "CH5SET", "CH6SET", "CH7SET", "CH8SET"};

// Programs every register of every linked converter from the cached state.
// Called before each stream start so a half-applied configuration cannot be
// streamed, and by the 'Y' repair command.
static void programAllDevices() {
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    if (!dev.linkOk) {
      continue;
    }

    ADS1299* ads = dev.ads;
    const bool srb1 = chipReferenceMode[i] == REFERENCE_MODE_SRB1;

    // CONFIG3 (reference buffer on) and CONFIG2 (internal test signal off) are
    // not touched by the per-channel path, and a converter that was probed late
    // may never have received them.
    ads->applyCytonDefaults(sampleRateCode);
    ads->configureInternalTestSignal(0x00, 0x00, false);
    ads->setDataRateBits(sampleRateCode);
    ads->setSrb1Routing(srb1);
    for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
      const uint8_t real = static_cast<uint8_t>(dev.channelOffset + ch);
      const ChannelSetting& setting = channelSettings[real];
      const bool srb2 = chipReferenceMode[i] == REFERENCE_MODE_SRB2 && setting.active && setting.srb2Connect;
      ads->configureChannel(ch, setting.active, setting.gainOrdinal, setting.inputTypeOrdinal,
                            setting.biasInclude, srb2, false);
    }
  }
}

// Reads every register back and reports the ones that do not match. Returns the
// number of problems so the caller can decide whether the state is usable.
static uint8_t verifyDeviceRegisters(size_t deviceIndex) {
  DeviceCtx& dev = devices[deviceIndex];
  if (!dev.linkOk) {
    return 0;
  }

  uint8_t regs[CONFIG4 + 1] = {0};
  dev.ads->readRegisters(0x00, CONFIG4 + 1, regs);

  const uint8_t config1 = expectedConfig1(deviceIndex);
  const uint8_t bias = expectedBiasMask(deviceIndex);
  const uint8_t misc1 = chipReferenceMode[deviceIndex] == REFERENCE_MODE_SRB1 ? 0x20 : 0x00;

  struct Expect {
    const char* name;
    uint8_t address;
    uint8_t value;
  };

  Expect expectations[CONFIG4 + 1];
  uint8_t count = 0;
  expectations[count++] = {"CONFIG1", CONFIG1, config1};
  expectations[count++] = {"CONFIG2", CONFIG2, 0xC0};
  expectations[count++] = {"CONFIG3", CONFIG3, 0xEC};
  expectations[count++] = {"BIAS_SENSP", BIAS_SENSP, bias};
  expectations[count++] = {"BIAS_SENSN", BIAS_SENSN, bias};
  expectations[count++] = {"MISC1", MISC1, misc1};
  for (uint8_t ch = 0; ch < dev.channelCount && count < CONFIG4 + 1; ++ch) {
    const uint8_t real = static_cast<uint8_t>(dev.channelOffset + ch);
    expectations[count++] = {kChannelRegisterNames[ch], static_cast<uint8_t>(CH1SET + ch),
                             expectedChannelRegister(real)};
  }

  uint8_t problems = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t actual = regs[expectations[i].address];
    if (actual == expectations[i].value) {
      continue;
    }
    problems++;
    Serial.printf("[CHK] BAD DEV=%s REG=%s GOT=%02X WANT=%02X\n", dev.name, expectations[i].name,
                  actual, expectations[i].value);
  }

  Serial.printf("[CHK] DEV=%s ID=%02X NCH=%u REGOK=%u CHECKED=%u\n", dev.name, regs[ID],
                static_cast<unsigned>(dev.channelCount), static_cast<unsigned>(count - problems),
                static_cast<unsigned>(count));

  // A converter can compare perfectly against the expected registers and still
  // read as a floating input. These are the two configurations that guarantee
  // it, and neither is visible in a register comparison:
  //
  //   * CHnSET bit 3 closed with no bias path - INP is driven from the SRB2 pin
  //     but INN has nothing to return to;
  //   * any reference routing at all with BIAS_SENSP/N = 0, i.e. no DC return
  //     path through the bias amplifier.
  uint8_t srb2Channels = 0;
  for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
    const uint8_t real = static_cast<uint8_t>(dev.channelOffset + ch);
    if (channelSettings[real].active && channelSettings[real].srb2Connect) {
      srb2Channels++;
    }
  }
  const uint8_t actualBiasP = regs[BIAS_SENSP];
  const uint8_t actualBiasN = regs[BIAS_SENSN];
  const uint8_t actualConfig3 = regs[CONFIG3];
  if (srb2Channels > 0) {
    Serial.printf("[CHK] NOTE DEV=%s SRB2_SWITCH=CLOSED CHANNELS=%u "
                  "EFFECT=INxP_DRIVEN_BY_SRB2_NOT_A_NEGATIVE_REFERENCE\n",
                  dev.name, static_cast<unsigned>(srb2Channels));
  }
  if ((srb2Channels > 0 || chipReferenceMode[deviceIndex] == REFERENCE_MODE_SRB1) &&
      actualBiasP == 0 && actualBiasN == 0) {
    Serial.printf("[CHK] WARN DEV=%s REASON=NO_DC_RETURN BIAS_SENSP=0 BIAS_SENSN=0 "
                  "EFFECT=inputs_float_and_rail\n",
                  dev.name);
  }
  if ((actualConfig3 & 0x80) == 0) {
    Serial.printf("[CHK] WARN DEV=%s REASON=REFERENCE_BUFFER_OFF CONFIG3=%02X "
                  "EFFECT=no_internal_reference\n",
                  dev.name, actualConfig3);
  }
  return problems;
}

static uint8_t repairAndVerify(const char* reason) {
  const bool wasStreaming = stopStreaming();

  programAllDevices();

  uint8_t totalProblems = 0;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      Serial.printf("[CHK] DEV=%s LINK=0 (not configured, check the SPI bus)\n", devices[i].name);
      continue;
    }
    totalProblems += verifyDeviceRegisters(i);
  }

  char reference[48];
  formatReferenceSummary(reference, sizeof(reference));
  Serial.printf("[CHK] CODE=%u SPS=%u DECIM=%u SPICLK=%lu REF=%s PROBLEMS=%u REASON=%s DONE=1\n",
                static_cast<unsigned>(sampleRateCode),
                static_cast<unsigned>(sampleRateFromCode(sampleRateCode)),
                static_cast<unsigned>(decimFactor), static_cast<unsigned long>(adsSpiClockHz),
                reference, static_cast<unsigned>(totalProblems), reason);

  if (wasStreaming) {
    startStreaming();
  }
  return totalProblems;
}

static bool applyLeadOffToHardware(uint8_t channelIndex) {
  if (channelIndex >= TOTAL_ADS_CHANNELS) {
    return false;
  }

  DeviceCtx& dev = devices[channelIndex / ADS_CHANNELS_PER_CHIP];
  if (!dev.linkOk) {
    return false;
  }

  const uint8_t localIndex = channelIndex % ADS_CHANNELS_PER_CHIP;
  return dev.ads->setLeadOffForChannel(localIndex, leadOffP[channelIndex], leadOffN[channelIndex]);
}

static void applyDefaultChannelState() {
  for (uint8_t i = 0; i < TOTAL_ADS_CHANNELS; ++i) {
    channelSettings[i].active = true;
    channelSettings[i].gainOrdinal = 6;
    channelSettings[i].inputTypeOrdinal = 0;
    channelSettings[i].biasInclude = true;
    // SRB2 (CHnSET bit 3) closes a switch between the SRB2 pin and this
    // channel's *non-inverting* input, so defaulting it on would short every
    // INxP to the SRB2 pin. On a board where SRB2 is not wired that leaves the
    // positive input floating and every channel reads as saturation or as pure
    // float noise - which is exactly what it did. Default to plain differential
    // and let the recorder program the reference the operator actually chose.
    channelSettings[i].srb2Connect = false;
    channelSettings[i].srb1Connect = false;
    leadOffP[i] = false;
    leadOffN[i] = false;
  }
  chipReferenceMode[0] = REFERENCE_MODE_DIFFERENTIAL;
  chipReferenceMode[1] = REFERENCE_MODE_DIFFERENTIAL;
}

static bool applyDefaultChannelSettings() {
  const bool wasStreaming = stopStreaming();

  applyDefaultChannelState();

  bool ok = true;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }
    ok = devices[i].ads->applyCytonDefaults(sampleRateCode) && ok;
    ok = devices[i].ads->configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31P2HZ) && ok;
    // applyCytonDefaults() only writes CHnSET, so the reference routing has to
    // be re-programmed or SRB1 could stay enabled from an earlier mode.
    ok = applyReferenceToDevice(i) && ok;
  }

  if (wasStreaming) {
    startStreaming();
  }

  return ok;
}

static void emitStateIfDebug() {
  if (USE_DEBUG_APP_MODE) {
    emitDebugState();
  }
}

static bool setInputTypeForAllActiveChannels(uint8_t inputType, uint8_t amplitudeCode, uint8_t freqCode) {
  const bool wasStreaming = stopStreaming();

  bool ok = true;
  for (uint8_t i = 0; i < TOTAL_ADS_CHANNELS; ++i) {
    if (channelSettings[i].active) {
      channelSettings[i].inputTypeOrdinal = inputType;
    }
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }
    ok = devices[i].ads->configureInternalTestSignal(amplitudeCode, freqCode) && ok;
    ok = devices[i].ads->setInputTypeForAllChannels(inputType) && ok;
  }

  if (wasStreaming) {
    startStreaming();
  }

  return ok;
}

// If a device was not present at bring-up it never received any configuration
// (no reference buffer, no channel setup). Re-check before streaming starts and
// configure anything that has since started answering.
static void recoverMissingDevices() {
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    if (dev.linkOk || !dev.ads->detect()) {
      continue;
    }

    dev.linkOk = true;
    dev.deviceId = dev.ads->readRegister(0x00);
    dev.ads->applyCytonDefaults(sampleRateCode);
    dev.ads->configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31P2HZ);
    applyReferenceToDevice(i);
    for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
      applyChannelToHardware(static_cast<uint8_t>(dev.channelOffset + ch));
    }
    attachInterrupt(digitalPinToInterrupt(dev.drdyPin), (i == 0) ? adsDrdyIsr0 : adsDrdyIsr1, FALLING);

    Serial.printf("[%s] recovered and configured at stream start\n", dev.name);
  }
}

static bool stopStreaming() {
  if (!streaming) {
    return false;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->stopContinuousConversion();
    }
  }

  streaming = false;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    devices[i].drdyPending = false;
  }
  emitStateIfDebug();
  return true;
}

static void startStreaming() {
  if (streaming) {
    return;
  }

  recoverMissingDevices();
  // Program every register from the cached state before conversion starts. A
  // half-applied configuration (a write that did not stick, a converter that was
  // not linked when the settings were replayed, a stray reset) would otherwise
  // be streamed as valid-looking but wrong data.
  programAllDevices();

  sampleCounter = 0;
  missedConversions = 0;
  droppedPackets = 0;
  corruptFrames = 0;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    devices[i].drdyPending = false;
  }
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->startContinuousConversion();
    }
  }

  streaming = true;
  emitStateIfDebug();
}

static uint32_t drdyTimeoutMs() {
  const uint16_t rate = sampleRateFromCode(sampleRateCode);
  uint32_t periodMs = 1000UL / static_cast<uint32_t>(rate);
  if (periodMs == 0) {
    periodMs = 1;
  }
  return periodMs * 4 + 5;
}

static bool waitForDeviceDrdy(DeviceCtx& dev, uint32_t timeoutMs) {
  const uint32_t startMs = millis();

  while (!dev.drdyPending) {
    if ((millis() - startMs) >= timeoutMs) {
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

// The 24-bit status word of every RDATAC frame starts with 1100b (datasheet
// section on the data output format). Anything else means the SPI read was
// misaligned or the part was not in RDATAC, in which case the sample must be
// discarded rather than published as a spike.
static bool readDeviceFrame(DeviceCtx& dev, int32_t channels[TOTAL_ADS_CHANNELS]) {
  // A frame is only valid if no conversion boundary happened while the bytes
  // were being clocked out. Reading across a DRDY reload returns a bit-shifted
  // frame whose status nibble can still look valid, which is what produced
  // single-sample spikes on every channel of that converter.
  for (int attempt = 0; attempt < 2; ++attempt) {
    int32_t frame[8] = {0};
    uint32_t status = 0;
    const uint32_t drdyBefore = dev.drdyCount;

    if (!dev.ads->readDataFrame(frame, status)) {
      return false;
    }

    if ((status >> 20) != 0xC) {
      corruptFrames++;
      continue;
    }

    if (dev.drdyCount != drdyBefore) {
      corruptFrames++;
      continue;
    }

    for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
      channels[dev.channelOffset + ch] = frame[ch];
    }
    return true;
  }

  return false;
}

static void pack24be(int32_t value, uint8_t* out3) {
  uint32_t raw = static_cast<uint32_t>(value) & 0x00FFFFFFUL;
  out3[0] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  out3[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  out3[2] = static_cast<uint8_t>(raw & 0xFF);
}

static void pack16be(int16_t value, uint8_t* out2) {
  const uint16_t raw = static_cast<uint16_t>(value);
  out2[0] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  out2[1] = static_cast<uint8_t>(raw & 0xFF);
}

static void fillAuxPayload(uint8_t packet[33], int16_t auxOut[3]) {
  int16_t localAux[3] = {0, 0, 0};

  // Timeout 0: if the accelerometer task happens to hold the lock, use the last
  // copied values rather than delaying a conversion. The values refresh at
  // 25 Hz against a 500 Hz stream, so one conversion of staleness is invisible.
  static int16_t cachedAux[3] = {0, 0, 0};
  if (auxMutex != nullptr && xSemaphoreTake(auxMutex, 0) == pdTRUE) {
    cachedAux[0] = gAuxData[0];
    cachedAux[1] = gAuxData[1];
    cachedAux[2] = gAuxData[2];
    xSemaphoreGive(auxMutex);
  }
  localAux[0] = cachedAux[0];
  localAux[1] = cachedAux[1];
  localAux[2] = cachedAux[2];

  memcpy(auxOut, localAux, sizeof(localAux));
  pack16be(localAux[0], &packet[26]);
  pack16be(localAux[1], &packet[28]);
  pack16be(localAux[2], &packet[30]);
}

// Drains the radio ring on its own core at the lowest useful priority, so a
// congested Bluetooth link can never delay the conversion path. Only pops what
// the stack actually accepted, and is the sole writer of the SPP API.
static void btFlushTask(void* parameter) {
  (void)parameter;

  uint8_t chunk[BT_FLUSH_CHUNK];
  for (;;) {
    if (!btStackReady || btRing == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    btClientConnected = btSerial.hasClient();
    if (!btClientConnected) {
      // Nobody is listening: drop what piled up so a reconnecting host starts
      // from a clean stream instead of replaying stale samples and old replies.
      portENTER_CRITICAL(&btRingLock);
      btRingHead = 0;
      btRingTail = 0;
      btTextHead = 0;
      btTextTail = 0;
      portEXIT_CRITICAL(&btRingLock);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Text replies first: a "Success:" line must not queue behind 1.5 s of
    // sample packets, or the GUI times the settings write out.
    char textChunk[256];
    const size_t textTaken = btTextRingPeek(textChunk, sizeof(textChunk));
    if (textTaken > 0) {
      size_t textWritten = 0;
      if (btApiMutex != nullptr && xSemaphoreTake(btApiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        textWritten = btSerial.write(reinterpret_cast<const uint8_t*>(textChunk), textTaken);
        xSemaphoreGive(btApiMutex);
      }
      if (textWritten == textTaken) {
        btTextRingAdvance(textTaken);
        btBytesSent += static_cast<uint32_t>(textWritten);
        btLastWriteOkMs = millis();
      } else {
        btWriteFailures++;
      }
      continue;
    }

    // Snapshot and copy under one lock: the buffer can be reallocated by the
    // Q command between two separate critical sections.
    size_t take = 0;
    portENTER_CRITICAL(&btRingLock);
    const size_t availableBytes =
        (btRingHead >= btRingTail) ? (btRingHead - btRingTail) : (btRingSize - btRingTail + btRingHead);
    take = min(availableBytes, sizeof(chunk));
    if (take > 0 && btRing != nullptr) {
      const size_t first = min(take, btRingSize - btRingTail);
      memcpy(chunk, btRing + btRingTail, first);
      if (first < take) {
        memcpy(chunk + first, btRing, take - first);
      }
    }
    portEXIT_CRITICAL(&btRingLock);
    if (take == 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    size_t written = 0;
    if (btApiMutex != nullptr && xSemaphoreTake(btApiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      written = btSerial.write(chunk, take);
      xSemaphoreGive(btApiMutex);
    }
    if (written == take) {
      portENTER_CRITICAL(&btRingLock);
      btRingTail = (btRingTail + take) % btRingSize;
      portEXIT_CRITICAL(&btRingLock);
      btStreamCrc = crc16Update(btStreamCrc, chunk, take);
      btBytesSent += static_cast<uint32_t>(written);
      btLastWriteOkMs = millis();
      // Measured rate, so "is the link keeping up" is a number rather than a guess.
      btRateWindowBytes += static_cast<uint32_t>(written);
      const uint32_t nowMs = millis();
      if (btRateWindowStartMs == 0) {
        btRateWindowStartMs = nowMs;
      } else if (nowMs - btRateWindowStartMs >= 1000) {
        btBytesPerSecond = (btRateWindowBytes * 1000U) / (nowMs - btRateWindowStartMs);
        btRateWindowBytes = 0;
        btRateWindowStartMs = nowMs;
      }
    } else {
      // The SPP queue is full; keep the bytes and retry rather than dropping a
      // packet that the host would see as a gap.
      btWriteFailures++;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

// Watches the SPP connection and repairs it. The failure mode that matters is a
// handle the library still counts as connected but that Windows has already
// closed: its single-client slot then rejects every new connection, which is
// exactly the "pairs once, then never works again" behaviour.
//
// The repair disconnects the dead client and, if the handle does not clear,
// restarts the SPP server. Everything is reported on the USB port, so the state
// is visible with a plain serial monitor even when Bluetooth is the thing that
// is broken.
#if SZD_BT_ENABLED
static void btSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  switch (event) {
    case ESP_SPP_SRV_OPEN_EVT: {
      btOpenEvents++;
      const bool already_open = btLinkOpen;
      btLinkOpen = true;
      if (already_open) {
        // A second client arrived while one was tracked, so the library has just
        // disconnected the new one and kept the stale handle.
        btRejectedOpens++;
        btNeedsRecovery = true;
      }
      btLastWriteOkMs = millis();
      btLogEvent("open handle=%u status=%d%s", static_cast<unsigned>(param->srv_open.handle),
                 static_cast<int>(param->srv_open.status),
                 already_open ? " REJECTED (a stale client was still tracked)" : "");
      break;
    }
    case ESP_SPP_CLOSE_EVT:
      btCloseEvents++;
      btLinkOpen = false;
      if (param->close.async) {
        btRemoteCloses++;
      }
      if (btSerial.hasClient()) {
        // The library kept the handle even though the link closed: it will
        // refuse the next connection unless this is cleaned up.
        btNeedsRecovery = true;
      }
      // remote=1 means the PC closed it, remote=0 means this end did. That
      // distinction is the whole diagnosis: a PC-side drop points at pairing or
      // Windows, a local one at the firmware.
      btLogEvent("close handle=%u remote=%d status=%d stale=%d", static_cast<unsigned>(param->close.handle),
                 param->close.async ? 1 : 0, static_cast<int>(param->close.status),
                 btSerial.hasClient() ? 1 : 0);
      break;
    case ESP_SPP_INIT_EVT:
      btLogEvent("spp init");
      break;
    case ESP_SPP_START_EVT:
      btLogEvent("spp server started");
      break;
    case ESP_SPP_CONG_EVT:
      // Congestion is normal and transient; only report the recovery.
      break;
    default:
      break;
  }
}
#endif  // SZD_BT_ENABLED

static void pollBluetoothLink() {
  const uint32_t nowMs = millis();

  // Stage 3 runs even while the stack is down: it is the step that brings it
  // back, so the "not ready" check must not come first.
  if (btRecoveryStage == 3 && nowMs >= btRecoveryDeadline) {
    btRecoveryStage = 0;
    btNeedsRecovery = false;
    btWriteFailures = 0;
    bringUpBluetooth();
    usbHostPort.println("[BT] bluetooth restarted, pair the PC with SZD_EEG again if it does not reattach");
    return;
  }

  if (!btStackReady) {
    return;
  }

  // Recovery is driven by connection *events* only. A stalled byte count is not
  // evidence of a broken link: a host that stops reading congests the stack
  // without being wrong, and disconnecting it would be worse than waiting.
  if (btNeedsRecovery && btRecoveryStage == 0) {
    btRecoveryStage = 1;
    btRecoveryDeadline = nowMs + 500;
    usbHostPort.printf(
        "[BT] repairing the link (opens=%lu closes=%lu rejected=%lu fail=%lu)\n",
        static_cast<unsigned long>(btOpenEvents), static_cast<unsigned long>(btCloseEvents),
        static_cast<unsigned long>(btRejectedOpens), static_cast<unsigned long>(btWriteFailures));
    return;
  }
  if (btRecoveryStage == 0 || nowMs < btRecoveryDeadline) {
    return;
  }

  // Every step below needs the SPP API, so a busy stack means "try again next
  // loop" rather than "carry on regardless" - restarting under a live write
  // would use freed memory.
  if (btApiMutex == nullptr || xSemaphoreTake(btApiMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }

  if (btRecoveryStage == 1) {
    // Drop the client. If the handle was merely stale this is enough, and the
    // waiting host reconnects on its own.
    btSerial.disconnect();
    xSemaphoreGive(btApiMutex);
    btRecoveryStage = 2;
    btRecoveryDeadline = nowMs + 700;
    return;
  }

  if (btRecoveryStage == 2) {
    if (!btSerial.hasClient()) {
      xSemaphoreGive(btApiMutex);
      btRecoveryStage = 0;
      btNeedsRecovery = false;
      btWriteFailures = 0;
      btLinkOpen = false;
      usbHostPort.println("[BT] connection cleared, waiting for the host to reconnect");
      return;
    }
    // The handle did not clear: only a full stack restart drops it.
    btSerial.disconnect();
    btSerial.end();
    xSemaphoreGive(btApiMutex);
    btStackReady = false;
    btStreamToRadio = false;
    btLinkOpen = false;
    btRecoveryStage = 3;
    btRecoveryDeadline = nowMs + 1500;
    usbHostPort.println("[BT] handle did not clear - restarting the bluetooth stack");
    return;
  }

  xSemaphoreGive(btApiMutex);
}

static void applyBluetoothBufferSize(int kilobytes) {
  if (kilobytes < 2 || static_cast<size_t>(kilobytes) * 1024 > BT_MAX_BUFFER_BYTES) {
    emitFailure("buffer size out of range (2..64 kB)");
    return;
  }
  const size_t bytes = static_cast<size_t>(kilobytes) * 1024;
  uint8_t* fresh = static_cast<uint8_t*>(malloc(bytes));
  if (fresh == nullptr) {
    emitFailure("not enough memory for that buffer size");
    return;
  }
  portENTER_CRITICAL(&btRingLock);
  uint8_t* previous = btRing;
  btRing = fresh;
  btRingSize = bytes;
  btRingHead = 0;
  btRingTail = 0;
  portEXIT_CRITICAL(&btRingLock);
  if (previous != nullptr) {
    free(previous);
  }
  char msg[96];
  snprintf(msg, sizeof(msg), "bluetooth buffer set to %d kB (latency at 500 SPS: %.1f s)", kilobytes,
           static_cast<double>(bytes) / 16500.0);
  emitSuccess(msg);
}

// Brings up classic Bluetooth SPP as a normal discoverable device: the PC pairs
// with SZD_EEG by name and Windows then exposes it as "Standard Serial over
// Bluetooth link", i.e. an ordinary COM port. No MAC address is known in
// advance and nothing is hardcoded.
static void bringUpBluetooth() {
#if SZD_BT_ENABLED
  if (btSerial.begin(SZD_BT_DEVICE_NAME)) {
    btStackReady = true;
    btStreamToRadio = true;
    btLinkOpen = false;
    // 2 ms: long enough that a read does not spin, short enough that a blocking
    // read can never delay the command parser.
    btSerial.setTimeout(2);
    btSerial.register_callback(btSppEvent);

    // Ask the controller for a lower ceiling than its +9 dBm maximum: a bench
    // link does not need the power, and a quieter radio disturbs the ADC less.
    esp_power_level_t level = ESP_PWR_LVL_P3;
#if SZD_BT_TX_POWER_DBM >= 6
    level = ESP_PWR_LVL_P6;
#endif
#if SZD_BT_TX_POWER_DBM >= 9
    level = ESP_PWR_LVL_P9;
#endif
#if SZD_BT_TX_POWER_DBM <= 0
    level = ESP_PWR_LVL_N0;
#endif
    // Applied last: the controller has to be enabled before it accepts a power
    // range, and a rejected setting must never abort the bring-up.
    const esp_err_t power_result = esp_bredr_tx_power_set(level, level);
    Serial.printf("[BT] SPP '%s' ready, tx power request %d dBm (%s), heap %u bytes\n", SZD_BT_DEVICE_NAME,
                  SZD_BT_TX_POWER_DBM, power_result == ESP_OK ? "accepted" : "rejected, using the default",
                  static_cast<unsigned>(ESP.getFreeHeap()));
    btLogEvent("ready txdbm=%d power=%s heap=%u", SZD_BT_TX_POWER_DBM,
               power_result == ESP_OK ? "ok" : "rejected", static_cast<unsigned>(ESP.getFreeHeap()));
  } else {
    btStackReady = false;
    btStreamToRadio = false;
    Serial.printf("[BT] SPP failed to start, running on USB only (heap %u bytes)\n",
                  static_cast<unsigned>(ESP.getFreeHeap()));
  }
#else
  Serial.println("[BT] disabled at compile time (SZD_BT_ENABLED=0)");
#endif
}

// Moves queued diagnostics to the host. They go to the USB port (which is the
// only channel that works while the radio is the problem) and, when a radio host
// is attached, over Bluetooth too, so the live GUI shows them either way.
// The full event history, one line per entry, so the GUI can show what happened
// around a disconnect without a serial monitor.
static void emitBluetoothLog() {
  char line[BT_LOG_LINE + 16];
  uint8_t emitted = 0;
  while (btLogPendingCount() > 0 && emitted < BT_LOG_SLOTS) {
    char entry[BT_LOG_LINE];
    if (!btLogPop(entry, sizeof(entry))) {
      break;
    }
    snprintf(line, sizeof(line), "#BTLOG %s\n", entry);
    const size_t length = strlen(line);
    if (usbHostPort.availableForWrite() >= static_cast<int>(length)) {
      usbHostPort.write(reinterpret_cast<const uint8_t*>(line), length);
    }
    btQueueText(line, length);
    emitted++;
  }
  if (emitted == 0) {
    emitSuccess("no bluetooth events recorded");
  }
}

// Continuous acquisition health. This is the line that separates "the board is
// not producing samples" from "the link is losing them": drdy* are the edges the
// converters actually raised, pkts is what became a packet, and the measured
// per-channel rate is derived from that. Printed every second while streaming.
static void emitAcquisitionHealth() {
  static uint32_t lastMs = 0;
  static uint32_t lastPublished = 0;
  static uint32_t lastDrdy[DEVICE_COUNT] = {0};
  const uint32_t nowMs = millis();
  if (lastMs == 0) {
    lastMs = nowMs;
    lastPublished = publishedConversions;
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      lastDrdy[i] = devices[i].drdyCount;
    }
    return;
  }
  const uint32_t elapsedMs = nowMs - lastMs;
  if (elapsedMs < 1000) {
    return;
  }
  const uint32_t published = publishedConversions;
  const uint32_t periodPackets = published - lastPublished;
  double edges = 0.0;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    edges += static_cast<double>(devices[i].drdyCount - lastDrdy[i]);
    lastDrdy[i] = devices[i].drdyCount;
  }
  lastPublished = published;
  lastMs = nowMs;

  const bool daisy = daisyActive;
  // One packet per conversion in daisy mode carries one converter half, so the
  // per-channel rate is half the packet rate; in 8-channel mode it is 1:1.
  const double packetRate = 1000.0 * static_cast<double>(periodPackets) / static_cast<double>(elapsedMs);
  const double channelRate = daisy ? packetRate / 2.0 : packetRate;
  const double edgesPerSecond = 1000.0 * edges / static_cast<double>(elapsedMs);

  char line[240];
  snprintf(line, sizeof(line),
           "#DX ch_rate=%.1f pkt_rate=%.1f drdy_s=%.1f pkts=%lu miss=%lu late=%lu gap_max_us=%lu corrupt=%lu "
           "spi_us=%lu spi_max_us=%lu usb_free=%d bt_buf=%u core=%d prio=%u\n",
           channelRate, packetRate, edgesPerSecond, static_cast<unsigned long>(published),
           static_cast<unsigned long>(missedConversions), static_cast<unsigned long>(lateConversions),
           static_cast<unsigned long>(worstGapUs), static_cast<unsigned long>(corruptFrames),
           static_cast<unsigned long>(spiReadLastUs), static_cast<unsigned long>(spiReadMaxUs),
           usbHostPort.availableForWrite(), static_cast<unsigned>(btRingUsedBytes()),
           xPortGetCoreID(), static_cast<unsigned>(uxTaskPriorityGet(nullptr)));
  const size_t length = strlen(line);
  if (usbHostPort.availableForWrite() >= static_cast<int>(length)) {
    usbHostPort.write(reinterpret_cast<const uint8_t*>(line), length);
  }
  btQueueText(line, length);
}

static void flushBluetoothDiagnostics() {
  char line[BT_LOG_LINE + 8];
  for (uint8_t burst = 0; burst < 2; ++burst) {
    if (!btLogPop(line, BT_LOG_LINE)) {
      return;
    }
    char framed[BT_LOG_LINE + 12];
    snprintf(framed, sizeof(framed), "[BT] %s\n", line);
    const size_t length = strlen(framed);
    if (usbHostPort.availableForWrite() >= static_cast<int>(length)) {
      usbHostPort.write(reinterpret_cast<const uint8_t*>(framed), length);
    }
    btQueueText(framed, length);
  }
}

static uint32_t uptimeSeconds() {
  return millis() / 1000UL;
}

static void emitBluetoothReport() {
  if (!btStackReady) {
    emitFailure("bluetooth stack is not running");
    return;
  }
  size_t used = 0;
  portENTER_CRITICAL(&btRingLock);
  used = (btRingHead >= btRingTail) ? (btRingHead - btRingTail) : (btRingSize - btRingTail + btRingHead);
  portEXIT_CRITICAL(&btRingLock);
  char msg[256];
  snprintf(msg, sizeof(msg),
           "BT name=%s client=%d link=%s boots=%lu uptime=%lus opens=%lu closes=%lu remote=%lu rejected=%lu "
           "writefail=%lu heap=%u buffer=%u/%u sent=%lu dropped=%lu crc=0x%04X",
           SZD_BT_DEVICE_NAME, btClientConnected ? 1 : 0,
           hostLinkMode == HOST_LINK_BT ? "bt" : (hostLinkMode == HOST_LINK_USB ? "usb" : "both"),
           static_cast<unsigned long>(bootCount), static_cast<unsigned long>(uptimeSeconds()),
           static_cast<unsigned long>(btOpenEvents), static_cast<unsigned long>(btCloseEvents),
           static_cast<unsigned long>(btRemoteCloses), static_cast<unsigned long>(btRejectedOpens),
           static_cast<unsigned long>(btWriteFailures), static_cast<unsigned>(ESP.getFreeHeap()),
           static_cast<unsigned>(used), static_cast<unsigned>(btRingSize),
           static_cast<unsigned long>(btPacketsQueued), static_cast<unsigned long>(btPacketsDropped),
           static_cast<unsigned>(btStreamCrc));
  emitSuccess(msg);
}

// What the live link has to carry at the current settings: one 33-byte packet per
// conversion. Compared with btBytesPerSecond this says whether the radio has
// headroom, independently of any buffer size.
static uint32_t btRequiredBytesPerSecond() {
  const uint32_t rate = sampleRateFromCode(sampleRateCode);
  return rate * 33U;
}

static void emitBluetoothHealth() {
  if (!btStackReady) {
    return;
  }
  size_t used = 0;
  portENTER_CRITICAL(&btRingLock);
  used = (btRingHead >= btRingTail) ? (btRingHead - btRingTail) : (btRingSize - btRingTail + btRingHead);
  portEXIT_CRITICAL(&btRingLock);

  const char* mode_name = (hostLinkMode == HOST_LINK_BT) ? "bt" : (hostLinkMode == HOST_LINK_USB ? "usb" : "both");
  char line[320];
  snprintf(line, sizeof(line),
           "#BT mode=%s client=%d boots=%lu up=%lus opens=%lu closes=%lu remote=%lu rej=%lu fail=%lu heap=%u "
           "name=%s txdbm=%d buf=%u bufsize=%u peak=%u need=%lu rate=%lu sent=%lu dropped=%lu crc=0x%04X\n",
           mode_name, btClientConnected ? 1 : 0, static_cast<unsigned long>(bootCount),
           static_cast<unsigned long>(uptimeSeconds()), static_cast<unsigned long>(btOpenEvents),
           static_cast<unsigned long>(btCloseEvents), static_cast<unsigned long>(btRemoteCloses),
           static_cast<unsigned long>(btRejectedOpens), static_cast<unsigned long>(btWriteFailures),
           static_cast<unsigned>(ESP.getFreeHeap()), SZD_BT_DEVICE_NAME, SZD_BT_TX_POWER_DBM,
           static_cast<unsigned>(used), static_cast<unsigned>(btRingSize),
           static_cast<unsigned>(btRingPeak),
           static_cast<unsigned long>(btRequiredBytesPerSecond()),
           static_cast<unsigned long>(btBytesPerSecond),
           static_cast<unsigned long>(btPacketsQueued), static_cast<unsigned long>(btPacketsDropped),
           static_cast<unsigned>(btStreamCrc));
  btSendText(line);
  if (hostLinkMode != HOST_LINK_BT && usbHostPort.availableForWrite() >= static_cast<int>(strlen(line))) {
    usbHostPort.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
  }
}

// A finger on GPIO32 switches the board to Bluetooth. Polled from loop(), never
// from the acquisition task, because a touch read takes milliseconds.
static void pollTouchKey() {
  const uint16_t raw = static_cast<uint16_t>(touchRead(PIN_TOUCH_BLUETOOTH));
  touchRaw = raw;
  const bool touched = raw > 4 && raw < touchThreshold;
  if (touched) {
    if (touchStableCount < 0xFF) {
      touchStableCount++;
    }
  } else {
    touchStableCount = 0;
  }
  if (touchStableCount >= 3 && !touchLatched) {
    touchLatched = true;
    btForceEnabled = !btForceEnabled;
    HostLink::setMode(btForceEnabled ? HOST_LINK_BT : HOST_LINK_BOTH);
    if (btForceEnabled && !btStackReady) {
      Serial.printf("[BT] radio requested but the stack is not running\n");
    } else {
      Serial.printf("[BT] touch: %s\n", btForceEnabled ? "bluetooth mode" : "auto (both)");
    }
  } else if (touchStableCount == 0) {
    touchLatched = false;
  }
}

static void calibrateTouchKey() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    total += static_cast<uint32_t>(touchRead(PIN_TOUCH_BLUETOOTH));
    delay(4);
  }
  touchBaseline = static_cast<uint16_t>(total / 8);
  touchThreshold = static_cast<uint16_t>(static_cast<float>(touchBaseline) * SZD_BT_TOUCH_TRIGGER_RATIO);
}

static void publishConversion(const int32_t fresh[TOTAL_ADS_CHANNELS], const int32_t previous[TOTAL_ADS_CHANNELS],
                              bool havePrevious, uint32_t sequence) {
  const bool boardSample = (sequence & 1) != 0;
  const uint8_t packetIndex = static_cast<uint8_t>(sequence);

  uint8_t packet[33];
  packet[0] = PACKET_START_BYTE;
  packet[1] = packetIndex;

  uint8_t offset = 2;
  for (uint8_t slot = 0; slot < WIRE_CHANNELS; ++slot) {
    int32_t value = 0;

    if (daisyActive) {
      // Slots 1..6 carry the active converter, slots 7..8 stay zero.
      if (slot < ADS_CHANNELS_PER_CHIP) {
        const uint8_t real = boardSample ? slot : static_cast<uint8_t>(ADS_CHANNELS_PER_CHIP + slot);
        value = havePrevious ? ((previous[real] + fresh[real]) / 2) : fresh[real];
      }
    } else {
      if (slot < TOTAL_ADS_CHANNELS) {
        value = fresh[slot];
      }
    }

    pack24be(value, &packet[offset]);
    offset += 3;
  }

  int16_t currentAux[3] = {0, 0, 0};
  fillAuxPayload(packet, currentAux);
  packet[32] = PACKET_END_STANDARD;
  queueSampleForSd(packetIndex, fresh, currentAux);
  publishedConversions++;

  // Never block the acquisition task on the UART: a stalled host must not cost
  // us a conversion. Dropping a packet is harmless because the GUI maps
  // board/daisy by the sample-counter parity inside the packet.
  if (Serial.availableForWrite() < static_cast<int>(sizeof(packet))) {
    droppedPackets++;
    return;
  }

  Serial.write(packet, sizeof(packet));
}

static void handleRadioCommandByte(uint8_t cmd) {
  radioCommand = cmd;
  if (cmd == 0x01 || cmd == 0x02) {
    awaitingRadioChannelValue = true;
    return;
  }

  switch (cmd) {
    case 0x00:
      emitRadioResponse("Success: Host and Device on Channel Number 1");
      break;
    case 0x07:
      emitRadioResponse("Success: System is Up");
      break;
    default:
      emitRadioResponse("Failure: Unsupported radio command");
      break;
  }
}

static void handleRadioChannelByte(uint8_t channel) {
  awaitingRadioChannelValue = false;

  if (channel == 0 || channel > 25) {
    emitRadioResponse("Failure: Invalid channel");
    return;
  }

  if (radioCommand == 0x01) {
    emitRadioResponse("Success: Channel set");
  } else if (radioCommand == 0x02) {
    emitRadioResponse("Success: Host override");
  } else {
    emitRadioResponse("Failure: Unexpected channel byte");
  }
}

static void handleChannelShortcut(char cmd) {
  int logical = -1;
  bool active = true;

  if (cmd >= '1' && cmd <= '8') {
    logical = cmd - '1';
    active = false;
  } else {
    switch (cmd) {
      case '!': logical = 0; active = true; break;
      case '@': logical = 1; active = true; break;
      case '#': logical = 2; active = true; break;
      case '$': logical = 3; active = true; break;
      case '%': logical = 4; active = true; break;
      case '^': logical = 5; active = true; break;
      case '&': logical = 6; active = true; break;
      case '*': logical = 7; active = true; break;
      default:
        if (daisyActive) {
          switch (cmd) {
            case 'q': logical = 8; active = false; break;
            case 'w': logical = 9; active = false; break;
            case 'e': logical = 10; active = false; break;
            case 'r': logical = 11; active = false; break;
            case 't': logical = 12; active = false; break;
            case 'y': logical = 13; active = false; break;
            case 'u': logical = 14; active = false; break;
            case 'i': logical = 15; active = false; break;
            case 'Q': logical = 8; active = true; break;
            case 'W': logical = 9; active = true; break;
            case 'E': logical = 10; active = true; break;
            case 'R': logical = 11; active = true; break;
            case 'T': logical = 12; active = true; break;
            case 'Y': logical = 13; active = true; break;
            case 'U': logical = 14; active = true; break;
            case 'I': logical = 15; active = true; break;
            default: break;
          }
        }
        break;
    }
  }

  const int real = logicalToReal(logical);
  if (real < 0) {
    return;
  }

  const bool wasStreaming = stopStreaming();
  channelSettings[real].active = active;
  const bool ok = applyChannelToHardware(static_cast<uint8_t>(real));
  if (wasStreaming) {
    startStreaming();
  }

  if (ok) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Channel %d %s", logical + 1, active ? "on" : "off");
    emitSuccess(msg);
  } else {
    emitFailure("channel toggle failed");
  }
}

static bool parseAndApplyXCommand(const char* payload, uint8_t len) {
  if (len < 7) {
    emitFailure("invalid channel command length");
    return false;
  }

  const int logical = channelSelectorToIndex(payload[0]);
  if (logical < 0) {
    emitFailure("invalid channel selector");
    return false;
  }

  const int real = logicalToReal(logical);
  if (real < 0) {
    emitFailure("channel not available");
    return false;
  }

  char digits[6];
  for (uint8_t i = 0; i < 6; ++i) {
    if (!isdigit(static_cast<unsigned char>(payload[i + 1]))) {
      emitFailure("invalid channel command digit");
      return false;
    }
    digits[i] = payload[i + 1];
  }

  ChannelSetting& setting = channelSettings[real];
  setting.active = digits[0] == '0';

  uint8_t gainOrdinal = static_cast<uint8_t>(digits[1] - '0');
  if (gainOrdinal > 6) {
    gainOrdinal = 6;
  }
  setting.gainOrdinal = gainOrdinal;

  uint8_t inputOrdinal = static_cast<uint8_t>(digits[2] - '0');
  if (inputOrdinal > 7) {
    inputOrdinal = 0;
  }
  setting.inputTypeOrdinal = inputOrdinal;

  setting.biasInclude = digits[3] == '1';
  setting.srb2Connect = digits[4] == '1';
  setting.srb1Connect = digits[5] == '1';

  const bool wasStreaming = stopStreaming();
  const bool ok = applyChannelToHardware(static_cast<uint8_t>(real));
  if (wasStreaming) {
    startStreaming();
  }

  if (ok) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Channel set for %d", logical + 1);
    emitSuccess(msg);
    return true;
  }

  emitFailure("channel settings update failed");
  return false;
}

static bool parseAndApplyZCommand(const char* payload, uint8_t len) {
  if (len < 3) {
    emitFailure("invalid impedance command length");
    return false;
  }

  const int logical = channelSelectorToIndex(payload[0]);
  if (logical < 0) {
    emitFailure("invalid impedance channel");
    return false;
  }

  const int real = logicalToReal(logical);
  if (real < 0) {
    emitFailure("channel not available");
    return false;
  }

  leadOffP[real] = payload[1] == '1';
  leadOffN[real] = payload[2] == '1';

  const bool wasStreaming = stopStreaming();
  const bool ok = applyLeadOffToHardware(static_cast<uint8_t>(real));
  if (wasStreaming) {
    startStreaming();
  }

  if (ok) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Lead off set for %d", logical + 1);
    emitSuccess(msg);
    return true;
  }

  emitFailure("lead off update failed");
  return false;
}

static void emitSampleRateResponse() {
  char msg[40];
  snprintf(msg, sizeof(msg), "Sample rate is %uHz", sampleRateFromCode(sampleRateCode));
  emitSuccess(msg);
}

// Reads the digit after an 'N' command and programs the same reference mode on
// both converters. Sent by the recorder after the per-channel writes, so the SRB
// state is always the last thing that changes and can never be left half
// applied. The 'n' command (see handleChipReferenceChars) programs one converter
// at a time, which is what a mixed CM-monitor / SRB1-test setup needs.
static void handleReferenceModeChar(char modeChar) {
  if (modeChar < '0' || modeChar > '2') {
    emitFailure("invalid reference mode (0=diff, 1=SRB1, 2=SRB2)");
    return;
  }

  const uint8_t mode = static_cast<uint8_t>(modeChar - '0');
  char msg[72];
  if (applyReferenceMode(mode)) {
    snprintf(msg, sizeof(msg), "Reference mode %s", referenceModeName(mode));
    emitSuccess(msg);
  } else {
    snprintf(msg, sizeof(msg), "reference mode %s failed", referenceModeName(mode));
    emitFailure(msg);
  }
}

// 'n<chip><mode>' - sets the reference mode of one ADS1299 only. Chip 1 or 2,
// mode 0 (differential) / 1 (SRB1) / 2 (SRB2).
static void handleChipReferenceChars(char chipChar, char modeChar) {
  if (chipChar < '1' || chipChar > '2') {
    emitFailure("invalid converter number for reference mode (1 or 2)");
    return;
  }
  if (modeChar < '0' || modeChar > '2') {
    emitFailure("invalid reference mode (0=diff, 1=SRB1, 2=SRB2)");
    return;
  }

  const size_t deviceIndex = static_cast<size_t>(chipChar - '1');
  const uint8_t mode = static_cast<uint8_t>(modeChar - '0');
  char msg[72];
  if (applyChipReferenceMode(deviceIndex, mode)) {
    snprintf(msg, sizeof(msg), "Reference mode ADS%u %s", static_cast<unsigned>(deviceIndex + 1),
             referenceModeName(mode));
    emitSuccess(msg);
  } else {
    snprintf(msg, sizeof(msg), "reference mode ADS%u %s failed", static_cast<unsigned>(deviceIndex + 1),
             referenceModeName(mode));
    emitFailure(msg);
  }
}

static void handleSampleRateChar(char rateChar) {
  if (rateChar == '~') {
    emitSampleRateResponse();
    return;
  }

  if (rateChar < '0' || rateChar > '6') {
    emitFailure("sample rate out of bounds");
    return;
  }

  const uint8_t newRate = static_cast<uint8_t>(rateChar - '0');
  const uint16_t requestedHz = sampleRateFromCode(newRate);
  // Only the decimated output rate reaches the host, so that is what the serial
  // link has to carry; the converters themselves are limited by SCLK.
  const uint32_t outputRate = static_cast<uint32_t>((requestedHz + decimFactor - 1) / decimFactor);
  if (outputRate > maxSustainedPacketRate()) {
    char msg[128];
    snprintf(msg, sizeof(msg), "rate %uHz /%u = %upkt/s needs %lukbit/s, serial link carries %lukbit/s",
             static_cast<unsigned>(requestedHz), static_cast<unsigned>(decimFactor),
             static_cast<unsigned>(outputRate),
             static_cast<unsigned long>(outputRate * 33UL * 10UL / 1000UL),
             static_cast<unsigned long>(SERIAL_BAUD / 1000UL));
    emitFailure(msg);
    return;
  }
  if (requestedHz > maxSampleRateFromSpi()) {
    char msg[128];
    snprintf(msg, sizeof(msg), "rate %uHz needs %luus per SPI frame, SPI link at %lukHz allows %luSPS",
             static_cast<unsigned>(requestedHz),
             static_cast<unsigned long>(spiFrameTimeUs()),
             static_cast<unsigned long>(adsSpiClockHz / 1000UL),
             static_cast<unsigned long>(maxSampleRateFromSpi()));
    emitFailure(msg);
    return;
  }
  if (setDataRateForAll(newRate)) {
    emitSampleRateResponse();
  } else {
    emitFailure("sample rate update failed");
  }
}

// Applies the block-average factor used before a sample is published. 1..32
// conversions are summed and divided, so the host keeps a low packet rate while
// the converters run fast enough to push the sinc filter corner up.
static bool applyDecimation(uint8_t factor) {
  if (factor < DECIM_MIN || factor > DECIM_MAX) {
    return false;
  }
  decimFactor = factor;
  return true;
}

static void handleDecimationPayload(const char* payload) {
  char* endPtr = nullptr;
  const long value = strtol(payload, &endPtr, 10);
  char msg[80];
  if (endPtr == payload || value < DECIM_MIN || value > DECIM_MAX) {
    snprintf(msg, sizeof(msg), "decimation must be %u..%u", static_cast<unsigned>(DECIM_MIN),
             static_cast<unsigned>(DECIM_MAX));
    emitFailure(msg);
    return;
  }
  if (!applyDecimation(static_cast<uint8_t>(value))) {
    emitFailure("decimation update failed");
    return;
  }
  const uint32_t output = static_cast<uint32_t>(sampleRateFromCode(sampleRateCode)) / decimFactor;
  snprintf(msg, sizeof(msg), "Decimation %u -> %upkt/s", static_cast<unsigned>(decimFactor),
           static_cast<unsigned>(output));
  emitSuccess(msg);
}

// SCLK is shared by both converters, so the new rate has to reach both objects.
// Streaming is stopped first: a rate change in the middle of a frame read is the
// one way to get a torn sample out of the bus.
static bool applySpiClockSetting(uint32_t hz) {
  if (hz < ADS_SPI_CLOCK_MIN_HZ || hz > ADS_SPI_CLOCK_MAX_HZ) {
    return false;
  }
  const bool wasStreaming = stopStreaming();
  adsSpiClockHz = hz;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    devices[i].ads->setSpiClock(hz);
  }
  if (wasStreaming) {
    startStreaming();
  }
  return true;
}

static void handleSpiClockPayload(const char* payload) {
  char* endPtr = nullptr;
  const long kiloHertz = strtol(payload, &endPtr, 10);
  char msg[96];
  if (endPtr == payload || kiloHertz <= 0) {
    emitFailure("SPI clock must be given in kHz, e.g. o4000o");
    return;
  }
  const uint32_t hz = static_cast<uint32_t>(kiloHertz) * 1000UL;
  if (!applySpiClockSetting(hz)) {
    snprintf(msg, sizeof(msg), "SPI clock must be %lu..%lu kHz",
             static_cast<unsigned long>(ADS_SPI_CLOCK_MIN_HZ / 1000UL),
             static_cast<unsigned long>(ADS_SPI_CLOCK_MAX_HZ / 1000UL));
    emitFailure(msg);
    return;
  }
  snprintf(msg, sizeof(msg), "SPI clock %lukHz, frame %luus x%u, limit %luSPS",
           static_cast<unsigned long>(adsSpiClockHz / 1000UL), static_cast<unsigned long>(spiFrameTimeUs()),
           static_cast<unsigned>(DEVICE_COUNT), static_cast<unsigned long>(maxSampleRateFromSpi()));
  emitSuccess(msg);
}

static const char* boardModeName(uint8_t mode) {
  switch (mode) {
    case BOARD_MODE_DEFAULT: return "default";
    case BOARD_MODE_DEBUG: return "debug";
    case BOARD_MODE_ANALOG: return "analog";
    case BOARD_MODE_DIGITAL: return "digital";
    case BOARD_MODE_MARKER: return "marker";
    default: return "unknown";
  }
}

static void handleBoardModeChar(char modeChar) {
  if (modeChar == '/') {
    char msg[32];
    snprintf(msg, sizeof(msg), "%s", boardModeName(boardMode));
    emitSuccess(msg);
    return;
  }

  if (modeChar < '0' || modeChar > '4') {
    emitFailure("invalid board mode value");
    return;
  }

  boardMode = static_cast<uint8_t>(modeChar - '0');
  emitSuccess(boardModeName(boardMode));
}

static void handleSdCommand(char cmd) {
  if (cmd == 'j') {
    const bool wasRecording = sdRecording;
    closeSdFile();
    emitSuccess(wasRecording ? "SD recording stopped" : "No SD recording open");
    return;
  }

  const SdModeSetting* mode = findSdMode(cmd);
  if (mode == nullptr) {
    return;
  }

  openSdFile(mode->durationMs, mode->label);
}

static void handleTestSignalCommand(char cmd) {
  bool ok = false;
  switch (cmd) {
    case '0':
      ok = setInputTypeForAllActiveChannels(1, TESTSIG_AMP_1X, TESTSIG_FREQ_SLOW);
      break;
    case '-':
      ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_1X, TESTSIG_FREQ_SLOW);
      break;
    case '=':
      ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_1X, TESTSIG_FREQ_FAST);
      break;
    case '[':
      ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_SLOW);
      break;
    case ']':
      ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_FAST);
      break;
    case 'p':
      ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_DC);
      break;
    default:
      return;
  }

  if (ok) {
    emitSuccess("Configured internal test signal");
  } else {
    emitFailure("test signal update failed");
  }
}

static void printAllRegisters() {
  stopStreaming();
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#REGHDR ", "Board ADS Registers");
  } else {
    Serial.println();
    Serial.println("Board ADS Registers");
  }
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->printRegisters(Serial, devices[i].name);
      if (!USE_DEBUG_APP_MODE) {
        Serial.println();
      }
    }
  }
  if (USE_DEBUG_APP_MODE) {
    emitDebugLine("#REGHDR ", "LIS3DH Registers");
    char regLine[32];
    snprintf(regLine, sizeof(regLine), "0x0F, 0x%02X", lis3dhWhoAmI);
    emitDebugLine("#REG ", regLine);
    emitDebugLine("#REGEND ", "done");
  } else {
    Serial.println("LIS3DH Registers");
    Serial.print("0x0F, 0x");
    if (lis3dhWhoAmI < 0x10) {
      Serial.print('0');
    }
    Serial.println(lis3dhWhoAmI, HEX);
    Serial.print("$$$");
  }
}

// ---------------------------------------------------------------------------
// Configuration readback ("P")
//
// The GUI writes settings with x/~ commands and only sees a per-command
// acknowledgement. This command clocks every register of both converters back
// out so the GUI can compare the whole chip state (CONFIG1..CONFIG4, each
// CHnSET, the bias masks and MISC1) against what it asked for, instead of
// trusting the write acknowledgements alone.
// ---------------------------------------------------------------------------
static void emitConfigurationReport() {
  const bool wasStreaming = stopStreaming();

  Serial.println("[CFG] BEGIN");

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    uint8_t regs[CONFIG4 + 1] = {0};
    if (dev.linkOk) {
      dev.ads->readRegisters(0x00, CONFIG4 + 1, regs);
    }
    char regsHex[(CONFIG4 + 1) * 2 + 1];
    for (uint8_t address = 0; address <= CONFIG4; ++address) {
      snprintf(&regsHex[address * 2], 3, "%02X", regs[address]);
    }
    regsHex[(CONFIG4 + 1) * 2] = '\0';
    Serial.printf("[CFG] DEV=%s LINK=%d REF=%s REGS=%s\n", dev.name, dev.linkOk ? 1 : 0,
                  referenceModeName(chipReferenceMode[i]), regsHex);
  }

  {
    char referenceText[48];
    formatReferenceSummary(referenceText, sizeof(referenceText));
    Serial.printf("[CFG] CODE=%u SPS=%u DECIM=%u OUT=%u SPICLK=%lu SPILIMIT=%lu SERIALLIMIT=%lu REF=%s DONE=1\n",
                  static_cast<unsigned>(sampleRateCode),
                  static_cast<unsigned>(sampleRateFromCode(sampleRateCode)),
                  static_cast<unsigned>(decimFactor),
                  static_cast<unsigned>(sampleRateFromCode(sampleRateCode) / decimFactor),
                  static_cast<unsigned long>(adsSpiClockHz),
                  static_cast<unsigned long>(maxSampleRateFromSpi()),
                  static_cast<unsigned long>(maxSustainedPacketRate()),
                  referenceText);
  }

  if (wasStreaming) {
    startStreaming();
  }
}

static bool parseUInt(const char* token, uint8_t& out) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }
  char* endPtr = nullptr;
  long value = strtol(token, &endPtr, 10);
  if (endPtr == token || *endPtr != '\0' || value < 0 || value > 255) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

static bool parseBoolToken(const char* token, bool& out) {
  if (token == nullptr) {
    return false;
  }
  if (strcmp(token, "1") == 0 || strcasecmp(token, "on") == 0 || strcasecmp(token, "true") == 0) {
    out = true;
    return true;
  }
  if (strcmp(token, "0") == 0 || strcasecmp(token, "off") == 0 || strcasecmp(token, "false") == 0) {
    out = false;
    return true;
  }
  return false;
}

static void handleDebugLine(const char* line) {
  if (!USE_DEBUG_APP_MODE || line == nullptr || line[0] == '\0') {
    return;
  }

  char buffer[128];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char* context = nullptr;
  char* cmd = strtok_r(buffer, " ", &context);
  if (cmd == nullptr) {
    return;
  }

  if (strcmp(cmd, "ping") == 0) {
    emitDebugLine("#PONG ", "ok");
    return;
  }
  if (strcmp(cmd, "state") == 0) {
    emitDebugState();
    return;
  }
  if (strcmp(cmd, "start") == 0) {
    startStreaming();
    emitDebugLine("#OK ", "start");
    return;
  }
  if (strcmp(cmd, "stop") == 0) {
    stopStreaming();
    emitDebugLine("#OK ", "stop");
    return;
  }
  if (strcmp(cmd, "defaults") == 0) {
    if (applyDefaultChannelSettings()) {
      emitDebugLine("#OK ", "defaults");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "defaults");
    }
    return;
  }
  if (strcmp(cmd, "registers") == 0) {
    printAllRegisters();
    return;
  }
  if (strcmp(cmd, "rate") == 0) {
    uint8_t rateCode = 0;
    if (!parseUInt(strtok_r(nullptr, " ", &context), rateCode) || rateCode > 6) {
      emitDebugLine("#ERR ", "rate");
      return;
    }
    if (setDataRateForAll(rateCode)) {
      emitDebugLine("#OK ", "rate");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "rate");
    }
    return;
  }
  if (strcmp(cmd, "boardmode") == 0) {
    uint8_t mode = 0;
    if (!parseUInt(strtok_r(nullptr, " ", &context), mode) || mode > 4) {
      emitDebugLine("#ERR ", "boardmode");
      return;
    }
    boardMode = mode;
    emitDebugLine("#OK ", "boardmode");
    emitDebugState();
    return;
  }
  if (strcmp(cmd, "channel") == 0) {
    uint8_t channel = 0;
    bool active = false;
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > GUI_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), active)) {
      emitDebugLine("#ERR ", "channel");
      return;
    }
    const int real = logicalToReal(channel - 1);
    if (real < 0) {
      emitDebugLine("#ERR ", "channel");
      return;
    }
    const bool wasStreaming = stopStreaming();
    channelSettings[real].active = active;
    const bool ok = applyChannelToHardware(static_cast<uint8_t>(real));
    if (wasStreaming) {
      startStreaming();
    }
    if (ok) {
      emitDebugLine("#OK ", "channel");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "channel");
    }
    return;
  }
  if (strcmp(cmd, "config") == 0) {
    uint8_t channel = 0;
    uint8_t gain = 0;
    uint8_t input = 0;
    bool active = false;
    bool bias = false;
    bool srb2 = false;
    bool srb1 = false;
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > GUI_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), active) ||
      !parseUInt(strtok_r(nullptr, " ", &context), gain) || gain > 6 ||
      !parseUInt(strtok_r(nullptr, " ", &context), input) || input > 7 ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), bias) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), srb2) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), srb1)) {
      emitDebugLine("#ERR ", "config");
      return;
    }

    const int real = logicalToReal(channel - 1);
    if (real < 0) {
      emitDebugLine("#ERR ", "config");
      return;
    }

    ChannelSetting& setting = channelSettings[real];
    setting.active = active;
    setting.gainOrdinal = gain;
    setting.inputTypeOrdinal = input;
    setting.biasInclude = bias;
    setting.srb2Connect = srb2;
    setting.srb1Connect = srb1;

    const bool wasStreaming = stopStreaming();
    const bool ok = applyChannelToHardware(static_cast<uint8_t>(real));
    if (wasStreaming) {
      startStreaming();
    }

    if (ok) {
      emitDebugLine("#OK ", "config");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "config");
    }
    return;
  }
  if (strcmp(cmd, "imp") == 0) {
    uint8_t channel = 0;
    bool pEnable = false;
    bool nEnable = false;
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > GUI_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), pEnable) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), nEnable)) {
      emitDebugLine("#ERR ", "imp");
      return;
    }
    const int real = logicalToReal(channel - 1);
    if (real < 0) {
      emitDebugLine("#ERR ", "imp");
      return;
    }
    leadOffP[real] = pEnable;
    leadOffN[real] = nEnable;
    const bool wasStreaming = stopStreaming();
    const bool ok = applyLeadOffToHardware(static_cast<uint8_t>(real));
    if (wasStreaming) {
      startStreaming();
    }
    if (ok) {
      emitDebugLine("#OK ", "imp");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "imp");
    }
    return;
  }
  if (strcmp(cmd, "test") == 0) {
    const char* name = strtok_r(nullptr, " ", &context);
    bool ok = false;
    if (name != nullptr) {
      if (strcmp(name, "normal") == 0) {
        ok = applyDefaultChannelSettings();
      } else if (strcmp(name, "ground") == 0) {
        ok = setInputTypeForAllActiveChannels(1, TESTSIG_AMP_1X, TESTSIG_FREQ_SLOW);
      } else if (strcmp(name, "pulse1slow") == 0) {
        ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_1X, TESTSIG_FREQ_SLOW);
      } else if (strcmp(name, "pulse1fast") == 0) {
        ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_1X, TESTSIG_FREQ_FAST);
      } else if (strcmp(name, "pulse2slow") == 0) {
        ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_SLOW);
      } else if (strcmp(name, "pulse2fast") == 0) {
        ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_FAST);
      } else if (strcmp(name, "dc") == 0) {
        ok = setInputTypeForAllActiveChannels(5, TESTSIG_AMP_2X, TESTSIG_FREQ_DC);
      }
    }
    if (ok) {
      emitDebugLine("#OK ", "test");
      emitDebugState();
    } else {
      emitDebugLine("#ERR ", "test");
    }
    return;
  }

  emitDebugLine("#ERR ", "unknown");
}

static void handleCommandChar(char cmd) {
  // The SD name is free text up to a ';' terminator, so it is consumed before any
  // single-character command could interpret its letters as channel selectors.
  if (parsingSdName) {
    if (cmd == ';') {
      sdNamePayload[sdNameLen] = '\0';
      parsingSdName = false;
      if (sdNameLen > 0) {
        strncpy(sdSessionName, sdNamePayload, sizeof(sdSessionName) - 1);
        sdSessionName[sizeof(sdSessionName) - 1] = '\0';
        char msg[96];
        snprintf(msg, sizeof(msg), "SD session name: %s", sdSessionName);
        emitSuccess(msg);
      } else {
        emitFailure("empty SD session name");
      }
      sdNameLen = 0;
      return;
    }
    if (sdNameLen + 1 < static_cast<int>(sizeof(sdNamePayload))) {
      sdNamePayload[sdNameLen++] = cmd;
    }
    return;
  }

  if (awaitingSdCommand) {
    awaitingSdCommand = false;
    switch (cmd) {
      case 's':
        emitSdStatus("status");
        break;
      case 'b':
        startSdRecordingFromBoard("host");
        break;
      case 'e':
        stopSdRecordingFromBoard("host");
        break;
      case 'p':
        toggleSdRecordingFromButton();
        break;
      case 'w':
        saveSdConfiguration();
        break;
      case 'r':
        loadSdConfiguration(true);
        break;
      case 'n':
        parsingSdName = true;
        sdNameLen = 0;
        break;
      default:
        emitFailure("unknown SD command (s,b,e,p,w,r,n)");
        break;
    }
    return;
  }

  if (awaitingSampleRate) {
    awaitingSampleRate = false;
    handleSampleRateChar(cmd);
    return;
  }

  if (awaitingBoardMode) {
    awaitingBoardMode = false;
    handleBoardModeChar(cmd);
    return;
  }

  if (awaitingMarker) {
    awaitingMarker = false;
    // Written into the card file's Marker column on the next sample, so the two
    // recordings share one unambiguous instant.
    sdMarkerPending = true;
    emitSuccess("marker accepted");
    return;
  }

  if (awaitingReferenceMode) {
    awaitingReferenceMode = false;
    handleReferenceModeChar(cmd);
    return;
  }

  if (awaitingChipReferenceChip) {
    awaitingChipReferenceChip = false;
    chipReferenceChip = cmd;
    awaitingChipReferenceMode = true;
    return;
  }

  if (awaitingChipReferenceMode) {
    awaitingChipReferenceMode = false;
    handleChipReferenceChars(chipReferenceChip, cmd);
    return;
  }

  if (parsingDecimation) {
    if (cmd == 'm') {
      decimationPayload[decimationLen] = '\0';
      handleDecimationPayload(decimationPayload);
      parsingDecimation = false;
      decimationLen = 0;
      return;
    }
    if (cmd >= '0' && cmd <= '9' && decimationLen + 1 < static_cast<int>(sizeof(decimationPayload))) {
      decimationPayload[decimationLen++] = cmd;
      return;
    }
    parsingDecimation = false;
    decimationLen = 0;
    emitFailure("decimation must be digits, e.g. m8m");
    return;
  }

  if (parsingSpiClock) {
    if (cmd == 'o') {
      spiClockPayload[spiClockLen] = '\0';
      handleSpiClockPayload(spiClockPayload);
      parsingSpiClock = false;
      spiClockLen = 0;
      return;
    }
    if (cmd >= '0' && cmd <= '9' && spiClockLen + 1 < static_cast<int>(sizeof(spiClockPayload))) {
      spiClockPayload[spiClockLen++] = cmd;
      return;
    }
    parsingSpiClock = false;
    spiClockLen = 0;
    emitFailure("SPI clock must be kHz digits, e.g. o4000o");
    return;
  }

  if (parsingX) {
    if (cmd == 'X') {
      parseAndApplyXCommand(xPayload, xLen);
      parsingX = false;
      xLen = 0;
      return;
    }

    if (xLen < sizeof(xPayload)) {
      xPayload[xLen++] = cmd;
    } else {
      parsingX = false;
      xLen = 0;
      emitFailure("channel command too long");
    }
    return;
  }

  if (parsingBufferSize) {
    if (cmd == 'Q') {
      bufferSizePayload[bufferSizeLen] = '\0';
      applyBluetoothBufferSize(atoi(bufferSizePayload));
      parsingBufferSize = false;
      bufferSizeLen = 0;
      return;
    }
    if (cmd >= '0' && cmd <= '9' && bufferSizeLen + 1 < static_cast<int>(sizeof(bufferSizePayload))) {
      bufferSizePayload[bufferSizeLen++] = cmd;
      return;
    }
    parsingBufferSize = false;
    bufferSizeLen = 0;
    emitFailure("buffer size must be kilobytes, e.g. Q24Q");
    return;
  }

  if (parsingZ) {
    if (cmd == 'Z') {
      parseAndApplyZCommand(zPayload, zLen);
      parsingZ = false;
      zLen = 0;
      return;
    }

    if (zLen < sizeof(zPayload)) {
      zPayload[zLen++] = cmd;
    } else {
      parsingZ = false;
      zLen = 0;
      emitFailure("impedance command too long");
    }
    return;
  }

  switch (cmd) {
    case 'b':
      suppressAutoStream = false;
      startStreaming();
      if (USE_DEBUG_APP_MODE) {
        emitDebugLine("#OK ", "b");
      }
      break;
    case ':':
      awaitingSdCommand = true;
      break;
    case 's':
      suppressAutoStream = true;
      if (stopStreaming()) {
        char msg[112];
        snprintf(msg, sizeof(msg), "Stream stopped (missed=%lu dropped=%lu corrupt=%lu)",
                 static_cast<unsigned long>(missedConversions), static_cast<unsigned long>(droppedPackets),
                 static_cast<unsigned long>(corruptFrames));
        emitSuccess(msg);
      } else {
        emitSuccess("Stream stopped");
      }
      break;
    case 'v':
      suppressAutoStream = true;
      stopStreaming();
      applyDefaultChannelSettings();
      printVersionInfo();
      break;
    case 'V':
      emitVersionString();
      break;
    case 'd':
      if (applyDefaultChannelSettings()) {
        emitSuccess("updating channel settings to default");
      } else {
        emitFailure("failed to set defaults");
      }
      break;
    case 'D':
      emitDefaultSettingsReport();
      break;
    case 'c':
      if (daisyActive) {
        daisyActive = false;
        if (!streaming) {
          sampleCounter = 0;
        }
        emitRawResponse("daisy removed");
      }
      break;
    case 'C': {
      if (DAISY_MODE) {
        const bool wasActive = daisyActive;
        daisyActive = true;
        if (!streaming) {
          sampleCounter = 0;
        }
        emitRawResponse(wasActive ? "16" : "daisy attached16");
      } else {
        emitRawResponse("no daisy to attach!8");
      }
      break;
    }
    case '?':
      printAllRegisters();
      break;
    case 'P':
      emitConfigurationReport();
      break;
    case '>':
      // '>' rather than a letter: every letter is already a Cyton channel
      // selector, and reusing one would silently toggle a daisy channel.
      repairAndVerify("host");
      break;
    case '~':
      awaitingSampleRate = true;
      break;
    case '/':
      awaitingBoardMode = true;
      break;
    case '`':
      awaitingMarker = true;
      break;
    case 'N':
      awaitingReferenceMode = true;
      break;
    case 'n':
      awaitingChipReferenceChip = true;
      break;
    case 'm':
      parsingDecimation = true;
      decimationLen = 0;
      break;
    case 'o':
      parsingSpiClock = true;
      spiClockLen = 0;
      break;
    case 'Q':
      parsingBufferSize = true;
      bufferSizeLen = 0;
      break;
    case 'T':
      emitBluetoothReport();
      break;
    case 'B':
      emitBluetoothLog();
      break;
    case 'x':
      parsingX = true;
      xLen = 0;
      break;
    case 'z':
      parsingZ = true;
      zLen = 0;
      break;
    case '0':
    case '-':
    case '=':
    case '[':
    case ']':
    case 'p':
      handleTestSignalCommand(cmd);
      break;
    case 'A':
    case 'S':
    case 'F':
    case 'G':
    case 'H':
    case 'J':
    case 'K':
    case 'L':
    case 'j':
      handleSdCommand(cmd);
      break;
    default:
      handleChannelShortcut(cmd);
      break;
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    uint8_t raw = static_cast<uint8_t>(Serial.read());

    if (USE_DEBUG_APP_MODE) {
      if (parsingDebugLine) {
        if (raw == '\n' || raw == '\r') {
          debugLine[debugLineLen] = '\0';
          parsingDebugLine = false;
          debugLineLen = 0;
          handleDebugLine(debugLine + 1);
          continue;
        }

        if (debugLineLen + 1 < sizeof(debugLine)) {
          debugLine[debugLineLen++] = static_cast<char>(raw);
          continue;
        }

        parsingDebugLine = false;
        debugLineLen = 0;
        emitDebugLine("#ERR ", "line-too-long");
        continue;
      }

      if (raw == ':') {
        parsingDebugLine = true;
        debugLineLen = 0;
        debugLine[debugLineLen++] = ':';
        continue;
      }
    }

    if (awaitingRadioCommand) {
      awaitingRadioCommand = false;
      handleRadioCommandByte(raw);
      continue;
    }

    if (awaitingRadioChannelValue) {
      handleRadioChannelByte(raw);
      continue;
    }

    if (raw == 0xF0) {
      awaitingRadioCommand = true;
      continue;
    }

    char cmd = static_cast<char>(raw);
    handleCommandChar(cmd);
  }

  // A host that is actively talking to us gets a data stream. The record button
  // can start an SD recording on its own, and a late-joining GUI then needs the
  // stream back without having to guess that it should send 'b'.
  ensureStreamingForHost();
}

static void IRAM_ATTR adsDrdyIsr0() {
  devices[0].drdyPending = true;
  devices[0].drdyCount++;
  BaseType_t woken = pdFALSE;
  if (conversionSem != nullptr) {
    xSemaphoreGiveFromISR(conversionSem, &woken);
  }
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void IRAM_ATTR adsDrdyIsr1() {
  devices[1].drdyPending = true;
  devices[1].drdyCount++;
  BaseType_t woken = pdFALSE;
  if (conversionSem != nullptr) {
    xSemaphoreGiveFromISR(conversionSem, &woken);
  }
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void streamingTask(void* parameter) {
  (void)parameter;

  int32_t previous[TOTAL_ADS_CHANNELS] = {0};
  int32_t decimSum[TOTAL_ADS_CHANNELS] = {0};
  uint8_t decimCount = 0;
  bool havePrevious = false;
  bool wasStreaming = false;
  // Time of the previous conversion. Reset when a stream starts: otherwise the
  // first gap of a new session is the whole time the stream was stopped (an
  // "18 s gap" that never happened), which poisons the late/gap statistics the
  // window displays and hides a real stall behind a meaningless number.
  uint32_t previousUs = 0;

  for (;;) {
    if (!streaming) {
      wasStreaming = false;
      havePrevious = false;
      previousUs = 0;
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!wasStreaming) {
      // Consume stale edges left over from the previous session.
      while (xSemaphoreTake(conversionSem, 0) == pdTRUE) {
      }
      havePrevious = false;
      previousUs = 0;
      memset(decimSum, 0, sizeof(decimSum));
      decimCount = 0;
      wasStreaming = true;
    }
    // Block until the next conversion. This yields the CPU, so the idle task
    // and the task watchdog keep running.
    const uint32_t wokeUs = micros();
    if (xSemaphoreTake(conversionSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      missedConversions++;
      continue;
    }

    // How long this conversion waited. Anything over 1.5 periods means another
    // task or an interrupt kept the acquisition task off the CPU.
    static uint32_t lastGapReportMs = 0;
    if (previousUs != 0) {
      const uint32_t gapUs = wokeUs - previousUs;
      const uint32_t periodUs = static_cast<uint32_t>(1000000.0 / sampleRateFromCode(sampleRateCode));
      if (gapUs > (periodUs * 3) / 2) {
        lateConversions++;
      }
      if (gapUs > worstGapUs) {
        worstGapUs = gapUs;
      }
      // Log a gap the moment it happens, with the counters that can explain it.
      // A visible step in the traces with no [GAP] line is not a timing problem -
      // it is the analog side - and knowing which of the two it is decides what
      // to investigate next. Rate limited, because a sustained stall would
      // otherwise flood the link.
      const uint32_t nowMs = millis();
      if (gapUs > (periodUs * 5) / 2 && (lastGapReportMs == 0 || nowMs - lastGapReportMs >= 100)) {
        lastGapReportMs = nowMs;
        char line[192];
        snprintf(line, sizeof(line),
                 "[GAP] MS=%lu GAP_US=%lu PERIOD_US=%lu LATE=%lu MISSED=%lu REC=%d QUEUED=%u "
                 "SPI_US=%lu HEAP=%u\n",
                 static_cast<unsigned long>(nowMs), static_cast<unsigned long>(gapUs),
                 static_cast<unsigned long>(periodUs),
                 static_cast<unsigned long>(lateConversions),
                 static_cast<unsigned long>(missedConversions), sdRecording ? 1 : 0,
                 static_cast<unsigned>(sdQueue != nullptr ? uxQueueMessagesWaiting(sdQueue) : 0),
                 static_cast<unsigned long>(spiReadLastUs), static_cast<unsigned>(ESP.getFreeHeap()));
        usbHostPort.print(line);
      }
    }
    previousUs = wokeUs;

    // Seed with the previous conversion so a failed read holds the last good
    // sample instead of publishing zeros or a spike.
    int32_t fresh[TOTAL_ADS_CHANNELS];
    if (havePrevious) {
      memcpy(fresh, previous, sizeof(fresh));
    } else {
      memset(fresh, 0, sizeof(fresh));
    }

    bool anyLinked = false;
    bool readFailed = false;
    const uint32_t readStartUs = micros();
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (!devices[i].linkOk) {
        continue;
      }
      anyLinked = true;
      if (readDeviceFrame(devices[i], fresh)) {
        devices[i].drdyPending = false;
        devices[i].failedReads = 0;
      } else {
        readFailed = true;
        // Holding the last value forever hides a dead converter: the packet
        // counter keeps advancing, so the host sees a healthy stream made of a
        // frozen sample. After a few misses publish zeros instead, which the
        // recorder shows as "no data" and the analyzer sees as a flat line.
        if (devices[i].failedReads < 0xFF) {
          devices[i].failedReads++;
        }
        if (devices[i].failedReads > STALE_FRAME_LIMIT) {
          for (uint8_t ch = 0; ch < devices[i].channelCount; ++ch) {
            fresh[devices[i].channelOffset + ch] = 0;
          }
        }
      }
    }

    const uint32_t readUs = micros() - readStartUs;
    spiReadLastUs = readUs;
    if (readUs > spiReadMaxUs) {
      spiReadMaxUs = readUs;
    }

    // Both converters raise one edge per conversion. Discard any second edge
    // that landed while we were reading, so one conversion produces exactly
    // one published sample.
    while (xSemaphoreTake(conversionSem, 0) == pdTRUE) {
    }

    if (!anyLinked) {
      missedConversions++;
      continue;
    }
    if (readFailed) {
      missedConversions++;
      // A held or zeroed sample must not enter the average: one bad frame would
      // otherwise bias the whole block.
      continue;
    }

    // Block-average before publishing. With decimFactor == 1 this publishes every
    // conversion unchanged. With a larger factor the extra conversions the faster
    // data rate produces are averaged back down, so the host packet rate stays
    // where the serial/BLE link is comfortable while the ADS1299 sinc3 corner
    // (-3 dB at 0.262 x data rate) sits well above the measurement band.
    if (decimCount >= decimFactor) {
      // The factor changed under us; throw away the partial block.
      memset(decimSum, 0, sizeof(decimSum));
      decimCount = 0;
    }
    for (uint8_t ch = 0; ch < TOTAL_ADS_CHANNELS; ++ch) {
      decimSum[ch] += fresh[ch];
    }
    decimCount++;
    if (decimCount < decimFactor) {
      continue;
    }

    int32_t averaged[TOTAL_ADS_CHANNELS];
    for (uint8_t ch = 0; ch < TOTAL_ADS_CHANNELS; ++ch) {
      averaged[ch] = decimSum[ch] / static_cast<int32_t>(decimCount);
    }
    memset(decimSum, 0, sizeof(decimSum));
    decimCount = 0;

    publishConversion(averaged, previous, havePrevious, sampleCounter);
    sampleCounter++;
    memcpy(previous, averaged, sizeof(previous));
    havePrevious = true;
  }
}

static void sdWriterTask(void* parameter) {
  (void)parameter;

  for (;;) {
    SdSample sample;
    // 200 ms rather than 20: when no recording is running this task has nothing to
    // do, and waking 50 times a second only to find an empty queue and an empty
    // buffer is 50 pointless scheduler entries on the same core as the radio.
    // When a recording is active the queue is never empty for long, and the tail of
    // a recording still reaches the card within 200 ms of the last sample.
    if (xQueueReceive(sdQueue, &sample, pdMS_TO_TICKS(200)) == pdTRUE) {
      writeSampleToSd(sample);
    } else {
      sdFlushWriteBuffer();
    }
  }
}

static void lis3dhTask(void* parameter) {
  (void)parameter;

  const TickType_t period = pdMS_TO_TICKS(40); // 25 Hz, same cadence as stock Cyton
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    int16_t local[3] = {0, 0, 0};
    readLis3dhAuxData(local);

    if (auxMutex != nullptr && xSemaphoreTake(auxMutex, portMAX_DELAY) == pdTRUE) {
      gAuxData[0] = local[0];
      gAuxData[1] = local[1];
      gAuxData[2] = local[2];
      xSemaphoreGive(auxMutex);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

static void bringUpDevices() {
  pinMode(PIN_ADS1_CS, OUTPUT);
  pinMode(PIN_ADS2_CS, OUTPUT);
  pinMode(PIN_LIS3DH_CS, OUTPUT);
  pinMode(PIN_SD_CS, OUTPUT);

  digitalWrite(PIN_ADS1_CS, HIGH);
  digitalWrite(PIN_ADS2_CS, HIGH);
  digitalWrite(PIN_LIS3DH_CS, HIGH);
  digitalWrite(PIN_SD_CS, HIGH);

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    // START/RESET/PWDN are shared between the two ADS1299 devices, so only the
    // first device is allowed to pulse the power/reset lines during bring-up.
    const bool pulseControlPins = (i == 0);

    dev.ads->setClockOutput(dev.clockMaster);
    dev.linkOk = dev.ads->begin(
        dev.drdyPin,
        dev.csPin,
        PIN_ADS_START,
        PIN_ADS_RESET,
        PIN_ADS_PWDN,
        PIN_SPI_SCLK,
        PIN_SPI_MISO,
        PIN_SPI_MOSI,
        adsSpiClockHz,
        ADS_CHANNELS_PER_CHIP,
        pulseControlPins);

    // SPIClass::begin() reconfigures its own SS pin (ADS1's CS) during the
    // first call, so make sure both chip selects are outputs and released.
    pinMode(PIN_ADS1_CS, OUTPUT);
    pinMode(PIN_ADS2_CS, OUTPUT);
    digitalWrite(PIN_ADS1_CS, HIGH);
    digitalWrite(PIN_ADS2_CS, HIGH);

    Serial.printf("[%s] bring-up %s (ID read 0x%02X)\n", dev.name, dev.linkOk ? "OK" : "NO RESPONSE",
                  dev.ads->readRegister(0x00));

    // The clock slave has CLKSEL tied low and cannot answer on SPI until the
    // master's internal oscillator is actually driven out on the CLK pin. The
    // master is at index 0, so publish the clock before the slave is probed.
    if (dev.clockMaster && dev.linkOk) {
      const bool clkOk = dev.ads->enableClockOutput();
      Serial.printf("[%s] clock output %s (CONFIG1=0x%02X)\n", dev.name, clkOk ? "enabled" : "FAILED",
                    dev.ads->readRegister(CONFIG1));
      delay(10);
    }
  }

  // Anything that did not answer may simply have been probed before the clock
  // reached it; retry now that it is running.
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    if (dev.linkOk) {
      continue;
    }
    for (uint8_t attempt = 0; attempt < 5 && !dev.linkOk; ++attempt) {
      delay(20);
      dev.linkOk = dev.ads->detect();
    }
    Serial.printf("[%s] re-probe %s\n", dev.name, dev.linkOk ? "OK" : "STILL NO RESPONSE");
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    if (!dev.linkOk) {
      continue;
    }
    dev.deviceId = dev.ads->readRegister(0x00);
    attachInterrupt(digitalPinToInterrupt(dev.drdyPin), (i == 0) ? adsDrdyIsr0 : adsDrdyIsr1, FALLING);
  }

  bringUpLis3dh();
  bringUpSdCard();
  beginRecordButton();
}

// ---------------------------------------------------------------------------
// Power-on self test (POST)
//
// Runs once at boot, before the GUI connects, and prints a pass/fail line for
// every digital and analog check so the serial monitor alone tells you whether
// the board is healthy.
// ---------------------------------------------------------------------------

struct PostStats {
  uint8_t channels;
  double meanUv[8];
  double rmsUv[8];
  double ppUv[8];
  uint32_t railed;
  uint32_t samples;
};

static uint32_t postFailures = 0;
static uint32_t postChecks = 0;

static void postResult(const char* tag, const char* label, bool ok, const char* detail) {
  postChecks++;
  if (!ok) {
    postFailures++;
  }
  Serial.printf("[%s] %-26s %s%s%s\n", tag, label, ok ? "OK" : "FAIL",
                (detail != nullptr && detail[0] != '\0') ? " : " : "",
                (detail != nullptr) ? detail : "");
}

static uint8_t expectedDeviceId() {
  switch (ADS_CHANNELS_PER_CHIP) {
    case 4: return 0x3C;  // NU_CH = 00
    case 6: return 0x3D;  // NU_CH = 01
    default: return 0x3E; // NU_CH = 10
  }
}

static double countsToMicrovolts(int32_t counts) {
  return static_cast<double>(counts) * (4.5 / 24.0 / 8388607.0) * 1e6;
}

static bool postReadSample(DeviceCtx& dev, int32_t out[TOTAL_ADS_CHANNELS]) {
  if (!waitForDeviceDrdy(dev, drdyTimeoutMs())) {
    return false;
  }
  if (!readDeviceFrame(dev, out)) {
    return false;
  }
  dev.drdyPending = false;
  return true;
}

// Collects `count` samples from one device and computes input-referred
// statistics for every channel of that device.
static bool postCollectAnalog(DeviceCtx& dev, uint16_t count, PostStats& out) {
  out.channels = dev.channelCount;
  out.railed = 0;
  out.samples = 0;

  double sum[8] = {0};
  double sumSq[8] = {0};
  int32_t minRaw[8];
  int32_t maxRaw[8];
  for (uint8_t ch = 0; ch < 8; ++ch) {
    sum[ch] = 0;
    sumSq[ch] = 0;
    minRaw[ch] = INT32_MAX;
    maxRaw[ch] = INT32_MIN;
    out.meanUv[ch] = 0;
    out.rmsUv[ch] = 0;
    out.ppUv[ch] = 0;
  }

  // Drop the first samples: the input mux change needs a conversion to settle.
  int32_t discard[TOTAL_ADS_CHANNELS];
  for (uint8_t i = 0; i < 3; ++i) {
    if (!postReadSample(dev, discard)) {
      return false;
    }
  }

  int32_t frame[TOTAL_ADS_CHANNELS];
  for (uint16_t i = 0; i < count; ++i) {
    if (!postReadSample(dev, frame)) {
      return false;
    }

    for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
      const int32_t raw = frame[dev.channelOffset + ch];
      const double uv = countsToMicrovolts(raw);

      sum[ch] += uv;
      sumSq[ch] += uv * uv;
      if (raw < minRaw[ch]) minRaw[ch] = raw;
      if (raw > maxRaw[ch]) maxRaw[ch] = raw;
      if (abs(raw) > 8200000) {
        out.railed++;
      }
    }
    out.samples++;
  }

  if (out.samples == 0) {
    return false;
  }

  for (uint8_t ch = 0; ch < dev.channelCount; ++ch) {
    const double n = static_cast<double>(out.samples);
    out.meanUv[ch] = sum[ch] / n;
    const double meanSq = sumSq[ch] / n;
    const double variance = (meanSq > out.meanUv[ch] * out.meanUv[ch])
                                ? (meanSq - out.meanUv[ch] * out.meanUv[ch])
                                : 0.0;
    out.rmsUv[ch] = sqrt(variance);
    out.ppUv[ch] = countsToMicrovolts(maxRaw[ch]) - countsToMicrovolts(minRaw[ch]);
  }

  return true;
}

// Puts one device into a known test configuration: every channel powered up,
// gain 24, SRB2 connected, and the requested input mux on all channels.
// The internal test signal must be armed *after* the defaults are written,
// because applyCytonDefaults() clears CONFIG2.
static void postPrepareDevice(DeviceCtx& dev, uint8_t mux, bool internalTestSignal) {
  dev.ads->stopContinuousConversion();
  dev.ads->applyCytonDefaults(sampleRateCode);
  if (internalTestSignal) {
    dev.ads->configureInternalTestSignal(TESTSIG_AMP_1X, TESTSIG_FREQ_SLOW);
  }
  dev.ads->setInputTypeForAllChannels(mux);
  dev.drdyPending = false;
  dev.ads->startContinuousConversion();
  delay(2);
}

static bool postDeviceDigital(DeviceCtx& dev, const char* tag) {
  char detail[96];
  bool ok = true;

  // ID register: DEV_ID must be '11' and NU_CH must match the fitted part.
  const uint8_t id = dev.ads->readRegister(0x00);
  dev.deviceId = id;
  snprintf(detail, sizeof(detail), "id=0x%02X (expected 0x%02X)", id, expectedDeviceId());
  const bool idOk = ((id & 0x1F) == (expectedDeviceId() & 0x1F));
  postResult(tag, "SPI + device ID", idOk, detail);
  ok = ok && idOk;

  // CONFIG1.CLK_EN must match the role of this device.
  const bool clkOk = dev.ads->verifyClockOutput(dev.clockMaster);
  const uint8_t config1 = dev.ads->readRegister(CONFIG1);
  snprintf(detail, sizeof(detail), "CONFIG1=0x%02X, CLK_EN=%d, expected=%d", config1,
           (config1 & 0x20) ? 1 : 0, dev.clockMaster ? 1 : 0);
  postResult(tag, "CLK_EN (clock master)", clkOk, detail);
  ok = ok && clkOk;

  // A shared clock is what keeps the two devices conversion-synchronous, so if
  // this device is not the master its DRDY must be paced by the master's CLK.
  return ok;
}

static bool postDeviceClock(DeviceCtx& dev, const char* tag) {
  const uint16_t expected = sampleRateFromCode(sampleRateCode);

  dev.drdyCount = 0;
  dev.drdyPending = false;
  dev.ads->startContinuousConversion();

  const uint32_t windowMs = 500;
  delay(windowMs);
  const uint32_t counted = dev.drdyCount;

  const uint32_t expectedCount = static_cast<uint32_t>(expected) * windowMs / 1000UL;
  const uint32_t tolerance = expectedCount / 10UL; // +/-10 %
  const bool ok = (counted + tolerance >= expectedCount) && (counted <= expectedCount + tolerance);

  char detail[96];
  snprintf(detail, sizeof(detail), "DRDY %lu Hz (expected %u Hz)", static_cast<unsigned long>(counted * 1000UL / windowMs),
           expected);
  postResult(tag, "DRDY / clock present", ok, detail);

  dev.ads->stopContinuousConversion();
  dev.drdyPending = false;
  return ok;
}

// Reads one real data frame and checks the 24-bit status word. The top nibble
// is always 1100b, so a mismatch means the SPI read is misaligned or the device
// is not in RDATAC.
static bool postDeviceFrame(DeviceCtx& dev, const char* tag) {
  dev.ads->applyCytonDefaults(sampleRateCode);
  dev.drdyPending = false;
  dev.ads->startContinuousConversion();

  int32_t frame[8] = {0};
  uint32_t status = 0;
  const bool gotDrdy = waitForDeviceDrdy(dev, drdyTimeoutMs());
  const bool gotFrame = gotDrdy && dev.ads->readDataFrame(frame, status);
  dev.drdyPending = false;
  dev.ads->stopContinuousConversion();

  char detail[128];
  if (!gotFrame) {
    postResult(tag, "RDATAC frame", false, gotDrdy ? "SPI read failed" : "no DRDY edge");
    return false;
  }

  const bool aligned = ((status >> 20) == 0xC);
  snprintf(detail, sizeof(detail), "status=0x%06lX ch1=%ld (status nibble must be 0xC)",
           static_cast<unsigned long>(status & 0xFFFFFFUL), static_cast<long>(frame[0]));
  postResult(tag, "RDATAC frame", aligned, detail);
  return aligned;
}

static bool postDeviceShorted(DeviceCtx& dev, const char* tag) {
  // MUX 001 shorts both inputs internally: the output should sit near zero with
  // only the converter's own noise.
  postPrepareDevice(dev, 1, false);

  PostStats stats;
  const bool collected = postCollectAnalog(dev, 250, stats);
  dev.ads->stopContinuousConversion();

  if (!collected) {
    postResult(tag, "AFE shorted input", false, "no DRDY while sampling");
    return false;
  }

  double worstMean = 0;
  double worstRms = 0;
  for (uint8_t ch = 0; ch < stats.channels; ++ch) {
    if (fabs(stats.meanUv[ch]) > fabs(worstMean)) {
      worstMean = stats.meanUv[ch];
    }
    if (stats.rmsUv[ch] > worstRms) {
      worstRms = stats.rmsUv[ch];
    }
  }

  const double railedPct = 100.0 * static_cast<double>(stats.railed) /
                           (static_cast<double>(stats.samples) * stats.channels);

  char detail[128];
  snprintf(detail, sizeof(detail), "worst mean=%.2fuV rms=%.2fuV railed=%.1f%%", worstMean, worstRms, railedPct);

  const bool ok = (railedPct < 5.0) && (fabs(worstMean) < 500.0) && (worstRms < 20.0);
  postResult(tag, "AFE shorted input", ok, detail);

  if (!ok) {
    for (uint8_t ch = 0; ch < stats.channels; ++ch) {
      Serial.printf("       ch%u shorted: mean=%.2fuV rms=%.2fuV\n", static_cast<unsigned>(ch + 1),
                    stats.meanUv[ch], stats.rmsUv[ch]);
    }
  }
  return ok;
}

static bool postDeviceTestSignal(DeviceCtx& dev, const char* tag) {
  // Internal square wave, 1x amplitude = (VREFP - VREFN) / 2400 = 1.875 mV,
  // paced at fCLK / 2^21 (~0.98 Hz). A healthy front end reproduces it.
  postPrepareDevice(dev, 5, true);

  PostStats stats;
  const bool collected = postCollectAnalog(dev, 300, stats);
  dev.ads->stopContinuousConversion();

  if (!collected) {
    postResult(tag, "AFE internal test signal", false, "no DRDY while sampling");
    return false;
  }

  double minAmp = 1e9;
  double maxAmp = -1e9;
  for (uint8_t ch = 0; ch < stats.channels; ++ch) {
    const double amp = stats.ppUv[ch] / 2.0;
    if (amp < minAmp) minAmp = amp;
    if (amp > maxAmp) maxAmp = amp;
  }

  const double railedPct = 100.0 * static_cast<double>(stats.railed) /
                           (static_cast<double>(stats.samples) * stats.channels);

  char detail[128];
  snprintf(detail, sizeof(detail), "amp %.0f..%.0fuV (expect ~1875uV) railed=%.1f%%", minAmp, maxAmp, railedPct);

  const bool ok = (railedPct < 5.0) && (minAmp > 1500.0) && (maxAmp < 2300.0);
  postResult(tag, "AFE internal test signal", ok, detail);

  if (!ok) {
    for (uint8_t ch = 0; ch < stats.channels; ++ch) {
      Serial.printf("       ch%u test signal: amp=%.0fuV\n", static_cast<unsigned>(ch + 1),
                    stats.ppUv[ch] / 2.0);
    }
  }
  return ok;
}

static bool postLis3dh() {
  bool ok = true;
  char detail[128];

  if (!lis3dhPresent) {
    snprintf(detail, sizeof(detail), "no response at 0x18/0x19 (WHO_AM_I=0x%02X)", lis3dhWhoAmI);
    postResult("AUX", "LIS3DH present", false, detail);
    return false;
  }

  snprintf(detail, sizeof(detail), "address=0x%02X WHO_AM_I=0x%02X", lis3dhAddress, lis3dhWhoAmI);
  postResult("AUX", "LIS3DH present", true, detail);

  // Sample the accelerometer and require at least one axis to show a plausible
  // magnitude; a dead or disconnected part reads all zeros.
  int16_t last[3] = {0, 0, 0};
  int16_t peak[3] = {0, 0, 0};
  bool readAny = false;
  for (uint8_t i = 0; i < 10; ++i) {
    int16_t sample[3] = {0, 0, 0};
    if (readLis3dhAuxData(sample)) {
      readAny = true;
      for (uint8_t axis = 0; axis < 3; ++axis) {
        last[axis] = sample[axis];
        if (abs(sample[axis]) > abs(peak[axis])) {
          peak[axis] = sample[axis];
        }
      }
    }
    delay(5);
  }

  if (!readAny) {
    postResult("AUX", "LIS3DH data read", false, "I2C transfer failed");
    return false;
  }

  snprintf(detail, sizeof(detail), "X=%d Y=%d Z=%d (4g range, ~500 LSB per g)", last[0], last[1], last[2]);
  const int32_t magnitude = abs(peak[0]) + abs(peak[1]) + abs(peak[2]);
  const bool dataOk = magnitude > 30 && magnitude < 12000;
  postResult("AUX", "LIS3DH data read", dataOk, detail);

  ok = ok && dataOk;
  return ok;
}

static bool postSdCard() {
  char detail[96];
  if (!sdPresent) {
    postResult("SD", "SD card present", false, "not detected - recording disabled");
    return false;
  }

  uint8_t cardType = SD.cardType();
  snprintf(detail, sizeof(detail), "type=%u size=%lluMB", static_cast<unsigned>(cardType),
           static_cast<unsigned long long>(SD.cardSize() / (1024ULL * 1024ULL)));
  postResult("SD", "SD card present", true, detail);
  return true;
}

// Raw view of the SPI bus: pin levels, the MISO level when nothing is
// selected, and the exact bytes each device clocks out for an RREG. This runs
// even when a device failed bring-up, because it is what tells you *why*.
static void postSpiBusProbe() {
  Serial.println("--- SPI bus probe ---");
  Serial.printf("bus       : SCLK=%d MISO=%d MOSI=%d @ 1 MHz, mode 1 (CPOL=0 CPHA=1)\n", PIN_SPI_SCLK,
                PIN_SPI_MISO, PIN_SPI_MOSI);
  Serial.printf("CS pins   : ADS1=%d ADS2=%d (must idle HIGH)\n", PIN_ADS1_CS, PIN_ADS2_CS);
  Serial.printf("levels    : CS1=%d CS2=%d DRDY1=%d DRDY2=%d RESET=%d PWDN=%d START=%d\n",
                digitalRead(PIN_ADS1_CS), digitalRead(PIN_ADS2_CS), digitalRead(PIN_ADS1_DRDY),
                digitalRead(PIN_ADS2_DRDY), digitalRead(PIN_ADS_RESET), digitalRead(PIN_ADS_PWDN),
                digitalRead(PIN_ADS_START));

  const uint8_t idleMiso = devices[0].ads->probeIdleMiso();
  Serial.printf("idle MISO : 0x%02X (no CS asserted) %s\n", idleMiso,
                idleMiso == 0xFF ? "- floating/pulled high"
                                 : (idleMiso == 0x00 ? "- low (floating, or last bit was 0)"
                                                     : "- partial drive"));

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    uint8_t trace[3] = {0};

    // Make sure the part is not in RDATAC where register reads are ignored.
    dev.ads->SDATAC();
    delay(1);
    dev.ads->rawReadRegister(0x00, trace);
    dev.deviceId = trace[2];

    Serial.printf("[%s] RREG(0x00) raw: opcode=0x%02X count=0x%02X data=0x%02X -> ID=0x%02X\n", dev.name,
                  trace[0], trace[1], trace[2], trace[2]);

    if (trace[2] == 0xFF) {
      Serial.printf("       MISO stuck high - device not driving (no power, wrong CS, or CLK missing)\n");
    } else if (trace[2] == 0x00) {
      Serial.printf("       MISO reads 0x00 - device not driving / line shorted low\n");
    }

    uint8_t regs[24] = {0};
    dev.ads->readRegisters(0x00, 0x18, regs);
    Serial.printf("[%s] regs 00-17:", dev.name);
    for (uint8_t r = 0; r < 0x18; ++r) {
      Serial.printf(" %02X", regs[r]);
    }
    Serial.println();
  }
}

// Proves the two chip selects really address two different devices: writes
// scratch patterns into the LOFF_FLIP register of ADS1 (unused by this
// firmware, and R/W in hardware) and checks that ADS2 does not see them.
static void postCsIsolation() {
  if (!devices[0].linkOk || !devices[1].linkOk) {
    postResult("SPI", "CS isolates devices", false, "skipped - a device is not responding");
    return;
  }

  const uint8_t patternA = 0xA5;
  const uint8_t patternB = 0x5A;

  devices[0].ads->writeRegister(LOFF_FLIP, patternA);
  delay(1);
  const uint8_t readA1 = devices[0].ads->readRegister(LOFF_FLIP);
  const uint8_t readB1 = devices[1].ads->readRegister(LOFF_FLIP);

  devices[0].ads->writeRegister(LOFF_FLIP, patternB);
  delay(1);
  const uint8_t readA2 = devices[0].ads->readRegister(LOFF_FLIP);
  const uint8_t readB2 = devices[1].ads->readRegister(LOFF_FLIP);

  devices[0].ads->writeRegister(LOFF_FLIP, 0x00);
  devices[1].ads->writeRegister(LOFF_FLIP, 0x00);

  char detail[160];
  snprintf(detail, sizeof(detail), "ADS1=0x%02X/0x%02X (want 0xA5/0x5A), ADS2=0x%02X/0x%02X", readA1, readA2,
           readB1, readB2);

  const bool ads1Writable = (readA1 == patternA) && (readA2 == patternB);
  const bool ads2Untouched = (readB1 != patternA) && (readB2 != patternB);
  const bool ok = ads1Writable && ads2Untouched;
  postResult("SPI", "CS isolates devices", ok, detail);

  if (!ads1Writable) {
    Serial.println("       ADS1 did not store the pattern - it is not answering on SPI");
  } else if (!ads2Untouched) {
    Serial.println("       ADS2 sees ADS1's register - CS1/CS2 are shorted or stuck low");
  }
}

static void runPowerOnSelfTest() {
  Serial.println();
  Serial.println("=== OpenBCI ESP32 power-on self test ===");
  Serial.printf("Hardware    : ADS1299-%u x2 = %u channels (%s)\n",
                static_cast<unsigned>(ADS_CHANNELS_PER_CHIP),
                static_cast<unsigned>(TOTAL_ADS_CHANNELS),
                DAISY_MODE ? "Cyton+Daisy 16ch emulation" : "Cyton 8ch");
  Serial.printf("Sample rate : %u Hz\n", sampleRateFromCode(sampleRateCode));

  postSpiBusProbe();
  postCsIsolation();

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    DeviceCtx& dev = devices[i];
    const char* tag = dev.name;

    if (!dev.linkOk) {
      postResult(tag, "SPI + device ID", false, "device did not respond at bring-up");
      continue;
    }

    postDeviceDigital(dev, tag);
    postDeviceClock(dev, tag);
    postDeviceFrame(dev, tag);
    postDeviceShorted(dev, tag);
    postDeviceTestSignal(dev, tag);
  }

  postLis3dh();
  postSdCard();

  // Return both devices to clean defaults with conversions stopped; the GUI
  // will issue its own configuration once it connects.
  applyDefaultChannelSettings();

  Serial.printf("=== POST %s (%lu checks, %lu failed) ===\n",
                postFailures == 0 ? "PASSED" : "FAILED", static_cast<unsigned long>(postChecks),
                static_cast<unsigned long>(postFailures));
  Serial.println();
}

void setup() {
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  // Printed before anything else: if Bluetooth "disappears", the first question
  // is whether the board rebooted, and the reason answers it. bootCount lives in
  // RTC memory, so a value that keeps climbing after a Bluetooth disconnect
  // proves the board is restarting rather than the link merely failing.
  bootCount++;
  Serial.printf("\n[BOOT] reset reason=%d (%s), boots=%lu, heap %u bytes\n", static_cast<int>(esp_reset_reason()),
                resetReasonName(esp_reset_reason()), static_cast<unsigned long>(bootCount),
                static_cast<unsigned>(ESP.getFreeHeap()));
  btLogEvent("boot=%lu reset=%d(%s)", static_cast<unsigned long>(bootCount), static_cast<int>(esp_reset_reason()),
             resetReasonName(esp_reset_reason()));

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  xTaskCreate(ledBlinkTask, "ledBlink", 1024, nullptr, tskIDLE_PRIORITY, nullptr);

  // The radio ring is the largest single allocation in the firmware, so it shrinks
  // rather than failing: a smaller ring only means less tolerance for a host that
  // stalls, while a failed allocation would mean no Bluetooth stream at all.
  static const size_t kBtRingSizes[] = {SZD_BT_TX_BUFFER_BYTES, 24576, 16384, 8192};
  for (size_t candidate : kBtRingSizes) {
    if (candidate > SZD_BT_TX_BUFFER_BYTES) {
      continue;
    }
    btRing = static_cast<uint8_t*>(malloc(candidate));
    if (btRing != nullptr) {
      btRingSize = candidate;
      if (candidate != SZD_BT_TX_BUFFER_BYTES) {
        usbHostPort.printf("[BT] only %u B of transmit buffer available (wanted %u B)\n",
                           static_cast<unsigned>(candidate),
                           static_cast<unsigned>(SZD_BT_TX_BUFFER_BYTES));
      }
      break;
    }
  }
  if (btRing == nullptr) {
    usbHostPort.println("[BT] WARNING: no transmit buffer - the stream will only go over USB");
  }
  calibrateTouchKey();

  sdQueue = xQueueCreate(SD_QUEUE_LENGTH, sizeof(SdSample));
  if (sdQueue == nullptr) {
    // Without the queue the card cannot record at all, so say so instead of
    // silently writing header-only files.
    usbHostPort.println("[SD] WARNING: could not allocate the sample queue - SD recording is disabled");
  }
  auxMutex = xSemaphoreCreateMutex();
  btApiMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  conversionSem = xSemaphoreCreateBinary();

  bringUpDevices();
  applyDefaultChannelState();
  applyDefaultChannelSettings();

  delay(100);
  runPowerOnSelfTest();

  // Started after the converters so the acquisition hardware is ready first: the
  // radio stack takes about a second and must not hold up the ADS1299 bring-up.
  bringUpBluetooth();

  // Acquisition alone on core 1; everything that can block lives on core 0.
  xTaskCreatePinnedToCore(streamingTask, "adsStream", 4096, nullptr, STREAMING_TASK_PRIO, nullptr, ACQUISITION_CORE);
  xTaskCreatePinnedToCore(sdWriterTask, "sdWriter", 4096, nullptr, SD_TASK_PRIO, nullptr, RADIO_CORE);
  // No record-button task: the press is caught by a GPIO interrupt, so it does not
  // depend on any task getting scheduled - which is why a polled version could be
  // missed entirely while the radio or the card was busy.
  xTaskCreatePinnedToCore(lis3dhTask, "lis3dh", 2048, nullptr, AUX_TASK_PRIO, nullptr, RADIO_CORE);
  // Bluetooth runs on its own core at a priority far below the acquisition
  // task, so a congested radio can slow the stream but never stall a conversion.
  // 5 kB: the Bluedroid write path plus its logging is deeper than it looks, and
  // a stack overflow here would look exactly like the board rebooting.
  xTaskCreatePinnedToCore(btFlushTask, "btFlush", 5120, nullptr, BT_FLUSH_TASK_PRIO, nullptr, BT_FLUSH_TASK_CORE);

  if (USE_DEBUG_APP_MODE) {
    emitDebugHello();
    emitDebugState();
  } else {
    printVersionInfo();
  }
}

void loop() {
  pollSerialCommands();
  if (recordButtonPressed) {
    recordButtonPressed = false;
    toggleSdRecordingFromButton();
  }
  updateSdRecordingTimeout();
  // The touch pad needs an ADC1 conversion, and the measurement owns the front
  // end: it is only polled when nothing is being acquired, so no analog activity
  // of the firmware's own can land inside a capture.
  static uint32_t lastTouchMs = 0;
  if (!streaming && millis() - lastTouchMs >= 50) {
    lastTouchMs = millis();
    pollTouchKey();
  }
  pollBluetoothLink();
  flushBluetoothDiagnostics();
  if (streaming) {
    emitAcquisitionHealth();
  }

  static uint32_t lastHealthMs = 0;
  const uint32_t nowMs = millis();
  // A live radio host gets the counters twice a second; on a cable-only link the
  // same line is only worth a rare glance, so it does not fill the monitor.
  const uint32_t healthPeriod = (btModeActive || btHasClient()) ? BT_HEALTH_PERIOD_MS : 10 * BT_HEALTH_PERIOD_MS;
  if (btStackReady && nowMs - lastHealthMs >= healthPeriod) {
    lastHealthMs = nowMs;
    emitBluetoothHealth();
  }
  delay(1);
}
