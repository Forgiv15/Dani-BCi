#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "ADS1299.h"

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
static constexpr int PIN_SD_MISO = 12;
static constexpr int PIN_SD_SCLK = 14;

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

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint32_t SERIAL_TX_BUFFER = 4096;
static constexpr bool USE_DEBUG_APP_MODE = false;
static constexpr uint32_t LIS3DH_I2C_CLOCK_HZ = 400000;
static constexpr uint32_t SD_SPI_CLOCK_HZ = 4000000;
// ADS1299 SCLK. The datasheet allows up to 4 MHz for reads, but everything on
// the shared bus is byte-banged through the ESP32 SPI peripheral, so 2 MHz
// halves the time spent inside a data frame compared to the old 1 MHz setting.
static constexpr uint32_t ADS_SPI_CLOCK_HZ = 2000000;

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

// Reference ("SRB") mode, applied to both converters at once.
//
// SRB1 is driven by MISC1 bit 5, which is one switch for the entire chip: it
// ties the negative input of *every* channel to the SRB1 pin. It therefore
// cannot be a per-channel setting, and a per-channel request that flipped it
// would silently move every other channel's reference. SRB2 is the
// per-channel CHnSET bit 3, but the recorder exposes it as one board-wide
// choice as well, so exactly one of these three states is always programmed.
static constexpr uint8_t REFERENCE_MODE_DIFFERENTIAL = 0;  // INP/INN from electrodes
static constexpr uint8_t REFERENCE_MODE_SRB1 = 1;          // all INN -> SRB1 pin
static constexpr uint8_t REFERENCE_MODE_SRB2 = 2;          // all INN -> SRB2 pin
static uint8_t referenceMode = REFERENCE_MODE_SRB2;

static const char* referenceModeName(uint8_t mode) {
  switch (mode) {
    case REFERENCE_MODE_DIFFERENTIAL: return "differential";
    case REFERENCE_MODE_SRB1: return "SRB1";
    case REFERENCE_MODE_SRB2: return "SRB2";
    default: return "unknown";
  }
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
static volatile uint32_t droppedPackets = 0;
static volatile uint32_t corruptFrames = 0;
static bool sdPresent = false;
static volatile bool sdRecording = false;
static File sdFile;
static char sdFileName[24] = {0};
static uint32_t sdRecordStartMs = 0;
static uint32_t sdRecordDurationMs = 0;
static SemaphoreHandle_t sdMutex = nullptr;
static QueueHandle_t sdQueue = nullptr;

static constexpr int SD_QUEUE_LENGTH = 64;
static constexpr UBaseType_t STREAMING_TASK_PRIO = 20;
static constexpr UBaseType_t SD_TASK_PRIO = 5;
static constexpr UBaseType_t AUX_TASK_PRIO = 3;

static void ledBlinkTask(void* parameter) {
  (void)parameter;
  bool ledOn = false;

  for (;;) {
    ledOn = !ledOn;
    digitalWrite(PIN_STATUS_LED, ledOn ? HIGH : LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
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
static bool stopStreaming();
static void startStreaming();
static void streamingTask(void* parameter);
static void sdWriterTask(void* parameter);
static void lis3dhTask(void* parameter);
static void IRAM_ATTR adsDrdyIsr0();
static void IRAM_ATTR adsDrdyIsr1();

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
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  sdSpi.begin(PIN_SD_SCLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdPresent = SD.begin(PIN_SD_CS, sdSpi, SD_SPI_CLOCK_HZ);
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
};

static void writeSampleToSd(const SdSample& sample) {
  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  if (!sdRecording || !sdFile) {
    xSemaphoreGive(sdMutex);
    return;
  }

  char sampleHex[3];
  snprintf(sampleHex, sizeof(sampleHex), "%02X", static_cast<unsigned>(sample.sampleIndex));
  sdFile.print(sampleHex);

  for (uint8_t ch = 0; ch < TOTAL_ADS_CHANNELS; ++ch) {
    sdFile.print(',');
    writeHex24(sdFile, sample.channels[ch]);
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    sdFile.print(',');
    writeHex16(sdFile, sample.aux[axis]);
  }

  sdFile.print('\n');
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

  // Non-blocking: if the SD writer can't keep up, drop the sample. The
  // streaming path must never stall behind the SD card.
  xQueueSend(sdQueue, &sample, 0);
}

static bool openSdFile(uint32_t durationMs, const char* label) {
  if (!sdPresent) {
    emitFailure("SD card not available");
    return false;
  }

  closeSdFile();

  if (!makeNextSdFileName(sdFileName, sizeof(sdFileName))) {
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

  if (sdFile) {
    sdFile.flush();
    sdFile.close();
  }

  sdRecording = false;
  sdRecordStartMs = 0;
  sdRecordDurationMs = 0;

  if (sdMutex != nullptr) {
    xSemaphoreGive(sdMutex);
  }
}

static void updateSdRecordingTimeout() {
  if (!sdRecording || sdRecordDurationMs == 0) {
    return;
  }

  if ((millis() - sdRecordStartMs) >= sdRecordDurationMs) {
    char finishedFile[24];
    strncpy(finishedFile, sdFileName, sizeof(finishedFile) - 1);
    finishedFile[sizeof(finishedFile) - 1] = '\0';
    closeSdFile();

    char msg[80];
    snprintf(msg, sizeof(msg), "SD recording finished: %s", finishedFile[0] != '\0' ? finishedFile : "file closed");
    emitSuccess(msg);
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
  return (totalBits * 1000000UL + ADS_SPI_CLOCK_HZ - 1UL) / ADS_SPI_CLOCK_HZ;
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
  Serial.print("Serial link limit: ");
  Serial.print(maxSustainedPacketRate());
  Serial.println(" SPS");
  Serial.print("SPI clock: ");
  Serial.print(ADS_SPI_CLOCK_HZ / 1000UL);
  Serial.print(" kHz, frame ");
  Serial.print(spiFrameTimeUs());
  Serial.print(" us x ");
  Serial.print(static_cast<unsigned>(DEVICE_COUNT));
  Serial.print(" devices, SPI link limit: ");
  Serial.print(maxSampleRateFromSpi());
  Serial.println(" SPS");
  Serial.print("Reference mode: ");
  Serial.println(referenceModeName(referenceMode));
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

  DeviceCtx& dev = devices[channelIndex / ADS_CHANNELS_PER_CHIP];
  if (!dev.linkOk) {
    return false;
  }

  const uint8_t localIndex = channelIndex % ADS_CHANNELS_PER_CHIP;
  const ChannelSetting& setting = channelSettings[channelIndex];
  // SRB2 is only ever programmed when the board is in SRB2 mode, and SRB1
  // (chip-wide) is never requested from here: the reference mode owns both, so
  // a stray per-channel digit cannot leave the inputs floating.
  const bool srb2 = (referenceMode == REFERENCE_MODE_SRB2) && setting.active;
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

// Programs the reference routing for every converter and remembers it, so the
// next channel write cannot fall back to a different mode.
static bool applyReferenceToDevice(DeviceCtx& dev) {
  if (!dev.linkOk) {
    return true;
  }
  bool ok = dev.ads->setSrb1Routing(referenceMode == REFERENCE_MODE_SRB1);
  ok = dev.ads->setSrb2ForAllChannels(referenceMode == REFERENCE_MODE_SRB2) && ok;
  return ok;
}

static bool applyReferenceMode(uint8_t mode) {
  if (mode > REFERENCE_MODE_SRB2) {
    return false;
  }

  const bool wasStreaming = stopStreaming();
  referenceMode = mode;

  bool ok = true;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    ok = applyReferenceToDevice(devices[i]) && ok;
  }

  if (wasStreaming) {
    startStreaming();
  }
  return ok;
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
    // SRB2 mode is the default because it is what a referential montage with
    // one reference electrode on the SRB2 pin needs; the recorder overrides
    // this with an 'N' command as soon as it applies its saved settings.
    channelSettings[i].srb2Connect = true;
    channelSettings[i].srb1Connect = false;
    leadOffP[i] = false;
    leadOffN[i] = false;
  }
  referenceMode = REFERENCE_MODE_SRB2;
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
    ok = applyReferenceToDevice(devices[i]) && ok;
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
    applyReferenceToDevice(dev);
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

  if (auxMutex != nullptr && xSemaphoreTake(auxMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    localAux[0] = gAuxData[0];
    localAux[1] = gAuxData[1];
    localAux[2] = gAuxData[2];
    xSemaphoreGive(auxMutex);
  }

  memcpy(auxOut, localAux, sizeof(localAux));
  pack16be(localAux[0], &packet[26]);
  pack16be(localAux[1], &packet[28]);
  pack16be(localAux[2], &packet[30]);
}

// Publishes one conversion. Both ADS1299s are sampled on every DRDY edge, so
// `fresh` always holds up-to-date data for all real channels. In 16-channel
// daisy emulation only one converter's channels fit in a packet, so packets
// alternate and each sent value is the average of the last two conversions of
// that converter - the same scheme the stock Cyton+Daisy firmware uses.
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

// Reads the digit after an 'N' command and programs the board-wide reference
// mode. Sent by the recorder after the per-channel writes, so the SRB state is
// always the last thing that changes and can never be left half applied.
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
  if (requestedHz > maxUsableSampleRate()) {
    char msg[128];
    if (maxSampleRateFromSpi() < maxSustainedPacketRate()) {
      snprintf(msg, sizeof(msg), "rate %uHz needs %luus per SPI frame, SPI link at %lukHz allows %luSPS",
               static_cast<unsigned>(requestedHz),
               static_cast<unsigned long>(spiFrameTimeUs()),
               static_cast<unsigned long>(ADS_SPI_CLOCK_HZ / 1000UL),
               static_cast<unsigned long>(maxSampleRateFromSpi()));
    } else {
      snprintf(msg, sizeof(msg), "rate %uHz needs %lukbit/s, serial link carries %lukbit/s",
               static_cast<unsigned>(requestedHz),
               static_cast<unsigned long>(requestedHz * 33UL * 10UL / 1000UL),
               static_cast<unsigned long>(SERIAL_BAUD / 1000UL));
    }
    emitFailure(msg);
    return;
  }
  if (setDataRateForAll(newRate)) {
    emitSampleRateResponse();
  } else {
    emitFailure("sample rate update failed");
  }
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
    Serial.printf("[CFG] DEV=%s LINK=%d REGS=%s\n", dev.name, dev.linkOk ? 1 : 0, regsHex);
  }

  Serial.printf("[CFG] CODE=%u SPS=%u SPICLK=%lu SPILIMIT=%lu SERIALLIMIT=%lu REF=%s DONE=1\n",
                static_cast<unsigned>(sampleRateCode),
                static_cast<unsigned>(sampleRateFromCode(sampleRateCode)),
                static_cast<unsigned long>(ADS_SPI_CLOCK_HZ),
                static_cast<unsigned long>(maxSampleRateFromSpi()),
                static_cast<unsigned long>(maxSustainedPacketRate()),
                referenceModeName(referenceMode));

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
    emitSuccess("marker accepted");
    return;
  }

  if (awaitingReferenceMode) {
    awaitingReferenceMode = false;
    handleReferenceModeChar(cmd);
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
      startStreaming();
      if (USE_DEBUG_APP_MODE) {
        emitDebugLine("#OK ", "b");
      }
      break;
    case 's':
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
  bool havePrevious = false;
  bool wasStreaming = false;

  for (;;) {
    if (!streaming) {
      wasStreaming = false;
      havePrevious = false;
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!wasStreaming) {
      // Consume stale edges left over from the previous session.
      while (xSemaphoreTake(conversionSem, 0) == pdTRUE) {
      }
      havePrevious = false;
      wasStreaming = true;
    }

    // Block until the next conversion. This yields the CPU, so the idle task
    // and the task watchdog keep running.
    if (xSemaphoreTake(conversionSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      missedConversions++;
      continue;
    }

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
    }

    publishConversion(fresh, previous, havePrevious, sampleCounter);
    sampleCounter++;
    memcpy(previous, fresh, sizeof(previous));
    havePrevious = true;
  }
}

static void sdWriterTask(void* parameter) {
  (void)parameter;

  for (;;) {
    SdSample sample;
    if (xQueueReceive(sdQueue, &sample, portMAX_DELAY) == pdTRUE) {
      writeSampleToSd(sample);
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
        ADS_SPI_CLOCK_HZ,
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

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  xTaskCreate(ledBlinkTask, "ledBlink", 1024, nullptr, tskIDLE_PRIORITY, nullptr);

  sdQueue = xQueueCreate(SD_QUEUE_LENGTH, sizeof(SdSample));
  auxMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  conversionSem = xSemaphoreCreateBinary();

  bringUpDevices();
  applyDefaultChannelState();
  applyDefaultChannelSettings();

  delay(100);
  runPowerOnSelfTest();

  xTaskCreatePinnedToCore(streamingTask, "adsStream", 4096, nullptr, STREAMING_TASK_PRIO, nullptr, 0);
  xTaskCreatePinnedToCore(sdWriterTask, "sdWriter", 4096, nullptr, SD_TASK_PRIO, nullptr, 1);
  xTaskCreatePinnedToCore(lis3dhTask, "lis3dh", 2048, nullptr, AUX_TASK_PRIO, nullptr, 1);

  if (USE_DEBUG_APP_MODE) {
    emitDebugHello();
    emitDebugState();
  } else {
    printVersionInfo();
  }
}

void loop() {
  pollSerialCommands();
  updateSdRecordingTimeout();
  delay(1);
}
