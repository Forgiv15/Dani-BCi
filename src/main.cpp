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

static constexpr uint8_t ADS_CHANNELS_PER_CHIP = 4;
static constexpr uint8_t TOTAL_CHANNELS = 8;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t DRDY_TIMEOUT_MS = 8;
static constexpr bool USE_DEBUG_APP_MODE = false;
static constexpr uint32_t LIS3DH_I2C_CLOCK_HZ = 400000;
static constexpr uint32_t SD_SPI_CLOCK_HZ = 4000000;

static constexpr uint8_t PACKET_START_BYTE = 0xA0;
static constexpr uint8_t PACKET_END_STANDARD = 0xC0;

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
  bool linkOk;
};

static DeviceCtx devices[] = {
    {&ads1, "ADS1", PIN_ADS1_DRDY, PIN_ADS1_CS, 0, false},
    {&ads2, "ADS2", PIN_ADS2_DRDY, PIN_ADS2_CS, 4, false},
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

static ChannelSetting channelSettings[TOTAL_CHANNELS];
static bool leadOffP[TOTAL_CHANNELS];
static bool leadOffN[TOTAL_CHANNELS];

static bool streaming = false;
static uint8_t sampleCounter = 0;
static uint8_t sampleRateCode = 0x06;
static uint8_t boardMode = BOARD_MODE_DEFAULT;
static bool awaitingSampleRate = false;
static bool awaitingBoardMode = false;
static bool awaitingMarker = false;
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
static int16_t auxData[3] = {0, 0, 0};
static bool sdPresent = false;
static bool sdRecording = false;
static File sdFile;
static char sdFileName[24] = {0};
static uint32_t sdRecordStartMs = 0;
static uint32_t sdRecordDurationMs = 0;

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
      bool ok = lis3dhWriteRegister(LIS3DH_REG_CTRL1, LIS3DH_CTRL1_100HZ_XYZ);
      ok = lis3dhWriteRegister(LIS3DH_REG_CTRL4, LIS3DH_CTRL4_BDU_HR_4G) && ok;
      lis3dhPresent = ok;
      return lis3dhPresent;
    }
  }

  lis3dhPresent = false;
  lis3dhWhoAmI = 0x00;
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

static void logSampleToSd(uint8_t packetIndex, const int32_t channels[8], const int16_t aux[3]) {
  if (!sdRecording || !sdFile) {
    return;
  }

  char sampleHex[3];
  snprintf(sampleHex, sizeof(sampleHex), "%02X", static_cast<unsigned>(packetIndex));
  sdFile.print(sampleHex);

  for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ++ch) {
    sdFile.print(',');
    writeHex24(sdFile, channels[ch]);
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    sdFile.print(',');
    writeHex16(sdFile, aux[axis]);
  }

  sdFile.print('\n');
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

  sdFile = SD.open(sdFileName, FILE_WRITE);
  if (!sdFile) {
    sdFileName[0] = '\0';
    emitFailure("failed to open SD file");
    return false;
  }

  sdRecording = true;
  sdRecordStartMs = millis();
  sdRecordDurationMs = durationMs;

  char msg[96];
  snprintf(msg, sizeof(msg), "SD recording started: %s (%s)", sdFileName, label != nullptr ? label : "manual");
  emitSuccess(msg);
  return true;
}

static void closeSdFile() {
  if (sdFile) {
    sdFile.flush();
    sdFile.close();
  }

  sdRecording = false;
  sdRecordStartMs = 0;
  sdRecordDurationMs = 0;
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
  snprintf(msg, sizeof(msg), "{\"mode\":\"debug\",\"channels\":%u,\"baud\":%lu}", TOTAL_CHANNELS, static_cast<unsigned long>(SERIAL_BAUD));
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

  for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
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
  Serial.println("OpenBCI V3 8-Channel");
  Serial.println("On Board ADS1299 Device ID: 0x3E");
  Serial.print("LIS3DH Device ID: 0x");
  if (lis3dhWhoAmI < 0x10) {
    Serial.print('0');
  }
  Serial.println(lis3dhWhoAmI, HEX);
  Serial.print("SD card: ");
  Serial.println(sdPresent ? "present" : "not present");
  Serial.println("Firmware: v3.1.5");
  Serial.print("$$$");
}

static int channelSelectorToIndex(char selector) {
  if (selector >= '1' && selector <= '8') {
    return selector - '1';
  }
  return -1;
}

static bool setDataRateForAll(uint8_t drBits) {
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
  return ok;
}

static bool applyChannelToHardware(uint8_t channelIndex) {
  if (channelIndex >= TOTAL_CHANNELS) {
    return false;
  }

  DeviceCtx& dev = devices[channelIndex / ADS_CHANNELS_PER_CHIP];
  if (!dev.linkOk) {
    return false;
  }

  const uint8_t localIndex = channelIndex % ADS_CHANNELS_PER_CHIP;
  const ChannelSetting& setting = channelSettings[channelIndex];
  bool ok = dev.ads->configureChannel(
      localIndex,
      setting.active,
      setting.gainOrdinal,
      setting.inputTypeOrdinal,
      setting.biasInclude,
      setting.srb2Connect,
      setting.srb1Connect);

  if (!setting.active) {
    leadOffP[channelIndex] = false;
    leadOffN[channelIndex] = false;
    ok = dev.ads->setLeadOffForChannel(localIndex, false, false) && ok;
  }

  return ok;
}

static bool applyLeadOffToHardware(uint8_t channelIndex) {
  if (channelIndex >= TOTAL_CHANNELS) {
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
  for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
    channelSettings[i].active = true;
    channelSettings[i].gainOrdinal = 6;
    channelSettings[i].inputTypeOrdinal = 0;
    channelSettings[i].biasInclude = true;
    channelSettings[i].srb2Connect = true;
    channelSettings[i].srb1Connect = false;
    leadOffP[i] = false;
    leadOffN[i] = false;
  }
}

static bool applyDefaultChannelSettings() {
  const bool wasStreaming = streaming;
  if (wasStreaming) {
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (devices[i].linkOk) {
        devices[i].ads->stopContinuousConversion();
      }
    }
    streaming = false;
  }

  applyDefaultChannelState();

  bool ok = true;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }
    ok = devices[i].ads->applyCytonDefaults(sampleRateCode) && ok;
    ok = devices[i].ads->configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31P2HZ) && ok;
  }

  if (wasStreaming) {
    sampleCounter = 0;
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (devices[i].linkOk) {
        devices[i].ads->startContinuousConversion();
      }
    }
    streaming = true;
  }

  return ok;
}

static void emitStateIfDebug() {
  if (USE_DEBUG_APP_MODE) {
    emitDebugState();
  }
}

static bool setInputTypeForAllActiveChannels(uint8_t inputType, uint8_t amplitudeCode, uint8_t freqCode) {
  const bool wasStreaming = streaming;
  if (wasStreaming) {
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (devices[i].linkOk) {
        devices[i].ads->stopContinuousConversion();
      }
    }
    streaming = false;
  }

  bool ok = true;
  for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
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
    sampleCounter = 0;
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (devices[i].linkOk) {
        devices[i].ads->startContinuousConversion();
      }
    }
    streaming = true;
  }

  return ok;
}

static void stopStreaming() {
  if (!streaming) {
    return;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->stopContinuousConversion();
    }
  }

  streaming = false;
  emitStateIfDebug();
}

static void startStreaming() {
  if (streaming) {
    return;
  }

  sampleCounter = 0;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->startContinuousConversion();
    }
  }

  streaming = true;
  emitStateIfDebug();
}

static bool waitForAllDrdyLow(uint32_t timeoutMs) {
  const uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs) {
    bool anyOnline = false;
    bool allReady = true;

    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (!devices[i].linkOk) {
        continue;
      }
      anyOnline = true;
      if (digitalRead(devices[i].drdyPin) != LOW) {
        allReady = false;
        break;
      }
    }

    if (anyOnline && allReady) {
      return true;
    }

    delayMicroseconds(5);
  }

  return false;
}

static bool readOneSample(int32_t mergedChannels[8]) {
  memset(mergedChannels, 0, sizeof(int32_t) * TOTAL_CHANNELS);

  bool anyOnline = false;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    anyOnline = anyOnline || devices[i].linkOk;
  }
  if (!anyOnline) {
    return false;
  }

  if (!waitForAllDrdyLow(DRDY_TIMEOUT_MS)) {
    return false;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }

    int32_t frame[8] = {0};
    uint32_t status = 0;
    if (!devices[i].ads->readDataFrame(frame, status)) {
      return false;
    }

    for (uint8_t ch = 0; ch < ADS_CHANNELS_PER_CHIP; ++ch) {
      mergedChannels[devices[i].channelOffset + ch] = frame[ch];
    }
  }

  return true;
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

static void fillAuxPayload(uint8_t packet[33]) {
  int16_t localAux[3] = {0, 0, 0};

  if (boardMode == BOARD_MODE_DEFAULT) {
    readLis3dhAuxData(localAux);
  }

  memcpy(auxData, localAux, sizeof(auxData));
  pack16be(localAux[0], &packet[26]);
  pack16be(localAux[1], &packet[28]);
  pack16be(localAux[2], &packet[30]);
}

static void writeOpenBCIPacket(const int32_t channels[8]) {
  uint8_t packet[33];
  const uint8_t packetIndex = sampleCounter;
  packet[0] = PACKET_START_BYTE;
  packet[1] = sampleCounter++;

  uint8_t offset = 2;
  for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ++ch) {
    pack24be(channels[ch], &packet[offset]);
    offset += 3;
  }

  fillAuxPayload(packet);
  packet[32] = PACKET_END_STANDARD;

  Serial.write(packet, sizeof(packet));

  const int16_t currentAux[3] = {auxData[0], auxData[1], auxData[2]};
  logSampleToSd(packetIndex, channels, currentAux);
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
  int index = -1;
  bool active = true;

  if (cmd >= '1' && cmd <= '8') {
    index = cmd - '1';
    active = false;
  } else {
    switch (cmd) {
      case '!': index = 0; active = true; break;
      case '@': index = 1; active = true; break;
      case '#': index = 2; active = true; break;
      case '$': index = 3; active = true; break;
      case '%': index = 4; active = true; break;
      case '^': index = 5; active = true; break;
      case '&': index = 6; active = true; break;
      case '*': index = 7; active = true; break;
      default: break;
    }
  }

  if (index < 0 || index >= TOTAL_CHANNELS) {
    return;
  }

  channelSettings[index].active = active;
  if (applyChannelToHardware(static_cast<uint8_t>(index))) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Channel %d %s", index + 1, active ? "on" : "off");
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

  int channelIndex = channelSelectorToIndex(payload[0]);
  if (channelIndex < 0) {
    emitFailure("invalid channel selector");
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

  ChannelSetting& setting = channelSettings[channelIndex];
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

  if (applyChannelToHardware(static_cast<uint8_t>(channelIndex))) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Channel set for %d", channelIndex + 1);
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

  int channelIndex = channelSelectorToIndex(payload[0]);
  if (channelIndex < 0) {
    emitFailure("invalid impedance channel");
    return false;
  }

  leadOffP[channelIndex] = payload[1] == '1';
  leadOffN[channelIndex] = payload[2] == '1';

  if (applyLeadOffToHardware(static_cast<uint8_t>(channelIndex))) {
    char msg[32];
    snprintf(msg, sizeof(msg), "Lead off set for %d", channelIndex + 1);
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
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > TOTAL_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), active)) {
      emitDebugLine("#ERR ", "channel");
      return;
    }
    channelSettings[channel - 1].active = active;
    if (applyChannelToHardware(channel - 1)) {
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
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > TOTAL_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), active) ||
      !parseUInt(strtok_r(nullptr, " ", &context), gain) || gain > 6 ||
      !parseUInt(strtok_r(nullptr, " ", &context), input) || input > 7 ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), bias) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), srb2) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), srb1)) {
      emitDebugLine("#ERR ", "config");
      return;
    }

    ChannelSetting& setting = channelSettings[channel - 1];
    setting.active = active;
    setting.gainOrdinal = gain;
    setting.inputTypeOrdinal = input;
    setting.biasInclude = bias;
    setting.srb2Connect = srb2;
    setting.srb1Connect = srb1;

    if (applyChannelToHardware(channel - 1)) {
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
    if (!parseUInt(strtok_r(nullptr, " ", &context), channel) || channel < 1 || channel > TOTAL_CHANNELS ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), pEnable) ||
      !parseBoolToken(strtok_r(nullptr, " ", &context), nEnable)) {
      emitDebugLine("#ERR ", "imp");
      return;
    }
    leadOffP[channel - 1] = pEnable;
    leadOffN[channel - 1] = nEnable;
    if (applyLeadOffToHardware(channel - 1)) {
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
      stopStreaming();
      emitSuccess("Stream stopped");
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
      emitSuccess("No daisy to remove");
      break;
    case 'C':
      emitSuccess("No daisy to attach");
      break;
    case '?':
      printAllRegisters();
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
    devices[i].linkOk = devices[i].ads->begin(
        devices[i].drdyPin,
        devices[i].csPin,
        PIN_ADS_START,
        PIN_ADS_RESET,
        PIN_ADS_PWDN,
        PIN_SPI_SCLK,
        PIN_SPI_MISO,
        PIN_SPI_MOSI,
        1000000,
        ADS_CHANNELS_PER_CHIP);
  }

  bringUpLis3dh();
  bringUpSdCard();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  bringUpDevices();
  applyDefaultChannelState();
  applyDefaultChannelSettings();
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

  if (!streaming) {
    delay(1);
    return;
  }

  int32_t channels[8] = {0};
  if (readOneSample(channels)) {
    writeOpenBCIPacket(channels);
  }
}
