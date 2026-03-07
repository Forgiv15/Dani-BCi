#include <Arduino.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ADS1299.h"

static constexpr int PIN_ADS1_DRDY = 34;
static constexpr int PIN_ADS1_CS = 5;
static constexpr int PIN_ADS2_DRDY = 16;
static constexpr int PIN_ADS2_CS = 4;
static constexpr int PIN_ADS_START = 25;
static constexpr int PIN_ADS_RESET = 17;
static constexpr int PIN_ADS_PWDN = 2;
static constexpr int PIN_AUX_CS3 = 26;

static constexpr int PIN_SPI_SCLK = 18;
static constexpr int PIN_SPI_MISO = 19;
static constexpr int PIN_SPI_MOSI = 23;

static constexpr uint8_t ADS_CHANNELS_PER_CHIP = 4;
static constexpr uint8_t TOTAL_CHANNELS = 8;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t DRDY_TIMEOUT_MS = 8;

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

static void emitRadioResponse(const char* msg) {
  Serial.println(msg);
}

static void emitRawResponse(const char* msg) {
  Serial.print(msg);
  Serial.print("$$$");
}

static void emitSuccess(const char* msg) {
  Serial.print("Success: ");
  Serial.print(msg);
  Serial.print("$$$");
}

static void emitFailure(const char* msg) {
  Serial.print("Failure: ");
  Serial.print(msg);
  Serial.print("$$$");
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

static void printVersionInfo() {
  Serial.println("OpenBCI V3 8-Channel");
  Serial.println("On Board ADS1299 Device ID: 0x3E");
  Serial.println("LIS3DH Device ID: 0x33");
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

static void writeOpenBCIPacket(const int32_t channels[8]) {
  uint8_t packet[33];
  packet[0] = PACKET_START_BYTE;
  packet[1] = sampleCounter++;

  uint8_t offset = 2;
  for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ++ch) {
    pack24be(channels[ch], &packet[offset]);
    offset += 3;
  }

  packet[26] = 0;
  packet[27] = 0;
  packet[28] = 0;
  packet[29] = 0;
  packet[30] = 0;
  packet[31] = 0;
  packet[32] = PACKET_END_STANDARD;

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
  Serial.println();
  Serial.println("Board ADS Registers");
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].linkOk) {
      devices[i].ads->printRegisters(Serial, devices[i].name);
      Serial.println();
    }
  }
  Serial.println("LIS3DH Registers");
  Serial.println("0x0F, 0x33");
  Serial.print("$$$");
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
    default:
      handleChannelShortcut(cmd);
      break;
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    uint8_t raw = static_cast<uint8_t>(Serial.read());

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
  pinMode(PIN_AUX_CS3, OUTPUT);

  digitalWrite(PIN_ADS1_CS, HIGH);
  digitalWrite(PIN_ADS2_CS, HIGH);
  digitalWrite(PIN_AUX_CS3, HIGH);

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
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  bringUpDevices();
  applyDefaultChannelState();
  applyDefaultChannelSettings();
  printVersionInfo();
}

void loop() {
  pollSerialCommands();

  if (!streaming) {
    delay(1);
    return;
  }

  int32_t channels[8] = {0};
  if (readOneSample(channels)) {
    writeOpenBCIPacket(channels);
  }
}
