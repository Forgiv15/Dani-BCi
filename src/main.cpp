#include <Arduino.h>
#include <ctype.h>
#include <stdint.h>
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
static constexpr uint32_t DRDY_TIMEOUT_MS = 6;

static constexpr uint8_t PACKET_START_BYTE = 0xA0;
static constexpr uint8_t PACKET_END_STANDARD = 0xC0;

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

static bool streaming = false;
static uint8_t sampleCounter = 0;
static bool awaitingSampleRate = false;
static bool parsingX = false;
static char xPayload[24];
static uint8_t xLen = 0;
static bool parsingZ = false;
static char zPayload[8];
static uint8_t zLen = 0;

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

  uint8_t localIndex = channelIndex % ADS_CHANNELS_PER_CHIP;
  const ChannelSetting& setting = channelSettings[channelIndex];

  return dev.ads->configureChannel(
      localIndex,
      setting.active,
      setting.gainOrdinal,
      setting.inputTypeOrdinal,
      setting.biasInclude,
      setting.srb2Connect,
      setting.srb1Connect);
}

static void applyDefaultChannelSettings() {
  for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
    channelSettings[i].active = true;
    channelSettings[i].gainOrdinal = 6;
    channelSettings[i].inputTypeOrdinal = 0;
    channelSettings[i].biasInclude = true;
    channelSettings[i].srb2Connect = true;
    channelSettings[i].srb1Connect = false;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }
    devices[i].ads->applyCytonDefaults(0x06);
  }
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

static bool readOneSample(int32_t mergedChannels[8]) {
  memset(mergedChannels, 0, sizeof(int32_t) * TOTAL_CHANNELS);

  bool anyOnline = false;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    anyOnline = anyOnline || devices[i].linkOk;
  }
  if (!anyOnline) {
    return false;
  }

  size_t syncDev = 0;
  if (!devices[syncDev].linkOk && DEVICE_COUNT > 1) {
    syncDev = 1;
  }

  if (!devices[syncDev].ads->waitForDRDY(DRDY_TIMEOUT_MS)) {
    return false;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }

    int32_t frame[8] = {0};
    uint32_t status = 0;
    devices[i].ads->readDataFrame(frame, status);

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

static void printVersionInfo() {
  Serial.println("OpenBCI V3 8-Channel Simulator");
  Serial.println("On Board ADS1299 Device ID: 0x3E");
  Serial.println("LIS3DH Device ID: 0x33");
  Serial.println("$$$");
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
  applyChannelToHardware(static_cast<uint8_t>(index));
}

static bool parseAndApplyXCommand(const char* payload, uint8_t len) {
  if (len < 7) {
    return false;
  }

  int channelIndex = channelSelectorToIndex(payload[0]);
  if (channelIndex < 0) {
    return false;
  }

  char digits[6];
  for (uint8_t i = 0; i < 6; ++i) {
    if (!isdigit(static_cast<unsigned char>(payload[i + 1]))) {
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

  return applyChannelToHardware(static_cast<uint8_t>(channelIndex));
}

static void parseAndApplyZCommand(const char* payload, uint8_t len) {
  if (len < 3) {
    return;
  }

  int channelIndex = channelSelectorToIndex(payload[0]);
  if (channelIndex < 0) {
    return;
  }

  bool testP = (payload[1] == '1');
  bool testN = (payload[2] == '1');

  ChannelSetting& setting = channelSettings[channelIndex];
  if (testP || testN) {
    setting.active = true;
    setting.gainOrdinal = 0;
    setting.inputTypeOrdinal = 0;
    setting.biasInclude = true;
    setting.srb2Connect = false;
    setting.srb1Connect = false;
  } else {
    setting.active = true;
    setting.gainOrdinal = 6;
    setting.inputTypeOrdinal = 0;
    setting.biasInclude = true;
    setting.srb2Connect = true;
    setting.srb1Connect = false;
  }
  applyChannelToHardware(static_cast<uint8_t>(channelIndex));
}

static void handleSampleRateChar(char rateChar) {
  switch (rateChar) {
    case '0': setDataRateForAll(0x00); break;
    case '1': setDataRateForAll(0x01); break;
    case '2': setDataRateForAll(0x02); break;
    case '3': setDataRateForAll(0x03); break;
    case '4': setDataRateForAll(0x04); break;
    case '5': setDataRateForAll(0x05); break;
    case '6': setDataRateForAll(0x06); break;
    default: break;
  }
}

static void handleCommandChar(char cmd) {
  if (awaitingSampleRate) {
    awaitingSampleRate = false;
    handleSampleRateChar(cmd);
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
    }
    return;
  }

  switch (cmd) {
    case 'b':
      startStreaming();
      break;
    case 's':
      stopStreaming();
      break;
    case 'v':
      stopStreaming();
      printVersionInfo();
      break;
    case 'd':
      stopStreaming();
      applyDefaultChannelSettings();
      Serial.println("ADS1299 set to default");
      break;
    case 'c':
      Serial.println("daisy removed");
      break;
    case 'C':
      Serial.println("no daisy to attach");
      break;
    case '?':
      Serial.println("register dump not implemented");
      break;
    case '~':
      awaitingSampleRate = true;
      break;
    case 'x':
      parsingX = true;
      xLen = 0;
      break;
    case 'z':
      parsingZ = true;
      zLen = 0;
      break;
    default:
      handleChannelShortcut(cmd);
      break;
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = static_cast<char>(Serial.read());
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
