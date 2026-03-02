#include <Arduino.h>
#include "ADS1299.h"

static constexpr int PIN_ADS1_DRDY = 34;
static constexpr int PIN_ADS1_CS = 4;
static constexpr int PIN_ADS2_DRDY = 16;
static constexpr int PIN_ADS2_CS = 5;
static constexpr int PIN_ADS1_START = 25;
static constexpr int PIN_ADS1_RESET = 17;
static constexpr int PIN_ADS1_PWDN = 2;
static constexpr int PIN_AUX_CS3 = 26;

static constexpr int PIN_SPI_SCLK = 18;
static constexpr int PIN_SPI_MISO = 19;
static constexpr int PIN_SPI_MOSI = 23;
static constexpr uint8_t ADS_CHANNEL_COUNT = 4;

ADS1299 ads1;
ADS1299 ads2;
static bool ads1Online = false;
static bool ads2Online = false;
static constexpr uint32_t REPORT_INTERVAL_MS = 200;
static uint32_t windowSampleCount = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastStatusA = 0;
static uint32_t lastStatusB = 0;
static float chSumUv[8];
static float chMinUv[8];
static float chMaxUv[8];

bool runDigitalDiagnostics(ADS1299& ads, const char* label) {
  ads.SDATAC();
  delay(2);

  uint8_t id = ads.readRegister(ID);
  Serial.printf("[DIGITAL-%s] ID register = 0x%02X\n", label, id);

  bool idLooksValid = ((id & 0x3C) == 0x3C) && (id != 0x00) && (id != 0xFF);
  if (!idLooksValid) {
    Serial.printf("[DIGITAL-%s] FAIL: ID value is not a valid ADS1299 family signature.\n", label);
    return false;
  }

  uint8_t original = ads.readRegister(CONFIG1);
  uint8_t testValue = (original & 0xF8) | 0x05;
  if (testValue == original) {
    testValue = (original & 0xF8) | 0x06;
  }

  ads.writeRegister(CONFIG1, testValue);
  uint8_t readBack = ads.readRegister(CONFIG1);

  ads.writeRegister(CONFIG1, original);

  if (readBack != testValue) {
    Serial.printf("[DIGITAL-%s] FAIL: CONFIG1 write/read mismatch (wrote 0x%02X read 0x%02X).\n", label, testValue, readBack);
    return false;
  }

  Serial.printf("[DIGITAL-%s] PASS: SPI command path and register R/W look healthy.\n", label);
  return true;
}

bool runAnalogInternalTest(ADS1299& ads, const char* label) {
  if (!ads.configureInternalTestSignal()) {
    Serial.printf("[ANALOG-%s] FAIL: Could not configure internal test signal mode.\n", label);
    return false;
  }

  ads.startContinuousConversion();

  int32_t minVal[4] = {INT32_MAX, INT32_MAX, INT32_MAX, INT32_MAX};
  int32_t maxVal[4] = {INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN};
  int signChanges[4] = {0, 0, 0, 0};
  int32_t prev[4] = {0, 0, 0, 0};
  bool havePrev[4] = {false, false, false, false};

  for (int i = 0; i < 500; ++i) {
    if (!ads.waitForDRDY(100)) {
      Serial.printf("[ANALOG-%s] FAIL: DRDY timeout while reading internal test signal.\n", label);
      ads.stopContinuousConversion();
      return false;
    }

    int32_t ch[8] = {0};
    uint32_t status = 0;
    ads.readDataFrame(ch, status);

    for (int channel = 0; channel < 4; ++channel) {
      int32_t sample = ch[channel];
      if (sample < minVal[channel]) minVal[channel] = sample;
      if (sample > maxVal[channel]) maxVal[channel] = sample;

      if (havePrev[channel]) {
        if ((prev[channel] < 0 && sample >= 0) || (prev[channel] >= 0 && sample < 0)) {
          signChanges[channel]++;
        }
      }
      prev[channel] = sample;
      havePrev[channel] = true;
    }
  }

  ads.stopContinuousConversion();

  int passCount = 0;
  for (int channel = 0; channel < 4; ++channel) {
    int32_t peakToPeak = maxVal[channel] - minVal[channel];
    float p2pUv = ads.countsToVolts(peakToPeak) * 1e6f;
    bool channelPass = peakToPeak >= 50000;

    Serial.printf("[ANALOG-%s] CH%d p2p=%ld counts (%.2f uV), sign changes=%d -> %s\n",
                  label,
                  channel + 1,
                  peakToPeak,
                  p2pUv,
                  signChanges[channel],
                  channelPass ? "PASS" : "FAIL");

    if (channelPass) {
      passCount++;
      if (signChanges[channel] < 1) {
        Serial.printf("[ANALOG-%s] CH%d WARN: Low sign-change count; amplitude still healthy.\n", label, channel + 1);
      }
    }
  }

  if (passCount == 0) {
    Serial.printf("[ANALOG-%s] FAIL: No channels passed internal analog amplitude test.\n", label);
    return false;
  }

  if (passCount < 4) {
    Serial.printf("[ANALOG-%s] WARN: %d/4 channels passed internal analog amplitude test.\n", label, passCount);
  } else {
    Serial.printf("[ANALOG-%s] PASS: 4/4 channels passed internal analog amplitude test.\n", label);
  }
  return true;
}

void setup() {
  Serial.begin(921600);
  delay(1500);
  Serial.println("\n=== ADS1299 dual-chip bring-up on ESP32 ===");
  Serial.println("Assumed supplies: AVDD=+2.5V AVSS=-2.5V, internal VREF=4.5V");

  pinMode(PIN_ADS1_CS, OUTPUT);
  pinMode(PIN_ADS2_CS, OUTPUT);
  pinMode(PIN_AUX_CS3, OUTPUT);
  digitalWrite(PIN_ADS1_CS, HIGH);
  digitalWrite(PIN_ADS2_CS, HIGH);
  digitalWrite(PIN_AUX_CS3, HIGH);
  Serial.printf("[BOOT] CS idle states: CS1=%d CS2=%d CS3=%d\n",
                digitalRead(PIN_ADS1_CS),
                digitalRead(PIN_ADS2_CS),
                digitalRead(PIN_AUX_CS3));

  bool linkOk1 = ads1.begin(
      PIN_ADS1_DRDY,
      PIN_ADS1_CS,
      PIN_ADS1_START,
      PIN_ADS1_RESET,
      PIN_ADS1_PWDN,
      PIN_SPI_SCLK,
      PIN_SPI_MISO,
      PIN_SPI_MOSI,
        1000000,
        ADS_CHANNEL_COUNT
  );

  bool linkOk2 = ads2.begin(
      PIN_ADS2_DRDY,
      PIN_ADS2_CS,
      PIN_ADS1_START,
      PIN_ADS1_RESET,
      PIN_ADS1_PWDN,
      PIN_SPI_SCLK,
      PIN_SPI_MISO,
      PIN_SPI_MOSI,
        1000000,
        ADS_CHANNEL_COUNT
  );

      Serial.printf("[BOOT] CS post-init states: CS1=%d CS2=%d\n",
            digitalRead(PIN_ADS1_CS),
            digitalRead(PIN_ADS2_CS));

  if (!linkOk1) {
    Serial.println("[BOOT] WARN: ADS1299 #1 not responding. Channels 1-4 will be zero.");
  } else {
    bool d1 = runDigitalDiagnostics(ads1, "ADS1");
    bool a1 = runAnalogInternalTest(ads1, "ADS1");
    bool c1 = ads1.configureBasicEEG();
    if (!d1) Serial.println("[BOOT] WARN: ADS1 digital test failed; continuing.");
    if (!a1) Serial.println("[BOOT] WARN: ADS1 analog internal test failed; continuing.");
    if (!c1) Serial.println("[BOOT] WARN: ADS1 normal EEG configuration failed; continuing.");
    ads1Online = c1;
    if (ads1Online) {
      ads1.startContinuousConversion();
    }
  }

  if (!linkOk2) {
    Serial.println("[BOOT] WARN: ADS1299 #2 not responding. Channels 5-8 will be zero.");
  } else {
    bool d2 = runDigitalDiagnostics(ads2, "ADS2");
    bool a2 = runAnalogInternalTest(ads2, "ADS2");
    bool c2 = ads2.configureBasicEEG();
    if (!d2) Serial.println("[BOOT] WARN: ADS2 digital test failed; continuing.");
    if (!a2) Serial.println("[BOOT] WARN: ADS2 analog internal test failed; continuing.");
    if (!c2) Serial.println("[BOOT] WARN: ADS2 normal EEG configuration failed; continuing.");
    ads2Online = c2;
    if (ads2Online) {
      ads2.startContinuousConversion();
    }
  }

  if (!ads1Online && !ads2Online) {
    Serial.println("[BOOT] WARN: No ADS1299 configured for live conversion. Streaming zeros for debug.");
  }

  Serial.printf("[BOOT] Live stream mode: ADS1=%s ADS2=%s\n", ads1Online ? "ON" : "OFF", ads2Online ? "ON" : "OFF");
  Serial.println("Waiting 3 seconds before live stream...");
  delay(3000);
  Serial.println("Live summary at 5 Hz: avg/min/max uV for ch1..ch8");

  for (int i = 0; i < 8; ++i) {
    chSumUv[i] = 0.0f;
    chMinUv[i] = 1e12f;
    chMaxUv[i] = -1e12f;
  }
  lastReportMs = millis();
}

void loop() {
  int32_t chA[8] = {0};
  int32_t chB[8] = {0};
  int32_t ch[8] = {0};
  uint32_t statusA = 0;
  uint32_t statusB = 0;

  if (ads1Online) {
    if (ads1.waitForDRDY(100)) {
      ads1.readDataFrame(chA, statusA);
    } else {
      Serial.println("[RUN] Warning: ADS1 DRDY timeout, using zeros for ch1-ch4 this frame.");
    }
  }

  if (ads2Online) {
    if (ads2.waitForDRDY(100)) {
      ads2.readDataFrame(chB, statusB);
    } else {
      Serial.println("[RUN] Warning: ADS2 DRDY timeout, using zeros for ch5-ch8 this frame.");
    }
  }

  ch[0] = chA[0];
  ch[1] = chA[1];
  ch[2] = chA[2];
  ch[3] = chA[3];
  ch[4] = chB[0];
  ch[5] = chB[1];
  ch[6] = chB[2];
  ch[7] = chB[3];

  float uV[8];
  for (int i = 0; i < 8; ++i) {
    uV[i] = ads1.countsToVolts(ch[i]) * 1e6f;
    chSumUv[i] += uV[i];
    if (uV[i] < chMinUv[i]) chMinUv[i] = uV[i];
    if (uV[i] > chMaxUv[i]) chMaxUv[i] = uV[i];
  }
  windowSampleCount++;
  lastStatusA = statusA;
  lastStatusB = statusB;

  uint32_t now = millis();
  if ((now - lastReportMs) >= REPORT_INTERVAL_MS && windowSampleCount > 0) {
    Serial.printf("[5Hz] t=%lu ms s1=0x%06lX s2=0x%06lX n=%lu\n",
                  now,
                  lastStatusA & 0xFFFFFF,
                  lastStatusB & 0xFFFFFF,
                  windowSampleCount);

    for (int i = 0; i < 8; ++i) {
      float avgUv = chSumUv[i] / static_cast<float>(windowSampleCount);
      Serial.printf("  ch%d avg=%8.2f uV min=%8.2f max=%8.2f\n", i + 1, avgUv, chMinUv[i], chMaxUv[i]);
      chSumUv[i] = 0.0f;
      chMinUv[i] = 1e12f;
      chMaxUv[i] = -1e12f;
    }

    lastReportMs = now;
    windowSampleCount = 0;
  }
}