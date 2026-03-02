#include <Arduino.h>
#include <math.h>
#include "ADS1299.h"

static constexpr int PIN_ADS1_DRDY = 34;
static constexpr int PIN_ADS1_CS = 5;
static constexpr int PIN_ADS2_DRDY = 16;
static constexpr int PIN_ADS2_CS = 4;
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
static constexpr uint32_t REPORT_INTERVAL_MS = 200;
static constexpr uint32_t DRDY_TIMEOUT_MS = 12;
static constexpr uint32_t DRDY_WARN_PRINT_EVERY = 50;
static constexpr uint32_t ADS_SAMPLE_PERIOD_US = 4000;
static constexpr uint32_t ANALOG_TEST_DRDY_TIMEOUT_MS = 30;
static constexpr int ANALOG_TEST_SETTLE_SAMPLES = 200;
static constexpr int ANALOG_TEST_SAMPLE_COUNT = 1000;
static constexpr float ANALOG_TEST_RMS_AC_PASS_COUNTS = 8000.0f;
static uint32_t windowSampleCount = 0;
static uint32_t lastReportMs = 0;
static float chSumUv[8];
static float chMinUv[8];
static float chMaxUv[8];

struct AdsDeviceContext {
  ADS1299* ads;
  const char* label;
  int drdyPin;
  int csPin;
  uint8_t mergedOffset;
  bool linkOk;
  bool online;
  uint32_t lastStatus;
  uint32_t drdyTimeoutCount;
};

static AdsDeviceContext devices[] = {
    {&ads1, "ADS1", PIN_ADS1_DRDY, PIN_ADS1_CS, 0, false, false, 0, 0},
    {&ads2, "ADS2", PIN_ADS2_DRDY, PIN_ADS2_CS, 4, false, false, 0, 0},
};
static constexpr size_t DEVICE_COUNT = sizeof(devices) / sizeof(devices[0]);

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

void bringUpDevice(AdsDeviceContext& dev) {
  dev.linkOk = dev.ads->begin(
      dev.drdyPin,
      dev.csPin,
      PIN_ADS1_START,
      PIN_ADS1_RESET,
      PIN_ADS1_PWDN,
      PIN_SPI_SCLK,
      PIN_SPI_MISO,
      PIN_SPI_MOSI,
      1000000,
      ADS_CHANNEL_COUNT
  );

  if (!dev.linkOk) {
    Serial.printf("[BOOT] WARN: %s not responding. Corresponding channels will be zero.\n", dev.label);
    dev.online = false;
    return;
  }

  bool digitalOk = runDigitalDiagnostics(*dev.ads, dev.label);
  if (!digitalOk) Serial.printf("[BOOT] WARN: %s digital test failed; continuing.\n", dev.label);
  dev.online = false;
}

void runSynchronizedAnalogInternalTests(bool analogPass[DEVICE_COUNT]) {
  bool testEnabled[DEVICE_COUNT] = {false};
  int32_t minVal[DEVICE_COUNT][4];
  int32_t maxVal[DEVICE_COUNT][4];
  int signChanges[DEVICE_COUNT][4];
  int32_t prev[DEVICE_COUNT][4];
  bool havePrev[DEVICE_COUNT][4];
  double sum[DEVICE_COUNT][4];
  double sumSquares[DEVICE_COUNT][4];
  uint32_t strictDrdyTimeouts[DEVICE_COUNT] = {0};
  int sampleCount[DEVICE_COUNT] = {0};

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    for (int channel = 0; channel < 4; ++channel) {
      minVal[i][channel] = INT32_MAX;
      maxVal[i][channel] = INT32_MIN;
      signChanges[i][channel] = 0;
      prev[i][channel] = 0;
      havePrev[i][channel] = false;
      sum[i][channel] = 0.0;
      sumSquares[i][channel] = 0.0;
    }

    if (!devices[i].linkOk) {
      analogPass[i] = false;
      continue;
    }

    if (!devices[i].ads->configureInternalTestSignal()) {
      Serial.printf("[ANALOG-%s] FAIL: Could not configure internal test signal mode.\n", devices[i].label);
      analogPass[i] = false;
      continue;
    }

    testEnabled[i] = true;
    analogPass[i] = false;
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (testEnabled[i]) {
      devices[i].ads->startContinuousConversion();
    }
  }

  for (int settleIndex = 0; settleIndex < ANALOG_TEST_SETTLE_SAMPLES; ++settleIndex) {
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (!testEnabled[i]) {
        continue;
      }

      int32_t chDiscard[8] = {0};
      uint32_t statusDiscard = 0;
      if (devices[i].ads->waitForDRDY(ANALOG_TEST_DRDY_TIMEOUT_MS)) {
        devices[i].ads->readDataFrame(chDiscard, statusDiscard);
      } else {
        strictDrdyTimeouts[i]++;
        testEnabled[i] = false;
      }
    }
  }

  for (int sampleIndex = 0; sampleIndex < ANALOG_TEST_SAMPLE_COUNT; ++sampleIndex) {
    for (size_t i = 0; i < DEVICE_COUNT; ++i) {
      if (!testEnabled[i]) {
        continue;
      }

      int32_t ch[8] = {0};
      uint32_t status = 0;
      if (devices[i].ads->waitForDRDY(ANALOG_TEST_DRDY_TIMEOUT_MS)) {
        devices[i].ads->readDataFrame(ch, status);
      } else {
        strictDrdyTimeouts[i]++;
        testEnabled[i] = false;
        continue;
      }

      sampleCount[i]++;

      for (int channel = 0; channel < 4; ++channel) {
        int32_t sample = ch[channel];
        if (sample < minVal[i][channel]) minVal[i][channel] = sample;
        if (sample > maxVal[i][channel]) maxVal[i][channel] = sample;
        sum[i][channel] += static_cast<double>(sample);
        sumSquares[i][channel] += static_cast<double>(sample) * static_cast<double>(sample);

        if (havePrev[i][channel]) {
          if ((prev[i][channel] < 0 && sample >= 0) || (prev[i][channel] >= 0 && sample < 0)) {
            signChanges[i][channel]++;
          }
        }
        prev[i][channel] = sample;
        havePrev[i][channel] = true;
      }
    }
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (testEnabled[i]) {
      devices[i].ads->stopContinuousConversion();
    }
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }

    if (strictDrdyTimeouts[i] > 0) {
      Serial.printf("[ANALOG-%s] FAIL: Strict DRDY timeout occurred %lu times during analog test.\n",
                    devices[i].label,
                    strictDrdyTimeouts[i]);
      analogPass[i] = false;
      continue;
    }

    if (sampleCount[i] < ANALOG_TEST_SAMPLE_COUNT) {
      Serial.printf("[ANALOG-%s] FAIL: Incomplete sample window (%d/%d).\n",
                    devices[i].label,
                    sampleCount[i],
                    ANALOG_TEST_SAMPLE_COUNT);
      analogPass[i] = false;
      continue;
    }

    int passCount = 0;
    for (int channel = 0; channel < 4; ++channel) {
      int32_t peakToPeak = maxVal[i][channel] - minVal[i][channel];
      float p2pUv = devices[i].ads->countsToVolts(peakToPeak) * 1e6f;
      double meanCounts = sum[i][channel] / static_cast<double>(ANALOG_TEST_SAMPLE_COUNT);
      double meanSquare = sumSquares[i][channel] / static_cast<double>(ANALOG_TEST_SAMPLE_COUNT);
      double variance = meanSquare - (meanCounts * meanCounts);
      if (variance < 0.0) {
        variance = 0.0;
      }
      float rmsAcCounts = sqrtf(static_cast<float>(variance));
      float rmsAcUv = devices[i].ads->countsToVolts(static_cast<int32_t>(rmsAcCounts)) * 1e6f;
      bool channelPass = rmsAcCounts >= ANALOG_TEST_RMS_AC_PASS_COUNTS;

      Serial.printf("[ANALOG-%s] CH%d p2p=%ld counts (%.2f uV), rms(ac)=%.0f counts (%.2f uV), mean=%.0f counts, sign changes=%d -> %s\n",
                    devices[i].label,
                    channel + 1,
                    peakToPeak,
                    p2pUv,
                    rmsAcCounts,
                    rmsAcUv,
                    static_cast<float>(meanCounts),
                    signChanges[i][channel],
                    channelPass ? "PASS" : "FAIL");

      if (channelPass) {
        passCount++;
        if (signChanges[i][channel] < 1) {
          Serial.printf("[ANALOG-%s] CH%d WARN: Low sign-change count; amplitude still healthy.\n", devices[i].label, channel + 1);
        }
      }
    }

    if (passCount == 0) {
      Serial.printf("[ANALOG-%s] FAIL: No channels passed internal analog amplitude test.\n", devices[i].label);
      analogPass[i] = false;
    } else {
      if (passCount < 4) {
        Serial.printf("[ANALOG-%s] WARN: %d/4 channels passed internal analog amplitude test.\n", devices[i].label, passCount);
      } else {
        Serial.printf("[ANALOG-%s] PASS: 4/4 channels passed internal analog amplitude test.\n", devices[i].label);
      }
      analogPass[i] = true;
    }
  }
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

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    bringUpDevice(devices[i]);
  }

  bool analogPass[DEVICE_COUNT] = {false};
  runSynchronizedAnalogInternalTests(analogPass);

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (!devices[i].linkOk) {
      continue;
    }

    if (!analogPass[i]) {
      Serial.printf("[BOOT] WARN: %s analog internal test failed; continuing.\n", devices[i].label);
    }

    bool configOk = devices[i].ads->configureBasicEEG();
    if (!configOk) {
      Serial.printf("[BOOT] WARN: %s normal EEG configuration failed; continuing.\n", devices[i].label);
    }
    devices[i].online = configOk;
  }

      Serial.printf("[BOOT] CS post-init states: CS1=%d CS2=%d\n",
            digitalRead(PIN_ADS1_CS),
            digitalRead(PIN_ADS2_CS));

  bool anyOnline = false;
  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    anyOnline = anyOnline || devices[i].online;
  }

  if (!anyOnline) {
    Serial.println("[BOOT] WARN: No ADS1299 configured for live conversion. Streaming zeros for debug.");
  }

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    if (devices[i].online) {
      devices[i].ads->startContinuousConversion();
    }
  }

  Serial.printf("[BOOT] Live stream mode: ADS1=%s ADS2=%s\n",
                devices[0].online ? "ON" : "OFF",
                devices[1].online ? "ON" : "OFF");
  Serial.println("Waiting 5 seconds before live stream...");
  delay(5000);
  Serial.println("Live summary at 5 Hz: avg/min/max uV for ch1..ch8");

  for (int i = 0; i < 8; ++i) {
    chSumUv[i] = 0.0f;
    chMinUv[i] = 1e12f;
    chMaxUv[i] = -1e12f;
  }
  lastReportMs = millis();
}

void loop() {
  int32_t ch[8] = {0};

  for (size_t i = 0; i < DEVICE_COUNT; ++i) {
    AdsDeviceContext& dev = devices[i];
    if (!dev.online) {
      continue;
    }

    int32_t frame[8] = {0};
    uint32_t status = 0;
    if (dev.ads->waitForDRDY(DRDY_TIMEOUT_MS)) {
      dev.ads->readDataFrame(frame, status);
      dev.lastStatus = status;
    } else {
      dev.drdyTimeoutCount++;
      delayMicroseconds(ADS_SAMPLE_PERIOD_US);
      dev.ads->readDataFrame(frame, status);
      dev.lastStatus = status;
      if ((dev.drdyTimeoutCount % DRDY_WARN_PRINT_EVERY) == 1) {
        Serial.printf("[RUN] Warning: %s DRDY timeout (%lu total), using timed fallback read.\n",
                      dev.label,
                      dev.drdyTimeoutCount);
      }
    }

    for (uint8_t channel = 0; channel < ADS_CHANNEL_COUNT; ++channel) {
      ch[dev.mergedOffset + channel] = frame[channel];
    }
  }

  float uV[8];
  for (int i = 0; i < 8; ++i) {
    uV[i] = devices[0].ads->countsToVolts(ch[i]) * 1e6f;
    chSumUv[i] += uV[i];
    if (uV[i] < chMinUv[i]) chMinUv[i] = uV[i];
    if (uV[i] > chMaxUv[i]) chMaxUv[i] = uV[i];
  }
  windowSampleCount++;

  uint32_t now = millis();
  if ((now - lastReportMs) >= REPORT_INTERVAL_MS && windowSampleCount > 0) {
    Serial.printf("[5Hz] t=%lu ms s1=0x%06lX s2=0x%06lX n=%lu\n",
                  now,
                  devices[0].lastStatus & 0xFFFFFF,
                  devices[1].lastStatus & 0xFFFFFF,
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