//
//  ADS1299.cpp
//

#include "ADS1299.h"

namespace {
SPIClass sharedSpi(VSPI);
bool sharedSpiInitialized = false;

constexpr uint8_t kGainMask = 0x70;
constexpr uint8_t kInputMask = 0x07;
constexpr uint8_t kSrb2Mask = 0x08;
constexpr uint8_t kPowerDownMask = 0x80;

constexpr uint8_t kLeadOffFreq31Hz = 0x02;
}

ADS1299::ADS1299()
    : spiSettings(1000000, MSBFIRST, SPI_MODE1),
      pinDRDY(-1),
      pinCS(-1),
      pinSTART(-1),
      pinRESET(-1),
      pinPWDN(-1),
      vref(4.5f),
            gain(24),
                        deviceId(0x00),
            activeChannels(8),
                        outputChannels(8),
            config1Cache(0x96),
            biasSenseP(0x00),
            biasSenseN(0x00),
            srb1Mask(0x00),
            leadOffConfig(kLeadOffFreq31Hz),
            leadOffSenseP(0x00),
            leadOffSenseN(0x00),
            lastDrdyLevel(HIGH),
            running(false) {
        for (uint8_t i = 0; i < 8; ++i) {
                channelSettings[i] = 0x68;
        }
}

bool ADS1299::begin(
    int8_t drdyPin,
    int8_t csPin,
    int8_t startPin,
    int8_t resetPin,
    int8_t pwdnPin,
    int8_t sclkPin,
    int8_t misoPin,
    int8_t mosiPin,
    uint32_t spiClockHz,
    uint8_t channelCount,
    bool doHardwareReset
) {
    pinDRDY = drdyPin;
    pinCS = csPin;
    pinSTART = startPin;
    pinRESET = resetPin;
    pinPWDN = pwdnPin;
    activeChannels = channelCount;
    if (activeChannels < 1) activeChannels = 1;
    if (activeChannels > 8) activeChannels = 8;
    spiSettings = SPISettings(spiClockHz, MSBFIRST, SPI_MODE1);

    pinMode(pinDRDY, INPUT);
    pinMode(pinCS, OUTPUT);
    digitalWrite(pinCS, HIGH);

    if (doHardwareReset) {
        pinMode(pinSTART, OUTPUT);
        pinMode(pinRESET, OUTPUT);
        pinMode(pinPWDN, OUTPUT);

        digitalWrite(pinSTART, LOW);
        digitalWrite(pinPWDN, HIGH);
        digitalWrite(pinRESET, HIGH);
        delay(10);

        digitalWrite(pinPWDN, LOW);
        delay(5);
        digitalWrite(pinPWDN, HIGH);
        delay(20);

        digitalWrite(pinRESET, LOW);
        delay(2);
        digitalWrite(pinRESET, HIGH);
        delay(20);
    }

    if (!sharedSpiInitialized) {
        sharedSpi.begin(sclkPin, misoPin, mosiPin, pinCS);
        sharedSpiInitialized = true;
    }

    SDATAC();
    delay(1);

    lastDrdyLevel = digitalRead(pinDRDY);
    running = false;

    deviceId = readRegister(ID);
    switch (deviceId & 0x03) {
        case 0x00:
            outputChannels = 4;
            break;
        case 0x01:
            outputChannels = 6;
            break;
        case 0x02:
            outputChannels = 8;
            break;
        default:
            outputChannels = activeChannels;
            break;
    }

    if (outputChannels < activeChannels) {
        outputChannels = activeChannels;
    }

    return (deviceId != 0x00) && (deviceId != 0xFF);
}

void ADS1299::beginConfigTransaction(bool& wasRunning) {
    wasRunning = running;
    if (wasRunning) {
        SDATAC();
        running = false;
    } else {
        SDATAC();
    }
}

void ADS1299::endConfigTransaction(bool wasRunning) {
    if (wasRunning) {
        RDATAC();
        running = true;
    }
}

void ADS1299::csLow() {
    digitalWrite(pinCS, LOW);
}

void ADS1299::csHigh() {
    digitalWrite(pinCS, HIGH);
}

uint8_t ADS1299::transfer(uint8_t data) {
    return sharedSpi.transfer(data);
}

void ADS1299::sendCommand(uint8_t command) {
    sharedSpi.beginTransaction(spiSettings);
    csLow();
    transfer(command);
    delayMicroseconds(2);   // tSCCS: 4 tCLK before CS HIGH
    csHigh();
    sharedSpi.endTransaction();
    delayMicroseconds(4);   // tSDECODE: command decode time
}

void ADS1299::WAKEUP() { sendCommand(_WAKEUP); }
void ADS1299::STANDBY() { sendCommand(_STANDBY); }
void ADS1299::RESET() { sendCommand(_RESET); delay(2); }
void ADS1299::START() { sendCommand(_START); }
void ADS1299::STOP() { sendCommand(_STOP); }
void ADS1299::RDATAC() { sendCommand(_RDATAC); }
void ADS1299::SDATAC() { sendCommand(_SDATAC); }
void ADS1299::RDATA() { sendCommand(_RDATA); }

uint8_t ADS1299::readRegister(uint8_t address) {
    uint8_t value = 0;
    sharedSpi.beginTransaction(spiSettings);
    csLow();
    transfer(_RREG | (address & 0x1F));
    delayMicroseconds(2);
    transfer(0x00);
    delayMicroseconds(2);
    value = transfer(0x00);
    delayMicroseconds(2);   // tSCCS
    csHigh();
    sharedSpi.endTransaction();
    return value;
}

void ADS1299::readRegisters(uint8_t startAddress, uint8_t count, uint8_t* data) {
    if (count == 0 || data == nullptr) {
        return;
    }

    sharedSpi.beginTransaction(spiSettings);
    csLow();
    transfer(_RREG | (startAddress & 0x1F));
    delayMicroseconds(2);
    transfer(count - 1);
    delayMicroseconds(2);
    for (uint8_t i = 0; i < count; ++i) {
        data[i] = transfer(0x00);
    }
    delayMicroseconds(2);   // tSCCS
    csHigh();
    sharedSpi.endTransaction();
}

void ADS1299::writeRegister(uint8_t address, uint8_t value) {
    sharedSpi.beginTransaction(spiSettings);
    csLow();
    transfer(_WREG | (address & 0x1F));
    delayMicroseconds(2);
    transfer(0x00);
    delayMicroseconds(2);
    transfer(value);
    delayMicroseconds(2);   // tSCCS
    csHigh();
    sharedSpi.endTransaction();
    delayMicroseconds(3);
}

bool ADS1299::configureBasicEEG() {
    return applyCytonDefaults(0x06);
}

bool ADS1299::configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode) {
    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    amplitudeCode &= 0x04;
    freqCode &= 0x03;
    uint8_t setting = static_cast<uint8_t>(0xD0 | amplitudeCode | freqCode);
    writeRegister(CONFIG2, setting);

    const bool ok = readRegister(CONFIG2) == setting;
    endConfigTransaction(wasRunning);
    return ok;
}

bool ADS1299::applyCytonDefaults(uint8_t dataRateBits) {
    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    config1Cache = static_cast<uint8_t>(0x90 | (dataRateBits & 0x07));
    writeRegister(CONFIG1, config1Cache);
    writeRegister(CONFIG2, 0xC0);
    writeRegister(CONFIG3, 0xEC);

    biasSenseP = 0;
    biasSenseN = 0;
    srb1Mask = 0;
    leadOffConfig = kLeadOffFreq31Hz;
    leadOffSenseP = 0;
    leadOffSenseN = 0;

    for (uint8_t ch = 0; ch < outputChannels && ch < 8; ++ch) {
        if (ch < activeChannels) {
            channelSettings[ch] = 0x68;
            biasSenseP |= static_cast<uint8_t>(1U << ch);
            biasSenseN |= static_cast<uint8_t>(1U << ch);
        } else {
            channelSettings[ch] = static_cast<uint8_t>(kPowerDownMask | 0x61);
        }
        writeRegister(CH1SET + ch, channelSettings[ch]);
    }

    writeRegister(BIAS_SENSP, biasSenseP);
    writeRegister(BIAS_SENSN, biasSenseN);
    writeRegister(LOFF, leadOffConfig);
    writeRegister(LOFF_SENSP, leadOffSenseP);
    writeRegister(LOFF_SENSN, leadOffSenseN);
    writeRegister(MISC1, 0x00);

    gain = 24;
    const bool ok = (readRegister(CONFIG1) == config1Cache) && ((readRegister(CONFIG3) & 0xE0) == 0xE0);
    endConfigTransaction(wasRunning);
    return ok;
}

bool ADS1299::setDataRateBits(uint8_t dataRateBits) {
    bool wasRunning = false;
    beginConfigTransaction(wasRunning);
    config1Cache = static_cast<uint8_t>((config1Cache & 0xF8) | (dataRateBits & 0x07));
    writeRegister(CONFIG1, config1Cache);
    const bool ok = (readRegister(CONFIG1) & 0x07) == (config1Cache & 0x07);
    endConfigTransaction(wasRunning);
    return ok;
}

bool ADS1299::setInputTypeForAllChannels(uint8_t inputTypeOrdinal) {
    if (inputTypeOrdinal > 7) {
        inputTypeOrdinal = 0;
    }

    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    for (uint8_t ch = 0; ch < activeChannels; ++ch) {
        uint8_t chset = channelSettings[ch];
        if ((chset & kPowerDownMask) != 0) {
            continue;
        }
        chset = static_cast<uint8_t>((chset & static_cast<uint8_t>(~kInputMask)) | inputTypeOrdinal);
        channelSettings[ch] = chset;
        writeRegister(CH1SET + ch, chset);
    }

    endConfigTransaction(wasRunning);
    return true;
}

bool ADS1299::configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode) {
    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    leadOffConfig = static_cast<uint8_t>((amplitudeCode & 0x0C) | (freqCode & 0x03));
    writeRegister(LOFF, leadOffConfig);

    const bool ok = readRegister(LOFF) == leadOffConfig;
    endConfigTransaction(wasRunning);
    return ok;
}

bool ADS1299::setLeadOffForChannel(uint8_t channelIndex, bool pEnable, bool nEnable) {
    if (channelIndex >= activeChannels) {
        return false;
    }

    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    uint8_t mask = static_cast<uint8_t>(1U << channelIndex);
    if (pEnable) {
        leadOffSenseP |= mask;
    } else {
        leadOffSenseP &= static_cast<uint8_t>(~mask);
    }

    if (nEnable) {
        leadOffSenseN |= mask;
    } else {
        leadOffSenseN &= static_cast<uint8_t>(~mask);
    }

    writeRegister(LOFF_SENSP, leadOffSenseP);
    writeRegister(LOFF_SENSN, leadOffSenseN);

    endConfigTransaction(wasRunning);
    return true;
}

bool ADS1299::configureChannel(
    uint8_t channelIndex,
    bool active,
    uint8_t gainOrdinal,
    uint8_t inputTypeOrdinal,
    bool biasInclude,
    bool srb2Connect,
    bool srb1Connect
) {
    if (channelIndex >= activeChannels) {
        return false;
    }

    static const uint8_t gainLut[7] = {0, 1, 2, 3, 4, 5, 6};
    static const uint8_t gainScalarLut[7] = {1, 2, 4, 6, 8, 12, 24};
    if (gainOrdinal > 6) {
        gainOrdinal = 6;
    }
    if (inputTypeOrdinal > 7) {
        inputTypeOrdinal = 0;
    }

    bool wasRunning = false;
    beginConfigTransaction(wasRunning);

    uint8_t chset = 0;
    if (!active) {
        chset |= kPowerDownMask;
        inputTypeOrdinal = 1;
        biasInclude = false;
        srb2Connect = false;
        srb1Connect = false;
    }

    chset |= static_cast<uint8_t>((gainLut[gainOrdinal] & 0x07) << 4);
    if (srb2Connect) {
        chset |= kSrb2Mask;
    }
    chset |= static_cast<uint8_t>(inputTypeOrdinal & 0x07);

    channelSettings[channelIndex] = chset;
    writeRegister(CH1SET + channelIndex, chset);

    uint8_t mask = static_cast<uint8_t>(1U << channelIndex);
    if (biasInclude) {
        biasSenseP |= mask;
        biasSenseN |= mask;
    } else {
        biasSenseP &= static_cast<uint8_t>(~mask);
        biasSenseN &= static_cast<uint8_t>(~mask);
    }

    if (srb1Connect) {
        srb1Mask |= mask;
    } else {
        srb1Mask &= static_cast<uint8_t>(~mask);
    }

    writeRegister(BIAS_SENSP, biasSenseP);
    writeRegister(BIAS_SENSN, biasSenseN);
    writeRegister(MISC1, (srb1Mask != 0) ? 0x20 : 0x00);

    if (!active) {
        leadOffSenseP &= static_cast<uint8_t>(~mask);
        leadOffSenseN &= static_cast<uint8_t>(~mask);
        writeRegister(LOFF_SENSP, leadOffSenseP);
        writeRegister(LOFF_SENSN, leadOffSenseN);
    }

    gain = gainScalarLut[gainOrdinal];
    uint8_t readBack = readRegister(CH1SET + channelIndex);
    const bool ok = readBack == chset;
    endConfigTransaction(wasRunning);
    return ok;
}

bool ADS1299::startContinuousConversion() {
    // Only enter RDATAC mode — caller manages START pin
    RDATAC();
    running = true;
    return true;
}

bool ADS1299::stopContinuousConversion() {
    // Only exit RDATAC mode — caller manages START pin
    SDATAC();
    running = false;
    return true;
}

bool ADS1299::waitForDRDY(uint32_t timeoutMs) {
    uint32_t startMs = millis();

    while (true) {
        int currentLevel = digitalRead(pinDRDY);
        if (currentLevel == LOW) {
            lastDrdyLevel = currentLevel;
            return true;
        }
        lastDrdyLevel = currentLevel;

        if ((millis() - startMs) >= timeoutMs) {
            return false;
        }
        delayMicroseconds(10);
    }
}

bool ADS1299::readDataFrame(int32_t channels[8], uint32_t& status) {
    if (channels == nullptr) {
        return false;
    }

    sharedSpi.beginTransaction(spiSettings);
    csLow();

    status = 0;
    for (int i = 0; i < 3; ++i) {
        status = (status << 8) | transfer(0x00);
    }

    for (uint8_t ch = 0; ch < outputChannels && ch < 8; ++ch) {
        uint32_t raw = 0;
        raw = (raw << 8) | transfer(0x00);
        raw = (raw << 8) | transfer(0x00);
        raw = (raw << 8) | transfer(0x00);

        if (raw & 0x00800000UL) {
            raw |= 0xFF000000UL;
        }
        if (ch < activeChannels) {
            channels[ch] = static_cast<int32_t>(raw);
        }
    }

    for (uint8_t ch = activeChannels; ch < 8; ++ch) {
        channels[ch] = 0;
    }

    delayMicroseconds(2);   // tSCCS: 4 tCLK before CS HIGH
    csHigh();
    sharedSpi.endTransaction();

    return true;
}

int8_t ADS1299::getDRDYPin() const {
    return pinDRDY;
}

bool ADS1299::isRunning() const {
    return running;
}

uint8_t ADS1299::getDeviceId() const {
    return deviceId;
}

uint8_t ADS1299::getOutputChannelCount() const {
    return outputChannels;
}

void ADS1299::printRegisters(Stream& stream, const char* label) {
    if (label != nullptr && label[0] != '\0') {
        stream.println(label);
    }

    uint8_t regs[CONFIG4 + 1] = {0};
    readRegisters(0x00, CONFIG4 + 1, regs);

    for (uint8_t address = 0; address <= CONFIG4; ++address) {
        char line[48];
        snprintf(line, sizeof(line), "0x%02X, 0x%02X", address, regs[address]);
        stream.println(line);
    }
}

float ADS1299::countsToVolts(int32_t counts) const {
    return static_cast<float>(counts) * (vref / (static_cast<float>(gain) * 8388607.0f));
}
