//
//  ADS1299.cpp
//

#include "ADS1299.h"

namespace {
SPIClass sharedSpi(VSPI);
bool sharedSpiInitialized = false;
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
        activeChannels(8),
        lastDrdyLevel(HIGH) {}

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
    uint8_t channelCount
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

    pinMode(pinDRDY, INPUT_PULLUP);
    pinMode(pinCS, OUTPUT);
    pinMode(pinSTART, OUTPUT);
    pinMode(pinRESET, OUTPUT);
    pinMode(pinPWDN, OUTPUT);

    digitalWrite(pinCS, HIGH);
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

    if (!sharedSpiInitialized) {
        sharedSpi.begin(sclkPin, misoPin, mosiPin, pinCS);
        sharedSpiInitialized = true;
    }

    SDATAC();
    delay(1);

    lastDrdyLevel = digitalRead(pinDRDY);

    uint8_t id = readRegister(ID);
    return (id != 0x00) && (id != 0xFF);
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
    csHigh();
    sharedSpi.endTransaction();
    delayMicroseconds(4);
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
    csHigh();
    sharedSpi.endTransaction();
    delayMicroseconds(3);
}

bool ADS1299::configureBasicEEG() {
    SDATAC();
    STOP();

    writeRegister(CONFIG1, 0x96);
    writeRegister(CONFIG2, 0xC0);
    writeRegister(CONFIG3, 0xEC);

    for (uint8_t ch = 0; ch < activeChannels; ++ch) {
        writeRegister(CH1SET + ch, 0x60);
    }

    gain = 24;
    return (readRegister(CONFIG1) == 0x96) && ((readRegister(CONFIG3) & 0xE0) == 0xE0);
}

bool ADS1299::configureInternalTestSignal() {
    SDATAC();
    STOP();

    writeRegister(CONFIG1, 0x96);
    writeRegister(CONFIG2, 0xD3);
    writeRegister(CONFIG3, 0xEC);

    for (uint8_t ch = 0; ch < activeChannels; ++ch) {
        writeRegister(CH1SET + ch, 0x65);
    }

    gain = 24;
    return (readRegister(CONFIG2) == 0xD3) && (readRegister(CH1SET) == 0x65);
}

bool ADS1299::startContinuousConversion() {
    RDATAC();
    delay(1);
    digitalWrite(pinSTART, HIGH);
    START();
    return true;
}

bool ADS1299::stopContinuousConversion() {
    digitalWrite(pinSTART, LOW);
    STOP();
    SDATAC();
    return true;
}

bool ADS1299::waitForDRDY(uint32_t timeoutMs) {
    uint32_t startMs = millis();

    while (true) {
        int currentLevel = digitalRead(pinDRDY);
        if (lastDrdyLevel == HIGH && currentLevel == LOW) {
            lastDrdyLevel = currentLevel;
            return true;
        }
        lastDrdyLevel = currentLevel;

        if ((millis() - startMs) >= timeoutMs) {
            return false;
        }
        delayMicroseconds(20);
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

    for (uint8_t ch = 0; ch < activeChannels; ++ch) {
        uint32_t raw = 0;
        raw = (raw << 8) | transfer(0x00);
        raw = (raw << 8) | transfer(0x00);
        raw = (raw << 8) | transfer(0x00);

        if (raw & 0x00800000UL) {
            raw |= 0xFF000000UL;
        }
        channels[ch] = static_cast<int32_t>(raw);
    }

    for (uint8_t ch = activeChannels; ch < 8; ++ch) {
        channels[ch] = 0;
    }

    csHigh();
    sharedSpi.endTransaction();

    return true;
}

int8_t ADS1299::getDRDYPin() const {
    return pinDRDY;
}

float ADS1299::countsToVolts(int32_t counts) const {
    return static_cast<float>(counts) * (vref / (static_cast<float>(gain) * 8388607.0f));
}
