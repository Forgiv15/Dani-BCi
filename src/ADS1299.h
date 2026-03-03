//
//  ADS1299.h
//

#ifndef ____ADS1299__
#define ____ADS1299__

#include <Arduino.h>
#include <SPI.h>
#include "Definitions.h"

class ADS1299 {
public:
    ADS1299();

    bool begin(
        int8_t drdyPin,
        int8_t csPin,
        int8_t startPin,
        int8_t resetPin,
        int8_t pwdnPin,
        int8_t sclkPin,
        int8_t misoPin,
        int8_t mosiPin,
        uint32_t spiClockHz = 1000000,
        uint8_t channelCount = 8
    );

    void WAKEUP();
    void STANDBY();
    void RESET();
    void START();
    void STOP();
    void RDATAC();
    void SDATAC();
    void RDATA();

    uint8_t readRegister(uint8_t address);
    void readRegisters(uint8_t startAddress, uint8_t count, uint8_t* data);
    void writeRegister(uint8_t address, uint8_t value);

    bool configureBasicEEG();
    bool configureInternalTestSignal();
    bool applyCytonDefaults(uint8_t dataRateBits = 0x06);
    bool setDataRateBits(uint8_t dataRateBits);
    bool configureChannel(
        uint8_t channelIndex,
        bool active,
        uint8_t gainOrdinal,
        uint8_t inputTypeOrdinal,
        bool biasInclude,
        bool srb2Connect,
        bool srb1Connect
    );
    bool startContinuousConversion();
    bool stopContinuousConversion();

    bool waitForDRDY(uint32_t timeoutMs);
    bool readDataFrame(int32_t channels[8], uint32_t& status);

    int8_t getDRDYPin() const;
    float countsToVolts(int32_t counts) const;

private:
    void sendCommand(uint8_t command);
    void csLow();
    void csHigh();
    uint8_t transfer(uint8_t data);

    SPISettings spiSettings;

    int8_t pinDRDY;
    int8_t pinCS;
    int8_t pinSTART;
    int8_t pinRESET;
    int8_t pinPWDN;

    float vref;
    uint8_t gain;
    uint8_t activeChannels;
    uint8_t config1Cache;
    uint8_t channelSettings[8];
    uint8_t biasSenseP;
    uint8_t biasSenseN;
    uint8_t srb1Mask;
    int lastDrdyLevel;
};

#endif