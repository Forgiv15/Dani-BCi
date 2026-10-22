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
        uint8_t channelCount = 8,
        bool pulseControlPins = true
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

    // Diagnostics: returns the exact bytes clocked in during an RREG command
    // (trace[0] = during opcode, trace[1] = during register count,
    // trace[2] = during the data byte) without any validation.
    void rawReadRegister(uint8_t address, uint8_t trace[3]);
    // Clocks one byte while CS stays deasserted, sampling the MISO level that
    // nothing on the bus is driving.
    uint8_t probeIdleMiso();

    bool configureBasicEEG();
    bool configureInternalTestSignal(uint8_t amplitudeCode = 0x04, uint8_t freqCode = 0x03);
    bool applyCytonDefaults(uint8_t dataRateBits = 0x06);
    bool setDataRateBits(uint8_t dataRateBits);

    // CONFIG1 bit 5. Required on the device whose CLKSEL pin is high so its
    // internal oscillator is driven out on the CLK pin (clock master for the
    // second ADS1299, whose CLKSEL pin is low).
    void setClockOutput(bool enable);
    bool isClockOutputEnabled() const;
    uint8_t config1Value() const;
    bool verifyClockOutput(bool expected);
    // Writes CONFIG1 with CLK_EN set right away, so the clock is available to
    // a slave device before that slave is probed.
    bool enableClockOutput();
    // Lightweight presence check: SDATAC followed by an ID register read.
    bool detect();
    bool setInputTypeForAllChannels(uint8_t inputTypeOrdinal);
    bool configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode);
    bool setLeadOffForChannel(uint8_t channelIndex, bool pEnable, bool nEnable);
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
    bool isRunning() const;
    void printRegisters(Stream& stream, const char* label = nullptr);
    float countsToVolts(int32_t counts) const;

private:
    void beginConfigTransaction(bool& wasRunning);
    void endConfigTransaction(bool wasRunning);
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
    int8_t pinMISO;

    float vref;
    uint8_t gain;
    uint8_t activeChannels;
    uint8_t config1Cache;
    bool clockOutput;
    uint8_t channelSettings[8];
    uint8_t biasSenseP;
    uint8_t biasSenseN;
    uint8_t srb1Mask;
    uint8_t leadOffConfig;
    uint8_t leadOffSenseP;
    uint8_t leadOffSenseN;
    int lastDrdyLevel;
    bool running;
};

#endif