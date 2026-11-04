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
        uint32_t spiClockHz = 2000000,
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

    // Batched frame reads.
    //
    // Both converters share the bus and use identical SPISettings, so locking the
    // mutex and re-applying the transaction once per chip is pure overhead on the
    // conversion critical path. These let a caller bracket the whole pair with one
    // lock: begin once, read each device, end once.
    //
    // The CS pulses stay strictly sequential. Both devices drive the same MISO line,
    // so asserting two chip selects together would put two drivers on it.
    //
    // Not static: the transaction uses this instance's SPISettings. Every instance on
    // the bus holds identical settings (setSpiClock is applied to all of them), so
    // whichever one brackets the batch is authoritative.
    void beginSharedTransaction();
    void endSharedTransaction();
    bool readDataFrameLocked(int32_t channels[8], uint32_t& status);

    // Writes a register and reads it back, retrying a few times. Returns false only
    // when the value still does not match after every attempt.
    //
    // This exists because a WREG that is clocked marginally - which is what a long
    // shared SCLK at 4 MHz and above produces - can land a *wrong value* rather than
    // none at all. A wrong data-rate bit reads back as a plausible slower rate and a
    // wrong CHnSET reads back as a plausible different montage, so neither is visible
    // without a readback. Callers must not be in RDATAC: the readback is an RREG.
    bool writeRegisterVerified(uint8_t address, uint8_t value, uint8_t attempts = 3);

    // Number of verified writes that never matched, and the last register that
    // failed. Configuration code reports these so "the settings did not apply" has a
    // name and a count instead of being inferred from the data.
    uint32_t verifyFailureCount() const;
    uint8_t lastVerifyFailureRegister() const;
    void clearVerifyFailures();

    // Diagnostics: returns the exact bytes clocked in during an RREG command
    // (trace[0] = during opcode, trace[1] = during register count,
    // trace[2] = during the data byte) without any validation.
    void rawReadRegister(uint8_t address, uint8_t trace[3]);
    // Clocks one byte while CS stays deasserted, sampling the MISO level that
    // nothing on the bus is driving.
    uint8_t probeIdleMiso();

    // SCLK is shared by both converters, so a rate change has to be applied to
    // every instance. 20 MHz is the ESP32 ceiling for this bus; the ADS1299
    // datasheet allows register access and frame reads well beyond that.
    void setSpiClock(uint32_t spiClockHz);
    uint32_t spiClockHz() const;

    bool configureBasicEEG();
    bool configureInternalTestSignal(uint8_t amplitudeCode = 0x04, uint8_t freqCode = 0x03,
                                     bool enable = true);
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

    // SRB1 is one switch for the whole chip (MISC1 bit 5): it ties the negative
    // input of every channel to the SRB1 pin. SRB2 is the per-channel switch
    // (CHnSET bit 3). Both are driven from a single board-wide mode so an
    // inconsistent combination cannot be programmed.
    bool setSrb1Routing(bool enable);
    bool setSrb2ForAllChannels(bool enable);
    uint8_t misc1Value() const;
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
    uint32_t spiClock;

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
    uint32_t verifyFailures = 0;
    uint8_t lastVerifyFailRegister = 0;    bool clockOutput;
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