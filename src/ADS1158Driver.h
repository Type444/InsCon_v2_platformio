#ifndef ADS1158_DRIVER_H
#define ADS1158_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#include "Config.h"

enum TimeDelay {
  DLY_0 = 0b000,    // 0 periods (0 μS)
  DLY_8 = 0b001,    // 1 period (8 μS)
  DLY_16 = 0b010,   // 2 periods (16 μS)
  DLY_32 = 0b011,   // 4 periods (32 μS)
  DLY_64 = 0b100,   // 8 periods (64 μS)
  DLY_128 = 0b101,  // 16 periods (128 μS)
  DLY_256 = 0b110,  // 32 periods (256 μS)
  DLY_384 = 0b111   // 48 periods (384 μS)
};

enum DataRate {
  DR_23739_SPS = 0b11,  // 23739 SPS (Samples per Second)
  DR_15123_SPS = 0b10,  // 15123 SPS
  DR_6168_SPS = 0b01,   // 6168 SPS
  DR_1831_SPS = 0b11    // 1831 SPS
};

class ADS1158 {
public:
    ADS1158(uint8_t cs_pin);
    void begin();
    void reset();
    void setFixedChannel(uint8_t channel);
    int16_t readData(uint8_t &channel, bool &newData, bool &overflow, bool &lowSupply, bool statusByteEnabled);
    void setAutoScanMode(uint16_t channels);
    void pinModeGPIO(uint8_t pin, bool state);
    void digitalWriteGPIO(uint8_t pin, bool value);
    void printRegisters();
    void configureSpiResetTimer(bool enable);
    void setMultiplexerMode(bool isFixedChannelMode);
    void setMuxBypass(bool useExternalMux);
    void setClockOutputEnabled(bool enable);
    void configureChopper(bool enable);
    void setStatusByteEnabled(bool enable);
    void setIdleMode(bool lowPowerMode);
    void setConversionDelay(uint8_t delaySetting);
    void setBiasSenseCurrent(uint8_t currentSetting);
    void setDataRate(uint8_t dataRateSetting);

private:
    uint8_t cs_pin;
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void printBinary(uint8_t value);
};


#endif  // ADS1158_DRIVER_H
