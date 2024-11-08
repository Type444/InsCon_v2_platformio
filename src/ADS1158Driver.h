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

// ADS1158 functions
void ADS1158_init();
void ADS1158_reset();
void ADS1158_setFixedChannel(uint8_t channel);
int16_t ADS1158_readData(uint8_t &channel, bool &newData, bool &overflow, bool &lowSupply, bool statusByteEnabled);
void ADS1158_setAutoScanMode(uint16_t channels);
void ADS1158_pinmodeGPIO(uint8_t pin, bool state);
void ADS1158_digitalWriteGPIO(uint8_t pin, bool value);
uint8_t ADS1158_readRegister(uint8_t reg);
void ADS1158_writeRegister(uint8_t reg, uint8_t value);
void ADS1158_printRegisters();
void ADS1158_configureSpiResetTimer(bool enable);
void ADS1158_setMultiplexerMode(bool isFixedChannelMode);
void ADS1158_setMuxBypass(bool useExternalMux);
void ADS1158_setClockOutputEnabled(bool enable);
void ADS1158_configureChopper(bool enable);
void ADS1158_setStatusByteEnabled(bool enable);
void ADS1158_setIdleMode(bool lowPowerMode);
void ADS1158_setConversionDelay(uint8_t delaySetting);
void ADS1158_setBiasSenseCurrent(uint8_t currentSetting);
void ADS1158_setDataRate(uint8_t dataRateSetting);

#endif  // ADS1158_DRIVER_H
