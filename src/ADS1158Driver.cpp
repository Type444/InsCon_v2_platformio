#include "ADS1158Driver.h"
#include "ADS1158Registers.h"
#include <Arduino.h>


ADS1158::ADS1158(uint8_t cs_pin) : cs_pin(cs_pin) {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

void ADS1158::begin() {
    reset();
    delay(100);
    uint8_t deviceID = readRegister(ID_REG);
    PRINT_DEBUG_VAL("[DEBUG] Device ID: 0x", deviceID);

    if (deviceID != ID_DEFAULT) {
        PRINT_DEBUG("[ERROR] Device ID Mismatch!");
    } else {
        PRINT_DEBUG("[DEBUG] Device ID Verified.");
    }
}

void ADS1158::reset() {
    PRINT_DEBUG("[DEBUG] Resetting ADS1158...");
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x60);  // Reset Command
    digitalWrite(cs_pin, HIGH);

    writeRegister(CONFIG0_REG, 0b00000010);
    writeRegister(CONFIG1_REG, 0b01010000);
    writeRegister(MUXDIF_REG, 0b00000000);
    writeRegister(MUXSCH_REG, 0b00000000);
    writeRegister(MUXSG0_REG, 0b11111111);
    writeRegister(MUXSG1_REG, 0b01111100);
    writeRegister(SYSRED_REG, 0b00000000);
    writeRegister(GPIOC_REG, 0b00000000);
    writeRegister(GPIOD_REG, 0b00000000);
    PRINT_DEBUG("[DEBUG] ADS1158 Reset Complete.");
}

void ADS1158::setFixedChannel(uint8_t channel) {
    if (channel > 15) {
        PRINT_DEBUG("[ERROR] Invalid channel number. Must be between 0 and 15.");
        return;
    }
    if (channel <= 7) {
        writeRegister(MUXSCH_REG, channel);
    } else {
        uint8_t muxsgValue = 1 << (channel - 8);
        writeRegister(MUXSG1_REG, muxsgValue);
    }
    PRINT_DEBUG("[DEBUG] Fixed-Channel Mode configured for AIN" + String(channel));
}

int16_t ADS1158::readData(uint8_t &channel, bool &newData, bool &overflow, bool &lowSupply, bool statusByteEnabled) {
    int16_t data = 0;
    uint8_t status = 0;
    digitalWrite(cs_pin, LOW);
    if (statusByteEnabled) {
        status = SPI.transfer(0x00);
    }
    uint8_t data1 = SPI.transfer(0x00);
    uint8_t data2 = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    data = ((int16_t)data1 << 8) | data2;
    if (statusByteEnabled) {
        newData = (status & 0x80) != 0;
        overflow = (status & 0x40) != 0;
        lowSupply = (status & 0x20) != 0;
        channel = (status & 0x1F) - 8;
        if (overflow) {
            PRINT_DEBUG("[ERROR] OVF bit active, overflow!");
            delay(2000);
        }
        if (lowSupply) {
            PRINT_DEBUG("[ERROR] SUPPLY bit active, low supply voltage!");
            delay(2000);
        }
    } else {
        newData = true;
        overflow = false;
        lowSupply = false;
        channel = 0;
    }
    return data;
}

void ADS1158::setAutoScanMode(uint16_t channels) {
    PRINT_DEBUG("[DEBUG] Configuring Auto-Scan Mode...");
    uint8_t muxsg0Value = channels & 0xFF;
    uint8_t muxsg1Value = (channels >> 8) & 0xFF;
    writeRegister(CONFIG0_REG, CONFIG0_STAT);
    setStatusByteEnabled(true);
    setConversionDelay(TimeDelay::DLY_128);
    setDataRate(DataRate::DR_1831_SPS);
    writeRegister(MUXSG0_REG, muxsg0Value);
    writeRegister(MUXSG1_REG, muxsg1Value);
    Serial.print("[DEBUG] Enabled channels: ");
    bool first = true;
    for (int i = 0; i < 16; i++) {
        if (channels & (1 << i)) {
            if (!first) {
                Serial.print(", ");
            }
            Serial.print(i);
            first = false;
        }
    }
    Serial.println();
    PRINT_DEBUG("[DEBUG] Auto-Scan Mode configured for channels 0x" + String(channels, HEX));
}

void ADS1158::pinModeGPIO(uint8_t pin, bool state) {
    PRINT_DEBUG("[DEBUG] Configuring GPIO...");
    uint8_t gpiodValue = readRegister(GPIOD_REG);
    if (state) {
        gpiodValue |= (1 << pin);
    } else {
        gpiodValue &= ~(1 << pin);
    }
    writeRegister(GPIOD_REG, gpiodValue);
    PRINT_DEBUG("[DEBUG] GPIO Pin " + String(pin) + " set to " + (state ? "OUTPUT" : "INPUT") + ".");
}

void ADS1158::digitalWriteGPIO(uint8_t pin, bool value) {
    PRINT_DEBUG("[DEBUG] Setting GPIO State...");
    uint8_t gpiocValue = readRegister(GPIOC_REG);
    if (value) {
        gpiocValue |= (1 << pin);
    } else {
        gpiocValue &= ~(1 << pin);
    }
    writeRegister(GPIOC_REG, gpiocValue);
    PRINT_DEBUG("[DEBUG] GPIO Pin " + String(pin) + " set to " + (value ? "HIGH" : "LOW") + ".");
}

void ADS1158::printRegisters() {
    Serial.println("Configuration Registers:");
    uint8_t config0 = readRegister(CONFIG0_REG);
    Serial.print("CONFIG0: 0x");
    Serial.print(config0, HEX);
    Serial.print(" (Binary: ");
    printBinary(config0);
    Serial.println(")");
    uint8_t config1 = readRegister(CONFIG1_REG);
    Serial.print("CONFIG1: 0x");
    Serial.print(config1, HEX);
    Serial.print(" (Binary: ");
    printBinary(config1);
    Serial.println(")");
    uint8_t muxsg0 = readRegister(MUXSG0_REG);
    Serial.print("MUXSG0: 0x");
    Serial.print(muxsg0, HEX);
    Serial.print(" (Binary: ");
    printBinary(muxsg0);
    Serial.println(")");
    uint8_t muxsg1 = readRegister(MUXSG1_REG);
    Serial.print("MUXSG1: 0x");
    Serial.print(muxsg1, HEX);
    Serial.print(" (Binary: ");
    printBinary(muxsg1);
    Serial.println(")");
    uint8_t muxsch = readRegister(MUXSCH_REG);
    Serial.print("MUXSCH: 0x");
    Serial.print(muxsch, HEX);
    Serial.print(" (Binary: ");
    printBinary(muxsch);
    Serial.println(")");
    uint8_t muxdif = readRegister(MUXDIF_REG);
    Serial.print("MUXDIF: 0x");
    Serial.print(muxdif, HEX);
    Serial.print(" (Binary: ");
    printBinary(muxdif);
    Serial.println(")");
    uint8_t sysred = readRegister(SYSRED_REG);
    Serial.print("SYSRED: 0x");
    Serial.print(sysred, HEX);
    Serial.print(" (Binary: ");
    printBinary(sysred);
    Serial.println(")");
    uint8_t gpioc = readRegister(GPIOC_REG);
    Serial.print("GPIOC: 0x");
    Serial.print(gpioc, HEX);
    Serial.print(" (Binary: ");
    printBinary(gpioc);
    Serial.println(")");
    uint8_t gpiod = readRegister(GPIOD_REG);
    Serial.print("GPIOD: 0x");
    Serial.print(gpiod, HEX);
    Serial.print(" (Binary: ");
    printBinary(gpiod);
    Serial.println(")");
    uint8_t deviceID = readRegister(ID_REG);
    Serial.print("Device ID: 0x");
    Serial.print(deviceID, HEX);
    Serial.print(" (Binary: ");
    printBinary(deviceID);
    Serial.println(")");
}

void ADS1158::configureSpiResetTimer(bool enable) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = enable ? (config0 | (1 << 6)) : (config0 & ~(1 << 6));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::setMultiplexerMode(bool isFixedChannelMode) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = isFixedChannelMode ? (config0 | (1 << 5)) : (config0 & ~(1 << 5));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::setMuxBypass(bool useExternalMux) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = useExternalMux ? (config0 | (1 << 4)) : (config0 & ~(1 << 4));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::setClockOutputEnabled(bool enable) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = enable ? (config0 | (1 << 3)) : (config0 & ~(1 << 3));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::configureChopper(bool enable) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = enable ? (config0 | (1 << 2)) : (config0 & ~(1 << 2));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::setStatusByteEnabled(bool enable) {
    uint8_t config0 = readRegister(CONFIG0_REG);
    config0 = enable ? (config0 | (1 << 1)) : (config0 & ~(1 << 1));
    writeRegister(CONFIG0_REG, config0);
}

void ADS1158::setIdleMode(bool lowPowerMode) {
    uint8_t config1 = readRegister(CONFIG1_REG);
    config1 = lowPowerMode ? (config1 | (1 << 7)) : (config1 & ~(1 << 7));
    writeRegister(CONFIG1_REG, config1);
}

void ADS1158::setConversionDelay(uint8_t delaySetting) {
    uint8_t config1 = readRegister(CONFIG1_REG);
    config1 = (config1 & ~(0x70)) | (delaySetting << 4);
    writeRegister(CONFIG1_REG, config1);
}

void ADS1158::setBiasSenseCurrent(uint8_t currentSetting) {
    uint8_t config1 = readRegister(CONFIG1_REG);
    config1 = (config1 & ~(0x0C)) | (currentSetting << 2);
    writeRegister(CONFIG1_REG, config1);
}

void ADS1158::setDataRate(uint8_t dataRateSetting) {
    uint8_t config1 = readRegister(CONFIG1_REG);
    config1 = (config1 & ~(0x03)) | dataRateSetting;
    writeRegister(CONFIG1_REG, config1);
}

void ADS1158::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t command = 0x60 | (reg & 0x0F);  // Register Write command
    digitalWrite(cs_pin, LOW);
    SPI.transfer(command);
    SPI.transfer(value);
    digitalWrite(cs_pin, HIGH);
}

uint8_t ADS1158::readRegister(uint8_t reg) {
    uint8_t command = 0x40 | (reg & 0x0F);  // Register Read command
    digitalWrite(cs_pin, LOW);
    SPI.transfer(command);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    return value;
}

void ADS1158::printBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        Serial.print((value >> i) & 0x01);
    }
}
