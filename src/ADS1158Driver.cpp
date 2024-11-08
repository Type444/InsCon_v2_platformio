#include <Arduino.h>
#include <Config.h>
#include <ADS1158Registers.h>
#include <ADS1158Driver.h>

// ADS1158 Initialization
void ADS1158_init() {
  // Reset ADS1158
  ADS1158_reset();
  delay(100);

  // Check device ID
  uint8_t deviceID = ADS1158_readRegister(ID_REG);
  PRINT_DEBUG_VAL("[DEBUG] Device ID: 0x", deviceID);

  if (deviceID != ID_DEFAULT) {
    PRINT_DEBUG("[ERROR] Device ID Mismatch!");
  } else {
    PRINT_DEBUG("[DEBUG] Device ID Verified.");
  }
}


// ADS1158 Reset
void ADS1158_reset() {
  PRINT_DEBUG("[DEBUG] Resetting ADS1158...");
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x60);  // Reset Command
  digitalWrite(CS_PIN, HIGH);

  // Reset configuration registers with brief bit descriptions
  ADS1158_writeRegister(CONFIG0_REG, 0b00000010);  // CONFIG0: RES|SPIRST|MUXMOD|BYPAS|CLKENB|CHOP|STAT|RES

  ADS1158_writeRegister(CONFIG1_REG, 0b01010000);  // CONFIG1: IDLMOD|DLY2|DLY1|DLY0|SBCS1|SBCS0|DRATE1|DRATE0

  ADS1158_writeRegister(MUXDIF_REG, 0b00000000);  // MUXDIF: DIFF7|DIFF6|DIFF5|DIFF4|DIFF3|DIFF2|DIFF1|DIFF0

  ADS1158_writeRegister(MUXSCH_REG, 0b00000000);  // MUXSCH: AINN3|AINN2|AINN1|AINN0|AINP3|AINP2|AINP1|AINP0

  ADS1158_writeRegister(MUXSG0_REG, 0b11111111);  // MUXSG0: AIN7|AIN6|AIN5|AIN4|AIN3|AIN2|AIN1|AIN0

  ADS1158_writeRegister(MUXSG1_REG, 0b01111100);  // MUXSG1: AIN15|AIN14|AIN13|AIN12|AIN11|AIN10|AIN9|AIN8

  ADS1158_writeRegister(SYSRED_REG, 0b00000000);  // SYSRED: RES|RES|REF|GAIN|TEMP|VCC|RES|OFFSET

  ADS1158_writeRegister(GPIOC_REG, 0b00000000);  // GPIOC: CIO7|CIO6|CIO5|CIO4|CIO3|CIO2|CIO1|CIO0

  ADS1158_writeRegister(GPIOD_REG, 0b00000000);  // GPIOD: DIO7|DIO6|DIO5|DIO4|DIO3|DIO2|DIO1|DIO0

  PRINT_DEBUG("[DEBUG] ADS1158 Reset Complete.");
}


void ADS1158_setFixedChannel(uint8_t channel) {
  if (channel > 15) {
    PRINT_DEBUG("[ERROR] Invalid channel number. Must be between 0 and 15.");
    return;
  }

  // For channels 0-7, configure MUXSCH register, for channels 8-15, configure MUXSG0 and MUXSG1 registers
  if (channel <= 7) {
    ADS1158_writeRegister(MUXSCH_REG, channel);  // Channels 0-7 are selected directly via MUXSCH
  } else if (channel <= 15) {
    uint8_t muxsgValue = 1 << (channel - 8);        // Calculate the bit to set in MUXSG1 for channels 8-15
    ADS1158_writeRegister(MUXSG1_REG, muxsgValue);  // Configure for channels 8-15 using MUXSG1
  }

  PRINT_DEBUG("[DEBUG] Fixed-Channel Mode configured for AIN" + String(channel));
}

int16_t ADS1158_readData(uint8_t &channel, bool &newData, bool &overflow, bool &lowSupply, bool statusByteEnabled) {
  int16_t data = 0;
  uint8_t status = 0;

  // Start SPI communication by pulling CS low
  digitalWrite(CS_PIN, LOW);

  // If status byte is enabled, read the status byte first
  if (statusByteEnabled) {
    status = SPI.transfer(0x00);
  }

  // Read the 3 bytes of ADC data (24 bits)
  uint8_t data1 = SPI.transfer(0x00);  // MSB
  uint8_t data2 = SPI.transfer(0x00);  // Middle byte

  // End SPI communication by pulling CS high
  digitalWrite(CS_PIN, HIGH);

  // Combine the three bytes into a 24-bit result
  data = ((int16_t)data1 << 8) | data2;



  // If the status byte is enabled, decode it
  if (statusByteEnabled) {
    newData = (status & 0x80) != 0;    // BIT 7: NEW (New Data)
    overflow = (status & 0x40) != 0;   // BIT 6: OVF (Overflow)
    lowSupply = (status & 0x20) != 0;  // BIT 5: SUPPLY (Low Supply Voltage)

    channel = (status & 0x1F) - 8;  // BIT 4-0: CHID (Channel ID, 5 bits)
                                    // Serial.print("Channel ID: ");
                                    // Serial.println(channel, BIN);

    if (overflow) {
      PRINT_DEBUG("[ERROR] OVF bit active, overflow!");
      delay(2000);
    }

    if (lowSupply) {
      PRINT_DEBUG("[ERROR] SUPPLY bit active, low supply voltage!");
      delay(2000);
    }
  } else {
    // If the status byte is not enabled, default the flags
    newData = true;     // Assume data is new
    overflow = false;   // No overflow
    lowSupply = false;  // No low supply
    channel = 0;        // Default channel if status byte is not available
  }



  return data;
}


void ADS1158_setAutoScanMode(uint16_t channels) {
  PRINT_DEBUG("[DEBUG] Configuring Auto-Scan Mode...");

  // Separate channels into MUXSG0 (for AIN0-AIN7) and MUXSG1 (for AIN8-AIN15)
  uint8_t muxsg0Value = channels & 0xFF;         // Channels AIN0 to AIN7
  uint8_t muxsg1Value = (channels >> 8) & 0xFF;  // Channels AIN8 to AIN15

  // Set the device to Auto-Scan mode by writing to CONFIG0 register
  ADS1158_writeRegister(CONFIG0_REG, CONFIG0_STAT);  // Auto-Scan mode
  ADS1158_setStatusByteEnabled(true);
  ADS1158_setConversionDelay(TimeDelay::DLY_128);
  ADS1158_setDataRate(DataRate::DR_1831_SPS);


  // Write to the MUXSG registers to select the channels
  ADS1158_writeRegister(MUXSG0_REG, muxsg0Value);  // Configure for AIN0 to AIN7
  ADS1158_writeRegister(MUXSG1_REG, muxsg1Value);  // Configure for AIN8 to AIN15

  // Print the enabled channels in decimal format
  Serial.print("[DEBUG] Enabled channels: ");
  bool first = true;
  for (int i = 0; i < 16; i++) {
    if (channels & (1 << i)) {
      if (!first) {
        Serial.print(", ");
      }
      Serial.print(i);  // Print the channel number in decimal
      first = false;
    }
  }
  Serial.println();

  PRINT_DEBUG("[DEBUG] Auto-Scan Mode configured for channels 0x" + String(channels, HEX));
}

// Test GPIO Control
void ADS1158_pinmodeGPIO(uint8_t pin, bool state) {
  PRINT_DEBUG("[DEBUG] Configuring GPIO...");

  // Read the current direction configuration
  uint8_t gpiodValue = ADS1158_readRegister(GPIOD_REG);

  if (state) {
    gpiodValue |= (1 << pin);  // Set the pin as output (bit = 1)
  } else {
    gpiodValue &= ~(1 << pin);  // Set the pin as input (bit = 0)
  }

  // Write back the new configuration
  ADS1158_writeRegister(GPIOD_REG, gpiodValue);

  PRINT_DEBUG("[DEBUG] GPIO Pin " + String(pin) + " set to " + (state ? "OUTPUT" : "INPUT") + ".");
}

void ADS1158_digitalWriteGPIO(uint8_t pin, bool value) {
  PRINT_DEBUG("[DEBUG] Setting GPIO State...");

  // Read the current GPIOC register (output states)
  uint8_t gpiocValue = ADS1158_readRegister(GPIOC_REG);

  if (value) {
    gpiocValue |= (1 << pin);  // Set the pin high (bit = 1)
  } else {
    gpiocValue &= ~(1 << pin);  // Set the pin low (bit = 0)
  }

  // Write back the new configuration
  ADS1158_writeRegister(GPIOC_REG, gpiocValue);

  PRINT_DEBUG("[DEBUG] GPIO Pin " + String(pin) + " set to " + (value ? "HIGH" : "LOW") + ".");
}


// Read Register
uint8_t ADS1158_readRegister(uint8_t reg) {
  uint8_t command = 0x40 | (reg & 0x0F);  // Register Read command
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(command);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return value;
}

// Write Register
void ADS1158_writeRegister(uint8_t reg, uint8_t value) {
  uint8_t command = 0x60 | (reg & 0x0F);  // Register Write command
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(command);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

void printBinary(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 0x01);
  }
}

void ADS1158_printRegisters() {
  Serial.println("Configuration Registers:");

  // Reading and printing CONFIG0 register
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  Serial.print("CONFIG0: 0x");
  Serial.print(config0, HEX);
  Serial.print(" (Binary: ");
  printBinary(config0);
  Serial.println(")");

  // Reading and printing CONFIG1 register
  uint8_t config1 = ADS1158_readRegister(CONFIG1_REG);
  Serial.print("CONFIG1: 0x");
  Serial.print(config1, HEX);
  Serial.print(" (Binary: ");
  printBinary(config1);
  Serial.println(")");

  // Reading and printing MUXSG0 register
  uint8_t muxsg0 = ADS1158_readRegister(MUXSG0_REG);
  Serial.print("MUXSG0: 0x");
  Serial.print(muxsg0, HEX);
  Serial.print(" (Binary: ");
  printBinary(muxsg0);
  Serial.println(")");

  // Reading and printing MUXSG1 register
  uint8_t muxsg1 = ADS1158_readRegister(MUXSG1_REG);
  Serial.print("MUXSG1: 0x");
  Serial.print(muxsg1, HEX);
  Serial.print(" (Binary: ");
  printBinary(muxsg1);
  Serial.println(")");

  // Reading and printing MUXSCH register
  uint8_t muxsch = ADS1158_readRegister(MUXSCH_REG);
  Serial.print("MUXSCH: 0x");
  Serial.print(muxsch, HEX);
  Serial.print(" (Binary: ");
  printBinary(muxsch);
  Serial.println(")");

  // Reading and printing MUXDIF register
  uint8_t muxdif = ADS1158_readRegister(MUXDIF_REG);
  Serial.print("MUXDIF: 0x");
  Serial.print(muxdif, HEX);
  Serial.print(" (Binary: ");
  printBinary(muxdif);
  Serial.println(")");

  // Reading and printing SYSRED register
  uint8_t sysred = ADS1158_readRegister(SYSRED_REG);
  Serial.print("SYSRED: 0x");
  Serial.print(sysred, HEX);
  Serial.print(" (Binary: ");
  printBinary(sysred);
  Serial.println(")");

  // Reading and printing GPIOC register
  uint8_t gpioc = ADS1158_readRegister(GPIOC_REG);
  Serial.print("GPIOC: 0x");
  Serial.print(gpioc, HEX);
  Serial.print(" (Binary: ");
  printBinary(gpioc);
  Serial.println(")");

  // Reading and printing GPIOD register
  uint8_t gpiod = ADS1158_readRegister(GPIOD_REG);
  Serial.print("GPIOD: 0x");
  Serial.print(gpiod, HEX);
  Serial.print(" (Binary: ");
  printBinary(gpiod);
  Serial.println(")");

  // Reading and printing Device ID register
  uint8_t deviceID = ADS1158_readRegister(ID_REG);
  Serial.print("Device ID: 0x");
  Serial.print(deviceID, HEX);
  Serial.print(" (Binary: ");
  printBinary(deviceID);
  Serial.println(")");
}

// Function to enable/disable SPI interface reset timer
void ADS1158_configureSpiResetTimer(bool enable) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = enable ? (config0 | (1 << 6)) : (config0 & ~(1 << 6));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to set multiplexer mode (Auto-Scan or Fixed-Channel)
void ADS1158_setMultiplexerMode(bool isFixedChannelMode) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = isFixedChannelMode ? (config0 | (1 << 5)) : (config0 & ~(1 << 5));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to set bypass mode (Internal or External MUX connection)
void ADS1158_setMuxBypass(bool useExternalMux) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = useExternalMux ? (config0 | (1 << 4)) : (config0 & ~(1 << 4));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to enable/disable clock output
void ADS1158_setClockOutputEnabled(bool enable) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = enable ? (config0 | (1 << 3)) : (config0 & ~(1 << 3));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to enable/disable chopper
void ADS1158_configureChopper(bool enable) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = enable ? (config0 | (1 << 2)) : (config0 & ~(1 << 2));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to enable/disable status byte
void ADS1158_setStatusByteEnabled(bool enable) {
  uint8_t config0 = ADS1158_readRegister(CONFIG0_REG);
  config0 = enable ? (config0 | (1 << 1)) : (config0 & ~(1 << 1));
  ADS1158_writeRegister(CONFIG0_REG, config0);
}

// Function to configure idle mode (Low Power or Normal Operation)
void ADS1158_setIdleMode(bool lowPowerMode) {
  uint8_t config1 = ADS1158_readRegister(CONFIG1_REG);
  config1 = lowPowerMode ? (config1 | (1 << 7)) : (config1 & ~(1 << 7));
  ADS1158_writeRegister(CONFIG1_REG, config1);
}

// Function to configure conversion delay (DLY2, DLY1, DLY0)
void ADS1158_setConversionDelay(uint8_t delaySetting) {
  uint8_t config1 = ADS1158_readRegister(CONFIG1_REG);
  config1 = (config1 & ~(0x70)) | (delaySetting << 4);
  ADS1158_writeRegister(CONFIG1_REG, config1);
}

// Function to configure bias sense current (SBCS1, SBCS0)
void ADS1158_setBiasSenseCurrent(uint8_t currentSetting) {
  uint8_t config1 = ADS1158_readRegister(CONFIG1_REG);
  config1 = (config1 & ~(0x0C)) | (currentSetting << 2);
  ADS1158_writeRegister(CONFIG1_REG, config1);
}

// Function to configure data rate (DRATE1, DRATE0)
void ADS1158_setDataRate(uint8_t dataRateSetting) {
  uint8_t config1 = ADS1158_readRegister(CONFIG1_REG);
  config1 = (config1 & ~(0x03)) | dataRateSetting;
  ADS1158_writeRegister(CONFIG1_REG, config1);
}