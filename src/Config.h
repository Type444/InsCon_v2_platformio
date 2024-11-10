// YourDevice.h
#ifndef SETTINGS_H
#define SETTINGS_H

#include <SPI.h>
#include "BluetoothSerial.h"  // Include the Bluetooth library for ESP32
#include <Preferences.h>

extern BluetoothSerial SerialBT;
extern bool isSerialDebugEnabled;
extern bool isBTdebugEnabled;

// Debugging Macro
#define DEBUG true
#define PRINT_DEBUG(x) \
    if (DEBUG) { Serial.println(x); }
#define PRINT_DEBUG_VAL(x, y) \
    if (DEBUG) { \
        Serial.print(x); \
        Serial.println(y, HEX); \
    }

// Pin Definitions
#define CS_PIN 2         // Chip Select
#define DRDY_PIN 17      // Data Ready
#define START_PIN 5      // Start Conversion
#define CLKEN_PIN 16     // Clock Enable
#define ADC_PWDN_PIN 21  // ADC Power Down
#define ADC_RST_PIN 4    // ADC Reset
#define EN_5V_PIN 35     // Enable 5V Power
#define LED_PIN 25       // MCU LED
#define BAT_ADC_EN 12
#define BAT_ADC 32

// Device Configuration
#define DEVICE_RIGHT 1
#define DEVICE_LEFT 2
#define DEVICE_NAME DEVICE_RIGHT  // Set this to either DEVICE_RIGHT or DEVICE_LEFT

#define ACTIVE_ADC_CHANNELS 0b0111111111111100
#define SPI_FREQ 2000000
#define SERIAL_FREQ 1000000
#define NUM_MEASUREMENTS 1   // Number of measurements to buffer before sending
#define MEASUREMENT_SIZE 31  // Size of each measurement (1 byte for start + 4 for timestamp + 13 * 2 for channels)
#define MARKER_SIZE 4        // Number of marker bytes
#define INSOLE_ID 1

// Marker bytes definition based on device name
#if DEVICE_NAME == DEVICE_RIGHT
const uint8_t markerBytes[MARKER_SIZE] = { 0xAA, 0xFF, 0xFF, INSOLE_ID };
#define DEVICE_BT_IDENTIFIER "ESP32_RIGHT"
#elif DEVICE_NAME == DEVICE_LEFT
const uint8_t markerBytes[MARKER_SIZE] = { 0xAA, 0xFF, 0x00, INSOLE_ID };
#define DEVICE_BT_IDENTIFIER "ESP32_LEFT"
#else
#error "Unknown DEVICE_NAME. Please set DEVICE_NAME to either DEVICE_RIGHT or DEVICE_LEFT."
#endif

// Function Prototypes
void ADS1158_init();
void ADS1158_reset();
void ADS1158_configure();
void ADS1158_testGPIO();
void ADS1158_autoScanTest();
void ADS1158_fixedChannelTest();
uint8_t ADS1158_readRegister(uint8_t reg);
void ADS1158_writeRegister(uint8_t reg, uint8_t value);
int16_t ADS1158_readData(uint8_t &channel, bool &newData, bool &overflow, bool &lowSupply, bool statusByteEnabled);
void ADS1158_setAutoScanMode(uint16_t channels);
void ADS1158_pinmodeGPIO(uint8_t pin, bool state);
void ADS1158_digitalWriteGPIO(uint8_t pin, bool value);
void ADS1158_setFixedChannel(uint8_t channel);
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

void scanAdcDataAutomatically(int16_t channelData[]);
void sendData(int32_t channelData[], int32_t previousChannelData[]);
void printBinary(uint8_t value);
void initializeHardwarePins();
void storeMeasurementInBuffer(const int16_t dataBuffer[], uint32_t timestamp, uint8_t *measurementBuffer, uint8_t *bufferIndex);
void transmitBufferData(uint8_t *measurementBuffer, uint8_t *bufferIndex);
void ADS1158_run_and_send_meas();
uint8_t getBatteryPercentage();
int lookupPercentage(float voltage);
void processSerialCommands();
void startRecording(int durationInSeconds);
void loadOrSetDefaultPreferences();

#endif  // SETTINGS_H
