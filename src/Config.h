// Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <SPI.h>
#include "BluetoothSerial.h"  // Include the Bluetooth library for ESP32
#include <Preferences.h>

extern BluetoothSerial SerialBT;
extern bool isSerialDebugEnabled;
extern bool isBTdebugEnabled;

// Debugging Macro
constexpr bool DEBUG = true;
#define PRINT_DEBUG(x) \
    if (DEBUG) { Serial.println(x); }
#define PRINT_DEBUG_VAL(x, y) \
    if (DEBUG) { \
        Serial.print(x); \
        Serial.println(y, HEX); \
    }

// Pin Definitions
constexpr uint8_t CS_PIN = 2;         // Chip Select
constexpr uint8_t DRDY_PIN = 17;      // Data Ready
constexpr uint8_t START_PIN = 5;      // Start Conversion
constexpr uint8_t CLKEN_PIN = 16;     // Clock Enable
constexpr uint8_t ADC_PWDN_PIN = 21;  // ADC Power Down
constexpr uint8_t ADC_RST_PIN = 4;    // ADC Reset
constexpr uint8_t EN_5V_PIN = 35;     // Enable 5V Power
constexpr uint8_t LED_PIN = 25;       // MCU LED
constexpr uint8_t BAT_ADC_EN = 12;    // Battery ADC Enable
constexpr uint8_t BAT_ADC = 32;       // Battery ADC Pin

// Device Configuration
constexpr uint8_t DEVICE_RIGHT = 1;
constexpr uint8_t DEVICE_LEFT = 2;
constexpr uint8_t DEVICE_NAME = DEVICE_RIGHT;  // Set this to either DEVICE_RIGHT or DEVICE_LEFT

constexpr uint16_t ACTIVE_ADC_CHANNELS = 0b0111111111111100;
constexpr uint32_t SPI_FREQ = 2000000;
constexpr uint32_t SERIAL_FREQ = 1000000;
constexpr uint8_t NUM_MEASUREMENTS = 1;   // Number of measurements to buffer before sending
constexpr uint8_t MEASUREMENT_SIZE = 31;  // Size of each measurement (1 byte for start + 4 for timestamp + 13 * 2 for channels)
constexpr uint8_t MARKER_SIZE = 4;        // Number of marker bytes
constexpr uint8_t INSOLE_ID = 1;
constexpr uint8_t TOTAL_CHANNELS = 13;
constexpr uint8_t MIN_VALID_CHANNEL = 2;
constexpr uint8_t MAX_VALID_CHANNEL = 14;

// Marker bytes definition based on device name
#if DEVICE_NAME == DEVICE_RIGHT
constexpr uint8_t markerBytes[MARKER_SIZE] = { 0xAA, 0xFF, 0xFF, INSOLE_ID };
constexpr char DEVICE_BT_IDENTIFIER[] = "ESP32_RIGHT";
#elif DEVICE_NAME == DEVICE_LEFT
constexpr uint8_t markerBytes[MARKER_SIZE] = { 0xAA, 0xFF, 0x00, INSOLE_ID };
constexpr char DEVICE_BT_IDENTIFIER[] = "ESP32_LEFT";
#else
#error "Unknown DEVICE_NAME. Please set DEVICE_NAME to either DEVICE_RIGHT or DEVICE_LEFT."
#endif

// Function Prototypes
void scanAdcDataAutomatically(int16_t channelData[]);
//void sendData(int32_t channelData[], int32_t previousChannelData[]);
void printBinary(uint8_t value);
void initializeHardwarePins();
void storeMeasurementInBuffer(const int16_t dataBuffer[], uint32_t timestamp, uint8_t *measurementBuffer, uint8_t *bufferIndex);
void transmitBufferData(uint8_t *measurementBuffer, uint8_t *bufferIndex);
void ADS1158_run_and_send_meas();
uint8_t getBatteryPercentage();
int lookupPercentage(float voltage);
void processSerialCommands();
void startRecording(int durationInSeconds);
void initializeBluetooth();
void initializeSPI();
void initializeADS1158();

// Command Handler Functions
void displayHelp();
void handleDebugBTCommand(String param);
void handleDebugUSBCommand(String param);
void displayMACAddress();
void performSingleADCRead();
void handleStartCommand(String param);
void performHardwareChecks();
void displayConfig();
void initializeCommandMap();

#endif  // CONFIG_H
