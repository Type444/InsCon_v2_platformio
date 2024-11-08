#include <Arduino.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include "Config.h"
#include "PowerManager.h"
#include "PreferencesManager.h"
#include "ADS1158Driver.h"
#include "Control.h"
#include <Preferences.h>


BluetoothSerial SerialBT;  // Initialize the Bluetooth serial object


volatile bool dataReadyFlag = false;
uint8_t measurementBuffer[NUM_MEASUREMENTS * MEASUREMENT_SIZE];  // Buffer to store multiple measurements
uint8_t bufferIndex = 0;


bool isSerialDebugEnabled = false;
bool isBTdebugEnabled = false;








void IRAM_ATTR dataReadyISR() {
  dataReadyFlag = true;  // Set the flag when DRDY_PIN goes low
}



void setup() {
  Serial.begin(SERIAL_FREQ);
  SerialBT.begin(DEVICE_BT_IDENTIFIER);
  SPI.begin();
  SPI.setFrequency(SPI_FREQ);
  SPI.setDataMode(SPI_MODE0);  // Set SPI Mode 0 (or the correct mode for your setup)
  SPI.setBitOrder(MSBFIRST);
  initializeHardwarePins();
  PRINT_DEBUG("[DEBUG] Initializing ADS1158...");
  ADS1158_init();
  ADS1158_setAutoScanMode(ACTIVE_ADC_CHANNELS);
  ADS1158_printRegisters();
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), dataReadyISR, FALLING);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(START_PIN, HIGH);
  loadOrSetDefaultPreferences();
}


void scanAdcDataAutomatically(int16_t channelData[]) {
  uint8_t channelsCollected = 0;      // To track how many channels have been collected
  bool channelFlags[13] = { false };  // Track which channels have been read (for channels 2 to 14)

  while (channelsCollected < 13) {  // Continue until all 13 channels are collected (2 to 14)
    if (dataReadyFlag) {            // Only process if interrupt occurred (data is ready)
      uint8_t currentChannel = 0;
      bool newData = false, overflow = false, lowSupply = false;

      // Read the ADC data and retrieve the current channel number
      int16_t adcData = ADS1158_readData(currentChannel, newData, overflow, lowSupply, true);

      // Store data if valid and the channel is within the range 2 to 14
      if (newData && currentChannel >= 2 && currentChannel <= 14) {
        uint8_t index = currentChannel - 2;  // Channel 2 corresponds to index 0

        // If this channel's data hasn't been collected yet
        if (!channelFlags[index]) {
          channelData[index] = adcData;  // Store the data in the array
          channelFlags[index] = true;    // Mark this channel as collected
          channelsCollected++;           // Increment the count of collected channels
        }
      }

      // Reset the interrupt flag to wait for the next interrupt
      dataReadyFlag = false;
    }
  }
}



void storeMeasurementInBuffer(const int16_t dataBuffer[], uint32_t timestamp, uint8_t *measurementBuffer, uint8_t *bufferIndex) {
  uint8_t *bufferPtr = &measurementBuffer[*bufferIndex * (MEASUREMENT_SIZE + MARKER_SIZE)];

  // Add marker bytes to the buffer
  memcpy(bufferPtr, markerBytes, MARKER_SIZE);
  bufferPtr += MARKER_SIZE;

  // Add timestamp to the buffer (4 bytes)
  memcpy(bufferPtr, &timestamp, sizeof(timestamp));
  bufferPtr += sizeof(timestamp);

  // Add actual values to the buffer (each value is 2 bytes, int16_t)
  memcpy(bufferPtr, dataBuffer, sizeof(dataBuffer[0]) * 13);

  // Increment the buffer index
  (*bufferIndex)++;

  // If the buffer is full, send the data and reset the buffer
  if (*bufferIndex >= NUM_MEASUREMENTS) {
    transmitBufferData(measurementBuffer, bufferIndex);
  }
}

void transmitBufferData(uint8_t *measurementBuffer, uint8_t *bufferIndex) {
  // Calculate the total size to send
  uint16_t totalBufferSize = (*bufferIndex) * (MEASUREMENT_SIZE + MARKER_SIZE);

  if (isSerialDebugEnabled){
    Serial.write(measurementBuffer, totalBufferSize);
  }else{
    SerialBT.write(measurementBuffer, totalBufferSize);
  }
  // Reset the buffer index after sending
  *bufferIndex = 0;
}

void ADS1158_run_and_send_meas() {
  int16_t channelData[13] = { 0 };  // Data for channels 2 to 14
  uint32_t timestamp = millis();    // Capture current timestamp

  // Step 1: Get ADC data for all channels
  scanAdcDataAutomatically(channelData);

  // Create a local buffer and index
  uint8_t measurementBuffer[NUM_MEASUREMENTS * (MEASUREMENT_SIZE + MARKER_SIZE)];  // Buffer to store multiple measurements
  uint8_t bufferIndex = 0;                                                         // Current position in the buffer

  // Step 2: Buffer the measurement
  storeMeasurementInBuffer(channelData, timestamp, measurementBuffer, &bufferIndex);
}

void loop() {
  processSerialCommands();
}


void initializeHardwarePins() {
  pinMode(CLKEN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  pinMode(ADC_RST_PIN, OUTPUT);
  pinMode(ADC_PWDN_PIN, OUTPUT);
  pinMode(EN_5V_PIN, OUTPUT);
  pinMode(BAT_ADC_EN, OUTPUT);
  pinMode(BAT_ADC, INPUT);
  digitalWrite(EN_5V_PIN, HIGH);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(ADC_RST_PIN, HIGH);
  digitalWrite(ADC_PWDN_PIN, HIGH);
  digitalWrite(START_PIN, LOW);
  digitalWrite(CLKEN_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BAT_ADC_EN, LOW);
}




