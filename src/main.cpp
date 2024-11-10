#include <Arduino.h>
#include <BluetoothSerial.h>
#include "Config.h"
#include "ADS1158.h"
#include <Preferences.h>

BluetoothSerial SerialBT; // Initialize the Bluetooth serial object
Preferences preferences;

volatile bool dataReadyFlag = false;
uint8_t measurementBuffer[NUM_MEASUREMENTS * MEASUREMENT_SIZE]; // Buffer to store multiple measurements
uint8_t bufferIndex = 0;

bool isSerialDebugEnabled = false;
bool isBTdebugEnabled = false;

ADS1158 adc(CS_PIN); // Create an instance of ADS1158

void IRAM_ATTR dataReadyISR()
{
  dataReadyFlag = true; // Set the flag when DRDY_PIN goes low
}

void setup() {
    Serial.begin(SERIAL_FREQ);
    initializeBluetooth();
    initializeSPI();
    initializeHardwarePins();
    initializeADS1158();
    loadOrSetDefaultPreferences();
}

void initializeBluetooth() {
    SerialBT.begin(DEVICE_BT_IDENTIFIER);
}

void initializeSPI() {
    SPI.begin();
    SPI.setFrequency(SPI_FREQ);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
}

void initializeADS1158() {
    PRINT_DEBUG("[DEBUG] Initializing ADS1158...");
    adc.begin();
    adc.setAutoScanMode(ACTIVE_ADC_CHANNELS);
    adc.printRegisters();
    attachInterrupt(digitalPinToInterrupt(DRDY_PIN), dataReadyISR, FALLING);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(START_PIN, HIGH);
}

void scanAdcDataAutomatically(int16_t channelData[])
{
    uint8_t channelsCollected = 0;   // To track how many channels have been collected
    bool channelFlags[TOTAL_CHANNELS] = {false}; // Track which channels have been read (for channels 2 to 14)

    while (channelsCollected < TOTAL_CHANNELS)
    {   // Continue until all 13 channels are collected (2 to 14)
        if (dataReadyFlag)
        {   // Only process if interrupt occurred (data is ready)
            uint8_t currentChannel = 0;
            bool newData = false, overflow = false, lowSupply = false;

            // Read the ADC data and retrieve the current channel number
            int16_t adcData = adc.readData(currentChannel, newData, overflow, lowSupply, true);

            // Reset the interrupt flag immediately to minimize data loss
            dataReadyFlag = false;

            // Store data if valid and the channel is within the range 2 to 14
            if (newData && currentChannel >= MIN_VALID_CHANNEL && currentChannel <= MAX_VALID_CHANNEL)
            {
                uint8_t index = currentChannel - MIN_VALID_CHANNEL; // Channel 2 corresponds to index 0

                // If this channel's data hasn't been collected yet
                if (!channelFlags[index])
                {
                    channelData[index] = adcData; // Store the data in the array
                    channelFlags[index] = true;   // Mark this channel as collected
                    channelsCollected++;          // Increment the count of collected channels
                }
            }
        }
    }
}


void storeMeasurementInBuffer(const int16_t dataBuffer[], uint32_t timestamp, uint8_t *measurementBuffer, uint8_t *bufferIndex)
{
    uint8_t *bufferPtr = &measurementBuffer[*bufferIndex * (MEASUREMENT_SIZE + MARKER_SIZE)];

    // Add marker bytes to the buffer
    memcpy(bufferPtr, markerBytes, MARKER_SIZE);
    bufferPtr += MARKER_SIZE;

    // Add timestamp to the buffer (4 bytes)
    memcpy(bufferPtr, &timestamp, sizeof(timestamp));
    bufferPtr += sizeof(timestamp);

    // Add actual values to the buffer (each value is 2 bytes, int16_t)
    memcpy(bufferPtr, dataBuffer, sizeof(int16_t) * TOTAL_CHANNELS);

    // Increment the buffer index
    (*bufferIndex)++;

    // If the buffer is full, send the data and reset the buffer
    if (*bufferIndex >= NUM_MEASUREMENTS)
    {
        transmitBufferData(measurementBuffer, bufferIndex);
    }
}

void transmitBufferData(uint8_t *measurementBuffer, uint8_t *bufferIndex)
{
  // Calculate the total size to send
  uint16_t totalBufferSize = (*bufferIndex) * (MEASUREMENT_SIZE + MARKER_SIZE);

  if (isSerialDebugEnabled)
  {
    Serial.write(measurementBuffer, totalBufferSize);
  }
  else
  {
    SerialBT.write(measurementBuffer, totalBufferSize);
  }
  // Reset the buffer index after sending
  *bufferIndex = 0;
}

void ADS1158_run_and_send_meas()
{
  int16_t channelData[TOTAL_CHANNELS] = {0}; // Data for channels 2 to 14
  uint32_t timestamp = millis(); // Capture current timestamp

  // Step 1: Get ADC data for all channels
  scanAdcDataAutomatically(channelData);

  // Create a local buffer and index
  uint8_t measurementBuffer[NUM_MEASUREMENTS * (MEASUREMENT_SIZE + MARKER_SIZE)]; // Buffer to store multiple measurements
  uint8_t bufferIndex = 0;                                                        // Current position in the buffer

  // Step 2: Buffer the measurement
  storeMeasurementInBuffer(channelData, timestamp, measurementBuffer, &bufferIndex);
}

void loop()
{
  processSerialCommands();
}

void initializeHardwarePins()
{
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

bool getPreference(const String &key, bool defaultValue) {
    return preferences.getBool(key.c_str(), defaultValue);
}

uint16_t getPreference(const String &key, uint16_t defaultValue) {
    return preferences.getUInt(key.c_str(), defaultValue);
}

void setPreference(const String &key, bool value) {
    preferences.putBool(key.c_str(), value);
}

void setPreference(const String &key, uint16_t value) {
    preferences.putUInt(key.c_str(), value);
}

void loadOrSetDefaultPreferences()
{
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Check if the preferences exist

  // Load and set the values from EEPROM

  uint16_t ActiveChannels = preferences.getUInt("Channels", ActiveChannels);
  isSerialDebugEnabled = preferences.getBool("DebugUSB", isSerialDebugEnabled);
  isBTdebugEnabled = preferences.getBool("DebugBT", isBTdebugEnabled);

  // Load and set other configuration values similarly
  if (isBTdebugEnabled)
  {
    SerialBT.println("[DEBG] Preferences loaded.");
  }
  if (isSerialDebugEnabled)
  {
    Serial.println("Preferences loaded.");
  }
  // Close the preferences storage
  preferences.end();
}

void resetPreferencesToDefaults()
{
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Remove all preferences
  preferences.clear();

  // Set all values to their default values
  uint16_t ActiveChannels = ACTIVE_ADC_CHANNELS; // For future use
  isSerialDebugEnabled = true;                   // Set to your desired default value
  isBTdebugEnabled = true;                       // Set to your desired default value

  // Set other configuration values to their defaults similarly

  // Close the preferences storage
  preferences.end();
}

void savePreferences()
{
  // Open the preferences storage
  uint16_t ActiveChannels = ACTIVE_ADC_CHANNELS;

  if (!preferences.begin("my-app", false))
  {
    SerialBT.println("[INFO] Failed to open preferences for saving");
    return;
  }
  // Save the current configuration values to EEPROM

  preferences.putUInt("Channels", ActiveChannels);
  preferences.putBool("DebugUSB", isSerialDebugEnabled);
  preferences.putBool("DebugBT", isBTdebugEnabled);

  // Save other configuration values similarly
  if (isBTdebugEnabled)
  {
    SerialBT.println("[INFO] Preferences saved.");
  }
  // Close the preferences storage
  preferences.end();
}

void listPreferences()
{
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Read and print each preference value
  SerialBT.println("[INFO] Current Configuration Preferences:");
  SerialBT.println("[INFO] NumberOfChannels: " + String(preferences.getUInt("Channels", 0)));
  SerialBT.println("[INFO] isSerialDebugEnabled: " + String(preferences.getBool("DebugUSB", true)));
  SerialBT.println("[INFO] isBTdebugEnabled: " + String(preferences.getBool("DebugBT", true)));

  // Read and print other preferences similarly

  // Close the preferences storage
  preferences.end();
}

void listDebugPreferences()
{
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Read the preferences
  bool isSerialDebugEnabled = preferences.getBool("DebugUSB", true);
  bool isBTdebugEnabled = preferences.getBool("DebugBT", true);

  // Close the preferences storage
  preferences.end();

  // Convert boolean values to '1' or '0' and concatenate them into a string
  String configString = "[CNFG]";
  configString += (isSerialDebugEnabled ? '1' : '0');
  configString += (isBTdebugEnabled ? '1' : '0');
  configString += DEVICE_NAME;

  // Output the configuration string to SerialBT
  SerialBT.print(configString);
}

uint8_t getBatteryPercentage()
{
  digitalWrite(BAT_ADC_EN, HIGH);
  delay(500);
  int adcValue = analogRead(BAT_ADC);
  float voltage = adcValue / 4095.0 * 3.3;
  int batteryPercentage = lookupPercentage(voltage);
  digitalWrite(BAT_ADC_EN, LOW);
  return batteryPercentage;
}

int lookupPercentage(float voltage)
{
  // Define a lookup table with voltage levels and corresponding battery percentages for 1S
  const float voltageTable[] = {
      4.2, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95, 3.91, 3.87, 3.85, 3.84, 3.82, 3.8, 3.79, 3.77, 3.75, 3.73, 3.71, 3.69, 3.61, 3.27};
  const int percentageTable[] = {
      100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
  // Check if voltage is outside the table range
  if (voltage > voltageTable[0])
  {
    return 100;
  }
  if (voltage < voltageTable[sizeof(voltageTable) / sizeof(voltageTable[0]) - 1])
  {
    return 0;
  }

  // Find the nearest voltage level in the table
  int i;
  for (i = 0; i < sizeof(voltageTable) / sizeof(voltageTable[0]) - 1; i++)
  {
    if (voltage >= voltageTable[i])
    {
      break;
    }
  }

  // Perform linear interpolation between the two nearest voltage points
  float percentage = map(voltage, voltageTable[i], voltageTable[i + 1], percentageTable[i], percentageTable[i + 1]);

  return int(percentage); // Convert to integer percentage
}

void processSerialCommands()
{
  if (SerialBT.available())
  {
    // String command = Serial.readStringUntil('\n');
    String command = SerialBT.readStringUntil('\n');
    if (isSerialDebugEnabled)
    {
      Serial.println("Command read: " + command);
    }
    // SerialBT.println("Command read: " + command);
    command.trim();
    if (command.startsWith("/help"))
    {
      SerialBT.println("Available commands:");
      SerialBT.println("/debugBT [true/false] - Enable or disable Bluetooth debug messages");
      SerialBT.println("/debugUSB [true/false] - Enable or disable Serial USB debug messages");
      SerialBT.println("/single - Perform a single read of ADC values");
      SerialBT.println("/start -t [Time in seconds] -r [sample rate in Hz] - Start recording and sending ADC data via Bluetooth");
      SerialBT.println("/setChannels [value] - Set the number of channels (1 to 12)");
      SerialBT.println("/testMode [true/false] - Enable or disable test mode with randomly generated values");
      SerialBT.println("/checkHardware - Perform hardware checks");
      SerialBT.println("/getConfig - Display the current configuration parameters");
      SerialBT.println("/setActiveChannels [config1] [config2] [config3] - Manually set the ADC configuration (0 to 4 for each value)");
      SerialBT.println("/getMAC - Display the MAC address of the ESP32");
      SerialBT.println("/listPreferences - List stored preferences");
      SerialBT.println("/help - Display this message again");
    }
    else if (command.startsWith("/debugBT"))
    {
      int spaceIndex = command.indexOf(" ");
      if (spaceIndex != -1)
      {
        String valueStr = command.substring(spaceIndex + 1);
        if (valueStr.equals("true"))
        {
          isBTdebugEnabled = true;
          SerialBT.println("[INFO] Debug messages via Bluetooth are enabled.");
        }
        else if (valueStr.equals("false"))
        {
          isBTdebugEnabled = false;
          SerialBT.println("[INFO] Debug messages via Bluetooth are disabled.");
        }
        else
        {
          SerialBT.println("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Bluetooth.");
        }
      }
      else
      {
        SerialBT.println("[ERROR] Missing value. Use 'true' or 'false' to enable or disable debug messages via Bluetooth.");
      }
    }
    else if (command.startsWith("/getMAC"))
    {
      String macAddress = "Not avaliable"; // getMACAddress();
      SerialBT.println("ESP32 MAC Address: " + macAddress);
    }
    else if (command.startsWith("/debugUSB"))
    {
      int spaceIndex = command.indexOf(" ");
      if (spaceIndex != -1)
      {
        String valueStr = command.substring(spaceIndex + 1);
        if (valueStr.equals("true"))
        {
          isSerialDebugEnabled = true;
          SerialBT.print("[INFO] Debug messages via Serial USB are enabled.");
        }
        else if (valueStr.equals("false"))
        {
          isSerialDebugEnabled = false;
          SerialBT.print("[INFO] Debug messages via Serial USB are disabled.");
        }
        else
        {
          SerialBT.print("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Serial USB.");
        }
      }
      else
      {
        SerialBT.print("[ERROR] Missing value. Use 'true' or 'false' to enable or disable debug messages via Serial USB.");
      }
    }
    else if (command.startsWith("/single"))
    {
      String data = "";
      // data = readADCValues();
      SerialBT.println(data);
    }
    else if (command.startsWith("/start"))
    {
      // Find the position of the time flag
      int timeFlagPos = command.indexOf("-t");
      if (timeFlagPos != -1)
      {
        // Extract the duration from the command
        int timeValuePos = timeFlagPos + 2; // Length of "-t"
        String durationStr = command.substring(timeValuePos);
        int durationInSeconds = durationStr.toInt();
        startRecording(durationInSeconds);
      }
      else
      {
        Serial.println("[ERROR] Missing -t flag for recording time");
        SerialBT.println("[ERROR] Missing -t flag for recording time");
      }
    }
    else if (command.startsWith("/checkHardware"))
    {
      // performHardwareChecks();
    }
    else if (command.startsWith("/getConfig"))
    {
      SerialBT.print("[INFO] Current Configuration Parameters:");
      SerialBT.print("[INFO] Device Name: " + String(DEVICE_NAME));
      SerialBT.print("[INFO] Serial Debug Enabled: " + String(isSerialDebugEnabled));
      SerialBT.print("[INFO] Bluetooth Debug Enabled: " + String(isBTdebugEnabled));
      // SerialBT.println("Battery Level: " + String(getBatteryPercentage()) + "%");
    }
    else if (command.startsWith("/listDebugPreferences"))
    {
      listDebugPreferences();
    }
    else
    {
      SerialBT.print("[INFO] Invalid command! Type '/help' to see the list of available commands.");
      return;
    }
    savePreferences();
  }
}

void startRecording(int durationInSeconds)
{
  uint32_t lastMeasEndMs = 0;
  unsigned long recordingDurationMs = durationInSeconds * 1000;
  bool isRecording = true;
  unsigned long recordingStartTime = millis();
  while (isRecording)
  {
    ADS1158_run_and_send_meas();
    if (millis() - recordingStartTime >= recordingDurationMs)
    {
      isRecording = false;
    }
  }
}