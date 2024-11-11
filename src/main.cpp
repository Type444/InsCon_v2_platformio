#include <Arduino.h>
#include <BluetoothSerial.h>
#include "Config.h"
#include "ADS1158.h"
#include <PreferencesManager.h>
#include <map>
#include <functional>

std::map<String, std::function<void(String)>> commandMap;
BluetoothSerial SerialBT; // Initialize the Bluetooth serial object
PreferencesManager preferencesManager("my-app");

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
            
    preferencesManager.loadOrSetDefaultPreferences();
}

void loop()
{
    processSerialCommands();
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



// Initialize command handlers
void initializeCommandMap() {
   commandMap = {
      {"/help", [](String param) { displayHelp(); }},
      {"/debugBT", [](String param) { handleDebugBTCommand(param); }},
      {"/debugUSB", [](String param) { handleDebugUSBCommand(param); }},
      {"/getMAC", [](String param) { displayMACAddress(); }},
      {"/single", [](String param) { performSingleADCRead(); }},
      {"/start", [](String param) { handleStartCommand(param); }},
      {"/checkHardware", [](String param) { performHardwareChecks(); }},
      {"/getConfig", [](String param) { displayConfig(); }},
      {"/listDebugPreferences", [](String param) { preferencesManager.listDebugPreferences(); }}
  };

}

void processSerialCommands() {
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        command.replace("\r", "");  // Remove any carriage returns
        command.trim();  // Remove trailing/leading spaces

        if (isSerialDebugEnabled) {
            Serial.println("Command read: '" + command + "'");
        }

        // Find the first word (command) and any additional parameters
        int spaceIndex = command.indexOf(' ');
        String cmdKey = (spaceIndex == -1) ? command : command.substring(0, spaceIndex);
        String param = (spaceIndex == -1) ? "" : command.substring(spaceIndex + 1);

        if (isSerialDebugEnabled) {
            Serial.println("Command: '" + command + "'");
            Serial.println("cmdKey: '" + cmdKey + "'");
            Serial.println("param: '" + param + "'");
            // Print cmdKey in hex to check for hidden characters
            Serial.print("cmdKey in hex: ");
            for (unsigned int i = 0; i < cmdKey.length(); ++i) {
                Serial.print(cmdKey[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }

        // Look up command in the map and execute associated function
        bool commandFound = false;
        for (auto const& entry : commandMap) {
            if (cmdKey.equals(entry.first)) {
                entry.second(param);
                commandFound = true;
                break;
            }
        }
        if (!commandFound) {
            SerialBT.print("[INFO] Invalid command! Type '/help' to see the list of available commands.");
        }
    }
}

// Command handler functions
void displayHelp() {
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

void handleDebugBTCommand(String param) {
    if (param == "true") {
        isBTdebugEnabled = true;
        SerialBT.println("[INFO] Debug messages via Bluetooth are enabled.");
    } else if (param == "false") {
        isBTdebugEnabled = false;
        SerialBT.println("[INFO] Debug messages via Bluetooth are disabled.");
    } else {
        SerialBT.println("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Bluetooth.");
    }
    preferencesManager.savePreferences();
}

void handleDebugUSBCommand(String param) {
    if (param == "true") {
        isSerialDebugEnabled = true;
        SerialBT.println("[INFO] Debug messages via Serial USB are enabled.");
    } else if (param == "false") {
        isSerialDebugEnabled = false;
        SerialBT.println("[INFO] Debug messages via Serial USB are disabled.");
    } else {
        SerialBT.println("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Serial USB.");
    }
    preferencesManager.savePreferences();
}

void displayMACAddress() {
    String macAddress = "Not available"; // Replace with actual MAC retrieval
    SerialBT.println("ESP32 MAC Address: " + macAddress);
}

void performSingleADCRead() {
    String data = ""; // Replace with actual ADC read logic
    SerialBT.println(data);
}

void handleStartCommand(String param) {
    int timeFlagPos = param.indexOf("-t");
    if (timeFlagPos != -1) {
        int timeValuePos = timeFlagPos + 2; // Length of "-t"
        String durationStr = param.substring(timeValuePos);
        int durationInSeconds = durationStr.toInt();
        startRecording(durationInSeconds);
    } else {
        SerialBT.println("[ERROR] Missing -t flag for recording time");
    }
}

void performHardwareChecks() {
    // Logic for hardware checks
}

void displayConfig() {
    SerialBT.println("[INFO] Current Configuration Parameters:");
    SerialBT.println("[INFO] Device Name: " + String(DEVICE_NAME));
    SerialBT.println("[INFO] Serial Debug Enabled: " + String(isSerialDebugEnabled));
    SerialBT.println("[INFO] Bluetooth Debug Enabled: " + String(isBTdebugEnabled));
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