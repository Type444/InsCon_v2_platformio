#include <Control.h>
#include <Config.h>

void processSerialCommands() {
  if (SerialBT.available()) {
    //String command = Serial.readStringUntil('\n');
    String command = SerialBT.readStringUntil('\n');
    if (isSerialDebugEnabled) {
      Serial.println("Command read: " + command);
    }
    //SerialBT.println("Command read: " + command);
    command.trim();
    if (command.startsWith("/help")) {
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
    } else if (command.startsWith("/debugBT")) {
      int spaceIndex = command.indexOf(" ");
      if (spaceIndex != -1) {
        String valueStr = command.substring(spaceIndex + 1);
        if (valueStr.equals("true")) {
          isBTdebugEnabled = true;
          SerialBT.println("[INFO] Debug messages via Bluetooth are enabled.");
        } else if (valueStr.equals("false")) {
          isBTdebugEnabled = false;
          SerialBT.println("[INFO] Debug messages via Bluetooth are disabled.");
        } else {
          SerialBT.println("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Bluetooth.");
        }
      } else {
        SerialBT.println("[ERROR] Missing value. Use 'true' or 'false' to enable or disable debug messages via Bluetooth.");
      }
    } else if (command.startsWith("/getMAC")) {
      String macAddress = "Not avaliable";//getMACAddress();
      SerialBT.println("ESP32 MAC Address: " + macAddress);
    } else if (command.startsWith("/debugUSB")) {
      int spaceIndex = command.indexOf(" ");
      if (spaceIndex != -1) {
        String valueStr = command.substring(spaceIndex + 1);
        if (valueStr.equals("true")) {
          isSerialDebugEnabled = true;
          SerialBT.print("[INFO] Debug messages via Serial USB are enabled.");
        } else if (valueStr.equals("false")) {
          isSerialDebugEnabled = false;
          SerialBT.print("[INFO] Debug messages via Serial USB are disabled.");
        } else {
          SerialBT.print("[ERROR] Invalid value. Use 'true' or 'false' to enable or disable debug messages via Serial USB.");
        }
      } else {
        SerialBT.print("[ERROR] Missing value. Use 'true' or 'false' to enable or disable debug messages via Serial USB.");
      }
    } else if (command.startsWith("/single")) {
      String data = "";
      //data = readADCValues();
      SerialBT.println(data);
    } else if (command.startsWith("/start")) {
      // Find the position of the time flag
      int timeFlagPos = command.indexOf("-t");
      if (timeFlagPos != -1) {
        // Extract the duration from the command
        int timeValuePos = timeFlagPos + 2;  // Length of "-t"
        String durationStr = command.substring(timeValuePos);
        int durationInSeconds = durationStr.toInt();
        startRecording(durationInSeconds);
        
      } else {
        Serial.println("[ERROR] Missing -t flag for recording time");
        SerialBT.println("[ERROR] Missing -t flag for recording time");
      }
    } else if (command.startsWith("/checkHardware")) {
      //performHardwareChecks();
    } else if (command.startsWith("/getConfig")) {
      SerialBT.print("[INFO] Current Configuration Parameters:");
      SerialBT.print("[INFO] Device Name: " + String(DEVICE_NAME));
      SerialBT.print("[INFO] Serial Debug Enabled: " + String(isSerialDebugEnabled));
      SerialBT.print("[INFO] Bluetooth Debug Enabled: " + String(isBTdebugEnabled));
      //SerialBT.println("Battery Level: " + String(getBatteryPercentage()) + "%");
  
    } else if (command.startsWith("/listDebugPreferences")) {
      listDebugPreferences();
    } else {
      SerialBT.print("[INFO] Invalid command! Type '/help' to see the list of available commands.");
      return;
    }
    savePreferences();
  }
}

void startRecording(int durationInSeconds) {
  uint32_t lastMeasEndMs = 0;
  unsigned long recordingDurationMs = durationInSeconds * 1000;
  bool isRecording = true;
  unsigned long recordingStartTime = millis();
  while (isRecording) {
    ADS1158_run_and_send_meas();
    if (millis() - recordingStartTime >= recordingDurationMs) {
      isRecording = false;
    }
  }
}