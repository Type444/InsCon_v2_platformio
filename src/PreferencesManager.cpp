#include <Arduino.h>
#include <Config.h>

Preferences preferences;

void loadOrSetDefaultPreferences() {
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Check if the preferences exist

  // Load and set the values from EEPROM
  
  uint16_t ActiveChannels = preferences.getUInt("Channels", ActiveChannels);
  isSerialDebugEnabled = preferences.getBool("DebugUSB", isSerialDebugEnabled);
  isBTdebugEnabled = preferences.getBool("DebugBT", isBTdebugEnabled);

  // Load and set other configuration values similarly
  if (isBTdebugEnabled) {
    SerialBT.println("[DEBG] Preferences loaded.");
  }
  if (isSerialDebugEnabled) {
    Serial.println("Preferences loaded.");
  }
  // Close the preferences storage
  preferences.end();
}

void resetPreferencesToDefaults() {
  // Open the preferences storage
  preferences.begin("my-app", false);

  // Remove all preferences
  preferences.clear();

  // Set all values to their default values
  uint16_t ActiveChannels = ACTIVE_ADC_CHANNELS;  // For future use
  isSerialDebugEnabled = true;                        // Set to your desired default value
  isBTdebugEnabled = true;                            // Set to your desired default value

  // Set other configuration values to their defaults similarly

  // Close the preferences storage
  preferences.end();
}

void savePreferences() {
  // Open the preferences storage
  uint16_t ActiveChannels = ACTIVE_ADC_CHANNELS;

  if (!preferences.begin("my-app", false)) {
    SerialBT.println("[INFO] Failed to open preferences for saving");
    return;
  }
  // Save the current configuration values to EEPROM
  
  preferences.putUInt("Channels", ActiveChannels);
  preferences.putBool("DebugUSB", isSerialDebugEnabled);
  preferences.putBool("DebugBT", isBTdebugEnabled);

  // Save other configuration values similarly
  if (isBTdebugEnabled) {
    SerialBT.println("[INFO] Preferences saved.");
  }
  // Close the preferences storage
  preferences.end();
}

void listPreferences() {
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



void listDebugPreferences() {
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

