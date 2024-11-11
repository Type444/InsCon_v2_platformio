#include "PreferencesManager.h"
#include "Config.h"

PreferencesManager::PreferencesManager(const char* namespaceName) : namespaceName(namespaceName) {}

void PreferencesManager::begin() {
    preferences.begin(namespaceName, false);
}

void PreferencesManager::end() {
    preferences.end();
}

bool PreferencesManager::getBoolPreference(const String& key, bool defaultValue) {
    begin();
    bool value = preferences.getBool(key.c_str(), defaultValue);
    end();
    return value;
}

uint16_t PreferencesManager::getUIntPreference(const String& key, uint16_t defaultValue) {
    begin();
    uint16_t value = preferences.getUInt(key.c_str(), defaultValue);
    end();
    return value;
}

void PreferencesManager::setBoolPreference(const String& key, bool value) {
    begin();
    preferences.putBool(key.c_str(), value);
    end();
}

void PreferencesManager::setUIntPreference(const String& key, uint16_t value) {
    begin();
    preferences.putUInt(key.c_str(), value);
    end();
}

void PreferencesManager::loadOrSetDefaultPreferences() {
    begin();

    uint16_t ActiveChannels = preferences.getUInt("Channels", ActiveChannels);
    isSerialDebugEnabled = preferences.getBool("DebugUSB", isSerialDebugEnabled);
    isBTdebugEnabled = preferences.getBool("DebugBT", isBTdebugEnabled);

    if (isBTdebugEnabled) {
        SerialBT.println("[DEBG] Preferences loaded.");
    }
    if (isSerialDebugEnabled) {
        Serial.println("Preferences loaded.");
    }

    end();
}

void PreferencesManager::resetPreferencesToDefaults() {
    begin();
    preferences.clear();

    uint16_t  ActiveChannels = ACTIVE_ADC_CHANNELS;
    isSerialDebugEnabled = true;
    isBTdebugEnabled = true;

    end();
}

void PreferencesManager::savePreferences() {
    uint16_t ActiveChannels = ACTIVE_ADC_CHANNELS;
    begin();
    preferences.putUInt("Channels", ActiveChannels);
    preferences.putBool("DebugUSB", isSerialDebugEnabled);
    preferences.putBool("DebugBT", isBTdebugEnabled);

    if (isBTdebugEnabled) {
        SerialBT.println("[INFO] Preferences saved.");
    }

    end();
}

void PreferencesManager::listPreferences() {
    begin();

    SerialBT.println("[INFO] Current Configuration Preferences:");
    SerialBT.println("[INFO] NumberOfChannels: " + String(preferences.getUInt("Channels", 0)));
    SerialBT.println("[INFO] isSerialDebugEnabled: " + String(preferences.getBool("DebugUSB", true)));
    SerialBT.println("[INFO] isBTdebugEnabled: " + String(preferences.getBool("DebugBT", true)));

    end();
}

void PreferencesManager::listDebugPreferences()
{
  begin();

  // Read the preferences
  bool isSerialDebugEnabled = preferences.getBool("DebugUSB", true);
  bool isBTdebugEnabled = preferences.getBool("DebugBT", true);

  // Close the preferences storage
  end();

  // Convert boolean values to '1' or '0' and concatenate them into a string
  String configString = "[CNFG]";
  configString += (isSerialDebugEnabled ? '1' : '0');
  configString += (isBTdebugEnabled ? '1' : '0');
  configString += DEVICE_NAME;

  // Output the configuration string to SerialBT
  SerialBT.print(configString);
}