#ifndef PREFERENCES_MANAGER_H
#define PREFERENCES_MANAGER_H

#include <Preferences.h>
#include <BluetoothSerial.h>

extern BluetoothSerial SerialBT;

class PreferencesManager {
public:
    PreferencesManager(const char* namespaceName);

    void begin();
    void end();

    bool getBoolPreference(const String& key, bool defaultValue);
    uint16_t getUIntPreference(const String& key, uint16_t defaultValue);

    void setBoolPreference(const String& key, bool value);
    void setUIntPreference(const String& key, uint16_t value);

    void loadOrSetDefaultPreferences();
    void resetPreferencesToDefaults();
    void savePreferences();
    void listPreferences();
    void listDebugPreferences();

private:
    Preferences preferences;
    const char* namespaceName;
};

#endif // PREFERENCES_MANAGER_H