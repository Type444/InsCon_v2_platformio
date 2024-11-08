#ifndef PREFERENCES_MANAGER_H
#define PREFERENCES_MANAGER_H

#include <Preferences.h>
#include <BluetoothSerial.h>
#include <Arduino.h>

// Preferences functions
void loadOrSetDefaultPreferences();
void resetPreferencesToDefaults();
void savePreferences();
void listPreferences();
void listDebugPreferences();

#endif  // PREFERENCES_MANAGER_H
