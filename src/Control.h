#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "PreferencesManager.h"
#include "PowerManager.h"
#include "ADS1158Driver.h"

// Function to process serial commands
void processSerialCommands();

// Function to start recording ADC data
void startRecording(int durationInSeconds);

#endif  // CONTROL_H
