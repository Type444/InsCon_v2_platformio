#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// Function to get battery percentage
uint8_t getBatteryPercentage();

// Function to lookup battery percentage based on voltage
int lookupPercentage(float voltage);

#endif  // POWER_MANAGER_H
