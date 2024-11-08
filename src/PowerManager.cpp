#include <Arduino.h>
#include <Config.h>

uint8_t getBatteryPercentage() {
  digitalWrite(BAT_ADC_EN, HIGH);
  delay(500);
  const int analogInPin = 34;  // ADC1_CH6 (GPIO 34)
  int adcValue = analogRead(BAT_ADC);
  float voltage = adcValue / 4095.0 * 3.3;
  int batteryPercentage = lookupPercentage(voltage);
  digitalWrite(BAT_ADC_EN, LOW);
  return batteryPercentage;
}


int lookupPercentage(float voltage) {
    // Define a lookup table with voltage levels and corresponding battery percentages for 1S
  const float voltageTable[] = {
    4.2, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95, 3.91, 3.87, 3.85, 3.84, 3.82, 3.8, 3.79, 3.77, 3.75, 3.73, 3.71, 3.69, 3.61, 3.27
  };
  const int percentageTable[] = {
    100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0
  };
  // Check if voltage is outside the table range
  if (voltage > voltageTable[0]) {
    return 100;
  }
  if (voltage < voltageTable[sizeof(voltageTable) / sizeof(voltageTable[0]) - 1]) {
    return 0;
  }

  // Find the nearest voltage level in the table
  int i;
  for (i = 0; i < sizeof(voltageTable) / sizeof(voltageTable[0]) - 1; i++) {
    if (voltage >= voltageTable[i]) {
      break;
    }
  }

  // Perform linear interpolation between the two nearest voltage points
  float percentage = map(voltage, voltageTable[i], voltageTable[i + 1], percentageTable[i], percentageTable[i + 1]);

  return int(percentage); // Convert to integer percentage
}