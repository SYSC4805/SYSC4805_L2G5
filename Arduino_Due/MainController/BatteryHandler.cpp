#include "BatteryHandler.h"
#include "Config.h"

void batteryBegin() {
  pinMode(BATTERY_PIN, INPUT);
}

float batteryVoltage() {
  int raw =
      analogRead(BATTERY_PIN);

  float adcVoltage =
      (raw / ADC_MAX) * ADC_REF_V;

  float dividerRatio =
      (BATTERY_R_TOP + BATTERY_R_BOTTOM)
      / BATTERY_R_BOTTOM;

  return adcVoltage * dividerRatio;
}

bool batteryIsLow() {
  float voltage =
      batteryVoltage();

  return voltage <= LOW_BATTERY_V &&
         voltage > CRITICAL_BATTERY_V;
}

bool batteryIsCritical() {
  return batteryVoltage()
         <= CRITICAL_BATTERY_V;
}