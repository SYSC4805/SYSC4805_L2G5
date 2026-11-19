#include "LineSensorHandler.h"
#include "Config.h"

static int thresholdValue =
    DEFAULT_LINE_THRESHOLD;

void lineSensorBegin() {
  pinMode(LINE_SENSOR_PIN, INPUT);
}

int lineSensorRaw() {
  return analogRead(LINE_SENSOR_PIN);
}

LineReadStatus lineSensorRead() {
  int raw = lineSensorRaw();

  if (raw < 0 || raw > 4095) {
    return LINE_READ_ERROR;
  }

  if (raw >= thresholdValue) {
    return LINE_BOUNDARY;
  }

  return LINE_CLEAR;
}

void lineSensorSetThreshold(int threshold) {
  thresholdValue =
      constrain(threshold, 0, 4095);
}

int lineSensorThreshold() {
  return thresholdValue;
}