#pragma once
#include <Arduino.h>

enum LineReadStatus : int8_t {
  LINE_READ_ERROR = -1,
  LINE_CLEAR = 0,
  LINE_BOUNDARY = 1
};

void lineSensorBegin();
int lineSensorRaw();
LineReadStatus lineSensorRead();
void lineSensorSetThreshold(int threshold);
int lineSensorThreshold();
