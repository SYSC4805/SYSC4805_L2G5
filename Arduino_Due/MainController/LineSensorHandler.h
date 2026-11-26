#pragma once
#include <Arduino.h>

// Two reflectance line followers at the front of the robot:
//   RIGHT = LINE_RIGHT_PIN (A0), LEFT = LINE_LEFT_PIN (A1)

enum LineReadStatus : int8_t {
  LINE_READ_ERROR = -1,
  LINE_CLEAR = 0,
  LINE_BOUNDARY = 1
};

// Which sensor(s) are on the boundary tape
enum LineSide : uint8_t {
  LINE_SIDE_NONE  = 0,
  LINE_SIDE_LEFT  = 1,
  LINE_SIDE_RIGHT = 2,
  LINE_SIDE_BOTH  = 3
};

void lineSensorBegin();

// Raw 12-bit readings (averaged, ADC channel settled)
int lineSensorRawLeft();
int lineSensorRawRight();

// Last raw values sampled by lineSensorRead()
int lineSensorLastRawLeft();
int lineSensorLastRawRight();

// BOUNDARY if either sensor is on the tape (debounced).
// If side is not null it receives which sensor(s) triggered.
LineReadStatus lineSensorRead(LineSide *side = nullptr);

// Manual thresholds (keeps current polarity)
void lineSensorSetThresholds(int left, int right);

// Calibrate one sensor from a floor reading and a tape reading.
// which must be LINE_SIDE_LEFT or LINE_SIDE_RIGHT.
// Threshold = midpoint, polarity from which one reads higher.
// Returns false (and changes nothing) if the readings are too close.
bool lineSensorSetCalibration(LineSide which, int floorRaw, int tapeRaw);

int  lineSensorThresholdLeft();
int  lineSensorThresholdRight();
bool lineSensorBoundaryIsHighLeft();
bool lineSensorBoundaryIsHighRight();

const char* lineSideText(LineSide side);