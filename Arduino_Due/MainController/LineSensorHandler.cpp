#include "LineSensorHandler.h"
#include "Config.h"

static int  thresholdLeft  = DEFAULT_LINE_THRESHOLD_LEFT;
static int  thresholdRight = DEFAULT_LINE_THRESHOLD_RIGHT;

static bool boundaryHighLeft  = DEFAULT_LINE_BOUNDARY_IS_HIGH;
static bool boundaryHighRight = DEFAULT_LINE_BOUNDARY_IS_HIGH;

static uint8_t countLeft  = 0;
static uint8_t countRight = 0;

static int lastRawLeft  = 0;
static int lastRawRight = 0;


// The Due has one ADC shared by all analog pins. The first conversion after
// switching channel can still be influenced by the previous channel, so do
// one throw-away read, then average a few.
static int readAveraged(uint8_t pin) {

  analogRead(pin);

  long sum = 0;

  for (uint8_t i = 0; i < LINE_ADC_AVG; i++) {
    sum += analogRead(pin);
  }

  return (int)(sum / LINE_ADC_AVG);
}

static bool onTape(int raw, int threshold, bool boundaryIsHigh) {

  return boundaryIsHigh ? (raw >= threshold)
                        : (raw <= threshold);
}


void lineSensorBegin() {

  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);

  countLeft  = 0;
  countRight = 0;
}

int lineSensorRawLeft() {
  return readAveraged(LINE_LEFT_PIN);
}

int lineSensorRawRight() {
  return readAveraged(LINE_RIGHT_PIN);
}

int lineSensorLastRawLeft() {
  return lastRawLeft;
}

int lineSensorLastRawRight() {
  return lastRawRight;
}

LineReadStatus lineSensorRead(LineSide *side) {

  int left  = lineSensorRawLeft();
  int right = lineSensorRawRight();

  lastRawLeft  = left;
  lastRawRight = right;

  if (side) {
    *side = LINE_SIDE_NONE;
  }

  if (left < 0 || left > 4095 ||
      right < 0 || right > 4095) {

    return LINE_READ_ERROR;
  }

  // Debounce: a sensor counts as "on the line" only after
  // LINE_DEBOUNCE consecutive samples past its threshold.
  if (onTape(left, thresholdLeft, boundaryHighLeft)) {
    if (countLeft < 255) countLeft++;
  } else {
    countLeft = 0;
  }

  if (onTape(right, thresholdRight, boundaryHighRight)) {
    if (countRight < 255) countRight++;
  } else {
    countRight = 0;
  }

  bool leftOn  = (countLeft  >= LINE_DEBOUNCE);
  bool rightOn = (countRight >= LINE_DEBOUNCE);

  if (side) {

    if (leftOn && rightOn) {
      *side = LINE_SIDE_BOTH;
    } else if (leftOn) {
      *side = LINE_SIDE_LEFT;
    } else if (rightOn) {
      *side = LINE_SIDE_RIGHT;
    }
  }

  return (leftOn || rightOn) ? LINE_BOUNDARY : LINE_CLEAR;
}

void lineSensorSetThresholds(int left, int right) {

  thresholdLeft  = constrain(left, 0, 4095);
  thresholdRight = constrain(right, 0, 4095);
}

bool lineSensorSetCalibration(LineSide which, int floorRaw, int tapeRaw) {

  if (which != LINE_SIDE_LEFT && which != LINE_SIDE_RIGHT) {
    return false;
  }

  int separation = tapeRaw - floorRaw;

  if (separation < 0) {
    separation = -separation;
  }

  if (separation < LINE_MIN_SEPARATION) {
    return false;
  }

  int  threshold = (floorRaw + tapeRaw) / 2;
  bool high      = (tapeRaw > floorRaw);

  if (which == LINE_SIDE_LEFT) {
    thresholdLeft    = threshold;
    boundaryHighLeft = high;
    countLeft        = 0;
  } else {
    thresholdRight    = threshold;
    boundaryHighRight = high;
    countRight        = 0;
  }

  return true;
}

int lineSensorThresholdLeft() {
  return thresholdLeft;
}

int lineSensorThresholdRight() {
  return thresholdRight;
}

bool lineSensorBoundaryIsHighLeft() {
  return boundaryHighLeft;
}

bool lineSensorBoundaryIsHighRight() {
  return boundaryHighRight;
}

const char* lineSideText(LineSide side) {

  switch (side) {
    case LINE_SIDE_LEFT:  return "LEFT";
    case LINE_SIDE_RIGHT: return "RIGHT";
    case LINE_SIDE_BOTH:  return "BOTH";
    default:              return "NONE";
  }
}