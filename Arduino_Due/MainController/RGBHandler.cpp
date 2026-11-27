#include "RGBHandler.h"
#include "Config.h"

static RobotState currentRgbState = INITIALIZE;

static bool blinkOn = false;
static unsigned long lastBlinkMs = 0;


// ============================================================
// SET RGB
// ============================================================

static void setRgb(
    uint8_t r,
    uint8_t g,
    uint8_t b
) {
  analogWrite(RGB_R_PIN, r);
  analogWrite(RGB_G_PIN, g);
  analogWrite(RGB_B_PIN, b);
}


// ============================================================
// INITIALIZATION
// ============================================================

void rgbBegin() {

  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);

  setRgb(0, 0, 0);

  blinkOn = false;
  lastBlinkMs = millis();
}


// ============================================================
// DISPLAY STATE
// ============================================================

void rgbShowState(
    RobotState state
) {
  currentRgbState = state;

  blinkOn = true;
  lastBlinkMs = millis();

  switch (state) {

    case INITIALIZE:
      // Blue
      setRgb(0, 0, 255);
      break;


    case SEARCH_NAVIGATE:
      // Green
      setRgb(0, 255, 0);
      break;


    case PLOW_CLEAR:
      // White
      setRgb(255, 255, 255);
      break;


    case AVOID_OBSTACLE:
      // Yellow
      setRgb(255, 255, 0);
      break;


    case BOUNDARY_RECOVERY:
      // Purple
      setRgb(255, 0, 255);
      break;


    case LOW_BATTERY_WARNING:
      // Flashing orange
      setRgb(255, 80, 0);
      break;


    case FAULT_SAFE_STOP:
      // Flashing red
      setRgb(255, 0, 0);
      break;
  }
}


// ============================================================
// BLINKING STATES
// ============================================================

void rgbUpdateBlink() {

  if (
      currentRgbState != LOW_BATTERY_WARNING &&
      currentRgbState != FAULT_SAFE_STOP
  ) {
    return;
  }


  if (
      millis() - lastBlinkMs <
      RGB_BLINK_MS
  ) {
    return;
  }


  lastBlinkMs = millis();

  blinkOn = !blinkOn;


  if (!blinkOn) {

    setRgb(0, 0, 0);

    return;
  }


  if (
      currentRgbState ==
      LOW_BATTERY_WARNING
  ) {

    // Orange
    setRgb(255, 80, 0);
  }

  else {

    // Red
    setRgb(255, 0, 0);
  }
}