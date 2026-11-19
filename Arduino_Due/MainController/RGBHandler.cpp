#include "RGBHandler.h"
#include "Config.h"

static RobotState displayedState = INITIALIZE;
static bool blinkOn = true;
static unsigned long lastBlinkMs = 0;

static void setRGB(bool r, bool g, bool b) {
  // Assumes common-cathode RGB LED.
  digitalWrite(RGB_R_PIN, r ? HIGH : LOW);
  digitalWrite(RGB_G_PIN, g ? HIGH : LOW);
  digitalWrite(RGB_B_PIN, b ? HIGH : LOW);
}

void rgbBegin() {
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  setRGB(false, false, false);
}

void rgbShowState(RobotState state) {
  displayedState = state;

  switch (state) {
    case INITIALIZE:           setRGB(false, false, true); break; // blue
    case SEARCH_NAVIGATE:      setRGB(false, true, false); break; // green
    case PLOW_CLEAR:           setRGB(true, true, true); break;    // white
    case AVOID_OBSTACLE:       setRGB(true, true, false); break;   // yellow
    case BOUNDARY_RECOVERY:    setRGB(true, false, true); break;   // purple
    case LOW_BATTERY_WARNING:  break; // flashing orange in update
    case FAULT_SAFE_STOP:      break; // flashing red in update
  }
}

void rgbUpdateBlink() {
  if (displayedState != LOW_BATTERY_WARNING &&
      displayedState != FAULT_SAFE_STOP) {
    return;
  }

  if (millis() - lastBlinkMs < 300) return;
  lastBlinkMs = millis();
  blinkOn = !blinkOn;

  if (!blinkOn) {
    setRGB(false, false, false);
  } else if (displayedState == LOW_BATTERY_WARNING) {
    setRGB(true, true, false);  // orange approximation on digital RGB
  } else {
    setRGB(true, false, false); // red
  }
}
