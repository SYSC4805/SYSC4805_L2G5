#include "RGBHandler.h"
#include "Config.h"

static RobotState displayedState = INITIALIZE;
static bool blinkOn = true;
static unsigned long lastBlinkMs = 0;

static void writeRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RGB_R_PIN, r);
  analogWrite(RGB_G_PIN, g);
  analogWrite(RGB_B_PIN, b);
}

void rgbBegin() {
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  writeRGB(0, 0, 0);
}

void rgbShowState(RobotState state) {
  displayedState = state;

  switch (state) {
    case INITIALIZE:
      writeRGB(0, 0, 255);
      break;

    case SEARCH_NAVIGATE:
      writeRGB(0, 255, 0);
      break;

    case PLOW_CLEAR:
      writeRGB(255, 255, 255);
      break;

    case AVOID_OBSTACLE:
      writeRGB(255, 255, 0);
      break;

    case BOUNDARY_RECOVERY:
      writeRGB(255, 0, 255);
      break;

    case LOW_BATTERY_WARNING:
    case FAULT_SAFE_STOP:
      break;
  }
}

void rgbUpdateBlink() {
  if (displayedState != LOW_BATTERY_WARNING &&
      displayedState != FAULT_SAFE_STOP) {
    return;
  }

  if (millis() - lastBlinkMs < RGB_BLINK_MS) return;

  lastBlinkMs = millis();
  blinkOn = !blinkOn;

  if (!blinkOn) {
    writeRGB(0, 0, 0);
    return;
  }

  if (displayedState == LOW_BATTERY_WARNING) {
    writeRGB(255, 80, 0);
  } else {
    writeRGB(255, 0, 0);
  }
}