#include "SafetyHandler.h"
#include "Config.h"

static volatile bool
    emergencyLatched = false;

static void emergencyStopISR() {
  emergencyLatched = true;
}

void safetyBegin() {
  pinMode(
      ESTOP_PIN,
      INPUT_PULLUP
  );

  attachInterrupt(
      digitalPinToInterrupt(
          ESTOP_PIN
      ),
      emergencyStopISR,
      FALLING
  );
}

bool emergencyStopActive() {
  return (
      emergencyLatched ||
      digitalRead(ESTOP_PIN) == LOW
  );
}

void safetyClearEmergencyStop() {
  if (
      digitalRead(ESTOP_PIN) ==
      HIGH
  ) {
    emergencyLatched = false;
  }
}