#include "SafetyHandler.h"
#include "Config.h"

static volatile bool eStopLatched = false;

static void emergencyStopISR() {
  eStopLatched = true;
}

void safetyBegin() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), emergencyStopISR, FALLING);
}

bool emergencyStopActive() {
  return eStopLatched || digitalRead(ESTOP_PIN) == LOW;
}

void safetyLatchEmergencyStop() {
  eStopLatched = true;
}

bool safetyFaultLatched() {
  return eStopLatched;
}

void safetyClearFaultLatch() {
  if (digitalRead(ESTOP_PIN) == HIGH) {
    eStopLatched = false;
  }
}
