#include "NanoInterface.h"
#include "Config.h"

static bool previousHeartbeat = LOW;
static unsigned long lastHeartbeatChangeMs = 0;

void nanoInterfaceBegin() {
  pinMode(NANO_IR_LEFT_PIN, INPUT);
  pinMode(NANO_IR_RIGHT_PIN, INPUT);
  pinMode(NANO_FRONT_PIN, INPUT);
  pinMode(NANO_REAR_PIN, INPUT);
  pinMode(NANO_HEARTBEAT_PIN, INPUT);

  previousHeartbeat = digitalRead(NANO_HEARTBEAT_PIN);
  lastHeartbeatChangeMs = millis();
}

bool nanoLeftBlocked() {
  return digitalRead(NANO_IR_LEFT_PIN) == HIGH;
}

bool nanoRightBlocked() {
  return digitalRead(NANO_IR_RIGHT_PIN) == HIGH;
}

bool nanoFrontBlocked() {
  return digitalRead(NANO_FRONT_PIN) == HIGH;
}

bool nanoRearBlocked() {
  return digitalRead(NANO_REAR_PIN) == HIGH;
}

void nanoHeartbeatUpdate() {
  bool now = digitalRead(NANO_HEARTBEAT_PIN);

  if (now != previousHeartbeat) {
    previousHeartbeat = now;
    lastHeartbeatChangeMs = millis();
  }
}

bool nanoHeartbeatHealthy() {
  return (millis() - lastHeartbeatChangeMs)
      <= NANO_HEARTBEAT_TIMEOUT_MS;
}