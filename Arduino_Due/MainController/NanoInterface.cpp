#include "NanoInterface.h"
#include "Config.h"

static bool previousHeartbeat = LOW;
static unsigned long lastHeartbeatChangeMs = 0;

void nanoInterfaceBegin() {
  pinMode(NANO_IR1_PIN, INPUT);
  pinMode(NANO_IR2_PIN, INPUT);
  pinMode(NANO_ULTRASONIC_PIN, INPUT);
  pinMode(NANO_RANGE_PIN, INPUT);
  pinMode(NANO_HEARTBEAT_PIN, INPUT);

  previousHeartbeat = digitalRead(NANO_HEARTBEAT_PIN);
  lastHeartbeatChangeMs = millis();
}

bool nanoIR1Event()        { return digitalRead(NANO_IR1_PIN) == HIGH; }
bool nanoIR2Event()        { return digitalRead(NANO_IR2_PIN) == HIGH; }
bool nanoUltrasonicEvent() { return digitalRead(NANO_ULTRASONIC_PIN) == HIGH; }
bool nanoRangeEvent()      { return digitalRead(NANO_RANGE_PIN) == HIGH; }

void nanoHeartbeatUpdate() {
  bool now = digitalRead(NANO_HEARTBEAT_PIN);
  if (now != previousHeartbeat) {
    previousHeartbeat = now;
    lastHeartbeatChangeMs = millis();
  }
}

bool nanoHeartbeatHealthy() {
  return (millis() - lastHeartbeatChangeMs) <= NANO_HEARTBEAT_TIMEOUT_MS;
}
