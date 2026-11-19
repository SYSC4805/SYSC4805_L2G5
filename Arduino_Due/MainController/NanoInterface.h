#pragma once
#include <Arduino.h>

void nanoInterfaceBegin();
bool nanoIR1Event();
bool nanoIR2Event();
bool nanoUltrasonicEvent();
bool nanoRangeEvent();
void nanoHeartbeatUpdate();
bool nanoHeartbeatHealthy();
