#pragma once
#include <Arduino.h>

void safetyBegin();
bool emergencyStopActive();
void safetyLatchEmergencyStop();
bool safetyFaultLatched();
void safetyClearFaultLatch();
