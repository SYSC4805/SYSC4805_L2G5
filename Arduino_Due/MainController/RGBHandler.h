#pragma once
#include <Arduino.h>

enum RobotState : uint8_t {
  INITIALIZE,

  SEARCH_NAVIGATE,

  PLOW_CLEAR,

  AVOID_OBSTACLE,

  BOUNDARY_RECOVERY,

  LOW_BATTERY_WARNING,

  FAULT_SAFE_STOP
};

void rgbBegin();

void rgbShowState(RobotState state);

void rgbUpdateBlink();