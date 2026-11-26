#pragma once
#include <Arduino.h>

// IMU + motor turn module.
//
// Uses MotorHandler (pivotLeft / pivotRight / stopAllMotors) and
// IMUHandler (validity-checked gyro, I2C recovery). If the IMU stops
// answering mid-turn it stops the motors, recovers the IMU and resumes
// the same turn.
//
// Gyro sign convention (verified): LEFT = +Z, RIGHT = -Z.

enum TurnResult : int8_t {
  TURN_OK       = 0,
  TURN_TIMEOUT  = 1,   // safety backup: motor-on time exceeded TURN_TIMEOUT_MS
  TURN_IMU_FAIL = 2,   // IMU could not be recovered
  TURN_ABORTED  = 3,   // abort callback returned true (e-stop, heartbeat...)
  TURN_NO_MOTION = 4   // motors commanded but the gyro sees no rotation
};

struct TurnReport {
  float measuredDeg;         // gyro-measured turn (incl. estimated part)
  float estimatedDeg;        // part that was ESTIMATED while the IMU was silent
  unsigned long motorOnMs;
  uint8_t recoveries;        // IMU recoveries during this turn
};

// Motors + IMU init + gyro calibration. Robot must be still. Returns false on failure.
bool turnControllerBegin();

// Optional: called every loop iteration while turning. Return true to abort
// the turn (motors are stopped and TURN_ABORTED is returned).
void turnControllerSetAbortCheck(bool (*fn)());

// Turn by `degrees` (positive magnitude). report may be nullptr.
TurnResult turnLeft(float degrees, TurnReport *report = nullptr);
TurnResult turnRight(float degrees, TurnReport *report = nullptr);

// 180-degree turn (leftward = true -> counter-clockwise)
TurnResult turnAround(bool leftward = true, TurnReport *report = nullptr);

const char* turnResultText(TurnResult result);