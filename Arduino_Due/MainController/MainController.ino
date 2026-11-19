#include "Config.h"
#include "MotorHandler.h"
#include "IMUHandler.h"
#include "LineSensorHandler.h"
#include "BatteryHandler.h"
#include "RGBHandler.h"
#include "NanoInterface.h"
#include "SafetyHandler.h"

RobotState currentState = INITIALIZE;

unsigned long runStartedMs = 0;
unsigned long stateEnteredMs = 0;

static void enterState(RobotState next) {
  currentState = next;
  stateEnteredMs = millis();

  rgbShowState(next);

  if (next == AVOID_OBSTACLE ||
      next == LOW_BATTERY_WARNING ||
      next == FAULT_SAFE_STOP) {
    stopAllMotors();
  }
}

static void turnLeft90Timed() {
  pivotLeft(MOTOR_PWM);
  delay(LEFT_90_FALLBACK_MS);
  stopAllMotors();
}

static void turnRight90Timed() {
  pivotRight(MOTOR_PWM);
  delay(RIGHT_90_FALLBACK_MS);
  stopAllMotors();
}

static void turn180Timed() {
  pivotLeft(MOTOR_PWM);
  delay(TURN_180_FALLBACK_MS);
  stopAllMotors();
}

static void performBoundaryRecovery() {
  stopAllMotors();
  delay(100);

  if (!nanoRearBlocked()) {
    driveReverse(MOTOR_PWM);
    delay(300);
  }

  stopAllMotors();
  delay(100);

  turn180Timed();

  if (!nanoRightBlocked()) {
    pivotRight(MOTOR_PWM);
    delay(250);
  } else if (!nanoLeftBlocked()) {
    pivotLeft(MOTOR_PWM);
    delay(250);
  }

  stopAllMotors();
}

static void performObstacleAvoidance() {
  stopAllMotors();

  delay(OBSTACLE_HOLD_MS);

  if (!nanoFrontBlocked()) {
    return;
  }

  bool leftClear  = !nanoLeftBlocked();
  bool rightClear = !nanoRightBlocked();

  if (rightClear) {
    turnRight90Timed();

    driveForward(MOTOR_PWM);
    delay(450);
    stopAllMotors();

    turnLeft90Timed();
    return;
  }

  if (leftClear) {
    turnLeft90Timed();

    driveForward(MOTOR_PWM);
    delay(450);
    stopAllMotors();

    turnRight90Timed();
    return;
  }

  enterState(FAULT_SAFE_STOP);
}

static bool initializeRobot() {
  stopAllMotors();

  if (!imuBegin()) {
    Serial.println("Initialization failed: IMU not detected");
    return false;
  }

  if (!imuCalibrate(500)) {
    Serial.println("Initialization failed: IMU calibration");
    return false;
  }

  Serial.print("Gyro Z offset = ");
  Serial.println(imuGyroOffset(), 4);

  return true;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  motorBegin();
  rgbBegin();
  lineSensorBegin();
  batteryBegin();
  nanoInterfaceBegin();
  safetyBegin();

  enterState(INITIALIZE);

  if (!initializeRobot()) {
    enterState(FAULT_SAFE_STOP);
    return;
  }

  runStartedMs = millis();

  enterState(PLOW_CLEAR);
}

void loop() {
  nanoHeartbeatUpdate();
  rgbUpdateBlink();

  if (emergencyStopActive()) {
    Serial.println("FAULT: emergency stop");
    enterState(FAULT_SAFE_STOP);
  }

  if (!nanoHeartbeatHealthy()) {
    Serial.println("FAULT: Nano heartbeat lost");
    enterState(FAULT_SAFE_STOP);
  }

  if (batteryIsCritical()) {
    Serial.println("FAULT: critical battery voltage");
    enterState(FAULT_SAFE_STOP);
  } else if (batteryIsLow() &&
             currentState != LOW_BATTERY_WARNING &&
             currentState != FAULT_SAFE_STOP) {

    Serial.println("WARNING: low battery");
    enterState(LOW_BATTERY_WARNING);
  }

  if (millis() - runStartedMs >= MAX_RUN_MS &&
      currentState != FAULT_SAFE_STOP) {

    Serial.println("5-minute run limit reached");
    enterState(FAULT_SAFE_STOP);
  }

  switch (currentState) {

    case INITIALIZE:
      if (initializeRobot()) {
        runStartedMs = millis();
        enterState(PLOW_CLEAR);
      } else {
        enterState(FAULT_SAFE_STOP);
      }
      break;

    case SEARCH_NAVIGATE:
      enterState(PLOW_CLEAR);
      break;

    case PLOW_CLEAR: {
      LineReadStatus line = lineSensorRead();

      if (line == LINE_READ_ERROR) {
        Serial.println("FAULT: line sensor read error");
        enterState(FAULT_SAFE_STOP);
        break;
      }

      if (line == LINE_BOUNDARY) {
        enterState(BOUNDARY_RECOVERY);
        break;
      }

      if (nanoFrontBlocked()) {
        enterState(AVOID_OBSTACLE);
        break;
      }

      driveForward(MOTOR_PWM);
      break;
    }

    case AVOID_OBSTACLE:
      performObstacleAvoidance();

      if (currentState != FAULT_SAFE_STOP) {
        enterState(PLOW_CLEAR);
      }
      break;

    case BOUNDARY_RECOVERY:
      performBoundaryRecovery();

      if (currentState != FAULT_SAFE_STOP) {
        enterState(SEARCH_NAVIGATE);
      }
      break;

    case LOW_BATTERY_WARNING:
      stopAllMotors();

      if (batteryIsCritical()) {
        enterState(FAULT_SAFE_STOP);
      } else if (!batteryIsLow()) {
        enterState(INITIALIZE);
      }
      break;

    case FAULT_SAFE_STOP:
      stopAllMotors();
      break;
  }

  delay(5);
}