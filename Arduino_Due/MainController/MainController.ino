#include "Config.h"
#include "MotorHandler.h"
#include "IMUHandler.h"
#include "LineSensorHandler.h"
#include "BatteryHandler.h"
#include "RGBHandler.h"
#include "NanoInterface.h"
#include "SafetyHandler.h"

RobotState currentState = INITIALIZE;
unsigned long stateEnteredMs = 0;
unsigned long runStartedMs = 0;

static void enterState(RobotState next) {
  currentState = next;
  stateEnteredMs = millis();
  rgbShowState(next);

  if (next == FAULT_SAFE_STOP ||
      next == LOW_BATTERY_WARNING ||
      next == AVOID_OBSTACLE) {
    stopAllMotors();
  }
}

static bool obstacleDetected() {
  return nanoIR1Event() ||
         nanoIR2Event() ||
         nanoUltrasonicEvent() ||
         nanoRangeEvent();
}

static void timedPivotLeft90() {
  pivotLeft(MOTOR_PWM);
  delay(LEFT_90_FALLBACK_MS);
  stopAllMotors();
}

static void timedPivotRight90() {
  pivotRight(MOTOR_PWM);
  delay(RIGHT_90_FALLBACK_MS);
  stopAllMotors();
}

static void timedTurn180() {
  pivotLeft(MOTOR_PWM);
  delay(TURN_180_FALLBACK_MS);
  stopAllMotors();
}

static void performBoundaryRecovery() {
  stopAllMotors();
  delay(100);

  driveReverse(MOTOR_PWM);
  delay(300);
  stopAllMotors();
  delay(100);

  timedTurn180();

  // Small lateral offset before the next strip.
  pivotRight(MOTOR_PWM);
  delay(250);
  stopAllMotors();
}

static void performObstacleAvoidance() {
  // Requirement SR-01: remain stopped for 2 seconds when obstacle is within 10 cm.
  stopAllMotors();
  delay(OBSTACLE_HOLD_MS);

  // Simple deterministic avoidance manoeuvre.
  timedPivotRight90();
  driveForward(MOTOR_PWM);
  delay(450);
  stopAllMotors();
  timedPivotLeft90();
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

  if (!imuBegin()) {
    Serial.println("FAULT: IMU initialization failed");
    enterState(FAULT_SAFE_STOP);
    return;
  }

  if (!imuCalibrate(500)) {
    Serial.println("FAULT: IMU calibration failed");
    enterState(FAULT_SAFE_STOP);
    return;
  }

  Serial.print("Gyro Z offset = ");
  Serial.println(imuGyroOffset(), 4);

  runStartedMs = millis();
  enterState(SEARCH_NAVIGATE);
}

void loop() {
  nanoHeartbeatUpdate();
  rgbUpdateBlink();

  // Highest-priority safety conditions.
  if (emergencyStopActive()) {
    Serial.println("FAULT: emergency stop");
    enterState(FAULT_SAFE_STOP);
  }

  if (!nanoHeartbeatHealthy()) {
    Serial.println("FAULT: Nano heartbeat lost");
    enterState(FAULT_SAFE_STOP);
  }

  if (batteryIsLow() &&
      currentState != FAULT_SAFE_STOP &&
      currentState != LOW_BATTERY_WARNING) {
    Serial.println("WARNING: low battery");
    enterState(LOW_BATTERY_WARNING);
  }

  if (millis() - runStartedMs >= MAX_RUN_MS &&
      currentState != FAULT_SAFE_STOP) {
    Serial.println("Run complete: 5-minute limit reached");
    enterState(FAULT_SAFE_STOP);
  }

  switch (currentState) {
    case INITIALIZE:
      // setup() handles initialization.
      break;

    case SEARCH_NAVIGATE: {
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

      if (obstacleDetected()) {
        enterState(AVOID_OBSTACLE);
        break;
      }

      // The plow is mechanically fixed, so forward strip motion simultaneously
      // performs the clearing action.
      driveForward(MOTOR_PWM);
      break;
    }

    case PLOW_CLEAR:
      // Retained as a named state to match the project statechart.
      // With a fixed plow, active clearing is the same forward motion used
      // during a strip pass.
      driveForward(MOTOR_PWM);

      if (lineSensorRead() == LINE_BOUNDARY) {
        enterState(BOUNDARY_RECOVERY);
      } else if (obstacleDetected()) {
        enterState(AVOID_OBSTACLE);
      }
      break;

    case AVOID_OBSTACLE:
      performObstacleAvoidance();
      enterState(SEARCH_NAVIGATE);
      break;

    case BOUNDARY_RECOVERY:
      performBoundaryRecovery();
      enterState(SEARCH_NAVIGATE);
      break;

    case LOW_BATTERY_WARNING:
      stopAllMotors();

      // Battery recovery is intentionally conservative. The robot only returns
      // to INITIALIZE if the voltage rises above the threshold.
      if (!batteryIsLow()) {
        enterState(INITIALIZE);
      }
      break;

    case FAULT_SAFE_STOP:
      stopAllMotors();
      break;
  }

  delay(5);
}
