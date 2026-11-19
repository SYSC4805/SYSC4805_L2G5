#include "Config.h"

#include "MotorHandler.h"

#include "IMUHandler.h"

#include "LineSensorHandler.h"

#include "BatteryHandler.h"

#include "RGBHandler.h"

#include "NanoInterface.h"

#include "SafetyHandler.h"


RobotState currentState =
    INITIALIZE;

unsigned long runStartedMs = 0;

unsigned long stateEnteredMs = 0;


// ============================================================
// STATE CONTROL
// ============================================================

static void enterState(
    RobotState next
) {
  currentState = next;

  stateEnteredMs = millis();

  rgbShowState(next);

  if (
      next == AVOID_OBSTACLE ||
      next == LOW_BATTERY_WARNING ||
      next == FAULT_SAFE_STOP
  ) {
    stopAllMotors();
  }
}


// ============================================================
// TIMED TURN FALLBACKS
// ============================================================

static void turnLeft90Timed() {
  pivotLeft(MOTOR_PWM);

  delay(
      LEFT_90_FALLBACK_MS
  );

  stopAllMotors();
}


static void turnRight90Timed() {
  pivotRight(MOTOR_PWM);

  delay(
      RIGHT_90_FALLBACK_MS
  );

  stopAllMotors();
}


// ============================================================
// IMU CONTROLLED TURN
// ============================================================

static bool turnByIMU(
    float targetDegrees,
    bool turnLeft
) {
  float accumulatedAngle = 0.0f;

  unsigned long previousTime =
      millis();

  unsigned long startTime =
      millis();

  if (turnLeft) {
    pivotLeft(MOTOR_PWM);
  }
  else {
    pivotRight(MOTOR_PWM);
  }

  while (
      fabs(accumulatedAngle) <
      targetDegrees
  ) {

    float gyroRate = 0.0f;

    if (
        !imuReadYawRate(
            gyroRate
        )
    ) {
      stopAllMotors();

      return false;
    }

    unsigned long now =
        millis();

    float dt =
        (now - previousTime)
        / 1000.0f;

    previousTime = now;

    accumulatedAngle +=
        gyroRate * dt;

    nanoHeartbeatUpdate();

    if (
        !nanoHeartbeatHealthy()
    ) {
      stopAllMotors();

      return false;
    }

    // Turn safety timeout
    if (
        now - startTime >
        6000
    ) {
      stopAllMotors();

      return false;
    }

    delay(2);
  }

  stopAllMotors();

  return true;
}


// ============================================================
// 180 DEGREE TURN
// ============================================================

static void turn180() {

  bool successful =
      turnByIMU(
          180.0f,
          true
      );

  if (!successful) {

    // Temporary timed fallback if
    // IMU communication becomes unreliable.
    pivotLeft(MOTOR_PWM);

    delay(
        TURN_180_FALLBACK_MS
    );

    stopAllMotors();
  }
}


// ============================================================
// BOUNDARY RECOVERY
// ============================================================

static void performBoundaryRecovery() {

  stopAllMotors();

  delay(100);

  // Reverse slightly if rear path is clear.
  if (!nanoRearBlocked()) {

    driveReverse(
        MOTOR_PWM
    );

    delay(300);
  }

  stopAllMotors();

  delay(100);

  // Report architecture:
  // use gyroscope to perform 180 degree turn.
  turn180();

  delay(100);

  // Move sideways into next strip.
  if (!nanoRightBlocked()) {

    pivotRight(
        MOTOR_PWM
    );

    delay(250);
  }
  else if (!nanoLeftBlocked()) {

    pivotLeft(
        MOTOR_PWM
    );

    delay(250);
  }

  stopAllMotors();
}


// ============================================================
// OBSTACLE AVOIDANCE
// ============================================================

static void performObstacleAvoidance() {

  stopAllMotors();

  // SR-01:
  // wait 2 seconds in case
  // obstacle is moving.
  delay(
      OBSTACLE_HOLD_MS
  );

  // If obstacle moved away,
  // resume navigation.
  if (!nanoFrontBlocked()) {
    return;
  }

  // Static obstacle.
  bool leftClear =
      !nanoLeftBlocked();

  bool rightClear =
      !nanoRightBlocked();


  // ==========================================================
  // TRY RIGHT
  // ==========================================================

  if (rightClear) {

    turnRight90Timed();

    driveForward(
        MOTOR_PWM
    );

    unsigned long start =
        millis();

    // Continue until original obstacle
    // is no longer detected beside robot.
    while (nanoLeftBlocked()) {

      nanoHeartbeatUpdate();

      if (
          !nanoHeartbeatHealthy()
      ) {
        stopAllMotors();

        enterState(
            FAULT_SAFE_STOP
        );

        return;
      }

      if (
          millis() - start >
          3000
      ) {
        stopAllMotors();

        enterState(
            FAULT_SAFE_STOP
        );

        return;
      }

      delay(5);
    }

    stopAllMotors();

    turnLeft90Timed();

    return;
  }


  // ==========================================================
  // TRY LEFT
  // ==========================================================

  if (leftClear) {

    turnLeft90Timed();

    driveForward(
        MOTOR_PWM
    );

    unsigned long start =
        millis();

    while (nanoRightBlocked()) {

      nanoHeartbeatUpdate();

      if (
          !nanoHeartbeatHealthy()
      ) {
        stopAllMotors();

        enterState(
            FAULT_SAFE_STOP
        );

        return;
      }

      if (
          millis() - start >
          3000
      ) {
        stopAllMotors();

        enterState(
            FAULT_SAFE_STOP
        );

        return;
      }

      delay(5);
    }

    stopAllMotors();

    turnRight90Timed();

    return;
  }


  // Neither side clear.
  enterState(
      FAULT_SAFE_STOP
  );
}


// ============================================================
// INITIALIZATION
// ============================================================

static bool initializeRobot() {

  stopAllMotors();

  if (!imuBegin()) {

    Serial.println(
        "Initialization failed: IMU not detected"
    );

    return false;
  }

  if (
      !imuCalibrate(500)
  ) {

    Serial.println(
        "Initialization failed: IMU calibration"
    );

    return false;
  }

  Serial.print(
      "Gyro Z offset = "
  );

  Serial.println(
      imuGyroOffset(),
      4
  );

  return true;
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  motorBegin();

  rgbBegin();

  lineSensorBegin();

  batteryBegin();

  nanoInterfaceBegin();

  safetyBegin();


  enterState(
      INITIALIZE
  );


  if (!initializeRobot()) {

    enterState(
        FAULT_SAFE_STOP
    );

    return;
  }


  runStartedMs =
      millis();


  // Report state transition:
  // INITIALIZE -> SEARCH/NAVIGATE
  enterState(
      SEARCH_NAVIGATE
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  nanoHeartbeatUpdate();

  rgbUpdateBlink();


  // ==========================================================
  // SAFETY CHECKS
  // ==========================================================

  if (
      emergencyStopActive()
  ) {

    Serial.println(
        "FAULT: emergency stop"
    );

    enterState(
        FAULT_SAFE_STOP
    );
  }


  if (
      !nanoHeartbeatHealthy()
  ) {

    Serial.println(
        "FAULT: Nano heartbeat lost"
    );

    enterState(
        FAULT_SAFE_STOP
    );
  }


  if (
      batteryIsCritical()
  ) {

    Serial.println(
        "FAULT: critical battery"
    );

    enterState(
        FAULT_SAFE_STOP
    );
  }

  else if (
      batteryIsLow() &&
      currentState !=
          LOW_BATTERY_WARNING &&
      currentState !=
          FAULT_SAFE_STOP
  ) {

    Serial.println(
        "WARNING: low battery"
    );

    enterState(
        LOW_BATTERY_WARNING
    );
  }


  if (
      millis() -
      runStartedMs >=
      MAX_RUN_MS &&
      currentState !=
          FAULT_SAFE_STOP
  ) {

    Serial.println(
        "5-minute run limit reached"
    );

    enterState(
        FAULT_SAFE_STOP
    );
  }


  // ==========================================================
  // STATE MACHINE
  // ==========================================================

  switch (currentState) {


    // --------------------------------------------------------
    // INITIALIZE
    // --------------------------------------------------------

    case INITIALIZE:

      if (initializeRobot()) {

        runStartedMs =
            millis();

        enterState(
            SEARCH_NAVIGATE
        );
      }

      else {

        enterState(
            FAULT_SAFE_STOP
        );
      }

      break;


    // --------------------------------------------------------
    // SEARCH / NAVIGATE
    // --------------------------------------------------------

    case SEARCH_NAVIGATE: {

      LineReadStatus line =
          lineSensorRead();


      if (
          line ==
          LINE_READ_ERROR
      ) {

        enterState(
            FAULT_SAFE_STOP
        );

        break;
      }


      if (
          line ==
          LINE_BOUNDARY
      ) {

        enterState(
            BOUNDARY_RECOVERY
        );

        break;
      }


      if (
          nanoFrontBlocked()
      ) {

        enterState(
            AVOID_OBSTACLE
        );

        break;
      }


      /*
       * Fixed plow:
       *
       * Forward navigation simultaneously
       * performs the snow-clearing action.
       *
       * PLOW_CLEAR represents the active
       * clearing portion of a strip.
       */

      enterState(
          PLOW_CLEAR
      );

      break;
    }


    // --------------------------------------------------------
    // PLOW / CLEAR
    // --------------------------------------------------------

    case PLOW_CLEAR: {

      LineReadStatus line =
          lineSensorRead();


      if (
          line ==
          LINE_READ_ERROR
      ) {

        enterState(
            FAULT_SAFE_STOP
        );

        break;
      }


      if (
          line ==
          LINE_BOUNDARY
      ) {

        enterState(
            BOUNDARY_RECOVERY
        );

        break;
      }


      if (
          nanoFrontBlocked()
      ) {

        enterState(
            AVOID_OBSTACLE
        );

        break;
      }


      driveForward(
          MOTOR_PWM
      );

      break;
    }


    // --------------------------------------------------------
    // AVOID OBSTACLE
    // --------------------------------------------------------

    case AVOID_OBSTACLE:

      performObstacleAvoidance();

      if (
          currentState !=
          FAULT_SAFE_STOP
      ) {

        enterState(
            SEARCH_NAVIGATE
        );
      }

      break;


    // --------------------------------------------------------
    // BOUNDARY RECOVERY
    // --------------------------------------------------------

    case BOUNDARY_RECOVERY:

      performBoundaryRecovery();

      if (
          currentState !=
          FAULT_SAFE_STOP
      ) {

        enterState(
            SEARCH_NAVIGATE
        );
      }

      break;


    // --------------------------------------------------------
    // LOW BATTERY
    // --------------------------------------------------------

    case LOW_BATTERY_WARNING:

      stopAllMotors();


      if (
          batteryIsCritical()
      ) {

        enterState(
            FAULT_SAFE_STOP
        );
      }

      else if (
          !batteryIsLow()
      ) {

        enterState(
            INITIALIZE
        );
      }

      break;


    // --------------------------------------------------------
    // FAULT / SAFE STOP
    // --------------------------------------------------------

    case FAULT_SAFE_STOP:

      stopAllMotors();

      nanoHeartbeatUpdate();


      /*
       * Report state chart:
       * Fault resolved -> INITIALIZE
       */

      if (
          !emergencyStopActive() &&
          nanoHeartbeatHealthy() &&
          !batteryIsCritical()
      ) {

        delay(500);


        if (
            !emergencyStopActive() &&
            nanoHeartbeatHealthy() &&
            !batteryIsCritical()
        ) {

          safetyClearEmergencyStop();

          enterState(
              INITIALIZE
          );
        }
      }

      break;
  }


  delay(5);
}