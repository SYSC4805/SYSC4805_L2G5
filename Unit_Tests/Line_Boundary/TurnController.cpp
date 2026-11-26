#include "TurnController.h"
#include "Config.h"
#include "MotorHandler.h"
#include "IMUHandler.h"

static bool (*abortCheck)() = nullptr;


void turnControllerSetAbortCheck(bool (*fn)()) {
  abortCheck = fn;
}


bool turnControllerBegin() {

  motorBegin();

  if (!imuBegin()) {

    Serial.println("[TURN] IMU not detected");

    return false;
  }

  for (int attempt = 1; attempt <= 3; attempt++) {

    if (imuCalibrate(500)) {

      Serial.print("[TURN] gyro Z offset = ");
      Serial.println(imuGyroOffset(), 4);

      return true;
    }

    Serial.println("[TURN] calibration retry...");

    delay(1000);
  }

  return false;
}


// Soft start: ramp PWM over TURN_RAMP_MS to limit inrush current
static uint8_t rampedPwm(unsigned long msSinceStart) {

  if (msSinceStart >= TURN_RAMP_MS) {
    return TURN_PWM;
  }

  long span = (long)TURN_PWM - (long)TURN_RAMP_START_PWM;

  return (uint8_t)((long)TURN_RAMP_START_PWM
                   + span * (long)msSinceStart / (long)TURN_RAMP_MS);
}


static void drivePivot(bool left, uint8_t pwm) {

  if (left) {
    pivotLeft(pwm);
  } else {
    pivotRight(pwm);
  }
}


static TurnResult turnGyro(bool left, float degrees, TurnReport *report) {

  const char* name = left ? "LEFT" : "RIGHT";

  // progress = sign * gyroZ    (left: +Z, right: -Z)
  const float sign = left ? 1.0f : -1.0f;

  float goal = degrees - TURN_COAST_DEG;

  if (goal < 1.0f) {
    goal = 1.0f;
  }

  float turned   = 0.0f;   // degrees turned in the commanded direction
  float lastRate = 0.0f;   // last good progress rate (deg/s)
  float estDeg   = 0.0f;   // degrees estimated while the IMU was silent

  uint8_t invalidCount = 0;
  uint8_t recoveries   = 0;

  float   maxRate      = 0.0f;   // largest |rate| seen in this motor-on segment
  uint8_t validSamples = 0;      // good samples in this segment (capped)

  unsigned long activeBefore = 0;   // motor-on ms before a recovery
  unsigned long lastPrint    = 0;

  unsigned long segmentStart = millis();
  unsigned long previousUs   = micros();

  unsigned long activeMs = 0;

  TurnResult result = TURN_OK;

  while (true) {

    if (abortCheck && abortCheck()) {

      stopAllMotors();

      result = TURN_ABORTED;

      break;
    }

    unsigned long segMs = millis() - segmentStart;

    activeMs = activeBefore + segMs;

    uint8_t pwm = rampedPwm(segMs);

    drivePivot(left, pwm);


    // ---- read gyro ----
    float rawZ;
    float correctedZ;

    uint8_t flags = imuReadGyro(rawZ, correctedZ);

    unsigned long nowUs = micros();

    float dt = (nowUs - previousUs) / 1000000.0f;

    previousUs = nowUs;

    bool valid = (flags == 0);

    if (valid) {

      invalidCount = 0;

      if (fabs(correctedZ) > maxRate) {
        maxRate = fabs(correctedZ);
      }

      if (validSamples < 255) {
        validSamples++;
      }

      if (fabs(correctedZ) < TURN_GYRO_DEADBAND_DPS) {
        correctedZ = 0.0f;
      }

      lastRate = sign * correctedZ;

      turned += lastRate * dt;

    } else {

      // Never integrate a dead/stale sample. A slow read is a definite
      // I2C timeout, so trip immediately.
      if (flags & IMU_FLAG_SLOW) {
        invalidCount = 2;
      } else if (invalidCount < 255) {
        invalidCount++;
      }

      // The robot keeps rotating while the IMU is silent: estimate that
      // interval from the last good rate.
      if (lastRate > 0.0f) {

        float est = lastRate * dt;

        turned += est;
        estDeg += est;
      }
    }


    if (TURN_VERBOSE && millis() - lastPrint >= 100) {

      lastPrint = millis();

      Serial.print("[TURN] ");
      Serial.print(name);
      Serial.print(" | raw ");
      Serial.print(rawZ, 2);
      Serial.print(" | angle ");
      Serial.print(sign * turned, 2);
      Serial.print(" | ");
      Serial.print(activeMs);
      Serial.print(" ms | PWM ");
      Serial.print(pwm);
      Serial.print(" | ");
      Serial.println(imuFlagText(flags));
    }


    // ---- target reached ----
    if (turned >= goal) {

      stopAllMotors();

      result = TURN_OK;

      break;
    }


    // ---- safety timeout (backup only) ----
    if (activeMs >= TURN_TIMEOUT_MS) {

      stopAllMotors();

      result = TURN_TIMEOUT;

      break;
    }


    // ---- IMU judged dead: stop, recover, resume ----
    if (invalidCount >= 2) {

      stopAllMotors();

      activeBefore += millis() - segmentStart;

      Serial.print("[TURN] ");
      Serial.print(name);
      Serial.print(" IMU dead (");
      Serial.print(imuFlagText(flags));
      Serial.print(") at ");
      Serial.print(sign * turned, 1);
      Serial.println(" deg");

      if (recoveries >= TURN_MAX_RECOVERIES || !imuRecover()) {

        result = TURN_IMU_FAIL;

        break;
      }

      recoveries++;

      invalidCount = 0;
      lastRate     = 0.0f;
      maxRate      = 0.0f;
      validSamples = 0;

      segmentStart = millis();   // restart soft start
      previousUs   = micros();   // don't integrate across the recovery

      continue;
    }


    // ---- motors commanded but the robot is not rotating ----
    // (checked after the IMU-dead logic so a dead IMU is never mistaken
    //  for "no motion")
    if (segMs >= TURN_NOMOTION_CHECK_MS &&
        validSamples >= TURN_NOMOTION_MIN_SAMPLES &&
        maxRate < TURN_NOMOTION_MIN_DPS) {

      stopAllMotors();

      Serial.print("[TURN] ");
      Serial.print(name);
      Serial.print(" NO ROTATION: max gyro rate only ");
      Serial.print(maxRate, 1);
      Serial.print(" deg/s with PWM ");
      Serial.print(pwm);
      Serial.println(" - check battery, motor power, wheels on the ground");

      result = TURN_NO_MOTION;

      break;
    }

    delay(5);
  }


  stopAllMotors();

  Serial.print("[TURN] ");
  Serial.print(name);
  Serial.print(" ");
  Serial.print(degrees, 0);
  Serial.print(" deg: ");
  Serial.print(turnResultText(result));
  Serial.print(" | gyro ");
  Serial.print(turned, 1);
  Serial.print(" deg | ");
  Serial.print(activeMs);
  Serial.print(" ms | recoveries ");
  Serial.print(recoveries);

  if (estDeg > 0.0f) {
    Serial.print(" | ESTIMATED ");
    Serial.print(estDeg, 1);
    Serial.print(" deg");
  }

  Serial.println();

  if (report) {

    report->measuredDeg  = turned;
    report->estimatedDeg = estDeg;
    report->motorOnMs    = activeMs;
    report->recoveries   = recoveries;
  }

  return result;
}


TurnResult turnLeft(float degrees, TurnReport *report) {
  return turnGyro(true, degrees, report);
}

TurnResult turnRight(float degrees, TurnReport *report) {
  return turnGyro(false, degrees, report);
}

TurnResult turnAround(bool leftward, TurnReport *report) {
  return turnGyro(leftward, 180.0f, report);
}

const char* turnResultText(TurnResult result) {

  switch (result) {
    case TURN_OK:       return "OK";
    case TURN_TIMEOUT:  return "TIMEOUT";
    case TURN_IMU_FAIL: return "IMU FAILED";
    case TURN_ABORTED:  return "ABORTED";
    case TURN_NO_MOTION: return "NO ROTATION (no motor power / stalled?)";
    default:            return "?";
  }
}