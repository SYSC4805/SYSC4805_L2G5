
// Arduino Due + MinIMU-9 v5 (LSM6) + 4 drive motors

#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

  
// I2C PINS
  

const int SDA_PIN = 20;
const int SCL_PIN = 21;


  
// MOTOR PINS
  

const int M1_DIR = 3;
const int M1_EN  = 4;

const int M2_DIR = 5;
const int M2_EN  = 6;

const int M3_DIR = 7;
const int M3_EN  = 8;

const int M4_DIR = 9;
const int M4_EN  = 10;


  
// SETTINGS
  

// Motors
const int TURN_SPEED       = 200;     // was 255: every IMU dropout on hardware happened at PWM 255
const int RAMP_START_SPEED = 120;     // soft start: ramp RAMP_START_SPEED -> TURN_SPEED
const unsigned long RAMP_MS = 250;

// Turn target (degrees, magnitude). Right = negative gyro angle, left = positive.
const float TURN_TARGET = 88.0;

// Gyro
const float GYRO_DPS_PER_LSB = 0.00875;   // +/-245 dps range (enableDefault)
const float GYRO_DEADBAND    = 3.0;

// Invalid-sample detection
const float RAW_INVALID_THRESHOLD = 0.20;     // |RawZ| below this = dead/stale
const int   INVALID_LIMIT         = 2;        // consecutive bad samples before recovery
const unsigned long SLOW_READ_US  = 30000;    // a gyro read slower than this = I2C timeout

// Recovery
const int MAX_RECOVERIES_PER_TURN = 4;

// Calibration
const int   CAL_MAX_SAMPLES  = 500;   // size of the sample buffer
const int   CAL_SAMPLES      = 500;
const int   RECAL_SAMPLES    = 200;
const float CAL_OUTLIER_DPS  = 2.0;   // samples further than this from the median are rejected
                                      // (glitch reads, or the robot being bumped)

// Timing
const unsigned long TURN_TIMEOUT = 3000;   // ms of MOTOR-ON time (safety backup only)
const unsigned long TURN_DELAY   = 3000;   // pause between turns

// Gyro sample flags
const uint8_t GF_SLOW     = 0x01;   // I2C read took too long
const uint8_t GF_FROZEN   = 0x02;   // x,y,z identical to previous sample
const uint8_t GF_NEARZERO = 0x04;   // |RawZ| < RAW_INVALID_THRESHOLD

// Turn results
const int TURN_OK       = 0;
const int TURN_TIMEOUT_ = 1;
const int TURN_IMU_FAIL = 2;

float gyroZOffset = 0.0;
int16_t prevGx = 0, prevGy = 0, prevGz = 0;

int testNumber     = 1;
int totalRecoveries = 0;


  
// I2C RECOVERY
  

void recoverI2CBus() {

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);

  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);

  // 9 clocks: lets a slave that is stuck mid-byte finish and release SDA
  for (int i = 0; i < 9; i++) {

    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);

    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
  }

  // STOP condition
  pinMode(SDA_PIN, OUTPUT);

  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);

  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(10);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  delay(100);
}

// Bus recovery + restart of the Due TWI peripheral (Wire.begin() resets it)
void restartI2C(unsigned long settleMs) {

  recoverI2CBus();

  delay(settleMs);

  Wire.begin();

  delay(settleMs);
}


  
// MOTORS
  

void setMotorSpeed(int speedValue) {

  analogWrite(M1_EN, speedValue);
  analogWrite(M2_EN, speedValue);
  analogWrite(M3_EN, speedValue);
  analogWrite(M4_EN, speedValue);
}

void stopAll() {

  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);
  analogWrite(M3_EN, 0);
  analogWrite(M4_EN, 0);
}

void setTurnDirection(bool right) {

  if (right) {

    // RIGHT motor pattern
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M4_DIR, LOW);

    digitalWrite(M2_DIR, HIGH);
    digitalWrite(M3_DIR, HIGH);

  } else {

    // LEFT motor pattern
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M4_DIR, HIGH);

    digitalWrite(M2_DIR, LOW);
    digitalWrite(M3_DIR, LOW);
  }
}

// Soft start: ramp PWM over RAMP_MS to limit inrush current / supply dip
int rampedSpeed(unsigned long msSinceStart) {

  if (msSinceStart >= RAMP_MS) {
    return TURN_SPEED;
  }

  long span = (long)(TURN_SPEED - RAMP_START_SPEED);

  return RAMP_START_SPEED + (int)(span * (long)msSinceStart / (long)RAMP_MS);
}


  
// GYRO READ WITH VALIDITY FLAGS
  
//
// Returns 0 if the sample is good, otherwise a combination of GF_* flags.
// rawZ is always filled in (deg/s) so it can be printed.

uint8_t readGyroZ(float &rawZ) {

  unsigned long t0 = micros();

  imu.readGyro();

  unsigned long readUs = micros() - t0;

  int16_t gx = imu.g.x;
  int16_t gy = imu.g.y;
  int16_t gz = imu.g.z;

  rawZ = gz * GYRO_DPS_PER_LSB;

  uint8_t flags = 0;

  if (readUs > SLOW_READ_US) {
    flags |= GF_SLOW;
  }

  if (gx == prevGx && gy == prevGy && gz == prevGz) {
    flags |= GF_FROZEN;
  }

  if (fabs(rawZ) < RAW_INVALID_THRESHOLD) {
    flags |= GF_NEARZERO;
  }

  prevGx = gx;
  prevGy = gy;
  prevGz = gz;

  return flags;
}

const char* flagText(uint8_t flags) {

  if (flags == 0)            return "OK";
  if (flags & GF_SLOW)       return "I2C-SLOW";
  if (flags & GF_FROZEN)     return "FROZEN";
  return "NEAR-ZERO";
}


  
// GYRO CALIBRATION
  
//
// Robust against glitch reads: collect samples, take the median, reject
// anything further than CAL_OUTLIER_DPS from it, average the rest.
// (A mean/min/max approach is ruined by a single bad read that returns ~0
// when the real value is the bias, e.g. -4.35.)
//
// Fails if too many samples are rejected (robot moving or bus glitching).
// On success gyroZOffset is updated; on failure it is left untouched.

float calBuf[CAL_MAX_SAMPLES];

bool calibrateGyro(int samples) {

  if (samples > CAL_MAX_SAMPLES) {
    samples = CAL_MAX_SAMPLES;
  }

  int good      = 0;
  int slowCount = 0;

  for (int i = 0; i < samples; i++) {

    float rz;

    uint8_t f = readGyroZ(rz);

    if (f & GF_SLOW) {

      slowCount++;

      if (slowCount >= 3) {
        break;   // bus is dead - don't spend a minute timing out
      }
    }

    if (!(f & (GF_SLOW | GF_FROZEN))) {

      calBuf[good] = rz;
      good++;
    }

    delay(5);
  }

  if (good < (samples * 8) / 10) {

    Serial.print("  [CAL] FAILED: only ");
    Serial.print(good);
    Serial.print(" of ");
    Serial.print(samples);
    Serial.println(" samples valid (I2C reads slow/frozen)");

    return false;
  }

  // insertion sort -> median, min, max
  for (int i = 1; i < good; i++) {

    float v = calBuf[i];
    int j = i - 1;

    while (j >= 0 && calBuf[j] > v) {
      calBuf[j + 1] = calBuf[j];
      j--;
    }

    calBuf[j + 1] = v;
  }

  float median = calBuf[good / 2];
  float minZ   = calBuf[0];
  float maxZ   = calBuf[good - 1];

  // mean of inliers
  float sum    = 0.0;
  int   inlier = 0;

  for (int i = 0; i < good; i++) {

    if (fabs(calBuf[i] - median) <= CAL_OUTLIER_DPS) {

      sum += calBuf[i];
      inlier++;
    }
  }

  int rejected = good - inlier;

  Serial.print("  [CAL] median ");
  Serial.print(median, 3);
  Serial.print(" | min ");
  Serial.print(minZ, 2);
  Serial.print(" | max ");
  Serial.print(maxZ, 2);
  Serial.print(" | rejected ");
  Serial.print(rejected);
  Serial.print(" of ");
  Serial.println(good);

  if (inlier < (samples * 8) / 10) {

    Serial.println("  [CAL] FAILED: too many outliers (robot moving or I2C glitching)");

    return false;
  }

  gyroZOffset = sum / inlier;

  return true;
}


  
// IMU INIT / HEALTH CHECK / RECOVERY
  

bool initImu() {

  for (int attempt = 1; attempt <= 5; attempt++) {

    if (imu.init()) {

      imu.enableDefault();

      return true;
    }

    Serial.println("Retrying IMU...");

    delay(200);
  }

  return false;
}

// Alive = reads are fast and values are changing
bool imuIsAlive() {

  int good = 0;

  for (int i = 0; i < 10; i++) {

    float rz;

    uint8_t f = readGyroZ(rz);

    if (!(f & (GF_SLOW | GF_FROZEN))) {
      good++;
    }

    delay(5);
  }

  return (good >= 8);
}

// Call with motors stopped (this function stops them anyway).
bool recoverImu() {

  stopAll();

  Serial.println("  [RECOVERY] motors stopped, resetting I2C bus + IMU");

  delay(400);   // let the robot stop rotating and motor current settle

  for (int attempt = 1; attempt <= 3; attempt++) {

    Serial.print("  [RECOVERY] attempt ");
    Serial.println(attempt);

    restartI2C(100);

    if (!initImu()) {

      Serial.println("  [RECOVERY] IMU init FAILED");

      continue;
    }

    if (!imuIsAlive()) {

      Serial.println("  [RECOVERY] IMU init ok but data not updating");

      continue;
    }

    float oldOffset = gyroZOffset;

    if (calibrateGyro(RECAL_SAMPLES)) {

      Serial.print("  [RECOVERY] gyro offset ");
      Serial.print(oldOffset, 4);
      Serial.print(" -> ");
      Serial.println(gyroZOffset, 4);

    } else {

      Serial.println("  [RECOVERY] recalibration skipped, keeping old offset");
    }

    totalRecoveries++;

    Serial.println("  [RECOVERY] SUCCESS");

    return true;
  }

  Serial.println("  [RECOVERY] FAILED");

  return false;
}


  
// GYRO-CONTROLLED TURN (both directions)
  
//
// right = true  -> right turn, gyro Z negative
// right = false -> left turn,  gyro Z positive
//
// Returns TURN_OK, TURN_TIMEOUT_ or TURN_IMU_FAIL.

int turnGyro(bool right) {

  const char* name = right ? "RIGHT" : "LEFT";

  // sign converts gyro Z into "progress in the commanded direction"
  //   left  : +Z  -> sign = +1
  //   right : -Z  -> sign = -1  (progress = -Z)
  const float sign = right ? -1.0 : 1.0;

  float turned   = 0.0;    // degrees turned in the commanded direction (>0 = progress)
  float lastRate = 0.0;    // last good progress rate (deg/s), used only to estimate blind time
  float estDeg   = 0.0;    // degrees that were ESTIMATED while the IMU was not answering

  int invalidCount = 0;
  int recoveries   = 0;

  unsigned long activeBefore  = 0;   // motor-on ms from earlier segments (before a recovery)
  unsigned long lastPrint     = 0;

  setTurnDirection(right);

  unsigned long segmentStart = millis();   // start of the current motor-on segment (ramp ref)
  unsigned long previousTime = micros();


  while (true) {

    unsigned long segMs    = millis() - segmentStart;
    unsigned long activeMs = activeBefore + segMs;

    int pwm = rampedSpeed(segMs);

    setMotorSpeed(pwm);


    // ---- read gyro ----
    float rawZ;

    uint8_t flags = readGyroZ(rawZ);

    unsigned long now = micros();

    float dt = (now - previousTime) / 1000000.0;

    previousTime = now;

    bool valid = (flags == 0);

    float correctedZ = 0.0;


    if (valid) {

      invalidCount = 0;

      correctedZ = rawZ - gyroZOffset;

      if (fabs(correctedZ) < GYRO_DEADBAND) {
        correctedZ = 0.0;
      }

      lastRate = sign * correctedZ;

      turned += lastRate * dt;

    } else {

      // NEVER integrate (rawZ - offset) from a dead/stale sample.
      // A slow read is a definite I2C timeout -> trip immediately.
      if (flags & GF_SLOW) {
        invalidCount = INVALID_LIMIT;
      } else {
        invalidCount++;
      }

      // The robot is still rotating while the IMU is silent: estimate that
      // interval from the last good rate so the turn doesn't overshoot.
      if (lastRate > 0.0) {

        float est = lastRate * dt;

        turned += est;
        estDeg += est;
      }
    }


    // ---- debug print ----
    if (millis() - lastPrint >= 100) {

      lastPrint = millis();

      Serial.print(name);
      Serial.print(" | RawZ: ");
      Serial.print(rawZ, 2);

      Serial.print(" | CorrectedZ: ");

      if (valid) {
        Serial.print(correctedZ, 2);
      } else {
        Serial.print("n/a");
      }

      Serial.print(" | Angle: ");
      Serial.print(sign * turned, 2);

      Serial.print(" | Time: ");
      Serial.print(activeMs);

      Serial.print(" | PWM: ");
      Serial.print(pwm);

      Serial.print(" | ");
      Serial.println(flagText(flags));
    }


    // ---- target reached ----
    if (turned >= TURN_TARGET) {

      stopAll();

      Serial.println();
      Serial.print("*** ");
      Serial.print(name);
      Serial.println(" TARGET REACHED ***");

      Serial.print("Final angle: ");
      Serial.println(sign * turned, 2);

      Serial.print("Time: ");
      Serial.print(activeMs);
      Serial.println(" ms");

      if (recoveries > 0 || estDeg > 0.0) {

        Serial.print("NOTE: turn included ");
        Serial.print(recoveries);
        Serial.print(" IMU recovery(ies), ");
        Serial.print(estDeg, 1);
        Serial.println(" deg ESTIMATED (not measured) - do not count toward accuracy");
      }

      return TURN_OK;
    }


    // ---- safety timeout (backup only) ----
    if (activeMs >= TURN_TIMEOUT) {

      stopAll();

      Serial.println();
      Serial.print("*** ");
      Serial.print(name);
      Serial.println(" TIMEOUT ***");

      Serial.print("Final gyro angle: ");
      Serial.println(sign * turned, 2);

      return TURN_TIMEOUT_;
    }


    // ---- IMU judged dead: stop, recover, resume ----
    if (invalidCount >= INVALID_LIMIT) {

      stopAll();

      activeBefore += (millis() - segmentStart);

      Serial.println();
      Serial.print("*** ");
      Serial.print(name);
      Serial.print(" IMU DEAD (");
      Serial.print(flagText(flags));
      Serial.print(") at angle ");
      Serial.print(sign * turned, 2);
      Serial.println(" ***");

      if (recoveries >= MAX_RECOVERIES_PER_TURN) {

        Serial.println("Too many recoveries in one turn.");

        return TURN_IMU_FAIL;
      }

      if (!recoverImu()) {

        return TURN_IMU_FAIL;
      }

      recoveries++;

      // resume the same turn
      invalidCount = 0;
      lastRate     = 0.0;

      Serial.print("Resuming ");
      Serial.print(name);
      Serial.println(" turn");

      setTurnDirection(right);

      segmentStart = millis();     // restart soft-start ramp
      previousTime = micros();     // don't integrate across the recovery

      continue;
    }


    delay(5);
  }
}


  
// ONE TURN + REPORTING
  

void runTurn(bool right) {

  Serial.println();
  Serial.println(right ? "RIGHT TURN" : "LEFT TURN");

  int result = turnGyro(right);

  stopAll();

  if (result == TURN_IMU_FAIL) {

    Serial.println();
    Serial.println("*** IMU UNRECOVERABLE - motors off, test halted ***");
    Serial.println("*** Power-cycle and check wiring / power / pull-ups ***");

    while (1) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("STOPPED");

  Serial.print("Total IMU recoveries so far: ");
  Serial.println(totalRecoveries);

  delay(TURN_DELAY);
}


  
// SETUP
  

void setup() {

  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("REPEATED GYRO TURN TEST (v2)");
  Serial.println("================================");


  // Motor setup
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT);

  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT);

  stopAll();


  // I2C recovery + start
  recoverI2CBus();

  delay(500);

  Wire.begin();

  delay(1000);


  // IMU initialization
  Serial.println("Initializing IMU...");

  if (!initImu()) {

    Serial.println("IMU FAILED");

    stopAll();

    while (1) {
      delay(1000);
    }
  }

  Serial.println("IMU detected.");


  // Gyro calibration (retry if the robot moved or the bus glitched)
  Serial.println();
  Serial.println("================================");
  Serial.println("KEEP ROBOT COMPLETELY STILL");
  Serial.println("CALIBRATING GYRO");
  Serial.println("================================");

  delay(2000);

  bool calibrated = false;

  for (int attempt = 1; attempt <= 3; attempt++) {

    if (calibrateGyro(CAL_SAMPLES)) {

      calibrated = true;

      break;
    }

    Serial.println("Calibration retry...");

    delay(1000);
  }

  if (!calibrated) {

    Serial.println("CALIBRATION FAILED - halting");

    while (1) {
      delay(1000);
    }
  }

  Serial.print("Gyro Z offset = ");
  Serial.println(gyroZOffset, 4);

  Serial.println("Calibration complete.");

  delay(3000);
}


  
// LOOP
  

void loop() {

  Serial.println();
  Serial.println("================================");
  Serial.print("TEST CYCLE ");
  Serial.println(testNumber);
  Serial.println("================================");

  runTurn(true);    // RIGHT

  runTurn(false);   // LEFT

  testNumber++;
}