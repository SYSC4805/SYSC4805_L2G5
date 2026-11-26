#include <Wire.h>
#include <LSM6.h>
#include "IMUHandler.h"
#include "Config.h"

static LSM6 imu;

static float zOffset = 0.0f;
static bool imuReady = false;

static int16_t prevGx = 0;
static int16_t prevGy = 0;
static int16_t prevGz = 0;

static const uint16_t CAL_MAX_SAMPLES = 500;
static float calBuf[CAL_MAX_SAMPLES];


void recoverI2CBus() {

  pinMode(DUE_SDA_PIN, INPUT_PULLUP);
  pinMode(DUE_SCL_PIN, OUTPUT);

  digitalWrite(DUE_SCL_PIN, HIGH);
  delayMicroseconds(10);

  // 9 clock pulses to recover a stuck I2C slave
  for (int i = 0; i < 9; i++) {

    digitalWrite(DUE_SCL_PIN, LOW);
    delayMicroseconds(10);

    digitalWrite(DUE_SCL_PIN, HIGH);
    delayMicroseconds(10);
  }

  // Generate STOP condition
  pinMode(DUE_SDA_PIN, OUTPUT);

  digitalWrite(DUE_SDA_PIN, LOW);
  delayMicroseconds(10);

  digitalWrite(DUE_SCL_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(DUE_SDA_PIN, HIGH);
  delayMicroseconds(10);

  pinMode(DUE_SDA_PIN, INPUT_PULLUP);
  pinMode(DUE_SCL_PIN, INPUT_PULLUP);

  delay(100);
}


static bool initImuChip() {

  for (int attempt = 0; attempt < 5; attempt++) {

    if (imu.init()) {

      imu.enableDefault();

      imuReady = true;

      prevGx = 0;
      prevGy = 0;
      prevGz = 0;

      return true;
    }

    delay(250);
  }

  imuReady = false;

  return false;
}


bool imuBegin() {

  recoverI2CBus();

  Wire.begin();

  delay(100);

  return initImuChip();
}


uint8_t imuReadGyro(float &rawDps, float &correctedDps) {

  rawDps       = 0.0f;
  correctedDps = 0.0f;

  if (!imuReady) {
    return IMU_FLAG_SLOW;
  }

  unsigned long t0 = micros();

  imu.readGyro();

  unsigned long readUs = micros() - t0;

  int16_t gx = (int16_t)imu.g.x;
  int16_t gy = (int16_t)imu.g.y;
  int16_t gz = (int16_t)imu.g.z;

  rawDps       = gz * IMU_DPS_PER_LSB;
  correctedDps = rawDps - zOffset;

  uint8_t flags = 0;

  if (readUs > IMU_SLOW_READ_US) {
    flags |= IMU_FLAG_SLOW;
  }

  if (gx == prevGx && gy == prevGy && gz == prevGz) {
    flags |= IMU_FLAG_FROZEN;
  }

  if (fabs(rawDps) < IMU_RAW_INVALID_DPS) {
    flags |= IMU_FLAG_NEARZERO;
  }

  prevGx = gx;
  prevGy = gy;
  prevGz = gz;

  return flags;
}


bool imuReadYawRate(float &degPerSec) {

  float raw;

  uint8_t flags = imuReadGyro(raw, degPerSec);

  return (flags == 0);
}


// Median + outlier rejection. NEARZERO is not rejected here because a
// healthy gyro at rest reads its bias, which could legitimately be small.
bool imuCalibrate(uint16_t samples) {

  if (!imuReady || samples == 0) {
    return false;
  }

  if (samples > CAL_MAX_SAMPLES) {
    samples = CAL_MAX_SAMPLES;
  }

  uint16_t good = 0;
  uint8_t  slowCount = 0;

  for (uint16_t i = 0; i < samples; i++) {

    float raw;
    float corrected;

    uint8_t f = imuReadGyro(raw, corrected);

    if (f & IMU_FLAG_SLOW) {

      slowCount++;

      if (slowCount >= 3) {
        break;   // bus is dead - don't spend a minute timing out
      }
    }

    if (!(f & (IMU_FLAG_SLOW | IMU_FLAG_FROZEN))) {

      calBuf[good] = raw;
      good++;
    }

    delay(5);
  }

  if (good < (samples * 8UL) / 10UL) {

    Serial.print("[IMU] cal FAILED: only ");
    Serial.print(good);
    Serial.print(" of ");
    Serial.print(samples);
    Serial.println(" samples valid");

    return false;
  }

  // insertion sort -> median
  for (uint16_t i = 1; i < good; i++) {

    float v = calBuf[i];
    int   j = (int)i - 1;

    while (j >= 0 && calBuf[j] > v) {
      calBuf[j + 1] = calBuf[j];
      j--;
    }

    calBuf[j + 1] = v;
  }

  float median = calBuf[good / 2];

  float sum    = 0.0f;
  uint16_t inliers = 0;

  for (uint16_t i = 0; i < good; i++) {

    if (fabs(calBuf[i] - median) <= IMU_CAL_OUTLIER_DPS) {

      sum += calBuf[i];
      inliers++;
    }
  }

  Serial.print("[IMU] cal median ");
  Serial.print(median, 3);
  Serial.print(" | min ");
  Serial.print(calBuf[0], 2);
  Serial.print(" | max ");
  Serial.print(calBuf[good - 1], 2);
  Serial.print(" | rejected ");
  Serial.print(good - inliers);
  Serial.print(" of ");
  Serial.println(good);

  if (inliers < (samples * 8UL) / 10UL) {

    Serial.println("[IMU] cal FAILED: too many outliers (robot moving or I2C glitching)");

    return false;
  }

  zOffset = sum / inliers;

  return true;
}


// Alive = reads are fast and values are changing
static bool imuIsAlive() {

  int good = 0;

  for (int i = 0; i < 10; i++) {

    float raw;
    float corrected;

    uint8_t f = imuReadGyro(raw, corrected);

    if (!(f & (IMU_FLAG_SLOW | IMU_FLAG_FROZEN))) {
      good++;
    }

    delay(5);
  }

  return (good >= 8);
}


bool imuRecover() {

  Serial.println("[IMU] recovering: reset I2C bus + re-init");

  delay(400);   // robot has stopped rotating, motor current has settled

  for (int attempt = 1; attempt <= 3; attempt++) {

    Serial.print("[IMU] recovery attempt ");
    Serial.println(attempt);

    recoverI2CBus();

    delay(100);

    Wire.begin();

    delay(100);

    if (!initImuChip()) {

      Serial.println("[IMU] init FAILED");

      continue;
    }

    if (!imuIsAlive()) {

      Serial.println("[IMU] init ok but data not updating");

      continue;
    }

    float oldOffset = zOffset;

    if (imuCalibrate(200)) {

      Serial.print("[IMU] gyro offset ");
      Serial.print(oldOffset, 4);
      Serial.print(" -> ");
      Serial.println(zOffset, 4);

    } else {

      Serial.println("[IMU] recalibration skipped, keeping old offset");
    }

    Serial.println("[IMU] recovery SUCCESS");

    return true;
  }

  Serial.println("[IMU] recovery FAILED");

  return false;
}


float imuGyroOffset() {
  return zOffset;
}


const char* imuFlagText(uint8_t flags) {

  if (flags == 0)               return "OK";
  if (flags & IMU_FLAG_SLOW)    return "I2C-SLOW";
  if (flags & IMU_FLAG_FROZEN)  return "FROZEN";
  return "NEAR-ZERO";
}