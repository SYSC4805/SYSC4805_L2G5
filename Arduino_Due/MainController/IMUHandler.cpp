#include <Wire.h>
#include <LSM6.h>

#include "IMUHandler.h"
#include "Config.h"

static LSM6 imu;

static float zOffset = 0.0f;

static bool imuReady = false;

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

bool imuBegin() {
  recoverI2CBus();

  Wire.begin();

  delay(100);

  for (int attempt = 0; attempt < 5; attempt++) {
    if (imu.init()) {
      imu.enableDefault();

      imuReady = true;

      return true;
    }

    delay(250);
  }

  imuReady = false;

  return false;
}

bool imuCalibrate(uint16_t samples) {
  if (!imuReady || samples == 0) {
    return false;
  }

  float sum = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    imu.read();

    float gyroZ =
        ((int16_t)imu.g.z) * 0.00875f;

    sum += gyroZ;

    delay(5);
  }

  zOffset = sum / samples;

  return true;
}

bool imuReadYawRate(float &degPerSec) {
  if (!imuReady) {
    return false;
  }

  imu.read();

  float rawRate =
      ((int16_t)imu.g.z) * 0.00875f;

  degPerSec = rawRate - zOffset;

  return true;
}

float imuGyroOffset() {
  return zOffset;
}