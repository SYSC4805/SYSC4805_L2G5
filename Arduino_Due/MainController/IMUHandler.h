#pragma once
#include <Arduino.h>

// Gyro sample validity flags (0 = good sample)
constexpr uint8_t IMU_FLAG_SLOW     = 0x01;   // I2C read timed out / was slow
constexpr uint8_t IMU_FLAG_FROZEN   = 0x02;   // x,y,z identical to previous sample
constexpr uint8_t IMU_FLAG_NEARZERO = 0x04;   // |raw Z| ~ 0 (dead / stale read)

bool imuBegin();

// Robust calibration: median + outlier rejection. Robot must be still.
// Leaves the stored offset untouched if it fails.
bool imuCalibrate(uint16_t samples = 500);

// Reads gyro Z. rawDps = sensor value, correctedDps = raw - bias.
// Returns validity flags; only trust the values when it returns 0.
uint8_t imuReadGyro(float &rawDps, float &correctedDps);

// Legacy API: returns false when the sample is invalid.
bool imuReadYawRate(float &degPerSec);

// Call with the motors STOPPED. Resets the I2C bus, restarts Wire,
// re-initialises the LSM6, checks it is updating and recalibrates.
bool imuRecover();

float imuGyroOffset();

const char* imuFlagText(uint8_t flags);

void recoverI2CBus();