#pragma once
#include <Arduino.h>

bool imuBegin();

bool imuCalibrate(uint16_t samples = 500);

bool imuReadYawRate(float &degPerSec);

float imuGyroOffset();

void recoverI2CBus();