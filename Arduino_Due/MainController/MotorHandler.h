#pragma once
#include <Arduino.h>

void motorBegin();
void stopAllMotors();
void setAllMotorPWM(uint8_t pwm);
void driveForward(uint8_t pwm);
void driveReverse(uint8_t pwm);
void pivotLeft(uint8_t pwm);
void pivotRight(uint8_t pwm);
