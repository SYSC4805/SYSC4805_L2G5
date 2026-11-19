#include "MotorHandler.h"
#include "Config.h"

void motorBegin() {
  pinMode(M1_DIR, OUTPUT); pinMode(M1_EN, OUTPUT);
  pinMode(M2_DIR, OUTPUT); pinMode(M2_EN, OUTPUT);
  pinMode(M3_DIR, OUTPUT); pinMode(M3_EN, OUTPUT);
  pinMode(M4_DIR, OUTPUT); pinMode(M4_EN, OUTPUT);
  stopAllMotors();
}

void setAllMotorPWM(uint8_t pwm) {
  analogWrite(M1_EN, pwm);
  analogWrite(M2_EN, pwm);
  analogWrite(M3_EN, pwm);
  analogWrite(M4_EN, pwm);
}

void stopAllMotors() {
  setAllMotorPWM(0);
}

void driveForward(uint8_t pwm) {
  // Direction convention should match the team's verified forward pattern.
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);
  setAllMotorPWM(pwm);
}

void driveReverse(uint8_t pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  setAllMotorPWM(pwm);
}

void pivotLeft(uint8_t pwm) {
  // Verified turn pattern from the uploaded motor/IMU baseline.
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  setAllMotorPWM(pwm);
}

void pivotRight(uint8_t pwm) {
  // Verified turn pattern from the uploaded motor/IMU baseline.
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);
  setAllMotorPWM(pwm);
}
