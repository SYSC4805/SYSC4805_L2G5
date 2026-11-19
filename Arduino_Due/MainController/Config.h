#pragma once
#include <Arduino.h>

 
// VERIFIED PROJECT WIRING
 
// Arduino Due motor/Cytron mapping already used by the team.
constexpr uint8_t M1_DIR = 3;
constexpr uint8_t M1_EN  = 4;
constexpr uint8_t M2_DIR = 5;
constexpr uint8_t M2_EN  = 6;
constexpr uint8_t M3_DIR = 7;
constexpr uint8_t M3_EN  = 8;
constexpr uint8_t M4_DIR = 9;
constexpr uint8_t M4_EN  = 10;

// Due I2C pins used by the MinIMU-9 v5.
constexpr uint8_t DUE_SDA_PIN = 20;
constexpr uint8_t DUE_SCL_PIN = 21;

 
// PROPOSED FREE-PIN MAP FOR REMAINING PROGRESS-REPORT HANDLERS
 
// These pins do not conflict with the verified motor/I2C map.
// Change ONLY this file if your physical wiring uses different pins.
constexpr uint8_t RGB_R_PIN = 22;
constexpr uint8_t RGB_G_PIN = 23;
constexpr uint8_t RGB_B_PIN = 24;

constexpr uint8_t ESTOP_PIN = 25;

// Nano -> Due event lines (through appropriate 5 V -> 3.3 V level shifting).
constexpr uint8_t NANO_IR1_PIN       = 26;
constexpr uint8_t NANO_IR2_PIN       = 27;
constexpr uint8_t NANO_ULTRASONIC_PIN= 28;
constexpr uint8_t NANO_RANGE_PIN     = 29;
constexpr uint8_t NANO_HEARTBEAT_PIN = 30;

constexpr uint8_t LINE_SENSOR_PIN = A0;
constexpr uint8_t BATTERY_PIN     = A1;

 
// PROJECT LIMITS / THRESHOLDS
 
constexpr uint8_t MOTOR_PWM = 255;

// Timed-turn baseline retained while IMU closed-loop turn accuracy is refined.
constexpr unsigned long LEFT_90_FALLBACK_MS  = 1550;
constexpr unsigned long RIGHT_90_FALLBACK_MS = 1450;
constexpr unsigned long TURN_180_FALLBACK_MS = 3000;

// Safety requirements.
constexpr unsigned long OBSTACLE_HOLD_MS = 2000;
constexpr unsigned long NANO_HEARTBEAT_TIMEOUT_MS = 2000;
constexpr unsigned long MAX_RUN_MS = 5UL * 60UL * 1000UL;

// Battery requirement.
constexpr float LOW_BATTERY_V = 5.0f;

// Due ADC: 12-bit when analogReadResolution(12) is called.
constexpr float ADC_REF_V = 3.3f;
constexpr float ADC_MAX = 4095.0f;

// Proposed 10k/10k divider for a nominal 6 V pack.

constexpr float BATTERY_R_TOP = 10000.0f;
constexpr float BATTERY_R_BOTTOM = 10000.0f;


// The handler supports runtime calibration; this is only the fallback.
constexpr int DEFAULT_LINE_THRESHOLD = 2048;
