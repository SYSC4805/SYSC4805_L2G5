#pragma once
#include <Arduino.h>

  
// VERIFIED MOTOR / IMU WIRING
  

constexpr uint8_t M1_DIR = 3;
constexpr uint8_t M1_EN  = 4;

constexpr uint8_t M2_DIR = 5;
constexpr uint8_t M2_EN  = 6;

constexpr uint8_t M3_DIR = 7;
constexpr uint8_t M3_EN  = 8;

constexpr uint8_t M4_DIR = 9;
constexpr uint8_t M4_EN  = 10;

// Arduino Due I2C pins
constexpr uint8_t DUE_SDA_PIN = 20;
constexpr uint8_t DUE_SCL_PIN = 21;

  
// REMAINING SYSTEM CONNECTIONS
  

constexpr uint8_t RGB_R_PIN = 22;
constexpr uint8_t RGB_G_PIN = 23;
constexpr uint8_t RGB_B_PIN = 24;

constexpr uint8_t ESTOP_PIN = 25;

// Nano Every -> Due status/event signals
constexpr uint8_t NANO_IR_LEFT_PIN   = 26;
constexpr uint8_t NANO_IR_RIGHT_PIN  = 27;
constexpr uint8_t NANO_FRONT_PIN     = 28;
constexpr uint8_t NANO_REAR_PIN      = 29;
constexpr uint8_t NANO_HEARTBEAT_PIN = 30;

// Due sensors
constexpr uint8_t LINE_SENSOR_PIN = A0;
constexpr uint8_t BATTERY_PIN     = A1;

  
// MOTOR / NAVIGATION SETTINGS
  

constexpr uint8_t MOTOR_PWM = 255;

// Existing timed-turn calibration
constexpr unsigned long LEFT_90_FALLBACK_MS  = 1550;
constexpr unsigned long RIGHT_90_FALLBACK_MS = 1300;
constexpr unsigned long TURN_180_FALLBACK_MS = 3000;

  
// PROJECT REQUIREMENTS
  

constexpr unsigned long OBSTACLE_HOLD_MS = 2000;

constexpr unsigned long NANO_HEARTBEAT_TIMEOUT_MS = 2000;

constexpr unsigned long MAX_RUN_MS =
    5UL * 60UL * 1000UL;

  
// BATTERY
  

constexpr float LOW_BATTERY_V = 5.0f;

// Critical level used to transition from warning -> safe stop.
constexpr float CRITICAL_BATTERY_V = 4.7f;

constexpr float ADC_REF_V = 3.3f;
constexpr float ADC_MAX   = 4095.0f;

constexpr float BATTERY_R_TOP    = 10000.0f;
constexpr float BATTERY_R_BOTTOM = 10000.0f;

  
// LINE SENSOR
  

constexpr int DEFAULT_LINE_THRESHOLD = 2048;

  
// RGB
  

constexpr unsigned long RGB_BLINK_MS = 300;