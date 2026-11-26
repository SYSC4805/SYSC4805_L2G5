#pragma once
#include <Arduino.h>


// ============================================================
// VERIFIED MOTOR / IMU WIRING
// ============================================================

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


// ============================================================
// REMAINING SYSTEM CONNECTIONS
// ============================================================

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

// Due analog sensors
//
// Line followers (front of the robot, powered from 3.3 V):
constexpr uint8_t LINE_RIGHT_PIN = A0;   // front-right line follower
constexpr uint8_t LINE_LEFT_PIN  = A1;   // front-left  line follower

// NOTE: the battery divider used to be on A1. A1 is now the left line
// follower, so the battery divider output must be moved to A2.
constexpr uint8_t BATTERY_PIN = A2;


// ============================================================
// MOTOR / NAVIGATION SETTINGS
// ============================================================

constexpr uint8_t MOTOR_PWM = 255;

// Straight driving used by the line/boundary test.
// !! Project limit is 30 cm/s. Measure the real speed at this PWM
// !! (drive a known distance, time it) and lower it if needed.
constexpr uint8_t DRIVE_PWM = 150;

// Existing timed-turn calibration (measured at PWM 255)
constexpr unsigned long LEFT_90_FALLBACK_MS  = 1550;
constexpr unsigned long RIGHT_90_FALLBACK_MS = 1300;
constexpr unsigned long TURN_180_FALLBACK_MS = 3000;


// ============================================================
// GYRO-CONTROLLED TURNS (TurnController)
// ============================================================

// 200 was chosen on hardware: at 255 the IMU I2C bus dropped out.
constexpr uint8_t TURN_PWM = 200;

// Soft start (limits motor inrush current / supply dip)
constexpr uint8_t TURN_RAMP_START_PWM = 120;
constexpr unsigned long TURN_RAMP_MS  = 250;

constexpr float TURN_GYRO_DEADBAND_DPS = 3.0f;

// Motors are cut this many degrees early because the robot coasts.
// (Requesting 90 deg -> stop at 88 deg gyro angle -> ~90 deg physical.)
constexpr float TURN_COAST_DEG = 2.0f;

// If the gyro turn fails (IMU unrecoverable / timeout) the robot finishes the
// remaining angle with a TIMED turn instead of stopping, so it keeps going.
// Less accurate than the gyro. Rotation speed at TURN_PWM, in deg/s, used to
// convert the remaining angle to a time (measured ~28-38 dps, falls with battery).
constexpr bool  TURN_FALLBACK_ENABLED = true;
constexpr float TURN_FALLBACK_DPS     = 30.0f;

// Safety backup only: ms of MOTOR-ON time before a turn is abandoned
constexpr unsigned long TURN_TIMEOUT_MS = 8000;

// IMU recoveries allowed inside one turn
constexpr uint8_t TURN_MAX_RECOVERIES = 4;

// "Motors commanded but the robot is not rotating" detection.
// If, TURN_NOMOTION_CHECK_MS after the motors start, the gyro has never
// seen more than TURN_NOMOTION_MIN_DPS of rotation, the turn is abandoned
// (dead battery, no motor power, wheels off the ground, robot stuck).
constexpr unsigned long TURN_NOMOTION_CHECK_MS = 800;
constexpr float TURN_NOMOTION_MIN_DPS = 10.0f;
constexpr uint8_t TURN_NOMOTION_MIN_SAMPLES = 20;

// true = print a line every 100 ms during turns (set false once turns work)
constexpr bool TURN_VERBOSE = true;


// ============================================================
// IMU
// ============================================================

constexpr float IMU_DPS_PER_LSB = 0.00875f;       // +/-245 dps range

// A gyro read slower than this is an I2C timeout
constexpr unsigned long IMU_SLOW_READ_US = 30000UL;

// |raw Z| below this = dead/stale sample
constexpr float IMU_RAW_INVALID_DPS = 0.20f;

// Calibration: samples further than this from the median are rejected
constexpr float IMU_CAL_OUTLIER_DPS = 2.0f;


// ============================================================
// BOUNDARY / STRIP SETTINGS
// ============================================================

// The wheels barely move the robot below about PWM 150-185 (the turn log shows
// almost no rotation at PWM 153), so reversing at DRIVE_PWM 150 can move the
// robot only a centimetre or two. Reverse and sideways shift use this PWM.
constexpr int BOUNDARY_REVERSE_PWM = 200;
constexpr int SHIFT_PWM            = 200;

// Minimum reverse time after a boundary is detected.
constexpr unsigned long BOUNDARY_REVERSE_MS = 300;

// The tape is wider than the distance reversed in BOUNDARY_REVERSE_MS, so the
// sensors can still be on the tape when the robot turns. Keep reversing until
// BOTH sensors read floor, but never longer than BOUNDARY_CLEAR_MAX_MS, then
// reverse BOUNDARY_CLEAR_EXTRA_MS more so the sensors end up clear of the tape
// edge with some margin.
constexpr unsigned long BOUNDARY_CLEAR_MAX_MS   = 1200;
constexpr unsigned long BOUNDARY_CLEAR_EXTRA_MS = 300;

// Line sensors are ignored for this long at the start of the sideways shift,
// so the tape the robot has just turned away from can not end the run.
constexpr unsigned long SHIFT_BLIND_MS = 400;

// false: when the boundary is hit while shifting sideways (arena edge), keep
//        going: back off, finish the U-turn and sweep back the other way.
// true : stop there (run complete).
constexpr bool STOP_AT_ARENA_EDGE = false;

// Time driven sideways between the two 90-degree turns of a U-turn.
// !! Calibrate so the robot shifts about one plow width.
constexpr unsigned long STRIP_SHIFT_MS = 1100;


// ============================================================
// PROJECT REQUIREMENTS
// ============================================================

constexpr unsigned long OBSTACLE_HOLD_MS = 2000;

constexpr unsigned long NANO_HEARTBEAT_TIMEOUT_MS = 2000;

constexpr unsigned long MAX_RUN_MS =
    5UL * 60UL * 1000UL;


// ============================================================
// BATTERY
// ============================================================

constexpr float LOW_BATTERY_V = 5.0f;

// Critical level used to transition from warning -> safe stop.
constexpr float CRITICAL_BATTERY_V = 4.7f;

constexpr float ADC_REF_V = 3.3f;
constexpr float ADC_MAX   = 4095.0f;

constexpr float BATTERY_R_TOP    = 10000.0f;
constexpr float BATTERY_R_BOTTOM = 10000.0f;


// ============================================================
// LINE SENSORS
// ============================================================

// Starting values only. Run the calibration in
// Unit_Tests/Line_Boundary_Turn (or read raw values over floor/tape)
// and put the real numbers here.
// Placed about 75% of the way from floor to tape, not at the midpoint: the
// floor is not uniform (a run logged the right sensor at 3656 on plain floor,
// above the old midpoint threshold of 3646).
constexpr int DEFAULT_LINE_THRESHOLD_LEFT  = 3760;   // measured: floor ~3164 (up to ~3500 seen), tape ~3964
constexpr int DEFAULT_LINE_THRESHOLD_RIGHT = 3820;   // measured: floor ~3298 (up to ~3660 seen), tape ~3994

// true  : tape reads HIGHER than floor  (boundary when raw >= threshold)
// false : tape reads LOWER  than floor  (boundary when raw <= threshold)
constexpr bool DEFAULT_LINE_BOUNDARY_IS_HIGH = true;

// Calibration is rejected if floor and tape readings differ by less
constexpr int LINE_MIN_SEPARATION = 300;

// ADC reads averaged per sample (noise) and consecutive boundary
// samples required before a sensor is reported as "on the line".
constexpr uint8_t LINE_ADC_AVG  = 4;
constexpr uint8_t LINE_DEBOUNCE = 2;


// ============================================================
// RGB
// ============================================================

constexpr unsigned long RGB_BLINK_MS = 300;