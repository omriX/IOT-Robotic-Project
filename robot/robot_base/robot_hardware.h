// Pins, battery sensing and measured geometry. No network settings, so the
// hardware tests (unit_tests/hardware) can include it.
#pragma once

// ==== Motor driver pins (L293D) ====
constexpr int MOTOR_A_DIRECTION_PIN = 27;
constexpr int MOTOR_A_PWM_PIN = 14;
constexpr int MOTOR_B_DIRECTION_PIN = 13;
constexpr int MOTOR_B_PWM_PIN = 12;

// ==== Encoder pins ====
constexpr int MOT_A_ENCODER_A = 23;
constexpr int MOT_A_ENCODER_B = 22;
constexpr int MOT_B_ENCODER_A = 19;
constexpr int MOT_B_ENCODER_B = 21;

// ==== Battery monitoring (2S pack through a voltage divider) ====
constexpr int BATTERY_PIN = 34;
constexpr int LOW_BATTERY_LED_PIN = 2;
constexpr float BATTERY_VOLTAGE_DIVIDER_FACTOR = 2.8;
constexpr float LOW_BATTERY_THRESHOLD_V = 6.6;
constexpr float ADC_MAX_VALUE = 4095.0;
constexpr float ADC_LOGIC_LEVEL_V = 3.3;

// ==== Calibration (see docs/hardware_calibration.md, docs/live_calibration.md) ====
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_BASE_M = 0.119528;
constexpr double WHEEL_RADIUS_M = 0.021933;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr double LEFT_DIR_SIGN = -1.0;
constexpr double RIGHT_DIR_SIGN = -1.0;
constexpr double MAX_TICKS_PER_INTERVAL = 350.0;
// PWM below this is sent as 0. Kept below the ~40 PWM where motion starts, so
// the PI controller can push the wheel through static friction.
constexpr int SPEED_DEADBAND = 10;
