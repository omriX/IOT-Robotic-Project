// robot_hardware.h
//
// Physical description of the robot: pins, battery sensing, measured geometry.
// No network settings, so the standalone hardware tests (unit_tests/hardware)
// can include it without a network_local.h.
#pragma once

// ==== Motor driver pins -- unchanged from POC code ====
constexpr int MOTOR_A_DIRECTION_PIN = 27;
constexpr int MOTOR_A_PWM_PIN = 14;
constexpr int MOTOR_B_DIRECTION_PIN = 13;
constexpr int MOTOR_B_PWM_PIN = 12;

// ==== Encoder pins ====
constexpr int MOT_A_ENCODER_A = 23;
constexpr int MOT_A_ENCODER_B = 22;
constexpr int MOT_B_ENCODER_A = 19;
constexpr int MOT_B_ENCODER_B = 21;

// ==== Battery monitoring -- unchanged from POC code ====
constexpr int BATTERY_PIN = 34;
constexpr int LOW_BATTERY_LED_PIN = 2;
constexpr float BATTERY_VOLTAGE_DIVIDER_FACTOR = 2.8;
constexpr float LOW_BATTERY_THRESHOLD_V = 6.6;
constexpr float ADC_MAX_VALUE = 4095.0;
constexpr float ADC_LOGIC_LEVEL_V = 3.3;

// ==== Calibration (see docs/calibration_log_1.md) ====
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_BASE_M = 0.116;
constexpr double WHEEL_RADIUS_M = 0.021961;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr double LEFT_DIR_SIGN = -1.0;
constexpr double RIGHT_DIR_SIGN = -1.0;
constexpr double MAX_TICKS_PER_INTERVAL = 350.0;
// PWM below this is sent as 0. Motion starts around PWM 40 (calibration ramp
// test), but the deadband is kept low so the PI controller can work in the
// 10-40 range and push the wheel through static friction. Was 40 before the
// 5 Oct 2026 PID retune; 10 was validated on the floor on 17 Oct 2026.
constexpr int SPEED_DEADBAND = 10;
