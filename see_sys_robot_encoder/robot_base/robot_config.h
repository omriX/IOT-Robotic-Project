// robot_config.h
//
// All constants for robot_base live here: pins, calibration/geometry, PID gains defaults.
#pragma once

// ==== WiFi ====
// Used for WebSerial (the debug channel once micro-ROS owns the Serial/USB UART)
// and the micro-ROS transport itself.
// TODO: move to NVS + serial config console instead of hardcoding credentials here.
#if __has_include("network_local.h")
#include "network_local.h"
#else
#error "Copy network_local.example.h to network_local.h and fill in your network settings."
#endif

// micro-ROS agent (docker/docker-compose.yml, agent-udp) -- the PC's LAN IP.
// TODO: re-check this IP if it changes (DHCP).
constexpr uint32_t MICROROS_AGENT_PORT = 8888;

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

// ==== PID gains ====
// ArduPID scales I by the sample period in seconds, so I=2.0 adds 0.1*error per 50ms cycle.
// D is off: the per-interval tick count is too noisy to differentiate.
constexpr double PID_P = 0.4;
constexpr double PID_I = 2.0;
constexpr double PID_D = 0.0;
// Outer encoder-heading loop, active only for straight motion. Its trim
// changes each wheel target by at most 20%, also respecting the speed limit.
constexpr double STRAIGHT_CORRECTION_GAIN_PER_S = 2.0;
constexpr double STRAIGHT_MAX_TRIM_FRACTION = 0.20;
constexpr int PID_OUTPUT_LIMIT = 230;
// Feedforward PWM per target tick/interval, so the PID only trims the rest.
// Measured wheels-up: ~35 PWM holds 76 ticks, ~80 holds 143.
constexpr double PID_FEEDFORWARD_PER_TICK = 0.4;
// Startup kick: a PWM burst to break static friction at low speeds, ended as
// soon as the encoder sees movement. 255 for 100ms spun the wheels to ~300
// ticks/interval and made them bounce. kick_pwm / kick_ms are live ROS
// parameters; kick_ms 0 disables it.
constexpr int DEFAULT_KICK_PWM = 150;
constexpr int DEFAULT_KICK_MS = 100;
constexpr double KICK_STALL_TICKS = 2.0;
constexpr unsigned long KICK_STALL_MS = 150;
constexpr int KICK_MAX_TRIES = 3;
// Logs per-wheel setpoint / measured ticks / PWM every 100ms over WebSerial.
#define DEBUG_PID_LOOP 1

// ==== Calibration ====
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_BASE_M = 0.119528;
constexpr double WHEEL_RADIUS_M = 0.021933;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr double LEFT_DIR_SIGN = -1.0;
constexpr double RIGHT_DIR_SIGN = -1.0;
constexpr double MAX_TICKS_PER_INTERVAL = 350.0;
constexpr int SPEED_DEADBAND = 10;

// ==== Control loop timing ====
constexpr unsigned long CONTROL_PERIOD_MS = 50;
// cmd_vel is best-effort UDP, and that path occasionally
// delays a packet past 500ms, which tripped the watchdog mid-drive and
// looked like the motors twitching.
constexpr unsigned long CMD_VEL_TIMEOUT_MS = 1000;

// ==== Parameter server defaults (rclc_parameter_server) ====
constexpr int DEFAULT_LOG_LEVEL = 20;
constexpr int DEFAULT_ODOM_RATE_HZ = 20;
constexpr double DEFAULT_MAX_LINEAR_MPS = 0.2;
constexpr double DEFAULT_MAX_ANGULAR_RPS = 1.5;
