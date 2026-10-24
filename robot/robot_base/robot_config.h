// Firmware settings: network, PID gains, timing, parameter defaults.
// Pins and geometry are in robot_hardware.h.
#pragma once

// ==== WiFi ====
// Wi-Fi credentials and the agent IP are in network_local.h.
#if __has_include("network_local.h")
#include "network_local.h"
#else
#error "Copy network_local.example.h to network_local.h and fill in your network settings."
#endif

constexpr uint32_t MICROROS_AGENT_PORT = 8888;

// 0: micro-ROS over Wi-Fi/UDP. 1: over USB serial (network_local.h or the robot_base_serial env).
#ifndef MICROROS_TRANSPORT_SERIAL
#define MICROROS_TRANSPORT_SERIAL 0
#endif

#include "robot_hardware.h"

// ==== PID gains ====
// ArduPID scales I by the sample period (I=2.0 adds 0.1*error per 50 ms). D is off: tick counts are too noisy.
constexpr double PID_P = 0.4;
constexpr double PID_I = 2.0;
constexpr double PID_D = 0.0;
// Straight-line heading correction; trims each wheel target by at most 20%.
constexpr double STRAIGHT_CORRECTION_GAIN_PER_S = 2.0;
constexpr double STRAIGHT_MAX_TRIM_FRACTION = 0.20;
constexpr int PID_OUTPUT_LIMIT = 230;
// Low-speed assist defaults (live ROS parameters, see docs/ros_parameters.md).
constexpr bool DEFAULT_LOW_SPEED_ASSIST = true;
constexpr int DEFAULT_FF_STATIC_PWM = 25;
constexpr double DEFAULT_FF_PWM_PER_TICK = 0.2;
constexpr int DEFAULT_KICK_PWM = 65;
constexpr int DEFAULT_KICK_MS = 0;
constexpr double KICK_STALL_TICKS = 2.0;
constexpr unsigned long KICK_STALL_MS = 150;
constexpr int KICK_MAX_TRIES = 3;
// Per-wheel PID log over WebSerial every 100 ms.
#define DEBUG_PID_LOOP 1

// ==== Control loop timing ====
constexpr unsigned long CONTROL_PERIOD_MS = 50;
// Best-effort UDP sometimes delays cmd_vel past 500 ms, which stopped the robot mid-drive.
constexpr unsigned long CMD_VEL_TIMEOUT_MS = 1000;

// ==== Parameter server defaults (rclc_parameter_server) ====
constexpr int DEFAULT_LOG_LEVEL = 20;
constexpr int DEFAULT_ODOM_RATE_HZ = 20;
constexpr double DEFAULT_MAX_LINEAR_MPS = 0.2;
constexpr double DEFAULT_MAX_ANGULAR_RPS = 1.5;
