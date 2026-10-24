// robot_config.h
//
// Firmware settings for robot_base: network, PID gains, timing, parameter defaults.
// Pins and measured geometry live in robot_hardware.h.
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

// 0: micro-ROS over Wi-Fi/UDP (agent-udp). 1: over the USB cable (agent-serial).
// Set it in network_local.h or with the robot_base_serial PlatformIO env.
// In serial mode micro-ROS owns the USB UART, so logs go to WebSerial only.
#ifndef MICROROS_TRANSPORT_SERIAL
#define MICROROS_TRANSPORT_SERIAL 0
#endif

#include "robot_hardware.h"

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
// Friction feedforward added to the PID output: ff_static_pwm in the setpoint's
// direction plus ff_pwm_per_tick per target tick/interval. Without it the
// integral needs ~1s to climb to the PWM where the wheel starts turning (~40
// wheels-up), and the wheel hums in place. Wheels-up: ~35 PWM holds 76 ticks,
// ~80 holds 143. Both are live ROS parameters; set them to 0 to disable.
// TODO: re-measure the static term on the floor (fine down-ramp stall PWM).
// low_speed_assist (live ROS parameter) false turns off the feedforward, the
// kick and the direction limit together, back to the plain PI controller.
constexpr bool DEFAULT_LOW_SPEED_ASSIST = true;
constexpr int DEFAULT_FF_STATIC_PWM = 25;
constexpr double DEFAULT_FF_PWM_PER_TICK = 0.2;
// Startup kick: a short PWM burst from standstill, ended as soon as the
// encoder sees movement. 150 PWM for 100ms made the wheels lurch and slip,
// which put the odom heading off on in-place turns. kick_pwm / kick_ms are
// live ROS parameters; kick_ms 0 disables it.
constexpr int DEFAULT_KICK_PWM = 65;
constexpr int DEFAULT_KICK_MS = 0;
constexpr double KICK_STALL_TICKS = 2.0;
constexpr unsigned long KICK_STALL_MS = 150;
constexpr int KICK_MAX_TRIES = 3;
// Logs per-wheel setpoint / measured ticks / PWM every 100ms over WebSerial.
#define DEBUG_PID_LOOP 1

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
