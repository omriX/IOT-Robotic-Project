// robot_config.h
//
// All constants for robot_base live here: pins, calibration/geometry, PID gains defaults.
#pragma once

// ==== WiFi ====
// Used for WebSerial (the debug channel once micro-ROS owns the Serial/USB UART)
// and the micro-ROS transport itself.
// TODO: move to NVS + serial config console instead of hardcoding credentials here.
constexpr char WIFI_SSID[] = "Apt1920";
constexpr char WIFI_PASSWORD[] = "DiraSisma1920";

// micro-ROS agent (docker/docker-compose.yml, agent-udp) -- the PC's LAN IP.
// TODO: re-check this IP if it changes (DHCP).
constexpr char MICROROS_AGENT_IP[] = "192.168.1.221";
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

// ==== PID gains -- unchanged from POC code ====
constexpr double PID_P = 0.4;
constexpr double PID_I = 0.001;
constexpr double PID_D = 0.05;
constexpr int PID_OUTPUT_LIMIT = 120;

// ==== Calibration ====
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_BASE_M = 0.116;
constexpr double WHEEL_RADIUS_M = 0.0220;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr double LEFT_DIR_SIGN = -1.0;
constexpr double RIGHT_DIR_SIGN = -1.0;
constexpr double MAX_TICKS_PER_INTERVAL = 350.0;
constexpr int SPEED_DEADBAND = 40;

// ==== Control loop timing ====
constexpr unsigned long CONTROL_PERIOD_MS = 50;
constexpr unsigned long CMD_VEL_TIMEOUT_MS = 500;
