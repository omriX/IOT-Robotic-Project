// robot_base.ino
//
// subscribes to /cmd_vel, runs it through the same
// diff_drive.h kinematics already verified on the PC (test/test_diff_drive.cpp),
// and logs the resulting per-wheel m/s and ticks/interval over WebSerial.
//
// Verify with the agent reachable on the same LAN as the ESP32:
//   docker compose -f docker/docker-compose.yml up agent-udp
//   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
// and watch WebSerial for equal left/right values.

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int8.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rclc_parameter/rclc_parameter.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ESP32Encoder.h>
#include <ArduPID.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "robot_config.h"
#include "diff_drive.h"
#include "status_led.h"
#include "selftest.h"
#include "ros_log.h"

constexpr double CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0;

#define RCCHECK(fn)                                                                            \
  {                                                                                            \
    rcl_ret_t temp_rc = fn;                                                                    \
    if (temp_rc != RCL_RET_OK)                                                                 \
    {                                                                                          \
      dprintln("RCCHECK failed at line " + String(__LINE__) + ": rc=" + String((int)temp_rc)); \
      return false;                                                                            \
    }                                                                                          \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

AsyncWebServer server(80);

// Log Serial and WebSerial
void dprintln(const String &line)
{
  Serial.println(line);
  WebSerial.println(line);
}

ESP32Encoder encoderA;
ESP32Encoder encoderB;

double setpointA = 0, inputA = 0, outputA = 0;
double setpointB = 0, inputB = 0, outputB = 0;
ArduPID controllerA;
ArduPID controllerB;

// cmd_vel writes here (critical section); controlTask reads it each cycle.
portMUX_TYPE sharedStateMux = portMUX_INITIALIZER_UNLOCKED;
struct SharedState
{
  double target_left_ticks = 0;
  double target_right_ticks = 0;
  unsigned long last_cmd_ms = 0;
  bool faulted = false;
  // written by controlTask, read by loop() for logging (and later /odom)
  double x = 0, y = 0, theta = 0;
  double linear_v = 0, angular_w = 0;
  // Live-tunable via rclc_parameter_server
  double wheel_radius_m = WHEEL_RADIUS_M;
  double wheel_base_m = WHEEL_BASE_M;
  double max_linear_mps = DEFAULT_MAX_LINEAR_MPS;
  double max_angular_rps = DEFAULT_MAX_ANGULAR_RPS;
  unsigned long cmd_vel_timeout_ms = CMD_VEL_TIMEOUT_MS;
};
SharedState shared;

// WebSerial "freewheel on"/"freewheel off"
// cuts motor output so the wheels can be turned by hand to test odometry.
volatile bool freewheelMode = false;

void motors(int speedA, int speedB)
{
  if (abs(speedA) < SPEED_DEADBAND)
    speedA = 0;
  if (abs(speedB) < SPEED_DEADBAND)
    speedB = 0;

  if (speedA >= 0)
  {
    analogWrite(MOTOR_A_PWM_PIN, speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  }
  else
  {
    analogWrite(MOTOR_A_PWM_PIN, 256 + speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, HIGH);
  }

  if (speedB >= 0)
  {
    analogWrite(MOTOR_B_PWM_PIN, speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);
  }
  else
  {
    analogWrite(MOTOR_B_PWM_PIN, 256 + speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, HIGH);
  }
}

void controlTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

  long prevCountA = 0;
  long prevCountB = 0;
  Pose2D pose;

  for (;;)
  {
    portENTER_CRITICAL(&sharedStateMux);
    double targetLeft = shared.target_left_ticks;
    double targetRight = shared.target_right_ticks;
    unsigned long lastCmdMs = shared.last_cmd_ms;
    bool faulted = shared.faulted;
    unsigned long cmdVelTimeoutMs = shared.cmd_vel_timeout_ms;
    double wheelBaseM = shared.wheel_base_m;
    double ticksPerM = ticksPerMeter(COUNTS_PER_WHEEL_REV, shared.wheel_radius_m);
    portEXIT_CRITICAL(&sharedStateMux);

    if (faulted || millis() - lastCmdMs > cmdVelTimeoutMs)
    {
      targetLeft = 0;
      targetRight = 0;
    }

    if (MOTOR_A_IS_LEFT)
    {
      setpointA = targetLeft;
      setpointB = targetRight;
    }
    else
    {
      setpointA = targetRight;
      setpointB = targetLeft;
    }

    long currCountA = (long)encoderA.getCount();
    inputA = (double)(currCountA - prevCountA);
    prevCountA = currCountA;
    controllerA.compute();

    long currCountB = (long)encoderB.getCount();
    inputB = (double)(currCountB - prevCountB);
    prevCountB = currCountB;
    controllerB.compute();

    if (freewheelMode)
      motors(0, 0);
    else
      motors((int)outputA, (int)outputB);

    double deltaLeftTicks = (MOTOR_A_IS_LEFT ? inputA : inputB) * LEFT_DIR_SIGN;
    double deltaRightTicks = (MOTOR_A_IS_LEFT ? inputB : inputA) * RIGHT_DIR_SIGN;
    OdometryDelta odom = integrateOdometry(pose, deltaLeftTicks, deltaRightTicks, ticksPerM, wheelBaseM, CONTROL_PERIOD_S);

    portENTER_CRITICAL(&sharedStateMux);
    shared.x = pose.x;
    shared.y = pose.y;
    shared.theta = pose.theta;
    shared.linear_v = odom.linear_v;
    shared.angular_w = odom.angular_w;
    portEXIT_CRITICAL(&sharedStateMux);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t status_publisher;
std_msgs__msg__UInt8 status_msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
bool timeSynced = false;
RosLogger logger;

void initOdomMsg()
{
  nav_msgs__msg__Odometry__init(&odom_msg);
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

  for (int i = 0; i < 36; i++)
  {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }
  const double diag[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.05};
  for (int i = 0; i < 6; i++)
  {
    odom_msg.pose.covariance[i * 6 + i] = diag[i];
    odom_msg.twist.covariance[i * 6 + i] = diag[i];
  }
}

void publish_odom()
{
  portENTER_CRITICAL(&sharedStateMux);
  double x = shared.x, y = shared.y, theta = shared.theta;
  double v = shared.linear_v, w = shared.angular_w;
  portEXIT_CRITICAL(&sharedStateMux);

  int64_t stamp_ns = timeSynced ? rmw_uros_epoch_nanos() : (int64_t)millis() * 1000000LL;
  odom_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  Quaternion2D q = yawToQuaternion(theta);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = q.z;
  odom_msg.pose.pose.orientation.w = q.w;

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = w;

  rcl_ret_t rc = rcl_publish(&odom_publisher, &odom_msg, NULL);
  if (rc != RCL_RET_OK)
  {
    static unsigned long last_warn_ms = 0;
    if (millis() - last_warn_ms > 2000)
    {
      dprintln("odom publish failed, rc=" + String((int)rc));
      last_warn_ms = millis();
    }
  }
}

rcl_publisher_t battery_publisher;
sensor_msgs__msg__BatteryState battery_msg;
float lastBatteryVoltage = 0;

void initBatteryMsg()
{
  sensor_msgs__msg__BatteryState__init(&battery_msg);
  battery_msg.temperature = NAN;
  battery_msg.current = NAN;
  battery_msg.charge = NAN;
  battery_msg.capacity = NAN;
  battery_msg.design_capacity = NAN;
  battery_msg.percentage = NAN;
  battery_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

float measure_battery()
{
  float adc_voltage = (analogRead(BATTERY_PIN) / ADC_MAX_VALUE) * ADC_LOGIC_LEVEL_V;
  return adc_voltage * BATTERY_VOLTAGE_DIVIDER_FACTOR;
}

void publish_battery()
{
  int64_t stamp_ns = timeSynced ? rmw_uros_epoch_nanos() : (int64_t)millis() * 1000000LL;
  battery_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
  battery_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

  battery_msg.voltage = lastBatteryVoltage;
  battery_msg.present = true;

  rcl_publish(&battery_publisher, &battery_msg, NULL);
}

Preferences prefs;
rclc_parameter_server_t param_server;
unsigned long odomPublishPeriodMs = CONTROL_PERIOD_MS;

void loadParamsFromNvs()
{
  double wheelRadiusM = prefs.getDouble("wheel_radius_m", WHEEL_RADIUS_M);
  double wheelBaseM = prefs.getDouble("wheel_base_m", WHEEL_BASE_M);
  double maxLinearMps = prefs.getDouble("max_linear_mps", DEFAULT_MAX_LINEAR_MPS);
  double maxAngularRps = prefs.getDouble("max_angular_rps", DEFAULT_MAX_ANGULAR_RPS);
  unsigned long cmdVelTimeoutMs = (unsigned long)prefs.getInt("cmd_to_ms", (int)CMD_VEL_TIMEOUT_MS);

  portENTER_CRITICAL(&sharedStateMux);
  shared.wheel_radius_m = wheelRadiusM;
  shared.wheel_base_m = wheelBaseM;
  shared.max_linear_mps = maxLinearMps;
  shared.max_angular_rps = maxAngularRps;
  shared.cmd_vel_timeout_ms = cmdVelTimeoutMs;
  portEXIT_CRITICAL(&sharedStateMux);

  logger.min_level = (uint8_t)prefs.getInt("log_level", DEFAULT_LOG_LEVEL);
  int hz = prefs.getInt("odom_rate_hz", DEFAULT_ODOM_RATE_HZ);
  odomPublishPeriodMs = hz > 0 ? 1000 / hz : CONTROL_PERIOD_MS;
}

bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context)
{
  (void)context;
  (void)old_param;
  if (new_param == NULL)
    return true;

  const char *name = new_param->name.data;
  if (strcmp(name, "log_level") == 0)
  {
    logger.min_level = (uint8_t)new_param->value.integer_value;
    prefs.putInt("log_level", (int)new_param->value.integer_value);
  }
  else if (strcmp(name, "cmd_vel_timeout_ms") == 0)
  {
    portENTER_CRITICAL(&sharedStateMux);
    shared.cmd_vel_timeout_ms = (unsigned long)new_param->value.integer_value;
    portEXIT_CRITICAL(&sharedStateMux);
    prefs.putInt("cmd_to_ms", (int)new_param->value.integer_value);
  }
  else if (strcmp(name, "odom_rate_hz") == 0)
  {
    long hz = new_param->value.integer_value;
    if (hz <= 0)
      return false;
    odomPublishPeriodMs = 1000 / hz;
    prefs.putInt("odom_rate_hz", (int)hz);
  }
  else if (strcmp(name, "wheel_radius_m") == 0)
  {
    portENTER_CRITICAL(&sharedStateMux);
    shared.wheel_radius_m = new_param->value.double_value;
    portEXIT_CRITICAL(&sharedStateMux);
    prefs.putDouble("wheel_radius_m", new_param->value.double_value);
  }
  else if (strcmp(name, "wheel_base_m") == 0)
  {
    portENTER_CRITICAL(&sharedStateMux);
    shared.wheel_base_m = new_param->value.double_value;
    portEXIT_CRITICAL(&sharedStateMux);
    prefs.putDouble("wheel_base_m", new_param->value.double_value);
  }
  else if (strcmp(name, "max_linear_mps") == 0)
  {
    portENTER_CRITICAL(&sharedStateMux);
    shared.max_linear_mps = new_param->value.double_value;
    portEXIT_CRITICAL(&sharedStateMux);
    prefs.putDouble("max_linear_mps", new_param->value.double_value);
  }
  else if (strcmp(name, "max_angular_rps") == 0)
  {
    portENTER_CRITICAL(&sharedStateMux);
    shared.max_angular_rps = new_param->value.double_value;
    portEXIT_CRITICAL(&sharedStateMux);
    prefs.putDouble("max_angular_rps", new_param->value.double_value);
  }
  else
  {
    return true;
  }

  ROS_LOGF(logger, rcl_interfaces__msg__Log__INFO, "param changed: %s", name);
  return true;
}

bool selftestPassed = false;
bool selftestLogged = false;

enum AgentState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState state = WAITING_AGENT;

StatusLedState statusLed;

void cmd_vel_callback(const void *msgin)
{
  portENTER_CRITICAL(&sharedStateMux);
  bool faulted = shared.faulted;
  double wheelBaseM = shared.wheel_base_m;
  double ticksPerM = ticksPerMeter(COUNTS_PER_WHEEL_REV, shared.wheel_radius_m);
  double maxLinear = shared.max_linear_mps;
  double maxAngular = shared.max_angular_rps;
  portEXIT_CRITICAL(&sharedStateMux);
  if (faulted)
    return;

  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double linear_v = constrain(msg->linear.x, -maxLinear, maxLinear);
  double angular_w = constrain(msg->angular.z, -maxAngular, maxAngular);

  WheelVelocities wv = wheelLinearVelocities(linear_v, angular_w, wheelBaseM);
  WheelSetpoints sp = cmdVelToWheelSetpoints(
      linear_v, angular_w,
      wheelBaseM, ticksPerM, CONTROL_PERIOD_S,
      LEFT_DIR_SIGN, RIGHT_DIR_SIGN, MAX_TICKS_PER_INTERVAL);

  portENTER_CRITICAL(&sharedStateMux);
  shared.target_left_ticks = sp.left_ticks_per_interval;
  shared.target_right_ticks = sp.right_ticks_per_interval;
  shared.last_cmd_ms = millis();
  portEXIT_CRITICAL(&sharedStateMux);

  dprintln("cmd_vel v:" + String(linear_v) + " w:" + String(angular_w) +
           " | wheel_mps L:" + String(wv.left_mps) + " R:" + String(wv.right_mps) +
           " | ticks_per_interval L:" + String(sp.left_ticks_per_interval) +
           " R:" + String(sp.right_ticks_per_interval));
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  RCCHECK(rclc_subscription_init_best_effort(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
      &status_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "robot_base/status"));

  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom"));

  RCCHECK(rclc_publisher_init_default(
      &battery_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"));

  if (!rosLoggerInit(logger, &node))
    return false;

  timeSynced = (RMW_RET_OK == rmw_uros_sync_session(1000));
  if (!timeSynced)
    rosLog(logger, rcl_interfaces__msg__Log__WARN, "time sync failed, /odom timestamps fall back to millis()");

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  rclc_parameter_options_t param_options = {false, 10, false, false};
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, &on_parameter_changed));

  portENTER_CRITICAL(&sharedStateMux);
  double wheelRadiusM = shared.wheel_radius_m, wheelBaseM = shared.wheel_base_m;
  double maxLinearMps = shared.max_linear_mps, maxAngularRps = shared.max_angular_rps;
  int64_t cmdVelTimeoutMs = (int64_t)shared.cmd_vel_timeout_ms;
  portEXIT_CRITICAL(&sharedStateMux);

  RCCHECK(rclc_add_parameter(&param_server, "log_level", RCLC_PARAMETER_INT));
  RCCHECK(rclc_add_parameter(&param_server, "cmd_vel_timeout_ms", RCLC_PARAMETER_INT));
  RCCHECK(rclc_add_parameter(&param_server, "odom_rate_hz", RCLC_PARAMETER_INT));
  RCCHECK(rclc_add_parameter(&param_server, "wheel_radius_m", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "wheel_base_m", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "max_linear_mps", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "max_angular_rps", RCLC_PARAMETER_DOUBLE));

  RCCHECK(rclc_parameter_set_int(&param_server, "log_level", logger.min_level));
  RCCHECK(rclc_parameter_set_int(&param_server, "cmd_vel_timeout_ms", cmdVelTimeoutMs));
  RCCHECK(rclc_parameter_set_int(&param_server, "odom_rate_hz", 1000 / odomPublishPeriodMs));
  RCCHECK(rclc_parameter_set_double(&param_server, "wheel_radius_m", wheelRadiusM));
  RCCHECK(rclc_parameter_set_double(&param_server, "wheel_base_m", wheelBaseM));
  RCCHECK(rclc_parameter_set_double(&param_server, "max_linear_mps", maxLinearMps));
  RCCHECK(rclc_parameter_set_double(&param_server, "max_angular_rps", maxAngularRps));

  rosLog(logger, rcl_interfaces__msg__Log__INFO, "robot_base connected");
  if (!selftestLogged)
  {
    rosLog(logger, selftestPassed ? rcl_interfaces__msg__Log__INFO : rcl_interfaces__msg__Log__FATAL,
           selftestPassed ? "self-test passed" : "self-test FAILED");
    selftestLogged = true;
  }

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc;
  rc = rcl_subscription_fini(&cmd_vel_subscriber, &node);
  (void)rc;
  rc = rcl_publisher_fini(&status_publisher, &node);
  (void)rc;
  rc = rcl_publisher_fini(&odom_publisher, &node);
  (void)rc;
  rc = rcl_publisher_fini(&battery_publisher, &node);
  (void)rc;
  rc = rclc_parameter_server_fini(&param_server, &node);
  (void)rc;
  rosLoggerFini(logger);
  rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  (void)rc;
  rclc_support_fini(&support);

  portENTER_CRITICAL(&sharedStateMux);
  shared.target_left_ticks = 0;
  shared.target_right_ticks = 0;
  portEXIT_CRITICAL(&sharedStateMux);
}

// Debug telemetry
void log_state()
{
  const char *names[] = {"WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
  dprintln(String("state:") + names[state]);
}

void setup()
{
  Serial.begin(115200);

  prefs.begin("robot_base", false);
  loadParamsFromNvs();

  // claim the pins and zero them before WiFi/anything else gets a chance to run.
  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_A_PWM_PIN, 0);
  digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  analogWrite(MOTOR_B_PWM_PIN, 0);
  digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(MOT_A_ENCODER_A, MOT_A_ENCODER_B);
  encoderA.clearCount();
  encoderB.attachFullQuad(MOT_B_ENCODER_A, MOT_B_ENCODER_B);
  encoderB.clearCount();

  controllerA.begin(&inputA, &outputA, &setpointA, PID_P, PID_I, PID_D);
  controllerB.begin(&inputB, &outputB, &setpointB, PID_P, PID_I, PID_D);
  controllerA.setOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  controllerB.setOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  controllerA.start();
  controllerB.start();

  statusLedBegin(statusLed, LOW_BATTERY_LED_PIN);

  // WiFi + WebSerial first, so the self-test below can report its result.
  Serial.println("connecting wifi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult(); // best-effort; WebSerial just won't be reachable if this fails

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "robot_base. Open http://" + WiFi.localIP().toString() + "/webserial"); });
  WebSerial.begin(&server);
  dprintln("wifi ip: " + WiFi.localIP().toString());
  WebSerial.onMessage([](const String &msg)
                      {
    if (msg == "freewheel on")
    {
      freewheelMode = true;
      dprintln("freewheel on -- motors disabled, encoders still counting");
    }
    else if (msg == "freewheel off")
    {
      freewheelMode = false;
      dprintln("freewheel off");
    } });
  server.begin();

  // Runs before the PID task exists
  SelftestResult testA = runMotorSelftest(MOTOR_A_DIRECTION_PIN, MOTOR_A_PWM_PIN, encoderA);
  SelftestResult testB = runMotorSelftest(MOTOR_B_DIRECTION_PIN, MOTOR_B_PWM_PIN, encoderB);
  selftestPassed = testA.passed && testB.passed;

  dprintln(String(selftestPassed ? "self-test passed" : "self-test FAILED") +
           " | A fwd:" + testA.forward_delta + " bwd:" + testA.backward_delta +
           " | B fwd:" + testB.forward_delta + " bwd:" + testB.backward_delta);

  encoderA.clearCount();
  encoderB.clearCount();
  shared.faulted = !selftestPassed;

  initOdomMsg();
  initBatteryMsg();
  lastBatteryVoltage = measure_battery();

  xTaskCreatePinnedToCore(controlTask, "PID_Task", 4096, NULL, 1, NULL, 1);

  set_microros_wifi_transports(
      const_cast<char *>(WIFI_SSID),
      const_cast<char *>(WIFI_PASSWORD),
      const_cast<char *>(MICROROS_AGENT_IP),
      MICROROS_AGENT_PORT);

  state = WAITING_AGENT;
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
      destroy_entities();
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      EXECUTE_EVERY_N_MS(odomPublishPeriodMs, publish_odom());
      EXECUTE_EVERY_N_MS(1000, publish_battery());
      EXECUTE_EVERY_N_MS(1000, {
        portENTER_CRITICAL(&sharedStateMux);
        status_msg.data = shared.faulted ? 1 : 0;
        portEXIT_CRITICAL(&sharedStateMux);
        rcl_publish(&status_publisher, &status_msg, NULL);
      });
    }
    break;
  case AGENT_DISCONNECTED:
    dprintln("agent lost, stopping");
    rosLog(logger, rcl_interfaces__msg__Log__WARN, "agent disconnected");
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }

  static bool watchdogTripped = false;
  portENTER_CRITICAL(&sharedStateMux);
  unsigned long lastCmdMs = shared.last_cmd_ms;
  unsigned long cmdVelTimeoutMs = shared.cmd_vel_timeout_ms;
  portEXIT_CRITICAL(&sharedStateMux);
  bool tripped = millis() - lastCmdMs > cmdVelTimeoutMs;
  if (tripped && !watchdogTripped)
  {
    dprintln("cmd_vel watchdog: no command, stopping");
    if (state == AGENT_CONNECTED)
      rosLog(logger, rcl_interfaces__msg__Log__WARN, "cmd_vel watchdog: no command, stopping");
  }
  watchdogTripped = tripped;

  static unsigned long last_battery_read_ms = 0;
  if (millis() - last_battery_read_ms >= 500)
  {
    last_battery_read_ms = millis();
    lastBatteryVoltage = measure_battery();
  }

  portENTER_CRITICAL(&sharedStateMux);
  bool faulted = shared.faulted;
  portEXIT_CRITICAL(&sharedStateMux);
  statusLedUpdate(statusLed, faulted, lastBatteryVoltage < LOW_BATTERY_THRESHOLD_V, false, state == AGENT_CONNECTED);

  static unsigned long last_log_ms = 0;
  if (millis() - last_log_ms >= 1000)
  {
    last_log_ms = millis();
    log_state();

    portENTER_CRITICAL(&sharedStateMux);
    double x = shared.x, y = shared.y, theta = shared.theta;
    portEXIT_CRITICAL(&sharedStateMux);
    dprintln("odom x:" + String(x) + " y:" + String(y) + " theta:" + String(theta));
  }

  WebSerial.loop();
}
