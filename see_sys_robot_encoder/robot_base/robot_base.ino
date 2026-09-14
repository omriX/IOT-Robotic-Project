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

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ESP32Encoder.h>
#include <ArduPID.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "robot_config.h"
#include "diff_drive.h"
#include "status_led.h"
#include "selftest.h"
#include "ros_log.h"

constexpr double TICKS_PER_METER = ticksPerMeter(COUNTS_PER_WHEEL_REV, WHEEL_RADIUS_M);
constexpr double CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0;

#define RCCHECK(fn)            \
  {                            \
    rcl_ret_t temp_rc = fn;    \
    if (temp_rc != RCL_RET_OK) \
    {                          \
      return false;            \
    }                          \
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
};
SharedState shared;

// WebSerial "freewheel on"/"freewheel off" -- cuts motor output so the
// wheels can be turned by hand to test odometry, without the PID fighting it.
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
    portEXIT_CRITICAL(&sharedStateMux);

    if (faulted || millis() - lastCmdMs > CMD_VEL_TIMEOUT_MS)
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
    OdometryDelta odom = integrateOdometry(pose, deltaLeftTicks, deltaRightTicks, TICKS_PER_METER, WHEEL_BASE_M, CONTROL_PERIOD_S);

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
RosLogger logger;
bool timeSynced = false;

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
  portEXIT_CRITICAL(&sharedStateMux);
  if (faulted)
    return;

  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  WheelVelocities wv = wheelLinearVelocities(msg->linear.x, msg->angular.z, WHEEL_BASE_M);
  WheelSetpoints sp = cmdVelToWheelSetpoints(
      msg->linear.x, msg->angular.z,
      WHEEL_BASE_M, TICKS_PER_METER, CONTROL_PERIOD_S,
      LEFT_DIR_SIGN, RIGHT_DIR_SIGN, MAX_TICKS_PER_INTERVAL);

  portENTER_CRITICAL(&sharedStateMux);
  shared.target_left_ticks = sp.left_ticks_per_interval;
  shared.target_right_ticks = sp.right_ticks_per_interval;
  shared.last_cmd_ms = millis();
  portEXIT_CRITICAL(&sharedStateMux);

  WebSerial.print("cmd_vel v:");
  WebSerial.print(msg->linear.x);
  WebSerial.print(" w:");
  WebSerial.print(msg->angular.z);
  WebSerial.print(" | wheel_mps L:");
  WebSerial.print(wv.left_mps);
  WebSerial.print(" R:");
  WebSerial.print(wv.right_mps);
  WebSerial.print(" | ticks_per_interval L:");
  WebSerial.print(sp.left_ticks_per_interval);
  WebSerial.print(" R:");
  WebSerial.println(sp.right_ticks_per_interval);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  timeSynced = (RMW_RET_OK == rmw_uros_sync_session(1000)) && rmw_uros_epoch_synchronized();

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

  if (!rosLoggerInit(logger, &node))
    return false;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  rosLog(logger, rcl_interfaces__msg__Log__INFO, "robot_base connected");
  if (!timeSynced)
    rosLog(logger, rcl_interfaces__msg__Log__WARN, "time sync failed, timestamps fall back to millis()");
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

float measure_battery()
{
  float adc_voltage = (analogRead(BATTERY_PIN) / ADC_MAX_VALUE) * ADC_LOGIC_LEVEL_V;
  return adc_voltage * BATTERY_VOLTAGE_DIVIDER_FACTOR;
}

void publish_odom()
{
  portENTER_CRITICAL(&sharedStateMux);
  double x = shared.x, y = shared.y, theta = shared.theta;
  double v = shared.linear_v, w = shared.angular_w;
  portEXIT_CRITICAL(&sharedStateMux);

  if (timeSynced)
  {
    int64_t nanos = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = (int32_t)(nanos / 1000000000LL);
    odom_msg.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);
  }
  else
  {
    odom_msg.header.stamp.sec = millis() / 1000;
    odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000UL;
  }

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

// Debug telemetry -- WebSerial only, never Serial, once micro-ROS owns the UART.
void log_state()
{
  const char *names[] = {"WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
  WebSerial.print("state:");
  WebSerial.println(names[state]);
}

void setup()
{
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

  nav_msgs__msg__Odometry__init(&odom_msg);
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
  // planar robot: no information on z/roll/pitch
  odom_msg.pose.covariance[0] = 0.01;
  odom_msg.pose.covariance[7] = 0.01;
  odom_msg.pose.covariance[14] = 1e6;
  odom_msg.pose.covariance[21] = 1e6;
  odom_msg.pose.covariance[28] = 1e6;
  odom_msg.pose.covariance[35] = 0.05;
  odom_msg.twist.covariance[0] = 0.01;
  odom_msg.twist.covariance[7] = 0.01;
  odom_msg.twist.covariance[14] = 1e6;
  odom_msg.twist.covariance[21] = 1e6;
  odom_msg.twist.covariance[28] = 1e6;
  odom_msg.twist.covariance[35] = 0.05;

  // WiFi + WebSerial first, so the self-test below can report its result.
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult(); // best-effort; WebSerial just won't be reachable if this fails

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "robot_base. Open http://" + WiFi.localIP().toString() + "/webserial"); });
  WebSerial.begin(&server);
  WebSerial.onMessage([](const String &msg)
                      {
    if (msg == "freewheel on")
    {
      freewheelMode = true;
      WebSerial.println("freewheel on -- motors disabled, encoders still counting");
    }
    else if (msg == "freewheel off")
    {
      freewheelMode = false;
      WebSerial.println("freewheel off");
    } });
  server.begin();

  // Runs before the PID task exists
  SelftestResult testA = runMotorSelftest(MOTOR_A_DIRECTION_PIN, MOTOR_A_PWM_PIN, encoderA);
  SelftestResult testB = runMotorSelftest(MOTOR_B_DIRECTION_PIN, MOTOR_B_PWM_PIN, encoderB);
  selftestPassed = testA.passed && testB.passed;

  WebSerial.print(selftestPassed ? "self-test passed" : "self-test FAILED");
  WebSerial.print(" | A fwd:");
  WebSerial.print(testA.forward_delta);
  WebSerial.print(" bwd:");
  WebSerial.print(testA.backward_delta);
  WebSerial.print(" | B fwd:");
  WebSerial.print(testB.forward_delta);
  WebSerial.print(" bwd:");
  WebSerial.println(testB.backward_delta);

  encoderA.clearCount();
  encoderB.clearCount();
  shared.faulted = !selftestPassed;

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
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      EXECUTE_EVERY_N_MS(CONTROL_PERIOD_MS, publish_odom());
      EXECUTE_EVERY_N_MS(1000, {
        portENTER_CRITICAL(&sharedStateMux);
        status_msg.data = shared.faulted ? 1 : 0;
        portEXIT_CRITICAL(&sharedStateMux);
        rcl_publish(&status_publisher, &status_msg, NULL);
      });
    }
    break;
  case AGENT_DISCONNECTED:
    WebSerial.println("agent lost, stopping");
    rosLog(logger, rcl_interfaces__msg__Log__WARN, "agent disconnected");
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }

  static bool watchdogTripped = false;
  portENTER_CRITICAL(&sharedStateMux);
  unsigned long lastCmdMs = shared.last_cmd_ms;
  portEXIT_CRITICAL(&sharedStateMux);
  bool tripped = millis() - lastCmdMs > CMD_VEL_TIMEOUT_MS;
  if (tripped && !watchdogTripped)
  {
    WebSerial.println("cmd_vel watchdog: no command, stopping");
    if (state == AGENT_CONNECTED)
      rosLog(logger, rcl_interfaces__msg__Log__WARN, "cmd_vel watchdog: no command, stopping");
  }
  watchdogTripped = tripped;

  portENTER_CRITICAL(&sharedStateMux);
  bool faulted = shared.faulted;
  portEXIT_CRITICAL(&sharedStateMux);
  statusLedUpdate(statusLed, faulted, false, false, state == AGENT_CONNECTED);

  static unsigned long last_log_ms = 0;
  if (millis() - last_log_ms >= 1000)
  {
    last_log_ms = millis();
    log_state();

    portENTER_CRITICAL(&sharedStateMux);
    double x = shared.x, y = shared.y, theta = shared.theta;
    portEXIT_CRITICAL(&sharedStateMux);
    WebSerial.print("odom x:");
    WebSerial.print(x);
    WebSerial.print(" y:");
    WebSerial.print(y);
    WebSerial.print(" theta:");
    WebSerial.println(theta);

    WebSerial.print("battery_V:");
    WebSerial.println(measure_battery());
  }

  WebSerial.loop();
}
