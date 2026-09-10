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

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include "robot_config.h"
#include "diff_drive.h"

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

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

enum AgentState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState state = WAITING_AGENT;

// Run the verified kinematics and log the result.
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  WheelVelocities wv = wheelLinearVelocities(msg->linear.x, msg->angular.z, WHEEL_BASE_M);
  WheelSetpoints sp = cmdVelToWheelSetpoints(
      msg->linear.x, msg->angular.z,
      WHEEL_BASE_M, TICKS_PER_METER, CONTROL_PERIOD_S,
      LEFT_DIR_SIGN, RIGHT_DIR_SIGN, MAX_TICKS_PER_INTERVAL);

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

  RCCHECK(rclc_subscription_init_best_effort(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc;
  rc = rcl_subscription_fini(&cmd_vel_subscriber, &node);
  (void)rc;
  rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  (void)rc;
  rclc_support_fini(&support);
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

  // WiFi + WebSerial first: this is the debug channel that stays alive once Serial itself is handed to micro-ROS below.
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult(); // best-effort; WebSerial just won't be reachable if this fails

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "robot_base. Open http://" + WiFi.localIP().toString() + "/webserial"); });
  WebSerial.begin(&server);
  server.begin();

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
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }

  static unsigned long last_log_ms = 0;
  if (millis() - last_log_ms >= 1000)
  {
    last_log_ms = millis();
    log_state();
  }

  WebSerial.loop();
}
