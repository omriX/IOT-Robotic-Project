// robot_base.ino
//
// Proves the toolchain end-to-end before any real topics exist -- WebSerial
// debug channel + the standard micro-ROS agent connect/disconnect lifecycle
// over Wi-Fi/UDP, publishing a plain std_msgs/Int32 counter. Motor control,
// odometry, etc. are not here yet -- they land in later stages, on top of
// this same lifecycle skeleton.
//
// Wi-Fi transport was brought forward from Stage B step 4: serial transport
// (set_microros_transports()) hit an unresolved Windows/WSL2 USB-passthrough
// issue (usbipd attach tears down ~45ms after connecting), so Wi-Fi is what's
// actually verified first here. It also means Serial/USB stays free for
// normal Serial.print -- only the serial-transport branch needs that off-limits.
//
// Verify with the agent reachable on the same LAN as the ESP32:
//   docker compose -f docker/docker-compose.yml up agent-udp
//   ros2 topic echo /robot_base/counter

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include "robot_config.h"

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
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t counter_publisher;
std_msgs__msg__Int32 counter_msg;

enum AgentState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState state = WAITING_AGENT;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    rcl_ret_t rc = rcl_publish(&counter_publisher, &counter_msg, NULL);
    (void)rc;
    counter_msg.data++;
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
      &counter_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "robot_base/counter"));

  const unsigned int timer_timeout_ms = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout_ms), timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc;
  rc = rcl_publisher_fini(&counter_publisher, &node);
  (void)rc;
  rc = rcl_timer_fini(&timer);
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
  WebSerial.print(names[state]);
  WebSerial.print(" counter:");
  WebSerial.println(counter_msg.data);
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
  counter_msg.data = 0;
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
