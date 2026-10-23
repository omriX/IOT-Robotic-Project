// ros_log.h
//
// Publishes rcl_interfaces/msg/Log on /robot_base/log.
#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl_interfaces/msg/log.h>
#include <micro_ros_utilities/string_utilities.h>

struct RosLogger
{
  rcl_publisher_t publisher;
  rcl_node_t *node = nullptr;
  uint8_t min_level = rcl_interfaces__msg__Log__INFO;
};

inline rcl_interfaces__msg__Log &rosLoggerMsg()
{
  static rcl_interfaces__msg__Log msg;
  static bool ready = false;
  if (!ready)
  {
    rcl_interfaces__msg__Log__init(&msg);
    msg.name = micro_ros_string_utilities_set(msg.name, "robot_base");
    ready = true;
  }
  return msg;
}

inline bool rosLoggerInit(RosLogger &logger, rcl_node_t *node)
{
  logger.node = node;
  return RCL_RET_OK == rclc_publisher_init_default(
                           &logger.publisher, node,
                           ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                           "robot_base/log");
}

inline void rosLoggerFini(RosLogger &logger)
{
  rcl_publisher_fini(&logger.publisher, logger.node);
}

inline void rosLog(RosLogger &logger, uint8_t level, const char *text)
{
  if (level < logger.min_level)
    return;
  rcl_interfaces__msg__Log &msg = rosLoggerMsg();
  msg.level = level;
  msg.stamp.sec = millis() / 1000;
  msg.stamp.nanosec = (millis() % 1000) * 1000000UL;
  msg.msg = micro_ros_string_utilities_set(msg.msg, text);
  rcl_publish(&logger.publisher, &msg, NULL);
}

#define ROS_LOGF(logger, level, fmt, ...)                     \
  do                                                          \
  {                                                           \
    char _log_buf[128];                                       \
    snprintf(_log_buf, sizeof(_log_buf), fmt, ##__VA_ARGS__); \
    rosLog(logger, level, _log_buf);                          \
  } while (0)
