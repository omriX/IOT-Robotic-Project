"""Link the ESP32 micro-ROS archive already included in this repository."""
Import("env")

env.Append(
    LIBPATH=[env.subst("$PROJECT_DIR/see_sys_robot_encoder/libraries/micro_ros_arduino/src/esp32")],
    LIBS=["microros"],
)
