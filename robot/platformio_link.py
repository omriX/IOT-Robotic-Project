"""Link the ESP32 micro-ROS archive already included in this repository."""
Import("env")

env.Append(
    LIBPATH=[env.subst("$PROJECT_DIR/robot/libraries/micro_ros_arduino/src/esp32")],
    LIBS=["microros"],
)
