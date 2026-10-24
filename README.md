## Robotic Project (Rocky) by :
Mishael Mandeleyl, Omri Ben Zion, Omri Hamani

## Details about the project :
Rocky is a two-wheeled differential-drive robot that receives and executes movement commands through ROS 2. An ESP32 runs a micro-ROS node that drives the motors with closed-loop speed control on encoder feedback, and reports the robot's position (odometry) and battery level. A PC running ROS 2 Humble in Docker sends the commands and shows the robot and the path it traveled in RViz.

System capabilities:
- Receive movement commands (`/cmd_vel`) through ROS and execute them on the robot.
- Report the robot's position (`/odom`) while and after it moves.
- Report battery status (`/battery_state`) and a low-battery LED warning.
- Display the robot and the path traveled in RViz.
- Drive test courses (square, half circle, out-and-back) using odometry feedback.
- Calibrate the robot live through ROS parameters, without reflashing.

## Folder description :
* Documentation: ESP32 connection diagram and pin mapping.
* robot/robot_base: ESP32 firmware (micro-ROS node, motor control, odometry, battery).
* robot/robot_calibration: standalone ESP32 sketch used to measure the hardware constants.
* robot/ros2_ws: ROS 2 package `robot_base_driver` for the PC (driver node, RViz launch, drive test and calibration tools).
* robot/docker: Docker Compose setup for the micro-ROS agent and the ROS 2 tools.
* robot/docs: setup and operating instructions, ROS parameters, calibration records.
* robot/libraries: custom build of micro_ros_arduino used by the firmware.
* robot/test: PC unit tests for the firmware's drive math.
* unit_tests: hardware tests for each component, ROS driver tests, and an end-to-end smoke test.

## Documentation:
* [Installation and setup](robot/docs/ubuntu_lab_setup.md): install the tools, flash the robot, start the agent, first checks. Start here.
* [RViz live view (Ubuntu)](robot/docs/rviz_live_view.md) and [RViz live view (macOS)](robot/docs/rviz_live_view_macos.md)
* [ROS parameters](robot/docs/ros_parameters.md): every live-tunable setting on the robot.
* [Live calibration](robot/docs/live_calibration.md): wheel radius / wheel base calibration and the drive test courses.
* [Hardware calibration](robot/docs/hardware_calibration.md): bench measurements of encoder counts, geometry and motor directions.
* [ESP32 pin mapping](Documentation/main_esp32_pin_mapping.md)
* [Tests](unit_tests/README.md)

## Arduino/ESP32 libraries used in this project:

* ESP32Encoder - version **0.11.8**
* ArduPID - version **0.2.1**
* AsyncTCP - version **3.3.2**
* ESPAsyncWebServer - version **3.6.0**
* WebSerial - version **2.1.2**
* WiFi - version **2.0.0** (included with Arduino-ESP32)
* FreeRTOS - version **10.4.3** (included with ESP-IDF)

Other required packages:

* micro_ros_arduino - version **2.0.8-humble** (custom build included in `robot/libraries`, see its [README](robot/libraries/README.md))
* Preferences - version **2.0.0** (included with Arduino-ESP32; stores the ROS parameters in flash)
* FireTimer - version **1.0.5** (dependency of ArduPID)

PC side: Docker with the `microros/micro-ros-agent:humble` and `ros:humble-ros-base` images (ROS 2 Humble, RViz 2).

## ESP32 SDK version used in this project:

The firmware uses **Arduino-ESP32 2.0.17**, based on **ESP-IDF 4.4.7**. The configured build target is **Espressif ESP32 Dev Module** (`esp32dev`).

For development, you can use the official **Arduino IDE** with the Arduino-ESP32 board package, or **PlatformIO** for easy integration with **Visual Studio Code**. The repository includes a `platformio.ini` configuration using the **Espressif 32 platform 6.9.0**. These tools are needed to build and upload firmware; they are not required to operate the robot after the firmware is flashed.

## Hardware components and quantities:
* ESP32 Dev Module on a carrier board - 1
* DC motors with quadrature encoders (about 4216 counts per wheel revolution) - 2
* L293D motor driver - 1
* 2S Li-ion/LiPo battery (7.4 V nominal) - 1
* Voltage divider for battery sensing (on GPIO 34) - 1
* Status / low-battery LED (on GPIO 2) - 1

## Connection diagram:

![Main ESP32 connection diagram](Documentation/main_esp32_pin_diagram.svg)

Pin-by-pin details: [ESP32 pin mapping](Documentation/main_esp32_pin_mapping.md).

## Project Poster:
**todo**

This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
