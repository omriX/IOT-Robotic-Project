## Robotic Project (Rocky) by : 
Mishael Mandeleyl, Omri Ben Zion, Omri Hamani 
  
## Details about the project : 
Rocky is a robotic platform designed to receive and execute movement commands through ROS-based communication. The intended system reports the robot's final position and battery level and displays a visual representation of the path traveled. 
Intended System Capabilities:
- receive movement commands through ROS and execute them on the robot.
- Report the robot's position after executing movement commands.
- Report battery status.
- Display the path traveled visually.

## Folder description :
* ESP32: firmware configuration files and parameters.
* Documentation: wiring diagram + basic operating instructions
* Unit Tests: tests for individual hardware components (input / output devices)
* flutter_app : dart code for our Flutter app.
* Parameters: contains description of parameters and settings that can be modified IN YOUR CODE
* Assets: link to 3D printed parts, Audio files used in this project, Fritzing file for connection diagram (FZZ format) etc

## Arduino/ESP32 libraries used in this project:

* ESP32Encoder - version **0.11.8**
* ArduPID - version **0.2.1**
* AsyncTCP - version **3.3.2**
* ESPAsyncWebServer - version **3.6.0**
* WebSerial - version **2.1.2**
* WiFi - version **2.0.0** (included with Arduino-ESP32)
* FreeRTOS - version **10.4.3** (included with ESP-IDF)

Other required packages:

* micro_ros_arduino - version **2.0.8-humble** (included in `robot/libraries`)
* Preferences - version **2.0.0** (included with Arduino-ESP32; used to store configuration)
* FireTimer - version **1.0.5** (dependency of ArduPID)

## ESP32 SDK version used in this project:

The firmware uses **Arduino-ESP32 2.0.17**, based on **ESP-IDF 4.4.7**. The configured build target is **Espressif ESP32 Dev Module** (`esp32dev`).

For development, you can use the official **Arduino IDE** with the Arduino-ESP32 board package, or **PlatformIO** for easy integration with **Visual Studio Code**. The repository includes a `platformio.ini` configuration using the **Espressif 32 platform 6.9.0**. These tools are needed to build and upload firmware; they are not required to operate the robot after the firmware is flashed.

## Hardware components and quantities:
* ESP32 board - 1
* Motors and encoders - the firmware controls two motors with encoder feedback
* Motor driver - (**todo**)
* Battery, voltage measurement components, and low-battery LED - (**todo**)

## Connection diagram:
 **todo**

![Main ESP32 connection diagram](Documentation/main_esp32_pin_diagram.svg)

## Project Poster:
**todo**
 
This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
