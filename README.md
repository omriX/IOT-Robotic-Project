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

## ESP32 SDK version used in this project: 
Arduino-ESP32 core: add the version used for the final build. (**todo**)
ESP32 board: add the exact model. (**todo**)

## Arduino/ESP32 libraries used in this project:
* ESP32Encoder - version **XXXXX**
* ArduPID - version **XXXXX**
* AsyncTCP - version **XXXXX**
* ESPAsyncWebServer - version **XXXXX**
* WebSerial - version **XXXXX**
* WiFi and FreeRTOS - version **XXXXX**

## Hardware components and quantities:
* ESP32 board - 1
* Motors and encoders - the firmware controls two motors with encoder feedback
* Motor driver - (**todo**)
* Battery, voltage measurement components, and low-battery LED - (**todo**)

## Connection diagram:

![Main ESP32 connection diagram](Documentation/main_esp32_pin_diagram.svg)

## Project Poster:
**todo**
 
This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
