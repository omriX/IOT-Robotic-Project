# Main ESP32 input/output pin mapping

Repository review date: 2026-10-19.

![Main ESP32 input/output connection diagram](main_esp32_pin_diagram.svg)

[Open the diagram as an SVG](main_esp32_pin_diagram.svg) for printing or inclusion in the submission.

## Carrier-board headers observed in the photo

The supplied hardware photo shows the ESP32 mounted on a carrier/expansion board with header groups labelled **G / V / S**. These expose separate ground, supply, and signal contacts for a GPIO group. They are not three copies of the GPIO and must not be treated as electrically shorted together. The updated diagram shows these groups schematically; its positions do not reproduce the physical header ordering. Only S is connected to the firmware-mapped signal in the drawing. The photo does not verify the voltage of V, rail routing, hidden traces, or every attached G/V wire. Those details still require hardware verification.

This mapping describes the robot-base ESP32 selected by the root `platformio.ini` (`board = esp32dev`). That build compiles `see_sys_robot_encoder/robot_base/platformio_main.cpp`, which includes `robot_base.ino`. GPIO numbers below are ESP32 GPIO identifiers, not physical header positions.

## Assigned GPIOs

Direction is relative to the ESP32. Motor A drives the **right wheel**; Motor B drives the **left wheel**, as recorded in the calibration log and configured by `MOTOR_A_IS_LEFT = false`.

| ESP32 GPIO | Direction | Connected subsystem / signal | Firmware constant | Purpose |
| --- | --- | --- | --- | --- |
| 27 | Output, digital | Right motor (Motor A), driver direction signal | `MOTOR_A_DIRECTION_PIN` | Selects motor drive direction. |
| 14 | Output, PWM | Right motor (Motor A), driver PWM signal | `MOTOR_A_PWM_PIN` | Controls motor drive using `analogWrite()`. |
| 13 | Output, digital | Left motor (Motor B), driver direction signal | `MOTOR_B_DIRECTION_PIN` | Selects motor drive direction. |
| 12 | Output, PWM | Left motor (Motor B), driver PWM signal | `MOTOR_B_PWM_PIN` | Controls motor drive using `analogWrite()`. |
| 23 | Input, digital | Right motor encoder, channel A | `MOT_A_ENCODER_A` | Quadrature encoder feedback. |
| 22 | Input, digital | Right motor encoder, channel B | `MOT_A_ENCODER_B` | Quadrature encoder feedback. |
| 19 | Input, digital | Left motor encoder, channel A | `MOT_B_ENCODER_A` | Quadrature encoder feedback. |
| 21 | Input, digital | Left motor encoder, channel B | `MOT_B_ENCODER_B` | Quadrature encoder feedback. |
| 34 | Input, analog | Battery voltage sensing, scaled voltage-divider output | `BATTERY_PIN` | Measures battery voltage using `analogRead()`. This is the sensing connection, not the battery power feed. |
| 2 | Output, digital | Status / low-battery LED | `LOW_BATTERY_LED_PIN` | Shows operating state, including critical battery, faults, self-test, and agent connection. |

Total: **10 explicitly assigned application GPIOs: 5 inputs and 5 outputs**, including two PWM outputs.

The motor signals go to the motor driver. The earlier motor sketches identify it as an **L293D** in comments, but the repository does not provide its physical terminal or IC pin mapping. The direction/PWM names above reproduce the firmware interface; they do not establish which L293D input or enable terminal each wire reaches.

Both encoders use `attachFullQuad()` with internal weak pull-ups requested in setup. A positive raw motor command moved both wheels backward in the recorded calibration; the current firmware compensates with `LEFT_DIR_SIGN = -1.0` and `RIGHT_DIR_SIGN = -1.0`.

## Battery measurement and LED

The firmware estimates battery voltage as:

```text
V_battery = (analogRead(GPIO34) / 4095.0) × 3.3 × 2.8
```

The configured low-battery threshold is **6.6 V**. Earlier sketches describe this as a 2S battery threshold of 3.3 V per cell. The separate `battery VBAT factor.txt` note reports a measured sensing ratio of `0.356 × actual battery voltage`, approximately consistent with the firmware's inverse factor of 2.8. That note also contains unexplained values of 3 V and 1.18 V; their measurement points are unspecified, so they should not replace the firmware threshold in the submission.

GPIO 2 is shared among status indications. It is not exclusively a low-battery indicator in the current firmware: triple blink indicates fault, double blink critical battery, fast blink self-test, slow blink no agent, and solid on normal connected operation. Fault has the highest indication priority.

## Communication and power connections

The firmware uses Wi-Fi for micro-ROS and WebSerial, with no external communication GPIOs explicitly assigned for those connections. It also starts `Serial` at 115200 baud without explicitly selecting TX/RX pins. The exact board UART/USB wiring therefore needs the board pinout or hardware inspection before it can be listed as a verified physical connection.

The repository does not establish the battery-to-board power route, regulator connections, encoder supply connections, motor power terminals, ground wiring, voltage-divider resistor values, or physical header numbering. The `Documentation/connection diagram` folder contains only placeholder text files, with no populated Fritzing schematic. These physical connections need hardware verification to complete a full wiring diagram. Other GPIOs are not explicitly assigned by this application; this does not establish that they are available or physically unconnected.

## Conflicting older pin definitions

`ESP32/parameters.h` defines the following separate set of peripherals. The active robot-base firmware does not include this file, and there is no populated sketch in `ESP32/` establishing their use on the main controller.

| Peripheral | Pin definitions in `ESP32/parameters.h` | Overlap with active robot-base assignments |
| --- | --- | --- |
| 4×3 keypad | R1=4, R2=12, R3=13, R4=19; C1=21, C2=27, C3=33 | GPIO 12, 13, 19, 21, 27 |
| INMP441 I2S microphone | WS=14, SD=15, SCK=32 | GPIO 14 |
| MAX98357A audio output | BCK=26, WS=25, DATA=22 | GPIO 22 |
| OLED | Width=128, height=64; no GPIOs defined | No pin assignment to compare |

These definitions conflict if used simultaneously with the active robot-base configuration on the same ESP32. The repository does not establish whether they belong to another board or an earlier design, so they are excluded from the confirmed main-controller table.

## Repository evidence

- [Active build configuration](../platformio.ini): selects the robot-base source directory, ESP32 board target, and build entry point.
- [Build entry point](../see_sys_robot_encoder/robot_base/platformio_main.cpp): includes the active Arduino sketch.
- [Current pin constants and wheel mapping](../see_sys_robot_encoder/robot_base/robot_config.h): GPIO assignments, battery conversion, threshold, and motor side configuration.
- [Current firmware](../see_sys_robot_encoder/robot_base/robot_base.ino): `motors()`, `measure_battery()`, and `setup()` confirm how the pins are used.
- [LED implementation](../see_sys_robot_encoder/robot_base/status_led.h): status patterns and priority.
- [Recorded calibration](../see_sys_robot_encoder/docs/calibration_log_1.md): motor identity/direction table confirms A=right and B=left.
- [Original motor sketch](../see_sys_robot_encoder/car_encoder_motors/car_encoder_motors.ino), [Wi-Fi motor sketch](../see_sys_robot_encoder/car_encoder_motors_wifi/car_encoder_motors_wifi.ino), and [calibration sketch](../see_sys_robot_encoder/robot_calibration/robot_calibration.ino): agree with the current ten GPIO assignments.
- [Battery sensing note](../see_sys_robot_encoder/battery%20VBAT%20factor.txt): measured voltage scaling.
- [Separate peripheral definitions](../ESP32/parameters.h): conflicting keypad/audio assignments, excluded from the active mapping.
