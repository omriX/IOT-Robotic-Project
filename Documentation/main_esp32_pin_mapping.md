# ESP32 pin mapping

![ESP32 connection diagram](main_esp32_pin_diagram.svg)

[Open the diagram as an SVG](main_esp32_pin_diagram.svg) for printing.

The robot uses an ESP32 Dev Module (`esp32dev`) on a carrier board whose header groups are labelled
**G / V / S** (ground, supply, signal). Only the S contact carries the GPIO signal; the diagram shows
the groups schematically, not in their physical order. GPIO numbers below are ESP32 GPIO identifiers,
not header positions. All constants are in
[robot_hardware.h](../robot/robot_base/robot_hardware.h).

## Assigned GPIOs

Direction is relative to the ESP32. Motor A drives the **right wheel** and Motor B the **left wheel**
(`MOTOR_A_IS_LEFT = false`, see [hardware_calibration.md](../robot/docs/hardware_calibration.md)).

| ESP32 GPIO | Direction | Connected to | Firmware constant | Purpose |
| --- | --- | --- | --- | --- |
| 27 | Output, digital | Right motor (A), driver direction | `MOTOR_A_DIRECTION_PIN` | Motor direction |
| 14 | Output, PWM | Right motor (A), driver PWM | `MOTOR_A_PWM_PIN` | Motor drive, `analogWrite()` |
| 13 | Output, digital | Left motor (B), driver direction | `MOTOR_B_DIRECTION_PIN` | Motor direction |
| 12 | Output, PWM | Left motor (B), driver PWM | `MOTOR_B_PWM_PIN` | Motor drive, `analogWrite()` |
| 23 | Input, digital | Right encoder, channel A | `MOT_A_ENCODER_A` | Quadrature feedback |
| 22 | Input, digital | Right encoder, channel B | `MOT_A_ENCODER_B` | Quadrature feedback |
| 19 | Input, digital | Left encoder, channel A | `MOT_B_ENCODER_A` | Quadrature feedback |
| 21 | Input, digital | Left encoder, channel B | `MOT_B_ENCODER_B` | Quadrature feedback |
| 34 | Input, analog | Battery voltage divider output | `BATTERY_PIN` | Battery voltage, `analogRead()` |
| 2 | Output, digital | Status LED | `LOW_BATTERY_LED_PIN` | Connection, fault and battery status |

10 GPIOs in total: 5 inputs and 5 outputs, two of them PWM.

## Motors and encoders

The motor driver is an **L293D**. Each motor takes a direction signal and a PWM signal from the ESP32.
A positive raw command turns both wheels backward; the firmware compensates with
`LEFT_DIR_SIGN = RIGHT_DIR_SIGN = -1.0`.

Both encoders run in full quadrature (`attachFullQuad()`) with the ESP32's internal pull-ups, giving
about 4216 counts per wheel revolution.

## Battery measurement

The battery is a **2S Li-ion/LiPo** pack (7.4 V nominal, 8.4 V full), read on GPIO 34 through a
voltage divider:

```text
V_battery = (analogRead(GPIO34) / 4095.0) × 3.3 × 2.8
```

The divider factor 2.8 was checked against a multimeter (measured pin ratio 0.356 ≈ 1 / 2.8; see the
`battery` hardware test in [unit_tests](../unit_tests/README.md)). The low-battery threshold is
**6.6 V**, 3.3 V per cell.

## Status LED (GPIO 2)

| Pattern | Meaning |
| --- | --- |
| Triple blink | Fault: boot self-test failed, `/cmd_vel` is ignored (highest priority) |
| Double blink | Battery below 6.6 V |
| Slow blink | Waiting for the micro-ROS agent |
| Solid on | Connected, all OK |

## Communication

micro-ROS and WebSerial run over Wi-Fi, so no extra GPIOs are used for communication. USB Serial
(115200 baud) is used for flashing, debug output, and optionally as the micro-ROS transport
(`robot_base_serial` build).
