# Drive validation — 2026-10-17

## Changes

- Keep the encoder heading of each straight segment using bounded wheel-target
  correction, alongside the existing per-wheel PI controllers.
- Clear the correction on stops, turns, and direction reversals. Explicit zero
  commands and freewheel mode also reset the wheel controllers.
- Add PlatformIO build support. Copy `robot_base/network_local.example.h` to
  `robot_base/network_local.h` and enter local Wi-Fi and agent settings. The local
  file is ignored by Git.

## Validation

- Drive math and correction suite: 34/34 checks passed.
- PlatformIO ESP32 firmware build passed.
- Python syntax: all five ROS package Python files parsed successfully.
- ROS `colcon test`: completed, but no ROS package tests are defined.
- Xacro expanded and resulting URDF parsed successfully (four links, three joints).
- Live encoder feedback checked with wheels raised and on the floor.
- Operator confirmed straight driving after uploading the correction.
- In the sampled corrected floor run, accumulated wheel mismatch peaked at
  30 ticks and returned near zero (four ticks at the last powered sample).
- Operator confirmed an in-place turn works; angle accuracy was not measured.

## Radius calibration

- Previous radius: 0.022000 m.
- Reported distance: 1.0419 m; tape-measured distance: 1.04 m.
- Accepted radius: 0.021961 m. The calibration tool's parameter update timed out;
  a subsequent ROS CLI update succeeded and readback confirmed the accepted value.
- Save that value in firmware defaults and the visual robot description. Existing
  NVS settings continue to take precedence over firmware defaults.

## Still pending

- Measured wheel-spacing calibration and physical arc/turn accuracy.
- Repeated physical validation at different speeds and in reverse.
- Startup motor self-test intermittently failed before passing on reset; its cause
  has not been established.

## Reproduce automated checks

From the repository root:

```sh
g++ -std=c++17 -Wall -Wextra -pedantic -I robot/robot_base \
  robot/test/test_diff_drive.cpp -o /tmp/robot-drive-tests
/tmp/robot-drive-tests
pio run -e robot_base
```
