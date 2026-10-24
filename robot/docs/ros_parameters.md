# ROS parameters

All runtime parameters belong to the firmware node `/robot_base` on the ESP32, served by
`rclc_parameter_server`. They can be read and changed live from the PC, no reflash needed. The
host-side nodes (`driver_node`, `drive_test`, `auto_calibrate`, `low_speed_test`) have no ROS
parameters; they take command-line flags instead.

## Reference

| Parameter | Type | Default | Range checked | Meaning |
|---|---|---|---|---|
| `wheel_radius_m` | double | 0.021933 | no | Wheel radius in m. Converts `cmd_vel` to wheel ticks and ticks back to `/odom` |
| `wheel_base_m` | double | 0.119528 | no | Distance between the wheel contact points in m. Sets turn rate and odom heading |
| `max_linear_mps` | double | 0.2 | no | `cmd_vel` `linear.x` is clamped to ±this |
| `max_angular_rps` | double | 1.5 | no | `cmd_vel` `angular.z` is clamped to ±this |
| `cmd_vel_timeout_ms` | int | 1000 | no | Motors stop if no `/cmd_vel` arrives for this long |
| `odom_rate_hz` | int | 20 | > 0 | `/odom` publish rate. The period is `1000 / hz` ms, so keep it ≤ 1000 |
| `log_level` | int | 20 | no | Lowest severity sent on `/robot_base/log`: 10 DEBUG, 20 INFO, 30 WARN, 40 ERROR, 50 FATAL |
| `low_speed_assist` | bool | true | – | false turns off the feedforward, the startup kick and the PID direction limit together |
| `ff_static_pwm` | int | 25 | 0–230 | PWM added in the setpoint's direction whenever a wheel is commanded to move |
| `ff_pwm_per_tick` | double | 0.2 | 0–2.0 | Extra PWM per target tick/interval |
| `kick_pwm` | int | 65 | 0–255 | PWM of the startup kick from standstill |
| `kick_ms` | int | 0 | 0–500 | Startup kick length; 0 disables the kick |

Defaults are in `robot_base/robot_hardware.h` (`WHEEL_RADIUS_M`, `WHEEL_BASE_M`) and
`robot_base/robot_config.h` (`CMD_VEL_TIMEOUT_MS`, `DEFAULT_*`). A value outside a checked range
is rejected and the old value stays. Unchecked parameters accept anything, including 0 or
negative values, so double-check before setting them.

## Commands

Inside the `ros_tools` shell (see [ubuntu_lab_setup.md](ubuntu_lab_setup.md)):

```sh
ros2 param list /robot_base
ros2 param get /robot_base wheel_radius_m
ros2 param set /robot_base max_linear_mps 0.1
```

## Persistence

Every accepted `ros2 param set` is also written to the ESP32's flash (NVS, namespace
`robot_base`). At boot a stored value takes precedence over the firmware default, so a changed
value survives resets **and reflashing**. There is no reset command: to go back to a default, set
it explicitly. To make a new value the default for fresh boards, also update it in the header.

Some NVS keys differ from the parameter names because of the 15-character key limit:
`cmd_vel_timeout_ms` → `cmd_to_ms`, `low_speed_assist` → `ls_assist`,
`ff_pwm_per_tick` → `ff_pwm_tick`. The rest use the parameter name.

## When changes take effect

- Speed limits, wheel geometry, low-speed assist and kick: from the next `/cmd_vel` or control
  loop cycle. Geometry changes also apply to `/odom` from then on; the pose already accumulated is
  not recomputed.
- `odom_rate_hz` and `log_level`: immediately.

## Related docs

- [live_calibration_log.md](live_calibration_log.md): measuring `wheel_radius_m` and `wheel_base_m`
  with `auto_calibrate`, and the values recorded so far.
- [low_speed_tuning.md](low_speed_tuning.md): measuring and comparing the feedforward and kick values.
- [ubuntu_lab_setup.md](ubuntu_lab_setup.md): why `cmd_vel_timeout_ms` is 1000 and not 500.
