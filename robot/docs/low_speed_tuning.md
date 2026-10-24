# Low-speed start tuning — friction feedforward and startup kick

At low wheel targets the PI controller used to need ~1 s to climb to the PWM where a wheel starts
turning, so the motors hummed in place and then lurched (stick-slip). A 150 PWM / 100 ms startup kick
(branch `omrih/try-kick`) got the wheels moving but made them slip, which put the odom heading off on
in-place turns.

The firmware now adds a friction feedforward to the PID output, `sign(sp) * ff_static_pwm +
ff_pwm_per_tick * sp`, and keeps the startup kick as an option (off by default). All four values are live
ROS parameters, persisted in NVS:

| Parameter | Default | Meaning |
|---|---|---|
| `low_speed_assist` | true | false turns off the feedforward, the kick and the direction limit together: the drive behaves exactly like the plain PI controller of commit `39edb5f`. The other values are kept for when it is turned back on |
| `ff_static_pwm` | 25 | PWM added in the setpoint's direction whenever a wheel is commanded to move |
| `ff_pwm_per_tick` | 0.2 | extra PWM per target tick/interval |
| `kick_pwm` | 65 | PWM of the startup kick |
| `kick_ms` | 0 | kick length; 0 disables the kick |

Keep the feedforward at or below what the wheel really needs. The PID output is limited to the
setpoint's direction, and ArduPID then holds the integral on that side too, so the integral can add to
the feedforward but never take any away. Too much feedforward shows up as a steady overspeed (`in` > `sp`).

WebSerial's `pid` lines show the feedforward (`ff:`) per wheel and `KICK` while a kick is active.

Reference target speeds (30593 ticks/m, 50 ms interval): 0.04 m/s = 61 ticks/interval; an in-place turn at
0.5 rad/s = 46 ticks/interval per wheel, at 0.8 rad/s = 73.

## 1. Measure friction on the floor

Flash `robot_calibration`, robot on the floor (it pivots around the other wheel). Note the battery voltage
(`read`). For each motor and direction, a fine ramp to PWM 100:

```
ramp A 2 100
ramp A 2 -100
ramp B 2 100
ramp B 2 -100
```

| Motor | Direction | Breakaway PWM (up-ramp, first dtick > 0) | Stall PWM (down-ramp, dtick reaches 0) | dtick just above stall |
|---|---|---|---|---|
| A | + |  |  |  |
| A | − |  |  |  |
| B | + |  |  |  |
| B | − |  |  |  |

- Battery V:
- `ff_static_pwm` ≈ the highest stall PWM:
- `kick_pwm` ≈ the highest breakaway PWM + 10:
- Minimum smooth wheel speed ≈ the highest "dtick just above stall" / 1530 m/s:

Flash `robot_base` back: `pio run -e robot_base -t upload`.

## 2. Compare configurations (no reflash)

In the `ros_tools` shell, for each configuration set the parameters, for example:

```
ros2 param set /robot_base ff_static_pwm 0
ros2 param set /robot_base ff_pwm_per_tick 0.0
ros2 param set /robot_base kick_ms 0
```

then run:

```
ros2 run robot_base_driver low_speed_test --alternate
ros2 run robot_base_driver auto_calibrate turn --angle 90 --angular-speed 0.5   # 4x, tape-measure each
ros2 run robot_base_driver auto_calibrate turn --angle 90 --angular-speed 0.8   # 4x
ros2 run robot_base_driver auto_calibrate straight                              # 1 m vs tape
```

Real heading: before the turns, put a strip of tape along the robot's side. After 4×90° measure how far
the side has moved off the tape over a known length L: error = atan(offset / L).

| Config | ff_static_pwm / ff_pwm_per_tick | kick_pwm / kick_ms | Rise at 0.5 rad/s | Peak | Stalls | Heading error after 4×90° @0.5 | @0.8 | 1 m straight error |
|---|---|---|---|---|---|---|---|---|
| C0 baseline | 0 / 0 | – / 0 |  |  |  |  |  |  |
| C1 old kick | 0 / 0.4 | 150 / 100 |  |  |  |  |  |  |
| C2 gentle kick | 0 / 0 | measured / 50 |  |  |  |  |  |  |
| C3 feedforward | measured / 0.2 | – / 0 |  |  |  |  |  |  |
| C4 feedforward + gentle kick | measured / 0.2 | measured / 50 |  |  |  |  |  |  |

Pass:
- rise ≤ 100 ms
- peak ≤ 1.5×
- no stalls at targets above the minimum smooth speed
- heading error ≤ 2° per 90° and no worse than C0
- straight error ≤ 1%

Use the simplest configuration that passes, and write its values into `DEFAULT_FF_*` / `DEFAULT_KICK_*`
in `robot_base/robot_config.h`.

## 3. Wrap-up

1. Raise `MIN_ANGULAR_RPS` in `drive_test.py` (and the `auto_calibrate` turn speed) so that
   `w * wheel_base_m / 2` is at least the minimum smooth wheel speed (0.04 m/s → ≥ 0.67 rad/s).
2. Re-check `wheel_base_m` with `auto_calibrate rotate`, then run `drive_test square` and
   `drive_test out_back` against the tape marks.
3. Record the results in `drive_validation_<date>.md`.
4. If the results drift with battery level, consider scaling the feedforward by the battery voltage (the
   L293D's drop makes the friction PWM depend on it).
