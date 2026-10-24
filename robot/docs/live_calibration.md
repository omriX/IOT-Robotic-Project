# Live calibration — wheel_radius_m / wheel_base_m

Refines the two geometry parameters against the integrated `/odom` output, live through
`ros2 param set` (no reflash). The raw-encoder bench measurements that gave the starting values are in
[hardware_calibration.md](hardware_calibration.md). Parameter details: [ros_parameters.md](ros_parameters.md).

## Setup

Start the agent and open a `ros_tools` shell as in [ubuntu_lab_setup.md](ubuntu_lab_setup.md)
(sections D–F), and wait for the robot to show `AGENT_CONNECTED`. On macOS, start the agent with
`ROBOT_BIND_IP` set instead (see [rviz_live_view_macos.md](rviz_live_view_macos.md)). All commands
below run in the `ros_tools` shell.

## Running auto_calibrate

`auto_calibrate` automates the "command it, then read odom" half of parts 1 and 2 below. You still
measure the real distance or angle with a tape, and it asks before applying anything.

```sh
# wheel_radius_m -- drives 1 m forward at 0.1 m/s (defaults)
ros2 run robot_base_driver auto_calibrate straight
ros2 run robot_base_driver auto_calibrate straight --distance 1.0 --speed 0.1

# wheel_base_m -- rotates 90 deg at 0.5 rad/s (defaults)
ros2 run robot_base_driver auto_calibrate rotate
ros2 run robot_base_driver auto_calibrate rotate --angle 90 --angular-speed 0.5
```

It asks `ready? [y/N]` first: give the robot clear floor space and wheels on the ground before answering
`y`. It drives the segment, prints the odom-reported distance or angle, asks for the tape-measured value,
prints the corrected constant, and asks before applying it with `ros2 param set`. `Ctrl-C` stops the
robot immediately at any point.

`ros2_ws/src` is copied into the `ros_tools` image at build time, so rebuild after editing the package:

```sh
docker compose -f docker/docker-compose.yml build ros_tools
```

## Odom-driven moves and stop-margin tuning (no tape)

These trust `/odom` and stop a move once odom says it reached the target, slowing down over the last
`--slow-zone` first. The `tune-*` modes repeat the move, alternating forward/back (or left/right) so the
robot stays roughly in place. After each run they stop earlier by however far it coasted past the target,
until two runs in a row land within `--tolerance`. They never change parameters; they print the learned
margin to pass as `--stop-margin`.

```sh
ros2 run robot_base_driver auto_calibrate tune-drive --distance 0.5 --speed 0.1      # tolerance 5 mm
ros2 run robot_base_driver auto_calibrate tune-turn --angle 90 --angular-speed 0.5   # tolerance 1 deg
ros2 run robot_base_driver auto_calibrate drive --distance 0.5 --stop-margin 0.006   # m; - = backward
ros2 run robot_base_driver auto_calibrate turn --angle 90 --stop-margin 1.9          # deg; - = right
```

These can't calibrate `wheel_radius_m` or `wheel_base_m`. The firmware uses each one both to turn
`cmd_vel` into wheel speeds and to turn ticks back into odom, so the error cancels and odom can't see
it. Do the tape-measured `straight` / `rotate` runs first; the odom-driven moves are only as accurate as
those values. Drive runs also print sideways drift and heading change, which show straight-line
tracking. The margin depends on speed, so tune at the speed you'll drive at.

---

## 0. Pre-flight check

Drive forward for 5 seconds at a slow speed and confirm the watchdog stops it cleanly (wheels spin
evenly, LED stays solid):

```sh
ros2 topic pub --rate 10 --times 50 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

50 messages at 10 Hz = 5 s at 0.1 m/s, about 0.5 m. It stops on its own once the messages are sent. If
the wheels stop and restart mid-drive, a delayed packet tripped `cmd_vel_timeout_ms`; check that
`ros2 param get /robot_base cmd_vel_timeout_ms` is `1000`.

## 1. wheel_radius_m — straight line

Automated: `auto_calibrate straight`. The manual procedure it follows:

1. Mark the robot's position on the floor with tape.
2. Drive it forward about 1 m.
3. Mark where it stopped and tape-measure the straight-line distance: `actual`.
4. `ros2 topic echo /odom --once`: `reported = sqrt(x² + y²)` (subtract the starting x/y if odom
   wasn't at the origin).
5. `new_wheel_radius_m = current_wheel_radius_m * (actual / reported)`.
6. `ros2 param set /robot_base wheel_radius_m <new_value>`.
7. Repeat once to confirm `reported` now tracks `actual`.

| Trial | actual (m) | reported (m) | old wheel_radius_m | new wheel_radius_m |
|---|---|---|---|---|
| 1 (17 Oct 2026) | 1.04 | 1.0419 | 0.022000 | 0.021961 |
| 2 (19 Oct 2026) | not recorded | not recorded | 0.021961 | 0.021933 |

Trial 2's actual/reported ratio was 0.99873 (derived from the old and new values; the raw readings were
not saved).

**Final `wheel_radius_m`: 0.021933 m**, confirmed by parameter readback and saved as the firmware default.

## 2. wheel_base_m — in-place 90° turn

A 90° turn is used rather than 360°: `theta` wraps at ±π, so a full turn returns to `theta ≈ 0`
regardless of any `wheel_base_m` error. Do this **after** part 1, since the rotation math uses per-wheel
distance.

Automated: `auto_calibrate rotate`. The manual procedure it follows:

1. Mark the robot's starting heading with a straight tape line.
2. Rotate it in place by exactly 90°, aligned with a second, perpendicular line.
3. Read the resulting `theta` from `/odom`: `yaw = 2 * atan2(z, w)` of the orientation quaternion.
4. `new_wheel_base_m = current_wheel_base_m * (reported_theta / (pi/2))`. The ratio is inverted
   compared to part 1, because the wheel base divides into the angle formula.
5. `ros2 param set /robot_base wheel_base_m <new_value>`.
6. Repeat once to confirm.

| Trial | reported theta (rad) | target (rad) | old wheel_base_m | new wheel_base_m |
|---|---|---|---|---|
| 1 (19 Oct 2026) | not recorded | 1.5708 | 0.116000 | 0.119528 |

Odom over-reported the turn by 3.04% (reported/actual = 1.03041, derived from the old and new values).

**Final `wheel_base_m`: 0.119528 m**, confirmed by parameter readback and saved as the firmware default.

## 3. Course test

`drive_test` drives full courses using `/odom` feedback and prints the odom closing error at the end.
Compare it with the real end position against tape marks on the floor:

```sh
ros2 run robot_base_driver drive_test square                # 1 m sides, back to the start
ros2 run robot_base_driver drive_test out_back              # 1 m forward/back, turn 90°, forward/back
ros2 run robot_base_driver drive_test half_circle           # radius 0.5 m
ros2 run robot_base_driver drive_test half_circle_return    # half circle, then straight back
ros2 run robot_base_driver drive_test full_circle
ros2 run robot_base_driver drive_test all
```

Options: `--side`, `--distance`, `--radius`, `--speed`, `--angular-speed`. The target is a real
closing error of at most 1.5% of the path length. If a course misses it, re-check parts 1 and 2 first:
a geometry error compounds over a multi-leg course far more than over a single straight or turn.
