# Live calibration log — wheel_radius_m / wheel_base_m via ros2 param set

Refines the two live-tunable geometry parameters (`rclc_parameter_server`, no reflash) against the
integrated `/odom` output, rather than raw encoder ticks (that was `calibration_log.md`, done earlier
and pre-ROS). Terminal 1: `docker compose up agent-udp`. Terminal 2: `docker compose run --rm ros_tools
bash`, confirm the board shows `AGENT_CONNECTED` on serial before starting.

## Running `auto_calibrate` (Parts 1 and 2)

Automates the "command it, then read odom" half of Parts 1 and 2 below — you still measure the actual
distance/angle with a tape and confirm before it applies anything. Same two terminals as everywhere else:

Terminal 1:
```
docker compose -f docker/docker-compose.yml up agent-udp
```
Confirm `AGENT_CONNECTED` on serial first.

Terminal 2:
```
docker compose -f docker/docker-compose.yml run --rm ros_tools bash
```
then, inside that shell:
```
# wheel_radius_m -- drives 1 m forward at 0.1 m/s (defaults)
ros2 run robot_base_driver auto_calibrate straight
ros2 run robot_base_driver auto_calibrate straight --distance 1.0 --speed 0.1

# wheel_base_m -- rotates 90 deg at 0.5 rad/s (defaults)
ros2 run robot_base_driver auto_calibrate rotate
ros2 run robot_base_driver auto_calibrate rotate --angle 90 --angular-speed 0.5
```

It asks `ready? [y/N]` first — give the robot clear floor space and wheels on the ground before answering
`y`. It drives the segment, prints the odom-reported distance/angle, prompts you to type in the
tape-measured actual value, prints the corrected constant, and asks before applying it live via
`ros2 param set`. `Ctrl-C` stops the robot immediately at any point.

`ros2_ws/src` is baked into the `ros_tools` image at build time, not volume-mounted — after editing
`auto_calibrate.py` yourself, rebuild before the change takes effect:
```
docker compose -f docker/docker-compose.yml build ros_tools
```

---

## Session info

- **Date:**
- **Location / surface:**
- **Operator:**
- **Starting `wheel_radius_m` / `wheel_base_m`:** (from `ros2 param get /robot_base wheel_radius_m` /
  `wheel_base_m`)

---

## 0. Pre-flight sanity check

Drive forward for exactly 5 seconds at a slow, safe speed, then confirm the watchdog stops it cleanly and
nothing looks wrong (wheels spin evenly, no crash on serial, LED still solid/AGENT_CONNECTED):

```
ros2 topic pub --rate 10 --times 50 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

`--rate 10 --times 50` = 50 messages at 10 Hz = 5.0 s of continuous forward motion at 0.1 m/s (well under
`max_linear_mps`'s 0.2 default). It stops on its own once the 50 messages are sent — no `Ctrl-C` needed,
but have it ready anyway. Give it clear floor space (~0.5 m) in front of it first. If the spin ever
stutters (stops and restarts) mid-drive instead of running smoothly, that's `cmd_vel_timeout_ms` tripping
on a delayed packet, not a control-loop bug — check `ros2 param get /robot_base cmd_vel_timeout_ms` is
still `1000` (found and fixed this exact issue once already; see `robot_config.h`'s comment on it).

- **Result (OK / not OK, notes):**

---

## 1. wheel_radius_m — straight-line 1 m

**Automated option:** `ros2 run robot_base_driver auto_calibrate straight --distance 1.0 --speed 0.1` drives
the segment and reads `/odom` for you — it still prompts you for the tape-measured `actual` distance and
asks before applying `ros2 param set`. Steps 2-7 below are what it's doing under the hood; use them if you
want to do it manually or the script doesn't behave as expected.

**Procedure:**
1. Mark the robot's current position on the floor with tape.
2. Drive it forward ~1 m as straight as possible (teleop, or push it by hand in freewheel mode).
3. Mark where it stopped; measure the real straight-line distance with a tape measure — `actual`.
4. `ros2 topic echo /odom --once`, compute `reported = sqrt(x² + y²)` (subtract your starting x/y if the
   board wasn't freshly booted).
5. `new_wheel_radius_m = current_wheel_radius_m * (actual / reported)`.
6. `ros2 param set /robot_base wheel_radius_m <new_value>`.
7. Repeat once to confirm `reported` now tracks `actual` closely.

| Trial | actual (m) | reported (m) | current wheel_radius_m | new wheel_radius_m |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 (confirm) |  |  |  |  |

- **WHEEL_RADIUS_M (final, live value):**

---

## 2. wheel_base_m — in-place 90° turn

A 90° turn is used here rather than a full 360° — `theta` wraps at ±π, so a full 360° turn returns to
`theta ≈ 0` regardless of any wheel_base_m error, which hides exactly the thing being measured. 90° gives
an unambiguous `theta ≈ π/2` to compare against, and two perpendicular tape lines (or a wall corner) are
an easy, precise physical reference. Do this **after** Part 1 — the rotation math uses per-wheel distance,
so a wrong `wheel_radius_m` throws this off too.

**Automated option:** `ros2 run robot_base_driver auto_calibrate rotate --angle 90 --angular-speed 0.5`
commands the turn and reads `/odom` for you, then prompts for the actual angle (measured against your tape
lines) before computing and offering to apply the new value.

**Procedure:**
1. Mark the robot's starting heading with a straight tape line.
2. Rotate it in place until turned exactly 90° — align with a second, perpendicular line.
3. Read the resulting `theta`: the Serial/WebSerial `odom x:... y:... theta:...` line (already radians),
   or from `/odom`'s quaternion via `yaw = 2 * atan2(z, w)`.
4. `new_wheel_base_m = current_wheel_base_m * (reported_theta / (pi/2))` — note the ratio is inverted
   relative to Part 1, since wheel_base_m divides into the angle formula rather than scaling it directly.
5. `ros2 param set /robot_base wheel_base_m <new_value>`.
6. Repeat once to confirm.

| Trial | reported theta (rad) | target (π/2 ≈ 1.5708) | current wheel_base_m | new wheel_base_m |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 (confirm) |  |  |  |  |

- **WHEEL_BASE_M (final, live value):**

---

## 3. Closed-loop course test

The acceptance test: a course with a straight leg, an in-place rotation, and an arc, returning to the
start. Do this **after** Parts 1 and 2 — it validates the refined constants, not raw ones. Target: final
position error **≤ 1.5% of total path length**. Run it 3× — a single good run on carpet is luck, not
calibration.

**Procedure, per run:**
1. Mark the start pose on the floor with tape (position + a heading line).
2. Drive the course: straight leg → in-place rotation → arc, back to roughly the start.
3. Mark where it actually stopped; measure the real gap between start and end pose with a tape measure —
   `actual_error_m`.
4. Note the total path length driven (sum of the leg lengths you drove) — `path_length_m`.
5. `ros2 topic echo /odom --once` for the final reported pose; compute `reported_error_m` the same way
   (distance from the odom origin (0,0) to the final x/y, since the course returns to the start).
6. `error_pct = actual_error_m / path_length_m * 100`.

| Run | path_length (m) | actual_error (m) | reported x,y,theta | error_pct |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |

- **All 3 runs ≤ 1.5%?**

If not, re-check Parts 1/2 (a `wheel_radius_m`/`wheel_base_m` error compounds over a multi-leg course far
more than over a single straight/turn test) before assuming something else is wrong.

---

## 4. Wrap-up — record the final constants

Once Part 3 passes:
1. Write the final `wheel_radius_m`/`wheel_base_m` (currently live-only, NVS-persisted) into
   `robot_base/robot_config.h`'s `WHEEL_RADIUS_M`/`WHEEL_BASE_M` defaults, so a fresh flash or NVS wipe
   doesn't regress to the old values.
2. Add a final row to `calibration_log.md`'s summary table with these values and the date.
3. Revisit `/odom`'s covariance diagonal (`initOdomMsg()` in `robot_base.ino`) against the error_pct
   actually observed in Part 3 — the current `[0.01, 0.01, 0.05]` (x, y, yaw) values were a starting guess,
   not measured.

- **Final WHEEL_RADIUS_M:**
- **Final WHEEL_BASE_M:**
