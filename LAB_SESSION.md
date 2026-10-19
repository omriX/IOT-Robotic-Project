# Lab session brief — 2026-10-19

For the Claude Code session on the lab Ubuntu VM. A session on Omri's Mac
prepared the repo for handoff; this session runs the hardware steps that need the
robot. Work through the steps with the operator, one at a time. The operator
runs anything physical and confirms before the robot moves.

Delete this file once its results are recorded (step 7); the permanent handoff
doc will replace it.

## Setup facts

- Ubuntu VM on a lab PC. The VM and the robot are on the same lab network.
  The robot's USB cable is plugged into the host PC, so flashing from the VM
  needs USB passthrough to the VM (check with `ls /dev/ttyUSB*`).
- Hardware: ESP32-WROOM-32 (CP2102 USB, `10c4:ea60`), L293D motor driver,
  2S Li-ion/LiPo battery, Technion lab chassis with two encoder gear motors.
- Branch: `omrib/handoff`. Separately, `omrih/start` holds this morning's lab
  work (19 Oct calibration, `drive_test.py`). The two branches will be merged
  later by the team; don't merge them in this session.
- The robot's NVS already holds the 19 Oct calibration: `wheel_base_m = 0.119528`,
  `wheel_radius_m = 0.021933`. NVS overrides firmware defaults, so these stay
  live even when flashing this branch (whose defaults are still 0.116 / 0.021961).

## What changed today (read before acting)

| Change | Where |
|---|---|
| `see_sys_robot_encoder/` renamed to `robot/` | everything |
| Pins and measured geometry moved out of `robot_config.h` | `robot/robot_base/robot_hardware.h` |
| New optional USB-serial micro-ROS transport (**never run on hardware yet**) | `MICROROS_TRANSPORT_SERIAL`, PIO env `robot_base_serial`, compose `agent-serial` |
| New standalone hardware tests (USB Serial only) | `unit_tests/hardware/` (`pio run -d unit_tests/hardware -e <name>`) |
| New ROS driver tests (34, all pass) and end-to-end smoke test | `unit_tests/ros/`, `unit_tests/integration/smoke_test.py` |
| `ros_tools` container mounts the repo read-only at `/repo` | `robot/docker/docker-compose.yml` |
| `auto_calibrate rotate` now wraps the angle (was wrong across ±180°); new `drive`/`turn`/`tune-drive`/`tune-turn` modes | `robot/ros2_ws/src/robot_base_driver/robot_base_driver/auto_calibrate.py` |
| Ubuntu doc rewritten as a setup guide | `robot/docs/ubuntu_lab_setup.md` |
| Deleted: `ESP32/`, `flutter_app/`, `commands.txt`, `car_encoder_motors*` | — |

`unit_tests/README.md` has every test command.

## Rules

- Never commit `robot/robot_base/network_local.h` (it is gitignored) or any
  Wi-Fi credentials. Ask the operator for them; don't print them back.
- Ask before changing firmware constants or thresholds. Propose with the data.
- Live parameter changes (`ros2 param set`) are saved in the ESP32's NVS and
  override firmware defaults, even after reflashing. Note every one you make.
- Don't force-push or rewrite history.

## Steps

### 0. Prepare

```sh
git fetch && git checkout omrib/handoff && git pull
cp robot/robot_base/network_local.example.h robot/robot_base/network_local.h   # operator fills it in
```

Set `MICROROS_AGENT_IP` to the VM's IP on the robot's network (`hostname -I`).
Install PlatformIO if `pio` is missing (`pip install platformio`). Follow
`robot/docs/ubuntu_lab_setup.md` part A for Docker, firewall and USB access, then:

```sh
cd robot
COMPOSE="docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml"
$COMPOSE build ros_tools                      # picks up the new auto_calibrate
$COMPOSE run --rm ros_tools python3 -m pytest -q -p no:cacheprovider /repo/unit_tests/ros   # expect 34 passed
```

### 1. Hardware tests (wheels off the ground)

Run each from the repo root: `pio run -d unit_tests/hardware -e <name> -t upload && pio device monitor -b 115200`.
Pass criteria are in `unit_tests/README.md`. Record the outcome of each.

1. `encoders`: hand-turn each wheel one turn; expect about ±4216 ticks.
2. `battery`: operator measures the battery with a multimeter at the same
   time. Record multimeter V, pin V and battery V, and compute the factor
   `V_multimeter / pin V` (firmware uses 2.8).
3. `status_led`: operator confirms each pattern.
4. `motors`: `a`, then `b` (operator confirms A = right wheel, B = left, +PWM rolls backward).
   Then `s` for the self-test pass rate (used in step 3).

### 2. Robot firmware and smoke test

```sh
pio run -e robot_base -t upload     # from the repo root
cd robot && $COMPOSE up agent-udp   # terminal 1, wait for LED solid / state:AGENT_CONNECTED
$COMPOSE run --rm ros_tools python3 /repo/unit_tests/integration/smoke_test.py          # terminal 2
$COMPOSE run --rm ros_tools python3 /repo/unit_tests/integration/smoke_test.py --drive  # robot on the floor
```

Read back and record every parameter value (the smoke test prints them). The
expected live values are `wheel_radius_m=0.021933`, `wheel_base_m=0.119528`
(from NVS, see Setup facts), and `cmd_vel_timeout_ms=1000`. Anything else was
changed in NVS at some point; record it.

### 3. Intermittent boot self-test failure

Known: the boot self-test sometimes fails, and passes after a reset. When it
fails, the LED triple-blinks and `/cmd_vel` is refused. Cause unknown.

The self-test (`robot/robot_base/selftest.h`) drives each motor at raw PWM 60
for 200 ms each way and needs at least 40 encoder ticks. Calibration found
that motion starts around PWM 40 (`robot/docs/calibration_log_1.md` section 5),
so PWM 60 for 200 ms is close to the limit.

Collect:
- the `motors` → `s` result from step 1 (pass rate and weakest forward/backward deltas per motor)
- with `robot_base` flashed, 10 resets (EN button), recording the boot
  `self-test passed/FAILED | A fwd:.. bwd:.. | B fwd:.. bwd:..` line
  from the USB serial monitor (`pio device monitor -b 115200`). WebSerial only
  shows lines printed after the page is open, so it misses the boot line.
- the battery voltage at the time

Look for a pattern: one motor, one direction, the first phase from rest, or low
battery. Propose a fix (for example PWM, phase length or threshold) with the
numbers, and ask the operator before changing anything.

### 4. Wheel-base calibration: confirm only

Done on 19 Oct (`omrih/start`): 0.116 → 0.119528 m, from a single 90° turn.
Run one confirmation turn and record the result. Don't apply a new value unless
the error is more than about 1°, and ask the operator first.

```sh
$COMPOSE run --rm ros_tools bash
ros2 run robot_base_driver auto_calibrate rotate --angle 90 --angular-speed 0.5
```

### 5. Course acceptance test

`robot/docs/live_calibration_log.md` part 3: straight leg, in-place turn, arc,
back to start. Run 3 times; the target is ≤ 1.5 % of path length for each run.
Fill in the table. Optional: RViz (`robot/docs/rviz_live_view.md`) shows the
path while it runs.

### 6. Optional: USB-serial mode

New and never run on hardware. Flash `pio run -e robot_base_serial -t upload`,
then `$COMPOSE up agent-serial` (with no serial monitor open), and run the
smoke test. Record whether it connects. Flash `robot_base` again afterwards.

### 6b. Arduino IDE build check

The repo supports the Arduino IDE as well as PlatformIO, but the Arduino path
has not been compiled since `platformio_main.cpp` was guarded with
`#ifdef PLATFORMIO`. Ask the operator before installing anything. Then:
board package **esp32 2.0.17**, board **ESP32 Dev Module**; libraries
ESP32Encoder 0.11.8, ArduPID 0.2.1 (+ FireTimer 1.0.5), WebSerial 2.1.2,
Async TCP 3.3.2 (ESP32Async), ESP Async WebServer 3.6.0; and copy
`robot/libraries/micro_ros_arduino` into the Arduino `libraries/` folder
(see `robot/libraries/README.md`). Compile `robot/robot_base/robot_base.ino`
and `robot/robot_calibration/robot_calibration.ino`, and record the results.

### 7. Record and hand back

- Don't edit the calibration logs or the geometry constants on this branch:
  `omrih/start` changed the same lines, and the values get reconciled at merge
  time. Record calibration results in the session file below instead.
- Everything (calibration confirmation, course test table, hardware test outcomes, battery factor, self-test data and
  proposal, smoke test output, serial result, any NVS parameter changes):
  a new `robot/docs/lab_session_2026-10-19.md`.
- Run `g++ ... test_diff_drive.cpp` and the pytest suite again if code changed.
- Commit to `omrib/handoff` and push, with the operator's go-ahead.
