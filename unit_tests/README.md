# Tests

| Folder | What it tests | Needs |
|---|---|---|
| [hardware/](hardware/) | One component at a time: motors, encoders, battery divider, status LED | Robot over USB |
| [ros/](ros/) | `robot_base_driver`: diagnostics, stale-odom watchdog, TF, calibration math | Docker only |
| [integration/](integration/) | The whole chain: robot → agent → ROS 2 topics and parameters | Robot connected to the agent |
| [../robot/test/](../robot/test/) | Firmware drive math (`diff_drive.h`): kinematics, odometry, straight correction | A C++ compiler |

Commands below run from the repository root unless stated otherwise.

## Firmware drive math (PC)

```sh
g++ -std=c++17 -Wall -Wextra -I robot/robot_base \
  robot/test/test_diff_drive.cpp -o /tmp/drive-tests && /tmp/drive-tests
```

## Hardware tests

Each test is a separate PlatformIO environment that replaces the robot firmware
until you flash it back. They use USB Serial only, so no Wi-Fi settings are needed.
Pins and constants come from `robot/robot_base/robot_hardware.h`,
the same header the firmware uses.

```sh
pio run -d unit_tests/hardware -e motors -t upload   # or encoders, battery, status_led
pio device monitor -b 115200
```

| Test | What to do | Pass when |
|---|---|---|
| `motors` | Lift the robot. Send `a`, then `b`. `s` repeats the boot self-test 20× per motor. | A turns the **right** wheel, B the **left**; raw +PWM rolls the wheel **backward**; the sketch prints PASS. Self-test 20/20. |
| `encoders` | Turn each wheel one full turn by hand. `z` zeroes. | About ±4216 ticks per turn, on the encoder of the wheel you turned (A = right, B = left); rolling forward makes the counts go down. |
| `battery` | Measure the battery with a multimeter at the same time. | `V_multimeter / pin V` is about 2.8. If not, update `BATTERY_VOLTAGE_DIVIDER_FACTOR`. |
| `status_led` | Watch the LED. | Each pattern matches the name printed on Serial. |

Then flash the robot firmware again: `pio run -e robot_base -t upload`.

## ROS driver tests

From `robot/`:

```sh
docker compose -f docker/docker-compose.yml run --rm ros_tools \
  python3 -m pytest -q -p no:cacheprovider /repo/unit_tests/ros
```

The repository is mounted read-only at `/repo`, so the tests use your current
source without rebuilding the image.

## Integration smoke test

Start the agent and wait for `state:AGENT_CONNECTED` (see the quick start), then
from `robot/`:

```sh
docker compose -f docker/docker-compose.yml run --rm ros_tools \
  python3 /repo/unit_tests/integration/smoke_test.py          # add --drive for a 14 cm drive
```

On Ubuntu add `-f docker/docker-compose.linux.yml` after the first `-f`.

Checks `/odom` rate, battery voltage, boot self-test result, and that all seven
parameters are readable. `--drive` asks first, then drives 14 cm forward and
checks distance, heading drift, and that the robot stops. Exits 1 on any FAIL.
