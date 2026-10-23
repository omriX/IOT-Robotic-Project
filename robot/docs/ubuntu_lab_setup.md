# Ubuntu lab PC setup

Run the robot from an Ubuntu PC with Docker Engine (native Linux, not WSL2).
The robot talks to the micro-ROS agent either over **Wi-Fi** (default) or over
the **USB cable** (serial). Serial needs Linux: Docker Desktop on macOS cannot
pass USB devices into containers.

All `docker compose` commands run from `robot/`, and use the
Linux override (host networking, `/logs` mount):

```sh
COMPOSE="docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml"
```

---

## A. Once per PC

### A1. Docker without sudo

```sh
docker run --rm hello-world
docker compose version     # must be v2.24.4 or newer (the override uses !reset)
```

If you get `permission denied`: `sudo usermod -aG docker $USER`, log out and back in.
If Compose is older, update the `docker-compose-plugin` package from Docker's apt repo.

### A2. Get the repository

`git clone` it, or `git pull` if it is already there.

### A3. Firewall (Wi-Fi only)

```sh
sudo ufw status
sudo ufw allow 8888/udp    # only if the status is "active"
```

### A4. USB access (serial, and for flashing)

```sh
groups | grep -q dialout || sudo usermod -aG dialout $USER   # then log out and back in
```

Stop ModemManager from probing the board's CP210x USB chip (VID:PID `10c4:ea60`),
which can disturb the serial link:

```sh
echo 'ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-robot-cp210x.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the robot afterwards.

---

## B. Each session: network

The ESP32 radio is **2.4 GHz only**. Use a 2.4 GHz lab network, or a phone
hotspot (iPhone: Settings > Personal Hotspot > **Maximize Compatibility** on).
Connect the PC to the same network and read its IP:

```sh
hostname -I
```

This is `PC_IP`. Hotspots can hand out a new IP each session, so check it every time.
Serial mode still uses Wi-Fi, but only for the WebSerial debug page.

---

## C. Configure and flash the robot

Do this once, and again whenever the network or `PC_IP` changes.

```sh
cp robot_base/network_local.example.h robot_base/network_local.h
```

Edit `robot_base/network_local.h` (Git ignores it, so credentials stay local):

| Setting | Value |
|---|---|
| `WIFI_SSID`, `WIFI_PASSWORD` | the 2.4 GHz network from B |
| `MICROROS_AGENT_IP` | `PC_IP` (Wi-Fi mode only) |

Flash from the **repository root** with the robot on USB:

```sh
pio run -e robot_base -t upload          # Wi-Fi
pio run -e robot_base_serial -t upload   # USB serial
```

---

## D. Start the agent (terminal 1)

Wi-Fi:

```sh
$COMPOSE up agent-udp
```

Serial (close any `pio device monitor` first: only one program can hold the port):

```sh
ls -l /dev/ttyUSB*                                         # expect /dev/ttyUSB0
$COMPOSE up agent-serial
MICROROS_SERIAL_DEV=/dev/ttyUSB1 $COMPOSE up agent-serial  # if the device differs
```

Leave it running. When the robot connects, the agent logs a new client session.

## E. Confirm the robot is connected

- The status LED goes **solid** (slow blink = still waiting for the agent).
- WebSerial: the robot prints `wifi ip: <ip>` at boot (on the USB serial monitor
  in Wi-Fi mode). Open `http://<robot ip>/webserial` and wait for `state:AGENT_CONNECTED`.

## F. Smoke test (terminal 2)

```sh
$COMPOSE run --rm ros_tools \
  python3 /repo/unit_tests/integration/smoke_test.py          # add --drive for a 14 cm drive
```

Or work by hand in a tools shell:

```sh
$COMPOSE run --rm ros_tools bash
ros2 topic list                       # /odom, /battery_state, /cmd_vel, /robot_base/log, ...
ros2 topic hz /odom                   # about 20 Hz
ros2 topic echo /battery_state --once
ros2 param get /robot_base cmd_vel_timeout_ms
```

To save the robot's ROS log to `<repo>/robot/logs/` on the PC:

```sh
ros2 topic echo /robot_base/log > /logs/session.txt
```

## G. Stop

`Ctrl-C` in every terminal, then:

```sh
$COMPOSE down
```

---

## Troubleshooting

| Symptom | Check |
|---|---|
| LED keeps slow-blinking (Wi-Fi) | `MICROROS_AGENT_IP` matches `hostname -I`; same 2.4 GHz network; `ping <robot ip>`; firewall (A3). |
| LED keeps slow-blinking (serial) | Flashed `robot_base_serial`? Port free (no monitor open)? Right `/dev/ttyUSB*`? ModemManager rule (A4)? |
| No WebSerial page | The robot couldn't join Wi-Fi: check SSID/password and that the network is 2.4 GHz. |
| Wheels stop and restart during a drive | A delayed `/cmd_vel` packet tripped the stop timeout. `ros2 param get /robot_base cmd_vel_timeout_ms` should be `1000`. At 500 ms this happened regularly over Wi-Fi. |
| LED triple-blinks, robot ignores `/cmd_vel` | The boot self-test failed. Reset the robot; WebSerial shows the per-motor encoder deltas. |
