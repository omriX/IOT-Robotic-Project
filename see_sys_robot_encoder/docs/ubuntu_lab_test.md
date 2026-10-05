# Ubuntu lab test — run the robot and A/B the delayed-packet issue

Goal: run the robot from the Ubuntu lab PC (Docker Engine, native Linux, no WSL2) over an iPhone hotspot, then
reproduce the `cmd_vel` gap problem with the same protocol used on Windows, and compare.

Compose command used everywhere below (the `-f` override adds host networking and a `/logs` mount):

```
COMPOSE="docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml"
```

Run from the repo root.

---

## Part A — Setup (once per lab session)

### A1. Check Docker works without sudo

```
docker run --rm hello-world
docker compose version
```

- If you get `permission denied`: `sudo usermod -aG docker $USER`, then log out and back in, and re-run.
- `docker compose version` must be **v2.24.4 or newer** (the override uses `!reset`). If older, update the
  `docker-compose-plugin` package from the Docker apt repo.

### A2. Get the repo onto the lab PC

Copy the `see_sys_robot_encoder` folder over (USB stick or cloud), or `git pull` if it is already a clone.

### A3. Turn on the iPhone hotspot (2.4 GHz)

The ESP32 radio is 2.4 GHz only.

- iPhone: Settings > Personal Hotspot > turn on **Maximize Compatibility**.
- Connect the Ubuntu PC to the hotspot from the Wi-Fi menu.

Read the PC's IP (expect `172.20.10.x` on an iPhone):

```
hostname -I
```

Write it down as `PC_IP`. You will re-check this every session, because the hotspot can hand out a new IP.

### A4. Serial only: dialout group

Skip this section if you are using WiFi/UDP only.

```
groups | grep -q dialout || sudo usermod -aG dialout $USER
```

Log out and back in (or run `newgrp dialout` in the terminal you will use).

### A5. ModemManager check and fix (serial only, recommended)

Ubuntu's ModemManager probes new USB serial devices and can disturb the CP210x link.

```
systemctl is-active ModemManager
```

Permanent fix for this board (VID:PID `10c4:ea60`):

```
echo 'ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-robot-cp210x.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug and replug the robot's USB cable.

Quick alternative for one session only: `sudo systemctl stop ModemManager`.

### A6. Firewall check

```
sudo ufw status
```

If it says `Status: active`, allow the agent port:

```
sudo ufw allow 8888/udp
```

If it says `inactive`, nothing to do.

### A7. Set the robot's network config and flash

Edit `robot_base/robot_config.h`:

| Constant | Value |
|---|---|
| `WIFI_SSID` | the iPhone's name, e.g. `"Omri's iPhone"` (the name shown in Settings > General > About) |
| `WIFI_PASSWORD` | the hotspot password (Settings > Personal Hotspot > Wi-Fi Password) |
| `MICROROS_AGENT_IP` | `PC_IP` from A3 |
| `MICROROS_TRANSPORT_SERIAL` | `0` for WiFi/UDP, `1` for USB serial |

Do **not** commit the hotspot password to git.

Flash `robot_base/robot_base.ino` from Arduino IDE. Re-flash whenever `MICROROS_TRANSPORT_SERIAL` or
`MICROROS_AGENT_IP` changes.

---

## Part B — Start the agent and check the link

### B1. Terminal 1: start the agent (WiFi/UDP)

```
$COMPOSE up agent-udp
```

Leave it running. Once the robot connects you will see the agent create and then keep a client session.

Check the port is listening (in a third terminal):

```
sudo ss -ulpn | grep 8888
```

### B2. Confirm the robot is connected

Find the robot's IP: it prints `wifi ip: <ip>` at boot. Open the WebSerial page in a browser:

```
http://<robot ip>/webserial
```

Wait for `state:AGENT_CONNECTED`.

If it never connects, check reachability from the PC:

```
ping -c 4 <robot ip>
```

### B3. Terminal 2: open the tools shell

```
$COMPOSE run --rm ros_tools bash
```

### B4. Smoke test (inside the tools shell)

```
ros2 topic list                                  # expect /odom, /battery_state, /cmd_vel, /robot_base/log, /diagnostics
ros2 topic hz /odom                              # expect about 20 Hz; Ctrl-C to stop
ros2 topic echo /battery_state --once
ros2 param get /robot_base cmd_vel_timeout_ms    # current value (default 1000)
ros2 pkg executables robot_base_driver           # expect auto_calibrate and driver_node
```

Robot in the air for this section.

### B-alt. USB serial instead of WiFi (optional, isolates WiFi from the agent)

1. Set `MICROROS_TRANSPORT_SERIAL 1` in `robot_config.h`, flash.
2. Apply A4 and A5, plug the USB cable into the lab PC.
3. Check the device: `ls -l /dev/ttyUSB*` (expect `/dev/ttyUSB0`).
4. Start the agent: `$COMPOSE up agent-serial`
   (if the device is not `/dev/ttyUSB0`: `MICROROS_SERIAL_DEV=/dev/ttyUSB1 $COMPOSE up agent-serial`).
5. Run B4 as usual.

---

## Part C — Burst test (the A/B)

Same protocol as the Windows runs. Robot in the air, wheels free. Keep the USB cable connected for debugging if
you use WiFi (WebSerial still works over WiFi).

### C0. Terminal 3: capture the ROS log to the host

```
$COMPOSE run --rm ros_tools bash
```

Inside it, start the log capture for this run (stop with Ctrl-C at the end of the run):

```
mkdir -p /logs
ros2 topic echo /robot_base/log > /logs/burst_500_run1.txt
```

Use a new filename for each run (`burst_500_run2.txt`, ...). Files appear in `<repo>/logs/` on the host.

Note: `cmd_vel watchdog: no command, stopping` and `agent disconnected` are logged to ROS. `odom publish failed`
is only on WebSerial, so copy that count from the browser page (see C4).

### C1. Set the timeout to 500 ms (reproduces the original sensitivity)

In Terminal 2 (tools shell):

```
ros2 param set /robot_base cmd_vel_timeout_ms 500
ros2 param get /robot_base cmd_vel_timeout_ms    # must print 500
```

### C2. Run 10 bursts

For each burst, in the tools shell:

```
ros2 topic pub --rate 10 --times 50 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.07}}'
```

Wait about 5 s between bursts. Use the same speed (0.07 m/s) for every run.

### C3. Count the events

After the 10 bursts, count in the log file (Terminal 3 or the host):

```
grep -c "cmd_vel watchdog" logs/burst_500_run1.txt
grep -c "agent disconnected" logs/burst_500_run1.txt
```

Record both numbers in the table in C5.

### C4. Count odom publish failures (WebSerial)

Open `http://<robot ip>/webserial`, select all the page text for the run, paste it into a text file on the
PC, then:

```
grep -c "odom publish failed" <that file>
```

### C5. Results — 500 ms

| Run | Watchdog trips (mid-burst) | Agent disconnects | `odom publish failed` |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |
| **Total** | | | |

### C6. Control — 1000 ms

Stop the capture (Ctrl-C), then:

```
ros2 param set /robot_base cmd_vel_timeout_ms 1000
ros2 param get /robot_base cmd_vel_timeout_ms    # must print 1000
```

Repeat C0 (new filename `burst_1000_run1.txt`), then C2 (10 bursts), C3 and C4. Fill in the same table.

Expected if the gaps are the cause: 0 watchdog trips at 1000 ms.

### C7. Put the Windows numbers next to these

Use the existing Windows home-network results (same protocol, same timeout).

---

## Part D — Reading the results

| What you see on Ubuntu (500 ms) | Meaning |
|---|---|
| 0 watchdog trips, Windows showed several | The Windows / Docker Desktop path is the dominant cause. |
| Trips on both OSes at 500 ms | Not Windows-specific. Likely ESP32 WiFi modem sleep or the hotspot link. Next: firmware variant with `WiFi.setSleep(false)` in `setup()`, repeat C2-C5. |
| 0 trips on both | The issue was the home network + Docker Desktop combination. |
| Trips at 1000 ms as well | The gaps are longer than 1 s. Look at the agent logs (`$COMPOSE logs agent-udp`) and the hotspot signal. |

Caveat: Ubuntu runs on the iPhone hotspot and the Windows baseline ran on home WiFi. A difference can be the network,
the OS, or both. Repeating C2-C5 on Windows over the same hotspot removes that confound.

---

## Stop everything

```
# Terminal 1 and any tools shells: Ctrl-C
docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml down
```

Before you leave: `ros2 param set /robot_base cmd_vel_timeout_ms 1000` if you want the robot back to its default.
