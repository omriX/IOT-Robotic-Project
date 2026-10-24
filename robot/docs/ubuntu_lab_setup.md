# Installation and setup (Ubuntu)

Run the robot from an Ubuntu PC with Docker Engine (native Linux, not WSL2).
For macOS and Windows, see [Other operating systems](#other-operating-systems).
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

### A0. Install the tools

The commands below are for Ubuntu 22.04 or newer. You need Git, Docker Engine with the Compose plugin, and a tool to flash
the ESP32 (PlatformIO, or the Arduino IDE in C2).

**Git:**

```sh
sudo apt-get update
sudo apt-get install -y git
```

**Docker Engine and Compose**, from Docker's apt repository (the Ubuntu `docker.io` package may ship
a Compose version older than the v2.24.4 this setup needs). Full instructions:
[docs.docker.com/engine/install/ubuntu](https://docs.docker.com/engine/install/ubuntu/).

```sh
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**PlatformIO Core** (command-line `pio`), through pipx:

```sh
sudo apt-get install -y pipx
pipx install platformio
pipx ensurepath
```

Open a new terminal afterwards and check with `pio --version`. Instead of the command line, you can
also install the **PlatformIO IDE** extension in Visual Studio Code. PlatformIO downloads the ESP32
toolchain and the firmware libraries on the first build, so the first build needs internet and
takes a few minutes.

ROS 2 itself does not need to be installed: it runs inside the Docker images.

### A1. Docker without sudo

```sh
docker run --rm hello-world
docker compose version     # must be v2.24.4 or newer (the override uses !reset)
```

If you get `permission denied`: `sudo usermod -aG docker $USER`, log out and back in.
If Compose is older, update the `docker-compose-plugin` package from Docker's apt repo.

### A2. Get the repository

```sh
git clone https://github.com/omriX/IOT-Robotic-Project.git
cd IOT-Robotic-Project
```

If it is already there, run `git pull` inside it instead.

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

Connect the robot by USB, then flash using either PlatformIO or Arduino IDE.

### C1. PlatformIO

Flash from the **repository root**:

```sh
pio run -e robot_base -t upload          # Wi-Fi
pio run -e robot_base_serial -t upload   # USB serial
```

### C2. Arduino IDE

1. Install the **Arduino-ESP32 2.0.17** board package and select
   **ESP32 Dev Module** as the board.
2. Install the library versions listed in the [repository README](../../README.md#arduinoesp32-libraries-used-in-this-project).
   Copy `robot/libraries/micro_ros_arduino` into the `libraries` directory of
   your Arduino sketchbook (see the IDE's sketchbook location in Preferences).
   Use this repository's custom build; the stock Library Manager version does
   not support this firmware's odometry and parameter server. See the
   [custom library notes](../libraries/README.md).
3. Open `robot_base/robot_base.ino` from `robot/`. Use the same
   `robot_base/network_local.h` settings configured above. For USB serial mode,
   uncomment `#define MICROROS_TRANSPORT_SERIAL 1` in that file; for Wi-Fi mode,
   leave it commented out.
4. Select the robot's USB port under **Tools > Port**, set **Upload Speed** to
   **115200**, and click **Upload**.

---

## D. Start the agent (terminal 1)

Wi-Fi:

```sh
$COMPOSE up agent-udp
```

Serial (close any `pio device monitor` or Arduino IDE Serial Monitor first:
only one program can hold the port):

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

To view the robot's live position and odometry in RViz, follow the supplementary
[RViz live view instructions for Ubuntu](rviz_live_view.md). That guide uses
the Wi-Fi agent (`agent-udp`), so flash the Wi-Fi firmware option in C first.

## G. Stop

`Ctrl-C` in every terminal, then:

```sh
$COMPOSE down
```

---

## Other operating systems

- **Flashing** works the same from Windows and macOS: install Git, Python 3, and PlatformIO
  (`pip install platformio`, or the VS Code extension), then run the `pio` commands in C1. On Windows
  the robot shows up as a `COM` port (CP210x driver); add `--upload-port COMx` if PlatformIO picks the
  wrong one. Keep the repository in a path without non-English characters on Windows: the ESP32
  compiler cannot open headers under such paths, which breaks the hardware-test builds.
- **macOS**: install [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/).
  Wi-Fi mode works without the Linux override; set `ROBOT_BIND_IP` to the Mac's Wi-Fi address as shown
  in [rviz_live_view_macos.md](rviz_live_view_macos.md). Serial mode is not available, since Docker
  Desktop cannot pass USB devices into containers.
- **Windows**: run the steps from D onward on an Ubuntu PC or in an Ubuntu virtual machine with a
  bridged network adapter, so the robot can reach it. Docker Desktop on Windows is not covered here.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| LED keeps slow-blinking (Wi-Fi) | `MICROROS_AGENT_IP` matches `hostname -I`; same 2.4 GHz network; `ping <robot ip>`; firewall (A3). |
| LED keeps slow-blinking (serial) | Flashed `robot_base_serial`? Port free (no monitor open)? Right `/dev/ttyUSB*`? ModemManager rule (A4)? |
| No WebSerial page | The robot couldn't join Wi-Fi: check SSID/password and that the network is 2.4 GHz. |
| Wheels stop and restart during a drive | A delayed `/cmd_vel` packet tripped the stop timeout. `ros2 param get /robot_base cmd_vel_timeout_ms` should be `1000`. At 500 ms this happened regularly over Wi-Fi. |
| LED triple-blinks, robot ignores `/cmd_vel` | The boot self-test failed. Reset the robot; WebSerial shows the per-motor encoder deltas. |
