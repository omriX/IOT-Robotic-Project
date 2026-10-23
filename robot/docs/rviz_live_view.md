# Live robot position in RViz (Ubuntu desktop)

This view follows the physical robot's encoder-based `/odom` estimate. It shows
the robot model, a 25 cm grid, and recent odometry arrows from directly above.
The red bar marks the robot's front. No map or external position sensor is
present, so wheel slip can make the picture drift from the real floor position.

Run these commands in **Ubuntu desktop terminals**, from `robot`.
The ESP32 and Ubuntu PC must already be on the same network, with the PC's IP
set as `MICROROS_AGENT_IP` in the robot's local firmware settings.

## 1. Start the micro-ROS agent

```sh
docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml up agent-udp
```

Wait for `state:AGENT_CONNECTED` on the robot. Leave this terminal running.

## 2. Allow the RViz window to use Ubuntu's X display

In a second terminal, check `echo "$DISPLAY"`. It should contain a display such
as `:0`. The following picks the X authority file used by the desktop session:

```sh
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
test -f "$XAUTHORITY" && echo "X authority: $XAUTHORITY"
```

If the file check fails, run `xauth info` and set `XAUTHORITY` to the path on
its `Authority file` line. Log into the graphical desktop as the same user
running Docker; a text-only SSH session does not provide a local display.

## 3. Launch the ROS driver and RViz

```sh
docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml \
  -f docker/docker-compose.rviz.yml run --rm --build ros_tools \
  bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch robot_base_driver bringup.launch.py rviz:=true'
```

The first build may take several minutes. The launch starts the driver that
broadcasts `odom -> base_link`, the robot model publisher, and RViz. It does not
drive the robot. In RViz, keep **Fixed Frame** set to `odom`. If the robot moves
out of view, use the Views panel to pan or zoom.

## 4. Drive while viewing

In a third terminal, open the ROS tools shell:

```sh
docker compose -f docker/docker-compose.yml -f docker/docker-compose.linux.yml \
  run --rm ros_tools bash
```

Then send the same short `/cmd_vel` commands used in the drive tests. Leave
clear floor space; the view updates from the actual robot's odometry. When
the robot stops, its image should stop too. Close the ROS command terminal
with `Ctrl-C` as usual, and close the RViz launch with `Ctrl-C` when done.

If the window cannot connect to the X display, check that `DISPLAY` and
`XAUTHORITY` refer to the active Ubuntu desktop session and that Xwayland is
running on a Wayland desktop. This Compose override is for Ubuntu; it does
not provide a Mac desktop display.
