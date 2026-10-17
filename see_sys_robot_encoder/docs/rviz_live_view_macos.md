# Live robot position on macOS

RViz can show the same live `/odom` view on a Mac. On Apple Silicon, RViz in a
Linux container may fail to create an OpenGL window through XQuartz. This setup
runs RViz on a virtual display inside Docker and shows it in your browser.
XQuartz is not needed for this route.

From `see_sys_robot_encoder`, start the micro-ROS agent and the RViz view:

```sh
export ROBOT_BIND_IP="$(ipconfig getifaddr en0)"
docker compose -f docker/docker-compose.yml -f docker/docker-compose.rviz-macos-web.yml \
  up --build agent-udp ros_view
```

On Macs where Wi-Fi is not `en0`, replace `en0` with the active Wi-Fi interface.
The agent listens only on that IPv4 address; the browser view is available only
at `localhost` on this Mac. Neither view needs XQuartz.

Open [the RViz view](http://localhost:6080/vnc.html?autoconnect=1&resize=remote)
on this Mac. Enter the `RViz browser password` printed in the first terminal.
The browser connection is bound to the Mac's localhost. The red
bar on the robot model marks its front, and the arrows show recent odometry.
If the robot is connected but the model stays at the origin, check that the
robot's firmware points `MICROROS_AGENT_IP` at this Mac's Wi-Fi IP address.

If the viewer is already running, start only the stopped agent with:

```sh
export ROBOT_BIND_IP="$(ipconfig getifaddr en0)"
docker compose -f docker/docker-compose.yml -f docker/docker-compose.rviz-macos-web.yml up -d agent-udp
```

To retrieve the current browser password, run:

```sh
docker logs docker-ros_view-1 2>&1 | grep 'RViz browser password'
```

In another terminal, open a ROS shell in the running viewer container:

```sh
docker exec -it docker-ros_view-1 bash
ros2 topic echo /odom --once
```

The odometry command should print one message and exit. If it waits, the robot
is not publishing to the agent yet; use `Ctrl-C` and check its Wi-Fi connection.
With the robot on a clear floor, send a short forward command in that shell:

```sh
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.07}, angular: {z: 0.0}}'
```

This commands about two seconds of movement at 0.07 m/s (roughly 14 cm). The
robot model should move forward on the grid, leave odometry arrows, then stop.

The browser view follows the physical robot's encoder-based estimate. It does
not measure absolute position, so wheel slip can make the displayed pose drift.
Press `Ctrl-C` in the first terminal to stop the agent and view.
If you started them in the background, stop the robot-facing UDP listener when
finished (with `ROBOT_BIND_IP` still set):

```sh
docker compose -f docker/docker-compose.yml -f docker/docker-compose.rviz-macos-web.yml stop agent-udp
```

The browser view can remain running on localhost.
