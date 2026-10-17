#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u
umask 077

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR=/tmp/robot-rviz-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

Xvfb "$DISPLAY" -screen 0 1280x800x24 -ac +extension GLX +render -noreset &
xvfb_pid=$!

cleanup() {
  kill "$xvfb_pid" "${vnc_pid:-}" "${web_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT

for attempt in {1..50}; do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
xdpyinfo -display "$DISPLAY" >/dev/null

vnc_password="$(python3 -c 'import secrets, string; alphabet = string.ascii_letters + string.digits; print("".join(secrets.choice(alphabet) for _ in range(8)))')"
password_file="$XDG_RUNTIME_DIR/vnc.pass"
x11vnc -storepasswd "$vnc_password" "$password_file" >/dev/null 2>&1
echo "RViz browser password: $vnc_password"
unset vnc_password

x11vnc -display "$DISPLAY" -rfbauth "$password_file" -listen 127.0.0.1 -forever -shared -rfbport 5900 -quiet &
vnc_pid=$!
websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900 &
web_pid=$!

echo 'RViz browser view: http://localhost:6080/vnc.html?autoconnect=1&resize=remote'
ros2 launch robot_base_driver bringup.launch.py rviz:=true
