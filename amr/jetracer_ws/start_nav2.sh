#!/usr/bin/env bash
# Start the Nav2 stack only (assumes the hardware layer is already running).
#
# Usage:
#   ./start_nav2.sh
#   ./start_nav2.sh map:=/ros2_ws/maps/my_map.yaml

WS="$(cd "$(dirname "$0")" && pwd)"

# Source ROS + every installed package (see ws_setup.bash).
source "$WS/ws_setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 not found. Source your ROS setup first." >&2
    exit 1
fi

echo "==> Launching Nav2 stack. Ctrl-C to stop."
exec ros2 launch jetracer_bringup nav_bringup.launch.py "$@"
