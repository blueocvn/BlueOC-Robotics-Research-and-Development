#!/usr/bin/env bash
# Start the autonomous MAPPING stack (assumes the hardware layer is already
# running AND a SLAM stack on another machine is publishing /map + map->odom TF).
#
# Runs the Nav2 motion stack (no map_server/amcl) + explore_lite frontier
# exploration. Do NOT run start_nav2.sh at the same time -- they both bring up
# the Nav2 motion nodes.
#
# Usage:
#   ./start_mapping.sh

WS="$(cd "$(dirname "$0")" && pwd)"

# Source ROS + every installed package (see ws_setup.bash).
source "$WS/ws_setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 not found. Source your ROS setup first." >&2
    exit 1
fi

echo "==> Launching MAPPING stack (Nav2 motion + explore_lite). Ctrl-C to stop."
exec ros2 launch jetracer_bringup mapping_bringup.launch.py "$@"
