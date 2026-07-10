#!/usr/bin/env bash
# Start the hardware + localization layer (NO Nav2):
#   base driver + RPLidar + EKF + static TFs
# Gives you /odom, /imu, /scan and /odometry/filtered.
#
# Usage:
#   ./start_hardware.sh
#   ./start_hardware.sh base_port:=/dev/ttyACM1 lidar_port:=/dev/ttyACM0
#
# Any extra args are forwarded to the launch file.

WS="$(cd "$(dirname "$0")" && pwd)"

# Source ROS + every installed package (works around the incomplete
# merged install/setup.bash on this box -- see ws_setup.bash).
source "$WS/ws_setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 not found. Source your ROS setup first." >&2
    exit 1
fi

echo "==> Launching hardware layer (driver + lidar + EKF). Ctrl-C to stop."
echo "==> Keep the robot STILL for ~2s while the gyro calibrates."
exec ros2 launch jetracer_bringup hardware.launch.py "$@"
