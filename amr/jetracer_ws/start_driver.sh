#!/usr/bin/env bash
# Start ONLY the JetRacer base driver (no navigation, no lidar, no EKF).
# Publishes /odom and /imu, subscribes /cmd_vel.
#
# Usage:
#   ./start_driver.sh                 # default port /dev/ttyACM0
#   ./start_driver.sh /dev/ttyACM1    # override serial port

WS="$(cd "$(dirname "$0")" && pwd)"

# Source ROS + every installed package (works around the incomplete
# merged install/setup.bash on this box -- see ws_setup.bash).
source "$WS/ws_setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 not found. Source your ROS setup first." >&2
    exit 1
fi

PORT="${1:-/dev/ttyACM0}"

echo "==> Starting jetracer_driver on $PORT (Ctrl-C to stop)"
echo "==> Keep the robot STILL for ~2s while the gyro calibrates."
exec ros2 run jetracer_driver jetracer_driver --ros-args -p port:="$PORT"
