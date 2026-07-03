#!/usr/bin/env bash
# Launch the Robot Web Bridge web UI/API inside the humble_ws container.
#
# Assumes you are already inside the container (or run it via run_workstation.sh).
# The container uses --network host, so port 8088 is on the host directly:
#   ngrok http 8088     # on the host, to expose it for QR scanning
#
# For local dev outside ROS you can also just:
#   pip install fastapi uvicorn jinja2 pyyaml
#   uvicorn robot_web_bridge.app:app --reload --port 8088
set -euo pipefail

# Share the same DDS domain as the rest of the robot (gitignored real values).
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$REPO_ROOT/network.env" ] && source "$REPO_ROOT/network.env"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash
# install/ exists after `colcon build`; ignore if running from source
source "$(dirname "$0")/install/setup.bash" 2>/dev/null || true

export ROBOT_WEB_BRIDGE_PORT="${ROBOT_WEB_BRIDGE_PORT:-8088}"
exec ros2 run robot_web_bridge server
