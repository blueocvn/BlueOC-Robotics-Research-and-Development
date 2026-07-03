#!/usr/bin/env bash
#
# run_orchestrator.sh
# Launch the LEAN orchestrator-only Docker container (../Dockerfile.orchestrator)
# -- just orchestrator_ws, no SLAM/Nav2/Isaac Sim, no GUI. For running/deploying
# robot_web_bridge standalone, pointed at a robot (real or simulated) elsewhere
# on the same DDS domain. For local dev alongside jetracer_ws, use
# jetracer_ws/run_workstation.sh + docker exec instead (see repo README).
#
# Usage:
#   ./run_orchestrator.sh                # interactive shell inside the container
#   ./run_orchestrator.sh <command>       # run a command then exit
#

set -euo pipefail

# ---------------------------------------------------------------------------
# Config — real network values live in <repo>/network.env (gitignored).
# Copy network.env.example to network.env and fill it in.
# ---------------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ -f "$REPO_ROOT/network.env" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/network.env"
else
    echo "WARNING: $REPO_ROOT/network.env not found — falling back to placeholders."
    echo "         Copy network.env.example to network.env and set your IPs."
fi

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
WORKSTATION_IP="${WORKSTATION_IP:-127.0.0.1}"
JETRACER_IP="${JETRACER_IP:-127.0.0.1}"
DDS_INTERFACE="${DDS_INTERFACE:-eth0}"
IMAGE="${IMAGE:-robot-orchestrator:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-robot_orchestrator}"
ROBOT_WEB_BRIDGE_PORT="${ROBOT_WEB_BRIDGE_PORT:-8088}"

echo "==> Starting lean orchestrator container"
echo "    ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "    Image         = $IMAGE"
echo ""

# ---------------------------------------------------------------------------
# Default command if none provided: source ROS + workspace, drop into shell
# ---------------------------------------------------------------------------
SETUP_CMD='source /opt/ros/humble/setup.bash;'
DEFAULT_CMD="${SETUP_CMD} source /ros2_ws/install/setup.bash 2>/dev/null; cd /ros2_ws && exec bash"

if [ $# -eq 0 ]; then
    USER_CMD="$DEFAULT_CMD"
else
    USER_CMD="${SETUP_CMD} source /ros2_ws/install/setup.bash 2>/dev/null; $*"
fi

# ---------------------------------------------------------------------------
# Generate CycloneDDS unicast config from network.env values
# ---------------------------------------------------------------------------
CYCLONE_XML="/tmp/cyclonedds_orchestrator.xml"
cat > "$CYCLONE_XML" << XMLEOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain id="${ROS_DOMAIN_ID}">
        <General>
            <Interfaces>
                <NetworkInterface name="${DDS_INTERFACE}"/>
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <Peers>
                <Peer Address="${WORKSTATION_IP}"/>
                <Peer Address="${JETRACER_IP}"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
XMLEOF

# ---------------------------------------------------------------------------
# Launch — mounts orchestrator_ws itself (this repo's source, not an external
# workspace like jetracer_ws's Isaac-managed one), so build artifacts
# (build/install/log) land back in orchestrator_ws/ on the host (gitignored).
# ---------------------------------------------------------------------------
exec sudo -E docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    -v "$REPO_ROOT/orchestrator_ws:/ros2_ws" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI="file://$CYCLONE_XML" \
    -v "$CYCLONE_XML":"$CYCLONE_XML" \
    -e ROBOT_WEB_BRIDGE_PORT="$ROBOT_WEB_BRIDGE_PORT" \
    "$IMAGE" \
    bash -c "$USER_CMD"
