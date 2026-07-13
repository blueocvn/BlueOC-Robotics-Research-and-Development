#!/usr/bin/env bash
#
# run_workstation.sh
# Launch a ROS 2 Humble Docker container on an Ubuntu 24.04 workstation,
# configured to talk to a JetRacer running Humble on the same LAN.
#
# Usage:
#   ./run_workstation.sh                    # interactive shell
#   ./run_workstation.sh rviz2              # run a single command then exit
#

set -euo pipefail

# ---------------------------------------------------------------------------
# Config — real network values live in <repo>/network.env (gitignored).
# Copy network.env.example to network.env and fill it in.
# ---------------------------------------------------------------------------
# Repo root is two levels up (amr/workstation_ws/ -> repo root), where
# network.env and Dockerfile.dev live.
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
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
WORKSPACE_HOST="${WORKSPACE_HOST:-$HOME/IsaacSim-ros_workspaces/humble_ws}"
WORKSPACE_CONTAINER="/ros2_ws"
IMAGE="${IMAGE:-jetracer-workstation:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-isaacsim_humble_ws}"

# Set to "true" if you have an NVIDIA GPU and nvidia-container-toolkit installed
USE_GPU="${USE_GPU:-false}"

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
echo "==> Starting Humble workstation container"
echo "    ROS_DOMAIN_ID    = $ROS_DOMAIN_ID"
echo "    Workspace (host) = $WORKSPACE_HOST"
echo "    Image            = $IMAGE"
echo ""

# Make sure workspace folder exists
if [ ! -d "$WORKSPACE_HOST" ]; then
    echo "Workspace directory '$WORKSPACE_HOST' does not exist."
    echo "Creating it now..."
    mkdir -p "$WORKSPACE_HOST/src"
fi

# Allow Docker containers to access the X server (for rviz2, rqt, etc.)
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null
else
    echo "WARNING: xhost not found — GUI apps may not work"
fi

# Build optional GPU flags
GPU_FLAGS=()
if [ "$USE_GPU" = "true" ]; then
    GPU_FLAGS=(--gpus all)
    echo "    GPU support      = enabled"
fi

# ---------------------------------------------------------------------------
# Default command if none provided: source ROS + workspace, drop into shell
# ---------------------------------------------------------------------------
SETUP_CMD='source /opt/ros/humble/setup.bash;'

DEFAULT_CMD="${SETUP_CMD} source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; cd /ros2_ws && exec bash"

if [ $# -eq 0 ]; then
    USER_CMD="$DEFAULT_CMD"
else
    USER_CMD="${SETUP_CMD} source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; $*"
fi

# ---------------------------------------------------------------------------
# Generate CycloneDDS unicast config from network.env values
# ---------------------------------------------------------------------------
CYCLONE_XML="/tmp/cyclonedds.xml"
cat > /tmp/cyclonedds.xml << XMLEOF
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

# Render the Fast DDS profile from its template (keeps the real IP out of git)
FASTDDS_TEMPLATE="$(dirname "$0")/fastdds.xml.template"
if [ -f "$FASTDDS_TEMPLATE" ]; then
    sed "s|\${WORKSTATION_IP}|${WORKSTATION_IP}|g" "$FASTDDS_TEMPLATE" \
        > "$(dirname "$0")/fastdds.xml"
fi

# ---------------------------------------------------------------------------
# Docker invocation — use sudo only if the daemon isn't reachable as the
# current user (rootless / docker-group setups don't need it).
# ---------------------------------------------------------------------------
DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
    DOCKER="sudo -E docker"
fi

# The image is built locally from Dockerfile.dev (never pulled from a registry).
# Build it automatically the first time if it's missing.
if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> Image '$IMAGE' not found locally — building from Dockerfile.dev (this is large, be patient)..."
    $DOCKER build -f "$REPO_ROOT/Dockerfile.dev" -t "$IMAGE" "$REPO_ROOT"
    echo ""
fi

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
exec $DOCKER run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --ipc host \
    --pid host \
    "${GPU_FLAGS[@]}" \
    -v "$WORKSPACE_HOST:$WORKSPACE_CONTAINER" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
    -v "$CYCLONE_XML":/tmp/cyclonedds.xml \
    --device /dev/dri \
    "$IMAGE" \
    bash -c "$USER_CMD"
