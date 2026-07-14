#!/usr/bin/env bash
#
# run_orchestrator.sh
# Launch the LEAN orchestrator-only Docker container (../Dockerfile.orchestrator)
# -- just orchestrator/, no SLAM/Nav2/Isaac Sim, no GUI. For running/deploying
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

# ---------------------------------------------------------------------------
# Docker invocation — use sudo only if the daemon isn't reachable as the
# current user (rootless / docker-group setups don't need it).
# ---------------------------------------------------------------------------
DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
    DOCKER="sudo -E docker"
fi

# ---------------------------------------------------------------------------
# The image is built locally from Dockerfile.orchestrator (never pulled from a
# registry). Build it automatically the first time if it's missing. This runs
# BEFORE the DDS validation below, since building the image is independent of
# network.env — a fresh install (no network.env yet) must still get the image.
# ---------------------------------------------------------------------------
if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> Image '$IMAGE' not found locally — building it now..."
    $DOCKER build -f "$REPO_ROOT/Dockerfile.orchestrator" -t "$IMAGE" "$REPO_ROOT"
    echo ""
fi

# ---------------------------------------------------------------------------
# Validate the DDS settings before we bake them into the CycloneDDS config.
# A malformed IP or a network interface that doesn't exist on this host makes
# CycloneDDS fail to start, surfacing only as the cryptic ROS error
# "rcl node's rmw handle is invalid" once nodes try to come up. Catch it here
# with a clear message instead — especially important on a fresh install where
# network.env was just copied from the example.
# ---------------------------------------------------------------------------
is_ipv4() {
    # Strict dotted-quad check: four 0-255 octets.
    [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
    local IFS=. o
    for o in $1; do (( o <= 255 )) || return 1; done
    return 0
}

for var in WORKSTATION_IP JETRACER_IP; do
    val="${!var}"
    if [[ "$val" == *XX* ]]; then
        echo "ERROR: $var is still a placeholder ('$val'). Edit $REPO_ROOT/network.env" >&2
        echo "       and set the real LAN IP." >&2
        exit 1
    fi
    if ! is_ipv4 "$val"; then
        echo "ERROR: $var='$val' is not a valid IPv4 address (check $REPO_ROOT/network.env)." >&2
        echo "       CycloneDDS would reject it and every ROS node would fail to start." >&2
        exit 1
    fi
done

if ! ip -o link show "$DDS_INTERFACE" >/dev/null 2>&1; then
    echo "ERROR: DDS_INTERFACE='$DDS_INTERFACE' does not exist on this host." >&2
    echo "       Available interfaces:" >&2
    ip -o -4 addr show | awk '{printf "         %-12s %s\n", $2, $4}' >&2
    echo "       Set DDS_INTERFACE in $REPO_ROOT/network.env to the one carrying" >&2
    echo "       WORKSTATION_IP ($WORKSTATION_IP)." >&2
    exit 1
fi

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
# Launch — mounts orchestrator/ itself (this repo's source, not an external
# workspace like jetracer_ws's Isaac-managed one), so build artifacts
# (build/install/log) land back in orchestrator/ on the host (gitignored).
# ---------------------------------------------------------------------------
exec $DOCKER run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    -v "$REPO_ROOT/orchestrator:/ros2_ws" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI="file://$CYCLONE_XML" \
    -v "$CYCLONE_XML":"$CYCLONE_XML" \
    -e ROBOT_WEB_BRIDGE_PORT="$ROBOT_WEB_BRIDGE_PORT" \
    "$IMAGE" \
    bash -c "$USER_CMD"
