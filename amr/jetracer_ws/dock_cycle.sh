#!/usr/bin/env bash
# Round-trip docking demo: dock A -> undock -> dock B -> undock.
# Defaults: A=dock1, B=dock0. Override:  ./dock_cycle.sh dock0 dock1
#
# Requires the stack (hardware + Nav2 + jetracer_docker) already running.
# Sequencing is driven by the /docking_state topic published by jetracer_docker.

set -e

WS="$(cd "$(dirname "$0")" && pwd)"
source "$WS/ws_setup.bash"

A=${1:-dock1}
B=${2:-dock0}
STEP_TIMEOUT=${STEP_TIMEOUT:-180}   # seconds to wait for each phase transition

pub_dock()   { ros2 topic pub --once /dock_robot   std_msgs/msg/String "{data: '$1'}" >/dev/null; }
pub_undock() { ros2 topic pub --once /undock_robot std_msgs/msg/Bool   '{data: true}' >/dev/null; }

# Block until /docking_state equals $1 (or STEP_TIMEOUT elapses).
wait_state() {
    local want="$1" t=0 s
    while (( t < STEP_TIMEOUT )); do
        s=$(timeout 3 ros2 topic echo /docking_state std_msgs/msg/String \
                --qos-durability transient_local --qos-reliability reliable --once \
                2>/dev/null | sed -n 's/^data:[[:space:]]*//p' | tr -d "'\"[:space:]")
        if [[ "$s" == "$want" ]]; then return 0; fi
        sleep 1; t=$((t + 1))
    done
    echo "!! TIMEOUT after ${STEP_TIMEOUT}s waiting for state='$want' (saw '$s')" >&2
    return 1
}

echo "==> Dock $A";  pub_dock "$A"; wait_state docked
echo "==> Undock";   pub_undock;    wait_state idle
echo "==> Dock $B";  pub_dock "$B"; wait_state docked
echo "==> Undock";   pub_undock;    wait_state idle
echo "==> Round trip complete: $A -> $B -> undocked."
