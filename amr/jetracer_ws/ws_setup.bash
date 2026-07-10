#!/usr/bin/env bash
# Source ROS 2 + EVERY package installed in this workspace.
#
# Why this exists: the merged install/setup.bash on this box is incomplete.
# Mixed file ownership (some packages root-owned, some user-owned) makes
# colcon drop the packages it can't rewrite when it regenerates setup.bash,
# so robot_localization, nav2_* and jetracer_bringup go missing from the
# environment even though they're installed on disk. To avoid relying on
# that file, we source each installed package's local_setup.bash directly.
#
# Usage:  source ws_setup.bash      (or let the start_*.sh scripts use it)

# --- ROS 2 underlay (skip if ros2 is already on PATH) ---
if ! command -v ros2 >/dev/null 2>&1; then
    for _d in /opt/ros/*/install/setup.bash /opt/ros/*/setup.bash; do
        if [ -f "$_d" ]; then
            source "$_d"
            break
        fi
    done
fi

# --- This workspace's overlay: source every installed package ---
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for _f in "$_WS"/install/*/share/*/local_setup.bash; do
    [ -f "$_f" ] && source "$_f" 2>/dev/null || true
done

unset _d _f _WS
