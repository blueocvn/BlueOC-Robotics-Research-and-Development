#!/usr/bin/env bash
# Bring up the robot in a tmux session with two side-by-side panes:
#   left  : hardware layer (driver + lidar + EKF)
#   right : Nav2 stack (starts after a short delay so TF/scan are up first)
#
# Usage:   ./start_tmux.sh
# Detach:  Ctrl-b d        Reattach: tmux attach -t jetracer
# Kill:    tmux kill-session -t jetracer

set -e

SESSION=jetracer
WS="$(cd "$(dirname "$0")" && pwd)"
NAV2_DELAY=8   # seconds to let the hardware layer settle before Nav2

if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed." >&2
    exit 1
fi

# If the session already exists, just attach to it.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running -- attaching."
    exec tmux attach -t "$SESSION"
fi

# Pane 0 (left): hardware layer
tmux new-session -d -s "$SESSION" -n bringup
tmux send-keys -t "$SESSION":0.0 "cd '$WS' && ./start_hardware.sh" C-m

# Pane 1 (right): Nav2, after a delay so /scan and TF are available
tmux split-window -h -t "$SESSION":0
tmux send-keys -t "$SESSION":0.1 \
    "cd '$WS' && echo 'Waiting ${NAV2_DELAY}s for hardware to come up...' && sleep ${NAV2_DELAY} && ./start_nav2.sh" C-m

tmux select-layout -t "$SESSION":0 even-horizontal
exec tmux attach -t "$SESSION"
