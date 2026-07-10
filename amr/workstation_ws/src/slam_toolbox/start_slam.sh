#!/usr/bin/env bash
# Start SLAM Toolbox (async mode) with custom params

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null

ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$(pwd)/my_slam_toolbox_params.yaml
