#!/usr/bin/env bash
# Start RPLidar A1 + static TF for Waveshare JetRacer

source /opt/ros/humble/setup.bash

# Start static TF publisher in background
# base_footprint -> laser_frame
# yaw=3.14159 because lidar is mounted inverted on Waveshare JetRacer
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 \
    --y 0.0 \
    --z 0.18 \
    --roll 0.0 \
    --pitch 0.0 \
    --yaw 3.14159 \
    --frame-id base_footprint \
    --child-frame-id laser_frame &

TF_PID=$!
echo "==> TF publisher started (PID $TF_PID)"

# Start lidar (foreground)
echo "==> Starting RPLidar A1 on /dev/ttyACM1"
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyACM1 \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser_frame \
    -p angle_compensate:=true \
    -p scan_mode:=Standard \
    -p scan_frequency:=10.0 

# If lidar exits, kill TF too
kill $TF_PID 2>/dev/null
