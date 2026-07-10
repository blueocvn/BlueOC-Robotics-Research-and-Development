# AMR Use Case — Navigate & Deliver

The AMR's job in the fulfillment loop: map a space, localize in it, then drive
dock-to-dock to carry the tray from the dispenser to the person. (A mobile base
doesn't grasp — the "pick" is done by the [arm](ra_pick_and_place.md); the AMR
transports.) For install/build see the [Setup Guide](amr_setup.md).

## Prerequisites

- Workspace built inside the `Dockerfile.dev` container and Isaac Sim running
  with the ROS contract verified — [Setup Guide §1–4](amr_setup.md).
- Isaac scene playing (`/chassis/odom`, `/front_3d_lidar/lidar_points`, `/clock`
  live).

## Scenario A — Navigate a known map

Bring up Nav2 (AMCL localization + planner/controller/BT + lidar→scan), then
send goals:

```bash
source install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/absolute/path/to/your_map.yaml
```

Seed the pose and send a goal:

```bash
# one-shot goal sender
ros2 run isaac_ros_navigation_goal SetNavigationGoal
```

Or, in RViz, drop a **2D Pose Estimate** (seeds `/initialpose`) then a **Nav2
Goal**. Full arg tables in [Setup Guide §6](amr_setup.md).

## Scenario B — Build a new map first

```bash
# 1. run SLAM (slam_toolbox online-async + rviz)
ros2 launch slam_custom slam_custom.launch.py
# 2. drive around (teleop or /cmd_vel), then serialize
ros2 run slam_toolbox serialize_map -f my_map
```

Saved maps land under `src/slam_toolbox/` — see [Setup Guide §5](amr_setup.md).

## Drive interface

Nav2 emits `/cmd_vel` (Twist); the car-like JetRacer needs Ackermann, so run the
bridge:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py
# /cmd_vel  →  /ackermann_cmd
```

## Docking (fulfillment)

Dock-to-dock delivery is the last leg of the loop, coordinated by the
orchestrator. The AMR-side consumer is the current open work — see the status
and contract in [Pick and Deliver](solution_pick_and_deliver.md) and
[Setup Guide §8](amr_setup.md).

## If it stalls

- **Robot never moves** → Isaac not in Play, or not subscribed to `/cmd_vel` /
  `/ackermann_cmd`.
- **SLAM empty / drifts** → no `/scan` (check `pointcloud_to_laserscan`), or
  `/clock` missing.
- **Nav2 won't localize** → no `/initialpose` seeded, or the map doesn't match
  the scene.

More in [Setup Guide §9](amr_setup.md).
