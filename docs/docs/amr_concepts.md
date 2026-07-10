# JetRacer (AMR) — Core Concepts

A car-like (Ackermann) autonomous mobile robot that maps a space, localizes, and
navigates between docks to fulfil orders — locate a person, drive from the water
dispenser to them, and avoid obstacles in between. Built on **ROS 2 Humble,
SLAM, and Nav2**.

`ROS 2 Humble` · `Jetson Nano · 4GB` · `Ackermann Drive` · `SLAM + Nav2` · `Isaac Sim`

!!! warning "Sim-only today"
    Everything runs on the **workstation** and drives **Isaac Sim** — there is
    no on-device JetRacer firmware in this repo yet. The same stack will later
    drive the real chassis once a driver consumes `/ackermann_cmd`.

## Objectives

- Accurately identify and locate the position of the person who sent the signal.
- Autonomously transport a water-filled cup from Position B (dispenser) to
  Position A (person).
- Detect and avoid obstacles — furniture and other objects — during navigation.
- Deliver the tray to the person without spilling the water.

## Technical Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| Navigation & SLAM | ROS 2 + SLAM + Nav2 | Navigation, localization/SLAM, obstacle avoidance |
| Hardware Base | JetRacer on Jetson Nano | Mobile platform |
| Sensors | LiDAR, Wheel Encoders, Camera | Navigation, localization, obstacle detection |
| Drive Interface | Ackermann Control | `/cmd_vel` → `/ackermann_cmd` |
| Compute | Jetson Nano (Legacy) | 4 GB RAM edge AI processing |
| Dev OS | Ubuntu 22.04 + ROS 2 Humble (`Dockerfile.dev`) | Containerized dev environment |

### Dependencies

ROS 2 Humble · Nav2 · `slam_toolbox` · OpenCV · geometry_msgs · Ackermann
steering messages · colcon.

## Package map

The AMR's own packages (pass the **package name**, not the folder, to `ros2
launch`). Full table with folders and vendored samples in the
[Setup Guide §2](amr_setup.md).

??? package "carter_navigation — Nav2 bring-up"
    Nav2 params, maps, and the `pointcloud_to_laserscan` conversion
    (`/front_3d_lidar/lidar_points` → `/scan`). Adapted from the Isaac Carter
    sample and tailored for the JetRacer.

??? package "slam_custom — SLAM bring-up"
    Wraps `slam_toolbox` online-async with a preconfigured RViz. Sim-time aware.

??? package "cmdvel_to_ackermann — Drive interface"
    Converts Nav2's `/cmd_vel` (Twist) → `/ackermann_cmd`
    (AckermannDriveStamped), guarding invalid commands.

??? package "isaac_ros_navigation_goal — Goal sender"
    Sends `NavigateToPose` goals and seeds `/initialpose`.

!!! note
    `src/slam_toolbox/` is **not a package** — it's where serialized maps land
    (`map_*.pgm` / `map_*.yaml`). The `slam_toolbox` package itself is an
    upstream `rosdep` dependency.

## The ROS Contract (Isaac Sim)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Isaac → ROS | `/chassis/odom` | nav_msgs/Odometry | base odometry for Nav2 localization |
| Isaac → ROS | `/front_3d_lidar/lidar_points` | sensor_msgs/PointCloud2 | converted to `/scan` for SLAM/Nav2 |
| ROS → Isaac | `/cmd_vel` | geometry_msgs/Twist | velocity command from Nav2 |
| ROS → Isaac | `/ackermann_cmd` | ackermann_msgs/AckermannDriveStamped | Ackermann drive |
| Isaac → ROS | `/clock` | rosgraph_msgs/Clock | everything runs with `use_sim_time:=true` |
| Isaac → ROS | camera RGB (compressed) | sensor_msgs/CompressedImage | decoded by `isaac_compressed_image_decoder` |

## Roadmap & Progress

| Phase | Focus | Status |
|-------|-------|--------|
| **P0** | Isaac Sim scene + ROS 2 bridge | ✅ Complete |
| **P1** | SLAM mapping | ✅ Complete |
| **P2** | Nav2 localization + goal nav | ✅ Complete |
| **P3** | Drive interface + fulfillment loop | 🟡 Partial |
| **P4** | Sim-to-real / on-device firmware | ⚪ Planned |

- **P3** — `cmd_vel` → Ackermann is built; the orchestrator's `/dock_robot`
  protocol is **not yet consumed** by the AMR (see
  [Pick and Deliver](solution_pick_and_deliver.md)).
- **P4** — No JetRacer firmware in the repo yet — the workstation drives sim
  only.
