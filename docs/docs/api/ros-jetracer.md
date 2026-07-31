# ROS 2 Interfaces — JetRacer

Topics, services and actions for the JetRacer AMR. Use these when you need
full-rate feedback or control that the [HTTP API](http.md) doesn't expose.

All names below are absolute unless marked *relative*, in which case the node's
namespace applies.

## Command the robot

These are the topics you publish to. The [HTTP API](http.md) is a thin wrapper
over exactly these.

| Topic | Type | Published by | Effect |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | you, bridge, docker | Direct velocity command |
| `/dock_robot` | `std_msgs/msg/String` | you, bridge | Dock at the named dock ID |
| `/dock_sequence` | `std_msgs/msg/String` | you | Run a multi-dock sequence |
| `/undock_robot` | `std_msgs/msg/Bool` | you | Leave the current dock |
| `/abort_docking` | `std_msgs/msg/Bool` | you, bridge | Abort the active docking run |
| `/relocalize_at_dock` | `std_msgs/msg/String` | bridge | Rotate to find the dock's AprilTag, then re-seed pose |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | bridge, docker | Seed AMCL pose |

```bash
# send the robot to dock0
ros2 topic pub --once /dock_robot std_msgs/msg/String "{data: 'dock0'}"

# stop everything
ros2 topic pub --once /abort_docking std_msgs/msg/Bool "{data: true}"
```

!!! danger "`/cmd_vel` has no arbitration"

    The bridge, `jetracer_docker`, Nav2 and your own node all publish to
    `/cmd_vel`, and nothing mediates between them. If you publish while a docking
    run is active, both streams reach the motors and the robot behaves
    unpredictably. Abort first, then drive.

## Read robot state

| Topic | Type | Published by | Notes |
|---|---|---|---|
| `/docking_state` | `std_msgs/msg/String` | `jetracer_docker` | **Latched** (transient local) — late subscribers still get the current phase |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF | Preferred map pose |
| `/chassis/odom` | `nav_msgs/msg/Odometry` | driver | Raw fallback when EKF is unavailable |
| `odom` *(relative)* | `nav_msgs/msg/Odometry` | `jetracer_driver` | Wheel odometry straight from serial |
| `imu` *(relative)* | `sensor_msgs/msg/Imu` | `jetracer_driver` | Chassis IMU |
| `motor/lvel`, `motor/rvel` | `std_msgs/msg/Int32` | `jetracer_driver` | Measured wheel velocities |
| `motor/lset`, `motor/rset` | `std_msgs/msg/Int32` | `jetracer_driver` | Commanded wheel setpoints |

!!! warning "`/docking_state` strings are robot-defined"

    The exact phase strings are not fixed by this API — confirm them against the
    live robot before matching on them:

    ```bash
    ros2 topic echo /docking_state
    ```

    The bridge maps them onto user-facing status via env vars
    (`ROBOT_WEB_BRIDGE_INPROGRESS_STATES`, `_SUCCESS_STATES`, `_ERROR_STATES`).

## Map configuration

The admin map editor pushes configuration to the robot over two **latched**
String topics carrying JSON:

| Topic | Type | Payload |
|---|---|---|
| `/virtual_obstacles` | `std_msgs/msg/String` | JSON list of keep-out rectangles |
| `/dock_registry` | `std_msgs/msg/String` | JSON dock registry with `pose_x`, `pose_y`, `yaw` |

Both are transient-local, so a node that starts late still receives the current
configuration.

## Actions and services

| Name | Type | Role |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 goal — used by `jetracer_docker` |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | Re-seed local costmap after undock |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | Re-seed global costmap after undock |

The costmap clears exist so the dock the robot just left isn't retained as a
stale obstacle.

## Node parameters

### `jetracer_driver` — `cmd_vel_to_serial`

Bridges `cmd_vel` to the chassis over serial at 50 Hz.

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial device for the chassis MCU |

### `ackermann_dock_filter`

Converts Twist commands into Ackermann-feasible motion during docking — the
JetRacer cannot turn in place, so raw differential-drive commands are unusable.

| Parameter | Default | Meaning |
|---|---|---|
| `wheelbase` | `0.20` | Metres between axles |
| `delta_max_deg` | `30.0` | Max steering angle |
| `v_min_threshold` | `0.02` | Below this speed, treat as stationary |
| `input_topic` | `/docking/cmd_vel` | Twist in |
| `output_topic` | `/cmd_vel` | Twist out |

### `dock_pose_publisher`

Publishes dock poses from AprilTag detections via TF.

| Parameter | Default | Meaning |
|---|---|---|
| `camera_frame` | `camera_optical_frame` | Frame detections arrive in |
| `dock_frames` | `[dock_0, dock_1, dock_2]` | TF frames to publish poses for |
| `detection_topic` | `detected_dock_pose` | `geometry_msgs/msg/PoseStamped` out |
| `publish_rate` | `15.0` | Hz |
| `detection_timeout` | `0.5` | Seconds before a detection goes stale |

## See also

- [HTTP API](http.md) — the same commands without a ROS install
- [Launch entry points](launch.md) — bringing the stack up
