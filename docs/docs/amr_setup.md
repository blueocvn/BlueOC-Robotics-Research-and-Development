# AMR Setup (`jetracer_ws`) — JetRacer SLAM + Nav2 in Isaac Sim

A car-like (Ackermann) mobile robot that maps a space, localizes, and navigates
between docks to fulfil orders from the web orchestrator. Everything here runs on
the **workstation** and drives **Isaac Sim** — there is no on-device JetRacer
firmware in this repo yet, so the same stack will later drive the real chassis
once a driver consumes `/ackermann_cmd`.

> **⚠️ ROS distro:** `jetracer_ws` targets **ROS 2 Humble** and is built/run
> inside the repo's **`Dockerfile.dev`** container — *not* the native Jazzy
> install `ra_ws` uses. The workspaces still interoperate over DDS (use the same
> `ROS_DOMAIN_ID`).

### 1. Prerequisites

| Component | Version / notes |
|---|---|
| OS (host) | Ubuntu 22.04 recommended |
| ROS 2 | **Humble** (provided by the `Dockerfile.dev` image) |
| Docker | with NVIDIA Container Toolkit (GPU passthrough for Isaac Sim) |
| Isaac Sim | Any recent release with the ROS 2 Bridge extension enabled |
| Build tools | `colcon`, `rosdep`, `git` |
| Extra deps | `slam_toolbox`, `nav2`, `pointcloud_to_laserscan` (via `rosdep`) |

> The AMR stack is designed to be built inside the container so the Humble
> toolchain and dependencies are pinned. Isaac Sim itself runs on the host (or in
> its own container) and talks to the workspace over the ROS 2 graph.

### 2. Workspace layout

The AMR project's own packages. Note the **package name** is what you pass to
`ros2 launch` / `ros2 run` — several live inside grouping folders (`navigation/`,
`ackermann_control/`), so the folder name and package name differ.

| Package (use this name) | Folder | What it is |
|---|---|---|
| `carter_navigation` | `navigation/carter_navigation` | Nav2 bring-up, params, maps, lidar→scan conversion (adapted from the Isaac carter sample) |
| `slam_custom` | `navigation/slam_custom` | SLAM bring-up — wraps `slam_toolbox` online-async + a preconfigured rviz |
| `isaac_ros_navigation_goal` | `navigation/isaac_ros_navigation_goal` | goal sender (`SetNavigationGoal`) → `NavigateToPose` + `/initialpose` |
| `cmdvel_to_ackermann` | `ackermann_control/cmdvel_to_ackermann` | `/cmd_vel` (Twist) → `/ackermann_cmd` (AckermannDriveStamped) bridge |
| `isaacsim` | `isaacsim` | Isaac Sim launcher (`run_isaacsim.launch.py`) |
| `isaac_ros2_messages` | `isaac_ros2_messages` | message types for the Isaac bridge |
| `isaac_compressed_image_decoder` | `isaac_compressed_image_decoder` | Isaac compressed image stream → raw `sensor_msgs/Image` (exec `decoder_node`) |

`src/slam_toolbox/` is **not a package** — it's where serialized maps land
(`map_*.pgm` / `map_*.yaml`). The `slam_toolbox` package itself is an upstream
dependency resolved by `rosdep`.

| Vendored (reference only, not core) | Note |
|---|---|
| `iw_hub_navigation` | Isaac iw.hub AMR nav sample |
| `carter_navigation` multi-robot launches (`multiple_robot_*`) | hospital/office multi-Carter demos |
| `isaac_tutorials` | Isaac ROS 2 sample publishers + rviz configs |
| `h1_fullbody_controller` (`humanoid_locomotion_policy_example`), `custom_message` | H1 humanoid sample + `SampleMsg` scaffold — unrelated |

### 3. Build the workspace

Inside the `Dockerfile.dev` container, with the repo mounted:

```bash
cd jetracer_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> `run_workstation.sh` (in `jetracer_ws/`) is the convenience entrypoint that
> launches the container with the right mounts, GPU flags, and DDS config
> (`fastdds.xml.template`). Prefer it over a hand-rolled `docker run`.

### 4. Isaac Sim setup (the ROS contract)

The AMR talks to Isaac Sim over these topics — confirm the scene's ROS 2 action
graph publishes/subscribes them:

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Isaac → ROS | `/chassis/odom` | `nav_msgs/Odometry` | base odometry; Nav2 localization uses this |
| Isaac → ROS | `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 3D lidar; `carter_navigation` runs a `pointcloud_to_laserscan` node that converts it to `/scan` for SLAM/Nav2 |
| ROS → Isaac | `/cmd_vel` | `geometry_msgs/Twist` | velocity command Nav2 emits |
| ROS → Isaac | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Ackermann drive (from `cmdvel_to_ackermann`) |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | everything runs with `use_sim_time:=true` |
| Isaac → ROS | camera RGB (compressed) | `sensor_msgs/CompressedImage` | decoded by `isaac_compressed_image_decoder` (`decoder_node`) |

You can start Isaac Sim from ROS with the `isaacsim` launcher (or open the stage
in the Isaac GUI manually):

```bash
ros2 launch isaacsim run_isaacsim.launch.py \
    gui:=/path/to/your_scene.usd \
    play_sim_on_start:=true \
    ros_distro:=humble
```

Then verify the contract in a sourced terminal:
```bash
ros2 topic hz /chassis/odom
ros2 topic hz /clock
ros2 topic echo /front_3d_lidar/lidar_points --once
```

### 5. Map the space (SLAM)

> **⚠️ Isaac Sim must be running first.** Open the AMR scene and press **Play**
> (see §4) *before* launching SLAM or Nav2. The nodes need `/clock`,
> `/chassis/odom`, and the lidar stream — they stall (or SLAM never builds a map)
> until Isaac is playing and publishing those topics.

`slam_custom` runs `slam_toolbox` online-async with a preconfigured rviz. It is
sim-time aware and waits `startup_delay` seconds for the clock to settle.

```bash
source install/setup.bash
ros2 launch slam_custom slam_custom.launch.py
```

| Arg | Default | Purpose |
|---|---|---|
| `slam_params_file` | `slam_custom/params/slam_toolbox_params.yaml` | slam_toolbox configuration |
| `startup_delay` | (see launch file) | seconds to wait before starting slam_toolbox so the sim clock is live |

Drive the robot around (teleop or `/cmd_vel`), then serialize the map:
```bash
ros2 run slam_toolbox serialize_map -f my_map
```
Saved maps land under `src/slam_toolbox/` (`map_*.pgm` / `map_*.yaml`).

### 6. Localize + navigate (Nav2)

With a saved map, bring up Nav2 (AMCL localization + planner/controller/BT
servers + the lidar→scan conversion):

```bash
source install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/absolute/path/to/your_map.yaml
```

| Arg | Default | Purpose |
|---|---|---|
| `map` | `carter_navigation/maps/carter_warehouse_navigation.yaml` | map to load — override with your own |
| `params_file` | `carter_navigation/params/...` | Nav2 params |
| `use_sim_time` | `true` | use the Isaac Sim clock |

> Sibling launch files exist for other setups:
> `carter_navigation_isaacsim.launch.py` (bundles the Isaac stage),
> `carter_navigation_individual.launch.py`, and the `multiple_robot_*` demos
> (vendored, multi-robot — not the JetRacer path).

Send a goal / seed the initial pose:
```bash
# one-shot goal sender
ros2 run isaac_ros_navigation_goal SetNavigationGoal
# or via its launch file (reads goals from its config)
ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py
```
You can also drop a **2D Pose Estimate** (seeds `/initialpose`) and a **Nav2 Goal**
from rviz.

### 7. Drive interface (Ackermann)

Nav2's controller emits `/cmd_vel` (Twist); the JetRacer is car-like, so
`cmdvel_to_ackermann` converts it to `/ackermann_cmd`:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py
# (equivalently: ros2 run cmdvel_to_ackermann cmdvel_to_ackermann.py)
```

It subscribes `/cmd_vel` → publishes `/ackermann_cmd`, guarding against invalid
Ackermann commands (zero linear velocity with non-zero steering).

### 8. Orchestrator integration (status)

The web orchestrator (`orchestrator/robot_web_bridge`) owns a single dispatcher
that drives the AMR dock-to-dock. On the AMR side this seam is **not yet wired**:

| Contract | State |
|---|---|
| Orchestrator publishes `/dock_robot`, reads `/docking_state`, `/chassis/odom`, seeds `/initialpose` | Built (orchestrator side) |
| A `/dock_robot` **consumer** on the AMR (dock id → Nav2 goal / docking behavior) | **Not implemented** |
| A real `/docking_state` **producer** on the AMR | **Not implemented** (dispatcher strings are placeholders) |

### 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Robot never moves | Isaac not in **Play**, or the sim isn't subscribed to `/cmd_vel` / `/ackermann_cmd`. |
| SLAM map is empty / drifts | No `/scan` — check the `pointcloud_to_laserscan` node is up and `/front_3d_lidar/lidar_points` is streaming; or `/clock` missing so `use_sim_time` nodes stall. |
| Nav2 won't localize | No `/initialpose` seeded, or the map doesn't match the scene — re-map. |
| `slam_custom` starts before the clock | Increase `startup_delay`; slam_toolbox needs `/clock` live first. |
| Everything is slow / time jumps | `/clock` not published, or a node started without `use_sim_time:=true`. |
| DDS peers can't see each other | Mismatched `ROS_DOMAIN_ID`, or `fastdds.xml` not applied across containers. |

### 10. Notes for maintainers

- The whole AMR stack runs on **sim time** (`use_sim_time:=true`); keep `/clock` flowing.
- Lidar reaches SLAM/Nav2 as `/scan`, produced by a `pointcloud_to_laserscan`
  node inside `carter_navigation` from `/front_3d_lidar/lidar_points`.
- Docking is currently a **topic contract** (`/dock_robot` + `/docking_state`), not a
  Nav2 docking server — the AMR-side consumer/producer are the main open work.
- The default `carter_navigation` map is the warehouse sample — supply your own
  mapped `.yaml` via `map:=` for the real space.
- Several vendored samples (`iw_hub_navigation`, `multiple_robot_*`,
  `h1_fullbody_controller`, `custom_message`, unused `isaac_tutorials`) are
  reference only and can be pruned.