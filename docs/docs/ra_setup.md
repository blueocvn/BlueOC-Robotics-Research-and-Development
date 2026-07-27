# Robotic Arm Setup (`ra_ws`) — SO-ARM 101 Isaac Sim Refill Demo

A 5-DOF SO-ARM 101 grasps a green-interior mug in NVIDIA Isaac Sim, visually
servos onto it with an eye-in-hand camera, carries it to an AprilTag-marked
dispenser to "fill", and places it in a pink tray. Perception, MoveIt 2 / MoveIt
Task Constructor (MTC), and a visual-servo loop run on the ROS 2 side; Isaac Sim
provides the physics, the robot, and the cameras.

This guide assumes you already have **Isaac Sim installed** and starts there.

!!! tip "Don't have Isaac Sim yet? Start with NVIDIA's official guides"
    - **[Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)**
      — download, install, and launch Isaac Sim on Linux (includes the
      compatibility checker for GPU/driver requirements).
    - **[ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)**
      — enable the ROS 2 Bridge extension and point it at your ROS install.

    !!! warning "Source ROS 2 *before* launching Isaac Sim"
        The bridge loads the ROS 2 libraries from your **sourced** environment.
        Run `source /opt/ros/jazzy/setup.bash` in the terminal you launch Isaac
        from, or the bridge will pick up the wrong distro (or none at all).

> **⚠️ ROS distro:** `ra_ws` targets **ROS 2 Jazzy (Ubuntu 24.04)** — *not* the
> Humble stack the rest of the repo (`jetracer_ws`, `orchestrator`) runs in.
> Build and run `ra_ws` against a native Jazzy install, **not** inside the
> `Dockerfile.dev` Humble container. The workspaces still interoperate over DDS
> (use the same `ROS_DOMAIN_ID`).

### 1. Prerequisites

| Component | Version / notes |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | **Jazzy** (`ros-jazzy-desktop`) |
| Isaac Sim | Any recent release with the ROS 2 Bridge extension enabled — [install guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html) · [ROS 2 setup](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html) |
| Build tools | `colcon`, `rosdep`, `git` |
| Python (perception) | numpy, opencv-python, ultralytics, scipy |

> Isaac Sim's ROS 2 Bridge must be configured for Jazzy (set the bridge to use
> your system ROS, or source your ROS 2 install before launching Isaac).

### 2. Workspace layout

Only these four packages are part of the arm project — everything else is a
standard upstream dependency you install separately (see §3):

| Package | What it is |
|---|---|
| `ra_ws/src/so_arm_description` | SO-ARM 101 URDF + meshes |
| `ra_ws/src/so_arm_moveit_config` | MoveIt 2 config (SRDF, kinematics, OMPL, controllers, `ros2_control`) |
| `ra_ws/src/so_arm_perception` | Cup + tray + AprilTag perception nodes (YOLO / HSV / OpenCV) |
| `ra_ws/src/mtc_tutorial` | `mtc_node` — the grasp → servo → fill → place pipeline, plus launch files |

The Isaac Sim scene (robot, table, mug, tray, dispenser, cameras) lives under
the repo's `simulation/` folder — open it in Isaac Sim before running.

### 3. Install dependencies

#### 3.1 ROS 2 Jazzy + MoveIt 2
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-moveit \
     ros-jazzy-topic-based-ros2-control \
     ros-jazzy-joint-trajectory-controller \
     ros-jazzy-position-controllers \
     ros-jazzy-joint-state-broadcaster \
     python3-colcon-common-extensions python3-rosdep
```

#### 3.2 MoveIt Task Constructor (MTC)
MTC drives the pick pipeline. If a Jazzy binary is available:
```bash
sudo apt install ros-jazzy-moveit-task-constructor-core
```
Otherwise clone it into `ra_ws/src/` and let `colcon` build it:
```bash
cd ra_ws/src && git clone -b jazzy https://github.com/moveit/moveit_task_constructor.git
```

#### 3.3 Perception Python packages
```bash
python3 -m pip install "numpy>=1.24" "opencv-python>=4.8" "ultralytics>=8.3" \
     "scipy>=1.11" "pupil-apriltags>=1.0"
```
`cv_bridge` comes from apt: `sudo apt install ros-jazzy-cv-bridge`.
The YOLO weights (`yolo11n.pt`) auto-download on first run; no manual step needed.
`pupil-apriltags` provides the AprilTag detector used by `apriltag_node` to
localize the dispenser marker — it is required (the perception launch always
starts that node).

#### 3.4 Resolve the rest with rosdep
```bash
cd ra_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
cd ra_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Isaac Sim setup (the ROS contract)

The ROS side talks to Isaac Sim through **`topic_based_ros2_control`**. Open the
scene and make sure its ROS 2 action graph publishes/subscribes these topics:

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Isaac → ROS | `/isaac_joint_states` | `sensor_msgs/JointState` | all 6 joints: `Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw` |
| ROS → Isaac | `/isaac_joint_commands` | `sensor_msgs/JointState` | position commands; drive the joints to these |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | everything runs with `use_sim_time:=true` |
| Isaac → ROS | top-cam RGB + `camera_info` | `sensor_msgs/Image`, `CameraInfo` | overhead camera (namespace `top_cam`) |
| Isaac → ROS | arm-cam RGB + depth + `camera_info` | `sensor_msgs/Image`, `CameraInfo` | eye-in-hand camera (namespace `arm_cam`) |

The camera namespaces are parameters of the perception node
(`camera_eth_ns` = `top_cam`, `camera_eih_ns` = `arm_cam`) — match Isaac's camera
topics to these, or override the params.

**Steps:**
1. Open the arm scene (under `simulation/`) in Isaac Sim.
2. Confirm the ROS 2 Bridge extension is enabled and set to ROS 2 Jazzy.
3. Press **Play** (physics + camera render products must be running, or
   `/isaac_joint_states` and the camera topics stay silent).
4. In a sourced terminal, verify the contract:
   ```bash
   ros2 topic hz /isaac_joint_states
   ros2 topic hz /clock
   ros2 topic list | grep -E "top_cam|arm_cam"
   ```

### 6. Run the pipeline

> **⚠️ Isaac Sim must be running first.** Open the arm scene and press **Play**
> (see §5) *before* the command below. `mtc_node` blocks on
> `/detected_object/position`, and `move_group` + the controllers need
> `/isaac_joint_states` and `/clock` — nothing moves until Isaac is playing and
> publishing those topics.

One command brings up MoveIt (`move_group` + controllers + RViz), perception, and
`mtc_node`, staggered so each layer's dependencies are up first:

```bash
source install/setup.bash
ros2 launch mtc_tutorial bringup.launch.py
```

What happens:
1. `move_group`, `ros2_control`, and the `arm_group` / `hand_group` controllers start.
2. Perception starts (YOLO cup detector, pink-tray detector, AprilTag detector).
3. `mtc_node` waits for `/detected_object/position`, then per cup:
   gross move → visual servo onto the mug → close claw → carry to the AprilTag
   dispenser → lean/press to "fill" → place in the tray.

The **number of cups** is not configured — it's however many the **top cam**
detects at start (if none, it defaults to one), and they're spread evenly across
the tray (a single cup is centred).

### 7. Key configuration (launch args)

All are args to `pick_place_demo.launch.py` (forward them through `bringup`):

| Arg | Default | Purpose |
|---|---|---|
| `skip_servo` | `false` | `false` = run the image-based (IBVS) arm-cam servo; `true` = skip it (open-loop straight-in grasp) |
| `grasp_yaw_bias` | `-0.5` | approach angle so the mug lands in the single-jaw gap |
| `servo_grasp_z` | `0.05986` | side-grasp height (mug mid-height) |
| `dispenser_standoff` | `0.10` | hold-back from the tag before pressing |
| `dispenser_fill_depth` | `-0.08` | how far the cup ends vs the tag (negative = stops short) |

Example: `ros2 launch mtc_tutorial bringup.launch.py servo_grasp_z:=0.05`.

### 8. Troubleshooting

<div class="table-even" markdown>

| Symptom | Likely cause / fix |
|---|---|
| Arm never moves | Isaac not in **Play**, or Isaac isn't subscribed to `/isaac_joint_commands`. Check `ros2 topic echo /isaac_joint_commands`. |
| `mtc_node` hangs at startup | It's blocked waiting for `/detected_object/position` — perception isn't publishing. Check the camera topics are streaming and match the `camera_*_ns` params. |
| Cameras silent | Isaac render products / camera OmniGraph not active while playing. |
| Planning fails "Start state out of bounds" | A joint settled a hair past a URDF limit; the node re-seats before release, but check `joint_limits.yaml`. |
| Everything is slow / time jumps | `/clock` not published, or a node started without `use_sim_time:=true`. |

</div>

### 9. Notes for maintainers

- The whole arm stack runs on **sim time** (`use_sim_time:=true`); keep `/clock` flowing.
- The gripper opens via the `hand_group_controller/gripper_cmd` GripperCommand
  action; the arm is commanded on `/arm_group_controller/joint_trajectory`.
- Perception's overhead camera also publishes the static `world → top_sim_camera`
  TF used for ray-plane unprojection (in `perception.launch.py`).