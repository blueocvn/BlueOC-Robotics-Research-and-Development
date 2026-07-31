# Robot Arm — Overview

A 5-DOF **SO-ARM 101** with a 1-DOF gripper, driven by **ROS 2 + MoveIt 2**,
that picks up an empty cup, fills it at a water dispenser, and places it on a
tray for an AMR (JetRacer) to carry away.

`5-DOF + gripper` · `ROS 2 Jazzy` · `MoveIt 2 + MTC` · `Isaac Sim` · `Workstation-driven`

## Objectives

- Accurately detect and locate empty cups on the table from the overhead camera.
- Grasp each cup reliably despite a 5-DOF arm and a single-jaw gripper.
- Autonomously transport the cup to the water dispenser and fill it.
- Place the filled cup into an assigned tray slot for the AMR (JetRacer) to collect.
- Repeat across multiple cups without re-grabbing a cup that was already placed.

## Joints

The arm exposes six joints; all six stream on `/isaac_joint_states`.

| Joint | Function |
|-------|----------|
| `Rotation` | Base yaw — turns the whole arm left / right |
| `Pitch` | Shoulder pitch — raises and lowers the upper arm |
| `Elbow` | Elbow flexion — extends forearm reach |
| `Wrist_Pitch` | Wrist tilt — angles the gripper up / down |
| `Wrist_Roll` | Wrist rotation — rolls the gripper about its axis |
| `Jaw` | 1-DOF gripper — opens / closes to grasp the cup |

## Tech Stack

| Layer | Choice | Purpose |
|-------|--------|---------|
| Communication & Control | ROS 2 + MoveIt 2 | Motion planning, arm control, perception for grasp/place |
| Simulation | Isaac Sim | Development and sim-to-real validation |
| Development OS | Ubuntu 24.04 + ROS 2 Jazzy | Native development environment |
| Build System | colcon, rosdep | Build and dependency management |
| Compute | **x86 workstation + NVIDIA GPU** | Runs the *entire* stack — the arm itself has no onboard compute |

### Hardware specifications

!!! info "The arm has no onboard compute"
    The SO-ARM 101 is a **USB peripheral**, not a compute node. Its servos are
    position-commanded over a serial bus from a **host workstation**, which does
    all the planning, perception, and control:

    ```
    6× Feetech STS3215 servos → BusLinker V3.0 → USB-serial → HOST WORKSTATION
    ```

    Unlike the JetRacer (AMR), which carries a Jetson on board, the arm has no
    edge compute of its own.

| Hardware | Spec | Notes |
|----------|------|-------|
| Robotic Arm | SO-ARM 101 — 5-DOF + 1-DOF jaw | Main manipulator |
| Gripper | 1-DOF Jaw — one moving, one **fixed** jaw | Cup is pinned against the fixed jaw, hence the angled approach (`grasp_yaw_bias`) |
| Servos | 6 × Feetech STS3215 bus servos | Magnetic encoders; position-commanded over a serial bus |
| Arm interface | BusLinker V3.0 → USB-serial | **No onboard compute** — the arm is a USB peripheral |
| Cameras | 2 × USB — `top_cam` (overhead), `arm_cam` (eye-in-hand) | Feed the perception + visual-servo loops |
| **Host compute** | **x86 workstation, Ubuntu 24.04, NVIDIA GPU** | Required for Isaac Sim and YOLO inference; runs ROS 2 + MoveIt + perception |

!!! info "Sim-first, with a real-hardware path"
    The arm runs in **Isaac Sim** via `topic_based_ros2_control` for development,
    and there is now a **real-hardware driver**: `feetech_ros2_driver` drives the
    physical Feetech servos over the BusLinker, brought up with `real_all.launch.py`
    (see [Real-Hardware Bringup](ra_hardware_bringup.md)). On real hardware the
    cameras are **not yet calibrated** accurately enough for perception-driven
    grasping, so the reliable demo runs open-loop with a predefined object —
    **camera calibration is the current gating task** (see
    [Camera Calibration](ra_camera_calibration.md)).

## Packages & Modules

Four packages make up the arm project. Everything else is an upstream
dependency (see the [Setup Guide](ra_setup.md)).

??? package "1 · so_arm_description"
    SO-ARM 101 URDF + meshes — hardware description and kinematics model.

    - URDF (Unified Robot Description Format) + 3D meshes
    - Joint definitions: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw
    - Hardware description and kinematics model

??? package "2 · so_arm_moveit_config"
    MoveIt 2 configuration for planning and control.

    - SRDF (Semantic Robot Description Format) configuration
    - Kinematics solvers: position-only IK for 5-DOF reachability
    - Motion planning: OMPL using RRTConnect
    - Integration: `ros2_control` + controller integration for joint control

??? package "3 · so_arm_perception"
    Cup, tray, and dispenser detection from the top and arm cameras.

    - Cup detection: YOLO (`yolo11n`) with HSV color-space fallback for sim
    - Tray detection: pink-tray detector (AprilTag-based localization)
    - Dispenser detection: AprilTag detector for water-dispenser positioning
    - Camera inputs: `top_cam` (overhead), `arm_cam` (eye-in-hand)

??? package "4 · mtc_tutorial"
    Orchestrates the complete grasp-and-place pipeline.

    - MTC Node (MoveIt Task Constructor) drives the pipeline end to end
    - Pipeline stages: grasp → servo (visual servoing) → fill → place
    - Launch files and task definitions

## The ROS Contract (Isaac Sim)

ROS talks to Isaac Sim through `topic_based_ros2_control`. The scene's action
graph must publish/subscribe these topics.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Isaac → ROS | `/isaac_joint_states` | sensor_msgs/JointState | all 6 joints |
| ROS → Isaac | `/isaac_joint_commands` | sensor_msgs/JointState | position commands; drive joints to these |
| Isaac → ROS | `/clock` | rosgraph_msgs/Clock | everything runs with `use_sim_time:=true` |
| Isaac → ROS | top-cam RGB + camera_info | sensor_msgs/Image, CameraInfo | overhead camera (namespace `top_cam`) |
| Isaac → ROS | arm-cam RGB + depth + camera_info | sensor_msgs/Image, CameraInfo | eye-in-hand camera (namespace `arm_cam`) |

Camera namespaces are perception-node parameters (`camera_eth_ns` = `top_cam`,
`camera_eih_ns` = `arm_cam`) — match Isaac's topics to these, or override the
params.

## Roadmap

**Phase 0 — Simulation** (5/5) ✅

- [x] Isaac Sim scene — arm, table, cups, dispenser, tray, overhead + eye-in-hand cameras
- [x] Perception in sim — YOLO cup detection, AprilTag dispenser, pink-tray detection
- [x] Grasp strategy — level side-grasp, position-only IK, IBVS visual servoing
- [x] **End-to-end cup refill simulated — detect → grasp → fill → place**
- [x] Multi-cup loop with even tray-slot assignment

**Phase 1 — Foundation & Setup** (2/5)

- [x] Hardware procurement: HiWonder LeRobot SO-ARM101
- [x] ROS 2 Jazzy environment setup (Ubuntu 24.04)
- [ ] Real-world training dataset collection
- [ ] YOLO model training on real-world data
- [ ] Arm kinematics validation on physical hardware

**Phase 2 — Integration & Testing** (0/5)

- [ ] MoveIt 2 trajectory planning validation
- [ ] Perception pipeline integration (YOLO + AprilTag)
- [ ] Sim-to-real transfer validation
- [ ] Integration testing with AMR (JetRacer)
- [ ] End-to-end cup delivery workflow testing

**Phase 3 — Optimization & Deployment** (0/4)

- [ ] Performance tuning & latency optimization
- [ ] Robustness testing (cup types, lighting)
- [ ] Error handling & recovery mechanisms
- [ ] Deployment to production environment
