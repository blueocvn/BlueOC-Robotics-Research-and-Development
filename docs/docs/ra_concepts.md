# Robot Arm — Core Concepts

A 5-DOF **SO-ARM 101** with a 1-DOF gripper, driven by **ROS 2 + MoveIt 2**,
that picks up an empty cup, fills it at a water dispenser, and places it on a
tray for an AMR (JetRacer) to carry away.

`5-DOF + gripper` · `ROS 2 Jazzy` · `MoveIt 2 + MTC` · `Isaac Sim` · `Jetson-class compute`

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
| Simulation | Isaac Sim / Isaac Lab | Development, training, sim-to-real validation |
| Development OS | Ubuntu 24.04 + ROS 2 Jazzy | Native development environment |
| Build System | colcon, rosdep | Build and dependency management |
| Compute | Jetson Nano / Jetson-class | Edge AI processing for control and perception |

### Hardware specifications

| Hardware | Spec | Notes |
|----------|------|-------|
| Robotic Arm | SO-ARM 101 (5-DOF) | Main manipulator |
| Gripper | 1-DOF (Jaw) | Cup grasping mechanism |
| Compute | Jetson Nano / Jetson-class | Edge AI processing |
| RAM | 4 GB (Jetson Nano Legacy) | Sufficient for ROS 2 + perception |

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

    - MTC Node (Motion Task Commander) drives the pipeline end to end
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
