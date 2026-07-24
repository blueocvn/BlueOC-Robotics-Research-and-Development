# `ra_ws` — SO-ARM 101 Robotic Arm Workspace

ROS 2 **Jazzy** colcon workspace for the SO-ARM 101 pick-and-place arm. Covers
both the Isaac Sim demo and **real-hardware** bringup. For the full narrative see
the docs site (`docs/`): [Setup](../docs/docs/ra_setup.md) ·
[Pick and Place](../docs/docs/ra_pick_and_place.md) ·
[Visual Servoing](../docs/docs/ra_visual_servoing.md) ·
[Imitation Learning](../docs/docs/ra_imitation_learning.md).

## Packages (`src/`)

| Package | Role | Hardware? |
|---------|------|-----------|
| `so_arm_description` | URDF/xacro, meshes, joint properties | shared |
| `so_arm_moveit_config` | MoveIt 2 config: SRDF, controllers, `ros2_control` xacro | shared |
| `mtc_tutorial` | `mtc_node` — MoveIt Task Constructor pick→fill→place pipeline + launch files | shared |
| `so_arm_perception` | YOLO/ray-plane cup + AprilTag + tray/handle detectors, USB camera node | shared |
| `feetech_ros2_driver` | **Real** `ros2_control` hardware interface for the Feetech servos (vendored — see its `VENDORED.md`) | **real only** |

## Real-hardware bringup

The hardware path (vs. the Isaac Sim path) is driven by:

- **`feetech_ros2_driver`** — the servo hardware interface (`/dev/ttyACM0`).
- **`mtc_tutorial/launch/real_all.launch.py`** — one-shot real bringup: MoveIt +
  controllers + perception + `mtc_node`. Toggles: `run_sensing`, `fake_object`,
  `skip_servo` (open-loop deterministic demo), `place_z`, object/tag poses.
- **`mtc_tutorial/launch/bringup_real.launch.py`** — lower-level real bringup.
- **`so_arm_perception/so_arm_perception/usb_camera_node.py`** +
  `launch/top_cam_view.launch.py` — real USB camera → `/arm_cam/rgb`.
- **`so_arm_moveit_config/config/follower_joints.yaml`**,
  `*.ros2_control.xacro`, `ros2_controllers.yaml` — real controller/joint wiring.

> The Isaac Sim scene (`arm_sim.usd`) and reinforcement-learning training are
> intentionally **not** in this repo — this workspace is the ROS 2 control,
> perception, and hardware layer only.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ra_ws
colcon build --symlink-install
source install/setup.bash
```

`build/`, `install/`, and `log/` are git-ignored.
