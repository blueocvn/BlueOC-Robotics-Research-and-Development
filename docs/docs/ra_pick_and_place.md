# RA Use Case — Pick and Place (Refill)

The end-to-end demo: the arm detects an empty mug, grasps it, carries it to the
dispenser to "fill", and places it in the tray. This page walks the scenario;
for install/build see the [Setup Guide](ra_setup.md).

## Prerequisites

- Workspace built and Isaac Sim running with the ROS contract verified — see
  [Setup Guide §1–5](ra_setup.md).
- The arm scene open in Isaac Sim and **Play** pressed.

## Run it

One command brings up MoveIt (`move_group` + controllers + RViz), perception,
and `mtc_node`, staggered so each layer's dependencies come up first:

```bash
source install/setup.bash
ros2 launch mtc_tutorial bringup.launch.py
```

## What happens

1. `move_group`, `ros2_control`, and the `arm_group` / `hand_group` controllers
   start.
2. Perception starts — YOLO (`yolo11n`) cup detector, pink-tray detector,
   AprilTag dispenser detector.
3. `mtc_node` waits for `/detected_object/position`, then **per cup**:

    | Step | Action |
    |------|--------|
    | Gross move | Approach toward the detected cup |
    | Visual servo | Fine-align onto the mug with the arm cam — see [Visual Servoing](ra_visual_servoing.md) |
    | Grasp | Close the claw |
    | Carry | Move to the AprilTag dispenser |
    | Fill | Lean / press to "fill" |
    | Place | Set the cup down in the tray |

!!! info "How many cups?"
    Not configured directly — it's however many the **top cam** detects at
    start (defaults to one if none), spread evenly across the tray. A single cup
    is centred.

## Tuning

The grasp/fill geometry is controlled by launch args (defaults shown) — full
table in [Setup Guide §7](ra_setup.md):

```bash
ros2 launch mtc_tutorial bringup.launch.py \
    grasp_yaw_bias:=-0.5 \
    dispenser_standoff:=0.10 \
    dispenser_fill_depth:=-0.08
```

## If it stalls

- **`mtc_node` waits on `/detected_object/position`** → perception isn't
  publishing; check the camera topics and `camera_*_ns` params.
- **Arm never moves** → Isaac not in Play, or not subscribed to
  `/isaac_joint_commands`.

More in [Setup Guide §8](ra_setup.md).
