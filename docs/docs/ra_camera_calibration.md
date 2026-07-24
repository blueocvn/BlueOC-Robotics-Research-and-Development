# Robotic Arm — Camera Calibration

The arm uses **two** cameras, and only one of them is *eye-to-hand*:

| Camera | Mounting | Frame | Role |
|--------|----------|-------|------|
| `top_cam` | **fixed overhead**, watching the table | `top_sim_camera` | **eye-to-hand** — localizes the cup in the world for the gross reach |
| `arm_cam` | on the gripper | `arm_cam` | eye-in-hand — the visual-servo close-in |

This page covers the **eye-to-hand `top_cam`** calibration. It is the current gating
task for perception-driven grasping (see the note in
[Real-Hardware Bringup](ra_hardware_bringup.md#deterministic-demo-mode-predefined-positions));
until it is tight, run the deterministic demo with `fake_object:=true`.

Calibration is **two stages, in order** — they use *different* targets:

1. **Intrinsics** — a **checkerboard**, standard OpenCV calibration. Produces
   `fx, fy, cx, cy` and the distortion coefficients.
2. **Extrinsic** (the `world → top_sim_camera` transform) — a **single AprilTag** at a
   measured world pose. Consumes the intrinsics from stage 1, so intrinsics **must**
   be done first.

> **⚠️ Python:** run the ROS calibration tools under the **system** interpreter
> (`/usr/bin/python3`) with ROS sourced and **no conda env active** — conda's Python
> can't import `rclpy`.

---

## Stage 1 — Intrinsics (checkerboard)

**Goal:** the real `fx/fy/cx/cy` + lens distortion for `top_cam`, so pixels
back-project to correct rays.

**Print a checkerboard.** Note its **interior-corner** count (a board with 9×7
*squares* has an **8×6** interior grid) and the **square edge length** in metres.
Mount it rigidly to something flat.

**1. Bring up just the camera** (its own terminal — this does not touch the arm's
serial bus):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash

# top_cam device link (see real_all.launch.py); camera_ns fixes the topics.
ros2 run so_arm_perception usb_camera_node --ros-args \
  -p video_device:=/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0 \
  -p camera_ns:=top_cam
```

**2. Run the OpenCV calibrator** against `/top_cam/rgb` (adjust `--size` to your
board's interior corners and `--square` to its edge length in metres):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/top_cam/rgb
```

Move the board through the view — near/far, tilted, all four corners — until **X, Y,
Size, Skew** fill up, then click **CALIBRATE**, then **SAVE**. The result lands in
`/tmp/calibrationdata.tar.gz`; open its `ost.yaml` for `camera_matrix` and
`distortion_coefficients` (plumb_bob order `k1, k2, p1, p2, k3`).

**3. Paste the numbers into the `top_cam` node params** in
`ra_ws/src/mtc_tutorial/launch/real_all.launch.py` (the `top_cam` block) and set
`undistort:=true` so the node rectifies and publishes a distortion-free `camera_info`:

```python
parameters=[{
    "video_device": TOP_CAM_DEV, "camera_ns": "top_cam", "frame_id": "top_sim_camera",
    "fx": 418.38762, "fy": 416.14640, "cx": 325.19068, "cy": 233.94865,   # <- from ost.yaml
    "undistort": True,
    "d0": -0.300890, "d1": 0.078304, "d2": 0.001265, "d3": -0.001828, "d4": 0.0,  # k1,k2,p1,p2,k3
}]
```

The parameter names (`fx/fy/cx/cy`, `d0..d4`, `undistort`) are defined and explained in
`ra_ws/src/so_arm_perception/so_arm_perception/usb_camera_node.py`.

!!! tip "Check the intrinsics before moving on"
    With `undistort:=true`, straight edges (the table lip, a ruler) should look
    straight across the **whole** frame, including the corners. Residual bowing means
    redo Stage 1 — a bad intrinsic silently corrupts the extrinsic that depends on it.

---

## Stage 2 — Extrinsic (AprilTag): `world → top_sim_camera`

**Goal:** where the camera sits in the robot-base (`world`) frame, so a detected pixel
ray meets the table at the right world point. This uses the helper
`ra_ws/src/so_arm_perception/scripts/calibrate_top_cam_extrinsics.py`, which solves it
from **one AprilTag lying flat at a known world pose** and reuses the Stage-1
intrinsics off `/top_cam/camera_info`.

!!! note "Why an AprilTag here, not the checkerboard"
    Intrinsics need many views of a dense corner grid — a checkerboard is ideal. The
    extrinsic needs a single target at a **known world pose**; a flat AprilTag with a
    measured centre gives a full 6-DoF pose from one detection, and its frame
    convention is unambiguous. The solver also handles the OpenCV→Isaac optical-frame
    flip that a naive `solvePnP` extrinsic gets mirrored.

**1.** Place a `tag36h11` AprilTag **flat on the table, face up**, top edge along
world **+X**, right edge along world **+Y**. Measure its **centre** in the base frame
`(tag_x, tag_y, tag_z)` and its printed black-square **edge** `tag_size`, all in metres.

**2.** With the `top_cam` node from Stage 1 still publishing, run the solver
(system Python, ROS sourced):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash

/usr/bin/python3 \
  ~/Desktop/robotics-arm/robotic-arm/ra_ws/src/so_arm_perception/scripts/calibrate_top_cam_extrinsics.py \
  --ros-args -p tag_x:=0.15 -p tag_y:=0.0 -p tag_z:=0.0 -p tag_size:=0.05
```

It averages ~30 detections and prints the transform. If it warns *"camera z is BELOW
the table"*, re-run with `-p tag_z_up:=false`. If you couldn't square the tag to the
axes, pass `-p tag_yaw_deg:=<deg>`. Watch the printed **position stddev** — a few mm is
good; centimetres means glare, motion, or a wrong `tag_size`.

**3.** Paste the printed `--x/--y/--z/--roll/--pitch/--yaw` into `eth_static_tf` in
`ra_ws/src/so_arm_perception/launch/perception.launch.py` (the `eth_x … eth_yaw`
defaults). `real_all.launch.py` also forwards `eth_x…eth_yaw` as launch args, so you
can test a fresh solve without editing the file:

```bash
ros2 launch mtc_tutorial real_all.launch.py \
  eth_x:=0.02967 eth_y:=0.31201 eth_z:=0.87595 \
  eth_roll:=0.6617028 eth_pitch:=0.0 eth_yaw:=3.1415926536
```

---

## Verify end-to-end

With both stages applied and perception running, place the cup at a **tape-measured**
world position and read the detection:

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /detected_object/position
```

The reported `x, y` should match the tape within ~1–2 cm. A constant offset can be
trimmed with the `eth_x_correction` / `eth_y_correction` launch args (in
`ra_ws/src/mtc_tutorial/launch/real_all.launch.py`) rather than re-solving; a *scale*
or *rotation* error means Stage 2 (or the Stage-1 intrinsics) needs redoing.

Once `/detected_object/position` tracks reality, drop `fake_object:=true` /
`skip_servo:=true` from the bringup command to run the real perception-driven pick.
