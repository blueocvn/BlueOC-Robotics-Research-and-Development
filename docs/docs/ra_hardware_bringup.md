# Robotic Arm — Real-Hardware Bringup

Running the SO-ARM 101 pick-and-place stack on the **physical arm** instead of
Isaac Sim. This is the **sim-to-real** path: the same MoveIt / MTC pipeline and
perception nodes, but driving real Feetech servos over USB and reading real USB
cameras. For the simulation workflow see the [Setup Guide](ra_setup.md).

> **⚠️ ROS distro:** same as the sim stack — **ROS 2 Jazzy**, built in `ra_ws`.
> Not the Humble container the AMR uses.

## What's different from sim

| Layer | Sim | Real hardware |
|-------|-----|---------------|
| Joints | Isaac Sim via `topic_based_ros2_control` | **`feetech_ros2_driver`** — `ros2_control` hardware interface over serial |
| Follower port | — | `/dev/ttyACM0` |
| Cameras | Isaac render products | **`usb_camera_node`** → `/arm_cam/rgb` (+ overhead cam) |
| Joint wiring | sim controllers | `config/follower_joints.yaml`, `*.ros2_control.xacro`, `ros2_controllers.yaml` |
| Bringup | `bringup.launch.py` | **`real_all.launch.py`** / `bringup_real.launch.py` |

## Packages involved

- **`feetech_ros2_driver`** — the real servo hardware interface (vendored; see its
  `VENDORED.md` — it's a fork of `JafarAbdi/feetech_ros2_driver` with local edits).
- **`mtc_tutorial`** — `mtc_node` (the pick→fill→place pipeline) + `real_all.launch.py`.
- **`so_arm_perception`** — `usb_camera_node` + `top_cam_view.launch.py` for real cameras.
- **`so_arm_moveit_config`** — real controller/joint config.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ra_ws
colcon build --symlink-install
source install/setup.bash
```

## Bring it up

One launch starts MoveIt + controllers + the Feetech driver + perception + `mtc_node`:

```bash
ros2 launch mtc_tutorial real_all.launch.py
```

### Useful launch arguments

| Arg | Default | Purpose |
|-----|---------|---------|
| `run_sensing` | `true` | start the cameras + perception nodes |
| `fake_object` | `false` | publish a fixed `/detected_object/position` instead of using perception |
| `obj_x` / `obj_y` / `obj_z` | — | the fake object's world pose (front = **−Y**) |
| `skip_servo` | `false` | bypass the visual-servo bridge — open-loop straight-in grasp |
| `place_z` | (unset) | absolute release height override |
| `tag_x` / `tag_y` / `tag_z` | — | fake AprilTag ("dispenser paddle") pose |
| `bridge_standoff` | `0.08` | clearance from the cup at the grasp standoff |

## Deterministic demo mode (predefined positions)

!!! important "Why predefined positions"
    On real hardware the cameras are **not yet calibrated accurately enough** to
    localize the object reliably, so perception-driven grasping isn't dependable
    yet — **camera calibration is the current gating task** (see
    [Camera Calibration](ra_camera_calibration.md)). For a repeatable live demo, run
    open-loop with the object and destination **predefined**, which skips perception
    and the visual-servo bridge entirely.

### From a fresh terminal — step by step

**1. Preconditions (once per login / boot).**

```bash
# You must be in the dialout group IN THIS SESSION, not just /etc/group.
id -nG | grep -q dialout && echo "dialout OK" || echo "LOG OUT AND BACK IN"
# No conda env active (its Python 3.13 breaks a colcon build of so_arm_perception;
# ROS Jazzy builds against 3.12). 'conda deactivate' until this prints "(none)".
echo "conda: ${CONDA_DEFAULT_ENV:-none}"
```

If `dialout` is missing, **log out and back in** (a new terminal is not enough —
group membership is fixed when the desktop session starts). If the arm was just
plugged in, confirm it enumerated as `/dev/ttyACM0`.

**2. Source ROS, then the workspace overlay** (order matters):

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash
```

**3. Launch the deterministic pick → carry → place:**

```bash
ros2 launch mtc_tutorial real_all.launch.py \
  run_sensing:=false fake_object:=true \
  obj_x:=0.0 obj_y:=-0.37 obj_z:=0.09 \
  skip_servo:=true \
  fake_apriltag:=true tag_x:=0.0 tag_y:=-0.32 tag_z:=0.20 \
  place_z:=0.04 bridge_standoff:=0.08
```

Success looks like this ~10 s in (once `move_group` and the controllers are up):

```
[bridge] standoff joints R=0.013 P=0.993 E=0.629 WP=-1.620 WR=-1.571 (residual 0.00 mm)
==== [1] Planning SUCCEEDED — 1 solution(s) ====
==== [4] Executing ====
```

This gives a reliable pick → carry → place cycle with no dependence on camera
accuracy. Once calibration is tightened, drop `fake_object`/`skip_servo` to return
to the perception-driven pipeline.

!!! danger "Run only ONE instance"
    `/dev/ttyACM0` is a USB-CDC device, so a **second** launch (or a leftover one
    from a previous run) still *opens* the port and then collides with the first on
    the servo bus. The symptom is misleading — it looks like dead hardware:

    ```
    FeetechHardwareInterface … SerialPort::read_exact [Read timeout]
    Failed to initialize hardware 'FakeSystem'
    ```

    → 0/3 controllers → no `/joint_states` → **RViz frozen at the default pose while
    the real arm sits elsewhere** → execution fails with code `99999`. Before
    launching, make sure nothing else is up:
    `pgrep -af "ros2_control_node|move_group|mtc_node"`.

!!! note "Cup must be far enough out to grasp level"
    The pre-grasp standoff holds the gripper **horizontal** (`Wrist_Pitch =
    −(Pitch+Elbow)`, capped at ±1.658 rad). At grasp height that confines the
    gripper origin to roughly **0.22 – 0.35 m** from the base, so an 8 cm standoff
    needs the cup at about **`obj_y ≤ −0.36`**. Closer in (e.g. the old `−0.30`) the
    standoff is *inside* the reachable ring and planning aborts with a
    `GOAL_STATE_INVALID` / gripper-vs-cup collision. `mtc_node` reports this case
    explicitly: *"standoff … is OUTSIDE the level-wrist workspace"*. To grasp a
    nearer cup, raise `servo_grasp_z` or shrink `bridge_standoff`.

## Known gotcha — home the arm before the first plan

!!! warning "Start state out of bounds"
    MoveIt's `CheckStartStateBounds` adapter **rejects planning** if the arm starts
    outside its URDF joint limits. The folded rest pose can park a joint just past
    a limit (e.g. `Pitch ≈ −1.84` vs the `−1.745` limit), producing:

    ```
    Joint 'Pitch' from the starting state is outside bounds …
    PlanningRequestAdapter 'CheckStartStateBounds' failed … Aborting planning pipeline.
    ```

    MTC can't do the *first* move because the same check blocks it. **Home the arm
    to a valid pose first** — stream a trajectory straight to the controller
    (`/arm_group_controller/joint_trajectory`, e.g. all-zeros over ~4 s) before
    launching the pipeline, or add an auto-home step ahead of the first plan.

## Safety notes

- No current/force sensing — the gripper closes on **kinematics alone**; it can't
  detect a failed grasp or contact. Treat every grasp as open-loop.
- The arm moves autonomously once `mtc_node` starts — keep the workspace clear and
  power within reach.
- Serial port order (`/dev/ttyACM*`) isn't guaranteed across reboots — confirm the
  follower is on `/dev/ttyACM0` before launching.
