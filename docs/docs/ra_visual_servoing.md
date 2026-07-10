# RA Use Case — Visual Servoing

Inside the [pick-and-place pipeline](ra_pick_and_place.md), the arm doesn't rely
on a single open-loop grasp pose. After the gross move it switches to
**image-based visual servoing (IBVS)** using the eye-in-hand `arm_cam`, closing
the loop on the mug until the grasp geometry is right.

## Why servo

The mug pose from the overhead detection is only approximate, and the SO-ARM 101
gripper is a single-jaw claw with a narrow capture gap. Servoing on the arm cam
corrects the last-centimetre error so the mug lands in the jaw gap reliably.

## Enabling it

Visual servoing is on by default via a launch arg:

| Arg | Default | Purpose |
|-----|---------|---------|
| `servo_image_mode` | `true` | image-based (IBVS) arm-cam servo (vs. open-loop pose) |
| `grasp_yaw_bias` | `-0.5` | approach angle so the mug lands in the single-jaw gap |
| `servo_grasp_z` | `0.05986` | side-grasp height (mug mid-height) |

```bash
ros2 launch mtc_tutorial bringup.launch.py servo_image_mode:=true servo_grasp_z:=0.05
```

## Tuning tips

- **Mug slips out of the jaw** → adjust `grasp_yaw_bias` so the approach lands
  the rim in the single-jaw gap.
- **Grasps too high / too low on the mug** → tune `servo_grasp_z` toward the
  mug's mid-height.
- **Servo never converges** → confirm the `arm_cam` RGB (and depth) topics are
  streaming and mapped to `camera_eih_ns` (`arm_cam`); the servo loop needs a
  live eye-in-hand feed.

## Where it fits

```
gross move → [ VISUAL SERVO (arm_cam, IBVS) ] → close claw → carry → fill → place
```

See the full sequence in [Pick and Place](ra_pick_and_place.md) and the topic
contract in [Core Concepts](ra_concepts.md#the-ros-contract-isaac-sim).
