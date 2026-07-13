# RA Use Case — Visual Servoing

Inside the [pick-and-place pipeline](ra_pick_and_place.md), the arm doesn't rely
on a single open-loop grasp pose. After the gross move it switches to
**image-based visual servoing (IBVS)** using the eye-in-hand `arm_cam`, closing
the loop on the mug until the grasp geometry is right.

## Objective

Close the **last-centimetre gap** between "roughly where the cup is" and "exactly
where the gripper must be" — using live camera feedback rather than trusting the
open-loop pose.

## Why servo

The mug pose from the overhead detection is only approximate, and the SO-ARM 101
gripper is a single-jaw claw with a narrow capture gap. Servoing on the arm cam
corrects the last-centimetre error so the mug lands in the jaw gap reliably.

Concretely, the open-loop pose carries three stacked errors:

| Error source | Effect |
|--------------|--------|
| Perception | residual azimuth/position error in the overhead detection |
| Calibration | camera-to-world extrinsic is never perfect (and on real hardware, much worse) |
| Kinematics | position-only IK on a 5-DOF arm leaves small pose residuals |

Individually small; together, enough to miss a narrow jaw gap. Servoing removes
them by **measuring the outcome instead of predicting it**.

## Why not MoveIt Servo

!!! warning "The arm deliberately does **not** use `moveit_servo`"
    `moveit_servo` implements **6-DOF Cartesian** servoing. The SO-ARM 101 has
    **5 DOF**, so a full 6-DOF Cartesian command is not achievable — the
    Jacobian degenerates and the servo **halts at a singularity** instead of
    converging. An earlier attempt to adopt it failed for exactly this reason.

    Instead, `mtc_node` runs a **custom servo loop**: it computes IK via
    `/compute_ik` and publishes a `JointTrajectory` directly to
    `/arm_group_controller/joint_trajectory`.

    A `servo_node` used to be started by the launch file but published nothing
    useful — it has since been **removed**. The `moveit_servo` package is not a
    dependency of this project.

This is the same pattern used by [XLeRobot on the SO-101](https://xlerobot.readthedocs.io/en/latest/software/getting_started/SO101.html),
which pairs closed-form kinematics with an image-based servo loop rather than a
generic Cartesian servo — independent corroboration that the custom loop is the
right call on this arm, not a workaround.

## How it works — two phases

The loop is split so that **rotation and translation never fight each other**:

```
gross move → [ phase 0: centre on pixels ] → [ phase 1: straight-in approach ] → close claw
```

| Phase | Controls | Error signal | Why separate |
|-------|----------|--------------|--------------|
| **0 — centre** | base bearing (yaw) | horizontal pixel error `dx` on `arm_cam` | Get the cup **left/right centred** first, while held back at a standoff. Rotating and advancing at once makes the approach curve and the cup drift out of frame. |
| **1 — approach** | Cartesian position | world distance along the approach line | Once centred, drive **straight in** along the jaw axis at constant speed. A straight line is predictable and keeps the cup in the gap. |

Three details make phase 0 behave:

- **Deadband** — ignore pixel errors below a threshold, so the arm parks instead
  of endlessly nudging.
- **Taper** — the per-tick yaw step shrinks as the cup nears centre, so the base
  *slows* into alignment rather than overshooting.
- **Anti-windup** — the bearing integrator is scaled back when the arm is lagging
  its command, so the setpoint can't wind ahead of the physical arm and cause an
  overshoot.

## IBVS vs PBVS — where each is used

Both are used, deliberately, for different jobs.

| | **IBVS** (image-based) | **PBVS** (position-based) |
|---|---|---|
| Error lives in | **pixels** | **3D world pose** |
| Used for | the **cup final approach** | the **AprilTag dispenser** / base centring |
| Needs good calibration? | **No — self-correcting.** It converges to "cup centred in view" regardless of exact extrinsics | **Yes — very sensitive.** Any extrinsic error shifts the target, and the arm confidently moves to the *wrong* place |
| Gives absolute metric pose? | No | Yes |

!!! tip "Why the split matters for sim-to-real"
    In simulation, every camera intrinsic and extrinsic is **exact and free**.
    On real hardware they must be calibrated, and residual error is unavoidable.

    **PBVS degrades with calibration error; IBVS does not.** Using IBVS for the
    precision-critical final grasp is therefore the choice that transfers best to
    the real arm — PBVS is reserved for the coarse, fiducial-anchored moves where
    absolute pose is genuinely needed.

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

## Challenges & limitations

??? warning "Gains are hand-tuned, and the signs are mount-dependent"
    `servo_img_k_yaw` encodes radians of base bearing per pixel of error — and
    **its sign depends on how the camera is mounted**. Get it wrong and the arm
    drives the cup *off* frame instead of centring it. The taper, deadband, and
    anti-windup constants are likewise empirical. This tuning burden is the
    single biggest weakness of the approach.

??? warning "No timeout or bail-out"
    If IK is unreachable, the loop can **spin indefinitely** instead of failing
    cleanly. There is no servo timeout, no retry limit, and no give-up path.
    This must be fixed before it runs on real hardware.

??? warning "No contact sensing"
    The servo stops on **kinematic arrival**, not on touching the cup. It cannot
    detect that it has bumped, missed, or knocked the cup over.

??? warning "Hand-eye extrinsic is a placeholder"
    The gripper → `arm_cam` transform is currently a nominal `eih_z = 0.05`, not
    a calibrated value. IBVS tolerates this (that is the point), but **phase 1's
    metric approach distance does not** — expect this to need real calibration.

??? warning "Tuned against simulated imagery"
    The detector feeding the servo is sim-tuned. Real lighting, motion blur, and
    a real cup will change the pixel signal the loop consumes.

## Future direction

1. **Safety rails first** — a servo **timeout** and a failure exit, plus a
   force/current-based contact stop. These are prerequisites for touching real
   hardware, not optional polish.
2. **Hand-eye calibration** — replace the placeholder eye-in-hand extrinsic with
   a measured one; this is what phase 1's metric approach depends on.
3. **Depth robustness** — median-filter the depth patch and reject invalid
   readings before they reach the loop.
4. **Replace hand-tuned gains with a learned policy.** The servo+grasp stage is
   short-horizon and contact-rich — exactly what imitation learning is good at,
   and exactly where the manual tuning burden lives. Collect keyboard-teleop
   demonstrations in Isaac Lab (LeIsaac) and train a policy for the *grasp only*,
   keeping scripted planning for transport and the fiducial-anchored dispenser.

    !!! note "Why hybrid, not end-to-end"
        The dispenser is AprilTag-marked and the geometry is known, so classical
        planning is **more accurate and more debuggable** than a learned policy
        there. Learning is worth it precisely where hand-tuning dominates — the
        grasp — and nowhere else.
