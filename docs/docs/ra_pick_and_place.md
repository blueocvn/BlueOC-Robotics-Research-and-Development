# RA Use Case — Pick and Place (Refill)

The end-to-end demo: the arm detects an empty mug, grasps it, carries it to the
dispenser to "fill", and places it in the tray. This page walks the scenario;
for install/build see the [Setup Guide](ra_setup.md).

## Objective

Autonomously fulfil a drink order end to end: **detect → grasp → fill → place**,
repeated for every cup the overhead camera sees, with no human in the loop.

The arm is the **RA half** of the wider [Pick and Deliver
solution](solution_pick_and_deliver.md) — it prepares filled cups onto a tray
that the AMR (JetRacer) then carries to a table.

The deliberate constraint: achieve this on **low-cost 5-DOF hardware**
(SO-ARM 101, single-jaw gripper) rather than a 6/7-DOF industrial arm. Nearly
every design decision below follows from that constraint.

**Success criteria**

| Criterion | Target |
|-----------|--------|
| Grasp | Every detected cup is picked without knocking it over |
| Fill | Cup reaches the dispenser paddle at the correct depth |
| Place | Cup lands in its assigned tray slot, no collision with placed cups |
| Loop | Repeats cleanly across N cups without re-grabbing a placed cup |

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

## Proposed solution — and why

Each stage was chosen to work *around* the 5-DOF, single-jaw hardware rather
than fight it.

| Decision | Why |
|----------|-----|
| **Side grasp, not top-down** | The gripper jaw is ~0.17 m long. Pointing it straight down at a short cup drives the jaw into the **table** before the cup is captured. Top-down is not a tuning problem — it is **geometrically impossible** on this arm. A level side grasp is the reachable one. |
| **Position-only IK** | With 5 DOF you cannot command an arbitrary position *and* orientation. Solving for position only, and fixing the wrist by construction (`Wrist_Roll = −90°`, `Wrist_Pitch = −(Pitch + Elbow)`), keeps the gripper level and the IK solvable. |
| **Angled approach (`grasp_yaw_bias`)** | The SO-ARM 101 pins the cup against a **fixed** jaw. Approaching dead-centre swats the cup with that jaw. Facing ~29° off-axis (`-0.5 rad`) lands the cup **in the gap** between the jaws. |
| **Overhead ray-plane unprojection** | Unprojecting the depth buffer on an oblique surface carries a systematic bias. Intersecting the detection-pixel ray with the known cup-height plane instead cut top-cam error from **≈31 mm → ≈3 mm**. |
| **AprilTag on the dispenser** | The fill target must be exact. A fiducial gives sub-millimetre pose from the overhead camera — far more reliable than detecting the dispenser visually. |
| **MoveIt Task Constructor (MTC)** | The task is naturally staged (approach → grasp → lift → carry → place). MTC expresses that as composable stages with collision-aware planning (OMPL / RRTConnect), instead of one monolithic script. |
| **Visual servo before the grasp** | Planning gets the gripper *close*, but open-loop pose carries perception + calibration error. Closing the loop on the arm cam corrects the final centimetres — see [Visual Servoing](ra_visual_servoing.md). |
| **Tray slot assignment + exclusion** | Cups are spread evenly across the tray, and placed cups are excluded from detection so the arm doesn't pick up a cup it just put down. |
| **Sim-first (Isaac + `topic_based_ros2_control`)** | The same ROS topic contract a real arm will use, with zero hardware risk and a reproducible scene. |

!!! tip "The through-line"
    Perception gives an approximate world pose → planning gets close →
    **closed-loop vision corrects the last step** → the gripper geometry
    (angle + height) does the rest. Each layer is independently debuggable.

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

## Challenges & limitations

Known constraints, in the order they are likely to bite a new developer.

??? warning "Hardware — 5-DOF underactuation"
    Only position can be commanded, not full orientation. Grasp poses are
    constrained to the level side-grasp, and **top-down grasping is impossible**
    (jaw length vs cup height). Any new manipulation idea must first be checked
    for reachability, not just planned.

??? warning "Gripper — single fixed jaw"
    The cup is pinned against a fixed jaw rather than squeezed by two moving
    ones. This makes capture sensitive to approach angle (`grasp_yaw_bias`) and
    is the single fiddliest part of the pipeline to tune.

??? warning "Grasp reliability — intermittent failures"
    The pipeline **runs end to end in simulation**, but the **grasp does not
    succeed every time** — the gripper occasionally fails to capture the cup.
    The root cause is not yet isolated. Likely suspects, in order:

    - **Approach angle** — `grasp_yaw_bias` aims the cup into the jaw gap. If the
      cup is slightly off-bearing, the *fixed* jaw swats it instead of capturing it.
    - **Grasp height** — `servo_grasp_z` sits at mug mid-height; drift here changes
      where the jaw meets the cup wall.
    - **Servo hand-off** — if phase 0 hands over before it is properly centred,
      phase 1 drives straight in slightly off-axis.
    - **No contact feedback** — the claw closes on kinematics alone, so a missed
      grasp is neither detected nor retried.

    Until this is characterised, treat grasp success as **probabilistic, not
    guaranteed**. See [Visual Servoing](ra_visual_servoing.md) for the tuning knobs.

??? warning "Calibration — an unexplained bias"
    A systematic place **x-offset** is currently cancelled by a hard-coded
    `+0.044 m` constant. The root cause is not yet identified — treat this as a
    known smell, not a solved problem.

??? warning "Perception — tuned for simulation"
    HSV thresholds and the COCO-pretrained YOLO weights are **sim-tuned**. Real
    lighting and real cups will shift both. The tray fix also drifts when the
    arm occludes the overhead camera mid-run.

??? warning "No force or contact feedback"
    The grasp closes on **kinematics alone** — there is no current/force sensing
    to detect contact or a failed grasp. On real hardware this is a safety gap
    as much as a reliability one.

??? warning "\"Fill\" is a gesture, not a dispense"
    The arm leans/presses against the tagged paddle. There is no liquid
    simulation, flow sensing, or spill handling.

??? warning "Not yet integrated with the wider system"
    The arm runs standalone. The orchestrator ↔ RA task topics and the AMR ↔ RA
    cup handoff are **designed but not wired**.

## Future direction

Ordered by what unblocks the most.

1. **Diagnose the intermittent grasp failure** — the pipeline already completes,
   so *reliability* is the gap between it and a dependable demo. Instrument the
   servo hand-off (log the pixel error `dx` at the phase 0 → 1 transition) and
   record grasp outcomes, so the failure mode is **characterised rather than
   guessed at**. A contact/force stop (item 2) would also let a failed grasp be
   detected and retried instead of silently continuing.
2. **Sim-to-real bring-up** — real SO-ARM 101 driver, **hand-eye calibration**
   (the eye-in-hand extrinsic is still a placeholder), force/current-based grasp
   stop, and a servo timeout so the loop bails instead of spinning on an
   unreachable IK target.
3. **System integration** — expose the RA task topics and implement the AMR ↔ RA
   handoff so the arm becomes a node in the [Pick and Deliver
   solution](solution_pick_and_deliver.md).
4. **Handle reorientation** — a handle detector already exists but is not wired
   in. Because the arm cannot grasp top-down, reorienting a mug by its handle
   must be a **push-to-rotate** manoeuvre rather than a regrasp.
5. **Learned grasping (hybrid)** — the transport and dispenser stages have exact
   fiducial-based geometry and are best left scripted. The **grasp** is the
   hand-tuned, contact-rich stage, and is the natural candidate to replace with
   an imitation-learning policy (keyboard-teleop demos in Isaac Lab / LeIsaac).
6. **Robustness** — domain randomisation for perception; generalise beyond a
   single cup colour and size; localise the remote Isaac assets so the scene
   loads offline.
