# JetRacer Calibration Guide

Everything on this robot that needs to be measured/tuned rather than assumed.
Many values shipped are **placeholders or estimates** — they work well enough to
bring the stack up, but docking precision, the Smac S-curve, and localization
all degrade until the real numbers are in.

Legend for **Status**:

- 🔴 **Placeholder** — fake value, will visibly hurt accuracy. Do first.
- 🟡 **Estimate** — plausible guess, refine on hardware.
- 🟢 **Auto / OK** — self-calibrates at runtime or already measured.

Priority order (most impact first): **1 → 2 → 3 → 4 → 5 → rest.**

---

## 1. Camera intrinsics 🔴 (do first — gates all docking accuracy)

- **Files:** [`imx219_measured.yaml`][measured] (real oST calibration) and
  [`imx219_inferred.yaml`][inferred] (rough, spec-derived).
- **Selected by:** the `camera_info_url` line in
  [`hardware.launch.py`][hardware-launch].

!!! danger "A good calibration exists but is not the one being loaded"
    `imx219_measured.yaml` holds a **real** `camera_calibration` result
    (`fx ≈ 521.7`, `fy ≈ 525.5`, real plumb_bob distortion). But
    `hardware.launch.py` currently points `camera_info_url` at
    **`imx219_inferred.yaml`** — the spec-inferred file whose own header warns:

    > *plumb_bob cannot model this lens's heavy barrel distortion, so rectified
    > images stay visibly distorted and AprilTag pose accuracy is degraded,
    > especially toward frame edges. Use imx219_measured.yaml for accurate
    > docking.*

    So docking is running on `fx = fy = 133` with **zero distortion** while a
    measured calibration sits unused next to it. **Switching that one line is
    the single highest-value fix on this page** — verify on hardware before
    trusting it, since the measured file's resolution must match the live 640×360
    pipeline output.

- **Why it matters:** AprilTag pose is back-projected through these intrinsics.
  Wrong intrinsics → wrong dock distance/angle → the docking controller aims at
  the wrong spot.
- **If you need to re-calibrate:**
  1. Bring up the camera at the **640×360** pipeline output (must match
     `image_width/height`).
  2. Collect frames with [`grab_frames.py`][grab-frames] (`--cols 8 --rows 6`,
     ~40 frames, near/far/tilted/corners).
  3. Run `camera_calibration` (or OpenCV `calibrateCamera`) on those frames.
  4. Paste real `camera_matrix`, `distortion_coefficients`, `projection_matrix`
     into the YAML you load.
- **Check:** put a tag at a known distance; `/detected_dock_pose` range should
  match the tape measure within ~1–2 cm.

## 2. Wheel odometry scale 🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — `ENCODER_SCALE = 0.001 * 10`
- **Why it matters:** EKF position dead-reckons from this forward velocity. Too
  small → odom thinks it travelled less than it did → planner/docking distances
  are off; AMCL has to fight it.
- **How:** mark a 1.0 m line. Drive the car straight along it. Read
  `/odometry/filtered` (or `/odom`) x-displacement.
  `new_scale = old_scale × (1.0 / measured_x)`. Repeat until a 1 m drive reads
  ~1.0 m.

## 3. Ackermann geometry: wheelbase, max steer, min turning radius 🟡

These appear in **three** places and should agree:

| Quantity | Where | Current (estimate) |
|---|---|---|
| Wheelbase `L` | [`ackermann_dock_filter.py`][ackermann], URDF `virtual_steering_joint` x=0.1 ×2 | 0.20 m |
| Max steer `δ_max` | [`ackermann_dock_filter.py`][ackermann] | 30° |
| Min turning radius `R_min` | [`jetracer_nav2.yaml`][nav2-yaml] Smac `minimum_turning_radius` | 0.40 m (0.35 + margin) |

- **Why it matters:** `R_min` drives the Smac Hybrid-A* S-curve. Too small →
  planner draws curves the car physically can't hold → it cuts corners / drifts
  wide. `L` and `δ_max` set the ackermann filter's ω clamp on the final docking
  approach.
- **How (the one real measurement):** drive at **full steering lock**, low
  constant throttle, ≥1 full circle. Measure circle **diameter ÷ 2 = R_min**. Do
  **both directions**, keep the larger. Then derive `δ_max = atan(L / R_min)`.
- **Then:** set `minimum_turning_radius` ≈ R_min × 1.1, update
  `wheelbase`/`delta_max_deg` in the filter, sanity-check URDF.

## 4. Lidar mounting TF 🔴 (conflicting — fix the duplication)

- **Conflict:** two different TFs for the same sensor:
    - URDF [`jetracer.urdf`][urdf]: child `laser`, `xyz 0.05 0 0.09`, no yaw.
    - [`start_lidar.sh`][start-lidar]: child `laser_frame`, `xyz 0 0 0.18`,
      **yaw=π** (inverted mount).
    - Driver publishes scans on `frame_id:=laser_frame`.
- **Why it matters:** the scan frame is `laser_frame`, so the URDF `laser` link
  is unused/dead, and the real transform is the hand-typed one in the shell
  script (height + 180° flip are both guesses). A wrong yaw rotates the whole
  scan → SLAM/AMCL see walls in the wrong place.
- **How:**
  1. Pick **one** source of truth (recommend the URDF; rename its link to
     `laser_frame` and delete the static publisher in `start_lidar.sh`).
  2. Measure lidar **height** above the floor and its **x/y** offset from
     `base_footprint`.
  3. Confirm the **yaw**: drive toward a flat wall, view `/scan` in RViz; the
     wall should appear ahead, not behind. The π flip is because the A1 is
     mounted upside-down on the Waveshare chassis — verify it's actually needed.

## 5. Camera extrinsic (mount pose) 🟡

- **File:** URDF [`jetracer.urdf`][urdf] — `camera_link` at `xyz 0.12 0 0.07`,
  `rpy 0 0.25 0` (~14° downward pitch).
- **Why it matters:** the AprilTag is detected in `camera_optical_frame`; this
  transform places the dock in `base_footprint`/`odom`. Wrong pitch/offset → the
  car aims slightly above/below or beside the real dock.
- **How:** measure camera x/y/z from base, and its downward tilt (protractor, or
  detect a tag at a known position and back out the angle). Update the joint
  `origin`.

## 6. AprilTag detection 🟡

- **File:** [`dock_tags_36h11.yaml`][dock-tags]
- **Tag size:** `0.188 m` edge — **measure the actual printed black-square edge**
  and match it (`size` and per-tag `sizes`). Wrong size scales range error
  linearly.
- **Tune if needed:** `decimate: 2.0` (lower = detect smaller/farther tags, costs
  CPU), `refine`, `sharpening`.

## 7. Dock detection offsets + staging 🟡

- **File:** [`jetracer_docking.yaml`][docking-yaml]
- **`external_detection_translation_x: -0.20`**, `rotation_yaw/pitch/roll` —
  convert tag-optical pose into the dock approach frame. Marked "tune on
  hardware."
- **`staging_x_offset: -0.7`** — where Smac drives to before handing off to the
  docking controller. Sets where the S-curve ends.
- **`docking_threshold: 0.15`** — distance at which "docked" is declared.
- **Dock poses** `dock0/1/2` are all `[0,0,0]` placeholders — **survey each tag's
  real pose in the map** and fill in.
- **How:** drive a manual dock, watch where the car stops vs. the tag; nudge
  `translation_x` until it stops centered at the desired standoff.

## 8. Costmap footprint 🟡

- **File:** [`jetracer_nav2.yaml`][nav2-yaml] —
  `footprint: [[0.17,0.09],[0.17,-0.09],[-0.17,-0.09],[-0.17,0.09]]`
  (~0.34×0.18 m, assumed centered on `base_footprint`).
- **Why:** Smac plans collision-free using this. If `base_footprint` is at the
  rear axle (check URDF), shift the rectangle forward instead of keeping it
  symmetric.
- **How:** measure chassis length/width and the origin offset; update both
  costmap footprints.

## 9. Straight-line yaw trim 🟢🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — `YAW_TRIM = 0.0145` (only applied
  while driving forward).
- **How:** command pure forward, watch for drift; adjust in ~0.005 steps until it
  tracks straight. Mechanical (toe/alignment) — re-check if the steering linkage
  changes.

## 10. Gyro bias 🟢 (auto, but respect the procedure)

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — averages 100 samples (~2 s) at
  startup.
- **Action required:** keep the robot **perfectly still** for the first ~2 s
  after launching the driver, or yaw will drift for the whole session. Watch the
  log for "Gyro calibrated."

## 11. IMU covariances 🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — hard-coded `0.01`. Only matters if
  the EKF over/under-trusts the gyro; tune only if heading is noisy or sluggish.

## 12. cmd_vel → speed scaling 🟡 (verify, may be fine)

- **Path:** [`cmd_vel_to_serial.py`][cmd-vel] packs `linear.x` as mm/s to
  firmware.
- **Check:** command a known `linear.x` (e.g. 0.2 m/s) for a timed run; measured
  speed should match. If commanded vs. actual differ, the controller's speed
  assumptions (and the ackermann ω clamp) are off. Encoder odom (item 2) partly
  covers this, but open-loop command scaling still matters for the firmware.

## 13. Localization (AMCL) — note, not a calibration 🟡

- **File:** [`jetracer_nav2.yaml`][nav2-yaml] —
  `robot_model_type: DifferentialMotionModel` on a car. Acceptable because
  heading comes from the gyro-fused EKF, but be aware it's not an exact ackermann
  motion model. Revisit only if AMCL convergence is poor.

---

## Suggested bring-up sequence

1. **Camera intrinsics** (#1) — start by checking *which* YAML is actually
   loaded; a measured one may already exist.
2. **Wheel odom scale** (#2) + **min turning radius** (#3) — needed for both EKF
   and the Smac S-curve.
3. **Lidar TF** (#4) — resolve the duplicate, verify in RViz against a wall.
4. **Camera extrinsic** (#5) + **tag size** (#6) — get `/detected_dock_pose`
   matching tape measure.
5. **Dock offsets / staging / dock poses** (#7) — close the docking loop.
6. Footprint, yaw trim, covariances, cmd_vel scaling — refine as issues show up.

[measured]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/imx219_measured.yaml
[inferred]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/imx219_inferred.yaml
[hardware-launch]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/launch/hardware.launch.py
[grab-frames]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/grab_frames.py
[cmd-vel]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_driver/jetracer_driver/cmd_vel_to_serial.py
[ackermann]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/scripts/ackermann_dock_filter.py
[nav2-yaml]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/jetracer_nav2.yaml
[urdf]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_description/urdf/jetracer.urdf
[start-lidar]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/start_lidar.sh
[dock-tags]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/dock_tags_36h11.yaml
[docking-yaml]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/jetracer_docking.yaml
