# jetracer_ws third-party setup

`src/` mixes **first-party packages** (committed to this repo) with **third-party
ROS 2 Humble packages** that are *not* committed. The third-party source is
restored from pinned upstream releases with [vcstool], keeping the repo small
while remaining byte-reproducible.

## Fresh-machine setup

```bash
cd amr/jetracer_ws

# 1. Restore the tag-pinned third-party packages  (needs: sudo apt install python3-vcstool)
vcs import src < thirdparty.repos

# 2. Re-apply local patches to the restored source
patch -p1 -d src/navigation2        < patches/nav2_util-bond-shared_ptr.patch
patch -p1 -d src/robot_localization < patches/robot_localization-ekf-jetracer-tune.patch

# 3. Build
colcon build --symlink-install
```

## What lives where

**Committed in git**
- First-party: `jetracer_bringup`, `jetracer_description`, `jetracer_driver`
- Third-party with no upstream release tag (tag pin unreliable, so vendored):
  `gscam2`, `m-explore-ros2`, `nav2_graceful_controller`,
  `navigation_msgs` (map_msgs)
- Third-party where a tag pin would **regress the robot**, so vendored:
  - `apriltag` — working copy is past `v3.4.5` (adds aruco + tag-pose estimation)
  - `rplidar_ros` — ROS 2 branch with C1 support; the upstream tags are ROS 1

**Restored via `thirdparty.repos`** (each pinned to a release tag and
byte-verified against it; see that file for exact URLs/versions):
`navigation2` 1.1.5, `robot_localization` 3.5.4, `angles`, `apriltag_msgs`,
`BehaviorTree.CPP`, `bond_core`, `diagnostics`, `geographic_info`,
`laser_geometry`, `teleop_twist_keyboard`.

## Local patches

Both were confirmed real by byte-diffing the working tree against the pinned
upstream tag:

- `patches/nav2_util-bond-shared_ptr.patch` — nav2 1.1.5's only local change:
  nav2_util's `bond_` member `unique_ptr` -> `shared_ptr`.
- `patches/robot_localization-ekf-jetracer-tune.patch` — robot_localization
  3.5.4's `params/ekf.yaml`: `frequency` 30 -> 20 Hz and `base_link_frame`
  -> `base_footprint`.

Stray IDE "optimize imports" edits (absolute import paths in
`nav2_smac_planner/lattice_primitives`, `bond_core/bondpy`,
`geographic_info/geodesy`) were **not** real changes and are intentionally
dropped — a fresh `vcs import` restores the correct upstream imports.

[vcstool]: https://github.com/dirk-thomas/vcstool
