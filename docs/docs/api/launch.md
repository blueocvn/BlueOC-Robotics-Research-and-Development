# Launch Entry Points

Which launch file to run, and what each one assumes is already running. Getting
this wrong is the most common way to lose an hour at a hackathon.

## JetRacer

The AMR stack is deliberately split into **hardware** and **navigation** layers
so you can restart Nav2 without power-cycling the chassis.

| Launch file | Brings up | Assumes already running |
|---|---|---|
| `hardware.launch.py` | Driver, lidar, EKF, static TFs, CSI camera | Nothing |
| `nav_bringup.launch.py` | `map_server`, AMCL, controller, planner, `bt_navigator`, lifecycle manager | Hardware layer |
| `mapping_bringup.launch.py` | Autonomous mapping (no localization) | Hardware layer, **plus** a SLAM stack on another machine publishing `/map` and `map→odom` |
| `jetracer.launch.py` | Hardware **and** navigation together | Nothing |

```bash
# split: two terminals, restart nav without touching the chassis
ros2 launch jetracer_bringup hardware.launch.py
ros2 launch jetracer_bringup nav_bringup.launch.py map:=/path/to/map.yaml

# or all at once
ros2 launch jetracer_bringup jetracer.launch.py map:=/path/to/map.yaml
```

### Arguments

| Launch file | Arguments |
|---|---|
| `hardware.launch.py` | `ekf_params_file`, `base_port`, `lidar_port` |
| `nav_bringup.launch.py` | `map`, `params_file` |
| `mapping_bringup.launch.py` | `params_file`, `explore_params_file` |
| `jetracer.launch.py` | `map`, `params_file`, `ekf_params_file`, `base_port`, `lidar_port` |

!!! note "Serial ports are not stable across reboots"

    `base_port` and `lidar_port` default to fixed `/dev/tty*` paths, but Linux
    assigns those in enumeration order — plugging the lidar in first can swap
    them. If bringup fails with a serial error, check the actual device before
    debugging anything else:

    ```bash
    ls -l /dev/serial/by-id/
    ```

### CSI camera

`hardware.launch.py` starts `gscam2` with a default GStreamer pipeline for the
IMX219, overridable via `$GSCAM_CONFIG`.

!!! warning "gscam2 rejects `bgr8`"

    The pipeline must end in `format=RGB`. `gscam2` publishes `rgb8` and a BGR
    pipeline simply fails to link to its appsink — with an error that doesn't
    obviously point at the format. If you override `$GSCAM_CONFIG`, keep the RGB
    ending or unset it to use the built-in default.

## Robot Arm

| Launch file | Purpose |
|---|---|
| `mtc_tutorial/bringup.launch.py` | Arm bringup — simulation |
| `mtc_tutorial/bringup_real.launch.py` | Arm bringup — real hardware |
| `mtc_tutorial/real_all.launch.py` | Full real-hardware stack: bringup + perception + MTC |
| `mtc_tutorial/mtc_demo.launch.py` | MTC task demo |
| `mtc_tutorial/pick_place_demo.launch.py` | Pick-and-place demo |
| `so_arm_perception/perception.launch.py` | YOLO detection, unprojection, AprilTag, handle detector |
| `so_arm_perception/tracking.launch.py` | Visual-servoing tracker |
| `so_arm_perception/top_cam_view.launch.py` | Top camera viewer — use while calibrating |

```bash
# simulation
ros2 launch mtc_tutorial bringup.launch.py

# real hardware, everything at once
ros2 launch mtc_tutorial real_all.launch.py
```

### `real_all.launch.py` arguments

The full-stack entry point. Grouped by what they control.

**Stage toggles**

| Argument | Meaning |
|---|---|
| `run_mtc` | Start the MTC pipeline |
| `run_sensing` | Start the perception stack |
| `fake_apriltag` | Publish a synthetic tag pose instead of detecting one |
| `fake_object` | Publish a synthetic object instead of detecting one |
| `obj_x`, `obj_y`, `obj_z` | Position of the synthetic object |
| `tag_x`, `tag_y`, `tag_z` | Position of the synthetic tag |

**Grasp geometry**

| Argument | Meaning |
|---|---|
| `grasp_yaw_bias` | Yaw offset applied to the grasp |
| `bridge_standoff` | Standoff distance at the dispenser |
| `place_at_pickup` | Place the cup back where it was picked up |
| `place_z` | Place height |

**Calibration correction**

| Argument | Meaning |
|---|---|
| `eth_plane_z` | Ray–plane intersection height |
| `eth_x_correction`, `eth_y_correction` | Manual extrinsic nudges |

**Visual servoing**

| Argument | Meaning |
|---|---|
| `skip_servo` | Bypass the servo stage entirely |
| `skip_servo_speed` | Speed used when servoing is skipped |
| `servo_img_u_offset_px` | Horizontal image offset correction |
| `servo_img_p1_pullback` | Pullback distance at phase 1 |
| `servo_grasp_z` | Grasp height during servoing |
| `mtc_delay` | Delay before MTC starts |

### `perception.launch.py` arguments

Mirrors the `perception_node` parameters — see
[Arm interfaces](ros-arm.md#key-parameters) for meanings. The extrinsic set is
the one you'll touch most:

`eth_x`, `eth_y`, `eth_z`, `eth_roll`, `eth_pitch`, `eth_yaw` (eye-to-hand) and
`eih_x` … `eih_yaw` (eye-in-hand).

!!! tip "Fake first"

    `fake_object` and `fake_apriltag` let you exercise the entire motion pipeline
    with no camera attached. If the arm misbehaves, run with fakes to establish
    whether the problem is perception or motion before touching calibration.

## Web bridge

```bash
ros2 run robot_web_bridge server        # http://localhost:8088
```

See the [HTTP API](http.md) for routes and the no-ROS fallback path.

## See also

- [JetRacer interfaces](ros-jetracer.md)
- [Arm interfaces](ros-arm.md)
- [Real-hardware bringup](../ra_hardware_bringup.md)
