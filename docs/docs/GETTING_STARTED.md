# JetRacer — Getting Started

The physical robot stack. Runs **on the JetRacer** (Waveshare JetRacer, ROS 2
Humble): base driver, RPLidar, EKF odometry, Nav2 navigation, and AprilTag
docking.

---

## 1. Build

The `start_*.sh` scripts source **`ws_setup.bash`**, which sources ROS + every
installed package individually. This is a deliberate workaround for an
incomplete merged `install/setup.bash` on this device — use it instead of
`source install/setup.bash`.

```bash
cd jetracer_ws
colcon build --symlink-install
source ws_setup.bash
```

> ⚠️ Keep the robot **still for ~2 s** at driver startup while the gyro
> calibrates. Moving during calibration corrupts odometry.

---

## 2. The layers

The stack is split so you can bring up only what you need:

| Script                | What it starts                                                     | Publishes                                      |
| --------------------- | ----------------------------------------------------------------- | ---------------------------------------------- |
| `./start_driver.sh`   | Base driver only (`/cmd_vel` → serial)                            | `/odom`, `/imu`                                |
| `./start_lidar.sh`    | RPLidar A1 + `base_footprint→laser_frame` TF                      | `/scan`                                        |
| `./start_hardware.sh` | **driver + lidar + EKF + static TFs + camera/AprilTag** (no Nav2) | `/odom`, `/imu`, `/scan`, `/odometry/filtered` |
| `./start_nav2.sh`     | Nav2 (map_server, AMCL, controller, planner, BT nav)              | drives `/cmd_vel`                              |
| `./start_mapping.sh`  | Nav2 motion + `explore_lite` frontier exploration (no map_server) | autonomous map building                        |

`start_hardware.sh` is the base every workflow needs. `start_nav2.sh` and
`start_mapping.sh` both bring up the Nav2 motion nodes — **don't run both.**

---

## 3. Common workflows

### Navigate on a known map

Easiest — tmux brings up hardware (left pane) then Nav2 (right pane, after 8 s):

```bash
cd jetracer_ws
./start_tmux.sh
# detach: Ctrl-b d   reattach: tmux attach -t jetracer   kill: tmux kill-session -t jetracer
```

Or manually, in two terminals:

```bash
./start_hardware.sh
./start_nav2.sh map:=/ros2_ws/maps/test_map_outer_v6.yaml
```

Maps live in `jetracer_ws/maps/` (`test_map_outer_v6.yaml` is the default). Then
set a Nav2 goal from RViz.

### Build a new map

1. `./start_hardware.sh`
2. Run SLAM (`slam_toolbox`) against the robot's `/scan` + TF.
3. Drive around — teleop, **or** `./start_mapping.sh` for autonomous frontier
   exploration.
4. Save the map and drop the `.yaml`/`.pgm` into `jetracer_ws/maps/`.

### Docking (AprilTag)

`start_hardware.sh` also brings up the CSI camera + AprilTag detector.
`jetracer_bringup/scripts/jetracer_docker.py` runs the dock/undock state machine
(sequencing driven by `/docking_state`). Round-trip demo, with the full stack
already running:

```bash
./dock_cycle.sh dock1 dock0     # dock A → undock → dock B → undock
```

Camera intrinsics and the dock-tag layout are in `jetracer_bringup/config/`.
See `CALIBRATION.md` — docking accuracy depends on the camera TF + calibration.

---

## 4. Robot Web Bridge (ordering app)

The application layer — a FastAPI + HTMX mobile web UI + HTTP API for commanding
the robot (the QR-code "Get Water" / "Refill" ordering flow). It lives in a
separate workspace, **`orchestrator/`**, and talks to this stack over the ROS
graph: it publishes `/dock_robot`, `/abort_docking`, `/cmd_vel`, `/initialpose`
and subscribes `/docking_state` + `/chassis/odom`. So the robot stack from §3
(hardware + Nav2 + docking) must already be running, on the **same
`ROS_DOMAIN_ID`**.

### Start it

```bash
cd ../orchestrator
./run_web_bridge.sh          # serves on http://<host>:8088
```

The script sources `network.env` + ROS, then runs `ros2 run robot_web_bridge
server`. It expects to run inside the Humble container (or a ROS-sourced shell).
Override the port with `ROBOT_WEB_BRIDGE_PORT=9000`, and set the operator PIN via
`ROBOT_WEB_BRIDGE_ADMIN_PIN` for the admin routes.

### Expose it for phones (QR codes)

In another shell, tunnel the port to a public URL so phones can scan and order:

```bash
./run_tunnel.sh              # prints an https://<...>.trycloudflare.com URL
```

### Without the robot (dev / demo)

```bash
./run_web_bridge_sim.sh      # SimBackend: no ROS, each leg completes on a timer
```

Verify the mode at runtime: `GET /api/state` → `{"mode": "robot" | "simulation"}`.
Health check: `GET /healthz`.

---

## 5. Overriding defaults

Extra args pass straight through to the launch files:

```bash
./start_hardware.sh base_port:=/dev/ttyACM1 lidar_port:=/dev/ttyACM0
./start_nav2.sh     map:=/ros2_ws/maps/my_map.yaml
```

Defaults: base port `/dev/ttyACM0`, lidar port `/dev/ttyACM1`.

---

## Troubleshooting

- **`ros2` or packages (nav2, robot_localization, jetracer_bringup) missing** →
  you sourced the merged setup. Use `source ws_setup.bash`.
- **Odometry drifts from the start** → the robot moved during the ~2 s gyro
  calibration. Restart the driver and hold it still.
- **Wrong serial port** → override with `base_port:=` / `lidar_port:=`.
- **Nav2 won't move / no path** → check `/scan` and TF are live
  (`ros2 topic hz /scan`), and that AMCL is localized on the right map.
