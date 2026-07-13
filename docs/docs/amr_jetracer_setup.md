# AMR Setup (`jetracer_ws`) — JetRacer SLAM + Nav2 on device

A car-like (Ackermann) Waveshare JetRacer on ROS 2 Humble that maps a
space, localizes, navigates, and docks. This page covers the real robot in
[`amr/jetracer_ws/`](https://github.com/blueocvn/robotic-arm/tree/main/amr/jetracer_ws):
base driver, RPLidar, EKF odometry, Nav2, and AprilTag docking.

> **Scope:** on-hardware robot only. The Isaac Sim workflow (`carter_navigation` /
> `slam_custom` on the workstation) is a separate stack in `amr/workstation_ws/`
> and is not covered here.

> **⚠️ ROS distro:** the JetRacer runs **ROS 2 Humble** natively on the device
> (Jetson). Build it there with `colcon`; source it with **`ws_setup.bash`**, not
> the merged `install/setup.bash` (see §3 for why).

### 1. Hardware & prerequisites

| Component | Notes |
|---|---|
| Chassis | Waveshare **JetRacer** (Ackermann steering), Jetson on-board computer |
| OS / ROS | JetPack + **ROS 2 Humble** installed natively on the Jetson |
| Base MCU | Serial (`/dev/ttyACM0` default) — motors + IMU/gyro; consumed by `jetracer_driver` |
| Lidar | **RPLidar A1** on `/dev/ttyACM1`, 115200 baud, mounted **inverted** (`laser_frame`, yaw π, z ≈ 0.18 m) |
| Camera | CSI **IMX219** at 640×360 (via `gscam2`) → AprilTag detector for docking |
| Build tools | `colcon`, `rosdep`, `vcstool` (`sudo apt install python3-vcstool`), `git`, `tmux` |

> Serial device order isn't guaranteed across reboots. If the base and lidar swap
> ports, override them (§7). Keep the robot **still for ~2 s** at driver startup —
> the gyro calibrates then, and motion corrupts odometry.

### 2. Restore third-party sources & build

`src/` mixes first-party packages (committed) with pinned third-party ROS 2
packages (not committed, restored via `vcstool`). On a fresh device:

```bash
cd amr/jetracer_ws

# 1. Restore tag-pinned third-party packages
vcs import src < thirdparty.repos

# 2. Re-apply local patches (nav2 bond shared_ptr; robot_localization EKF tune)
patch -p1 -d src/navigation2        < patches/nav2_util-bond-shared_ptr.patch
patch -p1 -d src/robot_localization < patches/robot_localization-ekf-jetracer-tune.patch

# 3. Build
colcon build --symlink-install
```

First-party packages are `jetracer_bringup`, `jetracer_description`,
`jetracer_driver`. A few third-party packages are vendored deliberately (e.g.
`apriltag` past `v3.4.5` for tag-pose estimation, `rplidar_ros` ROS 2 branch with
C1 support). See [`THIRDPARTY_SETUP.md`](https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/THIRDPARTY_SETUP.md)
for the full split and patch rationale.

### 3. Sourcing — use `ws_setup.bash`, not `install/setup.bash`

After building, source the workspace with:

```bash
source ws_setup.bash
```

> **Why:** the merged `install/setup.bash` on this device is **incomplete**. Mixed
> file ownership (some packages root-owned, some user-owned) makes `colcon` drop
> packages it can't rewrite when it regenerates `setup.bash`, so
> `robot_localization`, `nav2_*`, and `jetracer_bringup` go missing from the
> environment even though they're installed on disk. `ws_setup.bash` sidesteps
> that by sourcing every installed package's `local_setup.bash` directly. The
> `start_*.sh` scripts already source it for you.
>
> If `ros2` or a package (nav2, robot_localization, jetracer_bringup) is
> "not found", you sourced the merged setup — use `ws_setup.bash`.

### 4. DDS networking — static unicast peers

The robot has to share the DDS graph with the workstation / orchestrator. On the
JetRacer the DDS config lives in **`ros2_docker_v3.sh`** on the device.

**Why this manual step exists:** DDS normally auto-discovers peers via UDP
**multicast**, but multicast doesn't work reliably on the JetRacer's network
interface — so the robot never sees the workstation and no topics appear. The fix
is to disable multicast (`<AllowMulticast>false`) and fall back to **static
unicast discovery**: give the JetRacer a fixed list of peer **IP addresses** so it
reaches each machine directly. Because discovery is now manual, **every** machine's
config must list **every** other machine's IP — miss one and that pair won't see
each other.

1. SSH into the JetRacer:
   ```bash
   ssh jetracer@192.168.20.91     # use the JetRacer's real LAN IP / hostname
   ```
2. Open the launcher and locate its CycloneDDS config:
   ```bash
   grep -n "Discovery" ros2_docker_v3.sh   # find the block, then edit
   nano ros2_docker_v3.sh
   ```
3. Update the `<Discovery>` block so it lists the JetRacer **plus every peer** and
   raises the participant ceiling:
   ```xml
   <Discovery>
       <ParticipantIndex>auto</ParticipantIndex>
       <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
       <Peers>
           <Peer Address="192.168.20.XXX"/> <!-- JetRacer (this device)        -->
           <Peer Address="192.168.20.XXX"/>   <!-- workstation                    -->
           <Peer Address="192.168.20.XXX"/>   <!-- e.g. orchestrator host         -->
           <Peer Address="192.168.20.XXX"/>   <!-- one <Peer> per machine on graph -->
       </Peers>
   </Discovery>
   ```
   Replace each `192.168.20.XXX` with the machine's real LAN IP (`ip -o -4 addr show`
   on that machine). `MaxAutoParticipantIndex` must be high enough (200 is safe) to
   cover all participants across every host under `auto` indexing.
4. Confirm the same **`ROS_DOMAIN_ID`** everywhere (default `42`) and that the
   CycloneDDS `<NetworkInterface>` name matches the JetRacer's real NIC.

> The **workstation** side of this same peer list is generated automatically from
> `network.env` by `run_workstation.sh` / `run_orchestrator.sh` (`WORKSTATION_IP`,
> `JETRACER_IP`, `DDS_INTERFACE`). Keep the two in sync — every IP on one side must
> be reachable and listed on the other.

### 5. The layers (`start_*.sh`)

The stack is split so you bring up only what you need. All scripts source
`ws_setup.bash` first.

| Script | Starts | Publishes / does |
|---|---|---|
| `./start_driver.sh [port]` | Base driver only (`/cmd_vel` → serial) | `/odom`, `/imu` |
| `./start_lidar.sh` | RPLidar A1 + `base_footprint→laser_frame` TF | `/scan` |
| `./start_hardware.sh` | **driver + lidar + EKF + static TFs + camera/AprilTag** (no Nav2) | `/odom`, `/imu`, `/scan`, `/odometry/filtered` |
| `./start_nav2.sh` | Nav2 (map_server, AMCL, controller, planner, BT nav) | drives `/cmd_vel` |
| `./start_mapping.sh` | Nav2 motion + `explore_lite` frontier exploration (no map_server) | autonomous map building |

`start_hardware.sh` is the base every workflow needs. `start_nav2.sh` and
`start_mapping.sh` both bring up the Nav2 motion nodes — **don't run both at once.**

### 6. Common workflows

#### Navigate on a known map

Easiest — tmux brings up hardware (left pane) then Nav2 (right pane, after ~8 s so
TF and `/scan` are live first):

```bash
cd amr/jetracer_ws
./start_tmux.sh
# detach: Ctrl-b d   reattach: tmux attach -t jetracer   kill: tmux kill-session -t jetracer
```

Or manually, in two terminals:

```bash
./start_hardware.sh
./start_nav2.sh map:=/ros2_ws/maps/test_map_outer_v6.yaml
```

Maps live in `jetracer_ws/maps/` (`test_map_outer_v6.yaml` is the default). Then
drop a **2D Pose Estimate** to seed AMCL and a **Nav2 Goal** from RViz.

#### Build a new map

1. `./start_hardware.sh`
2. Run SLAM (`slam_toolbox`) against the robot's `/scan` + TF.
3. Explore the space — teleop (`teleop_twist_keyboard`), **or** `./start_mapping.sh`
   for autonomous frontier exploration.
4. Serialize the map and drop the `.yaml` / `.pgm` into `jetracer_ws/maps/`.

#### Docking (AprilTag)

`start_hardware.sh` also brings up the CSI camera + AprilTag detector.
`jetracer_bringup/scripts/jetracer_docker.py` runs the dock/undock state machine,
sequenced by the `/docking_state` topic. With the full stack (hardware + Nav2 +
docker) running, a round-trip demo:

```bash
./dock_cycle.sh dock1 dock0     # dock A → undock → dock B → undock
```

It publishes `/dock_robot` (String) and `/undock_robot` (Bool) and waits on
`/docking_state`. Docking accuracy depends on camera calibration — see §8.

### 7. Overriding defaults (serial ports, map)

Extra args pass straight through to the launch files:

```bash
./start_hardware.sh base_port:=/dev/ttyACM1 lidar_port:=/dev/ttyACM0
./start_nav2.sh     map:=/ros2_ws/maps/my_map.yaml
```

Defaults: base port `/dev/ttyACM0`, lidar port `/dev/ttyACM1`.

### 8. Calibration

Several shipped values are **placeholders/estimates** and will visibly hurt
accuracy until measured. Do these in priority order (see
[`CALIBRATION.md`](https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/CALIBRATION.md)):

1. **Camera intrinsics** 🔴 — `jetracer_bringup/config/imx219.yaml` ships a fake
   pinhole (`fx=fy=320`, zero distortion). Gates *all* docking accuracy. Collect
   frames with `grab_frames.py`, run `camera_calibration`, paste real values.
2. **Wheel odometry scale** 🟡 — `jetracer_driver`'s `ENCODER_SCALE`; drive a
   measured 1 m and correct so `/odometry/filtered` reads ~1.0 m.
3. **Ackermann geometry** 🟡 — wheelbase / max steer / min turning radius must
   agree across the three places they appear.

### 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ros2` / nav2 / robot_localization / jetracer_bringup "not found" | You sourced the merged `install/setup.bash`. Use `source ws_setup.bash` (§3). |
| Odometry drifts from the start | Robot moved during the ~2 s gyro calibration. Restart the driver and hold it still. |
| Wrong serial port / driver won't open device | Ports swapped across reboot — override with `base_port:=` / `lidar_port:=` (§7). |
| Nav2 won't move / no path | Check `/scan` and TF are live (`ros2 topic hz /scan`) and AMCL is localized on the right map (seed a 2D Pose Estimate). |
| Docking aims at the wrong spot | Camera intrinsics are still the placeholder — calibrate `imx219.yaml` (§8). |
| DDS peers can't see each other | Mismatched `ROS_DOMAIN_ID`, or the unicast `<Peers>` lists don't match — every machine must list every other's IP, since multicast is off (§4). |
| `rcl node's rmw handle is invalid` on startup | CycloneDDS can't bind — the `<NetworkInterface>` name doesn't exist on that host. Check `ip -o -4 addr show` and set the real NIC (modern Linux: `enp*` / `eno*` / `wlp*`, not `eth0`). |

### 10. Notes for maintainers

- Sourcing is deliberately `ws_setup.bash`, not the merged setup — don't "fix" the
  start scripts to use `install/setup.bash` (§3).
- Third-party source is restored from `thirdparty.repos` + local patches; keep them
  byte-reproducible (see `THIRDPARTY_SETUP.md`). `apriltag` and `rplidar_ros` are
  vendored on purpose — a tag pin would regress the robot.
- DDS discovery is **static unicast** (multicast off): the JetRacer's peer list
  lives in `ros2_docker_v3.sh` on the device (§4). Adding a machine to the graph
  means adding its IP there (and on every other peer).
- Odometry: the driver publishes raw `/odom`, the EKF fuses it into
  `/odometry/filtered` (what Nav2 consumes) and owns the `odom→base_footprint` TF.
- Docking is a **topic contract** (`/dock_robot`, `/undock_robot`, `/docking_state`)
  driven by `jetracer_docker.py`, not a Nav2 docking server.
- Calibration values in `jetracer_bringup/config/` ship as placeholders; docking
  and localization degrade until they're measured (§8).
