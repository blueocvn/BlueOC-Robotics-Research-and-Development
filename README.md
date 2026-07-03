# robot-fulfillment

ROS 2 workstation packages for the JetRacer + robot-arm fulfillment system.
On-device packages that run on the JetRacer itself are **not** in this repo.

## Workspaces

The repo is split into independent colcon workspaces by concern. Each builds
on its own (`colcon build`) and they communicate only over the ROS 2 graph, so
they share a **DDS domain**, not a build space.

| Workspace          | Concern                                                        |
| ------------------ | ------------------------------------------------------------- |
| `jetracer_ws/`     | SLAM, navigation, ackermann control, Isaac Sim, interfaces    |
| `ra_ws/`           | Robot arm: MoveIt, kortex/robotiq drivers, MTC                |
| `orchestrator_ws/` | `robot_web_bridge` — HTTP/web UI + mission/state logic         |

## Network setup

Real IPs are kept out of git. Copy the example and fill in your values:

```bash
cp network.env.example network.env
# edit network.env: WORKSTATION_IP, JETRACER_IP, DDS_INTERFACE, ROS_DOMAIN_ID
```

`network.env` is gitignored. The run scripts source it from the repo root, and
the DDS profiles (`/tmp/cyclonedds.xml`, `jetracer_ws/fastdds.xml`) are rendered
at runtime from `network.env` + `jetracer_ws/fastdds.xml.template`. Never commit
real IPs — put them only in `network.env`.

All workspaces must use the same `ROS_DOMAIN_ID` to see each other.

## Running

```bash
# Workstation container (SLAM/nav/Isaac), reads network.env:
./jetracer_ws/run_workstation.sh

# Orchestrator web bridge (shares the same ROS_DOMAIN_ID):
./orchestrator_ws/run_web_bridge.sh
```
