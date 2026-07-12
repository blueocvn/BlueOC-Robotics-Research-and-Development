# Getting Started

Welcome to the **Robot Fulfillment** system — a web orchestrator that dispatches
a mobile robot (AMR) and a robot arm (RA) to fulfil drink orders, all runnable
against NVIDIA Isaac Sim with no physical hardware.

This page is the map. Pick the workspace you want to set up, and skim the
**Concepts** section first if the terms are new — each links to the canonical
docs so you can read up before touching the code.

---

## Setup guides

Each workspace builds and runs on its own; they communicate only over the ROS 2
graph (a shared DDS domain), not a shared build space. **Use the same
`ROS_DOMAIN_ID` across all three** so they can see each other.

| Workspace | What it is | ROS distro | Guide |
|---|---|---|---|
| `ra_ws` | SO-ARM 101 robot arm — perception, MoveIt 2 / MTC, visual servo, grasp → fill → place | **Jazzy** (native) | **[Robotic Arm setup →](ra_setup.md)** |
| `jetracer_ws` | JetRacer AMR — SLAM, Nav2, Ackermann drive | **Humble** (Docker) | **[AMR setup →](amr_setup.md)** |
| `orchestrator_ws` | `robot_web_bridge` — FastAPI + HTMX web UI + dispatcher | Humble | see the package README |

> **⚠️ Two ROS distros on purpose.** `ra_ws` runs on **Jazzy (Ubuntu 24.04)**
> natively; `jetracer_ws` / `orchestrator_ws` run on **Humble** inside
> `Dockerfile.dev`. They interoperate over DDS — do **not** try to build `ra_ws`
> inside the Humble container.

> **▶ Isaac Sim must be playing before you launch either robot.** Both the arm and
> the AMR run entirely against Isaac Sim — open the relevant scene and press
> **Play** *first*, or the ROS nodes stall waiting on `/clock` and the sim's
> topics (joint states, cameras, odom, lidar). Each setup guide repeats this at
> its run step.

---

## Concepts & topics for new learners

New to this stack? These are the ideas the code is built on. Read the linked
docs for the ones you're unfamiliar with before diving in.

### Shared foundations (both robots)

| Concept | What it is here | Read up |
|---|---|---|
| **ROS 2** | The middleware everything runs on — nodes, topics, services, actions | [docs.ros.org](https://docs.ros.org/en/humble/index.html) |
| **DDS / `ROS_DOMAIN_ID`** | How the three workspaces discover and talk to each other over the network | [ROS 2 DDS & domain IDs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) |
| **colcon / rosdep** | Build the workspaces; resolve system dependencies | [colcon](https://colcon.readthedocs.io/) · [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) |
| **TF2** | Coordinate-frame tree (world → camera → gripper, etc.) | [tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html) |
| **NVIDIA Isaac Sim** | The simulator providing physics, robots, sensors, and the ROS 2 Bridge | [Isaac Sim docs](https://docs.isaacsim.omniverse.nvidia.com/) |
| **`use_sim_time` / `/clock`** | Every node runs on simulator time; `/clock` must keep flowing | [Using sim time](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html) |

### Robot arm (`ra_ws`) concepts

| Concept | What it is here | Read up |
|---|---|---|
| **MoveIt 2** | Motion-planning framework — `move_group`, kinematics, collision checking | [moveit.ai](https://moveit.picknik.ai/main/index.html) |
| **MoveIt Task Constructor (MTC)** | Stages the pick → lift → place task; the backbone of `mtc_node` | [MTC docs](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) |
| **OMPL / RRTConnect** | The sampling motion planner used for gross moves | [OMPL](https://ompl.kavrakilab.org/) |
| **Inverse kinematics (position-only, 5-DOF)** | Solving joint angles for a target; the arm is 5-DOF so orientation is only partly controllable | [MoveIt IK](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) |
| **`ros2_control` / `topic_based_ros2_control`** | The controller layer; the topic-based variant bridges to Isaac Sim | [ros2_control](https://control.ros.org/) |
| **Visual servoing (IBVS / PBVS)** | Closing the loop on camera feedback to home onto the mug (custom loop, *not* `moveit_servo`) | [Visual servo overview](https://visp.inria.fr/visual-servoing/) · [XLeRobot SO-101 servoing](https://xlerobot.readthedocs.io/en/latest/software/getting_started/SO101.html) |
| **YOLO (YOLO11n)** | Neural object detector used to find the mug | [Ultralytics YOLO](https://docs.ultralytics.com/) |
| **AprilTag** | Fiducial marker on the dispenser for sub-mm pose | [AprilTag](https://april.eecs.umich.edu/software/apriltag) |
| **HSV segmentation + ray-plane unprojection** | Color-threshold the mug/tray, then project the pixel onto a known ground plane to get a world coordinate | [OpenCV color spaces](https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html) |

→ Full walkthrough: **[Robotic Arm setup](ra_setup.md)**

### AMR (`jetracer_ws`) concepts

| Concept | What it is here | Read up |
|---|---|---|
| **SLAM (`slam_toolbox`)** | Build a map of the space while localizing in it | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) |
| **Nav2** | The navigation stack — planners, controllers, behavior trees, costmaps | [Nav2 docs](https://docs.nav2.org/) |
| **AMCL** | Particle-filter localization against a saved map | [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) |
| **Costmaps** | Occupancy grids Nav2 plans and avoids obstacles on | [Nav2 costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html) |
| **Behavior Trees** | How Nav2 sequences navigation behaviors | [Nav2 BT](https://docs.nav2.org/behavior_trees/index.html) |
| **`NavigateToPose` action** | The action a goal sender calls to drive to a pose | [Nav2 actions](https://docs.nav2.org/commander_api/index.html) |
| **Odometry (`/chassis/odom`)** | The base's estimated motion, fused for localization | [nav_msgs/Odometry](https://docs.ros.org/en/humble/p/nav_msgs/) |
| **Ackermann steering** | Car-like drive (steer angle + drive), unlike a differential base | [Ackermann geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) |

→ Full walkthrough: **[AMR setup](amr_setup.md)**

---

## Which do I set up first?

- Just want to see the **arm grasp cups**? → [ra_setup.md](ra_setup.md) (self-contained, needs only Isaac Sim + Jazzy).
- Want the **AMR to map and navigate**? → [amr_setup.md](amr_setup.md).
- Want the **web-order → robot** end-to-end flow? Set up `orchestrator_ws` plus at least the AMR, and keep a common `ROS_DOMAIN_ID`.

> **State of the system:** both robots run in **simulation**; there is no on-device
> firmware yet. The orchestrator ↔ AMR docking seam and the orchestrator ↔ RA
> integration are still open work — the setup guides flag exactly what's built vs.
> planned.