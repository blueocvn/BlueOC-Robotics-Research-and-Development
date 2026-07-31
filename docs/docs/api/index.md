# API Book

Everything you can call, publish, or subscribe to during the hackathon — for
both the **JetRacer** (AMR) and the **SO-ARM 101** (robot arm).

The surface splits into three layers. Most teams only need the first.

<div class="grid cards" markdown>

-   :material-api:{ .lg .middle } **HTTP API**

    ---

    Drive the JetRacer over plain HTTP — place orders, dock, teleop, e-stop.
    No ROS install required on your machine.

    [:octicons-arrow-right-24: HTTP reference](http.md)

-   :material-robot-outline:{ .lg .middle } **ROS 2 — JetRacer**

    ---

    Topics the AMR listens on and publishes: `/cmd_vel`, `/dock_robot`,
    `/docking_state`, odometry.

    [:octicons-arrow-right-24: JetRacer interfaces](ros-jetracer.md)

-   :material-robot-industrial:{ .lg .middle } **ROS 2 — Robot Arm**

    ---

    Perception detections, MoveIt/MTC pick-and-place, joint trajectory control,
    AprilTag poses.

    [:octicons-arrow-right-24: Arm interfaces](ros-arm.md)

-   :material-rocket-launch-outline:{ .lg .middle } **Launch entry points**

    ---

    How to actually start each stack, and which launch file to pick for sim
    versus real hardware.

    [:octicons-arrow-right-24: Launch reference](launch.md)

</div>

## Which layer should I use?

| If you want to… | Use | Needs ROS locally? |
|---|---|---|
| Send the robot to a table from a web app or script | [HTTP API](http.md) | No |
| React to live robot state at full rate | [ROS 2 topics](ros-jetracer.md) | Yes |
| Command the arm to pick something up | [Arm interfaces](ros-arm.md) | Yes |
| Bring a stack up from scratch | [Launch reference](launch.md) | Yes |

!!! tip "Start with HTTP"

    The HTTP API is the fastest path to a working demo — it runs without ROS at
    all. If `rclpy` isn't importable, the bridge transparently falls back to a
    **simulated backend** that progresses orders on a timer, so you can build
    and test your whole front end before you ever touch the robot.
    `GET /state` tells you which mode you're in.

## Conventions used in this book

- **Topic names** are absolute (`/cmd_vel`) unless noted as relative, in which
  case the node's namespace applies.
- **Message types** are given as `package/msg/Type`.
- Routes marked :material-lock: require operator authentication — see
  [Admin authentication](http.md#admin-authentication).

## Reporting drift

The HTTP reference is generated from the running FastAPI app, so it cannot go
stale. The ROS pages are hand-maintained against source. If you find a topic
that doesn't match reality, that's a bug worth reporting — please open an issue
rather than working around it silently.
