# Getting Started

**Robot Fulfillment** là một orchestrator web điều một xe tự hành (AMR) và một
cánh tay robot (RA) để hoàn thành order đồ uống. Chạy được hết trên NVIDIA Isaac
Sim, không cần phần cứng thật.

---

## Setup guides

Mỗi workspace tự build và tự chạy; chúng chỉ giao tiếp qua đồ thị ROS 2 (một miền
DDS chung), không dùng chung không gian build. **Hãy dùng cùng một
`ROS_DOMAIN_ID` cho cả ba** để chúng thấy được nhau.

| Workspace | Là gì | Bản phân phối ROS | Hướng dẫn |
|---|---|---|---|
| `ra_ws` | Cánh tay robot SO-ARM 101 — perception, MoveIt 2 / MTC, visual servoing, gắp → hứng → đặt | **Jazzy** (native) | **[Cài đặt cánh tay robot →](ra_setup.md)** |
| `jetracer_ws` | JetRacer AMR — SLAM, Nav2, Ackermann | **Humble** (Docker) | **[Cài đặt AMR →](amr_setup.md)** |
| `orchestrator` | `robot_web_bridge` — giao diện web FastAPI + HTMX + dispatcher | Humble | xem README của package |

> **⚠️ Hai bản phân phối ROS là có chủ đích.** `ra_ws` chạy **Jazzy (Ubuntu 24.04)**
> native; `jetracer_ws` / `orchestrator` chạy **Humble** bên trong `Dockerfile.dev`.
> Chúng giao tiếp với nhau qua DDS — **đừng** cố build `ra_ws` bên trong container
> Humble.

> **▶ Isaac Sim phải đang chạy trước khi bạn khởi động bất kỳ robot nào.** Cả cánh
> tay lẫn AMR đều chạy hoàn toàn dựa trên Isaac Sim — hãy mở scene tương ứng và
> bấm **Play** *trước*, nếu không các node ROS sẽ treo chờ `/clock` và các topic
> của bộ mô phỏng (trạng thái khớp, camera, odom, lidar). Mỗi hướng dẫn cài đặt
> đều nhắc lại điều này ở bước chạy.

---

## Concepts & topics for new learners

Mới làm quen với stack này? Đây là các khái niệm nền của mã nguồn. Khái niệm nào
chưa quen thì đọc link đính kèm trước.

### Shared foundations (both robots)

| Khái niệm | Ở đây nghĩa là gì | Đọc thêm |
|---|---|---|
| **ROS 2** | Middleware mà mọi thứ chạy trên đó — node, topic, service, action | [docs.ros.org](https://docs.ros.org/en/humble/index.html) |
| **DDS / `ROS_DOMAIN_ID`** | Cách ba workspace phát hiện và nói chuyện với nhau qua mạng | [ROS 2 DDS & domain ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) |
| **colcon / rosdep** | Build các workspace; giải quyết system dependency | [colcon](https://colcon.readthedocs.io/) · [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) |
| **TF2** | Cây hệ tọa độ (world → camera → gripper, v.v.) | [tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html) |
| **NVIDIA Isaac Sim** | Bộ mô phỏng cung cấp vật lý, robot, cảm biến và ROS 2 Bridge | [Tài liệu Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) |
| **`use_sim_time` / `/clock`** | Mọi node chạy theo thời gian mô phỏng; `/clock` phải luôn chảy | [Dùng sim time](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html) |

### Robot arm (`ra_ws`) concepts

| Khái niệm | Ở đây nghĩa là gì | Đọc thêm |
|---|---|---|
| **MoveIt 2** | Framework lập kế hoạch chuyển động — `move_group`, động học, kiểm tra va chạm | [moveit.ai](https://moveit.picknik.ai/main/index.html) |
| **MoveIt Task Constructor (MTC)** | Chia tác vụ gắp → nâng → đặt thành các giai đoạn; `mtc_node` dựng trên nó | [Tài liệu MTC](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) |
| **OMPL / RRTConnect** | Bộ lập kế hoạch chuyển động theo lấy mẫu, dùng cho các chuyển động lớn | [OMPL](https://ompl.kavrakilab.org/) |
| **Động học ngược (chỉ vị trí, 5 bậc tự do)** | Giải góc khớp cho một mục tiêu; cánh tay chỉ có 5 bậc tự do nên hướng chỉ điều khiển được một phần | [MoveIt IK](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) |
| **`ros2_control` / `topic_based_ros2_control`** | Lớp controller; biến thể dựa trên topic làm bridge tới Isaac Sim | [ros2_control](https://control.ros.org/) |
| **Visual servoing (IBVS / PBVS)** | Dùng ảnh camera để chỉnh dần cho tới khi gripper vào đúng thế gắp (vòng lặp tự viết, *không phải* `moveit_servo`) | [Tổng quan visual servo](https://visp.inria.fr/visual-servoing/) · [XLeRobot SO-101 servoing](https://xlerobot.readthedocs.io/en/latest/software/getting_started/SO101.html) |
| **YOLO (YOLO11n)** | Detector vật thể bằng mạng nơ-ron, dùng để tìm cốc | [Ultralytics YOLO](https://docs.ultralytics.com/) |
| **AprilTag** | Fiducial trên máy lọc nước cho độ chính xác pose dưới milimét | [AprilTag](https://april.eecs.umich.edu/software/apriltag) |
| **Phân vùng HSV + ray-plane unprojection** | Lọc màu cốc/khay, rồi chiếu điểm ảnh lên một mặt phẳng nền đã biết để ra tọa độ thế giới | [Không gian màu OpenCV](https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html) |

→ Hướng dẫn đầy đủ: **[Cài đặt cánh tay robot](ra_setup.md)**

### AMR (`jetracer_ws`) concepts

| Khái niệm | Ở đây nghĩa là gì | Đọc thêm |
|---|---|---|
| **SLAM (`slam_toolbox`)** | Dựng bản đồ không gian đồng thời tự định vị trong đó | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) |
| **Nav2** | Stack điều hướng — planner, controller, behavior tree, costmap | [Tài liệu Nav2](https://docs.nav2.org/) |
| **AMCL** | Định vị bằng particle filter dựa trên bản đồ đã lưu | [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) |
| **Costmap** | Lưới chiếm dụng mà Nav2 dùng để lập kế hoạch và né vật cản | [Costmap Nav2](https://docs.nav2.org/configuration/packages/configuring-costmaps.html) |
| **Behavior Tree** | Cách Nav2 sắp xếp trình tự các hành vi điều hướng | [Nav2 BT](https://docs.nav2.org/behavior_trees/index.html) |
| **Action `NavigateToPose`** | Action mà bên gửi goal gọi để đưa robot tới một pose | [Action Nav2](https://docs.nav2.org/commander_api/index.html) |
| **Odometry (`/chassis/odom`)** | Ước lượng chuyển động của bệ, được hợp nhất phục vụ định vị | [nav_msgs/Odometry](https://docs.ros.org/en/humble/p/nav_msgs/) |
| **Ackermann** | Kiểu lái ô tô (góc lái + lực kéo), khác với bệ vi sai | [Hình học Ackermann](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) |

→ Hướng dẫn đầy đủ: **[Cài đặt AMR](amr_setup.md)**

---

## Which do I set up first?

- Chỉ muốn xem **cánh tay gắp cốc**? → [ra_setup.md](ra_setup.md) (độc lập, chỉ cần Isaac Sim + Jazzy).
- Muốn **AMR dựng bản đồ và điều hướng**? → [amr_setup.md](amr_setup.md).
- Muốn luồng trọn vẹn **order web → robot**? Hãy dựng `orchestrator` cộng thêm ít nhất AMR, và giữ chung một `ROS_DOMAIN_ID`. → [orchestrator.md](orchestrator.md).

> **Trạng thái hệ thống:** cả hai robot đều chạy **mô phỏng**; chưa có firmware
> trên thiết bị. Mối nối docking orchestrator ↔ AMR và phần tích hợp orchestrator
> ↔ RA vẫn còn dang dở — các hướng dẫn cài đặt nêu rõ đâu là phần đã dựng và đâu
> là phần còn trong kế hoạch.
