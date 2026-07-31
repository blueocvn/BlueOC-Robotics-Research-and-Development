# Robotic Arm Setup (`ra_ws`) — SO-ARM 101 Isaac Sim Refill Demo

Một SO-ARM 101 5 bậc tự do gắp chiếc cốc có lòng màu xanh lá trong NVIDIA Isaac
Sim, visual servoing tới nó bằng camera gắn trên tay, mang tới máy lọc được đánh dấu
bằng AprilTag để "hứng nước", rồi đặt vào khay hồng. Perception, MoveIt 2 / MoveIt
Task Constructor (MTC) và vòng lặp visual servoing chạy ở phía ROS 2; Isaac Sim cung
cấp vật lý, robot và camera.

Hướng dẫn này giả định bạn đã **cài sẵn Isaac Sim** và bắt đầu từ đó.

!!! tip "Chưa có Isaac Sim? Hãy bắt đầu với hướng dẫn chính thức của NVIDIA"
    - **[Cài đặt trên workstation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)**
      — tải, cài và khởi chạy Isaac Sim trên Linux (bao gồm cả trình kiểm tra
      tương thích cho yêu cầu GPU/driver).
    - **[Cài đặt ROS 2](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)**
      — bật extension ROS 2 Bridge và trỏ nó tới bản ROS bạn đã cài.

    !!! warning "Hãy nạp ROS 2 *trước khi* khởi chạy Isaac Sim"
        Bridge nạp các thư viện ROS 2 từ môi trường **đã được source** của bạn.
        Hãy chạy `source /opt/ros/jazzy/setup.bash` trong đúng terminal mà bạn
        khởi chạy Isaac, nếu không bridge sẽ lấy nhầm bản phân phối (hoặc không
        lấy được gì cả).

> **⚠️ Bản phân phối ROS:** `ra_ws` nhắm tới **ROS 2 Jazzy (Ubuntu 24.04)** —
> *không phải* stack Humble mà phần còn lại của repo (`jetracer_ws`,
> `orchestrator`) đang chạy. Hãy build và chạy `ra_ws` trên bản Jazzy cài native,
> **không phải** bên trong container Humble `Dockerfile.dev`. Các workspace vẫn
> giao tiếp được với nhau qua DDS (dùng cùng `ROS_DOMAIN_ID`).

### 1. Prerequisites

| Thành phần | Phiên bản / ghi chú |
|---|---|
| Hệ điều hành | Ubuntu 24.04 |
| ROS 2 | **Jazzy** (`ros-jazzy-desktop`) |
| Isaac Sim | Bản phát hành gần đây bất kỳ có bật extension ROS 2 Bridge — [hướng dẫn cài đặt](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html) · [cài đặt ROS 2](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html) |
| Công cụ build | `colcon`, `rosdep`, `git` |
| Python (perception) | numpy, opencv-python, ultralytics, scipy |

> ROS 2 Bridge của Isaac Sim phải được cấu hình cho Jazzy (đặt bridge dùng ROS hệ
> thống của bạn, hoặc nạp bản ROS 2 trước khi khởi chạy Isaac).

### 2. Workspace layout

Chỉ bốn gói sau thuộc về dự án cánh tay — mọi thứ còn lại là phụ thuộc thượng
nguồn tiêu chuẩn mà bạn cài riêng (xem §3):

| Gói | Là gì |
|---|---|
| `ra_ws/src/so_arm_description` | URDF + mesh của SO-ARM 101 |
| `ra_ws/src/so_arm_moveit_config` | Cấu hình MoveIt 2 (SRDF, động học, OMPL, bộ điều khiển, `ros2_control`) |
| `ra_ws/src/so_arm_perception` | Các node perception cho cốc + khay + AprilTag (YOLO / HSV / OpenCV) |
| `ra_ws/src/mtc_tutorial` | `mtc_node` — pipeline gắp → servo → hứng → đặt, kèm các launch file |

Scene Isaac Sim (robot, bàn, cốc, khay, máy lọc, camera) nằm dưới thư mục
`simulation/` của repo — hãy mở nó trong Isaac Sim trước khi chạy.

### 3. Install dependencies

#### 3.1 ROS 2 Jazzy + MoveIt 2
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-moveit \
     ros-jazzy-topic-based-ros2-control \
     ros-jazzy-joint-trajectory-controller \
     ros-jazzy-position-controllers \
     ros-jazzy-joint-state-broadcaster \
     python3-colcon-common-extensions python3-rosdep
```

#### 3.2 MoveIt Task Constructor (MTC)
MTC dẫn dắt pipeline gắp. Nếu có sẵn gói nhị phân cho Jazzy:
```bash
sudo apt install ros-jazzy-moveit-task-constructor-core
```
Nếu không, hãy clone nó vào `ra_ws/src/` và để `colcon` build:
```bash
cd ra_ws/src && git clone -b jazzy https://github.com/moveit/moveit_task_constructor.git
```

#### 3.3 Perception Python packages
```bash
python3 -m pip install "numpy>=1.24" "opencv-python>=4.8" "ultralytics>=8.3" \
     "scipy>=1.11" "pupil-apriltags>=1.0"
```
`cv_bridge` lấy từ apt: `sudo apt install ros-jazzy-cv-bridge`.
Trọng số YOLO (`yolo11n.pt`) tự tải về ở lần chạy đầu tiên; không cần bước thủ
công nào. `pupil-apriltags` cung cấp bộ nhận dạng AprilTag mà `apriltag_node` dùng
để định vị fiducial trên máy lọc — gói này là bắt buộc (launch perception luôn khởi
động node đó).

#### 3.4 Resolve the rest with rosdep
```bash
cd ra_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
cd ra_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Isaac Sim setup (the ROS contract)

Phía ROS nói chuyện với Isaac Sim qua **`topic_based_ros2_control`**. Hãy mở scene
và đảm bảo action graph ROS 2 của nó publish/subscribe đúng các topic sau:

| Chiều | Topic | Kiểu | Ghi chú |
|---|---|---|---|
| Isaac → ROS | `/isaac_joint_states` | `sensor_msgs/JointState` | cả 6 khớp: `Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw` |
| ROS → Isaac | `/isaac_joint_commands` | `sensor_msgs/JointState` | lệnh vị trí; đưa các khớp tới đây |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | mọi thứ chạy với `use_sim_time:=true` |
| Isaac → ROS | RGB + `camera_info` của camera trên | `sensor_msgs/Image`, `CameraInfo` | camera phía trên (namespace `top_cam`) |
| Isaac → ROS | RGB + depth + `camera_info` của camera tay | `sensor_msgs/Image`, `CameraInfo` | camera gắn trên tay (namespace `arm_cam`) |

Namespace camera là tham số của perception node (`camera_eth_ns` = `top_cam`,
`camera_eih_ns` = `arm_cam`) — hãy chỉnh topic camera của Isaac cho khớp, hoặc ghi
đè các tham số này.

**Các bước:**
1. Mở scene cánh tay (trong `simulation/`) bằng Isaac Sim.
2. Xác nhận extension ROS 2 Bridge đã bật và được đặt cho ROS 2 Jazzy.
3. Bấm **Play** (vật lý + render product của camera phải đang chạy, nếu không
   `/isaac_joint_states` và các topic camera sẽ im lặng).
4. Trong một terminal đã source, kiểm chứng contract:
   ```bash
   ros2 topic hz /isaac_joint_states
   ros2 topic hz /clock
   ros2 topic list | grep -E "top_cam|arm_cam"
   ```

### 6. Run the pipeline

> **⚠️ Isaac Sim phải chạy trước.** Hãy mở scene cánh tay và bấm **Play**
> (xem §5) *trước* lệnh dưới đây. `mtc_node` chặn tại
> `/detected_object/position`, còn `move_group` + các bộ điều khiển cần
> `/isaac_joint_states` và `/clock` — sẽ không có gì chuyển động cho tới khi Isaac
> chạy và phát những topic đó.

Một lệnh duy nhất dựng MoveIt (`move_group` + bộ điều khiển + RViz), perception và
`mtc_node`, được xếp lệch nhau để phụ thuộc của mỗi lớp lên trước:

```bash
source install/setup.bash
ros2 launch mtc_tutorial bringup.launch.py
```

Điều gì xảy ra:
1. `move_group`, `ros2_control`, và các bộ điều khiển `arm_group` / `hand_group` khởi động.
2. Perception khởi động (bộ nhận dạng cốc YOLO, bộ nhận dạng khay hồng, bộ nhận dạng AprilTag).
3. `mtc_node` chờ `/detected_object/position`, rồi với mỗi cốc:
   di chuyển thô → visual servoing tới cốc → đóng kẹp → mang tới máy lọc có AprilTag
   → nghiêng/ấn để "hứng nước" → đặt vào khay.

**Số lượng cốc** không được cấu hình — nó bằng đúng số cốc mà **camera trên** nhận
ra lúc bắt đầu (nếu không có cốc nào, mặc định là một), và chúng được rải đều trên
khay (một cốc duy nhất sẽ được đặt ở giữa).

### 7. Key configuration (launch args)

Tất cả đều là tham số của `pick_place_demo.launch.py` (hãy truyền chúng qua
`bringup`):

| Tham số | Mặc định | Mục đích |
|---|---|---|
| `skip_servo` | `false` | `false` = chạy servo camera tay dựa trên ảnh (IBVS); `true` = bỏ qua (gắp thẳng open-loop) |
| `grasp_yaw_bias` | `-0.5` | góc tiếp cận sao cho cốc lọt vào khe của gripper một má |
| `servo_grasp_z` | `0.05986` | độ cao gắp ngang (giữa thân cốc) |
| `dispenser_standoff` | `0.10` | khoảng lùi khỏi tag trước khi ấn vào |
| `dispenser_fill_depth` | `-0.08` | cốc dừng ở đâu so với tag (số âm = dừng sớm hơn) |

Ví dụ: `ros2 launch mtc_tutorial bringup.launch.py servo_grasp_z:=0.05`.

### 8. Troubleshooting

<div class="table-even" markdown>

| Triệu chứng | Nguyên nhân / cách khắc phục |
|---|---|
| Cánh tay không hề nhúc nhích | Isaac chưa ở chế độ **Play**, hoặc Isaac chưa subscribe `/isaac_joint_commands`. Kiểm tra `ros2 topic echo /isaac_joint_commands`. |
| `mtc_node` treo lúc khởi động | Nó đang chặn chờ `/detected_object/position` — perception chưa publish. Kiểm tra các topic camera có dữ liệu và khớp với tham số `camera_*_ns`. |
| Camera im lặng | Render product / OmniGraph camera của Isaac không hoạt động trong lúc Play. |
| Lập kế hoạch lỗi "Start state out of bounds" | Một khớp dừng lại hơi quá giới hạn trong URDF; node sẽ đặt lại trước khi nhả, nhưng hãy kiểm tra `joint_limits.yaml`. |
| Mọi thứ chậm chạp / thời gian nhảy cóc | `/clock` không được publish, hoặc một node khởi động mà thiếu `use_sim_time:=true`. |

</div>

### 9. Notes for maintainers

- Toàn bộ stack cánh tay chạy theo **thời gian mô phỏng** (`use_sim_time:=true`); hãy giữ `/clock` luôn chảy.
- Gripper mở qua action GripperCommand `hand_group_controller/gripper_cmd`; cánh
  tay nhận lệnh trên `/arm_group_controller/joint_trajectory`.
- Camera phía trên của perception cũng publish TF tĩnh `world → top_sim_camera`
  dùng cho ray-plane unprojection (trong `perception.launch.py`).
