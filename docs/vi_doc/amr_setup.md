# AMR Setup (`jetracer_ws`) — JetRacer SLAM + Nav2 in Isaac Sim

Xe tự hành lái kiểu Ackermann: dựng bản đồ căn phòng, tự định vị, rồi chạy giữa
các dock để hoàn thành đơn từ orchestrator web.
Mọi thứ ở đây chạy trên **workstation** và điều khiển **Isaac Sim** — repo này chưa
có firmware chạy trên chính chiếc JetRacer, nên cùng stack đó sau này sẽ điều khiển
khung xe thật khi đã có driver đọc `/ackermann_cmd`.

> **⚠️ Bản phân phối ROS:** `jetracer_ws` nhắm tới **ROS 2 Humble** và được
> build/chạy bên trong container **`Dockerfile.dev`** của repo — *không phải* bản
> Jazzy cài native mà `ra_ws` dùng. Các workspace vẫn giao tiếp với nhau qua DDS
> (dùng cùng `ROS_DOMAIN_ID`).

### 1. Prerequisites

| Thành phần | Phiên bản / ghi chú |
|---|---|
| Hệ điều hành (máy chủ) | Khuyến nghị Ubuntu 22.04 |
| ROS 2 | **Humble** (do image `Dockerfile.dev` cung cấp) |
| Docker | kèm NVIDIA Container Toolkit (chuyển tiếp GPU cho Isaac Sim) |
| Isaac Sim | Bản phát hành gần đây bất kỳ có bật extension ROS 2 Bridge |
| Công cụ build | `colcon`, `rosdep`, `git` |
| Dependency thêm | `slam_toolbox`, `nav2`, `pointcloud_to_laserscan` (qua `rosdep`) |

> Stack AMR được thiết kế để build bên trong container nhằm ghim chặt bộ công cụ
> Humble và các dependency. Bản thân Isaac Sim chạy trên máy chủ (hoặc trong
> container riêng của nó) và nói chuyện với workspace qua đồ thị ROS 2.

### 2. Workspace layout

Các package của riêng dự án AMR. Lưu ý rằng **tên package** mới là thứ bạn truyền cho
`ros2 launch` / `ros2 run` — một số package nằm trong các thư mục gom nhóm
(`navigation/`, `ackermann_control/`), nên tên thư mục và tên package khác nhau.

| Package (dùng tên này) | Thư mục | Là gì |
|---|---|---|
| `carter_navigation` | `navigation/carter_navigation` | Dựng Nav2, tham số, bản đồ, chuyển đổi lidar→scan (phỏng theo mẫu Isaac carter) |
| `slam_custom` | `navigation/slam_custom` | Dựng SLAM — bọc `slam_toolbox` online-async + một cấu hình rviz sẵn có |
| `isaac_ros_navigation_goal` | `navigation/isaac_ros_navigation_goal` | bộ gửi goal (`SetNavigationGoal`) → `NavigateToPose` + `/initialpose` |
| `cmdvel_to_ackermann` | `ackermann_control/cmdvel_to_ackermann` | bridge `/cmd_vel` (Twist) → `/ackermann_cmd` (AckermannDriveStamped) |
| `isaacsim` | `isaacsim` | Bộ khởi chạy Isaac Sim (`run_isaacsim.launch.py`) |
| `isaac_ros2_messages` | `isaac_ros2_messages` | các kiểu message cho bridge Isaac |
| `isaac_compressed_image_decoder` | `isaac_compressed_image_decoder` | luồng ảnh nén của Isaac → `sensor_msgs/Image` thô (exec `decoder_node`) |

`src/slam_toolbox/` **không phải một package** — đó là nơi các bản đồ đã serialize
được lưu về (`map_*.pgm` / `map_*.yaml`). Bản thân package `slam_toolbox` là một phụ
thuộc upstream do `rosdep` giải quyết.

| Vendored (chỉ để tham khảo, không phải phần lõi) | Ghi chú |
|---|---|
| `iw_hub_navigation` | Mẫu điều hướng AMR iw.hub của Isaac |
| Các launch đa robot của `carter_navigation` (`multiple_robot_*`) | demo nhiều Carter cho bệnh viện/văn phòng |
| `isaac_tutorials` | Các bộ publish mẫu ROS 2 của Isaac + cấu hình rviz |
| `h1_fullbody_controller` (`humanoid_locomotion_policy_example`), `custom_message` | Mẫu robot hình người H1 + khung sườn `SampleMsg` — không liên quan |

### 3. Build the workspace

Bên trong container `Dockerfile.dev`, với repo đã được mount:

```bash
cd jetracer_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> `run_workstation.sh` (trong `jetracer_ws/`) là điểm vào tiện lợi, khởi chạy
> container với đúng các mount, cờ GPU và cấu hình DDS (`fastdds.xml.template`).
> Hãy ưu tiên dùng nó thay vì tự gõ `docker run`.

### 4. Isaac Sim setup (the ROS contract)

AMR nói chuyện với Isaac Sim qua các topic sau — hãy xác nhận action graph ROS 2
của scene publish/subscribe đúng chúng:

| Chiều | Topic | Kiểu | Ghi chú |
|---|---|---|---|
| Isaac → ROS | `/chassis/odom` | `nav_msgs/Odometry` | odometry gốc; Nav2 dùng để định vị |
| Isaac → ROS | `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | lidar 3D; `carter_navigation` chạy một node `pointcloud_to_laserscan` để chuyển nó thành `/scan` cho SLAM/Nav2 |
| ROS → Isaac | `/cmd_vel` | `geometry_msgs/Twist` | lệnh vận tốc do Nav2 phát ra |
| ROS → Isaac | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | lệnh Ackermann (từ `cmdvel_to_ackermann`) |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | mọi thứ chạy với `use_sim_time:=true` |
| Isaac → ROS | RGB camera (đã nén) | `sensor_msgs/CompressedImage` | được giải mã bởi `isaac_compressed_image_decoder` (`decoder_node`) |

Bạn có thể khởi động Isaac Sim từ ROS bằng bộ khởi chạy `isaacsim` (hoặc mở stage
thủ công trong giao diện Isaac):

```bash
ros2 launch isaacsim run_isaacsim.launch.py \
    gui:=/path/to/your_scene.usd \
    play_sim_on_start:=true \
    ros_distro:=humble
```

Sau đó kiểm chứng contract trong một terminal đã source:
```bash
ros2 topic hz /chassis/odom
ros2 topic hz /clock
ros2 topic echo /front_3d_lidar/lidar_points --once
```

### 5. Map the space (SLAM)

> **⚠️ Isaac Sim phải chạy trước.** Hãy mở scene AMR và bấm **Play** (xem §4)
> *trước* khi khởi chạy SLAM hay Nav2. Các node cần `/clock`, `/chassis/odom` và
> luồng lidar — chúng sẽ treo (hoặc SLAM không bao giờ dựng nổi bản đồ) cho tới khi
> Isaac chạy và phát những topic đó.

`slam_custom` chạy `slam_toolbox` chế độ online-async kèm một cấu hình rviz sẵn có.
Nó tự nhận sim time và chờ `startup_delay` giây để đồng hồ ổn định.

```bash
source install/setup.bash
ros2 launch slam_custom slam_custom.launch.py
```

| Tham số | Mặc định | Mục đích |
|---|---|---|
| `slam_params_file` | `slam_custom/params/slam_toolbox_params.yaml` | cấu hình slam_toolbox |
| `startup_delay` | (xem launch file) | số giây chờ trước khi khởi động slam_toolbox để đồng hồ mô phỏng đã sống |

Chạy robot vòng quanh (teleop hoặc `/cmd_vel`), rồi serialize bản đồ:
```bash
ros2 run slam_toolbox serialize_map -f my_map
```
Bản đồ đã lưu nằm dưới `src/slam_toolbox/` (`map_*.pgm` / `map_*.yaml`).

### 6. Localize + navigate (Nav2)

Với một bản đồ đã lưu, hãy dựng Nav2 lên (định vị AMCL + các server
planner/controller/BT + chuyển đổi lidar→scan):

```bash
source install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/absolute/path/to/your_map.yaml
```

| Tham số | Mặc định | Mục đích |
|---|---|---|
| `map` | `carter_navigation/maps/carter_warehouse_navigation.yaml` | bản đồ cần nạp — hãy ghi đè bằng bản đồ của bạn |
| `params_file` | `carter_navigation/params/...` | tham số Nav2 |
| `use_sim_time` | `true` | dùng đồng hồ của Isaac Sim |

> Có các launch file anh em cho những cấu hình khác:
> `carter_navigation_isaacsim.launch.py` (gộp luôn stage Isaac),
> `carter_navigation_individual.launch.py`, và các demo `multiple_robot_*`
> (vendored, đa robot — không phải hướng đi của JetRacer).

Gửi một goal / set pose ban đầu:
```bash
# bộ gửi goal một lần
ros2 run isaac_ros_navigation_goal SetNavigationGoal
# hoặc qua launch file của nó (đọc goal từ file cấu hình)
ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py
```
Bạn cũng có thể đặt một **2D Pose Estimate** (publish `/initialpose`) và một
**Nav2 Goal** từ rviz.

### 7. Drive interface (Ackermann)

Bộ điều khiển của Nav2 phát `/cmd_vel` (Twist); JetRacer kiểu ô tô, nên
`cmdvel_to_ackermann` chuyển nó thành `/ackermann_cmd`:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py
# (tương đương: ros2 run cmdvel_to_ackermann cmdvel_to_ackermann.py)
```

Nó subscribe `/cmd_vel` → publish `/ackermann_cmd`, đồng thời chặn các lệnh
Ackermann không hợp lệ (vận tốc thẳng bằng không kèm góc lái khác không).

### 8. Orchestrator integration (status)

Orchestrator web (`orchestrator/robot_web_bridge`) sở hữu một dispatcher duy
nhất, điều khiển AMR đi từ dock này sang dock khác. Mối nối này giờ đã được đấu ở
**cả hai** đầu:

| Hợp đồng | Trạng thái |
|---|---|
| Orchestrator publish `/dock_robot`, đọc `/docking_state`, `/chassis/odom`, publish `/initialpose` | ✅ Đã dựng (phía orchestrator) |
| Một **bên đọc** `/dock_robot` trên AMR (dock id → goal Nav2 / hành vi docking) | ✅ Đã hiện thực — `jetracer_bringup/scripts/jetracer_docker.py` |
| Một **bên phát** `/docking_state` thật trên AMR | ✅ Đã hiện thực — `jetracer_docker.py` publish các chuỗi pha thật |
| Chuyển giao RA ↔ AMR (khay sẵn sàng → AMR khởi hành) | ❌ Chưa đấu — cánh tay không nằm trong vòng lặp của orchestrator |

Phần việc còn để ngỏ là phía **cánh tay**. Xem
[Gắp và giao](solution_pick_and_deliver.md) để biết trạng thái đầy đủ, và
[Sổ tay API](api/ros-jetracer.md) để biết topic contract.

### 9. Troubleshooting

| Triệu chứng | Nguyên nhân / cách khắc phục |
|---|---|
| Robot không hề nhúc nhích | Isaac chưa ở chế độ **Play**, hoặc bộ mô phỏng chưa subscribe `/cmd_vel` / `/ackermann_cmd`. |
| Bản đồ SLAM trống rỗng / trôi dạt | Không có `/scan` — kiểm tra node `pointcloud_to_laserscan` đã lên và `/front_3d_lidar/lidar_points` đang có dữ liệu; hoặc thiếu `/clock` khiến các node dùng `use_sim_time` bị treo. |
| Nav2 không định vị được | Chưa publish `/initialpose`, hoặc bản đồ không khớp với scene — hãy dựng lại bản đồ. |
| `slam_custom` khởi động trước đồng hồ | Hãy tăng `startup_delay`; slam_toolbox cần `/clock` sống trước đã. |
| Mọi thứ chậm chạp / thời gian nhảy cóc | `/clock` không được publish, hoặc một node khởi động mà thiếu `use_sim_time:=true`. |
| Các bên DDS không thấy nhau | `ROS_DOMAIN_ID` không khớp, hoặc `fastdds.xml` chưa được áp dụng đồng nhất giữa các container. |

### 10. Notes for maintainers

- Toàn bộ stack AMR chạy theo **thời gian mô phỏng** (`use_sim_time:=true`); hãy giữ `/clock` luôn chảy.
- Lidar tới được SLAM/Nav2 dưới dạng `/scan`, do một node `pointcloud_to_laserscan`
  bên trong `carter_navigation` tạo ra từ `/front_3d_lidar/lidar_points`.
- Docking được kích hoạt bằng một **topic contract** (`/dock_robot` +
  `/docking_state`); `jetracer_docker.py` đọc nó và điều khiển action server
  `opennav_docking` của Nav2. Phần việc còn để ngỏ chính giờ là **chuyển giao
  RA ↔ AMR**.
- Bản đồ mặc định của `carter_navigation` là mẫu nhà kho — hãy cung cấp file
  `.yaml` bạn tự dựng qua `map:=` cho không gian thật.
- Một số mẫu vendored (`iw_hub_navigation`, `multiple_robot_*`,
  `h1_fullbody_controller`, `custom_message`, `isaac_tutorials` không dùng tới) chỉ
  để tham khảo và có thể được cắt bỏ.
