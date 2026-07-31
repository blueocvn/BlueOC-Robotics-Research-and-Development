# JetRacer (AMR) — Overview

Xe tự hành lái kiểu Ackermann: dựng bản đồ căn phòng, tự định vị, rồi chạy
giữa các dock để giao hàng — tìm đúng user, đi từ máy lọc nước tới chỗ họ, né vật
cản dọc đường. Chạy trên **ROS 2 Humble, SLAM và Nav2**.

`ROS 2 Humble` · `Jetson Nano · 4GB` · `Ackermann` · `SLAM + Nav2` · `Isaac Sim`

!!! warning "Hiện tại mới chỉ chạy mô phỏng"
    Mọi thứ chạy trên **workstation**, điều khiển **Isaac Sim**. Repo chưa có
    firmware nào chạy trên chính chiếc JetRacer. Vẫn stack đó sẽ lái xe thật khi
    nào có driver đọc `/ackermann_cmd`.

## Objectives

- Xác định đúng vị trí của user vừa gọi đồ.
- Tự chở cốc nước đầy từ máy lọc tới chỗ user.
- Né vật cản — bàn ghế, đồ đạc — trong lúc chạy.
- Giao khay tới nơi mà không làm sánh nước.

## Technical Stack

| Thành phần | Công nghệ | Mục đích |
|-----------|-----------|---------|
| Điều hướng & SLAM | ROS 2 + SLAM + Nav2 | Điều hướng, định vị/SLAM, né vật cản |
| Nền tảng phần cứng | JetRacer trên Jetson Nano | Thân xe |
| Cảm biến | LiDAR, encoder bánh xe, camera | Điều hướng, định vị, phát hiện vật cản |
| Giao tiếp lái | Điều khiển Ackermann | `/cmd_vel` → `/ackermann_cmd` |
| Tính toán | Jetson Nano (đời cũ) | Xử lý AI biên với 4 GB RAM |
| HĐH phát triển | Ubuntu 22.04 + ROS 2 Humble (`Dockerfile.dev`) | Môi trường phát triển đóng gói container |

### Dependencies

ROS 2 Humble · Nav2 · `slam_toolbox` · OpenCV · geometry_msgs · message Ackermann · colcon.

## Package map

Các package của riêng AMR. Nhớ truyền **tên package** cho `ros2 launch`, không
phải tên thư mục. Bảng đầy đủ kèm thư mục và mẫu vendored nằm ở
[Hướng dẫn cài đặt §2](amr_setup.md).

??? package "carter_navigation — Dựng Nav2"
    Tham số Nav2, bản đồ, và chuyển đổi `pointcloud_to_laserscan`
    (`/front_3d_lidar/lidar_points` → `/scan`). Phỏng theo mẫu Isaac Carter và
    chỉnh riêng cho JetRacer.

??? package "slam_custom — Dựng SLAM"
    Bọc `slam_toolbox` chế độ online-async kèm một cấu hình RViz sẵn có. Có nhận
    biết thời gian mô phỏng.

??? package "cmdvel_to_ackermann — Giao tiếp lái"
    Chuyển `/cmd_vel` (Twist) của Nav2 → `/ackermann_cmd`
    (AckermannDriveStamped), đồng thời chặn các lệnh không hợp lệ.

??? package "isaac_ros_navigation_goal — Gửi goal"
    Gửi goal `NavigateToPose` và publish `/initialpose`.

!!! note
    `src/slam_toolbox/` **không phải là một gói** — đó là nơi các bản đồ đã tuần
    tự hóa được lưu về (`map_*.pgm` / `map_*.yaml`). Bản thân gói `slam_toolbox`
    là một dependency từ upstream qua `rosdep`.

## The ROS Contract (Isaac Sim)

| Chiều | Topic | Kiểu | Ghi chú |
|-----------|-------|------|-------|
| Isaac → ROS | `/chassis/odom` | nav_msgs/Odometry | odometry gốc cho Nav2 định vị |
| Isaac → ROS | `/front_3d_lidar/lidar_points` | sensor_msgs/PointCloud2 | được chuyển thành `/scan` cho SLAM/Nav2 |
| ROS → Isaac | `/cmd_vel` | geometry_msgs/Twist | lệnh vận tốc từ Nav2 |
| ROS → Isaac | `/ackermann_cmd` | ackermann_msgs/AckermannDriveStamped | lệnh Ackermann |
| Isaac → ROS | `/clock` | rosgraph_msgs/Clock | mọi thứ chạy với `use_sim_time:=true` |
| Isaac → ROS | RGB camera (đã nén) | sensor_msgs/CompressedImage | được giải mã bởi `isaac_compressed_image_decoder` |

## Roadmap & Progress

| Giai đoạn | Trọng tâm | Trạng thái |
|-------|-------|--------|
| **P0** | Scene Isaac Sim + bridge ROS 2 | ✅ Hoàn thành |
| **P1** | Dựng bản đồ bằng SLAM | ✅ Hoàn thành |
| **P2** | Định vị bằng Nav2 + điều hướng theo goal | ✅ Hoàn thành |
| **P3** | Giao tiếp lái + vòng lặp hoàn thành đơn | 🟡 Một phần |
| **P4** | Sim-to-real / firmware trên thiết bị | ⚪ Dự kiến |

- **P3** — `cmd_vel` → Ackermann đã xong, và AMR **có** đọc giao thức
  `/dock_robot` của orchestrator: `jetracer_bringup/scripts/jetracer_docker.py`
  subscribe topic này, điều khiển action server `opennav_docking`, và publish
  chuỗi trạng thái thật lên `/docking_state`. Phần còn thiếu là **chuyển giao
  RA ↔ AMR** — cánh tay chưa nằm trong vòng lặp của orchestrator (xem
  [Gắp và giao](solution_pick_and_deliver.md)).
- **P4** — Repo chưa có firmware JetRacer — workstation mới chỉ điều khiển bản
  mô phỏng.
