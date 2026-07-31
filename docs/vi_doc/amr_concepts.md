# JetRacer (AMR) — Tổng quan

Một robot di động tự hành kiểu ô tô (Ackermann) có khả năng dựng bản đồ không
gian, tự định vị và di chuyển giữa các dock để hoàn thành đơn hàng — tìm ra người
cần phục vụ, chạy từ máy lọc nước tới chỗ họ, và né vật cản dọc đường. Xây trên
**ROS 2 Humble, SLAM và Nav2**.

`ROS 2 Humble` · `Jetson Nano · 4GB` · `Ackermann` · `SLAM + Nav2` · `Isaac Sim`

!!! warning "Hiện tại mới chỉ chạy mô phỏng"
    Mọi thứ chạy trên **workstation** và điều khiển **Isaac Sim** — trong repo này
    chưa có firmware chạy trên chính chiếc JetRacer. Cũng stack đó sau này sẽ điều
    khiển khung xe thật, khi đã có driver tiêu thụ `/ackermann_cmd`.

## Mục tiêu

- Xác định và định vị chính xác vị trí của người đã gửi tín hiệu.
- Tự động vận chuyển cốc nước đã đầy từ Vị trí B (máy lọc) tới Vị trí A (người dùng).
- Phát hiện và né vật cản — đồ nội thất và các vật khác — trong lúc di chuyển.
- Giao khay tới người dùng mà không làm sánh nước.

## Nền tảng kỹ thuật

| Thành phần | Công nghệ | Mục đích |
|-----------|-----------|---------|
| Điều hướng & SLAM | ROS 2 + SLAM + Nav2 | Điều hướng, định vị/SLAM, né vật cản |
| Nền tảng phần cứng | JetRacer trên Jetson Nano | Bệ di động |
| Cảm biến | LiDAR, encoder bánh xe, camera | Điều hướng, định vị, phát hiện vật cản |
| Giao tiếp lái | Điều khiển Ackermann | `/cmd_vel` → `/ackermann_cmd` |
| Tính toán | Jetson Nano (đời cũ) | Xử lý AI biên với 4 GB RAM |
| HĐH phát triển | Ubuntu 22.04 + ROS 2 Humble (`Dockerfile.dev`) | Môi trường phát triển đóng gói container |

### Phụ thuộc

ROS 2 Humble · Nav2 · `slam_toolbox` · OpenCV · geometry_msgs · message Ackermann · colcon.

## Bản đồ các gói

Các gói của riêng AMR (truyền **tên gói**, không phải tên thư mục, cho `ros2
launch`). Bảng đầy đủ kèm thư mục và mẫu vendored nằm ở
[Hướng dẫn cài đặt §2](amr_setup.md).

??? package "carter_navigation — Dựng Nav2"
    Tham số Nav2, bản đồ, và phép chuyển đổi `pointcloud_to_laserscan`
    (`/front_3d_lidar/lidar_points` → `/scan`). Phỏng theo mẫu Isaac Carter và
    chỉnh riêng cho JetRacer.

??? package "slam_custom — Dựng SLAM"
    Bọc `slam_toolbox` chế độ online-async kèm một cấu hình RViz sẵn có. Có nhận
    biết thời gian mô phỏng.

??? package "cmdvel_to_ackermann — Giao tiếp lái"
    Chuyển `/cmd_vel` (Twist) của Nav2 → `/ackermann_cmd`
    (AckermannDriveStamped), đồng thời chặn các lệnh không hợp lệ.

??? package "isaac_ros_navigation_goal — Gửi goal"
    Gửi goal `NavigateToPose` và gieo `/initialpose`.

!!! note
    `src/slam_toolbox/` **không phải là một gói** — đó là nơi các bản đồ đã tuần
    tự hóa được lưu về (`map_*.pgm` / `map_*.yaml`). Bản thân gói `slam_toolbox`
    là một phụ thuộc thượng nguồn qua `rosdep`.

## Hợp đồng ROS (Isaac Sim)

| Chiều | Topic | Kiểu | Ghi chú |
|-----------|-------|------|-------|
| Isaac → ROS | `/chassis/odom` | nav_msgs/Odometry | odometry gốc cho Nav2 định vị |
| Isaac → ROS | `/front_3d_lidar/lidar_points` | sensor_msgs/PointCloud2 | được chuyển thành `/scan` cho SLAM/Nav2 |
| ROS → Isaac | `/cmd_vel` | geometry_msgs/Twist | lệnh vận tốc từ Nav2 |
| ROS → Isaac | `/ackermann_cmd` | ackermann_msgs/AckermannDriveStamped | lệnh Ackermann |
| Isaac → ROS | `/clock` | rosgraph_msgs/Clock | mọi thứ chạy với `use_sim_time:=true` |
| Isaac → ROS | RGB camera (đã nén) | sensor_msgs/CompressedImage | được giải mã bởi `isaac_compressed_image_decoder` |

## Lộ trình & Tiến độ

| Giai đoạn | Trọng tâm | Trạng thái |
|-------|-------|--------|
| **P0** | Scene Isaac Sim + cầu nối ROS 2 | ✅ Hoàn thành |
| **P1** | Dựng bản đồ bằng SLAM | ✅ Hoàn thành |
| **P2** | Định vị bằng Nav2 + điều hướng theo goal | ✅ Hoàn thành |
| **P3** | Giao tiếp lái + vòng lặp hoàn thành đơn | 🟡 Một phần |
| **P4** | Sim-to-real / firmware trên thiết bị | ⚪ Dự kiến |

- **P3** — phần `cmd_vel` → Ackermann đã dựng xong; giao thức `/dock_robot` của
  bộ điều phối **vẫn chưa được AMR tiêu thụ** (xem
  [Gắp và giao](solution_pick_and_deliver.md)).
- **P4** — Repo chưa có firmware JetRacer — workstation mới chỉ điều khiển bản
  mô phỏng.
