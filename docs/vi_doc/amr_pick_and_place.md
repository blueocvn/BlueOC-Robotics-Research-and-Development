# AMR Use Case — Navigate & Deliver

Việc của AMR trong vòng lặp: dựng bản đồ căn phòng, tự định vị, rồi chạy từ dock
này sang dock khác để chở khay từ máy lọc tới chỗ user. (Xe không gắp — phần gắp
là của [cánh tay](ra_pick_and_place.md); AMR chỉ lo chở.) Về cài đặt/build, xem
[Hướng dẫn cài đặt](amr_setup.md).

## Prerequisites

- Workspace đã được build bên trong container `Dockerfile.dev` và Isaac Sim đang
  chạy với ROS contract đã được kiểm chứng — [Hướng dẫn cài đặt §1–4](amr_setup.md).
- Scene Isaac đang chạy (`/chassis/odom`, `/front_3d_lidar/lidar_points`, `/clock`
  đều có dữ liệu).

## Scenario A — Navigate a known map

Dựng Nav2 lên (định vị AMCL + planner/controller/BT + lidar→scan), rồi gửi goal:

```bash
source install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/absolute/path/to/your_map.yaml
```

Set pose và gửi một goal:

```bash
# bộ gửi goal một lần
ros2 run isaac_ros_navigation_goal SetNavigationGoal
```

Hoặc, trong RViz, đặt một **2D Pose Estimate** (publish `/initialpose`) rồi tới một
**Nav2 Goal**. Bảng tham số đầy đủ ở [Hướng dẫn cài đặt §6](amr_setup.md).

## Scenario B — Build a new map first

```bash
# 1. chạy SLAM (slam_toolbox online-async + rviz)
ros2 launch slam_custom slam_custom.launch.py
# 2. chạy vòng quanh (teleop hoặc /cmd_vel), rồi serialize
ros2 run slam_toolbox serialize_map -f my_map
```

Bản đồ đã lưu nằm dưới `src/slam_toolbox/` — xem
[Hướng dẫn cài đặt §5](amr_setup.md).

## Drive interface

Nav2 phát `/cmd_vel` (Twist); JetRacer kiểu ô tô cần lệnh Ackermann, nên hãy chạy
bridge:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py
# /cmd_vel  →  /ackermann_cmd
```

## Docking (fulfillment)

Giao hàng từ dock tới dock là chặng cuối của vòng lặp, do orchestrator điều khiển.
Phía AMR đã làm xong — `jetracer_docker.py` đọc `/dock_robot` và publish
`/docking_state`. Việc còn lại là **chuyển giao RA ↔ AMR**. Xem trạng thái và
contract tại [Gắp và giao](solution_pick_and_deliver.md) và
[Hướng dẫn cài đặt §8](amr_setup.md).

## If it stalls

- **Robot không hề nhúc nhích** → Isaac chưa ở chế độ Play, hoặc chưa subscribe
  `/cmd_vel` / `/ackermann_cmd`.
- **SLAM trống rỗng / trôi dạt** → không có `/scan` (kiểm tra
  `pointcloud_to_laserscan`), hoặc thiếu `/clock`.
- **Nav2 không định vị được** → chưa publish `/initialpose`, hoặc bản đồ không khớp
  với scene.

Thêm nữa ở [Hướng dẫn cài đặt §9](amr_setup.md).
