# Trường hợp sử dụng AMR — Điều hướng & Giao hàng

Nhiệm vụ của AMR trong vòng lặp hoàn thành đơn: dựng bản đồ một không gian, tự
định vị trong đó, rồi chạy từ dock này sang dock khác để chở khay từ máy lọc tới
người dùng. (Một bệ di động không gắp — phần "gắp" do [cánh tay](ra_pick_and_place.md)
đảm nhiệm; AMR lo vận chuyển.) Về cài đặt/build, xem
[Hướng dẫn cài đặt](amr_setup.md).

## Điều kiện tiên quyết

- Workspace đã được build bên trong container `Dockerfile.dev` và Isaac Sim đang
  chạy với hợp đồng ROS đã được kiểm chứng — [Hướng dẫn cài đặt §1–4](amr_setup.md).
- Scene Isaac đang chạy (`/chassis/odom`, `/front_3d_lidar/lidar_points`, `/clock`
  đều có dữ liệu).

## Kịch bản A — Điều hướng trên bản đồ đã biết

Dựng Nav2 lên (định vị AMCL + planner/controller/BT + lidar→scan), rồi gửi goal:

```bash
source install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/absolute/path/to/your_map.yaml
```

Gieo pose và gửi một goal:

```bash
# bộ gửi goal một lần
ros2 run isaac_ros_navigation_goal SetNavigationGoal
```

Hoặc, trong RViz, đặt một **2D Pose Estimate** (gieo `/initialpose`) rồi tới một
**Nav2 Goal**. Bảng tham số đầy đủ ở [Hướng dẫn cài đặt §6](amr_setup.md).

## Kịch bản B — Dựng bản đồ mới trước

```bash
# 1. chạy SLAM (slam_toolbox online-async + rviz)
ros2 launch slam_custom slam_custom.launch.py
# 2. chạy vòng quanh (teleop hoặc /cmd_vel), rồi tuần tự hóa
ros2 run slam_toolbox serialize_map -f my_map
```

Bản đồ đã lưu nằm dưới `src/slam_toolbox/` — xem
[Hướng dẫn cài đặt §5](amr_setup.md).

## Giao tiếp lái

Nav2 phát `/cmd_vel` (Twist); JetRacer kiểu ô tô cần lệnh Ackermann, nên hãy chạy
cầu nối:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py
# /cmd_vel  →  /ackermann_cmd
```

## Docking (hoàn thành đơn)

Giao hàng từ dock tới dock là chặng cuối của vòng lặp, do bộ điều phối điều khiển.
Bên tiêu thụ phía AMR là phần việc còn để ngỏ — xem trạng thái và hợp đồng tại
[Gắp và giao](solution_pick_and_deliver.md) và
[Hướng dẫn cài đặt §8](amr_setup.md).

## Nếu bị treo

- **Robot không hề nhúc nhích** → Isaac chưa ở chế độ Play, hoặc chưa subscribe
  `/cmd_vel` / `/ackermann_cmd`.
- **SLAM trống rỗng / trôi dạt** → không có `/scan` (kiểm tra
  `pointcloud_to_laserscan`), hoặc thiếu `/clock`.
- **Nav2 không định vị được** → chưa gieo `/initialpose`, hoặc bản đồ không khớp
  với scene.

Thêm nữa ở [Hướng dẫn cài đặt §9](amr_setup.md).
