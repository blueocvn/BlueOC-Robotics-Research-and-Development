# jetracer_ws third-party setup

`src/` trộn lẫn **các package của chính dự án** (được commit vào repo này) với **các
package ROS 2 Humble của bên thứ ba** *không* được commit. Mã nguồn bên thứ ba được
khôi phục từ các bản phát hành upstream đã ghim bằng [vcstool], giúp repo gọn
nhẹ mà vẫn tái tạo được chính xác từng byte.

## Fresh-machine setup

```bash
cd amr/jetracer_ws

# 1. Khôi phục các package bên thứ ba đã ghim theo tag  (cần: sudo apt install python3-vcstool)
vcs import src < thirdparty.repos

# 2. Áp lại các bản vá cục bộ lên mã nguồn vừa khôi phục
patch -p1 -d src/navigation2        < patches/nav2_util-bond-shared_ptr.patch
patch -p1 -d src/robot_localization < patches/robot_localization-ekf-jetracer-tune.patch

# 3. Build
colcon build --symlink-install
```

## What lives where

**Được commit trong git**
- Của chính dự án: `jetracer_bringup`, `jetracer_description`, `jetracer_driver`
- Bên thứ ba không có tag phát hành upstream (ghim tag không đáng tin nên
  phải vendored): `gscam2`, `m-explore-ros2`, `nav2_graceful_controller`,
  `navigation_msgs` (map_msgs)
- Bên thứ ba mà việc ghim tag sẽ **làm robot tệ đi**, nên phải vendored:
  - `apriltag` — bản đang dùng mới hơn `v3.4.5` (bổ sung aruco + ước lượng pose của tag)
  - `rplidar_ros` — nhánh ROS 2 có hỗ trợ C1; các tag upstream đều là ROS 1

**Được khôi phục qua `thirdparty.repos`** (mỗi package ghim theo một tag phát hành và
được kiểm chứng từng byte so với tag đó; xem file ấy để biết URL/phiên bản chính
xác): `navigation2` 1.1.5, `robot_localization` 3.5.4, `angles`, `apriltag_msgs`,
`BehaviorTree.CPP`, `bond_core`, `diagnostics`, `geographic_info`,
`laser_geometry`, `teleop_twist_keyboard`.

## Local patches

So sánh từng byte giữa cây làm việc và tag upstream đã ghim xác nhận cả hai đều
là thay đổi thật:

- `patches/nav2_util-bond-shared_ptr.patch` — thay đổi cục bộ duy nhất của nav2
  1.1.5: thành viên `bond_` của nav2_util đổi từ `unique_ptr` -> `shared_ptr`.
- `patches/robot_localization-ekf-jetracer-tune.patch` — trong
  `params/ekf.yaml` của robot_localization 3.5.4: `frequency` 30 -> 20 Hz và
  `base_link_frame` -> `base_footprint`.

Các sửa đổi lạc lối do IDE "optimize imports" (đường dẫn import tuyệt đối trong
`nav2_smac_planner/lattice_primitives`, `bond_core/bondpy`,
`geographic_info/geodesy`) **không phải** thay đổi thật, nên đã bị loại bỏ có chủ đích —
chạy lại `vcs import` sẽ khôi phục đúng các import upstream.

[vcstool]: https://github.com/dirk-thomas/vcstool
