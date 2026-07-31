# Điểm khởi chạy

Chạy launch file nào, và mỗi file giả định sẵn có những gì. Nhầm chỗ này là cách
phổ biến nhất để mất một giờ trong hackathon.

## JetRacer

Stack của AMR được tách có chủ đích thành lớp **phần cứng** và lớp **điều hướng**,
để bạn khởi động lại Nav2 mà không cần tắt bật nguồn khung xe.

| Launch file | Dựng lên | Giả định đã chạy sẵn |
|---|---|---|
| `hardware.launch.py` | Driver, lidar, EKF, TF tĩnh, camera CSI | Không cần gì |
| `nav_bringup.launch.py` | `map_server`, AMCL, controller, planner, `bt_navigator`, lifecycle manager | Lớp phần cứng |
| `mapping_bringup.launch.py` | Dựng bản đồ tự động (không định vị) | Lớp phần cứng, **cộng thêm** một stack SLAM trên máy khác publish `/map` và TF `map→odom` |
| `jetracer.launch.py` | Cả phần cứng **và** điều hướng cùng lúc | Không cần gì |

```bash
# tách rời: hai terminal, khởi động lại nav mà không đụng khung xe
ros2 launch jetracer_bringup hardware.launch.py
ros2 launch jetracer_bringup nav_bringup.launch.py map:=/path/to/map.yaml

# hoặc tất cả cùng lúc
ros2 launch jetracer_bringup jetracer.launch.py map:=/path/to/map.yaml
```

### Tham số

| Launch file | Tham số |
|---|---|
| `hardware.launch.py` | `ekf_params_file`, `base_port`, `lidar_port` |
| `nav_bringup.launch.py` | `map`, `params_file` |
| `mapping_bringup.launch.py` | `params_file`, `explore_params_file` |
| `jetracer.launch.py` | `map`, `params_file`, `ekf_params_file`, `base_port`, `lidar_port` |

!!! note "Cổng serial không cố định qua mỗi lần khởi động lại"

    `base_port` và `lidar_port` mặc định trỏ tới đường dẫn `/dev/tty*` cố định,
    nhưng Linux gán chúng theo thứ tự liệt kê — cắm lidar trước có thể làm hoán
    đổi hai cổng. Nếu bringup lỗi serial, hãy kiểm tra thiết bị thật trước khi gỡ
    bất cứ thứ gì khác:

    ```bash
    ls -l /dev/serial/by-id/
    ```

### Camera CSI

`hardware.launch.py` khởi động `gscam2` với một pipeline GStreamer mặc định cho
IMX219, có thể ghi đè qua `$GSCAM_CONFIG`.

!!! warning "gscam2 từ chối `bgr8`"

    Pipeline phải kết thúc bằng `format=RGB`. `gscam2` publish `rgb8`, và một
    pipeline BGR đơn giản là không liên kết được với appsink của nó — kèm theo một
    thông báo lỗi không hề chỉ rõ nguyên nhân là định dạng. Nếu bạn ghi đè
    `$GSCAM_CONFIG`, hãy giữ phần kết RGB, hoặc bỏ hẳn biến này để dùng mặc định.

## Cánh tay robot

| Launch file | Mục đích |
|---|---|
| `mtc_tutorial/bringup.launch.py` | Dựng cánh tay — mô phỏng |
| `mtc_tutorial/bringup_real.launch.py` | Dựng cánh tay — phần cứng thật |
| `mtc_tutorial/real_all.launch.py` | Toàn bộ stack phần cứng thật: bringup + perception + MTC |
| `mtc_tutorial/mtc_demo.launch.py` | Demo tác vụ MTC |
| `mtc_tutorial/pick_place_demo.launch.py` | Demo gắp và đặt |
| `so_arm_perception/perception.launch.py` | Nhận dạng YOLO, unprojection, AprilTag, nhận dạng quai |
| `so_arm_perception/tracking.launch.py` | Visual servoing tracker |
| `so_arm_perception/top_cam_view.launch.py` | Xem camera trên — dùng khi hiệu chuẩn |

```bash
# mô phỏng
ros2 launch mtc_tutorial bringup.launch.py

# phần cứng thật, tất cả cùng lúc
ros2 launch mtc_tutorial real_all.launch.py
```

### Tham số của `real_all.launch.py`

Điểm khởi chạy cho toàn bộ stack. Nhóm theo thứ chúng điều khiển.

**Bật/tắt giai đoạn**

| Tham số | Ý nghĩa |
|---|---|
| `run_mtc` | Khởi động pipeline MTC |
| `run_sensing` | Khởi động stack perception |
| `fake_apriltag` | Publish pose tag giả thay vì nhận dạng thật |
| `fake_object` | Publish vật giả thay vì nhận dạng thật |
| `obj_x`, `obj_y`, `obj_z` | Vị trí của vật giả |
| `tag_x`, `tag_y`, `tag_z` | Vị trí của tag giả |

**Hình học gắp**

| Tham số | Ý nghĩa |
|---|---|
| `grasp_yaw_bias` | Độ lệch yaw áp dụng cho thao tác gắp |
| `bridge_standoff` | Khoảng lùi tại vị trí rót |
| `place_at_pickup` | Đặt cốc lại đúng chỗ đã gắp lên |
| `place_z` | Độ cao khi đặt |

**Hiệu chỉnh hiệu chuẩn**

| Tham số | Ý nghĩa |
|---|---|
| `eth_plane_z` | Độ cao mặt phẳng giao tia |
| `eth_x_correction`, `eth_y_correction` | Hiệu chỉnh extrinsics thủ công |

**Bám bằng thị giác**

| Tham số | Ý nghĩa |
|---|---|
| `skip_servo` | Bỏ qua hoàn toàn giai đoạn servo |
| `skip_servo_speed` | Tốc độ dùng khi bỏ qua servo |
| `servo_img_u_offset_px` | Hiệu chỉnh lệch ngang trên ảnh |
| `servo_img_p1_pullback` | Khoảng lùi ở pha 1 |
| `servo_grasp_z` | Độ cao gắp trong lúc servo |
| `mtc_delay` | Độ trễ trước khi MTC bắt đầu |

### Tham số của `perception.launch.py`

Phản chiếu các tham số của `perception_node` — xem
[Giao diện cánh tay](ros-arm.md#các-tham-số-quan-trọng) để biết ý nghĩa. Bộ extrinsics là thứ bạn sẽ chỉnh nhiều nhất:

`eth_x`, `eth_y`, `eth_z`, `eth_roll`, `eth_pitch`, `eth_yaw` (eye-to-hand) và
`eih_x` … `eih_yaw` (eye-in-hand).

!!! tip "Dùng đồ giả trước"

    `fake_object` và `fake_apriltag` cho phép bạn chạy toàn bộ pipeline chuyển
    động mà không cần gắn camera. Nếu cánh tay hoạt động sai, hãy chạy với đồ giả
    để xác định vấn đề nằm ở perception hay ở chuyển động, trước khi đụng vào hiệu
    chuẩn.

## Web bridge

```bash
ros2 run robot_web_bridge server        # http://localhost:8088
```

Xem [API HTTP](http.md) để biết các route và đường chạy không cần ROS.

## Xem thêm

- [Giao diện JetRacer](ros-jetracer.md)
- [Giao diện cánh tay](ros-arm.md)
- [Khởi chạy phần cứng thật](../ra_hardware_bringup.md)
