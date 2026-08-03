# ROS 2 Interfaces — JetRacer

Topic, service và action cho JetRacer AMR. Dùng chúng khi bạn cần phản hồi ở tốc
độ đầy đủ hoặc mức điều khiển mà [API HTTP](http.md) không cung cấp.

Mọi tên dưới đây là tuyệt đối trừ khi được đánh dấu *tương đối*, khi đó namespace
của node sẽ được áp dụng.

## Command the robot

Đây là các topic bạn publish tới. [API HTTP](http.md) chỉ là một lớp bọc mỏng
quanh đúng những topic này.

| Topic | Kiểu | Publish bởi | Tác dụng |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | bạn, bridge, docker | Lệnh vận tốc trực tiếp |
| `/dock_robot` | `std_msgs/msg/String` | bạn, bridge | Docking tại dock ID được nêu |
| `/dock_sequence` | `std_msgs/msg/String` | bạn | Chạy chuỗi nhiều dock |
| `/undock_robot` | `std_msgs/msg/Bool` | bạn | Rời dock hiện tại |
| `/abort_docking` | `std_msgs/msg/Bool` | bạn, bridge | Hủy lượt docking đang chạy |
| `/relocalize_at_dock` | `std_msgs/msg/String` | bridge | Xoay tìm AprilTag của dock rồi set lại pose |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | bridge, docker | Set pose cho AMCL |

```bash
# gửi robot tới dock0
ros2 topic pub --once /dock_robot std_msgs/msg/String "{data: 'dock0'}"

# dừng tất cả
ros2 topic pub --once /abort_docking std_msgs/msg/Bool "{data: true}"
```

!!! danger "`/cmd_vel` không có cơ chế phân xử"

    Bridge, `jetracer_docker`, Nav2 và node của chính bạn đều publish tới
    `/cmd_vel`, và không có cơ chế nào phân xử. Nếu bạn publish trong khi một
    lượt docking đang chạy, cả hai luồng đều tới động cơ và robot sẽ hành xử
    khó lường. Hãy hủy trước, rồi mới điều khiển.

## Read robot state

| Topic | Kiểu | Publish bởi | Ghi chú |
|---|---|---|---|
| `/docking_state` | `std_msgs/msg/String` | `jetracer_docker` | **Có latch** (transient local) — subscriber vào muộn vẫn nhận được pha hiện tại |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF | Pose ưu tiên trên bản đồ |
| `/chassis/odom` | `nav_msgs/msg/Odometry` | driver | Nguồn dự phòng khi không có EKF |
| `odom` *(tương đối)* | `nav_msgs/msg/Odometry` | `jetracer_driver` | Odometry bánh xe lấy thẳng từ serial |
| `imu` *(tương đối)* | `sensor_msgs/msg/Imu` | `jetracer_driver` | IMU của khung xe |
| `motor/lvel`, `motor/rvel` | `std_msgs/msg/Int32` | `jetracer_driver` | Vận tốc bánh đo được |
| `motor/lset`, `motor/rset` | `std_msgs/msg/Int32` | `jetracer_driver` | Vận tốc bánh đặt ra |

!!! warning "Các chuỗi `/docking_state` do robot định nghĩa"

    API này không cố định các chuỗi mô tả pha — hãy xác nhận với robot thật trước
    khi so khớp:

    ```bash
    ros2 topic echo /docking_state
    ```

    Bridge ánh xạ chúng sang trạng thái hiển thị cho user qua các biến môi
    trường (`ROBOT_WEB_BRIDGE_INPROGRESS_STATES`, `_SUCCESS_STATES`,
    `_ERROR_STATES`).

## Map configuration

Trình biên tập bản đồ của admin đẩy cấu hình xuống robot qua hai topic String
**có latch**, mang nội dung JSON:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/virtual_obstacles` | `std_msgs/msg/String` | Danh sách JSON các hình chữ nhật vùng cấm |
| `/dock_registry` | `std_msgs/msg/String` | Danh bạ dock dạng JSON với `pose_x`, `pose_y`, `yaw` |

Cả hai đều là transient-local, nên một node khởi động muộn vẫn nhận được cấu hình
hiện tại.

## Actions and services

| Tên | Kiểu | Vai trò |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Goal của Nav2 — dùng bởi `jetracer_docker` |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | Gieo lại costmap cục bộ sau khi rời dock |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | Gieo lại costmap toàn cục sau khi rời dock |

Xóa costmap là để cái dock vừa rời không còn bị giữ lại như một vật cản cũ.

## Node parameters

### `jetracer_driver` — `cmd_vel_to_serial`

Chuyển `cmd_vel` xuống khung xe qua serial ở tần số 50 Hz.

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | `/dev/ttyACM0` | Cổng serial tới MCU của khung xe |

### `ackermann_dock_filter`

Chuyển lệnh Twist thành chuyển động khả thi kiểu Ackermann trong lúc docking —
JetRacer không thể xoay tại chỗ, nên lệnh vi sai thuần không dùng được.

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `wheelbase` | `0.20` | Khoảng cách hai trục, mét |
| `delta_max_deg` | `30.0` | Góc lái tối đa |
| `v_min_threshold` | `0.02` | Dưới tốc độ này thì coi như đứng yên |
| `input_topic` | `/docking/cmd_vel` | Twist vào |
| `output_topic` | `/cmd_vel` | Twist ra |

### `dock_pose_publisher`

Publish pose của dock từ kết quả nhận dạng AprilTag thông qua TF.

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `camera_frame` | `camera_optical_frame` | Frame mà kết quả nhận dạng đi tới |
| `dock_frames` | `[dock_0, dock_1, dock_2]` | Các frame TF cần publish pose |
| `detection_topic` | `detected_dock_pose` | `geometry_msgs/msg/PoseStamped` ra |
| `publish_rate` | `15.0` | Hz |
| `detection_timeout` | `0.5` | Số giây trước khi kết quả bị coi là cũ |

## See also

- [API HTTP](http.md) — cùng những lệnh này mà không cần cài ROS
- [Điểm khởi chạy](launch.md) — cách dựng stack lên
