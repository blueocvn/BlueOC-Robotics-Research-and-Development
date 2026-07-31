# Cánh tay robot — Hiệu chuẩn camera

Cánh tay dùng **hai** camera, và chỉ một trong số đó là *eye-to-hand*:

| Camera | Cách gắn | Frame | Vai trò |
|--------|----------|-------|------|
| `top_cam` | **cố định phía trên**, nhìn xuống bàn | `top_sim_camera` | **eye-to-hand** — định vị cốc trong hệ thế giới cho pha vươn thô |
| `arm_cam` | trên gripper | `arm_cam` | eye-in-hand — pha tiếp cận sát bằng visual servoing |

Trang này nói về việc hiệu chuẩn **`top_cam` eye-to-hand**. Đây là công việc
đang chặn tiến độ của việc gắp dựa trên perception (xem ghi chú trong
[Khởi chạy phần cứng thật](ra_hardware_bringup.md#chế-độ-demo-tất-định-vị-trí-định-sẵn));
cho tới khi nó đủ chặt chẽ, hãy chạy bản demo tất định với `fake_object:=true`.

Việc hiệu chuẩn gồm **hai giai đoạn, theo đúng thứ tự** — chúng dùng các bia
*khác nhau*:

1. **Intrinsics** — dùng **bàn cờ**, hiệu chuẩn OpenCV tiêu chuẩn. Cho ra
   `fx, fy, cx, cy` và các hệ số méo.
2. **Extrinsics** (phép biến đổi `world → top_sim_camera`) — dùng **một AprilTag
   duy nhất** đặt tại một pose thế giới đã đo. Nó tiêu thụ intrinsics từ giai đoạn 1,
   nên intrinsics **bắt buộc** phải làm trước.

> **⚠️ Python:** hãy chạy các công cụ hiệu chuẩn ROS bằng trình thông dịch **hệ
> thống** (`/usr/bin/python3`) với ROS đã được nạp và **không có môi trường conda
> nào đang bật** — Python của conda không import được `rclpy`.

---

## Giai đoạn 1 — Intrinsics (bàn cờ)

**Mục tiêu:** giá trị `fx/fy/cx/cy` thật + độ méo ống kính cho `top_cam`, để các
điểm ảnh chiếu ngược ra đúng tia.

**In một bàn cờ.** Hãy ghi lại số **góc bên trong** của nó (một bàn cờ có 9×7 *ô*
sẽ có lưới góc trong **8×6**) và **chiều dài cạnh ô** tính bằng mét. Gắn nó cứng
cáp lên một mặt phẳng.

**1. Chỉ dựng riêng camera** (ở terminal riêng — việc này không đụng tới bus serial
của cánh tay):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash

# liên kết thiết bị top_cam (xem real_all.launch.py); camera_ns quy định các topic.
ros2 run so_arm_perception usb_camera_node --ros-args \
  -p video_device:=/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0 \
  -p camera_ns:=top_cam
```

**2. Chạy trình hiệu chuẩn OpenCV** trên `/top_cam/rgb` (chỉnh `--size` theo số góc
trong của bàn cờ và `--square` theo chiều dài cạnh tính bằng mét):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/top_cam/rgb
```

Di chuyển bàn cờ khắp khung hình — gần/xa, nghiêng, cả bốn góc — cho tới khi **X,
Y, Size, Skew** đầy lên, rồi bấm **CALIBRATE**, sau đó **SAVE**. Kết quả nằm ở
`/tmp/calibrationdata.tar.gz`; mở file `ost.yaml` bên trong để lấy `camera_matrix`
và `distortion_coefficients` (thứ tự plumb_bob là `k1, k2, p1, p2, k3`).

**3. Dán các con số vào tham số của node `top_cam`** trong
`ra_ws/src/mtc_tutorial/launch/real_all.launch.py` (khối `top_cam`) và đặt
`undistort:=true` để node khử méo và publish một `camera_info` không còn méo:

```python
parameters=[{
    "video_device": TOP_CAM_DEV, "camera_ns": "top_cam", "frame_id": "top_sim_camera",
    "fx": 418.38762, "fy": 416.14640, "cx": 325.19068, "cy": 233.94865,   # <- từ ost.yaml
    "undistort": True,
    "d0": -0.300890, "d1": 0.078304, "d2": 0.001265, "d3": -0.001828, "d4": 0.0,  # k1,k2,p1,p2,k3
}]
```

Tên các tham số (`fx/fy/cx/cy`, `d0..d4`, `undistort`) được định nghĩa và giải
thích trong `ra_ws/src/so_arm_perception/so_arm_perception/usb_camera_node.py`.

!!! tip "Kiểm tra intrinsics trước khi đi tiếp"
    Với `undistort:=true`, các cạnh thẳng (mép bàn, một cây thước) phải trông thẳng
    trên **toàn bộ** khung hình, kể cả ở các góc. Nếu vẫn còn cong thì hãy làm lại
    Giai đoạn 1 — intrinsics sai sẽ âm thầm làm hỏng extrinsics vốn phụ thuộc vào nó.

---

## Giai đoạn 2 — Extrinsics (AprilTag): `world → top_sim_camera`

**Mục tiêu:** xác định camera nằm ở đâu trong hệ đế robot (`world`), để tia đi qua
một điểm ảnh được nhận dạng gặp mặt bàn đúng tại điểm thế giới tương ứng. Bước này
dùng script hỗ trợ
`ra_ws/src/so_arm_perception/scripts/calibrate_top_cam_extrinsics.py`, giải bài
toán từ **một AprilTag nằm phẳng tại một pose thế giới đã biết** và tái sử dụng intrinsics của Giai đoạn 1 lấy từ `/top_cam/camera_info`.

!!! note "Vì sao ở đây dùng AprilTag chứ không phải bàn cờ"
    Intrinsics cần nhiều góc nhìn của một lưới góc dày đặc — bàn cờ là lý tưởng.
    Extrinsics cần một bia duy nhất tại một **pose thế giới đã biết**; một AprilTag
    phẳng với tâm đã đo cho ra pose 6 bậc tự do đầy đủ chỉ từ một lần nhận dạng, và
    quy ước frame của nó không gây nhập nhằng. Bộ giải cũng xử lý luôn phép lật
    frame quang học OpenCV→Isaac, thứ khiến một extrinsics `solvePnP` ngây thơ bị
    lộn gương.

**1.** Đặt một AprilTag `tag36h11` **nằm phẳng trên bàn, mặt hướng lên**, cạnh trên
dọc theo trục **+X** thế giới, cạnh phải dọc theo **+Y** thế giới. Đo **tâm** của
nó trong hệ đế `(tag_x, tag_y, tag_z)` và **cạnh** ô vuông đen đã in `tag_size`,
tất cả tính bằng mét.

**2.** Trong khi node `top_cam` từ Giai đoạn 1 vẫn đang publish, hãy chạy bộ giải
(Python hệ thống, ROS đã nạp):

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash

/usr/bin/python3 \
  ~/Desktop/robotics-arm/robotic-arm/ra_ws/src/so_arm_perception/scripts/calibrate_top_cam_extrinsics.py \
  --ros-args -p tag_x:=0.15 -p tag_y:=0.0 -p tag_z:=0.0 -p tag_size:=0.05
```

Nó lấy trung bình khoảng 30 lần nhận dạng rồi in ra phép biến đổi. Nếu nó cảnh báo
*"camera z is BELOW the table"*, hãy chạy lại với `-p tag_z_up:=false`. Nếu bạn
không căn được tag thẳng theo các trục, hãy truyền `-p tag_yaw_deg:=<độ>`. Hãy để
ý **độ lệch chuẩn vị trí** được in ra — vài milimét là tốt; tới hàng centimét nghĩa
là có chói sáng, rung lắc, hoặc `tag_size` sai.

**3.** Lưu lại các giá trị `--x/--y/--z/--roll/--pitch/--yaw` đã giải được. Dán vào
đâu thì tùy cách bạn khởi chạy:

- **Bringup thật đầy đủ (`real_all.launch.py`)** — file này **không** phơi bày
  `eth_x…eth_yaw` dưới dạng tham số dòng lệnh; nó truyền chúng tới perception dưới
  dạng **giá trị viết cứng**. Hãy sửa trực tiếp trong phần include perception cho
  `top_cam` của `ra_ws/src/mtc_tutorial/launch/real_all.launch.py` (khối
  `"eth_x": … "eth_yaw": …`).
- **Chạy perception độc lập (`perception.launch.py`)** — ở đây `eth_x…eth_yaw`
  **đúng là** tham số launch được khai báo, nên bạn có thể sửa giá trị mặc định của
  chúng trong `eth_static_tf` tại
  `ra_ws/src/so_arm_perception/launch/perception.launch.py`, hoặc ghi đè ngay trên
  dòng lệnh để thử nhanh (các giá trị dưới đây là mặc định hiện tại của file đó):

  ```bash
  ros2 launch so_arm_perception perception.launch.py \
    eth_x:=0.02967 eth_y:=0.31201 eth_z:=0.87595 \
    eth_roll:=0.6617028 eth_pitch:=0.0 eth_yaw:=3.1415926536
  ```

---

## Kiểm chứng từ đầu đến cuối

Khi cả hai giai đoạn đã được áp dụng và perception đang chạy, hãy đặt cốc tại một
vị trí thế giới **đã đo bằng thước dây** rồi đọc kết quả nhận dạng:

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /detected_object/position
```

Giá trị `x, y` báo về phải khớp với số đo thước trong khoảng ~1–2 cm. Một độ lệch
không đổi có thể được tỉa bằng các tham số launch `eth_x_correction` /
`eth_y_correction` (trong `ra_ws/src/mtc_tutorial/launch/real_all.launch.py`) thay
vì phải giải lại; còn sai số về *tỉ lệ* hay *góc xoay* nghĩa là Giai đoạn 2 (hoặc
intrinsics của Giai đoạn 1) cần làm lại.

Khi `/detected_object/position` đã bám sát thực tế, hãy bỏ `fake_object:=true` /
`skip_servo:=true` khỏi lệnh bringup để chạy pha gắp dựa trên perception thật sự.
