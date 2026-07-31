# Cánh tay robot — Khởi chạy phần cứng thật

Chạy stack gắp-và-đặt của SO-ARM 101 trên **cánh tay vật lý** thay vì Isaac Sim.
Đây là con đường **sim-to-real**: vẫn pipeline MoveIt / MTC và các node perception
đó, nhưng điều khiển servo Feetech thật qua USB và đọc camera USB thật. Về quy
trình mô phỏng, xem [Hướng dẫn cài đặt](ra_setup.md).

> **⚠️ Bản phân phối ROS:** giống stack mô phỏng — **ROS 2 Jazzy**, build trong
> `ra_ws`. Không phải container Humble mà AMR dùng.

## Khác gì so với mô phỏng

| Lớp | Mô phỏng | Phần cứng thật |
|-------|-----|---------------|
| Khớp | Isaac Sim qua `topic_based_ros2_control` | **`feetech_ros2_driver`** — giao diện phần cứng `ros2_control` qua serial |
| Cổng follower | — | `/dev/ttyACM0` |
| Camera | Render product của Isaac | **`usb_camera_node`** → `/arm_cam/rgb` (+ camera phía trên) |
| Đấu nối khớp | bộ điều khiển mô phỏng | `config/follower_joints.yaml`, `*.ros2_control.xacro`, `ros2_controllers.yaml` |
| Khởi chạy | `bringup.launch.py` | **`real_all.launch.py`** / `bringup_real.launch.py` |

## Các gói liên quan

- **`feetech_ros2_driver`** — giao diện phần cứng servo thật (vendored; xem
  `VENDORED.md` của nó — đây là bản fork của `JafarAbdi/feetech_ros2_driver` có
  chỉnh sửa cục bộ).
- **`mtc_tutorial`** — `mtc_node` (pipeline gắp→hứng→đặt) + `real_all.launch.py`.
- **`so_arm_perception`** — `usb_camera_node` + `top_cam_view.launch.py` cho camera thật.
- **`so_arm_moveit_config`** — cấu hình bộ điều khiển/khớp cho phần cứng thật.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ra_ws
colcon build --symlink-install
source install/setup.bash
```

## Khởi chạy

Một lệnh launch khởi động MoveIt + bộ điều khiển + driver Feetech + perception +
`mtc_node`:

```bash
ros2 launch mtc_tutorial real_all.launch.py
```

### Các tham số launch hữu ích

| Tham số | Mặc định | Mục đích |
|-----|---------|---------|
| `run_sensing` | `true` | khởi động camera + các node perception |
| `fake_object` | `false` | publish một `/detected_object/position` cố định thay vì dùng perception |
| `obj_x` / `obj_y` / `obj_z` | — | pose thế giới của vật giả (phía trước = **−Y**) |
| `skip_servo` | `false` | bỏ qua cầu nối visual servoing — gắp thẳng open-loop |
| `place_z` | (không đặt) | ghi đè độ cao nhả tuyệt đối |
| `tag_x` / `tag_y` / `tag_z` | — | pose AprilTag giả ("mái chèo máy lọc") |
| `bridge_standoff` | `0.08` | khoảng hở so với cốc tại vị trí lùi trước khi gắp |

## Chế độ demo tất định (vị trí định sẵn)

!!! important "Vì sao dùng vị trí định sẵn"
    Trên phần cứng thật, các camera **chưa được hiệu chuẩn đủ chính xác** để định
    vị vật một cách tin cậy, nên việc gắp dựa trên perception vẫn chưa đáng tin —
    **hiệu chuẩn camera là công việc đang chặn tiến độ** (xem
    [Hiệu chuẩn camera](ra_camera_calibration.md)). Để có bản demo trực tiếp lặp
    lại được, hãy chạy open-loop với vật và đích đến **định sẵn**, cách này bỏ qua
    hoàn toàn perception và cầu nối visual servoing.

### Từ một terminal mới — từng bước

**1. Điều kiện tiên quyết (một lần mỗi phiên đăng nhập / khởi động máy).**

```bash
# Bạn phải thuộc nhóm dialout TRONG PHIÊN NÀY, không chỉ trong /etc/group.
id -nG | grep -q dialout && echo "dialout OK" || echo "HÃY ĐĂNG XUẤT RỒI ĐĂNG NHẬP LẠI"
# Không có môi trường conda nào đang bật (Python 3.13 của nó làm hỏng bản build colcon
# của so_arm_perception; ROS Jazzy build dựa trên 3.12). Chạy 'conda deactivate' cho tới
# khi dòng này in ra "(none)".
echo "conda: ${CONDA_DEFAULT_ENV:-none}"
```

Nếu thiếu `dialout`, hãy **đăng xuất rồi đăng nhập lại** (mở terminal mới là chưa
đủ — tư cách thành viên nhóm được cố định khi phiên desktop bắt đầu). Nếu cánh tay
vừa được cắm vào, hãy xác nhận nó hiện ra là `/dev/ttyACM0`.

**2. Nạp ROS, rồi nạp lớp phủ workspace** (thứ tự rất quan trọng):

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robotics-arm/robotic-arm/ra_ws/install/setup.bash
```

**3. Khởi chạy chu trình gắp → chở → đặt tất định:**

```bash
ros2 launch mtc_tutorial real_all.launch.py \
  run_sensing:=false fake_object:=true \
  obj_x:=0.0 obj_y:=-0.37 obj_z:=0.09 \
  skip_servo:=true \
  fake_apriltag:=true tag_x:=0.0 tag_y:=-0.32 tag_z:=0.20 \
  place_z:=0.04 bridge_standoff:=0.08
```

Thành công trông như thế này sau khoảng 10 giây (khi `move_group` và các bộ điều
khiển đã lên):

```
[bridge] standoff joints R=0.013 P=0.993 E=0.629 WP=-1.620 WR=-1.571 (residual 0.00 mm)
==== [1] Planning SUCCEEDED — 1 solution(s) ====
==== [4] Executing ====
```

Cách này cho một chu trình gắp → chở → đặt đáng tin cậy mà không phụ thuộc vào độ
chính xác của camera. Khi hiệu chuẩn đã chặt chẽ hơn, hãy bỏ
`fake_object`/`skip_servo` để quay lại pipeline dựa trên perception.

!!! danger "Chỉ chạy MỘT phiên bản duy nhất"
    `/dev/ttyACM0` là thiết bị USB-CDC, nên một lệnh launch **thứ hai** (hoặc một
    tiến trình sót lại từ lần chạy trước) vẫn *mở* được cổng rồi xung đột với cái
    đầu tiên trên bus servo. Triệu chứng gây hiểu nhầm — trông y như phần cứng đã
    chết:

    ```
    FeetechHardwareInterface … SerialPort::read_exact [Read timeout]
    Failed to initialize hardware 'FakeSystem'
    ```

    → 0/3 bộ điều khiển → không có `/joint_states` → **RViz đóng băng ở pose mặc
    định trong khi cánh tay thật đang ở chỗ khác** → thực thi lỗi với mã `99999`.
    Trước khi launch, hãy chắc chắn không còn gì đang chạy:
    `pgrep -af "ros2_control_node|move_group|mtc_node"`.

!!! note "Cốc phải đủ xa để gắp ngang bằng"
    Vị trí lùi trước khi gắp giữ gripper **nằm ngang** (`Wrist_Pitch =
    −(Pitch+Elbow)`, giới hạn ở ±1.658 rad). Ở độ cao gắp, điều này giam gốc gripper
    trong khoảng chừng **0.22 – 0.35 m** tính từ đế, nên khoảng lùi 8 cm đòi hỏi
    cốc ở khoảng **`obj_y ≤ −0.36`**. Gần hơn (ví dụ giá trị cũ `−0.30`) thì vị trí
    lùi nằm *bên trong* vành với tới được và việc lập kế hoạch sẽ hủy với lỗi
    `GOAL_STATE_INVALID` / va chạm giữa gripper và cốc. `mtc_node` báo rõ trường hợp
    này: *"standoff … is OUTSIDE the level-wrist workspace"*. Để gắp một chiếc cốc
    ở gần hơn, hãy tăng `servo_grasp_z` hoặc giảm `bridge_standoff`.

## Cạm bẫy đã biết — hãy đưa cánh tay về home trước lần lập kế hoạch đầu tiên

!!! warning "Start state out of bounds"
    Bộ điều hợp `CheckStartStateBounds` của MoveIt sẽ **từ chối lập kế hoạch** nếu
    cánh tay bắt đầu ngoài giới hạn khớp trong URDF. Tư thế nghỉ gập lại có thể
    khiến một khớp dừng hơi quá giới hạn (ví dụ `Pitch ≈ −1.84` so với giới hạn
    `−1.745`), sinh ra:

    ```
    Joint 'Pitch' from the starting state is outside bounds …
    PlanningRequestAdapter 'CheckStartStateBounds' failed … Aborting planning pipeline.
    ```

    MTC không thể thực hiện chuyển động *đầu tiên* vì cùng phép kiểm tra đó chặn
    nó lại. **Hãy đưa cánh tay về một pose hợp lệ trước** — gửi thẳng một quỹ đạo
    tới bộ điều khiển (`/arm_group_controller/joint_trajectory`, ví dụ toàn số 0
    trong khoảng 4 giây) trước khi khởi chạy pipeline, hoặc thêm một bước tự động
    về home trước lần lập kế hoạch đầu tiên.

## Ghi chú an toàn

- Không có cảm biến dòng/lực — gripper đóng lại **chỉ dựa vào động học**; nó không
  thể phát hiện một cú gắp hụt hay va chạm. Hãy xem mọi cú gắp là open-loop.
- Cánh tay tự động chuyển động ngay khi `mtc_node` khởi chạy — hãy giữ khu vực làm
  việc thông thoáng và để nguồn điện trong tầm với.
- Thứ tự cổng serial (`/dev/ttyACM*`) không được đảm bảo qua mỗi lần khởi động lại
  — hãy xác nhận follower đang ở `/dev/ttyACM0` trước khi launch.
