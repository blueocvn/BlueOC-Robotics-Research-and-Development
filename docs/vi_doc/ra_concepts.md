# Robot Arm — Overview

Một **SO-ARM 101** 5 bậc tự do kèm gripper 1 bậc tự do, chạy bằng **ROS 2 + MoveIt 2**,
làm nhiệm vụ gắp một chiếc cốc rỗng, hứng nước tại máy lọc, rồi đặt lên khay để
AMR (JetRacer) mang đi.

`5-DOF + gripper` · `ROS 2 Jazzy` · `MoveIt 2 + MTC` · `Isaac Sim` · `Chạy trên workstation`

## Objectives

- Phát hiện và định vị chính xác các cốc rỗng trên bàn từ camera phía trên.
- Gắp chắc từng cốc dù cánh tay chỉ có 5 bậc tự do và gripper chỉ một má động.
- Tự động đưa cốc tới máy lọc nước và hứng đầy.
- Đặt cốc đã đầy vào đúng ô khay được phân cho AMR (JetRacer) tới lấy.
- Lặp lại với nhiều cốc mà không gắp lại chiếc cốc đã đặt xong.

## Joints

Cánh tay có sáu khớp; cả sáu đều phát dữ liệu trên `/isaac_joint_states`.

| Khớp | Chức năng |
|-------|----------|
| `Rotation` | Xoay đế — quay toàn bộ cánh tay sang trái / phải |
| `Pitch` | Gập vai — nâng và hạ cánh tay trên |
| `Elbow` | Gập khuỷu — vươn dài tầm với của cẳng tay |
| `Wrist_Pitch` | Gập cổ tay — hướng gripper lên / xuống |
| `Wrist_Roll` | Xoay cổ tay — lăn gripper quanh trục của nó |
| `Jaw` | Gripper 1 bậc tự do — mở / đóng để gắp cốc |

## Tech Stack

| Lớp | Lựa chọn | Mục đích |
|-------|--------|---------|
| Giao tiếp & Điều khiển | ROS 2 + MoveIt 2 | Lập kế hoạch chuyển động, điều khiển cánh tay, perception cho gắp/đặt |
| Mô phỏng | Isaac Sim | Phát triển và kiểm chứng sim-to-real |
| HĐH phát triển | Ubuntu 24.04 + ROS 2 Jazzy | Môi trường phát triển native |
| Hệ thống build | colcon, rosdep | Build và quản lý phụ thuộc |
| Tính toán | **Workstation x86 + GPU NVIDIA** | Chạy *toàn bộ* stack — bản thân cánh tay không có máy tính nào |

### Hardware specifications

!!! info "Cánh tay không có máy tính tích hợp"
    SO-ARM 101 là một **thiết bị ngoại vi USB**, không phải một node tính toán.
    Các servo của nó nhận lệnh vị trí qua bus serial từ một **workstation chủ**,
    và máy này lo toàn bộ việc lập kế hoạch, perception và điều khiển:

    ```
    6× servo Feetech STS3215 → BusLinker V3.0 → USB-serial → WORKSTATION CHỦ
    ```

    Khác với JetRacer (AMR) vốn mang sẵn một Jetson trên xe, cánh tay không có
    năng lực tính toán biên nào của riêng nó.

| Phần cứng | Thông số | Ghi chú |
|----------|------|-------|
| Cánh tay robot | SO-ARM 101 — 5 bậc tự do + 1 bậc tự do cho má kẹp | Bộ thao tác chính |
| Gripper | Má kẹp 1 bậc tự do — một má động, một má **cố định** | Cốc bị ép vào má cố định, do đó phải tiếp cận chéo góc (`grasp_yaw_bias`) |
| Servo | 6 × servo bus Feetech STS3215 | Encoder từ tính; nhận lệnh vị trí qua bus serial |
| Giao tiếp cánh tay | BusLinker V3.0 → USB-serial | **Không có máy tính tích hợp** — cánh tay là thiết bị ngoại vi USB |
| Camera | 2 × USB — `top_cam` (phía trên), `arm_cam` (gắn trên tay) | Cấp dữ liệu cho vòng perception + visual servoing |
| **Máy tính chủ** | **Workstation x86, Ubuntu 24.04, GPU NVIDIA** | Bắt buộc cho Isaac Sim và suy luận YOLO; chạy ROS 2 + MoveIt + perception |

!!! info "Ưu tiên mô phỏng, có sẵn đường ra phần cứng thật"
    Cánh tay chạy trong **Isaac Sim** qua `topic_based_ros2_control` khi phát
    triển, và hiện đã có **driver phần cứng thật**: `feetech_ros2_driver` điều
    khiển các servo Feetech vật lý qua BusLinker, khởi chạy bằng
    `real_all.launch.py` (xem [Khởi chạy phần cứng thật](ra_hardware_bringup.md)).
    Trên phần cứng thật, các camera **chưa được hiệu chuẩn** đủ chính xác để gắp
    dựa trên perception, nên bản demo đáng tin cậy chạy open-loop với một vật đã
    định trước — **hiệu chuẩn camera là công việc đang chặn tiến độ** (xem
    [Hiệu chuẩn camera](ra_camera_calibration.md)).

## Packages & Modules

Bốn package tạo nên dự án cánh tay. Mọi thứ còn lại đều là phụ thuộc từ upstream
(xem [Hướng dẫn cài đặt](ra_setup.md)).

??? package "1 · so_arm_description"
    URDF + mesh của SO-ARM 101 — mô tả phần cứng và mô hình động học.

    - URDF (Unified Robot Description Format) + mesh 3D
    - Định nghĩa khớp: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw
    - Mô tả phần cứng và mô hình động học

??? package "2 · so_arm_moveit_config"
    Cấu hình MoveIt 2 cho việc lập kế hoạch và điều khiển.

    - Cấu hình SRDF (Semantic Robot Description Format)
    - Bộ giải động học: IK chỉ theo vị trí, phù hợp tầm với của cánh tay 5 bậc tự do
    - Lập kế hoạch chuyển động: OMPL dùng RRTConnect
    - Tích hợp: `ros2_control` + bộ điều khiển để điều khiển khớp

??? package "3 · so_arm_perception"
    Nhận dạng cốc, khay và máy lọc nước từ camera trên và camera trên tay.

    - Nhận dạng cốc: YOLO (`yolo11n`) kèm phương án dự phòng theo không gian màu HSV cho mô phỏng
    - Nhận dạng khay: bộ nhận dạng khay hồng (định vị dựa trên AprilTag)
    - Nhận dạng máy lọc: bộ nhận dạng AprilTag để định vị máy lọc nước
    - Đầu vào camera: `top_cam` (phía trên), `arm_cam` (gắn trên tay)

??? package "4 · mtc_tutorial"
    Điều phối toàn bộ pipeline gắp và đặt.

    - MTC Node (MoveIt Task Constructor) dẫn dắt pipeline từ đầu đến cuối
    - Các giai đoạn: gắp → servo (bám bằng thị giác) → hứng nước → đặt
    - Các launch file và định nghĩa tác vụ

## The ROS Contract (Isaac Sim)

ROS nói chuyện với Isaac Sim qua `topic_based_ros2_control`. Action graph của
scene phải publish/subscribe đúng những topic sau.

| Chiều | Topic | Kiểu | Ghi chú |
|-----------|-------|------|-------|
| Isaac → ROS | `/isaac_joint_states` | sensor_msgs/JointState | cả 6 khớp |
| ROS → Isaac | `/isaac_joint_commands` | sensor_msgs/JointState | lệnh vị trí; đưa các khớp tới đây |
| Isaac → ROS | `/clock` | rosgraph_msgs/Clock | mọi thứ chạy với `use_sim_time:=true` |
| Isaac → ROS | RGB + camera_info của camera trên | sensor_msgs/Image, CameraInfo | camera phía trên (namespace `top_cam`) |
| Isaac → ROS | RGB + depth + camera_info của camera tay | sensor_msgs/Image, CameraInfo | camera gắn trên tay (namespace `arm_cam`) |

Namespace camera là tham số của perception node (`camera_eth_ns` = `top_cam`,
`camera_eih_ns` = `arm_cam`) — hãy chỉnh topic của Isaac cho khớp, hoặc ghi đè
các tham số này.

## Roadmap

**Giai đoạn 0 — Mô phỏng** (5/5) ✅

- [x] Scene Isaac Sim — cánh tay, bàn, cốc, máy lọc, khay, camera trên + camera trên tay
- [x] Perception trong mô phỏng — YOLO nhận dạng cốc, AprilTag cho máy lọc, nhận dạng khay hồng
- [x] Chiến lược gắp — gắp ngang bằng, IK chỉ theo vị trí, visual servoing IBVS
- [x] **Mô phỏng trọn vẹn quy trình rót nước — nhận dạng → gắp → hứng → đặt**
- [x] Vòng lặp nhiều cốc với phân bổ đều các ô khay

**Giai đoạn 1 — Nền tảng & Cài đặt** (2/5)

- [x] Mua sắm phần cứng: HiWonder LeRobot SO-ARM101
- [x] Dựng môi trường ROS 2 Jazzy (Ubuntu 24.04)
- [ ] Thu thập tập dữ liệu huấn luyện từ thực tế
- [ ] Huấn luyện mô hình YOLO trên dữ liệu thực tế
- [ ] Kiểm chứng động học cánh tay trên phần cứng vật lý

**Giai đoạn 2 — Tích hợp & Kiểm thử** (0/5)

- [ ] Kiểm chứng lập kế hoạch quỹ đạo bằng MoveIt 2
- [ ] Tích hợp pipeline perception (YOLO + AprilTag)
- [ ] Kiểm chứng chuyển giao sim-to-real
- [ ] Kiểm thử tích hợp với AMR (JetRacer)
- [ ] Kiểm thử quy trình giao cốc từ đầu đến cuối

**Giai đoạn 3 — Tối ưu & Triển khai** (0/4)

- [ ] Tinh chỉnh hiệu năng & tối ưu độ trễ
- [ ] Kiểm thử độ bền vững (loại cốc, ánh sáng)
- [ ] Cơ chế xử lý lỗi & phục hồi
- [ ] Triển khai lên môi trường sản xuất
