# AMR Setup (`jetracer_ws`) — JetRacer SLAM + Nav2 on device

Waveshare JetRacer kiểu ô tô (Ackermann) chạy ROS 2 Humble: dựng bản đồ không
gian, tự định vị, điều hướng và docking. Trang này nói về
robot thật trong
[`amr/jetracer_ws/`](https://github.com/blueocvn/robotic-arm/tree/main/amr/jetracer_ws):
driver chassis, RPLidar, odometry EKF, Nav2, và docking bằng AprilTag.

> **Phạm vi:** chỉ robot chạy trên phần cứng. Quy trình Isaac Sim
> (`carter_navigation` / `slam_custom` trên workstation) là một stack riêng nằm
> trong `amr/workstation_ws/` và không được nói tới ở đây.

> **⚠️ Bản phân phối ROS:** JetRacer chạy **ROS 2 Humble** native ngay trên thiết
> bị (Jetson). Hãy build tại đó bằng `colcon`; nạp môi trường bằng
> **`ws_setup.bash`**, không phải `install/setup.bash` hợp nhất (xem §3 để biết lý
> do).

### 1. Hardware & prerequisites

| Thành phần | Ghi chú |
|---|---|
| Khung xe | Waveshare **JetRacer** (Ackermann), máy tính Jetson tích hợp |
| HĐH / ROS | JetPack + **ROS 2 Humble** cài native trên Jetson |
| MCU chassis | Serial (mặc định `/dev/ttyACM0`) — động cơ + IMU/con quay; do `jetracer_driver` đọc |
| Lidar | **RPLidar A1** trên `/dev/ttyACM1`, baud 115200, gắn **lộn ngược** (`laser_frame`, yaw π, z ≈ 0,18 m) |
| Camera | CSI **IMX219** ở 640×360 (qua `gscam2`) → detector AprilTag cho docking |
| Công cụ build | `colcon`, `rosdep`, `vcstool` (`sudo apt install python3-vcstool`), `git`, `tmux` |

> Thứ tự thiết bị serial không được đảm bảo qua các lần khởi động lại. Nếu chassis và
> lidar hoán đổi cổng, hãy ghi đè chúng (§7). Giữ robot **đứng yên khoảng 2 giây**
> lúc driver khởi động — con quay hiệu chuẩn khi đó, và chuyển động sẽ làm hỏng
> odometry.

### 2. Restore third-party sources & build

`src/` trộn các package của chính dự án (được commit) với các package ROS 2 bên thứ ba đã
ghim (không commit, khôi phục qua `vcstool`). Trên một thiết bị mới:

```bash
cd amr/jetracer_ws

# 1. Khôi phục các package bên thứ ba đã ghim theo tag
vcs import src < thirdparty.repos

# 2. Áp lại các bản vá cục bộ (nav2 bond shared_ptr; tinh chỉnh EKF cho robot_localization)
patch -p1 -d src/navigation2        < patches/nav2_util-bond-shared_ptr.patch
patch -p1 -d src/robot_localization < patches/robot_localization-ekf-jetracer-tune.patch

# 3. Build
colcon build --symlink-install
```

Các package của chính dự án là `jetracer_bringup`, `jetracer_description`,
`jetracer_driver`. Một vài package bên thứ ba được vendored có chủ đích (ví dụ
`apriltag` mới hơn `v3.4.5` để ước lượng pose của tag, nhánh ROS 2 của
`rplidar_ros` có hỗ trợ C1). Xem
[`THIRDPARTY_SETUP.md`](https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/THIRDPARTY_SETUP.md)
để biết đầy đủ cách phân chia và lý do các bản vá.

### 3. Sourcing — use `ws_setup.bash`, not `install/setup.bash`

Sau khi build, hãy nạp workspace bằng:

```bash
source ws_setup.bash
```

> **Vì sao:** file `install/setup.bash` hợp nhất trên thiết bị này bị **thiếu sót**.
> Quyền sở hữu file lẫn lộn (một số package thuộc root, một số thuộc user) khiến
> `colcon` bỏ qua những package nó không ghi đè được khi sinh lại `setup.bash`, nên
> `robot_localization`, `nav2_*` và `jetracer_bringup` biến mất khỏi môi trường dù
> chúng vẫn nằm trên đĩa. `ws_setup.bash` né được điều đó bằng cách nạp trực tiếp
> `local_setup.bash` của từng package đã cài. Các script `start_*.sh` đã tự nạp nó giúp
> bạn.
>
> Nếu `ros2` hay một package nào đó (nav2, robot_localization, jetracer_bringup) báo
> "not found", nghĩa là bạn đã nạp setup hợp nhất — hãy dùng `ws_setup.bash`.

### 4. DDS networking — static unicast peers

Robot phải dùng chung đồ thị DDS với workstation / orchestrator. Trên JetRacer,
cấu hình DDS nằm trong **`ros2_docker_v3.sh`** ngay trên thiết bị.

**Vì sao có bước thủ công này:** DDS bình thường tự phát hiện các bên qua UDP
**multicast**, nhưng multicast không hoạt động ổn định trên giao diện mạng của
JetRacer — nên robot không bao giờ thấy workstation và chẳng topic nào xuất hiện.
Cách khắc phục là tắt multicast (`<AllowMulticast>false`) và lùi về
**phát hiện unicast tĩnh**: đưa cho JetRacer một danh sách cố định các **địa chỉ
IP** của các bên để nó liên hệ trực tiếp từng máy. Vì việc phát hiện giờ là thủ
công, cấu hình của **mọi** máy đều phải liệt kê IP của **mọi** máy còn lại — thiếu
một cái là cặp đó không thấy nhau.

1. SSH vào JetRacer:
   ```bash
   ssh jetracer@192.168.20.91     # dùng IP LAN / hostname thật của JetRacer
   ```
2. Mở file khởi chạy và tìm phần cấu hình CycloneDDS của nó:
   ```bash
   grep -n "Discovery" ros2_docker_v3.sh   # tìm khối này, rồi sửa
   nano ros2_docker_v3.sh
   ```
3. Cập nhật khối `<Discovery>` để nó liệt kê JetRacer **cùng mọi bên khác** và nâng
   trần số participant:
   ```xml
   <Discovery>
       <ParticipantIndex>auto</ParticipantIndex>
       <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
       <Peers>
           <Peer Address="192.168.20.XXX"/> <!-- JetRacer (chính thiết bị này)   -->
           <Peer Address="192.168.20.XXX"/>   <!-- workstation                    -->
           <Peer Address="192.168.20.XXX"/>   <!-- ví dụ máy chạy orchestrator    -->
           <Peer Address="192.168.20.XXX"/>   <!-- mỗi máy trên đồ thị một <Peer> -->
       </Peers>
   </Discovery>
   ```
   Hãy thay từng `192.168.20.XXX` bằng IP LAN thật của máy đó (`ip -o -4 addr show`
   trên chính máy ấy). `MaxAutoParticipantIndex` phải đủ lớn (200 là an toàn) để bao
   hết tất cả participant trên mọi máy khi dùng chỉ mục `auto`.
4. Xác nhận **`ROS_DOMAIN_ID`** giống nhau ở mọi nơi (mặc định `42`) và tên
   `<NetworkInterface>` của CycloneDDS khớp với card mạng thật của JetRacer.

> Phía **workstation** của cùng danh sách peer này được sinh tự động từ
> `network.env` bởi `run_workstation.sh` / `run_orchestrator.sh` (`WORKSTATION_IP`,
> `JETRACER_IP`, `DDS_INTERFACE`). Hãy giữ hai bên đồng bộ — mọi IP ở bên này đều
> phải tới được và được liệt kê ở bên kia.

### 5. The layers (`start_*.sh`)

Stack được tách ra để bạn chỉ dựng đúng phần mình cần. Mọi script đều nạp
`ws_setup.bash` trước.

| Script | Khởi động | Publish / làm gì |
|---|---|---|
| `./start_driver.sh [port]` | Chỉ driver chassis (`/cmd_vel` → serial) | `/odom`, `/imu` |
| `./start_lidar.sh` | RPLidar A1 + TF `base_footprint→laser_frame` | `/scan` |
| `./start_hardware.sh` | **driver + lidar + EKF + TF tĩnh + camera/AprilTag** (không Nav2) | `/odom`, `/imu`, `/scan`, `/odometry/filtered` |
| `./start_nav2.sh` | Nav2 (map_server, AMCL, controller, planner, BT nav) | điều khiển `/cmd_vel` |
| `./start_mapping.sh` | Chuyển động Nav2 + khám phá biên `explore_lite` (không map_server) | tự động dựng bản đồ |

`start_hardware.sh` là nền tảng mà mọi quy trình đều cần. Cả `start_nav2.sh` và
`start_mapping.sh` đều dựng các node chuyển động của Nav2 — **đừng chạy cả hai cùng
lúc.**

### 6. Common workflows

#### Navigate on a known map

Cách dễ nhất — tmux dựng phần cứng (khung trái) rồi Nav2 (khung phải, sau khoảng 8
giây để TF và `/scan` sống trước):

```bash
cd amr/jetracer_ws
./start_tmux.sh
# tách ra: Ctrl-b d   gắn lại: tmux attach -t jetracer   tắt: tmux kill-session -t jetracer
```

Hoặc làm thủ công, trong hai terminal:

```bash
./start_hardware.sh
./start_nav2.sh map:=/ros2_ws/maps/test_map_outer_v6.yaml
```

Bản đồ nằm trong `jetracer_ws/maps/` (`test_map_outer_v6.yaml` là mặc định). Sau đó
đặt một **2D Pose Estimate** để khởi tạo cho AMCL và một **Nav2 Goal** từ RViz.

#### Build a new map

1. `./start_hardware.sh`
2. Chạy SLAM (`slam_toolbox`) dựa trên `/scan` + TF của robot.
3. Khám phá không gian — teleop (`teleop_twist_keyboard`), **hoặc**
   `./start_mapping.sh` để tự động khám phá biên.
4. Serialize bản đồ và bỏ file `.yaml` / `.pgm` vào `jetracer_ws/maps/`.

#### Docking (AprilTag)

`start_hardware.sh` cũng dựng luôn camera CSI + detector AprilTag.
`jetracer_bringup/scripts/jetracer_docker.py` chạy máy trạng thái dock/undock, lấy topic
`/docking_state` làm mốc trình tự. Khi toàn bộ stack (phần cứng + Nav2 +
docker) đang chạy, một bản demo khứ hồi:

```bash
./dock_cycle.sh dock1 dock0     # dock A → rời dock → dock B → rời dock
```

Nó publish `/dock_robot` (String) và `/undock_robot` (Bool) rồi chờ
`/docking_state`. Độ chính xác docking phụ thuộc vào hiệu chuẩn camera — xem §8.

### 7. Overriding defaults (serial ports, map)

Các tham số thêm được truyền thẳng xuống launch file:

```bash
./start_hardware.sh base_port:=/dev/ttyACM1 lidar_port:=/dev/ttyACM0
./start_nav2.sh     map:=/ros2_ws/maps/my_map.yaml
```

Mặc định: cổng chassis `/dev/ttyACM0`, cổng lidar `/dev/ttyACM1`.

### 8. Calibration

Một số giá trị được ship sẵn chỉ là **giá trị tạm/ước lượng** và sẽ làm giảm độ
chính xác thấy rõ cho tới khi được đo thật. Hãy làm theo thứ tự ưu tiên sau (xem
[`CALIBRATION.md`](https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/CALIBRATION.md)):

1. **Intrinsics camera** 🔴 — `jetracer_bringup/config/imx219.yaml` ship sẵn một mô
   hình pinhole giả (`fx=fy=320`, độ méo bằng không). Nó chặn *toàn bộ* độ chính
   xác của docking. Hãy thu khung hình bằng `grab_frames.py`, chạy
   `camera_calibration`, rồi dán giá trị thật vào.
2. **Hệ số tỉ lệ odometry bánh xe** 🟡 — `ENCODER_SCALE` của `jetracer_driver`; hãy
   cho xe chạy đúng 1 m đã đo và chỉnh sao cho `/odometry/filtered` đọc ra khoảng
   1,0 m.
3. **Hình học Ackermann** 🟡 — chiều dài cơ sở / góc lái tối đa / bán kính quay tối
   thiểu phải khớp nhau ở cả ba nơi chúng xuất hiện.

### 9. Troubleshooting

| Triệu chứng | Nguyên nhân / cách khắc phục |
|---|---|
| `ros2` / nav2 / robot_localization / jetracer_bringup báo "not found" | Bạn đã nạp `install/setup.bash` hợp nhất. Hãy dùng `source ws_setup.bash` (§3). |
| Odometry trôi ngay từ đầu | Robot đã di chuyển trong lúc hiệu chuẩn con quay ~2 giây. Khởi động lại driver và giữ nó đứng yên. |
| Sai cổng serial / driver không mở được thiết bị | Cổng bị hoán đổi sau khi khởi động lại — hãy ghi đè bằng `base_port:=` / `lidar_port:=` (§7). |
| Nav2 không chạy / không có đường đi | Kiểm tra `/scan` và TF còn sống (`ros2 topic hz /scan`) và AMCL đã định vị đúng bản đồ (hãy đặt một 2D Pose Estimate). |
| Docking nhắm sai chỗ | Intrinsics camera vẫn là giá trị tạm — hãy hiệu chuẩn `imx219.yaml` (§8). |
| Các bên DDS không thấy nhau | `ROS_DOMAIN_ID` không khớp, hoặc các danh sách `<Peers>` unicast không khớp — mọi máy đều phải liệt kê IP của mọi máy còn lại, vì multicast đã tắt (§4). |
| Lỗi `rcl node's rmw handle is invalid` lúc khởi động | CycloneDDS không bind được — tên `<NetworkInterface>` không tồn tại trên máy đó. Hãy kiểm tra `ip -o -4 addr show` và đặt đúng card mạng thật (Linux hiện đại: `enp*` / `eno*` / `wlp*`, không phải `eth0`). |

### 10. Notes for maintainers

- Việc nạp môi trường dùng `ws_setup.bash` là có chủ đích, không phải setup hợp
  nhất — đừng "sửa" các script khởi động để chúng dùng `install/setup.bash` (§3).
- Mã nguồn bên thứ ba được khôi phục từ `thirdparty.repos` + các bản vá cục bộ; hãy
  giữ chúng tái tạo được từng byte (xem `THIRDPARTY_SETUP.md`). `apriltag` và
  `rplidar_ros` được vendored có chủ đích — ghim theo tag sẽ làm robot tệ đi.
- Phát hiện DDS dùng **unicast tĩnh** (multicast tắt): danh sách peer của JetRacer
  nằm trong `ros2_docker_v3.sh` trên thiết bị (§4). Thêm một máy vào đồ thị nghĩa là
  phải thêm IP của nó vào đó (và vào mọi peer khác).
- Odometry: driver publish `/odom` thô, EKF hợp nhất nó thành `/odometry/filtered`
  (Nav2 đọc topic này) và sở hữu TF `odom→base_footprint`.
- Docking là một **topic contract** (`/dock_robot`, `/undock_robot`,
  `/docking_state`) do `jetracer_docker.py` điều khiển, không phải một Nav2 docking
  server.
- Các giá trị hiệu chuẩn trong `jetracer_bringup/config/` được ship dưới dạng giá
  trị tạm; docking và định vị sẽ kém đi cho tới khi chúng được đo thật (§8).
