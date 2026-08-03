# JetRacer — Getting Started

Stack chạy trên robot vật lý. Chạy **trên chính chiếc JetRacer** (Waveshare
JetRacer, ROS 2 Humble): driver bệ xe, RPLidar, odometry EKF, điều hướng Nav2, và
docking bằng AprilTag.

---

## 1. Build

Các script `start_*.sh` nạp **`ws_setup.bash`**, file này nạp ROS cùng từng package đã
cài riêng từng cái. Đây là cách đi vòng có chủ đích cho một
`install/setup.bash` hợp nhất bị thiếu sót trên thiết bị này — hãy dùng nó thay
cho `source install/setup.bash`.

```bash
cd jetracer_ws
colcon build --symlink-install
source ws_setup.bash
```

> ⚠️ Giữ robot **đứng yên khoảng 2 giây** lúc driver khởi động, trong khi con quay
> hồi chuyển tự hiệu chuẩn. Di chuyển trong lúc hiệu chuẩn sẽ làm hỏng odometry.

---

## 2. The layers

Stack được tách ra để bạn chỉ dựng đúng phần mình cần:

| Script                | Khởi động cái gì                                                   | Publish                                        |
| --------------------- | ----------------------------------------------------------------- | ---------------------------------------------- |
| `./start_driver.sh`   | Chỉ driver bệ xe (`/cmd_vel` → serial)                            | `/odom`, `/imu`                                |
| `./start_lidar.sh`    | RPLidar A1 + TF `base_footprint→laser_frame`                      | `/scan`                                        |
| `./start_hardware.sh` | **driver + lidar + EKF + TF tĩnh + camera/AprilTag** (không Nav2)  | `/odom`, `/imu`, `/scan`, `/odometry/filtered` |
| `./start_nav2.sh`     | Nav2 (map_server, AMCL, controller, planner, BT nav)              | điều khiển `/cmd_vel`                          |
| `./start_mapping.sh`  | Chuyển động Nav2 + khám phá biên `explore_lite` (không map_server) | tự động dựng bản đồ                            |

`start_hardware.sh` là nền tảng mà mọi quy trình đều cần. Cả `start_nav2.sh` và
`start_mapping.sh` đều dựng các node chuyển động của Nav2 — **đừng chạy cả hai.**

---

## 3. Common workflows

### Navigate on a known map

Cách dễ nhất — tmux dựng phần cứng (khung trái) rồi Nav2 (khung phải, sau 8 giây):

```bash
cd jetracer_ws
./start_tmux.sh
# tách ra: Ctrl-b d   gắn lại: tmux attach -t jetracer   tắt: tmux kill-session -t jetracer
```

Hoặc làm thủ công, trong hai terminal:

```bash
./start_hardware.sh
./start_nav2.sh map:=/ros2_ws/maps/test_map_outer_v6.yaml
```

Bản đồ nằm trong `jetracer_ws/maps/` (`test_map_outer_v6.yaml` là mặc định). Sau
đó đặt một goal Nav2 từ RViz.

### Build a new map

1. `./start_hardware.sh`
2. Chạy SLAM (`slam_toolbox`) dựa trên `/scan` + TF của robot.
3. Chạy vòng quanh — teleop, **hoặc** `./start_mapping.sh` để tự động khám phá
   biên.
4. Lưu bản đồ và bỏ file `.yaml`/`.pgm` vào `jetracer_ws/maps/`.

### Docking (AprilTag)

`start_hardware.sh` cũng dựng luôn camera CSI + bộ nhận dạng AprilTag.
`jetracer_bringup/scripts/jetracer_docker.py` chạy máy trạng thái dock/undock
(trình tự do `/docking_state` dẫn dắt). Demo một vòng khứ hồi, khi toàn bộ stack
đã chạy sẵn:

```bash
./dock_cycle.sh dock1 dock0     # dock A → rời dock → dock B → rời dock
```

Intrinsics camera và bố trí tag của dock nằm trong `jetracer_bringup/config/`.
Xem `CALIBRATION.md` — độ chính xác docking phụ thuộc vào TF camera + hiệu chuẩn.

---

## 4. Robot Web Bridge (ordering app)

Lớp ứng dụng — một giao diện web di động FastAPI + HTMX kèm API HTTP để ra lệnh
cho robot (luồng đặt hàng "Get Water" / "Refill" qua mã QR). Nó nằm ở một
workspace riêng, **`orchestrator/`**, và nói chuyện với stack này qua đồ thị ROS:
nó publish `/dock_robot`, `/abort_docking`, `/cmd_vel`, `/initialpose` và
subscribe `/docking_state` + `/chassis/odom`. Vì vậy stack robot ở §3 (phần cứng +
Nav2 + docking) phải đang chạy sẵn, trên **cùng một `ROS_DOMAIN_ID`**.

### Start it

```bash
cd ../orchestrator
./run_web_bridge.sh          # phục vụ tại http://<host>:8088
```

Script này nạp `network.env` + ROS, rồi chạy `ros2 run robot_web_bridge server`.
Nó cần được chạy bên trong container Humble (hoặc một shell đã nạp ROS). Ghi đè
cổng bằng `ROBOT_WEB_BRIDGE_PORT=9000`, và đặt mã PIN vận hành qua
`ROBOT_WEB_BRIDGE_ADMIN_PIN` cho các route admin.

### Expose it for phones (QR codes)

Ở một shell khác, tạo tunnel cổng này ra một URL công khai để điện thoại quét và
đặt hàng:

```bash
./run_tunnel.sh              # in ra một URL https://<...>.trycloudflare.com
```

### Without the robot (dev / demo)

```bash
./run_web_bridge_sim.sh      # SimBackend: không cần ROS, mỗi chặng xong theo bộ đếm
```

Kiểm tra chế độ lúc chạy: `GET /state` → `{"mode": "ros" | "simulation"}`.
Kiểm tra sức khỏe: `GET /healthz`. Danh sách route đầy đủ ở
[Sổ tay API](api/http.md).

---

## 5. Overriding defaults

Các tham số thêm được truyền thẳng xuống launch file:

```bash
./start_hardware.sh base_port:=/dev/ttyACM1 lidar_port:=/dev/ttyACM0
./start_nav2.sh     map:=/ros2_ws/maps/my_map.yaml
```

Mặc định: cổng bệ xe `/dev/ttyACM0`, cổng lidar `/dev/ttyACM1`.

---

## Troubleshooting

- **Thiếu `ros2` hoặc các package (nav2, robot_localization, jetracer_bringup)** →
  bạn đã nạp setup hợp nhất. Hãy dùng `source ws_setup.bash`.
- **Odometry trôi ngay từ đầu** → robot đã di chuyển trong lúc hiệu chuẩn con quay
  ~2 giây. Khởi động lại driver và giữ nó đứng yên.
- **Sai cổng serial** → ghi đè bằng `base_port:=` / `lidar_port:=`.
- **Nav2 không chạy / không có đường đi** → kiểm tra `/scan` và TF còn sống
  (`ros2 topic hz /scan`), và AMCL đã định vị đúng bản đồ.
