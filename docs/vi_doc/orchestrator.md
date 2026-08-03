# Orchestrator — `robot_web_bridge`

API HTTP + giao diện web di động để ra lệnh cho JetRacer. Xây bằng **FastAPI +
HTMX + Tailwind** (cả hai đều nạp từ CDN nên không có bước build front-end — rất
hợp để chạy sau tunnel rồi quét QR tại từng bàn).

Package này hiện thực **các trang user công khai**, **bridge ROS + dispatcher**,
và **API admin** (các endpoint vận hành có chắn mã PIN). Phần *giao diện*
admin và nhật ký order bằng SQLite thì vẫn còn ở phía trước.

!!! tip "Phần tham khảo route nằm trong Sổ tay API"
    Mọi route, tham số và phản hồi của nó đều được **sinh tự động từ định nghĩa
    FastAPI của chính ứng dụng này** — xem **[API HTTP](api/http.md)**. Phần tham
    khảo đó không thể lỗi thời; một bảng bảo trì thủ công đặt ở đây thì có (và
    thực tế đã từng bị).

    Trang này nói về **kiến trúc** — cách bridge, dispatcher và các backend
    khớp với nhau.

## Architecture

Ba mảnh, mỗi mảnh một việc:

| Module | Vai trò |
|---|---|
| `app.py` | Các route FastAPI — trang user, API JSON, endpoint admin |
| `ros_node.py` | Một node rclpy duy nhất chạy trên luồng daemon — mối nối với ROS |
| `dispatcher.py` | **Vòng lặp bất đồng bộ duy nhất sở hữu robot** — mỗi lúc một order |
| `store.py` | Sổ order trong bộ nhớ (tạm thay cho nhật ký SQLite) |
| `auth.py` | Chắn mã PIN vận hành — cookie phiên đã ký, có hạn |

### The ROS seam

`ros_node.py`:

- **publish** `/dock_robot` (String), `/abort_docking` (Bool), `/cmd_vel`
  (Twist), `/initialpose` (PoseWithCovarianceStamped), cùng với các topic có
  latch `/virtual_obstacles` và `/dock_registry` (String, JSON) và
  `/relocalize_at_dock` (String);
- **subscribe** `/docking_state` (String), `/odometry/filtered` và
  `/chassis/odom` (Odometry), cache giá trị mới nhất, an toàn giữa các
  luồng.

topic contract đầy đủ nằm ở [Sổ tay API](api/ros-jetracer.md).

### The dispatcher

`dispatcher.py` đưa order cũ nhất trong hàng đợi lên, publish `/dock_robot`,
rồi ánh xạ `/docking_state` trực tiếp sang trạng thái hiển thị cho user
(chuẩn bị → đang trên đường → đã giao / thất bại). `store.py` là nguồn của các
thông tin "robot đang bận / còn N order phía trước / ~thời gian dự kiến".

## No ROS? It still runs

Nếu không import được `rclpy` (một venv thuần để phát triển cục bộ, hoặc bản demo
qua tunnel), ứng dụng sẽ tự động lùi về một **backend mô phỏng**, đẩy tiến trình
order theo bộ đếm thời gian.

```bash
curl -s localhost:8088/state | jq .mode   # "ros" | "simulation"
```

!!! warning "Cơ chế lùi này diễn ra âm thầm"
    Thiếu `rclpy` **không** gây lỗi — ứng dụng vẫn tiếp tục phục vụ dù chẳng nói
    chuyện với ai. Hãy kiểm tra `mode` trong mọi bài test tích hợp, nếu không một
    lần chạy test xanh mướt có thể chẳng chứng minh được điều gì về robot.

### Matching the real robot's `/docking_state`

Các chuỗi trạng thái chính xác do robot định nghĩa. Hãy xác nhận và điều chỉnh nếu
cần:

```bash
ros2 topic info /docking_state -v     # xác nhận kiểu message
ros2 topic echo /docking_state        # bắt lấy các chuỗi thật
```

Sau đó ghi đè các ánh xạ qua biến môi trường (ngăn cách bằng dấu phẩy, không phân
biệt hoa thường):

```bash
export ROBOT_WEB_BRIDGE_INPROGRESS_STATES="docking,navigating"
export ROBOT_WEB_BRIDGE_SUCCESS_STATES="docked,arrived"
export ROBOT_WEB_BRIDGE_ERROR_STATES="failed,aborted"
export ROBOT_WEB_BRIDGE_ORDER_TIMEOUT=180   # đánh dấu order bị kẹt là thất bại sau N giây
```

## Run it

Bên trong container (sau khi `colcon build --packages-select robot_web_bridge`):

```bash
ros2 run robot_web_bridge server          # http://localhost:8088
# hoặc: orchestrator/run_web_bridge.sh
```

Python thuần (các trang user không cần ROS):

```bash
pip install fastapi "uvicorn[standard]" jinja2 pyyaml python-multipart
cd orchestrator/src/robot_web_bridge
uvicorn robot_web_bridge.app:app --reload --port 8088
```

Rồi mở <http://localhost:8088/?dock=dock0>.

## Config

| Thiết lập | Mặc định | Mục đích |
|---|---|---|
| `config/docks.yaml` | — | Danh bạ dock (`dock_id → nhãn, pose`) |
| `ROBOT_WEB_BRIDGE_CONFIG` | thư mục `config/` của package | Ghi đè thư mục cấu hình |
| `ROBOT_WEB_BRIDGE_HOST` / `_PORT` | `0.0.0.0:8088` | Địa chỉ lắng nghe |
| `ROBOT_WEB_BRIDGE_ADMIN_PIN` | `1234` | Mã PIN vận hành — **hãy đổi nó** |
| `ROBOT_WEB_BRIDGE_SECRET` | ngẫu nhiên mỗi tiến trình | Khóa ký cookie |
| `ROBOT_WEB_BRIDGE_ADMIN_TTL` | `28800` | Thời hạn phiên, tính bằng giây |

!!! danger "Hai giá trị mặc định cần đổi trước một sự kiện thật"
    Mã PIN mặc định là `1234`, và khóa ký được **sinh lại mỗi lần tiến trình khởi
    động** — nên mỗi lần restart sẽ âm thầm đăng xuất toàn bộ người vận hành. Hãy
    đặt rõ cả hai.

## See also

- [API HTTP](api/http.md) — phần tham khảo route được sinh tự động
- [Giao diện ROS của JetRacer](api/ros-jetracer.md) — topic contract
- [Gắp và giao](solution_pick_and_deliver.md) — vị trí của nó trong hệ thống
