# HTTP API

Dịch vụ `robot_web_bridge` mở JetRacer ra qua HTTP. **Không cần cài ROS trên
máy**, và nó vẫn chạy khi không có robot nào.

Địa chỉ mặc định là `0.0.0.0:8088` — ghi đè bằng `ROBOT_WEB_BRIDGE_HOST` và
`ROBOT_WEB_BRIDGE_PORT`.

```bash
# có ROS, chạy bên trong container
ros2 run robot_web_bridge server

# không có ROS — trang user + backend mô phỏng
pip install fastapi "uvicorn[standard]" jinja2 pyyaml python-multipart
uvicorn robot_web_bridge.app:app --reload --port 8088
```

## Are you talking to a real robot?

`GET /state` cho biết chế độ backend. Hãy kiểm tra trước khi tin vào bất cứ điều gì.

```bash
curl -s localhost:8088/state | jq .mode
# "ros"          -> lệnh đi tới robot thật
# "simulation"   -> order chỉ chạy theo bộ đếm, không có gì chuyển động
```

!!! warning "Chế độ mô phỏng diễn ra âm thầm"

    Bridge **không** báo lỗi khi thiếu `rclpy` — nó chuyển sang backend mô phỏng
    và tiếp tục phục vụ. Một demo "chạy tốt" trên laptop của bạn có thể đang nói
    chuyện với hư không. Hãy luôn kiểm tra `mode` trong các bài test tích hợp.

## Quick start

Đặt một order và theo dõi tiến trình:

```bash
# 1. có những bàn nào?
curl -s localhost:8088/docks | jq

# 2. đặt order cho dock0
curl -s -X POST localhost:8088/orders \
     -d 'dock=dock0' -d 'kind=water'

# 3. theo dõi trạng thái trực tiếp
curl -s localhost:8088/state | jq

# 4. dừng khẩn cấp — hủy order đang chạy và xóa hàng đợi
curl -s -X POST localhost:8088/abort
```

## Admin authentication

Các route vận hành dưới `/v1/admin/` được bảo vệ bằng mã PIN. Đây là những năng
lực thực sự nguy hiểm — teleop trực tiếp, đặt lại pose, docking thủ công — nên
chúng nằm sau một cookie phiên đã được ký.

```bash
# đăng nhập một lần; giữ lại cookie jar
curl -s -c jar.txt -X POST localhost:8088/v1/admin/login -d 'pin=1234'

# sau đó gọi các route điều khiển kèm cookie
curl -s -b jar.txt -X POST localhost:8088/v1/admin/teleop \
     -d 'linear=0.2' -d 'angular=0.0'
```

Mọi route trừ `login`, `logout` và `session` sẽ trả về **401** nếu thiếu cookie hợp lệ.

| Biến môi trường | Mặc định | Ý nghĩa |
|---|---|---|
| `ROBOT_WEB_BRIDGE_ADMIN_PIN` | `1234` | Mã PIN vận hành — **hãy đổi nó** |
| `ROBOT_WEB_BRIDGE_SECRET` | ngẫu nhiên mỗi tiến trình | Khóa ký cookie |
| `ROBOT_WEB_BRIDGE_ADMIN_TTL` | `28800` (8 giờ) | Thời hạn phiên, tính bằng giây |

!!! danger "Hai giá trị mặc định sẽ gây rắc rối tại sự kiện"

    Mã PIN mặc định là `1234`, còn khóa ký mặc định là một **giá trị ngẫu nhiên
    được sinh lại mỗi lần tiến trình khởi động** — nghĩa là mỗi lần restart sẽ âm
    thầm đăng xuất toàn bộ người vận hành. Hãy đặt rõ cả hai
    trước khi sàn hackathon mở cửa.

## Route reference

FastAPI tự sinh sẵn tài liệu tương tác cho chính nó, nên danh sách route luôn
khớp với mã nguồn đang chạy — không cần bảng viết tay nào cả. Khi bridge đang
chạy, hãy mở:

| Địa chỉ | Là gì |
|---|---|
| <http://localhost:8088/docs> | Swagger UI — xem **và gọi thử** từng route ngay trên trình duyệt |
| <http://localhost:8088/redoc> | ReDoc — bản đọc, trình bày gọn hơn |
| <http://localhost:8088/openapi.json> | Đặc tả OpenAPI thô (JSON) |

Lấy nhanh danh sách route từ dòng lệnh:

```bash
curl -s localhost:8088/openapi.json | jq '.paths | keys'
```

!!! tip "Dùng `/docs` để thử nhanh"
    Swagger UI cho phép bấm **Try it out** rồi gửi thẳng request thật tới robot.
    Với các route admin, hãy đăng nhập trước qua `POST /v1/admin/login` để trình
    duyệt giữ cookie phiên.
