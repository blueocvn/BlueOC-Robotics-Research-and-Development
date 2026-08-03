# API Book

Toàn bộ những gì bạn có thể gọi, publish hoặc subscribe trong hackathon — cho cả
**JetRacer** (AMR) và **SO-ARM 101** (cánh tay robot).

API chia làm ba lớp. Phần lớn các đội chỉ cần lớp đầu tiên.

<div class="grid cards" markdown>

-   :material-api:{ .lg .middle } **API HTTP**

    ---

    Điều khiển JetRacer qua HTTP thuần — đặt order, docking, teleop, dừng khẩn cấp.
    Không cần cài ROS trên máy của bạn.

    [:octicons-arrow-right-24: Tham khảo HTTP](http.md)

-   :material-robot-outline:{ .lg .middle } **ROS 2 — JetRacer**

    ---

    Các topic mà AMR lắng nghe và publish: `/cmd_vel`, `/dock_robot`,
    `/docking_state`, odometry.

    [:octicons-arrow-right-24: Giao diện JetRacer](ros-jetracer.md)

-   :material-robot-industrial:{ .lg .middle } **ROS 2 — Cánh tay robot**

    ---

    Kết quả nhận dạng từ perception, pipeline gắp–đặt MoveIt/MTC, điều khiển quỹ
    đạo khớp, pose AprilTag.

    [:octicons-arrow-right-24: Giao diện cánh tay](ros-arm.md)

-   :material-rocket-launch-outline:{ .lg .middle } **Điểm khởi chạy**

    ---

    Cách khởi động từng stack, và chọn launch file nào cho mô phỏng so với phần
    cứng thật.

    [:octicons-arrow-right-24: Tham khảo launch](launch.md)

</div>

## Which layer should I use?

| Nếu bạn muốn… | Dùng | Cần ROS trên máy? |
|---|---|---|
| Điều robot đến một bàn từ web app hoặc script | [API HTTP](http.md) | Không |
| Phản ứng theo trạng thái robot ở tốc độ đầy đủ | [Topic ROS 2](ros-jetracer.md) | Có |
| Ra lệnh cho cánh tay gắp vật | [Giao diện cánh tay](ros-arm.md) | Có |
| Khởi động một stack từ đầu | [Tham khảo launch](launch.md) | Có |

!!! tip "API HTTP không cần ROS"

    Nếu không import được `rclpy`, bridge tự chuyển sang **backend mô phỏng**,
    đẩy tiến trình order theo bộ đếm thời gian. `GET /state` cho biết bạn đang ở
    chế độ nào.

## Conventions used in this book

- **Tên topic** là tuyệt đối (`/cmd_vel`) trừ khi được ghi là *tương đối*, khi đó
  namespace của node sẽ được áp dụng.
- **Kiểu message** được viết dạng `package/msg/Type`.
- Các route có dấu :material-lock: yêu cầu xác thực người vận hành — xem
  [Xác thực admin](http.md#admin-authentication).

## Reporting drift

Phần tham khảo HTTP được sinh tự động từ ứng dụng FastAPI đang chạy, nên nó không
thể lỗi thời. Các trang ROS được bảo trì thủ công dựa trên mã nguồn. Nếu bạn thấy
một topic không khớp thực tế, đó là một lỗi đáng báo cáo — hãy mở issue thay vì
âm thầm tìm cách đi vòng.
