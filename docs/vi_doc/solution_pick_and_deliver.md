# Giải pháp robot — Gắp và giao (RA + AMR)

Vòng lặp hoàn thành đơn hàng đầy đủ kết hợp cả hai robot và một bộ điều phối web:
một người gọi nước từ điện thoại, **cánh tay** hứng đầy cốc và đặt lên khay, còn
**AMR** mang nó từ máy lọc tới chỗ người dùng.

```
   Điện thoại (đơn từ QR)                  RA (SO-ARM 101)
        │                                       │
        ▼                                       ▼
  ┌─────────────┐   /dock_robot        gắp → hứng → đặt lên khay
  │ Bộ điều phối │ ───────────────►                │
  │  (web bridge)│ ◄─── /docking_state             ▼
  └─────────────┘                        AMR (JetRacer)
        ▲                                dock → chở → giao → quay về
        └──────────────  /chassis/odom, /initialpose  ──────────────┘
```

## Ba thành phần

| Thành phần | Vai trò | Tài liệu |
|------|------|------|
| **RA — SO-ARM 101** | Gắp cốc, hứng nước tại máy lọc, đặt lên khay | [Cánh tay robot](ra_concepts.md) |
| **AMR — JetRacer** | Di chuyển từ dock này sang dock khác, chở khay tới người dùng | [JetRacer](amr_concepts.md) |
| **Bộ điều phối** | Giao diện web FastAPI + HTMX kèm bộ điều phái tuần tự hóa AMR | [Bộ điều phối](orchestrator.md) |

Hai workspace chạy trên **các bản phân phối ROS khác nhau** (RA dùng Jazzy native,
AMR dùng Humble trong `Dockerfile.dev`) nhưng vẫn giao tiếp được qua DDS — hãy giữ
**cùng một `ROS_DOMAIN_ID`**.

## Mối nối tại bộ điều phối

Bộ điều phối web (`orchestrator/robot_web_bridge`) sở hữu một bộ điều phái duy
nhất, điều khiển AMR đi từ dock này sang dock khác. Hợp đồng ROS của nó:

- **Publish** `/dock_robot`, `/abort_docking`, `/cmd_vel`, `/initialpose`
- **Subscribe** `/docking_state`, `/chassis/odom`

## Trạng thái tích hợp

Mối nối bộ điều phối ↔ AMR đã đấu xong ở cả hai đầu. Phần việc còn để ngỏ là
**chuyển giao RA (cánh tay) ↔ AMR** — cánh tay vẫn chưa được tích hợp vào bộ điều
phối.

| Hợp đồng | Trạng thái |
|----------|-------|
| Bộ điều phối publish `/dock_robot`, đọc `/docking_state`, `/chassis/odom`, gieo `/initialpose` | ✅ Đã dựng (phía bộ điều phối) |
| Một **bên tiêu thụ** `/dock_robot` trên AMR (dock id → goal Nav2 / hành vi docking) | ✅ Đã hiện thực — `jetracer_bringup/scripts/jetracer_docker.py` (subscribe `/dock_robot`, điều khiển Nav2 + action server `opennav_docking`) |
| Một **bên phát** `/docking_state` thật trên AMR | ✅ Đã hiện thực — `jetracer_docker.py` publish các chuỗi pha thật (gồm cả `relocalize_ok`/`relocalize_failed`) trên `/docking_state` |
| Chuyển giao RA ↔ AMR (khay sẵn sàng → AMR khởi hành) | ❌ Chưa đấu — cánh tay không nằm trong vòng lặp của bộ điều phối |

Docking chạy qua **`opennav_docking`** (một action server docking của Nav2) được
kích hoạt bởi topic `/dock_robot`; `jetracer_docker.py` cũng xử lý `/undock_robot`
và `/abort_docking`. Việc còn để ngỏ là kết nối **cánh tay**: phát tín hiệu "khay
sẵn sàng" để bộ điều phối điều phái AMR sau khi cánh tay xong một cốc.

## Chạy từng phần ở thời điểm hiện tại

Cho tới khi cánh tay được đấu vào vòng lặp, hãy chạy các lớp một cách độc lập:

1. **Rót nước bằng cánh tay** — [RA Gắp và đặt](ra_pick_and_place.md)
2. **Điều hướng AMR** — [AMR Điều hướng & Giao hàng](amr_pick_and_place.md)
3. **Giao diện web bộ điều phối** — xem [Bộ điều phối](orchestrator.md); nó có thể
   chạy ở chế độ `SimBackend` (không cần ROS, mỗi chặng hoàn tất theo bộ đếm thời
   gian) để demo luồng đặt hàng.
