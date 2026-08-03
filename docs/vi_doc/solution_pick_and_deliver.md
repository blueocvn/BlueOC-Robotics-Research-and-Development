# Robotic Solution — Pick and Deliver (RA + AMR)

Vòng lặp hoàn thành đơn hàng đầy đủ kết hợp cả hai robot và một orchestrator web:
user gọi nước từ điện thoại, **cánh tay** hứng đầy cốc và đặt lên khay, còn
**AMR** mang nó từ máy lọc tới chỗ user.

```
   Điện thoại (đơn từ QR)                  RA (SO-ARM 101)
        │                                       │
        ▼                                       ▼
  ┌─────────────┐   /dock_robot        gắp → hứng → đặt lên khay
  │ Orchestrator │ ───────────────►                │
  │  (web bridge)│ ◄─── /docking_state             ▼
  └─────────────┘                        AMR (JetRacer)
        ▲                                dock → chở → giao → quay về
        └──────────────  /chassis/odom, /initialpose  ──────────────┘
```

## The three parts

| Thành phần | Vai trò | Tài liệu |
|------|------|------|
| **RA — SO-ARM 101** | Gắp cốc, hứng nước tại máy lọc, đặt lên khay | [Cánh tay robot](ra_concepts.md) |
| **AMR — JetRacer** | Di chuyển từ dock này sang dock khác, chở khay tới chỗ user | [JetRacer](amr_concepts.md) |
| **Orchestrator** | Giao diện web FastAPI + HTMX kèm dispatcher serialize AMR | [Orchestrator](orchestrator.md) |

Hai workspace chạy trên **các bản phân phối ROS khác nhau** (RA dùng Jazzy native,
AMR dùng Humble trong `Dockerfile.dev`) nhưng vẫn giao tiếp được qua DDS — hãy giữ
**cùng một `ROS_DOMAIN_ID`**.

## The orchestrator seam

Orchestrator web (`orchestrator/robot_web_bridge`) sở hữu một dispatcher duy
nhất, điều khiển AMR đi từ dock này sang dock khác. ROS contract của nó:

- **Publish** `/dock_robot`, `/abort_docking`, `/cmd_vel`, `/initialpose`
- **Subscribe** `/docking_state`, `/chassis/odom`

## Integration status

Mối nối orchestrator ↔ AMR đã đấu xong ở cả hai đầu. Phần việc còn để ngỏ là
**chuyển giao RA (cánh tay) ↔ AMR** — cánh tay vẫn chưa được tích hợp vào orchestrator.

| Hợp đồng | Trạng thái |
|----------|-------|
| Orchestrator publish `/dock_robot`, đọc `/docking_state`, `/chassis/odom`, publish `/initialpose` | ✅ Đã dựng (phía orchestrator) |
| Một **bên đọc** `/dock_robot` trên AMR (dock id → goal Nav2 / hành vi docking) | ✅ Đã hiện thực — `jetracer_bringup/scripts/jetracer_docker.py` (subscribe `/dock_robot`, điều khiển Nav2 + action server `opennav_docking`) |
| Một **bên phát** `/docking_state` thật trên AMR | ✅ Đã hiện thực — `jetracer_docker.py` publish các chuỗi pha thật (gồm cả `relocalize_ok`/`relocalize_failed`) trên `/docking_state` |
| Chuyển giao RA ↔ AMR (khay sẵn sàng → AMR khởi hành) | ❌ Chưa đấu — cánh tay không nằm trong vòng lặp của orchestrator |

Docking chạy qua **`opennav_docking`** (một action server docking của Nav2) được
kích hoạt bởi topic `/dock_robot`; `jetracer_docker.py` cũng xử lý `/undock_robot`
và `/abort_docking`. Việc còn để ngỏ là kết nối **cánh tay**: phát tín hiệu "khay
sẵn sàng" để orchestrator điều AMR đi sau khi cánh tay xong một cốc.

## Running the pieces today

Cho tới khi cánh tay được đấu vào vòng lặp, hãy chạy từng lớp riêng:

1. **Rót nước bằng cánh tay** — [RA Gắp và đặt](ra_pick_and_place.md)
2. **Điều hướng AMR** — [AMR Điều hướng & Giao hàng](amr_pick_and_place.md)
3. **Giao diện web orchestrator** — xem [Orchestrator](orchestrator.md); nó có thể
   chạy ở chế độ `SimBackend` (không cần ROS, mỗi chặng hoàn tất theo bộ đếm thời
   gian) để demo luồng đặt hàng.
