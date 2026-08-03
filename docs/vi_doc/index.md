# BlueOC Robot Fulfillment

**Hệ thống robot giao đồ uống tự động: một robot arm rót nước, một xe tự hành
mang tới tận bàn, một web server điều phối cả hai.**

User quét QR tại bàn và gọi đồ uống. Một orchestrator trung tâm xếp việc vào
hàng đợi rồi điều hai robot: **cánh tay robot** hứng đầy cốc và đặt lên **xe tự
hành**, xe chạy qua phòng và giao tới bàn. Cốc cạn thì vòng lặp chạy ngược lại
để rót thêm.

`ROS 2` · `Isaac Sim` · `MoveIt 2` · `Nav2` · `FastAPI + HTMX`

---

## The three pieces

<div class="grid cards" markdown>

-   **Robot Arm (RA) — SO-ARM 101**

    Cánh tay 5 bậc tự do, gripper một má. Nó nhận cốc bằng camera trên cao,
    visual servoing tới từng chiếc, mang tới máy lọc hứng nước rồi đặt lên khay.

    [**Tổng quan →**](ra_concepts.md) · [Cài đặt](ra_setup.md) · [Gắp & Đặt](ra_pick_and_place.md)

-   **JetRacer (AMR) — xe tự hành**

    Xe lái kiểu Ackermann. Nó dựng bản đồ bằng SLAM, định vị bằng Nav2, chạy đi
    chạy lại giữa trạm và các bàn, docking chính xác ở từng điểm.

    [**Tổng quan →**](amr_concepts.md) · [Cài đặt](amr_setup.md) · [Điều hướng & Giao hàng](amr_pick_and_place.md)

-   **Orchestrator — bộ não**

    Server FastAPI + HTMX. Nó chạy trang QR cho user, giữ hàng đợi order, và ra
    lệnh cho cả hai robot qua ROS 2 — thành phần duy nhất nắm được *toàn bộ quy
    trình*, thay vì chỉ điều khiển *một con robot*.

    [**Tổng quan →**](orchestrator.md) · [Giải pháp kết hợp](solution_pick_and_deliver.md)

</div>

---

## How a delivery works

1. **Đặt order** — user quét QR tại bàn và gọi nước.
2. **Vào hàng đợi** — orchestrator nhận việc và giữ chỗ một robot.
3. **Hứng nước** — cánh tay nhận cốc, gắp lên, hứng đầy tại máy lọc.
4. **Chuyền** — cánh tay đặt cốc lên khay của xe tự hành tại trạm.
5. **Giao** — xe chạy tới bàn và docking.
6. **Rót thêm** — thu cốc rỗng về, vòng lặp chạy lại.

Chi tiết luồng message xem ở [Pick and Deliver](solution_pick_and_deliver.md).

---

## Where the project stands

!!! info "Giai đoạn 1 — Proof of Concept"
    Cả hai robot chạy **trong mô phỏng** (Isaac Sim). Cánh tay đi hết vòng nhận
    → gắp → hứng → đặt; xe tự hành dựng bản đồ, điều hướng và docking. **Chưa có
    firmware chạy trên thiết bị** — workstation điều khiển bộ mô phỏng, và đúng
    bộ topic ROS 2 đó sau này sẽ điều khiển phần cứng thật.

    Phần đang làm dở là tích hợp orchestrator ↔ robot: điểm nối docking của xe tự
    hành và các topic công việc của cánh tay đã đặc tả xong, mới đấu được một
    phần.

---

## Start here

| Bạn muốn… | Đọc |
|---|---|
| Hiểu khái niệm trước khi động vào code | [Get Started](GET-STARTED.md) |
| Chạy cánh tay robot | [RA Setup Guide](ra_setup.md) |
| Chạy xe tự hành | [AMR Setup Guide](amr_setup.md) |
| Xem hai robot và server ghép với nhau ra sao | [Pick and Deliver](solution_pick_and_deliver.md) |
| Tra topic, route hay tham số | [API Book](api/index.md) |
