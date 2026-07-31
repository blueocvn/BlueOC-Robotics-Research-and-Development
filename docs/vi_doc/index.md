# BlueOC Robot Fulfillment

**Hai robot và một web server cùng giao một cốc nước — trọn vẹn từ đầu đến cuối,
không cần con người can thiệp.**

Khách hàng quét mã QR tại bàn và gọi đồ uống. Một bộ điều phối trung tâm xếp tác
vụ vào hàng đợi và phối hợp hai robot: một **cánh tay robot** hứng đầy cốc rồi đặt
lên một **robot di động**, chiếc robot này băng qua phòng và giao tới bàn. Khi cốc
cạn, đúng vòng lặp đó chạy ngược lại để rót thêm.

`ROS 2` · `Isaac Sim` · `MoveIt 2` · `Nav2` · `FastAPI + HTMX`

---

## Tầm nhìn

Phần lớn các bản demo robot làm tốt *một* việc. Bài toán khó và thú vị nằm ở
**sự phối hợp** — khiến một cánh tay thao tác và một bệ di động cùng hợp tác trong
một nhiệm vụ mà không bên nào tự hoàn thành nổi, được kích hoạt bởi yêu cầu thật
của khách hàng chứ không phải một trigger viết sẵn.

Dự án này chính là hệ thống đó, dựng trọn vẹn:

- **Một bề mặt người dùng thật.** Liên kết sâu từ mã QR và màn hình theo dõi tác
  vụ trực tiếp — đơn hàng đến từ một con người, không phải từ terminal.
- **Một bộ não trung tâm.** Một bộ điều phối duy nhất nắm hàng đợi tác vụ, hoạch
  định từng chặng của mỗi lượt giao, và điều phái cả hai robot qua ROS 2.
- **Hai robot rất khác nhau.** Một cánh tay 5 bậc tự do phải *nhìn*, *gắp* và
  *đặt*; một bệ kiểu ô tô phải *dựng bản đồ*, *định vị* và *điều hướng*. Chúng gặp
  nhau tại trạm để chuyền cốc cho nhau.
- **Ưu tiên mô phỏng.** Mọi thứ chạy trên NVIDIA Isaac Sim, nên toàn bộ pipeline
  được phát triển và kiểm chứng trước khi một động cơ nào quay.

---

## Ba mảnh ghép

<div class="grid cards" markdown>

-   **Cánh tay robot (RA) — SO-ARM 101**

    Cánh tay 5 bậc tự do với gripper một má. Nó nhận dạng cốc bằng camera phía
    trên, visual servoing tới từng chiếc, mang tới máy lọc để hứng nước, rồi đặt lên
    khay.

    [**Tổng quan →**](ra_concepts.md) · [Cài đặt](ra_setup.md) · [Gắp & Đặt](ra_pick_and_place.md)

-   **JetRacer (AMR) — bệ di động**

    Robot kiểu ô tô (Ackermann). Nó dựng bản đồ không gian bằng SLAM, định vị
    bằng Nav2, và đi lại giữa trạm và các bàn, docking chính xác tại từng điểm.

    [**Tổng quan →**](amr_concepts.md) · [Cài đặt](amr_setup.md) · [Điều hướng & Giao hàng](amr_pick_and_place.md)

-   **Bộ điều phối — bộ não**

    Một server FastAPI + HTMX. Nó phục vụ trang QR của khách, nắm hàng đợi đơn
    hàng, và điều phái cả hai robot qua ROS 2 — là thành phần duy nhất hiểu về
    *tác vụ* thay vì về *robot*.

    [**Tổng quan →**](orchestrator.md) · [Giải pháp kết hợp](solution_pick_and_deliver.md)

</div>

---

## Một lượt giao hàng diễn ra thế nào

1. **Đặt đơn** — khách quét QR tại bàn và yêu cầu nước.
2. **Xếp hàng đợi** — bộ điều phối đưa tác vụ vào hàng đợi và giữ chỗ một robot.
3. **Hứng nước** — cánh tay nhận dạng cốc, gắp lên, hứng đầy tại máy lọc.
4. **Chuyển giao** — cánh tay đặt cốc lên khay của AMR tại trạm.
5. **Giao hàng** — AMR di chuyển tới bàn và docking.
6. **Rót thêm** — một cốc rỗng được thu về và vòng lặp chạy lại.

Xem đầy đủ luồng message tại [Gắp và giao](solution_pick_and_deliver.md).

---

## Dự án đang ở đâu

!!! info "Giai đoạn 1 — Bằng chứng khả thi"
    Cả hai robot đều chạy **trong mô phỏng** (Isaac Sim). Cánh tay hoàn thành trọn
    vòng nhận dạng → gắp → hứng → đặt; AMR dựng bản đồ, điều hướng và docking.
    **Chưa có firmware trên thiết bị** — workstation điều khiển bộ mô phỏng, và
    đúng hợp đồng topic ROS 2 đó sau này sẽ điều khiển phần cứng thật.

    Phần tích hợp bộ điều phối ↔ robot là biên giới hiện tại: mối nối docking của
    AMR và các topic tác vụ của cánh tay đã được đặc tả và đấu nối một phần.

Trang của mỗi robot đều nói thật về những gì đã **dựng xong**, những gì **chưa
kiểm chứng**, và những gì **còn nằm trong kế hoạch** — hãy bắt đầu từ các trang
tổng quan phía trên.

---

## Bắt đầu từ đây

| Tôi muốn… | Đi tới |
|---|---|
| Hiểu các khái niệm trước khi động vào mã | [Bắt đầu](GET-STARTED.md) |
| Chạy cánh tay robot | [Hướng dẫn cài đặt RA](ra_setup.md) |
| Chạy JetRacer | [Hướng dẫn cài đặt AMR](amr_setup.md) |
| Xem hai robot + server khớp với nhau ra sao | [Gắp và giao](solution_pick_and_deliver.md) |
| Tra cứu topic, route hoặc tham số | [Sổ tay API](api/index.md) |
