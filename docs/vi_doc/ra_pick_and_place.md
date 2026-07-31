# RA Use Case — Pick and Place (Refill)

Bản demo trọn vẹn: cánh tay nhận ra một chiếc cốc rỗng, gắp nó lên, mang tới máy
lọc để "hứng nước", rồi đặt vào khay. Trang này đi qua kịch bản đó; về cài
đặt/build, xem [Hướng dẫn cài đặt](ra_setup.md).

## Objective

Tự động hoàn thành một đơn đồ uống từ đầu tới cuối: **nhận dạng → gắp → hứng →
đặt**, lặp lại cho mọi chiếc cốc mà camera phía trên nhìn thấy, không cần con
người can thiệp.

Cánh tay là **nửa RA** của [giải pháp Gắp và giao](solution_pick_and_deliver.md)
rộng hơn — nó chuẩn bị những cốc đã đầy lên khay, rồi AMR (JetRacer) mang tới bàn.

Ràng buộc có chủ đích: làm được điều này trên **phần cứng 5 bậc tự do giá rẻ**
(SO-ARM 101, gripper một má) thay vì một cánh tay công nghiệp 6/7 bậc tự do. Gần
như mọi quyết định thiết kế dưới đây đều bắt nguồn từ ràng buộc ấy.

**Tiêu chí thành công**

| Tiêu chí | Mục tiêu |
|-----------|--------|
| Gắp | Mọi cốc được nhận dạng đều được gắp lên mà không làm đổ |
| Hứng | Cốc tới được mái chèo của máy lọc ở đúng độ sâu |
| Đặt | Cốc rơi vào đúng ô khay được phân, không va vào các cốc đã đặt |
| Vòng lặp | Lặp gọn gàng qua N cốc mà không gắp lại chiếc đã đặt xuống |

## Prerequisites

- Workspace đã build và Isaac Sim đang chạy với ROS contract đã kiểm chứng — xem
  [Hướng dẫn cài đặt §1–5](ra_setup.md).
- Scene cánh tay đã mở trong Isaac Sim và đã bấm **Play**.

## Run it

Một lệnh duy nhất dựng MoveIt (`move_group` + bộ điều khiển + RViz), perception và
`mtc_node`, xếp lệch nhau để phụ thuộc của mỗi lớp lên trước:

```bash
source install/setup.bash
ros2 launch mtc_tutorial bringup.launch.py
```

## What happens

1. `move_group`, `ros2_control`, và các bộ điều khiển `arm_group` / `hand_group`
   khởi động.
2. Perception khởi động — bộ nhận dạng cốc YOLO (`yolo11n`), bộ nhận dạng khay
   hồng, bộ nhận dạng AprilTag của máy lọc.
3. `mtc_node` chờ `/detected_object/position`, rồi **với mỗi cốc**:

    | Bước | Hành động |
    |------|--------|
    | Di chuyển thô | Tiến về phía chiếc cốc đã nhận dạng |
    | Visual servoing | Căn tinh vào cốc bằng camera trên tay — xem [Visual servoing](ra_visual_servoing.md) |
    | Gắp | Đóng gripper |
    | Chở | Di chuyển tới máy lọc có AprilTag |
    | Hứng | Nghiêng / ấn để "hứng nước" |
    | Đặt | Hạ cốc xuống khay |

!!! info "Bao nhiêu cốc?"
    Không được cấu hình trực tiếp — bằng đúng số cốc mà **camera trên** nhận ra lúc
    bắt đầu (mặc định là một nếu không có cốc nào), rải đều trên khay. Một chiếc
    cốc duy nhất sẽ được đặt ở giữa.

## Proposed solution — and why

Mỗi giai đoạn được chọn để làm việc *quanh* giới hạn của phần cứng 5 bậc tự do,
một má kẹp, thay vì chống lại nó.

| Quyết định | Lý do |
|----------|-----|
| **Gắp ngang, không gắp từ trên xuống** | Má kẹp dài khoảng 0,17 m. Chĩa thẳng xuống một chiếc cốc thấp sẽ đâm má kẹp vào **mặt bàn** trước khi bắt được cốc. Gắp từ trên xuống không phải bài toán tinh chỉnh — nó **bất khả thi về mặt hình học** trên cánh tay này. Gắp ngang bằng mới là phương án với tới được. |
| **IK chỉ theo vị trí** | Với 5 bậc tự do, bạn không thể ra lệnh đồng thời cả vị trí *và* hướng bất kỳ. Chỉ giải theo vị trí, còn cổ tay cố định theo cấu tạo (`Wrist_Roll = −90°`, `Wrist_Pitch = −(Pitch + Elbow)`), giúp gripper luôn nằm ngang và bài toán IK giải được. |
| **Tiếp cận chéo góc (`grasp_yaw_bias`)** | SO-ARM 101 ép cốc vào một má **cố định**. Tiếp cận thẳng chính giữa sẽ khiến má đó gạt văng chiếc cốc. Quay lệch trục khoảng 29° (`-0.5 rad`) đưa cốc **vào đúng khe** giữa hai má. |
| **Chiếu ngược tia–mặt phẳng từ camera trên** | Chiếu ngược bộ đệm độ sâu trên một bề mặt xiên mang theo sai lệch hệ thống. Thay vào đó, giao tia đi qua điểm ảnh nhận dạng với mặt phẳng chiều cao cốc đã biết đã giảm sai số camera trên từ **≈31 mm → ≈3 mm**. |
| **AprilTag trên máy lọc** | Đích hứng nước phải thật chính xác. Một fiducial cho ra pose chính xác dưới milimét từ camera phía trên — đáng tin cậy hơn nhiều so với nhận dạng máy lọc bằng thị giác. |
| **MoveIt Task Constructor (MTC)** | Tác vụ này vốn dĩ chia thành giai đoạn (tiếp cận → gắp → nâng → chở → đặt). MTC diễn đạt điều đó thành các giai đoạn ghép được, có lập kế hoạch nhận biết va chạm (OMPL / RRTConnect), thay vì một script khối duy nhất. |
| **Visual servoing trước khi gắp** | Lập kế hoạch đưa gripper tới *gần*, nhưng pose open-loop mang theo sai số của perception + hiệu chuẩn. Đóng vòng phản hồi bằng camera trên tay sẽ sửa nốt vài centimét cuối — xem [Visual servoing](ra_visual_servoing.md). |
| **Phân ô khay + loại trừ** | Các cốc được rải đều trên khay, và những cốc đã đặt bị loại khỏi việc nhận dạng để cánh tay không gắp lại chiếc mình vừa đặt xuống. |
| **Ưu tiên mô phỏng (Isaac + `topic_based_ros2_control`)** | Đúng topic contract ROS mà cánh tay thật sẽ dùng, với rủi ro phần cứng bằng không và một scene tái lập được. |

!!! tip "Mạch xuyên suốt"
    Perception cho ra pose thế giới gần đúng → lập kế hoạch đưa tới gần →
    **closed-loop vision sửa bước cuối** → hình học gripper (góc + độ cao) lo phần
    còn lại. Mỗi lớp đều gỡ lỗi được độc lập.

## Tuning

Hình học gắp/hứng được điều khiển bằng các tham số launch (giá trị mặc định hiển
thị bên dưới) — bảng đầy đủ ở [Hướng dẫn cài đặt §7](ra_setup.md):

```bash
ros2 launch mtc_tutorial bringup.launch.py \
    grasp_yaw_bias:=-0.5 \
    dispenser_standoff:=0.10 \
    dispenser_fill_depth:=-0.08
```

## If it stalls

- **`mtc_node` chờ mãi `/detected_object/position`** → perception không publish;
  hãy kiểm tra các topic camera và tham số `camera_*_ns`.
- **Cánh tay không hề nhúc nhích** → Isaac chưa ở chế độ Play, hoặc chưa subscribe
  `/isaac_joint_commands`.

Thêm nữa ở [Hướng dẫn cài đặt §8](ra_setup.md).

## Challenges & limitations

Các ràng buộc đã biết, xếp theo thứ tự mà một lập trình viên mới có khả năng vấp
phải.

??? warning "Phần cứng — thiếu truyền động ở 5 bậc tự do"
    Chỉ ra lệnh được vị trí, không phải hướng đầy đủ. Các pose gắp bị giới hạn ở
    kiểu gắp ngang bằng, và **gắp từ trên xuống là bất khả thi** (chiều dài má kẹp
    so với chiều cao cốc). Mọi ý tưởng thao tác mới đều phải được kiểm tra khả năng
    với tới trước, chứ không chỉ lập kế hoạch.

??? warning "Gripper — một má cố định"
    Cốc bị ép vào một má cố định thay vì bị bóp bởi hai má cùng chuyển động. Điều
    này khiến việc bắt cốc rất nhạy với góc tiếp cận (`grasp_yaw_bias`) và là phần
    khó tinh chỉnh nhất trong toàn bộ pipeline.

??? warning "Độ tin cậy khi gắp — lỗi không liên tục"
    Pipeline **chạy trọn vẹn trong mô phỏng**, nhưng **cú gắp không thành công mọi
    lần** — thỉnh thoảng gripper không bắt được cốc. Nguyên nhân gốc vẫn chưa được
    khoanh vùng. Các nghi phạm khả dĩ, theo thứ tự:

    - **Góc tiếp cận** — `grasp_yaw_bias` nhắm cốc vào khe kẹp. Nếu cốc lệch phương
      vị một chút, má **cố định** sẽ gạt nó thay vì bắt được.
    - **Độ cao gắp** — `servo_grasp_z` nằm ở giữa thân cốc; sai lệch ở đây làm thay
      đổi vị trí má kẹp chạm vào thành cốc.
    - **Bàn giao giữa các pha servo** — nếu pha 0 bàn giao trước khi căn giữa đủ
      tốt, pha 1 sẽ lao thẳng vào theo hướng hơi lệch trục.
    - **Không có phản hồi tiếp xúc** — gripper đóng chỉ dựa vào động học, nên một cú
      gắp hụt không được phát hiện cũng không được thử lại.

    Cho tới khi điều này được mô tả rõ, hãy xem thành công khi gắp là
    **có xác suất, không được đảm bảo**. Xem [Visual servoing](ra_visual_servoing.md)
    để biết các núm tinh chỉnh.

??? warning "Calibration — an unexplained bias"
    Cốc luôn rơi lệch về một phía ở **cả** x và y. Hai giá trị bù đã đo được dùng
    để triệt tiêu:

    ```cpp
    place_x += paramd("place_x_offset", 0.044);
    place_y += paramd("place_y_offset", 0.0386);
    ```

    Đây là **ROS parameter có giá trị mặc định**, nên bạn override được mà không
    cần build lại. Độ lớn đã đo (~4 cm), nhưng **nguyên nhân gốc thì chưa rõ** —
    coi đây là dấu hiệu đáng ngờ, không phải vấn đề đã giải quyết.

??? warning "Perception — được tinh chỉnh cho mô phỏng"
    Các ngưỡng HSV và trọng số YOLO tiền huấn luyện trên COCO đều được **tinh chỉnh
    cho mô phỏng**. Ánh sáng thật và cốc thật sẽ làm lệch cả hai. Điểm neo khay cũng
    trôi khi cánh tay che khuất camera phía trên giữa chừng.

??? warning "Không có phản hồi lực hay tiếp xúc"
    Cú gắp đóng lại **chỉ dựa vào động học** — không có cảm biến dòng/lực để phát
    hiện tiếp xúc hay một cú gắp hụt. Trên phần cứng thật, đây vừa là lỗ hổng an
    toàn vừa là lỗ hổng độ tin cậy.

??? warning "\"Hứng nước\" là một cử chỉ, không phải rót thật"
    Cánh tay nghiêng/ấn vào mái chèo có gắn tag. Không có mô phỏng chất lỏng, cảm
    biến dòng chảy, hay xử lý tràn đổ.

??? warning "Chưa tích hợp vào hệ thống lớn hơn"
    Cánh tay chạy độc lập. Các topic tác vụ giữa orchestrator ↔ RA và việc chuyển
    cốc AMR ↔ RA đã **được thiết kế nhưng chưa đấu nối**.

## Future direction

Xếp theo thứ tự việc nào tháo gỡ được nhiều nút thắt nhất.

1. **Chẩn đoán lỗi gắp không liên tục** — pipeline vốn đã chạy trọn vẹn, nên
   *độ tin cậy* chính là khoảng cách giữa nó và một bản demo đáng tin. Hãy đo đạc
   khoảnh khắc bàn giao servo (ghi lại sai số điểm ảnh `dx` tại thời điểm chuyển từ
   pha 0 → 1) và ghi nhận kết quả từng cú gắp, để chế độ hỏng được **mô tả rõ thay
   vì phỏng đoán**. Một cơ chế dừng theo tiếp xúc/lực (mục 2) cũng sẽ cho phép phát
   hiện và thử lại một cú gắp hụt thay vì âm thầm đi tiếp.
2. **Đưa lên phần cứng thật (sim-to-real)** — driver SO-ARM 101 thật,
   **hiệu chuẩn tay-mắt** (extrinsics eye-in-hand hiện vẫn là giá trị tạm), dừng
   gắp dựa trên lực/dòng, và một timeout cho servo để vòng lặp thoát ra thay vì
   quay mãi trên một mục tiêu IK không với tới được.
3. **Tích hợp hệ thống** — phơi bày các topic tác vụ của RA và hiện thực việc
   chuyển giao AMR ↔ RA để cánh tay trở thành một node trong
   [giải pháp Gắp và giao](solution_pick_and_deliver.md).
4. **Xoay lại quai cốc** — bộ nhận dạng quai đã tồn tại nhưng chưa được đấu vào.
   Vì cánh tay không thể gắp từ trên xuống, việc xoay lại chiếc cốc theo quai phải
   là thao tác **đẩy-để-xoay** chứ không phải gắp lại.
5. **Gắp bằng học máy (lai)** — các giai đoạn vận chuyển và máy lọc đều có hình học
   chính xác dựa trên fiducial, nên tốt nhất cứ để viết script. **Cú gắp** mới là
   giai đoạn tinh chỉnh thủ công, giàu tiếp xúc, và là ứng viên tự nhiên để thay
   bằng một chính sách imitation learning (demo teleop bàn phím trong Isaac Lab /
   LeIsaac).
6. **Độ bền vững** — domain randomization cho perception; tổng quát hóa vượt ra
   ngoài một màu và một kích cỡ cốc duy nhất; đưa các asset Isaac từ xa về máy để
   scene tải được khi ngoại tuyến.
