# Trường hợp sử dụng RA — Visual servoing

Bên trong [pipeline gắp và đặt](ra_pick_and_place.md), cánh tay không dựa vào một
pose gắp open-loop duy nhất. Sau bước di chuyển thô, nó chuyển sang **visual servoing
dựa trên ảnh (IBVS)** dùng camera eye-in-hand `arm_cam`, đóng vòng phản hồi lên
chiếc cốc cho tới khi hình học gắp đã đúng.

## Mục tiêu

Khép lại **khoảng cách vài centimét cuối cùng** giữa "cốc đại khái ở đâu" và "gripper phải ở chính xác chỗ nào" — bằng phản hồi camera trực tiếp thay vì tin vào
pose open-loop.

## Vì sao cần servo

Pose của cốc lấy từ camera phía trên chỉ là gần đúng, còn gripper SO-ARM 101 là một
càng một má với khe bắt hẹp. Visual servoing bằng camera trên tay sẽ sửa sai số của
vài centimét cuối để cốc rơi vào khe kẹp một cách tin cậy.

Cụ thể, pose open-loop mang theo ba sai số chồng lên nhau:

| Nguồn sai số | Ảnh hưởng |
|--------------|--------|
| Perception | sai số phương vị/vị trí còn dư trong kết quả nhận dạng từ camera trên |
| Hiệu chuẩn | extrinsics camera-sang-thế-giới không bao giờ hoàn hảo (và trên phần cứng thật thì tệ hơn nhiều) |
| Động học | IK chỉ theo vị trí trên cánh tay 5 bậc tự do để lại sai lệch pose nhỏ |

Từng cái thì nhỏ; cộng lại thì đủ để trượt khỏi một khe kẹp hẹp. Visual servoing loại
bỏ chúng bằng cách **đo kết quả thay vì dự đoán nó**.

## Vì sao không dùng MoveIt Servo

!!! warning "Cánh tay cố ý **không** dùng `moveit_servo`"
    `moveit_servo` hiện thực việc bám **Cartesian 6 bậc tự do**. SO-ARM 101 chỉ có
    **5 bậc tự do**, nên một lệnh Cartesian 6 bậc đầy đủ là bất khả thi — ma trận
    Jacobian suy biến và bộ servo **dừng lại tại điểm kỳ dị** thay vì hội tụ. Một
    nỗ lực áp dụng nó trước đây đã thất bại đúng vì lý do này.

    Thay vào đó, `mtc_node` chạy một **vòng lặp servo tự viết**: nó tính IK qua
    `/compute_ik` rồi publish thẳng một `JointTrajectory` tới
    `/arm_group_controller/joint_trajectory`.

    Trước đây launch file có khởi động một `servo_node` nhưng nó chẳng publish gì
    hữu ích — nay đã bị **gỡ bỏ**. Gói `moveit_servo` không còn là phụ thuộc của dự
    án này.

Đây cũng chính là khuôn mẫu mà [XLeRobot dùng trên SO-101](https://xlerobot.readthedocs.io/en/latest/software/getting_started/SO101.html)
— kết hợp động học dạng đóng với một vòng lặp servo dựa trên ảnh thay vì một bộ
servo Cartesian tổng quát. Đây là bằng chứng độc lập cho thấy vòng lặp tự viết là
lựa chọn đúng trên cánh tay này, chứ không phải một cách chống chế.

## Hoạt động thế nào — hai pha

Vòng lặp được tách ra để **xoay và tịnh tiến không bao giờ cãi nhau**:

```
di chuyển thô → [ pha 0: căn giữa theo điểm ảnh ] → [ pha 1: tiến thẳng vào ] → đóng kẹp
```

| Pha | Điều khiển | Tín hiệu sai số | Vì sao tách riêng |
|-------|----------|--------------|--------------|
| **0 — căn giữa** | phương vị đế (yaw) | sai số điểm ảnh theo phương ngang `dx` trên `arm_cam` | Đưa cốc về **giữa theo trái/phải** trước, trong khi vẫn giữ khoảng lùi. Vừa xoay vừa tiến khiến đường tiếp cận bị cong và cốc trôi khỏi khung hình. |
| **1 — tiếp cận** | vị trí Cartesian | khoảng cách thế giới dọc theo đường tiếp cận | Khi đã căn giữa, hãy **tiến thẳng vào** dọc trục má kẹp ở tốc độ không đổi. Một đường thẳng thì dự đoán được và giữ cốc nằm trong khe. |

Ba chi tiết khiến pha 0 hoạt động đúng mực:

- **Deadband** — bỏ qua sai số điểm ảnh dưới một ngưỡng, để cánh tay đứng yên
  thay vì cứ nhích mãi.
- **Thu nhỏ dần** — bước xoay yaw mỗi nhịp nhỏ dần khi cốc tiến về giữa, nên đế
  *chậm lại* khi vào đúng vị trí thay vì vọt lố.
- **Anti-windup** — bộ tích phân phương vị bị giảm lại khi cánh tay đang trễ so
  với lệnh, để điểm đặt không thể chạy trước cánh tay vật lý và gây vọt lố.

## IBVS so với PBVS — mỗi loại dùng ở đâu

Cả hai đều được dùng, một cách có chủ đích, cho những việc khác nhau.

| | **IBVS** (dựa trên ảnh) | **PBVS** (dựa trên vị trí) |
|---|---|---|
| Sai số nằm ở | **điểm ảnh** | **pose 3D trong thế giới** |
| Dùng cho | **pha tiếp cận cuối tới cốc** | **máy lọc có AprilTag** / căn giữa đế |
| Cần hiệu chuẩn tốt? | **Không — tự sửa sai.** Nó hội tụ về "cốc nằm giữa khung hình" bất kể extrinsics chính xác đến đâu | **Có — rất nhạy.** Bất kỳ sai số extrinsics nào cũng làm dịch mục tiêu, và cánh tay sẽ tự tin đi tới *sai* chỗ |
| Cho ra pose tuyệt đối theo mét? | Không | Có |

!!! tip "Vì sao sự phân chia này quan trọng cho sim-to-real"
    Trong mô phỏng, mọi intrinsics và extrinsics camera đều **chính xác và miễn phí**.
    Trên phần cứng thật, chúng phải được hiệu chuẩn, và sai số còn dư là không
    tránh khỏi.

    **PBVS xuống cấp theo sai số hiệu chuẩn; IBVS thì không.** Vì vậy dùng IBVS cho
    pha gắp cuối vốn đòi hỏi độ chính xác cao chính là lựa chọn chuyển giao tốt
    nhất sang cánh tay thật — PBVS được dành cho các chuyển động thô, neo vào fiducial, nơi pose tuyệt đối thực sự cần thiết.

## Bật nó lên

Visual servoing **được bật mặc định**; tắt nó bằng `skip_servo:=true`:

| Tham số | Mặc định | Mục đích |
|-----|---------|---------|
| `skip_servo` | `false` | `false` = servo camera tay dựa trên ảnh (IBVS); `true` = bỏ qua (gắp thẳng open-loop) |
| `grasp_yaw_bias` | `-0.5` | góc tiếp cận sao cho cốc lọt vào khe của gripper một má |
| `servo_grasp_z` | `0.05986` | độ cao gắp ngang (giữa thân cốc) |

```bash
# IBVS vốn đã bật sẵn — lệnh này chỉ tinh chỉnh độ cao gắp:
ros2 launch mtc_tutorial bringup.launch.py servo_grasp_z:=0.05
# hoặc gắp open-loop (bỏ qua servo):
ros2 launch mtc_tutorial bringup.launch.py skip_servo:=true
```

## Mẹo tinh chỉnh

- **Cốc tuột khỏi má kẹp** → chỉnh `grasp_yaw_bias` để đường tiếp cận đưa miệng
  cốc vào đúng khe của gripper một má.
- **Gắp quá cao / quá thấp trên thân cốc** → chỉnh `servo_grasp_z` về phía giữa
  thân cốc.
- **Servo không bao giờ hội tụ** → xác nhận các topic RGB (và depth) của `arm_cam`
  đang có dữ liệu và được ánh xạ tới `camera_eih_ns` (`arm_cam`); vòng lặp servo
  cần một luồng hình eye-in-hand còn sống.

## Nó nằm ở đâu

```
di chuyển thô → [ BÁM THỊ GIÁC (arm_cam, IBVS) ] → đóng kẹp → chở → hứng → đặt
```

Xem trình tự đầy đủ tại [Gắp và đặt](ra_pick_and_place.md) và hợp đồng topic tại
[Tổng quan](ra_concepts.md#hợp-đồng-ros-isaac-sim).

## Thách thức & giới hạn

??? warning "Các hệ số được tinh chỉnh thủ công, và dấu của chúng phụ thuộc cách gắn camera"
    `servo_img_k_yaw` mã hóa số radian phương vị đế trên mỗi điểm ảnh sai số — và
    **dấu của nó phụ thuộc vào cách camera được gắn**. Đặt sai thì cánh tay sẽ đẩy
    cốc *ra khỏi* khung hình thay vì căn giữa. Các hằng số thu nhỏ dần, deadband và
    anti-windup cũng đều là kinh nghiệm. Gánh nặng tinh chỉnh này là điểm yếu
    lớn nhất của cách tiếp cận.

??? warning "Không có timeout hay đường thoát"
    Nếu IK không với tới được, vòng lặp có thể **quay vô hạn** thay vì thất bại một
    cách gọn gàng. Không có timeout cho servo, không có giới hạn số lần thử lại, và
    không có đường bỏ cuộc. Điều này phải được sửa trước khi chạy trên phần cứng
    thật.

??? warning "Không cảm nhận được tiếp xúc"
    Servo dừng lại khi **đến nơi theo động học**, chứ không phải khi chạm vào cốc.
    Nó không thể phát hiện rằng mình đã va, đã trượt, hay đã làm đổ chiếc cốc.

??? warning "Extrinsics tay-mắt vẫn là giá trị tạm"
    Phép biến đổi gripper → `arm_cam` hiện là giá trị danh nghĩa `eih_z = 0.05`, chứ
    không phải giá trị đã hiệu chuẩn. IBVS chịu được điều này (đó chính là ưu điểm
    của nó), nhưng **khoảng cách tiếp cận theo mét ở pha 1 thì không** — hãy chuẩn
    bị tinh thần là chỗ này cần hiệu chuẩn thật.

??? warning "Được tinh chỉnh dựa trên hình ảnh mô phỏng"
    Bộ nhận dạng cấp dữ liệu cho servo được tinh chỉnh cho mô phỏng. Ánh sáng thật,
    nhòe chuyển động, và một chiếc cốc thật sẽ làm thay đổi tín hiệu điểm ảnh mà
    vòng lặp tiêu thụ.

## Hướng đi tương lai

1. **Rào an toàn trước đã** — một **timeout** cho servo và một lối thoát khi thất
   bại, cộng thêm cơ chế dừng theo tiếp xúc dựa trên lực/dòng. Đây là điều kiện
   tiên quyết để chạm vào phần cứng thật, không phải thứ đánh bóng tùy chọn.
2. **Hiệu chuẩn tay-mắt** — thay extrinsics eye-in-hand tạm thời bằng giá trị đã
   đo; đây chính là thứ mà pha tiếp cận theo mét của pha 1 phụ thuộc vào.
3. **Độ bền vững của depth** — lọc trung vị mảng độ sâu và loại bỏ các giá trị đọc
   không hợp lệ trước khi chúng tới vòng lặp.
4. **Thay các hệ số tinh chỉnh thủ công bằng một chính sách học được.** Giai đoạn
   servo+gắp có chân trời ngắn và giàu tiếp xúc — đúng thứ mà imitation learning làm tốt,
   và đúng chỗ mà gánh nặng tinh chỉnh thủ công đang nằm. Hãy thu thập các demo
   teleop bàn phím trong Isaac Lab (LeIsaac) và huấn luyện một chính sách *chỉ cho
   pha gắp*, giữ nguyên lập kế hoạch dạng script cho phần vận chuyển và cho máy lọc
   neo theo fiducial.

    !!! note "Vì sao lai chứ không phải đầu-cuối"
        Máy lọc được đánh dấu bằng AprilTag và hình học đã biết, nên lập kế hoạch
        cổ điển ở đó **chính xác hơn và dễ gỡ lỗi hơn** một chính sách học được.
        Việc học chỉ đáng giá đúng ở nơi tinh chỉnh thủ công đang chiếm ưu thế —
        pha gắp — và không ở đâu khác.
