# RA Use Case — Visual Servoing

Trong [pipeline gắp và đặt](ra_pick_and_place.md), cánh tay không tin vào một
pose gắp open-loop duy nhất. Sau bước di chuyển thô, nó chuyển sang **visual
servoing dựa trên ảnh (IBVS)** với camera eye-in-hand `arm_cam`: vừa nhìn cốc vừa
chỉnh dần, tới khi gripper vào đúng thế gắp mới dừng.

## Objective

Xoá nốt **vài centimét cuối** giữa chỗ perception đoán cốc đang nằm và chỗ
gripper thật sự phải tới — bằng ảnh camera thời gian thực, không phải bằng niềm
tin vào pose open-loop.

## Why servo

Camera trên cao chỉ cho pose gần đúng, mà gripper SO-ARM 101 lại chỉ có một má
động, khe kẹp thì hẹp. Visual servoing bằng camera trên tay sửa nốt sai số vài
centimét cuối để cốc lọt đúng khe.

Cụ thể, pose open-loop mang theo ba sai số chồng lên nhau:

| Nguồn sai số | Ảnh hưởng |
|--------------|--------|
| Perception | sai số phương vị/vị trí còn dư trong kết quả nhận dạng từ camera trên |
| Hiệu chuẩn | extrinsics camera-sang-thế-giới không bao giờ hoàn hảo (và trên phần cứng thật thì tệ hơn nhiều) |
| Động học | IK chỉ theo vị trí trên cánh tay 5 bậc tự do để lại sai lệch pose nhỏ |

Từng cái nhỏ, cộng lại đủ để trượt khỏi khe kẹp. Visual servoing xoá chúng bằng
cách **đo kết quả thật thay vì dự đoán trước**.

## Why not MoveIt Servo

!!! warning "Cánh tay cố ý **không** dùng `moveit_servo`"
    `moveit_servo` làm servo **Cartesian 6 bậc tự do**. SO-ARM 101 chỉ có
    **5 bậc tự do**, nên một lệnh Cartesian 6 bậc đầy đủ là bất khả thi — ma trận
    Jacobian suy biến và bộ servo **dừng lại tại điểm kỳ dị** thay vì hội tụ. Một
    nỗ lực áp dụng nó trước đây đã thất bại đúng vì lý do này.

    Thay vào đó, `mtc_node` chạy một **vòng lặp servo tự viết**: nó tính IK qua
    `/compute_ik` rồi publish thẳng một `JointTrajectory` tới
    `/arm_group_controller/joint_trajectory`.

    Trước đây launch file có khởi động một `servo_node` nhưng nó chẳng publish gì
    hữu ích — nay đã **gỡ**. Package `moveit_servo` không còn là dependency của
    dự án.

[XLeRobot trên SO-101](https://xlerobot.readthedocs.io/en/latest/software/getting_started/SO101.html)
cũng làm hệt như vậy — closed-form kinematics ghép với một servo loop dựa trên
ảnh, thay vì một bộ servo Cartesian tổng quát. Tức là vòng lặp tự viết ở đây là
lựa chọn đúng cho cánh tay 5 bậc, không phải cách chống chế.

## How it works — two phases

Vòng lặp tách đôi để **xoay và tiến không giành nhau**:

```
di chuyển thô → [ pha 0: căn giữa theo điểm ảnh ] → [ pha 1: tiến thẳng vào ] → đóng kẹp
```

| Pha | Điều khiển | Tín hiệu sai số | Vì sao tách riêng |
|-------|----------|--------------|--------------|
| **0 — căn giữa** | phương vị đế (yaw) | sai số điểm ảnh theo phương ngang `dx` trên `arm_cam` | Đưa cốc về **giữa theo trái/phải** trước, trong khi vẫn giữ khoảng lùi. Vừa xoay vừa tiến khiến đường tiếp cận bị cong và cốc trôi khỏi khung hình. |
| **1 — tiếp cận** | vị trí Cartesian | khoảng cách thế giới dọc theo đường tiếp cận | Khi đã căn giữa, hãy **tiến thẳng vào** dọc trục má kẹp ở tốc độ không đổi. Một đường thẳng thì dự đoán được và giữ cốc nằm trong khe. |

Ba chi tiết giữ cho pha 0 chạy ổn:

- **Deadband** — bỏ qua sai số điểm ảnh dưới một ngưỡng, để cánh tay đứng yên
  thay vì cứ nhích mãi.
- **Taper** — bước xoay yaw nhỏ dần khi cốc về gần giữa, nên đế *giảm tốc* khi
  vào vị trí chứ không vọt qua.
- **Anti-windup** — khi cánh tay chạy chậm hơn lệnh, bộ tích phân bị ghìm lại,
  để setpoint không chạy trước cánh tay thật rồi gây vọt lố.

## IBVS vs PBVS — where each is used

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
    pha gắp cuối là lựa chọn sống sót tốt nhất khi chuyển sang cánh tay thật.
    PBVS chỉ dùng cho các chuyển động thô neo vào fiducial, nơi thật sự cần pose
    tuyệt đối.

## Enabling it

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

## Tuning tips

- **Cốc tuột khỏi má kẹp** → chỉnh `grasp_yaw_bias` để đường tiếp cận đưa miệng
  cốc vào đúng khe của gripper một má.
- **Gắp quá cao / quá thấp trên thân cốc** → chỉnh `servo_grasp_z` về phía giữa
  thân cốc.
- **Servo không bao giờ hội tụ** → xác nhận các topic RGB (và depth) của `arm_cam`
  đang có dữ liệu và được ánh xạ tới `camera_eih_ns` (`arm_cam`); vòng lặp servo
  cần một luồng hình eye-in-hand còn sống.

## Where it fits

```
di chuyển thô → [ VISUAL SERVOING (arm_cam, IBVS) ] → đóng kẹp → chở → hứng → đặt
```

Xem trình tự đầy đủ tại [Gắp và đặt](ra_pick_and_place.md) và topic contract tại
[Tổng quan](ra_concepts.md#the-ros-contract-isaac-sim).

## Challenges & limitations

??? warning "Các hệ số được tinh chỉnh thủ công, và dấu của chúng phụ thuộc cách gắn camera"
    `servo_img_k_yaw` mã hóa số radian phương vị đế trên mỗi điểm ảnh sai số — và
    **dấu của nó phụ thuộc vào cách camera được gắn**. Đặt sai thì cánh tay sẽ đẩy
    cốc *ra khỏi* khung hình thay vì căn giữa. Các hằng số taper, deadband và
    anti-windup cũng đều rút từ kinh nghiệm. Gánh nặng tinh chỉnh này là điểm yếu
    lớn nhất của cách làm này.

??? warning "Không có timeout hay đường thoát"
    Nếu IK không với tới được, vòng lặp có thể **quay vô hạn** thay vì thất bại một
    cách gọn gàng. Không có timeout cho servo, không có giới hạn số lần thử lại, và
    không có đường bỏ cuộc. Điều này phải được sửa trước khi chạy trên phần cứng
    thật.

??? warning "Không cảm nhận được tiếp xúc"
    Servo dừng lại khi **đến nơi theo động học**, chứ không phải khi chạm vào cốc.
    Nó không thể phát hiện rằng mình đã va, đã trượt, hay đã làm đổ chiếc cốc.

??? warning "Hand-eye extrinsics vẫn là giá trị tạm"
    Phép biến đổi gripper → `arm_cam` hiện là giá trị danh nghĩa `eih_z = 0.05`, chứ
    không phải giá trị đã hiệu chuẩn. IBVS chịu được điều này (đó chính là ưu điểm
    của nó), nhưng **khoảng cách tiếp cận theo mét ở pha 1 thì không** — hãy chuẩn
    bị tinh thần là chỗ này cần hiệu chuẩn thật.

??? warning "Được tinh chỉnh dựa trên hình ảnh mô phỏng"
    Bộ nhận dạng cấp dữ liệu cho servo được tinh chỉnh cho mô phỏng. Ánh sáng thật,
    nhòe chuyển động, và một chiếc cốc thật sẽ làm thay đổi tín hiệu điểm ảnh mà
    vòng lặp đọc.

## Future direction

1. **An toàn trước đã** — thêm **timeout** cho servo và một lối thoát khi thất
   bại, cộng cơ chế dừng theo lực/dòng khi chạm. Đây là điều kiện bắt buộc trước
   khi động vào phần cứng thật, không phải thứ làm cho đẹp.
2. **Hand-eye calibration** — thay extrinsics eye-in-hand tạm thời bằng giá trị
   đo thật; pha 1 tính khoảng cách theo mét nên phụ thuộc trực tiếp vào nó.
3. **Depth bền hơn** — lọc trung vị vùng depth và bỏ các giá trị không hợp lệ
   trước khi đưa vào vòng lặp.
4. **Thay hệ số chỉnh tay bằng policy học được.** Giai đoạn servo + gắp diễn ra
   trong vài giây và nhiều tiếp xúc — đúng thứ imitation learning làm tốt, và cũng
   đúng chỗ đang tốn công chỉnh tay nhất. Thu demo teleop bàn phím trong Isaac Lab
   (LeIsaac) rồi huấn luyện policy *chỉ cho pha gắp*, còn phần vận chuyển và máy
   lọc neo fiducial thì giữ nguyên script.

    !!! note "Vì sao hybrid chứ không phải end-to-end"
        Máy lọc có AprilTag và hình học đã biết, nên planning cổ điển ở đó
        **chính xác hơn và dễ debug hơn** một policy học được. Học chỉ đáng ở đúng
        chỗ đang phải chỉnh tay nhiều nhất — pha gắp — chứ không phải mọi nơi.
