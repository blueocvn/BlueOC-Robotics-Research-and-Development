# JetRacer Calibration Guide

Những thứ trên robot phải đo hoặc chỉnh tay, không đoán được. Nhiều giá trị ship
sẵn chỉ là **giá trị tạm hoặc ước lượng** — chúng đủ tốt để dựng
stack lên, nhưng độ chính xác docking, đường cong S của Smac, và việc định vị đều
kém đi cho tới khi có số liệu thật.

Chú giải cho **Trạng thái**:

- 🔴 **Giá trị tạm** — số giả, sẽ làm giảm độ chính xác thấy rõ. Làm trước tiên.
- 🟡 **Ước lượng** — phỏng đoán hợp lý, hãy tinh chỉnh trên phần cứng.
- 🟢 **Tự động / Ổn** — tự hiệu chuẩn lúc chạy hoặc đã được đo.

Thứ tự ưu tiên (tác động lớn nhất trước): **1 → 2 → 3 → 4 → 5 → phần còn lại.**

---

## 1. Camera intrinsics 🔴 (do first — gates all docking accuracy)

- **Các file:** [`imx219_measured.yaml`][measured] (kết quả hiệu chuẩn oST thật) và
  [`imx219_inferred.yaml`][inferred] (thô, suy ra từ thông số kỹ thuật).
- **Chọn ở đâu:** dòng `camera_info_url` trong
  [`hardware.launch.py`][hardware-launch].

!!! danger "Đã có một bản hiệu chuẩn tốt nhưng nó không phải bản đang được nạp"
    `imx219_measured.yaml` chứa một kết quả `camera_calibration` **thật**
    (`fx ≈ 521,7`, `fy ≈ 525,5`, độ méo plumb_bob thật). Nhưng
    `hardware.launch.py` hiện đang trỏ `camera_info_url` tới
    **`imx219_inferred.yaml`** — file suy ra từ thông số, mà chính header của nó
    đã cảnh báo:

    > *plumb_bob không mô hình hóa nổi độ méo thùng nặng của ống kính này, nên ảnh
    > đã khử méo vẫn còn méo thấy rõ và độ chính xác pose AprilTag bị giảm, đặc
    > biệt là về phía rìa khung hình. Hãy dùng imx219_measured.yaml để docking
    > chính xác.*

    Nghĩa là docking đang chạy với `fx = fy = 133` và **độ méo bằng không**, trong
    khi một bản hiệu chuẩn đã đo nằm ngay bên cạnh mà không được dùng.
    **Đổi đúng một dòng đó là bản sửa có giá trị cao nhất trên trang này** — hãy
    kiểm chứng trên phần cứng trước khi tin tưởng, vì độ phân giải trong file đã đo
    phải khớp với đầu ra 640×360 thật của pipeline.

- **Vì sao quan trọng:** pose AprilTag được chiếu ngược qua chính các intrinsics này.
  Intrinsics sai → khoảng cách/góc tới dock sai → docking controller nhắm sai chỗ.
- **Nếu bạn cần hiệu chuẩn lại:**
  1. Dựng camera ở đầu ra pipeline **640×360** (phải khớp `image_width/height`).
  2. Thu khung hình bằng [`grab_frames.py`][grab-frames] (`--cols 8 --rows 6`,
     khoảng 40 khung, gần/xa/nghiêng/các góc).
  3. Chạy `camera_calibration` (hoặc `calibrateCamera` của OpenCV) trên các khung
     hình đó.
  4. Dán `camera_matrix`, `distortion_coefficients`, `projection_matrix` thật vào
     file YAML mà bạn nạp.
- **Kiểm tra:** đặt một tag ở khoảng cách đã biết; giá trị khoảng cách trên
  `/detected_dock_pose` phải khớp với thước dây trong khoảng ~1–2 cm.

## 2. Wheel odometry scale 🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — `ENCODER_SCALE = 0.001 * 10`
- **Vì sao quan trọng:** EKF suy vị trí bằng phép dead-reckoning từ vận tốc tiến
  này. Quá nhỏ → odom tưởng xe đi được ít hơn thực tế → các khoảng cách của
  planner/docking bị lệch; AMCL cũng khó hội tụ.
- **Cách làm:** kẻ một vạch 1,0 m. Cho xe chạy thẳng dọc theo nó. Đọc độ dịch
  chuyển x trên `/odometry/filtered` (hoặc `/odom`).
  `tỉ_lệ_mới = tỉ_lệ_cũ × (1.0 / x_đo_được)`. Lặp lại cho tới khi chạy 1 m đọc ra
  khoảng 1,0 m.

## 3. Ackermann geometry: wheelbase, max steer, min turning radius 🟡

Các đại lượng này xuất hiện ở **ba** nơi và phải khớp nhau:

| Đại lượng | Ở đâu | Hiện tại (ước lượng) |
|---|---|---|
| Chiều dài cơ sở `L` | [`ackermann_dock_filter.py`][ackermann], URDF `virtual_steering_joint` x=0.1 ×2 | 0,20 m |
| Góc lái tối đa `δ_max` | [`ackermann_dock_filter.py`][ackermann] | 30° |
| Bán kính quay tối thiểu `R_min` | `minimum_turning_radius` của Smac trong [`jetracer_nav2.yaml`][nav2-yaml] | 0,40 m (0,35 + biên) |

- **Vì sao quan trọng:** `R_min` chi phối đường cong S của Smac Hybrid-A*. Quá nhỏ
  → planner vẽ ra những đường cong mà xe không thể bám nổi về mặt vật lý → xe cắt
  cua / văng rộng. `L` và `δ_max` quy định giới hạn ω của ackermann filter ở pha
  tiếp cận docking cuối cùng.
- **Cách làm (phép đo thật duy nhất):** cho xe chạy ở **góc lái hết cỡ**, ga đều ở
  mức thấp, ít nhất 1 vòng tròn trọn vẹn. Đo **đường kính vòng tròn ÷ 2 = R_min**.
  Làm **cả hai chiều**, giữ giá trị lớn hơn. Rồi suy ra
  `δ_max = atan(L / R_min)`.
- **Sau đó:** đặt `minimum_turning_radius` ≈ R_min × 1,1, cập nhật
  `wheelbase`/`delta_max_deg` trong bộ lọc, và kiểm tra lại URDF cho hợp lý.

## 4. Lidar mounting TF 🔴 (conflicting — fix the duplication)

- **Mâu thuẫn:** có hai TF khác nhau cho cùng một cảm biến:
    - URDF [`jetracer.urdf`][urdf]: node con `laser`, `xyz 0.05 0 0.09`, không yaw.
    - [`start_lidar.sh`][start-lidar]: node con `laser_frame`, `xyz 0 0 0.18`,
      **yaw=π** (gắn lộn ngược).
    - Driver publish các bản quét với `frame_id:=laser_frame`.
- **Vì sao quan trọng:** frame của bản quét là `laser_frame`, nên link `laser`
  trong URDF là thừa/chết, còn phép biến đổi thật lại nằm ở dòng gõ tay trong
  shell script (cả độ cao lẫn phép lật 180° đều là phỏng đoán). Yaw sai sẽ xoay toàn bộ
  bản quét → SLAM/AMCL nhìn thấy tường ở sai vị trí.
- **Cách làm:**
  1. Chọn **một** nơi duy nhất làm chuẩn (khuyến nghị URDF; hãy đổi tên link của nó
     thành `laser_frame` và xóa bộ publish tĩnh trong `start_lidar.sh`).
  2. Đo **độ cao** lidar so với sàn và độ lệch **x/y** so với `base_footprint`.
  3. Xác nhận **yaw**: cho xe chạy về phía một bức tường phẳng, xem `/scan` trong
     RViz; bức tường phải hiện ra phía trước, không phải phía sau. Phép lật π là do
     A1 được gắn lộn ngược trên khung Waveshare — hãy kiểm chứng xem nó có thật sự
     cần thiết không.

## 5. Camera extrinsic (mount pose) 🟡

- **File:** URDF [`jetracer.urdf`][urdf] — `camera_link` ở `xyz 0.12 0 0.07`,
  `rpy 0 0.25 0` (chúc xuống khoảng 14°).
- **Vì sao quan trọng:** AprilTag được nhận dạng trong `camera_optical_frame`;
  phép biến đổi này đặt dock vào `base_footprint`/`odom`. Sai góc chúc/độ lệch →
  xe nhắm hơi cao/thấp hoặc lệch sang bên so với dock thật.
- **Cách làm:** đo x/y/z của camera so với đế, và góc chúc xuống của nó (bằng thước
  đo góc, hoặc nhận dạng một tag ở vị trí đã biết rồi suy ngược ra góc). Cập nhật
  `origin` của khớp.

## 6. AprilTag detection 🟡

- **File:** [`dock_tags_36h11.yaml`][dock-tags]
- **Kích thước tag:** cạnh `0.188 m` — hãy **đo cạnh ô vuông đen thật đã in** và
  chỉnh cho khớp (cả `size` lẫn `sizes` của từng tag). Sai kích thước sẽ làm sai số
  khoảng cách bị nhân theo tỉ lệ tuyến tính.
- **Tinh chỉnh nếu cần:** `decimate: 2.0` (giảm xuống = nhận được tag nhỏ hơn/xa
  hơn, tốn CPU), `refine`, `sharpening`.

## 7. Dock detection offsets + staging 🟡

- **File:** [`jetracer_docking.yaml`][docking-yaml]
- **`external_detection_translation_x: -0.20`**, `rotation_yaw/pitch/roll` —
  chuyển pose quang học của tag sang frame tiếp cận dock. File ghi rõ đây là giá
  trị "tinh chỉnh trên phần cứng".
- **`staging_x_offset: -0.7`** — nơi Smac lái tới trước khi bàn giao cho docking controller. Nó quy định điểm kết thúc của đường cong S.
- **`docking_threshold: 0.15`** — tới khoảng cách này thì xe báo "đã dock".
- **Pose các dock** `dock0/1/2` đều là giá trị tạm `[0,0,0]` — hãy **khảo sát pose
  thật của từng tag trên bản đồ** rồi điền vào.
- **Cách làm:** thực hiện một lần dock thủ công, quan sát xe dừng ở đâu so với tag;
  chỉnh dần `translation_x` cho tới khi nó dừng đúng giữa ở khoảng lùi mong muốn.

## 8. Costmap footprint 🟡

- **File:** [`jetracer_nav2.yaml`][nav2-yaml] —
  `footprint: [[0.17,0.09],[0.17,-0.09],[-0.17,-0.09],[-0.17,0.09]]`
  (~0,34×0,18 m, giả định lấy `base_footprint` làm tâm).
- **Vì sao:** Smac lập kế hoạch tránh va chạm dựa trên footprint này. Nếu
  `base_footprint` nằm ở trục sau (hãy kiểm tra URDF), hãy dịch hình chữ nhật về
  phía trước thay vì giữ nó đối xứng.
- **Cách làm:** đo chiều dài/chiều rộng khung xe và độ lệch gốc tọa độ; cập nhật cả
  hai footprint của costmap.

## 9. Straight-line yaw trim 🟢🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — `YAW_TRIM = 0.0145` (chỉ áp dụng
  khi chạy tiến).
- **Cách làm:** ra lệnh chạy thẳng thuần túy, quan sát độ lệch; chỉnh theo bước
  khoảng 0,005 cho tới khi xe đi thẳng. Đây là vấn đề cơ khí (độ chụm/căn chỉnh) —
  hãy kiểm tra lại nếu cơ cấu lái có thay đổi.

## 10. Gyro bias 🟢 (auto, but respect the procedure)

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — lấy trung bình 100 mẫu (khoảng 2
  giây) lúc khởi động.
- **Việc bắt buộc:** giữ robot **hoàn toàn đứng yên** trong khoảng 2 giây đầu sau
  khi khởi chạy driver, nếu không yaw sẽ trôi suốt cả session làm việc. Hãy theo dõi
  log tìm dòng "Gyro calibrated."

## 11. IMU covariances 🟡

- **File:** [`cmd_vel_to_serial.py`][cmd-vel] — viết cứng `0.01`. Chỉ quan trọng
  nếu EKF tin tưởng con quay quá nhiều hoặc quá ít; chỉ tinh chỉnh khi hướng bị
  nhiễu hoặc phản ứng chậm chạp.

## 12. cmd_vel → speed scaling 🟡 (verify, may be fine)

- **Đường đi:** [`cmd_vel_to_serial.py`][cmd-vel] đóng package `linear.x` thành mm/s
  gửi xuống firmware.
- **Kiểm tra:** ra lệnh một `linear.x` đã biết (ví dụ 0,2 m/s) trong một lần chạy
  có bấm giờ; tốc độ đo được phải khớp. Nếu lệnh và thực tế lệch nhau, các giả định
  về tốc độ của controller (và giới hạn ω của ackermann) đều sai. Odometry
  encoder (mục 2) bù được một phần, nhưng tỉ lệ lệnh open-loop vẫn quan trọng với
  firmware.

## 13. Localization (AMCL) — note, not a calibration 🟡

- **File:** [`jetracer_nav2.yaml`][nav2-yaml] —
  AMCL đang chạy `robot_model_type: DifferentialMotionModel` cho một chiếc xe lái
  kiểu ô tô. Tạm chấp nhận được vì hướng lấy từ EKF đã hợp nhất con quay, nhưng
  đây không phải mô hình chuyển động ackermann đúng nghĩa. Chỉ xem lại nếu AMCL
  hội tụ kém.

---

## Suggested bring-up sequence

1. **Intrinsics camera** (#1) — hãy bắt đầu bằng việc kiểm tra *file YAML nào* đang
   thật sự được nạp; có thể đã có sẵn bản đo rồi.
2. **Tỉ lệ odometry bánh xe** (#2) + **bán kính quay tối thiểu** (#3) — cần cho cả
   EKF lẫn đường cong S của Smac.
3. **TF lidar** (#4) — dẹp phần trùng lặp, kiểm chứng trong RViz với một bức tường.
4. **Extrinsics camera** (#5) + **kích thước tag** (#6) — làm cho
   `/detected_dock_pose` khớp với thước dây.
5. **Độ lệch dock / điểm chờ / pose các dock** (#7) — khép kín vòng docking.
6. Footprint, bù yaw, hiệp phương sai, tỉ lệ cmd_vel — tinh chỉnh dần khi vấn đề
   lộ ra.

[measured]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/imx219_measured.yaml
[inferred]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/imx219_inferred.yaml
[hardware-launch]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/launch/hardware.launch.py
[grab-frames]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/grab_frames.py
[cmd-vel]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_driver/jetracer_driver/cmd_vel_to_serial.py
[ackermann]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/scripts/ackermann_dock_filter.py
[nav2-yaml]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/jetracer_nav2.yaml
[urdf]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_description/urdf/jetracer.urdf
[start-lidar]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/start_lidar.sh
[dock-tags]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/dock_tags_36h11.yaml
[docking-yaml]: https://github.com/blueocvn/robotic-arm/blob/main/amr/jetracer_ws/src/jetracer_bringup/config/jetracer_docking.yaml
