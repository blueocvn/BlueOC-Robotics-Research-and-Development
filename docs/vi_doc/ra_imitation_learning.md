# Trường hợp sử dụng RA — Imitation learning (LeRobot)

Một lựa chọn thay thế cho pipeline gắp-và-đặt viết sẵn bằng MTC: dạy SO-ARM 101 kỹ
năng **gắp → giữ → đặt** bằng *biểu diễn mẫu*. Một người điều khiển cánh tay
**leader**, cánh tay **follower** bắt chước theo, và từng khung hình (trạng thái
khớp + ảnh camera) được ghi vào một tập dữ liệu
[LeRobot](https://github.com/huggingface/lerobot). Sau đó một chính sách (ACT /
diffusion / v.v.) được huấn luyện bằng behavior cloning rồi phát lại trên
follower.

Phần này **tách biệt** với stack ROS 2 / MoveIt trong `ra_ws` — nó chạy hoàn toàn
qua LeRobot trên cặp SO-101 leader+follower vật lý. Không có gì ở đây phụ thuộc
vào Isaac Sim hay `move_group`.

!!! note "Trạng thái bàn giao"
    Một **chính sách ACT đã huấn luyện và đã hội tụ** — xem
    [Mô hình đã huấn luyện](#mô-hình-đã-huấn-luyện) bên dưới. **Tập dữ liệu đã ghi**
    mới là tài sản tái sử dụng chính (bạn có thể huấn luyện lại bất kỳ chính sách
    nào từ nó). Mọi thứ cần để ghi lại, huấn luyện lại, triển khai hay phát lại đều
    nằm trên trang này. Bản thân thư viện LeRobot là một bản cài từ thượng nguồn
    (`~/lerobot`, chế độ editable) và **không** được vendored vào repo này.

## Môi trường

| Hạng mục | Giá trị |
|------|-------|
| Phiên bản LeRobot | **0.3.4** |
| Môi trường conda | `lerobot` (`conda activate lerobot`) |
| Python | qua `~/miniconda3/envs/lerobot` |
| Bổ sung | `ffmpeg` (mã hóa/giải mã video), PyTorch khớp CUDA để huấn luyện |

!!! warning "GPU / PyTorch để huấn luyện"
    Huấn luyện cần một bản PyTorch khớp với GPU. Mô hình ở đây được huấn luyện trên
    **RTX 4060 Laptop (8 GB)** với **torch 2.7.1 + CUDA 12.6 (cu126)** — hãy xác
    nhận `torch.cuda.is_available()` trả về `True` trước một lần chạy dài, nếu
    không nó sẽ âm thầm lùi về CPU. Trên card mới hơn (ví dụ dòng RTX 50) hãy dùng
    wheel cu128 tương ứng. `ffmpeg` phải nằm trong `PATH` để giải mã video.

## Phần cứng

Hai cánh tay SO-101 nối qua USB serial:

| Vai trò | Thiết bị | `id` trong LeRobot | Loại |
|------|--------|--------------|------|
| Follower (robot thực hiện hành động) | `/dev/ttyACM0` | `my_follower` | `so101_follower` |
| Leader (cái bạn cầm tay điều khiển) | `/dev/ttyACM1` | `my_leader` | `so101_leader` |

Cổng không được đảm bảo ổn định qua các lần khởi động lại — hãy xác nhận bằng:

```bash
conda activate lerobot
lerobot-find-port          # rút/cắm lại để nhận diện từng cánh tay
```

## Hiệu chuẩn

Hiệu chuẩn động cơ cho từng cánh tay nằm trong bộ nhớ đệm của LeRobot (**không**
nằm trong repo này):

```
~/.cache/huggingface/lerobot/calibration/
├── robots/so101_follower/my_follower.json
└── teleoperators/so101_leader/my_leader.json
```

Mỗi file JSON giữ độ lệch home + dải hoạt động cho sáu khớp
(`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`).
Tạo lại nếu bị mất:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower
lerobot-calibrate --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

## Tập dữ liệu

**`weeho/so101_pick_hold_place`** — các bản biểu diễn gắp/giữ/đặt.

| Thuộc tính | Giá trị |
|----------|-------|
| Vị trí | `~/.cache/huggingface/lerobot/weeho/so101_pick_hold_place` |
| Số episode | 48 |
| Số khung hình | 28.817 |
| FPS | 30 |
| Loại robot | `so101_follower` |
| Action / state | 6 bậc tự do: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` (`.pos`) |
| Camera | 2 (96 video = 48 × 2) |
| Số tác vụ | 1 |
| Định dạng tập dữ liệu | LeRobot codebase **v2.1** |

Bố cục: `meta/` (info/episodes/tasks jsonl) · `data/chunk-000/episode_*.parquet`
· `videos/chunk-000/<cam>/episode_*.mp4`. Có một bản sao anh em `.bak`.

!!! danger "Tập dữ liệu KHÔNG nằm trong git"
    Nó nặng 1,6 GB gồm parquet + mp4 và chỉ tồn tại trong bộ nhớ đệm HF nói trên.
    Để bàn giao, hãy làm **một** trong các cách sau:

    - **Đẩy lên Hub:** `huggingface-cli login` rồi
      `lerobot-record … --dataset.push_to_hub=true` (hoặc đẩy lên sau) — người nhận
      kéo về bằng `--dataset.repo_id=weeho/so101_pick_hold_place`.
    - **Sao chép thư mục cache** `~/.cache/huggingface/lerobot/weeho/so101_pick_hold_place`
      (và cả thư mục `calibration/`) sang máy mới.
    - **Đóng gói tarball:** `tar czf so101_pick_hold_place.tar.gz -C ~/.cache/huggingface/lerobot weeho/so101_pick_hold_place`.

    Trên máy mới, cần có tập dữ liệu **cộng thêm** một bản cài LeRobot, PyTorch khớp
    GPU, và `ffmpeg` thì mới huấn luyện được.

## Quy trình

Mọi lệnh đều chạy trong `conda activate lerobot`. Tên cờ chính xác thay đổi giữa
các bản LeRobot — hãy kiểm tra `--help` trên bản 0.3.4 đã ghim nếu một cờ nào đó
bị từ chối.

### 1. Điều khiển từ xa (kiểm tra nhanh)

Follower phải bắt chước leader mà không ghi lại gì:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

### 2. Ghi các bản biểu diễn

Thêm camera + nơi lưu tập dữ liệu vào chính vòng lặp teleop đó:

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader \
  --dataset.repo_id=weeho/so101_pick_hold_place \
  --dataset.single_task="Pick the cup, hold it, then place it" \
  --dataset.num_episodes=48 --dataset.fps=30
```

(Khóa camera được khai báo qua `--robot.cameras=…`; dùng `lerobot-find-cameras`
để liệt kê các thiết bị đang kết nối.)

### 3. Huấn luyện một chính sách

Lệnh đã dùng cho mô hình được bàn giao (ACT, khoảng 0,35 giây/bước trên RTX 4060):

```bash
lerobot-train \
  --dataset.repo_id=weeho/so101_pick_hold_place \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/so101_pick_hold_place_act_100k \
  --steps=100000 --batch_size=8 --save_freq=50000
```

- `--policy.push_to_hub=false` là **bắt buộc** trừ khi bạn cũng truyền
  `--policy.repo_id=…` — nếu không, việc huấn luyện sẽ hủy ngay ở bước kiểm tra
  cấu hình.
- ACT ở đây **hội tụ sớm** (hàm mất mát L1 phẳng quanh ~0,05 từ khoảng bước 50k trở
  đi); checkpoint tại 50k tốt ngang những checkpoint sau đó.

!!! danger "Deadlock ở dataloader khi ổ đĩa gần đầy"
    Một lần chạy đã từng **treo** (tiến trình còn sống, GPU ở 0%, không tiến thêm
    bước nào) khi ổ đĩa gần đầy — một worker của dataloader đang giải mã video bị
    kẹt và vòng lặp chính chờ nó mãi mãi, và nó **không** tự phục hồi. Cách phòng
    tránh:

    - Giữ vài GB dung lượng trống dự phòng (mỗi checkpoint nặng khoảng 0,6 GB kèm
      trạng thái optimizer).
    - Việc nạp dữ liệu không phải nút thắt ở đây (`data_s ≈ 0`), nên
      **`--num_workers=0`** là cách an toàn để loại bỏ deadlock đa tiến trình mà
      gần như không mất tốc độ.

## Mô hình đã huấn luyện

Một chính sách **ACT** đã hội tụ được lưu tại:

```
outputs/train/so101_pick_hold_place_act_100k/checkpoints/last/pretrained_model/
├── model.safetensors     # ~198 MB — trọng số của chính sách
├── config.json           # đặc tả chính sách + đặc trưng
└── train_config.json
```

| Thuộc tính | Giá trị |
|----------|-------|
| Chính sách | ACT |
| Số bước huấn luyện | 50.000 (mất mát L1 cuối ~0,05) |
| **Đầu vào** | `observation.state` (6 khớp) + **2 camera**: `observation.images.wrist`, `observation.images.top` (mỗi cái 640×480 @ 30 fps) |
| **Đầu ra** | `action` — 6 giá trị vị trí khớp mục tiêu |

> Trạng thái optimizer/tiếp tục (`training_state/`) đã bị xóa để tiết kiệm ổ đĩa —
> mô hình vẫn chạy suy luận được mà không cần nó, nhưng bạn **không thể tiếp tục
> huấn luyện** từ checkpoint này. Hãy huấn luyện lại từ tập dữ liệu nếu cần đi
> tiếp.

### 4. Phát lại một episode đã ghi

Phát lại open-loop một bản demo trên follower (kiểm tra nhanh, không dùng chính
sách):

```bash
lerobot-replay --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --dataset.repo_id=weeho/so101_pick_hold_place --dataset.episode=0
```

### 5. Triển khai chính sách đã huấn luyện lên cánh tay thật

Việc chạy một chính sách trên phần cứng thật được thực hiện bằng
**`lerobot-record --policy.path=…`** (chính sách thay thế cho leader/teleop và điều
khiển follower; `lerobot-eval` dành cho môi trường gym *mô phỏng*, không phải cánh
tay vật lý). **Không cần leader.**

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --robot.cameras='{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
                     top:   {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }' \
  --policy.path=outputs/train/so101_pick_hold_place_act_100k/checkpoints/last/pretrained_model \
  --dataset.repo_id=weeho/eval_so101_pick_hold_place \
  --dataset.single_task="Pick the cup, hold it, then place it" \
  --dataset.num_episodes=3 --dataset.episode_time_s=30 --dataset.push_to_hub=false
```

Hãy thay các giá trị `index_or_path` của camera bằng giá trị thật lấy từ
`lerobot-find-cameras`.

!!! danger "Điều gì quyết định thành bại khi triển khai"
    - **Tên camera phải đúng là `wrist` và `top`** — đó là cách chính sách ánh xạ
      hai đầu vào ảnh của nó (`observation.images.wrist` / `.top`). Sai tên là
      không chạy được.
    - **Góc nhìn phải khớp với lúc ghi** — camera cổ tay trên gripper, camera top ở
      phía trên. Chính sách đã học đúng những góc nhìn đó; xê dịch một camera là
      hỏng.
    - **Cùng bản hiệu chuẩn** (`my_follower.json`) và **30 fps** như khi huấn luyện.
    - Cánh tay chuyển động **tự động** ngay khi khởi chạy — hãy để tay sẵn trên nút
      dừng khẩn cấp, dọn quang khu vực làm việc, và đặt cốc gần vị trí như trong
      các bản demo.
    - **Khả năng tổng quát hóa rất hẹp** — 48 bản demo cho một chiếc cốc/một tác vụ;
      chỉ nên kỳ vọng thành công quanh đúng điều kiện đã được biểu diễn.

## Quan hệ với stack ROS 2

| | Viết sẵn (MTC) | Imitation learning (LeRobot) |
|---|---|---|
| Ở đâu | `ra_ws` (ROS 2 Jazzy, MoveIt) | LeRobot, không có ROS |
| Perception | YOLO + tia–mặt phẳng, AprilTag | Khung hình camera thô đưa thẳng vào chính sách |
| Điều khiển | IK + quỹ đạo đã lập kế hoạch | Hành động học được = vị trí khớp mục tiêu |
| Điểm mạnh | Tất định, xem xét được | Chịu được biến thiên mà không cần tinh chỉnh thủ công |

Hai cách này là hai bản hiện thực kỹ năng độc lập cho cùng một tác vụ
**gắp-và-đặt** và không dùng chung môi trường chạy. Xem trang
[Gắp và đặt](ra_pick_and_place.md) để biết pipeline viết sẵn.
