# Tài liệu tiếng Việt

Bản dịch tiếng Việt của trang tài liệu BlueOC Robotics.

## Đây không phải là một phần của trang web được build

Thư mục này nằm **cùng cấp** với `docs/docs/` (tức `docs_dir`), chứ không nằm
bên trong nó. MkDocs vì vậy **không** build các trang ở đây — trang web xuất bản
lên GitHub Pages chỉ có tiếng Anh.

Hãy đọc trực tiếp các file `.md` ở đây trên GitHub, hoặc bằng bất kỳ trình xem
Markdown nào.

## Cấu trúc

Phản chiếu đúng `docs/docs/`, nên các liên kết tương đối giữa các trang vẫn hoạt
động:

```
docs/vi_doc/
├── index.md                    # trang chủ
├── GET-STARTED.md              # bắt đầu
├── ra_*.md                     # cánh tay robot (RA)
├── amr_*.md                    # JetRacer (AMR)
├── solution_pick_and_deliver.md
├── orchestrator.md
├── CALIBRATION.md              # hiệu chuẩn JetRacer
├── THIRDPARTY_SETUP.md
├── mkdocs.md
└── api/                        # Sổ tay API
    ├── index.md
    ├── http.md
    ├── ros-jetracer.md
    ├── ros-arm.md
    └── launch.md
```

## Quy ước dịch

- **Giữ nguyên tiếng Anh** các thuật ngữ không có nghĩa tiếng Việt tương đương:
  Ackermann, visual servoing, eye-to-hand / eye-in-hand, intrinsics / extrinsics,
  open-loop / closed-loop, fiducial, deadband, anti-windup, footprint,
  imitation learning, behavior cloning, domain randomization, gripper.
- **Giữ nguyên** mọi tên topic, kiểu message, tên tham số, tên package và lệnh —
  đó là những gì bạn thực sự gõ vào terminal.
- **Không chú thích trong ngoặc**: chỉ dùng thuật ngữ tiếng Anh, không kèm bản
  dịch tiếng Việt phía sau.

## Đồng bộ với bản tiếng Anh

Không có cơ chế tự động nào phát hiện sai lệch giữa hai bản. Khi sửa một trang
trong `docs/docs/`, hãy cập nhật trang tương ứng ở đây.

!!! note "Phần tham khảo HTTP API"
    Trang `api/http.md` bản tiếng Anh dựng bảng route từ `openapi.json` được
    sinh tự động. Bản tiếng Việt ở đây **không** có phần bảng đó — hãy xem
    trang tiếng Anh để có danh sách route luôn khớp với mã nguồn.
