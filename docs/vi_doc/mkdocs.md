# MkDocs

Trang tài liệu này được dựng bằng [MkDocs](https://www.mkdocs.org) và theme
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/). Phụ thuộc
được quản lý bằng [uv](https://docs.astral.sh/uv/).

## Running the site locally

!!! warning "Chạy từ `docs/`, không phải từ thư mục gốc của repo"
    `mkdocs.yml` nằm trong `docs/`. MkDocs tìm nó ở **thư mục làm việc hiện tại**,
    nên chạy từ thư mục gốc repo sẽ lỗi
    *"Config file 'mkdocs.yml' does not exist."*

```bash
cd docs
uv sync                 # một lần: tạo .venv từ uv.lock
uv run mkdocs serve     # → http://127.0.0.1:8000 (tự tải lại khi lưu)
```

## Commands

| Lệnh | Tác dụng |
|---|---|
| `uv run mkdocs serve` | Khởi động server phát triển tự tải lại |
| `uv run mkdocs build` | Build trang tĩnh vào `docs/site/` |
| `uv run mkdocs build --strict` | Coi cảnh báo là lỗi (liên kết hỏng) — dùng trong CI |
| `uv add <package>` | Thêm một phụ thuộc (ví dụ một plugin MkDocs) |

Việc xuất bản diễn ra tự động: đẩy lên `main` sẽ kích hoạt
`.github/workflows/docs.yml`, workflow này build trang và triển khai lên GitHub
Pages (nguồn Pages phải được đặt thành **GitHub Actions**).

## Project layout

```
docs/
├── mkdocs.yml            # cấu hình trang — theme, bảng màu, nav, extension
├── pyproject.toml        # dự án + phụ thuộc (mkdocs-material, neoteroi)
├── uv.lock               # phiên bản phụ thuộc đã ghim
├── README.md             # ghi chú cho người bảo trì
├── scripts/
│   └── gen_openapi.py    # xuất đặc tả OpenAPI của bridge vào docs/api/
└── docs/                 # docs_dir — mọi thứ ở đây đều thành một trang
    ├── index.md          # trang chủ
    ├── ra_*.md           # các trang về cánh tay robot
    ├── amr_*.md          # các trang về JetRacer
    ├── api/              # Sổ tay API
    │   └── openapi.json  # ĐƯỢC SINH TỰ ĐỘNG — đừng sửa tay
    └── stylesheets/
        ├── extra.css     # bảng màu sáng/tối + kiểu dáng thành phần
        └── openapi.css   # kiểu dáng cho phần tham khảo API
```

!!! note "Về thư mục lồng nhau `docs/docs/`"
    `docs_dir` được phân giải **tương đối với `mkdocs.yml`**, đó là lý do các
    trang nằm trong `docs/docs/`. Các đường dẫn trong `mkdocs.yml` (`nav`,
    `extra_css`) là tương đối với thư mục bên trong đó.

## Adding a page

1. Tạo một file Markdown dưới `docs/docs/`.
2. Thêm nó vào danh sách `nav:` trong `mkdocs.yml` để quy định tiêu đề và vị trí.

Nếu không có mục trong `nav`, trang vẫn được build nhưng sẽ không xuất hiện trên
thanh bên.

## Translations

Trang web được build **chỉ bằng tiếng Anh**. Bản dịch tiếng Việt nằm riêng ở
`docs/vi_doc/` — cùng cấp với `docs/docs/` chứ **không** nằm trong `docs_dir`,
nên MkDocs không build chúng.

- Cấu trúc của `vi_doc/` phản chiếu `docs/docs/`, nên các liên kết tương đối
  giữa các trang tiếng Việt vẫn hoạt động khi đọc trực tiếp trên GitHub.
- Khi sửa một trang tiếng Anh, hãy cập nhật trang tương ứng trong `vi_doc/` —
  không có cơ chế nào tự phát hiện sai lệch giữa hai bên.

## The API Book

`docs/api/http.md` dựng phần tham khảo route từ `docs/api/openapi.json`, file này
được **sinh tự động** từ ứng dụng FastAPI `robot_web_bridge`:

```bash
uv sync --extra openapi
uv run python scripts/gen_openapi.py
```

CI sẽ sinh lại file này và làm hỏng build nếu bản đã commit bị lỗi thời, nhờ vậy
phần tham khảo HTTP không thể lệch khỏi mã nguồn. Tuyệt đối không sửa tay
`openapi.json`.

## Theming

Bảng màu sáng và tối được định nghĩa trong `docs/docs/stylesheets/extra.css` bằng
cách nhuộm lại các scheme `default` (sáng) và `slate` (tối) có sẵn của Material.
Để đổi một màu, hãy sửa thuộc tính CSS tương ứng (ví dụ `--md-accent-fg-color`)
trong khối `[data-md-color-scheme="…"]` liên quan.

Phông chữ (Inter + JetBrains Mono) và các tính năng theme được cấu hình dưới mục
`theme:` trong `mkdocs.yml`.

## Further reading

- [Tài liệu MkDocs](https://www.mkdocs.org)
- [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
- [Tài liệu uv](https://docs.astral.sh/uv/)
