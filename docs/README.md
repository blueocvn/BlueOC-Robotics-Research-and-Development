# Documentation site

This folder is a [MkDocs](https://www.mkdocs.org/) project using the
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme. It
builds the Robot Fulfillment docs into a static site with light/dark color
modes.

## Layout

```
docs/
├── mkdocs.yml                 # site config (theme, palette, nav, extensions)
├── README.md                 # this file — setup instructions
└── docs/                     # docs_dir: everything here becomes a page
    ├── index.md
    ├── GET-STARTED.md
    ├── CALIBRATION.md
    ├── orchestrator.md
    ├── THIRDPARTY_SETUP.md
    └── stylesheets/
        └── extra.css         # light/dark color themes
```

> **Note** — the site content lives in `docs/docs/`, not `docs/`. Paths in
> `mkdocs.yml` (`extra_css`, `nav`, links) are relative to that inner folder.

## Prerequisites

- Python 3.8+
- `pip` (ideally inside a virtual environment)

## Setup

From the `docs/` folder:

```bash
# 1. (recommended) create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# 2. install MkDocs + the Material theme
#    (mkdocs-material pulls in mkdocs and the pymdownx extensions)
pip install mkdocs-material
```

To pin the tooling for reproducible builds, capture it in a requirements file:

```bash
pip freeze | grep -E '^(mkdocs|mkdocs-material|pymdown-extensions)==' > requirements.txt
# later, on another machine:
pip install -r requirements.txt
```

## Preview locally

```bash
cd docs
mkdocs serve
```

Open <http://127.0.0.1:8000>. The dev server live-reloads on every save. Use a
different port with `mkdocs serve -a 127.0.0.1:8001`.

## Build the static site

```bash
cd docs
mkdocs build            # outputs to docs/site/
mkdocs build --strict   # fail on warnings (broken links, etc.) — use in CI
```

`site/` is generated output — keep it out of version control (add `docs/site/`
to `.gitignore` if it isn't already).

## Theme / color modes

Light and dark palettes are defined in
[`docs/stylesheets/extra.css`](docs/stylesheets/extra.css) by retinting
Material's built-in `default` (light) and `slate` (dark) schemes:

| Mode  | Reference          | Background | Accent    |
|-------|--------------------|------------|-----------|
| Light | `robot_arm_mt.html`| `#FFFFFF`  | `#1868B0` |
| Dark  | `jetracer-mt.html` | `#0F1419`  | `#4FE0C8` |

The palette toggle (sun/moon icon in the header) and the default fonts
(Inter + JetBrains Mono) are configured under `theme:` in `mkdocs.yml`. To
change a color, edit the matching CSS custom property (e.g.
`--md-accent-fg-color`) under the relevant `[data-md-color-scheme="…"]` block.

## Add a page

1. Create a Markdown file under `docs/docs/`.
2. (Optional) add it to a `nav:` list in `mkdocs.yml` to control ordering and
   titles; without `nav`, MkDocs auto-builds the navigation from the file tree.

## Deploy to GitHub Pages

```bash
cd docs
mkdocs gh-deploy        # builds and pushes to the gh-pages branch
```
