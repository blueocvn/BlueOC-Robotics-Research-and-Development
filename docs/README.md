# Documentation site

This folder is a [MkDocs](https://www.mkdocs.org/) project using the
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme. It
builds the Robot Fulfillment docs into a static site with light/dark color
modes. Dependencies are managed with [uv](https://docs.astral.sh/uv/).

## Layout

```
docs/
├── pyproject.toml            # project + deps (mkdocs-material)
├── uv.lock                   # pinned dependency versions
├── mkdocs.yml                # site config (theme, palette, nav, extensions)
├── README.md                 # this file — setup instructions
└── docs/                     # docs_dir: everything here becomes a page
    ├── index.md
    ├── robot-arm.md
    ├── jetracer.md
    ├── ...
    └── stylesheets/
        └── extra.css         # light/dark color themes
```

> **Note** — the site content lives in `docs/docs/`, not `docs/`. Paths in
> `mkdocs.yml` (`extra_css`, `nav`, links) are relative to that inner folder.

## Prerequisites

- [`uv`](https://docs.astral.sh/uv/) — install with
  `curl -LsSf https://astral.sh/uv/install.sh | sh` (or `pipx install uv`).

uv manages the Python version and the virtual environment for you; there's no
separate `pip install` or `venv` step.

## Setup

From the `docs/` folder:

```bash
uv sync          # creates .venv and installs the locked dependencies
```

That's it — `uv sync` reads `pyproject.toml` / `uv.lock` and provisions
everything. To add a dependency later (e.g. a plugin):

```bash
uv add mkdocs-git-revision-date-localized-plugin
```

## Preview locally

```bash
uv run mkdocs serve
```

Open <http://127.0.0.1:8000>. The dev server live-reloads on every save. Use a
different port with `uv run mkdocs serve -a 127.0.0.1:8001`.

## Build the static site

```bash
uv run mkdocs build            # outputs to docs/site/
uv run mkdocs build --strict   # fail on warnings (broken links, etc.) — use in CI
```

`site/` and `.venv/` are generated and git-ignored.

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
2. Add it to the `nav:` list in `mkdocs.yml` to control ordering and titles;
   without `nav`, MkDocs auto-builds the navigation from the file tree.

## Deploy to GitHub Pages

```bash
uv run mkdocs gh-deploy        # builds and pushes to the gh-pages branch
```
