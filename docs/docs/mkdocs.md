# MkDocs

This documentation site is built with [MkDocs](https://www.mkdocs.org) and the
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme.
Dependencies are managed with [uv](https://docs.astral.sh/uv/).

## Running the site locally

!!! warning "Run from `docs/`, not the repo root"
    `mkdocs.yml` lives in `docs/`. MkDocs looks for it in the **current working
    directory**, so running from the repo root fails with
    *"Config file 'mkdocs.yml' does not exist."*

```bash
cd docs
uv sync                 # one-time: creates .venv from uv.lock
uv run mkdocs serve     # → http://127.0.0.1:8000 (live-reloads on save)
```

## Commands

| Command | What it does |
|---|---|
| `uv run mkdocs serve` | Start the live-reloading dev server |
| `uv run mkdocs build` | Build the static site into `docs/site/` |
| `uv run mkdocs build --strict` | Fail on warnings (broken links) — use in CI |
| `uv add <package>` | Add a dependency (e.g. an MkDocs plugin) |

Publishing is automatic: pushing to `main` triggers `.github/workflows/docs.yml`,
which builds the site and deploys it to GitHub Pages (Pages source must be set to
**GitHub Actions**).

## Project layout

```
docs/
├── mkdocs.yml            # site config — theme, palette, nav, extensions
├── pyproject.toml        # project + deps (mkdocs-material)
├── uv.lock               # pinned dependency versions
├── README.md             # maintainer notes
└── docs/                 # docs_dir — everything here becomes a page
    ├── index.md          # home page
    ├── ra_*.md           # robot arm pages
    ├── amr_*.md          # JetRacer pages
    └── stylesheets/
        └── extra.css     # light/dark palettes + component styling
```

!!! note "The nested `docs/docs/`"
    `docs_dir` is resolved **relative to `mkdocs.yml`**, which is why the pages
    live in `docs/docs/`. Paths in `mkdocs.yml` (`nav`, `extra_css`) are relative
    to that inner folder.

## Adding a page

1. Create a Markdown file under `docs/docs/`.
2. Add it to the `nav:` list in `mkdocs.yml` to control its title and position.

Without a `nav` entry the page still builds, but it won't appear in the sidebar.

## Theming

Light and dark palettes are defined in `docs/docs/stylesheets/extra.css` by
retinting Material's built-in `default` (light) and `slate` (dark) schemes. To
change a colour, edit the matching CSS custom property (e.g.
`--md-accent-fg-color`) under the relevant `[data-md-color-scheme="…"]` block.

Fonts (Inter + JetBrains Mono) and theme features are configured under `theme:`
in `mkdocs.yml`.

## Further reading

- [MkDocs documentation](https://www.mkdocs.org)
- [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
- [uv documentation](https://docs.astral.sh/uv/)