"""Render a SLAM occupancy map (ROS map_server pgm+yaml) for the admin editor.

The operator's map editor draws docks/obstacles on top of the *real* environment,
so we load the robot's SLAM map and hand the browser a cleaned, georeferenced PNG:

- **Clean** — an untouched SLAM grid is mostly "unknown" grey with a spray of stray
  LIDAR rays. We keep only the largest connected free-space region plus the walls
  that bound it; everything else is transparent, so the editor canvas shows through.
- **Georeferenced** — the PNG covers the map's true world extent (from ``resolution``
  + ``origin``), so a dock dropped on a wall gets the pose the robot actually uses.
  Coordinates are never rotated: the editor frame *is* the SLAM/nav frame.
- **Display rotation (optional)** — by default the editor stays axis-aligned to the
    true SLAM/cartesian frame. If an operator wants a visual rotation, set
    ``display_rotation_deg`` in the map YAML; the browser rotates the whole picture
    (image + grid + markers) while stored coordinates remain in the true frame.

Pure-stdlib (``zlib`` only) so the package gains no new dependency: a minimal
grayscale+alpha PNG encoder emits the data URI.
"""

from __future__ import annotations

import base64
import math
import os
import struct
import zlib
from collections import deque
from pathlib import Path

import yaml

FREE_V = 250          # >= this  → free space (white)
OCC_V = 80            # <= this  → wall / occupied (black)
FREE_GRAY = 244       # rendered free tone
WALL_GRAY = 55        # rendered wall tone
WALL_HALO_PX = 4      # keep walls within this many px of the main free region

# ── app style (robot-vacuum look): soft blue floor + blocky blue walls ──────────
# Opt in with ROBOT_WEB_BRIDGE_MAP_STYLE=app. Rendering stays in the ORIGINAL frame
# (same size/origin/resolution → same world bounds), so coordinates are unchanged;
# the squared "leveled" look is a display-only rotation (see `rotation_deg` below).
MAP_STYLE = os.environ.get("ROBOT_WEB_BRIDGE_MAP_STYLE", "classic").strip().lower()
BLOCK_PX = max(1, int(os.environ.get("ROBOT_WEB_BRIDGE_MAP_BLOCK", "3")))  # px per "pixel" cell
APP_FLOOR_RGB = (188, 214, 240)   # #bcd6f0 soft blue floor
APP_WALL_RGB = (74, 144, 217)     # #4a90d9 medium-blue walls / outline


# ── PGM ────────────────────────────────────────────────────────────────────────
def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    with path.open("rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError("not a binary (P5) PGM")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        int(f.readline())  # maxval
        data = f.read(w * h)
    if len(data) < w * h:
        raise ValueError("truncated PGM")
    return w, h, data


# ── cleanup: largest free component + bounding walls ────────────────────────────
def _main_region_mask(w: int, h: int, data: bytes) -> bytearray:
    """1 where a pixel belongs to the largest free-space blob, else 0."""
    seen = bytearray(w * h)
    best: list[int] = []
    for start in range(w * h):
        if seen[start] or data[start] < FREE_V:
            continue
        comp: list[int] = []
        q = deque([start])
        seen[start] = 1
        while q:
            i = q.popleft()
            comp.append(i)
            x, y = i % w, i // w
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h:
                    j = ny * w + nx
                    if not seen[j] and data[j] >= FREE_V:
                        seen[j] = 1
                        q.append(j)
        if len(comp) > len(best):
            best = comp
    mask = bytearray(w * h)
    for i in best:
        mask[i] = 1
    return mask


def _dilate(w: int, h: int, mask: bytearray, iterations: int) -> bytearray:
    for _ in range(iterations):
        out = bytearray(mask)
        for y in range(h):
            row = y * w
            for x in range(w):
                if mask[row + x]:
                    continue
                if ((x and mask[row + x - 1]) or (x + 1 < w and mask[row + x + 1])
                        or (y and mask[row - w + x]) or (y + 1 < h and mask[row + w + x])):
                    out[row + x] = 1
        mask = out
    return mask


# ── minimal grayscale+alpha PNG ────────────────────────────────────────────────
def _png_gray_alpha(w: int, h: int, pixels: bytes) -> bytes:
    """pixels = w*h*2 bytes (gray, alpha per pixel). Returns a PNG byte string."""
    raw = bytearray()
    stride = w * 2
    for y in range(h):
        raw.append(0)  # filter: none
        raw.extend(pixels[y * stride:(y + 1) * stride])

    def chunk(tag: bytes, payload: bytes) -> bytes:
        body = tag + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 4, 0, 0, 0)  # 8-bit, colour type 4 (gray+alpha)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
            + chunk(b"IEND", b""))


# ── vector cells: snap a mask to a block grid, then emit merged rectangles ───────
def _cell_grid(w: int, h: int, mask: bytearray, block: int, cov: float) -> tuple[int, int, bytearray]:
    """0/1 grid at cell resolution: a cell is on when >= `cov` of its pixels are set.
    Snapping to cells is what gives the crisp square 'pixel' look — but as vectors."""
    cw, ch = w // block, h // block
    grid = bytearray(cw * ch)
    need = cov * block * block
    for cy in range(ch):
        for cx in range(cw):
            count = 0
            for y in range(cy * block, cy * block + block):
                row = y * w
                for x in range(cx * block, cx * block + block):
                    count += mask[row + x]
            if count >= need:
                grid[cy * cw + cx] = 1
    return cw, ch, grid


def _rects_path(cw: int, ch: int, grid: bytearray, block: int) -> str:
    """Merge on-cells into horizontal run rectangles and return one SVG path `d`
    string (compact: one M..h..v..h..z subpath per run, in full-pixel coordinates)."""
    parts = []
    for cy in range(ch):
        x = 0
        while x < cw:
            if grid[cy * cw + x]:
                x0 = x
                while x < cw and grid[cy * cw + x]:
                    x += 1
                px, py, ww, hh = x0 * block, cy * block, (x - x0) * block, block
                parts.append(f"M{px} {py}h{ww}v{hh}h{-ww}z")
            else:
                x += 1
    return "".join(parts)


# ── orientation: angle that levels the dominant walls ──────────────────────────
def _level_angle_deg(w: int, h: int, mask: bytearray) -> float:
    """Rotation (deg) whose min-area bounding box aligns the main region to axes."""
    pts = [(i % w, i // w) for i in range(w * h) if mask[i]]
    if not pts:
        return 0.0
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)

    def area(theta: float) -> float:
        c, s = math.cos(theta), math.sin(theta)
        xs = [(x - cx) * c - (y - cy) * s for x, y in pts]
        ys = [(x - cx) * s + (y - cy) * c for x, y in pts]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    coarse = min((i * 0.5 for i in range(-40, 41)), key=lambda a: area(math.radians(a)))
    fine = min((coarse + i * 0.1 for i in range(-5, 6)), key=lambda a: area(math.radians(a)))
    return round(fine, 2)


# ── app-style render (soft blue floor + blocky blue walls, original frame) ──────
def _render_app_style(w: int, h: int, data: bytes, mask: bytearray, halo: bytearray) -> str:
    """Robot-vacuum look as a georeferenced *SVG* data URI: soft blue floor + blocky
    blue walls, drawn as vectors so it stays crisp at any zoom (no pixelation). The
    viewBox is the input pixel grid (0 0 w h), so it stretches over the world bounds
    exactly like the raster map did — coordinates are unchanged."""
    wall = bytearray(w * h)
    for i in range(w * h):
        if data[i] <= OCC_V and halo[i] and not mask[i]:
            wall[i] = 1

    cw, ch, floor_cells = _cell_grid(w, h, mask, BLOCK_PX, 0.5)
    _, _, wall_cells = _cell_grid(w, h, wall, BLOCK_PX, 0.25)  # low coverage keeps thin walls
    floor_d = _rects_path(cw, ch, floor_cells, BLOCK_PX)
    wall_d = _rects_path(cw, ch, wall_cells, BLOCK_PX)

    floor_hex = "#%02x%02x%02x" % APP_FLOOR_RGB
    wall_hex = "#%02x%02x%02x" % APP_WALL_RGB
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}" shape-rendering="crispEdges">'
        f'<path fill="{floor_hex}" d="{floor_d}"/>'
        f'<path fill="{wall_hex}" d="{wall_d}"/>'  # walls painted over the floor
        f'</svg>'
    )
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode()).decode()


# ── public ─────────────────────────────────────────────────────────────────────
def load_slam_map(yaml_path: Path) -> dict | None:
    """Return the render payload for ``yaml_path``, or ``None`` if unavailable.

    Payload: ``image`` (PNG data URI in classic style, crisp SVG in app style),
    pixel size, ``resolution`` (m/px), ``origin`` [x, y, yaw], world ``bounds``
    and a display ``rotation_deg``.
    """
    try:
        meta = yaml.safe_load(yaml_path.read_text()) or {}
        pgm = yaml_path.parent / meta["image"]
        w, h, data = _read_pgm(pgm)
    except (FileNotFoundError, KeyError, ValueError, yaml.YAMLError):
        return None

    res = float(meta.get("resolution", 0.05))
    ox, oy, oyaw = (list(meta.get("origin", [0.0, 0.0, 0.0])) + [0.0, 0.0, 0.0])[:3]

    mask = _main_region_mask(w, h, data)
    halo = _dilate(w, h, mask, WALL_HALO_PX)

    if MAP_STYLE == "app":
        # Prefer a sibling <stem>.svg exported offline by the beautification notebook
        # (contour-traced -> smooth, matches the notebook). It's in the same pixel frame
        # (viewBox 0 0 w h), so it georeferences exactly like the raster. Fall back to the
        # stdlib blocky render when no SVG has been exported.
        svg_sibling = yaml_path.with_suffix(".svg")
        if svg_sibling.is_file():
            data_uri = "data:image/svg+xml;base64," + base64.b64encode(svg_sibling.read_bytes()).decode()
        else:
            data_uri = _render_app_style(w, h, data, mask, halo)
    else:
        px = bytearray(w * h * 2)
        for i in range(w * h):
            v = data[i]
            if mask[i]:
                px[2 * i], px[2 * i + 1] = FREE_GRAY, 255
            elif v <= OCC_V and halo[i]:
                px[2 * i], px[2 * i + 1] = WALL_GRAY, 255
            # else leave transparent (alpha 0)
        data_uri = "data:image/png;base64," + base64.b64encode(_png_gray_alpha(w, h, bytes(px))).decode()

    # Display orientation. Coordinates are NEVER rotated (the browser rotates the
    # whole picture via `rotation_deg` and maps clicks back through that transform),
    # so leveling is purely cosmetic. Priority: explicit YAML override, else "auto"
    # (square the walls) for the app style, else stay in true cartesian orientation.
    override = meta.get("display_rotation_deg")
    if override is not None and str(override).strip().lower() != "auto":
        rotation = float(override)
    elif str(override).strip().lower() == "auto" or MAP_STYLE == "app":
        rotation = _level_angle_deg(w, h, mask)
    else:
        rotation = 0.0

    return {
        "image": data_uri,
        "width_px": w,
        "height_px": h,
        "resolution": res,
        "origin": [ox, oy, oyaw],
        "bounds": {
            "x_min": round(ox, 3),
            "x_max": round(ox + w * res, 3),
            "y_min": round(oy, 3),
            "y_max": round(oy + h * res, 3),
        },
        "rotation_deg": rotation,
    }
