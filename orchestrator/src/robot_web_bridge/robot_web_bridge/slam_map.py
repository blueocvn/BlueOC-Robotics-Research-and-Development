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


# ── public ─────────────────────────────────────────────────────────────────────
def load_slam_map(yaml_path: Path) -> dict | None:
    """Return the render payload for ``yaml_path``, or ``None`` if unavailable.

    Payload: ``image`` (PNG data URI), pixel size, ``resolution`` (m/px),
    ``origin`` [x, y, yaw], world ``bounds`` and a display ``rotation_deg``.
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

    px = bytearray(w * h * 2)
    for i in range(w * h):
        v = data[i]
        if mask[i]:
            px[2 * i], px[2 * i + 1] = FREE_GRAY, 255
        elif v <= OCC_V and halo[i]:
            px[2 * i], px[2 * i + 1] = WALL_GRAY, 255
        # else leave transparent (alpha 0)

    data_uri = "data:image/png;base64," + base64.b64encode(_png_gray_alpha(w, h, bytes(px))).decode()

    # Keep the admin view in true map/cartesian orientation by default.
    # If needed, operators can still set display_rotation_deg in the map YAML.
    override = meta.get("display_rotation_deg")
    rotation = float(override) if override is not None else 0.0

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
