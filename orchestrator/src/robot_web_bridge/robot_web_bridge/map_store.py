"""Editable map for the /admin portal: world bounds, obstacles and docks.

The map the operator edits lives in two YAML files in the config dir:

- ``docks.yaml`` — the dock registry the user pages already read. Docks *are*
  the robot's navigation targets, so dragging one on the admin map edits this
  file.
- ``map.yaml``   — everything else the editor owns: the world-frame bounds the
  canvas shows and operator-drawn rectangular keep-out obstacles (x/y = centre,
  w/h in metres, world frame).

Saves are atomic (tmp file + ``os.replace``) so a crash mid-write can't corrupt
the config the app boots from. :func:`sanitize` is the single validator between
the browser's JSON and what gets persisted / published to the robot.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import yaml

DEFAULT_BOUNDS = {"x_min": -1.0, "x_max": 6.0, "y_min": -3.0, "y_max": 4.0}
MAX_OBSTACLES = 200
MIN_OBSTACLE_M = 0.05          # smallest obstacle side we keep, metres
_COORD_LIMIT = 1000.0          # sanity clamp on any world coordinate
_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")

_DOCKS_HEADER = (
    "# Dock registry — the set of valid QR targets. The QR at each table/dock\n"
    "# deep-links to <base>/?dock=<dock_id> and the user page shows `label`.\n"
    "# This file is maintained by the /admin map editor; hand edits are fine\n"
    "# but will be reformatted on the next admin save.\n"
)
_MAP_HEADER = (
    "# Admin map: world bounds shown by the /admin editor + operator-drawn\n"
    "# rectangular keep-out obstacles (x/y = centre, w/h in metres, world frame).\n"
    "# Maintained by the /admin map editor.\n"
)


def _f(value, default: float = 0.0) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(default)
    return max(-_COORD_LIMIT, min(_COORD_LIMIT, v))


def _obstacle(raw: dict, index: int) -> dict:
    if not isinstance(raw, dict):
        raise ValueError(f"obstacle #{index + 1} is not an object")
    oid = str(raw.get("id") or "")
    if not _ID_RE.match(oid):
        oid = f"obs-{index + 1}"
    return {
        "id": oid,
        "x": _f(raw.get("x")),
        "y": _f(raw.get("y")),
        "w": max(MIN_OBSTACLE_M, _f(raw.get("w"), MIN_OBSTACLE_M)),
        "h": max(MIN_OBSTACLE_M, _f(raw.get("h"), MIN_OBSTACLE_M)),
    }


def _bounds(raw) -> dict:
    raw = raw if isinstance(raw, dict) else {}
    b = {k: _f(raw.get(k), v) for k, v in DEFAULT_BOUNDS.items()}
    if b["x_max"] - b["x_min"] < 0.5 or b["y_max"] - b["y_min"] < 0.5:
        raise ValueError("bounds must span at least 0.5 m on each axis")
    return b


# ── load ──────────────────────────────────────────────────────────────────────
def load_map(config_dir: Path) -> dict:
    """Bounds + obstacles from map.yaml; lenient so a bad hand-edit can't stop boot."""
    try:
        data = yaml.safe_load((config_dir / "map.yaml").read_text()) or {}
    except (FileNotFoundError, yaml.YAMLError):
        data = {}
    try:
        bounds = _bounds(data.get("bounds"))
    except ValueError:
        bounds = dict(DEFAULT_BOUNDS)
    obstacles = []
    raw = data.get("obstacles")
    for i, o in enumerate(raw if isinstance(raw, list) else []):
        try:
            obstacles.append(_obstacle(o, i))
        except ValueError:
            continue  # skip a broken entry rather than failing boot
    return {"bounds": bounds, "obstacles": obstacles[:MAX_OBSTACLES]}


# ── validate a browser save ───────────────────────────────────────────────────
def sanitize(payload, home_dock: str) -> tuple[dict, dict, list[dict]]:
    """Validate the editor's JSON → (docks, bounds, obstacles). Raises ValueError."""
    if not isinstance(payload, dict):
        raise ValueError("body must be a JSON object")

    bounds = _bounds(payload.get("bounds"))

    raw_docks = payload.get("docks")
    if not isinstance(raw_docks, dict) or not raw_docks:
        raise ValueError("map needs at least one dock")
    docks: dict[str, dict] = {}
    for dock_id, d in raw_docks.items():
        dock_id = str(dock_id).strip()
        if not _ID_RE.match(dock_id):
            raise ValueError(f"bad dock id {dock_id!r} (use letters, digits, - or _)")
        d = d if isinstance(d, dict) else {}
        docks[dock_id] = {
            "label": str(d.get("label") or dock_id)[:64],
            "pose_x": _f(d.get("pose_x")),
            "pose_y": _f(d.get("pose_y")),
            "yaw": _f(d.get("yaw")),
        }
    if home_dock not in docks:
        raise ValueError(f"the home dock {home_dock!r} cannot be deleted")

    raw_obs = payload.get("obstacles", [])
    if not isinstance(raw_obs, list):
        raise ValueError("obstacles must be a list")
    if len(raw_obs) > MAX_OBSTACLES:
        raise ValueError(f"too many obstacles (max {MAX_OBSTACLES})")
    obstacles = [_obstacle(o, i) for i, o in enumerate(raw_obs)]

    return docks, bounds, obstacles


# ── save ──────────────────────────────────────────────────────────────────────
def _atomic_write(path: Path, text: str) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def save_map(config_dir: Path, bounds: dict, obstacles: list[dict]) -> None:
    config_dir.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump({"bounds": bounds, "obstacles": obstacles}, sort_keys=False)
    _atomic_write(config_dir / "map.yaml", _MAP_HEADER + body)


def save_docks(config_dir: Path, docks: dict[str, dict]) -> None:
    config_dir.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump({"docks": docks}, sort_keys=False)
    _atomic_write(config_dir / "docks.yaml", _DOCKS_HEADER + body)
