"""FastAPI app — Robot Web Bridge (user pages + ROS bridge).

The public, QR-scanned user surface for the JetRacer is served at the plain
root — no version prefix, so QR codes and hand-typed hosts stay short:

    GET  /                     landing page (reads ?dock=<id> from the QR deep-link)
    POST /orders              place a water/refill order -> task screen
    GET  /orders/{id}         task screen (also the HTMX poll target while live)
    POST /orders/{id}/cancel  cancel a still-queued/in-progress order
    POST /abort              emergency: abort the active order + cancel the queue
    GET  /docks              dock registry (JSON)
    GET  /state              live snapshot (backend mode, docking_state, odom)

Plus the PIN-gated operator console, kept under /v1 since it's an internal
surface rather than something meant to be typed/scanned by a customer (same
Tailwind/HTMX brand styling):

    GET  /v1/admin              operator console (login screen when no session)
    GET  /v1/admin/map          editable map: bounds + docks + obstacles
    POST /v1/admin/map          save the map (persists YAML, republishes obstacles)
    …plus the operator actions in §3.3 (login/teleop/dock/reset_pose/…)

Rendering uses Jinja2 + HTMX: a normal browser navigation gets the full page
(base.html wrapping a content template); an HTMX request (HX-Request header) gets
just the swapped fragment. Tailwind + HTMX are loaded from CDNs, so there is no
build step — exactly what the requirements doc asks for behind ngrok.

On startup the app spins the rclpy bridge node + the single dispatcher that owns
the robot. If ROS 2 isn't available it transparently falls back to a simulated
backend, so the user pages work in plain Python too. The PIN-gated /admin portal
is layered on top of this; see requirements-claude-plan.md.
"""

from __future__ import annotations

import asyncio
import datetime as dt
import logging
import os
import re
import subprocess
from contextlib import asynccontextmanager
from pathlib import Path

import yaml
from fastapi import APIRouter, Depends, FastAPI, Form, Request, Response
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from . import auth, map_store, slam_map
from .dispatcher import HOME_DOCK, Dispatcher, RosBackend, SimBackend
from .ros_node import shutdown_ros_node, start_ros_node
from .store import STEPS, Store

log = logging.getLogger("robot_web_bridge")

BASE_DIR = Path(__file__).resolve().parent
TEMPLATES_DIR = BASE_DIR / "templates"
STATIC_DIR = BASE_DIR / "static"


def _default_config_dir() -> Path:
    """Locate config/ whether running from source or from a colcon install.

    ament_python's install layout splits the two: the module ends up under
    ``install/robot_web_bridge/lib/python3.x/site-packages/robot_web_bridge/``
    while the ``data_files`` in setup.py (docks.yaml, slam_map.*) land under
    the unrelated ``install/robot_web_bridge/share/robot_web_bridge/config/``.
    So ``BASE_DIR.parent / "config"`` (correct for a source checkout) is empty
    in an installed deployment — fall back to the ament share dir there.
    """
    src_config = BASE_DIR.parent / "config"
    if src_config.exists():
        return src_config
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("robot_web_bridge")) / "config"
    except Exception:
        return src_config


CONFIG_DIR = Path(os.environ.get("ROBOT_WEB_BRIDGE_CONFIG", _default_config_dir()))
# SLAM occupancy map rendered behind the admin editor (ROS map_server yaml).
SLAM_YAML = Path(os.environ.get("ROBOT_WEB_BRIDGE_SLAM_MAP", CONFIG_DIR / "slam_map.yaml"))
# Destination folder for serialized SLAM maps consumed by Nav2 map_server.
NAV_MAP_DIR = Path(os.environ.get("ROBOT_WEB_BRIDGE_NAV_MAP_DIR", "/ros2_ws/maps"))
_MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
_SLAM_EXPORT_TIMEOUT_S = float(os.environ.get("ROBOT_WEB_BRIDGE_SLAM_EXPORT_TIMEOUT", "60"))
_LOAD_MAP_TIMEOUT_S = float(os.environ.get("ROBOT_WEB_BRIDGE_LOAD_MAP_TIMEOUT", "20"))
_LOAD_MAP_SERVICE = os.environ.get("ROBOT_WEB_BRIDGE_LOAD_MAP_SERVICE", "/map_server/load_map")
_LOAD_MAP_TYPE = os.environ.get("ROBOT_WEB_BRIDGE_LOAD_MAP_TYPE", "nav2_msgs/srv/LoadMap")

store = Store()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Bring up the robot bridge (or simulation) + the dispatcher loop."""
    node = start_ros_node()
    backend = RosBackend(node) if node is not None else SimBackend()
    dispatcher = Dispatcher(store, backend)
    app.state.backend = backend
    app.state.dispatcher = dispatcher
    app.state.ros_node = node
    log.warning("robot bridge backend: %s", backend.name)
    # Re-announce the persisted obstacle set (latched topic) on every boot.
    backend.set_obstacles(map_store.load_map(CONFIG_DIR)["obstacles"])
    backend.set_docks(DOCKS)
    task = asyncio.create_task(dispatcher.run())
    try:
        yield
    finally:
        task.cancel()
        shutdown_ros_node(node)


app = FastAPI(title="Robot Web Bridge", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
templates = Jinja2Templates(directory=str(TEMPLATES_DIR))


# ── dock registry ────────────────────────────────────────────────────────────
def load_docks() -> dict[str, dict]:
    path = CONFIG_DIR / "docks.yaml"
    try:
        data = yaml.safe_load(path.read_text()) or {}
        return data.get("docks", {})
    except FileNotFoundError:
        return {}


DOCKS = load_docks()


def default_dock() -> str:
    """A QR-less visitor is a *table*, never the home/water station.

    Computed on demand (not at import) because the admin map editor can add,
    rename or delete docks while the app is running.
    """
    return next((d for d in DOCKS if d != HOME_DOCK), next(iter(DOCKS), "dock1"))


def resolve_dock(dock_id: str | None) -> tuple[str, str]:
    """Return (dock_id, label), falling back to the first registered dock."""
    if dock_id and dock_id in DOCKS:
        return dock_id, DOCKS[dock_id].get("label", dock_id)
    if dock_id:  # unknown id from a stray QR — keep it, label = the raw id
        return dock_id, dock_id
    fallback = default_dock()
    return fallback, DOCKS.get(fallback, {}).get("label", fallback)


# ── render helper ─────────────────────────────────────────────────────────────
def render(request: Request, content_template: str, **ctx) -> HTMLResponse:
    """Full page for a normal load; bare fragment for an HTMX swap."""
    ctx = {"steps": STEPS, **ctx}
    is_htmx = request.headers.get("HX-Request") == "true"
    if is_htmx:
        return templates.TemplateResponse(request=request, name=content_template, context=ctx)
    return templates.TemplateResponse(
        request=request, name="base.html",
        context={**ctx, "content_template": content_template},
    )


# ── routes ────────────────────────────────────────────────────────────────────
# Customer-facing user pages hang off the bare root (no version prefix) — this
# is the QR-scanned surface, so the URL stays short. The operator console below
# uses its own /v1-prefixed router since it's an internal, not customer, surface.
router = APIRouter()


@router.get("/", response_class=HTMLResponse)
def landing(request: Request, dock: str | None = None):
    dock_id, label = resolve_dock(dock)
    return render(
        request, "landing.html",
        dock_id=dock_id, dock_label=label,
        robot_busy=request.app.state.dispatcher.robot_busy(),
    )


@router.post("/orders", response_class=HTMLResponse)
def place_order(request: Request, type: str = Form("water"), dock: str = Form(...)):
    dock_id, label = resolve_dock(dock)
    order_type = type if type in ("water", "refill") else "water"
    order = store.place(dock_id, order_type)
    view = store.view(order)
    return render(request, "task.html", dock_label=label, order=view)


@router.get("/orders/{order_id}", response_class=HTMLResponse)
def order_screen(request: Request, order_id: int):
    order = store.get(order_id)
    if order is None:
        return RedirectResponse("/", status_code=303)
    _, label = resolve_dock(order.dock_id)
    return render(request, "task.html", dock_label=label, order=store.view(order))


@router.post("/orders/{order_id}/cancel", response_class=HTMLResponse)
def cancel_order(request: Request, order_id: int):
    # If this order currently holds the robot, tell the robot to stop too.
    active = store.active()
    if active is not None and active.id == order_id:
        request.app.state.backend.abort()
    store.cancel(order_id)
    dock_id, label = resolve_dock(None)
    return render(
        request, "landing.html",
        dock_id=dock_id, dock_label=label,
        robot_busy=request.app.state.dispatcher.robot_busy(),
    )


@router.post("/abort")
def api_abort(request: Request):
    """Emergency stop: abort the active order and clear the queue."""
    request.app.state.backend.abort()
    active = store.active()
    if active is not None:
        store.cancel(active.id)
    cancelled = 0
    while (nxt := store.next_queued()) is not None:
        store.cancel(nxt.id)
        cancelled += 1
    return {"aborted": active.id if active else None, "queued_cancelled": cancelled}


@router.get("/docks")
def api_docks():
    return JSONResponse({"docks": DOCKS})


@router.get("/state")
def api_state(request: Request):
    """Live robot snapshot — cycle phase, backend mode, docking_state + odom pose."""
    dispatcher = request.app.state.dispatcher
    active = store.active()
    return {
        "robot_busy": dispatcher.robot_busy(),
        "active_order": active.id if active else None,
        "robot": dispatcher.robot_snapshot(),
        **request.app.state.backend.snapshot(),
    }


# ── admin portal (PIN-gated) ──────────────────────────────────────────────────
# The operator console: JSON API + the server-rendered /admin page, kept under
# /v1 since it's internal rather than a customer-facing surface. All actuation
# routes are gated by the operator session cookie (auth.require_admin).
admin_router = APIRouter(prefix="/v1")
ADMIN = [Depends(auth.require_admin)]


def map_payload() -> dict:
    """The full editor state: world bounds, docks, obstacles + SLAM background.

    Before the operator has saved anything (no map.yaml yet) the editor bounds
    default to the SLAM map's extent, so the occupancy image fills the canvas.
    """
    m = map_store.load_map(CONFIG_DIR)
    slam = slam_map.load_slam_map(SLAM_YAML)
    bounds = m["bounds"]
    if slam is not None and not (CONFIG_DIR / "map.yaml").exists():
        bounds = slam["bounds"]
    return {
        "bounds": bounds,
        "obstacles": m["obstacles"],
        "docks": DOCKS,
        "home_dock": HOME_DOCK,
        "slam": slam,
    }


@admin_router.get("/admin", response_class=HTMLResponse)
def admin_portal(request: Request):
    """Operator console — login screen without a valid session, else the dashboard."""
    if not auth.is_admin(request):
        return templates.TemplateResponse(request=request, name="admin_login.html", context={})
    return templates.TemplateResponse(
        request=request, name="admin.html", context={"boot": map_payload()},
    )


@admin_router.get("/admin/map", dependencies=ADMIN)
def admin_map():
    """The editable map: world bounds + docks + obstacles + SLAM background."""
    return map_payload()


@admin_router.post("/admin/map", dependencies=ADMIN)
async def admin_map_save(request: Request):
    """Persist the edited map, swap the live dock registry, republish obstacles."""
    try:
        payload = await request.json()
        docks, bounds, obstacles = map_store.sanitize(payload, home_dock=HOME_DOCK)
    except (ValueError, TypeError) as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=422)
    map_store.save_map(CONFIG_DIR, bounds, obstacles)
    map_store.save_docks(CONFIG_DIR, docks)
    DOCKS.clear()
    DOCKS.update(docks)  # in place: resolve_dock/api_docks share this dict
    request.app.state.backend.set_obstacles(obstacles)
    request.app.state.backend.set_docks(docks)
    return {"ok": True, "docks": len(docks), "obstacles": len(obstacles)}


@admin_router.post("/admin/sync_topics", dependencies=ADMIN)
async def admin_sync_topics(request: Request):
    """Push current editor docks+obstacles to robot topics without persisting files."""
    try:
        payload = await request.json()
        docks, _bounds, obstacles = map_store.sanitize(payload, home_dock=HOME_DOCK)
    except (ValueError, TypeError) as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=422)

    request.app.state.backend.set_obstacles(obstacles)
    request.app.state.backend.set_docks(docks)
    return {
        "ok": True,
        "docks": len(docks),
        "obstacles": len(obstacles),
        "topics": ["/virtual_obstacles", "/dock_registry"],
    }


@admin_router.post("/admin/login")
def admin_login(pin: str = Form("")):
    if not auth.check_pin(pin):
        return JSONResponse({"ok": False, "error": "bad pin"}, status_code=401)
    resp = JSONResponse({"ok": True})
    resp.set_cookie(auth.COOKIE, auth.make_token(), httponly=True,
                    samesite="lax", max_age=auth.TTL)
    return resp


@admin_router.post("/admin/logout")
def admin_logout():
    resp = JSONResponse({"ok": True})
    resp.delete_cookie(auth.COOKIE)
    return resp


@admin_router.get("/admin/session")
def admin_session(request: Request):
    """Cheap check the frontend can use to know if it's still logged in."""
    return {"authenticated": auth.is_admin(request)}


@admin_router.get("/admin/orders", dependencies=ADMIN)
def admin_orders():
    """Full order history + live state, newest first."""
    return {"orders": [store.view(o) for o in store.list_orders()]}


@admin_router.post("/admin/teleop", dependencies=ADMIN)
def admin_teleop(request: Request, linear: float = Form(0.0), angular: float = Form(0.0)):
    request.app.state.backend.teleop(linear, angular)
    return Response(status_code=204)  # rapid-fire; no body


@admin_router.post("/admin/dock", dependencies=ADMIN)
def admin_dock(request: Request, dock_id: str = Form(...)):
    # Manual override: publish /dock_robot directly, bypassing the queue.
    request.app.state.backend.dock(dock_id)
    return {"ok": True, "dock_id": dock_id}


@admin_router.post("/admin/abort", dependencies=ADMIN)
def admin_abort(request: Request):
    request.app.state.backend.abort()
    active = store.active()
    if active is not None:
        store.cancel(active.id)
    cancelled = 0
    while (nxt := store.next_queued()) is not None:
        store.cancel(nxt.id)
        cancelled += 1
    return {"ok": True, "aborted": active.id if active else None, "queued_cancelled": cancelled}


@admin_router.post("/admin/reset_pose", dependencies=ADMIN)
def admin_reset_pose(
    request: Request,
    dock_id: str = Form(""),
    x: float = Form(0.0), y: float = Form(0.0), yaw: float = Form(0.0),
):
    # A registered dock's pose wins over the manual x/y/yaw fields.
    if dock_id and dock_id in DOCKS:
        d = DOCKS[dock_id]
        x, y, yaw = d.get("pose_x", 0.0), d.get("pose_y", 0.0), d.get("yaw", 0.0)
    request.app.state.backend.reset_pose(x, y, yaw)
    return {"ok": True, "pose": {"x": x, "y": y, "yaw": yaw}}


@admin_router.post("/admin/relocalize", dependencies=ADMIN)
def admin_relocalize(request: Request, dock_id: str = Form(...)):
    # Rotate in place until the dock's AprilTag is visible, then reset pose to
    # that dock's surveyed map pose. jetracer_docker owns the search + /initialpose;
    # watch `docking_state` (relocalizing / relocalize_ok / relocalize_failed) for
    # progress. Only docks with a tag configured on the robot side will resolve —
    # unknown dock ids are logged and ignored there rather than erroring here.
    request.app.state.backend.relocalize(dock_id)
    return {"ok": True, "dock_id": dock_id}


@admin_router.post("/admin/slam/export", dependencies=ADMIN)
def admin_export_slam_map(map_name: str = Form("")):
    """Serialize the current SLAM map to a Nav2-compatible .yaml/.pgm pair."""
    name = (map_name or "").strip()
    if not name:
        name = dt.datetime.now().strftime("map_%Y%m%d_%H%M%S")
    if not _MAP_NAME_RE.match(name):
        return JSONResponse(
            {"ok": False, "error": "map_name must be 1-64 chars: letters, digits, _ or -"},
            status_code=422,
        )

    try:
        NAV_MAP_DIR.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        return JSONResponse(
            {"ok": False, "error": f"cannot create map directory {NAV_MAP_DIR}: {exc}"},
            status_code=500,
        )

    out_prefix = NAV_MAP_DIR / name
    cmd = ["ros2", "run", "slam_toolbox", "serialize_map", "-f", str(out_prefix)]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=_SLAM_EXPORT_TIMEOUT_S,
        )
    except FileNotFoundError:
        return JSONResponse(
            {"ok": False, "error": "ros2 CLI not found in this runtime"},
            status_code=503,
        )
    except subprocess.TimeoutExpired:
        return JSONResponse(
            {"ok": False, "error": "serialize_map timed out; verify SLAM is running"},
            status_code=504,
        )

    yaml_path = out_prefix.with_suffix(".yaml")
    pgm_path = out_prefix.with_suffix(".pgm")
    if proc.returncode != 0 or not yaml_path.exists() or not pgm_path.exists():
        detail = (proc.stderr or proc.stdout or "serialize_map failed").strip().splitlines()
        msg = detail[-1] if detail else "serialize_map failed"
        return JSONResponse(
            {"ok": False, "error": f"SLAM export failed: {msg}"},
            status_code=500,
        )

    # Try to hot-swap Nav2's active map without restarting the stack.
    map_loaded = False
    load_msg = ""
    load_cmd = [
        "ros2", "service", "call", _LOAD_MAP_SERVICE, _LOAD_MAP_TYPE,
        "{map_url: '" + str(yaml_path) + "'}",
    ]
    try:
        lp = subprocess.run(
            load_cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=_LOAD_MAP_TIMEOUT_S,
        )
        out = (lp.stdout or "") + "\n" + (lp.stderr or "")
        # nav2 LoadMap uses result code 0 on success.
        map_loaded = (lp.returncode == 0) and ("result: 0" in out)
        if map_loaded:
            load_msg = "map_server switched to exported map"
        else:
            lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
            load_msg = lines[-1] if lines else "map exported, but map_server load_map did not succeed"
    except subprocess.TimeoutExpired:
        load_msg = "map exported, but timed out waiting for map_server/load_map"

    return {
        "ok": True,
        "map_name": name,
        "map_yaml": str(yaml_path),
        "map_pgm": str(pgm_path),
        "map_loaded": map_loaded,
        "load_message": load_msg,
    }


@admin_router.post("/admin/orders/{order_id}/cancel", dependencies=ADMIN)
def admin_cancel(request: Request, order_id: int):
    active = store.active()
    if active is not None and active.id == order_id:
        request.app.state.backend.abort()
    order = store.cancel(order_id)
    return {"ok": order is not None, "order_id": order_id}


@admin_router.post("/admin/orders/{order_id}/requeue", dependencies=ADMIN)
def admin_requeue(order_id: int):
    order = store.requeue(order_id)
    return {"ok": order is not None, "order_id": order_id}


app.include_router(router)
app.include_router(admin_router)


@app.get("/healthz")
def healthz():
    return {"ok": True}


def main():
    """console_scripts entry point: `ros2 run robot_web_bridge server`."""
    import uvicorn

    host = os.environ.get("ROBOT_WEB_BRIDGE_HOST", "0.0.0.0")
    port = int(os.environ.get("ROBOT_WEB_BRIDGE_PORT", "8088"))
    # workers=1 on purpose: one process owns the single robot / order queue.
    uvicorn.run(app, host=host, port=port, workers=1)


if __name__ == "__main__":
    main()
