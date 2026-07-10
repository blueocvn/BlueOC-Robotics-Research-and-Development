"""FastAPI app — Robot Web Bridge (user pages + ROS bridge).

Serves the public, QR-scanned user surface for the JetRacer:

    GET  /                       landing page (reads ?dock=<id> from the QR deep-link)
    POST /orders                 place a water/refill order -> task screen
    GET  /orders/{id}            task screen (also the HTMX poll target while live)
    POST /orders/{id}/cancel     cancel a still-queued/in-progress order
    POST /api/abort              emergency: abort the active order + cancel the queue
    GET  /api/docks              dock registry (JSON)
    GET  /api/state              live snapshot (backend mode, docking_state, odom)

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
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path

import yaml
from fastapi import Depends, FastAPI, Form, Request, Response
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from . import auth
from .dispatcher import HOME_DOCK, Dispatcher, RosBackend, SimBackend
from .ros_node import shutdown_ros_node, start_ros_node
from .store import STEPS, Store

log = logging.getLogger("robot_web_bridge")

BASE_DIR = Path(__file__).resolve().parent
TEMPLATES_DIR = BASE_DIR / "templates"
STATIC_DIR = BASE_DIR / "static"
CONFIG_DIR = Path(os.environ.get("ROBOT_WEB_BRIDGE_CONFIG", BASE_DIR.parent / "config"))

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
# A QR-less visitor is a *table*, never the home/water station.
DEFAULT_DOCK = next((d for d in DOCKS if d != HOME_DOCK), next(iter(DOCKS), "dock1"))


def resolve_dock(dock_id: str | None) -> tuple[str, str]:
    """Return (dock_id, label), falling back to the first registered dock."""
    if dock_id and dock_id in DOCKS:
        return dock_id, DOCKS[dock_id].get("label", dock_id)
    if dock_id:  # unknown id from a stray QR — keep it, label = the raw id
        return dock_id, dock_id
    return DEFAULT_DOCK, DOCKS.get(DEFAULT_DOCK, {}).get("label", DEFAULT_DOCK)


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
@app.get("/", response_class=HTMLResponse)
def landing(request: Request, dock: str | None = None):
    dock_id, label = resolve_dock(dock)
    return render(
        request, "landing.html",
        dock_id=dock_id, dock_label=label,
        robot_busy=request.app.state.dispatcher.robot_busy(),
    )


@app.post("/orders", response_class=HTMLResponse)
def place_order(request: Request, type: str = Form("water"), dock: str = Form(...)):
    dock_id, label = resolve_dock(dock)
    order_type = type if type in ("water", "refill") else "water"
    order = store.place(dock_id, order_type)
    view = store.view(order)
    return render(request, "task.html", dock_label=label, order=view)


@app.get("/orders/{order_id}", response_class=HTMLResponse)
def order_screen(request: Request, order_id: int):
    order = store.get(order_id)
    if order is None:
        return RedirectResponse("/", status_code=303)
    _, label = resolve_dock(order.dock_id)
    return render(request, "task.html", dock_label=label, order=store.view(order))


@app.post("/orders/{order_id}/cancel", response_class=HTMLResponse)
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


@app.post("/api/abort")
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


@app.get("/api/docks")
def api_docks():
    return JSONResponse({"docks": DOCKS})


@app.get("/api/state")
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


# ── admin API (PIN-gated, JSON only — frontend comes later) ───────────────────
# Backend for the operator portal. No admin templates yet; these return JSON so
# the /admin UI can be wired to them when it's built. All actuation routes are
# gated by the operator session cookie (auth.require_admin).
ADMIN = [Depends(auth.require_admin)]


@app.post("/api/admin/login")
def admin_login(pin: str = Form("")):
    if not auth.check_pin(pin):
        return JSONResponse({"ok": False, "error": "bad pin"}, status_code=401)
    resp = JSONResponse({"ok": True})
    resp.set_cookie(auth.COOKIE, auth.make_token(), httponly=True,
                    samesite="lax", max_age=auth.TTL)
    return resp


@app.post("/api/admin/logout")
def admin_logout():
    resp = JSONResponse({"ok": True})
    resp.delete_cookie(auth.COOKIE)
    return resp


@app.get("/api/admin/session")
def admin_session(request: Request):
    """Cheap check the frontend can use to know if it's still logged in."""
    return {"authenticated": auth.is_admin(request)}


@app.get("/api/admin/orders", dependencies=ADMIN)
def admin_orders():
    """Full order history + live state, newest first."""
    return {"orders": [store.view(o) for o in store.list_orders()]}


@app.post("/api/admin/teleop", dependencies=ADMIN)
def admin_teleop(request: Request, linear: float = Form(0.0), angular: float = Form(0.0)):
    request.app.state.backend.teleop(linear, angular)
    return Response(status_code=204)  # rapid-fire; no body


@app.post("/api/admin/dock", dependencies=ADMIN)
def admin_dock(request: Request, dock_id: str = Form(...)):
    # Manual override: publish /dock_robot directly, bypassing the queue.
    request.app.state.backend.dock(dock_id)
    return {"ok": True, "dock_id": dock_id}


@app.post("/api/admin/abort", dependencies=ADMIN)
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


@app.post("/api/admin/reset_pose", dependencies=ADMIN)
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


@app.post("/api/admin/orders/{order_id}/cancel", dependencies=ADMIN)
def admin_cancel(request: Request, order_id: int):
    active = store.active()
    if active is not None and active.id == order_id:
        request.app.state.backend.abort()
    order = store.cancel(order_id)
    return {"ok": order is not None, "order_id": order_id}


@app.post("/api/admin/orders/{order_id}/requeue", dependencies=ADMIN)
def admin_requeue(order_id: int):
    order = store.requeue(order_id)
    return {"ok": order is not None, "order_id": order_id}


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
