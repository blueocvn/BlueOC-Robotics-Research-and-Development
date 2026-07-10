# robot_web_bridge

HTTP API + mobile web UI for commanding the JetRacer. Built with **FastAPI +
HTMX + Tailwind** (both loaded from CDNs, so there is no front-end build step —
ideal for serving behind ngrok and scanning from a QR code at each table).

This package implements the **public user pages**, the **ROS bridge +
dispatcher**, and the **admin API** (PIN-gated operator endpoints) per
[`requirements-claude-plan.md`](../../../requirements-claude-plan.md). The admin
*frontend* and the SQLite order log are still to come.

## Admin API (backend only — UI pending)

PIN-gated operator endpoints. Log in once to get a signed session cookie, then
call the actuation routes. Configure the PIN via `ROBOT_WEB_BRIDGE_ADMIN_PIN`
(see `auth.py`).

| Route | Body | Effect |
|-------|------|--------|
| `POST /api/admin/login` | `pin` | set operator cookie (401 on bad PIN) |
| `POST /api/admin/logout` | — | clear cookie |
| `GET  /api/admin/session` | — | `{authenticated: bool}` |
| `GET  /api/admin/orders` | — | full order history + live state |
| `POST /api/admin/teleop` | `linear, angular` | publish `/cmd_vel` (204) |
| `POST /api/admin/dock` | `dock_id` | publish `/dock_robot` directly (queue bypass) |
| `POST /api/admin/abort` | — | publish `/abort_docking` + clear queue |
| `POST /api/admin/reset_pose` | `dock_id` \| `x,y,yaw` | publish `/initialpose` |
| `POST /api/admin/orders/{id}/cancel` | — | cancel (aborts robot if active) |
| `POST /api/admin/orders/{id}/requeue` | — | send order back to the queue |

All except login/logout/session require the cookie (`401` otherwise).

## User pages + API

| Route | What it does |
|-------|--------------|
| `GET /?dock=<id>` | Landing screen the QR opens. Shows robot status + **Get Water** / **Refill**. |
| `POST /orders` | Place an order → task-tracking screen. |
| `GET /orders/{id}` | Task screen (also the HTMX poll target while the order is live). |
| `POST /orders/{id}/cancel` | Cancel a queued / in-progress order (aborts the robot if it's the active one). |
| `POST /api/abort` | Emergency stop: abort the active order + clear the queue. |
| `GET /api/docks` | Dock registry (JSON), seeded from `config/docks.yaml`. |
| `GET /api/state` | Live snapshot: backend mode, latest `docking_state` + odom pose. |
| `GET /healthz` | Liveness probe. |

Reactivity is HTMX-only: the task screen re-fetches itself every 2 s and swaps
`#content`, so the stepper, ETA, "robot is busy" notice and buttons stay live
without any custom JavaScript.

## The bridge

`ros_node.py` runs a single rclpy node on a daemon thread:

- **publishes** `/dock_robot` (String), `/abort_docking` (Bool), `/cmd_vel`
  (Twist), `/initialpose` (PoseWithCovarianceStamped — for the admin pose reset);
- **subscribes** `/docking_state` (String) + `/chassis/odom` (Odometry), caching
  the latest value thread-safely.

`dispatcher.py` is the **single async loop that owns the robot** — one order at a
time. It promotes the oldest queued order, publishes `/dock_robot`, then maps the
live `/docking_state` onto the user-facing status (preparing → on_the_way →
delivered / failed). `store.py` is the in-memory order book (stand-in for the
SQLite log) — the source of "robot busy / N orders ahead / ~ETA".

**No ROS? It still runs.** If `rclpy` can't be imported (e.g. a plain venv for
local dev / the tunnel demo), the app transparently falls back to a **simulated
backend** that progresses orders on a timer. `GET /api/state` tells you which
mode you're in (`"mode": "ros"` vs `"simulation"`).

### Matching the real robot's `/docking_state`

The exact state strings are robot-defined. Confirm them and adjust if needed:

```bash
ros2 topic info /docking_state -v     # confirm the message type
ros2 topic echo /docking_state        # capture the actual strings
```

Then override the mappings via env vars (comma-separated, case-insensitive):

```bash
export ROBOT_WEB_BRIDGE_INPROGRESS_STATES="docking,navigating"
export ROBOT_WEB_BRIDGE_SUCCESS_STATES="docked,arrived"
export ROBOT_WEB_BRIDGE_ERROR_STATES="failed,aborted"
export ROBOT_WEB_BRIDGE_ORDER_TIMEOUT=180   # mark a stuck order failed after Ns
```

## Run it

Inside the container (after `colcon build --packages-select robot_web_bridge`):

```bash
ros2 run robot_web_bridge server          # http://localhost:8088
# or: humble_ws/run_web_bridge.sh
```

Plain Python (no ROS needed for the user pages):

```bash
pip install fastapi "uvicorn[standard]" jinja2 pyyaml
cd humble_ws/src/robot_web_bridge
uvicorn robot_web_bridge.app:app --reload --port 8088
```

Then open <http://localhost:8088/?dock=dock0>.

### Config

- `config/docks.yaml` — dock registry (`dock_id → label, pose`). The user pages
  only need `id` + `label`; pose fields are for the later dispatcher / admin
  pose-reset. Override the directory with `ROBOT_WEB_BRIDGE_CONFIG`.
- `ROBOT_WEB_BRIDGE_HOST` / `ROBOT_WEB_BRIDGE_PORT` — bind address (default
  `0.0.0.0:8088`).
