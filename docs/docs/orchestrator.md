# Orchestrator — `robot_web_bridge`

HTTP API + mobile web UI for commanding the JetRacer. Built with **FastAPI +
HTMX + Tailwind** (both loaded from CDNs, so there is no front-end build step —
ideal for serving behind a tunnel and scanning from a QR code at each table).

The package implements the **public user pages**, the **ROS bridge +
dispatcher**, and the **admin API** (PIN-gated operator endpoints). The admin
*frontend* and the SQLite order log are still to come.

!!! tip "Route reference lives in the API Book"
    Every route, its parameters and responses are **generated from this app's
    FastAPI definitions** — see **[HTTP API](api/http.md)**. That reference cannot
    go stale; a hand-maintained table here would (and did).

    This page covers the **architecture** — how the bridge, dispatcher and
    backends fit together.

## Architecture

Three pieces, each with one job:

| Module | Role |
|---|---|
| `app.py` | FastAPI routes — user pages, JSON API, admin endpoints |
| `ros_node.py` | A single rclpy node on a daemon thread — the ROS seam |
| `dispatcher.py` | The **single async loop that owns the robot** — one order at a time |
| `store.py` | In-memory order book (stand-in for the SQLite log) |
| `auth.py` | Operator PIN gate — signed, expiring session cookie |

### The ROS seam

`ros_node.py`:

- **publishes** `/dock_robot` (String), `/abort_docking` (Bool), `/cmd_vel`
  (Twist), `/initialpose` (PoseWithCovarianceStamped), plus latched
  `/virtual_obstacles` and `/dock_registry` (String, JSON) and
  `/relocalize_at_dock` (String);
- **subscribes** `/docking_state` (String), `/odometry/filtered` and
  `/chassis/odom` (Odometry), caching the latest value thread-safely.

Full topic contract in the [API Book](api/ros-jetracer.md).

### The dispatcher

`dispatcher.py` promotes the oldest queued order, publishes `/dock_robot`, then
maps the live `/docking_state` onto the user-facing status
(preparing → on_the_way → delivered / failed). `store.py` is the source of
"robot busy / N orders ahead / ~ETA".

## No ROS? It still runs

If `rclpy` can't be imported (a plain venv for local dev, or the tunnel demo),
the app transparently falls back to a **simulated backend** that progresses
orders on a timer.

```bash
curl -s localhost:8088/state | jq .mode   # "ros" | "simulation"
```

!!! warning "The fallback is silent"
    Missing `rclpy` does **not** raise — the app keeps serving against nothing.
    Assert on `mode` in any integration test, or a green test run may prove
    nothing about the robot.

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
# or: orchestrator/run_web_bridge.sh
```

Plain Python (no ROS needed for the user pages):

```bash
pip install fastapi "uvicorn[standard]" jinja2 pyyaml python-multipart
cd orchestrator/src/robot_web_bridge
uvicorn robot_web_bridge.app:app --reload --port 8088
```

Then open <http://localhost:8088/?dock=dock0>.

## Config

| Setting | Default | Purpose |
|---|---|---|
| `config/docks.yaml` | — | Dock registry (`dock_id → label, pose`) |
| `ROBOT_WEB_BRIDGE_CONFIG` | package `config/` | Override the config directory |
| `ROBOT_WEB_BRIDGE_HOST` / `_PORT` | `0.0.0.0:8088` | Bind address |
| `ROBOT_WEB_BRIDGE_ADMIN_PIN` | `1234` | Operator PIN — **change it** |
| `ROBOT_WEB_BRIDGE_SECRET` | random per process | Cookie signing secret |
| `ROBOT_WEB_BRIDGE_ADMIN_TTL` | `28800` | Session lifetime, seconds |

!!! danger "Two defaults to change before a live event"
    The PIN defaults to `1234`, and the signing secret is **regenerated on every
    process start** — so any restart silently logs every operator out. Set both
    explicitly.

## See also

- [HTTP API](api/http.md) — the generated route reference
- [JetRacer ROS interfaces](api/ros-jetracer.md) — the topic contract
- [Pick and Deliver](solution_pick_and_deliver.md) — where this fits in the system
