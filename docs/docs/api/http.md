# HTTP API

The `robot_web_bridge` service exposes the JetRacer over HTTP. It's the fastest
way to get a working demo: **no ROS install needed on your machine**, and it runs
against a simulator when no robot is present.

Default bind address is `0.0.0.0:8088` — override with `ROBOT_WEB_BRIDGE_HOST`
and `ROBOT_WEB_BRIDGE_PORT`.

```bash
# with ROS, inside the container
ros2 run robot_web_bridge server

# without ROS — user pages + simulated backend
pip install fastapi "uvicorn[standard]" jinja2 pyyaml python-multipart
uvicorn robot_web_bridge.app:app --reload --port 8088
```

## Are you talking to a real robot?

`GET /state` reports the backend mode. Check it before you trust anything.

```bash
curl -s localhost:8088/state | jq .mode
# "ros"          -> commands reach the real robot
# "simulation"   -> orders advance on a timer, nothing moves
```

!!! warning "Simulation mode is silent"

    The bridge does **not** error when `rclpy` is missing — it falls back to the
    simulated backend and keeps serving. A demo that "works" on your laptop may
    be talking to nothing. Always assert on `mode` in integration tests.

## Quick start

Place an order and watch it progress:

```bash
# 1. what tables exist?
curl -s localhost:8088/docks | jq

# 2. place an order for dock0
curl -s -X POST localhost:8088/orders \
     -d 'dock=dock0' -d 'kind=water'

# 3. poll live state
curl -s localhost:8088/state | jq

# 4. emergency stop — aborts the active order and clears the queue
curl -s -X POST localhost:8088/abort
```

## Admin authentication

Operator routes under `/v1/admin/` are PIN-gated. These are the genuinely
dangerous capabilities — direct teleop, pose reset, manual docking — so they sit
behind a signed session cookie.

```bash
# log in once; keep the cookie jar
curl -s -c jar.txt -X POST localhost:8088/v1/admin/login -d 'pin=1234'

# then call actuation routes with it
curl -s -b jar.txt -X POST localhost:8088/v1/admin/teleop \
     -d 'linear=0.2' -d 'angular=0.0'
```

Everything except `login`, `logout` and `session` returns **401** without a valid
cookie.

| Env var | Default | Meaning |
|---|---|---|
| `ROBOT_WEB_BRIDGE_ADMIN_PIN` | `1234` | Operator PIN — **change it** |
| `ROBOT_WEB_BRIDGE_SECRET` | random per process | Cookie signing secret |
| `ROBOT_WEB_BRIDGE_ADMIN_TTL` | `28800` (8 h) | Session lifetime in seconds |

!!! danger "Two defaults that will bite you at an event"

    The PIN defaults to `1234`, and the signing secret defaults to a **random
    value regenerated on every process start** — so any restart silently logs
    every operator out. Set both explicitly before the hackathon floor opens.

## Route reference

Generated directly from the FastAPI app — see
[`docs/scripts/gen_openapi.py`](https://github.com/BlueOC-Robotics/Research-and-Development/blob/main/docs/scripts/gen_openapi.py).

[OAD(./openapi.json)]
