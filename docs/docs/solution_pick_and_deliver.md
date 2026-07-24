# Robotic Solution — Pick and Deliver (RA + AMR)

The full fulfillment loop combines both robots and a web orchestrator: a person
orders water from a phone, the **arm** fills a cup and places it on a tray, and
the **AMR** carries it from the dispenser to the person.

```
   Phone (QR order)                        RA (SO-ARM 101)
        │                                       │
        ▼                                       ▼
  ┌─────────────┐   /dock_robot        grasp → fill → place on tray
  │ Orchestrator │ ───────────────►                │
  │ (web bridge) │ ◄─── /docking_state             ▼
  └─────────────┘                        AMR (JetRacer)
        ▲                                dock → carry → deliver → return
        └──────────────  /chassis/odom, /initialpose  ──────────────┘
```

## The three parts

| Part | Role | Docs |
|------|------|------|
| **RA — SO-ARM 101** | Grasps the mug, fills at the dispenser, places it on the tray | [Robot Arm](ra_concepts.md) |
| **AMR — JetRacer** | Navigates dock-to-dock, carries the tray to the person | [JetRacer](amr_concepts.md) |
| **Orchestrator** | FastAPI + HTMX web UI + dispatcher that sequences the AMR | [Orchestrator](orchestrator.md) |

The workspaces run on **different ROS distros** (RA on native Jazzy, AMR on
Humble in `Dockerfile.dev`) but interoperate over DDS — keep the **same
`ROS_DOMAIN_ID`**.

## The orchestrator seam

The web orchestrator (`orchestrator_ws/robot_web_bridge`) owns a single
dispatcher that drives the AMR dock-to-dock. Its ROS contract:

- **Publishes** `/dock_robot`, `/abort_docking`, `/cmd_vel`, `/initialpose`
- **Subscribes** `/docking_state`, `/chassis/odom`

## Integration status

The orchestrator ↔ AMR seam is wired on both ends. The remaining open work is the
**RA (arm) ↔ AMR handoff** — the arm has no orchestrator integration yet.

| Contract | State |
|----------|-------|
| Orchestrator publishes `/dock_robot`, reads `/docking_state`, `/chassis/odom`, seeds `/initialpose` | ✅ Built (orchestrator side) |
| A `/dock_robot` **consumer** on the AMR (dock id → Nav2 goal / docking behavior) | ✅ Implemented — `jetracer_bringup/scripts/jetracer_docker.py` (subscribes `/dock_robot`, drives Nav2 + the `opennav_docking` action server) |
| A real `/docking_state` **producer** on the AMR | ✅ Implemented — `jetracer_docker.py` publishes real phase strings (incl. `relocalize_ok`/`relocalize_failed`) on `/docking_state` |
| RA ↔ AMR handoff (tray ready → AMR depart) | ❌ Not wired — the arm is not part of the orchestrator loop |

Docking runs through **`opennav_docking`** (a Nav2 docking action server) triggered
by the `/dock_robot` topic; `jetracer_docker.py` also handles `/undock_robot` and
`/abort_docking`. The open item is connecting the **arm**: signalling "tray ready"
so the orchestrator dispatches the AMR after the arm finishes a cup.

## Running the pieces today

Until the arm is wired into the loop, run the layers independently:

1. **Arm refill** — [RA Pick and Place](ra_pick_and_place.md)
2. **AMR navigation** — [AMR Navigate & Deliver](amr_pick_and_place.md)
3. **Orchestrator web UI** — see [Orchestrator](orchestrator.md); it can run in
   `SimBackend` mode (no ROS, each leg completes on a timer) for the ordering
   flow demo.
