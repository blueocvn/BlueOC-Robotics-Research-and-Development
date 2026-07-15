"""The single async dispatcher that owns the robot, plus its two backends.

One robot, one order at a time. The dispatcher walks each order through a
multi-leg **docking cycle** and is the only thing that promotes a queued order,
advances its status, and decides when the robot is free again — a single source
of truth for who has the robot.

Docking cycle (``HOME_DOCK`` = ``dock0`` is the water station / home base):

- **water**  → dock0 (load water) → table (deliver)
- **refill** → table (collect empty) → dock0 (refill) → table (return)

The robot carries one order at a time. A cycle ends at *delivery* — there is no
mandatory trip home baked into it. When the order finishes the robot chains
**straight into the next queued order** from wherever it is (so a water delivery
can roll directly on to collect someone's refill); it only drives back to dock0
to park when the queue is empty. That "returning home" trip happens *after* the
user already sees ``delivered``, so robot-busy state is tracked here
(``robot_busy`` / ``robot_snapshot``), not derived from the order book.

Backends:
- :class:`SimBackend`  — no ROS; each leg completes on a timer. Local dev / demo.
- :class:`RosBackend`  — publishes ``/dock_robot`` per leg and reads the live
  ``/docking_state`` string to tell when a leg has arrived / failed.

The exact ``/docking_state`` strings are robot-defined — confirm with
``ros2 topic echo /docking_state`` and adjust the *_STATES sets below (or override
via the ROBOT_WEB_BRIDGE_* env vars).
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Optional, Protocol

from .store import Order, Store

# Home / water station the robot starts and ends every cycle at.
HOME_DOCK = os.environ.get("ROBOT_WEB_BRIDGE_HOME_DOCK", "dock0")


# ── /docking_state mapping (edit to match the real robot) ─────────────────────
def _states(env: str, default: set[str]) -> set[str]:
    raw = os.environ.get(env)
    return {s.strip().lower() for s in raw.split(",")} if raw else default


IN_PROGRESS_STATES = _states(
    "ROBOT_WEB_BRIDGE_INPROGRESS_STATES",
    {"docking", "navigating", "in_progress", "moving", "approaching", "undocking"},
)
SUCCESS_STATES = _states(
    "ROBOT_WEB_BRIDGE_SUCCESS_STATES",
    {"docked", "success", "arrived", "completed", "done", "idle"},
)
ERROR_STATES = _states(
    "ROBOT_WEB_BRIDGE_ERROR_STATES",
    {"failed", "error", "aborted", "cancelled", "lost", "stuck"},
)

# Give up on a stuck leg after this long (s) and fail the order.
ORDER_TIMEOUT_S = float(os.environ.get("ROBOT_WEB_BRIDGE_ORDER_TIMEOUT", "180"))
# Min dwell before a SUCCESS reading counts as "arrived" — lets a stale "docked"
# state from the *previous* leg clear, and covers a no-op leg (already at target).
ROS_MIN_LEG_S = float(os.environ.get("ROBOT_WEB_BRIDGE_MIN_LEG", "2.0"))
# Simulated per-leg travel time for SimBackend.
SIM_LEG_SECONDS = float(os.environ.get("ROBOT_WEB_BRIDGE_SIM_LEG", "3.0"))


# ── docking cycle ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Leg:
    """One navigation leg of a cycle."""

    dock: str       # where the robot drives to
    status: str     # user-facing order status while travelling this leg
    carrying: bool  # whether the robot is holding the item on this leg


def plan_legs(order: Order, home: str = HOME_DOCK) -> list[Leg]:
    """The ordered legs for an order's docking cycle, ending at delivery.

    Arrival of the final leg is what marks the order ``delivered`` — the trip
    home (if any) is decided afterwards by the dispatcher, not part of the plan.
    """
    table = order.dock_id
    if order.type == "refill":
        return [
            Leg(table, "preparing", False),   # go collect the empty
            Leg(home, "preparing", True),     # take it to the water station to fill
            Leg(table, "on_the_way", True),   # bring the refill back → arrival = delivered
        ]
    # water (default)
    return [
        Leg(home, "preparing", False),        # go load water
        Leg(table, "on_the_way", True),       # deliver to the table → arrival = delivered
    ]


def _phase(leg: Leg) -> str:
    """Human-ish name for the robot's current activity (for snapshots/UI)."""
    if leg.dock == HOME_DOCK:
        return "to_station" if not leg.carrying else "to_refill"
    return "delivering" if leg.carrying else "collecting"


def _default_tag_frame(dock_id: str) -> str:
    """dock0 -> dock_0, dock12 -> dock_12; otherwise replace dashes with underscores."""
    if dock_id.startswith("dock") and dock_id[4:].isdigit():
        return f"dock_{dock_id[4:]}"
    return dock_id.replace("-", "_")


def _dock_registry_payload(docks: dict[str, dict]) -> str:
    """Build the full-snapshot /dock_registry JSON payload expected by jetracer_docker."""
    def f_or_none(v):
        if v is None:
            return None
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    def docked_heading(v):
        # The map editor stores a dock's yaw as the direction its staging pose
        # sticks out (away from the wall, into the room). jetracer_docker's
        # dock_pose_yaw is the opposite: the robot's heading when docked, facing
        # into the dock. Reverse by pi (a 180 deg flip, NOT a sign negation) so
        # the staging pose lands on the approach side and the docked heading lines
        # up. None (heading unknown) stays None.
        y = f_or_none(v)
        if y is None:
            return None
        return math.atan2(-math.sin(y), -math.cos(y))

    items: list[dict] = []
    for dock_id, raw in docks.items():
        d = raw if isinstance(raw, dict) else {}
        items.append({
            "id": str(d.get("id") or dock_id),
            "tag_frame": str(d.get("tag_frame") or _default_tag_frame(str(dock_id))),
            "x": float(d.get("pose_x", d.get("x", 0.0))),
            "y": float(d.get("pose_y", d.get("y", 0.0))),
            "yaw": docked_heading(d.get("yaw", None)),
            "staging_dist": f_or_none(d.get("staging_dist", d.get("staging_distance", None))),
        })
    return json.dumps({"docks": items})


# ── backends ──────────────────────────────────────────────────────────────────
class Backend(Protocol):
    """What the dispatcher + admin portal need from a robot backend.

    The dispatcher drives one leg at a time: ``go_to`` a dock, then poll
    ``leg_status`` until it reports ``"arrived"`` or ``"failed"``.
    """

    def go_to(self, dock_id: str) -> None: ...
    def leg_status(self) -> str: ...  # "navigating" | "arrived" | "failed"
    def abort(self) -> None: ...
    def snapshot(self) -> dict: ...
    # operator (admin) passthroughs
    def teleop(self, linear: float, angular: float) -> None: ...
    def reset_pose(self, x: float, y: float, yaw: float) -> None: ...
    def relocalize(self, dock_id: str) -> None: ...
    def dock(self, dock_id: str) -> None: ...
    def set_obstacles(self, obstacles: list[dict]) -> None: ...
    def set_docks(self, docks: dict[str, dict]) -> None: ...


class SimBackend:
    """Fake robot: every leg completes ``SIM_LEG_SECONDS`` after ``go_to``."""

    name = "simulation"

    def __init__(self) -> None:
        self._leg_start: Optional[float] = None
        self._target: Optional[str] = None
        self._obstacles: list[dict] = []
        self._docks: dict[str, dict] = {}

    def go_to(self, dock_id: str) -> None:
        self._target = dock_id
        self._leg_start = time.monotonic()

    def leg_status(self) -> str:
        if self._leg_start is None:
            return "navigating"
        if time.monotonic() - self._leg_start >= SIM_LEG_SECONDS:
            return "arrived"
        return "navigating"

    def abort(self) -> None:
        self._leg_start = None
        self._target = None

    # Admin actuation is a no-op in simulation (nothing to drive).
    def teleop(self, linear: float, angular: float) -> None:
        pass

    def reset_pose(self, x: float, y: float, yaw: float) -> None:
        pass

    def relocalize(self, dock_id: str) -> None:
        pass

    def dock(self, dock_id: str) -> None:
        self.go_to(dock_id)

    def set_obstacles(self, obstacles: list[dict]) -> None:
        self._obstacles = obstacles  # nothing to publish; kept for inspection

    def set_docks(self, docks: dict[str, dict]) -> None:
        self._docks = docks  # nothing to publish; kept for inspection

    def snapshot(self) -> dict:
        return {"connected": False, "mode": self.name, "docking_state": None, "odom": None}


class RosBackend:
    """Real robot via the rclpy bridge node."""

    name = "ros"

    def __init__(self, node) -> None:
        self._node = node
        self._leg_start: Optional[float] = None
        self._target: Optional[str] = None

    def go_to(self, dock_id: str) -> None:
        self._target = dock_id
        self._leg_start = time.monotonic()
        self._node.publish_dock(dock_id)

    def leg_status(self) -> str:
        if self._leg_start is None:
            return "navigating"
        elapsed = time.monotonic() - self._leg_start
        if elapsed > ORDER_TIMEOUT_S:
            return "failed"
        if elapsed < ROS_MIN_LEG_S:
            return "navigating"  # let the previous leg's state reading settle
        state = self._node.get_state().get("docking_state")
        if not state:
            return "navigating"  # commanded but no feedback yet
        s = str(state).strip().lower()
        if s in ERROR_STATES:
            return "failed"
        if s in SUCCESS_STATES:
            return "arrived"
        return "navigating"  # in-progress / unrecognised -> still working

    def abort(self) -> None:
        self._leg_start = None
        self._target = None
        self._node.publish_abort()

    def teleop(self, linear: float, angular: float) -> None:
        self._node.publish_cmd_vel(linear, angular)

    def reset_pose(self, x: float, y: float, yaw: float) -> None:
        self._node.publish_initialpose(x, y, yaw)

    def relocalize(self, dock_id: str) -> None:
        self._node.publish_relocalize(dock_id)

    def dock(self, dock_id: str) -> None:
        self.go_to(dock_id)

    def set_obstacles(self, obstacles: list[dict]) -> None:
        self._node.publish_obstacles(json.dumps({"obstacles": obstacles}))

    def set_docks(self, docks: dict[str, dict]) -> None:
        self._node.publish_docks(_dock_registry_payload(docks))

    def snapshot(self) -> dict:
        st = self._node.get_state()
        return {"connected": True, "mode": self.name, **st}


# ── dispatcher ────────────────────────────────────────────────────────────────
class Dispatcher:
    def __init__(self, store: Store, backend: Backend, tick: float = 1.0) -> None:
        self.store = store
        self.backend = backend
        self.tick = tick
        # Robot state machine: idle (parked at home) | busy (serving) | returning.
        self._state = "idle"
        self._order: Optional[Order] = None
        self._legs: list[Leg] = []
        self._i = 0
        self._at: Optional[str] = HOME_DOCK  # last dock reached; None while in transit

    # ── public robot state ────────────────────────────────────────────────────
    def robot_busy(self) -> bool:
        """True whenever the robot is out — serving an order or driving home to park."""
        return self._state != "idle"

    def robot_snapshot(self) -> dict:
        base = {"phase": self._state, "target_dock": None, "carrying": False,
                "order_id": None, "order_type": None, "at": self._at}
        if self._state == "returning":
            base["target_dock"] = HOME_DOCK
        elif self._state == "busy" and self._order is not None:
            leg = self._legs[self._i]
            base.update(phase=_phase(leg), target_dock=leg.dock, carrying=leg.carrying,
                        order_id=self._order.id, order_type=self._order.type,
                        leg=self._i + 1, legs=len(self._legs))
        return base

    # ── scheduling ────────────────────────────────────────────────────────────
    def step(self) -> None:
        """One tick: advance the active cycle, else serve next / finish parking."""
        if self._state == "busy":
            self._advance()
            return
        # Free (idle or mid-way home): a waiting order beats parking — and the
        # robot rolls into it straight from where it is.
        nxt = self.store.next_queued()
        if nxt is not None:
            self._begin(nxt)
            return
        if self._state == "returning":
            st = self.backend.leg_status()
            if st in ("arrived", "failed"):
                if st == "arrived":
                    self._at = HOME_DOCK
                self._state = "idle"  # parked (or gave up)

    def _begin(self, order: Order) -> None:
        self._order = order
        self._legs = plan_legs(order)
        self._i = 0
        self._state = "busy"
        self._start_leg()

    def _start_leg(self) -> None:
        assert self._order is not None
        leg = self._legs[self._i]
        self.store.set_status(self._order.id, leg.status)
        self._at = None  # in transit
        self.backend.go_to(leg.dock)

    def _advance(self) -> None:
        assert self._order is not None
        # The order may have been cancelled/aborted out from under us.
        current = self.store.get(self._order.id)
        if current is None or current.status == "cancelled":
            self._end_order()
            return

        st = self.backend.leg_status()
        if st == "failed":
            self.store.set_status(self._order.id, "failed")
            self._end_order()
            return
        if st != "arrived":
            return  # still navigating

        self._at = self._legs[self._i].dock
        self._i += 1
        if self._i < len(self._legs):
            self._start_leg()  # next leg of the cycle
        else:
            self.store.set_status(self._order.id, "delivered")
            self._end_order()  # cycle complete

    def _end_order(self) -> None:
        """Order delivered/failed/cancelled: chain to the next one, else park home."""
        self._order = None
        self._legs = []
        self._i = 0
        nxt = self.store.next_queued()
        if nxt is not None:
            self._begin(nxt)              # straight into it from wherever we are
        elif self._at == HOME_DOCK:
            self._state = "idle"          # already parked, nothing to do
        else:
            self._state = "returning"     # drive back to dock0 to wait
            self._at = None
            self.backend.go_to(HOME_DOCK)

    async def run(self) -> None:
        while True:
            try:
                self.step()
            except Exception:  # never let the loop die
                pass
            await asyncio.sleep(self.tick)
