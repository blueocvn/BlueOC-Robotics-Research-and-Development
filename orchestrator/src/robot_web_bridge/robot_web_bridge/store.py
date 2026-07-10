"""Order book for the user pages.

A thread-safe, single-robot order record. Orders are created ``queued``; the
:class:`~robot_web_bridge.dispatcher.Dispatcher` is the only writer of further
status transitions (``preparing`` → ``on_the_way`` → ``delivered`` / ``failed``),
so this module just *stores* state and renders the per-order view the templates
need. It's the in-memory stand-in for the SQLite log in the plan; the public API
(``place`` / ``cancel`` / ``get`` / ``set_status`` / ``view``) is what ``db.py``
will expose too.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from itertools import count
from typing import Literal, Optional

OrderType = Literal["water", "refill"]
Status = Literal["queued", "preparing", "on_the_way", "delivered", "failed", "cancelled"]

TERMINAL: set[str] = {"delivered", "failed", "cancelled"}
ACTIVE: set[str] = {"preparing", "on_the_way"}

# User-facing stepper (the friendly view of the robot's docking lifecycle).
STEPS: list[tuple[str, str]] = [
    ("queued", "Queued"),
    ("preparing", "Preparing"),
    ("on_the_way", "On the way"),
    ("delivered", "Delivered"),
]
_STEP_INDEX = {key: i for i, (key, _) in enumerate(STEPS)}


@dataclass
class Order:
    id: int
    dock_id: str
    type: OrderType
    status: Status = "queued"
    created_at: float = field(default_factory=time.monotonic)
    dispatched_at: Optional[float] = None
    finished_at: Optional[float] = None


class Store:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._orders: dict[int, Order] = {}
        self._ids = count(1)

    # ── mutations ────────────────────────────────────────────────────────────
    def place(self, dock_id: str, type: OrderType) -> Order:
        with self._lock:
            order = Order(id=next(self._ids), dock_id=dock_id, type=type)
            self._orders[order.id] = order
            return order

    def set_status(self, order_id: int, status: Status) -> Optional[Order]:
        with self._lock:
            order = self._orders.get(order_id)
            if order is None:
                return None
            if order.status in TERMINAL:  # never resurrect a finished order
                return order
            order.status = status
            if status != "queued" and order.dispatched_at is None:
                order.dispatched_at = time.monotonic()
            if status in TERMINAL:
                order.finished_at = time.monotonic()
            return order

    def cancel(self, order_id: int) -> Optional[Order]:
        with self._lock:
            order = self._orders.get(order_id)
            if order is not None and order.status not in TERMINAL:
                order.status = "cancelled"
                order.finished_at = time.monotonic()
            return order

    def requeue(self, order_id: int) -> Optional[Order]:
        """Operator action: send an order back to the queue from any state."""
        with self._lock:
            order = self._orders.get(order_id)
            if order is not None:
                order.status = "queued"
                order.dispatched_at = None
                order.finished_at = None
            return order

    # ── queries ───────────────────────────────────────────────────────────────
    def get(self, order_id: int) -> Optional[Order]:
        with self._lock:
            return self._orders.get(order_id)

    def active(self) -> Optional[Order]:
        """The order that currently holds the robot, if any."""
        with self._lock:
            for o in sorted(self._orders.values(), key=lambda o: o.created_at):
                if o.status in ACTIVE:
                    return o
            return None

    def next_queued(self) -> Optional[Order]:
        with self._lock:
            waiting = [o for o in self._orders.values() if o.status == "queued"]
            return min(waiting, key=lambda o: o.created_at) if waiting else None

    def list_orders(self) -> list[Order]:
        """All orders, newest first (admin history view)."""
        with self._lock:
            return sorted(self._orders.values(), key=lambda o: o.created_at, reverse=True)

    def robot_busy(self) -> bool:
        return self.active() is not None

    # ── view for templates ─────────────────────────────────────────────────────
    def view(self, order: Order) -> dict:
        with self._lock:
            status = order.status
            ahead = self._ahead_locked(order) if status == "queued" else 0
            step_index = _STEP_INDEX.get(status, 2 if status == "failed" else -1)
            return {
                "id": order.id,
                "dock_id": order.dock_id,
                "type": order.type,
                "status": status,
                "step_index": step_index,
                "ahead": ahead,
                "eta": self._eta(status, ahead),
                "delivered": status == "delivered",
                "failed": status == "failed",
                "cancelled": status == "cancelled",
                "busy": status == "queued",
            }

    # ── helpers (lock held) ─────────────────────────────────────────────────────
    def _ahead_locked(self, order: Order) -> int:
        """Non-terminal orders created before this one (the active one + earlier queued)."""
        return sum(
            1
            for o in self._orders.values()
            if o.id != order.id
            and o.status not in TERMINAL
            and o.created_at < order.created_at
        )

    @staticmethod
    def _eta(status: str, ahead: int) -> Optional[str]:
        if status in TERMINAL:
            return None
        if status == "queued":
            return f"~{max(1, ahead + 1)} min"
        if status == "preparing":
            return "~2 min"
        if status == "on_the_way":
            return "~1 min"
        return None
