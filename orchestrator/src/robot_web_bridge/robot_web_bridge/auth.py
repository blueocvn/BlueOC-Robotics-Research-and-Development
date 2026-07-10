"""Operator PIN gate for the /admin portal.

A deliberately small, dependency-free auth: a shared operator **PIN** (v1, per the
plan — not per-operator accounts) unlocks a signed, expiring session cookie. The
cookie is `"<expiry>.<hmac-sha256>"`, signed with a server secret, so it can't be
forged client-side. This is the abuse control for the genuinely-dangerous
capabilities (teleop, pose reset, manual dock).

Configure via env:
  ROBOT_WEB_BRIDGE_ADMIN_PIN   operator PIN            (default "1234" — CHANGE IT)
  ROBOT_WEB_BRIDGE_SECRET      cookie signing secret   (default: random per process)
  ROBOT_WEB_BRIDGE_ADMIN_TTL   session seconds         (default 28800 = 8h)
"""

from __future__ import annotations

import hashlib
import hmac
import os
import time

from fastapi import HTTPException, Request

COOKIE = "boc_admin"
PIN = os.environ.get("ROBOT_WEB_BRIDGE_ADMIN_PIN", "1234")
TTL = int(os.environ.get("ROBOT_WEB_BRIDGE_ADMIN_TTL", "28800"))
# A random per-process secret means a restart logs everyone out — fine for a POC.
# Set ROBOT_WEB_BRIDGE_SECRET to keep sessions stable across restarts.
_SECRET = os.environ.get("ROBOT_WEB_BRIDGE_SECRET", os.urandom(32).hex()).encode()


def _sign(payload: str) -> str:
    return hmac.new(_SECRET, payload.encode(), hashlib.sha256).hexdigest()


def make_token() -> str:
    exp = str(int(time.time()) + TTL)
    return f"{exp}.{_sign(exp)}"


def valid_token(token: str | None) -> bool:
    if not token or "." not in token:
        return False
    exp, sig = token.rsplit(".", 1)
    if not hmac.compare_digest(sig, _sign(exp)):
        return False
    try:
        return int(exp) > time.time()
    except ValueError:
        return False


def check_pin(pin: str) -> bool:
    return hmac.compare_digest(pin or "", PIN)


def is_admin(request: Request) -> bool:
    return valid_token(request.cookies.get(COOKIE))


def require_admin(request: Request) -> None:
    """FastAPI dependency — 401 unless a valid operator cookie is present."""
    if not is_admin(request):
        raise HTTPException(status_code=401, detail="operator login required")
