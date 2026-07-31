#!/usr/bin/env python3
"""Dump robot_web_bridge's OpenAPI spec into the docs tree.

The HTTP reference in the API book is rendered from this file, so it can never
drift from the code the way a hand-written route table does (the package README
did exactly that: it documented ``/api/admin/*`` long after the router moved to
``/v1/admin/*``).

``robot_web_bridge.app`` falls back to a simulated backend when ``rclpy`` is
missing, so this runs in a plain venv -- no ROS, no sourced workspace. That is
what lets CI regenerate the spec on every docs build.

Usage:
    python docs/scripts/gen_openapi.py [--check]

    --check  exit non-zero if the committed spec is stale instead of writing it.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BRIDGE_PKG = REPO_ROOT / "orchestrator" / "src" / "robot_web_bridge"
OUT_PATH = REPO_ROOT / "docs" / "docs" / "api" / "openapi.json"


def build_spec() -> dict:
    """Import the FastAPI app and ask it for its OpenAPI document."""
    sys.path.insert(0, str(BRIDGE_PKG))
    try:
        from robot_web_bridge.app import app
    except ImportError as exc:  # pragma: no cover - surfaced to the caller
        raise SystemExit(
            f"cannot import robot_web_bridge.app: {exc}\n"
            "install the bridge's doc-time deps first:\n"
            "  pip install fastapi jinja2 pyyaml python-multipart"
        ) from exc

    spec = app.openapi()

    # FastAPI defaults the description to the app docstring only when one is
    # passed; give the book a stable blurb regardless of app-side changes.
    spec.setdefault("info", {}).setdefault(
        "description",
        "HTTP API for commanding the JetRacer. Generated from the FastAPI app -- "
        "do not edit by hand.",
    )
    return spec


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if the committed spec differs from the generated one",
    )
    args = parser.parse_args()

    spec = build_spec()
    rendered = json.dumps(spec, indent=2, sort_keys=True) + "\n"

    if args.check:
        if not OUT_PATH.exists():
            print(f"MISSING: {OUT_PATH.relative_to(REPO_ROOT)}", file=sys.stderr)
            return 1
        if OUT_PATH.read_text() != rendered:
            print(
                f"STALE: {OUT_PATH.relative_to(REPO_ROOT)} differs from the app.\n"
                "regenerate with: python docs/scripts/gen_openapi.py",
                file=sys.stderr,
            )
            return 1
        print(f"up to date: {OUT_PATH.relative_to(REPO_ROOT)}")
        return 0

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(rendered)
    print(f"wrote {OUT_PATH.relative_to(REPO_ROOT)} ({len(spec['paths'])} paths)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
