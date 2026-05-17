#!/usr/bin/env python3
"""Capture Isaac Sim viewport from one or more cameras to PNG files.

Reusable harness lifted from the 2026-05-16 wrist-camera + reversed-USD
inspection sessions. Drives the aic-dt MCP socket to:
  1. (Optionally) spawn + start each wrist-camera stream
  2. For each named camera: switch viewport, wait for render, write PNG via
     omni.kit.viewport.utility.capture_viewport_to_file (viewport-only,
     UI-free — unlike X11 window screenshots)
  3. Print probe info (camera world pose, gripper pose) for context

Why this exists separately from x11-window-screenshot:
  - x11 screenshots capture the WHOLE Kit window including UI panels —
    image is mostly chrome, model off-center.
  - capture_viewport_to_file goes straight to the viewport texture buffer
    — clean center-framed renders. Required for any "render the cameras
    AS the cameras" workflow.

Usage:
    # All three wrist cameras at default 1280x720, sim must already be running
    # with aic-dt extension loaded:
    python3 exts/aic-dt/scripts/capture_viewport_set.py \\
        --out-dir ~/Share/my_capture_$(date +%Y%m%d_%H%M%S) \\
        --cameras center_camera left_camera right_camera

    # Single camera + an arbitrary prim path
    python3 exts/aic-dt/scripts/capture_viewport_set.py \\
        --out-dir /tmp/captures \\
        --camera-paths /World/workspace_camera

    # Add --auto-spawn to spawn+start any wrist cameras not already streaming
    # (only applies to canonical wrist cam names — center/left/right).

Output: one PNG per camera + probe.json with pose data.

Pairs with: run_freeze_isaac_sim.sh (which sets up the frozen-robot scene
this captures), build_cable_variant_usds.py (which produces the asset whose
gripper-end visual you're verifying), and the wrist-cameras-restore
commits (which authored the per-camera atoms).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from typing import Dict, Iterable, List


MCP_HOST = "127.0.0.1"
MCP_PORT = 8768

# Canonical wrist-camera names known to the aic-dt extension's
# spawn_wrist_camera / start_wrist_camera_stream MCP atoms (per WRIST_CAMERAS
# dict at extension.py top). Auto-spawn only applies to these names.
WRIST_CAMERA_NAMES = ("center_camera", "left_camera", "right_camera")


def mcp(cmd: str, params=None, timeout: int = 60) -> dict:
    """One-shot MCP request to aic-dt. Returns the decoded JSON response."""
    s = socket.socket()
    s.settimeout(timeout)
    s.connect((MCP_HOST, MCP_PORT))
    s.sendall(json.dumps({"type": cmd, "params": params or {}}).encode())
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk:
            break
        buf += chunk
        try:
            return json.loads(buf.decode())
        except json.JSONDecodeError:
            continue
    raise RuntimeError(f"MCP {cmd}: connection closed without complete JSON")


def auto_spawn_wrist_cameras(names: Iterable[str]) -> None:
    """spawn_wrist_camera + start_wrist_camera_stream for each known name.

    Skips names that aren't in WRIST_CAMERA_NAMES — those would fail the
    MCP atom's lookup. Idempotent — safe to call when streams already exist.
    """
    for name in names:
        if name not in WRIST_CAMERA_NAMES:
            print(f"  [skip auto-spawn] {name!r} not a canonical wrist cam")
            continue
        print(f"  spawning + starting {name}")
        mcp("spawn_wrist_camera", {"name": name})
        mcp("start_wrist_camera_stream", {"name": name})
        time.sleep(1.0)


def resolve_camera_path(name_or_path: str) -> str:
    """Map a wrist cam name to its USD prim path, OR pass an absolute path through."""
    if name_or_path.startswith("/"):
        return name_or_path
    if name_or_path in WRIST_CAMERA_NAMES:
        return f"/World/UR5e/aic_unified_robot/{name_or_path}_optical/{name_or_path}"
    raise ValueError(
        f"Unknown camera spec {name_or_path!r}. Use a wrist cam name "
        f"({', '.join(WRIST_CAMERA_NAMES)}) or an absolute USD prim path."
    )


def switch_viewport_and_capture(camera_prim_path: str, out_path: str,
                                 settle_seconds: float = 4.0,
                                 file_poll_seconds: float = 0.5,
                                 file_poll_attempts: int = 30) -> bool:
    """Point active viewport at `camera_prim_path`, render N seconds, capture.

    Returns True on success, False if the output file didn't appear in time.
    """
    mcp("execute_python_code", {"code": f"""
from omni.kit.viewport.utility import get_active_viewport
get_active_viewport().camera_path = {camera_prim_path!r}
result = "switched"
"""})
    time.sleep(settle_seconds)
    mcp("execute_python_code", {"code": f"""
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
capture_viewport_to_file(get_active_viewport(), {out_path!r})
result = "capture requested"
"""})
    for _ in range(file_poll_attempts):
        if os.path.exists(out_path) and os.path.getsize(out_path) > 5000:
            return True
        time.sleep(file_poll_seconds)
    return False


def probe_pose(prim_path: str) -> str:
    """Return world translation of a prim as a short string, or 'INVALID'."""
    code = f"""
import omni.usd
from pxr import UsdGeom
stage = omni.usd.get_context().get_stage()
p = stage.GetPrimAtPath({prim_path!r})
if not p.IsValid():
    result = "INVALID"
else:
    wt = UsdGeom.XformCache().GetLocalToWorldTransform(p)
    result = str(wt.ExtractTranslation())
"""
    r = mcp("execute_python_code", {"code": code})
    return r.get("result", {}).get("result", "?")


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out-dir", required=True,
                        help="Directory to write PNG captures + probe.json into. Created if missing.")
    parser.add_argument("--cameras", nargs="*", default=list(WRIST_CAMERA_NAMES),
                        help=f"Camera names to capture (default: all wrist cams: "
                             f"{', '.join(WRIST_CAMERA_NAMES)}).")
    parser.add_argument("--camera-paths", nargs="*", default=[],
                        help="Additional absolute USD camera prim paths to capture.")
    parser.add_argument("--auto-spawn", action="store_true",
                        help="Spawn+start streams for any wrist cam name in --cameras "
                             "that isn't already streaming. No-op for non-wrist cams.")
    parser.add_argument("--settle-seconds", type=float, default=4.0,
                        help="Seconds to wait after switching viewport before capturing (default 4).")
    parser.add_argument("--probe-prims", nargs="*", default=[
                            "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l",
                            "/World/UR5e/cable/sfp_module_visual",
                            "/World/UR5e/cable/sc_plug_visual",
                            "/World/UR5e/cable/Rope/Rope/link_0",
                            "/World/UR5e/cable/Rope/Rope/link_20",
                        ],
                        help="USD prim paths to probe world pose for + write to probe.json. "
                             "Defaults are aic-dt cable + gripper key frames.")
    args = parser.parse_args(argv[1:])

    os.makedirs(args.out_dir, exist_ok=True)
    print(f"Output dir: {args.out_dir}")

    if args.auto_spawn:
        print("Auto-spawn enabled — ensuring wrist camera streams exist:")
        auto_spawn_wrist_cameras(args.cameras)

    captures = {}
    # Capture named wrist cams
    for name in args.cameras:
        try:
            path = resolve_camera_path(name)
        except ValueError as exc:
            print(f"  [skip] {exc}")
            continue
        out_path = os.path.join(args.out_dir, f"{name}.png")
        print(f"  capturing {name} → {out_path}")
        ok = switch_viewport_and_capture(path, out_path,
                                          settle_seconds=args.settle_seconds)
        captures[name] = {"path": path, "ok": ok, "file": out_path if ok else None}
        if not ok:
            print(f"  [WARN] capture file didn't appear: {out_path}")

    # Capture extra absolute-path cameras
    for ext_path in args.camera_paths:
        slug = ext_path.strip("/").replace("/", "_")
        out_path = os.path.join(args.out_dir, f"{slug}.png")
        print(f"  capturing {ext_path} → {out_path}")
        ok = switch_viewport_and_capture(ext_path, out_path,
                                          settle_seconds=args.settle_seconds)
        captures[ext_path] = {"path": ext_path, "ok": ok, "file": out_path if ok else None}

    # Probe poses (useful for diagnostic context alongside captures)
    probes = {}
    print("Probing prim poses...")
    for pp in args.probe_prims:
        probes[pp] = probe_pose(pp)
        print(f"  {pp}: {probes[pp]}")

    probe_json = os.path.join(args.out_dir, "probe.json")
    with open(probe_json, "w") as f:
        json.dump({"captures": captures, "probes": probes,
                   "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")},
                   f, indent=2)
    print(f"\nWrote probe data: {probe_json}")
    print(f"Done. {sum(1 for c in captures.values() if c['ok'])}/{len(captures)} captures succeeded.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
