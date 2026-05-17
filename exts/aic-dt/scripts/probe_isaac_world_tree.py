#!/usr/bin/env python3
# Reference: derived from session work 2026-05-17 probing the live Isaac
# Sim TF tree for cable-fidelity comparison (commits 5ba2bbe..e34ee5f).
"""Probe the live Isaac Sim world transforms for a list of prim paths via
the aic-dt MCP socket, with SCALE-CORRECTED quaternion extraction.

# Why this script exists

`Gf.Matrix4d.ExtractRotation().GetQuat()` mis-extracts rotation from a
world matrix whose upper-3×3 has BAKED SCALE. The cable USD's
`xformOp:scale = (0.01, 0.01, 0.01)` on connector visuals means the
world transform's row magnitudes are 0.01, not 1.0; the built-in
extractor returns a quaternion that does not satisfy `M * v = R * v`
for the actual rows.

This cost ~1 hr of debugging on 2026-05-17 (chased a phantom "90° off"
orientation that was actually correct). The fix:

  1. Read the world matrix rows
  2. Normalize each row independently (strips uniform scale)
  3. Apply the standard quat-from-rotation-matrix formula on the
     normalized matrix, treating it as column-form (m[i][j] from row j,
     col i — the row-vector-to-column-vector transpose)

The `rotation_quat_wxyz_from_matrix` helper below is the canonical
implementation. Use it instead of `wt.ExtractRotation().GetQuat()`
whenever the prim has non-unit `xformOp:scale`.

# Output

For each entry in `ENTRIES`, prints world pos + wxyz quat + RPY°.
Missing prims are reported as `MISSING`.

Usage:

  python3 probe_isaac_world_tree.py

  # Or with a custom entry list (edit ENTRIES below; or import as
  # library and pass your own list)
"""

from __future__ import annotations

import json
import socket
import sys
from typing import List, Tuple

MCP_HOST = "127.0.0.1"
MCP_PORT = 8768

# Default entries — gripper chain + cable + task board (post-quick_start
# + load_trial). Edit to suit your scene; missing prims are reported.
DEFAULT_ENTRIES: List[Tuple[str, str]] = [
    ("tool0",                       "/World/UR5e/aic_unified_robot/tool0"),
    ("flange",                      "/World/UR5e/aic_unified_robot/flange"),
    ("wrist_3_link",                "/World/UR5e/aic_unified_robot/wrist_3_link"),
    ("ati/tool_link",               "/World/UR5e/aic_unified_robot/ati_tool_link"),
    ("gripper/hande_base_link",     "/World/UR5e/aic_unified_robot/gripper_hande_base_link"),
    ("gripper/hande_finger_link_l", "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l"),
    ("gripper/hande_finger_link_r", "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_r"),
    ("gripper/tcp",                 "/World/UR5e/aic_unified_robot/gripper_tcp"),
    ("cable",                       "/World/UR5e/cable"),
    ("cable/sfp_module_visual",     "/World/UR5e/cable/sfp_module_visual"),
    ("cable/sc_plug_visual",        "/World/UR5e/cable/sc_plug_visual"),
    ("cable/Rope/link_0",           "/World/UR5e/cable/Rope/Rope/link_0"),
    ("cable/Rope/link_10",          "/World/UR5e/cable/Rope/Rope/link_10"),
    ("cable/Rope/link_20",          "/World/UR5e/cable/Rope/Rope/link_20"),
    ("TaskBoard/Mesh",              "/World/TaskBoard/Mesh"),
    ("TaskBoard/SCPort_1",          "/World/TaskBoard/SCPort_1"),
    ("TaskBoard/SFPMountRail_0",    "/World/TaskBoard/SFPMountRail_0"),
    ("TaskBoard/SCMountRail_0",     "/World/TaskBoard/SCMountRail_0"),
    ("TaskBoard/LCMountRail_1",     "/World/TaskBoard/LCMountRail_1"),
]


def _mcp_call(payload: dict, timeout: int = 60) -> dict:
    s = socket.socket()
    s.settimeout(timeout)
    s.connect((MCP_HOST, MCP_PORT))
    s.sendall(json.dumps(payload).encode())
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
    raise RuntimeError("connection closed without complete JSON")


# Code that runs inside Kit's Python via execute_python_code. The
# scale-corrected quat extraction lives here as the canonical reference
# implementation. Copy this function whenever you need to extract a
# rotation quat from any prim whose world matrix has non-unit
# xformOp:scale baked in.
_PROBE_CODE_TEMPLATE = r'''
from pxr import UsdGeom, Gf
import omni.usd, math

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

def rotation_quat_wxyz_from_matrix(m_4x4):
    """Extract unit rotation quat from a 4x4 transform that may have
    uniform scale baked into the upper-3x3. Strips scale by per-row
    normalization, then applies the standard quat-from-rotation-matrix
    formula in column-form (USD stores row-form; transpose at access)."""
    rows = []
    for i in range(3):
        v = Gf.Vec3d(m_4x4[i][0], m_4x4[i][1], m_4x4[i][2])
        n = v.GetLength()
        rows.append([v[0]/n, v[1]/n, v[2]/n] if n > 1e-12 else [0.0, 0.0, 0.0])
    def col(i, j):
        return rows[j][i]   # transpose: row-form rows[j][i] = col-form m[i][j]
    t = col(0,0) + col(1,1) + col(2,2)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25*s,
                (col(2,1) - col(1,2)) / s,
                (col(0,2) - col(2,0)) / s,
                (col(1,0) - col(0,1)) / s)
    elif col(0,0) > col(1,1) and col(0,0) > col(2,2):
        s = math.sqrt(1.0 + col(0,0) - col(1,1) - col(2,2)) * 2
        return ((col(2,1) - col(1,2)) / s,
                0.25 * s,
                (col(0,1) + col(1,0)) / s,
                (col(0,2) + col(2,0)) / s)
    elif col(1,1) > col(2,2):
        s = math.sqrt(1.0 + col(1,1) - col(0,0) - col(2,2)) * 2
        return ((col(0,2) - col(2,0)) / s,
                (col(0,1) + col(1,0)) / s,
                0.25 * s,
                (col(1,2) + col(2,1)) / s)
    else:
        s = math.sqrt(1.0 + col(2,2) - col(0,0) - col(1,1)) * 2
        return ((col(1,0) - col(0,1)) / s,
                (col(0,2) + col(2,0)) / s,
                (col(1,2) + col(2,1)) / s,
                0.25 * s)

def rpy_deg(q):
    w, x, y, z = q
    r = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sp = max(-1.0, min(1.0, 2*(w*y - z*x)))
    p = math.asin(sp)
    yw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return (math.degrees(r), math.degrees(p), math.degrees(yw))

ENTRIES = __ENTRIES__
out_lines = []
for label, path in ENTRIES:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        out_lines.append(f"{label:<35s} MISSING ({path})")
        continue
    wt = xc.GetLocalToWorldTransform(prim)
    pos = wt.ExtractTranslation()
    q = rotation_quat_wxyz_from_matrix(wt)
    r, p, yw = rpy_deg(q)
    out_lines.append(
        f"{label:<35s} pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
        f"quat_wxyz=({q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f}) "
        f"rpy=({r:+6.1f},{p:+6.1f},{yw:+6.1f})°"
    )

result = "\n".join(out_lines)
'''


def probe_isaac_world(entries: List[Tuple[str, str]] = None) -> str:
    """Probe the live Isaac Sim world transforms for the given (label, prim_path)
    list. Returns the formatted text block. Uses MCP execute_python_code."""
    if entries is None:
        entries = DEFAULT_ENTRIES
    code = _PROBE_CODE_TEMPLATE.replace("__ENTRIES__", repr(entries))
    resp = _mcp_call(
        {"type": "execute_python_code", "params": {"code": code}},
        timeout=30,
    )
    return resp.get("result", {}).get("result", "")


def main(argv):
    text = probe_isaac_world()
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
