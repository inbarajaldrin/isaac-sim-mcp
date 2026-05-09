#!/usr/bin/env python3
"""Probe: verify the 4 task-board prims are authored as RigidBodyAPI +
kinematic_enabled=True (taskboard-prim-authoring acceptance gate 1+3).

Per PRD task `taskboard-prim-authoring` and audit finding #6 in STATE.md:
the 4 prims (base_visual, two sc_port_visual, nic_card_visual) carry
RigidBodyAPI from the on-disk task_board_rigid.usd / sc_port.usd /
nic_card.usd USDs. With mass=0 + density=0 PhysX silently treats them as
kinematic; this fragile authoring is replaced by explicit kinematic_enabled=True
in extension._spawn_component_via_usd post-reference walk. AIC's own Isaac Lab
env (~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/.../aic_task_env_cfg.py)
uses RigidObjectCfg(kinematic_enabled=True) for the same parts.

Exit 0 = all 4 prims pass. Exit 1 = at least one prim fails.

Sister probe: probe_insertion_event_fires.sh — drives a stimulus and asserts
/scoring/insertion_event fires (acceptance gate 2).
"""
from __future__ import annotations

import json
import socket
import sys
import time

EXPECTED_PRIMS = [
    "/World/TaskBoard/base_visual",
    "/World/TaskBoard/SCPort_0/sc_port_visual",
    "/World/TaskBoard/SCPort_1/sc_port_visual",
    "/World/TaskBoard/NICCard/nic_card_visual",
]


def _send(req: dict, timeout: float = 180.0) -> dict:
    s = socket.socket()
    s.settimeout(timeout)
    s.connect(("localhost", 8768))
    s.sendall(json.dumps(req).encode())
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
    raise RuntimeError("MCP socket closed before complete JSON")


def _clean_quick_start() -> None:
    """Always issue new_stage first, then quick_start. Avoids state-bleed from
    a prior load_trial / partial scene which can leave quick_start with a
    'success' status but only a subset of expected prims (we want all 4 board
    prims spawned via AIC_OBJECTS defaults — quick_start's add_objects path)."""
    _send({"type": "new_stage", "params": {}})
    time.sleep(1)
    resp = _send({"type": "quick_start", "params": {}})
    status = (resp.get("result") or {}).get("status") or resp.get("status")
    msg = (resp.get("result") or {}).get("message", "")
    if status == "error" and "name is not unique" in msg.lower():
        _send({"type": "new_stage", "params": {}})
        time.sleep(1)
        _send({"type": "quick_start", "params": {}})


def main() -> int:
    _clean_quick_start()

    code = """
import omni.usd
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()
expected = %r
out = []
for path in expected:
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        out.append({'path': path, 'exists': False, 'has_rb': False, 'kinematic_enabled': None})
        continue
    has_rb = p.HasAPI(UsdPhysics.RigidBodyAPI)
    ke = None
    if has_rb:
        a = UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr()
        ke = a.Get() if a.IsValid() else None
    out.append({'path': path, 'exists': True, 'has_rb': has_rb, 'kinematic_enabled': ke})
result = out
""" % (EXPECTED_PRIMS,)
    resp = _send({"type": "execute_python_code", "params": {"code": code}})
    findings = (resp.get("result") or {}).get("result") or []
    print("[probe_taskboard_authoring] findings:")
    for f in findings:
        print(f"  {f}")

    failures = [
        f for f in findings
        if not (f.get("exists") and f.get("has_rb") and f.get("kinematic_enabled") is True)
    ]
    if failures:
        print(f"[probe_taskboard_authoring] FAIL — {len(failures)} prim(s) not authored as RigidBodyAPI+kinematic_enabled=True")
        for f in failures:
            print(f"    BAD: {f}")
        return 1
    print("[probe_taskboard_authoring] PASS — all 4 prims authored consistently (RigidBodyAPI + kinematic_enabled=True)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
