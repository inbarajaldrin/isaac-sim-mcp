#!/usr/bin/env python3
# Reference: Plan 01-06 Task 0 — pre-graph probe pattern (cf. exts/aic-dt/scripts/probe_unified_usd.py)
"""probe_root_joint.py — verify the articulation root prim path expected by
ROS2PublishJointState's targetPrim relationship exists in the unified robot USD.

Per RESEARCH.md Pattern 1 line 466 the canonical articulation root for
ROS2PublishJointState is /World/UR5e/aic_unified_robot/root_joint. The static
USD has no /World/UR5e/ wrapper (that's added at runtime by add_reference_to_stage),
so the in-USD path is /aic_unified_robot/root_joint (default prim is /World).

Run via Isaac Sim's bundled python (env_isaaclab venv lacks pxr — see Plan 05
deviation 1):
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/probe_root_joint.py | tee /tmp/root_joint_probe.txt

Exit codes:
    0 = PASS  (root_joint prim found)
    2 = WARN  (aic_unified_robot Xform exists but no root_joint child)
    1 = FAIL  (USD missing or structure changed)
"""
import sys
import os
from pxr import Usd

# script lives at exts/aic-dt/scripts/probe_root_joint.py — 4 dirnames up = repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DEFAULT_USD = os.path.join(REPO_ROOT, "exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd")


def main():
    usd_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_USD
    print(f"USD: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"FAIL: cannot open {usd_path}")
        sys.exit(1)

    candidates = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.endswith("/root_joint") or path.endswith("/aic_unified_robot"):
            candidates.append((path, prim.GetTypeName()))

    print("Candidate root prims found:")
    for path, ty in candidates:
        print(f"  {path}  (type={ty})")

    has_root_joint = any(p.endswith("/root_joint") for p, _ in candidates)
    has_aic_unified = any(p.endswith("/aic_unified_robot") for p, _ in candidates)

    if has_root_joint:
        print("PASS: root_joint prim found — Task 2's targetPrim binding will resolve at runtime.")
        print("Runtime path (after add_reference_to_stage with prim_path=/World/UR5e):")
        print("  /World/UR5e/aic_unified_robot/root_joint")
        sys.exit(0)
    elif has_aic_unified:
        print("WARN: aic_unified_robot Xform exists but no root_joint child found.")
        print("ACTION: Inspect the USD via usdcat and adjust Task 2's articulation_root accordingly.")
        sys.exit(2)
    else:
        print("FAIL: neither root_joint nor aic_unified_robot found — USD path or structure changed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
