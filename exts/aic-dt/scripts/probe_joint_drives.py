#!/usr/bin/env python3
# Reference: PRD task joint-drives-urdf-reconcile; canonical values from
# exts/aic-dt/docs/aic-isaaclab-leverage.md (AIC Isaac Lab env
# ImplicitActuatorCfg: stiffness=2000, damping=100, effort_limit_sim=87).
"""probe_joint_drives.py — verify the 6 arm joint drives match canonical values.

Workflow:
  1. Connect to the live Isaac Sim aic-dt MCP socket on localhost:8768.
  2. Trigger quick_start so load_robot has run and authored DriveAPI values.
  3. Read each arm joint's UsdPhysics.DriveAPI("angular") attributes via
     execute_python_code, then compare against expected values within a
     tolerance percentage.

Exit codes:
  0 = PASS (all 6 arm joints match)
  1 = FAIL (≥1 joint outside tolerance OR MCP unreachable OR USD probe error)

Usage:
  python3 exts/aic-dt/scripts/probe_joint_drives.py \\
      --expected-stiffness 2000 --expected-damping 100 \\
      --expected-max-force 87 --tolerance-pct 5
"""
import argparse
import json
import socket
import sys
import time

MCP_HOST = "127.0.0.1"
MCP_PORT = 8768

ROBOT_PRIM = "/World/UR5e"
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def mcp_send(type_: str, params: dict, timeout_s: float = 120.0) -> dict:
    s = socket.socket()
    s.settimeout(timeout_s)
    s.connect((MCP_HOST, MCP_PORT))
    try:
        s.sendall(json.dumps({"type": type_, "params": params}).encode())
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
        raise RuntimeError("MCP socket closed before complete JSON response")
    finally:
        s.close()


def _extract_result_text(resp: dict) -> str:
    """execute_python_code result extraction (handles old + new wrapping)."""
    if isinstance(resp.get("result"), str):
        return resp["result"]
    if isinstance(resp.get("result"), dict):
        inner = resp["result"]
        return inner.get("result") or inner.get("output") or ""
    return resp.get("output") or ""


def probe_drives() -> dict:
    """Read live DriveAPI attrs for each arm joint. Returns dict per joint."""
    code = (
        "import json\n"
        "import omni.usd\n"
        "from pxr import UsdPhysics, PhysxSchema\n"
        "stage = omni.usd.get_context().get_stage()\n"
        f"joint_names = {JOINT_NAMES!r}\n"
        f"prim_path = {ROBOT_PRIM!r}\n"
        "out = {}\n"
        "for jn in joint_names:\n"
        "    jp = f'{prim_path}/aic_unified_robot/joints/{jn}'\n"
        "    prim = stage.GetPrimAtPath(jp)\n"
        "    if not prim.IsValid():\n"
        "        out[jn] = {'found': False, 'path': jp}\n"
        "        continue\n"
        "    drive = UsdPhysics.DriveAPI.Get(prim, 'angular')\n"
        "    if not drive:\n"
        "        out[jn] = {'found': True, 'has_drive': False, 'path': jp}\n"
        "        continue\n"
        "    s_attr = drive.GetStiffnessAttr()\n"
        "    d_attr = drive.GetDampingAttr()\n"
        "    f_attr = drive.GetMaxForceAttr()\n"
        "    out[jn] = {\n"
        "        'found': True, 'has_drive': True, 'path': jp,\n"
        "        'stiffness': float(s_attr.Get()) if s_attr.HasAuthoredValue() else None,\n"
        "        'damping':   float(d_attr.Get()) if d_attr.HasAuthoredValue() else None,\n"
        "        'maxForce':  float(f_attr.Get()) if f_attr.HasAuthoredValue() else None,\n"
        "    }\n"
        "from pxr import Sdf\n"
        "art_path = f'{prim_path}/aic_unified_robot'\n"
        "art_prim = stage.GetPrimAtPath(art_path)\n"
        "art_info = {'found': bool(art_prim.IsValid()), 'path': art_path}\n"
        "if art_prim.IsValid():\n"
        "    pa = PhysxSchema.PhysxArticulationAPI.Get(stage, Sdf.Path(art_path))\n"
        "    if pa:\n"
        "        ps = pa.GetSolverPositionIterationCountAttr()\n"
        "        vs = pa.GetSolverVelocityIterationCountAttr()\n"
        "        art_info['pos_iter'] = int(ps.Get()) if ps.HasAuthoredValue() else None\n"
        "        art_info['vel_iter'] = int(vs.Get()) if vs.HasAuthoredValue() else None\n"
        "out['__articulation__'] = art_info\n"
        "result = json.dumps(out)\n"
    )
    resp = mcp_send("execute_python_code", {"code": code})
    payload = _extract_result_text(resp).strip()
    if not payload:
        raise RuntimeError(f"empty execute_python_code result; raw={resp!r}")
    return json.loads(payload)


def within_tolerance(actual: float, expected: float, tolerance_pct: float) -> bool:
    if actual is None:
        return False
    if expected == 0:
        return abs(actual) <= tolerance_pct / 100.0
    return abs(actual - expected) / abs(expected) <= tolerance_pct / 100.0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--expected-stiffness", type=float, default=2000.0)
    p.add_argument("--expected-damping",   type=float, default=100.0)
    p.add_argument("--expected-max-force", type=float, default=87.0)
    p.add_argument("--tolerance-pct",      type=float, default=5.0)
    p.add_argument("--skip-quick-start",   action="store_true",
                   help="skip the quick_start MCP call (use if already invoked this session)")
    args = p.parse_args()

    print(f"[probe_joint_drives] expected stiffness={args.expected_stiffness} "
          f"damping={args.expected_damping} maxForce={args.expected_max_force} "
          f"tolerance={args.tolerance_pct}%")

    if not args.skip_quick_start:
        print("[probe_joint_drives] step 1/2: quick_start")
        try:
            qs = mcp_send("quick_start", {})
            qs_status = qs.get("status")
            qs_msg = qs.get("message", "") or ""
            inner = qs.get("result", {}) if isinstance(qs.get("result"), dict) else {}
            print(f"  status={qs_status or inner.get('status')}  "
                  f"message={(inner.get('message','') or qs_msg)[:120]}")
            # "ur5e_view name not unique" is the benign idempotency-failure mode:
            # load_robot already ran in this session AND authored drives in the
            # USD before the ArticulationView add raised. Drives persist on USD,
            # so the probe can still inspect them. Distinguish from genuine errors.
            if qs_status == "error" and "ur5e_view" not in qs_msg and "name is not unique" not in qs_msg:
                print(f"  FAIL: quick_start error (not the known idempotency mode): {qs_msg[:200]}",
                      file=sys.stderr)
                return 1
        except Exception as e:
            print(f"  FAIL: quick_start unreachable: {type(e).__name__}: {e}", file=sys.stderr)
            return 1
        # Brief settle so DriveAPI authoring (synchronous in load_robot) is committed.
        time.sleep(0.5)

    print("[probe_joint_drives] step 2/2: probe DriveAPI on each arm joint")
    try:
        result = probe_drives()
    except Exception as e:
        print(f"  FAIL: probe RPC error: {type(e).__name__}: {e}", file=sys.stderr)
        return 1

    art_info = result.pop("__articulation__", {})
    print(f"  articulation root: {art_info}")

    failures = []
    for jn in JOINT_NAMES:
        info = result.get(jn, {})
        if not info.get("found"):
            failures.append(f"{jn}: prim not found at {info.get('path')!r}")
            continue
        if not info.get("has_drive"):
            failures.append(f"{jn}: no UsdPhysics.DriveAPI at {info.get('path')!r}")
            continue
        s_ok = within_tolerance(info["stiffness"], args.expected_stiffness, args.tolerance_pct)
        d_ok = within_tolerance(info["damping"],   args.expected_damping,   args.tolerance_pct)
        f_ok = within_tolerance(info["maxForce"],  args.expected_max_force, args.tolerance_pct)
        verdict = "OK" if (s_ok and d_ok and f_ok) else "FAIL"
        print(f"  {verdict}  {jn}: "
              f"stiffness={info['stiffness']} (exp {args.expected_stiffness}) "
              f"damping={info['damping']} (exp {args.expected_damping}) "
              f"maxForce={info['maxForce']} (exp {args.expected_max_force})")
        if not s_ok:
            failures.append(f"{jn}: stiffness={info['stiffness']} outside ±{args.tolerance_pct}% of {args.expected_stiffness}")
        if not d_ok:
            failures.append(f"{jn}: damping={info['damping']} outside ±{args.tolerance_pct}% of {args.expected_damping}")
        if not f_ok:
            failures.append(f"{jn}: maxForce={info['maxForce']} outside ±{args.tolerance_pct}% of {args.expected_max_force}")

    if failures:
        print()
        print("=== VERDICT: FAIL ===", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    print()
    print("=== VERDICT: PASS — all 6 arm joints match canonical drive values ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
