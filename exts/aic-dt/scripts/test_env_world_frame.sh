#!/usr/bin/env bash
# Reference: PRD task `environment-world-frame-reconcile`.
# Reference: .tug/findings/world-frame-reconcile.md
#
# Verifies the multi-entity world-frame reconciliation:
#   - extension.py _enclosure_position == (0.0, 0.0, 0.0)
#   - extension.py _ground_plane_z == 0.0
#   - extension.py _cmd_spawn_task_board_base defaults match Gazebo
#     (x=0.15, y=-0.2, z=1.14, roll=0, pitch=0, yaw=3.1415)
#   - Live MCP probe: /World/AIC_Enclosure translateOp == (0,0,0)
#     and /World/defaultGroundPlane Z == 0
#   - Regression: trial-home-robot-pose still passes.
#
# Canonical source: ~/Documents/aic/aic_description/world/aic.sdf
# (Enclosure + Floor <include> have no <pose> → world origin) and
# ~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py:629-668
# (task_board defaults).

set -eo pipefail

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
EXT_PY="$REPO/exts/aic-dt/aic_dt/extension.py"
SCRIPTS="$(cd "$(dirname "$0")" && pwd)"

echo "[test] gate 1 — static grep on $EXT_PY"

grep -qE '^\s*self\._enclosure_position\s*=\s*\(0\.0,\s*0\.0,\s*0\.0\)' "$EXT_PY" \
    || { echo "FAIL: _enclosure_position is not (0.0, 0.0, 0.0)"; exit 1; }
grep -qE '^\s*self\._ground_plane_z\s*=\s*0\.0\b' "$EXT_PY" \
    || { echo "FAIL: _ground_plane_z is not 0.0"; exit 1; }
python3 - "$EXT_PY" <<'PY'
import sys, re, pathlib
src = pathlib.Path(sys.argv[1]).read_text()
m = re.search(
    r"def\s+_cmd_spawn_task_board_base\(\s*self,\s*"
    r"x:\s*float\s*=\s*(?P<x>[-0-9.]+),\s*"
    r"y:\s*float\s*=\s*(?P<y>[-0-9.]+),\s*"
    r"z:\s*float\s*=\s*(?P<z>[-0-9.]+),\s*"
    r"roll:\s*float\s*=\s*(?P<r>[-0-9.]+),\s*"
    r"pitch:\s*float\s*=\s*(?P<p>[-0-9.]+),\s*"
    r"yaw:\s*float\s*=\s*(?P<yw>[-0-9.]+)",
    src,
)
if not m:
    raise SystemExit("FAIL: _cmd_spawn_task_board_base signature did not parse")
got = {k: float(v) for k, v in m.groupdict().items()}
want = {"x": 0.15, "y": -0.2, "z": 1.14, "r": 0.0, "p": 0.0, "yw": 3.1415}
for k, v in want.items():
    if abs(got[k] - v) > 1e-6:
        raise SystemExit(f"FAIL: _cmd_spawn_task_board_base default {k}={got[k]} != {v}")
print(f"[test]   task_board defaults OK: {got}")
PY

echo "[test] gate 1 PASS"

echo "[test] gate 2 — live MCP probe via :8768"
if ! nc -z localhost 8768; then
    echo "FAIL: Isaac Sim MCP socket :8768 down — relaunch via isaacsim_launch.sh"
    exit 2
fi

python3 - <<'PY'
import socket, json, sys

def call(typ, params=None, timeout=180):
    s = socket.socket(); s.settimeout(timeout)
    s.connect(("127.0.0.1", 8768))
    s.sendall(json.dumps({"type": typ, "params": params or {}}).encode())
    buf = b""
    while True:
        c = s.recv(16384)
        if not c: break
        buf += c
        try:
            return json.loads(buf.decode())
        except json.JSONDecodeError:
            continue
    raise RuntimeError("socket closed before reply parsed")

print("[probe] new_stage")
call("new_stage")
print("[probe] quick_start (up to 180s)")
qs = call("quick_start", timeout=180)
print(f"[probe] quick_start status: {qs.get('status')}")
if qs.get("status") != "success":
    raise SystemExit(f"FAIL: quick_start did not succeed: {qs}")

code = r'''
from pxr import UsdGeom
import omni.usd
stage = omni.usd.get_context().get_stage()

def get_translate(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    x = UsdGeom.Xformable(prim)
    for op in x.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            v = op.Get()
            return [float(v[0]), float(v[1]), float(v[2])]
    return [0.0, 0.0, 0.0]

result = {
    "enclosure": get_translate("/World/AIC_Enclosure"),
    "ground": get_translate("/World/defaultGroundPlane"),
    "task_board": get_translate("/World/TaskBoard"),
}
'''
res = call("execute_python_code", {"code": code}, timeout=60)
inner = res.get("result", {}).get("result")
if not isinstance(inner, dict):
    raise SystemExit(f"FAIL: execute_python_code returned unexpected shape: {res}")
enc = inner.get("enclosure")
gnd = inner.get("ground")
tbd = inner.get("task_board")
if enc is None or gnd is None:
    raise SystemExit(f"FAIL: missing enclosure/ground in probe: {inner}")
ex, ey, ez = enc
gx, gy, gz = gnd
print(f"[probe] enclosure=({ex:.4f},{ey:.4f},{ez:.4f}) ground=({gx:.4f},{gy:.4f},{gz:.4f})")
if tbd is not None:
    print(f"[probe] task_board={tuple(round(v,4) for v in tbd)}")
TOL = 1e-3
if abs(ex) > TOL or abs(ey) > TOL or abs(ez) > TOL:
    raise SystemExit(f"FAIL: /World/AIC_Enclosure translate {enc} != (0,0,0)")
if abs(gz) > TOL:
    raise SystemExit(f"FAIL: /World/defaultGroundPlane Z {gz} != 0")
print("[probe] gate 2 sub-asserts PASS")
PY
RC=$?
if [ $RC -ne 0 ]; then
    echo "[test] gate 2 FAIL"
    exit 1
fi

echo "[test] gate 2 PASS"

echo "[test] gate 3 — regression check trial-home-robot-pose"
if bash "$SCRIPTS/test_trial_home_robot_pose.sh"; then
    echo "[test] gate 3 PASS"
else
    echo "[test] gate 3 FAIL — trial-home-robot-pose regressed"
    exit 1
fi

echo "[test] ALL GATES PASS"
exit 0
