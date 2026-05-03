#!/usr/bin/env bash
# scripts/verify_phase_2.sh — Phase 2 verifier harness.
# Runs Phase 1 regression smoke + Phase 2 smoke + Phase 2 workspace-build artifact
# checks. Per D-12; gates Phase 2 closure.
#
# Prerequisites:
#   1. Isaac Sim with aic-dt extension is RUNNING (per CLAUDE.md launch path).
#   2. quick_start MCP atom has been called (controller loop is active).
#   3. The Python 3.11 ROS workspace at
#      ~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/
#      contains aic_control_interfaces + ros_gz_interfaces (Plan 02-01 build).
#   4. ~/env_isaaclab/bin/activate has been sourced (workspace LD ordering).
#
# Usage:
#   bash exts/aic-dt/scripts/verify_phase_2.sh
set -uo pipefail   # NOT -e — report all failures rather than first-stop.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXT_DIR="$REPO_ROOT/exts/aic-dt"

# Per-package install layout (the ros_workspaces build doesn't merge installs):
#   $WS_BUILD/<pkg>/local/lib/python3.11/dist-packages/<pkg>/__init__.py
WS_BUILD="$HOME/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install"
AIC_PY="$WS_BUILD/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces"
ROS_GZ_PY="$WS_BUILD/ros_gz_interfaces/local/lib/python3.11/dist-packages/ros_gz_interfaces"

echo "============================================"
echo "Phase 2 Controller Loop — verifier harness"
echo "============================================"

PASS=0
FAIL=0
report() {
  if [[ $1 -eq 0 ]]; then
    echo "  ✓ $2"
    PASS=$((PASS+1))
  else
    echo "  ✗ $2"
    FAIL=$((FAIL+1))
  fi
}

# -------------------------------------------------------------------- #
# Step 1 — Workspace build artifacts present (D-05 fix; Plan 02-01)
# -------------------------------------------------------------------- #
echo
echo "[1/5] Workspace build artifacts (D-05 fix — Plan 02-01)"
[[ -f "$AIC_PY/__init__.py" ]]
report $? "aic_control_interfaces Python 3.11 build present at $AIC_PY"
[[ -f "$ROS_GZ_PY/__init__.py" ]]
report $? "ros_gz_interfaces Python 3.11 build present at $ROS_GZ_PY"
# Reject cpython-310 tagged .so (would mean wrong-Python build)
if ls "$AIC_PY"/*.so 2>/dev/null | grep -q cpython-310; then
  report 1 "aic_control_interfaces .so files are NOT cpython-310-tagged"
else
  report 0 "aic_control_interfaces .so files are NOT cpython-310-tagged"
fi
if ls "$ROS_GZ_PY"/*.so 2>/dev/null | grep -q cpython-310; then
  report 1 "ros_gz_interfaces .so files are NOT cpython-310-tagged"
else
  report 0 "ros_gz_interfaces .so files are NOT cpython-310-tagged"
fi

# -------------------------------------------------------------------- #
# Step 2 — Python 3.11 ABI import gate
# -------------------------------------------------------------------- #
echo
echo "[2/5] Python 3.11 ABI import (the Plan 02-02..06 prereq)"
bash -c 'source ~/env_isaaclab/bin/activate && ~/env_isaaclab/bin/python -c "
from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState
from ros_gz_interfaces.msg import Contacts, Contact, Entity
print(\"OK\")
" 2>&1' | grep -q "^OK$"
report $? "aic_control_interfaces.msg + ros_gz_interfaces.msg import under Python 3.11"

# -------------------------------------------------------------------- #
# Step 3 — Source-side syntactic checks (no Isaac Sim required)
# -------------------------------------------------------------------- #
echo
echo "[3/5] Source-side syntax checks"
python3 -c "import ast; ast.parse(open('$EXT_DIR/aic_dt/controller_loop.py').read())"
report $? "controller_loop.py is syntactically valid Python"
python3 -c "import ast; ast.parse(open('$EXT_DIR/aic_dt/extension.py').read())"
report $? "extension.py is syntactically valid Python"
python3 -c "import ast; ast.parse(open('$EXT_DIR/scripts/smoke_test_aic_controller.py').read())"
report $? "smoke_test_aic_controller.py is syntactically valid Python"
# All STUB markers from Plan 02-02 must be gone
if grep -q "STUB" "$EXT_DIR/aic_dt/controller_loop.py"; then
  report 1 "controller_loop.py has no STUB markers"
else
  report 0 "controller_loop.py has no STUB markers"
fi
# DEFAULT_OFF_LIMIT_PRIMS must be non-empty
python3 -c "
import re
m = re.search(r'DEFAULT_OFF_LIMIT_PRIMS\s*=\s*\[(.*?)\]', open('$EXT_DIR/aic_dt/controller_loop.py').read(), re.S)
items = [x for x in m.group(1).split(',') if x.strip().startswith(chr(34))]
assert len(items) > 0, 'DEFAULT_OFF_LIMIT_PRIMS empty'
print(f'OK: {len(items)} prims')
" >/dev/null 2>&1
report $? "DEFAULT_OFF_LIMIT_PRIMS is non-empty (PARITY-06 cannot silently no-op)"

# -------------------------------------------------------------------- #
# Step 4 — DX-02 audit (Phase 1 + Phase 2 atoms × 4 surfaces)
# -------------------------------------------------------------------- #
echo
echo "[4/5] DX-02 4-surface contract audit"
~/env_isaaclab/bin/python "$EXT_DIR/scripts/audit_dx02.py" >/tmp/audit_dx02_phase2.log 2>&1
report $? "audit_dx02.py exits 0 (Phase 1 atoms + Phase 2 controller-loop atoms)"

# -------------------------------------------------------------------- #
# Step 5 — Smoke tests against running Isaac Sim instance
# -------------------------------------------------------------------- #
echo
echo "[5/5] Smoke tests (require running aic-dt + quick_start)"
echo "      Phase 1 regression smoke:"
python3 "$EXT_DIR/scripts/smoke_test_aic_parity.py"
PARITY_RC=$?
report $PARITY_RC "Phase 1 smoke (smoke_test_aic_parity.py): 13/13 expected"
echo "      Phase 2 smoke:"
python3 "$EXT_DIR/scripts/smoke_test_aic_controller.py"
CONTROLLER_RC=$?
report $CONTROLLER_RC "Phase 2 smoke (smoke_test_aic_controller.py): all checks pass"

# -------------------------------------------------------------------- #
# Verdict
# -------------------------------------------------------------------- #
echo
echo "============================================"
echo "Phase 2 verifier: $PASS passed / $FAIL failed"
echo "============================================"
exit $FAIL
