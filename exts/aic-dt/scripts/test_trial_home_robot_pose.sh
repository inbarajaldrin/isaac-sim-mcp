#!/usr/bin/env bash
# Reference: PRD task `trial-home-robot-pose`.
#
# Verifies: load_trial applies home_joint_positions from sample_config.yaml
# top-level robot.home_joint_positions block so CheatCode starts trials with
# the gripper above the task board.
#
# Procedure:
#   1. Fire trial_1 via the standard wrapper.
#   2. Assert /tmp/aic_model_isaac.log contains a `xy_error: <x> <y>` line
#      with |x| <= 0.1 AND |y| <= 0.1 (units: meters; baseline pre-fix was
#      -0.66 / -0.61, target is ~10-50mm per PRD task acceptance).
# Cleans up the test outcome JSON on exit.

set -eo pipefail

SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
MODEL_LOG="/tmp/aic_model_isaac.log"
OUT_JSON="/tmp/test_trial_home_robot_pose__trial_1.json"

cleanup() {
    rm -f "$OUT_JSON" "/tmp/aic_engine_single_trial_trial_1.yaml"
}
trap cleanup EXIT

echo "[test] firing trial_1 (single-trial wrapper) ..."
set +e
bash "$SCRIPTS/run_aic_engine_against_isaac_sim.sh" trial_1 --output-json="$OUT_JSON" >/dev/null 2>&1
WRAPPER_RC=$?
set -e

if [ ! -f "$MODEL_LOG" ]; then
    echo "[test] FAIL: $MODEL_LOG missing — aic_model container never produced output"
    exit 1
fi

# Use the LAST xy_error line — wrapper truncates this file at fire start so
# whatever is here belongs to this run.
XY_LINE=$(grep -aE "xy_error:" "$MODEL_LOG" | tail -1 || true)
if [ -z "$XY_LINE" ]; then
    echo "[test] FAIL: no 'xy_error:' line found in $MODEL_LOG"
    echo "[test] last 20 model log lines for diagnosis:"
    tail -20 "$MODEL_LOG"
    exit 1
fi

echo "[test] observed: $XY_LINE"
# Format: "[INFO] [ts] [aic_model]: pfrac: <pf> xy_error: <ex> <ey>   integrators: ..."
ERR_X=$(echo "$XY_LINE" | sed -E 's/.*xy_error:[[:space:]]+(-?[0-9.e+-]+)[[:space:]]+(-?[0-9.e+-]+).*/\1/')
ERR_Y=$(echo "$XY_LINE" | sed -E 's/.*xy_error:[[:space:]]+(-?[0-9.e+-]+)[[:space:]]+(-?[0-9.e+-]+).*/\2/')

python3 - "$ERR_X" "$ERR_Y" <<'PY'
import sys
ex, ey = float(sys.argv[1]), float(sys.argv[2])
print(f"[test] parsed xy_error: x={ex} y={ey}")
TOL = 0.1
if abs(ex) > TOL or abs(ey) > TOL:
    raise SystemExit(
        f"FAIL: |xy_error|={abs(ex):.4f}/{abs(ey):.4f} exceeds tolerance {TOL} m "
        f"— home_joint_positions not landing at trial start (baseline was -0.66/-0.61)"
    )
print(f"[test] PASS — xy_error within +/-{TOL} m on both axes")
PY

echo "[test] PASS"
exit 0
