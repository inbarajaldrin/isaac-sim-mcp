#!/usr/bin/env bash
# probe_aic_controller_jointstate.sh -- inspect aic_controller's /joint_states
# subscriber to determine name-indexed vs positional access (D-11 / RESEARCH Pitfall #1).
#
# Authored by Plan 05 Task 2 (Phase 1 Foundation Parity / PARITY-03).
#
# Usage: ./probe_aic_controller_jointstate.sh [AIC_CONTROLLER_SRC_DIR]
# Default: ~/Documents/aic/aic_controller/src

set -uo pipefail
SRC="${1:-$HOME/Documents/aic/aic_controller/src}"

echo "Probing: $SRC"
if [ ! -d "$SRC" ]; then
  echo "ERROR: Source dir not found. Either pass the path as arg 1 or check out ~/Documents/aic."
  echo "Per D-13 the runtime works without ~/Documents/aic, but THIS PROBE NEEDS THE AIC REPO."
  exit 2
fi

GREP_INCLUDES=(--include='*.cpp' --include='*.hpp' --include='*.h' --include='*.cc')

echo
echo "[1] Searching for JointState subscriber declarations..."
grep -rn "JointState" "$SRC" "${GREP_INCLUDES[@]}" || \
  echo "  (no JointState references found in $SRC)"

echo
echo "[2] Searching for joint_states subscription topic..."
grep -rn 'joint_states' "$SRC" "${GREP_INCLUDES[@]}" || \
  echo "  (no joint_states topic literal found)"

echo
echo "[3] Searching for name-indexed access patterns (msg->name, name[i], find by name)..."
NAME_ACCESS=$(grep -rEn '(msg->name|->name\[|\.name\[|->name\.|\.name\.|find\([^)]*joint|index_of|joint_name|getJointIndex)' "$SRC" "${GREP_INCLUDES[@]}" | grep -iE 'joint|state|name' || true)
if [ -n "$NAME_ACCESS" ]; then
  echo "$NAME_ACCESS"
  echo "  -> name-indexed access PRESENT"
else
  echo "  -> no name-indexed access patterns found"
fi

echo
echo "[4] Searching for positional access patterns (msg->position[N], direct index)..."
POS_ACCESS=$(grep -rEn 'position\[[0-9]+\]|->position\[|\.position\[' "$SRC" "${GREP_INCLUDES[@]}" || true)
if [ -n "$POS_ACCESS" ]; then
  echo "$POS_ACCESS"
  echo "  -> positional access PRESENT"
else
  echo "  -> no positional access patterns found"
fi

echo
echo "[5] Verdict (heuristic):"
if [ -n "$NAME_ACCESS" ] && [ -z "$POS_ACCESS" ]; then
  echo "  CONCLUSION: NAME-INDEXED -- joint name ordering does NOT functionally matter."
  echo "  Implication: Plan 06's ROS2PublishJointState with articulation-discovery order is sufficient."
elif [ -z "$NAME_ACCESS" ] && [ -n "$POS_ACCESS" ]; then
  echo "  CONCLUSION: POSITIONAL -- joint name ordering IS functionally required."
  echo "  Implication: Plan 05 Task 4 (reordering wrapper) MUST be added."
elif [ -n "$NAME_ACCESS" ] && [ -n "$POS_ACCESS" ]; then
  echo "  CONCLUSION: MIXED -- both patterns present; manually review before deciding. Default-safe: POSITIONAL (add wrapper)."
else
  echo "  CONCLUSION: UNKNOWN -- neither pattern found in C++ source. Manually inspect the controller plugin."
fi
