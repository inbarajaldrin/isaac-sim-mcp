#!/bin/bash
# Visual-inspection wrapper: runs the full AIC stack (engine + adapter + model +
# Isaac Sim aic-dt extension) and freezes the robot at a chosen inspection pose
# so an operator can verify what the wrist cameras see.
#
# Mirrors run_aic_engine_against_isaac_sim.sh end-to-end, with two additions:
#   - exports AIC_FREEZE_AT_START or AIC_FREEZE_AT_HOVER into the host aic_model
#     process so CheatCode enters its hold-pose loop instead of returning
#   - skips the engine-completion wait (engine never completes; the freeze loop
#     is intentionally infinite — operator stops via Ctrl-C / cleanup)
#
# Usage:
#   bash exts/aic-dt/scripts/run_freeze_isaac_sim.sh trial_1 start    # freeze at home pose
#   bash exts/aic-dt/scripts/run_freeze_isaac_sim.sh trial_1 hover    # freeze 0.2m above port (recommended for camera check)
#   bash exts/aic-dt/scripts/run_freeze_isaac_sim.sh stop             # clean up
#
# After launch, inspect with:
#   tail -f /tmp/aic_model_isaac.log            # look for "FREEZE TRIGGERED"
#   python3 <wrist-camera-grab.py>              # sample pixels from /center_camera/image
#   In Kit GUI: switch viewport to center_camera to visually confirm what's in frame

set -euo pipefail

MODE="${2:-hover}"
TRIAL_KEY="${1:-trial_1}"

if [[ "${1:-}" == "stop" ]]; then
    echo "[freeze-wrapper] cleanup"
    pkill -TERM -f "install/aic_model/lib/aic_model/aic_model" 2>/dev/null || true
    pkill -TERM -f "install/aic_engine/lib/aic_engine/aic_engine" 2>/dev/null || true
    pkill -TERM -f "install/aic_adapter/lib/aic_adapter/aic_adapter" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "install/aic_model/lib/aic_model/aic_model" 2>/dev/null || true
    pkill -KILL -f "install/aic_engine/lib/aic_engine/aic_engine" 2>/dev/null || true
    pkill -KILL -f "install/aic_adapter/lib/aic_adapter/aic_adapter" 2>/dev/null || true
    echo "[freeze-wrapper] stopped (Isaac Sim left running — kill with isaacsim_launch.sh kill)"
    exit 0
fi

case "$MODE" in
    start) FREEZE_VAR="AIC_FREEZE_AT_START" ;;
    hover) FREEZE_VAR="AIC_FREEZE_AT_HOVER" ;;
    *) echo "Usage: $0 <trial_key> {start|hover}"; exit 2 ;;
esac

# Export the freeze flag so the host aic_model child process inherits it.
export "$FREEZE_VAR"=1

# Delegate to the canonical wrapper — it does the heavy lifting (Isaac Sim
# launch, zenoh router, model+engine+adapter sequencing). The freeze env var
# propagates into every child process via export above.
echo "[freeze-wrapper] mode=$MODE  trial=$TRIAL_KEY  freeze_env=$FREEZE_VAR=1"
echo "[freeze-wrapper] handing off to run_aic_engine_against_isaac_sim.sh"
echo "[freeze-wrapper] NOTE: engine will reach its time_limit (180s) and the wrapper"
echo "[freeze-wrapper] will then return — but the model freeze loop will already have"
echo "[freeze-wrapper] held the robot at the target pose for inspection."
echo ""

exec bash "$(dirname "$0")/run_aic_engine_against_isaac_sim.sh" "$TRIAL_KEY"
