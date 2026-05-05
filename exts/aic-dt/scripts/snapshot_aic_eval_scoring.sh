#!/bin/bash
# Reference: Plan 03-01 of Phase 3 (cable-physics).
# Brings up aic_eval + CheatCode for ~30s, captures /scoring/* + cable-related /tf
# frames into .planning/phases/03-cable-physics/snapshot/aic_eval_scoring_capture.txt.
#
# Usage:  bash exts/aic-dt/scripts/snapshot_aic_eval_scoring.sh [--no-stop]
#
# --no-stop: leave containers running after capture (for further inspection)
#
# Output: .planning/phases/03-cable-physics/snapshot/aic_eval_scoring_capture.txt
set -e

OUTDIR="$(cd "$(dirname "$0")/../../.." && pwd)/.planning/phases/03-cable-physics/snapshot"
mkdir -p "$OUTDIR"
OUT="$OUTDIR/aic_eval_scoring_capture.txt"
NOSTOP=0
[ "${1:-}" = "--no-stop" ] && NOSTOP=1

cd ~/Documents/aic
echo "=== Bringing up aic_eval + aic_model (CheatCode) headless ==="
./scripts/run_cheatcode.sh headless &
WRAPPER_PID=$!

echo "=== Waiting 35s for containers + CheatCode to stabilize ==="
sleep 35

# Detect both containers up
if ! docker ps --format '{{.Names}}' | grep -q "^aic_eval$"; then
    echo "FATAL: aic_eval container not running after 35s. Check /tmp/aic-gazebo.log"
    [ $NOSTOP -eq 0 ] && ./scripts/run_cheatcode.sh stop
    exit 1
fi

echo "=== Capturing topic surface + scoring messages ==="
docker exec aic_eval bash -c '
source /opt/ros/kilted/setup.bash
echo "===== ros2 topic list ====="
ros2 topic list
echo
echo "===== /scoring/insertion_event info ====="
ros2 topic info /scoring/insertion_event -v 2>&1 || echo "(topic absent)"
echo
echo "===== /scoring/insertion_event echo (10s) ====="
timeout 10 ros2 topic echo /scoring/insertion_event 2>&1 | head -30 || true
echo
echo "===== /scoring/tf info ====="
ros2 topic info /scoring/tf -v 2>&1 || echo "(topic absent)"
echo
echo "===== /scoring/tf echo (1 message) ====="
timeout 5 ros2 topic echo /scoring/tf --once 2>&1 | head -100 || true
echo
echo "===== /tf grep for cable / plug / port frames ====="
timeout 5 ros2 topic echo /tf 2>&1 | grep -E "child_frame_id.*(cable|plug|port|sfp|sc_)" | sort -u | head -40 || true
echo
echo "===== /scoring/tf rate (5s sample) ====="
timeout 6 ros2 topic hz /scoring/tf 2>&1 | head -8 || true
' > "$OUT" 2>&1

echo "=== Capture done. Output: $OUT ==="
wc -l "$OUT"

if [ $NOSTOP -eq 0 ]; then
    echo "=== Tearing down containers ==="
    cd ~/Documents/aic
    ./scripts/run_cheatcode.sh stop
    wait $WRAPPER_PID 2>/dev/null
fi

echo "=== DONE ==="
