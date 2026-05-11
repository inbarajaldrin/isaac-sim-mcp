#!/bin/bash
# Reference: plans/prd.json::cable-physics-descope-validate
#
# After R1 descope of cable-physics-fidelity (cable kept SetActive(False) per
# out_of_scope_for_m1), re-fire trial_1 to confirm scoring-stoprecording-tf-fix
# recovers (engine bag closes cleanly, completed_steps != -1) and document
# insertion_event_fired result (true or false — both acceptable; the value
# determines whether motion-fidelity is the next ship-gate blocker).
#
# Gates:
#   1. trial_1 outcome JSON written with completed_steps != -1
#   2. engine log shows '[rosbag2_storage]: Opened database ... for READ_ONLY'
#      (proves bag closed cleanly post-StopRecording — scoring-stoprecording-tf-fix held)
#   3. insertion_event_fired field is populated (value documented either way)
set -uo pipefail

REPO="/home/aaugus11/Documents/isaac-sim-mcp"
WRAPPER="$REPO/exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh"
TRIAL_OUTCOMES_DIR="$REPO/exts/aic-dt/docs/trial_outcomes"
OUTCOME_JSON="$TRIAL_OUTCOMES_DIR/trial_1_isaacsim.json"
ENGINE_LOG="/tmp/aic_engine_isaac.log"

mkdir -p "$TRIAL_OUTCOMES_DIR"

log() { echo "[test_cable_physics_descope] $*"; }
gate_pass() { log "  GATE $1 PASS: $2"; }
gate_fail() { log "  GATE $1 FAIL: $2"; FAIL=1; }

FAIL=0

# Preflight: sim must be up
if ! nc -z localhost 8768 2>/dev/null; then
    log "FATAL: Isaac Sim MCP not listening on 8768"
    exit 2
fi

# Fire trial_1 (trial_key is positional arg #1 per wrapper contract)
log "Firing trial_1 via wrapper..."
bash "$WRAPPER" trial_1 --output-json="$OUTCOME_JSON" 2>&1 | tail -30
WRAPPER_RC=${PIPESTATUS[0]}
log "Wrapper exit=$WRAPPER_RC"

# Gate 1: outcome JSON with completed_steps != -1
log "=== Gate 1: outcome JSON + completed_steps ==="
if [ ! -f "$OUTCOME_JSON" ]; then
    gate_fail 1 "outcome JSON missing at $OUTCOME_JSON"
else
    COMPLETED_STEPS=$(python3 -c "
import json
try:
    d = json.load(open('$OUTCOME_JSON'))
    print(d.get('completed_steps', '?'))
except Exception as e:
    print(f'PARSE-ERROR: {e}')
")
    if [ "$COMPLETED_STEPS" = "-1" ]; then
        gate_fail 1 "completed_steps=-1 (engine WaitForTfs timeout — scoring-stoprecording-tf-fix regressed)"
    elif [[ "$COMPLETED_STEPS" == "?"* ]] || [[ "$COMPLETED_STEPS" == "PARSE-ERROR"* ]]; then
        gate_fail 1 "completed_steps unparseable: $COMPLETED_STEPS"
    else
        gate_pass 1 "outcome JSON present, completed_steps=$COMPLETED_STEPS"
    fi
fi

# Gate 2: bag closed cleanly
log "=== Gate 2: engine bag closed cleanly ==="
if [ -f "$ENGINE_LOG" ] && grep -q "for READ_ONLY" "$ENGINE_LOG"; then
    gate_pass 2 "engine log shows '[rosbag2_storage]: Opened database ... for READ_ONLY' post-StopRecording"
else
    gate_fail 2 "no 'for READ_ONLY' in engine log (bag may not have closed cleanly)"
    grep -E "(WaitForTfs|Failed to stop|Timeout)" "$ENGINE_LOG" 2>/dev/null | head -5 | sed 's/^/    /'
fi

# Gate 3: insertion_event_fired field populated (true or false)
log "=== Gate 3: insertion_event_fired field documented ==="
if [ -f "$OUTCOME_JSON" ]; then
    INSERTION=$(python3 -c "
import json
try:
    d = json.load(open('$OUTCOME_JSON'))
    v = d.get('insertion_event_fired', '?')
    print(v)
except Exception as e:
    print(f'PARSE-ERROR: {e}')
")
    if [ "$INSERTION" = "True" ] || [ "$INSERTION" = "true" ] || [ "$INSERTION" = "False" ] || [ "$INSERTION" = "false" ]; then
        gate_pass 3 "insertion_event_fired=$INSERTION (value documented — either acceptable for this task)"
        if [ "$INSERTION" = "True" ] || [ "$INSERTION" = "true" ]; then
            log "  → M1 ship-gate likely satisfied; orchestrator next: parity-report-run, ship-paperwork"
        else
            log "  → motion-fidelity is the next blocker; orchestrator next: promote that task"
        fi
    else
        gate_fail 3 "insertion_event_fired unparseable or missing: $INSERTION"
    fi
fi

echo
if [ "$FAIL" -ne 0 ]; then
    log "RESULT: gates failed"
    exit 1
fi
log "RESULT: all 3 gates pass — cable-physics-descope-validate ready to close"
exit 0
