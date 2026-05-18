#!/bin/bash
# Reference: plans/prd.json::motion-fidelity-cheatcode-timing
#
# M1 ship-gate verifier. Two upstream fixes converge here:
#   (1) /aic_controller/change_target_mode service stub
#       (controller_loop.py) — without this aic_model wedges on
#       client.call() at the first handle_motion_update and CheatCode
#       never advances past pfrac=0.0.
#   (2) Path A cable-activation revert (extension.py) — cable_prim
#       SetActive(True) when attach_cable_to_gripper=True so the plug
#       rigid body exists and /scoring/insertion_event can fire on
#       PhysX contact.
#
# Gate 3 from the iter-1 draft (wall-clock loop-rate >= 1Hz on
# "execute loop" log lines) was DROPPED. That log timestamp measures
# aic_model's create_timer(1.0) sim-clock async timer, not CheatCode's
# real motion-command rate (sleep_for(0.05) ≈ 20Hz sim ≈ 11Hz wall).
# The pfrac line count below is the right signal for "policy is moving."
#
# Gates (ALL must pass):
#   1. trial_1 outcome JSON written + valid shape, completed_steps != -1
#      (engine ran the full task; not aborted by WaitForTfs timeout)
#   2. aic_model log shows >= 500 "pfrac:" CheatCode lines AND a final
#      "insert_cable() returned True" line (proves change_target_mode
#      stub held end-to-end, action thread executed the full insertion
#      ramp without wedging)
#   3. insertion_event_fired == true in outcome JSON (M1 SHIP GATE)
set -uo pipefail

REPO="/home/aaugus11/Documents/isaac-sim-mcp"
WRAPPER="$REPO/exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh"
TRIAL_OUTCOMES_DIR="$REPO/exts/aic-dt/docs/trial_outcomes"
OUTCOME_JSON="$TRIAL_OUTCOMES_DIR/trial_1_isaacsim.json"
ENGINE_LOG="/tmp/aic_engine_isaac.log"
MODEL_LOG="/tmp/aic_model_isaac.log"

mkdir -p "$TRIAL_OUTCOMES_DIR"

log() { echo "[test_motion_fidelity] $*"; }
gate_pass() { log "  GATE $1 PASS: $2"; }
gate_fail() { log "  GATE $1 FAIL: $2"; FAIL=1; }

FAIL=0

if ! nc -z localhost 8768 2>/dev/null; then
    log "FATAL: Isaac Sim MCP not listening on 8768"
    exit 2
fi

log "Firing trial_1 via wrapper..."
bash "$WRAPPER" trial_1 --output-json="$OUTCOME_JSON" 2>&1 | tail -30
WRAPPER_RC=${PIPESTATUS[0]}
log "Wrapper exit=$WRAPPER_RC"

# Gate 1: outcome JSON shape + completed_steps sanity
log "=== Gate 1: outcome JSON shape + completed_steps sanity ==="
if [ ! -f "$OUTCOME_JSON" ]; then
    gate_fail 1 "outcome JSON missing at $OUTCOME_JSON"
else
    python3 -c "
import json, sys
d = json.load(open('$OUTCOME_JSON'))
for k in ('trial_id','sim','insertion_event_fired','offlimit_contact_count','completed_steps','ts'):
    assert k in d, f'missing key {k}'
cs = d.get('completed_steps')
assert cs not in (-1, None), f'completed_steps={cs} (engine aborted; likely WaitForTfs timeout)'
sys.exit(0)
" && gate_pass 1 "outcome JSON valid; completed_steps != -1" || gate_fail 1 "outcome JSON shape/completed_steps invalid"
fi

# Gate 2: pfrac line count + insert_cable returned True
log "=== Gate 2: pfrac >= 500 AND insert_cable() returned True ==="
if [ ! -f "$MODEL_LOG" ]; then
    gate_fail 2 "model log missing at $MODEL_LOG"
else
    PFRAC_COUNT=$(grep -c "pfrac:" "$MODEL_LOG" 2>/dev/null || echo 0)
    log "  pfrac line count: $PFRAC_COUNT"
    INSERT_RET=$(grep -c "insert_cable() returned True" "$MODEL_LOG" 2>/dev/null || echo 0)
    log "  insert_cable() returned True line count: $INSERT_RET"
    if [ "$PFRAC_COUNT" -ge 500 ] && [ "$INSERT_RET" -ge 1 ]; then
        gate_pass 2 "$PFRAC_COUNT pfrac lines >= 500 AND insert_cable() returned True (CheatCode policy fully executed)"
    else
        gate_fail 2 "pfrac=$PFRAC_COUNT (need >=500) OR insert_cable_returned_true=$INSERT_RET (need >=1)"
    fi
fi

# Gate 3: insertion_event_fired == true (M1 SHIP GATE)
log "=== Gate 3: insertion_event_fired == true (M1 SHIP GATE) ==="
if [ -f "$OUTCOME_JSON" ]; then
    INSERTION=$(python3 -c "
import json
d = json.load(open('$OUTCOME_JSON'))
print(d.get('insertion_event_fired'))
")
    if [ "$INSERTION" = "True" ] || [ "$INSERTION" = "true" ]; then
        gate_pass 3 "insertion_event_fired=$INSERTION — M1 SHIP GATE PASSED"
    else
        gate_fail 3 "insertion_event_fired=$INSERTION (no PhysX contact fired between plug and port)"
    fi
fi

echo
if [ "$FAIL" -ne 0 ]; then
    log "RESULT: gates failed"
    exit 1
fi
log "RESULT: all 3 gates pass — motion-fidelity-cheatcode-timing ready to close"
exit 0
