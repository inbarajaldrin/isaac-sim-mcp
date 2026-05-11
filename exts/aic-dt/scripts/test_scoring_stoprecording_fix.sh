#!/usr/bin/env bash
# Reference: PRD task `scoring-stoprecording-tf-fix`.
#
# Verifies: engine's StopRecording() at trial_1 end successfully closes the bag.
# After this fix, /scoring/tf + /tf carry sim_clock timestamps matching the
# engine's use_sim_time=true, so ScoringTier2::WaitForTfs() no longer times
# out on its first iteration.
#
# Acceptance: for trial_1 (the trial loaded by the wrapper) — the engine MUST
# - open the bag for READ_ONLY (proof StopRecording succeeded; scoring tier_2/3
#   read the bag during ComputeScore)
# - NOT log 'Failed to stop recording' for trial_1
# - NOT log 'Timeout while waiting for transforms for scoring' for trial_1
#
# Subsequent trials (trial_2, trial_3) may still fail because the wrapper only
# loads one trial's CheatCode TF frames — that's a separate follow-up captured
# in progress.txt (trial-cycle-frames-loading) and out of scope here.
#
# Idempotent: cleans up its own /tmp test artifacts at exit.

set -eo pipefail

SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
TEST_JSON="/tmp/test_scoring_stoprecording_fix__trial_1.json"
TRIAL="trial_1"
ENGINE_LOG="/tmp/aic_engine_isaac.log"

trap 'rm -f "$TEST_JSON"' EXIT

# Build artifacts if missing.
if [ ! -x "$HOME/aic_humble_ws/install/aic_engine/lib/aic_engine/aic_engine" ]; then
    echo "[test] host aic_engine missing — building"
    bash "$SCRIPTS/build_aic_engine_host.sh"
fi
if [ ! -x "$HOME/aic_humble_ws/install/aic_model/lib/aic_model/aic_model" ]; then
    echo "[test] host aic_model missing — building"
    bash "$SCRIPTS/build_aic_model_host.sh"
fi

echo "[test] running trial $TRIAL fire end-to-end"
set +e
bash "$SCRIPTS/run_aic_engine_against_isaac_sim.sh" "$TRIAL" --output-json="$TEST_JSON"
set -e

# ---- gate 1: outcome JSON parseable ----
if [ ! -f "$TEST_JSON" ]; then
    echo "[test] FAIL: outcome JSON not written ($TEST_JSON missing)"
    exit 1
fi
python3 - "$TEST_JSON" <<'PY'
import json, sys
data = json.load(open(sys.argv[1]))
required = {"trial_id","sim","insertion_event_fired","offlimit_contact_count","completed_steps","ts"}
missing = required - data.keys()
if missing:
    raise SystemExit(f"FAIL: outcome JSON missing keys: {missing}")
if data["sim"] != "isaacsim":
    raise SystemExit(f"FAIL: sim != 'isaacsim'")
print(f"[test] gate 1 PASS: outcome JSON shape OK (trial={data['trial_id']})")
PY

if [ ! -f "$ENGINE_LOG" ]; then
    echo "[test] FAIL: engine log not at $ENGINE_LOG"
    exit 1
fi

# ---- gate 2: trial_1's bag closed cleanly (StopRecording succeeded) ----
# After StopRecording succeeds, ComputeScore reads the bag with READ_ONLY. So
# the presence of 'Opened database ... trial_1 ... for READ_ONLY' is a strong
# positive signal that the bag was closed cleanly.
trial_1_readonly=$(grep -cE "Opened database.*trial_1.*READ_ONLY" "$ENGINE_LOG" || true)
if [ "$trial_1_readonly" -lt 1 ]; then
    echo "[test] FAIL: no 'Opened database trial_1 ... READ_ONLY' line — bag wasn't closed cleanly"
    grep -nE "Opened database" "$ENGINE_LOG" || true
    exit 1
fi
echo "[test] gate 2 PASS: trial_1 bag closed and reopened READ_ONLY (StopRecording succeeded)"

# ---- gate 3: trial_1 had no scoring-tf timeout in its own window ----
# Anchor the search on the line index where trial_1 ends. The engine logs
# either 'Trial 'trial_1' failed' or 'Trial 'trial_1' completed' at the end of
# its block. We search for any 'Timeout while waiting for transforms' that
# fires BEFORE the trial_1 boundary line — those would be inside trial_1.
trial_1_end_line=$(grep -nE "Trial 'trial_1' (failed|completed)" "$ENGINE_LOG" | head -1 | cut -d: -f1 || true)
if [ -z "$trial_1_end_line" ]; then
    echo "[test] FAIL: couldn't find trial_1 end marker in engine log"
    exit 1
fi
timeouts_in_trial_1=$(head -n "$trial_1_end_line" "$ENGINE_LOG" | grep -cE "Timeout while waiting for transforms for scoring" || true)
stoprec_fails_in_trial_1=$(head -n "$trial_1_end_line" "$ENGINE_LOG" | grep -cE "Failed to stop recording" || true)
if [ "$timeouts_in_trial_1" -gt 0 ] || [ "$stoprec_fails_in_trial_1" -gt 0 ]; then
    echo "[test] FAIL: trial_1 had $timeouts_in_trial_1 WaitForTfs timeout(s) + $stoprec_fails_in_trial_1 StopRecording failure(s)"
    head -n "$trial_1_end_line" "$ENGINE_LOG" | grep -nE "Timeout while waiting|Failed to stop" || true
    exit 1
fi
echo "[test] gate 3 PASS: zero WaitForTfs timeouts in trial_1 window"

echo ""
echo "[test] PASS — trial_1 StopRecording WaitForTfs healthy under sim_clock"
echo "[test] note: trial_2/trial_3 may still fail (separate issue: wrapper only"
echo "       loads one trial's TF frames — see progress.txt 'trial-cycle' note)"
exit 0
