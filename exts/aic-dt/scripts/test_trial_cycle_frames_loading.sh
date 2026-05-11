#!/usr/bin/env bash
# Reference: PRD task `trial-cycle-frames-loading`.
#
# Verifies: the wrapper-generated single-trial config keeps the engine to
# exactly one trial per invocation. Fires trial_1, trial_2, trial_3 in
# sequence; for each, asserts:
#   - the engine log shows 'Total Trials: 1' (single-trial filter worked)
#   - the engine log shows 'Trial 1/1: <trial_key>' (right trial picked)
#   - the per-trial outcome JSON is parseable with the standard 6 keys
# Cleans up /tmp artifacts on exit.

set -eo pipefail

SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
ENGINE_LOG="/tmp/aic_engine_isaac.log"
ARTIFACTS=()

cleanup() {
    for a in "${ARTIFACTS[@]}"; do rm -f "$a"; done
}
trap cleanup EXIT

# Pre-build artifacts if missing.
if [ ! -x "$HOME/aic_humble_ws/install/aic_engine/lib/aic_engine/aic_engine" ]; then
    bash "$SCRIPTS/build_aic_engine_host.sh"
fi
if [ ! -x "$HOME/aic_humble_ws/install/aic_model/lib/aic_model/aic_model" ]; then
    bash "$SCRIPTS/build_aic_model_host.sh"
fi

for trial in trial_1 trial_2 trial_3; do
    echo
    echo "[test] === firing $trial ==="
    JSON="/tmp/test_trial_cycle__${trial}.json"
    ARTIFACTS+=("$JSON" "/tmp/aic_engine_single_trial_${trial}.yaml")
    set +e
    bash "$SCRIPTS/run_aic_engine_against_isaac_sim.sh" "$trial" --output-json="$JSON" >/dev/null 2>&1
    set -e

    if [ ! -f "$JSON" ]; then
        echo "[test] FAIL ($trial): outcome JSON not written"
        exit 1
    fi
    python3 - "$JSON" "$trial" <<'PY'
import json, sys
path, expected_trial = sys.argv[1:3]
data = json.load(open(path))
required = {"trial_id","sim","insertion_event_fired","offlimit_contact_count","completed_steps","ts"}
missing = required - data.keys()
if missing:
    raise SystemExit(f"FAIL: missing keys {missing}")
if data["trial_id"] != expected_trial:
    raise SystemExit(f"FAIL: trial_id {data['trial_id']} != expected {expected_trial}")
if data["sim"] != "isaacsim":
    raise SystemExit(f"FAIL: sim != isaacsim")
print(f"[test]   outcome JSON OK: {data['trial_id']} insertion={data['insertion_event_fired']} offlimit={data['offlimit_contact_count']}")
PY
    if ! grep -qE "Total Trials: 1" "$ENGINE_LOG"; then
        echo "[test] FAIL ($trial): engine log doesn't show 'Total Trials: 1' — single-trial filter broken"
        grep -E "Total Trials" "$ENGINE_LOG" || true
        exit 1
    fi
    if ! grep -qE "Trial 1/1: ${trial}\b" "$ENGINE_LOG"; then
        echo "[test] FAIL ($trial): engine log doesn't show 'Trial 1/1: ${trial}' — wrong trial picked"
        grep -E "Trial [0-9]+/[0-9]+:" "$ENGINE_LOG" || true
        exit 1
    fi
    echo "[test]   engine log OK: 'Total Trials: 1' + 'Trial 1/1: $trial'"
done

echo
echo "[test] PASS — trial_1 + trial_2 + trial_3 all fire under single-trial filter"
exit 0
