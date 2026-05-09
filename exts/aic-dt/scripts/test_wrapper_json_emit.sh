#!/bin/bash
# Reference: plans/prd.json wrapper-json-emit task
# Verify run_aic_engine_against_isaac_sim.sh --output-json contract:
#   1. Usage text mentions --output-json
#   2. With --output-json: JSON file written with all 6 required keys + correct
#      types, even when the wrapper bails on missing host preconditions.
#      Forced via AIC_WS=/nonexistent_path_zzz so the host engine binary check
#      fails predictably without needing sim/docker/network.
#   3. Without --output-json: no JSON file written (backwards compatible).
#   4. JSON shape matches parity_report.py REQUIRED_KEYS + sim='isaacsim'.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WRAPPER="$SCRIPT_DIR/run_aic_engine_against_isaac_sim.sh"
OUTPUT="/tmp/test_wrapper_json_emit_$$.json"
trap 'rm -f "$OUTPUT"' EXIT

if [ ! -f "$WRAPPER" ]; then
    echo "FAIL: wrapper not found at $WRAPPER"
    exit 1
fi

# --- Test 1: usage / help mentions --output-json ---
if ! bash "$WRAPPER" 2>&1 | grep -q -- '--output-json'; then
    echo "FAIL: usage text does not mention --output-json"
    bash "$WRAPPER" 2>&1 | sed 's/^/  /'
    exit 1
fi
echo "PASS Test 1 — usage mentions --output-json"

# --- Test 2: with --output-json + forced precond fail, JSON IS written via trap ---
# AIC_WS=/nonexistent makes the host-engine binary check fail at line ~86;
# wrapper exits non-zero, but the EXIT trap must still flush the outcome JSON.
rm -f "$OUTPUT"
AIC_WS=/nonexistent_path_zzz \
    bash "$WRAPPER" trial_test_stub --output-json "$OUTPUT" >/dev/null 2>&1 || true
if [ ! -f "$OUTPUT" ]; then
    echo "FAIL: $OUTPUT not written despite --output-json flag (EXIT trap missed)"
    exit 1
fi
python3 - "$OUTPUT" <<'PY'
import json, sys, re
with open(sys.argv[1]) as f:
    j = json.load(f)
required = {'trial_id', 'sim', 'insertion_event_fired', 'offlimit_contact_count', 'completed_steps', 'ts'}
missing = required - set(j.keys())
assert not missing, f'missing keys: {missing}; got {sorted(j.keys())}'
assert j['sim'] == 'isaacsim', f'wrong sim: {j["sim"]!r}'
assert j['trial_id'] == 'trial_test_stub', f'wrong trial_id: {j["trial_id"]!r}'
assert isinstance(j['insertion_event_fired'], bool), f'insertion_event_fired must be bool, got {type(j["insertion_event_fired"]).__name__}'
assert isinstance(j['offlimit_contact_count'], int), f'offlimit_contact_count must be int, got {type(j["offlimit_contact_count"]).__name__}'
assert isinstance(j['completed_steps'], int), f'completed_steps must be int, got {type(j["completed_steps"]).__name__}'
assert isinstance(j['ts'], str), f'ts must be str, got {type(j["ts"]).__name__}'
assert re.match(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}', j['ts']), f'ts not ISO 8601: {j["ts"]!r}'
PY
echo "PASS Test 2 — JSON written with all 6 keys + correct types on early-exit failure"

# --- Test 3: without --output-json, no JSON file written (backwards compat) ---
rm -f "$OUTPUT"
AIC_WS=/nonexistent_path_zzz \
    bash "$WRAPPER" trial_test_stub >/dev/null 2>&1 || true
if [ -f "$OUTPUT" ]; then
    echo "FAIL: JSON file written despite --output-json not provided"
    exit 1
fi
echo "PASS Test 3 — no JSON written without --output-json flag"

# --- Test 4: trial_id echoes wrapper arg; sim is exactly 'isaacsim' ---
# Confirms the JSON groups correctly under parity_report.py's load_outcomes()
rm -f "$OUTPUT"
AIC_WS=/nonexistent_path_zzz \
    bash "$WRAPPER" trial_xyz --output-json "$OUTPUT" >/dev/null 2>&1 || true
python3 - "$OUTPUT" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    j = json.load(f)
# parity_report.py groups by (trial_id, sim) — both must be canonical.
assert j['sim'] == 'isaacsim', f'sim must be exactly "isaacsim" for parity grouping (got {j["sim"]!r})'
assert j['trial_id'] == 'trial_xyz', f'trial_id must echo wrapper arg (got {j["trial_id"]!r})'
PY
echo "PASS Test 4 — JSON shape matches parity-report-tool input contract"

echo ""
echo "ALL TESTS PASS — wrapper-json-emit acceptance gates green"
