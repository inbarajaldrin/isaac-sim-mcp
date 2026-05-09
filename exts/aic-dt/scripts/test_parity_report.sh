#!/usr/bin/env bash
# test_parity_report.sh — gates parity_report.py against the PRD acceptance criteria.
#
# Asserts:
#  1. Empty trial_outcomes/ → graceful warning + exit 1
#  2. One mock trial_1__gazebo + one trial_1__isaacsim → 1-row markdown table, exit 0
#  3. Determinism: running twice with same input produces byte-identical output
#  4. --help prints the input file shape
#  5. Divergent outcomes → exit 1, "no" in Match column

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
SCRIPT="$REPO_ROOT/exts/aic-dt/scripts/parity_report.py"

TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

INPUT="$TMPDIR/trial_outcomes"
OUT1="$TMPDIR/report1.md"
OUT2="$TMPDIR/report2.md"
mkdir -p "$INPUT"

PASS=0
FAIL=0

assert_pass() { echo "PASS: $1"; PASS=$((PASS+1)); }
assert_fail() { echo "FAIL: $1"; FAIL=$((FAIL+1)); }

# --- 1. Empty input dir → exit 1 with graceful warning
set +e
python3 "$SCRIPT" --input-dir "$INPUT" --output "$OUT1" 2>"$TMPDIR/stderr1.txt"
rc=$?
set -e
if [[ $rc -eq 1 ]] && grep -q "no trial outcomes" "$TMPDIR/stderr1.txt"; then
  assert_pass "empty input → exit 1 with warning"
else
  assert_fail "empty input expected exit 1+warning, got rc=$rc, stderr=$(cat "$TMPDIR/stderr1.txt")"
fi

# --- 2. One matching trial pair → exit 0, 1-row table
cat > "$INPUT/trial_1__gazebo.json" <<'JSON'
{"trial_id":"trial_1","sim":"gazebo","insertion_event_fired":true,"offlimit_contact_count":0,"completed_steps":120,"ts":"2026-05-09T10:00:00Z"}
JSON
cat > "$INPUT/trial_1__isaacsim.json" <<'JSON'
{"trial_id":"trial_1","sim":"isaacsim","insertion_event_fired":true,"offlimit_contact_count":0,"completed_steps":120,"ts":"2026-05-09T10:01:00Z"}
JSON

set +e
python3 "$SCRIPT" --input-dir "$INPUT" --output "$OUT1" >"$TMPDIR/stdout2.txt"
rc=$?
set -e
if [[ $rc -eq 0 ]] && grep -q "trial_1" "$OUT1" && grep -q "all trials match" "$OUT1"; then
  assert_pass "matching pair → exit 0, table contains trial_1, summary green"
else
  assert_fail "matching pair expected exit 0+row+summary, got rc=$rc; report:"; cat "$OUT1" || true
fi

# --- 3. Determinism: byte-identical re-run
python3 "$SCRIPT" --input-dir "$INPUT" --output "$OUT2" >/dev/null
if cmp -s "$OUT1" "$OUT2"; then
  assert_pass "deterministic output (byte-identical re-run)"
else
  assert_fail "non-deterministic output; diff:"; diff "$OUT1" "$OUT2" || true
fi

# --- 4. --help exposes input file shape
if python3 "$SCRIPT" --help 2>&1 | grep -qi "trial_id"; then
  assert_pass "--help documents input file shape"
else
  assert_fail "--help missing input file shape docs"
fi

# --- 5. Divergent outcomes → exit 1
cat > "$INPUT/trial_1__isaacsim.json" <<'JSON'
{"trial_id":"trial_1","sim":"isaacsim","insertion_event_fired":false,"offlimit_contact_count":3,"completed_steps":120,"ts":"2026-05-09T10:01:00Z"}
JSON
set +e
python3 "$SCRIPT" --input-dir "$INPUT" --output "$OUT1" 2>/dev/null
rc=$?
set -e
if [[ $rc -eq 1 ]] && grep -q "| no |" "$OUT1"; then
  assert_pass "divergent outcomes → exit 1, Match=no"
else
  assert_fail "divergent expected exit 1+Match=no, got rc=$rc"; cat "$OUT1" || true
fi

echo "---"
echo "PASS=$PASS FAIL=$FAIL"
[[ $FAIL -eq 0 ]] && exit 0 || exit 1
