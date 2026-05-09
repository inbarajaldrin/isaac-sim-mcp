#!/usr/bin/env bash
# Reference: ~/Documents/aic/tools/capture_trial.sh (canonical Gazebo capture)
#
# Wrapper that runs CheatCode against Gazebo for ONE trial (sample_config.yaml
# trial_1|trial_2|trial_3) and emits a per-trial outcome JSON in the schema
# consumed by exts/aic-dt/scripts/parity_report.py.
#
# Sister script to run_aic_engine_against_isaac_sim.sh — same --output-json flag,
# same JSON shape, distinguishable only by `sim:'gazebo'` vs `sim:'isaacsim'`.
#
# Usage:
#   bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_1 [--output-json <path>]
#
# Mechanism:
#   1. Invokes `cd ~/Documents/aic && tools/capture_trial.sh --scene trial_X` —
#      AIC's canonical per-trial capture pipeline. It launches aic_eval (Gazebo +
#      engine + ros_gz_bridge + scoring) and aic_model (CheatCode policy) in
#      Docker, runs ONE trial, writes ~/Documents/aic/data/recordings/<trial_X>_<ts>/.
#   2. Locates the recording dir created by step 1 (most-recent matching scene).
#   3. Runs extract_gazebo_baseline.py against the recording dir to convert
#      metadata.json + eval.log into the parity-report JSON shape.
#
# Cross-repo discipline: this script INVOKES tools in ~/Documents/aic but does
# not edit any file there. Recording outputs land in ~/Documents/aic/data/
# recordings/ which is AIC's working dir for capture artifacts (not source).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
AIC_REPO="${AIC_REPO:-$HOME/Documents/aic}"
RECORDINGS_ROOT="${AIC_REPO}/data/recordings"

usage() {
  cat <<EOF
Usage: $(basename "$0") <trial_id> [--output-json <path>] [--reuse-latest]

Args:
  trial_id              trial_1 | trial_2 | trial_3 (canonical sample_config.yaml scenes)
  --output-json PATH    write parity-report JSON here
                        default: \$REPO_ROOT/exts/aic-dt/docs/trial_outcomes/<trial_id>__gazebo.json
  --reuse-latest        skip the docker capture step; just re-extract from the
                        most-recent ~/Documents/aic/data/recordings/<trial_id>_*
                        dir. Useful when a fresh capture has already been done
                        and you only want to regenerate the JSON.

Env overrides:
  AIC_REPO              defaults to \$HOME/Documents/aic
EOF
}

if [[ $# -lt 1 ]]; then usage; exit 2; fi

TRIAL_ID="$1"; shift || true
case "$TRIAL_ID" in
  trial_1|trial_2|trial_3) ;;
  -h|--help) usage; exit 0 ;;
  *) echo "ERROR: trial_id must be trial_1|trial_2|trial_3 (got '$TRIAL_ID')"; usage; exit 2 ;;
esac

OUTPUT_JSON="${REPO_ROOT}/exts/aic-dt/docs/trial_outcomes/${TRIAL_ID}__gazebo.json"
REUSE_LATEST=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-json) OUTPUT_JSON="$2"; shift 2 ;;
    --reuse-latest) REUSE_LATEST=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown arg '$1'"; usage; exit 2 ;;
  esac
done

if [[ ! -d "$AIC_REPO" ]]; then
  echo "ERROR: AIC_REPO not found: $AIC_REPO" >&2
  exit 1
fi
if [[ ! -x "$AIC_REPO/tools/capture_trial.sh" ]]; then
  echo "ERROR: $AIC_REPO/tools/capture_trial.sh not executable" >&2
  exit 1
fi

# Step 1 (optional): run the capture
if [[ $REUSE_LATEST -eq 0 ]]; then
  echo "[1/2] running AIC capture pipeline for $TRIAL_ID (this takes ~3-5 min)..."
  (
    cd "$AIC_REPO"
    bash tools/capture_trial.sh --scene "$TRIAL_ID"
  )
  capture_exit=$?
  if [[ $capture_exit -ne 0 ]]; then
    echo "WARN: capture_trial.sh exited $capture_exit; will still attempt extraction" >&2
  fi
else
  echo "[1/2] --reuse-latest: skipping capture, locating most-recent recording..."
fi

# Step 2: locate most-recent recording dir matching trial_id
shopt -s nullglob
mapfile -t CANDIDATES < <(ls -1dt "${RECORDINGS_ROOT}/${TRIAL_ID}"_* 2>/dev/null)
if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "ERROR: no recordings found for $TRIAL_ID under $RECORDINGS_ROOT" >&2
  exit 1
fi
RECORDING_DIR="${CANDIDATES[0]}"
echo "    recording dir: $RECORDING_DIR"

echo "[2/2] extracting parity-report JSON..."
python3 "${REPO_ROOT}/exts/aic-dt/scripts/extract_gazebo_baseline.py" \
  --recording-dir "$RECORDING_DIR" \
  --trial-id "$TRIAL_ID" \
  --output-json "$OUTPUT_JSON"

echo ""
echo "=== done ==="
echo "JSON: $OUTPUT_JSON"
cat "$OUTPUT_JSON"
