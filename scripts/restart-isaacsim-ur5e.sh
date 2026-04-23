#!/usr/bin/env bash
# Reference: hand-rolled for ur5e-dt digital-twin workflow.
# Mirrors restart-isaacsim-soarm.sh but for the ur5e-dt extension.
# See that script for the full rationale on the per-extension isolated
# ext-folder approach (Kit auto-enables every extension under
# --ext-folder, so we point Kit at a folder that contains only the one
# we want).
#
# Usage:
#   ./scripts/restart-isaacsim-ur5e.sh           # kill + touch + launch
#   ./scripts/restart-isaacsim-ur5e.sh --tail    # follow /tmp/isaacsim.log after launch
#   ./scripts/restart-isaacsim-ur5e.sh --no-touch # skip touch
#
# Exit codes:
#   0 = launched and MCP socket (8766) ready
#   1 = lifecycle script or extension entrypoint missing
#   2 = launch failed (Isaac Sim never came up)
#
# Note: ur5e-dt lives on the `main` branch of this repo, but the dir
# coexists on every branch under exts/, so this script works regardless
# of current branch.

set -e

EXT_ID="ur5e-dt"
EXT_DIR="$HOME/Documents/isaac-sim-mcp/exts/${EXT_ID}"
EXT_PY="${EXT_DIR}/ur5e_dt/extension.py"
ISOLATED_EXT_FOLDER="/tmp/isaacsim-ext-${EXT_ID}"
LAUNCH_SCRIPT="$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh"
ISAACSIM_BIN="$HOME/env_isaaclab/bin/isaacsim"
LOG="/tmp/isaacsim.log"
PIDFILE="/tmp/isaacsim.pid"
PORT=8766
SIBLING_PORTS=(8767 8768)  # soarm101-dt, aic-dt — should NOT be bound after launch

TAIL=0
DO_TOUCH=1
for arg in "$@"; do
  case "$arg" in
    --tail) TAIL=1 ;;
    --no-touch) DO_TOUCH=0 ;;
    *) echo "unknown arg: $arg" >&2; exit 64 ;;
  esac
done

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }

if [[ ! -x "$LAUNCH_SCRIPT" ]]; then
  echo "lifecycle script not found or not executable: $LAUNCH_SCRIPT" >&2
  exit 1
fi
if [[ ! -d "$EXT_DIR" ]]; then
  echo "extension dir not found: $EXT_DIR" >&2
  exit 1
fi
if [[ ! -f "$EXT_PY" ]]; then
  echo "extension entrypoint not found: $EXT_PY" >&2
  echo "  (you may be on a branch that doesn't include ur5e-dt)" >&2
  exit 1
fi

# 1. Graceful shutdown via the lifecycle helper.
log "stopping Isaac Sim (graceful) ..."
bash "$LAUNCH_SCRIPT" close || log "  (close returned non-zero — may already be stopped)"

# 2. Touch the extension entrypoint.
if [[ "$DO_TOUCH" == "1" ]]; then
  log "touching ${EXT_PY##*/}"
  touch "$EXT_PY"
fi

# 3. Build the isolated single-extension folder.
rm -rf "$ISOLATED_EXT_FOLDER"
mkdir -p "$ISOLATED_EXT_FOLDER"
ln -s "$EXT_DIR" "${ISOLATED_EXT_FOLDER}/${EXT_ID}"
log "isolated ext-folder: ${ISOLATED_EXT_FOLDER} -> only ${EXT_ID}"

# 4. Launch Isaac Sim against the isolated folder.
log "launching Isaac Sim with ${EXT_ID} (socket :${PORT}) ..."
export DISPLAY="${DISPLAY:-:0}"
nohup "$ISAACSIM_BIN" \
  --ext-folder "$ISOLATED_EXT_FOLDER" \
  --enable "$EXT_ID" \
  > "$LOG" 2>&1 &
echo "$!" > "$PIDFILE"
log "  PID $(cat "$PIDFILE") — log: $LOG"

# 5. Block until the MCP socket binds.
if ! bash "$LAUNCH_SCRIPT" wait 120 "$PORT"; then
  echo "Isaac Sim socket on :$PORT never came up — check $LOG" >&2
  exit 2
fi
log "Isaac Sim ready on socket :${PORT}"

# 6. Sanity: confirm sibling sockets are NOT bound.
for sp in "${SIBLING_PORTS[@]}"; do
  if ss -tlnp 2>/dev/null | grep -q ":${sp} "; then
    echo "WARNING: sibling socket :${sp} is bound — isolation failed (check $ISOLATED_EXT_FOLDER)" >&2
  fi
done
log "isolation OK (siblings :${SIBLING_PORTS[*]} are clear)"

if [[ "$TAIL" == "1" ]]; then
  log "tailing /tmp/isaacsim.log (Ctrl-C to stop) ..."
  exec tail -f /tmp/isaacsim.log
fi
