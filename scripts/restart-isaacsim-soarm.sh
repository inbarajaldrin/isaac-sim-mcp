#!/usr/bin/env bash
# Reference: hand-rolled for soarm101-dt digital-twin workflow.
# Kills any running Isaac Sim, touches the soarm101-dt extension.py, then
# launches Isaac Sim with the soarm101-dt extension enabled — and ONLY
# that extension.
#
# Why the per-extension temp ext-folder instead of pointing Kit at
# exts/ directly: Kit auto-enables EVERY extension it discovers under
# --ext-folder regardless of the --enable flag. There is no working
# --disable counterpart (--/exts/[id]/enabled=false is silently ignored
# in Kit 5.0.x). Pointing Kit at a folder that contains only ONE
# extension symlink is the durable workaround. Avoids the documented
# cross-extension stdout contamination hazard (CLAUDE.md "Gotchas")
# that manifests as Isaac Sim hung at ~2000% CPU with both MCP sockets
# bound to the same PID.
#
# Usage:
#   ./scripts/restart-isaacsim-soarm.sh           # kill + touch + launch
#   ./scripts/restart-isaacsim-soarm.sh --tail    # follow /tmp/isaacsim.log after launch
#   ./scripts/restart-isaacsim-soarm.sh --no-touch # skip touch
#
# Exit codes:
#   0 = launched and MCP socket (8767) ready
#   1 = lifecycle script or extension entrypoint missing
#   2 = launch failed (Isaac Sim never came up)

set -e

EXT_ID="soarm101-dt"
EXT_DIR="$HOME/Documents/isaac-sim-mcp/exts/${EXT_ID}"
EXT_PY="${EXT_DIR}/so_arm101_dt/extension.py"
ISOLATED_EXT_FOLDER="/tmp/isaacsim-ext-${EXT_ID}"
LAUNCH_SCRIPT="$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh"
ISAACSIM_BIN="$HOME/env_isaaclab/bin/isaacsim"
LOG="/tmp/isaacsim.log"
PIDFILE="/tmp/isaacsim.pid"
PORT=8767
SIBLING_PORTS=(8766 8768)  # ur5e-dt, aic-dt — should NOT be bound after launch

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
  exit 1
fi

# 1. Graceful shutdown via the lifecycle helper.
log "stopping Isaac Sim (graceful) ..."
bash "$LAUNCH_SCRIPT" close || log "  (close returned non-zero — may already be stopped)"

# 2. Touch the extension entrypoint to bust any .pyc bytecode cache.
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
