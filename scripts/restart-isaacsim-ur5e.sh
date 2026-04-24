#!/usr/bin/env bash
# Reference: hand-rolled for ur5e-dt digital-twin workflow.
# Mirrors restart-isaacsim-soarm.sh but for the ur5e-dt extension.
# See that script for the full rationale on the per-extension isolated
# ext-folder approach (Kit auto-enables every extension under
# --ext-folder, so we point Kit at a folder that contains only the one
# we want).
#
# Usage:
#   ./scripts/restart-isaacsim-ur5e.sh             # kill + touch + hardened launch
#   ./scripts/restart-isaacsim-ur5e.sh --tail      # follow /tmp/isaacsim.log after launch
#   ./scripts/restart-isaacsim-ur5e.sh --no-touch  # skip touch
#   ./scripts/restart-isaacsim-ur5e.sh --reset-user # ignore Kit user.config for a clean launch
#   ./scripts/restart-isaacsim-ur5e.sh --capture-on-play # opt back into Replicator captureOnPlay
#   ./scripts/restart-isaacsim-ur5e.sh --service   # launch under a transient systemd --user unit
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
RESET_USER=0
CAPTURE_ON_PLAY=0
SERVICE_MODE=0
SERVICE_NAME="isaacsim-${EXT_ID}"
for arg in "$@"; do
  case "$arg" in
    --tail) TAIL=1 ;;
    --no-touch) DO_TOUCH=0 ;;
    --reset-user) RESET_USER=1 ;;
    --capture-on-play) CAPTURE_ON_PLAY=1 ;;
    --service) SERVICE_MODE=1 ;;
    *) echo "unknown arg: $arg" >&2; exit 64 ;;
  esac
done

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }

source_if_exists() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    # shellcheck disable=SC1090
    source "$setup_file"
    log "sourced ROS env: $setup_file"
  fi
}

append_ld_library_path() {
  local candidate="$1"
  if [[ -z "$candidate" || ! -d "$candidate" ]]; then
    return
  fi
  if [[ ":${LD_LIBRARY_PATH:-}:" != *":${candidate}:"* ]]; then
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${candidate}"
  fi
}

append_systemd_env_arg() {
  local -n ref="$1"
  local name="$2"
  if [[ -n "${!name:-}" ]]; then
    ref+=("--setenv=${name}=${!name}")
  fi
}

wait_for_service_port() {
  local service_name="$1"
  local timeout_secs="$2"
  local port="$3"
  local deadline=$((SECONDS + timeout_secs))

  log "waiting up to ${timeout_secs}s for ${service_name}.service to bind :${port} ..."
  while (( SECONDS < deadline )); do
    if ss -tln 2>/dev/null | grep -q ":${port} "; then
      return 0
    fi
    if ! systemctl --user is-active --quiet "${service_name}.service"; then
      return 1
    fi
    sleep 1
  done
  return 1
}

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

# 4. Source the same ROS environment this workstation uses interactively.
source_if_exists "/opt/ros/humble/setup.bash"
source_if_exists "$HOME/Desktop/ros2_ws/install/setup.bash"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/Documents/ros-mcp-server/cyclonedds_local.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
append_ld_library_path "$HOME/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"
log "ROS_DISTRO=${ROS_DISTRO} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

# 5. Launch Isaac Sim against the isolated folder.
log "launching Isaac Sim with ${EXT_ID} (socket :${PORT}) ..."
export DISPLAY="${DISPLAY:-:0}"
export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"

# Keep Isaac on the NVIDIA Vulkan ICD and disable async throttling, which
# is a known instability path for Replicator/SDG-heavy sessions.
SAFE_GPU_ARGS=(
  "--/exts/isaacsim.core.throttling/enable_async=false"
)
if [[ "$CAPTURE_ON_PLAY" == "0" ]]; then
  SAFE_GPU_ARGS+=("--/persistent/omni/replicator/captureOnPlay=0")
fi
if [[ "$RESET_USER" == "1" ]]; then
  SAFE_GPU_ARGS+=("--reset-user")
fi
LAUNCH_CMD=(
  "$ISAACSIM_BIN"
  --ext-folder "$ISOLATED_EXT_FOLDER"
  --enable "$EXT_ID"
  "${SAFE_GPU_ARGS[@]}"
)
log "forcing Vulkan ICD: ${VK_ICD_FILENAMES}"
log "forcing GLX vendor: ${__GLX_VENDOR_LIBRARY_NAME}"
log "launch arg: /exts/isaacsim.core.throttling/enable_async=false"
if [[ "$CAPTURE_ON_PLAY" == "0" ]]; then
  log "launch arg: /persistent/omni/replicator/captureOnPlay=0"
fi
if [[ "$RESET_USER" == "1" ]]; then
  log "launch arg: --reset-user"
fi
if [[ "$SERVICE_MODE" == "1" ]]; then
  SYSTEMD_RUN_ARGS=(--user --unit="$SERVICE_NAME" --collect --same-dir)
  for env_name in \
    HOME PATH PYTHONPATH LD_LIBRARY_PATH \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
    ROS_DISTRO RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_DOMAIN_ID ROS_LOCALHOST_ONLY \
    DISPLAY VK_ICD_FILENAMES __GLX_VENDOR_LIBRARY_NAME; do
    append_systemd_env_arg SYSTEMD_RUN_ARGS "$env_name"
  done
  systemctl --user stop "${SERVICE_NAME}.service" >/dev/null 2>&1 || true
  log "launching transient user service: ${SERVICE_NAME}.service"
  systemd-run "${SYSTEMD_RUN_ARGS[@]}" "${LAUNCH_CMD[@]}"
  printf '%s\n' "${SERVICE_NAME}.service" > "$PIDFILE"
  log "  unit ${SERVICE_NAME}.service"
else
  nohup "${LAUNCH_CMD[@]}" > "$LOG" 2>&1 &
  echo "$!" > "$PIDFILE"
  log "  PID $(cat "$PIDFILE") — log: $LOG"
fi

# 6. Block until the MCP socket binds.
if [[ "$SERVICE_MODE" == "1" ]]; then
  if ! wait_for_service_port "$SERVICE_NAME" 120 "$PORT"; then
    echo "Isaac Sim service ${SERVICE_NAME}.service never bound :$PORT — check journalctl --user -u ${SERVICE_NAME}.service -n 200 --no-pager" >&2
    exit 2
  fi
else
  if ! bash "$LAUNCH_SCRIPT" wait 120 "$PORT"; then
    echo "Isaac Sim socket on :$PORT never came up — check $LOG" >&2
    exit 2
  fi
fi
log "Isaac Sim ready on socket :${PORT}"

# 7. Sanity: confirm sibling sockets are NOT bound.
for sp in "${SIBLING_PORTS[@]}"; do
  if ss -tlnp 2>/dev/null | grep -q ":${sp} "; then
    echo "WARNING: sibling socket :${sp} is bound — isolation failed (check $ISOLATED_EXT_FOLDER)" >&2
  fi
done
log "isolation OK (siblings :${SIBLING_PORTS[*]} are clear)"

if [[ "$TAIL" == "1" ]]; then
  if [[ "$SERVICE_MODE" == "1" ]]; then
    log "tailing journal for ${SERVICE_NAME}.service (Ctrl-C to stop) ..."
    exec journalctl --user -u "${SERVICE_NAME}.service" -f
  fi
  log "tailing /tmp/isaacsim.log (Ctrl-C to stop) ..."
  exec tail -f /tmp/isaacsim.log
fi
