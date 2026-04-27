#!/usr/bin/env bash
# Restart aruco_camera_localizer for the SO-ARM101 sim wrist camera.
# Always kills any running instance first (SIGINT → wait → SIGKILL fallback)
# before relaunching, so the cv2 preview window is fresh against the
# current viewport state.
#
# Defaults match the real-mode pipeline:
#   camera input:   /wrist_camera_rgb_sim
#   robot config:   so_arm101 (ee_pose, intrinsics from robot_config.yaml)
#   drop poses:     /drop_poses_real
#   aruco poses:    /aruco_poses_real
#
# Usage:
#   scripts/restart_aruco_localizer.sh                   # foreground (Ctrl+C to stop)
#   scripts/restart_aruco_localizer.sh --bg              # background, log to /tmp/loc_aruco.log
#   scripts/restart_aruco_localizer.sh --suppress        # no console prints (still draws cv2 window)
#   scripts/restart_aruco_localizer.sh --camera /workspace_camera_rgb_sim
#   scripts/restart_aruco_localizer.sh --no-drop         # skip --drop, publish raw aruco_poses only

set -eo pipefail
# (Don't use `-u` — /opt/ros/humble/setup.bash references unbound AMENT_* vars.)

CAMERA_TOPIC="/wrist_camera_rgb_sim"
ROBOT="so_arm101"
DROP_TOPIC="/drop_poses_real"
ARUCO_TOPIC="/aruco_poses_real"
RUN_BG=0
SUPPRESS=0
USE_DROP=1

while [ $# -gt 0 ]; do
  case "$1" in
    --bg) RUN_BG=1; shift ;;
    --suppress) SUPPRESS=1; shift ;;
    --no-drop) USE_DROP=0; shift ;;
    --camera) CAMERA_TOPIC="$2"; shift 2 ;;
    --robot) ROBOT="$2"; shift 2 ;;
    --drop-topic) DROP_TOPIC="$2"; shift 2 ;;
    --aruco-topic) ARUCO_TOPIC="$2"; shift 2 ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//' | head -20
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

# Pre-kill any stale instance — the cv2 window from the old process won't
# refresh on its own even if the topic is fresh, because cv2.imshow caches the
# last received frame and only updates when imshow is called again with a new
# one. SIGINT first (graceful, lets cv2 close cleanly), wait, then SIGKILL.
existing=$(pgrep -f "localize_aruco" || true)
if [ -n "$existing" ]; then
  echo "[restart] existing localize_aruco PIDs: $existing — sending SIGINT..."
  pkill -SIGINT -f "localize_aruco" 2>/dev/null || true
  for i in 1 2 3 4 5; do
    sleep 1
    pgrep -f "localize_aruco" >/dev/null || break
  done
  if pgrep -f "localize_aruco" >/dev/null; then
    echo "[restart] survived SIGINT — escalating to SIGKILL..."
    pkill -9 -f "localize_aruco" 2>/dev/null || true
    sleep 1
  fi
  if pgrep -f "localize_aruco" >/dev/null; then
    echo "[restart] ERROR: process still running after SIGKILL. Aborting."
    pgrep -af "localize_aruco"
    exit 1
  fi
  echo "[restart] cleared."
else
  echo "[restart] no existing localize_aruco running."
fi

source /opt/ros/humble/setup.bash
source /home/aaugus11/Desktop/ros2_ws/install/setup.bash

ARGS=(--camera-topic "$CAMERA_TOPIC" --robot "$ROBOT")
[ "$SUPPRESS" -eq 1 ] && ARGS+=(--suppress-prints)
[ "$USE_DROP" -eq 1 ] && ARGS+=(--drop)
ARGS+=(--ros-args
       -p "drop_poses_topic:=${DROP_TOPIC}"
       -p "aruco_poses_topic:=${ARUCO_TOPIC}")

echo "[restart] running: ros2 run aruco_camera_localizer localize_aruco ${ARGS[*]}"

if [ "$RUN_BG" -eq 1 ]; then
  LOG=/tmp/loc_aruco.log
  : > "$LOG"
  ros2 run aruco_camera_localizer localize_aruco "${ARGS[@]}" > "$LOG" 2>&1 &
  PID=$!
  disown
  echo "[restart] launched bg, PID=$PID, log=$LOG"
  sleep 4
  if kill -0 "$PID" 2>/dev/null; then
    echo "[restart] still running ✓"
  else
    echo "[restart] DIED, last log lines:"
    tail -20 "$LOG"
    exit 1
  fi
else
  exec ros2 run aruco_camera_localizer localize_aruco "${ARGS[@]}"
fi
