#!/usr/bin/env bash
# Restart localize_yoloe (YOLOE detector) for the SO-ARM101 sim wrist camera.
#
# Always kills any running instance first (SIGINT → wait → SIGTERM fallback)
# before relaunching, so the cv2 preview window is fresh against the current
# viewport state. Mirrors restart_aruco_localizer.sh's pattern.
#
# Defaults match the current real-mode pipeline:
#   camera input:   /wrist_camera_rgb_sim
#   prompts:        loaded from robot_config.yaml (so_arm101.detection.yolo)
#                   — currently "red object" "blue object" "green object" mapped
#                   to short color names. Override with --prompts if needed.
#   confidence:     loaded from robot_config.yaml (currently 0.25). Override
#                   with --conf.
#   topics:         /objects_poses_real, /objects_bbox_real
#   cv2 preview:    enabled by default  (use --headless to disable)
#
# Usage:
#   scripts/restart_yoloe.sh                        # foreground (cv2 window, Ctrl+C to stop)
#   scripts/restart_yoloe.sh --headless             # no cv2 window, foreground
#   scripts/restart_yoloe.sh --bg                   # background, log to /tmp/loc_yoloe.log
#   scripts/restart_yoloe.sh --camera /workspace_camera_rgb_sim
#   scripts/restart_yoloe.sh --conf 0.4             # tighten confidence
#
# RUN FROM YOUR OWN TERMINAL — Bash tool subshells lack reliable D-bus / XDG
# session context for cv2.imshow. Same pattern as restart_aruco_localizer.sh.

set -eo pipefail
# (Don't use `-u` — /opt/ros/humble/setup.bash references unbound AMENT_* vars.)

CAMERA_TOPIC="/wrist_camera_rgb_sim"
CONF=""
HEADLESS=""
RUN_BG=0
# Empty PROMPTS = let robot_config.yaml drive the vocabulary. Override with
# --prompts "color1 color2 ..." for ad-hoc testing without editing the YAML.
PROMPTS=""

while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS="--headless"; shift ;;
    --bg) RUN_BG=1; shift ;;
    --camera) CAMERA_TOPIC="$2"; shift 2 ;;
    --conf) CONF="$2"; shift 2 ;;
    --prompts) PROMPTS="$2"; shift 2 ;;   # space-separated string
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//' | head -28
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

# Pre-kill stale instance — cv2 windows from old processes don't refresh on
# their own. SIGINT first (graceful, lets cv2 close), wait, then SIGTERM.
existing=$(pgrep -f "localize_yoloe" || true)
if [ -n "$existing" ]; then
  echo "[restart] existing localize_yoloe PIDs: $existing — sending SIGINT..."
  pkill -SIGINT -f "localize_yoloe" 2>/dev/null || true
  for i in 1 2 3 4 5; do
    sleep 1
    pgrep -f "localize_yoloe" >/dev/null || break
  done
  if pgrep -f "localize_yoloe" >/dev/null; then
    echo "[restart] still alive after 5s — sending SIGTERM..."
    pkill -SIGTERM -f "localize_yoloe" 2>/dev/null || true
    sleep 2
  fi
fi

source /opt/ros/humble/setup.bash
source ~/Desktop/ros2_ws/install/setup.bash

# Build arg array. Skip --yolo-prompts and --yolo-conf when not provided so
# robot_config.yaml's so_arm101.detection.yolo block drives them. CLI args
# always override the config when present.
ARGS=(--camera-topic "$CAMERA_TOPIC")
if [ -n "$PROMPTS" ]; then
  ARGS+=(--yolo-prompts $PROMPTS)
fi
if [ -n "$CONF" ]; then
  ARGS+=(--yolo-conf "$CONF")
fi
if [ -n "$HEADLESS" ]; then ARGS+=("$HEADLESS"); fi
ARGS+=(--ros-args
       -p objects_poses_topic:=/objects_poses_real
       -p objects_bbox_topic:=/objects_bbox_real
       -p annotated_image_topic:=/yoloe_annotated)

if [ "$RUN_BG" -eq 1 ]; then
  nohup ros2 run aruco_camera_localizer localize_yoloe "${ARGS[@]}" \
    > /tmp/loc_yoloe.log 2>&1 &
  echo "[restart] launched in background, PID=$!  log=/tmp/loc_yoloe.log"
else
  echo "[restart] launching foreground (Ctrl+C to stop)..."
  exec ros2 run aruco_camera_localizer localize_yoloe "${ARGS[@]}"
fi
