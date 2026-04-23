#!/usr/bin/env bash
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
# Kills any running ROS2 control stack (+ rviz, tkinter GUI), builds the
# vla_SO-ARM101 workspace with --symlink-install, and relaunches
# control.launch.py with mtc:=true and rviz:=true. Leaves Isaac Sim alone.
#
# Usage:
#   ./scripts/restart-control-stack.sh          # build + relaunch
#   ./scripts/restart-control-stack.sh --clean  # full rm install/ build/ log/ before build
#   ./scripts/restart-control-stack.sh --no-mtc # launch without mtc:=true
#   ./scripts/restart-control-stack.sh --tail   # follow the launch log after start
#
# Exit codes:
#   0 = stack relaunched and healthy (move_group + control_gui + mtc + rviz up)
#   1 = build failed
#   2 = launch didn't come up within timeout
# NOTE: no `set -u` — ROS2's setup.bash references unset vars internally
# and would trip the whole script. We do use `set -e` sparingly below.

WORKSPACE="/home/aaugus11/Projects/Exploring-VLAs/vla_SO-ARM101"
LOG=/tmp/so_arm101_mtc.log
CLEAN=0
WITH_MTC=1
TAIL=0
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --no-mtc) WITH_MTC=0 ;;
    --tail) TAIL=1 ;;
    *) echo "unknown arg: $arg" >&2; exit 64 ;;
  esac
done

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }

################################################################
# 1. Kill anything still running from a prior stack.
################################################################
log 'killing any running control stack...'
# SIGINT propagates to launch children on Humble (SIGTERM does not)
pkill -SIGINT -f 'ros2.*launch.*control\.launch' 2>/dev/null || true
for i in 1 2 3 4 5 6 7 8; do
  if ! pgrep -f 'ros2.*launch.*control\.launch' >/dev/null; then
    log "  ros2 launch shut down in ${i}s"; break
  fi
  sleep 1
done

# Stragglers: X11-owning processes get SIGTERM-only (NEVER SIGKILL —
# KWin BadWindow cascade per global CLAUDE.md). Non-X11 processes get
# SIGKILL after a SIGTERM grace period because move_group and MTC nodes
# sometimes ignore SIGTERM when their executor is mid-spin.
x11_procs=('so_arm101_control_gui' 'rviz2')
killable_procs=('move_group' 'mtc_pick_place_node' 'ee_pose_publisher'
                'camera_pose_publisher' 'servo_driver' 'ros2_control_node'
                'controller_manager' 'robot_state_publisher')

# SIGTERM everyone first (gives X11 processes a chance to clean up)
for p in "${x11_procs[@]}" "${killable_procs[@]}"; do
  pkill -SIGTERM -f "$p" 2>/dev/null || true
done
sleep 3

# SIGKILL any non-X11 stragglers — safe, these have no windows
for p in "${killable_procs[@]}" 'ros2.*launch.*control\.launch'; do
  if pgrep -f "$p" >/dev/null 2>&1; then
    log "  SIGKILL stragglers: $p"
    pkill -9 -f "$p" 2>/dev/null || true
  fi
done
sleep 2

# Re-check only X11 processes — if they're still alive we can't safely
# force-kill and we MUST NOT proceed. Prior behaviour was "warn and
# continue," which guaranteed the next control_gui spawn would SIGSEGV
# on the duplicate ROS2 node name + X11 display contention, leaving the
# stack in a half-up state (new RViz + old GUI, or new GUI + no RViz).
# The new control_gui SIGTERM handler has a 2.5s force-exit fallback so
# this path should only fire if the process is pathologically stuck —
# in which case manual intervention is the only safe move.
x11_remain=''
for p in "${x11_procs[@]}"; do
  if pgrep -f "$p" >/dev/null 2>&1; then
    x11_remain="$x11_remain$p "
  fi
done
if [ -n "$x11_remain" ]; then
  log "FATAL: X11 processes survived SIGTERM + 2.5s force-exit: $x11_remain"
  log '       aborting restart — a new launch would SIGSEGV on the'
  log '       duplicate node name + X11 display contention.'
  log '       close the GUI window manually (File > Quit or X button)'
  log '       and re-run this script.'
  exit 3
fi

################################################################
# 2. Build the workspace.
################################################################
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WORKSPACE" || { log "cd $WORKSPACE failed"; exit 1; }

if [ "$CLEAN" -eq 1 ]; then
  log 'clean build: removing install/ build/ log/'
  rm -rf install build log
fi

log 'colcon build (symlink-install)...'
if ! colcon build \
      --packages-select so_arm101_mtc so_arm101_moveit_config so_arm101_control \
      --symlink-install 2>&1 | tail -12; then
  log 'BUILD FAILED'
  exit 1
fi

################################################################
# 3. Relaunch.
################################################################
: > "$LOG"
launch_args="rviz:=true"
if [ "$WITH_MTC" -eq 1 ]; then launch_args="$launch_args mtc:=true"; fi
log "launching: ros2 launch so_arm101_control control.launch.py $launch_args"
# shellcheck disable=SC1091
# shellcheck disable=SC2086
nohup bash -c "
  source /opt/ros/humble/setup.bash
  source $WORKSPACE/install/setup.bash
  exec ros2 launch so_arm101_control control.launch.py $launch_args
" > "$LOG" 2>&1 &
disown
launch_pid=$!
log "  launch pid=$launch_pid  log=$LOG"

################################################################
# 4. Health check — wait up to 30s for the four key processes to appear.
################################################################
# mtc_pick_place_node only when WITH_MTC=1
if [ "$WITH_MTC" -eq 1 ]; then
  required=('move_group' 'so_arm101_control_gui' 'rviz2' 'mtc_pick_place_node')
else
  required=('move_group' 'so_arm101_control_gui' 'rviz2')
fi
for i in $(seq 1 30); do
  missing=0
  for r in "${required[@]}"; do
    pgrep -f "$r" >/dev/null 2>&1 || missing=$((missing + 1))
  done
  [ "$missing" -eq 0 ] && break
  sleep 1
done
if [ "$missing" -ne 0 ]; then
  log 'STACK DID NOT COME UP WITHIN 30s — missing processes:'
  for r in "${required[@]}"; do
    pgrep -f "$r" >/dev/null 2>&1 || log "  (absent) $r"
  done
  log "last 20 log lines ($LOG):"
  tail -20 "$LOG"
  exit 2
fi

log "processes up after ${i}s — verifying node responsiveness..."
source "$WORKSPACE/install/setup.bash"

# Process existence != node healthy. PID 228696 incident (Apr 22): control_gui
# spawned, registered all services, then SIGSEGV'd ~0.5s later when it tried
# to realize its Tk window on a display still owned by a zombie from the
# previous launch. pgrep was happy, ros2 node list looked fine, but the new
# process was already a corpse. To catch this we poll a cheap service call.
node_ok=0
for i in $(seq 1 20); do
  if timeout 2 ros2 service call /so_arm101_control_gui/list_commands \
        std_srvs/srv/Trigger "{}" 2>&1 | grep -q 'success=True'; then
    node_ok=1
    break
  fi
  sleep 1
done
if [ "$node_ok" -eq 0 ]; then
  log 'STACK FAILED RESPONSIVENESS CHECK — control_gui not serving'
  log 'last 30 log lines:'
  tail -30 "$LOG"
  log 'likely cause: duplicate node name OR X11 display contention'
  log '              (see PID 228696 / -11 SIGSEGV pattern in log)'
  exit 4
fi

log "ready after ${i}s node-check — all ${#required[@]} processes up AND serving"
log 'services:'
ros2 service list 2>&1 | grep -E 'grasp_move|drop_point|drop_sweep|mtc_pick_place|/so_arm101_mtc/run' | sort | sed 's/^/  /'

if [ "$TAIL" -eq 1 ]; then
  log "tailing $LOG (Ctrl-C to stop tail; stack keeps running)"
  tail -f "$LOG"
fi
