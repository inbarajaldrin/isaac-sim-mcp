#!/usr/bin/env bash
# launch_rg2.sh — bring up the OnRobot RG2 SIM gripper from the STANDALONE colcon package.
#
# Why this exists: the RG2 gripper used to live EMBEDDED inside ros-mcp-server
# (ros-mcp-server/onrobot_rg2_sim_control/), started ad-hoc by hand. It is now a SEPARATE
# colcon package in the overlay workspace at ~/Desktop/ros2_ws/src/onrobot_rg2_sim_control
# (peer of ur_robot_driver), exactly like a4500 (prismatic-manipulation/scripts/launch_rg2.sh).
# ros-mcp-server no longer embeds it and talks to the gripper purely over ROS topics.
#
# This script DECLARES + brings up the sim gripper backend hands-off so a cold sim restart
# works without the deleted embedded copy. It starts two long-lived nodes the sim grasp loop
# needs, both from the sourced overlay (NOT from ros-mcp-server):
#   1. rg2_gripper_driver + rg2 state publisher
#        via `ros2 launch onrobot_rg2_sim_control gripper.launch.py`
#        -> /rg2ref/joint_states (CAD-arc driver: /rg2sim/gripper_cmd CONTACT mm -> joint)
#   2. rg2_sim_backend (Isaac contact-width seam)
#        via `python3 -m onrobot_rg2_sim_control.rg2_sim_backend`
#        -> /gripper_width_sim (20 Hz) + /rg2_sim/* (contact/actuated/joint_target/...)
#
# Sources THIS repo's config/ros_dds.env so both nodes join the same DDS island (Cyclone +
# ROS_LOCALHOST_ONLY=1 + domain 7) as Isaac and the ros-mcp-server consumer.
#
# Usage:
#   scripts/launch_rg2.sh up      # launch driver + backend, block until /gripper_width_sim live
#   scripts/launch_rg2.sh status  # node + topic state
#   scripts/launch_rg2.sh down    # tear down both nodes
#
# Reference: a4500 prismatic-manipulation/scripts/launch_rg2.sh (standalone-package pattern).
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS distro + the standalone overlay (the colcon ws holding onrobot_rg2_sim_control) + the
# homogeneous sim DDS env (domain 7 / Cyclone / localhost-only).
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
ROS_WS_SETUP="${ROS_WS_SETUP:-$HOME/Desktop/ros2_ws/install/setup.bash}"
if [ ! -f "$ROS_WS_SETUP" ]; then
  echo "ERROR: overlay '$ROS_WS_SETUP' not found. colcon build it first:" >&2
  echo "       (cd ~/Desktop/ros2_ws && colcon build --packages-select onrobot_rg2_sim_control)" >&2
  exit 5
fi
source "$ROS_WS_SETUP"
source "$REPO/config/ros_dds.env"

PKG="onrobot_rg2_sim_control"
LAUNCH_LOG="/tmp/rg2_gripper_launch.log"
LAUNCH_PIDFILE="/tmp/rg2_gripper_launch.pid"
BACKEND_LOG="/tmp/rg2_sim_backend.log"
BACKEND_PIDFILE="/tmp/rg2_sim_backend.pid"
ACTION="${1:-up}"

# Bracket-trick ([r]os2) so the match never hits the calling shell.
_launch_running()  { pgrep -f "[r]os2 launch $PKG gripper.launch.py" >/dev/null 2>&1; }
_backend_running() { pgrep -f "[o]nrobot_rg2_sim_control.rg2_sim_backend" >/dev/null 2>&1; }

case "$ACTION" in
  up)
    if ! ros2 pkg prefix "$PKG" >/dev/null 2>&1; then
      echo "ERROR: '$PKG' not on the ament path after sourcing $ROS_WS_SETUP" >&2; exit 5
    fi
    if _backend_running; then
      echo "[rg2] backend already running (pid $(cat "$BACKEND_PIDFILE" 2>/dev/null)); run '$0 down' first" >&2
      exit 2
    fi
    echo "[rg2] overlay: $ROS_WS_SETUP -> domain $ROS_DOMAIN_ID (Cyclone, localhost_only=$ROS_LOCALHOST_ONLY)"

    # 1. driver + state publisher
    echo "[rg2] launching gripper.launch.py (driver + state pub) ..."
    nohup ros2 launch "$PKG" gripper.launch.py > "$LAUNCH_LOG" 2>&1 &
    echo "$!" > "$LAUNCH_PIDFILE"

    # 2. Isaac contact-width backend (publishes /gripper_width_sim @20Hz on its own timer)
    echo "[rg2] launching rg2_sim_backend ..."
    nohup python3 -m onrobot_rg2_sim_control.rg2_sim_backend > "$BACKEND_LOG" 2>&1 &
    echo "$!" > "$BACKEND_PIDFILE"

    # Ready = /gripper_width_sim publishing (backend timer fires regardless of Isaac feed).
    # Event-based: poll the live topic, not a blind sleep.
    echo "[rg2] waiting for /gripper_width_sim ..."
    for i in $(seq 1 20); do
      sleep 1
      if timeout 3 ros2 topic echo /gripper_width_sim --once --field data >/dev/null 2>&1; then
        echo "[rg2] READY: /gripper_width_sim live (after ${i}s)"
        echo "  driver pid  $(cat "$LAUNCH_PIDFILE")  log $LAUNCH_LOG"
        echo "  backend pid $(cat "$BACKEND_PIDFILE") log $BACKEND_LOG"
        exit 0
      fi
      if [ -f "$BACKEND_PIDFILE" ] && ! kill -0 "$(cat "$BACKEND_PIDFILE")" 2>/dev/null; then
        echo "ERROR: rg2_sim_backend exited; see $BACKEND_LOG" >&2
        tail -20 "$BACKEND_LOG" | sed 's/^/  log: /'; exit 4
      fi
    done
    echo "ERROR: /gripper_width_sim not live within timeout; see $BACKEND_LOG / $LAUNCH_LOG" >&2
    tail -20 "$BACKEND_LOG" 2>/dev/null | sed 's/^/  backend: /'
    exit 4
    ;;

  status)
    _launch_running  && echo "[rg2] driver  RUNNING (pid $(cat "$LAUNCH_PIDFILE" 2>/dev/null))"  || echo "[rg2] driver  NOT running"
    _backend_running && echo "[rg2] backend RUNNING (pid $(cat "$BACKEND_PIDFILE" 2>/dev/null))" || echo "[rg2] backend NOT running"
    echo "--- nodes ---"; ros2 node list 2>/dev/null | grep -E "rg2" || echo "  (no rg2 nodes)"
    echo "--- /gripper_width_sim (1 msg) ---"
    timeout 5 ros2 topic echo /gripper_width_sim --once --field data 2>/dev/null || echo "  (no /gripper_width_sim)"
    ;;

  down)
    echo "[rg2] tearing down ..."
    for PF in "$BACKEND_PIDFILE" "$LAUNCH_PIDFILE"; do
      if [ -f "$PF" ]; then
        PID="$(cat "$PF")"
        kill -INT "$PID" 2>/dev/null || true
        for _ in $(seq 1 8); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
        kill -0 "$PID" 2>/dev/null && kill -TERM "$PID" 2>/dev/null || true
        for _ in $(seq 1 4); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
        kill -0 "$PID" 2>/dev/null && kill -KILL "$PID" 2>/dev/null || true
        rm -f "$PF"
      fi
    done
    # Reap stragglers (the launch's child nodes + the backend module).
    pkill -INT -f "[r]os2 launch $PKG gripper.launch.py" 2>/dev/null || true
    pkill -f "[o]nrobot_rg2_sim_control.rg2_sim_backend" 2>/dev/null || true
    pkill -f "[r]g2_gripper_driver.py" 2>/dev/null || true
    echo "[rg2] down."
    ;;

  *)
    echo "usage: $0 {up|status|down}" >&2; exit 1 ;;
esac
