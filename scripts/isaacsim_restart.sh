#!/bin/bash
# Restart Isaac Sim with a specific extension.
# Usage: isaacsim_restart.sh <ext-id> [--wait]
#
# Kills any running Isaac Sim, waits for cleanup, then launches fresh.
# Also kills YOLOE and control GUI processes to avoid stale connections.
#
# Examples:
#   isaacsim_restart.sh soarm101-dt --wait
#   isaacsim_restart.sh ur5e-dt

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EXT_ID="${1:?Usage: isaacsim_restart.sh <ext-id> [--wait]}"
WAIT="${2:-}"

echo "=== Stopping running instances ==="

# Kill YOLOE detector
pkill -9 -f "localize_yoloe" 2>/dev/null && echo "Killed YOLOE" || true

# Kill control GUI (SIGINT to propagate to children)
pkill -SIGINT -f "ros2.*launch.*control.launch" 2>/dev/null && echo "Sent SIGINT to control GUI launch" || true
sleep 2
# Mop up any orphaned GUI processes
ps aux | grep "so_arm101_control" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null && echo "Killed orphan GUI processes" || true

# Stop Isaac Sim
"$SCRIPT_DIR/isaacsim_close.sh" --force

sleep 2

echo ""
echo "=== Launching Isaac Sim ==="
"$SCRIPT_DIR/isaacsim_launch.sh" "$EXT_ID" $WAIT
