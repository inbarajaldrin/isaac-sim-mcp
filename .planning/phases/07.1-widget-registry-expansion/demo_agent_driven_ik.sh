#!/usr/bin/env bash
# Proves Phase 07.1 SC-6: agent drives full IK flow via CLI alone.
#
# Flow:
#   1. Switch to IK tab via set_widget_value
#   2. Type target xyz + rpy into spinboxes via set_widget_value
#   3. Read back via get_widget_value
#   4. Trigger ~/ik_plan_execute
#   5. Read process_log via get_widget_value
#   6. Screenshot via ~/screenshot
#
# Requires: control stack running with Phase 07.1 code (list_widgets /
# get_widget_value / set_widget_value services present).

set -euo pipefail
SVC=/so_arm101_control_gui
OUT_PNG=${1:-/tmp/07.1-demo-after-ik.png}

# ros2 param set auto-detects types. Force string by quoting the value
# argument with a leading/trailing single quote (yaml-escape quirk).
set_widget() {
  local id="$1"; local val="$2"
  ros2 param set "${SVC}" widget_id "${id}"
  ros2 param set "${SVC}" widget_value "'${val}'"
  ros2 service call "${SVC}/set_widget_value" std_srvs/srv/Trigger "{}"
}

get_widget() {
  local id="$1"
  ros2 param set "${SVC}" widget_id "${id}"
  ros2 service call "${SVC}/get_widget_value" std_srvs/srv/Trigger "{}"
}

echo "======================================================================"
echo "Phase 07.1 end-to-end agent-driven IK demo"
echo "Time: $(date -Iseconds)"
echo "Output screenshot: ${OUT_PNG}"
echo "======================================================================"

echo ""
echo "--- Step 1: switch to IK tab ---"
set_widget tab IK

echo ""
echo "--- Step 2: type target pose ---"
# Reachable target roughly in front of the robot, clearly in workspace
set_widget 'X' 0.20
set_widget 'Y' 0.00
set_widget 'Z' 0.15
set_widget 'Roll' 0.0
set_widget 'shoulder_lift@IK' 90.0
set_widget 'Yaw' 0.0

echo ""
echo "--- Step 3: read back target ---"
get_widget 'X'
get_widget 'Y'
get_widget 'Z'

echo ""
echo "--- Step 4: plan + execute via Trigger service ---"
ros2 service call "${SVC}/ik_plan_execute" std_srvs/srv/Trigger "{}"

echo ""
echo "--- Step 5: wait for motion, read process log ---"
sleep 3
get_widget process_log

echo ""
echo "--- Step 6: screenshot ---"
ros2 param set "${SVC}" ik_target "${OUT_PNG}"
ros2 service call "${SVC}/screenshot" std_srvs/srv/Trigger "{}"

echo ""
echo "======================================================================"
echo "Demo complete. Screenshot: ${OUT_PNG}"
echo "======================================================================"
