#!/usr/bin/env bash
# scripts/snapshot_aic_eval_offlimit.sh — Phase 2 D-10 + Open Q1 settlement.
#
# Captures from a running aic_eval Docker container:
#   (a) Open Q1: full /aic_controller-namespace topic list + /aic/gazebo/* topics
#       — confirms the topic names CONTEXT.md assumes are what the live container
#       actually publishes (per CLAUDE.md 2026-05-01 warning re YAML-vs-live drift).
#   (b) D-10:    /aic/gazebo/contacts/off_limit echoes during a CheatCode trial
#       — captures Entity.name fields of all off-limit collisions seen during a
#       full trial cycle. Used to seed Plan 02-06's DEFAULT_OFF_LIMIT_PRIMS set.
#
# Usage: ./snapshot_aic_eval_offlimit.sh [DEST]
#   DEST defaults to .planning/phases/02-controller-loop/snapshot
#
# Reference: ~/Documents/aic/scripts/run_cheatcode.sh (Docker bringup pattern)
# Reference: exts/aic-dt/scripts/snapshot_aic_eval.sh (Phase 1 analog)
# Source:    .planning/phases/02-controller-loop/02-RESEARCH.md §"Off-Limit Contact Pipeline"
#            CLAUDE.md "Inspecting Gazebo's actual topic surface" section
#
# Notes on RMW: aic_eval uses rmw_zenoh_cpp with custom router. Probing topics
# from inside the container requires the same env (mirrors Phase 1 snapshot script).
set -euo pipefail

DEST="${1:-.planning/phases/02-controller-loop/snapshot}"
mkdir -p "$DEST"
echo "Snapshot DEST: $DEST"

EVAL_CONTAINER="aic_eval"
MODEL_CONTAINER="aic_model"
EVAL_IMAGE="ghcr.io/intrinsic-dev/aic/aic_eval:latest"
MODEL_IMAGE="my-solution:v1"

cleanup() {
  echo "Tearing down..."
  docker stop "$MODEL_CONTAINER" "$EVAL_CONTAINER" 2>/dev/null || true
  docker rm "$MODEL_CONTAINER" "$EVAL_CONTAINER" 2>/dev/null || true
}
trap cleanup EXIT

# 1. Idempotent cleanup of any prior containers
docker rm -f "$MODEL_CONTAINER" "$EVAL_CONTAINER" 2>/dev/null || true

# 2. Bring up aic_eval headless
echo "Starting aic_eval container (headless, ground_truth=true)..."
docker run -d --name "$EVAL_CONTAINER" --gpus all --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host --privileged \
  "$EVAL_IMAGE" \
  ground_truth:=true start_aic_engine:=true gazebo_gui:=false

# 3. Wait for Gazebo + controller_manager to settle
echo "Waiting 35s for Gazebo + controllers..."
sleep 35

# 4. Capture aic_controller-namespace + aic/gazebo topic list (Open Q1 settlement)
echo "Capturing aic_controller / aic/gazebo topic list..."
docker exec "$EVAL_CONTAINER" bash -c '
  source /opt/ros/kilted/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET &&
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false" &&
  ros2 daemon stop 2>/dev/null; sleep 1 &&
  ros2 topic list
' > "$DEST/all_topics.txt" 2>&1 || true
grep -E "^/aic_controller|^/aic/gazebo|controller_state|joint_commands|pose_commands|off_limit" "$DEST/all_topics.txt" \
  > "$DEST/aic_controller_topic_list.txt" || echo "(no matches)" > "$DEST/aic_controller_topic_list.txt"
echo "  matched topics:"
cat "$DEST/aic_controller_topic_list.txt"

# 5. Snapshot topic info (type + QoS) for each candidate topic
> "$DEST/aic_controller_topic_info.txt"
for topic in /aic_controller/joint_commands /aic_controller/pose_commands /aic_controller/controller_state /aic/gazebo/contacts/off_limit; do
  echo "--- $topic ---" >> "$DEST/aic_controller_topic_info.txt"
  docker exec "$EVAL_CONTAINER" bash -c "
    source /opt/ros/kilted/setup.bash &&
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET &&
    export ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false' &&
    ros2 topic info $topic --verbose 2>&1
  " >> "$DEST/aic_controller_topic_info.txt" 2>&1 || true
done

# 6. Bring up aic_model (CheatCode policy) so the trial actually runs
echo "Bringing up aic_model (CheatCode)..."
docker run -d --name "$MODEL_CONTAINER" --gpus all --net=host \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
  -e AIC_ROUTER_ADDR=localhost:7447 \
  "$MODEL_IMAGE" \
  --ros-args -p "policy:=aic_example_policies.ros.CheatCode" -p use_sim_time:=true

echo "Waiting 60s for stack startup + first contact-relevant motion..."
sleep 60

# 7. Capture /aic/gazebo/contacts/off_limit for 120s (covers a full cheatcode cycle)
echo "Capturing /aic/gazebo/contacts/off_limit for 120s..."
docker exec "$EVAL_CONTAINER" bash -c '
  source /opt/ros/kilted/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET &&
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false" &&
  timeout 120 ros2 topic echo /aic/gazebo/contacts/off_limit
' > "$DEST/aic_eval_offlimit_capture.txt" 2>&1 || true

# 8. Also capture a single sample of /aic_controller/joint_commands + pose_commands
#    + controller_state if anything's publishing — best-effort, may be empty.
for topic in /aic_controller/joint_commands /aic_controller/pose_commands /aic_controller/controller_state; do
  SAFE=$(echo "$topic" | tr / _)
  echo "Capturing single sample of $topic..."
  docker exec "$EVAL_CONTAINER" bash -c "
    source /opt/ros/kilted/setup.bash &&
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET &&
    export ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false' &&
    timeout 10 ros2 topic echo $topic --once
  " > "$DEST/sample$SAFE.yaml" 2>&1 || true
done

echo ""
echo "Snapshot complete. Files in $DEST:"
ls -la "$DEST"
