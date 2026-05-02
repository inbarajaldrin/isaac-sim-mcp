#!/usr/bin/env bash
# scripts/snapshot_aic_eval.sh — capture live Gazebo aic_eval topic surface (D-01, D-14)
#
# Usage: ./snapshot_aic_eval.sh [DEST]
#   DEST defaults to .planning/phases/01-foundation-parity/snapshot
#
# Per Phase 1 D-01: live aic_eval Docker container is the canonical source of truth
# for "what Gazebo publishes". Per D-14: image is pinned by SHA-256 digest captured
# from `docker inspect --format='{{index .RepoDigests 0}}'`.
#
# Requirements: Docker with NVIDIA runtime, ~35s startup wait, ~6.4 GB image already pulled.
# Idempotent: removes any prior aic_eval container before starting.
#
# Reference: ~/Documents/aic/scripts/run_cheatcode.sh (Docker bringup pattern)
# Source: .planning/phases/01-foundation-parity/01-RESEARCH.md lines 575-643
set -euo pipefail

DEST="${1:-.planning/phases/01-foundation-parity/snapshot}"
mkdir -p "$DEST"

echo "Snapshot DEST: $DEST"

# 1. Idempotent cleanup of any prior container
docker rm -f aic_eval 2>/dev/null || true

# 2. Bring up aic_eval headless
echo "Starting aic_eval container (headless, ground_truth=true)..."
docker run -d --name aic_eval --gpus all --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host --privileged \
  ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  ground_truth:=true start_aic_engine:=true gazebo_gui:=false

# 3. Wait for Gazebo + controller_manager
echo "Waiting 35s for Gazebo + controllers..."
sleep 35

# 4. Capture image digest (D-14 — pin upstream supply chain)
#    Note: running containers may have empty .RepoDigests (the field is image metadata,
#    not container metadata). Inspect the image itself by name.
docker inspect --format='{{index .RepoDigests 0}}' ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  | tee "$DEST/image_digest.txt"

# 5. Capture full topic list
#    The container uses Zenoh as RMW with custom router; need explicit env to probe.
echo "Capturing topic list..."
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 daemon stop 2>/dev/null
  sleep 1
  ros2 topic list
' | sort | tee "$DEST/topic_list.txt"

# 6. Per-topic info (passive sensor topics for Phase 1)
echo "Capturing per-topic info..."
for T in /joint_states /tf /tf_static /clock /fts_broadcaster/wrench; do
  SAFE=$(echo "$T" | tr / _)
  docker exec aic_eval bash -c "
    source /opt/ros/kilted/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false'
    ros2 topic info $T --verbose
  " > "$DEST/topic_info$SAFE.txt" 2>&1 || true
done

# 7. /joint_states sample message (CRITICAL — name set + ordering)
echo "Capturing /joint_states sample..."
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 topic echo /joint_states --once
' > "$DEST/joint_states_sample.yaml" 2>&1 || true

# 8. view_frames TF tree capture
echo "Capturing TF tree via view_frames..."
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 daemon stop 2>/dev/null; sleep 1
  cd /tmp && ros2 run tf2_tools view_frames -o aic_frames
' || true
docker cp aic_eval:/tmp/aic_frames.gv "$DEST/aic_frames_live.gv" || true
docker cp aic_eval:/tmp/aic_frames.pdf "$DEST/aic_frames_live.pdf" 2>/dev/null || true

# 9. Tear down
echo "Tearing down container..."
docker stop aic_eval >/dev/null
docker rm aic_eval >/dev/null

echo "Snapshot saved to $DEST/"
