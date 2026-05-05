#!/bin/bash
# Reference: 04-RESEARCH.md Q4 Option 4B; replaces ~/Documents/aic/docker/aic_eval/entrypoint.sh
# Reference: ~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py:235-256 (Node specs reused)
# Reference: 04-01-SUMMARY.md (A2=PASS — fastrtps + ROS_DOMAIN_ID=7 verified end-to-end).
#
# Engine-only entrypoint: runs ONLY aic_engine + aic_adapter (no Gazebo,
# no ros2_control, no ros_gz_bridge, no robot_state_publisher). Isaac Sim
# is the source of truth for /joint_states, /tf, /fts_broadcaster/wrench,
# /aic_controller/* — aic_engine consumes those topics.
#
# Workspace path is /ws_aic/install (per upstream Dockerfile line ~25).

set -e

# Source the kilted overlay + the AIC workspace overlay.
source /opt/ros/kilted/setup.bash
if [ -f /ws_aic/install/setup.bash ]; then
    source /ws_aic/install/setup.bash
fi

# Trial config defaults to the upstream sample_config.yaml inside the image.
# Bind-mount or override via -e CONFIG_PATH for custom configs.
CONFIG_PATH="${CONFIG_PATH:-/ws_aic/install/share/aic_engine/config/sample_config.yaml}"

echo "[my-eval-isaac] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[my-eval-isaac] aic_engine config: $CONFIG_PATH"
echo "[my-eval-isaac] ROS workspace setup: /ws_aic/install/setup.bash"
echo "[my-eval-isaac] starting aic_adapter + aic_engine (no Gazebo, no ros2_control)"

# aic_adapter — the joint-state reorder bridge. Runs even when Isaac Sim
# already publishes /joint_states because the controller stack on the
# real-robot side expects the adapter's reordered topic.
if [ "${SKIP_ADAPTER:-0}" != "1" ]; then
    if ros2 pkg list 2>/dev/null | grep -q '^aic_adapter$'; then
        echo "[my-eval-isaac] launching aic_adapter (background)"
        ros2 run aic_adapter aic_adapter \
            --ros-args -p use_sim_time:=true &
        ADAPTER_PID=$!
    else
        echo "[my-eval-isaac] aic_adapter package not found — skipping (engine should still consume Isaac Sim topics)"
    fi
fi

# aic_engine — the trial orchestrator. Mirror Node spec from aic_gz_bringup.launch.py.
echo "[my-eval-isaac] exec aic_engine"
exec ros2 run aic_engine aic_engine \
    --ros-args \
    -p config_file_path:="$CONFIG_PATH" \
    -p use_sim_time:=true \
    -p model_discovery_timeout_seconds:=30.0 \
    -p model_configure_timeout_seconds:=30.0 \
    "$@"
