#!/bin/bash
# Canonical ur5e-dt sim launch. Sources config/ros_dds.env (Cyclone + ROS_LOCALHOST_ONLY=1
# + domain 7) so Isaac joins the SAME DDS island as ros-mcp-server + the UR driver --
# otherwise the consumer can't discover Isaac's /gripper_width_sim, /objects_poses_sim, etc.
# Then delegates to the skill launcher. Default action: launch ur5e-dt.
# Usage: scripts/launch_sim.sh [launch ur5e-dt | close | status | restart <ext> | wait ...]
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source ~/env_isaaclab/bin/activate
source "$REPO/config/ros_dds.env"
export DISPLAY="${DISPLAY:-:0}"
# Isaac Sim 5.1 async-rendering deadlock fix (NVIDIA forum 349513, verified on a4500 2026-06-17):
# the full experience file enables /app/asyncRendering via isaacsim.core.throttling, and reset()/play()
# deadlocks (render thread stuck compiling an external MDL material). Disabling enable_async at launch
# prevents the hang in quick_start -> load_ur5e -> world.reset_async. NOT a cache problem.
#
# --no-window: run truly headless. On 5.1 over SSH (no real window on $DISPLAY) RTX otherwise tries to
# set up a viewport window that doesn't exist ("Cannot setup ... without a default window") and the
# render thread crashes at the multigpu view-layout stage right after quick_start completes. a4500's
# verified-working launcher runs headless (--no-window) in its webrtc/headless VIEW modes for this reason.
export ISAACSIM_EXTRA_ARGS="${ISAACSIM_EXTRA_ARGS:-} --no-window --/exts/isaacsim.core.throttling/enable_async=false"
[ $# -eq 0 ] && set -- launch ur5e-dt
exec bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh "$@"
