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
[ $# -eq 0 ] && set -- launch ur5e-dt
exec bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh "$@"
