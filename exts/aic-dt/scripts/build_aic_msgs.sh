#!/usr/bin/env bash
# scripts/build_aic_msgs.sh — Phase 2 D-05 fix: rebuild Python 3.11 workspace
#                              with aic_control_interfaces + ros_gz_interfaces.
#
# See exts/aic-dt/docs/aic-msgs-setup.md for full background, including:
#   - Why the AIC pixi env can't be PYTHONPATH-linked (Python 3.12 vs 3.11 ABI)
#   - The two-workspace install-path landmine (build artifacts go to the OUTER
#     isaac_sim_ros_ws/, not the inner humble_ws/)
#   - The Dockerfile COPY context discipline (vendor to humble_ws/src/, not
#     build_ws/humble/humble_ws/src/)
#
# Reference: ~/IsaacSim-ros_workspaces/dockerfiles/ubuntu_22_humble_python_311_minimal.dockerfile
set -euo pipefail

WS_REPO="$HOME/IsaacSim-ros_workspaces"
WS_SRC="$WS_REPO/humble_ws/src"   # Dockerfile COPY context (NOT build_ws/humble/humble_ws/src)
INSTALL="$WS_REPO/build_ws/humble/isaac_sim_ros_ws/install"

[[ -d "$WS_REPO" ]] || { echo "ERROR: workspace repo not found at $WS_REPO — see exts/aic-dt/docs/rclpy-setup.md to clone it first"; exit 1; }
[[ -d "$WS_SRC" ]] || { echo "ERROR: workspace COPY-context src not found at $WS_SRC"; exit 1; }

# 1. Vendor aic_control_interfaces (idempotent — re-copy each time so source-of-truth tracks AIC)
SRC_AIC="$HOME/Documents/aic/aic_interfaces/aic_control_interfaces"
[[ -d "$SRC_AIC" ]] || { echo "ERROR: AIC source not found at $SRC_AIC — clone ~/Documents/aic first"; exit 1; }
rm -rf "$WS_SRC/aic_control_interfaces"
cp -r "$SRC_AIC" "$WS_SRC/"
echo "Vendored aic_control_interfaces -> $WS_SRC/aic_control_interfaces"

# 2. Clone & vendor ros_gz_interfaces (idempotent — re-clone each time for freshness)
cd /tmp && rm -rf ros_gz
git clone -b humble https://github.com/gazebosim/ros_gz.git --depth 1
rm -rf "$WS_SRC/ros_gz_interfaces"
cp -r /tmp/ros_gz/ros_gz_interfaces "$WS_SRC/"
echo "Vendored ros_gz_interfaces -> $WS_SRC/ros_gz_interfaces"

# 3. Rebuild the workspace (Docker-based; ~10-20 min cold cache, ~5-10 min warm cache).
#    Note: launchpad.net add-apt-repository deadsnakes/ppa step occasionally hits
#    transient HTTP 504 — re-run this script if so.
cd "$WS_REPO" && bash build_ros.sh -d humble -v 22.04

# 4. Sanity-check 3.11 ABI install paths (per the install-path landmine doc'd in aic-msgs-setup.md)
[[ -f "$INSTALL/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces/__init__.py" ]] || \
  { echo "ERROR: aic_control_interfaces __init__.py missing under $INSTALL — workspace build failed (see /workspace/build_ws/log/ inside the Docker image)"; exit 1; }
[[ -f "$INSTALL/ros_gz_interfaces/local/lib/python3.11/dist-packages/ros_gz_interfaces/__init__.py" ]] || \
  { echo "ERROR: ros_gz_interfaces __init__.py missing under $INSTALL — workspace build failed"; exit 1; }

# 5. Reject cpython-310 tagged .so (would mean the build picked up the wrong Python interpreter)
if find "$INSTALL/aic_control_interfaces/" "$INSTALL/ros_gz_interfaces/" -name "*.so" 2>/dev/null | grep -q cpython-310; then
  echo "ERROR: .so files tagged cpython-310 — workspace built against wrong Python (expected 3.11)"
  exit 1
fi

# 6. Round-trip import check via venv-activate (matches the Plan 02-02 consumer's runtime context)
bash -c 'source ~/env_isaaclab/bin/activate && \
  ~/env_isaaclab/bin/python -c "
from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState
from ros_gz_interfaces.msg import Contacts, Contact, Entity
print(\"OK\")
"' || { echo "ERROR: import check failed — env_isaaclab activate or workspace local_setup is not wiring the new packages"; exit 1; }

echo "OK: aic_control_interfaces + ros_gz_interfaces built for Python 3.11 and importable from Isaac Sim's bundled Python."
