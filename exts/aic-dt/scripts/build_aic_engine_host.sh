#!/usr/bin/env bash
# Reference: Plan 04-03 host-build pivot — replaces the my-eval-isaac Docker
# build (kilted-based, blocked by kilted↔humble fastrtps type-hash interop).
#
# Builds aic_engine + aic_adapter (and their AIC interface deps) into a host
# colcon workspace at $HOME/aic_humble_ws, against the system /opt/ros/humble
# stack — eliminating the cross-distro RMW boundary that blocked /clock and
# /joint_states in iter 4 of the 04-03 dry-run.
#
# Idempotent: re-running re-vendors fresh source from ~/Documents/aic and
# rebuilds (colcon's incremental build keeps this fast on warm cache).
#
# Required tweaks applied to the vendored sources (see source_pivots/ patches):
#   - aic_scoring/CMakeLists.txt: gz_math_vendor (kilted, gz-math8) replaced
#     with system gz-math7 (libgz-math7-dev). aic_scoring source does not
#     actually use any gz::math symbols (verified via grep).
#   - aic_scoring/src/ScoringTier2.cc: humble create_generic_subscription
#     1-arg lambda + humble rosbag2 4-arg write() overload.
#   - aic_adapter/{include/aic_adapter/aic_adapter.hpp, src/aic_adapter.cpp}:
#     C++20 <format> → libfmt (g++-11 on Ubuntu 22.04 lacks <format>).
#   - aic_adapter/CMakeLists.txt: find_package(fmt REQUIRED) + fmt::fmt link.
#
# `simulation_interfaces` is sourced from the apt package
# ros-humble-simulation-interfaces, extracted into vendored_apt/extracted/
# (no sudo required) and AMENT_PREFIX_PATH'd into the build env.

# Note: use `set -e` not `set -u` because /opt/ros/humble/setup.bash uses
# unbound vars (AMENT_TRACE_SETUP_FILES) which trips set -u.
set -eo pipefail

WS="$HOME/aic_humble_ws"
SRC="$WS/src"
APT_VENDOR="$WS/vendored_apt"
AIC="$HOME/Documents/aic"

mkdir -p "$SRC" "$APT_VENDOR"

# 1. Vendor AIC packages (idempotent re-copy)
for pkg in aic_engine aic_adapter aic_scoring; do
    rm -rf "$SRC/$pkg"
    cp -r "$AIC/$pkg" "$SRC/"
done
for pkg in aic_engine_interfaces aic_task_interfaces aic_model_interfaces aic_control_interfaces; do
    rm -rf "$SRC/$pkg"
    cp -r "$AIC/aic_interfaces/$pkg" "$SRC/"
done

# 2. Apply source pivots (kilted → humble + gcc11 compatibility).
#    These overwrite specific files in the vendored copies.
PATCHES_DIR="$(dirname "$(realpath "$0")")/../source_pivots"
if [ -d "$PATCHES_DIR" ]; then
    for f in "$PATCHES_DIR"/*; do
        bn="$(basename "$f")"
        # Only files with '@' separators are pivots (README.md etc. are skipped).
        if [[ "$bn" != *"@"* ]]; then
            continue
        fi
        rel="$(echo "$bn" | tr '@' '/')"
        target="$SRC/$rel"
        mkdir -p "$(dirname "$target")"
        cp "$f" "$target"
        echo "[build_aic_engine_host] applied pivot: $rel"
    done
fi

# 3. Vendor simulation_interfaces apt package (no sudo)
if [ ! -d "$APT_VENDOR/extracted/opt/ros/humble/share/simulation_interfaces" ]; then
    cd "$APT_VENDOR"
    apt-get download ros-humble-simulation-interfaces
    dpkg-deb -x ros-humble-simulation-interfaces*.deb extracted/
    # The apt install layout includes a package.xml that confuses colcon — ignore it
    touch "$APT_VENDOR/COLCON_IGNORE"
fi

# 4. Build
cd "$WS"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$APT_VENDOR/extracted/opt/ros/humble:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$APT_VENDOR/extracted/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$APT_VENDOR/extracted/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
colcon build --symlink-install --packages-up-to aic_engine aic_adapter

# 5. Verify
[ -x "$WS/install/aic_engine/lib/aic_engine/aic_engine" ] || {
    echo "[build_aic_engine_host] FAIL: aic_engine binary missing after build"
    exit 1
}
[ -x "$WS/install/aic_adapter/lib/aic_adapter/aic_adapter" ] || {
    echo "[build_aic_engine_host] FAIL: aic_adapter binary missing after build"
    exit 1
}

echo ""
echo "[build_aic_engine_host] OK"
echo "  aic_engine:  $WS/install/aic_engine/lib/aic_engine/aic_engine"
echo "  aic_adapter: $WS/install/aic_adapter/lib/aic_adapter/aic_adapter"
echo ""
echo "Run via:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  export AMENT_PREFIX_PATH=\"$APT_VENDOR/extracted/opt/ros/humble:\$AMENT_PREFIX_PATH\""
echo "  export LD_LIBRARY_PATH=\"$APT_VENDOR/extracted/opt/ros/humble/lib:\$LD_LIBRARY_PATH\""
echo "  ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run aic_engine aic_engine --ros-args -p config_file_path:=$AIC/aic_engine/config/sample_config.yaml -p use_sim_time:=true"
