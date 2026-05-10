#!/usr/bin/env bash
# Reference: zenoh-rpc-stall-fix.md — root-cause is rmw_zenoh-cpp version
# mismatch (humble apt: 0.1.8 ; kilted-pixi: 0.6.6 — major-version drift in
# service queryable key encoding). Discovery via liveliness tokens stayed
# backward-compatible (so `ros2 service list` finds the service from humble),
# but every actual service-RPC payload stalls.
#
# Fix: build aic_model + aic_example_policies humble-native on the host so
# every node in the pipeline (Isaac Sim rclpy publishers, aic_engine,
# aic_adapter, aic_model) speaks the SAME rmw_zenoh-cpp 0.1.8 wire format.
# Sister to build_aic_engine_host.sh.
#
# Idempotent: re-run vendors fresh + rebuilds incrementally.
#
# Required pivot (humble vs kilted rclpy API drift):
#   aic_model/aic_model/aic_model.py: kilted's `with rclpy.init(args=args):`
#   uses Context as a context manager — humble's rclpy.init() returns None and
#   has no __enter__. Patched in-place via sed below.

set -eo pipefail

WS="$HOME/aic_humble_ws"
SRC="$WS/src"
AIC="$HOME/Documents/aic"
APT_VENDOR="$WS/vendored_apt"

mkdir -p "$SRC"

# 1. Vendor aic_model + aic_example_policies (idempotent)
for pkg in aic_model aic_example_policies; do
    rm -rf "$SRC/$pkg"
    cp -r "$AIC/$pkg" "$SRC/"
    # aic_model ships a pixi.toml that confuses colcon — strip it.
    rm -f "$SRC/$pkg/pixi.toml" "$SRC/$pkg/pixi.lock"
done

# 2. Apply humble compatibility pivot:
#    Replace `with rclpy.init(args=args):` block with try/finally form that
#    works on humble (rclpy.init returns None) AND kilted (still functions).
python3 - <<'PY'
import re, pathlib
p = pathlib.Path.home() / "aic_humble_ws/src/aic_model/aic_model/aic_model.py"
src = p.read_text()
# Match the kilted-only context-manager idiom and rewrite into humble-safe form.
old = (
    "def main(args=None):\n"
    "    try:\n"
    "        with rclpy.init(args=args):\n"
    "            aic_model_node = AicModel()\n"
    "            executor = MultiThreadedExecutor()\n"
    "            executor.add_node(aic_model_node)\n"
    "            executor.spin()\n"
    "    except (KeyboardInterrupt, ExternalShutdownException):\n"
    "        pass\n"
)
new = (
    "def main(args=None):\n"
    "    # Humble compat: rclpy.init() returns None on humble (kilted returns a\n"
    "    # Context that supports __enter__). Use explicit try/finally instead.\n"
    "    rclpy.init(args=args)\n"
    "    try:\n"
    "        aic_model_node = AicModel()\n"
    "        executor = MultiThreadedExecutor()\n"
    "        executor.add_node(aic_model_node)\n"
    "        executor.spin()\n"
    "    except (KeyboardInterrupt, ExternalShutdownException):\n"
    "        pass\n"
    "    finally:\n"
    "        try:\n"
    "            rclpy.shutdown()\n"
    "        except Exception:\n"
    "            pass\n"
)
if old in src:
    src = src.replace(old, new)
    p.write_text(src)
    print("[build_aic_model_host] applied humble compat pivot to aic_model.py")
elif new.split("\n")[0] in src and "rclpy.init(args=args)\n    try:" in src:
    print("[build_aic_model_host] aic_model.py already has humble compat pivot — skipping")
else:
    raise SystemExit(f"[build_aic_model_host] FAIL: could not locate kilted main() block in {p}")
PY

# 3. Build (uses same vendored apt overlay that build_aic_engine_host.sh seeds).
cd "$WS"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$APT_VENDOR/extracted/opt/ros/humble:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$APT_VENDOR/extracted/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$APT_VENDOR/extracted/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
colcon build --symlink-install --packages-select aic_model aic_example_policies

# 4. Verify
[ -x "$WS/install/aic_model/lib/aic_model/aic_model" ] || {
    echo "[build_aic_model_host] FAIL: aic_model entry-point missing after build"
    exit 1
}

echo ""
echo "[build_aic_model_host] OK"
echo "  aic_model launcher:  $WS/install/aic_model/lib/aic_model/aic_model"
echo ""
echo "Run via:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_zenoh_cpp \\"
echo "    ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/localhost:7447\"];transport/shared_memory/enabled=false' \\"
echo "    ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.CheatCode"
