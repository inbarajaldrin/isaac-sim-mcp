#!/bin/bash
# Reference: 04-RESEARCH.md Q4 Deliverable 3; mirrors ~/Documents/aic/scripts/run_cheatcode.sh.
# Reference: Plan 04-03 host-build pivot (resolves kilted↔humble RMW interop blocker)
# Reference: CLAUDE.md (MCP socket protocol, ROS_DOMAIN_ID=7, two-launcher reality, cache discipline).
#
# Plan 04-03 TRIAL-03: end-to-end wrapper that drives a sample_config.yaml trial
# against Isaac Sim via the MCP load_trial atom + a HOST aic_engine process
# (humble-built via build_aic_engine_host.sh) + a participant-policy container
# (my-solution:v1).
#
# Why host process, not Docker (my-eval-isaac:v1):
#   The original Docker derived-image path (kilted aic_eval base) hit a
#   kilted↔humble fastrtps type-hash incompatibility — the engine container
#   could see Isaac Sim's /clock publisher in `ros2 topic info` but received
#   ZERO messages over rclpy subscribe. See dryrun_trial_1.txt iter-4 for the
#   full diagnosis. Host humble process = same RMW (humble fastrtps) on both
#   sides = no type-hash boundary.
#
# Usage:
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_2 --ground-truth=false
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1 --clean   # also kill Isaac Sim at end
#
# Requirements:
#   - Isaac Sim aic-dt extension launched (this script will start it if not running)
#   - DerivedDataCache populated (auto-restore from known-good if <100M)
#   - aic_engine + aic_adapter built via build_aic_engine_host.sh into ~/aic_humble_ws/install/
#   - Docker image my-solution:v1 (build via cd ~/Documents/aic && docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .)

set -euo pipefail

# ---------- argv parsing ----------
TRIAL_KEY="${1:-}"
shift || true
GROUND_TRUTH="true"
CLEAN="0"
for a in "$@"; do
    case "$a" in
        --ground-truth=true)  GROUND_TRUTH="true" ;;
        --ground-truth=false) GROUND_TRUTH="false" ;;
        --clean) CLEAN="1" ;;
        *) echo "[wrapper] unknown arg: $a (accepted: --ground-truth=true|false, --clean)"; exit 2 ;;
    esac
done
GROUND_TRUTH_LOWER="$GROUND_TRUTH"  # JSON-friendly already

if [[ -z "$TRIAL_KEY" ]]; then
    echo "Usage: $0 <trial_key> [--ground-truth=true|false] [--clean]"
    exit 2
fi

MODEL_CONTAINER="aic_model"
MODEL_IMAGE="my-solution:v1"
POLICY="aic_example_policies.ros.CheatCode"
ENGINE_LOG="/tmp/aic_engine_isaac.log"
ADAPTER_LOG="/tmp/aic_adapter_isaac.log"
WRAPPER_TOP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
AIC_WS="$HOME/aic_humble_ws"
AIC_REPO="$HOME/Documents/aic"
APT_VENDOR="$AIC_WS/vendored_apt/extracted/opt/ros/humble"
ENGINE_BIN="$AIC_WS/install/aic_engine/lib/aic_engine/aic_engine"
ADAPTER_BIN="$AIC_WS/install/aic_adapter/lib/aic_adapter/aic_adapter"
ENGINE_PID=""
ADAPTER_PID=""

cleanup() {
    echo "[wrapper] cleanup"
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ADAPTER_PID" ] && kill -TERM "$ADAPTER_PID" 2>/dev/null || true
    sleep 1
    [ -n "$ENGINE_PID" ] && kill -KILL "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ADAPTER_PID" ] && kill -KILL "$ADAPTER_PID" 2>/dev/null || true
    docker stop "$MODEL_CONTAINER" 2>/dev/null || true
    docker rm   "$MODEL_CONTAINER" 2>/dev/null || true
    if [[ "$CLEAN" == "1" ]]; then
        bash "$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh" kill 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "============================================================"
echo "[wrapper] start $WRAPPER_TOP  trial=$TRIAL_KEY  ground_truth=$GROUND_TRUTH"
echo "============================================================"

# ---------- pre-flight: host aic_engine + aic_adapter binaries ----------
if [ ! -x "$ENGINE_BIN" ] || [ ! -x "$ADAPTER_BIN" ]; then
    echo "[wrapper] ERROR: host aic_engine / aic_adapter binaries missing."
    echo "  Build them first:"
    echo "    bash $(dirname "$0")/build_aic_engine_host.sh"
    exit 1
fi
# ---------- pre-flight: docker images ----------
if ! docker image inspect "$MODEL_IMAGE" &>/dev/null; then
    echo "[wrapper] ERROR: $MODEL_IMAGE not found."
    echo "  Build it first (per ~/Documents/aic/CLAUDE.md):"
    echo "    cd ~/Documents/aic && docker build -f docker/aic_model/Dockerfile -t $MODEL_IMAGE ."
    exit 1
fi

# ---------- pre-flight: DerivedDataCache ----------
CACHE_SIZE=$(du -sm "$HOME/.cache/ov/DerivedDataCache" 2>/dev/null | awk '{print $1}')
CACHE_SIZE="${CACHE_SIZE:-0}"
if [ "$CACHE_SIZE" -lt 100 ]; then
    echo "[wrapper] DerivedDataCache size=${CACHE_SIZE}M (<100M) — restoring known-good"
    "$HOME/env_isaaclab/bin/python" \
        "$HOME/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py" \
        restore known-good
else
    echo "[wrapper] DerivedDataCache OK (${CACHE_SIZE}M)"
fi

# ---------- pre-flight: Isaac Sim status ----------
SIM_STATUS="$(bash "$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh" status 2>&1 || true)"
if echo "$SIM_STATUS" | grep -qE "aic-dt \(8768\): RESPONSIVE"; then
    echo "[wrapper] Isaac Sim aic-dt RESPONSIVE — reusing"
else
    echo "[wrapper] Isaac Sim not running aic-dt — launching cold"
    bash -c 'source ~/env_isaaclab/bin/activate && ROS_DOMAIN_ID=7 DISPLAY=${DISPLAY:-:0} \
        bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt' &
    LAUNCH_PID=$!
    # Wait up to 90s for the MCP socket to open
    for i in $(seq 1 90); do
        if (echo > /dev/tcp/localhost/8768) 2>/dev/null; then
            echo "[wrapper] MCP socket open after ${i}s"
            break
        fi
        sleep 1
    done
    if ! (echo > /dev/tcp/localhost/8768) 2>/dev/null; then
        echo "[wrapper] ERROR: MCP socket never opened on port 8768 after 90s — abort"
        exit 1
    fi
fi

# ---------- send load_trial via MCP ----------
echo "[wrapper] sending load_trial(trial_key=$TRIAL_KEY, ground_truth=$GROUND_TRUTH) via MCP"
"$HOME/env_isaaclab/bin/python" - "$TRIAL_KEY" "$GROUND_TRUTH_LOWER" <<'PY'
import json, socket, sys
trial_key, ground_truth_str = sys.argv[1], sys.argv[2]
ground_truth = (ground_truth_str.lower() == "true")
s = socket.socket()
s.settimeout(180)
s.connect(("127.0.0.1", 8768))
payload = {"type": "load_trial", "params": {"trial_key": trial_key, "ground_truth": ground_truth}}
s.sendall(json.dumps(payload).encode())
buf = b""
while True:
    chunk = s.recv(16384)
    if not chunk:
        break
    buf += chunk
    try:
        resp = json.loads(buf.decode())
        break
    except json.JSONDecodeError:
        continue
print(json.dumps(resp, indent=2))
res = resp.get("result", {}) or {}
status = res.get("status")
if status != "success":
    sys.stderr.write(f"[wrapper] load_trial DID NOT return status=success (got: {status!r})\n")
    sys.exit(3)
PY

# ---------- wait for /joint_states on the bus ----------
echo "[wrapper] waiting for /joint_states on ROS_DOMAIN_ID=7 (max 30s)"
WAIT_OK="0"
for i in $(seq 1 30); do
    if ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        timeout 2 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
        echo "[wrapper] /joint_states live (${i}s)"
        WAIT_OK="1"
        break
    fi
    sleep 1
done
if [ "$WAIT_OK" != "1" ]; then
    echo "[wrapper] WARNING: /joint_states never observed; continuing anyway (engine may still discover)"
fi

# ---------- launch host aic_adapter + aic_engine processes ----------
echo "[wrapper] launching host aic_adapter (humble, ROS_DOMAIN_ID=7)"
: > "$ADAPTER_LOG"
(
    set +u   # /opt/ros/humble/setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
    set -e
    source /opt/ros/humble/setup.bash
    source "$AIC_WS/install/setup.bash"
    export AMENT_PREFIX_PATH="$APT_VENDOR:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$APT_VENDOR/lib:${LD_LIBRARY_PATH:-}"
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    exec "$ADAPTER_BIN" --ros-args -p use_sim_time:=true \
        >>"$ADAPTER_LOG" 2>&1
) &
ADAPTER_PID=$!

echo "[wrapper] launching host aic_engine (humble, ROS_DOMAIN_ID=7)"
: > "$ENGINE_LOG"
(
    set +u   # /opt/ros/humble/setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
    set -e
    source /opt/ros/humble/setup.bash
    source "$AIC_WS/install/setup.bash"
    export AMENT_PREFIX_PATH="$APT_VENDOR:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$APT_VENDOR/lib:${LD_LIBRARY_PATH:-}"
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    exec "$ENGINE_BIN" --ros-args \
        -p config_file_path:="$AIC_REPO/aic_engine/config/sample_config.yaml" \
        -p use_sim_time:=true \
        >>"$ENGINE_LOG" 2>&1
) &
ENGINE_PID=$!

# ---------- launch model (policy) container ----------
echo "[wrapper] launching $MODEL_CONTAINER ($MODEL_IMAGE) policy=$POLICY"
docker rm -f "$MODEL_CONTAINER" 2>/dev/null || true
docker run -d \
    --name "$MODEL_CONTAINER" \
    --gpus all \
    --net=host \
    -e ROS_DOMAIN_ID=7 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ZENOH_ROUTER_CHECK_ATTEMPTS=0 \
    "$MODEL_IMAGE" \
    --ros-args -p "policy:=$POLICY" -p use_sim_time:=true >/dev/null

# ---------- wait for engine completion or exit (max 240s) ----------
echo "[wrapper] waiting for aic_engine completion (PID=$ENGINE_PID, log=$ENGINE_LOG, max 240s)"
ENGINE_DONE="0"
for i in $(seq 1 240); do
    if grep -qE "Engine Stopped|process has finished cleanly.*aic_engine|Finished scoring" "$ENGINE_LOG" 2>/dev/null; then
        ENGINE_DONE="1"
        echo "[wrapper] engine reached completion at t=${i}s"
        break
    fi
    if ! kill -0 "$ENGINE_PID" 2>/dev/null; then
        echo "[wrapper] engine process exited at t=${i}s"
        ENGINE_DONE="exit"
        break
    fi
    sleep 1
done

echo ""
echo "[wrapper] ============================================================"
echo "[wrapper] trial=$TRIAL_KEY engine_done=$ENGINE_DONE"
echo "[wrapper] --- aic_engine log (last 40 lines) ---"
tail -40 "$ENGINE_LOG" || true
echo "[wrapper] --- aic_adapter log (last 10 lines) ---"
tail -10 "$ADAPTER_LOG" || true
echo "[wrapper] ============================================================"
