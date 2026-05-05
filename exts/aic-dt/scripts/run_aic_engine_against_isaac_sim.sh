#!/bin/bash
# Reference: 04-RESEARCH.md Q4 Deliverable 3; mirrors ~/Documents/aic/scripts/run_cheatcode.sh.
# Reference: 04-01-SUMMARY.md A2=PASS (RMW interop), A4=MISMATCH (D-13 setter active).
# Reference: CLAUDE.md (MCP socket protocol, ROS_DOMAIN_ID=7, two-launcher reality, cache discipline).
#
# Plan 04-03 TRIAL-03: end-to-end wrapper that drives a sample_config.yaml trial
# against Isaac Sim via the MCP load_trial atom + an engine-only Docker
# container (my-eval-isaac:v1) + a participant-policy container (my-solution:v1).
#
# Usage:
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_2 --ground-truth=false
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1 --clean   # also kill Isaac Sim at end
#
# Requirements:
#   - Isaac Sim aic-dt extension launched (this script will start it if not running)
#   - DerivedDataCache populated (auto-restore from known-good if <100M)
#   - Docker images: my-eval-isaac:v1 (build via docker/my-eval-isaac/build.sh) AND
#                    my-solution:v1 (build via cd ~/Documents/aic && docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .)

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

EVAL_CONTAINER="aic_eval_isaac"
MODEL_CONTAINER="aic_model"
EVAL_IMAGE="my-eval-isaac:v1"
MODEL_IMAGE="my-solution:v1"
POLICY="aic_example_policies.ros.CheatCode"
ENGINE_LOG="/tmp/aic_engine_isaac.log"
WRAPPER_TOP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

cleanup() {
    echo "[wrapper] cleanup"
    docker stop "$MODEL_CONTAINER" "$EVAL_CONTAINER" 2>/dev/null || true
    docker rm   "$MODEL_CONTAINER" "$EVAL_CONTAINER" 2>/dev/null || true
    if [[ "$CLEAN" == "1" ]]; then
        bash "$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh" kill 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "============================================================"
echo "[wrapper] start $WRAPPER_TOP  trial=$TRIAL_KEY  ground_truth=$GROUND_TRUTH"
echo "============================================================"

# ---------- pre-flight: docker images ----------
if ! docker image inspect "$EVAL_IMAGE" &>/dev/null; then
    echo "[wrapper] ERROR: $EVAL_IMAGE not found."
    echo "  Build it first:"
    echo "    bash $(dirname "$0")/../docker/my-eval-isaac/build.sh"
    exit 1
fi
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

# ---------- launch eval (engine-only) container ----------
echo "[wrapper] launching $EVAL_CONTAINER ($EVAL_IMAGE)"
docker rm -f "$EVAL_CONTAINER" 2>/dev/null || true
docker run -d \
    --name "$EVAL_CONTAINER" \
    --gpus all \
    --net=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e ROS_DOMAIN_ID=7 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e CONFIG_PATH=/ws_aic/install/share/aic_engine/config/sample_config.yaml \
    "$EVAL_IMAGE" >/dev/null

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

# ---------- tail engine log until "Engine Stopped" or container exit ----------
echo "[wrapper] tailing $EVAL_CONTAINER logs → $ENGINE_LOG (max 240s)"
: > "$ENGINE_LOG"
docker logs -f "$EVAL_CONTAINER" >>"$ENGINE_LOG" 2>&1 &
TAIL_PID=$!
ENGINE_DONE="0"
for i in $(seq 1 240); do
    if grep -qE "Engine Stopped|process has finished cleanly.*aic_engine|Finished scoring" "$ENGINE_LOG" 2>/dev/null; then
        ENGINE_DONE="1"
        echo "[wrapper] engine reached completion at t=${i}s"
        break
    fi
    if ! docker ps --filter "name=$EVAL_CONTAINER" -q | grep -q .; then
        echo "[wrapper] engine container exited at t=${i}s"
        ENGINE_DONE="exit"
        break
    fi
    sleep 1
done
kill "$TAIL_PID" 2>/dev/null || true

echo ""
echo "[wrapper] ============================================================"
echo "[wrapper] trial=$TRIAL_KEY engine_done=$ENGINE_DONE"
docker logs "$EVAL_CONTAINER" 2>&1 | tail -40 || true
echo "[wrapper] ============================================================"
