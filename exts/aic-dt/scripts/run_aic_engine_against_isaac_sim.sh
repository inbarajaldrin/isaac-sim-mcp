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
#   bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1 --output-json /path/out.json
#       Emit per-trial outcome JSON consumed by exts/aic-dt/scripts/parity_report.py.
#       Shape (matches parity_report.REQUIRED_KEYS):
#           {trial_id, sim:'isaacsim', insertion_event_fired (bool),
#            offlimit_contact_count (int), completed_steps (int — -1 sentinel
#            since aic_engine doesn't expose a per-trial step counter),
#            ts (ISO 8601 of run start)}
#       JSON is flushed via the EXIT trap, so it's written even when the
#       wrapper bails early (e.g. missing host engine binary). Counters
#       reflect whatever was observable up to the point of exit.
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
OUTPUT_JSON=""
while [ "$#" -gt 0 ]; do
    a="$1"
    case "$a" in
        --ground-truth=true)  GROUND_TRUTH="true"; shift ;;
        --ground-truth=false) GROUND_TRUTH="false"; shift ;;
        --clean) CLEAN="1"; shift ;;
        --output-json=*) OUTPUT_JSON="${a#--output-json=}"; shift ;;
        --output-json) OUTPUT_JSON="${2:-}"; shift 2 ;;
        *) echo "[wrapper] unknown arg: $a (accepted: --ground-truth=true|false, --clean, --output-json=PATH)"; exit 2 ;;
    esac
done
GROUND_TRUTH_LOWER="$GROUND_TRUTH"  # JSON-friendly already

if [[ -z "$TRIAL_KEY" ]]; then
    echo "Usage: $0 <trial_key> [--ground-truth=true|false] [--clean] [--output-json=PATH]"
    exit 2
fi

MODEL_CONTAINER="aic_model"           # legacy name retained for cleanup() best-effort docker rm
MODEL_IMAGE="my-solution:v1"          # legacy reference; no longer used since zenoh-rpc-stall-fix
POLICY="aic_example_policies.ros.CheatCode"
ENGINE_LOG="/tmp/aic_engine_isaac.log"
ADAPTER_LOG="/tmp/aic_adapter_isaac.log"
MODEL_LOG="/tmp/aic_model_isaac.log"
WRAPPER_TOP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
# Env-overridable so test_wrapper_json_emit.sh can force precond failure
# without touching the real workspace.
AIC_WS="${AIC_WS:-$HOME/aic_humble_ws}"
AIC_REPO="${AIC_REPO:-$HOME/Documents/aic}"
APT_VENDOR="$AIC_WS/vendored_apt/extracted/opt/ros/humble"
ENGINE_BIN="$AIC_WS/install/aic_engine/lib/aic_engine/aic_engine"
ADAPTER_BIN="$AIC_WS/install/aic_adapter/lib/aic_adapter/aic_adapter"
MODEL_BIN="$AIC_WS/install/aic_model/lib/aic_model/aic_model"
ENGINE_PID=""
ADAPTER_PID=""
MODEL_PID=""

# ---------- outcome accumulators (read by emit_outcome_json) ----------
# Defaults are the CheatCode passing-trial baseline; cleanup() updates them
# from the engine log before the trap calls emit_outcome_json.
INSERTION_EVENT_FIRED="false"
OFFLIMIT_CONTACT_COUNT=0
COMPLETED_STEPS=-1   # aic_engine has no step counter — sentinel per parity_report contract.

emit_outcome_json() {
    # Idempotent JSON writer. Called from cleanup() trap when --output-json set.
    [ -z "$OUTPUT_JSON" ] && return 0
    mkdir -p "$(dirname "$OUTPUT_JSON")" 2>/dev/null || true
    python3 - "$OUTPUT_JSON" "$TRIAL_KEY" "$INSERTION_EVENT_FIRED" \
                "$OFFLIMIT_CONTACT_COUNT" "$COMPLETED_STEPS" "$WRAPPER_TOP" <<'PY'
import json, sys
out_path, trial_id, insertion_str, offlimit_str, steps_str, ts = sys.argv[1:7]
data = {
    "trial_id": trial_id,
    "sim": "isaacsim",
    "insertion_event_fired": insertion_str.lower() == "true",
    "offlimit_contact_count": int(offlimit_str),
    "completed_steps": int(steps_str),
    "ts": ts,
}
with open(out_path, "w") as f:
    json.dump(data, f, indent=2, sort_keys=True)
    f.write("\n")
PY
    echo "[wrapper] outcome JSON written: $OUTPUT_JSON"
}

parse_engine_log_outcomes() {
    # Mirrors extract_gazebo_baseline.py signal extraction for parity.
    # Both wrappers parse the SAME aic_engine binary's stdout; the canonical
    # tier-3 message ("Cable insertion successful"/"failed") and tier-2 message
    # ("No contact detected") are the source-of-truth signals.
    [ -f "$ENGINE_LOG" ] || return 0
    if grep -q "Cable insertion successful" "$ENGINE_LOG" 2>/dev/null; then
        INSERTION_EVENT_FIRED="true"
    elif grep -q "Cable insertion failed" "$ENGINE_LOG" 2>/dev/null; then
        INSERTION_EVENT_FIRED="false"
    fi
    if grep -q "No contact detected" "$ENGINE_LOG" 2>/dev/null; then
        OFFLIMIT_CONTACT_COUNT=0
    fi
}

cleanup() {
    echo "[wrapper] cleanup"
    parse_engine_log_outcomes || true
    emit_outcome_json || true
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ADAPTER_PID" ] && kill -TERM "$ADAPTER_PID" 2>/dev/null || true
    [ -n "$MODEL_PID" ] && kill -TERM "$MODEL_PID" 2>/dev/null || true
    sleep 1
    [ -n "$ENGINE_PID" ] && kill -KILL "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ADAPTER_PID" ] && kill -KILL "$ADAPTER_PID" 2>/dev/null || true
    [ -n "$MODEL_PID" ] && kill -KILL "$MODEL_PID" 2>/dev/null || true
    # Best-effort cleanup of legacy kilted-Docker model from prior wrapper revisions.
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

# ---------- pre-flight: host aic_engine + aic_adapter + aic_model binaries ----------
if [ ! -x "$ENGINE_BIN" ] || [ ! -x "$ADAPTER_BIN" ]; then
    echo "[wrapper] ERROR: host aic_engine / aic_adapter binaries missing."
    echo "  Build them first:"
    echo "    bash $(dirname "$0")/build_aic_engine_host.sh"
    exit 1
fi
if [ ! -x "$MODEL_BIN" ]; then
    echo "[wrapper] ERROR: host aic_model binary missing."
    echo "  Build it first (resolves humble↔kilted rmw_zenoh-cpp version mismatch — see"
    echo "  exts/aic-dt/docs/zenoh-rpc-stall-fix.md):"
    echo "    bash $(dirname "$0")/build_aic_model_host.sh"
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
    # Isaac Sim's rclpy bridge MUST find librmw_zenoh_cpp.so + libzenohc.so to
    # publish on the zenoh router. The libzenohc.so vendor lives at a
    # non-standard path that /opt/ros/humble/setup.bash leaves off LD_LIBRARY_PATH.
    bash -c '
        source ~/env_isaaclab/bin/activate
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=7
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp
        export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
        export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/localhost:7447\"];transport/shared_memory/enabled=false"
        # Explicitly add zenoh_cpp_vendor lib dir — humble setup.bash misses it.
        export LD_LIBRARY_PATH="/opt/ros/humble/opt/zenoh_cpp_vendor/lib:${LD_LIBRARY_PATH:-}"
        export DISPLAY=${DISPLAY:-:0}
        exec bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt
    ' &
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

# ---------- generate single-trial config (trial-cycle-frames-loading fix) ----------
# aic_engine reads the config's `trials:` block and runs ALL trials in
# sequence. The wrapper is invoked per-trial; we want the engine to run
# only the requested one so subsequent trials don't hit "Waiting for
# transform" failures (their per-trial CheatCode TF frames aren't published
# because we only load_trial(<requested>) once). Generate a derived yaml
# preserving the entire config but filtering `trials:` to just $TRIAL_KEY.
SOURCE_CFG="$AIC_REPO/aic_engine/config/sample_config.yaml"
SINGLE_CFG="/tmp/aic_engine_single_trial_${TRIAL_KEY}.yaml"
"$HOME/env_isaaclab/bin/python" - "$SOURCE_CFG" "$TRIAL_KEY" "$SINGLE_CFG" <<'PY'
import sys, yaml
src_path, trial_key, dst_path = sys.argv[1:4]
with open(src_path) as f:
    cfg = yaml.safe_load(f) or {}
trials = (cfg.get("trials") or {})
if trial_key not in trials:
    raise SystemExit(f"trial_key {trial_key!r} not in {src_path}; available: {sorted(trials.keys())}")
cfg["trials"] = {trial_key: trials[trial_key]}
with open(dst_path, "w") as f:
    yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
print(f"[wrapper] derived single-trial config: {dst_path}")
PY
ENGINE_CFG_PATH="$SINGLE_CFG"

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

# ---------- pre-flight: host zenoh router (per zenoh-decision.md Path (a)) ----------
# All three peers (Isaac Sim rclpy, host aic_engine/adapter, model container)
# unify on rmw_zenoh_cpp + AIC_ROUTER_ADDR=localhost:7447. The router is the
# single discovery point — mirrors AIC docker-compose topology where
# aic_eval container hosted the router.
bash "$(dirname "$0")/launch_host_zenohd.sh" launch >/dev/null
if ! ss -tln 2>/dev/null | grep -qE ':7447\b'; then
    echo "[wrapper] ERROR: zenoh router did not come up on tcp/[::]:7447"
    tail -20 /tmp/aic_zenohd.log 2>/dev/null || true
    exit 1
fi

# ---------- purge any stale host aic_model / aic_engine / aic_adapter processes ----------
# Prevents back-to-back wrapper runs from tripping over a stale liveliness token
# left by a previous run's model that crashed mid-handler. zenoh's liveliness
# TTL is generous; an explicit pkill is the deterministic fix.
pkill -TERM -f "install/aic_model/lib/aic_model/aic_model" 2>/dev/null || true
pkill -TERM -f "install/aic_engine/lib/aic_engine/aic_engine" 2>/dev/null || true
pkill -TERM -f "install/aic_adapter/lib/aic_adapter/aic_adapter" 2>/dev/null || true
sleep 1
pkill -KILL -f "install/aic_model/lib/aic_model/aic_model" 2>/dev/null || true
pkill -KILL -f "install/aic_engine/lib/aic_engine/aic_engine" 2>/dev/null || true
pkill -KILL -f "install/aic_adapter/lib/aic_adapter/aic_adapter" 2>/dev/null || true
# Brief settle so liveliness tokens age out before the next launch.
sleep 2

# Common zenoh peer override applied to every host process below.
# transport/shared_memory disabled per AIC eval Dockerfile:63 (cross-distro safety).
PEER_OVERRIDE='connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'

# ---------- wait for /joint_states on the bus ----------
echo "[wrapper] waiting for /joint_states on ROS_DOMAIN_ID=7 (max 30s)"
WAIT_OK="0"
for i in $(seq 1 30); do
    if ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_zenoh_cpp \
        ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE" \
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

# ---------- launch host aic_adapter ----------
echo "[wrapper] launching host aic_adapter (humble, ROS_DOMAIN_ID=7, RMW=zenoh)"
: > "$ADAPTER_LOG"
(
    set +u   # /opt/ros/humble/setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
    set -e
    source /opt/ros/humble/setup.bash
    source "$AIC_WS/install/setup.bash"
    export AMENT_PREFIX_PATH="$APT_VENDOR:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$APT_VENDOR/lib:${LD_LIBRARY_PATH:-}"
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
    export ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE"
    exec "$ADAPTER_BIN" --ros-args -p use_sim_time:=true \
        >>"$ADAPTER_LOG" 2>&1
) &
ADAPTER_PID=$!

# ---------- launch host aic_model BEFORE engine ----------
# Per exts/aic-dt/docs/zenoh-rpc-stall-fix.md: the original kilted-pixi Docker
# model runs rmw_zenoh-cpp 0.6.6 while host humble runs 0.1.8. The wire format
# for service queryable keys diverged across that gap, so engine RPCs would
# discover but never deliver. Host humble aic_model (built via
# build_aic_model_host.sh) speaks the SAME 0.1.8 wire format as the engine.
#
# IMPORTANT: model MUST launch BEFORE engine. The engine probes /aic_model
# state on its FIRST trial-start tick — if the model isn't registered yet,
# engine sees stale liveliness from a prior run (state=finalized) and tries
# to deactivate a dead node, producing 3 ChangeState timeouts. Pre-flighting
# the model avoids that race.
docker rm -f "$MODEL_CONTAINER" 2>/dev/null || true
echo "[wrapper] launching host aic_model (humble, ROS_DOMAIN_ID=7, RMW=zenoh) policy=$POLICY"
: > "$MODEL_LOG"
(
    set +u
    set -e
    source /opt/ros/humble/setup.bash
    source "$AIC_WS/install/setup.bash"
    export AMENT_PREFIX_PATH="$APT_VENDOR:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$APT_VENDOR/lib:${LD_LIBRARY_PATH:-}"
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
    export ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE"
    exec ros2 run aic_model aic_model --ros-args \
        -p "policy:=$POLICY" -p use_sim_time:=true \
        >>"$MODEL_LOG" 2>&1
) &
MODEL_PID=$!

# ---------- wait for aic_model to register on the bus before engine launches ----------
# Without this, engine starts probing /aic_model lifecycle on its first trial-start
# tick. If the model hasn't registered yet, engine may bind to a STALE liveliness
# token from a prior run (state=finalized) and try to deactivate a dead node,
# producing 3 ChangeState timeouts. We poll /aic_model/get_state visibility
# from the bus until either the new model registers fresh OR a 30s budget elapses.
echo "[wrapper] waiting for aic_model lifecycle service to register (max 30s)"
MODEL_READY="0"
for i in $(seq 1 30); do
    if grep -qE "Loaded policy module|Using policy" "$MODEL_LOG" 2>/dev/null; then
        # Belt-and-braces: also confirm the lifecycle service is actually visible
        # to a humble peer on the bus before declaring ready.
        if bash -c '
            set +u
            source /opt/ros/humble/setup.bash >/dev/null
            export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/localhost:7447\"];transport/shared_memory/enabled=false"
            timeout 3 ros2 service list 2>/dev/null | grep -qE "^/aic_model/get_state\$"
        '; then
            echo "[wrapper] aic_model READY (${i}s)"
            MODEL_READY="1"
            break
        fi
    fi
    sleep 1
done
if [ "$MODEL_READY" != "1" ]; then
    echo "[wrapper] WARNING: aic_model never registered on bus within 30s — engine may probe a stale liveliness token"
fi

# ---------- launch host aic_engine (after model is registered) ----------
echo "[wrapper] launching host aic_engine (humble, ROS_DOMAIN_ID=7, RMW=zenoh)"
: > "$ENGINE_LOG"
(
    set +u   # /opt/ros/humble/setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
    set -e
    source /opt/ros/humble/setup.bash
    source "$AIC_WS/install/setup.bash"
    export AMENT_PREFIX_PATH="$APT_VENDOR:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$APT_VENDOR/lib:${LD_LIBRARY_PATH:-}"
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
    export ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE"
    # skip_ready_simulator=true: aic-dt's load_trial USD atom already authored the
    #   per-trial scene state (board, ports, mount rails, cable, gripper attach).
    #   The engine's ready_simulator() would only spawn duplicate entities via
    #   the Gazebo /gz_server/spawn_entity service which Isaac Sim does not
    #   expose. The build_aic_engine_host.sh pivot also gates check_endpoints'
    #   spawn_entity probe on this flag, so passing it true is what unlocks
    #   the trial advancing past endpoint readiness.
    exec "$ENGINE_BIN" --ros-args \
        -p "config_file_path:=$ENGINE_CFG_PATH" \
        -p use_sim_time:=true \
        -p skip_ready_simulator:=true \
        -p "ground_truth:=$GROUND_TRUTH_LOWER" \
        >>"$ENGINE_LOG" 2>&1
) &
ENGINE_PID=$!

# ---------- wait for engine completion or exit (max 320s) ----------
# Each trial in sample_config.yaml has time_limit=180s. Add headroom for
# engine startup + lifecycle + scoring + teardown so one trial fits in budget.
echo "[wrapper] waiting for aic_engine completion (PID=$ENGINE_PID, log=$ENGINE_LOG, max 320s)"
ENGINE_DONE="0"
for i in $(seq 1 320); do
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
echo "[wrapper] --- aic_model log (last 10 lines) ---"
tail -10 "$MODEL_LOG" || true
echo "[wrapper] ============================================================"
