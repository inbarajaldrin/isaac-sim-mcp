#!/bin/bash
# Reference: PRD task `zenoh-path-implementation` (2026-05-09); verify gate.
# Reference: zenoh-decision.md §6 (verification recipe — 4 acceptance gates).
#
# Acceptance (from PRD `zenoh-path-implementation`):
#   (1) aic_engine container OR host process can subscribe to /clock + /joint_states + /tf
#       published from Isaac Sim — i.e. host-side `ros2 topic echo --once` succeeds via
#       zenoh router. (Wrapper now runs aic_engine as host process per Plan 04-03 pivot,
#       so the equivalent test is from a host shell with RMW_IMPLEMENTATION=rmw_zenoh_cpp.)
#   (2) my-solution:v1 container can find /aic_model/get_state lifecycle service via
#       host router (`ros2 service list` from inside container).
#   (3) bridge/router process is documented + reproducibly launched via
#       exts/aic-dt/scripts/launch_host_zenohd.sh.
#   (4) cross-repo edits logged in plans/cross_repo_changes.json.
#
# This script does NOT relaunch Isaac Sim — relaunch must happen out-of-band (with
# RMW_IMPLEMENTATION=rmw_zenoh_cpp on the wrapper env). If Isaac Sim is not on
# zenoh, gate (1) will fail with a clear error.
#
# Exit codes:
#   0 — all gates pass
#   1 — gate 1 (host-side topic discovery) failed
#   2 — gate 2 (model container service discovery) failed
#   3 — gate 3 (router lifecycle) failed
#   4 — gate 4 (cross-repo log) failed

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
LAUNCH_ZENOHD="$REPO_ROOT/exts/aic-dt/scripts/launch_host_zenohd.sh"
PEER_OVERRIDE='connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'
MODEL_IMAGE="my-solution:v1"
MODEL_CONTAINER="aic_model_zenoh_verify"
PASS_COUNT=0
FAIL_COUNT=0

print_gate() {
    local status="$1" gate="$2" detail="$3"
    if [ "$status" = "PASS" ]; then
        echo "  [PASS] $gate — $detail"
        PASS_COUNT=$((PASS_COUNT+1))
    else
        echo "  [FAIL] $gate — $detail"
        FAIL_COUNT=$((FAIL_COUNT+1))
    fi
}

cleanup_container() {
    docker stop "$MODEL_CONTAINER" >/dev/null 2>&1 || true
    docker rm   "$MODEL_CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup_container EXIT

echo "============================================================"
echo "verify_zenoh_path.sh — zenoh-path-implementation acceptance"
echo "============================================================"

# ---------- Gate 3: router lifecycle (run first; gates 1+2 require it) ----------
echo ""
echo "[Gate 3] router lifecycle"
if [ ! -x "$LAUNCH_ZENOHD" ]; then
    print_gate FAIL "gate-3" "$LAUNCH_ZENOHD missing or not executable"
    exit 3
fi
bash "$LAUNCH_ZENOHD" launch >/dev/null 2>&1 || true
sleep 1
if ! ss -tln 2>/dev/null | grep -qE ':7447\b'; then
    print_gate FAIL "gate-3" "no listener on tcp/[::]:7447 after launch_host_zenohd.sh"
    tail -20 /tmp/aic_zenohd.log 2>/dev/null || true
    exit 3
fi
PIDS="$(pgrep -f rmw_zenohd | tr '\n' ',' | sed 's/,$//')"
print_gate PASS "gate-3" "rmw_zenohd PID(s)=$PIDS, listener UP on tcp/[::]:7447"

# ---------- Gate 1: host can discover Isaac Sim topics via zenoh ----------
echo ""
echo "[Gate 1] host shell (rmw_zenoh_cpp) sees Isaac Sim topics"

# Source ROS env in a subshell with zenoh RMW. Use a fresh ros2 daemon to avoid
# the host's existing-daemon caching topics from a different RMW.
HOST_PROBE_LOG="/tmp/verify_zenoh_host_probe.log"
: > "$HOST_PROBE_LOG"
(
    set +u
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=7
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
    export ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE"
    # Stop any pre-existing daemon (it'd be on a different RMW).
    ros2 daemon stop >/dev/null 2>&1 || true
    sleep 1
    ros2 topic list 2>&1 | sort -u
) >"$HOST_PROBE_LOG" 2>&1 &
PROBE_PID=$!
# ros2 daemon spawn + zenoh discovery typically takes 3-5s.
for i in 1 2 3 4 5 6 7 8 9 10; do
    sleep 1
    if ! kill -0 "$PROBE_PID" 2>/dev/null; then
        break
    fi
done
wait "$PROBE_PID" 2>/dev/null || true

GATE1_TOPICS_FOUND=0
for t in /clock /joint_states /tf; do
    if grep -qx "$t" "$HOST_PROBE_LOG"; then
        GATE1_TOPICS_FOUND=$((GATE1_TOPICS_FOUND+1))
    fi
done
if [ "$GATE1_TOPICS_FOUND" -ge 3 ]; then
    print_gate PASS "gate-1" "host (rmw_zenoh_cpp) sees /clock + /joint_states + /tf published from Isaac Sim"
else
    print_gate FAIL "gate-1" "host sees only $GATE1_TOPICS_FOUND/3 expected topics — Isaac Sim must be relaunched with RMW_IMPLEMENTATION=rmw_zenoh_cpp + ZENOH_CONFIG_OVERRIDE='$PEER_OVERRIDE'"
    echo "    --- topic list seen ---"
    sed 's/^/    /' "$HOST_PROBE_LOG" | head -30
fi

# ---------- Gate 2: model container service discovery via host router ----------
echo ""
echo "[Gate 2] $MODEL_IMAGE container sees /aic_model/get_state via host router"
if ! docker image inspect "$MODEL_IMAGE" >/dev/null 2>&1; then
    print_gate FAIL "gate-2" "$MODEL_IMAGE not built locally — build via: cd ~/Documents/aic && docker build -f docker/aic_model/Dockerfile -t $MODEL_IMAGE ."
else
    cleanup_container
    docker run -d \
        --name "$MODEL_CONTAINER" \
        --net=host \
        -e ROS_DOMAIN_ID=7 \
        -e AIC_ROUTER_ADDR=localhost:7447 \
        -e ZENOH_CONFIG_OVERRIDE="$PEER_OVERRIDE" \
        "$MODEL_IMAGE" \
        --ros-args -p "policy:=aic_example_policies.ros.CheatCode" -p use_sim_time:=true \
        >/dev/null 2>&1 || true

    if ! docker ps --format '{{.Names}}' | grep -qx "$MODEL_CONTAINER"; then
        print_gate FAIL "gate-2" "model container failed to start; check `docker logs $MODEL_CONTAINER`"
    else
        # Lifecycle node bring-up takes ~15-25s after container start.
        SERVICE_FOUND=0
        for i in $(seq 1 40); do
            if docker exec "$MODEL_CONTAINER" bash -lc 'source /opt/ros/kilted/setup.bash 2>/dev/null; ros2 service list 2>/dev/null' \
                    | grep -q '/aic_model/get_state'; then
                SERVICE_FOUND=1
                print_gate PASS "gate-2" "model container sees /aic_model/get_state after ${i}s"
                break
            fi
            sleep 1
        done
        if [ "$SERVICE_FOUND" -ne 1 ]; then
            print_gate FAIL "gate-2" "model container did not register /aic_model/get_state within 40s"
            echo "    --- last 30 lines of model container log ---"
            docker logs --tail 30 "$MODEL_CONTAINER" 2>&1 | sed 's/^/    /' | head -40
        fi
    fi
fi

# ---------- Gate 4: cross-repo edits logged ----------
echo ""
echo "[Gate 4] plans/cross_repo_changes.json updated for any edits outside isaac-sim-mcp"
CR_LOG="$REPO_ROOT/plans/cross_repo_changes.json"
if [ ! -f "$CR_LOG" ]; then
    # First-time creation is the responsibility of this task — see emission below.
    print_gate FAIL "gate-4" "$CR_LOG missing"
else
    # If the apt install happened during this task, expect a system entry
    # mentioning ros-humble-rmw-zenoh-cpp.
    if grep -q 'ros-humble-rmw-zenoh-cpp' "$CR_LOG"; then
        print_gate PASS "gate-4" "$CR_LOG records the system apt install"
    else
        print_gate FAIL "gate-4" "$CR_LOG missing 'ros-humble-rmw-zenoh-cpp' entry — record the apt install"
    fi
fi

# ---------- summary ----------
echo ""
echo "============================================================"
echo "Summary: $PASS_COUNT pass / $FAIL_COUNT fail"
echo "============================================================"
[ "$FAIL_COUNT" -eq 0 ]
