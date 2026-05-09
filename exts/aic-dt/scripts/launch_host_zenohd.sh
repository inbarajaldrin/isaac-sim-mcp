#!/bin/bash
# Reference: PRD task `zenoh-path-implementation` (2026-05-09); decision in
#   exts/aic-dt/docs/zenoh-decision.md (Path (a) — host-side rmw_zenoh_cpp router).
# Reference: AIC Gazebo eval container at ~/Documents/aic/docker/aic_eval/Dockerfile:54-72
#   (this script mirrors the router-side ZENOH_CONFIG_OVERRIDE verbatim — same
#   listen endpoint tcp/[::]:7447, same shared_memory=false, same router mode).
#
# Launches the host-side zenoh router that bridges:
#   - Isaac Sim aic-dt extension (host rclpy, RMW_IMPLEMENTATION=rmw_zenoh_cpp)
#   - host aic_engine + aic_adapter (humble, RMW_IMPLEMENTATION=rmw_zenoh_cpp)
#   - my-solution:v1 model container (kilted, --net=host, AIC_ROUTER_ADDR=localhost:7447)
#
# AIC_ENABLE_ACL=false by default — matches AIC eval default; ACL bring-up is
# operator-driven (set AIC_ENABLE_ACL=true + AIC_EVAL_PASSWD/AIC_MODEL_PASSWD).
#
# Usage:
#   bash exts/aic-dt/scripts/launch_host_zenohd.sh        # foreground
#   bash exts/aic-dt/scripts/launch_host_zenohd.sh &      # background
#   bash exts/aic-dt/scripts/launch_host_zenohd.sh status # is it running?
#   bash exts/aic-dt/scripts/launch_host_zenohd.sh stop   # SIGINT + wait

set -euo pipefail

LOG="/tmp/aic_zenohd.log"
PIDFILE="/tmp/aic_zenohd.pid"

case "${1:-launch}" in
    status)
        if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
            PID="$(cat "$PIDFILE")"
            echo "rmw_zenohd RUNNING pid=$PID log=$LOG"
            ss -tlnp 2>/dev/null | grep -E ':7447\b' || echo "WARN: no listener on 7447"
            exit 0
        fi
        if pgrep -f rmw_zenohd >/dev/null 2>&1; then
            echo "rmw_zenohd RUNNING (no pidfile) pids=$(pgrep -f rmw_zenohd | tr '\n' ',')"
            ss -tlnp 2>/dev/null | grep -E ':7447\b' || echo "WARN: no listener on 7447"
            exit 0
        fi
        echo "rmw_zenohd NOT RUNNING"
        exit 1
        ;;
    stop)
        if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
            PID="$(cat "$PIDFILE")"
            echo "Stopping rmw_zenohd pid=$PID"
            kill -SIGINT "$PID" 2>/dev/null || true
            sleep 2
            kill -KILL "$PID" 2>/dev/null || true
            rm -f "$PIDFILE"
        fi
        pkill -SIGINT -f rmw_zenohd 2>/dev/null || true
        sleep 1
        pkill -SIGKILL -f rmw_zenohd 2>/dev/null || true
        echo "rmw_zenohd stopped"
        exit 0
        ;;
    launch|"")
        ;;
    *)
        echo "Unknown subcommand: $1 (accepted: launch | status | stop)"
        exit 2
        ;;
esac

# Already running? short-circuit so callers can be idempotent.
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "rmw_zenohd already running pid=$(cat "$PIDFILE") (idempotent no-op)"
    exit 0
fi
if pgrep -f rmw_zenohd >/dev/null 2>&1; then
    echo "rmw_zenohd already running (external pid) — leaving alone"
    pgrep -f rmw_zenohd | head -1 > "$PIDFILE"
    exit 0
fi

# Reuse AIC's authoritative router config — inherits ACL surface for free if
# AIC_ENABLE_ACL=true is later flipped on. For now (default false) it's inert.
AIC_REPO="${AIC_REPO:-$HOME/Documents/aic}"
ZENOH_CFG="$AIC_REPO/docker/aic_eval/aic_zenoh_config.json5"

if [ ! -f "$ZENOH_CFG" ]; then
    echo "ERROR: AIC zenoh config not found at $ZENOH_CFG"
    echo "  Set AIC_REPO env var to override, or vendor the config locally."
    exit 1
fi

: > "$LOG"
echo "[zenohd] starting — config=$ZENOH_CFG log=$LOG"
echo "[zenohd] (mirrors aic_eval Dockerfile:54-72 — listen tcp/[::]:7447, router mode, shared_memory=false)"

# Foreground if no '&' / piped to disown by caller; otherwise the trap-EXIT in
# the wrapper will SIGINT this process when the wrapper exits.
(
    set +u
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CONFIG_URI="$ZENOH_CFG"
    # Verbatim from aic_eval Dockerfile:59-63 router branch (no ACL by default,
    # matches AIC_ENABLE_ACL=false).
    ZENOH_CONFIG_OVERRIDE='mode="router"'
    ZENOH_CONFIG_OVERRIDE+=';listen/endpoints=["tcp/[::]:7447"]'
    ZENOH_CONFIG_OVERRIDE+=';connect/endpoints=[]'
    ZENOH_CONFIG_OVERRIDE+=';routing/router/peers_failover_brokering=true'
    ZENOH_CONFIG_OVERRIDE+=';transport/shared_memory/enabled=false'
    export ZENOH_CONFIG_OVERRIDE
    exec ros2 run rmw_zenoh_cpp rmw_zenohd
) >>"$LOG" 2>&1 &
PID=$!
echo "$PID" > "$PIDFILE"
echo "[zenohd] launched pid=$PID (background)"

# Brief wait for socket bind so callers can immediately use it.
for i in 1 2 3 4 5 6 7 8 9 10; do
    if ss -tln 2>/dev/null | grep -qE ':7447\b'; then
        echo "[zenohd] listener UP on tcp/[::]:7447 after ${i}s"
        exit 0
    fi
    sleep 1
done
echo "[zenohd] WARN: listener never appeared on tcp/[::]:7447 within 10s"
echo "[zenohd] tail of $LOG:"
tail -20 "$LOG" || true
exit 1
