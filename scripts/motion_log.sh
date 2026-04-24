#!/usr/bin/env bash
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
#
# Lifecycle wrapper for scripts/motion_logger.py.
#
# Commands:
#   ./scripts/motion_log.sh start     # background the logger (PID → /tmp/motion_logger.pid)
#   ./scripts/motion_log.sh stop      # SIGTERM the logger; waits for clean flush
#   ./scripts/motion_log.sh status    # running? how many motions captured so far?
#   ./scripts/motion_log.sh tail      # tail the logger's stderr
#   ./scripts/motion_log.sh restart   # stop + start
#   ./scripts/motion_log.sh latest    # analyze-print the most recent motion
#   ./scripts/motion_log.sh scan      # summarize every motion on disk
#   ./scripts/motion_log.sh verify [CSV|--latest|--scan] [--every N]
#                                     # replay captured trajectory through MoveIt's
#                                     # /check_state_validity — flags off-plan collisions
#
# Log root: ~/motion_logs (override with $MOTION_LOG_ROOT)

# NOTE: no `set -u` — ROS2's setup.bash references unset vars internally.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGGER="$REPO/scripts/motion_logger.py"
SUPERVISOR="$REPO/scripts/motion_log_supervisor.sh"
ANALYZER="$REPO/scripts/motion_analyze.py"
VERIFIER="$REPO/scripts/motion_verify.py"
PIDFILE=/tmp/motion_logger.pid           # supervisor pid
CHILD_PID_FILE=/tmp/motion_logger.child.pid
SHUTDOWN_FLAG=/tmp/motion_logger.shutdown
STDERR=/tmp/motion_logger.stderr
STDOUT=/tmp/motion_logger.stdout
LOG_ROOT="${MOTION_LOG_ROOT:-$HOME/motion_logs}"

cmd="${1:-}"

_source_ros() {
    # Best-effort source of ROS2 + workspace overlay
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    local ws="$HOME/Projects/Exploring-VLAs/vla_SO-ARM101/install/setup.bash"
    if [ -f "$ws" ]; then
        source "$ws"
    fi
}

_running() {
    [ -f "$PIDFILE" ] || return 1
    local pid
    pid=$(cat "$PIDFILE" 2>/dev/null || echo "")
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start() {
    if _running; then
        echo "motion_logger supervisor already running (pid $(cat "$PIDFILE"))"
        return 0
    fi
    _source_ros
    export MOTION_LOG_ROOT="$LOG_ROOT"
    mkdir -p "$LOG_ROOT"
    rm -f "$SHUTDOWN_FLAG"
    # Start the supervisor in background. It spawns + respawns the logger.
    nohup bash "$SUPERVISOR" < /dev/null >> "$STDOUT" 2>> "$STDERR" &
    local sup_pid=$!
    echo "$sup_pid" > "$PIDFILE"
    # Wait for supervisor to spawn logger + logger to announce readiness
    for i in 1 2 3 4 5 6 7 8; do
        sleep 1
        if [ -f "$CHILD_PID_FILE" ]; then
            local cpid
            cpid=$(cat "$CHILD_PID_FILE" 2>/dev/null)
            if [ -n "$cpid" ] && kill -0 "$cpid" 2>/dev/null; then
                echo "motion_logger started (supervisor pid $sup_pid, logger pid $cpid)"
                echo "  log dir:   $LOG_ROOT"
                echo "  stdout:    $STDOUT"
                echo "  stderr:    $STDERR"
                echo "  (respawns automatically if the logger is killed)"
                return 0
            fi
        fi
    done
    echo "motion_logger failed to start. Last stderr:"
    tail -n 30 "$STDERR" 2>/dev/null || true
    kill -TERM "$sup_pid" 2>/dev/null || true
    rm -f "$PIDFILE"
    return 1
}

stop() {
    if ! _running; then
        echo "motion_logger not running."
        rm -f "$PIDFILE" "$CHILD_PID_FILE" "$SHUTDOWN_FLAG"
        return 0
    fi
    local pid
    pid=$(cat "$PIDFILE")
    echo "Stopping motion_logger supervisor (pid $pid)..."
    touch "$SHUTDOWN_FLAG"           # tells supervisor not to respawn
    kill -TERM "$pid" 2>/dev/null    # supervisor's trap propagates to child
    for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "  clean shutdown in ${i}s"
            rm -f "$PIDFILE" "$CHILD_PID_FILE" "$SHUTDOWN_FLAG"
            return 0
        fi
        sleep 1
    done
    echo "  still alive after 12s, sending SIGKILL to supervisor + child"
    if [ -f "$CHILD_PID_FILE" ]; then
        kill -KILL "$(cat "$CHILD_PID_FILE" 2>/dev/null)" 2>/dev/null || true
    fi
    kill -KILL "$pid" 2>/dev/null || true
    rm -f "$PIDFILE" "$CHILD_PID_FILE" "$SHUTDOWN_FLAG"
}

status() {
    if _running; then
        local sup cpid cstate
        sup=$(cat "$PIDFILE")
        cpid=$(cat "$CHILD_PID_FILE" 2>/dev/null)
        cstate="missing"
        if [ -n "$cpid" ] && kill -0 "$cpid" 2>/dev/null; then
            cstate="alive (pid $cpid)"
        fi
        echo "supervisor RUNNING (pid $sup); logger child: $cstate"
    else
        echo "motion_logger NOT running"
    fi
    echo "log root: $LOG_ROOT"
    if [ -d "$LOG_ROOT" ]; then
        local n
        n=$(find "$LOG_ROOT" -name '*.csv' 2>/dev/null | wc -l)
        echo "motions captured on disk: $n"
        local latest
        latest=$(find "$LOG_ROOT" -name '*.csv' -printf '%T+ %p\n' 2>/dev/null | sort | tail -1 | awk '{print $2}')
        if [ -n "$latest" ]; then
            echo "latest: $latest"
        fi
    fi
}

tail_log() {
    tail -f "$STDERR" "$STDOUT" 2>/dev/null
}

latest() {
    _source_ros
    python3 "$ANALYZER" --latest --log-dir "$LOG_ROOT"
}

scan() {
    _source_ros
    python3 "$ANALYZER" --scan --log-dir "$LOG_ROOT"
}

verify() {
    # Pass-through to motion_verify.py. Default (no args) = --latest.
    _source_ros
    shift 2>/dev/null || true   # drop the "verify" subcommand
    local args=("$@")
    if [ ${#args[@]} -eq 0 ]; then
        args=(--latest)
    fi
    python3 "$VERIFIER" --log-dir "$LOG_ROOT" "${args[@]}"
}

case "$cmd" in
    start)   start ;;
    stop)    stop ;;
    restart) stop && start ;;
    status)  status ;;
    tail)    tail_log ;;
    latest)  latest ;;
    scan)    scan ;;
    verify)  verify "$@" ;;
    *)
        echo "usage: $0 {start|stop|restart|status|tail|latest|scan|verify}"
        echo ""
        echo "verify options:"
        echo "  $0 verify                         # replay latest motion"
        echo "  $0 verify --latest                # same as above (explicit)"
        echo "  $0 verify PATH/TO/motion.csv      # replay a specific motion"
        echo "  $0 verify --scan                  # replay every captured motion (slow)"
        echo "  $0 verify --latest --every 5      # downsample (fewer service calls)"
        exit 64
        ;;
esac
