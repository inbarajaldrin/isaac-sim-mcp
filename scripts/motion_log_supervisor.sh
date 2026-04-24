#!/usr/bin/env bash
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
# Watchdog for scripts/motion_logger.py.
#
# Invoked by motion_log.sh start. Runs in background, spawns the logger as a
# child, and restarts it if it dies for any reason other than an explicit
# "shutdown" flag being set. On TERM to the supervisor, it clears the child,
# exits cleanly, and lets motion_log.sh remove the pidfile.
#
# Design: resilient to SIGTERM bombs from stack restarts (the logger got
# collaterally killed once; this prevents silent drops in the telemetry
# record). If the logger is unable to stay alive (e.g. persistent import
# error), we back off exponentially so the log doesn't spam.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGGER="$REPO/scripts/motion_logger.py"
LOG_ROOT="${MOTION_LOG_ROOT:-$HOME/motion_logs}"
CHILD_PID_FILE=/tmp/motion_logger.child.pid
SHUTDOWN_FLAG=/tmp/motion_logger.shutdown
STDOUT=/tmp/motion_logger.stdout
STDERR=/tmp/motion_logger.stderr

mkdir -p "$LOG_ROOT"
rm -f "$SHUTDOWN_FLAG"

cleanup() {
    # Tell the current child we want it to go away.
    touch "$SHUTDOWN_FLAG"
    if [ -f "$CHILD_PID_FILE" ]; then
        local cpid
        cpid=$(cat "$CHILD_PID_FILE" 2>/dev/null)
        if [ -n "$cpid" ] && kill -0 "$cpid" 2>/dev/null; then
            kill -TERM "$cpid" 2>/dev/null
            # Give it 8s to flush its open motion + CSV cleanly.
            for i in 1 2 3 4 5 6 7 8; do
                kill -0 "$cpid" 2>/dev/null || break
                sleep 1
            done
            kill -KILL "$cpid" 2>/dev/null || true
        fi
        rm -f "$CHILD_PID_FILE"
    fi
    echo "supervisor: exiting" >> "$STDERR"
    exit 0
}
trap cleanup TERM INT

backoff=2
max_backoff=30
started_ok_at=0

while true; do
    if [ -f "$SHUTDOWN_FLAG" ]; then break; fi

    echo "supervisor: starting logger at $(date '+%H:%M:%S')" >> "$STDERR"
    python3 "$LOGGER" --log-dir "$LOG_ROOT" \
        >> "$STDOUT" 2>> "$STDERR" < /dev/null &
    cpid=$!
    echo "$cpid" > "$CHILD_PID_FILE"
    started_ok_at=$(date +%s)

    wait "$cpid"
    rc=$?
    rm -f "$CHILD_PID_FILE"

    if [ -f "$SHUTDOWN_FLAG" ]; then break; fi

    ran_for=$(( $(date +%s) - started_ok_at ))
    if [ "$ran_for" -lt 5 ]; then
        # crashed immediately → back off
        echo "supervisor: logger died in ${ran_for}s (exit $rc); backing off ${backoff}s" >> "$STDERR"
        sleep "$backoff"
        backoff=$(( backoff * 2 ))
        [ "$backoff" -gt "$max_backoff" ] && backoff="$max_backoff"
    else
        # was running fine, got killed externally → restart fast
        echo "supervisor: logger exited (${ran_for}s uptime, exit $rc); respawning in 2s" >> "$STDERR"
        backoff=2
        sleep 2
    fi
done

rm -f "$SHUTDOWN_FLAG" "$CHILD_PID_FILE"
echo "supervisor: shutdown flag seen; exiting" >> "$STDERR"
