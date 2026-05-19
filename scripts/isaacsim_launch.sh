#!/bin/bash
# Launch, restart, close, and check Isaac Sim with an extension.
#
# Usage:
#   isaacsim_launch.sh launch <ext-id>   # Launch Isaac Sim with extension
#   isaacsim_launch.sh close             # Graceful shutdown
#   isaacsim_launch.sh kill              # Force kill
#   isaacsim_launch.sh restart <ext-id>  # Close then launch
#   isaacsim_launch.sh status            # Check if running + socket
#   isaacsim_launch.sh wait [timeout]    # Block until socket ready (default 120s)
#
# Extension IDs: soarm101-dt, ur5e-dt, aic-dt
# Log: /tmp/isaacsim.log

ISAACSIM_BIN="$HOME/env_isaaclab/bin/isaacsim"
EXT_FOLDER="$HOME/Documents/isaac-sim-mcp/exts"
LOG="/tmp/isaacsim.log"
PIDFILE="/tmp/isaacsim.pid"

get_port() {
    case "$1" in
        soarm101-dt) echo 8767 ;;
        ur5e-dt)     echo 8766 ;;
        aic-dt)      echo 8768 ;;
        *)           echo 8767 ;;
    esac
}

check_socket() {
    local port="${1:-8767}"
    python3 -c "
import socket
s=socket.socket();s.settimeout(3)
try:
    s.connect(('localhost',$port))
except Exception:
    raise SystemExit(1)
finally:
    s.close()
" 2>/dev/null
    return $?
}

is_running() {
    pgrep -f "bin/isaacsim" > /dev/null 2>&1
    return $?
}

case "${1:-help}" in
    launch)
        EXT="${2:-soarm101-dt}"
        PORT=$(get_port "$EXT")

        if is_running; then
            echo "Isaac Sim already running"
            if check_socket "$PORT"; then
                echo "  Socket: RESPONSIVE on $PORT"
            else
                echo "  Waiting for socket on $PORT..."
                "$0" wait 120 "$PORT"
            fi
            exit $?
        fi

        echo "Launching Isaac Sim: $EXT (port $PORT)"
        export DISPLAY="${DISPLAY:-:0}"
        nohup "$ISAACSIM_BIN" --ext-folder "$EXT_FOLDER" --enable "$EXT" > "$LOG" 2>&1 &
        echo "$!" > "$PIDFILE"
        echo "  PID: $!"
        echo "  Log: $LOG"
        sleep 3
        if ! kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
            echo "  ERROR: exited immediately — check $LOG"
            exit 1
        fi
        echo "  Status: running"
        "$0" wait 120 "$PORT"
        ;;

    close)
        if ! is_running; then
            echo "Isaac Sim is not running"
            rm -f "$PIDFILE"
            exit 0
        fi
        echo "Closing Isaac Sim (SIGTERM)..."
        pkill -15 -f "bin/isaacsim" 2>/dev/null || true
        pkill -15 -f "kit" 2>/dev/null || true
        for i in 1 2 3 4 5 6 7 8 9 10; do
            is_running || { echo "Stopped (${i}s)."; rm -f "$PIDFILE"; exit 0; }
            sleep 1
        done
        echo "Force killing (SIGKILL)..."
        pkill -9 -f "bin/isaacsim" 2>/dev/null || true
        pkill -9 -f "kit" 2>/dev/null || true
        pkill -9 -f "carb" 2>/dev/null || true
        for i in 1 2 3 4 5; do
            is_running || { echo "Killed (${i}s after SIGKILL)."; rm -f "$PIDFILE"; exit 0; }
            sleep 1
        done
        rm -f "$PIDFILE"
        echo "WARNING: still running"
        exit 1
        ;;

    kill)
        pkill -9 -f "bin/isaacsim" 2>/dev/null || true
        pkill -9 -f "kit" 2>/dev/null || true
        pkill -9 -f "carb" 2>/dev/null || true
        rm -f "$PIDFILE"
        sleep 1
        echo "Killed."
        ;;

    restart)
        EXT="${2:-soarm101-dt}"
        "$0" close
        # Wait for all isaacsim processes to fully exit before relaunching
        while is_running; do sleep 1; done
        "$0" launch "$EXT"
        ;;

    status)
        if is_running; then
            echo "Isaac Sim: RUNNING"
            for ext in soarm101-dt ur5e-dt aic-dt; do
                p=$(get_port "$ext")
                check_socket "$p" && echo "  $ext ($p): RESPONSIVE"
            done
        else
            echo "Isaac Sim: NOT RUNNING"
        fi
        ;;

    wait)
        TIMEOUT="${2:-60}"
        PORT="${3:-8767}"
        echo "Waiting up to ${TIMEOUT}s for port $PORT..."
        ELAPSED=0
        while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
            if check_socket "$PORT"; then
                echo "READY (${ELAPSED}s)"
                exit 0
            fi
            if [ -f "$PIDFILE" ]; then
                kill -0 "$(cat $PIDFILE)" 2>/dev/null || { echo "ERROR: process exited. Check $LOG"; exit 1; }
            fi
            sleep 5
            ELAPSED=$((ELAPSED + 5))
        done
        echo "TIMEOUT (${TIMEOUT}s)"
        exit 1
        ;;

    *)
        echo "Usage: $0 {launch|close|kill|restart|status|wait} [ext-id]"
        echo ""
        echo "  launch <ext-id>       Launch and block until socket ready (60s timeout)"
        echo "  close                 Graceful SIGTERM → SIGKILL, blocks until stopped"
        echo "  kill                  Force SIGKILL"
        echo "  restart <ext-id>      Close then launch (blocks until ready)"
        echo "  status                Process + socket check"
        echo "  wait [timeout] [port] Block until socket ready (default 60s)"
        echo ""
        echo "Extensions: soarm101-dt (8767), ur5e-dt (8766), aic-dt (8768)"
        ;;
esac
exit 0
