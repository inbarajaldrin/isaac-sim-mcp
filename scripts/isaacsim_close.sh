#!/bin/bash
# Gracefully close Isaac Sim.
# Usage: isaacsim_close.sh [--force]
#   --force: Skip SIGTERM, go straight to SIGKILL

set -euo pipefail

FORCE="${1:-}"

# Use pgrep with exact pattern to avoid matching this script's own process
_isaacsim_running() {
    pgrep -f "[i]saacsim" > /dev/null 2>&1
}

_wait_for_exit() {
    local max_wait="$1"
    for i in $(seq 1 "$max_wait"); do
        if ! _isaacsim_running; then
            return 0
        fi
        sleep 1
    done
    return 1
}

if ! _isaacsim_running; then
    echo "Isaac Sim is not running"
    exit 0
fi

if [ "$FORCE" = "--force" ]; then
    echo "Force killing Isaac Sim..."
    pkill -9 -f "[i]saacsim" 2>/dev/null || true
    pkill -9 -f "[k]it" 2>/dev/null || true
    _wait_for_exit 5
else
    echo "Gracefully stopping Isaac Sim (SIGTERM)..."
    pkill -15 -f "[i]saacsim" 2>/dev/null || true
    if _wait_for_exit 10; then
        echo "Isaac Sim stopped gracefully"
        exit 0
    fi
    echo "Still running after 10s, sending SIGKILL..."
    pkill -9 -f "[i]saacsim" 2>/dev/null || true
    pkill -9 -f "[k]it" 2>/dev/null || true
    _wait_for_exit 5
fi

if _isaacsim_running; then
    echo "WARNING: Isaac Sim processes still running"
    ps aux | grep "[i]saacsim"
    exit 1
else
    echo "Isaac Sim stopped"
fi
