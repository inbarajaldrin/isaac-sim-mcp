#!/usr/bin/env bash
set -u

echo "[1/5] Checking for running Isaac Sim / Kit processes..."
PIDS=$(pgrep -f 'isaac-sim|kit/kit|omni\.isaac|omni\.kit\.app' || true)
if [ -n "$PIDS" ]; then
    echo "  Found Kit processes: $PIDS"
    echo "  Sending SIGTERM (graceful)..."
    kill -15 $PIDS 2>/dev/null || true
    for i in 1 2 3 4 5; do
        sleep 1
        REMAIN=$(pgrep -f 'isaac-sim|kit/kit|omni\.isaac|omni\.kit\.app' || true)
        [ -z "$REMAIN" ] && break
    done
    REMAIN=$(pgrep -f 'isaac-sim|kit/kit|omni\.isaac|omni\.kit\.app' || true)
    if [ -n "$REMAIN" ]; then
        echo "  Still alive after 5s: $REMAIN -- sending SIGTERM again"
        kill -15 $REMAIN 2>/dev/null || true
        sleep 2
    fi
else
    echo "  None running. Good."
fi

echo "[2/5] Checking who is holding /dev/nvidia-uvm..."
HOLDERS=$(sudo lsof /dev/nvidia-uvm 2>/dev/null | awk 'NR>1 {print $2":"$1}' | sort -u)
if [ -n "$HOLDERS" ]; then
    echo "  Current holders (pid:cmd):"
    echo "$HOLDERS" | sed 's/^/    /'
    echo "  NOTE: processes like Xorg / firefox may legitimately hold it."
    echo "        We will try rmmod anyway; if it fails we stop and let you decide."
else
    echo "  Nothing holding it."
fi

echo "[3/5] Reloading nvidia_uvm kernel module..."
if sudo rmmod nvidia_uvm 2>/tmp/rmmod.err; then
    echo "  rmmod nvidia_uvm: OK"
    if sudo modprobe nvidia_uvm; then
        echo "  modprobe nvidia_uvm: OK"
    else
        echo "  !! modprobe nvidia_uvm FAILED -- you likely need to reboot."
        exit 1
    fi
else
    echo "  !! rmmod nvidia_uvm FAILED:"
    sed 's/^/     /' /tmp/rmmod.err
    echo "  Module is in use. Options:"
    echo "    - Close browsers / any app using CUDA and rerun this script"
    echo "    - Reboot (most reliable after ERROR_DEVICE_LOST)"
    exit 1
fi

echo "[4/5] Clearing Omniverse shader cache (may be corrupt from the crash)..."
if [ -d "$HOME/.cache/ov/shaders" ]; then
    SIZE=$(du -sh "$HOME/.cache/ov/shaders" 2>/dev/null | awk '{print $1}')
    rm -rf "$HOME/.cache/ov/shaders"
    echo "  Cleared ~/.cache/ov/shaders ($SIZE). It will rebuild on first launch."
else
    echo "  No shader cache dir -- skipping."
fi

echo "[5/5] Sanity check: CUDA device enumeration..."
if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.used,memory.total --format=csv
fi
# Tiny CUDA context test via python if available
python3 - <<'PY' 2>/dev/null || echo "  (skipping python CUDA probe)"
import ctypes, sys
try:
    cuda = ctypes.CDLL("libcuda.so.1")
    r = cuda.cuInit(0)
    count = ctypes.c_int()
    r2 = cuda.cuDeviceGetCount(ctypes.byref(count))
    if r == 0 and r2 == 0:
        print(f"  cuInit OK, CUDA device count = {count.value}")
    else:
        print(f"  !! cuInit={r} cuDeviceGetCount={r2} (nonzero = still wedged -- reboot)")
        sys.exit(2)
except Exception as e:
    print(f"  cuda probe failed: {e}")
PY

echo
echo "Done. Try launching Isaac Sim now."
echo "If CUDA error 999 returns on startup, reboot -- the GSP firmware can stay"
echo "stuck across nvidia_uvm reloads after ERROR_DEVICE_LOST."
