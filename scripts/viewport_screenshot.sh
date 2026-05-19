#!/usr/bin/env bash
# Capture the active viewport of Isaac Sim or Gazebo via the proper renderer
# API — NOT via X11 window grab. Both targets write a PNG to an output path.
#
# ISAAC SIM:
#   Uses omni.kit.viewport.utility.capture_viewport_to_file() via the
#   running aic-dt MCP socket (default port 8768). The active viewport's
#   camera is whichever prim was last set via vp.camera_path.
#
# GAZEBO:
#   Uses the gz-gui `Screenshot` plugin's /gui/screenshot service. REQUIRES
#   the plugin to be declared in the world's <gui> block, e.g.:
#       <plugin filename="Screenshot" name="Screenshot"/>
#   Argument quirks (these caused silent fails first time around):
#     - reqtype = gz.msgs.StringMsg, REPLY type = gz.msgs.Boolean
#     - `data` field is a DIRECTORY, NOT a file path. The plugin auto-names
#       the PNG with an ISO timestamp.
#     - If you pass a file path here, service returns `data: true` but
#       writes nothing.
#
# Usage:
#   bash viewport_screenshot.sh isaacsim /tmp/isaac.png           [PORT]
#   bash viewport_screenshot.sh gazebo  /tmp/gazebo_dir/          [CONTAINER]
#   bash viewport_screenshot.sh both    /tmp/aic_caps/            [ISAAC_PORT GZ_CONTAINER]
#
# Defaults:
#   PORT          = 8768  (aic-dt)
#   CONTAINER     = aic_eval
#
# Both calls block until the PNG exists on disk.

set -euo pipefail

mode="${1:-}"
out="${2:-}"
arg3="${3:-}"
arg4="${4:-}"

usage() {
    echo "Usage: $0 {isaacsim|gazebo|both} <out-file-or-dir> [extras]" >&2
    exit 2
}

capture_isaacsim() {
    local target_path="$1"
    local port="${2:-8768}"
    mkdir -p "$(dirname "$target_path")"
    python3 - <<PY
import socket, json
def send(p, t=60):
    s = socket.socket(); s.settimeout(t); s.connect(("localhost", $port))
    s.sendall(json.dumps(p).encode()); b=b""
    while True:
        c = s.recv(16384)
        if not c: break
        b += c
        try: return json.loads(b.decode())
        except: continue
# Optional resolution override via env: ISAAC_CAP_W / ISAAC_CAP_H
import os
w = os.environ.get("ISAAC_CAP_W")
h = os.environ.get("ISAAC_CAP_H")
res_block = ""
if w and h:
    res_block = f"vp.resolution = ({w}, {h})"
code = f"""
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
vp = get_active_viewport()
{res_block}
capture_viewport_to_file(vp, "$target_path")
result = {{'viewport_camera': str(vp.camera_path), 'output_path': "$target_path", 'resolution': str(vp.resolution)}}
"""
r = send({"type": "execute_python_code", "params": {"code": code}})
print(json.dumps(r.get('result', r)))
PY
    # Wait for the file to flush (RTX writes async)
    until [ -s "$target_path" ]; do sleep 0.5; done
    echo "[isaacsim] saved $target_path ($(stat -c%s "$target_path") bytes)"
}

capture_gazebo() {
    local target_dir="$1"
    local container="${2:-aic_eval}"
    mkdir -p "$target_dir"
    # container-side: write into /tmp/screenshots, then we copy out
    docker exec "$container" bash -lc 'mkdir -p /tmp/screenshots'
    docker exec "$container" bash -lc \
        "source /ws_aic/install/setup.bash 2>/dev/null; \
         gz service -s /gui/screenshot \
           --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
           --timeout 5000 --req 'data: \"/tmp/screenshots\"'" \
        >/dev/null
    # Wait for fresh PNG to appear (timestamped) — pick the newest
    sleep 1
    local newest
    newest=$(docker exec "$container" bash -lc 'ls -t /tmp/screenshots/*.png 2>/dev/null | head -1')
    if [ -z "$newest" ]; then
        echo "[gazebo] no PNG written — is the Screenshot plugin in aic.sdf <gui>?" >&2
        return 1
    fi
    local host_out="$target_dir/$(basename "$newest")"
    docker cp "$container:$newest" "$host_out"
    echo "[gazebo] saved $host_out ($(stat -c%s "$host_out") bytes)"
}

case "$mode" in
    isaacsim) [ -z "$out" ] && usage; capture_isaacsim "$out" "${arg3:-8768}" ;;
    gazebo)   [ -z "$out" ] && usage; capture_gazebo  "$out" "${arg3:-aic_eval}" ;;
    both)
        [ -z "$out" ] && usage
        capture_isaacsim "$out/isaac_viewport.png" "${arg3:-8768}"
        capture_gazebo   "$out" "${arg4:-aic_eval}"
        ;;
    *) usage ;;
esac
