#!/bin/bash
# Canonical ur5e-dt sim launch. Sources config/ros_dds.env (Cyclone + ROS_LOCALHOST_ONLY=1
# + domain 7) so Isaac joins the SAME DDS island as ros-mcp-server + the UR driver --
# otherwise the consumer can't discover Isaac's /gripper_width_sim, /objects_poses_sim, etc.
# Then delegates to the skill launcher. Default action: launch ur5e-dt.
#
# Usage: scripts/launch_sim.sh [launch <ext> | close | status | restart <ext> | wait ...]
#   View (default webrtc): VIEW=webrtc|window|headless scripts/launch_sim.sh launch <ext>
#   e.g. VIEW=window scripts/launch_sim.sh launch ur5e-dt   (headed, for use at the Linux box)
#
# This host is en4226769-l == dual-a4500 (tailscale 100.80.151.46). webrtc streams on
# 100.80.151.46:49100; a4500 streams on 100.97.45.92:49100 — same port, different IP, no
# conflict. The Mac NVIDIA Streaming Client connects to whichever IP it's pointed at (the
# fuzzydroid isaacsim watchdog's server_ip).
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source ~/env_isaaclab/bin/activate
# Source ROS 2 so the in-Isaac ros2.bridge extension starts (needs ROS_DISTRO etc.).
# Non-interactive shells (ssh "cmd", automation) skip ~/.bashrc, so do it here.
[ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ] && { source /opt/ros/humble/setup.bash || true; }
source "$REPO/config/ros_dds.env"
export DISPLAY="${DISPLAY:-:0}"
# Suppress .pyc clutter at the source (Isaac's imports would litter __pycache__ everywhere).
export PYTHONDONTWRITEBYTECODE=1
[ $# -eq 0 ] && set -- launch ur5e-dt

# Base flags applied in EVERY view mode. Isaac Sim 5.1 async-rendering deadlock fix (NVIDIA
# forum 349513, verified on this host 2026-06-17): the full experience file enables
# /app/asyncRendering via isaacsim.core.throttling and reset()/play() deadlocks compiling an
# external MDL material. Disabling enable_async at launch prevents the hang in quick_start ->
# load_ur5e -> world.reset_async. NOT a cache problem. (a4500 only sets this in webrtc; we set
# it everywhere because this host reproduces the deadlock in non-webrtc launches too.)
ISAACSIM_BASE_ARGS="--/exts/isaacsim.core.throttling/enable_async=false"

# View mode — how/whether you SEE the sim. The MCP socket (8766 ur5e-dt) is IDENTICAL in all
# three; VIEW only changes display. Set VIEW=webrtc|window|headless.
#   webrtc   (default) headless + omni.kit.livestream.webrtc stream — view from the Mac's Isaac
#                      WebRTC Streaming Client at the printed tailscale IP:49100.
#   window             headed X window on $DISPLAY — for when you're AT the Linux box.
#   headless           --no-window, no stream — lean (CI / AFK).
# Extension path (--enable omni.kit.livestream.webrtc) on the DEFAULT "Isaac-Sim Full" app, NOT
# the isaacsim.exp.full.streaming experience (which is a separate Kit app / user.config).
VIEW="${VIEW:-webrtc}"
if [ "${1:-}" = "launch" ]; then
  case "$VIEW" in
    window)   export ISAACSIM_EXTRA_ARGS="${ISAACSIM_EXTRA_ARGS:-} $ISAACSIM_BASE_ARGS" ;;
    headless) export ISAACSIM_EXTRA_ARGS="${ISAACSIM_EXTRA_ARGS:-} $ISAACSIM_BASE_ARGS --no-window" ;;
    webrtc)
      # publicEndpointAddress = the reachable IP a remote (Mac/Tailscale) client dials; without
      # it Kit advertises 0.0.0.0 with zero ICE candidates and the peer can't connect.
      # Per-machine: ISAACSIM_STREAM_IP > tailscale IP > 127.0.0.1. APP-namespaced key only --
      # the EXT key --/exts/.../publicIp breaks NVST.
      STREAM_IP="${ISAACSIM_STREAM_IP:-$(command -v tailscale >/dev/null 2>&1 && tailscale ip -4 2>/dev/null | head -1)}"
      STREAM_IP="${STREAM_IP:-127.0.0.1}"
      STREAM_PORT="${ISAACSIM_STREAM_PORT:-49100}"
      echo "[launch_sim] webrtc publicEndpointAddress=$STREAM_IP:$STREAM_PORT (point the Mac client / fuzzydroid server_ip here)"
      # Block the NVStreamer-*.etli leak at the OS write layer (no kit setting gates it; point
      # outDirectory at a non-writable dir -> writer's openat fails EACCES, NVST gives up).
      ETLI_DIR="${ISAACSIM_NVST_DIR:-/tmp/nvst-blocked}"
      mkdir -p "$ETLI_DIR" 2>/dev/null || true
      chmod 0555 "$ETLI_DIR" 2>/dev/null || true
      export ISAACSIM_EXTRA_ARGS="${ISAACSIM_EXTRA_ARGS:-} $ISAACSIM_BASE_ARGS --enable omni.kit.livestream.webrtc --no-window --/app/livestream/publicEndpointAddress=$STREAM_IP --/app/livestream/port=$STREAM_PORT --/app/livestream/outDirectory=$ETLI_DIR --/persistent/omni/replicator/captureOnPlay=0" ;;
    *) echo "ERROR: VIEW must be webrtc|window|headless (got '$VIEW')" >&2; exit 1 ;;
  esac
fi

# Pre-flight cache heal (a4500 parity) — only if the script exists locally; non-fatal.
if [ "$1" = "launch" ] && [ -f "$REPO/scripts/prime_usd_cache.py" ]; then
  python "$REPO/scripts/prime_usd_cache.py" ensure || true
fi
exec bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh "$@"
