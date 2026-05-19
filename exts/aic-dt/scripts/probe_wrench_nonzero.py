#!/usr/bin/env python3
"""probe_wrench_nonzero.py — PARITY-05 (wrench-rootcause) verify gate.

Connects to the running aic-dt MCP socket on :8768, asks for a quick_start
to ensure scene + force publisher graph are alive, then subscribes to
/fts_broadcaster/wrench via host zenoh transport (post-zenoh-path-implementation
the host runs RMW=rmw_zenoh_cpp). Records wrench samples for --window-seconds,
asserts at least one sample's force.z magnitude exceeds --min-z-magnitude.

Acceptance per prd.json wrench-rootcause:
  - /fts_broadcaster/wrench publishes nonzero force/torque on idle gravity-
    loaded robot (idle weight should produce nonzero z-axis).

Exit 0 if pass, 1 if no samples / all zero / connection error.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time

PORT = int(os.environ.get("AIC_DT_MCP_PORT", "8768"))


def _send(cmd: str, params: dict | None = None, timeout: float = 180.0) -> dict:
    s = socket.socket()
    s.settimeout(timeout)
    s.connect(("localhost", PORT))
    s.sendall(json.dumps({"type": cmd, "params": params or {}}).encode())
    buf = b""
    while True:
        chunk = s.recv(8192)
        if not chunk:
            break
        buf += chunk
        try:
            r = json.loads(buf.decode())
            break
        except Exception:
            continue
    s.close()
    return r


def _ensure_quick_start() -> bool:
    """Ensure quick_start has run. Tolerate the benign 'name not unique'
    re-quick_start error — drives + force-publisher are scene-resident."""
    r = _send("quick_start", {})
    if r.get("status") == "success":
        return True
    msg = (r.get("message") or "") + " " + ((r.get("result") or {}).get("error") or "") + " " + (r.get("error") or "")
    if "not unique" in msg or "already" in msg.lower():
        print(f"[probe_wrench] quick_start idempotency tolerated: {msg.strip()}")
        return True
    print(f"[probe_wrench] quick_start failed: {r}")
    return False


def _subscribe_and_collect(window_seconds: float) -> list[dict]:
    """Spawn an rclpy subscriber subprocess under host zenoh transport;
    collect wrench samples for window_seconds; return list of dicts.

    Mirrors probe_insertion_event_fires.py's rclpy-subprocess pattern —
    the parent process can't import rclpy without polluting Isaac Sim's
    Python env, so we shell out under env_isaaclab + RMW=zenoh.
    """
    code = r"""
import json, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped

# Isaac Sim's OmniGraph ROS2Publisher uses BEST_EFFORT for sensor-style topics.
# Subscriber must match or zenoh transport silently drops every message.
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

samples = []
def _cb(msg):
    samples.append({
        'frame_id': msg.header.frame_id,
        'fx': float(msg.wrench.force.x),
        'fy': float(msg.wrench.force.y),
        'fz': float(msg.wrench.force.z),
        'tx': float(msg.wrench.torque.x),
        'ty': float(msg.wrench.torque.y),
        'tz': float(msg.wrench.torque.z),
    })

rclpy.init()
# Unique name avoids any cached-zenoh registration issue across runs.
node = rclpy.create_node('probe_wrench_sub_' + str(int(time.time())))
sub = node.create_subscription(WrenchStamped, '/fts_broadcaster/wrench', _cb, qos)
# Zenoh peer discovery needs ~1s before the subscriber receives messages.
# Without this settle, BEST_EFFORT samples emitted during discovery are
# silently dropped and the probe collects 0 even with a healthy publisher.
time.sleep(1.0)
deadline = time.time() + WINDOW
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
print('PROBE_RESULT:' + json.dumps(samples))
node.destroy_node()
rclpy.shutdown()
""".replace("WINDOW", str(window_seconds))

    # Sourcing /opt/ros/humble/setup.bash sets RMW_IMPLEMENTATION=rmw_fastrtps_cpp,
    # which clobbers any pre-set env. Export the zenoh trio AFTER the sources.
    # Strip any inherited RMW/ZENOH/ROS_LOCALHOST_ONLY from the parent so
    # the bash chain is the sole authority on transport vars. Parent harness
    # env can carry rmw_cyclonedds_cpp + ROS_LOCALHOST_ONLY=1 + ROS_DOMAIN_ID=0
    # (a live host UR5e driver), which silently hides Isaac Sim's domain-7
    # publishers from this subscriber.
    env = os.environ.copy()
    for v in ("RMW_IMPLEMENTATION", "ZENOH_ROUTER_CHECK_ATTEMPTS",
              "ZENOH_CONFIG_OVERRIDE", "ROS_LOCALHOST_ONLY"):
        env.pop(v, None)
    # Force ROS_DOMAIN_ID=7 (Isaac Sim's sim-isolation domain per CLAUDE.md);
    # never inherit the parent's domain (often 0 from a host robot driver).
    env["ROS_DOMAIN_ID"] = os.environ.get("AIC_DT_ROS_DOMAIN_ID", "7")
    cmd = [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source ~/env_isaaclab/bin/activate && "
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp "
        "ZENOH_ROUTER_CHECK_ATTEMPTS=-1 "
        "ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/localhost:7447\"];"
        "transport/shared_memory/enabled=false' && "
        "python3 -c \"$0\"",
        code,
    ]
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True,
                          timeout=window_seconds + 30)
    samples: list[dict] = []
    for line in proc.stdout.splitlines():
        if line.startswith("PROBE_RESULT:"):
            samples = json.loads(line[len("PROBE_RESULT:"):])
            break
    if not samples:
        print(f"[probe_wrench] subscriber rc={proc.returncode}")
        print(f"[probe_wrench] subscriber stdout tail:\n{proc.stdout[-1000:]}")
        print(f"[probe_wrench] subscriber stderr tail:\n{proc.stderr[-1500:]}")
    return samples


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window-seconds", type=float, default=5.0)
    ap.add_argument("--min-z-magnitude", type=float, default=0.5,
                    help="Minimum |force.z| in PhysX units required across the sample window.")
    args = ap.parse_args()

    if not _ensure_quick_start():
        return 1

    # Brief settle so the lazy_init warmup (30 ticks ≈ 0.5s) clears and the
    # publisher emits real values, not the OG default-zero attributes.
    settle = 4.0
    print(f"[probe_wrench] settle {settle}s for force publisher lazy-init...")
    time.sleep(settle)

    samples = _subscribe_and_collect(args.window_seconds)
    print(f"[probe_wrench] collected {len(samples)} samples over "
          f"{args.window_seconds}s window")
    if not samples:
        print("[probe_wrench] FAIL: no /fts_broadcaster/wrench samples received")
        return 1

    max_fz = max(abs(s["fz"]) for s in samples)
    nonzero = sum(1 for s in samples
                  if any(abs(s[k]) > 1e-9 for k in ("fx","fy","fz","tx","ty","tz")))
    print(f"[probe_wrench] nonzero samples: {nonzero}/{len(samples)}; max |fz|={max_fz:.4f}")
    print(f"[probe_wrench] first sample: {samples[0]}")
    print(f"[probe_wrench] last  sample: {samples[-1]}")

    if max_fz < args.min_z_magnitude:
        print(f"[probe_wrench] FAIL: max |fz|={max_fz} < min={args.min_z_magnitude}")
        return 1
    if nonzero == 0:
        print("[probe_wrench] FAIL: every sample is all-zero")
        return 1

    print(f"[probe_wrench] PASS: max |fz|={max_fz:.4f} >= {args.min_z_magnitude}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
