#!/usr/bin/env python3
"""probe_pose_roundtrip.py — PARITY-10 (pose-roundtrip-verify) verify gate.

Connects to running aic-dt MCP socket on :8768, ensures quick_start, then
exercises the Cartesian command path end-to-end:

  1. Lookup TF base_link -> tool0 to capture (x0, y0, z0).
  2. Publish MotionUpdate to /aic_controller/pose_commands with
     pose.position.z = z0 + delta (default delta = -0.05),
     header.frame_id = "base_link", mode = MODE_POSITION (=2),
     orientation = identity quaternion.
  3. Hold the publish at 30Hz for `--hold-seconds` (default 3s) so the buffered
     pose_cmd is not staled out by an empty re-publish, and so Lula IK + the
     three-step articulation write have time to converge.
  4. Re-lookup TF base_link -> tool0 to capture the final pose.
  5. Pass if signed_delta along the chosen axis matches the commanded delta
     within `--tolerance` (default 0.005 m, i.e. >=90% of a 0.05 m command).

Subscribes /tf + /tf_static under host zenoh transport (RMW=rmw_zenoh_cpp,
peer = tcp/localhost:7447) — Isaac Sim publishes via zenoh per
zenoh-path-implementation; a fastrtps subscriber would silently drop every msg.

Exit code 0 = pass, 1 = fail / setup error.
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
    """Tolerate the benign 'ur5e_view name not unique' re-quick_start error —
    pose-cmd surface is module-resident regardless."""
    r = _send("quick_start", {})
    if r.get("status") == "success":
        return True
    msg = (r.get("message") or "") + " " + ((r.get("result") or {}).get("error") or "") + " " + (r.get("error") or "")
    if "not unique" in msg or "already" in msg.lower():
        print(f"[probe_pose] quick_start idempotency tolerated: {msg.strip()}")
        return True
    print(f"[probe_pose] quick_start failed: {r}")
    return False


def _run_pose_roundtrip(axis: str, delta: float, hold_seconds: float,
                        warmup_seconds: float, stiffness: float, damping: float) -> dict:
    """Spawn an rclpy subprocess under host zenoh transport that:
      - listens to TF for base_link -> tool0,
      - publishes a single Cartesian pose target at 30Hz for hold_seconds,
      - returns initial/final translation as JSON.

    Mirrors probe_wrench_nonzero's subprocess pattern (parent can't import
    rclpy without polluting Isaac Sim's Python env).
    """
    code = r"""
import json, time, sys
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from aic_control_interfaces.msg import MotionUpdate

AXIS = "AXIS_VAL"
DELTA = DELTA_VAL
HOLD = HOLD_VAL
WARMUP = WARMUP_VAL
STIFF = STIFF_VAL
DAMP = DAMP_VAL

rclpy.init()
node = rclpy.create_node('probe_pose_roundtrip_' + str(int(time.time())))

# TF buffer auto-subscribes /tf (BEST_EFFORT) + /tf_static (RELIABLE,
# TRANSIENT_LOCAL) via TransformListener defaults — matches Isaac Sim's
# OG ROS2PublishTransformTree QoS exactly.
tf_buf = Buffer()
tf_listener = TransformListener(tf_buf, node)

pose_pub = node.create_publisher(MotionUpdate, '/aic_controller/pose_commands', 10)

# Warmup: let TF tree assemble + zenoh peer discovery settle.
deadline = time.time() + WARMUP
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

def _lookup():
    # Time() = latest available; tolerate ExtrapolationException for ~1s.
    end = time.time() + 1.5
    last_exc = None
    while time.time() < end:
        try:
            t = tf_buf.lookup_transform('base_link', 'tool0', Time())
            return t
        except Exception as exc:
            last_exc = exc
            rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError('lookup_transform base_link->tool0 timed out: ' + repr(last_exc))

initial = _lookup()
ix = float(initial.transform.translation.x)
iy = float(initial.transform.translation.y)
iz = float(initial.transform.translation.z)
qx = float(initial.transform.rotation.x)
qy = float(initial.transform.rotation.y)
qz = float(initial.transform.rotation.z)
qw = float(initial.transform.rotation.w)

# Build a Cartesian target: same orientation as current tool0, but axis += delta.
tx, ty, tz = ix, iy, iz
if AXIS == 'x': tx = ix + DELTA
elif AXIS == 'y': ty = iy + DELTA
elif AXIS == 'z': tz = iz + DELTA
else:
    print('PROBE_RESULT:' + json.dumps({'error': 'bad axis ' + AXIS}))
    sys.exit(1)

cmd = MotionUpdate()
cmd.header.frame_id = 'base_link'
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.pose.position.x = tx
cmd.pose.position.y = ty
cmd.pose.position.z = tz
# Preserve current end-effector orientation — IK should converge cleanly without
# re-orienting the wrist (the task is a pure Cartesian translation along one axis).
cmd.pose.orientation.x = qx
cmd.pose.orientation.y = qy
cmd.pose.orientation.z = qz
cmd.pose.orientation.w = qw
cmd.trajectory_generation_mode.mode = 2  # MODE_POSITION
# Cartesian impedance fields (target_stiffness 6x6, etc.) are ignored by
# _apply_pose_cmd per D-06 (controller-side math); leave at default zero.

# Publish at 30Hz for HOLD seconds. controller_loop one-shots
# self._latest_pose_cmd after apply, so a continuous stream keeps the buffered
# cmd fresh and Lula IK + the three-step write are exercised every tick.
period = 1.0 / 30.0
end = time.time() + HOLD
while time.time() < end:
    cmd.header.stamp = node.get_clock().now().to_msg()
    pose_pub.publish(cmd)
    spin_until = time.time() + period
    while time.time() < spin_until:
        rclpy.spin_once(node, timeout_sec=0.005)

# Brief settle after publish ends so the last applied IK target reflects in TF.
deadline = time.time() + 0.3
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

final = _lookup()
fx = float(final.transform.translation.x)
fy = float(final.transform.translation.y)
fz = float(final.transform.translation.z)

print('PROBE_RESULT:' + json.dumps({
    'initial': {'x': ix, 'y': iy, 'z': iz, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw},
    'commanded': {'x': tx, 'y': ty, 'z': tz},
    'final':   {'x': fx, 'y': fy, 'z': fz},
}))
node.destroy_node()
rclpy.shutdown()
"""
    code = (code
            .replace("AXIS_VAL", axis)
            .replace("DELTA_VAL", repr(float(delta)))
            .replace("HOLD_VAL", repr(float(hold_seconds)))
            .replace("WARMUP_VAL", repr(float(warmup_seconds)))
            .replace("STIFF_VAL", repr(float(stiffness)))
            .replace("DAMP_VAL", repr(float(damping))))

    env = os.environ.copy()
    for v in ("RMW_IMPLEMENTATION", "ZENOH_ROUTER_CHECK_ATTEMPTS",
              "ZENOH_CONFIG_OVERRIDE", "ROS_LOCALHOST_ONLY"):
        env.pop(v, None)
    env["ROS_DOMAIN_ID"] = os.environ.get("AIC_DT_ROS_DOMAIN_ID", "7")
    cmd_argv = [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source ~/env_isaaclab/bin/activate && "
        # Workspace overlay — aic_control_interfaces is built in the humble_ws
        # used by the controller_loop publishers. Sourcing it here ensures the
        # MotionUpdate type-hash matches the publisher in extension.py.
        "[ -f ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash ] && "
        "source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash; "
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp "
        "ZENOH_ROUTER_CHECK_ATTEMPTS=-1 "
        "ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/localhost:7447\"];"
        "transport/shared_memory/enabled=false' && "
        "python3 -c \"$0\"",
        code,
    ]
    timeout = warmup_seconds + hold_seconds + 30.0
    proc = subprocess.run(cmd_argv, env=env, capture_output=True, text=True,
                          timeout=timeout)
    payload: dict = {}
    for line in proc.stdout.splitlines():
        if line.startswith("PROBE_RESULT:"):
            payload = json.loads(line[len("PROBE_RESULT:"):])
            break
    if not payload:
        print(f"[probe_pose] subscriber rc={proc.returncode}")
        print(f"[probe_pose] subscriber stdout tail:\n{proc.stdout[-1500:]}")
        print(f"[probe_pose] subscriber stderr tail:\n{proc.stderr[-2000:]}")
    return payload


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--axis", choices=("x", "y", "z"), default="z",
                    help="Cartesian axis (in base_link frame) to translate along.")
    ap.add_argument("--delta", type=float, default=-0.05,
                    help="Commanded translation in metres (default -0.05 = 5 cm down).")
    ap.add_argument("--tolerance", type=float, default=0.005,
                    help="Maximum |actual_delta - commanded_delta| in metres "
                         "(default 0.005 = 90%% of a 0.05 m command).")
    ap.add_argument("--hold-seconds", type=float, default=3.0,
                    help="Continuous 30Hz pose publish window (s).")
    ap.add_argument("--warmup-seconds", type=float, default=2.5,
                    help="TF/zenoh discovery warmup before first lookup (s).")
    ap.add_argument("--stiffness", type=float, default=200.0,
                    help="Reserved (Cartesian impedance ignored by D-06 sink).")
    ap.add_argument("--damping", type=float, default=20.0,
                    help="Reserved (Cartesian impedance ignored by D-06 sink).")
    args = ap.parse_args()

    if not _ensure_quick_start():
        return 1

    print(f"[probe_pose] running pose round-trip: axis={args.axis} delta={args.delta:+.4f} m "
          f"hold={args.hold_seconds}s tol={args.tolerance}m")
    payload = _run_pose_roundtrip(
        args.axis, args.delta, args.hold_seconds, args.warmup_seconds,
        args.stiffness, args.damping,
    )
    if not payload or "initial" not in payload or "final" not in payload:
        print(f"[probe_pose] FAIL: no PROBE_RESULT — payload={payload}")
        return 1

    initial = payload["initial"]
    final = payload["final"]
    commanded = payload["commanded"]
    actual_delta = final[args.axis] - initial[args.axis]
    err = abs(actual_delta - args.delta)
    pct = 100.0 * actual_delta / args.delta if args.delta != 0 else 0.0

    print(f"[probe_pose] initial base_link->tool0: "
          f"x={initial['x']:+.5f} y={initial['y']:+.5f} z={initial['z']:+.5f}")
    print(f"[probe_pose] commanded target: "
          f"x={commanded['x']:+.5f} y={commanded['y']:+.5f} z={commanded['z']:+.5f}")
    print(f"[probe_pose] final   base_link->tool0: "
          f"x={final['x']:+.5f} y={final['y']:+.5f} z={final['z']:+.5f}")
    print(f"[probe_pose] axis={args.axis}: actual_delta={actual_delta:+.5f} m "
          f"(commanded={args.delta:+.5f} m, {pct:.1f}% of commanded)")
    print(f"[probe_pose] |actual_delta - commanded_delta| = {err:.5f} m "
          f"(tolerance = {args.tolerance:.5f} m)")

    if err <= args.tolerance:
        print("[probe_pose] PASS")
        return 0
    print(f"[probe_pose] FAIL: error {err:.5f} > tolerance {args.tolerance:.5f}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
