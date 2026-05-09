#!/usr/bin/env python3
"""probe_gripper_tcp_ingress.py — gripper-tcp-ingress verify gate.

Plan 02-04 Pitfall 2 Option A: when a /aic_controller/pose_commands MotionUpdate
arrives with header.frame_id='gripper/tcp', controller_loop._apply_pose_cmd
pre-multiplies the target pose by the static SE(3) offset
self._tcp_to_tool0_offset_xform (cached at _setup_kinematics from USD prim
hierarchy) before passing to Lula IK. This probe round-trips that path:

  1. Lookup TF base_link -> gripper/tcp to capture (x0, y0, z0, q0).
  2. Publish MotionUpdate(/aic_controller/pose_commands) with
       header.frame_id='gripper/tcp',
       pose.position.<axis> = initial.<axis> + delta,
       orientation = current,
       trajectory_generation_mode = MODE_POSITION (=2).
  3. Hold publish at 30Hz for --hold-seconds (the controller one-shots
     self._latest_pose_cmd after apply, so a continuous stream keeps the
     buffered cmd fresh + Lula IK + the three-step articulation write are
     exercised every tick).
  4. Re-lookup base_link -> gripper/tcp.
  5. Pass if |actual_delta - commanded_delta| <= --tolerance.

Sister of probe_pose_roundtrip.py (which exercises frame_id='base_link').
The cached offset is what's under test here — pose-roundtrip-verify's PASS
established that Lula IK + the controller self-heal land tool0 within tolerance,
so any residual delta seen here traces to the gripper/tcp -> tool0 transform.

Subscribes /tf + /tf_static under host zenoh transport (RMW=rmw_zenoh_cpp,
peer = tcp/localhost:7447) — Isaac Sim publishes via zenoh per
zenoh-path-implementation; a fastrtps subscriber would silently drop every msg.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys

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
    pose-cmd surface + cached gripper/tcp offset are module-resident regardless."""
    r = _send("quick_start", {})
    if r.get("status") == "success":
        return True
    msg = (r.get("message") or "") + " " \
          + ((r.get("result") or {}).get("error") or "") + " " \
          + (r.get("error") or "")
    if "not unique" in msg or "already" in msg.lower():
        print(f"[probe_tcp] quick_start idempotency tolerated: {msg.strip()}")
        return True
    print(f"[probe_tcp] quick_start failed: {r}")
    return False


def _run_tcp_roundtrip(axis: str, delta: float, hold_seconds: float,
                       warmup_seconds: float) -> dict:
    """Spawn an rclpy subprocess under host zenoh transport that:
      - listens to TF for base_link -> gripper/tcp,
      - publishes a Cartesian gripper/tcp pose target at 30Hz for hold_seconds,
      - returns initial/final translation as JSON.
    """
    code = r"""
import json, time, sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from aic_control_interfaces.msg import MotionUpdate

AXIS = "AXIS_VAL"
DELTA = DELTA_VAL
HOLD = HOLD_VAL
WARMUP = WARMUP_VAL

rclpy.init()
node = rclpy.create_node('probe_gripper_tcp_ingress_' + str(int(time.time())))

tf_buf = Buffer()
tf_listener = TransformListener(tf_buf, node)

pose_pub = node.create_publisher(MotionUpdate, '/aic_controller/pose_commands', 10)

# Warmup: TF tree assemble + zenoh peer discovery settle.
deadline = time.time() + WARMUP
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

def _lookup():
    end = time.time() + 1.5
    last_exc = None
    while time.time() < end:
        try:
            t = tf_buf.lookup_transform('base_link', 'gripper/tcp', Time())
            return t
        except Exception as exc:
            last_exc = exc
            rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError('lookup_transform base_link->gripper/tcp timed out: ' + repr(last_exc))

initial = _lookup()
ix = float(initial.transform.translation.x)
iy = float(initial.transform.translation.y)
iz = float(initial.transform.translation.z)
qx = float(initial.transform.rotation.x)
qy = float(initial.transform.rotation.y)
qz = float(initial.transform.rotation.z)
qw = float(initial.transform.rotation.w)

# Build a Cartesian target in gripper/tcp space: same orientation as current,
# axis += delta.
tx, ty, tz = ix, iy, iz
if AXIS == 'x': tx = ix + DELTA
elif AXIS == 'y': ty = iy + DELTA
elif AXIS == 'z': tz = iz + DELTA
else:
    print('PROBE_RESULT:' + json.dumps({'error': 'bad axis ' + AXIS}))
    sys.exit(1)

cmd = MotionUpdate()
cmd.header.frame_id = 'gripper/tcp'
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.pose.position.x = tx
cmd.pose.position.y = ty
cmd.pose.position.z = tz
cmd.pose.orientation.x = qx
cmd.pose.orientation.y = qy
cmd.pose.orientation.z = qz
cmd.pose.orientation.w = qw
cmd.trajectory_generation_mode.mode = 2  # MODE_POSITION

period = 1.0 / 30.0
end = time.time() + HOLD
while time.time() < end:
    cmd.header.stamp = node.get_clock().now().to_msg()
    pose_pub.publish(cmd)
    spin_until = time.time() + period
    while time.time() < spin_until:
        rclpy.spin_once(node, timeout_sec=0.005)

# Brief settle so the last applied IK target reflects in TF.
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
            .replace("WARMUP_VAL", repr(float(warmup_seconds))))

    env = os.environ.copy()
    for v in ("RMW_IMPLEMENTATION", "ZENOH_ROUTER_CHECK_ATTEMPTS",
              "ZENOH_CONFIG_OVERRIDE", "ROS_LOCALHOST_ONLY"):
        env.pop(v, None)
    env["ROS_DOMAIN_ID"] = os.environ.get("AIC_DT_ROS_DOMAIN_ID", "7")
    cmd_argv = [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source ~/env_isaaclab/bin/activate && "
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
        print(f"[probe_tcp] subscriber rc={proc.returncode}")
        print(f"[probe_tcp] subscriber stdout tail:\n{proc.stdout[-1500:]}")
        print(f"[probe_tcp] subscriber stderr tail:\n{proc.stderr[-2000:]}")
    return payload


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--axis", choices=("x", "y", "z"), default="z")
    ap.add_argument("--delta", type=float, default=-0.03,
                    help="Commanded translation in metres (default -0.03 = 3 cm).")
    ap.add_argument("--tolerance", type=float, default=0.005,
                    help="Maximum |actual_delta - commanded_delta| in metres (default 0.005).")
    ap.add_argument("--hold-seconds", type=float, default=3.0)
    ap.add_argument("--warmup-seconds", type=float, default=2.5)
    args = ap.parse_args()

    if not _ensure_quick_start():
        return 1

    print(f"[probe_tcp] running gripper/tcp round-trip: axis={args.axis} "
          f"delta={args.delta:+.4f} m hold={args.hold_seconds}s tol={args.tolerance}m")
    payload = _run_tcp_roundtrip(args.axis, args.delta, args.hold_seconds,
                                 args.warmup_seconds)
    if not payload or "initial" not in payload or "final" not in payload:
        print(f"[probe_tcp] FAIL: no PROBE_RESULT — payload={payload}")
        return 1

    initial = payload["initial"]
    final = payload["final"]
    commanded = payload["commanded"]
    actual_delta = final[args.axis] - initial[args.axis]
    err = abs(actual_delta - args.delta)
    pct = 100.0 * actual_delta / args.delta if args.delta != 0 else 0.0

    print(f"[probe_tcp] initial base_link->gripper/tcp: "
          f"x={initial['x']:+.5f} y={initial['y']:+.5f} z={initial['z']:+.5f}")
    print(f"[probe_tcp] commanded target: "
          f"x={commanded['x']:+.5f} y={commanded['y']:+.5f} z={commanded['z']:+.5f}")
    print(f"[probe_tcp] final   base_link->gripper/tcp: "
          f"x={final['x']:+.5f} y={final['y']:+.5f} z={final['z']:+.5f}")
    print(f"[probe_tcp] axis={args.axis}: actual_delta={actual_delta:+.5f} m "
          f"(commanded={args.delta:+.5f} m, {pct:.1f}% of commanded)")
    print(f"[probe_tcp] |actual_delta - commanded_delta| = {err:.5f} m "
          f"(tolerance = {args.tolerance:.5f} m)")

    if err <= args.tolerance:
        print("[probe_tcp] PASS")
        return 0
    print(f"[probe_tcp] FAIL: error {err:.5f} > tolerance {args.tolerance:.5f}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
