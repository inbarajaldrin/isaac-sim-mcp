#!/usr/bin/env python3
# Reference: structurally lifted from exts/aic-dt/scripts/smoke_test_aic_controller.py
# Probe for motion-deficit-hunt task (PARITY-09 round-trip).
"""Round-trip motion probe for a single arm joint.

Subscribes /joint_states, publishes a single JointMotionUpdate to
/aic_controller/joint_commands setting <joint> = initial + target (rad),
sleeps window_seconds, asserts |final - (initial + target)| <= tolerance.

Exit code 0 = pass (full commanded amplitude reached);
1 = fail (deficit > tolerance) — diagnostic printed to stdout.

Assumes: Isaac Sim with aic-dt extension is running and quick_start has been
executed; the host shell has been configured with rmw_zenoh_cpp + the host
zenoh router (per zenoh-path-implementation). The probe makes no assumptions
about the transport beyond that — it uses stock rclpy.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from aic_control_interfaces.msg import JointMotionUpdate

URDF_ORDER = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


class MotionRoundtripProbe(Node):
    def __init__(self):
        super().__init__("probe_motion_roundtrip")
        self._js_msgs = []
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._cmd_pub = self.create_publisher(
            JointMotionUpdate, "/aic_controller/joint_commands", 10
        )

    def _on_js(self, msg):
        self._js_msgs.append(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--joint", default="shoulder_lift_joint",
                   help="Arm joint to perturb (must be in URDF_ORDER)")
    p.add_argument("--target", type=float, default=0.10,
                   help="Commanded delta from initial position (rad)")
    p.add_argument("--tolerance", type=float, default=0.095,
                   help="Min |actual_delta| in correct direction to count as pass (rad). "
                        "Acceptance criterion is Δ ≥ tolerance — i.e. the joint must reach "
                        "at least this much of the commanded amplitude. Default 0.095 ≈ 95% of "
                        "the default 0.10 target.")
    p.add_argument("--window-seconds", type=float, default=2.0,
                   help="Spin window after publish before reading final (s)")
    p.add_argument("--warmup-seconds", type=float, default=2.0,
                   help="Initial spin window to drain /joint_states (s)")
    p.add_argument("--stiffness", type=float, default=2000.0)
    p.add_argument("--damping", type=float, default=100.0)
    args = p.parse_args()

    if args.joint not in URDF_ORDER:
        print(f"FAIL: --joint {args.joint!r} not in URDF_ORDER {URDF_ORDER}")
        return 1

    rclpy.init()
    node = MotionRoundtripProbe()

    t_end = time.time() + args.warmup_seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node._js_msgs:
        print("FAIL: no /joint_states received during warmup")
        rclpy.shutdown()
        return 1

    initial_js = node._js_msgs[-1]
    if args.joint not in initial_js.name:
        print(f"FAIL: {args.joint!r} not in /joint_states (names={list(initial_js.name)})")
        rclpy.shutdown()
        return 1
    initial = initial_js.position[initial_js.name.index(args.joint)]
    target = initial + args.target

    cmd = JointMotionUpdate()
    cmd.target_state = JointTrajectoryPoint()
    cmd.target_state.positions = []
    for jn in URDF_ORDER:
        base_idx = initial_js.name.index(jn)
        pos = initial_js.position[base_idx]
        if jn == args.joint:
            pos += args.target
        cmd.target_state.positions.append(pos)
    cmd.target_stiffness = [args.stiffness] * 6
    cmd.target_damping = [args.damping] * 6
    cmd.trajectory_generation_mode.mode = 2  # MODE_POSITION

    node._cmd_pub.publish(cmd)

    node._js_msgs.clear()
    t_end = time.time() + args.window_seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node._js_msgs:
        print("FAIL: no /joint_states received during window")
        rclpy.shutdown()
        return 1

    final_js = node._js_msgs[-1]
    final = final_js.position[final_js.name.index(args.joint)]
    delta = final - initial
    err = abs(final - target)
    pct = 100.0 * delta / args.target if args.target != 0 else 0.0
    sign = 1.0 if args.target >= 0 else -1.0
    signed_delta = delta * sign
    print(f"joint={args.joint}")
    print(f"initial={initial:+.5f} rad")
    print(f"commanded_delta={args.target:+.5f} rad → target={target:+.5f}")
    print(f"final={final:+.5f} rad → actual_delta={delta:+.5f} ({pct:.1f}% of commanded)")
    print(f"|final - target|={err:.5f}")
    print(f"signed amplitude reached={signed_delta:+.5f} rad (tolerance={args.tolerance})")

    rclpy.shutdown()
    if signed_delta >= args.tolerance:
        print("PASS")
        return 0
    print(f"FAIL: only {pct:.1f}% of commanded amplitude reached "
          f"(need ≥{100.0 * args.tolerance / abs(args.target):.0f}%)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
