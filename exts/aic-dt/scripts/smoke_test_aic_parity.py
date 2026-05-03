"""Phase 1 topic-surface smoke test — runs OUTSIDE Isaac Sim's Python.

Uses /opt/ros/humble Python 3.10's rclpy + tf2_ros (the same Python that
aic_adapter and CheatCode use). If this test passes, Phase 1's published
ROS surface can be consumed by the actual AIC stack with no special path
manipulation.

Verifies:
  TF tests (CheatCode-style):
    1. base_link -> tool0  (URDF arm chain end-effector — sanity check)
    2. base_link -> gripper/tcp  (CheatCode CheatCode.py:105 lookup)
    3. world -> aic_world  (synthesized identity transform)
    4. base_link -> gripper/hande_finger_link_l  (dynamic gripper edge)
    5. world -> tool0  (full chain: world -> tabletop -> base_link -> ... -> tool0)

  /joint_states tests (aic_adapter-style):
    6. Subscribe to /joint_states, verify message arrives within 5s
    7. Names exactly match aic_adapter::joint_sort_order_ keys (line 80-86)
    8. frame_id == 'base_link'
    9. position[gripper/left_finger_joint] == 0.0 (FixedJoint fake)
   10. Arm DOF values look like real angles (in [-2pi, 2pi])
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros.transform_listener import TransformListener

# AIC-canonical names per aic_adapter.cpp:80-86
EXPECTED_JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "gripper/left_finger_joint",
}

# (target_frame, source_frame, expected_meaning)
TF_TESTS = [
    ("base_link", "tool0", "URDF arm end-effector"),
    ("base_link", "gripper/tcp", "CheatCode L105 gripper TCP lookup"),
    ("world", "aic_world", "synthesized identity transform"),
    ("base_link", "gripper/hande_finger_link_l", "dynamic gripper edge"),
    ("world", "tool0", "full chain world -> tabletop -> base_link -> ... -> tool0"),
]


def vec_norm(t):
    return (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5


class SmokeTester(Node):
    def __init__(self):
        super().__init__("aic_parity_smoke_tester")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._js_msgs = []
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        self._js_msgs.append(msg)


def report(label, passed, detail=""):
    icon = "✓" if passed else "✗"
    print(f"  {icon} {label}{(': ' + detail) if detail else ''}")
    return passed


def main():
    rclpy.init()
    node = SmokeTester()

    # Spin for 3s to let TF buffer fill and joint states arrive
    print("Warming up TF buffer + joint_states subscriber for 3s...")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    results = []

    # ----- TF tests -----
    print("\n=== TF lookups (CheatCode-style) ===")
    for target, source, meaning in TF_TESTS:
        try:
            tf = node._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=2.0)
            )
            tr = tf.transform.translation
            rot = tf.transform.rotation
            n = vec_norm(tr)
            results.append(
                report(
                    f"lookup_transform('{target}', '{source}')",
                    True,
                    f"d={n:.3f}m  rot=({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})  [{meaning}]",
                )
            )
        except Exception as exc:
            results.append(
                report(
                    f"lookup_transform('{target}', '{source}')",
                    False,
                    f"FAILED: {exc!r}  [{meaning}]",
                )
            )

    # Geometric sanity: tool0 should be ~0.5-1.5m from base_link in any reasonable arm pose
    try:
        tf = node._tf_buffer.lookup_transform("base_link", "tool0", Time())
        d = vec_norm(tf.transform.translation)
        plausible = 0.3 < d < 1.5
        results.append(
            report(
                "tool0 distance from base_link plausible (0.3-1.5m)",
                plausible,
                f"d={d:.3f}m",
            )
        )
    except Exception:
        pass  # already counted as failure above

    # ----- /joint_states tests -----
    print("\n=== /joint_states (aic_adapter-style) ===")

    if not node._js_msgs:
        # Spin a bit more in case it's slow
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not node._js_msgs:
            rclpy.spin_once(node, timeout_sec=0.05)

    results.append(report("at least one /joint_states message received", bool(node._js_msgs),
                           f"got {len(node._js_msgs)} messages"))

    if node._js_msgs:
        msg = node._js_msgs[-1]
        actual_names = set(msg.name)

        results.append(report("frame_id == 'base_link'", msg.header.frame_id == "base_link",
                               f"got '{msg.header.frame_id}'"))

        results.append(report("name set matches aic_adapter::joint_sort_order_ exactly",
                               actual_names == EXPECTED_JOINT_NAMES,
                               f"missing={EXPECTED_JOINT_NAMES - actual_names}  extra={actual_names - EXPECTED_JOINT_NAMES}"))

        # name list should be alphabetical
        results.append(report("names alphabetical", list(msg.name) == sorted(msg.name),
                               f"got {list(msg.name)}"))

        # gripper joint position must be 0.0 (zero-DOF FixedJoint fake)
        if "gripper/left_finger_joint" in msg.name:
            idx = list(msg.name).index("gripper/left_finger_joint")
            results.append(report("gripper/left_finger_joint position is 0.0",
                                   msg.position[idx] == 0.0,
                                   f"got {msg.position[idx]}"))

        # Arm DOFs in plausible range
        all_in_range = True
        for n, p in zip(msg.name, msg.position):
            if n != "gripper/left_finger_joint" and not (-7.0 < p < 7.0):
                all_in_range = False
        results.append(report("arm DOF positions in [-7, 7] rad", all_in_range))

        # position / velocity / effort all length 7
        results.append(report("position/velocity/effort all length 7",
                               len(msg.position) == 7 and len(msg.velocity) == 7 and len(msg.effort) == 7,
                               f"sizes: pos={len(msg.position)} vel={len(msg.velocity)} eff={len(msg.effort)}"))

    # ----- Verdict -----
    passed = sum(1 for r in results if r)
    total = len(results)
    print(f"\n{passed}/{total} checks passed.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
