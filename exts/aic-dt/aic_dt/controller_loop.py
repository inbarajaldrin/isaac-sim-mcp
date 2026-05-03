# Reference: this is a sibling to parity_publishers.py (Phase 1) — same lifecycle,
# inverted I/O. See .planning/phases/02-controller-loop/02-PATTERNS.md and
# .planning/phases/02-controller-loop/02-RESEARCH.md for the full design rationale.
"""AicControllerLoop — Phase 2 controller-side rclpy class.

Subscribes to /aic_controller/joint_commands (PARITY-09), /aic_controller/pose_commands
(PARITY-10); publishes /aic_controller/controller_state (PARITY-11) and
/aic/gazebo/contacts/off_limit (PARITY-06). Mirror of parity_publishers.py
inverted on the rclpy I/O axis — same physics-tick lifecycle, same path
discipline, same on_shutdown teardown semantics.

Plans 02-03..06 fill in the callback bodies (this skeleton is the lifecycle
plumbing only; subscribers/publishers are created but their callbacks are stubs).
"""

import os
import sys

# rclpy / ROS msgs ship for Python 3.11 in the pre-built workspace; ensure it's
# on sys.path before any rclpy import. Mirrors the pattern in parity_publishers.py
# (Phase 1) — same workspace install layout, same stale-3.10 eviction.
_RCLPY_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages"
_ISAAC_ROS_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local/lib/python3.11/dist-packages"

# Empirically (2026-05-03 Gap-A/B inline fix in parity_publishers.py), Isaac Sim's
# startup leaves /opt/ros/humble/local/lib/python3.10/dist-packages on sys.path —
# typesupport .so files there are cp310 and cannot load under cp311. Evict at
# module-load and re-evict before each import via _force_python311_ros_paths().
_STALE_310_FRAGMENTS = (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/dist-packages",
)
sys.path[:] = [p for p in sys.path if not any(s in p for s in _STALE_310_FRAGMENTS)]

for _new_path in (_ISAAC_ROS_311, _RCLPY_311):
    if os.path.isdir(_new_path) and _new_path not in sys.path:
        sys.path.insert(0, _new_path)


def _force_python311_ros_paths():
    """In-place: ensure 3.11 ROS workspace paths come first on sys.path AND
    remove all stale Python-3.10 ROS paths.

    Mirror of parity_publishers.py:_force_python311_ros_paths. Called at the
    top of AicControllerLoop.start() — see that method for the full rationale
    (Isaac Sim re-injects 3.10 paths during Carbonite init, so module-load-time
    eviction alone is not sufficient).
    """
    target_311 = [
        _ISAAC_ROS_311,
        _RCLPY_311,
    ]
    sys.path[:] = (
        [p for p in target_311 if os.path.isdir(p)]
        + [
            p for p in sys.path
            if "python3.10" not in p
            and p not in target_311
        ]
    )


def _ensure_rclpy_clean_import():
    """Evict half-loaded ROS modules from sys.modules before (re)import.

    Mirror of parity_publishers.py:_ensure_rclpy_clean_import. Extends
    _ROS_PREFIXES with two additional packages used in Phase 2:

      - aic_control_interfaces (D-05 fix consumer — built in Plan 02-01)
      - ros_gz_interfaces      (PARITY-06 Contacts msg consumer — Plan 02-01 vendored)

    See parity_publishers.py for the full rationale (Isaac Sim's startup ROS2
    bridge attempt half-loads modules; subsequent imports use the failed
    __path__ unless we evict and reload).
    """
    _ROS_PREFIXES = (
        "rclpy", "rcl_interfaces", "sensor_msgs", "tf2_msgs", "geometry_msgs",
        "std_msgs", "builtin_interfaces", "action_msgs", "lifecycle_msgs",
        "rosidl_runtime_py", "rosidl_generator_py", "rosidl_parser",
        "rmw", "rmw_dds_common", "rosgraph_msgs", "diagnostic_msgs",
        "trajectory_msgs", "visualization_msgs", "shape_msgs", "nav_msgs",
        "controller_manager_msgs", "control_msgs", "ros2pkg", "ament_index_python",
        "aic_control_interfaces",  # NEW — Phase 2 D-05 fix consumer (Plan 02-01)
        "ros_gz_interfaces",       # NEW — Phase 2 PARITY-06 Contacts msg consumer
    )
    for mod_name in list(sys.modules):
        for prefix in _ROS_PREFIXES:
            if mod_name == prefix or mod_name.startswith(prefix + "."):
                del sys.modules[mod_name]
                break


# ----------------------------------------------------------------------
# Joint mapping constants (D-09)
# ----------------------------------------------------------------------
# UR5e arm DOFs — exact joint names from aic_adapter::joint_sort_order_
# (see ~/Documents/aic/aic_adapter/src/aic_adapter.cpp:80-86).
ARM_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}
# AIC's slashed gripper joint name (FixedJoint zero-DOF in the unified USD).
# Silently no-op'd per D-09 — gripper opens via /gripper_command (String),
# not /aic_controller/joint_commands.
GRIPPER_NOOP = "gripper/left_finger_joint"

# TrajectoryGenerationMode enum (mirrors aic_control_interfaces/TrajectoryGenerationMode.msg).
# NOTE: source-of-truth ordering — VELOCITY=1, POSITION=2 (NOT the inverse).
# Verified against ~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/TrajectoryGenerationMode.msg.
MODE_UNSPECIFIED = 0
MODE_VELOCITY = 1
MODE_POSITION = 2

# TargetMode enum (mirrors aic_control_interfaces/TargetMode.msg) —
# used by ControllerState publish (Plan 02-05 reads self._last_target_mode).
TARGET_MODE_UNSPECIFIED = 0
TARGET_MODE_CARTESIAN = 1
TARGET_MODE_JOINT = 2

# UR5e arm DOF count
N_ARM_JOINTS = 6


class AicControllerLoop:
    """Phase 2 controller-side rclpy class.

    Same physics-tick + omni.physx subscription pattern as AicParityPublishers (Phase 1).
    Owns one rclpy node `aic_dt_controller_loop` with 2 subscribers + 2 publishers.

    Per D-08: rclpy.spin_once is called from the physics-step callback (no separate
    spin thread). Per D-11: bad commands log debug and return; never raise into
    the physics callback.

    Plans 02-03..06 fill in:
      - _on_joint_cmd / _apply_joint_cmd (Plan 02-03 — PARITY-09)
      - _on_pose_cmd / _apply_pose_cmd  (Plan 02-04 — PARITY-10)
      - _publish_controller_state       (Plan 02-05 — PARITY-11)
      - _setup_contact_subscription / _on_contact_event / _publish_offlimit_contacts
                                        (Plan 02-06 — PARITY-06)
    """

    def __init__(
        self,
        robot_xform_path: str = "/World/UR5e/aic_unified_robot",
        off_limit_prims: list = None,
    ):
        self._robot_xform_path = robot_xform_path
        self._off_limit_prims = set(off_limit_prims or [])
        # rclpy
        self._node = None
        # Subscribers (Plans 02-03/02-04 wire callbacks)
        self._joint_cmd_sub = None
        self._pose_cmd_sub = None
        # Publishers (Plans 02-05/02-06 wire publication)
        self._ctrl_state_pub = None
        self._contacts_pub = None
        # Physics
        self._physx_sub = None
        self._articulation = None
        # IK/FK solver (Plan 02-04 initializes; Plan 02-05 reuses for FK)
        self._kinematics = None
        # Static SE(3) offsets cached at _setup_kinematics time (Plan 02-04 / Pitfall 2 Option A).
        # _tool0_to_tcp_offset_xform: 4x4 numpy SE(3); egress (FK tool0 pose -> tcp pose, Plan 02-05 reuses).
        # _tcp_to_tool0_offset_xform: 4x4 numpy SE(3); ingress (gripper/tcp pose_cmd -> tool0 IK target).
        self._tool0_to_tcp_offset_xform = None
        self._tcp_to_tool0_offset_xform = None
        # omni.physx contact-report subscription handle (Plan 02-06 initializes)
        self._contact_sub = None
        # Latest command state (modified by sub callbacks; read in physics tick)
        self._latest_joint_cmd = None
        self._latest_pose_cmd = None
        # ControllerState bookkeeping (Plan 02-05 reads/writes)
        self._tcp_pose_buffer = []          # (t, pose) for numerical-diff velocity
        self._last_reference_tcp_pose = None
        self._last_reference_joint_state = None
        self._last_target_mode = 0          # MODE_UNSPECIFIED
        # Log-once flags (mirror parity_publishers.py:_logged_publish_error)
        self._logged_apply_error = False
        self._logged_publish_error = False
        self._logged_contact_error = False

    def set_off_limit_prims(self, prim_paths: list):
        """Allow per-call atom override of the off-limit prim filter (Plan 02-06)."""
        self._off_limit_prims = set(prim_paths or [])

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> bool:
        """Start the controller loop (idempotent — calls stop() first)."""
        self.stop()
        _force_python311_ros_paths()
        _ensure_rclpy_clean_import()

        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
            # Custom message imports — these REQUIRE Plan 02-01's workspace rebuild
            from aic_control_interfaces.msg import (
                JointMotionUpdate, MotionUpdate, ControllerState,
            )
            from ros_gz_interfaces.msg import Contacts
        except ImportError as exc:
            print(f"[AIC-DT][controller] rclpy/aic_control_interfaces import failed: {exc!r}")
            print(f"[AIC-DT][controller] See exts/aic-dt/docs/aic-msgs-setup.md for the D-05 workspace-rebuild fix.")
            return False

        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as exc:
                print(f"[AIC-DT][controller] rclpy.init() failed: {exc!r}")
                return False

        try:
            self._node = rclpy.create_node("aic_dt_controller_loop")
        except Exception as exc:
            print(f"[AIC-DT][controller] create_node failed: {exc!r}")
            return False

        # QoS per RESEARCH.md "Don't Hand-Roll" — match aic_controller.cpp:182
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers (callbacks are stubs in this skeleton; Plans 02-03/02-04 wire them)
        self._joint_cmd_sub = self._node.create_subscription(
            JointMotionUpdate, "/aic_controller/joint_commands",
            self._on_joint_cmd, cmd_qos,
        )
        self._pose_cmd_sub = self._node.create_subscription(
            MotionUpdate, "/aic_controller/pose_commands",
            self._on_pose_cmd, cmd_qos,
        )

        # Publishers (callbacks publish from physics-tick; Plans 02-05/02-06 wire them)
        self._ctrl_state_pub = self._node.create_publisher(
            ControllerState, "/aic_controller/controller_state", cmd_qos,
        )
        self._contacts_pub = self._node.create_publisher(
            Contacts, "/aic/gazebo/contacts/off_limit", cmd_qos,
        )

        # Articulation handle (Pitfall 9: append /root_joint to the Xform path)
        try:
            from isaacsim.core.prims import Articulation
            art_root = f"{self._robot_xform_path}/root_joint"
            self._articulation = Articulation(prim_paths_expr=art_root)
            self._articulation.initialize()
        except Exception as exc:
            print(f"[AIC-DT][controller] Articulation init failed: {exc!r}")
            self._articulation = None

        # IK/FK solver init — Plan 02-04 implements _setup_kinematics()
        try:
            self._setup_kinematics()
        except Exception as exc:
            # Stub-friendly: not fatal for skeleton; Plan 02-04 makes this work
            print(f"[AIC-DT][controller] _setup_kinematics not yet implemented or failed: {exc!r}")

        # Contact subscription — Plan 02-06 implements _setup_contact_subscription()
        try:
            self._setup_contact_subscription()
        except Exception as exc:
            print(f"[AIC-DT][controller] _setup_contact_subscription not yet implemented or failed: {exc!r}")

        # Physics-step subscription (D-08).
        # Note: from-import to avoid `import omni.physx` creating a function-
        # local `omni` binding that would shadow other module-level omni.* uses.
        try:
            from omni.physx import get_physx_interface
            self._physx_sub = get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        except Exception as exc:
            print(f"[AIC-DT][controller] subscribe_physics_step_events failed: {exc!r}")
            return False

        print("[AIC-DT][controller] AicControllerLoop started "
              "(subs: /aic_controller/joint_commands + /aic_controller/pose_commands; "
              "pubs: /aic_controller/controller_state + /aic/gazebo/contacts/off_limit)")
        return True

    def stop(self):
        """Idempotent teardown: release physx sub, contact sub, destroy rclpy node."""
        if self._physx_sub is not None:
            try:
                self._physx_sub = None  # release handle (omni.physx auto-unsubscribes)
            except Exception:
                pass
        if self._contact_sub is not None:
            try:
                self._contact_sub = None  # Plan 02-06's omni.physx contact sub release
            except Exception:
                pass
        if self._node is not None:
            try:
                if self._joint_cmd_sub is not None:
                    self._node.destroy_subscription(self._joint_cmd_sub)
                if self._pose_cmd_sub is not None:
                    self._node.destroy_subscription(self._pose_cmd_sub)
                if self._ctrl_state_pub is not None:
                    self._node.destroy_publisher(self._ctrl_state_pub)
                if self._contacts_pub is not None:
                    self._node.destroy_publisher(self._contacts_pub)
                self._node.destroy_node()
            except Exception as exc:
                print(f"[AIC-DT][controller] stop() teardown error: {exc!r}")
        # Null all field handles
        self._node = None
        self._joint_cmd_sub = None
        self._pose_cmd_sub = None
        self._ctrl_state_pub = None
        self._contacts_pub = None
        self._articulation = None
        self._kinematics = None
        self._tool0_to_tcp_offset_xform = None
        self._tcp_to_tool0_offset_xform = None
        # Buffers
        self._latest_joint_cmd = None
        self._latest_pose_cmd = None
        self._tcp_pose_buffer = []
        self._last_reference_tcp_pose = None
        self._last_reference_joint_state = None
        self._last_target_mode = 0
        # Log-once flags
        self._logged_apply_error = False
        self._logged_publish_error = False
        self._logged_contact_error = False

    # ------------------------------------------------------------------ #
    # Physics-step callback (D-08): drain rclpy → apply cmds → publish state + contacts
    # ------------------------------------------------------------------ #

    def _on_physics_step(self, dt: float):
        if self._node is None:
            return
        try:
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0)
        except Exception as exc:
            if not self._logged_apply_error:
                print(f"[AIC-DT][controller] spin_once error: {exc!r}")
                self._logged_apply_error = True
            return

        # Apply latest commands (Plans 02-03 + 02-04 implement these)
        if self._latest_joint_cmd is not None:
            try:
                self._apply_joint_cmd(self._latest_joint_cmd)
            except Exception as exc:
                if not self._logged_apply_error:
                    print(f"[AIC-DT][controller] _apply_joint_cmd error: {exc!r}")
                    self._logged_apply_error = True
            finally:
                self._latest_joint_cmd = None

        if self._latest_pose_cmd is not None:
            try:
                self._apply_pose_cmd(self._latest_pose_cmd)
            except Exception as exc:
                if not self._logged_apply_error:
                    print(f"[AIC-DT][controller] _apply_pose_cmd error: {exc!r}")
                    self._logged_apply_error = True
            finally:
                self._latest_pose_cmd = None

        # Publish state + contacts (Plans 02-05 + 02-06 implement these)
        try:
            self._publish_controller_state()
        except Exception as exc:
            if not self._logged_publish_error:
                print(f"[AIC-DT][controller] _publish_controller_state error: {exc!r}")
                self._logged_publish_error = True

        try:
            self._publish_offlimit_contacts()
        except Exception as exc:
            if not self._logged_contact_error:
                print(f"[AIC-DT][controller] _publish_offlimit_contacts error: {exc!r}")
                self._logged_contact_error = True

    # ------------------------------------------------------------------ #
    # Subscriber callbacks — STUBS (Plans 02-03 / 02-04 implement bodies).
    # Callbacks MUST NOT raise — buffer to self._latest_*_cmd only.
    # ------------------------------------------------------------------ #

    def _on_joint_cmd(self, msg):
        """Validate incoming JointMotionUpdate per aic_controller.cpp:236-330 and buffer for next physics tick.

        Per D-11: bad commands log debug + return; never raise into the physics callback.
        Per D-08: callbacks MUST NOT apply commands directly — only buffer to self._latest_joint_cmd.

        Validation mirrors the C++ controller's input checks: MODE_UNSPECIFIED is
        silently dropped; MODE_POSITION requires positions vector of size n=6;
        MODE_VELOCITY requires velocities vector of size n=6; if target_stiffness or
        target_damping is non-empty, it MUST be size n=6 (D-06 per-joint contract).
        """
        try:
            n = N_ARM_JOINTS
            mode = msg.trajectory_generation_mode.mode
            if mode == MODE_UNSPECIFIED:
                self._node.get_logger().debug(
                    "Dropped joint_cmd: trajectory_generation_mode UNSPECIFIED")
                return
            if mode == MODE_POSITION:
                if len(msg.target_state.positions) != n:
                    self._node.get_logger().debug(
                        f"Dropped joint_cmd: positions.size()={len(msg.target_state.positions)} != {n}")
                    return
            elif mode == MODE_VELOCITY:
                if len(msg.target_state.velocities) != n:
                    self._node.get_logger().debug(
                        f"Dropped joint_cmd: velocities.size()={len(msg.target_state.velocities)} != {n}")
                    return
            # If stiffness or damping vectors are present, they MUST match n (D-06).
            if msg.target_stiffness and len(msg.target_stiffness) != n:
                self._node.get_logger().debug(
                    f"Dropped joint_cmd: target_stiffness.size()={len(msg.target_stiffness)} != {n}")
                return
            if msg.target_damping and len(msg.target_damping) != n:
                self._node.get_logger().debug(
                    f"Dropped joint_cmd: target_damping.size()={len(msg.target_damping)} != {n}")
                return
            # Validation passed — buffer for the physics tick.
            self._latest_joint_cmd = msg
        except Exception as exc:
            # Per D-11: never raise from the callback; log once.
            if not self._logged_apply_error:
                self._node.get_logger().debug(f"_on_joint_cmd validation error: {exc!r}")
                self._logged_apply_error = True

    def _on_pose_cmd(self, msg):
        """STUB — Plan 02-04 implements validation + buffering per D-02/D-11."""
        self._latest_pose_cmd = msg

    # ------------------------------------------------------------------ #
    # Apply / publish — STUBS (Plans 02-03..06 implement bodies).
    # ------------------------------------------------------------------ #

    def _apply_joint_cmd(self, msg):
        """Apply a validated JointMotionUpdate to the UR5e articulation.

        Per D-09: name-keyed parser. gripper/left_finger_joint silently no-op'd
                  (FixedJoint zero-DOF). Unknown joint names log a warning and skip
                  (don't fail the whole message).
        Per D-06: target_stiffness + target_damping → Articulation.set_gains(joint_names=...);
                  target_state.positions  → Articulation.apply_action(joint_positions=..., joint_names=...);
                  target_feedforward_torque → joint_efforts inside the same ArticulationActions.
                  Cartesian impedance fields (target_pose, target_twist, etc.) are MotionUpdate-side
                  and are explicitly NOT touched here.
        Per D-11: this method is wrapped by _on_physics_step's try/except. Any internal
                  Isaac Sim API failure (set_gains pre-play, apply_action shape mismatch)
                  is also caught here and log-once'd to avoid log spam.
        """
        if self._articulation is None:
            return  # Pre-play / load_robot not yet run

        import numpy as np
        from isaacsim.core.utils.types import ArticulationActions

        # Build a name-keyed view of the message's per-joint data. We iterate the
        # incoming joint_names list (D-09: name-keyed; do NOT assume positional
        # alignment with Isaac Sim's articulation DOF order — apply_action handles
        # the index resolution internally via joint_names=...).
        arm_names = []
        arm_positions = []
        arm_efforts = []
        arm_kps = []
        arm_kds = []

        msg_joint_names = list(msg.target_state.joint_names)
        n_msg = len(msg_joint_names)

        # has_* flags require the per-joint vector to align with the message's
        # joint_names list (one value per joint). Stiffness/damping at the
        # JointMotionUpdate level are a parallel array indexed by joint_names.
        has_positions = bool(msg.target_state.positions) and len(msg.target_state.positions) == n_msg
        has_efforts = bool(msg.target_feedforward_torque) and len(msg.target_feedforward_torque) == n_msg
        has_stiffness = bool(msg.target_stiffness) and len(msg.target_stiffness) == n_msg
        has_damping = bool(msg.target_damping) and len(msg.target_damping) == n_msg

        for i, name in enumerate(msg_joint_names):
            if name == GRIPPER_NOOP:
                # D-09: FixedJoint zero-DOF — silently no-op
                continue
            if name not in ARM_JOINTS:
                # Unknown joint — warn but don't fail the whole message (D-09).
                self._node.get_logger().warn(
                    f"Unknown joint name in joint_cmd: {name!r} — skipping")
                continue
            arm_names.append(name)
            if has_positions:
                arm_positions.append(float(msg.target_state.positions[i]))
            if has_efforts:
                arm_efforts.append(float(msg.target_feedforward_torque[i]))
            if has_stiffness:
                arm_kps.append(float(msg.target_stiffness[i]))
            if has_damping:
                arm_kds.append(float(msg.target_damping[i]))

        if not arm_names:
            return  # No valid arm joints in this message

        # Apply gains FIRST so the next PD step uses the new stiffness/damping.
        # set_gains is a no-op pre-play (Pitfall 6) — wrapped to swallow the
        # silent-fail signal as a debug log rather than error spam.
        if (arm_kps and arm_kds
                and len(arm_kps) == len(arm_names)
                and len(arm_kds) == len(arm_names)):
            try:
                kps = np.array([arm_kps], dtype=np.float32)
                kds = np.array([arm_kds], dtype=np.float32)
                self._articulation.set_gains(kps=kps, kds=kds, joint_names=arm_names)
            except Exception as exc:
                if not self._logged_apply_error:
                    self._node.get_logger().debug(f"set_gains failed: {exc!r}")
                    self._logged_apply_error = True

        # Apply positions + (optional) feedforward efforts via apply_action.
        # apply_action handles name→DOF-index resolution internally — we only
        # pass the subset of joints we have data for.
        if arm_positions and len(arm_positions) == len(arm_names):
            action_kwargs = {
                "joint_positions": np.array([arm_positions], dtype=np.float32),
                "joint_names": arm_names,
            }
            if arm_efforts and len(arm_efforts) == len(arm_names):
                action_kwargs["joint_efforts"] = np.array([arm_efforts], dtype=np.float32)
            try:
                self._articulation.apply_action(ArticulationActions(**action_kwargs))
            except Exception as exc:
                if not self._logged_apply_error:
                    self._node.get_logger().debug(f"apply_action failed: {exc!r}")
                    self._logged_apply_error = True
                return

        # Bookkeeping for ControllerState publish (Plan 02-05 reads these).
        self._last_reference_joint_state = msg.target_state
        self._last_target_mode = TARGET_MODE_JOINT

    def _apply_pose_cmd(self, msg):
        """STUB — Plan 02-04 implements per D-02/D-06."""
        pass

    def _publish_controller_state(self):
        """STUB — Plan 02-05 implements per D-07."""
        pass

    def _publish_offlimit_contacts(self):
        """STUB — Plan 02-06 implements per D-03/D-10."""
        pass

    # ------------------------------------------------------------------ #
    # Setup helpers — STUBS (Plans 02-04 / 02-06 implement bodies).
    # ------------------------------------------------------------------ #

    def _setup_kinematics(self) -> bool:
        """Initialize Lula IK + FK solver AND cache the static tool0 <-> gripper/tcp SE(3) offsets.

        Per D-02: Use LulaKinematicsSolver (NOT full RmpFlow — aic_controller pre-smooths
                  the trajectory; RMPflow reactive avoidance is redundant).
        Per Pitfall 1: Import from `isaacsim.robot_motion.motion_generation.lula.kinematics`,
                       NOT bare `isaacsim.robot_motion.lula` (CONTEXT D-02 wording is wrong).
        Per Pitfall 2 + Open Question Q3 RESOLVED: end_effector_frame_name = "tool0" for
                       the Lula solver (gripper/tcp is NOT in the bundled UR5e URDF). The
                       static `tool0 -> gripper/tcp` SE(3) offset is captured here from the
                       USD prim hierarchy (Phase 1's parity_publishers.py publishes the same
                       chain at startup, so the offset is well-defined and stable).
                       Caches:
                         self._tool0_to_tcp_offset_xform  : 4x4 numpy SE(3) (egress: FK tool0 -> tcp)
                         self._tcp_to_tool0_offset_xform  : 4x4 numpy SE(3) (ingress: cmd tcp -> tool0)
        Idempotent; safe on hot-reload.
        """
        if self._articulation is None:
            self._node.get_logger().warn("_setup_kinematics: articulation not initialized — deferring")
            return False
        try:
            import os
            import numpy as np
            import isaacsim.robot_motion.motion_generation as motion_generation
            from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
            from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver
        except ImportError as exc:
            self._node.get_logger().error(
                f"_setup_kinematics: lula imports failed — {exc!r}. "
                f"Ensure isaacsim.robot_motion.motion_generation extension is enabled."
            )
            return False

        # Walk up to ext root; the motion_policy_configs lives 3 levels up from motion_generation/
        mg_root = os.path.dirname(motion_generation.__file__)
        ur5e_root = os.path.normpath(os.path.join(
            mg_root, "..", "..", "..", "motion_policy_configs", "universal_robots", "ur5e"
        ))
        robot_description_path = os.path.join(ur5e_root, "rmpflow", "ur5e_robot_description.yaml")
        urdf_path = os.path.join(ur5e_root, "ur5e.urdf")

        if not os.path.exists(robot_description_path):
            self._node.get_logger().error(
                f"_setup_kinematics: Lula UR5e robot_description not found at {robot_description_path} — PARITY-10 will fail."
            )
            return False
        if not os.path.exists(urdf_path):
            self._node.get_logger().error(
                f"_setup_kinematics: Lula UR5e URDF not found at {urdf_path} — PARITY-10 will fail."
            )
            return False

        try:
            lula_solver = LulaKinematicsSolver(robot_description_path, urdf_path)
            self._kinematics = ArticulationKinematicsSolver(
                robot_articulation=self._articulation,
                kinematics_solver=lula_solver,
                end_effector_frame_name="tool0",
            )
        except Exception as exc:
            self._node.get_logger().error(f"_setup_kinematics: Lula construction failed — {exc!r}")
            self._kinematics = None
            return False

        # ---- Pitfall 2 Option A: cache static tool0 <-> gripper/tcp SE(3) offset ----
        # Read the USD prim hierarchy directly. Both prims live under the unified-robot
        # Xform; the USD authoring captures the same static chain that parity_publishers.py
        # publishes on /tf_static at startup (verified Phase 1).
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("no USD stage open")
            tool0_path = f"{self._robot_xform_path}/tool0"
            tcp_path = f"{self._robot_xform_path}/gripper/tcp"
            tool0_prim = stage.GetPrimAtPath(tool0_path)
            tcp_prim = stage.GetPrimAtPath(tcp_path)
            if not tool0_prim or not tool0_prim.IsValid():
                raise RuntimeError(f"tool0 prim not found at {tool0_path}")
            if not tcp_prim or not tcp_prim.IsValid():
                raise RuntimeError(f"gripper/tcp prim not found at {tcp_path}")
            # ComputeLocalToWorldTransform yields a Gf.Matrix4d in stage units (meters)
            xf_cache = UsdGeom.XformCache()
            tool0_world = xf_cache.GetLocalToWorldTransform(tool0_prim)   # Gf.Matrix4d (row-major)
            tcp_world = xf_cache.GetLocalToWorldTransform(tcp_prim)       # Gf.Matrix4d
            # tool0 -> tcp = inverse(tool0_world) * tcp_world
            tool0_to_tcp_gf = tool0_world.GetInverse() * tcp_world
            # Convert Gf.Matrix4d (row-major) -> numpy 4x4 column-major SE(3) we use in apply_pose
            # Pixar's Matrix4d is row-major; SE(3) math below uses standard column-major (row-vec * mat).
            # We store both directions as 4x4 numpy arrays in the row-major Pixar convention.
            def _gf_to_np(m):
                return np.array(
                    [[m[i][j] for j in range(4)] for i in range(4)],
                    dtype=np.float64,
                )
            self._tool0_to_tcp_offset_xform = _gf_to_np(tool0_to_tcp_gf)
            self._tcp_to_tool0_offset_xform = _gf_to_np(tool0_to_tcp_gf.GetInverse())
            self._node.get_logger().info(
                f"_setup_kinematics: Lula IK ready (UR5e bundled config, ee='tool0'); "
                f"tool0 -> gripper/tcp offset cached (translation={tool0_to_tcp_gf.ExtractTranslation()})"
            )
            return True
        except Exception as exc:
            # If the offset capture fails, leave the matrices as None (identity-ish) and log
            # a clear error — the gripper/tcp ingress path will then refuse to dispatch (see
            # _apply_pose_cmd guard) instead of silently producing wrong joint targets.
            self._tool0_to_tcp_offset_xform = None
            self._tcp_to_tool0_offset_xform = None
            self._node.get_logger().error(
                f"_setup_kinematics: tool0 <-> gripper/tcp offset capture failed: {exc!r}. "
                f"gripper/tcp pose_cmds will be refused until offset is recoverable."
            )
            # Lula itself is initialized — IK with frame_id='base_link' still works.
            return True

    def _setup_contact_subscription(self) -> bool:
        """STUB — Plan 02-06 implements omni.physx contact-report subscription per D-03."""
        return True
