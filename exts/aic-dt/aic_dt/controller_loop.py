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

    Phase 2 inter-module fix (2026-05-03): if rclpy is already loaded with
    a working `builtin_interfaces.msg.Time` (i.e., parity_publishers.py
    already established the 3.11 ROS workspace as canonical), do NOT
    evict the stock ROS prefixes — doing so would invalidate the Time
    class object that parity_publishers has cached references to, causing
    the Header.stamp type check to fail with
    `AssertionError("The 'stamp' field must be a sub message of type 'Time'")`.

    Only evict the Phase-2-specific prefixes (aic_control_interfaces +
    ros_gz_interfaces) which parity_publishers does not import. This lets
    the controller_loop's first init pick up the 3.11 build of the custom
    messages while leaving the stock-message classes other modules already
    use intact.

    On a true cold start (rclpy not yet loaded — i.e., controller_loop
    starts BEFORE parity_publishers), evict everything per the original
    Phase 1 pattern.
    """
    _PHASE_2_ONLY_PREFIXES = ("aic_control_interfaces", "ros_gz_interfaces")
    _STOCK_ROS_PREFIXES = (
        "rclpy", "rcl_interfaces", "sensor_msgs", "tf2_msgs", "geometry_msgs",
        "std_msgs", "builtin_interfaces", "action_msgs", "lifecycle_msgs",
        "rosidl_runtime_py", "rosidl_generator_py", "rosidl_parser",
        "rmw", "rmw_dds_common", "rosgraph_msgs", "diagnostic_msgs",
        "trajectory_msgs", "visualization_msgs", "shape_msgs", "nav_msgs",
        "controller_manager_msgs", "control_msgs", "ros2pkg", "ament_index_python",
    )

    # Detect whether parity_publishers (or any prior module) has already
    # loaded the canonical 3.11 builtin_interfaces. If so, don't evict it.
    canonical_already_loaded = (
        "builtin_interfaces.msg" in sys.modules
        and "rclpy" in sys.modules
    )

    prefixes = _PHASE_2_ONLY_PREFIXES if canonical_already_loaded \
        else (_STOCK_ROS_PREFIXES + _PHASE_2_ONLY_PREFIXES)

    for mod_name in list(sys.modules):
        for prefix in prefixes:
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
# URDF kinematic order — JointMotionUpdate.target_state.positions[i] maps to
# this joint at index i. JointMotionUpdate uses positional ordering (not
# name-keyed) — JointTrajectoryPoint has no joint_names field. aic_controller
# applies positions in config-defined order; AIC's controller config matches
# UR5e URDF order.
ARM_JOINTS_ORDERED = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
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

# ---------------------------------------------------------------------- #
# Off-limit contact pipeline constants (PARITY-06 / D-03 / D-10)
# ---------------------------------------------------------------------- #
from collections import deque

# Off-limit prim filter — sourced from exts/aic-dt/docs/offlimit-prim-mapping.md
# (Plan 02-01 output). Per Plan 02-01's settlement of Open Q1, the canonical
# off-limit set is "any contact involving the enclosure, enclosure-walls, or
# task-board models" — derived from the OffLimitContactsPlugin SDF block in
# `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` lines 122-130:
#
#     <off_limit_models>
#       <model>enclosure</model>
#       <model>enclosure walls</model>
#       <model>task_board</model>
#     </off_limit_models>
#
# These map to Isaac Sim USD top-level prims as follows (from offlimit-prim-mapping.md):
#   enclosure        -> /World/Enclosure         (extension.py: _setup_aic_enclosure)
#   enclosure walls  -> /World/Enclosure_Walls   (extension.py: _setup_aic_enclosure)
#   task_board       -> /World/TaskBoard         (add_objects atom)
#
# The list is prefix-based — a contact whose actor_path starts with any of
# these three prefixes is treated as off-limit. Descendant collider paths are
# USD-driven; the contact callback receives (actor0_path, actor1_path) tuples
# from the physx contact event and the filter is "startswith any prefix".
# Each entry below corresponds 1:1 with a Gazebo off-limit model name:
#   "/World/Enclosure"        -> Gazebo "enclosure" model
#   "/World/Enclosure_Walls"  -> Gazebo "enclosure walls" model
#   "/World/TaskBoard"        -> Gazebo "task_board" model
DEFAULT_OFF_LIMIT_PRIMS = [
    "/World/Enclosure",
    "/World/Enclosure_Walls",
    "/World/TaskBoard",
]

# Physics-thread contact event collector (per robot-collision-forensics skill).
# Module-level deque so the omni.physx callback (which runs on the physics thread)
# can append O(1) and the per-tick drain (which runs on the same physics thread
# via subscribe_physics_step_events) can pop without contention. maxlen=2048
# bounds memory in case the publisher falls behind the physics tick.
CONTACT_EVENTS = deque(maxlen=2048)


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
        # Last applied arm positions — used to skip redundant set_joint_positions
        # writes when the cmd hasn't changed (high-rate aic_controller publishing).
        self._last_applied_arm_positions = None
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

        # Articulation handle (Pitfall 9: append /root_joint to the Xform path).
        # Do NOT call .initialize() here — it must run AFTER the first physics
        # tick or _physics_view never gets populated and every set_*/get_*/
        # apply_action raises AttributeError. Defer to _on_physics_step.
        try:
            from isaacsim.core.prims import Articulation
            art_root = f"{self._robot_xform_path}/root_joint"
            self._articulation = Articulation(prim_paths_expr=art_root)
        except Exception as exc:
            print(f"[AIC-DT][controller] Articulation construction failed: {exc!r}")
            self._articulation = None

        # Phase-1 OGN articulation_controller cleanup is deferred to the first
        # physics tick because Phase 1's setup_action_graph (which CREATES that
        # node) runs LATER in the quick_start chain than controller_loop.start().
        # See _on_physics_step for the actual removal.
        self._ogn_articulation_ctrl_removed = False
        # Articulation re-initialization is also deferred — see _on_physics_step.
        # Articulation._physics_view is only populated AFTER the first physics
        # step runs, not at .initialize() time (verified 2026-05-04: every call
        # raises AttributeError("'Articulation' object has no attribute '_physics_view'")
        # if invoked before the first physics tick).
        self._articulation_reinitialized = False

        # IK/FK solver init is deferred to first physics tick (after the
        # articulation handle is rebuilt with a valid _physics_view). See
        # _on_physics_step.

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
        self._last_applied_arm_positions = None
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
        # First-tick cleanup: defang Phase-1's OGN IsaacArticulationController
        # at /Graph/ActionGraph_UR5e/articulation_controller. That node subscribes
        # to /joint_states and feeds the result back as positionCommand into the
        # same articulation — a feedback loop that re-targets PD to the previous
        # tick's position, undoing every set_joint_positions write within ~30 ticks.
        # The pattern was designed for an EXTERNAL controller publishing on
        # /joint_states; with Phase 1's parity_publishers ALSO publishing /joint_states
        # as feedback, the loop closes on itself. We can't fix it earlier because
        # setup_action_graph runs after controller_loop.start() in quick_start.
        # Mirrors the same race observed in ur5e-dt (sibling extension).
        # NOTE: stage.RemovePrim on the live OGN node CRASHES Isaac Sim (the graph
        # evaluator can be mid-traversal). Instead, deactivate the prim — the OGN
        # evaluator skips inactive nodes safely (same pattern as the cable D-04 fix).
        if not self._ogn_articulation_ctrl_removed:
            try:
                import omni.usd
                stage = omni.usd.get_context().get_stage()
                ognode_path = "/Graph/ActionGraph_UR5e/articulation_controller"
                node_prim = stage.GetPrimAtPath(ognode_path)
                if node_prim and node_prim.IsValid():
                    node_prim.SetActive(False)
                    print(f"[AIC-DT][controller] deactivated Phase-1 OGN articulation_controller at {ognode_path} (Bug 4 fix)")
                    self._ogn_articulation_ctrl_removed = True
                # If prim doesn't exist yet, leave the flag False — try again next tick.
            except Exception as exc:
                print(f"[AIC-DT][controller] OGN articulation_controller deactivation failed: {exc!r}")
                self._ogn_articulation_ctrl_removed = True  # don't keep retrying on permanent failure

        # Wait several ticks then RECREATE the Articulation handle (not just
        # re-initialize). Empirically, calling initialize() on the existing
        # handle from physics-tick thread doesn't populate _physics_view.
        # A fresh Articulation instance constructed AFTER the simulation has
        # been running for some ticks works correctly — verified against the
        # execute_python_code direct-test pattern that holds positions for 2s.
        if not self._articulation_reinitialized:
            self._init_wait_ticks = getattr(self, "_init_wait_ticks", 0) + 1
            if self._init_wait_ticks >= 30:
                try:
                    from isaacsim.core.prims import Articulation
                    art_root = f"{self._robot_xform_path}/root_joint"
                    self._articulation = Articulation(prim_paths_expr=art_root)
                    self._articulation.initialize()
                    self._articulation_reinitialized = True
                    has_view = hasattr(self._articulation, "_physics_view") and self._articulation._physics_view is not None
                    print(f"[AIC-DT][controller] Articulation handle recreated at tick {self._init_wait_ticks} (has_physics_view={has_view})")
                    # Now that articulation is live, set up the IK/FK solver too
                    try:
                        ok = self._setup_kinematics()
                        print(f"[AIC-DT][controller] _setup_kinematics returned {ok}; _kinematics={self._kinematics is not None}")
                    except Exception as exc:
                        print(f"[AIC-DT][controller] _setup_kinematics failed: {exc!r}")
                except Exception as exc:
                    if not getattr(self, "_logged_reinit_error", False):
                        print(f"[AIC-DT][controller] Articulation recreate failed: {exc!r}")
                        self._logged_reinit_error = True
        try:
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0)
        except Exception as exc:
            if not self._logged_apply_error:
                print(f"[AIC-DT][controller] spin_once error: {exc!r}")
                self._logged_apply_error = True
            return

        # Apply latest commands ONCE per cmd. Empirically (2026-05-05): a single
        # set_joint_positions + set_joint_velocities(0) + apply_action(joint_
        # positions) call HOLDS the position perfectly for 2+s. Calling the same
        # write every physics tick instead creates repeated discontinuities that
        # PD damping kd=100 resists, dragging position toward an equilibrium
        # ~50% of commanded — verified by smoke test at 50% motion under
        # continuous-apply, vs 100% motion under one-shot direct apply.
        # (The OGN articulation_controller race that originally motivated
        # continuous re-application is now solved separately by SetActive(False)
        # in the first-tick cleanup above, so one-shot apply wins safely.)
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
    # Subscriber callbacks — implemented across Plans 02-03 (joint) / 02-04 (pose).
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
        """Validate incoming MotionUpdate per aic_controller.cpp:218-227 and buffer for next physics tick.

        Per D-11: bad commands log debug + return; never raise into the physics callback.
        Per D-08: callbacks MUST NOT apply commands directly — only buffer to self._latest_pose_cmd.
        Per Pitfall 2 Option A: gripper/tcp frame_id is accepted; the static SE(3) offset
                                cached at _setup_kinematics is applied in _apply_pose_cmd.
        """
        try:
            if msg.header.frame_id not in ("base_link", "gripper/tcp"):
                self._node.get_logger().debug(
                    f"Dropped pose_cmd: header.frame_id={msg.header.frame_id} not in (base_link, gripper/tcp)"
                )
                return
            if msg.trajectory_generation_mode.mode == MODE_UNSPECIFIED:
                self._node.get_logger().debug("Dropped pose_cmd: trajectory_generation_mode UNSPECIFIED")
                return
            # Validation passed — buffer for the physics tick
            self._latest_pose_cmd = msg
        except Exception as exc:
            if not self._logged_apply_error:
                self._node.get_logger().debug(f"_on_pose_cmd validation error: {exc!r}")
                self._logged_apply_error = True

    # ------------------------------------------------------------------ #
    # Apply / publish — implemented across Plans 02-03..06.
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

        # POSITIONAL parser per actual JointMotionUpdate contract:
        # - target_state is a trajectory_msgs/JointTrajectoryPoint
        # - JointTrajectoryPoint has NO joint_names field (only positions/
        #   velocities/accelerations/effort/time_from_start)
        # - aic_controller maps positions[i] to its config-defined joint i;
        #   AIC's controller config matches UR5e URDF kinematic order
        # - Isaac Sim mirrors that mapping via ARM_JOINTS_ORDERED
        # - D-09 gripper note: GRIPPER_NOOP (gripper/left_finger_joint) is a
        #   FixedJoint zero-DOF, NEVER appears in JointMotionUpdate position
        #   arrays (gripper opens via /gripper_command String topic separately).
        #   So no slashed-name handling needed in this parser — the message
        #   just carries 6 arm-joint values.
        n_arm = len(ARM_JOINTS_ORDERED)
        positions = list(msg.target_state.positions or [])
        efforts = list(msg.target_feedforward_torque or [])
        stiffnesses = list(msg.target_stiffness or [])
        dampings = list(msg.target_damping or [])

        # Truncate or pad to n_arm — aic_controller treats config size as the
        # contract; we accept N==n_arm and silently drop anything else (D-11).
        if positions and len(positions) != n_arm:
            if not self._logged_apply_error:
                self._node.get_logger().debug(
                    f"joint_cmd positions size {len(positions)} != {n_arm} — dropping")
                self._logged_apply_error = True
            return

        arm_names = list(ARM_JOINTS_ORDERED)
        arm_positions = [float(p) for p in positions] if positions else []
        arm_efforts = [float(e) for e in efforts[:n_arm]] if len(efforts) == n_arm else []
        arm_kps = [float(k) for k in stiffnesses[:n_arm]] if len(stiffnesses) == n_arm else []
        arm_kds = [float(k) for k in dampings[:n_arm]] if len(dampings) == n_arm else []

        if not arm_positions:
            return  # No positions in this message — nothing to apply

        # Apply gains FIRST so the next PD step uses the new stiffness/damping.
        # set_gains is a no-op pre-play (Pitfall 6) — wrapped to swallow the
        # silent-fail signal as a debug log rather than error spam.
        # Use its own log-once flag (NOT _logged_apply_error) so a set_gains
        # failure doesn't silence subsequent position-write error logging —
        # this trapped the 2026-05-04 Bug-4 debug for ~hours: set_gains was
        # raising the Isaac-Sim joint_names= IndexError, setting the shared
        # flag, then position writes failed silently.
        if (arm_kps and arm_kds
                and len(arm_kps) == len(arm_names)
                and len(arm_kds) == len(arm_names)):
            try:
                kps = np.array([arm_kps], dtype=np.float32)
                kds = np.array([arm_kds], dtype=np.float32)
                # Articulation has 6 DOFs in URDF order matching ARM_JOINTS_ORDERED.
                # Do NOT pass joint_names — Isaac Sim 5.0 has the off-by-one bug
                # for set_gains too, same as set_joint_positions.
                self._articulation.set_gains(kps=kps, kds=kds)
            except Exception as exc:
                if not getattr(self, "_logged_set_gains_error", False):
                    print(f"[AIC-DT][controller] set_gains failed: {exc!r}")
                    self._logged_set_gains_error = True

        # Apply positions via set_joint_positions (direct write, bypasses PD).
        #
        # Why direct-write instead of apply_action(joint_positions=...) PD targets:
        # Phase 1's setup_action_graph creates an OGN ArticulationController node
        # at /Graph/ActionGraph_UR5e/articulation_controller subscribed to
        # /joint_states (Phase 1 echo of current state). At every physics tick
        # the OGN controller calls apply_action with whatever's on /joint_states,
        # effectively setting position targets back to current positions —
        # racing with our apply_action calls and cancelling them. set_joint_positions
        # writes dof_pos directly and the OGN PD then tracks the new state,
        # so the OGN controller can no longer fight us.
        #
        # aic_controller pre-smooths trajectories before publishing JointMotionUpdate,
        # so per-tick direct writes produce visually-smooth motion at the controller's
        # publish rate (no need for sim-side PD tracking).
        if arm_positions and len(arm_positions) == len(arm_names):
            try:
                # Three-step write — set_joint_positions writes dof_pos (instant
                # motion), apply_action(joint_positions=arr) writes the PD target
                # (so PD doesn't pull back to a stale target), set_joint_velocities
                # writes zeros to suppress the artificial velocity transient that
                # the instantaneous dof_pos write creates (without this, PD damping
                # kd=100 fights the artificial velocity and drags position back —
                # symptom: only ~50% of commanded motion achieved). All arrays are
                # shape (1, 6); positional indexing matches the 6-DOF articulation
                # in URDF order. Do NOT pass joint_names — Isaac Sim 5.0 has an
                # internal off-by-one that raises IndexError when joint_names is
                # provided.
                arr = np.array([arm_positions], dtype=np.float32)
                # Only set_joint_positions on cmd CHANGE — at high publish rates
                # (aic_controller >100Hz), one-shot-clear means apply runs every
                # physics tick (60Hz) with the same buffered cmd. Repeated
                # set_joint_positions of the same value still destabilizes
                # dof_pos via the artificial discontinuity each tick, dragging
                # the arm to ~50% of commanded motion. Verified 2026-05-05:
                # both single AND 200Hz publishing give 50% motion without this
                # guard. With the guard: single publishes hit 100% and high-rate
                # publishes refresh PD target each tick, also reaching target.
                cmd_changed = (
                    self._last_applied_arm_positions is None
                    or arm_positions != self._last_applied_arm_positions
                )
                if cmd_changed:
                    self._articulation.set_joint_positions(arr)
                    self._last_applied_arm_positions = list(arm_positions)
                # PD target ALWAYS refreshed (cheap; no destabilization).
                self._articulation.apply_action(ArticulationActions(joint_positions=arr))
            except Exception as exc:
                # Loud on FIRST error so future debuggers don't chase the rclpy
                # spin layer when the actual fault is here (the silent .debug()
                # log-once cost ~3 hrs of investigation in the 2026-05-03 cycle).
                if not self._logged_apply_error:
                    print(f"[AIC-DT][controller] set_joint_positions failed: {exc!r}")
                    self._logged_apply_error = True
                return
            # Feedforward effort still goes through apply_action (set_joint_positions
            # doesn't accept efforts; effort is applied alongside the position target).
            if arm_efforts and len(arm_efforts) == len(arm_names):
                try:
                    # Same Isaac Sim 5.0 quirk — no joint_names=.
                    self._articulation.apply_action(ArticulationActions(
                        joint_efforts=np.array([arm_efforts], dtype=np.float32),
                    ))
                except Exception as exc:
                    if not getattr(self, "_logged_efforts_error", False):
                        print(f"[AIC-DT][controller] apply_action(efforts) failed: {exc!r}")
                        self._logged_efforts_error = True

        # Bookkeeping for ControllerState publish (Plan 02-05 reads these).
        self._last_reference_joint_state = msg.target_state
        self._last_target_mode = TARGET_MODE_JOINT

    def _apply_pose_cmd(self, msg):
        """Apply a validated MotionUpdate by IK-resolving the Cartesian pose target to joint positions.

        Per D-02: LulaKinematicsSolver via ArticulationKinematicsSolver wrapper.
        Per D-06: target_stiffness[36] (6x6 Cartesian impedance), feedforward_wrench_at_tip,
                  wrench_feedback_gains_at_tip[6] are CONTROLLER-SIDE math — Isaac Sim
                  is the hardware sink. Log at debug; do NOT act on them. Double-applying
                  corrupts aic_controller's intent.
        Per Pitfall 2 Option A: end_effector_frame_name="tool0"; if msg.header.frame_id ==
                  "gripper/tcp", pre-multiply the target pose by self._tcp_to_tool0_offset_xform
                  (cached at _setup_kinematics) to convert from gripper/tcp frame to tool0
                  frame BEFORE passing to LulaKinematicsSolver. NO drop+log fallback (Open
                  Question Q3 RESOLVED).
        Per Pitfall 4: ROS quaternion is (x,y,z,w); Lula expects (w,x,y,z) — convert at boundary.
        Per D-11: callbacks never raise — outer try/except in _on_physics_step catches.
        """
        if self._articulation is None or self._kinematics is None:
            return  # Pre-play / IK setup not ready

        import numpy as np

        # Pitfall 2 Option A (Open Question Q3 RESOLVED): if frame_id == "gripper/tcp",
        # pre-multiply the target pose by self._tcp_to_tool0_offset_xform to convert from
        # gripper/tcp frame into tool0 frame before passing to LulaKinematicsSolver (which
        # only knows about tool0 — gripper/tcp is not in the bundled UR5e URDF).
        #
        # SE(3) composition (numpy, 4x4 row-major Pixar convention):
        #   T_world_tcp_target  = msg.pose (homogeneous)
        #   T_world_tool0_target = T_world_tcp_target @ T_tcp_to_tool0
        # where T_tcp_to_tool0 is the static rigid offset cached at _setup_kinematics time.
        #
        # If the offset capture failed at setup time (matrices are None), refuse the command
        # rather than silently producing wrong joint targets.
        if msg.header.frame_id == "gripper/tcp":
            if self._tcp_to_tool0_offset_xform is None:
                self._node.get_logger().debug(
                    "Dropped pose_cmd: gripper/tcp frame received but tool0<->tcp offset "
                    "matrices were not captured at _setup_kinematics time (see error log there)."
                )
                return
            # Build T_world_tcp_target as a 4x4 from msg.pose
            from scipy.spatial.transform import Rotation as _R   # bundled with isaacsim env
            R_tcp = _R.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]).as_matrix()
            T_world_tcp = np.eye(4, dtype=np.float64)
            T_world_tcp[:3, :3] = R_tcp
            T_world_tcp[0, 3] = msg.pose.position.x
            T_world_tcp[1, 3] = msg.pose.position.y
            T_world_tcp[2, 3] = msg.pose.position.z
            # Compose: T_world_tool0 = T_world_tcp @ T_tcp_to_tool0
            T_world_tool0 = T_world_tcp @ self._tcp_to_tool0_offset_xform
            # Decompose back to (position, quaternion)
            target_position_override = T_world_tool0[:3, 3].copy()
            target_orientation_override = _R.from_matrix(T_world_tool0[:3, :3]).as_quat()  # (x,y,z,w)
            # Re-pack as (w,x,y,z) for Lula (Pitfall 4)
            gripper_tcp_overrides = (
                target_position_override,
                np.array([
                    target_orientation_override[3],   # w
                    target_orientation_override[0],   # x
                    target_orientation_override[1],   # y
                    target_orientation_override[2],   # z
                ], dtype=np.float64),
            )
            self._node.get_logger().debug(
                f"Pose cmd transform: gripper/tcp target -> tool0 target "
                f"(translation diff = {target_position_override - np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])})"
            )
        else:
            gripper_tcp_overrides = None

        # Log + ignore Cartesian impedance fields per D-06
        # NOTE: msg.target_stiffness can be a numpy array — `if numpy_arr` raises
        # ValueError("truth value ambiguous"). Use len() > 0 + any() pattern.
        if len(msg.target_stiffness) > 0 and any(float(s) != 0.0 for s in msg.target_stiffness):
            self._node.get_logger().debug(
                f"Ignoring MotionUpdate.target_stiffness (6x6 Cartesian impedance) — D-06: controller-side math"
            )
        # feedforward_wrench_at_tip is a Wrench message (sub-msg, always truthy);
        # check field values directly without truthiness on the wrench itself.
        ffw = msg.feedforward_wrench_at_tip
        if (ffw.force.x != 0.0 or ffw.force.y != 0.0 or ffw.force.z != 0.0):
            self._node.get_logger().debug(
                "Ignoring MotionUpdate.feedforward_wrench_at_tip — D-06: controller-side math"
            )

        # Build IK target (Pitfall 4: convert ROS quat → Lula quat at boundary).
        # If the gripper/tcp -> tool0 transform produced overrides above, use those.
        if gripper_tcp_overrides is not None:
            target_position, target_orientation = gripper_tcp_overrides
        else:
            target_position = np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                dtype=np.float64,
            )
            # Lula expects (w, x, y, z); ROS Quaternion is (x, y, z, w)
            target_orientation = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ], dtype=np.float64)

        try:
            ik_action, success = self._kinematics.compute_inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
            )
        except Exception as exc:
            if not self._logged_apply_error:
                self._node.get_logger().debug(f"compute_inverse_kinematics raised: {exc!r}")
                self._logged_apply_error = True
            return

        if not success:
            self._node.get_logger().debug(
                f"IK did not converge for pose target ({target_position.tolist()}); dropping per D-11"
            )
            return

        # Apply IK output via set_joint_positions (direct write, bypasses PD).
        # Same OGN-controller-competition rationale as _apply_joint_cmd: Phase 1's
        # /Graph/ActionGraph_UR5e/articulation_controller races with apply_action;
        # set_joint_positions wins the race by writing dof_pos directly.
        # ik_action.joint_positions is a numpy array of shape (1, n_dof) or list.
        try:
            ik_positions = ik_action.joint_positions
            if ik_positions is not None:
                if not isinstance(ik_positions, np.ndarray):
                    ik_positions = np.array(ik_positions, dtype=np.float32)
                if ik_positions.ndim == 1:
                    ik_positions = ik_positions.reshape(1, -1)
                # ArticulationKinematicsSolver returns positions for the IK chain
                # joints (URDF arm order). Articulation has exactly 6 DOFs in the same
                # order, so positional indexing matches naturally.
                # Do NOT pass joint_names — Isaac Sim 5.0 set_joint_positions has
                # an internal off-by-one that raises IndexError when joint_names is
                # provided. See _apply_joint_cmd for the full story.
                self._articulation.set_joint_positions(
                    ik_positions.astype(np.float32)
                )
        except Exception as exc:
            if not self._logged_apply_error:
                print(f"[AIC-DT][controller] set_joint_positions(ik) failed: {exc!r}")
                self._logged_apply_error = True
            return

        # Bookkeeping for ControllerState publish (Plan 02-05 reads these)
        self._last_reference_tcp_pose = msg.pose
        self._last_target_mode = TARGET_MODE_CARTESIAN

        # PyKDL fallback path (D-02): documented but NOT implemented. Research recommendation:
        # defer until Lula proves insufficient (no failure observed yet). If Lula returns
        # success=False repeatedly for known-reachable targets, this is where a PyKDL
        # ChainIkSolverPos_LMA / TRAC-IK fallback would slot in.

    def _publish_controller_state(self):
        """Publish /aic_controller/controller_state at physics-tick rate per PARITY-11 + D-07.

        D-07 field policy:
          - MEASURED fields populated: tcp_pose (FK), tcp_velocity (numerical-diff)
          - REFERENCE fields echo last command: reference_tcp_pose (Plan 02-04),
            reference_joint_state (Plan 02-03), target_mode
          - TARE field zero per D-07: fts_tare_offset is zero WrenchStamped with
            frame_id "ati/tool_link" (Isaac Sim doesn't tare; aic_controller does
            from /fts_broadcaster/wrench history)
          - tcp_error: 6-vector (x,y,z) populated; (rx,ry,rz) zero first cut
            (axis-angle delta is finicky; can be filled later if a CheatCode
            trial reveals it's needed)
          - tcp_velocity.angular left at zero first cut (quaternion-diff is finicky)

        Frame conventions:
          - msg.header.frame_id = "base_link" (matches Phase 1 publisher convention)
          - tcp_pose: FK on the same self._kinematics Plan 02-04 set up
            (ArticulationKinematicsSolver, end_effector_frame_name="tool0").
            NOTE: this returns the tool0 pose; the static tool0->tcp offset is
            available as self._tool0_to_tcp_offset_xform if a future revision
            wants tcp-frame egress. First-cut publishes the tool0 FK directly,
            consistent with the rest of the AIC topic surface (which expresses
            EE pose at tool0; the Cartesian impedance loop in aic_controller
            handles the tool0->tcp tip transform on its end).

        Per D-11: any internal failure is caught and log-once'd; never raises
        into the physics-step callback.
        """
        if self._node is None or self._ctrl_state_pub is None:
            return

        try:
            from aic_control_interfaces.msg import ControllerState
        except ImportError:
            return  # Plan 02-01's workspace rebuild is the prereq; warn-once already in start()

        msg = ControllerState()

        # Header per Phase 1 publisher convention
        now = self._node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"

        # tcp_pose via FK (D-07: measured field; uses Plan 02-04's self._kinematics)
        if self._kinematics is not None:
            try:
                ee_pos, ee_rot = self._kinematics.compute_end_effector_pose()
                msg.tcp_pose.position.x = float(ee_pos[0])
                msg.tcp_pose.position.y = float(ee_pos[1])
                msg.tcp_pose.position.z = float(ee_pos[2])
                # Convert rotation matrix to quaternion (returns wxyz)
                from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
                import numpy as np
                q_wxyz = rot_matrices_to_quats(np.asarray(ee_rot)[None, :, :])[0]
                # ROS Quaternion is (x, y, z, w); Pitfall 4 — reorder at boundary
                msg.tcp_pose.orientation.w = float(q_wxyz[0])
                msg.tcp_pose.orientation.x = float(q_wxyz[1])
                msg.tcp_pose.orientation.y = float(q_wxyz[2])
                msg.tcp_pose.orientation.z = float(q_wxyz[3])
            except Exception as exc:
                if not self._logged_publish_error:
                    print(f"[AIC-DT][controller] FK compute_end_effector_pose failed: {exc!r}")
                    self._logged_publish_error = True
        else:
            if not getattr(self, "_logged_kinematics_none", False):
                print(f"[AIC-DT][controller] _publish_controller_state: self._kinematics is None — FK won't run")
                self._logged_kinematics_none = True

        # tcp_velocity via numerical-diff (D-07; 3-sample ring buffer per CONTEXT discretion).
        # First 1-2 samples will produce zero velocity (warmup); after that we have at least
        # two samples in the buffer and can compute a finite difference.
        t_sec = self._node.get_clock().now().nanoseconds * 1e-9
        # Snapshot the just-computed tcp_pose position (cheap tuple copy)
        self._tcp_pose_buffer.append((t_sec, (
            msg.tcp_pose.position.x, msg.tcp_pose.position.y, msg.tcp_pose.position.z,
        )))
        if len(self._tcp_pose_buffer) > 3:
            self._tcp_pose_buffer.pop(0)
        if len(self._tcp_pose_buffer) >= 2:
            (t0, p0), (t1, p1) = self._tcp_pose_buffer[-2], self._tcp_pose_buffer[-1]
            dt = max(t1 - t0, 1e-6)
            msg.tcp_velocity.linear.x = (p1[0] - p0[0]) / dt
            msg.tcp_velocity.linear.y = (p1[1] - p0[1]) / dt
            msg.tcp_velocity.linear.z = (p1[2] - p0[2]) / dt
            # Angular velocity left at zero first cut — quaternion-diff is finicky
            # and aic_controller doesn't consume tcp_velocity.angular for any
            # current trial. Defer to a later phase if a CheatCode trial reveals it.

        # reference_tcp_pose echoes last MotionUpdate.pose (Plan 02-04 set this)
        if self._last_reference_tcp_pose is not None:
            msg.reference_tcp_pose = self._last_reference_tcp_pose

        # tcp_error: 6-vector (x,y,z populated; rx,ry,rz zero first cut)
        if self._last_reference_tcp_pose is not None:
            try:
                err = [
                    float(msg.tcp_pose.position.x - self._last_reference_tcp_pose.position.x),
                    float(msg.tcp_pose.position.y - self._last_reference_tcp_pose.position.y),
                    float(msg.tcp_pose.position.z - self._last_reference_tcp_pose.position.z),
                    0.0,  # rx — defer (axis-angle delta finicky; not used by current trial)
                    0.0,  # ry
                    0.0,  # rz
                ]
                # Some rosidl bindings expose tcp_error as array.array, others as list — handle both.
                try:
                    for i, v in enumerate(err):
                        msg.tcp_error[i] = v
                except (TypeError, AttributeError):
                    msg.tcp_error = err
            except Exception:
                pass  # Leave at zero-init

        # reference_joint_state echoes last JointMotionUpdate.target_state (Plan 02-03 set this)
        if self._last_reference_joint_state is not None:
            msg.reference_joint_state = self._last_reference_joint_state

        # target_mode (Plan 02-03 sets TARGET_MODE_JOINT; Plan 02-04 sets TARGET_MODE_CARTESIAN)
        msg.target_mode.mode = self._last_target_mode

        # fts_tare_offset: zero WrenchStamped with frame_id ati/tool_link per D-07.
        # Isaac Sim doesn't tare the F/T sensor; aic_controller computes the tare
        # offset from /fts_broadcaster/wrench history (matches aic_controller.cpp:1275).
        msg.fts_tare_offset.header.frame_id = "ati/tool_link"
        msg.fts_tare_offset.header.stamp = msg.header.stamp
        # All Wrench fields default to 0.0 in the rosidl-generated dataclass

        try:
            self._ctrl_state_pub.publish(msg)
        except Exception as exc:
            if not self._logged_publish_error:
                self._node.get_logger().debug(f"_ctrl_state_pub.publish failed: {exc!r}")
                self._logged_publish_error = True

    def _publish_offlimit_contacts(self):
        """Drain CONTACT_EVENTS deque and publish a ros_gz_interfaces/Contacts message.

        Per robot-collision-forensics skill: only report CONTACT_FOUND events; suppress
        CONTACT_PERSIST and CONTACT_LOST as noise (they generate enormous traffic on
        sustained contacts and downstream consumers don't act on them).

        Per D-11: never raise into physics callback (outer try/except in _on_physics_step
        catches; this method also wraps publish in try/except).
        """
        if self._node is None or self._contacts_pub is None:
            return
        if not CONTACT_EVENTS:
            return

        # Drain everything available this tick
        events = []
        n = len(CONTACT_EVENTS)
        for _ in range(n):
            try:
                events.append(CONTACT_EVENTS.popleft())
            except IndexError:
                break

        try:
            from ros_gz_interfaces.msg import Contacts, Contact, Entity
        except ImportError:
            return  # Plan 02-01 workspace rebuild is the prereq

        msg = Contacts()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # frame_id = "world" — confirmed against live aic_eval snapshot (Plan 02-01).
        msg.header.frame_id = "world"

        for e in events:
            if e.get("type") != "CONTACT_FOUND":
                continue   # skill: PERSIST/LOST suppressed
            c = Contact()
            c.collision1 = Entity(name=e["a0"], type=Entity.LINK)
            c.collision2 = Entity(name=e["a1"], type=Entity.LINK)
            c.positions = []
            c.normals = []
            c.depths = []
            c.wrenches = []
            msg.contacts.append(c)

        if not msg.contacts:
            return

        try:
            self._contacts_pub.publish(msg)
        except Exception as exc:
            if not self._logged_contact_error:
                self._node.get_logger().debug(f"_contacts_pub.publish failed: {exc!r}")
                self._logged_contact_error = True

    # ------------------------------------------------------------------ #
    # Setup helpers — implemented across Plans 02-04 (kinematics) / 02-06 (contacts).
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
            # ArticulationKinematicsSolver requires a SingleArticulation
            # (legacy class with handles_initialized attribute), NOT the new
            # isaacsim.core.prims.Articulation we use elsewhere. Without this,
            # compute_end_effector_pose raises AttributeError("'Articulation'
            # object has no attribute 'handles_initialized'").
            from isaacsim.core.prims import SingleArticulation
            art_root = f"{self._robot_xform_path}/root_joint"
            kinematics_articulation = SingleArticulation(prim_path=art_root)
            kinematics_articulation.initialize()
            lula_solver = LulaKinematicsSolver(robot_description_path, urdf_path)
            self._kinematics = ArticulationKinematicsSolver(
                robot_articulation=kinematics_articulation,
                kinematics_solver=lula_solver,
                end_effector_frame_name="tool0",
            )
            print(f"[AIC-DT][controller] _setup_kinematics: ArticulationKinematicsSolver constructed with SingleArticulation")
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
        """Subscribe to omni.physx contact events on the configured off-limit prims.

        Per D-03 + robot-collision-forensics skill:
          - Apply PhysxSchema.PhysxContactReportAPI(threshold=0.0) to each off-limit prim.
          - Skip + warn if the prim lacks UsdPhysics.RigidBodyAPI (Pitfall 7).
          - Subscribe via omni.physx.get_physx_simulation_interface() (NOT ContactSensor —
            broken in 5.0).
          - Per Pitfall 8: explicit threshold=0.0 (default suppresses gentle contacts).
        """
        try:
            import omni.physx
            import omni.usd
            from pxr import UsdPhysics, PhysxSchema
        except ImportError as exc:
            self._node.get_logger().warn(
                f"_setup_contact_subscription: omni.physx unavailable: {exc!r}"
            )
            return False

        # If atom didn't override, fall back to the module-level default
        prim_paths = self._off_limit_prims if self._off_limit_prims else set(DEFAULT_OFF_LIMIT_PRIMS)
        if not prim_paths:
            self._node.get_logger().warn(
                "_setup_contact_subscription: no off-limit prims configured "
                "(DEFAULT_OFF_LIMIT_PRIMS empty and no per-call override) — PARITY-06 "
                "will silently no-op. See exts/aic-dt/docs/offlimit-prim-mapping.md."
            )
            return False
        self._off_limit_prims = set(prim_paths)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._node.get_logger().warn("_setup_contact_subscription: no USD stage open")
            return False

        watched = set()
        skipped_no_rb = []
        skipped_missing = []
        for path in self._off_limit_prims:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                skipped_missing.append(path)
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                # Pitfall 7: contacts won't fire on this prim
                skipped_no_rb.append(path)
                continue
            # Apply or update PhysxContactReportAPI with threshold=0.0 (Pitfall 8)
            if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
                cr = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                cr.CreateThresholdAttr().Set(0.0)
            else:
                cr = PhysxSchema.PhysxContactReportAPI(prim)
                thr = cr.GetThresholdAttr()
                if thr is None or thr.Get() is None or thr.Get() > 0.0:
                    cr.CreateThresholdAttr().Set(0.0)
            watched.add(path)

        if skipped_missing:
            self._node.get_logger().warn(
                f"_setup_contact_subscription: {len(skipped_missing)} off-limit prims "
                f"missing in stage: {skipped_missing}"
            )
        if skipped_no_rb:
            self._node.get_logger().warn(
                f"_setup_contact_subscription: {len(skipped_no_rb)} off-limit prims lack "
                f"RigidBodyAPI (Pitfall 7 — contacts won't fire). Skipping: {skipped_no_rb}"
            )

        if not watched:
            self._node.get_logger().warn(
                "_setup_contact_subscription: no off-limit prims survived API checks; "
                "no contact subscription installed"
            )
            return False

        try:
            sim_iface = omni.physx.get_physx_simulation_interface()
            self._contact_sub = sim_iface.subscribe_contact_report_events(self._on_contact_event)
            self._node.get_logger().info(
                f"_setup_contact_subscription: watching {len(watched)} off-limit prims "
                f"for contact events"
            )
            return True
        except Exception as exc:
            self._node.get_logger().error(
                f"_setup_contact_subscription: subscribe_contact_report_events failed: {exc!r}"
            )
            return False

    def _on_contact_event(self, headers, data):
        """Physics-thread callback per robot-collision-forensics skill.

        Runs on the physics thread — must be O(fast). Swallow per-event errors so
        one malformed header doesn't kill the subscription (D-11 + skill guidance).

        Filters events: at least one side of the contact pair must be in
        self._off_limit_prims. Appends survivors to module-level CONTACT_EVENTS deque
        for the per-tick drain to consume.
        """
        try:
            from pxr import PhysicsSchemaTools
        except ImportError:
            return

        for h in headers:
            try:
                a0 = str(PhysicsSchemaTools.intToSdfPath(int(h.actor0)))
                a1 = str(PhysicsSchemaTools.intToSdfPath(int(h.actor1)))
                if not (
                    any(a0.startswith(p) for p in self._off_limit_prims)
                    or any(a1.startswith(p) for p in self._off_limit_prims)
                ):
                    continue
                ix = iy = iz = 0.0
                for k in range(int(h.num_contact_data)):
                    d = data[int(h.contact_data_offset) + k]
                    ix += float(d.impulse[0])
                    iy += float(d.impulse[1])
                    iz += float(d.impulse[2])
                CONTACT_EVENTS.append({
                    "type": str(h.type).rsplit(".", 1)[-1],   # CONTACT_FOUND / PERSIST / LOST
                    "a0": a0,
                    "a1": a1,
                    "impulse": [ix, iy, iz],
                    "n_contacts": int(h.num_contact_data),
                })
            except Exception:
                # Per skill: swallow per-event errors silently
                continue

