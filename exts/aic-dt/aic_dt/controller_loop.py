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
        """STUB — Plan 02-03 implements validation + buffering per D-09/D-11."""
        # Default skeleton behavior: buffer the message; Plan 02-03 adds validation.
        self._latest_joint_cmd = msg

    def _on_pose_cmd(self, msg):
        """STUB — Plan 02-04 implements validation + buffering per D-02/D-11."""
        self._latest_pose_cmd = msg

    # ------------------------------------------------------------------ #
    # Apply / publish — STUBS (Plans 02-03..06 implement bodies).
    # ------------------------------------------------------------------ #

    def _apply_joint_cmd(self, msg):
        """STUB — Plan 02-03 implements per D-06/D-09."""
        pass

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
        """STUB — Plan 02-04 implements LulaKinematicsSolver + ArticulationKinematicsSolver init."""
        return True

    def _setup_contact_subscription(self) -> bool:
        """STUB — Plan 02-06 implements omni.physx contact-report subscription per D-03."""
        return True
