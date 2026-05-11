"""rclpy-based publishers for /joint_states, /tf, and /tf_static.

Replaces the OGN-based ROS2PublishJointState / ROS2PublishTransformTree action
graphs that Plan 06 wired -- those nodes have NO inputs for header.frame_id,
NO joint-name overrides, and derive TF frame_ids from USD prim leaf names
(which cannot legally contain '/' per Sdf.Path rules). AIC's surface uses
slashed frame names (`gripper/hande_base_link`, `cam_mount/cam_mount_link`,
etc.) and a 7-joint alphabetical /joint_states with frame_id="base_link" and
the literal `gripper/left_finger_joint` -- none of which the OGN nodes can
emit. This module bypasses OGN entirely and publishes via rclpy from inside
a physics-step callback.

Empirical findings driving this design (2026-05-03):
  - The unified USD's articulation has 6 DOF (UR5e arm only). Hand-E fingers
    are PhysicsFixedJoint (zero DOF, no articulation parent). Probe artifact:
    /tmp/probe_articulation.out.
  - Even with targetPrims pointed at the actual articulation root prim
    (/World/UR5e/aic_unified_robot/root_joint), ROS2PublishTransformTree
    emitted only the single World->aic_unified_robot edge. Articulation
    traversal documented in 5.0 OGN spec did not fire for this articulation.
  - 30 of 31 AIC frames map 1:1 to USD prims under /World/UR5e/aic_unified_robot/
    via mechanical underscore->slash translation. The 31st (`aic_world`) is a
    synthesized identity transform per Plan 05's TF research.

Mapping authority: exts/aic-dt/docs/topic-parity-reference.md
  - /joint_states: 7 alphabetical joints, frame_id="base_link",
    `gripper/left_finger_joint` literal name.
  - /tf: 8 dynamic transforms (6 arm + 2 finger).
  - /tf_static: 22 static frames (cameras, ATI, gripper base, world chain).
"""

import os
import sys

# rclpy / ROS msgs ship for Python 3.11 in the pre-built workspace; ensure it's
# on sys.path before any rclpy import. Mirrors the pattern in
# exts/ur5e-dt/ur5e_dt/extension.py:80-85.
_RCLPY_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages"
_ISAAC_ROS_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local/lib/python3.11/dist-packages"

# Empirically (2026-05-03 Gap-A/B inline fix), Isaac Sim's startup also adds
# /opt/ros/humble/local/lib/python3.10/dist-packages to sys.path. That path
# contains rcl_interfaces / sensor_msgs / etc. compiled against Python 3.10
# (so .so files have a `.cpython-310-*` extension that Python 3.11 cannot
# load). When `rclpy.create_node` walks the type-support import for a message
# class, the import resolver finds the Python 3.10 module file FIRST (because
# it's on sys.path) and tries to load its companion typesupport_c.so, which
# silently fails. We must REMOVE these stale python3.10 paths and prepend
# our python3.11 workspace paths so all message imports resolve to the 3.11
# build.
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

    Empirically (2026-05-03 inline Gap-A/B fix), Isaac Sim's startup leaves
    `/opt/ros/humble/local/lib/python3.10/dist-packages` and several
    `~/Desktop/ros2_ws/install/*/local/lib/python3.10/dist-packages` entries
    persistently on sys.path (likely re-injected by Carbonite or another
    Kit init step). Doing `sys.path.remove(...)` once at module load time
    is not enough -- the entries can re-appear before our publisher's
    actual import runs. This helper enforces the order at the moment we
    need it: just before each import of rclpy / message packages.
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

    Isaac Sim's internal ROS2 bridge attempts a system rclpy import during Kit
    startup; on this host that fails ("No module named 'rclpy._rclpy_pybind11'"
    -- the system Humble ships rclpy for Python 3.10, Isaac Sim runs 3.11) and
    leaves partially-initialized ROS modules cached. Subsequent imports
    succeed at the top level but submodule lookup uses the original (wrong)
    `__path__` set at the first failed import, producing errors like:

        rclpy.create_node failed: AttributeError: module 'rclpy' has no attribute 'impl'
        rcl_interfaces.rcl_interfaces_s__rosidl_typesupport_c not found
          (typesupport for ROS message types lives in package-local .so files
           whose visibility depends on `<package>.__path__`)

    Empirically (2026-05-03 Gap-A/B inline fix), evicting EVERY cached ROS
    package -- not just `rclpy.*` -- after putting the 3.11 dist-packages
    path on sys.path lets the next imports load typesupport-bearing message
    packages cleanly. The eviction set covers: rclpy core, rclpy generator
    packages (rcl_interfaces, sensor_msgs, tf2_msgs, geometry_msgs,
    std_msgs, builtin_interfaces, action_msgs, lifecycle_msgs, rosidl_*),
    rmw_*, and ros2cli helpers.
    """
    _ROS_PREFIXES = (
        "rclpy", "rcl_interfaces", "sensor_msgs", "tf2_msgs", "geometry_msgs",
        "std_msgs", "builtin_interfaces", "action_msgs", "lifecycle_msgs",
        "rosidl_runtime_py", "rosidl_generator_py", "rosidl_parser",
        "rmw", "rmw_dds_common", "rosgraph_msgs", "diagnostic_msgs",
        "trajectory_msgs", "visualization_msgs", "shape_msgs", "nav_msgs",
        "controller_manager_msgs", "control_msgs", "ros2pkg", "ament_index_python",
    )
    for mod_name in list(sys.modules):
        for prefix in _ROS_PREFIXES:
            if mod_name == prefix or mod_name.startswith(prefix + "."):
                del sys.modules[mod_name]
                break

from pxr import Sdf, Usd, UsdGeom, Gf
import omni.usd

# AIC frame name -> USD prim leaf name (relative to /World/UR5e/aic_unified_robot/)
# Source: empirical probe + topic-parity-reference.md.
_FRAME_TO_USD_LEAF = {
    "world": "world",
    "tabletop": "tabletop",
    "base_link": "base_link",
    "base": "base",
    "base_link_inertia": "base_link_inertia",
    "shoulder_link": "shoulder_link",
    "upper_arm_link": "upper_arm_link",
    "forearm_link": "forearm_link",
    "wrist_1_link": "wrist_1_link",
    "wrist_2_link": "wrist_2_link",
    "wrist_3_link": "wrist_3_link",
    "flange": "flange",
    "tool0": "tool0",
    "ft_frame": "ft_frame",
    "cam_mount/cam_mount_link": "cam_mount_cam_mount_link",
    "ati/base_link": "ati_base_link",
    "ati/tool_link": "ati_tool_link",
    "gripper/hande_base_link": "gripper_hande_base_link",
    "gripper/hande_finger_link_l": "gripper_hande_finger_link_l",
    "gripper/hande_finger_link_r": "gripper_hande_finger_link_r",
    "gripper/tcp": "gripper_tcp",
    "center_camera/camera_link": "center_camera_camera_link",
    "center_camera/sensor_link": "center_camera_sensor_link",
    "center_camera/optical": "center_camera_optical",
    "left_camera/camera_link": "left_camera_camera_link",
    "left_camera/sensor_link": "left_camera_sensor_link",
    "left_camera/optical": "left_camera_optical",
    "right_camera/camera_link": "right_camera_camera_link",
    "right_camera/sensor_link": "right_camera_sensor_link",
    "right_camera/optical": "right_camera_optical",
}

# TF parent->child edge list. Source: topic-parity-reference.md TF Tree
# section. Order does not matter -- each edge is published independently.
# (parent_frame, child_frame, is_dynamic)
_TF_EDGES = [
    # Static world chain
    ("world", "aic_world", False),     # synthesized identity transform
    ("world", "tabletop", False),
    ("tabletop", "base_link", False),
    ("base_link", "base_link_inertia", False),
    ("base_link", "base", False),
    # Dynamic UR5e arm
    ("base_link_inertia", "shoulder_link", True),
    ("shoulder_link", "upper_arm_link", True),
    ("upper_arm_link", "forearm_link", True),
    ("forearm_link", "wrist_1_link", True),
    ("wrist_1_link", "wrist_2_link", True),
    ("wrist_2_link", "wrist_3_link", True),
    # Static end effector chain
    ("wrist_3_link", "flange", False),
    ("wrist_3_link", "ft_frame", False),
    ("flange", "tool0", False),
    ("tool0", "cam_mount/cam_mount_link", False),
    ("cam_mount/cam_mount_link", "ati/base_link", False),
    ("ati/base_link", "ati/tool_link", False),
    ("ati/tool_link", "gripper/hande_base_link", False),
    # Dynamic Hand-E fingers (constant in Isaac Sim FixedJoint USD, but Gazebo
    # publishes them on /tf so we mirror the topic surface)
    ("gripper/hande_base_link", "gripper/hande_finger_link_l", True),
    ("gripper/hande_base_link", "gripper/hande_finger_link_r", True),
    # Static gripper TCP
    ("gripper/hande_base_link", "gripper/tcp", False),
    # Static cameras
    ("cam_mount/cam_mount_link", "center_camera/camera_link", False),
    ("center_camera/camera_link", "center_camera/sensor_link", False),
    ("center_camera/sensor_link", "center_camera/optical", False),
    ("cam_mount/cam_mount_link", "left_camera/camera_link", False),
    ("left_camera/camera_link", "left_camera/sensor_link", False),
    ("left_camera/sensor_link", "left_camera/optical", False),
    ("cam_mount/cam_mount_link", "right_camera/camera_link", False),
    ("right_camera/camera_link", "right_camera/sensor_link", False),
    ("right_camera/sensor_link", "right_camera/optical", False),
]

# /joint_states content. Source: topic-parity-reference.md /joint_states block.
_JOINT_FRAME_ID = "base_link"

# 7 names, alphabetical (NOT kinematic-chain). The 7th entry maps to a
# zero-DOF FixedJoint in the USD; we publish a constant 0.0 to satisfy
# aic_adapter's name-indexed reorder which expects the slashed name.
_JOINT_NAMES_ALPHA = [
    "elbow_joint",
    "gripper/left_finger_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
_GRIPPER_FAKE_JOINT = "gripper/left_finger_joint"


class AicParityPublishers:
    """rclpy-based publisher of /joint_states, /tf, /tf_static for AIC parity.

    Idempotent: start() can be called repeatedly; subsequent calls update the
    physics callback subscription rather than stacking duplicates. stop() is
    called from the extension shutdown path.
    """

    def __init__(self, robot_xform_path: str = "/World/UR5e/aic_unified_robot"):
        self._robot_xform_path = robot_xform_path
        self._node = None
        self._js_pub = None
        self._tf_pub = None
        self._tf_static_pub = None
        self._physx_sub = None
        self._articulation = None
        self._stage = None
        self._tf_static_msg = None
        self._tf_static_published = False
        self._dof_index_by_name = {}  # USD-prim-leaf joint name -> articulation DOF index

    # ----- lifecycle -----

    def start(self):
        """Create rclpy node, publishers, articulation handle, and physics callback.

        Safe to call repeatedly. On re-entry, tears down the prior subscription
        first so we don't accumulate physics callbacks on hot-reload.
        """
        self.stop()  # idempotent reset

        # Force Python 3.11 ROS workspace paths to the front and evict
        # stale Python 3.10 entries that Isaac Sim may have re-injected.
        # Then evict any half-loaded ROS modules from sys.modules so the
        # next import resolves to the 3.11 build cleanly.
        _force_python311_ros_paths()
        _ensure_rclpy_clean_import()

        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
            from sensor_msgs.msg import JointState
            from tf2_msgs.msg import TFMessage
        except ImportError as exc:
            print(f"[AIC-DT][parity] rclpy import failed: {exc!r}")
            print(f"[AIC-DT][parity] Ensure {_RCLPY_311} exists; see exts/aic-dt/docs/rclpy-setup.md")
            return False

        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as exc:
                print(f"[AIC-DT][parity] rclpy.init() failed: {exc!r}")
                return False

        # DEFERRED-5: use_sim_time=true so /joint_states + /tf + /tf_static carry
        # sim_clock timestamps. Engine consumes with use_sim_time=true; matching
        # the two avoids ScoringTier2::WaitForTfs spinlock timing out on its
        # first iteration when sim_clock and ROS_TIME diverge.
        from rclpy.parameter import Parameter
        self._node = rclpy.create_node(
            "aic_dt_parity_publisher",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )

        # /joint_states: RELIABLE / VOLATILE / KEEP_LAST(42) -- matches Gazebo
        # joint_state_broadcaster QoS per topic-parity-reference.md.
        js_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=42,
        )
        self._js_pub = self._node.create_publisher(JointState, "/joint_states", js_qos)

        # /tf: RELIABLE / VOLATILE / KEEP_LAST(10) -- matches Gazebo tf_relay.
        tf_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._tf_pub = self._node.create_publisher(TFMessage, "/tf", tf_qos)

        # /tf_static: RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1) -- the
        # Gazebo-default static TF QoS that lets late-joining subscribers
        # (e.g. CheatCode starting after Isaac Sim) still see the static tree.
        tf_static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._tf_static_pub = self._node.create_publisher(TFMessage, "/tf_static", tf_static_qos)

        self._stage = omni.usd.get_context().get_stage()

        # Articulation handle is constructed LAZILY on first physics tick
        # (see _on_physics_step). Constructing + initializing here, before
        # the first physics step, returns a handle without _physics_view —
        # every get_joint_positions / get_joint_velocities / get_measured_joint_efforts
        # then raises AttributeError("'Articulation' object has no attribute
        # '_physics_view'") and we silently publish stale/zero values to /joint_states.
        # Verified 2026-05-05 in controller_loop.py: same root cause as Bug 4;
        # fix is to defer construction to tick 30 of the physics loop.
        self._articulation = None
        self._articulation_init_attempted = False
        self._articulation_init_wait_ticks = 0

        # Subscribe to physics step events. omni.physx is the canonical 5.0
        # entry point for per-step callbacks driven by the PhysX update loop.
        # Note: from-import to avoid `import omni.physx` creating a function-
        # local `omni` binding that would shadow module-level `import omni.usd`.
        try:
            from omni.physx import get_physx_interface
            self._physx_sub = get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        except Exception as exc:
            print(f"[AIC-DT][parity] subscribe_physics_step_events failed: {exc!r}")

        print(f"[AIC-DT][parity] Started: /joint_states (7 joints, frame_id={_JOINT_FRAME_ID}), "
              f"/tf ({sum(1 for _,_,d in _TF_EDGES if d)} dynamic edges), "
              f"/tf_static ({sum(1 for _,_,d in _TF_EDGES if not d)} static edges)")
        return True

    def stop(self):
        """Tear down publishers, callbacks, and rclpy node."""
        try:
            if self._physx_sub is not None:
                # The subscription handle is a Carbonite IObject; releasing
                # it via Python `del` unsubscribes per the omni.physx pattern.
                self._physx_sub = None
        except Exception:
            pass

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        self._js_pub = None
        self._tf_pub = None
        self._tf_static_pub = None
        self._stage = None
        self._articulation = None
        self._tf_static_msg = None
        self._tf_static_published = False
        self._dof_index_by_name = {}

    # ----- per-step callback -----

    def _on_physics_step(self, dt: float):
        """Read sim state and publish /joint_states + /tf each tick.

        /tf_static is published once on the first tick (TRANSIENT_LOCAL keeps
        late subscribers happy without re-publishing).
        """
        if self._node is None:
            return
        # Lazy articulation construction. See start() for why we can't do
        # this at start-time. Wait 30 ticks (~0.5s @ 60Hz) for physics_sim_view
        # to be ready, then construct fresh.
        if not self._articulation_init_attempted:
            self._articulation_init_wait_ticks += 1
            if self._articulation_init_wait_ticks >= 30:
                try:
                    from isaacsim.core.prims import Articulation
                    art_root = f"{self._robot_xform_path}/root_joint"
                    self._articulation = Articulation(prim_paths_expr=art_root)
                    self._articulation.initialize()
                    self._dof_index_by_name = {}
                    for idx, name in enumerate(self._articulation.dof_names or []):
                        self._dof_index_by_name[name] = idx
                    has_view = hasattr(self._articulation, "_physics_view") and self._articulation._physics_view is not None
                    print(f"[AIC-DT][parity] Articulation constructed lazily at tick {self._articulation_init_wait_ticks} (has_physics_view={has_view}, dof_names={list(self._dof_index_by_name.keys())})")
                except Exception as exc:
                    print(f"[AIC-DT][parity] Articulation construction failed: {exc!r}")
                    self._articulation = None
                self._articulation_init_attempted = True
        # Capture sim time once for header consistency across messages.
        try:
            from rclpy.clock import Clock
            now = self._node.get_clock().now().to_msg()
        except Exception:
            return

        try:
            self._publish_joint_state(now)
            self._publish_tf_dynamic(now)
            if not self._tf_static_published:
                self._publish_tf_static(now)
                self._tf_static_published = True
        except Exception as exc:
            # Don't crash the physics loop on a publish error -- log once.
            if not getattr(self, "_logged_publish_error", False):
                print(f"[AIC-DT][parity] publish error: {exc!r}")
                self._logged_publish_error = True

    # ----- /joint_states -----

    def _publish_joint_state(self, stamp):
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = _JOINT_FRAME_ID
        msg.name = list(_JOINT_NAMES_ALPHA)

        # Read articulation positions / velocities / efforts. Six arm DOFs.
        positions = [0.0] * 7
        velocities = [0.0] * 7
        efforts = [0.0] * 7

        if self._articulation is not None:
            try:
                pos = self._articulation.get_joint_positions()
                vel = self._articulation.get_joint_velocities()
                # efforts: get_applied_actions / get_measured_joint_efforts may
                # not be available; tolerate.
                eff = None
                try:
                    eff = self._articulation.get_measured_joint_efforts()
                except Exception:
                    eff = None
                # pos/vel are torch.Tensor or numpy depending on backend; index 0.
                if pos is not None and len(pos) > 0:
                    pos0 = pos[0] if hasattr(pos[0], "__len__") else pos
                    vel0 = vel[0] if vel is not None and len(vel) > 0 and hasattr(vel[0], "__len__") else (vel if vel is not None else None)
                    eff0 = eff[0] if eff is not None and len(eff) > 0 and hasattr(eff[0], "__len__") else (eff if eff is not None else None)
                    for alpha_idx, name in enumerate(_JOINT_NAMES_ALPHA):
                        if name == _GRIPPER_FAKE_JOINT:
                            continue  # leaves 0.0 fake
                        dof_idx = self._dof_index_by_name.get(name)
                        if dof_idx is not None and dof_idx < len(pos0):
                            positions[alpha_idx] = float(pos0[dof_idx])
                            if vel0 is not None and dof_idx < len(vel0):
                                velocities[alpha_idx] = float(vel0[dof_idx])
                            if eff0 is not None and dof_idx < len(eff0):
                                efforts[alpha_idx] = float(eff0[dof_idx])
            except Exception as exc:
                if not getattr(self, "_logged_js_read", False):
                    print(f"[AIC-DT][parity] articulation read failed: {exc!r}")
                    self._logged_js_read = True

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts

        self._js_pub.publish(msg)

    # ----- /tf and /tf_static -----

    def _build_transform_msg(self, parent_frame, child_frame, stamp, transform_gf):
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        translation = transform_gf.ExtractTranslation()
        rot = transform_gf.ExtractRotationQuat()
        # Gf.Quatd.GetReal()/GetImaginary() ordering: real (w), imaginary (x,y,z)
        imag = rot.GetImaginary()
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(imag[0])
        t.transform.rotation.y = float(imag[1])
        t.transform.rotation.z = float(imag[2])
        t.transform.rotation.w = float(rot.GetReal())
        return t

    def _world_xform(self, prim):
        """Return the world-space matrix of a prim, or None if not Xformable."""
        if not prim or not prim.IsValid():
            return None
        x = UsdGeom.Xformable(prim)
        if not x:
            return None
        return x.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _relative_xform(self, parent_frame, child_frame):
        """Compute the transform from parent_frame to child_frame as Gf.Matrix4d.

        Returns identity for the synthesized aic_world edge; returns None if
        either prim cannot be resolved.
        """
        if parent_frame == "world" and child_frame == "aic_world":
            return Gf.Matrix4d(1.0)  # synthesized identity per Plan 05 research

        parent_leaf = _FRAME_TO_USD_LEAF.get(parent_frame)
        child_leaf = _FRAME_TO_USD_LEAF.get(child_frame)
        if not parent_leaf or not child_leaf:
            return None

        parent_prim = self._stage.GetPrimAtPath(f"{self._robot_xform_path}/{parent_leaf}")
        child_prim = self._stage.GetPrimAtPath(f"{self._robot_xform_path}/{child_leaf}")

        parent_world = self._world_xform(parent_prim)
        child_world = self._world_xform(child_prim)
        if parent_world is None or child_world is None:
            return None
        return child_world * parent_world.GetInverse()

    def _publish_tf_dynamic(self, stamp):
        from tf2_msgs.msg import TFMessage
        transforms = []
        for parent_f, child_f, is_dynamic in _TF_EDGES:
            if not is_dynamic:
                continue
            xform = self._relative_xform(parent_f, child_f)
            if xform is None:
                continue
            transforms.append(self._build_transform_msg(parent_f, child_f, stamp, xform))
        if transforms:
            self._tf_pub.publish(TFMessage(transforms=transforms))

    def _publish_tf_static(self, stamp):
        from tf2_msgs.msg import TFMessage
        transforms = []
        for parent_f, child_f, is_dynamic in _TF_EDGES:
            if is_dynamic:
                continue
            xform = self._relative_xform(parent_f, child_f)
            if xform is None:
                continue
            transforms.append(self._build_transform_msg(parent_f, child_f, stamp, xform))
        if transforms:
            self._tf_static_pub.publish(TFMessage(transforms=transforms))
            print(f"[AIC-DT][parity] /tf_static published: {len(transforms)} static frames")
