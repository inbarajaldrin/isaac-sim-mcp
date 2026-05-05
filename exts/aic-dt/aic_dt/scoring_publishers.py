# Reference: parity_publishers.py (Phase 1 sibling) and scoring_tier2_contract.md
# (.planning/phases/03-cable-physics/snapshot/scoring_tier2_contract.md).
"""Phase 3 scoring publishers — /scoring/tf + /objects_poses_real + /scoring/insertion_event.

Plan 03-04/05 deliverable. Mirror of parity_publishers.py lifecycle (lazy
Articulation construction at tick 30, _on_physics_step callback that publishes
each tick, on_shutdown teardown).

Per ScoringTier2 contract:
- /scoring/tf: TFMessage of CABLE-RELATED transforms (cable links + plug + port)
- /objects_poses_real: TFMessage with object + cable + task_board frames (broader)
- /scoring/insertion_event: std_msgs/String, payload = port-namespace string,
  fired ONCE per insertion (re-arm on plug-port separation)

This file is a SKELETON authored 2026-05-05 mid-Plan-03-04 to pre-position the
next session. _publish_scoring_tf / _publish_objects_poses / _maybe_publish_insertion_event
bodies are STUBs — fill in per scoring_tier2_contract.md before integration.

Integration: extension.py::_start_aic_scoring_publishers() helper, called from
quick_start AFTER load_robot + setup_action_graph (so the cable subtree exists +
TF chain is up). Mirror of _start_aic_parity_publishers pattern.
"""
import os
import sys

# rclpy / ROS msgs ship for Python 3.11 in the pre-built workspace; ensure it's
# on sys.path before any rclpy import. Mirror of parity_publishers.py.
_RCLPY_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages"
_ISAAC_ROS_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local/lib/python3.11/dist-packages"

_STALE_310_FRAGMENTS = (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/dist-packages",
)
sys.path[:] = [p for p in sys.path if not any(s in p for s in _STALE_310_FRAGMENTS)]
for _new_path in (_ISAAC_ROS_311, _RCLPY_311):
    if os.path.isdir(_new_path) and _new_path not in sys.path:
        sys.path.insert(0, _new_path)


# Frame-name constants (per scoring_tier2_contract.md).
# child_frame_id MUST contain "task_board" substring to trigger ScoringTier2.cc:51-53
# static-TF registration (setTransform with is_static=true).
_SCORING_TF_FRAMES = [
    # (parent_frame, child_frame, usd_prim_path)
    ("world", "cable/sc_plug_link", "/World/UR5e/cable/Rope/Rope/link_20"),  # plug-end (Plan 03-01 probe)
    ("world", "cable/sc_port_link", "/World/TaskBoard/sc_port_1"),  # port (TBD verify path)
]

_OBJECTS_POSES_FRAMES = [
    # (parent, child, usd_prim_path) — all task_board children get "task_board" prefix per scoring contract
    ("world", "task_board_base", "/World/TaskBoard"),
    ("world", "task_board_sc_port_1", "/World/TaskBoard/sc_port_1"),
    ("world", "task_board_sc_port_2", "/World/TaskBoard/sc_port_2"),
    ("world", "task_board_nic_card", "/World/TaskBoard/nic_card"),
    # Cable frames (also broadcast on /objects_poses_real for grasp_points_publisher)
    ("world", "cable/sc_plug_link", "/World/UR5e/cable/Rope/Rope/link_20"),
]

# Plug-port pair for /scoring/insertion_event detection (Plan 03-05).
_PLUG_END_LINK_PATH = "/World/UR5e/cable/Rope/Rope/link_20"
_PORT_LINK_PATHS = [  # any of these as actor1 triggers an insertion event
    "/World/TaskBoard/sc_port_1",
    "/World/TaskBoard/sc_port_2",
    "/World/TaskBoard/nic_card",
]
# Sustained-contact threshold (matches Plan 02-06 PARITY-06 pattern)
_INSERTION_CONTACT_TICKS_REQUIRED = 5  # ~80ms at 60Hz


class AicScoringPublishers:
    """Phase 3 scoring-publisher class (Plan 03-04/05).

    Lifecycle mirrors AicParityPublishers (Phase 1). Single rclpy node owns
    three publishers + omni.physx contact subscription. Lazy Articulation
    construction at tick 30 to avoid the _physics_view init-order bug
    (Phase 2 finding).

    SKELETON — _publish_scoring_tf, _publish_objects_poses, and
    _maybe_publish_insertion_event are stubs. Fill in per
    .planning/phases/03-cable-physics/snapshot/scoring_tier2_contract.md.
    """

    def __init__(self, robot_xform_path: str = "/World/UR5e/aic_unified_robot"):
        self._robot_xform_path = robot_xform_path
        self._node = None
        self._scoring_tf_pub = None
        self._objects_poses_pub = None
        self._scoring_event_pub = None
        self._physx_step_sub = None
        self._physx_contact_sub = None
        self._stage = None
        self._articulation = None
        self._articulation_init_attempted = False
        self._articulation_init_wait_ticks = 0
        # Insertion-event state
        from collections import deque
        self._insertion_contact_events = deque(maxlen=2048)
        self._insertion_event_armed = False  # True after publishing; reset on separation
        # Log-once flags
        self._logged_publish_error = False

    def start(self) -> bool:
        """Construct rclpy node + 3 publishers + physx subscriptions. Idempotent."""
        self.stop()
        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
            from tf2_msgs.msg import TFMessage  # noqa: F401 — for publisher-creation
            from std_msgs.msg import String  # noqa: F401
        except ImportError as exc:
            print(f"[AIC-DT][scoring] rclpy/msg import failed: {exc!r}")
            return False

        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as exc:
                print(f"[AIC-DT][scoring] rclpy.init() failed: {exc!r}")
                return False

        try:
            self._node = rclpy.create_node("aic_dt_scoring_publishers")
        except Exception as exc:
            print(f"[AIC-DT][scoring] create_node failed: {exc!r}")
            return False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._scoring_tf_pub = self._node.create_publisher(TFMessage, "/scoring/tf", qos)
        self._objects_poses_pub = self._node.create_publisher(TFMessage, "/objects_poses_real", qos)
        self._scoring_event_pub = self._node.create_publisher(String, "/scoring/insertion_event", qos)

        import omni.usd
        self._stage = omni.usd.get_context().get_stage()

        try:
            from omni.physx import get_physx_interface
            self._physx_step_sub = get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        except Exception as exc:
            print(f"[AIC-DT][scoring] subscribe_physics_step_events failed: {exc!r}")
            return False

        # Insertion contact subscription (Plan 03-05 SKELETON — fill in)
        # See controller_loop._setup_contact_subscription (Plan 02-06) for the pattern.
        # TODO(Plan 03-05): apply PhysxContactReportAPI on plug-end link + port links,
        # subscribe via get_physx_simulation_interface().subscribe_contact_report_events.

        print("[AIC-DT][scoring] Started: /scoring/tf + /objects_poses_real + /scoring/insertion_event")
        return True

    def stop(self):
        if self._physx_step_sub is not None:
            try:
                self._physx_step_sub = None
            except Exception:
                pass
        if self._physx_contact_sub is not None:
            try:
                self._physx_contact_sub = None
            except Exception:
                pass
        if self._node is not None:
            try:
                if self._scoring_tf_pub is not None:
                    self._node.destroy_publisher(self._scoring_tf_pub)
                if self._objects_poses_pub is not None:
                    self._node.destroy_publisher(self._objects_poses_pub)
                if self._scoring_event_pub is not None:
                    self._node.destroy_publisher(self._scoring_event_pub)
                self._node.destroy_node()
            except Exception as exc:
                print(f"[AIC-DT][scoring] stop() teardown error: {exc!r}")
        self._node = None
        self._scoring_tf_pub = None
        self._objects_poses_pub = None
        self._scoring_event_pub = None
        self._stage = None
        self._articulation = None
        self._articulation_init_attempted = False
        self._articulation_init_wait_ticks = 0
        self._insertion_contact_events.clear()
        self._insertion_event_armed = False
        self._logged_publish_error = False

    def _on_physics_step(self, dt: float):
        if self._node is None:
            return
        # Lazy Articulation construction (Phase 2 _physics_view fix)
        if not self._articulation_init_attempted:
            self._articulation_init_wait_ticks += 1
            if self._articulation_init_wait_ticks >= 30:
                try:
                    from isaacsim.core.prims import Articulation
                    art_root = f"{self._robot_xform_path}/root_joint"
                    self._articulation = Articulation(prim_paths_expr=art_root)
                    self._articulation.initialize()
                    print(f"[AIC-DT][scoring] Articulation constructed lazily at tick {self._articulation_init_wait_ticks}")
                except Exception as exc:
                    print(f"[AIC-DT][scoring] Articulation construction failed: {exc!r}")
                    self._articulation = None
                self._articulation_init_attempted = True

        try:
            from rclpy.clock import Clock  # noqa: F401
            now = self._node.get_clock().now().to_msg()
        except Exception:
            return

        try:
            self._publish_scoring_tf(now)
            self._publish_objects_poses(now)
            self._maybe_publish_insertion_event(now)
        except Exception as exc:
            if not self._logged_publish_error:
                print(f"[AIC-DT][scoring] publish error: {exc!r}")
                self._logged_publish_error = True

    # ---- STUBS (Plan 03-04/05 fill in) ---- #

    def _publish_scoring_tf(self, stamp):
        """Publish TFMessage of cable-related transforms on /scoring/tf.

        TODO Plan 03-04: iterate _SCORING_TF_FRAMES, for each (parent, child, usd_path)
        build a TransformStamped with header.stamp=stamp, header.frame_id=parent,
        child_frame_id=child, transform=USD prim's world matrix. Append to TFMessage.
        Publish once per tick.

        Reference parity_publishers.py::_publish_tf_dynamic for the world-xform
        + Gf-quat-to-ROS-quat reordering pattern.
        """
        pass  # TODO

    def _publish_objects_poses(self, stamp):
        """Publish TFMessage on /objects_poses_real (broader: includes task_board frames).

        TODO Plan 03-04: iterate _OBJECTS_POSES_FRAMES same pattern. Note
        child_frame_ids prefixed with 'task_board' for task-board children — this
        triggers ScoringTier2.cc:51-53 static-TF registration on the consumer side.
        """
        pass  # TODO

    def _maybe_publish_insertion_event(self, stamp):
        """If sustained plug-port contact detected, publish std_msgs/String once.

        TODO Plan 03-05: drain self._insertion_contact_events deque (filled by
        physx contact callback). If ≥_INSERTION_CONTACT_TICKS_REQUIRED contacts
        in the last 5-tick window AND not already armed, build String with
        msg.data=detected_port_namespace, publish, set self._insertion_event_armed=True.
        Reset armed flag when contacts cease.
        """
        pass  # TODO

    def _on_insertion_contact_event(self, contact_headers, contact_data):
        """Physics-thread callback: O(1) append to _insertion_contact_events.

        TODO Plan 03-05: parse actor0/actor1 paths via PhysicsSchemaTools.intToSdfPath.
        If pair matches (plug_end_link, any port_link), append (timestamp, port_name).
        Mirror Plan 02-06's _on_contact_event in controller_loop.
        """
        pass  # TODO
