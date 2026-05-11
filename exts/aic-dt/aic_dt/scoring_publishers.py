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
#
# DEFERRED-3 (2026-05-10): legacy snake_case paths (`/World/TaskBoard/sc_port_1`,
# `/World/UR5e/cable/Rope/Rope/link_20`) no longer match the live scene which
# uses CamelCase namespace (`/World/TaskBoard/SCPort_0`, etc.) and keeps the
# cable subtree SetActive(False) until Phase 3 SCENE-05 fully lands. With ALL
# entries pointing at invalid prims, `_publish_frame_list` skipped every frame,
# producing an empty TFMessage which was then gated off by `if msg.transforms:`
# (line ~380) — net result: /scoring/tf had publisher count=5 but ZERO messages.
# Engine's ready_scoring → WaitForTfs timed out after 10s, scoring setup failed
# 5 retries, trial never reached task execution.
#
# Minimal fix: include /World/TaskBoard (always valid post-quick_start) as a
# fallback so the message always carries at least one transform. The
# cable plug + port entries are still listed (using live CamelCase paths) and
# will populate the message when their prims become valid (cable activation
# is gated on attach_cable_to_gripper=True + Phase 3 SCENE-05 fully landing).
_SCORING_TF_FRAMES = [
    # (parent_frame, child_frame, usd_prim_path)
    # Always-valid fallback — guarantees /scoring/tf publishes ≥1 transform/tick
    # so engine's WaitForTfs() gate fires, even when cable subtree is inactive.
    ("world", "task_board", "/World/TaskBoard"),
    # Live SCPort path — replaces stale `/World/TaskBoard/sc_port_1`. Maps to
    # trial_1's primary port (sfp_sc_cable plug target).
    ("world", "task_board_sc_port_0", "/World/TaskBoard/SCPort_0/sc_port_visual"),
    # Cable plug-end — silently skipped while cable subtree is inactive
    # (Phase 3 SCENE-05 work to activate per trial). Path updated to match
    # the live cable USD topology where it does become valid.
    ("world", "cable/sc_plug_link", "/World/UR5e/cable/sc_plug_visual"),
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
# Both paths qualify as "the plug" for contact-report purposes:
#   - /World/UR5e/cable/Rope/Rope/link_20 — cable rope chain end (Plan 03-01
#     topology probe). RigidBodyAPI + CollisionAPI live on link_20 itself.
#   - /World/UR5e/cable/sc_plug_visual — plug visual body (RigidBodyAPI on the
#     parent prim, CollisionAPI on 13 child mesh prims). taskboard-prim-authoring
#     iter (2026-05-09) discovered that sustained (link_20 ↔ port) contact is
#     rare under live cable-physics + kinematic-port stimulus, while
#     (sc_plug_visual ↔ port) contacts dominate — so the rope-end-only filter
#     was effectively starving _maybe_publish_insertion_event of input. Both
#     are mechanically the plug for trial scoring.
_PLUG_PATH_PREFIXES = (
    "/World/UR5e/cable/Rope/Rope/link_20",
    "/World/UR5e/cable/sc_plug_visual",
)
# Backwards-compat alias retained for any external imports / test harnesses.
_PLUG_END_LINK_PATH = _PLUG_PATH_PREFIXES[0]
# Default _PORT_LINK_PATHS — used by Phase 3 cable-only scenes that don't
# spawn the full task board. Plan 04-03 D-13 fallback (Wave 1 A4=MISMATCH):
# load_trial overrides via AicScoringPublishers.set_port_link_paths(...) to
# the live spawn paths (CamelCase namespace under /World/TaskBoard/).
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
        # D-13 fallback (Plan 04-03, Wave 1 A4=MISMATCH): per-instance override of
        # the module-level _PORT_LINK_PATHS list. Populated by load_trial after
        # spawn-atom dispatch. None = use module-level default.
        self._port_link_paths_override = None
        self._trial_tf_frames = []
        self._trial_tf_pub = None  # /tf publisher for CheatCode-expected per-trial frames

    def set_port_link_paths(self, paths) -> None:
        """D-13 fallback (Plan 04-03): override the hardcoded _PORT_LINK_PATHS
        list with the live spawn paths computed by load_trial.

        MUST be called BEFORE start() — the contact-report subscription is
        wired at start time, not on each tick. Calling after start() is a no-op
        for the active subscription (would require stop() + start() cycle).

        Args:
            paths: iterable of USD prim path strings (e.g. ['/World/TaskBoard/SCPort_0', ...]).
        """
        self._port_link_paths_override = list(paths) if paths is not None else None
        print(f"[AIC-DT][scoring] set_port_link_paths: {len(self._port_link_paths_override or [])} paths "
              f"(override active={self._port_link_paths_override is not None})")

    def _effective_port_link_paths(self):
        """Return the active port-link path list (override if set, else module default)."""
        if self._port_link_paths_override is not None:
            return self._port_link_paths_override
        return _PORT_LINK_PATHS

    def set_trial_tf_frames(self, frames):
        """DEFERRED-4 (2026-05-10): publish per-trial CheatCode-expected TF frames.

        CheatCode.insert_cable() looks up two trial-specific frames from base_link:
          - port:  `task_board/<target_module_name>/<port_name>_link`
          - plug:  `<cable_name>/<plug_name>_link`
        Without these on /tf or /tf_static, CheatCode bails after 10s with
        "Transform not available" and insert_cable() returns False — tier_2/3
        score stays 0 by infrastructure, not by physics.

        `frames` is a list of dicts shaped:
          {
            'parent': str,           # parent frame (typically 'base_link' or 'world')
            'child':  str,           # exact frame_id CheatCode will look up
            'usd_anchor_path': str,  # USD prim whose live world pose anchors the frame
            'offset_translation': (x, y, z),  # local offset from anchor (m)
            'offset_quat_xyzw':   (x, y, z, w),  # local rotation from anchor
          }
        At each physics tick the live anchor world pose is computed and the
        offset applied to yield the published transform. Frames are republished
        on /tf at physics rate (RELIABLE, KeepLast(10)) — same QoS the existing
        parity_publishers /tf uses, so a single TransformListener aggregates
        across both publishers.

        Pass an empty list to clear all trial frames (called from load_trial
        reset path or quick_start re-entry).
        """
        self._trial_tf_frames = list(frames) if frames else []
        print(f"[AIC-DT][scoring] set_trial_tf_frames: {len(self._trial_tf_frames)} frame(s) "
              f"({[f['child'] for f in self._trial_tf_frames]})")

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
        # DEFERRED-4: CheatCode-expected per-trial frames go on /tf (default
        # TransformListener subscription). Separate publisher from parity's
        # /tf — ROS happily aggregates multiple publishers on the same topic.
        self._trial_tf_pub = self._node.create_publisher(TFMessage, "/tf", qos)

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

        # Insertion contact subscription (Plan 03-05). Mirror of
        # controller_loop._setup_contact_subscription (Plan 02-06 PARITY-06):
        # apply PhysxContactReportAPI(threshold=0.0) on plug + port prims,
        # then subscribe to physx_simulation_interface contact-report events.
        try:
            from pxr import UsdPhysics, PhysxSchema
            from omni.physx import get_physx_simulation_interface
            # The port prims spawned by Plan 01-09 atoms are bare Xform
            # containers — the actual RigidBody descendant lives at
            #   /World/TaskBoard/SCPort_{i}/sc_port_visual
            #   /World/TaskBoard/NICCardMount_{i}/<visual subprim>
            # We must walk each subtree to find rigid bodies and apply the
            # contact-report API on each. Phase 4 Plan 04-03 fix branch (b)
            # — the original code only checked the root prim, missing all
            # the actual rigid descendants.
            def _tag_rigid_bodies_under(root_path):
                """Apply PhysxContactReportAPI to all RigidBodyAPI prims in
                subtree (or to root_path itself if it's a rigid body).
                Returns (applied_count, paths_tagged)."""
                count = 0
                tagged = []
                root = self._stage.GetPrimAtPath(root_path)
                if not root or not root.IsValid():
                    return 0, []
                # Check root + walk all descendants
                if root.HasAPI(UsdPhysics.RigidBodyAPI):
                    api = PhysxSchema.PhysxContactReportAPI.Apply(root)
                    api.CreateThresholdAttr().Set(0.0)
                    count += 1
                    tagged.append(str(root.GetPath()))
                # Walk descendants via stage.Traverse with prefix filter
                # (works regardless of Xform/Mesh nesting depth).
                for prim in self._stage.Traverse():
                    p = str(prim.GetPath())
                    if not p.startswith(root_path + "/"):
                        continue
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                        api.CreateThresholdAttr().Set(0.0)
                        count += 1
                        tagged.append(p)
                return count, tagged

            applied = 0
            effective_ports = self._effective_port_link_paths()
            tag_audit = {}
            for plug_prefix in _PLUG_PATH_PREFIXES:
                plug_count, plug_tagged = _tag_rigid_bodies_under(plug_prefix)
                applied += plug_count
                tag_audit[plug_prefix] = plug_tagged
            for path in effective_ports:
                c, t = _tag_rigid_bodies_under(path)
                applied += c
                tag_audit[path] = t
            self._physx_contact_sub = (
                get_physx_simulation_interface()
                .subscribe_contact_report_events(self._on_insertion_contact_event)
            )
            print(f"[AIC-DT][scoring] insertion contact subscription wired ({applied} prims tagged with PhysxContactReportAPI)")
            print(f"[AIC-DT][scoring] active port paths: {effective_ports}")
            print(f"[AIC-DT][scoring] tag audit: {tag_audit}")
        except Exception as exc:
            print(f"[AIC-DT][scoring] insertion contact subscription failed: {exc!r}")

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
            self._publish_trial_tf_frames(now)
            self._maybe_publish_insertion_event(now)
        except Exception as exc:
            if not self._logged_publish_error:
                print(f"[AIC-DT][scoring] publish error: {exc!r}")
                self._logged_publish_error = True

    # ---- STUBS (Plan 03-04/05 fill in) ---- #

    def _publish_scoring_tf(self, stamp):
        """Publish TFMessage of cable-related transforms on /scoring/tf."""
        self._publish_frame_list(stamp, _SCORING_TF_FRAMES, self._scoring_tf_pub)

    def _publish_trial_tf_frames(self, stamp):
        """DEFERRED-4: publish per-trial CheatCode-expected TF frames on /tf.

        For each frame in self._trial_tf_frames, compute the published transform
        as `anchor_world @ offset` where `anchor_world` is the live USD prim
        pose and `offset` is the static SE(3) offset from the anchor to the
        expected frame (typically read from tools/anchor_target_offsets.yaml).
        Empty list → no-op.
        """
        if self._trial_tf_pub is None or self._stage is None or not self._trial_tf_frames:
            return
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        from pxr import UsdGeom, Gf
        msg = TFMessage()
        xf_cache = UsdGeom.XformCache()
        for f in self._trial_tf_frames:
            parent_frame = f.get('parent', 'world')
            child_frame = f.get('child')
            anchor_path = f.get('usd_anchor_path')  # may be None for pure-static frames
            ox, oy, oz = f.get('offset_translation', (0.0, 0.0, 0.0))
            qx, qy, qz, qw = f.get('offset_quat_xyzw', (0.0, 0.0, 0.0, 1.0))
            if not child_frame:
                continue
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            if anchor_path:
                # Mode 1: anchored to a USD prim's world pose. Published value
                # is the prim's world transform composed with the local offset.
                # `parent_frame` SHOULD be 'world' for this to be semantically
                # consistent with the live USD root.
                prim = self._stage.GetPrimAtPath(anchor_path)
                if not prim or not prim.IsValid() or not prim.IsActive():
                    continue
                try:
                    anchor_xf = xf_cache.GetLocalToWorldTransform(prim)
                except Exception:
                    continue
                offset_xf = Gf.Matrix4d().SetIdentity()
                offset_xf.SetTranslateOnly(Gf.Vec3d(float(ox), float(oy), float(oz)))
                offset_xf.SetRotateOnly(Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz))))
                world_xf = offset_xf * anchor_xf
                translation = world_xf.ExtractTranslation()
                rot = world_xf.ExtractRotationQuat()
                imag = rot.GetImaginary()
                t.transform.translation.x = float(translation[0])
                t.transform.translation.y = float(translation[1])
                t.transform.translation.z = float(translation[2])
                t.transform.rotation.x = float(imag[0])
                t.transform.rotation.y = float(imag[1])
                t.transform.rotation.z = float(imag[2])
                t.transform.rotation.w = float(rot.GetReal())
            else:
                # Mode 2: pure-static transform relative to `parent_frame`.
                # TransformListener chains through the existing /tf graph
                # (e.g. base_link → ... → gripper/tcp → child) so no live
                # USD anchor lookup is needed.
                t.transform.translation.x = float(ox)
                t.transform.translation.y = float(oy)
                t.transform.translation.z = float(oz)
                t.transform.rotation.x = float(qx)
                t.transform.rotation.y = float(qy)
                t.transform.rotation.z = float(qz)
                t.transform.rotation.w = float(qw)
            msg.transforms.append(t)
        if msg.transforms:
            self._trial_tf_pub.publish(msg)

    def _publish_objects_poses(self, stamp):
        """Publish TFMessage on /objects_poses_real (broader: includes task_board frames).

        child_frame_ids prefixed with 'task_board' for task-board children
        triggers ScoringTier2.cc:51-53 static-TF registration on the consumer side.
        """
        self._publish_frame_list(stamp, _OBJECTS_POSES_FRAMES, self._objects_poses_pub)

    def _publish_frame_list(self, stamp, frame_list, publisher):
        """Shared TFMessage builder — mirrors parity_publishers.py::_build_transform_msg.

        For each (parent, child, usd_path) tuple, build a TransformStamped from
        the USD prim's world transform and publish in a single TFMessage.
        Skips frames whose USD prim isn't valid (silent — common during scene
        transitions; not an error).
        """
        if publisher is None or self._stage is None:
            return
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        from pxr import UsdGeom
        msg = TFMessage()
        xf_cache = UsdGeom.XformCache()
        for parent_frame, child_frame, usd_path in frame_list:
            prim = self._stage.GetPrimAtPath(usd_path)
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            try:
                xf = xf_cache.GetLocalToWorldTransform(prim)
            except Exception:
                continue
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            translation = xf.ExtractTranslation()
            rot = xf.ExtractRotationQuat()
            imag = rot.GetImaginary()
            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])
            # Gf.Quatd: GetReal()=w, GetImaginary()=(x,y,z). ROS Quaternion: (x,y,z,w).
            t.transform.rotation.x = float(imag[0])
            t.transform.rotation.y = float(imag[1])
            t.transform.rotation.z = float(imag[2])
            t.transform.rotation.w = float(rot.GetReal())
            msg.transforms.append(t)
        if msg.transforms:
            publisher.publish(msg)

    def _maybe_publish_insertion_event(self, stamp):
        """If sustained plug-port contact detected, publish std_msgs/String once.

        Drains the deque populated by _on_insertion_contact_event. Counts
        contacts in the last 100ms window. If ≥_INSERTION_CONTACT_TICKS_REQUIRED
        AND not already armed, publishes String(data=port_name) and arms the
        flag. Re-arms when ≥100ms of no contacts elapses.
        """
        if self._scoring_event_pub is None:
            return
        import time
        now_t = time.time()
        recent = [(t, port) for (t, port) in self._insertion_contact_events
                  if now_t - t < 0.1]
        if not recent:
            # No recent contacts — re-arm if previously armed
            self._insertion_event_armed = False
            return
        if self._insertion_event_armed:
            return  # already published this insertion
        if len(recent) < _INSERTION_CONTACT_TICKS_REQUIRED:
            return  # not yet sustained
        # Pick the most-recent port name as the "inserted into" target
        port_name = recent[-1][1]
        from std_msgs.msg import String
        msg = String()
        msg.data = port_name
        self._scoring_event_pub.publish(msg)
        self._insertion_event_armed = True
        print(f"[AIC-DT][scoring] published /scoring/insertion_event data={port_name!r}")

    def _on_insertion_contact_event(self, contact_headers, contact_data):
        """Physics-thread callback (O(1) append to _insertion_contact_events).

        Filters for (plug_end_link, port_link) pairs. Port name extracted from
        the actor path's basename. Mirror of controller_loop.py::_on_contact_event
        from Plan 02-06.
        """
        try:
            from omni.physx.scripts.physicsUtils import PhysicsSchemaTools
        except Exception:
            try:
                from pxr import PhysicsSchemaTools
            except Exception:
                return
        import time
        now_t = time.time()
        for header in contact_headers:
            try:
                actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
                actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            except Exception:
                continue
            # Determine if this is a plug↔port pair (use effective override if set)
            def _is_plug(path):
                return any(path.startswith(prefix) for prefix in _PLUG_PATH_PREFIXES)
            for port_path in self._effective_port_link_paths():
                hit = (
                    (_is_plug(actor0) and actor1.startswith(port_path))
                    or (_is_plug(actor1) and actor0.startswith(port_path))
                )
                if hit:
                    port_name = port_path.rsplit("/", 1)[-1]
                    self._insertion_contact_events.append((now_t, port_name))
                    break
