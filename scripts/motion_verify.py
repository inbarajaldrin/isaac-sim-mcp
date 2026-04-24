#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
SO-ARM101 Motion Verifier — collision replay of captured motion data.

Every captured motion (CSV + /tmp/arm_traj/*.json plan dump) is a snapshot of
two trajectories: the plan MoveIt validated, and the Isaac physics path that
actually executed. Layer-B drive lag can make those diverge (see
docs/DEBUG-GUIDE.md § 4-5).

This tool answers, for any captured motion:

    Given the scene MoveIt can see, was the Isaac-executed trajectory ever
    in a state MoveIt would have rejected — even though the plan was accepted?

Method (per sample, downsampled by --every):
  1. /check_state_validity on plan ref joints  → sanity: should be 100% valid
  2. /check_state_validity on Isaac joints     → the failure mode
  3. /compute_fk on plan ref  → plan TCP
     Isaac TCP comes from the CSV (computed live from Isaac xforms during capture)
     → Cartesian divergence between plan TCP and Isaac TCP at each sample.

The attached lego from the plan's scene_at_plan_time is re-attached inline
via the request's RobotState.attached_collision_objects (is_diff=true) — no
live-scene mutation needed. If the motion had no attached lego, that step
is skipped.

IMPORTANT CAVEATS:
  * Uses the LIVE planning scene for cups/world. If cups have moved since
    capture, reset them per DEBUG-GUIDE.md § 11 before running.
  * Lego geometry is approximated as an axis-aligned bounding box (fallback
    when the STL isn't easily reachable from this process). Gripper-link vs
    cup contacts — the dominant failure mode per § 7 — are exact.

Usage:
    scripts/motion_verify.py <csv>                 verify one motion
    scripts/motion_verify.py --latest              most recent motion
    scripts/motion_verify.py --scan                every captured motion (slow)
    scripts/motion_verify.py <csv> --every 5       check every 5th Isaac sample
    scripts/motion_verify.py <csv> --fine-interp 2
                                                   add a sub-sample geometric
                                                   mesh-vs-cup sweep at 2 ms
                                                   resolution (closes the 50 Hz
                                                   sampling gap — see § 4.1)

Exit codes: 0 OK; 1 file/IO error; 2 services unavailable.
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import (
        GetStateValidity, GetPositionFK, GetPlanningScene, ApplyPlanningScene,
    )
    from moveit_msgs.msg import (
        RobotState, AttachedCollisionObject, CollisionObject, PlanningScene,
        PlanningSceneComponents,
    )
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    ROS2_OK = True
except ImportError as _ros_e:
    ROS2_OK = False
    _IMPORT_ERR = str(_ros_e)

# Fine-interp mesh sweep helper (local FK + URDF STL intersection). Optional
# — only used when --fine-interp is requested. Imported lazily to keep
# ROS-less environments working.
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from _mesh_sweep import MeshSweeper, default_sweeper  # noqa: E402
    MESH_SWEEP_OK = True
except ImportError as _m_e:
    MESH_SWEEP_OK = False
    _MESH_SWEEP_ERR = str(_m_e)


ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper_joint"
ALL_JOINTS = ARM + [GRIPPER]
TCP_LINK = "tcp_link"

# Fallback lego bounding-box dimensions (m) by size suffix. Source:
# vla_SO-ARM101 lego STLs — used only when the STL isn't attachable directly.
_LEGO_BBOX = {
    "2x2": (0.016, 0.016, 0.0192),
    "2x3": (0.016, 0.024, 0.0192),
    "2x4": (0.016, 0.032, 0.0192),
}


def _f(v):
    try:
        return float(v) if v not in ("", None) else None
    except Exception:
        return None


def _read_csv(path: Path):
    with path.open() as f:
        return list(csv.DictReader(f))


class Verifier:
    """rclpy wrapper kept separate from the reporting layer so we can init
    rclpy once in main() and reuse across many motions in --scan mode."""

    def __init__(self, node: "Node"):
        self.node = node
        self.validity_client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.fk_client = node.create_client(GetPositionFK, "/compute_fk")
        self.get_scene_client = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.apply_scene_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        # Cache of original cup mesh_poses so we can restore on exit.
        self._saved_cup_poses = None  # dict: cup_id -> list[Pose]
        self.live_attached_ids = []   # stale attachments to detach per check

    def wait_services(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (self.validity_client.service_is_ready()
                    and self.fk_client.service_is_ready()
                    and self.get_scene_client.service_is_ready()
                    and self.apply_scene_client.service_is_ready()):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    # ─── Cup-scene override (cups are stored as meshes with mesh_poses) ────

    def snapshot_cup_poses(self):
        """Fetch current cup_drop_* OBJECT root poses + live attached objects.
        Caches cup poses for restore(). Note: MoveIt's CollisionObject.pose is
        the root pose; mesh_poses[] are *relative* to that root — we override
        the root, not the mesh."""
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        fut = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        resp = fut.result()
        if not resp:
            return None
        saved = {}  # cup_id -> Pose (root pose snapshot)
        cups_present = {}  # cup_id -> CollisionObject (frame_id + geometry ref)
        for co in resp.scene.world.collision_objects:
            if co.id.startswith("cup_drop_"):
                saved[co.id] = Pose(position=co.pose.position,
                                    orientation=co.pose.orientation)
                cups_present[co.id] = co
        # Remember what's currently attached so we can inline-detach it per
        # check_state() call without actually mutating the scene.
        self.live_attached_ids = [
            aco.object.id
            for aco in resp.scene.robot_state.attached_collision_objects
        ]
        self._saved_cup_poses = saved
        self._cups_present = cups_present
        return saved

    def apply_cup_overrides(self, drop_data: dict):
        """Update cup_drop_N object root pose to match drop_data['drop_N'].
        `drop_data` is from the plan dump's scene_at_plan_time — authoritative
        cup positions at plan time. Returns True on success."""
        if not self._cups_present:
            return False
        diff_scene = PlanningScene()
        diff_scene.is_diff = True
        applied = 0
        for cup_id, co_original in self._cups_present.items():
            suffix = cup_id.split("cup_drop_")[-1]  # 'cup_drop_0' → '0'
            d = drop_data.get(f"drop_{suffix}")
            if not d:
                continue
            co = CollisionObject()
            co.id = cup_id
            co.header.frame_id = co_original.header.frame_id or "base"
            co.pose.position.x = float(d.get("x", 0.0))
            co.pose.position.y = float(d.get("y", 0.0))
            co.pose.position.z = float(d.get("z", 0.0))
            co.pose.orientation.w = 1.0
            co.operation = CollisionObject.MOVE
            diff_scene.world.collision_objects.append(co)
            applied += 1
        if applied == 0:
            return False
        return self._apply_scene_diff(diff_scene)

    def restore_cup_poses(self):
        if not self._saved_cup_poses or not self._cups_present:
            return
        diff_scene = PlanningScene()
        diff_scene.is_diff = True
        for cup_id, saved_pose in self._saved_cup_poses.items():
            co_original = self._cups_present.get(cup_id)
            if not co_original:
                continue
            co = CollisionObject()
            co.id = cup_id
            co.header.frame_id = co_original.header.frame_id or "base"
            co.pose = saved_pose
            co.operation = CollisionObject.MOVE
            diff_scene.world.collision_objects.append(co)
        self._apply_scene_diff(diff_scene)
        self._saved_cup_poses = None
        self._cups_present = None

    def _apply_scene_diff(self, diff_scene: "PlanningScene") -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = diff_scene
        fut = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        resp = fut.result()
        return bool(resp and resp.success)

    def _attached_lego_aco(self, lego_name: str, lego_size_hint: str = None):
        """Build an AttachedCollisionObject approximating the lego as a box
        rigidly attached to tcp_link (matches the live GUI convention:
        tcp_link ↔ attached, with touch_links on gripper/jaw/tcp_link).
        lego_size_hint extracts from the name (e.g. 'blue_2x3' → '2x3');
        falls back to 2x2."""
        size = lego_size_hint
        if not size and lego_name:
            parts = lego_name.split("_")
            if len(parts) >= 2 and parts[-1] in _LEGO_BBOX:
                size = parts[-1]
        dims = _LEGO_BBOX.get(size, _LEGO_BBOX["2x2"])

        aco = AttachedCollisionObject()
        aco.link_name = "tcp_link"
        co = CollisionObject()
        co.id = f"attached_{lego_name or 'lego'}"
        co.header.frame_id = "tcp_link"
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(dims)
        co.primitives.append(prim)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0  # tcp_link frame sits between fingertips already
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        aco.object = co
        aco.touch_links = ["gripper", "jaw", "tcp_link"]
        return aco

    def check_state(self, joints: dict, attached_aco=None,
                    detach_ids: list = None):
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(ALL_JOINTS)
        req.robot_state.joint_state.position = [
            float(joints.get(n, 0.0)) for n in ALL_JOINTS
        ]
        req.robot_state.is_diff = True  # merge with live scene
        # Detach any stale live attachments so they don't produce ghost contacts
        # with our motion-time attached lego.
        for detach_id in (detach_ids or []):
            det = AttachedCollisionObject()
            det.link_name = "tcp_link"
            det.object.id = detach_id
            det.object.operation = CollisionObject.REMOVE
            req.robot_state.attached_collision_objects.append(det)
        if attached_aco is not None:
            req.robot_state.attached_collision_objects.append(attached_aco)
        req.group_name = "arm"
        fut = self.validity_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        return fut.result()

    def fk(self, joints: dict, link: str = TCP_LINK):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ALL_JOINTS)
        req.robot_state.joint_state.position = [
            float(joints.get(n, 0.0)) for n in ALL_JOINTS
        ]
        fut = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        resp = fut.result()
        if not resp or not resp.pose_stamped:
            return None
        p = resp.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)


def _format_contacts(contacts) -> str:
    if not contacts:
        return "(contacts not reported)"
    out = []
    for c in contacts[:4]:
        a = getattr(c, "contact_body_1", "?")
        b = getattr(c, "contact_body_2", "?")
        depth = getattr(c, "depth", None)
        s = f"{a} ↔ {b}"
        if depth is not None:
            s += f" (depth {abs(depth)*1000:.2f}mm)"
        out.append(s)
    suffix = f" … (+{len(contacts)-4} more)" if len(contacts) > 4 else ""
    return "; ".join(out) + suffix


def _verify_one(csv_path: Path, v: Verifier, every: int = 2, verbose: bool = True,
                fine_interp_ms: int = 0, sweeper=None):
    meta_path = csv_path.with_suffix(".json")
    meta = {}
    if meta_path.exists():
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            pass
    traj_dump_path = meta.get("traj_dump")
    plan = None
    scn = {}
    if traj_dump_path and Path(traj_dump_path).exists():
        try:
            plan = json.loads(Path(traj_dump_path).read_text())
            scn = plan.get("scene_at_plan_time", {}) or {}
        except Exception:
            plan = None

    rows = _read_csv(csv_path)
    if not rows:
        if verbose:
            print(f"{csv_path}: empty")
        return None

    start_wall = float(meta.get("start_wall") or rows[0]["t_wall"])
    gripper_hold = float(scn.get("actual_joints", {}).get(GRIPPER, 0.0))

    attached_name = scn.get("attached_lego_name")
    # MoveIt-dependent sections are gated on v not None (offline degrade
    # path when move_group is down — fine-interp still works)
    moveit_available = v is not None
    aco = (v._attached_lego_aco(attached_name) if attached_name and moveit_available
           else None)

    # ── Apply cup position overrides from the plan's scene_at_plan_time
    # (the live scene often has cup_drop_* meshes at origin because the GUI
    # hasn't pushed drop positions post-restart; plan dump is authoritative.)
    cup_override_status = "skipped (no plan scene)"
    drop_data = scn.get("drop_data", {})
    cups_usable = False
    if moveit_available:
        cups_snapshot = v.snapshot_cup_poses()
        if drop_data and cups_snapshot is not None:
            if v.apply_cup_overrides(drop_data):
                cup_override_status = f"applied {len(drop_data)} cup position(s) from plan dump"
                cups_usable = True
            else:
                cup_override_status = "FAILED (apply_planning_scene rejected)"
        elif cups_snapshot is None:
            cup_override_status = "skipped (couldn't read current scene)"
        else:
            cup_override_status = ("skipped — no plan dump, cups default to origin "
                                   "(validity checks suppressed)")
    else:
        cup_override_status = "skipped (MoveIt offline; fine-interp only)"

    # ── Plan waypoints (sanity — planner validated at plan-time)
    plan_invalid = []
    plan_total = 0
    if plan and cups_usable and moveit_available:
        wps = plan.get("points", [])
        jn = plan.get("joint_names", [])
        for i, wp in enumerate(wps):
            d = dict(zip(jn, wp["positions"]))
            d.setdefault(GRIPPER, gripper_hold)
            resp = v.check_state(d, attached_aco=aco,
                                 detach_ids=v.live_attached_ids)
            plan_total += 1
            if resp is not None and not resp.valid:
                plan_invalid.append((i, wp.get("t", i * 0.1), resp.contacts))

    # ── Isaac trajectory (downsampled). Validity checks only meaningful if
    # cups are at plan-time positions; otherwise we'd report garbage collisions
    # with cups-at-origin. Divergence (TCP) is always computed.
    isaac_invalid = []
    isaac_total = 0
    divergences = []  # (t_rel, mm)
    for idx, r in enumerate(rows):
        if idx % every != 0:
            continue
        t_rel = float(r["t_wall"]) - start_wall
        isaac_joints = {n: _f(r.get(f"isaac_pos_{n}")) for n in ARM}
        isaac_joints[GRIPPER] = gripper_hold
        if any(val is None for val in isaac_joints.values()):
            continue
        if cups_usable and moveit_available:
            resp = v.check_state(isaac_joints, attached_aco=aco,
                                 detach_ids=v.live_attached_ids)
            isaac_total += 1
            if resp is not None and not resp.valid:
                isaac_invalid.append((idx, t_rel, resp.contacts))

        # Cartesian divergence: plan-ref TCP vs Isaac TCP (from CSV).
        # Needs /compute_fk — skipped if MoveIt offline (use _mesh_sweep's
        # local FK instead if divergence matters in offline mode).
        plan_ref_joints = {n: _f(r.get(f"ref_pos_{n}")) for n in ARM}
        plan_ref_joints[GRIPPER] = gripper_hold
        if moveit_available and all(x is not None for x in plan_ref_joints.values()):
            plan_tcp = v.fk(plan_ref_joints)
            isaac_tcp = (_f(r.get("tcp_x")), _f(r.get("tcp_y")), _f(r.get("tcp_z")))
            if plan_tcp and all(x is not None for x in isaac_tcp):
                d_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(plan_tcp, isaac_tcp))) * 1000.0
                divergences.append((t_rel, d_mm))

    # Restore original cup poses regardless of what happened above.
    if moveit_available:
        v.restore_cup_poses()

    # ── Optional: fine-interpolated geometric mesh sweep (bypasses MoveIt,
    # uses URDF thin-mesh + local FK). Closes the 50 Hz sampling gap in the
    # validity replay above — see DEBUG-GUIDE.md § 4.1.
    fine_res = None
    if fine_interp_ms > 0 and sweeper is not None:
        fine_res = sweeper.sweep_motion(rows, plan, resolution_ms=fine_interp_ms)

    summary = {
        "path": str(csv_path),
        "tag": meta.get("motion_tag", "?"),
        "attached_lego": attached_name,
        "cup_override": cup_override_status,
        "plan_total": plan_total,
        "plan_invalid": len(plan_invalid),
        "isaac_total": isaac_total,
        "isaac_invalid": len(isaac_invalid),
        "peak_divergence_mm": max((d for _, d in divergences), default=0.0),
        "mean_divergence_mm": (sum(d for _, d in divergences) / len(divergences))
                              if divergences else 0.0,
        "fine_interp_ms": fine_interp_ms,
        "fine_coarse_inside": len(fine_res["coarse_inside"]) if fine_res else 0,
        "fine_windows": len(fine_res.get("fine_windows", [])) if fine_res else 0,
        "fine_peak_depth_mm": max(
            ((-w[2]["min_dh_minus_r_m"]) * 1000.0
             for w in fine_res.get("fine_windows", [])), default=0.0
        ) if fine_res else 0.0,
        "min_clearance_mm": (fine_res["min_clearance"]["min_dist_m"] * 1000.0
                              if fine_res and fine_res.get("min_clearance")
                              and fine_res["min_clearance"].get("link") else None),
        "within_contact_zone": (fine_res.get("within_contact_zone", False)
                                 if fine_res else False),
    }

    if not verbose:
        return summary

    # ── Verbose report
    print("=" * 90)
    print(f"Verification: {csv_path.name}")
    print(f"  tag: {summary['tag']:<14} attached_lego: {attached_name}")
    print(f"  plan waypoints: {plan_total}   Isaac samples (every {every}): {isaac_total}")
    attach_note = " + inline attached-lego diff" if aco else ""
    print(f"  scene source  : LIVE /move_group planning scene{attach_note}")
    print(f"  cup overrides : {cup_override_status}")
    print()

    print("Plan validity replay (sanity — plan was validated at plan-time):")
    if plan_total == 0:
        print("  (no plan dump — nothing to replay)")
    elif not plan_invalid:
        print(f"  {plan_total}/{plan_total} waypoints valid ✓")
    else:
        print(f"  {plan_total - len(plan_invalid)}/{plan_total} valid, "
              f"{len(plan_invalid)} invalid ⚠")
        print("  → Possible scene drift (cups moved since capture) or scene changed.")
        for i, t, ctc in plan_invalid[:5]:
            print(f"    wp {i:>3} (t={t:.2f}s): {_format_contacts(ctc)}")
    print()

    print("Isaac trajectory validity replay:")
    if isaac_total == 0:
        print("  (no Isaac samples to check)")
    elif not isaac_invalid:
        print(f"  {isaac_total}/{isaac_total} samples valid ✓  "
              f"(Isaac never entered a state MoveIt would reject)")
    else:
        print(f"  {isaac_total - len(isaac_invalid)}/{isaac_total} valid, "
              f"{len(isaac_invalid)} IN COLLISION ⚠⚠")
        print("  → Layer-B lag drove Isaac into states MoveIt would have rejected,")
        print("    even though the plan itself was accepted. This is the mechanism.")
        for i, t, ctc in isaac_invalid[:10]:
            print(f"    sample {i:>4} (t={t:.2f}s): {_format_contacts(ctc)}")
        if len(isaac_invalid) > 10:
            print(f"    … +{len(isaac_invalid) - 10} more")
    print()

    if divergences:
        ds = [d for _, d in divergences]
        peak = max(divergences, key=lambda x: x[1])
        mean = sum(ds) / len(ds)
        rms = math.sqrt(sum(d * d for d in ds) / len(ds))
        print("Plan-vs-Isaac Cartesian divergence (TCP in world frame):")
        print(f"  peak: {peak[1]:>7.2f} mm at t={peak[0]:.2f}s")
        print(f"  mean: {mean:>7.2f} mm")
        print(f"  rms : {rms:>7.2f} mm")
        print()

    # Fine-interp section (URDF thin-mesh geometric sweep)
    if fine_res is not None:
        coarse_n = len(fine_res["coarse_inside"])
        windows = fine_res.get("fine_windows", [])
        mc = fine_res.get("min_clearance", {})
        zone_m = fine_res.get("physx_contact_zone_m", 0.02)
        print(f"Fine-interp URDF mesh sweep (resolution {fine_interp_ms} ms, "
              f"{fine_res['fine_samples']} interpolated instants):")
        print(f"  coarse (50 Hz) thin-mesh intrusions: {coarse_n}")
        if not windows:
            print(f"  fine interpolated intrusions       : none")
        else:
            print(f"  fine interpolated intrusion windows: {len(windows)}")
            coarse_ts = [t for (_idx, t, _w) in fine_res["coarse_inside"]]
            for (t_a, t_b, w) in windows:
                dur_ms = (t_b - t_a) * 1000
                depth = -w["min_dh_minus_r_m"] * 1000
                hidden = not any(t_a <= ct <= t_b for ct in coarse_ts)
                tag = " (HIDDEN from 50 Hz)" if hidden else ""
                print(f"    {t_a:.3f}s → {t_b:.3f}s ({dur_ms:.0f} ms): "
                      f"{w['link']} in {w['cup']}  "
                      f"peak depth {depth:.2f} mm{tag}")
        # PhysX contact-zone analysis — the KEY metric for cup knocks
        if mc and mc.get("link"):
            dist_mm = mc["min_dist_m"] * 1000
            zone_mm = zone_m * 1000
            in_zone = fine_res.get("within_contact_zone", False)
            print(f"  min mesh-surface clearance         : {dist_mm:.2f} mm "
                  f"({mc['link']} ↔ {mc['cup']} at t={mc['t_rel']:.3f}s)")
            print(f"  PhysX default contactOffset        : {zone_mm:.1f} mm")
            if in_zone:
                pct = (zone_mm - dist_mm) / zone_mm * 100
                severity = ("severe" if dist_mm < 5 else
                            "moderate" if dist_mm < 10 else "mild")
                print(f"  → {severity} intrusion into PhysX contact zone "
                      f"({pct:.0f}% depth)")
                print(f"    PhysX will generate contact forces at this distance,")
                print(f"    producing lateral pushes on cups during arm sweeps.")
                print(f"    This is the likely cup-knock mechanism (see DEBUG-GUIDE § 4.3).")
            else:
                print(f"  → mesh never enters PhysX contact zone; cup knocks unlikely.")
        print()

    print("Summary:")
    print(f"  CSV : {csv_path}")
    print(f"  meta: {meta_path if meta_path.exists() else '(none)'}")
    print(f"  plan: {traj_dump_path or '(none)'}")
    return summary


def _scan(root: Path, v: Verifier, every: int,
          fine_interp_ms: int = 0, sweeper=None):
    csvs = sorted(root.rglob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not csvs:
        print(f"No motion CSVs under {root}")
        return 0
    fine_col = f"{'fine⚠':>7}" if fine_interp_ms > 0 else ""
    clr_col = f"{'min_clr':>9}" if fine_interp_ms > 0 else ""
    print(f"{'file':<60} {'tag':<12} {'plan_ok':>8} "
          f"{'isaac_ok':>9} {'peak_div':>10} {fine_col} {clr_col}")
    print("-" * (105 + (len(fine_col) + len(clr_col) + 2 if fine_col else 0)))
    for p in csvs:
        try:
            s = _verify_one(p, v, every=max(every, 4), verbose=False,
                            fine_interp_ms=fine_interp_ms, sweeper=sweeper)
            if s is None:
                continue
            plan_s = f"{s['plan_total']-s['plan_invalid']}/{s['plan_total']}"
            isaac_s = f"{s['isaac_total']-s['isaac_invalid']}/{s['isaac_total']}"
            div = f"{s['peak_divergence_mm']:.1f}mm"
            flag = " ⚠" if s["isaac_invalid"] else ""
            fine_s = ""
            clr_s = ""
            if fine_interp_ms > 0:
                fine_n = s.get("fine_windows", 0)
                fine_s = f" {fine_n:>6}"
                if fine_n:
                    fine_s += f"⚠"
                mc = s.get("min_clearance_mm")
                if mc is not None:
                    clr_s = f" {mc:>7.2f}mm"
                    if s.get("within_contact_zone"):
                        clr_s += "🔥"
            print(f"{p.name:<60} {s['tag']:<12} {plan_s:>8} "
                  f"{isaac_s:>9} {div:>10}{flag}{fine_s}{clr_s}")
        except Exception as e:
            print(f"{p.name:<60} ERROR: {e}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Verify SO-ARM101 motion against MoveIt scene")
    ap.add_argument("csv", nargs="?", help="Motion CSV to verify")
    ap.add_argument("--latest", action="store_true",
                    help="Verify most recent motion under --log-dir")
    ap.add_argument("--scan", action="store_true",
                    help="Verify every motion under --log-dir (slow)")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/motion_logs"),
                    help="Log root (default ~/motion_logs)")
    ap.add_argument("--every", type=int, default=2,
                    help="Check every Nth Isaac sample (default 2 → 25 Hz)")
    ap.add_argument("--fine-interp", type=int, default=0, metavar="MS",
                    help="Additionally run URDF-mesh-vs-cup sweep at MS ms "
                         "interpolation between 50 Hz samples. Closes the "
                         "50 Hz sampling gap (see DEBUG-GUIDE § 4.1). "
                         "Default 0 = disabled.")
    args = ap.parse_args()

    if not ROS2_OK:
        print(f"FATAL: rclpy / moveit_msgs not importable ({_IMPORT_ERR}).", file=sys.stderr)
        print("       Did you source ROS2? scripts/motion_log.sh verify handles this.",
              file=sys.stderr)
        return 2

    root = Path(args.log_dir)

    # Resolve target(s)
    targets = []
    if args.scan:
        targets = None  # handled specially
    elif args.csv:
        p = Path(args.csv)
        if not p.exists():
            print(f"Not found: {p}", file=sys.stderr); return 1
        targets = [p]
    elif args.latest:
        csvs = sorted(root.rglob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        if not csvs:
            print(f"No CSVs under {root}", file=sys.stderr); return 1
        targets = [csvs[0]]
    else:
        ap.print_help(); return 1

    rclpy.init()
    node = rclpy.create_node("motion_verifier")
    v = Verifier(node)
    moveit_ready = v.wait_services(timeout=10.0)
    # Graceful degrade: if MoveIt is down but --fine-interp is requested,
    # we can still do the offline geometric sweep. If neither works, fail.
    if not moveit_ready:
        if args.fine_interp > 0:
            print("NOTE: /check_state_validity or /compute_fk not ready — "
                  "MoveIt-side checks skipped. Running fine-interp sweep "
                  "in offline-only mode.", file=sys.stderr)
            v = None  # signal to _verify_one that MoveIt is unavailable
        else:
            print("FATAL: /check_state_validity or /compute_fk not ready.",
                  file=sys.stderr)
            print("       Is the control stack running?  Check: "
                  "ros2 node list | grep move_group", file=sys.stderr)
            print("       (Tip: add --fine-interp MS to run geometric "
                  "sweep without MoveIt.)", file=sys.stderr)
            node.destroy_node(); rclpy.shutdown()
            return 2

    # Spin up the URDF mesh sweeper once for --fine-interp (reused across
    # all motions in --scan mode).
    sweeper = None
    if args.fine_interp > 0:
        if not MESH_SWEEP_OK:
            print(f"FATAL: --fine-interp requested but _mesh_sweep unavailable "
                  f"({_MESH_SWEEP_ERR}).", file=sys.stderr)
            node.destroy_node(); rclpy.shutdown()
            return 2
        sweeper = default_sweeper()

    try:
        if targets is None:
            rc = _scan(root, v, every=args.every,
                       fine_interp_ms=args.fine_interp, sweeper=sweeper)
        else:
            for p in targets:
                _verify_one(p, v, every=args.every, verbose=True,
                            fine_interp_ms=args.fine_interp, sweeper=sweeper)
            rc = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main() or 0)
