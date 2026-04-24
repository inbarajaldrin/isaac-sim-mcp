#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
Fast local FK + URDF collision-mesh sweep, shared by motion_verify.py (as an
optional fine-interpolation pass) and motion_sweep.py (characterization
driver).

Why this exists: `motion_verify.py`'s 50 Hz validity replay misses
sub-sample geometric penetrations — a real failure mode documented in
DEBUG-GUIDE.md § 4.1's "sampling gap" note. Closing that gap requires
FK at ~1-5 ms resolution, which is ~1000× too many calls for
`/compute_fk`. This module reads the SO-ARM101 URDF, builds a PyKDL
chain, and does FK locally so we can sweep thousands of interpolated
joint configurations per motion in seconds.

Public API:
    MeshSweeper(urdf_path).sweep(csv_rows, plan_dump, resolution_ms=2) -> dict
        Returns: {
          'coarse_inside': [(sample_idx, link, min_dh_minus_r_mm)],
          'fine_inside':   [(t_rel_s, link, min_dh_minus_r_mm, z_at_mm)],
          'interp_samples_run': int,
        }

    MeshSweeper.check_joints(joints, cup_positions) -> (inside, worst)
        One-shot query: is any mesh vertex inside any cup?
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import trimesh
import PyKDL
from urdf_parser_py.urdf import URDF


CUP_RADIUS_M = 0.039
CUP_HEIGHT_M = 0.0965

# PhysX default contactOffset for rigid bodies (m). When no explicit
# contactOffset is authored on a collision shape — which is the case for the
# SO-ARM101's gripper/jaw/wrist collisions (verified via USD introspection,
# see DEBUG-GUIDE § 4.3) — PhysX falls back to this. Any two shapes closer
# than this distance generate contact forces, even without geometric overlap.
# Hypothesis: the cup knocks are driven by gripper passing within this
# distance of the cup surface, not by STL intersection.
PHYSX_DEFAULT_CONTACT_OFFSET_M = 0.02
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll"]
GRIPPER_JOINT = "gripper_joint"

# URDF collision meshes per link (relative to link frame). Matches the live
# so_arm101.urdf — if that URDF changes, regenerate this table.
LINK_COLLISION_MESHES = {
    "gripper": [
        ("sts3215_03a_v1.stl",
         (0.0077, 0.0001, -0.0234), (-1.5708, 0, 0)),
        ("wrist_roll_follower_so101_v1.stl",
         (0.0, -0.000218, 0.000950), (-3.14159, 0, 0)),
    ],
    "jaw": [
        ("moving_jaw_so101_v1.stl",
         (0.0, 0.0, 0.0189), (0.0, 0.0, 0.0)),
    ],
    "wrist": [
        ("sts3215_03a_no_horn_v1.stl",
         (5.55e-17, -0.0424, 0.0306), (1.5708, 1.5708, 0)),
        ("wrist_roll_pitch_so101_v2.stl",
         (0.0, -0.028, 0.0181), (-1.5708, -1.5708, 0)),
    ],
}

# Cup in scene goes by drop_0/1/2 → we pass positions in via `cup_positions`.
# Each motion's plan dump has these under scene_at_plan_time.drop_data.


def _rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class _LinkFK:
    """Minimal per-link FK helper built from URDF joint list. We avoid
    PyKDL's tree builder (which needs the full xacro-resolved URDF) and
    build the chain manually from the flat urdf_parser_py output."""

    def __init__(self, urdf: URDF, target_links: list):
        self.urdf = urdf
        # Map joint name → (origin_xyz, origin_rpy, axis, type, parent, child)
        self.joints = {}
        for j in urdf.joints:
            origin_xyz = list(j.origin.xyz) if j.origin else [0, 0, 0]
            origin_rpy = list(j.origin.rpy) if j.origin else [0, 0, 0]
            axis = list(j.axis) if j.axis else [0, 0, 1]
            self.joints[j.name] = (origin_xyz, origin_rpy, axis, j.type,
                                   j.parent, j.child)
        # Map child_link → joint (each link has at most one parent joint)
        self.link_parent_joint = {}
        for jname, (_, _, _, _, _, child) in self.joints.items():
            self.link_parent_joint[child] = jname
        # Cache the joint chain from base to each target link
        self.chains = {}
        for link in target_links:
            self.chains[link] = self._chain_to(link)

    def _chain_to(self, link: str):
        chain = []
        cur = link
        while cur in self.link_parent_joint:
            jname = self.link_parent_joint[cur]
            chain.append(jname)
            cur = self.joints[jname][4]  # parent link
        chain.reverse()
        return chain

    def link_transform(self, link: str, joint_values: dict) -> np.ndarray:
        """4x4 homogeneous transform of `link` in base frame."""
        T = np.eye(4)
        for jname in self.chains[link]:
            origin_xyz, origin_rpy, axis, jtype, _, _ = self.joints[jname]
            T_origin = np.eye(4)
            T_origin[:3, :3] = _rpy_to_mat(origin_rpy)
            T_origin[:3, 3] = origin_xyz
            T_joint = np.eye(4)
            if jtype in ("revolute", "continuous"):
                theta = float(joint_values.get(jname, 0.0))
                ax = np.array(axis, dtype=float)
                ax = ax / np.linalg.norm(ax)
                # Rodrigues' rotation
                c, s = math.cos(theta), math.sin(theta)
                K = np.array([[0, -ax[2], ax[1]],
                              [ax[2], 0, -ax[0]],
                              [-ax[1], ax[0], 0]])
                R = np.eye(3) + s * K + (1 - c) * (K @ K)
                T_joint[:3, :3] = R
            elif jtype == "prismatic":
                d = float(joint_values.get(jname, 0.0))
                ax = np.array(axis, dtype=float)
                ax = ax / np.linalg.norm(ax)
                T_joint[:3, 3] = d * ax
            # fixed: T_joint = I
            T = T @ T_origin @ T_joint
        return T


class MeshSweeper:
    """Carries the URDF, pre-transformed collision meshes, and FK state."""

    def __init__(self, urdf_path: str, mesh_root: str, cup_mesh_path: str = None):
        # urdf_parser_py's from_xml_file trips on XML-declared UTF-8 headers
        # via lxml; feed bytes instead.
        with open(urdf_path, "rb") as f:
            self.urdf = URDF.from_xml_string(f.read())
        self.fk = _LinkFK(self.urdf, list(LINK_COLLISION_MESHES.keys()))
        self.link_verts = self._load_meshes(mesh_root)
        # Optional: load actual cup STL for proper mesh-surface distance
        # queries (handles hollow geometry correctly — a point inside the cup
        # cavity is measured against the inner wall, not "inside a solid
        # cylinder"). trimesh.proximity needs rtree; if missing, fall back
        # to the cylinder approximation.
        self._cup_mesh = None
        if cup_mesh_path and os.path.exists(cup_mesh_path):
            try:
                m = trimesh.load_mesh(cup_mesh_path)
                v = np.asarray(m.vertices)
                if v.ptp(axis=0).max() > 2.0:
                    m.vertices = v * 0.001
                # Touch triangles_tree now to ensure rtree is present
                _ = m.triangles_tree
                self._cup_mesh = m
            except Exception:
                self._cup_mesh = None

    def _load_meshes(self, mesh_root: str):
        link_verts = {}
        for link, cols in LINK_COLLISION_MESHES.items():
            acc = []
            for stl_name, xyz, rpy in cols:
                path = os.path.join(mesh_root, stl_name)
                m = trimesh.load_mesh(path)
                v = np.asarray(m.vertices)
                if v.ptp(axis=0).max() > 2.0:
                    v = v * 0.001  # STL in mm, convert to m
                T_collision = np.eye(4)
                T_collision[:3, :3] = _rpy_to_mat(rpy)
                T_collision[:3, 3] = xyz
                vl = (T_collision @ np.c_[v, np.ones(len(v))].T).T[:, :3]
                acc.append(vl)
            link_verts[link] = np.vstack(acc)
        return link_verts

    def min_clearance_to_cups(self, joints: dict, cup_positions: dict,
                              vertex_stride: int = 1):
        """Return the minimum clearance from any gripper/jaw/wrist mesh
        vertex to any cup's collision surface. Uses an analytical solid-
        cylinder approximation of the cup (matches PhysX's convex
        decomposition better than a hollow-mesh proximity query would,
        since a convex decomp is a union of convex outer shells).

        Fast path: fully vectorized numpy, no per-vertex queries.

        Returns dict: {min_dist_m, link, cup, vertex_world_xyz_m}.
        min_dist_m ≥ 0 means clearance (no geometric overlap).
        """
        best = {"min_dist_m": float("inf"), "link": None,
                "cup": None, "vertex_world_xyz_m": None}
        for link, V in self.link_verts.items():
            Tw = self.fk.link_transform(link, joints)
            Vw = (Tw @ np.c_[V, np.ones(len(V))].T).T[:, :3]
            Vd = Vw[::vertex_stride] if vertex_stride > 1 else Vw
            for cup_name, (cx, cy, cz) in cup_positions.items():
                vx, vy, vz = Vd[:, 0], Vd[:, 1], Vd[:, 2]
                dh = np.sqrt((vx - cx) ** 2 + (vy - cy) ** 2)
                # Distance from point to a solid cylinder surface centered
                # at (cx,cy), radius R, from z=cz to z=cz+H:
                #   - if point.z in [cz, cz+H]: distance = |dh - R|
                #     (negative sign = inside; we take abs for clearance)
                #   - else (above/below): dist = sqrt(max(0,dh-R)^2 + dz^2)
                inside_z = (vz >= cz) & (vz <= cz + CUP_HEIGHT_M)
                edge_z = np.where(vz > cz + CUP_HEIGHT_M, vz - (cz + CUP_HEIGHT_M),
                                  np.where(vz < cz, cz - vz, 0.0))
                dh_clip = np.maximum(dh - CUP_RADIUS_M, 0)
                dist = np.where(inside_z, np.abs(dh - CUP_RADIUS_M),
                                np.sqrt(dh_clip ** 2 + edge_z ** 2))
                i = int(np.argmin(np.abs(dist)))
                d = float(dist[i])
                if d < best["min_dist_m"]:
                    best["min_dist_m"] = d
                    best["link"] = link
                    best["cup"] = cup_name
                    best["vertex_world_xyz_m"] = tuple(float(x) for x in Vd[i])
        return best

    def check_joints(self, joints: dict, cup_positions: dict):
        """For one joint config, return (any_inside, per_link_worst_margin).
        cup_positions: {cup_name: (x, y, z)} — typically world positions.
        Returns (inside: bool, worst: dict | None).
        worst = {link, cup, min_dh_minus_r_m, z_at_m} for the most intruding
        vertex (only when inside=True).
        """
        any_inside = False
        worst = None
        worst_margin = 0.0  # most negative when inside
        for link, V in self.link_verts.items():
            Tw = self.fk.link_transform(link, joints)
            Vw = (Tw @ np.c_[V, np.ones(len(V))].T).T[:, :3]
            for cup_name, (cx, cy, _cz) in cup_positions.items():
                z = Vw[:, 2]
                dh = np.sqrt((Vw[:, 0] - cx) ** 2 + (Vw[:, 1] - cy) ** 2)
                below_rim = z < CUP_HEIGHT_M
                if not below_rim.any():
                    continue
                # Margin = dh - r (negative = inside)
                dh_br = dh[below_rim]
                z_br = z[below_rim]
                i_min = int(np.argmin(dh_br))
                margin = dh_br[i_min] - CUP_RADIUS_M
                if margin < 0:
                    any_inside = True
                    if margin < worst_margin:
                        worst_margin = margin
                        worst = {
                            "link": link,
                            "cup": cup_name,
                            "min_dh_minus_r_m": float(margin),
                            "z_at_m": float(z_br[i_min]),
                        }
        return any_inside, worst

    def sweep_motion(self, csv_rows, plan_dump, resolution_ms: int = 2,
                     include_gripper_from_scene: bool = True):
        """Run both coarse (per-sample) + fine (inter-sample) checks.
        Returns a dict with per-sample findings + aggregate counts.
        csv_rows: list of dict (already csv.DictReader'd)
        plan_dump: dict (loaded JSON from /tmp/arm_traj/*.json), or None
        resolution_ms: fine interpolation step between adjacent samples.
        """
        scn = (plan_dump or {}).get("scene_at_plan_time", {}) or {}
        gripper_hold = float(scn.get("actual_joints", {}).get(GRIPPER_JOINT, 0.0))
        # Use CSV row 0 cup positions — authoritative live Isaac pose at motion
        # start. Plan dump's drop_data.z = drop pose height (~half cup height,
        # above the rim), NOT cup base z; don't confuse the two.
        def _cup_from_csv(row, color):
            try:
                return (float(row[f"cup_{color}_x"]),
                        float(row[f"cup_{color}_y"]),
                        0.0)  # cups sit on ground; CSV z is noisy float-near-0
            except (KeyError, ValueError):
                return None
        row0 = csv_rows[0]
        color_to_pos = {}
        for color in ("red", "green", "blue"):
            p = _cup_from_csv(row0, color)
            if p is not None:
                color_to_pos[color] = p
        # Map cup_drop_N → color by x-coordinate (plan dump's drop_data has
        # the authoritative N↔color mapping via x).
        drop_data = scn.get("drop_data", {}) or {}
        cup_positions = {}
        for drop_key, d in drop_data.items():
            suffix = drop_key.split("drop_")[-1]
            # Match drop.x to one of the CSV cup x's (within 1cm tolerance)
            dx = float(d.get("x", 0.0))
            best_color = None
            best_err = 1e9
            for color, (cx, cy, cz) in color_to_pos.items():
                e = abs(cx - dx)
                if e < best_err:
                    best_err = e
                    best_color = color
            if best_color is not None and best_err < 0.02:
                cup_positions[f"cup_drop_{suffix}"] = color_to_pos[best_color]
        # Fall back: if no plan dump, just use all three CSV cups
        if not cup_positions:
            cup_positions = {f"cup_drop_{color}": pos
                             for color, pos in color_to_pos.items()}
        if not cup_positions:
            return {"coarse_inside": [], "fine_inside": [],
                    "coarse_samples": 0, "fine_samples": 0,
                    "reason": "no cup positions in CSV or plan dump"}

        def joints_at(row):
            j = {n: float(row[f"isaac_pos_{n}"]) for n in ARM_JOINTS}
            if include_gripper_from_scene:
                j[GRIPPER_JOINT] = gripper_hold
            return j

        t0 = float(csv_rows[0]["t_wall"])

        # Track MIN clearance across coarse + fine passes — this is the key
        # PhysX-contact-zone metric. PhysX default contactOffset = 20 mm:
        # any frame where the nearest vertex is within that distance of a
        # cup surface will generate contact forces in sim, even though no
        # STL geometry overlaps. See DEBUG-GUIDE § 4.3.
        # Clearance is computed at coarse samples only (50 Hz) — peak approach
        # events span ≥40 ms so 50 Hz is sufficient, and trimesh.proximity
        # queries are expensive so we don't run them at the fine resolution.
        global_min_clearance = {"min_dist_m": float("inf"), "t_rel": None,
                                "link": None, "cup": None}

        def _update_clearance(t_rel, joints):
            # All vertices (stride=1) — the analytical formula is fast enough.
            mc = self.min_clearance_to_cups(joints, cup_positions, vertex_stride=1)
            if mc["min_dist_m"] < global_min_clearance["min_dist_m"]:
                global_min_clearance.update(mc)
                global_min_clearance["t_rel"] = t_rel

        # Coarse pass — one check per Isaac sample
        coarse_inside = []
        for idx, row in enumerate(csv_rows):
            j = joints_at(row)
            inside, worst = self.check_joints(j, cup_positions)
            if inside:
                t_rel = float(row["t_wall"]) - t0
                coarse_inside.append((idx, t_rel, worst))
            _update_clearance(float(row["t_wall"]) - t0, j)

        # Fine pass — linear interp between adjacent Isaac samples
        fine_inside = []
        fine_samples = 0
        for idx in range(len(csv_rows) - 1):
            ra, rb = csv_rows[idx], csv_rows[idx + 1]
            ta = float(ra["t_wall"]); tb = float(rb["t_wall"])
            dt = tb - ta
            if dt <= 0:
                continue
            steps = max(2, int(dt * 1000.0 / resolution_ms))
            ja = joints_at(ra); jb = joints_at(rb)
            for s in range(steps + 1):
                # Skip the endpoints — they're already in the coarse pass
                if s == 0 or s == steps:
                    continue
                frac = s / steps
                j = {k: ja[k] + frac * (jb[k] - ja[k]) for k in ja}
                fine_samples += 1
                inside, worst = self.check_joints(j, cup_positions)
                t_rel = (ta + frac * dt) - t0
                if inside:
                    fine_inside.append((t_rel, worst))
                # Skip fine-step clearance tracking: coarse samples already
                # cover 50 Hz and that's enough granularity for peak approach
                # detection. The proximity query is too expensive to run at
                # the fine resolution (~2 ms steps).

        # Dedup fine_inside events into contiguous windows (adjacent inside=True
        # forms one "event" — more useful than every interpolated step).
        windows = []
        if fine_inside:
            cur_start = fine_inside[0][0]
            cur_worst = fine_inside[0][1]
            prev_t = fine_inside[0][0]
            prev_worst_margin = cur_worst["min_dh_minus_r_m"]
            for t_rel, w in fine_inside[1:]:
                # If more than ~3× resolution since last, close window
                if (t_rel - prev_t) * 1000 > resolution_ms * 3:
                    windows.append((cur_start, prev_t, cur_worst))
                    cur_start = t_rel
                    cur_worst = w
                    prev_worst_margin = w["min_dh_minus_r_m"]
                elif w["min_dh_minus_r_m"] < prev_worst_margin:
                    cur_worst = w
                    prev_worst_margin = w["min_dh_minus_r_m"]
                prev_t = t_rel
            windows.append((cur_start, prev_t, cur_worst))

        return {
            "coarse_inside": coarse_inside,
            "fine_inside": fine_inside,
            "fine_windows": windows,
            "coarse_samples": len(csv_rows),
            "fine_samples": fine_samples,
            "cup_positions": cup_positions,
            "resolution_ms": resolution_ms,
            "min_clearance": global_min_clearance,
            "physx_contact_zone_m": PHYSX_DEFAULT_CONTACT_OFFSET_M,
            "within_contact_zone": (
                global_min_clearance["min_dist_m"]
                < PHYSX_DEFAULT_CONTACT_OFFSET_M
            ),
        }


# ─── Convenience: default path resolution ─────────────────────────────

DEFAULT_URDF = "/home/aaugus11/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_description/urdf/so_arm101.urdf"
DEFAULT_MESH_ROOT = "/home/aaugus11/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_description/meshes"
# Try both install/ and src/ locations — colcon copies the STL into install/
DEFAULT_CUP_MESH = "/home/aaugus11/Projects/Exploring-VLAs/vla_SO-ARM101/install/so_arm101_description/share/so_arm101_description/meshes/cup/cup.stl"


def default_sweeper() -> MeshSweeper:
    cup_path = DEFAULT_CUP_MESH
    if not os.path.exists(cup_path):
        cup_path = os.path.join(DEFAULT_MESH_ROOT, "cup", "cup.stl")
        if not os.path.exists(cup_path):
            cup_path = None
    return MeshSweeper(DEFAULT_URDF, DEFAULT_MESH_ROOT, cup_mesh_path=cup_path)
