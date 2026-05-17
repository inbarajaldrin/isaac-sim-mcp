#!/usr/bin/env python3
# Reference: built per "two-end cable parity" investigation 2026-05-16.
# Mirrors the build_mount_rail_usds.py pattern (vendored-from-AIC + thin USD).
# Pairs with the Gazebo source at:
#   ~/Documents/aic/aic_description/urdf/cable.sdf.xacro
# which uses a literal xacro:if on `cable_type` to swap which connector goes
# at which cable end, then loads a different SDF model entirely
# (model://sfp_sc_cable vs model://sfp_sc_cable_reversed). Isaac Sim cannot
# import SDF natively (asset.importer.urdf + asset.importer.mjcf exist; no
# asset.importer.sdf), so the project's original
# `aic_unified_robot_cable_sdf.usd` was provided pre-built (per commit
# 880a3a4 import message — "from previously separate clone at
# ~/Documents/cli/isaac-sim-mcp-main"). We have no SDF→USD pipeline checked
# in.
#
# Rather than build a full SDF→USD converter (would require either
# xacro-flatten + SDF-to-URDF translation OR direct pxr authoring from
# parsed SDF), this script takes the existing single-orientation cable
# USD and produces the OPPOSITE-orientation variant via a surgical
# transform swap on the two connector visuals. The cable rope chain is
# unchanged; only the visual connector positions are mirrored.
#
# Why this is safe (in our scene topology):
#   - The cable rope (Rope/link_0..link_20) has mass=0 per AIC's M1
#     descope (cable is decorative stage-dressing; scoring is contact-
#     based on plug-end geometry via plug_proxy under the gripper finger).
#   - extension.py::_cmd_attach_cable_to_gripper joins Rope/link_20 to
#     the gripper finger regardless of which connector visual is there.
#   - extension.py::_compute_trial_tf_frames publishes the policy-facing
#     "plug at gripper" TF anchored on /gripper_hande_finger_link_l/plug_proxy
#     (a synthetic Xform child of the finger). Swapping visual subtrees
#     does NOT move the proxy — policy still commands "gripper to port",
#     correctness preserved.
#   - The two connector Xforms (sfp_module_visual containing the LC plug,
#     and sc_plug_visual) are direct siblings under /aic_unified_robot/cable
#     and share the same parent frame; a direct local-xform swap is a
#     valid mirror through the cable midplane.
#
# WHEN this approach BREAKS DOWN (so a future contributor can plan around):
#   - If the rope is upgraded to a real physical articulation (M2 cable-
#     physics) where link_20 position couples back through the chain to
#     the plug visual's pose, then a transform swap of the leaf visuals
#     won't compose correctly with rope dynamics.
#   - If the connector visuals later carry their own rigid-body / joint
#     APIs, swapping their xform won't move the physics body — that lives
#     in the link's RigidBodyAPI. This script intentionally swaps ONLY
#     UsdGeom.Xformable xformOp:* attributes; collision/physics is not
#     touched.
#
# CONTRIBUTION POTENTIAL:
#   The reusable core (swap_local_xform / mirror_subtree_pair) is a
#   primitive operation that any asymmetric-asset variant pipeline can
#   use. If we evolve aic-dt into a proper contribution to Isaac Sim
#   (e.g. an "asset variant utilities" extension), this is the kernel.
#   The build-script wrapper is project-specific; the swap function is
#   not.
#
# Usage:
#   ~/env_isaaclab/bin/python exts/aic-dt/scripts/build_cable_variant_usds.py \
#       --source exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd \
#       --dest   exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf_reversed.usd \
#       --variant sfp_sc_cable_reversed
#
#   # or no-arg: produces the reversed variant alongside the existing USD
#   ~/env_isaaclab/bin/python exts/aic-dt/scripts/build_cable_variant_usds.py
#
# Verify before committing: open the produced USD in Isaac Sim, point a
# wrist camera at the gripper, confirm the OPPOSITE connector is now
# visually at the held end.

"""Author a reversed-orientation cable USD by swapping connector visuals."""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
from typing import Tuple


# Paths inside the unified cable USD where connector visuals live.
# Both are Xform prims directly under /World/cable (siblings, same parent
# frame → local-xform swap is valid). Verified 2026-05-16 by probing the
# source USD on disk:
#   - /World/cable/sfp_module_visual (contains lc_plug_visual as child)
#     — currently at the gripper end of the cable
#   - /World/cable/sc_plug_visual
#     — currently at the far / non-gripper end
# At LIVE stage time, extension.py::load_robot calls
# `add_reference_to_stage(usd_path=…, prim_path="/World/UR5e")` which
# composes the USD's /World subtree under /World/UR5e, so the same
# prims appear at /World/UR5e/cable/* in a running Kit session. These
# constants are the SOURCE-USD-ON-DISK paths because this script edits
# the USD file directly, not the live stage.
CONNECTOR_A_PATH = "/World/cable/sfp_module_visual"
CONNECTOR_B_PATH = "/World/cable/sc_plug_visual"

DEFAULT_SOURCE_USD = os.path.expanduser(
    "~/Documents/isaac-sim-mcp/exts/aic-dt/assets/robot/"
    "aic_unified_robot_cable_sdf.usd"
)
DEFAULT_DEST_USD = os.path.expanduser(
    "~/Documents/isaac-sim-mcp/exts/aic-dt/assets/robot/"
    "aic_unified_robot_cable_sdf_reversed.usd"
)


# --------------------------------------------------------------------------
# Reusable kernel — these two functions are the part that could become a
# generic Isaac Sim "asset variant utilities" contribution. They depend
# only on pxr (no project-specific imports). Keep them pure.
# --------------------------------------------------------------------------

def read_local_xform_ops(prim) -> list:
    """Return the ordered list of local xform ops on a prim as
    (op_name, op_type, value) tuples. Preserves precision attribute.

    Result is consumable by `write_local_xform_ops` to copy the full
    local transform from one prim to another, lossless across translate
    / orient / scale / transform / arbitrary custom ops.
    """
    from pxr import UsdGeom
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    out = []
    for op in ops:
        out.append({
            "name": op.GetName(),
            "type": op.GetOpType(),
            "precision": op.GetPrecision(),
            "value": op.Get(),
        })
    return out


def write_local_xform_ops(prim, ops: list) -> None:
    """Overwrite a prim's local xform ops with the given list.

    Clears the prim's existing xformOpOrder, then re-adds each op from the
    given list and sets its value. Idempotent — calling twice produces
    the same result. Pairs with `read_local_xform_ops` for swap-style
    operations.
    """
    from pxr import UsdGeom
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    # Remove any leftover xformOp:* attributes from the previous ordering
    # so we don't carry stale data.
    for attr in list(prim.GetAttributes()):
        nm = attr.GetName()
        if nm.startswith("xformOp:"):
            prim.RemoveProperty(nm)
    # Re-add ops in their original order with their original values.
    for op_spec in ops:
        op = xformable.AddXformOp(op_spec["type"], op_spec["precision"])
        if op_spec["value"] is not None:
            op.Set(op_spec["value"])


def swap_local_xforms(stage, prim_a_path: str, prim_b_path: str,
                       ops_to_swap: Tuple[str, ...] = ("xformOp:translate",)) -> Tuple[str, str]:
    """Swap selected local xform op values between two prims.

    Both prims must exist and be UsdGeom.Xformable. Only the named ops are
    swapped — others are left alone on each prim. Returns the two paths
    on success. Raises ValueError if either prim is missing or not
    xformable, or if any named op is missing on either prim.

    `ops_to_swap` default is just ('xformOp:translate',) — translation
    only. This is intentional after empirical 2026-05-16 testing:
    swapping the full op stack (translate + orient + scale) on the AIC
    cable USD's two connector visuals caused PhysX BroadPhaseUpdateData
    errors that crashed Isaac Sim. The two connectors were authored with
    DIFFERENT scale conventions (sfp_module_visual scale=1 vs
    sc_plug_visual scale=0.01 — different source-mesh unit systems), so
    swapping the full stack put scale=0.01 onto a meter-authored mesh
    and produced near-degenerate AABBs that PhysX rejected.
    Translation-only swap leaves each prim's orientation + scale alone,
    so the mesh stays its intended size and is just placed at the other
    end of the cable. The visual orientation at the new end may be
    slightly off (because Gazebo's two SDFs author DIFFERENT orient
    values for each connector at each end — see
    aic_assets/models/sfp_sc_cable[_reversed]/model.sdf for the
    canonical reversed orients), but the swap is safe and reversible.

    To get pixel-perfect orient parity you'd also need to compose a
    per-prim-per-end orient correction, which means tracking what each
    end's expected orient is per the source SDF — a per-asset table.
    Future work; not blocking for visual disambiguation.

    DEEPER LIMITATION DISCOVERED 2026-05-16 (live test against trial_3):
    A translate swap alone is INSUFFICIENT to produce a stable reversed-
    end visual on this specific asset. The connector visuals
    (sfp_module_visual + sc_plug_visual) are authored as FREE-FLOATING
    rigid bodies (PhysicsRigidBodyAPI + mass=0 + diagonalInertia=0 +
    centerOfMass=-inf) with NO joints constraining them to the rope or
    to the gripper. The "plug-at-gripper" scoring/TF semantic is
    provided by a separate `plug_proxy` Xform (a 10mm sphere collider
    child of the gripper finger link, authored by
    `_attach_cable_to_gripper_impl` in extension.py:1955+). The cable
    rope and connector visuals are pure stage-dressing.
    Result: my translate swap sets the INITIAL spawn position, but
    PhysX drifts the free rigid bodies after that. Live position 50s
    after spawn does not match the authored swap. To produce a stable
    reversed visual, you'd ALSO need:
      (a) Author kinematicEnabled=True on both connector rigid bodies,
          OR
      (b) Author FixedJoints constraining each connector to its rope
          end (which is what Gazebo's CablePlugin does), OR
      (c) Reparent the connector visuals under their respective rope
          link Xforms so USD hierarchy gives them the rope's pose.
    Any of these means more than a one-line swap; see notes in the
    main build_cable_variant() docstring for the rebuild-vs-surgery
    decision.
    """
    a = stage.GetPrimAtPath(prim_a_path)
    b = stage.GetPrimAtPath(prim_b_path)
    if not a.IsValid():
        raise ValueError(f"Prim A not found: {prim_a_path}")
    if not b.IsValid():
        raise ValueError(f"Prim B not found: {prim_b_path}")

    from pxr import UsdGeom
    if not UsdGeom.Xformable(a):
        raise ValueError(f"Prim A is not Xformable: {prim_a_path}")
    if not UsdGeom.Xformable(b):
        raise ValueError(f"Prim B is not Xformable: {prim_b_path}")

    for op_name in ops_to_swap:
        attr_a = a.GetAttribute(op_name)
        attr_b = b.GetAttribute(op_name)
        if not attr_a.IsValid():
            raise ValueError(f"Op {op_name!r} missing on {prim_a_path}")
        if not attr_b.IsValid():
            raise ValueError(f"Op {op_name!r} missing on {prim_b_path}")
        val_a = attr_a.Get()
        val_b = attr_b.Get()
        attr_a.Set(val_b)
        attr_b.Set(val_a)
        print(f"  swapped {op_name}: {val_a} ↔ {val_b}")
    return (prim_a_path, prim_b_path)


def read_authored_trs(prim) -> dict:
    """Return the authored translate/orient/scale attrs for a connector root."""
    trs = {}
    for attr_name in ("xformOp:translate", "xformOp:orient", "xformOp:scale"):
        attr = prim.GetAttribute(attr_name)
        if not attr.IsValid():
            raise ValueError(f"Required attr {attr_name!r} missing on {prim.GetPath()}")
        value = attr.Get()
        if value is None:
            raise ValueError(f"Required attr {attr_name!r} unauthored on {prim.GetPath()}")
        trs[attr_name] = value
    return trs


def yaw_quat_radians(yaw_radians: float):
    """Build a quaternion representing a yaw rotation about +Z."""
    from pxr import Gf
    half_yaw = yaw_radians * 0.5
    return Gf.Quatd(math.cos(half_yaw), Gf.Vec3d(0.0, 0.0, math.sin(half_yaw)))


def cast_quat_like(template_quat, quat_value):
    """Cast a quaternion to the same precision/class as an authored orient value."""
    imag = quat_value.GetImaginary()
    return template_quat.__class__(
        quat_value.GetReal(),
        imag[0],
        imag[1],
        imag[2],
    )


def compose_orient_with_yaw(base_orient, yaw_radians: float):
    """Compose an authored orient with a yaw correction in matching precision."""
    yaw_correction = cast_quat_like(base_orient, yaw_quat_radians(yaw_radians))
    return base_orient * yaw_correction


def remove_orphan_finger_r_joint(stage) -> bool:
    """Remove the upstream-shipped orphan FixedJoint that pulls a cable connector
    to the gripper finger_link_r articulation member.

    Path: /World/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint
    body0: a cable connector visual (sfp_module_visual originally;
           swap_joint_endpoints_for_reversed_cable rebinds to sc_plug_visual in
           the reversed variant)
    body1: gripper_hande_finger_link_r (UR5e articulation member)

    Why removed:
      - The joint's localPos0 was authored for the SFP+LC body geometry (large
        protruding module). When the joint locks the connector at the gripper
        finger origin, a SFP-sized body extends out of the housing and is visible;
        a small SC plug (16mm thick) gets fully occluded INSIDE the housing.
      - The extension's _attach_cable_to_gripper_impl runs a physics-step tracker
        that positions the held connector at gripper_tcp (visible at the gripper
        TIP, matching Gazebo CablePlugin behavior). The orphan FixedJoint
        actively fights that tracker and wins (joint-cooked constraints can't
        be removed by USD RemovePrim alone after PhysX has cooked them).
      - Eliminating the joint at USD authoring time means PhysX never cooks it,
        leaving the field clear for the runtime tracker to govern position.

    Returns True if removed, False if already absent.
    """
    orphan_path = "/World/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint"
    prim = stage.GetPrimAtPath(orphan_path)
    if prim and prim.IsValid():
        stage.RemovePrim(orphan_path)
        print(f"[remove_orphan_finger_r_joint] removed {orphan_path}")
        return True
    print(f"[remove_orphan_finger_r_joint] not present at {orphan_path} — skipping")
    return False


def swap_joint_endpoints_for_reversed_cable(stage,
                                              connector_a_path: str,
                                              connector_b_path: str) -> dict:
    """Swap the cable's joint endpoints so the OTHER connector ends up at the gripper.

    The reversed-cable variant is NOT a transform swap (the connector visual
    transforms are irrelevant because PhysX joint constraints dominate
    position). It's a JOINT ENDPOINT swap: change which connector visual is
    bound to which rope-end + which is bound to the gripper-finger orphan
    joint.

    Joints touched (paths fixed in the unified-robot cable USD):
      /World/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint
          body0: connector_a → connector_b   (gripper anchor)
      /World/cable/Rope/fixedJoint
          body1: connector_a → connector_b   (rope link_0 end)
      /World/cable/Rope/fixedJoint2
          body1: connector_b → connector_a   (rope link_20 end)

    After the swap, in the reversed USD:
      - connector_b (sc_plug_visual) is pulled to gripper_finger_r AND to
        rope link_0 (the gripper-end of the rope) — SC plug visually at
        gripper.
      - connector_a (sfp_module_visual) is pulled to rope link_20 (the far
        end) — SFP+LC visually at the far end of the cable.

    Why this is the right layer:
      The connector visuals are PhysX RigidBody prims; their final world
      pose is dictated by the joint constraints, not the authored
      xformOp:translate. Swapping translates (the prior Layer 1 fix) was a
      no-op because every tick PhysX overwrites the translate to satisfy
      the joint pulls. Swapping the joint endpoints in the USD authoring is
      the only way to change which connector ends up at which end.

    The connector visual prim PATHS within /World/cable/ stay the same in
    both variants. The extension code (extension.py:_attach_cable_to_gripper_impl,
    _compute_trial_tf_frames, scoring publishers) references the prim paths
    directly — swapping at the joint layer preserves those references.

    Translates of sfp_module_visual + sc_plug_visual remain as-authored
    in the source USD (we don't touch them). Kinematic flag is authored
    separately by author_kinematic_on_connectors.

    Returns a dict mapping joint path → list of new body0/body1 target paths
    for verification.
    """
    from pxr import Sdf
    swaps = [
        ("/World/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint",
         "physics:body0", connector_a_path, connector_b_path),
        ("/World/cable/Rope/fixedJoint",
         "physics:body1", connector_a_path, connector_b_path),
        ("/World/cable/Rope/fixedJoint2",
         "physics:body1", connector_b_path, connector_a_path),
    ]
    results = {}
    for joint_path, rel_name, expected_old, expected_new in swaps:
        joint = stage.GetPrimAtPath(joint_path)
        if not joint.IsValid():
            # The orphan finger_r FixedJoint may have been removed by an earlier
            # build pass (remove_orphan_finger_r_joint). Skip cleanly when the
            # joint isn't present — its swap is meaningless after removal.
            print(f"[swap_joint_endpoints] skip {joint_path}: not present (probably already removed)")
            continue
        rel = joint.GetRelationship(rel_name)
        if not rel.IsValid():
            raise ValueError(f"Joint {joint_path} has no {rel_name}")
        current = [str(t) for t in rel.GetTargets()]
        if current != [expected_old]:
            print(f"[swap_joint_endpoints] WARNING {joint_path}.{rel_name}: "
                  f"expected {[expected_old]}, found {current} — applying swap anyway")
        rel.SetTargets([Sdf.Path(expected_new)])
        results[joint_path] = {rel_name: expected_new}
        print(f"[swap_joint_endpoints] {joint_path}.{rel_name}: "
              f"{current} → [{expected_new}]")
    return results


def author_kinematic_on_connectors(stage, prim_paths: Tuple[str, ...]) -> dict:
    """Author physics:kinematicEnabled=True on each given prim.

    PhysX treats kinematic rigid bodies as scripted — they stay at the authored
    pose, no drift, no integration. Required for the reversed cable variant
    because the connector visuals (sfp_module_visual + sc_plug_visual) are
    authored as free-floating rigid bodies with mass=0; without the kinematic
    flag, PhysX reads the authored translate as INITIAL position at spawn then
    drifts them toward whichever rope link they spawn near. Layer 1 end-anchor
    authoring puts them at the right initial pose; this is the Layer 2 fix
    that keeps them there.

    Applies RigidBodyAPI defensively (idempotent if already applied) before
    setting the attribute, so the call works whether or not the source USD
    already carries the API. Returns {prim_path: final_value} for verification.
    """
    from pxr import Sdf, UsdPhysics

    results = {}
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        UsdPhysics.RigidBodyAPI.Apply(prim)  # idempotent
        attr = prim.GetAttribute("physics:kinematicEnabled")
        if not attr.IsValid():
            attr = prim.CreateAttribute(
                "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool
            )
        attr.Set(True)
        results[prim_path] = attr.Get()
        print(f"[author_kinematic_on_connectors] {prim_path}: "
              f"physics:kinematicEnabled = {results[prim_path]}")
    return results


def apply_end_anchor_reversal(stage, gripper_end_path: str, far_end_path: str) -> dict:
    """Move each connector subtree to the opposite end using end-anchor TRS.

    The end anchor translate/orient values come from the source USD's authored
    connector root poses. Only the root translate/orient attrs are updated.
    Scale stays untouched on each connector root, and child prim transforms are
    left alone.
    """
    gripper_end_prim = stage.GetPrimAtPath(gripper_end_path)
    far_end_prim = stage.GetPrimAtPath(far_end_path)
    if not gripper_end_prim.IsValid():
        raise ValueError(f"Prim not found: {gripper_end_path}")
    if not far_end_prim.IsValid():
        raise ValueError(f"Prim not found: {far_end_path}")

    end_a_pose = read_authored_trs(gripper_end_prim)
    end_b_pose = read_authored_trs(far_end_prim)

    sc_at_gripper_orient = compose_orient_with_yaw(end_a_pose["xformOp:orient"], 3.14159)
    sfp_at_far_orient = compose_orient_with_yaw(end_b_pose["xformOp:orient"], -1.57079)

    far_end_prim.GetAttribute("xformOp:translate").Set(end_a_pose["xformOp:translate"])
    far_end_prim.GetAttribute("xformOp:orient").Set(
        cast_quat_like(end_b_pose["xformOp:orient"], sc_at_gripper_orient)
    )

    gripper_end_prim.GetAttribute("xformOp:translate").Set(end_b_pose["xformOp:translate"])
    gripper_end_prim.GetAttribute("xformOp:orient").Set(
        cast_quat_like(end_a_pose["xformOp:orient"], sfp_at_far_orient)
    )

    return {
        gripper_end_path: read_authored_trs(gripper_end_prim),
        far_end_path: read_authored_trs(far_end_prim),
    }


# --------------------------------------------------------------------------
# Project-specific wrapper — builds the reversed cable USD by copying the
# source USD and applying swap_local_xforms.
# --------------------------------------------------------------------------

def build_cable_variant(source_usd: str, dest_usd: str, variant: str,
                         connector_a: str = CONNECTOR_A_PATH,
                         connector_b: str = CONNECTOR_B_PATH) -> str:
    """Produce a cable USD variant from the source USD.

    variant="sfp_sc_cable"          → byte-for-byte copy (identity)
    variant="sfp_sc_cable_reversed" → copy + apply end-anchor reversal:
                                       SC root moved to the gripper end,
                                       SFP+LC root moved to the far end,
                                       orient composed from the destination
                                       end anchor plus the per-end SDF yaw
                                       correction, scale untouched

    Returns the absolute path of the written USD on success.
    """
    if variant not in ("sfp_sc_cable", "sfp_sc_cable_reversed"):
        raise ValueError(
            f"variant must be 'sfp_sc_cable' or 'sfp_sc_cable_reversed', "
            f"got {variant!r}"
        )

    if not os.path.isfile(source_usd):
        raise FileNotFoundError(f"source USD missing: {source_usd}")

    os.makedirs(os.path.dirname(dest_usd) or ".", exist_ok=True)
    # When source == dest, the caller is asking for an in-place patch of the
    # existing USD (e.g. kinematic-flag retrofit on the original USD). Skip
    # the copy; the modifications below land directly in the source file.
    if os.path.realpath(source_usd) == os.path.realpath(dest_usd):
        print(f"[build_cable_variant] in-place edit on {dest_usd}")
    else:
        shutil.copyfile(source_usd, dest_usd)
        print(f"[build_cable_variant] copied {source_usd}\n"
              f"                        → {dest_usd} "
              f"({os.path.getsize(dest_usd)} B)")

    from pxr import Usd
    stage = Usd.Stage.Open(dest_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open {dest_usd} for editing")

    # Both variants — apply the kinematic flag + remove the orphan FixedJoint at
    # gripper_hande_finger_link_r. The orphan joint was wrong-anchor (locked
    # the held connector at the gripper BASE, not the gripper TCP) — the
    # extension code's _attach_cable_to_gripper_impl now installs a per-tick
    # physics-step tracker that positions the held connector at gripper_tcp,
    # mirroring Gazebo CablePlugin behavior. The joint must be GONE so the
    # tracker has clear field; see remove_orphan_finger_r_joint() docstring
    # for the 2026-05-17 diagnostic trail.
    author_kinematic_on_connectors(stage, (connector_a, connector_b))
    remove_orphan_finger_r_joint(stage)

    if variant == "sfp_sc_cable":
        stage.GetRootLayer().Save()
        print(f"[build_cable_variant] saved identity variant (kinematic + orphan-joint removed): {dest_usd}")
        return dest_usd

    # Reversed variant — also swap which connector each cable-rope-end joint
    # binds to. The orphan-joint swap is no longer needed (the joint is gone),
    # but the fixedJoint/fixedJoint2 inside /World/cable/Rope/ still determine
    # which connector visually lives at which rope end. For trial_3, SC must
    # be at the gripper-end rope link, SFP at the far end.
    joint_results = swap_joint_endpoints_for_reversed_cable(
        stage, connector_a, connector_b
    )
    stage.GetRootLayer().Save()
    print(f"[build_cable_variant] applied joint-endpoint swap (rope-end bindings):")
    for joint_path, changes in joint_results.items():
        print(f"  {joint_path}: {changes}")
    print(f"[build_cable_variant] saved reversed variant: {dest_usd}")
    return dest_usd


# --------------------------------------------------------------------------
# CLI entry point
# --------------------------------------------------------------------------

def main(argv) -> int:
    parser = argparse.ArgumentParser(
        description="Build a reversed-orientation cable USD by swapping "
                     "connector visual subtrees.",
    )
    parser.add_argument("--source", default=DEFAULT_SOURCE_USD,
                        help=f"Source USD (default: {DEFAULT_SOURCE_USD})")
    parser.add_argument("--dest", default=DEFAULT_DEST_USD,
                        help=f"Destination USD (default: {DEFAULT_DEST_USD})")
    parser.add_argument("--variant", default="sfp_sc_cable_reversed",
                        choices=["sfp_sc_cable", "sfp_sc_cable_reversed"],
                        help="Cable type to produce. 'sfp_sc_cable' is identity copy.")
    parser.add_argument("--connector-a", default=CONNECTOR_A_PATH,
                        help="Prim path of first connector visual to swap.")
    parser.add_argument("--connector-b", default=CONNECTOR_B_PATH,
                        help="Prim path of second connector visual to swap.")
    args = parser.parse_args(argv[1:])

    try:
        out_path = build_cable_variant(
            args.source, args.dest, args.variant,
            connector_a=args.connector_a,
            connector_b=args.connector_b,
        )
    except Exception as exc:  # noqa: BLE001 — surface to stderr w/ traceback
        import traceback
        traceback.print_exc()
        print(f"[build_cable_variant] FAILED: {exc}", file=sys.stderr)
        return 1
    print(f"[build_cable_variant] done: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
