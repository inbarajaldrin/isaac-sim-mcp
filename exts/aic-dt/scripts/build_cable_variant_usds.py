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

    body0 (the connector visual) was originally bound by an earlier
    pre-removal build pass; the legacy `swap_joint_endpoints_for_reversed_cable`
    function used to swap it for reversed variants but was dropped on
    2026-05-17 since this orphan joint is now removed unconditionally
    regardless of variant.

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


def reauthor_rope_end_joints_identity(stage, *, variant: str) -> list:
    """Re-author the two rope-end FixedJoints with IDENTITY localRot0/Rot1.

    Why this exists (2026-05-17 follow-up):
      The ORIGINAL fixedJoint had non-identity localRot0 ≈ (0.5,-0.5,0.5,-0.5),
      a 120° rotation between link_0's frame and sfp_module_visual's frame. PhysX
      cooked that as a hard constraint and overrode the per-tick tracker's
      kinematic xform writes, producing the trial_3 90° orientation offset.
      Path-b removed both joints entirely — fixing the orientation fight but
      leaving the 21-link mass≈0 rope chain unanchored, so PhysX integrator
      drifted the rope ~10m off-scene (rope invisible in the wrist-cam render).

      Restore both joints with IDENTITY localRot0/Rot1. The connector and the
      anchored rope link share the SAME world frame at the joint origin —
      no rotation differential, no constraint fight against the kinematic
      pose. Keep the original localPos0/Pos1 sub-cm offsets so the rope
      visually attaches at the right point on each connector mesh.

    Variant routing (replaces the legacy `swap_joint_endpoints_for_reversed_cable`
    function dropped on 2026-05-17 — joint authoring is now canonical here
    rather than spread across separate authoring + swap passes):
      - sfp_sc_cable          : link_0 ↔ sfp_module_visual, link_20 ↔ sc_plug_visual
      - sfp_sc_cable_reversed : link_0 ↔ sc_plug_visual,    link_20 ↔ sfp_module_visual

    Both bindings give the kinematic connector at the gripper-end its
    corresponding rope-end link as a dependent (rope-end rope-link follows
    the connector). The rope chain dangles between via its D6 joints.

    Returns list of joint paths re-authored.
    """
    from pxr import UsdPhysics, Gf, Sdf

    joint_left = "/World/cable/Rope/fixedJoint"   # was link_0 ↔ sfp_module_visual
    joint_right = "/World/cable/Rope/fixedJoint2" # was link_20 ↔ sc_plug_visual

    if variant == "sfp_sc_cable":
        bindings = [
            (joint_left,  "/World/cable/Rope/Rope/link_0",  "/World/cable/sfp_module_visual",
             (-0.026749987, 0.0, 0.0)),  # original localPos0
            (joint_right, "/World/cable/Rope/Rope/link_20", "/World/cable/sc_plug_visual",
             (0.03325002, 0.0, 0.0)),
        ]
    elif variant == "sfp_sc_cable_reversed":
        bindings = [
            (joint_left,  "/World/cable/Rope/Rope/link_0",  "/World/cable/sc_plug_visual",
             (-0.026749987, 0.0, 0.0)),
            (joint_right, "/World/cable/Rope/Rope/link_20", "/World/cable/sfp_module_visual",
             (0.03325002, 0.0, 0.0)),
        ]
    else:
        raise ValueError(f"Unknown variant: {variant}")

    authored = []
    identity_quatf = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
    for jp, body0_path, body1_path, local_pos0 in bindings:
        joint = stage.GetPrimAtPath(jp)
        if not joint or not joint.IsValid():
            joint = stage.DefinePrim(jp, "PhysicsFixedJoint")
        else:
            joint.SetTypeName("PhysicsFixedJoint")
        # Set body0/body1
        rel0 = joint.GetRelationship("physics:body0") or joint.CreateRelationship("physics:body0")
        rel1 = joint.GetRelationship("physics:body1") or joint.CreateRelationship("physics:body1")
        rel0.SetTargets([Sdf.Path(body0_path)])
        rel1.SetTargets([Sdf.Path(body1_path)])
        # IDENTITY localRot0/Rot1 — no rotation differential
        for rot_attr, ptype in (("physics:localRot0", Sdf.ValueTypeNames.Quatf),
                                 ("physics:localRot1", Sdf.ValueTypeNames.Quatf)):
            a = joint.GetAttribute(rot_attr)
            if not a.IsValid():
                a = joint.CreateAttribute(rot_attr, ptype)
            a.Set(identity_quatf)
        # localPos0 from URDF authoring (sub-cm offsets)
        pos0 = joint.GetAttribute("physics:localPos0")
        if not pos0.IsValid():
            pos0 = joint.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Float3)
        pos0.Set(Gf.Vec3f(*local_pos0))
        pos1 = joint.GetAttribute("physics:localPos1")
        if not pos1.IsValid():
            pos1 = joint.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Float3)
        pos1.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        # Joint flags
        je = joint.GetAttribute("physics:jointEnabled")
        if not je.IsValid():
            je = joint.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool)
        je.Set(True)
        ce = joint.GetAttribute("physics:collisionEnabled")
        if not ce.IsValid():
            ce = joint.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
        ce.Set(False)
        authored.append(jp)
        print(f"[reauthor_rope_end_joints_identity] {jp}: body0={body0_path} body1={body1_path} "
              f"localPos0={local_pos0} localRot0=identity")
    return authored


def remove_rope_end_fixed_joints(stage) -> list:
    """Remove the two rope-end FixedJoints that anchor the kinematic connector
    visuals to the rope-end RigidBody links.

    Paths:
      /World/cable/Rope/fixedJoint   (body1 = sfp_module_visual or sc_plug_visual)
      /World/cable/Rope/fixedJoint2  (body1 = the other connector)

    Why removed (2026-05-17, GPT pose diagnosis path-b):
      fixedJoint authors physics:localRot0 = (0.499, -0.5, 0.5, -0.499) — a
      non-identity ~90° quat between link_0's frame and the connector body's
      frame. Once PhysX cooks the joint, the solver enforces
        W_link_0 * Gf.Matrix(localRot0) == W_connector * Gf.Matrix(localRot1)
      every tick, snapping the connector's pose to (link_0 pose) ∘ (90°). The
      kinematic xform writes from _install_held_connector_tcp_tracker are
      silently overridden by the joint solver, producing the ~90° orientation
      delta observed in trial_3 sc_plug (rpy_deg=(-179.7,2.1,0.5) instead of
      Gazebo's (91.7,63.4,-89.3)).

      The cable rope is decorative for M1 — scoring is contact-based on
      plug-end geometry through plug_proxy (a synthetic Xform on the gripper
      finger), not on rope shape. Removing the rope-end joints frees the
      kinematic plugs from the constraint fight; the rope links keep their
      D6Joint chain to each other but dangle from their authored origin
      without anchoring back to the connectors.

      With physics:kinematicEnabled=True already authored on both connector
      visuals (author_kinematic_on_connectors, Layer 2), and the per-tick
      tracker at extension.py::_install_held_connector_tcp_tracker driving
      pose for the held connector, removing these joints leaves the
      tracker as the only pose authority for the held connector. The
      non-held connector stays at its authored kinematic pose.

    Returns the list of joint paths actually removed.
    """
    removed = []
    for joint_path in (
        "/World/cable/Rope/fixedJoint",
        "/World/cable/Rope/fixedJoint2",
    ):
        prim = stage.GetPrimAtPath(joint_path)
        if prim and prim.IsValid():
            stage.RemovePrim(joint_path)
            removed.append(joint_path)
            print(f"[remove_rope_end_fixed_joints] removed {joint_path}")
        else:
            print(f"[remove_rope_end_fixed_joints] not present at {joint_path} — skipping")
    return removed


def replace_plug_subtree_with_glb_refs(
    stage, plug_path: str, child_specs: list,
    reset_parent_scale_to_identity: bool = False,
) -> dict:
    """Wipe all children of the connector parent prim at plug_path, then add
    new child Xforms that AddReference the thin GLB-USDs built by
    build_mount_rail_usds.py. Mirrors the socket-spawn pattern in
    extension.py::_spawn_component_via_usd so the gltf SDF plugin handles
    Y-up→Z-up axis conversion, meter-units conversion, and full GLB
    material + texture bindings at load time.

    Why this exists (2026-05-17 visual-parity fix):
      The cable USD's connector visuals (/World/cable/sc_plug_visual and
      /World/cable/sfp_module_visual) were inlined as raw mesh subtrees with
      empty Looks scopes (sc_plug — fixed in b481385 via /Looks AddReference)
      or hand-authored UsdPreviewSurface materials (sfp_module — e34ee5f).
      Neither carried the GLB-side baseColor textures + normal maps + PBR
      that the AIC source GLBs ship with, so the live render didn't match
      the Gazebo reference. The sockets (sc_port, lc_mount, nic_card_mount,
      etc.) use the clean thin-USD-AddReference pattern — let the gltf SDF
      plugin do the work. This helper rewires the plugs to that same path.

    The parent prim is preserved (it carries RigidBodyAPI + kinematicEnabled
    plus the runtime trackers' xformOp:translate / xformOp:orient writes).
    Only its children + MaterialBindingAPI are touched.

    Args:
      plug_path: e.g. "/World/cable/sc_plug_visual". The connector parent
                 whose pose is owned by the runtime kinematic trackers
                 (extension.py::_install_held_connector_tcp_tracker /
                 _install_far_connector_world_tracker).
      child_specs: list of dicts. Required keys:
        - name: child Xform name under plug_path
        - usd:  relative path to the thin GLB-USD from the cable USD's dir,
                e.g. "../assets/SC Plug/sc_plug_visual.usd"
        Optional keys:
        - translate: (x,y,z) tuple in parent-frame meters (default (0,0,0))
        - fallback_material: dict for cases where the gltf SDF plugin drops
                material bindings on the imported subtree (e.g. SFP body
                GLB whose Body.005 mesh ships with two GLB-primitive
                materials that don't survive the gltf→USD bridge). When
                present, an inline UsdPreviewSurface is authored under
                the child Xform's own /Looks scope and bound on the child
                via MaterialBindingAPI — propagates down to descendant
                meshes that lack their own bindings. Schema:
                  {"name": "Material_005",
                   "diffuse": (1.0, 1.0, 1.0),
                   "metallic": 1.0,
                   "roughness": 1.0}
      reset_parent_scale_to_identity: when True, sets the plug parent's
        xformOp:scale to (1,1,1). Needed for sc_plug — the source cable USD
        pre-flattened the GLB→meter 0.01 conversion onto the parent (each
        of the 13 inline meshes had identity scale). The new GLB-reference
        path lets the gltf SDF plugin apply its own meters-per-unit scale
        inside the referenced subtree, so leaving the parent at 0.01 would
        double-apply and shrink the plug 100×.

    Returns dict of {child_path: thin_usd_relpath} for verification.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    plug = stage.GetPrimAtPath(plug_path)
    if not plug.IsValid():
        print(f"[replace_plug_subtree_with_glb_refs] plug parent not found: {plug_path}")
        return {}

    # Clear any parent-level MaterialBindingAPI direct binding. USD's
    # MaterialBindingAPI semantics give parent bindings precedence over
    # descendant bindings (via "direct" strength) unless the descendant
    # binds with "strongerThanDescendants". The b481385/e34ee5f fixes both
    # bound on the parent; that binding would mask the GLB-side material
    # bindings the gltf SDF plugin authors on the imported mesh subtree.
    if plug.HasAPI(UsdShade.MaterialBindingAPI):
        binding = UsdShade.MaterialBindingAPI(plug)
        binding.GetDirectBindingRel().ClearTargets(removeSpec=False)
        print(f"[replace_plug_subtree_with_glb_refs] cleared parent MaterialBinding on {plug_path}")

    # Optional: reset parent scale. The runtime tracker writes only
    # translate/orient, so scale survives the rebuild — must be 1 here
    # (not 0.01) so the gltf-side units-resolve scale lands the GLB at the
    # right meter size, no double-application.
    if reset_parent_scale_to_identity:
        xf = UsdGeom.Xformable(plug)
        scale_op = None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scale_op = op
                break
        if scale_op is None:
            scale_op = xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        scale_op.Set(Gf.Vec3d(1.0, 1.0, 1.0))
        print(f"[replace_plug_subtree_with_glb_refs] reset {plug_path} scale=(1,1,1)")

    # Wipe ALL children — the inline meshes + Looks scope + any nested
    # Xforms. The runtime tracker doesn't iterate children; it writes only
    # on the parent. Cable rope-end joints attach to the parent prim
    # (per reauthor_rope_end_joints_identity), not its children, so this
    # is safe for the rope chain too.
    for child in list(plug.GetChildren()):
        child_path = child.GetPath()
        stage.RemovePrim(child_path)
    print(f"[replace_plug_subtree_with_glb_refs] removed all children under {plug_path}")

    out = {}
    for spec in child_specs:
        name = spec["name"]
        usd_rel = spec["usd"]
        translate = spec.get("translate", (0.0, 0.0, 0.0))
        fallback = spec.get("fallback_material")

        child_path = f"{plug_path}/{name}"
        child_xform = UsdGeom.Xform.Define(stage, child_path)
        child_xform.ClearXformOpOrder()
        if any(v != 0.0 for v in translate):
            child_xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
        child_xform.GetPrim().GetReferences().AddReference(usd_rel)
        out[child_path] = usd_rel
        print(f"[replace_plug_subtree_with_glb_refs] {child_path} → AddReference({usd_rel}) "
              f"translate={tuple(translate)}")

        if fallback is not None:
            mat_name = fallback["name"]
            mat_path = f"{child_path}/Looks/{mat_name}"
            looks_prim = stage.GetPrimAtPath(f"{child_path}/Looks")
            if not looks_prim.IsValid():
                stage.DefinePrim(f"{child_path}/Looks", "Scope")
            material = UsdShade.Material.Define(stage, mat_path)
            shader = UsdShade.Shader.Define(stage, f"{mat_path}/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                tuple(float(v) for v in fallback["diffuse"]))
            shader.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(float(fallback["metallic"]))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(fallback["roughness"]))
            shader.CreateInput("opacity",   Sdf.ValueTypeNames.Float).Set(1.0)
            shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)
            shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(child_xform.GetPrim()).GetDirectBindingRel().SetTargets(
                [Sdf.Path(mat_path)])
            print(f"[replace_plug_subtree_with_glb_refs] {child_path} fallback material "
                  f"{mat_name} (metallic={fallback['metallic']}, roughness={fallback['roughness']}) "
                  f"bound — propagates to GLB descendants lacking own bindings")
    return out


def author_gripper_prismatic_joints(stage) -> dict:
    """Convert the Robotiq Hand-E finger joints from PhysicsFixedJoint to
    PhysicsPrismaticJoint with DriveAPI + symmetric mirror, so the gripper
    width can adapt per trial instead of being locked at 17.3mm.

    Source-of-truth: ~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro
    + robotiq_hande_macro.xacro:
      - left_finger_joint  prismatic axis=(1,0,0)  lower=0  upper=0.025
      - right_finger_joint prismatic axis=(-1,0,0) lower=0  upper=0.025
                           <mimic joint=left multiplier=1 offset=0/>
      - grip_pos_min=0.0  grip_pos_max=0.025  grip_vel_max=0.05
      - grip_effort_max=130
      - gripper_initial_pos default=0.00655 (some trials 0.0073)

    USD authoring decisions:
      - axis token is X for BOTH joints (USD PrismaticJoint axis is a
        token, no sign).
      - Right joint gets localRot0 = 180°-about-Z so positive joint coord
        in its local frame maps to -X in body0 (gripper_hande_base_link)
        frame. URDF's axis=(-1,0,0) on right = our localRot0 flip.
      - Limits 0..0.025 on both joints per URDF.
      - DriveAPI("linear") with target=initial_pos, stiffness=10000,
        damping=1000, maxForce=130 (effort_max). Stiffness chosen for
        snappy ~50Hz position response; tune at runtime via gripper_command
        MCP atom.
      - existing localPos0 (≈5mm for left, ≈-12mm for right) preserved —
        represents URDF authoring offset already encoded in the unified-
        robot USD. Drive target = 0 then puts each finger at the
        URDF-authored neutral pose; positive target separates them along
        the mirror axes.

    Note: URDF mimic semantics (right follows left) are NOT a USD/PhysX
    concept — must be enforced at runtime by writing both drive targets
    equally. The gripper_command MCP atom in extension.py owns this.

    Returns dict mapping joint path → {'type': original_type, 'axis': ...}.
    """
    from pxr import UsdPhysics, Gf, Sdf

    INITIAL_POS = 0.00655
    LOWER = 0.0
    UPPER = 0.025
    STIFFNESS = 10000.0
    DAMPING = 1000.0
    MAX_FORCE = 130.0

    # 180° about Z in Gf.Quatf: (w, x, y, z) = (0, 0, 0, 1)
    flip_z_quatf = Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.0, 1.0))

    results = {}
    for joint_path, flip_axis in (
        ("/World/aic_unified_robot/joints/gripper_left_finger_joint", False),
        ("/World/aic_unified_robot/joints/gripper_right_finger_joint", True),
    ):
        joint = stage.GetPrimAtPath(joint_path)
        if not joint.IsValid():
            print(f"[author_gripper_prismatic_joints] joint not found: {joint_path}")
            continue
        orig_type = str(joint.GetTypeName())
        joint.SetTypeName("PhysicsPrismaticJoint")

        prismatic = UsdPhysics.PrismaticJoint(joint)
        prismatic.CreateAxisAttr("X")
        prismatic.CreateLowerLimitAttr(LOWER)
        prismatic.CreateUpperLimitAttr(UPPER)

        if flip_axis:
            # Right finger: rotate joint frame 180° about Z so positive
            # joint coord = -X motion in body0 frame (matches URDF axis=(-1,0,0)).
            rot_attr = joint.GetAttribute("physics:localRot0")
            if rot_attr.IsValid():
                rot_attr.Set(flip_z_quatf)
            else:
                joint.CreateAttribute(
                    "physics:localRot0", Sdf.ValueTypeNames.Quatf
                ).Set(flip_z_quatf)

        drive = UsdPhysics.DriveAPI.Apply(joint, "linear")
        drive.CreateTargetPositionAttr(INITIAL_POS)
        drive.CreateStiffnessAttr(STIFFNESS)
        drive.CreateDampingAttr(DAMPING)
        drive.CreateMaxForceAttr(MAX_FORCE)

        results[joint_path] = {
            "orig_type": orig_type,
            "axis": "X",
            "flip_localRot0_180Z": flip_axis,
            "limits": (LOWER, UPPER),
            "drive_target": INITIAL_POS,
        }
        print(f"[author_gripper_prismatic_joints] {joint_path}: "
              f"{orig_type} → PhysicsPrismaticJoint, axis=X, "
              f"flip={flip_axis}, limits=({LOWER},{UPPER}), target={INITIAL_POS}")
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
    # 2026-05-17 GPT pose diagnosis path-b: remove rope-end FixedJoints so the
    # per-tick TCP tracker is the sole pose authority for the held kinematic
    # connector. The joints' non-identity localRot0 was overriding the
    # tracker's xform Set, producing the trial_3 90° sc_plug delta.
    # First remove any pre-existing rope-end joints (idempotent cleanup), then
    # re-author them with IDENTITY localRot0/Rot1 + per-variant body0/body1
    # bindings. This pins the rope chain to the kinematic connectors without
    # the orientation fight that path-b removal was avoiding — and the cable
    # wire is visible again (chains no longer drift off-scene).
    rope_joints_removed = remove_rope_end_fixed_joints(stage)
    rope_joints_authored = reauthor_rope_end_joints_identity(stage, variant=variant)
    # Visual parity fix (2026-05-17): replace the inline mesh subtrees under
    # both connector parents with thin-USD references to GLB-backed visuals
    # built by build_mount_rail_usds.py. This mirrors the working socket-
    # spawn pattern (extension.py::_spawn_component_via_usd) so the gltf SDF
    # plugin handles Y-up→Z-up + units conversion AND the full GLB material
    # texture stack at load time — no hand-authored UsdPreviewSurface, no
    # /Looks-only AddReference partial fix. Both prior approaches lost the
    # GLB-side baseColor textures the AIC stack ships with; the live render
    # didn't match Gazebo. The new path makes both ends source-of-truth.
    #
    # The plug parent prims themselves (their RigidBodyAPI + kinematicEnabled
    # + runtime tracker xformOp writes) are preserved — only their children
    # and parent MaterialBinding are touched.
    replace_plug_subtree_with_glb_refs(
        stage, connector_b,
        [{"name": "visual",
          "usd":  "../assets/SC Plug/sc_plug_visual.usd"}],
        # sc_plug_visual parent has scale=0.01 baked in (the source cable USD
        # pre-flattened the GLB→meter conversion onto the parent). The new
        # GLB-reference path lets the gltf SDF plugin apply meters-per-unit
        # inside the referenced subtree, so leave parent at identity to
        # avoid 100× double-application.
        reset_parent_scale_to_identity=True,
    )
    replace_plug_subtree_with_glb_refs(
        stage, connector_a,
        [
            # SFP body (housing) — same local origin as the parent. The GLB's
            # Body.005 mesh has TWO GLB primitives sharing one mesh, each
            # with its own material (Material.005 metallic + Material.001
            # non-metallic) — the gltf SDF plugin can't synthesize
            # UsdGeomSubset partitions to preserve that mapping, so the
            # imported mesh arrives unbound (renders default gray). Author
            # the dominant Material.005 (metallic white, per AIC source-GLB
            # parse) inline as a fallback that propagates down via
            # MaterialBindingAPI. Per-GLB-primitive distinction (the small
            # Material.001 facet) is M2 work — needs UsdGeomSubset wiring.
            {"name": "sfp_body",
             "usd":  "../assets/SFP Module/sfp_module_visual.usd",
             "fallback_material": {
                 "name": "Material_005",
                 "diffuse": (1.0, 1.0, 1.0),
                 "metallic": 1.0,
                 "roughness": 1.0,
             }},
            # LC plug (the LC duplex connector attached to the SFP housing's
            # cable exit) — offset by the same translate the original inline
            # sub-Xform carried, preserving the plug-tip position relative
            # to the SFP body. The GLB ships with Plastic_Blue.002 bindings.
            {"name": "lc_plug",
             "usd":  "../assets/LC Plug/lc_plug_visual.usd",
             "translate": (-0.0007481417293789746,
                           -0.0013461820330080822,
                           0.031288921)},
        ],
        # sfp_module_visual parent already has scale=1; no reset needed.
        reset_parent_scale_to_identity=False,
    )
    # Gripper Hand-E 1-DOF prismatic conversion: both finger joints become
    # PhysicsPrismaticJoint + DriveAPI so the gripper width can vary per
    # trial (URDF mimic at runtime — extension.py writes both drive targets).
    author_gripper_prismatic_joints(stage)

    # With rope-end joints gone, the reversed variant no longer needs a
    # joint-endpoint swap — there is nothing to swap. The "which connector at
    # which end" decision moves entirely into the runtime tracker (which holds
    # the held connector at gripper_tcp per cable_kwargs.cable_type) plus the
    # connectors' authored kinematic poses (the far connector stays where the
    # source USD placed it). For trial_3 (sfp_sc_cable_reversed), the runtime
    # tracker reads _HELD_CONNECTOR_POSE_OFFSETS[cable_type]['rel_quat_xyzw']
    # to compose the right orient; the cable_type is set by load_trial. The
    # source USD's connector positions remain valid for the non-held end.
    if variant == "sfp_sc_cable":
        stage.GetRootLayer().Save()
        print(f"[build_cable_variant] saved identity variant "
              f"(kinematic + orphan-joint removed + {len(rope_joints_removed)} rope-end joints removed): {dest_usd}")
        return dest_usd

    stage.GetRootLayer().Save()
    print(f"[build_cable_variant] saved reversed variant "
          f"(kinematic + orphan-joint removed + {len(rope_joints_removed)} rope-end joints removed; "
          f"runtime tracker now owns held-connector pose per cable_type): {dest_usd}")
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
