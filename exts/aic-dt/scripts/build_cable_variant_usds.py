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


# --------------------------------------------------------------------------
# Project-specific wrapper — builds the reversed cable USD by copying the
# source USD and applying swap_local_xforms.
# --------------------------------------------------------------------------

def build_cable_variant(source_usd: str, dest_usd: str, variant: str,
                         connector_a: str = CONNECTOR_A_PATH,
                         connector_b: str = CONNECTOR_B_PATH) -> str:
    """Produce a cable USD variant from the source USD.

    variant="sfp_sc_cable"          → byte-for-byte copy (identity)
    variant="sfp_sc_cable_reversed" → copy + swap connector_a ↔ connector_b
                                       local xforms

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
    shutil.copyfile(source_usd, dest_usd)
    print(f"[build_cable_variant] copied {source_usd}\n"
          f"                        → {dest_usd} "
          f"({os.path.getsize(dest_usd)} B)")

    if variant == "sfp_sc_cable":
        return dest_usd  # identity copy — no swap

    # Reversed variant — apply the connector swap.
    from pxr import Usd
    stage = Usd.Stage.Open(dest_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open {dest_usd} for editing")

    swapped_paths = swap_local_xforms(stage, connector_a, connector_b)
    stage.GetRootLayer().Save()
    print(f"[build_cable_variant] swapped local xforms on:\n"
          f"                        {swapped_paths[0]}\n"
          f"                        {swapped_paths[1]}")
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
