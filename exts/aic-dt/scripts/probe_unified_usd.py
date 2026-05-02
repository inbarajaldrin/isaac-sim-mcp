#!/usr/bin/env python3
"""probe_unified_usd.py — enumerate USD prim names under the unified robot USD and
diff against the live aic_eval TF frame list (aic_frames_live.gv).

Authored by Plan 05 Task 1 (Phase 1 Foundation Parity / PARITY-04).

Usage:
    ~/env_isaaclab/bin/python exts/aic-dt/scripts/probe_unified_usd.py \\
        [USD_PATH] [FRAMES_GV_PATH]

Defaults:
    USD_PATH        = exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd
    FRAMES_GV_PATH  = .planning/phases/01-foundation-parity/aic_frames_live.gv

Output (stdout, also captured to usd_prim_inventory.txt by the caller):
    [USD prims under stage]    list of all Xform/Joint prim paths
    [Live frame names ...]     unique sorted frame names from .gv
    [Diff: ...]                per-live-frame match-status row
    [Recommended fix strategy] strategy name + rationale
    [Override list ...]        machine-parseable override entries
"""
import os
import re
import sys
from pathlib import Path

from pxr import Usd

DEFAULT_USD = "exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd"
DEFAULT_GV = ".planning/phases/01-foundation-parity/aic_frames_live.gv"


def parse_frames(gv_path):
    """Pull all unique double-quoted tokens from the .gv file.

    aic_frames_live.gv contains TF frame names as quoted node identifiers like
    "wrist_3_link" -> "flange". The legend cluster also contains a quoted
    "Recorded at time: ..." line we filter out by ":" exclusion.
    """
    text = Path(gv_path).read_text()
    raw = set(re.findall(r'"([^"]+)"', text))
    # Drop legend/timestamp tokens (anything containing ":" — frame names never have ":")
    return sorted(t for t in raw if ":" not in t)


def enumerate_prims(usd_path):
    """Open the USD layer with pxr standalone and traverse the full stage."""
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        raise RuntimeError(f"Cannot open USD: {usd_path}")
    keep_types = {
        "Xform",
        "PhysicsRevoluteJoint",
        "PhysicsPrismaticJoint",
        "PhysicsFixedJoint",
        "Joint",
    }
    prims = []
    for prim in stage.Traverse():
        tn = prim.GetTypeName()
        if tn in keep_types or (tn == "" and prim.GetPath().pathString != "/"):
            prims.append(str(prim.GetPath()))
    return sorted(prims)


def main():
    usd_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_USD
    gv_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_GV
    print(f"USD: {usd_path}")
    print(f"GV : {gv_path}\n")

    if not os.path.exists(usd_path):
        print(f"ERROR: USD path does not exist: {usd_path}")
        sys.exit(2)
    if not os.path.exists(gv_path):
        print(f"ERROR: GV path does not exist: {gv_path}")
        sys.exit(2)

    frames = parse_frames(gv_path)
    prims = enumerate_prims(usd_path)

    print("[USD prims under stage]")
    for p in prims:
        print(f"  {p}")

    print("\n[Live frame names from aic_frames_live.gv]")
    for f in frames:
        print(f"  {f}")

    print("\n[Diff: live frame -> USD prim match]")
    mismatches = []
    for f in frames:
        f_us = f.replace("/", "_")
        match = None
        # Walk all prims; pick the first whose leaf-name equals either form
        for p in prims:
            leaf = p.rsplit("/", 1)[-1]
            if leaf == f or leaf == f_us:
                match = p
                break
        if match is None:
            status = "NO-PRIM"
            mismatches.append((f, None))
        else:
            leaf = match.rsplit("/", 1)[-1]
            if leaf == f:
                status = "MATCH"
            else:
                status = "UNDERSCORE-MISMATCH"
                mismatches.append((f, match))
        print(f"  {status:22s}  live='{f}'  prim='{match or '<none>'}'")

    n_underscore = len([m for m in mismatches if m[1] is not None])
    n_no_prim = len([m for m in mismatches if m[1] is None])
    print(
        f"\n[Summary] {len(frames)} live frames | "
        f"{n_underscore} underscore-mismatches | {n_no_prim} no-prim"
    )

    print("\n[Recommended fix strategy]")
    if not mismatches:
        print(
            "  STRATEGY: NONE -- prim names already match live frame_ids; default "
            "ROS2PublishTransformTree mapping is correct."
        )
        recommended = "NONE"
    elif n_no_prim > 0 and n_underscore == 0:
        print(
            "  STRATEGY: PER-FRAME-RAW-OVERRIDE -- some live frames have no "
            "matching USD prim (likely cosmetic frames with no driver Xform: "
            "tool0, flange, ft_frame, world, tabletop, base, base_link_inertia, etc.). "
            "Plan 06's TF builder must publish these via ROS2PublishRawTransformTree "
            "with explicit parent/child frame_ids."
        )
        recommended = "PER-FRAME-RAW-OVERRIDE"
    elif n_underscore > 0 and all("/" in f for f, _ in mismatches if _ is not None):
        # All matched mismatches are underscore-vs-slash AND every no-prim entry exists
        # too -- mixed picture. Recommend Raw-publisher-override as the safe path.
        if n_no_prim > 0:
            print(
                f"  STRATEGY: PER-FRAME-RAW-OVERRIDE -- mixed mismatch pattern "
                f"({n_underscore} underscore prims + {n_no_prim} no-prim frames). "
                "ROS2PublishRawTransformTree is the only strategy that handles BOTH "
                "renaming AND synthesizing missing TF edges."
            )
            recommended = "PER-FRAME-RAW-OVERRIDE"
        else:
            print(
                f"  STRATEGY: SUBLAYER-RENAME -- {n_underscore} prims need "
                "underscore-to-slash renaming (all live frames have matching USD "
                "prims; only naming differs). Author "
                "exts/aic-dt/assets/robot/aic_unified_robot_frame_overrides.usda "
                "with prim-name overrides."
            )
            recommended = "SUBLAYER-RENAME"
    else:
        print(
            f"  STRATEGY: PER-FRAME-RAW-OVERRIDE -- heterogeneous mismatch types; "
            f"use ROS2PublishRawTransformTree for the {len(mismatches)} entries listed above. "
            "Plan 06's TF builder reads the override list."
        )
        recommended = "PER-FRAME-RAW-OVERRIDE"

    print("\n[Override list for Plan 06 / sublayer]")
    for live_frame, prim_path in mismatches:
        print(f"  OVERRIDE  prim='{prim_path}'  expected_frame_id='{live_frame}'")

    print(f"\n[Recommended strategy token] {recommended}")


if __name__ == "__main__":
    main()
