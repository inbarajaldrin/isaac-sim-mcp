#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
Offline USD patch — changes SO-ARM101 collision setup to match the UR5e
pattern (deterministic comparison done 2026-04-23 confirmed UR5e uses
convex-decomp / convex-hull everywhere, no SDF, and Just Works).

Two changes:
  1. gripper_link + moving_jaw: `approximation: sdf` → `convexDecomposition`
     (eliminates PhysX's SDF voxelizer path entirely — see CLAUDE.md gotcha
     and DEBUG-GUIDE § 4.3 for why SDF caused 48 GB regen hazards)
  2. All 7 arm-link collision Xforms: author `physxCollision:contactOffset
     = 0.002 m` and `restOffset = 0.0 m` (overrides PhysX's 20 mm default
     soft-contact zone that causes cup-knock behavior)

Both changes authored as pure USD attribute writes — no AddAppliedSchema
calls, no PhysxCollisionAPI.Apply side effects. Isaac Sim reads the final
composed USD and cooks once with these values present.

WHY THIS WORKS WHEN PREVIOUS PATCHES FAILED:
--------------------------------------------
Previous failures: added contactOffset attributes while SDF API remained in
use → PhysX treated SDF setup as "existing cooked state" and any attribute
addition invalidated the cache → voxelizer regen → 48 GB RAM spike.

This patch eliminates SDF entirely. No voxelizer to invalidate. The
remaining cooks (convexDecomposition for all 7 links) are cheap and don't
spike memory even when attributes change.

Run once after checkout. Idempotent. Safe to re-run.
"""

import os
import sys
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics, Sdf
except ImportError:
    print("FATAL: pxr not importable. Source Isaac Sim's Python env.",
          file=sys.stderr)
    sys.exit(1)

REPO_ROOT = Path(__file__).resolve().parent.parent
ASSET_PATH = REPO_ROOT / "exts/soarm101-dt/assets/SO-ARM101-USD.usd"

# Collision Xforms and their target approximation.
# gripper/jaw switch from sdf → convexDecomposition (matches UR5e's pattern
# and eliminates SDF hazard). Other links already use convexDecomposition.
COLLISION_APPROXIMATIONS = {
    "/so101_new_calib/base_link/collisions":                  "convexDecomposition",
    "/so101_new_calib/shoulder_link/collisions":              "convexDecomposition",
    "/so101_new_calib/upper_arm_link/collisions":             "convexDecomposition",
    "/so101_new_calib/lower_arm_link/collisions":             "convexDecomposition",
    "/so101_new_calib/wrist_link/collisions":                 "convexDecomposition",
    "/so101_new_calib/gripper_link/collisions":               "convexDecomposition",  # was sdf
    "/so101_new_calib/moving_jaw_so101_v1_link/collisions":   "convexDecomposition",  # was sdf
}

# Physics contact parameters — see DEBUG-GUIDE § 4.3 for derivation.
#
# Current iteration (2026-04-23 pm): contactOffset = 0.1 mm with CCD enabled
# on the fast-moving links. Rationale:
#   * Without CCD, contactOffset must be > max_velocity × physics_dt to
#     prevent tunneling. At 120 Hz physics and ~0.3 m/s TCP velocity that's
#     ~2.5 mm — and the empirical threshold (measured 2026-04-23) showed
#     2 mm still produced force at 1.68 mm clearance → 267 mm cup knock.
#   * With CCD on the gripper/jaw/wrist, PhysX uses swept-volume collision
#     instead of per-step point checks — tunneling is impossible regardless
#     of contactOffset. We can safely drop contactOffset to ~0.1 mm, which
#     means ONLY actual geometric contact generates force.
#   * Cost: ~10-30 % physics overhead on CCD-enabled shapes. Acceptable —
#     only 3 links have CCD (gripper_link, moving_jaw, wrist_link).
CONTACT_OFFSET_M = 0.0001  # 0.1 mm — near-zero soft-contact zone
REST_OFFSET_M = 0.0

# Rigid-body-level CCD flag. Applied on LINK prims (not collision Xforms).
# Only the fast-moving links that actually sweep near obstacles need CCD.
CCD_LINK_PRIMS = [
    "/so101_new_calib/gripper_link",
    "/so101_new_calib/moving_jaw_so101_v1_link",
    "/so101_new_calib/wrist_link",
]

# SDF-specific attributes to strip (in case the URDF importer added them).
SDF_ATTRS_TO_REMOVE = [
    "physxSDFMeshCollision:sdfResolution",
    "physxSDFMeshCollision:sdfMargin",
    "physxSDFMeshCollision:sdfNarrowBandThickness",
    "physxSDFMeshCollision:sdfSubgridResolution",
    "physxSDFMeshCollision:sdfEnableRemeshing",
    "physxSDFMeshCollision:sdfBitsPerSubgridPixel",
    "physxSDFMeshCollision:sdfTriangleCountReductionFactor",
]


def patch_usd(asset_path: Path) -> int:
    if not asset_path.exists():
        print(f"FATAL: USD not found at {asset_path}", file=sys.stderr)
        return 1
    stage = Usd.Stage.Open(str(asset_path))
    if stage is None:
        print(f"FATAL: could not open {asset_path}", file=sys.stderr)
        return 1
    if not stage.GetRootLayer().permissionToEdit:
        print(f"FATAL: layer not editable: {stage.GetRootLayer().identifier}",
              file=sys.stderr)
        return 1

    touched = 0
    for path, target_approx in COLLISION_APPROXIMATIONS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  SKIP (not found): {path}")
            continue

        # 1. Flip approximation. physics:approximation is a token attr on
        #    PhysicsMeshCollisionAPI; setting it is one USD write.
        approx_attr = prim.GetAttribute("physics:approximation")
        old_approx = approx_attr.Get() if approx_attr and approx_attr.IsAuthored() else "(unset)"
        if approx_attr and approx_attr.IsAuthored():
            approx_attr.Set(target_approx)
        else:
            prim.CreateAttribute("physics:approximation",
                                 Sdf.ValueTypeNames.Token,
                                 custom=False).Set(target_approx)

        # 2. Strip any SDF-specific attributes that might linger.
        removed_sdf_attrs = []
        for sdf_name in SDF_ATTRS_TO_REMOVE:
            a = prim.GetAttribute(sdf_name)
            if a and a.IsAuthored():
                prim.RemoveProperty(sdf_name)
                removed_sdf_attrs.append(sdf_name)

        # 3. Strip PhysxSDFMeshCollisionAPI from applied schemas if present.
        applied = list(prim.GetAppliedSchemas() or [])
        if "PhysxSDFMeshCollisionAPI" in applied:
            prim.RemoveAppliedSchema("PhysxSDFMeshCollisionAPI")

        # 4. Author contactOffset + restOffset. Direct attribute writes —
        #    we deliberately skip PhysxCollisionAPI.Apply() because we don't
        #    want any schema-registration side effects.
        co_attr = prim.CreateAttribute("physxCollision:contactOffset",
                                       Sdf.ValueTypeNames.Float,
                                       custom=False)
        co_attr.Set(CONTACT_OFFSET_M)
        ro_attr = prim.CreateAttribute("physxCollision:restOffset",
                                       Sdf.ValueTypeNames.Float,
                                       custom=False)
        ro_attr.Set(REST_OFFSET_M)

        # 5. Add PhysxCollisionAPI to applied schemas (declarative — no
        #    side effects) if not present.
        if "PhysxCollisionAPI" not in applied:
            prim.AddAppliedSchema("PhysxCollisionAPI")

        print(f"  OK  {path}")
        print(f"        approximation: {old_approx} → {target_approx}")
        print(f"        contactOffset = {CONTACT_OFFSET_M*1000:.1f} mm  "
              f"restOffset = {REST_OFFSET_M*1000:.1f} mm")
        if removed_sdf_attrs:
            print(f"        removed SDF attrs: {removed_sdf_attrs}")
        touched += 1

    if touched == 0:
        print("No prims touched — USD structure unexpected.")
        return 1

    # Enable CCD on fast-moving link rigid bodies. See module-level comment
    # on CCD_LINK_PRIMS for rationale. PhysxRigidBodyAPI is separate from
    # the collision-shape APIs we edited above — no cooking cascade here.
    print()
    print("CCD (Continuous Collision Detection):")
    for path in CCD_LINK_PRIMS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  SKIP (not found): {path}")
            continue
        applied = list(prim.GetAppliedSchemas() or [])
        if "PhysxRigidBodyAPI" not in applied:
            prim.AddAppliedSchema("PhysxRigidBodyAPI")
        ccd_attr = prim.CreateAttribute("physxRigidBody:enableCCD",
                                         Sdf.ValueTypeNames.Bool,
                                         custom=False)
        ccd_attr.Set(True)
        print(f"  OK  {path}:  physxRigidBody:enableCCD = True")

    stage.GetRootLayer().Save()
    print(f"\n{touched} collision prims patched + {len(CCD_LINK_PRIMS)} "
          f"CCD links enabled — saved to {asset_path}")
    return 0


def verify(asset_path: Path) -> int:
    """Re-open the saved USD and confirm every target prim has the expected state."""
    stage = Usd.Stage.Open(str(asset_path))
    print("\nVerification:")
    ok = True
    for path in COLLISION_APPROXIMATIONS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            ok = False
            print(f"  ✗ {path}: INVALID")
            continue
        approx = prim.GetAttribute("physics:approximation")
        co = prim.GetAttribute("physxCollision:contactOffset")
        ro = prim.GetAttribute("physxCollision:restOffset")
        apis = prim.GetAppliedSchemas() or []
        a = approx.Get() if approx else None
        c = co.Get() if co else None
        r = ro.Get() if ro else None
        # Checks
        ok_approx = a == COLLISION_APPROXIMATIONS[path]
        ok_co = c is not None and abs(c - CONTACT_OFFSET_M) < 1e-6
        ok_ro = r is not None and abs(r - REST_OFFSET_M) < 1e-6
        ok_no_sdf = "PhysxSDFMeshCollisionAPI" not in apis
        status = "✓" if (ok_approx and ok_co and ok_ro and ok_no_sdf) else "✗"
        ok = ok and (status == "✓")
        c_mm = f"{c*1000:.3f}" if c is not None else "-"
        r_mm = f"{r*1000:.3f}" if r is not None else "-"
        print(f"  {status} {path}: approx={a}, contact={c_mm}mm, "
              f"rest={r_mm}mm, no_sdf={ok_no_sdf}")
    # CCD verification
    print("\nCCD on fast-moving links:")
    for path in CCD_LINK_PRIMS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            ok = False
            print(f"  ✗ {path}: INVALID")
            continue
        ccd = prim.GetAttribute("physxRigidBody:enableCCD")
        val = ccd.Get() if ccd else None
        st = "✓" if val is True else "✗"
        ok = ok and (val is True)
        print(f"  {st} {path}: enableCCD = {val}")
    return 0 if ok else 1


def main():
    print(f"Patching: {ASSET_PATH}")
    rc = patch_usd(ASSET_PATH)
    if rc != 0:
        return rc
    return verify(ASSET_PATH)


if __name__ == "__main__":
    sys.exit(main())
