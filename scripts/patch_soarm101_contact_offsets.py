#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
Offline USD patch — authors explicit PhysX contactOffset + restOffset on the
SO-ARM101 collision Xforms so PhysX does NOT fall back to its 20 mm default
contactOffset, which causes visible cup knocks (200-290 mm lateral
displacement) during pick/place sweeps.

WHY OFFLINE AND NOT RUNTIME:
----------------------------
The gripper_link + moving_jaw_so101_v1_link Xforms have PhysxSDFMeshCollisionAPI
applied at SDF resolution 256 (see SO-ARM101-USD.usd). Authoring ANY collision
attribute at runtime on those prims invalidates PhysX's cooked collision cache,
forcing a voxelizer re-cook. Observed peak: ~48 GB RAM, causing a workstation
OOM hang (2026-04-23). Authoring offline lets PhysX read the attribute during
its initial cook — zero regen cost.

Run once after checkout, or after any regeneration of the source USD:

    python3 scripts/patch_soarm101_contact_offsets.py

Idempotent. Safe to re-run. Verifies the attribute is set before exiting.
"""

import os
import sys
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics
except ImportError:
    print("FATAL: pxr not importable. Source Isaac Sim's Python env or install usd-core.",
          file=sys.stderr)
    sys.exit(1)

# Path to the extension-local asset USD
REPO_ROOT = Path(__file__).resolve().parent.parent
ASSET_PATH = REPO_ROOT / "exts/soarm101-dt/assets/SO-ARM101-USD.usd"

# Physics constants — see DEBUG-GUIDE.md § 4.3 for derivation
CONTACT_OFFSET_M = 0.002  # 2 mm — tightens from PhysX default 20 mm
REST_OFFSET_M = 0.0       # shapes touch at zero geometric separation

# Collision Xforms to patch, listed in sweep-probability order
# (path in the referenced USD — /so101_new_calib/<link>/collisions)
COLLISION_PRIMS = [
    "/so101_new_calib/gripper_link/collisions",
    "/so101_new_calib/moving_jaw_so101_v1_link/collisions",
    "/so101_new_calib/wrist_link/collisions",
    "/so101_new_calib/lower_arm_link/collisions",
    "/so101_new_calib/upper_arm_link/collisions",
    "/so101_new_calib/shoulder_link/collisions",
    "/so101_new_calib/base_link/collisions",
]


def patch_usd(asset_path: Path) -> int:
    if not asset_path.exists():
        print(f"FATAL: USD not found at {asset_path}", file=sys.stderr)
        return 1

    stage = Usd.Stage.Open(str(asset_path))
    if stage is None:
        print(f"FATAL: could not open {asset_path}", file=sys.stderr)
        return 1

    # Layer where we'll author. Open with original layer for permanent edit.
    rootLayer = stage.GetRootLayer()
    if rootLayer.permissionToEdit is False:
        print(f"FATAL: layer not editable: {rootLayer.identifier}", file=sys.stderr)
        return 1

    patched = 0
    skipped = 0
    for path in COLLISION_PRIMS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  SKIP: {path} (not found)")
            skipped += 1
            continue
        # Direct attribute writes — avoid calling .Apply() on the schema,
        # which can cascade into PhysX cooked-cache invalidation when the
        # stage is later loaded by Isaac Sim. We author the attribute directly.
        co_attr = prim.CreateAttribute("physxCollision:contactOffset",
                                        _sdf_type_float(), custom=False)
        ro_attr = prim.CreateAttribute("physxCollision:restOffset",
                                        _sdf_type_float(), custom=False)
        co_attr.Set(CONTACT_OFFSET_M)
        ro_attr.Set(REST_OFFSET_M)
        # Also add the API schema metadata so UsdPrim.HasAPI() reports True
        # without calling PhysxCollisionAPI.Apply() (which could cascade).
        _add_api_schema(prim, "PhysxCollisionAPI")
        print(f"  OK  : {path}")
        patched += 1

    if patched == 0:
        print("No prims patched — check the USD structure.")
        return 1

    stage.GetRootLayer().Save()
    print(f"\n{patched} patched / {skipped} skipped / saved to {asset_path}")
    return 0


def _sdf_type_float():
    """Return the Sdf.ValueTypeName for a float attribute."""
    from pxr import Sdf
    return Sdf.ValueTypeNames.Float


def _add_api_schema(prim, api_name: str):
    """Append `api_name` to the prim's `apiSchemas` metadata if not present.
    This is the declarative equivalent of schema.Apply() without running
    the schema's Python side-effects (which can cascade into PhysX cooking
    triggers on live stages — not a risk here since we're offline, but
    keeps the pattern consistent)."""
    cur = prim.GetAppliedSchemas()
    if api_name in cur:
        return
    prim.AddAppliedSchema(api_name)


def verify(asset_path: Path) -> int:
    """Re-open the saved USD and confirm every target prim has the attrs set
    to the expected values."""
    stage = Usd.Stage.Open(str(asset_path))
    ok = True
    for path in COLLISION_PRIMS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        co = prim.GetAttribute("physxCollision:contactOffset")
        ro = prim.GetAttribute("physxCollision:restOffset")
        co_val = co.Get() if co else None
        ro_val = ro.Get() if ro else None
        status = "✓"
        if co_val is None or abs(co_val - CONTACT_OFFSET_M) > 1e-6:
            status = "✗"; ok = False
        if ro_val is None or abs(ro_val - REST_OFFSET_M) > 1e-6:
            status = "✗"; ok = False
        print(f"  {status} {path}: contact={co_val} rest={ro_val}")
    return 0 if ok else 1


def main():
    print(f"Patching: {ASSET_PATH}")
    rc = patch_usd(ASSET_PATH)
    if rc != 0:
        return rc
    print(f"\nVerifying:")
    return verify(ASSET_PATH)


if __name__ == "__main__":
    sys.exit(main())
