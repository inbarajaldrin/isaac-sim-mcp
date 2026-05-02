#!/usr/bin/env python3
# Reference: built per Phase 1 Plan 09 (SCENE-01) — emits thin USD wrappers
# for LC/SFP/SC Mount Rail components vendored from
# ~/Documents/aic/aic_assets/models/<X> Mount/. Pattern follows the Plan 02
# vendoring contract (capitalized folders, sibling textures preserved) and
# uses the same `_local_asset` resolver shape downstream extension code expects.
"""
Author thin USD wrappers around LC/SFP/SC Mount mesh assets.

The AIC source folders (~/Documents/aic/aic_assets/models/<X> Mount/) ship
.glb / .dae / .stl meshes — there are no pre-cooked USDs upstream. This
script emits one tiny USD per folder that references the mesh via a
relative AddReference, so the extension's existing `_local_asset(...)
+ Sdf payload` flow loads it the same way the Plan-02 vendored USDs load.

If the source folder lacks a mesh, the per-folder author step prints a
WARN and continues — Task 2's per-component spawn atom degrades gracefully
when the resulting USD is missing.

Usage (run with Isaac Sim's bundled python that ships pxr):
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/build_mount_rail_usds.py exts/aic-dt/assets/assets

Or any python with the `usd-core` package installed.
"""
from __future__ import annotations

import os
import sys


def author_mount_usd(folder: str, usd_name: str) -> str:
    """Author a tiny USD that references the first mesh found in `folder`.

    Returns the absolute path to the authored USD on success; raises on
    failure. Caller should catch and log to keep the loop tolerant.
    """
    from pxr import Usd, UsdGeom

    usd_path = os.path.join(folder, usd_name)
    # Prefer .dae (carries materials) > .glb > .stl > .obj
    candidates = []
    for ext in (".dae", ".glb", ".stl", ".obj"):
        candidates.extend(
            sorted(
                f for f in os.listdir(folder)
                if f.lower().endswith(ext)
            )
        )
    if not candidates:
        raise RuntimeError(f"No mesh (.dae/.glb/.stl/.obj) found in {folder}")
    mesh_src = candidates[0]

    stage = Usd.Stage.CreateNew(usd_path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    mesh_prim = stage.DefinePrim("/Root/Mesh", "Xform")
    # Relative path resolves against the USD layer location at load time —
    # since the mesh sits in the same vendored folder, this stays valid as
    # long as the folder is moved as a unit (Plan 02 D-05 vendoring contract).
    mesh_prim.GetReferences().AddReference(f"./{mesh_src}")
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()
    print(f"[build_mount_rail_usds] wrote {usd_path} referencing {mesh_src}")
    return usd_path


def main(argv):
    base = argv[1] if len(argv) > 1 else "exts/aic-dt/assets/assets"
    targets = [
        ("LC Mount", "lc_mount_visual.usd"),
        ("SFP Mount", "sfp_mount_visual.usd"),
        ("SC Mount", "sc_mount_visual.usd"),
    ]
    written = 0
    for name, usd_name in targets:
        folder = os.path.join(base, name)
        if not os.path.isdir(folder):
            print(f"[build_mount_rail_usds] WARN folder missing: {folder}")
            continue
        try:
            author_mount_usd(folder, usd_name)
            written += 1
        except Exception as exc:  # noqa: BLE001 — degrade gracefully per plan
            print(f"[build_mount_rail_usds] WARN {name}: {exc}")
    print(f"[build_mount_rail_usds] done — {written}/{len(targets)} USDs authored")
    # Exit 0 even on partial — Plan 09 Task 1 acceptance allows >= 2/3 success;
    # Task 2's atoms degrade gracefully when a USD is missing.
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
