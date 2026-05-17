#!/usr/bin/env python3
# Reference: built per Phase 1 Plan 09 (SCENE-01) as build_mount_rail_usds.py
# for 3 mount-rail targets; 2026-05-13 (51a7267) extended to all 7 task-board-
# part visuals; 2026-05-17 (b059074) extended to the 3 cable plugs (SC Plug,
# SFP Module, LC Plug); 2026-05-17 renamed build_mount_rail_usds.py →
# build_thin_glb_usds.py to match the actual scope.
#
# Replaces omni.kit.asset_converter outputs (the .usd files that shipped with
# rotateX:unitsResolve+scale:unitsResolve xform ops baked into the wrapping
# Xform, which composed wrong against spawn-atom RPYs for SC Port / NIC Card /
# Task Board Base / NIC Card Mount). The thin-USD-references-GLB pattern
# delegates axis/units handling to the gltf SDF plugin at load time and emits
# regular xform ops instead, matching how the working LC/SFP/SC mount rails
# load. Pattern follows the Plan 02 vendoring contract (capitalized folders,
# sibling textures preserved) and uses the `_local_asset` resolver shape
# downstream extension code expects.
"""
Author thin USD wrappers around any GLB-source visual asset.

The AIC source folders (~/Documents/aic/aic_assets/models/<X>/) ship
.glb / .dae / .stl meshes — there are no pre-cooked USDs upstream. This
script:
  1. Vendors any source GLB that's missing from the extension's
     assets/assets/<X>/ folder (copies from AIC repo, creating the
     target folder on-demand for new TARGETS).
  2. Emits one tiny USD per (folder, output_usd, source_glb) tuple that
     references the target GLB via a relative AddReference, so the
     extension's existing `_local_asset(...) + Sdf payload` flow loads it
     identically to the Plan-02 vendored mount-rail USDs.

Coverage as of 2026-05-17 (11 TARGETS):
  - 3 mount rails (LC / SFP / SC Mount)                  — Plan 09 original
  - 5 task-board parts (SC Port, NIC Card, Task Board    — 51a7267 extension
    Base, NIC Card Mount + NIC Card child under Mount)
  - 3 cable plugs (SC Plug, SFP Module, LC Plug)         — b059074 extension

The TARGETS list is explicit (folder, output_usd, source_glb) — necessary
because folders like "NIC Card Mount" contain multiple GLBs and we need
to map each output USD to the right source.

Usage (run with Isaac Sim's bundled python that ships pxr):
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/build_thin_glb_usds.py exts/aic-dt/assets/assets

Or any python with the `usd-core` package installed.
"""
from __future__ import annotations

import os
import shutil
import sys


# (folder, output_usd_filename, source_glb_filename)
# Explicit glb name disambiguates folders with multiple GLBs (NIC Card Mount).
TARGETS = [
    # Existing working pattern (mount rails) — re-authoring is idempotent.
    ("LC Mount",         "lc_mount_visual.usd",         "lc_mount_visual.glb"),
    ("SFP Mount",        "sfp_mount_visual.usd",        "sfp_mount_visual.glb"),
    ("SC Mount",         "sc_mount_visual.usd",         "sc_mount_visual.glb"),
    # Newly converted to thin-USD pattern (2026-05-13 replace asset_converter
    # outputs that shipped with unitsResolve ops). AIC_OBJECTS["…"]["usd"]
    # entries in extension.py point at these exact USD names.
    ("SC Port",          "sc_port.usd",                 "sc_port_visual.glb"),
    ("NIC Card",         "nic_card.usd",                "nic_card_visual.glb"),
    ("Task Board Base",  "task_board_rigid.usd",        "base_visual.glb"),
    ("NIC Card Mount",   "nic_card_mount_visual.usd",   "nic_card_mount_visual.glb"),
    # PCB child asset referenced separately by _cmd_spawn_nic_card_mount
    # (composed under each NICCardMount_<i>/nic_card_link/). AIC ships
    # nic_card_visual.glb inside BOTH the NIC Card and NIC Card Mount
    # folders; here we point at the one in NIC Card Mount.
    ("NIC Card Mount",   "nic_card_visual.usd",         "nic_card_visual.glb"),
    # Plug visuals (2026-05-17 visual-parity fix). The connector subtrees
    # under /World/cable/{sc_plug_visual, sfp_module_visual} in the cable USD
    # were inlined with empty Looks scopes (sc_plug) or hand-authored
    # UsdPreviewSurface (sfp_module) — neither carried the GLB-side baseColor
    # textures the AIC stack expects. The fix is to give each plug its own
    # thin GLB-reference USD (same pattern as the sockets), then have
    # build_cable_variant_usds.py replace the inline subtrees with references
    # to these thin USDs. The gltf SDF plugin then handles axis/units +
    # material textures + bindings at load time, restoring full Gazebo parity.
    ("SC Plug",          "sc_plug_visual.usd",          "sc_plug_visual.glb"),
    ("SFP Module",       "sfp_module_visual.usd",       "sfp_module_visual.glb"),
    ("LC Plug",          "lc_plug_visual.usd",          "lc_plug_visual.glb"),
]

# Source GLBs not yet vendored under the extension are copied from here.
AIC_SOURCE = os.path.expanduser("~/Documents/aic/aic_assets/models")


def vendor_source_glb(folder_path: str, glb_name: str) -> None:
    """Copy <glb_name> from the AIC source repo if missing locally.

    Idempotent: if the GLB is already present at folder_path/<glb_name>,
    this is a no-op. Raises RuntimeError if the source GLB is missing in
    AIC's repo (caller catches + logs).
    """
    dst = os.path.join(folder_path, glb_name)
    if os.path.exists(dst):
        return
    folder_name = os.path.basename(folder_path)
    src = os.path.join(AIC_SOURCE, folder_name, glb_name)
    if not os.path.exists(src):
        raise RuntimeError(
            f"AIC source GLB missing: {src} "
            f"(cannot vendor {glb_name} into {folder_path})"
        )
    shutil.copyfile(src, dst)
    sz = os.path.getsize(dst)
    print(f"[build_thin_glb_usds] vendored {glb_name} ({sz} B) "
          f"from AIC source → {folder_path}")


def author_mount_usd(folder: str, usd_name: str, glb_name: str = None) -> str:
    """Author a tiny USD at folder/usd_name that references glb_name.

    If glb_name is None, falls back to the legacy "first mesh by extension"
    heuristic (prefer .dae > .glb > .stl > .obj). Pass the explicit glb_name
    when the folder contains multiple meshes.

    Returns the absolute path on success; raises on failure (caller catches).
    """
    from pxr import Usd, UsdGeom

    usd_path = os.path.join(folder, usd_name)

    if glb_name is None:
        # Legacy heuristic — prefer .dae (carries materials) > .glb > .stl > .obj
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
        glb_name = candidates[0]
    else:
        if not os.path.exists(os.path.join(folder, glb_name)):
            raise RuntimeError(f"Specified mesh {glb_name} not found in {folder}")

    stage = Usd.Stage.CreateNew(usd_path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    mesh_prim = stage.DefinePrim("/Root/Mesh", "Xform")
    # Relative path resolves against the USD layer location at load time —
    # since the mesh sits in the same vendored folder, this stays valid as
    # long as the folder is moved as a unit (Plan 02 D-05 vendoring contract).
    mesh_prim.GetReferences().AddReference(f"./{glb_name}")
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()
    print(f"[build_thin_glb_usds] wrote {usd_path} "
          f"({os.path.getsize(usd_path)} B) referencing {glb_name}")
    return usd_path


def main(argv):
    base = argv[1] if len(argv) > 1 else "exts/aic-dt/assets/assets"
    written = 0
    for folder_name, usd_name, glb_name in TARGETS:
        folder = os.path.join(base, folder_name)
        # Create the vendored folder on-demand. New plug TARGETS (SC Plug,
        # SFP Module, LC Plug) don't have a pre-existing vendored folder
        # under exts/aic-dt/assets/assets/ — the GLB and thin USD are both
        # authored fresh by this script run.
        if not os.path.isdir(folder):
            os.makedirs(folder, exist_ok=True)
            print(f"[build_thin_glb_usds] created vendored folder: {folder}")
        try:
            vendor_source_glb(folder, glb_name)
            author_mount_usd(folder, usd_name, glb_name)
            written += 1
        except Exception as exc:  # noqa: BLE001 — degrade gracefully per plan
            print(f"[build_thin_glb_usds] WARN {folder_name}/{usd_name}: {exc}")
    print(f"[build_thin_glb_usds] done — {written}/{len(TARGETS)} USDs authored")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
