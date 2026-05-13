#!/usr/bin/env python3
# Reference: https://openusd.org/release/spec_usdpreviewsurface.html
"""Wire UsdPreviewSurface for every unwired material in the split USDCs.

Root cause this addresses
-------------------------
The GLB → USDC splitter pipeline (`omni.kit.asset_converter` + offline
TRS-baker) emits MDL `gltf/pbr.mdl` shaders for each material but does NOT
connect the `Material.outputs:surface` port to the shader. RTX in
RaytracedLighting mode falls back to a dim flat-gray when no surface is
wired, regardless of light intensity hitting the surface.

Symptom: lit-but-dark WHITE top frame, dim back wall, "panel-like" floor —
all materials read flat even with high-intensity scene lighting.

Source-truth authoring contract
-------------------------------
**Do NOT read material values from the converted USDC's MDL shaders.** The
converter (`omni.kit.asset_converter`) is not faithful — observed lying:

- Invented `inputs:emissive_strength = 625.0` on Material.003 when the
  source GLB has plain `emissiveFactor=(1,1,1)` and NO
  `KHR_materials_emissive_strength` extension.
- Default value handling can diverge from glTF 2.0 spec (e.g. material
  `metallicFactor` defaults to 1.0 per spec when absent, NOT 0).

This script's `GAZEBO_PARITY_OVERRIDES` table below is the canonical
material authoring. The values mirror the upstream GLBs verbatim — verify
with `parse_glb_materials.py` (sibling script) before editing. **Every
edit to overrides must cite a source-GLB JSON value.**

It walks every UsdShade.Material in:
    - exts/aic-dt/assets/Enclosure/enclosure_split.usdc
    - exts/aic-dt/assets/Floor/walls_split.usdc

and authors a sibling `<Material>_UPS_Auto` UsdPreviewSurface shader using
the override values (or the MDL-extracted values as a fallback if no
override is listed — but PREFER explicit overrides). Materials already
wired with a custom UPS shader (e.g. GRAY75_CEILING_TRANSLUCENT,
Material_003 emissive) are skipped.

Run via:

    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/bake_all_material_surfaces.py

Idempotent.
"""
from __future__ import annotations

import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdShade

ASSETS = Path(__file__).resolve().parents[1] / "assets"
TARGETS = [
    ASSETS / "Enclosure" / "enclosure_split.usdc",
    ASSETS / "Floor" / "walls_split.usdc",
]

# Gazebo-parity diffuse overrides. The MDL `base_color_factor` values shipped
# in the GLBs are the physically-authored linear values — they're realistic
# but very dark (e.g. WHITE=(0.13,0.13,0.13) metallic=1.0 — a dark metallic
# alloy, NOT a white plastic). Gazebo's ogre2 over-exposes them so the scene
# reads as a brightly-lit competition cell. To match that visual character
# in RTX, we override the diffuse + clamp metallic so each material renders
# how it looks in Gazebo, not how it's physically authored. Keyed by
# Material prim name (NOT full path — same name in enclosure + walls
# get treated as the same override).
GAZEBO_PARITY_OVERRIDES: dict[str, dict[str, object]] = {
    # SOURCE-FAITHFUL authoring: values mirror the upstream GLB JSON exactly
    # (verified by parsing aic_assets/models/Enclosure/enclosure_visual.glb
    # and aic_assets/models/Floor/walls_visual.glb directly).
    #
    # WHITE in particular is labeled "WHITE" only as a CAD organization
    # convention — the actual baseColorFactor is dark (0.13, 0.13, 0.13)
    # with metallicFactor defaulting to 1.0 (glTF 2.0 default when absent).
    # So WHITE is a dark-gray METAL, not a white plastic.
    #
    # SOURCE-TRUE (option ii, user-confirmed 2026-05-13). Values mirror the
    # source GLB JSON exactly (verified via parse_glb_materials.py). RTX
    # will render the WHITE metallic darker than Gazebo's ogre2 does — that
    # divergence is a renderer-pipeline difference, not an authoring one.
    "WHITE":        {"diffuse": (0.13, 0.13, 0.13), "metallic": 1.0, "roughness": 0.5},
    "GRAY75":       {"diffuse": (0.29, 0.29, 0.29), "metallic": 0.0, "roughness": 0.4},
    # Warehouse walls + floor (model://Floor) — source GLB values verbatim.
    "Material_001": {"diffuse": (0.26, 0.26, 0.26), "metallic": 0.0, "roughness": 0.2},
    "Material_004": {"diffuse": (0.21, 0.21, 0.21), "metallic": 0.0, "roughness": 0.2},
    "Material_005": {"diffuse": (0.02, 0.02, 0.02), "metallic": 0.0, "roughness": 0.5},
}


def patch_material(material: UsdShade.Material) -> str:
    """Return "skipped" / "patched" describing what happened."""
    surf = material.GetSurfaceOutput()
    if surf and surf.GetConnectedSource() is not None:
        # If the existing connection is NOT to our UPS_Auto shader, leave it
        # alone — e.g. GRAY75_CEILING_TRANSLUCENT and Material_003 emissive
        # already have hand-crafted UPS shaders.
        connected = surf.GetConnectedSource()
        if connected is not None:
            connected_path = str(connected[0].GetPath())
            if not connected_path.endswith("/UPS_Auto"):
                return "skipped (custom-wired UPS)"
        # Otherwise it's our auto-shader — fall through and update inputs.

    # Find the existing MDL shader child (first one with gltf/pbr.mdl source).
    mdl_shader = None
    for c in material.GetPrim().GetChildren():
        if not c.IsA(UsdShade.Shader):
            continue
        src_asset = c.GetAttribute("info:mdl:sourceAsset").Get()
        if src_asset and "pbr.mdl" in str(src_asset):
            mdl_shader = UsdShade.Shader(c)
            break

    diffuse = (0.8, 0.8, 0.8)
    roughness = 0.5
    metallic = 0.0
    if mdl_shader is not None:
        v = mdl_shader.GetPrim().GetAttribute("inputs:base_color_factor").Get()
        if v is not None:
            diffuse = (float(v[0]), float(v[1]), float(v[2]))
        rv = mdl_shader.GetPrim().GetAttribute("inputs:roughness_factor").Get()
        if rv is not None:
            roughness = float(rv)
        mv = mdl_shader.GetPrim().GetAttribute("inputs:metallic_factor").Get()
        if mv is not None:
            metallic = float(mv)

    # Override with Gazebo-parity preset if one exists for this material name.
    mat_name = material.GetPrim().GetName()
    override = GAZEBO_PARITY_OVERRIDES.get(mat_name)
    if override is not None:
        diffuse = override.get("diffuse", diffuse)
        roughness = override.get("roughness", roughness)
        metallic = override.get("metallic", metallic)

    stage = material.GetPrim().GetStage()
    ups_path = material.GetPath().AppendChild("UPS_Auto")
    ups_prim = stage.GetPrimAtPath(ups_path)
    if ups_prim and ups_prim.IsValid():
        ups = UsdShade.Shader(ups_prim)
    else:
        ups = UsdShade.Shader.Define(stage, ups_path)
    ups.CreateIdAttr("UsdPreviewSurface")
    ups.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    ups.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    ups.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    ups.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)
    ups.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    surf_output = material.CreateSurfaceOutput()
    surf_output.ConnectToSource(ups.ConnectableAPI(), "surface")
    return (
        f"patched (diffuse={diffuse} rough={roughness:.2f} metal={metallic:.2f})"
    )


def patch_stage(path: Path) -> None:
    print(f"\n== {path.name} ==")
    if not path.exists():
        print(f"  [SKIP] {path} not found")
        return
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        print(f"  [ERR] failed to open {path}")
        sys.exit(2)
    changes = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material):
            result = patch_material(UsdShade.Material(prim))
            print(f"  {prim.GetPath()}: {result}")
            if result.startswith("patched"):
                changes += 1
    if changes:
        stage.GetRootLayer().Save()
        print(f"  -> saved ({changes} materials patched)")
    else:
        print(f"  -> no changes")


def main() -> None:
    for target in TARGETS:
        patch_stage(target)


if __name__ == "__main__":
    main()
