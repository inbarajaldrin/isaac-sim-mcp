#!/usr/bin/env python3
# Reference: https://openusd.org/release/spec_usdpreviewsurface.html
"""Bake Material_003 emissive ceiling + Plane_003 doubleSided into walls_split.usdc.

Background
----------
walls_split.usdc is the splitter output for `walls_visual.glb` (the AIC enclosure
wall geometry from Gazebo's `model://Floor`). The glTF importer preserves MDL
inputs (`emissive_factor`, `emissive_strength=625`) on Material_003 but never
wires the `Material.outputs:surface` connection to the Shader — so RTX renders
the material as a fallback neutral gray and the ceiling is *not* emissive.

In Gazebo, Material_003 is the emissive ceiling panel that lights the underside
of the WHITE top frame from inside the enclosure. Per-session bisection
confirmed that the case-height-look-off divergence in Isaac Sim is a lighting
deficit, not a geometry deficit.

What this script does
---------------------
1. For Material_003: add a sibling UsdPreviewSurface shader with a tuned
   `emissiveColor` (linear HDR RGB) and connect Material.outputs:surface to it.
2. Set /WallsVisual/Plane_003_Plane_003.doubleSided = True so the inner face
   (facing the camera below) emits light (single-sided meshes are dark on the
   back face).

Run via Isaac Sim's pxr-bundled python (cp310 + pxr 0.26.x):

    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/bake_walls_emissive.py

Re-run is idempotent — if the UsdPreviewSurface child already exists it is
just updated in place.
"""
from __future__ import annotations

import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

WALLS_USDC = Path(__file__).resolve().parents[1] / "assets" / "Floor" / "walls_split.usdc"

EMISSIVE_RGB = (1.0, 1.0, 1.0)  # Source GLB Material.003 emissiveFactor=(1,1,1), NO KHR_materials_emissive_strength extension. The earlier 1.5 / 3 / 625 values were invented by Isaac's omni.kit.asset_converter — source is plain glTF unit emission.
DIFFUSE_RGB = (0.8, 0.8, 0.8)  # Matches source GLB baseColorFactor.
CEILING_MESH_PATH = "/WallsVisual/Plane_003_Plane_003"
EMISSIVE_MATERIAL_PATH = "/WallsVisual/Looks/Material_003"
UPS_SHADER_NAME = "Emissive_UPS"


def bake() -> None:
    if not WALLS_USDC.exists():
        print(f"[ERR] {WALLS_USDC} not found", file=sys.stderr)
        sys.exit(1)

    stage = Usd.Stage.Open(str(WALLS_USDC))
    if stage is None:
        print(f"[ERR] failed to open {WALLS_USDC}", file=sys.stderr)
        sys.exit(2)

    mat_prim = stage.GetPrimAtPath(EMISSIVE_MATERIAL_PATH)
    if not mat_prim or not mat_prim.IsValid():
        print(f"[ERR] {EMISSIVE_MATERIAL_PATH} not found", file=sys.stderr)
        sys.exit(3)
    material = UsdShade.Material(mat_prim)

    shader_path = f"{EMISSIVE_MATERIAL_PATH}/{UPS_SHADER_NAME}"
    shader_prim = stage.GetPrimAtPath(shader_path)
    if shader_prim and shader_prim.IsValid():
        shader = UsdShade.Shader(shader_prim)
    else:
        shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*DIFFUSE_RGB))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*EMISSIVE_RGB))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    surf_output = material.CreateSurfaceOutput()
    surf_output.ConnectToSource(shader.ConnectableAPI(), "surface")

    mesh_prim = stage.GetPrimAtPath(CEILING_MESH_PATH)
    if not mesh_prim or not mesh_prim.IsValid():
        print(f"[ERR] {CEILING_MESH_PATH} not found", file=sys.stderr)
        sys.exit(4)
    mesh = UsdGeom.Mesh(mesh_prim)
    mesh.CreateDoubleSidedAttr(True)

    stage.GetRootLayer().Save()
    print(f"[OK] Baked emissive into {WALLS_USDC.name}:")
    print(f"     {EMISSIVE_MATERIAL_PATH}/{UPS_SHADER_NAME} (emissiveColor={EMISSIVE_RGB})")
    print(f"     {CEILING_MESH_PATH}.doubleSided=True")


if __name__ == "__main__":
    bake()
