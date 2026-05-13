#!/usr/bin/env python3
# Reference: https://openusd.org/release/spec_usdpreviewsurface.html
"""Bake matte UsdPreviewSurface override for the warehouse floor.

The splitter pipeline emits floor_split.usdc with a MDL `gltf/pbr.mdl` shader
authored on /FloorVisual/Looks/Material_001 but with no Material.outputs:surface
connection. The MDL `roughness_factor=0.22` is too glossy for parity with
Gazebo's matte concrete look — Isaac's floor reads "mirror-like" in side-by-side
audits.

This script authors a sibling UsdPreviewSurface on the same Material with
roughness=0.9 (matte) and connects Material.outputs:surface to it so RTX uses
it. The base color texture is preserved by referencing the existing
baseColorTex output via inputs:diffuseColor.

Run via:

    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/bake_floor_material.py

Re-running is idempotent.
"""
from __future__ import annotations

import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdShade

FLOOR_USDC = (
    Path(__file__).resolve().parents[1] / "assets" / "Floor" / "floor_split.usdc"
)
MATERIAL_PATH = "/FloorVisual/Looks/Material_001"
UPS_SHADER_NAME = "Matte_UPS"
ST_READER_NAME = "stReader"
TEX_SHADER_NAME = "diffuseTex"
ROUGHNESS = 0.9
TEXTURE_FILE = "./textures/clean-concrete_albedo.png"
TEXTURE_FALLBACK_RGB = (0.55, 0.55, 0.55)


def bake() -> None:
    if not FLOOR_USDC.exists():
        print(f"[ERR] {FLOOR_USDC} not found", file=sys.stderr)
        sys.exit(1)

    stage = Usd.Stage.Open(str(FLOOR_USDC))
    mat_prim = stage.GetPrimAtPath(MATERIAL_PATH)
    if not mat_prim or not mat_prim.IsValid():
        print(f"[ERR] {MATERIAL_PATH} not found", file=sys.stderr)
        sys.exit(2)
    material = UsdShade.Material(mat_prim)

    # PrimvarReader for `st` (UV coords). UsdPreviewSurface expects the
    # texture coords to come through a UsdPrimvarReader_float2.
    st_path = f"{MATERIAL_PATH}/{ST_READER_NAME}"
    st_prim = stage.GetPrimAtPath(st_path)
    if st_prim and st_prim.IsValid():
        st = UsdShade.Shader(st_prim)
    else:
        st = UsdShade.Shader.Define(stage, st_path)
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    # UsdUVTexture reading the existing clean-concrete_albedo.png.
    tex_path = f"{MATERIAL_PATH}/{TEX_SHADER_NAME}"
    tex_prim = stage.GetPrimAtPath(tex_path)
    if tex_prim and tex_prim.IsValid():
        tex = UsdShade.Shader(tex_prim)
    else:
        tex = UsdShade.Shader.Define(stage, tex_path)
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(TEXTURE_FILE)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    # v7 baseline: no scale tint (texture rgb used as authored). The 0.75x
    # tint that v8 added was part of the regression that collapsed the scene.
    tex.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(*TEXTURE_FALLBACK_RGB, 1.0)
    )
    tex_st_in = tex.CreateInput("st", Sdf.ValueTypeNames.Float2)
    tex_st_in.ConnectToSource(st.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader_path = f"{MATERIAL_PATH}/{UPS_SHADER_NAME}"
    shader_prim = stage.GetPrimAtPath(shader_path)
    if shader_prim and shader_prim.IsValid():
        shader = UsdShade.Shader(shader_prim)
    else:
        shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    diffuse_in = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    diffuse_in.ConnectToSource(tex.ConnectableAPI(), "rgb")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(ROUGHNESS)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    surf_output = material.CreateSurfaceOutput()
    surf_output.ConnectToSource(shader.ConnectableAPI(), "surface")

    stage.GetRootLayer().Save()
    print(f"[OK] Baked matte floor material into {FLOOR_USDC.name}:")
    print(f"     {MATERIAL_PATH}/{UPS_SHADER_NAME} (roughness={ROUGHNESS})")


if __name__ == "__main__":
    bake()
