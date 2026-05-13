#!/usr/bin/env python3
# Reference: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#materials
"""Parse a glTF/GLB and dump its materials with glTF 2.0 spec defaults applied.

USE THIS AS THE SOURCE OF TRUTH for material authoring — NOT the MDL shader
values that `omni.kit.asset_converter` produces.

Why this matters
----------------
The Isaac Sim GLB → USD pipeline (`omni.kit.asset_converter`) does NOT
faithfully preserve source material values:

- It can INVENT `inputs:emissive_strength` (we observed `625.0` written for
  Material.003 even though the source GLB has plain `emissiveFactor=(1,1,1)`
  and NO `KHR_materials_emissive_strength` extension).
- It transcodes glTF materials into `gltf/pbr.mdl` shaders whose inputs
  read like the source but with converter-side defaulting that can drift.

Symptoms when authoring USD overrides from MDL-extracted values:
- Materials render with surreal emissive intensities or wrong roughness.
- "WHITE" labeled materials don't render as white because the source GLB
  has them as a dark-metallic CAD-organizational name.
- "Translucent" panels that aren't translucent in the source GLB.

glTF 2.0 spec gotchas applied here
----------------------------------
Material default values (per spec) when the corresponding key is absent:

- `pbrMetallicRoughness.baseColorFactor`   → (1, 1, 1, 1)
- `pbrMetallicRoughness.metallicFactor`    → 1.0   ← non-obvious; default IS metallic
- `pbrMetallicRoughness.roughnessFactor`   → 1.0
- `emissiveFactor`                          → (0, 0, 0)
- `alphaMode`                              → "OPAQUE"
- `alphaCutoff`                            → 0.5  (only when alphaMode=MASK)
- `doubleSided`                            → false

CLI usage
---------

    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/parse_glb_materials.py <path-to.glb> [<more.glb> …]

Output is a JSON document on stdout listing every material in every GLB with
defaults applied. Pipe through `jq` to grep specific materials.
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path


GLTF_DEFAULTS = {
    "baseColorFactor": (1.0, 1.0, 1.0, 1.0),
    "metallicFactor": 1.0,
    "roughnessFactor": 1.0,
    "emissiveFactor": (0.0, 0.0, 0.0),
    "alphaMode": "OPAQUE",
    "alphaCutoff": 0.5,
    "doubleSided": False,
}


def parse_glb(path: Path) -> dict:
    """Return the JSON chunk of a binary glTF file."""
    blob = path.read_bytes()
    magic, version, _ = struct.unpack("<4sII", blob[:12])
    if magic != b"glTF":
        raise ValueError(f"{path}: not a binary glTF (magic={magic!r})")
    if version != 2:
        raise ValueError(f"{path}: glTF version {version}, expected 2")
    c0_len, c0_type = struct.unpack("<I4s", blob[12:20])
    if c0_type != b"JSON":
        raise ValueError(f"{path}: first chunk type {c0_type!r}, expected JSON")
    return json.loads(blob[20 : 20 + c0_len])


def material_with_defaults(mat: dict) -> dict:
    """Return a copy of a glTF material with all spec defaults filled in."""
    pbr_src = mat.get("pbrMetallicRoughness", {})
    pbr = {
        "baseColorFactor": tuple(pbr_src.get("baseColorFactor", GLTF_DEFAULTS["baseColorFactor"])),
        "metallicFactor": pbr_src.get("metallicFactor", GLTF_DEFAULTS["metallicFactor"]),
        "roughnessFactor": pbr_src.get("roughnessFactor", GLTF_DEFAULTS["roughnessFactor"]),
    }
    out = {
        "name": mat.get("name", ""),
        "pbrMetallicRoughness": pbr,
        "emissiveFactor": tuple(mat.get("emissiveFactor", GLTF_DEFAULTS["emissiveFactor"])),
        "alphaMode": mat.get("alphaMode", GLTF_DEFAULTS["alphaMode"]),
        "alphaCutoff": mat.get("alphaCutoff", GLTF_DEFAULTS["alphaCutoff"]),
        "doubleSided": mat.get("doubleSided", GLTF_DEFAULTS["doubleSided"]),
        "extensions": mat.get("extensions", {}),
    }
    return out


def parse_file(path: Path) -> dict:
    gltf = parse_glb(path)
    return {
        "path": str(path),
        "extensionsUsed": gltf.get("extensionsUsed", []),
        "extensionsRequired": gltf.get("extensionsRequired", []),
        "materials": [material_with_defaults(m) for m in gltf.get("materials", [])],
    }


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    results = [parse_file(Path(p)) for p in argv[1:]]
    json.dump(results, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
