#!/usr/bin/env python3
"""Generate a thin USDA wrapper that references one or more GLB files.

The whole point: when you `add_reference_to_stage(usd_path="<thing>.usda")`
in Isaac Sim, the wrapper's per-child `xformOp:scale` and `xformOp:rotateXYZ`
can correct two common GLB authoring quirks WITHOUT touching the GLB itself:

    1. mm authored as m   → scale = (0.001, 0.001, 0.001)
    2. Y-up GLB on Z-up stage → rotateXYZ = (90, 0, 0)
       (maps +Y → +Z, +Z → -Y; OpenUSD's positive X-rotation)

Why this matters for the AIC project (and any GLB-heavy import):
  - Gazebo's gz-sim auto-detects + corrects upAxis at load time per-mesh.
  - Isaac Sim's gltf SDF plugin imports the GLB verbatim — no implicit
    correction. So a Y-up + mm GLB referenced into a Z-up + m USD stage
    will render rotated 90° AND 1000× too big.
  - The fix is in the wrapper, NOT the GLB (keeps source files clean and
    re-importable from Gazebo). This also keeps the wrapper diffable.

DETECTION:
  The script probes each GLB's bbox after referencing in a throwaway USD
  stage. Heuristics:
    - any-dim > 100 → likely mm (scale 0.001)
    - height dim is Y, not Z → likely Y-up (rotateX 90)
  These are HEURISTICS, not certainty. The probe results + chosen scale +
  rotation are printed; review before committing the wrapper.

USAGE:
    python3 glb_to_usd_wrapper.py \\
        --output     /path/to/wrapper.usda \\
        --default-prim Floor \\
        --glb floor_visual.glb \\
        --glb walls_visual.glb \\
        --auto-detect

    python3 glb_to_usd_wrapper.py \\
        --output       /path/to/wrapper.usda \\
        --default-prim Enclosure \\
        --glb enclosure_visual.glb=scale=0.001,rotateX=90 \\
        --glb enclosurewalls_visual.glb       # walls already in m + Z-up

  GLB paths are relative to the wrapper file's directory.

DIMENSION CROSS-CHECK:
  After authoring, the script prints the corrected bbox per child. Verify
  against your real-world dimensions (e.g. for the AIC enclosure: expected
  ~1.5 × 1.5 × 2.58 m). Mismatch ⇒ adjust scale/rotation overrides.

This script must be run via the Isaac Sim bundled python (`pxr` is
required + the gltfSDF plugin only resolves GLB references inside Kit):

    ~/env_isaaclab/bin/python glb_to_usd_wrapper.py ...

If you have no Kit handy, the script will still write the wrapper but
skip the bbox cross-check.
"""

import argparse
import os
import sys
from pathlib import Path

USDA_HEADER = """#usda 1.0
(
    defaultPrim = "{default_prim}"
    upAxis = "Z"
    metersPerUnit = 1.0
)

def Xform "{default_prim}"
{{
{children}
}}
"""

CHILD_BLOCK = """    def Xform "{name}" (
        prepend references = @./{glb}@
    )
    {{
        # {note}
        double3 xformOp:translate = (0, 0, 0)
        double3 xformOp:rotateXYZ = ({rx}, {ry}, {rz})
        double3 xformOp:scale = ({sx}, {sy}, {sz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
    }}
"""


def _try_pxr_probe(glb_path):
    """Return (size_xyz_meters_after_default_ref, succeeded_bool)."""
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        return None, False
    try:
        s = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(s, "/Probe").GetPrim()
        root.GetReferences().AddReference(str(glb_path))
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
        rng = bbc.ComputeWorldBound(root).ComputeAlignedRange()
        if rng.GetMin()[0] > 1e30:
            return None, True  # degenerate
        size = [round(rng.GetMax()[i] - rng.GetMin()[i], 4) for i in range(3)]
        return size, True
    except Exception as e:
        print(f"  [probe] error on {glb_path}: {e}", file=sys.stderr)
        return None, False


def _heuristics(size_xyz):
    """Suggest (scale, rotateX_deg, note) from raw bbox."""
    if not size_xyz:
        return (1.0, 0.0, "no bbox probe — wrapper applied at identity")
    sx, sy, sz = size_xyz
    max_dim = max(abs(sx), abs(sy), abs(sz))
    is_mm = max_dim > 100  # any-dim > 100 'units' likely means mm
    # Y-up if Y is the largest dimension AND it's clearly the 'up' axis
    is_yup = abs(sy) > max(abs(sx), abs(sz)) and abs(sy) / max(abs(sx), abs(sz), 1e-9) > 1.3
    scale = 0.001 if is_mm else 1.0
    rotateX = 90.0 if is_yup else 0.0
    notes = []
    if is_mm:
        notes.append(f"max-dim {max_dim:.1f} > 100 → mm units, scale 0.001")
    if is_yup:
        notes.append(f"Y largest ({sy:.2f}) > X,Z → Y-up source, rotateX +90")
    if not notes:
        notes.append(f"already m + Z-up (raw size {sx:.3f}×{sy:.3f}×{sz:.3f}) — identity")
    return (scale, rotateX, "; ".join(notes))


def parse_glb_spec(spec):
    """`floor.glb` or `floor.glb=scale=0.001,rotateX=90` → (path, overrides)."""
    if "=" not in spec:
        return spec, {}
    name, rest = spec.split("=", 1)
    overrides = {}
    for kv in rest.split(","):
        if "=" in kv:
            k, v = kv.split("=", 1)
            overrides[k.strip()] = float(v)
    return name, overrides


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", required=True, help="Wrapper .usda output path")
    ap.add_argument("--default-prim", required=True, help="Top-level Xform name")
    ap.add_argument("--glb", action="append", required=True,
                    help="GLB filename (relative to wrapper dir). Optional "
                         "overrides: NAME.glb=scale=S,rotateX=DEG")
    ap.add_argument("--auto-detect", action="store_true",
                    help="Probe each GLB with pxr to detect mm + Y-up; suggest "
                         "scale/rotateX automatically. Overrides per-GLB take "
                         "precedence over auto-detected values.")
    args = ap.parse_args()

    out = Path(args.output).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    base_dir = out.parent

    child_blocks = []
    print(f"=== glb_to_usd_wrapper: {out} ===")
    for spec in args.glb:
        glb_name, overrides = parse_glb_spec(spec)
        glb_full = base_dir / glb_name
        # Auto-naming child Xform: 'FooBar' from 'foo_bar.glb' or 'foo_bar_visual.glb'
        stem = Path(glb_name).stem
        if stem.endswith("_visual"):
            stem = stem[:-7]
        xform_name = "".join(part.capitalize() for part in stem.split("_")) + "Visual"

        scale = overrides.get("scale", 1.0)
        rotateX = overrides.get("rotateX", 0.0)
        note = "identity (no auto-detect, no overrides)"

        if args.auto_detect and not overrides:
            size, ok = _try_pxr_probe(glb_full)
            if ok and size is not None:
                scale_h, rotX_h, note = _heuristics(size)
                scale = overrides.get("scale", scale_h)
                rotateX = overrides.get("rotateX", rotX_h)
            else:
                note = "probe failed — defaults applied"

        # Compose post-correction expected bbox if we have the raw probe
        print(f"  {glb_name:40s} → /{args.default_prim}/{xform_name}")
        print(f"      scale=({scale}, {scale}, {scale})  rotateX={rotateX}")
        print(f"      reason: {note}")

        child_blocks.append(CHILD_BLOCK.format(
            name=xform_name,
            glb=glb_name,
            note=note,
            rx=rotateX, ry=0.0, rz=0.0,
            sx=scale, sy=scale, sz=scale,
        ))

    out.write_text(USDA_HEADER.format(
        default_prim=args.default_prim,
        children="\n".join(child_blocks).rstrip(),
    ))
    print(f"--- wrote {out} ({out.stat().st_size} bytes) ---")
    print("Reload Isaac Sim or re-issue load_scene; cross-check bbox via:")
    print(f"  bash {Path(__file__).parent}/probe_usd_bbox.py /World/<your_prim>")


if __name__ == "__main__":
    main()
