#!/usr/bin/env python3
"""
check_cad_dimensions.py — Three-source CAD dimension verifier for aic-dt.

Purpose
-------
Catch scale-bug regressions BEFORE they corrupt an Isaac Sim scene by
cross-checking the axis-aligned bounding box (AABB) of a CAD asset across
multiple independent loaders. If two sources disagree by more than 1%, the
script exits 1 and recommends a scale correction factor.

Background
----------
On 2026-05-12 we discovered that ``nic_card_visual.usd`` (Isaac Sim asset)
reported a mesh AABB of ~12.1m × 15.9m × 2.1m, while the Gazebo source
``nic_card_visual.glb`` reported ~12cm × 14.5cm × 5.8cm via its glTF
accessor metadata — a 100x scale corruption introduced during USD
authoring. This script exists so the next such regression is caught the
moment an asset is imported, not weeks later at runtime.

Sources of truth (best → worst)
-------------------------------
1. **Direct parser** — for GLB we read the JSON header + glTF accessor
   ``min``/``max`` fields (no mesh decode required; metadata is normative
   per the glTF spec). For USD we use the USD Python API
   (``UsdGeom.BBoxCache``). For OBJ/STL we parse vertex floats directly.
2. **Blender headless** — ``blender --background --python`` loads via the
   official Khronos glTF / USD importers and reads
   ``object.dimensions``. Blender's CAD-aware unit handling is the
   independent cross-check.
3. **Isaac Sim live stage** — if the MCP socket on 8768 responds within
   5s, we ask the running stage if this asset is referenced and read its
   world-space AABB. Falls back gracefully if Isaac Sim is offline.

Usage
-----
    # Single-file check (auto-detects format from extension)
    python3 check_cad_dimensions.py path/to/asset.glb

    # Compare GLB and USD versions of the "same" asset
    python3 check_cad_dimensions.py path/to/asset.glb --compare path/to/asset.usd

    # Verbose (show raw vertex parse, Blender stdout, USD prim tree)
    python3 check_cad_dimensions.py asset.glb --verbose

    # Skip the (slow) Blender source
    python3 check_cad_dimensions.py asset.glb --no-blender

Exit codes
----------
    0 — all available sources agree within 1% tolerance
    1 — mismatch detected (script prints recommended scale fix)
    2 — script error (file not found, parser crash, etc.)

Authoring notes
---------------
- STEP / IGES are NOT supported by stock Blender; for those, install
  ``CAD Sketcher`` add-on or use FreeCAD's ``cadquery`` Python module.
  This script prints a guidance message and falls back to direct/USD only.
- The script never touches Isaac Sim's main thread destructively — the
  MCP query is read-only and uses a 5s timeout.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# -----------------------------------------------------------------------------
# Data model
# -----------------------------------------------------------------------------

@dataclass
class AABB:
    """Axis-aligned bounding box in meters (assumed unit for glTF/USD spec)."""
    min: tuple[float, float, float]
    max: tuple[float, float, float]
    source: str = "unknown"
    notes: str = ""

    @property
    def size(self) -> tuple[float, float, float]:
        return tuple(self.max[i] - self.min[i] for i in range(3))

    @property
    def size_mm(self) -> tuple[float, float, float]:
        return tuple(s * 1000.0 for s in self.size)

    def pretty(self) -> str:
        sx, sy, sz = self.size
        return (
            f"  source     : {self.source}\n"
            f"  AABB min   : ({self.min[0]:+.4f}, {self.min[1]:+.4f}, {self.min[2]:+.4f}) m\n"
            f"  AABB max   : ({self.max[0]:+.4f}, {self.max[1]:+.4f}, {self.max[2]:+.4f}) m\n"
            f"  size (m)   : {sx:.4f} × {sy:.4f} × {sz:.4f}\n"
            f"  size (mm)  : {sx*1000:.1f} × {sy*1000:.1f} × {sz*1000:.1f}\n"
            + (f"  notes      : {self.notes}\n" if self.notes else "")
        )


# -----------------------------------------------------------------------------
# Source 1a: Direct GLB parser (JSON header + accessor min/max metadata)
# -----------------------------------------------------------------------------

def parse_glb_direct(glb_path: Path, verbose: bool = False) -> Optional[AABB]:
    """Read a GLB's JSON header and union all POSITION accessor min/max boxes.

    Per the glTF spec, every POSITION accessor MUST include a min[3] and
    max[3] in *local mesh space*. This is the same data Blender / Isaac
    Sim consume, so it is the canonical "what does the file claim its
    size is" reading — no mesh decode needed.

    We union the boxes of every primitive across every mesh, because
    most exported assets bake their transform hierarchy into vertex
    positions (so taking the global union approximates the world AABB).
    """
    try:
        with open(glb_path, "rb") as f:
            magic, version, length = struct.unpack("<III", f.read(12))
            if magic != 0x46546C67:  # 'glTF'
                print(f"[ERR] {glb_path}: bad GLB magic 0x{magic:x}", file=sys.stderr)
                return None
            # Read JSON chunk
            chunk_len, chunk_type = struct.unpack("<II", f.read(8))
            if chunk_type != 0x4E4F534A:  # 'JSON'
                print(f"[ERR] {glb_path}: first chunk is not JSON", file=sys.stderr)
                return None
            json_bytes = f.read(chunk_len)
            gltf = json.loads(json_bytes.decode("utf-8"))

        accessors = gltf.get("accessors", [])
        meshes = gltf.get("meshes", [])
        if not meshes:
            print(f"[WARN] {glb_path}: no meshes in glTF", file=sys.stderr)
            return None

        mins = [float("inf")] * 3
        maxs = [float("-inf")] * 3
        primitives_seen = 0
        for mesh in meshes:
            for prim in mesh.get("primitives", []):
                pos_idx = prim.get("attributes", {}).get("POSITION")
                if pos_idx is None:
                    continue
                acc = accessors[pos_idx]
                amin = acc.get("min")
                amax = acc.get("max")
                if not amin or not amax or len(amin) < 3 or len(amax) < 3:
                    continue
                for i in range(3):
                    mins[i] = min(mins[i], amin[i])
                    maxs[i] = max(maxs[i], amax[i])
                primitives_seen += 1
                if verbose:
                    print(f"    [glb-direct] prim accessor {pos_idx}: min={amin[:3]} max={amax[:3]}")

        if primitives_seen == 0:
            return None

        # glTF assets that go through nodes with non-identity transforms
        # would understate the world AABB — flag it so the user knows.
        has_node_transforms = any(
            ("matrix" in n or "scale" in n or "rotation" in n or "translation" in n)
            for n in gltf.get("nodes", [])
        )
        notes = (
            f"unioned {primitives_seen} POSITION accessor box(es); "
            f"node transforms {'present (AABB is mesh-local, world may differ)' if has_node_transforms else 'absent (mesh-local == world)'}"
        )
        return AABB(min=tuple(mins), max=tuple(maxs),
                    source=f"GLB direct (accessor min/max)", notes=notes)
    except Exception as e:
        print(f"[ERR] glb direct parse failed: {e}", file=sys.stderr)
        return None


# -----------------------------------------------------------------------------
# Source 1b: Direct OBJ / STL parsers
# -----------------------------------------------------------------------------

def parse_obj_direct(path: Path, verbose: bool = False) -> Optional[AABB]:
    """Walk OBJ vertex lines and compute AABB."""
    try:
        mins = [float("inf")] * 3
        maxs = [float("-inf")] * 3
        count = 0
        with open(path) as f:
            for line in f:
                if line.startswith("v "):
                    parts = line.split()
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        for i, v in enumerate((x, y, z)):
                            mins[i] = min(mins[i], v); maxs[i] = max(maxs[i], v)
                        count += 1
        if count == 0:
            return None
        return AABB(min=tuple(mins), max=tuple(maxs),
                    source="OBJ direct", notes=f"{count} vertices")
    except Exception as e:
        print(f"[ERR] obj parse failed: {e}", file=sys.stderr)
        return None


def parse_stl_direct(path: Path, verbose: bool = False) -> Optional[AABB]:
    """Binary STL parser; falls back to ASCII if header check fails."""
    try:
        with open(path, "rb") as f:
            header = f.read(80)
            tri_count_raw = f.read(4)
            if len(tri_count_raw) != 4:
                return None
            tri_count = struct.unpack("<I", tri_count_raw)[0]
            expected = 80 + 4 + tri_count * 50
            file_size = path.stat().st_size
            if expected == file_size:
                # Binary STL
                mins = [float("inf")] * 3; maxs = [float("-inf")] * 3
                for _ in range(tri_count):
                    f.read(12)  # normal
                    for _v in range(3):
                        x, y, z = struct.unpack("<fff", f.read(12))
                        for i, v in enumerate((x, y, z)):
                            mins[i] = min(mins[i], v); maxs[i] = max(maxs[i], v)
                    f.read(2)  # attr byte count
                return AABB(min=tuple(mins), max=tuple(maxs),
                            source="STL direct (binary)",
                            notes=f"{tri_count} triangles")
        # ASCII STL fallback
        mins = [float("inf")] * 3; maxs = [float("-inf")] * 3
        count = 0
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith("vertex "):
                    parts = line.split()
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        for i, v in enumerate((x, y, z)):
                            mins[i] = min(mins[i], v); maxs[i] = max(maxs[i], v)
                        count += 1
        if count == 0:
            return None
        return AABB(min=tuple(mins), max=tuple(maxs),
                    source="STL direct (ASCII)", notes=f"{count} vertices")
    except Exception as e:
        print(f"[ERR] stl parse failed: {e}", file=sys.stderr)
        return None


# -----------------------------------------------------------------------------
# Source 1c: USD direct (pxr.UsdGeom.BBoxCache)
# -----------------------------------------------------------------------------

def parse_usd_direct(path: Path, verbose: bool = False) -> Optional[AABB]:
    """Open a USD stage and compute the world-space AABB of the default prim
    (or pseudo-root). Uses ``UsdGeom.BBoxCache`` with the ``default`` purpose
    so render+default purposes are both included.
    """
    try:
        from pxr import Usd, UsdGeom, Gf
    except ImportError:
        print("[WARN] pxr (USD Python) not installed — skipping USD direct source", file=sys.stderr)
        return None
    try:
        stage = Usd.Stage.Open(str(path))
        if stage is None:
            print(f"[ERR] could not open USD stage: {path}", file=sys.stderr)
            return None
        root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        bbox = cache.ComputeWorldBound(root)
        rng = bbox.ComputeAlignedRange()
        if rng.IsEmpty():
            print(f"[WARN] USD AABB empty for {path}", file=sys.stderr)
            return None
        mn = rng.GetMin(); mx = rng.GetMax()

        # Detect stage metersPerUnit (defaults to 0.01 = centimeters per the
        # USD spec, but Isaac Sim assets typically declare 1.0 = meters).
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        notes = f"default prim={root.GetPath()}; metersPerUnit={mpu}"
        # Convert to meters using the stage's declared unit
        return AABB(
            min=(mn[0] * mpu, mn[1] * mpu, mn[2] * mpu),
            max=(mx[0] * mpu, mx[1] * mpu, mx[2] * mpu),
            source=f"USD direct (BBoxCache, metersPerUnit={mpu})",
            notes=notes,
        )
    except Exception as e:
        print(f"[ERR] USD parse failed: {e}", file=sys.stderr)
        return None


# -----------------------------------------------------------------------------
# Source 2: Blender headless
# -----------------------------------------------------------------------------

BLENDER_SCRIPT = r"""
import bpy, json, sys, os

_path = os.environ['CAD_CHECK_PATH']
_ext = os.path.splitext(_path)[1].lower()
# Caller-provided unit correction for formats that don't carry a unit
# convention (USD's metersPerUnit, etc.). 1.0 = no rescale.
_meters_per_unit = float(os.environ.get('CAD_CHECK_METERS_PER_UNIT', '1.0'))

# Wipe default scene
bpy.ops.wm.read_factory_settings(use_empty=True)
# Force scene unit to meters so .dimensions is in meters
bpy.context.scene.unit_settings.system = 'METRIC'
bpy.context.scene.unit_settings.scale_length = 1.0

try:
    if _ext == '.glb' or _ext == '.gltf':
        # glTF spec: positions are in meters. Blender's importer respects this.
        bpy.ops.import_scene.gltf(filepath=_path)
    elif _ext == '.obj':
        # Blender 4.x: import_scene.obj is removed; wm.obj_import is the new API
        if hasattr(bpy.ops.wm, 'obj_import'):
            bpy.ops.wm.obj_import(filepath=_path)
        else:
            bpy.ops.import_scene.obj(filepath=_path)
    elif _ext == '.stl':
        if hasattr(bpy.ops.wm, 'stl_import'):
            bpy.ops.wm.stl_import(filepath=_path)
        else:
            bpy.ops.import_mesh.stl(filepath=_path)
    elif _ext == '.fbx':
        bpy.ops.import_scene.fbx(filepath=_path)
    elif _ext in ('.usd', '.usda', '.usdc', '.usdz'):
        # IMPORTANT: Blender's USD importer (as of 4.3) does NOT honor the
        # stage's metersPerUnit metadata — it imports raw vertex values as
        # meters regardless. We compensate by passing a scale factor below.
        bpy.ops.wm.usd_import(filepath=_path)
    elif _ext in ('.step', '.stp', '.iges', '.igs'):
        sys.stderr.write('BLENDER_RESULT:{"error":"STEP/IGES not supported by stock Blender — install CAD Sketcher or use FreeCAD"}\n')
        sys.exit(0)
    else:
        sys.stderr.write(f'BLENDER_RESULT:{{"error":"unknown extension {_ext}"}}\n')
        sys.exit(0)
except Exception as e:
    sys.stderr.write(f'BLENDER_RESULT:{{"error":"import failed: {e}"}}\n')
    sys.exit(0)

# Union AABB of all mesh objects in world space
mins = [float('inf')]*3
maxs = [float('-inf')]*3
mesh_count = 0
for obj in bpy.context.scene.objects:
    if obj.type != 'MESH':
        continue
    mesh_count += 1
    # bound_box is 8 corners in LOCAL space; transform to world
    for corner in obj.bound_box:
        wc = obj.matrix_world @ __import__('mathutils').Vector(corner)
        for i in range(3):
            if wc[i] < mins[i]: mins[i] = wc[i]
            if wc[i] > maxs[i]: maxs[i] = wc[i]

if mesh_count == 0:
    sys.stderr.write('BLENDER_RESULT:{"error":"no mesh objects imported"}\n')
    sys.exit(0)

# Apply caller-provided unit correction (e.g. USD metersPerUnit). For glTF
# this is 1.0 since the spec mandates meters.
if _meters_per_unit != 1.0:
    mins = [m * _meters_per_unit for m in mins]
    maxs = [m * _meters_per_unit for m in maxs]

result = {
    'min': mins, 'max': maxs, 'mesh_count': mesh_count,
    'unit_system': bpy.context.scene.unit_settings.system,
    'scale_length': bpy.context.scene.unit_settings.scale_length,
    'applied_meters_per_unit': _meters_per_unit,
}
sys.stderr.write('BLENDER_RESULT:' + json.dumps(result) + '\n')
"""


def _detect_usd_meters_per_unit(path: Path) -> float:
    """Read the stage's metersPerUnit so we can compensate for Blender's
    USD importer ignoring it. Returns 1.0 if pxr isn't available or path
    isn't a USD file."""
    if path.suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        return 1.0
    try:
        from pxr import Usd, UsdGeom
        stage = Usd.Stage.Open(str(path))
        if stage is None:
            return 1.0
        return float(UsdGeom.GetStageMetersPerUnit(stage))
    except Exception:
        return 1.0


def parse_via_blender(path: Path, verbose: bool = False, timeout: float = 120.0) -> Optional[AABB]:
    """Spawn Blender headless, import the asset, and read object dimensions."""
    blender = subprocess.run(["which", "blender"], capture_output=True, text=True).stdout.strip()
    if not blender:
        print("[WARN] blender not in PATH — skipping Blender source", file=sys.stderr)
        return None

    env = os.environ.copy()
    env["CAD_CHECK_PATH"] = str(path)
    mpu = _detect_usd_meters_per_unit(path)
    env["CAD_CHECK_METERS_PER_UNIT"] = str(mpu)

    try:
        proc = subprocess.run(
            [blender, "--background", "--factory-startup", "--python-expr", BLENDER_SCRIPT],
            capture_output=True, text=True, timeout=timeout, env=env,
        )
    except subprocess.TimeoutExpired:
        print(f"[ERR] Blender timeout ({timeout}s)", file=sys.stderr)
        return None

    if verbose:
        print("    [blender stdout]", proc.stdout[:2000])
        print("    [blender stderr]", proc.stderr[:2000])

    # The script writes its JSON result to stderr prefixed with BLENDER_RESULT:
    result_line = None
    for line in (proc.stderr + "\n" + proc.stdout).splitlines():
        if "BLENDER_RESULT:" in line:
            result_line = line[line.index("BLENDER_RESULT:") + len("BLENDER_RESULT:"):]
            break
    if not result_line:
        print("[ERR] no BLENDER_RESULT line in Blender output", file=sys.stderr)
        return None

    try:
        data = json.loads(result_line.strip())
    except json.JSONDecodeError as e:
        print(f"[ERR] could not parse Blender result: {e}", file=sys.stderr)
        return None

    if "error" in data:
        print(f"[WARN] Blender: {data['error']}", file=sys.stderr)
        return None

    return AABB(
        min=tuple(data["min"]),
        max=tuple(data["max"]),
        source=f"Blender 4.x headless ({data.get('unit_system','?')}, scale_length={data.get('scale_length','?')})",
        notes=f"{data.get('mesh_count', '?')} mesh object(s) imported",
    )


# -----------------------------------------------------------------------------
# Source 3: Isaac Sim live stage (via MCP socket, read-only, short timeout)
# -----------------------------------------------------------------------------

def query_isaac_sim_aabb(asset_filename: str, host: str = "127.0.0.1",
                          port: int = 8768, timeout: float = 5.0,
                          verbose: bool = False) -> Optional[AABB]:
    """Ask the running aic-dt MCP server if the asset is referenced in the
    current stage, and if so return its world-space AABB.

    Uses ``execute_python_code`` because there is no dedicated MCP tool for
    AABB queries. Read-only — never mutates the stage.
    """
    # We use a Python snippet that walks the stage looking for prims whose
    # references include the asset filename.
    snippet = f"""
from pxr import Usd, UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()
needle = {asset_filename!r}
hits = []
for prim in stage.Traverse():
    refs = prim.GetMetadata('references')
    if refs is None: continue
    try:
        for r in refs.GetAppliedItems():
            if needle in r.assetPath:
                hits.append(str(prim.GetPath()))
                break
    except Exception:
        pass

if not hits:
    result = {{'found': False}}
else:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    out = []
    for path in hits:
        prim = stage.GetPrimAtPath(path)
        bbox = cache.ComputeWorldBound(prim)
        rng = bbox.ComputeAlignedRange()
        if rng.IsEmpty(): continue
        mn = rng.GetMin(); mx = rng.GetMax()
        out.append({{
            'path': path,
            'min': [mn[0], mn[1], mn[2]],
            'max': [mx[0], mx[1], mx[2]],
        }})
    result = {{'found': True, 'hits': out,
               'metersPerUnit': UsdGeom.GetStageMetersPerUnit(stage)}}
"""
    payload = json.dumps({"type": "execute_python_code",
                          "params": {"code": snippet}}).encode()
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall(payload)
            buf = b""
            while True:
                chunk = s.recv(16384)
                if not chunk: break
                buf += chunk
                try:
                    resp = json.loads(buf.decode())
                    break
                except json.JSONDecodeError:
                    continue
            else:
                return None
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        if verbose:
            print(f"    [isaac-sim] socket unavailable: {e}")
        return None

    if verbose:
        print(f"    [isaac-sim] raw response: {str(resp)[:400]}")

    res = resp.get("result", {})
    # The execute_python_code handler typically returns the last expression
    # or the value of `result =` — handle both
    payload_obj = res.get("result") if isinstance(res, dict) else None
    if not isinstance(payload_obj, dict):
        # Try common alternative shapes
        payload_obj = res if isinstance(res, dict) and "found" in res else None
    if not payload_obj or not payload_obj.get("found"):
        return None

    hits = payload_obj.get("hits", [])
    if not hits:
        return None

    # Union all hits
    mins = [float("inf")] * 3; maxs = [float("-inf")] * 3
    for h in hits:
        for i in range(3):
            mins[i] = min(mins[i], h["min"][i])
            maxs[i] = max(maxs[i], h["max"][i])
    mpu = payload_obj.get("metersPerUnit", 1.0)
    return AABB(
        min=tuple(m * mpu for m in mins),
        max=tuple(m * mpu for m in maxs),
        source=f"Isaac Sim live stage (MCP @ {port})",
        notes=f"{len(hits)} prim(s) reference {asset_filename}; metersPerUnit={mpu}",
    )


# -----------------------------------------------------------------------------
# Comparison + recommendation
# -----------------------------------------------------------------------------

def compare_aabbs(boxes: list[AABB], tolerance: float = 0.01) -> tuple[bool, str]:
    """Return (agree, message). Compares sorted-size tuples across all boxes,
    so axis-ordering differences (Y-up vs Z-up conventions) don't trigger
    false mismatches — we only care whether the *magnitudes* agree.

    Tolerance = fractional (0.01 = 1%).
    """
    if len(boxes) < 2:
        return True, "only one source available — cannot cross-check"

    # Sort each size triple descending so we compare like-with-like even when
    # one source is Y-up and another is Z-up (glTF / Blender / USD frequently
    # differ on axis convention; the asset's bounding extents are invariant).
    sorted_sizes = [tuple(sorted(b.size, reverse=True)) for b in boxes]
    max_size = max(max(s) for s in sorted_sizes)
    if max_size == 0:
        return False, "all sources report zero size"

    # For each "slot" (largest, mid, smallest extent), find spread
    slot_labels = ("largest", "middle", "smallest")
    mismatches = []
    for slot_i, slot in enumerate(slot_labels):
        vals = [s[slot_i] for s in sorted_sizes]
        vmin, vmax = min(vals), max(vals)
        if vmin == 0 and vmax == 0:
            continue
        rel = (vmax - vmin) / max(vmax, 1e-9)
        if rel > tolerance:
            details = ", ".join(
                f"{b.source.split('(')[0].strip()}={s[slot_i]*1000:.1f}mm"
                for b, s in zip(boxes, sorted_sizes)
            )
            mismatches.append(f"  {slot} extent: spread {rel*100:.1f}% — {details}")

    if not mismatches:
        return True, (
            f"all {len(boxes)} sources agree within {tolerance*100:.0f}% "
            f"(sorted-extent comparison; axis-order independent)"
        )

    # Recommend scale fix: pick the two most-disagreeing sources by largest
    # extent and compute ratio.
    sorted_by_max = sorted(zip(boxes, sorted_sizes), key=lambda bs: bs[1][0])
    (smallest_box, smallest_sz), (largest_box, largest_sz) = sorted_by_max[0], sorted_by_max[-1]
    if smallest_sz[0] > 0:
        ratio = largest_sz[0] / smallest_sz[0]
        fix = 1.0 / ratio
        rec = (
            f"\n  RECOMMENDED FIX:\n"
            f"  '{largest_box.source}' is {ratio:.2f}× larger than '{smallest_box.source}'.\n"
            f"  If '{largest_box.source}' is the corrupted source, apply scale = "
            f"{fix:.4g} on the reference Xform that brings it in.\n"
            f"  Example USD edit:\n"
            f"      over \"<your_prim>\" (\n"
            f"          references = @<asset>.usd@\n"
            f"      ) {{\n"
            f"          float3 xformOp:scale = ({fix:.4g}, {fix:.4g}, {fix:.4g})\n"
            f"          uniform token[] xformOpOrder = [\"xformOp:scale\"]\n"
            f"      }}\n"
        )
    else:
        rec = ""

    # If a GLB-direct reading is in the box list AND it flagged
    # "node transforms present", warn the user that GLB-direct is the LEAST
    # trustworthy source for this asset — it sees only mesh-local accessor
    # boxes, not the post-node-transform world layout. The other sources
    # (Blender / USD / Isaac Sim) all walk the scene graph and are correct.
    glb_caveat = ""
    for b in boxes:
        if "GLB direct" in b.source and "node transforms present" in b.notes:
            glb_caveat = (
                "\n  NOTE: 'GLB direct' uses raw POSITION accessor min/max, which is "
                "MESH-LOCAL.\n  This GLB has non-identity node transforms, so the "
                "accessor union understates\n  the true world AABB. If the OTHER "
                "sources agree with each other, trust them\n  and treat GLB-direct "
                "as an outlier (not a corruption signal).\n"
            )
            break

    return False, "DIMENSION MISMATCH:\n" + "\n".join(mismatches) + glb_caveat + rec


# -----------------------------------------------------------------------------
# Dispatch
# -----------------------------------------------------------------------------

def aabb_for_file(path: Path, verbose: bool = False) -> Optional[AABB]:
    """Pick the right direct parser based on the file extension."""
    ext = path.suffix.lower()
    if ext == ".glb":
        return parse_glb_direct(path, verbose)
    if ext == ".obj":
        return parse_obj_direct(path, verbose)
    if ext == ".stl":
        return parse_stl_direct(path, verbose)
    if ext in (".usd", ".usda", ".usdc", ".usdz"):
        return parse_usd_direct(path, verbose)
    print(f"[WARN] no direct parser for {ext}", file=sys.stderr)
    return None


def check_one_asset(path: Path, sources: list[AABB], verbose: bool,
                     use_blender: bool, use_isaac: bool) -> None:
    """Populate ``sources`` with every available AABB reading for ``path``."""
    direct = aabb_for_file(path, verbose)
    if direct:
        sources.append(direct)

    if use_blender:
        b = parse_via_blender(path, verbose)
        if b:
            sources.append(b)

    if use_isaac:
        ias = query_isaac_sim_aabb(path.name, verbose=verbose)
        if ias:
            sources.append(ias)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("path", type=Path, help="primary asset file (.glb, .usd, .obj, .stl)")
    ap.add_argument("--compare", type=Path, default=None,
                    help="optional second asset file (compares boxes across BOTH files)")
    ap.add_argument("--no-blender", action="store_true",
                    help="skip the Blender headless source (faster)")
    ap.add_argument("--no-isaac", action="store_true",
                    help="skip the Isaac Sim live-stage query")
    ap.add_argument("--tolerance", type=float, default=0.01,
                    help="fractional size disagreement allowed (default 0.01 = 1%%)")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    if not args.path.exists():
        print(f"[ERR] {args.path} not found", file=sys.stderr)
        return 2

    print(f"\n=== Verifying: {args.path} ===")
    sources: list[AABB] = []
    check_one_asset(args.path, sources, args.verbose,
                    use_blender=not args.no_blender,
                    use_isaac=not args.no_isaac)

    if args.compare:
        if not args.compare.exists():
            print(f"[ERR] --compare {args.compare} not found", file=sys.stderr)
            return 2
        print(f"\n=== Also reading: {args.compare} ===")
        check_one_asset(args.compare, sources, args.verbose,
                        use_blender=not args.no_blender,
                        use_isaac=not args.no_isaac)

    if not sources:
        print("[ERR] no sources produced an AABB — cannot verify", file=sys.stderr)
        return 2

    print("\n--- Source readings ---")
    for s in sources:
        print(s.pretty())

    print("--- Cross-source comparison ---")
    agree, msg = compare_aabbs(sources, tolerance=args.tolerance)
    print(msg)
    print()
    return 0 if agree else 1


if __name__ == "__main__":
    sys.exit(main())
