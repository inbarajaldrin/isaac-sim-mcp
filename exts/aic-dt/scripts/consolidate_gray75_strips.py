#!/usr/bin/env python3
# Reference: https://openusd.org/release/api/class_usd_geom_mesh.html
"""Consolidate GRAY75 FLOOR/MID/CEILING sub-meshes into a single GRAY75 mesh.

Background
----------
The prior-session GLB→USDC splitter pipeline subdivided the source GRAY75
primitive (single primitive in the GLB) into three Z-stratified meshes:
    V1014952_001_GRAY75_FLOOR    (z=[0.13, 1.08])
    V1014952_001_GRAY75_MID      (z=[0.77, 1.14])
    V1014952_001_GRAY75_CEILING  (z=[2.52, 2.53])

The split was originally for selective-hiding (hide the ceiling for
debugging visibility). That use-case was abandoned in the 2026-05-13
session. The split is now structural-only and was the indirect cause of
the bars-vs-ceiling color mismatch (the CEILING mesh ended up bound to a
separate `GRAY75_CEILING_TRANSLUCENT` material, diverging from the rest).

This script restores the source-faithful single-primitive structure:
    V1014952_001_GRAY75  (one mesh, all three Z bands)
bound to the regular `/Enclosure/Looks/GRAY75` material.

What it does
------------
1. Reads `enclosure_split.usdc`.
2. Loads points/counts/indices/normals/extent from each of FLOOR/MID/CEILING.
3. Builds a merged mesh — concatenated points; face indices re-offset to
   point into the merged points array; counts + per-face normals concatenated.
4. Authors a new `/Enclosure/V1014952_001_GRAY75` Mesh prim with the merged
   buffers, bound to `/Enclosure/Looks/GRAY75`.
5. Removes the three sub-mesh prims AND the orphan
   `GRAY75_CEILING_TRANSLUCENT` material.

Idempotent: re-running detects already-consolidated state and is a no-op.

Run via:

    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \\
        exts/aic-dt/scripts/consolidate_gray75_strips.py
"""
from __future__ import annotations

import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

ENCLOSURE_USDC = Path(__file__).resolve().parents[1] / "assets" / "Enclosure" / "enclosure_split.usdc"

OLD_MESH_PATHS = [
    "/Enclosure/V1014952_001_GRAY75_FLOOR",
    "/Enclosure/V1014952_001_GRAY75_MID",
    "/Enclosure/V1014952_001_GRAY75_CEILING",
]
NEW_MESH_PATH = "/Enclosure/V1014952_001_GRAY75"
GRAY75_MATERIAL_PATH = "/Enclosure/Looks/GRAY75"
ORPHAN_MATERIAL_PATH = "/Enclosure/Looks/GRAY75_CEILING_TRANSLUCENT"


def read_mesh(mesh_prim) -> dict:
    mesh = UsdGeom.Mesh(mesh_prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    normals_attr = mesh.GetNormalsAttr()
    normals = normals_attr.Get() if normals_attr else None
    interp = mesh.GetNormalsInterpolation() if normals is not None else None
    extent_attr = mesh.GetExtentAttr()
    extent = extent_attr.Get() if extent_attr else None
    if pts is None or counts is None or indices is None:
        raise RuntimeError(f"{mesh_prim.GetPath()}: missing points/counts/indices")
    return {
        "pts": list(pts),
        "counts": list(counts),
        "indices": list(indices),
        "normals": list(normals) if normals is not None else None,
        "normals_interp": interp,
        "extent": list(extent) if extent is not None else None,
    }


def merge_meshes(meshes: list[dict]) -> dict:
    merged_pts: list = []
    merged_counts: list = []
    merged_indices: list = []
    merged_normals: list | None = []
    normals_interp = None
    for m in meshes:
        offset = len(merged_pts)
        merged_pts.extend(m["pts"])
        merged_counts.extend(m["counts"])
        merged_indices.extend(idx + offset for idx in m["indices"])
        if m["normals"] is not None and merged_normals is not None:
            merged_normals.extend(m["normals"])
            if normals_interp is None:
                normals_interp = m["normals_interp"]
            elif normals_interp != m["normals_interp"]:
                print(
                    f"[WARN] Normals interpolation mismatch — "
                    f"falling back to no normals on merged mesh",
                    file=sys.stderr,
                )
                merged_normals = None
        elif m["normals"] is None:
            merged_normals = None
    # Recompute extent from merged pts
    if merged_pts:
        xs = [p[0] for p in merged_pts]
        ys = [p[1] for p in merged_pts]
        zs = [p[2] for p in merged_pts]
        extent = [Gf.Vec3f(min(xs), min(ys), min(zs)),
                  Gf.Vec3f(max(xs), max(ys), max(zs))]
    else:
        extent = None
    return {
        "pts": merged_pts,
        "counts": merged_counts,
        "indices": merged_indices,
        "normals": merged_normals,
        "normals_interp": normals_interp,
        "extent": extent,
    }


def author_merged_mesh(stage, data: dict) -> UsdGeom.Mesh:
    mesh_prim = stage.GetPrimAtPath(NEW_MESH_PATH)
    if mesh_prim and mesh_prim.IsValid():
        mesh = UsdGeom.Mesh(mesh_prim)
    else:
        mesh = UsdGeom.Mesh.Define(stage, NEW_MESH_PATH)
    mesh.CreatePointsAttr(Vt.Vec3fArray(data["pts"]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(data["counts"]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(data["indices"]))
    if data["normals"] is not None:
        mesh.CreateNormalsAttr(Vt.Vec3fArray(data["normals"]))
        if data["normals_interp"] is not None:
            mesh.SetNormalsInterpolation(data["normals_interp"])
    if data["extent"] is not None:
        mesh.CreateExtentAttr(Vt.Vec3fArray(data["extent"]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(False)
    binding_api = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    gray75_mat = UsdShade.Material(stage.GetPrimAtPath(GRAY75_MATERIAL_PATH))
    binding_api.Bind(gray75_mat)
    return mesh


def main() -> None:
    if not ENCLOSURE_USDC.exists():
        print(f"[ERR] {ENCLOSURE_USDC} not found", file=sys.stderr)
        sys.exit(1)

    stage = Usd.Stage.Open(str(ENCLOSURE_USDC))

    # Idempotency: if the new mesh exists and the old ones are gone, we're done.
    new_exists = stage.GetPrimAtPath(NEW_MESH_PATH).IsValid()
    olds_exist = [stage.GetPrimAtPath(p).IsValid() for p in OLD_MESH_PATHS]
    if new_exists and not any(olds_exist):
        print(f"[OK] {NEW_MESH_PATH} already consolidated (nothing to do)")
        return

    meshes_data: list[dict] = []
    for p in OLD_MESH_PATHS:
        prim = stage.GetPrimAtPath(p)
        if not (prim and prim.IsValid()):
            print(f"[WARN] {p} not found — skipping", file=sys.stderr)
            continue
        d = read_mesh(prim)
        print(f"[INFO] {p}: {len(d['pts'])} pts, {len(d['counts'])} faces")
        meshes_data.append(d)

    if not meshes_data:
        print(f"[ERR] no source meshes found; nothing to merge", file=sys.stderr)
        sys.exit(2)

    merged = merge_meshes(meshes_data)
    print(f"[INFO] merged: {len(merged['pts'])} pts, {len(merged['counts'])} faces")

    author_merged_mesh(stage, merged)
    print(f"[OK] authored {NEW_MESH_PATH} bound to {GRAY75_MATERIAL_PATH}")

    # Remove the FLOOR/MID/CEILING sub-meshes.
    for p in OLD_MESH_PATHS:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)
            print(f"[OK] removed {p}")

    # Remove the orphan GRAY75_CEILING_TRANSLUCENT material.
    if stage.GetPrimAtPath(ORPHAN_MATERIAL_PATH).IsValid():
        stage.RemovePrim(ORPHAN_MATERIAL_PATH)
        print(f"[OK] removed orphan {ORPHAN_MATERIAL_PATH}")

    stage.GetRootLayer().Save()


if __name__ == "__main__":
    main()
