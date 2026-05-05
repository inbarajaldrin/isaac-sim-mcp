#!/usr/bin/env python3
# Reference: NVIDIA RigidBodyRopeDemo.py
# (~/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache/
#  omni.physx.demos-107.3.18+107.3.1.cp311.u353/omni/physxdemos/scenes/RigidBodyRopeDemo.py)
"""Offline cable physics authoring — opens aic_unified_robot_cable_sdf.usd
directly via pxr (no Isaac Sim needed), authors MassAPI(density=0.00005) on
each link + DriveAPI(type=force, damping=10.0, stiffness=1.0) on each joint's
rotY/rotZ axes per RigidBodyRopeDemo template, saves in place per D-06.

D-06 in-place USD edit policy: a `.bak` of the original is created on first
run; subsequent runs are idempotent.

Run with Isaac Sim's bundled Python so pxr is available:

    ~/env_isaaclab/bin/python exts/aic-dt/scripts/author_cable_physics_offline.py [--probe-only]
"""
import argparse
import os
import shutil
import sys

CABLE_USD = "/home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd"
DENSITY = 0.00005
DAMPING = 10.0
STIFFNESS = 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-only", action="store_true",
                    help="Inspect current authoring state without saving.")
    ap.add_argument("--usd", default=CABLE_USD, help="cable USD path")
    args = ap.parse_args()

    if not os.path.exists(args.usd):
        print(f"FAIL: cable USD not found at {args.usd}")
        return 1

    from pxr import Usd, UsdPhysics

    # Make a .bak before first edit
    bak = args.usd + ".bak"
    if not os.path.exists(bak) and not args.probe_only:
        shutil.copy2(args.usd, bak)
        print(f"Backed up original → {bak}")

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"FAIL: could not open {args.usd}")
        return 1

    links_seen = 0
    links_authored = 0
    links_already = 0
    joints_seen = 0
    joints_authored = 0
    joints_already = 0

    for prim in stage.Traverse():
        # Per-link MassAPI
        if UsdPhysics.RigidBodyAPI(prim):
            links_seen += 1
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass_api = UsdPhysics.MassAPI(prim)
            else:
                mass_api = UsdPhysics.MassAPI.Apply(prim) if not args.probe_only else None
            if mass_api is None:
                continue
            density_attr = mass_api.GetDensityAttr()
            current = float(density_attr.Get()) if density_attr.HasAuthoredValue() else None
            if current is not None and abs(current - DENSITY) < 1e-9:
                links_already += 1
            elif not args.probe_only:
                if not density_attr.HasAuthoredValue():
                    mass_api.CreateDensityAttr().Set(DENSITY)
                else:
                    density_attr.Set(DENSITY)
                links_authored += 1
        # Per-joint DriveAPI on rotY + rotZ
        type_name = str(prim.GetTypeName())
        if type_name.startswith("Physics") and "Joint" in type_name and type_name != "PhysicsFixedJoint":
            joints_seen += 1
            for axis in ("rotY", "rotZ"):
                if args.probe_only:
                    if prim.HasAPI(UsdPhysics.DriveAPI, axis):
                        d = UsdPhysics.DriveAPI(prim, axis)
                        if d.GetTypeAttr().HasAuthoredValue() and \
                           d.GetDampingAttr().HasAuthoredValue() and \
                           d.GetStiffnessAttr().HasAuthoredValue() and \
                           str(d.GetTypeAttr().Get()) == "force" and \
                           abs(float(d.GetDampingAttr().Get()) - DAMPING) < 1e-6 and \
                           abs(float(d.GetStiffnessAttr().Get()) - STIFFNESS) < 1e-6:
                            joints_already += 1
                    continue
                drive = UsdPhysics.DriveAPI.Apply(prim, axis)
                drive.CreateTypeAttr().Set("force")
                drive.CreateDampingAttr().Set(DAMPING)
                drive.CreateStiffnessAttr().Set(STIFFNESS)
                joints_authored += 1

    print(f"links_seen={links_seen} links_authored_now={links_authored} links_already_correct={links_already}")
    print(f"joints_seen={joints_seen} joints_authored_now={joints_authored} joints_already_correct={joints_already}")

    if not args.probe_only and (links_authored + joints_authored > 0):
        stage.GetRootLayer().Save()
        print(f"Saved {args.usd}")

    expect_links = 23
    if links_seen != expect_links:
        print(f"WARN: expected {expect_links} links, found {links_seen}")
        return 0
    if not args.probe_only:
        # Re-open + re-probe to verify save took
        stage2 = Usd.Stage.Open(args.usd)
        verified_density = 0
        for prim in stage2.Traverse():
            if UsdPhysics.RigidBodyAPI(prim) and prim.HasAPI(UsdPhysics.MassAPI):
                d = UsdPhysics.MassAPI(prim).GetDensityAttr()
                if d.HasAuthoredValue() and abs(float(d.Get()) - DENSITY) < 1e-9:
                    verified_density += 1
        print(f"Verified after save: {verified_density}/{expect_links} links have density={DENSITY}")
        return 0 if verified_density == expect_links else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
