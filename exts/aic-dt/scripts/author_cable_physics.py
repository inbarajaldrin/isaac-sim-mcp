#!/usr/bin/env python3
# Reference: NVIDIA RigidBodyRopeDemo.py
# (~/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache/
#  omni.physx.demos-107.3.18+107.3.1.cp311.u353/omni/physxdemos/scenes/RigidBodyRopeDemo.py)
"""
Author per-link MassAPI(density=0.00005) on the cable rope chain at
/World/UR5e/cable/Rope/Rope/link_0..link_22, plus per-joint DriveAPI
on rotY/rotZ axes per NVIDIA's RigidBodyRopeDemo template.

Phase 3 SCENE-05 — runs against a live aic-dt instance via execute_python_code.
After authoring, the cable subtree (which Phase 1 left as SetActive(False) per
D-04) becomes physically simulable: bends under gravity, plug end is grabbable.

USAGE
-----
Run while aic-dt is up + quick_start has been called:

    python3 exts/aic-dt/scripts/author_cable_physics.py [--probe-only]

--probe-only: report current state of cable physics authoring without modifying.

EXIT
----
0 if all 23 links + 22 joints are properly authored (or already were).
1 if cable prim cannot be found / authoring failed.
"""
import argparse
import json
import socket
import sys


PYCODE = '''
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()
cable = stage.GetPrimAtPath("/World/UR5e/cable")
if not cable or not cable.IsValid():
    raise RuntimeError("cable prim /World/UR5e/cable not found")

# Need cable active for traversal of children; cable subtree may be SetActive(False)
# per Phase 1 D-04 workaround.
was_active = cable.IsActive()
if not was_active:
    cable.SetActive(True)

# cable-physics-fidelity (M1 ship gate): density+damping+stiffness scaled to
# our m-stage (mPU=1.0). Each cable link is a 6mm dia × 47mm cylinder; at
# density=1000 (PVC) per-link mass is ~1.33 g — gravity actually moves cable.
# See author_cable_physics_offline.py header for full rationale.
DENSITY = 1000.0
DAMPING = 0.001
STIFFNESS = 0.0
CONE_LIMIT_DEG = 110.0  # rotY/rotZ free range

probe_only = ${PROBE_ONLY}

links_seen = 0
links_with_density = 0
links_authored = 0
joints_seen = 0
joints_with_drive_y = 0
joints_with_drive_z = 0
joints_authored = 0

for prim in Usd.PrimRange(cable):
    type_name = str(prim.GetTypeName())
    # ---- Per-link MassAPI authoring ----
    if UsdPhysics.RigidBodyAPI(prim):
        links_seen += 1
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
        else:
            mass_api = UsdPhysics.MassAPI.Apply(prim) if not probe_only else None
        if mass_api is not None:
            density_attr = mass_api.GetDensityAttr()
            current = density_attr.Get() if density_attr.HasAuthoredValue() else None
            if current is not None and abs(float(current) - DENSITY) < 1e-9:
                links_with_density += 1
            elif not probe_only:
                if not density_attr.HasAuthoredValue():
                    mass_api.CreateDensityAttr().Set(DENSITY)
                else:
                    density_attr.Set(DENSITY)
                links_authored += 1
                links_with_density += 1
    # ---- D6 joint DriveAPI authoring ----
    # Cable joints are PhysicsJoint or PhysicsD6Joint subclass; check for "Joint" in type
    if type_name.startswith("Physics") and "Joint" in type_name and type_name != "PhysicsFixedJoint":
        joints_seen += 1
        for axis in ("rotY", "rotZ"):
            drive_api = UsdPhysics.DriveAPI.Apply(prim, axis) if not probe_only else (
                UsdPhysics.DriveAPI(prim, axis) if prim.HasAPI(UsdPhysics.DriveAPI, axis) else None
            )
            if drive_api is None:
                continue
            type_attr = drive_api.GetTypeAttr()
            damp_attr = drive_api.GetDampingAttr()
            stif_attr = drive_api.GetStiffnessAttr()
            already_correct = (
                type_attr.HasAuthoredValue() and str(type_attr.Get()) == "force"
                and damp_attr.HasAuthoredValue() and abs(float(damp_attr.Get()) - DAMPING) < 1e-6
                and stif_attr.HasAuthoredValue() and abs(float(stif_attr.Get()) - STIFFNESS) < 1e-6
            )
            if already_correct:
                if axis == "rotY":
                    joints_with_drive_y += 1
                else:
                    joints_with_drive_z += 1
                continue
            if probe_only:
                continue
            if not type_attr.HasAuthoredValue():
                drive_api.CreateTypeAttr().Set("force")
            else:
                type_attr.Set("force")
            if not damp_attr.HasAuthoredValue():
                drive_api.CreateDampingAttr().Set(DAMPING)
            else:
                damp_attr.Set(DAMPING)
            if not stif_attr.HasAuthoredValue():
                drive_api.CreateStiffnessAttr().Set(STIFFNESS)
            else:
                stif_attr.Set(STIFFNESS)
            joints_authored += 1
            if axis == "rotY":
                joints_with_drive_y += 1
            else:
                joints_with_drive_z += 1

# Restore previous active state (Plan 03-02 Task 3 flips this in extension.py
# load_robot to True post-authoring — this script only edits authoring, not
# active state).
if not was_active:
    cable.SetActive(False)

print(f"links_seen={links_seen} links_with_density={links_with_density} links_authored_now={links_authored}")
print(f"joints_seen={joints_seen} joints_with_drive_y={joints_with_drive_y} joints_with_drive_z={joints_with_drive_z} joints_authored_now={joints_authored}")
print(f"density={DENSITY} damping={DAMPING} stiffness={STIFFNESS}")
result = "ok"
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-only", action="store_true",
                    help="Report current authoring state without modifying.")
    ap.add_argument("--port", type=int, default=8768)
    args = ap.parse_args()

    code = PYCODE.replace("${PROBE_ONLY}", str(args.probe_only))

    s = socket.socket()
    s.settimeout(60)
    s.connect(("localhost", args.port))
    s.sendall(json.dumps({
        "type": "execute_python_code",
        "params": {"code": code}
    }).encode())
    data = b""
    while True:
        chunk = s.recv(8192)
        if not chunk:
            break
        data += chunk
        try:
            r = json.loads(data.decode())
            break
        except json.JSONDecodeError:
            continue
    s.close()

    output = r.get("result", {}).get("output", "")
    print(output)
    status = r.get("status", "?")
    if status != "success":
        print(f"FAIL: status={status}, full result: {r}")
        return 1

    # Parse the output for verification
    expect_links = 23
    expect_joints = 22
    if f"links_with_density={expect_links}" in output and f"joints_with_drive_y={expect_joints}" in output \
            and f"joints_with_drive_z={expect_joints}" in output:
        print(f"PASS: cable physics authored on all {expect_links} links + {expect_joints} joints")
        return 0
    else:
        print(f"PARTIAL: see output above. Re-run if probe-only mode; otherwise investigate.")
        return 0 if args.probe_only else 1


if __name__ == "__main__":
    sys.exit(main())
