# Standalone actuation + finger-gap check for the converted husarion RG2 USD.
#
# Validates the URDF->USD import BEFORE twin integration:
#   - the articulation loads + plays without exploding
#   - commanding the single actuated `rg2_gripper_joint` moves both fingers
#     SYMMETRICALLY via the PhysxMimicJointAPI coupling (the "mimic lag" gotcha)
#   - finger separation changes monotonically with joint angle theta
#
# Reusable as the seed of the live-Isaac contact-width parity sweep (task #6).
#
# Reference: Isaac Sim 5.1 standalone World API
#   https://docs.isaacsim.omniverse.nvidia.com/latest/
#
# Usage:
#   ~/env_isaaclab/bin/python scripts/test_husarion_rg2_actuation.py \
#       --usd exts/ur5e-dt/assets/gripper/husarion_rg2/husarion_rg2.usd \
#       --out /tmp/husarion_actuation.txt

import argparse
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import Articulation, XFormPrim  # noqa: E402
from pxr import UsdPhysics, UsdGeom, PhysxSchema  # noqa: E402
import omni.usd  # noqa: E402

ROOT = "/World/RG2"
ART_ROOT = f"{ROOT}/tool0"          # articulation root from the import
GRIP_JOINT = "rg2_gripper_joint"
L_FINGER = f"{ROOT}/rg2_l_finger_link"
R_FINGER = f"{ROOT}/rg2_r_finger_link"
# theta sweep: closed (-0.45) .. open (+0.78), per CAD stops
THETAS = [-0.45, -0.20, 0.0, 0.30, 0.60, 0.78]


def _disable_gravity_all(stage):
    """Pure-kinematic test: kill gravity on every rigid body so theta->gap is
    not contaminated by sag (the twin also disables arm/gripper-chain gravity)."""
    n = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateDisableGravityAttr().Set(True)
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--out", default="/tmp/husarion_actuation.txt")
    args = ap.parse_args()
    usd = os.path.abspath(os.path.expanduser(args.usd))

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd, ROOT)
    stage = omni.usd.get_context().get_stage()

    # Pin the flange to the world so the floating-base gripper doesn't drift.
    fj = UsdPhysics.FixedJoint.Define(stage, f"{ROOT}/world_fix")
    fj.GetBody1Rel().SetTargets([ART_ROOT])

    ngrav = _disable_gravity_all(stage)

    world.reset()
    art = Articulation(prim_paths_expr=ART_ROOT, name="rg2")
    art.initialize()
    # DOF-ordered names (matches get_joint_positions); joint_names also lists the
    # 3 fixed joints, which are NOT DOFs — using it to index the (n_dof) position
    # array overflows. dof_names is the correct, DOF-aligned list.
    jnames = list(getattr(art, "dof_names", None) or art.joint_names)

    lines = [f"dof_names ({len(jnames)}): {jnames}",
             f"all joint_names: {list(art.joint_names)}",
             f"disabled gravity on {ngrav} bodies", ""]
    if GRIP_JOINT not in jnames:
        lines.append(f"!! {GRIP_JOINT} not in joint_names — abort")
        _write(args.out, lines)
        simulation_app.close()
        return
    gidx = jnames.index(GRIP_JOINT)

    l_xf = XFormPrim(L_FINGER)
    r_xf = XFormPrim(R_FINGER)

    def settle(steps=90):
        for _ in range(steps):
            world.step(render=False)

    lines.append(f"{'cmd_theta':>10} {'act_theta':>10} {'finger_gap_mm':>14} "
                 f"{'mimic_max_err_rad':>18}")
    prev_gap = None
    mono = True
    for th in THETAS:
        # command only the actuated joint; mimics follow via the coupling
        q = art.get_joint_positions()
        q[0, gidx] = th
        from isaacsim.core.utils.types import ArticulationActions
        art.apply_action(ArticulationActions(
            joint_positions=np.array([[th]]),
            joint_indices=np.array([gidx])))
        settle()
        qa = art.get_joint_positions()[0]
        act = float(qa[gidx])
        # mimic tracking error: each finger joint should ~= +-theta
        mimic_err = 0.0
        for jn, mult in [("rg2_l_finger_2_joint", -1.0),
                         ("rg2_l_finger_passive_joint", 1.0),
                         ("rg2_r_finger_1_joint", -1.0),
                         ("rg2_r_finger_2_joint", 1.0),
                         ("rg2_r_finger_passive_joint", -1.0)]:
            if jn in jnames:
                e = abs(float(qa[jnames.index(jn)]) - mult * act)
                mimic_err = max(mimic_err, e)
        lp = l_xf.get_world_poses()[0][0]
        rp = r_xf.get_world_poses()[0][0]
        gap_mm = float(np.linalg.norm(np.array(lp) - np.array(rp))) * 1000.0
        lines.append(f"{th:10.3f} {act:10.4f} {gap_mm:14.2f} {mimic_err:18.5f}")
        if prev_gap is not None and gap_mm < prev_gap - 0.5:
            mono = False
        prev_gap = gap_mm

    lines.append("")
    lines.append(f"monotonic gap vs theta: {mono}")
    _write(args.out, lines)
    print(f"[test] wrote {args.out}")
    simulation_app.close()


def _write(path, lines):
    with open(path, "w") as f:
        f.write("\n".join(str(x) for x in lines) + "\n")


if __name__ == "__main__":
    main()
