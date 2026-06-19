# Settle the PhysX mimic-lag fix for the husarion RG2 BEFORE touching the twin.
#
# GPT plan-gate (gpt-5.5) flagged: do NOT slap a 1e9 DriveAPI on the mimic joints
# (a fixed-target drive fights the PhysxMimicJointAPI constraint gearing*ref+offset=0
# everywhere except that pose). Principled levers instead:
#   - articulation solver POSITION-iteration count (+ velocity iters)
#   - the mimic constraint's own attrs (gearing/offset/naturalFrequency/damping)
#   - armature conditioning on light finger links
#
# This script (1) DUMPS the PhysxMimicJointAPI attrs the importer authored, and
# (2) sweeps solverPositionIterationCount, measuring steady-state mimic-tracking
# error at the worst-case hold (theta=0.6). Output -> file (close() drops stdout).
#
# Reference: PhysxMimicJointAPI / articulation solver iteration semantics
#   https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/

import argparse
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from pxr import UsdPhysics, PhysxSchema  # noqa: E402
import omni.usd  # noqa: E402

ROOT = "/World/RG2"
ART_ROOT = f"{ROOT}/tool0"
GRIP_JOINT = "rg2_gripper_joint"
MIMIC_JOINTS = ["rg2_l_finger_2_joint", "rg2_l_finger_passive_joint",
                "rg2_r_finger_1_joint", "rg2_r_finger_2_joint",
                "rg2_r_finger_passive_joint"]
MIMIC_MULT = {"rg2_l_finger_2_joint": -1.0, "rg2_l_finger_passive_joint": 1.0,
              "rg2_r_finger_1_joint": -1.0, "rg2_r_finger_2_joint": 1.0,
              "rg2_r_finger_passive_joint": -1.0}
HOLD_THETA = 0.6
# The importer authors the mimic constraint as a weakly-damped spring
# (naturalFrequency=25 Hz, dampingRatio=0.005) -> it rings/lags. Sweep the
# constraint's OWN stiffness/damping (the principled lever, NOT a fighting
# DriveAPI) to find rigid tracking. (nf_hz, damping_ratio).
SPRING_COMBOS = [(25.0, 0.005), (100.0, 0.7), (250.0, 1.0),
                 (500.0, 1.0), (1000.0, 1.0), (2000.0, 1.0)]


def _dump_mimic_attrs(stage, lines):
    lines.append("=== PhysxMimicJointAPI attrs (per mimic joint) ===")
    for jn in MIMIC_JOINTS:
        p = stage.GetPrimAtPath(f"{ROOT}/joints/{jn}")
        if not p or not p.IsValid():
            lines.append(f"  {jn}: PRIM MISSING")
            continue
        attrs = {a.GetName(): a.Get() for a in p.GetAttributes()
                 if "imic" in a.GetName() or "earing" in a.GetName()
                 or "ffset" in a.GetName() or "atural" in a.GetName()
                 or "amping" in a.GetName()}
        rels = {r.GetName(): r.GetTargets() for r in p.GetRelationships()
                if "imic" in r.GetName() or "eference" in r.GetName()}
        lines.append(f"  {jn}: attrs={attrs} rels={rels}")
    lines.append("")


def _set_mimic_spring(stage, nf_hz, damping_ratio):
    for jn in MIMIC_JOINTS:
        p = stage.GetPrimAtPath(f"{ROOT}/joints/{jn}")
        if p and p.IsValid():
            nf = p.GetAttribute("physxMimicJoint:rotZ:naturalFrequency")
            dr = p.GetAttribute("physxMimicJoint:rotZ:dampingRatio")
            if nf:
                nf.Set(float(nf_hz))
            if dr:
                dr.Set(float(damping_ratio))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--out", default="/tmp/husarion_mimic_rigidity.txt")
    args = ap.parse_args()
    usd = os.path.abspath(os.path.expanduser(args.usd))

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd, ROOT)
    stage = omni.usd.get_context().get_stage()

    fj = UsdPhysics.FixedJoint.Define(stage, f"{ROOT}/world_fix")
    fj.GetBody1Rel().SetTargets([ART_ROOT])
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(True)

    lines = []
    _dump_mimic_attrs(stage, lines)

    lines.append(f"=== mimic tracking @ theta={HOLD_THETA} vs mimic spring "
                 f"(naturalFreq, dampingRatio) ===")
    lines.append("steady_err = mean |mimic - mult*theta| over last 40 steps; "
                 "osc = max-min of that error over last 40 (ringing amplitude)")
    lines.append(f"{'nf_hz':>8} {'damp':>6} {'act_theta':>10} "
                 f"{'steady_err_deg':>15} {'osc_deg':>10}")

    for nf, dr in SPRING_COMBOS:
        _set_mimic_spring(stage, nf, dr)
        world.reset()
        art = Articulation(prim_paths_expr=ART_ROOT, name=f"rg2_{int(nf)}_{dr}")
        art.initialize()
        jn = list(getattr(art, "dof_names", None) or art.joint_names)
        gidx = jn.index(GRIP_JOINT)
        art.apply_action(ArticulationActions(
            joint_positions=np.array([[HOLD_THETA]]),
            joint_indices=np.array([gidx])))
        for _ in range(160):
            world.step(render=False)
        # sample the tracking error over the last 40 steps to separate steady
        # lag from oscillation phase
        errs = []
        for _ in range(40):
            world.step(render=False)
            qa = art.get_joint_positions()[0]
            act = float(qa[gidx])
            e = max(abs(float(qa[jn.index(mj)]) - mult * act)
                    for mj, mult in MIMIC_MULT.items() if mj in jn)
            errs.append(e)
        errs = np.array(errs)
        steady = float(errs.mean())
        osc = float(errs.max() - errs.min())
        lines.append(f"{nf:8.0f} {dr:6.2f} {act:10.4f} "
                     f"{np.degrees(steady):15.4f} {np.degrees(osc):10.4f}")
        world.stop()

    with open(args.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[mimic] wrote {args.out}")
    simulation_app.close()


if __name__ == "__main__":
    main()
