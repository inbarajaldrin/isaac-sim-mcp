# Live-Isaac CONTACT-width parity sweep for the husarion RG2 twin (the endpoint test).
#
# Commands a set of CONTACT widths through the SAME CAD arc the rg2_sim backend uses
# (contact -> theta), applies theta to the twin's rg2_gripper_joint, and measures the
# achieved RUBBER PAD-to-PAD gap directly in Isaac at aruco-runner's pad-contact-face
# markers (finger-link-local, constant; the parallelogram keeps the links axis-aligned):
#     L rg2_l_finger_link: (X=+37.9, Y=-18.98, Z=0) mm
#     R rg2_r_finger_link: (X=+37.9, Y=+18.98, Z=0) mm
# gap_contact = inter-marker distance. Reports per-theta (gap_contact, resid_L, resid_R)
# and asserts |cmd_contact - achieved_gap_contact| <= 1mm (the CAD budget).
#
# HOME SELF-CHECK FIRST (aruco's gate): at theta=0, measured gap_contact MUST equal
# arc.contact_of_theta(0) (~47.6mm CAD) — if it doesn't, the marker/frame is wrong and the
# sweep is aborted before producing misleading numbers.
#
# Standalone (own headless Isaac): measures the twin's PHYSICAL theta->gap response, which is
# the only part the arc can't prove on its own (the contact<->theta round-trip is pure math).

import argparse
import math
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf  # noqa: E402
import omni.usd  # noqa: E402

# the CAD arc (single source of truth, shared with the rg2_sim backend)
sys.path.insert(0, os.path.expanduser("~/Documents/ros-mcp-server"))
from onrobot_rg2_sim_control.onrobot_rg2_sim_control.rg2_arc import Rg2Arc, Rg2ArcParams  # noqa: E402

ASSETS = os.path.expanduser("~/Documents/isaac-sim-mcp/exts/ur5e-dt/assets")
UR5E_USD = f"{ASSETS}/robot/ur5e.usd"
RG2_USD = f"{ASSETS}/gripper/husarion_rg2/husarion_rg2.usd"
YAML = os.path.expanduser("~/Documents/ros-mcp-server/onrobot_rg2_sim_control/config/rg2_gripper.yaml")
RG2_TOOL0 = "/World/RG2_Gripper/tool0"
WRIST3 = "/World/UR5e/wrist_3_link"
ACT_JOINT = "rg2_gripper_joint"
MIMIC = {"rg2_l_finger_2_joint": -1.0, "rg2_l_finger_passive_joint": 1.0,
         "rg2_r_finger_1_joint": -1.0, "rg2_r_finger_2_joint": 1.0, "rg2_r_finger_passive_joint": -1.0}
L_MIMIC = ["rg2_l_finger_2_joint", "rg2_l_finger_passive_joint"]
R_MIMIC = ["rg2_r_finger_1_joint", "rg2_r_finger_2_joint", "rg2_r_finger_passive_joint"]
L_FINGER, R_FINGER = "/World/RG2_Gripper/rg2_l_finger_link", "/World/RG2_Gripper/rg2_r_finger_link"
# aruco pad-contact-face markers, finger-link-LOCAL, METERS
PAD_L = Gf.Vec3d(0.0379, -0.01898, 0.0)
PAD_R = Gf.Vec3d(0.0379, 0.01898, 0.0)
CONTACT_CMDS_MM = [0, 10, 35, 55, 75, 100]
NF, DAMP = 1000.0, 1.0
A_STIFF, A_DAMP, A_FMAX = 10000.0, 1000.0, 1000.0


def euler_to_quatf(x, y, z):
    rx = Gf.Quatf(math.cos(math.radians(x)/2), Gf.Vec3f(1, 0, 0)*math.sin(math.radians(x)/2))
    ry = Gf.Quatf(math.cos(math.radians(y)/2), Gf.Vec3f(0, 1, 0)*math.sin(math.radians(y)/2))
    rz = Gf.Quatf(math.cos(math.radians(z)/2), Gf.Vec3f(0, 0, 1)*math.sin(math.radians(z)/2))
    return rx * ry * rz


TIP_BELOW_FACE_MM = 14.9  # fingertip protrusion below the pad contact face (config.py)


def pad_world(stage):
    lt = UsdGeom.Xformable(stage.GetPrimAtPath(L_FINGER)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rt = UsdGeom.Xformable(stage.GetPrimAtPath(R_FINGER)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return lt.Transform(PAD_L), rt.Transform(PAD_R)


def pad_gap_mm(stage):
    lw, rw = pad_world(stage)
    return (lw - rw).GetLength() * 1000.0


def tool0_to_padface_depth_mm(stage):
    """Depth from tool0 origin to the grasp centre (pad contact face midpoint), projected
    onto tool0's reach (+Z) axis — the descent quantity config.py's SIM_FINGERTIP_MM feeds."""
    lw, rw = pad_world(stage)
    center = (lw + rw) * 0.5
    t = UsdGeom.Xformable(stage.GetPrimAtPath(RG2_TOOL0)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    origin = t.ExtractTranslation()
    zaxis = Gf.Vec3d(t[2][0], t[2][1], t[2][2]).GetNormalized()  # tool0 +Z in world (row-major)
    return float(Gf.Dot(center - origin, zaxis)) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/husarion_parity.txt")
    args = ap.parse_args()
    lines = []

    try:
        arc = Rg2Arc(Rg2ArcParams.from_yaml(YAML))
        lines.append(f"arc params from {YAML}")
    except Exception as e:
        arc = Rg2Arc()
        lines.append(f"yaml load failed ({e}); using CAD defaults")

    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(UR5E_USD, "/World/UR5e")
    add_reference_to_stage(RG2_USD, "/World/RG2_Gripper")
    stage = omni.usd.get_context().get_stage()

    # mimic spring + strong actuated drive (the validated rigidity + authority fix)
    for jn in MIMIC:
        p = stage.GetPrimAtPath(f"/World/RG2_Gripper/joints/{jn}")
        if p and p.IsValid():
            for a, v in (("naturalFrequency", NF), ("dampingRatio", DAMP)):
                at = p.GetAttribute(f"physxMimicJoint:rotZ:{a}")
                if at:
                    at.Set(v)
    actp = stage.GetPrimAtPath(f"/World/RG2_Gripper/joints/{ACT_JOINT}")
    drv = UsdPhysics.DriveAPI.Get(actp, "angular") or UsdPhysics.DriveAPI.Apply(actp, "angular")
    drv.CreateStiffnessAttr().Set(A_STIFF); drv.CreateDampingAttr().Set(A_DAMP); drv.CreateMaxForceAttr().Set(A_FMAX)

    # single fixed weld wrist_3 -> husarion tool0 (verified quat0/quat1; pre-seat)
    quat0, quat1 = euler_to_quatf(-90, 0, -90), euler_to_quatf(-180, 90, 0)
    jp = stage.DefinePrim("/World/UR5e/joints/robot_gripper_joint", "PhysicsFixedJoint")
    jp.CreateRelationship("physics:body0").SetTargets([Sdf.Path(WRIST3)])
    jp.CreateRelationship("physics:body1").SetTargets([Sdf.Path(RG2_TOOL0)])
    jp.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)
    jp.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat0)
    jp.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat1)

    def _rot_m(qf):
        m = Gf.Matrix4d(1.0); m.SetRotateOnly(Gf.Quatd(float(qf.GetReal()), Gf.Vec3d(qf.GetImaginary()))); return m
    w3 = stage.GetPrimAtPath(WRIST3); tool0 = stage.GetPrimAtPath(RG2_TOOL0)
    w3_l2w = UsdGeom.Xformable(w3).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    J = _rot_m(quat1).GetInverse() * _rot_m(quat0) * w3_l2w
    root_seed = UsdGeom.Xformable(tool0).GetLocalTransformation().GetInverse() * J
    xf = UsdGeom.Xform(stage.GetPrimAtPath("/World/RG2_Gripper"))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(root_seed.ExtractTranslation())
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(root_seed.ExtractRotationQuat())

    # gravity off on the chain (twin parity)
    for root in ("/World/UR5e", "/World/RG2_Gripper"):
        rp = stage.GetPrimAtPath(root)
        if rp and rp.IsValid():
            for prim in Usd.PrimRange(rp):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(True)

    world.reset()
    grip = Articulation(prim_paths_expr=RG2_TOOL0, name="rg2")
    grip.initialize()
    jn = list(getattr(grip, "dof_names", None) or grip.joint_names)
    gidx = jn.index(ACT_JOINT)

    def settle_to(theta, steps=160):
        grip.apply_action(ArticulationActions(joint_positions=np.array([[theta]]),
                                              joint_indices=np.array([gidx])))
        for _ in range(steps):
            world.step(render=False)

    def resid(side_joints):
        qa = grip.get_joint_positions()[0]
        act = float(qa[gidx])
        return max(abs(float(qa[jn.index(j)]) - MIMIC[j] * act) for j in side_joints if j in jn)

    # --- HOME SELF-CHECK (theta=0): measured pad gap must equal arc.contact_of_theta(0) ---
    settle_to(0.0)
    home_meas = pad_gap_mm(stage)
    home_cad = arc.contact_of_theta(0.0)
    home_err = abs(home_meas - home_cad)
    lines.append(f"\nHOME SELF-CHECK @theta=0: measured pad gap {home_meas:.2f}mm vs "
                 f"CAD contact {home_cad:.2f}mm -> err {home_err:.2f}mm "
                 f"({'PASS' if home_err <= 1.0 else 'FAIL — marker/frame off, sweep ABORTED'})")
    if home_err > 1.0:
        _write(args.out, lines)
        print(f"[parity] HOME SELF-CHECK FAILED ({home_err:.2f}mm); aborting. wrote {args.out}")
        simulation_app.close()
        return

    # --- SWEEP (gap parity + descent-depth curve for config.py) ---
    lines.append("\n=== CONTACT-WIDTH PARITY SWEEP ===")
    lines.append(f"{'cmd_mm':>7} {'theta':>8} {'cad_contact':>12} {'meas_gap':>10} "
                 f"{'|cmd-meas|':>11} {'resid_L_deg':>12} {'resid_R_deg':>12} {'pass':>6}")
    worst = 0.0
    depth_rows = []
    for cmd in CONTACT_CMDS_MM:
        theta = arc.theta_of_contact(float(cmd))
        settle_to(theta)
        meas = pad_gap_mm(stage)
        cad_c = arc.contact_of_theta(theta)
        err = abs(cmd - meas)
        worst = max(worst, err)
        rL, rR = math.degrees(resid(L_MIMIC)), math.degrees(resid(R_MIMIC))
        lines.append(f"{cmd:7d} {theta:+8.4f} {cad_c:12.2f} {meas:10.2f} {err:11.2f} "
                     f"{rL:12.4f} {rR:12.4f} {'OK' if err <= 1.0 else 'OVER':>6}")
        # descent depth: tool0->pad-face (measured) ; tool0->fingertip = +14.9 ; CAD flange->tip = arc.depth
        padface = tool0_to_padface_depth_mm(stage)
        tip = padface + TIP_BELOW_FACE_MM
        cad_flange_tip = arc.depth_mm(theta)
        depth_rows.append((cmd, padface, tip, cad_flange_tip))
    lines.append(f"\nworst |cmd-meas| = {worst:.2f}mm  -> "
                 f"{'PASS (<=1mm everywhere)' if worst <= 1.0 else 'FAIL'}")

    # --- DESCENT-DEPTH CURVE (for config.py SIM_FINGERTIP_MM = tool0->fingertip) ---
    # const = tool0->fingertip(meas) - flange->tip(CAD); should be ~constant (= tool0 vs flange frame).
    lines.append("\n=== DESCENT DEPTH (config.py SIM_FINGERTIP_MM rebuild) ===")
    lines.append(f"{'W_mm':>6} {'tool0_padface':>14} {'tool0_tip':>10} {'cad_flange_tip':>15} {'const(tip-cad)':>15}")
    consts = []
    for cmd, padface, tip, cad in depth_rows:
        consts.append(tip - cad)
        lines.append(f"{cmd:6d} {padface:14.2f} {tip:10.2f} {cad:15.2f} {tip - cad:15.2f}")
    cmean = sum(consts) / len(consts)
    cspread = max(consts) - min(consts)
    lines.append(f"\ntool0->flange const: mean {cmean:.2f}mm, spread {cspread:.2f}mm "
                 f"({'OK <=1mm (CAD curve + const single-sources depth)' if cspread <= 1.0 else 'SPREAD >1mm — frame issue'})")
    lines.append("SIM_FINGERTIP_MM (new husarion twin, tool0->fingertip mm):")
    lines.append("  " + ", ".join(f"({cmd:.2f}, {tip:.2f})" for cmd, _, tip, _ in depth_rows))
    _write(args.out, lines)
    print(f"[parity] wrote {args.out}")
    simulation_app.close()


def _write(path, lines):
    with open(path, "w") as f:
        f.write("\n".join(str(x) for x in lines) + "\n")


if __name__ == "__main__":
    main()
