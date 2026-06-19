# Standalone integration check for the husarion RG2 mount + actuation (pre-twin verify).
#
# Loads the ur5e-dt UR5e + the converted husarion gripper, runs the SAME single-fixed-weld
# math as extension.attach_rg2_to_ur5e (wrist_3 -> tool0, reusing the verified quat0/quat1
# at-flange rotation, pre-seat, <0.1mm/<0.1deg delta gate), plays, commands rg2_gripper_joint,
# and reports: weld delta, post-play tool0 drift, joint tracking, finger-link gap. This isolates
# the gripper integration from the full quick_start scene.
#
# Reference: Isaac Sim 5.1 World API. Mirrors extension.py constants/math.

import argparse
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # noqa: E402
import omni.usd  # noqa: E402

ASSETS = os.path.expanduser("~/Documents/isaac-sim-mcp/exts/ur5e-dt/assets")
UR5E_USD = f"{ASSETS}/robot/ur5e.usd"
RG2_USD = f"{ASSETS}/gripper/husarion_rg2/husarion_rg2_fixed.usd"  # geometry-correct (has collision meshes)
RG2_TOOL0 = "/World/RG2_Gripper/tool0"
WRIST3 = "/World/UR5e/wrist_3_link"
ACT_JOINT = "rg2_gripper_joint"
MIMIC_JOINTS = ["rg2_l_finger_2_joint", "rg2_l_finger_passive_joint",
                "rg2_r_finger_1_joint", "rg2_r_finger_2_joint", "rg2_r_finger_passive_joint"]
L_FINGER, R_FINGER = "/World/RG2_Gripper/rg2_l_finger_link", "/World/RG2_Gripper/rg2_r_finger_link"
THETAS = [-0.45, 0.0, 0.3, 0.78]


def euler_to_quatf(x, y, z):
    rx = Gf.Quatf(math.cos(math.radians(x)/2), Gf.Vec3f(1, 0, 0)*math.sin(math.radians(x)/2))
    ry = Gf.Quatf(math.cos(math.radians(y)/2), Gf.Vec3f(0, 1, 0)*math.sin(math.radians(y)/2))
    rz = Gf.Quatf(math.cos(math.radians(z)/2), Gf.Vec3f(0, 0, 1)*math.sin(math.radians(z)/2))
    return rx * ry * rz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/husarion_mount.txt")
    args = ap.parse_args()
    lines = []

    world = World(stage_units_in_meters=1.0)
    if os.environ.get("RG2_GROUND", "1") == "1":
        world.scene.add_default_ground_plane()
    add_reference_to_stage(UR5E_USD, "/World/UR5e")
    add_reference_to_stage(RG2_USD, "/World/RG2_Gripper")
    stage = omni.usd.get_context().get_stage()

    # mimic spring (the validated rigidity fix) — nf via env for tuning
    MIMIC_NF = float(os.environ.get("RG2_NF", "500"))
    for jn in MIMIC_JOINTS:
        p = stage.GetPrimAtPath(f"/World/RG2_Gripper/joints/{jn}")
        if p and p.IsValid():
            for a, v in (("naturalFrequency", MIMIC_NF), ("dampingRatio", 1.0)):
                at = p.GetAttribute(f"physxMimicJoint:rotZ:{a}")
                if at:
                    at.Set(v)

    # Actuated-joint drive: the imported gains were sized for the weak default mimic spring;
    # with stiff mimics the actuated joint must have enough authority to drive the coupled
    # parallelogram across the full range. Tune via env.
    act_jp = stage.GetPrimAtPath(f"/World/RG2_Gripper/joints/{ACT_JOINT}")
    drv = UsdPhysics.DriveAPI.Get(act_jp, "angular") or UsdPhysics.DriveAPI.Apply(act_jp, "angular")
    A_STIFF = float(os.environ.get("RG2_ASTIFF", "10000"))
    A_DAMP = float(os.environ.get("RG2_ADAMP", "1000"))
    A_FMAX = float(os.environ.get("RG2_AFMAX", "1000"))
    drv.CreateStiffnessAttr().Set(A_STIFF)
    drv.CreateDampingAttr().Set(A_DAMP)
    drv.CreateMaxForceAttr().Set(A_FMAX)
    lines.append(f"mimic nf={MIMIC_NF} | act drive stiffness={A_STIFF} damping={A_DAMP} maxForce={A_FMAX}")

    # --- weld math (mirror of attach_rg2_to_ur5e) ---
    quat0 = euler_to_quatf(-90, 0, -90)
    quat1 = euler_to_quatf(-180, 90, 0)
    jp = stage.DefinePrim("/World/UR5e/joints/robot_gripper_joint", "PhysicsFixedJoint")
    jp.CreateRelationship("physics:body0").SetTargets([Sdf.Path(WRIST3)])
    jp.CreateRelationship("physics:body1").SetTargets([Sdf.Path(RG2_TOOL0)])
    jp.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    jp.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)
    jp.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f, custom=True).Set(Gf.Vec3f(0, 0, 0))
    jp.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f, custom=True).Set(Gf.Vec3f(0, 0, 0))
    jp.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat0)
    jp.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat1)

    def _rot_m(qf):
        m = Gf.Matrix4d(1.0)
        m.SetRotateOnly(Gf.Quatd(float(qf.GetReal()), Gf.Vec3d(qf.GetImaginary())))
        return m
    w3 = stage.GetPrimAtPath(WRIST3)
    tool0 = stage.GetPrimAtPath(RG2_TOOL0)
    w3_l2w = UsdGeom.Xformable(w3).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    J = _rot_m(quat1).GetInverse() * _rot_m(quat0) * w3_l2w
    tool0_local = UsdGeom.Xformable(tool0).GetLocalTransformation()
    root_seed = tool0_local.GetInverse() * J
    xf = UsdGeom.Xform(stage.GetPrimAtPath("/World/RG2_Gripper"))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(root_seed.ExtractTranslation())
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(root_seed.ExtractRotationQuat())

    tool0_now = UsdGeom.Xformable(tool0).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    dpos = (tool0_now.ExtractTranslation() - J.ExtractTranslation()).GetLength() * 1000.0
    qn = tool0_now.ExtractRotationQuat().GetNormalized()
    qd = J.ExtractRotationQuat().GetNormalized()
    dang = math.degrees(2.0 * math.acos(min(1.0, abs((qd.GetInverse()*qn).GetReal()))))
    Jpos = J.ExtractTranslation()
    lines.append(f"WELD DELTA (pre-play): dpos={dpos:.5f}mm dang={dang:.5f}deg "
                 f"(gate <0.1/<0.1) -> {'PASS' if dpos < 0.1 and dang < 0.1 else 'FAIL'}")
    lines.append(f"flange/tool0 world pos (m): ({Jpos[0]:.4f},{Jpos[1]:.4f},{Jpos[2]:.4f})")

    # Match the twin: disable gravity on the robot+gripper chain (position-mirroring twin).
    from pxr import PhysxSchema
    ng = 0
    for root in ("/World/UR5e", "/World/RG2_Gripper"):
        rp = stage.GetPrimAtPath(root)
        if rp and rp.IsValid():
            for prim in Usd.PrimRange(rp):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(True)
                    ng += 1
    lines.append(f"disabled gravity on {ng} robot+gripper bodies (twin parity)")

    # --- play + actuation ---
    world.reset()
    grip = Articulation(prim_paths_expr=RG2_TOOL0, name="rg2")
    grip.initialize()
    jn = list(getattr(grip, "dof_names", None) or grip.joint_names)
    gidx = jn.index(ACT_JOINT)
    for _ in range(30):
        world.step(render=False)

    # ===================== WIDTH-CONTROL FIDELITY IN A GRASP CONTEXT =====================
    # Measure the RUBBER-PAD CONTACT gap (aruco datum markers, finger-link LOCAL, metres):
    #   L rg2_l_finger_link (+0.0379, -0.01898, 0) ; R rg2_r_finger_link (+0.0379, +0.01898, 0)
    # and compare the achieved pad gap to the COMMANDED CONTACT width (the fidelity metric).
    import json, sys
    sys.path.insert(0, os.path.expanduser("~/Documents/ros-mcp-server"))
    from onrobot_rg2_sim_control.onrobot_rg2_sim_control.rg2_arc import Rg2Arc
    arc = Rg2Arc()
    PAD_L = Gf.Vec3d(0.0379, -0.01898, 0.0)
    PAD_R = Gf.Vec3d(0.0379, +0.01898, 0.0)
    Lx = UsdGeom.Xformable(stage.GetPrimAtPath(L_FINGER))
    Rx = UsdGeom.Xformable(stage.GetPrimAtPath(R_FINGER))
    def pad_gap_mm():
        lw = Lx.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(PAD_L)
        rw = Rx.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(PAD_R)
        return (Gf.Vec3d(lw) - Gf.Vec3d(rw)).GetLength() * 1000.0
    def theta_for_contact(w_mm):
        return arc.theta_of_raw_width(arc.raw_of_rubber(w_mm))
    def go(theta, n=150):
        grip.apply_action(ArticulationActions(joint_positions=np.array([[theta]]), joint_indices=np.array([gidx])))
        for _ in range(n): world.step(render=False)
        return float(grip.get_joint_positions()[0][gidx])

    CAND = os.path.expanduser("~/Documents/aruco-grasp-annotator/data/grasp_candidates/hex_red_grasp_candidates.json")
    cands = json.load(open(CAND))["grasp_candidates"]
    CLEAR = 14.0  # gripper clearance_mm (W_grip = width_mm - clearance)
    lines.append("")
    lines.append(f"{'cand':>6} {'open_w':>7} {'approach_padgap':>15} {'W_grip':>7} {'grip_padgap':>12} {'theta_act':>10} {'err_mm':>7}")
    for c in cands:
        cid = f"{c['grasp_point_id']}.{c['direction_id']}"
        open_w = float(c["width_mm"])
        wgrip = open_w - CLEAR
        # approach: command the open contact width, measure achieved pad gap
        go(theta_for_contact(open_w)); appad = pad_gap_mm()
        # grip: command W_grip contact width, measure achieved pad gap
        ta = go(theta_for_contact(wgrip)); gpad = pad_gap_mm()
        err = gpad - wgrip
        lines.append(f"{cid:>6} {open_w:7.1f} {appad:15.2f} {wgrip:7.1f} {gpad:12.2f} {ta:10.4f} {err:7.2f}")
    lines.append("")
    lines.append("EXPECTED (PICKPLACE-EXPECTED.md): cand 1.1 open 24.4 -> W_grip 10.4 ; cand 2.1 open 30.2 -> W_grip 16.2")
    lines.append("FIDELITY = grip_padgap should == W_grip within ~1mm (force-close compression).")

    with open(args.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[grasp] wrote {args.out}")
    simulation_app.close()


if __name__ == "__main__":
    main()
