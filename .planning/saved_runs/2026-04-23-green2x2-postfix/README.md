# Saved run bundle — green_2x2 pick-place, post contactOffset+CCD fix

**Date:** 2026-04-23 evening
**Context:** After landing the USD patch that sets contactOffset = 0.1 mm on
all 7 robot collision Xforms + CCD on gripper/jaw/wrist, AND the runtime
patch in `extension.py` that sets contactOffset = 0.1 mm on cup meshes at
creation time.

**Isaac Sim config at capture time:**
- Robot collisions: `approximation = convexDecomposition` on all 7 links
- Robot collisions: `physxCollision:contactOffset = 0.0001 m (0.1 mm)`
- Robot rigid bodies: `physxRigidBody:enableCCD = true` on gripper_link,
  moving_jaw_so101_v1_link, wrist_link
- Cup collisions: `approximation = convexDecomposition`,
  `physxCollision:contactOffset = 0.0001 m (0.1 mm)` (newly authored)
- Effective combined robot↔cup contact zone: **0.2 mm** (sum of both sides)

## What's in this bundle

- `20260423T2002*` and `20260423T2003*` — pick-place cycle #1 (knocked cup)
- `20260423T2004*` — re-home after cycle #1
- `20260423T2006*` and `20260423T2007*` — pick-place cycle #2 (knocked cup)
- `so_arm101_control.log` — full ROS2 control-stack log for the session

For each motion, there are three file types:
- `<ts>_<tag>.csv` — 50 Hz telemetry (joint plan/Isaac/JTC state, cup poses,
  lego poses, physics rate)
- `<ts>_<tag>.json` — summary metadata (peak lag, cup_disp, realtime_factor)
- `<ts>_<n>_<tag>.json` — plan dump from `/tmp/arm_traj/` (joint_names,
  waypoints, scene_at_plan_time)

## Key findings from this run

| Motion | peak_lag | cup_disp_mm | Notes |
|---|---|---|---|
| 20:02:07 grasp_home | 17.80° | 0.00 | initial home |
| 20:02:15 grasp_move | 8.11°  | 0.00 | approach green_2x2 |
| 20:02:18 grasp_move | 4.50°  | 0.00 | descent |
| 20:02:25 grasp_home | 9.42°  | 0.00 | with attached |
| 20:02:32 drop_point | 18.50° | 0.00 | move to cup |
| 20:03:07 drop_sweep | 13.51° | 0.00 | tilt wrist |
| 20:03:14 drop_release | 44.30° | **26.30** | ⚠ cup shifted during release |
| 20:03:58 grasp_home | 82.52° | **71.33** | ⚠ cup knocked 71 mm on return |
| 20:04:46 grasp_home | 19.63° | 0.00 | recovery home |
| 20:06:42 grasp_move | 16.06° | 0.00 | cycle 2 start |
| 20:06:46 grasp_move | 5.03°  | 0.00 | |
| 20:06:55 grasp_home | 16.63° | 0.00 | |
| 20:07:03 drop_point | 19.61° | 0.00 | |
| 20:07:10 drop_sweep | 9.04°  | 0.00 | |
| 20:07:21 grasp_home | 19.57° | **71.52** | ⚠ cup knocked 71 mm again |

**Verifier analysis** of `20260423T200721_grasp_home.csv`:
- Min clearance: **2.16 mm** (jaw ↔ cup_drop_2 at t=1.132s)
- Expected from 0.2 mm combined contact zone: ZERO force at 2.16 mm clearance
- Observed: 71 mm cup displacement

## Open questions for later investigation

1. **Why is the cup still being knocked at 2.16 mm clearance with 0.2 mm
   combined contact zone?** Three hypotheses:
   - **CCD swept-volume force**: CCD on gripper/jaw/wrist performs
     swept-volume collision detection. A fast lateral arm motion (~0.5 m/s)
     between two 120 Hz physics steps (8.3 ms) sweeps ~4 mm of volume. If
     the swept volume overlaps a cup's convex-decomposition piece, PhysX
     generates contact force even when START and END clearance are 2+ mm.
     This would match the symptom: knock intensity dropped 267 → 96 → 71 mm
     as we tightened contactOffset, but non-zero remains because CCD is
     separate.
   - **Cup convex-decomp overhang**: Our verifier approximates the cup
     boundary as a 39 mm-radius solid cylinder. PhysX's actual convex
     decomposition may have pieces that extend slightly beyond (hull of a
     hollow ring naturally includes small overhang). Actual clearance to
     nearest convex-piece surface could be sub-0.1 mm.
   - **Lego-in-cup interaction**: drop_release at 20:03:14 showed 26 mm of
     cup displacement — the dropped lego may be exerting force from inside
     the cup, tilting/shifting it. Then the grasp_home sweep pushes an
     already-unstable cup further.

2. **OMPL "fails to find target" intermittent behavior.** This run was
   notably free of that particular failure (user comment: "this run I
   didn't have to keep clicking the same button due overcome the ompl
   planning"). Check the plan dumps (`/tmp/arm_traj/*_grasp_home.json`)
   for the `points` arrays — compare against sessions where OMPL failed
   to find a valid path. Look for:
   - `scene_at_plan_time.drop_data` cup positions (knocked cups move)
   - `start_ref_pos` — if the arm is at an extreme joint config when
     grasp_home fires, IK search space may be empty
   - Whether `attached_lego_name` is present — attached mass might push
     RRTConnect into infeasible regions

## Tools to re-analyze

```bash
# Summary table of all motions
scripts/motion_log.sh scan

# Per-motion detail (min clearance, PhysX contact-zone check)
scripts/motion_sweep.py .planning/saved_runs/2026-04-23-green2x2-postfix/20260423T200721_grasp_home.csv --resolution 2

# Full MoveIt replay (requires live move_group, won't work from archive alone)
scripts/motion_log.sh verify <csv>
```

## Next-session hypotheses to test

1. **Disable CCD, keep 0.1 mm contactOffset.** If knocks disappear → CCD
   swept-volume is the source. If knocks persist → convex-decomp overhang
   or lego-in-cup is the cause.
2. **Inspect cup convex decomposition geometry live** — walk the
   PhysxConvexDecompositionCollisionAPI-generated pieces and check their
   actual world-space extents vs. the 39 mm nominal radius.
3. **Make cup STATIC (kinematic) during non-drop motions** — cups don't
   need to be dynamic rigid bodies except during the actual drop. Setting
   `kinematicEnabled = True` outside of drop windows would make them
   immune to contact impulses.
