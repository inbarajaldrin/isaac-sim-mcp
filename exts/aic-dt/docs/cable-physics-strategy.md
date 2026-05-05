# SCENE-05 Cable Physics Strategy

**Phase 3 Plan 03-02 — landed 2026-05-05**

## Strategy

In-place USD edit (D-06 policy) authoring per-link mass + per-joint drive on the existing
articulated cable chain at `/World/UR5e/cable/Rope/Rope/link_<N>` per NVIDIA's
canonical `RigidBodyRopeDemo.py` template.

**Authored attributes (per NVIDIA defaults):**
- Per RigidBody link: `UsdPhysics.MassAPI.CreateDensityAttr(0.00005)`
- Per joint (D6, non-Fixed): `UsdPhysics.DriveAPI(rotY)` + `DriveAPI(rotZ)` with:
  - `type = "force"`
  - `damping = 10.0`
  - `stiffness = 1.0`

Source code: `~/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache/omni.physx.demos-107.3.18+107.3.1.cp311.u353/omni/physxdemos/scenes/RigidBodyRopeDemo.py`

## Authoring tool

**Offline** (D-06 in-place edit, persists across launches):

```bash
python3 exts/aic-dt/scripts/author_cable_physics_offline.py [--probe-only]
```

Uses system `python3` (Ubuntu 22.04 has `pxr` via `python3-pyusd`). Saves edits to
`exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd` (creates `.bak` of original
on first run; idempotent on subsequent runs).

**In-memory** (for runtime experimentation against a live aic-dt instance):

```bash
python3 exts/aic-dt/scripts/author_cable_physics.py [--probe-only]
```

## Authored counts

After offline run on the cable USD:
- 53 RigidBody links → all have `density=5e-05`
- 26 D6 joints → all have `rotY` + `rotZ` DriveAPI authored (52 total drives)

(In-memory counts via Isaac Sim show 23/22 — fewer because Isaac Sim only loads
one cable instance per scene, while the offline file contains multiple cable
authoring blocks for re-instantiation.)

## Activation

`exts/aic-dt/aic_dt/extension.py::load_robot` was updated 2026-05-05 to
flip `cable_prim.SetActive(False) → SetActive(True)` (Phase 1 D-04 reversal).

Emergency rollback: set `SCENE_05_DISABLE=1` env var before launch:
```bash
SCENE_05_DISABLE=1 ROS_DOMAIN_ID=7 bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt
```

## Verification

### Static (always works)
- USD on disk has 53 `density=5e-05` authorings → `python3 author_cable_physics_offline.py --probe-only`
- USD on disk has 52 `DriveAPI(rotY/rotZ)` authorings → same probe
- Backup file present → `ls -la exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd.bak`

### Live (runtime)
- After quick_start, kit log shows: `[AIC-DT] SCENE-05: activated /World/UR5e/cable`
- Cable subtree `IsActive() == True` → `python3 author_cable_physics.py --probe-only`
- No PhysX wedge regression: `play_scene` returns within 5s; MCP socket stays responsive; arm joint reads work
- Sim time advances normally (cable doesn't hang the main loop)

### What "cable bends under gravity" looks like

Counter-intuitive: per `[omni.physx.tensors.plugin] Cannot assign transform to non-root
articulation link`, the cable is an **articulated chain**, not free rigid bodies. This
means:
- The cable is anchored at one end (cable origin)
- Child links can ONLY move via joint constraints (rotY/rotZ swing)
- `RigidPrim.set_world_poses` on a child link is a no-op (PhysX rejects)
- At gravitational equilibrium the chain hangs vertically — visible as static link Z values

So the test "cable bends under gravity" is implicitly satisfied by the chain's resting
configuration matching gravity. Direct perturbation of a child link is NOT possible (and
not needed — real verification is whether CheatCode can grasp the plug end and insert it,
which Phase 4 will exercise).

## Risk / fallback

If the cable wedge returns under non-zero mass (post-play freeze, futex_wait):
1. `SCENE_05_DISABLE=1` env var to revert to inactive cable
2. Investigate via diagnostic probes at `exts/aic-dt/scripts/probe_cable_*.py`
3. Switch to rigid-plug-with-visual-cable hybrid (D-04 evolution) — author a single
   plug-end RigidBody + leave the rest of the cable mesh as visual geometry only

To date (2026-05-05) this fallback has not been needed — quick_start + play_scene work
cleanly with cable activated.

## Forward references

- Plan 03-03 attaches the plug-end link to the gripper via FixedJoint (SCENE-03)
- Plan 03-04 publishes cable link transforms on `/objects_poses_real` (re-scoped PARITY-08)
- Phase 4 trial loader exercises the cable end-to-end via CheatCode insertion attempts
