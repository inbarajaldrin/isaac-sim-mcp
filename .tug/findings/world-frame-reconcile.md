# World-frame reconciliation — full scope (surfaced 2026-05-11)

**Status:** scope mis-estimated in cf27bdd ("trial-home-robot-pose"). That commit moved the robot from `(0,0,0)` to Gazebo's canonical `(-0.2, 0.2, 1.14, yaw=-π)`, but **left the rest of the scene in the stale robot-at-origin frame**. The full reconciliation is a multi-entity fix, not per-entity.

## The meta-lesson (project rule going forward)

**When fixing the world-frame placement of any one scene entity, walk the FULL scene-authoring chain in `extension.py` and reconcile every sibling entity + environment constant in the same change.**

The aic-dt scene-authoring chain is:
```
load_scene
  → _setup_world_scene  (ground plane Z, physics scene)
  → import_enclosure    (AIC_Enclosure pose)
load_robot              (UR5e mount pose + RPY)
load_trial / quick_start
  → _cmd_spawn_task_board_base  (board pose)
  → _cmd_spawn_lc_mount_rail / sfp / sc  (per-part anchors)
  → _cmd_spawn_sc_port / nic_card_mount  (per-part anchors)
  → cable spawn  (DEFERRED-6)
```

The Gazebo canonical for each of these lives in:
- `~/Documents/aic/aic_description/world/aic.sdf` — world layout (Enclosure / Floor includes, world origin convention)
- `~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py` — DeclareLaunchArgument defaults for robot, task_board, cable poses
- `~/Documents/aic/aic_description/urdf/task_board.urdf.xacro` — per-part anchor origins
- `~/Documents/aic/aic_engine/config/sample_config.yaml` — per-trial scene overrides

Cross-reference **every** Isaac Sim constant against its Gazebo equivalent before declaring a world-frame fix complete.

## Current divergence table

| Entity | Isaac Sim (now) | Gazebo canonical | Source-of-truth | Fix |
|---|---|---|---|---|
| Robot mount | `(-0.2, 0.2, 1.14, yaw=-π)` | `(-0.2, 0.2, 1.14, yaw=-3.141)` | aic_gz_bringup.launch.py:595-619 | ✓ cf27bdd |
| AIC Enclosure | `_enclosure_position=(0.0, 0.0, -1.15)` (extension.py:562) | `(0, 0, 0)` — `aic.sdf` `<include>` no-pose → world origin; floor at Z=0 | aic.sdf:188-198 + Enclosure/model.sdf | **Move to `(0, 0, 0)`** |
| Ground plane | `_ground_plane_z=-0.08` (extension.py:565) | `Z=0` per Floor model at world origin | aic.sdf:200 + Floor/model.sdf | **Move to `z=0.0`** |
| Task board default | `(0.25, 0.0, 1.14)` (_cmd_spawn_task_board_base:3815-3816) | `(0.15, -0.2, 1.14, yaw=3.1415)` | aic_gz_bringup.launch.py:609-637 | **Update defaults to Gazebo's** (per-trial YAML correctly overrides this so runtime is OK; default-trust matters for any caller not going through load_trial) |
| Task-board PART anchors | LC/SFP/SC mount Z=0; NIC + SC-port translation→Y axis bug; missing SC-port RPY offset (1.57, 0, 1.57) rad | URDF anchors: LC z=0.012, SFP z=0.01, SC z=0.012/0.01 by index; NIC + SC port translation→X axis with per-index Y; SC port RPY offset | task_board.urdf.xacro joint origins | **Uncommitted in working tree** — apply if executor continues cf27bdd's intent; otherwise re-pin |
| Cable pose | `cable.SetActive(False)` (no pose authored) | `(0.172, 0.024, 1.518, rpy=(0.4432,-0.48,1.3303))` | aic_gz_bringup.launch.py:660-695 | **DEFERRED-6** — author when cable physics closes |

## Vertical reference plane visualization

Before cf27bdd (Isaac Sim, robot-at-origin):
```
Z=1.35  ─── enclosure ceiling (-1.15 + 2.5)
Z=0.0   ─── world origin / robot base
Z=-0.08 ─── ground plane
Z=-1.15 ─── enclosure origin (centered around the workcell volume)
```

After cf27bdd (Isaac Sim NOW — broken):
```
Z=1.35  ─── enclosure ceiling (still stale)
Z=1.14  ─── ROBOT BASE (moved up, leaving everything else behind)
Z=0.0   ─── (nothing here)
Z=-0.08 ─── ground plane (floating in space, 1.22m below robot)
Z=-1.15 ─── enclosure origin (stale, ceiling intersects robot reach)
```

Gazebo (canonical target for Isaac Sim AFTER reconcile):
```
Z=2.5   ─── enclosure ceiling
Z=1.14  ─── robot base AND task board surface (work plane)
Z=0.0   ─── floor / world origin / enclosure origin (all aligned)
```

## Reconciliation plan

**Phase A — current batch (`trial-home-robot-pose` pinned):** orchestrator will let it land. Its scope was home-joints + robot-mount + radians-to-degrees. The verifier's threshold issue (|y|=0.183 > 0.1 m operator linter gate) likely traces to home-pose intent ("home is generic, not trial-aligned") + the broader env mismatch making the geometry look worse than it is. If executor only touches home-pose code paths, that's fine — anchor + enclosure fixes go in Phase B.

**Phase B — new PRD task `environment-world-frame-reconcile`:** single atomic change covering
- `extension.py:562` — set `_enclosure_position = (0.0, 0.0, 0.0)`
- `extension.py:565` — set `_ground_plane_z = 0.0`
- `extension.py:3815-3816` — update `_cmd_spawn_task_board_base` defaults to `(0.15, -0.2, 1.14, roll=0.0, pitch=0.0, yaw=3.1415)`
- `extension.py:3822-3950` — apply URDF-anchor corrections (the uncommitted working-tree diff, IF executor stashed it; otherwise verify it landed under Phase A)
- Verify: probe `/World/AIC_Enclosure` translation, `/World/defaultGroundPlane` Z, `/World/TaskBoard` translation, plus the 4-task-board-part frames all match URDF/Gazebo. Cable left at SetActive(False) per DEFERRED-6.
- Acceptance gates: (1) all 5 entities at canonical Z; (2) re-fire trial_1; (3) base_link→sc_port_0 TF distance within 10mm of expected (currently 184mm partial-error from home-pose vs port-target). Cable insertion will still fail (DEFERRED-6), but geometric pre-conditions for insertion will be sound.

**Phase C — DEFERRED-6 cable physics:** unchanged plan, but the cable's authored pose `(0.172, 0.024, 1.518)` needs to be applied at the same time mass/inertia is authored. Add to the cable-physics-fidelity task scope.

## Add to PRD when current batch lands

```json
{
  "id": "environment-world-frame-reconcile",
  "title": "Reconcile enclosure + ground + task-board + part anchors to Gazebo world frame",
  "category": "scene-authoring",
  "priority": "high",
  "passes": false,
  "requires_sim": true,
  "files_likely": ["exts/aic-dt/aic_dt/extension.py"],
  "skills_required": ["isaac-sim-extension-dev", "nvidia-suite-docs"],
  "description": "<see Phase B above>",
  "acceptance": ["<see Phase B above>"],
  "verify_command": "bash exts/aic-dt/scripts/verify_world_frame_reconcile.sh",
  "verification_mode": "sim"
}
```

(verify script to be written as part of the executor's task)
