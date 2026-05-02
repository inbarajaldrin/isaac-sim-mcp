# aic-dt — AIC Cable-Insertion Digital Twin in Isaac Sim

## What This Is

An Isaac Sim digital twin of the AI for Industry Challenge (AIC) cable-insertion task board, packaged as a Kit extension under `exts/aic-dt/` with an MCP socket server (port 8768) for agent-driven control. It loads the same USD/URDF assets the AIC competition stack uses in Gazebo, publishes the same ROS 2 topic surface Gazebo publishes, and runs unmodified `aic_controller`, `aic_engine`, and `aic_example_policies` (`CheatCode.py`, `GentleGiant.py`, `WallToucher.py`, …) from the sister AIC repo at `~/Documents/aic`. Built so policy + perception development can iterate fast in Isaac Sim, then redeploy to Gazebo (and the real UR5e rig) without code changes.

## Core Value

When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome — and only the *pose source* differs in future milestones (ground-truth TF in M1, 6D pose estimation in M2). Topic-and-asset parity with Gazebo is non-negotiable.

## Requirements

### Validated

- ✓ `aic-dt` extension scaffold with MCP socket server on port 8768 — existing (commits 880a3a4, 2be9e0c, 8ff7104)
- ✓ Atomic MCP tool model with single-source registry (`MCP_TOOL_REGISTRY` + `_cmd_<name>` handlers) — existing
- ✓ Postload launcher + `quick_start` Quick Start clubbing the common path — existing (commit 2be9e0c)
- ✓ UR5e load + RG2 attach ordering correct (RG2 attached before play) — existing (commit 8ff7104)
- ✓ AIC enclosure + initial task-board object set (`task_board_base`, `sc_port_1/2`, `nic_card`) loadable from local `assets/` (no Nucleus dependency) — existing
- ✓ Domain-randomization atoms for board / SC ports / NIC card — existing
- ✓ Built-in wrist camera ROS 2 publishers (center / left / right) at 640×480 — existing
- ✓ Force/torque publisher action graph (UR5e end-effector wrench) — existing
- ✓ Sibling extensions provide reference patterns: `ur5e-dt`, `soarm101-dt` — existing

### Active (Milestone 1 — Platform Transfer)

- [ ] **Topic parity with Gazebo**: Isaac Sim publishes the *exact* topic names `aic_controller` + `aic_engine` + `CheatCode` consume today (`/joint_states`, `/tf` with object frames in `base_link`, `/tf_static`, `/scoring/tf`, `/fts_broadcaster/wrench`, `/aic/gazebo/contacts/off_limit`, `/scoring/insertion_event`, `/aic_controller/{joint,pose}_commands`, `/aic_controller/controller_state`). No `_sim` / `_real` suffixes, no remap nodes, no bridge translators. Replace placeholder `objects_poses_sim` / `sync_real_poses` machinery currently in the extension.
- [ ] **Same USD/URDF as Gazebo**: load the AIC repo's `aic_description` xacro/URDF and `aic_assets` meshes for the UR5e + RG2 + 3 wrist cameras + cable + task board. No divergent geometry or kinematics.
- [ ] **Texture / material sweep**: load every USD/MDL referenced by the spawn path, find anything broken (pink, black, missing maps, broken MDL refs, GLB→USD PBR loss), and fix in place. Discover-by-loading.
- [ ] **Task board parameterization parity**: Isaac Sim spawn supports the same component deltas `spawn_task_board.launch.py` does (mount rails 0/1, sc/sfp/lc port presence + translation + RPY, NIC rails 0–3, etc.) so a `sample_config.yaml` trial config lands the same scene as Gazebo.
- [ ] **Cable spawn parity**: Isaac Sim spawn supports both `cable_type` values Gazebo supports (`sfp_sc_cable`, `sfp_sc_cable_reversed`) with `attach_cable_to_gripper` semantics. Cable physics fidelity strategy is a Phase 1 research item — see Key Decisions / Constraints.
- [ ] **`ground_truth:=true` mode**: object TF frames (`{cable_name}/{plug_name}_link`, port frames, etc.) published into `/tf` against `base_link` exactly as Gazebo does when `ground_truth:=true`. CheatCode's `lookup_transform` calls succeed against Isaac Sim with no code change.
- [ ] **Trial loader**: an MCP atom + UI button that reads a single trial entry from `aic_engine`'s YAML config (or an equivalent format) and spawns the matching scene in Isaac Sim. Used both interactively (button per trial) and by `aic_engine` (the engine sees only ROS, not MCP — it just expects the scene to be in the configured state when it queries TF).
- [ ] **Updated Quick Start atoms / clubbing**: refactor `quick_start` to club the M1 atoms cleanly. Each new piece (trial loader, cable spawn, controller-side bridge if needed, pose publisher) is one MCP tool + one UI button. Keep the atomic + clubbed pattern.
- [ ] **End-to-end M1 verification**: every trial in the AIC repo's `sample_config.yaml` passes under `CheatCode` against Isaac Sim with the same per-trial pass/fail outcomes Gazebo produces. Run via the same `aic_engine` invocation, no Isaac-Sim-specific patches in the AIC repo.
- [ ] **`CLAUDE.md` for this repo**: launching aic-dt (Isaac Sim startup, extension enable, MCP port), required env vars, and a clear pointer to the AIC repo (`~/Documents/aic`) explaining the cross-repo relationship — this repo is sim-side; ROS-side stays in the AIC repo. Becomes the Claude Code on-ramp for any future session.

### Active (Milestone 2 — Pose Source Swap, scope-aware only)

- [ ] Replace the `ground_truth:=true` TF publisher for object frames with a **6D pose estimator** (any — `aic_vision` registry, photogrammetry, CAD-defined hole-pose pipeline, etc.) feeding the *same* `/tf` channel CheatCode reads.
- [ ] No changes to `aic_controller`, `aic_engine`, `CheatCode`, or `sample_config.yaml`. The estimator is a drop-in TF publisher; everything downstream is unaware.
- [ ] Eventual redeploy back to Gazebo / real UR5e: because topic parity is M1's law, this is a re-launch, not a rewrite.

(M2 details will be planned in its own milestone — listed here only to anchor M1 architecture decisions.)

### Out of Scope

- **ROS-side MCP server / agent control of `aic_engine`** — if/when we build agent-driven AIC control (start trial, swap policy, query scoring), it lives in the AIC repo (`~/Documents/aic`), not here. This repo stays sim-only.
- **Modifying anything in the AIC repo to suit Isaac Sim** — `aic_controller`, `aic_engine`, `aic_example_policies`, `aic_description`, `aic_assets` are consumed read-only. If something doesn't fit, fix it on the Isaac Sim side or change topic *publishers* — never the AIC repo's controller / policy / config code.
- **New control schemes** — no replacing or augmenting `aic_controller`. The C++ impedance controller is the controller for the foreseeable future.
- **Training pose-estimation models** — M2 uses pre-trained estimators or CAD-based methods only. `aic_vision` (in the AIC repo) handles its own training/eval; aic-dt only consumes its outputs.
- **`omni.*` imports in the AIC repo** — sim-side stays out of the AIC repo's runtime path. (rclpy *is* allowed inside the aic-dt extension for non-control glue — other extensions in this repo already do this.)
- **Real-time / latency optimization** — M1 just needs functional parity, not Gazebo-matching wall-clock performance.
- **Cable visual fidelity beyond physics-needed accuracy** — cosmetic differences are OK if CheatCode behavior is preserved.
- **Headless / CI integration** — M1 verification can be interactive (load Isaac Sim, run engine, watch trials). Headless eval is a future concern.

## Context

**Larger ecosystem.** This repo (`isaac-sim-mcp`) hosts three Isaac Sim extensions — `aic-dt` (this project's focus), `ur5e-dt` (older UR5e + RG2 reference), and `soarm101-dt` — plus a top-level `isaac_mcp/` socket server, an `ompl_src/` motion-planning side experiment, and helper `scripts/`. The repo as a whole is "Claude-driven Isaac Sim digital twins for various robots." `aic-dt` was scaffolded recently (commit `880a3a4` adds it; `2be9e0c` and `8ff7104` are quality-of-life follow-ups). It is currently a placeholder — UI structure, MCP tool registry, atomic+clubbed pattern, ROS publisher patterns are all worth keeping; specific topic names like `objects_poses_sim` / `sync_real_poses` are *not* and will be replaced by Gazebo-native topic names in M1.

**The AIC competition.** AI for Industry Challenge (AIC) — UR5e robot inserting cables into ports on a task board. Qualification deadline May 15, 2026. Scoring: 75pts for full insertion, 12pts duration bonus, 6pts smoothness, 6pts efficiency. The competition's reference toolkit lives at `~/Documents/aic` and includes:

- `aic_controller/` — C++ ROS 2 impedance controller plugin (`aic_controller_plugin.xml`). Sim-agnostic; consumes joint states + TF + wrench, emits joint/pose commands. **Used unmodified.**
- `aic_engine/` — C++ trial orchestrator: spawns trials from YAML, subscribes to scoring topics, emits `/scoring/insertion_event`. **Used unmodified.**
- `aic_example_policies/aic_example_policies/ros/` — `CheatCode.py` (GT-pose policy), `GentleGiant.py`, `RunACT.py`, `SpeedDemon.py`, `WallPresser.py`, `WallToucher.py`, `WaveArm.py`. **Used unmodified.**
- `aic_bringup/launch/` — `aic_gz_bringup.launch.py` (main, takes `ground_truth`, `cable_type`, `start_aic_engine`, `attach_cable_to_gripper`, robot/board/cable poses), `spawn_task_board.launch.py` (parameterized rails + ports), `spawn_cable.launch.py` (cable variants). These define what "the Gazebo setup" *means*; Isaac Sim mirrors their **outcomes** (scene state + topic surface), not their file structure.
- `aic_description/`, `aic_assets/` — robot/board/cable URDFs + meshes. Same assets load on both sides.
- `aic_vision/` — separate sub-project (its own `.planning/`, on `aldrin/aic-vision` branch) building a plug-and-play 6D pose estimator framework. Phase 2 in flight (see `~/Documents/aic/.planning/PROJECT.md`, `HANDOFF.md`). Future M2 pose-source candidate.

**Why Isaac Sim?** Gazebo is "a little tricky to setup and run" (per the user) for the iteration loop the M2 perception work will need. Isaac Sim is the dev environment; Gazebo + real rig are the deploy targets. **Topic parity is what makes that deploy free**, which is why M1's success bar is platform transfer, not feature delivery.

**Existing extension patterns to keep.** `MCP_TOOL_REGISTRY` as the single source of truth for tool metadata. `_cmd_<name>` handler convention. Atomic UI button per tool + a `quick_start` that clubs the common path. `ScrollingWindow` UI with `CollapsableFrame` sections. Asset loading via local `file://` paths under `exts/aic-dt/assets/` (no Nucleus). Wrist cameras as built-in robot prims with their own ROS 2 publisher graphs. UR5e joint drive params taken from Isaac Lab config (`stiffness=2000, damping=100, max_force=87`).

**Hardware reality.** 3 Basler wrist cameras (1152×1024 RGB, 20fps, 60° yaw separation, 75° pitch down, 186mm L-R baseline). The Isaac Sim cameras currently default to 640×480 — may need bumping to match real for M2 perception work, but M1 only needs CheatCode (which doesn't read cameras) so this is deferred.

## Constraints

- **Asset source**: Same USD / URDF / mesh files Gazebo loads. No divergent geometry or kinematics. (If a USD doesn't exist where the URDF expects it, fix the asset, not the URDF.)
- **Topic surface**: Exactly Gazebo's. No `_sim` / `_real` suffixes, no remap nodes, no bridge translators on the M1 happy path. Isaac Sim *may* expose extra debug topics; it must not omit or rename any Gazebo topic.
- **No mods in AIC repo**: `aic_controller`, `aic_engine`, `aic_example_policies`, `aic_description`, `aic_assets`, `sample_config.yaml`, etc. are consumed read-only. Compatibility burden lives entirely on the Isaac Sim side.
- **Sim-side may use rclpy**: extension code is allowed to import `rclpy` for non-control glue (config reads, scoring event listening, trial orchestration). ROS-side never imports `omni.*`.
- **MCP atomic + clubbed**: every new capability lands as one MCP tool + one UI button + (where it belongs in the common path) inclusion in `quick_start`. Don't break the existing atomic model.
- **Local assets only**: `_get_assets_folder()` resolves to `exts/aic-dt/assets/` via `file://`. No Nucleus dependency. New assets get vendored under `assets/` (or symlinked from the AIC repo's `aic_assets/` install path — to be decided in research).
- **Cable physics**: fidelity strategy is *deliberately undecided*; will be selected after Phase 1 research using the `nvidia-suite-docs` skill (Isaac Sim deformable / articulated chain / rigid+visual hybrid all on the table). Spawn API + topic surface are still locked even before the strategy is picked.
- **Timeline**: AIC qualification May 15, 2026 — but that deadline is the AIC repo's concern. This project's pace is "M1 in service of M2 unblocking". M1 should land in days-to-weeks, not months.

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Repo split: sim-side here, ROS-side / agent control in `~/Documents/aic` | Keeps each repo's concern clean; AIC stack stays sim-agnostic; redeploy to Gazebo / real rig is a re-launch not a port | — Pending (locks M1 scope) |
| Topic parity with Gazebo (no `_sim`/`_real` suffixes, no bridges) | M2's eventual deploy back to Gazebo / real rig is free if topics are identical now; same USD/URDF means topic surfaces *should* match by construction | — Pending |
| Same USD/URDF as Gazebo (`aic_description`, `aic_assets`) | Zero-divergence guarantee; any mismatch becomes immediately visible | — Pending |
| Pose-source seam = `/tf` (M1 publishes GT into `/tf`, M2 swaps publisher) | CheatCode reads `/tf` via `lookup_transform`; cleanest seam is "who publishes the object frame", controller stays unaware | — Pending |
| `rclpy` allowed inside Isaac Sim extension (non-control glue) | Sibling extensions already do this; lets the extension self-verify against scoring events without an extra process | — Pending |
| MCP atomic + clubbed model preserved | The existing extension's pattern (single `MCP_TOOL_REGISTRY` + `_cmd_<name>` + per-tool button + `quick_start` clubbing) is good and was hard-won; new M1 capabilities follow the same shape | — Pending |
| Discover-by-loading texture sweep (no pre-enumerated culprits) | Faster than auditing USD references upfront; broken materials are immediately visible in the viewport | — Pending |
| Cable physics strategy deferred to Phase 1 research (use `nvidia-suite-docs`) | Picking deformable vs articulated vs rigid+visual hybrid without docs would over-commit M1 | — Pending |
| M1 success bar = all `sample_config.yaml` trials pass under CheatCode | Automatable via `aic_engine` outputs; matches Gazebo's existing test surface; one bar that's hard to fake | — Pending |
| Replace placeholder `objects_poses_sim` / `sync_real_poses` MCP atoms | They were a stand-in; M1 publishes the actual `/tf` Gazebo publishes, no `_sim`/`_real` discrimination | — Pending |
| `CLAUDE.md` is an M1 deliverable | This repo currently has no Claude on-ramp; once M1 lands, future sessions need a single doc that says "launch like this, see the AIC repo for that" | — Pending |
| AIC repo is read-only from this project's perspective | Forces the burden of compatibility onto Isaac Sim, where we have flexibility; AIC stack stays competition-deployable | — Pending |
| `nvidia-suite-docs` skill is the canonical Isaac Sim reference, not LLM training data | Isaac Sim's APIs evolve fast; physics + sensor + ROS2 bridge specifics need live docs. Triggered for cable physics + texture/material + USD asset work. | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-05-01 after initialization*
