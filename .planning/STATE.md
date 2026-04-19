---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: Phase 07.2 complete — FC-1 + FC-2 motion planning fixes landed
stopped_at: Phase 07.2 closed. FC-1 yaw-fallback + FC-2 OMPL goal-perturb retry + FixStartStateCollision adapters landed in vla_SO-ARM101 (control_gui.py + ompl_planning.yaml). Verified cross-axis N=10 run on 2026-04-19: 5/5 reached-spawn cycles clean with zero cup tilt; cycle 6 correctly rejected by FC-1 as unreachable spawn. Residual cycle-8+ cup-tipping scoped to Phase 9 (lego AttachedCollisionObject).
last_updated: "2026-04-19T11:45:00.000Z"
progress:
  total_phases: 15
  completed_phases: 10
  total_plans: 14
  completed_plans: 14
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-03-27)

**Core value:** SO-ARM101 can sort lego blocks by color into matching cups — pick from table, drop into the correct cup — driven by poses from Isaac Sim and ArUco camera detection
**Current focus:** Phase 9 next — Collision Scene Completeness (add lego `AttachedCollisionObject`s so MoveIt avoids carried-block-vs-cup-rim contact, structurally fixing the residual cycle-8+ cascade observed during 07.2 debugging)

## Current Position

Phase: 9 (collision-scene-completeness) — NOT STARTED (next up per user direction 2026-04-19)
Phases 1-6, 05.1, 7, 07.1, 07.2, 13 complete (10 of 15 phases done).
Remaining: Phase 9 → 8 → 10 → 11 → 12. (User elected Phase 9 ahead of Phase 8 because 9 is the structural fix for the cascade diagnosed in 07.2.)

## Performance Metrics

**Velocity:**

- Total plans completed: 0
- Average duration: -
- Total execution time: 0 hours

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

**Recent Trend:**

- Last 5 plans: none yet
- Trend: -

*Updated after each plan completion*

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

- Project init: 5-DOF wrist sweep instead of IK-to-pose (more reliable for small cups)
- Project init: Same TFMessage format as JETANK (reuse existing /drop_poses subscription code)
- Project init: Isaac Sim publishing first, ArUco second (get drop motion working with known-good poses before adding camera detection)
- Project init: Individual step buttons, not automated sequence (easier debugging)
- [Phase 01]: Wrapper Xform prims (drop_red/green/blue) for distinct child_frame_ids in ROS2PublishTransformTree
- [Verification]: Service calls block until trajectory completes (_motion_event pattern)
- [Verification]: All motion through _execute_trajectory (single entry point)
- [Verification]: WRIST_ROLL_OFFSET split into WRIST_ROLL_URDF_PITCH (0.0487) + inline π/2
- [Verification]: Hot reload reads from src/ directly, SIGINT shutdown in ~2s
- [Verification]: Cups anchored to shoulder pan axis (not world origin)
- [Phase 05-01]: Cup collision cylinders via SolidPrimitive.CYLINDER, no ACM entries (want collision checking)
- [Phase 05-02]: Concave STL mesh replaced with cylinder primitives (FCL can't check concave meshes)
- [Phase 05-02]: geometric_ik generalized with grip_angle (45° for drop) + wrist_roll (-90°) params
- [Phase 05-02]: _collision_free_ik_plan_and_execute: geometric IK → collision check → MoveIt OMPL path planning
- [Phase 05-02]: grasp_home and drop_sweep use _cmd_plan_execute with MoveIt collision-aware planning
- [Phase 05-02]: Action clients added to ReentrantCallbackGroup (fixed executor deadlock)
- [Phase 05-02]: OMPL config: simplify_solutions=false, longest_valid_segment_fraction=0.01, 50 planning attempts
- [Phase 05-02]: 10% cylinder padding compensates for trajectory execution deviation
- [Phase 06]: Verified 3/3 consecutive clean passes for all 3 cups (drop_0, drop_1, drop_2)
- [Phase 13]: Empirical RTF test: wrist action graph costs ~0 RTF; dedicated hidden viewport costs −0.32 RTF per extra viewport; active-viewport-reuse publishing is genuinely free.
- [Phase 13]: Ported `_ViewportCameraPublisher` (active-reuse only), `new_stage` MCP tool, `_RecordingState` + ffmpeg recording MCP tools, rclpy 3.11 path fix into soarm101-dt.
- [Phase 13]: Dropped migration of wrist camera — existing action graph is cheaper than dedicated viewport would be.
- [Phase 07.2]: FC-1 yaw-fallback sweeps a 10-candidate offset set via `find_reachable_grasp_yaw`; correctly rejects unreachable spawns before any motion.
- [Phase 07.2]: FC-2 replaces raw OMPL with joint-space-direct-first + goal-perturb-retry wrapper (`_joint_space_collision_free_execute` → `_ompl_plan_with_retry_execute`). Goal-side perturbation (not start-side) within ±0.02 rad keeps execution safe.
- [Phase 07.2]: FixStartStateCollision + FixStartStatePathConstraints request adapters added to `ompl_planning.yaml` — were MoveIt Setup Assistant defaults missing from our config.
- [Phase 07.2]: FC-3 (spawn filter in extension.py) reverted per 1:1 digital-twin principle; FC-1's pick-time detection covers the same ground without breaking sim/real symmetry.
- [Phase 07.2]: N=10 cross-axis verification run (2026-04-19): 5/5 reached-spawn cycles PASSED with zero cup tilt or drift; cycle 6 correctly rejected by FC-1 as unreachable spawn @(0.095,-0.105,0.008). Cascade did not reproduce — consistent with the hypothesis that tipping required carried-block-vs-cup-rim contact, which cross-axis grip minimizes.
- [Phase 07.2]: Residual cycle-8+ cascade root cause: lego blocks are **not** published as `AttachedCollisionObject` on the gripper, so MoveIt plans transit paths blind to the carried block. Deferred to Phase 9 by design (Phase 9's success criterion #3 is the structural fix).

### Pending Todos

None yet.

### Roadmap Evolution

- Phase 4 added: Isaac Sim Joint States Fallback Publisher — prevent snap-to-zero on control stack restart
- Phase 5 added: MoveIt Cup Collision and Planned Drop Motion — replace hardcoded wrist sweep with MoveIt-planned collision-aware drop
- Phase 6 added: Collision-free Drop Motion Verification Loop — systematic test: home→point→sweep→check cups→home→check cups, 3 consecutive passes required
- Phase 05.1 inserted after Phase 5: Replace cylinder collision primitives with CAD cup mesh (URGENT) — new CAD assets at /home/aaugus11/transfer/cad
- Phase 05.1 completed: convex hull mesh collision + colored visual markers + RViz settings tab (padding, grip angle, planning attempts)
- Phases 7-12 added: GUI audit, Isaac Sim cleanup, lego collision, motion quality, full pipeline verification, joint states fallback
- Merged main into so-arm101 (commit 0f07949): brings 10 commits of ur5e-dt work onto the so-arm101 branch so both extensions coexist
- Phase 13 added: Port UR5e-dt backend features to soarm101-dt — `_ViewportCameraPublisher` as camera publishing REPLACEMENT, stage lifecycle cleanup, `new_stage`, `execute_python_code` sessions, video recording backend (MCP-only, no UI)
- Phase 07.1 inserted after Phase 7 (URGENT, 2026-04-18): Widget Registry Expansion — Full CLI-Level Control. Extends button-only registry to Spinboxes/Checkbuttons/Entries/Listboxes/Scales with generic list/get/set widget services. Inserted mid-Phase-7 (after 07-01 landed) because Plan 07-03/07-04 UAT benefits from agent-driven widget control. Inspired by element-registry skill pattern.
- Phase 07.2 inserted after Phase 07.1 (URGENT, 2026-04-19): Motion Planning Problem Analysis & Inline Fix. Scope expanded during discuss-phase from analysis-only to analysis + inline fix — 07.2 now owns the three Phase-6 flakiness items (no-IK at edge, OMPL error −2 post-release, out-of-reach spawns) instead of routing them to Phase 10. Method: reachable-workspace + config-space analysis + N=10 random-seed trials per class in Isaac Sim only. See `.planning/phases/07.2-motion-planning-problem-analysis/07.2-CONTEXT.md`.

### Blockers/Concerns

None yet.

## Session Continuity

Last session: 2026-04-19T11:45Z (Phase 07.2 closed)
Stopped at: Phase 07.2 complete. Fixes committed to both repos. Next phase: Phase 9 (Collision Scene Completeness — AttachedCollisionObject for legos).
Resume file: none
Handoff: none (HANDOFF.json deleted after successful resumption per GSD convention)
