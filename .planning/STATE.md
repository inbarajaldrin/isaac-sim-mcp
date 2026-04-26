---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: Phase 11-01 in progress — color-driven single-pick workflow + cache lifecycle (Real Test tab)
stopped_at: implementation in flight
last_updated: "2026-04-25T22:30:00.000Z"
progress:
  total_phases: 12
  completed_phases: 3
  total_plans: 12
  completed_plans: 5
  percent: 42
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-03-27)

**Core value:** SO-ARM101 can sort lego blocks by color into matching cups — pick from table, drop into the correct cup — driven by poses from Isaac Sim and ArUco camera detection
**Current focus:** Phase 11-01 — color-driven single-pick workflow in Real Test tab. User picks color from dropdown → arm picks closest cached lego of that color → drops in matching cup → cache evicts dropped lego. Phase 11 also formally adopts the prior cross-repo work captured in HANDOFF.json (Real Test tab scaffold, marker_geometry cylinder_side_marker, cup orientation fix, YOLOE config-driven prompts). Deferred within Phase 11: 11-02 camera-spec single-source-of-truth + VFOV residual diagnosis; 11-03 autonomous mode (criterion 5).

## Current Position

Phase: 11 (Full Pick-and-Place Pipeline Verification) — IN PROGRESS, plan 11-01 (color-driven single-pick) authored 2026-04-25.
Phases 1-6, 05.1, 7, 07.1, 07.2, 9, 13 complete (11 of 15 phases done).
Remaining: Phase 8 (audit, deferred) → 10 (deferred) → 11 (in flight) → 12 (deferred).

**Phase 9 spillover that pre-completes parts of remaining phases:**

- Phase 8 criterion 1 (every button has a socket command): randomize_cups added as the latest example via the MCP_TOOL_REGISTRY pattern. Audit needed to confirm full button-to-service coverage.
- Phase 10 criterion 1 (drop point ergonomic approach): done via pan-lock + face-origin yaw on drop_sweep.
- Phase 10 criterion 3 (no jerk): partial — drop_point/drop_sweep slowed to 3.0 s default, peak ω 107°/s → 36°/s.
- Phase 11 sim half (criteria 3, 4, 5): done via `test_pick_all.sh` 27/27 PASS across all 3 colors × 3 lego sizes. Real-camera half (criteria 1, 2 — YOLOE + aruco_camera_localizer integration) remains.

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
- [Phase 9]: Tiered deterministic planner — tier-1 linear interp validates AND executes the same N waypoints (legacy tier-1 validated N but executed `(start, end, duration)` → spline divergence clipped cups). tier-2 retract-pan-settle handles "post-drop grasp_home swings near cup" deterministically. OMPL is fallback only, opt-in per primitive. `_cmd_grasp_home` opts OUT — RNG variance disallowed.
- [Phase 9]: Drop motion target-vs-physical alignment fixed by capturing `_attached_lego_tcp_offset` at `_attach_lego_to_gripper` time and using MEASURED `|ax|` for the gap→tcp shift in `_cmd_drop_sweep`, not theoretical `half_gap`.
- [Phase 9]: `lock_pan` kwarg on `_plan_collision_free_execute` mirrors `find_reachable_grasp_yaw`'s "single yaw across stages" pattern — drop_sweep passes current pan so geometric IK doesn't introduce ~1° base yaw on every sweep.
- [Phase 9]: `_CUP_COLLISION_PADDING` 1.0 → 1.05 (5%). Planning-side margin only — Isaac Sim physics still sees the real cup geometry. Absorbs multi-mm tracking-lag overshoot during fast pan motions.
- [Phase 9]: `randomize_cups` MCP tool samples CUP_LAYOUT params (mode/radius/angle/gap/color_order) and delegates placement to the existing `_cup_positions_arc` / `_cup_positions_line` generators — every random sample is a coherent layout with proper inter-cup spacing built in. Validated against lego footprints AND live-queried robot-link AABBs (z-min ≤ 0.15 m to count). ArUco markers face origin via face-origin yaw + ±15° jitter.
- [Phase 9]: `home_velocity_scale` param is dead code on the deterministic path (consumed only by `_ompl_plan_sync`); tier-1/tier-2 use `duration_s` directly. Documented in vla CLAUDE.md.

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

### Quick Tasks Completed

| # | Description | Date | Commit | Directory |
|---|-------------|------|--------|-----------|
| 260424-te8 | Promote OmniverseKit_Persp pose → workspace_camera_sim default + auto-switch on quick_start | 2026-04-25 | 480be86 | [260424-te8-promote-omniversekit-persp-pose-to-soarm](./quick/260424-te8-promote-omniversekit-persp-pose-to-soarm/) |

## Session Continuity

Last session: 2026-04-26T05:20:41.795Z
Stopped at: context exhaustion at 90% (2026-04-26)
Resume file: None
Handoff: none
