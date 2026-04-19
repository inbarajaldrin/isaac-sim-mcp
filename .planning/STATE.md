---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: Phase 7 + 07.1 complete and committed
stopped_at: Phase 7 (button audit + drop_refresh + IK cleanup) and Phase 07.1 (widget registry expansion) landed. Full CLI-level agent control over the control GUI. Commits da1e026 (vla_SO-ARM101) + 36cafb3 (isaac-sim-mcp).
last_updated: "2026-04-19T00:00:00.000Z"
progress:
  total_phases: 14
  completed_phases: 9
  total_plans: 14
  completed_plans: 14
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-03-27)

**Core value:** SO-ARM101 can sort lego blocks by color into matching cups — pick from table, drop into the correct cup — driven by poses from Isaac Sim and ArUco camera detection
**Current focus:** Phase 8 — Isaac Sim Extension Cleanup (next)

## Current Position

Phase: 8 (isaac-sim-extension-cleanup) — NOT STARTED
Phases 1-6, 05.1, 7, 07.1, 13 complete (9 of 14 phases done).
Remaining: Phase 8 → 9 → 10 → 11 → 12.

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

### Blockers/Concerns

None yet.

## Session Continuity

Last session: 2026-04-18T00:00:00Z
Stopped at: Session resumed, awaiting user selection for Phase 7 (discuss vs plan)
Resume file: none
