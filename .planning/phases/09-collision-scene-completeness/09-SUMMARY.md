# Phase 9 Summary — Collision Scene Completeness

**Completed:** 2026-04-24
**Implementation commits:**
- `vla_SO-ARM101 main` `af62673`, `53cc31e`, `a11c2dd` (deterministic planner + drop motion + scripts + docs)
- `isaac-sim-mcp so-arm101` `a2e04b5`, `fff7f18`, `ff4f3ed`, `86419c1` (CLAUDE.md fixes + randomize_cups iterations)

**Preceded by:** Phase 07.2 — which diagnosed the residual cycle-8+ cup-tipping cascade and surfaced "lego AttachedCollisionObject missing" as the structural root cause. Phase 9 also absorbed the deterministic-planner refactor that 07.2's analysis pointed at.

## Delivered

### vla_SO-ARM101: tiered deterministic motion planner (`control_gui.py:5817`)

Replaces always-OMPL discipline in `_joint_space_collision_free_execute` with a tiered planner. Every arm motion routes through:

1. **Tier 1** — `_plan_linear_joint_path`: linear joint-space interp, validates each waypoint via `_check_state_valid_with_contacts`, dispatches the SAME N-waypoint trajectory to `_execute_full_trajectory` (fixes the legacy bug where N waypoints were validated but only `(start, end, duration)` was sent — controller's spline interpolated paths that clipped cups between knots).
2. **Tier 2** — `_plan_retract_pan_settle`: 3-segment decomposition (retract to NEUTRAL_NON_PAN, pan across, settle to target). Handles "post-drop grasp_home swings near cup" deterministically.
3. **OMPL fallback** — opt-in per caller via `allow_ompl_fallback=True`. `_cmd_grasp_home` opts OUT — RNG variance disallowed for safety; surfaces failure loudly instead.

Each motion emits `tracer.event('planner_used', which='linear|retract_pan_settle|ompl_fallback|none')`.

### vla_SO-ARM101: drop motion robustness pass

- **`_attached_lego_tcp_offset`** captured at attach time (block_in_tcp pose). `_cmd_drop_sweep` uses MEASURED `|ax|` for the gap→tcp shift, not theoretical `half_gap`. Off-center grasps no longer drift the block 1-2 mm into cup walls.
- **`lock_pan` kwarg** on `_plan_collision_free_execute`: drop_sweep passes current pan so geometric IK doesn't introduce ~1° base yaw on every sweep. Mirrors `find_reachable_grasp_yaw`'s "single yaw across stages" pattern.
- **Hover above rim** default 30 → 50 mm — block clears cup wall during wrist_flex 90° → 55° sweep arc.
- **`drop_point` duration** 1.0 → 3.0 s — peak pan rate 107°/s → 36°/s, well below the PD drive's tracking envelope.
- **`drop_sweep` duration** wired through (`_drop_duration_var` was previously dead). Default 5.0 → 3.0 s for stack consistency.
- **`_CUP_COLLISION_PADDING`** global default 1.0 → 1.05 (5%). MoveIt planning scene only — Isaac Sim physics still sees the real cup geometry. Absorbs the multi-mm tracking-lag overshoot during fast pan motions.

### vla_SO-ARM101: race-condition fix on IK failure paths

`_plan_collision_free_execute` now writes `_last_motion_status = {'ok': False, ...}` BEFORE `on_complete.set()` on IK failure. Previously the QS runner's `_qs_wait_for_step` would see a stale `None` and read the failure as success.

### vla_SO-ARM101: pick-and-drop test harness (`scripts/`)

- `test_qs_cycle.sh` — single QS cycle for one named lego, polls `/get_log` for terminal sentinels, trims log to per-cycle scope at exit.
- `sim_reset.sh` — `qs_restart → detach → MCP update_cups → MCP randomize_object_poses → qs_refresh_all → grasp_home`.
- `test_pick_all.sh` — sequential pick-and-place for every available lego, summary CSV with per-cycle planner counts.

### isaac-sim-mcp: `randomize_cups` MCP tool (`extension.py:3032`) + UI button

Stress-test infrastructure. Samples CUP_LAYOUT params (mode/radius/angle/gap/color_order) and delegates placement to existing `_cup_positions_arc` / `_cup_positions_line` generators. Each random sample is a coherent layout, validated against:

- Lego footprints (mirrors `randomize_object_poses` cup-as-exclusion logic, in reverse)
- Robot-link AABBs at current pose (live-queried from `/World/SO_ARM101` direct children, z-min ≤ 0.15 m to count)

Yaw anchored on face-origin direction (ArUco markers face camera) with ±15° jitter. Cup positions teleported via the same Kit-command path as `update_cups`; `_cmd_publish_drop_poses` called explicitly afterward to refresh the action graph wrappers (otherwise /drop_poses publishes stale cached transforms — debugged once, not twice).

`randomize_order=True` shuffles color → slot mapping each sample (6 permutations).

## Phase 9 success criteria — verification

| Criterion | Status |
|---|---|
| 1. Lego blocks from /objects_poses appear as collision objects in MoveIt scene | ✓ shipped before this phase (verified `_lego_collision_names` tracker active) |
| 2. Collision dimensions match `/objects_bbox` | ✓ shipped before |
| 3. **Drop sweep plans avoid cups AND lego blocks** | ✓ this phase — deterministic planner + 5% cup padding + measured-attach-offset drop_sweep + tier-2 retract-pan-settle. Verified across 27 consecutive QS cycles (3 runs × 9 legos) — every motion via tier-1 or tier-2, zero OMPL fallback, zero REFUSED |
| 4. Collision objects update on re-detect | ✓ shipped before |

## Verification

- **27/27 QS cycles PASS** (`test_pick_all.sh` × 3 runs). Per cycle: ~5 tier-1 successes + 1-2 tier-2 (drop_sweep/post-drop grasp_home), 0 OMPL fallback, 0 REFUSED.
- **Pan-lock effectiveness:** trajectory dump shows `shoulder_pan` Δ = 0.000° on drop_sweep (was 0.918° pre-fix).
- **`drop_point` deceleration:** trajectory dump shows peak `shoulder_pan` velocity 35.82°/s (was 107.45°/s).
- **`randomize_cups` convergence:** 8/8 random seeds place valid layouts with proper inter-cup spacing, lego clearance, and robot-link clearance.

## Files changed in repos

- `vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py` (+354 / -57)
- `vla_SO-ARM101/scripts/{test_qs_cycle.sh,sim_reset.sh,test_pick_all.sh}` (+263 new)
- `vla_SO-ARM101/README.md` (+45 / -1) + `vla_SO-ARM101/CLAUDE.md` (+157 new)
- `isaac-sim-mcp/exts/soarm101-dt/so_arm101_dt/extension.py` (+~390 / -~95) — `randomize_cups` + `_cmd_randomize_cups` + UI button + MCP tool registry
- `isaac-sim-mcp/CLAUDE.md` (+~10 lines — drop-motion fix, MCP tool list, gotchas)
- `vla_SO-ARM101/src/so_arm101_moveit_config/config/pilz_industrial_motion_planner_planning.yaml` deleted (orphan)

## Spillover into other phases (not separately phased)

- **Phase 8** (Isaac Sim Extension Cleanup): +1 socket-callable button (`randomize_cups`).
- **Phase 10** (Motion Quality): drop_point ergonomic approach (pan-lock criterion 1) + reduced jerk via duration tuning (criterion 3) — partial.
- **Phase 11** (Pipeline Verification): sim-half complete (criteria 3, 4, 5 met via `test_pick_all.sh` 27/27 PASS); real-camera half (criteria 1, 2) untouched.

## Known followups (not blocking, deferred)

- **OMPL Mode B fallback rate on `randomize_cups` stress tests:** 1-2 of 5 random configs hit `drop_sweep ompl_mode_b` (path through padded cup wall). Mitigation: tighten `yaw_jitter_deg` further OR add per-cup drop_sweep IK pre-check at randomize-time.
- **Carry-grasp_home OMPL ban edge case:** with random cup positions, `return home (carry)` occasionally hits `tiered_planner_exhausted` because cup is in carry-arc. Mitigation: allow OMPL on carry-grasp_home selectively, or widen tier-2 NEUTRAL pose lift.
- **`home_velocity_scale` is dead code on the deterministic path:** the param is consumed by `_ompl_plan_sync` only; tier-1/tier-2 use `duration_s` directly. Documented in vla CLAUDE.md, no fix planned (use `duration_s` argument instead).

## Planning artifacts

- `.planning/ROADMAP.md` — Phase 9 checkbox flipped to [x] in summary list.
- `.planning/STATE.md` — progress 10/15 → 11/15, recent decisions logged, current focus → Phase 8 audit OR Phase 10 camera-confirm + real-hw.
- `.planning/phases/09-collision-scene-completeness/09-SUMMARY.md` — this file.
- `.planning/phases/09-collision-scene-completeness/.continue-here.md` — deleted (stale, predates this phase's actual implementation).
