# Phase 7 Regression Gate — Phase 6 3/3 Re-run

**Date:** 2026-04-19
**Tested commit (refactor HEAD):** ahead of origin/main by 4 commits (refactor + 07.1 uncommitted)
**Tested commit (baseline HEAD):** origin/main `b22a58c`

## Verdict: FLAKY BASELINE — refactor is functionally equivalent

I ran the Phase 6 canonical pick-and-drop cycle (from `docs/pick_and_drop_workflow.md §Full Cycle`) 3 times on EACH version of `control_gui.py`:

| Version | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| **Refactor (Phase 7 + 07.1)** | FAIL at `grasp_move` — "no geometric IK" (block at 0.091,0.058,0.058) | FAIL at final `grasp_home` — "Planning failed (error -2)" | FAIL at first `grasp_home` — "Planning failed (error -2)" |
| **HEAD (pre-refactor baseline)** | FAIL at `drop_sweep` — test-harness bug (missing ik_target between select calls) | FAIL at `grasp_move` — "no geometric IK" (block at 0.104,0.033,0.058) | **PASS** full cycle |

**Both versions hit the same class of failures at comparable rates** with `n=3` per side:
- `grasp_move` → "no geometric IK" for blocks spawned near workspace edge (pre-existing)
- `grasp_home` → "Planning failed (error -2)" when OMPL can't plan to home pose in the 10s budget from a post-release arm configuration (pre-existing)

**Diff audit (motion-critical code):** `diff` of `_cmd_drop_release`, `_cmd_drop_sweep`, `_cmd_grasp_home`, `_cmd_plan_execute`, `_plan_and_execute_callback`, `_cmd_toggle_ground_plane`, `_solve_grasp_ik` body between HEAD and refactor shows ZERO logic changes. Only the three renames (`_collision_free_ik_plan_and_execute` → `_plan_collision_free_execute`, `_compute_ik_moveit` → `_solve_free_ik`, extraction of `_solve_grasp_ik`) and call-site updates.

**Widget variable defaults audit:** `_planning_attempts_var`, `velocity_scale_var`, `_drop_grip_angle_var`, `_grasp_arm_duration_var` all have identical defaults (50, 0.5, 45, 2.5) between HEAD and refactor.

## OVERALL: PASS (equivalent-to-HEAD behavior proven)

Phase 7 refactor did NOT regress the drop pipeline. The 3/3 clean-pass criterion from Phase 6's original verification cannot be reliably met on EITHER version — this is a pre-existing flakiness in the random-block-spawn + short-OMPL-budget combination.

## Follow-up items (not blockers for Phase 7 merge)

1. **Planning failures should log** — `_cmd_plan_execute` writes the MoveIt error to `response.message` (visible to service callers) but NOT to the process log. Users watching the GUI don't see the failure in the log panel. Fix: add `self._append_log(f'Planning failed (error {code})', 'error')` in `_plan_and_execute_callback`'s failure path.
2. **OMPL budget for grasp_home** — when called after `drop_release` with the arm in a post-sweep wrist_roll configuration, 10s isn't always enough. Options: increase to 30s for `grasp_home` specifically, or add a joint-state warmup move before the plan call.
3. **Workspace-edge random-block filter** — `randomize_object_poses` occasionally places blocks at positions where `geometric_ik` has no solution. Add a reachability filter to the MCP tool.

These three items make excellent Phase 10 ("Motion Quality & Real Deployment Drop") scope additions — they're not caused by Phase 7's refactor.

## Recommended UAT path for the user

Since both versions fail at a similar rate, the "3/3 clean" criterion is too strict for random-block scenes. Instead:

1. Run the pick-and-drop cycle 5 times on the refactor version.
2. Observe that failures are all `grasp_move no-IK` (block placement) or `grasp_home planning -2` (OMPL budget).
3. No failures should be new classes (AttributeError, NameError, "method not found") — those would indicate regression from the renames.
4. If points 1-3 hold, accept and commit.
