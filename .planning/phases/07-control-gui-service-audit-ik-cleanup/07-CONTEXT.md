# Phase 7: Control GUI Service Audit & IK Cleanup - Context

**Gathered:** 2026-04-18
**Status:** Ready for planning

<domain>
## Phase Boundary

Code-hygiene pass over the SO-ARM101 control GUI. Four workstreams:

1. **Button↔service audit with runtime + CI enforcement** — every `tk.Button` / `ttk.Button` in the control GUI routes to a `_cmd_*` method (no inline-logic lambdas), a runtime `~/dump_services` Trigger prints the mapping, a unit test asserts 1:1 coverage, and `docs/AGENT_DEBUG_GUIDE.md` is regenerated from the dump.
2. **`drop_refresh` pose-update fix** — when cup positions change in `/drop_poses`, pressing Drop Refresh updates both MoveIt collision objects AND RViz visual markers to the new poses atomically. Remove the `root.after(500/600/700)` chain.
3. **Pragmatic IK consolidation** — extract shared helpers, rename entry points for clarity (grasp vs freeform vs planned-execute). Keep MoveIt KDL as the documented fallback for IK-tab freeform xyz+quaternion moves.
4. **FK/IK duplicate-widget consolidation** — extract the Reset Arm / Randomize / Reset Gripper / Open / Close cluster duplicated between FK tab (~L976) and IK tab (~L1884) into a single shared helper frame.

**Target repo:** `~/Projects/Exploring-VLAs/vla_SO-ARM101` (single repo). No Isaac Sim extension or aruco_camera_localizer changes.
**Primary file:** `src/so_arm101_control/so_arm101_control/control_gui.py` (3816 lines).

</domain>

<decisions>
## Implementation Decisions

### Button↔Service Audit & Enforcement (Workstream A)
- **D-01:** Ban inline-logic lambdas on Button `command=`. Every button either (a) calls a `_cmd_*` method directly, or (b) calls a thin GUI-only wrapper (e.g. `_ik_btn_plan_execute`) that validates local UI state and then invokes a `_cmd_*` method. No inline computation in the lambda body.
- **D-02:** Add `_cmd_dump_services` that returns the full button→service table as plain text. Auto-registers as `~/dump_services` Trigger via the existing `_cmd_*` convention (control_gui.py L417–L425). Output format: markdown table ready to paste into `AGENT_DEBUG_GUIDE.md`.
- **D-03:** Add a unit test in `src/so_arm101_control/so_arm101_control/` that introspects the widget tree (or a new registry helper) and asserts every Button.command resolves to a method whose name starts with `_cmd_` OR matches the `_*_btn_*` thin-wrapper pattern. The test fails CI if a new inline lambda with logic is added. This is the enforcement gate that would have caught the historical Drop Refresh lambda-vs-service divergence.
- **D-04:** Regenerate `docs/AGENT_DEBUG_GUIDE.md` by capturing the output of `_cmd_dump_services` at build time. The guide is now a byproduct of the runtime truth, not hand-maintained.

### drop_refresh Pose-Update Fix (Workstream B)
- **D-05:** Root-cause target: `_drop_data.clear()` (L3083) races ahead of the `/drop_poses` topic callback. Markers publish from empty data before the topic repopulates.
- **D-06:** Fix approach — sequence against FRESH data:
  1. Capture a snapshot of the CURRENT `_drop_data` (the cups to remove).
  2. Remove those cup collisions from the planning scene (reuse `_remove_cup_collision_objects`).
  3. Publish marker DELETEs for those cup ids (reuse `_delete_visual_markers` pattern).
  4. Invalidate `_drop_data` and wait for the next `/drop_poses` message to arrive (with a configurable timeout, e.g. 2.0s).
  5. Once fresh data arrives, add NEW cup collisions + republish markers in one atomic pass.
- **D-07:** Remove the `root.after(500)` / `after(600)` / `after(700)` chain entirely. The new sequence runs on a background thread with `threading.Event`-gated steps, consistent with the `_motion_event` pattern already used in drop commands.
- **D-08:** `_refresh_display_markers` must run regardless of `_show_visual_var` toggle when invoked by `_cmd_drop_refresh` — the toggle controls visibility at rest, but a refresh implies the user wants the current truth pushed. Use a dedicated `_publish_cup_visual_markers` call, not the toggle-gated path.
- **D-09:** Handle the "cup disappeared from topic" case: if fresh `/drop_poses` omits a cup that was previously present, its collision object AND marker are removed, not left stale.

### Pragmatic IK Consolidation (Workstream C)
- **D-10:** "ONE IK path" is interpreted pragmatically: ONE organized pipeline with named stages, NOT deletion of `_compute_ik_moveit`. The IK tab's freeform xyz+quaternion moves depend on the MoveIt KDL multi-seed solver — geometric IK is top-down-grasp-only. Deleting KDL regresses a real feature.
- **D-11:** Rename the three entry points for clarity:
  - `_collision_free_ik_plan_and_execute` → `_plan_collision_free_execute` (makes it clear this is the *execution wrapper*, not an IK solver)
  - `_compute_ik_full` grasp-yaw branch → extract to `_solve_grasp_ik` (geometric, top-down)
  - `_compute_ik_moveit` → rename to `_solve_free_ik` (MoveIt KDL, freeform orientation)
- **D-12:** Extract shared helpers that currently duplicate across the three paths:
  - `_ik_apply_and_act` (already shared at L2697 — audit callsites for correctness)
  - `_check_state_valid` (already shared at L2681 — confirm only entry point)
  - A new `_dispatch_ik_trajectory(target, mode, on_complete)` that owns the mode switch between `_cmd_set_joints` / `_cmd_plan_execute` / `_execute_trajectory` for grasp_approach/grasp_execute modes.
- **D-13:** Motion entry points stay as two named paths, NOT collapsed into one:
  - `_execute_trajectory` (L3540) — direct-send for trajectory-shaped moves (Point to Drop, grasp descent, drop sweep wrist animation)
  - `_cmd_plan_execute` (L1260) — MoveIt-planned collision-aware path (Drop Sweep IK target, grasp pre-approach, FK Plan & Execute button)
  - `_cmd_set_joints` (L1180) keeps its direct `_send_arm_goal` call — it's a 0.5s slider-quick-move with different semantics, not a trajectory. Document this explicitly in the code header comment.
- **D-14:** Non-negotiable acceptance gate: re-run Phase 6's 3-consecutive-clean-passes verification for all three cups (drop_0, drop_1, drop_2) AFTER the IK consolidation, BEFORE merging. Any regression blocks the phase.
- **D-15:** Preserve behavior byte-for-byte inside `_collision_free_ik_plan_and_execute` / `_plan_collision_free_execute`. This phase is organization + renaming, NOT logic changes. No tweaks to `grip_angle`, `wrist_roll`, OMPL config, planning attempts, or padding.

### FK/IK Duplicate-Widget Consolidation (Workstream D)
- **D-16:** Extract the cluster of duplicate buttons (`Reset Arm`, `Randomize`, `Reset Gripper`, `Open`, `Close`) that appear on both the FK tab (~L976) and IK tab (~L1884) into a single `_build_arm_gripper_btn_cluster(parent)` helper. Called from both tab builders.
- **D-17:** Tkinter layout preservation: the helper returns the same frame shape as the existing duplicated code — no grid/pack reflow. Visual output on both tabs must be pixel-identical to the current layout (verify with screenshot diff before/after using the x11-window-screenshot skill).
- **D-18:** The helper binds to the SAME `_cmd_*` methods already in use — no new services added or renamed as part of this consolidation.

### Claude's Discretion
- Exact order of commits/plan splits across the four workstreams (but suggest: audit → drop_refresh → tab consolidation → IK cleanup → Phase 6 regression, in that risk-ascending order).
- Naming of the internal service-registry helper used by the audit test.
- Whether the unit test walks the live widget tree at GUI-startup time or parses the AST of control_gui.py to find Button constructors. Either meets the enforcement goal.
- Timeout value for the drop_refresh "wait for fresh data" step (start at 2.0s, tune if flaky).

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Phase origin and scope (original user intent)
- `.planning/PROJECT.md` — Project vision and constraints (no commits until user confirms, subagent model pinning).
- `.planning/ROADMAP.md` §"Phase 7: Control GUI Service Audit & IK Cleanup" — five literal success criteria. Pragmatic reading of criterion #3 ("ONE IK path") is locked in D-10 above.
- `.planning/STATE.md` §"Accumulated Context" — Phase 6 verification notes that constrain IK consolidation.
- `.planning/phases/02-arm-drop-motion/02-CONTEXT.md` — established `_cmd_*` auto-registration convention, grasp-tab structure, drop subscriber pattern.

### Target source of truth
- `~/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py` — the only source file that changes substantively in this phase. Key landmarks:
  - L417–L425 — `_cmd_*` Trigger auto-registration factory (the convention to preserve).
  - L1180–L1192 — `_cmd_set_joints` (direct-send entry point; stays direct per D-13).
  - L1204–L1254 — `_collision_free_ik_plan_and_execute` (renamed to `_plan_collision_free_execute` per D-11; behavior preserved per D-15).
  - L1260–L1329 — `_cmd_plan_execute` (planned-execute entry point; stays per D-13).
  - L2494–L2536 — `_ik_btn_*` GUI-side wrappers (the `_*_btn_*` thin-wrapper pattern referenced in D-01).
  - L2540–L2607 — `_compute_ik_full` (split into `_solve_grasp_ik` extraction per D-11).
  - L2609–L2679 — `_compute_ik_moveit` (renamed to `_solve_free_ik` per D-11).
  - L2681–L2695 — `_check_state_valid` (already-shared helper).
  - L2697–L2737 — `_ik_apply_and_act` (already-shared helper).
  - L2786–L2810 — `_drop_callback` + `_update_drop_topic` (drop_refresh sequencing touches these per D-06).
  - L2828–L2912 — `_add_cup_collision_objects` + `_remove_cup_collision_objects` (reused by the new drop_refresh sequence).
  - L2914–L2987 — `_publish_cup_visual_markers`, `_refresh_display_markers`, `_delete_visual_markers` (the visual-marker set that needs atomic update per D-08).
  - L3081–L3089 — `_cmd_drop_refresh` (the function to rewrite per D-06/D-07/D-08/D-09).
  - L3540 — `_execute_trajectory` (direct-send trajectory entry point; stays per D-13).
  - L976, L1884 — duplicate FK/IK button clusters extracted per D-16/D-17.

### Phase 6 regression gate
- `.planning/phases/06-collision-free-drop-motion-verification-loop/` — the 3/3 passes procedure that D-14 re-runs post-IK-consolidation.

### Downstream artifact
- `~/Projects/Exploring-VLAs/vla_SO-ARM101/docs/AGENT_DEBUG_GUIDE.md` — regenerated from `_cmd_dump_services` output per D-04.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `_cmd_*` auto-registration factory (L417–L425): every new `_cmd_foo` method automatically becomes a `~/foo` Trigger service. This is THE convention that lets the audit work at all.
- `_motion_event` / `threading.Event` pattern: used throughout drop commands to make services block until trajectory completion. Reuse for the drop_refresh "wait for fresh data" step (D-06).
- `_check_state_valid` (L2681) and `_ik_apply_and_act` (L2697): already-shared helpers across IK paths. Audit only, no extraction needed.
- `_remove_cup_collision_objects` (L2889) and `_delete_visual_markers` (L2975): reusable for the drop_refresh "remove old" step.
- `x11-window-screenshot` skill: for D-17 pixel-identical layout verification before/after widget extraction.
- Phase 6's verification test harness (details in phase 06 directory): the regression gate for D-14.

### Established Patterns
- Background thread + `threading.Event` for async ROS2 calls (e.g. `_add_cup_collision_objects` spawns `_apply` thread at L2887). drop_refresh fix follows this pattern instead of the current `root.after` chain.
- Services call GUI methods via a dispatch-to-tkinter-thread wrapper (L466). Stays — the new `_cmd_dump_services` piggybacks on this.
- Rename-don't-delete discipline: Phase 05.2 kept `geometric_ik` + parameterized it with `grip_angle`/`wrist_roll` rather than forking. Phase 7 follows suit with the `_solve_grasp_ik` / `_solve_free_ik` rename over deletion.

### Integration Points
- Unit test lives under the existing `src/so_arm101_control/so_arm101_control/` test infrastructure (see `test_debug_services.py` L2-line file as a reference pattern).
- `~/dump_services` introspects the `self._debug_services` list populated at L418–L425 — no new infrastructure, just iteration.
- The regenerated `AGENT_DEBUG_GUIDE.md` goes at its existing path (`docs/AGENT_DEBUG_GUIDE.md`); the regeneration script should be repo-local and reproducible.

</code_context>

<specifics>
## Specific Ideas

- **Drop Refresh bug user-visible symptom:** "the visual or collision marker don't update the pose of the cups in rviz when I press drop refresh" (user turn 64, session 695a7f93). The fix must be verifiable by the "move a cup in Isaac Sim → press Drop Refresh in the GUI → confirm RViz shows the cup collision + visual at the NEW pose" check.
- **Audit motivation user-voiced:** "audit the so arm packages gui such that each of the buttons have a mapped service and it's not like each service is a separate implementation apart from the button code... agents don't make the same mistakes again" (user turn 61, session 695a7f93). The historical Drop Refresh lambda-vs-service divergence is the canonical example of what the enforcement test must prevent.
- **IK cleanup framing:** "cleaning out functions duplicates and find one proper way of executing all the codes" — pragmatic consolidation, not deletion. `_compute_ik_moveit` stays (renamed).
- **FK/IK tab consolidation choice was explicit** — user overrode the "defer" recommendation with "No, consolidate now." D-16/D-17/D-18 reflect that.

</specifics>

<deferred>
## Deferred Ideas

- Refactoring `_cmd_grasp_move` internals (L3313) — it works, it's Phase 6-verified, touching it invites regression risk not matched by cleanup value. Out of scope for Phase 7.
- Replacing the multi-seed MoveIt KDL solver with a better analytical free-IK (e.g. IKFast, TracIK integration) — separate investigation, future phase.
- Consolidating the THREE gripper button pairs (Grasp Open/Close, Open/Close range, Open/Close direct at L2072–L2081) — that's UX consolidation, distinct from Workstream D's arm-button-cluster duplication. Defer.
- Rewriting the CAD cup mesh loader's cache-invalidation path (L2954–L2957) — works today, not part of the `drop_refresh` symptom.
- Merging `/drop_poses` and `/objects_poses` into a unified pose source with `frame_id` namespacing — Phase 8/9 territory.

</deferred>

---

*Phase: 07-control-gui-service-audit-ik-cleanup*
*Context gathered: 2026-04-18*
