---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: executing
stopped_at: Plan 01-07 complete (verify harness shipped — diff_tf_tree.py + sweep_textures.py + verify_phase_1.sh + texture-sweep.md; 4 files, 3 task commits, all acceptance criteria PASS; smoke tests confirm regex sanity; PARITY-03/04 deferrals from Plan 06 will be visible to verify_phase_1.sh as honest FAILs gating Phase 1 completion)
last_updated: "2026-05-02T18:30:00.000Z"
last_activity: 2026-05-02 -- Plan 01-07 complete; 3 scripts + 1 doc shipped (diff_tf_tree.py 71 lines, sweep_textures.py 200 lines, verify_phase_1.sh 441 lines, texture-sweep.md scaffold); 3 atomic task commits (fea9efd diff_tf_tree + d883e4d sweep + 0d17ef4 verify); zero deviations; smoke tests: TF diff PASS 31 frames/30 edges on aic_frames_live.gv self-compare, sweep_textures captured 267 lines from existing Kit log including the augmented `Could not open` pattern catching sc_port_visual.usd broken sub-references, verify_phase_1.sh bash -n clean + --help works + --bogus correctly exits 2; 15/15 literal-token acceptance grep PASS; key pattern proven: audit-files-as-machine-readable-contracts (parity_05_wrench_framing.txt + joint_ordering_probe.txt grepped by verify script for Live frame_id / Verdict / Action fields)
progress:
  total_phases: 4
  completed_phases: 0
  total_plans: 9
  completed_plans: 7
  percent: 78
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-01)

**Core value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.
**Current focus:** Phase 1 — Foundation Parity

## Current Position

Phase: 1 (Foundation Parity) — EXECUTING
Plan: 8 of 9
Status: Executing Phase 1 — Plans 01-01, 01-02, 01-03, 01-04, 01-05, 01-06, 01-07 complete
Last activity: 2026-05-02 -- Plan 01-07 complete; verification harness shipped (3 scripts + 1 doc); diff_tf_tree.py D-08 self-test PASS 31/30; sweep_textures.py D-07 augmented PATTERNS confirmed via 267-hit smoke test on existing Kit log catching the known sc_port_visual.usd `Could not open` failures; verify_phase_1.sh D-15 10-step hybrid harness with port-detect + cold-launch fallback + audit-file consumption (parity_05_wrench_framing.txt + joint_ordering_probe.txt as machine-readable contracts); zero deviations; 3 atomic task commits

Progress: [███████░░░] 78%

## Performance Metrics

**Velocity:**

- Total plans completed: 7
- Average duration: 5.9 min
- Total execution time: 41 min

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| Phase 1 | 7 | 41 min | 5.9 min |

**Recent Trend:**

- Last 7 plans: 01-01 (7 min), 01-02 (~2 min), 01-03 (3 min), 01-04 (8 min), 01-05 (5 min), 01-06 (6 min), 01-07 (10 min)
- Trend: 01-07 mid-weight (3 tasks, 4 files created — 3 scripts + 1 doc; 712 LOC total). Zero deviations, zero auto-fixes. All three scripts smoke-tested live: diff_tf_tree.py self-compare PASS, sweep_textures.py 267-hit grep proves augmented patterns work, verify_phase_1.sh `bash -n` clean + arg parsing tested. Pattern proven: audit-files-as-machine-readable-contracts (Plan 04's parity_05_wrench_framing.txt and Plan 05's joint_ordering_probe.txt have standardized field names that downstream verify scripts grep). 3 task commits.

*Updated after each plan completion*

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table. Recent decisions affecting current work:

- Repo split: sim-side here, ROS-side / controller / policies / engine in `~/Documents/aic` (read-only)
- Topic parity with Gazebo (no `_sim`/`_real` suffixes, no bridges) is the architectural law
- Same USD/URDF as Gazebo (`aic_description`, `aic_assets`); zero-divergence guarantee
- Pose-source seam = `/tf` (M1 publishes GT into `/tf`; M2 swaps publisher only)
- Cable physics strategy deferred to Phase 3 research (use `nvidia-suite-docs` skill)
- M1 success bar = all `sample_config.yaml` trials pass under CheatCode against Isaac Sim
- MCP atomic + clubbed model preserved (`MCP_TOOL_REGISTRY` + `_cmd_<name>` + per-tool button + `quick_start` clubbing)
- Plan 01-01: image digest captured = `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433` (D-14 pin)
- Plan 01-01: live aic_eval surface is 36 topics (not 35; +2 controller motion_update topics drift since research)
- Plan 01-01: `docker inspect` for RepoDigests must target the image, not the running container — running containers may have empty `.RepoDigests` field
- Plan 01-02: vendored 5 capitalized AIC asset folders (`NIC Card`, `NIC Card Mount`, `SC Port`, `SC Plug`, `Task Board Base`) under `exts/aic-dt/assets/assets/` — 21 files total (9 USDs + 12 textures), md5sum byte-identical to upstream
- Plan 01-02: snake_case `exts/aic-dt/assets/objects/` retired in same plan; production extension is broken-by-design between this plan and Plan 04 lands (intentional sequencing per the plan)
- Plan 01-02: `cp -r` whole-folder is the vendoring contract — selective `.usd`-only copy reintroduces TEX-01 (missing sibling textures)
- Plan 01-02: AIC's CAPITALIZED layout (with spaces) preserved verbatim — USDs reference `./textures/...` relatively; renaming would re-break TEX-01 without USD rewrites
- Plan 01-03: AIC URDF xacro (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro`) is the canonical gripper-identification source — it includes `Robotiq Hand-E/robotiq_hande_macro.xacro`, NOT RG2; doc surface (`.planning/`, `CLAUDE.md`, `exts/aic-dt/docs/`) corrected accordingly
- Plan 01-03: ROADMAP.md plan-description lines naming this plan reworded from "RG2→Robotiq Hand-E correction" to "gripper-name correction (Robotiq Hand-E)" — keeps zero literal `RG2` tokens in the doc surface
- Plan 01-03: Stash-and-restore pattern used to keep pre-existing uncommitted CLAUDE.md edits out of plan commits — only RG2 hunks landed in commit `397b530`, unrelated dirty state preserved as before
- Plan 01-03: PARITY-01 doc wording corrected; actual implementation (URDF load) is downstream — PARITY-01 status remains `[ ]` until code path lands (likely 01-04/01-06)
- Plan 01-04: Live wrench frame_id sourced from URDF (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` line 242 — `AtiForceTorqueSensor` `param name="frame_id"` -> `ati/tool_link`) instead of running aic_eval container — saved ~60s docker bringup for a single string read; URDF is what Gazebo's ros2_control plugin loads at runtime
- Plan 01-04: Joint-path bug fix (`/World/UR5e/joints/<n>` -> `/World/UR5e/aic_unified_robot/joints/<n>`) revealed that joint drives were silently never applied pre-fix — UR5e ran on engine defaults; the 6x "Joint not found" Kit warning had been a long-standing diagnostic noise (not a runtime crash). Plan 07 verify will confirm the warning disappears.
- Plan 01-04: Forward-declared `self._articulation_root_prim_path = "/World/UR5e/aic_unified_robot"` opportunistically while the surface was hot — Plan 06's JointState publisher targetPrim relationship now has a single source of truth instead of needing to recompute the suffix.
- Plan 01-04: 4-surface atom-DELETION contract proven (DX-02 deletion side): registry + handler-map + _cmd method + UI button + caller (notably quick_start). 8 surfaces total across 2 atoms (setup_pose_publisher, sync_real_poses).
- Plan 01-04: quick_start step 7 left as a comment placeholder (NOT reordered) — Plan 07 owns the reorder once Plan 06 lands the new TF/JointState publishers. This plan deliberately ships an interim broken state (consistent with the plan-level note in 01-04-PLAN.md).
- Plan 01-04: PARITY-05 reconciliation pattern = `# PARITY-XX:` inline marker comment + docstring contract (topic / type / frame_id / source-of-truth file refs / verify recipe) + audit-trail .txt file in phase dir with standardized fields (Live frame_id, Live Type, Action taken, Verification command). Plan 07 verify_phase_1.sh consumes both the docstring recipe and the audit-trail file fields.
- Plan 01-04: Per-task verify regexes in PLAN.md were over-strict for intermediate states — Task 1's verify asserted `_sim\b|_real\b` returns 0 hits, but `objects_poses_sim/real` (deleted in Task 3, not renamed in Task 1) tripped it. Treated plan-level end-of-plan grep as authoritative gate; flagged in summary "Issues Encountered" for future planners.
- Plan 01-05: PARITY-04 strategy = PER-FRAME-RAW-OVERRIDE (NOT SUBLAYER-RENAME). USD prim names cannot legally contain `/` (Sdf.Path separator), and ROS2PublishTransformTree reads the prim leaf name not customData metadata — so sublayer rename via `over` blocks or customData:frameId attributes has no effect on published frame_id. Plan 06 must use per-frame ROS2PublishRawTransformTree nodes (or equivalent override-array inputs on TransformTree) for the 16 underscore->slash mismatches + 1 synthesized aic_world edge. Captured in usd_prim_inventory.txt `[Override list for Plan 06 / sublayer]` block (17 entries).
- Plan 01-05: PARITY-03 verdict = NAME-INDEXED via aic_adapter::ReorderJointState (`~/Documents/aic/aic_adapter/src/aic_adapter.cpp:80-86` for sort-order map; lines 266-279 for the reorder loop). aic_controller is a ros2_control plugin reading hardware-state interfaces, not /joint_states topic — initial probe scope (aic_controller/src) returned UNKNOWN; the actual subscriber lives in a sibling package. Manual probe extension over aic_adapter/src found explicit name-keyed reordering. Plan 06 publishes Isaac Sim's natural articulation order without a reorder bridge.
- Plan 01-05: One strict contract surfaced for Plan 06: aic_adapter's joint_sort_order_ map at line 86 has the literal key `gripper/left_finger_joint` (with slash). Isaac Sim's USD has the joint at `/World/aic_unified_robot/joints/gripper_left_finger_joint` (with underscore). Plan 06 MUST override the published joint_state name for this single joint, or aic_adapter logs "Ignoring unexpected joint name" and silently drops it from every Observation. This is the JointState analog of the TF frame_id override problem.
- Plan 01-05: Probe-running interpreter is `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh` (cp310 + pxr 0.26.3), NOT `~/env_isaaclab/bin/python` (cp311 from uv, no pxr installed). Future re-runs of probe_unified_usd.py must use isaac-sim's bundled python — env_isaaclab does NOT have the pxr OpenUSD module. (Could install `usd-core` pip package as alternative, but isaac-sim's bundled pxr is the same vendor that powers the runtime.)
- Plan 01-05: Pre-graph probe pattern proven — cheap minutes-long upstream-source inspection BEFORE wiring OmniGraph nodes that depend on the verdicts. When the auto heuristic returns UNKNOWN (here: aic_controller doesn't subscribe to /joint_states), DON'T re-run with different defaults — append manual extension sections [6]+[7] to the same .txt report so the entire reasoning trail stays in one file. Conditional task pattern (Tasks 3+4 gated on `Strategy:` / `Action:` regex matches) avoids over-engineering when the probe says no fix is needed.
- Plan 01-06: OGN-spec verification before authoring -- when a probe plan's verdict (here Plan 05's PER-FRAME-RAW-OVERRIDE + JointState gripper finger rename) implicitly assumes a node has certain inputs, READ the local OGN .rst spec from `~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/docs/ogn/Ogn*.rst` BEFORE authoring the builder. Caught two architectural gaps before runtime debugging: (a) ROS2PublishRawTransformTree takes static translation/rotation, NOT prim relationships; (b) ROS2PublishJointState has no jointNames/nameOverrides input. Both gaps documented inline in builder docstrings + Plan 08 verify-harness contract -- the verify gate must FAIL loudly on the deferred items, not silently pass.
- Plan 01-06: Plan-body deferral as authoritative when user-objective and PLAN.md disagree on scope. The user-spawn objective text said "use Raw overrides per Plan 05 verdict"; PLAN.md L51/L302 said "default mapping, accept iteration loop". The PLAN.md governs (it's the canonical contract). Documented the disagreement in 01-06-SUMMARY.md "Deviations" so the discrepancy is surfaced for future planners.
- Plan 01-06: 4-surface contract proven for ATOM ADDITION (mirrors Plan 04's deletion-side proof). DX-02 fully exercised: registry + handler-map + _cmd method + UI button = 4 surfaces per atom, 8 surfaces total for 2 atoms in this plan.
- Plan 01-06: JointState targetPrim binds at /World/UR5e/aic_unified_robot/root_joint (PhysicsFixedJoint), NOT at the Xform parent. Per RESEARCH Pattern 1 line 466 + verified by Task 0 probe_root_joint.py. Plan 04's forward-declared self._articulation_root_prim_path provides the Xform path; Plan 06 appends "/root_joint" at runtime.
- Plan 01-06: Task 0 probe_root_joint.py introduces a re-runnable PhysicsFixedJoint verifier that complements Plan 05's USD-prim probe. Run via `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/probe_root_joint.py | tee /tmp/root_joint_probe.txt` -- exit 0 = PASS / 2 = WARN (Xform exists no joint) / 1 = FAIL (USD missing or restructured). Plan 06 PASS verified.

### Pending Todos

None yet.

### Blockers/Concerns

- Cable physics fidelity strategy is the highest-risk requirement (SCENE-05) — research must land in Phase 3 before cable spawn / gripper-attach implementation can converge.

## Deferred Items

Items acknowledged and carried forward from previous milestone close:

| Category | Item | Status | Deferred At |
|----------|------|--------|-------------|
| *(none — M1 is the first milestone)* | | | |

## Session Continuity

Last session: 2026-05-02T18:30:00.000Z
Stopped at: Plan 01-07 complete — verify harness shipped (3 scripts + 1 doc); 3 atomic task commits (fea9efd diff_tf_tree, d883e4d sweep + texture-sweep.md, 0d17ef4 verify_phase_1.sh); zero deviations, all acceptance criteria PASS first try; smoke tests confirm regex sanity; audit-file-as-machine-readable-contract pattern proven (parity_05_wrench_framing.txt + joint_ordering_probe.txt grepped for standardized field names). PARITY-03/04 deferrals from Plan 06 will surface as honest FAILs in verify_phase_1.sh — the script does not paper over them; metadata commit pending
Resume file: .planning/phases/01-foundation-parity/01-08-PLAN.md (quick_start refactor per D-12 -- inserts new TF + JointState publisher calls between setup_force_publish_action_graph and setup_wrist_cameras; matches their UI button placement; also adds Phase 1 CHANGELOG entry + DX-02 final-audit table)
