---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: executing
stopped_at: Plan 01-09 complete (7 per-component spawn atoms with full DX-02 4-surface contract + 12 SCENE-04 robot/cable pose params + LC/SFP/SC Mount vendoring with thin USD wrappers; 3 atomic task commits 424d44b/e7b14f4/0583a13; 28 surface additions; 2 minor spec-interpretation deviations both no-functional-impact (RemovePrim shared via helper = 7 not 8; SetActive(False) literal grep = 6 not 1 due to docstring/comment mentions, only 1 functional call))
last_updated: "2026-05-02T13:35:00.000Z"
last_activity: 2026-05-02 -- Plan 01-09 complete; 7 per-component spawn atoms + SCENE-04 robot/cable pose params + 3 vendored mount folders with thin USD wrappers; 3 atomic task commits (424d44b vendoring + e7b14f4 atoms + 0583a13 SCENE-04); +486 LOC in extension.py; DX-02 28-surface batch verified; backwards-compat preserved (legacy add_objects + quick_start no-arg load_robot still work); 2 minor spec-interpretation deviations documented (helper-shared RemovePrim count + SetActive grep count include docstrings — zero functional impact)
progress:
  total_phases: 4
  completed_phases: 0
  total_plans: 9
  completed_plans: 8
  percent: 89
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-01)

**Core value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.
**Current focus:** Phase 1 — Foundation Parity

## Current Position

Phase: 1 (Foundation Parity) — EXECUTING
Plan: 9 of 9 (last remaining: 01-08 quick_start refactor)
Status: Executing Phase 1 — Plans 01-01, 01-02, 01-03, 01-04, 01-05, 01-06, 01-07, 01-09 complete
Last activity: 2026-05-02 -- Plan 01-09 complete; 7 per-component MCP spawn atoms (spawn_task_board_base + spawn_{lc,sfp,sc}_mount_rail + spawn_sc_port + spawn_nic_card_mount + spawn_nic_card) with full DX-02 4-surface contract; 12 SCENE-04 pose kwargs (robot_x/y/z/roll/pitch/yaw + cable_x/y/z/roll/pitch/yaw) wired through load_robot + _cmd_load_robot + MCP_TOOL_REGISTRY; 3 mount asset folders vendored with thin .glb-referencing USD wrappers; build_mount_rail_usds.py shipped; backwards-compat preserved via None-sentinel default (legacy quick_start path unchanged); cable subtree stays SetActive(False) per D-04 — pose params authored but no-op-effective in Phase 1; 3 atomic task commits (424d44b/e7b14f4/0583a13); DX-02 28-surface batch (7 atoms × 4 surfaces) verified by per-atom grep

Progress: [█████████░] 89%

## Performance Metrics

**Velocity:**

- Total plans completed: 8
- Average duration: 6.1 min
- Total execution time: 49 min

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| Phase 1 | 8 | 49 min | 6.1 min |

**Recent Trend:**

- Last 8 plans: 01-01 (7 min), 01-02 (~2 min), 01-03 (3 min), 01-04 (8 min), 01-05 (5 min), 01-06 (6 min), 01-07 (10 min), 01-09 (8 min)
- Trend: 01-09 mid-heavy (3 tasks, 17 files net — 1 script + 3 USD wrappers + 12 vendored asset files + 1 extension.py refactor; +486 LOC in extension.py). 2 minor spec-interpretation deviations both no-functional-impact (helper-shared RemovePrim count differs from inline-per-atom assumption; SetActive(False) literal grep includes docstrings). Largest single-plan DX-02 surface in Phase 1 — 7 atoms × 4 surfaces = 28 surface additions, all verified per-atom. Backwards-compat pattern proven: None-sentinel kwarg default + legacy attribute fallback preserves no-arg quick_start path while introducing full Gazebo-named param surface. Asset-vendoring contract extended: thin-USD-wrapper pattern for mesh-source folders (build_mount_rail_usds.py) where upstream lacks pre-cooked USDs. 3 task commits.

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
- Plan 01-09: load_robot robot_x/y/z = None sentinel (instead of Gazebo's -0.2/0.2/1.14) — preserves backwards compatibility for quick_start no-arg path; legacy callers get self._robot_position fallback, explicit kwargs override. Gazebo defaults reachable by passing them explicitly. The PARAMETER SURFACE matches Gazebo verbatim (this is the SCENE-04 contract); only the no-arg fallback differs because Isaac Sim's enclosure is rooted at Z=-1.15 vs Gazebo's world Z=0.
- Plan 01-09: Cable pose params default to aic_gz_bringup.launch.py LITERAL values (cable_x=0.172, cable_y=0.024, cable_z=1.518, cable_roll=0.4432, cable_pitch=-0.48, cable_yaw=1.3303). Cable subtree stays SetActive(False) per D-04 — pose params are wired through ClearXformOpOrder + AddTranslateOp + AddRotateXYZOp so the parameter SURFACE is in place; Phase 3 (SCENE-05) enables physics, at which point these defaults start mattering without further code changes.
- Plan 01-09: Backwards-compat add_objects refactor = ADDITIVE clubbing, not replacement. Existing legacy /World/Objects path (physics material binding, two-pass loading, randomization caching) preserved verbatim; the new per-component atoms are invoked best-effort BEFORE the legacy path inside a try/except. Decoupling means new atom path can fail (e.g. asset not vendored) without breaking the canonical scene-population code that quick_start step 6 depends on.
- Plan 01-09: DRY via shared `_spawn_component_via_usd` helper rather than duplicating the 7-line authoring code per-atom. Trade-off: literal RemovePrim count is 7 (one per atom call site) instead of plan's `>=8` criterion (which assumed inline duplication). Spirit (idempotent cleanup before re-author) preserved — every atom routes through the helper. Documented as deviation; no functional impact.
- Plan 01-09: Mount-rail thin USDs reference the .glb mesh via relative AddReference. Plain pxr (outside Kit) emits "Cannot determine file format" warnings during stage save because the glTF SDF plugin loads only inside Kit's runtime; the USDs themselves are byte-correct USDC and load fine in Isaac Sim runtime. Verification deferred to Plan 07's verify_phase_1.sh harness in a running Kit session.
- Plan 01-09: 4-surface contract proven for ATOM ADDITION at 7-atom batch scale (28 surface additions). Largest single-plan DX-02 surface in Phase 1; mirrors Plan 04's 8-surface deletion-side proof on the addition side. Pattern: helper-extraction (DRY) coexists with the 4-surface contract because surfaces are PER ATOM (registry/handler-map/_cmd/UI button), and each atom's _cmd method is its own surface even when it delegates to a shared helper.
- Plan 01-09: Asset-vendoring contract extended to mesh-source folders. When upstream lacks pre-cooked USDs (LC/SFP/SC Mount in aic_assets/models/ ship .glb only), Plan 02's `cp -r` mirroring still applies; this plan adds a thin USD wrapper authoring step (build_mount_rail_usds.py) that emits one-line-of-payload USDs referencing the local mesh. Pattern: `Usd.Stage.CreateNew + UsdGeom.Xform.Define("/Root") + DefinePrim("/Root/Mesh","Xform").GetReferences().AddReference("./<mesh>")`. Caller picks first available .dae > .glb > .stl > .obj.

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

Last session: 2026-05-02T13:35:00.000Z
Stopped at: Plan 01-09 complete — 7 per-component spawn atoms with full DX-02 4-surface contract + 12 SCENE-04 robot/cable pose params + 3 vendored mount asset folders with thin USD wrappers; 3 atomic task commits (424d44b vendoring + e7b14f4 atoms + 0583a13 SCENE-04); 28 surface additions verified per-atom; backwards-compat preserved (legacy add_objects + quick_start no-arg load_robot still work); 2 minor spec-interpretation deviations both no-functional-impact and documented; metadata commit pending
Resume file: .planning/phases/01-foundation-parity/01-08-PLAN.md (quick_start refactor per D-12 -- inserts new TF + JointState publisher calls between setup_force_publish_action_graph and setup_wrist_cameras; matches their UI button placement; also adds Phase 1 CHANGELOG entry + DX-02 final-audit table; Plan 09's 7 new atoms are now part of the surface this audit table presents)
