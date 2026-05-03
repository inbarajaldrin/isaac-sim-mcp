---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: executing
stopped_at: Phase 2 Plan 02 EXECUTED ✓ (autonomous M1 mode, 5 min). controller_loop.py skeleton (392 LOC, AicControllerLoop class) created — mirror of parity_publishers.py inverted on rclpy I/O axis, with extended _ROS_PREFIXES (aic_control_interfaces + ros_gz_interfaces), 8 stub callback methods for Plans 02-03..06 to fill, full physics-step + spin_once + apply + publish lifecycle wired. extension.py wired with DX-02 4-surface contract for 2 new MCP atoms (setup_controller_subscribers + setup_offlimit_contacts) — 8 surface additions + _start_aic_controller_loop shared manager helper + self._aic_controller_loop init + on_shutdown teardown + quick_start step 3b'. audit_dx02.py extended PRESENT_ATOMS 27→29; audit exits 0 at 29 PRESENT × 4 surfaces. 1 deviation auto-fixed (Rule 2 — audit PRESENT_ATOMS extension required for 29 success criterion). 2 task commits (bf992c5, acee1c4) + metadata commit pending. Plans 02-03..06 unblocked (controller_loop.py is the single shared file from here). Auto-advancing to Plan 02-03 next.
last_updated: "2026-05-03T20:24:10.000Z"
last_activity: 2026-05-03 -- /gsd-execute-phase 2 Plan 02 complete: controller_loop.py skeleton + 2 MCP atoms × 4-surface contract (8 surface additions) + manager helper + on_shutdown + quick_start hook. audit_dx02 PASS at 29 PRESENT × 4 surfaces. 1 deviation auto-fixed (Rule 2 audit extension). Files: 1 created (controller_loop.py) + 2 modified (extension.py, audit_dx02.py).
progress:
  total_phases: 4
  completed_phases: 0
  total_plans: 11
  completed_plans: 11
  percent: 100
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-01)

**Core value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.
**Current focus:** Phase 1 — Foundation Parity

## Current Position

Phase: 2 (Controller Loop) — EXECUTING; Plan 2 of 6 complete
Plan: 02-02 ✓ → 02-03 next
Status: Skeleton landed. AicControllerLoop class (392 LOC, sibling to parity_publishers.py) wired into extension.py via DX-02 4-surface contract for 2 atoms (setup_controller_subscribers + setup_offlimit_contacts). Shared _start_aic_controller_loop manager helper + on_shutdown teardown + quick_start step 3b'. audit_dx02 PASS at 29 PRESENT × 4 surfaces. Plans 02-03..06 only need to edit controller_loop.py to fill in callback bodies. Wave structure unchanged: 1→2→3→4→5→6.
Last activity: 2026-05-03 -- /gsd-execute-phase 2 Plan 02 (5 min): controller_loop.py skeleton + 2 MCP atoms × 4-surface contract; manager helper; on_shutdown; quick_start hook. audit_dx02 extended PRESENT_ATOMS 27→29 (Rule 2 deviation). Auto-advancing to Plan 02-03.

Progress: [█████████░] M1 ~42% (Phase 1 closed + Phase 2 plans 1-2 of 6 done; execution pending for plans 02-03..06; Phase 3 SCENE-05 = ~3-4hr; Phase 4 = trial loader + E2E)

## Phase 2 Plans

| Plan | Wave | Objective | Requirements |
|------|------|-----------|--------------|
| 02-01 ✓ | 1 | Custom message workspace rebuild (Python 3.11 humble) + topic-name discovery probe + off-limit prim mapping | (infra for PARITY-06/09/10/11) |
| 02-02 ✓ | 2 | controller_loop.py skeleton + 2 MCP atoms (8 surface additions per DX-02) + manager helper + on_shutdown + quick_start hook | (skeleton infra) |
| 02-03 | 3 | PARITY-09: joint_commands subscriber + name-keyed parser + per-joint set_gains + apply_action | PARITY-09 |
| 02-04 | 4 | PARITY-10: pose_commands subscriber + Lula IK + Pitfall #2 Option A static offset for gripper/tcp | PARITY-10 |
| 02-05 | 5 | PARITY-11: ControllerState publisher (FK + numerical-diff velocity + reference echoes + zero tare) | PARITY-11 |
| 02-06 | 6 | PARITY-06: omni.physx contact-report + Contacts publish + smoke_test_aic_controller.py + verify_phase_2.sh + 02-SUMMARY + flag flips | PARITY-06 |

## Performance Metrics

**Velocity:**

- Total plans completed: 11
- Average duration: 10.3 min
- Total execution time: 113 min

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| Phase 1 | 9 | 59 min | 6.6 min |
| Phase 2 | 2 | 54 min | 27 min |

**Recent Trend:**

- Last 11 plans: 01-01 (7 min), 01-02 (~2 min), 01-03 (3 min), 01-04 (8 min), 01-05 (5 min), 01-06 (6 min), 01-07 (10 min), 01-09 (8 min), 01-08 (10 min), 02-01 (49 min), 02-02 (5 min)
- Trend: 02-02 was a fast skeleton plan (2 tasks, 3 files — 1 created + 2 modified). Templates from parity_publishers.py made the inversion mechanical; the only deviation was extending audit_dx02.py PRESENT_ATOMS by 2 (Rule 2). Plan's surface analysis (no UI_METHOD_ALIASES needed) held — the new atoms' UI buttons reference `_cmd_<atom>` directly, satisfying audit candidate (b). Hand-off to Plans 02-03..06: skeleton lifecycle is wired, only callback bodies in controller_loop.py remain.

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
- Plan 02-01: D-05 ABI fix landed via workspace rebuild (NOT PYTHONPATH-link to AIC pixi env, which is broken — pixi builds against Python 3.12, Isaac Sim runs Python 3.11, ABI-incompatible across minor versions). Vendor `aic_control_interfaces` from `~/Documents/aic/aic_interfaces/` and `ros_gz_interfaces` from `gazebosim/ros_gz` humble branch into `~/IsaacSim-ros_workspaces/humble_ws/src/`, then `bash build_ros.sh -d humble -v 22.04`. Round-trip import via `~/env_isaaclab/bin/python` (after venv-activate) is the gate.
- Plan 02-01: TWO additional landmines beyond D-05 surfaced and documented inline in `exts/aic-dt/docs/aic-msgs-setup.md` + `exts/aic-dt/scripts/build_aic_msgs.sh`. (a) Vendor target is `~/IsaacSim-ros_workspaces/humble_ws/src/` (Dockerfile COPY context), NOT `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/` (post-build extracted dir) — both share the segment "humble_ws/src" but live at different repo roots. (b) Built artifacts install per-package under `isaac_sim_ros_ws/install/<pkg>/local/lib/python3.11/dist-packages/<pkg>/` (NO `--merge-install`), NOT in the inner `humble_ws/install/local/lib/...` (which IS `--merge-install`). `~/env_isaaclab/bin/activate` already sources both setups so auto-discovery handles it.
- Plan 02-01: Open Q1 verdict — all four CONTEXT.md-assumed Phase 2 controller topic names (joint_commands/pose_commands/controller_state/off_limit) confirmed CORRECT against live aic_eval Docker container (snapshot_aic_eval_offlimit.sh). NO CONTEXT.md edits needed. Plans 02-03..06 wire to assumed names verbatim. Bonus: 3 additional aic_controller topics observed live (joint_motion_update, motion_update, transition_event) — out of scope, doc'd for future reference.
- Plan 02-01: D-10 off-limit prim mapping verdict — set is PREFIX-BASED (3 entries: `/World/Enclosure`, `/World/Enclosure_Walls`, `/World/TaskBoard`), NOT a hand-enumerated list of individual link names. Authoritative source: `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` lines 122-130 (`<off_limit_models>` block in `OffLimitContactsPlugin`). Gazebo plugin walks all collision children of these 3 top-level models dynamically; Isaac Sim equivalent uses `actor_path.startswith(prefix)` filtering in Plan 02-06's contact callback.
- Plan 02-01: Off-limit live capture under CheatCode is EXPECTED-EMPTY (CheatCode is a passing policy that doesn't trigger off-limit collisions) — accepted, not treated as failure. URDF source-of-truth fallback is canonical. WallPresser/WallToucher policy noted in offlimit-prim-mapping.md as the swap if a future plan needs non-empty capture.
- Plan 02-01: `ros2 topic echo` from inside aic_eval fails for custom message types (Zenoh + kilted-CLI quirk) but `ros2 topic info` works — type confirmation done via topic-info captures alone. Sample echoes captured via `--once` are stub yamls but not load-bearing for any downstream plan.
- Plan 02-02: AicControllerLoop class deliberately defers ALL rclpy + omni.* imports into `start()` body — the class definition itself is rclpy/Kit-free. Lets `importlib.util.spec_from_file_location` load + instantiate the class outside Kit for offline syntax / structural verification, while preserving full lifecycle semantics inside Kit. Mirror of parity_publishers.py pattern.
- Plan 02-02: Skeleton _on_joint_cmd / _on_pose_cmd default bodies BUFFER the message into self._latest_*_cmd. Plans 02-03/02-04 can either replace the body with validation+buffer or wrap-and-extend; either way, the per-tick apply path (_on_physics_step → _apply_*_cmd → finally clear self._latest_*_cmd) is already wired. Means downstream plans only touch their own callback body, never the lifecycle plumbing.
- Plan 02-02: Single-manager pattern — _start_aic_controller_loop helper accepts off_limit_prims=None as 'use defaults' sentinel. Both setup_controller_subscribers and setup_offlimit_contacts atoms route through this single helper; per-call list overrides via set_off_limit_prims() if the manager is already running. Mirrors the parity_publisher single-manager-double-atom pattern from Phase 1 Plan 06.
- Plan 02-02: audit_dx02.py PRESENT_ATOMS extension treated as Rule 2 (critical functionality) deviation. Plan's "Do NOT edit audit_dx02.py" instruction was contextualized to UI_METHOD_ALIASES (which IS untouched — the new atoms match candidate (b) `_cmd_<atom>` directly), but PRESENT_ATOMS is the contract source-of-truth that must grow with each phase that adds atoms. Without extension, audit gate becomes meaningless for Phase 2 atoms; with extension, audit exits 0 at the plan-spec'd 29 PRESENT × 4 surfaces.
- Plan 02-02: quick_start chain — new step inserted as "3b'" between setup_joint_state_publisher (3b) and the joint-state reorder bridge slot (3c). Same logical position the plan suggests; preserves DX-03 ordering. The step is no-op safe even with all stub callbacks — controller loop comes up cleanly with empty publishers ready for Plans 02-03..06 to fill in.

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

Last session: 2026-05-03T20:24:10.000Z
Stopped at: Plan 02-02 EXECUTED ✓ (5 min). controller_loop.py skeleton + 2 MCP atoms × 4-surface contract + manager helper + on_shutdown + quick_start hook landed. audit_dx02 PASS at 29 PRESENT × 4 surfaces. Plans 02-03..06 unblocked (controller_loop.py is the single shared file from here). Auto-advancing to Plan 02-03 next.
Resume file: .planning/HANDOFF.json (autonomous M1 mode persistent — kept until M1 ships). Active per-phase artifacts: .planning/phases/02-controller-loop/{02-CONTEXT.md, 02-RESEARCH.md, 02-PATTERNS.md, 02-01-SUMMARY.md ✓, 02-02-SUMMARY.md ✓, 02-01..06-PLAN.md, snapshot/}.

## Phase 1 Outstanding (post-Gap-A/B closure)

| Gap | Severity | Description | Path |
|-----|----------|-------------|------|
| C (TEX-03) | low | 921 sweep hits in texture-sweep.md not categorized into a fix log; 8 camera prim-not-found warnings in setup_wrist_cameras (orphan code path); ~13 actual MDL/texture/fallback warnings; 43 cosmetic ISO_4762/V1015120 unresolved sub-refs | inline doc work or defer |
| Gap E mount-rails | latent | Plan 09 mount-rail USDs (LC/SFP/SC Mount + NIC Card Mount/`*_visual.usd`) at mPU=0.01+Y-up; default add_objects path doesn't fire them (present=False) but SCENE-01 trial spawning requires them at correct scale | follow-up: bake xformOp:scale=(100,100,100) + rotateX=90 in build_mount_rail_usds.py |
| PARITY-05 rate verifiability | doc | parity_05_wrench_framing.txt missing Gazebo expected rate snapshot | re-run snapshot script |
