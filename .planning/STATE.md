---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: executing
stopped_at: "Plan 04-01 EXECUTED ✓ (~5 min wall). Pre-flight Wave 1 risk de-risking complete: A2 (kilted↔humble RMW interop) PASS — sensor_msgs/JointState + custom aic_control_interfaces/JointMotionUpdate both decode against humble subscriber on fastrtps + domain 7. A4 (_PORT_LINK_PATHS coverage) MISMATCH_NIC_CARD_MOUNT — Phase 3's hardcoded list (sc_port_1, sc_port_2, nic_card under /World/TaskBoard) doesn't match live spawn output (SCPort_0/1, NICCardMount_0..4, NICCard) — Plan 04-03 must add a public set_port_link_paths(paths) setter on AicScoringPublishers (1-surface addition, NOT a 4-surface DX-02 atom add) and have load_trial compute paths from spawn-call sites. 5 deliverables shipped: 2 re-runnable probe scripts under exts/aic-dt/scripts/ + 2 audit-trail .txt reports + 04-01-SUMMARY.md. 3 atomic commits (ff6ca28, 400ac01, pending). Cache snapshotted (DerivedDataCache.bak.1777983174). Auto-advancing to Plan 04-02 (load_trial atom + ground_truth flag) next."
last_updated: "2026-05-05T13:30:00.000Z"
last_activity: 2026-05-05 -- Plan 04-01 closed (A2 PASS, A4 MISMATCH_NIC_CARD_MOUNT); D-13 setter forced into 04-03 baseline scope
progress:
  total_phases: 4
  completed_phases: 1
  total_plans: 31
  completed_plans: 19
  percent: 61
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-01)

**Core value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.
**Current focus:** Phase 04 — trial-loader

## Current Position

Phase: 04 (trial-loader) — EXECUTING
Plan: 2 of 5 (04-01 closed; 04-02 next)
Status: Executing Phase 04
Last activity: 2026-05-05 -- Plan 04-01 closed (A2 PASS, A4 MISMATCH_NIC_CARD_MOUNT)

Progress: [████████████░] M1 ~82% (Phases 1+2+3 closed; Phase 4 plan 1/5 closed via Wave 1 pre-flight probes — A2 PASS unblocks engine-container path, A4 MISMATCH forces D-13 setter into 04-03)

## Phase 2 Plans

| Plan | Wave | Objective | Requirements |
|------|------|-----------|--------------|
| 02-01 ✓ | 1 | Custom message workspace rebuild (Python 3.11 humble) + topic-name discovery probe + off-limit prim mapping | (infra for PARITY-06/09/10/11) |
| 02-02 ✓ | 2 | controller_loop.py skeleton + 2 MCP atoms (8 surface additions per DX-02) + manager helper + on_shutdown + quick_start hook | (skeleton infra) |
| 02-03 ✓ | 3 | PARITY-09: joint_commands subscriber + name-keyed parser + per-joint set_gains + apply_action | PARITY-09 |
| 02-04 ✓ | 4 | PARITY-10: pose_commands subscriber + Lula IK + Pitfall #2 Option A static offset for gripper/tcp | PARITY-10 |
| 02-05 ✓ | 5 | PARITY-11: ControllerState publisher (FK + numerical-diff velocity + reference echoes + zero tare) | PARITY-11 |
| 02-06 ✓ | 6 | PARITY-06: omni.physx contact-report + Contacts publish + smoke_test_aic_controller.py + verify_phase_2.sh + 02-SUMMARY + flag flips | PARITY-06 |

## Performance Metrics

**Velocity:**

- Total plans completed: 15
- Average duration: 8.8 min
- Total execution time: 132 min

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| Phase 1 | 9 | 59 min | 6.6 min |
| Phase 2 | 6 | 73 min | 12.2 min |

**Recent Trend:**

- Last 15 plans: 01-01 (7 min), 01-02 (~2 min), 01-03 (3 min), 01-04 (8 min), 01-05 (5 min), 01-06 (6 min), 01-07 (10 min), 01-09 (8 min), 01-08 (10 min), 02-01 (49 min), 02-02 (5 min), 02-03 (2 min), 02-04 (3 min), 02-05 (~2 min), 02-06 (~12 min)
- Trend: 02-06 took ~12 min — the predicted hand-off pattern played out. ~5x longer than the pure-pattern-fill plans (02-03/04/05) because PARITY-06 introduces a genuinely new surface (omni.physx contact-report subscription pipeline, ros_gz_interfaces/Contacts message construction) AND the closure pass writes 5 new artifacts (smoke_test_aic_controller.py 291 LOC, verify_phase_2.sh 132 LOC, 02-06-SUMMARY.md, 02-SUMMARY.md, REQUIREMENTS.md edits). 4 Rule 1 auto-fixes during execution: (1) DEFAULT_OFF_LIMIT_PRIMS canonical replacement vs plan's fictional fallback list; (2) prefix-startswith filter for actor paths (plan's `in` check would silently no-op on descendant collider paths); (3) MODE_POSITION enum value 2 (not plan's literal 1 = MODE_VELOCITY); (4) per-package WS_INSTALL path (plan assumed merged-install layout that doesn't exist on disk). All caught and fixed at write-time; 0 caused harness re-runs. Phase 2 now closed; auto-paused per autonomous-mode locked-in policy. Hand-off to Phase 3 (Cable Physics): different surface entirely (USD-side mass/inertia authoring + ros2 publishers for cable-link TF + scoring topics). Per CLAUDE.md SCENE-05 guidance, Phase 3 is now a smaller in-place USD edit (mass/inertia authoring on cable subtree) since the cable wedge no longer reproduces under current launch path — not the full deformable-vs-articulated research-gated decision the original ROADMAP scoped.

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
- Plan 02-03: TrajectoryGenerationMode enum source-of-truth = the .msg file, NOT the plan body. Plan body declared `MODE_POSITION=1, MODE_VELOCITY=2`; actual `~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/TrajectoryGenerationMode.msg` declares `MODE_VELOCITY=1, MODE_POSITION=2`. Without correction, every position command would route into the velocity-validation branch and fail size check (drop), while velocity commands would match the position branch and either fail differently or pass without proper validation. Rule 1 fix in controller_loop.py with inline source-of-truth comment.
- Plan 02-03: Per-joint size validation gate against `len(msg_joint_names)` not against `N_ARM_JOINTS`. The implementation is more permissive than the plan's draft: the message could legally include the gripper joint (taking len = 7) per aic_adapter::joint_sort_order_; the parser still extracts only the 6 arm DOFs after the GRIPPER_NOOP skip. Robust against future tool-link joints too. Loop's `if name not in ARM_JOINTS` is the actual application-side guard.
- Plan 02-03: Lazy `import numpy` + `from isaacsim.core.utils.types import ArticulationActions` inside `_apply_joint_cmd` (NOT at module top). Preserves Plan 02-02's contract that the class definition is loadable offline (no Kit / no Isaac Sim core imports at module load). Plans 02-04 / 02-05 should follow the same lazy-import pattern.
- Plan 02-03: `set_gains` and `apply_action` wrapped in INDEPENDENT try/except (not a single shared one). Reason: Pitfall 6 — `set_gains` is a silent no-op pre-play, and if it raises (physics_sim_view not ready), we still want positions to apply. Both share `_logged_apply_error` log-once flag (no spam, but each path can fail independently).
- Plan 02-03: Bookkeeping (`_last_reference_joint_state` + `_last_target_mode`) set ONLY after `apply_action` succeeds, not after `set_gains`. ControllerState.last_target_mode should reflect what was actually applied to the articulation, not what the validator accepted. If apply_action raises, bookkeeping stays stale until next successful command — Plan 02-05 publishes the previous target until then, which is correct behavior (controller_state mirrors the most-recently-applied command).
- Plan 02-05: tcp_pose published at tool0 frame (FK direct), NOT post-multiplied by self._tool0_to_tcp_offset_xform. The cached offset is *available* but the tool0 frame matches the rest of the AIC topic surface (e.g. /tcp_pose_broadcaster/pose from Phase 1); aic_controller's Cartesian impedance loop handles the tool0->tcp tip transform on its end. The cached offset stays available for any future plan that needs tcp-frame egress without re-computing.
- Plan 02-05: tcp_velocity.angular and tcp_error[3..5] (rx,ry,rz) deferred to a later phase, set to 0 first-cut. Quaternion finite-difference and axis-angle delta from quaternion difference both have finicky cases (axis-flip, half-revolution wrap-around); aic_controller doesn't consume these for any current trial. Documented inline at the code site so a future planner sees the path forward.
- Plan 02-05: TARGET_MODE_* enum values re-verified against ~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/TargetMode.msg source-of-truth (MODE_UNSPECIFIED=0, MODE_CARTESIAN=1, MODE_JOINT=2) — exact match with controller_loop.py:120-122 constants block. No Plan 02-03-style enum-inversion bug here. Source-of-truth verification is part of the "fill in" task even when the constants block was set up by an earlier plan.
- Plan 02-05: Defensive rosidl `tcp_error` typing — try `msg.tcp_error[i]=v` indexed assignment first (works for `array.array`, the rosidl-generated default for `float64[6]` in humble), fall back to list assignment. Static analysis confirms the indexed path runs first and succeeds; fallback is preserved as defensive scaffolding for non-standard rosidl backends or future ros2 releases. Plan 02-06's smoke test is the first runtime exercise.
- Plan 02-05: `_publish_controller_state` is a pure consumer of bookkeeping state set by sibling _apply_*_cmd methods — added zero new state fields. Pattern: setup-once state lives in __init__/stop reset; per-tick state read by the publisher is owned by whoever populates it. Plan 02-06 will follow the same: contact-event state lives in self._contact_events buffer populated by omni.physx callback, drained by _publish_offlimit_contacts. Single-writer/single-reader simplifies the physics-thread + rclpy-thread boundary.
- Plan 02-05: LOC overage (+128 vs plan's ~70-100 estimate) is pure documentation — extended docstring framing the "tcp_pose at tool0 frame" decision + the two deferred items + the D-07 field policy as a complete reference at the code site. Pattern: when the implementation is ~100 LOC of code, ~30 LOC of docstring is correct ratio for a future reader who lands on the function cold. Don't trim docstrings to hit estimate bands.
- Plan 04-01 (A2): kilted aic_eval ↔ humble Isaac Sim subscriber RMW interop is wire-compatible on `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + `ROS_DOMAIN_ID=7` for both `sensor_msgs/JointState` AND the kilted-built custom `aic_control_interfaces/msg/JointMotionUpdate`. Plan 04-03 wrapper uses stock `aic_eval:latest` image (digest `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433`); no derived `my-eval-isaac:v1` humble-base image needed. Saved ~30 min of scope from A6's worst-case escalation path.
- Plan 04-01 (A2 workaround): `env_isaaclab/install/bin/ros2` CLI is broken (Python 3.10 shebang vs 3.11 venv + missing netifaces). Bypass: write inline rclpy harness inside the bash probe — same wire path as `ros2 topic echo`, just bypassing the CLI entry-point bug. Logged as future-maintenance ticket; doesn't block any Phase 4 plan.
- Plan 04-01 (A4): scoring_publishers `_PORT_LINK_PATHS` hardcoded list (`/World/TaskBoard/sc_port_1/sc_port_2/nic_card`) DOES NOT match live spawn output. Live tree under `/World/TaskBoard/` after Phase 4 spawn cycle: `SCPort_0`, `SCPort_1`, `NICCardMount_0..4`, `NICCard` (PascalCase + index). Phase 1's legacy `add_objects` ALSO populates `/World/Objects/sc_port_1/sc_port_2/nic_card` (lowercase + underscore). Both namespaces co-exist; neither matches the hardcoded list. Phase 3's contact-report subscription would tag zero of the hardcoded prims and `/scoring/insertion_event` would never fire — direct path to TRIAL-04 mismatch.
- Plan 04-01 (D-13 forced active in 04-03): the YAGNI "maybe needed" `set_port_link_paths` setter is now confirmed required scope. Plan 04-03 must add a public `set_port_link_paths(paths: list[str])` method on `AicScoringPublishers` (1-surface addition — public method on publisher class, called from `_start_aic_scoring_publishers(port_link_paths=...)`). `load_trial` computes paths from spawn-call sites: `/World/TaskBoard/SCPort_{index}` for sc_port, `/World/TaskBoard/NICCardMount_{index}` for nic_card_mount, `/World/TaskBoard/NICCard` for nic_card. Module-level `_PORT_LINK_PATHS` constant stays as default for cable-only test scenes.
- Plan 04-01 (probe pattern): live-stage probes via the existing `execute_python_code` MCP atom + `stage.Traverse` are the cheapest way to verify on-disk USD authoring vs published-prim-path expectations. ~30s wall-clock for the round-trip. Use this pattern again in Plan 04-04 dry-run to verify the set_port_link_paths setter wires to actual prims.

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

Last session: 2026-05-05T13:30:00.000Z
Stopped at: Plan 04-01 EXECUTED ✓ (~5 min wall). Pre-flight Wave 1 risk de-risking: A2 PASS (kilted↔humble RMW interop) + A4 MISMATCH_NIC_CARD_MOUNT (_PORT_LINK_PATHS coverage). 5 deliverables shipped (2 probe scripts + 2 audit reports + SUMMARY); 3 atomic commits. D-13 set_port_link_paths setter is now confirmed required scope in Plan 04-03 (1-surface addition on AicScoringPublishers, NOT a 4-surface DX-02 atom). Auto-advancing to Plan 04-02 (load_trial atom + ground_truth flag) next.
Resume file: .planning/HANDOFF.json (autonomous M1 mode persistent — kept until M1 ships). Active per-phase artifacts: .planning/phases/04-trial-loader/{04-CONTEXT.md, 04-RESEARCH.md, 04-DISCUSSION-LOG.md, 04-01-PLAN.md, 04-01-SUMMARY.md ✓, 04-02..05-PLAN.md, kilted_humble_interop.txt, taskboard_prim_paths.txt}.

## Phase 1 Outstanding (post-Gap-A/B closure)

| Gap | Severity | Description | Path |
|-----|----------|-------------|------|
| C (TEX-03) | low | 921 sweep hits in texture-sweep.md not categorized into a fix log; 8 camera prim-not-found warnings in setup_wrist_cameras (orphan code path); ~13 actual MDL/texture/fallback warnings; 43 cosmetic ISO_4762/V1015120 unresolved sub-refs | inline doc work or defer |
| Gap E mount-rails | latent | Plan 09 mount-rail USDs (LC/SFP/SC Mount + NIC Card Mount/`*_visual.usd`) at mPU=0.01+Y-up; default add_objects path doesn't fire them (present=False) but SCENE-01 trial spawning requires them at correct scale | follow-up: bake xformOp:scale=(100,100,100) + rotateX=90 in build_mount_rail_usds.py |
| PARITY-05 rate verifiability | doc | parity_05_wrench_framing.txt missing Gazebo expected rate snapshot | re-run snapshot script |
