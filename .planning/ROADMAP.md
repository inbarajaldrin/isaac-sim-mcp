# Roadmap: aic-dt — AIC Cable-Insertion Digital Twin (Milestone 1)

## Overview

Milestone 1 (Platform Transfer) takes the existing `aic-dt` extension scaffold from a placeholder with `_sim`/`_real` topic discrimination to a Gazebo-faithful Isaac Sim digital twin where the unmodified `aic_controller` + `aic_engine` + `CheatCode` stack from `~/Documents/aic` runs against `sample_config.yaml` trials and produces the same per-trial pass/fail outcomes Gazebo does. The path is layered: first we get the same robot, the same assets, the same passive sensor topics (Phase 1); then we close the controller loop with command subscriptions, force/contact publishers, and parametric task-board spawn (Phase 2); then we tackle the highest-risk piece — cable physics — and publish the ground-truth object frames CheatCode reads (Phase 3); finally we wire up the trial loader, ground_truth flag, and end-to-end verification under the live `aic_engine` and ship the docs (Phase 4). Topic parity is the architectural law every phase must respect — no `_sim`/`_real` suffixes, no bridges, exact Gazebo topic names everywhere.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3, 4): Planned milestone work
- Decimal phases (e.g., 2.1): Urgent insertions (marked with INSERTED)

- [ ] **Phase 1: Foundation Parity** - Same UR5e + Robotiq Hand-E + cameras + assets + textures Gazebo loads, same passive ROS topics (`/joint_states`, `/tf`, `/tf_static`), `/fts_broadcaster/wrench` fully matched to live, parametric task-board spawn with component-delta atoms + pose params, full cross-phase parity audit table, atomic+clubbed contract enforced, `_sim`/`_real` removed
- [x] **Phase 2: Controller Loop** - Command-side topics close the loop with `aic_controller`; off-limit contacts published; subscribe + state publishers wired _(closed 2026-05-03; 4/4 must-haves implemented in code; runtime smoke gated on next session's quick_start; see .planning/phases/02-controller-loop/02-SUMMARY.md)_
- [ ] **Phase 3: Cable Physics & Ground-Truth Pose** - Cable physics strategy chosen (research-driven via `nvidia-suite-docs`) and implemented for both cable variants with gripper attach; object TF frames CheatCode reads published into `/tf` and `/scoring/tf`; insertion event fires
- [ ] **Phase 4: Trial Loader & End-to-End Verification** - Trial YAML loader (MCP atom + UI button) + `ground_truth` flag; `aic_engine` runs every `sample_config.yaml` trial under CheatCode against Isaac Sim with parity-report side-by-side outcomes; Quick Start refactored, CLAUDE.md and README shipped

## Phase Details

### Phase 1: Foundation Parity
**Goal**: Isaac Sim loads the exact same UR5e + Robotiq Hand-E + 3 wrist cameras + task-board + AIC enclosure assets the AIC repo's Gazebo bringup loads, with no missing/broken textures, and publishes the same passive sensor topics (`/joint_states`, `/tf`, `/tf_static`, `/fts_broadcaster/wrench`) Gazebo publishes — using the same joint names, frame names, frame hierarchy, message types, and rates. The task-board spawn accepts the same component-delta parameters `spawn_task_board.launch.py` does (mount rails 0/1, sc/sfp/lc port presence + translation + RPY, NIC rails 0-3), and robot/board/cable poses are configurable via the same parameter names Gazebo uses (`robot_x/y/z/roll/pitch/yaw`, `task_board_*`, `cable_*`). The atomic+clubbed contract (every new capability = one `MCP_TOOL_REGISTRY` entry + matching `_cmd_<name>` handler + one UI button) is preserved across all new atoms. A full cross-phase parity audit table lives in `exts/aic-dt/docs/topic-parity-reference.md`. All `_sim`/`_real` topic naming from the placeholder scaffold is removed.

**Pulled forward from Phase 2/3** (per CLAUDE.md "Phase scope is by surface, not capability" rule — these requirements share `extension.py` + `AIC_OBJECTS` + asset tree surfaces with the original Phase 1 work, so combining is cheaper than re-entering): SCENE-01, SCENE-04, PARITY-05, PARITY-12, DX-02.

**Depends on**: Nothing (first phase)
**Requirements**: PARITY-01, PARITY-02, PARITY-03, PARITY-04, PARITY-05, PARITY-12, TEX-01, TEX-02, TEX-03, SCENE-01, SCENE-04, DX-01, DX-02
**Canonical refs**: `~/.claude/skills/nvidia-suite-docs/SKILL.md` (Isaac Sim 5.0 / OmniGraph / OpenUSD / `isaacsim.ros2.bridge` live docs — for both research and execution-time recovery), `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` (extension lifecycle, cache discipline, MCP socket protocol, project-specific gotchas), `~/Documents/aic/aic_bringup/launch/spawn_task_board.launch.py` (parameter set source for SCENE-01), `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` (robot definition with Robotiq Hand-E gripper).
**Success Criteria** (what must be TRUE):
  1. After running the (refactored) Quick Start, `ros2 topic list` shows `/joint_states`, `/tf`, `/tf_static`, `/fts_broadcaster/wrench` and no topic anywhere in the extension carries a `_sim` or `_real` suffix
  2. `ros2 topic echo /joint_states --once` returns the same joint names in the same ordering Gazebo's `/joint_states` publishes (live snapshot: 7 joints alphabetical, including `gripper/left_finger_joint`)
  3. `ros2 run tf2_tools view_frames` against Isaac Sim produces a TF tree whose robot + gripper + camera frames match Gazebo's reference TF tree (same names, same parent-child links)
  4. Loading the full M1 scene in Isaac Sim's viewport shows zero pink/black/missing-texture materials and zero broken-MDL warnings in the console; `exts/aic-dt/docs/texture-sweep.md` contains a findings + fix log enumerating every asset touched
  5. Grepping the extension source for `objects_poses_sim`, `sync_real_poses`, `_sim`, and `_real` returns no production-topic matches (camera placeholder topics renamed or removed)
  6. `ros2 topic hz /fts_broadcaster/wrench` shows a steady stream of `WrenchStamped` messages matching Gazebo's frame_id and rate (PARITY-05)
  7. Calling the per-component spawn atoms with a `sample_config.yaml` `task_board` block (mount rails + sc/sfp/lc/nic occupancy + per-entity translation/RPY + robot/board pose) produces a viewport scene equivalent to the same parameters spawned by `spawn_task_board.launch.py` in Gazebo (SCENE-01, SCENE-04)
  8. `exts/aic-dt/docs/topic-parity-reference.md` contains a complete cross-phase parity audit table mapping every Gazebo topic to its Isaac Sim equivalent with phase status (implemented / Phase-N-deferred / not-applicable) and proof-of-publish per implemented row (PARITY-12)
  9. Every new MCP capability lands as one `MCP_TOOL_REGISTRY` entry + matching `_cmd_<name>` handler + one UI button — `MCP_TOOL_REGISTRY` is the only source of tool metadata (DX-02)
**Plans**: 9 plans + 5 gap-closure plans (verifier loop, 3 waves)
Plans:
- [x] 01-01-PLAN.md — Snapshot infrastructure: live aic_eval Docker capture script + topic-parity-reference.md (D-01, D-14) + cross-phase audit table (PARITY-12)
- [x] 01-02-PLAN.md — Asset vendoring: capitalized AIC layout with sibling textures/ folders (D-05); retire snake_case objects/
- [x] 01-03-PLAN.md — Doc-only gripper-name correction (Robotiq Hand-E) across .planning/, CLAUDE.md, exts/aic-dt/docs/README.md, exts/aic-dt/docs/CHANGELOG.md (D-03)
- [x] 01-04-PLAN.md — extension.py renames (33 _sim/_real cells) + gripper-name correction (Robotiq Hand-E) + prim-path bug fix + AIC_OBJECTS update + delete setup_pose_publisher / sync_real_poses atoms (DX-02 deletion-side) + PARITY-05 wrench full match
- [x] 01-05-PLAN.md — Pre-graph probes: USD prim probe → verdict PER-FRAME-RAW-OVERRIDE (17 frame_id overrides for Plan 06) for PARITY-04; aic_controller subscriber probe (extended to aic_adapter) → verdict NAME-INDEXED, NO-WRAPPER-NEEDED for PARITY-03; both conditional fixes skipped per probe outputs
- [x] 01-06-PLAN.md — Two new MCP atoms: setup_tf_publisher (/tf + /tf_static) + setup_joint_state_publisher (/joint_states) per D-10/D-11; targetPrim binds at /World/UR5e/aic_unified_robot/root_joint per RESEARCH Pattern 1; DX-02 4-surface contract enforced; 2 architectural deferrals (TF frame_id slash overrides, JointState gripper finger rename) documented inline + Plan 08 verify-harness "expect to fail" contract -- follow-up plan addresses via Raw publishers + IsaacReadOdometry/OgnGetPrimWorldPose source nodes + JointState wrapper
- [x] 01-07-PLAN.md — Verify harness: diff_tf_tree.py (D-08; 71 lines, regex-only) + sweep_textures.py (D-07/TEX-01/02/03; D-07 baseline + 4 augmented Isaac-Sim asset-failure patterns) + verify_phase_1.sh (D-15; 441 lines, 10-step hybrid runtime gate) + texture-sweep.md (TEX-03 scaffold). Smoke-tested: diff PASS 31/30, sweep 267 hits caught sc_port_visual broken sub-refs, verify bash -n clean.
- [x] 01-08-PLAN.md — quick_start refactor per D-12 + Phase 1 CHANGELOG entry + DX-02 final-audit table (presents 9 new atoms × 4 surfaces, 2 deleted atoms × 0 surfaces)
- [x] 01-09-PLAN.md — SCENE-01 + SCENE-04: 7 per-component spawn atoms (spawn_task_board_base / spawn_<lc|sfp|sc>_mount_rail / spawn_sc_port / spawn_nic_card_mount / spawn_nic_card) mirroring spawn_task_board.launch.py parameter surface; robot/board/cable pose params with Gazebo defaults; LC/SFP/SC Mount asset vendoring from aic_assets/models/; backwards-compatible add_objects clubbing
- [ ] 01-G01-PLAN.md — Gap E diagnosis: pxr-based per-prim worldbox capture + diff (pre-Plan-02 baseline @ 2be9e0c vs current HEAD), metersPerUnit audit, spawn-atom transform comparison; produces visual-regression-diagnosis.md with verdict token (METERSPERUNIT_DRIFT / SPAWN_ATOM_TRANSFORM_DRIFT / GLB_USD_SCALE_BAKED_WRONG / ADD_REFERENCE_UNIT_DRIFT / COMPOUND / INCONCLUSIVE) + recommended fix
- [ ] 01-G02-PLAN.md — Gap E fix: apply verdict-specific fix from G01 (USD edit / spawn-atom defaults / build_mount_rail_usds.py re-author / explicit xformOp:scale); human-verify viewport renders correctly; CHANGELOG + REQUIREMENTS sync
- [ ] 01-G03-PLAN.md — Gap B closure (PARITY-04 deferral): wire 17 per-frame ROS2PublishRawTransformTree publishers (16 underscore→slash overrides + 1 synthesized aic_world static edge) backed by OgnGetPrimWorldPose source nodes; preserves 14 MATCH frames via existing default-mapping publishers; closes diff_tf_tree.py to zero diff
- [ ] 01-G04-PLAN.md — Gap A closure (PARITY-03 deferral): probe_articulation_joints.py picks strategy (ARTICULATION_RESTRUCTURE / SUPPLEMENTARY_PUBLISHER_SUBGRAPH / WRAPPER_REPUBLISHER / HEADER_FRAME_ID_ONLY); applies fix so /joint_states publishes 7 names with gripper/left_finger_joint (slash) and frame_id=base_link
- [ ] 01-G05-PLAN.md — Gap C closure (TEX-01/02/03): verify_phase_1.sh Step 8 path-with-spaces hot-fix; sweep_textures.py noise filter (4 EXCLUDE_PATTERNS); setup_wrist_cameras prim path correction (camera-prim-not-found warnings); texture-sweep.md verdict section with row classification
**UI hint**: yes

### Phase 2: Controller Loop
**Goal**: The unmodified `aic_controller` (impedance controller plugin) successfully drives the UR5e in Isaac Sim — Isaac Sim subscribes to `/aic_controller/joint_commands` and `/aic_controller/pose_commands` and applies them, publishes `/aic_controller/controller_state` for tare/bookkeeping, and publishes `/aic/gazebo/contacts/off_limit` for off-limit contact events. (Parametric task-board spawn, pose parameters, wrench publisher, and atomic+clubbed contract enforcement were pulled forward to Phase 1 per the surface-adjacency rule.)
**Depends on**: Phase 1
**Requirements**: PARITY-06, PARITY-09, PARITY-10, PARITY-11
**Success Criteria** (what must be TRUE):
  1. Launching `aic_controller` against Isaac Sim and publishing a `JointMotionUpdate` to `/aic_controller/joint_commands` causes the UR5e in the viewport to move to the commanded configuration; same test against `/aic_controller/pose_commands` moves the end-effector
  2. `ros2 topic hz /fts_broadcaster/wrench` shows a steady stream of `WrenchStamped` messages with the correct end-effector `frame_id`, and intentional contacts visibly affect the force readings
  3. Driving the gripper into a flagged off-limit collision body produces a non-empty `/aic/gazebo/contacts/off_limit` message of type `ros_gz_interfaces/Contacts` (exact topic name preserved including the literal "gazebo" segment)
  4. Calling the parametric task-board spawn MCP tool with a `sample_config.yaml` `trial_1` `task_board` block (mount rails + sc/sfp/lc/nic occupancy + per-entity translation/RPY) produces a viewport scene that matches the same parameters spawned by `spawn_task_board.launch.py` in Gazebo
  5. Each new capability ships as one MCP tool entry in `MCP_TOOL_REGISTRY` + matching `_cmd_<name>` handler + one UI button — `MCP_TOOL_REGISTRY` is the only source of tool metadata
**Plans**: 6 plans (sequential waves 1→6 — controller_loop.py is a single shared file across plans 02-03..06, so they serialize)
Plans:
- [x] 02-01-PLAN.md — Workspace rebuild for Python 3.11 ABI (D-05 fix: vendor + build aic_control_interfaces + ros_gz_interfaces) + Open Q1 settlement (snapshot live aic_eval controller topic names + off-limit prim mapping)
- [x] 02-02-PLAN.md — controller_loop.py skeleton (AicControllerLoop class, lifecycle, stub callbacks, omni.physx physics-step subscription) + extension.py 2 MCP atoms × 4-surface contract (`setup_controller_subscribers` + `setup_offlimit_contacts`) + `_start_aic_controller_loop` manager helper + on_shutdown teardown + quick_start chain insertion
- [x] 02-03-PLAN.md — PARITY-09: `_on_joint_cmd` validation + `_apply_joint_cmd` with name-keyed parser (D-09), per-joint stiffness/damping via `Articulation.set_gains`, positions/feedforward via `Articulation.apply_action(ArticulationActions)` per D-06; gripper/left_finger_joint silently no-op'd
- [x] 02-04-PLAN.md — PARITY-10: `_setup_kinematics` (LulaKinematicsSolver + ArticulationKinematicsSolver from bundled UR5e config; `end_effector_frame_name="tool0"` per Pitfall 2) + `_on_pose_cmd` validation + `_apply_pose_cmd` with ROS↔Lula quaternion order conversion (Pitfall 4); D-06 Cartesian impedance fields logged + ignored; Pitfall 2 Option A static SE(3) offset cached at _setup_kinematics + applied at _apply_pose_cmd ingress for gripper/tcp frame_id
- [x] 02-05-PLAN.md — PARITY-11: `_publish_controller_state` with FK via `compute_end_effector_pose`, numerical-diff tcp_velocity over 3-sample ring buffer, reference echoes from Plans 02-03/02-04 bookkeeping vars, zero `fts_tare_offset` with `frame_id=ati/tool_link` per D-07
- [x] 02-06-PLAN.md — PARITY-06: `_setup_contact_subscription` (omni.physx contact-report per robot-collision-forensics skill; PhysxContactReportAPI threshold=0.0 per Pitfall 8; RigidBodyAPI guard per Pitfall 7) + `_on_contact_event` (physics-thread callback; CONTACT_FOUND only) + `_publish_offlimit_contacts` to `/aic/gazebo/contacts/off_limit` (ros_gz_interfaces/Contacts) + smoke_test_aic_controller.py D-12 7-step verifier + verify_phase_2.sh harness + REQUIREMENTS.md flips + 02-SUMMARY.md (phase closure)
**UI hint**: yes

### Phase 3: Cable Physics & Ground-Truth Pose
**Goal**: A cable physics strategy is researched (using the `nvidia-suite-docs` skill) across deformable / articulated chain / rigid-plug-with-visual-hybrid options, justified, and implemented such that both `sfp_sc_cable` and `sfp_sc_cable_reversed` variants spawn correctly with `attach_cable_to_gripper:=true` semantics (plug body attached to the Robotiq Hand-E at the same offset Gazebo uses, with `gripper_initial_pos` matching cable type — 0.0073 for sfp_sc_cable, 0.00655 default). The object TF frames CheatCode looks up — `{cable_name}/{plug_name}_link`, port frames against `base_link`, etc. — are published into `/tf` (and cable link poses into `/scoring/tf`) when ground-truth mode is on, and the insertion event fires on `/scoring/insertion_event` when a cable plug enters its target port. The Quick Start grows to club the new atoms cleanly.
**Depends on**: Phase 2
**Requirements**: SCENE-02, SCENE-03, SCENE-05, SCENE-06, PARITY-07, PARITY-08, DX-03
**Success Criteria** (what must be TRUE):
  1. A cable-physics design note in `exts/aic-dt/docs/` records which strategy (deformable / articulated chain / rigid+visual hybrid) was chosen, what was tested, and what the trade-offs were — and the chosen strategy is implemented for both `sfp_sc_cable` and `sfp_sc_cable_reversed`
  2. With `attach_cable_to_gripper:=true`, the plug remains rigidly co-moving with the Robotiq Hand-E gripper (verified by visual inspection and by `ros2 run tf2_ros tf2_echo base_link <cable>/<plug>_link` showing pose changes that track gripper motion)
  3. After a Quick Start with ground-truth mode on, `ros2 run tf2_ros tf2_echo base_link <cable>/<plug>_link` and equivalent calls for port frames return valid transforms — the same `lookup_transform` calls CheatCode makes succeed against Isaac Sim with no code change in the AIC repo
  4. Driving the attached plug into the matching port produces a `std_msgs/String` message on `/scoring/insertion_event`, and `/scoring/tf` carries cable link poses throughout the run
  5. `exts/aic-dt/docs/topic-parity.md` lists every Gazebo topic from `aic_engine`'s `sample_config.yaml` `scoring.topics` block alongside the Isaac Sim equivalent with proof-of-publish (`ros2 topic info` output or equivalent), surfacing zero unintentional gaps; the refactored `quick_start` clubs the M1 atoms in the documented common-path order
**Plans**: TBD
**UI hint**: yes

### Phase 4: Trial Loader & End-to-End Verification
**Goal**: An MCP tool + UI button reads a single trial entry from `aic_engine`'s `sample_config.yaml` (or equivalent format) and spawns the matching Isaac Sim scene — board pose, rail occupancy, port poses, cable pose, attach state. A `ground_truth` flag mirrors Gazebo's behavior (object TF on/off). The unmodified `aic_engine` runs against Isaac Sim using the same launch invocation pattern it uses against Gazebo, and every trial in `~/Documents/aic/aic_engine/config/sample_config.yaml` passes end-to-end under `aic_example_policies/CheatCode.py` against Isaac Sim with the same per-trial pass/fail outcomes Gazebo produces. An automated, re-runnable parity-report script captures both sims' per-trial outcomes side-by-side. The repo gets a `CLAUDE.md` on-ramp and the extension README + changelog reflect the AIC scope.
**Depends on**: Phase 3
**Requirements**: TRIAL-01, TRIAL-02, TRIAL-03, TRIAL-04, TRIAL-05, DX-04, DX-05
**Success Criteria** (what must be TRUE):
  1. Clicking the per-trial UI button (or invoking the MCP tool) for any trial in `sample_config.yaml` spawns an Isaac Sim scene whose board pose, rail occupancy, port poses, cable pose, and attach state match the YAML — verified by visual inspection and TF lookups
  2. With `ground_truth:=true`, object TF frames are published; with `ground_truth:=false`, only the robot's frames are published and CheatCode's `lookup_transform` calls would (correctly) fail — preparing the M2 swap surface
  3. Launching the unmodified `aic_engine` with `--config sample_config.yaml` and the unmodified `CheatCode.py` policy against Isaac Sim runs every trial end-to-end and produces the same per-trial pass/fail outcomes Gazebo produces (insertion_event fires for passes, no off_limit contacts on passes), with no Isaac-Sim-specific patches applied to anything in `~/Documents/aic`
  4. Running the parity-report script regenerates a side-by-side trial-outcome table comparing Isaac Sim vs Gazebo per-trial outcomes — re-runnable, deterministic structure, and the canonical "M1 done?" check
  5. `CLAUDE.md` at the repo root tells a fresh Claude session how to launch Isaac Sim with the `aic-dt` extension, what env vars matter (`MCP_SERVER_PORT=8768`, `MCP_CLIENT_OUTPUT_DIR`), and points to `~/Documents/aic` as the ROS-side / controller / policies / engine repo; `exts/aic-dt/docs/README.md` describes the AIC scope (no stale "ur5e-dt" framing) and the changelog records M1
**Plans**: 5 plans (sequential waves 1→5 — extension.py + scoring_publishers.py + audit_dx02.py shared across plans, so they serialize)
Plans:
- [x] 04-01-PLAN.md — Pre-flight risk de-risking: A2 (kilted↔humble RMW interop) + A4 (_PORT_LINK_PATHS vs live spawn paths) probes — 2026-05-05 (A2 PASS, A4 MISMATCH_NIC_CARD_MOUNT — D-13 set_port_link_paths setter is now required scope in Plan 04-03)
- [x] 04-02-PLAN.md — load_trial MCP atom + ground_truth flag on quick_start (TRIAL-01, TRIAL-02) — 2026-05-05 (4-surface DX-02 contract held; audit_dx02.py PRESENT_ATOMS=30; offline structural test PASS for all 3 trials; live-fire deferred to 04-03)
- [x] 04-03-PLAN.md — Derived my-eval-isaac:v1 Docker image + run_aic_engine_against_isaac_sim.sh wrapper + trial_1 dry-run (TRIAL-03 ✓, PARITY-07 ~) — 2026-05-05 (all deliverables landed: Docker image + wrapper + D-13 1-surface setter + recursive contact-tag walk; trial_1 dry-run BLOCKED at engine clock-wait on kilted↔humble fastrtps type-hash incompatibility for stock /clock messages — see .planning/HANDOFF.json blockers + .planning/.user_pause)
- [ ] 04-04-PLAN.md — parity_report.py + first M1 ship-gate exercise (TRIAL-04, TRIAL-05)
- [ ] 04-05-PLAN.md — DX-05 README + CHANGELOG + REQUIREMENTS flips + phase-level closure paperwork (DX-05; M1 ship gate)
**UI hint**: yes

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Foundation Parity | 9/9 | Code complete — pending verifier loop | - |
| 2. Controller Loop                       | 5/6   | Executing — Plans 02-01..05 done; Plan 02-06 pending | - |
| 3. Cable Physics & Ground-Truth Pose | 6/6 | Closed (2026-05-05) | 2026-05-05 |
| 4. Trial Loader & End-to-End Verification | 3/5 | BLOCKED — Plans 04-01..03 closed; 04-03 trial_1 dry-run reached HARD BLOCKER (kilted↔humble fastrtps type-hash incompatibility for stock /clock); 04-04..05 await user mitigation decision (see HANDOFF.json blockers) | - |

---
*Roadmap created: 2026-05-01 (Milestone 1 — Platform Transfer)*
*Granularity: coarse — 4 phases for 31 v1 requirements*
*Coverage: 31/31 v1 requirements mapped*
