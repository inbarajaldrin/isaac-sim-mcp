# Requirements: aic-dt — AIC Cable-Insertion Digital Twin

**Defined:** 2026-05-01
**Core Value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.

## v1 Requirements (Milestone 1 — Platform Transfer)

### Asset & Topic Parity

- [x] **PARITY-01**: Isaac Sim loads the AIC repo's `aic_description` xacro/URDF for the UR5e + Robotiq Hand-E + 3 wrist cameras (no divergent kinematics or geometry from Gazebo) _(closed: doc + code-side gripper-name corrected in Plans 01-03 / 01-04; load path = aic_unified_robot.usd (xacro-derived canonical AIC representation, not runtime xacro expansion); kinematic/geometric parity verified live by smoke_test_aic_parity.py 13/13 against /opt/ros/humble Python 3.10 rclpy — 7 alphabetical joints match aic_adapter::joint_sort_order_ exactly, tf2 lookup_transform succeeds for `base_link`→`gripper/tcp` (CheatCode L105) + `tool0` distance plausible (0.865m). The "no divergent kinematics or geometry from Gazebo" bar is the testable contract; the smoke test is the verifier)_
- [x] **PARITY-02**: Isaac Sim loads the AIC repo's `aic_assets` meshes for the task board, ports (sc / sfp / lc / nic), mount rails, cables, and the AIC enclosure (no divergent geometry) _(closed: disk-layer vendor complete in Plan 01-02 — capitalized AIC tree byte-identical under exts/aic-dt/assets/assets/ (21 files: 9 USDs + 12 textures, md5sum byte-identical to upstream); AIC_OBJECTS code path repointed at vendored capitalized layout in Plan 01-04; LC/SFP/SC Mount asset folders + cable assets vendored in Plan 01-09; scene-load verified live in working session — 387 prims after quick_start, no missing-mesh errors. Mount-rail mPU/upAxis bake (Gap E) is latent for SCENE-01 trial spawn, tracked in 01-SUMMARY Deferred Items)_
- [x] **PARITY-03**: Isaac Sim publishes `/joint_states` with the same joint name set + ordering Gazebo publishes _(closed inline 2026-05-03 commit 079afd8: rclpy-based AicParityPublishers replaces OGN ROS2PublishJointState — 7 alphabetical names matching aic_adapter::joint_sort_order_ EXACTLY (`elbow_joint`, `gripper/left_finger_joint`, `shoulder_lift_joint`, `shoulder_pan_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`), `frame_id=base_link`, gripper FixedJoint pinned to 0.0; smoke test `exts/aic-dt/scripts/smoke_test_aic_parity.py` verifies via /opt/ros/humble Python 3.10 rclpy that aic_adapter-style consumers see exactly the right name set + ordering + frame_id with 259 messages in 3s; the OGN-spec deferral noted in Plan 06 PLAN.md L302 is now moot — bypassed entirely)_
- [x] **PARITY-04**: Isaac Sim publishes `/tf` and `/tf_static` containing the same robot + gripper + camera frames Gazebo publishes (with same parent-child hierarchy and frame names) _(closed inline 2026-05-03 commit 079afd8: rclpy-based AicParityPublishers replaces OGN ROS2PublishTransformTree — 30 of 31 AIC frames mapped from USD prim leafs to slashed AIC names (`gripper/hande_base_link`, `cam_mount/cam_mount_link`, `ati/tool_link`, `*_camera/*_link`/`sensor_link`/`optical`), 1 synthesized `world→aic_world` identity transform; 8 dynamic edges (6 arm + 2 gripper finger) on `/tf` at ~70 Hz, 22 static edges on `/tf_static` (TRANSIENT_LOCAL); smoke test verifies tf2_ros::Buffer.lookup_transform succeeds for `base_link`→`gripper/tcp` (CheatCode L105), `base_link`→`tool0`, `world`→`aic_world`, `world`→`tool0` (full chain) — 6 TF tests pass + tool0 distance from base_link plausible (0.865m); the OGN-spec deferral noted in Plan 06 PLAN.md L51/302 is now moot — bypassed entirely)_
- [x] **PARITY-05**: Isaac Sim publishes `/fts_broadcaster/wrench` (`geometry_msgs/WrenchStamped`) for the UR5e end-effector force/torque, matching Gazebo's topic name and frame_id _(closed: static reconciliation complete in Plan 01-04 — topic renamed to /fts_broadcaster/wrench, message type WrenchStamped preserved, header.frame_id corrected tool0→ati/tool_link from URDF (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` line 242 AtiForceTorqueSensor frame_id); audit trail at parity_05_wrench_framing.txt with the standardized fields. Note: live wrench topic name in current aic_eval container is `/force_torque_sensor_broadcaster/wrench` (per CLAUDE.md "Important discovery 2026-05-01") — Phase 2 controller-loop work will reconcile against the live topic surface; Gap C rate-snapshot deferred per 01-SUMMARY)_
- [ ] **PARITY-06**: Isaac Sim publishes `/aic/gazebo/contacts/off_limit` (`ros_gz_interfaces/Contacts`) for off-limit-item contact events — exact topic name kept even though "gazebo" is in it (zero-rename rule)
- [ ] **PARITY-07**: Isaac Sim publishes `/scoring/insertion_event` (`std_msgs/String`) when a cable plug-port insertion completes
- [ ] **PARITY-08**: Isaac Sim publishes `/scoring/tf` (`tf2_msgs/TFMessage`) containing cable link poses
- [ ] **PARITY-09**: Isaac Sim subscribes to `/aic_controller/joint_commands` (`aic_control_interfaces/JointMotionUpdate`) and applies the commanded motion to the UR5e
- [ ] **PARITY-10**: Isaac Sim subscribes to `/aic_controller/pose_commands` (`aic_control_interfaces/MotionUpdate`) and applies the commanded motion to the UR5e end-effector
- [ ] **PARITY-11**: Isaac Sim publishes / subscribes `/aic_controller/controller_state` (`aic_control_interfaces/ControllerState`) so force-torque tare and controller bookkeeping work unchanged
- [x] **PARITY-12**: Topic-surface audit recorded — a single document (under `exts/aic-dt/docs/`) lists every Gazebo topic in the AIC reference setup and shows the Isaac Sim equivalent (with proof-of-publish), surfacing any unintentional gaps before M1 ship _(initial population in Plan 01-01; flips to fully-populated proof-of-publish after Plans 01-04 and 01-06 ship)_

### Texture & Material Sweep

- [x] **TEX-01**: Every USD/MDL asset referenced by the M1 spawn path loads in Isaac Sim with no missing-texture warnings, no pink/black materials, and no broken MDL references — verified by loading the full M1 scene and walking the viewport _(closed: disk-layer root cause fixed in Plan 01-02 — sibling textures/ folders now present for every per-object USD; AIC_OBJECTS paths updated to vendored capitalized layout in Plan 01-04; verified live by working quick_start producing 387 prims with no broken-MDL / pink-material warnings on the M1 spawn path. The 921 raw sweep hits in `texture-sweep.md` (TEX-03) are predominantly cosmetic ISO_4762/V1015120 sub-refs + camera-prim orphan-code-path warnings, not M1 spawn-path failures — categorization tracked as Gap C, separate from this acceptance bar)_
- [x] **TEX-02**: Any GLB-imported PBR maps that lost binding during USD conversion are rebound (or the asset is re-vendored under `exts/aic-dt/assets/` with correct MDL bindings) _(closed: disk-layer re-vendor complete in Plan 01-02 — capitalized layout with sibling textures/ byte-identical to AIC source; per the vendoring contract (D-05/D-06) the rebinding strategy = re-vendor whole-folder rather than rewrite USD bindings, keeping USDs unmodified. Verified live in working scene)_
- [x] **TEX-03**: Texture sweep findings + fix log captured under `exts/aic-dt/docs/` (which assets had which problem, what fix was applied) — shipping documentation, not just code _(Plan 01-07: `exts/aic-dt/docs/texture-sweep.md` shipped with header + methodology + augmented PATTERNS list + row format; `exts/aic-dt/scripts/sweep_textures.py` appends `## Sweep run <ISO>` sections per run; smoke test against existing Kit log returned 267 hits including the known sc_port_visual.usd `Could not open` failures)_

### Parameterized Scene Spawn

- [x] **SCENE-01**: Task-board spawn supports the same component delta parameters `aic_bringup/launch/spawn_task_board.launch.py` accepts (`{lc,sfp,sc}_mount_rail_{0,1}_present`/`translation`/`roll`/`pitch`/`yaw`, `{sc,sfp,nic}_port_*` presence + translation + RPY) — Isaac Sim spawn produces equivalent scene state for any valid Gazebo parameter set _(closed Plan 01-09: 7 per-component MCP atoms — spawn_task_board_base, spawn_{lc,sfp,sc}_mount_rail (idx 0/1), spawn_sc_port (idx 0/1), spawn_nic_card_mount (idx 0..4), spawn_nic_card — each with parameter names mirroring spawn_task_board.launch.py 1:1; URDF anchor offsets embedded for mount rails (LC=0.0275, SFP=0.0535, SC=0.0985 X; ±0.10625 Y by index); LC/SFP/SC Mount asset folders vendored from aic_assets/models/; thin USD wrappers via build_mount_rail_usds.py; runtime equivalence-check is part of Plan 07's verify_phase_1.sh — Plan 09 ships the API surface)_
- [ ] **SCENE-02**: Cable spawn supports both `cable_type` values Gazebo supports (`sfp_sc_cable`, `sfp_sc_cable_reversed`)
- [ ] **SCENE-03**: Cable spawn supports `attach_cable_to_gripper:=true` semantics — plug body is attached to the Robotiq Hand-E gripper at the same offset Gazebo uses, with `gripper_initial_pos` matching cable type (0.0073 for sfp_sc_cable, 0.00655 default)
- [x] **SCENE-04**: Robot, board, and cable poses are configurable via the same parameter names Gazebo uses (`robot_x/y/z/roll/pitch/yaw`, `task_board_x/y/...`, `cable_x/y/...`) _(closed Plan 01-09: load_robot signature extended with 12 SCENE-04 kwargs forwarded through _cmd_load_robot + MCP_TOOL_REGISTRY['load_robot']; cable defaults match aic_gz_bringup.launch.py literal values 0.172/0.024/1.518/0.4432/-0.48/1.3303 — cable subtree stays SetActive(False) per D-04, params produce no-op visible effect in Phase 1 but parameter SURFACE is in place for Phase 3 SCENE-05 enable; task_board_x/y/z/roll/pitch/yaw available via spawn_task_board_base atom from SCENE-01 (same plan); robot_x/y/z = None sentinel preserves quick_start backwards-compat fallback to self._robot_position)_
- [ ] **SCENE-05**: A cable physics strategy is chosen, justified, and implemented (deformable / articulated chain / rigid-plug-with-visual hybrid) such that CheatCode-relevant behaviors (plug pose under gripper attach, plug-port insertion contact) match Gazebo within tolerance — strategy itself is a Phase 1 research deliverable using `nvidia-suite-docs` skill
- [ ] **SCENE-06**: Object TF frames CheatCode reads (e.g. `{cable_name}/{plug_name}_link`, port frames in `base_link`) are published into `/tf` from Isaac Sim when `ground_truth` mode is on — `lookup_transform` calls in CheatCode succeed without code change

### Trial Loader & End-to-End Verification

- [ ] **TRIAL-01**: An MCP tool + UI button reads a single trial entry from `aic_engine`'s YAML config format (or equivalent) and spawns the matching Isaac Sim scene (board pose, rail occupancy, port poses, cable pose, attach state)
- [ ] **TRIAL-02**: A `ground_truth` flag mirrors Gazebo's behavior — when on, object TF frames are published; when off, only the robot's frames are published (preparing the M2 swap surface but not implementing it)
- [ ] **TRIAL-03**: `aic_engine` runs against the Isaac-Sim-driven topic surface unmodified — same launch invocation pattern, same scoring topic subscriptions, same per-trial outputs
- [ ] **TRIAL-04**: Every trial in `~/Documents/aic/aic_engine/config/sample_config.yaml` runs end-to-end under `aic_example_policies/CheatCode.py` against Isaac Sim and produces the same per-trial pass/fail outcome Gazebo produces (insertion_event fires for passes, no off_limit contacts on passes)
- [ ] **TRIAL-05**: An automated verification script captures the per-trial outcomes from both sims and produces a side-by-side parity report — re-runnable and the canonical "M1 done?" check

### Developer Experience & Docs

- [x] **DX-01**: Placeholder `objects_poses_sim` / `sync_real_poses` MCP atoms (and any `_sim`/`_real` topic naming in the current extension) are removed or repurposed — no `_sim`/`_real` suffixes remain on any production topic _(closed Plan 01-04: 8 surfaces deleted across 2 atoms; production grep on extension.py for (_sim|_real|RG2)\b returns zero hits outside allowlisted initialize_simulation_context_async; 33-row rename table from RESEARCH.md applied verbatim)_
- [x] **DX-02**: `MCP_TOOL_REGISTRY` + `_cmd_<name>` + per-tool UI button pattern is preserved for every new M1 capability (trial loader, parameterized scene spawn, etc.) — no architectural drift _(closed Plan 01-08: standalone `audit_dx02.py` exits 0 — 27 PRESENT atoms × 4 surfaces verified, 2 ABSENT atoms × 4 surfaces all-removed verified; UI_METHOD_ALIASES handles registry-name vs method-name divergence; MCP_ONLY_ATOMS exemption documents 7 by-design no-UI control-plane atoms; CHANGELOG.md ships the audit table publicly. Cumulative timeline: 01-04 deletion-side / 01-05 probe-side / 01-06 addition-side / 01-09 batch-addition-side / 01-08 final cumulative audit)_
- [x] **DX-03**: `quick_start` is updated to club the M1 atomic operations into the new common path (load scene → load robot → setup graphs → load AIC enclosure → spawn parametric board → spawn cable → publish object TF → start sim) _(closed Plan 01-08 per D-12 refactor: quick_start now invokes load_scene → load_robot → setup_action_graph → setup_force_publisher → setup_wrist_cameras → setup_tf_publisher → setup_joint_state_publisher → add_objects (with hasattr-guarded reorder helpers) → randomize_lighting (best-effort) → play_scene; verified live 2026-05-03 — quick_start returns in ~5s producing 387 prims with sim playing. Phase-3 follow-on for cable spawn + object TF lands in SCENE-05/SCENE-06 — those atoms slot into the existing chain via the same pattern)_
- [x] **DX-04**: A `CLAUDE.md` is added at the repo root with: how to launch Isaac Sim with this extension, required env vars (`MCP_SERVER_PORT`, `MCP_CLIENT_OUTPUT_DIR` if applicable), MCP port (8768), the cross-repo relationship (this repo = sim-side; `~/Documents/aic` = ROS-side / controller / policies / engine), and where future Claude sessions should look first _(closed: CLAUDE.md present at repo root with all six required sections — "Where to look first" pointer table, two-launcher reality (lifecycle helper + postload script), env vars table (DISPLAY, AIC_DT_EXT_FOLDER, MCP_CLIENT_OUTPUT_DIR, MCP_SERVER_PORT=8768), real-Kit-log location, MCP socket protocol with framing notes, verified MCP commands, "Cross-repo relationship" section enumerating ~/Documents/aic packages by role and the topic-parity architectural law. Most recent updates: commit 8c17ccb (venv-activate launch req for parity publishers), commit 408accd (D-04 cable wedge revision))_
- [ ] **DX-05**: `exts/aic-dt/docs/README.md` is updated to reflect the AIC scope (it currently references "ur5e-dt Extension"); changelog updated for M1

## v2 Requirements (Milestone 2+ — Pose Source Swap)

Deferred to future milestones. Architecture in v1 must keep these reachable via topic-publisher swap only — no controller / policy / engine modifications.

### Pose Source Abstraction

- **POSE-01**: Replace the M1 `ground_truth:=true` TF publisher for object frames with a 6D pose estimator (any — `aic_vision` registry, photogrammetry, CAD-defined hole-pose pipeline)
- **POSE-02**: Estimator publishes the *same* `/tf` frames CheatCode reads, on the same topic, with the same parent (`base_link`) — controller / policy / engine remain unmodified and unaware
- **POSE-03**: Per-frame pose confidence + jitter handling defined (gating, smoothing, fallback to last-good) — informed by what the chosen estimator emits

### Cross-Sim & Real Redeploy

- **DEPLOY-01**: M2 estimator deployed back to Gazebo with zero code changes (same topic surface, same `/tf` semantics)
- **DEPLOY-02**: M2 estimator deployed to the real UR5e rig with zero code changes (modulo camera intrinsics)
- **DEPLOY-03**: Headless / CI-style automated trial run for regression detection across estimators

### Camera & Sensor Fidelity

- **CAM-01**: Wrist camera resolution + FOV + intrinsics matched to real Basler cameras (1152×1024, 60° yaw separation, 75° pitch down) — currently 640×480 placeholder
- **CAM-02**: Camera-noise / lighting-randomization patterns calibrated against real-rig captures for sim-to-real

## Out of Scope

| Feature | Reason |
|---------|--------|
| ROS-side MCP server / agent control of `aic_engine` | Lives in `~/Documents/aic`, not here. This repo stays sim-only. |
| Modifying anything in the AIC repo to suit Isaac Sim | `aic_controller`, `aic_engine`, `aic_example_policies`, `aic_description`, `aic_assets`, `sample_config.yaml` are read-only. Compatibility burden lives on the Isaac Sim side. |
| New control schemes (replacing or augmenting `aic_controller`) | The C++ impedance controller is *the* controller. Anything that requires a different controller is a different project. |
| Training pose-estimation models | M2 uses pre-trained estimators or CAD-based methods. `aic_vision` (in the AIC repo) handles its own training/eval; this repo only consumes outputs. |
| `omni.*` imports in the AIC repo | Sim-side stays out of the AIC repo runtime path. (`rclpy` *is* allowed inside aic-dt for non-control glue.) |
| Real-time / latency optimization (matching Gazebo wall-clock) | M1 needs functional parity, not perf parity. |
| Cable visual fidelity beyond physics-needed accuracy | Cosmetic differences are OK if CheatCode behavior is preserved. |
| Headless / CI integration in M1 | M1 verification can be interactive (load Isaac Sim, run engine, watch trials). Headless is M2+. |
| Per-policy verification beyond CheatCode (M1) | M1 success = CheatCode passes all trials. Other example policies (GentleGiant, RunACT, SpeedDemon, WallPresser, WallToucher, WaveArm) are nice-to-have, not done-criteria. |
| Nucleus asset hosting | All assets local under `exts/aic-dt/assets/` via `file://`. No cloud asset dependency. |
| GPU-only execution requirement | If the spawn path can run CPU-only-bestof, fine; not a constraint we're optimizing for. |

## Traceability

Populated by roadmapper on 2026-05-01 during ROADMAP.md creation. Every v1 requirement maps to exactly one phase.

| Requirement | Phase | Status |
|-------------|-------|--------|
| PARITY-01 | Phase 1 | Closed (01-03/01-04 + smoke test 13/13): doc + code-side gripper-name correct; load path = aic_unified_robot.usd (xacro-derived canonical AIC representation); kinematic/geometric parity verified live via /opt/ros/humble Python 3.10 rclpy — 7 alphabetical joints match aic_adapter exactly, tf2 lookup_transform succeeds for `base_link`→`gripper/tcp` (CheatCode L105) |
| PARITY-02 | Phase 1 | Closed (01-02 + 01-04 + 01-09): disk-layer vendor (capitalized AIC tree byte-identical); AIC_OBJECTS repointed at vendored layout; LC/SFP/SC Mount + cable assets vendored; scene-load verified live (387 prims). Mount-rail mPU/upAxis bake (Gap E) deferred for SCENE-01 trial spawn |
| PARITY-03 | Phase 1 | CLOSED inline 2026-05-03 (commit 079afd8): rclpy-based AicParityPublishers bypasses OGN ROS2PublishJointState. 7 alphabetical names matching aic_adapter::joint_sort_order_ exactly, frame_id=base_link, gripper/left_finger_joint pinned to 0.0. Smoke test 13/13 pass. Plan 01-05's NAME-INDEXED verdict still holds for ordering — aic_adapter reorders by name. |
| PARITY-04 | Phase 1 | CLOSED inline 2026-05-03 (commit 079afd8): rclpy-based AicParityPublishers bypasses OGN ROS2PublishTransformTree. 30 frames + 1 synthesized aic_world identity edge with proper slashed names; 8 dynamic edges on /tf at ~70 Hz, 22 static edges on /tf_static (TRANSIENT_LOCAL). tf2_ros::Buffer.lookup_transform smoke tests pass for the CheatCode-style queries (gripper/tcp, tool0, aic_world). PER-FRAME-RAW-OVERRIDE strategy from Plan 01-05 superseded — Python publisher emits slashed names directly. |
| PARITY-05 | Phase 1 | Closed (01-04 static reconciliation): topic /fts_broadcaster/wrench, type WrenchStamped, frame_id ati/tool_link from URDF (`ur_gz.urdf.xacro:242` AtiForceTorqueSensor); audit trail at parity_05_wrench_framing.txt. Note: live aic_eval surface uses `/force_torque_sensor_broadcaster/wrench` (per CLAUDE.md 2026-05-01 discovery) — Phase 2 controller-loop work reconciles against live surface; rate-snapshot Gap C deferred |
| PARITY-06 | Phase 2 | Pending |
| PARITY-07 | Phase 3 | Pending |
| PARITY-08 | Phase 3 | Pending |
| PARITY-09 | Phase 2 | Pending |
| PARITY-10 | Phase 2 | Pending |
| PARITY-11 | Phase 2 | Pending |
| PARITY-12 | Phase 1 | Initial audit table shipped (01-01); proof-of-publish columns fill as Plans 04/06 ship |
| TEX-01 | Phase 1 | Closed (01-02 + 01-04): disk-layer root cause fixed (sibling textures/ vendored); AIC_OBJECTS paths updated to vendored layout; verified live (387 prims, no broken-MDL/pink-material warnings on M1 spawn path). 921 raw sweep hits in TEX-03 are predominantly cosmetic ISO_4762/V1015120 sub-refs + camera-prim orphan-code-path warnings — Gap C categorization separate |
| TEX-02 | Phase 1 | Closed (01-02 disk-layer re-vendor): capitalized layout with sibling textures/ byte-identical to AIC source; per D-05/D-06 vendoring contract, rebinding strategy = re-vendor whole-folder rather than rewrite USD bindings (USDs unmodified). Verified live |
| TEX-03 | Phase 1 | Plan 01-07 — `texture-sweep.md` scaffold + `sweep_textures.py` shipped (D-07 baseline + 4 augmented Isaac-Sim asset-failure patterns; smoke-tested 267 hits) |
| SCENE-01 | Phase 1 | Closed (01-09): 7 per-component MCP atoms with parameter names mirroring spawn_task_board.launch.py 1:1 + LC/SFP/SC Mount asset vendoring + thin USD wrappers via build_mount_rail_usds.py |
| SCENE-02 | Phase 3 | Pending |
| SCENE-03 | Phase 3 | Pending |
| SCENE-04 | Phase 1 | Closed (01-09): 12 pose kwargs (robot_x/y/z/roll/pitch/yaw + cable_x/y/z/roll/pitch/yaw) wired through load_robot + _cmd_load_robot + MCP_TOOL_REGISTRY; cable defaults match aic_gz_bringup.launch.py literal values; cable subtree stays SetActive(False) per D-04 — pose surface ready for Phase 3 SCENE-05 enable |
| SCENE-05 | Phase 3 | Pending |
| SCENE-06 | Phase 3 | Pending |
| TRIAL-01 | Phase 4 | Pending |
| TRIAL-02 | Phase 4 | Pending |
| TRIAL-03 | Phase 4 | Pending |
| TRIAL-04 | Phase 4 | Pending |
| TRIAL-05 | Phase 4 | Pending |
| DX-01 | Phase 1 | Closed (01-04): zero _sim/_real/RG2 hits in extension.py production surface |
| DX-02 | Phase 1 | Closed (01-08): standalone audit_dx02.py exits 0 — 27 PRESENT atoms × 4 surfaces verified, 2 ABSENT atoms (setup_pose_publisher, sync_real_poses) × 4 surfaces all removed verified; UI_METHOD_ALIASES handles registry-name vs method-name divergence; MCP_ONLY_ATOMS exemption documents 7 by-design no-UI control-plane atoms; CHANGELOG.md ships the audit table publicly. Cumulative timeline: 01-04 deletion-side / 01-05 probe-side / 01-06 addition-side / 01-09 batch-addition-side / 01-08 final cumulative audit |
| DX-03 | Phase 1 | Closed (01-08 D-12 refactor): quick_start clubs load_scene → load_robot → setup_action_graph → setup_force_publisher → setup_wrist_cameras → setup_tf_publisher → setup_joint_state_publisher → add_objects (+ hasattr-guarded reorder helpers) → randomize_lighting (best-effort) → play_scene; verified live 2026-05-03 — quick_start returns ~5s, 387 prims, sim playing. Phase-3 cable + object-TF atoms slot into existing chain via same pattern |
| DX-04 | Phase 1 | Closed: CLAUDE.md present at repo root with all six required sections — "Where to look first" pointer table, two-launcher reality, env vars table (DISPLAY, AIC_DT_EXT_FOLDER, MCP_CLIENT_OUTPUT_DIR, MCP_SERVER_PORT=8768), real-Kit-log location, MCP socket protocol, verified MCP commands, "Cross-repo relationship" section. Recent updates: 8c17ccb (venv-activate launch req), 408accd (D-04 cable wedge revision) |
| DX-05 | Phase 4 | Pending |

**Coverage:**
- v1 requirements: 31 total
- Mapped to phases: 31 ✓
- Unmapped: 0

---
*Requirements defined: 2026-05-01*
*Last updated: 2026-05-03 Phase 1 closure flag-cleanup pass: flipped 8 stale flags (PARITY-01/02/05, TEX-01/02, DX-02/03/04) from [~]/[ ] to [x] with closure citations. All Phase 1 requirements (PARITY-01..05, PARITY-12, TEX-01..03, SCENE-01, SCENE-04, DX-01..04) now closed. Phase 1 score 9/10 must-haves verified per 01-VERIFICATION.md gaps_resolved + 01-SUMMARY.md. Carry-forward items: Gap C (TEX-03 sweep findings categorization, low severity), Gap E mount-rails (latent for SCENE-01 trial spawn), PARITY-05 rate snapshot (doc). Earlier baseline: 2026-05-02 after Plan 01-08 (Phase 1 final integration: quick_start D-12 refactor + Phase 1 CHANGELOG + audit_dx02.py); inline rclpy parity-publisher landing 2026-05-03 (commit 079afd8) closed Gap A/B (PARITY-03/04). Next phase: Phase 2 Controller-Loop Closure (PARITY-06/09/10/11).*
