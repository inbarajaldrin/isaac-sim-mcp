# Requirements: aic-dt — AIC Cable-Insertion Digital Twin

**Defined:** 2026-05-01
**Core Value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.

## v1 Requirements (Milestone 1 — Platform Transfer)

### Asset & Topic Parity

- [~] **PARITY-01**: Isaac Sim loads the AIC repo's `aic_description` xacro/URDF for the UR5e + Robotiq Hand-E + 3 wrist cameras (no divergent kinematics or geometry from Gazebo) _(doc + code-side gripper-name corrected in Plans 01-03 / 01-04; URDF xacro load path remains unimplemented — final close lands in Plan 01-06 or 01-08)_
- [~] **PARITY-02**: Isaac Sim loads the AIC repo's `aic_assets` meshes for the task board, ports (sc / sfp / lc / nic), mount rails, cables, and the AIC enclosure (no divergent geometry) _(disk-layer vendor complete in Plan 01-02 — capitalized AIC tree byte-identical under exts/aic-dt/assets/assets/; AIC_OBJECTS code path repointed at vendored layout in Plan 01-04; scene-load verification at Plan 01-06; LC/SFP mount rails + cable assets vendored in Plan 01-09)_
- [~] **PARITY-03**: Isaac Sim publishes `/joint_states` with the same joint name set + ordering Gazebo publishes _(reference snapshot captured in Plan 01-01; implementation in Plan 01-06)_
- [~] **PARITY-04**: Isaac Sim publishes `/tf` and `/tf_static` containing the same robot + gripper + camera frames Gazebo publishes (with same parent-child hierarchy and frame names) _(reference snapshot captured in Plan 01-01; implementation in Plan 01-06)_
- [~] **PARITY-05**: Isaac Sim publishes `/fts_broadcaster/wrench` (`geometry_msgs/WrenchStamped`) for the UR5e end-effector force/torque, matching Gazebo's topic name and frame_id _(static reconciliation complete in Plan 01-04: topic renamed to /fts_broadcaster/wrench, message type WrenchStamped preserved, header.frame_id corrected tool0->ati/tool_link from URDF; runtime ros2 topic info + echo verification deferred to Plan 01-07's verify_phase_1.sh; audit trail at parity_05_wrench_framing.txt)_
- [ ] **PARITY-06**: Isaac Sim publishes `/aic/gazebo/contacts/off_limit` (`ros_gz_interfaces/Contacts`) for off-limit-item contact events — exact topic name kept even though "gazebo" is in it (zero-rename rule)
- [ ] **PARITY-07**: Isaac Sim publishes `/scoring/insertion_event` (`std_msgs/String`) when a cable plug-port insertion completes
- [ ] **PARITY-08**: Isaac Sim publishes `/scoring/tf` (`tf2_msgs/TFMessage`) containing cable link poses
- [ ] **PARITY-09**: Isaac Sim subscribes to `/aic_controller/joint_commands` (`aic_control_interfaces/JointMotionUpdate`) and applies the commanded motion to the UR5e
- [ ] **PARITY-10**: Isaac Sim subscribes to `/aic_controller/pose_commands` (`aic_control_interfaces/MotionUpdate`) and applies the commanded motion to the UR5e end-effector
- [ ] **PARITY-11**: Isaac Sim publishes / subscribes `/aic_controller/controller_state` (`aic_control_interfaces/ControllerState`) so force-torque tare and controller bookkeeping work unchanged
- [x] **PARITY-12**: Topic-surface audit recorded — a single document (under `exts/aic-dt/docs/`) lists every Gazebo topic in the AIC reference setup and shows the Isaac Sim equivalent (with proof-of-publish), surfacing any unintentional gaps before M1 ship _(initial population in Plan 01-01; flips to fully-populated proof-of-publish after Plans 01-04 and 01-06 ship)_

### Texture & Material Sweep

- [~] **TEX-01**: Every USD/MDL asset referenced by the M1 spawn path loads in Isaac Sim with no missing-texture warnings, no pink/black materials, and no broken MDL references — verified by loading the full M1 scene and walking the viewport _(disk-layer root cause fixed in Plan 01-02 — sibling textures/ folders now present for every per-object USD; final viewport verification at Plan 01-06 verify_phase_1.sh after Plan 01-04 updates AIC_OBJECTS paths)_
- [~] **TEX-02**: Any GLB-imported PBR maps that lost binding during USD conversion are rebound (or the asset is re-vendored under `exts/aic-dt/assets/` with correct MDL bindings) _(disk-layer re-vendor complete in Plan 01-02 — assets now at correct on-disk locations relative to USD's relative texture references; final binding verification at Plan 01-06)_
- [ ] **TEX-03**: Texture sweep findings + fix log captured under `exts/aic-dt/docs/` (which assets had which problem, what fix was applied) — shipping documentation, not just code

### Parameterized Scene Spawn

- [ ] **SCENE-01**: Task-board spawn supports the same component delta parameters `aic_bringup/launch/spawn_task_board.launch.py` accepts (`{lc,sfp,sc}_mount_rail_{0,1}_present`/`translation`/`roll`/`pitch`/`yaw`, `{sc,sfp,nic}_port_*` presence + translation + RPY) — Isaac Sim spawn produces equivalent scene state for any valid Gazebo parameter set
- [ ] **SCENE-02**: Cable spawn supports both `cable_type` values Gazebo supports (`sfp_sc_cable`, `sfp_sc_cable_reversed`)
- [ ] **SCENE-03**: Cable spawn supports `attach_cable_to_gripper:=true` semantics — plug body is attached to the Robotiq Hand-E gripper at the same offset Gazebo uses, with `gripper_initial_pos` matching cable type (0.0073 for sfp_sc_cable, 0.00655 default)
- [ ] **SCENE-04**: Robot, board, and cable poses are configurable via the same parameter names Gazebo uses (`robot_x/y/z/roll/pitch/yaw`, `task_board_x/y/...`, `cable_x/y/...`)
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
- [~] **DX-02**: `MCP_TOOL_REGISTRY` + `_cmd_<name>` + per-tool UI button pattern is preserved for every new M1 capability (trial loader, parameterized scene spawn, etc.) — no architectural drift _(deletion-side proven in Plan 01-04: 4-surface contract honored when removing setup_pose_publisher + sync_real_poses; addition-side audited cumulatively by Plan 01-08 final-audit table; ongoing as new atoms ship in 01-06 and 01-09)_
- [ ] **DX-03**: `quick_start` is updated to club the M1 atomic operations into the new common path (load scene → load robot → setup graphs → load AIC enclosure → spawn parametric board → spawn cable → publish object TF → start sim)
- [ ] **DX-04**: A `CLAUDE.md` is added at the repo root with: how to launch Isaac Sim with this extension, required env vars (`MCP_SERVER_PORT`, `MCP_CLIENT_OUTPUT_DIR` if applicable), MCP port (8768), the cross-repo relationship (this repo = sim-side; `~/Documents/aic` = ROS-side / controller / policies / engine), and where future Claude sessions should look first
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
| PARITY-01 | Phase 1 | Doc + code-side gripper-name corrected (01-03, 01-04); URDF xacro load path unimplemented — final close in 01-06/01-08 |
| PARITY-02 | Phase 1 | Disk-layer vendor shipped (01-02); AIC_OBJECTS code path repointed at vendored capitalized layout (01-04); scene-load verify in 01-06; LC/SFP/cable assets in 01-09 |
| PARITY-03 | Phase 1 | Snapshot reference captured (01-01); awaiting implementation (01-06) |
| PARITY-04 | Phase 1 | Snapshot reference captured (01-01); awaiting implementation (01-06) |
| PARITY-05 | Phase 1 | Static reconciliation complete (01-04): topic /fts_broadcaster/wrench, type WrenchStamped, frame_id ati/tool_link; runtime echo verify deferred to 01-07 |
| PARITY-06 | Phase 2 | Pending |
| PARITY-07 | Phase 3 | Pending |
| PARITY-08 | Phase 3 | Pending |
| PARITY-09 | Phase 2 | Pending |
| PARITY-10 | Phase 2 | Pending |
| PARITY-11 | Phase 2 | Pending |
| PARITY-12 | Phase 1 | Initial audit table shipped (01-01); proof-of-publish columns fill as Plans 04/06 ship |
| TEX-01 | Phase 1 | Disk-layer root cause fixed (01-02 — sibling textures/ vendored); final viewport verify at Plan 01-06 |
| TEX-02 | Phase 1 | Disk-layer re-vendor complete (01-02 — capitalized layout with textures siblings byte-identical to AIC source); final binding verify at Plan 01-06 |
| TEX-03 | Phase 1 | Pending |
| SCENE-01 | Phase 1 | Pending |
| SCENE-02 | Phase 3 | Pending |
| SCENE-03 | Phase 3 | Pending |
| SCENE-04 | Phase 1 | Pending |
| SCENE-05 | Phase 3 | Pending |
| SCENE-06 | Phase 3 | Pending |
| TRIAL-01 | Phase 4 | Pending |
| TRIAL-02 | Phase 4 | Pending |
| TRIAL-03 | Phase 4 | Pending |
| TRIAL-04 | Phase 4 | Pending |
| TRIAL-05 | Phase 4 | Pending |
| DX-01 | Phase 1 | Closed (01-04): zero _sim/_real/RG2 hits in extension.py production surface |
| DX-02 | Phase 1 | Deletion-side proven (01-04): 4-surface contract for 2 removed atoms; addition-side audit in 01-08 |
| DX-03 | Phase 3 | Pending |
| DX-04 | Phase 4 | Pending |
| DX-05 | Phase 4 | Pending |

**Coverage:**
- v1 requirements: 31 total
- Mapped to phases: 31 ✓
- Unmapped: 0

---
*Requirements defined: 2026-05-01*
*Last updated: 2026-05-02 after Plan 01-04 (extension.py cleanup): DX-01 closed; PARITY-01, PARITY-02, PARITY-05, DX-02 advanced; PARITY-05 static reconciliation done; 8 MCP surfaces deleted*
