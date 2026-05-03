# Phase 2: Controller Loop - Context

**Gathered:** 2026-05-03 (autonomous M1 mode — `--auto`)
**Status:** Ready for planning

<domain>
## Phase Boundary

The unmodified `aic_controller` (impedance controller plugin in `~/Documents/aic/aic_controller/`) successfully drives the UR5e in Isaac Sim. This means:

- Isaac Sim **subscribes** to `/aic_controller/joint_commands` (`aic_control_interfaces/JointMotionUpdate`) and applies `target_state.positions` to UR5e articulation drives — PARITY-09.
- Isaac Sim **subscribes** to `/aic_controller/pose_commands` (`aic_control_interfaces/MotionUpdate`) and resolves the target Pose via IK, then applies the resulting joint positions — PARITY-10.
- Isaac Sim **publishes** `/aic_controller/controller_state` (`aic_control_interfaces/ControllerState`) so tare/bookkeeping logic in the controller and downstream policies (CheatCode) can read the current TCP pose, TCP velocity, and FTS tare offset — PARITY-11.
- Isaac Sim **publishes** `/aic/gazebo/contacts/off_limit` (`ros_gz_interfaces/Contacts`) when configured off-limit collision bodies are touched by the robot — PARITY-06. Topic name preserved verbatim per the zero-rename rule (the literal "gazebo" segment stays even though it's now Isaac Sim publishing).

**In scope (PARITY-06, PARITY-09, PARITY-10, PARITY-11):** the controller↔sim socket on the command/state side; off-limit contact event publishing.

**Out of scope:**
- Cable physics + plug-port insertion event + ground-truth object TF — Phase 3 (SCENE-02/03/05/06, PARITY-07/08).
- Trial loader + end-to-end CheatCode parity verification — Phase 4 (TRIAL-01..05, DX-04 final, DX-05).
- Parametric task-board spawn + pose params + wrench publisher + atomic+clubbed contract — already pulled forward to Phase 1 per the surface-adjacency rule (closed in Plan 01-04/08/09).

</domain>

<decisions>
## Implementation Decisions

### Subscriber + publisher transport mechanism

- **D-01:** **Mirror Phase 1's rclpy-class architecture, inverted.** Build `exts/aic-dt/aic_dt/controller_loop.py` containing an `AicControllerLoop` class that owns one rclpy node with: 2 subscribers (`/aic_controller/joint_commands`, `/aic_controller/pose_commands`), 2 publishers (`/aic_controller/controller_state`, `/aic/gazebo/contacts/off_limit`), running on the same physics-tick callback `parity_publishers.py` already uses. **Rationale (auto-recommended):** the AIC message types `JointMotionUpdate`, `MotionUpdate`, `ControllerState` are custom (not stock `sensor_msgs/*`) — OGN's `ROS2SubscribeJointState` / `ROS2Publisher` nodes only deserialize stock types. The same architectural pivot Phase 1 made on the publisher side (D-10/D-11 in `01-CONTEXT.md`) applies here on the subscriber side. Reuses Phase 1's venv-activate + sys.path swap discipline (workspace's libgeometry_msgs LD ordering); requires no new infrastructure.

### IK strategy for `/aic_controller/pose_commands`

- **D-02:** **Lula RMPflow first, PyKDL fallback.** PARITY-10's `MotionUpdate.pose` is a Cartesian target on the EE; Isaac Sim resolves to joint positions via IK before writing to articulation drives. Lula RMPflow is bundled in `isaacsim.robot_motion.lula` and natively understands UR5e via the AIC unified-robot URDF/USD; PyKDL is the universal fallback if RMPflow's solver fails to converge or the chain definition mismatches. **Skip cuMotion** — overkill for this single-arm, no-collision-aware IK use (per HANDOFF.json decision). **Rationale (HANDOFF D-3 lock):** aic_controller does the impedance math; Isaac Sim just applies resolved motion. Simplest IK that works.

### Off-limit contact-event source for `/aic/gazebo/contacts/off_limit`

- **D-03:** **`omni.physx` contact-report subscription per the `robot-collision-forensics` skill.** Use `omni.physx.IPhysxSimulationInterface.subscribe_contact_report_events(...)` to receive per-physics-step contact events; filter by participant prim path against a configurable `OFF_LIMIT_PRIMS` set; emit `ros_gz_interfaces/Contacts` on the configured topic. **Do NOT use `isaacsim.sensors.physics.ContactSensor`** — the skill's "open-problem section" documents `is_valid=False` failure on `ContactSensor` in Isaac Sim 5.0. Don't use trimesh-based proximity checks — those don't reflect physics-step ground truth.

### MCP atom decomposition

- **D-04:** **Two new MCP atoms** — `setup_controller_subscribers` (manages the AicControllerLoop class: 2 subs + 1 state pub) and `setup_offlimit_contacts` (manages the contact-report subscription + Contacts pub). **Rationale:** clean two-domain split matches the two infrastructure layers (rclpy node + omni.physx contact-report). 4-surface contract per atom (registry + handler-map + `_cmd_<name>` method + UI button) per DX-02. Both atoms slot into `quick_start` after `setup_joint_state_publisher` in the existing chain (per DX-03 D-12 refactor pattern).

### Custom AIC message types — Python import path

- **D-05:** **PYTHONPATH-link to `~/Documents/aic/.pixi/envs/default/lib/python3.*/site-packages/`** at extension start-time. The AIC pixi env already builds `aic_control_interfaces` Python bindings via `colcon`-style ROS Kilted build; the `.pixi/envs/default/share/aic_control_interfaces/msg/*.msg` files are present and `.pixi/envs/default/lib/.../aic_control_interfaces/` ships compiled `_*.py` modules. Re-vendoring would duplicate source-of-truth and require running rosidl in this repo. **Failure-mode handling:** if the pixi env path doesn't exist (developer hasn't built AIC's pixi env), log a clear error and skip controller-loop atoms — extension still runs for non-controller workflows. Document in `exts/aic-dt/docs/aic-msgs-setup.md`.

### Impedance / stiffness / feedforward fields

- **D-06:** **Apply per-joint stiffness/damping/feedforward_torque to articulation drives; ignore Cartesian impedance + wrench-feedback fields.** `JointMotionUpdate.target_stiffness[]` + `target_damping[]` map to `Articulation.set_gains(kps, kds)` per DOF when length matches the articulation joint count (UR5e 6 + Hand-E 1 = 7). `JointMotionUpdate.target_feedforward_torque[]` maps to articulation `effort` command alongside `position`. `MotionUpdate.target_stiffness[36]` (6×6 Cartesian impedance) + `MotionUpdate.feedforward_wrench_at_tip` + `MotionUpdate.wrench_feedback_gains_at_tip[6]` are **controller-side math** — aic_controller already used them to compute the joint commands. Isaac Sim is the hardware sink; double-applying them in sim corrupts the controller's intent. Log them at debug level on receipt for trace; do not act on them.

### `ControllerState` publishing — what to compute, what to leave zero

- **D-07:** **Isaac Sim populates the *measured/actual* fields; leaves the *reference* and *tare* fields zero / passthrough.** Specifically:
  - `header.frame_id` = `base_link`, `header.stamp` = current sim time per ROS clock (matches Phase 1 publisher convention).
  - `tcp_pose` = forward-kinematics result from `Articulation.get_world_poses()` for `gripper/tcp` USD prim → expressed in `base_link` frame (use Phase 1's tf2 lookup_transform pattern in reverse).
  - `tcp_velocity` = numerically differentiated tcp_pose between physics ticks (small ring buffer; first 2-3 samples zero).
  - `reference_tcp_pose` = last-received `MotionUpdate.pose` (zero before first command).
  - `tcp_error` = computed from tcp_pose vs reference_tcp_pose (6-DOF: x,y,z,rx,ry,rz).
  - `reference_joint_state` = last-received `JointMotionUpdate.target_state` (empty before first command).
  - `target_mode` = inferred from which command stream last arrived (joint vs pose); zero-init.
  - `fts_tare_offset` = zero WrenchStamped (frame_id `ati/tool_link`). The actual tare offset is computed by aic_controller from /fts_broadcaster/wrench history; Isaac Sim doesn't tare — it just publishes the field as zero so the message structure is complete and CheatCode-style consumers don't trip on missing fields. Gazebo's behavior here verified before final close (PARITY-11 cross-check against live aic_eval container).

### Tick rate / spin model

- **D-08:** **Physics-tick callback (~60-120 Hz) for everything — same callback Phase 1 uses.** AicControllerLoop registers with the same `IPhysxSimulationInterface.subscribe_physics_step_events` machinery. Each tick:
  1. `rclpy.spin_once(node, timeout_sec=0)` to drain incoming messages into class-level state.
  2. Apply latest joint command (if any) to articulation.
  3. Apply latest pose command (if any) — IK → articulation.
  4. Read articulation state, compute tcp_pose/velocity/error, publish ControllerState.
  5. omni.physx contact-report callback (separate path) batches contacts into the per-tick Contacts message.
  
  No separate spin thread — keeps memory model simple and avoids GIL contention with Kit's main loop. Per `robot-collision-forensics` skill: physics-step events fire at the same rate as physics simulation ticks, so contact event batching aligns naturally with the loop's other work.

### Joint-name mapping for incoming joint_commands

- **D-09:** **Name-keyed input parser, mirror Phase 1's output mapper inverted.** `JointMotionUpdate.target_state.joint_names[]` is parsed against the live articulation's joint name table (`Articulation.dof_names`), with two known mappings:
  - Incoming `gripper/left_finger_joint` (literal slash, per aic_adapter::joint_sort_order_) → maps to USD path `gripper_left_finger_joint` (underscore) → which is a **FixedJoint**, so the position command is silently no-op'd (gripper opening is via `/gripper_command` String topic, not via /joint_commands — Phase 1 D-11 invariant).
  - Incoming UR5e joints map directly by name. Unknown joint names log a warning and are dropped (don't fail the message — controller may include extra joints during transitions).

### `OFF_LIMIT_PRIMS` configuration source

- **D-10:** **Hardcoded set of off-limit prim paths in `controller_loop.py`, parameterized via the `setup_offlimit_contacts` MCP atom signature.** Default set = task-board off-limit zones (e.g., screw heads, ESD-sensitive areas) per `~/Documents/aic/aic_assets/models/Task Board Base/`. Discovery: live Gazebo aic_eval publishes `/aic/gazebo/contacts/off_limit` only when the configured `<aic-engine>` collision filter triggers — Phase 2 research deliverable: snapshot which prims trigger the live container's off_limit topic during a CheatCode trial run (`docker exec aic_eval ros2 topic echo /aic/gazebo/contacts/off_limit`), then mirror the prim filter set in the Isaac Sim atom. If discovery returns ambiguous results, the atom accepts a `prim_paths: list[str]` parameter so callers can override.

### Failure modes

- **D-11:** **Drop bad commands silently with debug-level log; never raise into the physics callback.** A malformed `JointMotionUpdate` (joint count mismatch, NaN values, missing target_state) is logged and skipped — physics callback continues unaffected. AicControllerLoop tracks a `_last_good_command_time` for diagnostics. Rationale: sim should be permissive — controller's job to recover; sim crashing on a bad command makes debugging the controller harder, not easier.

### Verification

- **D-12:** **Phase 2 closes with a smoke test analog of `smoke_test_aic_parity.py`** — a Python 3.10 script using `/opt/ros/humble` rclpy that:
  1. Connects to the live Isaac Sim aic-dt instance.
  2. Publishes a known-good `JointMotionUpdate` (e.g., 6 UR5e joints to a small delta from current state).
  3. Verifies the articulation moved to the commanded configuration (within position tolerance; readable via `/joint_states` Phase 1 publisher).
  4. Publishes a known-good `MotionUpdate` (small EE delta).
  5. Verifies EE moved (verifiable via `/tf` lookup_transform `base_link → gripper/tcp`).
  6. Subscribes to `/aic_controller/controller_state` and verifies non-zero tcp_pose + nonzero tcp_error after the commands.
  7. Drives the gripper into a known off-limit prim and verifies non-empty `/aic/gazebo/contacts/off_limit` message.
  Closes on N/N pass — same pattern as Phase 1's 13/13.

### Phase 2 ops

- **D-13:** **Same launch path as Phase 1.** No new launcher. Cache discipline unchanged. venv-activate requirement extended to require `~/Documents/aic/.pixi/envs/default` in `LD_LIBRARY_PATH` for ros-kilted libs (in addition to humble for parity-publishers). Document in CLAUDE.md as Phase 2 ships.

### Claude's Discretion

- Exact name of the rclpy node (suggest `aic_dt_controller_loop` mirror of `aic_dt_parity_publishers`).
- Numerical-differentiation buffer size for tcp_velocity (suggest 3-sample, 1/dt scaling).
- Lula RMPflow config path for UR5e (researcher locates via `nvidia-suite-docs`; should already exist for stock Franka/UR templates).
- Off-limit prim path discovery script wording (D-10 details).
- Exact ROS QoS profile for each topic (suggest match Phase 1 — RELIABLE/VOLATILE for cmd, RELIABLE/TRANSIENT_LOCAL only when needed).
- Where the Lula RMPflow config file lives in the repo (suggest `exts/aic-dt/assets/robot/rmpflow_config.yaml`, derived/written from URDF).

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### This project (planning context)
- `.planning/PROJECT.md` — Vision, constraints, key decisions, repo split rationale.
- `.planning/REQUIREMENTS.md` — PARITY-06/09/10/11 acceptance criteria. Live aic_eval surface is the parity tiebreaker per the 2026-05-01 discovery note.
- `.planning/ROADMAP.md` §"Phase 2: Controller Loop" — goal + 5 success criteria the verifier checks against. Note: SC #4 (parametric task-board spawn) was pulled forward to Phase 1 — auto-pass for Phase 2 verifier.
- `.planning/HANDOFF.json` — autonomous-M1 anchor; D-2 + D-3 + D-4 lock the controller wiring + IK choices for this phase.
- `.planning/phases/01-foundation-parity/01-CONTEXT.md` — Phase 1 architectural decisions; D-10/D-11 establish the OGN-vs-rclpy precedent that drives Phase 2's D-01.
- `.planning/phases/01-foundation-parity/01-SUMMARY.md` — Phase 1 closure; rclpy-class architecture proven 13/13 against /opt/ros/humble Python 3.10.
- `CLAUDE.md` (repo root) — launch flow, venv-activate requirement, MCP socket protocol, real Kit log location, cross-repo relationship rules.

### AIC reference repo (read-only)
- `~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg` — PARITY-09 message contract: target_state (JointTrajectoryPoint) + per-joint stiffness/damping + TrajectoryGenerationMode + per-joint feedforward_torque.
- `~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg` — PARITY-10 message contract: Header + Pose + Twist + 6×6 Cartesian stiffness/damping + feedforward_wrench_at_tip + wrench_feedback_gains[6] + TrajectoryGenerationMode.
- `~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/ControllerState.msg` — PARITY-11 message contract: Header + tcp_pose + tcp_velocity + reference_tcp_pose + tcp_error[6] + reference_joint_state + target_mode + fts_tare_offset (WrenchStamped).
- `~/Documents/aic/aic_controller/src/` — Source code of the controller plugin we're driving from. Read for: which fields it expects to roundtrip via ControllerState, what its publish rate on `/aic_controller/joint_commands` is, what the TrajectoryGenerationMode enum values mean operationally.
- `~/Documents/aic/.pixi/envs/default/share/aic_control_interfaces/msg/` — Built message definitions (sanity-check against source). 
- `~/Documents/aic/.pixi/envs/default/lib/python*/site-packages/aic_control_interfaces/` — Compiled Python bindings; the import target for D-05.

### This extension's current state
- `exts/aic-dt/aic_dt/parity_publishers.py` — Phase 1's rclpy-class architecture; **the direct template for Phase 2's controller_loop.py**. Mirror the class structure, the physics-tick callback registration, the LD/sys.path discipline, the on_shutdown teardown.
- `exts/aic-dt/aic_dt/extension.py` lines 1280-1320 (`setup_action_graph`) — the EXISTING OGN-based subscribe pattern for stock `/joint_states` → IsaacArticulationController. Phase 2 supplements but does not replace this; this graph keeps `/joint_states` ↔ articulation as a fallback / debug path for low-level joint commanding outside the AIC controller flow.
- `exts/aic-dt/aic_dt/extension.py` lines 1233+ (`Articulation` setup) — articulation root prim path (`/World/UR5e/aic_unified_robot/root_joint`), DOF naming, drive config (stiffness=2000, damping=100, max_force=87 from Isaac Lab) used by Phase 1's load_robot.
- `exts/aic-dt/aic_dt/extension.py` lines 142-200 (`MCP_TOOL_REGISTRY`) — the registry to extend with `setup_controller_subscribers` + `setup_offlimit_contacts` per DX-02 4-surface contract.
- `exts/aic-dt/scripts/smoke_test_aic_parity.py` — Phase 1 smoke test pattern; Phase 2's `smoke_test_aic_controller.py` mirrors this on the controller side.

### Skills (canonical NVIDIA + collision references)
- `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` — Per CLAUDE.md, the source of truth for Isaac Sim 5.0 extension lifecycle, articulation API surface, MCP socket protocol, troubleshooting. Researcher invokes for: `Articulation.apply_action()` signature, `set_gains()` semantics in Isaac Sim 5.0, articulation drive parameter mapping for impedance.
- `~/.claude/skills/nvidia-suite-docs/SKILL.md` — For OmniGraph node lookup, OpenUSD authoring, `isaacsim.robot_motion.lula` (Lula RMPflow) docs, ROS2 bridge specifics. Researcher invokes for: Lula RMPflow setup for UR5e, PyKDL Python bindings inside Isaac Sim's bundled python.
- `~/.claude/skills/robot-collision-forensics/SKILL.md` — **Authoritative reference for D-03.** Documents the `omni.physx` contact-report subscription pipeline, the broken `ContactSensor.is_valid=False` problem in Isaac Sim 5.0, the per-link contact attribution methodology. Phase 2's off-limit contacts implementation MUST follow this skill's pipeline; the skill was written specifically because contact-event attribution in Isaac Sim 5.0 is non-obvious.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

- **`AicParityPublishers` class in `parity_publishers.py`**: 529 lines; the inverted template for `AicControllerLoop`. Reuse: rclpy node lifecycle, physics-tick callback registration via `omni.physx.acquire_physx_interface().subscribe_physics_step_events()`, on_shutdown cleanup, sys.path swap + sys.modules eviction discipline at start-time, LD/PYTHONPATH precondition checks.
- **`Articulation.apply_action(ArticulationActions(...))` (extension.py:1233+)**: Standard Isaac Sim 5.0 API for writing position/velocity/effort commands per DOF. Phase 2 calls this from physics-tick after IK + name-mapping.
- **`Articulation.dof_names` and `joint_index` mapping**: Already used in parity_publishers.py for the publish-side; Phase 2 inverts the same map for the subscribe-side input parser.
- **MCP_TOOL_REGISTRY 4-surface pattern (`registry entry + handler-map + _cmd_<name> method + UI button`)**: Proven 27 PRESENT × 4 surfaces in Plan 01-08 audit. Phase 2 adds 2 atoms; total atom count grows by 2. Verified by audit_dx02.py (extension.py:152+).
- **`quick_start` chain (extension.py:3150+)**: 9 steps in current order. Phase 2 inserts new atoms after `setup_joint_state_publisher` and before `add_objects` (preserves load order: scene → robot → publishers → subscribers → objects → play).
- **on_shutdown teardown in extension.py**: Cleans the parity_publishers manager; Phase 2 extends to clean the controller_loop manager + omni.physx contact-report subscription handle.

### Established Patterns

- **rclpy in physics-tick callback (Phase 1 D-10/D-11 pivot)**: When OGN can't deliver the message contract (here: custom AIC types), Python rclpy in a class running on `subscribe_physics_step_events` is the canonical answer. Already validated in production by parity_publishers.py.
- **Ground-truth via direct USD/articulation read, not via Isaac Sim sensors**: Phase 1 reads articulation state directly for /joint_states; Phase 2 reads articulation FK for tcp_pose. Avoids ContactSensor-style broken-in-5.0 sensor APIs.
- **Pre-flight infrastructure validation in atom handlers**: Phase 1 atoms check `articulation_root_prim_path` exists before publishing; Phase 2 atoms check articulation + AIC pixi env path before subscribing.
- **Physics-tick callback for ROS spin** (vs separate spin thread): keeps memory model simple, avoids GIL contention. Validated in Phase 1; mirror for Phase 2.

### Integration Points

- **`MCP_TOOL_REGISTRY` (extension.py:142)**: Add 2 entries (`setup_controller_subscribers`, `setup_offlimit_contacts`) per DX-02 contract.
- **Handler dispatch map in extension.py**: Add 2 entries mapping registry name → `_cmd_<name>` method.
- **`DigitalTwin` class methods**: Add `_cmd_setup_controller_subscribers` and `_cmd_setup_offlimit_contacts` methods.
- **UI buttons in extension.py UI section**: Add 2 buttons per DX-02 4-surface contract.
- **`quick_start` chain (extension.py:3150+)**: Add the 2 atoms after `setup_joint_state_publisher` (per DX-03 D-12 ordering).
- **on_shutdown (extension.py)**: Add cleanup for the new controller-loop manager + contact-report subscription handle.
- **Articulation root prim path (`/World/UR5e/aic_unified_robot`)**: Single source; both atoms read from `self._articulation_root_prim_path`.
- **`isaacsim.robot_motion.lula.motion_generation`** (Isaac Sim 5.0 API): Lula RMPflow integration point; researcher confirms exact import path + RmpFlow constructor signature.
- **`omni.physx.acquire_contact_report_event_stream()`** (Isaac Sim 5.0 / omni.physx 5.x): Contact-report event source; the `robot-collision-forensics` skill has the detailed subscription pipeline.

</code_context>

<specifics>
## Specific Ideas

- **"aic_controller IS the controller; Isaac Sim is the 'hardware' sink. Don't double up the controller layer."** (HANDOFF.json D-2, autonomous-M1 mode lock): the architectural anchor for D-01 + D-06. Justifies ignoring the impedance/wrench-feedback fields in MotionUpdate (those are controller-side computations, not hardware behavior) and the no-ros2_control-plugin-layer choice.
- **Phase 1's smoke test 13/13 is the closure pattern**: Phase 2 closes with a controller-side smoke test mirroring `smoke_test_aic_parity.py` in structure (Python 3.10 + rclpy + tf2_ros against the live extension). 7-step coverage per D-12.
- **`robot-collision-forensics` skill is the AUTHORITATIVE reference for off-limit contacts**: the skill was written specifically because contact-event attribution in Isaac Sim 5.0 is non-obvious and the stock ContactSensor is broken. Phase 2's implementation MUST follow the skill's omni.physx contact-report pipeline; deviating risks the same `is_valid=False` failure path documented in the skill's "open-problem section."
- **Live aic_eval container is the surface-of-truth for off-limit prim configuration** (D-10): rather than guessing which prims are off-limit from the YAML config, snapshot the live container during a CheatCode trial run and mirror the empirical filter set. Same empirical-not-config-doc principle that drove Phase 1's D-01 (live container = canonical reference).

</specifics>

<deferred>
## Deferred Ideas

- **`/gripper_command` (String) + `/gripper_status` / `/gripper_width` / `/gripper_grasp_detected` / `/gripper_motion_ongoing` / `/gripper_width_offset` topics** — gripper control surface is separate from the aic_controller arm-control surface. Belongs to a future Gripper-Control-Loop phase (or Phase 4 trial integration if the trial loader needs it). Not in PARITY-06/09/10/11 scope.
- **`/scaled_joint_trajectory_controller/*`, `/forward_{position,velocity,effort}_controller/*`, `/passthrough_trajectory_controller/*`, `/force_mode_controller/*`, `/freedrive_mode_controller/*`, `/tool_contact_controller/*`, `/io_and_status_controller/*`, `/tcp_pose_broadcaster/pose`** — full ros2_control surface from the live aic_eval snapshot. Per HANDOFF D-2 these are NOT mirrored: aic_controller bypasses them and talks to the AIC controller surface directly. Revisit only if a CheatCode-adjacent policy turns out to need one of these directly (audit during Phase 4 trial run).
- **cuMotion IK** — explicitly rejected per HANDOFF D-3 (overkill for single-arm no-collision-aware IK). Revisit only if RMPflow + PyKDL both prove insufficient for the workspace volumes CheatCode commands.
- **ros2_control plugin / hardware_interface adapter** — explicitly rejected per HANDOFF D-2 (don't double up the controller layer). Revisit only if AIC introduces a new policy that requires the ros2_control middleware layer.
- **Cartesian impedance simulation in sim** — D-06 ignores impedance fields per the "controller-side math" principle. If a Phase 4 trial reveals that sim-side impedance simulation is needed for parity (unlikely; aic_controller is supposed to handle impedance), revisit.
- **Off-limit contact discovery automation** — for now D-10 is a one-shot live snapshot. A more automated discovery (parse aic_engine config + auto-tag prims) is post-M1 cleanup work.
- **Tare offset computation in sim** — D-07 leaves `fts_tare_offset` as zero. If aic_controller's tare logic depends on Isaac Sim publishing a non-zero tare (unlikely; tare is conceptually a controller-side computation from /fts_broadcaster/wrench history), revisit.

</deferred>

---

*Phase: 02-controller-loop*
*Context gathered: 2026-05-03 (autonomous M1 mode — `--auto`)*
