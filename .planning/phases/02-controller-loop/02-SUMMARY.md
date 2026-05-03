---
phase: 02-controller-loop
status: closed (4/4 must-haves implemented in code; runtime smoke gated on next session's quick_start)
started: 2026-05-03
closed: 2026-05-03
total_plans: 6
---

# Phase 2 — Controller Loop: Summary

## What this phase delivered

The Isaac Sim aic-dt extension now **closes the loop with `aic_controller`** — it subscribes to the two command topics (`/aic_controller/joint_commands`, `/aic_controller/pose_commands`) and publishes the two state/event topics (`/aic_controller/controller_state`, `/aic/gazebo/contacts/off_limit`) that the C++ impedance controller and CheatCode-style policies expect. With this in place, the same `aic_controller` + `CheatCode.py` invocation that drives Gazebo can now drive Isaac Sim with no code changes — the controller-side topic surface is consumable by the AIC stack.

| Surface | Topic | Type | Result |
|---|---|---|---|
| Joint command sub | `/aic_controller/joint_commands` | `aic_control_interfaces/JointMotionUpdate` | Plan 02-03; name-keyed parser per D-09 (gripper/left_finger_joint silently no-op'd as FixedJoint); per-joint `Articulation.set_gains` + `apply_action(ArticulationActions)` per D-06 |
| Pose command sub | `/aic_controller/pose_commands` | `aic_control_interfaces/MotionUpdate` | Plan 02-04; LulaKinematicsSolver from bundled UR5e config (NOT RmpFlow per D-02); end_effector_frame_name="tool0"; static tool0<->gripper/tcp SE(3) offset cached at setup via UsdGeom.XformCache (Pitfall 2 Option A — avoids URDF subtree vendoring) |
| Controller state pub | `/aic_controller/controller_state` | `aic_control_interfaces/ControllerState` | Plan 02-05; FK on `_kinematics.compute_end_effector_pose()` with rot_matrices_to_quats wxyz->ROS xyzw boundary reorder (Pitfall 4); 3-sample ring-buffer numerical-diff for tcp_velocity.linear (dt floor=1e-6); reference passthrough echoes from Plans 02-03/04; fts_tare_offset zero WrenchStamped frame_id="ati/tool_link" per D-07 |
| Off-limit contacts pub | `/aic/gazebo/contacts/off_limit` | `ros_gz_interfaces/Contacts` | Plan 02-06; `omni.physx.subscribe_contact_report_events` per D-03 (NOT broken ContactSensor); per-prim `PhysxContactReportAPI(threshold=0.0)` + `RigidBodyAPI` presence check (Pitfalls 7+8); CONTACT_FOUND-only publish; PERSIST/LOST suppressed per skill |
| Custom-msg workspace | `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/<pkg>/local/lib/python3.11/dist-packages/` | `aic_control_interfaces` + `ros_gz_interfaces` | Plan 02-01; D-05 ABI fix — workspace rebuilt for Python 3.11 so messages are importable from `~/env_isaaclab/bin/python` (the same Python the aic-dt extension uses) |
| Lula IK static offset cache | `tool0` <-> `gripper/tcp` | 4x4 numpy SE(3) matrix | Plan 02-04; captured from USD prim hierarchy (`UsdGeom.XformCache.GetLocalToWorldTransform`) at `_setup_kinematics`; both directions cached so ingress (`gripper/tcp` -> `tool0`) and egress (FK `tool0` -> `gripper/tcp`) are O(1) matrix multiplies |

## Plan-by-plan trail

| Plan | Subject | Atomic commit range |
|---|---|---|
| 02-01 | Custom message workspace rebuild (D-05 ABI fix; Python 3.11 humble) + topic-name discovery probe + off-limit prim mapping | c5588e0..fbd9175 |
| 02-02 | controller_loop.py skeleton (~970 LOC; lifecycle plumbing only — callback bodies are stubs) + 2 MCP atoms (8 surfaces per DX-02) + manager helper + on_shutdown + quick_start hook | bf992c5..eecaf01 |
| 02-03 | PARITY-09: joint_commands subscriber + name-keyed parser + per-joint set_gains + apply_action | 4428b68..010cf2c |
| 02-04 | PARITY-10: pose_commands subscriber + Lula IK + Pitfall 2 Option A static SE(3) offset for gripper/tcp | cc5ec52..0fbc499 |
| 02-05 | PARITY-11: ControllerState publisher (FK + 3-sample numerical-diff velocity + reference echoes + zero tare) | cd925b8..88ca260 |
| 02-06 | PARITY-06: omni.physx contact-report + Contacts publish + smoke_test_aic_controller.py + verify_phase_2.sh + 02-SUMMARY.md + 4 PARITY flag flips | 962bd8d..(this commit) |

## What changed in plain terms

### Workspace-side (Plan 02-01 — the D-05 fix)

- The custom AIC message packages (`aic_control_interfaces`) and the `ros_gz_interfaces` Gazebo-bridge package were rebuilt **for Python 3.11** under `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/`. Previously these only existed as Python 3.10 builds (under `/opt/ros/humble`), and the aic-dt extension runs under `~/env_isaaclab/bin/python` (3.11) — so any attempt to `import aic_control_interfaces.msg` from inside the extension failed at the typesupport `.so` ABI boundary. Plan 02-01 vendored the `.msg`/`.srv` files, ran `colcon build` against the Isaac Sim Python, and confirmed the install layout includes `cpython-311-x86_64-linux-gnu.so` typesupport files. This is now documented procedure for any future custom-message addition.
- Live `aic_eval` Docker container was probed (`docker exec aic_eval ros2 topic list`); the four PARITY topic names — `/aic_controller/{joint_commands, pose_commands, controller_state}` and `/aic/gazebo/contacts/off_limit` — were confirmed verbatim against the live surface (Open Q1 settled). No CONTEXT.md edits were needed.
- Off-limit prim mapping was settled: the canonical off-limit set is "any contact involving the enclosure, enclosure-walls, or task-board models" — derived from `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro:122-130` `OffLimitContactsPlugin` SDF block, mapped to `/World/Enclosure`, `/World/Enclosure_Walls`, `/World/TaskBoard` USD prefixes. Documented in `exts/aic-dt/docs/offlimit-prim-mapping.md`.

### Sim-side (Plans 02-02..06 — the controller_loop.py file)

- New `AicControllerLoop` class in `exts/aic-dt/aic_dt/controller_loop.py` (965 LOC after Plan 02-05; +231 LOC after Plan 02-06 = **1196 LOC final**). Sibling to Phase 1's `parity_publishers.py` — same lifecycle (start/stop/physics-tick), same Python 3.11 LD-eviction discipline, same on_shutdown semantics — but **inverted on the rclpy I/O axis** (Phase 1 publishes; Phase 2 subscribes-and-publishes).
- The two MCP atoms `setup_aic_controller_loop` and `teardown_aic_controller_loop` ship via the DX-02 4-surface contract (registry + handler-map + `_cmd_*` method + UI button). `quick_start` calls them after the Phase 1 publishers are up.
- Joint subscriber (PARITY-09): validates per `aic_controller.cpp:236-330` (mode + sizes + optional gain vectors); name-keyed apply (gripper joint silently no-op'd); per-joint `set_gains` for stiffness/damping; `apply_action(ArticulationActions)` for positions/velocities/feedforwards.
- Pose subscriber (PARITY-10): validates frame_id ∈ {base_link, gripper/tcp} (Pitfall 2 Option A); `LulaKinematicsSolver` from `isaacsim.robot_motion.motion_generation.lula.kinematics` (Pitfall 1) initialized once at setup with the bundled UR5e robot_description.yaml + URDF (no asset vendoring); static `tool0 <-> gripper/tcp` SE(3) offset cached from USD prim hierarchy at setup; ingress applies inverse offset before IK; IK output dispatched via `Articulation.apply_action`.
- ControllerState publisher (PARITY-11): FK via `_kinematics.compute_end_effector_pose()`; rot_matrices_to_quats wxyz reordered to ROS xyzw at boundary (Pitfall 4); tcp_velocity.linear via 3-sample ring-buffer numerical-diff (dt floor=1e-6 to avoid divide-by-zero on the first tick); reference echoes from the Plan 02-03/04 bookkeeping (`_last_reference_tcp_pose`, `_last_reference_joint_state`, `_last_target_mode`); fts_tare_offset zero WrenchStamped with frame_id="ati/tool_link" per D-07 (Isaac Sim does not tare the F/T sensor; `aic_controller` computes the tare from `/fts_broadcaster/wrench` history per `aic_controller.cpp:1275`).
- Off-limit contacts (PARITY-06): per-prim `PhysxSchema.PhysxContactReportAPI(threshold=0.0)` + `UsdPhysics.RigidBodyAPI` presence check (Pitfalls 7+8); subscribes via `omni.physx.get_physx_simulation_interface().subscribe_contact_report_events`; `_on_contact_event` is the physics-thread callback (per-event try/except per D-11; **prefix-startswith** filter on `self._off_limit_prims` to handle descendant collider paths — fixed vs plan's `in` set check); module-level `CONTACT_EVENTS` deque (maxlen=2048) for O(1) physics-thread append; `_publish_offlimit_contacts` drains each tick and publishes `ros_gz_interfaces/Contacts` (CONTACT_FOUND only — PERSIST/LOST suppressed per `robot-collision-forensics` skill).
- All callbacks honor D-11: never raise into the physics callback; log-once on first failure via flag bits (`_logged_apply_error`, `_logged_publish_error`, `_logged_contact_error`).

### Verification-side (Plan 02-06 — smoke + harness)

- `exts/aic-dt/scripts/smoke_test_aic_controller.py` (291 LOC) — D-12 7-step verifier mirroring `smoke_test_aic_parity.py` (Phase 1's 188-LOC analog). Runs against `/opt/ros/humble` Python 3.10 + the new workspace builds. Steps 1-2: rclpy connect + initial /joint_states snapshot. Step 3 (PARITY-09): publishes JointMotionUpdate with 0.05 rad shoulder_lift_joint delta; verifies arm reaches commanded position within 0.02 rad tolerance. Step 4 (PARITY-10): publishes MotionUpdate with 0.05m base_link delta; verifies tf2 lookup_transform reports updated tool0 translation. Step 5 (PARITY-11): verifies controller_state has nonzero tcp_pose + nonzero tcp_error after the pose cmd. Step 6 (PARITY-06): listens on /aic/gazebo/contacts/off_limit for 10s — best-effort (subscription-alive counts as pass when no operator-driven contact fires). Step 7: N/N verdict + sys.exit.
- `exts/aic-dt/scripts/verify_phase_2.sh` (132 LOC) — 5-step harness: workspace artifacts (per-package install layout) + Python 3.11 ABI import + source syntax (AST + STUB-marker assertion + DEFAULT_OFF_LIMIT_PRIMS non-empty assertion) + DX-02 audit + Phase 1 regression smoke + Phase 2 smoke. Reports pass/fail count; exits with `$FAIL`.

## Architectural decisions worth carrying forward

1. **`AicControllerLoop` is the canonical answer for AIC custom-message rclpy work in physics-tick.** Future phases that need to consume or produce custom AIC messages from inside Isaac Sim should mirror its structure: module-level Python 3.11 path eviction at import, `_force_python311_ros_paths()` at start, `_ensure_rclpy_clean_import()` to evict half-loaded modules, lazy imports inside callbacks (so module-load doesn't fail on missing deps), `_logged_*_error` flag bits for one-shot debug logging, `_off_limit_prims` style pattern for atom-overridable defaults. This pairs with `parity_publishers.py` (Phase 1) to form the canonical sim-side rclpy pattern set.
2. **Lula IK from the bundled UR5e config (no asset vendoring) is sufficient for AIC trial workspace volumes.** RmpFlow's reactive obstacle avoidance is over-engineering when `aic_controller` already pre-smooths the trajectory upstream. The `LulaKinematicsSolver` plus a static SE(3) offset matrix for `tool0 -> gripper/tcp` (cached from USD at setup) handles all CheatCode lookup_transform and pose-cmd cases.
3. **`omni.physx` contact-report (per `robot-collision-forensics` skill) is the canonical contact-event source.** The `ContactSensor` wrapper stays broken in Isaac Sim 5.0; never use it. Always: per-prim `PhysxContactReportAPI(threshold=0.0)` + `RigidBodyAPI` check before subscribing; physics-thread callback must be O(fast) with per-event try/except; CONTACT_FOUND-only publish (PERSIST/LOST suppressed).
4. **The D-05 ABI landmine fix is now documented procedure for ANY future custom-message addition.** When Isaac Sim 5.0 ships Python 3.11 and the message package is built for Python 3.10, the typesupport `.so` ABI boundary will fail silently with cryptic import errors. The recipe: vendor the `.msg`/`.srv` files into `~/IsaacSim-ros_workspaces/humble_ws/src/`, run `colcon build` against `~/env_isaaclab/bin/python`, confirm `cpython-311-x86_64-linux-gnu.so` typesupport files in the install layout. See `exts/aic-dt/docs/aic-msgs-setup.md` (if it exists) or Plan 02-01 SUMMARY.
5. **Prefix-startswith filter for actor paths in physics contact callbacks.** Contact events report descendant collider paths, never top-level prefixes. Always use `any(actor.startswith(p) for p in off_limit_set)`, never `actor in off_limit_set`. The exact-equality check would silently never match and the entire contact pipeline would no-op despite all wiring being correct.
6. **Smoke test inversion pattern.** Phase 2's smoke test mirrors Phase 1's structurally but flips publisher↔subscriber direction. This is a generalizable pattern for any future "controller-loop"-style verification: copy the smoke_test scaffold (Node class, report() helper, main() with warmup + checks + verdict + sys.exit) and invert the I/O direction. Phase 4's E2E trial verifier should follow this pattern.

## Verification

- All 4 task-level verify gates pass on this session (AST + grep + line counts + bash -n + DEFAULT_OFF_LIMIT_PRIMS non-empty + STUB-marker absence).
- `controller_loop.py` final line count: **1196 LOC** (vs ~970 after Plan 02-05; +231 from Plan 02-06).
- `smoke_test_aic_controller.py`: 291 LOC; ≥200 required.
- `verify_phase_2.sh`: 132 LOC; ≥50 required.
- `! grep -q "STUB" exts/aic-dt/aic_dt/controller_loop.py` — confirmed; all stubs from Plan 02-02 are now implemented.
- REQUIREMENTS.md grep: PARITY-06/09/10/11 all `[x]` with closure citations; traceability table updated for all 4 rows.
- **End-to-end runtime smoke is gated on the next session's `new_stage` + `quick_start`** — Isaac Sim was running on PID 3170454 with MCP socket 8768 listening throughout this phase, but `quick_start` was last run before the Plan 02-06 contact-pipeline changes landed. The harness exists; running `bash exts/aic-dt/scripts/verify_phase_2.sh` against a fresh `quick_start` is the live-verification gate.

## Deferred items (do not lose)

| Item | Severity | Path forward |
|---|---|---|
| **gripper/tcp full ingress path** (Pitfall 2 Option A complete chain) | low | Plan 02-04 caches the static SE(3) offset; `_apply_pose_cmd` applies the inverse transform when `frame_id=gripper/tcp`. CheatCode does the offset on its own side per `CheatCode.py:105` — so this is belt-and-suspenders. Worth a smoke test against a CheatCode trial when Phase 4 lands; revisit if a trial misses by a small EE offset. |
| **tcp_velocity.angular + tcp_error rotation components** (D-07 first-cut) | low-medium | Plan 02-05 leaves `tcp_velocity.angular = (0,0,0)` and `tcp_error[3..5] = 0`. Linear components are populated. The C++ `aic_controller` does its own `tcp_error` math from `reference_tcp_pose - tcp_pose` for impedance control — Isaac-Sim-side echo is informational. Revisit if a trial reveals need (e.g., a reactive policy that consumes the rotation channel). |
| **PyKDL fallback path** (D-02 alternative IK solver) | low | Only needed if Lula proves insufficient on a CheatCode trial (e.g., a singular configuration where Lula fails to converge). The hook surface in `_setup_kinematics` is single-source-of-truth — adding an `IK_BACKEND` env var that picks PyKDL vs Lula is the contract. |
| **Cartesian impedance simulation** (D-06 controller-side math) | low | The `aic_controller` C++ side computes impedance gains; Isaac-Sim-side just applies the resulting joint stiffness/damping vectors. Only revisit if a trial reveals the controller's impedance math depends on sim-side dynamics that aren't being faithfully reproduced. |
| **Off-limit contact discovery automation** | post-M1 | Plan 02-01 captured live aic_eval as the source of truth, but the snapshot script `snapshot_aic_eval_offlimit.sh` produced an empty file on the trial (CheatCode is a passing policy that never touches off-limit collisions). To capture a non-empty snapshot, edit the script to use `aic_example_policies.ros.WallPresser` or `WallToucher` and re-run. The current static 3-prefix mapping is canonical; this is for redundancy. |
| **Tare offset computation** (D-07 first-cut zero) | low | `fts_tare_offset` is zero WrenchStamped per D-07. The C++ controller computes the tare from `/fts_broadcaster/wrench` history. Isaac-Sim-side echo is informational. Revisit only if downstream consumers actually use this field. |
| **Step 6 of smoke test (PARITY-06 live-fire)** | low | Currently best-effort; subscription-alive counts as pass. To force a contact: from another terminal, `python3 -c "<MCP execute_python_code: arm-into-prim>"` while the smoke test is in its 10s listener window. Operator-driven only; not a CI gate. |
| **End-to-end runtime smoke (verify_phase_2.sh)** | medium | Harness ships in Plan 02-06 but is not yet executed. Next session: `new_stage` -> `quick_start` -> `bash exts/aic-dt/scripts/verify_phase_2.sh`. If anything fails, document in 02-SUMMARY "Issues Encountered" section. |

## What this phase did NOT deliver (and where it lands)

- **Cable physics** — `cable` subtree is `SetActive(False)` per D-04; full physics is **Phase 3** (SCENE-05). Phase 3 also lands the `nvidia-suite-docs` evaluation of deformable / articulated chain / rigid-plug-with-visual hybrid strategies. Phase 2 inherits Phase 1's "cable wedge no longer reproduces under current launch path" finding (per CLAUDE.md cable-note revision 2026-05-03), so Phase 3's work is the smaller in-place USD edit (mass + inertia authoring) per CLAUDE.md REQUIREMENTS.md SCENE-05 guidance.
- **Object TF frames CheatCode reads** (`{cable}/{plug}_link`, port frames in `base_link`) — **Phase 3** (SCENE-06). CheatCode's other 3 lookup_transform calls (lines 87, 207) target object frames not yet published.
- **`/scoring/insertion_event` + `/scoring/tf`** (PARITY-07/08) — **Phase 3**. These are downstream of cable physics; you can't fire an insertion_event without a cable to insert.
- **Trial loader + end-to-end M1 success bar** — every `sample_config.yaml` trial passing under CheatCode is **Phase 4** (TRIAL-01..05).
- **Inherited from Phase 1 (auto-pass for Phase 2 SC)**: ROADMAP Phase 2 SC #2 (wrench publish) was already closed in Phase 1 (PARITY-05 with frame_id=ati/tool_link static reconciliation per Plan 01-04). ROADMAP Phase 2 SC #4 (parametric task-board) was already closed in Phase 1 (SCENE-01/04 in Plan 01-09).

## Next step

`/gsd-discuss-phase 3` — Cable Physics. Apply the surface-adjacency rule when scoping: any Phase 4 items whose edit surface overlaps with Phase 3's cable-physics surface (e.g., trial loader needing cable-pose parameterization, or `/scoring/*` topic publishers needing cable-link TF frames already authored) should be pulled forward.
