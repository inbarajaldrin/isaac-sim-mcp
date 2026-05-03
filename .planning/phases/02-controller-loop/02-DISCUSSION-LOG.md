# Phase 2: Controller Loop - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-03
**Phase:** 02-controller-loop
**Mode:** `--auto` (autonomous M1; Claude picks recommended defaults per HANDOFF.json autonomous-mode commitment)
**Areas discussed:** Subscriber transport, IK strategy, Off-limit contacts source, MCP atom decomposition, Custom message types Python path, Impedance/stiffness handling, ControllerState publish semantics, Tick rate / spin model, Joint-name mapping, Off-limit prim configuration, Failure modes, Verification strategy

---

## Subscriber transport mechanism (D-01)

| Option | Description | Selected |
|--------|-------------|----------|
| rclpy-class on physics-tick (mirror Phase 1 inverted) | Build `controller_loop.py` AicControllerLoop class; one rclpy node with 2 subs + 2 pubs; physics-tick callback drives spin_once; mirror parity_publishers.py architecture. Reuses Phase 1 venv-activate + sys.path swap. | ✓ |
| OGN ROS2Subscriber generic node | Use a generic OmniGraph subscriber for the custom AIC types. Untested; requires custom-type code generation. | |
| Separate ROS spin thread | Run rclpy spin in its own thread with command queue. More memory model complexity; GIL contention with Kit's main loop. | |

**Selected (auto-recommended):** rclpy-class on physics-tick.
**Rationale:** Custom AIC message types rule out OGN's stock subscribers. Phase 1's pivot precedent (D-10/D-11 in 01-CONTEXT.md) applies inversely here. Reuse proven infrastructure.

---

## IK strategy for /aic_controller/pose_commands (D-02)

| Option | Description | Selected |
|--------|-------------|----------|
| Lula RMPflow first, PyKDL fallback | Lula bundled in `isaacsim.robot_motion.lula`; native UR5e support; PyKDL universal fallback. | ✓ |
| PyKDL only | Universal but less Isaac-Sim-native; loses Lula's smoothing benefits. | |
| cuMotion | Overkill for single-arm no-collision-aware IK; rejected per HANDOFF D-3. | |
| scipy / numpy custom | Inappropriate; reinventing the IK wheel. | |

**Selected (HANDOFF D-3 lock):** Lula RMPflow first, PyKDL fallback.
**Rationale:** aic_controller does the impedance math; sim just resolves pose to joints. Simplest IK that works.

---

## Off-limit contact-event source (D-03)

| Option | Description | Selected |
|--------|-------------|----------|
| omni.physx contact-report subscription | Per `robot-collision-forensics` skill; verified working in Isaac Sim 5.0; per-link attribution; physics-step ground truth. | ✓ |
| isaacsim.sensors.physics.ContactSensor | Documented broken in Isaac Sim 5.0 — `is_valid=False` per the skill's "open-problem section". | |
| trimesh-based proximity check | Geometric only; doesn't reflect physics-step contact ground truth; misses dynamic resolved contacts. | |

**Selected (auto-recommended):** omni.physx contact-report subscription.
**Rationale:** The `robot-collision-forensics` skill was written specifically because contact attribution in Isaac Sim 5.0 is non-obvious and ContactSensor is broken. Following the skill avoids the known failure path.

---

## MCP atom decomposition (D-04)

| Option | Description | Selected |
|--------|-------------|----------|
| 2 atoms — setup_controller_subscribers + setup_offlimit_contacts | Clean two-domain split (rclpy node vs omni.physx contact-report); 4-surface contract per atom. | ✓ |
| 1 mega-atom (setup_controller_loop) | Violates atomic-tool model; harder to test and toggle independently. | |
| 4 separate atoms (one per topic) | Over-decomposed; 3 of 4 ROS topics share an rclpy node; splitting them apart pre-materially. | |

**Selected (auto-recommended):** 2 atoms.
**Rationale:** Matches the two natural infrastructure layers. DX-02 4-surface contract per atom; both slot into quick_start cleanly per DX-03 D-12 chain.

---

## Custom AIC message types — Python import path (D-05)

| Option | Description | Selected |
|--------|-------------|----------|
| PYTHONPATH-link to ~/Documents/aic/.pixi/envs/default/lib/python*/site-packages | AIC pixi env already builds aic_control_interfaces; no duplication; AIC remains source of truth per CLAUDE.md read-only-consumer rule. | ✓ |
| Vendor message definitions into this repo | Duplicates source-of-truth; requires running rosidl in this repo; maintenance burden. | |
| Build alongside ros2_humble_ws (workspace-link) | More complex setup; keeps separation clean; redundant if pixi env path works. | |

**Selected (auto-recommended):** PYTHONPATH-link to AIC pixi env.
**Rationale:** AIC pixi env is the canonical source. Failure-mode handled gracefully (skip controller-loop atoms with clear error if env missing). Aligns with CLAUDE.md's "AIC repo consumed read-only" rule.

---

## Impedance / stiffness / feedforward field handling (D-06)

| Option | Description | Selected |
|--------|-------------|----------|
| Apply per-joint stiffness/damping/feedforward_torque; ignore Cartesian impedance + wrench-feedback | JointMotionUpdate per-joint fields → Articulation.set_gains() + effort. MotionUpdate Cartesian fields are controller-side math; logging only. | ✓ |
| Ignore all impedance fields, position commands only | Loses fidelity for the per-joint stiffness/damping nuance from JointMotionUpdate. | |
| Implement full impedance simulation in sim | Overkill — that's aic_controller's job; double-applying corrupts controller intent. | |

**Selected (auto-recommended):** Apply per-joint, ignore Cartesian.
**Rationale:** Per-joint fields map naturally to Isaac Sim's articulation drive params. Cartesian fields are pre-computed by aic_controller's impedance math; sim shouldn't re-apply them.

---

## ControllerState publish semantics (D-07)

| Option | Description | Selected |
|--------|-------------|----------|
| Sim populates measured/actual fields (tcp_pose, velocity, error); leaves reference + tare zero/passthrough | tcp_pose from FK (Articulation.get_world_poses for gripper/tcp); tcp_velocity numerical-diff; reference fields echo last-received command; fts_tare_offset zero (aic_controller computes tare from /fts_broadcaster/wrench). | ✓ |
| Sim computes everything including tare | Overreach; tare is conceptually controller-side. | |
| Sim leaves all fields zero (echo-only) | Doesn't satisfy PARITY-11 — controller needs real tcp_pose feedback. | |

**Selected (auto-recommended):** Measured/actual populated, reference/tare echo or zero.
**Rationale:** Sim is the hardware. Hardware reports what it measures, not what the controller computes. PARITY-11 cross-check vs live aic_eval container before final close.

---

## Tick rate / spin model (D-08)

| Option | Description | Selected |
|--------|-------------|----------|
| Physics-tick callback (~60-120 Hz) for everything | Same callback Phase 1 uses; rclpy.spin_once(node, timeout_sec=0) per tick; contact-report callback batches into per-tick Contacts. | ✓ |
| Separate ROS spin thread with command queue | More memory model complexity; GIL contention with Kit main loop. | |
| OGN-graph-only | Rejected — custom message types incompatible with OGN. | |

**Selected (auto-recommended):** Physics-tick callback.
**Rationale:** Mirror Phase 1 architecture. Contact events fire at physics rate, so naturally aligned. Single thread keeps debugging simple.

---

## Joint-name mapping for incoming joint_commands (D-09)

| Option | Description | Selected |
|--------|-------------|----------|
| Name-keyed input parser (mirror Phase 1 output mapper inverted) | Parse JointMotionUpdate.target_state.joint_names against Articulation.dof_names; gripper/left_finger_joint→FixedJoint no-op; unknowns logged + dropped. | ✓ |
| Assume identical ordering, position-indexed | Brittle if AIC reorders. | |

**Selected (auto-recommended):** Name-keyed parser.
**Rationale:** Mirrors Phase 1 D-11 invariant (gripper/left_finger_joint slashed handling); robust to ordering changes.

---

## OFF_LIMIT_PRIMS configuration source (D-10)

| Option | Description | Selected |
|--------|-------------|----------|
| Hardcoded set in controller_loop.py + parameterized via setup_offlimit_contacts atom signature; discovered via live aic_eval snapshot | Empirical-from-live-container principle (mirrors Phase 1 D-01). Atom accepts prim_paths override for callers. | ✓ |
| Read from a separate config file at extension start | Adds config plumbing; one more file to maintain. | |
| Auto-tag prims with naming convention | Requires upstream USD edits to encode off-limit-ness; out of D-06 in-place edit scope. | |

**Selected (auto-recommended):** Hardcoded set + atom-signature override + live-snapshot discovery.
**Rationale:** Same empirical principle that drove Phase 1's parity reference.

---

## Failure modes (D-11)

| Option | Description | Selected |
|--------|-------------|----------|
| Drop bad commands silently with debug log; track _last_good_command_time for diagnostics | Permissive sim; controller's job to recover. | ✓ |
| Raise exception into physics callback | Crashes sim on bad command; makes debugging worse. | |
| Apply best-effort partial command | Risk of partial-state ambiguity; harder to reason about. | |

**Selected (auto-recommended):** Drop silently + track timestamp.
**Rationale:** Sim should be permissive. Controller bugs shouldn't take down the simulator.

---

## Verification strategy (D-12)

| Option | Description | Selected |
|--------|-------------|----------|
| Smoke test analog of Phase 1's smoke_test_aic_parity.py | 7-step Python 3.10 rclpy verifier: connect → publish JointMotionUpdate → verify articulation moved → publish MotionUpdate → verify EE moved → verify ControllerState non-zero → drive into off-limit → verify Contacts non-empty. Closes on N/N pass. | ✓ |
| Manual UI-driven verification only | Not reproducible; can't be re-run as evidence. | |
| Full integration test against live aic_controller | Phase 4 work; Phase 2 closure needs a self-contained smoke test. | |

**Selected (auto-recommended):** Smoke test analog.
**Rationale:** Mirror Phase 1 closure pattern. Reproducible, evidence-based, runs against the live extension.

---

## Claude's Discretion

The following were left to downstream agents (researcher / planner / executor):

- Exact rclpy node name (suggest `aic_dt_controller_loop`).
- Numerical-differentiation buffer size for tcp_velocity (suggest 3-sample, 1/dt scaling).
- Lula RMPflow config file path (suggest `exts/aic-dt/assets/robot/rmpflow_config.yaml`; researcher locates / writes from URDF).
- Off-limit prim path discovery script (D-10 details — researcher writes the snapshot script).
- Exact ROS QoS profiles per topic (suggest match Phase 1 — RELIABLE/VOLATILE for cmd, TRANSIENT_LOCAL only when needed).
- TrajectoryGenerationMode enum interpretation (researcher reads aic_controller source for operational semantics).

## Deferred Ideas

Captured in `02-CONTEXT.md` `<deferred>` section. Notable: full ros2_control surface, gripper command surface (separate phase), cuMotion IK, ros2_control plugin layer, Cartesian impedance simulation, automated off-limit prim discovery, sim-side tare computation. All explicitly punted to post-M1 work or to Phase 4 integration verification.
