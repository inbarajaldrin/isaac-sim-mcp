# Phase 2: Controller Loop — Research

**Researched:** 2026-05-03
**Domain:** rclpy subscriber/publisher loop in physics-tick + Lula RMPflow IK + omni.physx contact-report
**Confidence:** HIGH (all critical APIs verified against Isaac Sim 5.0 install on this host; one HIGH-IMPACT landmine identified in D-05 that requires plan-time correction)

## Summary

The 13 locked decisions (D-01..D-13) in 02-CONTEXT.md describe a complete architecture: an `AicControllerLoop` rclpy class mirrored from `parity_publishers.py` (inverted), Lula RMPflow for IK with PyKDL fallback, the `omni.physx` contact-report subscription pattern (per `robot-collision-forensics` skill) for off-limit contacts, two new MCP atoms (`setup_controller_subscribers` + `setup_offlimit_contacts`), and a 7-step Python-3.10 smoke test analog of `smoke_test_aic_parity.py`.

This research surfaces the implementation-level details the planner needs:

1. **Exact APIs are now nailed down** — `Articulation.apply_action(ArticulationActions(joint_positions=..., joint_efforts=...))` and `Articulation.set_gains(kps, kds)` signatures verified, both with empty-vs-None semantics for partial gain updates. Lula RMPflow lives at `isaacsim.robot_motion.motion_generation.lula.RmpFlow`; UR5e templates ship at `motion_policy_configs/universal_robots/ur5e/rmpflow/{ur5e_robot_description.yaml,ur5e_rmpflow_config.yaml}` and `motion_policy_configs/universal_robots/ur5e/ur5e.urdf`. The Phase 1 articulation handle lives at `/World/UR5e/aic_unified_robot/root_joint`.

2. **One CRITICAL landmine in D-05** (PYTHONPATH-link to AIC pixi env): the pixi env builds `aic_control_interfaces` against **Python 3.12** (compiled `.so` files in `lib/python3.12/site-packages/`), but Isaac Sim runs **Python 3.11**. C-extension typesupport `.so` files are not ABI-compatible across CPython minor versions. Verified empirically: `~/env_isaaclab/bin/python` (3.11) cannot load the pixi env's modules — even numpy fails to import. **D-05 as written will fail at first message-publish attempt.** The planner MUST replace the PYTHONPATH-link strategy with: build `aic_control_interfaces` from source against Python 3.11 in the existing IsaacSim-ros_workspaces workspace at `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/`. The package has zero non-stock dependencies (`builtin_interfaces`, `geometry_msgs`, `std_msgs`, `trajectory_msgs`) so it builds cleanly under Humble despite originally targeting Kilted.

3. **`ros_gz_interfaces` (for PARITY-06) needs the same treatment** — it's not in the Python 3.11 workspace either; the only build is at `/opt/ros/humble/local/lib/python3.10/dist-packages/ros_gz_interfaces/` with cpython-310-tagged `.so` files. Same fix: vendor the ros_gz_interfaces source into `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/` and rebuild.

4. **The aic_controller architecture clarification matters** — `aic_controller` is a C++ ros2_control plugin (in `~/Documents/aic/aic_controller/src/aic_controller.cpp`) that subscribes to `~/joint_commands` and `~/pose_commands` (resolved namespaces: `/aic_controller/joint_commands`, `/aic_controller/pose_commands`) and publishes `~/controller_state` at controller_manager update rate (`control_frequency` config param, default 500 Hz per Phase 1's `aic_ros2_controllers.yaml`). In Gazebo, the controller writes `effort` (Impedance mode) or `position` (Admittance mode) to ros2_control hardware_interface; gazebo_ros2_control then actuates joints. **In Isaac Sim, we are SKIPPING the ros2_control + hardware_interface layer entirely** — Isaac Sim subscribes directly to the same topics aic_controller would consume, applies them to articulation drives, and publishes the controller_state topic that aic_controller would publish. This is the HANDOFF D-2 architecture ("aic_controller IS the controller; Isaac Sim is the hardware sink"). It also means **only ONE party publishes `/aic_controller/controller_state`** in any given session: either the live aic_eval container (Gazebo path) OR Isaac Sim (Phase 2 path), never both at once. CheatCode-style policies talk to whichever stack is up.

5. **Drive config conflict to resolve at execution time:** Phase 1's `load_robot` sets `stiffness=2000, damping=100, max_force=87` per joint via `UsdPhysics.DriveAPI.Apply(joint_prim, "angular")` (extension.py:1245). D-06's `Articulation.set_gains()` overrides these at runtime. The two are compatible — USD-applied values are the boot defaults, set_gains overrides them per-tick or per-command. set_gains takes a `(M=1, K=num_dof)` shape array; for the 6-DOF UR5e arm articulation that's `(1, 6)`.

6. **Contact-report pattern is fully spec'd in `robot-collision-forensics`** — drop-in code at `~/.claude/skills/robot-collision-forensics/scripts/contact_subscription.py` (185 LOC, ready to import). Apply `PhysxSchema.PhysxContactReportAPI` with `threshold=0.0` to every off-limit prim that has `RigidBodyAPI`, then `omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(callback)`. Callback runs on the physics thread; resolve int actor IDs via `PhysicsSchemaTools.intToSdfPath`. Each contact event provides `actor0`, `actor1`, `collider0`, `collider1`, `impulse[3]`, `position[3]`, `n_contacts`, `type` (`CONTACT_FOUND`/`CONTACT_PERSIST`/`CONTACT_LOST`).

**Primary recommendation:** Plan Phase 2 in 6 plans matching the surface-adjacency natural breaks: (1) workspace rebuild for `aic_control_interfaces` + `ros_gz_interfaces` (D-05 fix), (2) `controller_loop.py` skeleton + 2 atoms + handler-map + UI buttons (D-01/D-04 surface), (3) joint-command application via Articulation+set_gains (D-06/D-09/D-11 + PARITY-09), (4) Lula RMPflow IK for pose-command application (D-02 + PARITY-10), (5) ControllerState publish via FK (D-07 + PARITY-11), (6) omni.physx contact-report + Contacts publish + smoke test (D-03/D-10/D-12 + PARITY-06).

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| PARITY-06 | Publish `/aic/gazebo/contacts/off_limit` (`ros_gz_interfaces/Contacts`) on off-limit contact events | "Off-Limit Contact Pipeline" + "ros_gz_interfaces" sections; drop-in from robot-collision-forensics skill; ros_gz_interfaces.msg.Contacts/Contact constructor verified |
| PARITY-09 | Subscribe to `/aic_controller/joint_commands` (`aic_control_interfaces/JointMotionUpdate`) and apply to UR5e | "Joint Command Application" section: Articulation.apply_action signature, ArticulationActions construction with name-keyed mapping, set_gains for impedance fields |
| PARITY-10 | Subscribe to `/aic_controller/pose_commands` (`aic_control_interfaces/MotionUpdate`) and apply to UR5e EE | "Lula RMPflow / IK Pipeline" section: ArticulationKinematicsSolver.compute_inverse_kinematics → ArticulationAction → Articulation.apply_action |
| PARITY-11 | Publish/subscribe `/aic_controller/controller_state` (`aic_control_interfaces/ControllerState`) | "ControllerState Publisher" section: FK via ArticulationKinematicsSolver.compute_end_effector_pose, numerical-diff for tcp_velocity, fts_tare_offset zero-init per D-07 |

## Project Constraints (from CLAUDE.md)

- **NVIDIA-suite-docs skill is canonical** for Isaac Sim 5.0 / OpenUSD / OmniGraph / Lula / omni.physx APIs — consult before guessing from training data, even mid-execution. Cached LLM knowledge is routinely stale for the NVIDIA stack.
- **isaac-sim-extension-dev skill is canonical** for project-specific Isaac Sim extension patterns (extension lifecycle, MCP socket protocol, cache management, prim-path bug history, the venv-activate launch requirement).
- **robot-collision-forensics skill is the AUTHORITATIVE reference for D-03** (`omni.physx` contact-report pipeline + `ContactSensor.is_valid=False` broken-state in Isaac Sim 5.0). The skill ships a drop-in `scripts/contact_subscription.py` (185 LOC). Do NOT use `isaacsim.sensors.physics.ContactSensor` — broken in 5.0.
- **MCP socket protocol on port 8768.**
- **venv-activate before launch is mandatory** (`source ~/env_isaaclab/bin/activate`) — Phase 2 extends this requirement to add `~/Documents/aic/.pixi/envs/default/lib` to `LD_LIBRARY_PATH` for ros-kilted shared libs (D-13). Note: this is for AIC-side C++ libs that aic_controller runtime depends on, NOT for the Python `aic_control_interfaces` package (which is the D-05 landmine).
- **Topic-parity is the architectural law** — no `_sim`/`_real` suffixes, no bridges, no remap nodes. All four PARITY-06/09/10/11 topics use the exact names the live `aic_eval` Docker container publishes.
- **rclpy in non-control glue is in-policy** (per Phase 1 closure) — Phase 2 extends this pattern to control-side topics.
- **`omni.*` imports stay out of `~/Documents/aic`** — but `rclpy` inside aic-dt is OK.
- **Hot-reload-safe extension state** — `on_shutdown` cleanly tears down rclpy node + omni.physx subscription handles; new code mirrors the parity_publishers.py teardown discipline.

## User Constraints (from CONTEXT.md)

### Locked Decisions (D-01..D-13)

D-01..D-13 from `02-CONTEXT.md` are locked. Summary (full text in CONTEXT.md):
- **D-01:** Build `controller_loop.py` with `AicControllerLoop` class — mirror `parity_publishers.py` inverted. One rclpy node, 2 subs + 2 pubs, on physics-tick callback.
- **D-02:** Lula RMPflow first (`isaacsim.robot_motion.lula` per CONTEXT, but actual import path is `isaacsim.robot_motion.motion_generation.lula` — see Pitfall #1), PyKDL fallback. NO cuMotion.
- **D-03:** Off-limit contacts via `omni.physx.acquire_contact_report_event_stream` per `robot-collision-forensics` skill. NOT ContactSensor.
- **D-04:** 2 MCP atoms — `setup_controller_subscribers` + `setup_offlimit_contacts`. 4-surface contract per atom.
- **D-05:** PYTHONPATH-link to AIC pixi env. **⚠️ THIS IS BROKEN AS WRITTEN — see "D-05 Landmine" section below.**
- **D-06:** Apply per-joint stiffness/damping/feedforward_torque to articulation drives via `Articulation.set_gains()` + effort. Cartesian impedance fields = log + ignore.
- **D-07:** ControllerState publishes measured fields (tcp_pose via FK, tcp_velocity via numerical-diff). Reference fields echo last command. fts_tare_offset = zero-init.
- **D-08:** Physics-tick callback for everything; rclpy.spin_once(node, timeout_sec=0) per tick.
- **D-09:** Name-keyed joint parser; gripper/left_finger_joint (slashed) is FixedJoint no-op.
- **D-10:** Hardcoded + atom-overridable off-limit prim filter; discovery via live aic_eval snapshot.
- **D-11:** Drop bad commands silently with debug log; never raise into physics callback.
- **D-12:** Smoke test analog of Phase 1's `smoke_test_aic_parity.py` (7-step Python 3.10 rclpy verifier).
- **D-13:** LD_LIBRARY_PATH extends venv-activate to include `~/Documents/aic/.pixi/envs/default`.

### Claude's Discretion (per CONTEXT.md)

- Exact rclpy node name (suggest `aic_dt_controller_loop` mirror of `aic_dt_parity_publisher`).
- Numerical-differentiation buffer size for tcp_velocity (suggest 3-sample, 1/dt scaling).
- Lula RMPflow config path inside the repo — research recommendation: **DO NOT vendor RMPflow YAML/URDF into this repo**. Use Isaac Sim's bundled UR5e templates directly. If divergence becomes necessary later (e.g., tighter joint limits for the AIC enclosure), copy the templates into `exts/aic-dt/assets/robot/rmpflow/` and edit in place per the D-06 vendoring discipline.
- Off-limit prim path discovery script wording.
- ROS QoS profiles per topic — research recommendation: match `aic_controller` C++ source: `RELIABLE / KEEP_LAST(10)` for joint_commands and pose_commands (line 182), `SystemDefaultsQoS` (= RELIABLE / VOLATILE / KEEP_LAST(10)) for controller_state (line 689).

### Deferred Ideas (OUT OF SCOPE)

- `/gripper_command` (String) and gripper status/width topics — gripper control surface is separate from arm-control surface. Future Gripper-Control-Loop phase or Phase 4.
- Full ros2_control surface (`/scaled_joint_trajectory_controller/*`, `/forward_*_controller/*`, `/passthrough_trajectory_controller/*`, `/force_mode_controller/*`, `/freedrive_mode_controller/*`, `/tool_contact_controller/*`, `/io_and_status_controller/*`, `/tcp_pose_broadcaster/pose`) — explicitly rejected per HANDOFF D-2.
- cuMotion IK — explicitly rejected per HANDOFF D-3.
- ros2_control plugin / hardware_interface adapter — explicitly rejected per HANDOFF D-2.
- Cartesian impedance simulation in sim (D-06 ignores).
- Off-limit contact discovery automation (post-M1 cleanup).
- Tare offset computation in sim (D-07 leaves zero).

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|--------------|----------------|-----------|
| Custom AIC message deserialization | Isaac Sim Python 3.11 venv | ROS Humble workspace (rebuild target) | rclpy needs Python 3.11 ABI typesupport `.so`; pixi env's Python 3.12 ABI is incompatible (D-05 landmine) |
| `JointMotionUpdate` → articulation positions | Articulation (PhysX joint drives) | Articulation `set_gains` for stiffness/damping fields | Drive PD parameters live in PhysX; per-tick override via runtime API |
| `MotionUpdate` (Cartesian pose) → joint positions | Lula RMPflow / Lula IK in Isaac Sim Python | Articulation.apply_action consumes IK result | RMPflow ships pre-tuned UR5e config; bundled in `isaacsim.robot_motion.motion_generation` |
| `ControllerState` publish (measured fields) | rclpy Python publisher in physics-tick | ArticulationKinematicsSolver for FK | tcp_pose computed via Lula FK is consistent with the IK solver's frame definitions |
| Off-limit contact detection | omni.physx contact-report event stream | PhysxSchema.PhysxContactReportAPI on each off-limit prim | Per-physics-step ground truth; `ContactSensor` wrapper broken in 5.0 |
| Off-limit contact → ros_gz_interfaces/Contacts publish | rclpy Python publisher in physics-tick | Same node as the controller-state publisher | One rclpy node owns all 4 topics, simplifies lifecycle |
| Lifecycle: extension startup, MCP atom invocation, physics-tick spin, MCP atom teardown, on_shutdown cleanup | aic-dt extension Python | mirror parity_publishers.py exactly | Battle-tested in Phase 1 with Gap A/B inline fix |

## Standard Stack

### Core (already-installed, verified on this host)

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `isaacsim.core.prims.SingleArticulation` (= `Articulation`) | Isaac Sim 5.0 | Per-DOF position/effort/gain control | The exact handle Phase 1's `load_robot` already creates and stores at `self._articulation` (extension.py:1233) |
| `isaacsim.core.prims.Articulation` (= `ArticulationView`) | Isaac Sim 5.0 | Multi-env articulation queries (Phase 1 reads via `self._robot_view`) | Already cached in `self._robot_view` (extension.py:1228) and used by Phase 1's force publisher |
| `isaacsim.core.utils.types.ArticulationActions` | Isaac Sim 5.0 | Container for joint_positions/velocities/efforts/joint_names/joint_indices to feed apply_action | Idiomatic 5.0 control API; replaces deprecated `ArticulationAction` (single-env) where ArticulationView is in play |
| `isaacsim.robot_motion.motion_generation.lula.RmpFlow` | Isaac Sim 5.0 | RMPflow motion policy (config-driven IK + reactive avoidance) | Bundled UR5e config pre-tuned by NVIDIA |
| `isaacsim.robot_motion.motion_generation.lula.LulaKinematicsSolver` | Isaac Sim 5.0 | Pure IK (no motion-policy overhead) — fallback when RMPflow over-engineers | Same lula::RobotDescription backbone as RmpFlow |
| `isaacsim.robot_motion.motion_generation.ArticulationKinematicsSolver` | Isaac Sim 5.0 | Wrapper that converts IK results into ArticulationAction; FK via `compute_end_effector_pose` | Bridges Lula → Articulation cleanly; **also gives us the tcp_pose for ControllerState (D-07)** |
| `omni.physx.get_physx_simulation_interface()` + `.subscribe_contact_report_events(callback)` | omni.physx 5.x | Per-physics-step contact event stream; the source of truth for D-03 | Documented in `robot-collision-forensics/SKILL.md` and shipped as drop-in code |
| `pxr.PhysxSchema.PhysxContactReportAPI` + `pxr.PhysicsSchemaTools.intToSdfPath` | OpenUSD / omni.physx | Apply contact-report flag per prim + resolve int actor IDs to SdfPath strings | The two USD-side primitives the contact callback depends on |
| `omni.physx.get_physx_interface().subscribe_physics_step_events(callback)` | omni.physx 5.x | Per-tick callback for the loop's spin/apply/publish work | Already used by Phase 1's parity_publishers.py:333 and force publisher (extension.py:1393) |
| rclpy 3.3.19 (Python 3.11 build) | ROS Humble (rebuilt for 3.11) | Node, publisher, subscriber, QoS | Phase 1 already stands this up via `IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages` |
| `aic_control_interfaces.msg.{JointMotionUpdate, MotionUpdate, ControllerState, TargetMode, TrajectoryGenerationMode}` | AIC source rebuilt for Python 3.11 | The 5 custom message types | Source: `~/Documents/aic/aic_interfaces/aic_control_interfaces/` — pure rosidl_default_generators package, no Kilted-only deps |
| `ros_gz_interfaces.msg.{Contacts, Contact, Entity, JointWrench}` | ROS Humble rebuilt for Python 3.11 | The Contacts message PARITY-06 publishes | Source: clone https://github.com/gazebosim/ros_gz then vendor `ros_gz_interfaces/` into the workspace |
| `tf2_msgs`, `geometry_msgs`, `std_msgs`, `trajectory_msgs`, `sensor_msgs` | ROS Humble Python 3.11 build | Stock messages used as field types | Already in the workspace per Phase 1 |

### Supporting

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| PyKDL | uninstalled | Backup IK if Lula fails | Per D-02; install only if Lula RMPflow doesn't converge for AIC trial workspace volumes. Verified PyKDL is NOT pre-installed in `env_isaaclab` or in the Python 3.11 ROS workspace. Acquisition path: `pip install` in venv may not work (PyKDL needs a libkdl C++ build); the cleaner path is to defer until a real failure forces it. **Likely never needed** — Lula's UR5e config is well-tested. |
| numpy | already in env_isaaclab | Joint state arrays, IK target poses, FK results | Idiomatic Isaac Sim; all of `Articulation` and `motion_generation` use numpy arrays |

### Alternatives Considered

| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Lula RMPflow | Lula IK alone (`LulaKinematicsSolver`) | Lighter; no RMP reactive avoidance. Recommended for first cut — `MotionUpdate.pose` is a pose target with controller-side smoothing already done. RMPflow's reactive avoidance and trajectory smoothing are redundant when aic_controller pre-smooths the trajectory. **Concrete recommendation: use `LulaKinematicsSolver` + `ArticulationKinematicsSolver`, NOT RmpFlow, for Phase 2 PARITY-10 first cut.** Switch to RmpFlow only if pure IK proves jumpy. |
| `omni.physx` contact-report | scene query "overlap" tests at sample rate | Not ground-truth — would miss CONTACT_FOUND events between samples. The skill explicitly warns against this. |
| `Articulation.apply_action` | Direct USD-side `JointPositionAttr.Set()` per joint | apply_action goes through PhysX physics-view; USD-side writes are NOT reflected in the simulated state during play (USD layer becomes stale post-play). Established Isaac Sim 5.0 idiom. |
| Single rclpy node owning all 4 topics | Two nodes (one for controller, one for contacts) | More moving parts. Phase 1 used one node for /joint_states + /tf + /tf_static; same pattern. |

### Installation / build steps for the missing packages

```bash
# 1. Clone ros_gz source for ros_gz_interfaces (Humble compatible — the Humble branch is `humble`)
cd /tmp
git clone -b humble https://github.com/gazebosim/ros_gz.git
# 2. Copy ros_gz_interfaces into the IsaacSim-ros_workspaces source tree
cp -r /tmp/ros_gz/ros_gz_interfaces ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
# 3. Copy aic_control_interfaces into the same workspace
cp -r ~/Documents/aic/aic_interfaces/aic_control_interfaces ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
# 4. Rebuild via the IsaacSim-ros_workspaces docker harness (per exts/aic-dt/docs/rclpy-setup.md)
cd ~/IsaacSim-ros_workspaces
bash build_ros.sh -d humble -v 22.04
# 5. Verify .so files are 3.11-tagged or untagged (NOT cpython-310-tagged)
ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/*.so
ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/ros_gz_interfaces/*.so
```

**Version verification:**
- Isaac Sim 5.0 install path: `~/env_isaaclab/lib/python3.11/site-packages/isaacsim/` — verified `[VERIFIED: filesystem inventory 2026-05-03]`.
- `isaacsim.robot_motion.motion_generation` ext present at `~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.robot_motion.motion_generation/` — `[VERIFIED]`.
- UR5e RMPflow config files present at `motion_policy_configs/universal_robots/ur5e/{ur5e.urdf, rmpflow/ur5e_robot_description.yaml, rmpflow/ur5e_rmpflow_config.yaml}` — `[VERIFIED]`.
- ROS Humble Python 3.11 workspace present at `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/` (80 packages) — `[VERIFIED]`.
- AIC pixi env Python version: 3.12 (`~/Documents/aic/.pixi/envs/default/bin/python3.12 --version` → `Python 3.12.12`) — `[VERIFIED]`.

## Architecture Patterns

### System Architecture Diagram

```
                                  Phase 2 surface
                                  ───────────────

  Policy (CheatCode, etc.)  ──pub──▶  /aic_controller/joint_commands  ──┐
                                       (JointMotionUpdate)               │
                            ──pub──▶  /aic_controller/pose_commands  ────┤
                                       (MotionUpdate)                    │
                                                                         │
                                                                         ▼
   ┌───────────────────────────────────────────────────────────────────────────┐
   │  AicControllerLoop (rclpy node aic_dt_controller_loop, in physics-tick)    │
   │  ─────────────────────────────────────────────────────────────────────     │
   │                                                                            │
   │  Per physics-step (~60-120 Hz):                                            │
   │   1. rclpy.spin_once(node, timeout_sec=0)  ── drains pending sub messages   │
   │   2. If joint_commands msg waiting:                                        │
   │        parse → name-key map → set_gains(kps,kds) → apply_action(positions, │
   │                                                                  efforts)  │
   │   3. If pose_commands msg waiting:                                         │
   │        parse pose → ArticulationKinematicsSolver.compute_inverse_kinematics │
   │           → apply_action(IK result)                                        │
   │   4. Read articulation state:                                              │
   │        FK via .compute_end_effector_pose                                    │
   │        velocity via numerical-diff (3-sample ring buffer)                  │
   │        publish ControllerState (measured fields + last-cmd reference)       │
   │   5. Drain omni.physx contact event deque:                                 │
   │        filter to OFF_LIMIT_PRIMS                                            │
   │        if any: build ros_gz_interfaces/Contacts → publish to               │
   │           /aic/gazebo/contacts/off_limit                                    │
   └───────────────────────────────────────────────────────────────────────────┘
                                      │                │
                                      │                │
                  ┌───────────────────┘                └────────────────┐
                  ▼                                                     ▼
   /aic_controller/controller_state              /aic/gazebo/contacts/off_limit
        (ControllerState)                              (ros_gz_interfaces/Contacts)
                  │                                                     │
                  ▼                                                     ▼
   Policy / aic_controller / observer                          Scoring / observer

                                  Side channel
                                  ─────────────

   Articulation drives  ◀── apply_action(positions, efforts)
                        ◀── set_gains(kps, kds)

   PhysxSchema.PhysxContactReportAPI on each /World/TaskBoard/<off_limit>/...
                        │
                        ▼
   omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(_on_contact)
                        │
                        ▼  (callback runs on physics thread, appends to module-level deque)
   CONTACT_EVENTS deque  ──drain── per-tick step 5 above
```

### Recommended Project Structure

```
exts/aic-dt/
├── aic_dt/
│   ├── extension.py              # +2 MCP atoms (registry+handler-map+_cmd+UI button), on_shutdown extension
│   ├── parity_publishers.py      # Phase 1 — left untouched
│   └── controller_loop.py        # NEW — AicControllerLoop class (~600 LOC; mirrors parity_publishers.py inverted)
├── docs/
│   ├── aic-msgs-setup.md         # NEW — D-05 fix instructions: how to add aic_control_interfaces + ros_gz_interfaces to the Python 3.11 workspace
│   ├── controller-loop.md        # NEW — Phase 2 design overview, links to RESEARCH.md sections
│   └── rclpy-setup.md            # Phase 1 — extended with the new packages + the LD_LIBRARY_PATH addition
├── scripts/
│   ├── smoke_test_aic_controller.py  # NEW — 7-step Python 3.10 verifier (D-12 contract)
│   └── snapshot_aic_eval_offlimit.sh # NEW — discovery script for D-10 (one-shot capture of off-limit prim filter from live aic_eval)
└── assets/
    └── (no new assets — Lula UR5e templates used in-place from Isaac Sim install)
```

### Pattern 1: Custom message rebuild for Python 3.11

```bash
# Source: ~/Documents/isaac-sim-mcp/exts/aic-dt/docs/rclpy-setup.md (Phase 1 pattern)
# Add a new package to the IsaacSim-ros_workspaces workspace:
cp -r <upstream_pkg_source> ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
cd ~/IsaacSim-ros_workspaces
bash build_ros.sh -d humble -v 22.04   # rebuilds the whole workspace under Docker
# Verify the new package is built for Python 3.11:
ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/<pkg>/
# Inside Isaac Sim, the parity_publishers.py path-management logic already prepends this path.
```

### Pattern 2: rclpy class registered with omni.physx physics-step events

```python
# Source: exts/aic-dt/aic_dt/parity_publishers.py:225-342 (Phase 1)
# Mirror exactly for AicControllerLoop, swapping subs↔pubs and adding 2 more topics.
class AicControllerLoop:
    def __init__(self, robot_xform_path: str = "/World/UR5e/aic_unified_robot",
                 off_limit_prims: list = None):
        self._robot_xform_path = robot_xform_path
        self._node = None
        # Subs
        self._joint_cmd_sub = None
        self._pose_cmd_sub = None
        # Pubs
        self._ctrl_state_pub = None
        self._contacts_pub = None
        # Physics
        self._physx_sub = None
        self._articulation = None
        self._kinematics_solver = None         # ArticulationKinematicsSolver (Lula-backed)
        # Contact subscription
        self._contact_sub = None               # omni.physx subscription handle
        self._off_limit_prims = set(off_limit_prims or [])
        # Latest command state (modified by sub callbacks; read in physics tick)
        self._latest_joint_cmd = None          # JointMotionUpdate
        self._latest_pose_cmd = None           # MotionUpdate
        # ControllerState bookkeeping
        self._tcp_pose_buffer = []             # (t, pose) — for numerical-diff velocity
        self._last_reference_tcp_pose = None
        self._last_reference_joint_state = None
        self._last_target_mode = 0             # MODE_UNSPECIFIED

    def start(self):
        self.stop()  # idempotent
        # ... mirror parity_publishers.py:248-342 for path setup, rclpy.init,
        # Articulation init, but ADD: setup ArticulationKinematicsSolver,
        # setup contact subscription, create subs+pubs.

    def _on_physics_step(self, dt: float):
        # 1. drain rclpy
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0)
        # 2. apply latest joint cmd
        if self._latest_joint_cmd is not None:
            self._apply_joint_cmd(self._latest_joint_cmd)
            self._latest_joint_cmd = None  # consumed
        # 3. apply latest pose cmd
        if self._latest_pose_cmd is not None:
            self._apply_pose_cmd(self._latest_pose_cmd)
            self._latest_pose_cmd = None
        # 4. publish ControllerState
        self._publish_controller_state()
        # 5. drain off-limit contacts and publish
        self._publish_offlimit_contacts()
```

### Pattern 3: ArticulationActions (apply_action with name-keyed mapping)

```python
# Source: ~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py:1614-1660
# (ArticulationView/Articulation.apply_action signature)
import numpy as np
from isaacsim.core.utils.types import ArticulationActions

# Articulation has shape (M, K) = (1 env, 6 DOFs) for the UR5e arm.
# joint_names list MUST be the AIC name set (NOT slashed gripper joint, since that's FixedJoint zero-DOF).
positions = np.array([[q1, q2, q3, q4, q5, q6]], dtype=np.float32)  # (1, 6)
efforts   = np.array([[t1, t2, t3, t4, t5, t6]], dtype=np.float32)  # (1, 6) — feedforward torques

action = ArticulationActions(
    joint_positions=positions,
    joint_efforts=efforts,                 # OPTIONAL — pass only if msg.target_feedforward_torque non-empty
    joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
)
self._articulation.apply_action(action)  # apply_action handles name→index resolution internally
```

### Pattern 4: set_gains for per-tick stiffness/damping override

```python
# Source: ~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py:2973-3050
import numpy as np
# Shapes are (M, K) — for single env, single articulation: (1, num_joints)
kps = np.array([[k1, k2, k3, k4, k5, k6]], dtype=np.float32)  # from msg.target_stiffness
kds = np.array([[d1, d2, d3, d4, d5, d6]], dtype=np.float32)  # from msg.target_damping
self._articulation.set_gains(
    kps=kps,
    kds=kds,
    joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
)
# Note: set_gains requires timeline NOT stopped AND physics_sim_view ready (line 3023-3027).
# In physics-step callback that's already true — but if calling outside the callback (e.g. from
# the MCP atom handler before play), set save_to_usd=True instead, then on next play the gains
# are picked up.
```

### Pattern 5: Lula IK setup (recommended over RMPflow for Phase 2 first cut)

```python
# Source: ~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.robot_motion.motion_generation/isaacsim/robot_motion/motion_generation/lula/kinematics.py:28-67
# + articulation_kinematics_solver.py:38-104

import os
import isaacsim.robot_motion.motion_generation as motion_generation
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver

# Locate UR5e bundled config (no asset copy needed; they ship with Isaac Sim)
mg_root = os.path.dirname(motion_generation.__file__)
ur5e_root = os.path.join(mg_root, "..", "..", "..",  # walk up to ext root
                         "motion_policy_configs", "universal_robots", "ur5e")
robot_description_path = os.path.join(ur5e_root, "rmpflow", "ur5e_robot_description.yaml")
urdf_path = os.path.join(ur5e_root, "ur5e.urdf")

lula_solver = LulaKinematicsSolver(
    robot_description_path=robot_description_path,
    urdf_path=urdf_path,
)
# Bridge to the live articulation
self._kinematics = ArticulationKinematicsSolver(
    robot_articulation=self._articulation,   # SingleArticulation handle from Phase 1
    kinematics_solver=lula_solver,
    end_effector_frame_name="tool0",   # OR "gripper/tcp" if MotionUpdate.pose.frame_id == "gripper/tcp"
)

# In physics tick — handle a MotionUpdate:
target_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
target_orientation = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z])  # wxyz!
ik_action, success = self._kinematics.compute_inverse_kinematics(
    target_position=target_position,
    target_orientation=target_orientation,
)
if success:
    self._articulation.apply_action(ik_action)
else:
    self._node.get_logger().debug(
        f"IK did not converge for pose target ({target_position.tolist()})"
    )

# Also for ControllerState publish — FK on current joint positions:
ee_position, ee_rotation = self._kinematics.compute_end_effector_pose()
# ee_position is a (3,) numpy array (translation in stage units, i.e., meters)
# ee_rotation is a (3, 3) rotation matrix
```

**Important per `aic_controller.cpp:220-227`:** `MotionUpdate.header.frame_id` is restricted to `base_link` or `gripper/tcp`. Phase 2 must support both:
- `base_link` → `target_position` is in base_link frame; if Lula's URDF root is `base_link` (verify in `ur5e.urdf`), pass directly. Otherwise transform via `tf_buffer.lookup_transform`.
- `gripper/tcp` → relative to current gripper position; transform to base_link before IK.

### Pattern 6: omni.physx contact-report subscription

```python
# Source: ~/.claude/skills/robot-collision-forensics/scripts/contact_subscription.py
# (Drop-in pattern; can either import-copy or import as a sibling module.)

import omni.physx
import omni.timeline
import omni.usd
from pxr import UsdPhysics, PhysxSchema, PhysicsSchemaTools
from collections import deque

CONTACT_EVENTS = deque(maxlen=2048)
_SUB = None
_WATCHED_PATHS = set()

def setup_contact_subscription(watched_paths):
    global _SUB, _WATCHED_PATHS
    stage = omni.usd.get_context().get_stage()
    watched = set()
    for path in watched_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue   # Off-limit prim MUST have RigidBodyAPI to fire contacts
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            cr = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            cr.CreateThresholdAttr().Set(0.0)  # 0 = report all contacts; non-zero suppresses gentle pushes
        watched.add(path)
    _WATCHED_PATHS = watched
    sim_iface = omni.physx.get_physx_simulation_interface()
    tl = omni.timeline.get_timeline_interface()
    _SUB = sim_iface.subscribe_contact_report_events(_on_contact)
    return watched

def _on_contact(headers, data):
    # Runs on physics thread — must be O(fast); swallow per-event errors.
    t_now = float(omni.timeline.get_timeline_interface().get_current_time())
    for h in headers:
        try:
            a0 = str(PhysicsSchemaTools.intToSdfPath(int(h.actor0)))
            a1 = str(PhysicsSchemaTools.intToSdfPath(int(h.actor1)))
            if not (_is_watched(a0) or _is_watched(a1)):
                continue
            ix = iy = iz = 0.0
            for k in range(int(h.num_contact_data)):
                d = data[int(h.contact_data_offset) + k]
                ix += float(d.impulse[0]); iy += float(d.impulse[1]); iz += float(d.impulse[2])
            CONTACT_EVENTS.append({
                "t": t_now,
                "type": str(h.type).rsplit(".", 1)[-1],   # CONTACT_FOUND/PERSIST/LOST
                "a0": a0, "a1": a1,
                "impulse": [ix, iy, iz],
                "n_contacts": int(h.num_contact_data),
                # ... and position from data[k].position averaged
            })
        except Exception:
            continue
```

**Per-tick drain in physics-step callback:**
```python
def _publish_offlimit_contacts(self):
    from ros_gz_interfaces.msg import Contacts, Contact, Entity
    from geometry_msgs.msg import Vector3
    events = []
    n = len(CONTACT_EVENTS)
    for _ in range(n):
        events.append(CONTACT_EVENTS.popleft())
    if not events:
        return
    msg = Contacts()
    msg.header.stamp = self._node.get_clock().now().to_msg()
    msg.header.frame_id = "world"  # or base_link; verify against live aic_eval snapshot
    for e in events:
        if e["type"] != "CONTACT_FOUND":
            continue   # only report new contacts; PERSIST/LOST are noise per skill
        c = Contact()
        c.collision1 = Entity(name=e["a0"], type=Entity.LINK)
        c.collision2 = Entity(name=e["a1"], type=Entity.LINK)
        c.positions = []        # position(s) of contacts; can fill from e["position"]
        c.normals = []          # contact normals; not in the deque format above — extend if needed
        c.depths = []           # penetration depths
        c.wrenches = []         # ros_gz_interfaces/JointWrench[]
        msg.contacts.append(c)
    if msg.contacts:
        self._contacts_pub.publish(msg)
```

### Pattern 7: ros2 Time conversion to ControllerState header.stamp

```python
# Source: parity_publishers.py:380-385 (Phase 1 pattern)
from rclpy.clock import Clock
now = self._node.get_clock().now().to_msg()
msg.header.stamp = now
msg.header.frame_id = "base_link"  # per D-07
```

### Anti-Patterns to Avoid

- **Do NOT use `isaacsim.sensors.physics.ContactSensor`** for off-limit contacts. Returns `is_valid=False` always in Isaac Sim 5.0; `omni.physx.contact` extension absent. (Per `wrapper-failure-mode.md`.)
- **Do NOT use direct USD `.JointPositionAttr().Set()`** for command application. USD-side writes are stale during play; only `apply_action` reaches PhysX physics-view.
- **Do NOT spin rclpy in a separate thread.** Phase 1 proved physics-tick spin works (D-08). Multi-threaded rclpy in Kit's main process triggers GIL contention with Carbonite scheduler; debugging is significantly harder.
- **Do NOT pass `target_orientation` as `[x, y, z, w]`** to Lula IK. Lula uses `[w, x, y, z]` order (per `LulaInterfaceHelper:set_robot_base_pose` and `quats_to_rot_matrices` convention) — same as Isaac Sim core conventions but inverse of typical ROS conventions. ROS `geometry_msgs/Quaternion` is `(x, y, z, w)`; convert.
- **Do NOT raise exceptions from the physics-step callback.** Phase 1's `_on_physics_step` swallows errors and logs once. Phase 2 follows the same discipline (D-11). Per `robot-collision-forensics/SKILL.md`: "Event callback runs on the physics thread. Must be O(fast). Swallow per-event errors so one malformed header doesn't kill the subscription."
- **Do NOT call `set_gains` outside the timeline-running window without `save_to_usd=True`.** The runtime path requires `physics_sim_view is not None`. Quick-start ordering matters: gains apply in physics tick AFTER timeline.play().
- **Do NOT publish `/aic_controller/controller_state` while Gazebo's aic_eval is also running.** Two publishers on a non-LATCHED topic produces interleaved messages that confuse downstream consumers. Document in CLAUDE.md: Phase 2 mode is "Isaac Sim only", Phase 1 mode is "Isaac Sim + Gazebo coexist OK (passive surfaces only)".

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| 6-DOF UR5e IK | Hand-coded analytical IK or custom Jacobian solver | `LulaKinematicsSolver` + `ArticulationKinematicsSolver` | Bundled UR5e config; CCD + BFGS hybrid solver; warm-start from current joint positions; returns `ArticulationAction` ready for `apply_action`. Hand-coded IK loses singularity-handling, joint-limit awareness, warm-start speed. |
| Contact event detection | trimesh proximity / scene query overlap tests | `omni.physx.subscribe_contact_report_events` per `robot-collision-forensics` skill | Per-tick ground truth from PhysX; no synthetic samples to miss CONTACT_FOUND between |
| FK for tcp_pose | Hand-coded DH-parameter chain product | `ArticulationKinematicsSolver.compute_end_effector_pose()` | Same robot description as IK; consistent frame conventions; <0.01ms per call |
| Time/stamp construction | `time.time_ns()` math | `node.get_clock().now().to_msg()` | Isaac Sim's clock interface is the canonical sim-time source; matches Phase 1 publishers |
| QoS profile design for the 4 topics | Custom QoS guesses | Match `aic_controller.cpp` source: `RELIABLE / KEEP_LAST(10)` for joint_commands and pose_commands (line 182), `SystemDefaultsQoS` for controller_state (line 689), `RELIABLE / KEEP_LAST(10)` for off_limit contacts (mirror /tf style from Phase 1) | Mismatched QoS between publisher and subscriber silently drops messages. Source-of-truth is the C++ code. |
| Bad-command rejection logic | Custom validation framework | Mirror the validations in `aic_controller.cpp:236-330` exactly: check trajectory_generation_mode != UNSPECIFIED, target_state.positions.size() == num_joints (in MODE_POSITION), target_state.velocities.size() == num_joints (in MODE_VELOCITY), target_stiffness.size() == num_joints, target_damping.size() == num_joints | aic_controller already defines what "valid" means. Don't reinvent — drop-on-fail with same conditions, log the same warning. |

**Key insight:** All four custom subsystems (IK, contacts, FK, joint apply) have first-class Isaac Sim 5.0 APIs. The danger is reaching for a "simpler" hand-rolled version that loses production-ready behaviors (singularity handling, ground-truth contact attribution, ABI-correct typesupport). Use the bundled APIs.

## Off-Limit Contact Pipeline (PARITY-06 deep dive)

### Pre-flight: which prims are "off-limit"?

D-10 says "snapshot live aic_eval container's `/aic/gazebo/contacts/off_limit` during a CheatCode run, mirror the prim filter set." Concrete recipe:

```bash
# Run CheatCode E2E in Gazebo, watching the topic in another terminal:
cd ~/Documents/aic && ./scripts/run_cheatcode.sh headless &
sleep 30   # wait for stack
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash &&
  timeout 120 ros2 topic echo /aic/gazebo/contacts/off_limit
' | tee /tmp/aic_eval_offlimit_capture.txt
./scripts/run_cheatcode.sh stop
```

The Gazebo `Entity.name` field in each `Contact` will give the off-limit body's name (Gazebo entity name, e.g. `task_board::off_limit_screw_top_left`). Map these to Isaac Sim USD prim paths via the already-vendored `Task Board Base/` folder structure (`/World/TaskBoard/<name>` etc.). Document the mapping in `docs/offlimit-prim-mapping.md`.

If Gazebo emits NO off_limit contacts during a passing CheatCode trial (the expected case for a passing trial), seed the filter set from `~/Documents/aic/aic_assets/models/Task Board Base/` USD inspection: any prim with name containing `off_limit`, `screw`, `esd`, or `electronics` is a candidate.

### MCP atom signature

```python
# In MCP_TOOL_REGISTRY:
"setup_offlimit_contacts": {
    "description": "Subscribe to omni.physx contact events on the configured set of off-limit prims and publish ros_gz_interfaces/Contacts on /aic/gazebo/contacts/off_limit. Per PARITY-06 + D-10. Default prim filter from docs/offlimit-prim-mapping.md; pass prim_paths to override.",
    "parameters": {
        "prim_paths": {
            "type": "array",
            "items": {"type": "string"},
            "description": "Optional list of USD prim paths to monitor for off-limit contact events. If omitted, uses the default set from docs/offlimit-prim-mapping.md."
        }
    }
}
```

### Validation commands

```bash
# After atom invocation, verify the subscription is firing:
~/env_isaaclab/bin/python -c "
import sys, socket, json
s = socket.socket(); s.settimeout(120); s.connect(('localhost', 8768))
s.sendall(json.dumps({'type': 'execute_python_code', 'params': {
    'code': '''
from aic_dt.controller_loop import _CONTROLLER_LOOP_INSTANCE
import time
# Check the omni.physx subscription handle exists
assert _CONTROLLER_LOOP_INSTANCE._contact_sub is not None, \"Contact subscription not installed\"
# Drive the gripper into a known off-limit prim (manual test) — see smoke test
'''}}).encode())
print(s.recv(8192))
"

# Verify the topic is publishing:
ros2 topic echo /aic/gazebo/contacts/off_limit --once   # after the smoke test step 7
```

## Joint Command Application (PARITY-09 deep dive)

### JointMotionUpdate message handling

```python
def _on_joint_cmd(self, msg):
    # Validation per aic_controller.cpp:236-330
    n = 6   # UR5e arm DOFs
    if msg.trajectory_generation_mode.mode == 0:  # MODE_UNSPECIFIED
        self._node.get_logger().debug("Dropped: trajectory_generation_mode UNSPECIFIED")
        return
    if msg.trajectory_generation_mode.mode == 1:  # MODE_POSITION
        if len(msg.target_state.positions) != n:
            self._node.get_logger().debug(f"Dropped: positions.size()={len(msg.target_state.positions)} != {n}")
            return
    elif msg.trajectory_generation_mode.mode == 2:  # MODE_VELOCITY
        if len(msg.target_state.velocities) != n:
            self._node.get_logger().debug(f"Dropped: velocities.size()={len(msg.target_state.velocities)} != {n}")
            return
    if msg.target_stiffness and len(msg.target_stiffness) != n:
        self._node.get_logger().debug(f"Dropped: target_stiffness.size()={len(msg.target_stiffness)} != {n}")
        return
    if msg.target_damping and len(msg.target_damping) != n:
        self._node.get_logger().debug(f"Dropped: target_damping.size()={len(msg.target_damping)} != {n}")
        return
    # Buffer for next physics tick (don't apply from sub callback — main thread vs physics thread)
    self._latest_joint_cmd = msg
```

### Name-keyed mapping (D-09)

```python
# msg.target_state.joint_names is the source-of-truth ordering. Map each name to articulation DOF index.
# UR5e arm joints map directly. gripper/left_finger_joint is FixedJoint zero-DOF — silently ignored.
# Known ARM joint names (from aic_adapter::joint_sort_order_):
ARM_JOINTS = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
GRIPPER_NOOP = "gripper/left_finger_joint"

def _apply_joint_cmd(self, msg):
    arm_names = []
    arm_positions = []
    arm_efforts = []
    for i, name in enumerate(msg.target_state.joint_names):
        if name == GRIPPER_NOOP:
            continue   # FixedJoint — silently no-op per D-09
        if name not in ARM_JOINTS:
            self._node.get_logger().warn(f"Unknown joint name: {name} — skipping")
            continue
        arm_names.append(name)
        arm_positions.append(msg.target_state.positions[i])
        if msg.target_feedforward_torque:
            arm_efforts.append(msg.target_feedforward_torque[i])
    if not arm_names:
        return
    # set_gains first (gain change applies before next tick's PD step)
    if msg.target_stiffness and msg.target_damping:
        kps = np.array([[msg.target_stiffness[i] for i, n in enumerate(msg.target_state.joint_names) if n in ARM_JOINTS]], dtype=np.float32)
        kds = np.array([[msg.target_damping[i] for i, n in enumerate(msg.target_state.joint_names) if n in ARM_JOINTS]], dtype=np.float32)
        self._articulation.set_gains(kps=kps, kds=kds, joint_names=arm_names)
    # apply_action
    action_kwargs = {
        "joint_positions": np.array([arm_positions], dtype=np.float32),
        "joint_names": arm_names,
    }
    if arm_efforts:
        action_kwargs["joint_efforts"] = np.array([arm_efforts], dtype=np.float32)
    self._articulation.apply_action(ArticulationActions(**action_kwargs))
    # Bookkeeping for ControllerState publish
    self._last_reference_joint_state = msg.target_state
    self._last_target_mode = 2  # MODE_JOINT (per ControllerState.msg TargetMode enum)
```

## Lula RMPflow / IK Pipeline (PARITY-10 deep dive)

### IK setup at atom-invocation time

```python
def _setup_kinematics(self):
    """Initialize Lula IK solver. Idempotent; safe on hot-reload."""
    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
    from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver
    mg_root = os.path.dirname(mg.__file__)
    # Walk up to ext root; the motion_policy_configs lives 3 levels up from motion_generation/
    ur5e_root = os.path.normpath(os.path.join(
        mg_root, "..", "..", "..", "motion_policy_configs", "universal_robots", "ur5e"
    ))
    robot_description_path = os.path.join(ur5e_root, "rmpflow", "ur5e_robot_description.yaml")
    urdf_path = os.path.join(ur5e_root, "ur5e.urdf")
    if not os.path.exists(robot_description_path) or not os.path.exists(urdf_path):
        self._node.get_logger().error(
            f"Lula UR5e templates not found at {ur5e_root}; PARITY-10 will fail. "
            f"Check that isaacsim.robot_motion.motion_generation extension is enabled."
        )
        return False
    lula_solver = LulaKinematicsSolver(robot_description_path, urdf_path)
    # CRITICAL: end_effector_frame_name must be a frame in ur5e.urdf. For aic_unified_robot, "tool0"
    # is the URDF arm-end frame; "gripper/tcp" is NOT in the bundled UR5e URDF. If the AIC controller
    # sends pose targets in gripper/tcp frame, transform to base_link first via tf2_ros lookup.
    self._kinematics = ArticulationKinematicsSolver(
        robot_articulation=self._articulation,
        kinematics_solver=lula_solver,
        end_effector_frame_name="tool0",
    )
    return True
```

### MotionUpdate handling

```python
def _on_pose_cmd(self, msg):
    # Validation per aic_controller.cpp:218-227
    if msg.header.frame_id not in ("base_link", "gripper/tcp"):
        self._node.get_logger().debug(f"Dropped: pose frame_id={msg.header.frame_id} not in (base_link, gripper/tcp)")
        return
    self._latest_pose_cmd = msg

def _apply_pose_cmd(self, msg):
    # Convert pose to base_link frame if necessary (gripper/tcp → base_link via tf2_ros)
    if msg.header.frame_id == "gripper/tcp":
        # Transform: target = current_tcp_pose * msg.pose
        # OR use tf2_ros.Buffer.transform_pose_stamped (requires running listener)
        # First cut: if gripper/tcp targets are common, add tf2_ros listener; if rare, log+drop
        self._node.get_logger().debug("gripper/tcp frame transform not yet implemented — dropping")
        return
    target_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    # Quaternion: ROS=(x,y,z,w), Lula expects=(w,x,y,z)
    target_orientation = np.array([
        msg.pose.orientation.w, msg.pose.orientation.x,
        msg.pose.orientation.y, msg.pose.orientation.z,
    ])
    ik_action, success = self._kinematics.compute_inverse_kinematics(
        target_position=target_position,
        target_orientation=target_orientation,
    )
    if not success:
        self._node.get_logger().debug(
            f"IK did not converge for pose target ({target_position.tolist()})"
        )
        return
    self._articulation.apply_action(ik_action)
    self._last_reference_tcp_pose = msg.pose
    self._last_target_mode = 1  # MODE_CARTESIAN
```

## ControllerState Publisher (PARITY-11 deep dive)

```python
def _publish_controller_state(self):
    from aic_control_interfaces.msg import ControllerState, TargetMode
    from geometry_msgs.msg import Pose, Twist, WrenchStamped
    msg = ControllerState()
    msg.header.stamp = self._node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    # tcp_pose via FK
    try:
        ee_pos, ee_rot = self._kinematics.compute_end_effector_pose()
        msg.tcp_pose.position.x = float(ee_pos[0])
        msg.tcp_pose.position.y = float(ee_pos[1])
        msg.tcp_pose.position.z = float(ee_pos[2])
        # Convert rotation matrix to quaternion (Lula returns 3x3); use isaacsim helper
        from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
        q = rot_matrices_to_quats(ee_rot[None, :, :])[0]   # wxyz
        msg.tcp_pose.orientation.w = float(q[0])
        msg.tcp_pose.orientation.x = float(q[1])
        msg.tcp_pose.orientation.y = float(q[2])
        msg.tcp_pose.orientation.z = float(q[3])
    except Exception:
        pass  # leave zero-init; physics may not be ready
    # tcp_velocity via numerical-diff (3-sample ring buffer)
    t = self._node.get_clock().now().nanoseconds * 1e-9
    self._tcp_pose_buffer.append((t, msg.tcp_pose))
    if len(self._tcp_pose_buffer) > 3:
        self._tcp_pose_buffer.pop(0)
    if len(self._tcp_pose_buffer) >= 2:
        (t0, p0), (t1, p1) = self._tcp_pose_buffer[-2], self._tcp_pose_buffer[-1]
        dt = max(t1 - t0, 1e-6)
        msg.tcp_velocity.linear.x = (p1.position.x - p0.position.x) / dt
        msg.tcp_velocity.linear.y = (p1.position.y - p0.position.y) / dt
        msg.tcp_velocity.linear.z = (p1.position.z - p0.position.z) / dt
        # Angular velocity: skip for first cut; quaternion-diff is finicky
    # reference_tcp_pose
    if self._last_reference_tcp_pose is not None:
        msg.reference_tcp_pose = self._last_reference_tcp_pose
    # tcp_error: 6-vector (x, y, z, rx, ry, rz)
    if self._last_reference_tcp_pose is not None:
        msg.tcp_error[0] = msg.tcp_pose.position.x - self._last_reference_tcp_pose.position.x
        msg.tcp_error[1] = msg.tcp_pose.position.y - self._last_reference_tcp_pose.position.y
        msg.tcp_error[2] = msg.tcp_pose.position.z - self._last_reference_tcp_pose.position.z
        # rotation error: leave 0 for first cut (axis-angle delta is finicky)
    # reference_joint_state
    if self._last_reference_joint_state is not None:
        msg.reference_joint_state = self._last_reference_joint_state
    # target_mode
    msg.target_mode.mode = self._last_target_mode
    # fts_tare_offset: zero per D-07
    msg.fts_tare_offset.header.frame_id = "ati/tool_link"   # match aic_controller.cpp:1275
    msg.fts_tare_offset.header.stamp = msg.header.stamp
    self._ctrl_state_pub.publish(msg)
```

## Runtime State Inventory

This is a code-addition phase; no rename/refactor scope. Categories:

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | None — Phase 2 is purely code-side | None |
| Live service config | None — no external services Phase 2 reconfigures | None |
| OS-registered state | None — no Task Scheduler or systemd state involved | None |
| Secrets/env vars | `LD_LIBRARY_PATH` extension (D-13: add `~/Documents/aic/.pixi/envs/default/lib`) — needs to land in `~/env_isaaclab/bin/activate` AND in `CLAUDE.md` under "ALWAYS source the venv before launch" block | Update venv-activate script + CLAUDE.md |
| Build artifacts | New artifacts: `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/` and `.../ros_gz_interfaces/` — created when the workspace rebuild step runs. The Phase 1 workspace MUST be rebuilt as part of Phase 2 plan 1 (D-05 fix). | Add to verify_phase_2.sh check; document in `docs/aic-msgs-setup.md` |

## D-05 Landmine — replacement strategy

**The problem:** D-05 says "PYTHONPATH-link to `~/Documents/aic/.pixi/envs/default/lib/python3.*/site-packages/`". Empirical verification on this host (2026-05-03):
- Pixi env Python: 3.12.12 (`~/Documents/aic/.pixi/envs/default/bin/python3.12`)
- Isaac Sim Python: 3.11.13 (`~/env_isaaclab/bin/python`)
- The `aic_control_interfaces` `.so` files (e.g. `aic_control_interfaces_s__rosidl_typesupport_c.so`) ship without cpython-XX tags but were built against Python 3.12's C API.
- Importing the package from Python 3.11 fails at the numpy-import barrier (`No module named 'numpy._core._multiarray_umath'`) because the pixi env's numpy is also Python 3.12-built and ABI-incompatible.
- Even if numpy were fine, the rclpy `_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so` lookup logic in pixi env's rclpy expects 3.11 but the env ships 3.12 → no compatible binary present.

**The fix** (replace D-05):
1. Vendor `~/Documents/aic/aic_interfaces/aic_control_interfaces/` (5 .msg files + 1 .srv file + CMakeLists.txt + package.xml) into `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/aic_control_interfaces/`.
2. Verify package.xml's deps (`builtin_interfaces`, `geometry_msgs`, `std_msgs`, `trajectory_msgs`) are all present in the workspace — `[VERIFIED 2026-05-03]` they are.
3. Run `bash ~/IsaacSim-ros_workspaces/build_ros.sh -d humble -v 22.04` to rebuild.
4. Verify the install:
   ```
   ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/
   ```
   Should show `__init__.py`, `msg/`, `srv/`, plus `.so` files without cpython-310 tags.
5. The `controller_loop.py` import at module load time is then identical to the parity_publishers.py pattern — no extra path manipulation. The Phase 1 path-management logic in parity_publishers.py:32-86 already prepends the workspace path.

**Same fix applies to ros_gz_interfaces.** Source: `git clone -b humble https://github.com/gazebosim/ros_gz` → vendor `ros_gz_interfaces/` into the workspace src tree → rebuild.

**Estimated effort:** ~30 minutes — clone, copy, run build (Docker rebuild is the long pole at ~10-20 min).

**Risk if NOT fixed:** PARITY-09/10/11 messages won't deserialize at runtime. Subscribers will silently never receive messages, publishers will fail at the typesupport-import boundary. Smoke test step 2-3 fails immediately.

## Common Pitfalls

### Pitfall 1: `isaacsim.robot_motion.lula` import path drift (CONTEXT D-02 wording)

**What goes wrong:** CONTEXT D-02 says "bundled in `isaacsim.robot_motion.lula`". The actual API surface is at `isaacsim.robot_motion.motion_generation.lula.RmpFlow` (and `.LulaKinematicsSolver`). The bare `isaacsim.robot_motion.lula` module exists (extension `isaacsim.robot_motion.lula`) but is the lula-Python-bindings module — not the high-level RmpFlow class. There's also a deprecated `omni.isaac.lula` path under `extsDeprecated/`. Mid-execution agents grepping for `from isaacsim.robot_motion.lula import RmpFlow` will fail.

**Why it happens:** Isaac Sim 5.0 split the lula bindings (low-level, ext name `isaacsim.robot_motion.lula`) from the motion-generation high-level wrappers (ext name `isaacsim.robot_motion.motion_generation`). Both import from the same underlying `lula` Python package.

**How to avoid:** Use `from isaacsim.robot_motion.motion_generation.lula import RmpFlow, LulaKinematicsSolver` and `from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver`. The ext that ships RmpFlow is `isaacsim.robot_motion.motion_generation` — make sure it's enabled (probably already is by default; verify in the live extension log).

**Warning signs:** `ImportError: cannot import name 'RmpFlow' from 'isaacsim.robot_motion.lula'` at controller_loop.py module-load time.

### Pitfall 2: Lula UR5e URDF doesn't include the `gripper/tcp` frame

**What goes wrong:** The bundled `motion_policy_configs/universal_robots/ur5e/ur5e.urdf` is the standard UR5e URDF — it has frames `base_link`, `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`, `flange`, `tool0`, `ft_frame`. It does NOT include the AIC additions `gripper/tcp`, `gripper/hande_base_link`, `cam_mount/cam_mount_link`, etc. So `compute_end_effector_pose()` for `end_effector_frame_name="gripper/tcp"` will raise an error.

**Why it happens:** The bundled UR5e URDF is generic; AIC's unified robot adds the Hand-E gripper subtree.

**How to avoid:** Two options:
- **Option A (simpler, recommended):** Use `tool0` as the IK end-effector frame. Compute the static `tool0 → gripper/tcp` transform once (it's ~constant across the UR5e arm — only the gripper finger TF moves), apply this offset to all incoming `MotionUpdate.pose` targets before IK, and apply the inverse offset to the FK result before publishing tcp_pose. Verify the static offset by reading from Phase 1's TF tree (parity_publishers.py:188 — the chain `tool0 → cam_mount/cam_mount_link → ati/base_link → ati/tool_link → gripper/hande_base_link → gripper/tcp`).
- **Option B (more work):** Vendor a custom `aic_unified_robot.urdf` and `aic_robot_description.yaml` that include the AIC frame additions. Per D-06 vendoring discipline, place under `exts/aic-dt/assets/robot/rmpflow/`. Re-tune RMPflow weights for the longer chain.

**Warning signs:** `Frame name "gripper/tcp" not recognized by KinematicsSolver` carb error at IK call time.

### Pitfall 3: Double-source ControllerState publishing

**What goes wrong:** If Gazebo's `aic_eval` container is running AND Isaac Sim's controller-loop is running, BOTH publish `/aic_controller/controller_state`. CheatCode-style consumers see interleaved messages from two different physics simulations and behave unpredictably.

**Why it happens:** D-07 says Isaac Sim publishes ControllerState. aic_controller.cpp:688 says aic_controller publishes it too. There's no coordination layer.

**How to avoid:** Document mode separation in CLAUDE.md and Phase 2 docs. Either:
- Stop the aic_eval container before the Phase 2 atom is invoked (`./scripts/run_cheatcode.sh stop`), OR
- Set `ROS_DOMAIN_ID` to different values in the two stacks (Isaac Sim uses 0 by default, run aic_eval with `--env ROS_DOMAIN_ID=1`).

**Warning signs:** `ros2 topic info /aic_controller/controller_state --verbose` shows publisher count > 1.

### Pitfall 4: ROS quaternion order vs Lula quaternion order

**What goes wrong:** ROS `geometry_msgs/Quaternion` uses `(x, y, z, w)` order. Lula's `set_end_effector_pose` and IK solvers expect `(w, x, y, z)` order. Passing the ROS-order quaternion produces wildly wrong IK results that silently "succeed" (the BFGS solver converges to SOME local minimum) but commands the arm to a totally different pose.

**Why it happens:** ROS quaternion historical convention vs Eigen/numpy convention.

**How to avoid:** Always reorder when crossing the boundary:
```python
# Going INTO Lula:
target_orientation_lula = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                                     msg.pose.orientation.y, msg.pose.orientation.z])
# Going OUT of Lula (FK result is rotation matrix; convert to quat):
q_wxyz = rot_matrices_to_quats(rot_3x3[None, :, :])[0]   # already wxyz
# To geometry_msgs/Quaternion:
ros_quat.w = q_wxyz[0]; ros_quat.x = q_wxyz[1]; ros_quat.y = q_wxyz[2]; ros_quat.z = q_wxyz[3]
```

**Warning signs:** Robot arm jumps to apparently-random configurations on the first MotionUpdate command. Visual sanity check fails immediately.

### Pitfall 5: rclpy spin_once blocking inside physics callback

**What goes wrong:** `rclpy.spin_once(node, timeout_sec=0)` is documented as non-blocking (poll), but if multiple sub callbacks are queued AND a callback raises an exception, spin_once propagates the exception into the physics callback — wedging Phase 2 the same way Phase 1 had the cooking deadlock.

**Why it happens:** Per D-11 ("drop bad commands silently"), but this discipline must extend to the sub callbacks themselves, not just the apply logic.

**How to avoid:** Wrap each subscriber callback body in `try/except` and log-then-return. Phase 1's parity_publishers.py:393-397 has the wrapper pattern (`_logged_publish_error`).

**Warning signs:** Kit log shows a single Python traceback then no further activity from the controller-loop (no ControllerState messages, no command application). MCP socket may still respond but `quick_start` and `play_scene` may wedge.

### Pitfall 6: Articulation handle not initialized when atom is invoked pre-play

**What goes wrong:** `Articulation.set_gains()` raises `Articulation needs to be initialized` warning if `_is_initialized=False`, AND silently no-ops if timeline is stopped. Atom invoked before `play_scene` or before `load_robot` produces silent failure.

**Why it happens:** Articulation init happens in `World.instance().reset_async()` in `load_robot` (extension.py:1230); set_gains pre-play has the additional check at line 3023-3027.

**How to avoid:**
- Atom handler verifies `self._articulation is not None and self._articulation.is_physics_handle_valid()` before installing the loop.
- If invoked pre-play, defer install to first physics-tick after timeline.play(); set a `_pending_install` flag and consume it in the first `_on_physics_step` callback.
- Mirror Phase 1's `_lazy_init_articulation` pattern (extension.py:849-...) for hot-reload safety.

**Warning signs:** Atom returns `success` but no PARITY-09 commands take effect; topic info shows 0 subscribers from Isaac Sim side.

### Pitfall 7: PhysxContactReportAPI applied to a prim WITHOUT RigidBodyAPI fires no events

**What goes wrong:** Off-limit prim is a static collider (UsdPhysics.CollisionAPI only, not RigidBodyAPI). Applying PhysxContactReportAPI silently does nothing; no contact events fire when the gripper touches it.

**Why it happens:** PhysX contact reports require at least one of the contact pair to be a rigid body actor.

**How to avoid:** The drop-in `setup_contact_subscription` in robot-collision-forensics already filters for `prim.HasAPI(UsdPhysics.RigidBodyAPI)` and skips otherwise. **Add diagnostic logging** when a watched prim is skipped:
```python
if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
    self._node.get_logger().warn(
        f"Off-limit prim {path} has no RigidBodyAPI — contact reports won't fire. "
        f"Apply UsdPhysics.RigidBodyAPI to make it a contact source, or watch the gripper side instead."
    )
    continue
```
Alternative: watch the GRIPPER fingers (which definitely have RigidBodyAPI) and filter contacts by the off-limit prim being in the OTHER side of the pair.

**Warning signs:** Smoke test step 7 fails with empty `/aic/gazebo/contacts/off_limit` despite verified gripper-into-off-limit-prim collision.

### Pitfall 8: PhysxContactReportAPI threshold default suppresses gentle contacts

**What goes wrong:** If the threshold attribute is left at its default (some non-zero impulse/force value), gentle sustained pushes (the "knock" pattern) don't fire CONTACT_FOUND events.

**Why it happens:** PhysX's threshold filters out low-impulse contacts to reduce per-tick callback churn.

**How to avoid:** Always explicitly set threshold to 0.0 — pattern in robot-collision-forensics drop-in code:
```python
cr = PhysxSchema.PhysxContactReportAPI.Apply(prim)
cr.CreateThresholdAttr().Set(0.0)
# OR if already applied:
cr = PhysxSchema.PhysxContactReportAPI(prim)
thr = cr.GetThresholdAttr()
if thr and (thr.Get() is None or thr.Get() > 0.0):
    cr.CreateThresholdAttr().Set(0.0)
```

**Warning signs:** Light touches don't fire events; only hard collisions do.

### Pitfall 9: Articulation root prim path is `/World/UR5e/aic_unified_robot/root_joint` (NOT `/World/UR5e`)

**What goes wrong:** Constructing `SingleArticulation("/World/UR5e")` or `Articulation(prim_paths_expr="/World/UR5e")` may "work" (return a handle) but `get_joint_positions()` returns None and apply_action does nothing.

**Why it happens:** The articulation root in the unified USD is the `/World/UR5e/aic_unified_robot/root_joint` PhysicsFixedJoint, NOT the Xform parent. Phase 1 verified this via `probe_root_joint.py`.

**How to avoid:** Use `self._articulation_root_prim_path` (already set by Phase 1 as `/World/UR5e/aic_unified_robot`, extension.py:487) and append `/root_joint` for ArticulationView:
```python
art_root = f"{self._articulation_root_prim_path}/root_joint"
self._articulation = Articulation(prim_paths_expr=art_root)
self._articulation.initialize()
```
Phase 1's parity_publishers.py:317-322 has this exact pattern.

**Warning signs:** `self._articulation.dof_names` is `[]` or None after `initialize()`.

## Code Examples

### Atom: setup_controller_subscribers

```python
# In MCP_TOOL_REGISTRY (extension.py:142):
"setup_controller_subscribers": {
    "description": "Subscribe to /aic_controller/joint_commands (JointMotionUpdate) and /aic_controller/pose_commands (MotionUpdate); publish /aic_controller/controller_state (ControllerState). Per PARITY-09/10/11 + D-01/D-04. Idempotent: re-invocation tears down prior loop.",
    "parameters": {}
},

# In MCP_HANDLERS (extension.py:350):
"setup_controller_subscribers": "_cmd_setup_controller_subscribers",

# Method on DigitalTwin class:
def _cmd_setup_controller_subscribers(self) -> Dict[str, Any]:
    try:
        self._start_aic_controller_loop()
        return {"status": "success", "message": "Controller subscribers + state publisher started"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": f"setup_controller_subscribers failed: {str(e)}"}

# Helper (mirror _start_aic_parity_publishers, extension.py:1538):
def _start_aic_controller_loop(self):
    try:
        from .controller_loop import AicControllerLoop
    except Exception as exc:
        print(f"[AIC-DT][controller] Import failed: {exc!r}")
        return
    if self._aic_controller_loop is None:
        self._aic_controller_loop = AicControllerLoop(
            robot_xform_path=self._articulation_root_prim_path,
        )
    self._aic_controller_loop.start()

# UI button (in create_ui, near the existing Phase 1 buttons):
ui.Button("Setup Controller Subscribers", clicked_fn=self.setup_controller_subscribers)
# (Where setup_controller_subscribers is a thin wrapper: self._start_aic_controller_loop())
```

### Atom: setup_offlimit_contacts (similar shape, omitted for brevity)

### on_shutdown extension

```python
# In on_shutdown (extension.py:3179), AFTER the parity_publishers cleanup:
if self._aic_controller_loop is not None:
    try:
        self._aic_controller_loop.stop()
    except Exception as exc:
        print(f"[AIC-DT][controller] stop() failed in shutdown: {exc!r}")
    self._aic_controller_loop = None

if self._aic_offlimit_contacts is not None:
    try:
        self._aic_offlimit_contacts.stop()
    except Exception as exc:
        print(f"[AIC-DT][offlimit] stop() failed in shutdown: {exc!r}")
    self._aic_offlimit_contacts = None
```

### quick_start chain extension

```python
# In quick_start (extension.py:982), AFTER the JointState publisher (step 3b), BEFORE setup_action_graph (step 4):

# 3d. Setup AIC controller-loop subscribers (Phase 2 PARITY-09/10/11)
print("--- Setting up AIC Controller Subscribers (joint_commands + pose_commands + controller_state) ---")
self._start_aic_controller_loop()
await app.next_update_async()

# 3e. Setup off-limit contact reporting (Phase 2 PARITY-06)
print("--- Setting up Off-Limit Contact Publisher (/aic/gazebo/contacts/off_limit) ---")
self._start_aic_offlimit_contacts()
await app.next_update_async()
```

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | Python 3.10 standalone scripts using `/opt/ros/humble`'s rclpy + tf2_ros (matches Phase 1's smoke test pattern; matches the Python that aic_adapter and CheatCode actually use) |
| Config file | none — single-file scripts under `exts/aic-dt/scripts/` |
| Quick run command | `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py` |
| Full suite command | `bash exts/aic-dt/scripts/verify_phase_2.sh` (extends Phase 1's `verify_phase_1.sh`; runs both phase smoke tests + topic-info checks) |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| PARITY-06 | `/aic/gazebo/contacts/off_limit` publishes Contacts on off-limit collision | smoke | `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py --test offlimit` | ❌ Wave 0 (smoke_test_aic_controller.py is the new artifact) |
| PARITY-09 | `/aic_controller/joint_commands` subscribed; UR5e moves to commanded positions | smoke | `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py --test joint_cmd` | ❌ Wave 0 |
| PARITY-10 | `/aic_controller/pose_commands` subscribed; UR5e EE moves to commanded pose (verified via `tf2_ros::Buffer.lookup_transform('base_link', 'gripper/tcp')`) | smoke | `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py --test pose_cmd` | ❌ Wave 0 |
| PARITY-11 | `/aic_controller/controller_state` publishes ControllerState; tcp_pose non-zero after a command; tcp_error reflects pose-vs-reference delta | smoke | `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py --test controller_state` | ❌ Wave 0 |
| (D-05 fix) | Custom message types build for Python 3.11 in the workspace | unit | `ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/__init__.py && ls ~/IsaacSim-ros_workspaces/.../ros_gz_interfaces/__init__.py` | ❌ Wave 0 (verify_phase_2.sh) |
| (D-13) | LD_LIBRARY_PATH includes pixi env path | unit | `bash -c 'source ~/env_isaaclab/bin/activate && echo \"$LD_LIBRARY_PATH\" | grep -q "aic/.pixi/envs/default/lib"'` | ❌ Wave 0 |

### Sampling Rate

- **Per task commit:** `python3 -c "import ast; ast.parse(open('exts/aic-dt/aic_dt/controller_loop.py').read())"` (syntax check only, fast)
- **Per wave merge:** `python3 exts/aic-dt/scripts/smoke_test_aic_controller.py` against a running Isaac Sim instance — full 7-step smoke
- **Phase gate:** Both `smoke_test_aic_parity.py` (Phase 1 regression) AND `smoke_test_aic_controller.py` pass before `/gsd-verify-work`

### Wave 0 Gaps

- [ ] `exts/aic-dt/scripts/smoke_test_aic_controller.py` — covers PARITY-06/09/10/11 (D-12 contract, 7 steps)
- [ ] `exts/aic-dt/scripts/verify_phase_2.sh` — extends Phase 1's verify; checks workspace builds + LD_LIBRARY_PATH + topic info
- [ ] `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh` — discovery script for off-limit prim filter (D-10)
- [ ] `exts/aic-dt/aic_dt/controller_loop.py` — the loop class (the Phase 2 deliverable, ~600 LOC)
- [ ] `exts/aic-dt/docs/aic-msgs-setup.md` — D-05 fix instructions (build aic_control_interfaces + ros_gz_interfaces in the workspace)
- [ ] `exts/aic-dt/docs/offlimit-prim-mapping.md` — Gazebo off-limit entity name → Isaac Sim USD prim path mapping
- [ ] Workspace src additions: `aic_control_interfaces/` and `ros_gz_interfaces/` cloned into `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/`

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Sim 5.0 (`isaacsim` CLI in env_isaaclab) | All Phase 2 work | ✓ | 5.0 (Python 3.11.13 venv) | none |
| `isaacsim.robot_motion.motion_generation` extension | PARITY-10 (Lula IK) | ✓ | bundled with Isaac Sim 5.0 | PyKDL fallback per D-02 |
| Lula UR5e templates | PARITY-10 | ✓ | shipped at `motion_policy_configs/universal_robots/ur5e/` | clone from NVIDIA samples repo if absent |
| `omni.physx` (with subscribe_contact_report_events) | PARITY-06 | ✓ | bundled with Isaac Sim 5.0 | none — `ContactSensor` wrapper is broken |
| ROS Humble Python 3.11 workspace | All rclpy work | ✓ | `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/` (80 packages) | rebuild via `bash ~/IsaacSim-ros_workspaces/build_ros.sh -d humble -v 22.04` |
| `aic_control_interfaces` (Python 3.11 build) | PARITY-09/10/11 message handling | ✗ | not built | **MUST add to workspace + rebuild (Phase 2 plan 1)** |
| `ros_gz_interfaces` (Python 3.11 build) | PARITY-06 message construction | ✗ | not built (only Python 3.10 build at `/opt/ros/humble/local/lib/python3.10/dist-packages/`) | **MUST add to workspace + rebuild (Phase 2 plan 1)** |
| `aic_controller` (the C++ ros2_control plugin source) | reference / source-of-truth (NOT a runtime dep) | ✓ | `~/Documents/aic/aic_controller/src/` | n/a (read-only reference) |
| AIC pixi env at `~/Documents/aic/.pixi/envs/default/` | `LD_LIBRARY_PATH` for ros-kilted shared libs (D-13) | ✓ | Python 3.12.12 — note ABI mismatch (D-05 landmine) | n/a |
| Docker (for aic_eval container — off-limit prim discovery, smoke verification baseline) | D-10 discovery script | ✓ (verified by Phase 1) | n/a | manual list from `Task Board Base/` USD inspection |
| `/opt/ros/humble`'s Python 3.10 + rclpy + tf2_ros | smoke_test_aic_controller.py | ✓ | matches Phase 1 smoke test stack | n/a |

**Missing dependencies with no fallback:**
- `aic_control_interfaces` Python 3.11 build — Phase 2 first plan must build it.
- `ros_gz_interfaces` Python 3.11 build — Phase 2 first plan must build it.

**Missing dependencies with fallback:**
- PyKDL — defer until Lula proves insufficient.

## Sources

### Primary (HIGH confidence)

- `~/.claude/skills/robot-collision-forensics/SKILL.md` + `scripts/contact_subscription.py` + `references/wrapper-failure-mode.md` — D-03 implementation source-of-truth (`[CITED]`)
- `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` — extension lifecycle, MCP socket protocol, articulation API (`[CITED]`)
- `~/.claude/skills/nvidia-suite-docs/SKILL.md` — NVIDIA stack live-doc router (`[CITED]`)
- `/home/aaugus11/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py` lines 1614 (apply_action), 2973 (set_gains) — exact API signatures `[VERIFIED: filesystem read 2026-05-03]`
- `/home/aaugus11/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.robot_motion.motion_generation/isaacsim/robot_motion/motion_generation/lula/{motion_policies.py,kinematics.py,interface_helper.py}` + `articulation_kinematics_solver.py` — Lula API surface `[VERIFIED]`
- `/home/aaugus11/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/universal_robots/ur5e/{ur5e.urdf, rmpflow/ur5e_robot_description.yaml, rmpflow/ur5e_rmpflow_config.yaml}` — UR5e bundled templates `[VERIFIED]`
- `/home/aaugus11/Documents/aic/aic_controller/src/aic_controller.cpp` lines 182-330 (subscriber setup + validation), 688-692 (controller_state publish), 1257-1280 (populate_controller_state) — source-of-truth for the topic contract this phase mirrors `[VERIFIED: filesystem read]`
- `/home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/aic_dt/parity_publishers.py` — Phase 1 template (mirror inverted) `[VERIFIED]`
- `/home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/aic_dt/extension.py` lines 142-379 (MCP_TOOL_REGISTRY + MCP_HANDLERS), 421-516 (on_startup state init), 982-1040 (quick_start chain), 1233-1249 (load_robot articulation + drive setup), 1317-1556 (Phase 1 publisher atoms), 3179-3207 (on_shutdown teardown) `[VERIFIED]`
- `/home/aaugus11/Documents/aic/aic_interfaces/aic_control_interfaces/{package.xml, CMakeLists.txt, msg/*.msg}` — message contracts + build dependencies `[VERIFIED]`
- `/home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/docs/topic-parity-reference.md` — empirical aic_eval topic surface from Phase 1 D-01 `[VERIFIED]`

### Secondary (MEDIUM confidence)

- ABI mismatch verification: `~/env_isaaclab/bin/python -c "import aic_control_interfaces"` from pixi env path → ImportError `[VERIFIED 2026-05-03]`
- `ros_gz_interfaces.msg.Contacts` available at `/opt/ros/humble/local/lib/python3.10/dist-packages/ros_gz_interfaces/` with cpython-310-tagged `.so` files (NOT Python 3.11 compatible at typesupport level) `[VERIFIED]`
- aic_controller publishes `~/controller_state` at controller_manager `update_rate` (per ros2_control architecture) — likely 500 Hz per Phase 1's note in topic-parity-reference.md `[CITED: aic_ros2_controllers.yaml mention in PARITY-03 closure note]`

### Tertiary (LOW confidence — flag for validation during execution)

- The `MotionUpdate.header.frame_id == "gripper/tcp"` case may never actually be exercised by CheatCode (most policies use base_link). Pitfall #2 Option A vs Option B choice can be deferred until evidence shows gripper/tcp targets in the live aic_eval `/aic_controller/pose_commands` topic. Recommend logging-and-dropping for Phase 2 first cut, revisit if Phase 4 trial run shows the case occurring.
- The Gazebo off-limit topic frame_id (msg.header.frame_id of `Contacts`) — this research uses "world" as a reasonable default; verify by inspecting a live `/aic/gazebo/contacts/off_limit` capture during D-10's discovery snapshot. `[ASSUMED]`
- `set_gains` interaction with USD-applied DriveAPI stiffness: research assumes runtime `set_gains` overrides the boot-time DriveAPI values for that physics tick onward. Not directly verified — confirm via probe early in execution. `[ASSUMED]`

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `omni.isaac.lula.RmpFlow` (deprecated module path) | `isaacsim.robot_motion.motion_generation.lula.RmpFlow` | Isaac Sim 5.0 reorg | All training-data references to `omni.isaac.*` are wrong; use `isaacsim.*` namespace |
| `isaacsim.sensors.physics.ContactSensor` (broken in 5.0) | `omni.physx.subscribe_contact_report_events` | Isaac Sim 5.0 (`omni.physx.contact` ext absent) | Per `wrapper-failure-mode.md` — every ContactSensor in 5.0 returns is_valid=False |
| OGN `ROS2SubscribeJointState` / `ROS2Publisher` for custom message types | rclpy in physics-tick callback (Phase 1 D-10/D-11) | Phase 1 Gap A/B inline fix (commit 079afd8) | OGN can't deliver custom msg types or AIC's slashed names; Phase 2 inverts the same pattern for subscribers |
| Single `ArticulationAction` (deprecated for ArticulationView) | `ArticulationActions` (plural; supports multi-env shape (M, K)) | Isaac Sim 5.0 ArticulationView API | Phase 2 controller_loop.py uses ArticulationActions throughout for forward-compat |

**Deprecated/outdated:**
- `omni.isaac.lula` — moved to `isaacsim.robot_motion.lula` (low-level bindings) + `isaacsim.robot_motion.motion_generation` (high-level wrappers). Old path under `extsDeprecated/`.
- `isaacsim.sensors.physics.ContactSensor` — not removed but functionally broken in Isaac Sim 5.0; do NOT use.
- ros2_control + hardware_interface integration in Isaac Sim — exists but explicitly out-of-scope per HANDOFF D-2.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | Gazebo `Contacts` message header.frame_id = "world" | Pattern 6 (Off-Limit Contact Pipeline) | Low — first publish sets it; if wrong, fix in 1 line after D-10 snapshot |
| A2 | `aic_controller` publish rate on `/aic_controller/controller_state` is the controller_manager update_rate (~500 Hz) | "aic_controller architecture clarification" in Summary | Low — Phase 2 publishes at physics-tick (~60-120 Hz) which is acceptable per topic-parity-reference.md note on /joint_states rate |
| A3 | `set_gains` runtime override DOES take precedence over boot-time `DriveAPI.GetStiffnessAttr().Set()` | Pattern 4 + Pitfall 6 | Medium — if PhysX caches the DriveAPI value, runtime set_gains may need an extra `physics_sim_view.refresh()` call or similar; needs probe in execution |
| A4 | The aic_eval Gazebo container uses ROS_DOMAIN_ID=0 by default, same as Isaac Sim, hence the conflict in Pitfall 3 | Pitfall 3 | Low — easily confirmed at run time; Phase 2 can mandate ROS_DOMAIN_ID separation as workflow rule |
| A5 | The bundled UR5e URDF's `tool0` frame is co-located with the AIC unified-robot's `tool0` frame (no static offset between them) | Pitfall 2 + Pattern 5 | Medium — if there's a small static offset, pose targets will be off by that offset; cross-check by comparing FK output with Phase 1's tf2 lookup_transform `base_link → tool0` |
| A6 | `MotionUpdate.header.frame_id == "gripper/tcp"` is rarely or never used by CheatCode | Pitfall 2 LOW-confidence note | Low for Phase 2 first cut (drop+log); revisit Phase 4 if E2E trial reveals usage |
| A7 | `aic_control_interfaces` builds cleanly under ROS Humble despite being authored for ROS Kilted | D-05 Landmine fix | Medium — if Kilted-only generators or msg-format changes are used, Humble build fails. Mitigation: package.xml lists no Kilted-only deps (verified), CMakeLists uses standard rosidl_default_generators (verified). |
| A8 | `ros_gz_interfaces` builds cleanly under ROS Humble (the `humble` branch of https://github.com/gazebosim/ros_gz exists and builds) | D-05 Landmine fix | Low — `ros_gz` is a long-standing project with stable Humble support |
| A9 | The drop-in `~/.claude/skills/robot-collision-forensics/scripts/contact_subscription.py` is forward-compatible with the omni.physx API in this Isaac Sim 5.0 install | Pattern 6 + Pitfall 7/8 | Low — the skill explicitly targets Isaac Sim 5.0; Phase 1's parity_publishers.py uses the same omni.physx subscribe pattern (subscribe_physics_step_events) successfully |
| A10 | `aic_controller` topics are at `/aic_controller/joint_commands`, `/aic_controller/pose_commands`, `/aic_controller/controller_state` (the `~` namespace in C++ source resolves to the controller node's name, which IS `aic_controller`) | Summary point 4 | Low — topic-parity-reference.md "drift note" 2026-05-02 confirms `/aic_controller/joint_motion_update` and `/aic_controller/motion_update` topics observed live (note: live snapshot used `_update` suffix; verify final topic name in D-10 snapshot before plan-time. CONTEXT.md uses `_commands` per the C++ source.) **DISCREPANCY ALERT — see Open Questions Q1.** |

**The largest assumption — verify in execution:** A3 (set_gains runtime override semantics) and A10 (topic name discrepancy) should both be verified by quick probes during Phase 2 plan 1 execution before the rest of the implementation depends on them.

## Open Questions (RESOLVED)

1. **Topic name discrepancy: `_commands` vs `_motion_update` suffix.**
   - What we know: `aic_controller.cpp:185,235` declares the subscriber topics as `~/pose_commands` and `~/joint_commands` (resolves to `/aic_controller/pose_commands` and `/aic_controller/joint_commands`). 02-CONTEXT.md uses these names. Phase 1's topic-parity-reference.md "drift note" mentions live aic_eval topics named `/aic_controller/joint_motion_update` and `/aic_controller/motion_update` (different suffix).
   - What's unclear: Whether the live container uses different topic names than the source code (possible if there's a remap in the launch file), OR the drift note text was wrong, OR there are MULTIPLE topics (commands AND motion_update for some duality reason).
   - Recommendation: Phase 2 plan 1 must include a discovery step — `docker exec aic_eval ros2 topic list | grep aic_controller` — to settle the question. CONTEXT.md is locked, but the topic name string is a configuration value not an architectural decision; correcting it is a 4-line edit. Do this BEFORE plan 2 to avoid cascading rework.
   - **RESOLVED:** Discovery-at-execution pattern. Plan 02-01 Task 2 runs `docker exec aic_eval ros2 topic list | grep aic_controller` against a live `aic_eval` container and writes the discovered names to `exts/aic-dt/docs/aic-controller-topic-names.md`. Plans 02-03 and 02-04 read this file before hardcoding subscriber topic strings (they presently use the C++-source `_commands` literal as the default — Plan 02-01's snapshot is the tiebreaker, and a 4-line edit in those plans is the contingency). The discovery is intentional — the live container is the parity tiebreaker per Phase 1 D-01's empirical principle ("live aic_eval surface is the parity tiebreaker"). This is not a deferred decision; it's an empirical-not-config-doc lookup folded into the plan as Task 2 of 02-01.

2. **Does aic_controller actually run in Isaac Sim mode, OR does Isaac Sim REPLACE it?**
   - What we know: HANDOFF D-2 says "Isaac Sim is the hardware sink." 02-CONTEXT.md D-01 says "Isaac Sim subscribes" to the same topics aic_controller subscribes to. `aic_controller.cpp:688` says aic_controller publishes `controller_state`. D-07 says Isaac Sim publishes `controller_state`. Two publishers for one topic is invalid.
   - What's unclear: Whether the Phase 2 architectural intent is "Isaac Sim acts as the controller AND the hardware" (replacing aic_controller entirely) OR "Isaac Sim acts as the hardware only and aic_controller still runs separately, talking to Isaac Sim via the joint_commands → joint_states feedback loop". The two interpretations have very different validation paths.
   - Recommendation: Treat the Pitfall #3 documentation as authoritative — Phase 2 mode is "Isaac Sim only" (replaces both controller and hardware). If aic_controller is also running, document the ROS_DOMAIN_ID separation as a workflow requirement. Phase 4 (CheatCode E2E) is when this REALLY matters; Phase 2's smoke test can sidestep the question by using only Isaac Sim's ros2 stack.
   - **RESOLVED:** Coexistence with `ROS_DOMAIN_ID` isolation. Per CONTEXT D-2 (HANDOFF lock): "aic_controller IS the controller; Isaac Sim is the 'hardware' sink." Phase 2's architectural anchor is that BOTH publishers can exist on `/aic_controller/controller_state` — but they MUST run on different ROS_DOMAIN_IDs to prevent the double-publish corruption. Phase 4 trial runs use `ROS_DOMAIN_ID=10` (or similar nonzero value) on Isaac Sim and the live `aic_eval` Docker container, with the participant policy + aic_engine bound to one or the other domain depending on whether the trial is sim-side or hardware-side. Phase 2's smoke test sidesteps this by running on the Isaac Sim domain only (no live aic_eval container concurrently). Pitfall #3 ("aic_controller_state double-publish") documents the verification step: `ROS_DOMAIN_ID=10 ros2 topic list | grep controller_state` MUST return exactly one `/aic_controller/controller_state` entry.

3. **What's the IK chain for AIC's `gripper/tcp` end-effector frame?**
   - What we know: Phase 1's TF tree has `tool0 → cam_mount/cam_mount_link → ati/base_link → ati/tool_link → gripper/hande_base_link → gripper/tcp` as static offsets (8 hops total in the gripper chain). Bundled UR5e URDF doesn't have these frames.
   - What's unclear: Whether the static `tool0 → gripper/tcp` offset can be hard-coded as a single 4x4 matrix (Pitfall #2 Option A) or whether AIC's controller expects IK on the full kinematic chain (where the gripper finger DOFs matter — but they're FixedJoint per Phase 1 D-09).
   - Recommendation: Option A (static offset) — AIC's gripper fingers don't move during IK (they only open/close on `/gripper_command`). Compute the offset once via a probe at extension load, cache in `controller_loop.py`. If a future need arises for full-chain IK, escalate to Option B (vendor a custom URDF).
   - **RESOLVED:** Pitfall #2 Option A (static offset). Plan 02-04 implements the static `tool0 → gripper/tcp` SE(3) offset cached at `_setup_kinematics` time. The offset is captured once from Phase 1's TF tree (the `parity_publishers.py` static-TF table publishes the full chain at startup) OR by reading the USD prim hierarchy directly (`/World/UR5e/aic_unified_robot/gripper/tcp` relative to `/World/UR5e/aic_unified_robot/tool0`) and stored as `self._tool0_to_tcp_offset_xform` + `self._tcp_to_tool0_offset_xform`. On ingress (`_on_pose_cmd` / `_apply_pose_cmd`): if `MotionUpdate.header.frame_id == "gripper/tcp"`, the target is pre-multiplied by `self._tcp_to_tool0_offset_xform` to convert to a tool0 target before IK. On egress (`_publish_controller_state` in Plan 02-05): the FK result for tool0 is post-multiplied by `self._tool0_to_tcp_offset_xform` to populate `tcp_pose`. Plan 02-04 was revised to remove the prior "drop+log" fallback — the static-offset implementation is mandatory. Option B (vendoring a custom URDF that includes the gripper chain) remains a documented escalation path if a future trial reveals the rigid offset assumption breaks (e.g., if AIC ever publishes commands that interact with the finger DOFs directly).

## Sources

(Already enumerated above under Sources.)

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — every API verified against the installed Isaac Sim 5.0 filesystem on this host; one D-05 landmine identified and fix prescribed.
- Architecture: HIGH for Phase 2 first cut. The aic_controller-vs-Isaac-Sim coexistence question (Open Q2) is MEDIUM but doesn't block Phase 2 implementation — only Phase 4 E2E.
- Pitfalls: HIGH — 9 pitfalls identified, all with concrete avoidance strategies and warning signs. Several derived from Phase 1's empirical lessons (parity_publishers.py path-management, hot-reload safety, articulation root prim path).

**Research date:** 2026-05-03
**Valid until:** 2026-06-03 (30 days for stable APIs; revisit if Isaac Sim 5.1 or if a new aic_controller topic-name remap lands upstream)

## RESEARCH COMPLETE
