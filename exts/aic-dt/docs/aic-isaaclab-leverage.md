# AIC Isaac Lab Env — Leverage Assessment for aic-dt M1

**Task:** `isaaclab-leverage-research` (PRD `plans/prd.json`).

**TL;DR:** AIC ships a real Isaac Lab env at `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/` registered as the gym task `AIC-Task-v0`. It is **not** a substitute for aic-dt's M1 ship-gate path (it has zero ROS surface — no rclpy, no `/joint_states`, no `/tf`, no scoring topics). **However**, it carries authoritative parameter values + scene-authoring contracts that directly answer three outstanding aic-dt M1 runtime-gate tasks. **Verdict: leverage M2 for the env itself; leverage NOW for the values + contracts** — extract into existing M1 tasks as forward-information.

## Located at

`/home/aaugus11/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/`

Layout (committed in the AIC repo):

```
aic_isaaclab/
├── README.md                          ← workflow doc (Docker, Isaac Lab 2.3.2)
├── pyproject.toml
├── scripts/
│   ├── list_envs.py                   ← ls registered AIC-Task-v0
│   ├── teleop.py                      ← keyboard / spacemouse / gamepad teleop
│   ├── record_demos.py / replay_demos.py
│   ├── random_agent.py / zero_agent.py
│   └── rsl_rl/{train,play,cli_args}.py  ← RL training entrypoints
└── source/aic_task/                   ← installable Python pkg `aic_task`
    └── aic_task/
        ├── tasks/manager_based/aic_task/
        │   ├── aic_task_env_cfg.py    ← THE config (scene + actions + rewards)
        │   ├── agents/rsl_rl_ppo_cfg.py
        │   └── mdp/{events,observations,rewards}.py
        └── Intrinsic_assets/          ← USD pack (robot + scene + parts)
            ├── aic_unified_robot_cable_sdf.usd
            ├── scene/aic.usd
            └── assets/{SC Port, NIC Card, Task Board Base, …}
```

Trained RL checkpoints (multiple `model_*.pt` runs from 2026-03-20) are present under `scripts/rsl_rl/logs/`, confirming the env was actually run end-to-end on the Intrinsic team's training hardware.

## What it contains

### Scene composition (`AICTaskSceneCfg` in `aic_task_env_cfg.py`)

| Asset | API | USD source | Notes |
|-------|-----|------------|-------|
| Robot | `ArticulationCfg` at `{ENV_REGEX_NS}/Robot` | `Intrinsic_assets/aic_unified_robot_cable_sdf.usd` | Same USD pack as aic-dt; `init_state.pos=(-0.18,-0.122,0)` matches aic-dt |
| Joint init | `init_state.joint_pos` | shoulder_pan=0.1597, shoulder_lift=−1.3542, elbow=−1.6648, wrist_1=−1.6933, wrist_2=1.5710, wrist_3=1.4110 | **Canonical home pose** — gravity-stable, board-facing |
| Arm drives | `ImplicitActuatorCfg` | `stiffness=2000.0, damping=100.0, effort_limit_sim=87.0` | **Canonical PD gains + max effort** — matches `joint-drives-urdf-reconcile` acceptance exactly |
| Articulation root | `ArticulationRootPropertiesCfg` | `enabled_self_collisions=True, solver_position_iteration_count=16, solver_velocity_iteration_count=8` | **Canonical solver iter counts** — note this is *fewer* than the 32+8 the motion-deficit-hunt iteration was guessing |
| Cable | **commented out** | — | Cable explicitly excluded from RL env — confirms cable physics is decorative for trial scoring |
| Scene world | `AssetBaseCfg` | `Intrinsic_assets/scene/aic.usd` | Workshop enclosure |
| Task board base | `RigidObjectCfg(kinematic_enabled=True)` | `assets/Task Board Base/task_board_rigid.usd` | **Canonical authoring** for `taskboard-prim-authoring` task |
| `sc_port` / `sc_port_2` | `RigidObjectCfg(kinematic_enabled=True)` | `assets/SC Port/sc_port.usd` | Same authoring contract |
| `nic_card` | `RigidObjectCfg(kinematic_enabled=True)` | `assets/NIC Card/nic_card.usd` | Same authoring contract |
| Cameras (3) | `TiledCameraCfg` | resolution 224×224, ROS convention | Prim paths: `{ENV_REGEX_NS}/Robot/aic_unified_robot/{left,center,right}_camera_optical/{...}_camera` — **canonical paths for `wrist-cameras-restore`** |
| Camera intrinsics | `PinholeCameraCfg` | `focal_length=22.48, focus_distance=0, horizontal_aperture=20.955, vertical_aperture=18.627, clipping_range=(0.07, 20.0)` | Per-camera intrinsics for the 3 wrist cams |
| Light | `DomeLightCfg` | `intensity=2500, color=(0.75,0.75,0.75)` | + `randomize_dome_light` event over (1500,3500) lumens |

### MDP layer

- **Action space:** `DifferentialInverseKinematicsActionCfg` (svd IK on `wrist_3_link`, scale=0.05, relative pose mode). Single arm action term; gripper action commented out.
- **Observation:** joint_pos_rel, joint_vel_rel, body_pose_w(wrist_3_link), body_incoming_wrench on 7 arm bodies, ResNet-18 features on each wrist camera RGB, last_action, ee_pose command. Configured for vision-feature policy training, not raw RGB.
- **Rewards:** position tracking (L2, tanh, exp kernels) + orientation tracking + sparse reaching bonus + smoothness penalties (action_rate, joint_vel/acc/torques) + joint pos limits.
- **Events / Domain randomization:** `randomize_dome_light` (intensity + color), `randomize_board_and_parts` (board pose ±5mm, sc_port range ±5–20mm in x, nic_card snap step 0.04 in y). Reset event resets joints with offset ±0.05.
- **Termination:** time_out only (`episode_length_s=200.0`, `decimation=4`, `sim.dt=1/120`).
- **Custom contact-sensor obs function** (`mdp/observations.py::contact_net_forces`) — pattern for reading `ContactSensor.data.net_forces_w` filtered by body_ids.

### Runtime architecture

- Runs inside `isaac-lab-base` Docker container (Isaac Lab 2.3.2, NVIDIA-prepared). Built from `~/IsaacLab/docker/container.py build base`.
- Entrypoint is `isaaclab -p <script>.py --task AIC-Task-v0 --num_envs N` — *not* the `isaacsim` Kit launcher that aic-dt uses.
- Uses `ManagerBasedRLEnv` step-driven semantics (`env.step(action)` → physics for `decimation` substeps). No persistent process; no socket interface.
- No ROS surface anywhere: `grep -r 'rclpy\|ros2\|/joint_states' aic_isaaclab/` returns 0 hits.

## Overlap with aic-dt

| Capability / surface | aic-dt (current) | aic_isaaclab | Overlap kind |
|----------------------|------------------|--------------|--------------|
| USD asset pack | Same `Intrinsic_assets/` (loaded via `load_robot` MCP atom) | Same `Intrinsic_assets/` (loaded via `UsdFileCfg`) | **Identical source** |
| Robot init pose | `(-0.18,-0.122,0)` | `(-0.18,-0.122,0)` | **Identical** |
| Joint home pose | aic-dt loads from USD default | Explicit dict in `init_state.joint_pos` | **aic_isaaclab is canonical** — copy these into `load_robot` post-init |
| Arm drives (stiffness/damping/effort) | Inherited from on-disk USD OEM defaults (stiffness 95→1322, damping 0.038→0.529, maxForce 28 or 150) | `stiffness=2000, damping=100, effort_limit_sim=87` | **MISMATCH — aic_isaaclab is canonical** for `joint-drives-urdf-reconcile` |
| Solver iter counts | Default (motion-deficit-hunt tried 32+8, regressed) | `pos_iter=16, vel_iter=8` | **aic_isaaclab is canonical** — 16+8 not 32+8 |
| Cable activation | Active in scene (decorative — mass=0/inertia=0; expanded articulation 6→46 DOFs) | **Commented out** | **aic_isaaclab confirms cable is OOS for the trial** — supports `motion-deficit-hunt` HYP-A direction (skip cable activation in load_robot) |
| Task-board kinematic state | RigidBodyAPI + mass=0 + density=0 (silent kinematic) | `kinematic_enabled=True` (explicit) | **aic_isaaclab confirms option (b)** for `taskboard-prim-authoring` (per PRD line 281) |
| Wrist cameras | Setup logs "Setup 0/3" — Camera prims not authored | TiledCameraCfg at `…/{l,c,r}_camera_optical/{...}_camera` with ROS convention + `(focal_length=22.48, h_aperture=20.955, v_aperture=18.627)` intrinsics | **aic_isaaclab is canonical** for `wrist-cameras-restore` prim paths + intrinsics |
| Topic surface (`/joint_states`, `/tf`, `/wrench`, `/scoring/*`) | Implemented via parity_publishers + OmniGraph | **NONE** | aic_isaaclab does not solve M1 ship-gate |
| MCP socket / scriptable atoms | Yes (port 8768) | No | aic-dt uniquely solves agent-driven scene-authoring |
| aic_engine / CheatCode integration | Yes (run_aic_engine_against_isaac_sim.sh + zenoh) | No | aic_isaaclab does not interface with aic_engine |
| RL training pipeline | None | rsl_rl PPO config + 5 trained model checkpoints | **aic-dt does not need this for M1** |
| Teleop UI | None | Keyboard/Spacemouse/Gamepad via Se3*Cfg | **aic-dt does not need this for M1** |
| Domain randomization | Limited (lighting only) | Lighting + board/parts pose + joint reset offset | **Future M2 leverage** for trial robustness |
| Contact sensing | Custom OmniGraph fallback for /scoring/insertion_event | `ContactSensor` + `mdp.contact_net_forces` pattern | aic_isaaclab pattern is cleaner; could replace OmniGraph contact path in M2 |

## Verdict

**Leverage M2** for the env itself.
**Leverage NOW** for the values + scene-authoring contracts.

### Why not "leverage now" for the env

Re-rooting aic-dt on `aic_isaaclab` for M1 would require, all of:

1. Building Isaac Lab 2.3.2 Docker image (`~/IsaacLab/docker/container.py build base`).
2. Exporting full ROS topic surface (`/joint_states`, `/tf`, `/wrench`, `/scoring/insertion_event`, `/aic/gazebo/contacts/off_limit`, optionally `/wrist_camera_*/image_raw`) from inside the `ManagerBasedRLEnv` step loop. Isaac Lab does not natively publish these — the entire parity_publishers + OmniGraph wiring would need to be re-implemented inside the env's step callback.
3. Bridging from inside the Docker container to the host zenoh router (cross-container DDS/zenoh discovery) so that `aic_engine` + the `my-solution:v1` model container can both reach Isaac Sim's topic surface. This is *additional* zenoh work on top of what `zenoh-path-implementation` already shipped.
4. Re-implementing the `aic_controller` step driver. `aic_engine` + `CheatCode` issue continuous joint/Cartesian commands at controller rate; `ManagerBasedRLEnv.step()` is a per-decimation callback expecting one action per step. Adapting requires either: (a) a thin "always step zero-action while letting the host-side aic_controller drive joints directly via Articulation TensorAPI" — which means we are *not* using the env's MDP at all, or (b) bridging aic_controller commands into `env.step()` actions, which fights the env design.
5. The `requires_sim:true` runtime gates (`motion-deficit-hunt`, `pose-roundtrip-verify`, `wrench-rootcause`, `taskboard-prim-authoring`, `wrist-cameras-restore`) all live in aic-dt's `extension.py` / `controller_loop.py` / `parity_publishers.py` and are scoped to debugging *that* surface. Re-rooting to aic_isaaclab does not eliminate these; it transmutes them into a parallel set of bugs in a new architecture under a new container — and burns the zenoh + wrapper-json-emit work shipped this milestone.

In short: **aic_isaaclab solves a different problem.** It is a training/eval env for the participant's RL/IL policy on the same physical task. It is not a digital-twin substitute for the running Gazebo-equivalent surface that aic_engine + CheatCode require for M1 ship-gate.

### Why "leverage M2"

After M1 ships, three concrete forward-pull opportunities exist:

- **M2 trial robustness:** absorb `randomize_board_and_parts` + `randomize_dome_light` events into aic-dt's per-trial reset path so the parity report exercises domain-randomization that matches what the participant policy will face during eval.
- **M2 cable physics:** the `aic_isaaclab` env's commented-out cable + the trained PPO checkpoints suggest Intrinsic chose to ship the M1 challenge with cable-as-decorative. If a future trial type requires real cable physics, aic_isaaclab's cable-handling pattern (or its absence) is the canonical reference for what "good enough" looks like.
- **M2 teleop / RL surface for aic-dt:** aic-dt could expose a teleop-equivalent path for human-in-the-loop trials by adding a `Se3KeyboardCfg`-style command source to `controller_loop.py`. Out of M1 scope.

### What to leverage NOW (forward-info into existing M1 tasks)

These are not new tasks — they are **canonical default values** to pull into existing-task implementations:

| Existing M1 task | Forward-info from aic_isaaclab |
|------------------|-------------------------------|
| `joint-drives-urdf-reconcile` | Set `effort_limit_sim=87.0, stiffness=2000.0, damping=100.0` on the 6 arm joints **post articulation init**, via `Articulation.set_gains(stiffnesses=[2000]*6, dampings=[100]*6)` + `Articulation.set_max_efforts([87]*6)` (or whatever 5.0 setter API exposes). The PRD acceptance text already says these — aic_isaaclab confirms them as the AIC team's source-of-truth. **Solver iter counts: pos=16, vel=8** (not 32+8 as motion-deficit-hunt was guessing). |
| `taskboard-prim-authoring` | Pick **option (b)** per PRD line 281 — `kinematicEnabled=true` on `RigidBodyAPI`. AIC team chose this in their Isaac Lab env; mirror it. Live PhysX 5.0 contact-report semantics: `kinematicEnabled=true` preserves contact reporting, makes intent explicit, no risk of unintended dynamics. |
| `wrist-cameras-restore` | Author 3 Camera prims at: `{ENV_REGEX_NS}/Robot/aic_unified_robot/left_camera_optical/left_camera`, `…/center_camera_optical/center_camera`, `…/right_camera_optical/right_camera` (in aic-dt: `/World/UR5e/aic_unified_robot/{l,c,r}_camera_optical/{l,c,r}_camera`). Camera intrinsics: `focal_length=22.48, focus_distance=0, horizontal_aperture=20.955, vertical_aperture=18.627, clipping_range=(0.07, 20.0)`. Quaternion convention: `(1.0, 0.0, 0.0, 0.0)` with `convention="ros"` — i.e. ROS-frame +X-forward (Isaac Sim's TiledCameraCfg handles the −Z→+X conversion when `convention="ros"`). |
| `motion-deficit-hunt` | `aic_isaaclab` uses `ImplicitActuatorCfg` which sets gains via `Articulation.write_joint_*` PhysX TensorAPI — that's the same API path aic-dt already uses. **The missing ingredient is `effort_limit_sim=87` set BEFORE articulation init** (USD-side, on the DriveAPI's `maxForce` attribute). Once joint drives match aic_isaaclab's values, the 88%-coverage finding from the second motion-deficit-hunt iter likely closes the rest of the way (no 32+8 over-iteration, no gravity-FF heroics needed — Isaac Lab gets full coverage on the same robot with stiffness=2000/damping=100/effort=87). HYP-A (cable disabled) is *also* corroborated — the env explicitly comments cable out for RL training. |

These extractions do **not** introduce a new task — they are forward-info to be applied when `joint-drives-urdf-reconcile` / `taskboard-prim-authoring` / `wrist-cameras-restore` / `motion-deficit-hunt` are picked in subsequent iterations.

## Rationale

`aic_isaaclab` exists because the AIC team built it for the *participant's* training pipeline (RL, IL, teleop on the same physical task that `aic_engine` evaluates). It is not a competitor or substitute for `aic-dt`; the two have orthogonal concerns:

- `aic-dt`: present a Gazebo-faithful ROS topic surface that lets unmodified `aic_engine` + `aic_controller` + `CheatCode` policy run against Isaac Sim.
- `aic_isaaclab`: present a vectorized Manager-Based RL env that lets participants train their own policies (no ROS, no `aic_engine`).

What `aic_isaaclab` *does* give us — and what justifies this research spike paying for itself — is a **second source-of-truth** beyond the AIC URDF + Gazebo configs for canonical parameter values. The M1 audit that produced `joint-drives-urdf-reconcile` (audit finding #4) and `taskboard-prim-authoring` (audit finding #6) flagged divergences from URDF spec but didn't have a confirming witness for what the *Intrinsic-team-canonical* values are. `aic_isaaclab` is that witness. Three runtime-gate tasks now have unambiguous target values.

The fact that AIC commented out the cable in their RL env is the most useful single signal: **the cable is decorative for the M1 trial set in aic_isaaclab as well**, which independently corroborates PRD line 8's `out_of_scope_for_m1` cable note and motion-deficit-hunt's HYP-A direction (cable activation expanded articulation 6→46 DOFs and is the suspected source of constraint propagation that throttles arm acceleration).

No PRD restructure is needed. **No follow-up note appended to plans/progress.txt** beyond the standard task-closure entry, because the verdict is "leverage M2" not "leverage now" for the env itself, and the parameter extractions are absorbed into existing tasks rather than spawning new ones.

## References

- `~/Documents/aic/aic_utils/aic_isaac/README.md` — workflow + asset placement
- `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/aic_task_env_cfg.py` — the env config (640 lines, all values cited above)
- `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/mdp/{events,observations,rewards}.py` — domain randomization + obs + reward terms
- Isaac Lab 2.3.2 [docs](https://isaac-sim.github.io/IsaacLab/main/index.html) — `ManagerBasedRLEnvCfg`, `ImplicitActuatorCfg`, `DifferentialInverseKinematicsActionCfg` API surface
