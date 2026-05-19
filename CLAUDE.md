# isaac-sim-mcp / aic-dt — Claude Code on-ramp

This repo hosts Isaac Sim Kit extensions that expose MCP socket servers for agent-driven control of robot digital twins. Active development focuses on **`exts/aic-dt/`** — the digital twin for the AIC (AI for Industry Challenge) cable-insertion task. Other extensions (`ur5e-dt`, `soarm101-dt`) are sibling reference implementations.

> ## ⛔ Operating-policy pointers
>
> - **Autonomous M1 mode + sudo policy:** `.planning/AUTONOMOUS_MODE.md` (marker file `.planning/.autonomous_m1_active`; both that and `.planning/.m1_shipped` exist — block kept active because Plans 04-04/04-05 remain blocked on the kilted↔humble RMW issue in `HANDOFF.json`). The `SessionStart` hook auto-reads it.
> - **Skill use is mandatory** for any Isaac Sim 5.0 / OpenUSD / OmniGraph / Replicator / Warp / Isaac Lab / Isaac ROS API question — invoke `isaac-sim-extension-dev` (project gotchas, cache discipline, prim-path history, postload launcher) and `nvidia-suite-docs` (live NVIDIA docs) BEFORE guessing from training data. APIs evolve monthly; cached LLM knowledge is routinely stale. Execution-time recovery: real Kit log first, then `nvidia-suite-docs`, then `isaac-sim-extension-dev`, then propose a fix.
> - **Privileged commands (sudo) are passwordless** for `aaugus11` via `/etc/sudoers.d/aaugus11-nopasswd` (installed 2026-05-17). Call `sudo <cmd>` directly. Discipline: refuse-destructive, log cross-repo changes, prefer non-TTY flags. Detail in the user-global `~/.claude/CLAUDE.md`.

## Where to look first

| When you want to … | Read |
|---|---|
| Understand intent, scope, constraints | `.planning/PROJECT.md` |
| See M1 deliverables vs deferred | `.planning/REQUIREMENTS.md` |
| See phased plan + per-phase success criteria | `.planning/ROADMAP.md` |
| See current execution state | `.planning/STATE.md` |
| Inventory the extension's MCP tools / UI | `MCP_TOOL_REGISTRY` dict at the top of `exts/aic-dt/aic_dt/extension.py` |
| Author a thin USD wrapper around a new GLB asset (socket / plug / mount) | `exts/aic-dt/scripts/build_thin_glb_usds.py` — vendors GLB from `~/Documents/aic/aic_assets/models/<X>/` + writes thin USD that `AddReference`s the GLB; gltf SDF plugin handles axis / units / PBR (incl. multi-primitive `UsdGeomSubset` partitions) at Kit load time |
| Patch the cable USD's connector visuals + materials + kinematic flags + finger joints | `exts/aic-dt/scripts/build_cable_variant_usds.py` — `build_cable_variant()` is the entry; emits both `aic_unified_robot_cable_sdf.usd` (identity) and `…_reversed.usd` (cable_type swap). Helper `replace_plug_subtree_with_glb_refs()` is the canonical way to wire a thin GLB-USD into a kinematic-tracker-owned parent |
| Build / source the Python 3.11 ROS 2 Humble workspace that rclpy parity publishers need | `exts/aic-dt/docs/rclpy-setup.md` — clones NVIDIA's `IsaacSim-ros_workspaces` at the `IsaacSim-5.0.0-full` tag, builds in Docker, appends `local_setup.bash` + `LD_LIBRARY_PATH` into `~/env_isaaclab/bin/activate`. Sibling extensions have their own copy: `exts/ur5e-dt/docs/rclpy-setup.md`. Verify with `exts/aic-dt/scripts/verify_kilted_humble_interop.sh` |
| Isaac Sim hangs / freezes during `quick_start` (run freeze) | First: `prime_usd_cache.py status` — cold cache is the #1 cause (see the "Cache state matters" section in this file). Second: real Kit log path (`.planning/LAUNCH_REFERENCE.md` § Real Kit log) — staleness in the log = main loop blocked. Historical cooking-deadlock + post-play-wedge background lives in the `isaac-sim-extension-dev` skill's "Cache Management" section |
| Debug Isaac Sim launch / hang / asset issues | invoke the `isaac-sim-extension-dev` skill |

---

## Launching the Isaac Sim aic-dt extension

**Port:** 8768 (TCP, localhost). **Robot:** UR5e + Robotiq Hand-E + cable at (-0.18, -0.122, 0). **Window:** "AIC Digital Twin".

### The two-launcher reality

There are two launch paths and they handle different cache states:

| Path | Use when | What it runs | Cold-cache risk |
|---|---|---|---|
| **Lifecycle helper** | `DerivedDataCache` is populated (warm cache, normal day-to-day) | `isaacsim --ext-folder … --enable aic-dt` | hangs PhysX cooking if cache is empty |
| **Postload script** | `DerivedDataCache` is empty/stale, OR you just cleared it | boots Kit first, then enables aic-dt via `extension_manager.set_extension_enabled_immediate(...)` | safe; will cook from scratch if needed |

> **⚠️ ALWAYS source the venv before launch (Phase 1 PARITY-03/04 requirement).** The aic-dt extension's rclpy-based parity publishers (`/joint_states`, `/tf`, `/tf_static`) require the Python 3.11 ROS Humble workspace to be on `LD_LIBRARY_PATH` (so the workspace's `libgeometry_msgs__rosidl_generator_c.so` with the `polygon_instance_stamped` symbol wins over `/opt/ros/humble`'s older lib) AND on `PYTHONPATH` / `AMENT_PREFIX_PATH` (so message-type imports resolve to the 3.11 build, not the broken Python 3.10 build that Isaac Sim's startup-time bridge attempt loads). The launcher script invokes `isaacsim` directly without sourcing activate, so wrap the call: `bash -c 'source ~/env_isaaclab/bin/activate && DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'`. Without this, `quick_start` fails at `setup_tf_publish_action_graph` with `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'geometry_msgs'`. See `exts/aic-dt/docs/rclpy-setup.md` for the workspace build instructions.

```bash
# Default path (after the cache has been primed at least once):
# IMPORTANT: source ~/env_isaaclab/bin/activate first, or wrap the call:
bash -c 'source ~/env_isaaclab/bin/activate && DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'

# (Without the source, you can still launch but rclpy parity publishers fail.
#  Useful for sessions that don't need /joint_states + /tf parity, e.g. testing
#  the MCP socket surface or OGN nodes alone.)
DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt

# Other lifecycle commands:
bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh status
bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh kill
bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh restart aic-dt

# Cold-cache path (use this whenever the warm path wedges at quick_start, or after wiping caches):
~/env_isaaclab/bin/isaacsim --exec /home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/scripts/launch_postload.py
```

Why two paths exist: loading aic-dt during Kit's startup boot sequence wedges PhysX UJITSO cooking on the unified robot USD when the cooked-SDF cache is empty (`reset_async` blocks on `futex_wait`, no progress, MCP socket opens but `quick_start` hangs at `load_robot`). The postload script avoids this by booting Kit normally first, pumping 60 ticks, then enabling the extension. See the docstring in `exts/aic-dt/scripts/launch_postload.py` for the original bisection trail.

Bootstrap log for postload: `/tmp/aic_dt_postload.log`. Lifecycle helper log: `/tmp/isaacsim.log`.

### ⚠️ Cache state matters more than the launcher choice — read this first

**Single biggest cause of "quick_start hangs at load_robot": an empty or stale `~/.cache/ov/DerivedDataCache`.** The cooked SDF for the cable subtree lives there; if it's gone, cooking has to redo from scratch inside the running extension and that's the wedge path. **Cold-cache cooking is currently broken in Isaac Sim 5.x** with the cable+UR5e USD — verified empirically on 2026-05-01 across every launch path. The reliable model is **snapshot-and-restore**, not re-cook.

There is one tool for all cache management. Use it before debugging anything else:

```bash
# Always check first when quick_start misbehaves:
~/env_isaaclab/bin/python scripts/prime_usd_cache.py status
# Healthy: ≥100MB. Empty/wedged: <1MB. List of backups shown.

# Snapshot — take after any successful quick_start as cheap insurance:
~/env_isaaclab/bin/python scripts/prime_usd_cache.py snapshot

# Restore — your fix when cache is empty/corrupt (~2s, deterministic):
~/env_isaaclab/bin/python scripts/prime_usd_cache.py restore             # newest backup
~/env_isaaclab/bin/python scripts/prime_usd_cache.py restore known-good  # by name
~/env_isaaclab/bin/python scripts/prime_usd_cache.py restore 1777683559  # by timestamp

# Best-effort prime via postload + quick_start (warm-cache only — refuses on cold cache by default):
~/env_isaaclab/bin/python scripts/prime_usd_cache.py prime aic-dt
```

**Operating discipline (do this, never violate):**

1. **NEVER** `mv DerivedDataCache aside; mkdir DerivedDataCache` for a "fresh test" without immediately running `restore` afterward. This single move broke a 90-minute debug session on 2026-05-01 — a verification test cleared the cache, never restored, the next session wasted a long time chasing "broken quick_start" when the only issue was the empty cache.
2. After any successful `quick_start`, run `prime_usd_cache.py snapshot`. Cheap, fast, prevents the next session's pain.
3. A stable named backup `~/.cache/ov/DerivedDataCache.bak.known-good` is set up — `prime_usd_cache.py restore known-good` is the deterministic recovery command. Rebuild it if you have a notably-improved cache state: `cp -r ~/.cache/ov/DerivedDataCache ~/.cache/ov/DerivedDataCache.bak.known-good`.
4. If `quick_start` hangs and you don't know why, **first** run `prime_usd_cache.py status`. If size is low, restore. Don't open the Kit log, don't kill processes, don't read this CLAUDE.md again — restore first, debug second.

The full background (why cold-cache cooking is broken, the cooking-deadlock + post-play-wedge pair, the bake_cable_fix.py diagnostic trail) lives in the `isaac-sim-extension-dev` skill — see its "Cache Management" section.

### Reference details (env vars, Kit log path, MCP socket protocol, verified-working recipe, streaming wrapper)

Moved to `.planning/LAUNCH_REFERENCE.md`. Key facts that need to stay top-of-mind:
- **`ROS_DOMAIN_ID=7` is mandatory** for any Isaac Sim launch + smoke test — the user has a live UR5e driver on domain 0 that competes on `/joint_states` and breaks Phase 1 smoke (discovered 2026-05-05). Always wrap.
- **Real Kit log:** `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log`. Staleness = main loop blocked. Grep for `[MCP]`, `[py stdout]`, `Traceback`, `[AIC-DT]`.
- **MCP socket protocol:** single JSON object in, single JSON object out over TCP (no framing). `json.loads` until the buffer parses. Code example in the reference file.

### MCP command surface

`MCP_TOOL_REGISTRY` at the top of `exts/aic-dt/aic_dt/extension.py` is the authoritative list — it rots faster than any prose summary. Handler bodies are `_cmd_<name>` methods on the `DigitalTwin` class. Quick orientation:

- **Lifecycle (always work):** `load_scene` (~0.3s), `play_scene` / `stop_scene` / `new_stage` (instant), `quick_start` (~5s warm cache → 387 prims, sim playing), `execute_python_code` (use `result = <value>`, not `print()`).
- **Trial loading:** `load_trial(trial_key='trial_1'|'trial_2'|'trial_3', ground_truth=True)` — reads `~/Documents/aic/aic_engine/config/sample_config.yaml`, wipes stage, runs `load_scene` + `load_robot` + per-trial spawn atoms + cable wiring. Main quick-test path now that the trial loader is wired.
- **Held-connector control:** `gripper_command(position=…)` writes both finger prismatic drives symmetrically (URDF mimic at write); `attach_cable_to_gripper` installs the per-tick TCP tracker for the held connector pose.
- If `quick_start` hangs at `load_robot` → cache is almost certainly empty. See "Cache state matters" above. Restore first, debug second.

### Cable note (REVISED 2026-05-17)

The cable USD is built by `build_cable_variant_usds.py` from a vendored source into two variants: `aic_unified_robot_cable_sdf.usd` (identity) and `…_reversed.usd` (cable_type swap for trial_3-style configs). The 21-link rope chain is anchored at both ends via identity-`localRot0` `fixedJoint`/`fixedJoint2` to the two kinematic connector parents (`/World/cable/sc_plug_visual` and `/World/cable/sfp_module_visual`).

**Held-connector pose:** owned by `_install_held_connector_tcp_tracker` (per-tick `xformOp:translate`/`orient` write to the held connector parent, composing TCP × cable_type-specific `rel_quat`). Far-end pose: owned by `_install_far_connector_world_tracker`. Both write USD + Fabric per the dedup'd `_apply_kinematic_world_pose` helper.

**Plug visuals (b059074 + feb53b8):** `sc_plug_visual` / `sfp_module_visual` subtrees are replaced at build time by `replace_plug_subtree_with_glb_refs()` with thin-USD references to `assets/assets/{SC Plug,SFP Module,LC Plug}/*_visual.usd` (built by `build_thin_glb_usds.py`). Isaac Sim's gltf SDF plugin auto-synthesizes `UsdGeomSubset` partitions for multi-primitive GLB meshes — SFP body's `Body_005` mesh gets 6684 + 184 face subsets for Material_005 + Material_001 with no manual fallback needed.

**Gripper width:** Hand-E fingers converted to `PhysicsPrismaticJoint` + `DriveAPI` (commit `3c1b754`); `gripper_command` MCP atom (`e5b4bee`) writes both targets symmetrically per URDF mimic.

**Still latent (SCENE-05, M2):** rope links have `mass=0` / `inertia=0` so PhysX treats them kinematically — the cable is decorative for dynamics (no swing/sag), but the held-connector tracker keeps the relevant plug visually + scoring-wise correct. The cooking-deadlock wedge that originally motivated `SetActive(False)` no longer reproduces under current launch conditions (venv-activate + warm cache + postload when needed); historical bisection in chat session 29ca157f if it returns.

---

## Launching the AIC Gazebo reference (the platform we're matching)

**This repo is sim-side only.** The ROS-side competition stack (controller, engine, policies, asset URDFs) lives at **`~/Documents/aic`** — see "Cross-repo relationship" below. The canonical "what should aic-dt look like when it works" reference comes from running CheatCode against Gazebo from the AIC repo.

### Run CheatCode end-to-end (Docker, headless)

```bash
cd ~/Documents/aic
./scripts/run_cheatcode.sh headless         # without GUI (faster)
./scripts/run_cheatcode.sh                  # with Gazebo GUI (needs DISPLAY + xhost)
./scripts/run_cheatcode.sh stop             # tear down both containers
```

This brings up two containers via `docker run`:
- **`aic_eval`** (image: `ghcr.io/intrinsic-dev/aic/aic_eval:latest`) — Gazebo + `aic_engine` + `aic_controller` + ros_gz_bridge + scoring. Args: `ground_truth:=true start_aic_engine:=true gazebo_gui:=<bool>`.
- **`aic_model`** (image: `my-solution:v1`, built locally from `docker/aic_model/Dockerfile`) — runs the participant policy. Started after a 25s sleep. Args: `policy:=aic_example_policies.ros.CheatCode use_sim_time:=true`.

Wrapper log: `/tmp/aic-gazebo.log` (or whatever you tee to). Container logs: `docker logs aic_eval` / `docker logs aic_model`.

### Why Docker, not native pixi

`docker/aic_model/Dockerfile` builds on `aic_eval` and runs ROS 2 Kilted Kaiju + the participant policy. **Native `pixi install` fails on this host (Ubuntu 22.04, GLIBC 2.35 < 2.39 required by `pixi-build-ros`).** Docker is the only working path until the host is upgraded to Ubuntu 24.04+. See `~/Documents/aic/CLAUDE.md` for the AIC repo's own development workflow.

### Inspecting Gazebo's actual topic surface (for parity work)

Once `aic_eval` is up:

```bash
docker exec aic_eval bash -c 'source /opt/ros/kilted/setup.bash && ros2 topic list'
docker exec aic_eval bash -c 'source /opt/ros/kilted/setup.bash && ros2 topic info /joint_states'
docker exec aic_eval bash -c 'source /opt/ros/kilted/setup.bash && ros2 topic echo /tf --once'
```

> ⚠️ **Important discovery (2026-05-01).** The topic names in `aic_engine/config/sample_config.yaml`'s `scoring.topics` block are **not** what the running eval container actually publishes. PROJECT.md and REQUIREMENTS.md were drafted off the YAML. **The live topic surface is the ground truth** — Phase 1's PARITY work must use `ros2 topic list` against a running `aic_eval` container, not the YAML. Notable divergences observed live:
> - F/T topic is `/force_torque_sensor_broadcaster/wrench`, **not** `/fts_broadcaster/wrench`.
> - Object poses come over `/objects_poses_real` (TFMessage), **not** `/tf` directly with per-object frames.
> - Grasp points: `/grasp_points_real` (MarkerArray).
> - Camera: a single `/intel_camera_rgb_raw` (sensor_msgs/Image), not three named wrist topics.
> - Robot control: standard ros2_control surface (`/joint_trajectory_controller/*`, `/scaled_joint_trajectory_controller/*`, `/forward_{position,velocity,effort}_controller/*`, `/force_mode_controller/*`, `/freedrive_mode_controller/*`, `/passthrough_trajectory_controller/*`, `/tool_contact_controller/*`, `/io_and_status_controller/*`, `/tcp_pose_broadcaster/pose`).
> - Gripper: `/gripper_command` (String), `/gripper_status` (String), `/gripper_width` (Float32), `/gripper_grasp_detected`, `/gripper_motion_ongoing`, `/gripper_width_offset`.
> - **No** topics under `/aic_controller/*`, `/aic/gazebo/*`, or `/scoring/*` were observed live (despite being listed in `sample_config.yaml`).
>
> Capture a fresh snapshot of the live surface as the first concrete deliverable of Phase 1 (PARITY-12 = topic-surface audit).

---

## Cross-repo relationship

This repo is **sim-side only**. The matching **ROS-side** repo lives at `~/Documents/aic` (the AIC competition toolkit). It is consumed read-only — no modifications to fit Isaac Sim.

| AIC package | Role | Consumed by aic-dt as |
|---|---|---|
| `aic_controller/` | C++ ROS 2 impedance controller (`aic_controller_plugin.xml`) | Unmodified consumer of `/joint_states`, `/tf`, `/wrench` etc. |
| `aic_engine/` | C++ trial orchestrator; `config/sample_config.yaml` defines trials | Unmodified — runs against Isaac Sim once topic parity holds |
| `aic_example_policies/aic_example_policies/ros/` | `CheatCode.py`, `GentleGiant.py`, `RunACT.py`, `SpeedDemon.py`, `WallPresser.py`, `WallToucher.py`, `WaveArm.py` | Unmodified |
| `aic_description/`, `aic_assets/` | URDFs + meshes for UR5e + Robotiq Hand-E + task board + cables | Same files loaded into Isaac Sim |
| `aic_bringup/launch/` | Gazebo bringup (`aic_gz_bringup.launch.py`, `spawn_task_board.launch.py`, `spawn_cable.launch.py`) | Mirrored as **outcomes** (scene state + topic surface), not file structure |

**Topic-parity is the architectural law.** Isaac Sim publishes the *exact* topic names the running Gazebo `aic_eval` container publishes — no `_sim`/`_real` discrimination beyond what Gazebo itself uses, no remap nodes, no bridge translators. The same `aic_controller` + `aic_engine` + `CheatCode.py` invocations that work against Gazebo must work against Isaac Sim with zero changes in `~/Documents/aic`.

**Scene-frame parity is also architectural law — and is a multi-entity reconciliation, not a per-entity fix.** When changing the world-frame placement of ANY scene entity (robot mount, task board, enclosure, ground plane, cable), you MUST walk the full scene-authoring chain in `exts/aic-dt/aic_dt/extension.py` and reconcile **every** sibling entity + environment constant in the same change. The chain is:
```
load_scene → _setup_world_scene (ground_plane_z, physics)
           → import_enclosure (_enclosure_position)
load_robot (robot mount pose + RPY)
load_trial / quick_start → spawn_task_board_base → spawn_{lc,sfp,sc}_mount_rail
                        → spawn_sc_port → spawn_nic_card_mount → cable
```
For each entity, cross-reference the Isaac Sim constant against its Gazebo equivalent in:
- `~/Documents/aic/aic_description/world/aic.sdf` (world layout, includes)
- `~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py` (DeclareLaunchArgument defaults for robot/task_board/cable poses)
- `~/Documents/aic/aic_description/urdf/task_board.urdf.xacro` (per-part anchor origins)
- `~/Documents/aic/aic_engine/config/sample_config.yaml` (per-trial scene overrides)

**Concrete failure that triggered this rule (2026-05-11):** cf27bdd moved the robot from `(0,0,0)` to Gazebo's canonical `(-0.2, 0.2, 1.14, yaw=-π)` but left `_enclosure_position=(0,0,-1.15)` and `_ground_plane_z=-0.08` and stale task-board-part anchors untouched. Robot ended up floating 1.22m above the ground plane with the enclosure ceiling intersecting its reach envelope. Working-tree diff that surfaced part-anchor errors did NOT include the enclosure/ground/task-board-default corrections — orchestrator (me) tunneled on the visible diff and missed the upstream constants. See `.tug/findings/world-frame-reconcile.md` for the full divergence table + Phase-B reconciliation plan.

The aic-dt extension MAY use `rclpy` for non-control glue (config reads, scoring event listening). The `~/Documents/aic` repo MUST NOT import `omni.*`.

---

## GSD workflow

This project is managed with `/gsd-*` slash commands. Standard flow:

```
/gsd-discuss-phase N     # gather phase context interactively
/gsd-plan-phase N        # produce PLAN.md (research-driven, plan-checked)
/gsd-execute-phase N     # execute the plan with atomic commits
/gsd-verify-work N       # confirm deliverables against success criteria
/gsd-progress            # status overview at any time
```

Workflow config (`.planning/config.json`): `mode=yolo`, `granularity=coarse`, `model_profile=quality`, research / plan-check / verifier all on, sequential execution. Change via `/gsd-settings`.

### Operating policies (phase scope, forward-pull, closure discipline)

Three rules that govern how phases scope, enforce, and close — each one is the "lesson learned" form of a real incident. Full text + enforcement specs in **`.planning/POLICIES.md`**. The TL;DRs:

- **Phase scope is by surface, not capability.** Before planning, scan future-phase requirements; pull one forward if its primary edit surface overlaps + dependencies met + no new research needed. The roadmap is the *plan*, not a *contract*.
- **Forward-pull is enforced at three checkpoints, not a guideline:** mandatory `## Forward-pull scan` block in CONTEXT.md (defended even when empty), plan-checker cross-checks for surface overlap, executor halts on surface-touch surprise. Surfaced 2026-05-08 after Phase 1's extension.py surface got re-edited by 2 + 3 + 4 without anyone scanning.
- **Phase closure has a backlog sweep + runtime payload sanity gate.** Closure ceremony updates the REQUIREMENTS.md traceability table atomically with STATE.md + ROADMAP. Verifiers sample real payload (not just rate/frame_id) — PARITY-05/09/10/12 all over-claimed because structural-only gates passed while payloads were zero/missing.
- **Status-reading discipline:** REQUIREMENTS.md inline checkboxes are primary truth; the traceability table is derived (cross-check); STATE.md is a phase-boundary snapshot; HANDOFF.json is live; runtime probe is ground truth when in doubt.

---

## Next step

**Phase 1 — Foundation Parity** is up next. Run `/gsd-discuss-phase 1` (or `/gsd-ui-phase 1` for the design contract, `/gsd-plan-phase 1` to plan directly).
