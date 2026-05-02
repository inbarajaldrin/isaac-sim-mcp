# isaac-sim-mcp / aic-dt — Claude Code on-ramp

This repo hosts Isaac Sim Kit extensions that expose MCP socket servers for agent-driven control of robot digital twins. Active development focuses on **`exts/aic-dt/`** — the digital twin for the AIC (AI for Industry Challenge) cable-insertion task. Other extensions (`ur5e-dt`, `soarm101-dt`) are sibling reference implementations.

> **Use the `isaac-sim-extension-dev` skill first** for anything Isaac Sim related — extension lifecycle, USD/physics, action graphs, MCP socket protocol, troubleshooting, the canonical asset workflow. The skill encodes hard-won knowledge (cooking deadlock workaround, post-play wedge anti-pattern, real-Kit-log location, etc.) that is the source of truth when this file goes stale.

## Where to look first

| When you want to … | Read |
|---|---|
| Understand intent, scope, constraints | `.planning/PROJECT.md` |
| See M1 deliverables vs deferred | `.planning/REQUIREMENTS.md` |
| See phased plan + per-phase success criteria | `.planning/ROADMAP.md` |
| See current execution state | `.planning/STATE.md` |
| Inventory the extension's MCP tools / UI | `MCP_TOOL_REGISTRY` dict at the top of `exts/aic-dt/aic_dt/extension.py` |
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

```bash
# Default path (after the cache has been primed at least once):
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
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py status
# Healthy: ≥100MB. Empty/wedged: <1MB. List of backups shown.

# Snapshot — take after any successful quick_start as cheap insurance:
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py snapshot

# Restore — your fix when cache is empty/corrupt (~2s, deterministic):
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py restore             # newest backup
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py restore known-good  # by name
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py restore 1777683559  # by timestamp

# Best-effort prime via postload + quick_start (warm-cache only — refuses on cold cache by default):
~/env_isaaclab/bin/python ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py prime aic-dt
```

**Operating discipline (do this, never violate):**

1. **NEVER** `mv DerivedDataCache aside; mkdir DerivedDataCache` for a "fresh test" without immediately running `restore` afterward. This single move broke a 90-minute debug session on 2026-05-01 — a verification test cleared the cache, never restored, the next session wasted a long time chasing "broken quick_start" when the only issue was the empty cache.
2. After any successful `quick_start`, run `prime_usd_cache.py snapshot`. Cheap, fast, prevents the next session's pain.
3. A stable named backup `~/.cache/ov/DerivedDataCache.bak.known-good` is set up — `prime_usd_cache.py restore known-good` is the deterministic recovery command. Rebuild it if you have a notably-improved cache state: `cp -r ~/.cache/ov/DerivedDataCache ~/.cache/ov/DerivedDataCache.bak.known-good`.
4. If `quick_start` hangs and you don't know why, **first** run `prime_usd_cache.py status`. If size is low, restore. Don't open the Kit log, don't kill processes, don't read this CLAUDE.md again — restore first, debug second.

The full background (why cold-cache cooking is broken, the cooking-deadlock + post-play-wedge pair, the bake_cable_fix.py diagnostic trail) lives in the `isaac-sim-extension-dev` skill — see its "Cache Management" section.

### Verified working flow (re-confirmed 2026-05-01)

```bash
# 1. Confirm cache is healthy
du -sh ~/.cache/ov/DerivedDataCache       # should be ≥100M

# 2. Launch
DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt
# → "READY (10s)"

# 3. quick_start (returns in ~5s; sim is playing afterwards with 387 prims)
python3 -c "
import socket, json
s = socket.socket(); s.settimeout(120)
s.connect(('localhost', 8768))
s.sendall(json.dumps({'type':'quick_start','params':{}}).encode())
data = b''
while True:
    c = s.recv(8192)
    if not c: break
    data += c
    try: json.loads(data.decode()); break
    except: continue
print(json.loads(data.decode())['result']['message'])
"
# Expected: "Quick start complete: AIC scene with UR5e, wrist cameras, task board objects, and simulation running."

# 4. (Iteration cycle: clear stage without restart)
python3 ~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py 8768 new_stage
# then re-run the quick_start block above
```

### Streaming wrapper (currently broken for aic-dt)

`~/bin/isaacsim` (interactive picker) calls `~/Documents/isaacsim-streaming/stream-local.sh --ext <name>`, which has a hardcoded `EXT_MCP_PORT` map for `soarm101-dt` (8767) and `ur5e-dt` (8766) but **does not yet include `aic-dt`** (8768). Running it for aic-dt errors with `unknown extension`. Either add `[aic-dt]=8768` to `EXT_MCP_PORT` in `stream-local.sh` or use one of the two paths above. Tracked as DX work for M1.

### Environment variables

| Var | Default | Purpose |
|---|---|---|
| `DISPLAY` | `:0` | X11 display for the Kit GUI window. Required. |
| `AIC_DT_EXT_FOLDER` | `/home/aaugus11/Documents/isaac-sim-mcp/exts` | Override hardcoded ext folder in `launch_postload.py`. Set when this repo lives elsewhere. |
| `MCP_CLIENT_OUTPUT_DIR` | unset | If set, the extension writes resources / saved scene state under `$MCP_CLIENT_OUTPUT_DIR/resources/`. Otherwise relative to `cwd`. |
| `MCP_SERVER_PORT` | `8768` (in code) | Socket port. Hardcoded as a constant in `extension.py` — change there, not via env. |

### Real Kit log (where the actual extension errors live)

```
~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log
```

The newest `kit_<timestamp>.log` is what to grep. Filter for `[MCP]`, `[py stdout]`, `Traceback`, and the `[AIC-DT]` print prefix the extension emits. **Staleness in this log = main loop blocked** (typical PhysX wedge symptom).

### MCP socket protocol

TCP, send a single JSON object, receive a single JSON object. No length prefix, no newline framing — server `json.loads` the buffer until it parses successfully.

```python
import socket, json
s = socket.socket(); s.connect(("127.0.0.1", 8768)); s.settimeout(120)
s.sendall(json.dumps({"type": "load_scene", "params": {}}).encode())
buf = b""
while True:
    chunk = s.recv(16384)
    if not chunk: break
    buf += chunk
    try: resp = json.loads(buf.decode()); break
    except json.JSONDecodeError: continue
print(resp); s.close()
```

Each MCP tool name + parameters lives in `MCP_TOOL_REGISTRY` (extension.py top). Handler bodies are `_cmd_<name>` methods on the `DigitalTwin` class.

### Verified MCP commands (as of 2026-05-01)

- `load_scene` — works, ~0.3s. Initializes physics scene, ground plane, AIC enclosure.
- `play_scene` / `stop_scene` / `new_stage` — work, instant.
- `quick_start` — **works in ~5s** when `DerivedDataCache` is populated. Result: 387 prims, sim playing (UR5e + Robotiq Hand-E + enclosure + 4 task-board objects + workspace camera). If it hangs at `load_robot` → see "Cache state matters" above; cache is almost certainly empty.
- `execute_python_code` — works. Use `result = <value>` (not `print()`) to return data; arbitrary Omniverse APIs available.
- Other atomic tools (`load_robot`, `setup_action_graph`, `setup_force_publisher`, `setup_wrist_cameras`, `add_objects`, `setup_pose_publisher`, `randomize_object_poses`, `randomize_lighting`, `save_scene_state`, `restore_scene_state`, `sync_real_poses`) — assume working as part of the `quick_start` chain; verify individually before relying. The current `objects_poses_sim` / `sync_real_poses` pair will be replaced with Gazebo-native topic names in Phase 1.

### Cable note

`load_robot` calls `cable_prim.SetActive(False)` on `/World/UR5e/cable` as a workaround — the 21-segment cable rope chain (mass=0/inertia=0 D6 joints) wedges PhysX simulation post-play when active. Cable subtree alone in any sim plays fine; the wedge only manifests inside the aic-dt extension Kit env. There's a partial USD-rebake attempt in the prior session's chat history (`bake_cable_fix.py`, never committed, marked NOT FUNCTIONAL by its own author) that documents the diagnostic trail. Final cable-physics fix is Phase 3 work (SCENE-05 in REQUIREMENTS.md).

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

---

## Next step

**Phase 1 — Foundation Parity** is up next. Run `/gsd-discuss-phase 1` (or `/gsd-ui-phase 1` for the design contract, `/gsd-plan-phase 1` to plan directly).
