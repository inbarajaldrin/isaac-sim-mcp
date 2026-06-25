# CLAUDE.md — isaac-sim-mcp runtime & operations

Project memory for agents working in this repo. Captures the **verified** way to launch Isaac Sim
with an extension, the ROS domain rule, and the **open ROS2-driver gap** that currently blocks
fully-automated robot motion. Extend the TODO section as the driver-spawn method is figured out —
the goal is to drive it from isaac-sim-mcp itself.

> **⚠️ ALSO READ `.local/context.md`.** This repo is now being driven for the **GRASP LOOP**
> (manipulation: move_to_grasp / close / lift, with the 15 mm Quick-Changer adapter in the chain) in
> collaboration with the **aruco-runner** Mac session — a use beyond this repo's original twin-loading
> purpose. `.local/context.md` is the operational guide for launching + driving that grasp stack and
> the open grasp-offset task. Read **both** files. (Latest session handoff: `.local/handoff/`.)

## What this repo is

MCP server (`isaac_mcp/server.py`) + Isaac Sim Kit extensions (`exts/`) exposing robot digital twins
over the Model Context Protocol. The MCP server connects over a socket to an extension running inside
a live Isaac Sim; each extension is the source of truth for its own tools via `MCP_TOOL_REGISTRY`.

## Extensions

| Extension    | Branch     | MCP port | Robot                    | Module        |
|--------------|------------|----------|--------------------------|---------------|
| `ur5e-dt`    | `main`     | 8766     | UR5e + RG2 gripper       | `ur5e_dt`     |
| `soarm101-dt`| `so-arm101`| 8767     | SO-ARM101 (5-DOF + grip) | `so_arm101_dt`|
| `aic-dt`     | `aic`      | 8768     | UR5e + RG2 + cable (AIC) | `aic_dt`      |

`ur5e-dt` (this `main` branch) is the twin the `mode2_*_fmb*_sim` ablations run against.

## Launching Isaac Sim — verified working 2026-05-27 (ur5e-dt)

**0. Cache health FIRST.** `~/.cache/ov/DerivedDataCache` must be **> 50 MB** or `quick_start` wedges
at `load_robot/reset_async` (cold-cooking the robot USD is broken on 5.x). Check / restore:
```bash
~/env_isaaclab/bin/python ~/Documents/isaac-sim-mcp/scripts/prime_usd_cache.py status
# if < 50 MB:  prime_usd_cache.py restore known-good
```

**1. Launch (blocks until the MCP socket is ready, ~5–10 s warm / ~90 s cold):**
```bash
bash -c 'source ~/env_isaaclab/bin/activate; export ROS_DOMAIN_ID=7 DISPLAY=:0; \
  bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch ur5e-dt'
```
Lifecycle helper also supports `status` / `close` / `kill` / `restart <ext-id>` / `wait [timeout] [port]`.
Raw equivalent: `~/env_isaaclab/bin/isaacsim --ext-folder ~/Documents/isaac-sim-mcp/exts --enable ur5e-dt`

**2. Sanity-check the socket** (returns the tool registry):
```bash
python3 ~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py 8766 list_available_tools
```

### Viewing the sim — **WEBRTC WORKS on dual-a4500. It was NEVER broken** (read this before "fixing" it)

`VIEW=webrtc bash scripts/launch_sim.sh launch ur5e-dt` → streams the active viewport to the Mac's
Isaac WebRTC client at **`<tailscale-ip>:49100`** (verified end-to-end 2026-06-19: srcObject:true,
readyState:4, 1920-wide, currentTime advancing). The **isaac-sim-watchdog** Mac session owns the client
(`/send-to-agent` → `isaac-sim-watchdog`); ask it to `switch set dual-a4500` and read back `<video>` state.
WebRTC streams **whatever the active viewport camera shows** — change `vp.camera_path` and it appears
live, no reconnect. Multiple sessions have burned hours wrongly concluding "webrtc is broken on 5.1." The
actual gotchas (all fixed/known now):
- **Stale-launcher "already running" (FIXED).** `isaacsim_launch.sh:is_running()` used `pgrep -f
  "bin/isaacsim"`, which false-matched launcher bash wrappers / the agent's own grep → spurious "Isaac
  Sim already running" → the launch silently never started. Now pidfile-based (`kill -0`). If you still
  see "already running" with no real sim, kill stray `bin/isaacsim` PIDs + `rm /tmp/isaacsim.pid`.
- **Wait for FULL load before `quick_start`.** The socket answers at ~3 s but the app isn't ready until
  `grep "Isaac Sim Full App is loaded" /tmp/isaacsim.log` (~17 s). Calling `quick_start` early crashes
  the process (→ next socket connect = refused, looks like "webrtc crashed").
- **Camera-ordering crash (webrtc only).** A persisted/invalid session camera + the StageAxis handler →
  `adjust_session_camera` → `NoneType.do()` hard-crash on `quick_start`'s stage edits. Fix: set
  `get_active_viewport().camera_path = "/OmniverseKit_Persp"` (built-in persp) BEFORE `quick_start`, build
  the scene (creates `/World/workspace_camera_sim`), THEN point the viewport at the scene cam.
- **Display is `:1`, not `:0`** on dual-a4500 (`VIEW=window` needs `DISPLAY=:1`). Headless offscreen
  capture (Camera/render_product) returns EMPTY without `enable_cameras`; for a still use webrtc or a
  standalone `SimulationApp` render (`scripts/render_husarion_twin.py`), not live-headless capture.

## ROS domain rule (important)

Set `ROS_DOMAIN_ID` **identically** on the sim launch AND on every ROS-side client (the
`mcp-client-example` process and the `ros-mcp-server` it spawns) — they must share a domain to see
each other's topics. We use **`7`** for sim isolation (no real UR5e on domain 0 by default). The
specific number doesn't matter; consistency does.

## DDS rule (RMW + ROS_LOCALHOST_ONLY) — the sibling of the domain rule

Same domain is necessary but **not sufficient**. Every ROS node sharing the domain must also agree on
the **DDS vendor** and **`ROS_LOCALHOST_ONLY`**, or discovery goes one-way. Symptom we hit (2026-05-27):
`move_home(mode='sim')` failed with *"UR robot driver isn't running"* while `control_gripper(mode='sim')`
"worked". Root cause: the UR driver was launched with the shell default `ROS_LOCALHOST_ONLY=0` + FastDDS,
but the consumer (`ros-mcp-server`) **and Isaac** pin `ROS_LOCALHOST_ONLY=1` + CycloneDDS +
`cyclonedds_local.xml`. A plain topic *publish* (gripper → Isaac, same profile) survives a mismatched
peer; the consumer→driver direction (`FollowJointTrajectory` **action discovery** + the `/joint_states`
subscription `move_home` needs) never matches, so `wait_for_server()` times out.

Proven empirically (daemon-free rclpy `ActionClient`): `cyclone+localhost1+profile` on **both** sides →
server found; any mismatch → not found. `ROS_LOCALHOST_ONLY` is the decisive knob (cross-vendor discovery
itself works once it matches). **rmw_cyclonedds gotcha:** `ROS_LOCALHOST_ONLY=1` only auto-injects a
loopback discovery config when you have **not** set `CYCLONEDDS_URI`; if both are set, your URI wins and
must itself permit localhost discovery — ours does (it's the same profile the consumer uses).

**Fix — `config/ros_dds.env`** (sourced by `sim_bringup.sh` in `source_ros()` and the driver's launch
shell): exports `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=1`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and
`CYCLONEDDS_URI` → `config/cyclonedds.xml` (a self-contained mirror of the consumer's profile). This makes
the whole sim ROS stack homogeneous. Keep it in sync with the consumer (`mcp_config*.json` env +
`server_mode2.py` `os.environ.setdefault`). Also: `cmd_up_sim` now runs `ros2 daemon stop` before its
readiness gates — a stale `ros2` CLI daemon from a prior call under a different RMW/domain caches a corrupt
graph and makes `ros2 topic echo /joint_states` see nothing even when the topic is live.

Verified 2026-05-27: with the driver brought up via the patched `sim_bringup.sh`, `move_home(mode='sim')`
returns `{"result": "success"}` (trajectory sent → accepted → completed) under the consumer's exact env.

## Loading a scene (MCP tools on ur5e-dt)

`quick_start()` → `add_objects(assembly='fmb1'|'fmb2'|'fmb3')` → `setup_pose_publisher()` →
`randomize_object_poses()`. Also: `new_stage()`, `assemble_objects(assembly=…)`,
`save_scene_state(json_file_path=…)`/`restore_scene_state(…)`, recording tools. The `mode2_*` ablation
`onStart` hooks call these, so a study self-loads its scene once the sim+extension are up.

## Two ways to bring up the UR driver half of the sim stack

The robot-motion tools need the **UR ros2_control driver** running (it publishes `/joint_states`,
which Isaac's ur5e-dt subscribes to and mirrors onto `/World/UR5e`, and it accepts
`FollowJointTrajectory` on `scaled_joint_trajectory_controller`). Two interchangeable paths — both
end at the same controllers/topics:

- **Fake (mock) hardware — `scripts/launch_ur_fake.sh` (lightweight, no Docker, no `~/ros2_ws`).**
  Uses the apt `ur_robot_driver` with `use_fake_hardware:=true`; the controllers run without URSim
  or a real robot. Best default for a quick twin. `up` / `status` / `down`; sources `config/ros_dds.env`
  so it shares Isaac's DDS island (Cyclone + localhost-only + domain 7). Verified 2026-06-17 on
  `en4226769-l`: commanded a trajectory → `/joint_states` followed → the Isaac `/World/UR5e`
  articulation mirrored it (`[0.8,-1.19,1.00,-0.50,0,0]`). Fresh-install recipe (apt + a4500
  source-build parity, pinned commits):  `~/Desktop/fresh-install-docs/ur_robot_driver-fake-hardware.md`.
  Modeled on a4500 `~/Documents/prismatic-manipulation/scripts/launch_ur.sh`.
- **URSim docker — `scripts/sim_bringup.sh` (heavier; needs Docker + a `~/ros2_ws` `ur_bringup` install).**
  Use when you need URSim-specific behavior (dashboard/program state/PolyScope parity). On a box with
  no `~/ros2_ws` (e.g. `en4226769-l`) the URSim *container* still comes up but the `ur_bringup` launch
  can't — use the fake-hardware path there.

### URSim docker path — verified 2026-05-27

Hands-free, no-browser bring-up of the **UR robot driver** half of the sim stack (the `docker run
ursim …` + `ros2 launch ur_bringup ur5e.launch.py …` steps that previously required clicking through
PolyScope at `:6080`). One command:

```bash
ROS_DOMAIN_ID=7 bash ~/Documents/isaac-sim-mcp/scripts/sim_bringup.sh up   sim  # bring sim stack up
ROS_DOMAIN_ID=7 bash ~/Documents/isaac-sim-mcp/scripts/sim_bringup.sh status sim # check state
ROS_DOMAIN_ID=7 bash ~/Documents/isaac-sim-mcp/scripts/sim_bringup.sh down sim   # tear down + reap
```

The 2nd arg is **mode**: `sim` (default, URSim docker — fully wired) or `real` (lab UR5e —
**intentionally disabled**: `up real` prints the lab sequence [`ur_bringup` @ 192.168.1.111 +
`onrobot_ros gripper_control` + `aruco_camera_localizer localize`] and refuses unless `FORCE_REAL=1`,
so we never ship an unverified robot-motion path; enable + verify in-lab).

Also exposed as an **MCP tool on ur5e-dt**: `start_ros_driver(mode='sim', action='up'|'down'|'status')`
— shells out to this script (blocks ≤240s until ready, under the 300s socket timeout; runs in the MCP
worker thread, touches no USD). So `run.start` / any MCP client can bring the driver up in one call.
**Verified live 2026-05-27** (client → socket 8766 → `_cmd_start_ros_driver` → driver ready, ~5s warm).

⚠️ **Gotcha (baked into the handler):** the subprocess runs with a **sanitized env** (only HOME / USER /
PATH / ROS_DOMAIN_ID). Isaac Sim's process carries its own ROS2 + `env_isaaclab` setup
(AMENT_PREFIX_PATH / PYTHONPATH / LD_LIBRARY_PATH / RMW_*); inheriting it makes the script's
`ros2 topic echo`/`ros2 launch` misbehave (false "no /joint_states"), which previously made the script
reap the healthy driver. Clean env → the script's own `source /opt/ros/humble` + ros2_ws is authoritative.
`launch_driver` also now **fast-fails** instead of silently reaping a running-but-unhealthy driver.

On success `/joint_states` is live and the driver logs *"Robot connected to reverse interface. Ready
to receive control commands."* Verified across a full cold up→down→up cycle on host **EN4226768**
(ur5e, URSim 5.25.1, ROS Humble); host-agnostic — needs Docker + the UR `~/ros2_ws` install.

**Why a custom script (the key insight).** `ros-mcp-server/utils/ursim_cli.py` drives the dashboard
via the driver's *ROS2 services*, which only exist **after** `ros2 launch` — chicken-and-egg. But the
UR **Dashboard Server (TCP 29999) is part of URSim itself**, reachable the moment the container boots.
So `scripts/ur_dashboard.py` speaks the dashboard wire protocol directly (no ROS), powering the robot
on *before* the driver exists. **Order matters and is the whole trick:**

1. start URSim container (`ros2 run ur_client_library start_ursim.sh -m ur5e -d`) → dashboard `:29999` up
2. stage `resources/ursim/external_control.urp` into the mounted programs dir
3. dashboard: power on + brake release → robot `RUNNING` (re-issued as a state machine; a fresh URSim
   silently drops an early `power on`)
4. launch `ur_bringup` → its script-sender starts listening → `/joint_states` flows
5. dashboard: `load` + `play` external_control.urp → External Control connects back → ready

Playing the program **before** the driver listens makes the URCap fail and stop the program — that's
why step 5 follows step 4. Default URSim has no remote-control gate, so dashboard commands work even
with `is in remote control: false` (the `LOCAL_MODE_BLOCKED` issue in `ursim_cli.py` is real-robot-only).

**Gotchas baked into the script:** the UR control node can hang/crash in its
`ScaledJointTrajectoryController` destructor on SIGINT, so `down` polls then escalates SIGINT→TERM→KILL
(headless node, no X11 — KILL is safe) and confirms death. If `up` finds a driver running but
`/joint_states` not live it **fast-fails** (tells you to `down` first) rather than silently reaping a
possibly-healthy driver. Process matching keys on the binary path `/ur_ros2_control_node` to avoid
matching unrelated shells. Config via env: `MODEL ROBOT_IP DASH_HOST ROS_DOMAIN_ID CONTAINER`
(defaults ur5e / 192.168.56.101 / 127.0.0.1 / 7 / ursim).

## Sim prerequisites for robot-motion tools (the driver was the only real gap)

For the **robot-motion** tools (consumed via `ros-mcp-server` `move_home`/`move_to_grasp`/
`control_gripper`/`translate_object`/`rotate_object`, all `mode='sim'`) to work, the **ROS2 robot
driver + the object-pose publisher must be running** and visible on the active `ROS_DOMAIN_ID`.

**Both are now accounted for in the automated pipeline:**
- **UR driver** — bring up with `scripts/sim_bringup.sh` / the `start_ros_driver` MCP tool (above).
- **Object-pose publisher** — the mode2 ablation YAMLs call `isaac-sim__setup_pose_publisher()` in
  **every phase's `onStart`**, right after `add_objects(...)` (objects must exist first) and before the
  `@wait:1` → `assemble_objects`/`randomize_object_poses`. It is *not* folded into `quick_start` on
  purpose — `quick_start` runs before `add_objects`, so the pose graph would be built on an empty
  `/World/Objects`. So the publisher is automated for ablations; it's only a manual step when you
  hand-drive the sim outside a run (call `setup_pose_publisher()` yourself after `add_objects`).

**Observed 2026-05-27** (mode2_anthropic_fmb1_sim via the mcp-client CLI): the assembly phase failed
with *"UR robot driver not running"* and *"cannot access object poses from `/objects_poses_sim`"*. The
pose error was a **downstream symptom of the driver being down** (the same `onStart` runs
`move_home(mode='sim')` at step 4, which needs the driver) — not a missing `setup_pose_publisher` call.
If `/objects_poses_sim` is ever dark *with* the driver up, it's a runtime issue: the **timeline must be
playing** (OmniGraph only ticks on play) and Isaac must be on the **same `ROS_DOMAIN_ID`** as the
consumer.

**Current state:** with Isaac Sim + ur5e-dt up and the driver up via `start_ros_driver`/`sim_bringup.sh`,
the full mode2 pipeline (incl. the pose publisher, via `onStart`) runs hands-free. The driver was the
single remaining blocker for unattended sim data collection, and it is now closed.

### Status — sim bring-up automation

1. ~~**UR-driver bring-up**~~ ✅ done — `scripts/sim_bringup.sh` + `ur_dashboard.py` (see bring-up
   section). `/joint_states` + command interface, hands-free.
2. ~~**Object-pose publisher automation**~~ ✅ already automated — the mode2 YAMLs call
   `setup_pose_publisher()` in every phase's `onStart` after `add_objects`. Not a code gap; deliberately
   not in `quick_start` (ordering). Only manual when hand-driving the sim outside a run.
3. ~~**Drive the driver from MCP**~~ ✅ done + live-verified — `start_ros_driver` tool on `ur5e-dt`
   (registry + `_cmd_start_ros_driver`; `up`→`status` round-trip over socket 8766).

**Net:** unattended sim data collection is unblocked. Bring the driver up once (terminal:
`sim_bringup.sh up sim`, or MCP: `start_ros_driver(mode='sim')`) before `run.start`; the run itself
handles everything else (incl. the pose publisher) via the mode2 `onStart` hooks.

## Downstream consumer — the mode2 ablation pipeline

`~/Documents/mcp-client-example` (separate repo, see its CLAUDE.md) drives ablations against this sim
through two MCP servers: **`isaac-sim`** (this repo's `isaac_mcp/server.py` → ur5e-dt socket 8766) and
**`ros-mcp-server`** (robot motion in sim mode). Its ablation run engine (`src/ablation-runner.ts`) is
driveable headlessly via the agent action **`run.start`** (`MCP_CLIENT_AGENT_API=1`). Once the ROS2
driver gap above is closed, `run.start` + this launch workflow = unattended end-to-end data collection.

## Conventions

- No `Co-Authored-By` in commits (global rule).
- Only used on **a4500**.
- See the **`isaac-sim-extension-dev` skill** for launch lifecycle, cache discipline
  (`prime_usd_cache.py`), the post-load launcher, and hard-won gotchas (cooking deadlock, post-play
  wedge, real Kit log location). See `README.md` for extension architecture + MCP client config.

## 2026-05-28 — sim/real parity + physics fixes (verified end-to-end)

Full FMB1 sim assembly now works: ground-truth replay seats 4/4, and a live 3-phase ablation
(claude-haiku-4-5) completed all 3 phases on attempt 1 (4/4 assembled), no escalation. Fixes
in THIS repo (consumer-side fixes live in ros-mcp-server):

- **Launch via `scripts/launch_sim.sh`** (NOT the bare skill launcher). It sources
  `config/ros_dds.env` (CycloneDDS + ROS_LOCALHOST_ONLY=1 + domain 7) so Isaac shares the DDS
  island with ros-mcp-server + the UR driver. A bare launch defaults to localhost_only=0 → the
  consumer can't discover /gripper_width_sim, /objects_poses_sim, /joint_states. (commit 0445657)
- **`config/ros_dds.env` + `config/cyclonedds.xml`**: canonical sim DDS env (mirrors the
  consumer profile); also sourced by `scripts/sim_bringup.sh` for the driver. (a2b776b)
- **`add_objects` rebuilds PhysX tensor views** (async `_cmd_add_objects` → stop+play after
  adding objects). Adding rigid bodies to a PLAYING sim invalidates the simulationView → parts
  slip out of the gripper mid-motion. Stop+play re-registers them. (0445657)
- **`ROBOT_BASE_Z = 0.0`** — Isaac convention: ground plane + robot base both at z=0. Do NOT
  raise it; the consumer (ros-mcp-server) is mode-aware to match this in sim. (c545bde)

## 2026-06-18 — gravity is DISABLED on the robot chain (position-mirrored twin) (commit ea282c2)

**First-class DT fact, bigger than it sounds.** `quick_start` now authors
`physxRigidBody:disableGravity=True` on all 16 robot-chain bodies (UR5e arm + Quick-Changer + RG2),
while the manipulated objects (`/World/Objects/*`) keep gravity. **Do NOT re-enable arm/gripper gravity.**

Why: the ur5e-dt arm is a **position-mirroring twin** — its joints come FROM `/joint_states` (the
real/fake UR driver), which already encode the real UR5e's *gravity-compensated* pose. Simulating gravity
on the twin's own links **double-counts** it: the PD joint drives must sag to hold a torque the real robot
never fights. The sag is **pose-dependent** — ~0 at home/balanced poses but **~4 mrad on
shoulder_lift/elbow → ~3 mm tool droop at extended reach** (grasp/place/insert poses). It was silently
costing mm-scale accuracy on *every* extended-reach motion (retroactively explains prior real-vs-sim drift
at reach); it first surfaced as the grasp table-clearance residual eating the ≥5 mm fingertip margin.

Isolated by joint-space attribution (`r_ik=0`, `r_model=0`, payload negligible → pure `r_pipeline` sag);
verified 4.06 → **0.00 mrad** at the grasp config via hot-reload + fresh `quick_start`. Feed-forward
gravity-comp via `set_joint_efforts` does NOT work here — the ROS2 OmniGraph `ArticulationController` zeros
applied efforts every physics step (feed-forward is the right fix only when the SIM drives the joints; here
the path is driver → sim, so the upstream already handled gravity). See `_disable_robot_chain_gravity` in
`exts/ur5e-dt/ur5e_dt/extension.py` for the full rationale, and the **`robot-joint-residual-attribution`**
skill for the reusable diagnosis method (`r_total = r_ik + r_ref + r_pipeline + r_model`, joint-space as
ground truth — works sim AND real).

## 2026-06-19 — RG2 grip is FORCE-LIMITED AUTO-STOP; `maxForce` is the grip-force cap (commit b34bea4)

**First-class DT fact (peer to gravity-disable).** The RG2 is a **force-limited auto-stop** gripper, by
design and for **sim/real parity**. The grasp command is just **`close`** (the actuated `rg2_gripper_joint`
targets its full-close hard stop); the drive closes until it contacts the part and **stalls at
`RG2_ACT_MAXFORCE`**. That `maxForce` IS the grip-force cap — exactly how the real RG2 firmware stops on
contact. **Do NOT "fix" a too-hard grip with a hard width target or a close-to-contact-then-hold policy —
that breaks parity. The only knob is the cap.**

`RG2_ACT_MAXFORCE = 1.0` (== the old force-drive `RG2.usd` value). The bug was purely the cap being set too
high: at 1000 the drive crushed parts **through** their collision mesh (u_brown settled 12.6 mm vs a true
24.5 mm wall); even 5 left a 16 mm hex at 12.4 mm. At 1.0 the hold is gentle (u_green wrap −0.25 N·m; the
worst-case hex friction grasp stalls at the ~1.0 cap, gap ~14.3 mm near the true surface) and **both wrap
and friction grasps still lift clean + level**. Free-air close still reaches the stop even at `maxForce=0.5`,
so the stiffened mimic does **not** block low-force closing — keep the strong ACCELERATION drive (it needs
the position-term authority to drag the parallelogram) AND the low cap (gentle contact); the two are
separable. Also `RG2_MAX_JOINT_VEL = 114.59` deg/s clamps the importer's ~1e6 (unclamped → jaws slam).

**Caveat:** `/rg2_sim/contact_width` is computed from the **actuated** joint angle, which over-rotates past
the part-blocked fingers under load → it publishes `actuated_width`, not the true finger gap (reads ~3.6 mm
on a hex while the fingers are physically at ~12.4 mm). For grasp-success use the **physical finger gap**,
not that topic (open item on the consumer `rg2_sim_backend` side). Calibrate the cap by **reading the joint
parity** (actuated vs mimic angle vs physical gap vs measured effort, free-air vs loaded) — method in the
**`isaac-urdf-import`** skill (`verify.md` §5), operational notes in **`drive-ur5e-grasp-sim`**.

## 2026-06-21 — BOTH cooking wedges are one bug: UJITSO multiprocess-fork deadlock (fix: disable UJITSO session-wide)

**Root cause, finally pinned — it overturns the old "SDF cost cliff" AND "cold-cache cooking broken on
5.x" lore, which are the SAME bug.** Cooking wedges (socket dies, Isaac alive at low CPU,
`futex_wait`/`carb.tasking` → `omni.physx.cooking` per gdb) are **UJITSO** — Isaac's *default* collision
cooker, which forks **subprocesses** to cook meshes (`/physics/cooking/ujitsoCollisionCooking`, on by
default; `ujitsoCookingMaxProcessCount`). Forking from the live, heavily-multithreaded Kit app (MCP
socket-server thread + render + physics) deadlocks. **Fix: set
`/physics/cooking/ujitsoCollisionCooking=False`** → cooking runs in-process → no fork → no wedge.

Done in **`on_startup`** (`exts/ur5e-dt/ur5e_dt/extension.py`) so it's in force session-wide before any
cook (the setting is non-persistent; `_cmd_add_objects` also sets it as a belt-and-suspenders guard).
**One disable fixes both long-standing wedges (both verified live 2026-06-21):**
- **Object cook** — `add_objects('fmb4')` (6× jenga **SDF-256**, robot present): wedged indefinitely →
  now **~2 s**, blocks settle, socket live.
- **Robot COLD-CACHE cook** — `quick_start`/`load_robot` with an empty `DerivedDataCache` (the
  "broken on 5.x", 240 s+ wedge): → now **~3 s**, robot built + articulated + playing. In-process
  cooking is fast enough that the **`DerivedDataCache` snapshot/restore dance is now just a speed cache,
  not a correctness crutch** — cold cooking simply works with UJITSO off. (Disciplined test: snapshot →
  move cache aside → cold launch → cook → restore. Cache untouched after.)

What this corrects (the A/B is decisive — each prior hypothesis was tested and failed):
- **NOT an SDF-resolution cost cliff.** The identical 6× SDF-256 set cooks in **1.3 s** in a robot-free
  standalone `SimulationApp` (clean process → the UJITSO fork is fine) and in **2 s** over MCP once UJITSO
  is off. The old "lower `SdfResolution` (300→256)" remedy only ever *shrank the fork-deadlock window* (a
  lighter/shorter cook is less likely to deadlock) — an incidental workaround, not the real lever.
- **NOT the robot articulation.** A *single* block, no robot, minimal scene, wedges too — robot-independent.
- **NOT rendering / play-state.** Headless (no webrtc) and a stop→add→play reorder both still wedged.
- **Collider is UNCHANGED.** UJITSO is only the cooking *backend* (how the shape is computed); the cooked
  SDF-256 collision shape and runtime collision are identical. Verified live: blocks rest on the table
  (`z_min≈0`) and a physics raycast hits every block's cooked SDF collider. **Do NOT downgrade box-like
  parts to `boundingCube` to "fix" this — keep the authored SDF.**
- Object-mesh SDF cooks still do **not** populate `DerivedDataCache`, and cooked data does **not** persist
  into the USD on a normal cook (no `PhysxCookedDataAPI` authored) — so "prebake into the USD / cache" is
  not a path; disabling UJITSO is.

Diagnostics in `.local/scripts/`: `load_fmb4_standalone.py` (robot-free cook benchmark, `--collider`/
`--sdf-res`), `min_add_jenga_noujitso_snip.py` (minimal socket add with UJITSO off). **Still untested:**
the `aic-dt` `--enable <ext>` boot-time cooking deadlock (a different trigger path — cooking during Kit's
startup boot, not a runtime MCP call) likely shares the UJITSO cause; the postload-launcher workaround can
probably be replaced by the same disable, but verify on `aic-dt` before removing it.

## 2026-06-24 — USD LAYER CACHE: reloading new CAD needs force-reload + re-cook (and how to VERIFY it)

**First-class operational gotcha (peer to UJITSO / gravity-disable).** Changing a part USD on disk and
calling `delete_objects`→`add_objects` does **NOT** reliably load the new CAD. USD keeps an in-memory
**layer registry**: `Sdf.Layer.FindOrOpen()` returns the *cached* layer instance (shared across stages), so
the `AddReference` in `add_objects` silently re-composes **stale** content. `delete_objects` removes the
prim/reference arc but **not** the cached layer. This is exactly the Kit **"source changed / pull latest CAD
changes" popup** — if a human clears it, the timing of whether a programmatic `add_objects` saw old or new
content is ambiguous. **Consequence:** any A/B that swapped USDs via `delete/add` *without* verifying the
live load is **suspect** — several box-vs-SDF / "which CAD is wrong" verdicts this session were taken before
this was understood and were not cleanly tested.

**The reliable reload recipe (proven live 2026-06-24):**
1. New CAD on disk → `Sdf.Layer.Reload(force=True)` on every cached layer under the assembly folder
   (defeat the cache). `Reload(force=False)` only reloads if the mtime changed; `force=True` always re-reads.
2. **THEN `delete_objects` + `add_objects`** (re-import + the stop/play in `_cmd_add_objects` **re-cooks** the
   collider into PhysX). For an SDF-resolution change a fresh **relaunch** is safest.
- **TRAP:** `Reload(force=True)` **alone** updates the USD prim live but leaves PhysX with **NO cooked
  collider** — the recompose drops the old collision shape and nothing re-cooks the new one until step 2.
  A USD-attribute read (e.g. `sdfResolution`) would say "works" while physics is actually broken. Always
  follow force-reload with a re-cook.

**VERIFY at the PHYSICS level, not just USD.** USD reads reflect the reloaded layer; they do **not** prove
PhysX re-cooked. Use an `omni.physx` overlap scene-query — it returns the *cooked* collider prim(s):
**SDF = 1 mesh collider** (`.../u_brown`), **box-compound = 3 `box_00X`** (`.../collisions/box_00X`). Probe:
`.local/scripts/fmb-reload-verify/physx_collider_probe.py` (locate the `UsdPhysics.RigidBodyAPI` prim, get
its world pose, `overlap_sphere` there, filter hits under the object). Verified end-to-end: SDF→1 mesh;
force-reload box (no re-cook)→`[]` (broken); +re-cook→3 cooked box colliders.

**Extension fix (the real fix at the source) — `add_objects()` now force-reloads cached layers** under the
assembly folder before re-referencing (`Sdf.Layer.GetLoadedLayers()` → `Reload(force=True)` for layers whose
`realPath` is under `folder_path`), so a stock `delete_objects`+`add_objects` picks up the latest on-disk CAD
+ re-cooks. ✅ **VERIFIED on launch-imported code 2026-06-24** (commit `0a95aa5`): live fmb1, u_brown box-
compound = 3 cooked boxes → drop `box_002` on disk → stock `delete_objects`+`add_objects` → omni.physx
overlap probe shows exactly **2** cooked boxes (then restored → 3). Force-reload defeats the layer cache AND
the stop/play re-cooks the new collider; the running process carried the patch from its launch import, not a
hot-reload (`add_objects` introspected over the socket confirmed the force-reload + overlay blocks present).

Scripts preserved (out of the ephemeral scratchpad): `.local/scripts/fmb-reload-verify/`
(`run_fmb1_verbose2.sh` + `regrasp_verbose.sh`/`direct_verbose.sh` = the hardened per-part driver with
grasp_points-publisher restart + topic gates; `reload_and_verify_fmb1.py`; `physx_collider_probe.py`).

## 2026-06-25 — RELEASE PHASE: use `pre_grasp` (~35mm), NOT `pre_release` (~62mm) — wide open disturbs seated parts

**Consumer-side assembly-reliability fact (orthogonal to any ros-mcp-server merge).** The final per-part
**release** step (`control_gripper --phase …` after `insert`) was using **`pre_release`**, whose DYNAMIC
*clearance* width opens the gripper WIDE (~62mm for u_brown) to clear the assembly for retract. That wide/fast
open **knocks the just-seated part — or a neighbour — loose**, so `verify_assembly` then reports it not-seated.
Symptom: `run_fmb1.sh` flaky **0–1/4** at 4/4, a DIFFERENT part dropping each run (line_green / u_orange /
inverted_u_yellow), every primitive returning `success` (it's a post-release physics disturbance, not a grasp/
insert failure). This masquerades as a regression but is pure release-width noise.

**Fix (applied):** the per-part drivers' final release now uses **`--phase pre_grasp`** (~35mm grasp-approach
width — opens just enough to release the ~21mm-gripped part without disturbing the assembly). `.local/scripts/
regrasp_part.sh` (~L26) + `direct_part.sh` (~L21); `run_fmb1/2/3.sh` share these drivers so all benefit. The
hardened `*_verbose.sh` runners already used `pre_grasp` and got **3/3 4/4** on box-fmb1 AND fmb3. **Do NOT
revert to `pre_release` for "clearance fidelity" — 35mm clears fine here and the wide open only adds noise.**
(If a future tight assembly genuinely needs a wider retract, gate the wide open on actual clearance, don't
blanket-widen.) Net: removes a confounding noise source — makes both A/B arms cleaner, not less faithful.
