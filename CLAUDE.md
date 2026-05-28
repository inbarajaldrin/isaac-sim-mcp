# CLAUDE.md — isaac-sim-mcp runtime & operations

Project memory for agents working in this repo. Captures the **verified** way to launch Isaac Sim
with an extension, the ROS domain rule, and the **open ROS2-driver gap** that currently blocks
fully-automated robot motion. Extend the TODO section as the driver-spawn method is figured out —
the goal is to drive it from isaac-sim-mcp itself.

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

## ROS domain rule (important)

Set `ROS_DOMAIN_ID` **identically** on the sim launch AND on every ROS-side client (the
`mcp-client-example` process and the `ros-mcp-server` it spawns) — they must share a domain to see
each other's topics. We use **`7`** for sim isolation (no real UR5e on domain 0 by default). The
specific number doesn't matter; consistency does.

## Loading a scene (MCP tools on ur5e-dt)

`quick_start()` → `add_objects(assembly='fmb1'|'fmb2'|'fmb3')` → `setup_pose_publisher()` →
`randomize_object_poses()`. Also: `new_stage()`, `assemble_objects(assembly=…)`,
`save_scene_state(json_file_path=…)`/`restore_scene_state(…)`, recording tools. The `mode2_*` ablation
`onStart` hooks call these, so a study self-loads its scene once the sim+extension are up.

## ⚠️ KNOWN GAP — ROS2 driver / pose publisher (TO AUTOMATE)

For the **robot-motion** tools (consumed via `ros-mcp-server` `move_home`/`move_to_grasp`/
`control_gripper`/`translate_object`/`rotate_object`, all `mode='sim'`) to work, the **ROS2 robot
driver + the object-pose publisher must be running** and visible on the active `ROS_DOMAIN_ID`.

**Observed 2026-05-27** (mode2_anthropic_fmb1_sim via the mcp-client CLI): with only the lifecycle
launch above, the sim came up and `isaac-sim__*` tools worked, but the assembly phase failed with the
agent reporting:
- *"UR robot driver not running"*
- *"cannot access object poses from `/objects_poses_sim`"*

→ The task cannot complete. (The ablation engine handles it correctly — escalates/aborts — but **no
successful run data is produced** until the driver + publisher are up.)

**Current state:** bringing up the ROS2 driver + pose publisher for sim mode is a manual, multi-step
process that is **not yet doable purely from the terminal**. This is the single remaining blocker for
unattended sim data collection.

### TODO — automate the ROS2 driver spawn (fill in as the method is found)

1. **Document the exact bring-up** for ur5e-dt in sim mode: commands / launch files / env, which
   `ROS_DOMAIN_ID`, and the topics that must appear (e.g. `/joint_states`, `/objects_poses_sim`,
   `/tf`, the workspace camera topic).
2. **Verify recipe:** `ROS_DOMAIN_ID=7 ros2 topic list` shows the expected topics; `ros2 topic echo
   /objects_poses_sim` produces data; a single `ros-mcp-server__move_home(mode='sim')` succeeds.
3. **Drive it from isaac-sim-mcp itself** (the goal): expose the bring-up as an MCP tool on `ur5e-dt`
   (e.g. `start_ros_driver` / `ensure_pose_publisher`), or fold it into `quick_start` / the extension's
   `on_startup`, so **one MCP call brings sim + driver + publisher fully live**. Then the whole
   pipeline runs headlessly with no out-of-terminal steps.

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
