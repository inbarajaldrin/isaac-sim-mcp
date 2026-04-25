<!-- GSD:project-start source:PROJECT.md -->
## Debug-First Entrypoint (read before touching anything)

Before debugging ANY pick-place behavior (cup collisions, IK failures, OMPL
post-check rejections, tracking lag, gripper attach issues, stale
`/drop_poses`, control-GUI crashes), **read `docs/DEBUG-GUIDE.md`**. It is a
living document with:

- How to confirm the stack + telemetry rig are healthy
- How to read the 3-layer lag decomposition the rig produces
- Known failure modes with detection/root-cause/fix notes
- An index of fixes already explored, with verdicts
- A contribution protocol — *extend that doc as you diagnose new things*

The motion telemetry rig (`scripts/motion_log*.sh`, `scripts/motion_logger.py`,
`scripts/motion_analyze.py`, `scripts/motion_verify.py`) captures every robot
motion at 50 Hz with per-joint plan/JTC/Isaac state, cup positions, and physics
rates. Runs under a supervisor that respawns it if killed. Start with
`scripts/motion_log.sh status`; analyze with `scripts/motion_log.sh latest` or
`scan`; run MoveIt collision replay on captured data with
`scripts/motion_log.sh verify` (replays plan + Isaac trajectories through
`/check_state_validity` to catch off-plan collisions the plan-time checker
missed — see DEBUG-GUIDE § 4.1).

## Project

**SO-ARM101 Pick-and-Place Drop Support**

Adding drop (place) capability to the SO-ARM101 pick-and-place pipeline across three repos: the Isaac Sim digital twin extension (drop pose publishing), the aruco_camera_localizer (ArUco-based drop pose detection), and the SO-ARM101 ROS2 control package (drop motion execution via 5-DOF wrist sweep). This completes the pick-and-place loop that currently only supports pick.

**Core Value:** The robot can sort lego blocks by color into matching cups — pick from table, drop into the correct cup — driven by poses from Isaac Sim (direct) and later from ArUco marker detection (camera-based).

### Constraints

- **No commits**: Until user confirms
- **Message format**: `/drop_poses` uses `tf2_msgs/TFMessage` (same as `/objects_poses`)
- **Drop pose convention**: Each transform's `child_frame_id` = `drop_{N}` (matching JETANK convention), translation = cup BODY-CENTER (z = cup_base + half_height). Consumers (e.g. `_cmd_drop_sweep`) compute rim_z = z + CUP_BODY_HEIGHT_M/2 internally before adding hover.
- **Subagent models**: Sonnet for planning/research, Opus for execution
- **Individual steps**: Drop motion broken into discrete button-press steps with matching debug services
<!-- GSD:project-end -->

<!-- GSD:stack-start source:STACK.md -->
## Technology Stack

Technology stack not yet documented. Will populate after codebase mapping or first phase.
<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->
## Conventions

Conventions not yet established. Will populate as patterns emerge during development.
<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->
## Architecture

Architecture not yet mapped. Follow existing patterns found in the codebase.
<!-- GSD:architecture-end -->

<!-- GSD:workflow-start source:GSD defaults -->
## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:
- `/gsd:quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd:debug` for investigation and bug fixing
- `/gsd:execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->

## Repos & Layout

- **This repo** (`~/Documents/isaac-sim-mcp`, branch `so-arm101`): Isaac Sim extension `soarm101-dt` at `exts/soarm101-dt/`. MCP socket on port **8767**.
- **ROS2 control stack**: `~/Projects/Exploring-VLAs/vla_SO-ARM101` — control GUI, MoveIt, geometric IK, drop motion scripts. Launch: `ros2 launch so_arm101_control control.launch.py rviz:=true`
- **aruco_camera_localizer**: publishes `/drop_poses` from real camera. (Path TBD — used in Phase 3 and onward.)

Branch model: `main` = UR5e extension (`ur5e-dt`), `so-arm101` = this project (`soarm101-dt`), `aic` = AIC extension. Do not cross branches without understanding the merge implications — the extensions coexist in `exts/` on each branch but only one is `--enable`'d per launch.

## Bring-up Sequence

Use the `isaac-sim-extension-dev` skill (provides `isaacsim_launch.sh` + port mappings).

### Preflight — always run first

Check what's already alive before launching anything:

```bash
ss -tlnp 2>/dev/null | grep :8767   # Isaac Sim MCP socket
ros2 topic list 2>&1 | grep -E '/joint_states|/drop_poses'   # ROS2 publishing
ros2 node list 2>&1 | grep so_arm101_control_gui   # control stack GUI node
```

Interpret:
- All three alive → stack is already up. **Skip steps 1–3 below.** Scene may already be `quick_start`'d; verify with the `list_viewport_publishers` MCP tool or check `/clock` is ticking (`ros2 topic echo /clock --once`). Run `quick_start` only if the scene actually needs rebuilding.
- Isaac Sim up but no ROS2 topics → socket is alive, scene may not be built. Call `quick_start` (step 2) and launch control stack (step 3).
- Nothing alive → full launch sequence below.

`isaacsim_launch.sh launch` is idempotent — if Isaac Sim is already running with `soarm101-dt` enabled and the socket is ready, it returns fast without respawning. Safe to call either way.

### Full launch (nothing running)

1. `bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch soarm101-dt` — blocks until socket ready (~10s warm, ~90s cold, max 120s). **NEVER** set a Bash timeout on this script; it has its own.
2. Call `quick_start` via socket 8767 to spawn scene + robot + action graphs + publishers.
3. Source ROS2 + launch control stack: `source /opt/ros/humble/setup.bash && source ~/Projects/Exploring-VLAs/vla_SO-ARM101/install/setup.bash && ros2 launch so_arm101_control control.launch.py rviz:=true`
4. Verify: `ros2 topic list` should show `/joint_states`, `/drop_poses`, `/objects_poses_sim`, `/wrist_camera_rgb_sim`.

Shutdown: `bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh close` (graceful SIGTERM). For the ROS2 stack: `pkill -SIGINT -f "ros2.*launch.*control.launch"` — SIGINT propagates to children, SIGTERM does not.

## MCP Tools (soarm101-dt extension.py)

Registered in `MCP_TOOL_REGISTRY` at the top of `exts/soarm101-dt/so_arm101_dt/extension.py`. Socket protocol: `{"type": "<tool_name>", "params": {...}}` on `localhost:8767`.

- **Scene lifecycle**: `quick_start`, `new_stage`, `load_scene`, `load_robot`, `setup_action_graph`, `play_scene`, `stop_scene`
- **Publishers**: `setup_pose_publisher`, `setup_force_publisher`, `setup_bbox_publisher`, `setup_wrist_camera_action_graph`, `publish_drop_poses`
- **Viewport publisher** (Phase 13, active-reuse only, zero RTF cost): `start_viewport_publisher`, `stop_viewport_publisher`, `list_viewport_publishers`
- **Video recording** (Phase 13, ffmpeg-backed): `start_recording`, `stop_recording`, `get_recording_status`
- **Objects / cups**: `add_objects`, `delete_objects`, `randomize_object_poses`, `randomize_single_object`, `sort_objects`, `sort_into_cups`, `add_cups`, `delete_cups`, `update_cups` (re-snaps cups to default `CUP_LAYOUT` poses; used by `vla_SO-ARM101/scripts/sim_reset.sh`), `randomize_cups` (random `fwd, lat, yaw` within drop-reachable bounds — defaults match grasp_workspace; for stress-testing the pick-place pipeline against varied cup poses)
- **State**: `save_scene_state`, `restore_scene_state`, `sync_real_poses`
- **Escape hatch**: `execute_python_code` (runs arbitrary code in Kit's Python context; set a `result` variable to return data)

## Gotchas

- **CUDA 999 on Isaac Sim startup** (`Failed to create any GPU devices`): reload UVM — `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`. Safe; doesn't touch display modules. Verify with `~/env_isaaclab/bin/python3 -c "import ctypes; print(ctypes.CDLL('libcuda.so.1').cuInit(0))"` — expect `0`.
- **rclpy in Isaac Sim**: requires the Python 3.11 build at `/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages`. soarm101-dt's extension.py prepends this to `sys.path` at module load time — do not remove or rclpy node creation fails with `AttributeError: module 'rclpy' has no attribute 'impl'` followed by `rosidl_typesupport_c` not found.
- **Hot-reload scope**: `touch exts/soarm101-dt/so_arm101_dt/extension.py` re-executes module-level code (classes, dicts, constants, imports). Full restart needed only for `extension.toml` changes or adding new `.py` files.
- **Dedicated hidden viewports cost ~0.32 RTF each** (measured Phase 13). `_ViewportCameraPublisher.start()` supports active-viewport reuse only for this reason. For simultaneous multi-camera publishing, keep the action-graph path.
- **Wrist camera uses an action graph** (`/Graph/ActionGraph_WristCamera`), not a viewport publisher. Don't "migrate" it — the action graph is already near-zero RTF cost and produces `/wrist_camera_rgb_sim` + `/wrist_camera_info` together.
- **X11 GUI safety**: never `kill -9` Isaac Sim, RViz, or any process owning an X window (causes KWin BadWindow cascades). Use the skill's `close` subcommand or SIGTERM. Details in `~/.claude/CLAUDE.md` global rules.
- **Ghost ROS2 topics** (publisher count = 0 after deleting an action graph prim): cycle the timeline — `tl.stop(); for _ in range(20): app.update(); tl.play()` — to flush the Kit ROS2 bridge publishers.
- **Cross-extension stdout contamination**: if both `ur5e-dt` and `soarm101-dt` get loaded in the same Isaac Sim session, ur5e-dt's `LogRedirector` can hijack stdout and crash soarm101-dt's `on_startup` with `AttributeError: 'DigitalTwin' object has no attribute '_ext_log_lines'`. Fix by launching with only one `--enable` at a time via `isaacsim_launch.sh` (handles this).
- **SDF collision regeneration OOM**: gripper_link + moving_jaw_so101_v1_link use `PhysxSDFMeshCollisionAPI` at `resolution: 256` with `margin: 10 mm`. PhysX's voxelizer peaks at ~48 GB RAM during SDF generation (observed: full workstation hang). **Do not call `PhysxSDFMeshCollisionAPI.Apply()` or write any SDF attribute on a running stage** — it triggers full regeneration. Use read-only introspection (`GetSdfResolutionAttr().Get()` etc) to query. If SDF rebuild is unavoidable, run Isaac Sim with swap disabled and close other heavy processes first. This is the PhysX contactOffset fix's main risk vector — prefer authoring `physxCollision:contactOffset` (no SDF rebuild) over changing SDF params.
- **Cup collision padding is ROS-side only**: `_CUP_COLLISION_PADDING` (default 5% = 1.05) lives in `vla_SO-ARM101/.../control_gui.py` and only enlarges cups in MoveIt's planning scene. Isaac Sim's USD cup mesh is unchanged — physics contact telemetry (e.g. omni.physx contact-report subscription) stays honest. If you want padding-aware physics evaluation, that's a separate change in `soarm101-dt`'s scene-build path.
- **Deterministic planner lives ROS-side**: vla_SO-ARM101 routes every arm motion through tier-1 linear → tier-2 retract-pan-settle → OMPL fallback (opt-in per primitive; `_cmd_grasp_home` opts OUT). End-to-end test scripts at `vla_SO-ARM101/scripts/{test_qs_cycle,sim_reset,test_pick_all}.sh`. Isaac Sim is the physics backend; the planner has zero awareness of physics state beyond `/joint_states` feedback. Planner-collision verdicts (planning-scene) and physics-contact verdicts (PhysX) intentionally don't agree — padding makes the planner conservative on top of physics ground-truth.
- **`_attached_lego_tcp_offset` populated at attach time**: when `_cmd_gripper_close_for_object` succeeds, the held block's pose-in-tcp_link is captured and used by `_cmd_drop_sweep` to position the block (not tcp_link) over the cup center. Isaac Sim physics decides where the block actually settles after grasp_close — off-center attaches (typical: ~−12 mm in TCP X) are normal and the planner compensates. If physics changes (different lego mass/inertia, new gripper material), the typical offset magnitude can shift; verify with `Attach OK:` log line.

## Robot Facts (SO-ARM101)

- 5-DOF arm: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` + `gripper_joint`.
- Forward axis = `+X`, joint drives: stiffness=15, damping=0.15, maxForce=3.
- **Drop motion is IK-planned, not a pure kinematic sweep** (despite the original design intent). Implemented in `vla_SO-ARM101/.../control_gui.py:_cmd_drop_sweep` — IK targets a wrist_flex ~55° pose using the MEASURED attach offset (`_attached_lego_tcp_offset`) and locks `shoulder_pan` to drop_point's value. drop_point itself is a pure pan rotation (only `shoulder_pan` and `wrist_roll` change). Cups anchored to shoulder pan axis (not world origin).
- ArUco markers on cups: red=ID 0, green=1, blue=2 (DICT_4X4_50).
- Wrist camera: OV9732 (640×480). Workspace camera: `/World/workspace_camera_sim` (created by Phase 13 workflows, pose mirrors ur5e-dt's).

<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd:profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
