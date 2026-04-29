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
- **aruco_camera_localizer**: `~/Desktop/ros2_ws/src/aruco_camera_localizer` (branch `robosort` — also has a stale duplicate at `~/Projects/RoboSort/aruco_camera_localizer/`; runtime always uses Desktop via colcon `--symlink-install`). Publishes `/aruco_poses_real`, `/drop_poses_real` (ArUco) and `/objects_poses_real`, `/objects_bbox_real` (YOLOE). Per-marker geometry in `config/aruco_config.json`'s `marker_to_object` blocks (function-driven via `marker_geometry.py`); YOLOE prompts in `config/robot_config.yaml`'s `<robot>.detection.yolo` block. **Read that repo's `CLAUDE.md` before editing.**

  Launchers (run from your own terminal — cv2.imshow needs D-bus / XDG_SESSION context that Bash subshells lack):
  - ArUco: `bash scripts/restart_aruco_localizer.sh`
  - YOLOE: `bash scripts/restart_yoloe.sh` (config-driven prompts; flags `--bg`, `--headless`, `--conf`, `--prompts`, `--camera`)
  - Add new marker/object: `python3 ~/Desktop/ros2_ws/src/aruco_camera_localizer/scripts/derive_marker_config.py --marker-id N --cup-prim PATH --marker-prim PATH` — derives `marker_to_object` config from USD ground truth + live PnP. Marker MUST be in camera FOV during the run (Kalman extrapolation produces nonsense values otherwise).

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
4. **For real-mode work** (Real Test tab, lego/cup detection from camera) — also launch the two detectors **from your own terminal** (NOT a Bash subshell — cv2.imshow needs D-bus / XDG_SESSION):
   - ArUco: `bash scripts/restart_aruco_localizer.sh` → publishes `/aruco_poses_real`, `/drop_poses_real`
   - YOLOE: `bash scripts/restart_yoloe.sh` → publishes `/objects_poses_real`, `/objects_bbox_real` (config-driven prompts from `~/Desktop/ros2_ws/src/aruco_camera_localizer/config/robot_config.yaml`)
   - Both have `--headless` and `--bg` flags if you don't need cv2 preview windows. For sim-only work, skip this step.
5. Verify topics:
   - Sim only: `ros2 topic list` should show `/joint_states`, `/drop_poses`, `/objects_poses_sim`, `/wrist_camera_rgb_sim`.
   - + Real-mode: also `/drop_poses_real`, `/aruco_poses_real`, `/objects_poses_real`, `/objects_bbox_real`.
6. Verify the planning scene (real-mode): `python3 -c "import rclpy; from rclpy.node import Node; from moveit_msgs.srv import GetPlanningScene; from moveit_msgs.msg import PlanningSceneComponents; rclpy.init(); n=Node('check'); c=n.create_client(GetPlanningScene,'/get_planning_scene'); c.wait_for_service(3.0); r=GetPlanningScene.Request(); r.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES; f=c.call_async(r); rclpy.spin_until_future_complete(n,f,5.0); [print(co.id) for co in f.result().scene.world.collision_objects]; rclpy.shutdown()"

Shutdown: `bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh close` (graceful SIGTERM). For the ROS2 stack: `pkill -SIGINT -f "ros2.*launch.*control.launch"` — SIGINT propagates to children, SIGTERM does not.

### Recording stack (lerobot dataset capture on Linux)

Once steps 1-3 above are up, the recording layer adds:

7. **Mirror** (system Humble): `python3 -u ~/Projects/Exploring-VLAs/linux-env/scripts/joint_states_to_commands.py &` — mirrors `/joint_states` → `/joint_commands_lerobot` so lerobot has an action-column source.
8. **Record Sim tab** (preferred path): in control_gui, press `▶ Start` on the Record Sim tab. Spawns lerobot-record + dedups mirror, runs N pick-place episodes, finalizes dataset on Stop. Driveable from CLI:
   ```bash
   ros2 service call /so_arm101_control_gui/rec_start std_srvs/srv/Trigger
   ros2 service call /so_arm101_control_gui/rec_stop  std_srvs/srv/Trigger
   ```
   Configure via `set_widget_value` (Episodes spinbox, Block color combobox, action checkboxes — see `~/Projects/Exploring-VLAs/linux-env/CLAUDE.md`).

9. **Manual lerobot-record** (alternative): `bash ~/Projects/Exploring-VLAs/linux-env/scripts/record_sim_isaac.sh --dataset.repo_id=local/<name> ...` — runs in pixi-Jazzy via the bash wrapper. Dataset lands at `~/.cache/huggingface/lerobot/local/<name>/`.

Datasets viewable in rerun via the canonical command — **don't write a custom rerun loader, the official tool already exists**:
```
pixi run --manifest-path ~/Projects/Exploring-VLAs/linux-env/pixi.toml \
  lerobot-dataset-viz --repo-id local/<name> --root <path> \
  --episode-index 0 --mode local
```
Re-invoke with a different `--episode-index` while the viewer is alive on `:9876` to attach more episodes to the recordings dropdown — no need to kill/restart. Custom rerun scripts using `rr.AssetVideo` will fail on Ubuntu 22.04 because system ffmpeg 4.4.2 is below rerun's 5.1 requirement; `lerobot-dataset-viz` decodes via pyav inside pixi-Jazzy and avoids that. See `~/Projects/Exploring-VLAs/linux-env/CLAUDE.md` Gotchas section.

## MCP Tools (soarm101-dt extension.py)

Registered in `MCP_TOOL_REGISTRY` at the top of `exts/soarm101-dt/so_arm101_dt/extension.py`. Socket protocol: `{"type": "<tool_name>", "params": {...}}` on `localhost:8767`.

- **Scene lifecycle**: `quick_start`, `new_stage`, `load_scene`, `load_robot`, `setup_action_graph`, `play_scene`, `stop_scene`
- **Publishers**: `setup_pose_publisher`, `setup_force_publisher`, `setup_bbox_publisher`, `setup_wrist_camera_action_graph`, `publish_drop_poses`
- **Viewport publisher** (Phase 13, active-reuse only, zero RTF cost): `start_viewport_publisher`, `stop_viewport_publisher`, `list_viewport_publishers`
- **Video recording** (Phase 13, ffmpeg-backed): `start_recording`, `stop_recording`, `get_recording_status`
- **Objects / cups**: `add_objects`, `delete_objects`, `randomize_object_poses`, `randomize_single_object`, `sort_objects`, `sort_into_cups`, `add_cups`, `delete_cups`, `update_cups` (re-snaps cups to default `CUP_LAYOUT` poses; used by `vla_SO-ARM101/scripts/sim_reset.sh`), `randomize_cups` (samples CUP_LAYOUT params — mode/radius/angle/gap/color_order — and delegates placement to the existing `_cup_positions_arc` / `_cup_positions_line` generators. Each random sample is a coherent layout that's then validated against lego footprints AND live-queried robot-link AABBs; retries until both clear. ArUco markers stay aimed at origin via face-origin yaw + jitter. Used to stress-test the pick-place pipeline against varied cup poses.)
- **State**: `save_scene_state`, `restore_scene_state`, `sync_real_poses`
- **Escape hatch**: `execute_python_code` (runs arbitrary code in Kit's Python context; set a `result` variable to return data)

## Skills available

- **`isaac-sim-extension-dev`** (`~/.claude/skills/`): provides `isaacsim_launch.sh` + port mappings — referenced throughout the bring-up sequence above.
- **`sim-color-matcher`** (`.claude/skills/sim-color-matcher/`, project-local): pixel-level color matching from a real-world reference photo to sim materials. Browser-based drag-box picker → computes sRGB→linear ratio → applies to live `inputs:diffuseColor` via MCP. Two view modes (side-by-side, overlay), live ROS topic refresh, responsive layout. Use when tuning material colors against a photo — replaces eyeballed RGB guessing or vision-LLM color suggestions. Note: `.claude/` is gitignored at this repo root, so the skill is local-only unless an exception is added.

## Gotchas

- **CUDA 999 on Isaac Sim startup** (`Failed to create any GPU devices`): reload UVM — `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`. Safe; doesn't touch display modules. Verify with `~/env_isaaclab/bin/python3 -c "import ctypes; print(ctypes.CDLL('libcuda.so.1').cuInit(0))"` — expect `0`.
- **rclpy in Isaac Sim**: requires the Python 3.11 build at `/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages`. soarm101-dt's extension.py prepends this to `sys.path` at module load time — do not remove or rclpy node creation fails with `AttributeError: module 'rclpy' has no attribute 'impl'` followed by `rosidl_typesupport_c` not found.
- **Hot-reload scope**: `touch exts/soarm101-dt/so_arm101_dt/extension.py` re-executes module-level code (classes, dicts, constants, imports). Full restart needed only for `extension.toml` changes or adding new `.py` files.
- **Dedicated hidden viewports cost ~0.32 RTF each** (measured Phase 13). `_ViewportCameraPublisher.start()` supports active-viewport reuse only for this reason. For simultaneous multi-camera publishing, keep the action-graph path.
- **Wrist camera now uses `_ReplicatorCameraPublisher`** (hidden viewport, ~0.05 RTF), not the action graph. Kit 107.3 / isaacsim.ros2.bridge 4.9.3 silently fails to register the DDS publisher from `ROS2CameraHelper`. The class lives at `extension.py` near the top; quick_start wires it to `/World/SO_ARM101/.../wrist_camera`. The `setup_wrist_camera_action_graph` MCP tool still creates the graph (forward-compat) but the actual publisher is replicator-based. **Defensive camera_path re-bind**: `start()` sets `viewport.camera_path` immediately, but Kit drops the assignment when the viewport_api isn't fully constructed yet — the `on_frame` callback re-asserts it for the first 10 frames, otherwise the wrist topic renders `/OmniverseKit_Persp` instead of the wrist prim.
- **Workspace camera uses `_ViewportCameraPublisher`** (active main-viewport reuse, ~0 RTF). Auto-created in `quick_start` at `/World/workspace_camera`. **Aspect-ratio fix**: `vfov` is derived from `hfov × H/W` (square-pixel formula); previously hardcoded RealSense 16:9 vfov (42.5°) on a 4:3 buffer caused a center-cropped "zoomed" frame.
- **PhysX inner-prim collision hang**: if two rigid-body USD references resolve to the same inner prim name (e.g. spawning `/World/Objects/red_a/red_2x3` and `/World/Objects/red_b/red_2x3` from the same source USD), PhysX explodes during friction-patch generation and quick_start hangs after logging "Dropping contacts in solver because we exceeded limit of 32 friction patches." Keep `LEGO_USDS` at one unique USD per instance with unique inner body prim names. Multi-instance from a single USD requires either per-instance USD copies or a post-reference inner-prim rename.
- **Don't `touch extension.py` mid-session**: Kit's hot-reload watcher re-executes module-level code, but if it races with an in-flight `quick_start` (or any MCP call that's spawning prims), the stage half-mutates and Isaac Sim hangs. Edit-then-restart is safer than edit-then-touch when a quick_start has run since launch.
- **X11 GUI safety**: never `kill -9` Isaac Sim, RViz, or any process owning an X window (causes KWin BadWindow cascades). Use the skill's `close` subcommand or SIGTERM. Details in `~/.claude/CLAUDE.md` global rules.
- **Ghost ROS2 topics** (publisher count = 0 after deleting an action graph prim): cycle the timeline — `tl.stop(); for _ in range(20): app.update(); tl.play()` — to flush the Kit ROS2 bridge publishers.
- **Cross-extension stdout contamination**: if both `ur5e-dt` and `soarm101-dt` get loaded in the same Isaac Sim session, ur5e-dt's `LogRedirector` can hijack stdout and crash soarm101-dt's `on_startup` with `AttributeError: 'DigitalTwin' object has no attribute '_ext_log_lines'`. Fix by launching with only one `--enable` at a time via `isaacsim_launch.sh` (handles this).
- **SDF collision regeneration OOM**: gripper_link + moving_jaw_so101_v1_link use `PhysxSDFMeshCollisionAPI` at `resolution: 256` with `margin: 10 mm`. PhysX's voxelizer peaks at ~48 GB RAM during SDF generation (observed: full workstation hang). **Do not call `PhysxSDFMeshCollisionAPI.Apply()` or write any SDF attribute on a running stage** — it triggers full regeneration. Use read-only introspection (`GetSdfResolutionAttr().Get()` etc) to query. If SDF rebuild is unavoidable, run Isaac Sim with swap disabled and close other heavy processes first. This is the PhysX contactOffset fix's main risk vector — prefer authoring `physxCollision:contactOffset` (no SDF rebuild) over changing SDF params.
- **Cup collision padding is ROS-side only**: `_CUP_COLLISION_PADDING` (default 5% = 1.05) lives in `vla_SO-ARM101/.../control_gui.py` and only enlarges cups in MoveIt's planning scene. Isaac Sim's USD cup mesh is unchanged — physics contact telemetry (e.g. omni.physx contact-report subscription) stays honest. If you want padding-aware physics evaluation, that's a separate change in `soarm101-dt`'s scene-build path.
- **Deterministic planner lives ROS-side**: vla_SO-ARM101 routes every arm motion through tier-1 linear → tier-2 retract-pan-settle → OMPL fallback (opt-in per primitive; `_cmd_grasp_home` opts OUT). End-to-end test scripts at `vla_SO-ARM101/scripts/{test_qs_cycle,sim_reset,test_pick_all}.sh`. Isaac Sim is the physics backend; the planner has zero awareness of physics state beyond `/joint_states` feedback. Planner-collision verdicts (planning-scene) and physics-contact verdicts (PhysX) intentionally don't agree — padding makes the planner conservative on top of physics ground-truth.
- **`_attached_lego_tcp_offset` populated at attach time**: when `_cmd_gripper_close_for_object` succeeds, the held block's pose-in-tcp_link is captured and used by `_cmd_drop_sweep` to position the block (not tcp_link) over the cup center. Isaac Sim physics decides where the block actually settles after grasp_close — off-center attaches (typical: ~−12 mm in TCP X) are normal and the planner compensates. If physics changes (different lego mass/inertia, new gripper material), the typical offset magnitude can shift; verify with `Attach OK:` log line.
- **Asyncio + `execute_python_code` deadlock**: NEVER use `asyncio.run_until_complete(...)` or `await app.next_update_async()` inside MCP `execute_python_code`. The handler runs on Kit's main thread; awaiting a future fulfilled by that same thread is an unrecoverable same-thread deadlock. Symptoms: socket stops responding, all viewport/replicator publishers stop emitting frames (their `on_frame` callbacks share the thread), even SIGTERM/SIGINT are ignored — only SIGKILL recovers (with the X11 caveat above). Same root cause as the existing `capture_viewport_to_file` warning. Pattern: set state and return immediately, let Kit advance naturally; if you need to wait for frames, do it from a separate process (e.g. via the ROS topic).
- **`start_viewport_publisher` arg names**: the MCP tool takes `camera_prim_path` (NOT `camera_path`) and `topic_name` + optional `frame_id`. Passing `camera_path` returns `got an unexpected keyword argument 'camera_path'`.
- **`/workspace_camera_sim` follows the active viewport**: `_ViewportCameraPublisher` captures whatever the *active main viewport* is rendering at any given moment, not the prim it was started against. If anything switches the active camera (`_set_active_viewport_camera`, a GUI camera-switch, or Replicator's render-product creation that pulls focus), the topic silently starts publishing the new view. To capture a specific camera deterministically: set the active viewport to it first AND don't await frame settlement (see deadlock gotcha). Compare to `/wrist_camera_rgb_sim` which uses `_ReplicatorCameraPublisher` with a dedicated render product — view-stable regardless of active viewport.

## Robot Facts (SO-ARM101)

- 5-DOF arm: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` + `gripper_joint`.
- Forward axis = `+X`, joint drives: stiffness=15, damping=0.15, maxForce=3.
- **Drop motion is IK-planned, not a pure kinematic sweep** (despite the original design intent). Implemented in `vla_SO-ARM101/.../control_gui.py:_cmd_drop_sweep` — IK targets a wrist_flex ~55° pose using the MEASURED attach offset (`_attached_lego_tcp_offset`) and locks `shoulder_pan` to drop_point's value. drop_point itself is a pure pan rotation (only `shoulder_pan` and `wrist_roll` change). Cups anchored to shoulder pan axis (not world origin).
- ArUco markers on cups: red=ID 3, green=2, blue=1 (DICT_4X4_50). Remapped from the original 0/1/2 in Phase 10.2 commit `a3bd4aa` — mirrored across all 3 repos: `CUP_ARUCO_CONFIG` (soarm101-dt), `DROP_ID_LABELS` + `REAL_COLOR_TO_CUP` (control_gui), `aruco_config.json` (localizer).
- Wrist camera: OV9732 (640×480). Workspace camera **prim**: `/World/workspace_camera`. **Topic**: `/workspace_camera_sim` (note the `_sim` suffix is on the topic name, NOT the prim path). When calling `start_viewport_publisher` the kwargs are `camera_prim_path=/World/workspace_camera`, `topic_name=workspace_camera_sim`.

<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd:profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
