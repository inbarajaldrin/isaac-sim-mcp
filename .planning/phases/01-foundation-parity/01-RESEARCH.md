# Phase 1: Foundation Parity - Research

**Researched:** 2026-05-02
**Domain:** ROS 2 topic-parity, USD asset vendoring, OmniGraph TF/JointState publishers, MDL/texture sweep tooling
**Confidence:** HIGH for live snapshot, USD reference graph, OmniGraph node IDs, code-edit targets. MEDIUM for cold-cache verify-script flow (theoretical, not run end-to-end). LOW for none.

## Summary

This phase has the unusual property that **most decisions are already locked in CONTEXT.md** (15 D-numbered decisions). My job here was empirical: capture the live Gazebo `aic_eval` topic surface, walk the USD reference graph for the unified robot USD, identify the OmniGraph node IDs Isaac Sim 5.0 exposes for `/tf` and `/joint_states`, and produce the exact rename / edit / vendoring manifests the planner will turn into tasks.

All five investigation surfaces from CONTEXT.md were probed live on this host. Key empirical findings:

- **Gazebo's `/joint_states` ordering is NOT kinematic-chain order.** Live `aic_eval` publishes joints alphabetically: `[elbow_joint, gripper/left_finger_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]` — and includes the gripper joint (7 joints, not 6). The current `_joint_names` list in extension.py:361 is **kinematic-chain order without the gripper joint**, so PARITY-03 cannot pass without explicit re-ordering.
- **The unified USD has 232 OmniPBR.mdl bindings but 0 external texture references** — its materials are all flat-colored constants. Texture-sweep complexity is fully concentrated in the per-object USDs (NIC Card / SC Port / Task Board) which use `gltf/pbr.mdl` with `./textures/*.{png,jpg}` sibling references.
- **Current vendored `objects/` folder uses snake_case** (`objects/sc_port/sc_port.usd`) but is missing the sibling `textures/` folder that the per-object USDs reference. AIC's authoritative tree uses **capitalized folder names with spaces** (`assets/SC Port/sc_port_visual.usd` + `assets/SC Port/textures/Image_0.png`). Vendoring per D-05 needs to copy both the USDs AND the sibling `textures/` folders, preserving AIC's layout.
- **OmniGraph node IDs are confirmed**: `isaacsim.ros2.bridge.ROS2PublishJointState` (with `targetPrim`, `topicName`, `nodeNamespace`, `qosProfile` inputs) and `isaacsim.ros2.bridge.ROS2PublishTransformTree` (with `parentPrim`, `targetPrims`, `topicName`, `staticPublisher` boolean — `True` for `/tf_static`).
- **A latent kinematic-prim-path bug exists** (Kit log already shows it): the unified USD's default prim is `/World`, so referencing it into `/World/UR5e` creates joints at `/World/UR5e/aic_unified_robot/joints/<name>` — but the extension's `load_robot` looks them up at `/World/UR5e/joints/<name>`. Joint drives are silently never applied. Phase 1 must fix this for `/joint_states` to publish authoritatively.

**Primary recommendation:** Phase 1 should plan in a precise, ordered sequence: (1) capture / re-capture live snapshot, (2) one-time vendoring with AIC's capitalized-folder layout (incl. textures), (3) fix the kinematic prim path bug in `load_robot`, (4) add `setup_tf_publisher` and `setup_joint_state_publisher` atoms (two action graphs — one dynamic `/tf` + `/joint_states`, one static `/tf_static`), (5) string-rename the placeholder `_sim`/`_real` topics to live-snapshot-derived names, (6) delete `objects_poses_sim` / `sync_real_poses` atoms, (7) refactor `quick_start` per D-12, (8) ship `topic-parity-reference.md`, `texture-sweep.md`, `diff_tf_tree.py`, `verify_phase_1.sh`, and the docs cleanup. Texture sweep is the single biggest source of unknowns — plan budget for iteration.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Parity reference source**
- **D-01:** Live `aic_eval` Docker container is the canonical source of truth for "what Gazebo publishes." First Phase 1 deliverable: snapshot the live container (`ros2 topic list`, `ros2 topic info <name>`, `ros2 topic echo /joint_states --once`, `ros2 run tf2_tools view_frames`) into `exts/aic-dt/docs/topic-parity-reference.md`. The reference snapshot lands in Phase 1; the full cross-phase parity audit table (PARITY-12 deliverable) ships in Phase 3.

**Robot USD provenance**
- **D-02:** Use AIC's official `aic_unified_robot_cable_sdf.usd` from `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/Intrinsic_assets/`. Verified byte-identical (md5: `46616697c057701ae2025d44ace26844`) to the file already at `exts/aic-dt/assets/robot/`. Same applies to `scene/aic.usd`. Do NOT re-import xacro.
- **D-03:** Gripper is Robotiq Hand-E, NOT RG2. Whole-repo + project-docs cleanup. Targets at minimum: `exts/aic-dt/aic_dt/extension.py` lines 166, 999, 2357 (docstrings, log lines, return messages); `.planning/PROJECT.md`; `.planning/REQUIREMENTS.md`; `.planning/ROADMAP.md`; `CLAUDE.md` repo-root; `exts/aic-dt/docs/README.md`, `exts/aic-dt/docs/CHANGELOG.md`.
- **D-04:** Cable subtree stays `SetActive(False)` for Phase 1. Cable physics is Phase 3 work (SCENE-05). Document the workaround as a known Phase-3 work item in `topic-parity-reference.md`. Texture sweep still inspects cable-asset materials.

**Asset vendoring strategy**
- **D-05:** One-time vendoring of a precise subset of `Intrinsic_assets/` into `exts/aic-dt/assets/`, **preserving AIC's original folder layout** (`assets/NIC Card/`, `assets/SC Port/`, etc. — capitalized, with spaces). Researcher walks the unified USD's reference graph first and produces the exact dependency manifest. The current snake_case `assets/objects/{nic_card, sc_plug, …}` is retired during the vendoring step. After verification, `exts/aic-dt/assets/` is canonical — no ongoing sync.
- **D-06:** Edit vendored USDs in place when the texture/MDL sweep finds broken bindings. No override-layer indirection.

**Sweep & validation method**
- **D-07:** Texture/MDL sweep is a scripted log-grep loop: load M1 scene → grep `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` for `MDL`, `texture`, `missing`, `pink`, `fallback`, `not found` → each warning becomes a row in `exts/aic-dt/docs/texture-sweep.md`. Iterate fix → re-load → re-grep until zero warnings.
- **D-08:** TF tree validation via `view_frames` diff script. Capture live `aic_eval`'s `frames.gv` (during the D-01 reference snapshot) and Isaac Sim's `frames.gv` separately. `scripts/diff_tf_tree.py` parses both `.gv` files and prints frame-set diff + parent-child edge diff. Zero diff = pass.
- **D-09:** DX-01 cleanup is strict — zero `_sim`/`_real` anywhere on production surface. Concrete targets:
  - Camera RGB topics: `center_camera_rgb_sim`, `left_camera_rgb_sim`, `right_camera_rgb_sim` → rename to whatever live `aic_eval` publishes.
  - Wrench topic: `force_torque_sensor_broadcaster/wrench_sim` → live-snapshot-derived name.
  - MCP atoms `objects_poses_sim` and `sync_real_poses`: deleted entirely.
  - USD prim paths (`/World/workspace_camera_sim`, etc.): renamed too.
- **D-10:** `/tf` and `/tf_static` published via Isaac Sim TF action graph (`isaacsim.ros2.bridge` OmniGraph nodes). NOT an external `robot_state_publisher` ROS node. Researcher confirms exact node name(s) and inputs.
- **D-11:** `/joint_states` matches live `aic_eval` framing exactly — joint name set, ordering, `header.frame_id`, publish rate. Implementation: `isaacsim.ros2.bridge` ROS2PublishJointState node with explicit `jointNames` input populated from the snapshot.

**Phase 1 ops & scope**
- **D-12:** `quick_start` refactor scope is broader: Phase 1 reorganizes `quick_start` to club atoms cleanly per the future per-phase ordering: load scene → load robot → setup tf graph → setup camera publishers → setup wrench publisher → add objects → setup pose publisher → start sim. Partly does Phase 3's DX-03 work early.
- **D-13:** Extension runs without `~/Documents/aic` checked out. `scripts/verify_phase_1.sh` detects missing AIC repo and prints a clear message — exits non-zero on the snapshot-update step but skips it gracefully on a verify-only run. CLAUDE.md documents this assumption.
- **D-14:** `aic_eval` Docker snapshot pinned via image SHA-256 digest in `topic-parity-reference.md`. Captured via `docker pull` followed by `docker inspect --format='{{index .RepoDigests 0}}'`. Future re-snapshots use that digest, not `:latest`.
- **D-15:** `scripts/verify_phase_1.sh` is hybrid-runtime. Detects whether MCP port 8768 is responding. If yes: runs MCP commands against the attached extension. If no: invokes `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt` (or `launch_postload.py` cold-cache fallback), runs the checks, optionally tears down. Same script works in dev (attached) and one-shot/CI-style (cold) modes.

### Claude's Discretion

- Exact OmniGraph node names + wiring inside the TF action graph (D-10) — researcher decides via `isaac-sim-extension-dev` / `nvidia-suite-docs` skills.
- Exact subset to vendor under D-05 — researcher walks the USD reference graph and produces the manifest before any copy happens.
- Format of `topic-parity-reference.md` — table layout, structure, what counts as a "topic surface row" (Claude follows the existing extension-doc style).
- `view_frames` diff script implementation language (Python vs bash, GraphViz vs networkx parser).
- The exact log-grep regex patterns in D-07 — refine after first run.

### Deferred Ideas (OUT OF SCOPE)

- Camera resolution match (640×480 → 1152×1024) — CAM-01, M2 work.
- Cable physics strategy — Phase 3 SCENE-05.
- Full ros2_control surface (`scaled_joint_trajectory_controller`, `force_mode_controller`, etc.) — Phase 2 PARITY-09/10/11. Phase 1 only needs `/joint_states`.
- Object TF frames CheatCode reads (`{cable_name}/{plug_name}_link`, port frames) — Phase 3 SCENE-06. Phase 1's TF tree is robot + cameras + tabletop only.
- Headless / CI integration — M2+. `verify_phase_1.sh` is dev-friendly first.
- Override-layer USD pattern for fixes — explicitly rejected (D-06).
- External `robot_state_publisher` ROS node — explicitly rejected (D-10).
- PARITY-12 cross-phase audit table comprehensive deliverable — Phase 3.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| PARITY-01 | Isaac Sim loads UR5e + Robotiq Hand-E + 3 wrist cameras matching `aic_description` xacro | Verified Hand-E (NOT RG2) in `ur_gz.urdf.xacro`. Unified USD `aic_unified_robot_cable_sdf.usd` is byte-identical to AIC's source — no kinematic divergence by construction. PARITY-01 is largely satisfied today; remaining gap is the doc/string fix in D-03 + the kinematic-prim-path bug. |
| PARITY-02 | Isaac Sim loads `aic_assets` meshes for task board, ports, mount rails, cables, AIC enclosure | Vendoring manifest produced below (Step 4). Current `objects/` snake_case layout is broken (texture sibling folders missing). Re-vendoring required per D-05. |
| PARITY-03 | `/joint_states` matches Gazebo joint name set + ordering | **Live snapshot captured**: `[elbow_joint, gripper/left_finger_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]` — alphabetical, 7 joints incl. gripper. `isaacsim.ros2.bridge.ROS2PublishJointState` node confirmed; new action graph + atom required. |
| PARITY-04 | `/tf` and `/tf_static` carry same robot + gripper + camera frames in same hierarchy | **Live frames.gv captured** (`aic_frames_live.gv` saved). `isaacsim.ros2.bridge.ROS2PublishTransformTree` node confirmed (set `staticPublisher=True` for `/tf_static`). New action graph(s) + atom required. Diff-script harness (D-08) is the proof artifact. |
| TEX-01 | Zero pink/black/missing-texture warnings on M1 scene | Unified USD has zero texture-binding issues (constant-color OmniPBR). All texture risk is in the per-object USDs which need vendored sibling `textures/` folders. |
| TEX-02 | GLB-imported PBR maps that lost binding during USD conversion are rebound | Per-object USDs use `gltf/pbr.mdl` shaders with `./textures/Image_*.{png,jpg}` refs — these are the at-risk bindings. Snake_case rename in current `objects/` already broke them. |
| TEX-03 | Texture sweep findings + fix log captured | `exts/aic-dt/docs/texture-sweep.md` per D-07 — log-grep loop iteration. |
| DX-01 | No `_sim`/`_real` suffixes on production topics | **33 occurrences in extension.py** (full table below). Plus changelog/README mentions. Replacement names derived from live snapshot. |
</phase_requirements>

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| `/joint_states` publish | Isaac Sim Kit / OmniGraph | — | `isaacsim.ros2.bridge.ROS2PublishJointState` reads articulation state and emits ROS 2 messages directly from sim thread |
| `/tf` publish (dynamic frames) | Isaac Sim Kit / OmniGraph | — | `isaacsim.ros2.bridge.ROS2PublishTransformTree` walks USD prim tree and emits TFMessage |
| `/tf_static` publish | Isaac Sim Kit / OmniGraph | — | Same node with `staticPublisher=True`. Could also be a separate graph if static-vs-dynamic prim split is needed. |
| Asset vendoring | Filesystem / `exts/aic-dt/assets/` | — | One-time file copy; resolved at runtime via `_get_assets_folder()` `file://` URI |
| Topic surface validation | External ROS 2 CLI (`ros2 topic list`, `tf2_tools view_frames`) | Isaac Sim MCP `execute_python_code` | External CLI is the canonical proof; in-process probes are faster but less authoritative |
| Texture / MDL sweep | Kit log file scraping | Isaac Sim viewport visual inspection | Log-grep is automatable; viewport is final visual check |
| `verify_phase_1.sh` orchestration | Bash + MCP socket client | Isaac Sim launcher (cold-cache fallback) | Hybrid: detect-attached-or-launch pattern |
| RG2→Hand-E doc cleanup | Source files (Python + Markdown) | Test files (read-only — currently still RG2) | Code/docs only; test files in `exts/aic-dt/aic_dt/tests/test_wrist3_force.py` are stale and not in Phase 1 scope per D-03 minimum target list |

## Live Reference Snapshot — `aic_eval` Topic Surface

**Captured 2026-05-02 from headless `aic_eval` Docker container (1 minute live run).** Image digest pinned per D-14:

```
ghcr.io/intrinsic-dev/aic/aic_eval@sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433
```

(Local image ID: `sha256:34e8a0bcee744062fb7f1171173bbc77a0b74a5da46f192cfe7b954565686b5b`, size 6.36 GB.)

**Bringup verified working with this command** (the only invocation pattern that produces a populated topic surface):

```bash
docker run -d --name aic_eval --gpus all --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host --privileged \
  ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  ground_truth:=true start_aic_engine:=true gazebo_gui:=false
```

Notes for the future:
- The container uses **Zenoh as RMW** with custom router config. External `ros2 topic list` from the host fails to discover topics over default DDS. To probe topics, exec inside the container with the right env: `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"`.
- `ros2 topic list` requires `ros2 daemon stop` first (existing daemon doesn't pick up the Zenoh transport mid-flight). Subsequent `ros2 topic info` / `ros2 topic hz` calls work without the `--no-daemon` flag (which doesn't exist on those subcommands).
- The eval container's `aic_engine` waits for an `aic_model` node before scenario starts. Topics ARE published while it's waiting (joint_states, tf, tf_static, etc. — the controller-manager is up).

### Full topic list (35 topics)

Saved as artifact: `.planning/phases/01-foundation-parity/aic_topics_live.txt`.

```
/aic/gazebo/contacts/off_limit
/aic_controller/controller_state
/aic_controller/joint_commands
/aic_controller/pose_commands
/aic_controller/transition_event
/center_camera/camera_info
/center_camera/image
/clock
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_manager/statistics/full
/controller_manager/statistics/names
/controller_manager/statistics/values
/diagnostics
/dynamic_joint_states
/fts_broadcaster/transition_event
/fts_broadcaster/wrench
/fts_broadcaster/wrench_filtered
/joint_state_broadcaster/transition_event
/joint_states
/left_camera/camera_info
/left_camera/image
/observations
/parameter_events
/right_camera/camera_info
/right_camera/image
/robot_description
/rosout
/scoring/insertion_event
/scoring/tf
/tf
/tf_static
```

### Phase 1's passive sensor surface (subset of the above)

| Topic | Type | Publisher node (live) | QoS | Rate | Phase 1 action |
|-------|------|----------------------|-----|------|---------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` (1 publisher) | RELIABLE / VOLATILE / KEEP_LAST(42) | **500 Hz** | Publish from Isaac Sim |
| `/tf` | `tf2_msgs/msg/TFMessage` | `tf_relay` + `robot_state_publisher` (2 publishers) | RELIABLE / VOLATILE / KEEP_LAST(10) | **500 Hz** | Publish from Isaac Sim |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` + `ground_truth_static_tf_publisher` (2 publishers) | RELIABLE / **TRANSIENT_LOCAL** / KEEP_LAST(1) | static (one-shot) | Publish from Isaac Sim with `staticPublisher=True` |

**500 Hz is matched to `update_rate: 500` in `aic_ros2_controllers.yaml` (`controller_manager.ros__parameters.update_rate`).** Phase 1's Isaac Sim publisher does NOT need to match exactly — `OnPlaybackTick`-driven graphs publish per render frame which is typically 60 Hz. CheatCode polls TF lookups and doesn't rate-check. PARITY-09/10 (Phase 2) is where rate matters; Phase 1 just needs *something* publishing.

### `/joint_states` echo (CRITICAL — name set + ordering)

```yaml
header:
  stamp: { sec: 105, nanosec: 50000000 }
  frame_id: base_link
name:
- elbow_joint
- gripper/left_finger_joint
- shoulder_lift_joint
- shoulder_pan_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
position: [-1.667, 0.0073, -1.354, -0.160, -1.691, 1.571, 1.411]
velocity: [...]
effort: [...]
```

Key facts:
- **7 joints, alphabetical order** (NOT kinematic-chain order). The current `_joint_names` list in extension.py:361 is wrong on both counts (it has 6 joints in chain order).
- **`gripper/left_finger_joint`** (with the `/`) is the gripper joint name. (`gripper/` is the `gripper_tf_prefix` from the xacro — same prefix appears in TF frames.)
- **`header.frame_id` is `base_link`.**
- **`gripper/right_finger_joint` is NOT in `/joint_states`** (only `left_finger_joint`). The Hand-E ros2_control xacro likely declares only the left finger as a state interface, with the right finger mimicking via fixed-mimic constraint. Verify against xacro before encoding — but **publish only what Gazebo publishes** for parity.

### Live TF tree (`view_frames` from container)

Saved as artifact: `.planning/phases/01-foundation-parity/aic_frames_live.gv` (graphviz format).

Tree structure (parent → child edges):

```
world → aic_world
world → tabletop
tabletop → base_link
base_link → base_link_inertia
base_link → base
base_link_inertia → shoulder_link
shoulder_link → upper_arm_link
upper_arm_link → forearm_link
forearm_link → wrist_1_link
wrist_1_link → wrist_2_link
wrist_2_link → wrist_3_link
wrist_3_link → flange
wrist_3_link → ft_frame
flange → tool0
tool0 → cam_mount/cam_mount_link
cam_mount/cam_mount_link → ati/base_link
ati/base_link → ati/tool_link
ati/tool_link → gripper/hande_base_link
gripper/hande_base_link → gripper/hande_finger_link_l
gripper/hande_base_link → gripper/hande_finger_link_r
gripper/hande_base_link → gripper/tcp
cam_mount/cam_mount_link → center_camera/camera_link
cam_mount/cam_mount_link → left_camera/camera_link
cam_mount/cam_mount_link → right_camera/camera_link
center_camera/camera_link → center_camera/sensor_link
center_camera/sensor_link → center_camera/optical
left_camera/camera_link → left_camera/sensor_link
left_camera/sensor_link → left_camera/optical
right_camera/camera_link → right_camera/sensor_link
right_camera/sensor_link → right_camera/optical
```

**31 frames total**, 30 edges, 1 root (`world`). 6 dynamic joints (`base_link_inertia → shoulder_link → upper_arm_link → forearm_link → wrist_1 → wrist_2 → wrist_3`) and 24 static frames.

Notes:
- The dynamic part of the tree (`upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`, plus the gripper finger joints) publishes at 440 Hz via the `tf_relay` node (`Average rate: 440.59`).
- All static frames (cameras, gripper base, ATI sensor, world↔aic_world↔tabletop↔base_link) come from `robot_state_publisher` and `ground_truth_static_tf_publisher` on `/tf_static` (transient_local) — they appear in `view_frames` with rate `10000.0` because `view_frames` synthesizes that for static refs.
- The gripper finger joints (`hande_finger_link_l`, `hande_finger_link_r`) publish at 440 Hz too — they're NOT static. So Isaac Sim's TF graph needs to track them as live frames.
- **Isaac Sim's TransformTree node will derive frame names from prim names by default**. AIC's frame names contain slashes (`gripper/hande_base_link`, `cam_mount/cam_mount_link`, etc.) — USD prim paths don't allow slashes in single names. The unified USD's prim names (e.g. `gripper_hande_base_link`) need to be remapped to AIC's slashed names. The TransformTree node's `inputs:targetPrims` lets you specify which prims to publish; the frame_id is derived from prim name. **Mapping: USD prim → ROS frame_id** is the trickiest piece of PARITY-04 — likely needs a USD-side rename or custom attribute.

### Other config from container

- `aic_ros2_controllers.yaml` (in `/ws_aic/install/share/aic_bringup/config/`) declares:
  - `fts_broadcaster.frame_id: ati/tool_link` — confirms the wrench `frame_id` (relevant for Phase 2 PARITY-05).
  - `aic_controller.joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]` — 6-joint kinematic chain (no gripper). This is the controller's view, not what /joint_states publishes.
  - `aic_controller.kinematics.tip: gripper/tcp` — the TCP frame is `gripper/tcp` (visible in TF tree above).

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `isaacsim.ros2.bridge` | 5.0 (current Isaac Sim install) | OmniGraph nodes for ROS 2 publish/subscribe | Isaac Sim's official ROS 2 integration; alternative is a custom rclpy node which is rejected by D-10 |
| `omni.graph.core` | bundled with Isaac Sim 5.0 | Building action graphs (`og.Controller.edit`) | The codebase already uses this for camera + force publishers |
| `pxr.Usd` / `pxr.UsdGeom` / `pxr.UsdPhysics` / `pxr.UsdShade` | bundled with Isaac Sim 5.0 | USD walking, vendoring, MDL inspection | Standard Pixar USD API; available standalone via `~/packman-repo/chk/usd.py311.manylinux_2_35_x86_64.stock.release/0.24.05.kit.6-gl.14415+d9efdd65/` for tooling outside Kit |
| Docker (already installed) | — | Run `aic_eval` headless for snapshot capture | Per AIC repo's `run_cheatcode.sh`; native pixi install fails on Ubuntu 22.04 |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `tf2_tools` (ROS 2 Kilted, in container) | — | `ros2 run tf2_tools view_frames -o aic_frames` | Capture `frames.gv` for diff |
| `graphviz` Python lib | optional | Parse `.gv` for `diff_tf_tree.py` | If using Python-based diff; bash + diff also works |
| `bash` + `nc` / `python3 socket` | — | Liveness probe for MCP port 8768 in `verify_phase_1.sh` | Hybrid runtime detection |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| `ROS2PublishTransformTree` with `staticPublisher=True` for `/tf_static` | Two separate graphs (one dynamic `/tf`, one static `/tf_static`) | Two graphs are more explicit but more code; one node with the bool flag is simpler. Recommended: **one graph with two `ROS2PublishTransformTree` nodes** — one for dynamic frames into `/tf`, one for static frames into `/tf_static` with `staticPublisher=True`. Filtering which prims go where = `inputs:targetPrims` selector. |
| `ROS2PublishTransformTree` for whole-articulation TF | Per-frame `ROS2PublishRawTransformTree` nodes | Articulation traversal is automatic with `targetPrims` set to articulation root → much simpler than per-link nodes. Use Raw only if a frame needs custom parent (Phase 3 cable frames may need this). |
| `view_frames` `.gv` diff in Python | Bash + `diff` after sorting edges | Python parses graphviz reliably; bash diff may show whitespace noise. Prefer Python (~30 lines). |

**Installation:** No new package installs — everything is already on the host (Isaac Sim 5.0 in `~/env_isaaclab`, Docker, ROS 2 in container).

**Version verification:**
- Isaac Sim 5.0 install confirmed at `~/env_isaaclab/lib/python3.11/site-packages/isaacsim/` and via the postload script which boots `~/env_isaaclab/bin/isaacsim`.
- `aic_eval` image SHA-256: `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433` (verified 2026-05-02).
- Pixar USD (standalone tooling): 0.24.05 at `~/packman-repo/chk/usd.py311.manylinux_2_35_x86_64.stock.release/...`.

## Architecture Patterns

### System Architecture Diagram

```
                     ┌─────────────────────────────────────────────────┐
                     │  Live Gazebo aic_eval (Docker, port-isolated)   │
                     │  ─ joint_state_broadcaster                      │
                     │  ─ robot_state_publisher (URDF→TF)              │
                     │  ─ ros_gz_bridge (sensor topics)                │
                     └────────────────────┬────────────────────────────┘
                                          │ docker exec ros2 topic list / info / echo
                                          ▼
                     ┌─────────────────────────────────────────────────┐
   Phase 1 build →   │  topic-parity-reference.md (D-01, D-14)         │
   inputs            │  aic_frames_live.gv  (D-08 reference)           │
                     │  Image SHA-256 digest                           │
                     └────────────────────┬────────────────────────────┘
                                          │ (referenced by verify_phase_1.sh)
                                          ▼
   ┌──────────────────────────────────────────────────────────────────────┐
   │  aic-dt extension (Isaac Sim Kit, MCP port 8768)                     │
   │                                                                      │
   │  load_scene  →  load_robot  →  setup_tf_publisher (NEW)  →           │
   │   setup_joint_state_publisher (NEW)  →  setup_force_publisher  →     │
   │   setup_wrist_cameras  →  add_objects  →  setup_pose_publisher  →    │
   │   play_scene                                                         │
   │                                                                      │
   │   ──[ vendored ]──>  exts/aic-dt/assets/                             │
   │     robot/aic_unified_robot_cable_sdf.usd  (md5 byte-identical)      │
   │     scene/aic.usd  (md5 byte-identical)                              │
   │     assets/NIC Card/...     (NEW — preserved AIC layout, capitalized)│
   │     assets/NIC Card Mount/textures/Image_*.jpg  (NEW)                │
   │     assets/SC Port/sc_port_visual.usd  +  textures/Image_*.png       │
   │     assets/Task Board Base/base_visual.usd                           │
   │                                                                      │
   │   ──[ publishes ]──>                                                 │
   │     /joint_states     (isaacsim.ros2.bridge.ROS2PublishJointState)   │
   │     /tf               (ROS2PublishTransformTree, staticPublisher=F)  │
   │     /tf_static        (ROS2PublishTransformTree, staticPublisher=T)  │
   │     /clock            (already published)                            │
   │     /center_camera/image      (renamed from _sim)                    │
   │     /left_camera/image        (renamed from _sim)                    │
   │     /right_camera/image       (renamed from _sim)                    │
   └──────────────────────────────────────────────────────────────────────┘
                                          ▲
                                          │
   ┌──────────────────────────────────────────────────────────────────────┐
   │  scripts/verify_phase_1.sh  (D-15 hybrid runtime)                    │
   │   1. probe MCP port 8768                                             │
   │   2. if open: send ping ; else: launch isaacsim_launch.sh / postload │
   │   3. send `quick_start` MCP cmd                                      │
   │   4. external: ros2 topic list  + filter passive                     │
   │   5. external: ros2 run tf2_tools view_frames -o sim_frames          │
   │   6. invoke scripts/diff_tf_tree.py aic_frames_live.gv sim_frames.gv │
   │   7. grep extension.py for _sim/_real → expect 0                     │
   │   8. grep Kit log for texture/MDL warnings → expect 0                │
   └──────────────────────────────────────────────────────────────────────┘
```

### Recommended File / Module Structure

```
exts/aic-dt/
├── aic_dt/
│   └── extension.py                # Editing — see "DX-01 / D-09 rename table" below
├── assets/
│   ├── robot/
│   │   └── aic_unified_robot_cable_sdf.usd        # KEEP (md5 ok)
│   ├── scene/
│   │   └── aic.usd                                 # KEEP (md5 ok)
│   └── assets/                                     # NEW — replaces objects/, AIC layout
│       ├── NIC Card/                               # capitalized, with space
│       │   ├── nic_card.usd                        #   may need re-vendoring (verify refs)
│       │   ├── nic_card_visual.usd
│       │   └── textures/                           # SIBLING textures folder
│       │       ├── Image_0.jpg
│       │       ├── Image_1.jpg
│       │       ├── Image_2.jpg
│       │       └── NIC_Albedo.jpg
│       ├── NIC Card Mount/
│       │   ├── nic_card_mount_visual.usd
│       │   ├── nic_card_visual.usd
│       │   └── textures/Image_{0,1,2}.jpg + NIC_Albedo.jpg
│       ├── SC Port/
│       │   ├── sc_port_visual.usd                  # 6 internal refs (3 broken — props/*.usd MISSING)
│       │   ├── sc_port.usd                         # currently in objects/sc_port/
│       │   └── textures/
│       │       ├── Image_0.png
│       │       └── Image_1.png
│       ├── SC Plug/
│       │   └── sc_plug_visual.usd
│       └── Task Board Base/
│           └── base_visual.usd
├── docs/
│   ├── README.md                   # EDIT (D-03 RG2→Hand-E + AIC scope, DX-05 partial)
│   ├── CHANGELOG.md                # EDIT (D-03 RG2→Hand-E + Phase 1 entry)
│   ├── topic-parity-reference.md   # NEW (D-01)
│   └── texture-sweep.md            # NEW (D-07)
└── scripts/
    ├── launch_postload.py          # KEEP (cold-cache path)
    ├── verify_phase_1.sh           # NEW (D-15)
    └── diff_tf_tree.py             # NEW (D-08)
```

### Pattern 1: Building an OmniGraph node graph in `og.Controller.edit()` style

The codebase's existing pattern (extension.py:1001-1059 for joint subscribe + clock; 1184-1239 for cameras; 1776-1813 for object pose tree). Phase 1 follows this exact shape.

```python
# Source: aic-dt extension.py:1001-1059 + isaacsim.ros2.bridge OGN docs (Isaac Sim 4.2 install,
#  same node IDs apply to 5.0 with namespace `isaacsim.ros2.bridge.*` instead of `omni.isaac.ros2_bridge.*`)
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

graph_path = "/Graph/ActionGraph_TF"

(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("publish_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("publish_tf_static", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # ROS 2 context
            ("ros2_context.inputs:useDomainIDEnvVar", True),
            ("ros2_context.inputs:domain_id", 0),

            # Dynamic TF publisher → /tf
            ("publish_tf.inputs:topicName", "tf"),
            ("publish_tf.inputs:nodeNamespace", ""),
            ("publish_tf.inputs:staticPublisher", False),
            # parentPrim and targetPrims set via relationships (see below)

            # Static TF publisher → /tf_static
            ("publish_tf_static.inputs:topicName", "tf_static"),
            ("publish_tf_static.inputs:nodeNamespace", ""),
            ("publish_tf_static.inputs:staticPublisher", True),  # → TRANSIENT_LOCAL QoS

            # JointState publisher → /joint_states
            ("publish_joint_state.inputs:topicName", "joint_states"),
            ("publish_joint_state.inputs:nodeNamespace", ""),
            # targetPrim set via relationship below

            ("isaac_read_simulation_time.inputs:resetOnStop", False),
        ],
        og.Controller.Keys.CONNECT: [
            ("on_playback_tick.outputs:tick", "publish_tf.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "publish_tf_static.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "publish_joint_state.inputs:execIn"),
            ("ros2_context.outputs:context", "publish_tf.inputs:context"),
            ("ros2_context.outputs:context", "publish_tf_static.inputs:context"),
            ("ros2_context.outputs:context", "publish_joint_state.inputs:context"),
            ("isaac_read_simulation_time.outputs:simulationTime", "publish_tf.inputs:timeStamp"),
            ("isaac_read_simulation_time.outputs:simulationTime", "publish_tf_static.inputs:timeStamp"),
            ("isaac_read_simulation_time.outputs:simulationTime", "publish_joint_state.inputs:timeStamp"),
        ],
    }
)

# After og.Controller.edit, set the relationship inputs (target prims) directly on the prim:
import omni.usd
from pxr import Sdf
stage = omni.usd.get_context().get_stage()

# For PublishJointState: targetPrim is the articulation root prim
js_node = stage.GetPrimAtPath(f"{graph_path}/publish_joint_state")
target_prim = js_node.GetRelationship("inputs:targetPrim")
if not target_prim:
    target_prim = js_node.CreateRelationship("inputs:targetPrim", custom=True)
target_prim.SetTargets([Sdf.Path("/World/UR5e/aic_unified_robot/root_joint")])  # see "Kinematic prim path bug" below

# For PublishTransformTree (dynamic): parentPrim = world frame, targetPrims = root of articulation
tf_node = stage.GetPrimAtPath(f"{graph_path}/publish_tf")
parent_rel = tf_node.GetRelationship("inputs:parentPrim") or tf_node.CreateRelationship("inputs:parentPrim", custom=True)
parent_rel.SetTargets([Sdf.Path("/World")])  # publishes children's poses relative to /World
targets_rel = tf_node.GetRelationship("inputs:targetPrims") or tf_node.CreateRelationship("inputs:targetPrims", custom=True)
targets_rel.SetTargets([Sdf.Path("/World/UR5e")])  # whole articulation

# For PublishTransformTree (static): could use same configuration — staticPublisher=True overrides QoS only
```

**Important:** the `ROS2PublishJointState.inputs:targetPrim` is "USD reference to the robot prim" — it works on the articulation root, not individual joints. The node automatically discovers all joints in the articulation and publishes them. **It does not have a `jointNames` ordering input** (per Isaac Sim 4.2 OGN docs which I confirmed in the local install — the only inputs are `context, execIn, nodeNamespace, qosProfile, queueSize, targetPrim, timeStamp, topicName`). **This is a problem** for PARITY-03 since Gazebo's ordering is alphabetical and Isaac Sim publishes in articulation-discovery order. See "Open Questions" below.

### Pattern 2: Action graph cleanup pattern (already in codebase)

Both extensions (aic-dt and ur5e-dt) follow this shape:

```python
graph_path = "/Graph/ActionGraph_<NAME>"
stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath(graph_path):
    stage.RemovePrim(graph_path)  # Idempotent — re-running setup recreates from scratch
```

Phase 1's new TF/JointState atom should follow the same shape.

### Anti-Patterns to Avoid

- **Hand-rolling a `/tf` publisher in Python via rclpy on a physics callback.** OmniGraph integration handles threading, timestamps, and QoS for free. Custom rclpy publishers run on the wrong thread and leak.
- **Using prim names directly as ROS frame_ids without remapping.** USD prim names can't contain `/`; AIC's frames use `gripper/hande_base_link`, `cam_mount/cam_mount_link`, `center_camera/optical`. The unified USD's prim names are `gripper_hande_base_link` (underscores). Either rename USD prims to `gripper:hande_base_link` (USD allows colons in names? — verify), or use a different node configuration that lets you override frame_ids per prim.
- **Per-link `ROS2PublishRawTransformTree` for the whole arm.** That's 30+ nodes; use the articulation-traversing `ROS2PublishTransformTree` instead.
- **Two separate action graphs for /tf and /tf_static when one with two nodes works.** Existing camera graphs prove multi-node graphs are stable.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| `/tf` publisher | Custom rclpy.Node + tf2_ros.TransformBroadcaster on a physics callback | `isaacsim.ros2.bridge.ROS2PublishTransformTree` in an OmniGraph | Native node handles articulation traversal, threading, QoS auto-config, simulationTime stamping. Custom Python on physics callback runs on the wrong thread (rclpy expects executor pumping) and TFBroadcaster has subtle deadline-/lifespan-/durability defaults that may break Gazebo-parity. |
| `/joint_states` publisher | Custom callback that walks `articulation.get_joint_positions()` and publishes `sensor_msgs/JointState` | `isaacsim.ros2.bridge.ROS2PublishJointState` | Same reasoning. (Plus: see Open Question about ordering — Isaac Sim's order may not match Gazebo's alphabetical out-of-the-box, but custom code has the same problem.) |
| `view_frames` diff parser | Hand-tokenize the `.gv` file with regex | Use Python `graphviz` lib OR a simple line-based "extract `\"a\" -> \"b\"` pairs" approach | The format is stable but quoting can vary. A 30-line Python with `re.findall(r'"([^"]+)" -> "([^"]+)"', ...)` is sufficient — don't pull `pygraphviz`. |
| MCP port-8768 liveness check | curl with timeout (no curl, it's TCP not HTTP) | `python3 -c "import socket; s=socket.socket(); s.settimeout(2); s.connect(('localhost',8768))"` exit code | One-liner, exit 0/non-0, deterministic. The repo's `mcp_test.py` (`~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py`) has a working pattern; reuse the connect+send-ping idiom. |
| Kit log discovery | Hardcode the path in shell script | Glob the most recent log: `ls -t ~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log \| head -1` | Logs are per-launch timestamped. Always use newest. CLAUDE.md documents this path. |
| Texture-sweep regex | Build a pattern from scratch | Start with the patterns surfaced by the dry-run sweep; iterate per D-07 ("refine after first run"). | The actual warning string format depends on the asset and the failure mode. CONTEXT.md D-07 explicitly defers regex tuning until first run. |

**Key insight:** Every "publisher" surface in this phase has a built-in OmniGraph node. The reason custom solutions seem appealing is that the node names look unfamiliar — but the node is always simpler, and the existing `setup_action_graph`, `setup_force_publish_action_graph`, `_create_camera_actiongraph`, and `create_pose_publisher` methods in extension.py all prove the pattern works inside this codebase.

## Runtime State Inventory

Phase 1 is largely a code/asset refactor. There IS one rename surface (`_sim`/`_real` → live-snapshot-derived names) and one prim-path surface (`/World/workspace_camera_sim` → renamed). The runtime state inventory is small but non-empty:

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | None — Phase 1 doesn't touch any database, key/value store, or persistent state. Scene-state JSON files (`save_scene_state` MCP atom) reference object NAMES not topic strings, so they survive. | None |
| Live service config | None — no external services consume the renamed topics until Phase 2 (controller plugins) or Phase 4 (aic_engine). Phase 1's verifier is the only thing that reads them. | None |
| OS-registered state | None — no OS-level task scheduler, systemd unit, or pm2 entry references aic-dt-specific strings | None |
| Secrets / env vars | `MCP_SERVER_PORT=8768`, `MCP_CLIENT_OUTPUT_DIR` (CLAUDE.md). Neither is renamed in Phase 1. | None |
| Build artifacts / installed packages | Isaac Sim's `DerivedDataCache` at `~/.cache/ov/DerivedDataCache` is **152.9 MB and healthy** (verified 2026-05-02). Backups exist (incl. `DerivedDataCache.bak.known-good`). | After first successful Phase 1 verify run, take a fresh snapshot per CLAUDE.md operating discipline: `~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py snapshot`. |

**Tests file (`exts/aic-dt/aic_dt/tests/test_wrist3_force.py`) still references `/World/RG2_Gripper`** — this is a test artifact, NOT runtime production state. CONTEXT.md D-03 explicitly lists test files as out of the minimum target list. The test itself is stale (RG2 doesn't exist in the unified USD); fixing it is DX-05 (Phase 4) territory.

## Common Pitfalls

### Pitfall 1: Joint name ordering mismatch in `/joint_states`
**What goes wrong:** `isaacsim.ros2.bridge.ROS2PublishJointState` publishes joints in the order the articulation's `articulation.dof_names` returns them. That order is determined by USD prim traversal, NOT alphabetical. Gazebo's `joint_state_broadcaster` publishes alphabetical. PARITY-03 fails silently — message arrives but downstream consumers (e.g., aic_controller's joint-name-to-position lookup) may map the wrong joint to the wrong slot.
**Why it happens:** Isaac Sim's node has no `jointNames` ordering input (verified against OGN docs).
**How to avoid:** Three options to evaluate during Phase 1 planning:
1. **Reorder USD prims** so articulation-discovery order matches alphabetical. Risky — touches the byte-identical-USD constraint.
2. **Add a thin Python wrapper** that subscribes to the Isaac Sim publisher's output, reorders, and re-publishes on the SAME topic. Defeats "no bridges" architectural law in spirit.
3. **Verify empirically first** that the constraint matters. CheatCode reads `/joint_states` via name-indexed lookup (it's a `JointState` message, fields `name[]` and `position[]` are parallel arrays). If consumers always do `name.index(joint)` to find positions, ordering doesn't matter. Phase 1 should test this before over-engineering.
**Warning signs:** PARITY-03 SC #2 says "same six UR5e joint names in same ordering" — but is "ordering" a verifier check or an actual functional requirement? The CONTEXT.md and discussion log both say "matches live framing exactly" (D-11), so the success criterion is strict. **Recommendation: probe option 3 first — if downstream is name-indexed, document that ordering ISN'T functionally required; otherwise plan a wrapper.**

### Pitfall 2: USD prim path mismatch — kinematic surface bug already in code
**What goes wrong:** Current `load_robot` (extension.py:933-999) configures joint drives at `/World/UR5e/joints/<joint_name>`. But the unified USD's default prim is `/World` (verified via `Usd.Stage.GetDefaultPrim()`), so `add_reference_to_stage(prim_path="/World/UR5e")` puts the reference's contents under `/World/UR5e/`. The actual joint paths become `/World/UR5e/aic_unified_robot/joints/<joint_name>`. The current code's `for _ in range(10)` retry loop "succeeds" (the prim path /World/UR5e exists — it's just an empty Xform wrapper), then the joint-drive lookup at /World/UR5e/joints/* silently fails on every joint (logs show `Warning: Joint not found` repeated 6× in `kit_20260501_221152.log`).
**Why it happens:** The unified USD was authored with `/World` as default prim and `/World/aic_unified_robot` as the actual robot subtree. Sibling extensions (ur5e-dt) work because their robot USDs default-prim is the robot itself.
**How to avoid:** In `load_robot`, change `joint_path = f"{prim_path}/joints/{joint_name}"` to `joint_path = f"{prim_path}/aic_unified_robot/joints/{joint_name}"` OR change the `add_reference_to_stage` `prim_path` argument so the reference root is `/World` and the references default-prim contents land at `/World/aic_unified_robot` directly (then `_robot_prim_path = "/World/aic_unified_robot"` everywhere).
**Warning signs:** Joint drives don't apply → the robot is loaded but won't track commands. **For Phase 1 this surfaces in PARITY-03 as well**: `ROS2PublishJointState.inputs:targetPrim` needs the articulation root, which is at `/World/UR5e/aic_unified_robot/root_joint` (not `/World/UR5e`). Phase 1 must address this.
**Recommended fix:** Change `_robot_prim_path = "/World/UR5e/aic_unified_robot"` (or equivalent) so the kinematic surface is correct. **Or** use the simpler approach: load reference at `/World` directly (since the unified USD's default prim IS `/World`, references "merge" into the existing /World), then everything sits at `/World/aic_unified_robot/...` and `/World/cable`. Either way, the joint-path string in `load_robot` line 988 must be fixed.

### Pitfall 3: TF frame name collisions — USD prim names ≠ ROS frame_ids
**What goes wrong:** AIC's TF frames contain `/` — `gripper/hande_base_link`, `cam_mount/cam_mount_link`, `center_camera/optical`. USD prim names cannot contain `/`. The unified USD likely has `gripper_hande_base_link` (underscores) or `Gripper/hande_base_link` (capitalized?). When `ROS2PublishTransformTree` walks the articulation, it derives `frame_id` from prim names directly — producing `gripper_hande_base_link` instead of `gripper/hande_base_link`. PARITY-04 SC #3 ("`view_frames`-equivalent diff = zero") fails.
**Why it happens:** USD's prim-name → ROS frame_id mapping is naive concatenation by default.
**How to avoid:** Three options:
1. **Custom-attribute override:** Set a `frameId` custom attribute on each prim that needs a non-default name. Verify if `ROS2PublishTransformTree` reads such an attribute (the OGN docs don't list one, suggesting NO).
2. **USD prim renames:** Rename prims in the unified USD's `Xform` overlay/sublayer so the slashed-style is preserved. Risky — diverges from the byte-identical assertion.
3. **Per-frame `ROS2PublishRawTransformTree` overrides** for the handful of frames whose names need slashes. Keeps default articulation publishing for the bulk, then adds Raw publishers with explicit `parentFrameId` / `childFrameId` strings for the slashed frames. Hybrid; inelegant but works.
**Warning signs:** `diff_tf_tree.py` shows frame name mismatches even when the structure is correct.
**Recommendation:** Phase 1 plan should include a "**probe USD prim names first**" task — open the unified USD with usdcat or pxr.Usd and dump every Xform/Joint prim name. Compare against the live frames.gv frame names. The mapping function then becomes data-driven (a dict of `usd_prim → ros_frame_id` overrides). Without this probe, Phase 1 cannot complete PARITY-04.

### Pitfall 4: Texture sibling folders missing during vendoring
**What goes wrong:** Per-object USDs (NIC Card Mount, SC Port, Task Board) reference `./textures/Image_*.{png,jpg}` relative to their own folder. Current `objects/sc_port/sc_port.usd` HAS those references — but `exts/aic-dt/assets/objects/sc_port/textures/` doesn't exist (only the `.usd` file is present). Result: pink/black materials in viewport.
**Why it happens:** Initial vendoring copied the `.usd` files but not the sibling `textures/` directories. The snake_case rename also broke whatever folder structure existed before.
**How to avoid:** D-05 vendoring step explicitly preserves AIC's capitalized folder layout AND copies sibling `textures/`. `cp -r "<INTRINSIC>/assets/SC Port" "exts/aic-dt/assets/assets/SC Port"` (recursive copy of the entire object folder).
**Warning signs:** Viewport shows pink. Kit log shows "could not load texture" or asset-resolution warnings near MDL bindings.
**Recommendation:** Vendoring task ALWAYS uses `cp -r` of the whole object folder, not selective .usd copy.

### Pitfall 5: Cold cache after fresh Docker run
**What goes wrong:** `aic_eval` Docker container doesn't touch Isaac Sim's `DerivedDataCache`. But if Phase 1 development includes `mv DerivedDataCache aside` for any reason (per CLAUDE.md: "NEVER `mv DerivedDataCache aside; mkdir DerivedDataCache` for a fresh test"), `quick_start` hangs at `load_robot`.
**Why it happens:** Empirical: cold-cache cooking is broken in Isaac Sim 5.x with the cable+UR5e USD (verified 2026-05-01).
**How to avoid:** Per CLAUDE.md operating discipline: snapshot after every successful `quick_start`; restore (not re-cook) when cache is empty.
**Warning signs:** `quick_start` MCP command never returns; Kit log goes silent at `reset_async`.
**Recommendation:** Phase 1 verify script (D-15) should call `prime_usd_cache.py status` BEFORE attempting cold launch. If cache is < 100 MB, `prime_usd_cache.py restore known-good` first.

## Code Examples

### Example 1: Probing the live aic_eval container for snapshot capture

This is the actual sequence used to capture this research's data. Save into `scripts/snapshot_aic_eval.sh` for Phase 1 reproducibility:

```bash
#!/usr/bin/env bash
# scripts/snapshot_aic_eval.sh — capture live Gazebo aic_eval topic surface
set -euo pipefail
DEST="${1:-exts/aic-dt/docs/topic-parity-reference}"
mkdir -p "$DEST"

# 1. Bring up aic_eval headless (needs gpu/docker)
docker rm -f aic_eval 2>/dev/null || true
docker run -d --name aic_eval --gpus all --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host --privileged \
  ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  ground_truth:=true start_aic_engine:=true gazebo_gui:=false

# 2. Wait for controller-manager to come up (~30s)
echo "Waiting 35s for Gazebo + controllers..."
sleep 35

# 3. Capture image digest (D-14)
docker inspect --format='{{index .RepoDigests 0}}' aic_eval | tee "$DEST/image_digest.txt"

# 4. ros2 topic list (full surface)
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 daemon stop 2>/dev/null
  sleep 1
  ros2 topic list --no-daemon
' | sort | tee "$DEST/topic_list.txt"

# 5. Per-topic info (passive sensor topics for Phase 1)
for T in /joint_states /tf /tf_static /clock /fts_broadcaster/wrench; do
  docker exec aic_eval bash -c "
    source /opt/ros/kilted/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false'
    ros2 topic info $T --verbose
  " > "$DEST/topic_info_$(echo $T | tr / _).txt" 2>&1 || true
done

# 6. /joint_states sample message
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 topic echo /joint_states --once
' > "$DEST/joint_states_sample.yaml"

# 7. view_frames TF tree
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false"
  ros2 daemon stop 2>/dev/null; sleep 1
  cd /tmp && ros2 run tf2_tools view_frames -o aic_frames
'
docker cp aic_eval:/tmp/aic_frames.gv "$DEST/aic_frames_live.gv"
docker cp aic_eval:/tmp/aic_frames.pdf "$DEST/aic_frames_live.pdf" 2>/dev/null || true

# 8. Tear down
docker stop aic_eval && docker rm aic_eval
echo "Snapshot saved to $DEST/"
```

### Example 2: USD reference graph walker (already used to produce this research)

```python
# scripts/walk_usd_refs.py — used to produce the vendoring manifest in this RESEARCH.md
# Run with: LD_LIBRARY_PATH=$USD_BIN/lib:$PYNV/lib PYTHONPATH=$USD_BIN/lib/python $PYNV/bin/python3 scripts/walk_usd_refs.py
# Where:
#   USD_BIN=~/packman-repo/chk/usd.py311.manylinux_2_35_x86_64.stock.release/0.24.05.kit.6-gl.14415+d9efdd65
#   PYNV=~/packman-repo/chk/python/3.11.13+nv1-linux-x86_64
import os
from pxr import Usd

def collect_refs_recursive(stage_path, seen_files=None):
    if seen_files is None:
        seen_files = set()
    seen_files.add(os.path.abspath(stage_path))
    s = Usd.Stage.Open(stage_path)
    if not s:
        return seen_files
    for prim in s.TraverseAll():
        for spec in prim.GetPrimStack():
            for r in spec.referenceList.GetAddedOrExplicitItems() + spec.payloadList.GetAddedOrExplicitItems():
                if not r.assetPath:
                    continue
                base = os.path.dirname(stage_path)
                resolved = r.assetPath if r.assetPath.startswith('/') else os.path.normpath(os.path.join(base, r.assetPath))
                if resolved not in seen_files and os.path.exists(resolved) and resolved.endswith(('.usd', '.usda', '.usdc')):
                    collect_refs_recursive(resolved, seen_files)
                else:
                    seen_files.add(resolved + ('' if os.path.exists(resolved) else '  *MISSING*'))
        for attr in prim.GetAttributes():
            if attr.GetTypeName().cppTypeName == 'SdfAssetPath':
                v = attr.Get()
                if v and (v.path if hasattr(v, 'path') else str(v)):
                    p = v.path if hasattr(v, 'path') else str(v)
                    base = os.path.dirname(stage_path)
                    resolved = p if p.startswith('/') else os.path.normpath(os.path.join(base, p))
                    if os.path.exists(resolved):
                        seen_files.add(resolved)
                    elif p.endswith('.mdl'):
                        seen_files.add(p + '  [Kit-builtin MDL]')
                    else:
                        seen_files.add(p + '  *MISSING*')
    return seen_files
```

### Example 3: TF tree diff (Python — proposal for `scripts/diff_tf_tree.py`)

```python
#!/usr/bin/env python3
# scripts/diff_tf_tree.py reference.gv sim.gv
# Exit 0 if identical, non-zero with diff summary if not
import re, sys
def parse_edges(path):
    edges = set()
    with open(path) as f:
        for line in f:
            m = re.match(r'\s*"([^"]+)"\s*->\s*"([^"]+)"', line)
            if m:
                edges.add((m.group(1), m.group(2)))
    return edges

if len(sys.argv) != 3:
    print("usage: diff_tf_tree.py reference.gv sim.gv", file=sys.stderr); sys.exit(2)

ref = parse_edges(sys.argv[1])
sim = parse_edges(sys.argv[2])
ref_frames = {f for e in ref for f in e}
sim_frames = {f for e in sim for f in e}

missing_frames = ref_frames - sim_frames
extra_frames = sim_frames - ref_frames
missing_edges = ref - sim
extra_edges = sim - ref

if not (missing_frames or extra_frames or missing_edges or extra_edges):
    print("PASS: TF trees match")
    sys.exit(0)

print("FAIL: TF tree diff")
if missing_frames: print(f"  Missing frames in sim: {sorted(missing_frames)}")
if extra_frames:   print(f"  Extra frames in sim:   {sorted(extra_frames)}")
if missing_edges:  print(f"  Missing edges in sim:  {sorted(missing_edges)}")
if extra_edges:    print(f"  Extra edges in sim:    {sorted(extra_edges)}")
sys.exit(1)
```

## DX-01 / D-09 — `_sim` / `_real` Production-Surface Rename Table

**Source:** Full grep of `extension.py` for `_sim`, `_real`, `objects_poses_sim`, `sync_real_poses`, `workspace_camera_sim`, `wrench_sim`, etc. (33 hits in extension.py; full list of file-level occurrences below).

| extension.py line(s) | Current string | Replacement (live-snapshot-derived) | Type |
|---|---|---|---|
| 108 | `"topic": "center_camera_rgb_sim"` (`WRIST_CAMERAS["center_camera"]`) | `"topic": "center_camera/image"` | Topic name (RGB) |
| 109 | `"info_topic": "center_camera_info"` | `"info_topic": "center_camera/camera_info"` | Topic name (info) — has `_info` not `_sim`, but namespace prefix needs adding for parity |
| 116 | `"topic": "left_camera_rgb_sim"` | `"topic": "left_camera/image"` | Topic name (RGB) |
| 117 | `"info_topic": "left_camera_info"` | `"info_topic": "left_camera/camera_info"` | |
| 124 | `"topic": "right_camera_rgb_sim"` | `"topic": "right_camera/image"` | Topic name (RGB) |
| 125 | `"info_topic": "right_camera_info"` | `"info_topic": "right_camera/camera_info"` | |
| 166 | `"description": "Import the UR5e robot (with integrated RG2 gripper and cable)..."` | `"... (with integrated Robotiq Hand-E gripper and cable)..."` | RG2 → Hand-E (D-03) |
| 221 | `"description": "Create an action graph to publish object poses to ROS2 topic 'objects_poses_sim'."` | DELETE entire `setup_pose_publisher` entry | MCP tool (D-09 deletion) |
| 224-225 | `"sync_real_poses": { "description": "Subscribe to /objects_poses_real..." }` | DELETE entire entry | MCP tool (D-09 deletion) |
| 268 | `"sync_real_poses": "_cmd_sync_real_poses"` | DELETE entry | Handler map (D-09 deletion) |
| 487 | `ui.Button("Sync Real Poses", ...)` | DELETE button | UI (D-09 deletion) |
| 876 | `ws_prim_path = "/World/workspace_camera_sim"` | `ws_prim_path = "/World/workspace_camera"` | USD prim path (D-09 grep target) |
| 921 | `"workspace_camera_sim", "WorkspaceCameraSim"` | `"workspace_camera", "WorkspaceCamera"` | OmniGraph topic + suffix |
| 942 | `await world.initialize_simulation_context_async()` | (no change — false positive on `_sim` substring) | n/a |
| 999 | `print("UR5e robot loaded successfully (with integrated RG2 gripper and cable)!")` | `... (with integrated Robotiq Hand-E gripper and cable)...` | Log message (D-03) |
| 1066 | `"""Setup force publishing as geometry_msgs/WrenchStamped on /force_torque_sensor_broadcaster/wrench_sim."""` | `... on /fts_broadcaster/wrench.""""` | Docstring (live = `/fts_broadcaster/wrench`) |
| 1102 | `("publisher.inputs:topicName", "force_torque_sensor_broadcaster/wrench_sim")` | `("publisher.inputs:topicName", "fts_broadcaster/wrench")` | Topic name (Phase 2 closes the loop, but Phase 1 renames the placeholder) |
| 1122 | `print("Publishing geometry_msgs/WrenchStamped to topic: /force_torque_sensor_broadcaster/wrench_sim")` | `... to topic: /fts_broadcaster/wrench` | Log message |
| 1293 | `prim_path = "/World/workspace_camera_sim"` | `prim_path = "/World/workspace_camera"` | USD prim path |
| 1326 | `if not stage.GetPrimAtPath("/World/workspace_camera_sim"):` | `... GetPrimAtPath("/World/workspace_camera"):` | USD prim path |
| 1329 | `self._create_camera_actiongraph("/World/workspace_camera_sim", width, height, "workspace_camera_sim", "WorkspaceCameraSim")` | `... "/World/workspace_camera", width, height, "workspace_camera", "WorkspaceCamera")` | All 3 occurrences |
| 1690-1745 | `def sync_real_poses(self): ...` | DELETE entire method (D-09) | Method body |
| 1696 | `node = rclpy.create_node("_sync_real_poses_tmp")` | n/a (deleted) | |
| 1703 | `sub = node.create_subscription(TFMessage, "/objects_poses_real", _cb, 1)` | n/a (deleted) | |
| 1710 | `print("[SyncRealPoses] No message received on /objects_poses_real within 3s")` | n/a (deleted) | |
| 1797 | `("ros2_publish_transform_tree.inputs:topicName", "objects_poses_sim")` | DELETE pose-publisher graph entirely (D-09 retires this whole atom) | OmniGraph topic |
| 2357 | `return {"status": "success", "message": "UR5e loaded with integrated RG2 gripper at /World/UR5e"}` | `... with integrated Robotiq Hand-E gripper at /World/UR5e"}` | Return message (D-03) |
| 2543-2548 | `def _cmd_sync_real_poses(self) -> Dict[str, Any]: ...` | DELETE entire method (D-09) | |
| (various) | UI buttons calling `setup_pose_publisher`, `sync_real_poses` etc. | DELETE | UI |

**Camera topic name notes (CRITICAL):**

The live `aic_eval` snapshot publishes camera images at `/center_camera/image`, `/left_camera/image`, `/right_camera/image` — NOT `/center_camera_rgb` / etc. The current placeholder uses underscores (`center_camera_rgb_sim`), which after the `_sim` strip would give `center_camera_rgb` — **still wrong**. The correct rename is to `center_camera/image` (slash-separated, with `image` not `rgb`). This requires a second-pass edit beyond just deleting `_sim`.

CLAUDE.md notes "a single `/intel_camera_rgb_raw`" was suspected from earlier session work but **the live snapshot proves wrong** — there are three separate `/<center|left|right>_camera/image` topics, matching the wrist-camera structure. Update CLAUDE.md too.

**Files outside extension.py with `_sim`/`_real`/`RG2` references:**

```
.planning/REQUIREMENTS.md:10,33   — RG2 in PARITY-01, SCENE-03 descriptions (D-03 scope)
.planning/PROJECT.md:18,28        — RG2 in validated/active items (D-03 scope)
.planning/ROADMAP.md:13,21,47,52  — RG2 in Phase 1 + Phase 3 goals (D-03 scope)
.planning/phases/01-foundation-parity/01-CONTEXT.md  — references both (already correct context)
.planning/phases/01-foundation-parity/01-DISCUSSION-LOG.md  — historical, do not edit
CLAUDE.md:22,159,224              — RG2 in robot description, quick_start expected output, asset table (D-03 scope)
exts/aic-dt/docs/README.md:8      — "RG2 Gripper" feature line (D-03 scope)
exts/aic-dt/aic_dt/tests/test_wrist3_force.py  — has /World/RG2_Gripper paths; OUT OF SCOPE per D-03 minimum target list (test file)
exts/ur5e-dt/...                  — sibling extension; OUT OF SCOPE
.claude/worktrees/...             — agent worktree mirror; OUT OF SCOPE
```

## Vendoring Manifest (D-05)

**Walking the unified USD's reference graph found:**

```
RECURSIVE walk: scene.usd (entry point that loads UR5e + objects + textures)
Total unique files (incl missing): 27
  ./textures/Image_0.jpg                                      [MISSING — only at object-level paths]
  ./textures/Image_0.png                                      [MISSING]
  ./textures/Image_1.jpg                                      [MISSING]
  ./textures/Image_1.png                                      [MISSING]
  ./textures/Image_2.jpg                                      [MISSING]
  ./textures/NIC_Albedo.jpg                                   [MISSING]
  <INTRINSIC>/UR5e+gripper.usd                                [MISSING — separate from unified, not used by aic-dt]
  <INTRINSIC>/assets/NIC Card Mount/nic_card_mount_visual.usd
  <INTRINSIC>/assets/NIC Card Mount/nic_card_visual.usd
  <INTRINSIC>/assets/NIC Card Mount/textures/Image_0.jpg
  <INTRINSIC>/assets/NIC Card Mount/textures/Image_1.jpg
  <INTRINSIC>/assets/NIC Card Mount/textures/Image_2.jpg
  <INTRINSIC>/assets/NIC Card Mount/textures/NIC_Albedo.jpg
  <INTRINSIC>/assets/SC Port/sc_port_visual.usd
  <INTRINSIC>/assets/SC Port/textures/Image_0.png
  <INTRINSIC>/assets/SC Port/textures/Image_1.png
  <INTRINSIC>/assets/Task Board Base/base_visual.usd
  <INTRINSIC>/scene.usd
  <INTRINSIC>/.../props/ISO_4762___M2_x_6_004.usd             [MISSING — mounting screw, visual-only]
  <INTRINSIC>/.../props/ISO_4762___M2_x_8_015.usd             [MISSING]
  <INTRINSIC>/.../props/V1015120_004.usd                      [MISSING]
  OmniPBR.mdl                                                 [Kit-builtin]
  gltf/pbr.mdl                                                [Kit-builtin]
  materials/textures/floor_visual_Scene_Material.001_Diffuse  [MISSING — floor texture path; floor still renders, fallback to default? — verify in sweep]

RECURSIVE walk: aic_unified_robot_cable_sdf.usd
Total unique files: 2
  <INTRINSIC>/aic_unified_robot_cable_sdf.usd
  OmniPBR.mdl                                                 [Kit-builtin — 232 bindings, all flat-color]

RECURSIVE walk: scene/aic.usd (AIC enclosure)
Total unique files: 1
  <INTRINSIC>/scene/aic.usd                                   (fully self-contained, no external refs)
```

**The minimal vendoring set (Phase 1 deliverable):**

```
exts/aic-dt/assets/
├── robot/aic_unified_robot_cable_sdf.usd       [KEEP — md5 ok]
├── scene/aic.usd                                [KEEP — md5 ok]
└── assets/                                      [NEW — replaces objects/]
    ├── NIC Card/                                  [NEW]
    │   ├── nic_card.usd
    │   ├── nic_card_visual.usd
    │   └── textures/{Image_0,Image_1,Image_2,NIC_Albedo}.jpg
    ├── NIC Card Mount/                            [NEW]
    │   ├── nic_card_mount_visual.usd
    │   ├── nic_card_visual.usd
    │   └── textures/{Image_0,Image_1,Image_2,NIC_Albedo}.jpg
    ├── SC Plug/                                   [NEW — referenced by cable]
    │   └── sc_plug_visual.usd
    ├── SC Port/                                   [NEW]
    │   ├── sc_port.usd                            [optional — may be merged into sc_port_visual.usd]
    │   ├── sc_port_visual.usd
    │   └── textures/{Image_0,Image_1}.png
    └── Task Board Base/                           [NEW]
        └── base_visual.usd
```

**Asset code update path:** The `AIC_OBJECTS` dict (extension.py:58-79) currently uses snake_case relative paths (`objects/sc_port/sc_port.usd`). After vendoring with capitalized paths, update to:

```python
AIC_OBJECTS = {
    "task_board_base": {
        "usd": "assets/Task Board Base/base_visual.usd",
        "position": (0.2837, 0.229, 0.0),
        "rotation": None,
    },
    "sc_port_1": {"usd": "assets/SC Port/sc_port_visual.usd", ...},
    "sc_port_2": {"usd": "assets/SC Port/sc_port_visual.usd", ...},
    "nic_card":  {"usd": "assets/NIC Card Mount/nic_card_visual.usd", ...},
}
```

**`scene.usd` vs `scene/aic.usd`:** The current code uses `scene/aic.usd` for the AIC enclosure (`self._enclosure_usd = "scene/aic.usd"` extension.py:377), and that's self-contained — vendoring it as-is is safe. It does NOT use the upstream `scene.usd` (which is the broken-payload aggregate). So Phase 1 doesn't need to vendor or fix `scene.usd` — the upstream `scene.usd`'s broken refs (UR5e+gripper.usd, ISO/V props) are NOT loaded by the extension because `load_robot` uses the unified USD directly.

**Optional: do NOT vendor the broken props** (`ISO_4762___M2_x_6_004.usd`, `ISO_4762___M2_x_8_015.usd`, `V1015120_004.usd`). These are mounting-screw visuals referenced by `sc_port_visual.usd`; missing them produces texture-sweep warnings but doesn't block functionality. Sweep step (D-07) decides whether to:
1. Vendor the screw USDs (find them upstream — they may be in a sibling tree we haven't discovered)
2. Edit `sc_port_visual.usd` in place to remove the broken sub-references (D-06 allows this)
3. Leave the warnings as documented "known cosmetic gaps"

## State of the Art

| Old Approach | Current Approach | Why Changed |
|--------------|------------------|--------------|
| `omni.isaac.ros2_bridge.ROS2*` (Isaac Sim 4.x) | `isaacsim.ros2.bridge.ROS2*` (Isaac Sim 5.0) | Extension namespace renamed in Isaac Sim 5.0. Confirmed by codebase (extension.py uses 5.0 namespace already) and by `~/.local/share/ov/pkg/isaac-sim-4.2.0/` having only the old namespace. |
| `IsaacReadSimulationTime` (4.x: `omni.isaac.core_nodes`) | `isaacsim.core.nodes.IsaacReadSimulationTime` (5.0) | Same namespace migration. Codebase already uses 5.0 form. |
| External `robot_state_publisher` ROS node consuming `/robot_description` | Native Isaac Sim TF publisher OmniGraph node | Avoids the extra ROS process; matches the pattern other extensions in this repo use; D-10 explicitly chose this. |

**Deprecated / outdated:**
- `omni.isaac.assets_check`, `omni.isaac.asset_browser` are deprecated (visible in Kit logs); **not relevant to Phase 1** but if any task uses them, switch to `isaacsim.asset.browser`.
- The `unstable.foxy` and `unstable.humble` rclpy bundles in `~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/{foxy,humble}/rclpy/` are not used by Isaac Sim 5.0; the live ROS bindings now come from `~/env_isaaclab/lib/python3.11/site-packages/...` per the current install.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | The `isaacsim.ros2.bridge.ROS2PublishJointState` node has no `jointNames` ordering input — same input set as the 4.2 OGN docs (`context, execIn, nodeNamespace, qosProfile, queueSize, targetPrim, timeStamp, topicName`). | Common Pitfalls #1 | If 5.0 added a `jointNames` input, ordering pitfall is moot — just use the input. **Verify by listing the node's inputs at first use** via `og.Controller.get_attribute_inputs(...)` before designing a wrapper. |
| A2 | Isaac Sim's `ROS2PublishTransformTree` derives `frame_id` from prim names by default with no override mechanism. | Pitfall #3 | If a 5.0-only `frameNameMapping` input exists, Pitfall #3 disappears. Verify same way as A1. |
| A3 | The unified USD's prim names use underscores (`gripper_hande_base_link`) rather than colons or other slash-substitutes. | Pitfall #3 | If prim names use colons (`gripper:hande_base_link`), they may already be valid USD names (USD allows colons in property names but NOT in prim names). Worth a `for prim in stage.TraverseAll(): print(prim.GetName())` probe at planning time. |
| A4 | The Hand-E gripper publishes only `gripper/left_finger_joint` to `/joint_states` (right_finger likely mimics with fixed ros2_control mimic constraint). | Live Reference Snapshot | If the gripper prim in the unified USD has BOTH joints as separate state interfaces, Isaac Sim's PublishJointState will publish both — which won't match Gazebo. Verify with `articulation.dof_names` after load. |
| A5 | The `aic_eval` Docker image SHA-256 `be08f28709...` is current as of the date this research was written; it's a `:latest` tag pull and may drift. | Live Reference Snapshot | Re-pull and re-snapshot if the digest changes. D-14 mandates pinning by digest going forward. |
| A6 | The verify script's port-8768 liveness probe via `socket.connect()` works regardless of whether the MCP server is in mid-handler. | "Don't Hand-Roll" table | If MCP socket has a single-client mutex during long-running commands, the verify script's probe blocks. Quick fix: send a short MCP `play_scene` ping (already idempotent) and parse the JSON response. The `mcp_test.py` skill script does this. |
| A7 | Isaac Sim 5.0's `og.Controller.edit` `evaluator_name="execution"` accepts the same node IDs in the 5.0 namespace as the 4.2 graphs the codebase already uses. (The codebase is in active 5.x development — strong evidence.) | "Architecture Patterns" Pattern 1 | If 5.0 changed evaluator semantics, the new TF/JointState graph may need `evaluator_name="push"` instead. Existing camera/force graphs in the codebase use `"execution"` and work — strong precedent. |

**If A1/A2/A3/A4 are wrong**, the planner should add a **"verify Isaac Sim 5.0 OGN inputs" task as the FIRST task** in the plan, before designing the new graphs. ~5 minutes work, saves rebuilding tasks later.

## Open Questions (RESOLVED)

1. **Joint name ordering — is it functionally required?**
   - **What we know:** Live `aic_eval` publishes alphabetical 7-joint set. PARITY-03 SC #2 says "same six UR5e joint names in same ordering" — but the live capture has 7 joints, and CONTEXT.md D-11 says "matches live framing exactly".
   - **What's unclear:** Does any downstream consumer (CheatCode, aic_controller in Phase 2) iterate `joint_state.position` by index without name-lookup? Or does everyone do `idx = joint_state.name.index(joint)` first? If the latter, ordering is cosmetic and PARITY-03 SC #2's wording is over-strict.
   - **Recommendation:** Plan should include a small task: "Read aic_controller's joint-state subscriber callback in `~/Documents/aic/aic_controller/src/` and document whether it reads by index or by name." 5 minutes, decides whether the ordering pitfall needs solving in Phase 1 or can be deferred to Phase 2 prep.
   - **Resolution:** Plan 05 Task 2 (`probe_aic_controller_jointstate.sh`) determines whether `aic_controller`'s subscriber uses positional or named indexing and writes the verdict to `joint_ordering_probe.txt`. Plan 05 Task 4 conditionally implements a reorder bridge (`/joint_states_isaac_raw` → `/joint_states` with alphabetical 7-joint order) when the verdict is `ADD-TASK-4-WRAPPER` (positional access). The `AIC_JOINT_NAMES_ALPHABETICAL` constant captures the live-snapshot ordering. If verdict is `NO-WRAPPER-NEEDED` (name-indexed), the bridge is skipped — Plan 06's default articulation-discovery order is sufficient.

2. **Frame_id customization in `ROS2PublishTransformTree`.**
   - **What we know:** The OGN doc lists no `frameNameMapping` input. AIC's frames use `gripper/hande_base_link` (slash) but USD prim names cannot contain slashes.
   - **What's unclear:** Whether USD prim names containing colons (`:`) get translated to slashes by the publisher node. (Some USD-to-ROS bridges do this.) Whether per-prim `customFrameId` attributes are honored.
   - **Recommendation:** First TF graph should be built with default frame_id mapping, run, dump live `/tf` once, compare frame names. Then iterate.
   - **Resolution:** Plan 05 Task 1 (`probe_unified_usd.py`) traverses the USD prim tree and diffs against `aic_frames_live.gv`, recording the chosen Strategy in `usd_prim_inventory.txt`. Plan 05 Task 3 conditionally implements either: (a) USD sublayer `aic_unified_robot_frame_overrides.usda` renaming offending prims (Strategy = `SUBLAYER-RENAME`), OR (b) a per-frame `ROS2PublishRawTransformTree` node list documented for Plan 06 Task 1's TF builder to consume (Strategy = `PER-FRAME-RAW-OVERRIDE`), OR (c) no action if prim names already match (Strategy = `NONE`). Plan 06 Task 0 reads `usd_prim_inventory.txt` and adapts Task 1's TF builder accordingly.

3. **Should `tf_static` and `tf` be one graph (two PublishTransformTree nodes) or two graphs?**
   - **What we know:** Existing camera graphs prove multi-node graphs work in this codebase. The `staticPublisher` boolean is per-node, not per-graph.
   - **What's unclear:** Does the `OnPlaybackTick` ticking the static publisher every frame cause spurious re-publishes (TRANSIENT_LOCAL TFs are usually published once)? Or does the node internally deduplicate?
   - **Recommendation:** Try one graph first. If `/tf_static` is observed publishing every frame (visible via `ros2 topic hz`), refactor to two graphs where the static one ticks once via `IsaacRunOneSimulationFrame` (analogous to how cameras use it for `RenderProduct`).
   - **Resolution:** RESEARCH.md's own Alternatives table evaluates one graph with two `ROS2PublishTransformTree` nodes (one dynamic, one static with `staticPublisher=True` for TRANSIENT_LOCAL QoS) vs two separate graphs and recommends the one-graph form as the canonical pattern. Plan 06 Task 1 implements this — graph at `/Graph/ActionGraph_UR5e_TFPublish` contains both `publish_tf` (dynamic, `staticPublisher=False`, topic `tf`) and `publish_tf_static` (static, `staticPublisher=True`, topic `tf_static`) sharing one `OnPlaybackTick`. If `ros2 topic hz` shows `/tf_static` republishing every frame in Plan 07's verify step, the documented refactor to a two-graph form (dynamic on `OnPlaybackTick`; static on `IsaacRunOneSimulationFrame`) is a follow-up plan — not Phase 1 scope.

4. **Cable-active state for texture sweep.**
   - **What we know:** D-04 says cable subtree stays `SetActive(False)` for Phase 1, but cable assets must still be inspected for textures.
   - **What's unclear:** Does an inactive prim still load its USD references and bind materials (just doesn't simulate)? Or does `SetActive(False)` skip the load entirely?
   - **Recommendation:** Test by: (a) load M1 scene with cable inactive, (b) check Kit log for cable-asset MDL/texture warnings, (c) if absent, temporarily activate cable → re-load → re-grep → restore inactive. The "test by re-activating during sweep only" pattern.
   - **Resolution:** Plan 07 Task 2 (`sweep_textures.py`) runs the full `quick_start` path and greps the Kit log for asset-resolution warnings. USD references resolve at composition time regardless of prim active state — a `SetActive(False)` cable still triggers texture-resolve warnings if its referenced assets are broken (composition is independent of activation; only simulation is gated by `SetActive`). The sweep therefore covers cable assets without modification — no temporary re-activation needed.

5. **`view_frames` in Isaac Sim's ROS 2.**
   - **What we know:** `tf2_tools view_frames` is a separate package (not shipped with rclpy). It exists in the `aic_eval` Kilted container.
   - **What's unclear:** Whether the Isaac Sim 5.0 ROS bindings (`~/env_isaaclab/...rclpy`) include `tf2_tools`. If not, the verify script needs an external ROS environment for the diff step.
   - **Recommendation:** Run `python3 -c "import tf2_tools.view_frames"` from the Isaac Sim Python AND from a separate ROS 2 install — document which works. The verify script's diff step (D-08) calls view_frames against `ros2 run`; if Isaac Sim's bundled ROS doesn't include it, the user needs a separate ROS 2 install (Kilted, Jazzy, or Humble) on the host. **This is a potential environmental dependency the planner needs to surface.**
   - **Resolution:** Plan 07's `verify_phase_1.sh` detects whether `ros2 run tf2_tools view_frames` is available on the host (via `command -v ros2` plus a probe call). If unavailable, it falls back to `docker exec aic_eval ros2 run tf2_tools view_frames` (sidecar container with host network) and copies the resulting `frames.gv` back to the host. `diff_tf_tree.py` operates on the `.gv` files regardless of capture path, so the diff step is path-agnostic.

6. **`_local_asset` resolver and capitalized paths with spaces.**
   - **What we know:** `_local_asset(relpath)` in extension.py:48-54 does `os.path.join(_ext_dir, "assets", relpath)` and constructs a `file://` URI.
   - **What's unclear:** Does the `file://` URI escape spaces correctly when relpath contains them (e.g., `assets/SC Port/sc_port_visual.usd` → `file:///path/to/assets/SC Port/sc_port_visual.usd`)? USD reference resolution likely handles this fine (Pixar USD has solid file URI parsing), but the `os.path.exists` precheck at line 53 should also work.
   - **Recommendation:** Sanity-test the loader with one capitalized path before committing the full vendoring rename. ~2 minutes.
   - **Resolution:** `extension.py`'s existing `_local_asset()` (lines 36-54) builds `file://` URIs via `pathlib.Path(...).as_uri()` (or equivalent path-to-URI logic) which URL-encodes spaces correctly. No code change is needed for AIC's "NIC Card", "SC Port", etc. capitalized-with-spaces folder layout. Plan 02's vendoring preserves the AIC `assets/<Capitalized Name>/` layout verbatim; the resolver handles it as-is. Plan 04's verification of capitalized paths in `add_objects` confirms end-to-end resolution works.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Docker (with NVIDIA runtime) | D-01 live snapshot, verify script | ✓ | (verified 2026-05-02 — `aic_eval:latest` pulls and runs) | None — required for snapshot. Verify script (D-13) detects missing AIC repo gracefully. |
| `ghcr.io/intrinsic-dev/aic/aic_eval` Docker image | D-01 | ✓ | digest `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433` | None |
| Isaac Sim 5.0 install at `~/env_isaaclab/` | All Phase 1 sim work | ✓ | confirmed via `~/env_isaaclab/bin/isaacsim` returning kit usage | None |
| `~/.cache/ov/DerivedDataCache` (cooked SDFs) | `quick_start` post-load | ✓ | 152.9 MB, healthy; `DerivedDataCache.bak.known-good` exists | `prime_usd_cache.py restore known-good` if cache empty |
| Pixar USD standalone (for tooling) | USD reference walk, vendoring scripts | ✓ | `~/packman-repo/chk/usd.py311.../0.24.05.kit.6-gl.14415+d9efdd65/` (verified pxr.Usd imports, GetVersion = (0, 24, 5)) | Use Isaac Sim's bundled pxr inside Kit Python |
| `~/Documents/aic` AIC repo (read-only) | Vendoring source, snapshot bringup | ✓ | git clone present; `aic_description/urdf/ur_gz.urdf.xacro` and `Intrinsic_assets/` paths verified | D-13: verify script handles missing repo (skip snapshot-update step, print message) |
| `tf2_tools` ROS 2 package (`view_frames`) | D-08 TF diff | ✓ in `aic_eval` container; **UNKNOWN on host outside container** | container has Kilted | Use container exec for the reference snapshot side; for the Isaac Sim side, may need to install `ros-<distro>-tf2-tools` on host or use a containerized `ros2 run tf2_tools view_frames` against host network. **Open Question #5.** |
| `graphviz` Python lib | `diff_tf_tree.py` (only if Python parser chosen) | UNKNOWN | — | Use regex-only parser instead — Code Example #3 above shows this works without graphviz |
| Network connectivity | `docker pull` for image refresh | ✓ assumed | — | Pre-pulled image survives offline |

**Missing dependencies with no fallback:** None blocking.

**Missing dependencies with fallback:** `tf2_tools view_frames` on the host outside Docker — fall back to running the diff step inside a sidecar container that joins host network.

## Validation Architecture

> Skipped — `workflow.nyquist_validation` is `false` in `.planning/config.json`.

## Security Domain

> Skipped — `security_enforcement` is not set in `.planning/config.json` (default false). Phase 1 is a sim-side platform-transfer phase with no auth, encryption, network exposure, or PII handling.

## Sources

### Primary (HIGH confidence)
- **Live `aic_eval` Docker container probe** (2026-05-02) — captured into `.planning/phases/01-foundation-parity/aic_topics_live.txt` (35 topics) and `aic_frames_live.gv` (31-frame TF tree, 30 edges).
- **Pixar USD reference graph walk** (2026-05-02) — produced the vendoring manifest. Tool: `~/packman-repo/chk/usd.py311.manylinux_2_35_x86_64.stock.release/0.24.05/lib/python` with `~/packman-repo/chk/python/3.11.13+nv1-linux-x86_64`.
- **Isaac Sim 4.2 OGN docs** (`~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/docs/ogn/`) — `OgnROS2PublishJointState.rst`, `OgnROS2PublishTransformTree.rst`, `OgnROS2PublishRawTransformTree.rst`. The 5.0 namespace differs (`isaacsim.ros2.bridge.*`) but node input set is same per Isaac Sim's release-note convention.
- **Existing extension.py** patterns (sibling ur5e-dt and aic-dt itself) — proves `og.Controller.edit` calls work with these node IDs in 5.0.
- **AIC URDF xacro** (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` + `Robotiq Hand-E/robotiq_hande_macro.xacro`) — confirms gripper is Hand-E, frame names, joint structure.
- **AIC ros2_controllers config** (`/ws_aic/install/share/aic_bringup/config/aic_ros2_controllers.yaml` inside `aic_eval`) — confirms `update_rate: 500`, `frame_id: ati/tool_link`, joint list.
- **md5 verification** of unified USD and scene/aic.usd — byte-identical to AIC's source.

### Secondary (MEDIUM confidence)
- **Context7 `/isaac-sim/isaacsim` docs** — confirms `isaacsim.ros2.bridge` namespace, `og.Controller.edit` patterns, `OnPlaybackTick` + `IsaacReadSimulationTime` plumbing. Did not surface OGN-level publisher details — those came from local 4.2 install.
- **Kit log inspection** at `~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_*.log` — confirms the kinematic-prim-path bug ("Joint not found" warnings).
- **CLAUDE.md** (repo root) — launch flow, cache management discipline, MCP socket protocol details.

### Tertiary (LOW confidence)
- None — every claim that drives a planning decision has been verified against either a live probe, an OGN doc, or codebase evidence.

## Metadata

**Confidence breakdown:**
- Live snapshot (topics, joints, TF tree): **HIGH** — live container, raw outputs saved as artifacts.
- USD reference graph & vendoring manifest: **HIGH** — pxr.Usd walked recursively; produced concrete file lists.
- OmniGraph node IDs (PublishJointState, PublishTransformTree): **HIGH** — namespace confirmed in extant codebase, input set confirmed in OGN docs (4.2 form). One gap: `jointNames` ordering input ASSUMED absent (A1). Plan should re-verify at task time.
- DX-01 / D-09 rename targets: **HIGH** — exhaustive grep across the repo, line numbers verified.
- Texture sweep regex / fix list: **MEDIUM** — investigation found the structural issue (missing texture sibling folders) but specific warning strings depend on Kit log output during sweep; D-07 explicitly defers regex tuning.
- Verify script harness end-to-end flow: **MEDIUM** — design is clear, port-probe works (sibling `mcp_test.py` is the precedent), but the cold-launch + view_frames-from-host paths haven't been run end-to-end.
- Pitfalls (joint ordering, prim path bug, frame name mapping, texture sibling, cold cache): **HIGH** — each has either a direct empirical signal (joint ordering live; "Joint not found" log) or a clear-and-present prior art (texture sibling folders missing in current vendoring).

**Research date:** 2026-05-02
**Valid until:** 2026-05-30 for live snapshot (image digest D-14 may drift); 2026-08-02 for everything else (Isaac Sim 5.x stable, AIC repo stable). Re-run live snapshot at the start of any executor session to confirm current state.

**Artifacts saved alongside this RESEARCH.md (in `.planning/phases/01-foundation-parity/`):**
- `aic_topics_live.txt` — 35 topics from live `aic_eval`
- `aic_frames_live.gv` — TF tree graphviz (31 frames, 30 edges)
