# Phase 1: Foundation Parity - Pattern Map

**Mapped:** 2026-05-02
**Files analyzed:** 8 create + 9 modify (+ delete/rename)
**Analogs found:** 14 / 17 (3 docs files have no codebase analog beyond extension-internal precedent)

## File Classification

| New / Modified File | Role | Data Flow | Closest Analog | Match Quality |
|---|---|---|---|---|
| New TF + `/tf_static` action graph (inside `extension.py`) | OmniGraph builder (in-extension method) | event-driven (OnPlaybackTick → ROS2 publish) | `exts/aic-dt/aic_dt/extension.py:1782-1814` (`create_pose_publisher`) + `exts/ur5e-dt/ur5e_dt/extension.py:3094-3137` (`setup_camera_action_graph`'s `TFPublish`) | exact (same node `ROS2PublishTransformTree`, same `og.Controller.edit` shape) |
| New JointState publisher action graph (inside `extension.py`) | OmniGraph builder | event-driven | `exts/aic-dt/aic_dt/extension.py:1022-1059` (`setup_action_graph` — already wires `ROS2SubscribeJointState`; mirror it for `ROS2PublishJointState`) | role-match (same namespace `isaacsim.ros2.bridge`, same builder pattern; node ID different) |
| `setup_tf_publisher` MCP atom + `_cmd_setup_tf_publisher` + UI button | controller / handler | request-response | `exts/aic-dt/aic_dt/extension.py:169-180` (`MCP_TOOL_REGISTRY` entries) + `2362-2385` (`_cmd_setup_action_graph`, `_cmd_setup_force_publisher`, `_cmd_setup_wrist_cameras`) + `432-438` (UI buttons) | exact (literal copy-paste-rename of the atomic+clubbed shape) |
| `setup_joint_state_publisher` MCP atom + cmd + UI | controller / handler | request-response | same as above | exact |
| `extension.py` `load_robot` prim-path bug fix (line 988) | service / loader | file-I/O (USD load) | self — `extension.py:933-999` (current method) | exact (single-line edit inside the same method) |
| `extension.py` `quick_start` refactor (line 824-929) | controller (orchestrator) | sequential pipeline | self — current method body | exact (reorders existing calls + adds two new atom invocations) |
| `extension.py` 33 `_sim`/`_real` string replacements | utility (string edits) | n/a | self — DX-01 / D-09 rename table in `01-RESEARCH.md` lines 731-766 (line-by-line spec) | exact (table is the spec) |
| `exts/aic-dt/scripts/verify_phase_1.sh` | test / verification harness | request-response (MCP socket + shell) | `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh` (lifecycle helper, port-detect pattern) + `~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py` (socket pattern) + `~/Documents/aic/scripts/run_cheatcode.sh` (Docker bringup pattern) | role-match (composite — no single existing bash verifier in this repo) |
| `exts/aic-dt/scripts/diff_tf_tree.py` | utility (parse+compare) | transform | none in repo; pattern is `re.findall` on `.gv` — full reference impl already inlined in `01-RESEARCH.md` lines 692-728 | self-contained (research has the implementation) |
| `exts/aic-dt/scripts/sweep_textures.py` | utility (log-grep loop) | batch | `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` glob discovery (CLAUDE.md repo root); `mcp_test.py` socket pattern for triggering the load | partial (no direct analog; pattern is `ls -t ... \| head -1` glob + grep loop) |
| `exts/aic-dt/scripts/snapshot_aic_eval.sh` | utility (Docker driver) | request-response (Docker exec) | `~/Documents/aic/scripts/run_cheatcode.sh` (Docker bringup) + reference impl already inlined in `01-RESEARCH.md` lines 575-643 | exact-external (full snapshot script written in research) |
| `exts/aic-dt/docs/topic-parity-reference.md` | doc | n/a | `exts/aic-dt/docs/README.md`, `exts/aic-dt/docs/CHANGELOG.md` (existing extension docs for tone/structure) | role-match (no parity-reference doc exists yet — first of its kind) |
| `exts/aic-dt/docs/texture-sweep.md` | doc (log artifact) | n/a | none — first sweep log | none (table format spec given in CONTEXT.md D-07) |
| `exts/aic-dt/assets/<vendoring>` | asset (one-time copy) | file-I/O | `exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd` already vendored; manifest in `01-RESEARCH.md` lines 830-868 | exact (just `cp -r` of subset; manifest is the spec) |
| Docs RG2→Hand-E corrections (`PROJECT.md`, `REQUIREMENTS.md`, `ROADMAP.md`, `CLAUDE.md`, `docs/README.md`, `docs/CHANGELOG.md`) | doc | n/a | self — string-grep + replace; line targets in `01-CONTEXT.md` D-03 + `01-RESEARCH.md` lines 776-786 | exact (line-numbered list) |

---

## Pattern Assignments

### Inline action graph: `setup_tf_publisher` (NEW — D-10) and `setup_joint_state_publisher` (NEW — D-11)

**Closest analogs (all in `exts/aic-dt/aic_dt/extension.py` itself; treat the file as its own template):**

1. **Existing pose-publisher action graph** (`create_pose_publisher`, lines 1776-1816) — already uses `isaacsim.ros2.bridge.ROS2PublishTransformTree` with `OnPlaybackTick`, `ROS2Context`, `IsaacReadSimulationTime`. Phase 1's TF graph is structurally identical; only difference is two TF nodes (one dynamic, one with `staticPublisher=True`) instead of one, and `targetPrims` points at the articulation root instead of `/World/Objects` children.
2. **Existing joint-state subscribe + clock graph** (`setup_action_graph`, lines 1001-1059) — wires `ROS2SubscribeJointState`, `ROS2PublishClock`, `OnPlaybackTick`, `ROS2Context`, `IsaacReadSimulationTime`, `IsaacArticulationController`. Phase 1's JointState publisher uses the symmetric publish node `ROS2PublishJointState` (same input set per RESEARCH §A1) and the same `articulation_controller.inputs:targetPrim`-style `targetPrim` relationship to the articulation root.
3. **ur5e-dt's camera+TF graph** (`exts/ur5e-dt/ur5e_dt/extension.py:3094-3137`) — second-source confirmation of the `ROS2PublishTransformTree` wiring inside `og.Controller.edit`. Sets `TFPublish.inputs:topicName`, `TFPublish.inputs:targetPrims`, `TFPublish.inputs:parentPrim` directly inside `SET_VALUES` (string-form), which differs from `create_pose_publisher`'s relationship-after-edit pattern. Both work; the relationship-after-edit pattern is more robust for multi-target prim lists.

**Idempotency / cleanup pattern (apply to both new graphs):**

```python
# extension.py:1086-1089 (inside setup_force_publish_action_graph)
graph_path = "/Graph/ActionGraph_UR5e_ForcePublish"
keys = og.Controller.Keys
stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath(graph_path):
    stage.RemovePrim(graph_path)
```

**Core pattern — TF graph (adapt from `create_pose_publisher`, extension.py:1782-1814):**

```python
# extension.py:1782-1814 — adapt for /tf + /tf_static (two TF publish nodes in one graph)
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.CONNECT: [
            ("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_publish_transform_tree.inputs:context"),
            ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_transform_tree.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("ros2_publish_transform_tree.inputs:topicName", "objects_poses_sim"),
            ("isaac_read_simulation_time.inputs:resetOnStop", False),
        ],
    },
)

ros2_node_path = f"{graph_path}/ros2_publish_transform_tree"
ros2_prim = stage.GetPrimAtPath(ros2_node_path)
if ros2_prim.IsValid():
    parent_rel = ros2_prim.GetRelationship("inputs:parentPrim")
    if not parent_rel:
        parent_rel = ros2_prim.CreateRelationship("inputs:parentPrim", custom=True)
    parent_rel.SetTargets([Sdf.Path("/World")])

    target_rel = ros2_prim.GetRelationship("inputs:targetPrims")
    if not target_rel:
        target_rel = ros2_prim.CreateRelationship("inputs:targetPrims", custom=True)
    target_rel.SetTargets([Sdf.Path(p) for p in body_paths])
```

Diverges for Phase 1: add a second TF publish node `ros2_publish_transform_tree_static` with `inputs:topicName="tf_static"` and `inputs:staticPublisher=True`; topicName for dynamic node becomes `"tf"` (not `"objects_poses_sim"`). The full target shape is in `01-RESEARCH.md` lines 408-475 (research already wrote it).

**Core pattern — JointState graph (adapt from `setup_action_graph`, extension.py:1022-1059):**

```python
# extension.py:1022-1059 — current code wires SUBSCRIBE; Phase 1 swaps to PUBLISH
(graph, nodes, _, _) = og.Controller.edit(
    {
        "graph_path": graph_path,
        "evaluator_name": "execution"
    },
    {
        og.Controller.Keys.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ros2_context.inputs:useDomainIDEnvVar", True),
            ("ros2_context.inputs:domain_id", 0),
            ("ros2_subscribe_joint_state.inputs:topicName", "/joint_states"),
            ("ros2_subscribe_joint_state.inputs:nodeNamespace", ""),
            # ...
            ("articulation_controller.inputs:targetPrim", self._robot_prim_path),
        ],
        og.Controller.Keys.CONNECT: [
            ("on_playback_tick.outputs:tick", "ros2_subscribe_joint_state.inputs:execIn"),
            # ... ros2_context, simulationTime fanout
        ],
    }
)
```

For Phase 1 the new node is `("ros2_publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState")` and the `targetPrim` relationship points at the articulation root — same pattern `articulation_controller.inputs:targetPrim` already uses (`self._robot_prim_path` string in `SET_VALUES`). Per RESEARCH §A1 + Pitfall #1, this node has no `jointNames` ordering input — ordering issue is resolved at consumer side, not here.

What's identical: builder shell (`og.Controller.edit`, `keys.CREATE_NODES/SET_VALUES/CONNECT`), context fanout, OnPlaybackTick exec wiring, idempotent `RemovePrim` cleanup, the integration with the rest of the extension via a `_cmd_*` handler.

What diverges: target prim for `ROS2PublishJointState` should account for the latent prim-path bug (RESEARCH Pitfall #2) — articulation root may be `/World/UR5e/aic_unified_robot/root_joint`, not `/World/UR5e`. **Fix the prim-path bug before wiring this node, or the targetPrim relationship will resolve to an empty Xform and silently publish nothing.**

---

### MCP atom + handler + UI button shape (atomic + clubbed model)

**Closest analog:** `setup_force_publisher` end-to-end (literal copy-paste-rename target).

**Registry entry** (`extension.py:173-176`):

```python
"setup_force_publisher": {
    "description": "Create the ROS2 force/torque publisher action graph for UR5e end-effector wrench.",
    "parameters": {}
},
```

**Handler map entry** (`extension.py:259`):

```python
"setup_force_publisher": "_cmd_setup_force_publisher",
```

**Handler method** (`extension.py:2371-2377`):

```python
def _cmd_setup_force_publisher(self) -> Dict[str, Any]:
    try:
        self.setup_force_publish_action_graph()
        return {"status": "success", "message": "Force publisher action graph created for UR5e"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}
```

**For async builders** (the new TF graph likely needs `async` because it depends on `load_robot` having finished — mirror `_cmd_setup_action_graph` instead, `extension.py:2362-2369`):

```python
def _cmd_setup_action_graph(self) -> Dict[str, Any]:
    try:
        from omni.kit.async_engine import run_coroutine
        run_coroutine(self.setup_action_graph())
        return {"status": "success", "message": "ROS2 action graph created for UR5e joint control"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}
```

**UI button** (`extension.py:432-433`):

```python
ui.Button("Setup Action Graph", width=200, height=35, clicked_fn=lambda: asyncio.ensure_future(self.setup_action_graph()))
ui.Button("Setup Force Publisher", width=200, height=35, clicked_fn=self.setup_force_publish_action_graph)
```

What's identical: 4-tuple shape (registry / handler-map / `_cmd_*` body / UI button) per atom. Wrap async builders with `run_coroutine` + `asyncio.ensure_future` per the existing precedent.

**Deletions for D-09** (mirror these targets — same 4 surfaces per deleted atom):

- `setup_pose_publisher` registry entry (`extension.py:220-223`), handler-map entry (`267`), `_cmd_setup_pose_publisher` method (`2534-2541`), UI button (`480`), and the body method `create_pose_publisher` (`1747-1816` — keep until Phase 3, or delete entirely; D-09 says delete).
- `sync_real_poses` registry entry (`224-227`), handler-map entry (`268`), `_cmd_sync_real_poses` method (`2543+`), `sync_real_poses` body (`1690-1745`), UI button (`487`).

---

### `extension.py` `load_robot` prim-path bug fix (RESEARCH Pitfall #2)

**Analog:** self — `extension.py:933-999` (current `load_robot`).

**Excerpt of bug location** (`extension.py:986-997`):

```python
# Configure joint drives (from Isaac Lab config)
for joint_name in self._joint_names:
    joint_path = f"{prim_path}/joints/{joint_name}"        # ← bug: actual joints live at
                                                            #   {prim_path}/aic_unified_robot/joints/<name>
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        print(f"Warning: Joint not found at {joint_path}")  # ← already in Kit log, x6
        continue
    drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive_api.GetMaxForceAttr().Set(self._ur5e_max_force)
    drive_api.GetStiffnessAttr().Set(self._ur5e_stiffness)
    drive_api.GetDampingAttr().Set(self._ur5e_damping)
```

Fix per RESEARCH: change `joint_path = f"{prim_path}/joints/{joint_name}"` to `joint_path = f"{prim_path}/aic_unified_robot/joints/{joint_name}"` — OR change the `add_reference_to_stage` `prim_path` argument so the unified USD's default prim `/World` merges into `/World` directly and joints land at `/World/aic_unified_robot/joints/<name>`. Either choice cascades into the new JointState publisher's `targetPrim` relationship.

What's identical: rest of `load_robot` (cable `SetActive(False)`, ArticulationView, drive config) is unchanged.

What diverges: this is a one-line edit; only the joint-path string changes. Plus a parallel update to `_robot_prim_path` if the second option is taken.

---

### `quick_start` refactor (D-12)

**Analog:** self — `extension.py:824-929` (current `quick_start`).

**Current sequence excerpt** (`extension.py:824-871`):

```python
async def quick_start(self):
    """Quick start: load scene, UR5e + action graph + force publisher, wrist cameras, objects, play."""
    # 1. Load scene
    await self.load_scene()
    # 2. Import UR5e (load_robot internally calls timeline.stop() at the end)
    await self.load_robot()
    # 2b. Play the scene EARLY — before any action graphs / cameras / objects exist.
    self._timeline.play()
    # 3. Setup UR5e action graph
    await self.setup_action_graph()
    # 4. Setup force publisher
    self.setup_force_publish_action_graph()
    # 5. Setup wrist cameras
    self.setup_wrist_cameras()
    # 6. Add task board objects
    self.add_objects()
    # 7. Setup pose publisher          ← DELETE per D-09
    self.create_pose_publisher()
    # 8. Create workspace camera at 640x480
    # 9. Setup workspace camera action graph
    # 10. Already playing from step 2b
```

D-12 reorders to: load scene → load robot → setup_tf_publisher (NEW) → setup_joint_state_publisher (NEW) → setup_action_graph (subscribe-side, kept) → setup wrist cameras → setup force publisher → add objects → randomize lighting → play (already playing). The `play_scene` early-play pattern at step 2b stays — its rationale (PhysX cooking + many graphs lock-contend if all ordered together) is encoded in the comment block lines 839-843 and is non-negotiable.

What's identical: every step is a method call on `self`; ordering and inserts are the only edits. The early-play and `app.next_update_async()` pumping (one between each step) stay.

What diverges: two new calls (`setup_tf_publisher`, `setup_joint_state_publisher`); one delete (`create_pose_publisher`); workspace-camera prim path renamed (`/World/workspace_camera_sim` → `/World/workspace_camera` per D-09 rename table).

---

### `_sim`/`_real` rename — 33-line edit table

**Analog:** the rename table is the spec — `01-RESEARCH.md` lines 731-766 lists each `extension.py` line, current string, replacement, and edit type.

**Pattern:** mechanical search-and-replace per row; each row is line-anchored. Examples for context:

```python
# extension.py:108  → "topic": "center_camera/image"
# extension.py:1102 → ("publisher.inputs:topicName", "fts_broadcaster/wrench")
# extension.py:876  → ws_prim_path = "/World/workspace_camera"
# extension.py:921  → ("workspace_camera", "WorkspaceCamera")
# extension.py:166  → "Import the UR5e robot (with integrated Robotiq Hand-E gripper and cable)..."
```

What's identical: the 4-surface atomic pattern (registry / handler / handler-map / UI) — for the deletions (`setup_pose_publisher`, `sync_real_poses`), all 4 surfaces of each atom go.

---

### `exts/aic-dt/scripts/verify_phase_1.sh` (NEW — D-15)

**No single existing analog in this repo. Composite of three sources:**

1. **Hybrid runtime / port-detect / cold-launch fallback** — `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh` (lines 27-46 below).
2. **MCP socket round-trip** — `~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py`.
3. **Docker bringup + `ros2 topic` shell idiom** — `~/Documents/aic/scripts/run_cheatcode.sh`, plus `01-RESEARCH.md` lines 575-643 for the snapshot script.

**Port-detect pattern from `isaacsim_launch.sh`:**

```bash
ISAACSIM_BIN="$HOME/env_isaaclab/bin/isaacsim"
EXT_FOLDER="$HOME/Documents/isaac-sim-mcp/exts"
LOG="/tmp/isaacsim.log"
PIDFILE="/tmp/isaacsim.pid"

check_socket() {
    local port="${1:-8767}"
    python3 -c "
import socket
s=socket.socket();s.settimeout(3)
try:
    s.connect(('localhost',$port))
except Exception:
    raise SystemExit(1)
finally:
    s.close()
" 2>/dev/null
    return $?
}

is_running() {
    pgrep -f "bin/isaacsim" > /dev/null 2>&1
    return $?
}
```

**Cold-launch fallback pattern from same file:**

```bash
if is_running; then
    if check_socket "$PORT"; then
        echo "  Socket: RESPONSIVE on $PORT"
    fi
    # else wait for socket
else
    nohup "$ISAACSIM_BIN" --ext-folder "$EXT_FOLDER" --enable "$EXT" > "$LOG" 2>&1 &
    echo "$!" > "$PIDFILE"
    sleep 3
    "$0" wait 120 "$PORT"
fi
```

**Per CLAUDE.md, the cold-cache fallback should prefer `launch_postload.py`** for aic-dt (the `--enable aic-dt` startup path deadlocks on cold caches). Use the helper for warm cache; fall through to `~/env_isaaclab/bin/isaacsim --exec exts/aic-dt/scripts/launch_postload.py` when the lifecycle path fails. Cache discipline (CLAUDE.md): `prime_usd_cache.py status` first; `prime_usd_cache.py restore known-good` if `< 100MB`.

**Header structure pattern from `run_cheatcode.sh:1-22`:**

```bash
#!/usr/bin/env bash
# Brief description.
#
# Usage: ./script.sh [arg]
#
# Requirements:
#   - Docker / GPU / image
#
# Note: known limitations.

set -euo pipefail
```

**MCP send-cmd loop from `mcp_test.py:18-37`:**

```python
def send_cmd(port, cmd_type, params=None):
    msg = json.dumps({"type": cmd_type, "params": params or {}})
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15)
    sock.connect(("localhost", port))
    sock.sendall(msg.encode("utf-8"))
    data = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
        try:
            json.loads(data.decode("utf-8"))
            break
        except json.JSONDecodeError:
            continue
    sock.close()
    return json.loads(data.decode("utf-8"))
```

What's identical: socket round-trip semantics, port-detect liveness check, cold-launch helper invocation.

What diverges: this script is hybrid (runs end-to-end checks: trigger `quick_start`, capture `view_frames` from Isaac Sim, invoke `diff_tf_tree.py`, grep extension.py for `_sim`/`_real` (expect 0), grep Kit log for texture warnings (expect 0)). The check sequence has no analog — only the building blocks do.

**D-13 missing-AIC-repo handling:** if `~/Documents/aic` is absent, snapshot-update step exits non-zero with a clear message but verify-only mode skips it. Pattern is a simple `if [ ! -d "$HOME/Documents/aic" ]; then ... fi` guard around the snapshot-related steps.

---

### `exts/aic-dt/scripts/diff_tf_tree.py` (NEW — D-08)

**No codebase analog.** The full reference implementation is already in `01-RESEARCH.md` lines 692-728 — copy it verbatim and place at `exts/aic-dt/scripts/diff_tf_tree.py`. The pattern is:

```python
#!/usr/bin/env python3
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
    print("PASS: TF trees match"); sys.exit(0)
# ... print diff and sys.exit(1)
```

What's identical: file format parsing is stable across `tf2_tools view_frames` versions (per RESEARCH "Don't Hand-Roll" table). No `pygraphviz` dependency.

What diverges: nothing — this is the canonical impl.

---

### `exts/aic-dt/scripts/sweep_textures.py` (NEW — D-07)

**No direct analog.** Pattern composed from:

- **CLAUDE.md repo root** for the canonical Kit log glob: `ls -t ~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log | head -1`.
- **`mcp_test.py`** (excerpt above) for the socket invocation that triggers the load via `quick_start`.
- **D-07** specifies the regex set: `MDL`, `texture`, `missing`, `pink`, `fallback`, `not found` — refine after first run.

**Skeleton:**

```python
#!/usr/bin/env python3
"""Texture / MDL sweep — load scene, grep latest Kit log for warnings, table-format result."""
import socket, json, glob, os, re, time, sys

KIT_LOG_GLOB = os.path.expanduser('~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_*.log')
PATTERNS = [r'MDL', r'texture', r'missing', r'pink', r'fallback', r'not found']

# 1. Trigger load via MCP (mcp_test.py send_cmd pattern — see verify_phase_1.sh excerpts above)
# 2. Wait for "=== Quick Start Complete ===" in newest log
# 3. Read newest log; for each line matching any pattern, classify as (asset, problem, fix-needed)
# 4. Emit Markdown table to exts/aic-dt/docs/texture-sweep.md
```

What's identical: the socket trigger and log-glob pattern (both from existing tools).

What diverges: the parse + classify step is bespoke; output target is a Markdown table per D-07.

---

### `exts/aic-dt/scripts/snapshot_aic_eval.sh` (NEW — D-01, D-14)

**Closest external analog:** `~/Documents/aic/scripts/run_cheatcode.sh` (Docker bringup, cleanup trap, NVIDIA runtime args, `xhost +local:docker`).

**Excerpt** (`run_cheatcode.sh:64-89`):

```bash
# Clean up containers on exit
trap cleanup EXIT

echo "Starting eval container..."
docker run -d \
    --name "$EVAL_CONTAINER" \
    --gpus all \
    --runtime=nvidia \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    --net=host \
    --privileged \
    "$EVAL_IMAGE" \
    ground_truth:=true start_aic_engine:=true gazebo_gui:="$GAZEBO_GUI"

echo "Waiting ${STARTUP_DELAY}s for Gazebo to load..."
sleep "$STARTUP_DELAY"
```

**Phase 1's snapshot_aic_eval.sh full implementation already written in `01-RESEARCH.md` lines 575-643** — has the exact `docker run` invocation (headless), the Zenoh env exports for in-container `ros2 topic` calls, the `--no-daemon` workaround, and the `view_frames` capture path. Just place it at `exts/aic-dt/scripts/snapshot_aic_eval.sh`.

What's identical: Docker run + cleanup trap + NVIDIA runtime + privileged + host network.

What diverges: Phase 1's snapshot is **headless-only** (`gazebo_gui:=false`); does not need the model container; runs `ros2 topic list/info/echo` + `tf2_tools view_frames` instead of starting CheatCode; pins image digest via `docker inspect --format='{{index .RepoDigests 0}}'` (D-14).

---

### `exts/aic-dt/docs/topic-parity-reference.md` (NEW — D-01, D-14)

**Closest in-repo analog:** `exts/aic-dt/docs/README.md`, `exts/aic-dt/docs/CHANGELOG.md` — for tone, header structure, code-block style.

**No structural analog** — this is the first parity-reference doc. The data to fill it is already captured in `01-RESEARCH.md`:

- "Live Reference Snapshot — `aic_eval` Topic Surface" section (lines 103-256) — full topic list, `/joint_states` echo, `view_frames` tree, image SHA-256 digest.
- D-04 cable workaround note: include it as a "Known Phase-3 work item" callout per CONTEXT.md.

**Format guidance from CONTEXT.md:** "Claude follows the existing extension-doc style." Existing extension docs use Markdown tables for capability-vs-status surfaces. The artifact files (`aic_topics_live.txt`, `aic_frames_live.gv`) referenced in RESEARCH live in the same phase directory and should be linked relatively.

---

### `exts/aic-dt/docs/texture-sweep.md` (NEW — D-07)

**No analog.** Format spec is the rows-per-warning model from CONTEXT.md D-07: "asset, problem, fix." Markdown table:

```markdown
| Asset | Problem | Fix | Status |
|---|---|---|---|
| assets/SC Port/sc_port_visual.usd | Missing texture ./textures/Image_0.png | cp -r upstream textures/ | resolved |
| ... | ... | ... | ... |
```

Generated/appended-to by `sweep_textures.py`.

---

### Asset vendoring (D-05)

**Analog:** the existing vendored `exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd` (md5 byte-identical to AIC source) — proves the `_local_asset` resolver pattern works. Copy semantics: preserve AIC's capitalized folder layout (`assets/NIC Card/`, `assets/SC Port/`, etc.) including sibling `textures/` directories.

**Existing resolver to keep working** (`extension.py:36-54`):

```python
def _get_assets_folder():
    _ext_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _assets = os.path.join(_ext_dir, "assets")
    if not os.path.isdir(_assets):
        raise FileNotFoundError(...)
    return "file://" + os.path.abspath(_assets) + "/"


def _local_asset(relpath):
    """Resolve a local asset path under exts/aic-dt/assets/ as a file:// URI."""
    _ext_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    full = os.path.join(_ext_dir, "assets", relpath)
    if not os.path.exists(full):
        raise FileNotFoundError(f"Asset not found: {full}")
    return "file://" + os.path.abspath(full)
```

**`AIC_OBJECTS` dict update target** (`extension.py:58-79`) — full diff already specified in `01-RESEARCH.md` lines 858-867:

```python
# Replace snake_case "objects/sc_port/sc_port.usd" with capitalized "assets/SC Port/sc_port_visual.usd"
AIC_OBJECTS = {
    "task_board_base": {"usd": "assets/Task Board Base/base_visual.usd", ...},
    "sc_port_1":       {"usd": "assets/SC Port/sc_port_visual.usd", ...},
    "sc_port_2":       {"usd": "assets/SC Port/sc_port_visual.usd", ...},
    "nic_card":        {"usd": "assets/NIC Card Mount/nic_card_visual.usd", ...},
}
```

What's identical: `_get_assets_folder` + `_local_asset` resolution semantics; the `file://` URI pattern; the `position`/`rotation` tuple shape.

What diverges: AIC's capitalized-with-spaces folder names replace snake_case; sibling `textures/` directories must be copied alongside each `.usd`; the legacy `objects/` tree is retired.

---

### Docs RG2→Hand-E corrections (D-03)

**Analog:** the line-numbered list in `01-CONTEXT.md` D-03 + `01-RESEARCH.md` lines 776-786 is the spec.

**Pattern:** simple string replacement of "RG2" → "Robotiq Hand-E" (or "Hand-E" depending on context) at exactly:

- `exts/aic-dt/aic_dt/extension.py:166` (registry description)
- `exts/aic-dt/aic_dt/extension.py:999` (print message)
- `exts/aic-dt/aic_dt/extension.py:2357` (return message)
- `.planning/PROJECT.md:18,28`
- `.planning/REQUIREMENTS.md:10,33`
- `.planning/ROADMAP.md:13,21,47,52`
- `CLAUDE.md:22,159,224` (repo-root)
- `exts/aic-dt/docs/README.md:8`
- `exts/aic-dt/docs/CHANGELOG.md` (Phase 1 entry — note the correction)

**Out of scope** (per CONTEXT.md D-03 minimum target list):

- `exts/aic-dt/aic_dt/tests/test_wrist3_force.py` (`/World/RG2_Gripper` paths) — test artifact, DX-05 / Phase 4.
- `exts/ur5e-dt/...` — sibling extension.
- `.claude/worktrees/...` — agent worktree mirror.
- `.planning/phases/01-foundation-parity/01-DISCUSSION-LOG.md` — historical.

---

## Shared Patterns

### Action graph idempotent cleanup
**Source:** `exts/aic-dt/aic_dt/extension.py:1086-1089` (`setup_force_publish_action_graph`); `exts/aic-dt/aic_dt/extension.py:1190-1193` (`_create_camera_actiongraph`). Both `RemovePrim` if exists before recreate.
**Apply to:** All new action graphs (TF, JointState).

```python
graph_path = "/Graph/ActionGraph_<NAME>"
stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath(graph_path):
    stage.RemovePrim(graph_path)
```

### Atomic + clubbed MCP model (4-surface contract)
**Source:** every existing atom in `extension.py` (e.g., `setup_force_publisher`).
**Apply to:** every Phase 1 capability change.

| Surface | Location |
|---|---|
| 1. `MCP_TOOL_REGISTRY` entry | `extension.py:135-249` |
| 2. `MCP_HANDLERS` entry | `extension.py:252-273` |
| 3. `_cmd_<name>` method | `extension.py:2300-2620` block |
| 4. UI button | `extension.py:418-510` block |

### MCP socket protocol (TCP, single JSON object, no length prefix)
**Source:** `~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py` `send_cmd` function; the same pattern is documented in CLAUDE.md repo root.
**Apply to:** `verify_phase_1.sh`, `sweep_textures.py`, any MCP-driven harness.

### `og.Controller.edit` 3-key shape
**Source:** every existing action graph in `extension.py` (`setup_action_graph`, `setup_force_publish_action_graph`, `_create_camera_actiongraph`, `create_pose_publisher`).
**Apply to:** the two new graphs.

```python
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [...],
        keys.SET_VALUES: [...],
        keys.CONNECT: [...],
    },
)
```

### Relationships set after `og.Controller.edit` (for prim-list inputs)
**Source:** `exts/aic-dt/aic_dt/extension.py:1803-1814` (`create_pose_publisher` sets `inputs:parentPrim` and `inputs:targetPrims` via `CreateRelationship` after the graph is created).
**Apply to:** TF graph (`parentPrim`, `targetPrims`); JointState graph (`targetPrim`).

```python
ros2_prim = stage.GetPrimAtPath(f"{graph_path}/<node_name>")
parent_rel = ros2_prim.GetRelationship("inputs:parentPrim") or \
             ros2_prim.CreateRelationship("inputs:parentPrim", custom=True)
parent_rel.SetTargets([Sdf.Path(parent_prim)])
```

### Cold-cache discipline (CLAUDE.md)
**Source:** `~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py` (referenced from CLAUDE.md operating discipline).
**Apply to:** `verify_phase_1.sh` — call `prime_usd_cache.py status` before any cold launch attempt; if `< 100MB`, `prime_usd_cache.py restore known-good`. After any successful `quick_start`, `snapshot`.

### Local `file://` asset resolution
**Source:** `extension.py:36-54` (`_get_assets_folder`, `_local_asset`).
**Apply to:** all new assets vendored under D-05; do not introduce Nucleus or HTTP refs.

---

## No Analog Found

| File | Role | Data Flow | Reason |
|---|---|---|---|
| `exts/aic-dt/scripts/diff_tf_tree.py` | utility | transform | First `.gv` parser in repo. Reference impl is RESEARCH-internal; no codebase precedent for graphviz parsing. |
| `exts/aic-dt/scripts/sweep_textures.py` | utility | batch | First Kit-log-grep utility. Pieces (mcp_test.py + log glob) come from skill scripts and CLAUDE.md, not the repo proper. |
| `exts/aic-dt/docs/topic-parity-reference.md` | doc | n/a | First parity-reference doc. Tone/structure inspired by existing `docs/README.md` only. |
| `exts/aic-dt/docs/texture-sweep.md` | doc | n/a | First sweep log. Format is the table spec from CONTEXT.md D-07. |

For these, the planner should reference RESEARCH.md (which has the implementations or specs already drafted) rather than searching for further codebase analogs.

---

## Metadata

**Analog search scope:**
- `exts/aic-dt/aic_dt/extension.py` (primary — 2700+ line file, all pattern slices live here)
- `exts/ur5e-dt/ur5e_dt/extension.py` (TF + camera publisher second-source)
- `exts/soarm101-dt/so_arm101_dt/extension.py` (no ROS2 publisher graphs found — not a useful analog)
- `exts/aic-dt/scripts/launch_postload.py` (cold-cache fallback pattern)
- `~/.claude/skills/isaac-sim-extension-dev/scripts/{isaacsim_launch.sh,mcp_test.py,prime_usd_cache.py}`
- `~/Documents/aic/scripts/run_cheatcode.sh` (external — Docker bringup pattern)
- CLAUDE.md repo root (Kit-log path glob, cache-discipline canonical commands)

**Files scanned:** ~10 source files + 3 skill scripts + 1 external repo script. Stopped at strong matches per pattern surface (3-5 analogs each).

**Pattern extraction date:** 2026-05-02
