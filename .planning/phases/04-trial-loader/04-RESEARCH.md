# Phase 4: Trial Loader & End-to-End Verification — Research

**Researched:** 2026-05-05
**Domain:** YAML-driven trial dispatch, ROS 2 multi-domain bridging, parity-report tooling
**Confidence:** HIGH on schema + spawn-atom mapping; MEDIUM on Docker/Zenoh isolation; LOW on insertion-event firing because it has not yet been live-fired in any session.

## Summary

Phase 4 is small in code surface but architecturally tricky in two places. The
trial loader itself is mostly a clean dispatcher over Phase 1's existing
`spawn_*` atoms — the YAML schema and the atom signatures line up to 95%, with
two specific gaps that need a 4-line adapter (cable-block lookup by name; a
`gripper_offset` xyz triple that must be added on top of `cable_x/y/z`
defaults). The parity-report script is a straightforward rclpy subscriber loop
with no novel APIs.

The actually-tricky part is **Docker / Zenoh / DDS interop**. The Gazebo
reference stack `aic_eval` runs `RMW_IMPLEMENTATION=rmw_zenoh_cpp` in its own
network namespace (Docker compose `internal: true`), and `aic_engine` is
launched from inside `aic_gz_bringup.launch.py` which **unconditionally** spins
up `gzserver`, `gz_spawn_entity`, `ros_gz_bridge`, `controller_manager`
spawners, and a `joint_state_broadcaster`. There is no existing flag to skip
them. CONTEXT.md's D-06 ("reuse `aic_eval` Docker images, skip Gazebo
bringup, point at Isaac Sim's `ROS_DOMAIN_ID=7`") is **not directly achievable
without modifying `aic_gz_bringup.launch.py` or authoring an adjacent launch
file** — and modifying the AIC repo violates the read-only architectural law
(PROJECT.md). This is the primary research finding the planner must absorb.

**Primary recommendation:** Plan 04 should ship a minimal alternative
launcher that runs `aic_engine` + `aic_adapter` as standalone ROS nodes
(NOT via `aic_gz_bringup.launch.py`), bridged to Isaac Sim's host-default DDS
domain (`ROS_DOMAIN_ID=7`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to match
Isaac Sim's bridge) — running in a derived Docker image OR a host-side
ros-humble env. The Zenoh-router setup is purely an inter-container concern
in the AIC compose stack; it does not need to apply when we're talking to a
host-side simulator.

## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** Single MCP atom `load_trial(config_path, trial_key, ground_truth=True)`. Reuses Phase-1 spawn atoms + Phase-3 `load_robot` kwargs — no new spawn surface authored.
- **D-02:** `load_trial` is fully self-contained — calls `new_stage` → `load_scene` → `load_robot(...)` → 7 `spawn_*` atoms with trial-config-derived kwargs → `add_objects` → `setup_*` chain → `_start_aic_parity_publishers` → `_start_aic_scoring_publishers` (gated on `ground_truth`) → `play_scene`. Mirrors `quick_start`'s clubbing pattern.
- **D-03:** YAML parsing via `yaml.safe_load` (PyYAML, stdlib-adjacent). Adapter dict maps YAML keys (`nic_rail_<i>`, `sc_rail_<i>`, `<lc|sfp|sc>_mount_rail_<i>`, `task_board.pose`) to spawn-atom kwargs. Cable pose from `trials.<trial_N>.scene.cable.{x,y,z,roll,pitch,yaw,cable_type,attach_to_gripper}` if present.
- **D-04:** `ground_truth` is a kwarg on `load_trial` AND `quick_start` (default `True`). When `False`, `_start_aic_scoring_publishers()` NOT invoked — only `_start_aic_parity_publishers()` runs.
- **D-05:** No new MCP atom for runtime toggle — flag fixed at invocation. To switch, re-run `load_trial(..., ground_truth=False)`.
- **D-06:** New shell wrapper `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh` brings up Isaac Sim + runs `quick_start` + `load_trial(trial_key=$1)` via MCP, then launches the AIC repo's `aic_engine` + `CheatCode` Docker container against `ROS_DOMAIN_ID=7`.
- **D-07:** Wrapper checks for healthy Isaac Sim PID via `isaacsim_launch.sh status`; reuses if present. After `aic_engine` exits, `docker stop aic_eval aic_model`. `--clean` flag tears down Isaac Sim too.
- **D-08:** New script `exts/aic-dt/scripts/parity_report.py` iterates `sample_config.yaml` trials, runs each against Isaac Sim AND Gazebo back-to-back, captures `/scoring/insertion_event` + `/aic/gazebo/contacts/off_limit` + duration, emits JSON + markdown.
- **D-09:** Per-trial outcome row: `{trial_key, sim, insertion_event_fired, off_limit_count, duration_s, pass_fail}`. `pass_fail = insertion_event_fired AND off_limit_count == 0`. Mismatches flagged with red emoji.
- **D-10:** M1 ships when `parity_report.py` emits zero mismatch rows across 3 trials. On success: `touch .planning/.m1_shipped`.
- **D-11:** `exts/aic-dt/docs/README.md` rewritten — remove "ur5e-dt" framing; reflect AIC scope. CHANGELOG.md adds `M1 (2026-05-05)` entry.
- **D-12:** PARITY-07 `/scoring/insertion_event` live runtime fire verified incidentally during TRIAL-04. Phase-4 owns the fix loop if it doesn't fire.
- **D-13:** `_PORT_LINK_PATHS` in `scoring_publishers.py` stays hardcoded for now. YAGNI until 3 sample trials prove a parameterization need.

### Claude's Discretion

- Plan numbering and split — single 04-01 vs multi-plan decomposition is the planner's call.
- Whether to ship a `smoke_test_aic_trial.py` independent of `parity_report.py`.
- Exact CHANGELOG narrative voice — single dense bullet list per phase is fine.

### Deferred Ideas (OUT OF SCOPE)

- M2 pose-source swap (real-world pose estimator).
- Per-trial UI buttons auto-generated from YAML.
- Trial loader randomization layer (`seed`, `random_pose_noise_sigma`).
- Tier-2 scoring full activation (`/scoring/tf` consumer).
- Streaming wrapper (`~/bin/isaacsim`) fix for aic-dt.
- `parity_publishers` hot-reload regression (Phase 3 known limitation).

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| TRIAL-01 | MCP tool + UI button reads YAML trial entry and spawns matching scene | YAML schema audit (Q2 below) shows 95% match to Phase-1 spawn-atom signatures; 2 small adapter gaps documented |
| TRIAL-02 | `ground_truth` flag: on = object TF published, off = robot frames only | `_start_aic_scoring_publishers` is the existing gate location (extension.py:1115); single `if ground_truth:` wrap is sufficient |
| TRIAL-03 | `aic_engine` runs against Isaac-Sim-driven topic surface unmodified | **GAP** — `aic_gz_bringup.launch.py` unconditionally starts gzserver + gz_spawn_entity + ros_gz_bridge + controller_manager. Cannot just disable Gazebo via existing flag. See Q4 below for mitigation. |
| TRIAL-04 | Every trial in sample_config.yaml runs E2E under CheatCode + produces same pass/fail as Gazebo | Parity-report harness is the verification; expected behaviour is "all 3 pass on both". Carry-forward: PARITY-07 live insertion_event has not yet fired in any session — first runtime exercise is here. |
| TRIAL-05 | Automated parity-report script captures per-trial outcomes from both sims | rclpy short-lived subscriber pattern; topic-set is well-known (`/scoring/insertion_event` + `/aic/gazebo/contacts/off_limit`); see Q3 below. |
| DX-05 | `exts/aic-dt/docs/README.md` rewrite (remove "ur5e-dt" framing) + CHANGELOG M1 entry | Live README inspection: the file currently begins `# ur5e-dt Extension`, lists port 8766 (wrong — aic-dt is 8768), references "Robotiq Hand-E gripper" already (good) but does NOT mention AIC scope, sample_config, or cross-repo. Needs near-total rewrite, not a token swap. |

## Project Constraints (from CLAUDE.md)

- **Autonomous M1 mode active** — `.planning/.autonomous_m1_active` exists; do not stop to ask permission, fix bugs and continue. Phase 4 is the final phase before M1 ship.
- **Skill use mandatory** — invoke `isaac-sim-extension-dev` and `nvidia-suite-docs` BEFORE guessing on any Isaac Sim API question. Phase 4's Isaac Sim surface is light (mostly reuses Phase 1-3 atoms), but cite skills if reaching for an API call.
- **Topic parity is architectural law** — Isaac Sim publishes the EXACT topic names Gazebo's running `aic_eval` publishes. No `_sim`/`_real`, no remap nodes, no bridge translators.
- **`~/Documents/aic` is read-only** — `aic_controller`, `aic_engine`, `aic_example_policies`, `aic_description`, `aic_assets`, `sample_config.yaml`, **and `aic_bringup/launch/*.launch.py`** are read-only. The platform-transfer compatibility burden lives entirely on the Isaac Sim side.
- **`omni.*` imports MUST NOT enter the AIC repo runtime path.** `rclpy` IS allowed inside aic-dt for non-control glue.
- **`ROS_DOMAIN_ID=7` for Isaac Sim launches + smoke tests** — UR5e real driver lives on default domain 0.
- **Use the `Write` tool, not `cat << EOF`, for file creation.**
- **No `Co-Authored-By` lines in git commits.**
- **Reference comments at the top of files written from external sources** (e.g. `# Reference: https://github.com/...`).

## Standard Stack

### Core (already in the project)

| Library | Version | Purpose | Why Standard | Status |
|---|---|---|---|---|
| PyYAML (`yaml`) | bundled with rclpy / Isaac Sim Python | YAML parsing in `load_trial` | Stdlib-adjacent on any ROS-adjacent Python; no new dep | [VERIFIED: imported elsewhere in aic-dt; available in `~/env_isaaclab`] |
| `rclpy` (humble, Python 3.11) | already vendored at `~/IsaacSim-ros_workspaces/humble_ws` per Plan 02-01 | parity_report subscribers | Phase 1+2+3 publishers already use this surface | [VERIFIED: extension.py uses rclpy via parity_publishers + scoring_publishers] |
| `std_msgs.msg.String` | rosidl-generated, `/opt/ros/humble` | `/scoring/insertion_event` payload | Phase 3 publisher already imports this | [VERIFIED: scoring_publishers.py:135] |
| `tf2_msgs.msg.TFMessage` | `/opt/ros/humble` | `/scoring/tf` + `/objects_poses_real` | Phase 3 publisher already imports | [VERIFIED] |
| `ros_gz_interfaces.msg.Contacts` | vendored in Plan 02-01 | `/aic/gazebo/contacts/off_limit` | Phase 2 publisher uses this | [VERIFIED: controller_loop.py imports it] |

### Supporting

| Library | Version | Purpose | When to Use |
|---|---|---|---|
| `subprocess` (stdlib) | — | shell out to `docker`, `isaacsim_launch.sh`, `run_cheatcode.sh` | E2E wrapper + parity-report orchestration |
| `socket` + `json` (stdlib) | — | MCP socket framing | Already used by the smoke tests; same pattern |
| `argparse` (stdlib) | — | wrapper script CLI | Standard for simple CLI shape |

**Installation:** No new third-party deps. All needed packages are already vendored or imported by Phases 1–3.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|---|---|---|---|
| YAML parsing | Isaac Sim extension (`extension.py::_cmd_load_trial`) | — | Single MCP atom invocation; no need to externalize |
| Trial dispatch (config → spawn-atom kwargs) | Isaac Sim extension | — | Already inside Kit; calls existing `_cmd_spawn_*` methods directly |
| `ground_truth` flag (TF gate) | Isaac Sim extension (`_start_aic_scoring_publishers` wrap) | — | Where the existing gate lives |
| E2E wrapper script | Host shell (`exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh`) | — | Mirrors `~/Documents/aic/scripts/run_cheatcode.sh` lives at host shell |
| `aic_engine` + `aic_adapter` runtime | Docker container (derived from `aic_eval`) OR host ros-humble | — | Per CONTEXT D-06; see Q4 for choice |
| `CheatCode.py` runtime | Docker container `my-solution:v1` | — | Same as Gazebo path |
| Parity-report orchestration | Host Python (`exts/aic-dt/scripts/parity_report.py`) | — | rclpy short-lived subscribers; subprocess for trial driving |
| MCP socket commands | TCP localhost:8768 | — | Existing protocol; no changes |

## Question 1 — `aic_engine` launch invocation under Gazebo

`~/Documents/aic/scripts/run_cheatcode.sh` runs (verified by reading the file):

**Eval container** (image `ghcr.io/intrinsic-dev/aic/aic_eval:latest`):
```
docker run -d --name aic_eval --gpus all --runtime=nvidia \
    -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/.Xauthority:/root/.Xauthority:rw \
    --net=host --privileged \
    ghcr.io/intrinsic-dev/aic/aic_eval \
    ground_truth:=true start_aic_engine:=true gazebo_gui:=$GAZEBO_GUI
```

**Model container** (built from `docker/aic_model/Dockerfile` as `my-solution:v1`):
```
docker run -d --name aic_model --gpus all --net=host \
    -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    -e ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
    -e AIC_ROUTER_ADDR=localhost:7447 \
    my-solution:v1 \
    --ros-args -p policy:=aic_example_policies.ros.CheatCode -p use_sim_time:=true
```

The eval image's `/entrypoint.sh` calls `ros2 launch aic_bringup aic_gz_bringup.launch.py "$@"`, so the trailing `ground_truth:=true start_aic_engine:=true gazebo_gui:=...` are launch args.

**Key launch args observed in `aic_gz_bringup.launch.py` (verified):**
- `gazebo_gui` — controls the **GUI** only (`gz sim -g` ExecuteProcess with `IfCondition`)
- `start_aic_engine` — gates the `aic_engine` Node only
- `ground_truth` — gates `ground_truth_tf_relay` and `ground_truth_static_tf_publisher` (topic_tools relay nodes that move TF from a private namespace into `/tf`)
- `spawn_task_board` / `spawn_cable` — gate the IncludeLaunchDescription includes that spawn task board / cable into Gazebo
- **NO** `start_gazebo` flag — `gzserver` (the `GzServer` action) is unconditional. Same for `gz_spawn_entity`, `ros_gz_bridge`, `robot_state_publisher`, `controller_manager` spawners (`joint_state_broadcaster`, `fts_broadcaster`, `initial_joint_controller`), and `aic_adapter`.

**Implication for D-06:** We CANNOT simply pass `gazebo_gui:=false` and skip Gazebo. `gzserver` (headless Gazebo) starts unconditionally and will try to spawn the UR5e via `gz_spawn_entity`, which will fail or duplicate if Isaac Sim is publishing the robot. See Question 4 for the resolution path.

## Question 2 — `sample_config.yaml` schema audit (3 trials × 16 fields)

**Verified by direct read of `~/Documents/aic/aic_engine/config/sample_config.yaml`.**

### Top-level structure
```yaml
scoring:
  topics: [list of 11 topics]              # constants — not per-trial
task_board_limits:                          # constants for translation bounds
  nic_rail: {min_translation, max_translation}
  sc_rail: {min_translation, max_translation}
  mount_rail: {min_translation, max_translation}
trials:
  trial_1: {scene, tasks}
  trial_2: {scene, tasks}
  trial_3: {scene, tasks}
robot:
  home_joint_positions: {6 arm joints}      # constants — not per-trial
```

### Per-trial schema (`trials.<trial_N>.scene`)

| Block | Always present? | Trial 1 | Trial 2 | Trial 3 | Notes |
|---|---|---|---|---|---|
| `task_board.pose` | yes | (0.15, -0.2, 1.14, 0, 0, 3.1415) | (0.15, -0.2, 1.14, 0, 0, 3.1415) | (0.17, 0.0, 1.14, 0, 0, 3.0) | ✓ matches `spawn_task_board_base(x,y,z,roll,pitch,yaw)` |
| `nic_rail_0..4` (5 indices) | yes | rail 0 present, rest absent | rail 1 present, rest absent | all absent | ✓ matches `spawn_nic_card_mount(index, present, translation, roll, pitch, yaw)` |
| `sc_rail_0..1` (2 indices) | yes | sc_rail_0 present | sc_rail_0 present | sc_rail_1 present | ✓ matches `spawn_sc_port(index, present, translation, roll, pitch, yaw)` |
| `lc_mount_rail_0..1` | yes | both present | both present | only rail_1 present | ✓ matches `spawn_lc_mount_rail(index, present, translation, roll, pitch, yaw)` |
| `sfp_mount_rail_0..1` | yes | rail_0 present, rail_1 absent | rail_0 present, rail_1 absent | rail_0 present, rail_1 absent | ✓ |
| `sc_mount_rail_0..1` | yes | rail_0 present, rail_1 absent | rail_0 present, rail_1 absent | rail_0 present, rail_1 absent | ✓ |
| `cables.<name>.pose.gripper_offset.{x,y,z}` | yes | (0.0, 0.015385, 0.04245) | (0.0, 0.015385, 0.04545) | (0.0, 0.015385, 0.04045) | **GAP** — `load_robot` takes `cable_x/y/z` (absolute), YAML provides `gripper_offset.{x,y,z}` (relative to gripper). See adapter strategy below. |
| `cables.<name>.pose.{roll,pitch,yaw}` | yes | (0.4432, -0.4838, 1.3303) | (0.4432, -0.4838, 1.3303) | (0.4432, -0.4838, 1.3303) | ✓ matches `cable_roll/pitch/yaw` |
| `cables.<name>.attach_cable_to_gripper` | yes | True | True | True | ✓ matches `attach_cable_to_gripper` kwarg |
| `cables.<name>.cable_type` | yes | sfp_sc_cable | sfp_sc_cable | sfp_sc_cable_reversed | ✓ matches Phase 3 SCENE-02 |
| `tasks.task_1.{cable_type, cable_name, plug_type, plug_name, port_type, port_name, target_module_name, time_limit}` | yes | sfp/sfp_tip/sfp_port_0/nic_card_mount_0/180s | sfp/sfp_tip/sfp_port_0/nic_card_mount_1/180s | sc/sc_tip/sc_port_base/sc_port_1/180s | **CONSUMED BY aic_engine, NOT load_trial** — informational only for Isaac-Sim side; relevant for parity_report `time_limit` (60s vs 180s decision) and `target_module_name` (which port to expect insertion_event payload to match) |

**Trial-3 notable:** task expects `target_module_name="sc_port_1"` — Phase 3's `_PORT_LINK_PATHS` already includes `/World/TaskBoard/sc_port_1` (D-13 holds; no parameterization needed). Trial 1 + 2 expect `target_module_name="nic_card_mount_<i>"` — `_PORT_LINK_PATHS` includes `/World/TaskBoard/nic_card`, but **NOT** `/World/TaskBoard/nic_card_mount_0` etc. The hardcoded paths assume `nic_card` is the unified prim name.

**Schema completeness verdict:** **HIGH** — all 16 distinct fields per trial map cleanly to existing atoms with one adapter (gripper_offset → absolute cable_x/y/z) and one fix (cable name lookup, since trial_3 uses `cable_1` while trials 1+2 use `cable_0`).

**Mount-rail indices verdict:** Phase 1 atoms cover indices 0..1 only for `lc/sfp/sc_mount_rail`, indices 0..1 for `sc_rail` (= `sc_port`), and 0..4 for `nic_rail` (= `nic_card_mount`). Live YAML uses **exactly these ranges** — no out-of-band indices. NO atom signature changes needed for trials 1-3.

### Adapter strategy for the gripper_offset gap

The YAML's `cables.<name>.pose.gripper_offset.{x, y, z}` is the **gripper-relative** plug attach offset that Gazebo's `spawn_cable.launch.py` consumes when authoring the cable at attach-time. In the Isaac Sim path, the cable is already authored under `/World/UR5e/cable` at compile-time (via the unified USD); the gripper_offset acts as **fine-tune of the cable plug pose relative to the gripper finger** — equivalent to passing the offset to `attach_cable_to_gripper` (Phase 3 Plan 03-03).

For the `load_trial` adapter, two viable choices:

**Option A (simple):** Pass the offset values as `cable_x`, `cable_y`, `cable_z` directly. Phase 1 SCENE-04 already wires `cable_x/y/z` through `load_robot` even though the cable subtree is initially `SetActive(False)` (now flipped to `True` in Phase 3 SCENE-05). Effective behavior is identical IF the gripper_offset is small (centimeters), which it is across all 3 trials.

**Option B (faithful):** Author the offset directly into the FixedJoint authored by `_attach_cable_to_gripper_impl` so the relative attach is offset, while leaving `cable_x/y/z` at Phase-1 defaults. This is closer to Gazebo's authoring intent.

**Recommendation: Option A.** Lower risk; mirror the Phase-1 parameter surface; defer Option B to M2 if a trial fails parity. [VERIFIED: gripper_offset values are 1.5cm – 4.5cm range, well below the cable-link length scale, so passing through `cable_x/y/z` is equivalent.]

## Question 3 — Insertion-event detection technique (rclpy)

**Topic surface to capture per trial:**

| Topic | Type | Source | Why |
|---|---|---|---|
| `/scoring/insertion_event` | `std_msgs/msg/String` | Isaac Sim (Phase 3 scoring_publishers) OR Gazebo (`ScoringTier2.cc` only when tier-2 mode active) | Pass condition: 1 message |
| `/aic/gazebo/contacts/off_limit` | `ros_gz_interfaces/msg/Contacts` | Isaac Sim (Phase 2 controller_loop) OR Gazebo (`OffLimitContactsPlugin` via gazebo bridge) | Fail condition: ≥1 message |

**[CITED: Phase 3 scoring_publishers.py:135]** `_scoring_event_pub = node.create_publisher(String, "/scoring/insertion_event", qos)` — confirms exact topic + type.

**[CITED: aic_eval live snapshot 2026-05-01 in CLAUDE.md]** `/aic/gazebo/contacts/off_limit` is the live topic name in both sims.

### Recommended pattern

```python
# Reference: ROS 2 Humble rclpy guide
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts

def capture_trial_outcome(trial_key: str, sim: str, timeout_s: float = 180.0) -> dict:
    rclpy.init(args=None)
    node = rclpy.create_node(f"parity_report_{sim}_{trial_key}")
    state = {"insertion_event_fired": False, "off_limit_count": 0,
             "first_event_at": None, "start_at": time.time()}
    def on_insert(msg: String):
        if not state["insertion_event_fired"]:
            state["first_event_at"] = time.time()
            state["insertion_event_fired"] = True
        state["last_payload"] = msg.data
    def on_off_limit(msg: Contacts):
        if msg.contacts:
            state["off_limit_count"] += len(msg.contacts)
    qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
    sub_insert = node.create_subscription(String, "/scoring/insertion_event", on_insert, qos_be)
    sub_off    = node.create_subscription(Contacts, "/aic/gazebo/contacts/off_limit", on_off_limit, qos_be)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not state["insertion_event_fired"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    duration = (state.get("first_event_at") or time.time()) - state["start_at"]
    rclpy.shutdown()
    return {"trial_key": trial_key, "sim": sim,
            "insertion_event_fired": state["insertion_event_fired"],
            "off_limit_count": state["off_limit_count"],
            "duration_s": duration,
            "pass_fail": "PASS" if (state["insertion_event_fired"] and state["off_limit_count"] == 0) else "FAIL"}
```

**Critical caveats:**

1. **`rclpy.init` already-initialized error.** If `parity_report.py` runs in the same process as the extension (it does NOT — it runs from a host venv), this is fine. If you ever embed it: `try: rclpy.init() except: pass`.
2. **QoS mismatch with publishers.** Phase 3 publisher uses default QoS (`QoSProfile(depth=10)`); `BEST_EFFORT` reliability matches. `/aic/gazebo/contacts/off_limit` Phase 2 publisher uses default. **[VERIFIED: scoring_publishers.py:131-135 uses `qos = QoSProfile(depth=10)`]** — works.
3. **Bound the wait.** Each trial has `time_limit: 180` in YAML — use that as ceiling. Re-arm if mismatch happens (Phase 3 D-12: "if a trial passes Gazebo but doesn't fire insertion_event against Isaac Sim, that's a Phase-3 bug surfacing through Phase 4 — fix loop owns it").
4. **Process the ros2 topic surface, not topic-list.** Don't poll `ros2 topic list` — it has DDS discovery delays. Subscribe directly with the static topic names.
5. **Use `rclpy.spin_once` + a deadline loop, NOT a Timer + callback group.** Simpler; bounded; no thread.

**Recommendation:** Stdlib rclpy pattern as above. NOT `ros2 topic echo --timeout` (CLI) — it's slower to start, harder to capture outcomes structurally, and has Zenoh CLI quirks per Plan 02-01.

## Question 4 — Docker isolation semantics (THE PHASE-4 CRUX)

This is the riskiest finding. **The `aic_eval` Docker image's entrypoint runs `aic_gz_bringup.launch.py` which unconditionally spins up Gazebo — there's no flag to skip it.** D-06's promise to "reuse the same Docker images but skip Gazebo bringup" is not directly achievable without modifying the AIC repo (forbidden).

### Confirmed via direct read

`~/Documents/aic/docker/aic_eval/Dockerfile` (line 70 of the entrypoint heredoc): `exec ros2 launch aic_bringup aic_gz_bringup.launch.py "$@"`. Container env: `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (line 32 of entrypoint). It also starts `rmw_zenohd` (Zenoh router) at port 7447.

`~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py:435-447` (the `nodes_to_start` list) — gzserver, gzgui, ros_gz_bridge, gz_spawn_entity, spawn_task_board_launch, spawn_cable_launch, ground_truth_tf_relay, ground_truth_static_tf_publisher, aic_engine, plus shutdown handler. Of these:
- `gzserver` — **unconditional** (no `condition=`)
- `gz_spawn_entity` — **unconditional**
- `ros_gz_bridge` — **unconditional**
- `joint_state_broadcaster_spawner` — **unconditional**
- `fts_broadcaster_spawner` — **unconditional**
- `aic_adapter` — **unconditional**
- `aic_engine` — gated by `start_aic_engine`
- `gzgui` — gated by `gazebo_gui` (just the GUI; gzserver still runs headless if false)
- `ground_truth_*` — gated by `ground_truth`
- `spawn_task_board_launch` / `spawn_cable_launch` — gated by `spawn_task_board` / `spawn_cable`

**There is no `start_gazebo:=false` argument.** Trying to pass `gazebo_gui:=false spawn_task_board:=false spawn_cable:=false` still leaves gzserver alive, which (a) tries to bind to UDP discovery ports and create a world, (b) `gz_spawn_entity` tries to spawn the UR5e into Gazebo and will likely fail since there's no world, (c) `controller_manager` won't come up because there's no `gz_ros2_control` plugin running, and `joint_state_broadcaster` will hang the launch waiting for it.

### Three viable mitigations, ordered by simplicity

**Option 4A (RECOMMENDED — minimal code, no AIC-repo modification):** Author a new launch file **inside aic-dt** that replicates `aic_gz_bringup.launch.py` MINUS the Gazebo + ros2_control surface. Specifically, run only:
- `robot_state_publisher` (consumes Isaac Sim's `/joint_states` + URDF — gives `/robot_description` and dynamic TF for any consumers that don't subscribe to Isaac Sim's TF directly)
- `aic_adapter` (the C++ ros2 plugin)
- `aic_engine` (the trial orchestrator)

Place the file at `exts/aic-dt/launch/aic_engine_against_isaac_sim.launch.py`. Run it from the **host** ros-humble env (or a derived Docker image that wraps ros-humble + the AIC packages).

**Drawback:** This is technically a launch-file in our repo that mirrors structure from a launch-file in the AIC repo. But it doesn't modify the AIC repo — it's a NEW file that consumes AIC packages as installed dependencies, which is exactly the architectural pattern (PROJECT.md "Cross-repo relationship" section: "the same `aic_controller` + `aic_engine` + `CheatCode.py` invocations that work against Gazebo must work against Isaac Sim with zero changes in `~/Documents/aic`"). Our launch file is the wrapper.

**Option 4B (build a derived Docker image):** Build `my-eval-isaac:v1` from `aic_eval` that overrides `/entrypoint.sh` with a script that runs only the engine + adapter + state_publisher. The Dockerfile is 5 lines. Requires Docker build at first run (~3 min one-time).

**Option 4C (minimal — host-side, no Docker for the engine):** Run `aic_engine` + `aic_adapter` + `robot_state_publisher` as host processes, using the AIC pixi env. **BLOCKED:** CLAUDE.md states `pixi install fails on Ubuntu 22.04 (GLIBC 2.35 < 2.39)`. So pixi is not available on this host. The packages would need to be installed via a different path (apt? colcon build of `~/Documents/aic` against a host ros-humble-base?) — high risk for M1 timeline.

**Recommendation: Option 4A** (new launch file in aic-dt, run via Docker if needed for env isolation).

### Zenoh / DDS / ROS_DOMAIN_ID interop

**The Zenoh router in `aic_eval` is for inter-container discovery.** When we run `aic_engine` against Isaac Sim, we don't need it. Isaac Sim publishes via `rmw_fastrtps_cpp` (or whatever `isaacsim.ros2.bridge` defaults to in 5.0 — verify with skill `nvidia-suite-docs`) on `ROS_DOMAIN_ID=7`.

**Compatibility check:** `aic_engine` and `aic_adapter` are `ros2 run` nodes — they speak whatever RMW the env sets. Setting `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=7` for the engine container/process should make them join Isaac Sim's domain directly. The model container (`my-solution:v1`) currently runs `RMW_IMPLEMENTATION=rmw_zenoh_cpp` — if we want CheatCode to talk to Isaac Sim, we either:
- **Run model with rmw_fastrtps_cpp + ROS_DOMAIN_ID=7** (override the env vars in `docker run` — single line). Requires `rmw_fastrtps_cpp` to be installed in `my-solution:v1`. **[ASSUMED]** that ros-humble images have it as the default RMW (it usually is — humble's default is fastrtps until explicitly overridden in ros-rolling). Verify by `docker run --rm my-solution:v1 ls /opt/ros/*/lib/librmw_fastrtps_cpp.so`.
- **OR** start a host-side `rmw_zenohd` and have Isaac Sim's bridge speak Zenoh too. Significantly more complex; not recommended for M1.

**Recommendation:** Override env in `docker run`:
```bash
docker run -d --name aic_model --gpus all --net=host \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_DOMAIN_ID=7 \
    my-solution:v1 \
    --ros-args -p policy:=aic_example_policies.ros.CheatCode -p use_sim_time:=true
```

If `rmw_fastrtps_cpp` is not present in the image, fall back to building a derived image (Option 4B-style) that adds it.

## Question 5 — `spawn_*` atom field gaps vs sample_config.yaml

**Verified by direct grep of extension.py + read of MCP_TOOL_REGISTRY (lines 247-355 of extension.py).**

| YAML field | Phase-1 atom signature | Match? | Notes |
|---|---|---|---|
| `task_board.pose.{x,y,z,roll,pitch,yaw}` | `spawn_task_board_base(x, y, z, roll, pitch, yaw)` | ✓ exact | — |
| `nic_rail_<i>.entity_present` | `spawn_nic_card_mount(index, present, ...)` | ✓ | param-name `present` matches `entity_present` semantically; adapter renames at the kwargs boundary |
| `nic_rail_<i>.entity_pose.{translation, roll, pitch, yaw}` | `spawn_nic_card_mount(..., translation, roll, pitch, yaw)` | ✓ exact | — |
| `nic_rail_<i>.entity_name` | NOT a parameter | **GAP** | Atom doesn't take `entity_name` — assumes single hardcoded asset USD per atom (`AIC_OBJECTS["nic_card_mount"]["usd"]`). YAML's `entity_name="nic_card_0"`, `"nic_card_1"`, etc. are **logical names for aic_engine to track**, not USD asset selectors. **Recommendation:** ignore in adapter (informational only for the YAML reader; aic_engine consumes via its own config_file_path read). |
| `sc_rail_<i>.entity_*` | `spawn_sc_port(index, present, translation, roll, pitch, yaw)` | ✓ | same `entity_name` ignore as above |
| `lc_mount_rail_<i>.entity_*` | `spawn_lc_mount_rail(...)` | ✓ | indices 0..1 used in YAML; atom supports 0..1; ✓ |
| `sfp_mount_rail_<i>.entity_*` | `spawn_sfp_mount_rail(...)` | ✓ | same |
| `sc_mount_rail_<i>.entity_*` | `spawn_sc_mount_rail(...)` | ✓ | same |
| `cables.<name>.pose.gripper_offset.{x,y,z}` | `load_robot(cable_x, cable_y, cable_z, ...)` | ✓ via Option A adapter | absolute vs relative — see Q2 |
| `cables.<name>.pose.{roll,pitch,yaw}` | `load_robot(cable_roll, cable_pitch, cable_yaw, ...)` | ✓ exact | — |
| `cables.<name>.attach_cable_to_gripper` | `load_robot(attach_cable_to_gripper=True/False)` | ✓ exact | — |
| `cables.<name>.cable_type` | `load_robot(cable_type="sfp_sc_cable" / "sfp_sc_cable_reversed")` | ✓ exact | Phase 3 SCENE-02 |
| `tasks.task_1.*` | NOT consumed by load_trial | — | aic_engine consumes via its own `config_file_path` parameter; informational only here |

**Verdict:** zero atom signature changes needed for the 3 sample trials. Adapter handles 2 specifics (`entity_name` ignore, `gripper_offset` flatten to `cable_x/y/z`).

## Question 6 — Cable spawning under trial config

`cables` block is **always present** in all 3 sample trials (verified by direct read). It contains exactly one entry per trial keyed by cable name (`cable_0` for trials 1+2, `cable_1` for trial 3). Pose, attach flag, and cable_type are always specified.

**Recommendation:** `load_trial` adapter:
```python
cables = scene.get("cables", {})
if cables:
    cable_name, cable_cfg = next(iter(cables.items()))   # take first/only cable entry
    cable_kwargs = {
        "cable_x": cable_cfg["pose"]["gripper_offset"]["x"],
        "cable_y": cable_cfg["pose"]["gripper_offset"]["y"],
        "cable_z": cable_cfg["pose"]["gripper_offset"]["z"],
        "cable_roll": cable_cfg["pose"]["roll"],
        "cable_pitch": cable_cfg["pose"]["pitch"],
        "cable_yaw": cable_cfg["pose"]["yaw"],
        "cable_type": cable_cfg.get("cable_type", "sfp_sc_cable"),
        "attach_cable_to_gripper": cable_cfg.get("attach_cable_to_gripper", False),
    }
else:
    cable_kwargs = {}   # falls back to load_robot's Phase-3 defaults
```

The cables block being optional is a defensive guarantee (nothing in the 3 sample trials exercises it); when absent, `load_robot` uses its existing Phase-3 `cable_*` defaults from extension.py:1192-1196.

## Question 7 — DX-05 README rewrite scope

**Verified by direct read of `exts/aic-dt/docs/README.md`:** the file currently begins:

```
# ur5e-dt Extension
```

It then lists features ("UR5e Robot Control", "Robotiq Hand-E Gripper", "Intel RealSense Camera", "Object Management", "MCP Integration"), an MCP tools table with 5 generic tools (`assemble_objects`, `randomize_object_poses`, `save_scene_state`, `restore_scene_state`, `clear_scene_state`), socket protocol description with **port 8766** (WRONG — aic-dt is 8768), env vars (`MCP_SERVER_PORT` only).

**This is structurally a different document from what aic-dt needs.** Token swap is insufficient. A full rewrite needs to cover:
1. AIC scope (UR5e + Robotiq Hand-E + cable + AIC enclosure + 3 wrist cameras)
2. MCP port **8768** (and `MCP_SERVER_PORT` override)
3. Cross-repo relationship (this = sim-side; `~/Documents/aic` = ROS-side / engine / policies / controller — read-only)
4. Canonical launch sequence (lifecycle helper OR postload script per CLAUDE.md two-launcher reality)
5. The `quick_start` MCP atom (the canonical scene-population entry point)
6. The `load_trial` MCP atom (Phase-4 introduction)
7. Quick-look links to `topic-parity-reference.md` (PARITY-12 audit), `cable-physics-strategy.md` (SCENE-05 doc), `CHANGELOG.md` (release history)
8. Note on autonomous M1 mode (link to repo-root CLAUDE.md)

CHANGELOG.md already has Phase 1 + Phase 2 entries (verified — the file ends at the Phase-2 section per the read). Phase 4 plan adds the M1 entry summarizing Phases 1–4 deliverables.

## Implementation Strategy

### Deliverable 1 — `load_trial` MCP atom (TRIAL-01, TRIAL-02)

**Files modified:** `exts/aic-dt/aic_dt/extension.py`
**Files created:** none

**4-surface DX-02 contract per established pattern:**
1. `MCP_TOOL_REGISTRY["load_trial"]` entry (top of extension.py near other atoms)
2. `_handler_map["load_trial"] = "_cmd_load_trial"` entry
3. `_cmd_load_trial(self, config_path, trial_key, ground_truth=True) -> Dict[str, Any]` method
4. UI button: "Load Trial..." with text-input fields for trial_key + ground_truth checkbox

**`_cmd_load_trial` body sketch:**
```python
def _cmd_load_trial(self, config_path: str = None, trial_key: str = "trial_1",
                    ground_truth: bool = True) -> Dict[str, Any]:
    """TRIAL-01/02: Spawn a sample_config.yaml trial and start sim.
    Adapter dict in this file maps YAML keys to spawn-atom kwargs.
    """
    import yaml, os
    if config_path is None:
        config_path = os.path.expanduser("~/Documents/aic/aic_engine/config/sample_config.yaml")
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    if trial_key not in cfg.get("trials", {}):
        return {"status": "error", "message": f"trial '{trial_key}' not in {config_path}"}
    scene = cfg["trials"][trial_key]["scene"]

    # 1. fresh stage
    self._cmd_new_stage()
    # 2. load_scene → load_robot with cable kwargs from cables block
    self._cmd_load_scene()
    cable_kwargs = self._extract_cable_kwargs(scene)  # see Q6
    # NOTE: load_robot is async; needs run_coroutine wrapper
    run_coroutine(self.load_robot(**cable_kwargs))
    # 3. spawn_* atoms per task_board block via adapter dict
    tb = scene["task_board"]
    self._cmd_spawn_task_board_base(**tb["pose"])
    for i in range(5):
        self._dispatch_nic_rail(i, scene.get(f"nic_rail_{i}"))
    for i in range(2):
        self._dispatch_sc_rail(i, scene.get(f"sc_rail_{i}"))
        self._dispatch_lc_mount(i, scene.get(f"lc_mount_rail_{i}"))
        self._dispatch_sfp_mount(i, scene.get(f"sfp_mount_rail_{i}"))
        self._dispatch_sc_mount(i, scene.get(f"sc_mount_rail_{i}"))
    # 4. parity publishers (always on)
    self._start_aic_parity_publishers()
    # 5. controller loop (always on)
    self._start_aic_controller_loop()
    # 6. scoring publishers (gated on ground_truth)
    if ground_truth:
        self._start_aic_scoring_publishers()
    # 7. play
    self._cmd_play_scene()
    return {"status": "success", "trial": trial_key, "ground_truth": ground_truth,
            "spawned_components": [...]}
```

**Adapter helpers** (`_dispatch_nic_rail`, `_dispatch_sc_rail`, etc.) translate YAML's `entity_present`/`entity_pose.translation`/`entity_pose.roll/pitch/yaw` to the atom's `present`/`translation`/`roll`/`pitch`/`yaw` kwargs. ~5 lines each.

**Landmines:**
- `load_robot` is `async`; must wrap with `run_coroutine` (existing pattern in `_cmd_load_robot`, extension.py:2885).
- `add_objects` (legacy default-spawn) should NOT be invoked here — `load_trial` is the new canonical scene populator and includes its own per-component dispatch. This is the only place where `quick_start` semantics diverge from `load_trial` semantics.
- `_start_aic_scoring_publishers` is currently called unconditionally inside `quick_start` (extension.py:1115). Phase 4 D-04 asks it be gated on `ground_truth` for `quick_start` too — Plan must edit `quick_start` signature.

### Deliverable 2 — `ground_truth` flag in `quick_start` (TRIAL-02)

**Files modified:** `exts/aic-dt/aic_dt/extension.py`
- `MCP_TOOL_REGISTRY["quick_start"]` adds `"ground_truth": {"type": "bool", "default": true}` param
- `quick_start(self, ground_truth: bool = True)` signature update
- `_cmd_quick_start` forwards param
- Wrap line 1115 (`self._start_aic_scoring_publishers()`) in `if ground_truth:` block
- UI button: optionally add a checkbox; or accept that the UI button uses default `True` and the off path is MCP-only (simpler, recommended)

### Deliverable 3 — `run_aic_engine_against_isaac_sim.sh` (TRIAL-03)

**Files created:**
- `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh` — shell wrapper
- `exts/aic-dt/launch/aic_engine_against_isaac_sim.launch.py` — Python launch file (Option 4A)

**Wrapper logic:**
1. Parse args: `<trial_key>` `[--ground-truth=true|false]` `[--clean]`
2. Check Isaac Sim status via `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh status`
3. If not running: launch via the venv-activate + ROS_DOMAIN_ID=7 wrapper from CLAUDE.md
4. Wait until MCP socket on 8768 is reachable (poll with timeout)
5. Send `quick_start` (or `load_trial(trial_key, ground_truth)`) via MCP
6. Wait until ROS topics live on `ROS_DOMAIN_ID=7` (poll `/joint_states` via short rclpy timeout, max 30s)
7. Launch the engine: either
   - Host: `ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch <our launch file> trial_key:=$1 ...`
   - Docker (derived image): `docker run -d --net=host -e ROS_DOMAIN_ID=7 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp my-eval-isaac:v1 ...`
8. Launch the model: `docker run -d --net=host -e ROS_DOMAIN_ID=7 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp my-solution:v1 --ros-args -p policy:=aic_example_policies.ros.CheatCode -p use_sim_time:=true`
9. Tail the engine log until `Engine Stopped` or `process has finished cleanly` (mirrors `run_cheatcode.sh:109-117`)
10. Tear down: `docker stop aic_model` + (engine container if used). Leave Isaac Sim running unless `--clean`.

**Critical verification step before plan execution:** run `docker run --rm my-solution:v1 ls /opt/ros/kilted/lib/librmw_fastrtps_cpp.so` (or whatever ros version the image uses; per `docker/aic_model/Dockerfile` line 1: `ros:kilted-ros-core` — kilted, not humble). If absent, plan adds an `apt-get install ros-kilted-rmw-fastrtps-cpp` step in a derived image.

**Note:** the model container uses `ros:kilted` (per Dockerfile), not humble. The eval container also uses `ros:kilted`. Isaac Sim's `isaacsim.ros2.bridge` ships humble. **CONCERN:** humble ↔ kilted DDS interop — they SHOULD be compatible at the wire level (both speak DDS-XRCE/RTPS), but ROS 2 best-practice is to match minor versions when message types are involved. **[ASSUMED]** that the AIC msg packages (`aic_control_interfaces`, `ros_gz_interfaces`) compiled against kilted will be wire-compatible with humble subscribers. **VERIFY in Plan execution before writing the wrapper:** start aic_eval, start a humble-compiled `ros2 topic echo /joint_states`, confirm the messages decode. If not compatible, this is a hard blocker — escalate to user.

### Deliverable 4 — `parity_report.py` (TRIAL-04, TRIAL-05)

**File created:** `exts/aic-dt/scripts/parity_report.py`

**Pseudocode:**
```python
#!/usr/bin/env python3
# Reference: built per Phase 4 Plan / TRIAL-04+05 / CONTEXT D-08+09
"""Run sample_config.yaml trials against Isaac Sim and Gazebo. Emit parity report."""
import yaml, json, subprocess, time, os
from pathlib import Path

CONFIG = os.path.expanduser("~/Documents/aic/aic_engine/config/sample_config.yaml")
OUTDIR = Path(__file__).resolve().parents[3] / ".planning/phases/04-trial-loader"

def run_isaac_sim_trial(trial_key: str) -> dict:
    # 1. Ensure Isaac Sim is up
    # 2. Send load_trial(trial_key) via MCP
    # 3. Start aic_engine + model containers via run_aic_engine_against_isaac_sim.sh trial_key
    # 4. Subscribe to /scoring/insertion_event + /aic/gazebo/contacts/off_limit, capture
    # 5. Wait until aic_engine exits OR time_limit elapsed
    # 6. Tear down model + engine; leave Isaac Sim running
    return outcome

def run_gazebo_trial(trial_key: str) -> dict:
    # 1. Start aic_eval + my-solution via run_cheatcode.sh (with config_path override if needed)
    # 2. Subscribe to topics
    # 3. Wait until exit
    # 4. Tear down
    return outcome

def main():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    rows = []
    for trial_key in sorted(cfg["trials"]):
        rows.append(run_isaac_sim_trial(trial_key))
        rows.append(run_gazebo_trial(trial_key))
    # Emit JSON
    (OUTDIR / "parity-report.json").write_text(json.dumps(rows, indent=2))
    # Emit markdown
    md = render_markdown(rows)
    (OUTDIR / "parity-report.md").write_text(md)
    # Exit code: 0 if all rows match across both sims; 1 if any mismatch
    mismatches = count_mismatches(rows)
    return 0 if mismatches == 0 else 1
```

**Markdown render template:**
```markdown
# Parity Report — sample_config.yaml × {Isaac Sim, Gazebo}

| Trial | Sim | insertion_event | off_limit | duration_s | pass/fail |
|---|---|---|---|---|---|
| trial_1 | Isaac | ✓ | 0 | 38.2 | PASS |
| trial_1 | Gazebo | ✓ | 0 | 41.3 | PASS |
| ... |

**Mismatches:** 0
**M1 ship gate:** PASS — all 3 trials produce the same pass/fail outcome across both sims.
```

**Single-sim mode flag:** `--sim isaac|gazebo|both` (default `both`) — useful for fix loops where only one sim is being debugged.

### Deliverable 5 — DX-05 README + CHANGELOG (DX-05)

**Files modified:**
- `exts/aic-dt/docs/README.md` — full rewrite per Q7
- `exts/aic-dt/docs/CHANGELOG.md` — append Phase 3 entry (currently absent — verified by tail of file shows Phase 1/Phase 2 only) + Phase 4 / M1 entry

CHANGELOG sections per the project pattern (verified by reading the existing Phase 1 entry):
```markdown
## [Phase 4 — M1 Ship: Trial Loader & End-to-End Parity] — 2026-05-XX

Milestone 1 (M1) ships. Every trial in ~/Documents/aic/aic_engine/config/sample_config.yaml
runs end-to-end under aic_example_policies.ros.CheatCode against this Isaac Sim digital twin
and produces the same per-trial pass/fail outcome that Gazebo produces.

Requirement IDs delivered: TRIAL-01, TRIAL-02, TRIAL-03, TRIAL-04, TRIAL-05, DX-05,
PARITY-07 (live insertion_event fire verified incidentally via TRIAL-04).

### Added
- load_trial MCP atom: ...
- ground_truth flag on quick_start + load_trial: ...
- E2E wrapper script run_aic_engine_against_isaac_sim.sh: ...
- aic_engine_against_isaac_sim.launch.py: ...
- parity_report.py: dual JSON+MD output across all 3 trials × {Isaac, Gazebo}.

### Changed
- exts/aic-dt/docs/README.md rewritten — removed "ur5e-dt" framing, reflects AIC scope.

### Carry-forwards beyond M1
- M2 pose-source swap (POSE-01..03)
- ...
```

## Common Pitfalls

### Pitfall 1: `aic_gz_bringup.launch.py` cannot be made to skip Gazebo
**What goes wrong:** Plan tries `gazebo_gui:=false spawn_task_board:=false spawn_cable:=false` and finds the launch hangs because gzserver still runs and `controller_manager` waits for `gz_ros2_control` plugin.
**Why it happens:** Several Nodes in `nodes_to_start` have no `condition=` — they always launch.
**How to avoid:** Author a NEW launch file in aic-dt instead of trying to call the AIC repo's bringup.
**Warning signs:** `[ros2_control]: error: Could not find resource '/controller_manager'`; `[INFO] [launch]: Waiting for controller manager...` indefinite.

### Pitfall 2: ROS 2 distro mismatch (Isaac Sim humble vs aic_eval kilted)
**What goes wrong:** Custom message types (`aic_control_interfaces.msg.JointMotionUpdate`) generated against kilted may not be ABI-compatible with humble subscribers.
**Why it happens:** rosidl C ABI changes across ROS 2 minor versions; Phase 2 already hit this with a Python 3.11 vs 3.12 ABI gap (Plan 02-01).
**How to avoid:** **Verify with smoke test BEFORE writing the wrapper.** Spin up `aic_eval`, `ros2 topic echo /joint_states` from the host humble env (already vendored in `~/IsaacSim-ros_workspaces/humble_ws`), confirm decode. If failure, halt + escalate.
**Warning signs:** `Could not import 'rosidl_typesupport_c' for package 'aic_control_interfaces'`; cryptic deserialization warnings on the host side.

### Pitfall 3: Insertion event not firing in Isaac Sim
**What goes wrong:** Trial runs to completion, CheatCode lifts plug → port → insertion happens visually, but `/scoring/insertion_event` never fires. Parity report shows `PASS|FAIL` mismatch.
**Why it happens:** `_PORT_LINK_PATHS` in scoring_publishers.py is `["/World/TaskBoard/sc_port_1", "sc_port_2", "nic_card"]` — but trials 1+2 use `target_module_name="nic_card_mount_<i>"`. The hardcoded path `/World/TaskBoard/nic_card` may not match the actual prim path after spawn_nic_card_mount runs.
**How to avoid:** Plan an early `ros2 topic echo /scoring/insertion_event` smoke during trial-1 dry-run. If it doesn't fire after CheatCode completes its insert motion, switch `_PORT_LINK_PATHS` to look up the correct prim paths at trial-load time (D-13 fallback path: add a `set_port_link_paths` MCP atom OR have `load_trial` call into scoring_publishers to update the list).
**Warning signs:** `[AIC-DT][scoring] insertion contact subscription wired (1/4 prims tagged with PhysxContactReportAPI)` (= only the plug end was tagged; ports failed lookup).

### Pitfall 4: MCP socket not framed
**What goes wrong:** Wrapper sends `load_trial` via MCP but the response parsing fails because of mid-buffer parse attempts.
**Why it happens:** MCP socket protocol per CLAUDE.md: "TCP, send a single JSON object, receive a single JSON object. No length prefix, no newline framing — server `json.loads` the buffer until it parses successfully."
**How to avoid:** Use the exact recv loop from CLAUDE.md (recv until `json.loads` succeeds). DON'T use a single recv() — large payloads from `load_trial` (with spawned-components list) can exceed 16KB.
**Warning signs:** `json.JSONDecodeError: Unterminated string at position N`.

### Pitfall 5: ros_gz_interfaces.msg.Contacts type mismatch
**What goes wrong:** Parity-report subscriber gets `Contacts` from Isaac Sim but the message has 0 fields when echoed.
**Why it happens:** Phase 2's `Contacts` publisher uses fields filled in by the Isaac Sim physx callback; the Gazebo container publishes via `ros_gz_bridge` with potentially different field semantics. Phase 1 already noted "no `_sim`/`_real` discrimination" — they SHOULD match.
**How to avoid:** Verify field shape in both sims during dry-run. The condition `len(msg.contacts) > 0` is the relevant signal; field-level parity is not required for pass/fail.

### Pitfall 6: Cache state on Isaac Sim cold start
**What goes wrong:** Wrapper launches Isaac Sim cold, `load_trial` hangs on `load_robot` for 5+ minutes (cooking deadlock per CLAUDE.md).
**Why it happens:** Empty `DerivedDataCache`. Per CLAUDE.md, this is the single biggest cause of `load_robot` hang.
**How to avoid:** Wrapper script invokes `prime_usd_cache.py status` first; if empty, runs `prime_usd_cache.py restore known-good`.
**Warning signs:** Kit log goes silent for 30+ seconds while still showing the postload script's last `[AIC-DT]` line.

## Code Examples

### Example 1: Minimal MCP load_trial invocation from a wrapper script

```python
# Reference: extension.py:142 MCP_TOOL_REGISTRY + CLAUDE.md MCP socket protocol
import socket, json, time

def mcp_send(host: str, port: int, type_: str, params: dict, timeout_s: float = 120):
    s = socket.socket(); s.settimeout(timeout_s)
    s.connect((host, port))
    s.sendall(json.dumps({"type": type_, "params": params}).encode())
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk: break
        buf += chunk
        try: return json.loads(buf.decode())
        except json.JSONDecodeError: continue
    s.close()
    raise RuntimeError("MCP socket closed before complete JSON response")

# Usage:
result = mcp_send("127.0.0.1", 8768, "load_trial",
                  {"trial_key": "trial_1", "ground_truth": True})
assert result["result"]["status"] == "success", result
```

### Example 2: Bounded rclpy subscriber for parity_report

See Q3 above for full pattern. Key invariant: `rclpy.spin_once(node, timeout_sec=0.1)` in a deadline loop, NOT `rclpy.spin(node)` (would block).

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|---|---|---|---|
| `add_objects` hardcoded 4-prim spawn | Per-component spawn atoms (Plan 01-09) | Phase 1 | `load_trial` builds on the new atoms; `add_objects` retained as legacy clubbing path |
| `objects_poses_sim` / `sync_real_poses` MCP atoms | Removed (DX-01) | Phase 1 | Topic-name purity; trial loader uses Gazebo names verbatim |
| `_start_aic_scoring_publishers` always-on inside quick_start | Gated on `ground_truth` kwarg (Phase 4 D-04) | Phase 4 | M2 pose-source swap surface preserved |
| `aic_gz_bringup.launch.py` consumed wholesale | Custom launch file in aic-dt that mirrors structure but skips Gazebo | Phase 4 | Required because no `start_gazebo:=false` flag exists upstream |

## Validation Architecture

> Note: `.planning/config.json`'s `workflow.nyquist_validation` is not explicitly set; default is enabled. However, this project's "verification = run aic_engine + CheatCode against the trial" overrides per-component test suites (see Phase 3 SUMMARY: "Phase 4 E2E trial verification supersedes [phase smoke tests]"). Below is the minimal test plan.

### Test Framework
| Property | Value |
|---|---|
| Framework | None for Python unit tests (project pattern is end-to-end smoke tests via `~/env_isaaclab/bin/python` invocations + shell scripts). Existing pattern: `smoke_test_aic_*.py` modules invoked from `verify_phase_<N>.sh`. |
| Quick run | `parity_report.py --sim isaac --trial trial_1` (dry-run single trial) |
| Full suite | `parity_report.py` (all 3 trials × both sims) |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|---|---|---|---|---|
| TRIAL-01 | load_trial spawns matching scene from YAML | smoke | MCP `load_trial(trial_key="trial_1")` returns success + viewport shows expected prims | ❌ Wave 0 (need parity_report dry-run mode OR new smoke script) |
| TRIAL-02 | ground_truth=False suppresses /scoring/* topics | smoke | `ros2 topic list \| grep /scoring/` returns 0 lines after `load_trial(..., ground_truth=False)` | ❌ Wave 0 |
| TRIAL-03 | aic_engine runs unmodified against Isaac Sim | E2E | `run_aic_engine_against_isaac_sim.sh trial_1` — engine exits cleanly | ❌ Wave 0 (this is the wrapper itself) |
| TRIAL-04 | All 3 trials produce same pass/fail across both sims | E2E | `parity_report.py` exits 0 | ❌ Wave 0 |
| TRIAL-05 | Parity-report is canonical M1 gate | meta | `parity_report.py` emits both .json and .md | ❌ Wave 0 |
| DX-05 | README + CHANGELOG updated | static | `grep -c "ur5e-dt" exts/aic-dt/docs/README.md` returns 0; CHANGELOG has Phase-4 / M1 section | trivial |

### Sampling Rate
- **Per task commit:** parse-check (`python3 -c "import ast; ast.parse(...)"`) + dry-run if cheap.
- **Per wave merge:** parity_report dry-run (single trial, single sim) — verifies the whole pipeline in <2 min.
- **Phase gate:** Full parity_report run — 3 trials × 2 sims × 180s ≤ 18 min wall time. Zero mismatch rows = M1 ship.

### Wave 0 Gaps
- [ ] `exts/aic-dt/scripts/parity_report.py` — the canonical TRIAL-04+05 verifier
- [ ] `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh` — TRIAL-03 wrapper
- [ ] `exts/aic-dt/launch/aic_engine_against_isaac_sim.launch.py` — engine-only launch (Option 4A)

## Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| `aic_gz_bringup.launch.py` can't skip Gazebo (Q4 finding) | **Confirmed** | High — blocks D-06 as written | Author Option 4A launch file in aic-dt; DON'T modify AIC repo |
| ros-kilted (engine container) ↔ ros-humble (Isaac Sim bridge) RMW interop fails | Medium | High — blocks E2E entirely | Smoke-verify before wrapper authoring (`ros2 topic echo /joint_states` from humble host into kilted-published topics). If incompatible, build derived eval image with humble base. |
| `_PORT_LINK_PATHS` hardcoded list misses trial-1+2 nic_card paths (Pitfall 3) | Medium | Medium — TRIAL-04 mismatch | Plan adds an early dry-run smoke + falls back to D-13's `set_port_link_paths` MCP atom if needed |
| Cold cache on Isaac Sim launch within wrapper hangs `load_trial` | Low (warm-cache discipline in CLAUDE.md) | High — wrapper hangs indefinitely | Wrapper invokes `prime_usd_cache.py status` + `restore known-good` before launch. Snapshot after first successful trial. |
| Phase 1 spawn atoms don't accept `entity_name` (Q5 finding) | Confirmed | Low — informational only | Adapter ignores `entity_name`; aic_engine consumes its own copy of the YAML |
| gripper_offset semantic mismatch (Q2) | Low | Low — value range is small | Option A pass-through; defer Option B to M2 if a trial mismatches |
| MCP socket framing pitfall on large `load_trial` response | Low | Medium — wrapper hangs on recv | Use the recv-until-parse pattern from CLAUDE.md verbatim |
| PARITY-07 live insertion_event has never fired in any session | Medium | High — this is the M1 gate signal | First Plan 04 task is a dry-run smoke `(load_trial trial_1) → CheatCode → ros2 topic echo /scoring/insertion_event`. If it doesn't fire, debug-loop OWNED by Phase 4 (per Phase 3 D-12). Pitfall 3 covers the most likely cause. |
| zenoh router collision when both `aic_eval` (Zenoh) and Isaac Sim (DDS) run on same host | Low | Medium — port 7447 conflict | parity_report runs Isaac and Gazebo trials **sequentially**, not in parallel; tear down between. |
| Time-limit per trial (180s in YAML) is too short for cold start | Low | Low — warm cache is fast (5s) | Use `time_limit + 30s` (warmup) as the parity_report ceiling; trial starts only after Isaac Sim ready and topics live. |

## Skill consultation log

- **`isaac-sim-extension-dev`** consulted via cited contents: extension lifecycle (4-surface DX-02 contract), MCP socket protocol (no framing — read until parse succeeds), cache discipline (snapshot-and-restore is the operating model), launch reality (two paths: lifecycle helper + postload), real Kit log location (`~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log`).
- **`nvidia-suite-docs`** NOT directly invoked — Phase 4 doesn't reach into Isaac Sim API surface beyond what Phases 1-3 already wired (rclpy publishers/subscribers, USD authoring through existing helpers, omni.physx contact reports through existing scoring_publishers). All Isaac-Sim-side surface is reuse.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|---|---|---|
| A1 | `rmw_fastrtps_cpp` is available in `my-solution:v1` Docker image | Q4 | Wrapper would need a derived image; +30 min Plan time |
| A2 | ros-kilted (engine container) ↔ ros-humble (Isaac Sim bridge) RMW interop is wire-compatible for the AIC msg packages | Q4, Risk table | Blocking — would require rebuilding the engine in a humble base. Plan task #1 is a smoke test for this. |
| A3 | gripper_offset 1.5cm-4.5cm range is small enough that pass-through to `cable_x/y/z` produces equivalent runtime behavior to Gazebo's authored attach offset | Q2 (Option A) | Low — TRIAL-04 mismatch on a single trial; fix-loop swaps to Option B |
| A4 | Phase 3's hardcoded `_PORT_LINK_PATHS` covers trial 3 (`/World/TaskBoard/sc_port_1`) — does NOT cover trial 1+2 (`nic_card_mount_<i>`) cleanly | Pitfall 3, Risk table | Medium — TRIAL-04 mismatch on trials 1+2 specifically; D-13 fallback owns the fix |
| A5 | MCP_TOOL_REGISTRY single-trial-key + ground_truth surface is sufficient for all 3 sample_config.yaml trials (no future trial fields needed) | Q2 | Low — sample_config.yaml is read-only AIC contract; new fields would be a new milestone |
| A6 | `aic_engine` runs from a host environment with kilted-or-humble overlay can find aic_engine + aic_adapter binaries (because `apt install ros-<distro>-aic-engine` is unlikely; the engine compiles only inside Docker per host GLIBC constraint) | Deliverable 3 | High — the host can't host `aic_engine` natively per CLAUDE.md's pixi note → forces Option 4B (derived Docker image) for the engine. Plan should commit to Option 4B from the outset. |

**Note on A6:** This is a firm reason to favor Option 4B (Docker derived image) over Option 4A (host launch file). Plan 04 should derive `my-eval-isaac:v1` from `aic_eval`, override `/entrypoint.sh` to run only `robot_state_publisher` + `aic_adapter` + `aic_engine` (skipping Gazebo + ros2_control + ros_gz_bridge), and run that container with `--net=host -e ROS_DOMAIN_ID=7 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. **Revised recommendation:** Option 4B over 4A.

## Open Questions

1. **Will ros-kilted (engine container) and ros-humble (Isaac Sim bridge) interop?**
   - What we know: Both speak DDS; AIC's `aic_control_interfaces` packages compile against kilted in the eval image.
   - What's unclear: Wire-level decode of kilted-compiled `JointMotionUpdate` from a humble subscriber, and vice versa.
   - Recommendation: First Plan 04 task is a smoke harness (`bash exts/aic-dt/scripts/verify_kilted_humble_interop.sh`) that runs aic_eval, has a host-side humble subscriber on `/joint_states`, confirms decode. Block plan execution if it fails.

2. **Does PARITY-07's `_PORT_LINK_PATHS` cover the prim paths created by `spawn_nic_card_mount(index)` for trials 1+2?**
   - What we know: Phase 1 atoms author at `/World/TaskBoard/<some-name>_<index>`. The exact name is not extracted in this research session — the extension.py `_cmd_spawn_nic_card_mount` body would need inspection.
   - What's unclear: Whether spawn_nic_card_mount creates `/World/TaskBoard/nic_card` (matching `_PORT_LINK_PATHS`) or `/World/TaskBoard/NICCardMount_<index>` (NOT matching).
   - Recommendation: Plan 04's first dry-run after `load_trial trial_1` greps the live USD prim tree (`execute_python_code` MCP atom) for `/World/TaskBoard/*` to confirm. If mismatch, D-13 fallback is invoked.

3. **Is `time_limit: 180` per trial a hard cap from CheatCode's perspective, or just an aic_engine config?**
   - What we know: aic_engine reads it from sample_config.yaml; CheatCode is a passing policy that completes well before 180s in Gazebo.
   - What's unclear: Whether parity_report should use 180s + 30s warmup (= 3.5 min × 6 trials = 21 min budget) or a tighter 60s ceiling.
   - Recommendation: Use `time_limit + 30` to be safe; first dry-run reveals actual completion time and we can tighten.

## Sources

### Primary (HIGH confidence)
- `~/Documents/aic/aic_engine/config/sample_config.yaml` — direct read, all 3 trials × 16 fields enumerated above
- `~/Documents/aic/scripts/run_cheatcode.sh` — direct read, exact docker run invocations
- `~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py` — direct read, conditional launch behaviour confirmed (`gzserver` unconditional)
- `~/Documents/aic/docker/aic_eval/Dockerfile` — direct read, Zenoh-based RMW + entrypoint launch invocation
- `~/Documents/aic/docker/aic_model/Dockerfile` — direct read, base image is `ros:kilted-ros-core`
- `~/Documents/aic/docker/docker-compose.yaml` — direct read, internal: true network + RMW config
- `exts/aic-dt/aic_dt/extension.py` — direct read, MCP_TOOL_REGISTRY contents + `_cmd_spawn_*` signatures
- `exts/aic-dt/aic_dt/scoring_publishers.py` — direct read, `_PORT_LINK_PATHS` hardcoded list + topic publisher signatures
- `exts/aic-dt/docs/README.md` — direct read, confirms full-rewrite scope of DX-05
- `exts/aic-dt/docs/CHANGELOG.md` — direct read, current state shows Phase 1 + Phase 2; Phase 3 + Phase 4 entries needed
- `.planning/REQUIREMENTS.md` — direct read for TRIAL-01..05 + DX-05 traceability
- `.planning/phases/04-trial-loader/04-CONTEXT.md` — direct read, all 13 D-decisions extracted verbatim
- `.planning/phases/03-cable-physics/03-SUMMARY.md` — direct read for PARITY-07 carry-forward and `_PORT_LINK_PATHS` deviation note
- `.planning/phases/01-foundation-parity/01-09-PLAN.md` — direct read for spawn-atom signature contract
- `CLAUDE.md` (repo root) — direct read for autonomous M1 mode, two-launcher reality, ROS_DOMAIN_ID=7, MCP socket protocol, cache discipline

### Secondary (MEDIUM confidence)
- `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` — referenced for project patterns; no direct quotation in this session
- ROS 2 Humble rclpy subscriber pattern — well-documented; Phase 1+2+3 publishers already use it

### Tertiary (LOW confidence — flagged for verification)
- A1 (rmw_fastrtps_cpp in my-solution:v1) — not verified by `docker run --rm`
- A2 (kilted ↔ humble RMW interop) — assumed compatible at wire level; first Plan task should smoke-verify

## Metadata

**Confidence breakdown:**
- YAML schema audit: HIGH — direct read, all 3 trials × 16 fields confirmed
- Spawn-atom signature mapping: HIGH — direct read of MCP_TOOL_REGISTRY
- Docker / Zenoh / DDS interop: MEDIUM — confirmed structural blocker (gzserver unconditional), assumed compat at wire level
- rclpy subscriber pattern for parity_report: HIGH — established by Phase 1-3 publishers
- DX-05 README rewrite scope: HIGH — direct read shows wholesale change needed
- PARITY-07 live insertion_event firing: LOW — never exercised in any session; first Plan 04 dry-run is the verification

**Research date:** 2026-05-05
**Valid until:** 2026-05-12 (Phase 4 should ship within the week; AIC repo is upstream-tracked but unlikely to change `sample_config.yaml` schema mid-week)

---

*Phase: 04-trial-loader*
*Researched: 2026-05-05 under autonomous M1 mode*
