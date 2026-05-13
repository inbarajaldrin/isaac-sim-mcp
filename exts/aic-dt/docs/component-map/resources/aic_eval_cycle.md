# AIC Evaluation Submission Contract — Research Report

Source repo (read-only): `/home/aaugus11/Documents/aic` (commit at time of reading)

---

## 1. Single Evaluation Cycle — End-to-End

### Entry point and outer loop
- Binary: `aic_engine` (C++). `main()` at `aic_engine/src/main.cpp:24` — calls `engine->start()` then `rclcpp::shutdown()`. Exit code = 0 if final state is `EngineState::Completed`, else 1.
- `Engine::start()` → `initialize()` → `run()` (`aic_engine.cpp:348`). `run()` (`aic_engine.cpp:567`) iterates over **every trial** parsed from the YAML and calls `handle_trial(trial)`.
- After every trial: `cleanup_model_node()` + `shutdown_model_node()` + `validate_model_shutdown()` + `score_run(...)` write `${HOME}/aic_results/scoring.yaml` (or `${AIC_RESULTS_DIR}/scoring.yaml`).

### Single-trial state machine (`Engine::handle_trial`, `aic_engine.cpp:682`)
```
Uninitialized
  → check_model()              [discover lifecycle node + transition to ACTIVE]
ModelReady
  → check_endpoints()          [adapter node + required scoring topics + spawn service]
EndpointsReady
  → ready_simulator(trial)     [spawn task_board + cable via /gz_server/spawn_entity]
SimulatorReady
  → ready_scoring(trial)       [open rosbag, subscribe to scoring topics, wait for TFs]
ScoringReady
  → tasks_started(trial)       [send InsertCable goal to /insert_cable action server]
TasksExecuting
  → tasks_completed_successfully(trial)
AllTasksCompleted             → score_trial() → returns TrialScore
```
Each non-terminal step has up to **5 retries** (`MAX_RETRIES = 5`, `aic_engine.cpp:687`). A failure at any step makes the engine: `reset_after_trial()` → `engine_state_ = Error` → return the (largely zero) `TrialScore`. The remaining trials are still attempted because the outer loop in `run()` only breaks on `engine_state_ == Error` *after* recording the breakdown.

### Trial timeout
Per-task. Set by **`time_limit` (seconds) inside each task block in the YAML** — `sample_config.yaml` uses 180s for every task. In code: `aic_engine.cpp:1394-1402` —
```cpp
if (!wait_for_interruptible(result_future,
                            std::chrono::seconds(task.time_limit))) {
  insert_cable_action_client_->async_cancel_goal(goal_handle);
  current_attempt.state = TaskState::TimeLimitExceeded;
  return false;
}
```
The scoring tier-2 bag recorder is sized to `max_task_limit + 5` seconds (`aic_engine.cpp:1325`).

### Trial pass/fail signal
- **Pass** = action result `result.result->success == true` returned from `/insert_cable` action before the time limit expires.
- **Fail** = any of: timeout, action goal rejected, action returned `success=false`, or any pre-task readiness check failing all 5 retries.
- The *primary* downstream signal that the trial succeeded is whether `/scoring/insertion_event` was published by the Gazebo cable plugin with `namespace == "<target_module_name>/<port_name>"` — this drives the Tier-3 75-point bonus regardless of what the action handler said (`ScoringTier2.cc:825-848`).

### Where the score is computed
- `Engine::score_trial()` → `aic_engine.cpp:1995` calls `ScoringTier2::ComputeScore()`.
- `ScoringTier2::ComputeScore()` → `aic_scoring/src/ScoringTier2.cc:181`. Reads the rosbag back, then computes:
  - `GetInsertionForceScore()` — `ScoringTier2.cc:874`
  - `GetContactsScore()` — `ScoringTier2.cc:922`
  - `ComputeTier3Score()` — `ScoringTier2.cc:813` (binary insertion + partial-insertion fallback via `GetDistanceScore()` at line 698)
  - `GetTaskDurationScore()` — `ScoringTier2.cc:941`
  - `GetTrajectoryJerkScore()` — `ScoringTier2.cc:543`
  - `GetTrajectoryEfficiencyScore()` — `ScoringTier2.cc:655`
- Tier-1 is set in `handle_trial` immediately after `ModelReady` (`aic_engine.cpp:706` — `score.tier_1_success()`). Tier-1 has no separate scorer; the binary `aic_scoring/src/ScoringTier1.cc` is a standalone topic-rate validator not wired into the engine for qualification trials.
- Final write: `Engine::score_run()` → `aic_engine.cpp:2009` — writes `<scoring_output_dir>/scoring.yaml`. `scoring_output_dir` defaults to `$HOME/aic_results` (overridable via `AIC_RESULTS_DIR`).

---

## 2. Input Parameters / CLI

### `aic_engine` CLI = ROS 2 parameters
There is no `argparse`-style flag set; everything is a ROS 2 parameter declared in `Engine::Engine()` (`aic_engine.cpp:300-320`). Pass with `--ros-args -p name:=value`. The full list:

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `config_file_path` | string | `""` | Path to the trial YAML (e.g. `sample_config.yaml`) — **required**, no default |
| `adapter_node_name` | string | `"aic_adapter_node"` | Required-node name for `check_endpoints` |
| `model_node_name` | string | `"aic_model"` | Lifecycle-node name to discover + transition |
| `gripper_frame_name` | string | `"gripper/tcp"` | TF frame used as end-effector for cable-spawn pose + scoring |
| `endpoint_ready_timeout_seconds` | int | `10` | Per-attempt timeout for `check_endpoints` |
| `model_discovery_timeout_seconds` | int | `30` | How long to wait for the lifecycle node to appear |
| `model_configure_timeout_seconds` | int | `60` | Lifecycle transition timeout |
| `model_activate_timeout_seconds` | int | `60` | ditto |
| `model_deactivate_timeout_seconds` | int | `60` | ditto |
| `model_cleanup_timeout_seconds` | int | `60` | ditto |
| `model_shutdown_timeout_seconds` | int | `60` | ditto |
| `ground_truth` | bool | `false` | **When true**, enables ground-truth `/scoring/tf` → `/tf` relay + xacro `ground_truth:=true` for the task board. See §4. |
| `skip_model_ready` | bool | `false` | Test-only: bypass lifecycle dance |
| `skip_ready_simulator` | bool | `false` | Test-only: skip spawn/delete |

Environment: `AIC_RESULTS_DIR` (output path), `HOME` (fallback for output path).

### `sample_config.yaml` top-level keys (consumed by engine)

| Key | Consumer | Meaning |
|---|---|---|
| `scoring.topics[]` | `ScoringTier2::ParseStats` (`ScoringTier2.cc:303`) | List of `{name, type, latched?}` topics the engine **subscribes to + records into the rosbag** during each trial. Drives `check_endpoints` (`get_publisher_count > 0`) and the bag deserialization in `ComputeScore`. |
| `task_board_limits.{nic_rail,sc_rail,mount_rail}.{min_translation,max_translation}` | `Engine::spawn_entity` (`aic_engine.cpp:1799-1813`) | Clamps for per-rail entity translations passed to the task-board xacro |
| `trials.<trial_id>.scene.task_board.pose.{x,y,z,roll,pitch,yaw}` | `ready_simulator` (`aic_engine.cpp:1175-1180`) | World-frame pose for spawning the task board |
| `trials.<trial_id>.scene.task_board.{nic_rail_0..4, sc_rail_0..1, lc_mount_rail_0..1, sfp_mount_rail_0..1, sc_mount_rail_0..1}` | `Engine::spawn_entity` task-board branch | Per-rail `entity_present`, `entity_name`, `entity_pose.{translation,roll,pitch,yaw}` → xacro args |
| `trials.<trial_id>.scene.cables.<cable_id>.{pose.gripper_offset.{x,y,z}, pose.{roll,pitch,yaw}, attach_cable_to_gripper, cable_type}` | `ready_simulator` cable spawn (`aic_engine.cpp:1216-1247`) | Cable initial pose **relative to current gripper TF** plus xacro args |
| `trials.<trial_id>.tasks.<task_id>.{cable_type, cable_name, plug_type, plug_name, port_type, port_name, target_module_name, time_limit}` | `Trial` ctor → packed into `aic_task_interfaces/msg/Task` → `InsertCable.Goal` (`aic_engine.cpp:236-248, 1360-1366`) | Per-task identity + per-task timeout |
| `robot.home_joint_positions.<joint_name>: <radians>` | `initialize` builds homing message + `ResetJoints` request (`aic_engine.cpp:506-535`) | Reset joint angles between trials. Must have exactly 6 entries. |

### Relationship: YAML vs launch args vs engine params
- The eval container launch file (`aic_bringup/launch/aic_gz_bringup.launch.py`) takes its own arg surface — `gazebo_gui`, `launch_rviz`, `start_aic_engine`, `ground_truth`, `shutdown_on_aic_engine_exit`, `model_discovery_timeout_seconds`, etc. — and forwards the relevant ones to `aic_engine` as ROS params.
- `config_file_path` is hard-set inside the launch file to `aic_engine`'s installed `sample_config.yaml` unless overridden. The YAML defines **scene layout, task identity, time limits, scoring topic list, home joints**; everything else (timeouts, ground-truth toggle, node names) lives on the engine CLI as ROS params.
- `docker-compose.yaml` line 9 is the canonical local invocation: `gazebo_gui:=false launch_rviz:=false ground_truth:=false start_aic_engine:=true shutdown_on_aic_engine_exit:=true model_discovery_timeout_seconds:=600`.

---

## 3. Policy Interface (What a Submitted Solution Must Expose)

### Engine → Policy (what the engine publishes / serves)
The submitted policy / `aic_model` node **does not subscribe to** any of these directly — the `aic_adapter` node aggregates them into the `Observation` message. The raw topic surface is (per `sample_config.yaml` + the launch file):

| Topic | Type | Direction | Role |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | engine + adapter consume | Joint positions / velocities. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | engine + adapter consume; ground-truth-only frames relayed from `/scoring/tf` | TF tree. **In ground-truth mode only**, cable + task-board frames are republished here. |
| `/scoring/tf` | `tf2_msgs/TFMessage` | engine internal (relayed → `/tf` if `ground_truth:=true`) | Ground-truth poses for cable links + task-board parts |
| `/aic/gazebo/contacts/off_limit` | `ros_gz_interfaces/Contacts` | engine consumes for scoring | Triggers the −24-point off-limit penalty |
| `/fts_broadcaster/wrench` | `geometry_msgs/WrenchStamped` | engine + adapter consume | Wrist 6-axis F/T, used by force-penalty scoring + Observation |
| `/scoring/insertion_event` | `std_msgs/String` | engine consumes from Gazebo cable plugin | Fires when plug fully seats in port. Drives the 75-point Tier-3 bonus. |
| `/aic_controller/controller_state` | `aic_control_interfaces/ControllerState` | engine + adapter consume | Includes FTS tare offset + TCP velocity |
| `/insert_cable` (action `aic_task_interfaces/InsertCable`) | action **server** lives on `aic_model`; engine is the **client** | engine → policy | One trial-start signal per task. Goal = `aic_task_interfaces/msg/Task` |
| `observations` | `aic_model_interfaces/Observation` | `aic_adapter` publishes at ~20 Hz; `aic_model` subscribes (`aic_model.py:99`) | The fused observation the policy actually reads |

### Policy → Engine (what the policy must publish/serve)
The policy framework (`aic_model` Python package) handles all of these for you if you derive from `aic_model.policy.Policy`. The framework owns:

| Topic / Service | Type | Direction | Notes |
|---|---|---|---|
| `/aic_controller/pose_commands` | `aic_control_interfaces/MotionUpdate` | policy → controller | Cartesian targets. Switching here calls `change_target_mode → MODE_CARTESIAN` (`aic_model.py:229-234`) |
| `/aic_controller/joint_commands` | `aic_control_interfaces/JointMotionUpdate` | policy → controller | Joint-space targets. Switches mode to `MODE_JOINT` |
| `/insert_cable` (action server) | `aic_task_interfaces/InsertCable` | policy advertises | Engine connects here. Policy invokes user's `insert_cable(task, get_observation, move_robot, send_feedback)` (`aic_model.py:275-283`) |
| `/aic_model/get_state`, `/aic_model/change_state` | `lifecycle_msgs/GetState, ChangeState` | engine → policy | Standard ROS 2 lifecycle |
| `/cancel_task` | `std_srvs/Empty` | optional | Provided to abort a running goal (`aic_model.py:94`) |
| `/aic_controller/change_target_mode` | `aic_control_interfaces/ChangeTargetMode` | policy → controller (called by framework) | Switches Cartesian / joint / unspecified mode before each command (`aic_model.py:122-123`) |

### Lifecycle contract — what the engine expects
`Engine::check_model()` (`aic_engine.cpp:1009`) drives:
1. Wait up to `model_discovery_timeout_seconds` for a node named `aic_model` (the `model_node_name` param) **and** for `/aic_model/get_state` service to exist (`aic_engine.cpp:1057`).
2. **On the first trial only:** call `/aic_model/get_state` and verify state == `PRIMARY_STATE_UNCONFIGURED` AND no publisher exists on `/aic_controller/{pose,joint}_commands` (`model_node_is_unconfigured`, line 895-945).
3. **On the first trial only:** transition `configure` (`aic_engine.cpp:949`). After configure: still no command publishes allowed AND the policy must **reject** any `InsertCable` goal sent (engine sends a probe goal; if accepted, it's a rules violation — line 966-998).
4. Every trial: transition `activate` (`aic_engine.cpp:1531`).
5. After every trial: `deactivate` (`reset_after_trial` line 1640).
6. After all trials: `cleanup` + `shutdown` + `validate_model_shutdown` (line 1587 — verifies the policy has no leftover command publishers within 2s).

### Start-trial / end-trial signaling
- **Start trial** = engine sends `InsertCable.Goal{ task: Task }` via `insert_cable_action_client_->async_send_goal(...)` (`aic_engine.cpp:1365`). Goal acceptance starts the wall-clock timer.
- **End trial** = either (a) action returns with `result.success` set + a `message`, OR (b) `time_limit` seconds elapse and the engine cancels the goal. In `aic_model.py:288-349` the framework runs the policy's `insert_cable()` in a background thread and returns `result.success = self._action_thread_result` when it exits.

---

## 4. Ground-Truth vs Estimated Pose — The Key Seam

### Where CheatCode reads ground-truth port pose
`aic_example_policies/aic_example_policies/ros/CheatCode.py:209-227`:
```python
port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"
...
port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
    "base_link", port_frame, Time())
```
The TF chain `task_board/<target_module_name>/<port_name>_link` and `<cable_name>/<plug_name>_link` **only exists on `/tf` when `ground_truth:=true`**. CheatCode's `_wait_for_tf` even prints "are you running eval with `ground_truth:=true`?" (line 63) when the lookup times out.

### What replaces it for a real submission
A real policy reads the **`Observation` message** delivered at ~20 Hz via `get_observation()` and computes the port pose itself, typically from:
- `observation.left_image / center_image / right_image` (`sensor_msgs/Image` from three wrist cameras) + corresponding `*_camera_info`.
- `observation.joint_states` to compute the camera extrinsic relative to `base_link` via the URDF.
- `observation.wrist_wrench` + `observation.controller_state.fts_tare_offset.wrench` for contact-confirmed seating.

The CV pose-estimation stack plugs in **inside the policy's `insert_cable()` body**, replacing the `_parent_node._tf_buffer.lookup_transform("base_link", port_frame, ...)` call with an estimate computed from the three RGB camera streams. Everything downstream (motion targets, IK, etc.) stays identical.

### Where the `ground_truth` flag flips topology
- Engine constructor (`aic_engine.cpp:310`): declares the `ground_truth` ROS param (default `false`).
- Passed through to the task-board xacro at spawn (`aic_engine.cpp:1908`): `cmd << " ground_truth:=" << (ground_truth_ ? "true" : "false");` — the xacro uses it to decide whether to emit Gazebo plugins that publish the per-port TF frames onto `/scoring/tf`.
- Launch file (`aic_bringup/launch/aic_gz_bringup.launch.py:367-394`):
  - `ground_truth_tf_relay`: `topic_tools relay` from `/scoring/tf` → `/tf`. **Conditional on `IfCondition(ground_truth)`.**
  - `ground_truth_static_tf_publisher`: static TF `world` → `aic_world`. Conditional too.
- When `ground_truth:=false` (the **real submission mode**, which `docker-compose.yaml:9` sets):
  - The `/scoring/tf` topic still exists — Gazebo bridges it (see `ros_gz_bridge_config.yaml` lines 57-92) — and the engine still uses it for **its own scoring buffer** (`ScoringTier2.cc:118-124, 213-216`). So scoring is always done against ground truth.
  - But the relay → `/tf` is OFF, so the policy cannot see the cable / task-board pose frames on `/tf`.
  - The `aic_world` static TF is also missing, which would break any policy that tries to look up `aic_world` directly.

### Line between CheatCode-mode and real-submission mode
There is no "engine evaluates against the policy's own pose estimate" — **the engine always scores using ground-truth TFs from `/scoring/tf` recorded into the rosbag**, regardless of what the policy sees. The `ground_truth` flag only controls **whether the policy is given a free copy of those TFs on `/tf`**. In submission mode (`ground_truth:=false`), the policy must estimate poses; the scorer compares those estimates' downstream behavior (final plug position, insertion event) against the simulator's ground truth.

---

## 5. Scoring Tiers

Source: `aic_scoring/src/ScoringTier2.cc` (all of Tier-2 and Tier-3 live there despite the filename), plus `docs/scoring.md`.

### Tier 1 — Model Validity (0 or 1)
`aic_engine.cpp:706`: `score.tier_1_success()` runs unconditionally once `TrialState::ModelReady` is reached. **It is binary — 1 point if the policy loaded, configured, and activated; 0 otherwise.** No further measurement.

### Tier 2 — Performance & Convergence (sum can be negative)
Computed in `ScoringTier2::ComputeScore` (`ScoringTier2.cc:271-282`). Five category scores, summed:

| Category | Range | Function | Notes |
|---|---|---|---|
| `insertion force` | 0 to −12 | `GetInsertionForceScore` (line 874) | −12 fixed penalty if F-mag > 20 N for > 1 s cumulative |
| `contacts` | 0 to −24 | `GetContactsScore` (line 922) | −24 fixed penalty if any contact appears on `/aic/gazebo/contacts/off_limit` |
| `duration` | 0 to 12 | `GetTaskDurationScore` (line 941) | Linear: 5s → 12, 60s → 0. **Gated on Tier-3 > 0** |
| `trajectory smoothness` (jerk) | 0 to 6 | `GetTrajectoryJerkScore` (line 543) | Time-weighted avg jerk; 0 → 6 pts, 50 m/s³ → 0 pts. Gated on Tier-3 > 0 |
| `trajectory efficiency` | 0 to 6 | `GetTrajectoryEfficiencyScore` (line 655) | Path length: minPath → 6 pts, minPath+1m → 0 pts. Gated on Tier-3 > 0 |

Tier-2 ceiling = **12 + 6 + 6 = 24 points** when both penalties are zero. Floor = **−36 points**.

### Tier 3 — Task Success (max 75)
`ComputeTier3Score` (`ScoringTier2.cc:813`):
- If `/scoring/insertion_event` namespace matches `{target_module_name}/{port_name}` → **75 points** (`kInsertionCompletionScore`, line 816 + 844).
- If insertion event matches wrong port → **−12 points** (`kInsertionPenalty`).
- Else, fall through to `GetDistanceScore()` (line 698):
  - If plug is inside the port bounding-box (XY tolerance 5 mm, Z between port tip and port entrance): partial insertion, **38–50 points** linear in depth.
  - Else: proximity score **0–25 points** linear, max at port entrance, zero at half-initial-distance.

### Trial total
```
trial_total = tier_1 + tier_2 + tier_3
            = 1 + (up to 24, possibly negative) + (up to 75)
            ≤ 100
```

### Confirming the trial_1 result
Observed: **tier_1=1, tier_2=11.85, tier_3=75, total=87.85.**
- tier_1=1 ✓ (constant)
- tier_3=75 ✓ (matches `kInsertionCompletionScore` for correct-port success; insertion-event fired correctly)
- tier_2=11.85 — within the [−36, 24] envelope. The 12.15-point gap between observed (11.85) and ceiling (24) is the **trajectory smoothness + efficiency + duration tradeoff** vs the 24-pt cap. With CheatCode being slow + jerky and 0 penalties, that's expected.

**Yes, 100 is the per-trial maximum** (1 + 24 + 75) but only achievable with a fast (≤5s), perfectly smooth (jerk=0), optimal-path policy that never exceeds 20N for >1s and never hits an off-limit. The submission-portal aggregate over 3 qualification trials is therefore 300 (per `docs/scoring.md`).

### Off-limit contact penalty trigger
Source: `/aic/gazebo/contacts/off_limit` topic (published by Gazebo's contact sensor on the enclosure + task-board off-limit collision surfaces). Penalty is **−24 points, fixed, fires on first contact** (`ScoringTier2.cc:922-938`). Insertion force >20N for >1s is a separate **−12** that stacks.

---

## 6. Submission Packaging

### How a participant submits (per `docs/submission.md`)
1. Container only — OCI image (Docker / Podman). Push to ECR repo URI assigned per team.
2. Image must start a node named **`aic_model`** that is a **`rclpy.lifecycle.LifecycleNode`** advertising `/insert_cable` action server and `/aic_model/{get_state,change_state}` services.
3. Default workflow: subclass `aic_model.policy.Policy`, implement `insert_cable(self, task, get_observation, move_robot, send_feedback)`, ship as a Python package, and re-use `docker/aic_model/Dockerfile` with `CMD ["--ros-args", "-p", "policy:=<your_module>.<YourPolicyClass>", "-p", "use_sim_time:=true"]`.

### `docker/aic_model/Dockerfile`
- Base: `docker.io/library/ros:kilted-ros-core` (ROS 2 **Kilted Kaiju** — the official eval distro).
- Installs `pixi`, copies `aic_example_policies/`, `aic_model/`, `aic_interfaces/`, `aic_utils/` plus `pixi.toml + pixi.lock`. Runs `pixi install --locked`.
- Entrypoint script sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, reads `AIC_ROUTER_ADDR` (mandatory — the Zenoh router run by the `aic_eval` container), optional `AIC_ENABLE_ACL` + `AIC_MODEL_PASSWD` (per-team auth), then `exec pixi run --as-is ros2 run aic_model aic_model "$@"`.
- Default `CMD`: `["--ros-args", "-p", "policy:=aic_example_policies.ros.CheatCode", "-p", "use_sim_time:=true"]`.
- Transport: **Zenoh** between eval and model containers (network is `internal: true` per `docker-compose.yaml`). DDS not used.

### Standard policy file structure (`aic_example_policies/aic_example_policies/ros/*.py`)
Every example file follows the same skeleton:
```python
from aic_model.policy import (GetObservationCallback, MoveRobotCallback,
                              Policy, SendFeedbackCallback)
from aic_task_interfaces.msg import Task

class MyPolicy(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)

    def insert_cable(self, task: Task,
                     get_observation: GetObservationCallback,
                     move_robot: MoveRobotCallback,
                     send_feedback: SendFeedbackCallback) -> bool:
        # 1. read observation = get_observation()
        # 2. estimate port pose (CV) — replaces CheatCode's tf_buffer lookup
        # 3. call move_robot(motion_update=MotionUpdate(...)) or move_robot(joint_motion_update=...)
        # 4. return True on success, False on failure
```
Helpers inherited from `Policy`: `self.set_pose_target(move_robot, pose)`, `self.sleep_for(s)`, `self.time_now()`, `self.get_logger()`, `self.force_history(N)`, `self.switch_to_admittance()`, `self.switch_to_impedance()`.

---

## SHORT SUMMARY (≤ 300 words)

**Eval cycle, one sentence:** `aic_engine` reads a YAML, then for each `trial.task` it discovers the participant's `aic_model` lifecycle node, transitions it through configure → activate, spawns the task board + cable, opens a rosbag, sends one `InsertCable` action goal with the task definition + a `time_limit` (180 s per `sample_config.yaml`), records every scored topic for the duration, computes Tier-1 (binary) + Tier-2 (5 categories, −36 to +24) + Tier-3 (insertion event 75 / partial 0-50 / proximity 0-25), then deactivates the model and resets for the next trial.

**Required ROS interface for a submission:**
- **In (from engine via `aic_model` framework):** `aic_model_interfaces/Observation` (camera RGB ×3 + JointState + WrenchStamped + ControllerState, ~20 Hz) and one `aic_task_interfaces/InsertCable` action goal per task.
- **Out (to controller):** `/aic_controller/pose_commands` (`MotionUpdate`) **or** `/aic_controller/joint_commands` (`JointMotionUpdate`).
- **Lifecycle services the policy must serve:** `/aic_model/get_state` + `/aic_model/change_state` (free with `LifecycleNode`); the policy must reject `InsertCable` goals while not in `ACTIVE` and must not publish on the controller topics outside `ACTIVE`.

**Ground-truth → estimated-pose seam (concrete citation):**
- `aic_example_policies/aic_example_policies/ros/CheatCode.py:217-223` — `tf_buffer.lookup_transform("base_link", port_frame, ...)` where `port_frame = "task_board/{target_module_name}/{port_name}_link"`. **This is the only line in CheatCode that cheats.** Replace it with a CV pipeline that consumes `get_observation().{left,center,right}_image + *_camera_info` and outputs a `geometry_msgs/Transform` in `base_link`. Everything else in the file (`calc_gripper_pose`, the interpolation loop, the descent loop) stays.
- The `ground_truth` engine param (`aic_engine.cpp:310`) gates the **`/scoring/tf` → `/tf` relay node** in `aic_bringup/launch/aic_gz_bringup.launch.py:367-380`. In submission mode (`ground_truth:=false` per `docker-compose.yaml:9`) those frames disappear from `/tf` — but the scorer still records them via `/scoring/tf` for its own ground-truth comparison.

**Surprises / hidden gates:**
1. `time_limit` is **per-task in YAML**, not engine-level. Default 180 s.
2. Engine sends a **probe `InsertCable` goal while the policy is in `CONFIGURED` state** and fails the trial if the policy *accepts* it (`aic_engine.cpp:966-998`). Rejecting goals outside `ACTIVE` is a hard rule.
3. Engine watches `/aic_controller/{pose,joint}_commands` publisher counts before configure and after shutdown — publishing in `UNCONFIGURED` or after `SHUTDOWN` is a rules violation (currently a warning, "might affect scoring in the future" — `aic_engine.cpp:1620-1629`).
4. Tier-2 `duration / smoothness / efficiency` are **all gated on Tier-3 > 0** — if the plug ends up outside the proximity radius (half the initial distance), all three are forced to 0 regardless of how nice the trajectory was.
5. The Tier-3 75-point bonus comes from `/scoring/insertion_event` (a string `<target_module>/<port>` published by the Gazebo `CablePlugin`, `aic_gazebo/src/CablePlugin.cc:147-148`) — **not** from the action returning `success=true`. A successful action with no insertion event = no 75 pts.
6. Transport is **Zenoh, not DDS**, between eval and model containers (`docker/aic_model/Dockerfile:26`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`).
7. Official eval ROS 2 distro: **Kilted Kaiju**. Humble/Jazzy participants are warned that inter-distro is not supported.
