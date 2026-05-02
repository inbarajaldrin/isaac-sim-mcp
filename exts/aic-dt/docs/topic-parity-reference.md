# Topic Parity Reference (aic_eval Live Snapshot)

## Provenance

This document is the canonical reference for "what Gazebo publishes" — the empirical
topic surface the Isaac Sim aic-dt extension must match for AIC trial parity. Per
**D-01** of Phase 1, the source of truth is the live `aic_eval` Docker container
(no YAML, no documentation, no inference): the container is brought up headlessly,
its `ros2 topic list` and `tf2_tools view_frames` output are captured, and any
divergence between the container and the documentation in `~/Documents/aic` is
resolved in favor of the container.

Per **D-14**, the upstream image is pinned by SHA-256 digest to detect tag drift:

```
ghcr.io/intrinsic-dev/aic/aic_eval@sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433
```

(Local image ID at capture time: `sha256:34e8a0bcee744062fb7f1171173bbc77a0b74a5da46f192cfe7b954565686b5b`, size 6.36 GB.)

**Re-capture with**: `bash exts/aic-dt/scripts/snapshot_aic_eval.sh`. This refreshes
`.planning/phases/01-foundation-parity/snapshot/` with image digest, topic list,
per-topic info, `/joint_states` echo, and `view_frames` TF tree.

**Drift note (2026-05-02 capture vs research-time 2026-05-02 capture):** Two new
topics appeared: `/aic_controller/joint_motion_update` and
`/aic_controller/motion_update` — both are Phase 2 deferred (controller surface).
Topic count went from 35 to 36 (research's "35" included a stray daemon-stopped
log line; the cleaned count is 33 + the 2 new = 36 real topics).

## Phase 1 Passive Sensor Surface

The three topics below are the entire deliverable surface Phase 1 must publish from
Isaac Sim. Phase 2 adds the controller-loop surface; Phase 3 adds the scoring/object-pose surface.

| Topic | Type | QoS | Rate | Phase 1 source |
|-------|------|-----|------|----------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | RELIABLE / VOLATILE / KEEP_LAST(42) | 500 Hz | Isaac Sim ROS2PublishJointState |
| `/tf` | `tf2_msgs/msg/TFMessage` | RELIABLE / VOLATILE / KEEP_LAST(10) | 500 Hz | Isaac Sim ROS2PublishTransformTree (`staticPublisher=False`) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | RELIABLE / **TRANSIENT_LOCAL** / KEEP_LAST(1) | static one-shot | Isaac Sim ROS2PublishTransformTree (`staticPublisher=True`) |

**Note on rate parity:** 500 Hz is `controller_manager.ros__parameters.update_rate`
in `aic_ros2_controllers.yaml`. CheatCode polls TF lookups and does not rate-check;
Isaac Sim's `OnPlaybackTick`-driven graphs publish per render frame (~60 Hz) which
is acceptable for Phase 1. PARITY-09/10 (Phase 2) is where rate parity matters.

## /joint_states Joint Name Set + Ordering

**7 joints, alphabetical order, `header.frame_id = base_link`.** Captured live:

```yaml
header:
  stamp: { sec: 31, nanosec: 484000000 }
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
effort:   [...]
```

Key facts (per D-11 — all parity-critical):

- **7 joints, alphabetical (NOT kinematic-chain) ordering.**
- **`gripper/left_finger_joint`** with the literal `/` character is the gripper joint
  name — `gripper/` is the `gripper_tf_prefix` from the xacro and the same prefix
  appears in TF frames.
- **`header.frame_id` is `base_link`** (not `world`, not `tabletop`).
- **`gripper/right_finger_joint` is NOT in `/joint_states`** — only the left finger.
  The Hand-E ros2_control xacro declares only the left finger as a state interface;
  the right mimics via fixed-mimic constraint. Publish only what Gazebo publishes.

The publisher is `joint_state_broadcaster` (1 publisher, 2 subscribers — `aic_adapter_node`
and an `rqt_*` introspection client).

## TF Tree (frames.gv)

**31 frames total**, 30 edges, root `world`. The tree captured live:

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

**Dynamic part (publishes at ~440 Hz on `/tf` via `tf_relay` node):** the 6 robot
joints (`base_link_inertia → shoulder_link → upper_arm_link → forearm_link →
wrist_1 → wrist_2 → wrist_3`) plus the 2 gripper finger joints
(`gripper/hande_finger_link_l`, `gripper/hande_finger_link_r`).

**Static part (one-shot on `/tf_static` via `robot_state_publisher` and
`ground_truth_static_tf_publisher`):** the remaining 22 frames — cameras, gripper
base, ATI sensor, world↔aic_world↔tabletop↔base_link.

**Frame-name implementation note:** AIC's frame names contain slashes
(`gripper/hande_base_link`, `cam_mount/cam_mount_link`, etc.). USD prim paths
disallow slashes inside a single name. When Isaac Sim's `ROS2PublishTransformTree`
node derives `frame_id` from prim names, the unified USD's prim names (e.g.
`gripper_hande_base_link`) need to be remapped to the slashed AIC names. This is
the trickiest piece of PARITY-04 — likely needs a USD-side rename or per-prim
custom attribute. Plan 06 owns this work.

Live capture artifact: `.planning/phases/01-foundation-parity/aic_frames_live.gv`.

## Phase-3 Work Items (Out of Phase 1)

The cable subtree is `SetActive(False)` for Phase 1 per **D-04**. Cable physics is
Phase 3 work (**SCENE-05**). Cable assets ARE inspected for textures during the
Phase 1 sweep (TEX-01..03) — only simulation is disabled. The 21-segment cable
rope chain (mass=0/inertia=0 D6 joints) wedges PhysX simulation post-play when
active inside the aic-dt extension Kit env; final cable-physics fix lands in Phase 3.

Other deferred Phase-3 items reachable from this surface:
- Object TF frames CheatCode reads (`{cable_name}/{plug_name}_link`, port frames in `base_link`) — **SCENE-06**.
- `/scoring/insertion_event` — **PARITY-07**.
- `/scoring/tf` — **PARITY-08**.

Phase 2 items: full `ros2_control` surface (`/aic_controller/*`, gripper command/status
topics, `force_mode_controller`, `freedrive_mode_controller`, etc.) — **PARITY-06/09/10/11**.

## Reproducing This Snapshot

```bash
bash exts/aic-dt/scripts/snapshot_aic_eval.sh
```

Per **D-13**, this works without `~/Documents/aic` checked out — only Docker is
required. The script:
1. `docker rm -f aic_eval` (idempotent cleanup)
2. `docker run -d --gpus all --runtime=nvidia --net=host --privileged ghcr.io/intrinsic-dev/aic/aic_eval:latest ground_truth:=true start_aic_engine:=true gazebo_gui:=false`
3. `sleep 35` for Gazebo + controller_manager bringup
4. `docker inspect ... ghcr.io/intrinsic-dev/aic/aic_eval:latest` → `image_digest.txt`
5. In-container `ros2 topic list` (with Zenoh env exports) → `topic_list.txt`
6. Per-topic `ros2 topic info --verbose` for `/joint_states /tf /tf_static /clock /fts_broadcaster/wrench`
7. `ros2 topic echo /joint_states --once` → `joint_states_sample.yaml`
8. `ros2 run tf2_tools view_frames` → `aic_frames_live.{gv,pdf}`
9. `docker stop && docker rm` cleanup

Outputs land under `.planning/phases/01-foundation-parity/snapshot/` by default
(override with positional arg). Total wall-clock: ~50 seconds when image is cached.

## Out-of-Phase-1 Topics in This Snapshot

The full live surface contains 36 topics. Phase 1 implements only the 3 passive-sensor
ones. The remaining 33 belong to later phases or are Gazebo / ROS internals. See the
**Cross-Phase Parity Audit** section below for the row-per-topic mapping.

| Topic group | Phase | Requirement |
|-------------|-------|-------------|
| `/aic_controller/*`, `/joint_*_controller/*`, `/gripper_*` | Phase 2 | PARITY-06, PARITY-09, PARITY-10, PARITY-11 |
| `/aic/gazebo/contacts/off_limit` | Phase 2 | PARITY-06 |
| `/scoring/insertion_event`, `/scoring/tf`, `/objects_poses_real`, `/grasp_points_real` | Phase 3 | PARITY-07, PARITY-08, SCENE-06 |
| Trial-loader-only topics (none observed live) | Phase 4 | TRIAL-01..05 |
| `/parameter_events`, `/rosout`, `/diagnostics`, `/observations`, `/robot_description`, `/clock`, `/dynamic_joint_states`, `/controller_manager/*` | not applicable (Gazebo / ROS internal) | n/a |

## Cross-Phase Parity Audit (PARITY-12)

This section is the single source of truth for "every Gazebo topic ↔ Isaac Sim
equivalent ↔ status". It is the canonical Phase-1-ship deliverable for **PARITY-12**:
a re-runnable check that no Gazebo topic is unintentionally missing or unmapped on
the Isaac Sim side. Every row in `aic_topics_live.txt` (the full 36-topic surface)
appears below with its disposition.

To regenerate the live source row-set, run `bash exts/aic-dt/scripts/snapshot_aic_eval.sh`.
Then update the table below if `topic_list.txt` shows new/removed topics.

| Topic | Type | Live Gazebo | Isaac Sim Status | Phase | Proof-of-Publish |
|-------|------|-------------|------------------|-------|------------------|
| `/aic_controller/controller_state` | `aic_control_interfaces/msg/ControllerState` | yes | Phase 2 deferred | Phase 2 (PARITY-11) | — |
| `/aic_controller/joint_commands` | `aic_control_interfaces/msg/JointMotionUpdate` | yes | Phase 2 deferred | Phase 2 (PARITY-09) | — |
| `/aic_controller/joint_motion_update` | `aic_control_interfaces/msg/JointMotionUpdate` | yes | Phase 2 deferred | Phase 2 (PARITY-09) | — |
| `/aic_controller/motion_update` | `aic_control_interfaces/msg/MotionUpdate` | yes | Phase 2 deferred | Phase 2 (PARITY-10) | — |
| `/aic_controller/pose_commands` | `aic_control_interfaces/msg/MotionUpdate` | yes | Phase 2 deferred | Phase 2 (PARITY-10) | — |
| `/aic_controller/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | yes | Phase 2 deferred | Phase 2 (PARITY-09/10/11) | — |
| `/aic/gazebo/contacts/off_limit` | `ros_gz_interfaces/msg/Contacts` | yes | Phase 2 deferred | Phase 2 (PARITY-06) | — |
| `/center_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /center_camera/camera_info` after `quick_start` |
| `/center_camera/image` | `sensor_msgs/msg/Image` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /center_camera/image` after `quick_start` |
| `/clock` | `rosgraph_msgs/msg/Clock` | yes | implemented | Phase 1 (Isaac Sim native via setup_action_graph) | `ros2 topic echo /clock --once` after `quick_start` |
| `/controller_manager/activity` | `controller_manager_msgs/msg/Activity` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/introspection_data/full` | `control_msgs/msg/MultiDOFStateStamped` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/introspection_data/names` | `control_msgs/msg/MultiDOFCommand` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/introspection_data/values` | `control_msgs/msg/MultiDOFCommand` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/statistics/full` | `control_msgs/msg/MultiDOFStateStamped` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/statistics/names` | `control_msgs/msg/MultiDOFCommand` | yes | not applicable (Gazebo internal) | n/a | — |
| `/controller_manager/statistics/values` | `control_msgs/msg/MultiDOFCommand` | yes | not applicable (Gazebo internal) | n/a | — |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | yes | not applicable (ROS internal) | n/a | — |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | yes | not applicable (Gazebo internal) | n/a | — |
| `/fts_broadcaster/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | yes | not applicable (Gazebo lifecycle internal) | n/a | — |
| `/fts_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | yes | implemented | Phase 1 (Plan 04 — DX-09 rename + Plan 04 Task 4 PARITY-05 full match) | `ros2 topic echo /fts_broadcaster/wrench --once` after `quick_start` (frame_id must equal `ati/tool_link`) |
| `/fts_broadcaster/wrench_filtered` | `geometry_msgs/msg/WrenchStamped` | yes | Phase 2 deferred | Phase 2 (PARITY-09 — controller-side filter) | — |
| `/joint_state_broadcaster/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | yes | not applicable (Gazebo lifecycle internal) | n/a | — |
| `/joint_states` | `sensor_msgs/msg/JointState` | yes | implemented | Phase 1 (Plan 06) | `ros2 topic echo /joint_states --once` after `quick_start` (must match the 7-joint alphabetical name set above) |
| `/left_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /left_camera/camera_info` after `quick_start` |
| `/left_camera/image` | `sensor_msgs/msg/Image` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /left_camera/image` after `quick_start` |
| `/observations` | `aic_engine_interfaces/msg/Observations` | yes | Phase 4 deferred | Phase 4 (TRIAL-03 — engine-side observation aggregator) | — |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | yes | not applicable (ROS internal) | n/a | — |
| `/right_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /right_camera/camera_info` after `quick_start` |
| `/right_camera/image` | `sensor_msgs/msg/Image` | yes | implemented | Phase 1 (Plan 04 rename) | `ros2 topic hz /right_camera/image` after `quick_start` |
| `/robot_description` | `std_msgs/msg/String` | yes | not applicable (Gazebo internal — robot_state_publisher param) | n/a | — |
| `/rosout` | `rcl_interfaces/msg/Log` | yes | not applicable (ROS internal) | n/a | — |
| `/scoring/insertion_event` | `std_msgs/msg/String` | yes | Phase 3 deferred | Phase 3 (PARITY-07) | — |
| `/scoring/tf` | `tf2_msgs/msg/TFMessage` | yes | Phase 3 deferred | Phase 3 (PARITY-08) | — |
| `/tf` | `tf2_msgs/msg/TFMessage` | yes | implemented | Phase 1 (Plan 06 — ROS2PublishTransformTree, staticPublisher=False) | `ros2 topic hz /tf` after `quick_start` (expect ~440 Hz on dynamic robot frames) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | yes | implemented | Phase 1 (Plan 06 — ROS2PublishTransformTree, staticPublisher=True) | `ros2 topic echo /tf_static --once` after `quick_start` (TRANSIENT_LOCAL QoS, one-shot) |

**Footnote: `not applicable` topics.** These are Gazebo physics-engine or ROS-runtime
internals with no expected Isaac Sim equivalent:
- `/controller_manager/*` and `*/transition_event` are `ros2_control` lifecycle
  diagnostics — Isaac Sim does not run a `controller_manager`; the publisher pattern
  is direct OmniGraph publishers, no lifecycle node.
- `/parameter_events`, `/rosout`, `/diagnostics` are ROS-runtime infrastructure;
  any rclpy-using node in aic-dt gets these for free.
- `/robot_description` is a `robot_state_publisher` parameter; Phase 1 D-10
  explicitly chooses Isaac Sim TF action graph over an external `robot_state_publisher`,
  so `/robot_description` is not republished from sim.
- `/dynamic_joint_states` is a `joint_state_broadcaster` extension topic that
  duplicates `/joint_states` with interface metadata; Isaac Sim publishes
  `/joint_states` only.

### How to re-run this audit

```bash
# 1. Refresh live snapshot (Docker required)
bash exts/aic-dt/scripts/snapshot_aic_eval.sh

# 2. Diff the audit table's topics against fresh topic_list.txt — any new/removed
#    topic means this table needs a corresponding row added/removed.
diff <(grep -E '^\| `/' exts/aic-dt/docs/topic-parity-reference.md | awk -F'`' '{print $2}' | sort) \
     <(sort .planning/phases/01-foundation-parity/snapshot/topic_list.txt)
```

**Which Plan(s) update the audit when a phase is closed:**

- **Phase 1 ship:** this Plan 01 Task 4 (initial population) + Plan 04 Task 4 (PARITY-05 full F/T match) + Plan 06 (TF / JointState implemented rows).
- **Phase 2 ship:** future plan flips Phase-2-deferred rows to `implemented` and
  fills their Proof-of-Publish columns (`/aic_controller/*`, `/aic/gazebo/contacts/off_limit`,
  `/fts_broadcaster/wrench_filtered`, etc.).
- **Phase 3 ship:** future plan flips Phase-3-deferred rows to `implemented`
  (`/scoring/insertion_event`, `/scoring/tf`).
- **Phase 4 ship:** future plan flips remaining `Phase 4 deferred` rows
  (`/observations`).

The audit's invariant: every topic in `aic_topics_live.txt` MUST appear as exactly
one row, with an explicit Isaac Sim Status disposition. No silent omissions —
total surface coverage with explicit dispositions is the value.
