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
