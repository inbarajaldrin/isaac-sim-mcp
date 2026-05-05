# Plan 03-01 Snapshot — Live aic_eval Scoring Topic Inventory

**Captured:** 2026-05-05
**Source:** `aic_eval` Docker container (`ghcr.io/intrinsic-dev/aic/aic_eval:latest`) running `ground_truth:=true` + `aic_example_policies.ros.CheatCode` policy.
**Capture command:** `bash exts/aic-dt/scripts/snapshot_aic_eval_scoring.sh`
**Capture file:** `.planning/phases/03-cable-physics/snapshot/aic_eval_scoring_capture.txt` + `aic_eval_objects_poses_capture.txt`

## Critical Finding

**Neither `/scoring/insertion_event` NOR `/scoring/tf` exists in the live `aic_eval` container.** Both are referenced in `aic_engine/config/sample_config.yaml` but never advertised by any node. This matches the CLAUDE.md note about sample_config / live divergence.

REQUIREMENTS.md PARITY-07/08 must be **re-scoped** to live topics.

## Live ground-truth surface

| Topic | Type | Publisher (live) | Subscriber (live) | Notes |
|---|---|---|---|---|
| `/objects_poses_real` | `tf2_msgs/msg/TFMessage` | **NONE (count=0)** | `grasp_points_publisher` | The ground-truth source the AIC stack expects. EMPTY in pure-Gazebo aic_eval — to be filled by Isaac Sim. |
| `/grasp_points_real` | `visualization_msgs/MarkerArray` | TBD (likely `grasp_points_publisher`) | aic_engine | Grasp-point markers per cable. |
| `/objects_poses_sim` | retired | — | — | Was Plan 01-04's atom name; deleted per Phase 1 closure. |
| `/scoring/insertion_event` | sample_config ghost | NONE | NONE | Does not exist live. |
| `/scoring/tf` | sample_config ghost | NONE | NONE | Does not exist live. |

## What the aic_eval topic surface DOES provide

Captured topic list (live from `aic_eval` after CheatCode warmup):
- All ros2_control topics (`/joint_trajectory_controller/*`, `/scaled_joint_trajectory_controller/*`, `/forward_*_controller/*`, `/force_mode_controller/*`, `/freedrive_mode_controller/*`, `/passthrough_trajectory_controller/*`, `/tool_contact_controller/*`, `/io_and_status_controller/*`, `/ur_configuration_controller/*`)
- `/joint_state_broadcaster/transition_event`
- `/force_torque_sensor_broadcaster/wrench` ✓ (matches Phase 1 PARITY-05)
- `/force_torque_sensor_broadcaster/transition_event`
- `/tcp_pose_broadcaster/pose` + `/tcp_pose_broadcaster/transition_event`
- `/joint_states`, `/dynamic_joint_states`
- `/gripper_command`, `/gripper_status`, `/gripper_width`, `/gripper_grasp_detected`, `/gripper_motion_ongoing`, `/gripper_width_offset`
- `/objects_poses_real` (TFMessage, awaiting publisher)
- `/grasp_points_real` (MarkerArray)
- `/camera/color/*`, `/camera/depth/*`, `/camera/depth_filter_status`, `/camera/depth_to_color`
- `/tf`, `/tf_static`
- `/diagnostics`, `/parameter_events`, `/rosout`
- `/rg2/joint_states`, `/rg2/robot_description`, `/robot_description`

NO `/scoring/*`. NO cable/plug/port frames in `/tf` (capture window was empty for these).

## Phase 3 plan revisions

**PARITY-08** (REQUIREMENTS.md current text: "Isaac Sim publishes `/scoring/tf` (`tf2_msgs/TFMessage`) containing cable link poses") **is re-pointed to:**

> Isaac Sim publishes `/objects_poses_real` (`tf2_msgs/TFMessage`) containing cable link + plug + port frame poses, so that `aic_engine`'s `grasp_points_publisher` subscription receives ground-truth data and CheatCode trial scoring can proceed.

**PARITY-07** (REQUIREMENTS.md current text: "Isaac Sim publishes `/scoring/insertion_event` ... when cable plug-port insertion completes") **is DEFERRED** — no live analog exists; the insertion-event signal is internal to `aic_engine`'s scoring pipeline. Phase 4 will revisit if a CheatCode trial reveals an actual need.

**SCENE-06** Object TF additions to `parity_publishers` `_TF_EDGES` are still valid but use the literal frame names `aic_engine`'s `grasp_points_publisher` expects (probe via Plan 03-01 follow-up — current capture didn't catch them because aic_eval has no publisher to fill /tf with cable frames either; we discover them by reading `aic_adapter` source).

## Re-scoped Phase 3 deliverables (this plan informs)

| Plan | Original | Revised |
|---|---|---|
| 03-04 | `/scoring/tf` publisher in parity_publishers | `/objects_poses_real` publisher in parity_publishers (TFMessage of cable + plug + port poses) |
| 03-05 | `/scoring/insertion_event` publisher with omni.physx contact-report filter | **DEFERRED** to Phase 4 — no live analog. Plan 03-05 retired. |
| 03-06 | smoke test 6/6 | smoke test 5/5 (one less requirement) |

REQUIREMENTS.md flag flips on close:
- PARITY-08 [x] (re-pointed to /objects_poses_real, contract honored)
- PARITY-07 stays [ ] with `deferred-to-phase-4` annotation
- SCENE-02/03/05/06 [x] as planned
