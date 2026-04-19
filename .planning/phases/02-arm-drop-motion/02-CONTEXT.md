# Phase 2: Arm Drop Motion - Context

**Gathered:** 2026-03-28
**Status:** Ready for planning

<domain>
## Phase Boundary

SO-ARM101 control_gui.py gets a complete drop sequence: Point to Drop (rotate pan toward cup), Sweep to Drop (wrist_flex 90°→0° sweep), Release (open gripper). Each step is a separate button in a new "Drop" LabelFrame within the Grasp tab, with matching `_cmd_*` debug Trigger services. A separate drop listbox subscribes to `/drop_poses` and displays available drop targets independently of the pick object listbox.

Target repo: ~/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py

</domain>

<decisions>
## Implementation Decisions

### Drop Motion Kinematics
- "Point to Drop": compute `shoulder_pan = atan2(-y, x - X_PAN)` (same formula as geometric_ik line 208 in compute_workspace.py). Only pan moves, all other joints hold.
- "Sweep to Drop": compute IK for target point above cup at wrist_flex=0°, extract shoulder_lift + elbow_flex values. Then animate wrist_flex from 90°→0° via FollowJointTrajectory while other joints hold at the computed config.
- Sweep is animated over default 2.5s duration (same as _grasp_arm_duration_var), using FollowJointTrajectory action.
- Before "Point to Drop": check_grasp_reachable() cylindrical bounds check. If out of bounds, log warning and refuse to move.

### UI Integration
- New "Drop" LabelFrame within the existing Grasp tab (not a separate tab)
- 3 buttons: "Point to Drop", "Sweep to Drop", "Release"
- Duration spinbox for sweep duration (default 2.5s)
- SEPARATE drop listbox that subscribes to `/drop_poses` independently — does NOT reuse the pick object listbox
- Drop topic entry field (default: `/drop_poses`) with "Update Drop Topic" button

### Debug Services
- `~/drop_point` — Trigger service for Point to Drop
- `~/drop_sweep` — Trigger service for Sweep to Drop
- `~/drop_release` — Trigger service for Release
- All follow the existing `_cmd_*` auto-registration convention

### Claude's Discretion
- Exact layout within the Drop LabelFrame
- How to handle the case where wrist_flex is not at 90° when Sweep is pressed
- Whether to add a "Drop Home" button (return to grasp home after drop)

</decisions>

<code_context>
## Existing Code Insights

### Reusable Assets
- `geometric_ik()` in compute_workspace.py (line 208) — pan angle formula: `theta1 = atan2(-y, x - X_PAN)`
- `_cmd_grasp_reset()` — moves to wrist_flex=π/2 (grasp home), all others 0
- `_execute_trajectory()` — smooth trajectory via FollowJointTrajectory action
- `_send_arm_goal()` — sends arm trajectory to action server
- `_send_gripper_goal()` — sends gripper trajectory to action server
- `check_grasp_reachable()` — cylindrical workspace bounds check (r_min=0.09, r_max=0.31, z_min=-0.20, z_max=0.07)
- `_objects_callback()` — TFMessage subscriber that populates objects_data dict
- `_build_grasp_tab()` (line 1600) — existing grasp tab layout to extend

### Established Patterns
- `_cmd_*` methods auto-register as debug Trigger services
- Buttons use `ttk.Button` with `command=` or `clicked_fn=`
- Topic switching: `_update_objects_subscription()` destroys old sub, creates new one
- Object listbox populated by `_populate_object_list()`
- Background thread execution via `_prevalidate_and_execute()`

### Integration Points
- New Drop LabelFrame goes inside `_build_grasp_tab()` after existing "Arm" and "Gripper" LabelFrames
- Drop topic subscription: new subscriber for `/drop_poses` (TFMessage), separate from objects_poses
- Drop data stored in new dict `self._drop_data` (separate from `self.objects_data`)
- Drop listbox separate from existing object listbox

</code_context>

<specifics>
## Specific Ideas

- The `/drop_poses` publishes `drop_red`, `drop_green`, `drop_blue` as child_frame_ids (from Phase 1)
- For "Point to Drop": only shoulder_pan moves. Use _send_arm_goal with a single-joint trajectory.
- For "Sweep to Drop": first compute IK at target with wrist_flex=0 to get lift+elbow, then send multi-point trajectory that sweeps wrist_flex from current (90°) to 0° while holding pan/lift/elbow.
- For "Release": call _send_gripper_goal with the open position (same as existing "Grasp Open")
- The drop listbox should show entries like "drop_red (0.15, 0.08, 0.12)" with position info

</specifics>

<deferred>
## Deferred Ideas

- "Drop Home" button (return to grasp home after drop completion)
- Automated pick-and-place sequence chaining pick → drop
- Visual feedback (color coding drop targets in listbox)

</deferred>
