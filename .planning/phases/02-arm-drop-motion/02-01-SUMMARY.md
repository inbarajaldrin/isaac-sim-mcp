---
phase: 02-arm-drop-motion
plan: 01
subsystem: so-arm101-control-gui
tags: [drop-motion, pick-and-place, ros2, tkinter]
dependency_graph:
  requires: [01-01]
  provides: [drop-subscriber, drop-ui, drop-motion-services]
  affects: [control_gui.py]
tech_stack:
  added: []
  patterns: [_cmd_auto_registration, TFMessage_subscription, geometric_ik]
key_files:
  modified:
    - /home/aaugus11/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py
decisions:
  - Reused existing _objects_callback pattern for _drop_callback (same TFMessage format)
  - Used geometric_ik for sweep IK rather than manual trig (consistent with grasp pipeline)
  - Pan-only motion uses _send_arm_goal directly (single point, no interpolation needed)
metrics:
  completed: "2026-03-27"
  tasks_completed: 3
  tasks_total: 3
---

# Phase 02 Plan 01: Drop Motion Commands Summary

Drop subscription, UI section, and three motion methods added to control_gui.py for SO-ARM101 cup-drop sequence.

## What Was Done

### Task 1: Drop data infrastructure
Added `_drop_data` dict, `_drop_lock`, `_drop_sub` in `__init__`. Created `_drop_callback` (TFMessage subscriber), `_update_drop_topic` (subscription swap), `_populate_drop_list` (GUI refresh), `_get_selected_drop_pose` (selection helper).

### Task 2: Drop LabelFrame UI
Added three new LabelFrames to `_build_grasp_tab`: Drop Source (topic entry + update/refresh buttons), Drop Targets (listbox showing drop_red/green/blue poses), and Drop (sweep duration spinbox + Point to Drop / Sweep to Drop / Release buttons).

### Task 3: Motion methods
- `_cmd_drop_point`: Computes `pan = atan2(-y, x - X_PAN)`, sends single-joint pan move via `_send_arm_goal`. Pre-validates with `check_grasp_reachable`.
- `_cmd_drop_sweep`: Calls `geometric_ik` at target position, then `_execute_trajectory` to sweep `wrist_flex` from current angle to 0 degrees over configurable duration. Pre-validates reachability.
- `_cmd_drop_release`: Opens gripper to `JOINT_LIMITS['gripper_joint'][1]` (max open position).

All three methods follow `_cmd_*` naming convention for automatic `~/drop_point`, `~/drop_sweep`, `~/drop_release` Trigger service registration.

## Requirements Satisfied

| Requirement | How |
|-------------|-----|
| ARM-01 | `_drop_sub` subscribes to `/drop_poses`; `_drop_listbox` shows entries |
| ARM-02 | `_cmd_drop_point` rotates only `shoulder_pan` via `atan2(-y, x-X_PAN)` |
| ARM-03 | `_cmd_drop_sweep` animates `wrist_flex` 90 to 0 degrees via `_execute_trajectory` with `geometric_ik` |
| ARM-04 | `_cmd_drop_release` opens gripper to JOINT_LIMITS max |
| ARM-05 | All three methods named `_cmd_*` so auto-registered as services |
| ARM-06 | `check_grasp_reachable` called in both `_cmd_drop_point` and `_cmd_drop_sweep` before motion |

## Deviations from Plan

None - plan executed exactly as written.

## Known Stubs

None.

## Self-Check: PASSED
