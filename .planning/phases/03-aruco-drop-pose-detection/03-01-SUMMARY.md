---
phase: 03-aruco-drop-pose-detection
plan: 01
subsystem: aruco-drop-pose-detection
tags: [aruco, multi-robot, drop-poses, naming-consistency]
dependency_graph:
  requires: []
  provides: [multi-robot-aruco-config, robot-aware-config-loader, consistent-drop-naming]
  affects: [extension.py, merged_localization_aruco.py, control_gui.py]
tech_stack:
  added: []
  patterns: [multi-robot-config-selection, backward-compat-fallback]
key_files:
  created: []
  modified:
    - ~/Desktop/ros2_ws/src/aruco_camera_localizer/config/aruco_config.json
    - ~/Desktop/ros2_ws/src/aruco_camera_localizer/aruco_camera_localizer/merged_localization_aruco.py
    - ~/Documents/isaac-sim-mcp/exts/soarm101-dt/so_arm101_dt/extension.py
    - ~/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py
decisions:
  - "D-01: active_robot defaults to so_arm101 (current working context)"
  - "D-02: drop_{aruco_id} naming replaces drop_{color} for consistency with ArUco localizer"
metrics:
  completed: "2026-03-27"
  tasks_completed: 4
  tasks_total: 4
---

# Phase 03 Plan 01: ArUco Drop Pose Detection Summary

Multi-robot aruco_config.json with SO-ARM101 cup markers (IDs 0-2, Y=0.083 Z=-0.039 offset), robot-aware load_aruco_config(), and drop_0/drop_1/drop_2 naming across Isaac Sim extension, ArUco localizer, and control GUI.

## Changes

### Task 1: Restructure aruco_config.json for multi-robot support
- Replaced flat `marker_rows` structure with `robots.jetank` and `robots.so_arm101` sections
- Added `active_robot: "so_arm101"` top-level field
- JETANK marker data (IDs 1-15, top/middle/bottom rows) preserved bit-for-bit
- SO-ARM101 cups config: marker_ids [0,1,2], position_offset X=0.0, Y=0.083, Z=-0.039

### Task 2: Update load_aruco_config() for multi-robot structure
- Changed signature to `load_aruco_config(robot: str = None)`
- Added multi-robot selection logic: reads `robots[active_robot]` or `robots[robot]` if overridden
- Preserved backward-compat fallback for old flat-format configs
- Added `--robot` CLI argument to `parse_args()`
- Wired `args.robot` through to `load_aruco_config()` call

### Task 3: Fix child_frame_id naming in Isaac Sim extension
- Changed `wrapper_path` from `drop_{color}` to `drop_{aruco_id}` in `_cmd_publish_drop_poses`
- Updated docstring and MCP_TOOL_REGISTRY description to reference drop_0/drop_1/drop_2
- No remaining references to drop_red/drop_green/drop_blue

### Task 4: Add drop ID color labels in control_gui drop listbox
- Added module-level `DROP_ID_LABELS` dict mapping drop_0->red, drop_1->green, drop_2->blue
- Updated `_populate_drop_list` to show `[color]` labels in listbox entries

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Fixed comment still referencing drop_{color}**
- Found during: Task 3
- Issue: Comment on line 4811 still said `drop_{color}` after wrapper_path was updated
- Fix: Updated comment to `drop_{aruco_id}`
- Files modified: extension.py

## Known Stubs

None.
