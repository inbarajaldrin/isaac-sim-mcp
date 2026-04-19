# SO-ARM101 Pick-and-Place Drop Support

## What This Is

Adding drop (place) capability to the SO-ARM101 pick-and-place pipeline across three repos: the Isaac Sim digital twin extension (drop pose publishing), the aruco_camera_localizer (ArUco-based drop pose detection), and the SO-ARM101 ROS2 control package (drop motion execution via 5-DOF wrist sweep). This completes the pick-and-place loop that currently only supports pick.

## Core Value

The robot can sort lego blocks by color into matching cups — pick from table, drop into the correct cup — driven by poses from Isaac Sim (direct) and later from ArUco marker detection (camera-based).

## Requirements

### Validated

- Pick pipeline works end-to-end (SO-ARM101 grasp tab, geometric IK, collision checking)
- YOLOE detection verified in both Gazebo (1.6mm) and Isaac Sim (3.2mm)
- Isaac Sim extension has cups with ArUco markers (IDs: red=0, green=1, blue=2, DICT_4X4_50)
- Cup positions and rim heights available from USD stage (`_get_cup_positions()`)
- SO-ARM101 grasp tab already switches button label to "Move to Drop" when topic is `/drop_poses`
- `sort_into_cups` teleports blocks above cups in sim (physics drop)

### Active

- [ ] Isaac Sim extension publishes `/drop_poses` topic (TFMessage) with cup positions + height offset so target pose is above cup rim
- [ ] SO-ARM101 drop motion: pan toward cup → wrist_flex 90°→0° sweep → gripper release (individual steps with buttons/services)
- [ ] SO-ARM101 grasp tab drop UI: buttons for each drop step + debug services for agent automation
- [ ] ArUco marker detection on cups via aruco_camera_localizer → publishes `/drop_poses` with relative offset above marker
- [ ] ArUco-to-cup color mapping shared between Isaac Sim extension and aruco_camera_localizer

### Out of Scope

- Automated full pick-and-place sequence (individual steps are fine) — keep steps separate for debugging
- Real hardware servo changes — software only, same servo_driver.py
- MoveIt motion planning for drop — geometric IK + direct joint control only
- JETANK integration — separate robot, separate package

## Context

- SO-ARM101 is a 5-DOF arm: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll + gripper_joint
- Drop motion is NOT a standard IK solve — it's a kinematic sweep: rotate pan to face cup, then wrist_flex from 90° (down) to 0° (forward) sweeps gripper tips over cup, release gripper
- The shoulder_lift + elbow_flex must be configured so the wrist_flex=0° position places gripper tips above the cup
- Cup ArUco config: DICT_4X4_50, 25mm markers, IDs {red:0, green:1, blue:2}
- JETANK's `move_to_drop.py` primitive (in ros-mcp-server robosort branch) is a reference for the `/drop_poses` topic subscription pattern but uses 3-DOF IK, not the 5-DOF sweep
- Isaac Sim publishes ROS2 topics via action graphs; cup pose publishing needs a new action graph or Python publisher
- The SO-ARM101 control GUI has a convention of `_cmd_*` methods auto-registering as debug `Trigger` services

### Multi-Repo Structure

| Repo | Path | What Changes |
|------|------|-------------|
| isaac-sim-mcp | `~/Documents/isaac-sim-mcp` | Extension: publish `/drop_poses` from cup USD positions |
| aruco_camera_localizer | `~/Desktop/ros2_ws/src/aruco_camera_localizer` | Detect cup ArUco markers → publish `/drop_poses` |
| vla_SO-ARM101 | `~/Projects/Exploring-VLAs/vla_SO-ARM101` | control_gui.py: drop motion, buttons, services |

## Constraints

- **No commits**: Until user confirms
- **Message format**: `/drop_poses` uses `tf2_msgs/TFMessage` (same as `/objects_poses`)
- **Drop pose convention**: Each transform's `child_frame_id` = `drop_{N}` (matching JETANK convention), translation = position above cup rim
- **Subagent models**: Sonnet for planning/research, Opus for execution
- **Individual steps**: Drop motion broken into discrete button-press steps with matching debug services

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| 5-DOF wrist sweep instead of IK-to-pose | SO-ARM101 wrist_flex sweep is more reliable than solving IK to a point above a small cup | -- Pending |
| Same TFMessage format as JETANK | Reuse existing `/drop_poses` subscription code in grasp tab | -- Pending |
| Isaac Sim direct publishing first, ArUco second | Get drop motion working with known-good poses before adding camera-based detection | -- Pending |
| Individual step buttons (not automated sequence) | Easier debugging, matches existing grasp tab pattern | -- Pending |

---
*Last updated: 2026-03-27 after initialization*
