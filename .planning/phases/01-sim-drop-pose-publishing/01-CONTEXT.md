# Phase 1: Sim Drop Pose Publishing - Context

**Gathered:** 2026-03-27
**Status:** Ready for planning

<domain>
## Phase Boundary

Isaac Sim extension publishes /drop_poses TFMessage topic with the ArUco marker poses on each cup. The published poses represent what a real camera would detect — the 6D pose of the ArUco marker on the cup surface. The SO-ARM101 ROS2 package (Phase 2) is responsible for computing drop height from these poses.

This maintains sim/real parity: both sim and real-world provide the same information (ArUco marker 6D pose), and the control package handles the geometry.

</domain>

<decisions>
## Implementation Decisions

### Publishing Mechanism
- Use ROS2PublishTransformTree action graph (same pattern as objects_poses publisher)
- Publish actual ArUco marker poses (position + orientation on cup surface), NOT pre-computed drop positions
- child_frame_id naming: `drop_red`, `drop_green`, `drop_blue`
- parentPrim: /World (base frame), targetPrims: ArUco marker mesh prims on each cup

### Drop Height Responsibility
- Isaac Sim only publishes marker poses — height calculation is the SO-ARM101 package's job
- The SO-ARM101 package computes cup rim height from the marker pose + known cup geometry
- Reachability validation uses the wrist sweep arc algorithm in the control package

### Claude's Discretion
- Action graph path naming convention
- Whether to add a UI button vs MCP tool vs both for triggering the publisher
- Timer/frequency for pose publishing

</decisions>

<code_context>
## Existing Code Insights

### Reusable Assets
- `_get_cup_positions()` at line 2810 — returns cup bbox data (center_x, center_y, rim_z, opening_radius)
- `CUP_ARUCO_CONFIG` at line 164 — ArUco IDs per color: {red:0, green:1, blue:2}
- Objects pose action graph at line 3780 — exact pattern to follow (ROS2PublishTransformTree with parentPrim/targetPrims)
- ArUco marker prims placed at `{cup_path}/aruco_{id:03d}` by `_add_aruco_to_cups()`

### Established Patterns
- Action graphs created via `og.Controller.edit()` with keys.CREATE_NODES, keys.CONNECT, keys.SET_VALUES
- Parent/target prims set via USD relationships after graph creation
- MCP tools registered in MCP_TOOL_REGISTRY dict, handlers in MCP_HANDLERS dict
- UI buttons in `_build_objects_frame()` call methods on DigitalTwin

### Integration Points
- New action graph goes under `/Graph/ActionGraph_drop_poses`
- MCP tool: `publish_drop_poses` in MCP_TOOL_REGISTRY + MCP_HANDLERS
- UI button alongside existing "Add Cups" button
- Requires cups to be spawned first (depends on `add_cups()` having been called)

</code_context>

<specifics>
## Specific Ideas

- The ArUco marker mesh prims are the targetPrims — their world position IS the marker pose
- The marker mesh is at `{cup_prim_path}/aruco_{id:03d}/aruco_marker_mesh`
- Topic name: `/drop_poses` (TFMessage)
- Frame naming: `drop_red`, `drop_green`, `drop_blue` (not numeric IDs)

### aruco_camera_localizer Integration (Reference for Phase 3)
- `aruco_camera_localizer` already supports `--drop` flag on `merged_localization_aruco.py`
- `aruco_config.json` maps marker IDs → position_offset in marker frame
- `transform_offset_marker_to_world()` rotates offset by marker quaternion + adds to marker world pos
- `localizer_bridge.py` has `publish_drop_poses()` publishing TFMessage to `/drop_poses`
- For SO-ARM101 cups: need new entries in `aruco_config.json` for marker IDs 0,1,2 with offset from marker (at 45% cup height on side) UP to above cup rim + INWARD to cup center
- Currently JETANK uses `drop_{marker_id}` naming; SO-ARM101 will use `drop_red`, `drop_green`, `drop_blue`

</specifics>

<deferred>
## Deferred Ideas

- aruco_config.json entries for SO-ARM101 cups (Phase 3)
- Offset calculation from marker frame to drop point above cup rim (Phase 3)

</deferred>
