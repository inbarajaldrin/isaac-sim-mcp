# Phase 3: ArUco Drop Pose Detection - Context

**Gathered:** 2026-03-28
**Status:** Ready for planning

<domain>
## Phase Boundary

Update aruco_camera_localizer to support SO-ARM101 cup ArUco markers alongside existing JETANK markers. Add cup marker entries (IDs 0,1,2) to aruco_config.json with position offsets from marker to drop point above cup rim. The existing `--drop` mode in merged_localization_aruco.py already handles the offset transformation — we just need the config entries. Use `drop_{id}` naming for child_frame_ids (consistent with JETANK).

Target repo: ~/Desktop/ros2_ws/src/aruco_camera_localizer

</domain>

<decisions>
## Implementation Decisions

### Multi-Robot aruco_config.json
- Restructure aruco_config.json to support multiple robot configs (JETANK and SO-ARM101)
- Add a top-level `active_robot` field or robot-specific sections
- JETANK's existing marker_rows (IDs 1-15) stay untouched
- SO-ARM101 cup markers: IDs 0, 1, 2 with color mapping (0=red, 1=green, 2=blue)
- No collision: JETANK uses IDs 1-15, SO-ARM101 uses IDs 0-2

### child_frame_id Naming
- Use `drop_{id}` for both robots (e.g., drop_0, drop_1, drop_2 for SO-ARM101)
- The SO-ARM101 control_gui already subscribes to /drop_poses and handles any child_frame_id naming
- Need to update Phase 2 code: the drop listbox should handle both `drop_red` and `drop_{id}` naming (or we standardize to `drop_{id}`)

### Position Offset (marker frame → drop point)
- Cup dimensions: diameter=78mm, height=96.5mm, marker at 45% height (43.4mm from bottom)
- Offset from marker to drop point above cup rim:
  - Y: +0.083m (up from 43.4mm to 96.5mm rim + 30mm clearance = 83mm)
  - Z: -0.039m (inward toward cup center = negative marker normal, ~half cup radius)
  - X: 0.0m (no lateral offset)
- These are initial estimates — may need tuning after testing in Isaac Sim

### robot_config.yaml
- No changes needed — JETANK doesn't use robot_config.yaml for drop either
- Everything stays in aruco_config.json

### Code Changes
- aruco_config.json: add SO-ARM101 section with cup markers
- merged_localization_aruco.py: update load_aruco_config() to handle multi-robot structure + active_robot selection
- No changes to localizer_bridge.py (publish_drop_poses already works generically)

### Claude's Discretion
- Exact JSON structure for multi-robot support in aruco_config.json
- Whether to use command-line arg or config field for robot selection
- Whether to also update the localize_yoloe.py entry point for consistency

</decisions>

<code_context>
## Existing Code Insights

### Reusable Assets
- `merged_localization_aruco.py`: --drop flag, load_aruco_config(), transform_offset_marker_to_world()
- `localizer_bridge.py`: publish_drop_poses() — generic, works with any drop pose data
- `aruco_config.json`: current JETANK marker_rows structure

### Established Patterns
- aruco_config.json: marker_rows → marker_ids + position_offset per row
- load_aruco_config() returns marker_to_row dict: {marker_id: (row_name, offset_dict)}
- merged_localization_aruco.py line 243-276: drop pose calculation loop
- frame naming: f"drop_{marker_id}"

### Integration Points
- aruco_config.json is the only file needing structural changes
- load_aruco_config() in merged_localization_aruco.py needs to select robot config
- No changes to localizer_bridge.py or the publishing pipeline

</code_context>

<specifics>
## Specific Ideas

- Multi-robot config could look like: {"active_robot": "so_arm101", "robots": {"jetank": {...}, "so_arm101": {...}}}
- Or simpler: add `--robot` CLI arg to merged_localization_aruco.py that selects which marker_rows to load
- The existing `--drop` flag stays — it enables drop mode. `--robot` selects which config.
- SO-ARM101 config entry: {"cups": {"marker_ids": [0, 1, 2], "position_offset": {"X": 0.0, "Y": 0.083, "Z": -0.039}}}

### Phase 1 child_frame_id mismatch
- Phase 1 publishes `drop_red`, `drop_green`, `drop_blue` from Isaac Sim
- This phase's aruco code would publish `drop_0`, `drop_1`, `drop_2`
- Phase 2 drop listbox needs to handle both namings
- OR: update Phase 1 to also use `drop_{id}` naming (simpler)

</specifics>

<deferred>
## Deferred Ideas

- Tuning offset values after testing in Isaac Sim
- Adding per-marker offsets (different cup sizes)
- Automatic robot detection from active topics

</deferred>
