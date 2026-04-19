# Requirements — SO-ARM101 Drop Support

## v1 Requirements

### Isaac Sim Extension (isaac-sim-mcp)

- [x] **SIM-01**: Extension publishes `/drop_poses` topic (TFMessage) with one transform per cup, where translation = position above cup rim (cup center XY + rim_z + height offset)
- [x] **SIM-02**: Each transform's `child_frame_id` follows `drop_{N}` convention where N maps to cup color via ArUco ID (drop_0=red, drop_1=green, drop_2=blue)
- [x] **SIM-03**: Drop pose publishing is triggered via MCP tool `publish_drop_poses` and/or UI button, publishing continuously or latched

### SO-ARM101 Control GUI (vla_SO-ARM101)

- [ ] **ARM-01**: Grasp tab subscribes to `/drop_poses` (TFMessage) and populates detected drop locations in the object listbox
- [ ] **ARM-02**: "Point to Drop" button/service — rotates shoulder_pan to face the selected drop location (base rotation only)
- [ ] **ARM-03**: "Sweep to Drop" button/service — moves wrist_flex from 90° (grasp home) to 0° (forward), sweeping gripper tips over the cup
- [ ] **ARM-04**: "Release" button/service — opens gripper to release the held object into the cup
- [ ] **ARM-05**: All drop commands registered as debug Trigger services following `_cmd_*` convention (e.g., `~/drop_point`, `~/drop_sweep`, `~/drop_release`)
- [ ] **ARM-06**: Drop motion pre-validates that the cup is reachable (within cylindrical workspace bounds) before executing

### ArUco Camera Localizer (aruco_camera_localizer)

- [ ] **CAM-01**: Detect cup ArUco markers (DICT_4X4_50, IDs 0/1/2) from wrist camera and publish detected poses
- [ ] **CAM-02**: Publish `/drop_poses` topic (TFMessage) with translation offset above the marker (accounting for cup geometry: marker is at 45% height, drop point is above rim)
- [ ] **CAM-03**: ArUco ID → drop name mapping configurable in robot_config.yaml (aruco_id 0 → drop_0, etc.)

## v2 Requirements (Deferred)

- [ ] Automated pick-and-place sequence (pick → move to drop → sweep → release → return home)
- [ ] Multi-cup sorting strategy (decide which cup based on detected object color)
- [ ] Real hardware validation

## Out of Scope

- JETANK package changes — separate robot, not part of this work
- MoveIt-based drop planning — geometric joint control only
- Gripper force/grasp detection — open-loop release
- New ArUco dictionary or marker sizes — reuse existing DICT_4X4_50

## Traceability

| REQ | Phase | Status |
|-----|-------|--------|
| SIM-01 | Phase 1 | Complete |
| SIM-02 | Phase 1 | Complete |
| SIM-03 | Phase 1 | Complete |
| ARM-01 | Phase 2 | Pending |
| ARM-02 | Phase 2 | Pending |
| ARM-03 | Phase 2 | Pending |
| ARM-04 | Phase 2 | Pending |
| ARM-05 | Phase 2 | Pending |
| ARM-06 | Phase 2 | Pending |
| CAM-01 | Phase 3 | Pending |
| CAM-02 | Phase 3 | Pending |
| CAM-03 | Phase 3 | Pending |
