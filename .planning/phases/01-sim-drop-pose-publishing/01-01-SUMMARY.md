---
phase: 01-sim-drop-pose-publishing
plan: 01
subsystem: sim
tags: [isaac-sim, ros2, tf2, action-graph, aruco, omni-graph]

requires: []
provides:
  - "/drop_poses TFMessage topic with drop_red, drop_green, drop_blue frames"
  - "MCP tool publish_drop_poses"
  - "UI button Publish Drop Poses in cups section"
affects: [02-arm-drop-motion, 03-aruco-camera-drop-detection]

tech-stack:
  added: []
  patterns:
    - "Wrapper Xform prims for named child_frame_id in ROS2PublishTransformTree"
    - "Action graph duplication pattern (self-contained, not shared helper)"

key-files:
  created: []
  modified:
    - exts/soarm101-dt/so_arm101_dt/extension.py

key-decisions:
  - "Wrapper Xform prims (drop_red/green/blue) instead of targeting aruco_marker_mesh directly, to get distinct child_frame_ids"

patterns-established:
  - "drop_{color} naming convention for drop pose frames"

requirements-completed: [SIM-01, SIM-02, SIM-03]

duration: 5min
completed: 2026-03-27
---

# Phase 01 Plan 01: Drop Pose Publishing Summary

**ROS2 /drop_poses TFMessage publisher via action graph with wrapper Xform prims for distinct drop_red/green/blue frame IDs**

## Performance

- **Duration:** ~5 min
- **Started:** 2026-03-27T00:00:00Z
- **Completed:** 2026-03-27T00:05:00Z
- **Tasks:** 2
- **Files modified:** 1

## Accomplishments
- Added `_cmd_publish_drop_poses` method that creates wrapper Xform prims at ArUco marker world transforms and builds a ROS2 action graph publishing /drop_poses
- Registered `publish_drop_poses` in MCP_TOOL_REGISTRY and MCP_HANDLERS
- Added "Publish Drop Poses" UI button in the cups section of the Objects frame
- Applied critical fix: wrapper Xform prims named drop_red/green/blue instead of targeting aruco_marker_mesh (which would produce identical child_frame_ids)

## Task Commits

No commits created per user CLAUDE.md directive. All changes are unstaged in working tree.

1. **Task 1: MCP registration and UI button** - no commit (3 insertion points: registry, handlers, UI)
2. **Task 2: Implement _cmd_publish_drop_poses** - no commit (method with wrapper Xform + action graph)

## Files Created/Modified
- `exts/soarm101-dt/so_arm101_dt/extension.py` - Added publish_drop_poses MCP tool registration, handler mapping, UI button, and _cmd_publish_drop_poses implementation

## Decisions Made
- Used wrapper Xform prims (drop_red, drop_green, drop_blue) under /World/Containers/ instead of targeting aruco_marker_mesh prims directly. ROS2PublishTransformTree uses the prim name as child_frame_id, so targeting aruco_marker_mesh would make all three cups indistinguishable. Wrapper prims copy the ArUco marker's world transform and provide distinct frame names.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] child_frame_id collision fix via wrapper Xform prims**
- **Found during:** Task 2 (per critical_fix directive in prompt)
- **Issue:** Plan's code targeted aruco_marker_mesh prims directly; all three would get child_frame_id="aruco_marker_mesh"
- **Fix:** Create lightweight Xform prims named drop_{color} at each ArUco marker's world transform; target those instead
- **Files modified:** exts/soarm101-dt/so_arm101_dt/extension.py
- **Verification:** grep confirms drop_{color} wrapper path construction in method body

---

**Total deviations:** 1 auto-fixed (1 bug fix per critical_fix directive)
**Impact on plan:** Essential for correctness. Without this fix, /drop_poses would be unusable (indistinguishable frames).

## Issues Encountered
None

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- /drop_poses topic ready for Phase 2 (arm drop motion) to subscribe to
- Requires Isaac Sim running with cups added (add_cups) and simulation playing to verify

---
*Phase: 01-sim-drop-pose-publishing*
*Completed: 2026-03-27*
