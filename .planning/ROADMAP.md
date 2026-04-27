# Roadmap: SO-ARM101 Pick-and-Place Drop Support

## Overview

Deliver the complete pick-and-place pipeline: Isaac Sim publishes known-good cup poses, the SO-ARM101 arm executes drop motions with MoveIt collision avoidance, ArUco camera detection provides real-hardware pose source, and the full pipeline is verified end-to-end. Subsequent phases clean up code, improve motion quality, and prepare for real deployment.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 5.1): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [x] **Phase 1: Sim Drop Pose Publishing** - Isaac Sim extension publishes /drop_poses TFMessage from cup USD positions (completed 2026-03-28)
- [x] **Phase 2: Arm Drop Motion** - SO-ARM101 executes 5-DOF wrist sweep drop with UI buttons and debug services (completed)
- [x] **Phase 3: ArUco Drop Pose Detection** - aruco_camera_localizer detects cup markers and publishes /drop_poses as real-hardware pose source (completed)
- [x] **Phase 5: MoveIt Cup Collision and Planned Drop Motion** - Cup collision objects + MoveIt-planned collision-free drop sweep (completed)
- [x] **Phase 05.1: CAD Cup Mesh Collision** - Replace cylinder collision with convex hull mesh, add colored visual markers and RViz settings tab (completed 2026-03-30)
- [x] **Phase 6: Collision-free Drop Motion Verification Loop** - 3/3 consecutive clean passes for all cups (completed)
- [x] **Phase 7: Control GUI Service Audit & IK Cleanup** - Map every button to service, clean IK control into single execution path (completed 2026-04-18)
- [x] **Phase 07.1: Widget Registry Expansion — Full CLI-Level Control** (INSERTED) - Extend `_button_registry` to cover Spinboxes, Checkbuttons, Entries, Listboxes, Scales. Add `_srv_list_widgets` / `_srv_get_widget_value` / `_srv_set_widget_value` so agents can read/write every interactive widget by name, not just click buttons. (completed 2026-04-18)
- [x] **Phase 07.2: Motion Planning Problem Analysis & Inline Fix** (INSERTED) - FC-1 yaw-fallback + FC-2 OMPL goal-perturb retry + FixStartStateCollision adapters landed. FC-3 (out-of-reach spawns) handled by FC-1's pick-time detection. Residual cycle-8+ cup-tipping root-caused to lego blocks not tracked in MoveIt planning scene — deferred to Phase 9 by design. Verified cross-axis N=10 run: 5/5 reached-spawn cycles clean, 0 cups tipped. (completed 2026-04-19)
- [ ] **Phase 8: Isaac Sim Extension Cleanup** - Button/socket parity, fix cup position update, scene state management, update ArUco marker positions to match real world
- [x] **Phase 9: Collision Scene Completeness** - Tiered deterministic motion planner (tier-1 linear → tier-2 retract-pan-settle → opt-in OMPL fallback) + drop motion robustness (`_attached_lego_tcp_offset` measured offset, `lock_pan` kwarg, hover 30→50 mm, drop_point/sweep slowed to 3.0 s, 5% cup padding) + `randomize_cups` MCP tool with lego/robot exclusions for stress-testing. 27/27 QS cycles PASS, 0 OMPL fallback. (completed 2026-04-24)
- [ ] **Phase 10: Motion Quality & Real Deployment Drop** - Fix drop point orientation, trajectory-based grasp, reduce jerk, camera-guided drop sequence
- [ ] **Phase 10.1: Reconcile + Merge Upstream Exploring-VLAs Main** *(INSERTED)* - Pull 27 unmerged upstream commits (lerobot submodule, mac-env, sim_ground_truth, real-hardware bring-up). Compare divergent control_gui.py + URDF + launch files. Pick best mechanism per overlap. Merge cleanly. Push as single source of truth. **Blocks resumption of Phase 11-01 task 6.**
- [ ] **Phase 11: Full Pick-and-Place Pipeline Verification** - End-to-end YOLOE + ArUco via aruco_camera_localizer (in progress — 11-01 color-driven single-pick workflow, paused at task 6 pending Phase 10.1)
- [ ] **Phase 12: Isaac Sim Joint States Fallback Publisher** - Publish /joint_states from physics engine, auto-backoff, prevent snap-to-zero
- [x] **Phase 13: Port UR5e-dt Backend Features to SO-ARM101** - Active-viewport publisher, `new_stage`, recording backend ported (MCP-only). Wrist action graph kept after empirical RTF test (completed 2026-04-18, commit 480ddf4)

## Phase Details

### Phase 1: Sim Drop Pose Publishing
**Goal**: Isaac Sim extension publishes reliable /drop_poses poses so the arm has known-good targets to work against
**Depends on**: Nothing (first phase)
**Requirements**: SIM-01, SIM-02, SIM-03
**Success Criteria** (what must be TRUE):
  1. Running `ros2 topic echo /drop_poses` shows one TFMessage transform per cup with child_frame_id drop_red, drop_green, drop_blue
  2. Each transform's translation reflects the ArUco marker mesh world position on the cup surface
  3. Calling the MCP tool `publish_drop_poses` (or pressing a UI button) starts the topic publishing without restarting the extension
**Plans**: 1 plan

Plans:
- [x] 01-01-PLAN.md — Add drop_poses action graph, MCP tool, and UI button to extension.py

### Phase 2: Arm Drop Motion
**Goal**: SO-ARM101 can execute a complete drop sequence (point, sweep, release) against a selected /drop_poses target, with each step individually triggerable via button and debug service
**Depends on**: Phase 1
**Requirements**: ARM-01, ARM-02, ARM-03, ARM-04, ARM-05, ARM-06
**Success Criteria** (what must be TRUE):
  1. The grasp tab object listbox populates with drop_0/drop_1/drop_2 entries when /drop_poses is publishing
  2. Pressing "Point to Drop" rotates shoulder_pan to face the selected cup; no other joints move
  3. Pressing "Sweep to Drop" moves wrist_flex from 90° to 0° while shoulder_pan, shoulder_lift, and elbow_flex hold position
  4. Pressing "Release" opens the gripper; object drops into cup
  5. All three drop steps are callable as ROS2 Trigger services (~/drop_point, ~/drop_sweep, ~/drop_release)
  6. Attempting a drop when the selected cup is outside cylindrical workspace bounds produces a logged warning and does not move the arm
**Plans**: 1 plan

Plans:
- [ ] 02-01-PLAN.md — Add drop subscriber, Drop LabelFrame UI, and three _cmd_drop_* motion methods to control_gui.py

### Phase 3: ArUco Drop Pose Detection
**Goal**: aruco_camera_localizer detects cup ArUco markers from the wrist camera and publishes /drop_poses so the arm can drop without Isaac Sim running
**Depends on**: Phase 2
**Requirements**: CAM-01, CAM-02, CAM-03
**Success Criteria** (what must be TRUE):
  1. With wrist camera pointing at cups, `ros2 topic echo /drop_poses` shows transforms for each visible cup (ID 0, 1, or 2 → drop_0, drop_1, drop_2)
  2. Published translation places the drop target above the cup rim, accounting for marker being at 45% cup height
  3. ArUco ID to drop name mapping configurable in robot_config.yaml without code changes
**Plans**: 1 plan

Plans:
- [ ] 03-01-PLAN.md — Add SO-ARM101 cup markers to aruco_config.json, update load_aruco_config() for multi-robot, fix extension.py to use drop_{id} naming

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 5 → 05.1 → 6 → 7 → 07.1 → 07.2 → 8 → 9 → 10 → 11 → 12

| Phase | Status | Completed |
|-------|--------|-----------|
| 1. Sim Drop Pose Publishing | Complete | 2026-03-28 |
| 2. Arm Drop Motion | Complete | 2026-03-28 |
| 3. ArUco Drop Pose Detection | Complete | 2026-03-29 |
| 5. MoveIt Cup Collision and Planned Drop Motion | Complete | 2026-03-29 |
| 05.1. CAD Cup Mesh Collision | Complete | 2026-03-30 |
| 6. Collision-free Drop Motion Verification | Complete | 2026-03-30 |
| 7. Control GUI Service Audit & IK Cleanup | In progress | - |
| 07.1. Widget Registry Expansion — Full CLI-Level Control | Not started | - |
| 07.2. Motion Planning Problem Analysis & Inline Fix | Context gathered | - |
| 8. Isaac Sim Extension Cleanup | Not started | - |
| 9. Collision Scene Completeness | Not started | - |
| 10. Motion Quality & Real Deployment Drop | Not started | - |
| 11. Full Pick-and-Place Pipeline Verification | In progress | - |
| 12. Isaac Sim Joint States Fallback Publisher | Not started | - |

## Phase Details (Upcoming)

### Phase 7: Control GUI Service Audit & IK Cleanup
**Goal**: Every GUI button maps to exactly one service with no duplicated implementation. IK control uses a single execution path — remove duplicate functions and consolidate. Fix drop_refresh to update collision + visual marker poses when cup positions change.
**Depends on**: Phase 6
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. Every button in the GUI has a corresponding `~/service_name` Trigger service
  2. No two services share duplicated motion execution code — single `_execute_trajectory` entry point
  3. IK solve has one code path (geometric_ik → collision check → plan → execute), not multiple variants
  4. `drop_refresh` updates both collision objects AND visual markers to new cup positions in RViz
  5. Agent debug guide updated with complete service list
**Plans**: 5 plans

### Phase 07.1: Widget Registry Expansion — Full CLI-Level Control (INSERTED)
**Goal**: Extend the button-only `_button_registry` (built in Phase 7) into a full widget registry that covers every interactive tkinter widget in control_gui.py — Spinboxes, Checkbuttons, Entries, Listboxes, Scales — and expose generic `~/list_widgets`, `~/get_widget_value`, `~/set_widget_value` Trigger services so agents can read and write any widget by name, not just click buttons. The "CLI-level control" bar: an agent should be able to type an IK target xyz+quat, toggle checkboxes, select a listbox row, and press Plan & Execute entirely via `ros2 service call` + `ros2 param set` — no screen coordinates, no clicks by coordinate.
**Depends on**: Phase 7 (needs `_button_registry` + `_register_button` pattern already in place)
**Requirements**: TBD
**Why inserted after Phase 7 and before Phase 7's remaining plans (07-02/03/04/05)**: Full-widget control unlocks autonomous UAT for Plans 07-03 (pixel-diff screenshot) and 07-04 (IK smoke test that needs to type xyz into spinboxes). Landing 07.1 before them reduces "user at keyboard" UAT steps later.
**Success Criteria** (what must be TRUE):
  1. `self._widget_registry` covers every `tk.Spinbox` / `ttk.Spinbox`, `tk.Checkbutton`, `tk.Entry` / `ttk.Entry`, `tk.Listbox`, `tk.Scale` / `ttk.Scale` constructed in control_gui.py (auditable via a generic `~/list_widgets` Trigger).
  2. `~/get_widget_value` returns the live value of any registered widget by name (Spinbox value, Checkbutton state, Entry text, Listbox selection index+text, Scale position).
  3. `~/set_widget_value` writes a value to any registered widget by name, coercing the parameter string to the widget's expected type (float for Spinbox/Scale, bool for Checkbutton, str for Entry, int row index for Listbox).
  4. AST unit test extends `test_button_service_mapping.py` (or a sibling file) to enforce: every `tk.Spinbox`/`tk.Entry`/`tk.Checkbutton`/etc. construction must go through the registration factories, no direct constructions outside the helpers.
  5. `AGENT_DEBUG_GUIDE.md` regenerated to include a "Widgets" table alongside the "Buttons" table.
  6. End-to-end agent-driven IK flow demonstrated: agent sets X/Y/Z/quaternion via `set_widget_value`, reads back via `get_widget_value`, calls `~/plan_execute` Trigger, all without user interaction.
**Plans**: 0 plans

### Phase 07.2: Motion Planning Problem Analysis & Inline Fix (INSERTED)
**Goal**: Root-cause and fix — inline, within this phase — the three motion-planning flakiness items from 07-PHASE6-REGRESSION.md: FC-1 (no geometric IK at workspace edge), FC-2 (OMPL error −2 on grasp_home post-release), FC-3 (out-of-reach block spawns in randomize_object_poses). Diagnosis grounded in reachable-workspace + configuration-space analysis, backed by N=10 instrumented trials per class, in Isaac Sim only. Phase 10 no longer inherits these items.
**Depends on**: Phase 7, Phase 07.1 (widget/button registries enable cheap N=10 scripted trials)
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. FINDINGS.md published with root-cause statement per class (FC-1, FC-2, FC-3), grounded in analytical evidence (reachable set R for FC-1/FC-3; C-space cluster map + planning-scene audit for FC-2).
  2. N=10 random-seed trial rates captured pre-fix and post-fix for each class, in Isaac Sim.
  3. Reachability filter landed in the `randomize_object_poses` MCP tool; FC-3 post-fix rate → 0.
  4. FC-1 residual rate (post-FC-3 fix) measured and documented; non-zero residuals explained.
  5. FC-2 fix landed (warmup move / budget bump / planning-scene fix — selection data-driven from the C-space + scene audit) with N=10 post-fix verification.
  6. Phase 6 canonical pick-drop regression: 5/5 consecutive clean passes post-fix under random-spawn conditions.
  7. ONE IK path discipline preserved — no reintroduction of a second IK solver; fixes live in the geometric_ik path.
**Plans**: 0 plans

### Phase 8: Isaac Sim Extension Cleanup
**Goal**: All Isaac Sim extension buttons callable via socket, cup position updates don't delete action graphs, scene state save/load works, ArUco marker positions on cups match real-world placement.
**Depends on**: Phase 7
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. Every UI button in the extension has a corresponding socket command
  2. Updating cup positions preserves existing action graphs (no re-creation needed)
  3. Save/load scene state preserves robot pose, cup positions, and action graph configuration
  4. ArUco marker mesh positions on cups match physical marker placement on real cups
**Plans**: 0 plans

### Phase 9: Collision Scene Completeness
**Goal**: Add lego block collision objects to the MoveIt planning scene so the arm avoids legos during drop sweep motions.
**Depends on**: Phase 7
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. Lego blocks from /objects_poses appear as collision objects in the MoveIt planning scene
  2. Collision object dimensions match detected bounding box (from /objects_bbox)
  3. Drop sweep plans avoid both cups AND lego blocks
  4. Collision objects update when objects are re-detected (pick removes the object)
**Plans**: 0 plans

### Phase 10: Motion Quality & Real Deployment Drop
**Goal**: Fix drop point orientation for ergonomic approach, switch to trajectory-based grasp motion (not fixed duration), reduce Isaac Sim jerk by tuning joint drive parameters, and implement camera-guided drop sequence that reads ArUco markers before sweep.
**Depends on**: Phase 8, Phase 9
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. Drop point orientation produces a natural approach angle (arm doesn't contort)
  2. Grasp motion uses trajectory waypoints with velocity profiles, not fixed-duration interpolation
  3. Robot motion in Isaac Sim is smooth — no sudden stops or jerk at trajectory endpoints
  4. Drop sequence: slow approach → read ArUco markers on cup → confirm target → sweep → release
  5. Works with both Isaac Sim and real hardware camera feeds
**Plans**: 0 plans

### Phase 10.1: Reconcile + Merge Upstream Exploring-VLAs Main *(INSERTED)*
**Goal**: Resolve the 27 unmerged upstream commits on `origin/main` (lerobot submodule add, mac-env, sim_ground_truth + cup containers + /drop_poses sim parity, real-hardware bring-up, ROS2 plugins docs) against the 26 local commits (Drop Scan / Real Test tab + deterministic motion planner + drop motion robustness). Compare overlapping mechanisms (control_gui.py, URDF, /drop_poses producers). Pick the better implementation per overlap and merge into a single coherent main. Push so collaborators see one source of truth.
**Why URGENT**: Discovered during Phase 11-01 task 6 wrap-up — divergence has been growing for ~2 weeks across the team. Several Phase 11-01 issues this session (sim_ground_truth /drop_poses overlap with isaac-sim-mcp's publisher, missing real-hardware bring-up infrastructure, no lerobot data recording) would have been avoided or framed differently if upstream had been pulled regularly. Continuing Phase 11-01 task 6 (UAT) without first reconciling means double-resolving the same conflicts later.
**Depends on**: Phase 9 (current local main lineage); blocks Phase 11-01 task 6 + lego workflow.
**Requirements**: TBD (to be derived during /gsd-plan-phase)
**Success Criteria** (what must be TRUE):
  1. `git status` on `vla_SO-ARM101` shows `[main: ahead 0, behind 0]` after merge — no divergence remaining.
  2. lerobot submodule cloned and initialized at the upstream-defined path; submodule pointer matches upstream.
  3. control_gui.py final state preserves: deterministic motion planner (tier-1/tier-2/OMPL), Drop Scan + Real Test tab, sim/real sub leak fix, gripper open planning-group switch — AND any improved equivalents from upstream (e.g. jointstatereader dual-publish if better than local).
  4. /drop_poses producer overlap resolved: ONE authoritative producer (either upstream sim_ground_truth or local isaac-sim-mcp soarm101-dt) — the other deferred or removed.
  5. URDF / launch / config conflicts resolved by side-by-side comparison; explicit log of which side won per file.
  6. Merge commit pushed to origin/main; downstream collaborators can pull cleanly.
  7. Phase 11-01 Drop Scan still works end-to-end on the merged branch (regression check before declaring complete).
**Plans**: 0 plans (to be created during /gsd-plan-phase 10.1)

### Phase 11: Full Pick-and-Place Pipeline Verification
**Goal**: Verify the complete pick-and-place pipeline end-to-end using YOLOE for object detection and ArUco markers for drop pose detection via the aruco_camera_localizer package.
**Depends on**: Phase 10, Phase 10.1
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. YOLOE detects lego blocks and publishes poses on /objects_poses
  2. ArUco localizer detects cup markers and publishes /drop_poses
  3. Full cycle: detect block → pick → detect cup → drop → verify block in cup
  4. Works for all 3 colors (red block → red cup, etc.)
  5. No manual intervention needed between pick and place
**Plans**: 0 plans

### Phase 12: Isaac Sim Joint States Fallback Publisher
**Goal**: Isaac Sim publishes /joint_states from its physics engine when no external publisher (joint_state_broadcaster) exists, and auto-backs-off when one appears. GUI reads actual joint positions on startup, preventing snap-to-zero on control stack restart.
**Depends on**: Phase 1
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. With control stack down, `ros2 topic echo /joint_states` shows Isaac Sim publishing joint positions matching the simulated robot state
  2. After launching control stack, Isaac Sim stops publishing /joint_states within 5s (joint_state_broadcaster takes over)
  3. GUI initializes joint_positions from /joint_states before sending any commands — no snap-to-zero
  4. Works across quick_start, hot-reload, and control stack restart cycles
**Plans**: 0 plans

### Phase 13: Port UR5e-dt Backend Features to SO-ARM101 (scope revised after empirical RTF measurement)
**Goal**: Add active-viewport-reuse ROS2 camera publishing, `new_stage` MCP tool, and video recording backend from ur5e-dt into soarm101-dt. Discovery during spike: dedicated-hidden-viewport publishing costs ~0.3 RTF per extra viewport (expensive), while the existing soarm101 wrist-camera action graph is cheap (~0 RTF cost). Revised scope keeps the action graph intact and only adds the genuinely-free active-viewport-reuse path for workspace/display cameras.
**Depends on**: Phase 1
**Requirements**: TBD
**Success Criteria** (what must be TRUE):
  1. `_ViewportCameraPublisher` class ported with active-viewport-reuse ONLY (no dedicated hidden viewports — measured RTF penalty).
  2. MCP tools `start_viewport_publisher` / `stop_viewport_publisher` / `list_viewport_publishers` point the active viewport at a camera prim and publish its frames to a ROS2 Image topic.
  3. `new_stage` MCP tool clears USD stage + PhysX state cleanly; existing `quick_start` rebuilds afterwards.
  4. Video recording MCP tools (`start_recording`, `stop_recording`, `get_recording_status`) produce valid MP4, do not block physics, and correctly report status across calls.
  5. rclpy 3.11 path fix applied at module top so rclpy-based publishing works in Isaac Sim 3.11 runtime.
  6. `on_shutdown` releases viewport publishers and finalizes any in-flight recording.
  7. Existing wrist-camera action graph is **preserved** — no migration, no regression on `/wrist_camera_rgb_sim`.
  8. No UI widgets added; all new functionality accessible via MCP tools only.
  9. Both ur5e-dt and soarm101-dt continue to work independently on their respective robots.
**Plans**: 0 plans

**Dropped from scope** (based on spike findings):
  - `execute_python_code` session_id/persistent — orthogonal, defer to a future port phase.
  - Stage OPENED event listener — the hot-reload contamination issue we hit was an ur5e-dt `LogRedirector` stdout hijack, fixed by clean restart; soarm101 doesn't need an OPENED listener for its current failure modes.
  - Migrating wrist camera from action graph to viewport publisher — action graph is already cheap; would have INCREASED RTF cost.
  - `dedicated_viewport=True` branch in the publisher class — measured −0.32 RTF per extra viewport, not worth the code surface.
