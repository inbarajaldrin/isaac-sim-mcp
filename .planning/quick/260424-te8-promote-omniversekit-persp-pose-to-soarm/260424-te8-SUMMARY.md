---
status: complete   # verified 2026-04-25: viewport switched, ROS2 topic publishing 17.15 Hz, pose matches spec
quick_id: 260424-te8
phase: quick
plan: te8
subsystem: soarm101-dt
tags: [viewport, camera, quick_start, ros2-publisher]
requirements: [TE-8]
dependency-graph:
  requires:
    - "_viewport_pub singleton at module level (extension.py:208-209)"
    - "ur5e-dt _set_active_viewport_camera reference (ur5e-dt/extension.py:1774-1791)"
  provides:
    - "Auto-rendered workspace view at /World/workspace_camera_sim after quick_start"
    - "/workspace_camera_rgb_sim ROS2 topic (~30 Hz, ~0 RTF cost via active-viewport reuse)"
    - "_set_active_viewport_camera(camera_path) helper on DigitalTwin"
  affects:
    - "soarm101-dt manual UI 'Create Camera' button (workspace branch — pose updated)"
    - "Operator UX: viewport auto-switches on quick_start"
tech-stack:
  added:
    - "omni.kit.viewport.utility (already available in Kit)"
  patterns:
    - "Skip-if-exists guard on prim creation"
    - "Active-viewport reuse for zero-RTF-cost ROS2 image publishing (Phase 13)"
key-files:
  created: []
  modified:
    - exts/soarm101-dt/so_arm101_dt/extension.py
decisions:
  - "Option 3b (defer ComboBox UI picker) — keeps change atomic; TODO marker on helper for future option 3a"
  - "Active-viewport reuse path (not dedicated hidden viewport) — Phase 13 measured ~0.32 RTF cost per dedicated viewport vs ~0 for active reuse"
metrics:
  duration: ~10 min (executor)
  completed: 2026-04-24
---

# Quick Task 260424-te8: Promote OmniverseKit_Persp Pose to SO-ARM101 Workspace Camera Summary

**One-liner:** Auto-create `/World/workspace_camera_sim` at SO-ARM101-tuned pose during `quick_start`, start its viewport publisher, and switch the active Isaac Sim viewport to render through it — operator sees the overhead-front robot + cup arc view immediately after Quick Start with zero manual steps.

## What was built

Four atomic edits in `exts/soarm101-dt/so_arm101_dt/extension.py`:

1. **Manual UI pose update (line 2382)** — `if is_workspace:` branch in `create_additional_camera()` now uses the SO-ARM101-tuned pose `position=(0.6980, -0.3958, 0.4525)` and `quat_xyzw=(0.4307, 0.2612, 0.4480, 0.7386)` instead of the stale UR5e-derived values.

2. **`quick_start()` step 10b (lines 1465–1524)** — new block inserted between `self._timeline.play()` and the contact-sensor setup. Defines `/World/workspace_camera_sim` (1280×720, RealSense intrinsics: HFOV 69.4° / VFOV 42.5°, opencvPinhole lens distortion model with zeroed k/p/s coefficients) only if the prim does not already exist; logs `"Workspace camera already exists at … skipping creation."` on the idempotent re-run path.

3. **`_set_active_viewport_camera(camera_path)` helper (lines 2481–2500)** — new method on `DigitalTwin`. Wraps `omni.kit.viewport.utility.get_active_viewport()` + `viewport.camera_path = Sdf.Path(camera_path)` in try/except. Records previous path in `self._prev_viewport_cam` for potential future undo. Carries a TODO marker referencing option 3a (ComboBox picker) deferred to a future task.

4. **Auto-switch wiring in `quick_start()`** — after the camera-create block, `_viewport_pub.start(ws_prim_path, "workspace_camera_sim", frame_id="workspace_camera_sim")` is called only when `is_active("workspace_camera_sim")` is False; one `await app.next_update_async()` tick later, `self._set_active_viewport_camera(ws_prim_path)` switches the active viewport.

The existing two `await app.next_update_async()` ticks before contact-sensor setup are preserved, so the play → sensor-init flow is unchanged.

## Files changed

- `exts/soarm101-dt/so_arm101_dt/extension.py` (+85 / −3)

## Commits

- `480be86` — `feat(soarm101-dt): auto-create + auto-switch to workspace_camera_sim on quick_start`

## Deviations from Plan

None — the four edits land verbatim per the plan's `<action>` block. The helper docstring includes the TODO marker for option 3a as specified.

## Verification

**Automated (executor):**
- `python3 -m py_compile exts/soarm101-dt/so_arm101_dt/extension.py` → SYNTAX OK
- `grep "0.6980338662434342"` → matches at lines 1474 (quick_start block) and 2382 (manual UI block) — pose present in both call sites
- `grep "_set_active_viewport_camera"` → matches at lines 1523 (call from quick_start) and 2481 (helper definition)
- `grep "_viewport_pub.start.*workspace"` → matches at line 1518
- `grep "skipping creation"` → matches at line 1471 (idempotent path)
- Post-commit: no file deletions, no untracked files outside `.planning/`

**Human verification required (Task 2 of plan — checkpoint:human-verify):**
1. `touch exts/soarm101-dt/so_arm101_dt/extension.py` to hot-reload
2. Click "Quick Start" in soarm101-dt control GUI; expect Kit log lines:
   - `--- Creating workspace camera ---`
   - `Workspace camera created at /World/workspace_camera_sim with resolution 1280x720` (or `already exists … skipping creation` on re-run)
   - `Workspace camera viewport publisher started.`
   - `Active viewport → /World/workspace_camera_sim`
3. Isaac Sim viewport should switch to overhead-front robot + cup arc view
4. `ros2 topic hz /workspace_camera_rgb_sim` → expect ~30 Hz
5. Click Quick Start a second time → expect skip log, no exception
6. Check Isaac Sim HUD or `ros2 topic hz /clock` → RTF should stay near 1.0 (Phase 13 active-viewport-reuse baseline)

## Threat Flags

None — surface introduced (workspace camera prim + ROS2 image topic + active viewport rebind) is identical to ur5e-dt's existing pattern that has been in production. Failure modes already accepted in plan threat register (T-te8-01, T-te8-02): wrapped in try/except; quick_start continues on failure.

## Self-Check: PASSED
- File present: `/home/aaugus11/Documents/isaac-sim-mcp/exts/soarm101-dt/so_arm101_dt/extension.py` (modified)
- Commit present: `480be86` (feat(soarm101-dt): auto-create + auto-switch …)
- All four `<done>` criteria from plan Task 1 satisfied (verified via grep)
