# Phase 13 Summary — Port UR5e-dt Backend Features to SO-ARM101

**Completed:** 2026-04-18
**Implementation commit:** `480ddf4` on branch `so-arm101`
**Preceded by:** merge `0f07949` (main → so-arm101, brings ur5e-dt code into the tree)

## Delivered

`exts/soarm101-dt/so_arm101_dt/extension.py` (+424 lines, strictly additive):

- `sys.path` insert for Isaac Sim's Python-3.11 rclpy build — prerequisite for rclpy-based publishers inside Kit.
- `_ViewportCameraPublisher` — active-viewport-reuse only. Sets `viewport.camera_path` and publishes the already-rendered frame to a ROS2 `sensor_msgs/Image` topic via a persistent rclpy node. Zero extra GPU render cost.
- `_RecordingState` — module-level singleton (hot-reload-safe via `globals()` guard). Pipes viewport frames to `ffmpeg` stdin for real-time MP4 recording. `/tmp/isaacsim_recording/recorder_state.json` persists cross-session recording status.
- `VIDEOS_DIR` — defaults to `exts/soarm101-dt/videos/`, overridable via `MCP_CLIENT_OUTPUT_DIR` env.
- 7 new MCP tools:
  - `new_stage` — clears USD stage + pumps Kit updates + clears World singleton.
  - `start_viewport_publisher` / `stop_viewport_publisher` / `list_viewport_publishers`
  - `start_recording` / `stop_recording` / `get_recording_status`
- `on_shutdown` — releases viewport publishers and finalizes any in-flight recording.

## Scope revised mid-phase (data-driven)

Original scope assumed action graphs were expensive and viewport publishers were cheap. Empirical RTF measurement on a clean-restart Isaac Sim proved otherwise:

| Scenario | RTF | Notes |
|---|---|---|
| No camera publishers | 0.435 | Floor |
| Old wrist action graph | 0.429 | Essentially free |
| + active-viewport publisher | 0.584 | No penalty (actually improves FPS due to forced captures) |
| + dedicated hidden viewport | 0.267 | **−0.32 RTF per extra viewport** |

Consequences:
- `dedicated_viewport=True` branch was removed from `start()` before commit — measured RTF penalty made it a liability.
- Wrist camera migration dropped from scope — action graph already cheap, migration would have regressed RTF.
- `execute_python_code` session_id/persistent and stage OPENED listener deferred to a future port phase (not needed for current failure modes).

## Verification (fresh Isaac Sim restart)

13-step end-to-end run passed on `isaacsim_launch.sh restart soarm101-dt`:

1. `quick_start` succeeded
2. `list_viewport_publishers` initial empty ✓
3. Create `/World/workspace_camera_sim` via `execute_python_code` ✓
4. `start_viewport_publisher` registered and started publishing ✓
5. `list_viewport_publishers` showed active entry with frame count ✓
6. `start_recording` ran ffmpeg, reported output path + 1280x720 @ 30fps ✓
7. `get_recording_status` reported 62 frames in 3.04s ✓
8. `stop_recording` finalized 63 frames, 2.10s duration ✓
9. `stop_viewport_publisher` cleaned up ✓
10. `list_viewport_publishers` empty again ✓
11. `new_stage` + `quick_start` lifecycle round-trip ✓
12. Verified MP4 output: 285KB, ISO Media MP4 Base Media v1 ✓
13. `ros2 topic list` shows `/wrist_camera_rgb_sim` (action graph preserved) ✓

## Files changed in repo

- `exts/soarm101-dt/so_arm101_dt/extension.py` (+424, -0)

## Planning artifacts (untracked, local only)

- `.planning/ROADMAP.md` — Phase 13 checkbox flipped to [x], scope revision + "Dropped from scope" captured in phase detail.
- `.planning/STATE.md` — progress 6/13 → 7/13, decisions logged.
- `.planning/phases/13-port-ur5e-features-to-soarm101/13-SUMMARY.md` — this file.

## Known followups (not blocking)

- `/wrist_camera_info` is not published by the wrist action graph's current setup path — if downstream perception needs CameraInfo paired with Image, add it (independent of Phase 13 scope).
- `HiddenVP_*` window class path in the trimmed publisher no longer reachable — could delete dead `width`/`height` parameters on a later cleanup pass.
