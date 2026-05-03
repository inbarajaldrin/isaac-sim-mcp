---
phase: 02-controller-loop
plan: 04
subsystem: controller-loop
status: complete
tags:
  - phase-2
  - parity-10
  - pose-command-application
  - lula-ik
  - articulation-kinematics-solver
  - d-02
  - d-06
  - d-11
  - pitfall-2-option-a
  - pitfall-4
requirements: [PARITY-10]
dependency_graph:
  requires:
    - 02-02 (controller_loop.py skeleton — _setup_kinematics + _apply_pose_cmd stubs)
    - 02-03 (module-level constants TARGET_MODE_CARTESIAN, MODE_UNSPECIFIED; bookkeeping field _last_target_mode)
    - 02-01 (aic_control_interfaces.MotionUpdate built into 3.11 workspace)
    - Phase 1 SCENE-04 (USD prim hierarchy: /World/UR5e/aic_unified_robot/{tool0, gripper/tcp})
  provides:
    - "AicControllerLoop._setup_kinematics: ready Lula IK + cached tool0<->tcp SE(3) offsets (4x4 numpy)"
    - "AicControllerLoop._on_pose_cmd: validated MotionUpdate buffering"
    - "AicControllerLoop._apply_pose_cmd: pose -> joint via Lula, with gripper/tcp ingress transform"
    - "self._tool0_to_tcp_offset_xform (4x4 numpy SE(3)) — egress matrix Plan 02-05 reuses for tcp_pose ControllerState publish"
    - "self._tcp_to_tool0_offset_xform (4x4 numpy SE(3)) — ingress matrix used in _apply_pose_cmd"
    - "self._last_reference_tcp_pose + self._last_target_mode = TARGET_MODE_CARTESIAN — Plan 02-05 ControllerState bookkeeping"
  affects:
    - "Plan 02-05 (PARITY-11): reuses self._kinematics for FK via .compute_end_effector_pose() AND self._tool0_to_tcp_offset_xform for tool0 -> tcp egress when populating ControllerState.tcp_pose"
    - "Plan 02-06 (PARITY-06 + smoke): smoke_test_aic_controller.py should send a MotionUpdate with header.frame_id='gripper/tcp' to validate the Pitfall 2 Option A path end-to-end"
tech_stack:
  added:
    - "isaacsim.robot_motion.motion_generation.lula.kinematics.LulaKinematicsSolver (Pitfall 1: NOT bare isaacsim.robot_motion.lula)"
    - "isaacsim.robot_motion.motion_generation.ArticulationKinematicsSolver"
    - "scipy.spatial.transform.Rotation (used inside gripper/tcp transform path; bundled with isaacsim env)"
    - "pxr.UsdGeom.XformCache + pxr.Gf.Matrix4d (used inside _setup_kinematics for static SE(3) capture)"
  patterns:
    - "Lula IK from bundled UR5e config (no asset vendoring; uses ~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/.../motion_policy_configs/universal_robots/ur5e/{ur5e.urdf, rmpflow/ur5e_robot_description.yaml})"
    - "Pitfall 2 Option A: end_effector_frame_name='tool0' for IK; static tool0<->gripper/tcp SE(3) offset captured at _setup_kinematics time and applied at _apply_pose_cmd ingress (NO drop+log fallback)"
    - "Pitfall 4: ROS quat (x,y,z,w) -> Lula quat (w,x,y,z) conversion at the boundary"
    - "D-06 ignored-field discipline: target_stiffness[36], feedforward_wrench_at_tip, wrench_feedback_gains_at_tip[6] logged at debug + ignored (Isaac Sim is the hardware sink, not the impedance solver)"
    - "D-11 drop-silently failure mode: every external API call wrapped in try/except with self._logged_apply_error log-once spam guard"
    - "PyKDL fallback path documented inline (D-02) but NOT implemented — research recommendation: defer until Lula proves insufficient"
key_files:
  modified:
    - exts/aic-dt/aic_dt/controller_loop.py
  created: []
decisions: []
metrics:
  duration_min: 3
  loc_added: 274
  loc_removed: 6
  files_modified: 1
  files_created: 0
  task_count: 2
  commits: [cc5ec52, b31732b]
  completed_at: "2026-05-03T20:40:03Z"
---

# Phase 2 Plan 02-04: PARITY-10 (pose_commands subscriber + Lula IK + Pitfall 2 Option A) Summary

PARITY-10 wired end-to-end in `controller_loop.py`: `/aic_controller/pose_commands` is now subscribed (MotionUpdate), validated against `aic_controller.cpp:218-227` (frame_id ∈ {`base_link`, `gripper/tcp`}, mode ≠ UNSPECIFIED), and resolved to joint targets through `LulaKinematicsSolver` wrapped in `ArticulationKinematicsSolver`. `gripper/tcp` frame targets are pre-multiplied by the static `T_tcp_to_tool0` SE(3) cached at `_setup_kinematics` time (Pitfall 2 Option A — Open Question Q3 RESOLVED, no drop+log fallback). ROS↔Lula quaternion order is converted at the boundary (Pitfall 4). Cartesian impedance and feed-forward wrench fields are logged at debug and ignored per D-06 (Isaac Sim is the hardware sink; double-applying corrupts `aic_controller`'s intent). Bookkeeping fields (`_last_reference_tcp_pose`, `_last_target_mode = TARGET_MODE_CARTESIAN`) populated for Plan 02-05's ControllerState publish.

## What Changed

| File | LOC delta | Note |
|------|-----------|------|
| `exts/aic-dt/aic_dt/controller_loop.py` | +274 / -6 (565 → 839) | `_setup_kinematics` (full body, +121); `_on_pose_cmd` validation (+22); `_apply_pose_cmd` full body (+138); `__init__` + `stop()` add cached SE(3) matrix attrs |

### `_setup_kinematics` (Task 1 — commit cc5ec52)
- Imports `LulaKinematicsSolver` from `isaacsim.robot_motion.motion_generation.lula.kinematics` (Pitfall 1: NOT bare `isaacsim.robot_motion.lula`).
- Resolves the bundled UR5e config path via `os.path.dirname(motion_generation.__file__)` walk-up — no asset copying, no vendoring.
- Constructs `ArticulationKinematicsSolver(end_effector_frame_name="tool0", ...)` (Pitfall 2: `gripper/tcp` is NOT in the bundled UR5e URDF).
- Captures the static `tool0 → gripper/tcp` SE(3) offset by `UsdGeom.XformCache.GetLocalToWorldTransform` on `/World/UR5e/aic_unified_robot/{tool0, gripper/tcp}` and `Gf.Matrix4d` inverse composition; both directions cached as 4×4 `numpy.float64` arrays:
  - `self._tool0_to_tcp_offset_xform` — egress (Plan 02-05 reuses for `tcp_pose` ControllerState publish)
  - `self._tcp_to_tool0_offset_xform` — ingress (used here in `_apply_pose_cmd`)
- If the offset capture fails, the matrices stay `None` and the `_apply_pose_cmd` guard refuses `gripper/tcp` commands rather than producing wrong joint targets. `base_link` IK still works.
- `__init__` initializes both matrix attributes to `None`; `stop()` nulls them for clean hot-reload.

### `_on_pose_cmd` (Task 2 — commit b31732b)
- Validates `msg.header.frame_id ∈ {"base_link", "gripper/tcp"}`; drops with debug log otherwise.
- Drops on `trajectory_generation_mode.mode == MODE_UNSPECIFIED`.
- Buffers to `self._latest_pose_cmd` for the next physics tick (D-08: callbacks MUST NOT apply directly).
- Wrapped try/except — never raises into the rclpy spin path (D-11).

### `_apply_pose_cmd` (Task 2 — commit b31732b)
- Early-out if `self._articulation is None or self._kinematics is None` (pre-play / IK setup not ready).
- **gripper/tcp ingress transform (Pitfall 2 Option A)**: builds `T_world_tcp = (R_msg | t_msg)` from the message pose using `scipy.spatial.transform.Rotation`, composes `T_world_tool0 = T_world_tcp @ T_tcp_to_tool0`, decomposes back to `(position, quaternion)`, and re-packs the quaternion as `(w, x, y, z)` for Lula. Logs translation diff at debug.
- **D-06 ignored-field surfacing**: any non-zero `target_stiffness` or `feedforward_wrench_at_tip.force` triggers a one-line debug log and is then ignored.
- **Pitfall 4 quat conversion**: in the `base_link` path, target_orientation is built directly as `(w, x, y, z)` from `(orientation.w, orientation.x, orientation.y, orientation.z)`.
- Calls `self._kinematics.compute_inverse_kinematics(target_position, target_orientation)`; on `success=False`, drops silently per D-11.
- Applies the resulting `ArticulationAction` via `self._articulation.apply_action(ik_action)`.
- Bookkeeping for Plan 02-05: `self._last_reference_tcp_pose = msg.pose`, `self._last_target_mode = TARGET_MODE_CARTESIAN`.
- PyKDL fallback documented as a comment block at end of method body — not implemented (D-02).

## Bundled UR5e Config Confirmation

`ls ~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/universal_robots/ur5e/`:

```
rmpflow
ur5e.urdf
```

`ls .../ur5e/rmpflow/`:

```
config.json
ur5e_rmpflow_config.yaml
ur5e_robot_description.yaml
```

Both `ur5e.urdf` and `rmpflow/ur5e_robot_description.yaml` exist at the resolved path. `_setup_kinematics`'s `os.path.exists(...)` guards will succeed at runtime; no asset vendoring needed.

## Verification Gates (all PASS)

```text
AST parse                                 OK
_setup_kinematics def count               1
_on_pose_cmd def count                    1
_apply_pose_cmd def count                 1
"STUB — Plan 02-04" markers               0 (removed)
LulaKinematicsSolver import + usage       OK
ArticulationKinematicsSolver import+use   OK
end_effector_frame_name="tool0"           OK
compute_inverse_kinematics                OK
msg.pose.orientation.w (Pitfall 4)        OK
quat order w then x in non-override path  OK
msg.header.frame_id == "gripper/tcp"      OK
_tool0_to_tcp_offset_xform count          6  (>= 2 required)
_tcp_to_tool0_offset_xform count          10 (>= 2 required)
GetLocalToWorldTransform                  OK
NO drop+log "transform not yet impl"      OK
target_stiffness debug log                OK
feedforward_wrench_at_tip debug log       OK
self._last_reference_tcp_pose = msg.pose  OK
TARGET_MODE_CARTESIAN bookkeeping         OK
self._tcp_to_tool0_offset_xform usage
   inside _apply_pose_cmd                 4 occurrences
```

## Deviations from Plan

None — plan executed exactly as written. Both tasks' acceptance criteria passed on first run; the only minor adjustment was a defensive `__init__` + `stop()` initialization of the two SE(3) matrix attributes to `None`, so the `is None` guard in `_apply_pose_cmd` is well-defined even if `_setup_kinematics` is never called (e.g., articulation not initialized at start time). This is consistent with the existing skeleton's discipline (`_kinematics`, `_articulation`, etc. all initialized to `None` in `__init__` and nulled in `stop()`); not a behavioral change.

## Hand-off to Plan 02-05 (PARITY-11)

- **`self._kinematics` is ready** — Plan 02-05 reuses it for FK via `.compute_end_effector_pose()` (no need to re-init).
- **Egress transform is cached** — `self._tool0_to_tcp_offset_xform` is the matrix Plan 02-05 should post-multiply onto the FK tool0 pose to populate `ControllerState.tcp_pose`. Specifically:
  ```
  T_world_tool0 = FK(joint_state)              # via self._kinematics
  T_world_tcp   = T_world_tool0 @ self._tool0_to_tcp_offset_xform
  ```
  The two SE(3) matrices are 4×4 numpy `float64` in row-major Pixar convention (matching how `Gf.Matrix4d` stores them); same composition order and same convention as the ingress path here, just the inverse direction.
- **Bookkeeping is populated** — `self._last_reference_tcp_pose` (= `geometry_msgs/Pose` from the most-recent applied MotionUpdate) and `self._last_target_mode` (= `TARGET_MODE_CARTESIAN` after a pose cmd, `TARGET_MODE_JOINT` after a joint cmd from Plan 02-03) are ready for ControllerState's reference-pose echo.

## Hand-off to Plan 02-06 (smoke test)

- **Add a gripper/tcp MotionUpdate to `smoke_test_aic_controller.py`**: send a `MotionUpdate` with `header.frame_id='gripper/tcp'` and verify the resulting articulation movement places `tool0` at the correct offset distance from the commanded pose. This is a standalone smoke step beyond D-12's 7-step contract (the 7-step canon doesn't differentiate `base_link` vs `gripper/tcp`); OK to log as best-effort if the trial doesn't surface gripper/tcp commands in practice.
- **Negative-validation smoke**: send a MotionUpdate with `header.frame_id='world'` (or some other invalid frame) and verify it's dropped silently with a debug log — confirms `_on_pose_cmd` validation gate. Same for `MODE_UNSPECIFIED` mode.
- **D-06 quiet-mode smoke**: send a MotionUpdate with non-zero `target_stiffness` and confirm Isaac Sim does NOT change its internal joint stiffness (verifies the log-and-ignore path, not log-and-act).

## Self-Check: PASSED

**Files exist:**
- `exts/aic-dt/aic_dt/controller_loop.py` — FOUND (839 LOC)
- `.planning/phases/02-controller-loop/02-04-SUMMARY.md` — FOUND (this file)

**Commits exist:**
- `cc5ec52` — FOUND (Task 1: `_setup_kinematics` with Lula IK + tool0↔tcp SE(3) cache)
- `b31732b` — FOUND (Task 2: `_on_pose_cmd` validation + `_apply_pose_cmd` with Pitfall 2 Option A)
