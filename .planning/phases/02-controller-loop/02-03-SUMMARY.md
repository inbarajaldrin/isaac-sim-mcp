---
phase: 02-controller-loop
plan: 03
subsystem: aic-dt extension / controller loop
tags:
  - phase-2
  - parity-09
  - joint-command-application
  - articulation-apply-action
  - set-gains
  - name-keyed-mapping
  - d-06
  - d-09
  - d-11
dependency-graph:
  requires:
    - "Plan 02-01 (D-05 ABI fix — aic_control_interfaces.JointMotionUpdate buildable for cp311)"
    - "Plan 02-02 (controller_loop.py skeleton — _on_physics_step lifecycle + _latest_joint_cmd buffer + _logged_apply_error log-once flag)"
    - "Phase 1 SCENE-04 (Articulation handle initialization — /World/UR5e/aic_unified_robot/root_joint)"
  provides:
    - "PARITY-09 surface complete: /aic_controller/joint_commands subscriber validates per aic_controller.cpp:236-330 + applies positions/gains/efforts via Articulation.apply_action + set_gains per D-06"
    - "Module-level constants for downstream plans: ARM_JOINTS (UR5e DOF set), GRIPPER_NOOP (slashed FixedJoint name), MODE_*/TARGET_MODE_* enums, N_ARM_JOINTS=6"
    - "Bookkeeping fields populated for Plan 02-05 ControllerState publish: _last_reference_joint_state (target JointTrajectoryPoint), _last_target_mode (TARGET_MODE_JOINT)"
  affects:
    - "exts/aic-dt/aic_dt/controller_loop.py (+177/-5; module constants + _on_joint_cmd validation body + _apply_joint_cmd full body)"
tech-stack:
  added:
    - "isaacsim.core.utils.types.ArticulationActions (lazy-imported inside _apply_joint_cmd to keep the class definition rclpy/Kit-free per Plan 02-02 contract)"
    - "Articulation.set_gains(kps, kds, joint_names=...) — per-joint stiffness/damping override path (D-06)"
    - "Articulation.apply_action(ArticulationActions(joint_positions=..., joint_efforts=..., joint_names=...)) — combined position + feedforward effort path (D-06)"
  patterns:
    - "Name-keyed parser per D-09: iterate msg.target_state.joint_names (NOT positional alignment); apply_action handles name→DOF-index resolution internally; gripper/left_finger_joint silently no-op'd as zero-DOF FixedJoint; unknown names log warn + skip without failing the whole message"
    - "Drop-silently failure mode per D-11: bad commands logged at debug + return; physics callback never raises; set_gains/apply_action exceptions caught + log-once'd via _logged_apply_error to avoid spam (mirrors parity_publishers.py:_logged_publish_error pattern)"
    - "Apply gains BEFORE positions: set_gains first so the next PD step uses the new stiffness/damping when computing torque from the new position target"
    - "Lazy numpy/ArticulationActions imports inside _apply_joint_cmd: preserves the Plan 02-02 contract that the class body itself is rclpy/Kit-free for offline structural verification"
key-files:
  created:
    - ".planning/phases/02-controller-loop/02-03-SUMMARY.md (this file)"
  modified:
    - "exts/aic-dt/aic_dt/controller_loop.py (+177/-5; 393 → 565 LOC)"
decisions:
  - "TrajectoryGenerationMode enum corrected per the source-of-truth message (Rule 1 deviation): VELOCITY=1, POSITION=2 — plan body had them inverted (POSITION=1, VELOCITY=2). Without this fix, every position command would be misrouted into the velocity-validation branch and dropped, while velocity commands would pass through unvalidated. Verified against ~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/TrajectoryGenerationMode.msg."
  - "Per-flag size-validation gate (has_positions / has_efforts / has_stiffness / has_damping) checked against len(msg_joint_names), not against N_ARM_JOINTS. Reason: the message could legally include the gripper joint (taking len = 7) per aic_adapter::joint_sort_order_; the parser still extracts only the 6 arm DOFs after the loop's GRIPPER_NOOP skip. This is more permissive than the plan's draft (which assumed n=6 throughout) and matches the live aic_controller behavior."
  - "Lazy `import numpy` + `from isaacsim.core.utils.types import ArticulationActions` inside _apply_joint_cmd (NOT at module top). Preserves Plan 02-02's contract that the class definition is loadable offline (no Kit / no Isaac Sim core imports at module load). Trade-off: per-tick import overhead is non-zero but cached after first call (Python's import system memoizes); negligible vs. the physics step cost."
  - "Cartesian impedance fields (target_pose, target_twist, target_acceleration) explicitly NOT touched here. Those are MotionUpdate-side and belong to Plan 02-04. JointMotionUpdate carries only joint-space data + the optional Cartesian-equivalent stiffness/damping/feedforward, all of which we DO honor via D-06."
  - "set_gains call wrapped in its own try/except (independent from apply_action's try/except). Reason: Pitfall 6 — set_gains is a silent no-op pre-play; if the underlying physics_sim_view isn't ready it raises rather than returns. Catching independently means a gains failure doesn't block the position application from going through, and vice versa."
  - "Bookkeeping (_last_reference_joint_state + _last_target_mode) set ONLY after apply_action succeeds, not after set_gains. ControllerState.last_target_mode should reflect what was actually applied to the articulation, not what the validator accepted. If apply_action raises, the bookkeeping stays stale until the next successful command — Plan 02-05 will publish the previous target until then, which is the correct behavior (controller_state mirrors the most-recently-applied command)."
metrics:
  duration: "2 minutes"
  completed-date: 2026-05-03
  tasks-completed: 1
  tasks-total: 1
  files-created: 1
  files-modified: 1
---

# Phase 02-controller-loop Plan 03 Summary

**One-liner:** PARITY-09 wired end-to-end — `/aic_controller/joint_commands` subscriber validates incoming `JointMotionUpdate` per `aic_controller.cpp`, name-keyed parser routes per-joint stiffness/damping to `Articulation.set_gains` and positions+optional feedforward torque to `Articulation.apply_action(ArticulationActions(...))` per D-06, gripper FixedJoint silently no-op'd per D-09, all failures swallowed at debug per D-11.

## What changed

**`exts/aic-dt/aic_dt/controller_loop.py` (+177/-5):**

1. **Module-level constants block (after the path-discipline section, before `class AicControllerLoop`):**
   - `ARM_JOINTS` — set of 6 UR5e DOF names from `aic_adapter::joint_sort_order_`
   - `GRIPPER_NOOP = "gripper/left_finger_joint"` — the slashed FixedJoint zero-DOF that's silently dropped
   - `MODE_UNSPECIFIED=0`, `MODE_VELOCITY=1`, `MODE_POSITION=2` — `TrajectoryGenerationMode.msg` enum
   - `TARGET_MODE_UNSPECIFIED=0`, `TARGET_MODE_CARTESIAN=1`, `TARGET_MODE_JOINT=2` — `TargetMode.msg` enum
   - `N_ARM_JOINTS = 6`

2. **`_on_joint_cmd(msg)` — replaced stub with validation body:**
   - Drops MODE_UNSPECIFIED with debug log
   - Validates positions size when MODE_POSITION; velocities size when MODE_VELOCITY
   - Validates stiffness/damping size if non-empty (D-06 per-joint contract)
   - On pass → buffers to `self._latest_joint_cmd`
   - All wrapped in try/except — never raises into the rclpy spin (D-11)

3. **`_apply_joint_cmd(msg)` — replaced stub with full implementation:**
   - Pre-play guard: returns early if `self._articulation is None`
   - Builds name-keyed `arm_names` / `arm_positions` / `arm_efforts` / `arm_kps` / `arm_kds` lists by iterating `msg.target_state.joint_names`
   - Skips `GRIPPER_NOOP` (silent no-op); warns + skips unknown names
   - `Articulation.set_gains(kps, kds, joint_names=arm_names)` — applied first
   - `Articulation.apply_action(ArticulationActions(joint_positions=..., joint_efforts=..., joint_names=arm_names))` — applied second
   - Both API calls wrapped in independent try/except + log-once
   - Updates `_last_reference_joint_state` + `_last_target_mode = TARGET_MODE_JOINT` only after a successful `apply_action`

## How it integrates with the existing skeleton

The skeleton's `_on_physics_step` (Plan 02-02) automatically picks up the new callback bodies — it already calls `rclpy.spin_once` (which triggers `_on_joint_cmd` when a message arrives), then calls `self._apply_joint_cmd(self._latest_joint_cmd)` if buffered, with its own try/except wrapping. **No changes needed in `extension.py`, `quick_start`, or anywhere else.** The MCP atom `setup_controller_subscribers` from Plan 02-02 starts the loop, which now begins applying joint commands the moment the first `JointMotionUpdate` arrives.

## Verification (per plan acceptance criteria — all pass)

```bash
python3 -c "import ast; ast.parse(open('exts/aic-dt/aic_dt/controller_loop.py').read())"  # exit 0
grep -q "ARM_JOINTS = {" exts/aic-dt/aic_dt/controller_loop.py                            # OK
grep -q "GRIPPER_NOOP = \"gripper/left_finger_joint\"" exts/aic-dt/aic_dt/controller_loop.py  # OK
grep -q "MODE_POSITION = 2" exts/aic-dt/aic_dt/controller_loop.py                         # OK (note: NOT =1)
grep -q "MODE_VELOCITY = 1" exts/aic-dt/aic_dt/controller_loop.py                         # OK (note: NOT =2)
grep -q "TARGET_MODE_JOINT = 2" exts/aic-dt/aic_dt/controller_loop.py                     # OK
grep -q "from isaacsim.core.utils.types import ArticulationActions" exts/aic-dt/aic_dt/controller_loop.py  # OK
grep -q "self._articulation.apply_action" exts/aic-dt/aic_dt/controller_loop.py           # OK
grep -q "self._articulation.set_gains" exts/aic-dt/aic_dt/controller_loop.py              # OK
grep -q "trajectory_generation_mode.mode" exts/aic-dt/aic_dt/controller_loop.py           # OK
grep -q "target_state.joint_names" exts/aic-dt/aic_dt/controller_loop.py                  # OK
grep -q "target_feedforward_torque" exts/aic-dt/aic_dt/controller_loop.py                 # OK
grep -q "target_stiffness" / "target_damping" exts/aic-dt/aic_dt/controller_loop.py       # OK
grep -q "self._last_reference_joint_state = msg.target_state" exts/aic-dt/aic_dt/controller_loop.py  # OK
grep -q "self._last_target_mode = TARGET_MODE_JOINT" exts/aic-dt/aic_dt/controller_loop.py  # OK
grep -c "def _on_joint_cmd" exts/aic-dt/aic_dt/controller_loop.py    # 1
grep -c "def _apply_joint_cmd" exts/aic-dt/aic_dt/controller_loop.py # 1
! grep -q "STUB — Plan 02-03" exts/aic-dt/aic_dt/controller_loop.py  # PASS (no stubs left)
```

End-to-end runtime verification (sub callback fires + apply_action lands on the articulation + UR5e joints actually move) is deferred to Plan 02-06's `smoke_test_aic_controller.py` — that's the canonical gate per the phase plan. This plan's verification is structural (imports, signatures, syntax, regex contract).

## Deviations from plan

### Auto-fixed issues

**1. [Rule 1 - Bug] TrajectoryGenerationMode enum values inverted in plan body**
- **Found during:** Task 1, while adding the constants block
- **Issue:** Plan body lines 154-156 declared `MODE_POSITION = 1` / `MODE_VELOCITY = 2`. Source-of-truth message at `~/Documents/aic/aic_interfaces/aic_control_interfaces/msg/TrajectoryGenerationMode.msg` lines 9-11 declares `MODE_VELOCITY = 1` / `MODE_POSITION = 2`. With the plan's values, the validator's `if mode == MODE_POSITION` branch would fire on actual velocity commands (and vice versa), routing them to the wrong size-check and either dropping all valid position commands or silently accepting invalid velocity commands.
- **Fix:** Used the message-file values verbatim. Added an inline comment in `controller_loop.py` documenting the source-of-truth and the inversion.
- **Files modified:** `exts/aic-dt/aic_dt/controller_loop.py`
- **Commit:** `4428b68`

### Refactors from plan body (no rule, just clarity)

**2. Per-flag size validation against `len(msg_joint_names)` instead of `N_ARM_JOINTS`**
- The plan's draft `has_stiffness = ... and len(msg.target_stiffness) == N_ARM_JOINTS + (1 if GRIPPER_NOOP in msg.target_state.joint_names else 0)` was correct in spirit but tightly coupled to the assumption that the message contains exactly the arm joints (optionally + gripper). The implemented version checks against `len(msg.target_state.joint_names)` directly — same outcome for the canonical case, but more robust if a message ever contains a different superset (e.g., a future tool-link joint). The loop's `if name not in ARM_JOINTS` skip still guards the actual application.
- **Files modified:** `exts/aic-dt/aic_dt/controller_loop.py`
- **Commit:** `4428b68`

## LOC delta

`controller_loop.py`: 393 → 565 LOC (+172 net). The plan's estimate of 80-120 LOC was undershot, primarily because:
- The validation body has 4 distinct size-check branches (position, velocity, stiffness, damping) rather than 2, each with a debug log message
- Both callbacks ship with full docstrings explaining the decision references (D-06/D-09/D-11) and pitfall references (Pitfall 6 set_gains pre-play)
- The constants block has source-of-truth comments documenting the enum-inversion fix

The actual Python logic is ~100 LOC; the rest is docstrings and inline source-tracing comments. Per autonomous M1 policy, the verbose docstrings stay — they're load-bearing for downstream plans (02-04 will mirror this exact validation pattern for MotionUpdate).

## Hand-off to Plan 02-04

> PARITY-09 done. The bookkeeping vars `_last_reference_joint_state` (JointTrajectoryPoint) and `_last_target_mode` (= `TARGET_MODE_JOINT`) are populated by the joint-cmd path on each successful `apply_action`. **Plan 02-04** populates the parallel pair `_last_reference_tcp_pose` (from MotionUpdate's Cartesian target) and updates `_last_target_mode` to `TARGET_MODE_CARTESIAN` (= 1) the same way. **Plan 02-05** reads both pairs and publishes whichever is non-stale into the ControllerState message.
>
> The constants `ARM_JOINTS` and `GRIPPER_NOOP` are intentionally module-level (not class attributes) so Plan 02-04's `_on_pose_cmd` / `_apply_pose_cmd` can reuse them when validating IK-output joint names against the same allowlist.
>
> The `MODE_VELOCITY=1 / MODE_POSITION=2` ordering is documented inline in `controller_loop.py` — Plan 02-04 should NOT redefine these and should NOT trust the plan-body value if it mentions them; always check the .msg file.

## Self-Check: PASSED

**Files exist:**
- `exts/aic-dt/aic_dt/controller_loop.py` — FOUND (565 LOC)
- `.planning/phases/02-controller-loop/02-03-SUMMARY.md` — FOUND (this file)

**Commits exist:**
- `4428b68` (feat 02-03 PARITY-09 implementation) — FOUND in `git log --oneline`
