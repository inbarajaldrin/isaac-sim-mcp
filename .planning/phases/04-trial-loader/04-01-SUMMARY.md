---
phase: 04-trial-loader
plan: 01
subsystem: pre-flight-risk-de-risking
tags: [phase-4, wave-1, probe, rmw-interop, prim-paths, A2, A4]
provides:
  - "A2 verdict (kilted ↔ humble RMW interop) on disk, audit-trail .txt"
  - "A4 verdict (_PORT_LINK_PATHS vs live spawn paths) on disk, audit-trail .txt"
  - "Two re-runnable probe scripts under exts/aic-dt/scripts/"
  - "Concrete go/no-go signal + remediation guidance for Plan 04-03"
requires:
  - "Isaac Sim aic-dt running on localhost:8768 (verified RUNNING + RESPONSIVE at probe time)"
  - "DerivedDataCache healthy (153.5 MB; backup snapshotted post-probe)"
  - "env_isaaclab humble venv available with rclpy + aic_control_interfaces"
affects: []
key_files:
  created:
    - exts/aic-dt/scripts/verify_kilted_humble_interop.sh
    - exts/aic-dt/scripts/probe_taskboard_prim_paths.py
    - .planning/phases/04-trial-loader/kilted_humble_interop.txt
    - .planning/phases/04-trial-loader/taskboard_prim_paths.txt
    - .planning/phases/04-trial-loader/04-01-SUMMARY.md
  modified: []
decisions:
  - "Bypassed env_isaaclab's broken /install/bin/ros2 CLI (Python 3.10 shebang vs 3.11 venv + missing netifaces) by writing inline rclpy harnesses — same wire path as ros2 topic echo, just bypassing the CLI entry-point bug"
  - "Confirmed at probe time that Isaac Sim was already RUNNING + RESPONSIVE; reused it instead of relaunching (per CLAUDE.md autonomous-mode 'reuse healthy Isaac Sim' discipline)"
  - "Cache snapshot taken post-probe (DerivedDataCache.bak.1777983174) per CLAUDE.md mandatory snapshot-after-quick_start discipline"
metrics:
  tasks_completed: 3
  artifacts_created: 5
  duration_min: ~5
  completed_date: "2026-05-05"
---

# Phase 4 Plan 01 — Pre-flight risk de-risking — SUMMARY

**One-liner:** Two HIGH-impact assumptions (A2 kilted↔humble RMW interop, A4 prim-path mismatch) verified before Plan 04-03 commits to architectural choices — A2 PASSED unblocks the engine container path, A4 MISMATCHED forcing D-13 fallback into Plan 04-03 baseline scope.

## Objective

Pre-flight risk de-risking for Phase 4 — verify the two HIGH-impact assumptions flagged in 04-RESEARCH.md (A2 RMW interop, A4 _PORT_LINK_PATHS prim-path coverage) BEFORE writing the trial loader. Burn 30 minutes on probes now to avoid 4 hours of trial-loader debug after the fact.

## Verdicts captured

### A2 (kilted ↔ humble RMW interop)

**Verdict:** `=== A2 VERDICT: PASS ===`

**Evidence (from `kilted_humble_interop.txt`):**
- Container started with overrides: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=7`. Image pinned at the Phase 1 D-14 digest `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433`.
- Host humble subscriber on `/joint_states` decoded a real message:
  - `header.stamp=1777983025.311887543`
  - `name=(7) ['elbow_joint', 'gripper/left_finger_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']`
  - `position=(7) [0.0017, 0.0, -0.0842, -0.004, -0.0003, 0.0005, -0.0011]`
- Custom-msg ABI test passed: `aic_control_interfaces.msg.JointMotionUpdate` imported cleanly AND `node.create_subscription(JointMotionUpdate, "/aic_controller/joint_commands", ...)` returned without `rosidl_typesupport_c` failure — the kilted-built ABI loads against humble.

**Implication for Plan 04-03:** The `run_aic_engine_against_isaac_sim.sh` wrapper (D-06) can configure the engine container with `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=7` and use the stock `aic_eval` image. **No need for the derived `my-eval-isaac:v1` humble-base path** (which would have added ~30 min of build time per A6's escalation note). Phase 4 plan body can stay as-written.

### A4 (_PORT_LINK_PATHS vs live spawn paths)

**Verdict:** `=== A4 VERDICT: MISMATCH_NIC_CARD_MOUNT ===`

**Evidence (from `taskboard_prim_paths.txt`):**

The hardcoded `_PORT_LINK_PATHS` in `scoring_publishers.py:62-66` was authored against an early test scene; live spawn-atom output uses different namespace + casing. Audit table:

| hardcoded path                  | match? | live equivalents (from probe)                                                |
|---------------------------------|--------|------------------------------------------------------------------------------|
| `/World/TaskBoard/sc_port_1`    | MISMATCH | `/World/TaskBoard/SCPort_0`, `/World/Objects/sc_port_1`                    |
| `/World/TaskBoard/sc_port_2`    | MISMATCH | `/World/TaskBoard/SCPort_1`, `/World/Objects/sc_port_2`                    |
| `/World/TaskBoard/nic_card`     | MISMATCH | `/World/TaskBoard/NICCard`, `/World/TaskBoard/NICCardMount_0..4`, `/World/Objects/nic_card` |

The probe sent `quick_start` then `spawn_sc_port(0)`, `spawn_sc_port(1)`, `spawn_nic_card_mount(0..4)`, `spawn_nic_card`, then `stage.Traverse` filtered to `/World/TaskBoard/*` + `/World/Objects/*`. Both namespaces co-exist because `add_objects` (legacy) writes to `/World/Objects/{obj_name}` and the new spawn atoms write to `/World/TaskBoard/{CamelCase}_{index}`. **Neither namespace matches the hardcoded list.**

The downstream consequence (Phase 3 `scoring_publishers._init_insertion_event_subscription`): the contact-report subscription tags zero of the hardcoded prims (because they don't exist), `/scoring/insertion_event` never fires, every TRIAL-04 row mismatches Gazebo's pass result. Would have surfaced as a multi-hour debug loop in Plan 04-04 / 04-05.

**Implication for Plan 04-03:** D-13 fallback path is now confirmed required, NOT a YAGNI "maybe". Plan 04-03's `load_trial` MUST:

1. Add a public setter `set_port_link_paths(paths: list[str])` on `AicScoringPublishers` (not a new MCP atom — just a public method on the publisher class).
2. After spawn-atom dispatch, compute the live port paths from spawn call sites:
   - `spawn_sc_port(index=i, present=True)` → `/World/TaskBoard/SCPort_{i}`
   - `spawn_nic_card_mount(index=i, present=True)` → `/World/TaskBoard/NICCardMount_{i}`
   - `spawn_nic_card(present=True)` → `/World/TaskBoard/NICCard`
3. Pass the computed list to `_start_aic_scoring_publishers(port_link_paths=...)` so the contact subscription wires to actual prims.

This is a **1-surface addition** (public method on the publisher class, called from `_start_aic_scoring_publishers`), NOT a 4-surface DX-02 atom add. Plan 04-03 should fold this into its task graph as a forced dependency on the trial loader (Sub-A path active).

## Deliverables

| Path                                                                                | Role                                                       |
|-------------------------------------------------------------------------------------|------------------------------------------------------------|
| `exts/aic-dt/scripts/verify_kilted_humble_interop.sh`                              | Re-runnable A2 RMW interop smoke (3 min wall-clock)        |
| `exts/aic-dt/scripts/probe_taskboard_prim_paths.py`                                | Re-runnable A4 prim-path probe (~30s on a warm Isaac Sim)  |
| `.planning/phases/04-trial-loader/kilted_humble_interop.txt`                       | A2 audit trail with full topic decode evidence             |
| `.planning/phases/04-trial-loader/taskboard_prim_paths.txt`                        | A4 audit trail with full prim list + mismatch table        |
| `.planning/phases/04-trial-loader/04-01-SUMMARY.md`                                | This file                                                   |

## Carry-forwards to Plan 04-02 / 04-03

### A2 PASS → Plan 04-03 baseline path active

- Wrapper script `run_aic_engine_against_isaac_sim.sh` (D-06) sets `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=7` on the engine container env. Use stock `aic_eval:latest` image (digest pinned at `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433`).
- No derived `my-eval-isaac:v1` humble-base build needed. A6's "Option 4B" remains the structural choice (Docker for the engine because of host GLIBC), but the base image stays the official `aic_eval` (no rebuild).

### A4 MISMATCH_NIC_CARD_MOUNT → Plan 04-03 must include set_port_link_paths setter

- **Add to scoring_publishers.py:** `def set_port_link_paths(self, paths: list[str])` — replaces the module-level `_PORT_LINK_PATHS` constant for this publisher instance. The constant stays as the default for cable-only test scenes.
- **Modify `_start_aic_scoring_publishers`:** accept `port_link_paths` kwarg, default to module-level constant if not passed.
- **In `_cmd_load_trial` (D-01/D-02 atom):** compute paths from spawn calls, pass into `_start_aic_scoring_publishers(port_link_paths=computed)`.
- **Test in Plan 04-04 dry-run:** `parity_report.py --trial trial_1 --sim isaac` — confirm `/scoring/insertion_event` fires when CheatCode plug-into-port completes.

### Cross-phase note on /World/Objects vs /World/TaskBoard duality

Plan 04-03's `load_trial` should consider whether to suppress the legacy `add_objects` /World/Objects spawn (which would simplify the prim tree), or leave it on for backwards-compat with `randomize_object_poses` etc. (Phase 1 atoms still target /World/Objects). Recommend: leave on, since /World/TaskBoard is the canonical scoring target now and the duplication is harmless (different-named prims, no physics conflict). Document this in 04-03-SUMMARY.

## Issues Encountered

1. **env_isaaclab `/install/bin/ros2` CLI is broken.** Shebang is `#!/usr/bin/python3` (Python 3.10 system) but the rclpy/ros2cli packages are built for Python 3.11; on top of that `netifaces` is missing in the venv. Workaround: inline rclpy harness inside the bash script (also what `ros2 topic echo` does internally — no functional gap). Logged as a future-maintenance ticket; doesn't block Phase 4.

2. **`env_isaaclab` venv defaults to `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**, not fastrtps. The plan body's A2 test explicitly tests fastrtps to match the engine container side; we honor that. Future Plan 04-03 wrapper must keep the `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp` override + match it on the host subscriber side.

3. **Container-side `ros2 topic list` returned empty** during Step 2's sanity check (the daemon-stop race against fastrtps discovery on first connection). Non-blocking — Step 3's host decode succeeded, which is the actual test. Logged in the report as-is; would need a longer warmup if it ever blocked the verdict.

## TDD Gate Compliance

This plan is `type=execute`, not `type=tdd` — no RED/GREEN/REFACTOR cycle expected. The probe scripts ARE the tests for downstream Plan 04-03 implementation; they verify before-the-fact assumptions rather than after-the-fact behavior. Acceptable per the project's "verify by running" pattern (Phase 3 SUMMARY).

## Self-Check: PASSED

- `exts/aic-dt/scripts/verify_kilted_humble_interop.sh` — FOUND, executable, passes `bash -n`, contains `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (6 occurrences) + `ROS_DOMAIN_ID=7` (7 occurrences) + `A2 VERDICT` (2 occurrences).
- `exts/aic-dt/scripts/probe_taskboard_prim_paths.py` — FOUND, executable, parses via ast.parse, stdlib-only imports, contains `_PORT_LINK_PATHS` (6 occurrences) + `A4 VERDICT` (5 occurrences).
- `.planning/phases/04-trial-loader/kilted_humble_interop.txt` — FOUND, ends with `=== A2 VERDICT: PASS ===`, contains `JointMotionUpdate` (2 occurrences).
- `.planning/phases/04-trial-loader/taskboard_prim_paths.txt` — FOUND, ends with `=== A4 VERDICT: MISMATCH_NIC_CARD_MOUNT ===`, contains both required `## Live /World/TaskBoard/* prim paths after spawn cycle` and `## _PORT_LINK_PATHS audit` sections + `set_port_link_paths` recommendation.
- Commit `ff6ca28` (Task 1) verified via `git log --oneline -5`; commit `400ac01` (Task 2) verified.
- No `Co-Authored-By` lines in any commit (per CLAUDE.md global rule).

## Next

**Plan 04-02:** `load_trial` MCP atom + `ground_truth` flag + YAML adapter dict (D-01..D-05). Inherits A2 PASS as a free unblock.

**Plan 04-03:** Engine wrapper script + Sub-A `set_port_link_paths` setter (now required, not optional). Inherits A2 PASS for env config and A4 MISMATCH for the setter scope.
