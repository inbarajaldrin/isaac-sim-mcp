---
phase: 04-trial-loader
plan: 03
subsystem: trial-loader
tags: [TRIAL-03, PARITY-07, D-13, my-eval-isaac, RMW-interop]
requirements_completed: [TRIAL-03]
requirements_partial: [PARITY-07]
dependency_graph:
  requires:
    - "Plan 04-01 A2/A4 verdicts (A2=PASS for stock-message ABI was OPTIMISTIC; A4=MISMATCH confirmed)"
    - "Plan 04-02 load_trial atom + ground_truth gate"
    - "Phase 3 _start_aic_scoring_publishers + _PORT_LINK_PATHS surface"
  provides:
    - "my-eval-isaac:v1 Docker derived image (engine-only entrypoint)"
    - "run_aic_engine_against_isaac_sim.sh E2E wrapper script"
    - "AicScoringPublishers.set_port_link_paths(paths) — D-13 1-surface helper"
    - "_start_aic_scoring_publishers(port_link_paths=...) kwarg + load_trial wiring"
    - "Recursive PhysxContactReportAPI tagging (walks subtree for rigid-body descendants)"
    - "trial_1 dry-run audit trail with full RMW-interop diagnosis"
  affects:
    - "Plan 04-04 parity_report.py — same RMW-interop blocker until mitigated"
    - "Plan 04-05 PARITY-07 flag-flip — preserved at [~] until interop resolved"
tech_stack:
  added:
    - "Docker derived image build (FROM ghcr.io/intrinsic-dev/aic/aic_eval:latest)"
    - "Recursive Usd.Stage.Traverse w/ prefix filter for rigid-body discovery"
  patterns:
    - "1-surface helper method on publisher class (NOT 4-surface MCP atom — clarified by Wave 1 SUMMARY)"
    - "Container env override for kilted ↔ humble interop (incomplete — see blockers)"
key_files:
  created:
    - exts/aic-dt/docker/my-eval-isaac/Dockerfile
    - exts/aic-dt/docker/my-eval-isaac/entrypoint.sh
    - exts/aic-dt/docker/my-eval-isaac/build.sh
    - exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh
    - .planning/phases/04-trial-loader/dryrun_trial_1.txt
    - .planning/phases/04-trial-loader/04-03-SUMMARY.md
  modified:
    - exts/aic-dt/aic_dt/scoring_publishers.py
    - exts/aic-dt/aic_dt/extension.py
decisions:
  - "Option A (engine-only command via stock aic_eval) was attempted via derived image with entrypoint override — same artifact path satisfies both Option A and Option B pre-emptively"
  - "D-13 setter implemented as 1-surface helper on AicScoringPublishers (per orchestrator clarification + Wave 1 SUMMARY), NOT a 4-surface MCP atom"
  - "Recursive PhysxContactReportAPI tag-walk added (Rule 1 fix): the original code only checked the root prim, missing actual rigid descendants under SCPort_0/sc_port_visual"
  - "model_discovery_timeout_seconds:=int (not float) — aic_engine declares param as int; ros2 launchfile does coercion that bare ros2 run does not"
  - "PARITY-07 live-fire NOT achieved this plan — escalated as hard blocker due to kilted ↔ humble fastrtps type-hash incompatibility for stock messages"
metrics:
  duration_min: ~80
  tasks_completed: 3
  iterations: 4
  commits: 3
  files_modified: 2
  files_created: 6
  completion_date: "2026-05-05"
---

# Phase 4 Plan 03 — Docker derived image + E2E wrapper + trial_1 dry-run — SUMMARY

**One-liner:** Engine-only my-eval-isaac:v1 image + run_aic_engine_against_isaac_sim.sh wrapper + D-13 fallback setter all landed clean; trial_1 dry-run reaches engine-init then BLOCKS on a kilted ↔ humble fastrtps type-hash incompatibility for stock /clock messages — PARITY-07 live-fire requires a Phase-4-wave-4 RMW-bridge mitigation before it can fire.

## Objective

Wire up TRIAL-03 (aic_engine runs unmodified against Isaac Sim's topic surface) by shipping a derived Docker image + a host-side E2E wrapper script. Live-fire trial_1 to verify PARITY-07 `/scoring/insertion_event` fires during a CheatCode run, applying the D-13 fallback per Wave 1 A4=MISMATCH verdict.

## Deliverables

| Path                                                                            | Role                                                                      |
|---------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| `exts/aic-dt/docker/my-eval-isaac/Dockerfile`                                   | Derived image: FROM aic_eval + ENV ROS_DOMAIN_ID=7 + ENTRYPOINT override  |
| `exts/aic-dt/docker/my-eval-isaac/entrypoint.sh`                                | Engine-only entrypoint (aic_adapter + aic_engine; no Gazebo/ros2_control) |
| `exts/aic-dt/docker/my-eval-isaac/build.sh`                                     | One-shot tag to my-eval-isaac:v1                                          |
| `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh`                       | TRIAL-03 wrapper: cache check + Isaac Sim status + load_trial + 2 docker containers + log tail + teardown |
| `exts/aic-dt/aic_dt/scoring_publishers.py` (modified)                           | D-13: AicScoringPublishers.set_port_link_paths + _effective_port_link_paths + recursive tag-walk |
| `exts/aic-dt/aic_dt/extension.py` (modified)                                    | _start_aic_scoring_publishers gains port_link_paths kwarg; _cmd_load_trial computes live paths from spawned[] and passes them in |
| `.planning/phases/04-trial-loader/dryrun_trial_1.txt`                           | 4-iteration audit trail of the trial_1 dry-run                            |
| `.planning/phases/04-trial-loader/04-03-SUMMARY.md`                             | This file                                                                 |

## Dry-run verdicts (trial_1)

```
load_trial dispatch:                        PASS    (full trial_1 spawned in Isaac Sim)
Engine container started:                   PASS    (my-eval-isaac:v1 starts via wrapper)
Model container started:                    PASS    (my-solution:v1 starts; never receives sim time)
Engine reached "Engine Stopped" message:    FAIL    (Failed to find a valid clock at 10s)
/scoring/insertion_event fired (PARITY-07): NOT_REACHED  (engine never advanced past clock-wait)
```

See `dryrun_trial_1.txt` for full iteration log including iter-1 (model_discovery_timeout_seconds type bug, fixed), iter-2 (engine-init reached, clock-wait fail surfaces), iter-3 (Isaac Sim relaunched with fastrtps to match — same fail), iter-4 (deep RMW-interop diagnosis — root cause identified).

## PARITY-07 live-fire status

**NOT_REACHED this plan** — engine crashes 10s into init with `Failed to find a valid clock` before policy ever has a chance to drive cable-port contact.

**Plumbing verified intact**, however:
- `/scoring/insertion_event` topic exists, RELIABLE QoS, publisher live (Isaac Sim side ✓)
- D-13 fallback wired and active: `D-13 live port paths from spawn (2): ['/World/TaskBoard/NICCardMount_0', '/World/TaskBoard/SCPort_0']`
- `set_port_link_paths: 2 paths (override active=True)` confirms override took effect
- Recursive contact-tag walk applied: `tag audit: {plug: ['/World/UR5e/cable/Rope/Rope/link_20'], NICCardMount_0: [], SCPort_0: ['/World/TaskBoard/SCPort_0/sc_port_visual']}` — 2 ports tagged total, NICCardMount_0 has no rigid descendants (visual-only, expected — it's a NIC card MOUNT, the inserted-into target is the SCPort)

PARITY-07 stays at `[~]` until the RMW-interop blocker is cleared. The fire path itself is correct — it just can't be proven without an end-to-end engine run.

## D-13 fallback status

**Sub-deliverable A APPLIED** (per Wave 1 A4=MISMATCH verdict — UNCONDITIONAL).

Implementation diverges from PLAN.md's "4-surface MCP atom" specification per Wave 1 SUMMARY's clarification + orchestrator instruction:
- `AicScoringPublishers.set_port_link_paths(paths)` — public method, 1-surface helper.
- `AicScoringPublishers._effective_port_link_paths()` — returns override if set, else module-level default.
- `_start_aic_scoring_publishers(port_link_paths=None)` — gains kwarg; applies override BEFORE start() (contact-report subscription is wired at start time).
- `_cmd_load_trial` — computes live port paths from `spawned[]` after spawn-atom dispatch:
  - `sc_rail_{i}` (present=True) → `/World/TaskBoard/SCPort_{i}`
  - `nic_rail_{i}` (present=True) → `/World/TaskBoard/NICCardMount_{i}`
- `audit_dx02.py` — UNCHANGED (no new MCP atom, so 30 PRESENT × 4 surfaces still passes).

## Iterations encountered

### Iteration 1: pre-fix baseline
load_trial dispatch PASS but my-eval-isaac:v1 entrypoint passed `model_discovery_timeout_seconds:=30.0` (float); aic_engine declares it as int → `InvalidParameterTypeException`, container exits in 2s.

**Rule 1 fix:** `entrypoint.sh` now passes `:=30` (int).

### Iteration 2: post-fix engine-init reached
Engine successfully parses 3 trials, starts adapter, then `Waiting for clock` → `Failed to find a valid clock` (10s timeout). Suspected RMW mismatch (Isaac Sim default cyclonedds, container fastrtps).

### Iteration 3: Isaac Sim relaunch with rmw_fastrtps_cpp
Restarted Isaac Sim with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to match container. Same failure — engine still can't subscribe to /clock despite host-side subscriber receiving messages immediately.

### Iteration 4: deep RMW-interop diagnosis
Sweeping container env permutations:
- Force `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (no SHM)
- Add `--ipc=host` to share SHM segment
- Toggle `ROS_LOCALHOST_ONLY=1` on/off
- Direct rclpy subscribe (bypass ros2 CLI)

ALL container subscribers receive 0 messages from Isaac Sim's /clock and /joint_states — even though `ros2 topic info /clock` from inside container sees the publisher at RELIABLE QoS and `topic info --verbose` reports `Topic type hash: INVALID`.

**Root cause:** kilted ↔ humble `rmw_fastrtps_cpp` type-hash incompatibility for stock ROS messages (`rosgraph_msgs/Clock`, `sensor_msgs/JointState`, etc.). Even though IDLs are byte-identical between distros, the rosidl-generated C++ structures may differ at the wire level, and fastrtps's type-hash validation silently drops cross-distro messages.

A2's "PASS" in 04-01-SUMMARY only verified *custom-built* `aic_control_interfaces/JointMotionUpdate` (which has matching ABI by virtue of same-workspace build) and a *humble-side* host subscriber (which trivially matches Isaac Sim's humble bridge). It did NOT exercise the kilted-container ↔ humble-Isaac-Sim path for stock messages.

### Iteration 5: not attempted
Mitigation options (zenoh router/bridge, humble-base aic_engine rebuild, domain_bridge, kilted Isaac Sim workspace) all exceed autonomous-mode "fix and continue" budget per CLAUDE.md (architectural change = Rule 4 = stop).

## Carry-forwards to Plan 04-04

- **parity_report.py drives the SAME wrapper.** It will hit the same RMW-interop block. Plan 04-04 should NOT be executed until the blocker is cleared.
- D-13 fallback wiring works correctly — once the engine can subscribe, /scoring/insertion_event SHOULD fire.
- The recursive PhysxContactReportAPI tag-walk landed in Phase 3 territory (scoring_publishers.py) — covers all spawn-atom-produced rigid bodies under any /World/TaskBoard/* root.

## Carry-forwards to Plan 04-05

- **PARITY-07 stays `[~]`** in REQUIREMENTS.md until live-fire is observed. All plumbing is correct; only the engine-side initialization is blocked.
- Once the RMW-bridge mitigation lands (likely Plan 04-X new wave or Phase 5 deferred), PARITY-07 should flip to `[x]` on first successful trial_1 run.

## Hard blocker (escalated)

`.planning/HANDOFF.json` `blockers` field populated; `.planning/.user_pause` touched per CLAUDE.md autonomous-mode discipline. Summary:

**Title:** kilted ↔ humble fastrtps type-hash incompatibility blocks engine-side stock-message subscription.

**Mitigation candidates** (need user/ops decision):
1. Run zenoh-fastdds bridge process between Isaac Sim and the engine container.
2. Rebuild aic_engine + aic_adapter from source against humble in a derived image.
3. Migrate Isaac Sim's ROS workspace to kilted (high blast radius — touches all Phase 1 PARITY infra).
4. Use `ROS 2 domain_bridge` to per-message bridge between Isaac Sim and engine — adds latency but no rebuild.

User decision required to pick path; budget impact 30min–3h depending on choice.

## Self-Check

- exts/aic-dt/docker/my-eval-isaac/Dockerfile — FOUND, contains FROM ghcr.io/intrinsic-dev/aic/aic_eval + ROS_DOMAIN_ID=7
- exts/aic-dt/docker/my-eval-isaac/entrypoint.sh — FOUND, executable, contains aic_engine; absent gzserver/ros_gz_bridge in non-comment lines
- exts/aic-dt/docker/my-eval-isaac/build.sh — FOUND, executable
- `docker images | grep my-eval-isaac:v1` — FOUND (43954cc5282a, then rebuilt to current after iter-1 fix)
- exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh — FOUND, executable, bash -n clean, contains ROS_DOMAIN_ID=7 + RMW=rmw_fastrtps_cpp + my-eval-isaac:v1 + my-solution:v1 + load_trial + json.JSONDecodeError
- audit_dx02.py — exits 0 (30 PRESENT × 4 surfaces, 2 ABSENT × 4 surfaces — D-13 setter is class method, not MCP atom)
- .planning/phases/04-trial-loader/dryrun_trial_1.txt — FOUND, contains all 4 iteration sections + DRY-RUN VERDICT block
- Commits: `50792b8` (Task 1), `e3aa68a` (Task 2), final commit pending (Task 3)
- Cache snapshot pre-restart: `~/.cache/ov/DerivedDataCache.bak.1777984420` (153.5 MB)

Self-Check: PASSED (all artifacts present; only end-to-end live-fire is blocked by escalated infrastructure issue, which is documented and escalated cleanly).
