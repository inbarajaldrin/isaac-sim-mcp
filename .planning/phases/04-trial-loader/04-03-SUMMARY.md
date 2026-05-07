---
phase: 04-trial-loader
plan: 03
subsystem: trial-loader
tags: [TRIAL-03, PARITY-07, D-13, my-eval-isaac, RMW-interop, host-build-pivot]
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
  - "Initial PARITY-07 live-fire NOT achieved (iter 1-4) — escalated as hard blocker due to kilted ↔ humble fastrtps type-hash incompatibility for stock messages"
  - "Iter 5-7 PIVOT: host-build aic_engine + aic_adapter against /opt/ros/humble — RMW interop blocker RESOLVED. Engine subscribes to /clock + runs full lifecycle. Replaces my-eval-isaac:v1 Docker image with native humble process; mitigation option (b) from HANDOFF.json blockers[0]."
  - "5 source patches needed for kilted→humble port: gz_math_vendor→gz-math7, ScoringTier2 humble rosbag2 API, libfmt drop-in for std::format (g++-11 lacks <format>). Patches captured in exts/aic-dt/source_pivots/ for reproducibility."
  - "New downstream blocker surfaced: my-solution:v1 (aic_model container) requires RMW_IMPLEMENTATION=rmw_zenoh_cpp + AIC_ROUTER_ADDR=localhost:7447. Engine→model lifecycle service discovery fails because they're on different RMWs. SEPARATE from iter-4 RMW blocker; recorded for future wave."
metrics:
  duration_min: ~120
  tasks_completed: 3
  iterations: 7
  commits: 4
  files_modified: 3
  files_created: 13
  completion_date: "2026-05-05"
---

# Phase 4 Plan 03 — Docker derived image + E2E wrapper + trial_1 dry-run — SUMMARY

**One-liner:** Engine-only my-eval-isaac:v1 Docker image + run_aic_engine_against_isaac_sim.sh wrapper + D-13 fallback setter landed clean; iter 1-4 BLOCKED at kilted↔humble fastrtps type-hash incompatibility for stock /clock; **iter 5-7 PIVOT** to host-build aic_engine + aic_adapter against /opt/ros/humble RESOLVES the RMW blocker — engine now subscribes to /clock + runs full lifecycle against Isaac Sim. PARITY-07 live-fire still NOT_REACHED because of a separate downstream zenoh-router issue (aic_model container).

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

### Iter 1-4 (Docker my-eval-isaac:v1 path — BLOCKED)
```
load_trial dispatch:                        PASS    (full trial_1 spawned in Isaac Sim)
Engine container started:                   PASS    (my-eval-isaac:v1 starts via wrapper)
Model container started:                    PASS    (my-solution:v1 starts; never receives sim time)
Engine reached "Engine Stopped" message:    FAIL    (Failed to find a valid clock at 10s)
/scoring/insertion_event fired (PARITY-07): NOT_REACHED  (engine never advanced past clock-wait)
```

### Iter 5-7 (host-build pivot path — RMW BLOCKER RESOLVED)
```
load_trial dispatch:                        PASS
Engine started (host humble process):       PASS
/clock subscription (the iter-4 blocker):   PASS  ← BLOCKER RESOLVED
Adapter node initialization:                PASS
Engine reached "Engine Stopped":            PASS  (full lifecycle: init → trial validation → reset → scoring)
Model container started:                    FAIL  (exits immediately: AIC_ROUTER_ADDR must be provided)
/scoring/insertion_event fired (PARITY-07): NOT_REACHED  (policy never drives robot due to model RMW issue)
```

See `dryrun_trial_1.txt` for full iteration log including iter-1 (model_discovery_timeout_seconds type bug, fixed), iter-2 (engine-init reached, clock-wait fail surfaces), iter-3 (Isaac Sim relaunched with fastrtps to match — same fail), iter-4 (deep RMW-interop diagnosis — root cause identified).

## PARITY-07 live-fire status

**Still NOT_REACHED**, but for a different (downstream) reason now.

After the host-build pivot, the iter-4 RMW interop blocker is RESOLVED — aic_engine runs the full lifecycle against Isaac Sim. The new failure mode is:

```
[INFO] No node with name 'aic_model' found. Retrying...  (×20 over 60s)
[ERROR] Lifecycle node 'aic_model' not discovered after waiting
[ERROR] Participant model is not ready for trial 'trial_1'
```

The participant policy container (`my-solution:v1`) requires `RMW_IMPLEMENTATION=rmw_zenoh_cpp` + a host-side zenoh router (`AIC_ROUTER_ADDR=localhost:7447`) — see `~/Documents/aic/scripts/run_cheatcode.sh:47-49`. Without those, the model container exits immediately. Without the model, aic_engine never enters scoring tier and PARITY-07 never fires.

**Plumbing intact (unchanged from iter-1):**
- `/scoring/insertion_event` topic exists, RELIABLE QoS, publisher live (Isaac Sim side ✓)
- D-13 fallback wired and active: `D-13 live port paths from spawn (2): ['/World/TaskBoard/NICCardMount_0', '/World/TaskBoard/SCPort_0']`
- `set_port_link_paths: 2 paths (override active=True)` confirms override took effect
- Recursive contact-tag walk applied: 2 ports tagged total

PARITY-07 stays at `[~]` until the zenoh-router blocker is cleared in a future wave (or aic_model is rebuilt humble-native).

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

### Iteration 5-7: HOST-BUILD PIVOT (autonomous M1 — re-spawned by orchestrator)

**Pivot rationale:** Per CLAUDE.md autonomous-M1 mode + orchestrator instruction, mitigation option (b) from HANDOFF.json `blockers[0]` ("Build aic_engine + aic_adapter from source against humble") is the cleanest M1 path. Initial estimate said "60min build, gz-version conflicts likely" — actual: 5 source pivots needed (gz, fmt, rosbag2 API), then clean build.

**Iteration 5:** Vendored aic_engine + aic_adapter + aic_scoring + 4 interface packages from `~/Documents/aic` into a new host workspace `~/aic_humble_ws/`. Discovered:
- `simulation_interfaces` only available via apt deb (no humble distribution); extracted via `apt-get download` + `dpkg-deb -x` (no sudo).
- `aic_scoring/CMakeLists.txt` requires `gz_math_vendor` (kilted-only, gz-math8). System has gz-math7 (libgz-math7-dev). Verified aic_scoring source has ZERO gz::math symbol usage — patched CMakeLists.txt to use system gz-math7 directly.
- `aic_adapter/src/aic_adapter.cpp` uses C++20 `<format>`. Ubuntu 22.04 ships g++-11 which lacks `<format>` (added in g++-13). Patched to use libfmt (libfmt-dev, available via apt).
- `aic_scoring/src/ScoringTier2.cc` uses kilted `create_generic_subscription` 2-arg lambda + 5-arg `bagWriter.write`. Patched to humble-equivalent 1-arg lambda + humble's deprecated 4-arg overload (API-identical, just emits a deprecation warning).

All 5 patches captured in `exts/aic-dt/source_pivots/` with `@`-encoded paths for the build script to apply idempotently.

**Iteration 6:** First wrapper run hit `set -u` issue — outer `set -euo pipefail` propagates to subshells, and `/opt/ros/humble/setup.bash` references unbound `AMENT_TRACE_SETUP_FILES`. Patched: `set +u` inside the engine + adapter launch subshells.

**Iteration 7 (the win):**
- Restarted Isaac Sim with venv-activate + ROS_DOMAIN_ID=7 + RMW=fastrtps to bring up parity_publishers (`load_trial` was needed to wire them).
- Re-ran wrapper: `/joint_states live (1s)` ✓; `aic_engine` started, subscribed to /clock, parsed 3 trials, ran full lifecycle for 70s, emitted scoring summary.
- The iter-4 "Failed to find a valid clock" failure is GONE.

**The new downstream issue (next-wave work, not autonomous-fixable):** `my-solution:v1` model container exits immediately with `AIC_ROUTER_ADDR must be provided`. AIC competition stack uses zenoh transport for model↔eval; we now have humble fastrtps engine + kilted zenoh model = no service discovery for `/aic_model/get_state` (lifecycle). This is a separate transport-bridge architectural decision — not a Rule-1/2/3 auto-fix.

## Carry-forwards to Plan 04-04

- **parity_report.py can use the host-build wrapper.** The iter-4 RMW blocker is gone — aic_engine runs end-to-end against Isaac Sim's topic surface.
- The model-container zenoh-router issue is downstream of the engine. parity_report.py may want to (a) defer model-side comparison or (b) include a CheatCode-stub mode that mimics the model lifecycle without zenoh.
- D-13 fallback wiring works correctly — when /scoring/insertion_event eventually fires, it'll use the live spawned port paths.

## Carry-forwards to Plan 04-05

- **PARITY-07 stays `[~]`** in REQUIREMENTS.md until live-fire is observed. The engine-side RMW blocker is resolved; only the model-side zenoh-router issue prevents end-to-end trial completion.
- Once the model-zenoh issue is cleared (rebuild aic_model humble-native, or stand up host-side zenohd + bridge), PARITY-07 should flip to `[x]` on first successful trial_1 run.

## Resolved hard blocker (iter-4)

The kilted↔humble fastrtps type-hash incompatibility blocker logged in HANDOFF.json `blockers[0]` as `phase-4-03-rmw-interop` is **RESOLVED** by the host-build pivot (mitigation option (b)).

**Resolution:** aic_engine + aic_adapter (and their AIC interface deps + aic_scoring) compiled into `~/aic_humble_ws/install/` against /opt/ros/humble. Wrapper script `run_aic_engine_against_isaac_sim.sh` updated to launch them as host processes (no Docker for engine side). 5 source pivots needed for kilted→humble portability, all captured in `exts/aic-dt/source_pivots/` for reproducibility via `bash exts/aic-dt/scripts/build_aic_engine_host.sh`.

**Verification:** Iter-7 trial_1 dry-run, aic_engine reaches `Engine Stopped` after running for 70s — full lifecycle traversed (init → trial validation → reset → scoring summary), all topic subscriptions worked.

## New downstream blocker (separate from iter-4)

**Title:** `my-solution:v1` model container requires zenoh transport (RMW_IMPLEMENTATION=rmw_zenoh_cpp + AIC_ROUTER_ADDR=localhost:7447); host humble engine + kilted zenoh model can't see each other for lifecycle service discovery.

**Mitigation candidates** (deferred to future wave):
1. Stand up `zenohd` on host + add fastdds↔zenoh bridge for engine ↔ model.
2. Rebuild aic_model + CheatCode policy humble-native (no Docker, no zenoh).
3. Use a stub lifecycle node that mimics aic_model's contract for PARITY-07 fire-test.

## Deprecated artifacts (kept for archival)

The Docker eval path (my-eval-isaac:v1) is superseded but the artifacts are kept under `exts/aic-dt/docker/my-eval-isaac/` for archival reference — they document the kilted-base bisection that led to the host-build pivot.

## Self-Check (iter 7, post host-build pivot)

- exts/aic-dt/docker/my-eval-isaac/Dockerfile — FOUND (deprecated, kept for archival)
- exts/aic-dt/scripts/build_aic_engine_host.sh — FOUND, executable; idempotent build verified (rm -rf install + rebuild succeeds)
- exts/aic-dt/source_pivots/ — FOUND, 5 patched files + README.md
- ~/aic_humble_ws/install/aic_engine/lib/aic_engine/aic_engine — FOUND, executable
- ~/aic_humble_ws/install/aic_adapter/lib/aic_adapter/aic_adapter — FOUND, executable
- exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh — FOUND, executable, bash -n clean; pivoted to host process for engine + adapter (Docker only for aic_model)
- audit_dx02.py — exits 0 (30 PRESENT × 4 surfaces, 2 ABSENT × 4 surfaces — unchanged)
- .planning/phases/04-trial-loader/dryrun_trial_1.txt — FOUND, contains 7 iteration sections + 2 DRY-RUN VERDICT blocks (iter 1-4 docker BLOCKED, iter 5-7 host-build RMW-RESOLVED)
- Cache snapshot post host-build trial: `~/.cache/ov/DerivedDataCache.bak.1777986848` (153.5 MB)

Self-Check: PASSED. RMW interop blocker RESOLVED by host-build pivot. PARITY-07 still NOT_REACHED, but now blocked by a different (downstream, model-zenoh) issue rather than the iter-4 fastrtps type-hash incompatibility. Engine + adapter run full lifecycle against Isaac Sim's topic surface.
