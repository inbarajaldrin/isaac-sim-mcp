---
phase: 02-controller-loop
plan: 01
subsystem: infra
tags: [phase-2, infrastructure, rclpy, workspace-build, custom-messages, off-limit-discovery, d-05-fix, open-q1, ros2-humble, python-3.11, abi]

# Dependency graph
requires:
  - phase: 01-foundation-parity
    provides: IsaacSim-ros_workspaces Python 3.11 workspace build infra (rclpy + base humble pkgs); env_isaaclab/bin/activate sources both inner humble_ws/install AND outer isaac_sim_ros_ws/install local_setup.bash
provides:
  - aic_control_interfaces Python 3.11 ABI build importable from Isaac Sim's bundled Python
  - ros_gz_interfaces Python 3.11 ABI build importable from Isaac Sim's bundled Python
  - Idempotent vendor+build wrapper script (build_aic_msgs.sh)
  - Idempotent topic-snapshot wrapper script (snapshot_aic_eval_offlimit.sh) for re-running discovery
  - Open Q1 settled — all four CONTEXT.md-assumed Phase 2 controller topic names confirmed correct
  - DEFAULT_OFF_LIMIT_PRIMS = 3 USD prefix entries (/World/Enclosure, /World/Enclosure_Walls, /World/TaskBoard) derived from authoritative URDF source
affects: [02-02 controller-loop skeleton, 02-03 joint_commands subscriber, 02-04 pose_commands subscriber, 02-05 controller_state publisher, 02-06 off_limit Contacts publisher]

# Tech tracking
tech-stack:
  added:
    - aic_control_interfaces (vendored from ~/Documents/aic, built for cp311)
    - ros_gz_interfaces (vendored from gazebosim/ros_gz humble branch, built for cp311)
  patterns:
    - "Two-workspace build_ros.sh discipline — vendor to humble_ws/src (Dockerfile COPY context), artifacts land in isaac_sim_ros_ws/install per-package (NOT in inner humble_ws/install)"
    - "ABI-tag rejection check in build wrapper — fail loud if cpython-310 .so files appear (would mean wrong-Python build)"
    - "Round-trip import verification in build wrapper — bash -c 'source venv && python -c \"from <pkg>.msg import ...\"' as the gate"
    - "Snapshot script idempotency via trap cleanup EXIT — never leave stale containers"
    - "Authoritative-source-fallback for off-limit prims when live capture is empty (CheatCode = passing policy = zero off_limit events)"

key-files:
  created:
    - exts/aic-dt/docs/aic-msgs-setup.md
    - exts/aic-dt/scripts/build_aic_msgs.sh
    - exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh
    - exts/aic-dt/docs/offlimit-prim-mapping.md
    - .planning/phases/02-controller-loop/snapshot/all_topics.txt
    - .planning/phases/02-controller-loop/snapshot/aic_controller_topic_list.txt
    - .planning/phases/02-controller-loop/snapshot/aic_controller_topic_info.txt
    - .planning/phases/02-controller-loop/snapshot/aic_eval_offlimit_capture.txt
  modified: []

key-decisions:
  - "D-05 ABI fix landed: vendor aic_control_interfaces + ros_gz_interfaces into IsaacSim-ros_workspaces/humble_ws/src, rebuild via build_ros.sh — replaces broken PYTHONPATH-link to AIC pixi env (Python 3.12 vs 3.11 ABI mismatch)"
  - "Vendor target is ~/IsaacSim-ros_workspaces/humble_ws/src/ (Dockerfile COPY context), NOT ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/ (post-build extracted dir) — initial confusion was a real landmine, doc'd in aic-msgs-setup.md"
  - "Built artifacts install per-package under isaac_sim_ros_ws/install/<pkg>/local/lib/python3.11/dist-packages/<pkg>/, NOT into the merge-install humble_ws/install — env_isaaclab/bin/activate already sources isaac_sim_ros_ws's local_setup.bash so auto-discovery handles this"
  - "Open Q1 verdict: ALL FOUR Plan 02 critical topic names confirmed correct (joint_commands/pose_commands/controller_state/off_limit) — no CONTEXT.md edits needed; Plans 02-03..06 wire to assumed names verbatim"
  - "Off-limit prim set is PREFIX-based (3 entries: /World/Enclosure, /World/Enclosure_Walls, /World/TaskBoard) derived from URDF authoritative source — NOT a hand-enumerated list of individual link names; Plan 02-06 contact callback uses startswith() filtering"
  - "Off-limit live capture was empty (expected — CheatCode is passing policy); URDF-derived authoritative source is the ground truth, with WallPresser/WallToucher noted as the policy to swap to for non-empty capture if needed"
  - "Discovery surfaced 3 additional aic_controller topics not in CONTEXT.md (joint_motion_update, motion_update, transition_event) — out of scope for Plan 02, doc'd for future reference"

patterns-established:
  - "Two-workspace build pattern: inner humble_ws (--merge-install, base humble) + outer build_ws (per-package install, our custom packages) — Phase 2+ extensions to the workspace ALWAYS go to humble_ws/src (Dockerfile COPY context) and land in isaac_sim_ros_ws/install"
  - "ABI verification gate: build wrappers MUST sanity-check the .so ABI tag (not cpython-310) AND round-trip-import the module via venv-activated Python before claiming success"
  - "Snapshot script as discovery oracle: when CONTEXT.md assumptions need confirming against live infra, the pattern is bash + docker-bringup + ros2 topic list/info + trap cleanup — capture EVERYTHING to .planning/phases/<N>/snapshot/, then commit raw artifacts alongside the doc that interprets them"
  - "Authoritative-source fallback for empty live captures: when a capture is expected-empty (passing policy, healthy state, etc.), document the URDF/SDF/code source-of-truth that defines the configuration, so future plans don't mistake 'no live data' for 'feature missing'"

requirements-completed: []

# Metrics
duration: 49min
completed: 2026-05-03
---

# Phase 2 Plan 01: Custom Messages Workspace Build + Topic Discovery Summary

**D-05 ABI landmine fixed (aic_control_interfaces + ros_gz_interfaces rebuilt for Python 3.11), Open Q1 settled (all four critical Phase 2 topics confirmed live), and off-limit prim mapping locked to 3 URDF-authoritative USD prefixes — ready for Plans 02-02..06 to consume.**

## Performance

- **Duration:** 49 min
- **Started:** 2026-05-03T19:23:30Z
- **Completed:** 2026-05-03T20:13:16Z
- **Tasks:** 3 (vendor+build, doc+wrapper, snapshot+mapping)
- **Files modified:** 0
- **Files created:** 9 (4 in repo + 5 raw snapshot artifacts under .planning/)

## Accomplishments

- **D-05 ABI fix landed**: `aic_control_interfaces` + `ros_gz_interfaces` now build for Python 3.11 in the IsaacSim-ros_workspaces tree. Verified via `bash -c 'source ~/env_isaaclab/bin/activate && ~/env_isaaclab/bin/python -c "from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState; from ros_gz_interfaces.msg import Contacts, Contact, Entity; print(\"OK\")"'` returning `OK`.

- **Open Q1 settled**: snapshot of running aic_eval container confirms ALL FOUR Plan 02 controller topic names + types match CONTEXT.md assumptions exactly. No CONTEXT.md edits needed.

- **Off-limit prim mapping documented**: Authoritative source is `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` lines 122-130 (3 Gazebo top-level models: enclosure, enclosure walls, task_board → 3 USD prefixes: /World/Enclosure, /World/Enclosure_Walls, /World/TaskBoard). Live capture was empty (expected — CheatCode is a passing policy that doesn't trigger off-limit) but URDF source is canonical.

- **Two re-runnable wrapper scripts**: `build_aic_msgs.sh` (idempotent vendor+build+verify) and `snapshot_aic_eval_offlimit.sh` (idempotent docker bringup + topic capture + cleanup) — future re-runs are one-shot.

## Task Commits

1. **Task 1+2: D-05 ABI fix + docs/wrapper** - `c5588e0` (feat) — vendoring + Docker rebuild + idempotent wrapper script + comprehensive procedure doc with the three landmines (D-05 ABI, install-path, Dockerfile COPY context) preserved
2. **Task 3: Open Q1 + D-10 settlement** - `523cfd7` (feat) — snapshot script + raw captured artifacts + interpretive doc with confirmation table + DEFAULT_OFF_LIMIT_PRIMS constant

## Files Created

**Repo (committed):**
- `exts/aic-dt/docs/aic-msgs-setup.md` — D-05 ABI landmine context + procedure + install-path landmine + COPY-context landmine + verification recipe + LD_LIBRARY_PATH note + re-running discipline
- `exts/aic-dt/scripts/build_aic_msgs.sh` — idempotent vendor+build+ABI-verify+round-trip-import wrapper (executable, set -euo pipefail, syntax-clean)
- `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh` — idempotent docker bringup + topic-list + topic-info + 120s off_limit echo + sample echoes + trap-cleanup wrapper
- `exts/aic-dt/docs/offlimit-prim-mapping.md` — Open Q1 confirmation table + URDF-derived authoritative model list + USD prim prefix mapping + DEFAULT_OFF_LIMIT_PRIMS constant + re-run instructions + verification snippet
- `.planning/phases/02-controller-loop/snapshot/{all_topics, aic_controller_topic_list, aic_controller_topic_info, aic_eval_offlimit_capture, sample_*}.{txt,yaml}` — raw captured artifacts (committed alongside the interpretive doc per Phase 1 PARITY-05 audit-trail pattern)

**Workspace (outside repo, NOT committed):**
- `~/IsaacSim-ros_workspaces/humble_ws/src/aic_control_interfaces/` (vendored from `~/Documents/aic`)
- `~/IsaacSim-ros_workspaces/humble_ws/src/ros_gz_interfaces/` (vendored from gazebosim/ros_gz humble)
- `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces/` (built artifacts, cp311 ABI)
- `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/ros_gz_interfaces/local/lib/python3.11/dist-packages/ros_gz_interfaces/` (built artifacts, cp311 ABI)

## Decisions Made

- **Replaced PYTHONPATH-link approach (CONTEXT.md D-05) with workspace rebuild.** Confirmed empirically: AIC pixi env builds against Python 3.12 — ABI-incompatible with Isaac Sim 5.0's Python 3.11. Cannot share .so artifacts across minor Python versions. Replacement: vendor source + rebuild against 3.11 in the workspace tree env_isaaclab already sources.

- **Vendored to the Dockerfile COPY context (`humble_ws/src/`), not the post-build extract dir (`build_ws/humble/humble_ws/src/`).** The two paths share the segment "humble_ws/src" but live at different repo roots. The Dockerfile reads from the former; the latter is just where build_ros.sh extracts results into AFTER the Docker build. First attempt confused these — doc and wrapper script call out the distinction explicitly.

- **Accepted that artifacts land in the OUTER `isaac_sim_ros_ws/install/<pkg>/local/lib/...` per-package layout (no `--merge-install` for the outer workspace), NOT in the inner `humble_ws/install/local/lib/python3.11/dist-packages/`.** Plan's expected install path was wrong; actual install path differs but env_isaaclab/bin/activate already sources both setups so the import works transparently. Documented in aic-msgs-setup.md "Where the built artifacts land" section.

- **Off-limit prim set is prefix-based, not link-enumerated.** Authoritative source is the URDF, which lists 3 top-level Gazebo models. The Gazebo plugin walks all collision children dynamically — so the Isaac Sim equivalent is "any contact whose actor_path starts with one of these 3 USD prefixes". DEFAULT_OFF_LIMIT_PRIMS has 3 entries, not the 6+ originally anticipated by the planner.

- **Live off-limit capture was empty (CheatCode produces zero off-limit events) — accepted as the EXPECTED case rather than treating it as a failure.** Documented WallPresser/WallToucher as the policy to swap to if a future plan needs a non-empty capture.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Discovery] Plan-expected install path was wrong for IsaacSim-ros_workspaces 5.0.0**
- **Found during:** Task 1 (workspace rebuild — verification step)
- **Issue:** Plan asserted built artifacts would land at `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/`. Actual install location is `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces/`.
- **Root cause:** `build_ros.sh` runs TWO colcon builds inside Docker — inner `humble_ws/` is `--merge-install` (base humble pkgs); outer `build_ws/` is per-package install (our custom pkgs). Custom packages we vendored land in the OUTER workspace because the Dockerfile COPYs from `humble_ws/src` into `/workspace/build_ws/src` (NOT `/workspace/humble_ws/src`).
- **Fix:** Updated build wrapper script + doc to reference the correct path; round-trip import verification still works because env_isaaclab/bin/activate sources both inner and outer local_setup.bash files (auto-discovery from the per-package install layout).
- **Files modified:** exts/aic-dt/docs/aic-msgs-setup.md, exts/aic-dt/scripts/build_aic_msgs.sh
- **Verification:** `bash -c 'source ~/env_isaaclab/bin/activate && python -c "from aic_control_interfaces.msg import JointMotionUpdate; print(\"OK\")"'` returns OK
- **Committed in:** `c5588e0` (Task 1+2 commit)

**2. [Rule 1 - Workflow] First vendoring attempt copied to wrong source path**
- **Found during:** Task 1 (first Docker build cycle)
- **Issue:** Initial vendor target was `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/` (the post-build extracted dir from Phase 1's rclpy build). Docker build completed successfully but our packages did not appear in the build log — they were silently absent because the Dockerfile's `COPY humble_ws/src /workspace/build_ws/src` line reads from the REPO root's `humble_ws/src`, not the build_ws extract dir.
- **Root cause:** Two paths share the segment "humble_ws/src" but live at different repo roots. Easy to confuse without reading the Dockerfile.
- **Fix:** Re-vendored to `~/IsaacSim-ros_workspaces/humble_ws/src/`. Second build picked them up correctly. Wrapper script and doc explicitly call out the distinction.
- **Files modified:** (no repo files — workflow correction)
- **Verification:** Second build log shows `Starting >>> aic_control_interfaces` + `Starting >>> ros_gz_interfaces` + their `Finished <<<` lines.
- **Committed in:** doc/script in `c5588e0` capture the correct path going forward

**3. [Rule 3 - Transient blocker] First Docker build hit launchpad.net 504 Gateway Timeout while adding deadsnakes/ppa**
- **Found during:** Task 1 (first Docker build cycle)
- **Issue:** `RUN add-apt-repository -y ppa:deadsnakes/ppa` failed with HTTP 504 from launchpad.net while fetching the PPA signing key. This is a transient infrastructure issue, not a build defect.
- **Fix:** Re-ran `bash build_ros.sh -d humble -v 22.04` — second attempt succeeded. Documented in build_aic_msgs.sh script comment as a known transient.
- **Files modified:** exts/aic-dt/scripts/build_aic_msgs.sh (comment explaining the transient)
- **Verification:** Build completed; both packages built.
- **Committed in:** `c5588e0`

**4. [Rule 1 - Discovery] aic_eval echo of Contacts/JointMotionUpdate/etc. failed with "message type invalid"**
- **Found during:** Task 3 (snapshot script — sample echo step)
- **Issue:** `ros2 topic echo` from inside aic_eval can't echo messages of type `aic_control_interfaces/msg/*` or `ros_gz_interfaces/msg/Contacts` — the kilted-side daemon-less ros2 CLI doesn't have these in its discovery despite the topics being live (proven by `ros2 topic info` succeeding for the same topics).
- **Root cause:** Likely a Zenoh-RMW discovery scope issue on the ros2-cli side.
- **Fix:** Treated topic-info as the authoritative type-confirmation source (which DID work). Off-limit-Contacts echo was expected-empty anyway (CheatCode is a passing policy). For future, the recommended override is to swap CheatCode → WallPresser/WallToucher in the script POLICY arg, doc'd in offlimit-prim-mapping.md "Re-running the snapshot" section.
- **Files modified:** exts/aic-dt/docs/offlimit-prim-mapping.md (re-run instructions)
- **Verification:** Topic-info captures show all four critical topics with correct types — confirmation gate satisfied.
- **Committed in:** `523cfd7` (Task 3 commit)

---

**Total deviations:** 4 auto-fixed (3 Rule 1 discoveries, 1 Rule 3 transient)
**Impact on plan:** All discoveries surfaced real landmines that would have bitten Plans 02-02..06 hard if left undocumented. Spirit of plan preserved (D-05 fix, Open Q1 settled, off-limit mapping doc'd); execution surfaced + documented + worked-around the actual mechanics. No scope creep.

## Issues Encountered

- **Three landmines surfaced during execution** (the install-path divergence, the COPY-context confusion, the launchpad transient). All resolved + documented. The first two are now baked into the wrapper script + doc so future re-runs sail through.
- **Live `ros2 topic echo` from aic_eval fails for custom message types** (Zenoh + kilted-CLI quirk). Worked around by relying on `ros2 topic info` (which works) for type confirmation; off-limit echo was expected-empty under CheatCode anyway.

## User Setup Required

None — fully autonomous M1 mode execution. The build workflow has no manual configuration steps; one `bash exts/aic-dt/scripts/build_aic_msgs.sh` invocation re-creates the entire state from scratch.

## Next Phase Readiness

**Hand-off to Plan 02-02 (controller_loop.py skeleton):** Custom messages are importable. `controller_loop.py` can `from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState` and `from ros_gz_interfaces.msg import Contacts, Contact, Entity` after applying the parity_publishers.py sys.path swap pattern (which env_isaaclab/bin/activate already wires).

**Hand-off to Plans 02-03..06 (subscriber/publisher atoms):** All four Plan 02 critical topic names + types confirmed live and CONTEXT.md-correct:
- `/aic_controller/joint_commands` → `aic_control_interfaces/msg/JointMotionUpdate` (sub for Plan 02-03)
- `/aic_controller/pose_commands` → `aic_control_interfaces/msg/MotionUpdate` (sub for Plan 02-04)
- `/aic_controller/controller_state` → `aic_control_interfaces/msg/ControllerState` (pub for Plan 02-05)
- `/aic/gazebo/contacts/off_limit` → `ros_gz_interfaces/msg/Contacts` (pub for Plan 02-06)

**Hand-off to Plan 02-06 (off-limit Contacts publisher):** `DEFAULT_OFF_LIMIT_PRIMS = ["/World/Enclosure", "/World/Enclosure_Walls", "/World/TaskBoard"]`. Use prefix-startswith matching in the contact callback — not exact-link-name matching.

**No blockers for Phase 2 execution** to proceed to Plan 02-02.

## Self-Check: PASSED

- `c5588e0`: FOUND in git log
- `523cfd7`: FOUND in git log
- `exts/aic-dt/docs/aic-msgs-setup.md`: FOUND
- `exts/aic-dt/scripts/build_aic_msgs.sh`: FOUND (executable)
- `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh`: FOUND (executable)
- `exts/aic-dt/docs/offlimit-prim-mapping.md`: FOUND
- `.planning/phases/02-controller-loop/snapshot/{all_topics,aic_controller_topic_list,aic_controller_topic_info,aic_eval_offlimit_capture,sample_*}.{txt,yaml}`: 7 files FOUND
- Workspace install paths: aic_control_interfaces/__init__.py FOUND; ros_gz_interfaces/__init__.py FOUND; no cpython-310 .so files
- Round-trip import: `from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState; from ros_gz_interfaces.msg import Contacts, Contact, Entity` returns OK

---
*Phase: 02-controller-loop*
*Completed: 2026-05-03*
