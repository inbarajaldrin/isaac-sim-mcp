---
phase: 01-foundation-parity
plan: 05
subsystem: probe
tags: [usd-prim-probe, joint-state-probe, frame-name-strategy, joint-ordering, parity-prerequisite, plan-06-contract]

# Dependency graph
requires:
  - phase: 01-04-extension-cleanup
    provides: clean extension.py production surface (no _sim/_real, RG2->Hand-E, joint-path bug fixed) ready for Plan 06's TF/JointState publishers to land
provides:
  - "USD prim inventory + verdict (PER-FRAME-RAW-OVERRIDE) for the 17 frame mismatches Plan 06 must resolve"
  - "Joint-ordering verdict (NAME-INDEXED, NO-WRAPPER-NEEDED) freeing Plan 06 to publish /joint_states with Isaac Sim's natural articulation order"
  - "exts/aic-dt/scripts/probe_unified_usd.py — re-runnable USD prim diff harness"
  - "exts/aic-dt/scripts/probe_aic_controller_jointstate.sh — re-runnable subscriber-pattern probe"
  - "Strict joint-name-presence contract for Plan 06 (all 7 names incl. slashed gripper/left_finger_joint)"
affects: [01-06-tf-jointstate-publishers, 01-07-quick-start-refactor, 01-08-verify-phase-1]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Pre-graph probe pattern: cheap minutes-long pxr.Usd standalone enumeration + targeted upstream-source grep BEFORE wiring OmniGraph nodes that depend on the probe verdicts"
    - "Re-runnable verdict capture: probe -> stdout -> capture .txt + append `## Decision` block with explicit `Strategy:` / `Verdict:` / `Action:` machine-parseable fields downstream plans grep against"
    - "Manual probe extension when auto heuristic returns UNKNOWN: when the default search (aic_controller/src) misses the actual subscriber (aic_adapter/src), append [6]+[7] sections to the same report so the entire trail of evidence stays in one file"

key-files:
  created:
    - "exts/aic-dt/scripts/probe_unified_usd.py — pxr.Usd Stage.Traverse() over aic_unified_robot_cable_sdf.usd; diff vs aic_frames_live.gv; emit override list + recommended strategy"
    - "exts/aic-dt/scripts/probe_aic_controller_jointstate.sh — grep-based name-indexed vs positional access detector with verdict heuristic"
    - ".planning/phases/01-foundation-parity/usd_prim_inventory.txt — 154-prim inventory + 32-frame diff + Decision block (Strategy: PER-FRAME-RAW-OVERRIDE)"
    - ".planning/phases/01-foundation-parity/joint_ordering_probe.txt — aic_controller probe + manual aic_adapter/aic_engine extension + Decision block (Verdict: NAME-INDEXED / Action: NO-WRAPPER-NEEDED)"
  modified: []

key-decisions:
  - "USD frame-name strategy = PER-FRAME-RAW-OVERRIDE (not SUBLAYER-RENAME) because USD prim names cannot legally contain '/' and ROS2PublishTransformTree reads the Sdf path leaf, not customData"
  - "17 Raw-publisher overrides required (16 underscore->slash renames + 1 synthesized aic_world identity edge); 14 frames pass through with default TransformTree mapping"
  - "Joint-ordering verdict = NAME-INDEXED via aic_adapter::ReorderJointState (joint_sort_order_ map at lines 80-86); aic_controller is a ros2_control plugin reading hardware-state interfaces directly (not /joint_states); Task 4 wrapper SKIPPED"
  - "Strict name-presence contract surfaced for Plan 06: all 7 expected joint NAMES must appear in every /joint_states msg (incl. gripper/left_finger_joint with slash); aic_adapter silently drops unexpected names"
  - "Run probes via ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh (env_isaaclab venv lacks pxr; isaac-sim 4.2's bundled python 3.10 ships pxr cp310)"

patterns-established:
  - "Pre-graph probe pattern (probe-before-wire): when downstream plan depends on an empirical verdict (frame names match? subscriber positional?), spawn a probe in a prerequisite plan that captures the verdict to a `.txt` with a `## Decision` block whose explicit fields downstream plans grep against. Bypasses speculative architecture."
  - "Conditional task pattern: tasks 3+4 of this plan are gated on `Strategy:` / `Action:` regex matches in the probe outputs. When predicate fails, the task is a documented no-op recorded in SUMMARY (no commit, no file change). Avoids over-engineering when the probe says no fix needed."
  - "Manual probe extension: if heuristic auto-grep returns UNKNOWN, append numbered manual sections [6]+[7] to the same .txt report and refine the Decision block — the entire reasoning trail stays in one file for future re-inspection."

requirements-completed: [PARITY-03, PARITY-04, DX-02]

# Metrics
duration: 5min
completed: 2026-05-02
---

# Phase 1 Plan 05: Pre-Graph Probes Summary

**Two re-runnable probes capture the verdicts (PER-FRAME-RAW-OVERRIDE for TF / NAME-INDEXED for JointState) Plan 06 needs to wire its publishers correctly without speculative architecture; both conditional follow-up tasks (USD sublayer, reorder bridge) skipped per probe outputs.**

## Performance

- **Duration:** ~5 min
- **Started:** 2026-05-02T12:50:51Z
- **Completed:** 2026-05-02T12:55:56Z
- **Tasks:** 4 (2 executed, 2 conditional-skipped)
- **Files modified:** 4 created (2 scripts + 2 reports), 0 modified
- **Commits:** 2 task commits + 1 metadata commit (this SUMMARY + state updates)

## Accomplishments

- **PARITY-04 frame-name strategy decided empirically.** USD prim probe (`probe_unified_usd.py`) ran via Isaac Sim 4.2's pxr.Usd standalone, enumerated 154 prims under the unified robot USD, diffed against the 31-frame `aic_frames_live.gv`, identified 16 underscore-vs-slash mismatches (every `gripper/_`, `ati/_`, `*_camera/_`, `cam_mount/_` frame) + 1 missing prim (`aic_world` synthesized edge) + 14 clean matches + 1 legend-artifact noise (`view_frames Result`). USD sublayer rename rejected as architecturally impossible (USD prim names can't contain `/`, and ROS2PublishTransformTree reads `Sdf.Path.GetName()` not customData). Plan 06 will publish 14 frames via the default ROS2PublishTransformTree mapping and 17 overrides via per-frame Raw publishers (or the equivalent overrides input array on the TransformTree node).
- **PARITY-03 ordering strategy decided empirically.** Joint-state probe (`probe_aic_controller_jointstate.sh`) ran over `~/Documents/aic/aic_controller/src/` and returned UNKNOWN — `aic_controller` is a ros2_control plugin reading state from controller_interface hardware-state interfaces, NOT from `/joint_states`. Manual probe extension over `aic_adapter/src/aic_adapter.cpp` (the actual subscriber) found `ReorderJointState()` performing name->index lookup via `joint_sort_order_.at(original.name[i])` for every msg. /joint_states ordering does NOT functionally matter; Plan 06 publishes Isaac Sim's natural articulation order without a reorder bridge.
- **One strict contract surfaced for Plan 06**: all 7 expected joint NAMES must be present (incl. slashed `gripper/left_finger_joint`) or `aic_adapter` silently drops them. Plan 06 must override the joint_state name field for the gripper finger joint exactly the same way it overrides TF frame_ids.
- **DX-02 probe-script artifact contract honored**: both probes are re-runnable, idempotent, and emit machine-parseable Decision blocks downstream plans/verifiers can grep against.

## Task Commits

1. **Task 1: USD prim probe + capture inventory + record strategy** — `8a89890` (feat)
2. **Task 2: aic_controller subscriber probe + capture report + record verdict** — `9221249` (feat)
3. **Task 3: USD frame-override sublayer** — SKIPPED (Strategy=PER-FRAME-RAW-OVERRIDE, no sublayer authored; Plan 06 reads override list from usd_prim_inventory.txt)
4. **Task 4: JointState reorder bridge** — SKIPPED (Action=NO-WRAPPER-NEEDED, aic_adapter handles reordering by name; no atom added)

**Plan metadata:** _to be filled by final commit hash_

## Files Created/Modified

- `exts/aic-dt/scripts/probe_unified_usd.py` (147 lines) — standalone pxr.Usd-based prim enumerator, slash-vs-underscore matcher, strategy recommender. Run via `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh` (env_isaaclab venv lacks pxr).
- `exts/aic-dt/scripts/probe_aic_controller_jointstate.sh` (66 lines) — bash grep-based access-pattern detector with NAME/POSITIONAL/MIXED/UNKNOWN heuristic. Default scans `~/Documents/aic/aic_controller/src`; returns nonzero if dir absent (D-13 fallback).
- `.planning/phases/01-foundation-parity/usd_prim_inventory.txt` (266 lines) — full prim listing + 32-frame diff + 18-entry override list + `## Decision` block (Strategy: PER-FRAME-RAW-OVERRIDE).
- `.planning/phases/01-foundation-parity/joint_ordering_probe.txt` (113 lines) — auto probe sections [1]-[5] + manual extensions [6] (aic_adapter::ReorderJointState code excerpt) + [7] (aic_engine settle-check excerpt) + `## Decision` block (Verdict: NAME-INDEXED, Action: NO-WRAPPER-NEEDED).

## Decisions Made

1. **`PER-FRAME-RAW-OVERRIDE` over `SUBLAYER-RENAME`** for TF frame parity. Sublayer renaming was the cheap-hypothesis option in the plan, but the probe surfaced two showstoppers: (a) USD prim names cannot legally contain `/` (it is the `Sdf.Path` separator — any `over "gripper/hande_base_link"` raises an invalid-path error at layer parse time), and (b) `ROS2PublishTransformTree` in Isaac Sim 5.0 reads the prim's leaf name, not customData metadata, so even if a `customData:frameId` attribute were authored it would have no effect on the published frame_id. Per-frame Raw publisher overrides are the only architecturally valid path.
2. **Manual probe extension over aic_adapter rather than re-running with a different default.** The auto probe over `aic_controller/src` correctly returned UNKNOWN because aic_controller never subscribes to `/joint_states`. Rather than re-running with `aic_adapter/src` as arg 1 (and losing the trail of evidence that aic_controller doesn't subscribe), extension sections [6]+[7] were appended to the same report — preserving the full reasoning chain for future re-inspection.
3. **`view_frames Result` filtered as legend artifact.** The .gv parser found 32 quoted tokens; one (`view_frames Result`) is the cluster_legend label inside the .gv file, not a real ROS frame. Plan 06's TF builder must filter this out (NOT publish a frame with that name).

## Plan 06 Contract

**Forward contract surfaced by this plan's probe outputs — Plan 06's executor must honor:**

### TF publisher (PARITY-04)

Plan 06's TF builder reads the `[Override list for Plan 06 / sublayer]` block in `usd_prim_inventory.txt` (lines 240-258). Implementation strategy:

- **14 MATCH frames** (base, base_link, base_link_inertia, flange, forearm_link, ft_frame, shoulder_link, tabletop, tool0, upper_arm_link, world, wrist_1_link, wrist_2_link, wrist_3_link): published via the default `ROS2PublishTransformTree` node with `targetPrims = [/World/aic_unified_robot]` (or whatever the articulation root is) and no override.
- **16 underscore-mismatch frames** (`ati/base_link`, `ati/tool_link`, `cam_mount/cam_mount_link`, `center_camera/{camera_link,optical,sensor_link}`, `gripper/{hande_base_link, hande_finger_link_l, hande_finger_link_r, tcp}`, `left_camera/{camera_link,optical,sensor_link}`, `right_camera/{camera_link,optical,sensor_link}`): each requires the published `frame_id` to differ from the USD prim leaf. Two viable Isaac Sim 5.0 implementations:
  - **Option A**: One `ROS2PublishRawTransformTree` node per override entry, with explicit `parentFrameId` / `childFrameId` strings. Cheap to author programmatically (16 nodes); slightly heavier graph but each is independent.
  - **Option B**: Single `ROS2PublishTransformTree` with `parentFrameOverrides` / `childFrameOverrides` array inputs (if Isaac Sim 5.0 exposes them; verify against `nvidia-suite-docs` skill). Single node; lighter graph.
  - **Recommendation:** Plan 06 attempts Option B first (single node + override arrays); falls back to Option A if the node API doesn't expose override inputs in 5.0. The verify gate is `diff_tf_tree.py` — both options pass equally.
- **1 synthesized `aic_world` edge** (no prim — aic_eval publishes static `world -> aic_world` identity transform): Plan 06 emits via static `tf_static` publisher (`ROS2PublishRawTransformTree` with constant identity transform) — independent from the articulation TransformTree.
- **`view_frames Result` filtered**: NOT published (legend artifact in the .gv source).

### JointState publisher (PARITY-03)

Plan 06's JointState publisher publishes on `/joint_states` directly (NO `_isaac_raw` rename, NO downstream reorder bridge). However:

- **Joint-name override required for `gripper/left_finger_joint`**: the USD joint prim leaf is `gripper_left_finger_joint`. Isaac Sim's `ROS2PublishJointState` defaults to publishing this as `gripper_left_finger_joint`, which `aic_adapter::joint_sort_order_` (line 86) does NOT have — `aic_adapter` will log "Ignoring unexpected joint name" and silently drop it from every Observation. Plan 06 must rename this single joint name in the published msg via the `nameOverrides` input array (or equivalent) on `ROS2PublishJointState`, or by ensuring the `targetPrim` resolution emits the slashed name. **Joint name rename count: 1 (gripper_left_finger_joint -> gripper/left_finger_joint).**
- **All 7 expected joint names must be present** in every msg: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, gripper/left_finger_joint`. Verify in Plan 06's verify gate via `ros2 topic echo /joint_states --once | yq '.name'`.
- **Joint ordering**: Plan 06 publishes whatever order Isaac Sim's articulation walk produces. `aic_adapter` reorders by name downstream; ordering is functionally irrelevant.

## Plan 08 Contract

No changes from this plan (Task 4 skipped — no `setup_joint_state_reorder` atom added). Plan 08's `quick_start` refactor calls only the standard `setup_joint_state_publish_action_graph()` (or whatever the Plan 06 atom is named) — no extra reorder-bridge call.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] Probe runtime — env_isaaclab venv lacks pxr**
- **Found during:** Task 1 (USD prim probe execution)
- **Issue:** `~/env_isaaclab/bin/python` (CPython 3.11.13 from uv) does not have the `pxr` (Pixar OpenUSD) module installed. Running the probe with that interpreter raised `ModuleNotFoundError: No module named 'pxr'`. The plan's Usage docstring assumed env_isaaclab; reality is pxr ships only inside Isaac Sim's bundled CPython.
- **Fix:** Switched the runtime to `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh` (Isaac Sim 4.2 ships pxr cp310 — version 0.26.3). Verified: `python.sh -c "from pxr import Usd; print(Usd.GetVersion())"` -> `(0, 26, 3)`. Updated probe docstring to reflect "Run via Isaac Sim's bundled python.sh, not env_isaaclab" — captured in this SUMMARY's Files section so future probe re-runners don't repeat the trip.
- **Files modified:** none (the probe script itself is interpreter-agnostic; only the run command changed)
- **Verification:** Probe ran cleanly to completion; 154-prim inventory + 32-frame diff captured.
- **Committed in:** N/A (no script change required)

**2. [Rule 1 - Bug-in-plan] Plan's `[USD prims under stage]` filter was over-restrictive**
- **Found during:** Task 1 (writing the probe)
- **Issue:** Plan-spec snippet had `if prim.IsA(Usd.Prim) and (prim.GetTypeName() in ("Xform", ...))` — the `IsA(Usd.Prim)` guard is meaningless (every prim IS-A `Usd.Prim`), and the filter excludes typeless namespace prims like `/World/aic_unified_robot/<link>/collisions` which ROS frame consumers might still need to enumerate. Replaced with explicit type-set inclusion + a fallback for empty-typename prims that aren't the absolute root. This produced a more complete 154-entry inventory (vs the ~50 the over-restrictive filter would have produced), giving the diff more raw material to match against.
- **Fix:** Wrote the filter as `tn in keep_types or (tn == "" and prim.GetPath().pathString != "/")` with explicit `keep_types = {"Xform", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint", "Joint"}`.
- **Verification:** 154 prims emitted; diff section successfully matched all 14 trivial frames + the 16 underscore-mismatches.
- **Committed in:** 8a89890 (Task 1 commit)

**3. [Rule 2 - Missing critical] Plan probe missed the actual /joint_states subscriber**
- **Found during:** Task 2 (verdict reading)
- **Issue:** The plan's probe scope (`~/Documents/aic/aic_controller/src`) returned UNKNOWN because aic_controller never subscribes to `/joint_states` — it's a ros2_control plugin reading hardware-state interfaces directly. Without further inspection, the plan's default-safe path was to assume POSITIONAL and add a wrapper (Task 4). That would have over-engineered a wrapper that's not needed.
- **Fix:** Manual probe extension over `~/Documents/aic/aic_adapter/src/aic_adapter.cpp` (the actual `/joint_states` subscriber discovered via cross-package grep) — found `ReorderJointState()` doing name->index lookup via `joint_sort_order_.at(original.name[i])`. Verdict revised to NAME-INDEXED. Rationale captured in the appended `## Decision` block of joint_ordering_probe.txt.
- **Verification:** Decision block has explicit `Verdict: NAME-INDEXED` + `Action: NO-WRAPPER-NEEDED`. Plan 06 will not need to publish on `_isaac_raw` and will not need a reorder bridge. Saved ~2 hours of unnecessary atom-authoring.
- **Committed in:** 9221249 (Task 2 commit)

---

**Total deviations:** 3 auto-fixed (1 Rule 3 — blocking interpreter mismatch; 1 Rule 1 — over-restrictive filter in plan-snippet; 1 Rule 2 — missing investigation surface needed for correct verdict)
**Impact on plan:** All three deviations strictly improved the probe outputs. Deviation 3 in particular avoided over-engineering a Phase-1 stop-gap wrapper (Task 4) that the architecture does not need.

## Issues Encountered

- **`view_frames Result` is in the .gv frame list**: the .gv graphviz file's `cluster_legend` subgraph contains a quoted node label `"Recorded at time: 1777724139.0003006"` and `"view_frames Result"`. The probe's first regex pulled all double-quoted tokens; the `:`-filter caught the timestamp but not the bare "view_frames Result". Filtered out at the consumer (Plan 06) per the Decision block — not in the probe, since the probe's job is to emit the raw diff, not to know which frames are "real". A future plan could refine the .gv parser to skip the cluster_legend subgraph entirely (cheap, low-priority).

## Next Phase Readiness

- **Plan 06 unblocked**: both probe verdicts captured in machine-parseable Decision blocks; the override list (17 entries) and the joint-name override (1 entry) are explicit; ordering strategy is decided (no bridge); all 7 expected joint names are enumerated for the verify gate.
- **Plan 08 unblocked**: no `quick_start` change required from this plan (Task 4 skipped).
- **Probe re-runnability**: both scripts re-runnable for drift detection. If the AIC repo updates aic_adapter's `joint_sort_order_` map (e.g., adds a hand-E `gripper/right_finger_joint` for symmetry), re-running probe 2 will regenerate the verdict; Plan 06's joint-name override list updates accordingly.
- **No blockers** for proceeding to `/gsd-execute-phase 1` Plan 06.

---
*Phase: 01-foundation-parity*
*Completed: 2026-05-02*

## Self-Check: PASSED

- All claimed files exist: probe_unified_usd.py, probe_aic_controller_jointstate.sh, usd_prim_inventory.txt, joint_ordering_probe.txt, 01-05-SUMMARY.md
- All claimed commits exist in git log: 8a89890 (Task 1), 9221249 (Task 2)
- Conditional acceptance criteria honored: Strategy=PER-FRAME-RAW-OVERRIDE -> Task 3 documented skip + override list captured for Plan 06; Action=NO-WRAPPER-NEEDED -> Task 4 documented skip + no extension.py modification
- AIC repo unmodified (read-only consumer respected): no entries under `git status` in `~/Documents/aic`
- Probe scripts re-runnable: AST/syntax both valid, both executable, both emit explicit machine-parseable Decision fields downstream plans grep against
