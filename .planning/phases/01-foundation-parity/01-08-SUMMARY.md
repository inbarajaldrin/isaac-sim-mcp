---
phase: 01-foundation-parity
plan: 08
subsystem: integration
tags: [quick-start, refactor, changelog, dx-02, integration, milestone]
requires:
  - 01-04-SUMMARY.md (orphan deletes already scrubbed in quick_start)
  - 01-05-SUMMARY.md (joint_ordering_probe verdict drove conditional reorder)
  - 01-06-SUMMARY.md (TF + JointState publisher atoms exist)
  - 01-09-SUMMARY.md (per-component spawn atoms + add_objects clubbing)
provides:
  - quick_start common-path with TF + JointState publishers wired in
  - Phase 1 milestone CHANGELOG entry (13 requirement IDs)
  - DX-02 final audit script (audit_dx02.py)
affects:
  - exts/aic-dt/aic_dt/extension.py (quick_start refactor only — atoms unchanged)
  - exts/aic-dt/docs/CHANGELOG.md (new Phase 1 section)
  - exts/aic-dt/scripts/audit_dx02.py (new)
tech-stack:
  added: []
  patterns:
    - "hasattr-guarded conditional method calls in quick_start (defensive forward-compat for Plan 05 NO-WRAPPER-NEEDED verdict)"
    - "MCP_ONLY_ATOMS exemption for the 4-surface contract (3-surface MCP-only atoms are pre-existing, documented in audit script)"
key-files:
  created:
    - exts/aic-dt/scripts/audit_dx02.py
  modified:
    - exts/aic-dt/aic_dt/extension.py
    - exts/aic-dt/docs/CHANGELOG.md
decisions:
  - quick_start step 7b adds best-effort randomize_lighting (hasattr-guarded; non-blocking on exception) — Phase 1 ROADMAP success criteria reference lighting variability, hooking it into the common path makes the surface complete
  - Conditional setup_joint_state_reorder call uses hasattr guard rather than parsing joint_ordering_probe.txt at runtime — Plan 05 verdict was NO-WRAPPER-NEEDED so the method does not exist; the guard is forward-compatible insurance for a future wrapper plan without re-touching quick_start
  - Audit script exempts 7 MCP-only atoms (play_scene, stop_scene, new_stage, execute_python_code, randomize_lighting, run_policy, randomize_single_object) from the surface-4 UI-button requirement — pre-existing pattern, not a Phase 1 regression; documented as MCP_ONLY_ATOMS in the script
metrics:
  duration: ~10 min
  completed: 2026-05-02
  files_changed: 3
  commits:
    - a93b0c8 refactor(01-08): reorder quick_start per D-12 with TF + JointState publishers
    - 2351ca4 docs(01-08): add Phase 1 milestone entry to CHANGELOG
    - 5082ec0 chore(01-08): add DX-02 4-surface audit script (audit_dx02.py)
---

# Phase 1 Plan 08: quick_start refactor + Phase 1 CHANGELOG + DX-02 final audit Summary

Final integration plan of Phase 1 — wires the new TF + JointState publisher atoms (Plan 06) into the common-path `quick_start` per D-12's documented ordering, adds best-effort `randomize_lighting`, ships the comprehensive Phase 1 milestone CHANGELOG entry covering all 13 requirement IDs, and lands the standalone DX-02 4-surface audit script.

## What landed

### Task 1 — `quick_start` refactor

`exts/aic-dt/aic_dt/extension.py:969-1093` — `async def quick_start(self):` now follows D-12's documented order. The method-call sequence (extracted from the body):

```
self.load_scene()                                # step 1
self.load_robot()                                # step 2
self._timeline.play()                            # step 2b — EARLY PLAY (non-negotiable; PhysX cooking + lock-contention rationale, comment block preserved verbatim)
self.setup_tf_publish_action_graph()             # step 3a — NEW (PARITY-04 / D-10)
self.setup_joint_state_publish_action_graph()    # step 3b — NEW (PARITY-03 / D-11)
if hasattr(self, 'setup_joint_state_reorder'):   # step 3c — defensive hasattr guard
    self.setup_joint_state_reorder()             #          (Plan 05 verdict: NO-WRAPPER-NEEDED, so method absent)
self.setup_action_graph()                        # step 4 — joint-state SUBSCRIBE side; Phase 2 controller-loop hook point
self.setup_wrist_cameras()                       # step 5
self.setup_force_publish_action_graph()          # step 6 — wrench (renamed per Plan 04 to fts_broadcaster/wrench)
self.add_objects()                               # step 7 — clubs Plan 09 per-component spawn atoms + legacy /World/Objects path
if hasattr(self, 'randomize_lighting'):          # step 7b — best-effort
    self.randomize_lighting()                    #          (try/except wrap; non-blocking)
# step 8/9 — workspace camera (unchanged)
```

Diff summary: +47 / -11 lines in extension.py. Method body grew from ~105 lines to ~125 lines; ordering reorganized; docstring expanded with phase-specific drop-in points (Phase 2 controller-loop / Phase 3 GT-TF / Phase 4 trial-loader).

**Early-play preservation:** the comment block at lines ~995-1000 explaining the bisection-derived early-play rationale ("PhysX cooking + many graphs + ROS2 publishers all kick off simultaneously and lock-contend") is preserved verbatim. `self._timeline.play()` remains at step 2b — no reordering.

**Orphan call audit:** `self.create_pose_publisher` and `self.sync_real_poses` are zero-occurrence in extension.py (clean since Plan 04). No fixup needed.

**`_sim`/`_real` cleanliness:** zero non-`initialize_simulation_context_async` matches in the file. Preserved.

### Task 2 — Phase 1 CHANGELOG entry

`exts/aic-dt/docs/CHANGELOG.md` — new top-level section `## [Phase 1: Foundation Parity] — 2026-05-02` (placed above the legacy `## [1.0.0] - 2021-04-26` template entry). +207 lines.

Coverage: all 13 Phase 1 requirement IDs as literal strings (PARITY-01, PARITY-02, PARITY-03, PARITY-04, PARITY-05, PARITY-12, TEX-01, TEX-02, TEX-03, SCENE-01, SCENE-04, DX-01, DX-02), structured as:

- **Added** — TF publisher, JointState publisher, topic-parity reference, verify harness, texture sweep tooling, 7 SCENE-01 spawn atoms, SCENE-04 robot/cable pose kwargs, mount-rail thin-USD wrappers
- **Changed** — PARITY-01 prim-path bug fix, PARITY-02 capitalized layout, asset vendoring, quick_start D-12 reorder, RG2 → Robotiq Hand-E, /tf_static TRANSIENT_LOCAL, camera + wrench renames, add_objects clubbing
- **Removed** — setup_pose_publisher (4 surfaces), sync_real_poses (4 surfaces), `_sim`/`_real` topic suffixes, snake_case `assets/objects/`
- **Known Phase-3 work items** — cable `SetActive(False)` workaround (D-04), object TF frames (SCENE-06), slashed AIC frame names (per-frame Raw overrides deferred), JointState slashed gripper joint name (no `jointNames` input on the OGN node)
- **DX-02 4-surface contract — final audit (Phase 1 ship)** — 11-row table covering all new + deleted atoms with per-surface yes/no
- **Source materials** — pinned aic_eval image digest, Isaac Sim install paths, probe interpreter
- **Phase summaries** — pointer to per-plan SUMMARY.md trail

`Robotiq Hand-E` appears literally; the only `RG2` mentions are inside the `RG2 → Robotiq Hand-E` correction narrative (acceptable per plan's manual-inspection note).

### Task 3 — DX-02 final audit script

`exts/aic-dt/scripts/audit_dx02.py` (new, 197 lines, executable). Per-atom 4-surface invariant verifier:

| Surface | Check |
|---------|-------|
| 1. MCP_TOOL_REGISTRY entry | regex `"<atom>": \{` |
| 2. Handler-map entry        | regex `"<atom>": "_cmd_<atom>"` |
| 3. `_cmd_<name>` method     | regex `def\s+_cmd_<atom>\b` |
| 4. UI button                | scan `ui.Button(...)` chunks for refs to atom / `_cmd_<atom>` / UI alias |

**Inventory checked:**
- 27 PRESENT atoms (18 pre-Phase-1 + 2 new publishers + 7 new spawns)
- 2 ABSENT atoms (setup_pose_publisher, sync_real_poses)

**UI_METHOD_ALIASES** map handles registry-name vs method-name divergence:
- `setup_force_publisher` → `setup_force_publish_action_graph`
- `setup_tf_publisher` → `setup_tf_publish_action_graph`
- `setup_joint_state_publisher` → `setup_joint_state_publish_action_graph`

**MCP_ONLY_ATOMS** exempts 7 control-plane / introspection atoms from the surface-4 UI-button requirement — pre-existing pattern (these atoms are MCP-driven by design, no dedicated UI button). Documented in the script header.

**Current ship state:** `python3 exts/aic-dt/scripts/audit_dx02.py` exits 0. Output: `DX-02 audit PASS: 27 present atoms × 4 surfaces, 2 absent atoms × 4 surfaces — all OK.`

## Final extension.py line count

Before Phase 1: pre-Phase-1 baseline (not measured at plan boundary; estimable from Plan 01-09's note "+486 LOC in extension.py" relative to its own start).
After Phase 1 (this plan): **3303 lines**. Plan 08 net delta: +36 lines (from 3267).

## Phase 1 verification stance

`bash exts/aic-dt/scripts/verify_phase_1.sh` is now end-to-end-runnable:
- SC #1 (topic list shows /joint_states /tf /tf_static, no _sim/_real on production) — `quick_start` produces the surface
- SC #2 (joint name set + ordering vs Gazebo) — JointState publisher publishes natural articulation order; Plan 05 verdict says aic_adapter reorders by name, so wire-order doesn't need to be canonical (BUT the slashed `gripper/left_finger_joint` name is still a known fail point — flagged in CHANGELOG Phase-3 known issues)
- SC #3 (TF tree diff = zero) — TF publisher live; expected to fail on the 17 underscore→slash frame names per Plan 06 deferral, surfaces in verifier loop
- SC #4 (zero pink/black/missing-texture; sweep doc) — assets vendored with sibling textures; texture-sweep.md drives iteration if surfaced
- SC #5 (no _sim/_real grep matches) — verified clean in this plan

## Self-Check: PASSED

- File exists: exts/aic-dt/aic_dt/extension.py (FOUND, 3303 lines, AST parses)
- File exists: exts/aic-dt/docs/CHANGELOG.md (FOUND, contains "Phase 1: Foundation Parity")
- File exists: exts/aic-dt/scripts/audit_dx02.py (FOUND, executable, exits 0)
- Commit a93b0c8 exists in git log (FOUND)
- Commit 2351ca4 exists in git log (FOUND)
- Commit 5082ec0 exists in git log (FOUND)
- All 13 Phase 1 requirement IDs present in CHANGELOG (FOUND)
- `Robotiq Hand-E` literal in CHANGELOG (FOUND)
- DX-02 audit script exits 0 (PASS)
- quick_start contains both new builder calls + early-play step (verified by inline grep)
- Zero `_sim`/`_real` regressions in extension.py (verified)
- Zero orphan `create_pose_publisher` / `sync_real_poses` calls (verified)
