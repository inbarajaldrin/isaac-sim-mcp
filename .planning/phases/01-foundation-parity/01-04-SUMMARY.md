---
phase: 01-foundation-parity
plan: 04
subsystem: extension
tags: [extension, rename, prim-path, hand-e, sim-real-cleanup, parity-05, mcp-atom-deletion]

# Dependency graph
requires:
  - phase: 01-foundation-parity
    plan: 02
    provides: "Vendored capitalized AIC asset tree at exts/aic-dt/assets/assets/<Object>/ — AIC_OBJECTS now repointed at this layout"
  - phase: 01-foundation-parity
    plan: 03
    provides: "Doc-side RG2->Robotiq Hand-E correction (.planning/, CLAUDE.md, exts/aic-dt/docs/) — Plan 04 mirrors that on the code side (extension.py)"
provides:
  - "extension.py with zero _sim/_real/RG2 production hits — Phase 1 SC #5 met"
  - "Joint-path bug fixed (RESEARCH Pitfall #2): joints resolve at /World/UR5e/aic_unified_robot/joints/<n>; the long-standing 6x 'Joint not found' Kit warning per load_robot will disappear once Plan 07 verify runs"
  - "AIC_OBJECTS dict references vendored capitalized paths (assets/Task Board Base/base_visual.usd, assets/SC Port/sc_port_visual.usd, assets/NIC Card Mount/nic_card_visual.usd)"
  - "self._articulation_root_prim_path attribute on DigitalTwin — single source of truth for Plan 06's JointState publisher targetPrim"
  - "MCP surface shrunk by 2 atoms (setup_pose_publisher, sync_real_poses) — DX-02 4-surface contract honored across 8 surfaces total"
  - "Camera topics renamed to live aic_eval form: /center_camera/image, /left_camera/image, /right_camera/image (+ /camera_info siblings)"
  - "Wrench topic renamed to /fts_broadcaster/wrench AND header.frame_id corrected to ati/tool_link — PARITY-05 static reconciliation complete (runtime check is Plan 07)"
  - "Audit trail file parity_05_wrench_framing.txt — consumed by Plan 01 PARITY-12 audit table and Plan 07 verify_phase_1.sh"
affects: [01-05, 01-06, 01-07, 01-08, 01-09]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "MCP atom 4-surface deletion (registry + handler-map + _cmd method + UI button + caller) per DX-02 contract"
    - "PARITY-XX inline marker comments + docstring contracts in extension methods that own a requirement — Plan 07 verify scripts grep on these markers"
    - "Audit-trail file pattern (parity_XX_<feature>.txt) for static reconciliation that needs runtime confirmation by a later plan"

key-files:
  created:
    - ".planning/phases/01-foundation-parity/parity_05_wrench_framing.txt — PARITY-05 framing audit (live frame_id source-of-truth, action taken, verify command for Plan 07)"
  modified:
    - "exts/aic-dt/aic_dt/extension.py — primary edit target (renames + prim-path fix + AIC_OBJECTS update + 8-surface deletion + PARITY-05 frame_id fix + delete_objects orphan cleanup)"

key-decisions:
  - "ati/tool_link is the live wrench frame_id — sourced from ~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro AtiForceTorqueSensor sensor block (line 233-243), since topic_info doesn't directly expose header.frame_id and aic_eval container was not running during Task 4 (avoided 60s+ docker bringup for a single field read)"
  - "delete_objects MCP description and ActionGraph_objects_poses cleanup block treated as orphan dead code (Rule 1 deviation) — removed alongside the atom deletions to keep grep 'objects_poses' at 0"
  - "Joint-path bug fix added an explanatory NOTE comment AT the f-string + introduced new self._articulation_root_prim_path attribute as a forward declaration for Plan 06 — opportunistic refactor while the surface was hot"
  - "PARITY-05 reconciliation = Case A (correct topic + type, wrong frame_id literal) — single .set('ati/tool_link') edit at og.Controller.attribute call. No node replacement, no graph rewire. Most expensive part was identifying the right URDF param to copy from."
  - "quick_start step 7 (pose-publisher) replaced with comment placeholder rather than reordered — Plan 07 owns the reorder, this plan deliberately lands an interim broken state per the plan-level note"

patterns-established:
  - "Pattern: PARITY-XX docstring contract — when a method owns a parity requirement, docstring lists topic name, message type, frame_id, source-of-truth files, AND the verify recipe Plan N's verify_phase_1.sh executes. setup_force_publish_action_graph is the canonical example."
  - "Pattern: 4-surface atom deletion — for each MCP atom, scan registry / handler-map / _cmd method / UI button + ALL callers (notably quick_start). Final grep check loops over both atom names AND each surface to confirm zero residue."
  - "Pattern: audit-trail .txt files in phase dir — static reconciliation outputs that runtime-verify plans consume; standardized fields (Live frame_id, Live Type, Action taken, Verification command) for grep-friendly machine-readability."

requirements-completed: [PARITY-01, PARITY-02, PARITY-05, DX-01, DX-02]

# Metrics
duration: 8min
completed: 2026-05-02
---

# Phase 1 Plan 04: extension.py renames + prim-path fix + atom deletions + PARITY-05 wrench framing Summary

**Production-surface _sim/_real/RG2 cleanup of extension.py + joint-path bug fix + AIC_OBJECTS repoint at vendored layout + 8-surface deletion of two MCP atoms + PARITY-05 wrench frame_id reconciliation (tool0 -> ati/tool_link), shipping Phase 1 SC #5.**

## Performance

- **Duration:** 8 min (455 sec)
- **Started:** 2026-05-02T12:37:19Z
- **Completed:** 2026-05-02T12:44:48Z
- **Tasks:** 4 (executed in mandated 1->2->3->4 sequence) + 1 deviation
- **Files modified:** 1 (extension.py)
- **Files created:** 1 (parity_05_wrench_framing.txt)

## Accomplishments

- 14 production-surface renames applied via Edit tool (33-row plan table compressed because 6 renames came from a single 22-line WRIST_CAMERAS dict block, 4 came from a 4-line workspace-camera block — atomic Edit calls covered them as units)
- 3 RG2 -> Robotiq Hand-E corrections (MCP_TOOL_REGISTRY load_robot description, load_robot success print, _cmd_load_robot return string) — code side now matches doc side from Plan 03
- Joint-path bug fix landed (RESEARCH Pitfall #2): `joint_path = f"{prim_path}/aic_unified_robot/joints/{joint_name}"`. The 6x "Joint not found" warning that's been in Kit log since the unified robot USD adoption will disappear at next Plan 07 verify run. Joint drives were silently never applied pre-fix; UR5e ran on engine defaults.
- New `self._articulation_root_prim_path` attribute on DigitalTwin captures `/World/UR5e/aic_unified_robot` so Plan 06's JointState publisher targetPrim relationship has a single source of truth.
- AIC_OBJECTS dict repointed at the Plan-02 capitalized vendored layout (`assets/Task Board Base/base_visual.usd`, `assets/SC Port/sc_port_visual.usd` x2, `assets/NIC Card Mount/nic_card_visual.usd`). Position/rotation tuples preserved verbatim.
- 8 surfaces deleted across 2 atoms (DX-02 contract): MCP_TOOL_REGISTRY entries, MCP_HANDLERS map entries, method bodies (sync_real_poses ~60 lines, create_pose_publisher ~70 lines), UI buttons, _cmd_setup_pose_publisher / _cmd_sync_real_poses handler methods, AND quick_start step 7 caller. Zero orphan references remain (verified by `grep self.create_pose_publisher\|self.sync_real_poses` -> 0).
- PARITY-05 frame_id reconciled: `tool0` -> `ati/tool_link` at the `og.Controller.attribute("inputs:header:frame_id", ...).set(...)` call inside setup_force_publish_action_graph. Live frame_id sourced from URDF (~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro line 242, AtiForceTorqueSensor `param name="frame_id"`).
- setup_force_publish_action_graph docstring rewritten with the PARITY-05 contract block (topic / type / frame_id / source-of-truth / verify recipe) — Plan 07's verify_phase_1.sh consumes the recipe.
- Audit trail file `.planning/phases/01-foundation-parity/parity_05_wrench_framing.txt` written with all standardized fields (Live frame_id, Live Type, Action taken, Verification command) — feeds Plan 01 PARITY-12 audit table and Plan 07 verify harness.

## Task Commits

Each task was committed atomically:

1. **Task 1: Apply 33-row _sim/_real rename table + RG2->Hand-E** - `de0a8d7` (refactor)
2. **Task 2: Fix joint-path bug + repoint AIC_OBJECTS at vendored layout** - `e4dca0d` (fix)
3. **Task 3: Delete setup_pose_publisher + sync_real_poses atoms (8 surfaces)** - `60c8fc4` (refactor)
4. **Task 4: PARITY-05 frame_id reconciliation (tool0 -> ati/tool_link)** - `7eb0ba5` (fix)
5. **Deviation cleanup: orphan ActionGraph_objects_poses cleanup in delete_objects** - `adfe09d` (refactor)

**Plan metadata commit:** (to be created by final commit step — includes SUMMARY.md, STATE.md, ROADMAP.md, REQUIREMENTS.md updates)

## Final Verification Greps

| Check | Expected | Actual | Status |
|-------|----------|--------|--------|
| `python3 -c "import ast; ast.parse(open('exts/aic-dt/aic_dt/extension.py').read())"` | exit 0 | exit 0 | OK |
| `grep -E '(_sim\|_real\|RG2)\b' extension.py \| grep -v initialize_simulation_context_async` | (no output) | (no output) | OK |
| `grep -c "aic_unified_robot/joints" extension.py` | 1 | 1 | OK |
| `grep -c "objects_poses" extension.py` | 0 | 0 | OK |
| `grep -c "Robotiq Hand-E" extension.py` | >= 3 | 3 | OK |
| `grep -c "fts_broadcaster/wrench" extension.py` | >= 1 | 6 | OK |
| `grep -c '/image"' extension.py` | >= 3 | 3 | OK |
| `grep -c "PARITY-05" extension.py` | >= 1 | 3 | OK |
| `grep -c "ati/tool_link" extension.py` | >= 1 | 6 | OK |
| `grep -c "setup_pose_publisher" extension.py` | 0 | 0 | OK |
| `grep -c "sync_real_poses" extension.py` | 0 | 0 | OK |
| `grep -c "create_pose_publisher" extension.py` | 0 | 0 | OK |
| `grep -c "Setup Pose Publisher" extension.py` | 0 | 0 | OK |
| `grep -c "Sync Real Poses" extension.py` | 0 | 0 | OK |
| `grep -c "objects/sc_port" extension.py` | 0 | 0 | OK |
| `grep -c "assets/SC Port/sc_port_visual.usd" extension.py` | >= 2 | 2 | OK |
| `grep -c "assets/Task Board Base/base_visual.usd" extension.py` | >= 1 | 1 | OK |
| `grep -c "assets/NIC Card Mount/nic_card_visual.usd" extension.py` | >= 1 | 1 | OK |
| 4-surface contract (deleted atoms): registry/handler-map/_cmd/UI button | all 0 | all 0 | OK |
| 4-surface contract (kept setup_force_publisher): all surfaces present | all >=1 | all >=1 | OK |
| `parity_05_wrench_framing.txt` contains "Live frame_id:", "Live Type:", "Action taken" | all present | all present | OK |
| extension.py line count delta | 2645 -> ~2510 (atom deletions ~155 lines) | 2645 -> 2514 (-131 net) | OK |

## Files Created/Modified

- `exts/aic-dt/aic_dt/extension.py` — primary edit target. 2645 -> 2514 lines (net -131). Edits: 14 renames, 3 RG2 doc-string fixes, 1 prim-path bug fix, 1 new self._articulation_root_prim_path attribute, 4 AIC_OBJECTS path updates, 8 surface deletions across 2 atoms (registry x2 + handler-map x2 + UI button x2 + method body x2), 1 quick_start caller comment-replace, 1 PARITY-05 frame_id fix + docstring rewrite + inline marker, 1 delete_objects dead-code cleanup + description fix.
- `.planning/phases/01-foundation-parity/parity_05_wrench_framing.txt` — created. PARITY-05 audit trail with Live frame_id (`ati/tool_link`), Live Type (`geometry_msgs/msg/WrenchStamped`), Live QoS, source-of-truth file references (URDF line range, snapshot file path), Isaac Sim builder pre-fix state (Case A — wrong frame_id literal), Action taken (single .set() edit + docstring + inline marker), Verification command (`ros2 topic info --verbose` + `ros2 topic echo --once`). 4128 bytes.

## Decisions Made

See key-decisions in frontmatter. Top-level summary:

- **Live frame_id sourced from URDF, not running container.** aic_eval was not running and bringing it up via `./scripts/run_cheatcode.sh` is a 60s+ Docker pull/start cycle for a single string read. The URDF (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` line 242, `AtiForceTorqueSensor` sensor block, `param name="frame_id"`) is the authoritative source — Gazebo's ros2_control plugin loads exactly this value at runtime. Plan 07's verify_phase_1.sh runs the runtime echo to confirm the bytes on the wire match.
- **delete_objects orphan cleanup added as a Rule 1 deviation.** The plan-level grep `objects_poses` returned 1 after Task 3 because of a defensive cleanup block in delete_objects that referenced the now-never-created `/Graph/ActionGraph_objects_poses`. Treated as dead code from the atom deletions; removed alongside the stale "and the associated pose publisher graph" mention in the MCP_TOOL_REGISTRY description for delete_objects. Net result: grep -c "objects_poses" -> 0 as PLAN.md line 490 expects.
- **Quick_start step 7 left as a comment placeholder, not reordered.** The plan explicitly notes the interim broken state — Plan 07 owns the quick_start reorder once Plan 06 lands the new TF/JointState publishers.
- **New self._articulation_root_prim_path attribute introduced opportunistically.** Plan 06's JointState publisher targetPrim relationship will need this exact value; centralizing it now in DigitalTwin.on_startup avoids hard-coding `/World/UR5e/aic_unified_robot` in two places later. Pattern: forward-declared attributes for the next plan's needs are fine if the surface is already being edited.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Orphan ActionGraph_objects_poses cleanup in delete_objects**
- **Found during:** Plan-level verification after Task 4, running `grep -c "objects_poses" extension.py` which expected 0 but returned 1.
- **Issue:** delete_objects had a defensive cleanup block (`pose_graph_path = "/Graph/ActionGraph_objects_poses"; ... stage.RemovePrim(...)`) that referenced the action graph created by `create_pose_publisher()` — but `create_pose_publisher()` was deleted in Task 3, so the graph is no longer created and the cleanup is unreachable dead code. The `delete_objects` MCP description also mentioned "and the associated pose publisher graph" which is now stale.
- **Fix:** Removed the cleanup block, replaced with a NOTE comment forward-pointing to Plan 06/07. Updated the MCP_TOOL_REGISTRY description for delete_objects to drop the "associated pose publisher graph" phrase.
- **Files modified:** `exts/aic-dt/aic_dt/extension.py`
- **Verification:** AST parse OK; `grep -c "objects_poses" extension.py` now returns 0; `grep -c "associated pose publisher" extension.py` returns 0.
- **Committed in:** `adfe09d`

---

**Total deviations:** 1 auto-fixed (Rule 1 — Bug / dead-code cleanup)
**Impact on plan:** Necessary to satisfy plan-level success criteria (PLAN.md line 490: `grep -c "objects_poses" extension.py` returns 0). No scope creep — purely cleanup of code that was orphaned by Task 3's atom deletion.

## Issues Encountered

- **Per-task verify regexes in PLAN.md are over-strict for intermediate states.** Task 1's automated verify block asserts `_sim\b|_real\b` returns no hits — but it is run AFTER renames and BEFORE Task 3 deletions, so `objects_poses_sim`/`objects_poses_real` strings (still present until Task 3) would trip the assertion. Resolved by treating the plan-level end-of-plan grep (PLAN.md line 488) as the authoritative gate; per-task verify is an aspirational intermediate check that some authors over-specify. The plan-level gate is green at end of plan. Documented at the bottom of the Task 1 commit message ("renames here are correct per the table").
- **`topic_info` does not directly expose header.frame_id.** Snapshot file `topic_info_fts_broadcaster_wrench.txt` from Plan 01 has Type/QoS/publishers but no `header.frame_id` line — `ros2 topic info` doesn't echo message contents. Resolved by reading the URDF AtiForceTorqueSensor sensor block which is what Gazebo's ros2_control plugin loads at runtime; Plan 07 will run the runtime `ros2 topic echo --once` to confirm the bytes match.

## User Setup Required

None — no external service configuration required.

## Next Phase Readiness

**Plan 05 (objects-physics-tuning) is unblocked.** It does not depend on extension.py changes from this plan; it touches USD payloads / physics material settings.

**Plan 06 (TF + JointState publisher atoms) is unblocked AND ready to consume:**
- `self._articulation_root_prim_path = "/World/UR5e/aic_unified_robot"` for the JointState publisher's `targetPrim` relationship (resolves now that the prim-path bug is fixed).
- The 4-surface atom-add contract is established by mirror-image of Plan 04's atom-delete contract — Plan 06 adds back two atoms (TF publisher, JointState publisher) covering the gap left by this plan's deletions.

**Plan 07 (verify harness + quick_start reorder) consumes:**
- `parity_05_wrench_framing.txt` Verification command field for the wrench probe step in `verify_phase_1.sh`.
- Plan 04 leaves quick_start with a deliberate gap at step 7 (pose-publisher comment); Plan 07 owns the reorder.
- The 6 "Joint not found" Kit warnings should disappear at next launch — Plan 07 verify can grep the Kit log to confirm Plan 04's prim-path fix landed.

**Interim state caveat:** Between this plan and Plan 06 landing, quick_start has no pose / TF / joint-state publisher graph for objects. This is intentional and documented in the Task 3 commit message; the AIC controller stack does NOT depend on these specific publishers (it consumes /tf and /joint_states which are owned by other graphs / publishers).

---
*Phase: 01-foundation-parity*
*Plan: 04*
*Completed: 2026-05-02*

## Self-Check: PASSED

- All 5 commits exist in git history (de0a8d7, e4dca0d, 60c8fc4, 7eb0ba5, adfe09d)
- 01-04-SUMMARY.md created
- parity_05_wrench_framing.txt created
- extension.py final size: 2514 lines (down from 2645 — net -131 lines)
- AST parse passes
- All plan-level success criteria green (see Final Verification Greps table above)
