---
phase: 01-foundation-parity
plan: G02
type: execute
wave: 2
depends_on: [G01]
files_modified:
  - "exts/aic-dt/aic_dt/extension.py"
  - "exts/aic-dt/scripts/build_mount_rail_usds.py"
  - "exts/aic-dt/assets/assets/LC Mount/lc_mount_visual.usd"
  - "exts/aic-dt/assets/assets/SFP Mount/sfp_mount_visual.usd"
  - "exts/aic-dt/assets/assets/SC Mount/sc_mount_visual.usd"
  - "exts/aic-dt/docs/CHANGELOG.md"
autonomous: false
gap_closure: true
addresses_gap: E
requirements: [PARITY-02, SCENE-01, SCENE-04, TEX-01, TEX-02]
tags: [visual-fix, scale-restoration, gazebo-parity]
must_haves:
  truths:
    - "Visual viewport correctness — object scales, sizes, and positions match the prior known-good aic-dt state (which loaded the unified USD with the snake_case assets/objects/ layout and a monolithic add_objects clubbing)"
  artifacts:
    - path: "exts/aic-dt/aic_dt/extension.py"
      provides: "Per-component spawn atoms produce visually-correct geometry equivalent to pre-Plan-02 monolithic add_objects"
      contains: "_spawn_component_via_usd"
    - path: "exts/aic-dt/docs/CHANGELOG.md"
      provides: "Gap E fix entry citing G01 verdict and the specific change applied"
      contains: "Gap E"
  key_links:
    - from: "_spawn_component_via_usd helper / per-atom defaults"
      to: "AIC_OBJECTS legacy translation+rotation values OR explicit xformOp:scale OR re-authored thin USDs"
      via: "G01 verdict-specific fix"
      pattern: "g01-verdict-specific"
---

<objective>
Apply the fix recommended by Plan G01's `visual-regression-diagnosis.md` to restore visual viewport correctness. The exact change set is determined by G01's verdict — this plan's tasks are conditioned on G01's recommendation.

Purpose: Closes Gap E. After this plan, a quick_start in Isaac Sim renders all task-board objects (NIC card, SC ports, NIC card mounts, mount rails, task board base) at the same visual scale and position they rendered at in the pre-Plan-02 state — which the user confirmed was visually correct.

Output: A code/asset change matching G01's recommendation + a CHANGELOG entry + user confirmation via human-verify checkpoint that the viewport now renders correctly.
</objective>

<execution_context>
@$HOME/.claude/get-shit-done/workflows/execute-plan.md
@$HOME/.claude/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/STATE.md
@.planning/phases/01-foundation-parity/01-VERIFICATION.md
@.planning/phases/01-foundation-parity/01-CONTEXT.md
@.planning/phases/01-foundation-parity/01-G01-PLAN.md
@exts/aic-dt/docs/visual-regression-diagnosis.md
@exts/aic-dt/aic_dt/extension.py
@exts/aic-dt/scripts/build_mount_rail_usds.py
@exts/aic-dt/scripts/diff_world_bbox.py
@CLAUDE.md
@~/.claude/skills/nvidia-suite-docs/SKILL.md
@~/.claude/skills/isaac-sim-extension-dev/SKILL.md

<interfaces>
<!-- Verdict-conditional behavior:
     METERSPERUNIT_DRIFT      -> edit affected USD layers in-place per D-06; OR add reference-time scale compensation in _spawn_component_via_usd
     SPAWN_ATOM_TRANSFORM_DRIFT -> update default x/y/z/roll/pitch/yaw values in _cmd_spawn_* methods to match the old monolithic AIC_OBJECTS values
     GLB_USD_SCALE_BAKED_WRONG -> re-author thin USDs in build_mount_rail_usds.py with explicit UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(...))
     ADD_REFERENCE_UNIT_DRIFT -> add explicit identity xformOp:scale in _spawn_component_via_usd helper after AddReference
     COMPOUND                 -> ordered list per G01
     INCONCLUSIVE             -> NOT a fix — re-run G01 with the protocol it specified (this plan would terminate at Task 0 with a checkpoint)

     Cache discipline (CLAUDE.md):
     - Before any quick_start test: ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py status
     - If cache size < 100MB: prime_usd_cache.py restore known-good
     - After successful quick_start: prime_usd_cache.py snapshot
-->

Spawn atom locations (extension.py):
  _spawn_component_via_usd helper:    line 2898
  _cmd_spawn_task_board_base:          line 2936  (default x=0.25, y=0.0, z=1.14)
  _cmd_spawn_lc_mount_rail:            line 2948
  _cmd_spawn_sfp_mount_rail:           line 2970
  _cmd_spawn_sc_mount_rail:            line 2992
  _cmd_spawn_sc_port:                  line 3014
  _cmd_spawn_nic_card_mount:           line 3035
  _cmd_spawn_nic_card:                 line 3054
  AIC_OBJECTS dict (current capitalized paths): line 58
  add_objects (legacy + new clubbing):         line 1912

For SPAWN_ATOM_TRANSFORM_DRIFT specifically: the old monolithic AIC_OBJECTS values are recoverable via:
  git show 2be9e0c:exts/aic-dt/aic_dt/extension.py | grep -A 80 "AIC_OBJECTS = {"
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: Read G01 verdict and apply the verdict-specific fix</name>
  <files>exts/aic-dt/aic_dt/extension.py, exts/aic-dt/scripts/build_mount_rail_usds.py, exts/aic-dt/assets/assets/LC Mount/lc_mount_visual.usd, exts/aic-dt/assets/assets/SFP Mount/sfp_mount_visual.usd, exts/aic-dt/assets/assets/SC Mount/sc_mount_visual.usd</files>
  <action>
    Read `exts/aic-dt/docs/visual-regression-diagnosis.md` produced by G01. Extract the **VERDICT** token. Apply the verdict-specific fix as documented in the diagnosis's "Recommended Fix Strategy" section.

    The five verdict-specific fix patterns (apply ONLY the one matching G01's verdict):

    **VERDICT = METERSPERUNIT_DRIFT**
    - For each layer named in the diagnosis with a non-1.0 metersPerUnit, edit the USD in place per D-06 (vendored-tree edits, NOT override layers). Use `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh` and pxr:
      ```python
      from pxr import Usd, UsdGeom
      stage = Usd.Stage.Open(path)
      UsdGeom.SetStageMetersPerUnit(stage, 1.0)
      stage.GetRootLayer().Save()
      ```
    - OR if metersPerUnit conflict is between robot USD and vendored object USDs and re-authoring is risky, add reference-time scale compensation in `_spawn_component_via_usd`: after `AddReference`, multiply spawn pose translation by the scale ratio (and add explicit xformOp:scale).

    **VERDICT = SPAWN_ATOM_TRANSFORM_DRIFT**
    - Read the old AIC_OBJECTS values: `git show 2be9e0c:exts/aic-dt/aic_dt/extension.py | sed -n '/^AIC_OBJECTS = {/,/^}/p'` — extract per-object position + rotation.
    - Update the defaults in each `_cmd_spawn_*` method (lines 2936-3068) to match those legacy values. Quaternion-rotation legacy entries map to RPY via the existing `_quat_to_rpy` helper — apply that conversion offline and bake the RPY values as defaults.
    - Update inline docstrings to cite "default values restored from pre-Plan-02 monolithic add_objects per G01 SPAWN_ATOM_TRANSFORM_DRIFT verdict."

    **VERDICT = GLB_USD_SCALE_BAKED_WRONG**
    - Edit `build_mount_rail_usds.py`: in the USD authoring path, after `prim.GetReferences().AddReference("./<mesh>.glb")`, add explicit identity-scale (or G01-recommended-scale) op:
      ```python
      from pxr import UsdGeom, Gf
      UsdGeom.Xformable(prim).ClearXformOpOrder()
      UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(<sx>, <sy>, <sz>))
      ```
    - Re-run the script to regenerate `LC Mount/lc_mount_visual.usd`, `SFP Mount/sfp_mount_visual.usd`, `SC Mount/sc_mount_visual.usd`. The three USDs are committed.

    **VERDICT = ADD_REFERENCE_UNIT_DRIFT**
    - Edit `_spawn_component_via_usd` (extension.py:2898) to add explicit `xformOp:scale = (1, 1, 1)` (or G01-recommended) after `AddReference`. Use `UsdGeom.Xformable(spawned_prim).AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))` placed AFTER `AddReference` and BEFORE `AddTranslateOp`/`AddRotateXYZOp`. The xformOpOrder must be Translate→RotateXYZ→Scale or Scale→Translate→RotateXYZ depending on G01's recommendation.

    **VERDICT = COMPOUND**
    - Apply each sub-fix in the order G01 specifies. Each commit is its own atomic change.

    **VERDICT = INCONCLUSIVE**
    - DO NOT proceed. Stop the task. Open a checkpoint:decision asking the user whether to (a) re-run G01 with a stricter capture protocol, (b) revert to the pre-Plan-02 layout entirely as a safer fallback (revert to commit 2be9e0c paths but keep the texture sibling fix from Plan 02), or (c) accept the regression for Phase 1 verifier purposes and defer the fix.

    Per CLAUDE.md canonical references rule: this task touches USD authoring (xformOp:scale, AddReference, metersPerUnit) — consult `~/.claude/skills/nvidia-suite-docs/SKILL.md` (USD authoring patterns, xformOp ordering, metersPerUnit semantics) before making any edit. If the fix involves OmniGraph (it shouldn't, but COMPOUND verdicts may), also consult that skill's OmniGraph section.

    Make the edit specific to the verdict. Do NOT apply multiple verdicts speculatively.
  </action>
  <verify>
    <automated>VERDICT=$(grep -oE '\*\*VERDICT: [A-Z_]+\*\*' exts/aic-dt/docs/visual-regression-diagnosis.md | head -1) &amp;&amp; echo "Applied for $VERDICT" &amp;&amp; python3 -m py_compile exts/aic-dt/aic_dt/extension.py &amp;&amp; python3 -m py_compile exts/aic-dt/scripts/build_mount_rail_usds.py &amp;&amp; git diff --stat -- exts/aic-dt/ | grep -E '(extension\.py|build_mount_rail|\.usd)' &amp;&amp; echo "OK fix applied"</automated>
  </verify>
  <done>
    Verdict-specific fix is applied. extension.py + build_mount_rail_usds.py compile. The git diff shows changes ONLY in the files relevant to G01's verdict (no speculative edits in unrelated areas). If verdict is INCONCLUSIVE, the task is paused at the checkpoint and no edits are made.
  </done>
</task>

<task type="checkpoint:human-verify" gate="blocking">
  <name>Task 2: Human-verify viewport renders correctly after fix</name>
  <what-built>Verdict-specific fix from Task 1 applied to the spawn / asset / unit-handling path that G01 identified as the regression cause.</what-built>
  <how-to-verify>
    1. Cache check: `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py status` — if &lt; 100MB, restore: `prime_usd_cache.py restore known-good`.
    2. Launch Isaac Sim: `DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt`. Wait for "READY".
    3. Send quick_start via MCP (CLAUDE.md "Verified working flow" recipe). Wait for "Quick start complete..." response.
    4. **Visually inspect the viewport** in the AIC Digital Twin window. Compare against the pre-regression state:
       - Task board base sits on the table at expected scale (length ~25cm).
       - SC ports are visually small components on the board (not giant or microscopic).
       - NIC card has the expected aspect ratio + reasonable size relative to board.
       - Any mount rails (if present in the spawn) have the expected length.
       - Robot UR5e + Hand-E gripper are at expected scale.
    5. Compare to G01's worldbox diff table — the prims that previously had off-ratio sizes should now read scale-ratio ~1.0 (re-run `diff_world_bbox.py live-capture 8768 /tmp/g02_post_fix.json` and `diff_world_bbox.py diff tmp/g01_baseline_bboxes.json /tmp/g02_post_fix.json` — table should be empty or have rows ratio 1.0).
    6. Snapshot the now-good cache: `prime_usd_cache.py snapshot`.
  </how-to-verify>
  <resume-signal>
    Type "approved" if viewport renders correctly. If still wrong, paste the new bbox-diff table output and any specific prim that still looks off — Task 1 may need an iteration.
  </resume-signal>
</task>

<task type="auto">
  <name>Task 3: CHANGELOG entry + REQUIREMENTS / VERIFICATION truth status update</name>
  <files>exts/aic-dt/docs/CHANGELOG.md, .planning/REQUIREMENTS.md</files>
  <action>
    1. Append a CHANGELOG entry under "Unreleased" or the Phase 1 milestone section:
       ```
       ## Gap E fix (visual viewport regression)
       - Verdict (per G01): &lt;TOKEN&gt;
       - Fix applied: &lt;1-line summary citing files changed&gt;
       - Verified by: human viewport inspection (G02 Task 2 checkpoint) + diff_world_bbox.py post-fix run
       - References: .planning/phases/01-foundation-parity/01-G01-SUMMARY.md, .planning/phases/01-foundation-parity/01-G02-SUMMARY.md, exts/aic-dt/docs/visual-regression-diagnosis.md
       ```
    2. Update `.planning/REQUIREMENTS.md` to flip PARITY-02 / SCENE-01 / SCENE-04 from `[~]` to `[x]` IF AND ONLY IF the human-verify checkpoint in Task 2 returned "approved" with no remaining viewport issues. If the checkpoint surfaced any partial-fix concern, leave them at `[~]` and add a note citing the open issue.

    Do NOT touch `01-VERIFICATION.md` — the verifier loop owns that. The next verifier run consumes G02-SUMMARY + the requirement status update.
  </action>
  <verify>
    <automated>grep -E "Gap E fix|visual viewport regression" exts/aic-dt/docs/CHANGELOG.md &amp;&amp; grep -E "PARITY-02|SCENE-01|SCENE-04" .planning/REQUIREMENTS.md | head -5</automated>
  </verify>
  <done>
    CHANGELOG entry appended with verdict + fix summary + reference paths. REQUIREMENTS.md status flips reflect the human-verify outcome accurately.
  </done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| Vendored USD edits (D-06) | Edits land on tree owned by this repo post-M1 — no upstream sync risk. |
| Live MCP execute_python_code | Same trust boundary as Phase 1 (already trusted). |

## STRIDE Threat Register (ASVS L1)

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-G02-01 | Tampering | Vendored USDs (LC/SFP/SC Mount, possibly NIC Card etc.) | mitigate | Edits via pxr only; commit each file separately so rollback is per-asset; verify with `usdview` or `diff_world_bbox.py` post-edit. |
| T-G02-02 | DoS | Cache wipe / restore from wrong known-good | mitigate | Always run `prime_usd_cache.py status` before any restore; never `mv DerivedDataCache aside; mkdir DerivedDataCache` per CLAUDE.md operating discipline rule 1. |
| T-G02-03 | Repudiation | Verdict-driven fix | accept | G01 diagnosis + this plan's CHANGELOG entry form the audit trail; verdict token is constrained-vocabulary. |
| T-G02-04 | Elevation | extension.py edits | accept | Same trust boundary as all Phase 1 plans; py_compile gate on every edit. |
</threat_model>

<verification>
After all 3 tasks: human-verify checkpoint passes ("approved"), diff_world_bbox.py post-fix run shows no scale-ratio outliers, CHANGELOG entry committed, REQUIREMENTS.md statuses reflect the gap closure outcome.
</verification>

<success_criteria>
- The fix matches G01's verdict (no speculative cross-verdict edits).
- extension.py + build_mount_rail_usds.py + any edited USDs pass syntax / open cleanly.
- Human confirms viewport renders correctly (Task 2 "approved").
- Post-fix bbox diff against pre-Plan-02 baseline has no scale-ratio outliers > 1% on the previously-regressed prims.
- CHANGELOG entry references G01 verdict + fix summary + linked SUMMARYs.
- REQUIREMENTS.md status flips reflect actual fix scope.
</success_criteria>

<output>
After completion, create `.planning/phases/01-foundation-parity/01-G02-SUMMARY.md` with the verdict applied, the specific code/asset changes, the post-fix bbox-diff outcome, and the human-verify response.
</output>
