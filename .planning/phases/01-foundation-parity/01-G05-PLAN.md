---
phase: 01-foundation-parity
plan: G05
type: execute
wave: 3
depends_on: [G02]
files_modified:
  - "exts/aic-dt/scripts/verify_phase_1.sh"
  - "exts/aic-dt/scripts/sweep_textures.py"
  - "exts/aic-dt/docs/texture-sweep.md"
  - "exts/aic-dt/aic_dt/extension.py"
  - "exts/aic-dt/docs/CHANGELOG.md"
autonomous: false
gap_closure: true
addresses_gap: C
requirements: [TEX-01, TEX-02, TEX-03]
tags: [texture-sweep, verify-harness, mdl, asset-binding]
must_haves:
  truths:
    - "Loading the full M1 scene in Isaac Sim's viewport shows zero pink/black/missing-texture materials and zero broken-MDL warnings in the console; exts/aic-dt/docs/texture-sweep.md contains a findings + fix log enumerating every asset touched"
  artifacts:
    - path: "exts/aic-dt/scripts/verify_phase_1.sh"
      provides: "Path-with-spaces hot-fix in Step 8 (Kit log path 'Isaac-Sim Full' must be quoted)"
      contains: "Isaac-Sim Full"
    - path: "exts/aic-dt/scripts/sweep_textures.py"
      provides: "Filter for benign Info-level lines (omni.kit.app extension registration noise) so 921-hit raw output reduces to actionable rows only"
      contains: "PATTERNS"
    - path: "exts/aic-dt/docs/texture-sweep.md"
      provides: "A clean post-fix sweep section with each remaining row classified (resolved | accepted-cosmetic) and a verdict line"
      contains: "Sweep run"
    - path: "exts/aic-dt/aic_dt/extension.py"
      provides: "Camera-prim-not-found warnings (3 hits) fixed by correcting the wrist-camera setup_wrist_cameras prim paths to match the unified USD's actual camera prim locations, OR documenting them as accepted-cosmetic with a clear rationale"
      contains: "setup_wrist_cameras"
  key_links:
    - from: "verify_phase_1.sh Step 8"
      to: "Kit log under ~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/"
      via: "properly-quoted path expansion"
      pattern: "Isaac-Sim Full"
---

<objective>
Close Gap C (texture sweep — texture-sweep.md scaffold + 921 raw hits in current sweep run, with most being noise-level Info entries from omni.kit registration; the actionable surface is ~13 real MDL/texture/fallback hits + 3 camera-prim-not-found warnings + 4 Body-prim-not-found warnings + ~30+ ISO_4762/V1015120 cosmetic sub-reference failures inside sc_port_visual.usd; plus one path-with-spaces bug in verify_phase_1.sh Step 8 that splits the Kit log path).

Purpose: Phase 1 SC #4 currently fails because (a) the sweep log isn't classified into actionable vs cosmetic, and (b) the 8 camera-prim-not-found warnings indicate a real bug in `setup_wrist_cameras` (the prim paths it references don't match the unified USD's actual camera prim locations).

Output: A clean texture-sweep.md with each remaining hit classified (resolved | accepted-cosmetic), the camera-prim path bug fixed in extension.py, and verify_phase_1.sh Step 8 path-with-spaces bug fixed.

NOTE: This plan is downstream of G02 (Gap E fix). If G01's verdict was METERSPERUNIT_DRIFT, the texture binding may have been a symptom — re-run sweep AFTER G02 lands and most hits may evaporate. Hence wave 3, not wave 2.
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
@.planning/phases/01-foundation-parity/01-07-SUMMARY.md
@.planning/phases/01-foundation-parity/usd_prim_inventory.txt
@exts/aic-dt/scripts/verify_phase_1.sh
@exts/aic-dt/scripts/sweep_textures.py
@exts/aic-dt/docs/texture-sweep.md
@exts/aic-dt/aic_dt/extension.py
@CLAUDE.md
@~/.claude/skills/nvidia-suite-docs/SKILL.md
@~/.claude/skills/isaac-sim-extension-dev/SKILL.md

<interfaces>
<!-- Categorized hit-set from current sweep (texture-sweep.md ## Sweep run 2026-05-02T07:11:54, 921 raw hits):

NOISE — Info-level extension registration (filter out, NOT real failures):
  - <unknown>  texture  "registered (path: ...isaacsim/extscache/omni.kit.hydra_..."
  - <unknown>  MDL      "registered (path: ...isaacsim/extscache/omni.kit.stage.mdl_converter..."
  - <unknown>  fallback "Ignoring embedded ui.scenes fallbacks for omni.kit.viewport.legacy_gizmos"
  - <unknown>  not\s*found "viewportHandle not found for bucket id b_"
  These are at startup, before scene load, and are not asset failures. The sweep regex needs to filter [Info] level lines and lines containing "registered" or "Ignoring embedded".

ACTIONABLE 1 — camera-prim-not-found (3 hits, real extension.py bug):
  /World/UR5e/center_camera_optical/center_camera, "Camera prim not found ... skipping center_camera"
  /World/UR5e/left_camera_optical/left_camera,    "Camera prim not found ... skipping left_camera"
  /World/UR5e/right_camera_optical/right_camera,  "Camera prim not found ... skipping right_camera"
  These come from setup_wrist_cameras. The path it tries to bind cameras to is `/World/UR5e/{name}_optical/{name}` but the unified USD has these prims at `/World/aic_unified_robot/{name}_optical` (no UR5e parent, no nested duplicate camera link). Fix: update setup_wrist_cameras prim paths.

ACTIONABLE 2 — Body-prim-not-found (4 hits, related but pre-Plan-09 legacy code):
  /World/Objects/task_board_base/task_board_base
  /World/Objects/sc_port_1/sc_port_1
  /World/Objects/sc_port_2/sc_port_2
  /World/Objects/nic_card/nic_card
  These come from the legacy add_objects path that nests prim_name/prim_name. Plan 09 added the new clubbing path; this is the pre-existing legacy /World/Objects path. Likely benign (cosmetic; doubled-name lookup). Decide accept-cosmetic vs fix.

COSMETIC — sc_port_visual.usd sub-reference failures (~30+ rows, ISO_4762 + V1015120 screws):
  /World/TaskBoard/SCPort_*/ISO_4762___M2_x_8_*  Could not open
  /World/TaskBoard/SCPort_*/V1015120_*           Could not open
  /World/Objects/sc_port_*/ISO_4762___M2_x_8_*   Could not open  (legacy path)
  These are screw cosmetic sub-references the parent SC Port USD points to but doesn't ship. Per Plan 02 vendor manifest, these were intentionally excluded as cosmetic. CLASSIFY accepted-cosmetic with rationale.

VERIFY_PHASE_1.SH STEP 8 BUG (tmp/verify_phase_1_run.log line 61):
  grep: /home/aaugus11/.nvidia-omniverse/logs/Kit/Isaac-Sim:: No such file or directory
  The path "Isaac-Sim Full" with the space split into two args. Fix: properly quote the path inside Step 8's grep/find/sweep_textures.py invocation.
-->

setup_wrist_cameras location: extension.py — search for `setup_wrist_cameras` definition; the offending prim path strings are likely in `_create_camera_actiongraph` or similar (around line 1066 per CONTEXT.md).

Real camera prim locations (from usd_prim_inventory.txt):
  /World/aic_unified_robot/center_camera_camera_link
  /World/aic_unified_robot/center_camera_optical
  /World/aic_unified_robot/center_camera_sensor_link
  (similar for left_camera, right_camera)
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: verify_phase_1.sh Step 8 path-with-spaces hot-fix + sweep_textures.py noise filter</name>
  <files>exts/aic-dt/scripts/verify_phase_1.sh, exts/aic-dt/scripts/sweep_textures.py</files>
  <action>
    **Part A — verify_phase_1.sh Step 8 fix:**
    Locate Step 8 in `exts/aic-dt/scripts/verify_phase_1.sh`. Find any unquoted shell expansion of `~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/` or any `grep` / `find` / subshell call that splits on the space. Wrap in proper quoting (`"...Isaac-Sim Full..."`). Re-test with `bash -n exts/aic-dt/scripts/verify_phase_1.sh` (syntax check) and ensure no other sites have the same bug.

    The verifier log (tmp/verify_phase_1_run.log line 61) shows: `grep: /home/aaugus11/.nvidia-omniverse/logs/Kit/Isaac-Sim:: No such file or directory`. The trailing `:` suggests the path was concatenated with another arg unquoted. Trace and fix.

    **Part B — sweep_textures.py noise filter:**
    Add a noise-line filter to `exts/aic-dt/scripts/sweep_textures.py`:
    - Skip lines matching `[Info] [omni.ext.plugin] [ext: ...] registered (path:` (extension-registration noise).
    - Skip lines matching `Ignoring embedded ui.scenes fallbacks` (legacy-gizmos noise).
    - Skip lines matching `viewportHandle not found for bucket id` (Fabric warning noise).

    Add the filter as an EXCLUDE_PATTERNS list near the existing PATTERNS list. Document each exclusion with an inline comment citing why (with a sample line for traceability).

    Per CLAUDE.md canonical references rule: cite `isaac-sim-extension-dev/SKILL.md` (project's verify-harness conventions) in inline doc.
  </action>
  <verify>
    <automated>bash -n exts/aic-dt/scripts/verify_phase_1.sh &amp;&amp; python3 -m py_compile exts/aic-dt/scripts/sweep_textures.py &amp;&amp; grep -E '"Isaac-Sim Full"' exts/aic-dt/scripts/verify_phase_1.sh &amp;&amp; grep -E "EXCLUDE_PATTERNS|registered \(path:|viewportHandle" exts/aic-dt/scripts/sweep_textures.py</automated>
  </verify>
  <done>
    verify_phase_1.sh path-with-spaces bug fixed (proper quoting). sweep_textures.py has EXCLUDE_PATTERNS that suppress the 4 known noise categories. Both scripts compile / pass syntax check.
  </done>
</task>

<task type="auto">
  <name>Task 2: Fix setup_wrist_cameras prim paths to match unified USD</name>
  <files>exts/aic-dt/aic_dt/extension.py</files>
  <action>
    Locate `setup_wrist_cameras` (and `_create_camera_actiongraph` if separate) in `exts/aic-dt/aic_dt/extension.py`. Find the literal prim path strings the camera setup tries to bind to. Per the sweep log, the existing code references:
    - `/World/UR5e/center_camera_optical/center_camera`
    - `/World/UR5e/left_camera_optical/left_camera`
    - `/World/UR5e/right_camera_optical/right_camera`

    Per `usd_prim_inventory.txt`, the actual camera prims in the unified USD are:
    - `/World/aic_unified_robot/center_camera_camera_link`
    - `/World/aic_unified_robot/center_camera_optical`
    - `/World/aic_unified_robot/center_camera_sensor_link`
    - (analogous for left_camera, right_camera)

    NOTE: The robot is loaded under `/World/UR5e` per Plan 04's prim path convention (`self._articulation_root_prim_path = "/World/UR5e/aic_unified_robot"`) — so the actual runtime location is `/World/UR5e/aic_unified_robot/{name}_camera_link` etc. The setup_wrist_cameras code likely has the wrong nested path.

    Decision before edit: confirm the runtime prim path by reading either (a) the extension's `_articulation_root_prim_path` literal, or (b) running a quick MCP `execute_python_code` snippet to traverse `/World/UR5e/aic_unified_robot/*camera*` and print the actual prim paths.

    Once confirmed, update the camera-binding prim path strings in setup_wrist_cameras to use the right path. The leaf prim (the camera itself) is named `{side}_camera_optical` or `{side}_camera_camera_link` (NOT a nested doubled-up `{name}_optical/{name}` segment).

    Per CLAUDE.md canonical references rule: this touches the camera OmniGraph subgraph + USD prim binding — consult `nvidia-suite-docs/SKILL.md` (Camera prim conventions in Isaac Sim 5.0) and `isaac-sim-extension-dev/SKILL.md` (project's existing camera setup pattern) before editing.

    Add inline marker: `# Gap C (G05): camera prim path corrected from /World/UR5e/{name}_optical/{name} to /World/UR5e/aic_unified_robot/{name}_optical`.
  </action>
  <verify>
    <automated>python3 -m py_compile exts/aic-dt/aic_dt/extension.py &amp;&amp; grep -E "Gap C \(G05\)|aic_unified_robot/(center|left|right)_camera" exts/aic-dt/aic_dt/extension.py | wc -l | awk '{if ($1 &lt; 2) {print "FAIL: too few camera-path corrections (" $1 ")"; exit 1} else print "OK: " $1 " hits"}'</automated>
  </verify>
  <done>
    setup_wrist_cameras prim paths corrected to match the unified USD's actual camera prim locations. extension.py compiles. Inline marker comment present. The legacy `/World/UR5e/{name}_optical/{name}` (nested-doubled) pattern is gone.
  </done>
</task>

<task type="checkpoint:human-verify" gate="blocking">
  <name>Task 3: Re-run sweep + populate texture-sweep.md verdict section</name>
  <what-built>verify_phase_1.sh Step 8 fix + sweep_textures.py noise filter + setup_wrist_cameras prim path fix.</what-built>
  <how-to-verify>
    1. Cache check + restore if needed.
    2. Launch Isaac Sim + quick_start.
    3. Re-run sweep: `python3 exts/aic-dt/scripts/sweep_textures.py --port 8768`. Observe row count drops dramatically (from 921 → expected &lt; 50 actionable rows).
    4. Re-run verify_phase_1.sh Step 8 in isolation: extract the Step 8 block from verify_phase_1.sh and run it standalone — confirm the path-with-spaces error is gone and the grep targets the correct Kit log file.
    5. Re-run `ros2 topic list` and verify the wrist camera topics now publish (Gap C also exercises the camera setup) — `ros2 topic info /center_camera/image` etc. should show a publisher.
    6. Open `exts/aic-dt/docs/texture-sweep.md` — the new "## Sweep run &lt;timestamp&gt;" appended by step 3 should have far fewer rows. The remaining rows are the actionable surface for Task 4.
    7. Snapshot cache.
  </how-to-verify>
  <resume-signal>
    Type "approved" with the new sweep row count (e.g. "approved — 12 rows remaining"). If row count is still high, paste a sample of unfiltered rows — Task 1's EXCLUDE_PATTERNS may need broadening.
  </resume-signal>
</task>

<task type="auto">
  <name>Task 4: Classify remaining sweep rows + author texture-sweep.md verdict section</name>
  <files>exts/aic-dt/docs/texture-sweep.md</files>
  <action>
    Read the latest "## Sweep run &lt;timestamp&gt;" section appended by Task 3 to `exts/aic-dt/docs/texture-sweep.md`. For each remaining row:

    1. Mark `Status` column as one of:
       - `resolved` — the camera-prim path fix from Task 2 closed it; this row should NOT appear in the next sweep.
       - `accepted-cosmetic` — known-safe (e.g. ISO_4762/V1015120 screw sub-references that ship without USDs by design per Plan 02 manifest).
       - `unresolved` — remaining real issue requiring follow-up.
    2. For each `accepted-cosmetic` row, add a 1-line rationale in a new "Notes" column or as a footnote (e.g. "ISO_4762___M2_x_8 series — cosmetic screws excluded from vendor per Plan 02 manifest; rendering unaffected.").
    3. Add a final "## Verdict" section at the end of the file:
       ```
       ## Verdict (G05 closure)

       Total rows in latest sweep: N
       resolved: X (camera-prim-path fix from G05 Task 2)
       accepted-cosmetic: Y (ISO_4762/V1015120 screw sub-references, omni.kit registration noise)
       unresolved: Z (&lt;list each, or "none"&gt;)

       TEX-01/TEX-02 status: PASS if Z == 0, otherwise PARTIAL (list unresolved).
       ```

    4. Append CHANGELOG entry:
       ```
       ## Gap C fix (texture sweep — TEX-01/TEX-02/TEX-03)
       - verify_phase_1.sh Step 8 path-with-spaces bug fixed
       - sweep_textures.py noise filter added (4 EXCLUDE_PATTERNS)
       - setup_wrist_cameras prim paths corrected to match unified USD
       - Final sweep: &lt;total&gt; rows, &lt;Y&gt; accepted-cosmetic, &lt;Z&gt; unresolved
       - References: .planning/phases/01-foundation-parity/01-G05-SUMMARY.md
       ```

    5. Update `.planning/REQUIREMENTS.md` TEX-01 and TEX-02 to `[x]` if Verdict shows zero unresolved rows; otherwise `[~]` with note.

    Per CLAUDE.md canonical references rule: cite `isaac-sim-extension-dev/SKILL.md` and `nvidia-suite-docs/SKILL.md` in CHANGELOG entry.
  </action>
  <verify>
    <automated>grep -E "^## Verdict \(G05 closure\)" exts/aic-dt/docs/texture-sweep.md &amp;&amp; grep -E "Gap C fix" exts/aic-dt/docs/CHANGELOG.md &amp;&amp; grep -E "TEX-01|TEX-02|TEX-03" .planning/REQUIREMENTS.md | head -3</automated>
  </verify>
  <done>
    texture-sweep.md has a Verdict (G05 closure) section. CHANGELOG entry appended. REQUIREMENTS.md TEX-01/02/03 statuses reflect the final sweep outcome. Each remaining sweep row is classified.
  </done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| verify_phase_1.sh shell-quoting | Local file-system path expansion — fix prevents path-with-spaces split from masking real errors. |
| sweep_textures.py log parser | Reads Kit log; classification is read-only. |
| setup_wrist_cameras prim path | Same trust as Phase 1 baseline. |

## STRIDE Threat Register (ASVS L1)

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-G05-01 | Tampering | verify_phase_1.sh quoting fix | mitigate | bash -n syntax check; spot-test by running Step 8 in isolation. |
| T-G05-02 | Information disclosure | sweep_textures.py classifies rows | accept | Classification labels are constrained-vocabulary; no PII. |
| T-G05-03 | Tampering | setup_wrist_cameras prim path edit | mitigate | py_compile gate; runtime check via `ros2 topic info /center_camera/image` post-fix. |
| T-G05-04 | DoS | EXCLUDE_PATTERNS over-filter masks real failures | mitigate | EXCLUDE_PATTERNS are anchored to Info-level + specific known-noise tokens; Verdict section explicitly counts excluded rows; reviewable. |
</threat_model>

<verification>
After all 4 tasks: sweep run row count drops from 921 to <50 actionable; each remaining row classified resolved/accepted-cosmetic/unresolved; Verdict section authored; verify_phase_1.sh Step 8 grep targets the right log file; wrist camera topics publish post-fix.
</verification>

<success_criteria>
- verify_phase_1.sh path-with-spaces bug fixed (bash -n clean, Step 8 grep targets correct Kit log).
- sweep_textures.py noise filter suppresses 4 known noise categories.
- setup_wrist_cameras prim paths corrected; camera topics publish.
- texture-sweep.md has Verdict section with row classification.
- CHANGELOG + REQUIREMENTS.md updated.
- Human-verify Task 3 returns "approved".
</success_criteria>

<output>
After completion, create `.planning/phases/01-foundation-parity/01-G05-SUMMARY.md` with the pre-fix vs post-fix sweep row counts, the camera-prim path correction details, and the final Verdict.
</output>
