---
phase: 01-foundation-parity
plan: G01
type: execute
wave: 1
depends_on: []
files_modified:
  - "exts/aic-dt/docs/visual-regression-diagnosis.md"
  - "exts/aic-dt/scripts/diff_world_bbox.py"
autonomous: true
gap_closure: true
addresses_gap: E
requirements: [PARITY-02, SCENE-01, SCENE-04, TEX-01, TEX-02]
tags: [diagnosis, visual-regression, usd-bbox, metersperunit, gazebo-parity]
must_haves:
  truths:
    - "exts/aic-dt/docs/visual-regression-diagnosis.md exists and names exactly which prims regressed (worldbox shifts > 1mm or scale ratio != 1.0 vs. pre-Plan-02 baseline) and which of the four hypothesized causes (metersPerUnit drift, per-component spawn transforms, GLB→USD scale, AddReference unit drift) is the actual root cause backed by numeric evidence."
  artifacts:
    - path: "exts/aic-dt/docs/visual-regression-diagnosis.md"
      provides: "Numeric per-prim worldbox + scale diff (pre-Plan-02 vs current HEAD), metersPerUnit audit across all loaded layers, root-cause verdict, recommended fix strategy"
      contains: "## Worldbox Diff (mm-precision), ## metersPerUnit Audit, ## Root Cause, ## Recommended Fix"
    - path: "exts/aic-dt/scripts/diff_world_bbox.py"
      provides: "Re-runnable per-prim worldbox capture+diff utility (pxr-based; runs against a saved-stage USD or against a live MCP-attached extension via execute_python_code)"
      exports: ["capture_bbox_for_stage", "diff_bboxes", "main"]
  key_links:
    - from: "diff_world_bbox.py"
      to: "exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd + assets/assets/<Capitalized>/*.usd"
      via: "Usd.Stage.Open + UsdGeom.BBoxCache(GetWorld())"
      pattern: "BBoxCache.*ComputeWorldBound"
---

<objective>
Diagnose Gap E (visual viewport regression — user reported "major mismatch in object size, quite a lot of things to fix"). Numerically compare per-prim worldbox bounds between the pre-Plan-02 known-good state (commit `2be9e0c` — last commit before snake_case `assets/objects/` was retired) and current HEAD. Audit metersPerUnit across every loaded USD layer. Inspect the per-component spawn atom translation/rotation values vs. the prior monolithic add_objects values. Produce a written diagnosis (`exts/aic-dt/docs/visual-regression-diagnosis.md`) with a concrete root-cause verdict and fix recommendation.

Purpose: Plan G02 cannot apply a fix without knowing the cause. Hypothesized causes (metersPerUnit mismatch, per-component spawn atom transform drift, GLB→USD scale baked wrong, AddReference unit drift) overlap and have non-trivial fixes. Guessing wastes a fix-and-rerun cycle.

Output: One diagnosis markdown + one re-runnable bbox-diff script. Gap E remains OPEN after this plan; G02 closes it.
</objective>

<execution_context>
@$HOME/.claude/get-shit-done/workflows/execute-plan.md
@$HOME/.claude/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/ROADMAP.md
@.planning/STATE.md
@.planning/phases/01-foundation-parity/01-VERIFICATION.md
@.planning/phases/01-foundation-parity/01-CONTEXT.md
@.planning/phases/01-foundation-parity/01-09-SUMMARY.md
@.planning/phases/01-foundation-parity/01-02-SUMMARY.md
@CLAUDE.md
@~/.claude/skills/nvidia-suite-docs/SKILL.md
@~/.claude/skills/isaac-sim-extension-dev/SKILL.md
@exts/aic-dt/aic_dt/extension.py
@exts/aic-dt/scripts/build_mount_rail_usds.py

<interfaces>
<!-- Pre-Plan-02 baseline commit: 2be9e0c (postload launcher + early-play quick_start)
     Plan 02 retire commit:        fd34184 (chore(01-02): retire snake_case assets/objects tree (D-05))
     Asset path before Plan 02:    exts/aic-dt/assets/objects/{nic_card, nic_card_mount, sc_plug, sc_port, task_board_base}/*.usd
     Asset path after Plan 02:     exts/aic-dt/assets/assets/<Capitalized With Spaces>/*.usd
     Robot USD (unchanged across both):  exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd (md5 46616697c057701ae2025d44ace26844)
     AIC_OBJECTS dict location:    exts/aic-dt/aic_dt/extension.py:58
     _spawn_component_via_usd:     exts/aic-dt/aic_dt/extension.py:2898 — RemovePrim → AddReference → AddTranslateOp + AddRotateXYZOp
     Plan 09 NEW spawn atoms:      _cmd_spawn_task_board_base/lc_mount_rail/sfp_mount_rail/sc_mount_rail/sc_port/nic_card_mount/nic_card (lines 2936-3068)
     Worldbox capture pattern (pxr): UsdGeom.BBoxCache(time, [UsdGeom.Tokens.default_]).ComputeWorldBound(prim).ComputeAlignedRange()
-->

Worktree pattern (do this for the diff baseline — git checkout in a worktree, never on main):
```bash
git worktree add /tmp/aic-dt-pre-plan02 2be9e0c
ls /tmp/aic-dt-pre-plan02/exts/aic-dt/assets/objects/   # snake_case tree should be present
# When done:  git worktree remove /tmp/aic-dt-pre-plan02
```

Pxr interpreter (the one that has UsdGeom.BBoxCache + glTF SDF plugin loaded):
```bash
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh <script>
```
(Plan 05 / 09 used this — it's the correct one for offline USD inspection.)

Live-mode bbox capture (preferred for ground-truth — uses execute_python_code over MCP, runs inside Kit so the glTF SDF plugin is loaded):
```python
import socket, json
s = socket.socket(); s.connect(("127.0.0.1", 8768)); s.settimeout(120)
code = '''
from pxr import Usd, UsdGeom
import omni.usd
stage = omni.usd.get_context().get_stage()
cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])
out = {}
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if any(s in path for s in ("/World/TaskBoard", "/World/Objects", "/World/UR5e", "/World/cable")):
        try:
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
            if r.IsEmpty(): continue
            out[path] = [list(mn), list(mx)]
        except Exception:
            pass
result = out
'''
s.sendall(json.dumps({"type":"execute_python_code","params":{"code":code}}).encode())
# read until parses
```
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: Author diff_world_bbox.py — pxr-based bounding-box capture and diff utility</name>
  <files>exts/aic-dt/scripts/diff_world_bbox.py</files>
  <action>
    Create exts/aic-dt/scripts/diff_world_bbox.py — a single-file pxr utility with three modes:
    1. `capture <stage_usd_path> <out_json>` — open a USD stage offline, traverse all prims under /World, compute per-prim worldbox via UsdGeom.BBoxCache.ComputeWorldBound().ComputeAlignedRange(), save to JSON {prim_path: [min_xyz, max_xyz, size_xyz, center_xyz]}. Skip empty bounds. metersPerUnit + upAxis go in a top-level "_stage_meta" key.
    2. `live-capture <port> <out_json>` — connect to MCP port (default 8768), send execute_python_code that traverses the LIVE stage (`omni.usd.get_context().get_stage()`) and returns the same shape via `result = out`. Use the framing pattern from CLAUDE.md (single JSON object, recv loop until json.loads succeeds).
    3. `diff <baseline_json> <current_json>` — load two JSON files, print a markdown table of prims with size or center delta > 1mm in any axis OR scale-ratio (size_current/size_baseline) outside [0.99, 1.01]. Sort descending by max-axis size delta. Also print metersPerUnit diff if present.

    Reference attribution at top of file: "# Reference: gap-closure G01 visual regression diagnostic / Phase 1 Foundation Parity"

    Use the isaac-sim-4.2.0 python interpreter (`~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh`) for offline mode (`capture` subcommand). The script's shebang is `#!/usr/bin/env python3` but invocation docstring at top documents which interpreter has pxr.

    Per CLAUDE.md canonical references rule: this task touches pxr USD APIs, so consult `~/.claude/skills/nvidia-suite-docs/SKILL.md` (Pxr OpenUSD bbox patterns) and `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` (project MCP socket framing) before authoring.
  </action>
  <verify>
    <automated>python3 -m py_compile exts/aic-dt/scripts/diff_world_bbox.py &amp;&amp; ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/diff_world_bbox.py capture exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd /tmp/g01_robot_baseline.json &amp;&amp; python3 -c "import json; d=json.load(open('/tmp/g01_robot_baseline.json')); assert '_stage_meta' in d, '_stage_meta key missing'; assert any('aic_unified_robot' in k for k in d if k != '_stage_meta'), 'no robot prims captured'; assert 'metersPerUnit' in d['_stage_meta']; print('OK robot prims captured:', len([k for k in d if k != '_stage_meta']))"</automated>
  </verify>
  <done>
    File compiles, exits cleanly with --help, `capture` subcommand produces JSON containing >=1 prim with non-empty bbox + a `_stage_meta` key with metersPerUnit and upAxis, `diff` subcommand emits a markdown table with at minimum 4 columns (prim_path, baseline_size_xyz, current_size_xyz, scale_ratio).
  </done>
</task>

<task type="auto">
  <name>Task 2: Capture pre-Plan-02 baseline + current-HEAD live bboxes; audit metersPerUnit; spawn-atom transform diff</name>
  <files>tmp/g01_baseline_bboxes.json, tmp/g01_current_bboxes.json, tmp/g01_metersperunit_audit.txt, tmp/g01_spawn_atom_param_diff.txt</files>
  <action>
    Capture three independent diagnostic data sources, all in /tmp (these are NOT committed — only the diagnosis markdown is):

    **1. Pre-Plan-02 baseline bboxes (commit 2be9e0c):**
    - `git worktree add /tmp/aic-dt-pre-plan02 2be9e0c`
    - For each snake_case USD in `/tmp/aic-dt-pre-plan02/exts/aic-dt/assets/objects/{nic_card,nic_card_mount,sc_plug,sc_port,task_board_base}/*.usd`, run `diff_world_bbox.py capture` to extract per-prim bboxes + metersPerUnit. Aggregate into `tmp/g01_baseline_bboxes.json` keyed by relative-asset-path.
    - Note: the baseline is the ASSET-LEVEL bbox (each USD on its own), because we don't have a saved stage from that era. This is OK because the regression hypothesis is per-asset scale/units.
    - Cleanup: `git worktree remove /tmp/aic-dt-pre-plan02` after capture.

    **2. Current-HEAD bboxes (live-mode):**
    - PRECONDITION: Isaac Sim must be running with quick_start completed. If port 8768 is not responsive, emit a clear note in the diagnosis markdown ("Live capture skipped — Isaac Sim not running. Falling back to offline asset-USD captures.") and capture the offline-USD versions instead from `exts/aic-dt/assets/assets/<Capitalized>/*.usd`.
    - Live mode: `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/diff_world_bbox.py live-capture 8768 tmp/g01_current_bboxes.json`
    - Fallback offline mode: capture the current vendored capitalized USDs the same way as baseline.

    **3. metersPerUnit audit (`tmp/g01_metersperunit_audit.txt`):**
    Use a one-liner pxr script: open every USD under `exts/aic-dt/assets/` and `exts/aic-dt/assets/robot/` and `exts/aic-dt/assets/scene/` and print `<path> upAxis=<axis> metersPerUnit=<float>`. Look for any layer with metersPerUnit != 1.0 (or != the robot USD's value — whichever is the project convention). Save to txt.

    **4. Spawn-atom transform diff (`tmp/g01_spawn_atom_param_diff.txt`):**
    - Old monolithic add_objects logic: read `git show 2be9e0c:exts/aic-dt/aic_dt/extension.py` and grep around `def add_objects` (around its old line) — extract the literal translation/rotation values for each AIC_OBJECTS entry as they were applied.
    - New per-component atoms: read current extension.py lines 2936-3068 (`_cmd_spawn_*` methods) — extract default x/y/z/roll/pitch/yaw values + how they're applied through `_spawn_component_via_usd`.
    - Side-by-side comparison written to txt: per-object (task_board_base, sc_port_1, sc_port_2, nic_card) — old translation, new default translation, delta. If any delta is non-trivial that's a candidate cause.

    Write all four diagnostic files into /tmp (NOT under .planning/ or exts/). They are working data; only Task 3's diagnosis markdown ships.

    Per CLAUDE.md canonical references rule: pxr/USD API surface — consult nvidia-suite-docs SKILL.md if BBoxCache produces empty ranges or unexpected unit handling.
  </action>
  <verify>
    <automated>test -s tmp/g01_baseline_bboxes.json &amp;&amp; test -s tmp/g01_current_bboxes.json &amp;&amp; test -s tmp/g01_metersperunit_audit.txt &amp;&amp; test -s tmp/g01_spawn_atom_param_diff.txt &amp;&amp; python3 -c "import json; b=json.load(open('tmp/g01_baseline_bboxes.json')); c=json.load(open('tmp/g01_current_bboxes.json')); assert len(b)&gt;1 and len(c)&gt;1, 'too few entries'; print(f'baseline {len(b)} prims, current {len(c)} prims')"</automated>
  </verify>
  <done>
    Four diagnostic files exist in tmp/ with non-zero size. Both bbox JSONs deserialize and contain >=1 prim each. Working data is captured for Task 3 to analyze. The offline-fallback path is documented in the diagnosis if Isaac Sim was not running.
  </done>
</task>

<task type="auto">
  <name>Task 3: Author exts/aic-dt/docs/visual-regression-diagnosis.md with numeric verdict + recommended fix</name>
  <files>exts/aic-dt/docs/visual-regression-diagnosis.md</files>
  <action>
    Author `exts/aic-dt/docs/visual-regression-diagnosis.md` synthesizing all four diagnostic data files from Task 2. Required sections (in this order):

    ## Summary (1 paragraph)
    - Restate the user-reported regression. State which of the four hypothesized causes the data confirms (or rejects).

    ## Worldbox Diff (mm-precision)
    - Run `diff_world_bbox.py diff tmp/g01_baseline_bboxes.json tmp/g01_current_bboxes.json` and embed the markdown table verbatim.
    - For each row with scale-ratio != 1.0 ± 0.01: name the prim path, the baseline size, the current size, the ratio, and call out whether this matches a hypothesis.

    ## metersPerUnit Audit
    - Embed the `tmp/g01_metersperunit_audit.txt` content as a fenced block.
    - Call out any layer whose metersPerUnit differs from the robot USD's. If all are 1.0, state explicitly "metersPerUnit IS NOT the cause."

    ## Spawn-Atom Transform Comparison
    - Embed the `tmp/g01_spawn_atom_param_diff.txt` analysis. State explicitly whether new-default translations match old-monolithic translations within numerical tolerance, or whether they drifted.

    ## Root Cause Verdict
    Choose ONE token and write it as a bold first line:
    - **VERDICT: METERSPERUNIT_DRIFT** (a layer's metersPerUnit changed across the vendor swap)
    - **VERDICT: SPAWN_ATOM_TRANSFORM_DRIFT** (per-component atom defaults differ from monolithic add_objects values)
    - **VERDICT: GLB_USD_SCALE_BAKED_WRONG** (mount-rail thin USDs have an implicit scale != 1.0 because the .glb's metersPerUnit conflicts with the wrapping USD's)
    - **VERDICT: ADD_REFERENCE_UNIT_DRIFT** (AddReference of a vendored USD without explicit xformOp:scale picks up unit drift via reference-time unit reconciliation)
    - **VERDICT: COMPOUND** (>1 of the above; list which)
    - **VERDICT: INCONCLUSIVE — INSUFFICIENT_DATA** (live capture skipped, fallback insufficient; Plan G02 must be a re-run-after-restart plan)

    Each verdict statement must cite specific evidence rows from the worldbox diff or metersPerUnit audit (e.g., "table row `nic_card` shows scale-ratio 0.01x → expected 1.0x; AIC NIC Card USD has metersPerUnit=0.01 vs unified robot USD metersPerUnit=1.0").

    ## Recommended Fix Strategy (for Plan G02)
    Given the verdict, recommend a SPECIFIC fix:
    - For METERSPERUNIT_DRIFT: which layers need re-authoring or which AddReference call needs explicit xformOp:scale.
    - For SPAWN_ATOM_TRANSFORM_DRIFT: which atom default values need to be updated to match old AIC_OBJECTS values.
    - For GLB_USD_SCALE_BAKED_WRONG: re-author the mount-rail thin USDs with explicit scale via `build_mount_rail_usds.py` v2.
    - For ADD_REFERENCE_UNIT_DRIFT: add explicit `UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(1,1,1))` or equivalent in `_spawn_component_via_usd`.
    - For COMPOUND: an ordered fix list.
    - For INCONCLUSIVE: a specific re-capture protocol Plan G02 must run first.

    The recommendation MUST be specific enough that Plan G02 can implement it without further investigation. No "consider X" language; this is the contract for the fix plan.

    Author attribution comment at top: "<!-- Reference: gap-closure G01 — Visual viewport regression diagnosis (Gap E from 01-VERIFICATION.md). Generated 2026-05-XX from tmp/g01_*.{json,txt}. -->"

    Per CLAUDE.md canonical references rule: include a "References" section at the bottom citing both `nvidia-suite-docs/SKILL.md` and `isaac-sim-extension-dev/SKILL.md` (the latter for the cache discipline / glTF SDF plugin notes that influence whether the .glb-referenced USDs scale correctly).
  </action>
  <verify>
    <automated>test -f exts/aic-dt/docs/visual-regression-diagnosis.md &amp;&amp; grep -E '^## Summary|^## Worldbox Diff|^## metersPerUnit Audit|^## Spawn-Atom Transform|^## Root Cause Verdict|^## Recommended Fix Strategy' exts/aic-dt/docs/visual-regression-diagnosis.md | wc -l | grep -q '^6$' &amp;&amp; grep -E '^\*\*VERDICT: (METERSPERUNIT_DRIFT|SPAWN_ATOM_TRANSFORM_DRIFT|GLB_USD_SCALE_BAKED_WRONG|ADD_REFERENCE_UNIT_DRIFT|COMPOUND|INCONCLUSIVE)' exts/aic-dt/docs/visual-regression-diagnosis.md</automated>
  </verify>
  <done>
    Diagnosis markdown exists with all 6 required sections. Verdict line is exactly one of the 6 allowed tokens. Recommended Fix Strategy section is specific (mentions a file path, function name, or numeric value). Plan G02 has an unambiguous starting point.
  </done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| Local pxr USD inspection → /tmp JSON | No external input; reads vendored USDs and live MCP execute_python_code response (already trusted in Phase 1) |
| Git worktree of historical commit | Read-only; cleaned up after capture; no merge/checkout on main |

## STRIDE Threat Register (ASVS L1)

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-G01-01 | Tampering | exts/aic-dt/scripts/diff_world_bbox.py | mitigate | New script — bbox capture only reads stages, writes to caller-specified out path; no file mutation in capture path. |
| T-G01-02 | Information disclosure | tmp/g01_*.json (asset bbox data) | accept | Asset geometry is public (already vendored under exts/aic-dt/assets/); not sensitive. |
| T-G01-03 | DoS | git worktree of 2be9e0c | mitigate | Worktree explicitly removed in Task 2 cleanup; not on main; idempotent. |
| T-G01-04 | Repudiation | diagnosis verdict | accept | Markdown is committed with reference attribution; verdict tokens are constrained-vocabulary so reviewer can challenge. |
</threat_model>

<verification>
After all 3 tasks: `exts/aic-dt/docs/visual-regression-diagnosis.md` is the single deliverable. The verdict line determines G02's scope. Tasks 1+2's data files in /tmp are working data and intentionally not committed.
</verification>

<success_criteria>
- diff_world_bbox.py exists, py_compile clean, has capture + live-capture + diff subcommands.
- tmp/g01_baseline_bboxes.json + tmp/g01_current_bboxes.json + tmp/g01_metersperunit_audit.txt + tmp/g01_spawn_atom_param_diff.txt all non-empty.
- visual-regression-diagnosis.md has all 6 required sections; verdict line is one of the 6 constrained tokens; recommended fix is specific (cites file/function/numeric value).
- Plan G02 can read this diagnosis and know exactly what to change.
</success_criteria>

<output>
After completion, create `.planning/phases/01-foundation-parity/01-G01-SUMMARY.md` with the verdict, the top 3 worldbox-diff rows, and a 1-line statement of what G02 must do.
</output>
