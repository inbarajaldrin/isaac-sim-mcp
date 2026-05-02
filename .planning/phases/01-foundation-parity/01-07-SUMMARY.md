---
phase: 01-foundation-parity
plan: 07
subsystem: testing
tags: [verify, sweep, tf-diff, harness, validation, mcp, kit-log, bash, python]

# Dependency graph
requires:
  - phase: 01-foundation-parity / 01-01
    provides: aic_frames_live.gv reference TF tree (31 frames / 30 edges) for diff_tf_tree.py input
  - phase: 01-foundation-parity / 01-04
    provides: parity_05_wrench_framing.txt audit-trail (frame_id=ati/tool_link) for verify Step 5a
  - phase: 01-foundation-parity / 01-05
    provides: joint_ordering_probe.txt Verdict/Action lines for verify Step 10
  - phase: 01-foundation-parity / 01-06
    provides: setup_tf_publisher + setup_joint_state_publisher MCP atoms for verify Step 4
provides:
  - diff_tf_tree.py — regex-only .gv diff utility (frame-set + edge-set, exit 0 on match)
  - sweep_textures.py — D-07 + augmented Isaac-Sim asset-failure pattern grep, append-only Markdown
  - verify_phase_1.sh — 10-step hybrid-runtime gate harness (port-detect + cold-launch + MCP atoms + topic check + TF diff + sweep)
  - texture-sweep.md — TEX-03 fix log scaffold with row format + augmented patterns documented
affects: [01-08 quick_start refactor, 01-09 final verify, future phases TEX-fix iterations]

# Tech tracking
tech-stack:
  added: []  # Pure Python stdlib + bash; no new deps
  patterns:
    - "Hybrid runtime harness — port-detect liveness probe with cold-launch fallback"
    - "Append-only Markdown audit doc — sweep runs prepend `## Sweep run <ISO>` sections preserving history"
    - "Regex-only .gv parsing — no pygraphviz / networkx; tf2_tools format is stable enough"
    - "Soft-fail with remediation hint — STRATEGY-FALLBACK-NEEDED message when TF diff fails on SUBLAYER-RENAME strategy"

key-files:
  created:
    - exts/aic-dt/scripts/diff_tf_tree.py
    - exts/aic-dt/scripts/sweep_textures.py
    - exts/aic-dt/scripts/verify_phase_1.sh
    - exts/aic-dt/docs/texture-sweep.md
  modified: []

key-decisions:
  - "diff_tf_tree.py is regex-only (RESEARCH.md reference impl) — no pygraphviz dep; .gv format is stable across tf2_tools versions per RESEARCH 'Don't Hand-Roll' table"
  - "sweep_textures.py PATTERNS = D-07 baseline + augmented Isaac-Sim asset-failure surface (Failed/Could not open / unresolved / reference.*invalid). Smoke-test on existing Kit log proves the augmented patterns catch sc_port_visual.usd's known broken sub-references (ISO_4762 / V1015120) — not a future iteration, baked in from day 1."
  - "Sweep output is append-only — each run prepends `## Sweep run <ISO>` section. Preserves audit history across iterations (D-07 is iterative per CONTEXT.md). Never clobber prior sweeps."
  - "verify_phase_1.sh is the truth-telling gate — PARITY-03 (gripper/left_finger_joint slash) and PARITY-04 (slashed TF frames) are EXPECTED to FAIL until Plan 06's deferrals are wired. The script exits non-zero with clear remediation message; it does not paper over the deferral."
  - "Step 5a (PARITY-05) consumes parity_05_wrench_framing.txt directly — frame_id and Type are extracted via grep from the audit file rather than hard-coded in the script. Single source of truth: the audit file Plan 04 wrote."
  - "Cold-launch path uses `launch_postload.py` per CLAUDE.md, not the lifecycle helper. The lifecycle path wedges on cold caches; postload boots Kit first then enables aic-dt. Step 0 also restores `known-good` cache backup if size <100MB before any launch attempt."
  - "Joint-ordering Step 10 branches on Action: line — for current `NO-WRAPPER-NEEDED` verdict, soft-checks /joint_states topic literal; for hypothetical `ADD-TASK-4-WRAPPER` (not chosen by Plan 05), would require setup_joint_state_reorder atom + joint_states_isaac_raw topic. Forward-compat for plan re-decisions."

patterns-established:
  - "Hybrid-runtime verify harness pattern (D-15) — generalizable: port-detect liveness probe + cold-launch fallback + section-by-section pass/fail/warn accumulator with final summary. Reusable for ur5e-dt and soarm101-dt verify scripts."
  - "Soft-fail with remediation hint — when a check fails for a known-deferred reason, log STRATEGY-FALLBACK-NEEDED or `expected until Plan N deferrals resolved` message; FAIL is recorded but the script continues all subsequent steps so the human gets the full picture in one run."
  - "Audit-file-as-contract — Plan 04's parity_05_wrench_framing.txt and Plan 05's joint_ordering_probe.txt have standardized field names (Live frame_id:, Live Type:, Verdict:, Action:) that downstream verify scripts grep. Audit files are not just documentation — they're machine-consumed contracts."

requirements-completed: [TEX-01, TEX-02, TEX-03]

# Metrics
duration: ~10min
completed: 2026-05-02
---

# Phase 1 Plan 07: Verification Harness Summary

**Three-script + one-doc verify gate: regex-only TF tree diff utility, MCP-driven Kit-log texture sweep with augmented Isaac-Sim asset-failure patterns, and a 10-step hybrid-runtime harness that consumes Plan 04/05/06 audit-file outputs as machine-readable contracts.**

## Performance

- **Duration:** ~10 min
- **Started:** 2026-05-02 (post-Plan-06 same session)
- **Completed:** 2026-05-02
- **Tasks:** 3
- **Files created:** 4 (3 scripts + 1 doc)
- **Files modified:** 0

## Accomplishments

- `diff_tf_tree.py` (D-08) — 71-line regex-only .gv diff utility; self-test passes (`PASS: TF trees match (31 frames, 30 edges)`).
- `sweep_textures.py` (D-07 / TEX-01/02/03) — 200-line MCP+grep tool; smoke test on existing Kit log returned 267 hits including the known `assets/assets/SC` and `UR5e+gripper.usd` `Could not open` warnings (proves augmented patterns work).
- `verify_phase_1.sh` (D-15) — 441-line, 10-step hybrid-runtime gate harness. Steps cover cache discipline, port-detect/cold-launch, MCP atoms, ros2 topic check, PARITY-05 wrench framing, TF diff, _sim/_real grep, sweep, D-13 absence handling, and joint-ordering Verdict check.
- `texture-sweep.md` (TEX-03) — initial fix-log scaffold with augmented PATTERNS list documented inline + row-format spec.

## Task Commits

1. **Task 1: diff_tf_tree.py** — `fea9efd` (feat)
2. **Task 2: sweep_textures.py + texture-sweep.md** — `d883e4d` (feat)
3. **Task 3: verify_phase_1.sh** — `0d17ef4` (feat)

**Plan metadata:** _(this commit, pending)_

## Files Created/Modified

- `exts/aic-dt/scripts/diff_tf_tree.py` — TF tree .gv diff (D-08); regex-only parse; exit 0 / 1 / 2 contract.
- `exts/aic-dt/scripts/sweep_textures.py` — MCP `quick_start` trigger + Kit-log grep + Markdown table emit; PATTERNS = D-07 baseline + 4 augmented Isaac-Sim asset-failure regexes.
- `exts/aic-dt/scripts/verify_phase_1.sh` — 10-step Phase-1 verify gate, hybrid runtime, accumulator-style FAIL counting with section headers and final pass/fail banner.
- `exts/aic-dt/docs/texture-sweep.md` — TEX-03 fix log scaffold; sweep_textures.py appends `## Sweep run <ISO>` sections.

## Decisions Made

See `key-decisions` in frontmatter. Highlights:

- **Augmented patterns baked in pre-emptively** rather than waiting for "first run" iteration per the strict CONTEXT.md D-07 reading. Smoke test confirms the augmented set already catches sc_port_visual.usd's known broken sub-references — saves a full iteration cycle.
- **verify_phase_1.sh is the truth-telling gate.** It does not silently pass when Plan 06's documented deferrals are still in place. PARITY-03/04 are EXPECTED to fail; the failure message points at the deferred work directly.
- **Audit files as contracts.** Plan 04's `parity_05_wrench_framing.txt` and Plan 05's `joint_ordering_probe.txt` have standardized field names that this script greps. The audit-file-as-machine-readable-contract pattern is reusable for future cross-plan handoffs.

## Deviations from Plan

None — plan executed exactly as written. All three task acceptance-criteria sets passed first-try; no auto-fixes were required.

The smoke test on the existing Kit log (a best-effort verification beyond what the plan strictly required) surfaced one observation: the augmented `Could not open` pattern correctly fires on `assets/assets/SC` and `UR5e+gripper.usd` references in the existing log. This is exactly the pre-emptive coverage the augmented set was designed for, so it confirms the design rather than indicating a deviation.

## Issues Encountered

None during execution. Two minor process notes:

1. The user-supplied gsd-sdk binary is not on PATH for this shell, but it isn't needed — STATE.md and ROADMAP.md updates can be done via direct edit (per the metadata-commit step below).
2. Initial acceptance-criteria grep for the literal `! -d "$HOME/Documents/aic"` returned a false negative because of regex special-char escaping in the bash wrapper. `grep -F` confirmed the literal is present in `verify_phase_1.sh`.

## Self-Check: PASSED

Files created (verified present):

- `exts/aic-dt/scripts/diff_tf_tree.py` ✓ (executable, syntax-valid, self-test passes)
- `exts/aic-dt/scripts/sweep_textures.py` ✓ (executable, syntax-valid, smoke-tested 267 hits)
- `exts/aic-dt/scripts/verify_phase_1.sh` ✓ (executable, `bash -n` clean, --help works, --bogus correctly exits 2)
- `exts/aic-dt/docs/texture-sweep.md` ✓ (header + methodology + augmented patterns + row format)

Commits verified in `git log --oneline -5`:

- `fea9efd` ✓ (Task 1)
- `d883e4d` ✓ (Task 2)
- `0d17ef4` ✓ (Task 3)

Acceptance-criteria checks: ALL PASS (literal-token greps for `set -euo pipefail`, `PORT=8768`, `prime_usd_cache.py`, `restore known-good`, `launch_postload.py`, `quick_start`, `setup_tf_publisher`, `setup_joint_state_publisher`, `_sim`, `_real`, `kit_*.log`, `--no-launch`, `! -d "$HOME/Documents/aic"`, `diff_tf_tree.py`, `parity_05_wrench_framing.txt`).

## Environmental Gotchas Surfaced (for /gsd-verify-work)

These are NOT blockers for this plan (the script-authoring phase) but ARE potential issues when `verify_phase_1.sh` runs end-to-end:

1. **`tf2_tools view_frames` host availability (Open Q #5).** Step 6 (TF diff) will skip with WARN if `ros2 pkg list | grep tf2_tools` is empty on this host. Mitigation: docker fallback noted but not implemented for view_frames specifically — only for `ros2 topic list`.
2. **Cold cache → cold launch may take >180s.** Cold-launch poll loop is 60×3s. If PhysX cooking exceeds that, the script will fail with a specific log file pointer (`/tmp/aic_dt_verify.log`).
3. **Plan 06 deferrals (PARITY-03/04) are expected to fail Step 5/6/Step 10.** This is by design — the verify gate is honest about open work. A green run requires the follow-up plan resolving the Raw publisher + JointState wrapper deferrals.
4. **`/fts_broadcaster/wrench` is published only after Step 4's `setup_tf_publisher`/etc. complete.** Step 5a will WARN (not FAIL) if the topic isn't there yet — PARITY-05 is gated on the publisher actually running.

## End-to-End Run Status

**Not run end-to-end during this plan.** All three scripts are syntax-validated and (where lightweight) smoke-tested. End-to-end execution is `/gsd-verify-work`'s responsibility per the plan's own caveat ("this script is NOT run end-to-end during this plan's execution").

## Next Phase Readiness

- **Plan 08 (quick_start refactor per D-12):** ready. Plan 08 reorders/inserts the Plan 06 atoms into `quick_start`; once that lands, the verify harness's full chain becomes runnable.
- **Plan 09 (final verify):** ready. The harness exists; Plan 09 will run it end-to-end and resolve any remaining deferrals.
- **Sweep-iteration follow-up:** the augmented patterns are pre-emptive; first real sweep run during /gsd-verify-work will populate `texture-sweep.md` with classified rows. Each unresolved row drives the next iteration of asset fixes.

---

*Phase: 01-foundation-parity*
*Plan: 07*
*Completed: 2026-05-02*
