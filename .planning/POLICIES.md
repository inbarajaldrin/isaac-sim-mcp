# GSD operating policies

> **Moved from CLAUDE.md on 2026-05-17.** These three policy sections —
> phase-scope-by-surface, forward-pull enforcement, and phase-closure
> discipline — accreted in CLAUDE.md as the project hit and absorbed
> structural failures. Each one is the "lesson learned" version of a real
> incident. They belong in the planning record, not the on-ramp.
>
> The CLAUDE.md `## GSD workflow` section now carries the command surface +
> a pointer to this file for the operational rules.

## Phase scope is by surface, not capability

This project deviates from default GSD phase discipline. **Before planning a phase, scan future-phase requirements** in `.planning/REQUIREMENTS.md` and `.planning/ROADMAP.md`. **Pull a future-phase requirement into the current phase if all of these are true:**

1. **Surface adjacency** — its primary edit surface (files, data structures, UI atoms) overlaps what the current phase is already touching.
2. **Dependencies met** — no upstream research decision, prior phase, or external review is still pending for it.
3. **No new research needed** — pulling it in doesn't expand the research scope or invalidate the discuss-phase decisions already locked.

The cost of splitting requirements that share a surface is invisible at plan time but real at execution: editor focus, mental model, and review surface all get re-paid weeks later. Pulling forward avoids that re-entry tax.

**When pulling forward:** edit `.planning/REQUIREMENTS.md` traceability table to remap the IDs to the current phase, edit `.planning/ROADMAP.md` to add them to the phase's `Requirements:` list, then re-run `/gsd-plan-phase N` (with `--research` if the new IDs need fresh investigation, otherwise without). The CONTEXT.md and existing research stay valid; the planner produces additional plans for the pulled-forward IDs.

**When NOT to pull forward:**
- The requirement is research-gated (e.g., cable physics in Phase 3 awaits `nvidia-suite-docs` evaluation of deformable / articulated / hybrid strategies — pulling it in fakes the decision).
- Its surface is meaningfully separate (e.g., the `aic_controller` command-loop surface vs. the passive-publisher surface — they share `extension.py` but the architectural concerns are distinct).
- The phase is already at its context budget (a planner returning `## PHASE SPLIT RECOMMENDED` is signal to NOT pull more in).

This rule applies recursively: when planning Phase 2, scan Phase 3+ for the same overlap criteria; same for Phase 3 looking at Phase 4. The roadmap is the *plan*, not a *contract*.

## Forward-pull enforcement — three checkpoints, not a guideline

**Surfaced 2026-05-08:** the original forward-pull rule above was a soft guideline with no verification. Agents were supposed to scan future-phase requirements at planning time but no checker confirmed it happened. Result: Phase 1's extension.py surface was re-edited by Phase 2 + 3 + 4 without anyone scanning forward — wrist-camera prim-path bug (Gap C / 01-G05) and the articulation-DOF coupling that broke PARITY-09 in Phase 3 are both forward-pull misses.

**Three enforcement points:**

**(1) discuss-phase produces a `## Forward-pull scan` block in CONTEXT.md.** Mandatory section. For every future-phase whose requirements share a surface (file / data structure / UI atom) with the current phase, list:

```
| Future Req | Future Phase | Surface overlap | Decision | Rationale |
|------------|--------------|-----------------|----------|-----------|
| SCENE-05   | Phase 3      | extension.py::load_robot cable subtree | DEFERRED | Research-gated on nvidia-suite-docs; pulling forward fakes the strategy decision |
| TEX-03     | Phase 1      | exts/aic-dt/docs/texture-sweep.md | PULLED  | Sweep-script surface adjacent; no research needed |
```

If no future-phase shares a surface with this one, write `## Forward-pull scan: NONE — no surface overlap with Phase N+1, N+2, ...` with explicit justification per future phase. The empty case must be defended, not silent.

**(2) plan-checker verifies the block exists + is non-vacuous.** Before approving a phase plan, the plan-checker agent:
- Confirms `## Forward-pull scan` exists in CONTEXT.md.
- For each future phase, confirms either at least one entry OR explicit "no overlap" justification.
- Cross-checks: for any future-phase requirement whose declared surface (per ROADMAP.md `Requirements:` block) matches a file in the current phase's plan task list, the requirement MUST appear in the forward-pull table. Missing entries fail the checker.

**(3) execute-phase halts on surface-touch surprise.** When a plan's task touches a file that's listed as primary surface for a future-phase requirement NOT in the forward-pull table, the executor halts and surfaces:

```
[FORWARD-PULL HALT] Task X edits exts/aic-dt/aic_dt/extension.py.
This file is also the primary surface for SCENE-05 (Phase 3) per ROADMAP.md.
SCENE-05 was not in the forward-pull table for this phase.
Either pull SCENE-05 forward now (update CONTEXT + REQUIREMENTS + ROADMAP) or
document why the surface-touch doesn't qualify (e.g. plan touches a different
function in the same file).
```

This catches the late-discovered overlap that discuss-phase missed.

**Forward + backward = closure principle.** Every phase scans both directions:
- **Forward (at discuss-phase):** "What future-phase requirements could I close while I'm in this surface?"
- **Backward (at phase-closure / backlog-sweep):** "What prior-phase deferrals can I close because this phase already edited their surface?"

A surface is "done" only when all its requirements across all phases that don't have hard external dependencies (research-gated, upstream-blocker-gated) are addressed. Phase boundaries are scope organizers, not capability fences.

**Concrete artifacts produced:**
- `.planning/phases/<phase>/CONTEXT.md` `## Forward-pull scan` block (mandatory)
- `.planning/phases/<phase>/CLOSURE.md` `## Backlog-sweep` block (mandatory at closure)
- Both blocks reference the live REQUIREMENTS.md inline checkboxes; both flip them as part of the same atomic commit.

## Phase closure discipline — backlog sweep + payload sanity

**Surfaced 2026-05-08 after a runtime audit found 5 over-claimed closures (PARITY-05/09/10/12, SCENE-05) and 6 stale-Pending entries that were actually closed.** Two structural failures:

1. **Closure ceremony only updated downstream artifacts.** Phase 3 + Phase 4 plan landings updated `STATE.md` and the inline `[x]/[ ]/[~]` checkboxes in REQUIREMENTS.md but never refreshed the **traceability table at the bottom of REQUIREMENTS.md**, which became the most-quoted-but-most-stale signal. Future status reports keep reading the wrong source.
2. **Verifiers were structural, not runtime.** Smoke tests checked "topic exists + rate > 0 + frame_id matches" but never sampled payload. PARITY-05 wrench passed rate=13.6Hz + frame_id=ati/tool_link gates but emitted all-zeros. PARITY-09 buffered+applied joint commands but never published a real cmd and read /joint_states for delta.

**Two policies enforced going forward:**

**(A) Backlog sweep at every phase closure.** Before declaring a phase `[x]` closed:
- Scan the **cumulative deferred-items list** across ALL prior phase SUMMARY.md files.
- For each deferred item whose surface (file / data structure / UI atom) the closing phase touched, **either close it now or document explicitly why the deferral persists** (with the new gating dependency).
- Update the REQUIREMENTS.md traceability table **as part of closure** — same commit as STATE.md update + ROADMAP plan checkbox flip. Closure isn't atomic if any of those drift.

**(B) Runtime payload sanity gate in every verifier.** `verify_phase_*.sh` harnesses must include for each published topic:
- One sample read with `rclpy` (or equivalent) within N seconds of `quick_start`.
- Assertion that payload has at least one nonzero field (or, for event topics, that the trigger fires deterministically under a known-good stimulus — e.g. CheatCode plug-into-port for PARITY-07).
- For each subscribed topic, a publish-then-read round-trip: publish a known-good command, read the topic that should reflect the response, assert delta within tolerance.

**Status-reading discipline for any agent reporting M1 progress:**

| Source | Truth status | When to consult |
|---|---|---|
| `REQUIREMENTS.md` **inline `[x]/[~]/[ ]`** checkboxes per requirement | **Primary** | When asked "is requirement X done?" |
| `REQUIREMENTS.md` **traceability table** (bottom of file) | Derived; verify against inline | Cross-check, never sole source |
| `STATE.md` progress block | Snapshot at last phase boundary | Direction + magnitude of progress, not per-requirement truth |
| `HANDOFF.json` blockers + completed_tasks | Live | Active blockers + last session's deltas |
| Live `ros2 topic` / runtime probe | Ground truth | When `[x]` claim is uncertain or the requirement is runtime-behavioral |

**Rule:** never quote the traceability table as gospel without cross-checking inline checkboxes. If they diverge, run a runtime probe before declaring status. The 2026-05-08 reconciliation pass was forced by exactly this divergence.
