# Phase 7: Control GUI Service Audit & IK Cleanup - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in `07-CONTEXT.md` — this log preserves the alternatives considered.

**Date:** 2026-04-18
**Phase:** 07-control-gui-service-audit-ik-cleanup
**Areas discussed:** Scope reconciliation (informed by chat-history subagent), IK consolidation strictness, FK/IK duplicate-widget consolidation, button↔service enforcement, drop_refresh fix approach, IK risk gating

---

## Gray area selection (multiSelect)

Initial presentation offered five gray areas. User responded with a meta-answer rather than a selection:

> "i just wanted to discuss whats the scope of this phase and if you know what to do with best principles in mind if the plan is diffenrt now is the time for you to speak up"

**Interpretation:** User asked for a best-principles scope pressure-test before selecting gray areas. Claude presented a refined four-workstream scope and flagged three concerns with the literal roadmap wording (IK one-path interpretation, motion entry-point discipline, Phase 6 regression risk).

---

## IK Unification Scope

| Option | Description | Selected |
|--------|-------------|----------|
| Organized, keep KDL (Recommended) | One pipeline with named stages. Keep both geometric and MoveIt KDL solvers as documented fallbacks. Extract shared helpers, rename entry points. No functional regression. Phase 6 verification as gate. | |
| Literal — delete KDL path | Delete `_compute_ik_moveit` entirely. Regresses the IK tab's freeform xyz+quaternion support. | |
| Defer IK cleanup to a later phase | Phase 7 only button audit + drop_refresh. Create Phase 7.1 for IK once a regression test harness exists. | |

**User's choice:** Requested chat-history investigation before committing. Subagent found turn-61 quote *"cleaning out functions duplicates and find one proper way of executing all the codes"* — pragmatic framing, not deletionist. After re-presentation with history context, user selected the Recommended "Organized, keep KDL" approach (implied via the later "IK consolidation — risk gating" selection confirming Phase 6 3/3 gate, which only applies to the non-deletion path).

**Notes:** The literal "ONE IK path" wording is Claude's ROADMAP summarization, not user voice. Pragmatic consolidation wins.

---

## FK/IK Duplicate Tab Widgets

| Option | Description | Selected |
|--------|-------------|----------|
| Yes, defer (Recommended) | Duplicate Reset Arm / Randomize / gripper buttons on both FK and IK tabs are UX choices, not code-hygiene debt. Note as deferred idea. | |
| No, consolidate now | Extract the repeated button cluster into a shared helper frame. | ✓ |

**User's choice:** "No, consolidate now."
**Notes:** User explicitly overrode the recommended "defer" option. Workstream D added to Phase 7 scope. D-17 adds pixel-identical layout verification via the x11-window-screenshot skill to mitigate tkinter layout fragility.

---

## Button↔Service Enforcement Deliverable

| Option | Description | Selected |
|--------|-------------|----------|
| Runtime `~/dump_services` + test (Recommended) | `_cmd_dump_services` Trigger + unit test asserting 1:1 Button→`_cmd_*` mapping. `AGENT_DEBUG_GUIDE.md` regenerated from the dump. CI gate. | ✓ |
| Runtime `~/dump_services` only | Introspection command + regenerated markdown. No CI gate. | |
| Markdown only | Regenerate `AGENT_DEBUG_GUIDE.md`. Same as today's surface. | |

**User's choice:** Recommended — runtime + test.
**Notes:** This is the highest-motivation workstream. The CI test is what would have caught the historical Drop Refresh lambda-vs-service divergence.

---

## drop_refresh Fix Approach

| Option | Description | Selected |
|--------|-------------|----------|
| Sequence against fresh data (Recommended) | Remove stale cups, invalidate `_drop_data`, wait for next `/drop_poses` message (with timeout), re-add collisions + markers atomically. Handles moved/disappeared/new cups. | ✓ |
| In-place overwrite | Don't clear `_drop_data`. Iterate current entries, republish by id. Stale entries persist if a cup disappears from the topic. | |
| You decide based on testing | Plan runs both, picks whichever passes the move-cup-refresh-verify check. Claude's discretion. | |

**User's choice:** Recommended — sequence against fresh data.
**Notes:** The `root.after(500/600/700)` chain is removed in favor of a threaded, event-gated sequence consistent with the existing `_motion_event` pattern.

---

## IK Consolidation Risk Gating

| Option | Description | Selected |
|--------|-------------|----------|
| Phase 6 3/3 regression as gate (Recommended) | Re-run Phase 6's 3-consecutive-clean-passes verification for all 3 cups after IK consolidation, before merge. Non-negotiable. | ✓ |
| Lighter gate: smoke test | One grasp, one drop, one IK-tab freeform pose. Misses intermittent collision cases. | |
| Split to Phase 7.1 | Ship button audit + drop_refresh + tab consolidation in Phase 7. Separate phase for IK once regression harness exists. | |

**User's choice:** Recommended — Phase 6 3/3 regression as gate.
**Notes:** Protects the Phase 5 → 05.1 → 6 verified pipeline. Any regression in the 3/3 test blocks merge.

---

## Claude's Discretion

- Exact commit/plan ordering across the four workstreams (suggested: audit → drop_refresh → tab consolidation → IK cleanup → Phase 6 regression, risk-ascending).
- Naming of the internal service-registry helper used by the audit test.
- Whether the unit test walks the live widget tree at GUI-startup time or parses the AST of `control_gui.py`.
- Timeout value for the drop_refresh "wait for fresh data" step (start 2.0s, tune if flaky).

## Deferred Ideas

- `_cmd_grasp_move` internals refactor — Phase 6-verified, regression risk outweighs cleanup value.
- Swapping MoveIt KDL for IKFast/TracIK — separate investigation, future phase.
- Three-gripper-button-pair UX consolidation (`Grasp Open/Close`, `Open/Close range`, direct `Open/Close`) — distinct from Workstream D's arm-cluster duplication.
- CAD cup mesh loader cache invalidation (L2954–L2957) — works, unrelated to `drop_refresh` symptom.
- Unified `/drop_poses` + `/objects_poses` pose source — Phase 8/9 territory.
