# Project-local tug for isaac-sim-mcp

Forked copy of `~/Documents/tug/bin/{tug,tug-once.sh,tug-afk.sh}` adapted for orchestrator-driven use against Milestone 1 of the aic-dt extension. Same two-pass loop (executor + verifier, fresh contexts, atomic prd.json + progress.txt + git discipline). The difference is the **orchestrator role** — the live Claude session driving development — has explicit hooks for pinning tasks and injecting batch context, replacing the blind "agent picks priority" heuristic.

## Roles

| Role | Who | Context | Responsibility |
|---|---|---|---|
| **Orchestrator** | live Claude session (you) | persistent | Reads PRD + progress + git state; decides next task(s); pins task per iter; writes `batch_context.md`; runs `tug afk N`; reviews logs between batches; escalates blockers; consults `/ask-gpt` on stuck |
| **Executor** | `claude -p` subprocess | fresh each iter | Works exactly one task (pinned or self-picked), implements, runs verify, regression-sweeps, commits atomically, exits |
| **Verifier** | `claude -p` subprocess | fresh after executor commits | Independently re-runs verify, three outcomes (VERIFIED / VERIFY-FAIL / VERIFY-INCONCLUSIVE), commits result |

## Orchestrator hooks

**`TUG_PIN_TASK=<task_id>`** — exported env var. Executor reads it via `echo $TUG_PIN_TASK` and pins to that task instead of running its priority heuristic. If the pinned task is already `passes:true`, executor no-ops and exits. Lets the orchestrator force sequential dependency order (e.g. `trial-cycle-frames-loading` → `trial-1-fire` → `trial-2-fire` → `trial-3-fire`) without relying on the executor's selection bias.

**`.tug/batch_context.md`** — short markdown briefing the executor reads BEFORE task selection. The briefing OUTRANKS the executor's default priority logic. Use it for:
- Recent decision context the executor would otherwise miss (gets compacted out of fresh contexts)
- Ordering bias inside a batch (`do A first, then B`)
- "Don't pull X forward yet — research-gated"
- Surface constraints (`only touch extension.py — leave scoring_publishers.py alone for this batch`)

The orchestrator owns the lifecycle of `batch_context.md` — write before a batch, blank or delete after. Gitignored (transient).

## Stuck-escape: `/ask-gpt`

Both executor and verifier prompts mandate consulting the `ask-gpt` skill when:
- **Executor**: 2 hypothesis-test commits / verify cycles on the same root-cause diagnosis fail. MUST consult before a 3rd attempt. (Re-flailing on the wrong root cause is the highest-cost failure mode of the loop.)
- **Verifier**: ambiguous outcome — verify_command exit code disagrees with acceptance-item observability, OR executor's claim is technically correct but underlying behavior is suspect. Consult before choosing CASE A/B/C.

Lesson encoded: 2026-05-08 PARITY-05 wrench-broadcaster passed `frame_id` + rate gates but emitted all-zeros payload. Verifier without ask-gpt would have marked VERIFIED.

## Usage

From project root (`/home/aaugus11/Documents/isaac-sim-mcp`):

```bash
# One iteration, interactive TUI (orchestrator can watch live):
bash .tug/bin/tug once

# Pin a specific task:
TUG_PIN_TASK=trial-cycle-frames-loading bash .tug/bin/tug once

# Capped AFK loop (orchestrator triggers, walks away, returns to review logs):
bash .tug/bin/tug afk 3

# AFK loop with pin + batch context:
echo "Work trial-cycle-frames-loading FIRST. Then trial-1-fire. Skip wrist-cameras-restore — Phase 3 surface." > .tug/batch_context.md
TUG_PIN_TASK=trial-cycle-frames-loading bash .tug/bin/tug afk 5

# Status:
bash .tug/bin/tug status
```

## Orchestrator loop

```
┌─── while M1 not shipped ──────────────────────────────────────────────┐
│ 1. Read plans/prd.json, progress.txt tail, git log -5                 │
│ 2. Pick next task(s) — sequential chain or independent fan-in         │
│ 3. (Optional) Write .tug/batch_context.md with batch briefing         │
│ 4. TUG_PIN_TASK=<id> bash .tug/bin/tug afk <small N like 2-5>         │
│ 5. Wait for completion / BLOCKED sentinel                             │
│ 6. Inspect .tug/logs/afk-*.log tail + progress.txt tail + git log     │
│ 7. If BLOCKED → consult /ask-gpt, edit prd.json, retry or escalate    │
│ 8. If COMPLETE → loop exits, check ship-paperwork remains             │
│ 9. Goto 1                                                             │
└───────────────────────────────────────────────────────────────────────┘
```

**Why serial batches, not parallel:** the three M1-bound serialization points (sim port 8768, prd.json write contention, `.git/index.lock`) make parallel iters unsafe without worktrees + lock infra. For M1's near-linear remaining dependency graph (`trial-cycle-frames-loading → trial-1/2/3-fire → parity-report-run → ship-paperwork`), parallelism buys zero. Serial batches with orchestrator check-ins between is the right shape.

**Why small N per batch:** each batch ≈ orchestrator check-in cost. N=2-5 keeps orchestrator context fresh on outcomes without paying the per-iter overhead of `tug once`. If the executor is reliably closing tasks, N can grow.

## Files

| File | Role |
|---|---|
| `bin/tug` | dispatcher (once / afk / status / help) |
| `bin/tug-once.sh` | one HITL iteration (interactive TUI) |
| `bin/tug-afk.sh` | capped AFK loop with per-iter logs to `.tug/logs/` |
| `batch_context.md` | (gitignored, transient) orchestrator briefing for current batch |
| `config.sh` | (gitignored, optional) `TUG_PERMISSION_FLAGS`, `TUG_NOTIFY_CMD` overrides |
| `logs/` | (gitignored) per-iteration agent stdout |
| `README.md` | this file |

## Sentinels

Loop driver greps the last 10 lines of executor stdout for:

- `<promise>COMPLETE</promise>` — every task in `plans/prd.json` is `passes:true` → loop exits 0
- `<promise>BLOCKED reason="..."</promise>` — hard external blocker → loop exits 4

The orchestrator (live Claude session) treats both as "wake up and decide what next". BLOCKED almost always = consult `/ask-gpt` or fix an architectural gap in the PRD.

## Relationship to the global `~/Documents/tug/`

The global tug at `~/Documents/tug/bin/` was originally single-tenant for isaac-sim-mcp. Now that this project has its own `.tug/bin/` fork with orchestrator hooks, the global is unmodified-but-stale — keep it around as the template, or retire it once aic_vision (also project-local-forked) is the only other consumer. Do NOT edit the global from this project.

---

*Bootstrapped 2026-05-11 during the M1 autonomous-mode push. Adapted from `~/Documents/aic/.tug/` pattern + orchestrator hooks added.*
