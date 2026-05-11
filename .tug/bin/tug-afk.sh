#!/usr/bin/env bash
# tug-afk.sh — capped Ralph loop. Per iter: EXECUTOR pass, then VERIFIER pass.
# Sentinel <promise>COMPLETE</promise> from executor exits the loop.

set -e

ITER="${1:?usage: tug afk <iterations> [project_dir]}"
PROJECT_DIR="${2:-${TUG_PROJECT_DIR:-$PWD}}"
PROJECT_DIR="$(cd "$PROJECT_DIR" && pwd)"
cd "$PROJECT_DIR"

[ -f plans/prd.json ]      || { echo "no plans/prd.json in $PROJECT_DIR"; exit 1; }
[ -f plans/progress.txt ]  || touch plans/progress.txt
mkdir -p .tug/logs

PERMISSION_FLAGS="${TUG_PERMISSION_FLAGS:---dangerously-skip-permissions}"
[ -f .tug/config.sh ] && . .tug/config.sh

# ─── EXECUTOR PROMPT ──────────────────────────────────────────────────────────
EXECUTOR_PROMPT=$(cat <<'PROMPT_EOF'
@plans/prd.json @plans/progress.txt @CLAUDE.md

You are the EXECUTOR agent inside one iteration of a tug loop driving Milestone 1 of the aic-dt extension. An ORCHESTRATOR (a separate live Claude session) is supervising this loop — it picks which iterations to run, may pin a specific task for you, and provides batch context. Read prd.json + progress.txt + CLAUDE.md + the orchestrator inputs below, then perform exactly ONE task and exit. A separate VERIFIER agent runs after you in a fresh context to independently re-verify your work.

ORCHESTRATOR INPUTS (read FIRST, BEFORE task-selection):
- Env var `TUG_PIN_TASK`: check via `echo $TUG_PIN_TASK` in a bash call. If set to a task id, you MUST work on exactly that task — skip the priority heuristic entirely. If the pinned task is already `passes:true`, append a one-line note to progress.txt explaining the no-op and exit normally (do NOT pick a substitute task).
- File `.tug/batch_context.md` (read if present): a short orchestrator briefing for the current batch — ordering bias, recent-decision context, "don't pull X forward yet", surface constraints, etc. The briefing OUTRANKS your default priority logic when they disagree (orchestrator has cross-iter context you lack).

STUCK ESCAPE VALVE (the /ask-gpt rule):
- Track your hypothesis-test attempts within this single iteration.
- If you make 2 hypothesis-test commits or 2 code-edit-then-verify cycles on the SAME root-cause diagnosis and both fail to close the task, you MUST invoke the `ask-gpt` skill (available in ~/.claude/skills/ask-gpt/) for an adversarial review of your diagnosis BEFORE a 3rd attempt. Include the failing diagnostic, your current hypothesis, and the 2 prior attempts in the consult. Log the consult outcome to progress.txt. This is non-negotiable — re-flailing on the same wrong root cause is the highest-cost failure mode of this loop.
- For OBVIOUSLY-different hypotheses (different root cause each attempt), the gate doesn't trigger — but be honest with yourself; "I changed the log line and re-ran" is not a new hypothesis.

PER-ITERATION MANDATES (BEFORE writing any code):
1. SKILL DISCIPLINE — if your selected task touches Isaac Sim 5.0, OpenUSD, OmniGraph, articulation, sensors, MCP socket, or any aic-dt extension internals, you MUST invoke the `isaac-sim-extension-dev` skill AND the `nvidia-suite-docs` skill BEFORE writing or editing code. NO EXCEPTIONS.
2. SIM PRECONDITION — if your selected task's `verification_mode` includes "sim", check `nc -z localhost 8768`. If down, attempt:
       bash -c 'source ~/env_isaaclab/bin/activate && DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'
   then poll up to 90s. If still down, append blocker note to plans/progress.txt and exit normally (no commit). SIM RESTART DISCIPLINE — sim stays running across iters; call quick_start() over MCP for clean scene; no kill+relaunch unless diagnosed corruption.
3. CROSS-REPO POLICY — edits anywhere outside isaac-sim-mcp PERMITTED in service of correct Isaac Sim integration; PROHIBITED if they artificially make trials pass (no disabling assertions in CheatCode/aic_*, no relaxing tolerances, no mocking). Every edit outside isaac-sim-mcp appended to plans/cross_repo_changes.json.
4. SCENE AUTHORING SOURCE OF TRUTH — AIC Gazebo configs in ~/Documents/aic/ are canonical for any default value. Mirror Gazebo, don't invent Isaac-Sim-idiomatic defaults.
5. WEB-FIRST DISCIPLINE — debug / interop tasks (errors, RMW, zenoh, ABI, RPC stalls, kilted-humble compat, etc.): BEFORE >1 hypothesis-test commit, you MUST:
   (a) WebSearch with exact-symptom queries — try `"<exact error>"`, `<framework> <symptom> site:discourse.ros.org`, `site:github.com issues`, `site:forums.developer.nvidia.com`. 2-3 variations.
   (b) WebFetch top 2-3 results — read GitHub issues, Discourse, forum posts.
   (c) Log searches in progress.txt (query + finding). Cite URLs in commit body for solutions.
   (d) NVIDIA stack → `nvidia-suite-docs` skill. ROS / zenoh / general → WebSearch + WebFetch directly.
   NOT optional for debug-shaped tasks — re-deriving an already-documented fix is the most common wasted-iteration pattern. Skip ONLY for pure new authoring (no error to search).

PRE-FLIGHT (BEFORE task selection):
- Run `git status --porcelain`. If uncommitted changes exist: read them + last 3 progress.txt entries, decide explicitly: continue prior task (only if its passes is still false and changes trace to it) OR `git stash` (no checkout/delete) and pick fresh. Log choice to progress.txt before proceeding.

BLOCKER DETECTION (before task selection):
- Read last 5 progress.txt entries. If a single architectural blocker is mentioned 2+ times AND no existing PRD task targets it: output on LAST line `<promise>BLOCKED reason="<short, <=80 chars>"</promise>` and exit. Driver halts loop, surfaces to operator.
- DO NOT emit BLOCKED if a PRD task already targets the blocker — pick that task.
- DO NOT emit BLOCKED for transient situations (sim down, single verify fail). BLOCKED is for genuinely-impassable architectural blockers needing operator decision.

TASK SELECTION:
1. Parse plans/prd.json. Find ALL tasks with `passes: false`. Flat kanban — pick freely.
2. Read each candidate's description. If a task references others informally and you have strong reason to believe an informal prerequisite isn't in place, prefer a different task whose path to success is clearer.
3. Pick highest-priority — NOT necessarily first in list, NOT alphabetical. Order: critical bugfixes > dev infrastructure > tracer bullets > polish > refactors.
4. ONLY WORK ON A SINGLE TASK.

IMPLEMENTATION:
1. Read task's `description`, `acceptance`, `verify_command`, `verification_mode` carefully.
2. TDD where applicable. For sim-bound tasks, the `verify_command` is the gate.
3. Minimal targeted edits.

VERIFY:
1. Run the task's `verify_command` exactly.
2. Confirm every `acceptance` item is observably true.
3. If verify fails: do NOT commit, do NOT flip passes. Append diagnostic to progress.txt and exit. Next iteration retries.

REGRESSION SWEEP (AFTER your verify passes, BEFORE committing):
1. Read your selected task's `files_likely`.
2. For every other task in plans/prd.json with `passes: true` whose `files_likely` overlaps with yours (any shared file path): re-run its `verify_command`.
3. If any regress: do NOT commit your work. Edit plans/prd.json to flip the regressed task's `passes` from true back to false. Append "REGRESSION: <regressed_task_id> broken by your work on <your_task_id> — <diagnostic>" to plans/progress.txt. Exit. Next iteration sees both tasks as passes:false.
4. If no regression: proceed to ON SUCCESS.

ON SUCCESS:
1. Edit plans/prd.json — flip your task's `passes` from false to true. Modify ONLY that field on that task.
2. APPEND to plans/progress.txt: timestamp, task id, key decisions, files changed, blockers/notes.
3. ONE git commit per git repository touched, each referencing the same task_id, Conventional Commits style. PRIMARY commit lives in isaac-sim-mcp covering prd.json + progress.txt + cross_repo_changes.json + any aic-dt code. If you edited ~/Documents/aic, cd there for SEPARATE commit; write its sha back into the matching cross_repo_changes.json entry. No Co-Authored-By. No --no-verify. No destructive git ops.

4. COMMIT BODY SKILL CITATION DISCIPLINE: For tasks whose `verification_mode` is "sim" or "sim+docker" OR whose `skills_required` is non-empty, the commit body MUST include one line per consulted skill in this exact format:
       Skill cited: <skill_name> for <one-line concrete use>
   If you genuinely did not consult any skill (trivial change, no API surface touched), include "Skill cited: none — <one-line reason>". Absence of either form is a discipline failure — flag in progress.txt for operator review.

If after this iteration EVERY task has `passes: true`, output on the LAST line: <promise>COMPLETE</promise>

Do NOT use any other sentinel. If stuck on external dep, append note and exit normally.
PROMPT_EOF
)

# ─── VERIFIER PROMPT ──────────────────────────────────────────────────────────
VERIFIER_PROMPT=$(cat <<'PROMPT_EOF'
@plans/prd.json @plans/progress.txt @CLAUDE.md

You are the VERIFIER agent. The EXECUTOR agent just finished one iteration. Your job: independently confirm the executor's claim in a FRESH context.

STUCK ESCAPE VALVE (the /ask-gpt rule):
- If your verification produces an AMBIGUOUS outcome (verify_command exit code disagrees with acceptance-item observability, OR you cannot decide between CASE A and CASE B because the executor's claim is technically correct but the underlying behavior is suspect), invoke the `ask-gpt` skill for an adversarial second opinion BEFORE choosing CASE A/B/C. Include the verify_command output, the acceptance items, and your concern. This prevents false-VERIFIED on regressions disguised as passes (the 2026-05-08 PARITY-05 wrench all-zeros lesson).

DISCIPLINE:
1. Run `git log -1 --format=%H%n%s%n%b`. Parse the conventional-commits scope for the task_id (e.g. "feat(parity-report-tool):" → "parity-report-tool"). If the most recent commit is a chore(verify) or revert(verify) (a previous verifier's commit), or there is no fresh executor commit, exit normally — nothing to verify.
2. Find that task in plans/prd.json. Confirm passes is true.
3. SKILL DISCIPLINE — if `verification_mode` includes "sim", invoke isaac-sim-extension-dev + nvidia-suite-docs before running anything. Same rule as executor.
4. SIM PRECONDITION — if verification_mode includes "sim", check `nc -z localhost 8768`. Do NOT launch (executor left it running). If down: append "VERIFY-INCONCLUSIVE <task_id> — sim socket down" to plans/progress.txt, make doc-only commit `chore(verify): inconclusive — <task_id>`, exit.

VERIFICATION:
1. Run the task's `verify_command` EXACTLY as written in prd.json.
2. Independently confirm every `acceptance` item is observably true RIGHT NOW (not what executor reported earlier). For sim items: query live MCP/ros2 state. For docker items: re-run container check. For unit-test items: re-run script.
3. Compare actual against acceptance.

OUTCOME:

CASE A — All gates green:
- Append "VERIFIED <task_id> by verifier-agent at <ts> — <gates passed brief>" to plans/progress.txt.
- Commit: `chore(verify): <task_id> verified — <one-line>`.
- COMMIT BODY SKILL CITATION: For sim or sim+docker verification, commit body MUST include "Skill cited: <skill_name> for <use>" per consulted skill, or "Skill cited: none — <reason>" if not needed.
- Exit.

CASE B — Any gate red:
- Edit plans/prd.json: flip `passes` from true back to false on that task.
- Append "VERIFY-FAIL <task_id> by verifier-agent at <ts> — <which gate failed, actual vs expected>" to plans/progress.txt with enough diagnostic for the NEXT executor iter to fix.
- Commit: `revert(verify): <task_id> failed verification — <one-line>`.
- DO NOT fix the code. Next executor iter's job.
- Exit.

CASE C — Inconclusive:
- Append "VERIFY-INCONCLUSIVE <task_id> by verifier-agent at <ts> — <why>".
- Commit: `chore(verify): inconclusive — <task_id>`.
- Exit.

ONLY verify the most recent task. No other tasks. No new work.
PROMPT_EOF
)

START_TS="$(date +%Y%m%d-%H%M%S)"
LOG="$PWD/.tug/logs/afk-$START_TS.log"
echo "[tug] AFK starting — project=$PROJECT_DIR  iter=$ITER  log=$LOG"
echo "[tug] TUG_PIN_TASK=${TUG_PIN_TASK:-<unset, executor picks>}"
if [ -f .tug/batch_context.md ]; then
  echo "[tug] batch_context.md present ($(wc -l < .tug/batch_context.md) lines) — executor will read it"
else
  echo "[tug] batch_context.md absent — executor uses default priority logic"
fi

for ((i=1; i<=ITER; i++)); do
  echo
  echo "=== tug iteration $i / $ITER  ($(date '+%Y-%m-%d %H:%M:%S')) ==="
  EXEC_LOG="$PWD/.tug/logs/iter-$START_TS-$(printf '%03d' "$i")-exec.log"
  VERIFY_LOG="$PWD/.tug/logs/iter-$START_TS-$(printf '%03d' "$i")-verify.log"

  HEAD_BEFORE="$(git rev-parse HEAD 2>/dev/null || echo none)"

  # ─── EXECUTOR ─────────────────────────────────────────────────────────────
  echo "--- EXECUTOR pass ---"
  set +e
  claude $PERMISSION_FLAGS -p "$EXECUTOR_PROMPT" 2>&1 | tee "$EXEC_LOG"
  EXEC_RC=${PIPESTATUS[0]}
  set -e

  EXEC_TAIL="$(tail -10 "$EXEC_LOG")"
  echo "$EXEC_TAIL" >> "$LOG"

  # COMPLETE sentinel from executor → end of loop.
  if printf '%s' "$EXEC_TAIL" | grep -q '<promise>COMPLETE</promise>'; then
    echo
    echo "[tug] COMPLETE detected after $i iterations."
    [ -n "${TUG_NOTIFY_CMD:-}" ] && eval "$TUG_NOTIFY_CMD \"tug COMPLETE after $i iterations in $PROJECT_DIR\""
    exit 0
  fi

  # BLOCKED sentinel from executor → halt loop, surface to operator.
  if BLOCKED_LINE="$(printf '%s' "$EXEC_TAIL" | grep -oE '<promise>BLOCKED[^<]*</promise>' | tail -1)" && [ -n "$BLOCKED_LINE" ]; then
    echo
    echo "[tug] ═══════════════════════════════════════════════════════════════"
    echo "[tug] BLOCKED detected after $i iterations — operator intervention needed."
    echo "[tug] $BLOCKED_LINE"
    echo "[tug] ───────────────────────────────────────────────────────────────"
    echo "[tug] Recent progress.txt context (last 15 lines):"
    tail -15 plans/progress.txt 2>/dev/null | sed 's/^/    /'
    echo "[tug] ═══════════════════════════════════════════════════════════════"
    echo "[tug] Resolve the blocker (add a PRD task targeting it, fix manually,"
    echo "[tug] or rescope), then re-run tug-afk.sh."
    [ -n "${TUG_NOTIFY_CMD:-}" ] && eval "$TUG_NOTIFY_CMD \"tug BLOCKED after $i iterations: $BLOCKED_LINE\""
    exit 4
  fi

  # ─── VERIFIER (only if executor made a new commit) ────────────────────────
  HEAD_AFTER="$(git rev-parse HEAD 2>/dev/null || echo none)"
  if [ "$HEAD_BEFORE" = "$HEAD_AFTER" ]; then
    echo "--- VERIFIER pass: skipped (executor did not commit) ---"
  else
    echo "--- VERIFIER pass ---"
    set +e
    claude $PERMISSION_FLAGS -p "$VERIFIER_PROMPT" 2>&1 | tee "$VERIFY_LOG"
    VERIFY_RC=${PIPESTATUS[0]}
    set -e
    VERIFY_TAIL="$(tail -10 "$VERIFY_LOG")"
    echo "$VERIFY_TAIL" >> "$LOG"
    if [ "$VERIFY_RC" -ne 0 ]; then
      echo "[tug] verifier exited rc=$VERIFY_RC — continuing"
    fi
  fi

  if [ "$EXEC_RC" -ne 0 ]; then
    echo "[tug] iteration $i executor exited rc=$EXEC_RC — continuing"
  fi
done

echo
echo "[tug] iteration cap ($ITER) reached without COMPLETE."
echo "[tug] review .tug/logs/, decide whether to extend, refine PRD, or step in."
[ -n "${TUG_NOTIFY_CMD:-}" ] && eval "$TUG_NOTIFY_CMD \"tug cap ($ITER) hit in $PROJECT_DIR\""
exit 3
