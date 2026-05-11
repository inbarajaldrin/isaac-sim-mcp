#!/usr/bin/env bash
# tug-once.sh — one Ralph iteration HITL: EXECUTOR pass, then VERIFIER pass.
# Both run interactively (Pocock-canonical TUI) so you watch each work.

set -e

PROJECT_DIR="${1:-${TUG_PROJECT_DIR:-$PWD}}"
PROJECT_DIR="$(cd "$PROJECT_DIR" && pwd)"
cd "$PROJECT_DIR"

[ -f plans/prd.json ]      || { echo "no plans/prd.json in $PROJECT_DIR — write your PRD first"; exit 1; }
[ -f plans/progress.txt ]  || touch plans/progress.txt

PERMISSION_FLAGS="${TUG_PERMISSION_FLAGS:---dangerously-skip-permissions}"
[ -f .tug/config.sh ] && . .tug/config.sh

echo "[tug] once starting — project=$PROJECT_DIR"
echo "[tug] TUG_PIN_TASK=${TUG_PIN_TASK:-<unset, executor picks>}"
if [ -f .tug/batch_context.md ]; then
  echo "[tug] batch_context.md present ($(wc -l < .tug/batch_context.md) lines) — executor will read it"
else
  echo "[tug] batch_context.md absent — executor uses default priority logic"
fi

# ─── EXECUTOR PROMPT ──────────────────────────────────────────────────────────
EXECUTOR_PROMPT=$(cat <<'PROMPT_EOF'
@plans/prd.json @plans/progress.txt @CLAUDE.md

You are the EXECUTOR agent inside one iteration of a tug loop driving Milestone 1 of the aic-dt extension. An ORCHESTRATOR (a separate live Claude session) is supervising this loop — it picks which iterations to run, may pin a specific task for you, and provides batch context. Read prd.json + progress.txt + CLAUDE.md + the orchestrator inputs below, then perform exactly ONE task and exit. A separate VERIFIER agent will run after you in a fresh context to independently re-verify your work.

ORCHESTRATOR INPUTS (read FIRST, BEFORE task-selection):
- Env var `TUG_PIN_TASK`: check via `echo $TUG_PIN_TASK` in a bash call. If set to a task id, you MUST work on exactly that task — skip the priority heuristic entirely. If the pinned task is already `passes:true`, append a one-line note to progress.txt explaining the no-op and exit normally (do NOT pick a substitute task).
- File `.tug/batch_context.md` (read if present): a short orchestrator briefing for the current batch — ordering bias, recent-decision context, "don't pull X forward yet", surface constraints, etc. The briefing OUTRANKS your default priority logic when they disagree (orchestrator has cross-iter context you lack).

STUCK ESCAPE VALVE (the /ask-gpt rule):
- Track your hypothesis-test attempts within this single iteration.
- If you make 2 hypothesis-test commits or 2 code-edit-then-verify cycles on the SAME root-cause diagnosis and both fail to close the task, you MUST invoke the `ask-gpt` skill (available in ~/.claude/skills/ask-gpt/) for an adversarial review of your diagnosis BEFORE a 3rd attempt. Include the failing diagnostic, your current hypothesis, and the 2 prior attempts in the consult. Log the consult outcome to progress.txt. This is non-negotiable — re-flailing on the same wrong root cause is the highest-cost failure mode of this loop.
- For OBVIOUSLY-different hypotheses (different root cause each attempt), the gate doesn't trigger — but be honest with yourself; "I changed the log line and re-ran" is not a new hypothesis.

PER-ITERATION MANDATES (BEFORE writing any code):
1. SKILL DISCIPLINE — if your selected task touches Isaac Sim 5.0, OpenUSD, OmniGraph, articulation, sensors, MCP socket, or any aic-dt extension internals, you MUST invoke the `isaac-sim-extension-dev` skill AND the `nvidia-suite-docs` skill BEFORE writing or editing code. NO EXCEPTIONS.
2. SIM PRECONDITION — if your selected task's `verification_mode` includes "sim", check `nc -z localhost 8768`. If down, attempt the **zenoh-canonical** launch (this project uses unified zenoh transport per closed zenoh-path-implementation task — without these env vars Isaac Sim's rclpy defaults to cyclonedds/fastrtps and the host engine pipeline can't discover /clock, breaking any trial-fire verify):
       # First ensure host zenohd is up (idempotent):
       bash exts/aic-dt/scripts/launch_host_zenohd.sh
       # Then launch sim with zenoh RMW + ROS_DOMAIN_ID=7 + zenoh peer config:
       bash -c 'source /opt/ros/humble/setup.bash && source ~/env_isaaclab/bin/activate && export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CHECK_ATTEMPTS=-1 ZENOH_CONFIG_OVERRIDE='\''connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'\'' DISPLAY=${DISPLAY:-:0} && bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'
   then poll `nc -z localhost 8768` for up to 90 seconds. If still down, append a one-line blocker note to plans/progress.txt and exit normally (no commit). SIM RESTART DISCIPLINE — sim stays running across iterations; call `quick_start(...)` over MCP to reset state. DO NOT kill+relaunch the sim unless an iteration explicitly diagnoses sim-state corruption. If sim is up but on the WRONG RMW (cyclonedds/fastrtps when zenoh is needed for engine-pipeline verification), surface as a diagnostic in progress.txt — do NOT silently kill+relaunch the sim, leave it to the orchestrator.
3. CROSS-REPO POLICY — edits anywhere outside /home/aaugus11/Documents/isaac-sim-mcp (especially ~/Documents/aic and ~/IsaacSim-ros_workspaces/) are PERMITTED in service of correctly setting up the Isaac Sim path. PROHIBITED: any edit that artificially makes trials pass without legitimate Isaac Sim integration — no disabling assertions in CheatCode/aic_controller/aic_engine/aic_adapter, no relaxing tolerances inside aic source, no mocking out scoring logic. Every edit outside isaac-sim-mcp MUST be appended (do not rewrite) as one JSON entry to plans/cross_repo_changes.json.
4. SCENE AUTHORING SOURCE OF TRUTH — when authoring any scene shortcut / atom default / robot init / sensor wiring / prim authoring / frame_id / topic name / RPY / mass / density / drive gain, FIRST consult the AIC Gazebo equivalent in ~/Documents/aic/. Mirror Gazebo; do not invent Isaac-Sim-idiomatic defaults.
5. WEB-FIRST DISCIPLINE — when your task involves debugging an error message, unexpected behavior, an interop question (RMW, ABI, zenoh, ROS message versions, kilted-humble compat, DDS-zenoh bridging, RPC stalls, transport mismatches), or any pattern other engineers have likely hit, BEFORE attempting more than ONE hypothesis-test commit you MUST:
   (a) Use WebSearch with concrete queries built from exact symptoms — examples: `"<exact error string>"`, `<framework> <symptom> site:discourse.ros.org`, `<framework> <symptom> site:github.com issues`, `<package> <symptom> site:forums.developer.nvidia.com`, `rmw_zenoh GetState timeout`, `kilted humble service RPC stall`. Try 2-3 query variations.
   (b) Use WebFetch on top 2-3 relevant results — read existing GitHub issues, Discourse threads, forum posts. Look for: same error → known cause → known fix.
   (c) Log searches in plans/progress.txt (queries + 1-line summary of findings). If you found a known solution, cite the URL in the commit body and note whether it applies + why.
   (d) NVIDIA stack questions → use `nvidia-suite-docs` skill (covers NVIDIA Developer Forums). ROS / zenoh / general non-NVIDIA → WebSearch + WebFetch directly.
   This is NOT optional for debug-shaped tasks. The most common form of wasted iterations is re-deriving a fix already documented in the wild. Web-first beats first-principles for known-interop / known-bug patterns. Skip ONLY if pure new authoring (no error to search) — e.g. parity-report-tool, ship-paperwork.

PRE-FLIGHT (BEFORE task selection):
- Run `git status --porcelain`. If uncommitted changes exist from a prior iteration: read them + the last 3 progress.txt entries, then explicitly (a) continue the prior task (only if its passes is still false and you can trace the file changes to it), or (b) `git stash` (do NOT checkout/delete) and pick a fresh task. Log your choice to progress.txt before proceeding.

BLOCKER DETECTION (before task selection):
- Read the last 5 entries of plans/progress.txt. If a single architectural blocker (e.g. "model-zenoh", "RPC stall", "phase-4-03", "kilted-humble ABI", or any repeated-failure pattern needing operator design decisions) is mentioned 2+ times AND there is NO existing PRD task targeting it (no passes:false task whose description / files_likely / verify_command speaks to that blocker):
  - Output on the LAST line of stdout: <promise>BLOCKED reason="<short, <=80 chars>"</promise>
  - Driver detects BLOCKED, halts the loop, surfaces to operator for intervention.
- DO NOT emit BLOCKED if a PRD task already targets the blocker — pick THAT task.
- DO NOT emit BLOCKED for transient/retry-able situations (sim socket down, single failed verify, missing recipe). BLOCKED is for genuinely-impassable architectural blockers requiring operator decision.

TASK SELECTION:
1. Parse plans/prd.json. Find ALL tasks with `passes: false`. Flat kanban — pick freely from the entire pool, no inter-task gates.
2. Read each candidate's `description`. Some reference others informally (e.g., "uses the wrapper extended in wrapper-json-emit"). If you have strong reason to believe a task's informal prerequisite isn't in place, prefer a task whose path to success is clearer.
3. Pick the one YOU decide has the highest priority — NOT necessarily first in list, NOT alphabetical. Prioritize: critical bugfixes > development infrastructure > tracer bullets > polish > refactors.
4. ONLY WORK ON A SINGLE TASK.

IMPLEMENTATION:
1. Read the selected task's `description`, `acceptance`, `verify_command`, `verification_mode` carefully.
2. Use TDD where applicable. For sim-bound tasks, the `verify_command` IS the gate — run it, observe, iterate.
3. Make minimal targeted edits. Do not refactor unrelated code.

VERIFY:
1. Run the task's `verify_command` exactly as specified.
2. Confirm every `acceptance` item is observably true.
3. If verify fails: do NOT commit, do NOT flip passes. Append diagnostic to progress.txt and exit. Next iteration retries.

REGRESSION SWEEP (AFTER your verify passes, BEFORE committing):
1. Read your selected task's `files_likely` array.
2. Find every other task in plans/prd.json with `passes: true` whose `files_likely` overlaps with yours (any shared file path or shared parent directory).
3. For each overlapping passes:true task, re-run its `verify_command`. If it fails:
   - DO NOT commit your work.
   - Edit plans/prd.json: flip the regressed task's `passes` from true back to false.
   - Append to plans/progress.txt: "REGRESSION: <regressed_task_id> broken by your work on <your_task_id> — <which acceptance item failed, actual vs expected>".
   - Exit (no commit). Next iteration sees both your task AND the regressed task as passes:false.
4. If no regression: proceed to ON SUCCESS.

ON SUCCESS:
1. Edit plans/prd.json — flip your selected task's `passes: false` to `passes: true`. Modify ONLY that field on that task.
2. APPEND (do not rewrite) to plans/progress.txt: timestamp, task id, key decisions, files changed, blockers/notes. Sacrifice grammar for concision.
3. Make ONE git commit per git repository touched, each referencing the same task_id, Conventional Commits style. PRIMARY commit lives in /home/aaugus11/Documents/isaac-sim-mcp covering plans/prd.json + plans/progress.txt + plans/cross_repo_changes.json + any aic-dt code. If you edited ~/Documents/aic, cd there for a SEPARATE commit; write its sha back into the matching cross_repo_changes.json entry. NO Co-Authored-By. NO --no-verify. NO destructive git ops.

4. COMMIT BODY SKILL CITATION DISCIPLINE: For tasks whose `verification_mode` is "sim" or "sim+docker" OR whose `skills_required` array in prd.json is non-empty, the commit body MUST include one line per consulted skill in this exact format:
       Skill cited: <skill_name> for <one-line concrete use, e.g. "Articulation _physics_view lifecycle pattern" or "PhysX 5.0 kinematic+RigidBodyAPI contact-report semantics">
   If you genuinely did not consult any skill (e.g. the change was a trivial typo / log-message tweak / pure cleanup with no API surface touched), include exactly one line: "Skill cited: none — <one-line reason>". Absence of either form is a discipline failure — append a note to plans/progress.txt flagging it for operator review.

If after this iteration EVERY task in plans/prd.json has `passes: true`, output on the LAST line of stdout: <promise>COMPLETE</promise>

Do NOT use any other sentinel. If genuinely stuck on an external dep, append a note to progress.txt and exit normally.
PROMPT_EOF
)

# ─── VERIFIER PROMPT ──────────────────────────────────────────────────────────
VERIFIER_PROMPT=$(cat <<'PROMPT_EOF'
@plans/prd.json @plans/progress.txt @CLAUDE.md

You are the VERIFIER agent. The EXECUTOR agent just finished one iteration. Your job: independently confirm the executor's claim of completion in a FRESH context. You did not write the code; you did not flip the passes flag. Verify it as if you have no idea whether the executor was honest.

STUCK ESCAPE VALVE (the /ask-gpt rule):
- If your verification produces an AMBIGUOUS outcome (verify_command exit code disagrees with acceptance-item observability, OR you cannot decide between CASE A and CASE B because the executor's claim is technically correct but the underlying behavior is suspect), invoke the `ask-gpt` skill for an adversarial second opinion BEFORE choosing CASE A/B/C. Include the verify_command output, the acceptance items, and your concern. This prevents false-VERIFIED on regressions disguised as passes (the 2026-05-08 PARITY-05 wrench all-zeros lesson).

DISCIPLINE:
1. Run `git log -1 --format=%H%n%s%n%b` to inspect the most recent commit. Parse the conventional-commits scope to find the task_id (e.g. "feat(parity-report-tool):" → task_id = "parity-report-tool"). If the commit message lacks a parseable task_id, OR if the most recent commit is not a fresh executor result (e.g. it's a previous verifier's chore(verify) commit, or no new commit since prior iteration), exit normally — there is nothing to verify.
2. Find that task in plans/prd.json. Confirm passes is true (compare against the previous commit's prd.json via `git show HEAD~1:plans/prd.json` if you want to be sure the executor flipped it).
3. SKILL DISCIPLINE — if the task's `verification_mode` is "sim" or "sim+docker" (or otherwise touches Isaac Sim API surface), invoke isaac-sim-extension-dev + nvidia-suite-docs BEFORE running anything. Same rule as executor.
4. SIM PRECONDITION — if verification_mode includes "sim", check `nc -z localhost 8768`. Do NOT launch (executor's iter should have left it running). If down: append "VERIFY-INCONCLUSIVE <task_id> — sim socket down, can't verify" to plans/progress.txt, make ONE doc-only commit `chore(verify): inconclusive — <task_id>`, exit normally.

VERIFICATION:
1. Run the task's `verify_command` EXACTLY as written in prd.json. Don't substitute, don't simplify, don't skip steps.
2. Read each item in the task's `acceptance` array. Independently confirm each is observably true RIGHT NOW. For sim-bound items: query live MCP / ros2 topic state. For docker-bound items: re-run the container check. For unit-test items: re-run the script.
3. Compare actual output against acceptance criteria.

OUTCOME (one of three):

CASE A — Verification passes (every acceptance item observably true, verify_command exits 0):
- Append to plans/progress.txt: "VERIFIED <task_id> by verifier-agent at <ISO 8601 ts> — <which gates passed, brief>".
- Make ONE git commit: `chore(verify): <task_id> verified — <one-line>`.
- COMMIT BODY SKILL CITATION DISCIPLINE: For sim or sim+docker verification, the commit body MUST include one line per consulted skill: "Skill cited: <skill_name> for <one-line use>". If verification was simple enough not to need a skill, include "Skill cited: none — <reason>".
- Exit normally.

CASE B — Verification fails (any acceptance item not observable, OR verify_command exits non-zero):
- Edit plans/prd.json: flip that task's `passes` from true back to false.
- Append to plans/progress.txt: "VERIFY-FAIL <task_id> by verifier-agent at <ISO 8601 ts> — <which acceptance gate failed, what actual output was, what was expected>". Capture enough diagnostic that the NEXT executor iter can fix it without re-deriving the failure.
- Make ONE git commit: `revert(verify): <task_id> failed verification — <one-line>`.
- DO NOT attempt to fix the underlying code. The next executor iteration's job is to fix.
- Exit normally.

CASE C — Inconclusive (sim down, dependency missing, can't reproduce environment):
- Append to plans/progress.txt: "VERIFY-INCONCLUSIVE <task_id> by verifier-agent at <ISO 8601 ts> — <why>".
- Make ONE doc-only commit: `chore(verify): inconclusive — <task_id>`.
- Exit normally. Operator should resolve the inconclusive condition before next AFK iter.

ONLY VERIFY THE MOST RECENT TASK. Do not re-verify other passes:true tasks. Do not pick a new task. Do not start new work. You are the verifier; your scope is one task.
PROMPT_EOF
)

# ─── EXECUTOR PASS ────────────────────────────────────────────────────────────
echo "[tug] === EXECUTOR pass — interactive TUI ==="
echo "[tug] When the agent finishes (or you exit the TUI), the VERIFIER pass starts."
echo
claude $PERMISSION_FLAGS "$EXECUTOR_PROMPT"

# ─── VERIFIER PASS ────────────────────────────────────────────────────────────
echo
echo "[tug] === VERIFIER pass — independent re-verification ==="
echo "[tug] Verifier reads the most recent commit, finds the task_id, and re-runs verify_command in a fresh context."
echo
claude $PERMISSION_FLAGS "$VERIFIER_PROMPT"

echo
echo "[tug] HITL iteration complete (executor + verifier)."
echo "[tug] Inspect plans/prd.json + progress.txt + git log -2 to see the result."
