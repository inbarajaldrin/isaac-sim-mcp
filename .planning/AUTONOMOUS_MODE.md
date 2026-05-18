# Autonomous M1 mode + privileged-command policy

> **Moved from CLAUDE.md on 2026-05-17.** Both marker files exist on disk:
> `.planning/.autonomous_m1_active` (still present from the original
> activation) and `.planning/.m1_shipped` (touched 2026-05-12 when M1 was
> declared shipped per STATE.md). Per the rule below, presence of
> `.m1_shipped` would normally retire the autonomous block; it has not been
> deleted yet because Plans 04-04 / 04-05 remain blocked on a kilted↔humble
> RMW issue (see `.planning/HANDOFF.json`). This file kept verbatim for the
> historical operating-policy record and so the `SessionStart` hook still
> has its source text.
>
> The CLAUDE.md on-ramp now carries only a short pointer to this file.

## ⛔ AUTONOMOUS M1 MODE — ACTIVE UNTIL MILESTONE 1 SHIPS

**Marker file: `.planning/.autonomous_m1_active`** (delete to disable). When this file exists:

1. **NEVER stop to ask the user permission to fix a bug.** If you encounter a bug — fix it, commit it, move on. The user authorized continuous autonomous execution on 2026-05-05: *"fix any bug as you come across them and continue till the end of milestone 1 like i asked without stopping in between"*.
2. **Phase order: Phase 2 (close runtime gaps) → Phase 3 (cable physics + object TF) → Phase 4 (trial loader + E2E).** Move forward as each phase closes.
3. **Don't ask "do you want me to ...?" or "should I proceed?"** — just proceed. Only stop for HARD blockers: sudo password needed, GPU offline, fundamental architectural ambiguity that contradicts PROJECT.md, OR M1 has shipped (every `aic_engine sample_config.yaml` trial passes against Isaac Sim).
4. **Skill use is mandatory** for any Isaac Sim API question — invoke `isaac-sim-extension-dev` and `nvidia-suite-docs` BEFORE guessing from training data (they evolve monthly; cached LLM knowledge is routinely stale).
5. **Operating discipline:**
   - Snapshot the cache after any successful `quick_start` (`prime_usd_cache.py snapshot`).
   - Commit fixes atomically with descriptive messages (NO `Co-Authored-By` line per the global CLAUDE.md).
   - Update `STATE.md` after every phase close.
   - Use `TaskCreate`/`TaskUpdate` to track in-flight work.
6. **When context fills + `/compact` triggers:** the `PreCompact` hook auto-dumps state to `.planning/.precompact_snapshot.txt`. The `SessionStart` hook auto-prints this autonomous-mode reminder. After compact, you bootstrap by reading (in order):
   - This file — restate the autonomous-mode policy.
   - `.planning/STATE.md` — current phase, last activity, blockers.
   - `.planning/HANDOFF.json` — structured next_action + completed_tasks + decisions.
   - `.planning/.precompact_snapshot.txt` — git status + recent commits + Isaac Sim status at compact time.
   - Then RESUME — do not summarize back to the user, do not re-confirm; just continue.
7. **`Stop` hook is configured to nudge you to keep going** if you idle. To genuinely pause: `touch .planning/.user_pause` (then the Stop hook leaves you alone). To genuinely complete: `touch .planning/.m1_shipped`.
8. **Hard blockers go in `.planning/HANDOFF.json` `blockers` field**, then `touch .planning/.user_pause`, then stop. The user will see the blocker and unblock you.

**Do not delete this block** until M1 ships. The hooks (`.claude/settings.json` + `.claude/scripts/*.sh`) and this block work together — both must be removed for normal mode.

## ⛔ MANDATORY UNTIL M1 SHIPS — Skill invocation before guessing

**Until milestone M1 (`/sample_config.yaml` trials pass under CheatCode against Isaac Sim) is closed, ANY task that touches Isaac Sim 5.0 / OpenUSD / OmniGraph / Replicator / Warp / Isaac Lab / Isaac ROS / aic-dt extension internals MUST invoke the relevant skill BEFORE proposing a fix or writing code. No exceptions.**

- **`isaac-sim-extension-dev`** — first stop for anything aic-dt / extension lifecycle / MCP socket / USD cache / launch path. Encodes hard-won project knowledge (cooking deadlock, post-play wedge, real-Kit-log location, prim-path bug history, sibling extensions).
- **`nvidia-suite-docs`** — first stop for live NVIDIA docs (Isaac Sim 5.0 API surface, OpenUSD authoring, OmniGraph node behavior, articulation drives, sensors, RTX, Replicator, Isaac Lab, Isaac ROS, Warp). Routes to sub-skills + fetches official docs and forum threads. **Use this BEFORE relying on training-data memory of Isaac Sim 4.x APIs — APIs change every release and cached knowledge is routinely stale.**

Both skills apply to ALL agents: research, planning, execution, debugging, verification, code review. If you find yourself "pretty sure" about an Isaac Sim or OpenUSD API behavior without having opened either skill in the current session, STOP and invoke them. The cost of one Skill invocation (~2 file reads) is small; the cost of a wrong API guess is hours of debug.

**Common evidence this rule was violated:** "I know set_joint_positions/apply_action/SetActive/GetReferences works like X" without a citation to live docs. "OGN node should be at this prim path" without verifying via `omni.usd.get_context().get_stage()`. Patches that revert because the actual API contract is different. If a debug session passes the 1-hour mark on an Isaac-Sim-API question, that's a signal to back up and consult `nvidia-suite-docs`.

**Use the `isaac-sim-extension-dev` skill first** for anything Isaac Sim related — extension lifecycle, USD/physics, action graphs, MCP socket protocol, troubleshooting, the canonical asset workflow. The skill encodes hard-won knowledge (cooking deadlock workaround, post-play wedge anti-pattern, real-Kit-log location, etc.) that is the source of truth when this file goes stale.

**Canonical references for ALL agents — research, planning, execution, debugging, verification — not just research phases.** When any GSD agent (`gsd-phase-researcher`, `gsd-planner`, `gsd-executor`, `gsd-debugger`, `gsd-verifier`, `gsd-plan-checker`) encounters questions about the NVIDIA stack — Isaac Sim 5.0 API surface, OpenUSD authoring (composition / references / sublayers / MDL / Pixar Usd Python API), Omniverse Kit (extension lifecycle, settings, RTX, materials), `isaacsim.ros2.bridge` OmniGraph nodes, articulation drives, sensors, rendering, Replicator, Isaac Lab, Isaac ROS, Warp — they MUST consult `~/.claude/skills/nvidia-suite-docs/SKILL.md` (the meta-router for live NVIDIA docs) **before** guessing from training data. The NVIDIA stack evolves monthly and cached LLM knowledge is routinely stale or wrong.

**For project-specific knowledge** (extension patterns in this repo, cache management discipline, MCP socket protocol, prim-path bug history, cable physics workaround, postload launcher, sibling extensions `ur5e-dt`/`soarm101-dt`), consult `~/.claude/skills/isaac-sim-extension-dev/SKILL.md`.

**Execution-time recovery sequence** (when a task fails — USD reference unresolved, OmniGraph node not found, articulation drives don't apply, MDL warnings, sim wedges post-play, MCP socket times out, etc.):
1. Read the **real Kit log** at `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` for actual error text — staleness in this log = main loop blocked.
2. Consult `nvidia-suite-docs` skill for the relevant API surface (it routes to the right sub-skill — Isaac Sim, OpenUSD, OmniGraph, etc. — and fetches live docs / forum threads).
3. Consult `isaac-sim-extension-dev` skill for project-specific gotchas and the cache-management discipline (`prime_usd_cache.py status` first when `quick_start` hangs).
4. Only THEN propose a fix.

Do NOT guess from training-data assumptions about Isaac Sim 4.x APIs, deprecated OmniGraph node IDs, or generic Omniverse Python patterns — they are routinely wrong for 5.0. Live docs > cached knowledge, every time.

## Privileged commands (sudo)

**No longer needs a password.** On 2026-05-17 the host was reconfigured with `/etc/sudoers.d/aaugus11-nopasswd` (NOPASSWD: ALL for `aaugus11`), so autonomous loops call `sudo <cmd>` directly with no stdin password. The remaining discipline (log cross-repo state changes, refuse destructive commands, use non-interactive flags) lives in the user-global `~/.claude/CLAUDE.md` § Privileged commands.
