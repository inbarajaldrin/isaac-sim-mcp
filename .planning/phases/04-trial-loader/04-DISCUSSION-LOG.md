# Phase 4: Trial Loader & End-to-End Verification — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in 04-CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-05
**Phase:** 04-trial-loader
**Mode:** `--auto` (autonomous M1) — gray areas auto-selected, recommended options chosen without `AskUserQuestion` per workflow contract.
**Areas discussed:** Trial Loader Surface, Ground-Truth Flag, E2E Launch Glue, Parity-Report Script, M1 Ship Gate, DX-05 + Phase-3 carry-forward

---

## Trial Loader Surface

| Option | Description | Selected |
|--------|-------------|----------|
| Single `load_trial(config_path, trial_key, ground_truth)` MCP atom | One atom + one UI button; trial_key is runtime arg; reuses existing 9 spawn atoms + load_robot kwargs | ✓ |
| Per-trial MCP atoms auto-generated from YAML | Cleaner DX (one button per trial); but pulls YAML parsing into UI rendering | |
| Generic `apply_yaml(config_path, trial_key, schema_version)` | Most flexible; over-engineered for current need (3 trials, fixed schema) | |

**User's choice:** Option 1 (auto-recommended). DX-02 4-surface contract minimal; trial selection is naturally runtime; per-trial buttons noted as deferred.
**Notes:** Adapter dict in `extension.py` maps YAML keys to existing spawn atom kwargs — no new spawn surface authored. `quick_start` clubbing pattern is the proven order; `load_trial` is `quick_start` parameterized.

---

## Ground-Truth Flag

| Option | Description | Selected |
|--------|-------------|----------|
| Kwarg on `load_trial` + `quick_start` (default `True`) | Mirrors Gazebo's `ground_truth:=true/false` launch arg semantics; consistent with DX-02 | ✓ |
| Env var `MCP_GROUND_TRUTH=1` | Single switch; splits state across two surfaces (env + kwarg fallback) | |
| New MCP atom `set_ground_truth_mode(enabled)` | Runtime toggle; risks mid-trial state divergence between aic_engine + Isaac Sim | |

**User's choice:** Option 1 (auto-recommended).
**Notes:** Gates `_start_aic_scoring_publishers()` invocation. Robot frames + `/joint_states` + `/tf` (joint chain) still publish; cable + object TF + `/scoring/*` topics do not. M2 swap surface preserved.

---

## E2E Launch Glue

| Option | Description | Selected |
|--------|-------------|----------|
| Wrapper script `run_aic_engine_against_isaac_sim.sh` mirroring `run_cheatcode.sh` | Single concrete invocation surface; reuses isaacsim_launch.sh + Docker patterns | ✓ |
| Python orchestrator that spawns Isaac Sim + Docker via subprocess | More cross-platform but adds Python dependency for what's already shell-friendly | |
| Manual launch sequence documented in DX-05 README | Lowest dev cost; but TRIAL-03/04 verification needs automation | |

**User's choice:** Option 1 (auto-recommended).
**Notes:** Args `<trial_key>` `[--ground-truth=true|false]` `[--clean]`. Reuses warm Isaac Sim if `isaacsim_launch.sh status` is healthy; only launches if absent. After `aic_engine` exits, container is stopped; Isaac Sim left running unless `--clean`.

---

## Parity-Report Script

| Option | Description | Selected |
|--------|-------------|----------|
| Single Python script emits BOTH JSON snapshot + markdown table | Diff-able + human-readable from one source of truth | ✓ |
| Markdown only | Human review only; not diff-able for CI | |
| JSON only + separate viewer | Two surfaces to maintain; risks drift | |

**User's choice:** Option 1 (auto-recommended).
**Notes:** Output: `.planning/phases/04-trial-loader/parity-report.{json,md}`. Per-trial outcome row is `{trial_key, sim, insertion_event_fired, off_limit_count, duration_s, pass_fail}`. `pass_fail = insertion_event_fired AND off_limit_count == 0`.

---

## M1 Ship Gate

| Option | Description | Selected |
|--------|-------------|----------|
| Zero mismatch rows in `parity_report.py` across all 3 trials → `touch .planning/.m1_shipped` | Concrete, automatable, deterministic ship gate | ✓ |
| Manual review of side-by-side outcomes by user | Subjective; doesn't gate the autonomous-mode flag flip | |
| Per-trial outcome match + extended scoring (force trajectories, durations) | Tier-2 scoring scope; M2 work | |

**User's choice:** Option 1 (auto-recommended).
**Notes:** Aligns with REQUIREMENTS.md success criteria 3+4. Autonomous mode block at top of CLAUDE.md remains in place as historical artifact — `.autonomous_m1_active` marker can be removed manually post-ship.

---

## DX-05 (README + CHANGELOG)

| Option | Description | Selected |
|--------|-------------|----------|
| Rewrite `exts/aic-dt/docs/README.md` for AIC scope + add `M1 (2026-05-05)` CHANGELOG entry | DX-04 is closed; DX-05 mirrors at the per-extension level | ✓ |
| README minimum diff (just remove "ur5e-dt" string) | Insufficient — REQUIREMENTS spec asks for AIC scope reflection | |

**User's choice:** Option 1 (auto-recommended).

---

## Phase-3 carry-forward: PARITY-07 live insertion_event fire

| Option | Description | Selected |
|--------|-------------|----------|
| Verify incidentally during TRIAL-04 (every passing trial fires exactly one insertion_event) | Phase 3 SUMMARY explicit carry-forward; no separate harness needed | ✓ |
| Standalone `smoke_test_aic_insertion.py` with manual plug-into-port script | Adds a maintenance surface for a one-shot verification | |

**User's choice:** Option 1 (auto-recommended).
**Notes:** If a trial passes Gazebo's CheatCode but doesn't fire `insertion_event` against Isaac Sim, that's a Phase-3 contact-subscription bug surfacing through Phase 4 — Plan 04-N (E2E plan) owns the fix loop.

---

## Claude's Discretion

- Plan numbering and split: planner's call. Phase 4 has 5 distinct deliverables (load_trial atom, ground_truth flag, E2E wrapper, parity-report, README) that may want separate plans.
- Whether to ship `smoke_test_aic_trial.py` independent of `parity_report.py` — collapse if redundant.
- Exact CHANGELOG.md narrative voice — single dense bullet list per phase is fine.

---

## Deferred Ideas

- M2 pose-source swap (real-world pose estimator).
- Per-trial UI buttons auto-generated from YAML.
- Trial loader randomization layer (`seed`, `random_pose_noise_sigma`).
- Tier-2 scoring full activation (requires non-default aic_engine mode).
- Streaming wrapper (`~/bin/isaacsim`) fix for aic-dt (separate DX task).
- `parity_publishers` hot-reload regression (Phase 3 known limitation).
