---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: executing
stopped_at: Plan 01-03 complete (RG2→Robotiq Hand-E doc correction; 12 occurrences across 5 files)
last_updated: "2026-05-02T12:34:00.000Z"
last_activity: 2026-05-02 -- Plan 01-03 complete; doc surface uniformly identifies gripper as Robotiq Hand-E
progress:
  total_phases: 4
  completed_phases: 0
  total_plans: 9
  completed_plans: 3
  percent: 33
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-01)

**Core value:** When the same `aic_controller` + `aic_example_policies/CheatCode.py` + `sample_config.yaml` that pass AIC trials in Gazebo are pointed at this Isaac Sim digital twin, every trial passes with the same outcome.
**Current focus:** Phase 1 — Foundation Parity

## Current Position

Phase: 1 (Foundation Parity) — EXECUTING
Plan: 4 of 9
Status: Executing Phase 1 — Plans 01-01, 01-02, 01-03 complete
Last activity: 2026-05-02 -- Plan 01-03 complete; RG2→Robotiq Hand-E correction landed across .planning/, CLAUDE.md, exts/aic-dt/docs/

Progress: [███░░░░░░░] 33%

## Performance Metrics

**Velocity:**

- Total plans completed: 3
- Average duration: 4 min
- Total execution time: 12 min

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| Phase 1 | 3 | 12 min | 4 min |

**Recent Trend:**

- Last 5 plans: 01-01 (7 min), 01-02 (~2 min), 01-03 (3 min)
- Trend: pure-doc correction plan; 2 tasks; 0 auto-fixes (deterministic Edit-tool replacements, all acceptance criteria first-pass; 1 issue handled cleanly = stash-and-restore for unrelated dirty CLAUDE.md state)

*Updated after each plan completion*

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table. Recent decisions affecting current work:

- Repo split: sim-side here, ROS-side / controller / policies / engine in `~/Documents/aic` (read-only)
- Topic parity with Gazebo (no `_sim`/`_real` suffixes, no bridges) is the architectural law
- Same USD/URDF as Gazebo (`aic_description`, `aic_assets`); zero-divergence guarantee
- Pose-source seam = `/tf` (M1 publishes GT into `/tf`; M2 swaps publisher only)
- Cable physics strategy deferred to Phase 3 research (use `nvidia-suite-docs` skill)
- M1 success bar = all `sample_config.yaml` trials pass under CheatCode against Isaac Sim
- MCP atomic + clubbed model preserved (`MCP_TOOL_REGISTRY` + `_cmd_<name>` + per-tool button + `quick_start` clubbing)
- Plan 01-01: image digest captured = `sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433` (D-14 pin)
- Plan 01-01: live aic_eval surface is 36 topics (not 35; +2 controller motion_update topics drift since research)
- Plan 01-01: `docker inspect` for RepoDigests must target the image, not the running container — running containers may have empty `.RepoDigests` field
- Plan 01-02: vendored 5 capitalized AIC asset folders (`NIC Card`, `NIC Card Mount`, `SC Port`, `SC Plug`, `Task Board Base`) under `exts/aic-dt/assets/assets/` — 21 files total (9 USDs + 12 textures), md5sum byte-identical to upstream
- Plan 01-02: snake_case `exts/aic-dt/assets/objects/` retired in same plan; production extension is broken-by-design between this plan and Plan 04 lands (intentional sequencing per the plan)
- Plan 01-02: `cp -r` whole-folder is the vendoring contract — selective `.usd`-only copy reintroduces TEX-01 (missing sibling textures)
- Plan 01-02: AIC's CAPITALIZED layout (with spaces) preserved verbatim — USDs reference `./textures/...` relatively; renaming would re-break TEX-01 without USD rewrites
- Plan 01-03: AIC URDF xacro (`~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro`) is the canonical gripper-identification source — it includes `Robotiq Hand-E/robotiq_hande_macro.xacro`, NOT RG2; doc surface (`.planning/`, `CLAUDE.md`, `exts/aic-dt/docs/`) corrected accordingly
- Plan 01-03: ROADMAP.md plan-description lines naming this plan reworded from "RG2→Robotiq Hand-E correction" to "gripper-name correction (Robotiq Hand-E)" — keeps zero literal `RG2` tokens in the doc surface
- Plan 01-03: Stash-and-restore pattern used to keep pre-existing uncommitted CLAUDE.md edits out of plan commits — only RG2 hunks landed in commit `397b530`, unrelated dirty state preserved as before
- Plan 01-03: PARITY-01 doc wording corrected; actual implementation (URDF load) is downstream — PARITY-01 status remains `[ ]` until code path lands (likely 01-04/01-06)

### Pending Todos

None yet.

### Blockers/Concerns

- Cable physics fidelity strategy is the highest-risk requirement (SCENE-05) — research must land in Phase 3 before cable spawn / gripper-attach implementation can converge.

## Deferred Items

Items acknowledged and carried forward from previous milestone close:

| Category | Item | Status | Deferred At |
|----------|------|--------|-------------|
| *(none — M1 is the first milestone)* | | | |

## Session Continuity

Last session: 2026-05-02T12:34:00.000Z
Stopped at: Plan 01-03 complete — RG2→Robotiq Hand-E doc correction across .planning/, CLAUDE.md, exts/aic-dt/docs/README.md
Resume file: .planning/phases/01-foundation-parity/01-04-PLAN.md (extension.py renames + code-side gripper-name correction + prim-path bug fix + AIC_OBJECTS update + DX-02 deletion-side + PARITY-05 wrench)
