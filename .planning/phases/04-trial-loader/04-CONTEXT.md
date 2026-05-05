# Phase 4: Trial Loader & End-to-End Verification — Context

**Gathered:** 2026-05-05
**Status:** Ready for planning
**Mode:** `--auto` (autonomous M1) — gray areas auto-selected, recommended option chosen for each, decisions logged inline.

<domain>
## Phase Boundary

Phase 4 wires a YAML-driven trial loader on top of the existing Phase 1–3 atomic surface, lets `aic_engine` + `CheatCode.py` run unmodified against Isaac Sim, and produces an automated parity report comparing Isaac Sim's per-trial outcomes against Gazebo's. M1 ships when every trial in `~/Documents/aic/aic_engine/config/sample_config.yaml` produces the same pass/fail outcome under CheatCode against Isaac Sim that it does under Gazebo.

**In scope (TRIAL-01..05 + DX-05):**
- `load_trial` MCP atom: parse a single trial entry from a YAML config and dispatch to existing spawn atoms (`load_robot`, `spawn_*`, `attach_cable_to_gripper`, `quick_start`).
- `ground_truth` flag: kwarg on `load_trial` / `quick_start` that gates `scoring_publishers` startup (TF off when `ground_truth=False`; M2 swap surface).
- E2E launch glue: shell wrapper that brings up Isaac Sim, runs `quick_start` + `load_trial`, then launches `aic_engine` + `CheatCode` against `ROS_DOMAIN_ID=7`.
- Parity-report script: runs each trial against both sims (Isaac Sim + Gazebo via `~/Documents/aic/scripts/run_cheatcode.sh`), captures per-trial outcomes (insertion_event fired? off_limit contact? duration?), emits markdown + JSON.
- DX-05: `exts/aic-dt/docs/README.md` rewrite (no "ur5e-dt" framing) + CHANGELOG entry for M1.
- Phase 3 carry-forward: live verification that `/scoring/insertion_event` fires during plug-into-port — verified incidentally by TRIAL-04 outcome match (passing trial publishes event; absent firing == fail).

**Out of scope (deferred to M2 or later):**
- Pose-source swap (`/tf` from real-world pose estimator instead of Isaac Sim ground-truth) — M2 work.
- Tier-2 scoring (`ScoringTier2.cc` activation requires non-default `aic_engine` mode) — `/scoring/tf` is published but full tier-2 path is not exercised.
- Randomization layers (lighting, object pose noise) beyond what the trial config specifies — Phase 1 atoms exist but are not invoked from `load_trial`.
- Streaming wrapper fix for `aic-dt` in `~/bin/isaacsim` (separate DX task, tracked in CLAUDE.md).
- Hot-reload regression on `parity_publishers` rclpy state (Phase 3 known limitation).
</domain>

<decisions>
## Implementation Decisions

### Trial Loader Surface
- **D-01:** Single MCP atom `load_trial(config_path, trial_key, ground_truth=True)` — `config_path` defaults to `~/Documents/aic/aic_engine/config/sample_config.yaml`, `trial_key` is the YAML key (e.g. `"trial_1"`, `"trial_2"`, `"trial_3"`). Returns `{"trial": ..., "spawned_components": [...], "ground_truth": True/False}`. Reuses the per-component spawn atoms from Plan 01-09 + `load_robot` from Phase 3 — no new spawn surface authored.
  - **Rationale:** auto-selected (recommended). Single atom keeps the DX-02 4-surface contract minimal (registry + handler + `_cmd_load_trial` + UI button) and trial selection is naturally a runtime choice, not a config-time choice. Per-trial UI buttons would force scope creep (3 buttons today, N tomorrow).
- **D-02:** `load_trial` is fully self-contained — internally calls `new_stage` → `load_scene` → `load_robot(...)` → 7 `spawn_*` atoms with trial-config-derived kwargs → `add_objects` → `setup_*` chain → `_start_aic_parity_publishers` → `_start_aic_scoring_publishers` (gated on `ground_truth`) → `play_scene`. Mirrors `quick_start`'s clubbing pattern but with config-driven parameters instead of hardcoded defaults.
  - **Rationale:** auto-selected (recommended). Idempotent + deterministic — fresh stage per trial avoids drift between back-to-back trials. The `quick_start` chain is the proven order; `load_trial` is `quick_start` parameterized.
- **D-03:** YAML parsing uses Python's `yaml.safe_load` from the stdlib's PyYAML. The `aic_engine`'s `sample_config.yaml` `trials.<trial_N>.scene.task_board` block maps to spawn-atom kwargs via a small adapter dict in `extension.py` — `nic_rail_<i>` → `spawn_nic_card_mount(index=i, present=..., translation=..., rpy=...)`, `sc_rail_<i>` → `spawn_sc_port(...)`, `<lc|sfp|sc>_mount_rail_<i>` → `spawn_<lc|sfp|sc>_mount_rail(...)`, `task_board.pose` → `spawn_task_board_base(x,y,z,roll,pitch,yaw)`. Cable pose comes from `trials.<trial_N>.scene.cable.{x,y,z,roll,pitch,yaw,cable_type,attach_to_gripper}` if present, else falls back to `load_robot`'s Phase-3 defaults.
  - **Rationale:** auto-selected (recommended). PyYAML is already a dep of any ROS-adjacent Python; no new third-party. Adapter dict makes the mapping legible and testable.

### Ground-Truth Flag
- **D-04:** `ground_truth` is a kwarg on `load_trial` AND `quick_start` (default `True`). When `False`, `_start_aic_scoring_publishers()` is NOT invoked — only `_start_aic_parity_publishers()` runs. Robot frames + `/joint_states` + `/tf` (joint chain) still publish; cable + object TF + `/scoring/*` topics do not.
  - **Rationale:** auto-selected (recommended). Mirrors Gazebo's `ground_truth:=true/false` launch arg semantics. Kwarg is the DX-02-consistent surface; env var would split state across two surfaces.
- **D-05:** No new MCP atom for runtime toggle — flag is fixed at `load_trial` / `quick_start` invocation. To switch, re-run `load_trial(..., ground_truth=False)` (which calls `new_stage` first, so it resets cleanly).
  - **Rationale:** auto-selected (recommended). Avoids mid-trial state divergence between `aic_engine` and Isaac Sim. M2 swap surface is preserved (the `_start_aic_scoring_publishers` gate is the seam where a real-world pose source plugs in).

### E2E Launch Glue
- **D-06:** New shell script `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh` brings up Isaac Sim + runs `quick_start` + `load_trial(trial_key=$1)` via the MCP socket, then launches the AIC repo's `aic_engine` + `CheatCode` Docker container against `ROS_DOMAIN_ID=7` (mirroring `~/Documents/aic/scripts/run_cheatcode.sh`'s pattern). Args: `<trial_key>` `[--ground-truth=true|false]`.
  - **Rationale:** auto-selected (recommended). The wrapper script is the natural mirror of `run_cheatcode.sh` (Gazebo path) and gives TRIAL-03 a single concrete invocation surface. Keeps Isaac Sim's own launcher (lifecycle helper or postload) unchanged.
- **D-07:** Wrapper checks for an existing healthy Isaac Sim PID via `isaacsim_launch.sh status` and reuses it; only launches if absent. After `aic_engine` exits, container is stopped via `docker stop aic_eval aic_model`; Isaac Sim is left running for the next trial unless `--clean` is passed.
  - **Rationale:** auto-selected (recommended). Trial cycle is fast when Isaac Sim stays warm (~5s `new_stage`+`load_trial` vs ~30s cold launch). `parity_publishers` hot-reload regression is a separate hazard that doesn't apply here (we're not touching `extension.py`).

### Parity-Report Script
- **D-08:** New script `exts/aic-dt/scripts/parity_report.py` (Python) iterates `sample_config.yaml`'s trials, runs each against Isaac Sim AND Gazebo back-to-back, captures `/scoring/insertion_event` fires + `/aic/gazebo/contacts/off_limit` events + trial duration via short-lived `rclpy` subscribers, and emits BOTH a JSON snapshot (`parity-report.json`) AND a human-readable markdown table (`parity-report.md`) to `.planning/phases/04-trial-loader/`.
  - **Rationale:** auto-selected (recommended). Dual output: JSON for re-runnable / diff-able snapshots, markdown for human review during the M1 ship gate review. Single script avoids drift between two ad-hoc viewers.
- **D-09:** Per-trial outcome row is `{trial_key, sim, insertion_event_fired, off_limit_count, duration_s, pass_fail}`. `pass_fail` is computed as `insertion_event_fired AND off_limit_count == 0`. Side-by-side delta shown in markdown table; mismatch rows are flagged with `🔴` for ship-gate review.
  - **Rationale:** auto-selected (recommended). Mirrors the AIC scoring rubric — insertion success + no off-limit contact = pass. Anything beyond that (insertion duration thresholds, force trajectories) is M2 / tier-2 work.

### M1 Ship Gate
- **D-10:** M1 ships when `parity_report.py` emits zero mismatch rows across all 3 trials in `sample_config.yaml`. On success, `touch .planning/.m1_shipped` (per autonomous-mode CLAUDE.md). The autonomous mode block at the top of CLAUDE.md remains in place as a historical artifact — the `.autonomous_m1_active` marker can be removed manually post-ship.
  - **Rationale:** auto-selected (recommended). Concrete, automatable, deterministic ship gate. Aligns with REQUIREMENTS.md success criterion 3 + 4.

### DX-05 (README + CHANGELOG)
- **D-11:** `exts/aic-dt/docs/README.md` is rewritten to remove "ur5e-dt" framing and reflect the AIC scope: UR5e + Robotiq Hand-E + cable + AIC enclosure, MCP port 8768, the cross-repo relationship, and the canonical launch sequence. CHANGELOG.md adds an `M1 (2026-05-05)` entry summarizing Phases 1–4 deliverables.
  - **Rationale:** auto-selected (recommended). DX-04 (CLAUDE.md) was closed in Phase 1 — DX-05 is the per-extension equivalent for `exts/aic-dt/docs/`.

### Carry-Forward from Phase 3
- **D-12:** PARITY-07 `/scoring/insertion_event` live runtime fire is verified incidentally during TRIAL-04 — every passing trial in `sample_config.yaml` MUST produce exactly one `/scoring/insertion_event` `std_msgs/String` message. If a trial passes Gazebo's CheatCode but doesn't fire `insertion_event` against Isaac Sim, that's a Phase-3 contact-subscription bug surfacing through Phase 4. Plan 04-N (E2E plan) owns the fix loop.
  - **Rationale:** auto-selected (recommended). The "live fire deferred to Phase 4 trial loader" note in 03-SUMMARY.md is the explicit carry-forward; no separate smoke harness needed.
- **D-13:** `_PORT_LINK_PATHS` in `scoring_publishers.py` is currently hardcoded (`/World/TaskBoard/sc_port_1`, `sc_port_2`, `nic_card`). Phase 4 may need to parameterize this from the trial config IF a trial spawns ports at non-default rail indices. First-pass: leave hardcoded and verify the 3 sample trials don't break it; if a trial does break it, Plan 04-N adds a `set_port_link_paths` MCP atom.
  - **Rationale:** auto-selected (recommended). YAGNI until the 3 sample trials prove a parameterization need. Adding the atom later is a 4-surface addition (DX-02), not an architectural change.

### Claude's Discretion
- Plan numbering and split (single 04-01 plan vs multi-plan decomposition) is the planner's call — granularity is "coarse" per `config.json`, but Phase 4 has 5 distinct deliverables that may want separate plans if they touch different surfaces.
- Whether to ship a `smoke_test_aic_trial.py` independent of `parity_report.py` — the planner can collapse if redundant.
- Exact CHANGELOG.md narrative voice — a single dense bullet list per phase is fine.
</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### AIC repo (~/Documents/aic, read-only consumer)
- `~/Documents/aic/aic_engine/config/sample_config.yaml` — trial config schema source-of-truth (3 trials × `task_board.{pose,nic_rail_<0-4>,sc_rail_<0-1>,<lc|sfp|sc>_mount_rail_<0-N>}` + `task_board_limits` + `scoring.topics`)
- `~/Documents/aic/aic_engine/` — engine source; READ but DO NOT MODIFY (architectural law)
- `~/Documents/aic/aic_example_policies/aic_example_policies/ros/CheatCode.py` — passing reference policy used by every TRIAL-04 trial
- `~/Documents/aic/scripts/run_cheatcode.sh` — Gazebo bringup pattern that `run_aic_engine_against_isaac_sim.sh` mirrors
- `~/Documents/aic/aic_bringup/launch/spawn_task_board.launch.py` — Gazebo's parameter surface for the spawn atoms (already consumed in Plan 01-09)
- `~/Documents/aic/aic_scoring/src/ScoringTier2.cc` — `/scoring/tf` + `/scoring/insertion_event` consumer (tier-2 mode); informs PARITY-07/08 framing

### This repo
- `.planning/PROJECT.md` — vision, principles (topic parity = architectural law), repo split contract
- `.planning/REQUIREMENTS.md` — TRIAL-01..05 + DX-05 (Phase 4 rows) + traceability table
- `.planning/ROADMAP.md` — Phase 4 success criteria (5 items: trial UI button outcome, ground_truth on/off, aic_engine E2E, parity-report regen, CLAUDE.md + README)
- `.planning/phases/03-cable-physics/03-SUMMARY.md` — Phase 3 closure record + carry-forwards (PARITY-07 live fire deferred here)
- `.planning/phases/01-foundation-parity/01-09-PLAN.md` — per-component spawn atom signatures (parameter surface for D-03 adapter dict)
- `exts/aic-dt/aic_dt/extension.py` — `MCP_TOOL_REGISTRY` + `_cmd_<atom>` handlers — the 4-surface DX-02 contract
- `exts/aic-dt/aic_dt/scoring_publishers.py` — `_start_aic_scoring_publishers` lifecycle (ground_truth gate location)
- `exts/aic-dt/aic_dt/parity_publishers.py` — `_start_aic_parity_publishers` lifecycle
- `exts/aic-dt/docs/topic-parity-reference.md` — Phase-1 cross-phase parity audit table (DX-05 README must reference)
- `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh` — lifecycle helper consumed by `run_aic_engine_against_isaac_sim.sh`
- `~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py` — cache snapshot/restore (called from E2E wrapper for cold-start safety)

### Live system surface (verified 2026-05-05)
- Isaac Sim MCP socket: `localhost:8768` (TCP, JSON, no length prefix)
- ROS_DOMAIN_ID=7 (sim isolation — UR5e real-driver runs on default 0)
- `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` — real Kit log (grep `[AIC-DT]`)
- `/tmp/isaacsim.log` (lifecycle helper) + `/tmp/aic_dt_postload.log` (postload script)
</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

**Spawn atoms (Plan 01-09) — exact match to YAML schema:**
- `spawn_task_board_base(x, y, z, roll, pitch, yaw)` ← `task_board.pose`
- `spawn_lc_mount_rail(index, present, translation, roll, pitch, yaw)` ← `lc_mount_rail_<i>`
- `spawn_sfp_mount_rail(...)` ← `sfp_mount_rail_<i>`
- `spawn_sc_mount_rail(...)` ← `sc_mount_rail_<i>`
- `spawn_sc_port(index, present, translation, rpy)` ← `sc_rail_<i>` (port spawn at SC rail)
- `spawn_nic_card_mount(index, present, translation, rpy)` ← `nic_rail_<i>` mount
- `spawn_nic_card(index, present, translation, rpy)` ← `nic_rail_<i>` card

**Lifecycle helpers:**
- `_start_aic_parity_publishers(self)` — `/joint_states` + `/tf` + `/tf_static` (Phase 1)
- `_start_aic_scoring_publishers(self)` — `/scoring/tf` + `/objects_poses_real` + `/scoring/insertion_event` (Phase 3) — the `ground_truth` flag gates this call
- `_start_aic_controller_loop(self)` — `joint_commands` + `pose_commands` + `controller_state` + off-limit contacts (Phase 2)

**Cable + attach (Phase 3):**
- `load_robot(..., cable_type, attach_cable_to_gripper, gripper_initial_pos)` — Phase 3 kwargs already plumbed
- `attach_cable_to_gripper(finger_link_path, plug_link_path)` — 4-surface MCP atom (Plan 03-03)

**Master clubber:**
- `quick_start(self)` — the canonical chain. `load_trial` is its config-driven sibling.

### Established Patterns

**4-surface DX-02 atom contract (proven across 9 atoms in Plan 01-09 + `attach_cable_to_gripper`):**
1. `MCP_TOOL_REGISTRY` entry (top of `extension.py`) — name + parameters JSON schema
2. Handler-map dispatch (`_cmd_<name>` method on `DigitalTwin`)
3. `_cmd_<name>` body that parses params + calls implementation
4. UI button under "AIC Digital Twin" window

**Lazy-import pattern (Phase 2 controller_loop + Phase 3 scoring_publishers):**
Class definition free of `omni.*` / `rclpy` imports; all heavy imports inside `start()`. Lets offline structural verification (`importlib.util.spec_from_file_location`) without Kit.

**ROS_DOMAIN_ID=7 isolation:** Every `aic-dt` launch + smoke test sets this; competing UR5e real driver lives on default domain 0.

### Integration Points

- New atom registers in `MCP_TOOL_REGISTRY` after `quick_start` — natural reading order for the trial-loader story.
- Wrapper script lives in `exts/aic-dt/scripts/` next to existing `smoke_test_*.py` and `verify_phase_*.sh`.
- Parity-report outputs land in `.planning/phases/04-trial-loader/` (alongside CONTEXT.md / PLAN.md / SUMMARY.md) for permanent audit trail.
- `aic_engine` Docker container reuses `ghcr.io/intrinsic-dev/aic/aic_eval:latest` (image digest pinned via D-14 in Phase 1) but with Gazebo bringup disabled — only `aic_engine` + `aic_controller` + ros_gz_bridge run, talking to Isaac Sim's already-published topics on `ROS_DOMAIN_ID=7`.
</code_context>

<specifics>
## Specific Ideas

- **Ship gate is an empirical match, not a feature inventory.** TRIAL-04 is THE M1 ship gate; if every Gazebo-passing trial passes Isaac Sim, M1 is done. If 1 of 3 trials mismatches, M1 is not done — debug + fix until parity holds.
- **Trial config schema is the AIC repo's contract, not ours.** We follow the YAML keys verbatim — no `_isaac` extensions, no remapping. If `aic_engine` adds a new top-level key in a future release, our `load_trial` adapter dict gracefully ignores unknown keys (warns once) and continues.
- **The "verify by running" pattern from Phase 3 carries forward.** `smoke_test_aic_trial.py` is OPTIONAL — `parity_report.py` is the canonical verification because it's also the M1 ship gate. Per Phase 3 SUMMARY: "Phase 4 E2E trial verification supersedes [phase smoke tests] — running the actual aic_engine + CheatCode trial against Isaac Sim is the canonical verification."
</specifics>

<deferred>
## Deferred Ideas

- **M2: pose-source swap.** `_start_aic_scoring_publishers` is the seam — replace with a real-world pose estimator publisher that emits the same `/tf` + `/objects_poses_real` topic surface. The `ground_truth=False` flag pre-sets the gate.
- **Per-trial UI buttons.** Today: one `load_trial` MCP atom + one UI button (free text trial_key). Future: auto-generate per-trial buttons from `sample_config.yaml` — clean DX, but pulls YAML parsing into UI rendering (cyclic concern). Defer.
- **Trial loader randomization layer.** `randomize_object_poses` + `randomize_lighting` already exist (Phase 1 atoms). Some future trial config may want `seed` + `random_pose_noise_sigma` keys. M2+ work.
- **Tier-2 scoring full activation.** `/scoring/tf` is published; `ScoringTier2.cc` will consume IF aic_engine launches in tier-2 mode. The 3 sample trials don't activate tier-2; Phase 4 doesn't either. Future config option.
- **Streaming wrapper (`~/bin/isaacsim`) fix for aic-dt.** Hardcoded `EXT_MCP_PORT` map missing aic-dt 8768. DX task tracked in CLAUDE.md; not a Phase 4 deliverable.
- **`parity_publishers` hot-reload regression.** `touch extension.py` corrupts rclpy node state; restart required. Documented Phase 3 limitation; defer to a maintenance pass.
</deferred>

---

*Phase: 04-trial-loader*
*Context gathered: 2026-05-05*
*Mode: --auto (autonomous M1) — all 6 gray areas auto-selected; recommended option chosen for each, rationale logged inline.*
