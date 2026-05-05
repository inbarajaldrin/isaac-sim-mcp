---
phase: 04-trial-loader
plan: 02
subsystem: trial-loader
tags: [TRIAL-01, TRIAL-02, DX-02, load_trial, ground_truth]
requirements_completed: [TRIAL-01, TRIAL-02]
dependency_graph:
  requires:
    - "Plan 01-09 spawn atoms (spawn_task_board_base + 6 rail/port atoms)"
    - "Phase 3 load_robot cable_kwargs (cable_x/y/z + cable_roll/pitch/yaw + cable_type + attach_cable_to_gripper)"
    - "Phase 1 D-12 quick_start ordering contract (early-play, then graphs)"
    - "Phase 3 _start_aic_scoring_publishers + Phase 1 _start_aic_parity_publishers"
  provides:
    - "load_trial(config_path, trial_key, ground_truth) MCP atom"
    - "ground_truth kwarg on quick_start + load_trial (M2 pose-source-swap surface)"
    - "_extract_cable_kwargs + _dispatch_rail_block helper methods on DigitalTwin"
  affects:
    - "Plan 04-03 E2E wrapper consumes load_trial via MCP socket"
    - "Plan 04-04 parity_report.py invokes load_trial back-to-back across trials"
tech_stack:
  added:
    - "PyYAML stdlib import (lazy, inside _cmd_load_trial body)"
    - "inspect.signature introspection for atom-fn dispatch"
  patterns:
    - "4-surface DX-02 atom contract (registry + handler-map + _cmd_method + UI button)"
    - "Lazy-import discipline (Phase 2 pattern)"
    - "Adapter dict (YAML keys → spawn-atom kwargs) per 04-RESEARCH.md Q5"
key_files:
  created: []
  modified:
    - "exts/aic-dt/aic_dt/extension.py"
    - "exts/aic-dt/scripts/audit_dx02.py"
decisions:
  - "Adapter Option A (gripper_offset → cable_x/y/z pass-through, not absolute) per 04-RESEARCH.md Q2"
  - "First-cable-wins for the cables.<name> block — aic_engine consumes one cable per trial"
  - "entity_name in rail blocks IGNORED — informational only; spawn atoms use AIC_OBJECTS hardcoded asset USDs"
  - "UI button label avoids parens — audit_dx02.py's UI-regex stops at first ')' so 'Load Trial (foo)' would silently fail surface 4"
  - "Default ground_truth=True everywhere (UI button + atom defaults) — Phase 3 backwards-compat preserved without behavior change"
metrics:
  duration_min: ~12
  tasks_completed: 3
  commits: 3
  files_modified: 2
  lines_added: ~280
  completion_date: "2026-05-05"
---

# Phase 4 Plan 02: load_trial atom + ground_truth flag — Summary

`load_trial` MCP atom (4-surface DX-02) + `ground_truth` kwarg on
`quick_start` and `load_trial` — TRIAL-01 + TRIAL-02 land as a
pure-code-edit pass; live-fire deferred to Plan 04-03.

## Objective

Land the API surface that Plan 04-03's E2E wrapper consumes:

1. **TRIAL-01** — single MCP atom `load_trial(config_path, trial_key, ground_truth)`
   that parses one trial entry from `sample_config.yaml`-format YAML and
   dispatches to existing per-component spawn atoms (Plan 01-09) +
   `load_robot` (Phase 3).
2. **TRIAL-02** — `ground_truth` kwarg on both `load_trial` and `quick_start`
   that gates `_start_aic_scoring_publishers` (the M2 pose-source-swap surface).

No new files except the SUMMARY; ~280 LOC added across `extension.py` +
1 line on `audit_dx02.py`.

## Surfaces added

### `load_trial` atom (4-surface DX-02)

| # | Surface | Location | Detail |
|---|---------|----------|--------|
| 1 | **registry** entry | `MCP_TOOL_REGISTRY` (`extension.py:237-256`) | name + 3 params (config_path / trial_key / ground_truth) |
| 2 | **handler-map** entry | `MCP_HANDLERS` (`extension.py:~415`) | `"load_trial": "_cmd_load_trial"` |
| 3 | **`_cmd_load_trial`** method | `class DigitalTwin` (`extension.py:~3092`) | async; full body below |
| 4 | **UI button** | `_build_ui` (`extension.py:~603`) | "Load Trial sample_config trial_1" → `asyncio.ensure_future(self._cmd_load_trial(trial_key="trial_1", ground_truth=True))` |

### `quick_start` parameter expansion (1 atom × 1 kwarg)

- `MCP_TOOL_REGISTRY["quick_start"]["parameters"]` gains `ground_truth` entry.
- `async def quick_start(self, ground_truth: bool = True)` signature update.
- `async def _cmd_quick_start(self, ground_truth: bool = True)` signature update.
- `if ground_truth:` gate around the existing `_start_aic_scoring_publishers()` call site (extension.py:~1115).
- No UI change — the existing "Quick Start" button continues to use default `True`.

## Adapter contract

Per 04-RESEARCH.md Q5 (schema audit) + Q6 (cable extraction):

### Per-rail / per-port (`_dispatch_rail_block`)

YAML `task_board.<rail_kind>_<i>` block → spawn-atom kwargs:

| YAML field                     | Spawn-atom kwarg | Notes                                            |
|--------------------------------|------------------|--------------------------------------------------|
| `entity_present: bool`         | `present`        | Mandatory                                        |
| `entity_pose.translation: float` | `translation`  | Default 0.0                                       |
| `entity_pose.roll/pitch/yaw`   | `roll/pitch/yaw` | Default 0.0                                      |
| `entity_name: str`             | **IGNORED**      | Informational only (aic_engine reads, not us)    |

Special: `spawn_nic_card` has no `index` parameter; `_dispatch_rail_block`
introspects via `inspect.signature` and strips `index` automatically.

### Cable (`_extract_cable_kwargs`)

YAML `cables.<name>` block → `load_robot` kwargs:

| YAML field                              | `load_robot` kwarg              | Notes                                  |
|-----------------------------------------|---------------------------------|----------------------------------------|
| `pose.gripper_offset.x`                 | `cable_x`                       | Option A pass-through (Q2)             |
| `pose.gripper_offset.y`                 | `cable_y`                       | Option A pass-through (Q2)             |
| `pose.gripper_offset.z`                 | `cable_z`                       | Option A pass-through (Q2)             |
| `pose.roll/pitch/yaw`                   | `cable_roll/pitch/yaw`          | Direct                                 |
| `cable_type: str`                       | `cable_type`                    | "sfp_sc_cable" / "sfp_sc_cable_reversed" |
| `attach_cable_to_gripper: bool`         | `attach_cable_to_gripper`       | Direct                                 |

If `cables` block is absent, `_extract_cable_kwargs` returns `{}` → `load_robot`
falls back to its Phase-3 hardcoded defaults (cable_x=0.172, cable_y=0.024,
cable_z=1.518, cable_roll=0.4432, cable_pitch=-0.48, cable_yaw=1.3303,
cable_type="sfp_sc_cable", attach_cable_to_gripper=False).

First-cable-wins rule: aic_engine reads exactly one cable per trial; if a
trial defines multiple cables (none currently do), `_extract_cable_kwargs`
takes the first.

### Task board base

YAML `task_board.pose.{x, y, z, roll, pitch, yaw}` → `spawn_task_board_base`
kwargs verbatim (defaults 0.25, 0.0, 1.14, 0.0, 0.0, 0.0).

## `_cmd_load_trial` body — execution order

1. Lazy `import yaml`
2. Default `config_path = ~/Documents/aic/aic_engine/config/sample_config.yaml`
3. `yaml.safe_load(open(config_path))` → validate `trial_key` against
   `cfg["trials"]`; return error dict if missing
4. Extract `scene = trials[trial_key]["scene"]` and `cable_kwargs`
5. `await self._cmd_new_stage()` (D-02 idempotency)
6. `await self.load_scene()`
7. `await self.load_robot(**cable_kwargs)`
8. **`self._timeline.play()` early** (Phase 1 D-12 non-negotiable)
9. `setup_tf_publish_action_graph()` + `setup_joint_state_publish_action_graph()`
10. `_start_aic_controller_loop()` (Phase 2)
11. `await self.setup_action_graph()` + `setup_wrist_cameras()` + `setup_force_publish_action_graph()`
12. `_cmd_spawn_task_board_base(**tb_kwargs)`
13. Iterate 13 rail/port blocks — `nic_rail_<0..4>`, `sc_rail_<0..1>`,
    `{lc,sfp,sc}_mount_rail_<0..1>` — via `_dispatch_rail_block`
14. `_start_aic_parity_publishers()` (always on per D-04)
15. `if ground_truth: _start_aic_scoring_publishers()` else skip-print
16. `self._timeline.play()` (idempotent)
17. Return `{status, trial, ground_truth, config_path, spawned_components, cable_kwargs}`

Error path: try/except wraps the whole body; returns
`{status: "error", message, traceback: traceback.format_exc()}`.

## Offline verification (live-fire deferred)

| Check | Result |
|-------|--------|
| `ast.parse(extension.py)` | OK |
| `grep "load_trial":` extension.py | 2 matches (registry + handler-map) |
| `grep "async def _cmd_load_trial"` | 1 match |
| `grep "def _extract_cable_kwargs"` | 1 match |
| `grep "def _dispatch_rail_block"` | 1 match |
| `grep "import yaml"` | 1 match (lazy, inside method body) |
| `grep "gripper_offset"` | 5 matches (helper body, comments) |
| `grep "Load Trial"` UI button | 1 match |
| `audit_dx02.py` PRESENT_ATOMS contains `"load_trial"` | yes |
| `~/env_isaaclab/bin/python audit_dx02.py` exit code | **0** (PASS, 30 PRESENT × 4 surfaces + 2 ABSENT × 4 surfaces) |
| YAML smoke (`trial_1` + cables block exist in sample_config.yaml) | OK |
| Offline structural test: `_extract_cable_kwargs` + `_dispatch_rail_block` against all 3 trials via stub harness | OK — 13 rail blocks dispatched per trial; no-index strip OK |

Live-fire (actual Isaac Sim launch + MCP `load_trial` round-trip + topic
inspection) is **deferred to Plan 04-03** by design — that plan owns the
E2E wrapper + cold-start safety + first runtime exercise.

## Deviations from plan

- **UI button label changed from `"Load Trial (sample_config.yaml: trial_1)"`
  to `"Load Trial sample_config trial_1"`** (Rule 3 fix). The audit_dx02.py
  UI-button regex `r"ui\.Button\([^)]{0,500}"` stops at the first `)`
  character; a label with parens (`(...)`) silently terminated the regex
  capture before reaching the `clicked_fn` handler reference, causing
  surface-4 to FAIL. Caught by running the audit; relabelled the button
  to be regex-friendly. Future planners adding UI buttons: keep `(`/`)`
  out of the visible label string until the audit regex is upgraded
  (DX-02 maintenance ticket — small future-cleanup item).

- **`grep '"load_trial":'` count**: plan acceptance criterion expected ≥3
  matches; actual is 2 (registry + handler-map). The 2 surfaces correctly
  cover this naming convention; the method def uses `def _cmd_load_trial`
  (no quotes), and the UI button has no quoted `"load_trial"` literal.
  audit_dx02.py is the canonical gate and exits 0; the grep count was
  a heuristic in the plan's `<verify>` block. Documented for clarity;
  no functional impact.

## Carry-forwards

- **Live-fire of `load_trial` deferred to Plan 04-03.** This plan ships the
  API surface (registry + handler + method + UI + adapter dict + audit).
  Plan 04-03's E2E wrapper (`run_aic_engine_against_isaac_sim.sh`) will be
  the first MCP-socket round-trip exercise. Cold-cache + USD reload churn
  for offline structural verification is wasteful per the plan's design.

- **`_PORT_LINK_PATHS` D-13 setter** (per Plan 04-01 A4 verdict
  `MISMATCH_NIC_CARD_MOUNT`) — Plan 04-03 must add a public
  `set_port_link_paths(paths: list[str])` method on `AicScoringPublishers`
  (1-surface addition) and have `load_trial` compute paths from spawn-call
  sites. **This plan does NOT yet wire that** — the spawn loop in
  `_cmd_load_trial` records spawn names into `spawned_components` for the
  return dict, but does not yet call `set_port_link_paths` on the scoring
  publisher. Plan 04-03 owns that wiring (and the setter itself).

- **Default-cable behavior verified across all 3 trials**: `cable_x=0.0`
  (not 0.172) and `cable_z` varies between 0.04045 and 0.04545. The
  Phase-3 default `cable_x=0.172` is the no-cable fallback; YAML-driven
  trials supply gripper_offset.x=0.0 instead. This matches Q2/Q6's "Option A"
  pass-through interpretation (offset, not absolute) — the offset is
  added to the gripper's pose at runtime, not used as a world coord.

## Next

**Plan 04-03** — Docker-derived image (verdict: stock `aic_eval:latest` per
04-01 A2 PASS — no derived image needed) + E2E wrapper script
(`run_aic_engine_against_isaac_sim.sh`) + dry-run with `load_trial("trial_1")`
+ `set_port_link_paths` setter on `AicScoringPublishers` (D-13, 1-surface
addition forced by 04-01 A4 MISMATCH).

## Self-Check: PASSED

Files verified to exist:
- `exts/aic-dt/aic_dt/extension.py` — FOUND (modified, +268 LOC)
- `exts/aic-dt/scripts/audit_dx02.py` — FOUND (modified, +2 LOC)

Commits verified to exist:
- `aa14256` (Task 1: ground_truth kwarg on quick_start) — FOUND
- `82fc1a7` (Task 2: load_trial atom + adapter helpers) — FOUND
