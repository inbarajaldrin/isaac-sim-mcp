# Texture / MDL Sweep Log (Phase 1 D-07 / TEX-03)

Each section below is a sweep run. Asset = path or prim. Status: unresolved | resolved | accepted-cosmetic.

## Sweep methodology

Per D-07 (CONTEXT.md), the sweep is a scripted log-grep loop:
1. Load M1 scene via MCP `quick_start`
2. Grep `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` for `MDL`, `texture`, `missing`, `pink`, `fallback`, `not found`
3. Each warning becomes a row below.
4. Iterate fix → re-load → re-grep until zero asset-related warnings.

Augmented patterns (Isaac-Sim asset-failure surface, added pre-emptively per CONTEXT.md D-07 "refine after first run"):

- `Failed to open`
- `Could not open`
- `unresolved`
- `reference.*invalid`

These cover the known-broken sub-references in `sc_port_visual.usd` (ISO_4762 / V1015120 missing USDs) surfaced during RESEARCH vendoring manifest analysis.

Run via: `python3 exts/aic-dt/scripts/sweep_textures.py`

Other invocations:

- `python3 exts/aic-dt/scripts/sweep_textures.py --skip-load` (re-grep without re-loading)
- `python3 exts/aic-dt/scripts/sweep_textures.py --port 8768 --out path/to/out.md`

## Row format

| Asset | Problem | Fix | Status |

- **Asset**: `assets/<path>` or `/World/<prim>` extracted from the log line; `<unknown>` if neither pattern matched.
- **Problem**: which regex pattern fired (one of the PATTERNS list above).
- **Fix**: trimmed log-line excerpt as a starting hint for the human reviewer.
- **Status**: `unresolved` (default) | `resolved` (asset fixed; verified clean on next sweep) | `accepted-cosmetic` (Kit-builtin MDL fallback or other intentional non-fix).

## Pre-run baseline (Plan 06 landing)

This file is initialized with no rows. The first sweep run will append a `## Sweep run <timestamp>` section.
## Sweep run 2026-05-02T07:11:54

Source log: `/home/aaugus11/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_20260502_071116.log`
Hit count: 921

| Asset | Problem | Fix | Status |
|-------|---------|-----|--------|
| `<unknown>` | `texture` | _2026-05-02T14:11:16Z [4ms] [Info] [omni.ext.plugin] [ext: omni.kit.hydra_texture-1.4.5] registered (path: /home/aaugus11/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache/omni.kit.hydra_..._ | unresolved |
| `<unknown>` | `MDL` | _2026-05-02T14:11:16Z [5ms] [Info] [omni.ext.plugin] [ext: omni.kit.stage.mdl_converter-1.0.8] registered (path: /home/aaugus11/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache/omni.kit...._ | unresolved |
| `<unknown>` | `fallback` | _2026-05-02T14:11:17Z [1,840ms] [Info] [omni.kit.viewport.window.extension] Ignoring embedded ui.scenes fallbacks for "omni.kit.viewport.legacy_gizmos"_ | unresolved |
| `<unknown>` | `not\s*found` | _2026-05-02T14:11:25Z [9,461ms] [Warning] [omni.fabric.plugin] Warning: attribute viewportHandle not found for bucket id b_ | unresolved |
| `/World/UR5e/center_camera_optical/center_camera,` | `not\s*found` | _2026-05-02T14:11:33Z [17,249ms] [Info] [omni.kit.app._impl] [py stdout]: Warning: Camera prim not found at /World/UR5e/center_camera_optical/center_camera, skipping center_camera_ | unresolved |
| `/World/UR5e/left_camera_optical/left_camera,` | `not\s*found` | _2026-05-02T14:11:33Z [17,249ms] [Info] [omni.kit.app._impl] [py stdout]: Warning: Camera prim not found at /World/UR5e/left_camera_optical/left_camera, skipping left_camera_ | unresolved |
| `/World/UR5e/right_camera_optical/right_camera,` | `not\s*found` | _2026-05-02T14:11:33Z [17,249ms] [Info] [omni.kit.app._impl] [py stdout]: Warning: Camera prim not found at /World/UR5e/right_camera_optical/right_camera, skipping right_camera_ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,333ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_013/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,333ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_138/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_139/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/V1015120_012/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_012/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/V1015120_013/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_137/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,334ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_0/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/V1015120_013/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,339ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_139/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_138/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_137/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_012/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_013/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,340ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_136/V1015120_012/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,341ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/TaskBoard/SCPort_1/ISO_4762___M2_x_8_..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_138/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,354ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_137/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,354ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/V1015120_013/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,354ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,354ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/V1015120_012/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,355ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_013/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,355ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_139/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,355ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_1/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_012/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,355ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_1/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_139/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,357ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_137/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,357ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_013/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/V1015120_012/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/V1015120_013/V1015120_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_6_012/ISO_4762___M2_x_6_004>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/sc_port_2/ISO_4762___M2_x_8_136/ISO_4762___M2_x_8_138/ISO_4762___M2_x_8_015>:` | `Could\s+not\s+open` | _2026-05-02T14:11:33Z [17,358ms] [Warning] [omni.usd] Warning: in _ReportErrors at line 3172 of /builds/omniverse/usd-ci/USD/pxr/usd/usd/stage.cpp -- In </World/Objects/sc_port_2/ISO_4762___M2_x_8_1..._ | unresolved |
| `/World/Objects/task_board_base/task_board_base` | `not\s*found` | _2026-05-02T14:11:33Z [17,361ms] [Info] [omni.kit.app._impl] [py stdout]:   Warning: Body prim not found at /World/Objects/task_board_base/task_board_base_ | unresolved |
| `/World/Objects/sc_port_1/sc_port_1` | `not\s*found` | _2026-05-02T14:11:33Z [17,361ms] [Info] [omni.kit.app._impl] [py stdout]:   Warning: Body prim not found at /World/Objects/sc_port_1/sc_port_1_ | unresolved |
| `/World/Objects/sc_port_2/sc_port_2` | `not\s*found` | _2026-05-02T14:11:33Z [17,361ms] [Info] [omni.kit.app._impl] [py stdout]:   Warning: Body prim not found at /World/Objects/sc_port_2/sc_port_2_ | unresolved |
| `/World/Objects/nic_card/nic_card` | `not\s*found` | _2026-05-02T14:11:33Z [17,362ms] [Info] [omni.kit.app._impl] [py stdout]:   Warning: Body prim not found at /World/Objects/nic_card/nic_card_ | unresolved |

