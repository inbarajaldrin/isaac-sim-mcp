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
