---
phase: 01-foundation-parity
plan: 02
subsystem: assets
tags: [vendoring, usd, textures, pink-material-fix, aic-source]

# Dependency graph
requires:
  - phase: 01-foundation-parity
    provides: "Plan 01-01: live topic surface snapshot (anchors what other plans target); does not block this plan technically"
provides:
  - "Vendored capitalized AIC asset tree at exts/aic-dt/assets/assets/<Object>/ with sibling textures/ folders"
  - "Byte-identical copies of NIC Card, NIC Card Mount, SC Port, SC Plug, Task Board Base from Intrinsic_assets"
  - "TEX-01 root cause resolved at the disk layer: per-object USDs' relative ./textures/Image_*.{png,jpg} references now resolvable"
  - "Retired snake_case exts/aic-dt/assets/objects/ tree (no longer on disk)"
affects:
  - "01-foundation-parity Plan 04 (extension code-path update — AIC_OBJECTS dict must point at new capitalized paths)"
  - "01-foundation-parity Plan 06 (verify_phase_1 sweep — no missing-texture warnings expected post-Plan-04)"
  - "Phase 2 (perception/control) — relies on materially-correct task board for visual policies"

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Vendoring pattern (D-05): one-time cp -r of upstream tree subset; no submodule, no override layer"
    - "Capitalized folder layout preservation: AIC layout (with spaces) mirrored verbatim — preserves USD's relative texture references"

key-files:
  created:
    - "exts/aic-dt/assets/assets/NIC Card/{nic_card.usd, nic_card_visual.usd, textures/Image_{0,1,2}.jpg, textures/NIC_Albedo.jpg}"
    - "exts/aic-dt/assets/assets/NIC Card Mount/{nic_card_mount_visual.usd, nic_card_visual.usd, textures/Image_{0,1,2}.jpg, textures/NIC_Albedo.jpg}"
    - "exts/aic-dt/assets/assets/SC Port/{sc_port.usd, sc_port_visual.usd, textures/Image_{0,1}.png}"
    - "exts/aic-dt/assets/assets/SC Plug/{sc_plug_visual.usd, textures/Image_1.png, textures/sc_plug_visual_image1.png}"
    - "exts/aic-dt/assets/assets/Task Board Base/{base_visual.usd, task_board_rigid.usd}"
  modified: []
  deleted:
    - "exts/aic-dt/assets/objects/{nic_card, nic_card_mount, sc_plug, sc_port, task_board_base}/*.usd (5 files)"

key-decisions:
  - "Vendored via cp -r (whole capitalized folder), NOT selective .usd-only copy — guarantees sibling textures/ ride along (avoids reintroducing TEX-01)"
  - "Preserved AIC's CAPITALIZED folder names verbatim ('NIC Card', 'SC Port', etc. with spaces) — USDs reference textures relatively (./textures/...) so any rename would re-break TEX-01"
  - "Snake_case objects/ tree removed in same plan (Wave 1) even though extension.py code still references it — code-path update is Plan 04 / Wave 2 scope; production extension is broken-by-design between this plan and Plan 04 lands; verify_phase_1.sh runs at Plan 06"
  - "Did NOT vendor Intrinsic_assets/scene.usd, UR5e+gripper.usd, props/ISO_*.usd, props/V1015120_004.usd per RESEARCH.md (broken aggregate / not-used / broken sub-references)"

patterns-established:
  - "Asset-vendoring pattern: cp -r whole upstream folder (preserves siblings), md5sum verification per representative file, retire-old-tree in same commit-pair, preserve upstream layout verbatim"

requirements-completed: [PARITY-02, TEX-01, TEX-02]

# Metrics
duration: 2min
completed: 2026-05-02
---

# Phase 1 Plan 02: Asset Vendoring Summary

**Vendored 5 capitalized AIC asset folders (21 files: 9 USDs + 12 textures) into exts/aic-dt/assets/assets/, preserving the sibling textures/ layout that the per-object USDs reference relatively — resolves TEX-01/TEX-02 at the disk layer.**

## Performance

- **Duration:** ~2 min
- **Started:** 2026-05-02T12:24:39Z
- **Completed:** 2026-05-02T12:26:00Z
- **Tasks:** 2
- **Files added:** 21
- **Files deleted:** 5
- **Commits:** 2 (one per task)

## Accomplishments

- Capitalized AIC asset tree (`NIC Card/`, `NIC Card Mount/`, `SC Port/`, `SC Plug/`, `Task Board Base/`) vendored under `exts/aic-dt/assets/assets/` with byte-identical copies of upstream USDs and their `textures/` siblings (8 JPG + 3 PNG, 11 textures).
- TEX-01 disk-layer root cause eliminated: every per-object USD's relative `./textures/Image_*.{png,jpg}` reference now resolves to a sibling file under the same capitalized parent. After Plan 04 updates the `AIC_OBJECTS` dict in `extension.py`, no pink/black materials should remain.
- Snake_case legacy `exts/aic-dt/assets/objects/` tree removed (5 USDs, ~6.5 MB). The capitalized `assets/assets/` tree is now the sole on-disk asset layout, matching AIC's source layout verbatim.
- md5sum byte-identity confirmed against upstream for representative files (4 spot-checks: 1 PNG + 3 USDs across different folders).

## Task Commits

Each task was committed atomically:

1. **Task 1: Vendor capitalized AIC asset folders with sibling textures/** — `7858369` (feat)
2. **Task 2: Retire snake_case objects/ tree** — `fd34184` (chore)

Plan metadata + state updates committed separately at end of plan.

## Files Created/Modified

### Created (21 files under `exts/aic-dt/assets/assets/`)

| Folder | USDs | Textures |
|---|---|---|
| `NIC Card/` | `nic_card.usd`, `nic_card_visual.usd` | `Image_{0,1,2}.jpg`, `NIC_Albedo.jpg` |
| `NIC Card Mount/` | `nic_card_mount_visual.usd`, `nic_card_visual.usd` | `Image_{0,1,2}.jpg`, `NIC_Albedo.jpg` |
| `SC Port/` | `sc_port.usd`, `sc_port_visual.usd` | `Image_{0,1}.png` |
| `SC Plug/` | `sc_plug_visual.usd` | `Image_1.png`, `sc_plug_visual_image1.png` |
| `Task Board Base/` | `base_visual.usd`, `task_board_rigid.usd` | (none — Task Board Base has no textures/ subfolder at source) |

Total: 9 USDs + 12 texture files = 21 files.

### Deleted (5 USDs under retired `exts/aic-dt/assets/objects/`)

Pre-removal inventory (preserved here for traceability):

```
exts/aic-dt/assets/objects/nic_card/nic_card.usd                     (1538685 bytes)
exts/aic-dt/assets/objects/nic_card_mount/nic_card_mount_visual.usd  (363554 bytes)
exts/aic-dt/assets/objects/sc_plug/sc_plug_visual.usd                (1484363 bytes)
exts/aic-dt/assets/objects/sc_port/sc_port.usd                       (1395485 bytes)
exts/aic-dt/assets/objects/task_board_base/task_board_rigid.usd      (1743263 bytes)
```

No `*.md` / `README*` / `*.txt` docs existed in the retired tree (verified with `find ... -name "*.md" -o -name "README*" -o -name "*.txt"` → empty). No doc-preservation move was needed.

## md5sum Byte-Identity Verification

Verified upstream (`~/Documents/aic/.../Intrinsic_assets/assets/`) against vendored (`exts/aic-dt/assets/assets/`):

| File | md5 (both src and dst) |
|---|---|
| `SC Port/sc_port_visual.usd` | `71bfbb52e23b6c4cb533840fbf60a02a` |
| `Task Board Base/base_visual.usd` | `00f960a83a494fb79b8b17ef8b3abdd5` |
| `NIC Card Mount/nic_card_visual.usd` | `c5ffabcb055ed6ea99f6d1f3e6018dfa` |
| `SC Port/textures/Image_0.png` | `8133998ae705480b01a08e7b940aee33` |

All 4 spot-checks: byte-identical. Vendoring did no transformation.

## AIC Source Used

- **Path:** `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/Intrinsic_assets/assets/`
- **Permissions on this repo's side:** read-only consumer; AIC repo not modified.
- **Source files dated:** Mar 13 08:09 (per `ls -la` of source folders during vendoring).

## Source Files Not Found / Resolved

None. All 5 expected source folders (`NIC Card`, `NIC Card Mount`, `SC Port`, `SC Plug`, `Task Board Base`) were present at the documented `Intrinsic_assets/assets/` path. No deviations from the manifest in RESEARCH.md "Vendoring Manifest (D-05)".

Note re: PLAN's expected-files list: the plan listed 5 visual USDs as core expected files but the source folders also contain non-visual companions (`nic_card.usd`, `sc_port.usd`, `task_board_rigid.usd`) plus a duplicated `nic_card_visual.usd` shared between `NIC Card/` and `NIC Card Mount/`. `cp -r` correctly captured all of them; this is in line with the plan's instruction to use whole-folder recursive copies (NOT selective `.usd`-only). Total USD count = 9 (≥5 required).

Note re: `Task Board Base/`: source has no `textures/` subfolder. Plan's acceptance criterion expected `>=4` JPG files in `NIC Card Mount/textures` (got 4, ✓) and `>=1` PNG in `SC Port/textures` (got 2, ✓). Task Board Base needing no textures is consistent with its USDs not declaring texture bindings.

## Decisions Made

- **Used `cp -r` per the plan**, not selective copy. This is the entire reason we don't reintroduce TEX-01 — the textures arrive automatically as siblings.
- **Preserved spaces and capitalization in folder names** (e.g., `NIC Card Mount`). The unified scene's per-object USDs reference textures relatively (`./textures/...`) and Pixar's USD resolver handles paths with spaces fine; renaming would require a USD-side rewrite (out of scope, contradicts D-05's "one-time vendor, no transformation" stance).
- **Did NOT update `extension.py` AIC_OBJECTS** — that's Plan 04 / Wave 2 scope. Production extension is broken-by-design between this plan and Plan 04; the next test gate is Plan 06's `verify_phase_1.sh`.
- **Snake_case `objects/` retirement done in same plan (Wave 1)** — chosen over deferring to Plan 04 because keeping two layouts on disk creates confusion about which is canonical, and the new layout is fully sufficient.

## Deviations from Plan

None - plan executed exactly as written.

(No auto-fixes required. The vendoring step was deterministic: source verified, `cp -r` succeeded, all acceptance criteria passed on first run, md5sum matches first try.)

## Issues Encountered

None.

## User Setup Required

None - no external service configuration required. AIC repo at `~/Documents/aic` was already present (D-13 says the extension runs without it after this lands; the vendoring step itself requires it).

## Threat Surface Scan

No new attack surface. This plan only adds upstream-public USD/PNG/JPG content and removes one stale folder. No new network endpoints, auth paths, or secrets. The threat-register entry T-02-03 (DoS via mistyped `rm -rf`) was mitigated by:
- pre-removal `test ! -d` short-circuit and inventory snapshot,
- explicit literal path `exts/aic-dt/assets/objects` (no globs, no env var expansion at the dangerous step),
- post-removal verification `test ! -d`.

## Self-Check: PASSED

Verified by re-checking on disk and in git after both task commits:

```
exts/aic-dt/assets/assets/SC Port/sc_port_visual.usd                 → FOUND
exts/aic-dt/assets/assets/SC Port/textures/Image_0.png               → FOUND
exts/aic-dt/assets/assets/NIC Card Mount/nic_card_visual.usd         → FOUND
exts/aic-dt/assets/assets/Task Board Base/base_visual.usd            → FOUND
exts/aic-dt/assets/assets/SC Plug/sc_plug_visual.usd                 → FOUND
exts/aic-dt/assets/assets/NIC Card/nic_card_visual.usd               → FOUND
exts/aic-dt/assets/objects                                           → ABSENT (correctly retired)

git log --oneline | grep 7858369                                     → FOUND (Task 1 commit)
git log --oneline | grep fd34184                                     → FOUND (Task 2 commit)

USD count under assets/assets:  9  (>= 5 required)
Texture count (any/textures/*): 12 (>= 4 required)
md5sum byte-identity:           4/4 representative files match upstream
```

## Next Phase Readiness

**Ready for Plan 01-03** (per ROADMAP.md). Plan 04 (`AIC_OBJECTS` path update in `extension.py`) will pick up these vendored assets. Until Plan 04 lands the production extension's `add_objects` / `load_robot` paths still reference the now-absent snake_case `objects/` tree — this is intentional per the plan's sequencing and verified at Plan 06.

No blockers.

---
*Phase: 01-foundation-parity*
*Plan: 02*
*Completed: 2026-05-02*
