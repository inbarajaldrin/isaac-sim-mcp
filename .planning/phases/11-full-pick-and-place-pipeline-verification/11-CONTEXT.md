# Phase 11: Full Pick-and-Place Pipeline Verification — Context

**Logged:** 2026-04-25
**Status:** In progress (lightweight track — informal logging, not a discuss-phase output)
**Source of truth for this phase:** this CONTEXT + the 11-NN-PLAN files in this directory.

## Phase Boundary

End-to-end verification of the real-camera pick-and-place pipeline:
**YOLOE detects lego → ArUco detects cups → arm picks one block → drops it in matching cup → cache reflects what's still on the workspace.**

All inputs are real-camera topics (`/objects_poses_real`, `/drop_poses_real`). No sim ground-truth topics are read by Phase 11 paths.

**Target repo:** `~/Projects/Exploring-VLAs/vla_SO-ARM101` (control_gui.py — primary).
**No changes** in `isaac-sim-mcp` extension or `aruco_camera_localizer` for this increment — both already publish what we need.

## Prior Work Already Landed (pre-phase, attributed here)

This phase formally adopts the cross-repo work captured in `.planning/HANDOFF.json` (paused 2026-04-25, `phase: null`). That work was implicitly Phase 11 scope and had no phase folder — adopting it here.

- **vla_SO-ARM101 commits `3ea86d8`, `2317238`:** Real Test tab scaffold (Setup + Calibration + Run sections); subscribe-once refresh handlers for cups (`/drop_poses_real`) and legos (`/objects_poses_real`) with partial-merge cache + wipe-then-push planning-scene update; Clear-all-caches button; loop-over-all-cached-legos run handler; `_lookup_bbox` 3-tier fallback (exact / color-only / color-prefix); `_build_lego_geometry` (mesh-or-box dispatch, no color→size inference); drop_point wrist_roll sign fix; per-robot YOLOE config in `robot_config.yaml`.
- **aruco_camera_localizer commit `2317238`:** `marker_geometry.py` with `cylinder_side_marker` function (computes both translation offset AND `R_marker_to_object` orientation rotation); `marker_to_object` config schema (method+params, position_offset legacy fallback for jetank); `localize_aruco` dispatches on schema, applies orientation rotation in published drop_quat. Cups now appear UPRIGHT in MoveIt planning scene (verified via `/get_planning_scene`).
- **vla_SO-ARM101 `11edc99`:** `scripts/restart_yoloe.sh` launcher (mirrors `restart_aruco_localizer.sh`; config-driven prompts; `--bg`, `--headless`, `--conf`, `--prompts`, `--camera` flags).

## Phase 11 Success Criteria (from ROADMAP)

| # | Criterion | Status entering this phase |
|---|---|---|
| 1 | YOLOE detects lego blocks → `/objects_poses_real` | ✓ done (prior work) |
| 2 | ArUco localizer detects cup markers → `/drop_poses_real` | ✓ done (prior work) |
| 3 | Full cycle: detect block → pick → detect cup → drop → verify block in cup | ⚪ untested end-to-end |
| 4 | Works for all 3 colors (red→red, etc.) | ⚪ untested across all colors |
| 5 | No manual intervention needed between pick and place | ⚠ explicit deviation: 11-01 introduces a color dropdown trigger; full autonomy deferred to a later increment |

## Plan Overview

- **11-01** — Color-driven single-pick workflow + cache lifecycle. Adds color dropdown above Run, single-pick-per-press semantics, post-drop cache eviction, dropdown auto-refresh on cache mutations. Targets criteria 3, 4. Defers criterion 5.
- **(future)** 11-02 — VFOV + camera-spec single-source-of-truth across repos. Diagnose ~25–30 mm xy/z PnP residual + ~6° orientation residual (HANDOFF remaining tasks 18–19). Build a shared camera-spec module so vla / aruco_camera_localizer / isaac-sim-mcp can't drift.
- **(future)** 11-03 — Autonomous mode (criterion 5). Iterate detected legos without human trigger, opt-in.

## Implementation Decisions (locked)

### D-01: Single-shot pick-per-press, not loop-over-all
The current `_real_pick_drop_thread` iterates ALL cached legos sequentially. 11-01 replaces this with single-shot: user picks color from dropdown → presses Run → ONE lego of that color goes through the full 9-step `_QS_SEQUENCE` → cache updates → user picks next color (or refreshes).

**Rationale:** Verification surface for criteria 3 + 4 is "one cycle works correctly per color". Looping is a feature for criterion 5, not 3/4.

### D-02: Lego selection within a color = closest-to-base
When multiple legos of the chosen color are cached (e.g., 2 red), pick the one with smallest `sqrt(x² + y²)` from `base_link`. Alphabetical tiebreak on cache key (`red_lego_0` before `red_lego_1`) for determinism.

**Rationale:** Closest is most likely reachable (FC-1 yaw-fallback already handles edge-of-workspace, but starting closer reduces retries). Deterministic tiebreak avoids UAT flakiness.

### D-03: Dropdown values = intersection(lego colors cached, cup colors cached)
A red lego alone in cache with no red cup is **not** selectable — there's nowhere to drop it. Cup color is derived from `_qs_auto_drop_for_lego` mapping (`drop_0`→red, `drop_1`→green, `drop_2`→blue) inverted.

**Rationale:** Pre-flight failure ("skip — drop_X not in cache") becomes invisible-by-construction at the UI layer. User can't pick an impossible cycle.

### D-04: Cache lifecycle on Refresh = partial-merge (already current)
Refresh updates positions of detected legos; entries for legos NOT detected in this scan keep their previous cached pose. Matches user spec ("preserve cache but only if the object is still in the scene then it updates its position; if not in the scene we don't worry about it"). This is already the behavior of `_cached_lego_poses.update(captured)` in `_real_refresh_legos_thread` — no change.

### D-05: Cache lifecycle on successful drop = evict
After all 9 `_QS_SEQUENCE` steps return ok, pop the just-dropped lego from `_cached_lego_poses` and `objects_data`, wipe-then-push lego collisions, refresh listbox + dropdown.

**Rationale:** A successful drop means the lego is now in a cup, no longer on the workspace. Keeping it cached would let the user re-select that color and "pick" a phantom block. Eviction makes the dropdown shrink as the user makes progress.

### D-06: Cache lifecycle on failed cycle = NO eviction
Any step in `_QS_SEQUENCE` failing leaves the cache untouched. User retries via Refresh (to update pose if the failure shifted things) + Run again.

**Rationale:** Safe default. If grasp_close failed mid-grasp, the lego is still on the workspace.

### D-07: Sim-data isolation guarantee preserved
The single-pick handler must read ONLY from `_cached_lego_poses` / `_cached_cup_poses` (which are populated from `/objects_poses_real` / `/drop_poses_real`). Per-cycle re-injection into `objects_data` / `_drop_data` (existing pattern at L4401–L4406) defeats any live-topic overwrite. No new code path may call `_qs_refresh_objects` mid-run, since that respects the Grasp Topic toggle and could pull from `/objects_poses` (sim).

### D-08: New widget = ttk.Combobox (readonly)
Extend the Phase 07.1 widget registry with `_register_combobox`. State `readonly` (user picks from list, can't type free-form). Mirrors `_register_listbox` pattern for `widget_registry_add` shape.

**Rationale:** Combobox is the natural Tk widget for "pick one of N short strings". Extending the registry keeps the Phase 07.1 "every interactive widget addressable by name" invariant.

### D-09: Empty-state UX
If intersection is empty: combobox shows `(no compatible legos)` and Run button is disabled. As soon as a Refresh produces a non-empty intersection, dropdown re-enables.

### D-10: Dropdown refresh trigger sites
A single helper `_real_refresh_color_dropdown()` is called at every cache mutation:
- After Refresh Cups Pose (cup set may have grown)
- After Refresh Legos Pose (lego set may have grown)
- After Clear all caches (intersection becomes empty)
- After successful drop + evict (color may have just become unavailable)

### D-11: Sim-data isolation requires forcing topics + per-step re-inject (revised D-07)
**Discovered during 11-01 UAT:** D-07's "re-inject cache once at cycle start" was insufficient. Two leak paths emerged:

1. `_objects_callback` (subscribed to whatever `Grasp Topic` Entry says, defaulting to `/objects_poses_sim`) overwrites `objects_data` on every TFMessage AND auto-schedules `_add_lego_collision_objects` on >3 mm moves — so sim poses leak into both `objects_data` and the MoveIt planning scene mid-cycle.
2. `_bbox_callback` (subscribed to `/objects_bbox_sim` by default; bbox sub is hard-locked at `__init__`, no _cmd helper for re-subscription) leaves `_lookup_bbox` returning sim catalog values instead of the real bbox catalog YOLOE publishes.

**Fix (landed 11-01-04 + new helpers):**
- `_real_ensure_real_topics()` — idempotent helper called from `_cmd_real_run_one_color` BEFORE spawning the worker thread. Forces `Grasp Topic` → `/objects_poses_real` (re-uses existing `_cmd_grasp_update_topic`); forces `BBox Topic` → `/objects_bbox_real` and inline destroy+create the bbox subscription (no _cmd helper exists for it). Clears stale `objects_bbox` (refills within ~1 s from real topic).
- `_real_inject_active_pair(lego, cup)` — called RIGHT BEFORE every `_QS_SEQUENCE` step in the worker thread. Cheap defensive re-injection that defeats any stray callback (even from a real topic that just re-detected with a slightly drifted YOLOE pose).

These two together make the cache the sole authoritative source for IK inputs during a Real Test run.

### D-12: Visual marker source divergence — same shape as D-11, third callback (Drop Topic)
**Discovered during 11-01 visual debug session:** user pointed wrist camera at an ArUco marker, pressed Refresh Cups Pose, and observed visual markers + collision objects at *different* positions in RViz. Forensic dump of `/cup_visual_markers_array` vs `/get_planning_scene` proved the visual markers were tracking SIM `/drop_poses` exactly (e.g. drop_0 visual y=−0.280 = sim ground truth, while planning scene y=−0.312 = cached real localizer output).

**Root cause:** `_publish_cup_visual_markers` had no `cups_dict` parameter — it only ever read `self._drop_data`, which the live `_drop_callback` (subscribed to whatever `Drop Topic` Entry says, defaulting to `/drop_poses` sim) continuously overwrites with sim values. Meanwhile `_add_cup_collision_objects` was upgraded with `cups_dict=` in the HANDOFF era and correctly used the cached real poses → divergence.

**Fix (landed alongside 11-01-05):**
- `_publish_cup_visual_markers(cups_dict=None)` now accepts an explicit cups dict, mirroring `_add_cup_collision_objects`'s pattern. Defaults to `_drop_data` when omitted (sim-mode path unchanged).
- `_real_refresh_cups_thread` now publishes visual markers explicitly with `cups_dict=dict(_cached_cup_poses)` right after the collision add, so visual + collision share one source-of-truth in real mode.
- `_real_ensure_real_topics()` extended to also force `Drop Topic` → `/drop_poses_real` (uses existing `_update_drop_topic` helper). Defends against future code paths that publish visual markers from the default `_drop_data` path (e.g. `_cmd_apply_collision_padding` → `_refresh_display_markers`).
- Both `_cmd_real_refresh_cups_pose` and `_cmd_real_refresh_legos_pose` now call `_real_ensure_real_topics()` at entry — refresh is the implicit "I'm in real mode" signal, not just Run.

**Pattern note:** This is the third callback (`_objects_callback`, `_bbox_callback`, `_drop_callback`) that follows the same shape: passive sub writing to a globally-read state dict, with no opt-out for "I'm using a different source right now." A future Phase 11 increment could refactor all three behind a single "real-mode-active" flag that suspends sim-source writes — but for 11-01 the surgical fix (force topics + cups_dict parameter) is enough.

**Note on PnP residual visibility:** Even with this fix, the cached real poses themselves carry the known ~25-30 mm xy/z + ~6° orientation residual from PnP / camera intrinsics (HANDOFF tasks 18-19, deferred to 11-02). Cups will still appear ~28 mm below the table for markers not freshly in FOV (Kalman extrapolation noise). That's a separate root cause and a separate fix.

### D-13: Idempotent topic-switch is wrong — hot-reload bypass
**Discovered during 11-01 Run UAT:** even with D-12's `_real_ensure_real_topics()`, the run cycle showed sim legos appearing in the planning scene (`Added 14 lego collision objects (9 mesh + 5 box)` — 9 sim-named legos with size suffixes + 5 real cached). Topic introspection confirmed: widget showed `Grasp Topic = /objects_poses_real` BUT `ros2 topic info /objects_poses_sim --verbose` showed `so_arm101_control_gui` as a subscriber (count=2), and `/objects_poses_real` showed `count=0`.

**Root cause:** Order bug in `_hot_reload_gui` (L3001-3047):
1. Save state (`grasp_topic = /objects_poses_real`)
2. Destroy + rebuild Grasp tab → `_build_grasp_tab` ends with `_cmd_grasp_update_topic()` (L3464), which subscribes to whatever `_grasp_topic_var.get()` is. **At this point the StringVar was just initialized to the build-time default `/objects_poses_sim`** → subscription is created on `/objects_poses_sim`.
3. `_restore_gui_state` (L2953) sets `_grasp_topic_var` back to `/objects_poses_real` BUT does NOT call `_cmd_grasp_update_topic()` to trigger a re-subscription.

Result: post-reload, widget reports `/objects_poses_real`, actual sub is on `/objects_poses_sim`. My D-12 idempotent check read the StringVar, saw "real," skipped the re-sub, leaving sim sub active. Same shape for BBox (no auto-resub helper but the build-time default applies the same way) and Drop (similar pattern via `_update_drop_topic`).

**Fix (in `_real_ensure_real_topics`):** removed the idempotent skip. ALWAYS re-subscribes to the real topic on every call. Cost is ~1 ms per call (destroy + create sub); benefit is correctness regardless of prior widget/sub state. The StringVar is no longer treated as the source of truth for the actual subscription.

**Followup (out of scope for 11-01, defer to a later increment or hygiene pass):** fix the underlying hot-reload order bug — `_restore_gui_state` should trigger re-subscriptions for any topic vars it restores. That'd also fix the analogous case where the user manually edits the topic Entry and hot-reloads expecting their value to take effect (it doesn't currently).

### D-14: Lego collision tracker leaks orphans across topic switches
**Discovered during 11-01 step-button UAT:** user pressed Real Test → Clear all caches; cache emptied (`Detected Objects` listbox count=0), but planning scene still contained 8 sim-named lego collisions (`lego_blue_2x2`, `lego_red_2x4`, etc).

**Root cause:** `_remove_lego_collision_objects` only iterates `self._lego_collision_names`, which `_add_lego_collision_objects` REPLACES wholesale on every call (`self._lego_collision_names = added`). So when:
1. Pre-fix: GUI subscribed to `/objects_poses_sim` → adds 8 sim-named collisions, tracker=[8 sim]
2. Topic-switch fix lands → real refresh adds 5 real-named collisions, tracker=[5 real] ← prior 8 sim names lost
3. User presses Clear → removes the 5 in tracker → 8 sim leak forever

**Fix (in `_remove_lego_collision_objects`):** before constructing the REMOVE diff, query the live planning scene via `/get_planning_scene` for any id starting with `lego_`. Union with the local tracker → remove the entire union. Costs a service round-trip but eliminates orphan accumulation across topic switches, hot-reloads, sim/real mode flips. Same shape as D-13 — abandon a "trust the local cache" optimization in favor of "always re-query the authoritative source." Logs an "incl. N orphan(s)" line when discovery exceeds the tracker — silent in the normal case.

**Pattern note (D-13 + D-14):** Both bugs are local-cache-vs-remote-truth divergence. D-13: tk StringVar said one thing, rclpy subscription said another. D-14: tracker list said one thing, planning scene said another. Whenever you have a local mirror of remote state AND mutations on either side, you need either (a) bidirectional sync that's bulletproof, or (b) "re-query authoritative source on every mutation" as the cheap correctness baseline. We picked (b) for both. A future hygiene pass could explore (a) but the cost-benefit doesn't justify it for 11-01.

## Out of Scope for Phase 11

- Camera-spec single-source-of-truth (deferred to 11-02; user explicitly named this).
- VFOV / PnP residual diagnosis (deferred to 11-02).
- Autonomous color-detection mode (deferred to 11-03; criterion 5).
- Any change to motion planning, IK, or drop motion physics (Phase 9 closed those).

## Files Touched (planned for 11-01)

- `~/Projects/Exploring-VLAs/vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py` — `_register_combobox` helper, Real Test tab Run section UI, `_real_refresh_color_dropdown`, `_cmd_real_run_one_color`, post-drop eviction, eviction wipe-then-push.

No changes in `isaac-sim-mcp` or `aruco_camera_localizer` for 11-01.
