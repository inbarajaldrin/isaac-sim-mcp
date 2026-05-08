# Phase 4: Trial Loader — Backlog Sweep

**Generated:** 2026-05-08 — first run of the backlog-sweep discipline (CLAUDE.md "Phase closure discipline — backlog sweep + payload sanity", commit `b6d1169`).

**Purpose:** Scan all deferred items from prior phases (01 + 02 + 03 SUMMARY.md, ROADMAP.md gap-plans, runtime audit findings 2026-05-08) and decide for each whether Phase 4's surface intersection requires CLOSE-NOW or PERSIST-DEFERRED.

**Phase 4 surface** — see `04-CONTEXT.md` `<surface_adjacency>` block.

**Decision key:**
- **CLOSE-NOW** — Phase 4 already touched this surface (or will in 04-04/05); fold the close into a 04-04 task or a new Plan 04.5.
- **PERSIST-DEFERRED** — surface not intersected by Phase 4, OR explicit out-of-scope per PROJECT.md, OR research-gated. Recorded with new gating dependency.
- **SUPERSEDED** — closed indirectly because a Phase 4 deliverable replaces it (e.g., parity_report.py supersedes verify_phase_*.sh).

---

## Deferral inventory + decisions

| # | Source | Item | Severity | Surface | Phase 4 intersect? | Decision | Rationale / next |
|---|---|---|---|---|---|---|---|
| 1 | Phase 1 ROADMAP | **01-G01 Gap E diagnosis** (mount-rail visual regression) | medium | `build_mount_rail_usds.py`, AIC_OBJECTS, mount-rail USDs | NO | PERSIST-DEFERRED | Phase 4 doesn't touch mount-rail authoring. Gating: visual-regression-diagnosis.md production. Not M1-blocking — trials don't depend on rail visual fidelity. |
| 2 | Phase 1 ROADMAP | **01-G02 Gap E fix** | medium | same as G01 | NO | PERSIST-DEFERRED | Gated on G01 verdict. Same M1-non-blocker rationale. |
| 3 | Phase 1 ROADMAP | **01-G03 Gap B closure** (PARITY-04: 17 per-frame ROS2PublishRawTransformTree publishers) | medium | `parity_publishers.py` | NO | PERSIST-DEFERRED | Phase 4 hasn't edited `parity_publishers.py`. PARITY-04 is `[~]` because rclpy publisher emits slashed names directly, bypassing the OGN approach G03 was scoped to. Effectively-superseded; G03 is dead code. **RECOMMEND**: re-scope G03 plan or drop. |
| 4 | Phase 1 ROADMAP | **01-G04 Gap A closure** (PARITY-03: gripper/left_finger_joint slash) | medium | `parity_publishers.py` | NO | PERSIST-DEFERRED | Same as G03 — rclpy publisher already emits the slash name (Plan 03 commit 079afd8). G04 is effectively-superseded. **RECOMMEND**: re-scope G04 plan or drop. |
| 5 | Phase 1 ROADMAP | **01-G05 Gap C closure** (TEX + setup_wrist_cameras prim path) | low | `extension.py::setup_wrist_cameras`, `sweep_textures.py` | YES (extension.py) | PERSIST-DEFERRED | Wrist cameras are explicitly M2 per PROJECT.md ("CheatCode doesn't read cameras"). Phase 4 touched extension.py but NOT setup_wrist_cameras. Surface intersection is incidental, not functional. **CLOSURE PATH**: M2, alongside POSE-03. |
| 6 | Phase 2 SUMMARY | **gripper/tcp full ingress path** (Pitfall 2 Option A complete chain) | low | `controller_loop.py::_apply_pose_cmd` | **YES** (PARITY-10 fix today touched _apply_pose_cmd write path) | **CLOSE-NOW** | Phase 4 just fixed shape-mismatch in same function. Once PARITY-10 round-trip is re-verified post-fix, sample a `frame_id=gripper/tcp` ingress to confirm the offset transform fires correctly. **Folded into**: 04-04 task list (verification step). |
| 7 | Phase 2 SUMMARY | **tcp_velocity.angular + tcp_error rotation components** (D-07 first-cut) | low-medium | `controller_loop.py::_publish_controller_state` | NO (Phase 4 didn't edit publisher) | PERSIST-DEFERRED | M1 success is per-trial outcome match against Gazebo. CheatCode doesn't consume rotation channels per `CheatCode.py:87-105` (uses linear position only). Revisit if a trial reveals need. |
| 8 | Phase 2 SUMMARY | **PyKDL fallback path** (D-02 alternative IK solver) | low | `controller_loop.py::_setup_kinematics` | NO (Lula has not failed; hook surface untouched) | PERSIST-DEFERRED | Trigger condition not met. Hook surface intact for any future swap. |
| 9 | Phase 2 SUMMARY | **Cartesian impedance simulation** (D-06 controller-side math) | low | C++ aic_controller side | NO (out-of-repo) | PERSIST-DEFERRED | Math lives in `~/Documents/aic/aic_controller`; sim-side just applies resulting joint stiffness/damping. No code-level intersection possible from this repo. |
| 10 | Phase 2 SUMMARY | **Off-limit contact discovery automation** | post-M1 | `scripts/snapshot_aic_eval_offlimit.sh` | NO | PERSIST-DEFERRED | Static 3-prefix mapping is canonical. Automation is redundancy for non-CheatCode policies. M2+. |
| 11 | Phase 2 SUMMARY | **Tare offset computation** (D-07 first-cut zero) | low | `controller_loop.py::_publish_controller_state` | NO (publisher untouched) | PERSIST-DEFERRED | Zero `fts_tare_offset` is contract per D-07; aic_controller computes tare from /fts_broadcaster/wrench history. ECHO-only field, downstream doesn't consume. |
| 12 | Phase 2 SUMMARY | **Step 6 of smoke test (PARITY-06 live-fire)** | low | `smoke_test_aic_controller.py` | NO | SUPERSEDED | Phase 4-04 `parity_report.py` is the canonical M1 verifier (per Phase 3 SUMMARY). Phase-2 smoke test is deprecated path. |
| 13 | Phase 2 SUMMARY | **End-to-end runtime smoke (verify_phase_2.sh)** | medium | `verify_phase_2.sh` | YES (parity_report.py replaces it) | SUPERSEDED | Phase 4-04 `parity_report.py` IS the canonical M1 ship gate. verify_phase_2.sh stays as a debug-only structural check. |
| 14 | Phase 3 SUMMARY | **/scoring/insertion_event live fire** | medium | `scoring_publishers.py::_on_insertion_contact_event` | **YES** (Phase 4-03 wired live, 04-04 will exercise it under CheatCode) | **CLOSE-NOW** | This is THE gating runtime test for PARITY-07 closure. Once 04-04 runs trial_1 end-to-end with model-zenoh resolved, the event MUST fire on plug-into-port contact. **Folded into**: 04-04 ship-gate criteria. |
| 15 | Phase 3 SUMMARY | **`_PORT_LINK_PATHS` parameterizable for non-default scenes** | medium | `scoring_publishers.py::set_port_link_paths` | **YES** (Plan 04-03 D-13 setter) | **ALREADY-CLOSED** | Plan 04-03 commit `e3aa68a` shipped `set_port_link_paths()` setter. Just needs traceability table flag flip. **Action**: mark closed in next traceability-table-refresh commit. |
| 16 | Phase 3 SUMMARY | **smoke_test_aic_cable.py + verify_phase_3.sh not shipped** | low | `scripts/` | NO | SUPERSEDED | Phase 4-04 `parity_report.py` supersedes per Phase 3 SUMMARY explicit guidance. |
| 17 | Phase 3 SUMMARY | **Hot-reload via `touch extension.py` corrupts rclpy state** | low | `extension.py` + `parity_publishers.py` | YES (Phase 4 has hot-reloaded extension.py multiple times) | PERSIST-DEFERRED | Workaround = restart sim. Real fix needs rclpy node lifecycle pattern that survives module reload — research-effort, not M1-blocking. Pinned in CLAUDE.md "Hot-reload" section. |
| 18 | Audit 2026-05-08 | **PARITY-05 wrench publishes garbage zeros** (`_physics_view` AttributeError) | HIGH | `extension.py::_on_physics_step_force` + `_lazy_init_articulation` | **YES** (Phase 4 commit `6fc6d26` touched both) | **CLOSE-NOW** | aic_adapter consumes via Observation.wrist_wrench → reaches model. Partial fix shipped 6fc6d26; root cause needs Isaac Sim 5.0 Articulation lifecycle deep-dive. **Folded into**: new Plan 04.5 OR 04-04 prerequisite. |
| 19 | Audit 2026-05-08 | **PARITY-09 motion deficit** (16% of commanded amplitude) | HIGH | `controller_loop.py::_apply_joint_cmd` | **YES** (Phase 4 commit `b682de1` touched) | **CLOSE-NOW** | Articulation writes reach DOFs after joint_indices fix; remaining 84% likely OGN articulation_controller PD interaction or stiffness scaling. **Folded into**: new Plan 04.5 — needs >95% commanded amplitude before 04-04 can pass any trial. |
| 20 | Audit 2026-05-08 | **PARITY-10 round-trip not re-verified post-fix** (`922a7d5`) | MEDIUM | `controller_loop.py::_apply_pose_cmd` | **YES** | **CLOSE-NOW** | Fix structural; runtime re-verification deferred today (TF tree desync after hot-reload). **Folded into**: same Plan 04.5 verification block. |
| 21 | Audit 2026-05-08 | **PARITY-12 stale traceability table** | MEDIUM | `.planning/REQUIREMENTS.md` | YES | **ALREADY-CLOSED** | Reconciled in commit `b6d1169` today. No further action. |
| 22 | Audit 2026-05-08 | **SCENE-05 cable doesn't fall under gravity** | LOW | `aic_unified_robot_cable_sdf.usd` via `author_cable_physics_offline.py` | NO | PERSIST-DEFERRED | Per PROJECT.md Out-of-Scope ("Cable visual fidelity beyond physics-needed accuracy"). Scoring is contact-based on plug-end geometric proximity, not cable shape. M2 retune. |
| 23 | Audit 2026-05-08 | **Force callback `[Force callback]` log spam still firing post-fix** | MEDIUM | `extension.py::_on_physics_step_force` | YES | **CLOSE-NOW** (paired with #18) | Same root cause as #18; same fix path. Same Plan 04.5 line item. |

---

## Summary by decision

| Decision | Count | Items |
|---|---|---|
| **CLOSE-NOW** | 5 | 6, 14, 18, 19, 20 |
| **ALREADY-CLOSED** (just needs flag flip) | 2 | 15, 21 |
| **SUPERSEDED** | 4 | 12, 13, 16 (and POSE-related items deferred via the forward-pull table) |
| **PERSIST-DEFERRED** | 12 | 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 17, 22 |

**Effective M1 backlog before Phase 4-04 can pass:** 5 CLOSE-NOW items + 1 model-zenoh blocker = 6 things. The 5 CLOSE-NOW items group naturally into one new plan:

### Recommended new plan: 04-3.5 (or insert as a separate phase 4.5)

**Plan title:** "Runtime payload + round-trip gate before 04-04 ship-gate exercise"

**Tasks:**
1. **PARITY-09 motion deficit hunt** — investigate the 84% deficit (OGN articulation_controller residual? Stiffness vs cable mass coupling? Per-tick PD target reset?). Goal: ≥95% commanded amplitude on shoulder_lift_joint round-trip.
2. **PARITY-10 round-trip re-verification** post-fix `922a7d5`. Same target: tool0 reaches commanded delta within tolerance.
3. **gripper/tcp ingress path verification** (item #6): publish a `frame_id=gripper/tcp` MotionUpdate, confirm offset transform fires.
4. **PARITY-05 wrench root-cause hunt** (items #18 + #23): why does `get_measured_joint_forces()` raise `_physics_view` AttributeError even when `is_physics_handle_valid()` returns True. Use `nvidia-suite-docs` skill for Isaac Sim 5.0 Articulation lifecycle.
5. **Truthful state refresh after fixes** — flip PARITY-05/09/10 from `[~]` back to `[x]` once each runtime test passes.

**Then:** Phase 4-04 (parity_report.py + first ship-gate exercise) becomes meaningful — it'll actually have working actuation + sensing to test against trial outcomes.

### PERSIST-DEFERRED items needing tracker entries

For traceability honesty, items 1, 2, 5, 17, 22 should appear in REQUIREMENTS.md inline checkboxes as `[~]` (not silent omissions). Items 3, 4 should be re-scoped or marked SUPERSEDED in ROADMAP.md gap-plans block (G03/G04 are dead code post-079afd8).

---

## Process notes (first-run dogfood)

**What worked:**
- The deferral list was already centralized in SUMMARY.md "Deferred items" tables. Easy to gather.
- Surface-intersection check was tractable — for each deferred item I could grep Phase 4 commits for the file path.
- The 4-decision system (CLOSE-NOW / ALREADY-CLOSED / SUPERSEDED / PERSIST-DEFERRED) covers all cases cleanly.

**What was clunky / needs iteration:**
- Phase 1 SUMMARY didn't have a single "Deferred items" table; gap-plans live in ROADMAP.md instead. Sweep had to query two sources. **Fix**: future Phase SUMMARY templates should include the table + cross-reference any ROADMAP-tracked gap-plans.
- "Surface intersection" is sometimes incidental (Phase 4 touched extension.py but NOT setup_wrist_cameras). The intersection rule needs file:function granularity, not just file. **Fix**: surface registry (per CLAUDE.md execute-phase enforcement) should track function names, not just file paths.
- 4 of the PERSIST-DEFERRED items (3, 4, 12, 16) are SUPERSEDED but I marked them PERSIST-DEFERRED in earlier rows. Distinction blurs when a deferral is "still listed but factually obsolete." **Fix**: SUPERSEDED is the right status — refining the decision matrix.

**Upstream contribution observation:** the 4-status matrix (CLOSE-NOW / ALREADY-CLOSED / SUPERSEDED / PERSIST-DEFERRED) is generic enough that GSD upstream could ship it as a `templates/backlog-sweep.md` template without robotics-specific assumptions. Even software-only projects accumulate deferrals; the sweep just runs less frequently for them.

---

*Sweep date: 2026-05-08*
*Sweep generator: manual (first dogfood run; future runs target `gsd-tools backlog-sweep <phase>` CLI per CLAUDE.md "execute-phase halt" mechanic)*
