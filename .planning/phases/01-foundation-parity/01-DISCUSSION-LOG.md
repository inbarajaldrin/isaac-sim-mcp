# Phase 1: Foundation Parity - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-02
**Phase:** 01-foundation-parity
**Areas discussed:** Parity reference source, Robot USD provenance, Asset vendoring strategy, Sweep & validation method, plus follow-up rounds covering /tf publisher, cable handling in Phase 1, RG2→Hand-E doc fix scope, verify artifact format, /joint_states publisher source, quick_start refactor scope, AIC repo not-present behavior, vendoring scope, MDL/material fix style, snapshot version pinning, verify script runtime.

---

## Gray Area Selection

| Option | Description | Selected |
|--------|-------------|----------|
| Parity reference source | Live aic_eval Docker container topics vs sample_config.yaml as source of truth. Resolves PARITY-12 timing conflict. | ✓ |
| Robot USD provenance | Keep baked aic_unified_robot_cable_sdf.usd vs re-import ur_gz.urdf.xacro through xacro→URDF→Isaac Sim URDF importer. | ✓ |
| Asset vendoring strategy | How aic_assets/models/* meshes/USDs land in exts/aic-dt/assets/ — copy/symlink/sync/import. | ✓ |
| Sweep & validation method | How to systematize the discover-by-loading texture audit and prove TF tree matches Gazebo. Includes _sim/_real cleanup scope. | ✓ |

**User selected all four.**

---

## Area 1 — Parity reference source

| Option | Description | Selected |
|--------|-------------|----------|
| Live aic_eval container | Run docker container, capture ros2 topic list / topic info / view_frames once into exts/aic-dt/docs/topic-parity-reference.md. Catches YAML↔live divergences. | |
| sample_config.yaml only | Trust the scoring.topics block as authoritative. | |
| Both: YAML for scoring, live for everything else | Use YAML for scoring topics, live snapshot for the rest. | |
| Live now, but only for Phase 1 scope | Capture live snapshot for /joint_states and /tf only; defer PARITY-12 to its mapped Phase 3. | |

**User's choice (free-text):** "this is a question that needs to be investigated and not asked to me"
**Notes:** User pushed back — empirical questions about live system behavior are research items, not user decisions. Locked outcome: researcher captures live aic_eval snapshot as Phase 1 deliverable; CLAUDE.md timing (Phase 1 first deliverable) wins over REQUIREMENTS.md mapping (PARITY-12 in Phase 3) for the *reference snapshot*; full audit table still ships in Phase 3.

---

## Area 2 — Robot USD provenance

| Option | Description | Selected |
|--------|-------------|----------|
| Re-import xacro fresh | Run xacro on ur_gz.urdf.xacro, import via Isaac Sim URDF Importer, post-process drives. | |
| Keep current baked USD, audit-only | Keep aic_unified_robot_cable_sdf.usd, write a script to prove joint set + frame names + hierarchy match xacro. | |
| Hybrid: keep robot, re-import attachments | Keep cooked UR5e core, re-import camera mount + Basler + RG2 + Axia80 as separate sub-USDs. | |
| Defer until I see the xacro re-import outcome | Researcher attempts xacro path, reports back, lock after evidence. | |

**User's choice (free-text):** "i dont think they use rg2 gripper from where did you get that idea. verifying everuthong from gazebo is inside isaacsim is a requirement i thought that aic already provided a unified usd for that"
**Notes:**
- User identified two errors in the question framing: (1) gripper is Robotiq Hand-E, not RG2 (the question and existing project docs were wrong); (2) AIC already provides a unified Isaac Sim USD.
- Verified: `aic_description/urdf/ur_gz.urdf.xacro` includes `Robotiq Hand-E/robotiq_hande_macro.xacro`. No RG2.
- Verified via md5sum: `aic_unified_robot_cable_sdf.usd` and `scene/aic.usd` are byte-identical between `exts/aic-dt/assets/` and AIC repo's `Intrinsic_assets/`. AIC's USD is the canonical source.
- Locked outcome: use AIC's official USD (already in our tree); no xacro re-import; whole-repo doc fix for RG2→Hand-E.

---

## Area 3 — Asset vendoring strategy

| Option | Description | Selected |
|--------|-------------|----------|
| Mirror tree via symlink | Symlink exts/aic-dt/assets/ into Intrinsic_assets/. Zero copy, zero divergence. | |
| Mirror tree via rsync script | rsync at setup or on demand; copy step + drift risk; self-contained. | |
| Direct file:// reference into AIC repo | _get_assets_folder() resolves to AIC repo path via env var; couples to a specific path. | |
| Investigate first, then decide | Verify what unified USD references; recommend strategy with evidence. | |

**User's choice (free-text):** "isnt importing assets onetime thhng if verified we dont need to worry about the aic root repo assets because we ,might ahve to modify the usd as needed in this isaacsim repo"
**Notes:** User reframed: vendoring is a one-time copy, not an ongoing sync. After verification, this repo owns the assets locally and may modify them in place (cable physics workaround, texture rebinding, etc.). AIC repo is the *origin* but not a runtime dependency. Folder layout preserves AIC's original (`assets/NIC Card/` etc.) so USD references resolve.

---

## Area 4 — Sweep & validation method (3 sub-questions)

### Sub-Q1: Texture sweep methodology

| Option | Description | Selected |
|--------|-------------|----------|
| Scripted log-grep loop (Recommended) | Load scene → grep kit log → log fixes in texture-sweep.md → iterate. | ✓ |
| Manual viewport walk + checklist | Visual inspection only. | |
| Both — log-grep first, viewport pass after | 2x time, most thorough. | |

**User's choice:** Scripted log-grep loop (Recommended)

### Sub-Q2: TF tree validation

| Option | Description | Selected |
|--------|-------------|----------|
| view_frames diff (Recommended) | Capture aic_eval's frames.gv, diff against Isaac Sim's, scripted. | ✓ |
| Manual side-by-side PDF inspection | Eyeball both PDFs; risky for ~20-frame tree. | |
| Frame-name set check only | Skip parent-child verification; weaker evidence. | |

**User's choice:** view_frames diff (Recommended)

### Sub-Q3: DX-01 cleanup scope

| Option | Description | Selected |
|--------|-------------|----------|
| Strict: zero _sim/_real anywhere (Recommended) | Rename everything now per Phase 1 SC #5. | ✓ |
| Light: only delete placeholder atoms | Leave wrench_sim/camera_rgb_sim for owning phases. | |
| Strict on ROS topics, USD prim paths deferred | Pragmatic middle ground. | |

**User's choice:** Strict: zero _sim/_real anywhere (Recommended)

---

## Round 2 — Follow-up gray areas (4 sub-questions)

### TF publisher implementation

| Option | Description | Selected |
|--------|-------------|----------|
| Isaac Sim TF action graph (Recommended) | isaacsim.ros2.bridge OmniGraph nodes inside aic-dt action graph. | ✓ |
| External robot_state_publisher ROS node | Canonical ROS way; extra process. | |
| Researcher decides after probing | Lock based on evidence. | |

**User's choice:** Isaac Sim TF action graph (Recommended)

### Cable handling in Phase 1

| Option | Description | Selected |
|--------|-------------|----------|
| Keep SetActive(False) workaround (Recommended) | Cable physics is Phase 3; defer cleanly. | ✓ |
| Strip cable from unified USD for Phase 1 | Clean phase boundary; two USD variants to maintain. | |
| Leave cable active and accept post-play wedge | Risk: TEX sweep blocked. | |

**User's choice:** Keep SetActive(False) workaround (Recommended)

### RG2 → Robotiq Hand-E doc fix scope

| Option | Description | Selected |
|--------|-------------|----------|
| Whole-repo + project docs (Recommended) | Fix every RG2 mention in extension.py, .planning/*, CLAUDE.md, exts/aic-dt/docs/*. | ✓ |
| Code + project docs only | Skip CLAUDE.md and exts/aic-dt/docs/*. | |
| Code only — doc rewrites are Phase 4 territory | Just extension.py docstrings. | |

**User's choice:** Whole-repo + project docs (Recommended)

### Phase 1 verification artifact format

| Option | Description | Selected |
|--------|-------------|----------|
| Re-runnable bash script + generated docs (Recommended) | scripts/verify_phase_1.sh produces topic-parity-reference.md, texture-sweep.md, tf-tree-diff.txt. | ✓ |
| One-time captured artifacts only | Pasted into a single phase-1-verification.md. | |
| Markdown-only doc, no script | Hand-written; no automation. | |

**User's choice:** Re-runnable bash script + generated docs (Recommended)

---

## Round 3 — Follow-up gray areas (4 sub-questions)

### /joint_states publisher source

| Option | Description | Selected |
|--------|-------------|----------|
| Match live aic_eval framing (Recommended) | Snapshot live joint_state_broadcaster output; replicate joint name set, ordering, frame_id, rate. | ✓ |
| Raw Isaac Sim articulation → /joint_states | Whatever ordering articulation API gives. | |
| Researcher decides after probing live aic_eval | Lock after evidence. | |

**User's choice:** Match live aic_eval framing (Recommended)

### quick_start refactor scope in Phase 1

| Option | Description | Selected |
|--------|-------------|----------|
| Minimal: only remove deleted atom calls (Recommended) | Surgical to phase scope. | |
| Broader: reorganize for clean per-phase clubbing | Restructure now so Phase 2/3 atoms drop in cleanly; partly does Phase 3 DX-03 work early. | ✓ |
| Don't touch quick_start in Phase 1 | Defer all to Phase 3; risk of broken state in between. | |

**User's choice:** Broader (deviation from recommended)
**Notes:** User wants less churn over the full milestone — restructure now to club atoms cleanly per the future per-phase ordering. Partly does Phase 3 DX-03 work early.

### AIC repo not-present behavior

| Option | Description | Selected |
|--------|-------------|----------|
| Extension runs; verification scripts warn (Recommended) | Vendored assets work; verify scripts skip-with-warning on missing AIC repo. | ✓ |
| Hard requirement: error if AIC repo missing | Both extension and scripts hard-fail. | |
| Out of scope for Phase 1 | Document assumption only. | |

**User's choice:** Extension runs; verification scripts warn (Recommended)

### Vendoring scope

| Option | Description | Selected |
|--------|-------------|----------|
| Full Intrinsic_assets/ tree (Recommended) | Copy everything; few hundred MB but no surprises. | |
| Phase 1 subset only | Copy what M1 spawn touches; smaller commit. | |
| Inspect refs first, vendor exactly what's needed | Researcher walks USD reference graph, produces precise dependency manifest. | ✓ |

**User's choice:** Inspect refs first (deviation from recommended)
**Notes:** User wants precision — researcher uses usdview/usdcat to walk the unified USD's reference graph before any copy.

---

## Round 4 — Follow-up gray areas (3 sub-questions)

### MDL/material rebinding strategy

| Option | Description | Selected |
|--------|-------------|----------|
| USD override layer (Recommended) | Non-destructive aic-dt-fixes.usd layer that re-binds. | |
| Edit vendored USDs in place | Modify USD files directly; vendored tree drifts from AIC source. | ✓ |
| Re-vendor source asset with fixes | Regenerate USD from SDF/mesh sources. | |

**User's choice (free-text):** "edit vendor in place once we are done with milestone 1 we dont have to worry abou the aic source we can actually backfill if required"
**Notes:** Aligns with Area 3 framing — assets are owned locally post-M1, backfill from AIC if needed. Override-layer indirection rejected for explicitness.

### aic_eval Docker version pinning

| Option | Description | Selected |
|--------|-------------|----------|
| Image SHA-256 digest (Recommended) | docker inspect digest pasted into topic-parity-reference.md. | ✓ |
| AIC repo git commit + image tag | Commit + :latest at snapshot day. | |
| Don't pin — refresh as needed | No pinning ceremony. | |

**User's choice:** Image SHA-256 digest (Recommended)

### Verify script runtime context

| Option | Description | Selected |
|--------|-------------|----------|
| Hybrid: launch Isaac Sim if not running, else use it (Recommended) | Detect port 8768; attach if up else cold-launch. | ✓ |
| Requires Isaac Sim already running | Simpler script. | |
| Headless via launch_postload.py only | Always cold-boot; minutes per run. | |

**User's choice:** Hybrid (Recommended)

---

## Claude's Discretion

- Exact OmniGraph node names + wiring inside the TF action graph (D-10).
- Exact subset to vendor under D-05 — researcher's USD reference graph walk is the authoritative input.
- Format of `topic-parity-reference.md`.
- `view_frames` diff script implementation language.
- Log-grep regex patterns in D-07.

## Deferred Ideas

- Camera resolution match → CAM-01 (M2).
- Cable physics strategy → SCENE-05 (Phase 3, `nvidia-suite-docs` skill).
- Full ros2_control surface → PARITY-09/10/11 (Phase 2).
- Object TF frames CheatCode reads → SCENE-06 (Phase 3).
- Headless/CI integration → M2+.
- Override-layer USD pattern → explicitly rejected (D-06).
- External robot_state_publisher → explicitly rejected (D-10).
- PARITY-12 *full audit table* → stays in Phase 3 (only the *reference snapshot* lands in Phase 1).
