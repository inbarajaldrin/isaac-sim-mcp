# Phase 1: Foundation Parity - Context

**Gathered:** 2026-05-02
**Status:** Ready for planning

<domain>
## Phase Boundary

Isaac Sim loads the *same* UR5e + Robotiq Hand-E + 3 wrist cameras + task-board + AIC enclosure assets the AIC repo's Gazebo bringup loads, with no missing/broken textures, and publishes the same passive sensor topics (`/joint_states`, `/tf`, `/tf_static`) Gazebo publishes — same joint names, same frame names, same parent-child hierarchy. All `_sim`/`_real` placeholder topics from the scaffold are removed from the production topic surface. **Scope:** PARITY-01, PARITY-02, PARITY-03, PARITY-04, TEX-01, TEX-02, TEX-03, DX-01.

Out of this phase: controller loop closure (Phase 2), parametric task-board spawn (Phase 2), cable physics strategy (Phase 3), object TF frames CheatCode reads (Phase 3), trial loader and end-to-end engine run (Phase 4).

</domain>

<decisions>
## Implementation Decisions

### Parity reference source

- **D-01:** **Live `aic_eval` Docker container is the canonical source of truth** for "what Gazebo publishes." User explicitly stated parity reference is empirical, not a config-document choice. First Phase 1 deliverable: snapshot the live container (`ros2 topic list`, `ros2 topic info <name>`, `ros2 topic echo /joint_states --once`, `ros2 run tf2_tools view_frames`) into `exts/aic-dt/docs/topic-parity-reference.md`. Resolves the conflict between REQUIREMENTS.md (PARITY-12 mapped to Phase 3) and CLAUDE.md ("first concrete deliverable of Phase 1") in favor of CLAUDE.md: the reference snapshot lands in Phase 1; the *full cross-phase parity audit table* (PARITY-12 deliverable as scoped) still ships in Phase 3.

### Robot USD provenance

- **D-02:** **Use AIC's official `aic_unified_robot_cable_sdf.usd`** from `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/Intrinsic_assets/`. Verified byte-identical (md5: `46616697c057701ae2025d44ace26844`) to the file already at `exts/aic-dt/assets/robot/aic_unified_robot_cable_sdf.usd`. Same applies to `scene/aic.usd`. **Do NOT re-import xacro** — AIC already provides the unified Isaac Sim USD; their tree is canonical, ours mirrors it.
- **D-03:** **Gripper is Robotiq Hand-E, NOT RG2.** Confirmed via `aic_description/urdf/ur_gz.urdf.xacro` (`xacro:include filename="$(find aic_assets)/models/Robotiq Hand-E/robotiq_hande_macro.xacro"`). The current code and project docs call it "RG2" — wrong. Phase 1 cleanup scope: **whole repo + project docs**. Targets at minimum:
  - `exts/aic-dt/aic_dt/extension.py` lines 166, 999, 2357 (docstrings, log lines, return messages)
  - `.planning/PROJECT.md` ("UR5e + RG2 + cable" line)
  - `.planning/REQUIREMENTS.md` (validated section + scope mentions)
  - `.planning/ROADMAP.md` (any RG2 references)
  - `CLAUDE.md` repo-root (Robot: line, cable note)
  - `exts/aic-dt/docs/README.md`, `exts/aic-dt/docs/CHANGELOG.md`
- **D-04:** **Cable subtree stays `SetActive(False)` for Phase 1.** Cable physics is Phase 3 work (SCENE-05). Phase 1's PARITY/TEX deliverables don't need an active cable. Document the workaround as a known Phase-3 work item in `topic-parity-reference.md`. Verify the texture sweep still inspects cable-asset materials (cable assets are loaded into the stage, just simulation-disabled).

### Asset vendoring strategy

- **D-05:** **One-time vendoring of a precise subset of `Intrinsic_assets/`** into `exts/aic-dt/assets/`, **preserving AIC's original folder layout** (`assets/NIC Card/`, `assets/SC Port/`, etc. — capitalized, with spaces — to keep USD references resolvable). Researcher's job: walk the unified USD's reference graph (usdview/usdcat) and produce an exact dependency manifest first, then vendor exactly that set. The current snake_case `assets/objects/{nic_card, sc_plug, …}` reorganization is retired during the vendoring step. After verification, **`exts/aic-dt/assets/` is canonical** — no ongoing sync, AIC repo path is documented as the *origin* but not a runtime dependency.
- **D-06:** **Edit vendored USDs in place** when the texture/MDL sweep finds broken bindings. User's framing: "after milestone 1 we don't have to worry about the AIC source; we can backfill if required." No override-layer indirection — keep the asset tree explicit and inspectable. If AIC publishes upstream fixes later, manually merge them in.

### Sweep & validation method

- **D-07:** **Texture/MDL sweep is a scripted log-grep loop**: load M1 scene → grep `~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log` for `MDL`, `texture`, `missing`, `pink`, `fallback`, `not found` → each warning becomes a row in `exts/aic-dt/docs/texture-sweep.md` (asset, problem, fix). Iterate fix → re-load → re-grep until zero asset-related warnings. Reproducible audit; satisfies TEX-01/02/03 with a shipping artifact, not vibes.
- **D-08:** **TF tree validation via `view_frames` diff script.** Capture live `aic_eval`'s `frames.gv` (during the D-01 reference snapshot) and Isaac Sim's `frames.gv` separately. `scripts/diff_tf_tree.py` parses both `.gv` files and prints frame-set diff + parent-child edge diff. Zero diff = pass. Re-runnable, evidence-based proof for PARITY-04 SC #3.
- **D-09:** **DX-01 cleanup is strict — zero `_sim`/`_real` anywhere on production surface.** Concrete targets surfaced from `extension.py`:
  - Camera RGB topics: `center_camera_rgb_sim`, `left_camera_rgb_sim`, `right_camera_rgb_sim` → rename to whatever live `aic_eval` publishes (per CLAUDE.md, single `/intel_camera_rgb_raw` may be the truth — confirm in D-01 snapshot).
  - Wrench topic: `force_torque_sensor_broadcaster/wrench_sim` → `/force_torque_sensor_broadcaster/wrench` (matches CLAUDE.md observation; note REQUIREMENTS.md PARITY-05 said `/fts_broadcaster/wrench` — live snapshot is the tiebreaker).
  - MCP atoms `objects_poses_sim` and `sync_real_poses` (and the `/objects_poses_real` subscription): **deleted entirely.** Phase 3 publishes GT directly into `/tf`; the placeholder sim↔real bridge is unneeded.
  - USD prim paths (`/World/workspace_camera_sim`, etc.): renamed too (USD prim paths are not topics but a literal grep for `_sim`/`_real` should return zero production-surface hits per Phase 1 SC #5).
- **D-10:** **`/tf` and `/tf_static` published via Isaac Sim TF action graph** (`isaacsim.ros2.bridge` OmniGraph nodes inside the existing aic-dt action graph). NOT an external `robot_state_publisher` ROS node. Self-contained, matches the pattern other extensions in this repo use. Researcher confirms exact node name(s) and inputs against `nvidia-suite-docs` / `isaac-sim-extension-dev` skills.
- **D-11:** **`/joint_states` matches live `aic_eval` framing exactly** — joint name set, ordering, `header.frame_id`, publish rate. Implementation: `isaacsim.ros2.bridge` ROS2PublishJointState node (or equivalent) with explicit `jointNames` input populated from the snapshot. Phase 2's controller-loop work depends on this being right; getting it wrong here means PARITY-09/10 silently misbehave later.

### Phase 1 ops & scope

- **D-12:** **`quick_start` refactor scope is broader** than the minimal "drop deleted atom calls." Phase 1 reorganizes `quick_start` to club atoms cleanly per the future per-phase ordering: load scene → load robot → setup tf graph → setup camera publishers → setup wrench publisher → add objects → setup pose publisher → start sim. Partly does Phase 3's DX-03 work early so subsequent phases can drop new atoms in cleanly. User's call (deviates from the recommended minimal-scope option in favor of less churn over the milestone).
- **D-13:** **Extension runs without `~/Documents/aic` checked out** (assets are vendored, so runtime is self-contained). `scripts/verify_phase_1.sh` detects missing AIC repo and prints a clear message — exits non-zero on the snapshot-update step but skips it gracefully on a verify-only run. CLAUDE.md documents this assumption.
- **D-14:** **`aic_eval` Docker snapshot pinned via image SHA-256 digest** in `topic-parity-reference.md`. Captured via `docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest` followed by `docker inspect --format='{{index .RepoDigests 0}}'`. Future re-snapshots use that digest, not `:latest`. Survives ghcr tag drift.
- **D-15:** **`scripts/verify_phase_1.sh` is hybrid-runtime.** Detects whether MCP port 8768 is responding. If yes: runs MCP commands against the attached extension. If no: invokes `~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt` (or `launch_postload.py` cold-cache fallback), runs the checks, optionally tears down. Same script works in dev (attached) and one-shot/CI-style (cold) modes.

### Claude's Discretion

- Exact OmniGraph node names + wiring inside the TF action graph (D-10) — researcher decides via `isaac-sim-extension-dev`/`nvidia-suite-docs` skills.
- Exact subset to vendor under D-05 — researcher walks the USD reference graph and produces the manifest before any copy happens.
- Format of `topic-parity-reference.md` — table layout, structure, what counts as a "topic surface row" (Claude follows the existing extension-doc style).
- `view_frames` diff script implementation language (Python vs bash, GraphViz vs networkx parser).
- The exact log-grep regex patterns in D-07 — refine after first run.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### This project
- `.planning/PROJECT.md` — Vision, constraints, key decisions, repo split rationale, RG2→Hand-E correction needed.
- `.planning/REQUIREMENTS.md` — PARITY-01..04, TEX-01..03, DX-01 acceptance criteria. Note: live aic_eval snapshot is the parity tiebreaker, not the doc's text where they conflict.
- `.planning/ROADMAP.md` §"Phase 1: Foundation Parity" — goal + success criteria the verifier will check against.
- `CLAUDE.md` (repo root) — launch flow, cache management discipline (`prime_usd_cache.py`), real Kit log location, MCP socket protocol, **live-vs-YAML divergence note** (PARITY-12-in-Phase-1 directive), known broken streaming wrapper.

### AIC reference repo (read-only)
- `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` — Authoritative robot definition: UR5e + Axia80-M20 + Camera Mount + Basler trio + **Robotiq Hand-E** (NOT RG2). Source for all kinematics-divergence checks under PARITY-01.
- `~/Documents/aic/aic_engine/config/sample_config.yaml` §`scoring.topics` — Documented topic surface (used as cross-check; live `aic_eval` overrides where they disagree).
- `~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/Intrinsic_assets/` — **Authoritative origin** of vendored USDs (`aic_unified_robot_cable_sdf.usd`, `scene/aic.usd`, `assets/<Object>/...`). Researcher walks this tree's USD reference graph to determine the exact vendoring subset.
- `~/Documents/aic/aic_assets/models/<Object>/model.sdf` — Original SDF source for each asset (Gazebo-side); reference if a USD MDL/texture rebind requires going back to mesh sources.
- `~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py` — Gazebo bringup whose *outcomes* (scene state + topic surface) Phase 1 mirrors.
- `~/Documents/aic/CLAUDE.md` — AIC repo's own dev workflow notes (Docker-only, GLIBC pin).

### This extension's current state
- `exts/aic-dt/aic_dt/extension.py` — `MCP_TOOL_REGISTRY` (line 135), `_cmd_<name>` handler convention, current `_sim`/`_real` topic strings (lines 108/116/124/221/224/268/487/876/921/1066/1102/1122/1293/1326/1329/1690/1696), current `load_robot` cable workaround (`SetActive(False)` on `/World/UR5e/cable`).
- `exts/aic-dt/scripts/launch_postload.py` — Cold-cache-safe boot path.
- `exts/aic-dt/docs/{README.md, CHANGELOG.md}` — Current state (still references "ur5e-dt" — DX-05 scope but RG2→Hand-E correction touches them in Phase 1 too).

### Skills (canonical Isaac Sim references)
- `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` — Per CLAUDE.md, the source of truth for extension lifecycle, USD/physics, action graphs, MCP socket protocol, troubleshooting. Researcher invokes this before designing the TF action graph (D-10) or the joint_state publisher (D-11).
- `~/.claude/skills/nvidia-suite-docs/SKILL.md` — For OmniGraph node lookup, USD authoring docs, `isaacsim.ros2.bridge` specifics. Researcher invokes for node-name confirmation.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

- **`MCP_TOOL_REGISTRY` (extension.py:135)**: Single source of truth for tool metadata. Every Phase 1 capability change (renames, deletions, new TF/joint-state atoms) edits this dict. `_cmd_<name>` handler convention preserves the atomic + clubbed model.
- **Existing camera publisher action graph code (extension.py around line 1066+, `_create_camera_actiongraph`, `setup_wrist_cameras`)**: Already builds OmniGraph nodes for ROS2 publishing. Phase 1 reuses this; only changes are topic-name strings and parameterizing for the live-snapshot-derived names.
- **Existing force/torque publisher action graph (`setup_force_publisher`, `_cmd_setup_force_publisher`)**: Same — keep the graph structure, fix the topic string in `publisher.inputs:topicName` (line 1102).
- **Asset folder resolver (`_get_assets_folder`, `_local_asset` near line 37)**: Already does `file://` resolution under `exts/aic-dt/assets/`. Vendoring (D-05) preserves AIC's folder layout so this resolver keeps working with the unified USD's relative references.
- **`load_robot` (line 933)**: Current path imports `aic_unified_robot_cable_sdf.usd`, configures articulation drives (`stiffness=2000, damping=100, max_force=87` from Isaac Lab), calls `cable_prim.SetActive(False)`. Phase 1 keeps the import + drive config + cable workaround (D-04); changes are docstring fixes (D-03) and any joint-name verification follow-on.
- **`scripts/launch_postload.py`** + **`isaac-sim-extension-dev` skill's `isaacsim_launch.sh`**: Both used by `verify_phase_1.sh` (D-15) for the cold-launch fallback path.
- **`prime_usd_cache.py`** (in `isaac-sim-extension-dev` skill): Required tool for any session that hits a cold `DerivedDataCache` — Phase 1 plan should include taking a snapshot after the first successful load.

### Established Patterns

- **Atomic + clubbed MCP model**: every new capability = one `MCP_TOOL_REGISTRY` entry + one `_cmd_<name>` handler + one UI button + (where appropriate) inclusion in `quick_start`. Phase 1's TF publisher and per-camera/wrench rename atoms follow this shape.
- **Action graph for ROS publishing**: ROS2 topics published via OmniGraph node graphs, not from the Python main loop. TF publishers (D-10) follow this convention.
- **Local file:// asset resolution**: No Nucleus dependency. Vendoring (D-05) preserves this constraint.
- **Hot-reload-safe extension state**: `on_shutdown` cleanly tears down sockets/timers; new code follows.

### Integration Points

- **`MCP_TOOL_REGISTRY`** (extension.py:135) — every tool change funnels here. Phase 1 deletes `objects_poses_sim` + `sync_real_poses` entries (D-09), adds at minimum `setup_tf_publisher` (or extends an existing graph atom), edits camera/wrench atom topic strings.
- **`quick_start` clubbing path** (extension.py:2597) — Phase 1 broader-scope refactor (D-12) touches this; downstream phases drop new atoms in.
- **`isaacsim.ros2.bridge` OmniGraph node namespace** — TF + joint state + camera + wrench publishers all live here. Researcher confirms exact node IDs against the live Isaac Sim 5.0 install.
- **Kit log path** (`~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log`) — texture sweep (D-07) greps this; verify script (D-15) reads it.

</code_context>

<specifics>
## Specific Ideas

- **"Verifying everything from gazebo is inside Isaac Sim is a requirement"** (user, area 2): Phase 1's verify script (D-15) is the proof artifact for this — re-runnable, evidence-based, not a written claim.
- **"AIC already provided a unified USD"** (user, area 2): Pivot away from xacro re-import — AIC's `aic_unified_robot_cable_sdf.usd` is the canonical Isaac Sim USD; we mirror it.
- **"Importing assets is one-time thing if verified"** (user, area 3): Vendoring is a one-shot copy + verify step. After M1, `exts/aic-dt/assets/` is canonical and AIC repo is just the origin record.
- **"After milestone 1 we don't have to worry about the AIC source; we can actually backfill if required"** (user, area 4 fix-style): Justifies in-place USD edits (D-06) over override layers — we own the asset tree post-M1.
- **"This is a question that needs to be investigated and not asked to me"** (user, area 1 reference source): Establishes a precedent for this project — *empirical answers come from running the system, not a multiple-choice menu*. Researcher should treat ambiguities about live behavior as investigation tasks, not question-bank entries.

</specifics>

<deferred>
## Deferred Ideas

- **Camera resolution match (640×480 → 1152×1024)** — CAM-01, M2 work. Phase 1 keeps current 640×480 since CheatCode doesn't read cameras.
- **Cable physics strategy (deformable / articulated chain / rigid+visual hybrid)** — Phase 3 SCENE-05; researched via `nvidia-suite-docs` skill at that time.
- **Full `ros2_control` surface (`scaled_joint_trajectory_controller`, `force_mode_controller`, etc.)** — Phase 2 PARITY-09/10/11. Phase 1's joint_state publisher does NOT need to emulate the full ros2_control surface — only the `/joint_states` topic with matching ordering.
- **Object TF frames CheatCode reads (`{cable_name}/{plug_name}_link`, port frames)** — Phase 3 SCENE-06. Phase 1's TF tree is robot + cameras + tabletop only.
- **Headless / CI integration** — M2+. `verify_phase_1.sh` (D-15) hybrid runtime is dev-friendly first.
- **Override-layer USD pattern for fixes** — explicitly rejected (D-06); revisit only if vendored-tree edits become unmanageable.
- **External `robot_state_publisher` ROS node** — explicitly rejected (D-10); revisit only if Isaac Sim TF action graph proves insufficient.
- **PARITY-12 cross-phase audit table as a comprehensive deliverable** — that scoped form of the audit stays in Phase 3 per REQUIREMENTS.md. Phase 1 ships the *reference snapshot* that audit will compare against.

</deferred>

---

*Phase: 01-foundation-parity*
*Context gathered: 2026-05-02*
