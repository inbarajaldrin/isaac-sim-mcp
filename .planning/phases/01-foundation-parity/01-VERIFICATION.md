---
phase: 01-foundation-parity
verified: 2026-05-02T22:00:00Z
status: gaps_found
score: 5/9 must-haves verified
overrides_applied: 0
gaps:
  - truth: "ros2 topic echo /joint_states --once returns the same joint names in the same ordering Gazebo's /joint_states publishes (live snapshot: 7 joints alphabetical, including gripper/left_finger_joint)"
    status: failed
    reason: "ROS2PublishJointState OGN node has no nameOverrides input. The USD prim leaf name is gripper_left_finger_joint (underscore); aic_adapter::ReorderJointState expects gripper/left_finger_joint (slash) at line 86. Isaac Sim will publish the underscore form. aic_adapter silently drops unrecognized joint names, so this joint is missing from every Observation. Documented inline in setup_joint_state_publish_action_graph docstring and CHANGELOG Known-Phase-3 callouts."
    artifacts:
      - path: "exts/aic-dt/aic_dt/extension.py"
        issue: "setup_joint_state_publish_action_graph defers the gripper/left_finger_joint name override with comment 'expected to FAIL pre-follow-up'. No runtime name mapping exists."
    missing:
      - "A wrapper republisher node that renames gripper_left_finger_joint -> gripper/left_finger_joint OR a USD-side joint prim rename so the prim leaf contains the slash-form name."
  - truth: "ros2 run tf2_tools view_frames against Isaac Sim produces a TF tree whose robot + gripper + camera frames match Gazebo's reference TF tree (same names, same parent-child links)"
    status: failed
    reason: "ROS2PublishTransformTree derives frame_id from USD prim leaf names. The unified USD has 16 underscore-form prims (e.g., gripper_hande_base_link) that must publish as slash-form frames (gripper/hande_base_link). Additionally, the aic_world synthesized edge (world -> aic_world) has no driving prim. All 17 mismatches are enumerated in usd_prim_inventory.txt [Override list for Plan 06]. ROS2PublishRawTransformTree takes static translation/rotation inputs only -- it does NOT read prim transforms. Per-frame override design needs additional pose-source nodes per frame (IsaacReadOdometry/OgnGetPrimWorldPose). Deferred per Plan 06 PLAN.md L51/L302."
    artifacts:
      - path: "exts/aic-dt/aic_dt/extension.py"
        issue: "setup_tf_publish_action_graph uses two ROS2PublishTransformTree nodes with default prim-name->frame_id mapping. The 16 underscore->slash mismatches and 1 synthesized aic_world edge are deferred with inline docstring note."
      - path: ".planning/phases/01-foundation-parity/usd_prim_inventory.txt"
        issue: "Lists 17 required overrides (16 UNDERSCORE-MISMATCH, 1 NO-PRIM synthesized); confirms STRATEGY: PER-FRAME-RAW-OVERRIDE is the only workable approach."
    missing:
      - "Per-frame ROS2PublishRawTransformTree nodes backed by pose-source nodes (IsaacReadOdometry or OgnGetPrimWorldPose) for each of the 17 override entries."
      - "The synthesized static world->aic_world edge (static transform identity, no prim)."
  - truth: "Loading the full M1 scene in Isaac Sim's viewport shows zero pink/black/missing-texture materials and zero broken-MDL warnings in the console; exts/aic-dt/docs/texture-sweep.md contains a findings + fix log enumerating every asset touched"
    status: failed
    reason: "texture-sweep.md is initialized with no sweep-run rows (scaffold only -- 'This file is initialized with no rows. The first sweep run will append a Sweep run section.'). No verified-clean sweep has been executed with the current codebase. The most recent Kit log available (kit_20260501_221152.log, 2026-05-02T05:12 UTC) was from a pre-Plan-04 session and showed 8 unique UsdToMdl texture-not-found errors (Image_0.jpg, Image_1.jpg, Image_2.jpg, NIC_Albedo.jpg, Image_0.png, Image_1.png) for NIC Card, NIC Card Mount, and SC Port materials. While texture files exist on disk under assets/assets/, the Fabric material pool relative-path resolver can't find them at runtime -- the UsdToMdl errors are KIt-level [Error] entries, not warnings. SC #4 cannot be verified without running verify_phase_1.sh with current code."
    artifacts:
      - path: "exts/aic-dt/docs/texture-sweep.md"
        issue: "Contains methodology scaffold only. Zero sweep runs recorded. The REQUIREMENTS.md marks TEX-01/TEX-02 as [~] (partial) pending viewport verification."
    missing:
      - "Run verify_phase_1.sh (or sweep_textures.py --skip-load against a running Kit session) and record actual sweep results in texture-sweep.md."
      - "Identify and fix the UsdToMdl relative-path resolution failures for NIC Card, NIC Card Mount, SC Port materials (or confirm the errors don't produce visible pink/black in the viewport)."
      - "If broken MDL warnings persist, apply the D-06 in-place USD edit pattern to rebind texture references to file:// absolute paths."
  - truth: "Calling the per-component spawn atoms with a sample_config.yaml task_board block (mount rails + sc/sfp/lc/nic occupancy + per-entity translation/RPY + robot/board pose) produces a viewport scene equivalent to the same parameters spawned by spawn_task_board.launch.py in Gazebo (SCENE-01, SCENE-04)"
    status: partial
    reason: "API surface is fully implemented: 7 atoms (spawn_task_board_base, spawn_{lc,sfp,sc}_mount_rail, spawn_sc_port, spawn_nic_card_mount, spawn_nic_card) with parameter names matching spawn_task_board.launch.py 1:1. SCENE-04 pose parameters wired. However, viewport equivalence verification has NOT been executed -- there are no Kit logs from after Plans 08/09 landed. Mount-rail thin USD wrappers reference .glb meshes via relative AddReference; outside-Kit pxr emits 'Cannot determine file format' warnings (glTF SDF plugin is Kit-only). Runtime load verification is deferred to verify_phase_1.sh in a running Kit session."
    artifacts:
      - path: "exts/aic-dt/assets/assets/LC Mount/lc_mount_visual.usd"
        issue: "Thin USD wrapper referencing .glb mesh. Loads fine in Isaac Sim runtime per Plan 09 summary, but has not been verified in a post-Plan-09 Kit session."
      - path: "exts/aic-dt/assets/assets/SFP Mount/sfp_mount_visual.usd"
        issue: "Same as above."
      - path: "exts/aic-dt/assets/assets/SC Mount/sc_mount_visual.usd"
        issue: "Same as above."
    missing:
      - "Run verify_phase_1.sh step with per-component spawn atoms against a live Kit session."
      - "Visual confirmation that task board renders correctly with sample_config.yaml trial_1 parameters."
deferred:
  - truth: "gripper/left_finger_joint name published in /joint_states (SC #2 partial aspect)"
    addressed_in: "Follow-up plan within Phase 1 or early Phase 2"
    evidence: "REQUIREMENTS.md PARITY-03 [~]: 'gripper_left_finger_joint -> gripper/left_finger_joint rename DEFERRED (no nameOverrides on OGN), follow-up plan'. CHANGELOG Known-Phase-3-work-items documents it. Plan 06 PLAN.md L302 explicitly marks as 'follow-up plan' scope."
  - truth: "TF frame names with slashes (gripper/hande_base_link, cam_mount/cam_mount_link, etc.) in /tf and /tf_static (SC #3 partial aspect)"
    addressed_in: "Follow-up plan within Phase 1 or early Phase 2"
    evidence: "REQUIREMENTS.md PARITY-04 [~]: 'per-frame Raw publisher overrides for 17 underscore->slash frame names -- Plan 06 deferral'. CHANGELOG Known-Phase-3-work-items documents it. usd_prim_inventory.txt provides the complete 17-entry override list."
human_verification:
  - test: "Run verify_phase_1.sh end-to-end against a live Kit session"
    expected: "Steps 5 (/joint_states /tf /tf_static /fts_broadcaster/wrench topics) PASS; step 6 (TF tree diff) FAILS on 16 underscore->slash mismatches + aic_world edge; step 2 (quick_start) PASS; step 8 (texture sweep) reveals which MDL errors remain active in current code"
    why_human: "Isaac Sim is not running in this verification session. verify_phase_1.sh is the designated end-to-end gate (D-15) and requires an active Kit process."
  - test: "Visual inspection of viewport after quick_start"
    expected: "UR5e + Robotiq Hand-E + task board + enclosure all render with correct materials (no pink/black objects). NIC Card, SC Port, NIC Card Mount should show their PBR textures."
    why_human: "Texture rendering quality requires visual inspection. Cannot be verified from log grep alone -- UsdToMdl errors may or may not produce visible artifacts depending on Kit's fallback material behavior."
  - test: "ros2 topic echo /joint_states --once"
    expected: "7 joint names are published. The critical question: does gripper_left_finger_joint (underscore) or gripper/left_finger_joint (slash) appear? The deferred OGN gap predicts underscore form."
    why_human: "Requires a running Isaac Sim instance and a ROS 2 environment. Cannot verify statically."
  - test: "ros2 run tf2_tools view_frames (captured to frames.gv) then python3 exts/aic-dt/scripts/diff_tf_tree.py aic_frames_live.gv /tmp/sim_frames.gv"
    expected: "diff_tf_tree.py reports 16+ mismatches on gripper/*, cam_mount/*, center_camera/*, left_camera/*, right_camera/*, ati/* frames, plus missing aic_world edge. These are the deferred items. All other frames (base, base_link, shoulder_link, etc.) should match."
    why_human: "Requires running Isaac Sim and TF tools."
---

# Phase 1: Foundation Parity Verification Report

**Phase Goal:** Isaac Sim loads same UR5e + Robotiq Hand-E + 3 wrist cameras + task-board + AIC enclosure assets, no missing/broken textures, publishes same passive sensor topics (/joint_states, /tf, /tf_static, /fts_broadcaster/wrench) — same joint names, frame names, hierarchy, message types, rates. Task-board spawn accepts same component-delta parameters as spawn_task_board.launch.py. Robot/board/cable poses configurable via Gazebo's parameter names. Atomic+clubbed contract preserved. Full cross-phase parity audit table. _sim/_real removed.
**Verified:** 2026-05-02T22:00:00Z
**Status:** GAPS_FOUND
**Re-verification:** No — initial verification

---

## Goal Achievement

### Observable Truths

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | After quick_start, `ros2 topic list` shows `/joint_states` `/tf` `/tf_static` `/fts_broadcaster/wrench` and no `_sim`/`_real` suffix anywhere | VERIFIED | Topic strings hardcoded in extension.py: `fts_broadcaster/wrench` (line 1365), `tf`/`tf_static` (lines 1491/1495), `joint_states` (line 1611). grep for `\b_sim\b\|\b_real\b` returns zero hits outside `initialize_simulation_context_async`. `objects_poses_sim` and `sync_real_poses` atoms fully deleted (8 surfaces gone). audit_dx02.py exits 0. |
| 2 | `/joint_states` returns same joint names as Gazebo (7 joints alphabetical including `gripper/left_finger_joint`) | FAILED | `setup_joint_state_publish_action_graph` binds `targetPrim=/World/UR5e/aic_unified_robot/root_joint` and publishes prim leaf names. USD prim is `gripper_left_finger_joint` (underscore — slash is illegal in USD path segments). OGN spec has no `nameOverrides` input. Isaac Sim will publish `gripper_left_finger_joint` not `gripper/left_finger_joint`. aic_adapter drops unexpected joint names (line 268: `if (!joint_sort_order_.contains(original.name[i]))`). Documented in Plan 06, inline docstring, and CHANGELOG. |
| 3 | TF tree from Isaac Sim matches Gazebo's reference TF tree (same names, same parent-child links) | FAILED | `setup_tf_publish_action_graph` uses two `ROS2PublishTransformTree` nodes. USD prim inventory (usd_prim_inventory.txt) documents 16 UNDERSCORE-MISMATCH frames and 1 NO-PRIM synthesized `aic_world` edge. Published frame_ids will be underscore-form (e.g., `gripper_hande_base_link`) instead of slash-form (`gripper/hande_base_link`). `diff_tf_tree.py` will show 17 mismatches against `aic_frames_live.gv`. Fix requires per-frame `ROS2PublishRawTransformTree` nodes with additional pose-source inputs — not yet implemented. |
| 4 | Zero pink/black/missing-texture materials and zero broken-MDL warnings; `texture-sweep.md` contains a findings + fix log | FAILED | `texture-sweep.md` is a scaffold with no sweep runs executed. The most recent available Kit log (from a pre-Plan-04 session) shows 8 unique `[Error] [UsdToMdl]` entries for NIC Card, NIC Card Mount, and SC Port materials (`./textures/Image_0.jpg`, `NIC_Albedo.jpg`, etc. — files exist on disk, but Kit's Fabric material pool cannot resolve relative paths). No post-Plans-04-09 Kit log exists to confirm the current code's state. TEX-01/TEX-02 marked `[~]` pending viewport verification in REQUIREMENTS.md. |
| 5 | grep extension source for `objects_poses_sim`, `sync_real_poses`, `_sim`, `_real` returns zero production-topic matches | VERIFIED | `grep -E '(_sim\|_real)\b' extension.py \| grep -v initialize_simulation_context_async` returns zero lines. `objects_poses_sim` and `sync_real_poses` atoms fully deleted. `camera_rgb_sim` topics renamed to `center_camera/image`, `left_camera/image`, `right_camera/image`. RG2 identifier zero production matches. |
| 6 | `/fts_broadcaster/wrench` publishes steady `WrenchStamped` at `frame_id=ati/tool_link` | PARTIAL | Static wiring verified: `publisher.inputs:topicName` = `fts_broadcaster/wrench` (line 1365), `header.frame_id` = `ati/tool_link` (sourced from URDF `ati_description/urdf/ur_gz.urdf.xacro` line 242). Runtime publish rate and actual message content require live Isaac Sim verification — no post-cleanup Kit log available. Audit trail in `parity_05_wrench_framing.txt`. |
| 7 | Per-component spawn atoms with `sample_config.yaml task_board` parameters produce equivalent scene state as Gazebo | PARTIAL | API surface fully implemented: 7 atoms covering all 6 parameter families from `spawn_task_board.launch.py` (task_board, lc/sfp/sc_mount_rail, sc_port, nic_card_mount). SCENE-04 pose params wired through `load_robot`. Mount-rail USD wrappers authored and vendored. No post-Plans-08/09 Kit session confirms runtime correctness — viewport equivalence requires human verification. |
| 8 | `exts/aic-dt/docs/topic-parity-reference.md` contains complete cross-phase parity audit table | VERIFIED | `topic-parity-reference.md` exists (279 lines), covers all 36 live `aic_eval` topics from `aic_topics_live.txt`. Each row has: Live Gazebo status, Isaac Sim Status (implemented/Phase-N-deferred/not-applicable), Phase, and Proof-of-Publish command or `—` for deferred. Image digest pinned per D-14 (`sha256:be08f28709...`). `snapshot_aic_eval.sh` is the re-capture command (exists, syntax OK). |
| 9 | Every new MCP capability lands as one `MCP_TOOL_REGISTRY` entry + matching `_cmd_<name>` handler + one UI button | VERIFIED | `audit_dx02.py` exits 0: "27 present atoms × 4 surfaces, 2 absent atoms × 4 surfaces — all OK". Verified independently: 9 new atoms (TF publisher, JointState publisher, 7 spawn atoms) all have registry/handler-map/\_cmd/UI button. 2 deleted atoms (setup\_pose\_publisher, sync\_real\_poses) have all 4 surfaces removed. 7 MCP-only atoms documented as exemptions with rationale. |

**Score:** 5/9 truths verified (2 FAILED outright, 2 PARTIAL pending human verification, with 2 architectural deferrals documented inline)

---

### Deferred Items

Items not yet met but explicitly documented as deferred within Phase 1 plans and addressed by follow-up plans.

| # | Item | Addressed In | Evidence |
|---|------|-------------|----------|
| 1 | `gripper/left_finger_joint` (slash) in `/joint_states` | Follow-up plan (Phase 1 continuation or Phase 2) | REQUIREMENTS.md PARITY-03 `[~]`: "gripper_left_finger_joint -> gripper/left_finger_joint rename DEFERRED (no nameOverrides on OGN), follow-up plan". Plan 06 PLAN.md L302. CHANGELOG Known-Phase-3-work-items. |
| 2 | 16 underscore->slash TF frame name overrides + synthesized `aic_world` edge | Follow-up plan (Phase 1 continuation or Phase 2) | REQUIREMENTS.md PARITY-04 `[~]`: "per-frame Raw publisher overrides for 17 underscore->slash frame names". usd_prim_inventory.txt lines 237-265. CHANGELOG Known-Phase-3-work-items. Plan 06 PLAN.md L51/L302. |

---

### Required Artifacts

| Artifact | Expected | Status | Details |
|----------|----------|--------|---------|
| `exts/aic-dt/aic_dt/extension.py` | Phase 1 production code (3303 LOC) | VERIFIED | Syntax clean (`python3 -m py_compile` passes). Zero `_sim/_real` production hits. All 27 MCP atoms present × 4 surfaces. |
| `exts/aic-dt/docs/topic-parity-reference.md` | Complete 36-topic cross-phase audit table | VERIFIED | 279 lines, all aic_topics_live.txt rows covered with explicit dispositions. |
| `exts/aic-dt/docs/CHANGELOG.md` | Phase 1 milestone entry | VERIFIED | 207-line Phase 1 entry covering all 13 requirement IDs; Added/Changed/Removed/Known-Phase-3/DX-02-audit sections. |
| `exts/aic-dt/docs/texture-sweep.md` | Findings + fix log for all assets | STUB | Scaffold only — zero sweep runs executed. Methodology documented. Sweep tool (`sweep_textures.py`) exists and is syntax-clean. |
| `exts/aic-dt/docs/README.md` | AIC-scope description (not ur5e-dt) | FAILED | Still says "ur5e-dt Extension", references port 8766, lists `assemble_objects` tool. DX-05 is a Phase 4 requirement — correctly deferred per REQUIREMENTS.md. |
| `exts/aic-dt/scripts/verify_phase_1.sh` | Hybrid-runtime Phase 1 gate | VERIFIED | 441-line script, `bash -n` syntax clean. 10-step structure matches D-15 spec. Cold-launch + attached modes. Cache discipline integrated. |
| `exts/aic-dt/scripts/diff_tf_tree.py` | TF tree .gv diff utility | VERIFIED | 71 lines, syntax clean. Smoke-tested: live-vs-live diff returns "PASS: TF trees match (31 frames, 30 edges)". |
| `exts/aic-dt/scripts/sweep_textures.py` | Kit log grep + sweep runner | VERIFIED | Exists, syntax clean. Documented augmented patterns (Isaac-Sim-specific failures). |
| `exts/aic-dt/scripts/audit_dx02.py` | DX-02 4-surface contract verifier | VERIFIED | Exits 0: "27 present atoms × 4 surfaces, 2 absent atoms × 4 surfaces — all OK". |
| `exts/aic-dt/scripts/snapshot_aic_eval.sh` | Live aic_eval Docker capture | VERIFIED | Exists, syntax clean. SHA-256 digest pinned. Snapshot artifacts present (36 topics, 31 frames). |
| `exts/aic-dt/scripts/probe_unified_usd.py` | USD prim inventory probe | VERIFIED | Exists, syntax clean. Output in `usd_prim_inventory.txt`. |
| `exts/aic-dt/scripts/probe_root_joint.py` | PhysicsFixedJoint verifier | VERIFIED | Exists, syntax clean. Used by Plan 06 to confirm `/World/UR5e/aic_unified_robot/root_joint` target. |
| `exts/aic-dt/scripts/build_mount_rail_usds.py` | Mount-rail USD wrapper author | VERIFIED | Exists, syntax clean. 3 USD wrappers shipped (LC/SFP/SC Mount). |
| `.planning/phases/01-foundation-parity/snapshot/` | Live aic_eval reference artifacts | VERIFIED | Contains: `image_digest.txt`, `topic_list.txt` (36 topics), `joint_states_sample.yaml` (7 joints), `aic_frames_live.gv` (31 frames), per-topic info files. |
| `exts/aic-dt/assets/assets/` (capitalized layout) | Vendored AIC assets with textures/ siblings | VERIFIED (structure) / PARTIAL (runtime load) | All 8 asset folders present (NIC Card, NIC Card Mount, SC Port, SC Plug, Task Board Base, LC Mount, SFP Mount, SC Mount). Texture files on disk. Runtime material resolution unconfirmed (post-Plan-04 Kit log unavailable). |

### Key Link Verification

| From | To | Via | Status | Details |
|------|----|-----|--------|---------|
| `setup_tf_publish_action_graph` | `/tf` ROS topic | `ROS2PublishTransformTree (staticPublisher=False)` | WIRED | OmniGraph built, `targetPrims=/World/UR5e/aic_unified_robot`, relationship-after-edit pattern. Frame_id mismatch is the deferred gap. |
| `setup_tf_publish_action_graph` | `/tf_static` ROS topic | `ROS2PublishTransformTree (staticPublisher=True)` | WIRED | TRANSIENT_LOCAL QoS via `staticPublisher=True`. Same caveats on frame_id. |
| `setup_joint_state_publish_action_graph` | `/joint_states` ROS topic | `ROS2PublishJointState, targetPrim=/World/UR5e/aic_unified_robot/root_joint` | WIRED (partial) | Topic published. `gripper/left_finger_joint` name override absent. |
| `setup_force_publish_action_graph` | `/fts_broadcaster/wrench` | `ROS2Publisher, frame_id=ati/tool_link` | WIRED | Topic name and frame_id correct per PARITY-05 audit. Runtime rate unconfirmed. |
| `quick_start` | All 4 passive-sensor topics | `setup_tf_publish_action_graph + setup_joint_state_publish_action_graph + setup_force_publish_action_graph + setup_wrist_cameras` | WIRED | quick_start steps 3a/3b/6 call all four graph builders post-play. hasattr guard on reorder bridge is safe (method absent per Plan 05 verdict). |
| `_cmd_spawn_*` atoms | `/World/TaskBoard/<prim>` | `_spawn_component_via_usd helper (AddReference + pose ops)` | WIRED | All 7 spawn atoms delegate to shared helper. Idempotent cleanup (RemovePrim). Asset FileNotFoundError degrades to status:error, not exception. |
| `AIC_OBJECTS` paths | Vendored USD files | `_local_asset resolver (file:// URI)` | VERIFIED | `assets/Task Board Base/base_visual.usd`, `assets/SC Port/sc_port_visual.usd`, `assets/NIC Card Mount/nic_card_visual.usd` all resolve to existing files. |
| `load_robot` SCENE-04 kwargs | Cable prim transform | `ClearXformOpOrder + AddTranslateOp + AddRotateXYZOp` | WIRED (surface only) | Parameter surface matches Gazebo; cable subtree remains `SetActive(False)` per D-04. Pose values applied but no-op in Phase 1. |

---

### Data-Flow Trace (Level 4)

Not applicable for this phase — no components render dynamic data from an external database. All publishers are Isaac Sim OmniGraph nodes reading physics/articulation state directly. The key data flows are:

- Physics articulation state → `ROS2PublishJointState` → `/joint_states` (articulation state is real; joint name override is the gap)
- Articulation transform tree → `ROS2PublishTransformTree` → `/tf` + `/tf_static` (transforms are real; frame_id string names are the gap)
- Force/torque sensor → `ROS2Publisher` → `/fts_broadcaster/wrench` (physics output; runtime unconfirmed)

---

### Behavioral Spot-Checks

| Behavior | Command | Result | Status |
|----------|---------|--------|--------|
| audit_dx02.py exits 0 (DX-02 contract) | `python3 exts/aic-dt/scripts/audit_dx02.py` | "27 present atoms × 4 surfaces, 2 absent atoms × 4 surfaces — all OK" | PASS |
| extension.py syntax clean | `python3 -m py_compile exts/aic-dt/aic_dt/extension.py` | Exit 0 | PASS |
| All 9 scripts syntax-valid | `bash -n` / `python3 -m py_compile` on all 9 scripts | 9/9 SYNTAX OK | PASS |
| diff_tf_tree.py live-vs-live | `python3 diff_tf_tree.py aic_frames_live.gv aic_frames_live.gv` | "PASS: TF trees match (31 frames, 30 edges)" | PASS |
| Zero `_sim`/`_real` in production surface | `grep -E '\b(_sim\|_real)\b' extension.py \| grep -v initialize_simulation_context_async` | No output | PASS |
| Zero standalone `RG2` identifiers | `grep -rn '\bRG2\b' exts/aic-dt/aic_dt/extension.py` | No output | PASS |
| AIC_OBJECTS USD paths resolve | File existence check for all 4 paths | All 4 present | PASS |
| Mount-rail asset folders vendored | `ls assets/assets/{LC Mount,SFP Mount,SC Mount}/` | All 3 present with .usd, .glb, model files | PASS |
| `/joint_states` topic published (runtime) | `ros2 topic hz /joint_states` after quick_start | Isaac Sim not running | SKIP |
| TF tree diff against Gazebo (runtime) | `python3 diff_tf_tree.py aic_frames_live.gv /tmp/sim_frames.gv` | Isaac Sim not running | SKIP |
| Texture errors in viewport (runtime) | `python3 sweep_textures.py` after quick_start | Isaac Sim not running | SKIP |

---

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
|-------------|-------------|-------------|--------|---------|
| PARITY-01 | 01-03, 01-04 | UR5e + Robotiq Hand-E USD + joint-path bug fix | [~] | Gripper correctly named; joint prim-path bug fixed (`/World/UR5e/aic_unified_robot/joints/<name>`). URDF xacro load path not re-implemented (AIC's pre-built USD used per D-02). |
| PARITY-02 | 01-02, 01-04, 01-09 | AIC assets vendored, paths updated | [~] | Capitalized asset layout correct; AIC_OBJECTS paths verified. Viewport load and texture binding unconfirmed (no post-Plans-04 Kit log). |
| PARITY-03 | 01-05, 01-06 | `/joint_states` joint names + ordering | [~] | Publisher exists; aic_adapter ordering strategy confirmed (NAME-INDEXED). Deferred: `gripper/left_finger_joint` underscore→slash rename. |
| PARITY-04 | 01-05, 01-06 | TF tree frame names and hierarchy | [~] | Publisher exists; 14 MATCH frames will be correct. Deferred: 16 underscore→slash + 1 synthesized `aic_world` frame. |
| PARITY-05 | 01-04 | `/fts_broadcaster/wrench` topic + frame_id | [~] | Static wiring correct (`fts_broadcaster/wrench`, `ati/tool_link`). Runtime rate unconfirmed. |
| PARITY-12 | 01-01 | Cross-phase parity audit table | [x] | `topic-parity-reference.md` covers all 36 live topics with dispositions, phase assignments, and proof-of-publish commands. |
| TEX-01 | 01-02, 01-04 | Zero pink/black materials, zero broken-MDL warnings | [~] | Disk-layer fixed (sibling textures/ folders). Runtime verification pending — pre-Plan-04 Kit log showed active UsdToMdl errors. |
| TEX-02 | 01-02 | GLB-imported PBR maps rebound | [~] | Capitalized asset layout preserves relative texture references. Runtime binding confirmation pending. |
| TEX-03 | 01-07 | Texture sweep findings + fix log | [~] | `texture-sweep.md` scaffold shipped; `sweep_textures.py` ready. Actual sweep rows require running Kit session. |
| SCENE-01 | 01-09 | Per-component spawn atoms mirroring spawn_task_board.launch.py | [x] | 7 atoms with 4-surface DX-02 contract; parameter names match launch.py 1:1 for all 6 families. Runtime equivalence check pending. |
| SCENE-04 | 01-09 | Robot/board/cable pose parameters | [x] | 12 Gazebo-named pose kwargs wired through `load_robot`. None-sentinel preserves backwards-compat. Cable SetActive(False) per D-04. |
| DX-01 | 01-04 | Zero `_sim`/`_real` on production surface | [x] | Confirmed by grep: zero hits excluding `initialize_simulation_context_async`. 33-row rename table applied. |
| DX-02 | 01-04, 01-06, 01-09, 01-08 | MCP_TOOL_REGISTRY + _cmd + UI button contract | [x] | audit_dx02.py exits 0: 27 present × 4 surfaces, 2 absent × 4 surfaces. |

---

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
|------|------|---------|----------|--------|
| `exts/aic-dt/aic_dt/extension.py` | 3275-3280 | `run_policy` returns "not yet implemented" placeholder | Info | Correctly scoped as Phase 4 work. Not a Phase 1 capability gap. |
| `exts/aic-dt/aic_dt/extension.py` | 1540+ | `setup_joint_state_publish_action_graph` docstring: "expected to FAIL pre-follow-up (surfaces deferral)" | Warning | The joint name override deferral directly blocks SC #2. Stub language in docstring confirms the known gap. |
| `exts/aic-dt/aic_dt/extension.py` | 1418+ | `setup_tf_publish_action_graph` docstring: "Open question... DEFERRED PER PLAN BODY" for 16 frame_id overrides | Warning | Frame_id overrides deferred blocks SC #3. Stub language confirms the known gap. |
| `exts/aic-dt/docs/README.md` | 1-41 | Stale "ur5e-dt Extension" title, port 8766, `assemble_objects` tool listing | Warning | DX-05 is Phase 4 work — correctly deferred. Not actionable as a Phase 1 gap. |
| `exts/aic-dt/docs/texture-sweep.md` | 40 | "This file is initialized with no rows" — no sweep runs recorded | Blocker | SC #4 requires populated findings log. Sweep tooling exists but no sweep has been executed against current code. |

---

### Human Verification Required

#### 1. Full verify_phase_1.sh End-to-End Gate

**Test:** With DerivedDataCache at 153MB (healthy), launch Isaac Sim via lifecycle helper: `DISPLAY=:0 bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt`, then run: `bash exts/aic-dt/scripts/verify_phase_1.sh`

**Expected (steps that should PASS):**
- Step 0: Cache health check (153MB, healthy)
- Step 2: quick_start returns "Quick start complete: AIC scene with UR5e, wrist cameras, task board objects, and simulation running."
- Step 5: `ros2 topic list` shows `/joint_states`, `/tf`, `/tf_static`, `/fts_broadcaster/wrench`
- Step 5a: `ros2 topic echo /fts_broadcaster/wrench --once` returns `header.frame_id: ati/tool_link`
- Step 7: `_sim`/`_real` grep returns zero hits

**Expected (steps that will FAIL — deferred gaps):**
- Step 6: `diff_tf_tree.py` reports 16+ frame_id mismatches + missing `aic_world` edge
- Step 5 (joint names): `gripper_left_finger_joint` appears in `/joint_states` instead of `gripper/left_finger_joint`

**Expected (steps that need result inspection):**
- Step 8: `sweep_textures.py` — how many UsdToMdl errors remain with current asset paths? SC #4 depends on this.

**Why human:** Isaac Sim is not running in this verification session. verify_phase_1.sh is the designated hybrid-runtime gate (D-15).

#### 2. Viewport Visual Inspection (SC #4)

**Test:** After `quick_start`, visually inspect the Isaac Sim viewport. Check NIC Card, SC Port, NIC Card Mount, Task Board Base, and the robot USD for pink/black/placeholder materials.

**Expected:** Zero pink/black materials. All assets render with PBR textures (NIC Card silvery, SC Port grey/beige, task board base colored per model).

**Why human:** Texture rendering quality can only be assessed visually. UsdToMdl log errors may produce visible pink/black objects OR may fall back to a neutral grey (both are failures per SC #4 since the log still shows `[Error]`). Visual confirmation is required alongside the sweep log.

#### 3. `/joint_states` Joint Name Confirmation (SC #2 — deferred gap severity)

**Test:** `ros2 topic echo /joint_states --once | grep -A 10 "name:"` after quick_start.

**Expected:** 6 UR5e joints publish correctly (shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint). The `gripper/left_finger_joint` entry is expected to appear as `gripper_left_finger_joint` (the deferred deferral) — confirm which form appears so the gap closure plan can be scoped precisely.

**Why human:** Requires running Isaac Sim with ROS 2 bridge active.

---

### Gaps Summary

**Two architectural deferrals** block two SCs and are the highest-priority gaps:

**Gap A (SC #2 — Joint name override, PARITY-03):** The `ROS2PublishJointState` OGN node has no `nameOverrides` or `jointNames` input. Isaac Sim publishes USD prim leaf names, so `gripper_left_finger_joint` (underscore) is published instead of `gripper/left_finger_joint` (slash). The `aic_adapter::ReorderJointState` function at line 268 calls `joint_sort_order_.contains(original.name[i])` — the slash-form is the only key in the map (line 86). The underscore-form is silently dropped from every `Observation` message. This means the controller never sees the gripper finger position, which will cause malfunction in Phase 2 when the controller loop closes. Fix: a wrapper republisher node that intercepts `/joint_states`, renames `gripper_left_finger_joint` → `gripper/left_finger_joint`, and republishes on `/joint_states`. Or a USD-side prim rename (not feasible with slashes in USD path segments without using custom attributes and a modified publisher).

**Gap B (SC #3 — TF frame_id overrides, PARITY-04):** The `ROS2PublishTransformTree` node uses USD prim leaf names as `frame_id`. All 16 frames with slashes in Gazebo's TF tree (e.g., `gripper/hande_base_link`, `cam_mount/cam_mount_link`, `ati/tool_link`, `center_camera/camera_link`, etc.) will publish with underscores. CheatCode and aic_adapter perform `lookup_transform` against these frame names — wrong names will cause TF lookup failures. The `aic_world` frame (child of `world`, no USD prim, published as a static identity edge in Gazebo) is entirely absent. Fix: per-frame `ROS2PublishRawTransformTree` nodes backed by `IsaacReadOdometry` or `OgnGetPrimWorldPose` source nodes, one node per override entry. 17 nodes total. The complete list is in `usd_prim_inventory.txt` lines 239-256.

**One unverified SC** needs runtime confirmation:

**Gap C (SC #4 — Texture sweep, TEX-01/02/03):** `texture-sweep.md` contains only the scaffold — no sweep rows. The most recent Kit log available (from a pre-Plan-04 session) showed active `[Error] [UsdToMdl]` entries for NIC Card, NIC Card Mount, and SC Port materials. While texture files exist on disk, the Fabric material pool relative-path resolver may not be resolving them correctly at runtime. The sweep must be run with current code to close TEX-01/TEX-02.

**One SC is API-surface-complete but visually unconfirmed:**

**Gap D (SC #7 — Parametric spawn scene equivalence):** All 7 spawn atoms are implemented with correct parameter names. Mount-rail USD wrappers reference `.glb` via `AddReference`. The implementation is substantive — the concern is whether the rendered scene matches Gazebo's output for a trial_1-equivalent parameterization. This requires a live Kit session to confirm viewport equivalence.

Gaps A and B are documented as explicit deferrals in Plan 06, REQUIREMENTS.md PARITY-03/04 `[~]`, CHANGELOG Known-Phase-3-work-items, and `usd_prim_inventory.txt`. A follow-up plan closing both deferrals is the next actionable step before Phase 1 can be declared VERIFIED.

---

_Verified: 2026-05-02T22:00:00Z_
_Verifier: Claude (gsd-verifier)_
