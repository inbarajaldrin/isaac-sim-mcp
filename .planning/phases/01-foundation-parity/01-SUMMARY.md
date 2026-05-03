---
phase: 01-foundation-parity
status: closed (9/10 must-haves verified, 3 small items deferred)
started: 2026-05-01
closed: 2026-05-03
total_plans: 9 + 1 inline visual fix + 1 inline rclpy parity fix
---

# Phase 1 — Foundation Parity: Summary

## What this phase delivered

The Isaac Sim aic-dt extension now publishes the **exact ROS topic surface AIC's tooling expects** when running with `ground_truth:=true`-equivalent mode. CheatCode-style policies, `aic_adapter`, and any tf2-based consumer can talk to Isaac Sim without code changes — verified by a smoke test that uses `/opt/ros/humble`'s Python 3.10 rclpy + tf2_ros (the exact stack AIC's policies run on) and confirms 13/13 checks pass against the live publishers.

| Surface | Topic | Result |
|---|---|---|
| Joint state | `/joint_states` (`sensor_msgs/JointState`) | 7 alphabetical names matching `aic_adapter::joint_sort_order_` exactly, `frame_id=base_link`, gripper FixedJoint pinned to 0.0 |
| Dynamic TF | `/tf` (`tf2_msgs/TFMessage`) | 8 transforms at ~70 Hz: 6 UR5e arm joints + 2 Hand-E finger joints, all in slashed AIC frame names |
| Static TF | `/tf_static` (TRANSIENT_LOCAL) | 22 transforms covering the world→tabletop→base_link static chain, ATI sensor, gripper TCP, 3 wrist cameras, all slashed |
| F/T sensor | `/fts_broadcaster/wrench` (`geometry_msgs/WrenchStamped`) | frame_id corrected to `ati/tool_link` per URDF (Plan 04) |
| Asset library | `exts/aic-dt/assets/assets/` | AIC's `Intrinsic_assets` capitalized layout vendored byte-identical (21 files: 9 USDs + 12 textures + 4 mount-asset folders) |
| Scene parameterization | `add_objects` + 7 per-component spawn atoms | `spawn_task_board.launch.py` parameter names mirrored 1:1 (mount rails, port presence + translations + RPY, NIC card mount index 0..4, robot/board/cable poses) |

## Plan-by-plan trail

| Plan | Subject | Atomic commit range |
|---|---|---|
| 01-01 | Snapshot infrastructure + topic-parity-reference + PARITY-12 audit table | 5dc5b75..2a1c636 |
| 01-02 | Asset vendoring (capitalized AIC layout + sibling textures) | 7858369..e56ab9d |
| 01-03 | RG2 → Robotiq Hand-E doc fix (.planning/, CLAUDE.md, exts/aic-dt/docs/) | be59b05..86822fc |
| 01-04 | extension.py renames + prim-path bug + atom deletes + AIC_OBJECTS update + PARITY-05 wrench framing | de0a8d7..cde6a1a |
| 01-05 | USD prim probe + aic_controller subscriber probe (verdicts: PER-FRAME-RAW-OVERRIDE, NAME-INDEXED) | 8a89890..56500f5 |
| 01-06 | TF + JointState OGN publisher MCP atoms (4-surface contract) | f3dd646..721ae0a |
| 01-07 | diff_tf_tree.py + sweep_textures.py + verify_phase_1.sh + texture-sweep.md scaffold | fea9efd..beafec1 |
| 01-08 | quick_start refactor + Phase 1 CHANGELOG + DX-02 final audit script | a93b0c8..def50be |
| 01-09 | 7 per-component spawn atoms (SCENE-01) + SCENE-04 pose params + 4 mount asset folders vendored | 424d44b..22e1395 |
| (inline) | Visual regression: AIC_OBJECTS reverted to m+Z-up originals (Plan 04 had silently swapped to *_visual.usd cm+Y-up variants) | 121244a |
| (inline) | rclpy parity publishers replace OGN: closes Gap A (PARITY-03) + Gap B (PARITY-04) | 079afd8..a70ef52 |

## What changed in plain terms

### USD-side (none of this was changed in the rclpy fix; all earlier plans)

- AIC's `Intrinsic_assets/<Capitalized>/` tree (NIC Card, NIC Card Mount, SC Plug, SC Port, Task Board Base, LC/SFP/SC Mount) is vendored under `exts/aic-dt/assets/assets/` byte-identical to upstream. `cp -r` whole-folder vendoring is the contract; selective USD-only copies reintroduce missing-texture errors.
- The `AIC_OBJECTS` dict in `extension.py` points at the m+Z-up originals (`task_board_rigid.usd`, `sc_port.usd`, `nic_card.usd`), NOT the `*_visual.usd` variants which are mPU=0.01+Y-up and would import 100x smaller and rotated 90° on `AddReference`.
- Plan 09's `build_mount_rail_usds.py` authored 3 thin USD wrappers (LC/SFP/SC Mount/`*_visual.usd`) that reference local `.glb` meshes. These have a known mPU/upAxis bug — latent because the mount-rail spawn atoms default to `present=False`. Required for SCENE-01 trial-spawn correctness when `sample_config.yaml` enables them.

### ROS-side (this is where this phase's deliverable lives)

- New rclpy-based `AicParityPublishers` class in `exts/aic-dt/aic_dt/parity_publishers.py` reads articulation state and USD world poses each PhysX tick and publishes `/joint_states`, `/tf`, `/tf_static` directly via rclpy. Bypasses OGN nodes entirely (they don't expose the inputs needed for slashed frame names, frame_id=base_link, joint name overrides).
- The two MCP atoms `setup_tf_publish_action_graph` and `setup_joint_state_publish_action_graph` retain their names + UI buttons + handler-map entries (DX-02 4-surface contract preserved); their bodies now delegate to the shared publisher manager.
- `quick_start` refactored (Plan 08) to call them on the common path; `extension.on_shutdown` cleans up the publisher manager.
- `/fts_broadcaster/wrench` framing corrected to publish with `frame_id=ati/tool_link` (Plan 04 from URDF spec).

## Architectural decisions worth carrying forward

1. **Surface-adjacency rule (added to CLAUDE.md during Phase 1)**: pull future-phase requirements into the current phase if they share an edit surface, dependencies are met, and no new research is needed. This is why SCENE-01, SCENE-04, PARITY-05, PARITY-12, DX-02 landed in Phase 1 instead of Phase 2/3.
2. **Canonical references for ALL agents (added to CLAUDE.md during Phase 1)**: `nvidia-suite-docs` and `isaac-sim-extension-dev` skills are consulted during research AND execution AND debugging — not just research phases. Live docs > cached LLM knowledge for the NVIDIA stack.
3. **Vendoring is one-time, then this repo owns the assets** (D-05/D-06): no ongoing AIC sync; edit vendored USDs in place for fixes.
4. **rclpy is in-policy for non-control glue** (CLAUDE.md): when OGN's input surface can't deliver the AIC topic contract, Python rclpy publishers are the right hammer. Documented in this phase's closure as the canonical answer.
5. **venv-activate before Isaac Sim launch is mandatory for parity publishers**: workspace's `libgeometry_msgs__rosidl_generator_c.so` (with `polygon_instance_stamped` symbol) must win LD ordering over `/opt/ros/humble`'s older lib. CLAUDE.md updated with the wrap-it-in-bash-c command form.

## Verification

- `01-VERIFICATION.md` flipped to `gaps_resolved`, score 9/10. Closure summary documents the 3-piece architectural fix for Gap A/B.
- `exts/aic-dt/scripts/smoke_test_aic_parity.py` — reusable Python 3.10 verifier that runs against any future Isaac Sim+aic-dt session and confirms the topic surface is consumable. **All 13 checks pass on the closing session.**
- Live evidence:
  - `ros2 topic echo /joint_states --once` → 7 alphabetical names, `frame_id=base_link`
  - `ros2 run tf2_tools view_frames` → 30 edges, slashed AIC frame names
  - `tf2_ros::Buffer.lookup_transform('base_link', 'gripper/tcp')` → succeeds, d=0.937m, geometrically plausible

## Deferred items (do not lose)

| Item | Severity | Path forward |
|---|---|---|
| Gap C (TEX-03 sweep findings categorization) | low | `exts/aic-dt/docs/texture-sweep.md` has the 921 raw hits; needs categorization into 4 buckets (cosmetic screw-subref / camera-prim-bug / actual MDL/texture / unresolved-but-benign). Pure doc work. |
| Gap E mount-rails | latent | `build_mount_rail_usds.py` authors LC/SFP/SC Mount + `nic_card_mount_visual.usd` at mPU=0.01+Y-up. Default `present=False` so they don't fire today. SCENE-01 trial spawning with explicit `present=True` will spawn them at 100x-smaller / wrong orientation. Fix: bake xformOp:scale=(100,100,100) + rotateX=90 in the wrapper authoring step. |
| PARITY-05 rate verifiability | doc | `parity_05_wrench_framing.txt` doesn't capture Gazebo's wrench-rate snapshot. Re-run `bash exts/aic-dt/scripts/snapshot_aic_eval.sh` against a freshly-launched aic_eval container; record `ros2 topic hz` for `/force_torque_sensor_broadcaster/wrench`. |

## What this phase did NOT deliver (and where it lands)

- **Controller-loop closure** — `aic_controller` cannot drive Isaac Sim yet; this is **Phase 2** (PARITY-06/09/10/11). Phase 1 is passive-publisher only.
- **Object TF frames CheatCode reads** (`{cable}/{plug}_link`, port frames in `base_link`) — **Phase 3** (SCENE-06). CheatCode's other 3 lookup_transform calls (lines 87, 207) target object frames not in scope here.
- **Cable physics** — `cable` subtree is `SetActive(False)` per D-04; full physics is **Phase 3** (SCENE-05) pending `nvidia-suite-docs` evaluation.
- **End-to-end M1 success bar** — every `sample_config.yaml` trial passing under CheatCode is **Phase 4** (or whatever final integration phase ends up being).

## Next step

`/gsd-discuss-phase 2` — Controller-Loop Closure. Apply the surface-adjacency rule when scoping: any Phase 3 items whose edit surface overlaps with Phase 2's controller surface (e.g. force_mode_controller, freedrive_mode_controller, scaled trajectory controller variants) should be pulled forward.
