# Next Session — Pickup Point (2026-05-17 wrap)

> Replaces the earlier 2026-05-16 NEXT-SESSION.md. Read this first when resuming.

## Where we ended this session

Cable-fidelity work has moved through four commits this session:

| SHA | What landed |
|---|---|
| `e131c36` | Layer 1: end-anchor pattern (no-op — superseded; translates of connector visuals are dominated by joint constraints) |
| `e90d27a` | Layer 2: `physics:kinematicEnabled=True` on both connector visuals in build_cable_variant_usds.py |
| `52d4eb9` | Swap joint endpoints in reversed USD (fixedJoint / fixedJoint2 body1 swap so the right connector binds to each rope-end) |
| `69b8b5c` | Remove orphan FixedJoint at `gripper_hande_finger_link_r/FixedJoint` from both USDs + add per-tick physics-step TCP tracker in `_attach_cable_to_gripper_impl` |
| `6383881` | Add orientation tracking via Fabric (usdrt) writes — Fabric-only |
| `bab929f` | **Dual USD+Fabric write** — Hydra needs USD-side value too, Fabric-only was silently ignored by the renderer |
| `e13ec03` | **Quat compose order fix (GPT-diagnosed)** — `rel * tcp` not `tcp * rel` per Gf row-vector convention. Isolated math now gives 0.0° diff; runtime still 90° off due to PhysX Rope/fixedJoint override (the only remaining cause) |

## Confirmed status (numerical, post `bab929f`)

| Frame | Gazebo trial_3 (truth) | Isaac Sim trial_3 (current) | Match? |
|---|---|---|---|
| sc_plug_visual world X | 0.1721 | 0.1747 | ✅ ~3mm off |
| sc_plug_visual world Y | -0.0021 | -0.0005 | ✅ ~2mm off |
| sc_plug_visual world Z | 1.4545 | 1.4637 | ✅ ~9mm off |
| Plug X between finger_l/r | True | **True** | ✅ |
| sc_plug rpy (deg) | (91.7, 63.4, -89.3) | (-179.7, 2.1, 0.5) | **❌ orientation still wrong** |

Position: now matches Gazebo within ~10mm. Plug IS between the gripper fingers.

**Orientation: still wrong.** The held SC plug visual ends up with TCP's orientation (≈180° about Y) instead of Gazebo's compound rotation (92°, 63°, -89°). My orient tracker writes the computed local_quat to BOTH USD and Fabric every tick, and probe confirms both stores have the right local value. But when world transform is composed via XformCache/Hydra, the result equals TCP's orientation as if the local orient was identity.

## Three open gaps (priority order)

### 1. ❌ Pose orientation — **the most important remaining problem (NEARLY SOLVED)**

**Symptom**: sc_plug_visual local orient = my computed value, but world orient ≈ TCP's orient (rel_quat is silently dropped in composition).

**Root-cause diagnosis (via /ask-gpt 2026-05-17, result at `/tmp/gpt-pose-result-9174.txt`):**

The PRIMARY bug was matrix composition order. OpenUSD `Gf` uses ROW-VECTOR convention where the LEFT matrix is MORE LOCAL. The prior code `tcp_rot_mat * rel_rot_mat` was column-vector style. Corrected in commit `e13ec03` to `rel_rot_mat * tcp_rot_mat`. Verified via isolated math: gives 0.0° diff to Gazebo target.

But the SECONDARY bug remains: even with correct compose, the runtime world orient still ≈ TCP's orient. The `/World/UR5e/cable/Rope/fixedJoint` (body0=link_0, body1=sc_plug_visual, non-identity `physics:localRot0` ≈ (0.499,-0.5,0.5,-0.499)) overrides the kinematic body's xform Set every PhysX step. Rope/fixedJoint2 has the same pattern with link_20 and the other connector.

**Concrete next actions:**

Try in order until one works:

1. **Use PhysX tensors API** (GPT's recommended pathway):
   ```python
   import omni.physics.tensors as tensors
   import warp as wp, numpy as np
   sim_view = tensors.create_simulation_view("warp")
   rb_view = sim_view.create_rigid_body_view("/World/UR5e/cable/sc_plug_visual")
   target = np.array([[x, y, z, qx, qy, qz, qw]], dtype=np.float32)
   idx = wp.array([0], dtype=wp.int32, device="cpu")
   rb_view.set_kinematic_targets(target, idx)
   ```
   Docs: https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/simulation_control/simulation_control.html
   This is the supported pathway for kinematic body pose control — should bypass the joint-constraint fight.

2. **Remove Rope/fixedJoint AND fixedJoint2** from both cable USDs in `build_cable_variant_usds.py`. The rope will dangle without anchors to the connectors, but for M1 the rope is decorative — only plug position+orient matters for scoring. This is the cheapest experiment.

3. **Reparent `sc_plug_visual` (and `sfp_module_visual`) under `gripper_tcp`** via USD hierarchy at trial load time. The renderer uses USD composition with no per-tick fight. More invasive (changes prim paths, breaks joint references downstream) but architecturally cleanest.

4. **Drop the bare `except: pass`** in `_install_held_connector_tcp_tracker._on_step` while diagnosing — GPT noted this can hide silent Set failures.

The matrix-order fix (`e13ec03`) is committed and correct in isolation. The remaining work is gating the PhysX writeback.

### 2. ✅ Materials missing — CLOSED 2026-05-17 (commit `b059074`)

The path predicted in this section's "Concrete next actions" was the right
one and shipped as `b059074 feat(cable-fidelity): plugs use thin-USD +
GLB-reference (socket pattern)`. Summary of what landed:

- `build_thin_glb_usds.py` (renamed from `build_mount_rail_usds.py` in
  the same wave) gained 3 plug TARGETS — SC Plug / SFP Module / LC Plug —
  with vendored `.glb` from `~/Documents/aic/aic_assets/models/<X>/` +
  thin USD wrappers alongside.
- `build_cable_variant_usds.py` gained
  `replace_plug_subtree_with_glb_refs()` which wipes the inline
  `sc_plug_visual` / `sfp_module_visual` subtrees and substitutes child
  Xforms that `AddReference` the thin GLB-USDs. Parent prims (with
  RigidBodyAPI + kinematic tracker writes) are preserved; only their
  children and parent MaterialBinding are touched. For `sc_plug`, parent
  `xformOp:scale` is reset 0.01→1.0 so the gltf-side meters-per-unit
  scale doesn't double-apply.
- Rope fixedJoint body1 paths needed no update — the parent connector
  prim (`/World/cable/sc_plug_visual`, `/World/cable/sfp_module_visual`)
  is the joint target and was preserved.
- **SFP body multi-primitive material handling (initially flagged as M2,
  later confirmed automatic):** the SFP source GLB's `Body.005` mesh has
  two GLB primitives sharing one mesh with different materials
  (`Material.005` metallic + `Material.001` non-metallic). `b059074`
  added a parent-level fallback `UsdPreviewSurface` on `sfp_body` on
  the assumption the gltf SDF plugin wouldn't synthesize per-primitive
  partitions. Live probe disproved that: at load time Isaac Sim 5.0's
  gltf SDF plugin authors
    `Body_005/Material_005`  GeomSubset family=materialBind faces[0..6683]   → Material_005
    `Body_005/Material_001`  GeomSubset family=materialBind faces[6684..6867] → Material_001
  and both subsets bind to inline Materials at `…/sfp_body/Mesh/Looks/`
  authored from the GLB's own material definitions. Subsets cover 100%
  of faces (6684 + 184 = 6868), so the fallback never propagated
  anywhere. `<followup_commit>` dropped both the unused fallback call
  for `sfp_body` and the `fallback_material` capability from the helper.

Verified live (trial_3, post-restart with venv-activate +
ROS_DOMAIN_ID=7):

| Connector | Faces | Resolved Material | Source |
| --- | --- | --- | --- |
| `sc_plug` | full mesh | `…/visual/Mesh/Looks/DODGERBLUE3_001` | GLB-side, full textures |
| `sfp_body` subset 0 | 6684 | `…/sfp_body/Mesh/Looks/Material_005` | gltf-SDF, GLB-side |
| `sfp_body` subset 1 | 184  | `…/sfp_body/Mesh/Looks/Material_001` | gltf-SDF, GLB-side |
| `lc_plug` | 10 meshes | `…/lc_plug/Mesh/Looks/Plastic_Blue_002` | GLB-side, full textures |

Pose tracker preserved (sc_plug REL TCP angular delta still 0.000°,
sfp_module far-end world pos still (+0.541, -0.018, +1.147) matching
Gazebo trial_3 sfp_module_link).

### 3. ❌ Gripper finger separation — fixed at 17.3mm, doesn't adapt per cable

**Root cause**: Per D-09, the Robotiq Hand-E gripper finger is currently a `PhysicsFixedJoint` (zero-DOF) in the cable USD. The two fingers are statically positioned 17.3mm apart. Gazebo's gripper opens to:
- 14.6mm for trial_1 (LC plug — small)
- 27.2mm for trial_3 (SC plug — wider)

Isaac Sim can't grip the wider SC plug realistically; the plug overlaps with the gripper fingers at the current width.

**Concrete next actions:**
1. Author the Hand-E finger as a `PhysicsPrismaticJoint` with DriveAPI in the cable USD (build_cable_variant_usds.py edit) — 1 DOF prismatic along the gripper-opening axis, drive stiffness for position control.
2. Set drive target via MCP atom (`gripper_command`) per cable_kwargs.gripper_initial_pos. Default 0.00655 currently does nothing (per D-09); should map to position 14.6mm for trial_1, 27.2mm for trial_3.
3. Per `nvidia-suite-docs` `isaac-sim/router.md`, the canonical Hand-E example might exist in `isaacsim.robot.manipulators` — worth checking before re-authoring from scratch.

## Verified working

Everything from prior session's wrap, plus:
- ✅ Both trial_1 (SFP+LC at gripper) and trial_3 (SC plug at gripper) — plug X between gripper fingers
- ✅ Per-tick TCP tracker tracks gripper motion (held connector moves with gripper)
- ✅ Orphan FixedJoint removed from both USDs at build time
- ✅ Cache snapshot: `DerivedDataCache.bak.1779012551`

## Test commands for next session

```bash
# Restart Isaac Sim with venv + ROS_DOMAIN_ID=7
bash -c 'source ~/env_isaaclab/bin/activate && ROS_DOMAIN_ID=7 DISPLAY=${DISPLAY:-:0} \
  bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'

# Quick test: load trial_3 + probe orientation
python3 /tmp/probe_trial3_state.py

# Visual: re-launch Gazebo and grab live wrist-cam for direct comparison
cd ~/Documents/aic && bash scripts/run_cheatcode.sh headless
# wait for trial_3 active, then:
docker exec aic_eval bash -lc 'source /opt/ros/kilted/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_CONFIG_OVERRIDE=";transport/shared_memory/enabled=false" && \
  python3 -c "..." # see prior session for the rclpy image-grab snippet'

# Isaac Sim wrist-cam capture (use existing helper)
python3 ~/Documents/isaac-sim-mcp/exts/aic-dt/scripts/capture_viewport_set.py \
  --out-dir ~/Share/aic_session_$(date +%Y%m%d_%H%M%S) \
  --cameras center_camera left_camera right_camera --auto-spawn
```

## Don't forget

- GPT pose diagnosis result (if it landed): `/tmp/gpt-pose-result-9174.txt`
- Gazebo ground-truth trial_3 image (already in Share): `~/Library/Mobile Documents/com~apple~CloudDocs/Share/aic_cable_fidelity_20260516_234420/POST_DUAL_WRITE/GAZEBO_trial3_center_GROUND_TRUTH.png`
- Trial_1 + trial_3 Isaac Sim renders: same folder, `trial{1,3}_center.png`
- Gazebo /tf bag for both trials saved locally:
  - `/tmp/bag_trial_1_20260517_085246_597/`
  - `/tmp/bag_trial_3_20260517_082306_946/`
  - Use to re-extract Gazebo's geometric ground truth at any time without re-running docker.

## Outstanding from HANDOFF.json (unchanged from prior session)

- `parity-09-motion-deficit` — 84% commanded-amplitude loss
- `parity-05-wrench-root-cause` — `/fts_broadcaster/wrench` publishes zeros
- `phase-4-03-model-zenoh` — zenoh ↔ fastrtps interop blocker for full E2E

These are unrelated to the cable-fidelity work and can be tackled in parallel.
