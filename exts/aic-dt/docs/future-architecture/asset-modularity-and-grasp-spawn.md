# Future Architecture Notes — Asset Modularity + Grasp/Spawn State

> **Status:** Forward-looking. Not blocking M1. Captures user-requested
> architectural direction surfaced during the 2026-05-16 cable-variant
> investigation so it isn't lost. Implement when a future project needs
> it; reference back here to avoid re-discovery.

## Context

While debugging "wrist camera shows wrong plug at gripper for trial_3"
(see commit history around 2026-05-16, scripts/build_cable_variant_usds.py,
and the GPT cable-fidelity analysis), the operator surfaced two
architectural directions for future projects. They are intentionally
not implemented now — aic-dt for M1 uses a monolithic unified USD with
a `plug_proxy` hack — but should be addressed when the patterns get
re-used in a different robot / different task.

## 1. Asset modularity — separate USDs per concern

**Goal:** instead of one `aic_unified_robot_cable_sdf.usd` that bundles
robot + gripper + camera-rig + cable + plugs as a single PhysX-cooked
articulation, split into addressable per-concern assets that can be
mixed and matched.

**Proposed split:**

| Asset | Owns |
|---|---|
| `robot_arm.usd` | UR5e (or other arm) articulation + joint drives + parity TF frames |
| `gripper.usd` | Robotiq Hand-E (or other gripper) + finger articulation + control surface |
| `camera_rig.usd` | Mount + Basler cameras + optical frames + URDF-derived poses |
| `cable_<type>.usd` | Cable rope + two terminal connectors (one USD per cable variant) |
| `plug_<sc|lc|sfp>.usd` | Single connector geometry + sensor frames |

**Composition surface:**
- A `scene.usd` references the above with relative paths and authors
  the cross-asset joints (gripper-to-cable, camera-to-arm) explicitly.
- Variants (USD VariantSet) on the wrapper scene select which gripper /
  cable / plug combination is in use per trial.

**Benefits over the current monolithic asset:**
- Asset swaps for future projects: a different arm, different gripper,
  different plugs — without re-baking a 16 MB unified USD.
- Cleaner physics cooking: each articulation cooks independently;
  cross-articulation joints become explicit composition decisions
  instead of implicit baked geometry.
- The `plug_proxy` workaround (extension.py:1955+) becomes unnecessary
  because the cable-to-gripper attachment is authored once at the
  composition layer with a real `FixedJoint`, not synthesized at
  runtime from a sphere-collider child of the finger link.
- Isaac Lab and aic-dt could share component assets instead of each
  carrying its own copy of the bundled USD.

**What blocks doing this for aic-dt right now:**
- The unified USD was provided pre-built by NVIDIA / intrinsic-dev and
  has been the substrate for many cooked-cache stability decisions
  (see CLAUDE.md cache-management section). Re-splitting it touches
  the entire scene-load pipeline + plug_proxy + cable physics +
  scoring TF publishers. Effort estimate: 1-2 weeks engineering with
  meaningful risk to currently-passing M1 trials.

**When to do this:**
- New project that doesn't inherit aic-dt's unified asset, OR
- aic-dt M2 cable-physics rework that's going to touch the cable
  topology anyway.

---

## 2. save/restore scene state — grasp/spawn fidelity

**Observed gap:** the existing `save_scene_state` / `restore_scene_state`
MCP atoms record per-prim world poses but **cannot restore "gripper
holding object X" semantics**. After restore, the object is at the
saved world pose but the gripper attachment relationship is gone — if
the gripper moves, the object stays put.

**Root cause:** USD save/restore captures attribute state but not
authored Joints created at runtime via `attach_cable_to_gripper`,
`_attach_cable_to_gripper_impl`, or the `plug_proxy` Xform graft. Those
relationships have to be re-authored after restore, but the
restore-state atom doesn't carry the info to do that.

**Proposed minimum-viable fix (for future use):**

`save_scene_state` should additionally record:
1. List of runtime-authored joints (path, jointType, body0, body1,
   localPos0, localRot0, localPos1, localRot1).
2. List of runtime-authored Xform grafts (e.g. plug_proxy under a
   gripper link) with parent path + child name + local xform.
3. The gripper's commanded joint position at save time so the gripper
   can be re-driven to that opening width after restore.

`restore_scene_state` should:
1. Restore per-prim poses (current behavior).
2. Re-author each saved joint (idempotent — replace if already there).
3. Re-graft each saved Xform child.
4. Re-drive gripper to saved joint position.
5. Tick physics N frames to let the joints settle, OR set
   kinematicEnabled=True briefly during settle.

**Pattern alternatives** (see references/usd-and-physics.md in the
isaac-sim-extension-dev skill for fuller treatment):
- **A. USD-hierarchy parenting** — object becomes a child prim of the
  gripper link. Survives save/restore automatically via USD composition,
  but the object isn't a separate dynamic body.
- **B. UsdPhysics.FixedJoint** — object stays at top level, joint
  constrains it. Needs explicit save/restore re-authoring.
- **C. Isaac's GripperBase / SurfaceGripper API** — framework manages
  grasp from contact; save/restore would need to record "gripper is
  closed on X" state and re-establish grasp by re-closing.

aic-dt's current `plug_proxy` is essentially Pattern A (proxy parented
to finger link). Pattern B + the save/restore plumbing above is what a
robust future-project save/restore needs.

---

## 3. References used to draft this

- 2026-05-16 GPT cable-fidelity analysis (Approaches 1-5 with
  effort/risk/ROI table) — `~/Documents/isaac-sim-mcp/exts/aic-dt/docs/`
  (commit chain near `b9d167a..`).
- 2026-05-16 live probe findings on the monolithic cable USD:
  - Connector visuals are free-floating PhysX rigid bodies with mass=0,
    diagonalInertia=0, centerOfMass=-inf — drift after spawn.
  - Plug-at-gripper semantic is provided by `plug_proxy` Xform graft
    under the gripper finger link, NOT by the cable subtree.
- extension.py:1888-1991 — the iter-1..7 history of trying to attach
  the cable to the gripper (closed-loop articulation cook crashes;
  current `plug_proxy` is "iter 7 PATH (a')" workaround).
- aic_assets/models/sfp_sc_cable[_reversed]/model.sdf — Gazebo's two
  source SDFs for each cable variant.
- aic_gazebo/src/CablePlugin.cc:228 — Gazebo's
  `DetachableJoint({endEffector, cableConnection0, "fixed"})` pattern
  that physically attaches the per-variant correct end at startup.
- aic_utils/aic_isaac/aic_isaaclab/.../aic_task_env_cfg.py:108-117 —
  Isaac Lab also uses the monolithic asset; cable Articulation is
  commented out; same gap.

When this directory accumulates more notes, factor them into a single
ARCHITECTURE.md and link back here.
