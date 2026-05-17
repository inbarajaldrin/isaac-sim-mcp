# Next Session — Pickup Point

> Written 2026-05-16 at session pause. Read this first when resuming.

## Confirmed status (user verified 2026-05-16)

**The cable-end visual in Isaac Sim trial_3 does NOT match the Gazebo docker
launch of trial_3.** Docker trial_3 spawns the SC plug at the gripper for
insertion into sc_port_base. Isaac Sim trial_3 currently spawns the SFP+LC
end at the gripper instead — wrong visual identity.

**This is the priority fix for next session.**

## Where we left it

The fix has two layers and we've completed only layer 1:

### Layer 1: end-anchor authoring (DONE this session — commits not yet landed)

- `exts/aic-dt/scripts/build_cable_variant_usds.py` — refactored by GPT to
  the end-anchor pattern (composes per-end orient = base end orient × per-end
  yaw from Gazebo SDF). Final TRS values applied to reversed USD:
  - `/World/cable/sfp_module_visual`: translate=(1,0,0), orient=(0.707,0,0,-0.707), scale=(1,1,1)
  - `/World/cable/sc_plug_visual`:    translate=(0,0,0), orient=(0.5,0.5,0.5,0.5), scale=(0.01,0.01,0.01)
- `exts/aic-dt/aic_dt/extension.py` — load_robot branches on cable_type:
  cable_type=sfp_sc_cable_reversed → loads `aic_unified_robot_cable_sdf_reversed.usd`
- Kit log at trial-load time confirms the right USD is loaded:
  `[AIC-DT] cable_type=sfp_sc_cable_reversed → loading file:///.../aic_unified_robot_cable_sdf_reversed.usd`

### Layer 2: constrain free-rigid-body drift (NOT YET DONE — the actual fix needed)

GPT's warning + verified live: the connector visuals are PhysX RigidBody
prims with mass=0. PhysX reads the authored translate as INITIAL position
at spawn, then drifts them toward whichever rope link they spawn near. So
the end-anchor authoring is necessary but not sufficient — at runtime the
SC plug visual still ends up near the FAR rope end (not the gripper end)
because that's where the rope's free rigid body settles.

## What to do next session — concrete plan

Pick ONE (in order of effort):

### Option A — kinematic flag on connector visuals (fastest, ~15 min)

Extend `build_cable_variant_usds.py` to also author
`physics:kinematicEnabled = True` on both `/World/cable/sfp_module_visual`
and `/World/cable/sc_plug_visual` in BOTH the original AND reversed USDs.
PhysX treats kinematic bodies as scripted — they stay at authored pose,
no drift. Then re-test trial_3 freeze and verify SC plug visually at the
gripper end.

Implementation sketch:
```python
def author_kinematic(stage, prim_path):
    from pxr import UsdPhysics, Sdf
    p = stage.GetPrimAtPath(prim_path)
    p.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
```

Caveats:
- Need to verify this doesn't break the `plug_proxy` scoring path
  (it shouldn't — plug_proxy is a separate Xform under the gripper finger,
  unrelated to these connector rigid bodies).
- May need to also author on the ORIGINAL USD or only-apply-when-reversed
  depending on whether the user wants the original behavior preserved.

### Option B — FixedJoint from each connector to its rope-end link (slower, ~1h)

Author `UsdPhysics.FixedJoint` constraining sfp_module_visual to its
rope-end link and sc_plug_visual to its rope-end link. Matches Gazebo's
CablePlugin pattern. More work but produces real physical attachment
(connectors follow rope motion if rope is ever made dynamic in M2).

This is also the option that maps to the future-arch doc's "save/restore
with grasp" pattern — if we go this route the
`save_scene_state`/`restore_scene_state` MCP atoms also need to round-trip
these joint definitions.

### Option C — reparent connector visuals under rope-end link Xforms

USD-hierarchy parenting: move sfp_module_visual and sc_plug_visual to be
CHILDREN of `Rope/link_X` Xforms (which one each goes under depends on
gripper-end-of-rope identity). USD transform composition then gives each
connector the rope link's pose for free. No physics joint authoring needed.

Caveat: changes prim paths (`/World/cable/sc_plug_visual` →
`/World/cable/Rope/Rope/link_X/sc_plug_visual`), which may break the
`build_cable_variant_usds.py` constants AND any extension.py code that
references those paths by name. Audit required first.

## Helper scripts ready for next session

- `exts/aic-dt/scripts/capture_viewport_set.py` — capture wrist cameras
  (or any USD camera) as PNG via Isaac Sim's viewport API. Replaces
  X11 window screenshots for clean center-framed renders.

  ```
  python3 exts/aic-dt/scripts/capture_viewport_set.py \\
      --out-dir ~/Share/my_capture_$(date +%Y%m%d_%H%M%S) \\
      --auto-spawn
  ```

- `exts/aic-dt/scripts/run_freeze_isaac_sim.sh` — launches the AIC engine
  + model + Isaac Sim stack with `AIC_FREEZE_AT_HOVER=1` so CheatCode holds
  the robot at the hover pose for visual inspection.

  ```
  bash exts/aic-dt/scripts/run_freeze_isaac_sim.sh trial_3 hover
  # then in another shell:
  python3 exts/aic-dt/scripts/capture_viewport_set.py \\
      --out-dir ~/Share/trial3_inspection \\
      --auto-spawn
  ```

## Uncommitted state to commit at start of next session

```
build_cable_variant_usds.py            — refactored to end-anchor pattern
aic_unified_robot_cable_sdf_reversed.usd — regenerated with new TRS
extension.py                            — load_robot cable_type branch
capture_viewport_set.py                 — new (this session)
docs/future-architecture/NEXT-SESSION.md (this file)
```

## Don't forget

- GPT diagnostic chain is preserved at:
  - `/tmp/gpt-cable-fidelity-result-9173.txt` — 5-approach analysis
  - `/tmp/gpt-cable-asset-result-9173.txt` — visual disambiguation (with composite image)
  - `/tmp/gpt-reversed-transform-result-9173.txt` — identified per-end orient as Layer 1 bug
  - `/tmp/gpt-end-anchor-result-9173.txt` — implementation report for Layer 1 fix
  Copy these into `.tug/findings/` or similar before they age out of /tmp.
- Reference HTML for visual sanity: `~/Share/gazebo_cable_asset_reference_20260516_220716/index.html`
