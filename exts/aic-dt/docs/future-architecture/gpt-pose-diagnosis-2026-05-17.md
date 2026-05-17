1. Most-likely root cause

Your rotation compose is in the wrong order for USD/Gf math. Confidence: 0.82.

In OpenUSD, `GfMatrix*` uses row-vector conventions, and the matrix on the left is the more-local transform. So in [extension.py](/home/aaugus11/Documents/isaac-sim-mcp/exts/aic-dt/aic_dt/extension.py:2148), this is backwards:

```python
held_rot_mat = tcp_rot_mat * rel_rot_mat
```

It should be:

```python
held_rot_mat = rel_rot_mat * tcp_rot_mat
```

Why this matters: OpenUSD documents that vectors pre-multiply matrices and that `R*T` means rotate then translate; `UsdGeomXformable` also says `xformOpOrder` is stored in least-to-most-local order, which is the opposite of many column-vector mental models. So your current code is effectively applying the Gazebo relative rotation in the wrong space, which is why the result collapses toward TCP orientation instead of `TCP ∘ rel`. Sources: OpenUSD `GfMatrix4d` row-vector convention and “left matrix is more local”; `UsdGeomXformable` xform-op ordering docs:  
https://openusd.org/24.08/api/class_gf_matrix4d.html  
https://openusd.org/24.08/api/class_usd_geom_xformable.html

Secondary issue, lower probability: `sc_plug_visual` is still a `RigidBodyAPI` prim with a rope-end `FixedJoint`, so driving it by xform writes is architecturally fragile. But that is not the main reason `rel_quat` looks ignored; the math is already wrong before PhysX arbitration.

2. Why the prior attempts failed

- `orient.Set(...)`: wrote the wrong local quaternion, because the world rotation was composed with the wrong matrix order first.
- Fabric write: same bug, just faster writeback to the same wrong target pose.
- Dual USD+Fabric write: same wrong quaternion authored twice; duplication cannot fix a bad compose.

3. Concrete fix recommendation

Immediate fix in the tracker:

```python
# USD/Gf row-vector convention: left matrix is MORE LOCAL.
# Desired held world rotation is TCP followed by relative plug rotation,
# so the relative rotation matrix must be on the LEFT.
rel_rot_mat = Gf.Matrix3d().SetRotate(rel_quat)
held_rot_mat = rel_rot_mat * tcp_rot_mat

held_wtm = Gf.Matrix4d(1.0)
held_wtm.SetRotateOnly(held_rot_mat)
held_wtm.SetTranslateOnly(held_world_pos)

parent_wtm = xc.GetLocalToWorldTransform(held.GetParent())
held_local_mat = held_wtm * parent_wtm.GetInverse()   # this part is fine for USD row-vector math
```

For the stable Isaac Sim 5.0 solution to Q6: if this body must remain kinematic, drive it through physics, not raw xform attrs. NVIDIA’s runtime docs recommend USDRT/Fabric for runtime changes, and the physics tensors API exposes explicit kinematic targets:

```python
import omni.physics.tensors as tensors
import warp as wp
import numpy as np

sim_view = tensors.create_simulation_view("warp")
rb_view = sim_view.create_rigid_body_view("/World/UR5e/cable/sc_plug_visual")

target = np.array([[x, y, z, qx, qy, qz, qw]], dtype=np.float32)
idx = wp.array([0], dtype=wp.int32, device="cpu")
rb_view.set_kinematic_targets(target, idx)
```

Docs:  
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/simulation_control/simulation_control.html  
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/106.5/extensions/runtime/source/omni.physics.tensors/docs/api/python.html

Two additional concrete recommendations:

- Remove the bare `except: pass` in the step callback while diagnosing. It can hide a failed `Set`.
- If the held SC plug is visual-only, the cleanest design is to stop making `sc_plug_visual` a rigid body at all and parent a visual proxy under `gripper_tcp`. If it must remain physical, then the rope-end `FixedJoint` means the connector is still semantically “owned” by the rope, not the gripper.

On Q3 and Q4 specifically:
- Q3: `xformOpOrder = [translate, orient, scale]` means least-to-most-local ops. For row-vector points, the effective point transform is `p * Scale * Orient * Translate`, not `T*S*R`. Sources above.
- Q4: a PhysX fixed joint enforces equality of the two joint frames: in row-vector notation, `W0 * L0 == W1 * L1`. PhysX docs describe this as the fixed joint constraining the positions and orientations of the two constraint frames to be the same. So yes, the joint can matter, but here it is probably secondary to the compose-order bug. Sources:  
https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/Joints.html  
https://openusd.org/dev/api/usd_physics_page_front.html

4. Does this block mesh-fidelity / gripper-DOF work?

Yes for held-plug visual fidelity; no for unrelated gripper-DOF work.

The orientation bug should be fixed before any further “SC plug looks wrong” or cable-mesh-fidelity debugging, because the current tracker is feeding the renderer/physics the wrong orientation. But it is independent of the separate gripper-DOF issue.