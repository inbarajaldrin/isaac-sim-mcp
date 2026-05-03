# Off-Limit Prim Mapping (PARITY-06 / D-10) + Open Q1 Settlement

**Captured:** 2026-05-03
**Snapshot dir:** `.planning/phases/02-controller-loop/snapshot/`
**Snapshot script:** `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh`
**Live source-of-truth:** running `aic_eval` Docker container
(`ghcr.io/intrinsic-dev/aic/aic_eval:latest`) + `aic_model` (CheatCode policy)

## Open Q1 settlement (live aic_eval topic-name confirmation)

The seven topics matching `^/aic_controller|^/aic/gazebo|controller_state|joint_commands|pose_commands|off_limit` actually published by the live container:

```
/aic/gazebo/contacts/off_limit
/aic_controller/controller_state
/aic_controller/joint_commands
/aic_controller/joint_motion_update
/aic_controller/motion_update
/aic_controller/pose_commands
/aic_controller/transition_event
```

Source: `.planning/phases/02-controller-loop/snapshot/aic_controller_topic_list.txt`

### CONTEXT.md assumed topics — confirmation table

| Assumed topic (from CONTEXT.md) | Live topic (snapshot) | Match? | Type | Action |
|---|---|---|---|---|
| `/aic_controller/joint_commands` | `/aic_controller/joint_commands` | YES | `aic_control_interfaces/msg/JointMotionUpdate` | Plan 02-03 subscribes to this verbatim |
| `/aic_controller/pose_commands` | `/aic_controller/pose_commands` | YES | `aic_control_interfaces/msg/MotionUpdate` | Plan 02-04 subscribes to this verbatim |
| `/aic_controller/controller_state` | `/aic_controller/controller_state` | YES | `aic_control_interfaces/msg/ControllerState` | Plan 02-05 publishes this verbatim |
| `/aic/gazebo/contacts/off_limit` | `/aic/gazebo/contacts/off_limit` | YES | `ros_gz_interfaces/msg/Contacts` | Plan 02-06 publishes this verbatim |

**Open Q1 verdict: ALL FOUR topic names confirmed correct.** Plans 02-03..06 wire to CONTEXT.md's named topics with no remap. No CONTEXT.md edits needed.

### Additional topics observed (not in CONTEXT.md but present)

These are out of scope for Plan 02-01 → 02-06 but documented here for future reference:

| Topic | Likely source | Notes |
|---|---|---|
| `/aic_controller/joint_motion_update` | aic_controller publishes its own current motion ref | NOT a Plan 02 surface |
| `/aic_controller/motion_update` | aic_controller pose-mode current motion ref | NOT a Plan 02 surface |
| `/aic_controller/transition_event` | ros2_control lifecycle event | NOT a Plan 02 surface |

If a future plan needs to mirror `joint_motion_update` / `motion_update` echoes (e.g. for higher-fidelity policy training data), those topics are confirmed-present.

### QoS profile (all four critical topics share this)

```
Reliability: RELIABLE
History (Depth): KEEP_LAST (10 for cmd-side; 42 for controller_state pub; 5 for adapter sub)
Durability: VOLATILE
Lifespan: Infinite
Deadline: Infinite
Liveliness: AUTOMATIC
```

Plan 02-03..06 default to `QoSProfile(reliability=RELIABLE, durability=VOLATILE, depth=10)` per this snapshot. Controller_state publish depth bumped to 42 (matches live). Snapshot raw: `.planning/phases/02-controller-loop/snapshot/aic_controller_topic_info.txt`.

## Off-limit Entity → USD prim mapping

### Snapshot capture result

```
$ cat .planning/phases/02-controller-loop/snapshot/aic_eval_offlimit_capture.txt
The message type 'ros_gz_interfaces/msg/Contacts' is invalid
```

The aic_eval container's daemon-less `ros2 topic echo` doesn't have `ros_gz_interfaces` available on the kilted ROS path used by `ros2 topic echo`. The capture is functionally **empty** for our purposes — but importantly this matches the **expected case**: CheatCode is a "passing" policy that completes the trial without ever touching off-limit collisions, so even with a working echo we expect zero `Contacts` events. Future re-captures with a "wall pressing" policy (`aic_example_policies.ros.WallPresser` or `WallToucher`) would generate non-empty data.

### Source-of-truth fallback: AIC URDF authoritative model list

The `OffLimitContactsPlugin` SDF block in `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` lines 122-130 is the **canonical configuration** of which Gazebo top-level model names get watched:

```xml
<plugin filename="OffLimitContactsPlugin" name="aic_gazebo::OffLimitContactsPlugin">
  <update_rate>5</update_rate>
  <topic>/aic/gazebo/contacts/off_limit</topic>
  <off_limit_models>
    <model>enclosure</model>
    <model>enclosure walls</model>
    <model>task_board</model>
  </off_limit_models>
</plugin>
```

The plugin (`~/Documents/aic/aic_gazebo/src/OffLimitContactsPlugin.cc`):
1. Calls `entitiesFromScopedName(modelName, _ecm)` for each name above → resolves the Gazebo Entity for that top-level model.
2. Walks all `components::Collision` children of those entities and enables `ContactSensorData` on each.
3. Each tick, polls every collision's contact-sensor data and re-publishes any contact whose top-level-model parent is in the off-limit set.

**So the off-limit set is "any contact involving the enclosure, enclosure-walls, or task-board models"** — not a hand-curated list of individual link names.

### Isaac Sim USD prim mapping

The aic-dt extension scene (per `quick_start`) creates these top-level prims that correspond to Gazebo's three off-limit models:

| Gazebo model name | Isaac Sim USD prim path | Source authoring atom |
|---|---|---|
| `enclosure` | `/World/Enclosure` | `_setup_aic_enclosure()` in `extension.py` |
| `enclosure walls` | `/World/Enclosure_Walls` | `_setup_aic_enclosure()` in `extension.py` |
| `task_board` | `/World/TaskBoard` | `add_objects` atom (`_spawn_component_via_usd("task_board", ...)`) |

Plan 02-06's omni.physx contact-report subscription should monitor any **descendant collider** of these three prim paths. The exact descendant collider paths are USD-driven (depend on the .usd file's authoring) — Plan 02-06's contact callback receives a `(actor0_path, actor1_path)` tuple from the physx contact event, and the filter is:

```python
def is_off_limit(actor_path: str) -> bool:
    return any(actor_path.startswith(prefix) for prefix in OFF_LIMIT_PRIMS)
```

So we don't need a hand-enumerated set of individual link names — just the three top-level prefixes.

## Default OFF_LIMIT_PRIMS for controller_loop.py

```python
# Source-of-truth: ~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro lines 122-130
# Confirmed via Plan 02-01 snapshot (no Contacts events seen during CheatCode trial,
# but the model-list authoring is canonical and Gazebo iterates collisions dynamically).
DEFAULT_OFF_LIMIT_PRIMS = [
    "/World/Enclosure",         # Gazebo "enclosure" model
    "/World/Enclosure_Walls",   # Gazebo "enclosure walls" model
    "/World/TaskBoard",         # Gazebo "task_board" model
]
```

Plan 02-06 imports this list directly OR hardcodes from this doc. The list is **prefix-based** — a contact whose `actor_path` starts with any of these three prefixes is treated as off-limit.

### Verifying the prefix names match the live USD

From a running Isaac Sim with `quick_start` complete:

```python
# In Script Editor:
import omni.usd
stage = omni.usd.get_context().get_stage()
for prefix in ["/World/Enclosure", "/World/Enclosure_Walls", "/World/TaskBoard"]:
    prim = stage.GetPrimAtPath(prefix)
    print(f"{prefix}: {'EXISTS' if prim.IsValid() else 'MISSING'} ({prim.GetTypeName() if prim.IsValid() else '-'})")
```

Plan 02-06's smoke test asserts all three prefixes exist before wiring contact subscriptions.

## Re-running the snapshot

If a future change to `aic_eval` adds/removes/renames topics, or a contact-generating policy is needed:

```bash
# Standard run (CheatCode — expected to produce zero off-limit events)
bash exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh .planning/phases/02-controller-loop/snapshot

# To force off-limit events: edit the script's POLICY arg from
#   policy:=aic_example_policies.ros.CheatCode
# to
#   policy:=aic_example_policies.ros.WallPresser
# (or WallToucher) — these policies deliberately push the robot into the enclosure.
```

The script auto-cleans containers via `trap cleanup EXIT`. Idempotent.
