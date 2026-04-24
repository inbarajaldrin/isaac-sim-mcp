# SO-ARM101 Pick-and-Place Debug Guide

**Living document.** If you diagnose something new, *add it here*. The pattern at
the bottom tells you how. Future agents: this guide is yours to extend.

Scope: everything Claude/you need to interpret the pick-and-place loop in
Isaac Sim + MoveIt + ros2_control. Tracking lag, cup collisions, IK failures,
post-check rejections, gripper attach issues, stale `/drop_poses`, command-path
latency — one place.

---

## 1. Start here: what to check first

Before anything else, confirm the stack is healthy:

```bash
# 1. Is Isaac Sim's MCP socket alive?
ss -tlnp 2>/dev/null | grep :8767        # expect a listener

# 2. Is the control stack up?
ros2 node list | grep -E 'so_arm101_control_gui|move_group|controller_manager'
# expect all three

# 3. Is the motion telemetry logger running?
scripts/motion_log.sh status             # expect "supervisor RUNNING"
```

If any are down:
- MCP socket down → `bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch soarm101-dt`
- Control stack down → `scripts/restart-control-stack.sh`
- Logger down → `scripts/motion_log.sh start` (supervisor will keep it alive)

## 2. The canonical bring-up sequence

See `CLAUDE.md` § Bring-up Sequence. TL;DR:

1. `isaacsim_launch.sh launch soarm101-dt` — blocks until MCP ready
2. Call `quick_start` MCP tool to spawn scene + robot + action graphs
3. `ros2 launch so_arm101_control control.launch.py rviz:=true` (or use
   `scripts/restart-control-stack.sh`)
4. Call `publish_drop_poses` MCP tool to seed `/drop_poses` from the live cup
   positions (the GUI's "Update Drops" button does the same)

## 3. The telemetry rig

Three scripts, all in `scripts/`, all permanent:

| Script | Role |
|---|---|
| `motion_logger.py` | Auto-detects every motion, records at 50 Hz |
| `motion_log_supervisor.sh` | Respawns `motion_logger.py` if it dies |
| `motion_log.sh` | `start \| stop \| restart \| status \| tail \| latest \| scan \| verify` |
| `motion_analyze.py` | Pure-Python reader — detailed report from one CSV or summary of all |
| `motion_verify.py` | **MoveIt collision replay** — pipes plan & Isaac trajectories through `/check_state_validity` + `/compute_fk` on the live move_group (§ 4.1). `--fine-interp MS` adds the § 4.2 URDF thin-mesh sweep. |
| `motion_sweep.py` | **Standalone URDF thin-mesh sweep** — local PyKDL FK, linearly interpolates between 50 Hz samples at MS-ms resolution, answers "did any mesh vertex *actually* enter a cup?" (§ 4.2). Use for cross-motion characterization. |
| `_mesh_sweep.py` | Shared helper: URDF parser + local FK + STL vertex transformer. Imported by both `motion_verify.py` (optional fine-interp pass) and `motion_sweep.py`. |

**What gets captured per motion**, one CSV + one JSON in `~/motion_logs/YYYY-MM-DD/`:

- **ros2_control view:** `ref_pos`, `ref_vel`, `fb_pos`, `err_pos` from
  `/arm_controller/controller_state` (JTC's live interpolation of the plan)
- **Mock-hardware view:** `js_pos`, `js_vel` from `/joint_states` (what Isaac's
  action graph subscribes to)
- **Isaac Sim physics view:** `isaac_pos`, `isaac_vel` via MCP DynamicControl
  (the arm's *actual* joint state in simulation)
- **Derived:** `isaac_lag = ref_pos − isaac_pos` per joint per sample
- **TCP world pose:** via gripper-link USD xform × fixed tcp_joint offset
- **All cup poses** (`cup_red/green/blue`): position + quat
- **All lego poses** (red/green/blue × 2x2/2x3/2x4): position + quat
- **Sim rate:** `physics_rate_hz`, `sim_time_s`, `render_fps`, `app_dt_ms`

The paired JSON has the summary (peak/mean/diff lag, cup proximity, cup
displacement, RTF, FPS stats) so you don't always need to re-parse the CSV.

**Using it:**

```bash
scripts/motion_log.sh latest     # verbose diagnosis of most recent motion
scripts/motion_log.sh scan       # table of every motion on disk
# Specific file:
scripts/motion_analyze.py ~/motion_logs/2026-04-23/20260423T072537_grasp_home.csv
```

## 4. How to read analyzer output — the 3-layer lag model

The analyzer's **"Three-layer tracking decomposition"** section is the most
important single thing this rig produces. It splits total plan-to-physics
tracking error into two stacked sources:

```
MoveIt planner trajectory (in /tmp/arm_traj/*.json)
        │
        ▼ JTC interpolates @ 50 Hz                          ← Layer A
        │
/arm_controller/controller_state.reference.positions
        │
        ▼ mock_components → /joint_states → OmniGraph subscribe → Isaac drive
        │                                                      ← Layer B
        ▼
Isaac Sim physics joint state
```

- **Layer A (plan → JTC ref):** planner's interpolation + action-server lead time.
  In SO-ARM101 today this is **~1–2°** regardless of joint/velocity. JTC is
  NOT the bottleneck.
- **Layer B (JTC ref → Isaac):** drive PD tracking + command-path latency
  (OmniGraph tick rate, action-graph subscribe queue depth, PhysX drive
  response). Here SO-ARM101 sees **~10–21°** during fast shoulder_pan motions.

**Lag budget ≈ 90 % Layer B, 10 % Layer A** across all captured grasp_home
motions as of 2026-04-23. Any fix targeting Layer A (e.g. tightening
controller update rate) is low-leverage. Levers on Layer B:
1. Command-path latency — tick rate, queue depth on `ROS2SubscribeJointState`
2. `UsdPhysics.DriveAPI.targetVelocity` feed-forward (currently 0 — **not** being
   written by `IsaacArticulationController` despite `velocityCommand` wire)
3. PD gains (`stiffness`, `damping`) and `max_force`
4. Trajectory speed (lower `velocity_scale_var` in the GUI → less demand)

## 4.1 Collision replay — turning rich data into a dry-run verifier

`scripts/motion_verify.py` (wired into `scripts/motion_log.sh verify`) closes
a diagnostic gap the analyzer can't touch: *did the executed Isaac trajectory
ever enter a state MoveIt would have rejected, even though the plan itself
was validated?* The plan passed MoveIt's sub-segment post-check. Isaac's arm
— via Layer-B lag (§ 4) — may have visited a **different** joint configuration.
MoveIt never sees that. This tool does.

What it runs, per captured motion:

1. `/check_state_validity` on **every plan waypoint** → should be ~100%
   valid (sanity check on the planner). Tail waypoints of `drop_sweep` will
   show expected contacts between the attached lego and the target cup — see
   § 8.2; those aren't bugs, that's "drop is geometrically a cup contact".
2. `/check_state_validity` on **Isaac physics samples** (downsampled via
   `--every N`, default N=2). Any invalid samples here *are* the off-plan
   collisions that the plan-time checker missed.
3. `/compute_fk` on plan-ref joints per sample → compares TCP Cartesian
   position to the Isaac TCP (from the CSV). Reports peak / mean / RMS
   divergence as a 3D-space proxy for differential lag.

Scene correctness — the tool patches two live-scene defects automatically
so the results aren't noise:

- **Cup position override:** MoveIt's live scene often has `cup_drop_*`
  meshes at origin (0,0,0) after a restart because the GUI hasn't pushed
  drop positions. The verifier reads `scene_at_plan_time.drop_data` from
  the plan dump and overrides `cup_drop_N.pose` via `/apply_planning_scene`
  (**root pose**, not `mesh_poses[]` — which are *relative* to the root;
  the MoveIt gotcha that bit this project). Restores on exit.
- **Stale live attachment:** if a prior `lego_*` is still attached, it
  produces ghost contacts with our injected motion-time attached lego. The
  tool reads the current attachments from the scene and inlines a `REMOVE`
  for them via the `robot_state.attached_collision_objects` diff per check.
  No scene mutation — just per-check override.

Usage:

```bash
# Verify the latest motion (default — no args)
scripts/motion_log.sh verify

# Specific motion
scripts/motion_log.sh verify ~/motion_logs/2026-04-23/20260423T091239_drop_sweep.csv

# Downsample for speed (default --every 2 → 25 Hz)
scripts/motion_log.sh verify --every 5

# All captured motions (slow — ~5s/motion × count)
scripts/motion_log.sh verify --scan
```

What a report tells you (worked example from 07:34 `grasp_home`, 288 mm cup knock):

- Plan waypoints 0-4 show `cup_drop_1 ↔ gripper` (depth ~4-8 mm) at the
  start — i.e. the starting joint config left the gripper already inside
  cup_green's collision mesh. The knock mechanism is **not** mid-motion
  Layer-B lag; it's **carrying over a drop-pose start state** where the
  arm was parked inside the cup after the preceding `drop_sweep`.
- Then isaac's samples 8-12 show the collision shifting to cup_drop_2 as
  the arm sweeps — but the damage (the 288 mm displacement) was done by
  rim-drag from that starting state.

When to run it:

- After any surprising knock or near-miss — the verifier tells you
  *which link hit which cup, at what depth, at which sample* — data the
  analyzer and the motion's summary JSON don't expose.
- As part of validating a proposed fix: run with `--scan` before/after to
  see the count of `IN COLLISION` samples shift across the motion library.

Caveats:

- The cup is a **hollow mesh**, not a solid cylinder. FCL reports contact
  whenever the gripper intersects a triangle — it does *not* distinguish
  "inside the cup cavity (harmless)" from "hitting the wall (knock)". So
  "gripper ↔ cup_drop_N" at 3 mm depth while the arm is over an open cup
  may not be a physical knock. Use the `cup displacement` field from the
  analyzer's summary as the physical-truth ground state. The verifier's
  strength is **localization**: *when* and *where* on the trajectory
  Isaac diverged from plan into the cup's volume.
- MoveIt also applies **default per-link collision padding** (a few mm) on
  top of the STL surface. That means `/check_state_validity` reports
  "depth 4 mm" even when the thin STL isn't anywhere close to intersecting.
  Use § 4.2 to know whether the underlying geometry actually overlaps.
- Lego attached-body is approximated as an axis-aligned bounding box
  (dimensions table in `motion_verify.py`). Acceptable for gripper-link
  contact detection; wouldn't catch sub-mm tolerance geometry.

## 4.2 Fine-interp URDF mesh sweep — catching sub-sample geometric contact

`motion_verify.py --fine-interp MS` adds a second pass that uses **local
FK + URDF thin-mesh** (no MoveIt, no padding) and linearly interpolates
between adjacent 50 Hz Isaac samples at MS-ms resolution. It answers the
strictest possible geometric question:

    Is any vertex of the arm's URDF collision STL *actually inside* any
    cup's cylindrical volume (z < 96.5 mm AND dh < 39 mm from cup axis)
    at any continuous instant during the motion?

This is different from § 4.1 in three ways:

| | § 4.1 validity replay | § 4.2 fine-interp mesh sweep |
|---|---|---|
| Checker | MoveIt `/check_state_validity` | Local PyKDL FK + numpy |
| Geometry | Thin STL + MoveIt link **padding** (mm-scale safety margin) | Thin STL, **no padding** |
| Sample rate | 50 Hz (20 ms) | MS ms (default 2 ms, 10× finer) |
| What triggers | Any triangle intersection (including from padding) | Actual vertex inside cup volume |

### How to use

```bash
# One motion, 2 ms fine sweep:
scripts/motion_log.sh verify <csv> --fine-interp 2

# Scan all motions, 5 ms (faster for bulk characterization):
scripts/motion_log.sh verify --scan --fine-interp 5
```

Or the standalone driver for characterization only:
`scripts/motion_sweep.py --characterize --resolution 5`

### Tooling

- `scripts/_mesh_sweep.py` — helper module: loads the SO-ARM101 URDF,
  parses collision mesh origins, builds a PyKDL-style local FK chain, and
  pre-transforms the STL vertices into link frame. Exposes `MeshSweeper`
  with `check_joints(joints, cup_positions)` and `sweep_motion(rows, plan)`.
- `scripts/motion_sweep.py` — characterization driver (detail mode +
  cross-motion summary).
- `scripts/motion_verify.py --fine-interp MS` — integrated into the main
  verifier; runs alongside the MoveIt validity replay.

### Finding (2026-04-23 across 59 captured motions, 5 ms sweep)

- Only **2** motions (3%) showed any fine-interpolated thin-mesh intrusion.
- Worst intrusion depth: **1.65 mm** for 38 ms on one "unknown"-tag motion.
- **All** documented cup knocks (07:16 drop_sweep 217 mm, 07:25 grasp_home
  260 mm, 07:34 grasp_home 288 mm) show **zero** thin-mesh intrusion at
  any 2 ms interpolated instant.

**Interpretation.** The thin URDF collision mesh does not enter the cup's
cylindrical volume during normal pick-place motions. Cup knocks are
therefore not a geometry failure — they are **PhysX-side collision
approximation artifacts**. Isaac Sim uses `convexHull` (or similar) for
the arm links, which inflates the effective collision boundary by 2-4 mm.
That inflated hull can graze the cup rim where the thin STL stays clear.

The fix target therefore shifts:

- Not: "the plan puts the arm in collision" (MoveIt's plan is correct).
- Not: "Layer-B lag drives Isaac into a cup-intersecting joint config"
  (Isaac's actual joint states also keep the thin mesh clear).
- **Yes**: either (a) raise the release-pose clearance so even the
  inflated convex hull has margin, or (b) switch the arm's Isaac-side
  collision approximation to `convexDecomposition` / `sdf` so it matches
  the URDF thin-mesh more tightly. See CLAUDE.md "Collision Approximation
  Guide" for the tradeoffs.

This split — "MoveIt thinks contact, geometry says no; PhysX thinks
contact, PhysX moves the cup" — is the important thing to keep in mind.
§ 4.1 catches MoveIt-visible issues, § 4.2 catches strict-geometry issues,
and for a true physical knock you need the § 4.1 + § 4.2 + § 4.3 + the
motion analyzer's `cup_disp_mm` row — together they triangulate which
collision system was responsible.

## 4.3 The real knock mechanism — PhysX `contactOffset` padding

After the § 4.2 fine-interp sweep showed that gripper STL **never enters
the cup volume** yet cups physically knocked 200-290 mm, we dug into
Isaac Sim's PhysX collision model. Key finding:

**The SO-ARM101 URDF does not author `physxCollision:contactOffset` on
the gripper/jaw/wrist collisions.** Verified via USD introspection —
`co_attr.IsAuthored() == False` on all three. When unauthored, PhysX
falls back to its internal default: **`contactOffset = 0.02 m (20 mm)`**.

`contactOffset` is the distance at which PhysX starts generating contact
forces between two collision shapes. Two shapes closer than this
distance experience soft repulsive + damping forces — even when their
meshes don't geometrically overlap. Under lateral arm motion, these
forces translate into sideways impulses on the cup, accumulating over
milliseconds of near-contact into sliding displacement.

### How to measure it — the min-clearance metric

`scripts/motion_sweep.py` + `motion_verify.py --fine-interp` now compute,
per motion, the minimum distance from any gripper/jaw/wrist mesh vertex
to the cup's cylindrical collision surface (approximating the cup's
convex-decomposition as a solid cylinder, which is what PhysX "sees"
from outside the cup). Distances are compared to the 20 mm contact
zone:

```
closest approach : 2.12 mm  (jaw ↔ cup_drop_2 at t=1.161s)
PhysX contactOffset (default)  : 20.0 mm
→ SEVERE intrusion into PhysX contact zone (89% deep)  ⚠
```

### Finding (59 motions, 2026-04-23)

Clearance distribution across all captured motions, vs. whether the
motion physically knocked a cup:

| Closest approach | Count | Physical outcome |
|---|---|---|
| 0 mm (geometric overlap) | 2 | 1 known knock (07:16 drop_sweep, 217 mm) + 1 tiny-depth "unknown" |
| 0–5 mm | 3 | All knocked (07:25 grasp_home 260 mm, 07:34 grasp_home 288 mm, 094259 1.4 mm on "unknown") |
| 5–10 mm | 2 | All clean, but within the contact zone — PhysX forces present but below knock threshold |
| 10–20 mm | 6 | All clean, soft contact forces only |
| > 20 mm | 46 | Outside contact zone entirely, safe |

**Rough threshold for a visible cup knock: ~5 mm clearance to cup surface.**
Above that, contact forces exist but don't accumulate into meaningful
cup displacement in the ~0.5 s sweep window. Below that, they do.

### Why the same motion is sometimes clean, sometimes a knock

Layer-B lag + OMPL random arcs (§ 4, § 9) shift the gripper's spatial
trajectory by 10-80 mm between runs. Near-clean motions sit at ~15 mm
clearance; unlucky ones land at ~2-3 mm. That's the variance bucket.

### Fix directions

Ranked by invasiveness:

1. **Author `physxCollision:contactOffset = 0.002` on the gripper/jaw/wrist
   collision prims** at scene build time (extension.py). This would tighten
   the contact zone from 20 mm → 2 mm, leaving only physical-contact
   cases as knocks. *Risk*: PhysX uses contactOffset internally for
   numerical stability at high relative velocity — setting it too low
   can cause tunneling. 2 mm is still above the 0.05 × char-length
   threshold NVIDIA recommends, so likely safe.
2. **Shape motions to keep jaw ≥ 20 mm from cups** during sweeps. Pre-
   home vertical-lift segment on grasp_home (§ 4.1), or tighter
   drop_release pose. Avoids the contact zone entirely.
3. **Raise the SDF resolution/margin** — the 10 mm SDF margin might also
   contribute; lowering it to 2 mm plus setting contactOffset might help
   further. *Cost*: SDF regeneration at `resolution: 256` is expensive
   — ~48 GB peak RAM during voxelization observed (1 trigger caused the
   workstation to OOM-hang during this session). Only touch if the
   contactOffset fix alone doesn't resolve.

### The permanent tool chain

- `scripts/motion_sweep.py <csv>` — per-motion clearance report
- `scripts/motion_sweep.py --characterize` — cross-motion table with
  `in_zone` 🔥 flag and min clearance column
- `scripts/motion_verify.py --fine-interp MS` — clearance included in
  the integrated report
- `scripts/_mesh_sweep.py` — helper with `min_clearance_to_cups()`
  (analytical solid-cylinder distance, fully vectorized numpy — fast)

### Collision-system hierarchy (who sees what)

| System | What it sees | Padding | Sample rate |
|---|---|---|---|
| MoveIt `/check_state_validity` | STL triangles + per-link `link_padding` (2-10 mm) | Yes, explicit | plan waypoints (sparse) |
| Motion-verify fine-interp (§ 4.2) | STL triangles, NO padding | No | 1-5 ms interpolation |
| motion-sweep clearance (§ 4.3) | STL vertices vs cup cylinder | Reports clearance vs 20 mm PhysX zone | 50 Hz coarse |
| Isaac Sim PhysX runtime | SDF (gripper/jaw) + convexDecomp (cups), `contactOffset = 20 mm` | Yes, implicit via contactOffset | 120 Hz physics |

The 20 mm PhysX contactOffset is the one that bit us. It's invisible to
the plan checker (MoveIt has its own padding, different mechanism) and
invisible to the thin-mesh geometric check (no overlap). Only the
clearance metric surfaces it.

## 5. The differential-lag verdict

> `peak differential = max(|lag_j|) − min(|lag_j|) across joints at each sample`

The analyzer prints one of two verdicts at the end of the differential-lag
block:

- **Uniform lag (all joints track together within ~0.6°):** Isaac's spatial
  path is a time-shifted copy of the planned path. Cup-static collisions via
  lag are *ruled out* in this mode.
- **Differential lag detected:** Isaac's arm visits joint configurations that
  are NOT on any planned waypoint. 3D path diverges from plan. Post-check-safe
  plans can yield collisions.

Why this matters: a safe trajectory (validated by MoveIt's sub-segment post-
check at 20 samples/seg) assumes the physical arm will visit the plan's
waypoints — not interpolated blends of different plan moments. Differential
lag breaks that assumption.

## 6. Cup-hit diagnosis flow

When you see `⚠⚠ KNOCKED` on a cup in a motion's summary, walk this decision
tree (bookmarks to known mechanisms):

1. **Which cup moved, and during which motion?** `cup_disp_mm` timestamp points
   at the precise moment.
2. **Is the TCP close to that cup in the plan?** If `closest_approach_mm` for
   the hit cup is > 100 mm, the TCP itself didn't hit — it's probably an arm
   LINK (gripper/wrist) that got close. See § 7 on arm-link collision geometry.
3. **Which joint has the biggest Layer B lag at the hit moment?** Look at the
   per-joint lag at the differential-lag peak.
4. **Does that joint's axis point *toward* the cup?** If yes, lag is the
   mechanism. If no, look elsewhere.
5. **Is it reproducible or OMPL-random?** OMPL-RRTConnect without a seed
   produces a different arc per planning call. Running the same motion N times
   and measuring hit *rate* is the only way to distinguish "fix removed the
   bug" from "fix was lucky this run."

Known patterns from the 2026-04-23 debug session:
- `grasp_home` after a `drop_sweep` to cup_blue can hit cup_blue:
  `shoulder_pan` lags ~20° at ~1.2 rad/s peak velocity, which projects into
  ~100 mm lateral arc deviation — enough to swing through cup_blue envelope.
- `drop_sweep` itself rarely hits cups because its dominant mover is
  `shoulder_lift`, whose lag projects vertically, not laterally.
- **The same grasp_home plan can hit or miss depending on OMPL's random arc**
  — high-lag runs on a wide arc missed, moderate-lag runs on a tight arc hit.
  Hit rate, not single trials, is the relevant metric.

## 7. Arm-link geometry cheat sheet

When `closest_approach_mm` looks safe but cups still get hit, the culprit is
usually arm link collision geometry extending away from TCP:

- `gripper_link` collision mesh extends from z = −104 mm to +1 mm in its local
  frame (via `wrist_roll_follower_so101_v1.stl` rigidly attached). When the
  gripper is tilted by wrist_flex ≠ 90°, that long axis extends *laterally*.
- `wrist_link` has a similar extension via `wrist_roll_pitch_so101_v2.stl`.
- `tcp_link` itself is an *empty frame* — no collision mesh. It's just a fixed
  offset from `gripper_link` (−0.0079, −0.000218, −0.0981 m, rpy (0, π, 0)).

So a "10 mm TCP clear of cup" reading with the gripper tilted 46° from vertical
means the gripper body's far side is *much* closer to the cup than the TCP.
Always correlate `wrist_flex` attitude with TCP proximity.

## 8. Known recurring failure modes

### 8.1 `grasp_move` rejected with "too high: z=0.096m > 0.086m"

`GRASP_WORKSPACE_BOUNDS.Z_MAX = 0.086 m` (per `control_gui.py:362`). A lego
resting on top of another lego can push its graspable Z above the workspace
ceiling. Either randomize scene to resettle or pick a different target lego.
Gate-C workspace bounds are derived in `.planning/phases/09-collision-scene-completeness/compute_grasp_workspace_gate_c.py`.

### 8.2 `drop_sweep` rejected: "cup_drop_N ↔ lego_XXX(d=0.1mm)"

OMPL post-check detects the attached lego's collision envelope overlapping the
cup rim during the sweep arc. Happens with small (0.1–1.5 mm) penetration
distance. Fixes tried:
- Bbox-centering lego STL+USD (done, committed in `98c3142` + `2438e99`)
- Sub-segment post-check at 20 samples/seg (done, committed in `7d862b0`)

Remaining gap: the drop motion is GEOMETRICALLY placing a lego *into* a cup;
the ACM doesn't explicitly allow `attached_lego ↔ cup_drop_*` during drops, so
any plan whose trajectory briefly overlaps the cup envelope at a sub-sample
gets rejected. Long-term fix: add a drop-specific ACM entry at the start of
drop_sweep that's reverted after drop_release. *Not yet implemented.*

### 8.3 `gripper_close_for_object: success=False: motion completed but no _last_motion_status`

The close action dispatched but the GUI's state machine didn't register
completion. Usually benign — the gripper did close. But if the lego's final
position didn't change (check `~/motion_logs/.../*grasp_move*.json` against the
object pose before/after), the physical grasp didn't engage. Common cause:
target lego was at a configuration where the gripper fingers closed on air.
Investigate via MCP: query the gripper joint angle and the target lego's world
pose.

### 8.4 Control GUI node dies silently (segfault, exit code −11)

Symptom: `/so_arm101_control_gui` missing from `ros2 node list` while the
launch process still lives. Seen during aggressive scene-graph rebuilds (e.g.
`stop_scene → delete graph → setup_action_graph → play_scene`) — topic
subscribers go stale and the tkinter GUI segfaults on the next callback.
Recovery: `scripts/restart-control-stack.sh`. Prevention: avoid
stop/delete/setup cycles while the GUI is running; pause the GUI or kill the
control stack first.

### 8.5 `/drop_poses` stale after scene manipulation

`/drop_poses` is published from the Isaac Sim side via the `publish_drop_poses`
MCP tool. It's **not** a live echo of cup positions — it's a snapshot that
needs to be refreshed. GUI's "Update Drops" button calls `drop_refresh` which
updates the GUI's cache, but **does not** reseed the publisher. To reseed the
publisher after a cup reset, call MCP `publish_drop_poses`.

### 8.5b Contact events as deterministic ground-truth (2026-04-23 rewrite)

When debugging "did the robot actually touch the cup" the telemetry CSV now
carries PhysX contact events directly — no more inferring from cup
displacement.

**Mechanism:** `DigitalTwin._setup_contact_sensors` subscribes to
`omni.physx.get_physx_simulation_interface().subscribe_contact_report_events`.
Any contact pair where at least one side has `PhysxContactReportAPI` fires a
callback. We apply that API (with `threshold=0`) on the three cups plus
`gripper_link`, `moving_jaw_so101_v1_link`, `wrist_link`. Events are
resolved to USD paths via `PhysicsSchemaTools.intToSdfPath` on the physics
thread and appended to a bounded deque (`_CONTACT_EVENTS`, maxlen=2048).

**Why not `ContactSensor`?** The `isaacsim.sensors.physics.ContactSensor`
wrapper depends on an `omni.physx.contact` extension that does not load
under that name in Isaac Sim 5.0. Every sensor we created via it returned
`is_valid=False` regardless of USD state, sensor parent (collider vs body),
pre-play creation, or bare vs `ContactSensor(physics_sim_view=...)`
initialization. Bypassing the wrapper and using the PhysX simulation
interface directly sidesteps the failure mode entirely. The three
NVIDIA files that pinned down the diagnosis live (permanent, committed)
in `docs/references/contact-sensor/` with a README explaining each
file's role. Full clones of the source repos sit in `tmp/contact_ref_*/`
— those are `.gitignore`d and re-cloneable on demand.

**CSV schema** (added columns):

| column | meaning |
|---|---|
| `contact_cup_<c>_hit` | 1 if any contact event involving that cup fired during this 20 ms sample, else 0 |
| `contact_cup_<c>_impulse_n` | sum of `impulse_mag` (N·s) across events involving that cup this sample |
| `contact_n_events` | total events drained this sample |
| `contact_events_json` | compact JSON of all events: `[{t, type (FOUND/PERSIST/LOST), a0, a1, c0, c1, impulse [x,y,z], impulse_mag, position, n_contacts}]` |

**Analyzer reconstruction:** `motion_analyze.py` walks the CSV, opens an
event when `contact_cup_<c>_hit` transitions 0→1 and closes it on 1→0;
tracks peak and total impulse plus the union of non-cup bodies seen across
the event. The output reads like:

```
Contact events (from omni.physx contact reports — deterministic):
  cup_blue   t=2.134s → 2.398s (264 ms)  peak_impulse=0.412 N·s  total=3.87 N·s
    colliding: /World/SO_ARM101/moving_jaw_so101_v1_link
```

→ That's the deterministic "which link hit which cup at what time" signal.

**Gotchas:**
- Events use *impulse* (N·s) not *force* (N). Impulse is what physically
  imparts momentum — the correct quantity for "why the cup moved."
- Cup-vs-ground contacts are kept in the stream (cups *are* watched). That's
  useful noise: a cup's ground `CONTACT_LOST` timestamp tells you when it
  left the table, i.e. it got knocked. Filter out cup-vs-ground pairs at
  analysis time if not wanted.
- Hot-reload doesn't rebind the method on the live `DigitalTwin` instance.
  Either restart Isaac Sim after editing `_setup_contact_sensors`, or run
  `importlib.reload(mod)` + rebind manually (see commits).
- Subscription is (re)installed at `quick_start` time. If you observe
  `get_contact_events` returning `subscribed: False`, call `quick_start`
  again — or invoke `_setup_contact_sensors()` directly via
  `execute_python_code`.

**MCP tool:** `get_contact_events(drain=true, max_events=256)` drains the
deque. `motion_logger` calls it implicitly via the snapshot script (it
reads `_CONTACT_EVENTS` directly from the module since snapshot runs
inline in Kit's Python context). External consumers should use the MCP
tool.

### 8.6 Motion logger stops silently

The supervisor (`motion_log_supervisor.sh`) now respawns the logger on any
non-shutdown exit. If logger goes down again, check:
- `tail -30 /tmp/motion_logger.stderr` — supervisor logs its respawn events
- Fast-crash loop? Supervisor backs off exponentially (2s → 4s → 8s → 30s cap)
- Was `stop` called? Supervisor only exits when `/tmp/motion_logger.shutdown`
  flag is set.

## 9. Planner non-determinism

OMPL-RRTConnect without a seed produces a different trajectory per planning
call. Implications for debugging:

- Single-trial results are untrustworthy for confirming fixes.
- "The fix reduced peak lag from 21° to 19.7°" may be planner variance, not
  causal. Measure N≥5 trials before drawing conclusions.
- To make a motion reproducible, set the OMPL seed in moveit_config's
  `ompl_planning.yaml` (or via a one-off parameter). *Not currently set —
  every run is a fresh random draw.*

## 10. Cross-referencing plan dumps and motion logs

Every planned motion writes a dump to `/tmp/arm_traj/YYYYMMDDTHHMMSS_mmm_TAG.json`
with `{joint_names, points[{t, positions}], scene_at_plan_time{...}}`. These
are the **planner's ground truth**. The telemetry rig CSVs are the **physics
ground truth**. The analyzer correlates them via `meta.traj_dump` (the logger
finds the dump written within 2 s of the motion start) — that's how the
Layer-A / Layer-B decomposition works.

If a motion's meta has `traj_dump: null`, the logger didn't find a matching
dump (logger started after the plan, or plan was discarded pre-execution).
The 3-layer decomposition falls back to Layer-B only in that case.

## 11. Scene reset reference

```bash
# Reset cup_blue (or any cup) to nominal, via MCP:
python3 -c "
import socket, json
s = socket.socket(); s.connect(('localhost',8767))
code = '''
from pxr import UsdGeom, Gf
import omni.usd
stage = omni.usd.get_context().get_stage()
for cup, x in [('cup_red', -0.0492), ('cup_green', 0.0388), ('cup_blue', 0.1268)]:
    p = stage.GetPrimAtPath(f'/World/Containers/{cup}')
    xf = UsdGeom.Xformable(p)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == 'xformOp:translate':
            op.Set(Gf.Vec3d(x, -0.28, 0.0))
        elif op.GetOpName() == 'xformOp:orient':
            op.Set(Gf.Quatd(1.0, Gf.Vec3d(0, 0, 0)))
result = 'reset'
'''
s.sendall(json.dumps({'type':'execute_python_code','params':{'code':code}}).encode()+b'\n')
print(s.recv(65536).decode()[:200])
"

# Then reseed drop_poses publisher:
python3 -c "
import socket, json
s = socket.socket(); s.connect(('localhost',8767))
s.sendall(json.dumps({'type':'publish_drop_poses'}).encode()+b'\n')
print(s.recv(65536).decode()[:200])
"
```

Nominal cup positions from CLAUDE.md: red `x=-0.0492`, green `+0.0388`,
blue `+0.1268`, all at `y=-0.28, z=0`.

## 12. What to do when a fix is proposed but not yet measured

Before claiming any fix works:

1. Confirm the logger is capturing: `scripts/motion_log.sh status` → supervisor
   RUNNING; latest CSV timestamp recent.
2. Run the affected motion **at least 5 times** (planner is non-deterministic).
3. For each run, note cup displacement + peak Layer-B lag on the dominant
   joint.
4. Only if the distribution of outcomes clearly moves — not one single run —
   is the fix confirmed.
5. Revert and re-test to confirm the baseline hit rate still matches. If the
   "baseline" changed too, the observed delta is noise.

## 13. Index of fixes explored this session (2026-04-22/23)

| # | Fix | Status | Verdict |
|---|---|---|---|
| #1 | Wire `velocityCommand` → `DriveAPI.targetVelocity` in OmniGraph | wire exists; USD attr not authored by `IsaacArticulationController` | UNCLEAR — runtime path may apply it without authoring USD; unverified |
| #2 | Add `velocity` command interface to ros2_control | APPLIED, uncommitted | velocities now in `/joint_states` but Layer B unchanged |
| #3 | `OnPlaybackTick` → `OnPhysicsStep` in joint-command graph | TRIED, reverted with anti-regression comment | BROKE IT — ROS2 bridge needs render thread |
| #10 | Raise `shoulder_pan.max_force` 3 → 8 Nm | tried, reverted | +7 % lag reduction; not dominant |

Open candidate fixes (none yet tried end-to-end with N≥5):
- Reduce `velocity_scale_var` from 0.5 → 0.2 to cap commanded velocities
- Fix `IsaacArticulationController` to author `targetVelocity` on USD drive
- Add `attached_lego ↔ cup_drop_*` ACM allowance during drops
- Seed OMPL for reproducibility

## 14. How to extend this guide

If you (future agent or human) diagnose something new:

1. Find the most relevant section above (e.g. "known failure modes" or "fixes
   explored").
2. Add a subsection with:
   - **Symptom:** the smallest unambiguous description of what the user/you
     observed.
   - **Detection:** exact commands or log patterns that identify it.
   - **Root cause** (if known) or **hypotheses** (if not).
   - **Fix tried:** reference the commit, the file+line, or "not fixed".
   - **Verdict:** after measurement.
3. If the new knowledge changes the *approach* (new category of bug, new
   interpretation layer in the logger), add a new top-level section and link
   it from § 6 (the diagnosis flow).
4. Keep entries *dated* and reference motion-log filenames when you have them —
   future agents can replay the analyzer against those CSVs and confirm your
   conclusions.

This guide is the project's memory. Anything not written here dies with the
conversation.

---

Last updated: 2026-04-23, active session on debugging cup_blue collision
during `grasp_home` after `drop_sweep`. Status: differential-lag mechanism
confirmed, OMPL non-determinism acknowledged, specific fix still unproven.
PhysX contact-event subscription added as deterministic ground-truth
channel (§ 8.5b) — future runs capture which-link-hit-which-cup directly
from the physics engine instead of inferring from cup displacement.
