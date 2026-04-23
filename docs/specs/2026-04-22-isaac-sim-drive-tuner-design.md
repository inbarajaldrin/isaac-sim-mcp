# Isaac Sim PD Drive Tuner — Design Spec

**Status:** Draft, pre-implementation
**Date:** 2026-04-22
**Context:** Root-caused the SO-ARM101 cup-blue drop_sweep collision bug to Hypothesis (C) — steady-state wrist_flex tracking lag of ~0.9° → ~2.5 mm TCP error against planner margins of ~2–3 mm. Fix Option 1 selected (raise PhysX drive stiffness), but must be measurement-driven, config-driven, and reusable across robots.

---

## 1. Purpose & non-goals

### Purpose

Given an articulated robot running in Isaac Sim under PhysX with position-driven joints, automatically produce **per-joint** PD drive parameters (`stiffness`, `damping`, `max_force`) such that when the robot executes a MoveIt-planned joint trajectory via the ros2_control controller, the actual joint positions track the commanded reference within a physically-meaningful margin derived from the MoveIt collision-padding setting.

The tuner is principled (analytical, not random guess-and-check), produces a full audit trail of its decisions, captures comprehensive telemetry for post-hoc analysis, and is invokable from the command line with one flag per robot.

### Non-goals

- Tuning the ros2_control low-level PID (different layer; we treat the controller's output as ground-truth reference at the DriveAPI input).
- Tuning non-robot rigid bodies (legos, cups, ground — those have their own damping settings already).
- Compensating for modeling errors outside the drive (mass/inertia mismatches, joint friction, backlash). We log the signatures but don't fix them.
- Replacing NVIDIA's built-in `isaacsim.robot_setup.gain_tuner` — we *leverage* it for synthetic per-joint sweeps and inertia queries. We own the *analytical recommendation logic*, the *task-trajectory verification layer*, the *config pipeline*, and the *reporting*.
- Auto-discovering robots we've never seen. Config-driven per-robot stanzas. Adding SO-ARM102 later = new YAML stanza.

---

## 2. Big-picture architecture

```
                              ┌─────────────────────────────────────────────┐
                              │  scripts/tune_drives.py    (standalone CLI) │
                              │                                             │
                              │  ┌──────────────┐   ┌──────────────────┐    │
                              │  │ config/      │   │ /tmp/arm_traj/   │    │
                              │  │ drive_tuning │   │    *.json        │    │
                              │  │    .yaml     │   │  (ref trajs)     │    │
                              │  └──────┬───────┘   └──────┬───────────┘    │
                              │         │                  │                │
                              │         ▼                  ▼                │
                              │  ┌──────────────────────────────────┐       │
                              │  │ Orchestrator                     │       │
                              │  │  - load config                   │       │
                              │  │  - discover robots               │       │
                              │  │  - iterate tuning loop           │       │
                              │  │  - apply analytical formula      │       │
                              │  │  - write YAML + reports          │       │
                              │  └─────┬────────────────────┬───────┘       │
                              │        │                    │               │
                              └────────┼────────────────────┼───────────────┘
                                       │                    │
                 ┌─────────────────────┘                    └──────────────────────┐
                 │ MCP localhost:8767                                     ROS2     │
                 ▼                                                                 ▼
       ┌─────────────────────────┐                                      ┌───────────────────────┐
       │ exts/soarm101-dt/       │                                      │ /arm_controller/      │
       │   extension.py          │                                      │   follow_joint_traj.. │
       │                         │                                      │                       │
       │  MCP_TOOL_REGISTRY      │                                      │ /arm_controller/      │
       │   + read_drive_params   │                                      │   controller_state    │
       │   + apply_drive_params  │                                      │                       │
       │   + run_gain_test       │                                      │ /joint_states         │
       │   + probe_joint_state   │                                      │                       │
       │   + compute_inertias    │                                      │ /tf (for TCP FK)      │
       │   + restore_scene_state │                                      │                       │
       │   + dump_physx_joints   │                                      │ /move_group           │
       │                         │                                      │  parameters           │
       │  (YAML-driven drive     │                                      │                       │
       │   setup at scene init)  │                                      │                       │
       └───────────┬─────────────┘                                      └───────────┬───────────┘
                   │                                                                │
                   ▼                                                                ▼
       ┌──────────────────────────┐                                     ┌──────────────────────┐
       │ PhysX articulation       │                                     │ ros2_control         │
       │  UsdPhysics.DriveAPI     │                                     │  arm_controller      │
       │  per joint               │                                     │  (JointTrajectoryCtrl)│
       └──────────────────────────┘                                     └──────────────────────┘
```

Two boundaries, one role each:

- **MCP channel** → authoritative physics: read/write DriveAPI, query per-joint inertia, probe articulation state, restore scene state for deterministic replay.
- **ROS2 channel** → trajectory execution + reference/feedback telemetry from the same pipeline the real robot uses.

The tuner is pure orchestration; all robot-specific logic lives in the YAML stanza and (optionally) in the extension's MCP tool registration.

---

## 3. Core algorithm

### 3.1 Reference & target

The **reference trajectory** is a planned-trajectory dump from `/tmp/arm_traj/*.json` (existing infrastructure). Each dump already contains:

- `joint_names` — ordered list
- `waypoints[*]` — `{positions, velocities, accelerations, time_from_start}` per the MoveIt time-parameterization (TOTG / IPTP)
- `scene_at_plan_time` — object poses, attached bodies, allowed_collision_matrix (used for deterministic scene restore)
- `actual_joints` — the joint state at plan start
- `motion_tag` — e.g. `drop_sweep`, `grasp_move`, etc.
- `wall_timestamp` and `sim_time_at_plan`

The **target error threshold** is derived, not assumed:

```
τ_TCP = moveit_link_padding × safety_factor
```

Where `moveit_link_padding` is read live from the `/move_group` parameter `default_robot_link_padding` at startup (typically 0.003 m). `safety_factor` is a YAML value (default 0.66). The tuner targets `peak_tcp_error ≤ τ_TCP`.

Per-joint targets are derived by inverse-Jacobian projection at the high-risk waypoints — "what's the angular error budget on joint *j* such that TCP error stays ≤ τ_TCP when *j* is the dominant contributor?" Falls back to a scalar per-joint threshold `ε_j_rad` if Jacobian projection is disabled.

### 3.2 Single-iteration measurement

```
1. Restore scene to ref.scene_at_plan_time            (MCP restore_scene_state)
2. Set robot to ref.actual_joints as start state       (direct DC teleport + short settle)
3. Record t0_wall and t0_sim                          (probe_joint_state returns both)
4. Send FollowJointTrajectory(ref.waypoints)          (ROS2 action)
5. Concurrently subscribe at ~100 Hz to:
     /arm_controller/controller_state    → (t, ref_pos[], fb_pos[], error[], ref_vel[], fb_vel[])
     /joint_states                       → ground-truth physics state
     /tf                                 → tcp_link pose in base (for TCP FK cross-check)
6. Concurrently poll MCP probe_joint_state at ~60 Hz for:
     (applied_torque[], instantaneous_inertia[], joint_force_sensor_values[], contact_count)
7. Wait for action result
8. Record t1_wall and t1_sim
9. Compute per-joint derived metrics (§ 3.4)
10. Persist all streams to reports/<session>/replay_iterN/*.csv
```

The 60 Hz physics probe is critical instrumentation the user asked for — without it, we can't distinguish "drive is working but saturated" from "drive is working perfectly but noise masks it".

### 3.3 Analytical formula

For a PD-controlled joint with effective inertia `J`, stiffness `K`, damping `B`, tracking position reference `q_ref(t)` with velocity `v_ref(t)` and acceleration `a_ref(t)`:

```
τ_applied(t) = K · (q_ref - q_actual)  +  B · (v_ref - v_actual)
            (approximately, per PhysX position-drive model)

Steady-state (ideal tracking, constant v):
    q_ref - q_actual ≈ (B·v) / K                    ← position lag
    required torque ≈ J·a + B·v + K·e               ← feasibility
```

For a target position error `ε_target` at peak velocity `v_peak`, with critical damping `B = 2·√(K·J)`:

```
ε_target = (2·√(K·J) · v_peak) / K = 2·v_peak·√(J/K)
        ⇒ K = J · (2·v_peak / ε_target)²
        ⇒ B = 2·√(K·J)
```

Feasibility check:

```
τ_peak = J·a_peak + B·v_peak + K·ε_target
      ≤ F_j   (drive max_force limit)
```

If `τ_peak > F_j`, two remediation paths:
- Raise `F_j` (motor is undersized) — if YAML allows (`max_force_auto: true`), bump by 25% and retry.
- Otherwise flag "saturated" and suggest slowing the trajectory (lower `v_peak`) or use a more tolerant `ε_target`.

### 3.4 Derived metrics (per joint, per iteration)

Computed from the streams captured in § 3.2:

- `peak_err_rad`, `peak_err_time_s`, `peak_err_q_ref_at_peak`
- `rms_err_rad` (whole trajectory)
- `rms_err_at_peak_velocity_rad` (RMS only during top 25% of velocity magnitude — isolates the hardest-to-track region)
- `peak_v_rad_s`, `peak_a_rad_s²`
- `velocity_integrated_lag` (∫|error|·|velocity| dt — relates joint error to spatial travel)
- `overshoot_rad` (if ref ends flat and fb ≠ ref at t_end)
- `settling_time_ms` (time from last waypoint to `|err| < ε_target` for remainder)
- `ringing_frequency_hz` (FFT on err post-last-waypoint; identifies under-damping)
- `saturation_ticks` (count of control ticks where `|τ_applied| ≥ F_j · 0.98`)
- `torque_headroom_pct` (F_j relative to τ_peak_measured)
- `effective_inertia_J_kgm2` (queried once per session, cached)
- `phase_lag_deg` at fundamental (for sinusoidal-like sub-segments; optional)
- `q_fb_minus_q_js_rmse` (controller_state.feedback vs /joint_states — sanity check; should be ~0)

### 3.5 Iteration loop (per session)

```
load config, discover robots
for robot in config.robots:
    J_map = MCP.compute_inertias(robot)          # one-shot, cached
    padding = read_moveit_padding()
    τ_TCP = padding * robot.safety_factor
    ε_map = derive_per_joint_targets(J_map, ref_trajs, τ_TCP)   # Jacobian projection

    for iter in range(max_iters):
        metrics = run_replay(ref_trajs, robot)    # § 3.2
        violating = [j for j in joints if metrics[j].peak_err > ε_map[j]]
        if not violating and tcp_verify(metrics) passes:
            VERDICT = converged; break
        for j in violating:
            K_new, B_new = analytical_recompute(J_map[j], metrics[j], ε_map[j])
            check_saturation(K_new, B_new, F_map[j], metrics[j])
            apply[j] = (K_new, B_new, F_map[j])
        if apply == last_apply: VERDICT = stalled; break
        MCP.apply_drive_params(apply)
        last_apply = apply
    else:
        VERDICT = timeout

    task_verification_pass(ref_trajs, τ_TCP)      # TCP FK check across all refs
    write_report(robot, session)
    maybe_write_yaml_update(robot)                # unless --dry-run
```

### 3.6 Multi-trajectory tuning

`--ref` accepts a glob, a list, or a directory. If multiple trajectories are given, the tuning target for each joint is:

```
ε_target_j = min(ε_j from target) across all refs
```

I.e. we tune against the worst-case-demanding ref, then verify all refs meet the threshold. Prevents overfitting to one trajectory.

### 3.7 Determinism check

Because we restore `scene_at_plan_time` before each replay, running the *same* drive params twice should yield *identical* error curves (modulo physics-thread scheduling jitter, which should be ≤ 1 %). If two replays with identical K, B, F diverge by > 10 % on any joint's `peak_err`, the tuner logs a **non-determinism event**. This is never a reason to abort — it's a separate signal surfaced in the report. Typical causes: fabric/async write races in Isaac Sim, USD notice-handler reorderings, or CUDA nondeterminism on solver kernels.

---

## 4. Component specifications

### 4.1 `config/drive_tuning.yaml`

```yaml
# Reference: MoveIt default_robot_link_padding, PhysX UsdPhysics.DriveAPI,
#            NVIDIA isaacsim.robot_setup.gain_tuner
version: 1
defaults:
  safety_factor: 0.66
  max_iters: 8
  no_improvement_stop_after: 2
  determinism_tolerance_pct: 10.0
  plot: true
  jacobian_projection: true
  probe_rate_hz: 60
  replay_telemetry_rate_hz: 100
  max_force_auto: false          # if true, tuner can bump max_force when saturated
  max_force_auto_step_pct: 25.0
  settle_ms_before_replay: 250

robots:
  so_arm101:
    articulation_root: /World/SO_ARM101/base_link
    mcp_port: 8767
    trajectory_action: /arm_controller/follow_joint_trajectory
    controller_state_topic: /arm_controller/controller_state
    joint_states_topic: /joint_states
    moveit_padding_param: /move_group:default_robot_link_padding
    moveit_padding_fallback_m: 0.003
    tf_base_frame: base
    tf_tcp_frame: tcp_link
    home_pose_rad:
      shoulder_pan: 0.0
      shoulder_lift: 0.0
      elbow_flex: 0.0
      wrist_flex: 1.5707963
      wrist_roll: 0.0
      gripper_joint: 0.0
    joints:
      shoulder_pan:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: [stiffness, damping]
      shoulder_lift:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: [stiffness, damping]
      elbow_flex:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: [stiffness, damping]
      wrist_flex:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: [stiffness, damping]
      wrist_roll:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: [stiffness, damping]
      gripper_joint:
        stiffness: 15.0
        damping: 0.15
        max_force: 3.0
        tunable: []    # don't tune gripper via this pipeline
    reference_trajectories:
      default: "/tmp/arm_traj/*drop_sweep*.json"
      grasp:   "/tmp/arm_traj/*grasp_move*.json"
      home:    "/tmp/arm_traj/*grasp_home*.json"
    # Optional per-joint overrides
    overrides:
      # wrist_flex: { safety_factor: 0.5 }       # tighter margin on this joint
```

### 4.2 `scripts/tune_drives.py`

Single-file orchestrator; uses only stdlib + numpy + scipy + ruamel.yaml + rclpy + socket (for MCP).

Module structure (same file, top-down):

1. CLI / argparse (§ 5)
2. Config loader (ruamel.yaml — preserves comments on write)
3. MCP client wrapper (thin dict-over-socket helper, reuses the pattern from existing MCP calls)
4. ROS2 client wrapper (rclpy Node subclass: action client, topic subscribers, TF listener, param client for moveit padding)
5. Scene restore + start-state teleport
6. Replay recorder (the 100 Hz + 60 Hz concurrent streams)
7. Metric extractor (§ 3.4)
8. Jacobian projection (pinocchio if available; URDF-parsed ad-hoc FK fallback using the same code as Test C)
9. Analytical recompute (§ 3.3)
10. Iteration loop (§ 3.5)
11. Task verification pass (TCP FK error vs τ_TCP)
12. Report writer (§ 7)
13. YAML updater (with backup)

### 4.3 Extension additions (`exts/soarm101-dt/so_arm101_dt/extension.py`)

#### New MCP tools

| Tool | Purpose |
|------|---------|
| `read_drive_params` | Return current DriveAPI values per joint as `{name: {stiffness, damping, max_force}}`. |
| `apply_drive_params` | Apply new values via `UsdPhysics.DriveAPI.Set*`. Return the live values after set (round-trip confirm) plus `applied_via: "live" \| "reset"` depending on whether articulation needed a timeline stop/start. |
| `run_gain_test` | Programmatic wrapper over NVIDIA `GainTuner.initialize_gains_test` → `update_gains_test` loop. Params: `joint_name, mode: "step"|"sinusoidal", duration_s, amplitude_rad, frequency_hz?`. Returns `{pos_rmse, vel_rmse, effective_inertia, sampled_positions, sampled_velocities, timestamps}`. |
| `probe_joint_state` | Single-shot per-joint physics probe: `{q, qd, qdd, applied_torque, reaction_force, reaction_torque, wall_time_ns, sim_time_ns}`. The backbone of the 60 Hz probe stream. |
| `compute_inertias` | Wraps `GainTuner.compute_joints_acumulated_inertia()`. Returns `{joint_name: J_kgm2}` once per session. |
| `restore_scene_state` | Existing feature extended: takes a serialized `scene_at_plan_time` blob (or a path to the dump file), sets rigid-body poses + velocities, zeros physics residuals. Already partially exists via `save_scene_state` / `restore_scene_state`; extend to accept our trajectory-dump blob format. |
| `dump_physx_joints` | One-shot dump of all joints' current articulation state: `{name, type, position, velocity, acceleration, applied_torque, drive_type, stiffness, damping, max_force, limits}`. Used at start/end of tuning sessions as a pre/post snapshot. |

All tools register via `MCP_TOOL_REGISTRY` at the top of `extension.py` and follow the existing dict-in / dict-out protocol on `localhost:8767`.

#### YAML-driven drive setup

Replace the hardcoded `self._robot_stiffness = 15.0` (lines 820–827) with:

```python
self._drive_config_path = os.path.join(ext_root, "..", "..", "..", "config", "drive_tuning.yaml")
self._drive_config = self._load_drive_config()   # dict or None

# When applying DriveAPI to each joint prim:
joint_cfg = self._drive_config_for_joint(joint_name)   # dict with stiffness/damping/max_force
# fallback to 15/0.15/3 if config missing or joint absent
drive_api.GetStiffnessAttr().Set(joint_cfg.get("stiffness", 15.0))
drive_api.GetDampingAttr().Set(joint_cfg.get("damping", 0.15))
drive_api.GetMaxForceAttr().Set(joint_cfg.get("max_force", 3.0))
```

Missing YAML = current behavior preserved. Missing joint in YAML = fallback with a warning log. `_drive_config` is cached but re-read on each `quick_start` / `load_robot` to pick up edits without an extension reload.

### 4.4 Live drive updates

Before wiring the full tuner: a probe test (hand-run once, result captured in the design log) verifies that `UsdPhysics.DriveAPI.GetStiffnessAttr().Set()` during play is reflected in the next physics step without an articulation reset. If it isn't, `apply_drive_params` internally does:

```
timeline.stop()
for _ in range(20): app.update()
<apply DriveAPI sets>
timeline.play()
for _ in range(5): app.update()
```

and returns `applied_via: "reset"`. Either way the tuner script doesn't care — it just sees a confirmed round-trip.

---

## 5. CLI

```
scripts/tune_drives.py [--robot <name>]
                       [--ref <glob|path|list>]
                       [--target-tcp-err-mm <float>]
                       [--safety-factor <float>]
                       [--max-iters <int>]
                       [--report-dir <path>]
                       [--yaml <path>]
                       [--dry-run]
                       [--verify]
                       [--isolate <joint>]
                       [--sinusoid-amplitude <rad>] [--sinusoid-freq <hz>]
                       [--no-live-apply]
                       [--no-plots]
                       [--no-jacobian-projection]
                       [--probe-rate <hz>]
                       [--replay-telemetry-rate <hz>]
                       [--determinism-check <N>]    # run N replays per iter with same K,B to characterize noise
                       [--json-log]                 # stdout as JSON lines instead of human-readable
                       [-v|--verbose] [-q|--quiet]
```

Exit codes:

- `0` converged within target on all refs
- `1` config/IO error
- `2` stalled (no improvement ≥ N iters)
- `3` infeasible (motor saturation unresolvable)
- `4` max_iters exceeded
- `5` `--verify` failed
- `6` non-determinism above threshold on > 1 joint

---

## 6. "Extract as much information as possible" — telemetry specification

**Design principle:** this is a one-time build; over-capture now, prune later. The storage cost of a full session is small (~20 MB per robot per run), but the analytical cost of re-running after realizing we didn't log X is large.

### 6.1 Streams captured every session

| Stream | Source | Rate | Columns |
|--------|--------|------|---------|
| `replay_iter<N>.csv` | `/arm_controller/controller_state` | 100 Hz | `t_wall_ns, t_sim_ns, iter, joint, ref_pos, fb_pos, err, ref_vel, fb_vel, ref_acc` |
| `joint_states_iter<N>.csv` | `/joint_states` | topic rate | `t_wall_ns, joint, position, velocity, effort` |
| `physx_probe_iter<N>.csv` | MCP `probe_joint_state` | 60 Hz | `t_wall_ns, t_sim_ns, joint, q, qd, qdd, applied_torque, reaction_force_xyz, reaction_torque_xyz, contact_count` |
| `tf_iter<N>.csv` | `/tf` `base→tcp_link` | topic rate | `t_wall_ns, x, y, z, qx, qy, qz, qw` |
| `tcp_fk_iter<N>.csv` | computed (ref vs fb via FK) | 100 Hz | `t_wall_ns, ref_x, ref_y, ref_z, fb_x_from_tf, fb_x_from_fk, dx_mm, dy_mm, dz_mm, d_mm, tcp_err_exceeds_target` |
| `scene_probe_iter<N>.json` | MCP `dump_physx_joints` at start & end | 2 | full articulation snapshot |
| `actions_iter<N>.csv` | controller goal + result | per-iter | `iter, t_wall_sent, t_wall_result, error_code, error_string, duration_s, n_waypoints` |

### 6.2 Derived tables

- `iterations.csv` — one row per (joint, iter): K_before, B_before, F_before, J_eff, peak_err, peak_err_time, rms_err, rms_err_at_peak_v, peak_v, peak_a, velocity_integrated_lag, overshoot, settling_time_ms, ringing_freq_hz, saturation_ticks, torque_headroom_pct, K_after, B_after, F_after, action_rationale (string), action_tag (e.g. `analytical_recompute`, `clamped`, `saturation_detected`, `no_change_needed`)
- `session.json` — top-level: config path, ref list, per-robot final verdict + all iteration rationales, moveit_padding read, τ_TCP computed, environment snapshot (Isaac Sim version, ros2 distro, physx build, kernel version, GPU, CUDA driver — snapped from `uname -a`, `nvidia-smi --query-gpu=driver_version`, Isaac Sim build info)
- `spectra.csv` — FFT of the post-trajectory error tail per joint per iter; columns: iter, joint, freq_hz, amplitude_rad, phase_rad
- `determinism.csv` (only if `--determinism-check > 1`) — per (joint, iter, replay_k): peak_err values, cross-replay std

### 6.3 Plots (default on; `--no-plots` to skip)

- `plots/joint_<j>_iter_<n>_tracking.png` — ref and fb overlay for this joint, this iter, with shaded error band
- `plots/joint_<j>_iter_<n>_error.png` — error-vs-time, with ε_target line
- `plots/joint_<j>_convergence.png` — K, B, peak_err across iterations
- `plots/tcp_error_iter_<n>.png` — |TCP error| vs time with τ_TCP line
- `plots/heatmap_saturation.png` — joint × iter matrix of saturation_ticks
- `plots/phase_diagram_joint_<j>.png` — fb_pos vs fb_vel, colored by iter (shows over/under-damping as ellipse shape)
- `plots/summary_tile.png` — 2×3 tile of the most-informative plots for at-a-glance review

All plots use a fixed color scheme (matplotlib `tab10`), consistent axis labels, and the y-axes are annotated with `ε_target`, `π/2`, etc.

### 6.4 Report outputs

Per session, `reports/tuning_<timestamp>/`:

```
session.json
environment.json
config_snapshot.yaml
moveit_params.json
iterations.csv
spectra.csv
determinism.csv
replay_iter0.csv     physx_probe_iter0.csv    joint_states_iter0.csv    tf_iter0.csv    tcp_fk_iter0.csv    scene_probe_iter0.json    actions_iter0.csv
replay_iter1.csv     ...                      ...                       ...             ...                 ...                       ...
...
plots/
  (as in § 6.3)
summary.txt          # human-readable one-page result
summary.md           # same info, markdown, with inlined plots for browser preview
session.log          # full debug log at -v
```

### 6.5 One-page `summary.txt` shape

```
Isaac Sim PD Drive Tuner — Session 2026-04-22T22:15:03Z
Robot: so_arm101
Refs : /tmp/arm_traj/*drop_sweep*.json   (3 trajectories)
MoveIt padding: 0.00300 m   →  τ_TCP = 0.00198 m (safety 0.66)

Verdict: CONVERGED in 4 iterations (7m 12s wall)

Per-joint:
  joint         K_before → K_after   B_before → B_after   peak_err_before  peak_err_after   status
  shoulder_pan     15 →   15            0.15 →   0.15         0.12°            0.12°         no-change
  shoulder_lift    15 →   15            0.15 →   0.15         0.08°            0.08°         no-change
  elbow_flex       15 →   15            0.15 →   0.15         0.11°            0.11°         no-change
  wrist_flex       15 →   180           0.15 →   1.31         0.904°           0.063°        converged
  wrist_roll       15 →   15            0.15 →   0.15         0.04°            0.04°         no-change
  gripper_joint    15 →   15            0.15 →   0.15          —                —            not-tunable

TCP verification (3 refs): peak 0.62 mm / 0.44 mm / 0.71 mm  (all < 1.98 mm) ✓

Saturation events: 0
Non-determinism events: 0
Plots: reports/tuning_<ts>/plots/
Updated config: config/drive_tuning.yaml  (backup: config/drive_tuning.yaml.bak.<ts>)
```

---

## 7. Error handling matrix

| Failure | Detection | Response | Exit |
|---------|-----------|----------|------|
| MCP socket unreachable | First `read_drive_params` call times out | Print `isaacsim_launch.sh launch` hint; abort | 1 |
| ROS2 action server unreachable | `wait_for_server` times out at 10 s | Print `scripts/restart-control-stack.sh` hint; abort | 1 |
| Ref trajectory has no `scene_at_plan_time` | Parser | Warn, continue without restore; flag "determinism not guaranteed" | — |
| Ref uses joint names not in YAML | Parser | Abort, list missing joints | 1 |
| Measured velocity exceeds URDF limit during replay | Post-replay check | Refuse: ref is out of spec; abort | 1 |
| Drive set failed (live or reset) | `apply_drive_params` round-trip diff | Retry once; if second fail, abort | 1 |
| Motor saturated at τ_peak > F_j | `check_saturation` in § 3.3 | `max_force_auto` → bump F_j by 25% and retry; else flag and skip joint | 3 at end |
| Non-deterministic replay (>10% variance in peak_err at fixed K,B,F) | Determinism check (if enabled) | Log, continue; exit 6 at end | 6 |
| YAML write fails mid-write | Atomic write: temp + rename | Backup preserved; original unchanged | 1 |
| `peak_err` diverges iter-over-iter | Stall detector | Revert to prior iter's values, flag | 2 |
| All refs produced identical plans (user didn't run new data) | File mtime spread | Warn: "all refs planned within 10 min, consider running fresh data" | — |
| Isaac Sim timeline paused mid-session | Detected when no sim_time advance in 2 s | Attempt to resume via MCP `play_scene`; abort if fails | 1 |
| GPU memory pressure / physics step stall | Wall-time between probes exceeds 500 ms | Flag "physics stall"; don't use that iter's data for analytical recompute | — |

---

## 8. Testing strategy

### 8.1 Unit (fast, no sim required)

- `test_analytical_formula.py` — hand-computed cases: for J=0.002, v=0.8, ε=0.005 → K should be 0.002·(2·0.8/0.005)² ≈ 205; exact.
- `test_yaml_roundtrip.py` — ruamel.yaml preserves comments + ordering + a specific stanza mutation path
- `test_ref_parser.py` — feeds a golden `/tmp/arm_traj/*.json` sample, confirms joint ordering, velocity extraction
- `test_jacobian_projection.py` — URDF-driven FK chain vs pinocchio reference at 10 random joint configs
- `test_metric_extraction.py` — synthetic controller_state stream with known lag → expects correct `peak_err`, `rms_err`, `rms_err_at_peak_v`

### 8.2 Integration (live sim, slow)

- **`test_mcp_tools.py`** — for each new MCP tool, round-trips a representative payload, asserts schema
- **`test_replay_determinism.py`** — runs the same ref trajectory twice with identical K/B; asserts `peak_err` std < 1 %
- **`test_live_apply.py`** — confirms `apply_drive_params` during timeline play changes physics step behavior in the next observed tick
- **`test_dry_run.py`** — `--dry-run` against live sim with one ref, asserts: (a) report dir created with expected schema, (b) `config/drive_tuning.yaml` unchanged, (c) `config/drive_tuning.tuned.yaml` written

### 8.3 End-to-end (regression)

- **`test_wrist_flex_bug_fix.py`** — replays the Test C trajectory; asserts:
  - wrist_flex K raised above 15
  - wrist_flex peak_err < ε_target
  - TCP peak error on all 3 ref trajectories < τ_TCP
  - saturation_ticks == 0 on all joints

### 8.4 Verify mode in CI

```bash
scripts/tune_drives.py --robot so_arm101 --verify --ref "/tmp/arm_traj/*drop_sweep*.json"
```

Runs no tuning; just replays + measures. Exit 5 if any ref fails τ_TCP. Wire into CI so someone changing the extension and breaking drives fails the build.

---

## 9. Rollout & backwards compatibility

1. Extension code changes: YAML-reading drive setup + new MCP tools. **If YAML missing or unreadable**, extension falls back to hardcoded 15/0.15/3 — no behavior change from today.
2. First commit of `config/drive_tuning.yaml` ships with the exact current values (15/0.15/3 for all joints). Running the sim unchanged produces identical physics.
3. First tuning run (recommended against `/tmp/arm_traj/*drop_sweep*.json` bundle) produces updated YAML + report; user diffs YAML, commits if satisfied.
4. Subsequent runs: `scripts/tune_drives.py --verify` in CI catches regressions.

No rollback needed beyond reverting a single YAML commit.

---

## 10. Open questions / explicitly deferred

- **Payload-aware tuning.** Effective inertia changes when a lego is grasped. Two options: (a) re-tune with attached body; (b) add `--carry-mode` pass that uses `ref trajectories containing attached objects in scene_at_plan_time`. Deferred until empirically needed (measure the gap first with/without payload; we can run the tuner in both configs with zero extra code).
- **Joint friction / backlash modeling.** If steady-state error is non-linear in velocity (not matching `(B·v)/K`), it's likely not a drive-tuning issue. The tuner logs this signature; fixing requires a separate effort.
- **Non-position drives.** Velocity-drive and torque-drive joints: our formula is position-drive-specific. Config schema supports `drive_type: position|velocity|torque` but analytical path is position-only for now. NVIDIA `GainTuner` already handles both; adding a velocity-drive code path is ~50 LOC when needed.
- **Simultaneous multi-joint cross-talk.** The per-joint analytical formula assumes decoupled dynamics. In practice, the articulation mass matrix is not diagonal (M[i,j] ≠ 0 for i ≠ j). NVIDIA's `compute_joints_acumulated_inertia` returns the *diagonal effective inertia* which folds the coupling into `J_eff`. For strongly coupled joints (e.g., shoulder_pan + shoulder_lift on a typical arm), this might be insufficient. We log per-joint coupling strength (off-diagonal / on-diagonal mass matrix ratio) so future us can tell if decoupled tuning is losing accuracy.
- **Multi-robot articulations.** Current design assumes one robot per sim. Multi-arm setups (bimanual, multi-arm cells) would need a per-articulation loop in the extension setup and per-articulation stanzas in YAML. Schema supports it (`robots:` is a map). Extension code is single-robot today; generalizing is straightforward.
- **Parametric sweep mode.** Sometimes you want a *survey* (what's the tracking error at K=30, 60, 120 with B at critical each?) instead of convergence to a single optimum. Add `--sweep K=30,60,120,240` mode. Deferred; one-shot mode covers the current need.

---

## 11. File layout delta on repo

```
isaac-sim-mcp/
├── config/
│   └── drive_tuning.yaml                                  NEW  (~50 lines)
├── docs/
│   └── specs/
│       └── 2026-04-22-isaac-sim-drive-tuner-design.md     NEW  (this file)
├── exts/soarm101-dt/so_arm101_dt/
│   └── extension.py                                       MODIFY  (+~150 LOC for MCP tools + YAML loader)
├── reports/
│   └── tuning_<timestamp>/                                NEW per run, gitignored
├── scripts/
│   ├── tune_drives.py                                     NEW  (~800 LOC with full telemetry)
│   └── tune_drives_verify.sh                              NEW  (thin wrapper for CI)
└── tests/
    └── drive_tuner/                                       NEW
        ├── test_analytical_formula.py
        ├── test_yaml_roundtrip.py
        ├── test_ref_parser.py
        ├── test_jacobian_projection.py
        ├── test_metric_extraction.py
        ├── test_mcp_tools.py
        ├── test_replay_determinism.py
        ├── test_live_apply.py
        ├── test_dry_run.py
        └── test_wrist_flex_bug_fix.py
```

`.gitignore` additions:
- `reports/tuning_*/`
- `config/drive_tuning.yaml.bak.*`
- `config/drive_tuning.tuned.yaml`

---

## 12. Summary of design choices (rationale log)

| Choice | Alternative | Why chosen |
|--------|-------------|------------|
| Task trajectory as reference | Synthetic sinusoid-only | Task is the real workload; synthetic becomes diagnostic-only. User insight: planner output is the correct ground truth. |
| Analytical K/B recompute | Gradient-free optimizer (CMA-ES/NM) | Explainable ("K=180 because J=0.002, v=0.8, ε=0.005"); auditable config changes; ~5x fewer sim runs. |
| Per-joint config schema | Single global scalar (current) | Bug is joint-specific (wrist_flex); one scalar hides the problem. |
| Standalone CLI script + MCP helpers | MCP tool only / ROS node only | Script portable + CI-friendly; helpers keep drive I/O efficient. |
| Auto-update YAML with backup | Report-only | User explicitly wants "not temporary"; auto-update eliminates "forgot to commit" failure mode. Backup + dry-run preserve safety. |
| Threshold from MoveIt padding | Hard-coded mm number | Physically meaningful; same threshold the planner already uses; no magic numbers. |
| Leverage NVIDIA gain_tuner | Build from scratch | Already computes J per joint; synthetic signal gen + error metrics free; isolates drive from ros2_control. |
| Full per-tick capture | Summary stats only | User explicit: "extract as much information"; one-time-build context; ≤ 20 MB per session. |
| Live drive apply (with reset fallback) | Always-reset every iter | Faster iteration; fallback path preserves correctness if PhysX doesn't re-pick live. |
| Scene restore per replay | Start state teleport only | Determinism check becomes meaningful; removes a noise source from measurements. |
| Critical damping analytical | Separate B search | For clean articulation with no non-linearities, critical damping is correct; one less free parameter to tune. |

---

## 13. Definition of done

- [ ] Extension reads `config/drive_tuning.yaml` at setup; falls back to hardcoded values if missing
- [ ] All 7 MCP tools registered, round-trip tested against live sim
- [ ] `scripts/tune_drives.py --verify --ref "/tmp/arm_traj/*drop_sweep*.json"` runs without modifying anything, produces full report
- [ ] Full tune run on the Test C trajectory drops wrist_flex peak TCP error below τ_TCP (from 2.51 mm measured)
- [ ] YAML auto-update + backup path exercised on a dry-run + real run
- [ ] All unit tests green; integration tests green against a running sim
- [ ] `reports/tuning_<ts>/summary.md` renders readable in a markdown viewer
- [ ] `scripts/tune_drives.py --help` shows all flags with defaults
- [ ] `.gitignore` updated
- [ ] Works on a fresh Isaac Sim restart (i.e., no latent state from this session's manual poking)
