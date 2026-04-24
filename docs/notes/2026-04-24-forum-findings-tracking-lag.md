# Forum Findings — SO-ARM101 Tracking-Lag Problem

**Session date:** 2026-04-24
**Status:** Working notes, living document — extend as better info surfaces
**Context:** SO-ARM101 wrist_flex has measured ~0.9° steady-state tracking lag → ~2.5 mm TCP error against MoveIt 3 mm collision padding. Before committing to the analytical drive tuner in `docs/specs/2026-04-22-isaac-sim-drive-tuner-design.md`, traversed NVIDIA Developer Forums to see what others have tried.

**Methodology:** 4 keyword searches (depth 0) → 10 threads (depth 1 via `related_topics[]`) → 5 threads (depth 2 via star node) → full-content reads of all 15 threads (no truncation). Total: 15 threads read in full.

---

## The unifying observation

**Official NVIDIA assets (Franka, UR10) don't reproduce the bug. URDF-imported robots do.**

Reported by JeanLesur in [#328322 post #9](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322), confirmed by zhengwang (NVIDIA) in post #22 ("I followed the tutorial... joints are following command pretty well").

This points the root cause at **the URDF importer's output** (drive values, effort/scale, joint limits, mass/inertia), not at the ROS2 bridge, OmniGraph, or PhysX solver. The expensive analytical tuner may be fixing the wrong layer.

---

## Confirmed fixes (user-verified across multiple threads)

| Fix | Who confirmed | Source | Notes |
|-----|---------------|--------|-------|
| **URDF xacro joint limits match USD joint limits** | radhen.17 (2 threads) | [#269397 #7](https://forums.developer.nvidia.com/t/arm-freezes-while-executing-motion-plan-generated-via-moveit/269397), [#312476 #3](https://forums.developer.nvidia.com/t/inconsistent-trajectory-following-in-isaac-sim-with-moveit-using-topic-based-ros2-control/312476) | "after setting the right joint limits values, I was not seeing an issue" |
| **Re-import the URDF** | swimpark | [#240913 #5](https://forums.developer.nvidia.com/t/the-joint-gap-btw-ros-and-sim/240913) | Suspected Effort/Scale mismatch in URDF |
| **Bypass OmniGraph, use rclpy + articulation controller from physics callback** (cuRobo example pattern) | daohui.liu | [#312476 #6](https://forums.developer.nvidia.com/t/inconsistent-trajectory-following-in-isaac-sim-with-moveit-using-topic-based-ros2-control/312476) | "dozens of tests, no problems" — **but contradicts blueed96 #363351 #4** |
| **Increase damping parameter** | lee43 | [#349267 #4](https://forums.developer.nvidia.com/t/robot-arm-is-shaking-during-pose-tracking/349267) | Cartesian controller context (not MoveIt) |
| **Physics rate + iteration count (single-manipulator only)** | JeanLesur | [#328322 #15](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322) | "only one manipulator: now working fine" — breaks multi-arm scenes |
| **Upgrade Isaac Sim version (strange jumps only)** | rodineye | [#303545 #6](https://forums.developer.nvidia.com/t/isaacsim-moveit2-joint-tracking-accuracy-is-not-good-enough/303545) | 2023.1.1 didn't have elbow-joint jump |

---

## User-confirmed FAILURES (attempts that did NOT fix tracking lag)

Critical — my earlier summaries presented several of these as "fixes" because I only read the first staff reply, not the user's follow-up.

| Attempt | Who failed | Source |
|---------|------------|--------|
| OnPhysicsStep + pipelineStageOnDemand | JeanLesur | [#328322 #19](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322) |
| Bypass OmniGraph + Python physics callback | blueed96 | [#363351 #4](https://forums.developer.nvidia.com/t/ros2-isaac-sim-panda-robot-arm-trajectory-deviation/363351) |
| Physics rate 120–240 Hz (multi-manipulator) | blueed96, JeanLesur | [#363351 #4](https://forums.developer.nvidia.com/t/ros2-isaac-sim-panda-robot-arm-trajectory-deviation/363351), [#328322 #13](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322) |
| Lower PD gains | blueed96 | [#363351 #4](https://forums.developer.nvidia.com/t/ros2-isaac-sim-panda-robot-arm-trajectory-deviation/363351) — **made it worse** |
| Higher PD gains (kp=60000 vs 22918) | blueed96 | [#363351 #4](https://forums.developer.nvidia.com/t/ros2-isaac-sim-panda-robot-arm-trajectory-deviation/363351) — partial only |
| Remove joint limits in Isaac Sim | JeanLesur | [#328322 #4](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322) |
| use_sim_time=true on ros2_control | sidd | [#309047 #5](https://forums.developer.nvidia.com/t/state-tolerance-errors-when-simulating-ur10-robot-in-isaac-sim/309047) |
| Matching /joint_commands rate to physics_dt | blueed96 | [#351113 #1](https://forums.developer.nvidia.com/t/collision-from-isaac-sim-running-with-moveit-trajectory/351113) |
| Interpolating trajectory to more waypoints | blueed96 | [#351113 #1](https://forums.developer.nvidia.com/t/collision-from-isaac-sim-running-with-moveit-trajectory/351113) |
| restOffset=0, contactOffset=1e-5 | blueed96 | [#351113 #1](https://forums.developer.nvidia.com/t/collision-from-isaac-sim-running-with-moveit-trajectory/351113) — *already tried your current setup (0.1mm contactOffset)* |
| Min Simulation Frame Rate 30→120 | radhen.17, JeanLesur | [#269397 #4, #8](https://forums.developer.nvidia.com/t/arm-freezes-while-executing-motion-plan-generated-via-moveit/269397) |

### Key contradictions worth understanding

- **daohui.liu (#312476)** says rclpy + articulation controller physics callback solved it; **blueed96 (#363351 #4)** says it didn't. Possible explanation: different failure modes — daohui.liu was fixing wrong-goal-pose issues; blueed96 was fixing mid-trajectory deviation. Worth trying anyway but don't count on it.
- **zhengwang recommended 3 workarounds in #363351 (bypass OG / raise physics rate / lower PD gains)**; blueed96 tried all three and none fully worked. Staff recommendations are hypotheses, not confirmed fixes.

---

## Staff root-cause explanations (may or may not correspond to real fix)

- **[#363351 #3 — zhengwang]**: Default ROS2 Joint State OmniGraph uses `OnPlaybackTick` (render frame), not per physics step. ROS2 messages arrive between graph evaluations → old command persists for extra physics steps → position/velocity jump amplified by high PD gains. *However, blueed96 reports the recommended workarounds don't fix this — so either the root cause is incomplete or the workarounds don't fully address it.*
- **[#365482 #3 — PeterNV]**: For robots shaking after RL training deployment, recommend `damping ≈ stiffness/10`, Solver PosIter=64, VelIter=4, SleepThresh=5e-5, StabThresh=1e-5. **Context: legged robot, not arm.** Apply the ratio directionally, not the specific numbers verbatim.
- **[#346978 #3 — zhengwang]**: `JointState.header.stamp` is "when sent," not "when to execute." Isaac Sim processes commands immediately. If you need temporal pacing, use an OG Delay node or script-side scheduling.
- **[#212101 — un-answered bug, filed 2022]**: `Ros2BridgeUsePhysicsStepSimTime` corrupts `/joint_states` timestamp (sec → nanosec field). Never got a staff fix.

---

## Recommendation for SO-ARM101 (updated after full-thread read)

Ordered cheapest → most structural:

1. **Diff SO-ARM101 USD vs. official Franka/UR10 USD** (30–60 min, zero code)
   - Joint limits (match URDF/xacro → USD)
   - DriveAPI stiffness/damping/max_force and their ratios
   - URDF `effort` and `scale` values
   - Link mass and inertia
   - Articulation root and joint structure
2. **Verify URDF xacro joint limits match USD joint limits** — the only fix confirmed by two independent users.
3. **If 1–2 don't close it:** try daohui.liu's pattern (rclpy + articulation controller from physics callback, cuRobo example). Don't expect it to work but cheap to test.
4. **If 1–3 don't close it:** build the `--verify` measurement harness from the drive tuner spec (§5 of `2026-04-22-isaac-sim-drive-tuner-design.md`) — ~200 LOC, no analytical logic. Tells you whether tuning is even needed.
5. **Only after measurement confirms drive dynamics are the dominant error source:** build the full analytical tuner.

---

## Skill-improvement principles for `nvidia-suite-docs/nvidia-forums/`

Captured for future update to `router.md`. These are things I got wrong this session and want the skill to prevent:

1. **Never truncate post bodies.** Read the full thread. Resolutions commonly live in posts 6–20.
2. **Verify staff recommendations ≠ fixes.** Staff post hypotheses; users try them; users often report back they failed. My initial summary credited zhengwang's recommendations in #363351 as canonical fixes. blueed96 #4 said all three failed.
3. **"Auto-closed after 14 days" ≠ "solved".** Check the last user post, not the bot post.
4. **Cross-reference repeat users across threads.** blueed96 posted 3 threads (#346978, #351113, #363351) about the same problem — their full journey is more informative than any single thread.
5. **Look for edits/updates in post #1.** Users frequently append "UPDATE:" or "SOLVED:" to their original post.
6. **Tabulate attempt → outcome, not chronology.** The structure "tried X, failed / tried Y, worked" extracted from full threads is what actually answers a debugging question.
7. **"Official asset works, my asset doesn't" is the most powerful cross-thread pattern.** When that surfaces, pivot from tuning to asset-diffing.
8. **Depth-2 graph traversal surfaces adjacent clusters.** Keyword search alone misses them (e.g., the "shaking" cluster had the only concrete PD-tuning numbers, via an RL-locomotion context).
9. **Budget ~15 min per thread for full read.** Cheaper than implementing a wrong fix based on a truncated summary.
10. **Star-node centrality heuristic.** The thread with the highest ratio of [NEW] related_topics at depth 1 is the bridge into adjacent problem domains. In this session: #309047 had 9/10 new at depth 2.
11. **Staff name filtering:** `zhengwang` → primary Isaac Sim + MoveIt staff; `PeterNV` → ROS control + RL; `rthaker` → older 2023 threads. Filtering long threads by these usernames is a fast triage.
12. **Expect graph closure ~depth 3.** Depth 1 = 100% new, depth 2 = 76% new, depth 3 projected ~40%. Stop before hitting diminishing returns.

---

## All 15 threads indexed (for future reference)

| ID | Title | Posts | Last | Role in finding |
|----|-------|-------|------|-----------------|
| [363351](https://forums.developer.nvidia.com/t/ros2-isaac-sim-panda-robot-arm-trajectory-deviation/363351) | Panda Robot-arm trajectory deviation | 4 | 2026-04-13 | Staff root cause + all 3 workarounds FAILED per user |
| [328322](https://forums.developer.nvidia.com/t/difference-between-path-planned-in-moveit-and-executed-in-isaac-sim/328322) | Difference between path planned in MoveIt and executed | 21 | 2025-09-24 | Official-asset-works finding; multi-partial-solutions post #15 |
| [312476](https://forums.developer.nvidia.com/t/inconsistent-trajectory-following-in-isaac-sim-with-moveit-using-topic-based-ros2-control/312476) | Inconsistent Trajectory Following w/ topic_based_ros2_control | 8 | 2025-03-10 | daohui.liu's rclpy-articulation-controller fix |
| [303545](https://forums.developer.nvidia.com/t/isaacsim-moveit2-joint-tracking-accuracy-is-not-good-enough/303545) | MoveIt2 joint tracking accuracy not good enough | 7 | 2025-06-28 | Elbow-joint jump fixed by Isaac Sim version downgrade |
| [351113](https://forums.developer.nvidia.com/t/collision-from-isaac-sim-running-with-moveit-trajectory/351113) | Collision from Isaac Sim Running with MoveIt | 2 | 2025-11-13 | blueed96 already tried contactOffset tuning (matches SO-ARM101 setup) |
| [346978](https://forums.developer.nvidia.com/t/isaac-sim-ignores-ros2-moveit-joint-command-timestamp/346978) | Isaac sim ignores timestamp | 3 | 2025-10-07 | Staff-confirmed: stamp = sent-at, not execute-at |
| [309047](https://forums.developer.nvidia.com/t/state-tolerance-errors-when-simulating-ur10-robot-in-isaac-sim/309047) | State tolerance errors UR10 | 8 | 2025-06-10 | Star node; use_sim_time tried, didn't help |
| [269397](https://forums.developer.nvidia.com/t/arm-freezes-while-executing-motion-plan-generated-via-moveit/269397) | Arm freezes while executing motion plan | 8 | 2025-04-30 | URDF xacro limits = USD limits fix (radhen.17) |
| [240913](https://forums.developer.nvidia.com/t/the-joint-gap-btw-ros-and-sim/240913) | Joint gap btw ROS and Sim | 5 | 2023-03-01 | URDF re-import fix (Effort/Scale) |
| [349267](https://forums.developer.nvidia.com/t/robot-arm-is-shaking-during-pose-tracking/349267) | Robot Arm is Shaking during Pose Tracking | 4 | 2025-11-03 | Increase damping fix |
| [345143](https://forums.developer.nvidia.com/t/isaacsim-ros2-control-moveit/345143) | Isaacsim + ros2_control + moveit | 7 | 2025-10-20 | Off-topic; user had GPU crash, resolved by kernel update |
| [353068](https://forums.developer.nvidia.com/t/ros2-joint-commands-ignored-in-isaac-sim-5-1-0-cloud-docker-topics-visible-robot-doesnt-move/353068) | ROS2 joint commands ignored Isaac Sim 5.1.0 cloud/docker | 5 | 2025-12-23 | fastdds.xml not sourced (cloud/docker only) |
| [212101](https://forums.developer.nvidia.com/t/timestamp-bug-in-ros2-bridge-of-isaac-sim-using-physicsstepsimtime/212101) | Timestamp Bug in ROS2 Bridge | 2 | 2024-04-05 | Open bug since 2022, no staff fix |
| [324907](https://forums.developer.nvidia.com/t/unstable-robot-movement-with-moveit/324907) | Unstable robot movement with MoveIt | 1 | 2025-02-24 | No replies; confirms widespread symptom |
| [365482](https://forums.developer.nvidia.com/t/isaac-sim-robot-shaking-after-implementing-isaac-ros-control-via-reinforcement-learning/365482) | Robot shaking after RL control | 2 | 2026-04-13 | PeterNV's full tuning recipe (legged-robot context) |

---

## Session log

- 2026-04-24: Initial traversal — 4 keyword searches + 2-hop related_topics graph walk. Full-text read of all 15 threads after user called out truncation error.
- 2026-04-24 (later): Audited our motion telemetry pipeline (`scripts/motion_logger.py` + `motion_analyze.py`) against forum-canonical plots. Built `scripts/motion_plot.py` to reproduce the forum bug-report plot style.
- 2026-04-24 (evening): Damping bump experiment (B 0.15 → 1.5) made tracking WORSE; reverted. Discovered all control-stack nodes had `use_sim_time=False` while `/clock` ran degraded — matches forum #309047 (sidd) root cause. Flipped use_sim_time live; broke RViz TF buffer; restarted control stack. **First post-restart batch of 10 motions (07:08–07:10) captured no cup collisions despite same lag magnitude (10-13°) as pre-restart baseline — preserved as positive baseline at `docs/notes/2026-04-24-no-collision-baseline/`.** Tracking lag and RTF degradation persist; collision avoidance was stochastic, not solved.
- 2026-04-24 (later, after cup_blue collision in `20260424T072151_grasp_home`): Confirmed the drop_sweep streak was misleading — the failure mode shifted to **grasp_home from far-pan start pose**. shoulder_pan moves at peak ref_vel 1.256 rad/s (vs drop_sweep wrist_flex 0.7-0.8) and accumulates 22° lag. Layer B = 89% (drive-PD, not planner). The `moving_jaw_so101_v1_link` (wider than tcp_link) hit cup_blue at 0.527 N·s impulse over 1.6 s. **Underlying mechanism is intact, just shifted to whichever motion has the highest commanded velocity.**
- 2026-04-24 (workaround applied): Bumped cup-padding default `_collision_padding_var` from 0 → 1 in `vla_SO-ARM101/src/so_arm101_control/so_arm101_control/control_gui.py:3793` (also changed spinbox `increment=5 → 1` so 1% steps are reachable from the UI). Two readings worked at 1%; no large sample yet. **This is a workaround stacked on an unresolved root cause** — padding compensates by widening the planner's no-go zone around cups, which costs reachable workspace. Phase 9 historical comment (`control_gui.py:166`) notes 5% wasn't enough at the time, so 1% being sufficient now is surprising — likely transient until RTF degrades again or geometry changes.

### Workaround status (revisit when underlying lag is fixed)

| Workaround | What it does | When to revert |
|---|---|---|
| Cup padding default 1% | Forces planner to plan trajectories with 1% extra clearance from cup rims, absorbing some drive-lag-induced TCP excursion | When the drive-lag root cause is fixed (RTF restored, use_sim_time properly set in launch, or drive params re-tuned with verified evidence) — revert to 0% to recover reachable workspace |
| Periodic control-stack restart | Frees accumulated CPU/memory from long-running ros2 nodes; bumps RTF from ~0.10 to ~0.17 for ~10 minutes | When RTF degradation is root-caused and fixed |

These workarounds compose: padding gives geometric headroom, restart gives temporal headroom. Both should be retired once the underlying lag is resolved.
- TODO: Consider extending `nvidia-forums/router.md` with the 12 skill-improvement principles above.
- TODO: If SO-ARM101 diff against Franka reveals a specific drive-param pattern, append to this document as "Hypothesis confirmed / rejected."

---

## Logging pipeline audit (vs. forum canon)

### What the forum posters captured

All three canonical threads (#303545 rodineye, #328322 JeanLesur, #363351 blueed96) used `ros2 bag record` of two topics:
- `/isaac_joint_commands` — JTC reference position (post-interpolation command)
- `/isaac_joint_states` — Isaac Sim-published joint state (physics ground truth)

Then matplotlib overlay plot per joint: command (solid) vs state (dashed or different color).

### What our pipeline captures

`scripts/motion_logger.py` records **4 position views per joint** at 50 Hz:

| CSV column | Source | Role | Forum equivalent |
|---|---|---|---|
| `ref_pos_<j>` | `/arm_controller/controller_state.reference.positions` | JTC interpolated command | `/isaac_joint_commands` ≈ |
| `fb_pos_<j>` | `/arm_controller/controller_state.feedback.positions` | ros2_control's feedback | *(none — forum didn't have this)* |
| `js_pos_<j>` | `/joint_states` | Published joint state (mock hardware in our setup) | `/isaac_joint_states` (partial — ours is a mock echo, see finding below) |
| `isaac_pos_<j>` | MCP `dynamic_control.get_dof_position` | Physics ground truth | `/isaac_joint_states` ≈ (bypass bridge) |

Plus per-sample: velocities (3 views), ros2_control's computed error, contact events, cup poses, TCP FK, sim-rate telemetry (physics_rate_hz, render_fps, app_dt_ms, realtime_factor), gripper world pose.

**Verdict: our telemetry is strictly richer than the forum's.** The 4-view structure lets us localize *which layer* the lag lives in (plan→JTC vs JTC→Isaac vs Isaac→/joint_states). Forum posters could only see the aggregate "ref vs state" divergence.

### What our pipeline was missing until today

**Plots.** The existing `motion_analyze.py` is text-only. No visualization existed to directly compare to the forum screenshots. Added `scripts/motion_plot.py` (~225 LOC):

- `forum_style.png` — 2-curve overlay (ref_pos vs isaac_pos) per joint, matches rodineye/JeanLesur/blueed96 plot style directly
- `full_stack.png` — 3 panels per joint (position with 4 curves, velocity with 3, error with 2)
- `joint_zoom_<j>.png` — single-joint detailed view, matches blueed96 #363351's plot style
- `tcp_xyz.png` — TCP Cartesian position over time (bonus, forum didn't capture this)

Output lands in `~/motion_logs/plots/<csv_stem>/`.

### Critical findings from running the plots

Ran against `20260423T225859_drop_sweep.csv`:

1. **`/joint_states` in our setup is NOT physics ground truth.** It tracks `ref_pos` exactly, which means ros2_control's `topic_based_ros2_control` is echoing mock feedback, not Isaac Sim's actual state. Only `isaac_pos` (from MCP dynamic_control) is authoritative. This matches the pattern `sidd` reported in forum #309047 — state-tolerance aborts never fire because ros2_control believes tracking is perfect.

2. **ros2_control's internal `err_pos` ≈ 0** across the motion, while `ref_pos − isaac_pos` peaks at 13° on wrist_flex. The controller layer is blind to the physics layer's tracking failure.

3. **Velocity overshoot is 3.75× commanded** on wrist_flex (ref −0.8 rad/s, isaac −3 rad/s). Drive chases accumulated position error with high-velocity catch-up — classic underdamped behavior given the 1:100 stiffness:damping ratio vs. PeterNV's 1:10 forum guideline.

4. **Error is transient-dominant, not steady-state.** Peaks at ~t=2.5s during max commanded velocity, converges to 0 after settling. **This invalidates the analytical tuner spec's formula** `K = J·(2·v_peak/ε_target)²` which assumes steady-state `ε = (B·v)/K`. The real error signature is catch-up transient, different mathematical regime.

5. **Per-motion variance is large.** Recent drop_sweeps show 13° wrist_flex peak lag; earlier ones (071657) show 44° wrist_flex, 90° wrist_roll — likely initial-condition mismatch rather than pure tracking lag. Suggests logger should capture a "start-pose-mismatch" flag.

### Peak lag measurements across recent drop_sweeps

From `motion_analyze.py --scan`:

| CSV | wrist_flex peak lag |
|---|---|
| 20260423T225859_drop_sweep (most recent, representative) | 13.04° |
| 20260423T074500_drop_sweep | TBD |
| 20260423T071657_drop_sweep (early-session artifact) | 44.51° |

**Key takeaway:** the problem is real, measurable, and far worse than the 0.9° figure in `docs/specs/2026-04-22-isaac-sim-drive-tuner-design.md`. But the shape of the error (transient, catch-up-dominant) doesn't match the steady-state assumption the tuner's analytical formula was built on.

---

## Logging-pipeline TODOs surfaced by this audit

1. **Subscribe to `/isaac_joint_states` directly** (not just the mock `/joint_states`). Gives us the bridge-side view of physics, lets us measure JS-publishing lag separately from drive lag.
2. **Start-state mismatch detector.** Flag motions where `isaac_pos − ref_pos_at_t0 > threshold` — avoids conflating initial-condition errors with tracking lag.
3. **Plot ruler / threshold line.** Add a horizontal line at τ_TCP-equivalent angular error on the error panel (e.g., the `ε_target` each joint is allowed) so the plot immediately shows whether the motion was "safe."
4. **Optional: `/arm_controller/controller_state` velocity headroom.** Currently captured but not plotted; could help distinguish control-commanded velocity spikes vs. physics-response velocity spikes.
