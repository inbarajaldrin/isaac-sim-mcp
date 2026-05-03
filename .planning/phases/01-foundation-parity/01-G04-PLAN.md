---
phase: 01-foundation-parity
plan: G04
type: execute
wave: 3
depends_on: [G02]
files_modified:
  - "exts/aic-dt/aic_dt/extension.py"
  - "exts/aic-dt/scripts/probe_articulation_joints.py"
  - "exts/aic-dt/docs/CHANGELOG.md"
autonomous: false
gap_closure: true
addresses_gap: A
requirements: [PARITY-03]
tags: [joint-state, articulation, name-override, gazebo-parity]
must_haves:
  truths:
    - "ros2 topic echo /joint_states --once returns the same joint names in the same ordering Gazebo's /joint_states publishes (live snapshot: 7 joints alphabetical, including gripper/left_finger_joint)"
  artifacts:
    - path: "exts/aic-dt/aic_dt/extension.py"
      provides: "/joint_states publishes all 7 expected joints (6 UR5e arm + gripper/left_finger_joint with slash) and frame_id='base_link', via either articulation restructure, supplementary publisher subgraph, or rclpy republisher — strategy chosen by probe outcome"
      contains: "joint_state"
    - path: "exts/aic-dt/scripts/probe_articulation_joints.py"
      provides: "Re-runnable diagnostic that reports which joints are inside /World/UR5e/aic_unified_robot/root_joint articulation, which are excluded, and the OGN-published name set"
      exports: ["main"]
    - path: "exts/aic-dt/docs/CHANGELOG.md"
      provides: "Gap A fix entry citing the 7-joint expected set and the chosen fix strategy"
      contains: "Gap A"
  key_links:
    - from: "ROS2PublishJointState (or supplementary publisher) in setup_joint_state_publish_action_graph"
      to: "/joint_states ROS topic with all 7 joint names + frame_id=base_link"
      via: "articulation traversal OR supplementary subgraph OR republisher"
      pattern: "joint_states"
---

<objective>
Close Gap A (JointState — sim publishes only 6 UR5e joints in articulation-discovery order, missing `gripper/left_finger_joint` entirely; Gazebo publishes 7 joints alphabetical including the slash-form name; frame_id is empty in sim where Gazebo uses 'base_link'). The OGN node has no nameOverrides input, so a wrapper or restructure is required.

Purpose: Phase 1 SC #2 currently fails. aic_adapter::ReorderJointState (the actual /joint_states subscriber, per Plan 05's verdict) has the literal key `gripper/left_finger_joint` (with slash) and silently drops names it doesn't recognize. Without this fix, every Observation message is missing the gripper finger position, breaking Phase 2 controller parity.

Output: After this plan, `ros2 topic echo /joint_states --once` shows 7 joint names (the 6 UR5e arm joints + `gripper/left_finger_joint` with slash) and `header.frame_id=base_link`. Strategy is chosen by Task 1's probe outcome — DIAGNOSE FIRST.
</objective>

<execution_context>
@$HOME/.claude/get-shit-done/workflows/execute-plan.md
@$HOME/.claude/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/STATE.md
@.planning/phases/01-foundation-parity/01-VERIFICATION.md
@.planning/phases/01-foundation-parity/01-CONTEXT.md
@.planning/phases/01-foundation-parity/01-05-SUMMARY.md
@.planning/phases/01-foundation-parity/01-06-SUMMARY.md
@.planning/phases/01-foundation-parity/joint_ordering_probe.txt
@.planning/phases/01-foundation-parity/usd_prim_inventory.txt
@.planning/phases/01-foundation-parity/snapshot/joint_states_sample.yaml
@exts/aic-dt/aic_dt/extension.py
@exts/aic-dt/scripts/probe_root_joint.py
@CLAUDE.md
@~/.claude/skills/nvidia-suite-docs/SKILL.md
@~/.claude/skills/isaac-sim-extension-dev/SKILL.md

<interfaces>
<!-- The 7 expected joint names (from joint_ordering_probe.txt section 6, aic_adapter joint_sort_order_):
  shoulder_pan_joint     (alphabetical sort idx 0 ... but aic_adapter is name-INDEXED per Plan 05 verdict, so order at /joint_states is irrelevant; presence is what matters)
  shoulder_lift_joint
  elbow_joint
  wrist_1_joint
  wrist_2_joint
  wrist_3_joint
  gripper/left_finger_joint  <-- SLASH form; USD prim is `gripper_left_finger_joint` (underscore — slash is illegal in Sdf.Path)

USD prim path of gripper joint (from usd_prim_inventory.txt):
  /World/aic_unified_robot/joints/gripper_left_finger_joint

Live observation (tmp/verify_phase_1_run.log Step 5 + verifier note):
  Currently published: 6 UR5e arm joints in articulation-discovery order
  Missing: gripper_left_finger_joint entirely (or gripper/left_finger_joint — TBD by probe)
  frame_id: empty (Gazebo: 'base_link')

ROS2PublishJointState OGN spec (Plan 06 confirmed):
  - inputs: targetPrim (relationship to articulation root), topicName, nodeNamespace, header.frameId, qosProfile, etc.
  - NO inputs.nameOverrides, NO inputs.jointNames filter — names come from articulation traversal of the bound prim.

Three viable fix strategies (probe in Task 1 picks one):

  STRATEGY A: ARTICULATION_RESTRUCTURE
    Investigate why gripper_left_finger_joint isn't traversed by the OGN under /World/UR5e/aic_unified_robot/root_joint. If it's a USD-side scoping issue (joint scoped under a non-articulated parent), restructure the articulation tree to include it. Risk: high — touches PhysicsArticulationRoot semantics, may regress drives.

  STRATEGY B: SUPPLEMENTARY_PUBLISHER_SUBGRAPH
    Add a second OGN subgraph in setup_joint_state_publish_action_graph: an OgnReadPrimAttribute (or equivalent) that reads the gripper finger joint's position attribute, plus a custom publisher (script-node + rclpy?) that emits a sensor_msgs/JointState message with name=['gripper/left_finger_joint'] and concatenates with the main publisher. Possibly simpler: an Isaac Sim ScriptNode running rclpy that re-broadcasts from articulation state.

  STRATEGY C: WRAPPER_REPUBLISHER (rclpy node inside the extension)
    Subscribe to /joint_states (the publisher's underscore-form output), rename `gripper_left_finger_joint` -> `gripper/left_finger_joint`, set frame_id='base_link', republish on /joint_states. Risk: topic loop. Mitigation: publisher emits on /joint_states_raw, republisher reads /joint_states_raw and writes /joint_states. The OGN node's topicName is editable (Plan 06 set it explicitly), so this is feasible.

  STRATEGY D: HEADER_FRAME_ID_ONLY (partial fix — name override deferred further)
    Set header.frameId='base_link' on the OGN publisher. This closes the frame_id half of Gap A but NOT the missing-gripper-joint half. Use only as a fallback if A/B/C all fail.

Existing setup_joint_state_publish_action_graph (extension.py around line 1540).
Plan 06 set targetPrim='/World/UR5e/aic_unified_robot/root_joint' and topicName='joint_states' explicitly.
-->
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: Author probe_articulation_joints.py — diagnose where gripper_left_finger_joint lives + which strategy is feasible</name>
  <files>exts/aic-dt/scripts/probe_articulation_joints.py</files>
  <action>
    Per CLAUDE.md canonical references rule: this task touches PhysX articulation + USD scoping — consult `~/.claude/skills/nvidia-suite-docs/SKILL.md` (PhysicsArticulationRoot, fixed-joint-as-articulation-root pattern, isaacsim.core.utils.articulations) AND `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` (project's existing probe pattern from probe_root_joint.py / probe_unified_usd.py).

    Author `exts/aic-dt/scripts/probe_articulation_joints.py` — a re-runnable probe that answers four questions:
    1. **Articulation set**: Which joints are children of `/World/UR5e/aic_unified_robot/root_joint` (the PhysicsFixedJoint articulation root)? Print the prim paths.
    2. **Joint traversal scope**: When ROS2PublishJointState binds to that articulation root, which joints does it actually traverse and publish? (May require running quick_start and capturing `ros2 topic echo /joint_states --once`.)
    3. **Gripper joint scoping**: Where is `/World/aic_unified_robot/joints/gripper_left_finger_joint` (note: this prim path differs from the articulation root by being under `/World/aic_unified_robot/...` not `/World/UR5e/aic_unified_robot/...` — confirm the actual location). Is it inside the articulation, outside, or excluded due to a missing PhysicsRevoluteJoint API?
    4. **OGN name surface check**: Run `python3 -c "import omni.isaac.ros2_bridge.OgnROS2PublishJointState as O; print(dir(O))"` (inside Kit) or read the OGN .rst file at `~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/docs/ogn/OgnROS2PublishJointState.rst` to confirm definitively whether nameOverrides was added in any minor version. (Plan 06 confirmed it wasn't — re-confirm against the installed version.)

    Two execution modes:
    - `python3 exts/aic-dt/scripts/probe_articulation_joints.py offline` — uses pxr only (Usd.Stage.Open on the unified robot USD); answers questions 1, 3 (USD scoping). Uses `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh`.
    - `python3 exts/aic-dt/scripts/probe_articulation_joints.py live --port 8768` — sends MCP execute_python_code to: (a) traverse the live articulation via `omni.isaac.core.articulations.Articulation` or `dynamic_control`; (b) call `ros2 topic echo /joint_states --once` via subprocess to capture the actual published name set; answers question 2.

    Output: a `## Decision` section at the bottom (mirror Plan 05's pattern) writing one of four verdict tokens and a rationale:
    ```
    ## Decision (recorded by Plan G04 Task 1)

    Strategy: ARTICULATION_RESTRUCTURE | SUPPLEMENTARY_PUBLISHER_SUBGRAPH | WRAPPER_REPUBLISHER | HEADER_FRAME_ID_ONLY
    Rationale: <why this strategy was chosen given probe outputs>
    Implementing task: Task 2
    ```

    Save the probe output to `tmp/g04_articulation_probe.txt` (working data, not committed) and the decision to `exts/aic-dt/scripts/probe_articulation_joints.py` itself as a comment block at the top OR to a small artifact `.planning/phases/01-foundation-parity/g04_articulation_decision.txt` (committed).

    Reference attribution at top of file: "# Reference: gap-closure G04 — Gap A (JointState completion). Mirrors Plan 05's probe_unified_usd.py / probe_aic_controller_jointstate.sh decision-block pattern."
  </action>
  <verify>
    <automated>python3 -m py_compile exts/aic-dt/scripts/probe_articulation_joints.py &amp;&amp; ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/probe_articulation_joints.py offline | tee /tmp/g04_probe_out.txt &amp;&amp; (test -f .planning/phases/01-foundation-parity/g04_articulation_decision.txt || grep -E "Strategy: (ARTICULATION_RESTRUCTURE|SUPPLEMENTARY_PUBLISHER_SUBGRAPH|WRAPPER_REPUBLISHER|HEADER_FRAME_ID_ONLY)" /tmp/g04_probe_out.txt)</automated>
  </verify>
  <done>
    Probe script compiles, runs offline mode successfully, outputs a strategy decision token from the constrained vocabulary. Probe output identifies the actual location of `gripper_left_finger_joint` and confirms whether it's reachable from the articulation root.
  </done>
</task>

<task type="auto">
  <name>Task 2: Apply the strategy-specific JointState fix</name>
  <files>exts/aic-dt/aic_dt/extension.py</files>
  <action>
    Read the strategy decision from Task 1's output. Apply the corresponding fix in `setup_joint_state_publish_action_graph` (extension.py around line 1540).

    Per CLAUDE.md canonical references rule: this task touches OGN nodes / articulation APIs / rclpy — consult `nvidia-suite-docs/SKILL.md` for the chosen-strategy-specific API surface. If the strategy involves a ScriptNode or rclpy republisher, also consult `isaac-sim-extension-dev/SKILL.md` for project-specific glue patterns.

    Strategy-specific implementations:

    **STRATEGY A: ARTICULATION_RESTRUCTURE**
    - Locate the gripper_left_finger_joint prim (probe Task 1 confirmed location). Verify it has the right PhysicsRevoluteJoint API + body0/body1 relationships.
    - If scoping is the issue (joint outside the articulation root subtree), use pxr to move/duplicate it under the right scope, OR mark the joint with the right `physxArticulation:articulationEnabled` attribute.
    - Re-test: post-fix, the OGN traverses 7 joints. Apply via in-place edit per D-06 if it's a USD edit, or via runtime articulation API call inside `load_robot` if it's a configuration step.
    - Always set `header.frameId='base_link'` on the OGN publisher input regardless.

    **STRATEGY B: SUPPLEMENTARY_PUBLISHER_SUBGRAPH**
    - Add a second OGN subgraph in `setup_joint_state_publish_action_graph` after the existing publisher: a Python ScriptNode (or compatible OGN chain) that reads the gripper joint's position via dynamic_control or articulation API and publishes a sensor_msgs/JointState message containing only `gripper/left_finger_joint` on `/joint_states`. Risk: tf2_ros / aic_adapter may not consume two messages on the same topic — check by running the verify harness.
    - Set frame_id='base_link' on both the main and the supplementary publisher.

    **STRATEGY C: WRAPPER_REPUBLISHER (preferred fallback if A is risky)**
    - Change main OGN publisher's `topicName` from 'joint_states' to 'joint_states_raw'.
    - Add a Python rclpy republisher node inside the extension (init in `on_startup` or lazily in `setup_joint_state_publish_action_graph`):
      - Subscribe to `/joint_states_raw`.
      - For each incoming message: rewrite `name[i]` mapping `gripper_left_finger_joint` -> `gripper/left_finger_joint`. Set `header.frame_id='base_link'`.
      - Publish on `/joint_states`.
    - Use `rclpy.create_node('aic_dt_joint_state_remap')` plus a one-line subscription callback. This is consistent with CLAUDE.md cross-repo rule ("aic-dt MAY use rclpy for non-control glue").
    - Lifecycle: shutdown the rclpy node in `on_shutdown`.

    **STRATEGY D: HEADER_FRAME_ID_ONLY (escape hatch)**
    - Set the existing OGN publisher's `header.frameId` input to `'base_link'`. Document inline that the missing-gripper-joint fix is deferred to a Phase 2 follow-up. Update the verifier expectation accordingly. Use ONLY if A/B/C all fail in Task 1's probe analysis.

    Add inline comment: `# Gap A (G04): &lt;STRATEGY-TOKEN&gt; — see .planning/phases/01-foundation-parity/01-G04-SUMMARY.md`

    Idempotency: re-invocation of `setup_joint_state_publish_action_graph` should not duplicate nodes / spawn duplicate rclpy nodes.
  </action>
  <verify>
    <automated>python3 -m py_compile exts/aic-dt/aic_dt/extension.py &amp;&amp; grep -E "Gap A \(G04\)|gripper/left_finger_joint|joint_states_raw|base_link" exts/aic-dt/aic_dt/extension.py | wc -l | awk '{if ($1 &lt; 2) {print "FAIL: too few Gap A markers/strings (" $1 ")"; exit 1} else print "OK: " $1 " hits"}'</automated>
  </verify>
  <done>
    extension.py compiles. Strategy-specific fix is in place. Inline marker comment cites Gap A + chosen strategy. The literal string `gripper/left_finger_joint` (with slash) appears somewhere in extension.py source. `base_link` frame_id assignment is present.
  </done>
</task>

<task type="checkpoint:human-verify" gate="blocking">
  <name>Task 3: Human-verify /joint_states has 7 joints + base_link frame_id</name>
  <what-built>Strategy-specific JointState fix from Task 2 — name set + frame_id now match Gazebo's live snapshot.</what-built>
  <how-to-verify>
    1. Cache check + restore if needed.
    2. Launch Isaac Sim + quick_start (CLAUDE.md recipe).
    3. With ROS2 sourced: `ros2 topic echo /joint_states --once`. Expected:
       ```
       header:
         stamp: ...
         frame_id: base_link
       name:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint
       - gripper/left_finger_joint
       ```
       (Order can be articulation-natural; aic_adapter is name-indexed per Plan 05 verdict — only presence + slash-form matters. Comparison reference: `.planning/phases/01-foundation-parity/snapshot/joint_states_sample.yaml`.)
    4. Verify rate: `ros2 topic hz /joint_states` — steady stream, no message drops.
    5. Cross-check with aic_adapter contract: count of distinct `name:` entries == 7. Slash-form `gripper/left_finger_joint` is present (NOT `gripper_left_finger_joint`).
    6. Snapshot cache.
  </how-to-verify>
  <resume-signal>
    Type "approved" if all 7 joints appear with the slash-form gripper name and frame_id='base_link'. If still wrong, paste the actual `ros2 topic echo --once` output — Task 2 may need an iteration or strategy fallback.
  </resume-signal>
</task>

<task type="auto">
  <name>Task 4: CHANGELOG entry + REQUIREMENTS status flip</name>
  <files>exts/aic-dt/docs/CHANGELOG.md, .planning/REQUIREMENTS.md</files>
  <action>
    1. Append CHANGELOG entry:
       ```
       ## Gap A fix (JointState completion — PARITY-03)
       - Strategy: &lt;TOKEN from G04 probe Task 1&gt;
       - 7 joints now published: 6 UR5e arm + gripper/left_finger_joint (slash)
       - frame_id='base_link' (matches live Gazebo snapshot)
       - Verified: ros2 topic echo /joint_states --once shows 7-name set (G04 Task 3 checkpoint)
       - References: joint_ordering_probe.txt aic_adapter::ReorderJointState verdict, .planning/phases/01-foundation-parity/01-G04-SUMMARY.md
       ```
    2. Update `.planning/REQUIREMENTS.md` PARITY-03 from `[~]` to `[x]` IF AND ONLY IF Task 3 returned "approved" with all 7 joints present. Otherwise note remaining gaps.

    Per CLAUDE.md canonical references rule: cite both skills in CHANGELOG entry as references.
  </action>
  <verify>
    <automated>grep -E "Gap A fix|JointState completion" exts/aic-dt/docs/CHANGELOG.md &amp;&amp; grep -E "^- \[(x|~)\] \*\*PARITY-03\*\*" .planning/REQUIREMENTS.md</automated>
  </verify>
  <done>
    CHANGELOG entry committed. PARITY-03 status reflects checkpoint outcome.
  </done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| OGN subgraph / rclpy republisher (if Strategy C) | Same trust as Phase 1 baseline; no external input. |
| /joint_states ROS topic | Consumed by aic_adapter, aic_engine; controller-loop critical. |

## STRIDE Threat Register (ASVS L1)

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-G04-01 | Tampering | Articulation restructure (Strategy A) | mitigate | If A is chosen: in-place USD edit per D-06; verify articulation drives still apply post-edit (no regression on UR5e arm). |
| T-G04-02 | Tampering | rclpy republisher topic loop (Strategy C) | mitigate | Use distinct topic names (joint_states_raw → joint_states); test no message-storm via `ros2 topic hz`. |
| T-G04-03 | DoS | Republisher subscription buffer overflow | mitigate | rclpy subscription with bounded queue size (e.g. KeepLast(5)). |
| T-G04-04 | Spoofing | name remapping in republisher | accept | Mapping is hard-coded gripper_left_finger_joint→gripper/left_finger_joint only; no input-driven remapping. |
</threat_model>

<verification>
ros2 topic echo /joint_states --once shows 7 names with `gripper/left_finger_joint` (slash) + frame_id=base_link. ros2 topic hz steady. aic_adapter (if running) does not log "Ignoring unexpected joint name" warnings.
</verification>

<success_criteria>
- probe_articulation_joints.py compiles, picks a strategy from the 4-token vocabulary.
- Strategy-specific fix applied; extension.py compiles.
- Human-verify Task 3 returns "approved" with 7 joints + base_link frame_id.
- CHANGELOG + REQUIREMENTS.md updated.
</success_criteria>

<output>
After completion, create `.planning/phases/01-foundation-parity/01-G04-SUMMARY.md` with the chosen strategy, the actual published joint set post-fix, and any iteration notes.
</output>
