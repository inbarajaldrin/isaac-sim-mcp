---
phase: 01-foundation-parity
plan: G03
type: execute
wave: 3
depends_on: [G02]
files_modified:
  - "exts/aic-dt/aic_dt/extension.py"
  - "exts/aic-dt/docs/CHANGELOG.md"
autonomous: false
gap_closure: true
addresses_gap: B
requirements: [PARITY-04]
tags: [tf-tree, raw-publisher, omnigraph, frame-overrides, gazebo-parity]
must_haves:
  truths:
    - "ros2 run tf2_tools view_frames against Isaac Sim produces a TF tree whose robot + gripper + camera frames match Gazebo's reference TF tree (same names, same parent-child links)"
  artifacts:
    - path: "exts/aic-dt/aic_dt/extension.py"
      provides: "Per-frame ROS2PublishRawTransformTree nodes for the 16 underscore-form prims + 1 synthesized aic_world edge, each backed by an OgnGetPrimWorldPose / IsaacReadOdometry pose-source node, wired into setup_tf_publish_action_graph"
      contains: "ROS2PublishRawTransformTree"
    - path: "exts/aic-dt/docs/CHANGELOG.md"
      provides: "Gap B fix entry citing the 17-entry override list from usd_prim_inventory.txt"
      contains: "Gap B"
  key_links:
    - from: "OmniGraph subgraph in setup_tf_publish_action_graph"
      to: "/tf and /tf_static ROS topics with slash-form frame_ids"
      via: "OgnGetPrimWorldPose source -> ROS2PublishRawTransformTree publisher (1 chain per override entry)"
      pattern: "ROS2PublishRawTransformTree"
---

<objective>
Close Gap B (TF tree mismatch — sim publishes 15 frames / 13 edges; reference publishes 31 / 30; 18 frames missing including all gripper/*, ati/*, cam_mount/*, *_camera/* subtrees + tabletop + aic_world). Wire per-frame `ROS2PublishRawTransformTree` nodes for each entry in the 17-entry override list documented in `usd_prim_inventory.txt`, each backed by a pose-source node (`OgnGetPrimWorldPose` or `IsaacReadOdometry`) so the published frame_id can carry slash characters that USD prim names cannot.

Purpose: Phase 1 SC #3 currently fails. This plan completes the deferral that Plan 06 documented. After this plan, `diff_tf_tree.py aic_frames_live.gv /tmp/sim_frames.gv` returns "PASS: TF trees match (31 frames, 30 edges)".

Output: Per-frame Raw publisher subgraph in `setup_tf_publish_action_graph`; the existing default-mapping `ROS2PublishTransformTree` nodes are preserved for the 14 MATCH frames (base, base_link, base_link_inertia, flange, forearm_link, ft_frame, shoulder_link, tabletop, tool0, upper_arm_link, world, wrist_1_link, wrist_2_link, wrist_3_link).
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
@.planning/phases/01-foundation-parity/usd_prim_inventory.txt
@.planning/phases/01-foundation-parity/aic_frames_live.gv
@exts/aic-dt/aic_dt/extension.py
@exts/aic-dt/scripts/diff_tf_tree.py
@CLAUDE.md
@~/.claude/skills/nvidia-suite-docs/SKILL.md
@~/.claude/skills/isaac-sim-extension-dev/SKILL.md

<interfaces>
<!-- The 17-entry override list from usd_prim_inventory.txt:

OVERRIDE  prim='None'                                                     expected_frame_id='aic_world'   parent_frame='world'
OVERRIDE  prim='/World/aic_unified_robot/ati_base_link'                   expected_frame_id='ati/base_link'                  parent='cam_mount/cam_mount_link'
OVERRIDE  prim='/World/aic_unified_robot/ati_tool_link'                   expected_frame_id='ati/tool_link'                  parent='ati/base_link'
OVERRIDE  prim='/World/aic_unified_robot/cam_mount_cam_mount_link'        expected_frame_id='cam_mount/cam_mount_link'       parent='tool0'
OVERRIDE  prim='/World/aic_unified_robot/center_camera_camera_link'       expected_frame_id='center_camera/camera_link'      parent='cam_mount/cam_mount_link'
OVERRIDE  prim='/World/aic_unified_robot/center_camera_optical'           expected_frame_id='center_camera/optical'          parent='center_camera/sensor_link'
OVERRIDE  prim='/World/aic_unified_robot/center_camera_sensor_link'       expected_frame_id='center_camera/sensor_link'      parent='center_camera/camera_link'
OVERRIDE  prim='/World/aic_unified_robot/gripper_hande_base_link'         expected_frame_id='gripper/hande_base_link'        parent='ati/tool_link'
OVERRIDE  prim='/World/aic_unified_robot/gripper_hande_finger_link_l'     expected_frame_id='gripper/hande_finger_link_l'    parent='gripper/hande_base_link'
OVERRIDE  prim='/World/aic_unified_robot/gripper_hande_finger_link_r'     expected_frame_id='gripper/hande_finger_link_r'    parent='gripper/hande_base_link'
OVERRIDE  prim='/World/aic_unified_robot/gripper_tcp'                     expected_frame_id='gripper/tcp'                    parent='gripper/hande_base_link'
OVERRIDE  prim='/World/aic_unified_robot/left_camera_camera_link'         expected_frame_id='left_camera/camera_link'        parent='cam_mount/cam_mount_link'
OVERRIDE  prim='/World/aic_unified_robot/left_camera_optical'             expected_frame_id='left_camera/optical'            parent='left_camera/sensor_link'
OVERRIDE  prim='/World/aic_unified_robot/left_camera_sensor_link'         expected_frame_id='left_camera/sensor_link'        parent='left_camera/camera_link'
OVERRIDE  prim='/World/aic_unified_robot/right_camera_camera_link'        expected_frame_id='right_camera/camera_link'       parent='cam_mount/cam_mount_link'
OVERRIDE  prim='/World/aic_unified_robot/right_camera_optical'            expected_frame_id='right_camera/optical'           parent='right_camera/sensor_link'
OVERRIDE  prim='/World/aic_unified_robot/right_camera_sensor_link'        expected_frame_id='right_camera/sensor_link'       parent='right_camera/camera_link'

The parent-frame column is derived from aic_frames_live.gv (the live Gazebo TF tree edges).
Verify-step 6 missing-edges output in tmp/verify_phase_1_run.log confirms the parent-child structure verbatim.

Key OmniGraph node IDs (consult nvidia-suite-docs SKILL.md for current Isaac Sim 5.0 spec):
  isaacsim.core.nodes.IsaacReadOdometry  (or the modern equivalent that reads a prim's pose)
  isaacsim.core.nodes.OgnGetPrimWorldPose  (preferred — direct prim->translation/rotation extraction)
  isaacsim.ros2.bridge.ROS2PublishRawTransformTree  (consumes static-or-input translation+rotation+frame_ids)
  isaacsim.ros2.bridge.ROS2PublishTransformTree  (current default-mapping publisher; KEEP for 14 MATCH frames)

Behavioral contract:
  - 14 MATCH frames continue via the existing default-mapping publishers (no change).
  - 16 UNDERSCORE-MISMATCH frames each get an OgnGetPrimWorldPose -> ROS2PublishRawTransformTree chain;
    inputs: parentFrameId = parent column above; childFrameId = expected_frame_id; translation/rotation = pose-source outputs.
  - 1 NO-PRIM (aic_world) gets a static identity Raw publisher: parentFrameId='world', childFrameId='aic_world',
    translation=(0,0,0), rotation=(0,0,0,1). Published as static (transient_local QoS) on /tf_static.
  - All 17 publishers run on the same OnPlaybackTick / OnTick that the existing TF graph uses, so the rate is consistent.

Existing setup_tf_publish_action_graph location (extension.py line ~1418) — extends, does not replace.
-->
</interfaces>
</context>

<tasks>

<task type="auto">
  <name>Task 1: Read OGN specs; design and implement per-frame Raw publisher subgraph</name>
  <files>exts/aic-dt/aic_dt/extension.py</files>
  <action>
    Per CLAUDE.md canonical references rule: this task touches OmniGraph + isaacsim.ros2.bridge OGN nodes which are version-sensitive. **Before authoring**, consult `~/.claude/skills/nvidia-suite-docs/SKILL.md` (Isaac Sim 5.0 / OmniGraph / `isaacsim.ros2.bridge` ROS2PublishRawTransformTree input attributes) AND read the local OGN spec the same way Plan 06 did:
    ```
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/docs/ogn/OgnROS2PublishRawTransformTree.rst
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/docs/ogn/OgnROS2PublishTransformTree.rst
    ```
    Specifically confirm: (a) ROS2PublishRawTransformTree's exact input attribute names for parentFrameId, childFrameId, translation, rotation, staticPublisher; (b) the right node ID + spec for OgnGetPrimWorldPose (or its equivalent — IsaacReadPrimAttribute, IsaacReadOdometry, etc. — pick whichever current 5.0 OGN reads a prim's world pose).

    Then extend `setup_tf_publish_action_graph` in `exts/aic-dt/aic_dt/extension.py` (around line 1418, where the existing `ROS2PublishTransformTree` x2 nodes are wired):

    1. Define a static list `_TF_FRAME_OVERRIDES` (module-level or inside the method) — exactly the 17 entries from `usd_prim_inventory.txt`, each with: `(prim_path_or_None, expected_frame_id, parent_frame_id, is_static)`. The aic_world entry has `prim_path=None, is_static=True`. The other 16 have `prim_path=<USD path>, is_static=False`.

    2. For each non-static override (16 entries):
       - Create an `OgnGetPrimWorldPose` (or current equivalent) source node bound to the prim path.
       - Create a `ROS2PublishRawTransformTree` publisher node with parentFrameId/childFrameId set per the override entry, staticPublisher=False, topicName='tf'.
       - Wire source.translation -> publisher.translation, source.rotation -> publisher.rotation.
       - Connect to the OnPlaybackTick that drives the existing TF nodes.

    3. For the static `aic_world` entry:
       - No source node — set translation=(0,0,0), rotation=(0,0,0,1) literally on the publisher inputs.
       - parentFrameId='world', childFrameId='aic_world', staticPublisher=True, topicName='tf_static'.

    4. KEEP the existing two `ROS2PublishTransformTree` nodes (default-mapping, staticPublisher=False/True) — they continue to publish the 14 MATCH frames. Do NOT remove or alter their `targetPrims` binding.

    5. Add inline docstring at the top of the method explaining the 14+17 split (14 default-mapping MATCH frames + 16 Raw underscore→slash overrides + 1 synthesized aic_world edge = 31 total frames matching aic_frames_live.gv).

    6. Update CHANGELOG note inline (one-line comment) referencing Gap B closure: `# Gap B (G03): per-frame Raw overrides for 17 underscore->slash + aic_world`.

    Use the relationship-after-edit pattern Plan 06 used (per its summary). The graph builder must be hot-reload-safe — re-invocation should clean up prior nodes before recreating.

    Idempotency: if `setup_tf_publish_action_graph` is called twice in one session (UI button re-click + quick_start), the second call should not duplicate nodes. Reuse Plan 06's "remove existing graph then recreate" pattern if present, otherwise add it.
  </action>
  <verify>
    <automated>python3 -m py_compile exts/aic-dt/aic_dt/extension.py &amp;&amp; grep -E "ROS2PublishRawTransformTree|OgnGetPrimWorldPose|_TF_FRAME_OVERRIDES" exts/aic-dt/aic_dt/extension.py | wc -l | awk '{if ($1 &lt; 5) {print "FAIL: too few overrides referenced (" $1 ")"; exit 1} else print "OK: " $1 " hits"}' &amp;&amp; grep -E "aic_world|gripper/hande_base_link|cam_mount/cam_mount_link|ati/tool_link|center_camera/camera_link" exts/aic-dt/aic_dt/extension.py | wc -l | awk '{if ($1 &lt; 5) {print "FAIL: not all expected slash-form frame_ids present"; exit 1} else print "OK"}'</automated>
  </verify>
  <done>
    extension.py compiles. The 17 override entries are present in source (the static list `_TF_FRAME_OVERRIDES`). All 5 representative slash-form frame_ids (aic_world, gripper/hande_base_link, cam_mount/cam_mount_link, ati/tool_link, center_camera/camera_link) appear as literal strings. The 14 MATCH frames continue via the existing publishers (no removal).
  </done>
</task>

<task type="checkpoint:human-verify" gate="blocking">
  <name>Task 2: Human-verify TF tree diff is clean</name>
  <what-built>17 per-frame Raw publishers wired into setup_tf_publish_action_graph (16 underscore→slash overrides + 1 synthesized aic_world static edge).</what-built>
  <how-to-verify>
    1. Cache check: `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh ~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py status`. Restore if &lt; 100MB.
    2. Launch Isaac Sim: `DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt`.
    3. Send quick_start via MCP (CLAUDE.md recipe). Confirm "Quick start complete..." response.
    4. In a separate terminal with ROS2 sourced (e.g. `source /opt/ros/kilted/setup.bash` inside the aic_eval container or local Kilted install): `ros2 run tf2_tools view_frames -o /tmp/sim_frames` (produces /tmp/sim_frames.gv).
    5. Run the verify script's TF diff: `python3 exts/aic-dt/scripts/diff_tf_tree.py .planning/phases/01-foundation-parity/aic_frames_live.gv /tmp/sim_frames.gv`. Expected: "PASS: TF trees match (31 frames, 30 edges)" OR a much smaller delta than the 18 frames currently missing.
    6. Spot-check specific frames via tf2_echo: `ros2 run tf2_ros tf2_echo base_link gripper/hande_base_link` → returns valid transform; `tf2_echo world aic_world` → returns identity.
    7. After successful verification: `prime_usd_cache.py snapshot`.
  </how-to-verify>
  <resume-signal>
    Type "approved" if diff_tf_tree.py reports zero diff (or only the legend artifact `view_frames Result`). If diffs remain, paste the diff_tf_tree.py output — Task 1 may need an iteration on the failing frames.
  </resume-signal>
</task>

<task type="auto">
  <name>Task 3: CHANGELOG entry + REQUIREMENTS status flip</name>
  <files>exts/aic-dt/docs/CHANGELOG.md, .planning/REQUIREMENTS.md</files>
  <action>
    1. Append CHANGELOG entry under "Unreleased" or the Phase 1 milestone section:
       ```
       ## Gap B fix (TF tree completion — PARITY-04)
       - 17 per-frame Raw publishers wired (16 underscore→slash overrides + 1 synthesized aic_world)
       - 14 MATCH frames continue via existing default-mapping ROS2PublishTransformTree x2 nodes
       - Verified: diff_tf_tree.py reports zero diff (G03 Task 2 checkpoint)
       - References: usd_prim_inventory.txt override list, .planning/phases/01-foundation-parity/01-G03-SUMMARY.md
       ```
    2. Update `.planning/REQUIREMENTS.md` PARITY-04 from `[~]` to `[x]` IF AND ONLY IF the human-verify checkpoint returned "approved" with zero diff. Otherwise append a "remaining diffs" note and keep `[~]`.

    Per CLAUDE.md canonical references rule: cite `nvidia-suite-docs/SKILL.md` and `isaac-sim-extension-dev/SKILL.md` in the CHANGELOG entry as references.
  </action>
  <verify>
    <automated>grep -E "Gap B fix|TF tree completion" exts/aic-dt/docs/CHANGELOG.md &amp;&amp; grep -E "^- \[(x|~)\] \*\*PARITY-04\*\*" .planning/REQUIREMENTS.md</automated>
  </verify>
  <done>
    CHANGELOG entry committed. PARITY-04 status reflects checkpoint outcome (x for clean, ~ if any diffs remain).
  </done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| OmniGraph subgraph in setup_tf_publish_action_graph | Same trust boundary as Plan 06 — internal to the extension. |
| /tf and /tf_static ROS topics | Published by Isaac Sim; consumed by aic_adapter / aic_engine / CheatCode. |

## STRIDE Threat Register (ASVS L1)

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-G03-01 | Tampering | OmniGraph node creation | mitigate | py_compile gate; remove-then-recreate idempotency; OGN spec confirmed against local Isaac Sim 5.0 install before authoring. |
| T-G03-02 | Spoofing | Raw publisher with arbitrary frame_id | accept | frame_ids are constrained to the 17-entry override list — no input-driven values. |
| T-G03-03 | DoS | 17 additional OGN nodes per tick | mitigate | Same OnPlaybackTick as existing TF graph; rate-bound by sim tick. Spot-check `ros2 topic hz /tf` post-fix. |
| T-G03-04 | Information disclosure | Robot/gripper/camera frames in /tf | accept | Same trust as Phase 1 baseline — these frames are required for controller parity. |
</threat_model>

<verification>
diff_tf_tree.py reports zero diff (or only `view_frames Result` legend artifact) against `aic_frames_live.gv`. tf2_echo on representative slash-form frames returns valid transforms.
</verification>

<success_criteria>
- 17 Raw publisher chains in extension.py (16 prim-backed + 1 static aic_world).
- 14 MATCH frames continue via existing publishers — no regression.
- `diff_tf_tree.py` returns "PASS" or zero diff in human-verify Task 2.
- CHANGELOG + REQUIREMENTS.md updated with verdict.
</success_criteria>

<output>
After completion, create `.planning/phases/01-foundation-parity/01-G03-SUMMARY.md` with the override count, the diff_tf_tree.py post-fix output, and any frames that needed iteration.
</output>
