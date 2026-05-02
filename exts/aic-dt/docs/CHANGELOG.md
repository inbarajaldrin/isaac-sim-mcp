# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).


## [Phase 1: Foundation Parity] — 2026-05-02

Milestone 1 (M1) toward platform-transfer parity with the AIC competition Gazebo
stack. The unmodified `aic_controller` + `aic_engine` + `aic_example_policies/ros/CheatCode.py`
from `~/Documents/aic` will run against this Isaac Sim digital twin without code
changes (M1 success bar). Phase 1 lands the passive-sensor surface (TF + JointState +
wrench + cameras), aligns asset paths with AIC's vendored layout, fixes the
articulation prim-path bug that silently dropped joint drives, retires the
`_sim`/`_real` placeholder topic suffixes, vendors all task-board assets locally,
and ships a per-component scene-population API plus Gazebo-named robot/cable
pose parameters. After this milestone, `quick_start` emits the full Phase 1
sensor surface in one MCP call.

Requirement IDs delivered: PARITY-01, PARITY-02, PARITY-03, PARITY-04, PARITY-05,
PARITY-12, TEX-01, TEX-02, TEX-03, SCENE-01, SCENE-04, DX-01, DX-02.

### Added

- **TF publisher (PARITY-04, D-10)** — New MCP atom `setup_tf_publisher` (registry +
  handler-map + `_cmd_setup_tf_publisher` + UI button) builds an OmniGraph with two
  `isaacsim.ros2.bridge.ROS2PublishTransformTree` nodes — one for `/tf` (dynamic)
  and one for `/tf_static` (TRANSIENT_LOCAL via `staticPublisher=True`). Articulation
  root targets `/World/UR5e/aic_unified_robot`. Late-joining subscribers (CheatCode,
  aic_engine) now see static frames immediately on subscribe.
- **JointState publisher (PARITY-03, D-11)** — New MCP atom
  `setup_joint_state_publisher` (4 surfaces) builds an OmniGraph with
  `isaacsim.ros2.bridge.ROS2PublishJointState` publishing `sensor_msgs/JointState` on
  `/joint_states`. Live `aic_eval` reference: 7 joints (incl. `gripper/left_finger_joint`)
  with `header.frame_id=base_link`. Plan 05 probe verdict (`NAME-INDEXED via
  aic_adapter::ReorderJointState`) confirmed Isaac Sim's natural articulation order
  is acceptable on the wire — `aic_adapter` reorders by name on receipt.
- **Live `aic_eval` topic-parity reference (PARITY-12, D-01, D-14)** —
  `exts/aic-dt/scripts/snapshot_aic_eval.sh` + `exts/aic-dt/docs/topic-parity-reference.md`.
  Image SHA-256 digest pinned per D-14 (`sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433`).
  Live surface enumerated: 36 topics on the running `aic_eval` container.
- **Phase 1 verify harness (DX-01, D-15)** — `exts/aic-dt/scripts/verify_phase_1.sh`.
  Hybrid-runtime (attached or cold-launch via `launch_postload.py`); integrates
  `prime_usd_cache.py` cache discipline (per CLAUDE.md). Companion script
  `diff_tf_tree.py` (D-08) for `view_frames` `.gv` diff against live Gazebo.
- **Texture sweep tooling (TEX-03, D-07)** — `exts/aic-dt/scripts/sweep_textures.py`
  + `exts/aic-dt/docs/texture-sweep.md` (log-grep loop; iterative). Plus the
  pre-graph USD probe pattern (`probe_unified_usd.py`, `probe_root_joint.py`) used
  by Plans 05 and 06 to verify USD/PhysX assumptions before authoring OmniGraphs.
- **Per-component scene-population MCP atoms (SCENE-01)** — 7 new atoms with full
  4-surface coverage:
    - `spawn_task_board_base`
    - `spawn_lc_mount_rail`
    - `spawn_sfp_mount_rail`
    - `spawn_sc_mount_rail`
    - `spawn_sc_port`
    - `spawn_nic_card_mount`
    - `spawn_nic_card`

  Each can be invoked individually with explicit pose kwargs; collectively clubbed
  by `add_objects` and `quick_start`. Default values match the AIC bringup launch
  files (`spawn_task_board.launch.py`).
- **SCENE-04 robot + cable pose kwargs on `load_robot`** — 12 Gazebo-named
  parameters (`robot_x/y/z/roll/pitch/yaw` + `cable_x/y/z/roll/pitch/yaw`) wired
  through `load_robot` + `_cmd_load_robot` + `MCP_TOOL_REGISTRY` schema.
  None-sentinel defaults preserve backwards compatibility for the no-arg
  `quick_start` path; explicit kwargs override. Cable defaults match
  `aic_gz_bringup.launch.py` literals (`cable_x=0.172, cable_y=0.024, cable_z=1.518,
  cable_roll=0.4432, cable_pitch=-0.48, cable_yaw=1.3303`); cable subtree
  remains `SetActive(False)` per D-04 — pose surface is wired, physics is Phase 3.
- **Mount-rail thin-USD wrappers** — `build_mount_rail_usds.py` authors one-line
  payload USDs that reference upstream `.glb` meshes via relative `AddReference`,
  for the LC / SFP / SC Mount components that ship as glTF (no pre-cooked USDs)
  in `aic_assets/models/`.

### Changed

- **Robot reload prim-path bug fix (PARITY-01)** — `load_robot` now looks up joint
  drives at `/World/UR5e/aic_unified_robot/joints/<name>` (was
  `/World/UR5e/joints/<name>`). The unified USD's default prim is `/World`, so the
  actual joints live under `aic_unified_robot/`. Joint drives now apply correctly;
  the long-standing 6× "Joint not found" Kit-log warnings (visible since the
  unified USD was adopted) are resolved. Forward-declared
  `self._articulation_root_prim_path = "/World/UR5e/aic_unified_robot"` so the
  JointState publisher's `targetPrim` relationship has a single source of truth.
- **Asset paths updated to AIC's capitalized layout (PARITY-02, TEX-01, TEX-02, D-05)**
  — `AIC_OBJECTS` snake_case (`objects/sc_port/sc_port.usd`) → AIC capitalized
  (`assets/SC Port/sc_port_visual.usd`). Sibling `textures/` folders now present;
  pink/black/missing-texture cases driven by the missing sibling-texture path are
  resolved. `_local_asset` resolver unchanged; only the dict values changed.
- **Asset vendoring (D-05, D-13)** — One-time `cp -r` vendoring of
  `Intrinsic_assets/<Object>/` subset into `exts/aic-dt/assets/assets/<Capitalized>/`
  with sibling `textures/` directories. 5 capitalized AIC asset folders (`NIC Card`,
  `NIC Card Mount`, `SC Port`, `SC Plug`, `Task Board Base`) — 21 files total
  (9 USDs + 12 textures), md5sum byte-identical to upstream. Snake_case
  `assets/objects/` tree retired in Plan 02. After Phase 1, the AIC repo is the
  documented origin but not a runtime dependency.
- **`quick_start` reordered per D-12** — Now calls `setup_tf_publish_action_graph`
  and `setup_joint_state_publish_action_graph` after the early-play step and
  before `setup_action_graph` (joint-state subscribe). Adds best-effort
  `randomize_lighting` step (hasattr-guarded). Documented common-path order;
  future phases drop new atoms in via this path. Early-play step preserved
  (PhysX cooking + lock-contention rationale). Defensive `hasattr` guard on
  optional `setup_joint_state_reorder` bridge — Plan 05 verdict was
  NO-WRAPPER-NEEDED (`aic_adapter` reorders by name on receipt) so the method
  typically doesn't exist; the guard is forward-compatible insurance.
- **Robot identification — RG2 → Robotiq Hand-E (D-03)** — Confirmed via AIC's
  `aic_description/urdf/ur_gz.urdf.xacro` (`xacro:include` of
  `Robotiq Hand-E/robotiq_hande_macro.xacro`). Code (extension.py docstrings,
  log lines, return messages) and docs (`PROJECT.md`, `REQUIREMENTS.md`,
  `ROADMAP.md`, `CLAUDE.md`, `docs/README.md`) updated. Zero standalone "RG2"
  identifiers remain on the production doc surface.
- **`/tf_static` QoS = TRANSIENT_LOCAL** — `staticPublisher=True` on the static
  `ROS2PublishTransformTree` node. CheatCode + aic_engine see static frames on
  subscribe, not just on republish.
- **Camera topic names match live `aic_eval` (PARITY-05, DX-01, D-09)** — `_sim`
  suffix removed; topics now `/center_camera/image`, `/left_camera/image`,
  `/right_camera/image` (with `/camera_info` siblings).
- **Wrench topic name matches live `aic_eval` (PARITY-05, DX-01)** —
  `force_torque_sensor_broadcaster/wrench_sim` → `fts_broadcaster/wrench`.
  `frame_id` set to `ati/tool_link` (sourced from
  `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro:242`,
  `AtiForceTorqueSensor` `param name="frame_id"`).
- **`add_objects` refactored to club per-component atoms (SCENE-01)** — Additive
  refactor (not replacement). Existing legacy `/World/Objects` path (physics
  material binding, two-pass loading, randomization caching) preserved verbatim;
  the new per-component atoms are invoked best-effort BEFORE the legacy path
  inside a try/except. Decoupled — the new-atom path can fail (e.g. asset not
  vendored) without breaking the canonical scene-population code that
  `quick_start` step 7 depends on.

### Removed

- **`setup_pose_publisher` MCP atom (DX-01, D-09)** — Placeholder bridge from the
  pre-M1 era. Phase 3 publishes GT directly into `/tf` (SCENE-06). All 4 surfaces
  deleted: registry / handler-map / `_cmd_*` method / UI button.
- **`sync_real_poses` MCP atom (DX-01, D-09)** — Placeholder
  `/objects_poses_real` subscriber. All 4 surfaces deleted. Plus the orphan
  `quick_start` caller scrubbed.
- **All `_sim` / `_real` topic suffixes on production surface** — 33-row rename
  table in `01-RESEARCH.md` lines 731-766. Zero matches remain in `extension.py`
  after this milestone (verified by
  `grep -E '(_sim|_real)\b' extension.py | grep -v initialize_simulation_context_async`).
- **Snake_case `assets/objects/` tree** — Replaced by capitalized
  `assets/assets/` per D-05.

### Known Phase-3 work items

- **Cable subtree `SetActive(False)` workaround (D-04)** — Persists for Phase 1.
  Cable physics is Phase 3 (SCENE-05). Cable assets ARE inspected for textures
  during the Phase 1 sweep; only physics simulation is disabled. SCENE-04 cable
  pose params are wired through `ClearXformOpOrder` + `AddTranslateOp` +
  `AddRotateXYZOp` so the parameter SURFACE is in place; Phase 3 enables physics
  at which point those defaults take effect without further code changes.
- **Object TF frames CheatCode reads (`{cable_name}/{plug_name}_link`, port
  frames)** — Phase 3 (SCENE-06).
- **Frame name mapping for slashed AIC frame names (`gripper/hande_base_link`,
  etc.)** — Plan 05's `usd_prim_inventory.txt` enumerates 17 underscore→slash
  override entries; Plan 06 deferred per-frame `ROS2PublishRawTransformTree`
  overrides to a follow-up plan. The default-mapping initial deploy will likely
  fail PARITY-04's `view_frames` zero-diff assertion in Phase 1 verification —
  expected and tracked.
- **JointState slashed name (`gripper/left_finger_joint`)** — Isaac Sim's
  `ROS2PublishJointState` has no `jointNames` / `nameOverrides` input (verified
  against the OGN .rst spec in Plan 06). Initial deploy publishes the USD prim
  leaf name (`gripper_left_finger_joint`); `aic_adapter` will log "Ignoring
  unexpected joint name" and silently drop it from observations. Follow-up
  required.

### DX-02 4-surface contract — final audit (Phase 1 ship)

Each MCP atom requires all 4 surfaces in lockstep: registry entry, handler-map
entry, `_cmd_<name>` method, and UI button. Deleted atoms must have all 4
surfaces removed. Phase 1 net delta: 9 atoms added (28 net new surfaces +
3 expected via existing patterns), 2 atoms removed (8 surfaces gone).

| Atom | Registry | Handler-map | `_cmd_*` | UI button | Status |
|------|----------|-------------|----------|-----------|--------|
| setup_tf_publisher | yes | yes | yes | yes | PRESENT (new, PARITY-04) |
| setup_joint_state_publisher | yes | yes | yes | yes | PRESENT (new, PARITY-03) |
| spawn_task_board_base | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_lc_mount_rail | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_sfp_mount_rail | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_sc_mount_rail | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_sc_port | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_nic_card_mount | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| spawn_nic_card | yes | yes | yes | yes | PRESENT (new, SCENE-01) |
| setup_pose_publisher | no | no | no | no | DELETED (DX-01) |
| sync_real_poses | no | no | no | no | DELETED (DX-01) |

A standalone audit script (`exts/aic-dt/scripts/audit_dx02.py`) enumerates every
atom in `MCP_TOOL_REGISTRY` and verifies the 4-surface invariant; expected
present and expected absent atoms are listed inline in the script and exercised
on every Phase verifier run.

### Source materials

- AIC repo (read-only): `~/Documents/aic` —
  `aic_controller/`, `aic_engine/`, `aic_adapter/`, `aic_example_policies/ros/`
  (`CheatCode.py`, `RunACT.py`, etc.), `aic_description/urdf/ur_gz.urdf.xacro`,
  `aic_assets/`, `Intrinsic_assets/`.
- Live `aic_eval` Docker container: `ghcr.io/intrinsic-dev/aic/aic_eval:latest`
  (digest pinned in `topic-parity-reference.md`).
- Isaac Sim 5.0 install at `~/env_isaaclab/`. Probe interpreter is
  `~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh` (cp310 + bundled `pxr`).

### Phase summaries

Per-plan summaries live under `.planning/phases/01-foundation-parity/`:
`01-01-SUMMARY.md` through `01-09-SUMMARY.md` (Plans 01-01 through 01-09;
Plan 08 is this milestone integration plan).


## [1.0.0] - 2021-04-26
- Initial version of extension UI template with a window
