# Phase 3: Cable Physics + Object TF + Per-Object PARITY — Context

**Gathered:** 2026-05-05
**Status:** Ready for planning (autonomous M1 mode — decisions locked by Claude per project policy)

## Phase Boundary

Phase 3 delivers:
- **SCENE-02** Cable type variants (`sfp_sc_cable`, `sfp_sc_cable_reversed`)
- **SCENE-03** `attach_cable_to_gripper:=true` semantics + `gripper_initial_pos` per cable type
- **SCENE-05** Cable physics — per-link mass/inertia authoring per NVIDIA `RigidBodyRopeDemo.py` template
- **SCENE-06** Object TF for CheatCode `lookup_transform` calls (cable link poses, port frames in base_link)
- **PARITY-07** `/scoring/insertion_event` (`std_msgs/String`) — publish when plug-port insertion completes
- **PARITY-08** `/scoring/tf` (`tf2_msgs/TFMessage`) — cable link poses

Out of scope: trial loader (Phase 4), end-to-end aic_engine integration (Phase 4).

## Implementation Decisions

### D-01 — Cable physics strategy (SCENE-05)
- **Approach:** In-place USD edit (D-06 policy) authoring per-link `UsdPhysics.MassAPI(mass + diagonalInertia)` on the existing 21-segment cable rope chain at `/World/UR5e/cable`. Mirror NVIDIA's `RigidBodyRopeDemo.py` template — `MassAPI.CreateDensityAttr(0.00005)` per link gives realistic flexible-cable behavior under PhysX.
- **NOT research-gated** — per CLAUDE.md SCENE-05 update: empirical re-test 2026-05-03 disproved D-04 (cable wedge no longer reproduces under current launch path with venv-activate + warm cache + postload script).
- **Cable subtree activation:** flip `cable_prim.SetActive(False)` → `SetActive(True)` in `load_robot` once mass/inertia are populated. Test: cable should bend under gravity, plug end is grabbable.
- **Fallback if wedge returns:** rigid-plug-with-visual-cable hybrid (D-04 evolution). Diagnostic probes already exist at `exts/aic-dt/scripts/probe_cable_wedge.py` + `probe_cable_behavior.py`.

### D-02 — Cable type variants (SCENE-02)
- **Both variants use the same vendored cable USD** (`exts/aic-dt/assets/...` cable subtree). Difference is initial pose only — `sfp_sc_cable_reversed` = same USD with `cable_yaw + π` (180° flipped).
- `cable_type` parameter on `load_robot` (string, default `"sfp_sc_cable"`). Single switch.

### D-03 — Gripper attach (SCENE-03)
- **Authoring:** when `attach_cable_to_gripper=True`, author a `UsdPhysics.FixedJoint` at `/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l/CableAttachJoint` connecting the gripper finger link to the cable's plug-end link.
- **Plug-end link path:** detect at runtime — the cable USD's last segment's `*_plug_*` link prim. For `sfp_sc_cable`: typically `/World/UR5e/cable/sc_plug_link`.
- **Gripper opening pose:** `gripper_initial_pos` parameter on `load_robot` (float, default `0.00655`; pass `0.0073` for `sfp_sc_cable` per Gazebo). Apply via `Articulation.set_joint_positions` on `gripper/left_finger_joint` (FixedJoint zero-DOF in unified USD — actually NO-OP; documented; gripper opening lives in articulation drives elsewhere).
- **Atom:** new MCP atom `attach_cable_to_gripper` (4 surfaces per DX-02). Idempotent (deletes prior CableAttachJoint before authoring new one).

### D-04 — Object TF (SCENE-06)
- **Surface:** ADDITIONAL TF edges added to existing `parity_publishers.py` `_TF_EDGES` list. No separate publisher class.
- **Edges to add (per CheatCode):**
  - `world → {cable_name}/{plug_name}_link` — plug pose in world
  - `world → {cable_name}/{port_name}_link` — port pose in world (port = task_board's NIC card SC port)
  - `base_link → {cable_name}/{plug_name}_link` — derived edge for CheatCode L105 lookup
- **Frame names:** literal `cable/sc_plug_link`, `cable/sc_port_link` etc. — match what CheatCode requests verbatim. Source: live `aic_eval` container TF tree dump (capture as deliverable in Plan 03-01).
- **Source-of-truth:** USD prim world transforms via `UsdGeom.XformCache` (same pattern parity_publishers already uses for `_world_xform`).

### D-05 — Insertion event (PARITY-07)
- **Surface:** new `aic_dt_scoring_publisher` rclpy node OR add to `parity_publishers`. **Decision: add to `parity_publishers`** — single rclpy node per extension reduces handle count; topic logically lives next to `/scoring/tf`.
- **Trigger:** omni.physx contact-report subscription (mirror PARITY-06 pattern in `controller_loop._setup_contact_subscription`) filtered for plug-link ↔ port-link contacts.
- **Detection logic:** when contact pair matches (plug_end_link, port_link) and contact normal magnitude > 0 sustained for 5 ticks (~80ms) → publish `std_msgs/String` with payload format matching live aic_eval (capture format in Plan 03-01).
- **One-shot per insertion** — don't spam. Re-arm after plug separates from port.

### D-06 — /scoring/tf (PARITY-08)
- **Content:** `tf2_msgs/TFMessage` containing ONLY cable link transforms (separate from main `/tf` which carries robot kinematic chain).
- **Publisher:** add to `parity_publishers` alongside SCENE-06 TF additions. Same physics-tick callback.
- **Frame parent:** `world` (root frame). Each cable link is a separate transform.
- **Rate:** 60Hz (matches main /tf publisher cadence).

### Claude's Discretion
- Per-cable-link TF edge enumeration (auto-discover via traversal of activated cable subtree).
- Insertion-event payload format (mirror what live `aic_eval` publishes — capture in Plan 03-01).
- Whether to pull `attach_cable_to_gripper` semantic into `quick_start` chain by default (recommend: yes — `quick_start` already calls `load_robot` with cable params; add `attach_cable_to_gripper=True` to default kwargs).

## Specific Ideas

- **NVIDIA `RigidBodyRopeDemo.py` template** is the canonical per-link mass/inertia pattern — find and read via `nvidia-suite-docs` skill before authoring SCENE-05.
- **Plug-port contact pair filter** — mirror Plan 02-06's PARITY-06 prefix-startswith pattern, but specialize for plug↔port match (NOT a generic off-limit list).
- **`/scoring/tf` cable frames** — capture from live `aic_eval` container before authoring. The literal frame names CheatCode reads are non-negotiable.

## Canonical References

**Downstream agents MUST read these before planning or implementing:**

- `~/.claude/skills/nvidia-suite-docs/SKILL.md` — meta-router for live NVIDIA docs (Isaac Sim 5.0 + OpenUSD + OmniGraph). MANDATORY for SCENE-05 cable physics.
- `~/.claude/skills/isaac-sim-extension-dev/SKILL.md` — project-specific patterns (lazy Articulation construction, OGN race precedent, cable workaround history).
- `~/.claude/skills/robot-collision-forensics/SKILL.md` — omni.physx contact-report subscription pattern (Plan 02-06 precedent for PARITY-07).
- `exts/aic-dt/aic_dt/parity_publishers.py` — Phase 1 sibling implementation (will be EXTENDED by Phase 3 PARITY-08 + SCENE-06).
- `exts/aic-dt/aic_dt/controller_loop.py` Plan 02-06 `_setup_contact_subscription` — PARITY-07 contact-event template.
- `exts/aic-dt/scripts/probe_cable_wedge.py` + `probe_cable_behavior.py` — diagnostic probes to confirm SCENE-05 fix doesn't reintroduce the wedge.
- `~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro` — URDF source-of-truth for cable + gripper joint values.
- Live `aic_eval` Docker container — must capture `/scoring/tf` and `/scoring/insertion_event` formats BEFORE Plan 03-04/05 (PARITY-07/08).

## Surface Adjacency Pull-Forward Check

Per CLAUDE.md "Phase scope by surface" rule, scanning Phase 4 for pull-forward candidates:

- **Phase 4 trial loader** — separate surface (config parsing, trial sequencing). NOT pulling forward.
- **Phase 4 E2E aic_engine bringup** — depends on Phase 3 closure. NOT pulling forward.

No pull-forward this phase.

## Risk Map

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| SCENE-05 cable mass/inertia reintroduces post-play wedge | Low (D-04 update disproved this) | High (blocks PARITY-08 + insertion event) | Diagnostic probes pre-authoring; rigid-plug fallback if wedge returns |
| `/scoring/tf` frame names differ from live `aic_eval` | Medium | High (CheatCode `lookup_transform` fails) | Plan 03-01 captures live container snapshot before authoring |
| `attach_cable_to_gripper` FixedJoint authoring breaks articulation tensor view | Low | Medium | Mirror `cable_prim.SetActive(False)` precedent — toggle, test, snapshot cache |

## Estimated Effort

- Plan 03-01: live aic_eval snapshot + cable inspection (1h)
- Plan 03-02: SCENE-05 cable mass/inertia authoring + activation (3-4h)
- Plan 03-03: SCENE-02/03 cable type + attach (2h)
- Plan 03-04: SCENE-06 + PARITY-08 object TF additions (2h)
- Plan 03-05: PARITY-07 insertion event publisher (2h)
- Plan 03-06: smoke test + closure (1h)

**Total: ~11-12h** (vs CLAUDE.md's "~3-4hr for SCENE-05 alone" — that was just the cable physics piece).

---

_Authored autonomously per autonomous M1 mode policy. All gray areas resolved with default decisions; recoverable if a plan-time discovery invalidates one (per CLAUDE.md "deviations are documented in plan SUMMARY")._
