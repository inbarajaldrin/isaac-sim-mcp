# Agent Handoff — AIC Digital Twin Scene-Fidelity Pass

You are picking up an interactive HITL pair-programming session focused on closing scene-fidelity divergences between **Gazebo** (the AIC competition reference simulator) and **Isaac Sim** (the digital twin running the aic-dt extension).

## Orient yourself FIRST — read these in order before asking anything

1. `~/Documents/isaac-sim-mcp/CLAUDE.md` — top-of-file autonomous-mode block + cross-repo relationship rules
2. `~/Documents/isaac-sim-mcp/exts/aic-dt/docs/component-map/index.html` — **the master visual reference**. 10 tabs (Overview / Architecture / Isaac Sim Tree / Gazebo Tree / Scene Inventory / Topic Surface / Submission / Audit Findings / Live Kit UI / Tools). Open it via the HTTP server already running at `http://100.97.45.92:8090/index.html` (Tailscale). Pay special attention to **Isaac Sim Tree** + **Gazebo Tree** tabs — they are interactive D3 trees showing the parallel scene hierarchies
3. `~/Documents/isaac-sim-mcp/exts/aic-dt/docs/component-map/resources/aic_object_inventory.json` — full 49-entry inventory; the `known_divergences` field on each entry flags Isaac-vs-Gazebo gaps
4. `~/Documents/isaac-sim-mcp/exts/aic-dt/docs/component-map/resources/aic_eval_cycle.md` — competition eval cycle, scoring tiers, the **ground-truth → CV pose-estimation seam at CheatCode.py:217-223**, submission contract
5. `~/Documents/aic/CLAUDE.md` and `~/Documents/aic/.planning/PROJECT.md` — the user's broader project is `aic_vision`, a 6D pose-estimation platform; the digital twin work feeds the CV pipeline that replaces CheatCode's ground-truth lookup

## What is already set up — DO NOT rebuild

### Running infrastructure
- **Isaac Sim** is launched via the canonical zenoh-envelope command (see CLAUDE.md `Launching the Isaac Sim aic-dt extension`). MCP socket at `localhost:8768`. Don't restart unless you have to — restarts are ~30s each
- **`component_map_server.py`** is running on port 8090 — custom HTTP+MCP proxy server that serves the component map AND forwards `/api/mcp` POST requests to the MCP socket. Used by the **Live Kit UI** tab in the HTML
- **`zenohd`** router on port 7447 — bridges humble↔kilted RMW for engine↔model RPC

### Scripts (all in `exts/aic-dt/scripts/`)
- `build_component_map.py` — regenerates the HTML component map at `docs/component-map/index.html` from current AIC files. Re-run after any change to `extension.py` or the resources
- `scene_divergence.py` — meta-script auditing live Isaac Sim stage against Gazebo source-of-truth. 3 sections: metersPerUnit consistency, vertex-sampled size vs GLB raw, world-pose drift. Section 1 has size-cross-check to suppress false positives. **Run after any scene change to verify no regression**
- `check_cad_dimensions.py` — per-asset 4-source dimension verifier (GLB JSON / Blender headless / live MCP / USD-direct). Use when adding/replacing CAD assets
- `component_map_server.py` — the HTTP+MCP proxy (already running)
- `run_aic_engine_against_isaac_sim.sh` — fires one or more trials end-to-end against Isaac Sim (use to verify scoring still works after a fix)

### Recent commits (this session)
- `1308759 feat(scene-fidelity): metersPerUnit helper + divergence audit + HTML component map`
- `0a12167 fix(scene): NIC card PCB visible on mount + orphan finger_r joint cleanup`
- `a31b3a1 ship(m1): Milestone 1 — Platform Transfer SHIPPED — all 3 trials pass insertion=true`

### Helper functions added to extension.py
- `_apply_meters_per_unit_scale(xform, usd_uri, stage)` — OPT-IN per-asset helper. Reads asset's metersPerUnit, computes scale ratio, authors TypeScale op. Some assets need it (e.g. nic_card_visual.usd, ~100x mismatch), some have internal compensation (e.g. mount rails) — divergence script tells you which

## What the user cares about specifically

The user is doing a HITL pass focused on **how scenes are loaded into Isaac Sim**. Three concrete concerns to track and cross off as you fix each:

1. **Object + asset loading correctness** — every Gazebo SDF model should have an Isaac Sim USD equivalent rendering at the right size, pose, and orientation. The Scene Tree tabs show parallel hierarchies; divergences are real bugs to fix
2. **The trial scene loading flow** — `_cmd_load_trial` in `extension.py` should produce a scene that is visually and behaviorally faithful to Gazebo's equivalent (`docker_run aic_eval gazebo_gui:=true` against the same `sample_config.yaml` trial). User wants the spawn pipeline to be predictable and correct
3. **Quick_start vs load_trial scope** — user wants `quick_start` to spawn ONLY what's common across trials (environment + robot + cable). Trial-specific entities (mount-rail occupancy, ports, cable attach) should ONLY come from `load_trial`. The current code blurs this — `quick_start` spawns objects via `add_objects` which overlaps with `load_trial`. **Untangle this**

## Outstanding divergences (from latest `scene_divergence.py` audit, after FP suppression)

Run `python3 exts/aic-dt/scripts/scene_divergence.py` to refresh — current state is 4 real FAILs:

| # | Location | Type | Severity rationale | Fix difficulty |
|---|---|---|---|---|
| 1 | `/World/TaskBoard/NICCardMount_0` | Section 1 mpu | Mount root has no scale fix → screw mesh renders at 18.5km (face-culled invisible). Real but cosmetic since user can't see it anyway | trivial: add `_apply_meters_per_unit_scale` call on the mount root in `_cmd_spawn_nic_card_mount` |
| 2 | `/World/UR5e/cable/sc_plug_visual` | Section 2 size | live Z=21mm vs GLB Z=10mm — 109% drift. Real geometry interpretation difference between USD and GLB authoring | medium: investigate the sc_plug_visual.usd and confirm if USD has extra geometry not in GLB |
| 3 | `/World/UR5e/cable/sfp_module_visual` | Section 2 size | live=74×27×15mm vs GLB=56×15×12mm — 81% drift. Same kind of authoring mismatch | medium: same investigation |
| 4 | `/World/TaskBoard/NICCardMount_0/nic_card_link` (PCB) | Section 2 size | live Z=21mm vs GLB Z=58mm — the USD may be missing depth geometry the GLB has | medium |

## Outstanding broader gaps (NOT in audit, also user-visible)

| Gap | Why it matters | Reference |
|---|---|---|
| **Cable rope-chain whipping** | 21-link chain has `stiffness=1.0 damping=10.0` d6 joints AND `density=5e-5` per-link mass → MHz natural frequency aliased to viewport refresh. Visually distracting; doesn't affect scoring (plug_proxy handles it) | Fix #2 in audit conversation; deliberate descope for M1 — fix is either (a) `kinematic_enabled=True` on rope links to freeze, (b) bump density to ~500 kg/m³ AND lower joint stiffness |
| **Cable's `sc_plug_visual` is 857mm from gripper** | Visual disconnect — cable end floats away from where the gripper is, plug_proxy at finger_link_l handles scoring instead. Looks wrong | Fix is non-trivial: cross-articulation FixedJoints are dropped by PhysX cook; would need a USD-hierarchy reparenting or kinematic body teleport |
| **NIC mount has no body geometry** | Mount renders only as a screw + PCB child. Gazebo's SDF has 11 collision boxes for the mount BODY but no visual mesh either. So both sims show "floating PCB + screw". Adding a synthetic bracket-shaped Cube would close the visual gap | New work: author a simple bracket primitive in `_cmd_spawn_nic_card_mount` |
| **Mount-body collision boxes missing** | Gazebo's mount has 11 box colliders for off-limit-contact detection. Tier-2 -24 point penalty per contact. CheatCode never touches them so it's invisible currently; a real CV policy hitting the mount sides loses points silently | Port the SDF collision boxes to UsdPhysics.CollisionAPI'd Cube children under NICCardMount_0 |
| **`/aic/gazebo/contacts/off_limit` topic not published by Isaac Sim** | Same penalty trap as above — engine subscribes; if Isaac Sim never publishes, the penalty never fires. CheatCode safe; CV policy hitting walls loses 24pts silently | New publisher needed. Mirror Gazebo's ContactSensor plugin behavior |
| **Wrist cameras broken** | M1's only remaining open PRD item AND the foundation for the user's `aic_vision` CV pipeline (which replaces CheatCode's `tf_buffer.lookup_transform` with image-based estimation). HIGHEST LEVERAGE | Currently `setup_wrist_cameras` exists but produces broken output. Needs investigation |
| **Trial_2 + trial_3 not re-verified** | We ran trial_1 successfully under the new metersPerUnit helper but haven't re-fired the other two. The M1 ship commit asserts all 3 pass | `bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_2 --output-json=/tmp/t2.json` |
| **ground_truth=False mode untested** | "Real submission" path. CheatCode-mode (gt=true) works; with gt=false, port frames disappear from /tf and CheatCode times out. A CV policy would estimate poses from cameras. Untested end-to-end | Try `load_trial(trial_1, ground_truth=False)` + fire engine + see what fails |

## Your operating instructions

1. **READ all 5 orientation docs first** — understand context before asking questions
2. **Open the HTML at `http://100.97.45.92:8090/index.html` from the user's perspective** (they're on a Mac via Tailscale). Click through every tab to know what's there. The Live Kit UI tab is your interactive control surface — you can fire MCP commands via `curl -X POST http://localhost:8090/api/mcp -d '{...}'`
3. **Then ASK the user**: present the divergence list (4 audit FAILs + 8 broader gaps) and ask which to fix in what order. Be explicit about effort/risk per item
4. **Fix ONE thing at a time**, verify with `scene_divergence.py` + a trial fire, commit atomically with descriptive message (NO `Co-Authored-By` line per global CLAUDE.md), then ASK what's next
5. **After each fix**: cross it off by regenerating the HTML component map (`python3 exts/aic-dt/scripts/build_component_map.py --no-open`) and confirming the audit section in the HTML reflects the new state
6. **Use the existing skills aggressively**: `isaac-sim-extension-dev` for project-specific Isaac Sim patterns, `nvidia-suite-docs` for live NVIDIA API questions. Both override training-data memory of stale 4.x APIs
7. **DO NOT** restart Isaac Sim unless you genuinely need new extension code loaded. Hot-edit + reload-extension pattern preferred
8. **DO NOT** rebuild the scripts in `exts/aic-dt/scripts/` — they work and are committed
9. **DO NOT** touch the cable physics deeper than the user authorizes — the Path-a' compromise is load-bearing for M1 scoring

## Quick-reference commands

```bash
# Check Isaac Sim alive
nc -z localhost 8768 && echo OPEN || echo CLOSED

# Verify component map server alive
curl -s http://localhost:8090/api/health

# Regenerate component map HTML
python3 ~/Documents/isaac-sim-mcp/exts/aic-dt/scripts/build_component_map.py --no-open

# Run divergence audit (ensure trial loaded first)
python3 ~/Documents/isaac-sim-mcp/exts/aic-dt/scripts/scene_divergence.py

# Load a trial via MCP (verifies a fix didn't break it)
curl -s -X POST http://localhost:8090/api/mcp -H 'Content-Type: application/json' \
  -d '{"type":"load_trial","params":{"trial_key":"trial_1","ground_truth":true}}'

# Fire a full trial end-to-end (against Isaac Sim) and check insertion_event_fired
bash ~/Documents/isaac-sim-mcp/exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1 --output-json=/tmp/t1.json
cat /tmp/t1.json

# Probe the live stage anything (Python via MCP)
curl -s -X POST http://localhost:8090/api/mcp -H 'Content-Type: application/json' \
  -d '{"type":"execute_python_code","params":{"code":"from pxr import Usd; import omni.usd; result = {\"prim_count\": sum(1 for _ in omni.usd.get_context().get_stage().Traverse())}"}}'
```

## YOUR FIRST MESSAGE

After reading the orientation docs, your first message to the user should be:

> I've oriented on the AIC digital twin work. Read CLAUDE.md, the inventory, the eval cycle research, and the HTML component map. Current state: M1 shipped (all 3 sample_config trials pass insertion_event=true), with N visible scene-fidelity divergences flagged by the audit + M broader gaps not in the audit.
>
> Here's the prioritized list of what could be fixed next, ordered by my read of leverage-vs-effort:
>   [render the 12-item list of audit FAILs + broader gaps here]
>
> Which do you want me to tackle first? Or different priority — name it.

After they pick, fix it, verify, commit, regen the component map, then ask "what's next?" — loop until they stop you.

---
*Generated from session 2026-05-12; component map server at http://100.97.45.92:8090.*
