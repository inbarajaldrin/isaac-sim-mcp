# Scene-Fidelity Pipeline (Gazebo → Isaac Sim)

How the AIC scene is authored in Isaac Sim to mirror the Gazebo Ionic
reference, what compromises are documented, and how to extend it.

## Source-truth authoring principle

Every per-material value (diffuse, metallic, roughness, emissive, opacity)
in the on-disk USDCs **must mirror the upstream GLB JSON verbatim**. The
NVIDIA `omni.kit.asset_converter` is not a faithful transcriber — it has
been observed inventing values (notably `emissive_strength=625.0` on a
material whose source GLB only has `emissiveFactor=(1,1,1)` with no
`KHR_materials_emissive_strength` extension), and applying glTF defaults
inconsistently. **Always parse the source GLB before authoring any
material override:**

```bash
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \
    exts/aic-dt/scripts/parse_glb_materials.py \
    /home/aaugus11/Documents/aic/aic_assets/models/Enclosure/enclosure_visual.glb \
    /home/aaugus11/Documents/aic/aic_assets/models/Floor/walls_visual.glb
```

The output applies glTF 2.0 spec defaults explicitly (e.g. `metallicFactor`
defaults to **1.0** when absent — counterintuitive, but per spec). Use the
output as the truth table. Every divergence in a bake script must be cited
in a code comment.

Skill-level documentation: `~/.claude/skills/isaac-sim-extension-dev/SKILL.md`
section "Source-truth material authoring — parse the GLB, NEVER trust the
converter MDL".

## Bake scripts inventory

All offline scripts run via the Isaac-Sim-bundled pxr Python
(`~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh`) and modify the split
USDCs idempotently. After editing, re-run + `new_stage` + `load_scene` to
pick up changes.

| Script | Reads | Writes (in `assets/`) |
|---|---|---|
| `parse_glb_materials.py` | source GLBs (`aic_assets/models/...`) | stdout JSON (no file writes) |
| `bake_all_material_surfaces.py` | `enclosure_split.usdc`, `walls_split.usdc` | both — wires `UsdPreviewSurface` (`<Material>/UPS_Auto`) using `GAZEBO_PARITY_OVERRIDES` table (currently source-true) |
| `bake_walls_emissive.py` | `walls_split.usdc` | adds `Material_003/Emissive_UPS` + sets `Plane_003_Plane_003.doubleSided=true` so the warehouse ceiling emits into the booth |
| `bake_floor_material.py` | `floor_split.usdc` | wires matte `UsdPreviewSurface` on `Material_001` with `clean-concrete_albedo.png` referenced via `UsdPrimvarReader_float2 → UsdUVTexture → diffuseColor` |
| `consolidate_gray75_strips.py` | `enclosure_split.usdc` | merges `V1014952_001_GRAY75_FLOOR + _MID + _CEILING` into a single `V1014952_001_GRAY75` mesh; removes any orphan `GRAY75_CEILING_TRANSLUCENT` material |

The Z-stratified `FLOOR/MID/CEILING` split came from an earlier session's
splitter pipeline (selective-hiding use-case, since abandoned). The
`consolidate_gray75_strips.py` script makes the on-disk structure mirror
the source GLB (one mesh primitive per material), restoring architectural
cleanliness. It is idempotent — re-running on already-consolidated state
is a no-op.

## Lighting setup (`extension.py::_setup_aic_lights`)

| Prim | Type | Position | Intensity | Purpose |
|---|---|---|---|---|
| `/World/Lights/enclosure_light` | SphereLight + ShapingAPI cone | `(0, 0, 2.5)` | 15000 | Mirrors `aic.sdf` spot light `enclosure_light`; lights the booth interior from inside the top |
| `/World/Lights/ambient_dome` | DomeLight | (no transform) | 300 | Soft ambient fill — without this RTX has no broad-fill contribution and corners collapse to black |
| `/World/Lights/enclosure_panel_rect` | RectLight | `(0, 0, 2.5)`, 1.4×1.4 m, emits `-Z` | 3000 | Proxy for "ceiling panel" interior fill so the workspace area reads as diffuse-overhead-lit, not point-source |
| `/World/Lights/ceiling_01` | SphereLight | `(+2, +2, 6.0)` | 2500 | Mirrors `aic.sdf` warehouse point light |
| `/World/Lights/ceiling_02` | SphereLight | `(-2, -2, 6.0)` | 2500 | Mirrors `aic.sdf` warehouse point light |
| `/World/Lights/ceiling_panel_rect` | RectLight | `(0, 0, 7.048)`, 9.43×9.43 m, emits `-Z` | 6000 | **Light-emission proxy for the warehouse `Material_003` emissive ceiling** (see below) |

### The warehouse-ceiling emissive proxy — important

The source GLB material `Material_003` (warehouse ceiling) has
`emissiveFactor=(1,1,1)` with no extensions. In Gazebo's ogre2, emissive
geometry contributes light into the scene. In Isaac Sim **RaytracedLighting
mode**, emissive surfaces render bright pixels but **do not propagate
light** — only PathTracing mode does. So just baking the emissive material
into `walls_split.usdc` (via `bake_walls_emissive.py`) makes the ceiling
LOOK emissive in the viewport, but the booth interior gets no benefit from
that emission.

To get the emission's lighting contribution under RaytracedLighting, we
author a `UsdLux.RectLight` at the same position and area as the ceiling
panel (`9.43×9.43 m` at `z=7.048`, emitting `-Z` straight down). This
RectLight acts as the actual light source for RTX; the baked emissive
material on the ceiling geometry provides the visual cue of the ceiling
glowing. Both must exist together:

- **Mesh emissive only** → ceiling looks bright but workspace stays dark
- **RectLight only** → workspace is lit but the ceiling looks dark/unlit
- **Both** → ceiling glows + workspace is lit (the Gazebo look)

If/when you switch the viewport to PathTracing (`/rtx/rendermode = PathTracing`),
the RectLight becomes redundant — path tracing converges emissive
contribution correctly. Until then, keep both.

## Renderer compensations (RTX-vs-ogre2)

Authored in `extension.py::load_scene()` settings block:

| Carb setting | Default | Set to | Why |
|---|---|---|---|
| `/rtx/ambientOcclusion/rayLength` | 35.0 m | 1.0 m | Default ray length escapes interior geometry → no crevice darkening. 1.0 m was the sweet-spot per A/B sweep at 0.3, 0.7, 1.0, 1.5, 2.0, 35 m. |
| `/rtx/post/tonemap/filmIso` | 100 | 81.2 | -0.3 EV exposure drop; RTX over-exposes whites vs Gazebo. Preferred over -0.7 EV in head-to-head A/B. |
| `/rtx/sceneDb/ambientLightColor` | (0.1, 0.1, 0.1) | (0, 0, 0) | Default residual ambient floods corners with gray fill and flattens contrast. Preferred zeroed in A/B. |
| `/rtx/indirectDiffuse/maxBounces` | 2 | 4 | More indirect-light bounces = more interreflection between adjacent surfaces (back wall lit by reflected floor light, etc.) closer to Gazebo's "soft falloff" character. |

Also: viewport switched from the default Kit "Default" light rig to **Stage
Lights mode** (`_set_lighting_mode("Stage Lights")`). The Default rig hides
all scene-authored `UsdLux` prims via `VisibilityEdit`, so without this
mode switch the `/World/Lights/*` prims would be authored-but-suppressed.

## Known parity gaps (RaytracedLighting plateau)

After all the above, GPT side-by-side reads against the Gazebo reference
(see chat history 2026-05-13) settle at "darker interpretation of the same
scene." Specifically:

- WHITE-bound surfaces (frame/posts/walls) render noticeably darker in
  Isaac than Gazebo. Root cause: RTX renders source-true metallic=1.0 +
  diffuse=0.13 as near-black (PBR metallic has ~0 diffuse contribution +
  `/rtx/reflections/maxRoughness=0.3` default skips reflections for
  roughness>0.3). Gazebo's ogre2 doesn't apply the same metallic-PBR
  clamp.
- GRAY75 panels render slightly darker / flatter than Gazebo.
- WHITE-vs-GRAY75 distinction is compressed — they sit closer together
  than Gazebo's intended separation.
- Overall character matches; tonal levels do not.

These gaps are renderer-pipeline differences, not authoring divergences.
Future closing-the-gap work options (in increasing complexity):

1. **HDR environment dome texture** — replace the flat-color DomeLight
   with a warehouse-interior HDRI so metallic surfaces have a real bright
   environment to reflect.
2. **PathTracing mode** for snapshot/replay renders — physically-correct
   but slow + firefly tradeoff. Acceptable for marketing screenshots, not
   real-time simulation.
3. **Documented renderer-compensation overrides** — re-introduce a runtime
   compensation table in `extension.py::import_enclosure()` that lifts
   WHITE diffuse from 0.13 → ~0.30 (matte, metallic=0) while keeping the
   USDC source-true. The previous architecture had this; it was removed
   in favor of pure source-true when option (ii) was chosen 2026-05-13.

## Quick verify after any change

```bash
# Re-bake all material authoring (idempotent)
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/bake_all_material_surfaces.py
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/bake_walls_emissive.py
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/bake_floor_material.py
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh exts/aic-dt/scripts/consolidate_gray75_strips.py

# Hot-reload extension + clean stage + reload
touch exts/aic-dt/aic_dt/extension.py
python3 -c "
import socket, json
s = socket.socket(); s.connect(('localhost', 8768))
s.sendall(json.dumps({'type':'new_stage','params':{}}).encode())
... # See exts/aic-dt/scripts/mcp_test.py for the framing boilerplate
"

# Capture viewport for compare
bash ~/.claude/skills/isaac-sim-extension-dev/scripts/viewport_screenshot.sh isaacsim /tmp/iso.png
```
