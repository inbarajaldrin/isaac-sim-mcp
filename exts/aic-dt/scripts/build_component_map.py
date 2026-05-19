#!/usr/bin/env python3
"""
build_component_map.py — Generate the AIC component map HTML page.

Reads dynamically from:
  - /tmp/aic_object_inventory.json     (asset inventory; produced by audit agent)
  - /tmp/aic_eval_cycle.md             (eval-cycle research; produced by research agent)
  - ~/Documents/aic/.planning/PROJECT.md
  - ~/Documents/aic/CLAUDE.md
  - ~/Documents/aic/docs/*.md          (competition docs)
  - ~/Documents/aic/aic_engine/config/sample_config.yaml

Writes:
  - /tmp/aic_components.html           (the rendered map)

Opens the result in the user's default browser via xdg-open.

Tabs:
  1. Overview          — competition context (sourced from docs/overview.md + PROJECT.md)
  2. Architecture      — container + topic + lifecycle topology
  3. Scene Inventory   — categorized component tree
  4. Topic Surface     — ROS topics + lifecycle services
  5. Submission        — what a real submission needs + CV pose-estimation seam
  6. Audit             — Gazebo vs Isaac Sim divergences
  7. Tools             — links to project scripts

Usage:
  python3 exts/aic-dt/scripts/build_component_map.py [--no-open] [--output PATH]
"""

from __future__ import annotations
import argparse
import html
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

AIC_REPO = Path.home() / "Documents/aic"
EXT_REPO = Path.home() / "Documents/isaac-sim-mcp/exts/aic-dt"
COMPONENT_MAP_DIR = EXT_REPO / "docs/component-map"
RESOURCES_DIR = COMPONENT_MAP_DIR / "resources"
# In-repo first; /tmp as backward-compatible fallback for sessions where
# the audit/research agents haven't moved their output yet.
INVENTORY_JSON = (RESOURCES_DIR / "aic_object_inventory.json"
                  if (RESOURCES_DIR / "aic_object_inventory.json").exists()
                  else Path("/tmp/aic_object_inventory.json"))
EVAL_CYCLE_MD = (RESOURCES_DIR / "aic_eval_cycle.md"
                 if (RESOURCES_DIR / "aic_eval_cycle.md").exists()
                 else Path("/tmp/aic_eval_cycle.md"))
DEFAULT_OUTPUT = COMPONENT_MAP_DIR / "index.html"
# When a referenced image is not at its source-relative path, the generator
# also checks here (component-map/media/) before falling back to a placeholder.
# Drop replacement PNGs / SVGs here to have them picked up automatically.
USER_IMAGE_OVERRIDE_DIR = COMPONENT_MAP_DIR / "media"


# ---------- helpers ----------

def safe_read(path: Path) -> str:
    if not path.exists():
        return f"<missing: {path}>"
    return path.read_text()


def safe_json(path: Path) -> Any:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except Exception as e:
        return {"_error": str(e)}


def esc(s: Any) -> str:
    return html.escape(str(s)) if s is not None else ""


def md_to_html(md: str, source_path: Path | None = None) -> str:
    """Tiny markdown→HTML — handles headings, lists, code, bold/italics,
    links, IMAGES (with path resolution + missing-asset graceful fallback),
    tables, hr. No external deps.

    Pass source_path so relative image links can be resolved correctly."""
    # Pre-pass: replace image markdown ![alt](path) BEFORE escaping. Resolution
    # order: (1) path relative to source .md file; (2) user override at
    # component-map/media/<basename>; (3) inline SVG placeholder.
    # Use RELATIVE paths so the page works both via file:// (local) AND via
    # HTTP (e.g. python -m http.server, Tailscale-served remote access).
    def replace_img(m):
        alt, src = m.group(1), m.group(2)
        if source_path is not None and not src.startswith(("http://", "https://", "file://", "/")):
            resolved = (source_path.parent / src).resolve()
        else:
            resolved = Path(src)
        if resolved.exists():
            # If resolved sits inside our component-map dir, use a path relative
            # to the HTML output so HTTP serving works. Otherwise stick with
            # file:// (only works locally — degraded mode).
            try:
                rel = resolved.relative_to(COMPONENT_MAP_DIR)
                return f'<img src="{rel}" alt="{html.escape(alt)}" class="md-img" loading="lazy">'
            except ValueError:
                return f'<img src="file://{resolved}" alt="{html.escape(alt)}" class="md-img" loading="lazy">'
        # Fallback: user override at component-map/media/<basename> — always relative
        override = USER_IMAGE_OVERRIDE_DIR / Path(src).name
        if override.exists():
            rel = override.relative_to(COMPONENT_MAP_DIR)
            return f'<img src="{rel}" alt="{html.escape(alt)}" class="md-img" loading="lazy">'
        # Final fallback: inline SVG placeholder with the expected path so
        # the user knows where to drop a real image to override.
        label = html.escape(alt) if alt else html.escape(Path(src).name)
        return f"""<div class="md-img-placeholder" title="Drop a replacement at: {USER_IMAGE_OVERRIDE_DIR}/{Path(src).name}">
  <svg viewBox="0 0 600 200" xmlns="http://www.w3.org/2000/svg" class="md-img-svg">
    <defs>
      <linearGradient id="grad" x1="0" y1="0" x2="1" y2="1">
        <stop offset="0%" stop-color="#1f6feb"/>
        <stop offset="100%" stop-color="#bc4a00"/>
      </linearGradient>
    </defs>
    <rect width="600" height="200" fill="url(#grad)" opacity="0.15"/>
    <rect x="2" y="2" width="596" height="196" fill="none" stroke="#30363d" stroke-width="2" stroke-dasharray="6,4"/>
    <text x="300" y="90" text-anchor="middle" fill="#c9d1d9" font-family="sans-serif" font-size="20" font-weight="600">{label}</text>
    <text x="300" y="120" text-anchor="middle" fill="#8b949e" font-family="monospace" font-size="11">image placeholder — source path not found</text>
    <text x="300" y="140" text-anchor="middle" fill="#8b949e" font-family="monospace" font-size="10">drop {html.escape(Path(src).name)} into exts/aic-dt/docs/component-map/media/ to override</text>
  </svg>
</div>"""
    md = re.sub(r"!\[([^\]]*)\]\(([^)]+)\)", replace_img, md)

    # Pre-pass: rewrite relative LINKS [text](./scoring.md) → GitHub URLs so they
    # don't 404 when the page is served over HTTP (e.g. via Tailscale). External
    # links (http://...) and anchors (#...) pass through unchanged.
    AIC_GITHUB = "https://github.com/intrinsic-dev/aic/blob/main"
    def rewrite_link(m):
        text, href = m.group(1), m.group(2)
        if href.startswith(("http://", "https://", "mailto:", "#")):
            return m.group(0)  # unchanged
        if source_path is None:
            return m.group(0)
        # Resolve href relative to the source markdown file's directory
        try:
            anchor = ""
            if "#" in href:
                href, anchor = href.split("#", 1)
                anchor = "#" + anchor
            resolved = (source_path.parent / href).resolve()
            rel = resolved.relative_to(AIC_REPO)
            new_href = f"{AIC_GITHUB}/{rel}{anchor}"
            return f"[{text}]({new_href})"
        except (ValueError, OSError):
            return m.group(0)
    md = re.sub(r"(?<!!)\[([^\]]+)\]\(([^)]+)\)", rewrite_link, md)
    if not md:
        return ""
    out_lines = []
    in_code = False
    in_table = False
    in_list = False
    for raw in md.splitlines():
        line = raw.rstrip()
        if line.startswith("```"):
            if in_code:
                out_lines.append("</code></pre>")
                in_code = False
            else:
                out_lines.append('<pre class="code"><code>')
                in_code = True
            continue
        if in_code:
            out_lines.append(esc(line))
            continue
        if line.startswith("---"):
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append("<hr>"); continue
        # tables
        if "|" in line and re.search(r"\s*\|", line):
            if not in_table:
                in_table = True
                out_lines.append('<table class="md-table">')
            cells = [c.strip() for c in line.split("|")[1:-1]]
            if all(re.fullmatch(r"[\s:\-]+", c) for c in cells):
                continue  # separator row
            tag = "th" if "header_pending" not in dir(md_to_html) else "td"
            # First row treated as header by convention
            row = "".join(f"<td>{render_inline(c)}</td>" for c in cells)
            out_lines.append(f"<tr>{row}</tr>")
            continue
        elif in_table:
            in_table = False
            out_lines.append("</table>")
        if line.startswith("#### "):
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append(f"<h4>{render_inline(line[5:])}</h4>")
        elif line.startswith("### "):
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append(f"<h3>{render_inline(line[4:])}</h3>")
        elif line.startswith("## "):
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append(f"<h2>{render_inline(line[3:])}</h2>")
        elif line.startswith("# "):
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append(f"<h1>{render_inline(line[2:])}</h1>")
        elif re.match(r"^\s*[-*]\s+", line):
            if not in_list:
                in_list = True
                out_lines.append("<ul>")
            content = re.sub(r"^\s*[-*]\s+", "", line)
            out_lines.append(f"<li>{render_inline(content)}</li>")
        elif line.strip() == "":
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append("")
        else:
            if in_list: out_lines.append("</ul>"); in_list = False
            out_lines.append(f"<p>{render_inline(line)}</p>")
    if in_table: out_lines.append("</table>")
    if in_list: out_lines.append("</ul>")
    if in_code: out_lines.append("</code></pre>")
    return "\n".join(out_lines)


def render_inline(s: str) -> str:
    """Inline markdown: code, bold, italic, links. Preserves any pre-rendered
    HTML tags (e.g. <img>, <div class="md-img-missing">) emitted by the
    image-resolution pre-pass in md_to_html — those are spliced in BEFORE
    escaping happens here, so we restore them after esc()."""
    # Step 1: extract pre-rendered HTML img/div tags into placeholders so esc()
    # doesn't mangle them
    placeholders = []
    def stash(m):
        placeholders.append(m.group(0))
        return f"\x00PH{len(placeholders)-1}\x00"
    s = re.sub(r"<img [^>]+>|<div class=\"md-img-missing\">.*?</div>", stash, s)
    # Step 2: standard inline rendering
    s = esc(s)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
    s = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", s)
    # External links open in new tab; internal anchors don't
    def render_a(m):
        text, href = m.group(1), m.group(2)
        if href.startswith(("http://", "https://")):
            return f'<a href="{href}" target="_blank" rel="noopener">{text}</a>'
        return f'<a href="{href}">{text}</a>'
    s = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", render_a, s)
    # Step 3: restore placeholders
    for i, ph in enumerate(placeholders):
        s = s.replace(f"\x00PH{i}\x00", ph)
    return s


# ---------- tab content builders ----------

def build_overview_tab() -> str:
    overview_path = AIC_REPO / "docs/overview.md"
    project_path = AIC_REPO / ".planning/PROJECT.md"
    overview = safe_read(overview_path)
    project = safe_read(project_path)
    return f"""
    <section>
      <h2>What is AIC?</h2>
      <div class="card">
        {md_to_html(overview[:4000], overview_path)}
      </div>
      <h2>aic_vision project framing</h2>
      <p class="sub">From <code>~/Documents/aic/.planning/PROJECT.md</code> — this is the *user's* project that builds on top of AIC.</p>
      <div class="card">
        {md_to_html(project[:5000], project_path)}
      </div>
    </section>
    """


def build_architecture_tab() -> str:
    eval_cycle = safe_read(EVAL_CYCLE_MD)
    # Extract sections 1 & 4 (eval cycle + ground-truth seam)
    sec1 = ""
    sec4 = ""
    if eval_cycle:
        m = re.search(r"## 1\. Single Evaluation Cycle.*?(?=\n## )", eval_cycle, re.DOTALL)
        if m: sec1 = m.group(0)
        m = re.search(r"## 4\. Ground-Truth vs Estimated Pose.*?(?=\n## )", eval_cycle, re.DOTALL)
        if m: sec4 = m.group(0)

    return f"""
    <section>
      <h2>Container topology (Isaac Sim + host engine/adapter/model)</h2>
      <div class="card">
        <p class="sub">Renders via Mermaid 10 (CDN). If the diagram doesn't appear after switching to this tab, check your network or open DevTools console for errors.</p>
        <div class="mermaid">
flowchart LR
  subgraph SIM["Isaac Sim host process"]
    direction TB
    ISIM["Kit + aic-dt extension<br/>MCP socket :8768"]
    IPP["parity_publishers<br/>joint_states + tf"]
    ISP["scoring_publishers<br/>insertion_event + scoring/tf"]
    ISIM --> IPP
    ISIM --> ISP
  end
  subgraph BUS["host zenoh transport :7447"]
    HZ[("zenohd router")]
  end
  subgraph ENG["host aic_engine (humble)"]
    HENG["Engine.handle_trial loop<br/>tier_1 + tier_2 + tier_3"]
  end
  subgraph ADP["host aic_adapter (humble)"]
    HADP["fuses observation @ 20Hz"]
  end
  subgraph MDL["host aic_model (kilted)"]
    HMDL["LifecycleNode<br/>policy = CheatCode"]
  end
  IPP -- joint_states + tf --> HZ
  ISP -- scoring/tf + insertion_event --> HZ
  HZ --> HENG
  HZ --> HADP
  HZ --> HMDL
  HENG -. InsertCable action goal .-> HMDL
  HADP -. Observation msg .-> HMDL
  HMDL -. pose_commands / joint_commands .-> HENG
        </div>
      </div>

      <h2>Single-trial state machine (engine perspective)</h2>
      <div class="card">
        <p class="sub">Source: <code>aic_engine/src/aic_engine.cpp:682</code> (Engine::handle_trial). Each state has up to 5 retries; failure at any step ends the trial.</p>
        <div class="mermaid">
stateDiagram-v2
  [*] --> Uninitialized
  Uninitialized --> ModelReady: check_model<br/>(lifecycle node discovered + ACTIVE)
  ModelReady --> EndpointsReady: check_endpoints<br/>(adapter + scoring topics present)
  EndpointsReady --> SimulatorReady: ready_simulator<br/>(spawn task_board + cable)
  SimulatorReady --> ScoringReady: ready_scoring<br/>(open rosbag + subscribe topics)
  ScoringReady --> TasksExecuting: tasks_started<br/>(send InsertCable goal)
  TasksExecuting --> AllTasksCompleted: action.success=true<br/>OR time_limit reached
  AllTasksCompleted --> [*]: score_trial<br/>(tier_1 + tier_2 + tier_3)
  TasksExecuting --> Error: any failure
  Error --> [*]
        </div>
      </div>

      <h2>Ground-truth → CV pose-estimation seam</h2>
      <div class="card">
        <p class="sub"><strong>This is the architectural insertion point for the user's aic_vision CV pipeline.</strong> The diagram below shows where ground-truth flows through CheatCode (cyan) and where a CV-based submission must replace that lookup with image-based estimation (orange).</p>
        <div class="mermaid">
flowchart LR
  GT["/scoring/tf<br/>(simulator ground truth)"] --> R{{"ground_truth flag?"}}
  R -- "TRUE" --> RELAY["topic_tools relay<br/>/scoring/tf → /tf"]
  RELAY --> CC_CHEAT["CheatCode.py:217-223<br/>tf_buffer.lookup_transform<br/>(base_link → port_link)"]
  CC_CHEAT --> CC_MOTION["calc_gripper_pose<br/>descent loop<br/>insertion attempt"]
  R -- "FALSE (real submission)" --> X["X — port frames NOT on /tf"]
  ADP["aic_adapter<br/>Observation msg"] --> CAM["left + center + right<br/>RGB images @ 20Hz"]
  CAM --> CV["CV pose estimator<br/>(aic_vision: GigaPose / FoundationPose)"]
  CV --> CC_MOTION
  CC_MOTION --> CMD["pose_commands<br/>to controller"]
  R -.->|"always"| SCORE["scorer reads<br/>/scoring/tf<br/>internally for grading"]
  classDef cheat fill:#1f6feb,stroke:#58a6ff,color:#fff
  classDef cv fill:#bc4a00,stroke:#d29922,color:#fff
  classDef seam fill:#3a2a06,stroke:#d29922,color:#d29922
  class GT,RELAY,CC_CHEAT cheat
  class CAM,CV cv
  class R seam
        </div>
      </div>

      <h2>State machine details (source citation)</h2>
      <div class="card">
        <p class="sub">From <code>/tmp/aic_eval_cycle.md</code> section 1.</p>
        {md_to_html(sec1)}
      </div>

      <h2>Ground-truth seam details (source citation)</h2>
      <div class="card">
        <p class="sub">From <code>/tmp/aic_eval_cycle.md</code> section 4.</p>
        {md_to_html(sec4)}
      </div>
    </section>
    """


def build_scene_tree_tab(inv: list[dict] | None) -> str:
    """Renders an interactive D3-based USD prim hierarchy with pan/zoom.
    Drag to pan, scroll wheel to zoom, click any node to see its inventory
    entry in the side panel."""
    if not inv:
        return "<section><div class='card'>Inventory missing — run audit agent first.</div></section>"

    cat_colors = {
        "environment":       "#3fb950",
        "static_structure":  "#58a6ff",
        "perception_target": "#d29922",
        "robot":             "#bc8cff",
        "sensor":            "#39c5cf",
        "cable_part":        "#f85149",
    }

    # Build a nested tree from spawn_prim_paths
    root: dict = {"name": "/World", "path": "/World", "category": None, "inv_entry": None, "children": {}}

    for e in inv:
        path = (e.get("isaacsim") or {}).get("spawn_prim_path") or ""
        if not path or not path.startswith("/"):
            continue
        parts = path.strip("/").split("/")
        if not parts:
            continue
        cur = root
        accum_path = ""
        for i, seg in enumerate(parts):
            accum_path = (accum_path + "/" + seg) if accum_path else "/" + seg
            if i == 0 and seg == "World":
                continue
            if seg not in cur["children"]:
                matching = next((x for x in inv if (x.get("isaacsim") or {}).get("spawn_prim_path") == accum_path), None)
                cur["children"][seg] = {
                    "name": seg,
                    "path": accum_path,
                    "category": matching.get("category") if matching else cur["category"],
                    "inv_entry": matching,
                    "children": {},
                }
            cur = cur["children"][seg]

    # Convert nested-dict tree → D3-compatible JSON with sorted children arrays
    def to_d3(n: dict) -> dict:
        out = {
            "name": n["name"],
            "path": n["path"],
            "category": n.get("category"),
            "inv_name": (n.get("inv_entry") or {}).get("name", ""),
        }
        kids = sorted(n.get("children", {}).values(), key=lambda c: c["name"])
        if kids:
            out["children"] = [to_d3(c) for c in kids]
        return out

    tree_json = json.dumps(to_d3(root))

    # Build flat inventory map for the JS side panel
    inv_map_json = json.dumps({
        e["name"]: {
            "category": e.get("category"),
            "role": e.get("role"),
            "gazebo": e.get("gazebo", {}),
            "isaacsim": e.get("isaacsim", {}),
            "notes": e.get("notes", ""),
        } for e in inv
    })

    cat_colors_json = json.dumps(cat_colors)
    legend_rows = "".join(
        f'<span class="legend-pill"><span class="legend-swatch" style="background:{c}"></span>{cat}</span>'
        for cat, c in cat_colors.items()
    )
    n_prims = sum(1 for _ in walk_tree(root))

    return f"""
    <section>
      <h2>USD prim hierarchy — interactive D3 tree ({n_prims} prims)</h2>
      <p class="sub">Built from <code>resources/aic_object_inventory.json</code> spawn paths.
      <strong>Drag</strong> to pan · <strong>scroll wheel</strong> to zoom · <strong>click a node</strong> for inventory details ·
      <strong>click circle</strong> to collapse/expand subtree.</p>
      <div class="card legend-card">
        {legend_rows}
        <button class="tree-ctl-btn" onclick="treeFit()">⊡ fit to view</button>
        <button class="tree-ctl-btn" onclick="treeReset()">⟳ reset</button>
      </div>
      <div class="d3-tree-layout">
        <div class="d3-tree-svg-wrap">
          <svg id="d3-tree-svg" width="100%" height="780"></svg>
        </div>
        <div class="tree-detail" id="tree-detail">
          <div class="sub">Click any node to see its inventory entry here.</div>
        </div>
      </div>
      <script src="https://cdn.jsdelivr.net/npm/d3@7/dist/d3.min.js"></script>
      <script>
        const TREE_DATA      = {tree_json};
        const INVENTORY_MAP  = {inv_map_json};
        const CAT_COLORS     = {cat_colors_json};
        let _treeState = {{ zoom: null, svg: null, g: null, root: null, treeLayout: null }};

        function initD3Tree() {{
          const svg = d3.select('#d3-tree-svg');
          if (svg.empty()) return;
          const width = svg.node().getBoundingClientRect().width || 1200;
          const height = +svg.attr('height') || 780;
          svg.selectAll('*').remove();

          const g = svg.append('g');
          _treeState.svg = svg;
          _treeState.g = g;

          // Build hierarchy and tree layout
          const root = d3.hierarchy(TREE_DATA);
          // Initially collapse anything past depth 2 to keep the initial view sane
          root.descendants().forEach(d => {{
            if (d.depth >= 2 && d.children) {{ d._children = d.children; d.children = null; }}
          }});
          _treeState.root = root;
          // Horizontal node spacing: depth * 220px, vertical: 24px per leaf
          _treeState.treeLayout = d3.tree().nodeSize([26, 240]);

          // Zoom behavior — pan + zoom on the whole drawing
          const zoom = d3.zoom()
            .scaleExtent([0.2, 5])
            .on('zoom', (ev) => {{ g.attr('transform', ev.transform); }});
          svg.call(zoom).call(zoom.transform, d3.zoomIdentity.translate(60, height/2).scale(0.8));
          _treeState.zoom = zoom;

          render();
        }}

        function render() {{
          const {{ g, root, treeLayout }} = _treeState;
          treeLayout(root);

          // Links (edges)
          const links = g.selectAll('path.tree-link')
            .data(root.links(), d => d.target.data.path);
          links.enter().append('path')
            .attr('class', 'tree-link')
            .attr('fill', 'none')
            .attr('stroke', '#444c56')
            .attr('stroke-width', 1.2)
            .merge(links)
            .attr('d', d3.linkHorizontal().x(d => d.y).y(d => d.x));
          links.exit().remove();

          // Nodes
          const nodes = g.selectAll('g.tree-node-g')
            .data(root.descendants(), d => d.data.path);
          const enter = nodes.enter().append('g')
            .attr('class', 'tree-node-g')
            .attr('transform', d => `translate(${{d.y}},${{d.x}})`)
            .style('cursor', 'pointer');
          // Toggle circle (controls expand/collapse)
          enter.append('circle')
            .attr('r', 5)
            .attr('fill', d => (d.data.category && CAT_COLORS[d.data.category]) || '#6e7681')
            .attr('stroke', '#0e1116')
            .attr('stroke-width', 2)
            .on('click', (ev, d) => {{
              ev.stopPropagation();
              if (d.children) {{ d._children = d.children; d.children = null; }}
              else if (d._children) {{ d.children = d._children; d._children = null; }}
              render();
            }});
          // Label
          enter.append('text')
            .attr('dy', '0.32em')
            .attr('x', d => (d.children || d._children) ? -10 : 10)
            .attr('text-anchor', d => (d.children || d._children) ? 'end' : 'start')
            .text(d => d.data.name)
            .attr('fill', '#c9d1d9')
            .attr('font-size', '12px')
            .attr('font-family', 'sans-serif')
            .on('click', (ev, d) => {{
              ev.stopPropagation();
              showDetail(d.data);
            }});

          // Merge update
          const all = enter.merge(nodes)
            .attr('transform', d => `translate(${{d.y}},${{d.x}})`);
          all.select('circle')
            .attr('fill', d => (d.data.category && CAT_COLORS[d.data.category]) || '#6e7681')
            .attr('r', d => (d._children ? 7 : 5))
            .attr('stroke-width', d => (d._children ? 2 : 2));

          nodes.exit().remove();
        }}

        function showDetail(d) {{
          const detail = document.getElementById('tree-detail');
          const inv = INVENTORY_MAP[d.inv_name];
          if (!inv) {{
            detail.innerHTML = `<div class="sub">No inventory entry for <code>${{d.path}}</code> — intermediate path node.</div>`;
            return;
          }}
          const gz = inv.gazebo || {{}};
          const iss = inv.isaacsim || {{}};
          const E = s => String(s||'').replace(/[&<>"']/g, c => ({{
            '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
          }})[c]);
          const list = arr => (arr||[]).map(x => `<code>${{E(x)}}</code>`).join('<br>');
          detail.innerHTML = `
            <h3>${{E(d.inv_name)}}</h3>
            <p class="sub"><strong>Category:</strong> ${{E(inv.category||'')}} · <strong>Path:</strong> <code>${{E(d.path)}}</code></p>
            <p>${{E(inv.role||'')}}</p>
            <h4>Gazebo</h4>
            <table>
              <tr><td>Sources</td><td>${{list(gz.source_files)}}</td></tr>
              <tr><td>Pose source</td><td>${{E(gz.world_pose_source||'')}}</td></tr>
              <tr><td>Default pose</td><td><code>${{E(gz.default_pose||'')}}</code></td></tr>
              <tr><td>Visual URI</td><td><code>${{E(gz.visual_uri||'')}}</code></td></tr>
              <tr><td>GLB AABB (raw)</td><td><code>${{E(JSON.stringify(gz.visual_glb_aabb_raw_m||[]))}}</code></td></tr>
              <tr><td>Collision</td><td>${{E(gz.collision_type||'')}}</td></tr>
              <tr><td>Scoring refs</td><td>${{E(gz.scoring_references||'')}}</td></tr>
            </table>
            <h4>Isaac Sim</h4>
            <table>
              <tr><td>Extension code</td><td>${{list(iss.extension_code_paths)}}</td></tr>
              <tr><td>USD assets</td><td>${{list(iss.usd_asset_paths)}}</td></tr>
              <tr><td>Spawn path</td><td><code>${{E(iss.spawn_prim_path||'')}}</code></td></tr>
              <tr><td>Live pose</td><td><code>${{E(iss.current_live_pose||'')}}</code></td></tr>
              <tr><td>Scale op</td><td><code>${{E(iss.scale_op||'')}}</code></td></tr>
              ${{iss.known_divergences ? `<tr><td>Divergences</td><td>${{E(iss.known_divergences)}}</td></tr>` : ''}}
            </table>
            ${{inv.notes ? `<p><strong>Notes:</strong> ${{E(inv.notes)}}</p>` : ''}}
          `;
        }}

        function treeFit() {{
          if (!_treeState.svg) return;
          const svg = _treeState.svg;
          const gNode = _treeState.g.node();
          const bbox = gNode.getBBox();
          const fullW = svg.node().getBoundingClientRect().width;
          const fullH = +svg.attr('height');
          const scale = Math.min(fullW / (bbox.width + 60), fullH / (bbox.height + 60), 1);
          const tx = -bbox.x * scale + 30;
          const ty = -bbox.y * scale + (fullH - bbox.height * scale) / 2;
          svg.transition().duration(500).call(
            _treeState.zoom.transform,
            d3.zoomIdentity.translate(tx, ty).scale(scale)
          );
        }}

        function treeReset() {{
          if (!_treeState.root) return;
          // Collapse all but the first 2 levels
          _treeState.root.descendants().forEach(d => {{
            if (d.depth >= 2 && d.children) {{ d._children = d.children; d.children = null; }}
            else if (d.depth < 2 && d._children) {{ d.children = d._children; d._children = null; }}
          }});
          render();
          treeFit();
        }}

        // Initialize when the Scene Tree tab is first activated
        function maybeInitTree() {{
          if (document.getElementById('tree').classList.contains('active') && !_treeState.svg) {{
            initD3Tree();
            setTimeout(treeFit, 100);
          }}
        }}
        window.addEventListener('DOMContentLoaded', () => {{
          // If user starts on this tab somehow, initialize. Otherwise wait for tab switch.
          maybeInitTree();
          // Hook into the existing tab-switch JS by observing the active class change
          const tabEl = document.getElementById('tree');
          new MutationObserver(maybeInitTree).observe(tabEl, {{attributes: true, attributeFilter: ['class']}});
        }});
      </script>
    </section>
    """


def walk_tree(node: dict):
    """Yield all descendant nodes (incl. self) of a tree node built by build_scene_tree_tab."""
    yield node
    for child in node.get("children", {}).values():
        yield from walk_tree(child)


def build_live_kit_tab() -> str:
    """Renders the Live Kit UI mirror — same button groups as the aic-dt
    extension's Kit panel, wired to fire MCP commands via /api/mcp.
    Requires component_map_server.py running (NOT plain python -m http.server)."""
    # Mirrors the omni.ui structure in extension.py:_build_ui() (CollapsableFrames)
    # Each entry: (label, mcp_atom_name or None for clubbed, optional params override)
    groups = [
        ("Simulation Setup", [
            ("Load Scene",            "load_scene",   {}),
            ("Quick Start",           "quick_start",  {}),
            ("Play Scene",            "play_scene",   {}),
            ("Stop Scene",            "stop_scene",   {}),
            ("New Stage",             "new_stage",    {}),
        ]),
        ("AIC Scene", [
            ("Import Enclosure",      "import_enclosure",     {}),
        ]),
        ("UR5e + Cable", [
            ("Import UR5e",           "load_robot",           {}),
            ("Setup TF Publisher",    "setup_tf_publish_action_graph", {}),
            ("Setup JointState Pub",  "setup_joint_state_publish_action_graph", {}),
            ("Setup Action Graph",    "setup_action_graph",   {}),
            ("Setup Force Publisher", "setup_force_publisher", {}),
            ("Attach Cable",          "attach_cable_to_gripper", {}),
        ]),
        ("Cameras", [
            ("Setup Wrist Cameras",   "setup_wrist_cameras",  {}),
        ]),
        ("Task Board Objects", [
            ("Add All Objects",       "add_objects",          {}),
            ("Delete Objects",        "delete_objects",       {}),
        ]),
        ("Trials (sample_config.yaml)", [
            ("Load trial_1 (GT=on)",  "load_trial",           {"trial_key": "trial_1", "ground_truth": True}),
            ("Load trial_2 (GT=on)",  "load_trial",           {"trial_key": "trial_2", "ground_truth": True}),
            ("Load trial_3 (GT=on)",  "load_trial",           {"trial_key": "trial_3", "ground_truth": True}),
            ("Load trial_1 (GT=off — submission mode)", "load_trial", {"trial_key": "trial_1", "ground_truth": False}),
        ]),
    ]

    def render_button(label: str, atom: str, params: dict) -> str:
        params_json = json.dumps(params)
        atom_esc = esc(atom)
        DQ = '"'
        params_attr_safe = params_json.replace(DQ, "&quot;")
        return (f'<button class="live-btn" onclick="fireMcp(this, &quot;{atom_esc}&quot;, '
                f'{params_attr_safe})">'
                f'<span class="atom-label">{esc(label)}</span>'
                f'<span class="atom-name">{atom_esc}</span></button>')

    sections_html = ""
    for title, items in groups:
        buttons = "".join(render_button(*item) for item in items)
        sections_html += f'''
        <details class="live-frame" open>
          <summary><strong>{esc(title)}</strong> ({len(items)} atoms)</summary>
          <div class="live-btn-row">{buttons}</div>
        </details>
        '''

    return f"""
    <section>
      <h2>Live Kit UI — remote control over MCP</h2>
      <p class="sub">Mirrors the aic-dt extension's Kit UI panel. Click any button to fire an MCP command against
      the running Isaac Sim at <code>localhost:8768</code> (via the local proxy at <code>/api/mcp</code>).
      <strong>Requires <code>component_map_server.py</code> running</strong> instead of
      <code>python -m http.server</code> — otherwise the buttons will 404.</p>
      <div class="card">
        <div class="live-toolbar">
          <button onclick="probeHealth()">⚙ probe MCP health</button>
          <button onclick="probeState()">📊 refresh live state</button>
          <span id="live-health" class="health-indicator">unknown</span>
        </div>
        {sections_html}
      </div>
      <div class="live-result-pane">
        <div class="card">
          <h3>Live scene state</h3>
          <pre id="live-state-pre" class="code">(click "refresh live state" to probe)</pre>
        </div>
        <div class="card">
          <h3>Last MCP response</h3>
          <pre id="live-response-pre" class="code">(no calls yet)</pre>
        </div>
      </div>
      <script>
        async function fireMcp(btn, atomName, params) {{
          btn.classList.add('firing');
          btn.disabled = true;
          const original = btn.querySelector('.atom-label').textContent;
          btn.querySelector('.atom-label').textContent = '⏳ ' + original;
          try {{
            const resp = await fetch('/api/mcp', {{
              method: 'POST',
              headers: {{'Content-Type': 'application/json'}},
              body: JSON.stringify({{type: atomName, params: params}}),
            }});
            const data = await resp.json();
            document.getElementById('live-response-pre').textContent =
              `[${{new Date().toLocaleTimeString()}}] ${{atomName}}(${{JSON.stringify(params)}})\\n` +
              JSON.stringify(data, null, 2);
            btn.classList.remove('firing');
            btn.classList.add(data.status === 'success' ? 'ok' : 'err');
            setTimeout(() => btn.classList.remove('ok', 'err'), 2000);
          }} catch (e) {{
            document.getElementById('live-response-pre').textContent =
              `[${{new Date().toLocaleTimeString()}}] ${{atomName}} ERROR\\n${{e.message}}`;
            btn.classList.remove('firing');
            btn.classList.add('err');
            setTimeout(() => btn.classList.remove('err'), 2000);
          }} finally {{
            btn.querySelector('.atom-label').textContent = original;
            btn.disabled = false;
          }}
        }}
        async function probeHealth() {{
          const r = document.getElementById('live-health');
          r.textContent = '...';
          try {{
            const resp = await fetch('/api/health');
            const data = await resp.json();
            r.textContent = data.mcp_reachable ? '✓ MCP reachable :' + data.port : '✗ MCP DOWN: ' + (data.error || '');
            r.className = 'health-indicator ' + (data.mcp_reachable ? 'ok' : 'err');
          }} catch (e) {{
            r.textContent = '✗ server unreachable';
            r.className = 'health-indicator err';
          }}
        }}
        async function probeState() {{
          const pre = document.getElementById('live-state-pre');
          pre.textContent = '...probing...';
          try {{
            const resp = await fetch('/api/state');
            const data = await resp.json();
            pre.textContent = JSON.stringify(data, null, 2);
          }} catch (e) {{
            pre.textContent = 'error: ' + e.message;
          }}
        }}
        // Auto-probe health on tab activation
        function maybeProbe() {{
          if (document.getElementById('live-kit').classList.contains('active')) {{
            probeHealth();
          }}
        }}
        window.addEventListener('DOMContentLoaded', () => {{
          maybeProbe();
          const tabEl = document.getElementById('live-kit');
          new MutationObserver(maybeProbe).observe(tabEl, {{attributes: true, attributeFilter: ['class']}});
        }});
      </script>
    </section>
    """


def _gazebo_path_for_entry(name: str, category: str | None) -> str | None:
    """Map an inventory entry to its Gazebo-side hierarchical path.
    Mirrors the SDF structure: aic_world → top-level model → link/sub-model.
    Source: aic_description/world/aic.sdf + per-model.sdf + task_board.urdf.xacro."""
    # environment
    if name == "ground_plane":
        return "aic_world/ground_plane"
    if name == "floor_walls_decorative":
        return "aic_world/floor"
    if name == "aic_enclosure":
        return "aic_world/enclosure"
    if name == "enclosure_walls":
        return "aic_world/enclosure/walls"
    if name == "dome_light":
        return "aic_world/lighting/dome_light"
    # task_board family
    if name == "task_board_base":
        return "aic_world/task_board/task_board_base_link"
    if name.startswith("nic_card_mount_"):
        idx = name.rsplit("_", 1)[-1]
        return f"aic_world/task_board/nic_card_mount_{idx}/nic_card_mount_link"
    if name == "nic_card_link_pcb":
        return "aic_world/task_board/nic_card_mount_0/nic_card_link"
    if name.startswith("sc_port_") and name[-1].isdigit():
        idx = name.rsplit("_", 1)[-1]
        return f"aic_world/task_board/sc_port_{idx}/sc_port_link"
    if name.startswith("lc_mount_rail_"):
        idx = name.rsplit("_", 1)[-1]
        return f"aic_world/task_board/lc_mount_rail_{idx}"
    if name.startswith("sfp_mount_rail_"):
        idx = name.rsplit("_", 1)[-1]
        return f"aic_world/task_board/sfp_mount_rail_{idx}"
    if name.startswith("sc_mount_rail_"):
        idx = name.rsplit("_", 1)[-1]
        return f"aic_world/task_board/sc_mount_rail_{idx}"
    # perception targets (sfp_port_0/1 colliders sit under nic_card_mount_0 in Gazebo)
    if name == "sfp_port_0_collider":
        return "aic_world/task_board/nic_card_mount_0/sfp_port_0_link"
    if name == "sfp_port_1_collider":
        return "aic_world/task_board/nic_card_mount_0/sfp_port_1_link"
    if name == "sc_port_base_collider":
        return "aic_world/task_board/sc_port_0/sc_port_base_link"
    # robot
    if name == "ur5e":
        return "aic_world/ur5e"
    if name.startswith("ur5e_link__"):
        link = name[len("ur5e_link__"):]
        return f"aic_world/ur5e/{link}"
    if name == "robotiq_hande_gripper":
        return "aic_world/ur5e/gripper_hande"
    # sensors
    if name == "axia80_ft_sensor":
        return "aic_world/ur5e/ati_axia80"
    if name == "workspace_camera":
        return "aic_world/workspace_camera"
    if name == "wrist_cameras":
        return "aic_world/ur5e/wrist_camera"
    # cable
    if name == "cable_root_xform":
        return "aic_world/cable"
    if name == "cable_rope_chain_21_links":
        return "aic_world/cable/rope_chain"
    if name == "cable_sc_plug_visual":
        return "aic_world/cable/sc_plug_link"
    if name == "cable_lc_plug_visual":
        return "aic_world/cable/lc_plug_link"
    if name == "cable_sfp_module_visual":
        return "aic_world/cable/sfp_module_link"
    if name == "cable_attach_joint":
        return "aic_world/cable/attach_joint"
    # Isaac-only synth (no Gazebo equivalent)
    if name == "plug_proxy":
        return None
    return None


def build_gazebo_tree_tab(inv: list[dict] | None) -> str:
    """Renders the Gazebo-side scene tree as a true hierarchy matching Gazebo's
    SDF model + link structure. Same D3 visualization as the Isaac Sim tree."""
    if not inv:
        return "<section><div class='card'>Inventory missing — run audit agent first.</div></section>"

    cat_colors = {
        "environment":       "#3fb950",
        "static_structure":  "#58a6ff",
        "perception_target": "#d29922",
        "robot":             "#bc8cff",
        "sensor":            "#39c5cf",
        "cable_part":        "#f85149",
    }

    # Build a nested-dict tree from each entry's mapped Gazebo path
    root: dict = {"name": "aic_world", "path": "aic_world",
                  "category": None, "inv_entry": None, "children": {}}
    skipped_isaac_only: list[str] = []
    for e in inv:
        gz_path = _gazebo_path_for_entry(e["name"], e.get("category"))
        if gz_path is None:
            skipped_isaac_only.append(e["name"])
            continue
        parts = gz_path.split("/")
        # First part should be "aic_world" (the root)
        if parts[0] != "aic_world":
            continue
        cur = root
        accum = "aic_world"
        for seg in parts[1:]:
            accum = f"{accum}/{seg}"
            if seg not in cur["children"]:
                # Is this the LEAF (terminal node) for this entry?
                is_leaf = (accum == gz_path)
                cur["children"][seg] = {
                    "name": seg,
                    "path": accum,
                    "category": e.get("category") if is_leaf else None,
                    "inv_entry": e if is_leaf else None,
                    "children": {},
                }
            elif accum == gz_path and not cur["children"][seg].get("inv_entry"):
                # The path was created as an intermediate, but this entry IS the leaf
                cur["children"][seg]["inv_entry"] = e
                cur["children"][seg]["category"] = e.get("category")
            cur = cur["children"][seg]

    # Bubble categories up to intermediate nodes for color coding (majority vote)
    def bubble_categories(node: dict) -> str | None:
        if not node["children"]:
            return node.get("category")
        child_cats = [bubble_categories(c) for c in node["children"].values()]
        child_cats = [c for c in child_cats if c]
        if not node.get("category") and child_cats:
            node["category"] = max(set(child_cats), key=child_cats.count)
        return node.get("category")
    bubble_categories(root)

    def to_d3(n: dict) -> dict:
        out = {
            "name": n["name"],
            "path": n["path"],
            "category": n.get("category"),
            "inv_name": (n.get("inv_entry") or {}).get("name", ""),
        }
        kids = sorted(n.get("children", {}).values(), key=lambda c: c["name"])
        if kids:
            out["children"] = [to_d3(c) for c in kids]
        return out

    root_node = to_d3(root)

    tree_json = json.dumps(root_node)
    inv_map_json = json.dumps({
        e["name"]: {
            "category": e.get("category"),
            "role": e.get("role"),
            "gazebo": e.get("gazebo", {}),
            "isaacsim": e.get("isaacsim", {}),
            "notes": e.get("notes", ""),
        } for e in inv
    })
    cat_colors_json = json.dumps(cat_colors)
    legend_rows = "".join(
        f'<span class="legend-pill"><span class="legend-swatch" style="background:{c}"></span>{cat}</span>'
        for cat, c in cat_colors.items()
    )
    # Count actual leaf entries in the tree
    def _count_leaves(n: dict) -> int:
        if not n.get("children"):
            return 1
        return sum(_count_leaves(c) for c in n["children"])
    n_entries = _count_leaves(root_node)
    n_iso_only = len(skipped_isaac_only)
    iso_note = f" · {n_iso_only} Isaac-Sim-only entries skipped (e.g. plug_proxy)" if n_iso_only else ""

    return f"""
    <section>
      <h2>Gazebo scene tree — {n_entries} prims hierarchical{iso_note}</h2>
      <p class="sub">Mirrors Gazebo SDF model structure: <code>aic_world</code> → top-level included models (ur5e, task_board, cable, enclosure) → their link/sub-model hierarchy from the per-model SDFs and <code>task_board.urdf.xacro</code>.
      Same depth + ordering as the Isaac Sim tree.
      <strong>Drag to pan · scroll to zoom · click node for details · click circle to expand/collapse.</strong></p>
      <div class="card legend-card">
        {legend_rows}
        <button class="tree-ctl-btn" onclick="gzTreeFit()">⊡ fit to view</button>
        <button class="tree-ctl-btn" onclick="gzTreeReset()">⟳ reset</button>
      </div>
      <div class="d3-tree-layout">
        <div class="d3-tree-svg-wrap">
          <svg id="gz-tree-svg" width="100%" height="780"></svg>
        </div>
        <div class="tree-detail" id="gz-tree-detail">
          <div class="sub">Click any node to see its inventory entry here.</div>
        </div>
      </div>
      <script>
        const GZ_TREE_DATA     = {tree_json};
        const GZ_INVENTORY_MAP = {inv_map_json};
        const GZ_CAT_COLORS    = {cat_colors_json};
        let _gzTreeState = {{ zoom: null, svg: null, g: null, root: null, treeLayout: null }};

        function initGzD3Tree() {{
          const svg = d3.select('#gz-tree-svg');
          if (svg.empty()) return;
          const height = +svg.attr('height') || 780;
          svg.selectAll('*').remove();
          const g = svg.append('g');
          _gzTreeState.svg = svg;
          _gzTreeState.g = g;
          const root = d3.hierarchy(GZ_TREE_DATA);
          root.descendants().forEach(d => {{
            if (d.depth >= 1 && d.children) {{ d._children = d.children; d.children = null; }}
          }});
          _gzTreeState.root = root;
          _gzTreeState.treeLayout = d3.tree().nodeSize([26, 280]);
          const zoom = d3.zoom().scaleExtent([0.2, 5]).on('zoom', (ev) => g.attr('transform', ev.transform));
          svg.call(zoom).call(zoom.transform, d3.zoomIdentity.translate(60, height/2).scale(0.8));
          _gzTreeState.zoom = zoom;
          gzRender();
        }}

        function gzRender() {{
          const {{ g, root, treeLayout }} = _gzTreeState;
          treeLayout(root);
          const links = g.selectAll('path.tree-link').data(root.links(), d => d.target.data.path);
          links.enter().append('path').attr('class', 'tree-link').attr('fill', 'none').attr('stroke', '#444c56').attr('stroke-width', 1.2)
            .merge(links).attr('d', d3.linkHorizontal().x(d => d.y).y(d => d.x));
          links.exit().remove();
          const nodes = g.selectAll('g.tree-node-g').data(root.descendants(), d => d.data.path);
          const enter = nodes.enter().append('g').attr('class', 'tree-node-g')
            .attr('transform', d => `translate(${{d.y}},${{d.x}})`).style('cursor', 'pointer');
          enter.append('circle').attr('r', 5)
            .attr('fill', d => (d.data.category && GZ_CAT_COLORS[d.data.category]) || '#6e7681')
            .attr('stroke', '#0e1116').attr('stroke-width', 2)
            .on('click', (ev, d) => {{
              ev.stopPropagation();
              if (d.children) {{ d._children = d.children; d.children = null; }}
              else if (d._children) {{ d.children = d._children; d._children = null; }}
              gzRender();
            }});
          enter.append('text').attr('dy', '0.32em')
            .attr('x', d => (d.children || d._children) ? -10 : 10)
            .attr('text-anchor', d => (d.children || d._children) ? 'end' : 'start')
            .text(d => d.data.name).attr('fill', '#c9d1d9').attr('font-size', '12px')
            .attr('font-family', 'sans-serif')
            .on('click', (ev, d) => {{ ev.stopPropagation(); gzShowDetail(d.data); }});
          const all = enter.merge(nodes).attr('transform', d => `translate(${{d.y}},${{d.x}})`);
          all.select('circle')
            .attr('fill', d => (d.data.category && GZ_CAT_COLORS[d.data.category]) || '#6e7681')
            .attr('r', d => (d._children ? 7 : 5));
          nodes.exit().remove();
        }}

        function gzShowDetail(d) {{
          const detail = document.getElementById('gz-tree-detail');
          const inv = GZ_INVENTORY_MAP[d.inv_name];
          if (!inv) {{
            detail.innerHTML = `<div class="sub">No inventory entry for <code>${{d.path}}</code> — this is a model-group node grouping its children.</div>`;
            return;
          }}
          const gz = inv.gazebo || {{}};
          const iss = inv.isaacsim || {{}};
          const E = s => String(s||'').replace(/[&<>"']/g, c => ({{
            '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
          }})[c]);
          const list = arr => (arr||[]).map(x => `<code>${{E(x)}}</code>`).join('<br>');
          detail.innerHTML = `
            <h3>${{E(d.inv_name)}}</h3>
            <p class="sub"><strong>Category:</strong> ${{E(inv.category||'')}} · <strong>Path:</strong> <code>${{E(d.path)}}</code></p>
            <p>${{E(inv.role||'')}}</p>
            <h4>Gazebo</h4>
            <table>
              <tr><td>Sources</td><td>${{list(gz.source_files)}}</td></tr>
              <tr><td>Pose source</td><td>${{E(gz.world_pose_source||'')}}</td></tr>
              <tr><td>Default pose</td><td><code>${{E(gz.default_pose||'')}}</code></td></tr>
              <tr><td>Visual URI</td><td><code>${{E(gz.visual_uri||'')}}</code></td></tr>
              <tr><td>GLB AABB (raw)</td><td><code>${{E(JSON.stringify(gz.visual_glb_aabb_raw_m||[]))}}</code></td></tr>
              <tr><td>Collision</td><td>${{E(gz.collision_type||'')}}</td></tr>
              <tr><td>Scoring refs</td><td>${{E(gz.scoring_references||'')}}</td></tr>
            </table>
            <h4>Isaac Sim equivalent</h4>
            <table>
              <tr><td>Extension code</td><td>${{list(iss.extension_code_paths)}}</td></tr>
              <tr><td>USD assets</td><td>${{list(iss.usd_asset_paths)}}</td></tr>
              <tr><td>Spawn path</td><td><code>${{E(iss.spawn_prim_path||'')}}</code></td></tr>
              <tr><td>Live pose</td><td><code>${{E(iss.current_live_pose||'')}}</code></td></tr>
              <tr><td>Scale op</td><td><code>${{E(iss.scale_op||'')}}</code></td></tr>
              ${{iss.known_divergences ? `<tr><td>Divergences</td><td>${{E(iss.known_divergences)}}</td></tr>` : ''}}
            </table>
            ${{inv.notes ? `<p><strong>Notes:</strong> ${{E(inv.notes)}}</p>` : ''}}
          `;
        }}

        function gzTreeFit() {{
          if (!_gzTreeState.svg) return;
          const svg = _gzTreeState.svg;
          const bbox = _gzTreeState.g.node().getBBox();
          const fullW = svg.node().getBoundingClientRect().width;
          const fullH = +svg.attr('height');
          const scale = Math.min(fullW / (bbox.width + 60), fullH / (bbox.height + 60), 1);
          const tx = -bbox.x * scale + 30;
          const ty = -bbox.y * scale + (fullH - bbox.height * scale) / 2;
          svg.transition().duration(500).call(_gzTreeState.zoom.transform, d3.zoomIdentity.translate(tx, ty).scale(scale));
        }}

        function gzTreeReset() {{
          if (!_gzTreeState.root) return;
          _gzTreeState.root.descendants().forEach(d => {{
            if (d.depth >= 1 && d.children) {{ d._children = d.children; d.children = null; }}
            else if (d.depth < 1 && d._children) {{ d.children = d._children; d._children = null; }}
          }});
          gzRender();
          gzTreeFit();
        }}

        function maybeInitGzTree() {{
          if (document.getElementById('gz-tree').classList.contains('active') && !_gzTreeState.svg) {{
            initGzD3Tree();
            setTimeout(gzTreeFit, 100);
          }}
        }}
        window.addEventListener('DOMContentLoaded', () => {{
          maybeInitGzTree();
          const tabEl = document.getElementById('gz-tree');
          new MutationObserver(maybeInitGzTree).observe(tabEl, {{attributes: true, attributeFilter: ['class']}});
        }});
      </script>
    </section>
    """


def build_inventory_tab(inv: list[dict] | None) -> str:
    if not inv:
        return "<section><div class='card'>Inventory missing — run audit agent first.</div></section>"

    # Group by category. Special handling for `robot` — collapse arm joints,
    # surface only separately-declared assets per user direction.
    by_cat: dict[str, list[dict]] = {}
    for e in inv:
        cat = e.get("category", "other")
        by_cat.setdefault(cat, []).append(e)

    out = ["<section>"]
    cat_order = ["environment", "static_structure", "perception_target",
                 "robot", "sensor", "cable_part", "other"]
    cat_labels = {
        "environment":       ("Environment",        "ground, enclosure, lighting"),
        "static_structure":  ("Task Board",          "base, mounts, ports, rails — fixed within a trial"),
        "perception_target": ("Perception Targets",  "the holes the gripper inserts INTO (CV-detection targets)"),
        "robot":             ("Robot",               "UR5e + Robotiq Hand-E gripper (collapsed; expand for sub-assets)"),
        "sensor":            ("Sensors",             "F/T + cameras"),
        "cable_part":        ("Cable",               "rope chain + connectors + attach mechanism"),
        "other":             ("Other",               ""),
    }
    for cat in cat_order:
        if cat not in by_cat: continue
        items = by_cat[cat]
        label, sub = cat_labels.get(cat, (cat.title(), ""))
        # Robot: collapse the individual link entries — show as one card with a
        # nested "sub-assets that warrant separate attention" list inside
        if cat == "robot":
            top = [e for e in items if not e["name"].startswith("ur5e_link__")]
            sublinks = [e for e in items if e["name"].startswith("ur5e_link__")]
            out.append(f"<details open><summary><strong>{label}</strong> <span class='sub'>— {sub} ({len(items)} entries)</span></summary>")
            for e in top:
                out.append(render_object_card(e))
            if sublinks:
                out.append(f"<details><summary>UR5e links ({len(sublinks)} — collapsed, expand for details)</summary>")
                for e in sublinks:
                    out.append(render_object_card(e, compact=True))
                out.append("</details>")
            out.append("</details>")
        else:
            out.append(f"<details open><summary><strong>{label}</strong> <span class='sub'>— {sub} ({len(items)} entries)</span></summary>")
            for e in items:
                out.append(render_object_card(e))
            out.append("</details>")
    out.append("</section>")
    return "\n".join(out)


def render_object_card(e: dict, compact: bool = False) -> str:
    gz = e.get("gazebo", {}) or {}
    iss = e.get("isaacsim", {}) or {}
    diverg = iss.get("known_divergences", "") or ""
    severity = classify_divergence(diverg)
    sev_class = {"PASS": "ok", "WARN": "warn", "FAIL": "fail"}[severity]
    if compact:
        return f"""
        <div class="obj-card compact">
          <span class="badge {sev_class}">{severity}</span>
          <code>{esc(e['name'])}</code>
          <span class="role">{esc(e.get('role', ''))}</span>
        </div>"""
    return f"""
    <details class="obj-card">
      <summary>
        <span class="badge {sev_class}">{severity}</span>
        <code>{esc(e['name'])}</code>
        <span class="role">{esc(e.get('role', ''))}</span>
      </summary>
      <div class="obj-detail">
        <div class="col">
          <h4>Gazebo</h4>
          <table>
            <tr><td>Source</td><td>{render_file_list(gz.get('source_files', []))}</td></tr>
            <tr><td>Default pose</td><td><code>{esc(gz.get('default_pose', ''))}</code></td></tr>
            <tr><td>Pose source</td><td>{esc(gz.get('world_pose_source', ''))}</td></tr>
            <tr><td>Visual URI</td><td><code>{esc(gz.get('visual_uri', ''))}</code></td></tr>
            <tr><td>GLB AABB (raw)</td><td><code>{esc(gz.get('visual_glb_aabb_raw_m', ''))}</code></td></tr>
            <tr><td>Collision</td><td>{esc(gz.get('collision_type', ''))}</td></tr>
            <tr><td>Scoring refs</td><td>{esc(gz.get('scoring_references', ''))}</td></tr>
          </table>
        </div>
        <div class="col">
          <h4>Isaac Sim</h4>
          <table>
            <tr><td>Extension code</td><td>{render_file_list(iss.get('extension_code_paths', []))}</td></tr>
            <tr><td>USD assets</td><td>{render_file_list(iss.get('usd_asset_paths', []))}</td></tr>
            <tr><td>Spawn path</td><td><code>{esc(iss.get('spawn_prim_path', ''))}</code></td></tr>
            <tr><td>Live pose</td><td><code>{esc(iss.get('current_live_pose', ''))}</code></td></tr>
            <tr><td>Scale op</td><td><code>{esc(iss.get('scale_op', ''))}</code></td></tr>
          </table>
        </div>
      </div>
      {f'<div class="divergence {sev_class}"><strong>Known divergences:</strong> {esc(diverg)}</div>' if diverg else ''}
      {f'<div class="notes"><strong>Notes:</strong> {esc(e.get("notes", ""))}</div>' if e.get('notes') else ''}
    </details>
    """


def render_file_list(files: list) -> str:
    if not files: return ""
    return "<br>".join(f"<code>{esc(f)}</code>" for f in files)


def classify_divergence(text: str) -> str:
    if not text: return "PASS"
    t = text.lower()
    # An author-asserted "[RESOLVED ...]" or "[UPDATED ...]" prefix wins —
    # the prior divergence keywords ("divergence", "mismatch", "missing")
    # legitimately appear in the prose explaining what USED TO be wrong
    # and shouldn't keep the entry flagged after a fix.
    if t.lstrip().startswith(("[resolved", "[updated")):
        return "PASS"
    fail_signs = ["100x too big", "1000x", "missing", "wedge", "fails to author",
                  "scale bug", "structurally broken", "face-culled invisible",
                  "no equivalent", "broken"]
    if any(s in t for s in fail_signs):
        return "FAIL"
    warn_signs = ["divergence", "mismatch", "missing collision", "differs", "warn",
                  "scale", "cosmetic", "deprecated"]
    if any(s in t for s in warn_signs):
        return "WARN"
    return "PASS"


def build_topics_tab() -> str:
    eval_cycle = safe_read(EVAL_CYCLE_MD)
    sec3 = ""
    if eval_cycle:
        m = re.search(r"## 3\. Policy Interface.*?(?=\n## )", eval_cycle, re.DOTALL)
        if m: sec3 = m.group(0)
    return f"""
    <section>
      <h2>Topic surface — what flows between engine ↔ adapter ↔ policy ↔ controller</h2>
      <div class="card">
        {md_to_html(sec3)}
      </div>
    </section>
    """


def build_submission_tab() -> str:
    sub_path = AIC_REPO / "docs/submission.md"
    pol_path = AIC_REPO / "docs/policy.md"
    sub_md = safe_read(sub_path)
    pol_md = safe_read(pol_path)
    eval_cycle = safe_read(EVAL_CYCLE_MD)
    sec6 = ""
    if eval_cycle:
        m = re.search(r"## 6\. Submission Packaging.*?(?=\n## |\Z)", eval_cycle, re.DOTALL)
        if m: sec6 = m.group(0)
    return f"""
    <section>
      <h2>Submission contract — what a real policy must ship as</h2>
      <div class="card">
        {md_to_html(sec6)}
      </div>
      <h2>Official policy docs</h2>
      <div class="card">
        {md_to_html(pol_md[:8000], pol_path)}
      </div>
      <h2>Official submission docs</h2>
      <div class="card">
        {md_to_html(sub_md[:8000], sub_path)}
      </div>
    </section>
    """


def build_audit_tab(inv: list[dict] | None) -> str:
    if not inv:
        return "<section><div class='card'>Audit data missing.</div></section>"
    # Surface only FAIL + WARN entries
    flagged = []
    for e in inv:
        sev = classify_divergence((e.get("isaacsim", {}) or {}).get("known_divergences", ""))
        if sev in ("FAIL", "WARN"):
            flagged.append((sev, e))
    flagged.sort(key=lambda x: 0 if x[0] == "FAIL" else 1)

    rows = []
    for sev, e in flagged:
        sev_class = "fail" if sev == "FAIL" else "warn"
        diverg = (e.get("isaacsim", {}) or {}).get("known_divergences", "")
        rows.append(f"""
        <tr>
          <td><span class="badge {sev_class}">{sev}</span></td>
          <td><code>{esc(e['name'])}</code></td>
          <td>{esc(e.get('category', ''))}</td>
          <td>{esc(diverg)}</td>
        </tr>""")
    return f"""
    <section>
      <h2>Divergences flagged (Gazebo ↔ Isaac Sim)</h2>
      <p class="sub">{len(flagged)} of {len(inv)} entries have known divergences. Run <code>check_cad_dimensions.py &lt;asset&gt;</code> on any FAIL to verify scale.</p>
      <div class="card">
        <table class="audit-table">
          <thead><tr><th>Severity</th><th>Object</th><th>Category</th><th>Divergence detail</th></tr></thead>
          <tbody>{''.join(rows)}</tbody>
        </table>
      </div>
    </section>
    """


def build_tools_tab() -> str:
    return f"""
    <section>
      <h2>Reusable scripts</h2>
      <div class="card">
        <table>
          <tr><th>Script</th><th>Purpose</th><th>Usage</th></tr>
          <tr><td><code>exts/aic-dt/scripts/build_component_map.py</code></td><td>This page (regenerates from current AIC files)</td><td><code>python3 exts/aic-dt/scripts/build_component_map.py</code></td></tr>
          <tr><td><code>exts/aic-dt/scripts/check_cad_dimensions.py</code></td><td>Verify CAD dimensions across GLB JSON / Blender / USD / live MCP sources</td><td><code>python3 exts/aic-dt/scripts/check_cad_dimensions.py PATH [--compare PATH]</code></td></tr>
          <tr><td><code>exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh</code></td><td>Fire one or more trials end-to-end against Isaac Sim</td><td><code>bash run_aic_engine_against_isaac_sim.sh trial_1 --output-json=/tmp/out.json</code></td></tr>
          <tr><td><code>scripts/isaacsim_launch.sh</code></td><td>Launch / kill / status of Isaac Sim with aic-dt extension</td><td><code>bash scripts/isaacsim_launch.sh launch aic-dt</code></td></tr>
          <tr><td><code>scripts/prime_usd_cache.py</code></td><td>USD cache snapshot/restore (avoid cold-cook wedge)</td><td><code>python3 scripts/prime_usd_cache.py status</code></td></tr>
        </table>
      </div>
      <h2>Source-of-truth files</h2>
      <div class="card">
        <table>
          <tr><th>Topic</th><th>File</th></tr>
          <tr><td>World layout</td><td><code>~/Documents/aic/aic_description/world/aic.sdf</code></td></tr>
          <tr><td>Per-model SDFs</td><td><code>~/Documents/aic/aic_assets/models/*/model.sdf</code></td></tr>
          <tr><td>Task board URDF</td><td><code>~/Documents/aic/aic_description/urdf/task_board.urdf.xacro</code></td></tr>
          <tr><td>Trial configuration</td><td><code>~/Documents/aic/aic_engine/config/sample_config.yaml</code></td></tr>
          <tr><td>Anchor → port offsets (CV-detected)</td><td><code>~/Documents/aic/tools/anchor_target_offsets.yaml</code></td></tr>
          <tr><td>Eval engine</td><td><code>~/Documents/aic/aic_engine/src/aic_engine.cpp</code></td></tr>
          <tr><td>Scoring tier-2 + tier-3</td><td><code>~/Documents/aic/aic_scoring/src/ScoringTier2.cc</code></td></tr>
          <tr><td>Ground-truth cheat</td><td><code>~/Documents/aic/aic_example_policies/aic_example_policies/ros/CheatCode.py:217-223</code></td></tr>
          <tr><td>Isaac Sim extension</td><td><code>exts/aic-dt/aic_dt/extension.py</code></td></tr>
        </table>
      </div>
    </section>
    """


# ---------- HTML envelope ----------

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<title>AIC Component Map</title>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>
  // Pin to v10 (stable global API). startOnLoad=false because the Architecture
  // tab is hidden on initial load via display:none — mermaid fails to compute
  // SVG dimensions for hidden containers. We explicitly trigger re-render
  // on tab activation (see below).
  if (typeof mermaid !== 'undefined') {{
    mermaid.initialize({{ startOnLoad: false, theme: 'dark', securityLevel: 'loose' }});
  }}
  function renderMermaidIn(tabEl) {{
    if (typeof mermaid === 'undefined') return;
    const blocks = tabEl.querySelectorAll('.mermaid');
    blocks.forEach(b => {{
      // If already rendered (contains <svg>), reset to the source text so
      // mermaid.run can re-process. Source is stored in data-src on first
      // render.
      if (!b.dataset.src) b.dataset.src = b.textContent;
      else b.textContent = b.dataset.src;
      b.removeAttribute('data-processed');
    }});
    try {{ mermaid.run({{ nodes: blocks }}); }}
    catch (e) {{ console.error('mermaid:', e); }}
  }}
  window.addEventListener('DOMContentLoaded', () => {{
    // Render the diagram on first activation (Overview is active by default).
    renderMermaidIn(document.querySelector('.tab.active'));
  }});
</script>
<style>
  :root {{ --bg:#0e1116; --panel:#161b22; --border:#30363d; --fg:#c9d1d9; --accent:#58a6ff; --ok:#3fb950; --warn:#d29922; --fail:#f85149; }}
  * {{ box-sizing: border-box; }}
  body {{ margin:0; background:var(--bg); color:var(--fg); font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; line-height:1.5; }}
  nav.tabs {{ display:flex; background:var(--panel); border-bottom:1px solid var(--border); position:sticky; top:0; z-index:10; padding:0 16px; }}
  nav.tabs button {{ background:none; color:var(--fg); border:none; padding:14px 18px; cursor:pointer; font-size:14px; border-bottom:3px solid transparent; }}
  nav.tabs button.active {{ border-bottom-color:var(--accent); color:var(--accent); }}
  nav.tabs button:hover {{ background:#1f2630; }}
  .tab {{ display:none; padding:20px 32px; max-width:1400px; }}
  .tab.active {{ display:block; }}
  h1, h2 {{ color:var(--fg); margin-top:0; }}
  h2 {{ margin-top:24px; border-bottom:1px solid var(--border); padding-bottom:6px; }}
  h3, h4 {{ color:var(--accent); margin-top:16px; }}
  .card {{ background:var(--panel); border:1px solid var(--border); border-radius:6px; padding:16px 20px; margin:12px 0; }}
  .sub {{ color:#8b949e; font-size:13px; }}
  code, pre.code {{ background:#0d1117; padding:2px 6px; border-radius:3px; font-size:12.5px; }}
  pre.code {{ display:block; padding:12px; overflow-x:auto; }}
  table {{ width:100%; border-collapse:collapse; margin:8px 0; font-size:13px; }}
  table th, table td {{ text-align:left; padding:6px 10px; border-bottom:1px solid var(--border); vertical-align:top; }}
  table.md-table th {{ background:#21262d; }}
  details {{ margin:6px 0; }}
  details summary {{ cursor:pointer; padding:6px 10px; border-radius:4px; }}
  details summary:hover {{ background:#1f2630; }}
  details.obj-card {{ margin:4px 0; padding:0 8px; }}
  .obj-card.compact {{ padding:4px 8px; }}
  .obj-detail {{ display:flex; gap:24px; padding:8px 16px 16px; }}
  .obj-detail .col {{ flex:1; }}
  .obj-detail table {{ font-size:12px; }}
  .role {{ color:#8b949e; font-size:13px; margin-left:8px; }}
  .badge {{ display:inline-block; padding:2px 8px; border-radius:3px; font-size:11px; font-weight:600; margin-right:8px; min-width:44px; text-align:center; }}
  .badge.ok {{ background:#0c2d1a; color:var(--ok); }}
  .badge.warn {{ background:#3a2a06; color:var(--warn); }}
  .badge.fail {{ background:#3a0c0c; color:var(--fail); }}
  .divergence {{ margin:8px 16px; padding:8px 12px; border-radius:4px; font-size:13px; }}
  .divergence.fail {{ background:#3a0c0c33; border-left:3px solid var(--fail); }}
  .divergence.warn {{ background:#3a2a0633; border-left:3px solid var(--warn); }}
  .notes {{ margin:8px 16px; padding:8px 12px; background:#21262d; border-radius:4px; font-size:13px; color:#8b949e; }}
  hr {{ border:0; border-top:1px solid var(--border); margin:16px 0; }}
  a {{ color:var(--accent); }}
  ul {{ padding-left:20px; }}
  table.audit-table td:first-child {{ width:90px; }}
  img.md-img {{ max-width:100%; height:auto; border:1px solid var(--border); border-radius:4px; margin:8px 0; display:block; }}
  .md-img-missing {{ display:inline-block; padding:6px 12px; background:#3a2a06; color:var(--warn); border-radius:4px; font-size:12px; margin:8px 0; }}
  .md-img-placeholder {{ display:block; margin:8px 0; max-width:600px; }}
  .md-img-placeholder svg {{ width:100%; height:auto; border-radius:6px; }}
  .scene-tree {{ overflow:auto; max-height:80vh; }}
  .scene-tree .mermaid {{ min-width:1200px; }}
  table.legend-table {{ width:auto; }}
  table.legend-table td {{ padding:3px 8px; border:none; font-size:12px; }}
  .legend-swatch {{ display:inline-block; width:14px; height:14px; border-radius:3px; vertical-align:middle; }}
  /* Interactive tree UI */
  .legend-card {{ display:flex; gap:12px; flex-wrap:wrap; padding:10px 16px; }}
  .legend-pill {{ display:inline-flex; align-items:center; gap:6px; padding:4px 10px; background:#0d1117; border:1px solid var(--border); border-radius:14px; font-size:12px; }}
  .tree-layout {{ display:flex; gap:16px; margin-top:12px; }}
  .tree-panel {{ flex:0 0 auto; min-width:480px; max-width:50%; background:var(--panel); border:1px solid var(--border); border-radius:6px; padding:10px 12px; max-height:78vh; overflow:auto; }}
  .tree-detail {{ flex:1; background:var(--panel); border:1px solid var(--border); border-radius:6px; padding:14px 18px; max-height:78vh; overflow:auto; position:sticky; top:60px; }}
  .tree-detail table td:first-child {{ width:120px; color:#8b949e; }}
  .tree-search {{ width:100%; padding:8px 12px; background:#0d1117; color:var(--fg); border:1px solid var(--border); border-radius:4px; font-size:13px; margin-bottom:8px; }}
  .tree-search:focus {{ outline:none; border-color:var(--accent); }}
  .tree-controls {{ display:flex; gap:6px; flex-wrap:wrap; margin-bottom:10px; }}
  .tree-controls button {{ background:#0d1117; color:var(--fg); border:1px solid var(--border); border-radius:14px; padding:3px 10px; cursor:pointer; font-size:11px; }}
  .tree-controls button:hover {{ background:#1f2630; }}
  .tree-controls button.cat-filter {{ border-width:1px; border-style:solid; }}
  ul.tree-root, ul.tree-children {{ list-style:none; padding-left:0; margin:0; }}
  ul.tree-children {{ padding-left:18px; border-left:1px dashed #30363d; margin-left:7px; }}
  li.tree-node {{ margin:1px 0; }}
  .tree-row {{ display:flex; align-items:center; gap:6px; padding:3px 6px; border-radius:3px; cursor:pointer; font-size:13px; user-select:none; }}
  .tree-row:hover {{ background:#1f2630; }}
  .tree-row.selected {{ background:#1f3b5e; }}
  .tree-bar {{ display:inline-block; width:3px; height:14px; border-radius:1.5px; flex-shrink:0; }}
  .tree-toggle {{ display:inline-block; width:14px; text-align:center; cursor:pointer; color:#8b949e; font-size:10px; flex-shrink:0; }}
  .tree-name {{ font-family:-apple-system,sans-serif; color:var(--fg); }}
  .tree-inv-name {{ color:#8b949e; font-size:11px; font-style:italic; }}
  .tree-children-count {{ color:#6e7681; font-size:11px; margin-left:4px; }}
  /* D3 tree visualization */
  .d3-tree-layout {{ display:flex; gap:12px; margin-top:12px; height:800px; }}
  .d3-tree-svg-wrap {{ flex:1; background:var(--panel); border:1px solid var(--border); border-radius:6px; overflow:hidden; position:relative; }}
  .d3-tree-svg-wrap svg {{ display:block; cursor:grab; }}
  .d3-tree-svg-wrap svg:active {{ cursor:grabbing; }}
  .d3-tree-svg-wrap text {{ pointer-events:visiblePainted; }}
  .d3-tree-svg-wrap text:hover {{ fill:var(--accent); }}
  .d3-tree-svg-wrap circle:hover {{ stroke:var(--accent); stroke-width:3px; }}
  .tree-ctl-btn {{ background:#0d1117; color:var(--fg); border:1px solid var(--accent); border-radius:14px; padding:3px 12px; cursor:pointer; font-size:11px; margin-left:auto; }}
  .tree-ctl-btn:hover {{ background:#1f2630; }}
  /* Live Kit UI mirror */
  .live-toolbar {{ display:flex; gap:8px; align-items:center; margin-bottom:6px; }}
  .live-toolbar button {{ background:#0d1117; color:var(--fg); border:1px solid var(--accent); border-radius:14px; padding:5px 14px; cursor:pointer; font-size:12px; }}
  .live-toolbar button:hover {{ background:#1f2630; }}
  .health-indicator {{ margin-left:auto; font-size:12px; padding:3px 10px; border-radius:12px; background:#21262d; color:#8b949e; }}
  .health-indicator.ok {{ background:#0c2d1a; color:var(--ok); }}
  .health-indicator.err {{ background:#3a0c0c; color:var(--fail); }}
  details.live-frame {{ background:#0d1117; border:1px solid var(--border); border-radius:6px; padding:8px 12px; margin:8px 0; }}
  details.live-frame > summary {{ font-size:13px; color:var(--fg); padding:4px 0; }}
  .live-btn-row {{ display:flex; gap:8px; flex-wrap:wrap; padding:8px 0; }}
  .live-btn {{ background:#161b22; color:var(--fg); border:1px solid var(--border); border-radius:5px; padding:8px 12px; cursor:pointer; font-size:12px; display:flex; flex-direction:column; gap:2px; min-width:160px; text-align:left; transition:all 0.15s; }}
  .live-btn:hover {{ background:#1f2630; border-color:var(--accent); }}
  .live-btn .atom-label {{ font-weight:500; }}
  .live-btn .atom-name {{ font-family:monospace; font-size:10px; color:#6e7681; }}
  .live-btn.firing {{ background:#1f2940; border-color:var(--accent); }}
  .live-btn.ok {{ background:#0c2d1a; border-color:var(--ok); }}
  .live-btn.err {{ background:#3a0c0c; border-color:var(--fail); }}
  .live-result-pane {{ display:flex; gap:12px; margin-top:14px; }}
  .live-result-pane .card {{ flex:1; min-width:0; }}
  .live-result-pane pre {{ max-height:340px; overflow:auto; font-size:11px; }}
</style>
</head>
<body>
<nav class="tabs">
  <button class="tab-btn active" data-tab="overview">Overview</button>
  <button class="tab-btn" data-tab="architecture">Architecture</button>
  <button class="tab-btn" data-tab="tree">Isaac Sim Tree</button>
  <button class="tab-btn" data-tab="gz-tree">Gazebo Tree</button>
  <button class="tab-btn" data-tab="inventory">Scene Inventory</button>
  <button class="tab-btn" data-tab="topics">Topic Surface</button>
  <button class="tab-btn" data-tab="submission">Submission</button>
  <button class="tab-btn" data-tab="audit">Audit Findings</button>
  <button class="tab-btn" data-tab="live-kit">Live Kit UI</button>
  <button class="tab-btn" data-tab="tools">Tools</button>
</nav>
<div id="overview" class="tab active">{overview_tab}</div>
<div id="architecture" class="tab">{architecture_tab}</div>
<div id="tree" class="tab">{tree_tab}</div>
<div id="gz-tree" class="tab">{gz_tree_tab}</div>
<div id="inventory" class="tab">{inventory_tab}</div>
<div id="topics" class="tab">{topics_tab}</div>
<div id="submission" class="tab">{submission_tab}</div>
<div id="audit" class="tab">{audit_tab}</div>
<div id="live-kit" class="tab">{live_kit_tab}</div>
<div id="tools" class="tab">{tools_tab}</div>
<script>
  document.querySelectorAll('.tab-btn').forEach(btn => {{
    btn.addEventListener('click', () => {{
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      btn.classList.add('active');
      const activated = document.getElementById(btn.dataset.tab);
      activated.classList.add('active');
      // Re-render any mermaid diagrams now that the container is visible
      renderMermaidIn(activated);
    }});
  }});
</script>
<footer style="padding:16px 32px; color:#6e7681; font-size:12px; border-top:1px solid var(--border); margin-top:32px;">
  Generated by <code>exts/aic-dt/scripts/build_component_map.py</code> · Re-run after asset / config changes ·
  Data sources: <code>exts/aic-dt/docs/component-map/resources/aic_object_inventory.json</code>, <code>exts/aic-dt/docs/component-map/resources/aic_eval_cycle.md</code>, <code>~/Documents/aic/{{docs,.planning}}/*</code>
</footer>
</body></html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-open", action="store_true")
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = ap.parse_args()

    inv = safe_json(INVENTORY_JSON)
    print(f"[build_component_map] inventory entries: {len(inv) if isinstance(inv, list) else 'MISSING'}")
    print(f"[build_component_map] eval-cycle bytes:   {EVAL_CYCLE_MD.stat().st_size if EVAL_CYCLE_MD.exists() else 'MISSING'}")

    overview      = build_overview_tab()
    architecture  = build_architecture_tab()
    inventory     = build_inventory_tab(inv if isinstance(inv, list) else None)
    topics        = build_topics_tab()
    submission    = build_submission_tab()
    audit         = build_audit_tab(inv if isinstance(inv, list) else None)
    tools         = build_tools_tab()

    tree = build_scene_tree_tab(inv if isinstance(inv, list) else None)
    gz_tree = build_gazebo_tree_tab(inv if isinstance(inv, list) else None)
    live_kit = build_live_kit_tab()
    out = HTML_TEMPLATE.format(
        overview_tab=overview,
        architecture_tab=architecture,
        tree_tab=tree,
        gz_tree_tab=gz_tree,
        inventory_tab=inventory,
        topics_tab=topics,
        submission_tab=submission,
        audit_tab=audit,
        live_kit_tab=live_kit,
        tools_tab=tools,
    )
    args.output.write_text(out)
    print(f"[build_component_map] wrote: {args.output} ({len(out):,} bytes)")
    if not args.no_open:
        try:
            subprocess.run(["xdg-open", str(args.output)], check=False, timeout=5)
            print(f"[build_component_map] opened in default browser")
        except Exception as e:
            print(f"[build_component_map] couldn't auto-open: {e}; open manually: file://{args.output}")


if __name__ == "__main__":
    main()
