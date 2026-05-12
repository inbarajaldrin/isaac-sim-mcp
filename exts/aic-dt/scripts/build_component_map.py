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
INVENTORY_JSON = Path("/tmp/aic_object_inventory.json")
EVAL_CYCLE_MD = Path("/tmp/aic_eval_cycle.md")
DEFAULT_OUTPUT = Path("/tmp/aic_components.html")


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


def md_to_html(md: str) -> str:
    """Tiny markdown→HTML — only handles what we use: headings, lists, code,
    bold, italics, links, tables, hr. No external deps."""
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
    """Inline markdown: code, bold, italic, links."""
    s = esc(s)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
    s = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", s)
    s = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', s)
    return s


# ---------- tab content builders ----------

def build_overview_tab() -> str:
    overview = safe_read(AIC_REPO / "docs/overview.md")
    project = safe_read(AIC_REPO / ".planning/PROJECT.md")
    return f"""
    <section>
      <h2>What is AIC?</h2>
      <div class="card">
        {md_to_html(overview[:4000])}
      </div>
      <h2>aic_vision project framing</h2>
      <p class="sub">From <code>~/Documents/aic/.planning/PROJECT.md</code> — this is the *user's* project that builds on top of AIC.</p>
      <div class="card">
        {md_to_html(project[:5000])}
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
      <h2>Container topology (host + Docker)</h2>
      <div class="card">
        <div class="mermaid">
        graph LR
          subgraph "host"
            HZ[zenohd<br/>localhost:7447]
            HENG[aic_engine<br/>humble + rmw_zenoh]
            HADP[aic_adapter<br/>humble + rmw_zenoh]
            HMDL[aic_model<br/>kilted + CheatCode]
          end
          subgraph "isaac-sim-mcp (host)"
            ISIM[Isaac Sim Kit<br/>aic-dt extension<br/>MCP socket :8768]
            IPP[parity_publishers<br/>/joint_states /tf]
            ISP[scoring_publishers<br/>/scoring/insertion_event<br/>/scoring/tf]
          end
          ISIM --> IPP
          ISIM --> ISP
          IPP -- /joint_states /tf --> HZ
          ISP -- /scoring/* --> HZ
          HZ --> HENG
          HZ --> HADP
          HZ --> HMDL
          HENG -- InsertCable goal --> HMDL
          HADP -- Observation --> HMDL
          HMDL -- pose_commands --> HENG
        </div>
      </div>
      <h2>Single-trial state machine</h2>
      <div class="card">
        <p class="sub">Source: <code>aic_engine/src/aic_engine.cpp:682</code> (Engine::handle_trial)</p>
        {md_to_html(sec1)}
      </div>
      <h2>Ground-truth → CV pose-estimation seam</h2>
      <div class="card">
        <p class="sub"><strong>This is the architectural insertion point for the user's aic_vision CV pipeline.</strong></p>
        {md_to_html(sec4)}
      </div>
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
    sub_md = safe_read(AIC_REPO / "docs/submission.md")
    pol_md = safe_read(AIC_REPO / "docs/policy.md")
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
        {md_to_html(pol_md[:8000])}
      </div>
      <h2>Official submission docs</h2>
      <div class="card">
        {md_to_html(sub_md[:8000])}
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
          <tr><td><code>~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh</code></td><td>Launch / kill / status of Isaac Sim with aic-dt extension</td><td><code>bash isaacsim_launch.sh launch aic-dt</code></td></tr>
          <tr><td><code>~/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py</code></td><td>USD cache snapshot/restore (avoid cold-cook wedge)</td><td><code>python3 prime_usd_cache.py status</code></td></tr>
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
<script src="https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js"></script>
<script>mermaid.initialize({{startOnLoad: true, theme: 'default'}});</script>
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
</style>
</head>
<body>
<nav class="tabs">
  <button class="tab-btn active" data-tab="overview">Overview</button>
  <button class="tab-btn" data-tab="architecture">Architecture</button>
  <button class="tab-btn" data-tab="inventory">Scene Inventory</button>
  <button class="tab-btn" data-tab="topics">Topic Surface</button>
  <button class="tab-btn" data-tab="submission">Submission</button>
  <button class="tab-btn" data-tab="audit">Audit Findings</button>
  <button class="tab-btn" data-tab="tools">Tools</button>
</nav>
<div id="overview" class="tab active">{overview_tab}</div>
<div id="architecture" class="tab">{architecture_tab}</div>
<div id="inventory" class="tab">{inventory_tab}</div>
<div id="topics" class="tab">{topics_tab}</div>
<div id="submission" class="tab">{submission_tab}</div>
<div id="audit" class="tab">{audit_tab}</div>
<div id="tools" class="tab">{tools_tab}</div>
<script>
  document.querySelectorAll('.tab-btn').forEach(btn => {{
    btn.addEventListener('click', () => {{
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(btn.dataset.tab).classList.add('active');
      // Force mermaid to re-render diagrams on tab switch in case they didn't render initially
      try {{ mermaid.run(); }} catch(e) {{}}
    }});
  }});
</script>
<footer style="padding:16px 32px; color:#6e7681; font-size:12px; border-top:1px solid var(--border); margin-top:32px;">
  Generated by <code>exts/aic-dt/scripts/build_component_map.py</code> · Re-run after asset / config changes ·
  Data sources: <code>/tmp/aic_object_inventory.json</code>, <code>/tmp/aic_eval_cycle.md</code>, <code>~/Documents/aic/{{docs,.planning}}/*</code>
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

    out = HTML_TEMPLATE.format(
        overview_tab=overview,
        architecture_tab=architecture,
        inventory_tab=inventory,
        topics_tab=topics,
        submission_tab=submission,
        audit_tab=audit,
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
