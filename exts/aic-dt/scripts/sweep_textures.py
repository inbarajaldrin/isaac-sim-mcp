#!/usr/bin/env python3
"""sweep_textures.py — Texture / MDL sweep harness (Phase 1 / D-07 / TEX-03).

Triggers a scene load via MCP `quick_start` (or skip with --skip-load), then
greps the newest Kit log for texture / MDL / asset-resolution warnings, and
appends a Markdown table row per hit to `exts/aic-dt/docs/texture-sweep.md`.

PATTERNS (case-insensitive, regex):
  - D-07 baseline:  MDL, texture, missing, pink, fallback, not\\s*found
  - Augmented (Isaac-Sim asset failure surface — sc_port_visual.usd's broken
    sub-references ISO_4762 / V1015120 missing USDs surfaced during RESEARCH
    vendoring manifest):
        Failed\\s+to\\s+open, Could\\s+not\\s+open, unresolved,
        reference.*invalid

Output format per D-07 / TEX-03:
    | Asset | Problem | Fix | Status |

Status starts as "unresolved"; humans (or follow-up plans) annotate fixes.
The doc is APPEND-only — each sweep run prepends a "## Sweep run <ISO>" header
preserving prior runs as audit trail.

Usage:
    python3 exts/aic-dt/scripts/sweep_textures.py
    python3 exts/aic-dt/scripts/sweep_textures.py --skip-load
    python3 exts/aic-dt/scripts/sweep_textures.py --port 8768 --out path/to/out.md

Real Kit log location (per CLAUDE.md):
    ~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_*.log
"""
import argparse
import datetime
import glob
import json
import os
import re
import socket
import sys


KIT_LOG_GLOB = os.path.expanduser(
    '~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_*.log'
)

PATTERNS = [
    r'MDL',
    r'texture',
    r'missing',
    r'pink',
    r'fallback',
    r'not\s*found',
    # Augmented (Isaac Sim asset-failure surface — pre-emptive coverage for
    # sc_port_visual.usd's broken props sub-references; per CONTEXT.md D-07
    # "refine after first run", these are known-needed up front):
    r'Failed\s+to\s+open',
    r'Could\s+not\s+open',
    r'unresolved',
    r'reference.*invalid',
]

COMPILED = [re.compile(p, re.IGNORECASE) for p in PATTERNS]


def send_cmd(port, cmd_type, params=None, timeout=300):
    """MCP socket round-trip — connects to localhost:port, sends one JSON
    command, reads chunks until JSON parses (no length framing).

    Mirrors ~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py.
    """
    msg = json.dumps({"type": cmd_type, "params": params or {}})
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect(("localhost", port))
    sock.sendall(msg.encode("utf-8"))
    data = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
        try:
            json.loads(data.decode("utf-8"))
            break
        except json.JSONDecodeError:
            continue
    sock.close()
    return json.loads(data.decode("utf-8"))


def find_newest_kit_log():
    """Return the path to the newest kit_*.log file by mtime. Raises
    FileNotFoundError if the glob is empty."""
    candidates = glob.glob(KIT_LOG_GLOB)
    if not candidates:
        raise FileNotFoundError(
            f"No Kit log found matching: {KIT_LOG_GLOB}\n"
            "Has Isaac Sim ever been launched on this host?"
        )
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def grep_warnings(log_path):
    """Read log_path and return a list of (line_no, line_stripped) tuples for
    every line that matches at least one of PATTERNS (case-insensitive).
    Skips overly-long lines defensively to avoid pathological cases."""
    hits = []
    with open(log_path, encoding='utf-8', errors='replace') as f:
        for i, line in enumerate(f, start=1):
            if len(line) > 4000:
                continue
            for rx in COMPILED:
                if rx.search(line):
                    hits.append((i, line.rstrip('\n')))
                    break
    return hits


def classify(line):
    """Heuristic — extract (asset, problem, raw_line) from a log line.

    asset:    first match of `assets/<...>` substring or `/World/<...>` prim path,
              else "<unknown>"
    problem:  the first PATTERNS regex that matches the line (uncompiled string)
    raw:      the full log line, trimmed
    """
    asset = '<unknown>'
    m = re.search(r'(assets/[^\s\'"\)]+|/World/[^\s\'"\)]+)', line)
    if m:
        asset = m.group(1)

    problem = ''
    for rx, pat_src in zip(COMPILED, PATTERNS):
        if rx.search(line):
            problem = pat_src
            break

    return (asset, problem, line.strip())


def emit_markdown(out_path, hits, log_path):
    """Append a sweep-run section to out_path. Creates the file with a header
    if it doesn't exist (uses the same stub format as docs/texture-sweep.md)."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    file_existed = os.path.exists(out_path)

    with open(out_path, 'a', encoding='utf-8') as f:
        if not file_existed:
            f.write("# Texture / MDL Sweep Log (Phase 1 D-07 / TEX-03)\n\n")
            f.write(
                "Each section below is a sweep run. "
                "Asset = path or prim. Status: unresolved | resolved | accepted-cosmetic.\n\n"
            )

        ts = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        f.write(f"## Sweep run {ts}\n\n")
        f.write(f"Source log: `{log_path}`\n")
        f.write(f"Hit count: {len(hits)}\n\n")

        if not hits:
            f.write("_No matching warnings — clean run._\n\n")
            return

        f.write("| Asset | Problem | Fix | Status |\n")
        f.write("|-------|---------|-----|--------|\n")
        seen = set()
        for line_no, line in hits:
            asset, problem, raw = classify(line)
            # de-dupe identical (asset, problem) rows
            key = (asset, problem)
            if key in seen:
                continue
            seen.add(key)
            # Truncate raw line for the Fix column hint; keep it terse.
            hint = raw.replace('|', '\\|')
            if len(hint) > 200:
                hint = hint[:197] + '...'
            f.write(f"| `{asset}` | `{problem}` | _{hint}_ | unresolved |\n")
        f.write("\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--port', type=int, default=8768,
                    help='MCP socket port on localhost (default 8768 for aic-dt)')
    ap.add_argument('--out', default='exts/aic-dt/docs/texture-sweep.md',
                    help='Markdown output path (default exts/aic-dt/docs/texture-sweep.md)')
    ap.add_argument('--skip-load', action='store_true',
                    help='Skip the quick_start MCP call; just grep current Kit log')
    ap.add_argument('--timeout', type=int, default=300,
                    help='quick_start MCP timeout in seconds (default 300)')
    args = ap.parse_args()

    if not args.skip_load:
        print(f"[sweep] Triggering quick_start via MCP on localhost:{args.port}...")
        try:
            resp = send_cmd(args.port, 'quick_start', {}, timeout=args.timeout)
        except Exception as e:
            print(f"[sweep] FAIL: MCP quick_start error: {e}", file=sys.stderr)
            sys.exit(1)
        status = resp.get('status') or resp.get('result', {}).get('status')
        if status not in ('success', 'ok'):
            print(f"[sweep] FAIL: quick_start status={status}: {resp}", file=sys.stderr)
            sys.exit(1)
        print("[sweep] quick_start success.")

    try:
        log_path = find_newest_kit_log()
    except FileNotFoundError as e:
        print(f"[sweep] {e}", file=sys.stderr)
        sys.exit(1)
    print(f"[sweep] Scanning newest Kit log: {log_path}")

    hits = grep_warnings(log_path)
    print(f"[sweep] {len(hits)} matching lines found.")

    emit_markdown(args.out, hits, log_path)
    print(f"[sweep] Appended sweep section to: {args.out}")

    # Exit 0 always — D-07 says iterate; the verify harness decides pass/fail.
    sys.exit(0)


if __name__ == '__main__':
    main()
