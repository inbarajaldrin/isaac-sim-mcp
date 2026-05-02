#!/usr/bin/env python3
"""diff_tf_tree.py — TF tree diff utility (Phase 1 / D-08)

Compares two GraphViz .gv files produced by `ros2 run tf2_tools view_frames` and
emits a frame-set + edge-set diff. Exits 0 on identical trees, 1 on any mismatch,
2 on argv usage error.

Usage:
    diff_tf_tree.py reference.gv sim.gv

The reference .gv is the live aic_eval Gazebo snapshot captured during Plan 01
(`.planning/phases/01-foundation-parity/aic_frames_live.gv`). The sim .gv is
freshly captured from Isaac Sim during /gsd-verify-work via `ros2 run tf2_tools
view_frames -o sim_frames` (writes `/tmp/sim_frames.gv`).

Reference: see .planning/phases/01-foundation-parity/01-RESEARCH.md
"Example 3: TF tree diff (Python)" lines 690-728 — this is the canonical
implementation. No graphviz / pygraphviz dependency; regex-only parse.
"""
import re
import sys


def parse_edges(path):
    """Return a set of (parent, child) tuples parsed from a .gv file.

    Matches lines of the form:  "parent" -> "child"[label=...]
    """
    edges = set()
    with open(path) as f:
        for line in f:
            m = re.match(r'\s*"([^"]+)"\s*->\s*"([^"]+)"', line)
            if m:
                edges.add((m.group(1), m.group(2)))
    return edges


def main():
    if len(sys.argv) != 3:
        print("usage: diff_tf_tree.py reference.gv sim.gv", file=sys.stderr)
        sys.exit(2)

    ref = parse_edges(sys.argv[1])
    sim = parse_edges(sys.argv[2])
    ref_frames = {f for e in ref for f in e}
    sim_frames = {f for e in sim for f in e}

    missing_frames = ref_frames - sim_frames
    extra_frames = sim_frames - ref_frames
    missing_edges = ref - sim
    extra_edges = sim - ref

    if not (missing_frames or extra_frames or missing_edges or extra_edges):
        print(f"PASS: TF trees match ({len(ref_frames)} frames, {len(ref)} edges)")
        sys.exit(0)

    print(f"FAIL: TF tree diff (reference={len(ref_frames)} frames/{len(ref)} edges, "
          f"sim={len(sim_frames)} frames/{len(sim)} edges)")
    if missing_frames:
        print(f"  Missing frames in sim: {sorted(missing_frames)}")
    if extra_frames:
        print(f"  Extra frames in sim:   {sorted(extra_frames)}")
    if missing_edges:
        print(f"  Missing edges in sim:  {sorted(missing_edges)}")
    if extra_edges:
        print(f"  Extra edges in sim:    {sorted(extra_edges)}")
    sys.exit(1)


if __name__ == "__main__":
    main()
