#!/usr/bin/env python3
# Reference: derived from session work 2026-05-17 cross-checking Isaac Sim
# scene against Gazebo /tf bags (commits 5ba2bbe..e34ee5f).
"""Build a markdown comparison table: Gazebo /tf ground truth vs Isaac Sim
live world transforms.

Joins two data sources via a hand-curated mapping:

  - Gazebo: trial_1 + trial_3 TF snapshots produced by
            extract_tf_from_bag.py (default location:
            exts/aic-dt/docs/cable-fidelity/tf_trial_{1,3}.json).

  - Isaac Sim: live state probed via the aic-dt MCP socket, using the
               scale-corrected quat extractor from probe_isaac_world_tree.py.

Output (stdout, markdown):

  === TRIAL_3 COMPARISON TABLE ===

  | Entity | Gazebo world pos | Gazebo rpy° | Isaac world pos | Isaac rpy° | Δ pos (m) |
  |--------|------------------|-------------|------------------|------------|-----------|
  | gripper/tcp | (...) | (...) | (...) | (...) | (Δx, Δy, Δz) |
  ...

Usage:

  # 1. Make sure Isaac Sim is running with trial loaded:
  #    bash <repo-root>/scripts/isaacsim_launch.sh launch aic-dt
  #    python3 -c "..." # quick_start + load_trial
  #
  # 2. Run the compare:
  python3 compare_isaac_vs_gazebo_tf.py \\
      exts/aic-dt/docs/cable-fidelity/tf_trial_1.json \\
      exts/aic-dt/docs/cable-fidelity/tf_trial_3.json

Pairs with: extract_tf_from_bag.py (input JSON producer),
probe_isaac_world_tree.py (live probe library — imported here).
"""

from __future__ import annotations

import json
import math
import os
import socket
import sys
from typing import List, Tuple

# Reuse the scale-corrected probe + MCP plumbing from the sibling probe.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_isaac_world_tree import probe_isaac_world  # noqa: E402

from build_tf_tree import load_snapshot, world_resolver, quat_to_rpy  # noqa: E402


# (display_label, gz_frame_trial_1, gz_frame_trial_3, isaac_sim_prim_path)
# Use None for entries that don't have a Gazebo or Isaac counterpart.
# HELD/FAR rows are filled in per-trial below since they swap by variant.
MAPPINGS = [
    ("gripper/tcp",                 "gripper/tcp",                   "gripper/tcp",
     "/World/UR5e/aic_unified_robot/gripper_tcp"),
    ("gripper/hande_base_link",     "gripper/hande_base_link",       "gripper/hande_base_link",
     "/World/UR5e/aic_unified_robot/gripper_hande_base_link"),
    ("gripper finger l",            "gripper/hande_finger_link_l",   "gripper/hande_finger_link_l",
     "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l"),
    ("gripper finger r",            "gripper/hande_finger_link_r",   "gripper/hande_finger_link_r",
     "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_r"),
    ("tool0",                       "tool0",                         "tool0",
     "/World/UR5e/aic_unified_robot/tool0"),
    ("base_link",                   "base_link",                     "base_link",
     "/World/UR5e/aic_unified_robot/base_link"),
    ("cable root",                  "cable_0",                       "cable_1",
     "/World/UR5e/cable"),
    ("cable connector_0",           "cable_0/cable_connection_0",    "cable_1/cable_connection_0",
     None),
    ("cable connector_1",           "cable_0/cable_connection_1",    "cable_1/cable_connection_1",
     None),
    ("HELD plug (Gazebo link)",
     "cable_0/sfp_module_link",     "cable_1/sc_plug_link",          None),
    ("FAR plug (Gazebo link)",
     "cable_0/sc_plug_link",        "cable_1/sfp_module_link",       None),
    # HELD/FAR Isaac visuals: variant-aware, filled below.
    ("HELD VISUAL (Isaac)",         None, None, None),
    ("FAR VISUAL (Isaac)",          None, None, None),
    ("rope link_0",                 None, None,
     "/World/UR5e/cable/Rope/Rope/link_0"),
    ("rope link_10",                None, None,
     "/World/UR5e/cable/Rope/Rope/link_10"),
    ("rope link_20",                None, None,
     "/World/UR5e/cable/Rope/Rope/link_20"),
    ("task board base",             "task_board/task_board_base_link",
     "task_board/task_board_base_link", None),
    ("SC port link",                "task_board/sc_port_0/sc_port_link",
     None, None),
]


def _fmt_pos(t):
    if t is None:
        return "—"
    return f"({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f})"


def _fmt_rpy(q):
    if q is None:
        return "—"
    r, p, y = quat_to_rpy(q)
    return f"({r:+6.1f},{p:+6.1f},{y:+6.1f})"


def _delta(g, i):
    if g is None or i is None:
        return "—"
    return f"({i[0] - g[0]:+.3f}, {i[1] - g[1]:+.3f}, {i[2] - g[2]:+.3f})"


def _resolve_gz(snap_path: str, frames: List[str]) -> dict:
    """Resolve a list of frames to world (pos, quat_wxyz) via the snapshot."""
    edges, _ = load_snapshot(snap_path)
    world = world_resolver(edges)
    return {f: world(f) for f in frames}


def _probe_isaac(paths: List[str]) -> dict:
    """Probe live Isaac Sim for the given prim paths. Returns
    {prim_path: [tx, ty, tz, qw, qx, qy, qz] or None}.

    Splits the label off by the trailing " pos=(" or " MISSING " sentinel
    instead of a fixed-width slice — works for paths of any length."""
    entries = [(p, p) for p in paths]
    text = probe_isaac_world(entries)
    out = {}
    for line in text.splitlines():
        if not line.strip():
            continue
        if " MISSING " in line:
            path = line.split(" MISSING ", 1)[0].strip()
            out[path] = None
            continue
        if " pos=(" not in line:
            continue
        path, rest = line.split(" pos=(", 1)
        path = path.strip()
        try:
            pos_end = rest.index(")")
            pos = tuple(float(x) for x in rest[:pos_end].split(","))
            q_idx = rest.index("quat_wxyz=(") + 11
            q_end = rest.index(")", q_idx)
            q = tuple(float(x) for x in rest[q_idx:q_end].split(","))
            out[path] = [pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3]]
        except (ValueError, IndexError):
            out[path] = None
    return out


def build_compare(trial_1_json: str, trial_3_json: str, out=sys.stdout) -> None:
    # Gather every Isaac path we want to probe
    isaac_paths = sorted({m[3] for m in MAPPINGS if m[3]})
    isaac_paths += [
        "/World/UR5e/cable/sc_plug_visual",
        "/World/UR5e/cable/sfp_module_visual",
    ]
    isaac = _probe_isaac(isaac_paths)

    print("\n=== ISAAC SIM (current — trial loaded) ===", file=out)
    for path in sorted(isaac):
        v = isaac[path]
        if v is None:
            print(f"  {path:<60s} MISSING", file=out)
        else:
            t = (v[0], v[1], v[2])
            q = (v[3], v[4], v[5], v[6])
            print(f"  {path:<60s} pos={_fmt_pos(t)} rpy={_fmt_rpy(q)}", file=out)

    for label, snap_path in (("trial_1", trial_1_json), ("trial_3", trial_3_json)):
        frames = []
        for m in MAPPINGS:
            frames.append(m[1] if label == "trial_1" else m[2])
        frames = [f for f in frames if f]
        gz = _resolve_gz(snap_path, frames)

        print(f"\n\n=== {label.upper()} COMPARISON TABLE ===\n", file=out)
        print(
            "| Entity | Gazebo world pos | Gazebo rpy° | Isaac world pos | Isaac rpy° | Δ pos (m) |",
            file=out,
        )
        print("|--------|------------------|-------------|------------------|------------|-----------|", file=out)

        for m in MAPPINGS:
            disp = m[0]
            gz_name = m[1] if label == "trial_1" else m[2]
            isaac_path = m[3]
            if disp == "HELD VISUAL (Isaac)":
                isaac_path = (
                    "/World/UR5e/cable/sfp_module_visual"
                    if label == "trial_1"
                    else "/World/UR5e/cable/sc_plug_visual"
                )
                gz_name = (
                    "cable_0/sfp_module_link" if label == "trial_1" else "cable_1/sc_plug_link"
                )
            elif disp == "FAR VISUAL (Isaac)":
                isaac_path = (
                    "/World/UR5e/cable/sc_plug_visual"
                    if label == "trial_1"
                    else "/World/UR5e/cable/sfp_module_visual"
                )
                gz_name = (
                    "cable_0/sc_plug_link" if label == "trial_1" else "cable_1/sfp_module_link"
                )

            gz_v = gz.get(gz_name) if gz_name else None
            isaac_v = isaac.get(isaac_path) if isaac_path else None
            isaac_t = (isaac_v[0], isaac_v[1], isaac_v[2]) if isaac_v else None
            isaac_q = (isaac_v[3], isaac_v[4], isaac_v[5], isaac_v[6]) if isaac_v else None
            print(
                f"| {disp:<35s} "
                f"| {_fmt_pos(gz_v[0]) if gz_v else '—'} "
                f"| {_fmt_rpy(gz_v[1]) if gz_v else '—'} "
                f"| {_fmt_pos(isaac_t)} "
                f"| {_fmt_rpy(isaac_q)} "
                f"| {_delta(gz_v[0] if gz_v else None, isaac_t)} |",
                file=out,
            )


def main(argv):
    if len(argv) != 3:
        print(f"Usage: {argv[0]} <trial_1.json> <trial_3.json>", file=sys.stderr)
        return 1
    build_compare(argv[1], argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
