#!/usr/bin/env python3
# Reference: derived from session work 2026-05-17 visualizing Gazebo /tf
# trees for the cable-fidelity comparison (commits 5ba2bbe..e34ee5f).
"""Pretty-print a TF tree from an extract_tf_from_bag.py JSON snapshot.

For every frame in the snapshot, walks parent chain to a world root
(parent ∈ {"world", "", "map"}) and prints both the LOCAL transform (as
authored by the publisher) and the WORLD transform (composed via
parent walk).

The world resolver uses Hamilton quaternion composition with row-vector
USD convention: parent rotates the child's local translation BEFORE
adding parent's world translation, and world_quat = parent_world_quat *
local_quat. Memoized — O(n) per snapshot.

Output format:

  world (ROOT)
  ├─ [S] aic_world
          local=(+0.0000,+0.0000,+0.0000)
          world pos=(+0.0000,+0.0000,+0.0000) rpy=(  +0.0,  +0.0,  +0.0)°
    ├─ [D] cable_0
            local=(+0.1843,-0.0142,+1.3755)
            world pos=(+0.1843,-0.0142,+1.3755) rpy=( +27.6, -12.4, +75.3)°
    ...

  [S] = static transform (/tf_static)
  [D] = dynamic transform (/tf)

Usage:

  python3 build_tf_tree.py <tf_snapshot.json> <label>

  python3 build_tf_tree.py docs/cable-fidelity/tf_trial_3.json trial_3 \\
      > docs/cable-fidelity/tree_trial_3.txt

Pairs with: extract_tf_from_bag.py (produces input JSON),
compare_isaac_vs_gazebo_tf.py (diff live Isaac Sim).
"""

from __future__ import annotations

import json
import math
import sys


def quat_mul(q1, q2):
    """Hamilton wxyz quaternion product."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_rotate(q, v):
    """Rotate a 3-vector by a wxyz quat. Column-vector form: v' = q v q^-1."""
    qw, qx, qy, qz = q
    t = (
        2 * (qy * v[2] - qz * v[1]),
        2 * (qz * v[0] - qx * v[2]),
        2 * (qx * v[1] - qy * v[0]),
    )
    return (
        v[0] + qw * t[0] + (qy * t[2] - qz * t[1]),
        v[1] + qw * t[1] + (qz * t[0] - qx * t[2]),
        v[2] + qw * t[2] + (qx * t[1] - qy * t[0]),
    )


def compose(parent_t, parent_q, local_t, local_q):
    """Compose parent ∘ local. parent_q rotates local_t, then add."""
    rotated = quat_rotate(parent_q, local_t)
    wt = (parent_t[0] + rotated[0], parent_t[1] + rotated[1], parent_t[2] + rotated[2])
    wq = quat_mul(parent_q, local_q)
    return wt, wq


def quat_to_rpy(q):
    """wxyz → (roll, pitch, yaw) in degrees, intrinsic ZYX (URDF convention)."""
    w, x, y, z = q
    return (
        math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
        math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
        math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
    )


def load_snapshot(path: str):
    """Return (edges_dict, children_of_dict) from an extract_tf_from_bag JSON.

    edges[child] = (parent, t, q, stamp, kind) where kind ∈ {"stat","dyn"}.
    children_of[parent] = list of child frame_ids.
    """
    data = json.load(open(path))
    edges = {}
    for c, (p, t, q, s) in data["stat"].items():
        edges[c] = (p, tuple(t), tuple(q), s, "stat")
    for c, (p, t, q, s) in data["dyn"].items():
        # Dynamic overrides static (matches tf2 lookup precedence)
        edges[c] = (p, tuple(t), tuple(q), s, "dyn")
    children_of = {}
    for c, (p, *_) in edges.items():
        children_of.setdefault(p, []).append(c)
    return edges, children_of


def world_resolver(edges):
    """Build a memoized resolver: child_frame_id → (world_t, world_q) or None."""
    mem = {}

    def world(c):
        if c in mem:
            return mem[c]
        if c not in edges:
            return None
        p, t, q, _, _ = edges[c]
        if p in ("world", "", "map"):
            mem[c] = (t, q)
            return (t, q)
        pw = world(p)
        if pw is None:
            return None
        pt, pq = pw
        wt, wq = compose(pt, pq, t, q)
        mem[c] = (wt, wq)
        return (wt, wq)

    return world


def render_tree(snapshot_path: str, label: str, out=sys.stdout) -> None:
    edges, children_of = load_snapshot(snapshot_path)
    world = world_resolver(edges)

    all_children = set(edges.keys())
    parents = {p for c, (p, *_) in edges.items()}
    roots = parents - all_children

    print(f"\n{'=' * 80}", file=out)
    print(f"TF TREE — {label}", file=out)
    print(f"{'=' * 80}\n", file=out)

    def render(node, depth=0):
        if depth == 0:
            print(f"{node} (ROOT)", file=out)
        for c in sorted(children_of.get(node, [])):
            p, t, q, _, k = edges[c]
            w = world(c)
            if w:
                wt, wq = w
                rpy = quat_to_rpy(wq)
                pos_str = f"pos=({wt[0]:+.4f},{wt[1]:+.4f},{wt[2]:+.4f})"
                rpy_str = f"rpy=({rpy[0]:+6.1f},{rpy[1]:+6.1f},{rpy[2]:+6.1f})°"
            else:
                pos_str = " "
                rpy_str = " "
            indent = "  " * depth + "├─ "
            kmark = "S" if k == "stat" else "D"
            print(f"{indent}[{kmark}] {c}", file=out)
            print(
                f"{'  ' * (depth + 1)}      local=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})",
                file=out,
            )
            print(f"{'  ' * (depth + 1)}      world {pos_str} {rpy_str}", file=out)
            render(c, depth + 1)

    for r in sorted(roots):
        render(r)


def main(argv):
    if len(argv) != 3:
        print(f"Usage: {argv[0]} <tf_snapshot.json> <label>", file=sys.stderr)
        return 1
    render_tree(argv[1], argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
