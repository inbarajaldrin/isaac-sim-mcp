#!/usr/bin/env python3
# Reference: derived from session work 2026-05-17 parsing Gazebo /tf bags
# for the cable-fidelity audit (commits 5ba2bbe..e34ee5f).
"""Stream-extract the latest /tf + /tf_static state from an mcap bag.

Why this script exists (and why not just `ros2 bag`):

  The Gazebo /tf bags produced by `run_cheatcode.sh` have an anomalous
  trailer that breaks the seekable mcap reader path:

      File "mcap/stream_reader.py", line 174, in records
        raise RecordLengthLimitExceeded(opcode, length, ...)
      mcap.exceptions.RecordLengthLimitExceeded: unknown (opcode 148)
        record has length 6324563572405247233 that exceeds limit 4294967296

  `mcap.reader.make_reader()` calls `get_summary()` which seeks to the
  bag's footer and reads the trailer record — that's where it dies.

  Workaround: use `mcap.reader.NonSeekingReader` which skips the summary
  read and just streams forward through the chunks. Pair with
  `record_size_limit=2**62` to silence the length sanity check.

  Verified 2026-05-17 against /tmp/bag_trial_{1,3}_*/*.mcap and the
  dual-a4500 bags. Parses trial_1 (1.5 GB) in ~30 s; trial_3 (35 GB
  remote, 432 MB local) in ~4 min. Drops all topics except /tf and
  /tf_static for speed.

Output is a JSON file with two dicts (`dyn` and `stat`) keyed by child
frame_id; each value is a 4-tuple [parent, [tx,ty,tz], [qw,qx,qy,qz],
stamp_seconds]. Consume with `build_tf_tree.py` or
`compare_isaac_vs_gazebo_tf.py`.

Usage:

  python3 extract_tf_from_bag.py <bag.mcap> <out.json>

Example:

  python3 extract_tf_from_bag.py \\
      /tmp/bag_trial_3_20260517_082306_946/bag_0.mcap \\
      ~/Documents/isaac-sim-mcp/exts/aic-dt/docs/cable-fidelity/tf_trial_3.json

Pairs with: build_tf_tree.py (visualize as tree), compare_isaac_vs_gazebo_tf.py
(diff against live Isaac Sim state via MCP).
"""

from __future__ import annotations

import json
import sys

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory


def extract_tf(bag_path: str, out_json: str) -> dict:
    """Stream-decode /tf and /tf_static from `bag_path` and write the
    latest-wins snapshot to `out_json`. Returns the snapshot dict."""
    dyn: dict[str, list] = {}
    stat: dict[str, list] = {}
    n_total = 0

    with open(bag_path, "rb") as f:
        reader = NonSeekingReader(
            f,
            record_size_limit=2**62,
            decoder_factories=[DecoderFactory()],
        )
        for _schema, channel, _message, ros_msg in reader.iter_decoded_messages(
            topics=["/tf", "/tf_static"]
        ):
            target = stat if channel.topic == "/tf_static" else dyn
            for tr in ros_msg.transforms:
                child = tr.child_frame_id
                parent = tr.header.frame_id
                t = (
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z,
                )
                r = tr.transform.rotation
                q = (r.w, r.x, r.y, r.z)
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                target[child] = (parent, t, q, stamp)
            n_total += 1
            if n_total % 50000 == 0:
                print(
                    f"  [{n_total} tf msgs, dyn={len(dyn)} stat={len(stat)}]",
                    file=sys.stderr,
                )

    out = {
        "bag": bag_path,
        "n_tf_messages": n_total,
        "dyn": {c: list(v) for c, v in dyn.items()},
        "stat": {c: list(v) for c, v in stat.items()},
    }
    with open(out_json, "w") as f:
        json.dump(out, f)
    print(
        f"DONE: {n_total} tf msgs, dyn={len(dyn)} stat={len(stat)} → {out_json}",
        file=sys.stderr,
    )
    return out


def main(argv):
    if len(argv) != 3:
        print(f"Usage: {argv[0]} <bag.mcap> <out.json>", file=sys.stderr)
        return 1
    extract_tf(argv[1], argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
