#!/usr/bin/env python3
"""Offline grasp-reachability map for the SO-ARM101 randomize region.

Samples a dense grid of block positions (x, y) over the runtime randomize
region, and for each sample replays Gates A (workspace bbox) and B
(find_reachable_grasp_yaw) exactly as control_gui.py does at grasp time.
Produces a per-cell classification + heatmap + summary stats.

Does NOT run Gate C (self-collision) because that needs /check_state_validity
against a live planning scene. If A + B already reject a cell, Gate C
matters less. Gate C can be layered on in a second pass that queries the
live MoveIt service — trivial once the A+B map exists.

Scope: answers the question "what fraction of the randomize region is
geometrically unreachable at grasp time, and where?" Concretely bounds
Issues A (IK fails for close-in blocks) and the upper bound on Issue B
(self-collision is a subset of Gate C, which is a subset of A-then-B-pass
cells).

Usage:
    source ~/Projects/Exploring-VLAs/vla_SO-ARM101/install/setup.bash
    python3 compute_grasp_reachability.py                      # default 5mm grid
    python3 compute_grasp_reachability.py --resolution 0.002   # 2mm
    python3 compute_grasp_reachability.py --no-plot            # json only

Output (same directory as this script):
    grasp_reachability.json        # per-cell results
    grasp_reachability_h{N}mm.png  # one heatmap per approach height
    grasp_reachability_summary.txt # headline stats

Runtime: ~2s at 5mm grid, ~30s at 2mm grid (pure python, no ROS calls).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

# These come from the control package — must be importable.
try:
    from so_arm101_control.compute_workspace import geometric_ik
    from so_arm101_control.control_gui import (
        _GRASP_YAW_FALLBACK_OFFSETS,
        check_grasp_reachable,
    )
except ImportError as e:  # pragma: no cover
    sys.stderr.write(
        f"ImportError: {e}\n\n"
        "This script needs the so_arm101_control package on PYTHONPATH.\n"
        "Run: source ~/Projects/Exploring-VLAs/vla_SO-ARM101/install/setup.bash\n"
    )
    sys.exit(1)


# Mirror of the runtime randomize bounds (control_gui.py constants).
BLOCK_RANDOM_FORWARD = (0.10, 0.25)
BLOCK_RANDOM_LATERAL = (-0.10, 0.10)
# Block z (bottom) and half-height are approximated from a 2x4 lego.
# Grasp target z = block_z + half_height (tcp at block vertical mid-height).
DEFAULT_BLOCK_BOTTOM_Z = 0.008
DEFAULT_BLOCK_HALF_HEIGHT = 0.0095
DEFAULT_APPROACH_HEIGHTS_M = (0.0, 0.010, 0.020, 0.050)
DEFAULT_RESOLUTION_M = 0.005  # 5mm grid


def classify(x: float, y: float, approach_h: float,
             block_z_target: float) -> dict:
    """Run gates A + B on a single (x, y) at a given approach height.

    Returns a dict with:
      gate_a:  'pass' | reason string (from check_grasp_reachable per stage)
      gate_b:  yaw_used (deg) | 'all_yaws_failed'
      n_yaws_with_ik:  number of fallback yaws that returned solutions for
                       every stage (max = len of _GRASP_YAW_FALLBACK_OFFSETS)
      detail:  per-yaw-offset detail when gate_b fails, for forensics
    """
    target_z = block_z_target
    # Same logic as _prevalidate_and_execute: approach + final stages if
    # approach_h > 0, else final only.
    if approach_h > 0:
        stages = [
            ('approach', x, y, target_z + approach_h),
            ('final',    x, y, target_z),
        ]
    else:
        stages = [('final', x, y, target_z)]

    # Gate A — workspace bbox check per stage.
    gate_a_reason = None
    for stage, sx, sy, sz in stages:
        ok, reason = check_grasp_reachable(sx, sy, sz, ground_z=None)
        if not ok:
            gate_a_reason = f'{stage}: {reason}'
            break
    if gate_a_reason is not None:
        return {
            'gate_a': gate_a_reason,
            'gate_b': None,
            'n_yaws_with_ik': 0,
            'detail': [],
        }

    # Gate B — geometric_ik across fallback yaws. Requested yaw is 0 (pure
    # forward grasp); runtime computes a requested yaw from the block's qz
    # and pan, but 0 is representative for reachability purposes — any
    # object at this (x, y) that requires a different yaw will only need
    # to rotate within the same offset set.
    requested_yaw = 0.0
    yaws_with_all_stage_ik: list[float] = []
    per_offset_detail: list[dict] = []

    for offset in _GRASP_YAW_FALLBACK_OFFSETS:
        candidate = requested_yaw + offset
        stage_n_sols: dict[str, int] = {}
        all_ok = True
        for stage, sx, sy, sz in stages:
            sols = geometric_ik(sx, sy, sz, grasp_yaw=candidate)
            n = len(sols) if sols else 0
            stage_n_sols[stage] = n
            if n == 0:
                all_ok = False
        per_offset_detail.append({
            'offset_deg': round(math.degrees(offset), 1),
            'candidate_deg': round(math.degrees(candidate), 1),
            'stage_n_sols': stage_n_sols,
            'ok': all_ok,
        })
        if all_ok:
            yaws_with_all_stage_ik.append(candidate)

    if not yaws_with_all_stage_ik:
        return {
            'gate_a': 'pass',
            'gate_b': 'all_yaws_failed',
            'n_yaws_with_ik': 0,
            'detail': per_offset_detail,
        }

    # Pick yaw_used = first candidate that worked (matches runtime order).
    first_good = yaws_with_all_stage_ik[0]
    return {
        'gate_a': 'pass',
        'gate_b': round(math.degrees(first_good), 1),
        'n_yaws_with_ik': len(yaws_with_all_stage_ik),
        'detail': per_offset_detail,
    }


def build_grid(resolution: float):
    """Return list of (x, y) samples over the randomize region."""
    x0, x1 = BLOCK_RANDOM_FORWARD
    y0, y1 = BLOCK_RANDOM_LATERAL
    nx = int(round((x1 - x0) / resolution)) + 1
    ny = int(round((y1 - y0) / resolution)) + 1
    xs = [x0 + i * resolution for i in range(nx)]
    ys = [y0 + j * resolution for j in range(ny)]
    return xs, ys


def run_sweep(xs, ys, approach_heights, block_z_target):
    """Classify every (x, y, h) triple. Returns nested dict indexed by h."""
    results: dict[float, list[list[dict]]] = {}
    total = len(xs) * len(ys) * len(approach_heights)
    t0 = time.monotonic()
    done = 0
    for h in approach_heights:
        grid: list[list[dict]] = []
        for y in ys:
            row: list[dict] = []
            for x in xs:
                row.append(classify(x, y, h, block_z_target))
                done += 1
            grid.append(row)
        results[h] = grid
        dt = time.monotonic() - t0
        print(f'  approach_h={int(h*1000):>3d}mm: {len(xs)*len(ys):>5d} cells '
              f'done ({done}/{total}) elapsed={dt:.1f}s')
    return results


def summarize(results, xs, ys) -> dict:
    """Compute headline stats per approach height."""
    per_h: dict[str, dict] = {}
    for h, grid in results.items():
        n = sum(len(row) for row in grid)
        gate_a_fail = 0
        gate_b_fail = 0
        all_pass = 0
        yaw_hist: dict[str, int] = {}
        for row in grid:
            for cell in row:
                if cell['gate_a'] != 'pass':
                    gate_a_fail += 1
                elif cell['gate_b'] == 'all_yaws_failed':
                    gate_b_fail += 1
                else:
                    all_pass += 1
                    label = f'{cell["n_yaws_with_ik"]} yaws'
                    yaw_hist[label] = yaw_hist.get(label, 0) + 1
        per_h[f'{int(h*1000)}mm'] = {
            'n_cells': n,
            'pass': all_pass,
            'gate_a_fail': gate_a_fail,
            'gate_b_fail': gate_b_fail,
            'pass_frac': round(all_pass / max(n, 1), 4),
            'yaw_redundancy_hist': yaw_hist,
        }
    per_h['grid_meta'] = {
        'x_range_m': BLOCK_RANDOM_FORWARD,
        'y_range_m': BLOCK_RANDOM_LATERAL,
        'nx': len(xs), 'ny': len(ys),
        'resolution_m': round(xs[1] - xs[0], 6) if len(xs) > 1 else None,
    }
    return per_h


def write_heatmaps(results, xs, ys, outdir: Path):
    """Render one PNG per approach height. Green=pass, red=gate_b_fail,
    grey=gate_a_fail. Returns list of written paths."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print('[warn] matplotlib/numpy not available — skipping PNG')
        return []
    written = []
    for h, grid in results.items():
        arr = np.zeros((len(ys), len(xs)), dtype=np.int8)
        for j, row in enumerate(grid):
            for i, cell in enumerate(row):
                if cell['gate_a'] != 'pass':
                    arr[j, i] = 0   # grey
                elif cell['gate_b'] == 'all_yaws_failed':
                    arr[j, i] = 1   # red
                else:
                    # Shade by yaw redundancy (more yaws = more robust).
                    arr[j, i] = 2 + min(cell['n_yaws_with_ik'], 8)
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(
            arr, origin='lower',
            extent=[xs[0], xs[-1], ys[0], ys[-1]],
            cmap='RdYlGn', vmin=0, vmax=10, aspect='equal',
        )
        ax.set_title(
            f'SO-ARM101 grasp reachability — approach_h={int(h*1000)}mm\n'
            f'grey=Gate A fail, red=Gate B fail, green=pass (shade=yaw redundancy)'
        )
        ax.set_xlabel('x (forward, m)')
        ax.set_ylabel('y (lateral, m)')
        plt.colorbar(im, ax=ax, label='classification')
        out = outdir / f'grasp_reachability_h{int(h*1000)}mm.png'
        fig.savefig(out, dpi=120, bbox_inches='tight')
        plt.close(fig)
        written.append(out)
        print(f'  wrote {out.name}')
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION_M,
                    help='grid resolution in meters (default 0.005)')
    ap.add_argument('--block-z', type=float,
                    default=DEFAULT_BLOCK_BOTTOM_Z + DEFAULT_BLOCK_HALF_HEIGHT,
                    help='grasp target z (block mid-height) in meters')
    ap.add_argument('--heights', type=str,
                    default=','.join(str(h) for h in DEFAULT_APPROACH_HEIGHTS_M),
                    help='comma-separated approach heights in meters')
    ap.add_argument('--no-plot', action='store_true', help='skip PNG heatmaps')
    ap.add_argument('--outdir', type=str,
                    default=str(Path(__file__).parent))
    args = ap.parse_args()

    outdir = Path(args.outdir).resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    approach_heights = [float(s) for s in args.heights.split(',')]

    xs, ys = build_grid(args.resolution)
    print(f'Grid: {len(xs)} × {len(ys)} = {len(xs)*len(ys)} cells, '
          f'res={args.resolution*1000:.1f}mm, '
          f'block_z_target={args.block_z:.4f}m, '
          f'heights={[int(h*1000) for h in approach_heights]}mm')

    t0 = time.monotonic()
    results = run_sweep(xs, ys, approach_heights, args.block_z)
    elapsed = time.monotonic() - t0
    print(f'Sweep done in {elapsed:.1f}s')

    summary = summarize(results, xs, ys)
    summary['runtime_s'] = round(elapsed, 2)
    summary['block_z_target_m'] = args.block_z

    # JSON payload — full per-cell detail is verbose, so drop the per-cell
    # detail lists and keep only (gate_a, gate_b, n_yaws_with_ik) per cell.
    compact = {
        'summary': summary,
        'xs': xs,
        'ys': ys,
        'grids': {
            f'h_{int(h*1000)}mm': [[
                {k: v for k, v in cell.items() if k != 'detail'}
                for cell in row
            ] for row in grid]
            for h, grid in results.items()
        },
    }
    out_json = outdir / 'grasp_reachability.json'
    out_json.write_text(json.dumps(compact, indent=2))
    print(f'Wrote {out_json} ({out_json.stat().st_size/1024:.1f} KB)')

    if not args.no_plot:
        write_heatmaps(results, xs, ys, outdir)

    # Headline stats
    lines = ['SO-ARM101 grasp reachability summary', '=' * 40]
    for k, v in summary.items():
        if k in ('grid_meta', 'runtime_s', 'block_z_target_m'):
            continue
        lines.append(f'{k}: {v["pass"]}/{v["n_cells"]} pass '
                     f'({v["pass_frac"]*100:.1f}%) | '
                     f'Gate A fail: {v["gate_a_fail"]}, '
                     f'Gate B fail: {v["gate_b_fail"]}')
    lines.append('')
    lines.append(f'grid: {summary["grid_meta"]}')
    lines.append(f'runtime: {summary["runtime_s"]}s')
    summary_txt = outdir / 'grasp_reachability_summary.txt'
    summary_txt.write_text('\n'.join(lines))
    print()
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
