#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
SO-ARM101 Motion Fine-Sweep — closes the 50 Hz sampling gap in motion_verify.

For every captured motion (CSV + /tmp/arm_traj/*.json plan dump), compares:

  * coarse check: URDF collision mesh vs cup at each 50 Hz Isaac sample
  * fine check  : URDF collision mesh vs cup at linearly-interpolated joint
                  values between adjacent samples, at --resolution ms

Reports how many motions have sub-sample geometric penetrations that the
coarse check missed — a real failure mode MoveIt's post-check at ~12 ms
may also have missed.

Usage:
    scripts/motion_sweep.py --characterize   # table over all motions
    scripts/motion_sweep.py <csv>            # detail for one motion
    scripts/motion_sweep.py --resolution 1   # finer (default 2 ms)
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

# Add script dir for _mesh_sweep import
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _mesh_sweep import MeshSweeper, default_sweeper  # noqa: E402


def _read_csv(path: Path):
    with path.open() as f:
        return list(csv.DictReader(f))


def _plan_for(meta_path: Path):
    if not meta_path.exists():
        return None
    try:
        meta = json.loads(meta_path.read_text())
    except Exception:
        return None
    td = meta.get("traj_dump")
    if not td or not Path(td).exists():
        return None
    try:
        return json.loads(Path(td).read_text())
    except Exception:
        return None


def _sweep_one(csv_path: Path, sw: MeshSweeper, resolution_ms: int):
    rows = _read_csv(csv_path)
    if not rows:
        return None
    meta_path = csv_path.with_suffix(".json")
    plan = _plan_for(meta_path)
    return sw.sweep_motion(rows, plan, resolution_ms=resolution_ms)


def _format_cup_name(cup_drop_name: str, cup_positions: dict) -> str:
    """Resolve cup_drop_N → color name by x-coordinate (rough cue)."""
    xyz = cup_positions.get(cup_drop_name)
    if not xyz:
        return cup_drop_name
    # Nominal x: red=-0.049, green=+0.039, blue=+0.127
    x = xyz[0]
    if x < -0.01:
        return f"{cup_drop_name} (cup_red)"
    if x < 0.08:
        return f"{cup_drop_name} (cup_green)"
    return f"{cup_drop_name} (cup_blue)"


def characterize(log_root: Path, sw: MeshSweeper, resolution_ms: int):
    csvs = sorted(log_root.rglob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not csvs:
        print(f"No CSVs under {log_root}")
        return 0

    hdr = (f"{'file':<55} {'tag':<14} {'samples':>7} "
           f"{'coarse #':>8} {'fine #':>7} {'hidden':>7} "
           f"{'depth_mm':>9} {'min_clr_mm':>11} {'in_zone':<9} "
           f"{'worst_cup':<28}")
    print(hdr)
    print("-" * len(hdr))
    t0 = time.time()
    total = 0
    with_plan = 0
    coarse_motions = 0
    fine_motions = 0
    hidden_motions = 0
    for p in csvs:
        total += 1
        res = _sweep_one(p, sw, resolution_ms)
        if res is None or res.get("reason") == "no drop_data in plan dump" \
                or not res.get("cup_positions"):
            print(f"{p.name:<55} {'—':<14} {'—':>7} {'—':>8} {'—':>7} {'—':>7} "
                  f"{'(no plan)':>15}")
            continue
        with_plan += 1
        coarse = res["coarse_inside"]
        windows = res["fine_windows"]
        tag = "?"
        meta_path = p.with_suffix(".json")
        if meta_path.exists():
            try:
                tag = json.loads(meta_path.read_text()).get("motion_tag", "?")
            except Exception:
                pass
        # Worst depth across all windows (mm below rim at the intrusion)
        # dh_minus_r: negative means inside by that much
        worst_depth_mm = 0.0
        worst_cup = ""
        for (t_a, t_b, w) in windows:
            depth_mm = (-w["min_dh_minus_r_m"]) * 1000.0
            if depth_mm > worst_depth_mm:
                worst_depth_mm = depth_mm
                worst_cup = _format_cup_name(w["cup"], res["cup_positions"])
        if coarse:
            coarse_motions += 1
        if windows:
            fine_motions += 1
        # "hidden" = fine found intrusion that coarse did not catch at any sample
        # Use sample index overlap: a fine window [ta,tb] "hidden" if no coarse
        # sample in that interval flagged inside.
        hidden = 0
        coarse_ts = set(round(t, 3) for idx, t, w in coarse)
        for (t_a, t_b, w) in windows:
            # Coarse sample times are at 50 Hz → 20 ms spacing. Window hidden
            # if no coarse-flagged time falls within the window.
            any_coarse_in_window = any(t_a <= ct <= t_b for ct in coarse_ts)
            if not any_coarse_in_window:
                hidden += 1
        if hidden > 0:
            hidden_motions += 1
        mc = res.get("min_clearance", {})
        clr_mm = mc.get("min_dist_m", float("inf")) * 1000 if mc.get("link") else float("inf")
        in_zone = "🔥" if res.get("within_contact_zone") else "-"
        clr_str = f"{clr_mm:.2f}" if clr_mm != float("inf") else "—"
        flag = " ⚠" if hidden > 0 else ""
        print(f"{p.name:<55} {tag:<14} {res['coarse_samples']:>7} "
              f"{len(coarse):>8} {len(windows):>7} {hidden:>7} "
              f"{worst_depth_mm:>9.2f} {clr_str:>11} {in_zone:<9} "
              f"{worst_cup:<28}{flag}")

    dt = time.time() - t0
    print()
    print(f"Totals: {total} motions scanned in {dt:.1f}s "
          f"(resolution {resolution_ms} ms)")
    print(f"  with plan dumps     : {with_plan}")
    print(f"  coarse-flagged      : {coarse_motions}  (inside at ≥1 of 50 Hz samples)")
    print(f"  fine-flagged        : {fine_motions}  (inside at ≥1 interpolated instant)")
    print(f"  HIDDEN events       : {hidden_motions}  (fine saw it, coarse missed it)")
    if fine_motions:
        pct_hidden = 100.0 * hidden_motions / fine_motions
        print(f"  → {pct_hidden:.0f}% of fine-detected intrusions were invisible to 50 Hz check.")
    return 0


def detail(csv_path: Path, sw: MeshSweeper, resolution_ms: int):
    res = _sweep_one(csv_path, sw, resolution_ms)
    if res is None:
        print(f"{csv_path}: empty or unreadable")
        return 1
    if res.get("reason"):
        print(f"{csv_path}: {res['reason']}")
        return 1

    print("=" * 90)
    print(f"Fine-sweep: {csv_path.name}")
    print(f"  resolution: {resolution_ms} ms   coarse samples: {res['coarse_samples']}   "
          f"fine interp instants: {res['fine_samples']}")
    print(f"  cup positions (from plan dump):")
    for cup_name, (x, y, z) in res["cup_positions"].items():
        color = _format_cup_name(cup_name, res["cup_positions"])
        print(f"    {color:<28} ({x*1000:6.1f}, {y*1000:6.1f}, {z*1000:6.1f}) mm")
    print()

    if res["coarse_inside"]:
        print(f"Coarse (50 Hz) intrusions: {len(res['coarse_inside'])}")
        for idx, t_rel, w in res["coarse_inside"]:
            depth = -w["min_dh_minus_r_m"] * 1000.0
            z_mm = w["z_at_m"] * 1000.0
            print(f"  sample {idx:>4} (t={t_rel:.3f}s): "
                  f"{w['link']} in {w['cup']}  "
                  f"depth={depth:.2f} mm inside radius, z={z_mm:.2f} mm")
    else:
        print("Coarse (50 Hz) intrusions: none")
    print()

    windows = res["fine_windows"]
    if windows:
        print(f"Fine ({resolution_ms} ms) intrusion WINDOWS: {len(windows)}")
        for (t_a, t_b, w) in windows:
            dur_ms = (t_b - t_a) * 1000
            depth = -w["min_dh_minus_r_m"] * 1000.0
            z_mm = w["z_at_m"] * 1000.0
            print(f"  {t_a:.3f}s → {t_b:.3f}s ({dur_ms:.0f} ms): "
                  f"{w['link']} in {w['cup']}  "
                  f"peak depth={depth:.2f} mm, z_min={z_mm:.2f} mm")
        # Hidden — windows that no coarse sample flagged
        coarse_ts = [t for (_, t, _) in res["coarse_inside"]]
        hidden = [(t_a, t_b, w) for (t_a, t_b, w) in windows
                  if not any(t_a <= ct <= t_b for ct in coarse_ts)]
        if hidden:
            print()
            print(f"  → {len(hidden)} of {len(windows)} windows are HIDDEN "
                  f"(no coarse sample inside this window was flagged).")
    else:
        print(f"Fine ({resolution_ms} ms) intrusion windows: none")
    print()

    # PhysX contact-zone analysis — the key metric for cup knocks
    mc = res.get("min_clearance", {})
    zone_m = res.get("physx_contact_zone_m", 0.02)
    if mc and mc.get("link"):
        dist_mm = mc["min_dist_m"] * 1000
        zone_mm = zone_m * 1000
        in_zone = res.get("within_contact_zone", False)
        print(f"Min clearance (PhysX contact-zone analysis):")
        print(f"  closest approach : {dist_mm:.2f} mm  ({mc['link']} ↔ {mc['cup']} "
              f"at t={mc['t_rel']:.3f}s)")
        v = mc.get("vertex_world_xyz_m")
        if v:
            print(f"    vertex world xyz = ({v[0]*1000:.1f}, {v[1]*1000:.1f}, "
                  f"{v[2]*1000:.1f}) mm")
        print(f"  PhysX contactOffset (default)  : {zone_mm:.1f} mm")
        if in_zone:
            pct = (zone_mm - dist_mm) / zone_mm * 100
            sev = ("SEVERE" if dist_mm < 5 else
                   "MODERATE" if dist_mm < 10 else "MILD")
            print(f"  → {sev} intrusion into PhysX contact zone "
                  f"({pct:.0f}% deep)  ⚠")
            print(f"    PhysX generates contact forces here. Lateral arm motion")
            print(f"    translates these into cup displacement (see DEBUG-GUIDE § 4.3).")
        else:
            print(f"  → clear of PhysX contact zone; cup knock unlikely.")
    print()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Fine-resolution URDF-mesh-vs-cup sweep across captured motions")
    ap.add_argument("csv", nargs="?", help="One CSV to detail-report")
    ap.add_argument("--characterize", action="store_true",
                    help="Scan all motions under --log-dir (default mode when no CSV)")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/motion_logs"),
                    help="Log root")
    ap.add_argument("--resolution", type=int, default=2,
                    help="Fine interp step in ms (default 2)")
    args = ap.parse_args()

    sw = default_sweeper()

    if args.csv:
        return detail(Path(args.csv), sw, args.resolution)
    if args.characterize or not args.csv:
        return characterize(Path(args.log_dir), sw, args.resolution)
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
