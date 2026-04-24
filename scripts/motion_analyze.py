#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
SO-ARM101 Motion Telemetry Analyzer — read-side for motion_logger.py output.

Answers the questions:
  * Did Isaac's physics track the plan during this motion? Per joint?
  * Did any joint lag *differentially* vs. others? (The mechanism for un-planned
    arm configurations and unintended cup contact.)
  * Did the TCP come close to any cup? Did any cup actually move (collision)?
  * How did the 3-way state (plan reference, ros2_control feedback, Isaac
    physics) agree at each instant?

Usage:
    scripts/motion_analyze.py <csv>           # full report on one motion
    scripts/motion_analyze.py --latest        # analyze most recent motion
    scripts/motion_analyze.py --scan [DIR]    # list + summary of all motions
    scripts/motion_analyze.py --json <csv>    # print meta JSON only

Exit codes: 0 OK, 1 file/io error.
"""

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path


ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _latest_csv(root: Path):
    csvs = sorted(root.rglob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return csvs[0] if csvs else None


def _read_csv(path: Path):
    rows = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def _f(v):
    try:
        return float(v) if v not in ("", None) else None
    except Exception:
        return None


def _analyze(path: Path, verbose=True):
    rows = _read_csv(path)
    if not rows:
        print(f"{path}: no rows"); return 1

    meta_path = path.with_suffix(".json")
    meta = {}
    if meta_path.exists():
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            pass

    n = len(rows)
    t0 = float(rows[0]["t_wall"])
    dur = float(rows[-1]["t_wall"]) - t0

    # ─── Peak + mean per joint: ref_vel, isaac_lag, ros2_err ───
    stats = {}
    for j in ARM:
        lags = [_f(r.get(f"isaac_lag_{j}")) for r in rows]
        lags = [abs(x) for x in lags if x is not None]
        errs = [_f(r.get(f"err_pos_{j}")) for r in rows]
        errs = [abs(x) for x in errs if x is not None]
        rvs = [_f(r.get(f"ref_vel_{j}")) for r in rows]
        rvs = [abs(x) for x in rvs if x is not None]
        stats[j] = {
            "peak_isaac_lag_rad": max(lags) if lags else None,
            "mean_isaac_lag_rad": (sum(lags) / len(lags)) if lags else None,
            "peak_ros2_err_rad": max(errs) if errs else None,
            "peak_ref_vel_rad_s": max(rvs) if rvs else None,
        }

    # ─── Differential-lag timeseries: at each instant, find max - min across joints ───
    diff_lag_peak = 0.0
    diff_lag_peak_t = 0.0
    diff_lag_peak_per_joint = None
    per_instant_diff = []
    for r in rows:
        t_rel = float(r["t_wall"]) - t0
        per_joint = {}
        for j in ARM:
            v = _f(r.get(f"isaac_lag_{j}"))
            if v is not None:
                per_joint[j] = v
        if len(per_joint) < 2:
            continue
        lags_abs = {j: abs(v) for j, v in per_joint.items()}
        mx = max(lags_abs.values())
        mn = min(lags_abs.values())
        d = mx - mn
        per_instant_diff.append((t_rel, d, per_joint))
        if d > diff_lag_peak:
            diff_lag_peak = d
            diff_lag_peak_t = t_rel
            diff_lag_peak_per_joint = dict(per_joint)

    # ─── Cup proximity + displacement ───
    cup_closest_mm = {"cup_red": 1e9, "cup_green": 1e9, "cup_blue": 1e9}
    cup_closest_t = {}
    cup_start = {}
    cup_max_disp_mm = {}
    cup_max_disp_t = {}
    for r in rows:
        tx, ty, tz = _f(r.get("tcp_x")), _f(r.get("tcp_y")), _f(r.get("tcp_z"))
        t_rel = float(r["t_wall"]) - t0
        for color in ("red", "green", "blue"):
            key = f"cup_{color}"
            cx = _f(r.get(f"{key}_x"))
            cy = _f(r.get(f"{key}_y"))
            cz = _f(r.get(f"{key}_z"))
            if cx is None:
                continue
            if tx is not None:
                dh = math.hypot(tx - cx, ty - cy)
                if dh * 1000.0 < cup_closest_mm[key]:
                    cup_closest_mm[key] = dh * 1000.0
                    cup_closest_t[key] = t_rel
            if key not in cup_start:
                cup_start[key] = (cx, cy, cz)
            sx, sy, sz = cup_start[key]
            disp = math.sqrt((cx - sx) ** 2 + (cy - sy) ** 2 + (cz - sz) ** 2) * 1000.0
            if disp > cup_max_disp_mm.get(key, 0.0):
                cup_max_disp_mm[key] = disp
                cup_max_disp_t[key] = t_rel

    if not verbose:
        return {
            "path": str(path),
            "motion_tag": meta.get("motion_tag", "?"),
            "duration_s": dur,
            "samples": n,
            "stats": stats,
            "diff_lag_peak_rad": diff_lag_peak,
            "diff_lag_peak_t": diff_lag_peak_t,
            "cup_closest_mm": cup_closest_mm,
            "cup_max_disp_mm": cup_max_disp_mm,
        }

    # ─── Verbose report ───
    print("=" * 90)
    print(f"Motion: {path.name}")
    print(f"  tag: {meta.get('motion_tag', '?')}   duration: {dur:.2f}s   samples: {n}")
    if meta.get("traj_dump"):
        print(f"  planner dump: {meta['traj_dump']}")
    print()

    print("Per-joint tracking (Isaac physics vs. plan reference):")
    print(f"  {'joint':<16} {'peak |lag|':>12} {'mean |lag|':>12} {'peak |ros2 err|':>16} {'peak |ref_vel|':>16}")
    for j in ARM:
        s = stats[j]
        pl = s["peak_isaac_lag_rad"]
        ml = s["mean_isaac_lag_rad"]
        pe = s["peak_ros2_err_rad"]
        pv = s["peak_ref_vel_rad_s"]
        print(
            f"  {j:<16} "
            f"{(math.degrees(pl) if pl is not None else 0):>11.3f}° "
            f"{(math.degrees(ml) if ml is not None else 0):>11.3f}° "
            f"{(math.degrees(pe) if pe is not None else 0):>15.3f}° "
            f"{(pv if pv is not None else 0):>15.3f}"
        )
    print()

    print("Differential-lag analysis (max − min |lag| across joints at each sample):")
    print(f"  peak differential = {math.degrees(diff_lag_peak):.3f}° at t_rel={diff_lag_peak_t:.2f}s")
    if diff_lag_peak_per_joint:
        print(f"  per-joint lag at peak moment (rad → deg):")
        for j in ARM:
            v = diff_lag_peak_per_joint.get(j)
            if v is not None:
                print(f"    {j:<16} {v:+8.5f} rad  ({math.degrees(v):+7.3f}°)")
    if diff_lag_peak < 0.010:
        print("  VERDICT: uniform lag (all joints track together within 0.6°).")
        print("  → Isaac's 3D path is plan-path-time-shifted. Any waypoint MoveIt validated")
        print("    is still on the Isaac-visited path, so static-cup collisions via lag ruled out.")
    else:
        print("  VERDICT: differential lag detected.")
        print("  → Isaac visits joint configurations that are NOT on the planned trajectory.")
        print("    Spatial arc diverges from plan. Post-check-safe plan can still yield collision.")
    print()

    print("Cup proximity (TCP horizontal distance to each cup, mm):")
    for key, d in cup_closest_mm.items():
        if d > 1e8:
            continue
        t = cup_closest_t.get(key, 0.0)
        print(f"  {key:<12} closest = {d:>7.1f} mm   at t_rel={t:.2f}s")
    print()

    print("Cup displacement during motion (physical knock detection):")
    any_moved = False
    for key, d in cup_max_disp_mm.items():
        mark = ""
        if d > 1.0:
            mark = "   ⚠ MOVED"
            any_moved = True
        if d > 5.0:
            mark = "   ⚠⚠ KNOCKED"
        print(f"  {key:<12} max disp = {d:>7.2f} mm   at t_rel={cup_max_disp_t.get(key, 0.0):.2f}s{mark}")
    if not any_moved:
        print("  All cups stationary throughout the motion.")
    print()

    # ─── Contact events from omni.physx contact-report subscription ──
    # Each CSV row has:
    #   contact_cup_<c>_hit (0/1), contact_cup_<c>_impulse_n (float),
    #   contact_n_events (int), contact_events_json (list of dicts).
    # We accumulate per-cup events by walking rows: when hit transitions
    # 0→1 we open an event; we keep extending while hit stays 1; when
    # it returns to 0 we close. The bodies set is built from the a0/a1
    # paths in the raw event JSON.
    contact_events = []
    open_events = {}  # cup_color -> current event dict
    for r in rows:
        t_rel = float(r["t_wall"]) - t0
        # Parse the raw event list for this sample, map cup -> non-cup bodies.
        raw_json = r.get("contact_events_json", "") or ""
        raw_events = []
        if raw_json:
            try:
                raw_events = json.loads(raw_json)
            except Exception:
                raw_events = []
        for color in ("red", "green", "blue"):
            hit_flag = r.get(f"contact_cup_{color}_hit", "")
            hit = hit_flag in (1, "1", True)
            impulse_n = _f(r.get(f"contact_cup_{color}_impulse_n")) or 0.0
            if not hit:
                if color in open_events:
                    contact_events.append(open_events.pop(color))
                continue
            # Harvest the non-cup side from raw events involving this cup.
            cup_marker = f"cup_{color}"
            bodies = set()
            for ev in raw_events:
                a0 = ev.get("a0", ""); a1 = ev.get("a1", "")
                if cup_marker in a0 and cup_marker not in a1:
                    bodies.add(a1)
                elif cup_marker in a1 and cup_marker not in a0:
                    bodies.add(a0)
            if color in open_events:
                cur = open_events[color]
                cur["t_end"] = t_rel
                cur["peak_impulse_n"] = max(cur["peak_impulse_n"], impulse_n)
                cur["total_impulse_n"] += impulse_n
                cur["bodies"] |= bodies
            else:
                open_events[color] = {
                    "cup": color,
                    "t_start": t_rel,
                    "t_end": t_rel,
                    "peak_impulse_n": impulse_n,
                    "total_impulse_n": impulse_n,
                    "bodies": set(bodies),
                }
    for ev in open_events.values():
        contact_events.append(ev)

    if contact_events:
        print("Contact events (from omni.physx contact reports — deterministic):")
        for ev in sorted(contact_events, key=lambda e: e["t_start"]):
            dur_ms = (ev["t_end"] - ev["t_start"]) * 1000.0
            bodies = ", ".join(sorted(ev["bodies"])) if ev["bodies"] else "(unknown)"
            print(f"  cup_{ev['cup']:<6} t={ev['t_start']:.3f}s → {ev['t_end']:.3f}s "
                  f"({dur_ms:.0f} ms)  peak_impulse={ev['peak_impulse_n']:.4f} N·s  "
                  f"total={ev['total_impulse_n']:.4f} N·s")
            print(f"    colliding: {bodies}")
        print()
    else:
        if rows and "contact_cup_red_hit" in rows[0]:
            print("Contact events: NONE — no cup touches during this motion. ✓")
            print()

    # ─── Plan-vs-Isaac end-to-end tracking (decomposes the 3-layer lag) ───
    traj_dump_path = meta.get("traj_dump")
    motion_start_wall = meta.get("start_wall")
    if traj_dump_path and motion_start_wall:
        try:
            td = json.loads(Path(traj_dump_path).read_text())
            plan_names = td.get("joint_names", [])
            plan_points = td.get("points", [])
        except Exception:
            plan_points = []; plan_names = []
        if plan_points and plan_names:
            # Per-joint linear interpolation of the plan
            def plan_pos_at(joint, t_plan):
                if joint not in plan_names:
                    return None
                j = plan_names.index(joint)
                if t_plan <= plan_points[0]["t"]:
                    return plan_points[0]["positions"][j]
                if t_plan >= plan_points[-1]["t"]:
                    return plan_points[-1]["positions"][j]
                for k in range(len(plan_points) - 1):
                    a, b = plan_points[k], plan_points[k + 1]
                    if a["t"] <= t_plan <= b["t"]:
                        frac = (t_plan - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0
                        return a["positions"][j] + frac * (b["positions"][j] - a["positions"][j])
                return None

            peak_plan_isaac = {j: 0.0 for j in ARM}
            peak_plan_ref = {j: 0.0 for j in ARM}
            peak_ref_isaac = {j: 0.0 for j in ARM}
            t_peak_plan_isaac = {j: 0.0 for j in ARM}
            for r in rows:
                t_plan = float(r["t_wall"]) - float(motion_start_wall)
                for j in ARM:
                    pp = plan_pos_at(j, t_plan)
                    if pp is None:
                        continue
                    rp = _f(r.get(f"ref_pos_{j}"))
                    ip = _f(r.get(f"isaac_pos_{j}"))
                    if ip is not None:
                        e = abs(pp - ip)
                        if e > peak_plan_isaac[j]:
                            peak_plan_isaac[j] = e
                            t_peak_plan_isaac[j] = t_plan
                    if rp is not None:
                        e2 = abs(pp - rp)
                        if e2 > peak_plan_ref[j]:
                            peak_plan_ref[j] = e2
                    if ip is not None and rp is not None:
                        e3 = abs(rp - ip)
                        if e3 > peak_ref_isaac[j]:
                            peak_ref_isaac[j] = e3

            plan_dur = plan_points[-1]["t"]
            print("Three-layer tracking decomposition (|plan − actual|, per joint):")
            print("  layer A = plan → JTC ref  (planner interpolation + action-server delay)")
            print("  layer B = JTC ref → Isaac (drive PD + command-path latency)")
            print("  A+B    = plan → Isaac     (end-to-end, what the cup actually sees)")
            print()
            print(f"  plan duration: {plan_dur:.2f}s    motion logger duration: {dur:.2f}s")
            print(f"  {'joint':<16} {'A: plan vs ref':>18} {'B: ref vs isaac':>18} {'A+B: plan vs isaac':>22}")
            for j in ARM:
                print(
                    f"  {j:<16} "
                    f"{math.degrees(peak_plan_ref[j]):>17.3f}° "
                    f"{math.degrees(peak_ref_isaac[j]):>17.3f}° "
                    f"{math.degrees(peak_plan_isaac[j]):>21.3f}°   @t={t_peak_plan_isaac[j]:.2f}s"
                )
            # Summary of where the lag lives (averaged across joints)
            tot_A = sum(peak_plan_ref.values())
            tot_B = sum(peak_ref_isaac.values())
            tot_E = sum(peak_plan_isaac.values())
            if tot_E > 1e-6:
                share_A = 100 * tot_A / (tot_A + tot_B)
                share_B = 100 - share_A
                print()
                print(f"  Lag budget (by summed peak across joints):")
                print(f"    Layer A (plan → JTC ref) : {share_A:.0f}%")
                print(f"    Layer B (JTC → Isaac)    : {share_B:.0f}%")
                if share_A > 60:
                    print(f"    → Most lag is in layer A. Raising drive gains / F_max won't help.")
                    print(f"      Look at JTC startup/interpolation or planner time-parameterization.")
                elif share_B > 60:
                    print(f"    → Most lag is in layer B. Drive PD / command-path is the bottleneck.")
                    print(f"      Tuner or action-graph latency are the levers.")
                else:
                    print(f"    → Lag roughly split. Both layers contribute.")
            print()

    # ─── Sim rate diagnostics ───
    rtf = meta.get("realtime_factor")
    fps_mean = meta.get("render_fps_mean")
    fps_min = meta.get("render_fps_min")
    app_dt_mean = meta.get("app_dt_ms_mean")
    app_dt_max = meta.get("app_dt_ms_max")
    phys_hz = None
    for r in rows[:10]:
        v = _f(r.get("physics_rate_hz"))
        if v:
            phys_hz = v; break
    if any(x is not None for x in (rtf, fps_mean, app_dt_mean, phys_hz)):
        print("Sim rate diagnostics:")
        if phys_hz is not None:
            print(f"  physics_rate_hz (target)    : {phys_hz:.1f} Hz")
        if rtf is not None:
            print(f"  realtime factor (sim/wall)  : {rtf:.3f} "
                  f"{'OK (≥0.9)' if rtf >= 0.9 else 'DEGRADED — physics not keeping up'}")
        if fps_mean is not None:
            extra = f"min={fps_min:.1f}" if fps_min is not None else ""
            print(f"  render_fps mean             : {fps_mean:.1f} Hz  {extra}")
        if app_dt_mean is not None:
            extra = f"max={app_dt_max:.1f} ms" if app_dt_max is not None else ""
            print(f"  app update dt mean          : {app_dt_mean:.2f} ms  {extra}  "
                  f"(command-path tick period)")
        print()

    print("Summary:")
    print(f"  CSV : {path}")
    print(f"  meta: {meta_path if meta_path.exists() else '(none)'}")
    return 0


def _scan(root: Path):
    csvs = sorted(root.rglob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not csvs:
        print(f"No motion CSVs under {root}")
        return 0
    print(f"{'file':<60} {'tag':<18} {'dur':>6} {'n':>5} {'peak_lag':>9} {'diff_lag':>9} {'cup_disp':>9}")
    print("-" * 110)
    for p in csvs:
        try:
            s = _analyze(p, verbose=False)
            if isinstance(s, int):
                continue
            stats = s["stats"]
            peak_lag_deg = max(
                (math.degrees(stats[j]["peak_isaac_lag_rad"] or 0) for j in ARM), default=0.0
            )
            disp_mm = max(s["cup_max_disp_mm"].values(), default=0.0)
            print(
                f"{p.name:<60} {s['motion_tag']:<18} {s['duration_s']:>6.2f} {s['samples']:>5} "
                f"{peak_lag_deg:>8.2f}° {math.degrees(s['diff_lag_peak_rad']):>8.2f}° {disp_mm:>8.2f}mm"
            )
        except Exception as e:
            print(f"{p.name:<60} ERROR: {e}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Analyze SO-ARM101 motion telemetry CSVs")
    ap.add_argument("csv", nargs="?", help="Path to motion CSV (optional)")
    ap.add_argument("--latest", action="store_true",
                    help="Analyze most recent CSV under --log-dir")
    ap.add_argument("--scan", nargs="?", const="", default=None,
                    help="List + summarize all CSVs under DIR (or --log-dir)")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/motion_logs"),
                    help="Log root (default ~/motion_logs)")
    ap.add_argument("--json", action="store_true",
                    help="Emit machine-readable meta only")
    args = ap.parse_args()

    root = Path(args.log_dir)

    if args.scan is not None:
        scan_root = Path(args.scan) if args.scan else root
        return _scan(scan_root)

    path = None
    if args.csv:
        path = Path(args.csv)
    elif args.latest:
        path = _latest_csv(root)
        if not path:
            print(f"No CSVs under {root}"); return 1

    if not path:
        ap.print_help(); return 1

    if not path.exists():
        print(f"Not found: {path}", file=sys.stderr); return 1

    if args.json:
        out = _analyze(path, verbose=False)
        print(json.dumps(out, indent=2, default=str))
        return 0

    return _analyze(path)


if __name__ == "__main__":
    sys.exit(main() or 0)
