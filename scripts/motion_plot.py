#!/usr/bin/env python3
# Reference: plot style modeled on NVIDIA Developer Forum threads
#   - rodineye #303545 (IsaacSim+MoveIt2 joint tracking accuracy is not good enough)
#   - JeanLesur  #328322 (Difference between path planned in MoveIt and executed in Isaac Sim)
#   - blueed96   #363351 (Panda Robot-arm trajectory deviation)
# All three posted overlays of JTC reference vs Isaac-Sim actual joint position.
# This script reproduces that plot on our own telemetry and adds two extra views
# the forum doesn't capture (fb_pos from ros2_control, js_pos from /joint_states).
"""
SO-ARM101 Motion Telemetry Plotter.

Inputs: CSVs written by scripts/motion_logger.py (richer than forum data —
four per-joint position views captured: ref_pos, fb_pos, js_pos, isaac_pos).

Outputs per motion, under motion_plots/<csv_stem>/:
  * forum_style.png  — 2-curve overlay (ref_pos vs isaac_pos) per joint, one
                       column per joint. Directly comparable to the plots in
                       forum threads #303545/#328322/#363351.
  * full_stack.png   — 3 panels (position, velocity, error) per joint, with all
                       four position views overlaid. Shows WHICH layer the lag
                       lives in (plan→JTC, JTC→Isaac, Isaac→/joint_states).
  * tcp_xyz.png      — TCP Cartesian position over time (optional, if present).

Usage:
    scripts/motion_plot.py <csv>                # plot one motion
    scripts/motion_plot.py --latest             # most recent CSV
    scripts/motion_plot.py --scan DIR           # plot every CSV under DIR
    scripts/motion_plot.py <csv> --style forum  # only forum-style plot
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Color scheme keyed to the four per-joint position views. Forum posters only
# had the first two — the latter two are our extra instrumentation.
COLORS = {
    "ref_pos":   "#1f77b4",  # blue   — JTC reference (what was commanded)
    "fb_pos":    "#ff7f0e",  # orange — ros2_control feedback
    "js_pos":    "#2ca02c",  # green  — /joint_states published
    "isaac_pos": "#d62728",  # red    — Isaac Sim ground truth (physics)
    "ref_vel":   "#1f77b4",
    "isaac_vel": "#d62728",
    "js_vel":    "#2ca02c",
    "err":       "#8c564b",  # brown  — ref_pos − isaac_pos
}


def _latest_csv(root: Path) -> Path | None:
    csvs = sorted(root.rglob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return csvs[0] if csvs else None


def _load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_rel"] = df["t_wall"] - df["t_wall"].iloc[0]
    return df


def _plot_forum_style(df: pd.DataFrame, title: str, out: Path) -> None:
    """Reproduces the forum-canonical plot: ref_pos vs isaac_pos per joint.

    This is the plot shape used by rodineye/JeanLesur/blueed96 to report
    tracking lag. Keep it minimal on purpose — apples-to-apples comparison
    with their screenshots.
    """
    fig, axes = plt.subplots(len(ARM), 1, figsize=(11, 2.1 * len(ARM)), sharex=True)
    for ax, j in zip(axes, ARM):
        ref = df[f"ref_pos_{j}"]
        isaac = df[f"isaac_pos_{j}"]
        ax.plot(df["t_rel"], ref, color=COLORS["ref_pos"],
                label="ref_pos (JTC command)", linewidth=1.8)
        ax.plot(df["t_rel"], isaac, color=COLORS["isaac_pos"],
                label="isaac_pos (actual physics)", linewidth=1.2, alpha=0.9)
        ax.set_ylabel(f"{j}\n(rad)", fontsize=9)
        ax.grid(True, alpha=0.3)
        peak_lag = (ref - isaac).abs().max()
        ax.set_title(f"peak |lag| = {math.degrees(peak_lag):.3f}°",
                     loc="right", fontsize=8, color="gray")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time since motion start (s)")
    fig.suptitle(f"Forum-style: ref vs actual — {title}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot_full_stack(df: pd.DataFrame, title: str, out: Path) -> None:
    """Three panels per joint: position (4 curves), velocity (3), error (1).

    Distinguishes WHICH layer the lag lives in:
      ref_pos   → JTC command (what we asked)
      fb_pos    → ros2_control feedback (what JTC thinks)
      js_pos    → /joint_states published (what ROS sees)
      isaac_pos → dynamic_control ground truth (what physics did)
    If fb_pos and js_pos diverge from isaac_pos, the publishing bridge has lag.
    If ref_pos and isaac_pos diverge but js_pos tracks isaac_pos, the drive is
    the bottleneck. Forum posts can't distinguish these — we can.
    """
    n = len(ARM)
    fig, axes = plt.subplots(n, 3, figsize=(18, 3.2 * n), sharex=True)
    for i, j in enumerate(ARM):
        ax_p, ax_v, ax_e = axes[i, 0], axes[i, 1], axes[i, 2]

        ax_p.plot(df["t_rel"], df[f"ref_pos_{j}"], color=COLORS["ref_pos"], label="ref (JTC)", linewidth=1.8)
        ax_p.plot(df["t_rel"], df[f"fb_pos_{j}"], color=COLORS["fb_pos"], label="fb (ros2_ctrl)", linewidth=0.9, linestyle=":")
        ax_p.plot(df["t_rel"], df[f"js_pos_{j}"], color=COLORS["js_pos"], label="/joint_states", linewidth=0.9, linestyle="--")
        ax_p.plot(df["t_rel"], df[f"isaac_pos_{j}"], color=COLORS["isaac_pos"], label="isaac (physics)", linewidth=1.3, alpha=0.9)
        ax_p.set_ylabel(f"{j}\npos (rad)", fontsize=8)
        ax_p.grid(True, alpha=0.3)
        if i == 0:
            ax_p.legend(loc="upper right", fontsize=7)
            ax_p.set_title("position", fontsize=10)

        ax_v.plot(df["t_rel"], df[f"ref_vel_{j}"], color=COLORS["ref_vel"], label="ref_vel", linewidth=1.3)
        if f"isaac_vel_{j}" in df.columns:
            ax_v.plot(df["t_rel"], df[f"isaac_vel_{j}"], color=COLORS["isaac_vel"], label="isaac_vel", linewidth=1.0, alpha=0.85)
        if f"js_vel_{j}" in df.columns:
            ax_v.plot(df["t_rel"], df[f"js_vel_{j}"], color=COLORS["js_vel"], label="js_vel", linewidth=0.9, linestyle="--")
        ax_v.set_ylabel("vel (rad/s)", fontsize=8)
        ax_v.grid(True, alpha=0.3)
        if i == 0:
            ax_v.legend(loc="upper right", fontsize=7)
            ax_v.set_title("velocity", fontsize=10)

        err_ref_isaac = df[f"ref_pos_{j}"] - df[f"isaac_pos_{j}"]
        err_ros = pd.to_numeric(df[f"err_pos_{j}"], errors="coerce")
        ax_e.axhline(0, color="black", linewidth=0.5, alpha=0.4)
        ax_e.plot(df["t_rel"], err_ref_isaac.apply(math.degrees),
                  color=COLORS["err"], linewidth=1.2, label="ref − isaac (deg)")
        ax_e.plot(df["t_rel"], err_ros.apply(lambda v: math.degrees(v) if pd.notna(v) else float("nan")),
                  color="black", linewidth=0.7, linestyle=":", label="ros2_ctrl err (deg)")
        ax_e.set_ylabel("err (deg)", fontsize=8)
        ax_e.grid(True, alpha=0.3)
        if i == 0:
            ax_e.legend(loc="upper right", fontsize=7)
            ax_e.set_title("tracking error", fontsize=10)

    for ax in axes[-1]:
        ax.set_xlabel("time since motion start (s)")
    fig.suptitle(f"Full telemetry — {title}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(out, dpi=110)
    plt.close(fig)


def _plot_joint_zoom(df: pd.DataFrame, joint: str, title: str, out: Path) -> None:
    """Single-joint detail, matching blueed96's (#363351) zoomed per-joint view.

    Three stacked panels: position (all 4 views), velocity (3 views), error.
    Bigger so lag transitions at waypoint boundaries are readable.
    """
    fig, (ax_p, ax_v, ax_e) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    ax_p.plot(df["t_rel"], df[f"ref_pos_{joint}"], color=COLORS["ref_pos"], label="ref (JTC)", linewidth=2.0)
    ax_p.plot(df["t_rel"], df[f"fb_pos_{joint}"], color=COLORS["fb_pos"], label="fb (ros2_ctrl)", linewidth=1.0, linestyle=":")
    ax_p.plot(df["t_rel"], df[f"js_pos_{joint}"], color=COLORS["js_pos"], label="/joint_states", linewidth=1.0, linestyle="--")
    ax_p.plot(df["t_rel"], df[f"isaac_pos_{joint}"], color=COLORS["isaac_pos"], label="isaac (physics)", linewidth=1.5, alpha=0.9)
    ax_p.set_ylabel("position (rad)")
    ax_p.grid(True, alpha=0.3)
    ax_p.legend(loc="upper right", fontsize=9)
    ax_p.set_title(f"{joint} — position", fontsize=11)

    ax_v.plot(df["t_rel"], df[f"ref_vel_{joint}"], color=COLORS["ref_vel"], label="ref_vel", linewidth=1.6)
    if f"isaac_vel_{joint}" in df.columns:
        ax_v.plot(df["t_rel"], df[f"isaac_vel_{joint}"], color=COLORS["isaac_vel"], label="isaac_vel", linewidth=1.2, alpha=0.9)
    if f"js_vel_{joint}" in df.columns:
        ax_v.plot(df["t_rel"], df[f"js_vel_{joint}"], color=COLORS["js_vel"], label="js_vel", linewidth=1.0, linestyle="--")
    ax_v.set_ylabel("velocity (rad/s)")
    ax_v.grid(True, alpha=0.3)
    ax_v.legend(loc="upper right", fontsize=9)
    ax_v.set_title("velocity", fontsize=11)

    err_ref_isaac = df[f"ref_pos_{joint}"] - df[f"isaac_pos_{joint}"]
    err_ros = pd.to_numeric(df[f"err_pos_{joint}"], errors="coerce")
    ax_e.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax_e.plot(df["t_rel"], err_ref_isaac.apply(math.degrees),
              color=COLORS["err"], linewidth=1.5, label="ref − isaac (deg)")
    ax_e.plot(df["t_rel"], err_ros.apply(lambda v: math.degrees(v) if pd.notna(v) else float("nan")),
              color="black", linewidth=0.9, linestyle=":", label="ros2_ctrl err (deg)")
    ax_e.set_ylabel("error (deg)")
    ax_e.set_xlabel("time since motion start (s)")
    ax_e.grid(True, alpha=0.3)
    ax_e.legend(loc="upper right", fontsize=9)
    peak = err_ref_isaac.abs().max()
    ax_e.set_title(f"tracking error — peak |ref − isaac| = {math.degrees(peak):.3f}°",
                   fontsize=11)

    fig.suptitle(f"{joint} detail — {title}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot_tcp(df: pd.DataFrame, title: str, out: Path) -> bool:
    if "tcp_x" not in df.columns or df["tcp_x"].isna().all():
        return False
    tcp_x = pd.to_numeric(df["tcp_x"], errors="coerce")
    tcp_y = pd.to_numeric(df["tcp_y"], errors="coerce")
    tcp_z = pd.to_numeric(df["tcp_z"], errors="coerce")
    if tcp_x.isna().all():
        return False
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(df["t_rel"], tcp_x * 1000, label="TCP x (mm)", color="#1f77b4")
    ax.plot(df["t_rel"], tcp_y * 1000, label="TCP y (mm)", color="#2ca02c")
    ax.plot(df["t_rel"], tcp_z * 1000, label="TCP z (mm)", color="#d62728")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position (mm)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title(f"TCP Cartesian position — {title}")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return True


def _plot_one(csv_path: Path, style: str, out_root: Path) -> Path:
    df = _load(csv_path)
    stem = csv_path.stem
    out_dir = out_root / stem
    out_dir.mkdir(parents=True, exist_ok=True)
    title = stem
    if style in ("forum", "both"):
        _plot_forum_style(df, title, out_dir / "forum_style.png")
    if style in ("full", "both"):
        _plot_full_stack(df, title, out_dir / "full_stack.png")
        _plot_tcp(df, title, out_dir / "tcp_xyz.png")
        worst_joint = max(
            ARM,
            key=lambda j: (df[f"ref_pos_{j}"] - df[f"isaac_pos_{j}"]).abs().max(),
        )
        _plot_joint_zoom(df, worst_joint, title,
                         out_dir / f"joint_zoom_{worst_joint}.png")
    return out_dir


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot SO-ARM101 motion telemetry")
    ap.add_argument("csv", nargs="?", help="Path to motion CSV")
    ap.add_argument("--latest", action="store_true", help="Most recent CSV under --log-dir")
    ap.add_argument("--scan", nargs="?", const="", default=None,
                    help="Plot every CSV under DIR (or --log-dir)")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/motion_logs"))
    ap.add_argument("--out", default=os.path.expanduser("~/motion_logs/plots"),
                    help="Output root directory")
    ap.add_argument("--style", choices=["forum", "full", "both"], default="both",
                    help="forum-canonical 2-curve, our 4-curve full stack, or both")
    args = ap.parse_args()

    out_root = Path(args.out)
    root = Path(args.log_dir)

    targets: list[Path] = []
    if args.scan is not None:
        scan_root = Path(args.scan) if args.scan else root
        targets = sorted(scan_root.rglob("*.csv"), key=lambda p: p.stat().st_mtime)
    elif args.csv:
        targets = [Path(args.csv)]
    elif args.latest:
        p = _latest_csv(root)
        if not p:
            print(f"No CSVs under {root}", file=sys.stderr)
            return 1
        targets = [p]
    else:
        ap.print_help()
        return 1

    for p in targets:
        if not p.exists():
            print(f"skip (missing): {p}", file=sys.stderr)
            continue
        try:
            out_dir = _plot_one(p, args.style, out_root)
            print(f"{p.name} -> {out_dir}")
        except Exception as e:
            print(f"{p.name}: ERROR {e}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
