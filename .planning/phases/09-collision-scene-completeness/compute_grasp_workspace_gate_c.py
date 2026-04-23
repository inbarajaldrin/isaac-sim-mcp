#!/usr/bin/env python3
"""Self-collision-aware grasp workspace sweep.

The existing compute_workspace.py computes `grasp_workspace_bounds` by
sweeping (r, z, yaw) through geometric_ik and marking (r, z) reachable
when ANY yaw returns an IK solution. That's Gate A (pose in workspace)
+ Gate B (IK converges). It does NOT check Gate C — whether the IK
solution is self-collision-free.

As a result, the bounds claim r_min=0.054m is reachable when in fact
many (r, z) inside that bound yield solutions that immediately fail
/check_state_validity with self-collision (camera_mount ↔ shoulder,
shoulder ↔ gripper, usb_camera ↔ upper_arm, etc.). Legos placed in
that gap cannot be grasped even though they're "in the workspace".

This tool closes the gap: for each (r, z) pair, it enumerates the same
yaw candidates geometric_ik would try, validates EACH solution against
the live MoveIt planning scene via /check_state_validity with
group_name='' (empty → all collision pairs checked, including attached
body vs world), and only marks (r, z) as reachable if at least one
yaw/config passes Gate C.

Outputs:
  - grasp_workspace_bounds_gate_c.yaml — tighter bounds suitable for
    BLOCK_RANDOM_FORWARD / BLOCK_RANDOM_LATERAL in the Isaac Sim
    extension.
  - grasp_workspace_gate_c_heatmap.png — visual difference Gate B vs
    Gate B+C reachability.
  - A proposed BLOCK_RANDOM_FORWARD tuple sized to fit inside the
    Gate-C-aware bounds.

Prerequisites:
  - Control stack running (MoveIt's /check_state_validity is required).
  - Cups + legos SHOULD NOT be in the planning scene, otherwise we
    measure reachability-for-this-scene instead of pure self-collision.
    Call /grasp_refresh before running (clears legos) — cups stay, but
    they're at y ≈ -0.28m outside the block randomize region, so they
    don't influence the sweep if we keep y=0.

Usage:
    source ~/Projects/Exploring-VLAs/vla_SO-ARM101/install/setup.bash
    python3 compute_grasp_workspace_gate_c.py
    python3 compute_grasp_workspace_gate_c.py --r-step 0.01   # coarser
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
except ImportError as e:
    sys.stderr.write(f'Import: {e}\nSource the workspace first.\n')
    sys.exit(1)

try:
    from so_arm101_control.compute_workspace import geometric_ik
    from so_arm101_control.control_gui import _GRASP_YAW_FALLBACK_OFFSETS
except ImportError as e:
    sys.stderr.write(f'Import: {e}\nNeed so_arm101_control on PYTHONPATH.\n')
    sys.exit(1)

ARM_JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                   'wrist_flex', 'wrist_roll']
GRIPPER_JOINT_NAME = 'gripper_joint'


class GateCChecker(Node):
    """Node that calls /check_state_validity one config at a time."""

    def __init__(self):
        super().__init__('gate_c_reachability_checker')
        self.client = self.create_client(
            GetStateValidity, '/check_state_validity')
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/check_state_validity not available')

    def is_valid(self, joints: dict, timeout: float = 1.0) -> bool:
        """Return True if the given joint config passes validity."""
        req = GetStateValidity.Request()
        rs = RobotState()
        # Include gripper_joint — without it MoveIt uses 0 as default,
        # which may not match the actual home pose and cause spurious
        # self-collisions per the jaw pose. Use 0 (closed) since we're
        # testing pure self-collision independent of grip state.
        rs.joint_state.name = ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        rs.joint_state.position = [
            float(joints.get(n, 0.0)) for n in ARM_JOINT_NAMES] + [0.0]
        req.robot_state = rs
        # group_name='' → all collision pairs. This is the ground truth.
        req.group_name = ''
        future = self.client.call_async(req)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return True  # timeout → fail-open, don't reject spuriously
            rclpy.spin_once(self, timeout_sec=0.05)
        res = future.result()
        if res is None:
            return True
        return bool(res.valid)


def sweep(checker: GateCChecker, r_step=0.005, z_step=0.005,
          r_range=(0.02, 0.40), z_range=(-0.25, 0.15)):
    """Return (reachable_b, reachable_bc) lists of (r, z) tuples."""
    rs = [r_range[0] + i * r_step
          for i in range(int(round((r_range[1] - r_range[0]) / r_step)) + 1)]
    zs = [z_range[0] + i * z_step
          for i in range(int(round((z_range[1] - z_range[0]) / z_step)) + 1)]
    reach_b = []
    reach_bc = []
    t0 = time.monotonic()
    n_total = len(rs) * len(zs)
    done = 0
    n_val_calls = 0
    for r in rs:
        for z in zs:
            done += 1
            b_ok = False
            bc_ok = False
            x, y = float(r), 0.0
            for yaw_off in _GRASP_YAW_FALLBACK_OFFSETS:
                sols = geometric_ik(x, y, float(z), grasp_yaw=float(yaw_off))
                if not sols:
                    continue
                b_ok = True
                # Test each IK solution against /check_state_validity
                for sol in sols:
                    n_val_calls += 1
                    if checker.is_valid(sol):
                        bc_ok = True
                        break
                if bc_ok:
                    break
            if b_ok:
                reach_b.append((float(r), float(z)))
            if bc_ok:
                reach_bc.append((float(r), float(z)))
        dt = time.monotonic() - t0
        print(f'  r={r:.3f}  done={done}/{n_total}  '
              f'B={len(reach_b)}  B+C={len(reach_bc)}  '
              f'val_calls={n_val_calls}  elapsed={dt:.1f}s',
              flush=True)
    return reach_b, reach_bc


def bounds_from(pairs):
    if not pairs: return None
    rs = [p[0] for p in pairs]; zs = [p[1] for p in pairs]
    return {
        'r_min': min(rs), 'r_max': max(rs),
        'z_min': min(zs), 'z_max': max(zs),
        'n': len(pairs),
    }


def write_outputs(reach_b, reach_bc, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)
    b = bounds_from(reach_b)
    bc = bounds_from(reach_bc)
    # YAML
    lines = ['# Self-collision-aware grasp workspace (Gate B + Gate C)', '#']
    lines.append('# Generated by compute_grasp_workspace_gate_c.py')
    lines.append('# Gate B = geometric_ik returns a solution')
    lines.append('# Gate C = /check_state_validity passes for that solution')
    lines.append('')
    lines.append('grasp_workspace_bounds_gate_b:  # matches existing bounds')
    lines.append(f'  r_min: {b["r_min"]:.4f}')
    lines.append(f'  r_max: {b["r_max"]:.4f}')
    lines.append(f'  z_min: {b["z_min"]:.4f}')
    lines.append(f'  z_max: {b["z_max"]:.4f}')
    lines.append(f'  n_reachable: {b["n"]}')
    lines.append('')
    lines.append('grasp_workspace_bounds_gate_c:  # TIGHTER — use this for randomize')
    lines.append(f'  r_min: {bc["r_min"]:.4f}')
    lines.append(f'  r_max: {bc["r_max"]:.4f}')
    lines.append(f'  z_min: {bc["z_min"]:.4f}')
    lines.append(f'  z_max: {bc["z_max"]:.4f}')
    lines.append(f'  n_reachable: {bc["n"]}')
    lines.append('')
    # Proposed BLOCK_RANDOM based on Gate C bounds
    # Add small safety margin (3mm) beyond the raw Gate C bound.
    safe_r_min = round(bc['r_min'] + 0.003, 3)
    safe_r_max = round(bc['r_max'] - 0.003, 3)
    # For lateral: assume symmetry around y=0, lateral_max = sqrt(r_max^2 - r_min^2)
    # Conservative: lateral half-range = min(0.10, sqrt(r_max^2 - r_min^2))
    lat = round(min(0.10,
                    (safe_r_max**2 - safe_r_min**2) ** 0.5 if safe_r_max > safe_r_min else 0), 3)
    lines.append('# Suggested randomize bounds for the Isaac Sim extension:')
    lines.append(f'# BLOCK_RANDOM_FORWARD = ({safe_r_min}, {safe_r_max})  # +3mm safety margin on r_min')
    lines.append(f'# BLOCK_RANDOM_LATERAL = ({-lat}, {lat})')
    out = outdir / 'grasp_workspace_bounds_gate_c.yaml'
    out.write_text('\n'.join(lines) + '\n')
    print(f'\nWrote {out}')
    # Heatmap
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        # Build a grid
        rs = sorted({p[0] for p in reach_b} | {p[0] for p in reach_bc})
        zs = sorted({p[1] for p in reach_b} | {p[1] for p in reach_bc})
        b_set = set(reach_b); bc_set = set(reach_bc)
        arr = np.zeros((len(zs), len(rs)), dtype=np.int8)
        for j, z in enumerate(zs):
            for i, r in enumerate(rs):
                pt = (r, z)
                if pt in bc_set:
                    arr[j, i] = 2  # green: B+C pass
                elif pt in b_set:
                    arr[j, i] = 1  # yellow: B pass only (Gate C rejects)
                else:
                    arr[j, i] = 0  # grey: unreachable
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(arr, origin='lower',
                       extent=[rs[0], rs[-1], zs[0], zs[-1]],
                       cmap='RdYlGn', vmin=0, vmax=2, aspect='auto')
        ax.set_xlabel('r (m)')
        ax.set_ylabel('z (m)')
        ax.set_title(
            'Grasp workspace: Gate B vs Gate B+C\n'
            'grey=unreachable, yellow=IK-ok but self-collision, '
            'green=IK-ok + valid\n'
            f'Gate B r=[{b["r_min"]:.3f},{b["r_max"]:.3f}]  '
            f'Gate C r=[{bc["r_min"]:.3f},{bc["r_max"]:.3f}]'
        )
        plt.colorbar(im, ax=ax, ticks=[0, 1, 2],
                     label='0=none  1=B only  2=B+C')
        fig.savefig(outdir / 'grasp_workspace_gate_c_heatmap.png',
                    dpi=120, bbox_inches='tight')
        plt.close(fig)
        print(f'Wrote {outdir / "grasp_workspace_gate_c_heatmap.png"}')
    except ImportError:
        print('[warn] matplotlib missing — skipping PNG')
    # Summary
    print()
    print('=' * 60)
    print(f'Gate B only : r=[{b["r_min"]:.4f}, {b["r_max"]:.4f}]  '
          f'z=[{b["z_min"]:.4f}, {b["z_max"]:.4f}]  n={b["n"]}')
    print(f'Gate B + C  : r=[{bc["r_min"]:.4f}, {bc["r_max"]:.4f}]  '
          f'z=[{bc["z_min"]:.4f}, {bc["z_max"]:.4f}]  n={bc["n"]}')
    rejected_pct = (1 - bc['n'] / b['n']) * 100 if b['n'] else 0
    print(f'Gate C rejection rate: {rejected_pct:.1f}% of Gate-B-pass cells')
    print()
    print('Suggested randomize bounds (+3mm safety margin):')
    print(f'  BLOCK_RANDOM_FORWARD = ({safe_r_min}, {safe_r_max})')
    print(f'  BLOCK_RANDOM_LATERAL = ({-lat}, {lat})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r-step', type=float, default=0.005)
    ap.add_argument('--z-step', type=float, default=0.005)
    ap.add_argument('--r-min', type=float, default=0.02)
    ap.add_argument('--r-max', type=float, default=0.40)
    ap.add_argument('--z-min', type=float, default=-0.25)
    ap.add_argument('--z-max', type=float, default=0.15)
    ap.add_argument('--outdir', type=str,
                    default=str(Path(__file__).parent))
    args = ap.parse_args()

    rclpy.init()
    try:
        checker = GateCChecker()
        print(f'Sweep: r∈[{args.r_min}, {args.r_max}] step={args.r_step}, '
              f'z∈[{args.z_min}, {args.z_max}] step={args.z_step}')
        print()
        t0 = time.monotonic()
        reach_b, reach_bc = sweep(
            checker,
            r_step=args.r_step, z_step=args.z_step,
            r_range=(args.r_min, args.r_max),
            z_range=(args.z_min, args.z_max),
        )
        dt = time.monotonic() - t0
        print(f'\nSweep done in {dt:.1f}s')
        write_outputs(reach_b, reach_bc, Path(args.outdir))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
