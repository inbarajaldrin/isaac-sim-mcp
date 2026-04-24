#!/usr/bin/env python3
# Reference: hand-rolled for vla_SO-ARM101 + soarm101-dt digital-twin workflow.
"""
SO-ARM101 Motion Telemetry Logger — permanent data-capture daemon.

Records, at ~50 Hz during every commanded motion:
  * ros2_control view: /arm_controller/controller_state.{reference,feedback,error}
  * Mock-hardware echo: /joint_states (positions + velocities)
  * Isaac Sim ground truth: arm DOF positions+velocities via MCP dynamic_control
  * Cup + attached-lego poses via MCP dynamic_control
  * Gripper-link world transform via Isaac Sim USD (for TCP FK)
  * Sim time (/clock) and wall time

A "motion" is bounded by ref_vel crossing 0.01 rad/s on any joint. Each motion
yields one CSV (time-series) + one JSON (metadata + summary) in
~/motion_logs/YYYY-MM-DD/ .

Runs as a single standalone process. Safe to leave running indefinitely —
overhead is ~one MCP socket round-trip every 20 ms (<<1% RTF).

Usage:
    python3 scripts/motion_logger.py                  # foreground
    python3 scripts/motion_logger.py --log-dir PATH   # custom output root
    python3 scripts/motion_logger.py --rate 100       # Hz (default 50)
    python3 scripts/motion_logger.py --motion-tag-from-dumps   # read /tmp/arm_traj/
"""

import argparse
import csv
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


# ─── MCP helper ──────────────────────────────────────────────────────────────

_MCP_CODE_SNAPSHOT = '''
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Usd, UsdGeom, Gf, UsdPhysics
import omni.usd
import time as _t

dc = _dynamic_control.acquire_dynamic_control_interface()
stage = omni.usd.get_context().get_stage()

# --- Sim rate probes ---
# Target physics rate from /physicsScene's timeStepsPerSecond attribute
physics_rate_hz = None
try:
    for _p in stage.Traverse():
        if _p.IsA(UsdPhysics.Scene):
            _a = _p.GetAttribute("physxScene:timeStepsPerSecond")
            if not _a or not _a.HasValue():
                _a = _p.GetAttribute("timeStepsPerSecond")
            if _a and _a.HasValue():
                physics_rate_hz = float(_a.Get())
            break
except Exception:
    pass

# Timeline simulation time (authoritative sim clock)
sim_time_s = None
try:
    import omni.timeline
    _tl = omni.timeline.get_timeline_interface()
    sim_time_s = float(_tl.get_current_time())
except Exception:
    pass

# Render FPS — try a couple of known sources, tolerate absence
render_fps = None
try:
    import carb.stats
    # Common Kit stats channel
    for k in ("/ext/fps", "/app/fps", "Rendering/fps"):
        try:
            v = carb.stats.get_stat_value(k)
            if v is not None:
                render_fps = float(v)
                break
        except Exception:
            pass
except Exception:
    pass

# App update dt (Kit main loop; proxy for render-tick period)
app_dt_ms = None
try:
    import omni.kit.app
    app = omni.kit.app.get_app()
    if hasattr(app, "get_last_update_dt"):
        app_dt_ms = float(app.get_last_update_dt()) * 1000.0
except Exception:
    pass

# --- Arm articulation ---
art = dc.get_articulation("/World/SO_ARM101/base_link")
arm_names = []
arm_pos = []
arm_vel = []
for i in range(dc.get_articulation_dof_count(art)):
    d = dc.get_articulation_dof(art, i)
    arm_names.append(dc.get_dof_name(d))
    arm_pos.append(dc.get_dof_position(d))
    arm_vel.append(dc.get_dof_velocity(d))

# --- Gripper-link world pose (source for TCP FK) ---
def _mat_to_pq(M):
    """Return translation + quaternion xyzw of a Gf.Matrix4d (already orthonormal for pose)."""
    t = M.ExtractTranslation()
    q = M.ExtractRotationQuat()
    im = q.GetImaginary()
    return [t[0], t[1], t[2], im[0], im[1], im[2], q.GetReal()]

grip = stage.GetPrimAtPath("/World/SO_ARM101/gripper_link")
grip_xform = UsdGeom.Xformable(grip) if grip and grip.IsValid() else None
grip_pq = _mat_to_pq(grip_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())) if grip_xform else None

# --- Cups, legos, attached bodies ---
def _body(path):
    h = dc.get_rigid_body(path)
    if h == 0:
        return None
    p = dc.get_rigid_body_pose(h)
    return [p.p[0], p.p[1], p.p[2], p.r[0], p.r[1], p.r[2], p.r[3]]

cups = {c: _body(f"/World/Containers/{c}") for c in ("cup_red", "cup_green", "cup_blue")}
legos = {}
for color in ("red", "green", "blue"):
    for size in ("2x2", "2x3", "2x4"):
        name = f"{color}_{size}"
        legos[name] = _body(f"/World/Objects/{name}/{name}")

# --- Contact events (populated by DigitalTwin._setup_contact_sensors,
# which installs an omni.physx contact-report subscription on cups +
# robot gripper/jaw/wrist links). The PhysX callback appends to a
# deque; we drain it here once per snapshot (50 Hz).
#
# Each event: {t, type, a0, a1, c0, c1, impulse [x,y,z], impulse_mag,
# position, n_contacts}. CONTACT_FOUND = first-contact transition (the
# deterministic "gripper hit cup" signal we want for lag attribution).
contacts = []
try:
    from so_arm101_dt.extension import _CONTACT_EVENTS
    n_drain = min(len(_CONTACT_EVENTS), 64)
    for _ in range(n_drain):
        contacts.append(_CONTACT_EVENTS.popleft())
except Exception:
    pass  # subscription not installed — older quick_start, fine to skip

result = {
    "t_wall": _t.time(),
    "arm_names": arm_names,
    "arm_pos": arm_pos,
    "arm_vel": arm_vel,
    "gripper_world_pq": grip_pq,
    "cups": cups,
    "legos": legos,
    "contacts": contacts,
    "physics_rate_hz": physics_rate_hz,
    "sim_time_s": sim_time_s,
    "render_fps": render_fps,
    "app_dt_ms": app_dt_ms,
}
'''.strip()


class MCPClient:
    """Persistent socket to the Isaac-Sim MCP extension on :8767."""

    def __init__(self, host="localhost", port=8767):
        self.host, self.port = host, port
        self.sock = None
        self._connect()

    def _connect(self):
        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
        except Exception as e:
            self.sock = None

    def snapshot(self):
        """Return dict from execute_python_code(snapshot) or None on failure."""
        if self.sock is None:
            self._connect()
            if self.sock is None:
                return None
        payload = {
            "type": "execute_python_code",
            "params": {"code": _MCP_CODE_SNAPSHOT},
        }
        try:
            self.sock.sendall(json.dumps(payload).encode() + b"\n")
            buf = b""
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                try:
                    json.loads(buf.decode())
                    break
                except Exception:
                    pass
            d = json.loads(buf.decode())
            if d.get("status") != "success":
                return None
            return d.get("result", {}).get("result")
        except Exception:
            self.sock = None
            return None


# ─── TCP FK (from gripper_world_pq) ──────────────────────────────────────────

# URDF: tcp_joint is fixed offset from gripper_link at xyz=(-0.0079, -0.000218121, -0.0981274), rpy=(0, pi, 0)
# Applied as a pure rigid transform to the gripper world pose → TCP world pose.
_TCP_LOCAL = (-0.0079, -0.000218121, -0.0981274)


def _quat_xyzw_to_R(qx, qy, qz, qw):
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return (
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)),
    )


def tcp_from_gripper(grip_pq):
    """Given gripper_link pose (xyz + quat xyzw), return TCP world xyz."""
    if grip_pq is None or len(grip_pq) != 7:
        return None
    gx, gy, gz, qx, qy, qz, qw = grip_pq
    R = _quat_xyzw_to_R(qx, qy, qz, qw)
    lx, ly, lz = _TCP_LOCAL
    wx = gx + R[0][0] * lx + R[0][1] * ly + R[0][2] * lz
    wy = gy + R[1][0] * lx + R[1][1] * ly + R[1][2] * lz
    wz = gz + R[2][0] * lx + R[2][1] * ly + R[2][2] * lz
    return (wx, wy, wz)


# ─── Logger ─────────────────────────────────────────────────────────────────

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT = "gripper_joint"


class MotionLogger(Node):
    REF_VEL_START_THRESH = 0.010   # rad/s — motion begins when any |ref_vel| > this
    REF_VEL_STOP_THRESH = 0.005    # rad/s — motion ends when all |ref_vel| < this for STOP_DWELL_S
    STOP_DWELL_S = 0.5
    IDLE_FLUSH_S = 60.0            # flush any open motion if stale

    def __init__(self, log_dir: Path, rate_hz: float, motion_tag_from_dumps: bool):
        super().__init__("motion_logger")
        self.log_dir = log_dir
        self.rate_hz = rate_hz
        self.motion_tag_from_dumps = motion_tag_from_dumps

        self.mcp = MCPClient()
        self.latest_ctrl = None   # (t_wall, ref_pos, ref_vel, fb_pos, err_pos)
        self.latest_js = None     # (t_wall, names, pos, vel)
        self.latest_clock_ns = None  # int

        self._lock = threading.Lock()

        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._on_ctrl,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)

        # Motion state machine
        self.in_motion = False
        self.motion_start_wall = None
        self.motion_stop_pending_since = None
        self.current_csv = None
        self.current_writer = None
        self.current_csv_path = None
        self.current_motion_meta = {}
        self.current_samples = 0

        # Run-time stats carried to summary
        self.peak_ref_vel = [0.0] * len(ARM_JOINTS)
        self.peak_abs_err = [0.0] * len(ARM_JOINTS)
        self.peak_isaac_lag = [0.0] * len(ARM_JOINTS)
        self.per_cup_closest = {"cup_red": 1e9, "cup_green": 1e9, "cup_blue": 1e9}
        self.per_cup_start_pos = {}
        self.per_cup_max_disp = {}
        # Sim-rate tracking (for command-path latency diagnosis)
        self.sim_time_start = None
        self.sim_time_last = None
        self.wall_time_start_for_rtf = None
        self.fps_samples = []
        self.app_dt_samples = []

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"motion_logger writing to {self.log_dir} at {rate_hz:.1f} Hz"
        )

    # ─── ROS callbacks ───────────────────────────────────────────────────

    def _on_ctrl(self, m: JointTrajectoryControllerState):
        ref_pos = list(m.reference.positions) if m.reference.positions else []
        ref_vel = list(m.reference.velocities) if m.reference.velocities else []
        fb_pos = list(m.feedback.positions) if m.feedback.positions else []
        err_pos = list(m.error.positions) if m.error.positions else []
        with self._lock:
            self.latest_ctrl = (time.time(), list(m.joint_names), ref_pos, ref_vel, fb_pos, err_pos)

    def _on_js(self, m: JointState):
        with self._lock:
            self.latest_js = (time.time(), list(m.name), list(m.position),
                              list(m.velocity) if m.velocity else [])

    def _on_clock(self, m: Clock):
        self.latest_clock_ns = m.clock.sec * 1_000_000_000 + m.clock.nanosec

    # ─── Sampling loop (called from main thread) ─────────────────────────

    def sample_once(self):
        """One sample. Handles motion detection + CSV open/close."""
        with self._lock:
            ctrl = self.latest_ctrl
            js = self.latest_js
            clock_ns = self.latest_clock_ns

        if ctrl is None:
            return

        t_wall = time.time()
        _, ctrl_names, ref_pos, ref_vel, fb_pos, err_pos = ctrl

        # map ref_vel to arm-joint order (ctrl_names may or may not match ARM_JOINTS order)
        def pick(vals, names_src, want):
            out = []
            for n in want:
                if n in names_src:
                    i = names_src.index(n)
                    out.append(vals[i] if i < len(vals) else 0.0)
                else:
                    out.append(0.0)
            return out

        ref_pos_ordered = pick(ref_pos, ctrl_names, ARM_JOINTS)
        ref_vel_ordered = pick(ref_vel, ctrl_names, ARM_JOINTS)
        fb_pos_ordered = pick(fb_pos, ctrl_names, ARM_JOINTS)
        err_pos_ordered = pick(err_pos, ctrl_names, ARM_JOINTS)

        peak_rv = max((abs(v) for v in ref_vel_ordered), default=0.0)

        # Motion state transitions
        if not self.in_motion:
            if peak_rv > self.REF_VEL_START_THRESH:
                self._start_motion(t_wall, ref_pos_ordered)
        else:
            if peak_rv < self.REF_VEL_STOP_THRESH:
                if self.motion_stop_pending_since is None:
                    self.motion_stop_pending_since = t_wall
                elif t_wall - self.motion_stop_pending_since > self.STOP_DWELL_S:
                    self._end_motion(t_wall)
                    return  # file closed; don't try to write this sample
            else:
                self.motion_stop_pending_since = None

        if not self.in_motion:
            return

        # MCP snapshot
        snap = self.mcp.snapshot()
        if snap is None:
            return

        mcp_pos = pick(snap.get("arm_pos", []), snap.get("arm_names", []), ARM_JOINTS)
        mcp_vel = pick(snap.get("arm_vel", []), snap.get("arm_names", []), ARM_JOINTS)
        grip_pq = snap.get("gripper_world_pq")
        tcp_xyz = tcp_from_gripper(grip_pq) if grip_pq else (None, None, None)
        cups = snap.get("cups") or {}
        legos = snap.get("legos") or {}

        # /joint_states view
        if js:
            _, js_names, js_pos, js_vel = js
            js_pos_ordered = pick(js_pos, js_names, ARM_JOINTS)
            js_vel_ordered = pick(js_vel, js_names, ARM_JOINTS)
        else:
            js_pos_ordered = [None] * len(ARM_JOINTS)
            js_vel_ordered = [None] * len(ARM_JOINTS)

        # Per-joint isaac lag (ref - mcp)
        isaac_lag = [
            (r - m) if (m is not None) else None
            for r, m in zip(ref_pos_ordered, mcp_pos)
        ]

        # Running stats
        for i, v in enumerate(ref_vel_ordered):
            if abs(v) > self.peak_ref_vel[i]:
                self.peak_ref_vel[i] = abs(v)
        for i, e in enumerate(err_pos_ordered):
            if abs(e) > self.peak_abs_err[i]:
                self.peak_abs_err[i] = abs(e)
        for i, g in enumerate(isaac_lag):
            if g is not None and abs(g) > self.peak_isaac_lag[i]:
                self.peak_isaac_lag[i] = abs(g)

        # Sim-rate running stats
        st = snap.get("sim_time_s")
        if st is not None:
            if self.sim_time_start is None:
                self.sim_time_start = st
                self.wall_time_start_for_rtf = t_wall
            self.sim_time_last = st
        fps = snap.get("render_fps")
        if fps is not None and fps > 0:
            self.fps_samples.append(fps)
        adt = snap.get("app_dt_ms")
        if adt is not None and adt > 0:
            self.app_dt_samples.append(adt)

        # Cup proximity + displacement
        if tcp_xyz and tcp_xyz[0] is not None:
            for name, cb in cups.items():
                if cb is None:
                    continue
                dx = tcp_xyz[0] - cb[0]
                dy = tcp_xyz[1] - cb[1]
                dz = tcp_xyz[2] - cb[2]
                dh = math.sqrt(dx * dx + dy * dy)
                if dh < self.per_cup_closest.get(name, 1e9):
                    self.per_cup_closest[name] = dh
        for name, cb in cups.items():
            if cb is None:
                continue
            if name not in self.per_cup_start_pos:
                self.per_cup_start_pos[name] = (cb[0], cb[1], cb[2])
            sx, sy, sz = self.per_cup_start_pos[name]
            d = math.sqrt((cb[0] - sx) ** 2 + (cb[1] - sy) ** 2 + (cb[2] - sz) ** 2)
            if d > self.per_cup_max_disp.get(name, 0.0):
                self.per_cup_max_disp[name] = d

        # Write row
        row = {
            "t_wall": t_wall,
            "t_sim_ns": clock_ns if clock_ns is not None else 0,
        }
        for i, n in enumerate(ARM_JOINTS):
            row[f"ref_pos_{n}"] = ref_pos_ordered[i]
            row[f"ref_vel_{n}"] = ref_vel_ordered[i]
            row[f"fb_pos_{n}"] = fb_pos_ordered[i]
            row[f"err_pos_{n}"] = err_pos_ordered[i]
            row[f"js_pos_{n}"] = js_pos_ordered[i]
            row[f"js_vel_{n}"] = js_vel_ordered[i]
            row[f"isaac_pos_{n}"] = mcp_pos[i]
            row[f"isaac_vel_{n}"] = mcp_vel[i]
            row[f"isaac_lag_{n}"] = isaac_lag[i] if isaac_lag[i] is not None else ""
        if tcp_xyz and tcp_xyz[0] is not None:
            row["tcp_x"], row["tcp_y"], row["tcp_z"] = tcp_xyz
        else:
            row["tcp_x"] = row["tcp_y"] = row["tcp_z"] = ""
        for color in ("red", "green", "blue"):
            cb = cups.get(f"cup_{color}")
            row[f"cup_{color}_x"] = cb[0] if cb else ""
            row[f"cup_{color}_y"] = cb[1] if cb else ""
            row[f"cup_{color}_z"] = cb[2] if cb else ""
        # Contact events drained from the omni.physx contact-report
        # subscription (installed by DigitalTwin._setup_contact_sensors).
        # Each event is {t, type, a0, a1, c0, c1, impulse, impulse_mag,
        # position, n_contacts}. We keep the raw event list in a compact
        # JSON column and fold per-cup {hit, impulse_n} columns so the
        # analyzer can cheaply answer "which cup got hit in this sample."
        # See DEBUG-GUIDE § 4.4 for the interpretation rule.
        contacts_events = snap.get("contacts") or []
        per_cup_hit = {"red": 0, "green": 0, "blue": 0}
        per_cup_imp = {"red": 0.0, "green": 0.0, "blue": 0.0}
        for ev in contacts_events:
            for side in ("a0", "a1"):
                p = ev.get(side, "")
                for color in ("red", "green", "blue"):
                    if p.endswith(f"cup_{color}") or p.startswith(f"/World/Containers/cup_{color}"):
                        per_cup_hit[color] = 1
                        per_cup_imp[color] += float(ev.get("impulse_mag", 0.0))
        for color in ("red", "green", "blue"):
            row[f"contact_cup_{color}_hit"] = per_cup_hit[color]
            row[f"contact_cup_{color}_impulse_n"] = per_cup_imp[color]
        row["contact_n_events"] = len(contacts_events)
        row["contact_events_json"] = (
            json.dumps(contacts_events, separators=(",", ":"))
            if contacts_events else ""
        )
        row["attached_lego_xyz"] = ""  # filled below if we can infer
        # Sim rate telemetry (added for command-path latency diagnosis)
        row["physics_rate_hz"] = snap.get("physics_rate_hz", "")
        row["sim_time_s"] = snap.get("sim_time_s", "")
        row["render_fps"] = snap.get("render_fps", "")
        row["app_dt_ms"] = snap.get("app_dt_ms", "")
        # heuristic: the attached lego is the one whose z is rising with the arm
        # leave blank for now; post-analyzer can infer from motion_tag

        if self.current_writer:
            self.current_writer.writerow(row)
            self.current_samples += 1

    # ─── Motion lifecycle ───────────────────────────────────────────────

    def _latest_traj_dump(self, within_s: float = 2.0, now_wall: float = None):
        """Return (path, meta_dict) for a /tmp/arm_traj/*.json whose mtime is
        within `within_s` seconds of `now_wall` (defaults to time.time()).
        Only dumps freshly written by the control_gui for *this* motion qualify
        — older dumps (e.g. from a prior session) are ignored so the motion_tag
        reflects reality."""
        try:
            root = Path("/tmp/arm_traj")
            if not root.is_dir():
                return None, None
            now = now_wall if now_wall is not None else time.time()
            candidates = sorted(
                root.glob("*.json"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            for path in candidates:
                age = now - path.stat().st_mtime
                if abs(age) <= within_s:
                    d = json.loads(path.read_text())
                    return str(path), d
                # candidates are newest-first; once we fall out of window we're done
                break
            return None, None
        except Exception:
            return None, None

    def _start_motion(self, t_wall, start_ref_pos):
        self.in_motion = True
        self.motion_start_wall = t_wall
        self.motion_stop_pending_since = None
        self.current_samples = 0
        for i in range(len(ARM_JOINTS)):
            self.peak_ref_vel[i] = 0.0
            self.peak_abs_err[i] = 0.0
            self.peak_isaac_lag[i] = 0.0
        for k in self.per_cup_closest:
            self.per_cup_closest[k] = 1e9
        self.per_cup_start_pos.clear()
        self.per_cup_max_disp.clear()
        self.sim_time_start = None
        self.sim_time_last = None
        self.wall_time_start_for_rtf = None
        self.fps_samples.clear()
        self.app_dt_samples.clear()

        motion_tag = "unknown"
        traj_path = None
        if self.motion_tag_from_dumps:
            tp, td = self._latest_traj_dump(within_s=2.0, now_wall=t_wall)
            if td:
                tag = (td.get("scene_at_plan_time") or {}).get("motion_tag")
                if tag:
                    motion_tag = tag
                    traj_path = tp

        date_dir = self.log_dir / datetime.fromtimestamp(t_wall).strftime("%Y-%m-%d")
        date_dir.mkdir(parents=True, exist_ok=True)
        stem = datetime.fromtimestamp(t_wall).strftime("%Y%m%dT%H%M%S") + f"_{motion_tag}"
        self.current_csv_path = date_dir / f"{stem}.csv"

        header = ["t_wall", "t_sim_ns"]
        for n in ARM_JOINTS:
            header += [
                f"ref_pos_{n}", f"ref_vel_{n}", f"fb_pos_{n}", f"err_pos_{n}",
                f"js_pos_{n}", f"js_vel_{n}",
                f"isaac_pos_{n}", f"isaac_vel_{n}", f"isaac_lag_{n}",
            ]
        header += ["tcp_x", "tcp_y", "tcp_z"]
        for c in ("red", "green", "blue"):
            header += [f"cup_{c}_x", f"cup_{c}_y", f"cup_{c}_z"]
        # Per-cup hit/impulse aggregates + raw event list. Populated
        # when the PhysX contact-report subscription is installed; all
        # zero / empty otherwise (backward compatible).
        for c in ("red", "green", "blue"):
            header += [
                f"contact_cup_{c}_hit",
                f"contact_cup_{c}_impulse_n",
            ]
        header += ["contact_n_events", "contact_events_json"]
        header += ["attached_lego_xyz"]
        header += ["physics_rate_hz", "sim_time_s", "render_fps", "app_dt_ms"]

        f = self.current_csv_path.open("w", newline="")
        self.current_csv = f
        self.current_writer = csv.DictWriter(f, fieldnames=header)
        self.current_writer.writeheader()
        self.current_motion_meta = {
            "motion_tag": motion_tag,
            "traj_dump": traj_path,
            "start_wall": t_wall,
            "start_ref_pos": dict(zip(ARM_JOINTS, start_ref_pos)),
        }
        self.get_logger().info(
            f"[motion_start] tag={motion_tag}  csv={self.current_csv_path.name}"
        )

    def _end_motion(self, t_wall):
        if not self.in_motion:
            return
        dur = t_wall - (self.motion_start_wall or t_wall)
        self.in_motion = False
        self.motion_stop_pending_since = None
        if self.current_csv:
            self.current_csv.close()
        self.current_csv = None
        self.current_writer = None

        meta = dict(self.current_motion_meta)
        meta["end_wall"] = t_wall
        meta["duration_s"] = dur
        meta["samples"] = self.current_samples
        meta["peak_ref_vel_rad_s"] = dict(zip(ARM_JOINTS, self.peak_ref_vel))
        meta["peak_abs_err_ros2_control_rad"] = dict(zip(ARM_JOINTS, self.peak_abs_err))
        meta["peak_isaac_lag_rad"] = dict(zip(ARM_JOINTS, self.peak_isaac_lag))
        meta["peak_isaac_lag_deg"] = {
            k: math.degrees(v) for k, v in zip(ARM_JOINTS, self.peak_isaac_lag)
        }
        meta["closest_approach_to_cups_mm"] = {
            k: (v * 1000.0 if v < 1e8 else None)
            for k, v in self.per_cup_closest.items()
        }
        meta["cup_displacement_during_motion_mm"] = {
            k: v * 1000.0 for k, v in self.per_cup_max_disp.items()
        }
        max_lag = max(self.peak_isaac_lag) if self.peak_isaac_lag else 0.0
        min_lag = min(self.peak_isaac_lag) if self.peak_isaac_lag else 0.0
        meta["differential_lag_max_minus_min_rad"] = max_lag - min_lag
        meta["differential_lag_max_minus_min_deg"] = math.degrees(max_lag - min_lag)
        # Sim-rate summary
        rtf = None
        if (self.sim_time_start is not None and self.sim_time_last is not None
                and self.wall_time_start_for_rtf is not None):
            dwall = t_wall - self.wall_time_start_for_rtf
            dsim = self.sim_time_last - self.sim_time_start
            if dwall > 0:
                rtf = dsim / dwall
        meta["realtime_factor"] = rtf
        meta["sim_time_advance_s"] = (
            (self.sim_time_last - self.sim_time_start)
            if (self.sim_time_start is not None and self.sim_time_last is not None)
            else None
        )
        meta["render_fps_mean"] = (
            sum(self.fps_samples) / len(self.fps_samples) if self.fps_samples else None
        )
        meta["render_fps_min"] = min(self.fps_samples) if self.fps_samples else None
        meta["app_dt_ms_mean"] = (
            sum(self.app_dt_samples) / len(self.app_dt_samples)
            if self.app_dt_samples else None
        )
        meta["app_dt_ms_max"] = max(self.app_dt_samples) if self.app_dt_samples else None

        meta_path = self.current_csv_path.with_suffix(".json")
        meta_path.write_text(json.dumps(meta, indent=2, default=str))
        self.get_logger().info(
            f"[motion_end]   tag={meta['motion_tag']}  dur={dur:.2f}s  "
            f"samples={self.current_samples}  "
            f"peak_lag_max={math.degrees(max_lag):.2f}°  "
            f"diff_lag={meta['differential_lag_max_minus_min_deg']:.2f}°  "
            f"cup_disp_mm={max(meta['cup_displacement_during_motion_mm'].values(), default=0):.1f}"
        )
        self.current_csv_path = None
        self.current_motion_meta = {}


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="SO-ARM101 motion telemetry logger")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/motion_logs"),
                    help="Root output directory (default ~/motion_logs)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="Sample rate in Hz (default 50)")
    ap.add_argument("--motion-tag-from-dumps", action="store_true", default=True,
                    help="Read motion tag from latest /tmp/arm_traj/*.json (default on)")
    args = ap.parse_args()

    rclpy.init()
    node = MotionLogger(Path(args.log_dir), args.rate, args.motion_tag_from_dumps)

    stop = threading.Event()

    def on_sig(sig, frame):
        stop.set()

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    period = 1.0 / args.rate

    # Pump rclpy in a worker so our main loop can sleep tight
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    try:
        while not stop.is_set():
            t0 = time.time()
            try:
                node.sample_once()
            except Exception as e:
                node.get_logger().error(f"sample error: {e}")
            dt = time.time() - t0
            sleep_s = period - dt
            if sleep_s > 0:
                stop.wait(timeout=sleep_s)
    finally:
        # Flush any open motion
        if node.in_motion:
            node._end_motion(time.time())
        node.destroy_node()
        rclpy.shutdown()
        print("motion_logger: stopped cleanly", file=sys.stderr)


if __name__ == "__main__":
    main()
