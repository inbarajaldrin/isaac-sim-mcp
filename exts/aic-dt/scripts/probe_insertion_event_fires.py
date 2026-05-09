#!/usr/bin/env python3
"""Drive a manual plug-port collision and observe /scoring/insertion_event.

Sister probe to probe_taskboard_authoring.py — validates the RUNTIME contact
report pipeline for the kinematic_enabled task-board prims.

Acceptance gate 2 of taskboard-prim-authoring (plans/prd.json):
  /scoring/insertion_event fires when a plug contacts its target port
  (verified by ros2 topic echo /scoring/insertion_event during a manual
  plug-port collision via MCP).

Strategy:
  1. Pop SCENE_05_DISABLE from sim's env (motion-deficit-hunt iter set it
     for HYP-A testing; cable subtree therefore inactive). Drive new_stage +
     quick_start so load_robot activates the cable (per extension.py
     L1354-1360 — env-var-gated SetActive(True/False)) and AicScoringPublishers
     re-runs PhysxContactReportAPI tagging on the live plug + port subtrees.
  2. Verify cable plug end (`/World/UR5e/cable/Rope/Rope/link_20`) is
     PhysxContactReportAPI-tagged AND the 4 kinematic-enabled task-board prims
     are PhysxContactReportAPI-tagged.
  3. Spawn an rclpy subscriber subprocess on /scoring/insertion_event.
  4. Teleport `/World/TaskBoard/SCPort_0` parent (kinematic_enabled visual
     descendant) so its visual mesh world-position lands at the cable plug
     end. Kinematic body pose is updated through xformOp:translate; PhysX
     respects this on the next physics step.
  5. Hold the teleport for ~7s (≥5 ticks at 60Hz to satisfy
     scoring_publishers._INSERTION_CONTACT_TICKS_REQUIRED + buffer).
  6. Read subscriber subprocess output for FINAL COUNT.
  7. Restore SCENE_05_DISABLE='1' in the sim env so the next iteration
     starts from the motion-deficit-hunt baseline (no quick_start needed —
     the next iter will issue its own).

Exit 0 if at least one /scoring/insertion_event message arrived during the
teleport hold; exit 1 otherwise.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import textwrap
import time
from pathlib import Path

MCP_HOST = "localhost"
MCP_PORT = 8768
ROS_DOMAIN_ID = "7"
SUBSCRIBER_LIFETIME_S = 12.0
TELEPORT_HOLD_S = 9.0

PORT_PARENT_PATH = "/World/TaskBoard/SCPort_0"
PORT_VISUAL_PATH = f"{PORT_PARENT_PATH}/sc_port_visual"
PLUG_END_PATH = "/World/UR5e/cable/Rope/Rope/link_20"


def _send_mcp(req: dict, timeout: float = 180.0) -> dict:
    s = socket.socket()
    s.settimeout(timeout)
    s.connect((MCP_HOST, MCP_PORT))
    s.sendall(json.dumps(req).encode())
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk:
            break
        buf += chunk
        try:
            return json.loads(buf.decode())
        except json.JSONDecodeError:
            continue
    raise RuntimeError("MCP socket closed before complete JSON")


def _exec(code: str, timeout: float = 180.0) -> dict:
    return _send_mcp({"type": "execute_python_code", "params": {"code": code}}, timeout=timeout)


def _exec_result(code: str, timeout: float = 180.0):
    r = _exec(code, timeout=timeout)
    return (r.get("result") or {}).get("result")


def _quick_start_with_recovery() -> str:
    qr = _send_mcp({"type": "quick_start", "params": {}})
    msg = (qr.get("result") or {}).get("message", "")
    status = (qr.get("result") or {}).get("status") or qr.get("status")
    if status == "error" and "name is not unique" in msg.lower():
        _send_mcp({"type": "new_stage", "params": {}})
        time.sleep(1.0)
        qr = _send_mcp({"type": "quick_start", "params": {}})
        msg = (qr.get("result") or {}).get("message", "")
    return msg


def _activate_cable_and_load_trial() -> bool:
    """Pop SCENE_05_DISABLE so cable subtree activates, then drive load_trial
    trial_1 — load_trial issues its own new_stage + load_scene + load_robot
    + spawn-atom dispatch + parity_publishers + scoring_publishers with
    live CamelCase port paths (D-13 fallback, scoring_publishers.py:62-65).
    Without load_trial the default _PORT_LINK_PATHS is snake_case and tagging
    misses the live spawned ports."""
    print("[probe_insertion_event] popping SCENE_05_DISABLE so cable activates")
    _exec("import os; os.environ.pop('SCENE_05_DISABLE', None); result = 'popped'")
    print("[probe_insertion_event] load_trial trial_1 (canonical scene incl. live port paths)")
    lr = _send_mcp(
        {"type": "load_trial", "params": {"trial_key": "trial_1", "ground_truth": True}},
        timeout=240.0,
    )
    payload = lr.get("result") or {}
    status = payload.get("status") or lr.get("status")
    if status != "success":
        msg = payload.get("message") or lr.get("message") or str(lr)[:200]
        print(f"[probe_insertion_event] load_trial failed: {msg}")
        return False
    spawned = payload.get("spawned_components") or []
    print(f"[probe_insertion_event] load_trial OK — {len(spawned)} components spawned")
    # Allow physics + tensor-view + scoring tagging to settle. RigidPrim
    # tensor backend needs ~3-5s on freshly-restarted sim.
    time.sleep(5.0)
    return True


def _probe_tagging() -> dict:
    code = textwrap.dedent("""
        from pxr import UsdPhysics, PhysxSchema
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        out = {}
        plug = stage.GetPrimAtPath("/World/UR5e/cable/Rope/Rope/link_20")
        out["plug_active"] = bool(plug and plug.IsValid() and plug.IsActive())
        out["plug_contact_report"] = (plug.HasAPI(PhysxSchema.PhysxContactReportAPI) if plug and plug.IsValid() else False)
        port_paths = [
            "/World/TaskBoard/base_visual",
            "/World/TaskBoard/SCPort_0/sc_port_visual",
            "/World/TaskBoard/SCPort_1/sc_port_visual",
            "/World/TaskBoard/NICCard/nic_card_visual",
        ]
        for path in port_paths:
            p = stage.GetPrimAtPath(path)
            if p and p.IsValid():
                out[path] = {
                    "kinematic_enabled": (UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get() if p.HasAPI(UsdPhysics.RigidBodyAPI) else None),
                    "contact_report": p.HasAPI(PhysxSchema.PhysxContactReportAPI),
                }
            else:
                out[path] = None
        result = out
    """)
    return _exec_result(code) or {}


def _read_world_translate(prim_path: str) -> tuple | None:
    code = textwrap.dedent(f"""
        import omni.usd
        from pxr import UsdGeom
        stage = omni.usd.get_context().get_stage()
        p = stage.GetPrimAtPath({prim_path!r})
        if not (p and p.IsValid() and p.IsActive()):
            result = None
        else:
            xfc = UsdGeom.XformCache()
            t = xfc.GetLocalToWorldTransform(p).ExtractTranslation()
            result = (float(t[0]), float(t[1]), float(t[2]))
    """)
    r = _exec_result(code)
    if r is None:
        return None
    return tuple(r)


def _teleport_kinematic_to_world(rb_path: str, world_target: tuple) -> dict:
    """Teleport a kinematic rigid body to world_target. Tries the PhysX-aware
    `isaacsim.core.prims.RigidPrim.set_world_poses` first (which sets both USD
    xform and PhysX kinematic target via the tensor API); falls back to
    xformOp:translate on the body's parent (with parent-frame inversion) if
    RigidPrim init fails — common on a freshly-restarted sim where the
    physics tensor view hasn't fully populated yet.
    """
    tx, ty, tz = float(world_target[0]), float(world_target[1]), float(world_target[2])
    parent_path = "/".join(rb_path.rstrip("/").split("/")[:-1]) or "/"
    code = textwrap.dedent(f"""
        import numpy as np
        import omni.usd
        from pxr import UsdGeom, Gf
        out = {{"target_world": ({tx}, {ty}, {tz}), "primary": None, "fallback": None, "actual_world": None}}
        try:
            from isaacsim.core.prims import RigidPrim
            rp = RigidPrim(prim_paths_expr={rb_path!r})
            rp.set_world_poses(positions=np.array([[{tx}, {ty}, {tz}]], dtype=np.float32))
            pos_after, _ = rp.get_world_poses()
            out["primary"] = "RigidPrim.set_world_poses"
            out["actual_world"] = (float(pos_after[0][0]), float(pos_after[0][1]), float(pos_after[0][2]))
        except Exception as e:
            out["primary"] = f"RigidPrim failed: {{e!r}}"
            try:
                stage = omni.usd.get_context().get_stage()
                prim = stage.GetPrimAtPath({rb_path!r})
                parent = stage.GetPrimAtPath({parent_path!r})
                xfc = UsdGeom.XformCache()
                parent_world = xfc.GetLocalToWorldTransform(parent)
                inv = parent_world.GetInverse()
                local_target = inv.Transform(Gf.Vec3d({tx}, {ty}, {tz}))
                xform = UsdGeom.Xform(parent)
                done = False
                for op in xform.GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:translate":
                        op.Set(Gf.Vec3d(float(local_target[0]), float(local_target[1]), float(local_target[2])))
                        done = True
                        break
                if not done:
                    xform.AddTranslateOp().Set(Gf.Vec3d(float(local_target[0]), float(local_target[1]), float(local_target[2])))
                xfc2 = UsdGeom.XformCache()
                actual = xfc2.GetLocalToWorldTransform(prim).ExtractTranslation()
                out["fallback"] = "xformOp parent-inverse"
                out["actual_world"] = (float(actual[0]), float(actual[1]), float(actual[2]))
            except Exception as e2:
                out["fallback"] = f"xformOp failed: {{e2!r}}"
        result = out
    """)
    rr = _exec_result(code) or {}
    print(f"[probe_insertion_event] teleport {rb_path} → world {world_target}: {rr}")
    return rr


def _spawn_subscriber(lifetime_s: float) -> tuple[subprocess.Popen, str]:
    """Spawn an rclpy subscriber for /scoring/insertion_event via the host
    zenoh router. Isaac Sim publishes via rmw_zenoh_cpp (per zenoh-path-
    implementation iter); a fastrtps subscriber would silently miss every
    message.
    """
    sub_src = textwrap.dedent("""
        import os, sys, time
        import rclpy
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("probe_insertion_event_subscriber")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=20)
        count = [0]
        samples = []
        def cb(msg):
            count[0] += 1
            samples.append(msg.data)
            print("[sub] received: %r (#%d)" % (msg.data, count[0]), flush=True)
        node.create_subscription(String, "/scoring/insertion_event", cb, qos)
        print("[sub] subscriber READY", flush=True)
        deadline = time.time() + float(sys.argv[1])
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        print("[sub] FINAL_COUNT=%d" % count[0], flush=True)
        print("[sub] SAMPLES=%r" % (samples,), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    """)
    sub_path = "/tmp/probe_insertion_event_subscriber.py"
    Path(sub_path).write_text(sub_src)
    log_path = "/tmp/probe_insertion_event_subscriber.log"
    open(log_path, "w").close()
    # Source /opt/ros/humble (apt rmw_zenoh_cpp lives there) THEN env_isaaclab,
    # then export RMW + ZENOH peer pointing at host router. Mirrors the launch
    # discipline in run_aic_engine_against_isaac_sim.sh / launch_host_zenohd.sh.
    inline = (
        "set +u; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/env_isaaclab/bin/activate; "
        "set -u; "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp; "
        "export ZENOH_ROUTER_CHECK_ATTEMPTS=-1; "
        'export ZENOH_CONFIG_OVERRIDE=\'connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false\'; '
        f"python3 -u {sub_path} {lifetime_s} > {log_path} 2>&1"
    )
    proc = subprocess.Popen(["bash", "-c", inline])
    return proc, log_path


def _restore_scene_05_disable() -> None:
    print("[probe_insertion_event] restoring SCENE_05_DISABLE=1 in sim env (motion-deficit baseline)")
    _exec("import os; os.environ['SCENE_05_DISABLE'] = '1'; result = 'restored'")


def _clean_scene_for_next_iter() -> None:
    """Issue a fresh new_stage so a subsequent probe / iter starts clean. The
    teleport pushed SCPort_0 well outside the canonical layout and the load_trial
    scene is partial vs the AIC_OBJECTS quick_start defaults — both can confuse
    sibling probes that don't issue their own new_stage."""
    try:
        _send_mcp({"type": "new_stage", "params": {}}, timeout=20.0)
        print("[probe_insertion_event] scene cleaned (new_stage)")
    except Exception as e:
        print(f"[probe_insertion_event] WARN — new_stage cleanup failed: {e!r}")


def main() -> int:
    if not _activate_cable_and_load_trial():
        print("[probe_insertion_event] FAIL — load_trial did not complete cleanly")
        _restore_scene_05_disable()
        return 1

    audit = _probe_tagging()
    print(f"[probe_insertion_event] tagging audit:\n{json.dumps(audit, indent=2)}")
    if not audit.get("plug_active"):
        print("[probe_insertion_event] FAIL — cable plug not active after pop+quick_start")
        _restore_scene_05_disable()
        return 1
    if not audit.get("plug_contact_report"):
        print("[probe_insertion_event] FAIL — plug missing PhysxContactReportAPI")
        _restore_scene_05_disable()
        return 1
    port_visual_audit = audit.get(PORT_VISUAL_PATH) or {}
    if not (port_visual_audit.get("kinematic_enabled") is True
            and port_visual_audit.get("contact_report") is True):
        print(f"[probe_insertion_event] FAIL — {PORT_VISUAL_PATH} not kinematic_enabled+contact_report")
        _restore_scene_05_disable()
        return 1

    plug_pos = _read_world_translate(PLUG_END_PATH)
    port_visual_pos = _read_world_translate(PORT_VISUAL_PATH)
    if plug_pos is None or port_visual_pos is None:
        print(f"[probe_insertion_event] FAIL — could not read positions: plug={plug_pos} visual={port_visual_pos}")
        _restore_scene_05_disable()
        return 1
    print(f"[probe_insertion_event] plug @ world {plug_pos}")
    print(f"[probe_insertion_event] port_visual @ world {port_visual_pos}")

    print("[probe_insertion_event] starting rclpy subscriber subprocess")
    proc, log_path = _spawn_subscriber(SUBSCRIBER_LIFETIME_S)
    # Wait for subscriber READY (drives discovery before stimulus)
    ready_deadline = time.time() + 6.0
    while time.time() < ready_deadline:
        try:
            if "subscriber READY" in Path(log_path).read_text():
                break
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    else:
        print("[probe_insertion_event] WARN — subscriber not READY within 6s")

    # Teleport SCPort_0/sc_port_visual (kinematic_enabled rigid body) onto
    # plug end via the PhysX-aware RigidPrim API. Plain xformOp updates during
    # play don't reliably propagate to PhysX kinematic actor pose; RigidPrim
    # handles kinematicTarget plumbing.
    _teleport_kinematic_to_world(PORT_VISUAL_PATH, plug_pos)
    time.sleep(TELEPORT_HOLD_S)

    # Cleanup: don't bother teleporting back — _restore_scene_05_disable below
    # plus the next iter's quick_start will re-spawn cleanly.
    _restore_scene_05_disable()

    try:
        proc.wait(timeout=max(2.0, SUBSCRIBER_LIFETIME_S - TELEPORT_HOLD_S + 4.0))
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    out = ""
    try:
        out = Path(log_path).read_text()
    except FileNotFoundError:
        pass
    print("[probe_insertion_event] subscriber output:")
    for line in out.splitlines():
        print(f"  {line}")

    final = 0
    for line in (out or "").splitlines():
        if "FINAL_COUNT=" in line:
            try:
                final = int(line.split("FINAL_COUNT=", 1)[1].strip())
            except ValueError:
                final = 0
    print(f"[probe_insertion_event] /scoring/insertion_event message count = {final}")
    _clean_scene_for_next_iter()
    if final >= 1:
        print("[probe_insertion_event] PASS — at least one insertion event observed")
        return 0
    print("[probe_insertion_event] FAIL — no /scoring/insertion_event messages observed during teleport hold")
    return 1


if __name__ == "__main__":
    sys.exit(main())
