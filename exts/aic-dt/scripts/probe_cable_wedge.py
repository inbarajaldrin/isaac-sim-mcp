"""Empirical test: does the cable subtree wedge PhysX in CURRENT launch conditions?

Live test plan:
  1. Stop the running simulation
  2. Activate the cable prim (SetActive(True))
  3. Probe cable subtree health (joint count, mass/inertia, articulation parent)
  4. Play simulation and step several physics ticks
  5. Report whether physics step responds or wedges

If physics wedges, the test will time out (we send the play+step request with a
deadline; if the response doesn't return within 30s, the main thread is blocked
on futex_wait — same wedge prior session reported).

If physics responds within reasonable time (a few seconds), the cable plays and
SetActive(False) is no longer required.
"""
import json, socket, time

PAYLOAD = r"""
import omni.usd
import omni.timeline
from pxr import Usd, UsdPhysics

stage = omni.usd.get_context().get_stage()
out = []

cable_prim = stage.GetPrimAtPath("/World/UR5e/cable")
if not (cable_prim and cable_prim.IsValid()):
    result = "NO CABLE PRIM at /World/UR5e/cable"
else:
    out.append(f"Cable prim exists: {cable_prim.GetPath()}")
    out.append(f"  current Active: {cable_prim.IsActive()}")

    # Stop simulation first
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
        out.append("Simulation stopped.")

    # Activate the cable
    cable_prim.SetActive(True)
    out.append(f"Cable activated: IsActive={cable_prim.IsActive()}")

    # Probe cable subtree
    joint_types = {}
    rb_count = 0
    for p in Usd.PrimRange(cable_prim):
        t = p.GetTypeName()
        if t.startswith("Physics") and "Joint" in t:
            joint_types[t] = joint_types.get(t, 0) + 1
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rb_count += 1
    out.append(f"Cable subtree: {rb_count} RigidBody prims, joints: {joint_types}")

    # Sample mass/inertia of first few cable links + joints
    samples = []
    for p in Usd.PrimRange(cable_prim):
        if p.HasAPI(UsdPhysics.MassAPI) and len(samples) < 3:
            mass_api = UsdPhysics.MassAPI(p)
            mass = mass_api.GetMassAttr().Get()
            inertia = mass_api.GetDiagonalInertiaAttr().Get()
            samples.append(f"{p.GetName()}: mass={mass}, inertia={inertia}")
    out.append("Sample mass/inertia: " + " | ".join(samples) if samples else "no MassAPI on cable links")

result = "\n".join(out)
"""

req = {"type": "execute_python_code", "params": {"code": PAYLOAD}}
s = socket.socket(); s.settimeout(60); s.connect(("127.0.0.1", 8768))
s.sendall(json.dumps(req).encode())
buf = b""
while True:
    chunk = s.recv(16384)
    if not chunk: break
    buf += chunk
    try:
        resp = json.loads(buf.decode()); break
    except json.JSONDecodeError:
        continue
s.close()
print("=== STEP 1: Cable activation + probe ===")
print(resp.get("result", {}).get("result", "<no result>"))

# Step 2: try play_scene with timeout
print("\n=== STEP 2: Play simulation with 30s timeout ===")
play_req = {"type": "play_scene", "params": {}}
s = socket.socket(); s.settimeout(30); s.connect(("127.0.0.1", 8768))
s.sendall(json.dumps(play_req).encode())
t0 = time.time()
try:
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk: break
        buf += chunk
        try:
            resp = json.loads(buf.decode()); break
        except json.JSONDecodeError:
            continue
    elapsed = time.time() - t0
    print(f"play_scene returned in {elapsed:.2f}s: {resp}")
except socket.timeout:
    elapsed = time.time() - t0
    print(f"play_scene TIMED OUT after {elapsed:.1f}s — likely wedged on futex_wait")
finally:
    s.close()

# Step 3: probe whether physics is actually advancing
print("\n=== STEP 3: Step physics + check time advancement ===")
TICK_PROBE = r"""
import omni.timeline
import time
tl = omni.timeline.get_timeline_interface()
out = []
out.append(f"is_playing={tl.is_playing()}")
t0 = tl.get_current_time()
out.append(f"timeline t0={t0}")
# We can't easily block-wait here from inside execute_python_code without
# wedging the main MCP loop, so just sample and return.
result = "\n".join(out)
"""
req = {"type": "execute_python_code", "params": {"code": TICK_PROBE}}
s = socket.socket(); s.settimeout(15); s.connect(("127.0.0.1", 8768))
s.sendall(json.dumps(req).encode())
t0 = time.time()
try:
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk: break
        buf += chunk
        try:
            resp = json.loads(buf.decode()); break
        except json.JSONDecodeError:
            continue
    elapsed = time.time() - t0
    print(f"timeline probe returned in {elapsed:.2f}s")
    print(resp.get("result", {}).get("result", resp))
except socket.timeout:
    elapsed = time.time() - t0
    print(f"timeline probe TIMED OUT after {elapsed:.1f}s — main thread blocked")
finally:
    s.close()
