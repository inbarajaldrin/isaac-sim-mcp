"""Probe whether the active cable actually responds to physics or is decorative.

Sample world poses of cable link_0 (root) and link_20 (tip) at sim_t=N and
sim_t=N+1s. If poses change, cable is physics-active. If poses identical,
cable is kinematically frozen (zero-mass-bodies-skip behavior in PhysX).

Also check: gravity should pull a free chain downward. If link_20.z dropped
relative to link_0.z over 1s, gravity is acting on the cable.
"""
import json, socket, time

PROBE = r"""
from pxr import UsdGeom, Usd
import omni.usd
stage = omni.usd.get_context().get_stage()

samples = {}
for name in ["link_0", "link_5", "link_10", "link_15", "link_20"]:
    p = stage.GetPrimAtPath(f"/World/UR5e/cable/Rope/Rope/{name}")
    if p and p.IsValid():
        x = UsdGeom.Xformable(p)
        m = x.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        samples[name] = (float(t[0]), float(t[1]), float(t[2]))
result = str(samples)
"""

def probe():
    s = socket.socket(); s.settimeout(15); s.connect(("127.0.0.1", 8768))
    s.sendall(json.dumps({"type": "execute_python_code", "params": {"code": PROBE}}).encode())
    buf = b""
    while True:
        chunk = s.recv(16384)
        if not chunk: break
        buf += chunk
        try: resp = json.loads(buf.decode()); break
        except: continue
    s.close()
    return eval(resp.get("result", {}).get("result", "{}"))

print("=== T+0.0s ===")
poses_0 = probe()
for n, p in poses_0.items():
    print(f"  {n}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")

print("\nWaiting 2s of wall time (~1.3s sim time)...")
time.sleep(2)

print("\n=== T+2.0s ===")
poses_1 = probe()
for n, p in poses_1.items():
    p0 = poses_0.get(n, (0,0,0))
    dx = p[0] - p0[0]; dy = p[1] - p0[1]; dz = p[2] - p0[2]
    delta_norm = (dx**2 + dy**2 + dz**2) ** 0.5
    print(f"  {n}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  Δ={delta_norm*1000:.2f}mm")

# Verdict
total_motion = sum(
    sum((poses_1[n][i] - poses_0[n][i])**2 for i in range(3))**0.5
    for n in poses_1 if n in poses_0
)
print(f"\nTotal motion across {len(poses_0)} sampled cable links: {total_motion*1000:.2f}mm over 2s wall")
print(f"Verdict: {'PHYSICS ACTIVE — cable moves' if total_motion > 0.001 else 'CABLE FROZEN (zero-mass kinematic skip; not really simulating)'}")
