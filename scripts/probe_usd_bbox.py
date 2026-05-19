#!/usr/bin/env python3
"""Probe the world-frame AABB of a USD prim via the running aic-dt MCP socket.

Use this after authoring a GLB → USDA wrapper to cross-check that the
final live bbox matches the real-world dimensions you expected. Critical
for catching mm-vs-m and Y-up-vs-Z-up errors that the wrapper was supposed
to correct.

USAGE:
    python3 probe_usd_bbox.py /World/AIC_Enclosure
    python3 probe_usd_bbox.py /World/AIC_Floor /World/AIC_Floor/FloorVisual
    python3 probe_usd_bbox.py --port 8768 /World/UR5e

Each prim path is probed independently. Output shows min, max, size in m.

Expected dimensions for the AIC scene (cross-check reference):
    /World/AIC_Enclosure        ~ 1.5  ×  1.5  ×  2.58 m
    /World/AIC_Floor            ~10.14 × 10.14 ×  7.05 m (outer warehouse)
    /World/AIC_Floor/FloorVisual~ 2.0  ×  2.0  ×  0.0  m (under-enclosure floor)
    /World/UR5e                 < ~1.5m reach radius from base
"""

import argparse
import json
import socket
import sys


def send(payload, port=8768, timeout=30):
    s = socket.socket(); s.settimeout(timeout); s.connect(("localhost", port))
    s.sendall(json.dumps(payload).encode())
    buf = b""
    while True:
        c = s.recv(16384)
        if not c: break
        buf += c
        try: return json.loads(buf.decode())
        except json.JSONDecodeError: continue
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="USD prim path(s) to probe")
    ap.add_argument("--port", type=int, default=8768, help="MCP socket port (default 8768 = aic-dt)")
    args = ap.parse_args()

    paths_json = json.dumps(args.paths)
    code = f"""
from pxr import Usd, UsdGeom
import omni.usd
stage = omni.usd.get_context().get_stage()
out = {{}}
for path in {paths_json}:
    p = stage.GetPrimAtPath(path)
    if not (p and p.IsValid()):
        out[path] = {{'exists': False}}
        continue
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render'])
    rng = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    if rng.GetMin()[0] > 1e30:
        out[path] = {{'exists': True, 'bbox': 'degenerate'}}
        continue
    out[path] = {{
        'exists': True,
        'type': str(p.GetTypeName()),
        'min_m':  [round(rng.GetMin()[i], 4) for i in range(3)],
        'max_m':  [round(rng.GetMax()[i], 4) for i in range(3)],
        'size_m': [round(rng.GetMax()[i] - rng.GetMin()[i], 4) for i in range(3)],
        'children': [c.GetName() for c in p.GetChildren()][:12],
    }}
result = out
"""
    r = send({"type": "execute_python_code", "params": {"code": code}}, port=args.port)
    if not r:
        print("MCP socket unreachable", file=sys.stderr)
        sys.exit(1)
    payload = r.get("result", r)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
