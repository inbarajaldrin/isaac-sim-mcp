#!/usr/bin/env python3
"""Perf diagnosis: isolate render-FPS vs physics-RTF contribution of the Vention
bench + Simple Room, by rebuilding the scene in stages and measuring each.

All waiting happens HERE (between socket calls) so Kit's main loop runs freely —
sleeping inside execute_python_code freezes the loop and zeroes sim-time.
"""
import socket, json, time

PORT = 8766

def call(cmd, params=None, timeout=300):
    msg = json.dumps({"type": cmd, "params": params or {}})
    s = socket.socket(); s.settimeout(timeout); s.connect(("localhost", PORT))
    s.sendall(msg.encode()); data = b""
    while True:
        c = s.recv(16384)
        if not c: break
        data += c
        try: json.loads(data.decode()); break
        except json.JSONDecodeError: continue
    s.close(); return json.loads(data.decode())

def pcode(code):
    return call("execute_python_code", {"code": code}).get("result", {}).get("result")

READ_FPS = r'''
from omni.kit.viewport.utility import get_active_viewport
vp=get_active_viewport()
result=round(float(vp.fps),3) if vp.fps else 0.0
'''
READ_SIMTIME = r'''
import omni.timeline
result=round(omni.timeline.get_timeline_interface().get_current_time(),5)
'''
COUNTS = r'''
import omni.usd
st=omni.usd.get_context().get_stage()
nprim=nmesh=nmat=0
for p in st.Traverse():
    nprim+=1; t=p.GetTypeName()
    if t=="Mesh": nmesh+=1
    elif t=="Material": nmat+=1
result={"prims":nprim,"meshes":nmesh,"materials":nmat}
'''
def set_render(on):
    pcode(f'''
from omni.kit.viewport.utility import get_active_viewport
get_active_viewport().updates_enabled={'True' if on else 'False'}
result="ok"
''')

def avg_fps(n=4, gap=0.8):
    vals=[]
    for _ in range(n):
        v=pcode(READ_FPS)
        if isinstance(v,(int,float)): vals.append(v)
        time.sleep(gap)
    return round(sum(vals)/len(vals),2) if vals else None

def rtf(window=2.0):
    t0=pcode(READ_SIMTIME); w0=time.time()
    time.sleep(window)
    t1=pcode(READ_SIMTIME); w1=time.time()
    dw=w1-w0
    return round((t1-t0)/dw,3) if (dw>0 and t0 is not None and t1 is not None) else None

def measure(label, settle=5.0):
    print(f"  measuring [{label}] (settle {settle}s)...", flush=True)
    time.sleep(settle)
    counts=pcode(COUNTS)
    fps_render_on=avg_fps()
    rtf_render_on=rtf()
    set_render(False); time.sleep(1.0)
    rtf_render_off=rtf()
    set_render(True); time.sleep(0.5)
    row={"scenario":label,"prims":counts.get("prims"),"meshes":counts.get("meshes"),
         "materials":counts.get("materials"),"fps_render_on":fps_render_on,
         "rtf_render_on":rtf_render_on,"rtf_render_off":rtf_render_off}
    print("   ->", json.dumps(row), flush=True)
    return row

def main():
    rows=[]
    print("S0: new_stage + quick_start (robot only)...", flush=True)
    print("   new_stage:", call("new_stage",timeout=60).get("result",{}).get("message","?")[:40], flush=True)
    time.sleep(2)
    call("quick_start", timeout=240)
    rows.append(measure("robot only (quick_start)"))

    print("S1: + load_workstation...", flush=True)
    call("load_workstation", timeout=120)
    rows.append(measure("+ vention"))

    print("S2: + load_simple_room...", flush=True)
    call("load_simple_room", timeout=120)
    rows.append(measure("+ vention + simple room"))

    # markdown table
    print("\n\n=== RESULTS ===")
    hdr=["scenario","prims","meshes","materials","fps_render_on","rtf_render_on","rtf_render_off"]
    print("| "+" | ".join(hdr)+" |")
    print("|"+"|".join(["---"]*len(hdr))+"|")
    for r in rows:
        print("| "+" | ".join(str(r.get(h)) for h in hdr)+" |")

if __name__=="__main__":
    main()
