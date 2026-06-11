#!/usr/bin/env python3
"""Bundle a (Nucleus / remote / local) USD asset + ALL its MDL + texture
dependencies into a local folder via ``omni.kit.usd.collect``, driven over a
running Isaac Sim extension MCP socket.

WHY THIS EXISTS
---------------
``omni.client.copy`` copies only the ``.usd`` byte stream. It silently drops the
shader MDL modules and textures that materials reference by relative/absolute
asset paths, so after a plain copy the materials resolve to a fallback (e.g. a
flat untextured color). The Collector walks the full dependency graph, copies the
MDL + textures alongside, and rewrites the in-USD asset paths to local relative
paths — the correct way to bundle a Nucleus asset into this repo.

Discovered 2026-05-28 bundling the Vention workstation: the bench's
``Rubber_Smooth`` material referenced ``../../NVIDIA/Materials/.../Rubber_Smooth.mdl``,
which resolved on Nucleus but not from the bundled folder until collected.

PREREQUISITE
------------
A running Isaac Sim with one of this repo's extensions enabled (the MCP socket).
Default port is ur5e-dt = 8766 (soarm101-dt 8767, aic-dt 8768).

USAGE
-----
    python3 scripts/collect_nucleus_asset.py \
        --src "omniverse://localhost/Library/DT demo/vention.usd" \
        --dst /tmp/vention_collect \
        [--port 8766] [--timeout 300]

Then copy ``<dst>/<name>.usd`` and the sibling dependency folders into your
extension's ``assets/`` directory, and reference it via ``_local_asset(...)``.
"""
import argparse
import json
import socket
import time


def send(code: str, port: int, timeout: int = 120) -> dict:
    """Run a Python snippet inside the live Isaac Sim via execute_python_code.

    The snippet must assign a JSON-serialisable value to ``result``.
    """
    msg = json.dumps({"type": "execute_python_code", "params": {"code": code}})
    s = socket.socket()
    s.settimeout(timeout)
    s.connect(("localhost", port))
    s.sendall(msg.encode())
    data = b""
    while True:
        chunk = s.recv(16384)
        if not chunk:
            break
        data += chunk
        try:
            json.loads(data.decode())
            break
        except json.JSONDecodeError:
            continue
    s.close()
    resp = json.loads(data.decode())
    # The MCP socket wraps the Python `result` variable inside
    # response["result"]["result"] — unwrap to the innermost value.
    inner = resp.get("result", {})
    if isinstance(inner, dict) and "result" in inner:
        return inner["result"]
    return inner


def collect(src: str, dst: str, port: int, timeout: int) -> bool:
    # Fire-and-return: the Collector's collect() is a coroutine; we schedule it on
    # Kit's event loop and poll, rather than blocking the loop (which deadlocks).
    start = r'''
import traceback, asyncio, builtins
result = {}
try:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.kit.usd.collect")
    import omni.kit.usd.collect as C
    collector = C.Collector(usd_path=%r, collect_dir=%r, usd_only=False,
                            flat_collection=False, skip_existing=False)
    builtins._collect_done = False
    builtins._collect_err = None
    async def _go():
        try:
            await collector.collect()
        except Exception as e:
            builtins._collect_err = repr(e)
        finally:
            builtins._collect_done = True
    asyncio.ensure_future(_go())
    result = {"started": True}
except Exception:
    result = {"EXC": traceback.format_exc()}
''' % (src, dst)
    r = send(start, port).get("result", {})
    if not r.get("started"):
        print("Failed to start collector:", json.dumps(r, indent=2))
        return False

    poll = r'''
import builtins
result = {"done": getattr(builtins, "_collect_done", None),
          "err": getattr(builtins, "_collect_err", None)}
'''
    waited = 0
    interval = 5
    while waited < timeout:
        time.sleep(interval)
        waited += interval
        pr = send(poll, port).get("result", {})
        print(f"[{waited}s] done={pr.get('done')} err={pr.get('err')}")
        if pr.get("done"):
            if pr.get("err"):
                print("Collector error:", pr["err"])
                return False
            return True
    print("Timed out waiting for collect()")
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, help="Source USD URL (omniverse://, http(s)://, or local path)")
    ap.add_argument("--dst", required=True, help="Local collect directory (created if absent)")
    ap.add_argument("--port", type=int, default=8766, help="Extension MCP socket port (default 8766 = ur5e-dt)")
    ap.add_argument("--timeout", type=int, default=300, help="Max seconds to wait for collect (default 300)")
    args = ap.parse_args()

    ok = collect(args.src, args.dst, args.port, args.timeout)
    if ok:
        print(f"\nCollected into {args.dst} — copy its contents into your extension's assets/ folder.")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
