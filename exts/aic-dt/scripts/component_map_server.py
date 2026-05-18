#!/usr/bin/env python3
"""
component_map_server.py — Custom HTTP server for the AIC component map UI.

Replaces `python3 -m http.server` with a server that ALSO:
  - Proxies /api/mcp POST → Isaac Sim MCP socket (localhost:8768)
  - Serves /api/state GET → current scene state probe
  - Serves /api/registry GET → MCP_TOOL_REGISTRY (parsed from extension.py)

This unlocks the "Live Kit UI" tab in index.html: buttons in the browser
fire MCP commands against the running Isaac Sim, just like the Kit-side
extension panel. Useful for remote control over Tailscale.

USAGE:
  python3 exts/aic-dt/scripts/component_map_server.py
  # Then open http://<this-machine-ip>:8090/index.html on any device

  # Custom port:
  python3 exts/aic-dt/scripts/component_map_server.py --port 9000

  # Bind to Tailscale only (not all interfaces):
  python3 exts/aic-dt/scripts/component_map_server.py --bind 100.97.45.92

Static files are served from the parent directory of this script's location
(i.e. exts/aic-dt/docs/component-map/), so the URL structure mirrors the
filesystem layout.
"""
from __future__ import annotations
import argparse
import json
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

SCRIPT_DIR  = Path(__file__).resolve().parent
EXT_REPO    = SCRIPT_DIR.parent
SERVE_ROOT  = EXT_REPO / "docs/component-map"
MCP_HOST    = "localhost"
MCP_PORT    = 8768
MCP_TIMEOUT = 60

# Lazily parsed (only on /api/registry call) to avoid import-time cost
_REGISTRY_CACHE: dict | None = None


def parse_mcp_registry() -> dict:
    """Parse MCP_TOOL_REGISTRY from extension.py without importing the
    extension (which would require omni.* modules). Uses ast for safety."""
    global _REGISTRY_CACHE
    if _REGISTRY_CACHE is not None:
        return _REGISTRY_CACHE
    import ast
    ext_py = EXT_REPO / "aic_dt/extension.py"
    if not ext_py.exists():
        _REGISTRY_CACHE = {}
        return _REGISTRY_CACHE
    tree = ast.parse(ext_py.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == "MCP_TOOL_REGISTRY":
                    try:
                        _REGISTRY_CACHE = ast.literal_eval(node.value)
                        return _REGISTRY_CACHE
                    except (ValueError, SyntaxError):
                        # Dict literal contained non-literal nodes (e.g. function refs)
                        # Fall back to extracting just the keys + descriptions
                        out = {}
                        if isinstance(node.value, ast.Dict):
                            for k, v in zip(node.value.keys, node.value.values):
                                if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                                    continue
                                entry: dict = {"description": "", "params": {}}
                                if isinstance(v, ast.Dict):
                                    for kk, vv in zip(v.keys, v.values):
                                        if not (isinstance(kk, ast.Constant) and isinstance(kk.value, str)):
                                            continue
                                        if kk.value == "description" and isinstance(vv, ast.Constant):
                                            entry["description"] = vv.value
                                        elif kk.value == "params":
                                            try:
                                                entry["params"] = ast.literal_eval(vv)
                                            except (ValueError, SyntaxError):
                                                entry["params"] = {}
                                out[k.value] = entry
                        _REGISTRY_CACHE = out
                        return _REGISTRY_CACHE
    _REGISTRY_CACHE = {}
    return _REGISTRY_CACHE


def mcp_call(payload: dict) -> dict:
    """Forward a JSON payload to Isaac Sim's MCP socket, return parsed response."""
    s = socket.socket()
    s.settimeout(MCP_TIMEOUT)
    try:
        s.connect((MCP_HOST, MCP_PORT))
        s.sendall(json.dumps(payload).encode())
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
        # Connection closed mid-message
        if buf:
            return {"status": "error", "message": f"partial response: {buf[:200]!r}"}
        return {"status": "error", "message": "empty response from MCP"}
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        return {"status": "error", "message": f"{type(e).__name__}: {e}"}
    finally:
        s.close()


PROBE_STATE_CODE = """
from pxr import Usd, UsdGeom
import omni.usd, omni.timeline
stage = omni.usd.get_context().get_stage()
tl = omni.timeline.get_timeline_interface()
def count_under(path):
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid(): return 0
    return sum(1 for _ in Usd.PrimRange(p))
def list_children(path):
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid(): return []
    return [{"name": c.GetName(), "type": c.GetTypeName(), "path": str(c.GetPath())}
            for c in p.GetChildren()]
result = {
    "timeline_playing": tl.is_playing() if tl else None,
    "timeline_time": tl.get_current_time() if tl else None,
    "world_children": list_children("/World"),
    "prim_count_total": sum(1 for _ in stage.Traverse()),
    "prim_count_under_world": count_under("/World"),
    "prim_count_under_ur5e": count_under("/World/UR5e"),
    "prim_count_under_taskboard": count_under("/World/TaskBoard"),
    "prim_count_under_cable": count_under("/World/UR5e/cable"),
}
"""


class Handler(SimpleHTTPRequestHandler):
    """Serves static files from SERVE_ROOT + handles /api/* endpoints."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(SERVE_ROOT), **kwargs)

    def log_message(self, fmt, *args):
        # Less noisy than default — only log API + error
        if "api/" in self.path or args and args[0].startswith(("4", "5")):
            sys.stderr.write(f"[srv] {self.client_address[0]} {self.command} {self.path} → {args[0] if args else ''}\n")

    def _json(self, code: int, body: dict):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/state":
            resp = mcp_call({"type": "execute_python_code", "params": {"code": PROBE_STATE_CODE}})
            state = (resp.get("result") or {}).get("result") if isinstance(resp.get("result"), dict) else None
            self._json(200, {"mcp_responsive": resp.get("status") == "success",
                             "state": state,
                             "raw_mcp_status": resp.get("status")})
            return
        if self.path == "/api/registry":
            self._json(200, parse_mcp_registry())
            return
        if self.path == "/api/health":
            # Confirm Sim is reachable
            try:
                s = socket.socket(); s.settimeout(2)
                s.connect((MCP_HOST, MCP_PORT))
                s.close()
                self._json(200, {"mcp_reachable": True, "port": MCP_PORT})
            except Exception as e:
                self._json(200, {"mcp_reachable": False, "error": str(e)})
            return
        # Otherwise, static file
        return super().do_GET()

    def do_POST(self):
        if self.path == "/api/mcp":
            length = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(length).decode())
            except Exception as e:
                self._json(400, {"error": f"invalid JSON: {e}"})
                return
            resp = mcp_call(body)
            self._json(200, resp)
            return
        self.send_error(404)

    def do_OPTIONS(self):
        # CORS preflight (in case the page is served from a different origin)
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__)
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="Bind address. Default 0.0.0.0 (all interfaces). Use Tailscale IP to restrict.")
    args = ap.parse_args()

    print(f"[srv] root  = {SERVE_ROOT}", file=sys.stderr)
    print(f"[srv] bind  = {args.bind}:{args.port}", file=sys.stderr)
    print(f"[srv] MCP   = {MCP_HOST}:{MCP_PORT}", file=sys.stderr)
    print(f"[srv] open: http://{args.bind if args.bind != '0.0.0.0' else 'localhost'}:{args.port}/index.html",
          file=sys.stderr)

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.allow_reuse_address = True
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[srv] shutting down", file=sys.stderr)


if __name__ == "__main__":
    main()
