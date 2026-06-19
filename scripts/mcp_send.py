#!/usr/bin/env python3
"""mcp_send.py — minimal socket driver for a running ur5e-dt Isaac Sim MCP extension.

The ur5e-dt extension listens on TCP 8766 and accepts JSON `{"type": <cmd>, "params": {...}}`.
This is the lightweight way to drive the live sim from the shell (quick_start, add_objects,
assemble_objects, setup_pose_publisher, start_ros_driver, execute_python_code, ...).

Usage:
    python3 scripts/mcp_send.py quick_start
    python3 scripts/mcp_send.py add_objects '{"assembly":"fmb2"}'
    python3 scripts/mcp_send.py assemble_objects '{"assembly":"fmb2"}'
    python3 scripts/mcp_send.py start_ros_driver '{"action":"up","backend":"fake"}'
    python3 scripts/mcp_send.py execute_python_code '{"code":"result=1+1"}'
    python3 scripts/mcp_send.py list_available_tools

Long ops (add_objects SDF cook, start_ros_driver) can take a while; the socket timeout is 300s.
Run in the background if you expect >~60s and poll, since the call blocks until the handler returns.
"""
import socket, json, sys


def send(cmd, params=None, port=8766, timeout=300):
    s = socket.socket(); s.settimeout(timeout); s.connect(("localhost", port))
    s.sendall(json.dumps({"type": cmd, "params": params or {}}).encode())
    d = b""
    while True:
        c = s.recv(16384)
        if not c:
            break
        d += c
        try:
            json.loads(d.decode()); break
        except Exception:
            continue
    s.close()
    return json.loads(d.decode())


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    cmd = sys.argv[1]
    params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    print(json.dumps(send(cmd, params), indent=2)[:3000])
