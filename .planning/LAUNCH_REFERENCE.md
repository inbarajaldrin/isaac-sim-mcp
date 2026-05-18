# Isaac Sim aic-dt — launch deep reference

> **Moved from CLAUDE.md on 2026-05-17.** The reference details that don't
> change per-session (verified-working-flow recipe, env-var table, MCP
> socket-protocol code example, real Kit log path, streaming wrapper note)
> live here. The CLAUDE.md launch section now carries only the two-launcher
> reality table + the wrapped launch command + the cache discipline (the
> things a session actually needs at hand). When the recipe or env vars
> drift, update this file.

## Verified working flow (re-confirmed 2026-05-01)

```bash
# 1. Confirm cache is healthy
du -sh ~/.cache/ov/DerivedDataCache       # should be ≥100M

# 2. Launch
DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt
# → "READY (10s)"

# 3. quick_start (returns in ~5s; sim is playing afterwards with 387 prims)
python3 -c "
import socket, json
s = socket.socket(); s.settimeout(120)
s.connect(('localhost', 8768))
s.sendall(json.dumps({'type':'quick_start','params':{}}).encode())
data = b''
while True:
    c = s.recv(8192)
    if not c: break
    data += c
    try: json.loads(data.decode()); break
    except: continue
print(json.loads(data.decode())['result']['message'])
"
# Expected: "Quick start complete: AIC scene with UR5e, wrist cameras, task board objects, and simulation running."

# 4. (Iteration cycle: clear stage without restart)
python3 ~/.claude/skills/isaac-sim-extension-dev/scripts/mcp_test.py 8768 new_stage
# then re-run the quick_start block above
```

## Streaming wrapper (currently broken for aic-dt)

`~/bin/isaacsim` (interactive picker) calls `~/Documents/isaacsim-streaming/stream-local.sh --ext <name>`, which has a hardcoded `EXT_MCP_PORT` map for `soarm101-dt` (8767) and `ur5e-dt` (8766) but **does not yet include `aic-dt`** (8768). Running it for aic-dt errors with `unknown extension`. Either add `[aic-dt]=8768` to `EXT_MCP_PORT` in `stream-local.sh` or use one of the two paths above. Tracked as DX work for M1.

## Environment variables

| Var | Default | Purpose |
|---|---|---|
| `DISPLAY` | `:0` | X11 display for the Kit GUI window. Required. |
| `AIC_DT_EXT_FOLDER` | `/home/aaugus11/Documents/isaac-sim-mcp/exts` | Override hardcoded ext folder in `launch_postload.py`. Set when this repo lives elsewhere. |
| `MCP_CLIENT_OUTPUT_DIR` | unset | If set, the extension writes resources / saved scene state under `$MCP_CLIENT_OUTPUT_DIR/resources/`. Otherwise relative to `cwd`. |
| `MCP_SERVER_PORT` | `8768` (in code) | Socket port. Hardcoded as a constant in `extension.py` — change there, not via env. |
| `ROS_DOMAIN_ID` | **`7` (sim isolation)** | **MUST be set to 7 for Isaac Sim launch + smoke tests.** The user has a live UR5e ROS driver (`ur5e_with_rg2.launch.py` against `192.168.1.111`) running on the default domain 0 — its `joint_state_broadcaster` competes with Isaac Sim's `aic_dt_parity_publisher` on `/joint_states` and breaks Phase 1 smoke. Discovered 2026-05-05. Always wrap launches: `bash -c 'source ~/env_isaaclab/bin/activate && ROS_DOMAIN_ID=7 DISPLAY=${DISPLAY:-:0} bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'` and smoke tests: `ROS_DOMAIN_ID=7 python3 .../smoke_test_aic_*.py`. |

## Real Kit log (where the actual extension errors live)

```
~/.nvidia-omniverse/logs/Kit/"Isaac-Sim Full"/5.0/kit_*.log
```

The newest `kit_<timestamp>.log` is what to grep. Filter for `[MCP]`, `[py stdout]`, `Traceback`, and the `[AIC-DT]` print prefix the extension emits. **Staleness in this log = main loop blocked** (typical PhysX wedge symptom).

## MCP socket protocol

TCP, send a single JSON object, receive a single JSON object. No length prefix, no newline framing — server `json.loads` the buffer until it parses successfully.

```python
import socket, json
s = socket.socket(); s.connect(("127.0.0.1", 8768)); s.settimeout(120)
s.sendall(json.dumps({"type": "load_scene", "params": {}}).encode())
buf = b""
while True:
    chunk = s.recv(16384)
    if not chunk: break
    buf += chunk
    try: resp = json.loads(buf.decode()); break
    except json.JSONDecodeError: continue
print(resp); s.close()
```

Each MCP tool name + parameters lives in `MCP_TOOL_REGISTRY` (extension.py top). Handler bodies are `_cmd_<name>` methods on the `DigitalTwin` class.
