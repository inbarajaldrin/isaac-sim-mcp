# Isaac Sim MCP Server

MCP Server for NVIDIA Isaac Sim integration, enabling AI assistants to control robotic simulations through the Model Context Protocol.

## Architecture

```
MCP Client (Claude, etc.)
    │  MCP protocol
    └── MCP Server (isaac_mcp/server.py — thin policy shim over kit_mcp)
            │  raw-TCP JSON socket (configurable port)
            └── omni.kit.mcp bridge  ── inside Isaac Sim ──┐
                    │  socket server + main-thread dispatch + tool registry
                    │  built-in verbs: run_python, list_tools
                    └── robot extension (ur5e-dt, …) registers its tools
                            └── Isaac Sim / Omniverse APIs
```

The generic transport — the Kit-side socket **bridge** and the LLM-facing MCP **server** — lives in the
separate [`omni-kit-mcp`](https://github.com/) package (`omni.kit.mcp` Kit extension + the `kit_mcp`
pip package). Robot extensions are thin **consumers**: each defines its tools via `MCP_TOOL_REGISTRY`
and registers them into the bridge. The bridge provides the built-in `run_python` (arbitrary Python on
Kit's main thread) and `list_tools` (discovery) verbs for free. `isaac_mcp/server.py` is a thin shim
that adds this project's policy (checkpoint gates in `isaac_mcp/gates.py`, output-dir injection) to the
generic server via hooks.

## Requirements

- NVIDIA Isaac Sim 5.0+
- Python 3.10+
- [uv](https://github.com/astral-sh/uv) package manager
- The [`omni-kit-mcp`](https://github.com/) bridge package (cloned next to this repo)

## Installation

### 1. Clone the repositories

```bash
cd ~/Documents
git clone https://github.com/omni-mcp/isaac-sim-mcp
git clone https://github.com/ <omni-kit-mcp>   # the generic bridge + MCP server
cd isaac-sim-mcp
```

### 2. Install dependencies

```bash
uv sync   # also installs the editable kit-mcp package (see [tool.uv.sources] in pyproject.toml)
```

### 3. Enable the extension in Isaac Sim

The bridge and the robot extension live in two `exts/` folders, so add **both** search paths:
- `/path/to/isaac-sim-mcp/exts`
- `/path/to/omni-kit-mcp/exts`

Enable your robot extension (e.g. `ur5e-dt`). It depends on `omni.kit.mcp`, so the bridge loads first.
On startup you should see:
```
[omni.kit.mcp] startup
[MCP] Server started on localhost:<port>
```

Or launch from the CLI with both folders:
```bash
isaacsim --ext-folder /path/to/isaac-sim-mcp/exts --ext-folder /path/to/omni-kit-mcp/exts --enable ur5e-dt
```

### 4. Configure the MCP client

```json
{
  "mcpServers": {
    "isaac-sim": {
      "command": "/path/to/isaac-sim-mcp/.venv/bin/python",
      "args": ["/path/to/isaac-sim-mcp/isaac_mcp/server.py"],
      "env": { "ISAAC_SIM_PORT": "8766" }
    }
  }
}
```

## Port Configuration

Each extension runs in its own Isaac Sim process on its own port. Configure via:
- **Consumer extension**: binds the bridge socket via `bridge.start(port)`, where the port is read from
  `OMNI_KIT_MCP_PORT` (falling back to `ISAAC_SIM_PORT`, default `8766`).
- **MCP server**: connects on the same `OMNI_KIT_MCP_PORT` / `ISAAC_SIM_PORT`.

To talk to multiple extensions (run separately, one per process), create separate MCP server entries
with different ports:

```json
{
  "mcpServers": {
    "isaac-sim-ur5e": {
      "command": "/path/to/isaac-sim-mcp/.venv/bin/python",
      "args": ["/path/to/isaac-sim-mcp/isaac_mcp/server.py"],
      "env": { "ISAAC_SIM_PORT": "8766" }
    },
    "isaac-sim-other": {
      "command": "/path/to/isaac-sim-mcp/.venv/bin/python",
      "args": ["/path/to/isaac-sim-mcp/isaac_mcp/server.py"],
      "env": { "ISAAC_SIM_PORT": "8767" }
    }
  }
}
```

## Development

### Run MCP Inspector

```bash
uv run mcp dev isaac_mcp/server.py
```

Visit http://localhost:5173 to test tools interactively.

## Creating Extensions

Each extension in `exts/` exposes tools by registering them into the `omni.kit.mcp` bridge — it does
**not** run its own socket server. To add MCP support to an extension:

### 1. Define tools in `MCP_TOOL_REGISTRY`

```python
MCP_TOOL_REGISTRY = {
    "my_tool": {
        "description": "What this tool does",
        "parameters": {
            "param1": {"type": "string", "description": "Parameter description"}
        }
    },
}
```
(`run_python` and `list_tools` are provided by the bridge — do not redefine them.)

### 2. Depend on the bridge

In `config/extension.toml`:
```toml
[dependencies]
"omni.kit.mcp" = {}
```

### 3. Register tools + start the bridge in `on_startup`

```python
from omni_kit_mcp import get_mcp_bridge, ToolDefinition

MCP_PORT = int(os.getenv("OMNI_KIT_MCP_PORT") or os.getenv("ISAAC_SIM_PORT") or "8766")

def on_startup(self, ext_id):
    # ... build your tools ...
    self._mcp = get_mcp_bridge()
    for name, schema in MCP_TOOL_REGISTRY.items():
        self._mcp.register_tool(ToolDefinition(
            name=name,
            description=schema["description"],
            parameters=schema.get("parameters", {}),
            handler=getattr(self, "_cmd_" + name),   # the _cmd_<name> convention
        ))
    self._mcp.start(MCP_PORT)

def on_shutdown(self):
    if self._mcp is not None:
        self._mcp.stop()
```

### 4. Create handler methods

```python
def _cmd_my_tool(self, param1: str = "") -> Dict[str, Any]:
    try:
        # Implementation
        return {"status": "success", "message": "Done"}
    except Exception as e:
        return {"status": "error", "message": str(e)}
```

See `exts/ur5e-dt/` for a complete example.

## Available Extensions

| Extension | Branch | Port | Description |
|-----------|--------|------|-------------|
| ur5e-dt | `main` | 8766 | UR5e Digital Twin with gripper and camera |
| soarm101-dt | `so-arm101` | 8767 | SO-ARM101 (5-DOF + gripper) |
| aic-dt | `aic` | 8768 | UR5e + RG2 + cable (AIC) |

## License

MIT License - see LICENSE file for details.
