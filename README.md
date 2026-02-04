# Isaac Sim MCP Server

MCP Server for NVIDIA Isaac Sim integration, enabling AI assistants to control robotic simulations through the Model Context Protocol.

## Architecture

```
MCP Client (Claude, etc.)
    │
    └── MCP Server (isaac_mcp/server.py)
            │
            └── Socket connection (configurable port)
                    │
                    └── Isaac Sim Extension
                            │
                            └── Isaac Sim / Omniverse APIs
```

The MCP server dynamically discovers tools from the connected Isaac Sim extension. Each extension defines its own tools via `MCP_TOOL_REGISTRY`, making the extension the single source of truth.

## Requirements

- NVIDIA Isaac Sim 5.0+
- Python 3.10+
- [uv](https://github.com/astral-sh/uv) package manager

## Installation

### 1. Clone the repository

```bash
cd ~/Documents
git clone https://github.com/omni-mcp/isaac-sim-mcp
cd isaac-sim-mcp
```

### 2. Install dependencies

```bash
uv sync
```

### 3. Enable an extension in Isaac Sim

Add the extension path in Isaac Sim:
- Go to **Window > Extensions > Gear icon > Extension Search Paths**
- Add: `/path/to/isaac-sim-mcp/exts`
- Enable your desired extension

The extension should start and show:
```
[MCP] Server started on localhost:<port>
```

### 4. Configure the MCP client

Add to your MCP client config (e.g., Claude Desktop `claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "isaac-sim": {
      "command": "/path/to/isaac-sim-mcp/.venv/bin/python",
      "args": [
        "/path/to/isaac-sim-mcp/isaac_mcp/server.py"
      ],
      "env": {
        "ISAAC_SIM_PORT": "8766"
      }
    }
  }
}
```

## Port Configuration

Each extension can run on a different port. Configure via:
- **Extension**: Set `MCP_SERVER_PORT` constant in `extension.py`
- **Server**: Set `ISAAC_SIM_PORT` environment variable

To connect to multiple extensions simultaneously, create separate MCP server entries with different ports:

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

Each extension in `exts/` can expose tools via MCP. To add MCP support to an extension:

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

### 2. Add socket server startup

```python
MCP_SERVER_PORT = 8766

def on_startup(self):
    # ... existing code
    self._start_mcp_server()

def on_shutdown(self):
    self._stop_mcp_server()
    # ... existing code
```

### 3. Implement command handlers

```python
def _execute_mcp_command(self, cmd_type, params):
    if cmd_type == "list_available_tools":
        return {"status": "success", "result": {"status": "success", "tools": MCP_TOOL_REGISTRY}}

    handlers = {
        "my_tool": self._cmd_my_tool,
    }
    # ... dispatch to handler
```

### 4. Create handler methods

```python
def _cmd_my_tool(self, param1: str = None) -> Dict[str, Any]:
    try:
        # Implementation
        return {"status": "success", "message": "Done"}
    except Exception as e:
        return {"status": "error", "message": str(e)}
```

See `exts/ur5e-dt/` for a complete example.

## Available Extensions

| Extension | Port | Description |
|-----------|------|-------------|
| ur5e-dt | 8766 | UR5e Digital Twin with gripper and camera |

## License

MIT License - see LICENSE file for details.
