"""
MIT License

Copyright (c) 2023-2025 omni-mcp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# isaac_sim_mcp_server.py
#
# This server dynamically discovers tools from the Isaac Sim extension.
# The extension is the single source of truth for tool definitions.
# To add new tools, only modify the extension's MCP_TOOL_REGISTRY.

from mcp.server.fastmcp import FastMCP
import glob
import socket
import json
import logging
from dataclasses import dataclass
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any
import os

# Isaac Sim extension socket port - must match the extension's MCP_SERVER_PORT
# Can be overridden with ISAAC_SIM_PORT environment variable
ISAAC_SIM_PORT = int(os.getenv("ISAAC_SIM_PORT", "8766"))

# Output directories - use MCP_CLIENT_OUTPUT_DIR if set, otherwise use relative paths
BASE_OUTPUT_DIR = os.getenv("MCP_CLIENT_OUTPUT_DIR", "").strip()

if BASE_OUTPUT_DIR:
    BASE_OUTPUT_DIR = os.path.abspath(BASE_OUTPUT_DIR)
    RESOURCES_DIR = os.path.join(BASE_OUTPUT_DIR, "resources")
else:
    RESOURCES_DIR = "resources"

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("IsaacMCPServer")

@dataclass
class IsaacConnection:
    host: str
    port: int
    sock: socket.socket = None  # Changed from 'socket' to 'sock' to avoid naming conflict
    
    def connect(self) -> bool:
        """Connect to the Isaac addon socket server"""
        if self.sock:
            return True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            logger.info(f"Connected to Isaac at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Isaac: {str(e)}")
            self.sock = None
            return False
    
    def disconnect(self):
        """Disconnect from the Isaac addon"""
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                logger.error(f"Error disconnecting from Isaac: {str(e)}")
            finally:
                self.sock = None

    def receive_full_response(self, sock, buffer_size=16384):
        """Receive the complete response, potentially in multiple chunks"""
        chunks = []
        sock.settimeout(300.0)

        try:
            while True:
                try:
                    chunk = sock.recv(buffer_size)
                    if not chunk:
                        if not chunks:
                            raise Exception("Connection closed before receiving any data")
                        break

                    chunks.append(chunk)

                    # Check if we've received a complete JSON object
                    try:
                        data = b''.join(chunks)
                        json.loads(data.decode('utf-8'))
                        return data
                    except json.JSONDecodeError:
                        continue
                except socket.timeout:
                    logger.warning("Socket timeout during receive")
                    break
                except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
                    logger.error(f"Socket connection error: {str(e)}")
                    raise
        except socket.timeout:
            logger.warning("Socket timeout during receive")
        except Exception as e:
            logger.error(f"Error during receive: {str(e)}")
            raise

        if chunks:
            data = b''.join(chunks)
            try:
                json.loads(data.decode('utf-8'))
                return data
            except json.JSONDecodeError:
                raise Exception("Incomplete JSON response received")
        else:
            raise Exception("No data received")

    def send_command(self, command_type: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """Send a command to Isaac and return the response"""
        if not self.sock and not self.connect():
            raise ConnectionError("Not connected to Isaac")

        command = {
            "type": command_type,
            "params": params or {}
        }

        try:
            self.sock.sendall(json.dumps(command).encode('utf-8'))
            self.sock.settimeout(300.0)

            response_data = self.receive_full_response(self.sock)
            response = json.loads(response_data.decode('utf-8'))

            if response.get("status") == "error":
                raise Exception(response.get("message", "Unknown error from Isaac"))

            return response.get("result", {})
        except socket.timeout:
            self.sock = None
            raise Exception("Timeout waiting for Isaac response")
        except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
            self.sock = None
            raise Exception(f"Connection to Isaac lost: {str(e)}")
        except json.JSONDecodeError as e:
            raise Exception(f"Invalid response from Isaac: {str(e)}")
        except Exception as e:
            self.sock = None
            raise

@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    """Manage server startup and shutdown lifecycle"""
    try:
        logger.info("IsaacMCP server starting up")

        try:
            get_isaac_connection()
            discover_and_register_tools(server)
        except Exception as e:
            logger.warning(f"Could not connect to Isaac on startup: {str(e)}")
            logger.warning("Make sure the Isaac Sim extension is running before using tools")

        yield {}
    finally:
        global _isaac_connection
        if _isaac_connection:
            logger.info("Disconnecting from Isaac Sim")
            _isaac_connection.disconnect()
            _isaac_connection = None
        logger.info("IsaacMCP server shut down")

# Create the MCP server with lifespan support
mcp = FastMCP(
    "IsaacSimMCP",
    instructions="Isaac Sim integration through the Model Context Protocol. Tools are dynamically discovered from the Isaac Sim extension.",
    lifespan=server_lifespan
)

# Global connection
_isaac_connection = None


def get_isaac_connection(verify_health: bool = True):
    """Get or create a persistent Isaac connection.

    Args:
        verify_health: If True, verify the connection is alive before returning.
                      Set to False to skip health check (e.g., during initial connection).
    """
    global _isaac_connection

    if _isaac_connection is not None and _isaac_connection.sock is not None:
        if verify_health:
            logger.debug("Checking connection health...")
            # Proactively check if connection is still alive
            if not _check_connection_health(_isaac_connection):
                logger.warning("Existing connection is stale, reconnecting...")
                try:
                    _isaac_connection.disconnect()
                except:
                    pass
                _isaac_connection = None
            else:
                logger.debug("Connection health check passed")
                return _isaac_connection
        else:
            return _isaac_connection

    if _isaac_connection is None:
        logger.info("Creating new connection to Isaac...")
        _isaac_connection = IsaacConnection(host="localhost", port=ISAAC_SIM_PORT)
        if not _isaac_connection.connect():
            logger.error("Failed to connect to Isaac")
            _isaac_connection = None
            raise Exception("Could not connect to Isaac. Make sure the Isaac Sim extension is running.")

    return _isaac_connection


def _check_connection_health(conn: IsaacConnection) -> bool:
    """Check if the connection to Isaac Sim is still alive.

    Uses multiple methods to detect if the connection was closed
    by the remote end (e.g., after extension hot-reload).

    Returns:
        True if connection appears healthy, False if it's dead/stale.
    """
    if conn.sock is None:
        return False

    try:
        # Method 1: Check if socket is still connected via getpeername()
        # This will raise an exception if the socket is disconnected
        conn.sock.getpeername()
    except (OSError, socket.error):
        logger.debug("Connection health check failed: socket not connected")
        return False

    try:
        # Method 2: Use select to check if socket has error condition or is readable
        # with data (which could indicate closure)
        import select
        readable, _, errored = select.select([conn.sock], [], [conn.sock], 0)

        if errored:
            logger.debug("Connection health check failed: socket in error state")
            return False

        if readable:
            # Socket is readable - check if it's a close notification (0 bytes)
            try:
                original_timeout = conn.sock.gettimeout()
                conn.sock.setblocking(False)
                data = conn.sock.recv(1, socket.MSG_PEEK)
                conn.sock.setblocking(True)
                conn.sock.settimeout(original_timeout)

                if data == b'':
                    logger.debug("Connection health check failed: remote closed connection")
                    return False
            except BlockingIOError:
                # No data but socket is readable - unusual, but might be OK
                pass
            except (ConnectionError, OSError) as e:
                logger.debug(f"Connection health check failed: {e}")
                return False

        return True
    except Exception as e:
        logger.debug(f"Unexpected error during health check: {e}")
        return False


def discover_and_register_tools(server: FastMCP):
    """Query the extension for available tools and register them dynamically."""
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("list_available_tools", {})

        if result.get("status") != "success":
            logger.error(f"Failed to get tool list: {result.get('message', 'Unknown error')}")
            return

        tools = result.get("tools", {})
        for tool_name, tool_def in tools.items():
            register_dynamic_tool(server, tool_name, tool_def)

    except Exception as e:
        logger.error(f"Error discovering tools: {str(e)}")


def _build_tool_function(name: str, description: str, tool_params: Dict[str, Any]):
    """Dynamically build a tool function with proper signature and annotations."""
    from typing import Literal

    # Build parameter list for exec
    param_list = []
    annotations = {"return": str}

    for param_name, param_def in tool_params.items():
        param_type = param_def.get("type", "string")
        enum_values = param_def.get("enum")

        if enum_values:
            # Create Literal type for enum
            type_annotation = Literal[tuple(enum_values)]
            default = enum_values[0]
        elif param_type == "string":
            type_annotation = str
            default = ""
        elif param_type == "number":
            type_annotation = float
            default = 0.0
        elif param_type == "integer":
            type_annotation = int
            default = 0
        elif param_type == "boolean":
            type_annotation = bool
            default = False
        else:
            type_annotation = str
            default = ""

        param_list.append(f"{param_name}={repr(default)}")
        annotations[param_name] = type_annotation

    params_str = ", ".join(param_list) if param_list else ""

    # Create the function body
    func_code = f'''
def {name}({params_str}) -> str:
    """{description}"""
    return _tool_impl("{name}", locals())
'''

    # Execute to create the function
    local_ns = {"_tool_impl": _tool_implementation}
    exec(func_code, local_ns)
    func = local_ns[name]
    func.__annotations__.update(annotations)
    return func


def _is_host_call() -> bool:
    """True when the current MCP request was stamped by the client as a host
    @tool-exec / onStart / hook call (params._meta.toolExec=true; stamped in
    tool-executor.ts). Host calls are scene SETUP, not agent decisions, so the
    checkpoint-floor gate exempts them — same exemption ros-mcp-server uses.
    Fail-safe: any lookup error => treat as an agent call (stay gated)."""
    try:
        from mcp.server.lowlevel.server import request_ctx
        meta = getattr(request_ctx.get(), "meta", None)
        if meta is None:
            return False
        if isinstance(meta, dict):
            return bool(meta.get("toolExec"))
        val = getattr(meta, "toolExec", None)
        if val is None:
            val = (getattr(meta, "model_extra", None) or {}).get("toolExec")
        return bool(val)
    except Exception:
        return False


def _commit_floor_state():
    """Read the commit floor written by ros-mcp-server on each successful
    commit_object ({RESOURCES_DIR}/commit_floor.json: {phase, object_name, k,
    checkpoint, mtime}; cleared on set_task_phase / phase onStart). Returns:
      ("none", None, None)        — no floor (no commits yet) or no usable mtime
      ("ok", floor, floor_mtime)  — floor dict + effective floor mtime
      ("unreadable", err, None)   — floor exists but can't be parsed (corrupt or
                                    mid-write); callers fail CLOSED on this
    Floor mtime is the live mtime of the floor checkpoint file, falling back to
    the recorded 'mtime' field if that file is gone."""
    floor_path = os.path.join(RESOURCES_DIR, "commit_floor.json")
    if not os.path.exists(floor_path):
        return ("none", None, None)
    try:
        with open(floor_path, "r") as f:
            floor = json.load(f)
    except Exception as e:
        return ("unreadable", str(e), None)
    floor_ckpt = os.path.basename(str(floor.get("checkpoint", "")))
    try:
        floor_mtime = os.path.getmtime(
            os.path.join(RESOURCES_DIR, "scene_states", floor_ckpt))
    except OSError:
        floor_mtime = floor.get("mtime")
    if not isinstance(floor_mtime, (int, float)):
        return ("none", None, None)
    return ("ok", floor, floor_mtime)


def _checkpoint_floor_block(params: Dict[str, Any]):
    """Monotonic-floor gate for restore_scene_state (env MCP_CHECKPOINT_GATE=1).

    L1 transaction invariant: committed work is immutable. Restoring a scene
    state saved BEFORE the floor checkpoint would discard committed progress,
    so it is refused. No floor file => no commits yet => unrestricted. Returns
    a refusal string to BLOCK, None to proceed.
    """
    if os.getenv("MCP_CHECKPOINT_GATE", "").strip() != "1":
        return None
    if not BASE_OUTPUT_DIR:
        return None  # no shared run dir => nowhere a floor file could be
    if _is_host_call():
        return None

    state, floor, floor_mtime = _commit_floor_state()
    if state == "none":
        return None  # no commits yet -> unrestricted
    if state == "unreadable":
        # Fail closed: refusing a rollback is recoverable, discarding committed
        # work is not.
        return (f"Blocked: commit floor exists but is unreadable ({floor}). "
                f"Retry, or restore only checkpoints saved after the last commit.")

    scene_dir = os.path.join(RESOURCES_DIR, "scene_states")
    floor_ckpt = os.path.basename(str(floor.get("checkpoint", "")))
    requested = str(params.get("json_file_path") or "").strip()
    if requested:
        if not requested.endswith(".json"):
            requested += ".json"
        req_path = os.path.join(scene_dir, os.path.basename(requested))
    else:
        # No name => the extension restores the newest scene_state_* autosave;
        # resolve it the same way (lexicographic sort of timestamped names).
        files = sorted(glob.glob(os.path.join(scene_dir, "scene_state_*.json")))
        req_path = files[-1] if files else ""
    if not req_path or not os.path.exists(req_path):
        return None  # let the extension report its own not-found error

    if os.path.getmtime(req_path) < floor_mtime:
        req_label = os.path.splitext(os.path.basename(req_path))[0]
        obj = floor.get("object_name") or os.path.splitext(floor_ckpt)[0] or "last commit"
        k = floor.get("k")
        k_note = f" (k={k})" if k is not None else ""
        return (f"Blocked: '{req_label}' predates committed '{obj}'{k_note}. "
                f"Rollback only to '{floor_ckpt or obj + '.json'}' or later. "
                f"Committed work is immutable.")
    return None


def _known_scene_objects() -> list:
    """Object names the scene already knows, learned from saved scene states.

    Each scene_states JSON is {object_name: pose, ...}, written by the extension's
    save_scene_state from the live /World/Objects children — so the union of
    top-level keys across saved files is the scene's object list, with no extension
    round-trip and no cross-server file. Phase onStart saves 'init' right after
    add_objects, so the list is populated before any agent-named save can happen."""
    names = set()
    for path in glob.glob(os.path.join(RESOURCES_DIR, "scene_states", "*.json")):
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except Exception:
            continue  # unreadable/corrupt save contributes nothing
        if isinstance(data, dict):
            names.update(k for k in data if isinstance(k, str))
    return sorted(names)


def _checkpoint_name_block(params: Dict[str, Any]):
    """Checkpoint NAME gate for save_scene_state (env MCP_CHECKPOINT_GATE=1).

    Every save closes the current stitch window and attributes it to the save's
    name, which the windower cross-checks against the commit's object_name —
    invented names ('u_green_final', ...) and unnamed timestamped autosaves both
    junk the recipe. Allowed basenames: 'init' or an exact (case-sensitive) object
    name the scene already knows. Host (_meta.toolExec) calls are exempt, so host
    autosaves keep working. Returns a refusal string to BLOCK, None to proceed."""
    if os.getenv("MCP_CHECKPOINT_GATE", "").strip() != "1":
        return None
    if not BASE_OUTPUT_DIR:
        return None
    if _is_host_call():
        return None

    known = _known_scene_objects()
    listing = f"[{', '.join(known)}]" if known else "(none known yet — save 'init' first)"

    requested = str(params.get("json_file_path") or "").strip()
    if not requested:
        return (f"Blocked: unnamed checkpoint. "
                f"Name it 'init' or an exact object name: {listing}.")
    label = os.path.basename(requested)
    if label.endswith(".json"):
        label = label[:-len(".json")]
    if label == "init" or label in known:
        return None
    return (f"Blocked: '{label}' is not a valid checkpoint name. "
            f"Use 'init' or an exact object name: {listing}.")


def _checkpoint_overwrite_block(params: Dict[str, Any]):
    """Write-once gate for save_scene_state (env MCP_CHECKPOINT_GATE=1).

    Overwriting a checkpoint at-or-below the commit floor corrupts committed
    history AND defeats the floor gate (the overwrite bumps the file's mtime
    above the floor, so restoring the now-broken state passes the mtime
    compare). So with a floor present, a save whose target file already exists
    with mtime <= floor is refused — committed brackets (and init, which
    predates any floor by construction) are write-once. Re-saving a checkpoint
    NEWER than the floor (the in-progress object iterating) and brand-new names
    stay allowed. Returns a refusal string to BLOCK, None to proceed."""
    if os.getenv("MCP_CHECKPOINT_GATE", "").strip() != "1":
        return None
    if not BASE_OUTPUT_DIR:
        return None
    if _is_host_call():
        return None

    requested = str(params.get("json_file_path") or "").strip()
    if not requested:
        return None  # unnamed: already refused by the name gate
    fname = os.path.basename(requested)
    if not fname.endswith(".json"):
        fname += ".json"
    target = os.path.join(RESOURCES_DIR, "scene_states", fname)
    if not os.path.exists(target):
        return None  # brand-new checkpoint — never an overwrite

    state, floor, floor_mtime = _commit_floor_state()
    if state == "none":
        return None  # no commits yet -> unrestricted
    label = os.path.splitext(fname)[0]
    if state == "unreadable":
        # Fail closed, same rationale as the floor gate.
        return (f"Blocked: commit floor exists but is unreadable ({floor}); "
                f"refusing to overwrite existing checkpoint '{label}'. "
                f"Retry, or save under the in-progress object's name.")
    if os.path.getmtime(target) <= floor_mtime:
        floor_obj = floor.get("object_name") or "last commit"
        if label == "init":
            # Distinct wording: calling init "committed" would teach a literal-
            # minded model that init is restorable committed state.
            return (f"Blocked: 'init' is the phase baseline and predates the last "
                    f"commit ('{floor_obj}'). Checkpoints at or below the floor are "
                    f"immutable — save under the in-progress object's name.")
        return (f"Blocked: '{label}' is a committed checkpoint (floor: '{floor_obj}'). "
                f"Committed checkpoints are immutable — save under the in-progress "
                f"object's name, or rollback to '{floor_obj}' to retry from the "
                f"last commit.")
    return None


def _tool_implementation(name: str, params: Dict[str, Any]) -> str:
    """Shared implementation for all dynamic tools."""
    global _isaac_connection

    # Add output_dir for tools that save files
    if name in ["save_scene_state", "restore_scene_state", "start_recording"]:
        if BASE_OUTPUT_DIR:
            params["output_dir"] = BASE_OUTPUT_DIR

    # Checkpoint gates (MCP_CHECKPOINT_GATE=1): rollback must never cross the last
    # committed object; checkpoint names must be 'init' or a known object name;
    # checkpoints at-or-below the commit floor are write-once.
    if name == "restore_scene_state":
        blocked = _checkpoint_floor_block(params)
        if blocked:
            return blocked
    elif name == "save_scene_state":
        blocked = _checkpoint_name_block(params) or _checkpoint_overwrite_block(params)
        if blocked:
            return blocked

    # Retry logic: if first attempt fails with connection error, reconnect and retry once
    last_error = None
    for attempt in range(2):
        try:
            isaac = get_isaac_connection()
            result = isaac.send_command(name, params)

            if result.get("status") == "success":
                message = result.get("message", f"{name} completed successfully")
                extras = []
                if "saved_count" in result:
                    extras.append(f"Saved: {result['saved_count']} object(s)")
                if "restored_count" in result:
                    extras.append(f"Restored: {result['restored_count']} object(s)")
                if result.get("failed_names"):
                    extras.append(f"Failed: {result['failed_names']}")
                if extras:
                    message += "\n" + "\n".join(extras)
                return message
            else:
                return f"Error: {result.get('message', 'Unknown error')}"

        except Exception as e:
            last_error = e
            error_msg = str(e).lower()

            # Check if this is a connection-related error worth retrying
            is_connection_error = any(x in error_msg for x in [
                "connection", "closed", "reset", "broken pipe", "not connected",
                "timeout", "recv", "send"
            ])

            if is_connection_error and attempt == 0:
                logger.warning(f"Connection error on attempt {attempt + 1}, reconnecting: {e}")
                # Force reconnection by clearing the connection
                if _isaac_connection:
                    try:
                        _isaac_connection.disconnect()
                    except:
                        pass
                    _isaac_connection = None
                continue  # Retry
            else:
                break  # Don't retry non-connection errors or second attempt

    logger.error(f"Error in {name}: {str(last_error)}")
    return f"Error: {str(last_error)}"


# MCP _meta category tags for specific dynamically-registered tools.
_TOOL_META_CATEGORIES = {
    "save_scene_state": "checkpoint",
    "restore_scene_state": "checkpoint",
}


def register_dynamic_tool(server: FastMCP, tool_name: str, tool_def: Dict[str, Any]):
    """Register a single tool dynamically based on its definition."""
    description = tool_def.get("description", f"Execute {tool_name} on Isaac Sim")
    parameters = tool_def.get("parameters", {})

    # Build function with proper signature
    handler = _build_tool_function(tool_name, description, parameters)
    category = _TOOL_META_CATEGORIES.get(tool_name)
    if category is not None:
        server.tool(meta={"category": category})(handler)
    else:
        server.tool()(handler)


# Main execution

def main():
    """Run the MCP server"""
    mcp.run()

if __name__ == "__main__":
    main()