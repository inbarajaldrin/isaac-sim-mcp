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

# isaac_mcp/server.py
#
# Thin POLICY shim over the generic `kit_mcp` server (omni-kit-mcp repo).
#
# The transport client + FastMCP discovery/dispatch are now generic and live in
# `kit_mcp.server`. This module keeps the isaac-sim-mcp entry points working
# (`isaac_mcp.server:main`, `from . import server`, `mcp dev isaac_mcp/server.py`)
# and plugs in THIS project's policy via the generic server's hooks:
#   1. output_dir injection for file-writing tools, and
#   2. the checkpoint gates (gates.py).
#
# Requires the generic package: `pip install -e ~/Documents/omni-kit-mcp`.

import os

# Preserve the original client-visible MCP server identity (kit_mcp defaults to
# "KitMCP"). Must be set BEFORE importing kit_mcp.server, which constructs the
# FastMCP instance at import time. setdefault => an explicit env override wins.
os.environ.setdefault("KIT_MCP_NAME", "IsaacSimMCP")
os.environ.setdefault(
    "KIT_MCP_INSTRUCTIONS",
    "Isaac Sim integration through the Model Context Protocol. "
    "Tools are dynamically discovered from the Isaac Sim extension.",
)

# Map this project's port env onto the bridge's standard knob, so existing
# ISAAC_SIM_PORT configs keep working without the generic kit_mcp package
# referencing anything isaac-specific.
if os.getenv("ISAAC_SIM_PORT") and not os.getenv("OMNI_KIT_MCP_PORT"):
    os.environ["OMNI_KIT_MCP_PORT"] = os.environ["ISAAC_SIM_PORT"]

try:
    from kit_mcp import server as kit_server
except ModuleNotFoundError as e:  # pragma: no cover - setup guard
    raise ModuleNotFoundError(
        "kit_mcp not found — install the generic Kit MCP server/bridge package:\n"
        "    pip install -e ~/Documents/omni-kit-mcp\n"
        "(isaac_mcp.server is now a thin policy shim over kit_mcp.server.)"
    ) from e

# Checkpoint-gate policy. Robust to package import and script execution.
try:
    from . import gates
except ImportError:
    import gates

# Re-export the FastMCP instance + entrypoint so existing launch configs keep working.
mcp = kit_server.mcp

# --- Policy 1: output_dir injection (project-specific) -----------------------
# File-writing tools get the run's output dir injected into their params.
BASE_OUTPUT_DIR = os.getenv("MCP_CLIENT_OUTPUT_DIR", "").strip()
if BASE_OUTPUT_DIR:
    BASE_OUTPUT_DIR = os.path.abspath(BASE_OUTPUT_DIR)


def _inject_output_dir(name, params):
    """Inject the run's output dir for tools that save files. Never blocks."""
    if BASE_OUTPUT_DIR and name in ("save_scene_state", "restore_scene_state", "start_recording"):
        params["output_dir"] = BASE_OUTPUT_DIR
    return None


# Register policy hooks once. The guard prevents duplicate registration if this
# module is re-imported/reloaded in the same interpreter (hook lists are global
# on kit_server). Order matches the original inline logic: inject output_dir
# first, then run the checkpoint gates.
if not getattr(kit_server, "_isaac_policy_registered", False):
    kit_server.register_pre_dispatch_hook(_inject_output_dir)
    kit_server.register_pre_dispatch_hook(gates.check_tool_call)
    kit_server.register_meta_category_hook(gates.tool_meta_category)
    kit_server._isaac_policy_registered = True


def main():
    """Run the MCP server (with this project's policy hooks registered)."""
    kit_server.main()


if __name__ == "__main__":
    main()
