"""Checkpoint-gate policy for the mode2 ablation pipeline.

This is PROJECT POLICY, extracted out of the otherwise-generic MCP server
(`server.py`) so the server can stay a thin, reusable shell. The server calls
exactly two hooks here — `check_tool_call(name, params)` before dispatch, and
`tool_meta_category(name)` at registration — and knows nothing else about
checkpoints.

The gates enforce the L1 transaction invariant for `save_scene_state` /
`restore_scene_state` when `MCP_CHECKPOINT_GATE=1`:
  - rollback must never cross the last committed object (floor gate),
  - checkpoint names must be 'init' or a known object name (name gate),
  - checkpoints at-or-below the commit floor are write-once (overwrite gate).

They read the ablation pipeline's files under RESOURCES_DIR (scene_states/ and
commit_floor.json), so this module is bound to the run layout — it stays in this
repo when the generic server.py moves to the bridge repo.
"""

import glob
import json
import os
from typing import Any, Dict

# Output dir config. MCP_CLIENT_OUTPUT_DIR is the single source of truth (the
# server reads the same env var). RESOURCES_DIR is where the ablation pipeline
# writes scene_states/ and commit_floor.json that these gates inspect.
BASE_OUTPUT_DIR = os.getenv("MCP_CLIENT_OUTPUT_DIR", "").strip()
if BASE_OUTPUT_DIR:
    BASE_OUTPUT_DIR = os.path.abspath(BASE_OUTPUT_DIR)
    RESOURCES_DIR = os.path.join(BASE_OUTPUT_DIR, "resources")
else:
    RESOURCES_DIR = "resources"


# MCP _meta category tags for specific dynamically-registered tools.
_TOOL_META_CATEGORIES = {
    "save_scene_state": "checkpoint",
    "restore_scene_state": "checkpoint",
}


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


# ==================== Public hooks called by the generic server ====================

def check_tool_call(name: str, params: Dict[str, Any]):
    """Pre-dispatch policy gate. Returns a refusal string to BLOCK, or None to proceed.

    The server calls this for every tool; only save/restore_scene_state are gated.
    """
    if name == "restore_scene_state":
        return _checkpoint_floor_block(params)
    if name == "save_scene_state":
        return _checkpoint_name_block(params) or _checkpoint_overwrite_block(params)
    return None


def tool_meta_category(name: str):
    """MCP _meta category for a tool name (e.g. 'checkpoint'), or None."""
    return _TOOL_META_CATEGORIES.get(name)
