#!/usr/bin/env python3
"""Permanently register the omni.kit.mcp base bridge with Isaac Sim.

Edits the Isaac Sim app's persistent user.config.json so omni.kit.mcp auto-loads
(and self-binds its socket) on EVERY launch through the interface — no
--ext-folder / --enable flags needed. This is the scriptable equivalent of the
Extensions window's "add search path" + per-extension AUTOLOAD checkbox.

It sets three things under /persistent/app/exts (+ /persistent/exts):
  userFolders : adds omni-kit-mcp/exts so Kit can FIND the extension
  enabled     : adds the ext id so Kit AUTO-ENABLES it at startup
  autostartPort: /persistent/exts/omni.kit.mcp/autostartPort, read by the bridge's
                 on_startup so the autoloaded base BINDS its socket with the
                 built-in tools (run_python/list_tools) — zero launch flags.

Idempotent (re-running is a no-op), backs up the config first, and --remove undoes
exactly what it added. Run with the env_isaaclab python (any python3 works; only
stdlib used).

  python3 scripts/register_permanent_exts.py            # register (apply)
  python3 scripts/register_permanent_exts.py --dry-run  # show changes only
  python3 scripts/register_permanent_exts.py --remove   # undo
"""
import argparse
import glob
import json
import os
import re
import shutil
import sys
import time

HOME = os.path.expanduser("~")
DEFAULT_FOLDER = "/home/aaugus11/Documents/omni-kit-mcp/exts"
DEFAULT_EXT = "omni.kit.mcp"
# Generic base bridge port. Deliberately uncommon so this stays safe to ship as a
# package: not IANA-listed (/etc/services), not a famous dev default (avoids 9000
# php-fpm, 8080, 3000, 8888 ...), and non-privileged (>1024, so non-root can bind).
# 8766 stays reserved for the ur5e-dt twin; 9090 is rosbridge. Override via the
# setting /persistent/exts/omni.kit.mcp/autostartPort or OMNI_KIT_MCP_AUTOSTART_PORT.
DEFAULT_AUTOSTART_PORT = 9009


def find_config(explicit):
    if explicit:
        return explicit
    # Prefer the "Isaac-Sim Full" app (what normal/GUI launches use), newest version.
    cands = glob.glob(f"{HOME}/.local/share/ov/data/Kit/Isaac-Sim Full/*/user.config.json")
    if not cands:
        cands = glob.glob(f"{HOME}/.local/share/ov/data/Kit/Isaac-Sim*/*/user.config.json")
    if not cands:
        sys.exit("No Isaac Sim user.config.json found. Launch Isaac once, or pass --config.")
    return max(cands, key=os.path.getmtime)


def _idx_dict_values(d):
    """user.config serializes lists as {'0': v, '1': v}. Return the value list."""
    if not isinstance(d, dict):
        return []
    return [d[k] for k in sorted(d, key=lambda x: int(x) if x.isdigit() else x)]


def _to_idx_dict(values):
    return {str(i): v for i, v in enumerate(values)}


def _ext_version(ext_name, folders):
    """Read the extension's package version from <folder>/<ext_name>/config/extension.toml.

    Returns the version string (e.g. '0.1.0') or None. Stdlib-only line parse — the
    first standalone `version = "..."` line is the [package] version (inline dep
    version pins use a different syntax and aren't matched).
    """
    for fol in folders:
        toml = os.path.join(fol, ext_name, "config", "extension.toml")
        if os.path.isfile(toml):
            with open(toml) as f:
                for line in f:
                    m = re.match(r'\s*version\s*=\s*"([^"]+)"', line)
                    if m:
                        return m.group(1)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="Path to user.config.json (default: auto-detect Isaac-Sim Full).")
    ap.add_argument("--folder", action="append", default=None, help=f"Ext search folder(s) to register (default: {DEFAULT_FOLDER}).")
    ap.add_argument("--enable", action="append", default=None, help=f"Ext name(s) to auto-enable (default: {DEFAULT_EXT}).")
    ap.add_argument("--autostart-port", type=int, default=DEFAULT_AUTOSTART_PORT, help="Base bridge autostart port (0 to skip).")
    ap.add_argument("--remove", action="store_true", help="Undo: remove exactly what this script adds.")
    ap.add_argument("--dry-run", action="store_true", help="Print intended changes; write nothing.")
    args = ap.parse_args()

    folders = args.folder or [DEFAULT_FOLDER]
    exts = args.enable or [DEFAULT_EXT]
    cfg_path = find_config(args.config)
    print(f"config: {cfg_path}")

    with open(cfg_path) as f:
        cfg = json.load(f)

    p = cfg.setdefault("persistent", {})
    app_exts = p.setdefault("app", {}).setdefault("exts", {})
    user_folders = _idx_dict_values(app_exts.get("userFolders", {}))
    enabled = _idx_dict_values(app_exts.get("enabled", {}))

    changes = []

    if args.remove:
        for fol in folders:
            if fol in user_folders:
                user_folders.remove(fol); changes.append(f"- userFolders: {fol}")
        for e in exts:
            variants = [x for x in enabled if x == e or x.startswith(e + "-")]
            for v in variants:
                enabled.remove(v); changes.append(f"- enabled: {v}")
        # remove autostart setting
        exts_tree = p.get("exts", {})
        if "omni.kit.mcp" in exts_tree and "autostartPort" in exts_tree["omni.kit.mcp"]:
            del exts_tree["omni.kit.mcp"]["autostartPort"]
            if not exts_tree["omni.kit.mcp"]:
                del exts_tree["omni.kit.mcp"]
            changes.append("- /persistent/exts/omni.kit.mcp/autostartPort")
    else:
        for fol in folders:
            if not os.path.isdir(fol):
                print(f"  WARN: folder does not exist: {fol}")
            if fol not in user_folders:
                user_folders.append(fol); changes.append(f"+ userFolders: {fol}")
        for e in exts:
            # Use the EXACT versioned ext id (e.g. "omni.kit.mcp-0.1.0") so Kit gets a
            # precise match — a bare name makes it log "User selected ext ... no longer
            # available, found <id>". Version is read from the ext's own toml, so a
            # re-run after a version bump self-corrects.
            ver = _ext_version(e, folders)
            ext_id = f"{e}-{ver}" if ver else e
            # Collapse all variants (bare + any other versions) down to this one id.
            variants = [x for x in enabled if x == e or x.startswith(e + "-")]
            if variants != [ext_id]:
                enabled = [x for x in enabled if x not in variants]
                enabled.append(ext_id)
                removed = [v for v in variants if v != ext_id]
                changes.append(f"+ enabled: {ext_id}" + (f" (removed stale: {removed})" if removed else ""))
        if args.autostart_port:
            exts_tree = p.setdefault("exts", {}).setdefault("omni.kit.mcp", {})
            if exts_tree.get("autostartPort") != args.autostart_port:
                exts_tree["autostartPort"] = args.autostart_port
                changes.append(f"+ /persistent/exts/omni.kit.mcp/autostartPort = {args.autostart_port}")

    app_exts["userFolders"] = _to_idx_dict(user_folders)
    app_exts["enabled"] = _to_idx_dict(enabled)

    if not changes:
        print("Already in desired state — nothing to do.")
        return
    print("Changes:")
    for c in changes:
        print("  " + c)

    if args.dry_run:
        print("(dry-run — wrote nothing)")
        return

    backup = f"{cfg_path}.bak.{int(time.time())}"
    shutil.copy2(cfg_path, backup)
    print(f"backup: {backup}")
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=4)
    print("WROTE config. Restart Isaac Sim (via the interface) — omni.kit.mcp will load + bind automatically.")


if __name__ == "__main__":
    main()
