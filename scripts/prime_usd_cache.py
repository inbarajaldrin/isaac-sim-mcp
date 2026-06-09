#!/usr/bin/env python3
"""prime_usd_cache.py — manage ~/.cache/ov/DerivedDataCache for Isaac Sim extensions.

Background — what this script can and cannot do:

    PhysX UJITSO + DerivedDataCache stores cooked SDF/convex/mesh collision
    shapes for the unified robot USDs used by the aic-dt / ur5e-dt / soarm101-dt
    extensions. When this cache is **populated**, `quick_start` against an
    extension completes in ~5s. When it is **empty** (or below ~50MB), every
    known launch path — `--enable <ext>`, the postload script, even bare
    `SimulationApp` + `ArticulationView` + `reset_async` — wedges at
    `load_robot/reset_async` in `futex_wait_queue`. We have not found a
    reliable cold-cache cooking path in Isaac Sim 5.x with these particular
    USDs (the issue is documented in the prior session's
    `bake_cable_fix.py` diagnostic notes and in the project CLAUDE.md's
    "Cache state matters" section).

What this script DOES reliably do:

    1. **snapshot** — take a copy of the current `DerivedDataCache` to
       `.bak.<unix_timestamp>` so you have a known-good restore point if the
       cache is later cleared/corrupted/wiped.
    2. **restore** — given a backup timestamp, swap the current cache aside
       and put the backup in place.
    3. **status** — show the size of `DerivedDataCache` and list available
       backups (sorted newest-first, with sizes).
    4. **prime <ext>** — best-effort cold-cache priming via the extension's
       postload script + `quick_start` over MCP. Will succeed if the cache is
       already warm or if cooking happens to complete; will fail loudly with
       a clear message + auto-restore prompt if it wedges. Use this AFTER
       restoring a backup if you want to refresh the snapshot, not as a
       cold-cache rescue.

Cold-cache rescue:

    If `~/.cache/ov/DerivedDataCache` is empty/small AND no `.bak.*` exists,
    you have a real problem that this script cannot fix. Options:
    - Get a `DerivedDataCache.bak.<timestamp>` from another machine that has
      a working cache and place it under `~/.cache/ov/`, then run
      `prime_usd_cache.py restore <timestamp>`.
    - Try `bash isaacsim_launch.sh launch <ext>` and wait MUCH longer than
      240s on `quick_start` — sometimes cooking eventually completes.
    - File an Isaac Sim bug.

Usage:
    prime_usd_cache.py status
    prime_usd_cache.py snapshot
    prime_usd_cache.py restore                  # restore newest backup
    prime_usd_cache.py restore 1777683559       # restore specific timestamp
    prime_usd_cache.py prime aic-dt
    prime_usd_cache.py prime aic-dt ur5e-dt soarm101-dt
    prime_usd_cache.py prime aic-dt --quick-start-timeout 600   # for slow cooks

Exit codes: 0 success, 1 failure.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

# ---- Configuration --------------------------------------------------------

def _resolve_cache_dir() -> str:
    """Runtime cache path (verified via lsof on a live Isaac process).

    The ~153 MB seed at ~/env_isaaclab/.../omni/cache/DerivedDataCache is NOT opened
    at runtime -- do not point here. Override with ISAAC_DDC_DIR if your install differs.
    """
    override = os.environ.get("ISAAC_DDC_DIR")
    if override:
        return os.path.expanduser(override)
    return os.path.expanduser("~/.cache/ov/DerivedDataCache")


CACHE_DIR = _resolve_cache_dir()
CACHE_PARENT = os.path.dirname(CACHE_DIR)
ISAACSIM_BIN = os.path.expanduser("~/env_isaaclab/bin/isaacsim")
EXTS_ROOT = os.path.expanduser("~/Documents/isaac-sim-mcp/exts")

EXT_PORT = {
    "aic-dt": 8768,
    "ur5e-dt": 8766,
    "soarm101-dt": 8767,
}


# ---- Helpers --------------------------------------------------------------

def dir_size_mb(path: str) -> float:
    if not os.path.isdir(path):
        return 0.0
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / (1024 * 1024)


def cache_health(path: str) -> tuple[str, str]:
    """OK / THIN / EMPTY by largecachedata segment count + GB, not raw MB.

    A complete robot cache spans several `largecachedata_*` segments and is multi-GB
    (known-good 5.0 ref: ~14.6 GB / 8 segments). A lone ~150 MB seed segment still
    wedges reset_async -- ">50 MB" is not a meaningful floor.
    """
    if not os.path.isdir(path):
        return "EMPTY", "directory does not exist"
    segs = sorted(f for f in os.listdir(path) if f.startswith("largecachedata_"))
    seg_mb = sum(dir_size_mb(os.path.join(path, s)) if os.path.isdir(os.path.join(path, s))
                 else os.path.getsize(os.path.join(path, s)) / (1024 * 1024)
                 for s in segs)
    detail = f"{len(segs)} largecachedata segment(s) = {seg_mb:.0f} MB cooked"
    if not segs or seg_mb < 1:
        return "EMPTY", detail
    if len(segs) < 2 or seg_mb < 400:
        return "THIN", detail + "  <- partial/seed cache (cold-cook risk)"
    return "OK", detail


def list_backups() -> list[tuple[int, str, float]]:
    """Return [(timestamp, path, size_mb), ...] sorted newest-first."""
    backups = []
    if not os.path.isdir(CACHE_PARENT):
        return backups
    for name in os.listdir(CACHE_PARENT):
        if not name.startswith("DerivedDataCache.bak."):
            continue
        path = os.path.join(CACHE_PARENT, name)
        if not os.path.isdir(path):
            continue
        # Try to extract integer timestamp from suffix
        suffix = name[len("DerivedDataCache.bak."):]
        try:
            ts = int(suffix.split(".")[0]) if suffix.split(".")[0].isdigit() else int(os.path.getmtime(path))
        except ValueError:
            ts = int(os.path.getmtime(path))
        backups.append((ts, path, dir_size_mb(path)))
    backups.sort(key=lambda t: t[0], reverse=True)
    return backups


def socket_open(port: int, timeout: float = 1.0) -> bool:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def send_mcp(port: int, cmd: str, timeout: float, params: dict | None = None) -> tuple[bool, str]:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect(("127.0.0.1", port))
        s.sendall(json.dumps({"type": cmd, "params": params or {}}).encode())
        buf = b""
        while True:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                return False, f"timeout after {timeout}s"
            if not chunk:
                break
            buf += chunk
            try:
                resp = json.loads(buf.decode())
                inner = resp.get("result", resp)
                msg = inner.get("message") if isinstance(inner, dict) else str(inner)
                ok = resp.get("status") == "success"
                return ok, msg or json.dumps(resp)
            except json.JSONDecodeError:
                continue
        return False, "connection closed mid-response"
    except OSError as e:
        return False, f"socket error: {e}"
    finally:
        s.close()


def kill_isaacsim(timeout: float = 10.0) -> None:
    for pat in ("env_isaaclab.*isaacsim", "omni.telemetry"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(socket_open(p, 0.3) for p in EXT_PORT.values()):
            return
        time.sleep(0.5)


def wait_for_socket(port: int, timeout: float = 120.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if socket_open(port, 0.5):
            return True
        time.sleep(1.0)
    return False


# ---- Subcommands ----------------------------------------------------------

def cmd_status(_args) -> int:
    verdict, detail = cache_health(CACHE_DIR)
    print(f"DerivedDataCache: {dir_size_mb(CACHE_DIR):.1f} MB ({CACHE_DIR})")
    print(f"Health: {verdict} -- {detail}")
    if verdict != "OK":
        print("  Restore a complete known-good cache before launching, e.g.:")
        print("    prime_usd_cache.py restore <ts>   (or a ~/.cache/isaacsim-recovery snapshot)")
    backups = list_backups()
    if not backups:
        print("Backups: (none)")
        return 0
    print(f"Backups (newest first):")
    for ts, path, size_mb in backups:
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))
        print(f"  {ts}  {when}  {size_mb:7.1f} MB  {os.path.basename(path)}")
    return 0


def cmd_snapshot(_args) -> int:
    size = dir_size_mb(CACHE_DIR)
    if size < 1.0:
        print(f"ERROR: refusing to snapshot empty cache ({size:.1f} MB). "
              f"Populate cache first (run quick_start successfully) or use restore.", file=sys.stderr)
        return 1
    ts = int(time.time())
    dst = f"{CACHE_DIR}.bak.{ts}"
    print(f"Snapshotting {CACHE_DIR} ({size:.1f} MB) → {dst}")
    shutil.copytree(CACHE_DIR, dst)
    print(f"OK. Snapshot size: {dir_size_mb(dst):.1f} MB")
    return 0


def cmd_restore(args) -> int:
    backups = list_backups()
    if not backups:
        print(f"ERROR: no backups found in {CACHE_PARENT}/", file=sys.stderr)
        return 1

    # Pick which backup
    if args.which is None:
        # Pick newest (largest timestamp)
        chosen = backups[0]
    else:
        # Accept either an int timestamp ('1777683559') or a name suffix ('known-good')
        # match against either the parsed timestamp or the trailing suffix in the directory name.
        match = []
        for b in backups:
            ts, path, _ = b
            suffix = os.path.basename(path)[len("DerivedDataCache.bak."):]
            if str(ts) == args.which or suffix == args.which:
                match.append(b)
        if not match:
            print(f"ERROR: no backup matching '{args.which}'. Available:", file=sys.stderr)
            for ts, path, size in backups:
                suffix = os.path.basename(path)[len("DerivedDataCache.bak."):]
                print(f"  {suffix:30s}  ({size:.1f} MB)", file=sys.stderr)
            return 1
        chosen = match[0]
    ts, src, size = chosen
    when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))

    # Make sure no Isaac Sim is using the cache
    if any(socket_open(p, 0.3) for p in EXT_PORT.values()):
        print("Killing existing Isaac Sim before restore (cache must not be in use)...")
        kill_isaacsim()

    # Move current cache aside
    if os.path.exists(CACHE_DIR):
        old_dst = f"{CACHE_DIR}.replaced.{int(time.time())}"
        print(f"Moving current cache aside → {old_dst} ({dir_size_mb(CACHE_DIR):.1f} MB)")
        shutil.move(CACHE_DIR, old_dst)

    print(f"Restoring backup {ts} ({when}, {size:.1f} MB) → {CACHE_DIR}")
    shutil.copytree(src, CACHE_DIR)
    print(f"OK. DerivedDataCache: {dir_size_mb(CACHE_DIR):.1f} MB")
    return 0


def _prime_one_ext(ext_id: str, qs_timeout: float) -> tuple[bool, str]:
    if ext_id not in EXT_PORT:
        return False, f"unknown extension '{ext_id}'. known: {list(EXT_PORT)}"
    port = EXT_PORT[ext_id]
    postload = os.path.join(EXTS_ROOT, ext_id, "scripts", "launch_postload.py")
    if not os.path.exists(postload):
        return False, f"no postload script at {postload}"
    log_path = f"/tmp/prime_usd_cache.{ext_id}.isaacsim.log"

    # Make sure no other Isaac Sim is running
    if any(socket_open(p, 0.3) for p in EXT_PORT.values()):
        print(f"  Killing existing Isaac Sim...")
        kill_isaacsim()

    # Launch postload
    print(f"  Launching: {ISAACSIM_BIN} --exec {postload}")
    print(f"  Sim log: {log_path}")
    env = os.environ.copy()
    env.setdefault("DISPLAY", ":0")
    env.setdefault("XAUTHORITY", os.path.expanduser("~/.Xauthority"))
    log_fp = open(log_path, "w")
    proc = subprocess.Popen(
        [ISAACSIM_BIN, "--exec", postload],
        env=env, stdout=log_fp, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        if not wait_for_socket(port, timeout=120):
            return False, "MCP socket never opened (Kit boot failure?)"
        print(f"  Socket up.")
        print(f"  Sending quick_start (timeout {qs_timeout:.0f}s)...")
        t0 = time.time()
        ok, msg = send_mcp(port, "quick_start", qs_timeout)
        dur = time.time() - t0
        if not ok:
            return False, f"quick_start failed after {dur:.1f}s: {msg}"
        print(f"  ✓ quick_start completed in {dur:.1f}s")
        return True, msg
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        kill_isaacsim(timeout=5)


def cmd_prime(args) -> int:
    initial = dir_size_mb(CACHE_DIR)
    print(f"DerivedDataCache before: {initial:.1f} MB")

    if initial < args.cold_threshold_mb and list_backups():
        newest = list_backups()[0]
        print(f"\nWARNING: cache is below {args.cold_threshold_mb} MB ({initial:.1f}).")
        print(f"Cold-cache priming via quick_start is unreliable — it usually wedges.")
        print(f"You have a backup available: {newest[2]:.1f} MB at timestamp {newest[0]}.")
        print(f"Recommended: prime_usd_cache.py restore {newest[0]}")
        if not args.force:
            print(f"Use --force to attempt cold-cache priming anyway.")
            return 1
        print(f"--force given; attempting cold-cache priming (likely to wedge at quick_start)...")

    failed = []
    for ext_id in args.extensions:
        print(f"\n=== Priming {ext_id} (port {EXT_PORT.get(ext_id, '?')}) ===")
        before_ext = dir_size_mb(CACHE_DIR)
        ok, msg = _prime_one_ext(ext_id, args.quick_start_timeout)
        after_ext = dir_size_mb(CACHE_DIR)
        delta = after_ext - before_ext
        print(f"  cache delta: +{delta:.1f} MB  (now {after_ext:.1f} MB)")
        if not ok:
            print(f"  ✗ FAILED: {msg}")
            failed.append((ext_id, msg))
        elif args.snapshot:
            ts = int(time.time())
            dst = f"{CACHE_DIR}.bak.{ts}"
            print(f"  Snapshotting → {dst}")
            shutil.copytree(CACHE_DIR, dst)

    print(f"\n=== summary ===")
    print(f"DerivedDataCache final: {dir_size_mb(CACHE_DIR):.1f} MB")
    if failed:
        print(f"FAILED ({len(failed)}):")
        for ext, why in failed:
            print(f"  - {ext}: {why}")
        if list_backups():
            newest = list_backups()[0]
            print(f"\nRecover with: prime_usd_cache.py restore {newest[0]}")
        return 1
    print("OK.")
    return 0


# ---- CLI ------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Manage Isaac Sim DerivedDataCache (status / snapshot / restore / prime)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See script docstring for the full background on what works and what doesn't.",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="Show current cache size and list backups")

    sub.add_parser("snapshot", help="Snapshot current cache to .bak.<unix_ts>")

    sp_restore = sub.add_parser("restore", help="Restore a backup into DerivedDataCache")
    sp_restore.add_argument("which", nargs="?", default=None,
                             help="Backup to restore — either a timestamp ('1777683559') "
                                  "or a name suffix ('known-good'). Default: newest.")

    sp_prime = sub.add_parser("prime", help="Best-effort prime via postload + quick_start (warm-cache only)")
    sp_prime.add_argument("extensions", nargs="+",
                          help="Extension id(s) to prime: " + ", ".join(EXT_PORT))
    sp_prime.add_argument("--quick-start-timeout", type=float, default=240.0,
                          help="quick_start MCP timeout in seconds (default: 240)")
    sp_prime.add_argument("--cold-threshold-mb", type=float, default=50.0,
                          help="If cache is below this and a backup exists, suggest restore (default: 50)")
    sp_prime.add_argument("--force", action="store_true",
                          help="Attempt cold-cache priming even though it usually wedges")
    sp_prime.add_argument("--snapshot", action="store_true",
                          help="Snapshot cache after each successful prime")

    return p.parse_args()


def main() -> int:
    args = parse_args()
    return {
        "status": cmd_status,
        "snapshot": cmd_snapshot,
        "restore": cmd_restore,
        "prime": cmd_prime,
    }[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
