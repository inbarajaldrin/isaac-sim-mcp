"""Launch Isaac Sim with aic-dt enabled POST-STARTUP, instead of via --enable.

Why: enabling aic-dt with the `--enable aic-dt` command-line flag loads the extension
during Kit's boot sequence and consistently deadlocks PhysX SDF cooking the first time
the unified robot USD is referenced (futex_wait, no cooking ever happens). This was
narrowed down by step-by-step bisection on 2026-05-01:

  - cable_only.usda alone (any sim env)                  → ✅ plays in 1s
  - unified USD via add_reference (bare SimulationApp)   → ✅ reset_async 1.4s, cooks
  - unified USD via add_reference (`isaacsim --exec`,
    "Isaac Sim Full" preset, no aic-dt)                  → ✅ reset_async 1.4s, cooks
  - All of aic-dt's startup mimicked (UI window +
    LogRedirector + MCP socket + load aic-dt via
    extension manager AFTER kit ready)                   → ✅ reset_async 0.6s, cooks
  - `isaacsim --enable aic-dt` (loads at startup)        → ❌ deadlock, no cooking

The workaround: boot Kit normally, wait for app-ready, THEN call
`extension_manager.set_extension_enabled_immediate("aic-dt", True)`. With aic-dt
loaded post-startup, PhysX cooking runs to completion and the unified USD (cable
included) loads + simulates correctly.

Usage:
  ~/env_isaaclab/bin/isaacsim --exec /path/to/launch_postload.py
"""
import time
LOG = open("/tmp/aic_dt_postload.log", "w")
def log(m):
    LOG.write(f"[{time.strftime('%H:%M:%S')}] {m}\n"); LOG.flush()

log("=== aic-dt postload bootstrap ===")

import omni.kit.app
app = omni.kit.app.get_app()
em = app.get_extension_manager()

# Discover the aic-dt extension folder. Update this path if the repo lives elsewhere.
import os
EXT_FOLDER = os.environ.get(
    "AIC_DT_EXT_FOLDER",
    "/home/aaugus11/Documents/isaac-sim-mcp/exts",
)
em.add_path(EXT_FOLDER)
log(f"added ext folder: {EXT_FOLDER}")

# Pump the app loop until Kit boot is complete (typically <60 ticks)
for _ in range(60):
    app.update()
log("Kit boot pump complete (60 ticks)")

# Neutralize omni.kit.scripting's ScriptManager. aic-dt does NOT use the OmniScriptingAPI;
# however, ScriptManager subscribes to USD ObjectsChanged via Tf.Notice and schedules
# _pending_sync_task_handler on every change. aic-dt's parity + scoring publishers modify
# USD every physics tick. On Isaac Sim 5.1 (Python 3.11 strict asyncio), the handler's
# continuous re-fire wedges Kit's main thread at 94% CPU and starves load_trial.
#
# Why monkeypatch (not set_extension_enabled_immediate(False) or destroy()):
#   - set_extension_enabled_immediate("…", False) sets is_enabled=False but leaves the
#     Tf.Notice subscription live (verified — _on_stage_attach re-fires on new_stage).
#   - destroy() unsubscribes the USD listener but does NOT clear _stage_sub, so the
#     stage_update_node re-runs _on_stage_attach on the next stage change which
#     re-registers the listener.
#   - Monkeypatching _on_usd_objects_changed and _pending_sync_task_handler to no-op
#     short-circuits both pathways at the source. Verified 2026-05-18 dual-a4500.
try:
    import omni.kit.scripting.scripts.script_manager as _smm
    _orig_on_usd = _smm.ScriptManager._on_usd_objects_changed
    def _noop_on_usd(self, *a, **kw):
        pass
    async def _noop_pending(self, *a, **kw):
        return
    _smm.ScriptManager._on_usd_objects_changed = _noop_on_usd
    _smm.ScriptManager._pending_sync_task_handler = _noop_pending
    log("omni.kit.scripting ScriptManager neutralized (USD-change + pending-sync handlers monkeypatched)")
except Exception as _e:
    log(f"WARNING: ScriptManager monkeypatch failed ({_e!r})")

# Now enable aic-dt POST-startup — this is the deadlock workaround
ok = em.set_extension_enabled_immediate("aic-dt", True)
log(f"set_extension_enabled_immediate('aic-dt', True) = {ok}")

# Pump again so aic-dt's on_startup runs to completion (UI window, MCP server)
for _ in range(60):
    app.update()
log("aic-dt on_startup complete (60 more ticks)")

# Verify MCP socket actually bound (handoff fix: the previous unconditional log lied
# when EADDRINUSE got swallowed; cost a 20+ min debug session 2026-05-18 dual-a4500).
import socket as _socket
_probe = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
try:
    _probe.settimeout(2)
    _probe.connect(("127.0.0.1", 8768))
    log("MCP server listening on port 8768. Sim is ready for normal use.")
except Exception as _e:
    log(f"WARNING: MCP server bind on 8768 NOT confirmed ({_e!r}); another instance may be holding the port.")
finally:
    _probe.close()

# Idle the app loop forever so the sim stays up
while True:
    app.update()
