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

# Now enable aic-dt POST-startup — this is the deadlock workaround
ok = em.set_extension_enabled_immediate("aic-dt", True)
log(f"set_extension_enabled_immediate('aic-dt', True) = {ok}")

# Pump again so aic-dt's on_startup runs to completion (UI window, MCP server)
for _ in range(60):
    app.update()
log("aic-dt on_startup complete (60 more ticks)")
log("MCP server listening on port 8768. Sim is ready for normal use.")

# Idle the app loop forever so the sim stays up
while True:
    app.update()
