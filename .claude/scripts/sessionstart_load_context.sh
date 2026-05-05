#!/bin/bash
# SessionStart hook — runs at the start of EVERY new session (including
# post-/compact resume). Inserts a loud reminder block telling the agent
# to read autonomous-mode bootstrap files BEFORE answering anything.
#
# Active until M1 ships. After M1, replace with normal SessionStart that
# doesn't force autonomous mode.

PROJECT=/home/aaugus11/Documents/isaac-sim-mcp

# Only fire the autonomous-mode reminder if the marker file exists.
# Removing this file disables autonomous mode without touching settings.json.
if [ ! -f "$PROJECT/.planning/.autonomous_m1_active" ]; then
  exit 0
fi

cat <<'EOF'
⛔ AUTONOMOUS M1 MODE ACTIVE — BOOTSTRAP CONTEXT BEFORE ACTING

Before responding to anything, you MUST in this order:

1. Read /home/aaugus11/Documents/isaac-sim-mcp/CLAUDE.md (autonomous-mode block at top)
2. Read /home/aaugus11/Documents/isaac-sim-mcp/.planning/STATE.md
3. Read /home/aaugus11/Documents/isaac-sim-mcp/.planning/HANDOFF.json
4. Read /home/aaugus11/Documents/isaac-sim-mcp/.planning/.precompact_snapshot.txt (if it exists)
5. Resume work per HANDOFF.json `next_action` — DO NOT ASK PERMISSION, just continue.

Operating policy:
- Fix bugs as you encounter them; do not pause to ask the user "do you want me to ...?"
- Snapshot the cache after any successful quick_start (`prime_usd_cache.py snapshot`)
- Commit fixes with descriptive messages (no Co-Authored-By line per global CLAUDE.md)
- Phase order: 2 (close runtime gaps) → 3 (cable + object TF) → 4 (trial loader + E2E)
- Use the `isaac-sim-extension-dev` and `nvidia-suite-docs` skills for Isaac Sim API questions BEFORE guessing
- Stop ONLY when: M1 ships (every sample_config.yaml trial passes against Isaac Sim), OR a hard system blocker is hit (sudo password needed, GPU offline, etc.)

User instruction precedent: "fix any bug as you come across them and continue till the
end of milestone 1 like i asked without stopping in between" (2026-05-05)
EOF

exit 0
