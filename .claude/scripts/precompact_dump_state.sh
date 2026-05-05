#!/bin/bash
# PreCompact hook — runs BEFORE the conversation gets compacted.
# Dumps current state into HANDOFF.json + STATE.md so the post-compact
# agent can resume cold from CLAUDE.md → STATE.md → HANDOFF.json.
#
# Active until M1 ships. Remove this hook (or the autonomous-mode block in
# CLAUDE.md) once `aic_engine sample_config` trials all pass.

set +e  # never block compact even if dump fails
PROJECT=/home/aaugus11/Documents/isaac-sim-mcp
TS=$(date -Iseconds)

# Append a precompact marker to STATE.md so post-compact agent knows
# "this state is fresh from precompact, not stale from prior session".
if [ -f "$PROJECT/.planning/STATE.md" ]; then
  python3 - <<PY 2>/dev/null
import re, sys
p = "$PROJECT/.planning/STATE.md"
with open(p, "r") as f:
    s = f.read()
# Replace the last_updated line so resume sees the precompact timestamp
s = re.sub(r'^last_updated:.*$', f'last_updated: "$TS"', s, count=1, flags=re.M)
with open(p, "w") as f:
    f.write(s)
PY
fi

# Capture git status + uncommitted file list into HANDOFF.json snapshot
cd "$PROJECT" 2>/dev/null && {
  GIT_STATUS=$(git status --short 2>/dev/null | head -50)
  GIT_LOG=$(git log --oneline -10 2>/dev/null)
  cat > "$PROJECT/.planning/.precompact_snapshot.txt" <<EOF
=== PreCompact snapshot @ $TS ===

[git status]
$GIT_STATUS

[recent commits]
$GIT_LOG

[Isaac Sim status]
$(bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh status 2>/dev/null | head -3)
EOF
}

exit 0
