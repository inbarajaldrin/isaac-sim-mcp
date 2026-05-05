#!/bin/bash
# Stop hook — when the agent stops (idle / waiting for user).
# In autonomous M1 mode, this checks whether M1 is actually shipped.
# If NOT shipped and no hard blocker, emits a system reminder telling
# the agent to keep going.
#
# Active until M1 ships.

PROJECT=/home/aaugus11/Documents/isaac-sim-mcp
MARKER="$PROJECT/.planning/.autonomous_m1_active"

# Disabled if marker absent
[ -f "$MARKER" ] || exit 0

# Check if M1 actually shipped by looking for the M1 ship marker
if [ -f "$PROJECT/.planning/.m1_shipped" ]; then
  exit 0  # nothing to do
fi

# Check for explicit user-pause marker (set by user when they want to interrupt)
if [ -f "$PROJECT/.planning/.user_pause" ]; then
  exit 0
fi

# Emit JSON output telling Claude to continue (per Stop hook protocol).
# decision="block" forces a continuation; reason becomes the prompt.
cat <<'EOF'
{
  "decision": "block",
  "reason": "Autonomous M1 mode is active. M1 is not shipped (no .m1_shipped marker), no user-pause marker. Resume per HANDOFF.json next_action — fix bugs as you encounter them, do not stop. If you hit a hard blocker (needs sudo password, GPU offline, ROS topic surface mismatch you cannot fix from inside Isaac Sim), write the blocker into HANDOFF.json + create .planning/.user_pause then stop. Otherwise: continue. /gsd-resume-work then keep going."
}
EOF

exit 0
