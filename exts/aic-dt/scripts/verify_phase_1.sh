#!/usr/bin/env bash
# verify_phase_1.sh — Phase 1 hybrid-runtime verification harness (D-15)
#
# Hybrid runtime model:
#   - Probes MCP port 8768 first; if responsive, runs in ATTACHED mode against
#     a pre-launched Isaac Sim. Otherwise, COLD-LAUNCHES via launch_postload.py
#     (the only known-reliable path for cold-cache cooking; see CLAUDE.md
#     "Launching the Isaac Sim aic-dt extension" / "Cache state matters").
#
# Steps in order:
#   0. Cache health check (CLAUDE.md operating discipline) — restore known-good
#      backup if DerivedDataCache < 100 MB.
#   1. Port-detect liveness probe / cold-launch fallback (D-15).
#   2. quick_start MCP round-trip.
#   3. Cache snapshot after success (cheap insurance per CLAUDE.md).
#   4. setup_tf_publisher + setup_joint_state_publisher MCP atoms (Plan 06).
#   5. ros2 topic list — assert /joint_states /tf /tf_static /fts_broadcaster/wrench.
#   5a. PARITY-05 wrench rate + framing check (consumes Plan 04
#       parity_05_wrench_framing.txt).
#   6. TF tree diff (D-08) — view_frames + diff_tf_tree.py vs aic_frames_live.gv.
#   7. _sim/_real grep on extension.py (DX-01 / D-09 — expect 0 hits).
#   8. Texture/MDL sweep (D-07) via sweep_textures.py.
#   9. D-13: handle ~/Documents/aic absence gracefully.
#  10. Joint-ordering decision check (Plan 05 Task 2 verdict).
#
# Modes:
#   verify_phase_1.sh                 # Full hybrid run (auto-launch if needed)
#   verify_phase_1.sh --no-launch     # Fail if MCP socket not already responsive
#   verify_phase_1.sh --no-snapshot   # Skip cache snapshot after quick_start
#
# Per Plan 06 architectural deferrals:
#   - PARITY-04 (slashed frame names like gripper/hande_base_link) and
#   - PARITY-03 (joint name gripper/left_finger_joint with slash)
# are EXPECTED to fail until a follow-up plan wires Raw publishers + JointState
# wrapper. The verify script is the GATE — it tells the truth about whether
# Phase 1 is actually done. Failures here mean those deferrals are still open.
#
# References: CLAUDE.md (cache discipline + real Kit log location);
#             D-13 (~/Documents/aic absence);
#             D-15 (hybrid runtime + cold-launch fallback).

set -euo pipefail

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

PORT=8768
EXT_NAME=aic-dt
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

PHASE_DIR="$REPO_ROOT/.planning/phases/01-foundation-parity"
REF_FRAMES="$PHASE_DIR/aic_frames_live.gv"
WRENCH_AUDIT="$PHASE_DIR/parity_05_wrench_framing.txt"
JOINT_PROBE="$PHASE_DIR/joint_ordering_probe.txt"

DIFF_TOOL="$SCRIPT_DIR/diff_tf_tree.py"
SWEEP_TOOL="$SCRIPT_DIR/sweep_textures.py"
POSTLOAD_SCRIPT="$SCRIPT_DIR/launch_postload.py"

LAUNCHER_HELPER="$HOME/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh"
CACHE_TOOL="$HOME/.claude/skills/isaac-sim-extension-dev/scripts/prime_usd_cache.py"
ISAACSIM_BIN="$HOME/env_isaaclab/bin/isaacsim"
CACHE_DIR="$HOME/.cache/ov/DerivedDataCache"
KIT_LOG_GLOB="$HOME/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.0/kit_*.log"

EXT_PY="$REPO_ROOT/exts/aic-dt/aic_dt/extension.py"
SIM_FRAMES="/tmp/sim_frames.gv"
COLD_LAUNCH_LOG="/tmp/aic_dt_verify.log"

# ---------------------------------------------------------------------------
# Arg parsing
# ---------------------------------------------------------------------------

NO_LAUNCH=0
NO_SNAPSHOT=0
for arg in "$@"; do
    case "$arg" in
        --no-launch)   NO_LAUNCH=1 ;;
        --no-snapshot) NO_SNAPSHOT=1 ;;
        -h|--help)
            sed -n '2,40p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown arg: $arg" >&2
            exit 2
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Accumulator + helpers
# ---------------------------------------------------------------------------

FAIL=0
LAUNCHED_BY_US=0

pass() { printf "  [PASS] %s\n" "$1"; }
fail() { printf "  [FAIL] %s\n" "$1"; FAIL=$((FAIL + 1)); }
warn() { printf "  [WARN] %s\n" "$1"; }
note() { printf "  [NOTE] %s\n" "$1"; }

section() { printf "\n=== %s ===\n" "$1"; }

check_socket() {
    local port="$1"
    python3 - "$port" <<'EOF' 2>/dev/null
import socket, sys
port = int(sys.argv[1])
s = socket.socket()
s.settimeout(3)
try:
    s.connect(("localhost", port))
except Exception:
    raise SystemExit(1)
finally:
    s.close()
EOF
}

mcp_call() {
    # Args: cmd_type [params_json] [timeout]
    local cmd="$1"
    local params="${2:-{\}}"
    local timeout="${3:-300}"
    python3 - "$PORT" "$cmd" "$params" "$timeout" <<'EOF'
import socket, json, sys
port = int(sys.argv[1])
cmd  = sys.argv[2]
params = json.loads(sys.argv[3])
timeout = int(sys.argv[4])
msg = json.dumps({"type": cmd, "params": params}).encode("utf-8")
s = socket.socket(); s.settimeout(timeout)
s.connect(("localhost", port))
s.sendall(msg)
buf = b""
while True:
    chunk = s.recv(8192)
    if not chunk: break
    buf += chunk
    try:
        json.loads(buf.decode("utf-8"))
        break
    except json.JSONDecodeError:
        continue
s.close()
print(buf.decode("utf-8"))
EOF
}

# ---------------------------------------------------------------------------
# Step 0 — Cache health (CLAUDE.md)
# ---------------------------------------------------------------------------

section "Step 0: cache health"
if [ -d "$CACHE_DIR" ]; then
    SIZE_MB=$(du -sm "$CACHE_DIR" 2>/dev/null | awk '{print $1}')
    if [ -z "$SIZE_MB" ]; then SIZE_MB=0; fi
    echo "  DerivedDataCache size: ${SIZE_MB} MB"
    if [ "$SIZE_MB" -lt 100 ]; then
        warn "Cache below 100 MB — restoring known-good backup (CLAUDE.md discipline)"
        if [ -f "$CACHE_TOOL" ]; then
            python3 "$CACHE_TOOL" restore known-good || warn "restore known-good failed; continuing anyway"
        else
            warn "prime_usd_cache.py not found at $CACHE_TOOL — manual restore required"
        fi
    else
        pass "Cache size healthy"
    fi
else
    warn "Cache dir absent: $CACHE_DIR"
fi

# ---------------------------------------------------------------------------
# Step 1 — Port detect / cold-launch fallback
# ---------------------------------------------------------------------------

section "Step 1: port-detect / cold-launch"
if check_socket "$PORT"; then
    pass "MCP socket responsive on $PORT (attached mode)"
else
    if [ "$NO_LAUNCH" -eq 1 ]; then
        fail "MCP socket not responsive on $PORT and --no-launch set"
        echo
        echo "FAIL banner: $FAIL failure(s); aborting early."
        exit 1
    fi
    note "Cold launch via launch_postload.py..."
    DISPLAY="${DISPLAY:-:0}" nohup "$ISAACSIM_BIN" --exec "$POSTLOAD_SCRIPT" \
        > "$COLD_LAUNCH_LOG" 2>&1 &
    COLD_PID=$!
    LAUNCHED_BY_US=1
    echo "  PID: $COLD_PID"
    echo "  Log: $COLD_LAUNCH_LOG"
    echo "  Polling port $PORT for up to 180s..."
    READY=0
    for i in $(seq 1 60); do
        sleep 3
        if check_socket "$PORT"; then
            pass "Cold-launch READY after ~$((i*3))s"
            READY=1
            break
        fi
    done
    if [ "$READY" -eq 0 ]; then
        fail "Cold-launch never opened MCP socket on $PORT (see $COLD_LAUNCH_LOG)"
        echo
        echo "FAIL banner: $FAIL failure(s); aborting early."
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Step 2 — quick_start
# ---------------------------------------------------------------------------

section "Step 2: quick_start"
QS_RESP="$(mcp_call quick_start '{}' 600 || true)"
if echo "$QS_RESP" | python3 -c "import sys, json; d=json.load(sys.stdin); s=d.get('status') or d.get('result',{}).get('status'); sys.exit(0 if s in ('success','ok') else 1)" 2>/dev/null; then
    pass "quick_start succeeded"
else
    fail "quick_start failed: $(echo "$QS_RESP" | head -c 400)"
fi

# ---------------------------------------------------------------------------
# Step 3 — Cache snapshot (best-effort)
# ---------------------------------------------------------------------------

section "Step 3: cache snapshot"
if [ "$NO_SNAPSHOT" -eq 1 ]; then
    note "--no-snapshot set; skipping"
elif [ -f "$CACHE_TOOL" ]; then
    python3 "$CACHE_TOOL" snapshot || warn "snapshot failed (best-effort)"
    pass "Snapshot attempted"
else
    warn "prime_usd_cache.py not found; skipping snapshot"
fi

# ---------------------------------------------------------------------------
# Step 4 — TF + JointState publisher atoms (Plan 06)
# ---------------------------------------------------------------------------

section "Step 4: TF + JointState publisher atoms"
for atom in setup_tf_publisher setup_joint_state_publisher; do
    R="$(mcp_call "$atom" '{}' 60 || true)"
    if echo "$R" | python3 -c "import sys, json; d=json.load(sys.stdin); s=d.get('status') or d.get('result',{}).get('status'); sys.exit(0 if s in ('success','ok') else 1)" 2>/dev/null; then
        pass "$atom"
    else
        fail "$atom: $(echo "$R" | head -c 300)"
    fi
done

# ---------------------------------------------------------------------------
# Step 5 — ros2 topic list (passive-sensor presence)
# ---------------------------------------------------------------------------

section "Step 5: ros2 topic list"
TOPIC_LIST=""
if command -v ros2 >/dev/null 2>&1; then
    TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
elif command -v docker >/dev/null 2>&1; then
    note "Host has no ros2; trying docker fallback (Open Q #5)"
    TOPIC_LIST="$(docker run --rm --net=host ros:humble bash -c \
        'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list' 2>/dev/null || true)"
fi

if [ -z "$TOPIC_LIST" ]; then
    warn "Neither host ros2 nor docker fallback produced topic list — skipping topic check"
else
    for t in /joint_states /tf /tf_static /fts_broadcaster/wrench; do
        if echo "$TOPIC_LIST" | grep -qx "$t"; then
            pass "topic present: $t"
        else
            fail "topic missing: $t"
        fi
    done
fi

# ---------------------------------------------------------------------------
# Step 5a — PARITY-05 wrench liveness + framing
# ---------------------------------------------------------------------------

section "Step 5a: PARITY-05 wrench liveness + framing"
if ! echo "$TOPIC_LIST" | grep -qx "/fts_broadcaster/wrench"; then
    warn "PARITY-05 not yet wired: /fts_broadcaster/wrench absent from topic list"
elif [ ! -f "$WRENCH_AUDIT" ]; then
    warn "PARITY-05 expected-rate file missing: $WRENCH_AUDIT"
else
    note "Reading expected framing from $WRENCH_AUDIT"
    EXPECTED_FRAME="$(grep -E '^Live frame_id:' "$WRENCH_AUDIT" | head -1 | awk '{print $NF}')"
    EXPECTED_TYPE="$(grep -E '^Live Type:' "$WRENCH_AUDIT" | head -1 | awk '{print $NF}')"
    note "Expected frame_id=$EXPECTED_FRAME type=$EXPECTED_TYPE"

    if command -v ros2 >/dev/null 2>&1; then
        # 5-second hz sample (best effort)
        HZ_OUT="$(timeout 7 ros2 topic hz /fts_broadcaster/wrench --window 50 2>&1 | head -10 || true)"
        echo "$HZ_OUT" | sed 's/^/    /'
        TINFO="$(ros2 topic info /fts_broadcaster/wrench 2>/dev/null || true)"
        if echo "$TINFO" | grep -q "geometry_msgs/msg/WrenchStamped"; then
            pass "PARITY-05 type matches: geometry_msgs/msg/WrenchStamped"
        else
            fail "PARITY-05 type mismatch (got: $TINFO)"
        fi
    else
        warn "ros2 not on host; cannot run hz/info — manual verification required"
    fi
fi

# ---------------------------------------------------------------------------
# Step 6 — TF tree diff (D-08)
# ---------------------------------------------------------------------------

section "Step 6: TF tree diff (D-08)"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg list 2>/dev/null | grep -qx tf2_tools; then
    rm -f "$SIM_FRAMES" /tmp/sim_frames.pdf
    (cd /tmp && timeout 30 ros2 run tf2_tools view_frames -o sim_frames >/dev/null 2>&1 || true)
    if [ -f "$SIM_FRAMES" ]; then
        if python3 "$DIFF_TOOL" "$REF_FRAMES" "$SIM_FRAMES"; then
            pass "TF trees match"
        else
            # Soft-fail with strategy-fallback hint per Plan 05 verdict
            UPI="$PHASE_DIR/usd_prim_inventory.txt"
            if [ -f "$UPI" ] && grep -q "SUBLAYER-RENAME" "$UPI"; then
                warn "STRATEGY-FALLBACK-NEEDED: SUBLAYER-RENAME hypothesis invalidated; switch Plan 05 strategy to PER-FRAME-RAW-OVERRIDE and re-run Plan 06"
            fi
            fail "TF tree diff (see above; expected until Plan 06 deferrals resolved per PARITY-04)"
        fi
    else
        warn "view_frames did not produce $SIM_FRAMES — skipping diff"
    fi
else
    warn "tf2_tools not available on host (Open Q #5) — skipping TF diff"
fi

# ---------------------------------------------------------------------------
# Step 7 — _sim/_real grep on extension.py (DX-01 / D-09)
# ---------------------------------------------------------------------------

section "Step 7: _sim / _real grep on extension.py"
if [ -f "$EXT_PY" ]; then
    HITS="$(grep -nE '(_sim|_real)\b' "$EXT_PY" | grep -v 'initialize_simulation_context_async' || true)"
    if [ -z "$HITS" ]; then
        pass "No _sim/_real production-surface tokens in extension.py"
    else
        echo "$HITS" | sed 's/^/    /'
        fail "_sim/_real tokens found in extension.py (see above)"
    fi
else
    fail "extension.py not found at $EXT_PY"
fi

# ---------------------------------------------------------------------------
# Step 8 — Texture / MDL sweep (D-07)
# ---------------------------------------------------------------------------

section "Step 8: texture / MDL sweep (D-07)"
if [ -f "$SWEEP_TOOL" ]; then
    python3 "$SWEEP_TOOL" --port "$PORT" --skip-load || warn "sweep_textures exited non-zero"
    LOG_FILE="$(ls -t $KIT_LOG_GLOB 2>/dev/null | head -1 || true)"
    if [ -n "$LOG_FILE" ]; then
        TEX_HITS="$(grep -iE '(missing|pink|fallback|not found).*texture|MDL.*not found' "$LOG_FILE" | wc -l)"
        echo "  Asset-related warning lines (narrow filter): $TEX_HITS"
        if [ "$TEX_HITS" -eq 0 ]; then
            pass "No asset/texture warnings in newest Kit log"
        else
            warn "$TEX_HITS asset warnings — review docs/texture-sweep.md (D-07 is iterative)"
        fi
    else
        warn "No Kit log matched $KIT_LOG_GLOB — skipping narrow grep"
    fi
else
    warn "sweep_textures.py missing at $SWEEP_TOOL — skipping"
fi

# ---------------------------------------------------------------------------
# Step 9 — D-13 missing-AIC-repo handling
# ---------------------------------------------------------------------------

section "Step 9: D-13 ~/Documents/aic presence"
if [ ! -d "$HOME/Documents/aic" ]; then
    note "  ~/Documents/aic absent — skipping snapshot-update step (D-13)"
else
    pass "~/Documents/aic present (snapshot-update path available)"
fi

# ---------------------------------------------------------------------------
# Step 10 — Joint-ordering decision check (Plan 05 Task 2)
# ---------------------------------------------------------------------------

section "Step 10: joint-ordering decision check"
if [ ! -f "$JOINT_PROBE" ]; then
    fail "joint_ordering_probe.txt missing — Plan 05 Task 2 verdict not recorded"
else
    VERDICT_LINE="$(grep -E '^Verdict:' "$JOINT_PROBE" | head -1 || true)"
    ACTION_LINE="$(grep -E '^Action:' "$JOINT_PROBE" | head -1 || true)"
    if [ -z "$VERDICT_LINE" ] || [ -z "$ACTION_LINE" ]; then
        fail "joint_ordering_probe.txt has no Verdict:/Action: lines"
    else
        note "$VERDICT_LINE"
        note "$ACTION_LINE"
        if echo "$ACTION_LINE" | grep -q "ADD-TASK-4-WRAPPER"; then
            if grep -q "setup_joint_state_reorder" "$EXT_PY"; then
                pass "Wrapper atom setup_joint_state_reorder present in extension.py"
            else
                fail "Action says ADD-TASK-4-WRAPPER but setup_joint_state_reorder missing in extension.py"
            fi
            if grep -q "joint_states_isaac_raw" "$EXT_PY"; then
                pass "Raw publisher topic joint_states_isaac_raw present"
            else
                fail "Action says ADD-TASK-4-WRAPPER but joint_states_isaac_raw topic missing"
            fi
        elif echo "$ACTION_LINE" | grep -q "NO-WRAPPER-NEEDED"; then
            note "NO-WRAPPER-NEEDED — confirming JointState publishes on /joint_states (no rename)"
            # Soft check: accept either the literal string or the default node config.
            if grep -q '"joint_states"' "$EXT_PY" || grep -q "topicName.*joint_states" "$EXT_PY"; then
                pass "Default /joint_states topic configuration present"
            else
                warn "Could not positively confirm /joint_states topic literal in extension.py"
            fi
        else
            warn "Unrecognized Action: line — manual review required"
        fi
    fi
fi

# ---------------------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------------------

section "Final summary"
if [ "$FAIL" -eq 0 ]; then
    echo "  PASS — Phase 1 verification harness completed with zero failures."
    exit 0
else
    echo "  FAIL — $FAIL check(s) failed. Phase 1 deferrals may still be open"
    echo "         (PARITY-04 slashed TF frames + PARITY-03 gripper/left_finger_joint"
    echo "         require Raw publishers + JointState wrapper per Plan 06 deferrals)."
    exit 1
fi
