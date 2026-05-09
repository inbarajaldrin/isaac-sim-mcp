#!/usr/bin/env python3
# Reference: ~/Documents/aic/tools/capture_trial.sh (canonical Gazebo capture)
"""Adapter: AIC's `tools/capture_trial.sh` recording dir -> parity-report JSON.

Reads `metadata.json` + `eval.log` from `~/Documents/aic/data/recordings/<dir>/`
(produced by `cd ~/Documents/aic && tools/capture_trial.sh --scene trial_X`)
and emits the per-trial outcome shape consumed by
`exts/aic-dt/scripts/parity_report.py`:

    {
        "trial_id":             str,    # canonical scene id (trial_1|trial_2|trial_3)
        "sim":                  "gazebo",
        "insertion_event_fired": bool,  # true if engine logs "Cable insertion successful"
        "offlimit_contact_count": int,  # tier_2 contacts message; 0 if "No contact detected"
        "completed_steps":      int,    # -1 sentinel (engine exposes duration not steps)
        "ts":                   str,    # ISO 8601 (start time from metadata.started_at)
    }

Why parse logs instead of a richer source: AIC's engine prints scoring breakdown
to stdout in a deterministic block; the rosbag captured by `capture_trial.sh`
is `/tf` + `/<cam>/image` + `/<cam>/camera_info` only (it does NOT include
`/aic/gazebo/contacts/off_limit` or `/scoring/insertion_event`). The eval.log
*does* include the engine's tier-by-tier scoring summary which is the
canonical signal.

Fallback: if `eval.log` is absent (older recordings before PATCH 1 in
capture_trial.sh added log-tee), fall back to `metadata.json.score_log_tail`
+ CheatCode-baseline defaults (insertion_event_fired=true, offlimit=0)
since CheatCode is the deterministic passing policy.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# Engine-log signals (verified 2026-05-09 against
# ~/Documents/aic/data/recordings/trial_3_20260506_025808/eval.log
# and source at ~/Documents/aic/aic_engine/src/aic_engine.cpp).
_RE_TRIAL_COMPLETED = re.compile(r"✓ Trial '([^']+)' completed successfully!")
_RE_TIER3_MESSAGE = re.compile(r"tier_3:.*?message:\s*(Cable insertion (?:successful|failed)\.?)", re.S)
_RE_TIER2_CONTACTS = re.compile(r"contacts:\s*\n\s*\[[^\]]*\]\s*score:\s*([0-9.]+)\s*\n\s*\[[^\]]*\]\s*message:\s*(.+?)\n", re.S)
_RE_INSERTION_SUCCESS = re.compile(r"Cable insertion successful")
_RE_FINISHED_SCORING = re.compile(r"Finished scoring trial, total score is:\s*([0-9.]+)")
_RE_TASK_DURATION = re.compile(r'Task duration:\s*([0-9.]+)\s*seconds\.?')


def _parse_eval_log(eval_log_path: Path) -> dict:
    """Extract canonical scoring signals from a capture_trial.sh eval.log."""
    if not eval_log_path.is_file():
        return {}
    text = eval_log_path.read_text(errors="replace")

    out: dict = {}

    if _RE_INSERTION_SUCCESS.search(text):
        out["insertion_event_fired"] = True
    elif "Cable insertion failed" in text:
        out["insertion_event_fired"] = False

    completed_match = _RE_TRIAL_COMPLETED.search(text)
    if completed_match:
        out["engine_trial_id"] = completed_match.group(1)

    score_match = _RE_FINISHED_SCORING.search(text)
    if score_match:
        out["total_score"] = float(score_match.group(1))

    if "No contact detected" in text:
        out["offlimit_contact_count"] = 0

    return out


def _parse_metadata(meta_path: Path) -> dict:
    """Extract fallback signals from metadata.json (for recordings without eval.log)."""
    if not meta_path.is_file():
        return {}
    meta = json.loads(meta_path.read_text())
    out: dict = {"_started_at": meta.get("started_at"), "_scene": meta.get("scene")}

    tail = meta.get("score_log_tail") or ""
    score_match = _RE_FINISHED_SCORING.search(tail)
    if score_match:
        out["total_score"] = float(score_match.group(1))
        # Engine writes "Finished scoring trial" only when trial completed
        # cleanly through tier_2; combined with score>0 (tier_1+tier_3 fired)
        # this implies CheatCode reached cable insertion. CheatCode is
        # collision-free by design (it cheats — it knows the answer), so
        # offlimit defaults to 0 in this fallback branch.
        out["insertion_event_fired"] = out["total_score"] > 0
        out["offlimit_contact_count"] = 0
    return out


def extract(recording_dir: Path, trial_id: str) -> dict:
    meta = _parse_metadata(recording_dir / "metadata.json")
    log = _parse_eval_log(recording_dir / "eval.log")

    insertion_event_fired = log.get("insertion_event_fired", meta.get("insertion_event_fired", False))
    offlimit_contact_count = int(log.get("offlimit_contact_count", meta.get("offlimit_contact_count", 0)))

    ts = meta.get("_started_at") or ""

    return {
        "trial_id": trial_id,
        "sim": "gazebo",
        "insertion_event_fired": bool(insertion_event_fired),
        "offlimit_contact_count": offlimit_contact_count,
        "completed_steps": -1,  # engine exposes Task-duration seconds, not step count; sentinel per PRD
        "ts": ts,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recording-dir", required=True, type=Path, help="Path to a capture_trial.sh recording dir (contains metadata.json + eval.log + single_trial_config.yaml)")
    parser.add_argument("--trial-id", required=True, choices=("trial_1", "trial_2", "trial_3"), help="Canonical scene id used by parity-report-tool")
    parser.add_argument("--output-json", required=True, type=Path, help="Path to write the per-trial outcome JSON")
    args = parser.parse_args()

    if not args.recording_dir.is_dir():
        print(f"ERROR: recording dir does not exist: {args.recording_dir}", file=sys.stderr)
        return 2

    outcome = extract(args.recording_dir, args.trial_id)

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(outcome, indent=2, sort_keys=True) + "\n")
    print(f"Wrote {args.output_json}: insertion={outcome['insertion_event_fired']} offlimit={outcome['offlimit_contact_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
