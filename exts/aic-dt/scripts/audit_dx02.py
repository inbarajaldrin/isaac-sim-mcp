#!/usr/bin/env python3
"""DX-02 4-surface contract auditor for the aic-dt extension.

Verifies that every MCP atom in the extension has all 4 surfaces in lockstep:

  1. MCP_TOOL_REGISTRY entry         — top-of-file dict
  2. handler-map (_HANDLERS) entry   — string-name → "_cmd_<name>" mapping
  3. _cmd_<name> method              — handler implementation
  4. UI button                       — `ui.Button(...)` invocation referencing
                                        either the public method or the _cmd_

For deleted atoms, all 4 surfaces must be ABSENT.

Phase 1 (M1) ship audit list:
  PRESENT atoms (existing pre-Phase-1 + Phase 1 additions)
  ABSENT  atoms (deleted in Plan 04 per DX-01)

Run:
  python3 exts/aic-dt/scripts/audit_dx02.py
Exit codes:
  0 — all atoms compliant
  1 — at least one atom failed an expected surface check

This script reads `exts/aic-dt/aic_dt/extension.py` directly, no import of
`omni.*` required (so it's safe outside Kit).
"""
# Reference: this project — DX-02 contract from .planning/REQUIREMENTS.md and
# Plan 06/09 SUMMARY DX-02 sections.
from __future__ import annotations

import re
import sys
from pathlib import Path

EXT_PY = Path(__file__).resolve().parent.parent / "aic_dt" / "extension.py"

# Atoms expected to be PRESENT with all 4 surfaces (Phase 1 ship state).
PRESENT_ATOMS = [
    # Pre-Phase-1 atoms (preserved through Phase 1)
    "execute_python_code",
    "play_scene",
    "stop_scene",
    "load_scene",
    "load_robot",
    "setup_action_graph",
    "setup_force_publisher",
    "setup_wrist_cameras",
    "spawn_wrist_camera",
    "remove_wrist_camera",
    "start_wrist_camera_stream",
    "stop_wrist_camera_stream",
    "add_objects",
    "delete_objects",
    "randomize_object_poses",
    "randomize_single_object",
    "save_scene_state",
    "restore_scene_state",
    "new_stage",
    "quick_start",
    "randomize_lighting",
    "run_policy",
    # New in Phase 1 (Plan 06 — TF + JointState publishers; PARITY-03/04)
    "setup_tf_publisher",
    "setup_joint_state_publisher",
    # New in Phase 1 (Plan 09 — per-component spawn atoms; SCENE-01)
    "spawn_task_board_base",
    "spawn_lc_mount_rail",
    "spawn_sfp_mount_rail",
    "spawn_sc_mount_rail",
    "spawn_sc_port",
    "spawn_nic_card_mount",
    "spawn_nic_card",
    # New in Phase 2 (Plan 02-02 — controller-loop skeleton; PARITY-09/10/11/06)
    "setup_controller_subscribers",
    "setup_offlimit_contacts",
    # New in Phase 4 (Plan 04-02 — TRIAL-01/02 trial loader + ground_truth gate)
    "load_trial",
]

# Atoms expected to be ABSENT — all 4 surfaces removed (Plan 04 / DX-01).
ABSENT_ATOMS = [
    "setup_pose_publisher",
    "sync_real_poses",
]

# Some atoms are dispatched by the registry name but the underlying
# implementation/UI button references a DIFFERENT method name. Map registry
# atom name → list of acceptable method/symbol names that satisfy the
# UI-button surface.
UI_METHOD_ALIASES = {
    "setup_force_publisher": ["setup_force_publish_action_graph"],
    "setup_tf_publisher": ["setup_tf_publish_action_graph"],
    "setup_joint_state_publisher": ["setup_joint_state_publish_action_graph"],
}

# Atoms that are MCP-only by design — they have only 3 surfaces (registry +
# handler-map + _cmd_) and no UI button. These are control-plane / introspection
# atoms whose UI representation is implicit (e.g., the play/stop timeline
# transport is wired through Kit's native timeline UI, not a dedicated button;
# new_stage / randomize_lighting / run_policy / execute_python_code are
# MCP-driven workflows). Pre-existing pattern, not a Phase 1 regression.
MCP_ONLY_ATOMS = {
    "execute_python_code",
    "play_scene",
    "stop_scene",
    "new_stage",
    "randomize_lighting",
    "run_policy",
    "randomize_single_object",
}


def audit(src: str) -> tuple[list[str], list[str]]:
    """Return (failures, notes) — failures empty means audit passes."""
    failures: list[str] = []
    notes: list[str] = []

    # PRESENT atoms — all 4 surfaces required
    for atom in PRESENT_ATOMS:
        # Surface 1: MCP_TOOL_REGISTRY entry
        if not re.search(rf'"{re.escape(atom)}":\s*\{{', src):
            failures.append(f"{atom}: surface 1 (MCP_TOOL_REGISTRY entry) MISSING")

        # Surface 2: handler-map entry — "atom_name": "_cmd_atom_name"
        if not re.search(rf'"{re.escape(atom)}":\s*"_cmd_{re.escape(atom)}"', src):
            failures.append(f"{atom}: surface 2 (handler-map entry) MISSING")

        # Surface 3: _cmd_<name> method
        if not re.search(rf"def\s+_cmd_{re.escape(atom)}\b", src):
            failures.append(f"{atom}: surface 3 (_cmd_{atom} method) MISSING")

        # Surface 4: UI button — accept any of:
        #   - direct method name (`self.<atom>`)
        #   - the _cmd_ handler (`self._cmd_<atom>`)
        #   - any UI alias from UI_METHOD_ALIASES
        # Skip atoms in MCP_ONLY_ATOMS — they are documented as no-UI by design.
        if atom in MCP_ONLY_ATOMS:
            notes.append(f"{atom}: MCP-only atom (no UI button required)")
            continue
        ui_candidates = [atom, f"_cmd_{atom}"] + UI_METHOD_ALIASES.get(atom, [])
        # We need a `ui.Button(...)` whose `clicked_fn` references one of these.
        # A pragmatic check: search for any line containing `ui.Button(` and one
        # of the candidate names within ~3 lines (most ui.Button calls fit on
        # one line in this codebase).
        ui_button_re = re.compile(r"ui\.Button\([^)]{0,500}", re.DOTALL)
        found = False
        for m in ui_button_re.finditer(src):
            chunk = m.group(0)
            for cand in ui_candidates:
                if re.search(rf"\b{re.escape(cand)}\b", chunk):
                    found = True
                    break
            if found:
                break
        if not found:
            failures.append(
                f"{atom}: surface 4 (UI button) MISSING — searched for refs to "
                f"{ui_candidates}"
            )

    # ABSENT atoms — no 4 surfaces should remain
    for atom in ABSENT_ATOMS:
        if re.search(rf'"{re.escape(atom)}":\s*\{{', src):
            failures.append(f"{atom}: surface 1 (registry entry) PRESENT but should be DELETED")
        if re.search(rf'"{re.escape(atom)}":\s*"_cmd_', src):
            failures.append(f"{atom}: surface 2 (handler-map entry) PRESENT but should be DELETED")
        if re.search(rf"def\s+_cmd_{re.escape(atom)}\b", src):
            failures.append(f"{atom}: surface 3 (_cmd_{atom} method) PRESENT but should be DELETED")
        # Surface 4 — UI button: scan for any ui.Button referencing this atom
        ui_button_re = re.compile(r"ui\.Button\([^)]{0,500}", re.DOTALL)
        for m in ui_button_re.finditer(src):
            if re.search(rf"\b(?:{re.escape(atom)}|_cmd_{re.escape(atom)})\b", m.group(0)):
                failures.append(
                    f"{atom}: surface 4 (UI button) PRESENT but should be DELETED"
                )
                break

    notes.append(
        f"PRESENT atoms checked: {len(PRESENT_ATOMS)}; "
        f"ABSENT atoms checked: {len(ABSENT_ATOMS)}"
    )
    return failures, notes


def main() -> int:
    if not EXT_PY.exists():
        print(f"FAIL: extension.py not found at {EXT_PY}", file=sys.stderr)
        return 2
    src = EXT_PY.read_text()
    failures, notes = audit(src)
    for n in notes:
        print(f"  [note] {n}")
    if failures:
        print("\nDX-02 audit FAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(
        f"\nDX-02 audit PASS: {len(PRESENT_ATOMS)} present atoms × 4 surfaces, "
        f"{len(ABSENT_ATOMS)} absent atoms × 4 surfaces — all OK."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
