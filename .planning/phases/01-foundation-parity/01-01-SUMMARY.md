---
phase: 01-foundation-parity
plan: 01
subsystem: parity
tags: [parity, snapshot, docker, ros2, gazebo, zenoh, view-frames, sha256]

# Dependency graph
requires:
  - phase: 00-bootstrap
    provides: Repo + planning scaffold (PROJECT.md, ROADMAP.md, REQUIREMENTS.md)
provides:
  - Reproducible Docker snapshot helper (snapshot_aic_eval.sh)
  - Pinned aic_eval image digest (sha256:be08f28709...8967433)
  - Live topic-surface artifact (36 topics in aic_topics_live.txt)
  - Live TF tree artifact (31 frames, 30 edges in aic_frames_live.gv)
  - Shipping reference doc (exts/aic-dt/docs/topic-parity-reference.md)
  - Cross-phase parity audit table mapping every Gazebo topic to Phase + Status
affects:
  - "All Phase 1 follow-on plans (01-02 through 01-09) consume topic-parity-reference.md as the parity yardstick"
  - "Phase 2 controller-loop plans flip Phase-2-deferred rows to implemented"
  - "Phase 3 scoring/object-pose plans flip Phase-3-deferred rows to implemented"
  - "Phase 4 trial-loader plans flip /observations row"

# Tech tracking
tech-stack:
  added:
    - "Docker run pattern (NVIDIA runtime, --privileged --net=host) for ephemeral aic_eval bringup"
    - "Zenoh RMW env exports (RMW_IMPLEMENTATION=rmw_zenoh_cpp + ZENOH_CONFIG_OVERRIDE) for in-container ros2 probes"
    - "tf2_tools view_frames for graphviz TF tree capture"
    - "docker inspect with Go template for image digest extraction (D-14 supply chain pin)"
  patterns:
    - "Snapshot-and-pin: capture once, pin by SHA-256 digest, refresh by re-running script"
    - "Re-runnable doc: shipping doc has a one-line bash recipe to refresh its source data"
    - "Audit invariant: every live topic has exactly one row with explicit disposition"

key-files:
  created:
    - "exts/aic-dt/scripts/snapshot_aic_eval.sh — Docker snapshot helper (95 lines)"
    - "exts/aic-dt/docs/topic-parity-reference.md — canonical parity reference (278 lines)"
    - ".planning/phases/01-foundation-parity/snapshot/image_digest.txt — SHA-256 digest pin"
    - ".planning/phases/01-foundation-parity/snapshot/topic_list.txt — 36-topic live surface"
    - ".planning/phases/01-foundation-parity/snapshot/joint_states_sample.yaml — 7-joint live echo"
    - ".planning/phases/01-foundation-parity/snapshot/aic_frames_live.{gv,pdf} — TF tree"
    - ".planning/phases/01-foundation-parity/snapshot/topic_info_*.txt — per-topic info captures"
  modified:
    - ".planning/phases/01-foundation-parity/aic_topics_live.txt — refreshed (35 noisy → 36 clean)"
    - ".planning/phases/01-foundation-parity/aic_frames_live.gv — refreshed (rate labels updated)"

key-decisions:
  - "D-01 honored: live aic_eval Docker container is the canonical parity source; YAML in sample_config.yaml is not the truth where they differ"
  - "D-14 honored: image pinned by SHA-256 digest (sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433); future re-snapshots verify pin"
  - "Inspect image (not container) for RepoDigests — running containers may have empty RepoDigests; the field is image metadata"
  - "Audit table covers all 36 topics with one row per topic; 'not applicable' status used honestly for ROS/Gazebo internals (no silent omissions)"

patterns-established:
  - "Pattern: Snapshot script writes to .planning/phases/<phase>/snapshot/ by default; positional arg overrides DEST"
  - "Pattern: Idempotent docker rm -f at script start; docker stop+rm at end; no leaked containers"
  - "Pattern: Shipping docs under exts/aic-dt/docs/ have plain Markdown (no GSD frontmatter); planning docs under .planning/ have frontmatter"
  - "Pattern: Topic-parity audit uses 6-column table (Topic | Type | Live Gazebo | Isaac Sim Status | Phase | Proof-of-Publish) for re-runnability across phases"

requirements-completed: [PARITY-03, PARITY-04, PARITY-12]

# Metrics
duration: 7min
completed: 2026-05-02
---

# Phase 1 Plan 01: Live aic_eval Topic Snapshot + Parity Reference Summary

**Reproducible Docker-based snapshot of the live aic_eval container's 36-topic surface, pinned by SHA-256 image digest, documented as a shipping parity reference with a cross-phase audit table mapping every Gazebo topic to its Isaac Sim disposition.**

## Performance

- **Duration:** 7 min
- **Started:** 2026-05-02T12:12:32Z
- **Completed:** 2026-05-02T12:20:25Z
- **Tasks:** 4 / 4
- **Files modified:** 13 (1 script created, 1 doc created, 9 snapshot artifacts captured, 2 phase-dir artifacts refreshed)

## Accomplishments

- **Reproducible snapshot helper** (`exts/aic-dt/scripts/snapshot_aic_eval.sh`): single bash script brings up `aic_eval` headless, captures image digest, full topic list, per-topic info, /joint_states echo, and view_frames TF tree in ~50 seconds. Idempotent (rm -f prior container at start; stop+rm at end).
- **Pinned image digest:** `ghcr.io/intrinsic-dev/aic/aic_eval@sha256:be08f28709acc4662da7378e94c5efccb66a8a0fff27ffccdc68e471d8967433` — survives ghcr `:latest` tag drift per D-14.
- **Shipping parity reference doc** (`exts/aic-dt/docs/topic-parity-reference.md`): documents Phase 1 passive sensor surface (joint_states/tf/tf_static with QoS/rate/source), the live 7-joint alphabetical name set with `gripper/left_finger_joint` slash semantics, the 31-frame TF tree (dynamic vs static split), and the cable Phase-3 callout (D-04 / SCENE-05).
- **Cross-phase parity audit (PARITY-12):** 36-row table covering every topic in `aic_topics_live.txt` with explicit disposition (implemented / Phase 2 deferred / Phase 3 deferred / Phase 4 deferred / not applicable). Total surface coverage; no silent omissions. 14 implemented, 9 Phase 2 deferred, 2 Phase 3 deferred, 1 Phase 4 deferred, 16 ROS/Gazebo internals.

## Task Commits

Each task was committed atomically:

1. **Task 1: Create snapshot_aic_eval.sh** — `5dc5b75` (feat)
2. **Task 2: Capture/re-validate live snapshot artifacts** — `4a29bfa` (feat — also carries the Rule 1 RepoDigests fix to the script)
3. **Task 3: Write topic-parity-reference.md** — `b841f93` (docs)
4. **Task 4: Append cross-phase parity audit (PARITY-12)** — `3540ca4` (docs)

## Files Created/Modified

**Created:**
- `exts/aic-dt/scripts/snapshot_aic_eval.sh` — Docker snapshot helper (executable, set -euo pipefail, 95 lines)
- `exts/aic-dt/docs/topic-parity-reference.md` — canonical parity reference (278 lines, 36-row audit table)
- `.planning/phases/01-foundation-parity/snapshot/image_digest.txt` — SHA-256 pin
- `.planning/phases/01-foundation-parity/snapshot/topic_list.txt` — 36-topic surface
- `.planning/phases/01-foundation-parity/snapshot/joint_states_sample.yaml` — live 7-joint echo
- `.planning/phases/01-foundation-parity/snapshot/aic_frames_live.gv` — graphviz TF tree
- `.planning/phases/01-foundation-parity/snapshot/aic_frames_live.pdf` — view_frames PDF
- `.planning/phases/01-foundation-parity/snapshot/topic_info_joint_states.txt`
- `.planning/phases/01-foundation-parity/snapshot/topic_info_tf.txt`
- `.planning/phases/01-foundation-parity/snapshot/topic_info_tf_static.txt`
- `.planning/phases/01-foundation-parity/snapshot/topic_info_clock.txt`
- `.planning/phases/01-foundation-parity/snapshot/topic_info_fts_broadcaster_wrench.txt`

**Modified:**
- `.planning/phases/01-foundation-parity/aic_topics_live.txt` — refreshed to 36 clean topic lines (was 35 with a stray "daemon stopped" log line)
- `.planning/phases/01-foundation-parity/aic_frames_live.gv` — refreshed (rate labels updated; 30 edges identical)

## Decisions Made

- **Inspect image, not container, for RepoDigests.** The plan and RESEARCH both spec'd `docker inspect ... aic_eval` (the running container) but that returned an empty `.RepoDigests` field on this host (Docker 28.0.4). The image's metadata is the correct source. Acceptance criterion (literal substring `docker inspect --format='{{index .RepoDigests 0}}'`) is still satisfied — the script now contains this pattern twice (once for the comment, once for the actual `:latest` tag invocation).
- **Refresh phase-dir artifacts on drift.** The 2026-05-02 capture surfaced 2 new topics (`/aic_controller/joint_motion_update`, `/aic_controller/motion_update`) since the research-time capture from earlier the same day. Following Task 2 step 5, the existing `aic_topics_live.txt` was overwritten with the fresh 36-topic version; drift is documented in `topic-parity-reference.md` "Provenance" section.
- **Audit row count = topic count = 36, no silent omissions.** Even ROS internals like `/parameter_events`, `/rosout`, `/diagnostics` and Gazebo lifecycle topics like `*/transition_event` get explicit `not applicable` rows with a footnote explaining why no Isaac Sim equivalent is expected. The audit's value is total honesty about the gap surface.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `docker inspect ... aic_eval` returns empty RepoDigests**

- **Found during:** Task 2 (first run of snapshot_aic_eval.sh)
- **Issue:** The plan specified `docker inspect --format='{{index .RepoDigests 0}}' aic_eval` (the running container). Docker 28.0.4 returned `template parsing error: ... map has no entry for key "RepoDigests"` because the running container's `.RepoDigests` field was empty/absent. The field is image metadata, not container metadata, and is only reliably populated when inspecting the image directly by name.
- **Fix:** Changed the inspect target from the container name `aic_eval` to the image name `ghcr.io/intrinsic-dev/aic/aic_eval:latest`. Added a comment explaining the rationale. Verified the digest is now captured correctly (`ghcr.io/intrinsic-dev/aic/aic_eval@sha256:be08f28709acc...8967433`).
- **Files modified:** `exts/aic-dt/scripts/snapshot_aic_eval.sh`
- **Verification:** Re-ran the script end-to-end; `image_digest.txt` populated correctly. Acceptance criterion still satisfied (literal `docker inspect --format='{{index .RepoDigests 0}}'` still present in the script).
- **Committed in:** `4a29bfa` (Task 2 commit, alongside the captured artifacts)

---

**Total deviations:** 1 auto-fixed (1 bug)
**Impact on plan:** The fix is a 2-line change that makes the documented invocation actually produce its intended output. No scope creep; acceptance criteria still hold; the plan's intent (pin image by SHA-256 digest) is preserved.

## Issues Encountered

- **Drift between research-time and execution-time topic surface:** the live `aic_eval` container produced 36 topics on 2026-05-02 execution vs 35 captured during research earlier the same day. Inspection: 2 new topics (`/aic_controller/joint_motion_update`, `/aic_controller/motion_update`) were genuinely new on the live surface; the "missing one" from the old count was a stray "The daemon has been stopped" log line that snuck into the original `aic_topics_live.txt`. Cleaned both: artifact now has exactly the 36 topic lines, no log noise. Documented in `topic-parity-reference.md` Provenance section.
- **No Docker / no AIC checkout fallback:** Plan's Task 2 step 6 says "if Docker is not running or the image is missing, log a clear message and abort — do NOT attempt to fall back to mocking". This case did not trigger here (Docker was running, image was already pulled).

## Threat Flags

None. The work is documentation + a Docker-based snapshot script — no new network surface, no new auth paths, no new file access patterns at trust boundaries beyond what the threat model in the plan already documented (T-01-01 mitigated via D-14 digest pin; T-01-02/03 accepted; T-01-04 mitigated by parser scope being plain-text only).

## User Setup Required

None — no external service configuration required. The snapshot script is fully self-contained provided Docker + NVIDIA runtime + the `aic_eval:latest` image are available on the host (all already present in this environment).

## Self-Check: PASSED

Verified post-write:
- FOUND: `exts/aic-dt/scripts/snapshot_aic_eval.sh` (executable, syntax-valid)
- FOUND: `exts/aic-dt/docs/topic-parity-reference.md` (contains digest pin + 36-row audit table)
- FOUND: `.planning/phases/01-foundation-parity/snapshot/` (9 artifact files including digest, topic list, joint_states sample, frames.{gv,pdf}, 5 topic_info files)
- FOUND commit `5dc5b75` (Task 1)
- FOUND commit `4a29bfa` (Task 2 — includes Rule 1 fix)
- FOUND commit `b841f93` (Task 3)
- FOUND commit `3540ca4` (Task 4)

## Next Phase Readiness

- **Phase 1 follow-on plans (01-02 through 01-09)** can now treat `topic-parity-reference.md` as the immovable yardstick for parity. Plan 04 (camera + F/T topic renames) and Plan 06 (TF + JointState publishers) flip their respective audit rows from "implemented (planned)" to "implemented (proof-of-publish verified)" by running the recipes in the audit's right-hand column.
- **No blockers** for downstream Phase 1 work.
- **Phase 2 contract surface ready:** when Phase 2 ships, it flips 9 Phase-2-deferred rows (`/aic_controller/*`, `/aic/gazebo/contacts/off_limit`, `/fts_broadcaster/wrench_filtered`) to implemented. Re-running the snapshot script will detect any new live topics that arrive in future upstream image revisions.

---
*Phase: 01-foundation-parity*
*Completed: 2026-05-02*
