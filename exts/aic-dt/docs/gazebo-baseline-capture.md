# Gazebo Baseline Capture (M1 reference outcomes)

This doc explains how the Gazebo baseline outcomes in
`exts/aic-dt/docs/trial_outcomes/trial_<id>__gazebo.json` are produced. They
are the M1 reference outcomes that Isaac Sim trial outcomes are compared
against by `exts/aic-dt/scripts/parity_report.py`.

## Source-of-truth pipeline

AIC ships a per-trial Gazebo capture script:
`~/Documents/aic/tools/capture_trial.sh`. It is the canonical mechanism — we
adapt its outputs rather than reimplement.

`capture_trial.sh --scene trial_X` (where `X ∈ {1, 2, 3}`):

1. Generates a 1-trial config from `aic_engine/config/sample_config.yaml`.
2. Launches the **`aic_eval`** container (Gazebo + `aic_engine` +
   `aic_controller` + `ros_gz_bridge` + scoring) headless with
   `ground_truth:=true start_aic_engine:=true`.
3. Launches the **`aic_model`** container (`my-solution:v1`,
   `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, policy
   `aic_example_policies.ros.CheatCode`).
4. Tees the eval container log to `eval.log` (PATCH 1 in capture_trial.sh).
5. Records `/tf` + camera topics into `trial_bag/` via a third container.
6. Watches for engine-completion patterns
   (`All Trials Processed | Engine Stopped with Errors |
   process has finished cleanly.*aic_engine`).
7. Writes `metadata.json` with `started_at` / `ended_at` / `score_log_tail`.

Output dir: `~/Documents/aic/data/recordings/trial_<id>_<YYYYMMDD_HHMMSS>/`.
Contents: `metadata.json`, `eval.log` (when present), `single_trial_config.yaml`,
`tcp_object_offsets.yaml`, `trial_bag/`.

## Adapter — recording dir → parity-report JSON

`exts/aic-dt/scripts/extract_gazebo_baseline.py` reads the recording dir and
emits the per-trial outcome shape consumed by `parity_report.py`:

```json
{
  "trial_id":              "trial_1|trial_2|trial_3",
  "sim":                   "gazebo",
  "insertion_event_fired": true,
  "offlimit_contact_count": 0,
  "completed_steps":       -1,
  "ts":                    "2026-05-06T06:48:18Z"
}
```

Parsing strategy:

| Field | Source signal |
|-------|--------------|
| `insertion_event_fired` | `eval.log` regex `Cable insertion successful` (engine `tier_3` message). Fallback when `eval.log` absent: `metadata.score_log_tail` shows `Finished scoring trial, total score is: <num>` with `<num> > 0` (`tier_1_success` implies model reached cable insertion under CheatCode). |
| `offlimit_contact_count` | `eval.log` `tier_2.contacts.message: No contact detected.` → 0. CheatCode is collision-free by design (it knows the answer); the M1 reference baseline is therefore `0` for all 3 trials. Fallback when `eval.log` absent: `0` (CheatCode invariant). |
| `completed_steps` | `-1` sentinel — engine exposes `Task duration: <s>` seconds, not a discrete step count. Per PRD task spec, sentinel is acceptable. |
| `ts` | `metadata.started_at` (ISO 8601 UTC of capture start). |

The fallback branch only activates for older recordings predating the
`eval.log` tee in `capture_trial.sh` (PATCH 1). All current recordings will
have `eval.log`.

## Wrapper — sister of the Isaac Sim wrapper

`exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh` is the single command
that ties capture + extraction together. It mirrors the `--output-json` flag
of `run_aic_engine_against_isaac_sim.sh` so both paths feed
`parity_report.py` with the same shape.

```bash
# Fresh capture + extract (takes ~3-5 min per trial in Docker):
bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_1
bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_2
bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_3

# Re-extract against the most-recent recording without re-running Docker:
bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_1 --reuse-latest

# Custom output path:
bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh trial_1 \
  --output-json /tmp/some_path.json
```

Default output path: `exts/aic-dt/docs/trial_outcomes/<trial_id>__gazebo.json`.

## Regenerating the baselines from scratch

```bash
# Pre-flight: sim isolation — make sure Isaac Sim isn't fighting Gazebo for GPU.
# (Optional. Hardware caveat is acknowledged in the PRD scope_note; a 100%
# clean re-run shuts Isaac Sim down first via the lifecycle helper.)
bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh kill

# Per-trial capture + extraction (deterministic; CheatCode is a passing policy).
for t in trial_1 trial_2 trial_3; do
  bash exts/aic-dt/scripts/run_aic_engine_against_gazebo.sh "$t"
done

# Verify shape:
for t in trial_1 trial_2 trial_3; do
  python3 -c "import json; print(json.load(open('exts/aic-dt/docs/trial_outcomes/${t}__gazebo.json')))"
done
```

Expected outcome for every trial: `insertion_event_fired=True`,
`offlimit_contact_count=0`. These are the M1 reference outcomes that the
parity-report compares Isaac Sim's trial fires against.

## Provenance — current baselines

The committed `trial_outcomes/trial_<id>__gazebo.json` files were extracted
from these existing recordings:

| Trial | Recording dir | Score | eval.log present |
|-------|---------------|-------|-------------------|
| trial_1 | `~/Documents/aic/data/recordings/trial_1_20260505_234730/` | 92.61 | no (predates PATCH 1) |
| trial_2 | `~/Documents/aic/data/recordings/trial_2_20260505_234937/` | 92.58 | no (predates PATCH 1) |
| trial_3 | `~/Documents/aic/data/recordings/trial_3_20260506_025808/` | 91.95 | yes |

For trial_1 + trial_2, the score-from-metadata fallback applies; the
CheatCode-collision-free invariant gives `offlimit=0`. For trial_3, eval.log
parsing confirmed `tier_3 message: Cable insertion successful` and
`tier_2 contacts: No contact detected`. All three converge to the same
verdict (`insertion=True, offlimit=0`).

## Cross-repo touch policy

This pipeline INVOKES `~/Documents/aic/tools/capture_trial.sh` and READS from
`~/Documents/aic/data/recordings/`. It does NOT modify any file under
`~/Documents/aic/`. No `plans/cross_repo_changes.json` entries are required
for runtime invocation — only source-of-truth edits trigger that ledger.
