# Dependency Map — operator reference only

> **NOT loaded by the iteration prompt.** This file exists for the operator to track which tasks have informal prerequisites, in case `tug afk` iterations start failing in patterns that suggest a missing prerequisite. The agent picks freely from `plans/prd.json` without seeing this file.
>
> Pocock canon: kanban tasks are independently grabbable; blocking relationships are metadata, not gates. Failed verifies teach the agent to pick differently next iteration.

## Informal prerequisites (probable, not enforced)

These are the prerequisite chains I'd expect, surfaced from each task's `description` text. The agent reads each task's description and uses judgment to decide what to attempt.

| Task | Probable prerequisites | Why |
|---|---|---|
| `motion-deficit-hunt` | none | Leaf — sim-side controller_loop diagnostic |
| `pose-roundtrip-verify` | `motion-deficit-hunt` (likely closes simultaneously) | Same surface (controller_loop.py); Plan 04-3.5 notes say they share root cause |
| `wrench-rootcause` | none | Independent extension.py + parity_publishers.py work |
| `gripper-tcp-ingress` | `pose-roundtrip-verify` | gripper/tcp ingress relies on pose-cmd path being correct |
| `wrist-cameras-restore` | none | Independent USD authoring + atom wiring |
| `joint-drives-urdf-reconcile` | none | Independent extension.py drive setting |
| `taskboard-prim-authoring` | none | Independent USD authoring; gates trial scoring |
| `isaaclab-leverage-research` | none | Independent docs research; informational only |
| `zenoh-path-research` | none | Independent docs research |
| `zenoh-path-implementation` | `zenoh-path-research` | Verdict has to be picked before implementation |
| `wrapper-json-emit` | `zenoh-path-implementation` | Wrapper exercise requires engine container to actually talk to model |
| `gazebo-baselines-capture` | none | Independent — Gazebo bringup is orthogonal to Isaac Sim work |
| `parity-report-tool` | none | Independent — pure Python infra |
| `trial-1-fire` | `motion-deficit-hunt`, `pose-roundtrip-verify`, `wrench-rootcause`, `gripper-tcp-ingress`, `joint-drives-urdf-reconcile`, `taskboard-prim-authoring`, `zenoh-path-implementation`, `wrapper-json-emit` | First end-to-end E2E run; needs sim runtime gates + transport + wrapper |
| `trial-2-fire` | `trial-1-fire` (sequencing reuses sim state) | Sequential to amortize sim restart |
| `trial-3-fire` | `trial-2-fire` | Sequential |
| `parity-report-run` | `parity-report-tool`, `gazebo-baselines-capture`, `trial-1-fire`, `trial-2-fire`, `trial-3-fire`, `wrapper-json-emit` | Needs both sides' JSONs + the report script |
| `ship-paperwork` | `parity-report-run` | Only paperwork once parity is green |

## Iteration-cap planning

Loose estimate of how many iterations to expect per cap, given the above:

- **First `tug afk 8`** — should close 5-7 leaves (gazebo-baselines, parity-report-tool, isaaclab-leverage-research, zenoh-path-research, joint-drives-urdf-reconcile, wrist-cameras-restore, taskboard-prim-authoring). The big spike (`motion-deficit-hunt`) likely needs >1 iteration; mark progress in progress.txt and retry next cap.
- **Second cap (`tug afk 15`)** — should close `motion-deficit-hunt` + `pose-roundtrip-verify` + `wrench-rootcause` + `gripper-tcp-ingress` + `zenoh-path-implementation` + `wrapper-json-emit`. ~6 hard tasks; budget for retries.
- **Third cap (`tug afk 8`)** — `trial-1/2/3-fire` + `parity-report-run` + `ship-paperwork`. Ship gate.

Total estimate: 30-40 iterations to ship M1, depending on how many spike tasks need multi-iter exploration.

## When to consult this file

- After 2-3 iterations failing on the same task → check if it has unmet informal prerequisites.
- Before bumping iteration cap → check what's likely still pickable.
- When PRD task ordering needs operator review → check dependency-implied ordering vs current PRD task list.

## When to update this file

- If a task is added or removed from `plans/prd.json`.
- If a task's description changes its informal prerequisites.
- If real-world experience reveals a prerequisite this map missed.
