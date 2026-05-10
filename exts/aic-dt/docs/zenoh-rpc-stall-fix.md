# Zenoh service-RPC stall — root cause + fix

> **Task:** `zenoh-rpc-stall-fix` (PRD 2026-05-10).
> **Status:** ROOT-CAUSED + FIXED.
> **Operator handoff:** trial-{1,2,3}-fire flipped back to `passes:false`; once
> the wrapper is exercised under this fix, those tasks re-fire and produce
> outcome JSONs that reflect the real Isaac Sim behavior (not stall artifacts).

## Symptom

After `zenoh-path-implementation` landed (router + transport up; topic
discovery + service-name discovery green), every `aic_engine` ↔ `aic_model`
service-RPC payload stalled. Engine log signature (verbatim from
`/tmp/aic_engine_isaac.log`, run 2026-05-09T17:24Z, identical across all 3
trial fires):

```
t=0    "Checking participant model readiness..."
t=0    "No node with name 'aic_model' found. Retrying..."
t=2s   "Found 1 node(s) with name 'aic_model'. Checking if it is a lifecycle node..."
t=2s   "Service '/aic_model/get_state' is available. Participant model discovered."
t=2s   "Lifecycle node 'aic_model' is available. Checking if it is in 'unconfigured' state..."
t=10s  "GetState service call timed out for node 'aic_model'"      ← payload STALLS
t=107s "ChangeState service call timed out for transition 'deactivate'"
t=123s "SwitchController service call timed out"
t=217s "ChangeState service call timed out for transition 'cleanup'"
t=241s "ChangeState service call timed out for transition 'shutdown'"
```

The pattern: zenoh **liveliness/discovery** crosses the boundary; zenoh
**queryable/service-RPC** does not.

## Root cause

`rmw_zenoh-cpp` major-version drift between the host and the model container:

| Side | Source | rmw_zenoh-cpp version |
|---|---|---|
| Host (humble engine + adapter + Isaac Sim rclpy) | apt `ros-humble-rmw-zenoh-cpp` | **0.1.8** (Jan 2024) |
| Model container (`my-solution:v1`, kilted-via-pixi) | conda-forge via pixi-build-ros | **0.6.6** (much newer) |

A 5-major-version gap. The version was located via:
```bash
docker exec aic_model cat /ws_aic/src/aic/.pixi/envs/default/share/rmw_zenoh_cpp/package.xml
```

Why **liveliness/discovery still works** across the gap: the liveliness-token
key namespace (`@ros2_lv/...`) is conservative and stayed backward-compatible
between 0.1.x and 0.6.x. So `ros2 service list` / `ros2 node list` from the
humble side enumerates the model's services successfully — fooling the engine
into believing the boundary is healthy.

Why **queryable/service-RPC payloads do NOT cross**: the queryable key
encoding for service request/response payloads carries a type-hash and
internal versioning that is NOT version-stable across that 0.1→0.6 gap. The
model registers its queryable on one key; the engine queries a different key
(or with an incompatible payload framing); zenoh router has nothing to route.

### Bisection probes (reproducible)

H3a — kilted-side (in-container) RPC: **WORKS** in <1s.
```bash
docker exec aic_model bash -c '
  cd /ws_aic/src/aic && pixi run --as-is bash -c "
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=7
    export ZENOH_CONFIG_OVERRIDE=\"connect/endpoints=[\\\"tcp/localhost:7447\\\"];transport/shared_memory/enabled=false\"
    timeout 10 ros2 service call /aic_model/get_state lifecycle_msgs/srv/GetState
  "'
# → response in 0.X s — server is healthy
```

H3b — humble-side (host) RPC into kilted model: **STALLS** (cross-distro reproducer).
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=7
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'
timeout 10 ros2 service call /aic_model/get_state lifecycle_msgs/srv/GetState
# → exit 124 (timeout); discovery worked but RPC didn't deliver
```

H3c — humble-side (host) RPC into HOST-NATIVE humble aic_model: **WORKS** in
~0.9s (post-fix proof).

This 3-probe ladder isolates the boundary cleanly: server-side healthy, the
bridge fails specifically when the two ends are on different rmw_zenoh-cpp
versions.

## Fix applied

**Path:** rebuild `aic_model` + `aic_example_policies` humble-native on the
host so every node in the pipeline (Isaac Sim rclpy publishers, aic_engine,
aic_adapter, aic_model) speaks rmw_zenoh-cpp 0.1.8 — same wire format on every
side, no cross-distro queryable-key boundary.

**Why this path** (vs. building humble→0.6.x or kilted→0.1.x): rmw_zenoh-cpp
0.6.x is not packaged for humble (apt has 0.1.8), and pixi-locked kilted does
not allow downgrade. Source-builds on either side work but require porting
significant transitive deps. Path-of-least-resistance is to keep the host on
its apt-pinned 0.1.8 and bring the model onto the same toolchain.

### Concrete changes

1. **New script `exts/aic-dt/scripts/build_aic_model_host.sh`** — sister to
   `build_aic_engine_host.sh`. Vendors `aic_model` + `aic_example_policies`
   from `~/Documents/aic` into `~/aic_humble_ws/src`, applies one Python
   compatibility pivot, builds via colcon. Idempotent.

2. **Pivot — `aic_model/aic_model/aic_model.py:323`** — humble-vs-kilted
   `rclpy.init` API drift:
   - Kilted: `rclpy.init(args=args)` returns a `Context` that supports
     `__enter__`/`__exit__` so `with rclpy.init(args=args):` works.
   - Humble: `rclpy.init(args=args)` returns `None`; the kilted idiom raises
     `AttributeError: __enter__`.
   The pivot rewrites `main()` into the `try:`/`finally:` form which works on
   both distros (applied via in-script `sed`-like Python edit; see
   `build_aic_model_host.sh` for the exact `old`/`new` blocks). The vendored
   copy in `aic_humble_ws/src` is patched on every build; the upstream
   `~/Documents/aic/aic_model/aic_model/aic_model.py` is **not** modified.

3. **Wrapper change — `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh`** —
   the model launch step replaced. Was: `docker run my-solution:v1 …` (kilted).
   Now: host process running `~/aic_humble_ws/install/aic_model/lib/aic_model/aic_model`
   under the same humble + zenoh env as the engine and adapter. The legacy
   `MODEL_CONTAINER`/`MODEL_IMAGE` constants are retained for cleanup
   (best-effort `docker rm -f aic_model`) so old leftovers don't collide.

4. **No edits in `~/Documents/aic`** — the upstream sources are read-only;
   the pivot lives in our build script. Therefore no `cross_repo_changes.json`
   entry is required for this task.

## Verification

The test harness is `exts/aic-dt/scripts/test_zenoh_rpc_stall_fix.sh`. It:
1. Pre-flights the new host humble model binary; rebuilds via
   `build_aic_model_host.sh` if missing.
2. Pre-flights the host engine binaries (`build_aic_engine_host.sh` if
   missing).
3. Re-runs the trial_1 fire end-to-end via `run_aic_engine_against_isaac_sim.sh`
   with `--output-json`.
4. Asserts engine stdout contains **zero** `service call timed out` lines.
5. Asserts the captured outcome JSON has `insertion_event_fired ∈ {true, false}`
   (i.e. parses cleanly), and emits a one-line verdict.

Reproducible:
```bash
bash exts/aic-dt/scripts/test_zenoh_rpc_stall_fix.sh
# → "PASS" + exit 0 when the RPC stall is gone.
```

A standalone "humble↔humble vs humble↔kilted" probe is also documented in the
H3a/H3b commands above; either operator can manually re-run them to convince
themselves the fix changes the regime.

## Operator follow-up

`trial-1-fire` / `trial-2-fire` / `trial-3-fire` carry `passes:false` post-fix
because their captured JSONs were taken under the stalled-engine regime. Each
needs to re-fire under the new transport so the next `parity-report-run`
captures a real Isaac Sim outcome and not a stall artifact. The PRD `notes`
field on `zenoh-rpc-stall-fix` already calls this out.
