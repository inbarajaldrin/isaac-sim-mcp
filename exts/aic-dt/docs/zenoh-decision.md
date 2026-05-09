# Zenoh Transport Decision — model ↔ engine ↔ Isaac Sim

> **Task:** `zenoh-path-research` (PRD 2026-05-09).
> **Author:** autonomous loop, 2026-05-09.
> **Output gates:** `zenoh-path-implementation`.

## Why zenoh

Investigated `~/Documents/aic` source-of-truth. Three corroborating reasons drove AIC's
choice of `rmw_zenoh_cpp` over the ROS 2 default RMW (`rmw_fastrtps_cpp` for Humble,
`rmw_zenoh_cpp` is also default for newer distros):

1. **Process-boundary security via ACL** — `~/Documents/aic/CLAUDE.md` lines 341-343:
   *"ACL (Access Control Lists) supported for eval security
   (`docker/aic_eval/aic_zenoh_config.json5`)"*. The eval container runs the zenoh
   router with optional `usrpwd` auth (env `AIC_ENABLE_ACL=true`,
   `AIC_EVAL_PASSWD`/`AIC_MODEL_PASSWD`); the model container connects with credentials
   gated against `/credentials.txt`. In the eval phase the participant model is
   untrusted code; the router's ACL is the boundary that prevents the model container
   from publishing/subscribing on protected topics. **DDS multicast/discovery has no
   equivalent boundary** — a fastdds bridge would silently lose this.

2. **Container-network isolation, no multicast** — `docker/docker-compose.yaml`
   declares `networks.default: { internal: true }` and the model container connects via
   `AIC_ROUTER_ADDR=eval:7447` (TCP). Zenoh's router-client topology cleanly maps onto
   isolated Docker networks with a known router endpoint. **DDS discovery is multicast
   by default**; getting it to work across Docker networks requires either host-mode
   networking (defeats isolation) or unicast peer lists (operationally fragile,
   distro-dependent). Zenoh sidesteps the problem entirely.

3. **Strict process-boundary architecture** — `~/Documents/aic/CLAUDE.md` line 496:
   *"Strict process boundary between evaluation infrastructure (C++, runs in Docker)
   and participant model (Python, runs outside or in separate container).
   Communication exclusively via ROS 2 topics, services, and actions over Zenoh
   middleware."* The architectural decision is explicit; using anything else for the
   model⇄engine path is a deviation from the documented contract.

Confirming evidence:
- `pixi.toml` declares `ros-kilted-rmw-zenoh-cpp = "*"`.
- `pixi_env_setup.sh` sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` for the host pixi env.
- Both Dockerfiles (`docker/aic_eval/Dockerfile`, `docker/aic_model/Dockerfile`)
  `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in the entrypoint **after** the docker
  `-e` env vars are processed — meaning a docker-run-level RMW override does **not**
  stick. The container will always use zenoh internally.

> ⚠️ **Latent bug in current wrapper.** `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh:224`
> sets `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on the model container, but the
> entrypoint exports `rmw_zenoh_cpp` immediately after — the docker env var is dead
> code. Same line sets `ZENOH_ROUTER_CHECK_ATTEMPTS=0` to skip the router check, which
> is a workaround for the **absence** of a router, not a fix. Net effect: the model
> container speaks zenoh into the void; the host engine speaks fastrtps; **discovery
> never crosses the gap**, which is exactly the symptom this research task is gating.

## Path verdict

**Chosen: Path (a) — host-side zenoh router + unify both Isaac Sim host engine and
model container on `rmw_zenoh_cpp`.**

Rationale:

- **Preserves architectural fidelity.** The participant-model boundary is the most
  load-bearing AIC architectural choice; the chosen reason (ACL-able process boundary)
  must survive into Isaac Sim's pipeline. Path (a) preserves it directly.
- **`ros-humble-rmw-zenoh-cpp` exists in apt.** Verified `apt-cache search` —
  the host's Humble workspace (which is what Isaac Sim's rclpy + the host-built
  aic_engine load) can install zenoh natively. **No source build, no humble-rebuild
  workaround.** This makes Path (a) cheaper than Path (b)'s spec.
- **No bridge needed.** Earlier framings of Path (a) assumed a fastdds↔zenoh bridge
  (`zenoh-bridge-ros2dds`). With the apt package available we just **switch sides**
  rather than bridge them — every node on the host (Isaac Sim rclpy publishers,
  aic_engine, aic_adapter) gets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` set, and the model
  container points its `AIC_ROUTER_ADDR` at the host-running `zenohd`. Single
  transport, no translator, no type-hash boundary problems.
- **Path (b) is dominated.** Building aic_engine humble-native already happened in
  Plan 04-03 (`build_aic_engine_host.sh` produces `~/aic_humble_ws/install/aic_engine/`).
  The blocker in 04-03 was kilted↔humble fastrtps type-hash incompatibility; switching
  the host engine to zenoh removes that variable too — zenoh's wire format is
  ROS-distro-agnostic.
- **Path (c) is rejected for the M1 ship gate.** A stub `aic_model` would let us
  smoke-test "Isaac Sim publishes correctly + scoring fires" but **does not run
  unmodified CheatCode**. The M1 stop condition explicitly requires "all 3 trials in
  sample_config.yaml pass under unmodified CheatCode against Isaac Sim". A stub fails
  that requirement. Path (c) is allowed only as a **discriminator** during impl
  debugging if Path (a) hits unexpected friction.

Architectural-fidelity check: with Path (a), Isaac Sim host = zenoh peer/client; model
container = zenoh peer/client; both connect to a single host-side zenoh router on
`tcp/[::]:7447`. This is **structurally identical** to the AIC docker-compose topology
where `aic_eval` container hosts the router and `model` container connects — only the
deployment changes (eval-functionality moved from Docker to Isaac-Sim-host) while the
transport semantics, ACL surface, and discovery model are preserved.

## Implementation plan (consumed by `zenoh-path-implementation`)

Concrete steps the next task should execute. Cross-repo touches are permitted under
PRD `cross_repo_policy`; track all edits outside `~/Documents/isaac-sim-mcp/` in
`plans/cross_repo_changes.json`.

### 1. Install host zenoh stack (apt)

```bash
sudo apt update
sudo apt install -y ros-humble-rmw-zenoh-cpp
```

This provides `rmw_zenoh_cpp` (the RMW shim) AND the `rmw_zenohd` standalone router
binary (`ros2 run rmw_zenoh_cpp rmw_zenohd`). No host-side compilation.

### 2. Launch the host zenoh router

Add `exts/aic-dt/scripts/launch_host_zenohd.sh` that:
- Sources `/opt/ros/humble/setup.bash` (or the aic_humble_ws overlay if present).
- Exports `ZENOH_ROUTER_CONFIG_URI=$HOME/Documents/aic/docker/aic_eval/aic_zenoh_config.json5`
  (reuse AIC's authoritative router config so we inherit the ACL surface for free if
  later toggled; for now we run without ACL to match `AIC_ENABLE_ACL=false` default).
- Sets `ZENOH_CONFIG_OVERRIDE='mode="router";listen/endpoints=["tcp/[::]:7447"];connect/endpoints=[];routing/router/peers_failover_brokering=true;transport/shared_memory/enabled=false'`
  (mirrors `docker/aic_eval/Dockerfile:60-68`).
- Execs `ros2 run rmw_zenoh_cpp rmw_zenohd`.

Bootstrap log: `/tmp/aic_zenohd.log`. Lifecycle: bare process, can be killed with
`pkill -SIGINT -f rmw_zenohd`.

### 3. Switch the host aic_engine + aic_adapter to zenoh

Edit `exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh`:
- Replace the two `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` lines (lines 192, 208)
  with `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- Add `export ZENOH_ROUTER_CHECK_ATTEMPTS=-1` and
  `export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'`
  before the engine + adapter execs.
- Also do the same for the `ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=...` ros2-cli
  precondition probe at line 169.

### 4. Switch the Isaac Sim aic-dt extension publishers to zenoh

The extension's rclpy parity publishers run inside the Kit process. Two approaches:

- **Preferred:** wrap the Isaac Sim launch with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  in addition to the existing `ROS_DOMAIN_ID=7`. This requires the
  `~/IsaacSim-ros_workspaces/humble_ws` build to have `rmw_zenoh_cpp` discoverable on
  the workspace's `LD_LIBRARY_PATH` / `AMENT_PREFIX_PATH`. Verify with
  `ros2 doctor --report` from a sourced shell. If `/opt/ros/humble` is sourced as part
  of the env_isaaclab activation, the apt-installed zenoh RMW is already on the path.
- **Fallback:** if the workspace overrides `/opt/ros/humble` and shadows the apt
  zenoh, build `rmw_zenoh_cpp` from source into the workspace. Documented as the
  fallback path in the impl task; no need to pre-empt unless verification surfaces
  the issue.

### 5. Switch the model container connection target to host router

Edit the same wrapper:
- Remove `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (it's dead code anyway since the
  entrypoint overrides; remove for clarity).
- Remove `-e ZENOH_ROUTER_CHECK_ATTEMPTS=0` (we DO want it to wait for the router).
- Add `-e AIC_ROUTER_ADDR=host.docker.internal:7447` (Linux: requires
  `--add-host=host.docker.internal:host-gateway`) **OR** drop `--net=host` and add
  `-e AIC_ROUTER_ADDR=<host-LAN-ip>:7447`. Path-of-least-resistance: keep
  `--net=host`, change `AIC_ROUTER_ADDR=localhost:7447` since the container shares
  the host network namespace and `localhost` resolves to the host. **Prefer this last
  variant** — minimal delta from current wrapper.
- Add `-e ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7447"];transport/shared_memory/enabled=false'`.

### 6. Verify discovery + service registration

`exts/aic-dt/scripts/verify_zenoh_path.sh` (NEW, created by impl task) must:
- Confirm `pgrep -f rmw_zenohd` returns a PID.
- From a fresh shell with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, run
  `ros2 topic list` and assert `/clock`, `/joint_states`, `/tf` are visible while
  Isaac Sim is publishing.
- `docker exec aic_model bash -c 'source /opt/ros/kilted/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 topic echo --once /clock'` returns a Clock
  message within 10s — proves cross-container discovery.
- `docker exec aic_model bash -c '... && ros2 service list | grep /aic_model/get_state'`
  returns the lifecycle service — proves the model container's lifecycle node
  registered.
- Exit 0 only if all four pass.

### 7. Stretch (optional for impl task, definitely for ship-paperwork)

Document the host-zenohd lifecycle in `CLAUDE.md` alongside the Isaac Sim launch
section, so future sessions don't lose the convention.

## Risk register

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Apt `ros-humble-rmw-zenoh-cpp` lacks the `rmw_zenohd` binary on this distro/version | Low | Verified package exists; will see in step 1. Fallback: install `ros-humble-zenoh-cpp-vendor` directly. |
| Isaac Sim's workspace overlay shadows apt zenoh | Medium | Step 4 fallback addresses; check with `ros2 doctor` first. |
| Kilted (model container) zenoh ↔ Humble (host) zenoh wire-protocol mismatch | Low | Zenoh wire protocol is ROS-distro-agnostic by design (one of its selling points vs DDS). If it fails, switch host engine to kilted via Path (b)'s previously-built `~/aic_humble_ws` (rebuilt with kilted overlay) — but this is a deeper detour, treat as escalation, not first attempt. |
| ACL config from AIC's json5 doesn't load cleanly without ACL env vars | Low | The config is mode-router with ACL gated by `AIC_ENABLE_ACL`; reading without that env var should be inert. Verify with `rmw_zenohd` startup logs in step 2. |

## Open questions deferred to impl

- Whether to cache the host zenohd as a systemd-user service vs ad-hoc launch. Defer
  to operator preference; impl task just makes a launch script.
- Whether to mirror `aic_zenoh_config.json5` into this repo for self-containment vs
  reference AIC's. Reference for now (no edit risk in AIC repo); copy-and-customize
  only if we need divergence.
