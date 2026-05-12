# aic-dt Extension

Isaac Sim Kit extension for the AIC (AI for Industry Challenge) cable-insertion
digital twin. Exposes an MCP (Model Context Protocol) socket on port 8768 so
the AIC competition stack — unmodified `aic_controller`, `aic_engine`, and
`aic_example_policies/ros/CheatCode.py` from `~/Documents/aic` — can drive
Isaac Sim with the same topic surface it expects from Gazebo.

## Status — Milestone 1 (Platform Transfer): SHIPPED 2026-05-12

Every trial in `~/Documents/aic/aic_engine/config/sample_config.yaml`
(trial_1, trial_2, trial_3) passes against this Isaac Sim digital twin
under unmodified `CheatCode.py`. Side-by-side parity with Gazebo baselines
is verified in `docs/parity-report.md` (regenerable via
`exts/aic-dt/scripts/parity_report.py`).

## Features

- **UR5e + Robotiq Hand-E** at Gazebo-canonical mount pose `(-0.2, 0.2, 1.14, yaw=-π)`
- **AIC task board** spawned per-trial: NIC card mounts, SC ports, LC/SFP/SC mount rails — URDF-canonical anchors mirroring the Gazebo `aic_description/urdf/task_board.urdf.xacro` exactly
- **Cable + plug proxy** — kinematic plug proxy authored as a USD-hierarchy child of `gripper_hande_finger_link_l` (bypasses PhysX articulation cook crashes for cross-articulation loop joints); scoring uses contact-based plug-port detection
- **ROS 2 topic surface** matching Gazebo's `aic_eval` container: `/joint_states`, `/tf`, `/clock`, `/force_torque_sensor_broadcaster/wrench`, `/scoring/insertion_event`, `/scoring/tf`, `/objects_poses_real`, `/aic_controller/*` — under zenoh transport (`rmw_zenoh_cpp`) at `ROS_DOMAIN_ID=7`
- **Per-trial scene loader** (`load_trial`) reads `sample_config.yaml` directly; spawns task-board parts, applies home joints, wires per-trial TF frames + scoring publishers
- **MCP socket server** on port 8768 — JSON request/response protocol

## Launching

See repo-root `CLAUDE.md` for the canonical launch sequence + cache discipline.
Quick path (with host zenohd running):

```bash
bash exts/aic-dt/scripts/launch_host_zenohd.sh
bash -c 'source /opt/ros/humble/setup.bash && source ~/env_isaaclab/bin/activate && \
  export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_zenoh_cpp \
         ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
         ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/localhost:7447\"];transport/shared_memory/enabled=false" \
         DISPLAY=:0 && \
  bash ~/.claude/skills/isaac-sim-extension-dev/scripts/isaacsim_launch.sh launch aic-dt'
```

## Firing a trial end-to-end

```bash
bash exts/aic-dt/scripts/run_aic_engine_against_isaac_sim.sh trial_1 \
     --output-json=exts/aic-dt/docs/trial_outcomes/trial_1_isaacsim.json
```

The wrapper starts host `aic_engine` + `aic_adapter` and the `my-solution:v1`
model container running CheatCode, drives the trial against Isaac Sim,
captures the outcome JSON. ~30-60s wall-clock per trial.

## MCP socket protocol

TCP on `127.0.0.1:8768`. Send a single JSON object, receive one JSON object.

```python
import socket, json
s = socket.socket(); s.connect(("127.0.0.1", 8768))
s.sendall(json.dumps({"type": "quick_start", "params": {}}).encode())
buf = b""
while True:
    chunk = s.recv(16384)
    if not chunk: break
    buf += chunk
    try: print(json.loads(buf.decode())); break
    except json.JSONDecodeError: continue
```

Every MCP tool name + parameters lives in the `MCP_TOOL_REGISTRY` dict at
the top of `aic_dt/extension.py`. Handler bodies are `_cmd_<name>` methods
on the `DigitalTwin` class.

## Configuration

- `MCP_SERVER_PORT`: socket port (hardcoded to 8768 in code, not env-overridable)
- `MCP_CLIENT_OUTPUT_DIR`: if set, scene-state JSON writes under `$DIR/resources/`
- `AIC_DT_EXT_FOLDER`: override hardcoded ext-folder in `scripts/launch_postload.py`
- `ROS_DOMAIN_ID`: MUST be 7 (matches Gazebo + isolates from live UR5e drivers on domain 0)
- `RMW_IMPLEMENTATION`: MUST be `rmw_zenoh_cpp` for engine pipeline tasks (transport handshake with humble aic_engine + kilted my-solution:v1 model container)
- `SCENE_05_ENABLE_FOR_M2`: optional, re-activates cable rope dynamics for M2 work; M1 keeps cable inactive (decorative, plug proxy is the scoring body)

## See also

- Repo-root `CLAUDE.md` — full project guide, M1 autonomous-mode block, launch paths, cross-repo policy
- `~/Documents/aic/CLAUDE.md` — ROS-side stack reference (controller, engine, policies, asset URDFs)
- `docs/parity-report.md` — Gazebo ↔ Isaac trial outcome comparison (regenerable)
- `docs/CHANGELOG.md` — per-milestone delivery history
