# AIC Custom Messages Setup for Isaac Sim 5.0

Phase 2's controller loop subscribes to `aic_control_interfaces/{JointMotionUpdate, MotionUpdate, ControllerState}` and publishes `ros_gz_interfaces/Contacts`. Neither package ships in the IsaacSim-ros_workspaces base build; both must be vendored into the workspace's Dockerfile COPY context and rebuilt against Python 3.11.

This file is the **D-05 ABI landmine fix** — see "Why not use the AIC pixi env directly?" below for why the original CONTEXT.md PYTHONPATH-link approach is broken.

## Why not use the AIC pixi env directly?

The `~/Documents/aic` repo's pixi env builds `aic_control_interfaces` against **Python 3.12**. Isaac Sim 5.0 runs **Python 3.11**. CPython C-extension `.so` files (which ROS message typesupport ships as) are NOT ABI-compatible across minor Python versions. Loading the pixi env's modules from Isaac Sim's Python 3.11 fails at the typesupport-`.so` import (verified 2026-05-03; even numpy fails to import from the pixi env under 3.11 — RESEARCH.md §"D-05 Landmine").

CONTEXT.md's D-05 originally proposed PYTHONPATH-linking the pixi env. **That approach is broken; this doc is the replacement.**

## Prerequisites

- Phase 1's IsaacSim-ros_workspaces is already cloned + built and present at `~/IsaacSim-ros_workspaces/`. If not, complete `rclpy-setup.md` first.
- Docker installed and working (the build runs inside a Humble container).
- `~/Documents/aic` checkout present (source of `aic_control_interfaces`).
- ~10-20 minutes of build time on first run (warm cache: ~5-10 minutes).

## Build

```bash
# 1. Vendor aic_control_interfaces source into the Dockerfile COPY context.
#    NOTE: The path is ~/IsaacSim-ros_workspaces/humble_ws/src/ (the REPO root's
#    humble_ws/src), NOT ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
#    (which is where build_ros.sh extracts results into AFTER the build).
cp -r ~/Documents/aic/aic_interfaces/aic_control_interfaces \
      ~/IsaacSim-ros_workspaces/humble_ws/src/

# 2. Clone ros_gz (humble branch) and vendor ros_gz_interfaces into the same context.
cd /tmp && rm -rf ros_gz && git clone -b humble https://github.com/gazebosim/ros_gz.git --depth 1
cp -r /tmp/ros_gz/ros_gz_interfaces \
      ~/IsaacSim-ros_workspaces/humble_ws/src/

# 3. Rebuild the workspace via build_ros.sh (Docker-based; ~10-20 min cold cache).
cd ~/IsaacSim-ros_workspaces && bash build_ros.sh -d humble -v 22.04
```

Or run the wrapper script that does all three steps with idempotency checks + ABI verification:

```bash
bash exts/aic-dt/scripts/build_aic_msgs.sh
```

## Where the built artifacts land (the install-path landmine)

`build_ros.sh` runs **two colcon builds** inside the Docker container:

1. The **inner** `humble_ws/` build is `--merge-install` and provides the upstream-pinned Humble base (rclpy, common_interfaces, builtin_interfaces, geometry_msgs, std_msgs, trajectory_msgs, ...). It extracts to `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/`.

2. The **outer** `build_ws/` build is **per-package install** (no `--merge-install`) and is where our vendored packages land. Each package gets its own `install/<pkg>/local/lib/python3.11/dist-packages/<pkg>/` subtree. It extracts to `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/`.

So our packages live at:

- `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces/`
- `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/ros_gz_interfaces/local/lib/python3.11/dist-packages/ros_gz_interfaces/`

`~/env_isaaclab/bin/activate` already sources both `humble_ws/install/local_setup.bash` AND `isaac_sim_ros_ws/install/local_setup.bash`. The outer setup auto-discovers per-package installs and adds each one's `local/lib/python3.11/dist-packages` to `PYTHONPATH`. **No env-script edits needed** — vendor + rebuild + venv-activate is the whole story.

## Verification

After the build completes, from a venv-activated shell:

```bash
bash -c 'source ~/env_isaaclab/bin/activate && \
  ~/env_isaaclab/bin/python -c "
from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState
from ros_gz_interfaces.msg import Contacts, Contact, Entity
print(\"AIC custom messages OK\")
"'
```

Expected: `AIC custom messages OK`. If `ImportError`, check:

1. The `.so` files in `~/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/aic_control_interfaces/local/lib/python3.11/dist-packages/aic_control_interfaces/` MUST NOT be tagged `cpython-310-*` (which would indicate the workspace built against the wrong Python). They should be tagged `cpython-311-x86_64-linux-gnu` or untagged.

2. `~/env_isaaclab/bin/activate` MUST source `isaac_sim_ros_ws/install/local_setup.bash`. Without it, `aic_control_interfaces` is not on `PYTHONPATH`.

3. The vendored sources MUST be at `~/IsaacSim-ros_workspaces/humble_ws/src/` (Dockerfile COPY context), NOT at `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/` (post-build extracted dir). The Dockerfile reads from the former.

## D-13 LD_LIBRARY_PATH addition (optional — for the surrounding aic_controller process)

Phase 2's `controller_loop.py` only needs the Python message imports above (which Python's normal `sys.path` resolves via the venv-activate). If the wider `aic_controller` C++ binary is also being run from inside the digital twin (it shouldn't be — the controller stays in the AIC pixi env), the AIC pixi env's lib dir would need to be on `LD_LIBRARY_PATH`:

```bash
export LD_LIBRARY_PATH="$HOME/Documents/aic/.pixi/envs/default/lib:$LD_LIBRARY_PATH"
```

For the controller_loop.py atom this is NOT required — it imports the message classes via Python's pure-Python entry points (which `__init__.py` re-exports), not the underlying C++ libraries.

## Re-running

The build script is idempotent:

- Re-vendoring overwrites `~/IsaacSim-ros_workspaces/humble_ws/src/{aic_control_interfaces,ros_gz_interfaces}/` from source.
- `build_ros.sh` rebuilds the Docker image (Docker layer cache makes incremental rebuilds fast — ~5-10 minutes when only the COPY layer changed).
- `build_ws/humble/` is wiped + repopulated each run.

To pick up upstream changes in `~/Documents/aic/aic_interfaces/aic_control_interfaces/`, just re-run `bash exts/aic-dt/scripts/build_aic_msgs.sh`.
