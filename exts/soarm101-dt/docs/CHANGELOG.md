# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [1.1.0] - 2026-02-28

### Added
- Bundled local USD assets: SO-ARM101-USD.usd, lego blocks (red/green/blue 2x2/2x3/2x4), camera_wrist_mount.usd
- Wrist camera section: Setup Camera Action Graph button and `setup_wrist_camera_action_graph` MCP tool
- BBox publisher: `setup_bbox_publisher` MCP tool for object bounding boxes on `objects_bbox_sim`
- `assets/legos/README.md` and `assets/wrist_mounts/` layout for local assets
- `scripts/fix_soarm101_usd.py` to copy and fix original USD (base_delta, camera mount reference)

### Changed
- **Local-only assets**: Robot, legos, camera mount load from `assets/`; no Nucleus fallback (FileNotFoundError if missing)
- Pre-fixed SO-ARM101 USD in `assets/SO-ARM101-USD.usd` (base_delta baked in; skip runtime correction)
- Joint drive params: max force 3 Nm (STS3215 spec), stiffness/damping tuned for real2sim
- **MCP handlers delegate to async methods**: `load_scene`, `load_robot`, `setup_action_graph` now use `run_coroutine()` to call the same code as UI buttons (fixes ArticulationView, camera mass/inertia, simulation context)
- Quick Start: camera mount, lego objects, pose publisher, bbox publisher; removed workspace camera and gripper action graph steps

### Removed
- Gripper action graph (integrated gripper controlled via `/joint_states` joint index 5)
- `setup_gripper_action_graph` MCP tool and UI button

## [1.0.0] - 2026-02-28
- Initial release of SO-ARM101 digital twin extension
- SO-ARM101 5-DOF arm with integrated gripper control
- ROS2 action graphs: joint state subscriber, gripper control, force/torque publisher
- Wrist-mounted Intel RealSense camera with custom mount
- Lego block object management (add, delete, sort, randomize)
- Per-object collision-aware randomization with largest-first placement
- Scene state save/restore with timestamped JSON files
- Sim-to-real object pose synchronization via ROS2
- Object pose publisher to ROS2 topic
- Quick Start UI button for one-click setup
- MCP socket server integration on port 8767
