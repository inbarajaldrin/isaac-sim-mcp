# soarm101-dt Extension

Isaac Sim extension for SO-ARM101 Digital Twin with MCP (Model Context Protocol) integration.

## Features

- **SO-ARM101 Robot Control**: Load and configure the SO-ARM101 5-DOF robotic arm with integrated gripper
- **ROS2 Joint Control**: Action graphs for joint state subscription, gripper control, and force/torque publishing
- **Intel RealSense Camera**: Wrist-mounted camera with custom 3D-printed mount
- **Lego Block Objects**: Add, delete, sort, and randomize colored lego blocks (red, green, blue; 2x2, 2x3, 2x4)
- **Scene State Management**: Save and restore object poses to JSON for repeatable experiments
- **Sim-to-Real Sync**: Subscribe to real-world detected poses and mirror them in simulation
- **MCP Integration**: Socket server for remote control via MCP protocol

## MCP Tools

This extension exposes the following tools via MCP:

| Tool | Description |
|------|-------------|
| `execute_python_code` | Execute arbitrary Python code in Isaac Sim's environment |
| `play_scene` | Start/resume the simulation timeline |
| `stop_scene` | Stop the simulation timeline |
| `load_scene` | Initialize scene with physics, ground plane, and frame rate settings |
| `load_robot` | Import SO-ARM101 robot USD with joint drives and gripper physics |
| `setup_action_graph` | Create ROS2 action graph for joint state subscription and clock |
| `setup_gripper_action_graph` | Create ROS2 gripper control action graph |
| `setup_force_publisher` | Create ROS2 force/torque publisher for end-effector wrench |
| `add_objects` | Add lego block objects to the scene |
| `delete_objects` | Delete all objects and associated action graphs |
| `sort_objects` | Sort blocks by placing same-color blocks in clusters |
| `randomize_object_poses` | Randomize block positions with per-object collision avoidance |
| `randomize_single_object` | Randomize a single named object while keeping others fixed |
| `save_scene_state` | Save current object poses to a timestamped JSON file |
| `restore_scene_state` | Restore object poses from a JSON file |
| `setup_pose_publisher` | Publish object poses to ROS2 topic `objects_poses_sim` |
| `sync_real_poses` | Subscribe to `/objects_poses_real` and update sim poses |

## Socket Server

The extension runs a socket server on port 8767 (configurable via `MCP_SERVER_PORT`).

Commands are sent as JSON:
```json
{
  "type": "command_name",
  "params": {}
}
```

## Configuration

- `MCP_SERVER_PORT`: Socket server port (default: 8767)
- Robot USD: `omniverse://localhost/Projects/so-arm101/SO-ARM101-USD.usd`
- Lego blocks: `omniverse://localhost/Projects/so-arm101/legos/`
- Objects folder: `/World/Objects`
