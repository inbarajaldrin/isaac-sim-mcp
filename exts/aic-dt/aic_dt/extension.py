# Reference: Combines patterns from SO-ARM101 and UR5e Isaac Sim MCP extensions
# https://github.com/aaugus11/isaac-sim-mcp (SO-ARM101 + UR5e templates)
import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import os
import sys
import io
import time
import threading
import glob
import omni.client
import omni.kit.commands
import carb
import math
import socket
import json
import traceback
from typing import Dict, Any

from isaacsim.core.api.world import World
from isaacsim.gui.components.element_wrappers import ScrollingWindow

# MCP socket server port - AIC extension
MCP_SERVER_PORT = 8768

# MCP output directory configuration
BASE_OUTPUT_DIR = os.getenv("MCP_CLIENT_OUTPUT_DIR", "").strip()
if BASE_OUTPUT_DIR:
    BASE_OUTPUT_DIR = os.path.abspath(BASE_OUTPUT_DIR)
    RESOURCES_DIR = os.path.join(BASE_OUTPUT_DIR, "resources")
else:
    RESOURCES_DIR = "resources"

# Local asset paths (no Nucleus dependency)
def _get_assets_folder():
    _ext_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _assets = os.path.join(_ext_dir, "assets")
    if not os.path.isdir(_assets):
        raise FileNotFoundError(
            f"Assets folder not found at {_assets}. "
            "Place asset files in exts/aic-dt/assets/"
        )
    return "file://" + os.path.abspath(_assets) + "/"


def _local_asset(relpath):
    """Resolve a local asset path under exts/aic-dt/assets/ as a file:// URI."""
    _ext_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    full = os.path.join(_ext_dir, "assets", relpath)
    if not os.path.exists(full):
        raise FileNotFoundError(f"Asset not found: {full}")
    return "file://" + os.path.abspath(full)


# AIC task board object definitions (local assets only)
AIC_OBJECTS = {
    # Path values restored to the m + Z-up originals (mPU=1.0, upAxis=Z) — Plan 04
    # silently swapped these to the cm + Y-up `*_visual.usd` variants that AIC's
    # IsaacLab vendored alongside the originals (mPU=0.01, upAxis=Y) — those import
    # at 100x smaller scale and rotated 90deg into the mPU=1.0 + Z-up parent stage.
    # The originals are byte-identical to the pre-Plan-02 snake_case versions
    # (md5-verified) — this restores the prior known-good visual state while
    # preserving Plan 02's vendoring layout (capitalized folders + sibling textures/).
    "task_board_base": {
        "usd": "assets/Task Board Base/task_board_rigid.usd",
        "position": (0.2837, 0.229, 0.0),
        "rotation": None,  # identity quaternion
    },
    "sc_port_1": {
        "usd": "assets/SC Port/sc_port.usd",
        "position": (0.2904, 0.1928, 0.005),
        "rotation": (0.73136, 0.0, 0.0, -0.682),  # wxyz
    },
    "sc_port_2": {
        "usd": "assets/SC Port/sc_port.usd",
        "position": (0.2913, 0.1507, 0.005),
        "rotation": (0.73136, 0.0, 0.0, -0.682),  # wxyz
    },
    "nic_card": {
        "usd": "assets/NIC Card/nic_card.usd",
        "position": (0.25135, 0.25229, 0.0743),
        "rotation": None,
    },
}

# AIC domain randomization offsets (from Isaac Lab events.py)
# Offsets are relative to the task board base position
AIC_RANDOMIZATION = {
    "task_board_base": {
        "x_range": (-0.005, 0.005),
        "y_range": (-0.005, 0.005),
    },
    "sc_port_1": {
        "offset_from_board": (0.0067, -0.0362, 0.005),
        "x_range": (-0.005, 0.020),
        "y_range": (0.0, 0.0),  # no Y randomization
    },
    "sc_port_2": {
        "offset_from_board": (0.0076, -0.0783, 0.005),
        "x_range": (-0.005, 0.020),
        "y_range": (0.0, 0.0),  # no Y randomization
    },
    "nic_card": {
        "offset_from_board": (-0.03235, 0.02329, 0.0743),
        "y_steps": [0.0, 0.040, 0.080, 0.120],  # snapped to 40mm steps
    },
}

# Wrist cameras built into the robot USD.
# prim_suffix is appended to self._robot_prim_path (= /World/UR5e). The optical
# frame Xforms live UNDER the articulation root (aic_unified_robot/), which is
# why the legacy "/World/UR5e/center_camera_optical/..." paths silently
# skipped during setup_wrist_cameras (STATE.md Finding #3 root cause).
WRIST_CAMERAS = {
    "center_camera": {
        "prim_suffix": "aic_unified_robot/center_camera_optical/center_camera",
        "topic": "center_camera/image",
        "info_topic": "center_camera/camera_info",
        "frame_id": "center_camera_optical",
        "width": 640,
        "height": 480,
    },
    "left_camera": {
        "prim_suffix": "aic_unified_robot/left_camera_optical/left_camera",
        "topic": "left_camera/image",
        "info_topic": "left_camera/camera_info",
        "frame_id": "left_camera_optical",
        "width": 640,
        "height": 480,
    },
    "right_camera": {
        "prim_suffix": "aic_unified_robot/right_camera_optical/right_camera",
        "topic": "right_camera/image",
        "info_topic": "right_camera/camera_info",
        "frame_id": "right_camera_optical",
        "width": 640,
        "height": 480,
    },
}

# =============================================================================
# MCP TOOL REGISTRY - Single source of truth for all MCP tools
# =============================================================================
MCP_TOOL_REGISTRY = {
    "execute_python_code": {
        "description": "Execute Python code directly in Isaac Sim's environment with access to Omniverse APIs. Set persistent=True with a session_id to keep variables alive between calls.",
        "parameters": {
            "code": {
                "type": "string",
                "description": "Python code to execute in Isaac Sim"
            },
            "session_id": {
                "type": "string",
                "description": "Session identifier for persistent execution. Use the same ID across calls to share state. Defaults to 'default'."
            },
            "persistent": {
                "type": "boolean",
                "description": "If true, variables persist across calls with the same session_id."
            }
        }
    },
    "play_scene": {
        "description": "Start/resume the simulation timeline in Isaac Sim.",
        "parameters": {}
    },
    "stop_scene": {
        "description": "Stop the simulation timeline in Isaac Sim.",
        "parameters": {}
    },
    "load_scene": {
        "description": "Initialize the simulation scene with physics, ground plane, dome light, and AIC enclosure.",
        "parameters": {}
    },
    "load_robot": {
        "description": "Import the UR5e robot (with integrated Robotiq Hand-E gripper and cable) at /World/UR5e. SCENE-04: pose configurable via robot_x/y/z/roll/pitch/yaw and cable_x/y/z/roll/pitch/yaw — parameter names + defaults match aic_gz_bringup.launch.py (Gazebo canonical). Cable params are wired through but produce no-op effect in Phase 1 (cable SetActive(False) per D-04).",
        "parameters": {
            "robot_x": {"type": "float", "description": "Robot base X (meters). Default None → -0.2 (Gazebo canonical)."},
            "robot_y": {"type": "float", "description": "Robot base Y (meters). Default None → 0.2 (Gazebo canonical)."},
            "robot_z": {"type": "float", "description": "Robot base Z (meters). Default None → 1.14 (Gazebo canonical; sits on task-board mount surface)."},
            "robot_roll": {"type": "float", "description": "Robot base roll (radians). Default 0.0."},
            "robot_pitch": {"type": "float", "description": "Robot base pitch (radians). Default 0.0."},
            "robot_yaw": {"type": "float", "description": "Robot base yaw (radians). Default None → -3.141 (Gazebo canonical; 180° flip facing -X)."},
            "cable_x": {"type": "float", "description": "Cable subtree X (meters). Default 0.172 (aic_gz_bringup.launch.py). No-op effect in Phase 1."},
            "cable_y": {"type": "float", "description": "Cable subtree Y. Default 0.024."},
            "cable_z": {"type": "float", "description": "Cable subtree Z. Default 1.518."},
            "cable_roll": {"type": "float", "description": "Cable subtree roll (radians). Default 0.4432."},
            "cable_pitch": {"type": "float", "description": "Cable subtree pitch (radians). Default -0.48."},
            "cable_yaw": {"type": "float", "description": "Cable subtree yaw (radians). Default 1.3303."},
        }
    },
    "setup_action_graph": {
        "description": "Create the ROS2 action graph for UR5e joint state subscription and clock publishing.",
        "parameters": {}
    },
    "setup_force_publisher": {
        "description": "Create the ROS2 force/torque publisher action graph for UR5e end-effector wrench.",
        "parameters": {}
    },
    "setup_tf_publisher": {
        "description": "Create the ROS2 TF publisher action graph for /tf (dynamic) and /tf_static (TRANSIENT_LOCAL via staticPublisher=True). Articulation root walks via ROS2PublishTransformTree.",
        "parameters": {}
    },
    "setup_joint_state_publisher": {
        "description": "Create the ROS2 /joint_states publisher action graph (sensor_msgs/JointState). Reads articulation state and emits ROS2 messages from the sim thread.",
        "parameters": {}
    },
    # Phase 2 — controller-loop atoms (Plan 02-02 skeleton; Plans 02-03..06 fill callbacks).
    "setup_controller_subscribers": {
        "description": "Subscribe to /aic_controller/joint_commands (JointMotionUpdate) and /aic_controller/pose_commands (MotionUpdate); publish /aic_controller/controller_state (ControllerState). Per PARITY-09/10/11 + D-01/D-04. Idempotent: re-invocation tears down prior loop.",
        "parameters": {}
    },
    "setup_offlimit_contacts": {
        "description": "Subscribe to omni.physx contact events on the configured set of off-limit prims and publish ros_gz_interfaces/Contacts on /aic/gazebo/contacts/off_limit. Per PARITY-06 + D-03/D-10. Default prim filter from exts/aic-dt/docs/offlimit-prim-mapping.md; pass prim_paths to override.",
        "parameters": {
            "prim_paths": {
                "type": "array",
                "items": {"type": "string"},
                "description": "Optional list of USD prim paths to monitor for off-limit contact events. If omitted, uses the default set from offlimit-prim-mapping.md."
            }
        }
    },
    "attach_cable_to_gripper": {
        "description": "SCENE-03 (Plan 03-03): author UsdPhysics.FixedJoint connecting the cable plug-end link to the Robotiq Hand-E gripper finger so the cable moves with the gripper. Idempotent — replaces any prior CableAttachJoint. Default plug-end link discovered by Plan 03-01 topology probe (link_20, closest to sc_plug_visual).",
        "parameters": {
            "plug_link_path": {
                "type": "string",
                "description": "USD path to cable plug-end RigidBody. Default /World/UR5e/cable/Rope/Rope/link_20."
            },
            "finger_link_path": {
                "type": "string",
                "description": "USD path to gripper finger link. Default /World/UR5e/aic_unified_robot/gripper_hande_finger_link_l."
            },
            "gripper_initial_pos": {
                "type": "number",
                "description": "Gripper opening position parameter (matches Gazebo's gripper_initial_pos). Recorded for parity but no DOF effect since Hand-E finger is a FixedJoint zero-DOF in our USD per D-09. Default 0.00655."
            }
        }
    },
    "load_trial": {
        "description": "TRIAL-01/02 (Plan 04-02, D-01..D-05): Read a single trial from sample_config.yaml-format YAML and spawn the matching Isaac Sim scene (board pose, rail occupancy, port poses, cable pose, attach state). Reuses existing per-component spawn atoms + load_robot. ground_truth=True (default) starts /scoring/* publishers; False skips them (M2 swap surface).",
        "parameters": {
            "config_path": {
                "type": "string",
                "description": "Path to sample_config.yaml. Default: ~/Documents/aic/aic_engine/config/sample_config.yaml (~ expanded)."
            },
            "trial_key": {
                "type": "string",
                "description": "YAML key under 'trials' (e.g. 'trial_1', 'trial_2', 'trial_3'). Required."
            },
            "ground_truth": {
                "type": "boolean",
                "description": "If True (default), /scoring/tf + /objects_poses_real + /scoring/insertion_event publishers run. If False, only parity publishers (robot frames). Mirrors Gazebo's ground_truth:=... launch arg."
            }
        }
    },
    "gripper_command": {
        "description": "Set the Robotiq Hand-E gripper width by writing drive:linear:physics:targetPosition on both finger prismatic joints (URDF mimic enforced at write — both joints share the same target). Position is the per-finger joint coordinate in meters, range 0..0.025 per URDF grip_pos_max. URDF default is 0.00655.",
        "parameters": {
            "position": {
                "type": "number",
                "description": "Per-finger joint coordinate in meters. Each finger travels 'position' along its axis; total jaw gap delta = 2 * position relative to the URDF-authored neutral. Range 0..0.025. Default 0.00655 (matches sample_config gripper_initial_pos default)."
            }
        }
    },
    "setup_wrist_cameras": {
        "description": "Create ROS2 action graphs for all 3 built-in wrist cameras (center, left, right) publishing RGB and CameraInfo. Legacy convenience — for per-camera control prefer spawn_wrist_camera + start_wrist_camera_stream.",
        "parameters": {}
    },
    "spawn_wrist_camera": {
        "description": "Author a Camera prim under the named wrist camera's _optical Xform (e.g. /World/UR5e/aic_unified_robot/center_camera_optical/center_camera). Applies ROS-optical to Isaac-camera quaternion + intrinsics from WRIST_CAMERAS. Idempotent — re-authors if prim already exists.",
        "parameters": {
            "name": {"type": "string", "description": "Camera name (key in WRIST_CAMERAS dict): 'center_camera', 'left_camera', or 'right_camera'."},
        }
    },
    "remove_wrist_camera": {
        "description": "Delete the Camera prim under the named wrist camera's _optical Xform. Does NOT remove the parent _optical Xform (that's part of the unified robot USD). Stops any active stream for this camera first.",
        "parameters": {
            "name": {"type": "string", "description": "Camera name from WRIST_CAMERAS dict."},
        }
    },
    "start_wrist_camera_stream": {
        "description": "Create the ROS2 action graph that publishes RGB + CameraInfo for the named wrist camera. Idempotent — replaces existing graph for the same camera. Other wrist cameras' streams are NOT affected (each runs independently).",
        "parameters": {
            "name": {"type": "string", "description": "Camera name from WRIST_CAMERAS dict."},
        }
    },
    "stop_wrist_camera_stream": {
        "description": "Tear down the named wrist camera's ROS2 action graph. The Camera prim itself stays in place — call remove_wrist_camera to remove that too.",
        "parameters": {
            "name": {"type": "string", "description": "Camera name from WRIST_CAMERAS dict."},
        }
    },
    "add_objects": {
        "description": "Add AIC task board objects (base, SC ports, NIC card) to the scene at their default positions. Backwards-compatible clubbing — invokes the per-component spawn atoms (spawn_task_board_base, spawn_sc_port, spawn_nic_card) with AIC_OBJECTS defaults.",
        "parameters": {}
    },
    # SCENE-01 / DX-02 — Per-component spawn atoms (parameter names mirror
    # ~/Documents/aic/aic_bringup/launch/spawn_task_board.launch.py 1:1).
    "spawn_task_board_base": {
        "description": "Spawn the task board base prim at /World/TaskBoard with the given pose. Mirrors spawn_task_board.launch.py task_board_x/y/z/roll/pitch/yaw arguments. Idempotent: removes prior /World/TaskBoard before recreate.",
        "parameters": {
            "x": {"type": "float", "description": "Base X position (meters). Default 0.25 per spawn_task_board.launch.py."},
            "y": {"type": "float", "description": "Base Y position (meters). Default 0.0."},
            "z": {"type": "float", "description": "Base Z position (meters). Default 1.14."},
            "roll": {"type": "float", "description": "Base roll (radians). Default 0.0."},
            "pitch": {"type": "float", "description": "Base pitch (radians). Default 0.0."},
            "yaw": {"type": "float", "description": "Base yaw (radians). Default 0.0."},
        },
    },
    "spawn_lc_mount_rail": {
        "description": "Spawn an LC Mount Rail (index 0 or 1) under /World/TaskBoard. Mirrors spawn_task_board.launch.py lc_mount_rail_<index>_present/translation/roll/pitch/yaw.",
        "parameters": {
            "index": {"type": "integer", "description": "Rail index (0 = left, 1 = right). Required."},
            "present": {"type": "boolean", "description": "Whether the rail is present. Default false (matches launch.py)."},
            "translation": {"type": "float", "description": "Translation along rail Y axis (meters, valid range [-0.09625, 0.09625]). Default 0.0."},
            "roll": {"type": "float", "description": "Rail roll (radians). Default 0.0."},
            "pitch": {"type": "float", "description": "Rail pitch (radians). Default 0.0."},
            "yaw": {"type": "float", "description": "Rail yaw (radians). Default 0.0."},
        },
    },
    "spawn_sfp_mount_rail": {
        "description": "Spawn an SFP Mount Rail (index 0 or 1) under /World/TaskBoard. Mirrors spawn_task_board.launch.py sfp_mount_rail_<index>_*.",
        "parameters": {
            "index": {"type": "integer", "description": "Rail index (0 = left, 1 = right). Required."},
            "present": {"type": "boolean", "description": "Whether the rail is present. Default false."},
            "translation": {"type": "float", "description": "Translation along rail Y axis. Default 0.0."},
            "roll": {"type": "float", "description": "Rail roll. Default 0.0."},
            "pitch": {"type": "float", "description": "Rail pitch. Default 0.0."},
            "yaw": {"type": "float", "description": "Rail yaw. Default 0.0."},
        },
    },
    "spawn_sc_mount_rail": {
        "description": "Spawn an SC Mount Rail (index 0 or 1) under /World/TaskBoard. Mirrors spawn_task_board.launch.py sc_mount_rail_<index>_*.",
        "parameters": {
            "index": {"type": "integer", "description": "Rail index (0 = left, 1 = right). Required."},
            "present": {"type": "boolean", "description": "Whether the rail is present. Default false."},
            "translation": {"type": "float", "description": "Translation along rail Y axis. Default 0.0."},
            "roll": {"type": "float", "description": "Rail roll. Default 0.0."},
            "pitch": {"type": "float", "description": "Rail pitch. Default 0.0."},
            "yaw": {"type": "float", "description": "Rail yaw. Default 0.0."},
        },
    },
    "spawn_sc_port": {
        "description": "Spawn an SC Port (index 0 or 1) under /World/TaskBoard. Mirrors spawn_task_board.launch.py sc_port_<index>_present/translation/roll/pitch/yaw.",
        "parameters": {
            "index": {"type": "integer", "description": "Port index (0 or 1). Required."},
            "present": {"type": "boolean", "description": "Whether the port is present. Default false."},
            "translation": {"type": "float", "description": "Translation along rail. Default 0.0."},
            "roll": {"type": "float", "description": "Roll (radians). Default 0.0."},
            "pitch": {"type": "float", "description": "Pitch (radians). Default 0.0."},
            "yaw": {"type": "float", "description": "Yaw (radians). Default 0.0."},
        },
    },
    "spawn_nic_card_mount": {
        "description": "Spawn a NIC Card Mount (index 0..4) under /World/TaskBoard. Mirrors spawn_task_board.launch.py nic_card_mount_<index>_present/translation/roll/pitch/yaw — note 5 indices (0..4), not 4.",
        "parameters": {
            "index": {"type": "integer", "description": "Mount index (0..4). Required."},
            "present": {"type": "boolean", "description": "Whether the mount is present. Default false."},
            "translation": {"type": "float", "description": "Translation along rail. Default 0.0."},
            "roll": {"type": "float", "description": "Roll. Default 0.0."},
            "pitch": {"type": "float", "description": "Pitch. Default 0.0."},
            "yaw": {"type": "float", "description": "Yaw. Default 0.0."},
        },
    },
    "spawn_nic_card": {
        "description": "Spawn a standalone NIC Card prim under /World/TaskBoard. spawn_task_board.launch.py has no nic_card_* family (only nic_card_mount_*); this atom mirrors the legacy AIC_OBJECTS['nic_card'] entry for parity with prior add_objects behavior.",
        "parameters": {
            "present": {"type": "boolean", "description": "Whether the NIC card is present. Default false."},
            "translation": {"type": "float", "description": "Translation along rail. Default 0.0."},
            "roll": {"type": "float", "description": "Roll. Default 0.0."},
            "pitch": {"type": "float", "description": "Pitch. Default 0.0."},
            "yaw": {"type": "float", "description": "Yaw. Default 0.0."},
        },
    },
    "delete_objects": {
        "description": "Delete all objects from /World/Objects.",
        "parameters": {}
    },
    "randomize_object_poses": {
        "description": "Randomize AIC task board object positions using domain randomization (board +/-5mm, SC ports +/-5mm to +20mm in X, NIC card snapped to 40mm Y steps).",
        "parameters": {}
    },
    "randomize_single_object": {
        "description": "Randomize the position of a single named AIC object, keeping all other objects fixed.",
        "parameters": {
            "object_name": {
                "type": "string",
                "description": "Name of the object to randomize (e.g. 'sc_port_1', 'nic_card')"
            }
        }
    },
    "save_scene_state": {
        "description": "Save current scene object poses to a JSON file.\n\nReturns:\n    result: \"success\" or \"failure\"\n    file_path: path to the saved JSON file",
        "parameters": {
            "json_file_path": {
                "type": "string",
                "description": "Optional filename for the saved JSON. If omitted, a timestamped name is generated."
            }
        }
    },
    "restore_scene_state": {
        "description": "Restore scene object poses from a previously saved JSON file.\n\nReturns:\n    result: \"success\" or \"failure\"",
        "parameters": {
            "json_file_path": {
                "type": "string",
                "description": "Optional filename of the JSON to restore. If omitted, the latest timestamped save is used."
            }
        }
    },
    "new_stage": {
        "description": "Clear the entire USD stage and free accumulated memory. The scene will be empty afterwards -- use quick_start to rebuild it.",
        "parameters": {}
    },
    "quick_start": {
        "description": "One-click scene setup: loads AIC scene with enclosure, imports UR5e robot with action graphs, force publisher, wrist cameras, task board objects, and starts simulation. Set ground_truth=False to skip /scoring/* publishers (M2 pose-source-swap surface).",
        "parameters": {
            "ground_truth": {
                "type": "boolean",
                "description": "If True (default), starts /scoring/tf + /objects_poses_real + /scoring/insertion_event publishers. If False, only parity publishers run (robot frames + /joint_states + /tf only). Mirrors Gazebo's ground_truth:=true/false launch arg semantics."
            }
        }
    },
    "randomize_lighting": {
        "description": "Randomize dome light intensity (1500-3500 lux) and color (RGB 0.5-1.0) for domain randomization.",
        "parameters": {}
    },
    "run_policy": {
        "description": "Load and run a trained policy checkpoint. (Not yet implemented -- Phase 4 placeholder.)",
        "parameters": {
            "checkpoint_path": {
                "type": "string",
                "description": "Path to the trained policy checkpoint file."
            }
        }
    },
}

# Handler method names for each tool (maps to self._cmd_<name> methods)
MCP_HANDLERS = {
    "execute_python_code": "_cmd_execute_python_code",
    "play_scene": "_cmd_play_scene",
    "stop_scene": "_cmd_stop_scene",
    "load_scene": "_cmd_load_scene",
    "load_robot": "_cmd_load_robot",
    "setup_action_graph": "_cmd_setup_action_graph",
    "setup_force_publisher": "_cmd_setup_force_publisher",
    "setup_tf_publisher": "_cmd_setup_tf_publisher",
    "setup_joint_state_publisher": "_cmd_setup_joint_state_publisher",
    # Phase 2 — controller-loop atoms (Plan 02-02 skeleton)
    "setup_controller_subscribers": "_cmd_setup_controller_subscribers",
    "setup_offlimit_contacts": "_cmd_setup_offlimit_contacts",
    "attach_cable_to_gripper": "_cmd_attach_cable_to_gripper",
    # Phase 4 — trial loader (Plan 04-02; TRIAL-01/02)
    "load_trial": "_cmd_load_trial",
    # Robotiq Hand-E 1-DOF runtime control (paired with prismatic gripper authoring
    # in build_cable_variant_usds.py::author_gripper_prismatic_joints)
    "gripper_command": "_cmd_gripper_command",
    "setup_wrist_cameras": "_cmd_setup_wrist_cameras",
    "spawn_wrist_camera": "_cmd_spawn_wrist_camera",
    "remove_wrist_camera": "_cmd_remove_wrist_camera",
    "start_wrist_camera_stream": "_cmd_start_wrist_camera_stream",
    "stop_wrist_camera_stream": "_cmd_stop_wrist_camera_stream",
    "add_objects": "_cmd_add_objects",
    # SCENE-01 / DX-02 — Per-component spawn atoms
    "spawn_task_board_base": "_cmd_spawn_task_board_base",
    "spawn_lc_mount_rail": "_cmd_spawn_lc_mount_rail",
    "spawn_sfp_mount_rail": "_cmd_spawn_sfp_mount_rail",
    "spawn_sc_mount_rail": "_cmd_spawn_sc_mount_rail",
    "spawn_sc_port": "_cmd_spawn_sc_port",
    "spawn_nic_card_mount": "_cmd_spawn_nic_card_mount",
    "spawn_nic_card": "_cmd_spawn_nic_card",
    "delete_objects": "_cmd_delete_objects",
    "randomize_object_poses": "_cmd_randomize_object_poses",
    "randomize_single_object": "_cmd_randomize_single_object",
    "save_scene_state": "_cmd_save_scene_state",
    "restore_scene_state": "_cmd_restore_scene_state",
    "new_stage": "_cmd_new_stage",
    "quick_start": "_cmd_quick_start",
    "randomize_lighting": "_cmd_randomize_lighting",
    "run_policy": "_cmd_run_policy",
}

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation as ArticulationView
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.prims import SingleArticulation as Articulation
import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf, Sdf, Usd, UsdPhysics, UsdShade, PhysxSchema
from scipy.spatial.transform import Rotation as R


class LogRedirector:
    """Redirects stdout/stderr to both the original stream and a UI callback."""
    def __init__(self, original, callback, throttle_seconds=0.0):
        self._original = original
        self._callback = callback
        self._throttle = throttle_seconds
        self._last_msgs = {}  # dedup cache: msg -> timestamp

    def write(self, text):
        self._original.write(text)
        stripped = text.strip()
        if not stripped:
            return
        if self._throttle > 0:
            import time as _t
            now = _t.monotonic()
            if stripped in self._last_msgs and now - self._last_msgs[stripped] < self._throttle:
                return
            self._last_msgs[stripped] = now
            if len(self._last_msgs) > 200:
                cutoff = now - self._throttle * 3
                self._last_msgs = {k: v for k, v in self._last_msgs.items() if v > cutoff}
        self._callback(text.rstrip("\n"))

    def flush(self):
        self._original.flush()


class DigitalTwin(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[AIC-DT] Digital Twin startup")

        self._main_loop = asyncio.get_event_loop()
        self._timeline = omni.timeline.get_timeline_interface()
        self._robot_view = None
        self._articulation = None
        self._viewport_rendering_enabled = True
        self._viewport_toggle_btn = None

        # Force publisher physics callback state
        self._force_physx_sub = None
        self._effort_articulation = None
        self._force_graph_path = None
        self._force_pub_node = None
        self._force_warmup = 0
        # PARITY-05 wrench-rootcause: cache of body_names.index("ati_tool_link")
        # so we read the correct articulation row each tick (not [-1]).
        self._wrench_body_idx = None

        # AIC parity publisher (rclpy-based /joint_states + /tf + /tf_static).
        # Replaces OGN-based publishers from Plan 06; the OGN nodes lacked
        # frame_id / joint-name / TF slashed-frame inputs needed for AIC
        # topic parity. See exts/aic-dt/aic_dt/parity_publishers.py.
        self._aic_parity_publishers = None

        # AIC controller loop (rclpy-based command subscribers + state/contacts publishers).
        # Phase 2 Plan 02-02 skeleton; Plans 02-03..06 fill in callback bodies.
        # See exts/aic-dt/aic_dt/controller_loop.py.
        self._aic_controller_loop = None  # Phase 2 — controller loop manager (Plan 02-02)

        # MCP socket server state
        self._mcp_socket = None
        self._mcp_server_thread = None
        self._mcp_server_running = False
        self._mcp_client_threads = []

        # Persistent Python execution sessions (in-memory, keyed by session_id)
        self._python_sessions = {}

        # Object state
        self._hidden_objects = {}  # {name: {ref_path, body_pose}}
        self._initial_orientations = {}  # cached from initial add_objects for randomization

        # Output directory pushed by MCP server on connect (None = use default)
        self._output_dir = None

        # Physics scene settings
        self._min_frame_rate = 60
        self._time_steps_per_second = 120

        # UR5e joint drive parameters (from Isaac Lab config)
        self._ur5e_stiffness = 2000.0
        self._ur5e_damping = 100.0
        self._ur5e_max_force = 87.0

        # UR5e joint home positions (radians)
        self._joint_home = [0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]

        # UR5e joint names in kinematic chain order
        self._joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Robot prim path (USD reference root)
        self._robot_prim_path = "/World/UR5e"

        # Articulation root prim path (the actual joints live under this — used by
        # downstream JointState publisher targetPrim relationships in Plan 06)
        self._articulation_root_prim_path = "/World/UR5e/aic_unified_robot"

        # Robot base pose — Gazebo canonical from aic_gz_bringup.launch.py
        # (robot_x=-0.2, robot_y=0.2, robot_z=1.14, robot_yaw=-3.141).
        # Z=1.14 sits the UR5e base on top of the task-board mount surface
        # whose world Z is set by the enclosure authoring (load_scene).
        # Pre-2026-05-13 this was (-0.18, -0.122, 0.0), which is the old
        # Isaac-Sim-local frame and buries the robot below the new
        # source-true enclosure floor. See CLAUDE.md "Scene-frame parity"
        # for the multi-entity reconciliation rule.
        self._robot_position = (-0.2, 0.2, 1.14)
        self._robot_yaw = -3.141

        # AIC enclosure — at world origin per aic.sdf (no <pose> on Enclosure <include>).
        # Direct-GLB-reference wrapper (enclosure_visual.glb + enclosurewalls_visual.glb
        # + light_visual.glb). Preserves per-panel GLB materials/textures —
        # the prior scene/aic.usd pre-bake collapsed them to a single
        # /World/Looks/visual_material UsdPreviewSurface (material parity gap).
        self._enclosure_usd = "Enclosure/enclosure.usda"
        self._enclosure_position = (0.0, 0.0, 0.0)

        # Ground plane Z — Floor model at world origin in aic.sdf.
        self._ground_plane_z = 0.0

        # Common physics material (applied to all objects)
        self._object_dynamic_friction = 0.6
        self._object_static_friction = 0.5
        self._object_restitution = 0.0
        self._object_friction_combine_mode = "max"
        self._object_restitution_combine_mode = "min"

        # Collision settings
        self._sdf_resolution = 300
        self._contact_offset = 0.003

        # Object collision/physics settings
        self._collision_approximation = "convexDecomposition"
        self._rest_offset = -0.0005
        self._angular_damping = 30.0

        # Isaac Sim handles ROS2 initialization automatically through its bridge
        print("ROS2 bridge will be initialized by Isaac Sim when needed")

        # Create the window UI
        self._window = ScrollingWindow(title="AIC Digital Twin", width=300, height=800)
        with self._window.frame:
            with ui.VStack(spacing=5):
                self.create_ui()

        # Start MCP socket server
        self._start_mcp_server()

    def create_ui(self):
        with ui.VStack(spacing=5):
            # 1. Simulation Setup
            with ui.CollapsableFrame(title="Simulation Setup", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    with ui.HStack(spacing=5):
                        ui.Button("Load Scene", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_scene()))
                        self._viewport_toggle_btn = ui.Button("Disable Viewport", width=140, height=35, clicked_fn=self._toggle_viewport_rendering)
                        ui.Button("Quick Start", width=120, height=35, clicked_fn=lambda: asyncio.ensure_future(self.quick_start()))

            # 2. AIC Scene
            with ui.CollapsableFrame(title="AIC Scene", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    with ui.HStack(spacing=5):
                        ui.Button("Import AIC Enclosure", width=200, height=35, clicked_fn=self.import_enclosure)

            # 3. UR5e Robot Control
            with ui.CollapsableFrame(title="UR5e Robot Control", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    with ui.HStack(spacing=5):
                        ui.Button("Import UR5e", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_robot()))
                        ui.Button("Setup Action Graph", width=200, height=35, clicked_fn=lambda: asyncio.ensure_future(self.setup_action_graph()))
                        ui.Button("Setup Force Publisher", width=200, height=35, clicked_fn=self.setup_force_publish_action_graph)
                        ui.Button("Setup TF Publisher", width=200, height=35, clicked_fn=self.setup_tf_publish_action_graph)
                        ui.Button("Setup JointState Publisher", width=200, height=35, clicked_fn=self.setup_joint_state_publish_action_graph)
                        # Phase 2 — controller-loop atoms (Plan 02-02 skeleton; Plans 02-03..06 fill callbacks)
                        ui.Button("Setup Controller Subscribers", width=220, height=35,
                                  clicked_fn=lambda: self._cmd_setup_controller_subscribers())
                        ui.Button("Setup Off-Limit Contacts", width=220, height=35,
                                  clicked_fn=lambda: self._cmd_setup_offlimit_contacts())
                        ui.Button("Attach Cable to Gripper", width=220, height=35,
                                  clicked_fn=lambda: self._cmd_attach_cable_to_gripper())
                        # Hand-E 1-DOF gripper width — default 0.00655 m per URDF
                        ui.Button("Gripper Command default 0.00655", width=220, height=35,
                                  clicked_fn=lambda: self._cmd_gripper_command(position=0.00655))
                        # Phase 4 / Plan 04-02 — TRIAL-01/02 trial loader (default trial_1, ground_truth=True)
                        ui.Button("Load Trial sample_config trial_1", width=320, height=35,
                                  clicked_fn=lambda: asyncio.ensure_future(
                                      self._cmd_load_trial(trial_key="trial_1", ground_truth=True)))

            # 4. Cameras
            with ui.CollapsableFrame(title="Cameras", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    # Wrist cameras — ComboBox-driven per-camera lifecycle.
                    # ComboBox is populated from WRIST_CAMERAS.keys(); adding
                    # a 4th/5th camera to the dict auto-extends this UI.
                    with ui.CollapsableFrame(title="Wrist Cameras", collapsed=False, height=0):
                        with ui.VStack(spacing=5, height=0):
                            _wrist_names = list(WRIST_CAMERAS.keys())
                            with ui.HStack(spacing=5):
                                ui.Label("Camera:", alignment=ui.Alignment.LEFT, width=80)
                                self._wrist_camera_combo = ui.ComboBox(0, *_wrist_names, width=200)
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn Camera Prim", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_wrist_camera(self._selected_wrist_camera_name()))
                                ui.Button("Remove Camera Prim", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_remove_wrist_camera(self._selected_wrist_camera_name()))
                            with ui.HStack(spacing=5):
                                ui.Button("Start ROS Stream", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_start_wrist_camera_stream(self._selected_wrist_camera_name()))
                                ui.Button("Stop ROS Stream", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_stop_wrist_camera_stream(self._selected_wrist_camera_name()))

                    # Additional Camera
                    with ui.CollapsableFrame(title="Additional Camera", collapsed=True, height=0):
                        with ui.VStack(spacing=5, height=0):
                            ui.Label("Camera Configuration", alignment=ui.Alignment.LEFT)

                            with ui.VStack(spacing=5):
                                with ui.HStack(spacing=5):
                                    self._workspace_checkbox = ui.CheckBox(width=20)
                                    self._workspace_checkbox.model.set_value(False)
                                    ui.Label("Workspace Camera", alignment=ui.Alignment.LEFT, width=120)

                                with ui.HStack(spacing=5):
                                    self._custom_checkbox = ui.CheckBox(width=20)
                                    ui.Label("Custom Camera", alignment=ui.Alignment.LEFT, width=120)

                            with ui.HStack(spacing=5):
                                ui.Label("Camera Prim Path:", alignment=ui.Alignment.LEFT, width=120)
                                self._custom_camera_prim_field = ui.StringField(width=200)
                                self._custom_camera_prim_field.model.set_value("/World/custom_camera")

                            with ui.HStack(spacing=5):
                                ui.Label("ROS2 Topic Name:", alignment=ui.Alignment.LEFT, width=120)
                                self._custom_camera_topic_field = ui.StringField(width=200)
                                self._custom_camera_topic_field.model.set_value("custom_camera")

                            with ui.HStack(spacing=5):
                                ui.Label("Resolution:", alignment=ui.Alignment.LEFT, width=120)
                                self._resolution_combo = ui.ComboBox(0, "640x480", "1280x720", "1920x1080", width=150)

                            with ui.HStack(spacing=5):
                                ui.Button("Create Camera", width=150, height=35, clicked_fn=self.create_additional_camera)
                                ui.Button("Create Action Graph", width=180, height=35, clicked_fn=self.create_additional_camera_actiongraph)

            # 5. Task Board Objects > Import
            with ui.CollapsableFrame(title="Task Board Objects", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    with ui.CollapsableFrame(title="Import", name="subFrame", collapsed=False, height=0):
                        with ui.VStack(spacing=5, height=0):
                            with ui.HStack(spacing=5):
                                ui.Button("Add All Objects", width=150, height=35, clicked_fn=self.add_objects)
                                ui.Button("Delete Objects", width=150, height=35, clicked_fn=self.delete_objects)

                    # SCENE-01 / DX-02 — Per-component spawn atoms
                    # (parameter surface mirrors spawn_task_board.launch.py 1:1)
                    with ui.CollapsableFrame(title="Spawn Atoms", name="subFrame", collapsed=True, height=0):
                        with ui.VStack(spacing=5, height=0):
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn Task Board Base", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_task_board_base())
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn LC Mount Rail 0", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_lc_mount_rail(index=0, present=True))
                                ui.Button("Spawn LC Mount Rail 1", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_lc_mount_rail(index=1, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn SFP Mount Rail 0", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sfp_mount_rail(index=0, present=True))
                                ui.Button("Spawn SFP Mount Rail 1", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sfp_mount_rail(index=1, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn SC Mount Rail 0", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sc_mount_rail(index=0, present=True))
                                ui.Button("Spawn SC Mount Rail 1", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sc_mount_rail(index=1, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn SC Port 0", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sc_port(index=0, present=True))
                                ui.Button("Spawn SC Port 1", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_sc_port(index=1, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn NIC Card Mount 0", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card_mount(index=0, present=True))
                                ui.Button("Spawn NIC Card Mount 1", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card_mount(index=1, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn NIC Card Mount 2", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card_mount(index=2, present=True))
                                ui.Button("Spawn NIC Card Mount 3", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card_mount(index=3, present=True))
                                ui.Button("Spawn NIC Card Mount 4", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card_mount(index=4, present=True))
                            with ui.HStack(spacing=5):
                                ui.Button("Spawn NIC Card", width=180, height=30,
                                          clicked_fn=lambda: self._cmd_spawn_nic_card(present=True))

                    # 6. Task Board Objects > Scene State
                    with ui.CollapsableFrame(title="Scene State", name="subFrame", collapsed=True, height=0):
                        with ui.VStack(spacing=5, height=0):
                            with ui.HStack(spacing=5):
                                ui.Button("Randomize Poses", width=150, height=35, clicked_fn=self.randomize_object_poses)
                            with ui.HStack(spacing=5):
                                ui.Button("Save State", width=120, height=35,
                                    clicked_fn=lambda: self._cmd_save_scene_state())
                                ui.Button("Restore State", width=120, height=35,
                                    clicked_fn=lambda: self._cmd_restore_scene_state())
                            with ui.CollapsableFrame(title="Exclude / Include Objects", collapsed=True, height=0):
                                self._visibility_frame = ui.Frame(build_fn=self._build_visibility_ui)

            # 7. Logs
            with ui.CollapsableFrame(title="Logs", collapsed=True, height=0):
                with ui.VStack(spacing=3, height=0):
                    self._log_tab_style = {
                        "RadioButton": {
                            "margin": 2, "border_radius": 2, "font_size": 16,
                            "background_color": 0xFF212121, "color": 0xFF444444,
                        },
                        "RadioButton.Label": {"font_size": 16, "color": 0xFF777777},
                        "RadioButton:checked": {"background_color": 0xFF3A3A3A, "color": 0xFF222222},
                        "RadioButton.Label:checked": {"color": 0xFFCCCCCC},
                    }
                    self._log_tab_collection = ui.RadioCollection()
                    with ui.HStack(spacing=3, height=30, style=self._log_tab_style):
                        ui.RadioButton(text="Extension", width=80,
                            radio_collection=self._log_tab_collection,
                            clicked_fn=lambda: self._switch_log_tab("extension"))
                        ui.RadioButton(text="System", width=80,
                            radio_collection=self._log_tab_collection,
                            clicked_fn=lambda: self._switch_log_tab("system"))
                    self._log_stack = ui.ZStack(height=100)
                    with self._log_stack:
                        self._ext_log_scroll = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            style={"background_color": 0xFF24211F},
                        )
                        with self._ext_log_scroll:
                            self._ext_log_vstack = ui.VStack(spacing=0)
                        self._sys_log_scroll = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            visible=False,
                            style={"background_color": 0xFF24211F},
                        )
                        with self._sys_log_scroll:
                            self._sys_log_vstack = ui.VStack(spacing=0)
                    with ui.HStack(spacing=5):
                        ui.Button("Copy Logs", width=120, height=30, clicked_fn=self._copy_logs)
                        ui.Button("Clear Logs", width=120, height=30, clicked_fn=self._clear_logs)

        self._active_log_tab = "extension"
        self._ext_log_lines = []
        self._sys_log_lines = []
        self._setup_log_redirect()

        # Re-subscribe physics callbacks if action graphs already exist (e.g. after hot-reload)
        self._resubscribe_physics_callbacks()

    # ==================== Log Redirect ====================

    def _setup_log_redirect(self):
        if isinstance(sys.stdout, LogRedirector):
            sys.stdout = sys.stdout._original
        if isinstance(sys.stderr, LogRedirector):
            sys.stderr = sys.stderr._original
        self._orig_stdout = sys.stdout
        self._orig_stderr = sys.stderr
        sys.stdout = LogRedirector(self._orig_stdout, self._append_ext_log)
        sys.stderr = LogRedirector(self._orig_stderr, lambda t: self._append_ext_log(f"[ERROR] {t}"), throttle_seconds=2.0)
        logging = carb.logging.acquire_logging()
        self._carb_log_sub = logging.add_logger(self._on_carb_log)

    def _on_carb_log(self, source, level, filename, linenum, msg):
        import time as _t
        now = _t.monotonic()
        key = (source, level, msg)
        last = getattr(self, '_carb_log_last', {})
        if key in last and now - last[key] < 2.0:
            return
        last[key] = now
        if len(last) > 200:
            cutoff = now - 5.0
            last = {k: v for k, v in last.items() if v > cutoff}
        self._carb_log_last = last

        if level >= carb.logging.LEVEL_ERROR:
            self._append_sys_log(f"[ERROR] [{source}] {msg}")
        elif level == carb.logging.LEVEL_WARN:
            self._append_sys_log(f"[WARN] [{source}] {msg}")

    def _teardown_log_redirect(self):
        sys.stdout = self._orig_stdout
        sys.stderr = self._orig_stderr
        if hasattr(self, "_carb_log_sub") and self._carb_log_sub is not None:
            logging = carb.logging.acquire_logging()
            logging.remove_logger(self._carb_log_sub)
            self._carb_log_sub = None

    def _log_color(self, text):
        if "[ERROR]" in text or "[FATAL]" in text:
            return 0xFFAAB1F6
        elif "[WARN]" in text:
            return 0xFF4ACBDF
        return 0xFFFFFFFF

    def _add_log_label(self, vstack, scroll, text):
        with vstack:
            ui.Label(text, alignment=ui.Alignment.LEFT_TOP, word_wrap=True,
                     style={"color": self._log_color(text), "font_size": 13, "margin": 0},
                     height=0)
        scroll.scroll_y = scroll.scroll_y_max + 100

    def _rebuild_log_vstack(self, vstack, lines):
        vstack.clear()
        with vstack:
            for line in lines:
                ui.Label(line, alignment=ui.Alignment.LEFT_TOP, word_wrap=True,
                         style={"color": self._log_color(line), "font_size": 13, "margin": 0},
                         height=0)

    def _run_on_main_thread(self, fn):
        if threading.current_thread() is threading.main_thread():
            fn()
        else:
            self._main_loop.call_soon_threadsafe(fn)

    def _append_ext_log(self, text):
        self._ext_log_lines.append(text)
        if len(self._ext_log_lines) > 500:
            self._ext_log_lines = self._ext_log_lines[-500:]
            self._run_on_main_thread(lambda: self._rebuild_log_vstack(self._ext_log_vstack, self._ext_log_lines))
            return
        if hasattr(self, "_ext_log_vstack") and self._ext_log_vstack:
            self._run_on_main_thread(lambda: self._add_log_label(self._ext_log_vstack, self._ext_log_scroll, text))

    def _append_sys_log(self, text):
        self._sys_log_lines.append(text)
        if len(self._sys_log_lines) > 500:
            self._sys_log_lines = self._sys_log_lines[-500:]
            self._run_on_main_thread(lambda: self._rebuild_log_vstack(self._sys_log_vstack, self._sys_log_lines))
            return
        if hasattr(self, "_sys_log_vstack") and self._sys_log_vstack:
            self._run_on_main_thread(lambda: self._add_log_label(self._sys_log_vstack, self._sys_log_scroll, text))

    def _switch_log_tab(self, tab):
        self._active_log_tab = tab
        is_ext = tab == "extension"
        self._ext_log_scroll.visible = is_ext
        self._sys_log_scroll.visible = not is_ext

    def _copy_logs(self):
        lines = self._ext_log_lines if self._active_log_tab == "extension" else self._sys_log_lines
        try:
            import subprocess
            text = "\n".join(lines)
            proc = subprocess.Popen(["xclip", "-selection", "clipboard"], stdin=subprocess.PIPE)
            proc.communicate(text.encode())
        except Exception:
            import omni.kit.clipboard
            omni.kit.clipboard.copy("\n".join(lines))

    def _clear_logs(self):
        if self._active_log_tab == "extension":
            self._ext_log_lines.clear()
            self._ext_log_vstack.clear()
        else:
            self._sys_log_lines.clear()
            self._sys_log_vstack.clear()

    # ==================== Physics Callback Utilities ====================

    def _resubscribe_physics_callbacks(self):
        """Re-subscribe physics callbacks for graphs that survived a hot-reload."""
        import omni.physx

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        # Force publisher action graph
        force_graph = "/Graph/ActionGraph_UR5e_ForcePublish"
        if stage.GetPrimAtPath(force_graph):
            self._force_graph_path = force_graph
            pub_prim = stage.GetPrimAtPath(f"{force_graph}/publisher")
            if pub_prim:
                self._force_pub_node = og.get_node_by_path(f"{force_graph}/publisher")
            self._force_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step_force
            )
            print(f"[HotReload] Re-subscribed force publisher physics callback for {force_graph}")

    def _stop_physics_callback(self, sub_attr, artic_attr, active_attr, warmup_attr):
        """Stop a physics callback and clean up its resources."""
        if hasattr(self, sub_attr) and getattr(self, sub_attr) is not None:
            setattr(self, sub_attr, None)
        if hasattr(self, artic_attr):
            setattr(self, artic_attr, None)
        if hasattr(self, active_attr):
            setattr(self, active_attr, False)
        if hasattr(self, warmup_attr):
            setattr(self, warmup_attr, 0)

    def _lazy_init_articulation(self, artic_attr, prim_path, name_prefix, warmup_attr):
        """Lazy-init an ArticulationView, returning it or None if not ready.

        Mirrors parity_publishers.py:382-398 wait-then-construct pattern.
        Construction MUST happen AFTER the first ~30 physics ticks; an
        Articulation initialized before any physics step gets a degenerate
        handle whose `_physics_view` never populates, even on subsequent
        ticks. Reusing a pre-existing _robot_view from load_robot triggers
        the same trap. Discipline: caller starts warmup=30 + artic=None,
        and on _physics_view miss we NULL the handle so the next attempt
        constructs fresh against a stage that's now physics-active.
        """
        warmup = getattr(self, warmup_attr)
        if warmup > 0:
            setattr(self, warmup_attr, warmup - 1)
            return None

        artic = getattr(self, artic_attr)
        if artic is None:
            try:
                import time as _t
                from isaacsim.core.prims import Articulation as _AV
                artic = _AV(prim_paths_expr=prim_path, name=f"{name_prefix}_{int(_t.time()*1000)}")
                artic.initialize()
                setattr(self, artic_attr, artic)
            except Exception:
                setattr(self, artic_attr, None)
                setattr(self, warmup_attr, 30)
                return None

        if not artic.is_physics_handle_valid():
            setattr(self, artic_attr, None)
            setattr(self, warmup_attr, 30)
            return None

        # _physics_view is populated when initialize() is called AFTER physics
        # has been stepping. If we're holding a pre-physics-step handle
        # (e.g. _robot_view created in load_robot before reset_async returned),
        # it stays None forever — we can't fix it on this instance. NULL it
        # so the next call reconstructs fresh against a stage that's now
        # physics-active. Set warmup=30 so reconstruction happens after a
        # margin of further physics ticks (matches parity_publishers.py).
        if not (hasattr(artic, "_physics_view") and artic._physics_view is not None):
            setattr(self, artic_attr, None)
            setattr(self, warmup_attr, 30)
            return None

        return artic

    # ==================== Scene Setup ====================

    async def load_scene(self):
        stage = omni.usd.get_context().get_stage()

        # Only initialize simulation context if physics scene doesn't exist yet
        physics_scene_prim = stage.GetPrimAtPath("/physicsScene")
        if not physics_scene_prim or not physics_scene_prim.IsValid():
            world = World()
            await world.initialize_simulation_context_async()
            print("Initialized simulation context and physics scene")
        else:
            world = World()
            print("Physics scene already exists, skipping initialization")

        # Check for ground plane on stage and add if missing
        ground_prim = stage.GetPrimAtPath("/World/defaultGroundPlane")
        if ground_prim and ground_prim.IsValid():
            print("Ground plane already exists, skipping")
        else:
            from isaacsim.core.api.objects import GroundPlane
            GroundPlane(prim_path="/World/defaultGroundPlane", z_position=self._ground_plane_z, size=5000)
            print(f"Added ground plane at z={self._ground_plane_z}")

        # Author Gazebo-parity lighting (replaces the single default dome).
        # Sources: ~/Documents/aic/aic_description/world/aic.sdf lines 138-186
        #   <light spot enclosure_light>  pose 0 0 2.5, intensity 0.4, range 2, cone 0.1↔2 rad
        #   <light point ceiling_01>      pose  2  2 6, intensity 2, range 9
        #   <light point ceiling_02>      pose -2 -2 6, intensity 2, range 9
        # Gazebo's <ambient>0 0 0</ambient> means NO global ambient term — so we
        # author NO DomeLight (instead of one with intensity=0, which still leaks).
        self._setup_aic_lights()

        # Set viewport background to match aic.sdf <background>0.15 0.15 0.15 1</background>
        try:
            _settings = carb.settings.get_settings()
            _settings.set("/rtx/sceneDb/ambientLightIntensity", 0.0)
            _settings.set("/rtx/raytracing/showLights", 1)
            _settings.set("/app/window/clearColor", [0.15, 0.15, 0.15, 1.0])
        except Exception as exc:
            print(f"[AIC-DT] viewport bg config skipped: {exc!r}")

        # Import AIC enclosure
        self.import_enclosure()

        # Import AIC floor (Gazebo's model://Floor — floor_visual.glb +
        # walls_visual.glb). Authored as a thin USDA wrapper referencing
        # the two GLBs locally (assets/Floor/floor.usda).
        self.import_floor()

        # Floor material overrides (runtime patches, parity with Gazebo's
        # ogre2 gltf import). Three things the split USD doesn't yet bake in:
        #   1. Plane_003 (warehouse ceiling) ships with NO material binding in
        #      the GLB → bind to Material_003.
        #   2. Plane_003 face normal points +Z; camera below sees back-face
        #      which gets culled (doubleSided=false default). Flip to true.
        #   3. Material_003 has glTF-native `emissive_factor`+`emissive_strength`
        #      that RTX's gltf importer ignores → author a UsdPreviewSurface
        #      sibling shader with `emissiveColor` set, rewire Material's
        #      `surface` output to it.
        # TODO bake these into walls_split.usdc via the offline splitter.
        self._apply_floor_material_overrides()

        # Switch viewport to "Stage Lights" mode so the /World/Lights/* prims
        # authored by `_setup_aic_lights()` actually contribute. The default
        # boot state hides scene-authored UsdLux while a rig (Default /
        # Colored Lights / Grey Studio) is active — VisibilityEdit applies
        # `visibility=invisible` overrides to every UsdLux prim in the scene
        # for the rig duration. Switching to "Stage Lights" removes those
        # overrides and re-enables /World/Lights/{enclosure_light,
        # ceiling_01, ceiling_02, ceiling_panel_rect}.
        try:
            from omni.kit.viewport.menubar.lighting.actions import _set_lighting_mode
            ok, applied, prev = _set_lighting_mode("Stage Lights", usd_context=omni.usd.get_context())
            print(f"[AIC-DT] lighting mode: Stage Lights (ok={ok}, prev={prev!r})")
        except Exception as exc:
            print(f"[AIC-DT] lighting-mode switch skipped: {exc!r}")

        # Author /World/workspace_camera (Gazebo-parity GUI vantage,
        # viewport-only — no ROS publish). Coupled to baseline so the user
        # can switch viewport to it right after load_scene.
        self._setup_workspace_camera()

        # Hide /World/defaultGroundPlane visually (Xform invisible — both
        # `geom` Mesh and `collisionPlane` Plane inherit). PhysX collision
        # is driven by CollisionAPI, not USD visibility — floor still works.
        try:
            from pxr import UsdGeom as _UsdGeom
            gp_root = stage.GetPrimAtPath("/World/defaultGroundPlane")
            if gp_root and gp_root.IsValid():
                _UsdGeom.Imageable(gp_root).MakeInvisible()
                print("[AIC-DT] /World/defaultGroundPlane → invisible (physics retained)")
        except Exception as exc:
            print(f"[AIC-DT] ground plane hide skipped: {exc!r}")


        # Set minimum frame rate
        settings = carb.settings.get_settings()
        settings.set("/persistent/simulation/minFrameRate", self._min_frame_rate)
        print(f"Set persistent/simulation/minFrameRate to {self._min_frame_rate}")

        # RTX ambient-occlusion ray length. Default 35 m means the AO sample
        # ray almost always escapes interior geometry → no crevice darkening.
        # 1.0 m was the iteratively-found sweet spot (Gazebo-vs-Isaac parity
        # audits — 8.2/10 vs default 35m=3/10, 0.3m=4.0/10, 2.0m=7.3/10).
        # Restores contact shadows at panel junctions / corner seams without
        # the broad-AO washout that the long ray produces.
        settings.set("/rtx/ambientOcclusion/rayLength", 1.0)
        settings.set("/rtx/ambientOcclusion/enabled", True)
        # Iray tonemap filmIso 100 → 81 (-0.3 EV). The default exposure
        # over-exposes whites vs Gazebo; head-to-head GPT preference test
        # picked -0.3 EV. Smaller drops feel washed-out, bigger drops crush
        # interior detail.
        settings.set("/rtx/post/tonemap/filmIso", 81.2)
        # Zero the residual scene-DB ambient (default (0.1,0.1,0.1)) so the
        # interior corners aren't flooded with gray fill. Head-to-head GPT
        # parity test preferred (0,0,0) over (0.1,0.1,0.1).
        settings.set("/rtx/sceneDb/ambientLightColor", (0.0, 0.0, 0.0))
        # Lift indirect-diffuse bounces 2→4. More bounces = more interreflection
        # = stronger tonal variation between adjacent surfaces (back wall lit
        # by reflected floor light, etc.). Closes Gazebo's "softer falloff,
        # more indirect shadowing" character that RaytracedLighting otherwise
        # under-renders at the default 2 bounces.
        settings.set("/rtx/indirectDiffuse/maxBounces", 4)
        print("[AIC-DT] RTX AO rayLength = 1.0 m, filmIso = 81.2 (-0.3 EV), "
              "sceneDb/ambientLightColor zeroed")

        # Configure physics scene
        physics_scene_prim = stage.GetPrimAtPath("/physicsScene")
        if physics_scene_prim and physics_scene_prim.IsValid():
            physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
            physx_scene_api.CreateEnableGPUDynamicsAttr().Set(False)
            physx_scene_api.CreateTimeStepsPerSecondAttr().Set(self._time_steps_per_second)
            physx_scene_api.CreateEnableCCDAttr().Set(False)
            print(f"GPU dynamics disabled, timeStepsPerSecond={self._time_steps_per_second}, CCD disabled on /physicsScene")
        else:
            print("Warning: /physicsScene not found")

        print("Scene loaded successfully.")

    def import_enclosure(self):
        """Import the AIC enclosure USD into the scene."""
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World/AIC_Enclosure"

        # Skip if already exists
        if stage.GetPrimAtPath(prim_path).IsValid():
            print(f"AIC enclosure already exists at {prim_path}, skipping")
            return

        try:
            asset_path = _local_asset(self._enclosure_usd)
        except FileNotFoundError as e:
            print(f"Warning: {e}")
            return

        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        # Wait for prim to exist
        for _ in range(10):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                break
            time.sleep(0.1)

        # Apply position
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*self._enclosure_position))
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
            print(f"AIC enclosure imported at {prim_path}, position={self._enclosure_position}")

        # No runtime material compensation — the bake script authors
        # source-true GLB values into enclosure_split.usdc (verified via
        # parse_glb_materials.py). RTX renders metallic WHITE darker than
        # ogre2 by renderer-pipeline difference, not by authoring divergence.

    def import_floor(self):
        """Import the AIC Floor USD (floor_visual.glb + walls_visual.glb).

        Mirrors `model://Floor` from `aic.sdf:200-204` — included at world
        origin with no pose override. The local USDA wrapper at
        `assets/Floor/floor.usda` references the two GLBs as Xform children.
        """
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World/AIC_Floor"
        if stage.GetPrimAtPath(prim_path).IsValid():
            print(f"AIC floor already exists at {prim_path}, skipping")
            return
        try:
            asset_path = _local_asset("Floor/floor.usda")
        except FileNotFoundError as e:
            print(f"Warning: {e}")
            return
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        for _ in range(10):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                break
            time.sleep(0.1)
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
            print(f"AIC floor imported at {prim_path}")

    def _apply_floor_material_overrides(self):
        """Runtime overrides on /World/AIC_Floor — three Gazebo-parity fixes
        the GLB→USDC splitter pipeline doesn't yet bake into walls_split.usdc.

        1. Bind Material_003 to Plane_003 (the warehouse ceiling Plane has no
           material assigned by the gltf import — would render with the default
           fallback material).
        2. Set Plane_003 doubleSided=true (the face normal points +Z; viewport
           camera at z=1.3 sees the back-face which is culled by default).
        3. Author a UsdPreviewSurface sibling shader on Material_003 with
           `emissiveColor` set, and rewire Material_003's `surface` output to
           that shader. The original glTF-native shader has emissive_factor +
           emissive_strength inputs which RTX's gltf importer doesn't read.
           Without this the warehouse ceiling is dark in Isaac while bright
           in Gazebo (which DOES read those glTF emissive attrs via ogre2).

        Idempotent — safe to call multiple times. Future work: bake into the
        offline splitter so the runtime patch becomes unnecessary.
        """
        stage = omni.usd.get_context().get_stage()
        # Splitter flattens path /WallsVisual/Plane_003/Plane_003 → /WallsVisual/Plane_003_Plane_003
        ceil_path = "/World/AIC_Floor/WallsVisual/Plane_003_Plane_003"
        mat_path = "/World/AIC_Floor/WallsVisual/Looks/Material_003"

        ceil_prim = stage.GetPrimAtPath(ceil_path)
        mat_prim = stage.GetPrimAtPath(mat_path)
        if not (ceil_prim and ceil_prim.IsValid()):
            print(f"[AIC-DT] floor-overrides skipped: {ceil_path} not found")
            return
        if not (mat_prim and mat_prim.IsValid()):
            print(f"[AIC-DT] floor-overrides skipped: {mat_path} not found")
            return

        # (1) Bind Material_003 to ceiling Plane_003
        from pxr import UsdShade
        mat = UsdShade.Material.Get(stage, mat_path)
        UsdShade.MaterialBindingAPI(ceil_prim).Bind(mat)

        # (2) doubleSided=true on the ceiling Mesh (face is normal-up; camera
        #     looks up from below at z=1.3, so without doubleSided the back
        #     side culls and we see straight through to the warehouse beyond)
        mesh = UsdGeom.Mesh(ceil_prim)
        mesh.CreateDoubleSidedAttr(True)

        # (3) Add UsdPreviewSurface emissive shader as Material_003 child.
        #     emissiveColor=(3,3,3) — modest to avoid RTX path-tracer fireflies
        #     that we saw at 625. The original glTF strength=625 was tuned for
        #     ogre2's emissive scaling, not RTX. Tune further if too dim.
        ups_path = mat_path + "/Emissive_UPS"
        existing = stage.GetPrimAtPath(ups_path)
        if existing and existing.IsValid():
            print(f"[AIC-DT] {ups_path} already authored, refreshing inputs")
        ups = UsdShade.Shader.Define(stage, ups_path)
        ups.CreateIdAttr("UsdPreviewSurface")
        ups.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set((0.8, 0.8, 0.8))
        ups.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set((3.0, 3.0, 3.0))
        ups.CreateInput("roughness",     Sdf.ValueTypeNames.Float).Set(0.5)
        ups.CreateInput("metallic",      Sdf.ValueTypeNames.Float).Set(0.0)
        ups.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(ups.ConnectableAPI(), "surface")
        print(f"[AIC-DT] floor-overrides applied: ceiling bound + doubleSided + emissive UPS")

    def _setup_workspace_camera(self):
        """Author /World/workspace_camera (Gazebo-parity GUI vantage).

        Mirrors aic.sdf:32 <camera_pose>0.95 -0.1 1.3 0 0 3.14</camera_pose>
        with USD camera frame conversion → quat(x,y,z,w)=(0.5,0.5,0.5,0.5).
        Idempotent — skips if /World/workspace_camera already exists. NO ROS
        publish (Gazebo's MinimalScene <camera_pose> is viewport-only).
        """
        stage = omni.usd.get_context().get_stage()
        ws_prim_path = "/World/workspace_camera"
        if stage.GetPrimAtPath(ws_prim_path).IsValid():
            print(f"[AIC-DT] workspace_camera already at {ws_prim_path}, skipping")
            return
        ws_position = (0.95, -0.1, 1.3)
        ws_quat_xyzw = (0.5, 0.5, 0.5, 0.5)
        # Match Gazebo MinimalScene defaults from
        # ~/Documents/aic/aic_description/world/aic.sdf:28-32
        #   <camera_clip><near>0.01</near><far>500</far></camera_clip>
        # Gazebo MinimalScene's default HFOV is M_PI/2 = 90° (verified
        # against gz-gui/src/plugins/minimal_scene/MinimalScene.hh:251 —
        # `math::Angle cameraHFOV = math::Angle(M_PI * 0.5)`). NOT the 60°
        # of a typical camera sensor. VFOV is derived from widget aspect.
        # Resolution 781×952 (portrait) matches the Gazebo 3D View widget's
        # current screenshot size so vertical FOV + framing collapse on
        # both sides. Adjust if the Gazebo widget geometry changes.
        ws_width, ws_height = 781, 952
        hfov_deg = 90.0
        vfov_deg = 2.0 * np.rad2deg(np.arctan(np.tan(np.deg2rad(hfov_deg / 2)) * ws_height / ws_width))
        camera_prim = UsdGeom.Camera.Define(stage, ws_prim_path)
        if not camera_prim:
            return
        fx = ws_width / (2 * np.tan(np.deg2rad(hfov_deg / 2)))
        fy = ws_height / (2 * np.tan(np.deg2rad(vfov_deg / 2)))
        cx, cy = ws_width * 0.5, ws_height * 0.5
        horizontal_aperture_mm = 36.0
        focal_length_mm = fx * horizontal_aperture_mm / ws_width
        vertical_aperture_mm = ws_height * focal_length_mm / fy
        cam = UsdGeom.Camera(camera_prim.GetPrim())
        cam.CreateHorizontalApertureAttr().Set(horizontal_aperture_mm)
        cam.CreateVerticalApertureAttr().Set(vertical_aperture_mm)
        cam.CreateFocalLengthAttr().Set(focal_length_mm)
        cam.CreateProjectionAttr().Set("perspective")
        # Gazebo aic.sdf <camera_clip>: near 0.01, far 500
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 500.0))
        cp = camera_prim.GetPrim()
        cp.CreateAttribute("omni:lensdistortion:model", Sdf.ValueTypeNames.String).Set("opencvPinhole")
        cp.CreateAttribute("omni:lensdistortion:opencvPinhole:imageSize", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(ws_width, ws_height))
        cp.CreateAttribute("omni:lensdistortion:opencvPinhole:fx", Sdf.ValueTypeNames.Float).Set(fx)
        cp.CreateAttribute("omni:lensdistortion:opencvPinhole:fy", Sdf.ValueTypeNames.Float).Set(fy)
        cp.CreateAttribute("omni:lensdistortion:opencvPinhole:cx", Sdf.ValueTypeNames.Float).Set(cx)
        cp.CreateAttribute("omni:lensdistortion:opencvPinhole:cy", Sdf.ValueTypeNames.Float).Set(cy)
        for attr_name in ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]:
            cp.CreateAttribute(f"omni:lensdistortion:opencvPinhole:{attr_name}", Sdf.ValueTypeNames.Float).Set(0.0)
        xform = UsdGeom.Xform(camera_prim.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*ws_position))
        quat = Gf.Quatd(ws_quat_xyzw[3], ws_quat_xyzw[0], ws_quat_xyzw[1], ws_quat_xyzw[2])
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
        print(f"[AIC-DT] workspace_camera authored at {ws_prim_path} ({ws_width}x{ws_height})")
        # Make this camera the active viewport so the user sees it right after load_scene
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp:
                vp.camera_path = ws_prim_path
                print(f"[AIC-DT] active viewport → {ws_prim_path}")
        except Exception as exc:
            print(f"[AIC-DT] viewport switch to workspace_camera skipped: {exc!r}")

    def _setup_aic_lights(self):
        """Author the 3 lights Gazebo's aic.sdf declares (lines 138-186).

        Mapping Gazebo → USD:
          - <light type="spot">  → UsdLux.SphereLight + UsdLux.ShapingAPI cone
          - <light type="point"> → UsdLux.SphereLight (omnidirectional, no shaping)

        Gazebo intensity units differ from USD/RTX. Gazebo uses small physical
        scalars (0.4 for the spot, 2 for the points) tuned for ogre2. RTX
        expects much larger values for visible illumination — we scale to
        Isaac Sim-appropriate magnitudes while preserving relative ratios
        (spot=0.2× → 500, points=1.0× → 2500). Adjust if exposure looks off.
        """
        from pxr import UsdLux, Gf, Sdf, UsdGeom
        stage = omni.usd.get_context().get_stage()
        lights_root = "/World/Lights"
        # Clear any prior lighting so we don't double up across re-runs
        for legacy in ("/World/defaultDomeLight",):
            p = stage.GetPrimAtPath(legacy)
            if p and p.IsValid():
                stage.RemovePrim(legacy)
                print(f"[AIC-DT] removed legacy {legacy}")
        UsdGeom.Xform.Define(stage, lights_root)

        # 1. Spot — enclosure_light at (0,0,2.5) pointing -Z (Gazebo spot
        # default look-dir is +X but pose RPY=0 in aic.sdf means it points
        # straight down +X... wait — Gazebo light "spot" with no rpy points
        # down -Z by convention for ogre2. We mirror that: USD spot also
        # defaults -Z. So zero-rotation is correct.
        spot_path = f"{lights_root}/enclosure_light"
        spot = UsdLux.SphereLight.Define(stage, spot_path)
        spot.CreateRadiusAttr(0.05)
        # RTX RaytracedLighting needs higher intensity than ogre2 to reach
        # the workspace through enclosure occlusion. 15000 chosen after
        # Gazebo-vs-Isaac parity audit found B much darker than A.
        spot.CreateIntensityAttr(15000.0)
        spot.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        shaping = UsdLux.ShapingAPI.Apply(spot.GetPrim())
        shaping.CreateShapingConeAngleAttr(2.0 * 180.0 / 3.14159265)  # rad → deg (Gazebo outer_angle=2)
        shaping.CreateShapingConeSoftnessAttr(0.5)
        xf = UsdGeom.Xformable(spot.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 2.5))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))

        # 2/3. Point lights — ceiling_01 at (+2,+2,6) and ceiling_02 at (-2,-2,6)
        for name, xyz in [("ceiling_01", (2.0, 2.0, 6.0)),
                          ("ceiling_02", (-2.0, -2.0, 6.0))]:
            path = f"{lights_root}/{name}"
            pt = UsdLux.SphereLight.Define(stage, path)
            pt.CreateRadiusAttr(0.1)
            pt.CreateIntensityAttr(2500.0)
            pt.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            xf = UsdGeom.Xformable(pt.GetPrim())
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))

        # 3a. DomeLight for broad ambient fill. Gazebo's reference render has
        # near-uniform diffuse fill on the back wall + floor. RTX with only
        # SphereLight + RectLight sources still produces shadow pooling.
        # A low-intensity DomeLight gives the "everywhere fill" character.
        # 3a. DomeLight at low intensity for ambient fill. v7 baseline; the
        # v8 attempt to remove it collapsed the scene's contrast — kept here.
        # Slightly warm tint (1.0, 0.96, 0.88) compensates for Gazebo's
        # "soft warm cast" observed in the reference render vs RTX's
        # cool/neutral default. Lift / warm tint are documented RTX-vs-ogre2
        # compensation, NOT a Gazebo light setting (Gazebo's lights are 1,1,1).
        dome_path = f"{lights_root}/ambient_dome"
        dome = UsdLux.DomeLight.Define(stage, dome_path)
        dome.CreateIntensityAttr(300.0)
        dome.CreateColorAttr(Gf.Vec3f(1.0, 0.91, 0.78))

        # 3b. RectLight INSIDE the enclosure — the dominant interior fill.
        # Gazebo's render shows broad diffuse overhead illumination (not the
        # point-source falloff that SphereLights produce). A RectLight just
        # below the translucent inner ceiling (z=2.5) covering the enclosure
        # footprint (≈1.4×1.4 m) emits downward into the workspace, giving
        # the same area-light character. Sits below GRAY75_CEILING so its
        # cone is unobstructed; visible through the translucent panel as the
        # "white opening above" that Gazebo's ceiling panel reveals.
        enc_rect_path = f"{lights_root}/enclosure_panel_rect"
        enc_rect = UsdLux.RectLight.Define(stage, enc_rect_path)
        enc_rect.CreateWidthAttr(1.4)
        enc_rect.CreateHeightAttr(1.4)
        enc_rect.CreateIntensityAttr(3000.0)
        enc_rect.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        xf = UsdGeom.Xformable(enc_rect.GetPrim())
        xf.ClearXformOpOrder()
        # z=2.50 (just below GRAY75_CEILING at z≈2.52); emits -Z (down) by default
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 2.50))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))

        # 4. RectLight — proxy for the Material_003 emissive ceiling panel.
        # In Gazebo's ogre2 renderer, walls_visual.glb Material_003 has
        # `emissive_strength=625` and acts as a full-ceiling area light. RTX
        # in RaytracedLighting mode renders the emissive material as bright
        # pixels but does NOT propagate light from it (only PathTracing
        # mode does). UsdLux.RectLight works in BOTH modes and avoids
        # path-tracer firefly artifacts. Position + dimensions match the
        # Plane_003 Material_003 GeomSubset bbox in walls_split.usdc:
        # corners ±4.7165, z=7.0478 → 9.433×9.433 m at z=7.048 facing -Z.
        rect_path = f"{lights_root}/ceiling_panel_rect"
        rect = UsdLux.RectLight.Define(stage, rect_path)
        rect.CreateWidthAttr(9.433)
        rect.CreateHeightAttr(9.433)
        # 6000 = v7 value (4200 was tried in v8 — produced a flatter look).
        rect.CreateIntensityAttr(6000.0)
        rect.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        # RectLight emits along -Z by default; ceiling pointing -Z (down) is
        # the no-rotation case. Translate to the panel center.
        xf = UsdGeom.Xformable(rect.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 7.048))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))

        print(f"[AIC-DT] authored 6 lights under {lights_root} "
              f"(enclosure_light spot @ 2.5m, enclosure_panel_rect @ 2.5m, "
              f"ambient_dome, 2× point @ 6m, ceiling_panel_rect @ 7.05m)")

    def _toggle_viewport_rendering(self):
        """Toggle viewport rendering on/off to free GPU resources."""
        import omni.kit.viewport.utility as vp_utils
        viewport = vp_utils.get_active_viewport()
        if viewport is None:
            print("Warning: No active viewport found")
            return
        self._viewport_rendering_enabled = not self._viewport_rendering_enabled
        viewport.updates_enabled = self._viewport_rendering_enabled
        state = "enabled" if self._viewport_rendering_enabled else "disabled"
        print(f"Viewport rendering {state}")
        if self._viewport_toggle_btn:
            self._viewport_toggle_btn.text = "Disable Viewport" if self._viewport_rendering_enabled else "Enable Viewport"

    # ==================== Quick Start ====================

    async def quick_start(self, ground_truth: bool = True):
        """Quick start: load scene → load robot → play → TF/JointState/JointSubscribe graphs → cameras → wrench → objects → workspace cam.

        Per Phase 1 D-12: refactored ordering with TF + JointState publishers (PARITY-03/04)
        inserted after the early-play step and before the joint-state subscribe graph. Future
        phases drop new atoms in via this common path:
          - Phase 2 controller-loop: subscribes between JointState publisher and Force publisher
          - Phase 3 GT-TF: between objects and workspace cam
          - Phase 4 trial-loader: replaces the static add_objects with a parametric variant

        The early-play step (2b) is non-negotiable — see comment block below.

        Args:
            ground_truth: If True (default), `_start_aic_scoring_publishers` runs
                (Phase 3 /scoring/tf + /objects_poses_real + /scoring/insertion_event).
                If False, that call is suppressed; only parity publishers run. Mirrors
                Gazebo's `ground_truth:=true/false` launch arg semantics. M2 pose-source
                swap will replace the publisher behind this gate.
        """
        print("=== Quick Start ===")
        app = omni.kit.app.get_app()

        # 1. Load scene (physics, ground plane, dome light, enclosure)
        print("--- Loading scene ---")
        await self.load_scene()
        await app.next_update_async()

        # 2. Import UR5e (load_robot internally calls timeline.stop() at the end)
        print("--- Importing UR5e ---")
        await self.load_robot()
        await app.next_update_async()

        # 2b. Play the scene EARLY — before any action graphs / cameras / objects exist.
        # Bisection on 2026-05-01 showed: doing all setup THEN play() at the end (the
        # original ordering) wedges the kit main thread because PhysX cooking + many
        # graphs + ROS2 publishers all kick off simultaneously and lock-contend.
        # Step-by-step adds with the timeline already playing remain responsive.
        print("--- Playing scene early (before graphs/objects) ---")
        self._timeline.play()
        await app.next_update_async()

        # 3a. Setup TF publisher (Plan 06 / Phase 1 D-10) — /tf + /tf_static
        print("--- Setting up TF Publisher (/tf + /tf_static) ---")
        self.setup_tf_publish_action_graph()
        await app.next_update_async()

        # 3b. Setup JointState publisher (Plan 06 / Phase 1 D-11) — /joint_states
        print("--- Setting up JointState Publisher (/joint_states) ---")
        self.setup_joint_state_publish_action_graph()
        await app.next_update_async()

        # 3b'. Phase 2 — controller-loop subscribers + state publisher + off-limit contacts
        # (PARITY-09/10/11 + PARITY-06). Single AicControllerLoop manager handles all 4
        # topics. Plans 02-03..06 fill in the callback bodies; the skeleton is no-op safe.
        print("--- Setting up AIC Controller Loop (controller subscribers + off-limit contacts) ---")
        self._start_aic_controller_loop()
        await app.next_update_async()

        # 3c. Conditional bridge to reorder /joint_states_isaac_raw → /joint_states alphabetical
        # (D-11 / PARITY-03). Plan 05 verdict was NO-WRAPPER-NEEDED (aic_adapter reorders by
        # name, not by index), so this attribute typically does not exist. The hasattr guard
        # makes the call safely a no-op in that case while leaving room for a future plan to
        # land a wrapper without re-touching quick_start.
        if hasattr(self, 'setup_joint_state_reorder'):
            print("--- Setting up JointState Reorder Bridge ---")
            self.setup_joint_state_reorder()
            await app.next_update_async()

        # 4. Setup UR5e action graph (joint-state SUBSCRIBE side for Phase 2 controller-loop)
        print("--- Setting up UR5e Action Graph ---")
        await self.setup_action_graph()
        await app.next_update_async()

        # 5. Setup wrist cameras
        print("--- Setting up Wrist Cameras ---")
        self.setup_wrist_cameras()
        await app.next_update_async()

        # 6. Setup force publisher (renamed wrench topic per Plan 04 — fts_broadcaster/wrench)
        print("--- Setting up UR5e Force Publisher ---")
        self.setup_force_publish_action_graph()
        await app.next_update_async()

        # 7. Add task board objects (uses capitalized vendored paths from Plan 04 +
        #    per-component spawn atoms from Plan 09 / SCENE-01)
        print("--- Adding task board objects ---")
        self.add_objects()
        await app.next_update_async()

        # Phase 3 Plan 03-04+05: scoring publishers (after task_board prims exist).
        # Phase 4 Plan 04-02 (D-04): ground_truth gate — when False, scoring topics
        # are suppressed (M2 pose-source-swap surface preserved).
        if ground_truth:
            print("--- Setting up AIC scoring publishers (/scoring/tf + /objects_poses_real + /scoring/insertion_event) ---")
            try:
                self._start_aic_scoring_publishers()
            except Exception as exc:
                print(f"[AIC-DT][scoring] _start_aic_scoring_publishers failed: {exc!r}")
        else:
            print("--- ground_truth=False — skipping AIC scoring publishers (M2 swap surface preserved) ---")
        await app.next_update_async()

        # 7b. Randomize lighting (best-effort — present from earlier work)
        if hasattr(self, 'randomize_lighting'):
            print("--- Randomizing Lighting ---")
            try:
                self.randomize_lighting()
            except Exception as e:
                print(f"[AIC-DT] randomize_lighting skipped: {e}")
            await app.next_update_async()

        # 8. Workspace camera now authored by load_scene (baseline). Viewport-
        # only (no ROS publish), Gazebo-parity pose. Idempotent — already in
        # stage from step 1 unless caller manipulated the stage.
        print("--- Workspace camera was authored in load_scene (viewport-only) ---")
        await app.next_update_async()

        # 10. Already playing from step 2b — no-op the original play() at the end.
        # (Calling play() again on a playing timeline is harmless but unnecessary.)
        print("--- Already playing (from step 2b) ---")

        print("=== Quick Start Complete ===")

    # ==================== Robot ====================

    async def load_robot(self,
                         robot_x: float = None, robot_y: float = None, robot_z: float = None,
                         robot_roll: float = 0.0, robot_pitch: float = 0.0, robot_yaw: float = None,
                         cable_x: float = 0.172, cable_y: float = 0.024, cable_z: float = 1.518,
                         cable_roll: float = 0.4432, cable_pitch: float = -0.48, cable_yaw: float = 1.3303,
                         cable_type: str = "sfp_sc_cable",
                         attach_cable_to_gripper: bool = False,
                         gripper_initial_pos: float = 0.00655):
        """Load the unified UR5e + Robotiq Hand-E + cable USD into /World/UR5e.

        SCENE-04: Robot base + cable poses are configurable via the same parameter
        names Gazebo uses (mirroring
        ~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py:
        `robot_x/y/z/roll/pitch/yaw` and `cable_x/y/z/roll/pitch/yaw`).

        Defaults:
          - robot_x/y/z/yaw = None → use the Gazebo-canonical base pose held
            in self._robot_position = (-0.2, 0.2, 1.14) and
            self._robot_yaw = -3.141 (from aic_gz_bringup.launch.py).
            Pre-2026-05-13 the legacy fallback was (-0.18, -0.122, 0.0)
            yaw=0, which only made sense for the old Isaac-Sim-local frame
            and buries the robot under the source-true enclosure floor.
            The parameter SURFACE still matches Gazebo verbatim; the only
            behavior change is the no-arg fallback now matches Gazebo too.
          - cable_x/y/z/roll/pitch/yaw = aic_gz_bringup.launch.py defaults
            (0.172, 0.024, 1.518, 0.4432, -0.48, 1.3303). Cable subtree is
            SetActive(False) per D-04 (cable physics is Phase 3 / SCENE-05),
            so cable pose params are wired through but produce a no-op
            visible effect in Phase 1. Phase 3 enables physics, at which
            point these params start mattering.
        """
        # Cable USD selection by cable_type (2026-05-16: two-end fidelity fix).
        # Until 2026-05-16 the only USD was aic_unified_robot_cable_sdf.usd which
        # had the LC plug at the gripper end and the SC plug at the far end.
        # CheatCode trial_3 (cable_type=sfp_sc_cable_reversed) wants the SC end
        # held by the gripper — visually wrong before the fix. exts/aic-dt/scripts/
        # build_cable_variant_usds.py produces _reversed.usd by surgically
        # swapping the connector-visual subtree transforms on the source USD;
        # see that script's docstring for the why/how/limits.
        if cable_type == "sfp_sc_cable_reversed":
            asset_path = _local_asset("robot/aic_unified_robot_cable_sdf_reversed.usd")
            print(f"[AIC-DT] cable_type=sfp_sc_cable_reversed → loading {asset_path}")
        else:
            asset_path = _local_asset("robot/aic_unified_robot_cable_sdf.usd")
            if cable_type != "sfp_sc_cable":
                print(f"[AIC-DT] Unknown cable_type {cable_type!r}; defaulting to sfp_sc_cable")
        prim_path = self._robot_prim_path

        # Ensure World exists
        world = World.instance()
        if world is None:
            print("World not initialized. Creating simulation context for existing stage...")
            world = World()
            await world.initialize_simulation_context_async()
            print("Simulation context initialized.")

        # Add the USD asset
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        # Wait for prim to exist
        stage = omni.usd.get_context().get_stage()
        for _ in range(10):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"Failed to load prim at {prim_path}")

        # Resolve robot pose: explicit kwargs override the Gazebo-canonical
        # fallback in self._robot_position + self._robot_yaw. Mix-and-match
        # is allowed (e.g. caller passes robot_z only).
        rx = self._robot_position[0] if robot_x is None else float(robot_x)
        ry = self._robot_position[1] if robot_y is None else float(robot_y)
        rz = self._robot_position[2] if robot_z is None else float(robot_z)
        ryaw = self._robot_yaw if robot_yaw is None else float(robot_yaw)

        # Apply translation + RPY rotation. AddRotateXYZOp is intrinsic XYZ
        # Euler matching Gazebo's -R -P -Y semantics — but USD's RotateXYZOp
        # values are in DEGREES while the API surface accepts RADIANS (mirrors
        # Gazebo launch arg semantics). Convert here.
        import math
        xform = UsdGeom.Xform(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(rx, ry, rz))
        # If all RPY are zero, keep the existing identity-quaternion behavior
        # (avoids any rounding drift in the no-rotation explicit-zero path).
        if (float(robot_roll), float(robot_pitch), ryaw) == (0.0, 0.0, 0.0):
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        else:
            roll_deg = math.degrees(float(robot_roll))
            pitch_deg = math.degrees(float(robot_pitch))
            yaw_deg = math.degrees(ryaw)
            xform.AddRotateXYZOp().Set(Gf.Vec3f(roll_deg, pitch_deg, yaw_deg))
        print(f"Applied translation ({rx},{ry},{rz}) RPY=({robot_roll},{robot_pitch},{ryaw}) rad to {prim_path}")

        # Cable workaround: even when aic-dt is loaded post-startup (which lets PhysX
        # cook the cable + Body_005 SDF successfully — cache grows from 0 to ~113 MB),
        # the live simulation of the 21-segment cable rope chain wedges the kit main
        # thread shortly after timeline.play(). Until cable is re-authored as a proper
        # rope (PhysxPhysicsJointInstancer + density-based mass per RigidBodyRopeDemo)
        # AND the post-play wedge is resolved, deactivate the cable subtree so the
        # rest of the scene stays responsive.
        #
        # SCENE-04: cable_x/y/z/roll/pitch/yaw pose params are wired through to
        # the cable transform (matching Gazebo's parameter surface), but the
        # subtree is SetActive(False) per D-04 — Phase 3 enables physics, at
        # which point these params start mattering. Authoring the pose now
        # means the parameter surface is in place without any additional code
        # change in Phase 3.
        cable_prim = stage.GetPrimAtPath(f"{prim_path}/cable")
        if cable_prim and cable_prim.IsValid():
            try:
                cable_xform = UsdGeom.Xform(cable_prim)
                # Only re-apply if not already at the requested pose to avoid
                # double-stacking xformOps on hot-reload.
                cable_xform.ClearXformOpOrder()
                cable_xform.AddTranslateOp().Set(Gf.Vec3d(float(cable_x), float(cable_y), float(cable_z)))
                # RotateXYZOp expects degrees; API surface mirrors Gazebo radians.
                cable_xform.AddRotateXYZOp().Set(Gf.Vec3f(
                    math.degrees(float(cable_roll)),
                    math.degrees(float(cable_pitch)),
                    math.degrees(float(cable_yaw)),
                ))
                print(f"Authored cable pose translation=({cable_x},{cable_y},{cable_z}) RPY=({cable_roll},{cable_pitch},{cable_yaw}) rad (no-op effect in Phase 1 — cable SetActive(False))")
            except Exception as exc:  # noqa: BLE001 — pose authoring on a deactivated subtree is best-effort
                print(f"Cable pose authoring skipped: {exc}")
            # Phase 3 SCENE-05 (Plan 03-02): cable physics authored on-disk via
            # author_cable_physics_offline.py — per-link MassAPI(density=0.00005)
            # + per-joint DriveAPI(force, damping=10.0, stiffness=1.0) per
            # NVIDIA's RigidBodyRopeDemo template.
            #
            # Cable activation contract (motion-fidelity-cheatcode-timing,
            # 2026-05-11 PATH A):
            #   - attach_cable_to_gripper=True  → SetActive(True). Plug rigid
            #     body must exist for scoring's PhysX contact-based
            #     /scoring/insertion_event to fire. All sample_config.yaml
            #     trials (trial_1..trial_3) set this True.
            #   - attach_cable_to_gripper=False → SetActive(False). No cable
            #     load, used for cable-free smoke / probe scenes.
            #   - SCENE_05_DISABLE=1 (env)      → SetActive(False) regardless,
            #     emergency global rollback (D-04 fallback).
            #
            # The earlier R1 descope (commit 865e9c8) hard-disabled activation
            # under the hypothesis that cable load was making sim too slow for
            # CheatCode to complete inside the 180s budget. That hypothesis is
            # FALSIFIED: the real bottleneck was a missing
            # /aic_controller/change_target_mode service stub (controller_loop.py)
            # that wedged aic_model's action thread at the first
            # handle_motion_update — sim-realtime was healthy throughout, the
            # worker was just blocked on an unfulfilled RPC. With the stub in
            # place CheatCode finishes in ~65s wall, leaving ~115s headroom
            # for cable load. Even at 0.56× sim-realtime under cable+TF load,
            # 65s × 1/0.56 ≈ 116s — within budget.
            import os as _os
            if _os.environ.get("SCENE_05_DISABLE", "").lower() in ("1", "true"):
                cable_prim.SetActive(False)
                print(f"SCENE_05_DISABLE=1 → deactivated {prim_path}/cable (emergency global rollback)")
            elif attach_cable_to_gripper:
                cable_prim.SetActive(True)
                print(f"SCENE-05: activated {prim_path}/cable (attach_cable_to_gripper=True; plug rigid body required for /scoring/insertion_event)")
            else:
                cable_prim.SetActive(False)
                print(f"SCENE-05: deactivated {prim_path}/cable (attach_cable_to_gripper=False; cable-free scene)")

        # joint-drives-urdf-reconcile: author USD DriveAPI on the 6 arm joints
        # BEFORE articulation init so PhysX picks up the canonical AIC values
        # at reset_async. Source-of-truth: aic_isaaclab/.../aic_task_env_cfg.py
        # ImplicitActuatorCfg(stiffness=2000, damping=100, effort_limit_sim=87)
        # — confirmed in exts/aic-dt/docs/aic-isaaclab-leverage.md.
        # Articulation root solver iter counts likewise: pos=16, vel=8.
        self._configure_arm_drives(stage, prim_path)

        # Setup Articulation
        self._robot_view = ArticulationView(prim_paths_expr=prim_path, name="ur5e_view")
        World.instance().scene.add(self._robot_view)
        await World.instance().reset_async()
        self._timeline.stop()

        # SCENE-03 Plan 03-03: optionally attach cable plug-end → gripper finger.
        # Done AFTER articulation setup so the gripper finger exists.
        if attach_cable_to_gripper:
            try:
                self._attach_cable_to_gripper_impl(
                    gripper_initial_pos=gripper_initial_pos,
                    cable_type=cable_type,
                )
            except Exception as exc:
                print(f"[AIC-DT] SCENE-03 attach_cable_to_gripper failed: {exc!r}")

    def _attach_cable_to_gripper_impl(self,
                                      gripper_initial_pos: float = 0.00655,
                                      plug_link_path: str = "/World/UR5e/cable/Rope/Rope/link_20",
                                      finger_link_path: str = "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l",
                                      cable_type: str = "sfp_sc_cable"):
        """SCENE-03 Plan 03-03 — attach cable plug-end to gripper finger via FixedJoint.

        Plug-end default = link_20 per Plan 03-01 topology probe (closest to
        sc_plug_visual at d=0.033m of 21 cable links). Idempotent — removes
        any prior CableAttachJoint before authoring new one.

        Returns the joint prim path on success; raises on failure.
        """
        from pxr import UsdPhysics, Sdf, Gf
        stage = omni.usd.get_context().get_stage()

        plug = stage.GetPrimAtPath(plug_link_path)
        finger = stage.GetPrimAtPath(finger_link_path)
        if not plug or not plug.IsValid():
            raise RuntimeError(f"plug-end link not found at {plug_link_path}")
        if not finger or not finger.IsValid():
            raise RuntimeError(f"gripper finger link not found at {finger_link_path}")

        # Idempotent removal of prior attach joint
        joint_path = f"{finger_link_path}/CableAttachJoint"
        existing = stage.GetPrimAtPath(joint_path)
        if existing and existing.IsValid():
            stage.RemovePrim(joint_path)
            print(f"[AIC-DT] SCENE-03 removed prior CableAttachJoint at {joint_path}")

        # Author UsdPhysics.FixedJoint with EXPLICIT localPos0 = (0,0,0) and
        # localPos1 = (0,0,0). Without explicit values, PhysX auto-snapshots
        # the current world-relative offset at first physics step — which
        # is ~70cm because the cable is spawned in its Gazebo-canonical world
        # pose but the robot is at default load_robot joint values, not yet
        # at sample_config trial home pose. The captured offset then
        # permanently anchors the plug 70cm off from the gripper so even when
        # CheatCode drives gripper precisely to the port, the physical plug
        # stays nowhere near — engine reports final plug-port distance ~1m.
        #
        # Setting localPos0=0 / localPos1=0 explicitly tells PhysX "this is
        # the constraint, no snapshotting" — body1 (link_20) is TELEPORTED
        # to coincide with body0 (finger_link_l) at the next sim step. The
        # rope chain (link_0..link_19) has near-zero mass per SCENE-05
        # (density=5e-5) so PhysX yanks link_20 to the constraint without
        # tension propagating destabilizing forces back through the chain.
        #
        # localRot0/1 left UNauthored so PhysX still snapshots the relative
        # rotation at first tick (preserves the cable's "plug points down"
        # orientation, which is pose-dependent and not knowable at authoring
        # time).
        # motion-fidelity-cheatcode-timing iter 7 (PATH (a) FULL COMMITMENT):
        # SKIP authoring CableAttachJoint (link_20 → finger). This joint is a
        # CLOSED LOOP between the cable articulation (link_20 belongs to
        # /World/UR5e/cable/Rope/Rope chain) and the UR5e articulation
        # (finger_link_l belongs to /World/UR5e/aic_unified_robot). PhysX's
        # reduced-coordinate articulation solver requires tree topology;
        # this cross-articulation joint creates a cycle. excludeFromArticulation
        # flag attempts (iter 4-5) deterministically crashed PhysX cook.
        # Even explicit localPos0/1=(0,0,0) without the flag (iter 6) wedges
        # the load_trial async pipeline: probe sample 2026-05-12T11:06Z confirms
        # load_trial hangs on cable activation + this joint authoring,
        # blocking the Kit asyncio runtime entirely (TaskStepMethWrapper
        # "Cannot enter into task" cascade in kit log).
        #
        # NEW APPROACH: drop the link_20 joint entirely. The PlugVisualAttachJoint
        # block BELOW (sc_plug_visual → finger_link_l) provides the scoring-
        # critical attachment: sc_plug_visual is the body PhysX checks for
        # plug-port contact (Issue B finding 2026-05-11). It lives at
        # /World/UR5e/cable/sc_plug_visual as a SIBLING Xform of the Rope
        # chain — NOT part of the cable articulation tree. Adding RigidBodyAPI
        # to it makes it a standalone dynamic body; FixedJoint to finger_link_l
        # is body-to-articulation, no loop topology, cooks cleanly.
        #
        # Cost: cable rope (link_0..link_20) is no longer constrained to the
        # gripper — it drops under gravity / dangles freely / drifts. This is
        # acceptable for M1 per `out_of_scope_for_m1`: cable bend dynamics are
        # decorative, scoring is contact-based on plug-end geometry. The plug
        # (sc_plug_visual) tracks the gripper via PlugVisualAttachJoint.
        # Cable USD per-link mass authoring (commit 865e9c8) is preserved
        # on disk and harmless under this approach.
        print(f"[AIC-DT] SCENE-03 SKIP CableAttachJoint authoring (iter-7 Path (a)) — "
              f"plug-visual proxy joint below handles the scoring-critical attachment "
              f"without closed-loop articulation topology.")

        # motion-fidelity-cheatcode-timing iter 6 — PATH (a) plug proxy.
        # Make sc_plug_visual a STANDALONE rigid body (not part of any
        # articulation) constrained to finger_link_l via a separate
        # FixedJoint, bypassing the 21-link cable rope dynamics for the
        # scoring contact body. The published /tf plug frame is anchored
        # on sc_plug_visual world pose (extension.py::_compute_trial_tf_frames
        # line ~3437), so CheatCode reads "plug at sc_plug_visual" and
        # commands gripper until that frame coincides with port frame.
        # Topology: sc_plug_visual is an Xform child of /World/UR5e/cable
        # (sibling of /World/UR5e/cable/Rope/Rope/link_*), NOT part of the
        # cable articulation chain. Adding RigidBodyAPI to it makes it a
        # standalone dynamic body; FixedJoint to finger_link_l constrains
        # it to the UR5e+RG2 articulation. NO closed-loop topology — this
        # is distinct from the link_20→finger CableAttachJoint above which
        # IS a closed loop between cable-articulation and ur5e-articulation
        # (iters 4-5 tried excludeFromArticulation=True on that loop joint
        # to break the cook crash; deterministically crashed PhysX). The
        # plug proxy here is a body-to-articulation constraint, no loop.
        # iter 7 PATH (a') — USD-HIERARCHY KINEMATIC PLUG PROXY.
        # Empirical findings (2026-05-12 live probe + 6 prior iters):
        #   - Both link_20 → finger_link_l (CableAttachJoint) and
        #     sc_plug_visual → finger_link_l (PlugVisualAttachJoint) wedge
        #     load_trial because they cross articulation roots
        #     (/World/UR5e/cable's articulation ↔ /World/UR5e/aic_unified_robot's
        #     articulation). PhysX's reduced-coordinate solver can't cook a
        #     loop joint across articulation roots; excludeFromArticulation
        #     flag crashes during cook (iters 4-5); explicit localPos0/1=0
        #     wedges the asyncio runtime (iter 6).
        #   - With cable active but NO cross-articulation joint, sc_plug_visual
        #     stays at its Gazebo-canonical world spawn pose (~70cm Y-offset
        #     from gripper at home), drifting only ~30mm under gripper motion
        #     because cable density=5e-5 has near-zero gravity coupling.
        #
        # SOLUTION: author a new child Xform under finger_link_l with
        # CollisionAPI + a sphere collider. USD hierarchical transform
        # composition gives this child the finger_link_l world pose every
        # frame for free — no joint, no callback, no articulation entanglement.
        # CollisionAPI on the child extends the finger_link_l rigid body's
        # collision shape (PhysX standard pattern for multi-shape articulation
        # links). The proxy is what scoring queries for plug-port contact and
        # what /scoring/insertion_event publishes. sc_plug_visual is left
        # alone (decorative, in cable subtree); cable remains active for
        # visual completeness but is not the scoring body.
        try:
            plug_proxy_path = f"{finger_link_path}/plug_proxy"
            existing_proxy = stage.GetPrimAtPath(plug_proxy_path)
            if existing_proxy and existing_proxy.IsValid():
                stage.RemovePrim(plug_proxy_path)
            # Author Xform at finger tip (zero local offset = gripper/tcp pose
            # since gripper/tcp is essentially the same as finger_link_l tip).
            # Engineering judgment: putting proxy at gripper/tcp position
            # matches CheatCode's commanded-pose convention (CheatCode drives
            # gripper/tcp to target frame). Insertion fires when proxy
            # collider overlaps port collider.
            plug_proxy_xform = UsdGeom.Xform.Define(stage, Sdf.Path(plug_proxy_path))
            # Zero translation — proxy at finger_link_l origin. The 33mm offset
            # GPT identified between gripper/tcp and sc_plug_visual was the
            # cable-as-spawned vs gripper-as-positioned offset, which is
            # irrelevant under USD-hierarchy approach (proxy IS the gripper
            # for collision purposes).
            plug_proxy_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            # Add sphere collider as child of the Xform (PhysX gathers
            # collision geometry from CollisionAPI'd descendants of the
            # rigid body).
            plug_collider_path = f"{plug_proxy_path}/collider_geom"
            existing_coll = stage.GetPrimAtPath(plug_collider_path)
            if existing_coll and existing_coll.IsValid():
                stage.RemovePrim(plug_collider_path)
            plug_sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(plug_collider_path))
            plug_sphere.CreateRadiusAttr().Set(0.010)  # 10mm radius — matches plug-tip size
            # NOTE iter 7 Path Y debug 2026-05-12: do NOT set Purpose=guide here.
            # purpose=guide excludes the prim from PhysX contact discovery,
            # which silently breaks the entire scoring contact pipeline (saw
            # trial_2 fail insertion while trial_1 fired by lucky incidental
            # cable-link contact). Default purpose (default-render) keeps the
            # sphere PhysX-visible. Visible-in-viewport is acceptable.
            UsdPhysics.CollisionAPI.Apply(plug_sphere.GetPrim())
            print(f"[AIC-DT] plug_proxy: authored kinematic-via-hierarchy at {plug_proxy_path} "
                  f"(child of finger_link_l, zero local offset, 10mm sphere collider). "
                  f"USD hierarchy gives this proxy the gripper's pose every frame — "
                  f"no joint, no callback, no articulation entanglement. "
                  f"Scoring queries this path for plug-port contact.")
        except Exception as exc:
            print(f"[AIC-DT] plug_proxy authoring failed: {exc!r}")

        # orphan-finger_r-fixedjoint-cleanup 2026-05-12: the upstream
        # aic_unified_robot_cable_sdf.usd ships with a pre-authored FixedJoint at
        # /World/UR5e/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint
        # whose body0 rel targets /World/UR5e/cable/sfp_module_visual. At asset
        # cook time the cable subtree isn't fully resolved yet (load order), so
        # PhysX emits:
        #   [Error] [omni.physicsschema.plugin] Joint (.../finger_link_r/FixedJoint)
        #     body relationship /World/UR5e/cable/sfp_module_visual points to a
        #     non existent prim, joint will not be created.
        # The USD prim survives as a phantom (PhysX never bound it), generating
        # log noise on every load. Cleanup is safe: PhysX already dropped the
        # constraint at cook; removing the USD prim just stops the duplicate
        # error spam on each subsequent load_trial.
        try:
            r_finger_joint = f"/World/UR5e/aic_unified_robot/gripper_hande_finger_link_r/FixedJoint"
            orphan = stage.GetPrimAtPath(r_finger_joint)
            if orphan and orphan.IsValid():
                stage.RemovePrim(r_finger_joint)
                print(f"[AIC-DT] orphan-finger_r-fixedjoint-cleanup: removed phantom {r_finger_joint}")
        except Exception as exc:
            print(f"[AIC-DT] orphan-finger_r-fixedjoint-cleanup failed: {exc!r}")

        # gripper_initial_pos → fire the gripper_command MCP atom.
        # D-09 has closed: the finger joints are now PhysicsPrismaticJoint with
        # DriveAPI per build_cable_variant_usds.py::author_gripper_prismatic_joints,
        # so we drive both fingers to the per-trial position. The atom writes
        # drive:linear:physics:targetPosition directly (USD attribute layer);
        # PhysX picks it up on the next solver step. Mimic semantics enforced
        # at write — both fingers receive the same target.
        try:
            self._cmd_gripper_command(position=gripper_initial_pos)
            print(f"[AIC-DT] gripper_initial_pos={gripper_initial_pos} → drive target applied to both fingers")
        except Exception as exc:
            print(f"[AIC-DT] gripper_command from load_robot failed: {exc!r}")

        # tcp-track-held-connector 2026-05-17 — mirror Gazebo CablePlugin behavior.
        # Diagnosis (this session, /tmp/bag_trial_3_*/*.mcap probe):
        #   Gazebo trial_3 cable_1/sc_plug_link world Z = 1.4545 (essentially at
        #   gripper/tcp world Z = 1.4602; the SC plug is HELD at the gripper TIP).
        #   Isaac Sim (pre-fix) had sc_plug_visual world Z = 1.639 — at gripper
        #   hande_base level, 172mm above where Gazebo holds it, INSIDE the
        #   gripper housing and visually occluded. The 172mm = the static
        #   hande_base→TCP offset.
        # Root cause: the orphan FixedJoint at finger_link_r (sfp_module_visual ↔
        # articulation member) anchored the connector to the articulation origin,
        # not to the gripper tip. SFP+LC (28×67×45mm) was big enough to protrude
        # past the housing; small SC plug (16mm thick) disappears inside it.
        # Fix architecture:
        #   1. build_cable_variant_usds.py removes the orphan FixedJoint at USD
        #      authoring time (so PhysX never cooks the wrong-anchor constraint).
        #   2. This function (below) installs a physics-step callback that sets
        #      the held connector's xformOp:translate every tick so its world
        #      position tracks gripper_tcp. Connector is kinematic (Layer 2 in
        #      the build script), so the assignment sticks against PhysX.
        # Per-tick callback (not one-shot) because the gripper moves during
        # CheatCode trajectories — the held connector must follow.
        try:
            held_path = ("/World/UR5e/cable/sc_plug_visual"
                         if cable_type == "sfp_sc_cable_reversed"
                         else "/World/UR5e/cable/sfp_module_visual")
            self._held_connector_path = held_path
            self._held_connector_cable_type = cable_type
            self._install_held_connector_tcp_tracker(held_path)
        except Exception as exc:
            print(f"[AIC-DT] tcp-track-held-connector install failed: {exc!r}")

    # Held-connector pose offsets relative to gripper_tcp, sourced from Gazebo
    # /tf bags (trial_3 captured 2026-05-17 from bag_trial_3_20260517_082306_946;
    # trial_1 captured same day from bag_trial_1_*). Each entry has:
    #   rel_translate_world: position offset in WORLD frame (constant under
    #     Gazebo's gripper hover pose; converted to TCP-local at runtime).
    #   rel_quat_xyzw: orientation relative to TCP (held_world_quat =
    #     tcp_world_quat * rel_quat_xyzw).
    # The TCP-local translation is recomputed from the world offset per tick
    # because the offset is small (sub-cm) and the rotation alignment matters
    # more than absolute world offset.
    _HELD_CONNECTOR_POSE_OFFSETS = {
        "sfp_sc_cable_reversed": {
            # held = sc_plug_visual; observed Gazebo trial_3 hover pose
            "rel_translate_world": (0.0003, -0.0079, -0.0057),
            "rel_quat_xyzw": (0.68471, -0.15624, 0.69152, -0.16912),
        },
        # trial_1 SFP module captured 2026-05-17 from
        # bag_trial_1_20260517_085246_597. The Gazebo body frame for the held
        # connector in sfp_sc_cable variant is sfp_module_link (held=SFP+LC,
        # which in Isaac Sim's USD lives under /World/cable/sfp_module_visual
        # with lc_plug_visual as a child).
        "sfp_sc_cable": {
            "rel_translate_world": (0.0, -0.0124, -0.032),
            "rel_quat_xyzw": (-0.56979, 0.02292, -0.0158, 0.82132),
        },
    }

    def _install_held_connector_tcp_tracker(self, held_path: str) -> None:
        """Install a physics-step callback that pins held_path to gripper_tcp pose.

        Sets both POSITION and ORIENTATION every tick so the connector visually
        matches Gazebo's CablePlugin behavior (orientation matters as much as
        position — the plug must point INTO the port for insertion realism).

        Idempotent: removes any prior subscription before installing a new one.
        Light per-tick cost: one XformCache lookup + one Transform compose +
        translate/orient Set per tick. Safe alongside parity publishers and
        AicControllerLoop physics-step callbacks.
        """
        import omni.physx
        from pxr import UsdGeom, Gf

        # Remove prior subscription if any (e.g. on trial reload).
        prev = getattr(self, "_held_connector_sub", None)
        if prev is not None:
            try:
                prev.unsubscribe()
            except Exception:
                pass
            self._held_connector_sub = None

        cable_type = getattr(self, "_held_connector_cable_type", "sfp_sc_cable")
        offset = self._HELD_CONNECTOR_POSE_OFFSETS.get(
            cable_type, self._HELD_CONNECTOR_POSE_OFFSETS["sfp_sc_cable"]
        )
        rel_quat_xyzw = offset["rel_quat_xyzw"]
        rel_xyz_world = offset["rel_translate_world"]
        # Gf.Quatd takes (real, imaginary) = (w, x, y, z)
        rel_quat = Gf.Quatd(rel_quat_xyzw[3], rel_quat_xyzw[0],
                            rel_quat_xyzw[1], rel_quat_xyzw[2])

        # Fabric (usdrt) provides the runtime scene representation that PhysX
        # writes back to each step. Writing directly to USD attributes during
        # a physics step is overridden by PhysX's Fabric writeback for any
        # body with RigidBodyAPI (even kinematic). Write to Fabric instead so
        # the tracker's pose wins.
        try:
            import usdrt  # noqa: F401 — import probe (raises if unavailable)
            from usdrt import Gf as RtGf  # noqa: F401
            has_fabric = True
        except Exception as exc:
            print(f"[AIC-DT] tcp-track-held-connector: Fabric (usdrt) unavailable, "
                  f"falling back to USD-level Set: {exc!r}")
            has_fabric = False
            RtGf = None  # type: ignore[assignment]

        def _on_step(dt: float) -> None:
            try:
                stage = omni.usd.get_context().get_stage()
                if stage is None: return
                held = stage.GetPrimAtPath(held_path)
                tcp = stage.GetPrimAtPath("/World/UR5e/aic_unified_robot/gripper_tcp")
                if not (held and held.IsValid() and tcp and tcp.IsValid()):
                    return
                xc = UsdGeom.XformCache()
                # Build desired held world transform (matrix, not quat — avoids
                # scale-shear discrepancies when parent has scale).
                tcp_wtm = xc.GetLocalToWorldTransform(tcp)
                tcp_world_pos = tcp_wtm.ExtractTranslation()
                tcp_rot_mat = tcp_wtm.ExtractRotationMatrix()
                rel_rot_mat = Gf.Matrix3d().SetRotate(rel_quat)
                # OpenUSD/Gf uses ROW-vector convention: the LEFT matrix is more
                # local in composition. Desired held world rotation = TCP world
                # rotation followed by the cable-asset rel rotation; in row-vector
                # math the rel matrix must go on the LEFT of TCP.
                # (Per GPT diagnosis 2026-05-17 via /ask-gpt — citing OpenUSD
                # GfMatrix4d row-vector convention + UsdGeomXformable xform-op
                # ordering docs.)
                held_rot_mat = rel_rot_mat * tcp_rot_mat
                held_world_pos = Gf.Vec3d(tcp_world_pos[0] + rel_xyz_world[0],
                                           tcp_world_pos[1] + rel_xyz_world[1],
                                           tcp_world_pos[2] + rel_xyz_world[2])
                # Convert held world → held local via parent inverse.
                parent_wtm = xc.GetLocalToWorldTransform(held.GetParent())
                parent_wtm_inv = parent_wtm.GetInverse()
                held_wtm = Gf.Matrix4d().SetIdentity()
                held_wtm.SetRotateOnly(held_rot_mat)
                held_wtm.SetTranslateOnly(held_world_pos)
                held_local_mat = held_wtm * parent_wtm_inv
                local_pos = held_local_mat.ExtractTranslation()
                local_rot_quat = held_local_mat.ExtractRotation().GetQuat()

                # Write to BOTH USD and Fabric. Hydra renderer + omni.physx
                # writeback can read from either, depending on context; writing
                # both removes the ambiguity. Without USD-level write, the
                # renderer (which composes xformOps through USD's stage delegate
                # in some configurations) would see the on-disk value.
                tr_attr = held.GetAttribute("xformOp:translate")
                or_attr = held.GetAttribute("xformOp:orient")
                if tr_attr.IsValid():
                    tr_attr.Set(Gf.Vec3d(local_pos[0], local_pos[1], local_pos[2]))
                if or_attr.IsValid():
                    type_str = str(or_attr.GetTypeName()).lower()
                    if "quatf" in type_str:
                        or_attr.Set(Gf.Quatf(local_rot_quat.GetReal(),
                                             local_rot_quat.GetImaginary()[0],
                                             local_rot_quat.GetImaginary()[1],
                                             local_rot_quat.GetImaginary()[2]))
                    else:
                        or_attr.Set(local_rot_quat)
                if has_fabric:
                    import usdrt
                    from usdrt import Gf as _RtGf
                    fstage = usdrt.Usd.Stage.Attach(
                        omni.usd.get_context().get_stage_id()
                    )
                    fprim = fstage.GetPrimAtPath(held_path)
                    if fprim and fprim.IsValid():
                        ftr = fprim.GetAttribute("xformOp:translate")
                        forn = fprim.GetAttribute("xformOp:orient")
                        if ftr.IsValid():
                            ftr.Set(_RtGf.Vec3d(local_pos[0], local_pos[1], local_pos[2]))
                        if forn.IsValid():
                            forn.Set(_RtGf.Quatf(local_rot_quat.GetReal(),
                                                 local_rot_quat.GetImaginary()[0],
                                                 local_rot_quat.GetImaginary()[1],
                                                 local_rot_quat.GetImaginary()[2]))
            except Exception as exc:
                # One bad tick should never crash the sim, but a silent
                # except can hide a failed Set (GPT pose-diagnosis 2026-05-17).
                # Log once per session to surface tracker errors during
                # diagnostics; subsequent ticks stay silent.
                if not getattr(self, "_tracker_error_logged", False):
                    self._tracker_error_logged = True
                    import traceback
                    print(f"[AIC-DT] tcp-track-held-connector tick error: {exc!r}")
                    traceback.print_exc()

        physx = omni.physx.acquire_physx_interface()
        self._tracker_error_logged = False
        self._held_connector_sub = physx.subscribe_physics_step_events(_on_step)
        print(f"[AIC-DT] tcp-track-held-connector: subscribed physics-step tracker for {held_path} "
              f"(cable_type={cable_type!r}, rel_quat_xyzw={rel_quat_xyzw})")

    def _configure_arm_drives(self, stage, prim_path: str) -> None:
        """Author USD DriveAPI + PhysxArticulationAPI on the unified-robot arm.

        joint-drives-urdf-reconcile: arm joint drives previously inherited the
        on-disk USD's OEM defaults (stiffness 95→1322, damping 0.038→0.529,
        maxForce 28 or 150). Canonical values come from AIC's Isaac Lab env
        (`aic_task_env_cfg.py::ImplicitActuatorCfg`):
          - stiffness=2000, damping=100, effort_limit_sim=87 on each arm joint
          - articulation root solver iters: pos=16, vel=8

        Authoring happens BEFORE ArticulationView init / reset_async so PhysX
        picks the values up at first articulation cook. Joint-not-found is a
        soft warning (legacy USD layouts may differ); solver-API absence is
        likewise non-fatal — values stay USD-only and runtime defaults apply.
        """
        for joint_name in self._joint_names:
            joint_path = f"{prim_path}/aic_unified_robot/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if not joint_prim.IsValid():
                print(f"[AIC-DT] joint-drives-urdf-reconcile: joint not found at {joint_path}")
                continue
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.CreateMaxForceAttr().Set(self._ur5e_max_force)
            drive_api.CreateStiffnessAttr().Set(self._ur5e_stiffness)
            drive_api.CreateDampingAttr().Set(self._ur5e_damping)
            print(
                f"[AIC-DT] joint-drives-urdf-reconcile: {joint_name} → "
                f"stiffness={self._ur5e_stiffness} damping={self._ur5e_damping} "
                f"maxForce={self._ur5e_max_force}"
            )

        # Articulation root sits at /World/UR5e/aic_unified_robot per
        # _articulation_root_prim_path. Author solver iters from aic_isaaclab.
        artic_root_prim = stage.GetPrimAtPath(self._articulation_root_prim_path)
        if artic_root_prim.IsValid():
            try:
                physx_artic = PhysxSchema.PhysxArticulationAPI.Apply(artic_root_prim)
                physx_artic.CreateSolverPositionIterationCountAttr().Set(16)
                physx_artic.CreateSolverVelocityIterationCountAttr().Set(8)
                print("[AIC-DT] joint-drives-urdf-reconcile: solver iters pos=16 vel=8")
            except Exception as exc:  # noqa: BLE001 — solver iter authoring is best-effort
                print(f"[AIC-DT] joint-drives-urdf-reconcile: solver iter author skipped: {exc}")
        else:
            print(
                f"[AIC-DT] joint-drives-urdf-reconcile: articulation root prim "
                f"not found at {self._articulation_root_prim_path}"
            )

    async def setup_action_graph(self):
        import omni.graph.core as og
        from isaacsim.core.utils.extensions import enable_extension

        print("Setting up ROS 2 Action Graph...")

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(self._robot_prim_path):
            print(f"Error: UR5e not found at {self._robot_prim_path}. Load UR5e first.")
            return

        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.core.nodes")
        enable_extension("omni.graph.action")

        graph_path = "/Graph/ActionGraph_UR5e"

        if stage.GetPrimAtPath(graph_path):
            print(f"Action Graph already exists at {graph_path}, skipping creation.")
            return

        (graph, nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution"
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ros2_subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("ros2_context.inputs:domain_id", 0),
                    ("ros2_subscribe_joint_state.inputs:topicName", "/joint_states"),
                    ("ros2_subscribe_joint_state.inputs:nodeNamespace", ""),
                    ("ros2_publish_clock.inputs:topicName", "/clock"),
                    ("ros2_publish_clock.inputs:nodeNamespace", ""),
                    ("isaac_read_simulation_time.inputs:resetOnStop", False),
                    ("articulation_controller.inputs:targetPrim", self._robot_prim_path),
                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_subscribe_joint_state.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_subscribe_joint_state.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_clock.inputs:timeStamp"),
                    ("ros2_subscribe_joint_state.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
                    ("ros2_subscribe_joint_state.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("ros2_subscribe_joint_state.outputs:effortCommand", "articulation_controller.inputs:effortCommand"),
                    ("ros2_subscribe_joint_state.outputs:jointNames", "articulation_controller.inputs:jointNames"),
                ],
            }
        )

        print("ROS 2 Action Graph setup complete.")

    # ==================== Force Publisher ====================

    def setup_force_publish_action_graph(self):
        """Setup force publishing as geometry_msgs/WrenchStamped on /fts_broadcaster/wrench.

        PARITY-05 contract (Plan 04 Task 4 — Phase 1 ship):
          Topic name:     /fts_broadcaster/wrench
          Message type:   geometry_msgs/msg/WrenchStamped
          frame_id:       ati/tool_link
          Source of truth: ~/Documents/aic/aic_description/urdf/ur_gz.urdf.xacro
                          line 233-243 (AtiForceTorqueSensor → frame_id=ati/tool_link)
          Snapshot:       .planning/phases/01-foundation-parity/snapshot/
                          topic_info_fts_broadcaster_wrench.txt (Plan 01 Task 2)

        Pre-Plan-04 state: builder published WrenchStamped on the correct topic
        AFTER Task 1 rename, but with frame_id="tool0" (UR5e tool flange) which
        does NOT match the live Gazebo aic_eval container's "ati/tool_link" frame
        (the actual ATI force/torque sensor mount frame in the URDF). Task 4
        rewrites frame_id to "ati/tool_link" — Case A reconciliation per PLAN.md.

        PARITY-05 verify (Plan 07 / verify_phase_1.sh executes once Isaac Sim is
        running):
            ros2 topic info /fts_broadcaster/wrench --verbose
              -> Type: geometry_msgs/msg/WrenchStamped
              -> Publisher count: >= 1
            ros2 topic echo /fts_broadcaster/wrench --once
              -> header.frame_id: ati/tool_link
        """
        import omni.physx

        print("Setting up UR5e Force Publisher (WrenchStamped)...")

        self._stop_force_publish()

        stage_check = omni.usd.get_context().get_stage()
        if not stage_check.GetPrimAtPath(self._robot_prim_path):
            print(f"Error: UR5e prim not found at {self._robot_prim_path}. Load UR5e first.")
            return

        # Do NOT seed _effort_articulation from self._robot_view: that handle
        # was constructed in load_robot before reset_async returned, so its
        # _physics_view is None and never populates. The lazy-init path in
        # _on_physics_step_force constructs a fresh handle post-physics-tick
        # (warmup=30), which IS the working pattern (see parity_publishers.py
        # /joint_states reading proves it). PARITY-05 wrench-rootcause fix.
        self._effort_articulation = None
        self._force_warmup = 30
        self._wrench_body_idx = None
        print("Force publisher will lazy-init Articulation post-physics-tick (warmup=30).")

        graph_path = "/Graph/ActionGraph_UR5e_ForcePublish"
        keys = og.Controller.Keys
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)

        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnPlaybackTick"),
                    ("context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("publisher", "isaacsim.ros2.bridge.ROS2Publisher")
                ],
                keys.SET_VALUES: [
                    ("publisher.inputs:messageName", "WrenchStamped"),
                    ("publisher.inputs:messagePackage", "geometry_msgs"),
                    ("publisher.inputs:topicName", "fts_broadcaster/wrench"),
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "publisher.inputs:execIn"),
                    ("context.outputs:context", "publisher.inputs:context"),
                ]
            }
        )

        self._force_graph_path = graph_path
        self._force_pub_node = nodes[2]  # publisher node

        # PARITY-05: frame_id matches live aic_eval — see method docstring for source of truth.
        og.Controller.attribute("inputs:header:frame_id", self._force_pub_node).set("ati/tool_link")

        self._force_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step_force
        )
        self._force_publish_active = True

        print(f"UR5e Force Publisher created at {graph_path}")
        print("Publishing geometry_msgs/WrenchStamped to topic: /fts_broadcaster/wrench")

    def _on_physics_step_force(self, dt):
        """Physics step callback - read joint forces and update OmniGraph WrenchStamped attributes.

        PARITY-05 wrench-rootcause:
          - Articulation root is at <robot>/aic_unified_robot/root_joint
            (PhysicsFixedJoint), not at /World/UR5e (Xform).
          - frame_id="ati/tool_link" → read forces row for body_names index of
            "ati_tool_link" (USD slash→underscore convention). With cable
            active or arm-only, body order varies; indexing by [-1] picks
            whatever happens to be last (a cable rope link or finger pad)
            and produces all-zero or wrong-frame readings. Resolve once
            after construction; cache in self._wrench_body_idx.
        """
        try:
            artic = self._lazy_init_articulation(
                '_effort_articulation',
                f"{self._articulation_root_prim_path}/root_joint",
                'force_pub_ctrl', '_force_warmup')
            if artic is None:
                return
            self._effort_articulation = artic

            if self._wrench_body_idx is None:
                body_names = list(getattr(artic, 'body_names', []) or [])
                # ati_tool_link is the canonical FT-sensor mount (URDF
                # ati/tool_link → USD ati_tool_link). Fallbacks ordered by
                # proximity to the wrist for robustness across robot variants.
                for cand in ("ati_tool_link", "tool0", "wrist_3_link", "flange"):
                    if cand in body_names:
                        self._wrench_body_idx = body_names.index(cand)
                        print(f"[Force callback] wrench body resolved: {cand} -> idx {self._wrench_body_idx} (of {len(body_names)} bodies)")
                        break
                if self._wrench_body_idx is None:
                    # No match — fall back to last body (legacy behavior) but
                    # warn loudly. Should never happen on the AIC unified USD.
                    self._wrench_body_idx = len(body_names) - 1
                    print(f"[Force callback] WARNING: no wrist body match in {body_names!r}; falling back to [-1]")

            forces = artic.get_measured_joint_forces()
            if forces is not None and len(forces) > 0:
                wrist = forces[0][self._wrench_body_idx]
                node = self._force_pub_node
                og.Controller.attribute("inputs:wrench:force:x", node).set(float(wrist[0]))
                og.Controller.attribute("inputs:wrench:force:y", node).set(float(wrist[1]))
                og.Controller.attribute("inputs:wrench:force:z", node).set(float(wrist[2]))
                og.Controller.attribute("inputs:wrench:torque:x", node).set(float(wrist[3]))
                og.Controller.attribute("inputs:wrench:torque:y", node).set(float(wrist[4]))
                og.Controller.attribute("inputs:wrench:torque:z", node).set(float(wrist[5]))
        except Exception as e:
            print(f"[Force callback] {e}")

    def _stop_force_publish(self):
        """Stop the physics-rate force publisher and clean up resources."""
        self._stop_physics_callback('_force_physx_sub', '_effort_articulation',
                                    '_force_publish_active', '_force_warmup')
        # Cached wrench body index belongs to the previous Articulation
        # instance — reset so the next setup re-resolves against fresh bodies.
        self._wrench_body_idx = None

    # ==================== TF Publisher (PARITY-04) ====================

    def setup_tf_publish_action_graph(self):
        """Start the rclpy-based AIC TF publisher (/tf + /tf_static).

        PARITY-04 contract:
          Topics:        /tf, /tf_static
          Message types: tf2_msgs/msg/TFMessage (both)
          QoS:           /tf       = RELIABLE / VOLATILE / KEEP_LAST(10)
                         /tf_static = RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)
          Frame names:   slashed AIC form (`gripper/hande_finger_link_l`,
                         `cam_mount/cam_mount_link`, etc.) per
                         exts/aic-dt/docs/topic-parity-reference.md.

        IMPLEMENTATION NOTE (2026-05-03 Gap-B inline fix):
          The OGN-based ROS2PublishTransformTree node from Plan 06 cannot
          deliver this contract: (a) frame_ids derive from USD prim leaf
          names, but USD prim names cannot legally contain '/' so AIC's
          slashed frame names are unrepresentable; (b) empirical Stage 1
          test showed the node emitted only the single
          World->aic_unified_robot edge even when targetPrims pointed at
          the actual articulation root prim
          (/World/UR5e/aic_unified_robot/root_joint), so articulation
          traversal documented in the 5.0 OGN spec was not firing for
          this articulation. Decision: bypass OGN and publish directly
          via rclpy from a physics-step callback.

          AIC-frame -> USD-prim mapping + parent/child edge list +
          dynamic-vs-static partitioning live in
          exts/aic-dt/aic_dt/parity_publishers.py. This setup atom and
          setup_joint_state_publish_action_graph share the same
          AicParityPublishers backend (one rclpy node, two publishers);
          calling either is idempotent and starts both surfaces.

        Verify (Plan 07/08 verify_phase_1.sh):
            ros2 topic list | grep -E "^/tf(_static)?$"
            ros2 topic info /tf --verbose
              -> Type: tf2_msgs/msg/TFMessage / RELIABLE / VOLATILE
            ros2 topic info /tf_static --verbose
              -> Reliability: RELIABLE / Durability: TRANSIENT_LOCAL
            ros2 run tf2_tools view_frames
              -> 31 frames including aic_world, world, tabletop,
                 base_link..wrist_3_link, gripper/*, cam_mount/*, ati/*,
                 *_camera/* in slashed form.
        """
        print("[AIC-DT] Setting up TF Publisher via rclpy (/tf + /tf_static)...")
        # Remove prior OGN-based action graph if present (Plan 06 leftover); the
        # rclpy publisher takes its place on the same topic names.
        self._remove_legacy_publish_graphs()
        self._start_aic_parity_publishers()

    def setup_joint_state_publish_action_graph(self):
        """Start the rclpy-based AIC JointState publisher (/joint_states).

        PARITY-03 contract:
          Topic:         /joint_states
          Message type:  sensor_msgs/msg/JointState
          QoS:           RELIABLE / VOLATILE / KEEP_LAST(42)
          Joint names:   7 alphabetical -- 6 UR5e arm DOFs + the literal
                         `gripper/left_finger_joint` (slashed) per the live
                         Gazebo /joint_states snapshot in
                         exts/aic-dt/docs/topic-parity-reference.md.
          frame_id:      `base_link`

        IMPLEMENTATION NOTE (2026-05-03 Gap-A inline fix):
          The OGN-based ROS2PublishJointState node from Plan 06 cannot
          deliver this contract: (a) no jointNames / nameOverrides input,
          so it publishes USD-prim-leaf names (`gripper_left_finger_joint`
          underscore form, not `gripper/left_finger_joint`); (b) no frameId
          input, so header.frame_id is empty string; (c) only emits the
          articulation's natural DOF order (URDF kinematic-chain), not
          alphabetical; (d) cannot include the `gripper/left_finger_joint`
          7th name -- the Hand-E finger is a PhysicsFixedJoint with zero
          DOF and is not part of the articulation. aic_adapter::ReorderJointState
          is name-indexed against literal `gripper/left_finger_joint`
          (aic_adapter.cpp:86), so the slashed name is non-negotiable.

          Solution: bypass OGN and publish via rclpy. The 7th joint is
          published with a constant 0.0 position (the Hand-E gripper does
          not animate in the unified USD; matches the FixedJoint reality
          while satisfying aic_adapter's name-indexed reorder). 6 arm DOFs
          read live from the Articulation API.

        Verify (Plan 07/08 verify_phase_1.sh):
            ros2 topic info /joint_states --verbose
              -> Type: sensor_msgs/msg/JointState
            ros2 topic echo /joint_states --once | yq '.name'
              -> [elbow_joint, gripper/left_finger_joint, shoulder_lift_joint,
                  shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
            ros2 topic echo /joint_states --once | yq '.header.frame_id'
              -> base_link
        """
        print("[AIC-DT] Setting up JointState Publisher via rclpy (/joint_states)...")
        self._remove_legacy_publish_graphs()
        self._start_aic_parity_publishers()

    def _remove_legacy_publish_graphs(self):
        """Tear down OGN action graphs from Plan 06 if any remain.

        Idempotent. Prevents the legacy OGN nodes from racing the rclpy
        publishers on /tf and /joint_states topic ownership.
        """
        stage = omni.usd.get_context().get_stage()
        for path in ("/Graph/ActionGraph_UR5e_TFPublish",
                     "/Graph/ActionGraph_UR5e_JointStatePublish"):
            if stage.GetPrimAtPath(path):
                stage.RemovePrim(path)
                print(f"[AIC-DT][parity] Removed legacy OGN graph: {path}")

    def _start_aic_parity_publishers(self):
        """Start the shared AicParityPublishers manager (idempotent).

        Both setup_tf_publish_action_graph and
        setup_joint_state_publish_action_graph delegate here. The manager
        owns the rclpy node, /joint_states + /tf + /tf_static publishers,
        and the physics-step callback that computes and publishes each
        tick. Safe to call multiple times -- internally tears down prior
        state before re-initializing.
        """
        try:
            from .parity_publishers import AicParityPublishers
        except Exception as exc:
            print(f"[AIC-DT][parity] Failed to import parity_publishers: {exc!r}")
            return
        if self._aic_parity_publishers is None:
            self._aic_parity_publishers = AicParityPublishers(
                robot_xform_path=self._articulation_root_prim_path
            )
        ok = self._aic_parity_publishers.start()
        if not ok:
            print("[AIC-DT][parity] Publisher start() returned False -- check rclpy availability.")

    def _start_aic_scoring_publishers(self, port_link_paths=None):
        """Start the AicScoringPublishers manager (Phase 3 Plan 03-04+05). Idempotent.

        Owns rclpy node + /scoring/tf + /objects_poses_real + /scoring/insertion_event
        publishers + omni.physx contact subscription for plug-port detection.
        Mirrors _start_aic_parity_publishers pattern. Should be called from
        quick_start AFTER add_objects so task_board prims exist for TF lookup.

        Args:
            port_link_paths: D-13 fallback (Plan 04-03 Wave 1 A4=MISMATCH).
                Optional list of port USD prim paths to use for the insertion
                contact-report subscription. If provided, overrides the
                module-level scoring_publishers._PORT_LINK_PATHS default
                (which targets the legacy snake_case /World/TaskBoard/sc_port_*
                namespace). load_trial passes the live CamelCase paths
                computed from the actual spawn calls.
        """
        try:
            from .scoring_publishers import AicScoringPublishers
        except Exception as exc:
            print(f"[AIC-DT][scoring] Failed to import scoring_publishers: {exc!r}")
            return
        if not hasattr(self, "_aic_scoring_publishers") or self._aic_scoring_publishers is None:
            self._aic_scoring_publishers = AicScoringPublishers(
                robot_xform_path=self._articulation_root_prim_path
            )
        # D-13 fallback: apply the override BEFORE start() — contact-report
        # subscription is wired at start time, not on each tick.
        if port_link_paths is not None:
            self._aic_scoring_publishers.set_port_link_paths(port_link_paths)
        ok = self._aic_scoring_publishers.start()
        if not ok:
            print("[AIC-DT][scoring] Publisher start() returned False -- check rclpy availability.")

    def _start_aic_controller_loop(self, off_limit_prims: list = None):
        """Start the shared AicControllerLoop manager (idempotent).

        Mirrors _start_aic_parity_publishers (Phase 1). Both
        setup_controller_subscribers AND setup_offlimit_contacts atoms delegate
        here — single manager instance per extension lifecycle. Re-invocation
        with off_limit_prims=<list> updates the off-limit prim filter; otherwise
        a single manager owns all 4 controller-loop topics.
        """
        try:
            from .controller_loop import AicControllerLoop
        except Exception as exc:
            print(f"[AIC-DT][controller] Failed to import controller_loop: {exc!r}")
            return
        if self._aic_controller_loop is None:
            self._aic_controller_loop = AicControllerLoop(
                robot_xform_path=self._articulation_root_prim_path,
                off_limit_prims=off_limit_prims,
            )
        elif off_limit_prims is not None:
            # Allow per-call override of off_limit set when the offlimit atom is invoked
            self._aic_controller_loop.set_off_limit_prims(off_limit_prims)
        ok = self._aic_controller_loop.start()
        if not ok:
            print("[AIC-DT][controller] start() returned False — check rclpy/aic_control_interfaces availability (see exts/aic-dt/docs/aic-msgs-setup.md).")
        # motion-deficit-hunt callback-ordering fix: re-register parity_publishers
        # AFTER the controller loop has subscribed its physics_step callback. The
        # carb event dispatcher fires per-tick callbacks in subscription order;
        # without this restart, parity_publishers (which subscribed earlier in
        # quick_start via setup_{tf,joint_state}_publish_action_graph) reads
        # /joint_states BEFORE controller_loop's _apply_joint_cmd writes the new
        # dof_pos for the tick, so /joint_states reports the previous tick's
        # post-PhysX-integration state (drifted from controller's last write
        # by ~25% for shoulder_lift due to gravity moment ≈ 538 N·m). After
        # restart, parity fires AFTER controller per tick → /joint_states sees
        # the controller's just-written target. Empirical: 73% → ≥95%
        # commanded amplitude in 2s window. parity.start() is idempotent (calls
        # stop() first then resubscribes); only re-subscribes if already running.
        if self._aic_parity_publishers is not None and self._aic_parity_publishers._node is not None:
            self._aic_parity_publishers.start()
            print("[AIC-DT][controller] parity_publishers re-subscribed AFTER controller_loop (callback-ordering fix for motion-deficit-hunt)")

    # ==================== Wrist Cameras ====================

    def setup_wrist_cameras(self):
        """Create ROS2 action graphs for all 3 built-in wrist cameras (center, left, right)."""
        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath(self._robot_prim_path).IsValid():
            print(f"Error: UR5e not found at {self._robot_prim_path}. Load UR5e first.")
            return

        created = 0
        for cam_name, cam_cfg in WRIST_CAMERAS.items():
            camera_prim_path = f"{self._robot_prim_path}/{cam_cfg['prim_suffix']}"

            if not stage.GetPrimAtPath(camera_prim_path).IsValid():
                print(f"Warning: Camera prim not found at {camera_prim_path}, skipping {cam_name}")
                continue

            self._attach_camera_ros2_writers(
                camera_prim_path,
                cam_cfg["width"],
                cam_cfg["height"],
                cam_cfg["topic"],
                cam_name,
                frame_id=cam_cfg["frame_id"],
                info_topic=cam_cfg["info_topic"],
            )
            created += 1

        print(f"Setup {created}/{len(WRIST_CAMERAS)} wrist camera ROS2 writers")

    def _attach_camera_ros2_writers(self, camera_prim, width, height, topic, name,
                                     frame_id="camera_link", info_topic="camera_info",
                                     freq=30):
        """Author a Helper-based ROS2 camera-publish ActionGraph.

        Canonical layout (per project's action-graphs.md reference snippet):
            OnPlaybackTick → ROS2CameraHelper.execIn         (per-frame publish)
            OnPlaybackTick → ROS2CameraInfoHelper.execIn     (per-frame publish)
            ROS2Context.outputs:context → both Helpers' inputs:context

        Each Helper carries its own `inputs:cameraPrim` + `inputs:width` +
        `inputs:height` and INTERNALLY creates its render product + SDG
        annotator chain at first tick. The previously-attempted approach
        (with a separate `IsaacCreateRenderProduct` node wired into the
        Helpers' `inputs:renderProductPath`) caused an init-order race —
        the Helper began its own RP setup and got partially overridden,
        leaving the SDG annotator chain unspawned (width=0 on the
        publisher, no data flowing). Verified live 2026-05-13 by probing
        Replicator_NodeWriterWriter.inputs:width=0 after the
        rep.writers.get + .attach pattern returned nominal success.

        The pure-Python `rep.writers.get().attach()` API (tutorial pattern
        for SimulationApp standalone scripts) doesn't init the ROS2
        context inside extension context, leaving inputs:context=0.

        State: per-camera ActionGraph paths tracked in
        `self._wrist_camera_handles[name]` for clean teardown.
        """
        graph_path = self._wrist_camera_graph_path(name)
        stage = omni.usd.get_context().get_stage()

        if not hasattr(self, "_wrist_camera_handles"):
            self._wrist_camera_handles = {}

        # Idempotent: tear down any existing graph for this name
        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)
        self._wrist_camera_handles.pop(name, None)

        # Pattern lifted verbatim from working ur5e-dt sibling extension
        # (~/Documents/isaac-sim-mcp/exts/ur5e-dt/ur5e_dt/extension.py:3094).
        # Critical wiring detail: Helpers' execIn comes from
        # RenderProduct.outputs:execOut, NOT OnPlaybackTick. RenderProduct's
        # execOut fires AFTER the render product is ready each frame. Feeding
        # Helpers from OnPlaybackTick directly causes them to publish BEFORE
        # the render product is populated → silent no-publish (verified live
        # 2026-05-13: ROS2PublishImage on stage with topic + context + width
        # but inputs:width=0 because annotator chain never received data).
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("InfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:useDomainIDEnvVar", True),
                    ("RenderProduct.inputs:cameraPrim", camera_prim),
                    ("RenderProduct.inputs:width", int(width)),
                    ("RenderProduct.inputs:height", int(height)),
                    ("InfoPublish.inputs:topicName", info_topic),
                    ("InfoPublish.inputs:frameId", frame_id),
                    ("InfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ("RGBPublish.inputs:topicName", topic),
                    ("RGBPublish.inputs:type", "rgb"),
                    ("RGBPublish.inputs:frameId", frame_id),
                    ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    # Helpers fire on RenderProduct.execOut (every frame, AFTER RP ready)
                    ("RenderProduct.outputs:execOut", "InfoPublish.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "InfoPublish.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                    ("ROS2Context.outputs:context", "InfoPublish.inputs:context"),
                    ("ROS2Context.outputs:context", "RGBPublish.inputs:context"),
                ],
            },
        )

        self._wrist_camera_handles[name] = {"graph_path": graph_path}
        print(f"[AIC-DT] '{name}' camera publish graph authored at {graph_path}")
        print(f"[AIC-DT] '{name}' topics: /{topic} + /{info_topic}")

        # Kick the replicator orchestrator so Hydra actively renders the
        # secondary IsaacCreateRenderProduct. Without this, the render product
        # exists but Hydra never feeds RGBA pixels into the SDG annotator chain
        # (verified live: inputs:width=0 on the auto-spawned ROS2PublishImage,
        # no topic ever reaches the ROS bus). step_async drives a full render
        # pass for all active render products + flushes the SDG pipeline →
        # publishers fire. Inside a Kit extension the synchronous variant errors
        # ("may not be made from within Kit"); the async one schedules onto the
        # Kit event loop. We keep a single perpetual pump task alive while any
        # camera stream is active; cancelled in _detach_camera_ros2_writers
        # when the last stream tears down.
        self._ensure_camera_pump_running()

    def _detach_camera_ros2_writers(self, name: str):
        """Tear down a wrist camera stream's ActionGraph. Idempotent.

        Cancels the replicator orchestrator pump task if no other camera
        streams remain active.
        """
        if not hasattr(self, "_wrist_camera_handles"):
            self._wrist_camera_handles = {}
        handles = self._wrist_camera_handles.pop(name, None)
        graph_path = (handles or {}).get("graph_path") or self._wrist_camera_graph_path(name)
        stage = omni.usd.get_context().get_stage()
        p = stage.GetPrimAtPath(graph_path)
        removed = False
        if p.IsValid():
            stage.RemovePrim(graph_path)
            removed = True
        # If no streams remain, cancel the pump
        if not self._wrist_camera_handles:
            self._cancel_camera_pump()
        return removed

    def _ensure_camera_pump_running(self):
        """(Re)start the replicator orchestrator pump.

        The pump perpetually calls `rep.orchestrator.step_async` which drives
        Hydra to render secondary `IsaacCreateRenderProduct` render products
        each cycle. Required for camera streams to actually publish — without
        it, the Helper graph executes and the SDG publishers exist with valid
        wiring, but the annotator chain never receives RGBA data (publisher
        inputs:width stays at 0, no topic on ROS bus).

        Always cancels-then-restarts. A stale pump task from a previous stage
        (orphaned by `new_stage` / `load_scene` since render products are
        per-stage) keeps `.done() == False` but no longer drives anything
        useful — confirmed live 2026-05-13 after a user reported a fresh
        load_scene → load_robot → start_wrist_camera_stream sequence
        producing no topic. Cancel + restart cheaply re-binds the pump
        onto the live stage's render-product pipeline.
        """
        import asyncio
        import omni.kit.async_engine
        import omni.replicator.core as rep

        # Cancel any pre-existing pump (handles orphaned-after-new_stage case)
        prior = getattr(self, "_camera_pump_task", None)
        if prior is not None:
            try:
                prior.cancel()
            except Exception:
                pass

        async def _pump():
            while True:
                try:
                    await rep.orchestrator.step_async(rt_subframes=1)
                except asyncio.CancelledError:
                    break
                except Exception as exc:
                    print(f"[AIC-DT] camera pump exception (continuing): {exc}")
                    await asyncio.sleep(0.5)

        self._camera_pump_task = omni.kit.async_engine.run_coroutine(_pump())
        print("[AIC-DT] replicator orchestrator pump (re)started for camera streams")

    def _cancel_camera_pump(self):
        """Cancel the perpetual pump task. Idempotent."""
        task = getattr(self, "_camera_pump_task", None)
        if task is None:
            return
        try:
            task.cancel()
        except Exception:
            pass
        self._camera_pump_task = None
        print("[AIC-DT] replicator orchestrator pump cancelled (no active streams)")

    def _selected_wrist_camera_name(self) -> str:
        """Read the current ComboBox selection. Falls back to the first
        WRIST_CAMERAS key if the ComboBox isn't built yet (e.g., direct MCP).
        """
        names = list(WRIST_CAMERAS.keys())
        if not hasattr(self, "_wrist_camera_combo") or self._wrist_camera_combo is None:
            return names[0]
        try:
            idx = self._wrist_camera_combo.model.get_item_value_model().get_value_as_int()
        except Exception:
            idx = 0
        return names[idx] if 0 <= idx < len(names) else names[0]

    # ==================== Per-wrist-camera atoms (DX-02 4-surface) ====================
    # Independent spawn/stream lifecycle per camera. Scales to any N cameras by
    # adding entries to WRIST_CAMERAS — UI ComboBox + atoms pick them up
    # automatically. Each stream runs independently; nothing in this layer
    # enforces "only one at a time" (that's a user-side testing choice).

    def _author_wrist_camera_prim(self, name: str):
        """Author a Camera prim at WRIST_CAMERAS[name]['prim_suffix'] under the robot.

        Applies ROS-optical→Isaac-camera quaternion (R_x 180° rotation: ROS optical
        has +Z forward / +Y down; Isaac camera has -Z forward / +Y up; flipping
        both Y and Z = 180° about X). Intrinsics derived from the same
        configure_camera_properties pattern as create_additional_camera, scoped
        to the per-camera (width, height) declared in WRIST_CAMERAS.

        Returns the authored Camera prim path on success.
        """
        if name not in WRIST_CAMERAS:
            raise ValueError(f"Unknown wrist camera '{name}'. Known: {list(WRIST_CAMERAS.keys())}")
        cfg = WRIST_CAMERAS[name]
        cam_path = f"{self._robot_prim_path}/{cfg['prim_suffix']}"
        stage = omni.usd.get_context().get_stage()

        # Parent _optical Xform must already exist (authored in the unified robot USD)
        parent_path = "/".join(cam_path.split("/")[:-1])
        if not stage.GetPrimAtPath(parent_path).IsValid():
            raise RuntimeError(
                f"Parent Xform {parent_path} missing — load_robot first."
            )

        # Idempotent: remove existing Camera prim if any
        existing = stage.GetPrimAtPath(cam_path)
        if existing.IsValid():
            stage.RemovePrim(cam_path)

        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam_prim = cam.GetPrim()

        # Pose: R_x(180°) ROS-optical → USD camera convention + forward
        # offset to escape the basler camera visual mesh's lens housing.
        #
        # Direction (R_x(180°)):
        #   ROS-optical:    +Z forward, +Y down, +X right
        #   USD/Isaac cam:  looks down LOCAL -Z, +Y up, +X right
        #   To make USD camera's -Z = ROS-optical's +Z, rotate 180° around X.
        #   USD Quatd is (w, x, y, z); cos(90°)=0, sin(90°)=1 → (0, 1, 0, 0).
        #
        # Position (translate +Z = +0.06m in optical frame, = forward past lens):
        #   Per ~/Documents/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro:
        #     - sensor / optical frame at camera_link +X = 21.74mm (the CCD)
        #     - lens cylinder front face at camera_link +X = 70mm
        #     - lens housing extends 48mm PAST the sensor surface
        #   The URDF places the optical frame INSIDE the visible lens housing
        #   mesh. Gazebo's <sensor> is virtual and ignores mesh occlusion;
        #   Isaac Sim's Hydra rendering does NOT — the camera renders from
        #   inside the lens body and everything is occluded, producing a pure
        #   black image. Verified empirically 2026-05-16: image black at
        #   hover pose despite correct down-pointing direction.
        #
        # Push the camera prim +0.06m along optical +Z (= 12mm past the lens
        # front face) so it sits in free space outside the housing.
        #
        # NB: Isaac Lab's TiledCameraCfg(rot=(1,0,0,0), convention="ros") works
        # because its TiledCamera renderer treats the camera as a virtual
        # sensor that bypasses Hydra mesh-visibility checks for the parent
        # mount (similar to Gazebo). Our raw UsdGeom.Camera authoring goes
        # through standard Hydra and needs the physical clearance.
        xform = UsdGeom.Xformable(cam_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.06))
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.0, 1.0, 0.0, 0.0))

        # Intrinsics (mirror create_additional_camera's Intel-RealSense-ish HFoV/VFoV)
        width, height = int(cfg.get("width", 640)), int(cfg.get("height", 480))
        hfov_deg, vfov_deg = 69.4, 42.5
        fx = width / (2 * np.tan(np.deg2rad(hfov_deg / 2)))
        fy = height / (2 * np.tan(np.deg2rad(vfov_deg / 2)))
        cx, cy = width * 0.5, height * 0.5
        horizontal_aperture_mm = 36.0
        focal_length_mm = fx * horizontal_aperture_mm / width
        vertical_aperture_mm = height * focal_length_mm / fy
        cam.CreateHorizontalApertureAttr().Set(horizontal_aperture_mm)
        cam.CreateVerticalApertureAttr().Set(vertical_aperture_mm)
        cam.CreateFocalLengthAttr().Set(focal_length_mm)
        cam.CreateProjectionAttr().Set("perspective")
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
        # OpenCV pinhole distortion attributes — zero distortion by default
        cam_prim.CreateAttribute("omni:lensdistortion:model", Sdf.ValueTypeNames.String).Set("opencvPinhole")
        cam_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:imageSize", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(width, height))
        cam_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:fx", Sdf.ValueTypeNames.Float).Set(float(fx))
        cam_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:fy", Sdf.ValueTypeNames.Float).Set(float(fy))
        cam_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:cx", Sdf.ValueTypeNames.Float).Set(float(cx))
        cam_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:cy", Sdf.ValueTypeNames.Float).Set(float(cy))
        for attr_name in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"):
            cam_prim.CreateAttribute(f"omni:lensdistortion:opencvPinhole:{attr_name}", Sdf.ValueTypeNames.Float).Set(0.0)

        print(f"[AIC-DT] wrist camera '{name}' authored at {cam_path} ({width}x{height})")
        return cam_path

    def _wrist_camera_graph_path(self, name: str) -> str:
        """Canonical /Graph path for a wrist camera's ROS publish graph."""
        suffix = name.replace("_", " ").title().replace(" ", "")
        return f"/Graph/ActionGraph_{suffix}"

    def _cmd_spawn_wrist_camera(self, name: str) -> Dict[str, Any]:
        try:
            cam_path = self._author_wrist_camera_prim(name)
            return {"status": "success",
                    "message": f"Wrist camera '{name}' Camera prim authored at {cam_path}"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"spawn_wrist_camera({name!r}) failed: {e}"}

    def _cmd_remove_wrist_camera(self, name: str) -> Dict[str, Any]:
        try:
            if name not in WRIST_CAMERAS:
                return {"status": "error", "message": f"Unknown wrist camera '{name}'"}
            stage = omni.usd.get_context().get_stage()
            cam_path = f"{self._robot_prim_path}/{WRIST_CAMERAS[name]['prim_suffix']}"
            # Stop any active stream first (best-effort)
            graph_path = self._wrist_camera_graph_path(name)
            if stage.GetPrimAtPath(graph_path).IsValid():
                stage.RemovePrim(graph_path)
            cam_prim = stage.GetPrimAtPath(cam_path)
            if cam_prim.IsValid():
                stage.RemovePrim(cam_path)
                return {"status": "success", "message": f"Removed Camera prim {cam_path} (and any active stream)"}
            return {"status": "success", "message": f"Camera prim {cam_path} already absent"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"remove_wrist_camera({name!r}) failed: {e}"}

    def _cmd_start_wrist_camera_stream(self, name: str) -> Dict[str, Any]:
        try:
            if name not in WRIST_CAMERAS:
                return {"status": "error", "message": f"Unknown wrist camera '{name}'"}
            cfg = WRIST_CAMERAS[name]
            stage = omni.usd.get_context().get_stage()
            cam_path = f"{self._robot_prim_path}/{cfg['prim_suffix']}"
            # Auto-spawn the Camera prim if missing — convenience for "just start streaming"
            if not stage.GetPrimAtPath(cam_path).IsValid():
                print(f"[AIC-DT] '{name}' Camera prim absent; auto-spawning before stream start")
                self._author_wrist_camera_prim(name)
            self._attach_camera_ros2_writers(
                cam_path, cfg["width"], cfg["height"],
                cfg["topic"], name,
                frame_id=cfg["frame_id"],
                info_topic=cfg["info_topic"],
            )
            return {"status": "success",
                    "message": f"Streaming '{name}' on /{cfg['topic']} + /{cfg['info_topic']}"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"start_wrist_camera_stream({name!r}) failed: {e}"}

    def _cmd_stop_wrist_camera_stream(self, name: str) -> Dict[str, Any]:
        try:
            if name not in WRIST_CAMERAS:
                return {"status": "error", "message": f"Unknown wrist camera '{name}'"}
            detached = self._detach_camera_ros2_writers(name)
            # Also tear down any legacy ActionGraph from prior Helper-based code path
            stage = omni.usd.get_context().get_stage()
            legacy = stage.GetPrimAtPath(self._wrist_camera_graph_path(name))
            if legacy.IsValid():
                stage.RemovePrim(legacy.GetPath())
                detached = True
            if detached:
                return {"status": "success", "message": f"Stopped stream for '{name}'"}
            return {"status": "success", "message": f"No active stream for '{name}' (nothing to stop)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"stop_wrist_camera_stream({name!r}) failed: {e}"}

    # ==================== Additional Camera (workspace/custom) ====================

    def create_additional_camera(self):
        """Create additional camera based on selected view type"""
        from pxr import Gf, Sdf, UsdGeom

        def set_camera_pose(prim_path, position_xyz, quat_xyzw):
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                raise RuntimeError(f"Camera prim '{prim_path}' not found.")
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*position_xyz))
            quat = Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

        def configure_camera_properties(camera_prim, width, height):
            hfov_deg, vfov_deg = 69.4, 42.5
            fx = width / (2 * np.tan(np.deg2rad(hfov_deg / 2)))
            fy = height / (2 * np.tan(np.deg2rad(vfov_deg / 2)))
            cx, cy = width * 0.5, height * 0.5
            horizontal_aperture_mm = 36.0
            focal_length_mm = fx * horizontal_aperture_mm / width
            vertical_aperture_mm = height * focal_length_mm / fy
            camera = UsdGeom.Camera(camera_prim)
            camera.CreateHorizontalApertureAttr().Set(horizontal_aperture_mm)
            camera.CreateVerticalApertureAttr().Set(vertical_aperture_mm)
            camera.CreateFocalLengthAttr().Set(focal_length_mm)
            camera.CreateProjectionAttr().Set("perspective")
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
            camera_prim.CreateAttribute("omni:lensdistortion:model", Sdf.ValueTypeNames.String).Set("opencvPinhole")
            camera_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:imageSize", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(width, height))
            camera_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:fx", Sdf.ValueTypeNames.Float).Set(fx)
            camera_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:fy", Sdf.ValueTypeNames.Float).Set(fy)
            camera_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:cx", Sdf.ValueTypeNames.Float).Set(cx)
            camera_prim.CreateAttribute("omni:lensdistortion:opencvPinhole:cy", Sdf.ValueTypeNames.Float).Set(cy)
            for attr_name in ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]:
                camera_prim.CreateAttribute(f"omni:lensdistortion:opencvPinhole:{attr_name}", Sdf.ValueTypeNames.Float).Set(0.0)

        is_workspace = self._workspace_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()
        resolution_index = self._resolution_combo.model.get_item_value_model().get_value_as_int()
        resolutions = [(640, 480), (1280, 720), (1920, 1080)]
        resolution = resolutions[resolution_index]

        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Error: No stage found")
            return

        if is_workspace:
            prim_path = "/World/workspace_camera"
            position = (0.8572405778988392, -1.3321141046870788, 0.9906567613694909)
            quat_xyzw = (0.4714, 0.1994, 0.3347, 0.7912)
            camera_prim = UsdGeom.Camera.Define(stage, prim_path)
            if camera_prim:
                configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])
                set_camera_pose(prim_path, position, quat_xyzw)
                print(f"Workspace camera created at {prim_path} with resolution {resolution[0]}x{resolution[1]}")

        if is_custom:
            custom_prim_path = self._custom_camera_prim_field.model.get_value_as_string()
            if not custom_prim_path or custom_prim_path.strip() == "":
                print("Error: Please enter a valid camera prim path")
                return
            existing_prim = stage.GetPrimAtPath(custom_prim_path)
            if existing_prim and existing_prim.IsValid():
                camera_prim = UsdGeom.Camera(existing_prim)
            else:
                camera_prim = UsdGeom.Camera.Define(stage, custom_prim_path)
            if camera_prim:
                configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])
                print(f"Custom camera created/updated at {custom_prim_path} with resolution {resolution[0]}x{resolution[1]}")

    def create_additional_camera_actiongraph(self):
        """Create ActionGraph for additional camera ROS2 publishing"""
        is_workspace = self._workspace_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()
        resolution_index = self._resolution_combo.model.get_item_value_model().get_value_as_int()
        resolutions = [(640, 480), (1280, 720), (1920, 1080)]
        width, height = resolutions[resolution_index]
        stage = omni.usd.get_context().get_stage()

        if is_workspace:
            if not stage.GetPrimAtPath("/World/workspace_camera"):
                print("Error: Workspace camera not found. Create it first.")
            else:
                self._attach_camera_ros2_writers(
                    "/World/workspace_camera", width, height,
                    "workspace_camera", "workspace_camera",
                    frame_id="workspace_camera",
                    info_topic="workspace_camera_info",
                )

        if is_custom:
            custom_prim_path = self._custom_camera_prim_field.model.get_value_as_string()
            if not custom_prim_path or custom_prim_path.strip() == "":
                print("Error: Please enter a valid camera prim path")
                return
            topic_name = self._custom_camera_topic_field.model.get_value_as_string()
            if not topic_name or topic_name.strip() == "":
                prim_name = custom_prim_path.split("/")[-1] if "/" in custom_prim_path else custom_prim_path
                topic_name = prim_name.lower().replace(" ", "_").replace("-", "_")
            if not stage.GetPrimAtPath(custom_prim_path):
                print(f"Error: Camera prim not found at {custom_prim_path}. Create it first.")
            else:
                handle_name = topic_name.replace("/", "_").strip("_")
                self._attach_camera_ros2_writers(
                    custom_prim_path, width, height,
                    topic_name, handle_name,
                    frame_id=handle_name,
                    info_topic=f"{topic_name}_info",
                )

    # ==================== Object Management ====================

    def _get_prim_path(self, object_name, folder_path="/World/Objects"):
        """Get the full nested prim path for an AIC object.

        AIC objects use two-level nesting: {folder}/{name}/{name}
        The inner prim has RigidBodyAPI.
        """
        return f"{folder_path}/{object_name}/{object_name}"

    def _get_scene_object_names(self, folder_path="/World/Objects"):
        """Return names of objects currently under /World/Objects (excludes PhysicsMaterial)."""
        stage = omni.usd.get_context().get_stage()
        objects_prim = stage.GetPrimAtPath(folder_path)
        if not objects_prim or not objects_prim.IsValid():
            return []
        return [
            child.GetName() for child in objects_prim.GetChildren()
            if child.IsA(UsdGeom.Xformable) and child.GetName() != "PhysicsMaterial"
        ]

    def _get_prim_bbox_size(self, prim_path):
        """Return (sx, sy, sz) world-aligned bounding box size for a prim."""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return (0.0, 0.0, 0.0)
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(prim)
        size = bbox.ComputeAlignedRange().GetSize()
        return (size[0], size[1], size[2])

    @staticmethod
    def _quat_to_rpy(quat):
        """Convert (w, x, y, z) quaternion to (roll, pitch, yaw) Euler angles (XYZ intrinsic).

        Used by add_objects to bridge the legacy AIC_OBJECTS rotation field
        (quaternion) to the per-component spawn atoms' RPY parameter surface.
        """
        if quat is None:
            return (0.0, 0.0, 0.0)
        w, x, y, z = quat
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def add_objects(self, folder_path="/World/Objects"):
        """Add AIC task board objects to the scene at their configured positions.

        SCENE-01 / DX-02: this is the backwards-compatible CLUBBED spawn path.
        The canonical per-component spawn surface is the new atoms
        (_cmd_spawn_task_board_base, _cmd_spawn_sc_port, _cmd_spawn_nic_card,
        ...). This method preserves the prior 4-prim spawn UX (which the
        UI button "Add All Objects" and `_cmd_add_objects` depend on) by
        invoking the per-component atoms with AIC_OBJECTS legacy defaults
        (NOT the all-`_present=false` defaults that spawn_task_board.launch.py
        ships with — those would spawn nothing). For full launch.py-parity
        per-component params, callers use the per-component atoms directly
        via MCP or UI.

        Uses two-pass loading: create references first, then position.
        Skips entirely if all objects already exist (idempotent).
        """
        stage = omni.usd.get_context().get_stage()

        # Canonical spawn path (Phase 4 / SCENE-01): the per-component spawn atoms
        # author /World/TaskBoard/* — the namespace that load_trial + scoring
        # publishers + spawn_task_board.launch.py-parity all target. The legacy
        # /World/Objects/* path below is fallback-only (asset not vendored).
        #
        # 2026-05-13 reconcile: pre-fix this method threaded AIC_OBJECTS["…"]["position"]
        # values into the spawn-atom kwargs. Those values are OLD-frame world coords
        # from when the robot was at the Isaac-Sim-local origin (-0.18, -0.122, 0);
        # passing them as task-board-local "translation" kwargs put the board on the
        # floor (z=0) with children scrambled. The spawn atoms now ship with
        # Gazebo-canonical defaults verified against aic_gz_bringup.launch.py +
        # task_board.urdf.xacro, so we invoke them no-arg and the parent pre-flight
        # (_ensure_task_board_parent) lands the board on the workbench.
        try:
            # Full canonical task-board population — all 13 parts authored
            # in spawn_task_board.launch.py / task_board.urdf.xacro. Pre-2026-05-13
            # this method only spawned the 4-entry AIC_OBJECTS subset
            # (task_board_base + 2 sc_ports + 1 standalone nic_card), missing the
            # 5 nic_card_mounts + 6 mount rails (LC/SFP/SC × 0/1). The "Add All
            # Objects" UI button now lives up to its name.
            self._cmd_spawn_task_board_base()
            for i in range(5):
                self._cmd_spawn_nic_card_mount(index=i, present=True)
            self._cmd_spawn_sc_port(index=0, present=True)
            self._cmd_spawn_sc_port(index=1, present=True)
            for i in (0, 1):
                self._cmd_spawn_lc_mount_rail(index=i, present=True)
                self._cmd_spawn_sfp_mount_rail(index=i, present=True)
                self._cmd_spawn_sc_mount_rail(index=i, present=True)
            # 2026-05-13: standalone NICCard spawn REMOVED from add_objects.
            # AIC_OBJECTS["nic_card"] is a pre-Phase-4 legacy entry that
            # duplicates the PCB already composed under each NICCardMount_<i>/
            # nic_card_link block. spawn_task_board.launch.py has no
            # nic_card_<n> family — only nic_card_mount_<0..4>. The standalone
            # NICCard's referenced nic_card_visual.glb is the FULL nic card
            # mount assembly (incl. long PCB element Plane_003/Plane_034) and
            # was landing half-inside the base board when placed at task-board
            # origin. The Spawn NIC Card UI button is preserved for callers
            # that explicitly want the legacy standalone, but "Add All Objects"
            # no longer fires it.
            # Cache initial orientations so randomize_object_poses still works
            # for callers that invoke it post-spawn (legacy AIC_OBJECTS rotations
            # remain the per-object identity quaternions used by randomization).
            for obj_name, obj_cfg in AIC_OBJECTS.items():
                rot = obj_cfg.get("rotation")
                if rot is not None:
                    self._initial_orientations[obj_name] = Gf.Quatf(
                        float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
                    )
                else:
                    self._initial_orientations[obj_name] = Gf.Quatf(1, 0, 0, 0)
            print("[add_objects] Spawn atoms succeeded → /World/TaskBoard authored at Gazebo canon; skipping legacy /World/Objects path")
            return True
        except Exception as exc:  # noqa: BLE001 — fall back to legacy path on failure
            print(f"[add_objects] Spawn-atom path failed: {exc} — falling through to legacy /World/Objects path")

        # Check if all objects already exist
        all_exist = True
        if not stage.GetPrimAtPath(folder_path).IsValid():
            all_exist = False
        else:
            for obj_name in AIC_OBJECTS:
                if not stage.GetPrimAtPath(f"{folder_path}/{obj_name}").IsValid():
                    all_exist = False
                    break

        if all_exist:
            print(f"[add_objects] All {len(AIC_OBJECTS)} objects already exist, skipping")
            return False

        # Create Objects folder if needed
        if not stage.GetPrimAtPath(folder_path):
            UsdGeom.Xform.Define(stage, folder_path)

        # Create common physics material
        physics_mat_path = f"{folder_path}/PhysicsMaterial"
        material = UsdShade.Material.Define(stage, physics_mat_path)
        physics_mat_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_mat_api.CreateDynamicFrictionAttr().Set(self._object_dynamic_friction)
        physics_mat_api.CreateRestitutionAttr().Set(self._object_restitution)
        physics_mat_api.CreateStaticFrictionAttr().Set(self._object_static_friction)
        physx_mat_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx_mat_api.CreateFrictionCombineModeAttr().Set(self._object_friction_combine_mode)
        physx_mat_api.CreateRestitutionCombineModeAttr().Set(self._object_restitution_combine_mode)

        print(f"Adding {len(AIC_OBJECTS)} AIC task board objects")

        # First pass: create reference prims
        for obj_name, obj_cfg in AIC_OBJECTS.items():
            prim_path = f"{folder_path}/{obj_name}"
            if stage.GetPrimAtPath(prim_path).IsValid():
                print(f"  Skipping {obj_name} - already exists")
                continue

            usd_path = _local_asset(obj_cfg["usd"])
            prim = stage.DefinePrim(prim_path)
            prim.GetReferences().AddReference(usd_path)

            # Rename Body1 to object_name (standard FMB pattern)
            def _rename_body1(search_prim, target_name):
                for child in search_prim.GetAllChildren():
                    if child.GetName() == "Body1":
                        new_path = child.GetPath().GetParentPath().AppendChild(target_name)
                        omni.kit.commands.execute("MovePrim", path_from=child.GetPath(), path_to=new_path)
                        return True
                    if _rename_body1(child, target_name):
                        return True
                return False
            _rename_body1(stage.GetPrimAtPath(prim_path), obj_name)
            print(f"  Created reference for {obj_name}")

        # Second pass: position each object on its body prim
        for obj_name, obj_cfg in AIC_OBJECTS.items():
            body_path = self._get_prim_path(obj_name, folder_path)
            body_prim = stage.GetPrimAtPath(body_path)
            if not body_prim or not body_prim.IsValid():
                print(f"  Warning: Body prim not found at {body_path}")
                continue

            pos = obj_cfg["position"]
            rot = obj_cfg["rotation"]

            # Set orientation
            if rot is not None:
                quat = Gf.Quatf(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
            else:
                quat = Gf.Quatf(1, 0, 0, 0)

            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{body_path}.xformOp:orient", value=quat, prev=None)
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{body_path}.xformOp:translate",
                value=Gf.Vec3d(pos[0], pos[1], pos[2]), prev=None)

            # Cache initial orientation for randomization (orientations are NEVER randomized)
            self._initial_orientations[obj_name] = quat

            print(f"  Positioned {obj_name} at ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

        # Bind physics material to collision prims
        from omni.physx.scripts import physicsUtils
        physics_mat_sdf_path = Sdf.Path(physics_mat_path)
        objects_prim = stage.GetPrimAtPath(folder_path)
        if objects_prim.IsValid():
            for child in objects_prim.GetChildren():
                if child.GetName() == "PhysicsMaterial":
                    continue
                for desc in Usd.PrimRange(child):
                    if desc.HasAPI(UsdPhysics.CollisionAPI):
                        physicsUtils.add_physics_material_to_prim(stage, desc, physics_mat_sdf_path)

                # Apply collision settings to the body prim
                mesh_path = self._get_prim_path(child.GetName(), folder_path)
                mesh_prim = stage.GetPrimAtPath(mesh_path)
                if mesh_prim and mesh_prim.IsValid():
                    UsdPhysics.CollisionAPI.Apply(mesh_prim)
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                    mesh_collision_api.CreateApproximationAttr(self._collision_approximation)
                    physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
                    physx_collision_api.CreateContactOffsetAttr().Set(self._contact_offset)
                    physx_collision_api.CreateRestOffsetAttr().Set(self._rest_offset)
                    physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(mesh_prim)
                    physx_rb_api.CreateAngularDampingAttr().Set(self._angular_damping)

        print(f"[add_objects] Added {len(AIC_OBJECTS)} AIC task board objects")
        return True

    def delete_objects(self, folder_path="/World/TaskBoard"):
        """Delete the task-board subtree (plus the legacy /World/Objects path).

        2026-05-13 reconcile: default folder_path was /World/Objects pre-fix,
        which is no longer where add_objects authors prims — the new spawn
        atoms write to /World/TaskBoard. With the old default this method
        was a no-op on anything spawned via "Add All Objects" / "Spawn …"
        UI buttons. New default targets /World/TaskBoard so the "Delete
        Objects" UI button does what its name implies. The legacy
        /World/Objects path is still swept as a belt-and-suspenders
        cleanup in case a fallback spawn left prims there.
        """
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        if was_playing:
            timeline.stop()
            print("Stopped simulation before deleting objects")

        stage = omni.usd.get_context().get_stage()
        deleted = []
        for path in (folder_path, "/World/Objects"):
            if path == folder_path and not path:
                continue
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                stage.RemovePrim(path)
                deleted.append(path)
        if deleted:
            print(f"Deleted: {', '.join(deleted)}")
        else:
            print(f"Warning: nothing to delete (neither {folder_path} nor /World/Objects exist)")

        # NOTE: pose-publisher cleanup retired alongside create_pose_publisher() in
        # Plan 04 Task 3 (D-09). The new TF/JointState publisher graphs created in
        # Plan 06 will own their own teardown; delete_objects no longer needs to
        # remove the legacy ActionGraph (which is no longer created in Plan 04+).

        # Clear cached orientations
        self._initial_orientations.clear()

    # ==================== Object Randomization (AIC-specific) ====================

    def randomize_object_poses(self, folder_path="/World/Objects"):
        """Randomize AIC task board object positions using domain randomization.

        From Isaac Lab events.py:
        - Task board base: +/-5mm in x,y
        - SC Port 1: board + offset, x randomize -5mm to +20mm
        - SC Port 2: board + offset, x randomize -5mm to +20mm
        - NIC Card: board + offset, y randomize 0 to +120mm snapped to 40mm steps
        - Orientations: NEVER randomized (cached from initial state)
        """
        stage = omni.usd.get_context().get_stage()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root or not objects_root.IsValid():
            print(f"[randomize] Error: {folder_path} does not exist")
            return 0

        # Get the task board base default position
        base_cfg = AIC_OBJECTS["task_board_base"]
        base_default_pos = np.array(base_cfg["position"])

        # Randomize task board base position (+/-5mm in x,y)
        rand_cfg = AIC_RANDOMIZATION["task_board_base"]
        board_dx = np.random.uniform(*rand_cfg["x_range"])
        board_dy = np.random.uniform(*rand_cfg["y_range"])
        board_pos = base_default_pos + np.array([board_dx, board_dy, 0.0])

        # Apply board position
        board_body_path = self._get_prim_path("task_board_base", folder_path)
        board_prim = stage.GetPrimAtPath(board_body_path)
        if board_prim and board_prim.IsValid():
            quat = self._initial_orientations.get("task_board_base", Gf.Quatf(1, 0, 0, 0))
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{board_body_path}.xformOp:orient", value=quat, prev=None)
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{board_body_path}.xformOp:translate",
                value=Gf.Vec3d(float(board_pos[0]), float(board_pos[1]), float(board_pos[2])),
                prev=None)

        randomized = 1  # board

        # Randomize SC Port 1
        sc1_cfg = AIC_RANDOMIZATION["sc_port_1"]
        sc1_offset = np.array(sc1_cfg["offset_from_board"])
        sc1_dx = np.random.uniform(*sc1_cfg["x_range"])
        sc1_pos = board_pos + sc1_offset + np.array([sc1_dx, 0.0, 0.0])
        self._apply_object_pose("sc_port_1", sc1_pos, folder_path, stage)
        randomized += 1

        # Randomize SC Port 2
        sc2_cfg = AIC_RANDOMIZATION["sc_port_2"]
        sc2_offset = np.array(sc2_cfg["offset_from_board"])
        sc2_dx = np.random.uniform(*sc2_cfg["x_range"])
        sc2_pos = board_pos + sc2_offset + np.array([sc2_dx, 0.0, 0.0])
        self._apply_object_pose("sc_port_2", sc2_pos, folder_path, stage)
        randomized += 1

        # Randomize NIC Card (Y snapped to 40mm steps)
        nic_cfg = AIC_RANDOMIZATION["nic_card"]
        nic_offset = np.array(nic_cfg["offset_from_board"])
        nic_dy = np.random.choice(nic_cfg["y_steps"])
        nic_pos = board_pos + nic_offset + np.array([0.0, nic_dy, 0.0])
        self._apply_object_pose("nic_card", nic_pos, folder_path, stage)
        randomized += 1

        print(f"[randomize] Randomized {randomized} AIC objects (board dx={board_dx*1000:.1f}mm, dy={board_dy*1000:.1f}mm)")
        return randomized

    def _apply_object_pose(self, obj_name, position, folder_path, stage):
        """Apply position to an object, preserving its cached orientation."""
        body_path = self._get_prim_path(obj_name, folder_path)
        body_prim = stage.GetPrimAtPath(body_path)
        if not body_prim or not body_prim.IsValid():
            print(f"  Warning: {obj_name} body prim not found at {body_path}")
            return

        quat = self._initial_orientations.get(obj_name, Gf.Quatf(1, 0, 0, 0))
        omni.kit.commands.execute("ChangeProperty",
            prop_path=f"{body_path}.xformOp:orient", value=quat, prev=None)
        omni.kit.commands.execute("ChangeProperty",
            prop_path=f"{body_path}.xformOp:translate",
            value=Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])),
            prev=None)

    def randomize_single_object(self, object_name, folder_path="/World/Objects"):
        """Randomize a single AIC object's position using its specific randomization rules."""
        if object_name not in AIC_OBJECTS:
            raise ValueError(f"Unknown object: {object_name}. Must be one of: {list(AIC_OBJECTS.keys())}")

        stage = omni.usd.get_context().get_stage()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root or not objects_root.IsValid():
            raise ValueError(f"{folder_path} does not exist")

        # Read the current board position from the scene
        board_body_path = self._get_prim_path("task_board_base", folder_path)
        board_prim = stage.GetPrimAtPath(board_body_path)
        if not board_prim or not board_prim.IsValid():
            raise ValueError("task_board_base not found in scene")

        board_translate = board_prim.GetAttribute("xformOp:translate").Get()
        board_pos = np.array([float(board_translate[0]), float(board_translate[1]), float(board_translate[2])])

        if object_name == "task_board_base":
            # Randomize board position (+/-5mm)
            base_default = np.array(AIC_OBJECTS["task_board_base"]["position"])
            rand_cfg = AIC_RANDOMIZATION["task_board_base"]
            dx = np.random.uniform(*rand_cfg["x_range"])
            dy = np.random.uniform(*rand_cfg["y_range"])
            new_pos = base_default + np.array([dx, dy, 0.0])
            self._apply_object_pose("task_board_base", new_pos, folder_path, stage)
            print(f"[randomize_single] Moved task_board_base by dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm")

        elif object_name in ("sc_port_1", "sc_port_2"):
            rand_cfg = AIC_RANDOMIZATION[object_name]
            offset = np.array(rand_cfg["offset_from_board"])
            dx = np.random.uniform(*rand_cfg["x_range"])
            new_pos = board_pos + offset + np.array([dx, 0.0, 0.0])
            self._apply_object_pose(object_name, new_pos, folder_path, stage)
            print(f"[randomize_single] Placed {object_name} with dx={dx*1000:.1f}mm from board")

        elif object_name == "nic_card":
            rand_cfg = AIC_RANDOMIZATION["nic_card"]
            offset = np.array(rand_cfg["offset_from_board"])
            dy = np.random.choice(rand_cfg["y_steps"])
            new_pos = board_pos + offset + np.array([0.0, dy, 0.0])
            self._apply_object_pose("nic_card", new_pos, folder_path, stage)
            print(f"[randomize_single] Placed nic_card with dy={dy*1000:.1f}mm from board")

    # ==================== Lighting Randomization ====================

    def randomize_lighting(self):
        """Randomize dome light intensity (1500-3500 lux) and color (RGB 0.5-1.0)."""
        from pxr import UsdLux

        stage = omni.usd.get_context().get_stage()
        dome_path = "/World/defaultDomeLight"
        dome_prim = stage.GetPrimAtPath(dome_path)

        if not dome_prim or not dome_prim.IsValid():
            # Try to find any dome light
            for prim in stage.Traverse():
                if prim.IsA(UsdLux.DomeLight):
                    dome_prim = prim
                    dome_path = str(prim.GetPath())
                    break

        if not dome_prim or not dome_prim.IsValid():
            print("[randomize_lighting] No dome light found in scene")
            return

        dome = UsdLux.DomeLight(dome_prim)

        # Randomize intensity (1500-3500 lux)
        intensity = np.random.uniform(1500.0, 3500.0)
        dome.CreateIntensityAttr().Set(float(intensity))

        # Randomize color (RGB 0.5-1.0)
        r = np.random.uniform(0.5, 1.0)
        g = np.random.uniform(0.5, 1.0)
        b = np.random.uniform(0.5, 1.0)
        dome.CreateColorAttr().Set(Gf.Vec3f(float(r), float(g), float(b)))

        print(f"[randomize_lighting] Intensity={intensity:.0f}, Color=({r:.2f}, {g:.2f}, {b:.2f})")

    # ==================== Visibility (Exclude/Include) ====================

    def _build_visibility_ui(self):
        scene_objects = self._get_scene_object_names()
        excluded_names = list(self._hidden_objects.keys())
        with ui.VStack(spacing=5, height=0):
            with ui.HStack(spacing=5):
                ui.Label("Object:", alignment=ui.Alignment.LEFT, width=80)
                if scene_objects:
                    self._vis_object_combo = ui.ComboBox(0, *scene_objects, width=150)
                else:
                    self._vis_object_combo = ui.ComboBox(0, "(none)", width=150)
                ui.Button("Exclude", width=80, height=30, clicked_fn=self._on_hide_clicked)
            with ui.HStack(spacing=5):
                ui.Label("Excluded:", alignment=ui.Alignment.LEFT, width=80)
                if excluded_names:
                    self._vis_hidden_combo = ui.ComboBox(0, *excluded_names, width=150)
                else:
                    self._vis_hidden_combo = ui.ComboBox(0, "(none)", width=150)
                ui.Button("Include", width=80, height=30, clicked_fn=self._on_unhide_clicked)

    def _on_hide_clicked(self):
        scene_objects = self._get_scene_object_names()
        if not scene_objects:
            print("No objects to exclude")
            return
        idx = self._vis_object_combo.model.get_item_value_model().as_int
        if idx < len(scene_objects):
            self.hide_object(scene_objects[idx])
        self._visibility_frame.rebuild()

    def _on_unhide_clicked(self):
        hidden_names = list(self._hidden_objects.keys())
        if not hidden_names:
            print("No excluded objects to include")
            return
        idx = self._vis_hidden_combo.model.get_item_value_model().as_int
        if idx < len(hidden_names):
            self.unhide_object(hidden_names[idx])
        self._visibility_frame.rebuild()

    def _save_all_object_poses(self, folder_path="/World/Objects", exclude=None):
        poses = {}
        for name in self._get_scene_object_names(folder_path):
            if name == exclude:
                continue
            body_path = self._get_prim_path(name, folder_path)
            pose = self._read_prim_pose(body_path)
            if pose:
                poses[name] = pose
        return poses

    def _restore_object_poses(self, poses, folder_path="/World/Objects"):
        for name, pose_data in poses.items():
            body_path = self._get_prim_path(name, folder_path)
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(body_path)
            if not prim or not prim.IsValid():
                continue
            quat = pose_data.get("quaternion", {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
            pos = pose_data.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{body_path}.xformOp:orient",
                value=Gf.Quatf(float(quat["w"]), float(quat["x"]), float(quat["y"]), float(quat["z"])),
                prev=None)
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{body_path}.xformOp:translate",
                value=Gf.Vec3d(pos["x"], pos["y"], pos["z"]),
                prev=None)

    def hide_object(self, object_name, folder_path="/World/Objects"):
        stage = omni.usd.get_context().get_stage()
        prim_path = f"{folder_path}/{object_name}"
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            print(f"Object {object_name} not found at {prim_path}")
            return

        ref_path = None
        prim_spec = stage.GetRootLayer().GetPrimAtPath(prim_path)
        if prim_spec:
            refs = prim_spec.referenceList.prependedItems
            if refs:
                ref_path = refs[0].assetPath

        body_prim_path = self._get_prim_path(object_name, folder_path)
        body_pose = self._read_prim_pose(body_prim_path)

        other_poses = self._save_all_object_poses(folder_path, exclude=object_name)

        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        if was_playing:
            timeline.stop()

        self._hidden_objects[object_name] = {
            "ref_path": ref_path,
            "body_pose": body_pose,
        }

        stage.RemovePrim(prim_path)
        print(f"Hidden (deleted) object: {object_name}")

        self._restore_object_poses(other_poses, folder_path)
        if was_playing:
            timeline.play()

    def unhide_object(self, object_name, folder_path="/World/Objects"):
        if object_name not in self._hidden_objects:
            print(f"Object {object_name} is not in the hidden list")
            return

        data = self._hidden_objects[object_name]
        ref_path = data["ref_path"]
        body_pose = data["body_pose"]

        if not ref_path:
            print(f"Error: No reference path saved for {object_name}, cannot restore")
            del self._hidden_objects[object_name]
            return

        stage = omni.usd.get_context().get_stage()
        prim_path = f"{folder_path}/{object_name}"
        if stage.GetPrimAtPath(prim_path).IsValid():
            print(f"Object {object_name} already exists in scene, removing from hidden list")
            del self._hidden_objects[object_name]
            return

        other_poses = self._save_all_object_poses(folder_path)

        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        if was_playing:
            timeline.stop()

        if not stage.GetPrimAtPath(folder_path):
            UsdGeom.Xform.Define(stage, folder_path)

        prim = stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(ref_path)

        def _rename_body1(search_prim):
            for child in search_prim.GetAllChildren():
                if child.GetName() == "Body1":
                    new_path = child.GetPath().GetParentPath().AppendChild(object_name)
                    omni.kit.commands.execute("MovePrim", path_from=child.GetPath(), path_to=new_path)
                    return True
                if _rename_body1(child):
                    return True
            return False
        _rename_body1(stage.GetPrimAtPath(prim_path))

        # Rebind physics material
        physics_mat_path = f"{folder_path}/PhysicsMaterial"
        if stage.GetPrimAtPath(physics_mat_path).IsValid():
            from omni.physx.scripts import physicsUtils
            physics_mat_sdf_path = Sdf.Path(physics_mat_path)
            obj_prim = stage.GetPrimAtPath(prim_path)
            for desc in Usd.PrimRange(obj_prim):
                if desc.HasAPI(UsdPhysics.CollisionAPI):
                    physicsUtils.add_physics_material_to_prim(stage, desc, physics_mat_sdf_path)

        del self._hidden_objects[object_name]
        print(f"Unhidden (restored) object: {object_name}")

        all_poses = other_poses
        all_poses[object_name] = body_pose

        if was_playing:
            timeline.play()
            async def _restore_after_play():
                app = omni.kit.app.get_app()
                for _ in range(3):
                    await app.next_update_async()
                self._restore_object_poses(all_poses, folder_path)
            asyncio.ensure_future(_restore_after_play())
        else:
            self._restore_object_poses(all_poses, folder_path)

    # ==================== Scene State (Save/Restore) ====================

    def _get_scene_state_dir(self):
        if self._output_dir:
            scene_dir = os.path.join(self._output_dir, "resources", "scene_states")
        else:
            scene_dir = os.path.join(RESOURCES_DIR, "scene_states")
        os.makedirs(scene_dir, exist_ok=True)
        return scene_dir

    def _get_latest_scene_state_path(self):
        scene_dir = self._get_scene_state_dir()
        import glob as glob_mod
        files = glob_mod.glob(os.path.join(scene_dir, "scene_state_*.json"))
        if not files:
            return None
        files.sort()
        return files[-1]

    def _resolve_output_dir(self, output_dir=None):
        if output_dir:
            self._output_dir = os.path.abspath(output_dir)

    def _read_prim_pose(self, prim_path):
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None

            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            scale_attr = prim.GetAttribute("xformOp:scale")

            position = {"x": 0.0, "y": 0.0, "z": 0.0}
            quaternion = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
            scale = {"x": 1.0, "y": 1.0, "z": 1.0}

            if translate_attr.IsValid():
                v = translate_attr.Get()
                if v:
                    position = {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}

            if orient_attr.IsValid():
                v = orient_attr.Get()
                if v:
                    quaternion = {
                        "w": float(v.GetReal()),
                        "x": float(v.GetImaginary()[0]),
                        "y": float(v.GetImaginary()[1]),
                        "z": float(v.GetImaginary()[2])
                    }

            if scale_attr.IsValid():
                v = scale_attr.Get()
                if v:
                    scale = {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}

            return {"position": position, "quaternion": quaternion, "scale": scale}
        except Exception as e:
            carb.log_error(f"Error reading pose for {prim_path}: {e}")
            return None

    def _write_prim_pose(self, prim_path, pose_data):
        try:
            pos = pose_data.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
            quat = pose_data.get("quaternion", {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
            scale = pose_data.get("scale", {"x": 1.0, "y": 1.0, "z": 1.0})

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return False

            translate_attr = prim.GetAttribute("xformOp:translate")
            if translate_attr:
                translate_attr.Set(Gf.Vec3d(pos["x"], pos["y"], pos["z"]))

            orient_attr = prim.GetAttribute("xformOp:orient")
            if orient_attr:
                orient_attr.Set(Gf.Quatf(quat["w"], quat["x"], quat["y"], quat["z"]))

            scale_attr = prim.GetAttribute("xformOp:scale")
            if scale_attr:
                scale_attr.Set(Gf.Vec3d(scale["x"], scale["y"], scale["z"]))

            return True
        except Exception as e:
            carb.log_error(f"Error writing pose for {prim_path}: {e}")
            return False

    # ==================== MCP Socket Server ====================

    def _start_mcp_server(self):
        if self._mcp_server_running:
            print("[MCP] Server is already running")
            return

        self._mcp_server_running = True
        host = "localhost"
        port = MCP_SERVER_PORT

        try:
            self._mcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._mcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._mcp_socket.bind((host, port))
            self._mcp_socket.listen(1)

            self._mcp_server_thread = threading.Thread(target=self._mcp_server_loop)
            self._mcp_server_thread.daemon = True
            self._mcp_server_thread.start()

            print(f"[MCP] Server started on {host}:{port}")
        except Exception as e:
            print(f"[MCP] Failed to start server: {str(e)}")
            self._stop_mcp_server()

    def _stop_mcp_server(self):
        self._mcp_server_running = False

        if self._mcp_socket:
            try:
                self._mcp_socket.close()
            except:
                pass
            self._mcp_socket = None

        if self._mcp_server_thread:
            try:
                if self._mcp_server_thread.is_alive():
                    self._mcp_server_thread.join(timeout=2.0)
            except:
                pass
            self._mcp_server_thread = None

        for t in self._mcp_client_threads:
            try:
                if t.is_alive():
                    t.join(timeout=2.0)
            except:
                pass
        self._mcp_client_threads = []

        print("[MCP] Server stopped")

    def _mcp_server_loop(self):
        self._mcp_socket.settimeout(1.0)

        while self._mcp_server_running:
            try:
                try:
                    client, address = self._mcp_socket.accept()
                    print(f"[MCP] Connected to client: {address}")

                    client_thread = threading.Thread(
                        target=self._handle_mcp_client,
                        args=(client,)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    self._mcp_client_threads = [t for t in self._mcp_client_threads if t.is_alive()]
                    self._mcp_client_threads.append(client_thread)
                except socket.timeout:
                    continue
                except OSError:
                    if not self._mcp_server_running:
                        break
                except Exception as e:
                    if self._mcp_server_running:
                        print(f"[MCP] Error accepting connection: {str(e)}")
                    time.sleep(0.5)
            except Exception as e:
                if self._mcp_server_running:
                    print(f"[MCP] Error in server loop: {str(e)}")
                if not self._mcp_server_running:
                    break
                time.sleep(0.5)

    def _handle_mcp_client(self, client):
        print("[MCP] Client handler started")
        client.settimeout(None)
        buffer = b''

        try:
            while self._mcp_server_running:
                try:
                    data = client.recv(16384)
                    if not data:
                        print("[MCP] Client disconnected")
                        break

                    buffer += data
                    try:
                        command = json.loads(buffer.decode('utf-8'))
                        buffer = b''

                        async def execute_wrapper():
                            try:
                                response = await self._execute_mcp_command(command)
                                response_json = json.dumps(response)
                                print(f"[MCP] response_json: {response_json}")
                                try:
                                    client.sendall(response_json.encode('utf-8'))
                                except:
                                    print("[MCP] Failed to send response - client disconnected")
                            except Exception as e:
                                print(f"[MCP] Error executing command: {str(e)}")
                                traceback.print_exc()
                                try:
                                    error_response = {"status": "error", "message": str(e)}
                                    client.sendall(json.dumps(error_response).encode('utf-8'))
                                except:
                                    pass
                            return None

                        from omni.kit.async_engine import run_coroutine
                        run_coroutine(execute_wrapper())

                    except json.JSONDecodeError:
                        pass
                except Exception as e:
                    print(f"[MCP] Error receiving data: {str(e)}")
                    break
        except Exception as e:
            print(f"[MCP] Error in client handler: {str(e)}")
        finally:
            try:
                client.close()
            except:
                pass
            print("[MCP] Client handler stopped")

    async def _execute_mcp_command(self, command) -> Dict[str, Any]:
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})

            if cmd_type == "list_available_tools":
                return {
                    "status": "success",
                    "result": {
                        "status": "success",
                        "tools": MCP_TOOL_REGISTRY
                    }
                }

            handler_name = MCP_HANDLERS.get(cmd_type)
            if handler_name:
                handler = getattr(self, handler_name, None)
            else:
                handler = None

            if handler:
                try:
                    print(f"[MCP] Executing handler for {cmd_type}")
                    import inspect
                    if inspect.iscoroutinefunction(handler):
                        result = await handler(**params)
                    else:
                        result = handler(**params)
                    print(f"[MCP] Handler execution complete: {result}")
                    if result and result.get("status") == "success":
                        return {"status": "success", "result": result}
                    else:
                        return {"status": "error", "message": result.get("message", "Unknown error")}
                except Exception as e:
                    print(f"[MCP] Error in handler: {str(e)}")
                    traceback.print_exc()
                    return {"status": "error", "message": str(e)}
            else:
                return {"status": "error", "message": f"Unknown command type: {cmd_type}"}

        except Exception as e:
            print(f"[MCP] Error executing command: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    # ==================== MCP Command Handlers ====================

    def _cmd_execute_python_code(self, code: str, session_id: str = "default", persistent: bool = False) -> Dict[str, Any]:
        """Execute Python code in Isaac Sim's environment with persistent sessions."""
        _builtin_keys = {"omni", "carb", "Usd", "UsdGeom", "Sdf", "Gf", "__builtins__"}

        try:
            exec_globals = {
                "omni": omni, "carb": carb,
                "Usd": Usd, "UsdGeom": UsdGeom, "Sdf": Sdf, "Gf": Gf,
                "__builtins__": __builtins__,
            }

            if persistent and session_id in self._python_sessions:
                exec_globals.update(self._python_sessions[session_id])

            old_stdout = sys.stdout
            sys.stdout = capture = io.StringIO()
            try:
                exec(code, exec_globals)
                output = capture.getvalue()
            except Exception as e:
                output = capture.getvalue()
                output += f"ERROR: {e}\n{traceback.format_exc()}"
                carb.log_error(f"Error executing code: {e}")
                return {
                    "status": "error", "message": str(e),
                    "output": output, "traceback": traceback.format_exc(),
                }
            finally:
                sys.stdout = old_stdout

            if persistent:
                saved = {}
                for k, v in exec_globals.items():
                    if k.startswith("_") or k in _builtin_keys:
                        continue
                    saved[k] = v
                self._python_sessions[session_id] = saved

            result = exec_globals.get("result", None)
            response = {
                "status": "success", "message": "Code executed successfully",
                "output": output, "result": result,
            }
            if persistent:
                response["session_id"] = session_id
                response["session_vars"] = list(self._python_sessions.get(session_id, {}).keys())
            return response
        except Exception as e:
            carb.log_error(f"Error executing code: {e}")
            traceback.print_exc()
            return {"status": "error", "message": str(e), "traceback": traceback.format_exc()}

    def _cmd_play_scene(self) -> Dict[str, Any]:
        try:
            self._timeline.play()
            return {"status": "success", "message": "Simulation started"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to start simulation: {str(e)}"}

    def _cmd_stop_scene(self) -> Dict[str, Any]:
        try:
            self._timeline.stop()
            return {"status": "success", "message": "Simulation stopped"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to stop simulation: {str(e)}"}

    def _cmd_load_scene(self) -> Dict[str, Any]:
        try:
            from omni.kit.async_engine import run_coroutine
            run_coroutine(self.load_scene())
            return {"status": "success", "message": "Scene loaded (physics, hidden ground plane, AIC enclosure + walls, AIC floor, Gazebo-parity lights, Default light rig)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_load_robot(self,
                        robot_x: float = None, robot_y: float = None, robot_z: float = None,
                        robot_roll: float = 0.0, robot_pitch: float = 0.0, robot_yaw: float = None,
                        cable_x: float = 0.172, cable_y: float = 0.024, cable_z: float = 1.518,
                        cable_roll: float = 0.4432, cable_pitch: float = -0.48, cable_yaw: float = 1.3303
                        ) -> Dict[str, Any]:
        """SCENE-04: forward Gazebo-named pose params through to load_robot.

        See load_robot() docstring for the Gazebo parameter mapping and
        backwards-compat default semantics. Cable params are wired through
        to a no-op-effective transform per D-04 (cable SetActive(False) in
        Phase 1; Phase 3 enables physics).
        """
        try:
            from omni.kit.async_engine import run_coroutine
            run_coroutine(self.load_robot(
                robot_x=robot_x, robot_y=robot_y, robot_z=robot_z,
                robot_roll=robot_roll, robot_pitch=robot_pitch, robot_yaw=robot_yaw,
                cable_x=cable_x, cable_y=cable_y, cable_z=cable_z,
                cable_roll=cable_roll, cable_pitch=cable_pitch, cable_yaw=cable_yaw,
            ))
            return {"status": "success",
                    "message": "UR5e loaded with integrated Robotiq Hand-E gripper at /World/UR5e (SCENE-04 pose params applied)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_setup_action_graph(self) -> Dict[str, Any]:
        try:
            from omni.kit.async_engine import run_coroutine
            run_coroutine(self.setup_action_graph())
            return {"status": "success", "message": "ROS2 action graph created for UR5e joint control"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_setup_force_publisher(self) -> Dict[str, Any]:
        try:
            self.setup_force_publish_action_graph()
            return {"status": "success", "message": "Force publisher action graph created for UR5e"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_setup_tf_publisher(self) -> Dict[str, Any]:
        try:
            self.setup_tf_publish_action_graph()
            return {"status": "success", "message": "TF publisher action graph created (/tf + /tf_static)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_setup_joint_state_publisher(self) -> Dict[str, Any]:
        try:
            self.setup_joint_state_publish_action_graph()
            return {"status": "success", "message": "JointState publisher action graph created on /joint_states"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_setup_controller_subscribers(self) -> Dict[str, Any]:
        """MCP atom — start the AicControllerLoop's joint_commands + pose_commands + controller_state surfaces.

        Implements PARITY-09 + PARITY-10 + PARITY-11 wiring (Plans 02-03/02-04/02-05 fill in callback bodies).
        """
        try:
            self._start_aic_controller_loop()
            return {"status": "success",
                    "message": "AIC controller loop started (subs: /aic_controller/joint_commands + /aic_controller/pose_commands; pub: /aic_controller/controller_state)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"setup_controller_subscribers failed: {str(e)}"}

    def _cmd_setup_offlimit_contacts(self, prim_paths: list = None) -> Dict[str, Any]:
        """MCP atom — start (or re-configure) the AicControllerLoop's off-limit contact monitoring + Contacts publish surface.

        Implements PARITY-06 wiring (Plan 02-06 fills in omni.physx contact-report subscription + Contacts publish).
        """
        try:
            self._start_aic_controller_loop(off_limit_prims=prim_paths)
            n = len(prim_paths) if prim_paths else 0
            return {"status": "success",
                    "message": f"Off-limit contact subscription active ({n} prim_paths overridden; defaults used otherwise)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"setup_offlimit_contacts failed: {str(e)}"}

    # Gripper finger joint paths (paired with build_cable_variant_usds.py::
    # author_gripper_prismatic_joints — both joints converted from
    # PhysicsFixedJoint to PhysicsPrismaticJoint + DriveAPI).
    _GRIPPER_FINGER_JOINT_PATHS = (
        "/World/UR5e/aic_unified_robot/joints/gripper_left_finger_joint",
        "/World/UR5e/aic_unified_robot/joints/gripper_right_finger_joint",
    )

    def _cmd_gripper_command(self, position: float = 0.00655) -> Dict[str, Any]:
        """MCP atom — set Robotiq Hand-E gripper width by writing drive target
        on both finger prismatic joints.

        Mimic semantics: URDF defines right_finger_joint as <mimic joint=left
        multiplier=1 offset=0>, but USD/PhysX has no mimic concept. This atom
        enforces mimic at write — both joints receive the same target.

        Write strategy: direct USD attribute Set on
        `drive:linear:physics:targetPosition`. PhysX picks up the new target
        on the next solver step (same pattern as _configure_arm_drives). This
        is the safe path because Isaac Sim 5.0 Articulation DOF ordering is
        NOT documented (per nvidia-suite-docs consultation 2026-05-17; forum
        topic 330417 shows URDF→USD ordering is not preserved and no NVIDIA
        staff has documented the rule). USD-attribute writes bypass the
        index-lookup ambiguity entirely.

        Args:
          position: Per-finger joint coordinate in meters. URDF range
            [grip_pos_min=0, grip_pos_max=0.025]. Default 0.00655 matches
            sample_config.yaml's gripper_initial_pos default. Right finger
            mirrors left via authored localRot0 = 180°-about-Z.

        Returns success with the position actually applied. Out-of-range
        values are clamped with a warning; the URDF limits authored on both
        prismatic joints make PhysX clamp anyway, but we surface it at the
        atom layer for caller diagnostics.
        """
        try:
            # Clamp to URDF grip_pos_min/max
            clamped = max(0.0, min(0.025, float(position)))
            if clamped != float(position):
                print(f"[AIC-DT][gripper] position {position} clamped to [0, 0.025] → {clamped}")
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return {"status": "error", "message": "no stage"}
            n_set = 0
            n_missing = 0
            for joint_path in self._GRIPPER_FINGER_JOINT_PATHS:
                joint = stage.GetPrimAtPath(joint_path)
                if not joint.IsValid():
                    print(f"[AIC-DT][gripper] joint missing: {joint_path} — "
                          f"USD likely pre-prismatic-conversion (rebuild cable USD)")
                    n_missing += 1
                    continue
                attr = joint.GetAttribute("drive:linear:physics:targetPosition")
                if not attr.IsValid():
                    print(f"[AIC-DT][gripper] drive:linear:physics:targetPosition "
                          f"missing on {joint_path} — DriveAPI not applied "
                          f"(rebuild cable USD via author_gripper_prismatic_joints)")
                    n_missing += 1
                    continue
                attr.Set(clamped)
                n_set += 1
            if n_set == 0:
                return {
                    "status": "error",
                    "message": (
                        f"No gripper drive targets written ({n_missing} joint(s) missing). "
                        f"Rebuild the cable USD via "
                        f"exts/aic-dt/scripts/build_cable_variant_usds.py to author "
                        f"PhysicsPrismaticJoint + DriveAPI."
                    ),
                }
            return {
                "status": "success",
                "message": (
                    f"Gripper drive target set to {clamped:.5f} m on {n_set} "
                    f"finger joint(s); mirror via authored localRot0=180°Z."
                ),
                "position": clamped,
            }
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"gripper_command failed: {str(e)}"}

    def _cmd_attach_cable_to_gripper(self,
                                     plug_link_path: str = "/World/UR5e/cable/Rope/Rope/link_20",
                                     finger_link_path: str = "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l",
                                     gripper_initial_pos: float = 0.00655) -> Dict[str, Any]:
        """MCP atom — author UsdPhysics.FixedJoint connecting cable plug-end to gripper finger.

        Implements SCENE-03 (Plan 03-03). Idempotent — replaces any prior CableAttachJoint.
        """
        try:
            joint_path = self._attach_cable_to_gripper_impl(
                gripper_initial_pos=gripper_initial_pos,
                plug_link_path=plug_link_path,
                finger_link_path=finger_link_path,
            )
            return {"status": "success",
                    "message": f"CableAttachJoint authored at {joint_path}"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"attach_cable_to_gripper failed: {str(e)}"}

    # ====================================================================
    # Phase 4 / Plan 04-02 — Trial loader (TRIAL-01 + TRIAL-02)
    #
    # _cmd_load_trial parses a single trial entry from sample_config.yaml-format
    # YAML and dispatches to the existing per-component spawn atoms (Plan 01-09)
    # + load_robot (Phase 3). ground_truth=True (default) starts /scoring/*
    # publishers; False suppresses them (M2 swap surface).
    #
    # Adapter strategy (per 04-RESEARCH.md):
    #   - YAML cables.<name>.pose.gripper_offset.{x,y,z} → load_robot's
    #     cable_x/y/z (Option A pass-through; informative offset, not absolute).
    #   - entity_name in nic_rail_<i> / sc_rail_<i> / *_mount_rail_<i> blocks
    #     is IGNORED — informational for aic_engine; spawn atoms use
    #     hardcoded asset USDs per AIC_OBJECTS.
    #
    # Helpers (private methods on DigitalTwin):
    #   - _extract_cable_kwargs(scene)  → kwargs forwarded to load_robot
    #   - _dispatch_rail_block(kind, index, block, atom_fn) → dispatches one
    #     YAML scene-block entry to its spawn atom; returns (label, kwargs)
    # ====================================================================

    def _extract_cable_kwargs(self, scene: dict) -> dict:
        """Q6 cable extraction (04-RESEARCH.md): take the first cables.<name>
        entry; flatten gripper_offset to absolute cable_x/y/z (Option A per
        Q2). If no cables block exists, return {} so load_robot falls back
        to its Phase-3 defaults.
        """
        cables = scene.get("cables", {}) or {}
        if not cables:
            return {}
        # First-cable-wins (Q6): aic_engine consumes one cable per trial.
        cable_name, cable_cfg = next(iter(cables.items()))
        pose = (cable_cfg or {}).get("pose", {}) or {}
        offset = pose.get("gripper_offset", {}) or {}
        return {
            "cable_x": float(offset.get("x", 0.172)),
            "cable_y": float(offset.get("y", 0.024)),
            "cable_z": float(offset.get("z", 1.518)),
            "cable_roll": float(pose.get("roll", 0.4432)),
            "cable_pitch": float(pose.get("pitch", -0.48)),
            "cable_yaw": float(pose.get("yaw", 1.3303)),
            "cable_type": cable_cfg.get("cable_type", "sfp_sc_cable"),
            "attach_cable_to_gripper": bool(cable_cfg.get("attach_cable_to_gripper", False)),
        }

    def _compute_trial_tf_frames(self, trial_cfg: dict, spawned: list) -> list:
        """DEFERRED-4 (2026-05-10): build per-trial frame dicts for CheatCode's
        `lookup_transform(base_link, …)` calls.

        CheatCode.insert_cable() (~/Documents/aic/aic_example_policies/.../CheatCode.py:197-204)
        constructs two frame_ids from the trial task config:
          - port_frame  = f"task_board/{target_module_name}/{port_name}_link"
          - plug_frame  = f"{cable_name}/{plug_name}_link"
        and waits up to 10s for both to resolve from base_link.

        Source-of-truth for the mount→port SE(3) offset:
          ~/Documents/aic/tools/anchor_target_offsets.yaml
        which contains cached translation_xyz + 4x4 rowmajor SE(3) matrices
        per CAD pair (computed from real Gazebo recordings). We look up the
        anchor matching the spawned anchor link, then compose:
          T_world_port = T_world_anchor @ T_anchor_port

        For the plug frame: cable is rigidly attached to gripper finger via
        FixedJoint when attach_cable_to_gripper=True (the only case in the 3
        sample_config trials). The plug tip is therefore approximately the
        gripper/tcp pose itself. We publish the plug frame as a static child
        of gripper/tcp with zero offset — CheatCode's relative-motion logic
        (plug_tip_gripper_offset = gripper - plug) gracefully collapses to
        moving the gripper directly to the port. This is the same approach
        Gazebo's TouchPlugin uses for the trial: the plug sensor frame sits
        at the contact point on the plug visual which is itself near the
        gripper finger tip.

        Returns a list of dicts in the shape expected by
        AicScoringPublishers.set_trial_tf_frames:
          [
            {parent: 'world', child: 'task_board/.../...', usd_anchor_path: '...',
             offset_translation: (x,y,z), offset_quat_xyzw: (x,y,z,w)},
            {parent: 'gripper/tcp', child: 'cable_0/sfp_tip_link',
             offset_translation: (0,0,0), offset_quat_xyzw: (0,0,0,1)},
          ]
        Empty list if the trial has no tasks or if anchor offsets are missing
        (logged as a single line to stdout for operator triage).
        """
        import yaml as _yaml
        tasks = (trial_cfg or {}).get("tasks", {}) or {}
        if not tasks:
            return []
        # Single task per trial across the 3 sample_config trials. Take first.
        _, task = next(iter(tasks.items()))
        task = task or {}
        target_module = task.get("target_module_name")
        port_name = task.get("port_name")
        cable_name = task.get("cable_name")
        plug_name = task.get("plug_name")
        if not all((target_module, port_name, cable_name, plug_name)):
            print(f"[AIC-DT][trial-tf] task missing required fields: {task}")
            return []

        # Load anchor offsets (cheap; only on load_trial).
        offsets_path = os.path.expanduser("~/Documents/aic/tools/anchor_target_offsets.yaml")
        try:
            with open(offsets_path, "r") as fh:
                offsets_doc = _yaml.safe_load(fh) or {}
        except Exception as exc:
            print(f"[AIC-DT][trial-tf] cannot read {offsets_path}: {exc!r}")
            return []
        anchors = offsets_doc.get("anchors", {}) or {}

        # Map the target_module to an anchors-yaml key + live USD mount path.
        # Naming convention from sample_config.yaml:
        #   target_module_name='nic_card_mount_<i>'   → anchor key 'nic_card_mount'
        #                                               → live USD '/World/TaskBoard/NICCardMount_<i>'
        #   target_module_name='sc_port_<i>'          → anchor key 'sc_port'
        #                                               → live USD '/World/TaskBoard/SCPort_<i>'
        if target_module.startswith("nic_card_mount_"):
            idx = target_module.rsplit("_", 1)[-1]
            anchor_key = "nic_card_mount"
            usd_anchor_path = f"/World/TaskBoard/NICCardMount_{idx}"
        elif target_module.startswith("sc_port_"):
            idx = target_module.rsplit("_", 1)[-1]
            anchor_key = "sc_port"
            usd_anchor_path = f"/World/TaskBoard/SCPort_{idx}"
        else:
            print(f"[AIC-DT][trial-tf] unrecognised target_module_name pattern: {target_module}")
            return []

        anchor_info = anchors.get(anchor_key) or {}
        translation = anchor_info.get("translation_xyz_m") or anchor_info.get("translation_xyz")
        T44 = anchor_info.get("T_anchor_port_4x4_rowmajor")
        if not (translation and T44):
            print(f"[AIC-DT][trial-tf] no offsets for anchor_key={anchor_key} in {offsets_path}")
            return []

        # Extract rotation from the 4x4 rowmajor as a quaternion (xyzw).
        # The yaml stores it as a flat 16-element list. Use numpy/pxr to convert.
        try:
            import numpy as np
            from pxr import Gf
            T = np.array(T44, dtype=float).reshape(4, 4)
            rot33 = T[:3, :3]
            # Build a Gf rotation matrix, then extract quaternion.
            gfm = Gf.Matrix3d(*[float(rot33[i, j]) for i in range(3) for j in range(3)])
            quat = gfm.ExtractRotation().GetQuaternion()
            qw = float(quat.GetReal())
            qi = quat.GetImaginary()
            qx, qy, qz = float(qi[0]), float(qi[1]), float(qi[2])
        except Exception as exc:
            print(f"[AIC-DT][trial-tf] rotation extraction failed: {exc!r}; falling back to identity")
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        # parent="base_link" — see scoring_publishers._publish_trial_tf_frames
        # Mode 1 doc: parity_publishers's "world" is the robot's local-origin
        # USD prim, NOT the simulation world. Publishing as base_link-relative
        # gives CheatCode the geometrically correct port pose regardless of
        # where the robot is mounted in the world.
        port_frame = {
            "parent": "base_link",
            "child": f"task_board/{target_module}/{port_name}_link",
            "usd_anchor_path": usd_anchor_path,
            "offset_translation": (float(translation[0]), float(translation[1]), float(translation[2])),
            "offset_quat_xyzw": (qx, qy, qz, qw),
        }

        # motion-fidelity-cheatcode-timing (2026-05-11): port_entrance frame.
        # ScoringTier2.cc::ComputeTier3Score (line 753) queries
        # `<task_board>/<target_module>/<port_name>_link_entrance` for the
        # partial-insertion bonus geometry. Gazebo's NIC Card Mount SDF
        # (~/Documents/aic/aic_assets/models/NIC Card Mount/model.sdf:374)
        # authors this as a fixed child of port_link at port-local pose
        # (0, 0, -0.0458). We compose the same offset onto our published
        # port frame so the engine's lookup succeeds.
        # Note: -45.8mm Z in port-local frame, which is the insertion
        # depth (entrance sits 4.58cm above the seated-port datum).
        port_entrance_offset_local = (0.0, 0.0, -0.0458)
        try:
            import numpy as _np
            from pxr import Gf as _Gf
            # Build T_anchor_entrance = T_anchor_port @ T_port_entrance.
            T_anchor_port = _np.array(T44, dtype=float).reshape(4, 4)
            T_port_entrance = _np.eye(4)
            T_port_entrance[0, 3] = port_entrance_offset_local[0]
            T_port_entrance[1, 3] = port_entrance_offset_local[1]
            T_port_entrance[2, 3] = port_entrance_offset_local[2]
            T_anchor_entrance = T_anchor_port @ T_port_entrance
            ent_trans = (
                float(T_anchor_entrance[0, 3]),
                float(T_anchor_entrance[1, 3]),
                float(T_anchor_entrance[2, 3]),
            )
            ent_rot33 = T_anchor_entrance[:3, :3]
            _gfm = _Gf.Matrix3d(*[float(ent_rot33[i, j]) for i in range(3) for j in range(3)])
            _q = _gfm.ExtractRotation().GetQuaternion()
            ent_qw = float(_q.GetReal())
            _qi = _q.GetImaginary()
            ent_qxyzw = (float(_qi[0]), float(_qi[1]), float(_qi[2]), ent_qw)
        except Exception as exc:
            print(f"[AIC-DT][trial-tf] port_entrance offset compose failed: {exc!r}; using port-aligned identity")
            ent_trans = (float(translation[0]), float(translation[1]), float(translation[2]) - 0.0458)
            ent_qxyzw = (qx, qy, qz, qw)
        port_entrance_frame = {
            "parent": "base_link",
            "child": f"task_board/{target_module}/{port_name}_link_entrance",
            "usd_anchor_path": usd_anchor_path,
            "offset_translation": ent_trans,
            "offset_quat_xyzw": ent_qxyzw,
        }

        # Plug frame — anchor on the real plug-visual rigid body so CheatCode
        # commands the gripper to a pose where the PHYSICAL plug reaches the
        # port, not where the gripper TCP reaches the port. The plug body is
        # attached to the gripper finger via FixedJoint in
        # _attach_cable_to_gripper_impl; its world pose moves with the gripper
        # but with the correct ~33mm offset that the prior gripper/tcp-zero
        # anchor was lying about. Publish base_link-relative (Mode 1) so it
        # composes correctly with the port_frame above.
        # iter 7 Path (a') 2026-05-12: anchor plug frame on the kinematic
        # plug_proxy (USD-hierarchy child of finger_link_l) instead of
        # sc_plug_visual. sc_plug_visual is decorative — it sits at the
        # cable's Gazebo-canonical world spawn pose (~70cm from gripper at
        # home) and only drifts ~30mm under gripper motion because the cable
        # rope chain has near-zero mass + no working joint to the gripper.
        # plug_proxy, in contrast, tracks the gripper's finger_link_l world
        # pose perfectly via USD transform composition. CheatCode reads this
        # /tf plug frame and commands gripper until the frame coincides with
        # the port frame — accurate motion targeting.
        plug_proxy_usd_path = "/World/UR5e/aic_unified_robot/gripper_hande_finger_link_l/plug_proxy"
        plug_frame = {
            "parent": "base_link",
            "child": f"{cable_name}/{plug_name}_link",
            "usd_anchor_path": plug_proxy_usd_path,
            "offset_translation": (0.0, 0.0, 0.0),
            "offset_quat_xyzw": (0.0, 0.0, 0.0, 1.0),
        }

        print(f"[AIC-DT][trial-tf] frames: port={port_frame['child']} (anchor={usd_anchor_path}, "
              f"translation={port_frame['offset_translation']}); entrance={port_entrance_frame['child']} "
              f"(translation={port_entrance_frame['offset_translation']}); "
              f"plug={plug_frame['child']} (anchor={plug_proxy_usd_path})")
        return [port_frame, port_entrance_frame, plug_frame]

    def _apply_home_joint_positions(self, robot_cfg: dict) -> bool:
        """trial-home-robot-pose (2026-05-11): drive the 6 arm joints to the
        config-specified home pose so CheatCode starts trials with the gripper
        above the task board.

        sample_config.yaml top-level `robot.home_joint_positions:` maps the
        6 arm joint names (shoulder_pan/lift, elbow, wrist_1/2/3) → radians.
        Engine consumes the same block in its own home_robot() but only runs
        that path when skip_ready_simulator=false. We pass it true (aic-dt
        owns scene authoring); without this helper, the robot sits at
        load_robot defaults and the gripper is ~600mm from any port.

        Applies via the existing controller_loop articulation handle:
          - Pulls the live articulation from self._aic_controller_loop
          - Uses set_joint_positions + apply_action with joint_indices=[[0..5]]
            (same pattern as controller_loop._apply_joint_cmd; this scopes
            on the 6 arm joints even after Phase-3 cable activation expanded
            the articulation to 46 DOFs).
        Returns True if values landed, False if articulation wasn't ready /
        the cfg missed the home_joint_positions block.
        """
        home = (robot_cfg or {}).get("home_joint_positions") or {}
        if not home:
            print("[AIC-DT][home] robot.home_joint_positions absent — skipping")
            return False
        # ARM_JOINTS_ORDERED is the canonical ordering used by controller_loop.
        try:
            from .controller_loop import ARM_JOINTS_ORDERED
        except Exception as exc:
            print(f"[AIC-DT][home] import ARM_JOINTS_ORDERED failed: {exc!r}")
            return False
        missing = [n for n in ARM_JOINTS_ORDERED if n not in home]
        if missing:
            print(f"[AIC-DT][home] home_joint_positions missing joints {missing} — skipping")
            return False
        home_positions = [float(home[n]) for n in ARM_JOINTS_ORDERED]
        # Reach the live articulation. controller_loop holds it after physics
        # tick 30 (lazy init); if it hasn't initialized yet, our short retry
        # loop below pumps the timeline to give it a chance.
        cl = getattr(self, "_aic_controller_loop", None)
        if cl is None or getattr(cl, "_articulation", None) is None:
            print("[AIC-DT][home] controller_loop articulation not ready yet — waiting up to 5s")
            import asyncio
            for _ in range(50):
                cl = getattr(self, "_aic_controller_loop", None)
                if cl is not None and getattr(cl, "_articulation", None) is not None \
                        and hasattr(cl._articulation, "_physics_view") \
                        and cl._articulation._physics_view is not None:
                    break
                try:
                    asyncio.get_event_loop().run_until_complete(asyncio.sleep(0.1))
                except RuntimeError:
                    import time
                    time.sleep(0.1)
        if cl is None or getattr(cl, "_articulation", None) is None:
            print("[AIC-DT][home] controller_loop articulation never ready — skipping home pose")
            return False
        artic = cl._articulation
        try:
            import numpy as np
            from isaacsim.core.utils.types import ArticulationActions
            arm_indices = np.array([[0, 1, 2, 3, 4, 5]])
            pos_arr = np.array([home_positions], dtype=np.float32)
            # Direct write (bypass PD) so the robot snaps to home pose.
            artic.set_joint_positions(pos_arr, joint_indices=arm_indices)
            # apply_action so the PD target also matches (otherwise the next PD
            # step would pull the joints back toward the previous target).
            action = ArticulationActions(
                joint_positions=pos_arr,
                joint_indices=arm_indices,
            )
            artic.apply_action(action)
            print(f"[AIC-DT][home] applied home pose: {dict(zip(ARM_JOINTS_ORDERED, home_positions))}")
            return True
        except Exception as exc:
            print(f"[AIC-DT][home] articulation write failed: {exc!r}")
            return False

    def _dispatch_rail_block(self, kind: str, index: int, block: dict, atom_fn) -> tuple:
        """Q5 adapter (04-RESEARCH.md): YAML's entity_present/entity_pose.{translation,roll,pitch,yaw}
        → atom kwargs. Returns (atom_name_with_index, kwargs) for the
        spawned_components return list. Ignores entity_name (informational).

        Some atoms (spawn_nic_card) have no `index` parameter — the helper
        introspects via inspect.signature and strips `index` as needed.
        """
        import inspect  # lazy import (Phase 2 pattern)
        block = block or {}
        pose = block.get("entity_pose", {}) or {}
        kwargs = {
            "index": index,
            "present": bool(block.get("entity_present", False)),
            "translation": float(pose.get("translation", 0.0)),
            "roll":  float(pose.get("roll", 0.0)),
            "pitch": float(pose.get("pitch", 0.0)),
            "yaw":   float(pose.get("yaw", 0.0)),
        }
        sig = inspect.signature(atom_fn)
        if "index" not in sig.parameters:
            kwargs.pop("index")
        atom_fn(**kwargs)
        return (f"{kind}_{index}", kwargs)

    async def _cmd_load_trial(self,
                              config_path: str = None,
                              trial_key: str = "trial_1",
                              ground_truth: bool = True) -> Dict[str, Any]:
        """TRIAL-01/02 (D-01..D-05): Spawn a sample_config.yaml trial and start sim.

        See .planning/phases/04-trial-loader/04-CONTEXT.md (D-01..D-05) and
        04-RESEARCH.md (Q2 schema audit, Q5 spawn-atom field gaps, Q6 cable
        extraction).

        Adapter strategy (per 04-RESEARCH.md Option A):
          - YAML cables.<name>.pose.gripper_offset.{x,y,z} → load_robot's
            cable_x/y/z (small offset, ≤4.5cm; passes through cleanly).
          - entity_name in *rail* blocks is IGNORED — informational only for
            aic_engine consumption; spawn atoms use hardcoded asset USDs per
            AIC_OBJECTS.

        Args:
            config_path: Path to sample_config.yaml. Default
                ~/Documents/aic/aic_engine/config/sample_config.yaml (~ expanded).
            trial_key: YAML key under 'trials' (e.g. 'trial_1').
            ground_truth: D-04 gate — if True (default), starts /scoring/*
                publishers; if False, suppresses them (M2 swap surface).

        Returns:
            {"status": "success", "trial": trial_key, "ground_truth": bool,
             "spawned_components": [(atom_name_with_index, kwargs), ...]}
            or {"status": "error", "message": ..., "traceback": ...}.
        """
        # Lazy imports (Phase 2 pattern — keep module load Kit-free)
        import yaml
        try:
            if config_path is None:
                config_path = os.path.expanduser(
                    "~/Documents/aic/aic_engine/config/sample_config.yaml")
            else:
                config_path = os.path.expanduser(config_path)

            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)

            trials = (cfg or {}).get("trials", {}) or {}
            if trial_key not in trials:
                return {
                    "status": "error",
                    "message": f"trial_key {trial_key!r} not found in {config_path}; available: {sorted(trials.keys())}",
                }

            scene = ((trials[trial_key] or {}).get("scene", {})) or {}
            cable_kwargs = self._extract_cable_kwargs(scene)

            print(f"=== load_trial({trial_key}, ground_truth={ground_truth}) ===")
            spawned: list = []

            # 1. Fresh stage (D-02 idempotency)
            await self._cmd_new_stage()

            # 2. load_scene (physics + ground + dome + enclosure)
            print("--- Loading scene ---")
            await self.load_scene()

            # 3. load_robot with cable kwargs from YAML + Gazebo-faithful robot base pose
            # (~/Documents/aic/aic_bringup/launch/aic_gz_bringup.launch.py declares
            #  robot_x=-0.2, robot_y=0.2, robot_z=1.14, robot_yaw=-3.141 — table-
            #  mounted UR5e facing back toward the operator). Without these the robot
            #  sits at world origin and the gripper TCP at home pose lands ~930mm BELOW
            #  the task_board, making CheatCode's xy_error ~600+mm at trial start —
            #  unreachable in the 180s task time_limit even with cable physics working.
            robot_base_kwargs = {
                "robot_x": -0.2,
                "robot_y": 0.2,
                "robot_z": 1.14,
                "robot_roll": 0.0,
                "robot_pitch": 0.0,
                "robot_yaw": -3.141,
            }
            print(f"--- Importing UR5e (robot_base={robot_base_kwargs}, cable_kwargs={cable_kwargs}) ---")
            await self.load_robot(**robot_base_kwargs, **cable_kwargs)

            # 4. Early play (Phase 1 D-12 non-negotiable order)
            print("--- Playing scene early (before graphs/objects) ---")
            self._timeline.play()

            # 5. TF + JointState publishers (PARITY-03/04)
            self.setup_tf_publish_action_graph()
            self.setup_joint_state_publish_action_graph()

            # 6. Phase 2 controller loop
            self._start_aic_controller_loop()

            # 7. Setup action graph (joint subscribe side) + force publisher + cameras
            await self.setup_action_graph()
            self.setup_wrist_cameras()
            self.setup_force_publish_action_graph()

            # 8. Spawn task_board_base from YAML pose
            tb_pose = (scene.get("task_board") or {}).get("pose", {}) or {}
            tb_kwargs = {
                "x":     float(tb_pose.get("x",     0.25)),
                "y":     float(tb_pose.get("y",     0.0)),
                "z":     float(tb_pose.get("z",     1.14)),
                "roll":  float(tb_pose.get("roll",  0.0)),
                "pitch": float(tb_pose.get("pitch", 0.0)),
                "yaw":   float(tb_pose.get("yaw",   0.0)),
            }
            print(f"--- Spawning task_board_base {tb_kwargs} ---")
            self._cmd_spawn_task_board_base(**tb_kwargs)
            spawned.append(("spawn_task_board_base", tb_kwargs))

            # 9. Iterate rails — nic_rail_<0..4> / sc_rail_<0..1> / {lc,sfp,sc}_mount_rail_<0..1>
            tb_scene = scene.get("task_board") or {}
            for i in range(5):
                block = tb_scene.get(f"nic_rail_{i}")
                if block:
                    spawned.append(self._dispatch_rail_block(
                        "nic_rail", i, block, self._cmd_spawn_nic_card_mount))
            for i in range(2):
                block = tb_scene.get(f"sc_rail_{i}")
                if block:
                    spawned.append(self._dispatch_rail_block(
                        "sc_rail", i, block, self._cmd_spawn_sc_port))
                for prefix, fn in [
                    ("lc_mount_rail",  self._cmd_spawn_lc_mount_rail),
                    ("sfp_mount_rail", self._cmd_spawn_sfp_mount_rail),
                    ("sc_mount_rail",  self._cmd_spawn_sc_mount_rail),
                ]:
                    block = tb_scene.get(f"{prefix}_{i}")
                    if block:
                        spawned.append(self._dispatch_rail_block(prefix, i, block, fn))

            # 10. Parity publishers (always on per D-04)
            print("--- Setting up AIC parity publishers ---")
            try:
                self._start_aic_parity_publishers()
            except Exception as exc:
                print(f"[AIC-DT][parity] _start_aic_parity_publishers failed: {exc!r}")

            # 11. Scoring publishers (gated on ground_truth, mirrors quick_start gate)
            if ground_truth:
                # D-13 fallback (Plan 04-03 Wave 1 A4=MISMATCH): compute live
                # port USD prim paths from spawned-component list and override
                # the scoring publisher's hardcoded _PORT_LINK_PATHS. The
                # spawn-atom prim-path templates were captured by 04-01
                # taskboard_prim_paths.txt:
                #   spawn_sc_port(index=i, present=True)   → /World/TaskBoard/SCPort_{i}
                #   spawn_nic_card_mount(index=i, present=True) → /World/TaskBoard/NICCardMount_{i}
                live_port_paths = []
                for atom_name, kw in spawned:
                    if not kw or not kw.get("present"):
                        continue
                    if atom_name.startswith("sc_rail_"):
                        i = int(atom_name.rsplit("_", 1)[-1])
                        live_port_paths.append(f"/World/TaskBoard/SCPort_{i}")
                    elif atom_name.startswith("nic_rail_"):
                        i = int(atom_name.rsplit("_", 1)[-1])
                        live_port_paths.append(f"/World/TaskBoard/NICCardMount_{i}")
                print(f"--- Setting up AIC scoring publishers (/scoring/tf + /objects_poses_real + /scoring/insertion_event) ---")
                print(f"[AIC-DT][scoring] D-13 live port paths from spawn ({len(live_port_paths)}): {live_port_paths}")
                try:
                    self._start_aic_scoring_publishers(
                        port_link_paths=live_port_paths if live_port_paths else None,
                    )
                except Exception as exc:
                    print(f"[AIC-DT][scoring] _start_aic_scoring_publishers failed: {exc!r}")

                # DEFERRED-4 (2026-05-10): publish per-trial CheatCode-expected
                # TF frames so insert_cable() can look up `task_board/<mount>/<port>_link`
                # and `<cable>/<plug>_link` from base_link. Source-of-truth for the
                # mount→port offset is ~/Documents/aic/tools/anchor_target_offsets.yaml
                # (cached SE(3) constants per CAD pair, baked from Gazebo recordings).
                try:
                    trial_tf_frames = self._compute_trial_tf_frames(
                        trial_cfg=trials[trial_key] or {},
                        spawned=spawned,
                    )
                    if trial_tf_frames and hasattr(self, "_aic_scoring_publishers") and self._aic_scoring_publishers:
                        self._aic_scoring_publishers.set_trial_tf_frames(trial_tf_frames)
                except Exception as exc:
                    print(f"[AIC-DT][scoring] _compute_trial_tf_frames failed: {exc!r}")
                    traceback.print_exc()

                # motion-fidelity-cheatcode-timing (2026-05-11): wire the active
                # trial target (target_module_name + port_name) into the scoring
                # publisher so /scoring/insertion_event emits the engine-canonical
                # payload "<target_module_name>/<port_name>" per ScoringTier2.cc
                # tokenizer (line 825-855) + Gazebo CablePlugin.cc:268,323.
                try:
                    tasks = (trials[trial_key] or {}).get("tasks", {}) or {}
                    if tasks and hasattr(self, "_aic_scoring_publishers") and self._aic_scoring_publishers:
                        _, _task = next(iter(tasks.items()))
                        _task = _task or {}
                        _tm = _task.get("target_module_name")
                        _pn = _task.get("port_name")
                        # Map target_module → live USD prefix for contact filtering.
                        # Same mapping convention used by _compute_trial_tf_frames.
                        _usd_prefix = None
                        if _tm and _tm.startswith("nic_card_mount_"):
                            _idx = _tm.rsplit("_", 1)[-1]
                            _usd_prefix = f"/World/TaskBoard/NICCardMount_{_idx}"
                        elif _tm and _tm.startswith("sc_port_"):
                            _idx = _tm.rsplit("_", 1)[-1]
                            _usd_prefix = f"/World/TaskBoard/SCPort_{_idx}"
                        if _tm and _pn:
                            self._aic_scoring_publishers.set_active_trial_target(
                                target_module_name=_tm,
                                port_name=_pn,
                                usd_prefix=_usd_prefix,
                            )
                except Exception as exc:
                    print(f"[AIC-DT][scoring] set_active_trial_target wiring failed: {exc!r}")
                    traceback.print_exc()
            else:
                print("--- ground_truth=False — skipping AIC scoring publishers (M2 swap surface preserved) ---")

            # 12. Ensure timeline is playing (idempotent)
            try:
                self._timeline.play()
            except Exception:
                pass

            # 13. trial-home-robot-pose: apply home_joint_positions from the
            # top-level robot.home_joint_positions block. Engine's home_robot()
            # path is gated on skip_ready_simulator=false; we pass true so it
            # never homes, and the robot stays at load_robot defaults (gripper
            # ~600mm from any task-board port). Apply explicitly so CheatCode
            # starts with the gripper above the board where it can reach.
            try:
                self._apply_home_joint_positions(cfg.get("robot", {}) or {})
            except Exception as exc:
                print(f"[AIC-DT][home] _apply_home_joint_positions failed: {exc!r}")
                traceback.print_exc()

            # 14. (motion-fidelity-cheatcode-timing iter NN — REMOVED): re-author
            # of CableAttachJoint after home pose was attempted here but does
            # NOT propagate to PhysX runtime — once timeline is playing, USD
            # joint attribute edits land in USD but PhysX has already cooked
            # the articulation and ignores them. Probe confirmed localPos0=
            # (0,0,0) lands in USD but link_20 stays 1m from finger_l in world.
            # The fix path therefore needs a timeline-stop / re-author / play
            # cycle OR an entirely different anchoring strategy (e.g. kinematic
            # plug driven from gripper pose every physics step). Logged in
            # plans/progress.txt for orchestrator routing — needs /ask-gpt
            # adversarial review per stuck-escape-valve rule before next attempt.

            return {
                "status": "success",
                "trial": trial_key,
                "ground_truth": ground_truth,
                "config_path": config_path,
                "spawned_components": spawned,
                "cable_kwargs": cable_kwargs,
            }
        except Exception as e:
            carb.log_error(f"Error in load_trial: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"load_trial failed: {str(e)}",
                "traceback": traceback.format_exc(),
            }

    def _cmd_setup_wrist_cameras(self) -> Dict[str, Any]:
        try:
            self.setup_wrist_cameras()
            return {"status": "success", "message": "Wrist camera action graphs created for center, left, right cameras"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_add_objects(self) -> Dict[str, Any]:
        try:
            self.add_objects()
            return {"status": "success", "message": "AIC task board populated: task_board_base + 5 nic_card_mounts (each w/ PCB child) + 2 sc_ports + 2 lc/sfp/sc mount rails (6 total) (13 prims)"}
        except Exception as e:
            carb.log_error(f"Error in add_objects: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to add objects: {str(e)}"}

    # =============================================================================
    # SCENE-01 / DX-02 — Per-component spawn atoms
    # =============================================================================
    # Each atom mirrors a parameter family from
    # ~/Documents/aic/aic_bringup/launch/spawn_task_board.launch.py.
    # 4-surface contract per atom (DX-02):
    #   1. MCP_TOOL_REGISTRY entry  (top of this file, MCP_TOOL_REGISTRY dict)
    #   2. MCP_HANDLERS entry       (top of this file, MCP_HANDLERS dict)
    #   3. _cmd_<name> method       (this section)
    #   4. UI button                (create_ui CollapsableFrame "Spawn Atoms")
    # Atoms:
    #   - spawn_task_board_base
    #   - spawn_lc_mount_rail (index 0/1)
    #   - spawn_sfp_mount_rail (index 0/1)
    #   - spawn_sc_mount_rail (index 0/1)
    #   - spawn_sc_port (index 0/1)
    #   - spawn_nic_card_mount (index 0..4)
    #   - spawn_nic_card
    # Backwards-compatible clubbing: add_objects() invokes the above with
    # AIC_OBJECTS defaults (Task 3) — see refactored add_objects below.
    # =============================================================================

    # Mount-rail URDF anchor offsets (canonical source:
    # ~/Documents/aic/aic_description/urdf/task_board.urdf.xacro joint origins).
    # Mount rails: index 0 = left side (Y=-0.10625+translation), index 1 = right
    # side (Y=+0.10625+translation). X+Z anchors per rail; translation goes on Y.
    _LC_MOUNT_ANCHOR_X = 0.01    # URDF: lc_mount_rail_*_joint origin x=0.01
    _LC_MOUNT_ANCHOR_Z = 0.012   # URDF: ...z=0.012
    _SFP_MOUNT_ANCHOR_X = 0.055  # URDF: sfp_mount_rail_*_joint origin x=0.055
    _SFP_MOUNT_ANCHOR_Z = 0.01   # URDF: ...z=0.01
    _SC_MOUNT_ANCHOR_X_0 = 0.1   # URDF: sc_mount_rail_0_joint origin x=0.1
    _SC_MOUNT_ANCHOR_X_1 = 0.0985  # URDF: sc_mount_rail_1_joint origin x=0.0985
    _SC_MOUNT_ANCHOR_Z_0 = 0.012  # URDF: sc_mount_rail_0_joint origin z=0.012
    _SC_MOUNT_ANCHOR_Z_1 = 0.01   # URDF: sc_mount_rail_1_joint origin z=0.01
    _MOUNT_ANCHOR_Y = 0.10625  # rail Y; sign flips with index (URDF ±)

    # NIC card mount URDF anchors: 5 indices share X=-0.081418+translation,
    # Z=0.012, but each index has a different Y. Translation maps to X (NOT Y).
    _NIC_CARD_MOUNT_ANCHOR_X = -0.081418
    _NIC_CARD_MOUNT_ANCHOR_Z = 0.012
    _NIC_CARD_MOUNT_ANCHOR_Y_BY_INDEX = (-0.1745, -0.1345, -0.0945, -0.0545, -0.0145)

    # SC port URDF anchors: shared X=-0.075+translation, Z=0.0165, per-index Y,
    # and a static RPY offset of (1.57, 0, 1.57) rad layered on top of the
    # caller-supplied RPY. Translation maps to X (NOT Y).
    _SC_PORT_ANCHOR_X = -0.075
    _SC_PORT_ANCHOR_Z = 0.0165
    _SC_PORT_ANCHOR_Y_BY_INDEX = (0.0295, 0.0705)
    _SC_PORT_RPY_OFFSET_RAD = (1.57, 0.0, 1.57)

    def _apply_meters_per_unit_scale(self, xform, usd_uri: str, stage) -> float:
        """Author a TypeScale op on xform that compensates for unit-mismatch
        between the referenced asset and the parent stage.

        OPT-IN per asset — not auto-called from _spawn_component_via_usd.
        Reason: some AIC-shipped USDs declare metersPerUnit=0.01 BUT also bake
        an internal Xform scale=0.001 (or similar) into their asset hierarchy
        as compensation. Adding an external scale on top of that internal
        compensation double-applies and shrinks geometry 100x. The cases that
        DO need this helper are assets where the asset USD has no internal
        unit compensation — e.g. nic_card_visual.usd whose meshes' raw vertex
        coordinates are CAD-millimeter-scale (12.1 etc.) and need to be
        treated as cm before entering a m-stage.

        How to determine if an asset needs this:
          1. Reference the asset with no scale op
          2. Run `scene_divergence.py` — section 2 will compare live AABB to
             Gazebo GLB raw AABB. If they're 100x apart (or 1000x), this
             helper is the fix. If they match within ~10%, the asset has
             internal compensation and this helper would BREAK it.

        Returns the scale ratio applied (1.0 = no scale added)."""
        stage_mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
        # Strip file:// prefix if present (omni.client URI form)
        asset_fs_path = usd_uri.replace("file://", "")
        asset_mpu = 1.0
        try:
            asset_stage = Usd.Stage.Open(asset_fs_path)
            if asset_stage:
                asset_mpu = UsdGeom.GetStageMetersPerUnit(asset_stage) or 1.0
        except Exception:
            pass  # If we can't open the asset, assume no scale needed
        scale_ratio = asset_mpu / stage_mpu
        if abs(scale_ratio - 1.0) > 1e-9:
            xform.AddScaleOp().Set(Gf.Vec3f(scale_ratio, scale_ratio, scale_ratio))
            print(f"[AIC-DT][mpu] auto-scale {scale_ratio:.6f} applied to {xform.GetPath()} "
                  f"(asset_mpu={asset_mpu}, stage_mpu={stage_mpu}, source={asset_fs_path})")
        return scale_ratio

    def _spawn_component_via_usd(self, prim_path: str, usd_relpath: str,
                                  position, rpy) -> Dict[str, Any]:
        """Helper: idempotently spawn a USD-referenced prim at prim_path.

        Returns a Dict suitable for an MCP atom result. The pose application
        uses UsdGeom.Xformable.AddTranslateOp + AddRotateXYZOp (intrinsic
        XYZ Euler matching Gazebo's -R -P -Y semantics). After translate+rotate
        a metersPerUnit auto-scale is applied (see _apply_meters_per_unit_scale).
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return {"status": "error", "message": "No stage available"}
            # Asset must be vendored — degrade gracefully per Plan 09 Task 1.
            try:
                usd_uri = _local_asset(usd_relpath)
            except FileNotFoundError as e:
                return {"status": "error",
                        "message": f"Asset not vendored ({usd_relpath}) — see Plan 09 Task 1 WARN log: {e}"}
            # Idempotent cleanup
            existing = stage.GetPrimAtPath(prim_path)
            if existing.IsValid():
                stage.RemovePrim(prim_path)
            # Create parent path if missing (e.g. /World/TaskBoard before any spawn)
            parent_path = "/".join(prim_path.rstrip("/").split("/")[:-1])
            if parent_path and parent_path != "/World" and not stage.GetPrimAtPath(parent_path).IsValid():
                UsdGeom.Xform.Define(stage, parent_path)
            # Author the prim with USD reference + pose
            xform = UsdGeom.Xform.Define(stage, prim_path)
            xform.GetPrim().GetReferences().AddReference(usd_uri)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
            # Bug fix 2026-05-11 (trial-home-robot-pose surface): USD's
            # AddRotateXYZOp value is in DEGREES, but our API surface (mirroring
            # spawn_task_board.launch.py / sample_config.yaml) accepts RADIANS.
            # Without this convert, task_board yaw=π lands as 3.14° → board is
            # essentially unrotated → ports' world Y flips sign vs Gazebo →
            # CheatCode xy_error ~0.18 m on Y (board half-width × 2). Same bug
            # was present in _apply_robot_pose / cable pose (both fixed in same
            # commit). Mirrors Gazebo radian convention; identical pose
            # outcome in Isaac Sim.
            import math as _math
            xform.AddRotateXYZOp().Set(Gf.Vec3f(
                _math.degrees(float(rpy[0])),
                _math.degrees(float(rpy[1])),
                _math.degrees(float(rpy[2])),
            ))
            # metersPerUnit auto-scale REMOVED 2026-05-12: see _apply_meters_per_unit_scale
            # comment. Some AIC-shipped USDs declare mpu=0.01 BUT also bake a
            # compensating internal Xform scale (e.g. LCMountRail with internal
            # scale=0.001), so auto-applying an external scale double-applies
            # and shrinks the asset 100x. The helper remains available for
            # explicit per-asset use in spawn handlers when the asset is known
            # to NOT have internal compensation (e.g. nic_card_visual.usd in
            # _cmd_spawn_nic_card_mount). Use scene_divergence.py to flag
            # remaining mpu-related divergences for per-asset review.
            # taskboard-prim-authoring (audit finding #6): explicitly mark any
            # RigidBodyAPI descendant kinematic_enabled=True. The on-disk
            # task_board_rigid.usd / sc_port.usd / nic_card.usd carry RigidBodyAPI
            # with mass=0 + density=0, which PhysX silently treats as kinematic —
            # but the authoring is fragile. AIC's Isaac Lab env at
            # ~/Documents/aic/aic_utils/aic_isaac/aic_isaaclab/.../aic_task_env_cfg.py
            # uses RigidObjectCfg(kinematic_enabled=True) on all task-board parts;
            # mirror that explicit authoring so PhysX contact-report semantics are
            # deterministic and PhysxContactReportAPI tagging finds rigid bodies
            # reliably (per scoring_publishers._tag_rigid_bodies_under).
            spawned_prim = stage.GetPrimAtPath(prim_path)
            if spawned_prim and spawned_prim.IsValid():
                for desc in Usd.PrimRange(spawned_prim):
                    if desc.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(desc).CreateKinematicEnabledAttr().Set(True)
            return {"status": "success",
                    "message": f"Spawned {prim_path} pose=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f}) RPY=({rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}) rad"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"spawn failed at {prim_path}: {e}"}

    def _ensure_task_board_parent(self) -> None:
        """Pre-flight: ensure /World/TaskBoard exists at Gazebo-canonical pose.

        Called from every child spawn atom (lc/sfp/sc mount rails, sc_port,
        nic_card_mount, nic_card) so a UI-button click in any order lands the
        child at its correct world pose. Without this pre-flight,
        _spawn_component_via_usd's "auto-create parent Xform if missing" path
        silently creates /World/TaskBoard at the identity transform, and the
        child ends up at world-frame = task-board-local coords (~10cm above
        the floor near the workspace origin instead of on the workbench).

        Idempotent: respects an existing /World/TaskBoard prim regardless of
        how it was authored (explicit Spawn button, load_trial dispatcher, or
        a prior child-spawn pre-flight). Detection is by presence of a USD
        reference on the prim — distinguishes "fully-authored task-board"
        from "empty auto-created identity Xform stub" that earlier code
        could leave behind.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        tb = stage.GetPrimAtPath("/World/TaskBoard")
        if tb.IsValid() and tb.HasAuthoredReferences():
            return
        print("[AIC-DT] /World/TaskBoard not authored — auto-spawning at Gazebo canonical pose (0.15, -0.2, 1.14, yaw=π)")
        self._cmd_spawn_task_board_base()

    def _cmd_spawn_task_board_base(self, x: float = 0.15, y: float = -0.2, z: float = 1.14,
                                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 3.1415) -> Dict[str, Any]:
        """SCENE-01: Spawn task board base.

        Parameter names mirror spawn_task_board.launch.py task_board_x/y/z/roll/pitch/yaw.
        Defaults match aic_gz_bringup.launch.py:629-668 DeclareLaunchArgument defaults.
        """
        prim_path = "/World/TaskBoard"
        usd_rel = AIC_OBJECTS["task_board_base"]["usd"]
        return self._spawn_component_via_usd(prim_path, usd_rel,
                                             position=(x, y, z),
                                             rpy=(roll, pitch, yaw))

    def _cmd_spawn_lc_mount_rail(self, index: int = 0, present: bool = False,
                                  translation: float = 0.0,
                                  roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn an LC Mount Rail (index 0 or 1).

        Parameter names mirror spawn_task_board.launch.py
        lc_mount_rail_<index>_present/translation/roll/pitch/yaw.
        URDF anchor: x=0.0275, y=±0.10625 (sign by index).
        """
        if not present:
            return {"status": "skipped", "message": f"lc_mount_rail_{index}_present=false"}
        if index not in (0, 1):
            return {"status": "error", "message": f"lc_mount_rail index must be 0 or 1, got {index}"}
        self._ensure_task_board_parent()
        anchor_x = self._LC_MOUNT_ANCHOR_X
        anchor_y = -self._MOUNT_ANCHOR_Y if index == 0 else self._MOUNT_ANCHOR_Y
        local_pos = (anchor_x, anchor_y + float(translation), self._LC_MOUNT_ANCHOR_Z)
        prim_path = f"/World/TaskBoard/LCMountRail_{index}"
        return self._spawn_component_via_usd(prim_path,
                                             "assets/LC Mount/lc_mount_visual.usd",
                                             position=local_pos,
                                             rpy=(roll, pitch, yaw))

    def _cmd_spawn_sfp_mount_rail(self, index: int = 0, present: bool = False,
                                   translation: float = 0.0,
                                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn an SFP Mount Rail (index 0 or 1).

        Parameter names mirror spawn_task_board.launch.py
        sfp_mount_rail_<index>_present/translation/roll/pitch/yaw.
        URDF anchor: x=0.0535, y=±0.10625 (sign by index).
        """
        if not present:
            return {"status": "skipped", "message": f"sfp_mount_rail_{index}_present=false"}
        if index not in (0, 1):
            return {"status": "error", "message": f"sfp_mount_rail index must be 0 or 1, got {index}"}
        self._ensure_task_board_parent()
        anchor_x = self._SFP_MOUNT_ANCHOR_X
        anchor_y = -self._MOUNT_ANCHOR_Y if index == 0 else self._MOUNT_ANCHOR_Y
        local_pos = (anchor_x, anchor_y + float(translation), self._SFP_MOUNT_ANCHOR_Z)
        prim_path = f"/World/TaskBoard/SFPMountRail_{index}"
        return self._spawn_component_via_usd(prim_path,
                                             "assets/SFP Mount/sfp_mount_visual.usd",
                                             position=local_pos,
                                             rpy=(roll, pitch, yaw))

    def _cmd_spawn_sc_mount_rail(self, index: int = 0, present: bool = False,
                                  translation: float = 0.0,
                                  roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn an SC Mount Rail (index 0 or 1).

        Parameter names mirror spawn_task_board.launch.py
        sc_mount_rail_<index>_present/translation/roll/pitch/yaw.
        URDF anchor: x=0.0985, y=±0.10625 (sign by index).
        """
        if not present:
            return {"status": "skipped", "message": f"sc_mount_rail_{index}_present=false"}
        if index not in (0, 1):
            return {"status": "error", "message": f"sc_mount_rail index must be 0 or 1, got {index}"}
        self._ensure_task_board_parent()
        anchor_x = self._SC_MOUNT_ANCHOR_X_0 if index == 0 else self._SC_MOUNT_ANCHOR_X_1
        anchor_z = self._SC_MOUNT_ANCHOR_Z_0 if index == 0 else self._SC_MOUNT_ANCHOR_Z_1
        anchor_y = -self._MOUNT_ANCHOR_Y if index == 0 else self._MOUNT_ANCHOR_Y
        local_pos = (anchor_x, anchor_y + float(translation), anchor_z)
        prim_path = f"/World/TaskBoard/SCMountRail_{index}"
        return self._spawn_component_via_usd(prim_path,
                                             "assets/SC Mount/sc_mount_visual.usd",
                                             position=local_pos,
                                             rpy=(roll, pitch, yaw))

    def _cmd_spawn_sc_port(self, index: int = 0, present: bool = False,
                            translation: float = 0.0,
                            roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn an SC Port (index 0 or 1).

        Parameter names mirror spawn_task_board.launch.py
        sc_port_<index>_present/translation/roll/pitch/yaw.
        Wired to AIC_OBJECTS["sc_port_1"]["usd"] (both indices share the same USD).
        """
        if not present:
            return {"status": "skipped", "message": f"sc_port_{index}_present=false"}
        if index not in (0, 1):
            return {"status": "error", "message": f"sc_port index must be 0 or 1, got {index}"}
        self._ensure_task_board_parent()
        # URDF (task_board.urdf.xacro line ~150): sc_port_<i> at
        #   pose="${-0.075 + sc_port_<i>_translation} <Y[i]> 0.0165
        #          ${1.57 + roll} pitch ${1.57 + yaw}"
        # Translation maps to X (NOT Y); rail Y is per-index; rotation has a
        # static (1.57, 0, 1.57) rad offset on top of caller-supplied RPY.
        anchor_x = self._SC_PORT_ANCHOR_X + float(translation)
        anchor_y = self._SC_PORT_ANCHOR_Y_BY_INDEX[index]
        anchor_z = self._SC_PORT_ANCHOR_Z
        local_pos = (anchor_x, anchor_y, anchor_z)
        r_off, p_off, y_off = self._SC_PORT_RPY_OFFSET_RAD
        rpy_final = (float(roll) + r_off, float(pitch) + p_off, float(yaw) + y_off)
        prim_path = f"/World/TaskBoard/SCPort_{index}"
        return self._spawn_component_via_usd(prim_path,
                                             AIC_OBJECTS["sc_port_1"]["usd"],
                                             position=local_pos,
                                             rpy=rpy_final)

    def _cmd_spawn_nic_card_mount(self, index: int = 0, present: bool = False,
                                   translation: float = 0.0,
                                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn a NIC Card Mount (index 0..4).

        Parameter names mirror spawn_task_board.launch.py
        nic_card_mount_<index>_present/translation/roll/pitch/yaw — note 5 indices.
        """
        if not present:
            return {"status": "skipped", "message": f"nic_card_mount_{index}_present=false"}
        if index not in (0, 1, 2, 3, 4):
            return {"status": "error", "message": f"nic_card_mount index must be 0..4, got {index}"}
        self._ensure_task_board_parent()
        # URDF (task_board.urdf.xacro line ~130): nic_card_mount_<i> at
        #   pose="${-0.081418 + translation} <Y[i]> 0.012 roll pitch yaw"
        # Translation maps to X (rail direction); each of the 5 rails has a
        # distinct Y anchor (-0.1745, -0.1345, -0.0945, -0.0545, -0.0145).
        anchor_x = self._NIC_CARD_MOUNT_ANCHOR_X + float(translation)
        anchor_y = self._NIC_CARD_MOUNT_ANCHOR_Y_BY_INDEX[index]
        anchor_z = self._NIC_CARD_MOUNT_ANCHOR_Z
        local_pos = (anchor_x, anchor_y, anchor_z)
        prim_path = f"/World/TaskBoard/NICCardMount_{index}"
        result = self._spawn_component_via_usd(prim_path,
                                               "assets/NIC Card Mount/nic_card_mount_visual.usd",
                                               position=local_pos,
                                               rpy=(roll, pitch, yaw))

        # iter 7 Path Y 2026-05-12: author sfp_port_0 + sfp_port_1 contact
        # colliders that the Gazebo USD asset is missing. Without these
        # colliders, the kinematic plug_proxy reaches the published port TF
        # but no physical body exists for PhysX contact detection to fire
        # /scoring/insertion_event. The Gazebo asset (~/Documents/aic/
        # aic_assets/models/NIC Card Mount/model.sdf) authors:
        #   sfp_port_0_link at (0.01295, -0.031572, 0.00501) relative to nic_card_link
        #   sfp_port_1_link at (-0.01025, -0.031572, 0.00501) relative to nic_card_link
        # with paired contact_collision_01/02 boxes serving as the contact
        # sensor geometry for the TouchPlugin sfp_port_<n> namespace.
        # We author each port as a small sphere collider on the NIC mount
        # body — sphere is more forgiving than the 8mm × 0.25mm Gazebo box
        # for first-time CheatCode targeting, and matches what scoring
        # contact-based insertion detection actually needs.
        try:
            stage = omni.usd.get_context().get_stage()
            from pxr import UsdGeom, UsdPhysics, Gf, Sdf
            # nic_card_link is at offset relative to NIC mount root per Gazebo SDF
            # link pose; we author ports directly under NICCardMount_<i> with
            # absolute (relative-to-mount) offsets that include the nic_card_link
            # offset combined with sfp_port_<n>_link offset. From SDF:
            #   nic_card_link <pose>-0.002 -0.01785 0.0899 -1.57 0 0</pose>
            #   sfp_port_0_link rel to nic_card_link: (0.01295, -0.031572, 0.00501)
            #   sfp_port_1_link rel to nic_card_link: (-0.01025, -0.031572, 0.00501)
            # Because nic_card_link has RPY(-1.57, 0, 0), the relative position needs
            # transformation. For M1 first-pass, use spheres at empirically reasonable
            # local positions — Gazebo's TouchPlugin's contact_collision boxes are at
            # (0.012963, -0.0305, 0.002845) and (-0.010237, -0.0305, 0.002845)
            # relative to nic_card_link. Combining with nic_card_link's offset gives
            # approximate world-relative-to-NIC-mount positions:
            # iter 7 Path Y fix 2026-05-12: port offsets MUST match the
            # published port TF in scoring_publishers (which reads
            # ~/Documents/aic/tools/anchor_target_offsets.yaml). For
            # nic_card_mount/sfp_port_0 the YAML defines translation_xyz_m=
            # (0.01095, -0.012865, 0.121476) — this is the exact anchor→port
            # offset CheatCode targets. Mismatched offsets between the
            # published TF and the physical collider cause CheatCode to drive
            # plug_proxy to the published frame but the physical port is
            # offset elsewhere → 13cm residual gap → no PhysX contact (trial_2
            # failure 2026-05-12T18:37Z log).
            #
            # port_1 isn't in the anchor yaml but is in NIC mount SDF: the
            # port_0 X offset in nic_card_link is 0.01295, port_1 X is -0.01025
            # — delta -0.0232m. Apply same delta to anchor yaml's port_0 X
            # (0.01095) → port_1 X = 0.01095 - 0.0232 = -0.01225.
            port_offsets = {
                0: (0.01095, -0.012865, 0.121476),   # exact anchor_target_offsets.yaml
                1: (-0.01225, -0.012865, 0.121476),  # derived from SDF Δ on port_0
            }
            for port_idx, (px, py, pz) in port_offsets.items():
                port_path = f"{prim_path}/sfp_port_{port_idx}"
                existing = stage.GetPrimAtPath(port_path)
                if existing and existing.IsValid():
                    stage.RemovePrim(port_path)
                port_xform = UsdGeom.Xform.Define(stage, Sdf.Path(port_path))
                port_xform.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
                sphere_path = f"{port_path}/collider_geom"
                port_sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(sphere_path))
                port_sphere.CreateRadiusAttr().Set(0.012)  # 12mm — generous target
                # NOTE iter 7 Path Y debug 2026-05-12: do NOT set Purpose=guide.
                # See plug_proxy block — purpose=guide silently excludes from
                # PhysX contact discovery (broke trial_2 insertion fire).
                UsdPhysics.CollisionAPI.Apply(port_sphere.GetPrim())
                print(f"[AIC-DT] Path Y: authored {port_path} at NIC-relative ({px:.4f},{py:.4f},{pz:.4f}) with 12mm sphere collider")
        except Exception as exc:
            print(f"[AIC-DT] Path Y port-collider authoring failed: {exc!r}")

        # nic-card-visual-compose 2026-05-12: the upstream nic_card_mount_visual.usd
        # ships from the AIC organizers as ONLY a single ISO 4762 M2x8 screw mesh —
        # no mount body, no PCB. The actual NIC card PCB is a separate Gazebo SDF
        # link (`nic_card_link`) at local pose (-0.002, -0.01785, 0.0899, RPY=-1.57,0,0)
        # under nic_card_mount_link, referencing nic_card_visual.glb (21MB; converted
        # to nic_card_visual.usd 1.5MB / 45 meshes / 33K vertices — works fine).
        # Without composing this child here, load_trial spawns only the M2 screw
        # and the user sees no NIC card on the mount rail. Gazebo SDF reference:
        # ~/Documents/aic/aic_assets/models/NIC Card Mount/model.sdf
        try:
            import math as _m
            card_path = f"{prim_path}/nic_card_link"
            existing_card = stage.GetPrimAtPath(card_path)
            if existing_card and existing_card.IsValid():
                stage.RemovePrim(card_path)
            card_xform = UsdGeom.Xform.Define(stage, Sdf.Path(card_path))
            card_xform.AddTranslateOp().Set(Gf.Vec3d(-0.002, -0.01785, 0.0899))
            card_xform.AddRotateXYZOp().Set(Gf.Vec3f(_m.degrees(-1.57), 0.0, 0.0))
            try:
                card_uri = _local_asset("assets/NIC Card Mount/nic_card_visual.usd")
                card_xform.GetPrim().GetReferences().AddReference(card_uri)
                # 2026-05-13: _apply_meters_per_unit_scale call REMOVED.
                # Was needed when nic_card_visual.usd was an omni.kit.asset_converter
                # output with stage-level mpu=0.01 — the helper authored
                # scale=0.01 to compensate for the cm→m interpretation. Now
                # nic_card_visual.usd is a thin USD (mpu=1.0 default) that
                # references nic_card_visual.glb directly, and the gltf SDF
                # plugin handles mm→m conversion via its own runtime ops
                # (scale=0.001 on the gltf-loaded Xform). Applying the helper
                # on top stacks 0.01 × 0.001 = 1e-5 effective scale → PCB
                # renders at ~1 µm and disappears. Drop it.
                print(f"[AIC-DT] NIC card PCB composed at {card_path} (Gazebo SDF pose)")
            except FileNotFoundError as e:
                print(f"[AIC-DT] NIC PCB asset not vendored: {e}")
        except Exception as exc:
            print(f"[AIC-DT] NIC card PCB compose failed: {exc!r}")

        return result

    def _cmd_spawn_nic_card(self, present: bool = False,
                             translation: float = 0.0,
                             roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
        """SCENE-01: Spawn a standalone NIC Card.

        spawn_task_board.launch.py has no nic_card_<index> family (only
        nic_card_mount); this atom mirrors the legacy AIC_OBJECTS["nic_card"]
        for parity with prior add_objects behavior.
        """
        if not present:
            return {"status": "skipped", "message": "nic_card_present=false"}
        self._ensure_task_board_parent()
        local_pos = (float(translation), 0.0, 0.0)
        prim_path = "/World/TaskBoard/NICCard"
        return self._spawn_component_via_usd(prim_path,
                                             AIC_OBJECTS["nic_card"]["usd"],
                                             position=local_pos,
                                             rpy=(roll, pitch, yaw))

    def _cmd_delete_objects(self) -> Dict[str, Any]:
        try:
            self.delete_objects()
            return {"status": "success", "message": "Objects deleted (swept /World/TaskBoard + /World/Objects)"}
        except Exception as e:
            carb.log_error(f"Error in delete_objects: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to delete objects: {str(e)}"}

    def _cmd_randomize_object_poses(self) -> Dict[str, Any]:
        try:
            count = self.randomize_object_poses()
            return {"status": "success", "message": f"Randomized {count} AIC objects"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to randomize object poses: {str(e)}"}

    def _cmd_randomize_single_object(self, object_name: str = None) -> Dict[str, Any]:
        if not object_name:
            return {"status": "error", "message": "object_name is required"}
        try:
            self.randomize_single_object(object_name)
            return {"status": "success", "message": f"Randomized '{object_name}'"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to randomize '{object_name}': {str(e)}"}

    def _cmd_save_scene_state(self, json_file_path: str = None, output_dir: str = None) -> Dict[str, Any]:
        try:
            from datetime import datetime

            self._resolve_output_dir(output_dir)
            scene_dir = self._get_scene_state_dir()
            if json_file_path:
                if not json_file_path.endswith(".json"):
                    json_file_path += ".json"
                save_path = os.path.join(scene_dir, os.path.basename(json_file_path))
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = os.path.join(scene_dir, f"scene_state_{timestamp}.json")

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {"status": "error", "message": "No stage is currently open"}

            objects_prim = stage.GetPrimAtPath("/World/Objects")
            if not objects_prim.IsValid():
                return {"status": "error", "message": "/World/Objects path does not exist"}

            children = objects_prim.GetChildren()
            object_names = [child.GetName() for child in children if child.IsA(UsdGeom.Xformable)]

            if not object_names:
                return {"status": "error", "message": "No objects found in /World/Objects"}

            poses = {}
            saved_count = 0
            failed_names = []

            for object_name in object_names:
                prim_path = self._get_prim_path(object_name)
                pose = self._read_prim_pose(prim_path)
                if pose:
                    poses[object_name] = pose
                    saved_count += 1
                else:
                    failed_names.append(object_name)

            with open(save_path, 'w') as f:
                json.dump(poses, f, indent=4)

            abs_path = os.path.abspath(save_path)
            msg = f"[save_state] Saved {saved_count} objects to {abs_path}"
            if failed_names:
                msg += f", {len(failed_names)} failed: {failed_names}"
            print(msg)

            return {
                "status": "success", "message": msg,
                "saved_count": saved_count, "failed_names": failed_names,
                "json_file_path": save_path
            }
        except Exception as e:
            print(f"[save_state] Error: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to save scene state: {str(e)}"}

    def _cmd_restore_scene_state(self, json_file_path: str = None, output_dir: str = None) -> Dict[str, Any]:
        try:
            self._resolve_output_dir(output_dir)

            if json_file_path:
                if not json_file_path.endswith(".json"):
                    json_file_path += ".json"
                restore_path = os.path.join(self._get_scene_state_dir(), os.path.basename(json_file_path))
                if not os.path.exists(restore_path):
                    return {"status": "error", "message": f"Scene state file not found: {restore_path}"}
                latest_path = restore_path
            else:
                latest_path = self._get_latest_scene_state_path()

            if not latest_path:
                return {"status": "error", "message": f"No scene state files found in {self._get_scene_state_dir()}"}

            try:
                with open(latest_path, 'r') as f:
                    poses = json.load(f)
            except json.JSONDecodeError as e:
                return {"status": "error", "message": f"Invalid JSON file: {str(e)}"}

            if not poses:
                return {"status": "error", "message": "No objects found in scene state file"}

            restored_count = 0
            failed_names = []

            for object_name, pose_data in poses.items():
                prim_path = self._get_prim_path(object_name)
                if self._write_prim_pose(prim_path, pose_data):
                    restored_count += 1
                else:
                    failed_names.append(object_name)

            msg = f"[restore_state] Restored {restored_count} objects from {latest_path}"
            if failed_names:
                msg += f", {len(failed_names)} failed: {failed_names}"
            print(msg)

            return {
                "status": "success", "message": msg,
                "restored_count": restored_count, "failed_names": failed_names,
                "json_file_path": latest_path
            }
        except Exception as e:
            print(f"[restore_state] Error: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to restore scene state: {str(e)}"}

    async def _cmd_new_stage(self) -> Dict[str, Any]:
        """Clear the entire USD stage to free accumulated memory."""
        try:
            import os as _os
            import gc

            with open(f"/proc/{_os.getpid()}/status") as f:
                for line in f:
                    if line.startswith("VmRSS"):
                        rss_before = int(line.split()[1]) // 1024

            self._stop_force_publish()

            timeline = omni.timeline.get_timeline_interface()
            if timeline.is_playing():
                timeline.stop()

            self._python_sessions.clear()
            self._hidden_objects.clear()
            self._initial_orientations.clear()

            omni.usd.get_context().new_stage()

            app = omni.kit.app.get_app()
            for _ in range(5):
                await app.next_update_async()

            gc.collect()

            with open(f"/proc/{_os.getpid()}/status") as f:
                for line in f:
                    if line.startswith("VmRSS"):
                        rss_after = int(line.split()[1]) // 1024

            freed_mb = rss_before - rss_after
            return {
                "status": "success",
                "message": f"Stage cleared. Freed {freed_mb} MB (RSS: {rss_before} MB -> {rss_after} MB).",
                "rss_before_mb": rss_before, "rss_after_mb": rss_after, "freed_mb": freed_mb,
            }
        except Exception as e:
            carb.log_error(f"Error in new_stage: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to create new stage: {str(e)}"}

    async def _cmd_quick_start(self, ground_truth: bool = True) -> Dict[str, Any]:
        try:
            await self.quick_start(ground_truth=ground_truth)
            return {
                "status": "success",
                "ground_truth": ground_truth,
                "message": "Quick start complete: AIC scene with UR5e, wrist cameras, task board objects, and simulation running."
            }
        except Exception as e:
            carb.log_error(f"Error in quick_start: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Quick start failed: {str(e)}"}

    def _cmd_randomize_lighting(self) -> Dict[str, Any]:
        try:
            self.randomize_lighting()
            return {"status": "success", "message": "Dome light randomized (intensity 1500-3500, color 0.5-1.0)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to randomize lighting: {str(e)}"}

    def _cmd_run_policy(self, checkpoint_path: str = None) -> Dict[str, Any]:
        """Placeholder for Phase 4 policy execution."""
        return {
            "status": "success",
            "message": f"run_policy not yet implemented. checkpoint_path='{checkpoint_path}' received. This will be implemented in Phase 4."
        }

    # ==================== Shutdown ====================

    def on_shutdown(self):
        self._teardown_log_redirect()
        print("[AIC-DT] Digital Twin shutdown")

        self._stop_mcp_server()
        self._stop_force_publish()

        # Stop the rclpy-based AIC parity publishers (idempotent).
        if self._aic_parity_publishers is not None:
            try:
                self._aic_parity_publishers.stop()
            except Exception as exc:
                print(f"[AIC-DT][parity] stop() failed in shutdown: {exc!r}")
            self._aic_parity_publishers = None

        # Stop the rclpy-based AIC controller loop (idempotent). Phase 2 Plan 02-02.
        if self._aic_controller_loop is not None:
            try:
                self._aic_controller_loop.stop()
            except Exception as exc:
                print(f"[AIC-DT][controller] stop() failed in shutdown: {exc!r}")
            self._aic_controller_loop = None

        print("ROS2 bridge shutdown handled by Isaac Sim")

        # Clear UI widget references to break closure references
        self._workspace_checkbox = None
        self._custom_checkbox = None
        self._custom_camera_prim_field = None
        self._custom_camera_topic_field = None
        self._resolution_combo = None

        if self._window:
            self._window.frame.clear()
            self._window.destroy()
            self._window = None
