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
    "task_board_base": {
        "usd": "objects/task_board_base/task_board_rigid.usd",
        "position": (0.2837, 0.229, 0.0),
        "rotation": None,  # identity quaternion
    },
    "sc_port_1": {
        "usd": "objects/sc_port/sc_port.usd",
        "position": (0.2904, 0.1928, 0.005),
        "rotation": (0.73136, 0.0, 0.0, -0.682),  # wxyz
    },
    "sc_port_2": {
        "usd": "objects/sc_port/sc_port.usd",
        "position": (0.2913, 0.1507, 0.005),
        "rotation": (0.73136, 0.0, 0.0, -0.682),  # wxyz
    },
    "nic_card": {
        "usd": "objects/nic_card/nic_card.usd",
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

# Wrist cameras built into the robot USD
WRIST_CAMERAS = {
    "center_camera": {
        "prim_suffix": "center_camera_optical/center_camera",
        "topic": "center_camera_rgb_sim",
        "info_topic": "center_camera_info",
        "frame_id": "center_camera_optical",
        "width": 640,
        "height": 480,
    },
    "left_camera": {
        "prim_suffix": "left_camera_optical/left_camera",
        "topic": "left_camera_rgb_sim",
        "info_topic": "left_camera_info",
        "frame_id": "left_camera_optical",
        "width": 640,
        "height": 480,
    },
    "right_camera": {
        "prim_suffix": "right_camera_optical/right_camera",
        "topic": "right_camera_rgb_sim",
        "info_topic": "right_camera_info",
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
        "description": "Import the UR5e robot (with integrated RG2 gripper and cable) into the scene at the AIC position. Configures articulation and joint drives.",
        "parameters": {}
    },
    "setup_action_graph": {
        "description": "Create the ROS2 action graph for UR5e joint state subscription and clock publishing.",
        "parameters": {}
    },
    "setup_force_publisher": {
        "description": "Create the ROS2 force/torque publisher action graph for UR5e end-effector wrench.",
        "parameters": {}
    },
    "setup_wrist_cameras": {
        "description": "Create ROS2 action graphs for all 3 built-in wrist cameras (center, left, right) publishing RGB and CameraInfo.",
        "parameters": {}
    },
    "add_objects": {
        "description": "Add AIC task board objects (base, SC ports, NIC card) to the scene at their default positions.",
        "parameters": {}
    },
    "delete_objects": {
        "description": "Delete all objects from /World/Objects and the associated pose publisher graph.",
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
    "setup_pose_publisher": {
        "description": "Create an action graph to publish object poses to ROS2 topic 'objects_poses_sim'.",
        "parameters": {}
    },
    "sync_real_poses": {
        "description": "Subscribe to /objects_poses_real ROS2 topic and update sim object poses to match real-world detected poses.",
        "parameters": {}
    },
    "new_stage": {
        "description": "Clear the entire USD stage and free accumulated memory. The scene will be empty afterwards -- use quick_start to rebuild it.",
        "parameters": {}
    },
    "quick_start": {
        "description": "One-click scene setup: loads AIC scene with enclosure, imports UR5e robot with action graphs, force publisher, wrist cameras, task board objects, and starts simulation.",
        "parameters": {}
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
    "setup_wrist_cameras": "_cmd_setup_wrist_cameras",
    "add_objects": "_cmd_add_objects",
    "delete_objects": "_cmd_delete_objects",
    "randomize_object_poses": "_cmd_randomize_object_poses",
    "randomize_single_object": "_cmd_randomize_single_object",
    "save_scene_state": "_cmd_save_scene_state",
    "restore_scene_state": "_cmd_restore_scene_state",
    "setup_pose_publisher": "_cmd_setup_pose_publisher",
    "sync_real_poses": "_cmd_sync_real_poses",
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

        # Robot prim path
        self._robot_prim_path = "/World/UR5e"

        # Robot position and orientation (identity - no rotation)
        self._robot_position = (-0.18, -0.122, 0.0)

        # AIC enclosure
        self._enclosure_usd = "scene/aic.usd"
        self._enclosure_position = (0.0, 0.0, -1.15)

        # Ground plane Z
        self._ground_plane_z = -0.08

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

            # 4. Cameras
            with ui.CollapsableFrame(title="Cameras", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Button("Setup Wrist Cameras", width=200, height=35, clicked_fn=self.setup_wrist_cameras)

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
                                ui.Button("Setup Pose Publisher", width=180, height=35, clicked_fn=self.create_pose_publisher)
                                ui.Button("Delete Objects", width=150, height=35, clicked_fn=self.delete_objects)

                    # 6. Task Board Objects > Scene State
                    with ui.CollapsableFrame(title="Scene State", name="subFrame", collapsed=True, height=0):
                        with ui.VStack(spacing=5, height=0):
                            with ui.HStack(spacing=5):
                                ui.Button("Sync Real Poses", width=180, height=35, clicked_fn=self.sync_real_poses)
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
        """Lazy-init an ArticulationView, returning it or None if not ready."""
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
                return None

        if not artic.is_physics_handle_valid():
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

        # Add default dome light if no lights exist
        from pxr import UsdLux
        has_light = False
        for prim in stage.Traverse():
            if prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight):
                has_light = True
                break
        if not has_light:
            dome = UsdLux.DomeLight.Define(stage, "/World/defaultDomeLight")
            dome.CreateIntensityAttr().Set(1000.0)
            print("Added default dome light at 1000 lux")

        # Import AIC enclosure
        self.import_enclosure()

        # Set minimum frame rate
        settings = carb.settings.get_settings()
        settings.set("/persistent/simulation/minFrameRate", self._min_frame_rate)
        print(f"Set persistent/simulation/minFrameRate to {self._min_frame_rate}")

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

    async def quick_start(self):
        """Quick start: load scene, UR5e + action graph + force publisher, wrist cameras, objects, play."""
        print("=== Quick Start ===")
        app = omni.kit.app.get_app()

        # 1. Load scene (physics, ground plane, dome light, enclosure)
        print("--- Loading scene ---")
        await self.load_scene()
        await app.next_update_async()

        # 2. Import UR5e
        print("--- Importing UR5e ---")
        await self.load_robot()
        await app.next_update_async()

        # 3. Setup UR5e action graph
        print("--- Setting up UR5e Action Graph ---")
        await self.setup_action_graph()
        await app.next_update_async()

        # 4. Setup force publisher
        print("--- Setting up UR5e Force Publisher ---")
        self.setup_force_publish_action_graph()
        await app.next_update_async()

        # 5. Setup wrist cameras
        print("--- Setting up Wrist Cameras ---")
        self.setup_wrist_cameras()
        await app.next_update_async()

        # 6. Add task board objects
        print("--- Adding task board objects ---")
        self.add_objects()
        await app.next_update_async()

        # 7. Setup pose publisher
        print("--- Setting up pose publisher ---")
        self.create_pose_publisher()
        await app.next_update_async()

        # 8. Create workspace camera at 640x480
        print("--- Creating Workspace Camera (640x480) ---")
        stage = omni.usd.get_context().get_stage()
        ws_prim_path = "/World/workspace_camera_sim"
        ws_position = (0.8572405778988392, -1.3321141046870788, 0.9906567613694909)
        ws_quat_xyzw = (0.4714, 0.1994, 0.3347, 0.7912)
        ws_width, ws_height = 640, 480

        camera_prim = UsdGeom.Camera.Define(stage, ws_prim_path)
        if camera_prim:
            hfov_deg, vfov_deg = 69.4, 42.5
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
            cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))

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
            print(f"Workspace camera created at {ws_prim_path} with resolution {ws_width}x{ws_height}")
        await app.next_update_async()

        # 9. Setup workspace camera action graph
        print("--- Setting up Workspace Camera Action Graph ---")
        self._create_camera_actiongraph(
            ws_prim_path, ws_width, ws_height,
            "workspace_camera_sim", "WorkspaceCameraSim"
        )
        await app.next_update_async()

        # 10. Play the scene
        print("--- Playing scene ---")
        self._timeline.play()

        print("=== Quick Start Complete ===")

    # ==================== Robot ====================

    async def load_robot(self):
        asset_path = _local_asset("robot/aic_unified_robot_cable_sdf.usd")
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

        # Apply translation with identity orientation (NO 180 degree yaw)
        xform = UsdGeom.Xform(prim)
        xform.ClearXformOpOrder()
        position = Gf.Vec3d(*self._robot_position)
        xform.AddTranslateOp().Set(position)
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        print(f"Applied translation {self._robot_position} to {prim_path} (identity orientation)")

        # Setup Articulation
        self._robot_view = ArticulationView(prim_paths_expr=prim_path, name="ur5e_view")
        World.instance().scene.add(self._robot_view)
        await World.instance().reset_async()
        self._timeline.stop()

        self._articulation = Articulation(prim_path)

        # Configure joint drives (from Isaac Lab config)
        for joint_name in self._joint_names:
            joint_path = f"{prim_path}/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if not joint_prim.IsValid():
                print(f"Warning: Joint not found at {joint_path}")
                continue
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetMaxForceAttr().Set(self._ur5e_max_force)
            drive_api.GetStiffnessAttr().Set(self._ur5e_stiffness)
            drive_api.GetDampingAttr().Set(self._ur5e_damping)
            print(f"Set {joint_name}: maxForce={self._ur5e_max_force}, stiffness={self._ur5e_stiffness}, damping={self._ur5e_damping}")

        print("UR5e robot loaded successfully (with integrated RG2 gripper and cable)!")

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
        """Setup force publishing as geometry_msgs/WrenchStamped on /force_torque_sensor_broadcaster/wrench_sim."""
        import omni.physx

        print("Setting up UR5e Force Publisher (WrenchStamped)...")

        self._stop_force_publish()

        stage_check = omni.usd.get_context().get_stage()
        if not stage_check.GetPrimAtPath(self._robot_prim_path):
            print(f"Error: UR5e prim not found at {self._robot_prim_path}. Load UR5e first.")
            return

        if self._robot_view is not None:
            self._effort_articulation = self._robot_view
            print(f"Using existing ArticulationView. Joints: {self._effort_articulation.joint_names}")
        else:
            self._effort_articulation = None
            print("ArticulationView not cached (hot-reload?). Will lazy-init in physics callback.")

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
                    ("publisher.inputs:topicName", "force_torque_sensor_broadcaster/wrench_sim"),
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "publisher.inputs:execIn"),
                    ("context.outputs:context", "publisher.inputs:context"),
                ]
            }
        )

        self._force_graph_path = graph_path
        self._force_pub_node = nodes[2]  # publisher node

        og.Controller.attribute("inputs:header:frame_id", self._force_pub_node).set("tool0")

        self._force_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step_force
        )
        self._force_publish_active = True

        print(f"UR5e Force Publisher created at {graph_path}")
        print("Publishing geometry_msgs/WrenchStamped to topic: /force_torque_sensor_broadcaster/wrench_sim")

    def _on_physics_step_force(self, dt):
        """Physics step callback - read joint forces and update OmniGraph WrenchStamped attributes."""
        try:
            artic = self._lazy_init_articulation(
                '_effort_articulation', self._robot_prim_path,
                'force_pub_ctrl', '_force_warmup')
            if artic is None:
                return
            self._effort_articulation = artic

            forces = self._effort_articulation.get_measured_joint_forces()
            if forces is not None and len(forces) > 0:
                wrist = forces[0][-1]
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

            graph_suffix = cam_name.replace("_", " ").title().replace(" ", "")
            self._create_camera_actiongraph(
                camera_prim_path,
                cam_cfg["width"],
                cam_cfg["height"],
                cam_cfg["topic"],
                graph_suffix,
                frame_id=cam_cfg["frame_id"],
                info_topic=cam_cfg["info_topic"],
            )
            created += 1

        print(f"Setup {created}/{len(WRIST_CAMERAS)} wrist camera action graphs")

    def _create_camera_actiongraph(self, camera_prim, width, height, topic, graph_suffix,
                                    frame_id="camera_link", info_topic="camera_info"):
        """Helper method to create camera ActionGraph using og.Controller.edit()."""
        graph_path = f"/Graph/ActionGraph_{graph_suffix}"

        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)
            print(f"Removed existing ActionGraph at {graph_path}")

        print(f"Creating ActionGraph: {graph_path}")
        print(f"Camera: {camera_prim}, Resolution: {width}x{height}, Topic: /{topic}")

        keys = og.Controller.Keys

        try:
            (graph_handle, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        ("RenderProduct.inputs:cameraPrim", camera_prim),
                        ("RenderProduct.inputs:width", width),
                        ("RenderProduct.inputs:height", height),
                        ("CameraInfoPublish.inputs:topicName", info_topic),
                        ("CameraInfoPublish.inputs:frameId", frame_id),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                        ("RGBPublish.inputs:topicName", topic),
                        ("RGBPublish.inputs:type", "rgb"),
                        ("RGBPublish.inputs:frameId", frame_id),
                        ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                        ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                        ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "RGBPublish.inputs:context"),
                    ],
                },
            )
            print(f"{graph_suffix} ActionGraph created successfully!")
            print(f"Test with: ros2 topic echo /{topic}")
        except Exception as e:
            print(f"Error creating ActionGraph: {e}")
            traceback.print_exc()

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
            prim_path = "/World/workspace_camera_sim"
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
            if not stage.GetPrimAtPath("/World/workspace_camera_sim"):
                print("Error: Workspace camera not found. Create it first.")
            else:
                self._create_camera_actiongraph("/World/workspace_camera_sim", width, height, "workspace_camera_sim", "WorkspaceCameraSim")

        if is_custom:
            custom_prim_path = self._custom_camera_prim_field.model.get_value_as_string()
            if not custom_prim_path or custom_prim_path.strip() == "":
                print("Error: Please enter a valid camera prim path")
                return
            topic_name = self._custom_camera_topic_field.model.get_value_as_string()
            if not topic_name or topic_name.strip() == "":
                prim_name = custom_prim_path.split("/")[-1] if "/" in custom_prim_path else custom_prim_path
                topic_name = prim_name.lower().replace(" ", "_").replace("-", "_")
            graph_suffix = topic_name.replace("_", " ").replace("-", " ").title().replace(" ", "")
            if not stage.GetPrimAtPath(custom_prim_path):
                print(f"Error: Camera prim not found at {custom_prim_path}. Create it first.")
            else:
                self._create_camera_actiongraph(custom_prim_path, width, height, topic_name, graph_suffix)

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

    def add_objects(self, folder_path="/World/Objects"):
        """Add AIC task board objects to the scene at their configured positions.

        Uses two-pass loading: create references first, then position.
        Skips entirely if all objects already exist (idempotent).
        """
        stage = omni.usd.get_context().get_stage()

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

    def delete_objects(self, folder_path="/World/Objects"):
        """Delete the Objects folder from the scene."""
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        if was_playing:
            timeline.stop()
            print("Stopped simulation before deleting objects")

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(folder_path)
        if prim and prim.IsValid():
            stage.RemovePrim(folder_path)
            print(f"Deleted {folder_path}")
        else:
            print(f"Warning: {folder_path} does not exist")

        pose_graph_path = "/Graph/ActionGraph_objects_poses"
        pose_graph_prim = stage.GetPrimAtPath(pose_graph_path)
        if pose_graph_prim and pose_graph_prim.IsValid():
            stage.RemovePrim(pose_graph_path)
            print(f"Deleted {pose_graph_path}")

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

    # ==================== Sync Real Poses ====================

    def sync_real_poses(self):
        """Subscribe to /objects_poses_real and update sim object poses to match."""
        import rclpy
        from tf2_msgs.msg import TFMessage

        rclpy.init(args=None)
        node = rclpy.create_node("_sync_real_poses_tmp")
        try:
            msg_received = [None]

            def _cb(msg):
                msg_received[0] = msg

            sub = node.create_subscription(TFMessage, "/objects_poses_real", _cb, 1)
            t0 = time.time()
            while msg_received[0] is None and (time.time() - t0) < 3.0:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_subscription(sub)

            if msg_received[0] is None:
                print("[SyncRealPoses] No message received on /objects_poses_real within 3s")
                return

            real_poses = {}
            for tf in msg_received[0].transforms:
                t = tf.transform.translation
                r = tf.transform.rotation
                real_poses[tf.child_frame_id] = {
                    'position': (t.x, t.y, t.z),
                    'quat_wxyz': (r.w, r.x, r.y, r.z),
                }

            stage = omni.usd.get_context().get_stage()
            updated = []
            for name, pose in real_poses.items():
                prim_path = self._get_prim_path(name)
                prim = stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    print(f"[SyncRealPoses] Prim not found: {prim_path}")
                    continue

                qw, qx, qy, qz = pose['quat_wxyz']
                quat = Gf.Quatf(float(qw), float(qx), float(qy), float(qz))
                omni.kit.commands.execute("ChangeProperty",
                    prop_path=f"{prim_path}.xformOp:orient", value=quat, prev=None)
                omni.kit.commands.execute("ChangeProperty",
                    prop_path=f"{prim_path}.xformOp:translate",
                    value=Gf.Vec3d(*pose['position']), prev=None)
                updated.append(name)

            print(f"[SyncRealPoses] Updated {len(updated)} objects: {updated}")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    # ==================== Pose Publisher ====================

    def create_pose_publisher(self):
        """Create action graph for publishing object poses to ROS2"""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Error: No stage found")
            return

        # Collect body prim paths
        body_paths = []
        folder_path = "/World/Objects"
        objects_prim = stage.GetPrimAtPath(folder_path)
        if not objects_prim or not objects_prim.IsValid():
            print(f"Warning: {folder_path} does not exist")
            return

        for child in objects_prim.GetChildren():
            object_name = child.GetName()
            if object_name == "PhysicsMaterial":
                continue
            body_path = self._get_prim_path(object_name, folder_path)
            body_prim = stage.GetPrimAtPath(body_path)
            if body_prim and body_prim.IsValid():
                body_paths.append(body_path)
                print(f"Found: {body_path}")

        if not body_paths:
            print("No objects found in /World/Objects")
            return

        graph_path = "/Graph/ActionGraph_objects_poses"
        if stage.GetPrimAtPath(graph_path) and stage.GetPrimAtPath(graph_path).IsValid():
            print(f"Action graph already exists at {graph_path}, skipping creation")
            return

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_transform_tree.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_transform_tree.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    ("ros2_publish_transform_tree.inputs:topicName", "objects_poses_sim"),
                    ("isaac_read_simulation_time.inputs:resetOnStop", False),
                ],
            },
        )

        ros2_node_path = f"{graph_path}/ros2_publish_transform_tree"
        ros2_prim = stage.GetPrimAtPath(ros2_node_path)
        if ros2_prim.IsValid():
            parent_rel = ros2_prim.GetRelationship("inputs:parentPrim")
            if not parent_rel:
                parent_rel = ros2_prim.CreateRelationship("inputs:parentPrim", custom=True)
            parent_rel.SetTargets([Sdf.Path("/World")])

            target_rel = ros2_prim.GetRelationship("inputs:targetPrims")
            if not target_rel:
                target_rel = ros2_prim.CreateRelationship("inputs:targetPrims", custom=True)
            target_rel.SetTargets([Sdf.Path(p) for p in body_paths])

            print(f"Pose publisher created at {graph_path}, publishing {len(body_paths)} objects")

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
            return {"status": "success", "message": "Scene loaded (physics, ground plane, dome light, AIC enclosure)"}
        except Exception as e:
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _cmd_load_robot(self) -> Dict[str, Any]:
        try:
            from omni.kit.async_engine import run_coroutine
            run_coroutine(self.load_robot())
            return {"status": "success", "message": "UR5e loaded with integrated RG2 gripper at /World/UR5e"}
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
            return {"status": "success", "message": f"AIC task board objects added ({len(AIC_OBJECTS)} objects)"}
        except Exception as e:
            carb.log_error(f"Error in add_objects: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to add objects: {str(e)}"}

    def _cmd_delete_objects(self) -> Dict[str, Any]:
        try:
            self.delete_objects()
            return {"status": "success", "message": "Objects deleted from /World/Objects"}
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

    def _cmd_setup_pose_publisher(self) -> Dict[str, Any]:
        try:
            self.create_pose_publisher()
            return {"status": "success", "message": "Pose publisher action graph created at /Graph/ActionGraph_objects_poses"}
        except Exception as e:
            carb.log_error(f"Error in setup_pose_publisher: {e}")
            traceback.print_exc()
            return {"status": "error", "message": f"Failed to setup pose publisher: {str(e)}"}

    def _cmd_sync_real_poses(self) -> Dict[str, Any]:
        try:
            self.sync_real_poses()
            return {"status": "success", "message": "Synced real poses to sim"}
        except Exception as e:
            carb.log_error(f"Error in sync_real_poses: {e}")
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

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

    async def _cmd_quick_start(self) -> Dict[str, Any]:
        try:
            await self.quick_start()
            return {
                "status": "success",
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
