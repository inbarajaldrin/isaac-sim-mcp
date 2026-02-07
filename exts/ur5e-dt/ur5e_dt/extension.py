import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import os
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

# MCP socket server port - change this for different extensions
MCP_SERVER_PORT = 8766

# MCP output directory configuration
BASE_OUTPUT_DIR = os.getenv("MCP_CLIENT_OUTPUT_DIR", "").strip()
if BASE_OUTPUT_DIR:
    BASE_OUTPUT_DIR = os.path.abspath(BASE_OUTPUT_DIR)
    RESOURCES_DIR = os.path.join(BASE_OUTPUT_DIR, "resources")
else:
    RESOURCES_DIR = "resources"

# Assembly definitions: each entry pairs an objects folder with its assembly JSON
ASSEMBLIES = {
    "fmb1": {
        "folder": "omniverse://localhost/Library/DT demo/fmb1/",
        "assembly_file": os.path.expanduser("~/Projects/aruco-grasp-annotator/data/fmb_assembly1.json"),
    },
    "fmb2": {
        "folder": "omniverse://localhost/Library/DT demo/fmb2/",
        "assembly_file": os.path.expanduser("~/Projects/aruco-grasp-annotator/data/fmb_assembly2.json"),
    },
    "fmb3": {
        "folder": "omniverse://localhost/Library/DT demo/fmb3/",
        "assembly_file": os.path.expanduser("~/Projects/aruco-grasp-annotator/data/fmb_assembly3.json"),
    },
}

# =============================================================================
# MCP TOOL REGISTRY - Single source of truth for all MCP tools
# =============================================================================
# Each tool definition includes:
#   - description: Tool description shown to MCP clients
#   - parameters: Dict of parameter definitions (name -> {type, description, required, default})
#
# The MCP server queries this registry on startup and dynamically registers tools.
# To add a new tool:
#   1. Add entry here with description and parameters
#   2. Add handler method _cmd_<tool_name>() in this extension
#   3. That's it! The server will automatically discover and expose it.
# =============================================================================

MCP_TOOL_REGISTRY = {
    "execute_python_code": {
        "description": "Execute Python code directly in Isaac Sim's environment with access to Omniverse APIs.",
        "parameters": {
            "code": {
                "type": "string",
                "description": "Python code to execute in Isaac Sim"
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
    "assemble_objects": {
        "description": "Assemble objects into a specified configuration.",
        "parameters": {
            "assembly": {
                "type": "string",
                "enum": ["fmb1", "fmb2", "fmb3"],
                "description": "Which assembly configuration to use"
            }
        }
    },
    "randomize_object_poses": {
        "description": "Randomize the positions of non-base objects in /World/Objects.",
        "parameters": {}
    },
    "save_scene_state": {
        "description": "Saves current object poses to a JSON file so it can be retrieved later.",
        "parameters": {}
    },
    "restore_scene_state": {
        "description": "Restores previously saved object poses from a JSON file to the scene.",
        "parameters": {}
    },
    "clear_scene_state": {
        "description": "Delete the scene state JSON file, removing all saved object poses.",
        "parameters": {}
    },
    "add_objects": {
        "description": "Add objects to the scene from a predefined assembly folder.",
        "parameters": {
            "assembly": {
                "type": "string",
                "enum": ["fmb1", "fmb2", "fmb3"],
                "description": "Which assembly folder to load objects from"
            }
        }
    },
    "delete_objects": {
        "description": "Delete all objects from /World/Objects and the associated pose publisher graph.",
        "parameters": {}
    },
    "setup_pose_publisher": {
        "description": "Create an action graph to publish object poses to ROS2 topic 'objects_poses_sim'.",
        "parameters": {}
    },
}

# Handler method names for each tool (maps to self._cmd_<name> methods)
MCP_HANDLERS = {
    "execute_python_code": "_cmd_execute_python_code",
    "play_scene": "_cmd_play_scene",
    "stop_scene": "_cmd_stop_scene",
    "assemble_objects": "_cmd_assemble_objects",
    "randomize_object_poses": "_cmd_randomize_object_poses",
    "save_scene_state": "_cmd_save_scene_state",
    "restore_scene_state": "_cmd_restore_scene_state",
    "clear_scene_state": "_cmd_clear_scene_state",
    "add_objects": "_cmd_add_objects",
    "delete_objects": "_cmd_delete_objects",
    "setup_pose_publisher": "_cmd_setup_pose_publisher",
}

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation as ArticulationView
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.prims import SingleArticulation as Articulation
import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf, Sdf, Usd, UsdPhysics, UsdShade, PhysxSchema
from scipy.spatial.transform import Rotation as R

class DigitalTwin(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[DigitalTwin] Digital Twin startup")

        self._timeline = omni.timeline.get_timeline_interface()
        self._ur5e_view = None
        self._articulation = None
        self._gripper_view = None
        self._viewport_rendering_enabled = True
        self._viewport_toggle_btn = None

        # Gripper physics callback state
        self._gripper_physx_sub = None
        self._gripper_articulation = None
        self._gripper_graph_path = None
        self._gripper_sub_attr_path = None
        self._gripper_pub_attr_path = None
        self._gripper_publish_active = False
        self._gripper_warmup = 0  # skip N physics steps before re-init

        # Force publisher physics callback state
        self._force_warmup = 0

        # MCP socket server state
        self._mcp_socket = None
        self._mcp_server_thread = None
        self._mcp_server_running = False

        # Add Objects UI state
        self._selected_assembly = "fmb1"
        self._object_spacing = 0.25  # Spacing between objects along X-axis in meters
        self._y_offset = -0.5  # Y offset for all objects
        self._z_offset = 0.0495  # Z offset for all objects

        # Scene state file path (can be set by MCP client)
        self._scene_state_file_path = None

        # Output directory pushed by MCP server on connect (None = use default)
        self._output_dir = None

        # Physics scene settings
        self._min_frame_rate = 60
        self._time_steps_per_second = 120

        # UR5e joint drive parameters
        self._ur5e_max_force = 200.0
        self._ur5e_stiffness = 100.0
        self._ur5e_damping = 10.0

        # RG2 gripper joint drive parameters
        self._gripper_max_force = 1.2
        self._gripper_stiffness = 0.1
        self._gripper_damping = 0.05

        # Gripper physics material
        self._gripper_dynamic_friction = 0.6
        self._gripper_static_friction = 0.5
        self._gripper_restitution = 0.0
        self._gripper_friction_combine_mode = "max"
        self._gripper_restitution_combine_mode = "min"

        # Common physics material (applied to all objects)
        self._object_dynamic_friction = 0.1
        self._object_static_friction = 0.1
        self._object_restitution = 0.0
        self._object_friction_combine_mode = "max"
        self._object_restitution_combine_mode = "min"

        # Shared collision settings
        self._sdf_resolution = 300
        self._contact_offset = 0.0005

        # Per-category prim settings (collision approximation, rest offset, angular damping)
        # Board (type: "board", e.g. base1, base2, base3)
        self._board_collision_approximation = "sdf"  # "sdf" or "convexDecomposition"
        self._board_rest_offset = 0.0
        self._board_angular_damping = 20.0

        # Block (subtype: "block", e.g. u_brown, fork_orange, line_green)
        self._block_collision_approximation = "sdf"
        self._block_rest_offset = -0.0005
        self._block_angular_damping = 20.0

        # Socket (subtype: "socket", e.g. inverted_u_brown)
        self._socket_collision_approximation = "sdf"
        self._socket_rest_offset = -0.0005
        self._socket_angular_damping = 20.0

        # Peg (subtype: "peg", e.g. hex_blue, hex_red)
        self._peg_collision_approximation = "sdf"
        self._peg_rest_offset = -0.0005
        self._peg_angular_damping = 50.0

        # Isaac Sim handles ROS2 initialization automatically through its bridge
        print("ROS2 bridge will be initialized by Isaac Sim when needed")

        # Create the window UI
        self._window = ui.Window("UR5e Digital Twin", width=300, height=800)  # Increased height
        with self._window.frame:
            with ui.VStack(spacing=5):
                self.create_ui()

        # Start MCP socket server
        self._start_mcp_server()

    def create_ui(self):
        with ui.VStack(spacing=5):
            with ui.CollapsableFrame(title="Setup", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Simulation Setup", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Load Scene", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_scene()))
                        self._viewport_toggle_btn = ui.Button("Disable Viewport", width=140, height=35, clicked_fn=self._toggle_viewport_rendering)


            with ui.CollapsableFrame(title="UR5e Control", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("UR5e Robot Control", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Load UR5e", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_ur5e()))
                        ui.Button("Setup UR5e Action Graph", width=200, height=35, clicked_fn=lambda: asyncio.ensure_future(self.setup_action_graph()))
                        ui.Button("Setup UR5e Force Publisher", width=200, height=35, clicked_fn=self.setup_force_publish_action_graph)

            with ui.CollapsableFrame(title="RG2 Gripper", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("RG2 Gripper Control", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Import RG2 Gripper", width=150, height=35, clicked_fn=self.import_rg2_gripper)
                        ui.Button("Attach Gripper to UR5e", width=180, height=35, clicked_fn=self.attach_rg2_to_ur5e)
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Setup Gripper Action Graph", width=200, height=35, clicked_fn=self.setup_gripper_action_graph)

            # Intel RealSense Camera
            with ui.CollapsableFrame(title="Intel RealSense Camera", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Intel RealSense D455 Camera", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Import RealSense Camera", width=170, height=35, clicked_fn=self.import_realsense_camera)
                        ui.Button("Attach Camera to UR5e", width=160, height=35, clicked_fn=self.attach_camera_to_ur5e)
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Setup Camera Action Graph", width=200, height=35, clicked_fn=self.setup_camera_action_graph)

            # NEW SECTION: Additional Camera
            with ui.CollapsableFrame(title="Additional Camera", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Camera Configuration", alignment=ui.Alignment.LEFT)
                    
                    # Camera type selection with checkboxes and labels
                    with ui.VStack(spacing=5):
                        with ui.HStack(spacing=5):
                            self._workspace_checkbox = ui.CheckBox(width=20)
                            self._workspace_checkbox.model.set_value(False)  # Default unchecked
                            ui.Label("Workspace Camera", alignment=ui.Alignment.LEFT, width=120)

                        with ui.HStack(spacing=5):
                            self._custom_checkbox = ui.CheckBox(width=20)
                            ui.Label("Custom Camera", alignment=ui.Alignment.LEFT, width=120)

                    # Custom camera prim path input
                    with ui.HStack(spacing=5):
                        ui.Label("Camera Prim Path:", alignment=ui.Alignment.LEFT, width=120)
                        self._custom_camera_prim_field = ui.StringField(width=200)
                        self._custom_camera_prim_field.model.set_value("/World/custom_camera")
                    
                    # Custom camera ROS2 topic name input
                    with ui.HStack(spacing=5):
                        ui.Label("ROS2 Topic Name:", alignment=ui.Alignment.LEFT, width=120)
                        self._custom_camera_topic_field = ui.StringField(width=200)
                        self._custom_camera_topic_field.model.set_value("custom_camera")

                    # Resolution selection
                    with ui.HStack(spacing=5):
                        ui.Label("Resolution:", alignment=ui.Alignment.LEFT, width=120)
                        self._resolution_combo = ui.ComboBox(0, "1280x720", "1920x1080", width=150)

                    # Camera control buttons
                    with ui.HStack(spacing=5):
                        ui.Button("Create Camera", width=150, height=35, clicked_fn=self.create_additional_camera)
                        ui.Button("Create Action Graph", width=180, height=35, clicked_fn=self.create_additional_camera_actiongraph)

            with ui.CollapsableFrame(title="Objects", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    with ui.HStack(spacing=5):
                        ui.Label("Assembly:", alignment=ui.Alignment.LEFT, width=100)
                        assembly_names = list(ASSEMBLIES.keys())
                        self._assembly_combo = ui.ComboBox(0, *assembly_names, width=150)
                        self._assembly_combo.model.add_item_changed_fn(
                            lambda m, _: self._on_assembly_changed(m)
                        )
                    with ui.HStack(spacing=5):
                        ui.Button("Add Objects", width=150, height=35, clicked_fn=self.add_objects)
                        ui.Button("Setup Pose Publisher", width=180, height=35, clicked_fn=self.create_pose_publisher)
                        ui.Button("Delete Objects", width=150, height=35, clicked_fn=self.delete_objects)
                    with ui.HStack(spacing=5):
                        ui.Button("Assemble", width=120, height=35, clicked_fn=self.assemble_objects)
                        ui.Button("Disassemble", width=120, height=35, clicked_fn=self.disassemble_objects)
                        ui.Button("Randomize Poses", width=150, height=35, clicked_fn=self.randomize_object_poses)
                    ui.Label("Scene State", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Save State", width=120, height=35,
                            clicked_fn=lambda: self._cmd_save_scene_state())
                        ui.Button("Restore State", width=120, height=35,
                            clicked_fn=lambda: self._cmd_restore_scene_state())
                        ui.Button("Clear State", width=120, height=35,
                            clicked_fn=lambda: self._cmd_clear_scene_state())

    def _on_assembly_changed(self, model):
        """Called when the assembly combo box selection changes."""
        idx = model.get_item_value_model().as_int
        self._selected_assembly = list(ASSEMBLIES.keys())[idx]

    @property
    def _objects_folder_path(self):
        return ASSEMBLIES[self._selected_assembly]["folder"]

    @property
    def _assembly_file_path(self):
        return ASSEMBLIES[self._selected_assembly]["assembly_file"]

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
            GroundPlane(prim_path="/World/defaultGroundPlane", z_position=0, size=5000)
            print("Added ground plane")

        # Set minimum frame rate to 60
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

    async def load_ur5e(self):
        asset_path = "omniverse://localhost/Library/ur5e.usd"
        prim_path = "/World/UR5e"

        # 1. Ensure World exists - create and initialize if needed (won't clear existing stage)
        world = World.instance()
        if world is None:
            print("World not initialized. Creating simulation context for existing stage...")
            world = World()
            await world.initialize_simulation_context_async()
            # Note: Not adding ground plane here - assumes existing scene has one or user doesn't want it
            print("Simulation context initialized.")

        # 2. Add the USD asset
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        # 3. Wait for prim to exist
        import time
        stage = omni.usd.get_context().get_stage()
        for _ in range(10):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"Failed to load prim at {prim_path}")

        # 4. Apply translation and orientation
        xform = UsdGeom.Xform(prim)
        xform.ClearXformOpOrder()

        # Custom position and orientation
        position = Gf.Vec3d(0.0, 0.0, 0.11)  # Replace with your desired position
        rpy_deg = np.array([0.0, 0.0, 180.0])  # Replace with your desired RPY
        rpy_rad = np.deg2rad(rpy_deg)
        quat_xyzw = euler_angles_to_quats(rpy_rad)
        quat = Gf.Quatd(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])

        xform.AddTranslateOp().Set(position)
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

        print(f"Applied translation and rotation to {prim_path}")

        # 5. Setup Articulation
        self._ur5e_view = ArticulationView(prim_paths_expr=prim_path, name="ur5e_view")
        World.instance().scene.add(self._ur5e_view)
        await World.instance().reset_async()
        self._timeline.stop()

        self._articulation = Articulation(prim_path)

        # Configure joint drives: set maxForce=150 and stiffness=300 for all joints
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        for joint_name in joint_names:
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

        print("UR5e robot loaded successfully!")

        
    async def setup_action_graph(self):
        import omni.graph.core as og
        import isaacsim.core.utils.stage as stage_utils
        from isaacsim.core.utils.extensions import enable_extension

        print("Setting up ROS 2 Action Graph...")

        # Ensure extensions are enabled
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.core.nodes")
        enable_extension("omni.graph.action")

        graph_path = "/World/Graphs/ActionGraph_UR5e"

        # Check if graph already exists
        stage = omni.usd.get_context().get_stage()
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
                    ("ros2_subscribe_joint_state.inputs:queueSize", 10),
                    ("ros2_publish_clock.inputs:topicName", "/clock"),
                    ("ros2_publish_clock.inputs:nodeNamespace", ""),
                    ("ros2_publish_clock.inputs:queueSize", 10),
                    ("isaac_read_simulation_time.inputs:resetOnStop", True),
                    ("articulation_controller.inputs:robotPath", "/World/UR5e"),
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

    def setup_gripper_action_graph(self):
        """Setup gripper action graph for ROS2 control.

        Uses the same pattern as the force publisher: a pure OmniGraph action
        graph for ROS2 communication and an extension-level physics callback
        for ArticulationView control. No ScriptNodes — the extension owns
        the ArticulationView lifecycle so stop/play works without refresh.
        """
        import omni.physx

        print("Setting up Gripper Action Graph...")

        # Clean up any previous physics callback
        self._stop_gripper_physics()

        graph_path = "/World/Graphs/ActionGraph_RG2"
        keys = og.Controller.Keys

        # Delete existing graph
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)

        # Create graph with only ROS2 nodes — no ScriptNodes
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    (f"{graph_path}/tick", "omni.graph.action.OnPlaybackTick"),
                    (f"{graph_path}/context", "isaacsim.ros2.bridge.ROS2Context"),
                    (f"{graph_path}/subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    (f"{graph_path}/ros2_publisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                ],
                keys.SET_VALUES: [
                    (f"{graph_path}/subscriber.inputs:messageName", "String"),
                    (f"{graph_path}/subscriber.inputs:messagePackage", "std_msgs"),
                    (f"{graph_path}/subscriber.inputs:topicName", "gripper_command"),
                    (f"{graph_path}/ros2_publisher.inputs:messageName", "Float64"),
                    (f"{graph_path}/ros2_publisher.inputs:messagePackage", "std_msgs"),
                    (f"{graph_path}/ros2_publisher.inputs:topicName", "gripper_width_sim"),
                ],
                keys.CONNECT: [
                    (f"{graph_path}/tick.outputs:tick", f"{graph_path}/subscriber.inputs:execIn"),
                    (f"{graph_path}/tick.outputs:tick", f"{graph_path}/ros2_publisher.inputs:execIn"),
                    (f"{graph_path}/context.outputs:context", f"{graph_path}/subscriber.inputs:context"),
                    (f"{graph_path}/context.outputs:context", f"{graph_path}/ros2_publisher.inputs:context"),
                ],
            }
        )

        # Create custom data attribute on publisher for gripper width
        publisher_prim = stage.GetPrimAtPath(f"{graph_path}/ros2_publisher")
        publisher_prim.CreateAttribute("inputs:data", Sdf.ValueTypeNames.Double, custom=True)

        # Store attribute paths for the physics callback
        self._gripper_graph_path = graph_path
        self._gripper_sub_attr_path = f"{graph_path}/subscriber.outputs:data"
        self._gripper_pub_attr_path = f"{graph_path}/ros2_publisher.inputs:data"
        self._gripper_articulation = None

        # Subscribe to physics step events — gripper control runs at physics rate
        self._gripper_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step_gripper
        )
        self._gripper_publish_active = True

        print(f"Gripper Action Graph created at {graph_path}")
        print("Gripper control runs at physics rate via extension callback (no ScriptNodes)")
        print("\nTest commands:")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"open\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"close\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"1100\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"550\"'")
        print("\nMonitor gripper width:")
        print("ros2 topic echo /gripper_width_sim")

    def _on_physics_step_gripper(self, dt):
        """Physics step callback — read ROS2 command, apply to gripper, publish state."""
        import numpy as np
        from isaacsim.core.prims import Articulation as ArticulationView
        from isaacsim.core.utils.types import ArticulationActions

        try:
            # Warmup: skip physics steps after a stop/play to let engine settle
            if self._gripper_warmup > 0:
                self._gripper_warmup -= 1
                return

            # Lazy-init ArticulationView on first physics step
            if self._gripper_articulation is None:
                try:
                    import time as _t
                    self._gripper_articulation = ArticulationView(
                        prim_paths_expr="/World/RG2_Gripper",
                        name=f"gripper_ctrl_{int(_t.time()*1000)}"
                    )
                    self._gripper_articulation.initialize()
                except Exception:
                    self._gripper_articulation = None
                    return  # Physics not ready yet, try next step

            if not self._gripper_articulation.is_physics_handle_valid():
                # Handle went stale (e.g. after timeline stop/play cycle).
                # Discard and wait for physics engine to settle before re-init.
                self._gripper_articulation = None
                self._gripper_warmup = 30  # ~0.25s at 120 Hz
                return

            # --- Read command from ROS2 subscriber ---
            try:
                raw = og.Controller.get(og.Controller.attribute(self._gripper_sub_attr_path))
                input_str = str(raw).strip() if raw else ""
            except Exception:
                input_str = ""

            # Parse command
            if input_str and input_str not in ("", "None", "0"):
                if input_str == "open":
                    width_mm = 110.0
                elif input_str == "close":
                    width_mm = 0.0
                else:
                    try:
                        width_mm = float(input_str) / 10.0
                    except ValueError:
                        width_mm = None

                if width_mm is not None:
                    width_mm = float(np.clip(width_mm, 0.0, 110.0))
                    ratio = width_mm / 110.0
                    joint_angle = -np.pi / 4 + ratio * (np.pi / 4 + np.pi / 6)
                    target = np.array([joint_angle, joint_angle])
                    action = ArticulationActions(
                        joint_positions=target,
                        joint_indices=np.array([0, 1])
                    )
                    self._gripper_articulation.apply_action(action)

            # --- Read gripper state and publish ---
            joint_positions = self._gripper_articulation.get_joint_positions()
            if joint_positions is not None and joint_positions.shape[1] >= 2:
                actual_angle = float(np.mean(joint_positions[0, :2]))
                actual_ratio = (actual_angle + np.pi / 4) / (np.pi / 4 + np.pi / 6)
                actual_width_mm = float(np.clip(actual_ratio * 110.0, 0.0, 110.0))
                og.Controller.set(
                    og.Controller.attribute(self._gripper_pub_attr_path),
                    actual_width_mm
                )
        except Exception:
            pass  # Silently ignore during initialization/teardown

    def _stop_gripper_physics(self):
        """Stop the gripper physics callback and clean up resources."""
        if hasattr(self, '_gripper_physx_sub') and self._gripper_physx_sub is not None:
            self._gripper_physx_sub = None
        if hasattr(self, '_gripper_articulation'):
            self._gripper_articulation = None
        self._gripper_publish_active = False
        self._gripper_warmup = 0

    def setup_force_publish_action_graph(self):
        """Setup force publishing using joint forces from UR5e ArticulationView.

        Uses a physics step callback to read measured joint forces at physics rate
        (~60 Hz). Uses get_measured_joint_forces() which returns 6-DOF spatial forces
        [Fx, Fy, Fz, Tx, Ty, Tz] per joint. Publishes Fz from the UR5e wrist joint.
        """
        import omni.physx

        print("Setting up UR5e Force Publisher...")

        # Clean up any previous physics callback
        self._stop_force_publish()

        # Reuse the ArticulationView created during UR5e loading
        if self._ur5e_view is None:
            print("Error: UR5e ArticulationView not available. Load UR5e first.")
            return
        self._effort_articulation = self._ur5e_view
        print(f"Using existing ArticulationView. Joints: {self._effort_articulation.joint_names}")

        # Delete existing graph if it exists
        graph_path = "/World/Graphs/ActionGraph_UR5e_ForcePublish"
        keys = og.Controller.Keys
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)

        # Create OmniGraph with OnPlaybackTick for ROS2 publisher execution
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    (f"{graph_path}/tick", "omni.graph.action.OnPlaybackTick"),
                    (f"{graph_path}/context", "isaacsim.ros2.bridge.ROS2Context"),
                    (f"{graph_path}/publisher", "isaacsim.ros2.bridge.ROS2Publisher")
                ],
                keys.SET_VALUES: [
                    (f"{graph_path}/publisher.inputs:messageName", "Float64"),
                    (f"{graph_path}/publisher.inputs:messagePackage", "std_msgs"),
                    (f"{graph_path}/publisher.inputs:topicName", "ur5e_wrist_force"),
                ],
                keys.CONNECT: [
                    (f"{graph_path}/tick.outputs:tick", f"{graph_path}/publisher.inputs:execIn"),
                    (f"{graph_path}/context.outputs:context", f"{graph_path}/publisher.inputs:context"),
                ]
            }
        )

        # Create data attribute on publisher for the effort value
        publisher_prim = stage.GetPrimAtPath(f"{graph_path}/publisher")
        publisher_prim.CreateAttribute("inputs:data", Sdf.ValueTypeNames.Double, custom=True)

        self._force_graph_attr_path = f"{graph_path}/publisher.inputs:data"

        # Physics callback reads joint forces at physics rate and updates the attribute
        self._force_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step_force
        )

        self._force_publish_active = True

        print(f"UR5e Force Publisher created at {graph_path}")
        print("Publishing Fz (N) from UR5e wrist joint to topic: /ur5e_wrist_force")
        print("Using get_measured_joint_forces() - 6-DOF spatial forces per joint")
        print("Joint forces read at physics rate (~60 Hz)")

    def _on_physics_step_force(self, dt):
        """Physics step callback - read joint forces and update OmniGraph attribute.

        Uses get_measured_joint_forces() which returns 6-DOF spatial forces
        [Fx, Fy, Fz, Tx, Ty, Tz] per joint. Publishes Fz from the last joint
        (wrist joint) as the main collision/contact indicator.
        """
        try:
            # Warmup: skip physics steps after a stop/play to let engine settle
            if self._force_warmup > 0:
                self._force_warmup -= 1
                return

            # Lazy-init ArticulationView (or re-init after stop/play cycle)
            if self._effort_articulation is None:
                try:
                    import time as _t
                    from isaacsim.core.prims import Articulation as _AV
                    self._effort_articulation = _AV(
                        prim_paths_expr="/World/UR5e",
                        name=f"force_pub_ctrl_{int(_t.time()*1000)}"
                    )
                    self._effort_articulation.initialize()
                except Exception:
                    self._effort_articulation = None
                    return  # Physics not ready yet

            if not self._effort_articulation.is_physics_handle_valid():
                # Handle went stale (e.g. after timeline stop/play cycle).
                # Discard and wait for physics engine to settle before re-init.
                self._effort_articulation = None
                self._force_warmup = 30  # ~0.25s at 120 Hz
                return

            # get_measured_joint_forces returns shape (num_articulations, num_joints, 6)
            # where 6 = [Fx, Fy, Fz, Tx, Ty, Tz]
            forces = self._effort_articulation.get_measured_joint_forces()
            if forces is not None and len(forces) > 0:
                # Use last joint (wrist joint), get Fz (index 2) as collision indicator
                fz = float(forces[0][-1][2])
                og.Controller.set(
                    og.Controller.attribute(self._force_graph_attr_path),
                    fz
                )
        except Exception:
            pass

    def _stop_force_publish(self):
        """Stop the physics-rate force publisher and clean up resources."""
        if hasattr(self, '_force_physx_sub') and self._force_physx_sub is not None:
            self._force_physx_sub = None
        if hasattr(self, '_effort_articulation'):
            self._effort_articulation = None
        self._force_publish_active = False
        self._force_warmup = 0

    def import_rg2_gripper(self):
        from isaacsim.core.utils.stage import add_reference_to_stage
        rg2_usd_path = "omniverse://localhost/Library/RG2.usd"
        add_reference_to_stage(rg2_usd_path, "/World/RG2_Gripper")
        print("RG2 Gripper imported at /World/RG2_Gripper")

        # Configure gripper joint drives
        stage = omni.usd.get_context().get_stage()
        base_link = "/World/RG2_Gripper/onrobot_rg2_base_link"
        gripper_joints = [
            f"{base_link}/finger_joint",
            f"{base_link}/right_outer_knuckle_joint",
        ]
        for joint_path in gripper_joints:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if not joint_prim.IsValid():
                print(f"Warning: Gripper joint not found at {joint_path}")
                continue
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetMaxForceAttr().Set(self._gripper_max_force)
            drive_api.GetStiffnessAttr().Set(self._gripper_stiffness)
            drive_api.GetDampingAttr().Set(self._gripper_damping)
            print(f"Set {joint_path}: maxForce=1, stiffness=0.1, damping=0.05")

        # Configure gripper physics material
        gripper_mat_path = "/World/RG2_Gripper/PhysicsMaterial"
        gripper_material = UsdShade.Material.Define(stage, gripper_mat_path)
        gripper_mat_api = UsdPhysics.MaterialAPI.Apply(gripper_material.GetPrim())
        gripper_mat_api.CreateDynamicFrictionAttr().Set(self._gripper_dynamic_friction)
        gripper_mat_api.CreateRestitutionAttr().Set(self._gripper_restitution)
        gripper_mat_api.CreateStaticFrictionAttr().Set(self._gripper_static_friction)
        gripper_physx_mat_api = PhysxSchema.PhysxMaterialAPI.Apply(gripper_material.GetPrim())
        gripper_physx_mat_api.CreateFrictionCombineModeAttr().Set(self._gripper_friction_combine_mode)
        gripper_physx_mat_api.CreateRestitutionCombineModeAttr().Set(self._gripper_restitution_combine_mode)
        print(f"Created gripper physics material at {gripper_mat_path}")

        # Bind physics material to finger body prims
        gripper_mat_sdf_path = Sdf.Path(gripper_mat_path)
        finger_prims = [
            f"{base_link}/left_inner_finger",
            f"{base_link}/right_inner_finger",
        ]
        for finger_path in finger_prims:
            finger_prim = stage.GetPrimAtPath(finger_path)
            if finger_prim and finger_prim.IsValid():
                from omni.physx.scripts import physicsUtils
                physicsUtils.add_physics_material_to_prim(stage, finger_prim, gripper_mat_sdf_path)
                print(f"Bound gripper physics material to {finger_path}")
            else:
                print(f"Warning: Finger prim not found at {finger_path}")

    def attach_rg2_to_ur5e(self):
        import omni.usd
        from pxr import Usd, Sdf, UsdGeom, Gf
        import math

        stage = omni.usd.get_context().get_stage()
        ur5e_gripper_path = "/World/UR5e/Gripper"
        rg2_path = "/World/RG2_Gripper"
        joint_path = "/World/UR5e/joints/robot_gripper_joint"
        rg2_base_link = "/World/RG2_Gripper/onrobot_rg2_base_link"

        ur5e_prim = stage.GetPrimAtPath(ur5e_gripper_path)
        rg2_prim = stage.GetPrimAtPath(rg2_path)
        joint_prim = stage.GetPrimAtPath(joint_path)

        if not ur5e_prim or not rg2_prim:
            print("Error: UR5e or RG2 gripper prim not found.")
            return

        # Copy transforms from UR5e gripper to RG2
        translate_attr = ur5e_prim.GetAttribute("xformOp:translate")
        orient_attr = ur5e_prim.GetAttribute("xformOp:orient")

        if translate_attr.IsValid() and orient_attr.IsValid():
            rg2_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(translate_attr.Get())
            rg2_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(orient_attr.Get())

        print("Setting RG2 gripper orientation with 180° Z offset and rotated position...")

        from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
        import numpy as np

        if rg2_prim.IsValid():
            # === 1. Apply rotated position ===
            original_pos = translate_attr.Get()
            # Get the UR5e root translate to account for its world position offset
            ur5e_root = stage.GetPrimAtPath("/World/UR5e")
            ur5e_translate = ur5e_root.GetAttribute("xformOp:translate").Get() if ur5e_root.IsValid() else Gf.Vec3d(0, 0, 0)
            x, y, z = original_pos[0], original_pos[1], original_pos[2]
            rotated_pos = Gf.Vec3d(-x + ur5e_translate[0], -y + ur5e_translate[1], z + ur5e_translate[2])

            # === 2. Apply combined orientation ===
            fixed_quat = Gf.Quatd(0.70711, Gf.Vec3d(-0.70711, 0.0, 0.0))
            offset_rpy_deg = np.array([0.0, 0.0, 180.0])
            offset_rpy_rad = np.deg2rad(offset_rpy_deg)
            offset_quat_arr = euler_angles_to_quats(offset_rpy_rad)
            offset_quat = Gf.Quatd(offset_quat_arr[0], offset_quat_arr[1], offset_quat_arr[2], offset_quat_arr[3])
            final_quat = offset_quat * fixed_quat

            # === 3. Apply to prim ===
            xform = UsdGeom.Xform(rg2_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(rotated_pos)
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(final_quat)

            print(f"RG2 position rotated to: {rotated_pos}")
            print("RG2 orientation set with fixed+180°Z rotation.")
        else:
            print(f"Gripper not found at {rg2_path}")


        # Create or update the physics joint
        if not joint_prim:
            joint_prim = stage.DefinePrim(joint_path, "PhysicsFixedJoint")

        joint_prim.CreateRelationship("physics:body1").SetTargets([Sdf.Path(rg2_base_link)])
        joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        joint_prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)

        # Set localRot0 and localRot1 for joint
        print("Setting joint rotation parameters...")
        if joint_prim.IsValid():
            def euler_to_quatf(x_deg, y_deg, z_deg):
                """Convert Euler angles (XYZ order, degrees) to Gf.Quatf"""
                rx = Gf.Quatf(math.cos(math.radians(x_deg) / 2), Gf.Vec3f(1, 0, 0) * math.sin(math.radians(x_deg) / 2))
                ry = Gf.Quatf(math.cos(math.radians(y_deg) / 2), Gf.Vec3f(0, 1, 0) * math.sin(math.radians(y_deg) / 2))
                rz = Gf.Quatf(math.cos(math.radians(z_deg) / 2), Gf.Vec3f(0, 0, 1) * math.sin(math.radians(z_deg) / 2))
                return rx * ry * rz  # Apply in XYZ order

            # Set the rotation quaternions for proper joint alignment
            quat0 = euler_to_quatf(-90, 0, -90)
            quat1 = euler_to_quatf(-180, 90, 0)
            
            joint_prim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat0)
            joint_prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat1)
            print(" Set physics:localRot0 and localRot1 for robot_gripper_joint.")
        else:
            print(f" Joint not found at {joint_path}")

        print("RG2 successfully attached to UR5e with proper orientation and joint configuration.")

    def import_realsense_camera(self):
        """Import Intel RealSense D455 camera"""
        usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        filename = os.path.splitext(os.path.basename(usd_path))[0]
        prim_path = f"/World/{filename}"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        print(f"Prim imported at {prim_path}")

    def attach_camera_to_ur5e(self):
        """Attach RealSense camera to UR5e wrist"""
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, Usd, UsdPhysics
        
        # Move the prim
        omni.kit.commands.execute('MovePrim',
                                 path_from="/World/rsd455",
                                 path_to="/World/UR5e/wrist_3_link/rsd455")
        
        # Set transform properties
        omni.kit.commands.execute('ChangeProperty',
                                 prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:translate",
                                 value=Gf.Vec3d(-0.012, -0.055, 0.1),
                                 prev=None)
        omni.kit.commands.execute('ChangeProperty',
                                 prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:rotateZYX",
                                 value=Gf.Vec3d(-90, -180, 270),
                                 prev=None)

        # Remove RigidBodyAPI only from the RSD455 prim to fix nested rigid body error.
        # Only target this specific prim — other descendants may be needed.
        stage = omni.usd.get_context().get_stage()
        rsd455_prim = stage.GetPrimAtPath("/World/UR5e/wrist_3_link/rsd455/RSD455")
        if rsd455_prim.IsValid() and rsd455_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(rsd455_prim).GetRigidBodyEnabledAttr().Set(False)
            rsd455_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            print("Removed RigidBodyAPI from RSD455 (nested under wrist_3_link rigid body)")

        print("RealSense camera attached to UR5e wrist_3_link")

    def setup_camera_action_graph(self):
        """Create ActionGraph for camera ROS2 publishing"""
        # Configuration
        CAMERA_PRIM = "/World/UR5e/wrist_3_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color" 
        IMAGE_WIDTH = 1280
        IMAGE_HEIGHT = 720
        ROS2_TOPIC = "intel_camera_rgb_sim"
        
        graph_path = "/World/Graphs/ActionGraph_Camera"  # Different name to avoid conflicts
        print(f"Creating ActionGraph: {graph_path}")
        print(f"Camera: {CAMERA_PRIM}")
        print(f"Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
        print(f"ROS2 Topic: {ROS2_TOPIC}")
        
        # Create ActionGraph
        try:
            og.Controller.create_graph(graph_path)
            print(f"Created ActionGraph at {graph_path}")
        except Exception:
            print(f"ActionGraph already exists at {graph_path}")
        
        # Create nodes
        nodes = [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        
        print("\nCreating nodes...")
        for node_name, node_type in nodes:
            try:
                node_path = f"{graph_path}/{node_name}"
                og.Controller.create_node(node_path, node_type)
                print(f"Created {node_name}")
            except Exception as e:
                print(f"Node {node_name} already exists")
        
        # Set node attributes
        print("\nConfiguring nodes...")
        # Configure render product
        try:
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:cameraPrim").set([CAMERA_PRIM])
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:width").set(IMAGE_WIDTH)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:height").set(IMAGE_HEIGHT)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:enabled").set(True)
            print(f"Configured render product: {CAMERA_PRIM} @ {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
        except Exception as e:
            print(f"Error configuring render product: {e}")
        
        # Configure ROS2 camera helper
        try:
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:topicName").set(ROS2_TOPIC)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:frameId").set("camera_link")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:type").set("rgb")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:enabled").set(True)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:queueSize").set(10)
            print(f"Configured ROS2 helper: topic={ROS2_TOPIC}")
        except Exception as e:
            print(f"Error configuring ROS2 helper: {e}")
        
        # Create connections
        print("\nConnecting nodes...")
        connections = [
            ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
            ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
            ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
        ]
        
        for source, target in connections:
            try:
                og.Controller.connect(f"{graph_path}/{source}", f"{graph_path}/{target}")
                print(f"Connected {source.split('.')[0]} -> {target.split('.')[0]}")
            except Exception as e:
                print(f"Failed to connect {source} -> {target}: {e}")
        
        print("\nCamera ActionGraph created successfully!")
        print(f"Test with: ros2 topic echo /{ROS2_TOPIC}")

    def create_additional_camera(self):
        """Create additional camera based on selected view type"""
        import omni.usd
        from pxr import Gf, UsdGeom
        import numpy as np

        def set_camera_pose(prim_path: str, position_xyz, quat_xyzw):
            """Apply translation and quaternion orientation (x, y, z, w) to a camera prim."""
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
            """Configure camera properties based on resolution"""
            # Calculate focal length and aperture based on resolution
            # Using standard 35mm film equivalent calculations
            # Horizontal aperture in mm (standard 35mm film is 36mm wide)
            horizontal_aperture_mm = 36.0
            # Vertical aperture calculated from aspect ratio
            aspect_ratio = height / width
            vertical_aperture_mm = horizontal_aperture_mm * aspect_ratio
            
            # Focal length (50mm is a standard "normal" lens)
            focal_length_mm = 50.0
            
            # Convert mm to USD units (USD uses cm, but aperture is in mm in USD)
            camera = UsdGeom.Camera(camera_prim)
            camera.CreateHorizontalApertureAttr().Set(horizontal_aperture_mm)
            camera.CreateVerticalApertureAttr().Set(vertical_aperture_mm)
            camera.CreateFocalLengthAttr().Set(focal_length_mm)
            camera.CreateProjectionAttr().Set("perspective")
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))

        # Check which camera type is selected
        is_workspace = self._workspace_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()

        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Error: No stage found")
            return

        if is_workspace:
            prim_path = "/World/workspace_camera"
            position = (1.80632, -2.25319, 1.70657)
            # Euler XYZ: (52.147, 39.128, 26.127) degrees
            quat_xyzw = (0.4714, 0.1994, 0.3347, 0.7912)  # x, y, z, w
            resolution = (1280, 720)

            # Create camera prim using UsdGeom.Camera
            camera_prim = UsdGeom.Camera.Define(stage, prim_path)
            if not camera_prim:
                print(f"Error: Failed to create camera at {prim_path}")
                return

            # Configure camera properties
            configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])

            # Set camera pose
            set_camera_pose(prim_path, position, quat_xyzw)
            print(f"Workspace camera created at {prim_path} with resolution {resolution[0]}x{resolution[1]}")

        if is_custom:
            # Get custom prim path from text field
            custom_prim_path = self._custom_camera_prim_field.model.get_value_as_string()
            if not custom_prim_path or custom_prim_path.strip() == "":
                print("Error: Please enter a valid camera prim path")
                return
            
            # Check if the prim already exists
            existing_prim = stage.GetPrimAtPath(custom_prim_path)
            if existing_prim and existing_prim.IsValid():
                print(f"Camera prim already exists at {custom_prim_path}. Using existing prim.")
                camera_prim = UsdGeom.Camera(existing_prim)
                if not camera_prim:
                    # If it exists but is not a camera, convert it
                    print(f"Prim at {custom_prim_path} exists but is not a camera. Creating camera...")
                    camera_prim = UsdGeom.Camera.Define(stage, custom_prim_path)
            else:
                # Create new camera prim at the specified path
                camera_prim = UsdGeom.Camera.Define(stage, custom_prim_path)
                if not camera_prim:
                    print(f"Error: Failed to create camera at {custom_prim_path}")
                    return
            
            # Default resolution for custom camera
            resolution = (1280, 720)
            
            # Configure camera properties
            configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])
            
            # Note: Pose is not set for custom prim - user should position it manually or it uses existing pose
            print(f"Custom camera created/updated at {custom_prim_path} with resolution {resolution[0]}x{resolution[1]}")

    def create_additional_camera_actiongraph(self):
        """Create ActionGraph for additional camera ROS2 publishing"""
        # Check which cameras exist and create action graphs accordingly
        is_workspace = self._workspace_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()

        # Get selected resolution from combo box
        resolution_index = self._resolution_combo.model.get_item_value_model().get_value_as_int()
        if resolution_index == 0:
            width, height = 1280, 720
        else:
            width, height = 1920, 1080

        if is_workspace:
            self._create_camera_actiongraph(
                "/World/workspace_camera",
                width, height,
                "workspace_camera",
                "WorkspaceCamera"
            )

        if is_custom:
            # Get custom prim path from text field
            custom_prim_path = self._custom_camera_prim_field.model.get_value_as_string()
            if not custom_prim_path or custom_prim_path.strip() == "":
                print("Error: Please enter a valid camera prim path")
                return

            # Get ROS2 topic name from text field, or generate from prim path if empty
            topic_name = self._custom_camera_topic_field.model.get_value_as_string()
            if not topic_name or topic_name.strip() == "":
                # Extract a clean name for the topic from the prim path if topic is empty
                prim_name = custom_prim_path.split("/")[-1] if "/" in custom_prim_path else custom_prim_path
                topic_name = prim_name.lower().replace(" ", "_").replace("-", "_")
                print(f"Topic name not specified, using generated name: {topic_name}")

            # Use topic name for graph suffix (sanitize for graph name - remove special chars, capitalize)
            # Convert topic name to a valid graph suffix: remove underscores, capitalize words
            graph_suffix = topic_name.replace("_", " ").replace("-", " ").title().replace(" ", "")

            self._create_camera_actiongraph(
                custom_prim_path,
                width, height,
                topic_name,
                graph_suffix
            )

    def _create_camera_actiongraph(self, camera_prim, width, height, topic, graph_suffix):
        """Helper method to create camera ActionGraph

        TODO: Camera action graphs slow down physics and stop producing frames as FPS drops.
        Root cause: OnPlaybackTick fires every render frame, and IsaacRunOneSimulationFrame
        couples rendering to the physics step. When physics slows (e.g. SDF collisions, many
        objects), render FPS drops, which starves the camera pipeline. Possible fixes:
          - Decouple camera publishing from the physics tick (use a separate event-based or
            timer-driven graph instead of OnPlaybackTick → IsaacRunOneSimulationFrame chain)
          - Use OnImpulseEvent with a fixed-rate async timer instead of OnPlaybackTick
          - Lower render product resolution or publish at a reduced rate (e.g. skip N frames)
          - Profile whether the ROS2CameraHelper itself is the bottleneck (GPU readback stall)
        """
        graph_path = f"/World/Graphs/ActionGraph_{graph_suffix}"
        print(f"Creating ActionGraph: {graph_path}")
        print(f"Camera: {camera_prim}")
        print(f"Resolution: {width}x{height}")
        print(f"ROS2 Topic: {topic}")
        
        # Create ActionGraph
        try:
            og.Controller.create_graph(graph_path)
            print(f"Created ActionGraph at {graph_path}")
        except Exception:
            print(f"ActionGraph already exists at {graph_path}")
        
        # Create nodes
        nodes = [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        
        print("\nCreating nodes...")
        for node_name, node_type in nodes:
            try:
                node_path = f"{graph_path}/{node_name}"
                og.Controller.create_node(node_path, node_type)
                print(f"Created {node_name}")
            except Exception as e:
                print(f"Node {node_name} already exists")
        
        # Set node attributes
        print("\nConfiguring nodes...")
        
        # Configure render product
        try:
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:cameraPrim").set([camera_prim])
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:width").set(width)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:height").set(height)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:enabled").set(True)
            print(f"Configured render product: {camera_prim} @ {width}x{height}")
        except Exception as e:
            print(f"Error configuring render product: {e}")
        
        # Configure ROS2 camera helper
        try:
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:topicName").set(topic)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:frameId").set("camera_link")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:type").set("rgb")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:enabled").set(True)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:queueSize").set(10)
            print(f"Configured ROS2 helper: topic={topic}")
        except Exception as e:
            print(f"Error configuring ROS2 helper: {e}")
        
        # Create connections
        print("\nConnecting nodes...")
        connections = [
            ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
            ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
            ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
        ]
        
        for source, target in connections:
            try:
                og.Controller.connect(f"{graph_path}/{source}", f"{graph_path}/{target}")
                print(f"Connected {source.split('.')[0]} -> {target.split('.')[0]}")
            except Exception as e:
                print(f"Failed to connect {source} -> {target}: {e}")
        
        print(f"\n{graph_suffix} ActionGraph created successfully!")
        print(f"Test with: ros2 topic echo /{topic}")

    def _load_object_type_map(self):
        """Read the assembly JSON and return a dict mapping object name -> category (board/block/peg/socket).

        JSON uses type="board" for boards, and type="object" with subtype for the rest.
        Returns the effective category for prim settings lookup.
        """
        type_map = {}
        assembly_file = self._assembly_file_path
        if assembly_file and os.path.exists(assembly_file):
            with open(assembly_file, 'r') as f:
                assembly_data = json.load(f)
            for component in assembly_data.get('components', []):
                comp_type = component.get('type', 'object')
                if comp_type == 'board':
                    category = 'board'
                else:
                    category = component.get('subtype', 'block')
                type_map[component['name']] = category
        else:
            print(f"Warning: Assembly file not found: {assembly_file}, defaulting all objects to 'block'")
        return type_map

    def _get_prim_params(self, category):
        """Return prim settings dict for the given category (board/block/peg/socket)."""
        prefix = f"_{category}_"
        return {
            "collision_approximation": getattr(self, f"{prefix}collision_approximation"),
            "rest_offset": getattr(self, f"{prefix}rest_offset"),
            "angular_damping": getattr(self, f"{prefix}angular_damping"),
        }

    def _sample_non_overlapping_objects(
        self,
        num_objects,
        x_range=(-0.2, 0.5),
        y_range=(-0.5, -0.3),
        min_sep=0.2,
        yaw_range=(-180.0, 180.0),
        z_values=None,
        fixed_positions=None,
        max_attempts=10_000,
        max_retries=100,
    ):
        """
        Sample non-overlapping object poses in world frame.

        Args:
            num_objects: Number of objects to place
            x_range: X position range in world frame (default: -0.5 to 0.5)
            y_range: Y position range in world frame (default: -0.5 to 0.0)
            min_sep: Minimum separation between objects
            yaw_range: Yaw rotation range in degrees
            z_values: List of Z values for each object (preserves current Z)
            fixed_positions: List of fixed positions (e.g., base objects) to avoid
            max_attempts: Maximum placement attempts per retry
            max_retries: Maximum number of times to restart the entire placement
        """
        # Convert fixed positions to numpy arrays for distance checking
        fixed_xy = []
        if fixed_positions:
            for pos in fixed_positions:
                if isinstance(pos, (list, tuple, np.ndarray)):
                    fixed_xy.append(np.array(pos[:2]))  # Only X, Y
                elif hasattr(pos, '__getitem__'):
                    fixed_xy.append(np.array([pos[0], pos[1]]))

        for retry in range(max_retries):
            poses, attempts = [], 0

            while len(poses) < num_objects and attempts < max_attempts:
                attempts += 1
                candidate_xy = np.array([
                    np.random.uniform(*x_range),
                    np.random.uniform(*y_range)
                ])
                # Check separation against both existing poses AND fixed positions
                valid = True
                # Check against existing randomized poses
                if any(np.linalg.norm(candidate_xy - p["position"][:2]) < min_sep for p in poses):
                    valid = False
                # Check against fixed positions (base objects)
                if valid and fixed_xy:
                    if any(np.linalg.norm(candidate_xy - fixed) < min_sep for fixed in fixed_xy):
                        valid = False

                if valid:
                    yaw_deg = np.random.uniform(*yaw_range)
                    # Use provided Z value or default
                    z = z_values[len(poses)] if z_values and len(poses) < len(z_values) else 0.0495
                    poses.append({
                        "position": np.array([candidate_xy[0], candidate_xy[1], z]),
                        "yaw_deg": yaw_deg
                    })

            if len(poses) == num_objects:
                if retry > 0:
                    print(f"Placement succeeded after {retry + 1} retries")
                return poses

        raise RuntimeError("Could not place all objects without violating min_sep; reduce density.")

    def _set_obj_prim_pose(self, prim_path, position, quat_wxyz):
        import omni.kit.commands
        # Handle both Gf.Vec3d and tuple/list positions
        if isinstance(position, Gf.Vec3d):
            pos_value = position
        else:
            pos_value = Gf.Vec3d(*position)
        
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=f"{prim_path}.xformOp:translate",
            value=pos_value,
            prev=None,
        )
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=f"{prim_path}.xformOp:orient",
            value=Gf.Quatf(*quat_wxyz),
            prev=None,
        )

    def assemble_objects(self, folder_path="/World/Objects"):
        """Assemble objects based on JSON assembly data, positioned relative to the current board pose."""
        assembly_file = self._assembly_file_path
        if not os.path.exists(assembly_file):
            print(f"Error: Assembly file not found: {assembly_file}")
            return

        print(f"Loading assembly data from {assembly_file}...")
        with open(assembly_file, 'r') as f:
            assembly_data = json.load(f)

        stage = omni.usd.get_context().get_stage()

        # Find the board object's current world position to use as assembly origin
        type_map = self._load_object_type_map()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root.IsValid():
            print(f"Warning: {folder_path} does not exist")
            return

        base_world_pos = Gf.Vec3d(0, 0, 0)
        for child in objects_root.GetChildren():
            obj_name = child.GetName()
            if type_map.get(obj_name, 'block') == 'board':
                child_path = f"{folder_path}/{obj_name}/{obj_name}/{obj_name}"
                child_prim = stage.GetPrimAtPath(child_path)
                if child_prim and child_prim.IsValid():
                    child_xform = UsdGeom.Xformable(child_prim)
                    world_transform = child_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    base_world_pos = world_transform.ExtractTranslation()
                    print(f"Board '{obj_name}' at world pos: ({base_world_pos[0]:.4f}, {base_world_pos[1]:.4f}, {base_world_pos[2]:.4f})")
                break

        # Position each component from assembly data
        for component in assembly_data['components']:
            name = component['name']
            position = component['position']
            rotation = component['rotation']

            prim_path = f"{folder_path}/{name}/{name}/{name}"
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                print(f"Warning: Prim not found at {prim_path}, skipping {name}")
                continue

            # Assembly position offset relative to base
            assembly_pos = Gf.Vec3d(
                base_world_pos[0] + position['x'],
                base_world_pos[1] + position['y'],
                base_world_pos[2] + position['z']
            )

            # Use quaternion directly from assembly data (w, x, y, z for Gf.Quatf)
            q = rotation['quaternion']
            orient_quat = Gf.Quatf(float(q['w']), float(q['x']), float(q['y']), float(q['z']))

            # Set orientation first, then translate into position
            omni.kit.commands.execute('ChangeProperty',
                prop_path=f"{prim_path}.xformOp:orient",
                value=orient_quat,
                prev=None)

            omni.kit.commands.execute('ChangeProperty',
                prop_path=f"{prim_path}.xformOp:translate",
                value=assembly_pos,
                prev=None)

            print(f"Assembled {name} at ({assembly_pos[0]:.4f}, {assembly_pos[1]:.4f}, {assembly_pos[2]:.4f})")

        print("Assembly complete!")

    def disassemble_objects(self, folder_path="/World/Objects"):
        """Lay out objects spaced apart along the X-axis (like initial add_objects positioning)."""
        stage = omni.usd.get_context().get_stage()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root.IsValid():
            print(f"Warning: {folder_path} does not exist")
            return

        i = 0
        for child in objects_root.GetChildren():
            obj_name = child.GetName()
            if obj_name.endswith("Material"):
                continue

            child_path = f"{folder_path}/{obj_name}/{obj_name}/{obj_name}"
            child_prim = stage.GetPrimAtPath(child_path)
            if not child_prim or not child_prim.IsValid():
                print(f"Warning: Prim not found at {child_path}, skipping {obj_name}")
                continue

            # Calculate spaced position: first at center, then alternating +X and -X
            if i == 0:
                x_position = 0.0
            elif i % 2 == 1:
                x_position = self._object_spacing * ((i + 1) // 2)
            else:
                x_position = -self._object_spacing * (i // 2)

            pos = Gf.Vec3d(x_position, self._y_offset, self._z_offset)

            omni.kit.commands.execute('ChangeProperty',
                prop_path=f"{child_path}.xformOp:translate",
                value=pos,
                prev=None)

            # Reset orientation to identity quaternion (same op type as randomize uses)
            omni.kit.commands.execute('ChangeProperty',
                prop_path=f"{child_path}.xformOp:orient",
                value=Gf.Quatf(1, 0, 0, 0),
                prev=None)

            print(f"Disassembled {obj_name} to ({x_position:.3f}, {self._y_offset:.3f}, {self._z_offset:.3f})")
            i += 1

        print("Disassembly complete!")

    def randomize_object_poses(self, folder_path="/World/Objects"):
        """
        Randomize object poses in world frame, then transform to local child prim frame.
        Randomizes position of /World/Objects/{object_name} in world coordinates,
        then applies the appropriate local transform to the child prim.
        """
        stage = omni.usd.get_context().get_stage()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root.IsValid():
            print(f"Warning: {folder_path} does not exist")
            return

        # Collect object info: parent path, child path, parent's current world position, and child's current Z
        # Also collect board positions to respect during randomization
        type_map = self._load_object_type_map()
        object_info = []
        board_positions = []  # Store board world positions

        for child in objects_root.GetChildren():
            obj = child.GetName()
            parent_path = f"{folder_path}/{obj}"
            child_path = f"{folder_path}/{obj}/{obj}/{obj}"

            parent_prim = stage.GetPrimAtPath(parent_path)
            child_prim = stage.GetPrimAtPath(child_path)

            if not parent_prim.IsValid() or not child_prim.IsValid():
                continue

            # Get child's current world transform
            child_xform = UsdGeom.Xformable(child_prim)
            child_world_transform = child_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            child_world_pos = child_world_transform.ExtractTranslation()

            # Check if this is a board object
            if type_map.get(obj, 'block') == 'board':
                # Store board position for separation checking
                board_positions.append(child_world_pos)
                print(f"Skipping {obj} (board) at world pos: ({child_world_pos[0]:.3f}, {child_world_pos[1]:.3f}, {child_world_pos[2]:.3f})")
                continue
            
            # Get parent's current world transform and position
            parent_xform = UsdGeom.Xformable(parent_prim)
            parent_world_transform = parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            parent_world_pos = parent_world_transform.ExtractTranslation()
            
            object_info.append({
                "parent_path": parent_path,
                "child_path": child_path,
                "parent_world_pos": parent_world_pos,
                "parent_world_transform": parent_world_transform,
                "current_z": child_world_pos[2]
            })
            print(f"Found: {child_path}")
            print(f"  Parent world pos: ({parent_world_pos[0]:.3f}, {parent_world_pos[1]:.3f}, {parent_world_pos[2]:.3f})")
            print(f"  Child world pos: ({child_world_pos[0]:.3f}, {child_world_pos[1]:.3f}, {child_world_pos[2]:.3f})")

        if not object_info:
            print("No valid objects found")
            return

        # Get Z values for preserving current heights
        z_values = [info["current_z"] for info in object_info]
        
        # Randomize positions in world frame
        # Pass board positions so randomization respects them for minimum separation
        poses = self._sample_non_overlapping_objects(
            num_objects=len(object_info),
            z_values=z_values,
            fixed_positions=board_positions
        )

        if board_positions:
            print(f"Randomization will respect {len(board_positions)} board positions for minimum separation")

        # Apply randomized poses
        for obj_info, pose in zip(object_info, poses):
            parent_path = obj_info["parent_path"]
            child_path = obj_info["child_path"]
            parent_world_transform = obj_info["parent_world_transform"]
            parent_world_pos = obj_info["parent_world_pos"]
            
            # Target world position (randomized)
            target_world_pos = Gf.Vec3d(pose["position"][0], pose["position"][1], pose["position"][2])
            
            # Calculate local position relative to parent
            # The parent's world transform already accounts for its current position
            # Local = Parent^-1 * Target_World
            # This correctly transforms the target world position to local coordinates
            # accounting for where the parent currently is
            parent_inverse = parent_world_transform.GetInverse()
            local_pos = parent_inverse.Transform(target_world_pos)
            
            # Convert yaw to quaternion
            quat_xyzw = R.from_euler("xyz", [0.0, 0.0, pose["yaw_deg"]], degrees=True).as_quat()
            quat_wxyz = np.roll(quat_xyzw, 1)
            
            # Apply local transform to child prim (parent prim stays unchanged)
            self._set_obj_prim_pose(child_path, local_pos, quat_wxyz)
            
            print(f"Randomized {child_path}:")
            print(f"  Parent world pos (unchanged): ({parent_world_pos[0]:.3f}, {parent_world_pos[1]:.3f}, {parent_world_pos[2]:.3f})")
            print(f"  Target world pos: ({target_world_pos[0]:.3f}, {target_world_pos[1]:.3f}, {target_world_pos[2]:.3f})")
            print(f"  Child local pos: ({local_pos[0]:.3f}, {local_pos[1]:.3f}, {local_pos[2]:.3f})")

    def delete_objects(self, folder_path="/World/Objects"):
        """Delete the Objects folder from the scene. Stops simulation first to avoid tensor view crash."""
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

        # Also delete the object poses action graph if present
        pose_graph_path = "/World/Graphs/ActionGraph_objects_poses"
        pose_graph_prim = stage.GetPrimAtPath(pose_graph_path)
        if pose_graph_prim and pose_graph_prim.IsValid():
            stage.RemovePrim(pose_graph_path)
            print(f"Deleted {pose_graph_path}")

    def add_objects(self):
        """Import all objects from the selected assembly's folder into the scene"""
        folder_path = self._objects_folder_path
        if not folder_path:
            print("Error: No folder path for selected assembly")
            return
        
        # Check if path is a single USD file or a folder
        if folder_path.endswith(('.usd', '.usda', '.usdc')):
            # Single file: import it directly
            print(f"Importing single file: {folder_path}")
            stage = omni.usd.get_context().get_stage()
            target_path = "/World/Objects"
            if not stage.GetPrimAtPath(target_path):
                UsdGeom.Xform.Define(stage, target_path)
            base_name = os.path.splitext(os.path.basename(folder_path))[0]
            prim_path = f"{target_path}/{base_name}"

            # Skip if prim already exists
            existing_prim = stage.GetPrimAtPath(prim_path)
            if existing_prim and existing_prim.IsValid():
                print(f"Skipping {base_name} - already exists at {prim_path}")
                return

            prim = stage.DefinePrim(prim_path)
            prim.GetReferences().AddReference(folder_path)
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, self._y_offset, self._z_offset))
            print(f"Added {folder_path} to {prim_path}")
            return

        # Ensure folder path ends with /
        if not folder_path.endswith("/"):
            folder_path += "/"

        print(f"Adding objects from folder: {folder_path}")

        # Get the current stage and selection
        stage = omni.usd.get_context().get_stage()
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()

        # Always use /World/Objects as target path
        target_path = "/World/Objects"
        if not stage.GetPrimAtPath(target_path):
            UsdGeom.Xform.Define(stage, target_path)

        # List all files in the folder
        result, entries = omni.client.list(folder_path)

        if result == omni.client.Result.OK:
            # Filter for USD files
            usd_files = [entry.relative_path for entry in entries 
                         if entry.relative_path.endswith(('.usd', '.usda', '.usdc'))]
            
            print(f"Found {len(usd_files)} USD files")

            # Load object type map from assembly file
            type_map = self._load_object_type_map()

            # Create single common physics material for all objects
            physics_mat_path = f"{target_path}/PhysicsMaterial"
            material = UsdShade.Material.Define(stage, physics_mat_path)
            physics_mat_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            physics_mat_api.CreateDynamicFrictionAttr().Set(self._object_dynamic_friction)
            physics_mat_api.CreateRestitutionAttr().Set(self._object_restitution)
            physics_mat_api.CreateStaticFrictionAttr().Set(self._object_static_friction)
            physx_mat_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            physx_mat_api.CreateFrictionCombineModeAttr().Set(self._object_friction_combine_mode)
            physx_mat_api.CreateRestitutionCombineModeAttr().Set(self._object_restitution_combine_mode)
            print(f"Created common physics material at {physics_mat_path}")

            # Import each USD file with positioning
            for i, usd_file in enumerate(usd_files):
                usd_file_path = folder_path + usd_file

                # Create a child prim under the selected path
                base_name = os.path.splitext(usd_file)[0]
                prim_path = f"{target_path}/{base_name}"

                # Skip if prim already exists
                existing_prim = stage.GetPrimAtPath(prim_path)
                if existing_prim and existing_prim.IsValid():
                    print(f"Skipping {base_name} - already exists at {prim_path}")
                    continue

                # Create a reference to the USD file
                prim = stage.DefinePrim(prim_path)
                references = prim.GetReferences()
                references.AddReference(usd_file_path)
                
                # Calculate position: first at center, then alternating +X and -X
                if i == 0:
                    x_position = 0.0
                else:
                    if i % 2 == 1:  # Odd indices: +X direction
                        x_position = self._object_spacing * ((i + 1) // 2)
                    else:  # Even indices: -X direction
                        x_position = -self._object_spacing * (i // 2)

                # Rename Body1 to match the object name
                def rename_body1_to_object_name(stage, prim_path, object_name):
                    """Recursively search for Body1 and rename it to object_name"""
                    prim = stage.GetPrimAtPath(prim_path)
                    if not prim:
                        return False
                    
                    # Search for Body1 in the hierarchy
                    for child in prim.GetAllChildren():
                        if child.GetName() == "Body1":
                            # Found Body1, rename it
                            new_path = child.GetPath().GetParentPath().AppendChild(object_name)
                            omni.kit.commands.execute('MovePrim',
                                path_from=child.GetPath(),
                                path_to=new_path)
                            print(f"Renamed Body1 to {object_name} at {new_path}")
                            return True
                        # Recursively search in children
                        if rename_body1_to_object_name(stage, child.GetPath(), object_name):
                            return True
                    return False
                
                # Try to rename Body1 to the object name
                rename_body1_to_object_name(stage, prim_path, base_name)

                # Apply position to the final body prim: {name}/{name}/{name}
                # Apply position to the final body prim without clearing inherited xform ops
                body_path = f"{prim_path}/{base_name}/{base_name}"
                body_prim = stage.GetPrimAtPath(body_path)
                if body_prim and body_prim.IsValid():
                    omni.kit.commands.execute('ChangeProperty',
                        prop_path=f"{body_path}.xformOp:translate",
                        value=Gf.Vec3d(x_position, self._y_offset, self._z_offset),
                        prev=None)
                    print(f"Added {usd_file_path} to {body_path} at position ({x_position}, {self._y_offset}, {self._z_offset})")
                else:
                    print(f"Warning: Body prim not found at {body_path}, positioning parent instead")
                    omni.kit.commands.execute('ChangeProperty',
                        prop_path=f"{prim_path}.xformOp:translate",
                        value=Gf.Vec3d(x_position, self._y_offset, self._z_offset),
                        prev=None)
                    print(f"Added {usd_file_path} to {prim_path} at position ({x_position}, {self._y_offset}, {self._z_offset})")

            # Bind common physics material and apply per-category prim settings
            from omni.physx.scripts import physicsUtils
            physics_mat_sdf_path = Sdf.Path(physics_mat_path)
            objects_prim = stage.GetPrimAtPath(target_path)
            if objects_prim.IsValid():
                for child in objects_prim.GetChildren():
                    child_name = child.GetName()
                    if child_name == "PhysicsMaterial":
                        continue
                    # Determine category from assembly file (board/block/peg/socket)
                    category = type_map.get(child_name, 'block')
                    prim_params = self._get_prim_params(category)
                    # Walk all descendants and bind common material to prims with CollisionAPI
                    bound_count = 0
                    for desc in Usd.PrimRange(child):
                        if desc.HasAPI(UsdPhysics.CollisionAPI):
                            physicsUtils.add_physics_material_to_prim(stage, desc, physics_mat_sdf_path)
                            bound_count += 1
                            print(f"  Bound physics material to collision prim: {desc.GetPath()}")
                    # Set per-category prim settings on the mesh prim: {name}/{name}/{name}
                    mesh_path = f"{target_path}/{child_name}/{child_name}/{child_name}"
                    mesh_prim = stage.GetPrimAtPath(mesh_path)
                    if mesh_prim and mesh_prim.IsValid():
                        approx = prim_params["collision_approximation"]
                        UsdPhysics.CollisionAPI.Apply(mesh_prim)
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                        if approx == "sdf":
                            mesh_collision_api.CreateApproximationAttr(PhysxSchema.Tokens.sdf)
                            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(mesh_prim)
                            sdf_api.CreateSdfResolutionAttr(self._sdf_resolution)
                            print(f"  Set SDF mesh (resolution={self._sdf_resolution}) on {mesh_path} [{category}]")
                        else:
                            mesh_collision_api.CreateApproximationAttr(approx)
                            print(f"  Set {approx} collision on {mesh_path} [{category}]")
                        # Set contact/rest offset
                        rest_offset = prim_params["rest_offset"]
                        physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
                        physx_collision_api.CreateContactOffsetAttr().Set(self._contact_offset)
                        physx_collision_api.CreateRestOffsetAttr().Set(rest_offset)
                        print(f"  Set contactOffset={self._contact_offset}, restOffset={rest_offset} on {mesh_path} [{category}]")
                        # Apply angular damping
                        angular_damping = prim_params["angular_damping"]
                        physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(mesh_prim)
                        physx_rb_api.CreateAngularDampingAttr().Set(angular_damping)
                        print(f"  Set angularDamping={angular_damping} on {mesh_path} [{category}]")
                    else:
                        print(f"  Warning: Mesh prim not found at {mesh_path}")

                    if bound_count == 0:
                        # No collision prims found, bind to the nested child directly
                        nested_path = f"{target_path}/{child_name}/{child_name}"
                        nested_prim = stage.GetPrimAtPath(nested_path)
                        if nested_prim and nested_prim.IsValid():
                            physicsUtils.add_physics_material_to_prim(stage, nested_prim, physics_mat_sdf_path)
                            print(f"  Bound physics material to {nested_path} (no collision prims found)")
                    else:
                        print(f"Bound physics material to {bound_count} collision prim(s) under {child_name} [{category}]")
        else:
            print(f"Failed to list folder: {folder_path}")

    def create_pose_publisher(self):
        """Create action graph for publishing object poses to ROS2"""
        import omni.kit.commands
        from pxr import Sdf, Usd, UsdGeom
        import omni.usd
        import omni.graph.core as og

        def get_objects_in_folder(stage, folder_path="/World/Objects"):
            """
            Scan the Objects folder and find all prims following the pattern:
            /World/Objects/{object_name}/{object_name}/{object_name}
            
            Args:
                stage: USD Stage
                folder_path: Path to the Objects folder
            
            Returns:
                List of paths to object prims
            """
            body_paths = []
            
            # Get the Objects prim
            objects_prim = stage.GetPrimAtPath(folder_path)
            
            if not objects_prim.IsValid():
                print(f"Warning: {folder_path} does not exist")
                return body_paths
            
            # Iterate through children (e.g., fork_orange, base, fork_yellow, line_brown)
            for child in objects_prim.GetChildren():
                object_name = child.GetName()
                
                # Look for the nested structure: {object_name}/{object_name}/object_name{}
                nested_path = f"{folder_path}/{object_name}/{object_name}/{object_name}"
                nested_prim = stage.GetPrimAtPath(nested_path)
                
                if nested_prim.IsValid():
                    body_paths.append(nested_path)
                    print(f"Found: {nested_path}")
            
            return body_paths

        def create_action_graph_with_transforms(target_prims, parent_prim="/World", topic_name="objects_poses_sim"):
            """
            Create an action graph with OnPlaybackTick and ROS2PublishTransformTree nodes
            
            Args:
                target_prims: List of prim paths to publish transforms for
                parent_prim: Parent prim for the transform tree
                topic_name: ROS2 topic name
            """
            
            graph_path = "/World/Graphs/ActionGraph_objects_poses"

            # Check if graph already exists
            stage = omni.usd.get_context().get_stage()
            existing_graph = stage.GetPrimAtPath(graph_path)
            if existing_graph and existing_graph.IsValid():
                print(f"Action graph already exists at {graph_path}, skipping creation")
                return

            # Create the action graph using OmniGraph API
            keys = og.Controller.Keys
            
            (graph, nodes, _, _) = og.Controller.edit(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                },
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
                        ("ros2_publish_transform_tree.inputs:topicName", topic_name),
                        ("isaac_read_simulation_time.inputs:resetOnStop", True),
                    ],
                },
            )
            
            # Get the stage to set relationships
            stage = omni.usd.get_context().get_stage()
            
            # Get the ROS2 node prim
            ros2_node_path = f"{graph_path}/ros2_publish_transform_tree"
            ros2_prim = stage.GetPrimAtPath(ros2_node_path)
            
            if ros2_prim.IsValid():
                # Set parent prim as a relationship
                parent_rel = ros2_prim.GetRelationship("inputs:parentPrim")
                if not parent_rel:
                    parent_rel = ros2_prim.CreateRelationship("inputs:parentPrim", custom=True)
                parent_rel.SetTargets([Sdf.Path(parent_prim)])
                
                # Set target prims as a relationship
                target_rel = ros2_prim.GetRelationship("inputs:targetPrims")
                if not target_rel:
                    target_rel = ros2_prim.CreateRelationship("inputs:targetPrims", custom=True)
                target_paths = [Sdf.Path(path) for path in target_prims]
                target_rel.SetTargets(target_paths)
                
                print(f"Action graph created at {graph_path}")
                print(f"Publishing {len(target_prims)} objects to topic: {topic_name}")
            else:
                print(f"Error: Could not find ROS2 node at {ros2_node_path}")

        # Get the current USD stage
        stage = omni.usd.get_context().get_stage()
        
        if not stage:
            print("Error: No stage found")
            return
        
        # Get all Body1 prims from the Objects folder
        object_paths = get_objects_in_folder(stage, "/World/Objects")
        
        if not object_paths:
            print("No objects found in /World/Objects")
            return
        
        print(f"\nFound {len(object_paths)} objects:")
        for path in object_paths:
            print(f"  - {path}")
        
        # Create the action graph with all found objects
        create_action_graph_with_transforms(
            target_prims=object_paths,
            parent_prim="/World",
            topic_name="objects_poses_sim"
        )

    # ==================== MCP Socket Server Methods ====================

    def _start_mcp_server(self):
        """Start the MCP socket server."""
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
        """Stop the MCP socket server."""
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
                    self._mcp_server_thread.join(timeout=1.0)
            except:
                pass
            self._mcp_server_thread = None

        print("[MCP] Server stopped")

    def _mcp_server_loop(self):
        """Main server loop in a separate thread."""
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
                except socket.timeout:
                    continue
                except OSError:
                    # Socket closed during shutdown - this is expected
                    if not self._mcp_server_running:
                        break
                except Exception as e:
                    if self._mcp_server_running:
                        print(f"[MCP] Error accepting connection: {str(e)}")
                    import time
                    time.sleep(0.5)
            except Exception as e:
                if self._mcp_server_running:
                    print(f"[MCP] Error in server loop: {str(e)}")
                if not self._mcp_server_running:
                    break
                import time
                time.sleep(0.5)

    def _handle_mcp_client(self, client):
        """Handle connected client."""
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

                        # Execute command in Isaac Sim's main thread
                        async def execute_wrapper():
                            try:
                                response = self._execute_mcp_command(command)
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
                                    error_response = {
                                        "status": "error",
                                        "message": str(e)
                                    }
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

    def _execute_mcp_command(self, command) -> Dict[str, Any]:
        """Execute a command and return the response."""
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})

            # Special command: list_available_tools - returns the tool registry
            if cmd_type == "list_available_tools":
                return {
                    "status": "success",
                    "result": {
                        "status": "success",
                        "tools": MCP_TOOL_REGISTRY
                    }
                }

            # Look up handler from MCP_HANDLERS defined at top of file
            handler_name = MCP_HANDLERS.get(cmd_type)
            if handler_name:
                handler = getattr(self, handler_name, None)
            else:
                handler = None

            if handler:
                try:
                    print(f"[MCP] Executing handler for {cmd_type}")
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

    # ==================== Common MCP Handlers ====================

    def _cmd_execute_python_code(self, code: str) -> Dict[str, Any]:
        """Execute Python code in Isaac Sim's environment.

        Args:
            code: Python code to execute

        Returns:
            Dictionary with execution result.
        """
        try:
            # Create a local namespace with commonly used modules
            local_ns = {}
            local_ns["omni"] = omni
            local_ns["carb"] = carb
            local_ns["Usd"] = Usd
            local_ns["UsdGeom"] = UsdGeom
            local_ns["Sdf"] = Sdf
            local_ns["Gf"] = Gf

            # Execute the code
            exec(code, local_ns)

            # Check for a result variable
            result = local_ns.get("result", None)

            return {
                "status": "success",
                "message": "Code executed successfully",
                "result": result
            }
        except Exception as e:
            carb.log_error(f"Error executing code: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc()
            }

    def _cmd_play_scene(self) -> Dict[str, Any]:
        """Start/resume the simulation timeline."""
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            return {
                "status": "success",
                "message": "Simulation started"
            }
        except Exception as e:
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to start simulation: {str(e)}"
            }

    def _cmd_stop_scene(self) -> Dict[str, Any]:
        """Stop the simulation timeline."""
        try:
            import omni.timeline
            timeline = omni.timeline.get_timeline_interface()
            timeline.stop()
            return {
                "status": "success",
                "message": "Simulation stopped"
            }
        except Exception as e:
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to stop simulation: {str(e)}"
            }

    # ==================== Scene Management Handlers ====================

    def _get_scene_state_path(self) -> str:
        """Get the path to scene_state.json file.

        If a path was previously set by MCP client, use that.
        Otherwise, use the default resources directory.
        """
        if self._scene_state_file_path:
            # Use the path set by MCP client
            parent_dir = os.path.dirname(self._scene_state_file_path)
            if parent_dir:
                os.makedirs(parent_dir, exist_ok=True)
            return self._scene_state_file_path

        # Use output dir pushed by MCP server, or fall back to default
        if self._output_dir:
            resources_dir = os.path.join(self._output_dir, "resources")
        else:
            resources_dir = RESOURCES_DIR
        os.makedirs(resources_dir, exist_ok=True)
        return os.path.join(resources_dir, "scene_state.json")

    def _get_prim_path(self, object_name: str) -> str:
        """Get the full prim path for an object name."""
        return f"/World/Objects/{object_name}/{object_name}/{object_name}"

    def _read_prim_pose(self, prim_path: str) -> Dict[str, Any]:
        """Helper method to read pose (position, quaternion, scale) from a prim."""
        try:
            from pxr import UsdGeom, Gf

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)

            if not prim.IsValid():
                return None

            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            scale_attr = prim.GetAttribute("xformOp:scale")

            position = {"x": 0.0, "y": 0.0, "z": 0.0}
            quaternion = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}  # Identity quaternion
            scale = {"x": 1.0, "y": 1.0, "z": 1.0}

            if translate_attr.IsValid():
                translate_value = translate_attr.Get()
                if translate_value:
                    position = {
                        "x": float(translate_value[0]),
                        "y": float(translate_value[1]),
                        "z": float(translate_value[2])
                    }

            if orient_attr.IsValid():
                orient_value = orient_attr.Get()
                if orient_value:
                    quaternion = {
                        "w": float(orient_value.GetReal()),
                        "x": float(orient_value.GetImaginary()[0]),
                        "y": float(orient_value.GetImaginary()[1]),
                        "z": float(orient_value.GetImaginary()[2])
                    }

            if scale_attr.IsValid():
                scale_value = scale_attr.Get()
                if scale_value:
                    scale = {
                        "x": float(scale_value[0]),
                        "y": float(scale_value[1]),
                        "z": float(scale_value[2])
                    }

            return {
                "position": position,
                "quaternion": quaternion,
                "scale": scale
            }
        except Exception as e:
            carb.log_error(f"Error reading pose for {prim_path}: {e}")
            return None

    def _write_prim_pose(self, prim_path: str, pose_data: Dict[str, Any]) -> bool:
        """Helper method to write pose (position, quaternion, scale) to a prim."""
        try:
            from pxr import Gf

            pos = pose_data.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
            quat = pose_data.get("quaternion", {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
            scale = pose_data.get("scale", {"x": 1.0, "y": 1.0, "z": 1.0})

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                carb.log_error(f"Prim not found: {prim_path}")
                return False

            # Set attributes directly instead of using ChangeProperty command
            # This avoids stale property reference issues in async context
            translate_attr = prim.GetAttribute("xformOp:translate")
            if translate_attr:
                translate_attr.Set(Gf.Vec3d(pos["x"], pos["y"], pos["z"]))

            orient_attr = prim.GetAttribute("xformOp:orient")
            if orient_attr:
                quat_gf = Gf.Quatf(quat["w"], quat["x"], quat["y"], quat["z"])
                orient_attr.Set(quat_gf)

            scale_attr = prim.GetAttribute("xformOp:scale")
            if scale_attr:
                scale_attr.Set(Gf.Vec3d(scale["x"], scale["y"], scale["z"]))

            return True
        except Exception as e:
            carb.log_error(f"Error writing pose for {prim_path}: {e}")
            traceback.print_exc()
            return False

    def _cmd_assemble_objects(self, assembly: str = "fmb1") -> Dict[str, Any]:
        """MCP handler for assembling objects."""
        try:
            if assembly not in ASSEMBLIES:
                return {
                    "status": "error",
                    "message": f"Invalid assembly: {assembly}. Must be one of: {', '.join(ASSEMBLIES.keys())}"
                }

            self._selected_assembly = assembly
            assembly_file = self._assembly_file_path
            if not os.path.exists(assembly_file):
                return {
                    "status": "error",
                    "message": f"Assembly file not found: {assembly_file}"
                }

            self.assemble_objects()
            return {
                "status": "success",
                "message": f"Objects assembled using {assembly}"
            }
        except Exception as e:
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to assemble objects: {str(e)}"
            }

    def _cmd_randomize_object_poses(self) -> Dict[str, Any]:
        """MCP handler for randomizing object poses."""
        try:
            self.randomize_object_poses()
            return {
                "status": "success",
                "message": "Object poses randomized successfully"
            }
        except Exception as e:
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to randomize object poses: {str(e)}"
            }

    def _cmd_save_scene_state(self, json_file_path: str = None, output_dir: str = None) -> Dict[str, Any]:
        """Save scene state (object poses) to a JSON file.

        Automatically discovers and saves all objects in /World/Objects.

        Args:
            json_file_path: Optional path to the JSON file. If not provided, uses default.
            output_dir: Optional output directory pushed by MCP server.

        Returns:
            Dictionary with execution result.
        """
        try:
            from pxr import UsdGeom

            # Update output dir if provided by MCP server
            if output_dir:
                self._output_dir = os.path.abspath(output_dir)

            # Store the path if provided by MCP client
            if json_file_path is not None:
                self._scene_state_file_path = json_file_path
            else:
                json_file_path = self._get_scene_state_path()

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {
                    "status": "error",
                    "message": "No stage is currently open"
                }

            objects_prim = stage.GetPrimAtPath("/World/Objects")
            if not objects_prim.IsValid():
                return {
                    "status": "error",
                    "message": "/World/Objects path does not exist"
                }

            children = objects_prim.GetChildren()
            object_names = [child.GetName() for child in children if child.IsA(UsdGeom.Xformable)]

            if not object_names:
                return {
                    "status": "error",
                    "message": "No objects found in /World/Objects"
                }

            print(f"[MCP] Auto-discovered {len(object_names)} object(s) in /World/Objects: {object_names}")

            resources_dir = os.path.dirname(json_file_path)
            os.makedirs(resources_dir, exist_ok=True)

            poses = {}
            saved_count = 0
            failed_names = []

            for object_name in object_names:
                prim_path = self._get_prim_path(object_name)
                pose = self._read_prim_pose(prim_path)
                if pose:
                    poses[object_name] = pose
                    saved_count += 1
                    print(f"[MCP] ✓ Saved pose for {object_name} ({prim_path})")
                else:
                    failed_names.append(object_name)
                    print(f"[MCP] ⚠ Failed to read pose for {object_name} ({prim_path})")

            with open(json_file_path, 'w') as f:
                json.dump(poses, f, indent=4)

            abs_path = os.path.abspath(json_file_path)
            message = f"Saved {saved_count} object pose(s) to {abs_path}"
            if failed_names:
                message += f". Failed to save {len(failed_names)} object(s): {failed_names}"

            return {
                "status": "success",
                "message": message,
                "saved_count": saved_count,
                "failed_names": failed_names,
                "json_file_path": json_file_path
            }

        except Exception as e:
            carb.log_error(f"Error in save_scene_state: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to save scene state: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def _cmd_restore_scene_state(self, json_file_path: str = None, output_dir: str = None) -> Dict[str, Any]:
        """Restore scene state (object poses) from a JSON file.

        Automatically restores all objects that were saved in the scene state file.

        Args:
            json_file_path: Optional path to the JSON file. If not provided, uses default.
            output_dir: Optional output directory pushed by MCP server.

        Returns:
            Dictionary with execution result.
        """
        try:
            # Update output dir if provided by MCP server
            if output_dir:
                self._output_dir = os.path.abspath(output_dir)

            # Store the path if provided by MCP client
            if json_file_path is not None:
                self._scene_state_file_path = json_file_path
            else:
                json_file_path = self._get_scene_state_path()

            if not os.path.exists(json_file_path):
                return {
                    "status": "error",
                    "message": f"JSON file not found: {json_file_path}"
                }

            try:
                with open(json_file_path, 'r') as f:
                    poses = json.load(f)
            except json.JSONDecodeError as e:
                return {
                    "status": "error",
                    "message": f"Invalid JSON file: {str(e)}"
                }

            if not poses:
                return {
                    "status": "error",
                    "message": "No objects found in scene state file"
                }

            restored_count = 0
            failed_names = []

            for object_name, pose_data in poses.items():
                prim_path = self._get_prim_path(object_name)
                if self._write_prim_pose(prim_path, pose_data):
                    restored_count += 1
                    print(f"[MCP] ✓ Restored pose for {object_name} ({prim_path})")
                else:
                    failed_names.append(object_name)
                    print(f"[MCP] ⚠ Failed to restore pose for {object_name} ({prim_path})")

            message = f"Restored {restored_count} object pose(s) from {json_file_path}"
            if failed_names:
                message += f". Failed to restore {len(failed_names)} object(s): {failed_names}"

            return {
                "status": "success",
                "message": message,
                "restored_count": restored_count,
                "failed_names": failed_names,
                "json_file_path": json_file_path
            }

        except Exception as e:
            carb.log_error(f"Error in restore_scene_state: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to restore scene state: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def _cmd_clear_scene_state(self, json_file_path: str = None, output_dir: str = None) -> Dict[str, Any]:
        """Clear/delete the scene state JSON file.

        Deletes the entire scene state file, removing all saved object poses.

        Args:
            json_file_path: Optional path to the JSON file. If not provided, uses default.
            output_dir: Optional output directory pushed by MCP server.

        Returns:
            Dictionary with execution result.
        """
        try:
            # Update output dir if provided by MCP server
            if output_dir:
                self._output_dir = os.path.abspath(output_dir)

            # Store the path if provided by MCP client
            if json_file_path is not None:
                self._scene_state_file_path = json_file_path
            else:
                json_file_path = self._get_scene_state_path()

            if os.path.exists(json_file_path):
                os.remove(json_file_path)
                return {
                    "status": "success",
                    "message": f"Deleted scene state file: {json_file_path}",
                    "json_file_path": json_file_path
                }
            else:
                return {
                    "status": "error",
                    "message": f"Scene state file not found: {json_file_path}"
                }

        except Exception as e:
            carb.log_error(f"Error in clear_scene_state: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to clear scene state: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def _cmd_add_objects(self, assembly: str = "fmb1") -> Dict[str, Any]:
        """MCP handler for adding objects to the scene.

        Args:
            assembly: Which assembly to load (fmb1, fmb2, fmb3)

        Returns:
            Dictionary with execution result.
        """
        try:
            if assembly not in ASSEMBLIES:
                return {
                    "status": "error",
                    "message": f"Invalid assembly: {assembly}. Must be one of: {', '.join(ASSEMBLIES.keys())}"
                }

            self._selected_assembly = assembly
            self.add_objects()

            return {
                "status": "success",
                "message": f"Objects added from {assembly}",
                "folder_path": self._objects_folder_path
            }
        except Exception as e:
            carb.log_error(f"Error in add_objects: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to add objects: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def _cmd_delete_objects(self) -> Dict[str, Any]:
        """MCP handler for deleting objects from the scene.

        Returns:
            Dictionary with execution result.
        """
        try:
            self.delete_objects()
            return {
                "status": "success",
                "message": "Objects deleted from /World/Objects"
            }
        except Exception as e:
            carb.log_error(f"Error in delete_objects: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to delete objects: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def _cmd_setup_pose_publisher(self) -> Dict[str, Any]:
        """MCP handler for setting up pose publisher action graph.

        Returns:
            Dictionary with execution result.
        """
        try:
            self.create_pose_publisher()
            return {
                "status": "success",
                "message": "Pose publisher action graph created at /World/Graphs/ActionGraph_objects_poses"
            }
        except Exception as e:
            carb.log_error(f"Error in setup_pose_publisher: {e}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to setup pose publisher: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def on_shutdown(self):
        """Clean shutdown"""
        print("[DigitalTwin] Digital Twin shutdown")

        # Stop MCP socket server
        self._stop_mcp_server()

        # Stop physics-rate callbacks
        self._stop_force_publish()
        self._stop_gripper_physics()

        # Isaac Sim handles ROS2 shutdown automatically
        print("ROS2 bridge shutdown handled by Isaac Sim")

        # Clear UI widget references to break closure references
        self._workspace_checkbox = None
        self._custom_checkbox = None
        self._custom_camera_prim_field = None
        self._custom_camera_topic_field = None
        self._resolution_combo = None
        self._assembly_combo = None

        # Destroy window - clear frame first to break lambda closure references
        if self._window:
            self._window.frame.clear()
            self._window.destroy()
            self._window = None