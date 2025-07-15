"""
MIT License

Copyright (c) 2023-2025 omni-mcp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""Extension module for Isaac Sim MCP."""

import asyncio
import carb
# import omni.ext
# import omni.ui as ui
import omni.usd
import threading
import time
import socket
import json
import traceback

import gc
from pxr import Usd, UsdGeom, Sdf, Gf

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
from typing import Dict, Any, List, Optional, Union
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
import numpy as np
from omni.isaac.core import World
# Import Beaver3d and USDLoader
from isaac_sim_mcp_extension.gen3d import Beaver3d
from isaac_sim_mcp_extension.usd import USDLoader
from isaac_sim_mcp_extension.usd import USDSearch3d
import requests

# Extension Methods required by Omniverse Kit
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MCPExtension(omni.ext.IExt):
    def __init__(self) -> None:
        """Initialize the extension."""
        super().__init__()
        self.ext_id = None
        self.running = False
        self.host = None
        self.port = None
        self.socket = None
        self.server_thread = None
        self._usd_context = None
        self._physx_interface = None
        self._timeline = None
        self._window = None
        self._status_label = None
        self._server_thread = None
        self._models = None
        self._settings = carb.settings.get_settings()
        self._image_url_cache = {} # cache for image url
        self._text_prompt_cache = {} # cache for text prompt
        

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        print("trigger  on_startup for: ", ext_id)
        print("settings: ", self._settings.get("/exts/omni.kit.pipapi"))
        self.port = self._settings.get("/exts/isaac.sim.mcp/server, port") or 8766
        self.host = self._settings.get("/exts/isaac.sim.mcp/server.host") or "localhost"
        if not hasattr(self, 'running'):
            self.running = False

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        # omni.kit.commands.execute("CreatePrim", prim_type="Sphere")

        # print("sphere created")
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)  
        self._start()
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)  
    
    def on_shutdown(self):
        print("trigger  on_shutdown for: ", self.ext_id)
        self._models = {}
        gc.collect()
        self._stop()
    
    def _start(self):
        if self.running:
            print("Server is already running")
            return
            
        self.running = True
        
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)
            
            # Start server thread
            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            print(f"Isaac Sim MCP server started on {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to start server: {str(e)}")
            self.stop()
            
    def _stop(self):
        self.running = False
        
        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        # Wait for thread to finish
        if self.server_thread:
            try:
                if self.server_thread.is_alive():
                    self.server_thread.join(timeout=1.0)
            except:
                pass
            self.server_thread = None
        
        print("Isaac Sim MCP server stopped")

    def _server_loop(self):
        """Main server loop in a separate thread"""
        print("Server thread started")
        self.socket.settimeout(1.0)  # Timeout to allow for stopping
        if not hasattr(self, 'running'):
            self.running = False

        while self.running:
            try:
                # Accept new connection
                try:
                    client, address = self.socket.accept()
                    print(f"Connected to client: {address}")
                    
                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client,)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    # Just check running condition
                    continue
                except Exception as e:
                    print(f"Error accepting connection: {str(e)}")
                    time.sleep(0.5)
            except Exception as e:
                print(f"Error in server loop: {str(e)}")
                if not self.running:
                    break
                time.sleep(0.5)
        
        print("Server thread stopped")
    
    def _handle_client(self, client):
        """Handle connected client"""
        print("Client handler started")
        client.settimeout(None)  # No timeout
        buffer = b''
        
        try:
            while self.running:
                # Receive data
                try:
                    data = client.recv(16384)
                    if not data:
                        print("Client disconnected")
                        break
                    
                    buffer += data
                    try:
                        # Try to parse command
                        command = json.loads(buffer.decode('utf-8'))
                        buffer = b''
                        
                        # Execute command in Isaac Sim's main thread
                        async def execute_wrapper():
                            try:
                                response = self.execute_command(command)
                                response_json = json.dumps(response)
                                print("response_json: ", response_json)
                                try:
                                    client.sendall(response_json.encode('utf-8'))
                                except:
                                    print("Failed to send response - client disconnected")
                            except Exception as e:
                                print(f"Error executing command: {str(e)}")
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
                        # import omni.kit.commands
                        # import omni.kit.async
                        from omni.kit.async_engine import run_coroutine
                        task = run_coroutine(execute_wrapper())
                        # import asyncio
                        # asyncio.ensure_future(execute_wrapper())
                        #time.sleep(30)
                        
    
                        # 
                        # omni.kit.async.get_event_loop().create_task(create_sphere_async())
                        # TODO:Schedule execution in main thread
                        # bpy.app.timers.register(execute_wrapper, first_interval=0.0)
                        # omni.kit.app.get_app().post_to_main_thread(execute_wrapper())
                        # carb.apputils.get_app().get_update_event_loop().post(execute_wrapper)

                        # from omni.kit.async_engine import run_coroutine
                        # run_coroutine(execute_wrapper())
                        # omni.kit.app.get_app().get_update_event_stream().push(0, 0, {"fn": execute_wrapper})
                    except json.JSONDecodeError:
                        # Incomplete data, wait for more
                        pass
                except Exception as e:
                    print(f"Error receiving data: {str(e)}")
                    break
        except Exception as e:
            print(f"Error in client handler: {str(e)}")
        finally:
            try:
                client.close()
            except:
                pass
            print("Client handler stopped")

    # TODO: This is a temporary function to execute commands in the main thread
    def execute_command(self, command):
        """Execute a command in the main thread"""
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})
            
            # TODO: Ensure we're in the right context
            if cmd_type in ["create_object", "modify_object", "delete_object"]:
                self._usd_context = omni.usd.get_context()
                self._execute_command_internal(command)
            else:
                return self._execute_command_internal(command)
                
        except Exception as e:
            print(f"Error executing command: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _execute_command_internal(self, command):
        """Internal command execution with proper context"""
        cmd_type = command.get("type")
        params = command.get("params", {})

        #todo: add a handler for extend simulation method if necessary
        handlers = {
            "execute_script": self.execute_script,
            "list_prims": self.list_prims,
            "open_usd": self.open_usd,
            "import_usd": self.import_usd,
            "load_scene": self.load_scene,
            "get_object_info": self.get_object_info,
            "move_prim": self.move_prim,
            "control_gripper": self.control_gripper,  
            "perform_ik": self.perform_ik,
            "execute_joint_trajectory": self.execute_joint_trajectory,
            "get_ee_pose": self.get_ee_pose,
        }
        
        handler = handlers.get(cmd_type)
        if handler:
            try:
                print(f"Executing handler for {cmd_type}")
                result = handler(**params)
                print(f"Handler execution complete: /n", result)
                # return result
                if result and result.get("status") == "success":   
                    return {"status": "success", "result": result}
                else:
                    return {"status": "error", "message": result.get("message", "Unknown error")}
            except Exception as e:
                print(f"Error in handler: {str(e)}")
                traceback.print_exc()
                return {"status": "error", "message": str(e)}
        else:
            return {"status": "error", "message": f"Unknown command type: {cmd_type}"}
        

    def execute_script(self, code: str) :
        """Execute a Python script within the Isaac Sim context.
        
        Args:
            code: The Python script to execute.
            
        Returns:
            Dictionary with execution result.
        """
        try:
            # Create a local namespace
            local_ns = {}
            
            # Add frequently used modules to the namespace
            local_ns["omni"] = omni
            local_ns["carb"] = carb
            local_ns["Usd"] = Usd
            local_ns["UsdGeom"] = UsdGeom
            local_ns["Sdf"] = Sdf
            local_ns["Gf"] = Gf
            # code = script["code"]
            
            # Execute the script
            exec(code,  local_ns)
            
            # Get the result if any
            # result = local_ns.get("result", None)
            result = None
            
            
            return {
                "status": "success",
                "message": "Script executed successfully",
                "result": result
            }
        except Exception as e:
            carb.log_error(f"Error executing script: {e}")
            import traceback
            carb.log_error(traceback.format_exc())
            return {
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc()
            }
        
    def get_scene_info(self):
        self._stage = omni.usd.get_context().get_stage()
        assert self._stage is not None
        stage_path = self._stage.GetRootLayer().realPath
        assets_root_path = get_assets_root_path()
        return {"status": "success", "message": "pong", "assets_root_path": assets_root_path}

    def list_prims(self) -> Dict[str, Any]:
        """List all prim paths in the current USD scene."""
        try:
            import omni.usd
            
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {
                    "status": "error",
                    "message": "No active stage found"
                }
            
            # Collect all prims
            prims_list = []
            for prim in stage.Traverse():
                prim_info = {
                    "path": str(prim.GetPath()),
                    "name": prim.GetName(),
                    "type": prim.GetTypeName()
                }
                prims_list.append(prim_info)
            
            return {
                "status": "success",
                "message": f"Found {len(prims_list)} prims in scene",
                "prims": prims_list,
                "total_count": len(prims_list)
            }
            
        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to list prims: {str(e)}"
            }

    def open_usd(self, usd_path: str) -> Dict[str, Any]:
        """Open a USD file as the main stage in Isaac Sim."""
        try:
            from omni.isaac.core.utils.stage import open_stage
            
            # Open the USD file as the main stage
            open_stage(usd_path=usd_path)
            print(f"Opened USD stage: {usd_path}")
            
            return {
                "status": "success",
                "message": f"Successfully opened USD stage: {usd_path}",
                "usd_path": usd_path
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to open USD stage: {str(e)}",
                "usd_path": usd_path,
                "traceback": traceback.format_exc()
            }
            
    def import_usd(self, usd_path: str, prim_path: str = None, position: List[float] = None, orientation: List[float] = None, orientation_format: str = "degrees") -> Dict[str, Any]:
        """Import a USD file as a prim into the Isaac Sim stage with flexible orientation input."""
        try:
            import os
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from pxr import Usd, UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
            import omni.usd
            
            # Auto-generate prim path from filename if not provided
            if prim_path is None:
                filename = os.path.splitext(os.path.basename(usd_path))[0]
                prim_path = f"/World/{filename}"
            
            # Import USD file
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            print(f"Prim imported at {prim_path}")
            
            # Apply positioning if specified, otherwise default to [0, 0, 0]
            if position is None:
                position = [0.0, 0.0, 0.0]
            
            # Apply orientation if specified, otherwise default to [0, 0, 0]
            if orientation is None:
                orientation = [0.0, 0.0, 0.0]
                orientation_format = "degrees"
            
            # Apply transformation
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                raise RuntimeError(f"Prim at {prim_path} not found after import.")
            
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            
            # Translation
            position_vec = Gf.Vec3d(position[0], position[1], position[2])
            xform.AddTranslateOp().Set(position_vec)
            
            # Handle different orientation input formats
            if orientation_format.lower() == "quaternion" and len(orientation) == 4:
                # Input is already quaternion [w, x, y, z] or [x, y, z, w]
                # Assume [w, x, y, z] format (scalar first)
                quat = Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3])
                print(f"Applied quaternion orientation: {orientation}")
            
            elif orientation_format.lower() == "radians" and len(orientation) == 3:
                # Input is RPY in radians
                rpy_rad = np.array(orientation)
                quat_xyzw = euler_angles_to_quats(rpy_rad)
                quat = Gf.Quatd(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])
                print(f"Applied radian orientation {orientation} rad")
            
            else:
                # Default: Input is RPY in degrees
                rpy_deg = np.array(orientation)
                rpy_rad = np.deg2rad(rpy_deg)
                quat_xyzw = euler_angles_to_quats(rpy_rad)
                quat = Gf.Quatd(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])
                print(f"Applied degree orientation {orientation} degrees")
            
            # Add orient op with matching precision
            orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            orient_op.Set(quat)
            
            print(f"Applied translation {position} and rotation with double-precision quaternion.")
            
            return {
                "status": "success",
                "message": f"Successfully imported USD file at {prim_path} with position {position} and orientation {orientation} ({orientation_format})",
                "usd_path": usd_path,
                "prim_path": prim_path,
                "position": position,
                "orientation": orientation,
                "orientation_format": orientation_format
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to import USD file: {str(e)}",
                "usd_path": usd_path,
                "prim_path": prim_path,
                "traceback": traceback.format_exc()
            }

    def load_scene(self) -> Dict[str, Any]:
        """Load a basic scene with world and ground plane."""
        try:
            from omni.isaac.core import World
            from omni.kit.async_engine import run_coroutine
            
            async def load_scene_async():
                world = World()
                await world.initialize_simulation_context_async()
                world.scene.add_default_ground_plane()
                print("Scene loaded successfully.")
                return world
            
            # Run the async function
            world = run_coroutine(load_scene_async())
            
            return {
                "status": "success",
                "message": "Scene loaded successfully with world and ground plane"
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to load scene: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def get_object_info(self, prim_path: str) -> Dict[str, Any]:
        """Get comprehensive object information including pose using direct USD attribute access."""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import quats_to_euler_angles
            
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at {prim_path} not found"
                }
            
            # Basic prim info
            prim_info = {
                "path": str(prim.GetPath()),
                "name": prim.GetName(),
                "type": prim.GetTypeName(),
                "is_valid": prim.IsValid(),
                "is_active": prim.IsActive(),
                "has_children": bool(prim.GetChildren())
            }
            
            # Direct attribute access using your code
            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            scale_attr = prim.GetAttribute("xformOp:scale")
            
            # Get values
            position = [0.0, 0.0, 0.0]
            quaternion = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
            scale = [1.0, 1.0, 1.0]
            
            if translate_attr.IsValid():
                translate_value = translate_attr.Get()
                if translate_value:
                    position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]
            
            if orient_attr.IsValid():
                orient_value = orient_attr.Get()
                if orient_value:
                    quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                                float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]
            
            if scale_attr.IsValid():
                scale_value = scale_attr.Get()
                if scale_value:
                    scale = [float(scale_value[0]), float(scale_value[1]), float(scale_value[2])]
            
            # Convert quaternion to euler angles using your code
            if abs(quaternion[0] - 1.0) < 1e-6 and abs(quaternion[1]) < 1e-6 and abs(quaternion[2]) < 1e-6 and abs(quaternion[3]) < 1e-6:
                # Identity quaternion - no rotation
                rotation_radians = [0.0, 0.0, 0.0]
                rotation_degrees = [0.0, 0.0, 0.0]
            else:
                # Convert w,x,y,z to x,y,z,w format for Isaac function
                quat_xyzw = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
                rpy_rad = quats_to_euler_angles(quat_xyzw.reshape(1, 4))[0]
                rpy_deg = np.rad2deg(rpy_rad)
                rotation_radians = [float(rpy_rad[0]), float(rpy_rad[1]), float(rpy_rad[2])]
                rotation_degrees = [float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])]
            
            return {
                "status": "success",
                "message": f"Successfully retrieved info for {prim_path}",
                "prim_info": prim_info,
                "position": position,
                "rotation_quaternion": quaternion,  # [w, x, y, z]
                "rotation_radians": rotation_radians,  # [roll, pitch, yaw]
                "rotation_degrees": rotation_degrees,  # [roll, pitch, yaw]
                "scale": scale
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to get object info: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def move_prim(self, prim_path: str, position: List[float] = None, orientation: List[float] = None, 
                orientation_format: str = "degrees") -> Dict[str, Any]:
        """Move a prim to a new pose with flexible orientation input (degrees/radians/quaternions)."""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
            
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at {prim_path} not found"
                }
            
            # Read current pose using direct attribute access (same as your working code)
            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            
            # Get current values as defaults
            current_position = [0.0, 0.0, 0.0]
            current_quaternion = [1.0, 0.0, 0.0, 0.0]
            
            if translate_attr.IsValid():
                translate_value = translate_attr.Get()
                if translate_value:
                    current_position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]
            
            if orient_attr.IsValid():
                orient_value = orient_attr.Get()
                if orient_value:
                    current_quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                                        float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]
            
            # Use provided values or keep current ones
            final_position = position if position is not None else current_position
            
            # Handle orientation conversion (same logic as working script)
            if orientation is not None:
                if orientation_format.lower() == "quaternion" and len(orientation) == 4:
                    # Input is quaternion [w, x, y, z]
                    final_quaternion = orientation
                    print(f"Using quaternion orientation: {orientation}")
                elif orientation_format.lower() == "radians" and len(orientation) == 3:
                    # Input is RPY in radians
                    rpy_rad = np.array(orientation)
                    quat_xyzw = euler_angles_to_quats(rpy_rad)
                    final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                    print(f"Converted radian orientation {orientation} to quaternion {final_quaternion}")
                else:
                    # Default: Input is RPY in degrees
                    rpy_deg = np.array(orientation)
                    rpy_rad = np.deg2rad(rpy_deg)
                    quat_xyzw = euler_angles_to_quats(rpy_rad)
                    final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                    print(f"Converted degree orientation {orientation} to quaternion {final_quaternion}")
            else:
                # Keep current orientation
                final_quaternion = current_quaternion
                orientation = "unchanged"
            
            # Apply transformations (detect existing precision and match it - same as working script)
            if not prim.IsA(UsdGeom.Xformable):
                return {
                    "status": "error",
                    "message": f"Prim {prim_path} is not transformable"
                }
            
            xform = UsdGeom.Xform(prim)
            
            # Check existing orient operation precision BEFORE clearing
            existing_orient_precision = None
            existing_ops = xform.GetOrderedXformOps()
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    # Get the precision from existing operation
                    if "quatd" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionDouble
                    elif "quatf" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionFloat
                    break
            
            # Clear and recreate with matching precision
            xform.ClearXformOpOrder()
            
            # Apply translation (always double precision for position)
            position_vec = Gf.Vec3d(final_position[0], final_position[1], final_position[2])
            xform.AddTranslateOp().Set(position_vec)
            
            # Apply rotation with matching precision
            if existing_orient_precision == UsdGeom.XformOp.PrecisionFloat:
                quat = Gf.Quatf(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
                print("Using float precision for quaternion")
            else:
                quat = Gf.Quatd(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
                print("Using double precision for quaternion")
            
            orient_op.Set(quat)
            
            print(f"Moved {prim_path} to position {final_position}, orientation {orientation}")
            
            return {
                "status": "success",
                "message": f"Successfully moved {prim_path} to new pose",
                "prim_path": prim_path,
                "previous_position": current_position,
                "previous_quaternion": current_quaternion,
                "new_position": final_position,
                "new_quaternion": final_quaternion,
                "orientation_format": orientation_format
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to move prim: {str(e)}",
                "prim_path": prim_path,
                "traceback": traceback.format_exc()
            }


    def control_gripper(self, command: Union[str, int, float]) -> Dict[str, Any]:
        """
        Control gripper with flexible input - string commands or numeric values.
        
        Args:
            command: Can be:
                    - String: "open", "close" 
                    - Numeric: 0-1100 (0=closed/0cm, 1100=open/11cm)
                    
        Returns:
            Dictionary with execution result.
        """
        try:
            import subprocess
            import os
            
            # Convert command to appropriate ROS2 message
            if isinstance(command, str):
                if command.lower() == "open":
                    ros_command = "open"
                    numeric_value = 1100
                    width_cm = 11.0
                elif command.lower() == "close":
                    ros_command = "close" 
                    numeric_value = 0
                    width_cm = 0.0
                else:
                    return {
                        "status": "error",
                        "message": f"Invalid string command '{command}'. Use 'open' or 'close'."
                    }
            else:
                # Numeric input
                try:
                    numeric_value = float(command)
                    if not (0 <= numeric_value <= 1100):
                        return {
                            "status": "error",
                            "message": f"Numeric value {numeric_value} out of range. Use 0-1100."
                        }
                    
                    # Convert to ROS command string and width
                    ros_command = str(int(numeric_value))
                    width_cm = numeric_value / 100.0  # Convert to cm (1100 -> 11cm)
                    
                except (ValueError, TypeError):
                    return {
                        "status": "error",
                        "message": f"Invalid numeric command '{command}'. Use number 0-1100."
                    }
            
            # Construct and execute ROS2 command
            ros2_cmd = [
                'ros2', 'topic', 'pub', '--once',
                '/gripper_command',
                'std_msgs/String', 
                f"{{data: '{ros_command}'}}"
            ]
            
            try:
                result = subprocess.run(ros2_cmd, 
                                    capture_output=True, 
                                    text=True, 
                                    timeout=5)
                
                if result.returncode == 0:
                    return {
                        "status": "success",
                        "message": f"Gripper command sent successfully",
                        "command_sent": ros_command,
                        "numeric_value": numeric_value,
                        "width_cm": width_cm,
                        "ros_output": result.stdout.strip() if result.stdout else None
                    }
                else:
                    return {
                        "status": "error", 
                        "message": f"ROS2 command failed: {result.stderr.strip()}",
                        "command_attempted": ros_command
                    }
                    
            except subprocess.TimeoutExpired:
                return {
                    "status": "error",
                    "message": "ROS2 command timed out (5 seconds)"
                }
            except FileNotFoundError:
                return {
                    "status": "error", 
                    "message": "ros2 command not found. Make sure ROS2 is properly installed and sourced."
                }
                
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Unexpected error in gripper control: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def perform_ik(self, robot_prim_path: str, target_position: List[float], target_rpy: List[float], 
                duration: float = 3.0, custom_lib_path: str = "/home/aaugus11/Desktop/ros2_ws/src/ur_asu-main/ur_asu/custom_libraries") -> Dict[str, Any]:
        """
        Perform inverse kinematics on the robot and execute smooth trajectory movement.
        
        Args:
            robot_prim_path: Path to the robot prim (e.g., "/World/UR5e")
            target_position: [x, y, z] target position in meters
            target_rpy: [roll, pitch, yaw] target orientation in degrees
            duration: Time to complete the movement in seconds (default: 3.0)
            custom_lib_path: Path to your custom IK solver library
            
        Returns:
            Dictionary with execution result including joint angles and trajectory execution status.
        """
        try:
            import sys
            import numpy as np
            from omni.isaac.core.articulations import Articulation
            import omni.usd
            
            # Add custom libraries to Python path if not already there
            if custom_lib_path not in sys.path:
                sys.path.append(custom_lib_path)
                print(f"Added {custom_lib_path} to Python path")
            
            # Import the IK solver
            try:
                from ik_solver import compute_ik
                print("Successfully imported compute_ik from ik_solver")
            except ImportError as e:
                return {
                    "status": "error",
                    "message": f"Failed to import ik_solver: {str(e)}. Check if {custom_lib_path}/ik_solver.py exists."
                }
            
            # Check if robot exists in scene
            stage = omni.usd.get_context().get_stage()
            robot_prim = stage.GetPrimAtPath(robot_prim_path)
            if not robot_prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Robot prim at {robot_prim_path} not found. Use list_prims() to see available objects."
                }
            
            # Load and initialize robot articulation
            try:
                robot = Articulation(robot_prim_path)
                robot.initialize()
                print(f"Robot articulation initialized at {robot_prim_path}")
            except Exception as e:
                return {
                    "status": "error", 
                    "message": f"Failed to initialize robot articulation: {str(e)}"
                }
            
            # Get current joint positions for reference
            try:
                current_joints = robot.get_joint_positions()
                current_joints_deg = np.degrees(current_joints) if current_joints is not None else None
                print(f"Current joint positions (rad): {current_joints}")
            except Exception as e:
                print(f"Warning: Could not get current joint positions: {str(e)}")
                current_joints = None
                current_joints_deg = None
            
            # Solve IK
            print(f"Computing IK for position: {target_position}, RPY: {target_rpy}")
            joint_angles = compute_ik(position=target_position, rpy=target_rpy)
            
            if joint_angles is not None:
                print(f"IK solution found: {joint_angles} (rad)")
                joint_angles_deg = np.degrees(joint_angles)
                print(f"IK solution (deg): {joint_angles_deg}")
                
                # Execute joint trajectory instead of direct setting
                try:
                    trajectory_result = self.execute_joint_trajectory(joint_angles.tolist(), duration)
                    
                    if trajectory_result.get("status") == "success":
                        print("Joint trajectory executed successfully")
                        
                        return {
                            "status": "success",
                            "message": "IK solved and trajectory executed successfully",
                            "robot_prim_path": robot_prim_path,
                            "target_position": target_position,
                            "target_rpy": target_rpy,
                            "joint_angles_rad": joint_angles.tolist(),
                            "joint_angles_deg": joint_angles_deg.tolist(),
                            "previous_joints_rad": current_joints.tolist() if current_joints is not None else None,
                            "previous_joints_deg": current_joints_deg.tolist() if current_joints_deg is not None else None,
                            "duration": duration,
                            "trajectory_status": "executed",
                            "ros_output": trajectory_result.get("ros_output")
                        }
                    else:
                        # IK succeeded but trajectory failed - still return the joint angles
                        return {
                            "status": "partial_success",
                            "message": f"IK solved but trajectory execution failed: {trajectory_result.get('message', 'Unknown trajectory error')}",
                            "robot_prim_path": robot_prim_path,
                            "target_position": target_position,
                            "target_rpy": target_rpy,
                            "joint_angles_rad": joint_angles.tolist(),
                            "joint_angles_deg": joint_angles_deg.tolist(),
                            "previous_joints_rad": current_joints.tolist() if current_joints is not None else None,
                            "previous_joints_deg": current_joints_deg.tolist() if current_joints_deg is not None else None,
                            "duration": duration,
                            "trajectory_status": "failed",
                            "trajectory_error": trajectory_result.get("message", "Unknown error")
                        }
                        
                except Exception as e:
                    return {
                        "status": "partial_success",
                        "message": f"IK solved but trajectory execution error: {str(e)}",
                        "joint_angles_rad": joint_angles.tolist(),
                        "joint_angles_deg": joint_angles_deg.tolist(),
                        "previous_joints_rad": current_joints.tolist() if current_joints is not None else None,
                        "previous_joints_deg": current_joints_deg.tolist() if current_joints_deg is not None else None,
                        "duration": duration,
                        "trajectory_status": "error",
                        "trajectory_error": str(e)
                    }
            else:
                return {
                    "status": "error",
                    "message": "IK solver failed to find a solution for the given target pose",
                    "robot_prim_path": robot_prim_path,
                    "target_position": target_position,
                    "target_rpy": target_rpy,
                    "current_joints_rad": current_joints.tolist() if current_joints is not None else None,
                    "duration": duration
                }
                
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Unexpected error in IK computation: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def execute_joint_trajectory(self, joint_angles: List[float], duration: float = 3.0, 
                                joint_names: List[str] = None) -> Dict[str, Any]:
        """
        Execute joint trajectory using the simplest possible method (like gripper).
        
        Args:
            joint_angles: Target joint angles in radians [j1, j2, j3, j4, j5, j6]
            duration: Time to complete the movement in seconds (default: 3.0)
            joint_names: Custom joint names (optional, uses UR5e defaults)
            
        Returns:
            Dictionary with execution result.
        """
        try:
            import subprocess
            
            # Validate inputs
            if len(joint_angles) != 6:
                return {
                    "status": "error",
                    "message": f"Expected 6 joint angles, got {len(joint_angles)}"
                }
            
            # Format joint angles as simple comma-separated string (exactly like gripper)
            joint_command = ",".join([f"{angle:.6f}" for angle in joint_angles])
            
            # Send to a topic using std_msgs/String (same as gripper command)
            ros2_cmd = [
                'ros2', 'topic', 'pub', '--once',
                '/robot_joint_command',  # Simple topic name
                'std_msgs/String',       # Same message type as gripper
                f"{{data: '{joint_command}'}}"
            ]
            
            try:
                result = subprocess.run(ros2_cmd, 
                                    capture_output=True, 
                                    text=True, 
                                    timeout=5)
                
                if result.returncode == 0:
                    return {
                        "status": "success",
                        "message": "Joint command sent successfully",
                        "joint_angles": joint_angles,
                        "duration": duration,
                        "command_sent": joint_command,
                        "ros_output": result.stdout.strip() if result.stdout else None
                    }
                else:
                    return {
                        "status": "error",
                        "message": f"ROS2 command failed: {result.stderr.strip()}",
                        "joint_angles": joint_angles,
                        "command_attempted": joint_command,
                        "ros_output": result.stdout.strip() if result.stdout else None
                    }
                    
            except subprocess.TimeoutExpired:
                return {
                    "status": "error",
                    "message": "ROS2 command timed out (5 seconds)"
                }
            except FileNotFoundError:
                return {
                    "status": "error", 
                    "message": "ros2 command not found. Make sure ROS2 is properly installed and sourced."
                }
                
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Unexpected error in joint command: {str(e)}",
                "traceback": traceback.format_exc()
            }
                            
    def get_ee_pose(self, robot_prim_path: str, joint_angles: List[float] = None,
                    custom_lib_path: str = "/home/aaugus11/Desktop/ros2_ws/src/ur_asu-main/ur_asu/custom_libraries") -> Dict[str, Any]:
        """
        Get end-effector pose using forward kinematics from current or specified joint angles.
        
        Args:
            robot_prim_path: Path to the robot prim (e.g., "/World/UR5e")
            joint_angles: Optional joint angles in radians. If None, uses current robot joint positions
            custom_lib_path: Path to your custom IK solver library
            
        Returns:
            Dictionary with end-effector position, orientation, and joint angles used.
        """
        try:
            import sys
            import numpy as np
            from omni.isaac.core.articulations import Articulation
            from scipy.spatial.transform import Rotation as R
            import omni.usd
            
            # Add custom libraries to Python path if not already there
            if custom_lib_path not in sys.path:
                sys.path.append(custom_lib_path)
                print(f"Added {custom_lib_path} to Python path")
            
            # Import the IK solver module (which contains forward_kinematics)
            try:
                from ik_solver import forward_kinematics, dh_params
                print("Successfully imported forward_kinematics and dh_params from ik_solver")
            except ImportError as e:
                return {
                    "status": "error",
                    "message": f"Failed to import from ik_solver: {str(e)}. Check if {custom_lib_path}/ik_solver.py exists."
                }
            
            # Check if robot exists in scene
            stage = omni.usd.get_context().get_stage()
            robot_prim = stage.GetPrimAtPath(robot_prim_path)
            if not robot_prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Robot prim at {robot_prim_path} not found. Use list_prims() to see available objects."
                }
            
            # Get joint angles (either provided or current)
            if joint_angles is None:
                # Load and initialize robot articulation to get current joint positions
                try:
                    robot = Articulation(robot_prim_path)
                    robot.initialize()
                    current_joints = robot.get_joint_positions()
                    
                    if current_joints is None:
                        return {
                            "status": "error",
                            "message": "Failed to get current joint positions from robot"
                        }
                    
                    joint_angles = current_joints
                    source = "current_robot_state"
                    print(f"Using current robot joint positions: {joint_angles}")
                    
                except Exception as e:
                    return {
                        "status": "error", 
                        "message": f"Failed to get current joint positions: {str(e)}"
                    }
            else:
                # Use provided joint angles
                joint_angles = np.array(joint_angles)
                source = "provided_joint_angles"
                print(f"Using provided joint angles: {joint_angles}")
            
            # Compute forward kinematics using your existing function
            try:
                T_ee = forward_kinematics(dh_params, joint_angles)
                print(f"Forward kinematics computed successfully")
                
                # Extract position (translation part)
                ee_position = T_ee[:3, 3]
                
                # Extract rotation matrix and convert to RPY
                ee_rotation_matrix = T_ee[:3, :3]
                
                # Convert rotation matrix to RPY (roll, pitch, yaw) in degrees
                rotation = R.from_matrix(ee_rotation_matrix)
                ee_rpy_rad = rotation.as_euler('xyz', degrees=False)
                ee_rpy_deg = rotation.as_euler('xyz', degrees=True)
                
                # Convert to quaternion as well
                ee_quaternion = rotation.as_quat()  # [x, y, z, w] format
                
                # Convert joint angles to degrees for display
                joint_angles_deg = np.degrees(joint_angles)
                
                return {
                    "status": "success",
                    "message": "Forward kinematics computed successfully",
                    "robot_prim_path": robot_prim_path,
                    "joint_angles_source": source,
                    "joint_angles_rad": joint_angles.tolist(),
                    "joint_angles_deg": joint_angles_deg.tolist(),
                    "ee_position": ee_position.tolist(),  # [x, y, z] in meters
                    "ee_rpy_rad": ee_rpy_rad.tolist(),   # [roll, pitch, yaw] in radians
                    "ee_rpy_deg": ee_rpy_deg.tolist(),   # [roll, pitch, yaw] in degrees
                    "ee_quaternion_xyzw": ee_quaternion.tolist(),  # [x, y, z, w]
                    "transformation_matrix": T_ee.tolist()  # Full 4x4 transformation matrix
                }
                
            except Exception as e:
                return {
                    "status": "error",
                    "message": f"Failed to compute forward kinematics: {str(e)}",
                    "joint_angles_rad": joint_angles.tolist(),
                    "joint_angles_deg": np.degrees(joint_angles).tolist()
                }
                
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Unexpected error in forward kinematics computation: {str(e)}",
                "traceback": traceback.format_exc()
            }