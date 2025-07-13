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

# isaac_sim_mcp_server.py
import time
from mcp.server.fastmcp import FastMCP, Context, Image
import socket
import json
import asyncio
import logging
from dataclasses import dataclass
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any, List
import os
from pathlib import Path
import base64
from urllib.parse import urlparse

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("IsaacMCPServer")

@dataclass
class IsaacConnection:
    host: str
    port: int
    sock: socket.socket = None  # Changed from 'socket' to 'sock' to avoid naming conflict
    
    def connect(self) -> bool:
        """Connect to the Isaac addon socket server"""
        if self.sock:
            return True
            
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            logger.info(f"Connected to Isaac at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Isaac: {str(e)}")
            self.sock = None
            return False
    
    def disconnect(self):
        """Disconnect from the Isaac addon"""
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                logger.error(f"Error disconnecting from Isaac: {str(e)}")
            finally:
                self.sock = None

    def receive_full_response(self, sock, buffer_size=16384):
        """Receive the complete response, potentially in multiple chunks"""
        chunks = []
        # Use a consistent timeout value that matches the addon's timeout
        sock.settimeout(300.0)  # Match the extension's timeout
        
        try:
            while True:
                try:
                    logger.info("Waiting for data from Isaac")
                    #time.sleep(0.5)
                    chunk = sock.recv(buffer_size)
                    if not chunk:
                        # If we get an empty chunk, the connection might be closed
                        if not chunks:  # If we haven't received anything yet, this is an error
                            raise Exception("Connection closed before receiving any data")
                        break
                    
                    chunks.append(chunk)
                    
                    # Check if we've received a complete JSON object
                    try:
                        data = b''.join(chunks)
                        json.loads(data.decode('utf-8'))
                        # If we get here, it parsed successfully
                        logger.info(f"Received complete response ({len(data)} bytes)")
                        return data
                    except json.JSONDecodeError:
                        # Incomplete JSON, continue receiving
                        continue
                except socket.timeout:
                    # If we hit a timeout during receiving, break the loop and try to use what we have
                    logger.warning("Socket timeout during chunked receive")
                    break
                except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
                    logger.error(f"Socket connection error during receive: {str(e)}")
                    raise  # Re-raise to be handled by the caller
        except socket.timeout:
            logger.warning("Socket timeout during chunked receive")
        except Exception as e:
            logger.error(f"Error during receive: {str(e)}")
            raise
            
        # If we get here, we either timed out or broke out of the loop
        # Try to use what we have
        if chunks:
            data = b''.join(chunks)
            logger.info(f"Returning data after receive completion ({len(data)} bytes)")
            try:
                # Try to parse what we have
                json.loads(data.decode('utf-8'))
                return data
            except json.JSONDecodeError:
                # If we can't parse it, it's incomplete
                raise Exception("Incomplete JSON response received")
        else:
            raise Exception("No data received")

    def send_command(self, command_type: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """Send a command to Isaac and return the response"""
        if not self.sock and not self.connect():
            raise ConnectionError("Not connected to Isaac")
        
        command = {
            "type": command_type,
            "params": params or {}
        }
        
        try:
            # Log the command being sent
            logger.info(f"Sending command: {command_type} with params: {params}")
            
            # Send the command
            self.sock.sendall(json.dumps(command).encode('utf-8'))
            logger.info(f"Command sent, waiting for response...")
            
            # Set a timeout for receiving - use the same timeout as in receive_full_response
            self.sock.settimeout(300.0)  # Match the extension's timeout
            
            # Receive the response using the improved receive_full_response method
            response_data = self.receive_full_response(self.sock)
            logger.info(f"Received {len(response_data)} bytes of data")
            
            response = json.loads(response_data.decode('utf-8'))
            logger.info(f"Response parsed, status: {response.get('status', 'unknown')}")
            
            if response.get("status") == "error":
                logger.error(f"Isaac error: {response.get('message')}")
                raise Exception(response.get("message", "Unknown error from Isaac"))
            
            return response.get("result", {})
        except socket.timeout:
            logger.error("Socket timeout while waiting for response from Isaac")
            # Don't try to reconnect here - let the get_isaac_connection handle reconnection
            # Just invalidate the current socket so it will be recreated next time
            self.sock = None
            raise Exception("Timeout waiting for Isaac response - try simplifying your request")
        except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
            logger.error(f"Socket connection error: {str(e)}")
            self.sock = None
            raise Exception(f"Connection to Isaac lost: {str(e)}")
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON response from Isaac: {str(e)}")
            # Try to log what was received
            if 'response_data' in locals() and response_data:
                logger.error(f"Raw response (first 200 bytes): {response_data[:200]}")
            raise Exception(f"Invalid response from Isaac: {str(e)}")
        except Exception as e:
            logger.error(f"Error communicating with Isaac: {str(e)}")
            # Don't try to reconnect here - let the get_isaac_connection handle reconnection
            self.sock = None
            raise Exception(f"Communication error with Isaac: {str(e)}")

@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    """Manage server startup and shutdown lifecycle"""
    # We don't need to create a connection here since we're using the global connection
    # for resources and tools
    
    try:
        # Just log that we're starting up
        logger.info("IsaacMCP server starting up")
        
        # Try to connect to Isaac on startup to verify it's available
        try:
            # This will initialize the global connection if needed
            isaac = get_isaac_connection()
            logger.info("Successfully connected to Isaac on startup")
        except Exception as e:
            logger.warning(f"Could not connect to Isaac on startup: {str(e)}")
            logger.warning("Make sure the Isaac addon is running before using Isaac resources or tools")
        
        # Return an empty context - we're using the global connection
        yield {}
    finally:
        # Clean up the global connection on shutdown
        global _isaac_connection
        if _isaac_connection:
            logger.info("Disconnecting from Isaac Sim on shutdown")
            _isaac_connection.disconnect()
            _isaac_connection = None
        logger.info("Isaac SimMCP server shut down")

# Create the MCP server with lifespan support
mcp = FastMCP(
    "IsaacSimMCP",
    description="Isaac Sim integration through the Model Context Protocol",
    lifespan=server_lifespan
)

# Resource endpoints

# Global connection for resources (since resources can't access context)
_isaac_connection = None
# _polyhaven_enabled = False  # Add this global variable

def get_isaac_connection():
    """Get or create a persistent Isaac connection"""
    global _isaac_connection, _polyhaven_enabled  # Add _polyhaven_enabled to globals
    
    # If we have an existing connection, check if it's still valid
    if _isaac_connection is not None:
        try:
            
            return _isaac_connection
        except Exception as e:
            # Connection is dead, close it and create a new one
            logger.warning(f"Existing connection is no longer valid: {str(e)}")
            try:
                _isaac_connection.disconnect()
            except:
                pass
            _isaac_connection = None
    
    # Create a new connection if needed
    if _isaac_connection is None:
        _isaac_connection = IsaacConnection(host="localhost", port=8766)
        if not _isaac_connection.connect():
            logger.error("Failed to connect to Isaac")
            _isaac_connection = None
            raise Exception("Could not connect to Isaac. Make sure the Isaac addon is running.")
        logger.info("Created new persistent connection to Isaac")
    
    return _isaac_connection


@mcp.tool()
def get_scene_info(ctx: Context) -> str:
    """Ping status of Isaac Sim Extension Server"""
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("get_scene_info")
        print("result: ", result)
        
        # Just return the JSON representation of what Isaac sent us
        return json.dumps(result, indent=2)
        # return json.dumps(result)
        # return result
    except Exception as e:
        logger.error(f"Error getting scene info from Isaac: {str(e)}")
        # return f"Error getting scene info: {str(e)}"
        return {"status": "error", "error": str(e), "message": "Error getting scene info"}

@mcp.tool()
def execute_script(ctx: Context, code: str) -> str:
    """
    Before execute script pls check prompt from asset_creation_strategy() to ensure the scene is properly initialized.
    Execute arbitrary Python code in Isaac Sim. Before executing any code, first verify if get_scene_info() has been called to ensure the scene is properly initialized. Always print the formatted code into chat to confirm before execution to confirm its correctness. 
    Before execute script pls check if create_physics_scene() has been called to ensure the physics scene is properly initialized.
    When working with robots, always try using the create_robot() function first before resorting to execute_script(). The create_robot() function provides a simpler, more reliable way to add robots to your scene with proper initialization and positioning. Only use execute_script() for robot creation when you need custom configurations or behaviors not supported by create_robot().
    
    For physics simulation, avoid using simulation_context to run simulations in the main thread as this can cause blocking. Instead, use the World class with async methods for initializing physics and running simulations. For example, use my_world = World(physics_dt=1.0/60.0) and my_world.step_async() in a loop, which allows for better performance and responsiveness. If you need to wait for physics to stabilize, consider using my_world.play() followed by multiple step_async() calls.
    To create an simulation of Franka robot, the code should be like this:
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage, is_stage_loading
from omni.isaac.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
simulation_context = SimulationContext()
add_reference_to_stage(asset_path, "/Franka")
#create_prim("/DistantLight", "DistantLight")




    To control the Franka robot, the code should be like this:

from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

my_world = World(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

simulation_context = SimulationContext()
add_reference_to_stage(asset_path, "/Franka")

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize(my_world.physics_sim_view)
dof_ptr = art.get_dof_index("panda_joint2")

simulation_context.play()
# NOTE: before interacting with dc directly you need to step physics for one step at least
# simulation_context.step(render=True) which happens inside .play()
for i in range(1000):
    art.set_joint_positions([-1.5], [dof_ptr])
    simulation_context.step(render=True)

simulation_context.stop()


    
    Parameters:
    - code: The Python code to execute, e.g. "omni.kit.commands.execute("CreatePrim", prim_type="Sphere")"
    """
    try:
        # Get the global connection
        isaac = get_isaac_connection()
        print("code: ", code)
        
        result = isaac.send_command("execute_script", {"code": code})
        print("result: ", result)
        return result
        # return f"Code executed successfully: {result.get('result', '')}"
    except Exception as e:
        logger.error(f"Error executing code: {str(e)}")
        # return f"Error executing code: {str(e)}"
        return {"status": "error", "error": str(e), "message": "Error executing code"}

    
@mcp.tool()
def list_prims() -> str:
    """
    List all prim paths in the current USD scene.
    Useful for discovering available objects before moving, reading poses, or other operations.
    
    Example:
        list_prims()
    """
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("list_prims")
        
        if result.get("status") == "success":
            prims = result.get("prims", [])
            
            response = f"Prims in scene ({len(prims)}):\n\n"
            
            for prim in prims:
                path = prim['path']
                name = prim['name']
                prim_type = prim['type']
                
                # Format: path (type) or path [name] (type) if name differs
                if name and name != path.split('/')[-1]:
                    response += f"{path} [{name}] ({prim_type})\n"
                else:
                    response += f"{path} ({prim_type})\n"
            
            return response
        else:
            return f"Error listing prims: {result.get('message', 'Unknown error')}"
            
    except Exception as e:
        logger.error(f"Error listing prims: {str(e)}")
        return f"Error listing prims: {str(e)}"

@mcp.tool()
def open_usd(usd_path: str) -> str:
    """
    Open a USD file in the Isaac Sim stage.
    
    Args:
        usd_path: Path to the USD file (local or Omniverse path)
    
    Examples:
        open_usd("omniverse://localhost/Library/Aruco/DT.usd")
        open_usd("/path/to/your/file.usd")
    """
    try:
        # Use the existing connection pattern like other tools
        isaac = get_isaac_connection()
        result = isaac.send_command("open_usd", {
            "usd_path": usd_path
        })
        return f"Successfully opened USD stage: {result.get('message', '')}"
    except Exception as e:
        logger.error(f"Error opening USD: {str(e)}")
        return f"Error opening USD: {str(e)}"

@mcp.tool()
def import_usd(usd_path: str, prim_path: str = None, position: list = None, orientation: list = None, orientation_format: str = "degrees") -> str:
    """
    Import a USD file as a prim into the Isaac Sim stage with flexible orientation input.
    
    Args:
        usd_path: Path to the USD file (local or Omniverse path)
        prim_path: Target prim path (optional, auto-generated if not provided)
        position: [x, y, z] position coordinates (optional, defaults to [0, 0, 0])
        orientation: Orientation values (optional, defaults to [0, 0, 0])
        orientation_format: Format of orientation input - "degrees", "radians", or "quaternion"
    
    Examples:
        import_usd("omniverse://localhost/Library/Aruco/objs/base_0.usd")
        import_usd("omniverse://localhost/Library/Aruco/objs/base_0.usd", position=[1.0, 2.0, 0.0], orientation=[0, 0, 45], orientation_format="degrees")
        import_usd("omniverse://localhost/Library/Aruco/objs/base_0.usd", orientation=[0, 0, 0.785], orientation_format="radians")  
        import_usd("omniverse://localhost/Library/Aruco/objs/base_0.usd", orientation=[1, 0, 0, 0], orientation_format="quaternion")
    """
    try:
        # Use the existing connection pattern like other tools
        isaac = get_isaac_connection()
        
        result = isaac.send_command("import_usd", {
            "usd_path": usd_path,
            "prim_path": prim_path,
            "position": position,
            "orientation": orientation,
            "orientation_format": orientation_format
        })
        
        return f"Successfully imported USD: {result.get('message', '')}, prim path: {result.get('prim_path', '')}, position: {result.get('position', [0, 0, 0])}, orientation: {result.get('orientation', [0, 0, 0])} ({result.get('orientation_format', 'degrees')})"
        
    except Exception as e:
        logger.error(f"Error importing USD: {str(e)}")
        return f"Error importing USD: {str(e)}"

@mcp.tool()
def load_scene() -> str:
    """
    Load a basic scene with world and ground plane in Isaac Sim.
    This initializes the simulation context and adds a default ground plane.
    
    Example:
        load_scene()
    """
    try:
        # Use the existing connection pattern
        isaac = get_isaac_connection()
        
        result = isaac.send_command("load_scene")
        
        return f"Successfully loaded scene: {result.get('message', '')}"
        
    except Exception as e:
        logger.error(f"Error loading scene: {str(e)}")
        return f"Error loading scene: {str(e)}"


@mcp.tool()
def get_object_info(prim_path: str) -> str:
    """
    Get comprehensive information about an object including its pose in all formats.
    
    Args:
        prim_path: Path to the prim/object (use list_prims() to find available paths)
    
    Example:
        get_object_info("/World/UR5e")
    """
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("get_object_info", {"prim_path": prim_path})
        
        if result.get("status") == "success":
            prim_info = result.get("prim_info", {})
            position = result.get("position", [])
            quat = result.get("rotation_quaternion", [])
            rad = result.get("rotation_radians", [])
            deg = result.get("rotation_degrees", [])
            scale = result.get("scale", [])
            
            response = f"Object Info for {prim_path}:\n\n"
            
            # Basic info
            response += f"Basic Properties:\n"
            response += f"  Name: {prim_info.get('name', 'N/A')}\n"
            response += f"  Type: {prim_info.get('type', 'N/A')}\n"
            response += f"  Active: {prim_info.get('is_active', False)}\n"
            response += f"  Has Children: {prim_info.get('has_children', False)}\n\n"
            
            # Transform info
            response += f"Transform:\n"
            response += f"  Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]\n\n"
            response += f"  Rotation:\n"
            response += f"    Degrees (r,p,y): [{deg[0]:.3f}, {deg[1]:.3f}, {deg[2]:.3f}]\n"
            response += f"    Radians (r,p,y): [{rad[0]:.3f}, {rad[1]:.3f}, {rad[2]:.3f}]\n"
            response += f"    Quaternion (w,x,y,z): [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]\n\n"
            response += f"  Scale: [{scale[0]:.3f}, {scale[1]:.3f}, {scale[2]:.3f}]"
            
            return response
        else:
            return f"Error getting object info: {result.get('message', 'Unknown error')}"
            
    except Exception as e:
        logger.error(f"Error getting object info: {str(e)}")
        return f"Error getting object info: {str(e)}"


@mcp.tool()
def move_prim(prim_path: str, position: list = None, orientation: list = None, 
              orientation_format: str = "degrees") -> str:
    """
    Move a prim to a new pose with flexible orientation input.
    Reads current pose and allows selective updates to position and orientation.
    
    Args:
        prim_path: Path to the prim/object (use list_prims() to find available paths)
        position: [x, y, z] new position (optional, keeps current if not specified)
        orientation: New orientation values (optional, keeps current if not specified)
        orientation_format: "degrees", "radians", or "quaternion" (default: degrees)
    
    Examples:
        # Move to new position only
        move_prim("/World/trial", position=[1.0, 2.0, 0.5])
        
        # Move with rotation in degrees
        move_prim("/World/trial", position=[1.0, 2.0, 0.5], orientation=[0, 0, 45])
        
        # Move with rotation in radians
        move_prim("/World/trial", position=[1.0, 2.0, 0.5], orientation=[0, 0, 0.785], orientation_format="radians")
        
        # Move with quaternion
        move_prim("/World/trial", position=[1.0, 2.0, 0.5], orientation=[0.924, 0, 0, 0.383], orientation_format="quaternion")
        
        # Just rotate, keep current position
        move_prim("/World/trial", orientation=[90, 0, 0])
    """
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("move_prim", {
            "prim_path": prim_path,
            "position": position,
            "orientation": orientation,
            "orientation_format": orientation_format
        })
        
        if result.get("status") == "success":
            prev_pos = result.get("previous_position", [])
            prev_quat = result.get("previous_quaternion", [])
            new_pos = result.get("new_position", [])
            new_quat = result.get("new_quaternion", [])
            
            response = f"Successfully moved {prim_path}:\n\n"
            
            response += f"Previous Pose:\n"
            response += f"  Position: [{prev_pos[0]:.3f}, {prev_pos[1]:.3f}, {prev_pos[2]:.3f}]\n"
            response += f"  Quaternion: [{prev_quat[0]:.3f}, {prev_quat[1]:.3f}, {prev_quat[2]:.3f}, {prev_quat[3]:.3f}]\n\n"
            
            response += f"New Pose:\n"
            response += f"  Position: [{new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f}]\n"
            response += f"  Quaternion: [{new_quat[0]:.3f}, {new_quat[1]:.3f}, {new_quat[2]:.3f}, {new_quat[3]:.3f}]\n"
            response += f"  Orientation Format: {result.get('orientation_format', 'degrees')}"
            
            return response
        else:
            return f"Error moving prim: {result.get('message', 'Unknown error')}"
            
    except Exception as e:
        logger.error(f"Error moving prim: {str(e)}")
        return f"Error moving prim: {str(e)}"

@mcp.tool()
def control_gripper(ctx: Context, command: str) -> str:
    """
    Control gripper with flexible input commands.
    
    Args:
        command: Gripper control command. Can be:
                - String commands: "open", "close"
                - Numeric values: "0" to "1100" (representing 0cm to 11cm gripper width)
                
    Value meanings:
        - "open" or "1100" = Fully open (11cm width)
        - "close" or "0" = Fully closed (0cm width)  
        - Any value "0"-"1100" = Proportional opening (e.g., "550" = 5.5cm width)
    
    Examples:
        control_gripper("open")          # Fully open gripper
        control_gripper("close")         # Fully close gripper
        control_gripper("550")           # Half open (5.5cm width)
        control_gripper("1100")          # Fully open (same as "open")
        control_gripper("0")             # Fully closed (same as "close")
    """
    try:
        isaac = get_isaac_connection()
        
        # Try to convert to numeric if it's a string number
        if command.isdigit() or (command.replace('.', '').isdigit()):
            # It's a numeric string, convert to float for validation
            try:
                numeric_command = float(command)
                result = isaac.send_command("control_gripper", {"command": numeric_command})
            except ValueError:
                result = isaac.send_command("control_gripper", {"command": command})
        else:
            # It's a string command like "open" or "close"
            result = isaac.send_command("control_gripper", {"command": command})
        
        if result.get("status") == "success":
            command_sent = result.get("command_sent", "unknown")
            numeric_value = result.get("numeric_value", 0)
            width_cm = result.get("width_cm", 0.0)
            
            response = f"✓ Gripper control successful!\n\n"
            response += f"Command sent: '{command_sent}'\n"
            response += f"Numeric value: {numeric_value}\n"
            response += f"Gripper width: {width_cm:.1f} cm\n"
            
            if result.get("ros_output"):
                response += f"ROS output: {result.get('ros_output')}"
                
            return response
        else:
            return f"✗ Gripper control failed: {result.get('message', 'Unknown error')}"
            
    except Exception as e:
        logger.error(f"Error controlling gripper: {str(e)}")
        return f"✗ Error controlling gripper: {str(e)}"

@mcp.tool()
def perform_ik(ctx: Context, robot_prim_path: str, target_position: list, target_rpy: list, 
               custom_lib_path: str = "/home/aaugus11/Desktop/ros2_ws/src/ur_asu-main/ur_asu/custom_libraries") -> str:
    """
    Perform inverse kinematics on the robot and move it to the target pose.
    
    Args:
        robot_prim_path: Path to the robot prim (e.g., "/World/UR5e")
        target_position: [x, y, z] target position in meters (e.g., [0.4, 0.2, 0.3])
        target_rpy: [roll, pitch, yaw] target orientation in degrees (e.g., [0, 90, 0])
        custom_lib_path: Path to custom IK solver library (optional, defaults to your ur_asu path)
        
    Examples:
        perform_ik("/World/UR5e", [0.4, 0.2, 0.3], [0, 90, 0])
        perform_ik("/World/UR5e", [0.5, -0.1, 0.4], [45, 0, -30])
        perform_ik("/World/UR5e", [0.3, 0.0, 0.5], [0, 0, 180])
        
    Note: 
        - Position is in meters relative to robot base
        - RPY is in degrees (roll, pitch, yaw)
        - Robot must exist in scene (use list_prims() to check)
        - Custom IK solver must be available at the specified path
    """
    try:
        isaac = get_isaac_connection()
        
        result = isaac.send_command("perform_ik", {
            "robot_prim_path": robot_prim_path,
            "target_position": target_position,
            "target_rpy": target_rpy,
            "custom_lib_path": custom_lib_path
        })
        
        if result.get("status") == "success":
            target_pos = result.get("target_position", [])
            target_rot = result.get("target_rpy", [])
            joint_rad = result.get("joint_angles_rad", [])
            joint_deg = result.get("joint_angles_deg", [])
            prev_rad = result.get("previous_joints_rad", [])
            prev_deg = result.get("previous_joints_deg", [])
            
            response = f"✓ IK solved and robot moved successfully!\n\n"
            
            response += f"Target Pose:\n"
            response += f"  Position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] m\n"
            response += f"  RPY: [{target_rot[0]:.1f}, {target_rot[1]:.1f}, {target_rot[2]:.1f}] deg\n\n"
            
            if prev_rad:
                response += f"Previous Joint Angles:\n"
                response += f"  Radians: {[f'{j:.3f}' for j in prev_rad]}\n"
                response += f"  Degrees: {[f'{j:.1f}' for j in prev_deg]}\n\n"
            
            response += f"New Joint Angles:\n"
            response += f"  Radians: {[f'{j:.3f}' for j in joint_rad]}\n"
            response += f"  Degrees: {[f'{j:.1f}' for j in joint_deg]}\n\n"
            
            response += f"Robot: {result.get('robot_prim_path')}"
            
            return response
            
        else:
            error_msg = result.get("message", "Unknown error")
            response = f"✗ IK failed: {error_msg}\n\n"
            
            # Add helpful info if available
            if "target_position" in result:
                response += f"Target position: {result['target_position']}\n"
            if "target_rpy" in result:
                response += f"Target RPY: {result['target_rpy']}\n"
            if "current_joints_rad" in result and result["current_joints_rad"]:
                response += f"Current joints (rad): {[f'{j:.3f}' for j in result['current_joints_rad']]}\n"
            
            # Common troubleshooting tips
            if "not found" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Use list_prims() to see available objects\n"
                response += "- Make sure robot is loaded in the scene\n"
            elif "import" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Check if ik_solver.py exists in custom_lib_path\n"
                response += "- Verify the custom library path is correct\n"
            elif "solution" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Try a different target position (closer to robot)\n"
                response += "- Check if target is within robot's reach\n"
                response += "- Verify orientation values are reasonable\n"
            
            return response
            
    except Exception as e:
        logger.error(f"Error performing IK: {str(e)}")
        return f"✗ Error performing IK: {str(e)}"
        
@mcp.tool()
def get_ee_pose(ctx: Context, robot_prim_path: str, joint_angles: list = None,
                custom_lib_path: str = "/home/aaugus11/Desktop/ros2_ws/src/ur_asu-main/ur_asu/custom_libraries") -> str:
    """
    Get end-effector pose using forward kinematics from current or specified joint angles.
    
    Args:
        robot_prim_path: Path to the robot prim (e.g., "/World/UR5e")
        joint_angles: Optional joint angles in radians [j1, j2, j3, j4, j5, j6]. If None, uses current robot state
        custom_lib_path: Path to custom IK solver library (optional)
        
    Examples:
        # Get current end-effector pose
        get_ee_pose("/World/UR5e")
        
        # Get end-effector pose for specific joint angles (in radians)
        get_ee_pose("/World/UR5e", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        
        # Get pose for joint angles in a known configuration
        get_ee_pose("/World/UR5e", [1.48, -1.40, 1.57, -1.57, -1.57, 0.0])
        
    Note:
        - If joint_angles not provided, reads current robot joint positions
        - Joint angles should be in radians
        - Uses the same forward kinematics as your IK solver
        - Returns position in meters and orientation in multiple formats
    """
    try:
        isaac = get_isaac_connection()
        
        result = isaac.send_command("get_ee_pose", {
            "robot_prim_path": robot_prim_path,
            "joint_angles": joint_angles,
            "custom_lib_path": custom_lib_path
        })
        
        if result.get("status") == "success":
            source = result.get("joint_angles_source", "unknown")
            joint_rad = result.get("joint_angles_rad", [])
            joint_deg = result.get("joint_angles_deg", [])
            ee_pos = result.get("ee_position", [])
            ee_rpy_rad = result.get("ee_rpy_rad", [])
            ee_rpy_deg = result.get("ee_rpy_deg", [])
            ee_quat = result.get("ee_quaternion_xyzw", [])
            
            response = f"✓ End-effector pose computed successfully!\n\n"
            
            response += f"Robot: {result.get('robot_prim_path')}\n"
            response += f"Joint angles source: {source.replace('_', ' ')}\n\n"
            
            response += f"Joint Angles:\n"
            response += f"  Radians: {[f'{j:.3f}' for j in joint_rad]}\n"
            response += f"  Degrees: {[f'{j:.1f}' for j in joint_deg]}\n\n"
            
            response += f"End-Effector Position:\n"
            response += f"  X: {ee_pos[0]:.4f} m\n"
            response += f"  Y: {ee_pos[1]:.4f} m\n"
            response += f"  Z: {ee_pos[2]:.4f} m\n\n"
            
            response += f"End-Effector Orientation:\n"
            response += f"  RPY (deg): [{ee_rpy_deg[0]:.1f}, {ee_rpy_deg[1]:.1f}, {ee_rpy_deg[2]:.1f}]\n"
            response += f"  RPY (rad): [{ee_rpy_rad[0]:.3f}, {ee_rpy_rad[1]:.3f}, {ee_rpy_rad[2]:.3f}]\n"
            response += f"  Quaternion (x,y,z,w): [{ee_quat[0]:.3f}, {ee_quat[1]:.3f}, {ee_quat[2]:.3f}, {ee_quat[3]:.3f}]\n\n"
            
            # Add a summary line for quick reference
            response += f"📍 Summary: Position [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] m, "
            response += f"RPY [{ee_rpy_deg[0]:.1f}, {ee_rpy_deg[1]:.1f}, {ee_rpy_deg[2]:.1f}] deg"
            
            return response
            
        else:
            error_msg = result.get("message", "Unknown error")
            response = f"✗ Forward kinematics failed: {error_msg}\n\n"
            
            # Add helpful info if available
            if "joint_angles_rad" in result:
                response += f"Joint angles used (rad): {[f'{j:.3f}' for j in result['joint_angles_rad']]}\n"
            if "joint_angles_deg" in result:
                response += f"Joint angles used (deg): {[f'{j:.1f}' for j in result['joint_angles_deg']]}\n"
            
            # Common troubleshooting tips
            if "not found" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Use list_prims() to see available objects\n"
                response += "- Make sure robot is loaded in the scene\n"
            elif "import" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Check if ik_solver.py exists in custom_lib_path\n"
                response += "- Verify the custom library path is correct\n"
            elif "joint positions" in error_msg.lower():
                response += "\n💡 Tips:\n"
                response += "- Robot may not be properly initialized\n"
                response += "- Try providing joint_angles explicitly\n"
                response += "- Ensure robot articulation is set up correctly\n"
            
            return response
            
    except Exception as e:
        logger.error(f"Error getting end-effector pose: {str(e)}")
        return f"✗ Error getting end-effector pose: {str(e)}"
        
# Main execution

def main():
    """Run the MCP server"""
    mcp.run()

if __name__ == "__main__":
    main()