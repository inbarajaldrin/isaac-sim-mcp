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
        
# Main execution

def main():
    """Run the MCP server"""
    mcp.run()

if __name__ == "__main__":
    main()