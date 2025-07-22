import json
import os
import sys
from openai import OpenAI
from dotenv import load_dotenv

# Fix the import path - don't duplicate the directory
sys.path.append(os.path.dirname(__file__))
from client import MCPClient

load_dotenv()

class ChatGPTAgent:
    def __init__(self):
        # OpenAI API
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("Set OPENAI_API_KEY in your environment variables")
        
        self.client = OpenAI(api_key=api_key)
        
        # MCP Client - fix the path to server.py (same as Claude agent)
        # Get the absolute path to server.py
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        server_path = os.path.join(parent_dir, "isaac_mcp", "server.py")
        
        self.mcp_client = MCPClient(server_path)
    
    async def process_request(self, user_input: str, history: list = None):
        """Process user request - enhanced for Gradio integration"""
        
        try:
            # First, get available tools by doing a quick connection
            await self.get_available_tools()
            
            # Ask ChatGPT what to do
            print("Asking ChatGPT...")
            chatgpt_response = await self.ask_chatgpt(user_input, history)
            
            # Show ChatGPT's reasoning
            reasoning = chatgpt_response.split("TOOLS:")[0].strip()
            if reasoning:
                print(f"ChatGPT's plan: {reasoning}")
            
            # Extract tool calls
            tool_calls = self.extract_tools(chatgpt_response)
            
            # Execute tools via MCP client if any found
            if tool_calls:
                print(f"🔧 Executing {len(tool_calls)} tool(s)...")
                await self.mcp_client.run_with_isaac(tool_calls)
                
                # Return reasoning + execution confirmation
                return f"{reasoning}\n\nExecuted {len(tool_calls)} Isaac Sim command(s) successfully!"
            else:
                # No tools to execute, just return ChatGPT's response
                return chatgpt_response
                
        except Exception as e:
            error_msg = f"ChatGPT agent error: {e}"
            print(error_msg)
            return error_msg
    
    async def get_available_tools(self):
        """Quick connection to get available tools"""
        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
            
            server_params = StdioServerParameters(
                command="python3",
                args=[self.mcp_client.server_script]
            )
            
            async with stdio_client(server_params) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools_result = await session.list_tools()
                    self.mcp_client.available_tools = [tool.name for tool in tools_result.tools]
        except Exception as e:
            print(f"Could not get tools: {e}")
            self.mcp_client.available_tools = []
    
    async def ask_chatgpt(self, user_input: str, history: list = None) -> str:
        """Ask ChatGPT what tools to use - enhanced with history support"""
        
        tools_list = ", ".join(self.mcp_client.available_tools)
        
        system_prompt = f"""You control Isaac Sim using these MCP tools: {tools_list}

Available tools:
- get_scene_info: Check scene status (no params)
- execute_script: Run Python code {{"code": "print('hello')"}}
- list_prims: List all objects in scene
- open_usd: Open USD file {{"usd_path": "path/to/file.usd"}}
- import_usd: Import USD as prim {{"usd_path": "path", "prim_path": "/World/obj", "position": [0,0,0]}}
- load_scene: Load basic scene with ground plane
- get_object_info: Get detailed object information {{"prim_path": "/World/object"}}
- move_prim: Move objects {{"prim_path": "/World/object", "position": [1, 2, 3], "orientation": [0, 0, 45]}}
- control_gripper: Control robot gripper {{"command": "open"}} or {{"command": "close"}}
- perform_ik: Robot inverse kinematics {{"robot_prim_path": "/World/UR5e", "target_position": [0.4, 0.2, 0.3], "target_rpy": [0, 90, 0]}}
- get_ee_pose: Get end-effector pose {{"robot_prim_path": "/World/UR5e"}}

IMPORTANT: When responding with tool calls, put them in a single line JSON array format.

Respond with brief explanation then on a single line:
TOOLS: [{{"tool": "tool_name", "params": {{"key": "value"}}}}]

Example:
I'll check the scene information for you.
TOOLS: [{{"tool": "get_scene_info", "params": {{}}}}]

For multiple tools:
TOOLS: [{{"tool": "get_scene_info", "params": {{}}}}, {{"tool": "load_scene", "params": {{}}}}]

If no tools are needed (for general questions), just respond normally without TOOLS: line."""

        try:
            # Build messages with history for context
            messages = [{"role": "system", "content": system_prompt}]
            
            # Add previous conversation history if provided (for Gradio integration)
            if history:
                for msg in history[-5:]:  # Keep last 5 messages for context
                    if isinstance(msg, dict) and msg.get("role") in ["user", "assistant"]:
                        messages.append({"role": msg["role"], "content": msg.get("content", "")})
            
            # Add current user input
            messages.append({"role": "user", "content": user_input})
            
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=messages,
                max_tokens=1500,
                temperature=0.3  # Lower temperature for more consistent tool calling
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            return f"ChatGPT error: {e}"
    
    def extract_tools(self, chatgpt_response: str):
        """Extract tool calls from ChatGPT response"""
        try:
            if "TOOLS:" in chatgpt_response:
                # Find the start of the JSON array
                start = chatgpt_response.find("TOOLS:") + 6
                json_start = chatgpt_response.find("[", start)
                
                if json_start == -1:
                    return []
                
                # Find the matching closing bracket
                bracket_count = 0
                json_end = json_start
                
                for i, char in enumerate(chatgpt_response[json_start:], json_start):
                    if char == '[':
                        bracket_count += 1
                    elif char == ']':
                        bracket_count -= 1
                        if bracket_count == 0:
                            json_end = i + 1
                            break
                
                # Extract the JSON string
                json_str = chatgpt_response[json_start:json_end]
                
                # Clean up the JSON string - remove extra whitespace and newlines
                json_str = json_str.replace('\n', ' ').replace('\r', '')
                
                # Parse the JSON
                tools = json.loads(json_str)
                return tools
                
        except Exception as e:
            print(f"Could not parse tools: {e}")
            print(f"Debug - Raw response: {chatgpt_response[:500]}...")
        
        return []