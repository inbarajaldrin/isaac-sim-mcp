import asyncio
import json
import os
import sys
from anthropic import AsyncAnthropic
from dotenv import load_dotenv
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'isaac_mcp'))
from client import MCPClient

load_dotenv()

class ClaudeAgent:
    def __init__(self):
        # Claude
        api_key = os.getenv("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("Set ANTHROPIC_API_KEY")
        
        self.claude = AsyncAnthropic(api_key=api_key)
        
        # MCP Client
        self.mcp_client = MCPClient("isaac_mcp/server.py")
    
    async def process_request(self, user_input: str):
        """Process user request"""
        
        # First, get available tools by doing a quick connection
        await self.get_available_tools()
        
        # Ask Claude what to do
        print("Asking Claude...")
        claude_response = await self.ask_claude(user_input)
        
        # Show Claude's reasoning
        reasoning = claude_response.split("TOOLS:")[0].strip()
        if reasoning:
            print(f"Claude's plan: {reasoning}")
        
        # Extract tool calls
        tool_calls = self.extract_tools(claude_response)
        
        # Execute tools via MCP client
        await self.mcp_client.run_with_isaac(tool_calls)
    
    async def get_available_tools(self):
        """Quick connection to get available tools"""
        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
            
            server_params = StdioServerParameters(
                command="python3",
                args=["isaac_mcp/server.py"]
            )
            
            async with stdio_client(server_params) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools_result = await session.list_tools()
                    self.mcp_client.available_tools = [tool.name for tool in tools_result.tools]
        except Exception as e:
            print(f"Could not get tools: {e}")
            self.mcp_client.available_tools = []
    
    async def ask_claude(self, user_input: str) -> str:
        """Ask Claude what tools to use - copied from your working code"""
        
        tools_list = ", ".join(self.mcp_client.available_tools)
        
        system_prompt = f"""You control Isaac Sim using these MCP tools: {tools_list}

Available tools:
- get_scene_info: Check scene status (no params)
- create_physics_scene: Create physics scene {{"gravity": [0, -9.81, 0]}}
- create_robot: Create robot {{"robot_type": "franka", "position": [0, 0, 0]}}
- execute_script: Run Python code {{"script": "print('hello')"}}
- transform: Move objects {{"object_path": "/robot", "position": [1, 2, 3]}}
- generate_3d_from_text_or_image: Generate objects {{"prompt": "blue cube", "position": [1, 1, 1]}}
- search_3d_usd_by_text: Search assets {{"query": "chair"}}

IMPORTANT: When responding with tool calls, put them in a single line JSON array format.

Respond with brief explanation then on a single line:
TOOLS: [{{"tool": "tool_name", "params": {{"key": "value"}}}}]

Example:
I'll create a Franka robot for you.
TOOLS: [{{"tool": "create_robot", "params": {{"robot_type": "franka", "position": [1, 0, 0]}}}}]

For multiple tools:
TOOLS: [{{"tool": "get_scene_info", "params": {{}}}}, {{"tool": "create_robot", "params": {{"robot_type": "franka", "position": [0, 0, 0]}}}}]"""

        try:
            response = await self.claude.messages.create(
                model="claude-3-5-sonnet-20241022",
                max_tokens=1500,
                system=system_prompt,
                messages=[{"role": "user", "content": user_input}]
            )
            
            return response.content[0].text
            
        except Exception as e:
            return f"Claude error: {e}"
    
    def extract_tools(self, claude_response: str):
        """Extract tool calls from Claude response"""
        try:
            if "TOOLS:" in claude_response:
                # Find the start of the JSON array
                start = claude_response.find("TOOLS:") + 6
                json_start = claude_response.find("[", start)
                
                if json_start == -1:
                    return []
                
                # Find the matching closing bracket
                bracket_count = 0
                json_end = json_start
                
                for i, char in enumerate(claude_response[json_start:], json_start):
                    if char == '[':
                        bracket_count += 1
                    elif char == ']':
                        bracket_count -= 1
                        if bracket_count == 0:
                            json_end = i + 1
                            break
                
                # Extract the JSON string
                json_str = claude_response[json_start:json_end]
                
                # Clean up the JSON string - remove extra whitespace and newlines
                json_str = json_str.replace('\n', ' ').replace('\r', '')
                
                # Parse the JSON
                tools = json.loads(json_str)
                return tools
                
        except Exception as e:
            print(f"Could not parse tools: {e}")
            print(f"Debug - Raw response: {claude_response[:500]}...")
        
        return []
