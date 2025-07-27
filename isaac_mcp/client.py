#!/usr/bin/env python3
"""
MCP client
"""

import asyncio
from typing import List, Dict, Any

try:
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client
except ImportError:
    print("Install MCP: pip install mcp")
    exit(1)

class MCPClient:
    def __init__(self, server_script: str):
        self.server_script = server_script
        self.available_tools = []
        
    async def run_with_isaac(self, tool_calls: List[Dict[str, Any]]):
        """Connect, execute tools, disconnect - just like your working code"""
        
        # Connect to Isaac MCP server (copied from your working code)
        server_params = StdioServerParameters(
            command="python3",
            args=[self.server_script]
        )
        
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                # Initialize
                await session.initialize()
                
                # Get available tools
                tools_result = await session.list_tools()
                self.available_tools = [tool.name for tool in tools_result.tools]
                
                print(f"Connected to Isaac Sim! Available tools: {self.available_tools}")
                
                # Execute tool calls
                if tool_calls:
                    print(f" Executing {len(tool_calls)} tool(s)...")
                    
                    for tool_call in tool_calls:
                        tool_name = tool_call.get("tool", tool_call.get("name"))
                        params = tool_call.get("params", tool_call.get("arguments", {}))
                        
                        if tool_name in self.available_tools:
                            print(f"     Calling {tool_name} with {params}")
                            
                            try:
                                result = await session.call_tool(tool_name, params)
                                if hasattr(result, 'content') and result.content:
                                    print(f"   Result: {result.content[0].text}")
                                else:
                                    print(f"   Tool executed successfully")
                            except Exception as e:
                                print(f"   Error: {e}")
                        else:
                            print(f"   Tool '{tool_name}' not available")
                else:
                    print("   No tools to execute")

    async def get_available_tools(self):
            """Quick connection to get available tools"""
            try:
                server_params = StdioServerParameters(
                    command="python3",
                    args=[self.server_script]
                )
                
                async with stdio_client(server_params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        tools_result = await session.list_tools()
                        self.available_tools = [tool.name for tool in tools_result.tools]
            except Exception as e:
                print(f"Could not get tools: {e}")
                self.available_tools = []