#!/usr/bin/env python3
"""
Test script for MCP configuration
"""

import asyncio
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(__file__))

from client import MCPClient

async def test_config():
    """Test the MCP configuration"""
    print("Testing MCP Configuration...")
    
    # Create config client
    client = MCPClient()
    
    # List available servers
    servers = client.list_servers()
    print(f"Available servers: {servers}")
    
    # Test getting server config
    for server in servers:
        config = client.get_server_config(server)
        print(f"Server '{server}' config: {config}")
    
    # Test getting available tools
    print("\nTesting tool discovery...")
    try:
        tools = await client.get_available_tools()
        print(f"Available tools: {tools}")
    except Exception as e:
        print(f"Error getting tools: {e}")

if __name__ == "__main__":
    asyncio.run(test_config())
