#!/usr/bin/env python3
"""
Host module for Isaac Sim MCP Interface
Unified host that handles both Claude and ChatGPT agents
"""

import asyncio
import json
import os
import sys
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv

# Import your existing MCP client
from client import MCPClient

# Import LLM clients
try:
    from anthropic import AsyncAnthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

load_dotenv()

class MCPHost:
    """Unified host for both Claude and ChatGPT agents"""
    
    def __init__(self, backend: str):
        self.backend = backend.lower()
        self.llm_client = None
        
        # Use your existing MCP client - fix path detection
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Check if we're already in isaac_mcp directory
        if os.path.basename(current_dir) == "isaac_mcp":
            # We're inside isaac_mcp, server.py is in the same directory
            server_path = os.path.join(current_dir, "server.py")
        else:
            # We're in parent directory, look for isaac_mcp/server.py
            server_path = os.path.join(current_dir, "isaac_mcp", "server.py")
        
        print(f"Debug: Looking for server at: {server_path}")
        self.mcp_client = MCPClient(server_path)
        
        # Initialize the appropriate LLM client
        self._initialize_llm_client()
    
    def _initialize_llm_client(self):
        """Initialize the LLM client based on backend"""
        if self.backend == "claude":
            if not ANTHROPIC_AVAILABLE:
                raise ImportError("Anthropic package not available. Install with: pip install anthropic")
            
            api_key = os.getenv("ANTHROPIC_API_KEY")
            if not api_key:
                raise ValueError("ANTHROPIC_API_KEY not found in environment variables")
            
            self.llm_client = AsyncAnthropic(api_key=api_key)
            
        elif self.backend == "chatgpt":
            if not OPENAI_AVAILABLE:
                raise ImportError("OpenAI package not available. Install with: pip install openai")
            
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY not found in environment variables")
            
            self.llm_client = OpenAI(api_key=api_key)
            
        elif self.backend == "ollama":
            if not OLLAMA_AVAILABLE:
                raise ImportError("Ollama package not available. Install with: pip install ollama")
            
            # Set the model name for Ollama
            self.model_name = "codellama:13b"
            self.llm_client = ollama  # Use ollama module directly
            
        else:
            raise ValueError(f"Unsupported backend: {self.backend}")
    
    async def process_request(self, user_input: str, history: list = None):
        """Process user request using the selected backend"""
        
        try:
            # First, get available tools using your existing MCP client
            await self._get_available_tools()
            
            # Ask the LLM what to do
            print(f"Asking {self.backend.title()}...")
            llm_response = await self._ask_llm(user_input, history)
            
            # Show LLM's reasoning
            reasoning = llm_response.split("TOOLS:")[0].strip()
            if reasoning:
                print(f"{self.backend.title()}'s plan: {reasoning}")
            
            # Extract tool calls
            tool_calls = self._extract_tools(llm_response)
            
            # Execute tools via your existing MCP client if any found
            if tool_calls:
                print(f"🔧 Executing {len(tool_calls)} tool(s)...")
                await self.mcp_client.run_with_isaac(tool_calls)
                
                # Return reasoning + execution confirmation
                return f"{reasoning}\n\nExecuted {len(tool_calls)} Isaac Sim command(s) successfully!"
            else:
                # No tools to execute, just return LLM's response
                return llm_response
                
        except Exception as e:
            error_msg = f"{self.backend.title()} agent error: {e}"
            print(error_msg)
            return error_msg
    
    async def _get_available_tools(self):
        """Get available tools using your existing MCP client"""
        try:
            # Use the method from your existing MCPClient
            await self.mcp_client.get_available_tools()
        except Exception as e:
            print(f"Could not get tools: {e}")
            self.mcp_client.available_tools = []
    
    async def _ask_llm(self, user_input: str, history: list = None) -> str:
        """Ask the LLM what tools to use"""
        
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
            if self.backend == "claude":
                return await self._ask_claude(system_prompt, user_input, history)
            elif self.backend == "chatgpt":
                return await self._ask_chatgpt(system_prompt, user_input, history)
            elif self.backend == "ollama":
                return await self._ask_ollama(system_prompt, user_input, history)
            else:
                return f"Unsupported backend: {self.backend}"
                
        except Exception as e:
            return f"{self.backend.title()} error: {e}"
    
    async def _ask_claude(self, system_prompt: str, user_input: str, history: list = None) -> str:
        """Ask Claude specifically"""
        
        # Build messages with history for context
        messages = []
        
        # Add previous conversation history if provided
        if history:
            for msg in history[-5:]:  # Keep last 5 messages for context
                if isinstance(msg, dict) and msg.get("role") in ["user", "assistant"]:
                    messages.append({"role": msg["role"], "content": msg.get("content", "")})
        
        # Add current user input
        messages.append({"role": "user", "content": user_input})
        
        response = await self.llm_client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1500,
            system=system_prompt,
            messages=messages
        )
        
        return response.content[0].text
    
    async def _ask_chatgpt(self, system_prompt: str, user_input: str, history: list = None) -> str:
        """Ask ChatGPT specifically"""
        
        # Build messages with history for context
        messages = [{"role": "system", "content": system_prompt}]
        
        # Add previous conversation history if provided
        if history:
            for msg in history[-5:]:  # Keep last 5 messages for context
                if isinstance(msg, dict) and msg.get("role") in ["user", "assistant"]:
                    messages.append({"role": msg["role"], "content": msg.get("content", "")})
        
        # Add current user input
        messages.append({"role": "user", "content": user_input})
        
        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=messages,
            max_tokens=1500,
            temperature=0.3  # Lower temperature for more consistent tool calling
        )
        
        return response.choices[0].message.content
    
    async def _ask_ollama(self, system_prompt: str, user_input: str, history: list = None) -> str:
        """Ask Ollama specifically"""
        
        # Build the full prompt with system message and history
        full_prompt = f"{system_prompt}\n\n"
        
        # Add previous conversation history if provided
        if history:
            for msg in history[-5:]:  # Keep last 5 messages for context
                if isinstance(msg, dict) and msg.get("role") in ["user", "assistant"]:
                    role = msg.get("role", "")
                    content = msg.get("content", "")
                    if role == "user":
                        full_prompt += f"User: {content}\n"
                    elif role == "assistant":
                        full_prompt += f"Assistant: {content}\n"
        
        # Add current user input
        full_prompt += f"User: {user_input}\nAssistant:"
        
        try:
            response = self.llm_client.generate(
                model=self.model_name,
                prompt=full_prompt,
                options={
                    'temperature': 0.3,  # Lower temperature for more consistent tool calling
                    'top_p': 0.9,
                    'num_predict': 1500  # Max tokens to generate
                }
            )
            
            return response['response']
            
        except Exception as e:
            return f"Ollama error: {e}"
    
    def _extract_tools(self, llm_response: str):
        """Extract tool calls from LLM response"""
        try:
            if "TOOLS:" in llm_response:
                # Find the start of the JSON array
                start = llm_response.find("TOOLS:") + 6
                json_start = llm_response.find("[", start)
                
                if json_start == -1:
                    return []
                
                # Find the matching closing bracket
                bracket_count = 0
                json_end = json_start
                
                for i, char in enumerate(llm_response[json_start:], json_start):
                    if char == '[':
                        bracket_count += 1
                    elif char == ']':
                        bracket_count -= 1
                        if bracket_count == 0:
                            json_end = i + 1
                            break
                
                # Extract the JSON string
                json_str = llm_response[json_start:json_end]
                
                # Clean up the JSON string - remove extra whitespace and newlines
                json_str = json_str.replace('\n', ' ').replace('\r', '')
                
                # Parse the JSON
                tools = json.loads(json_str)
                return tools
                
        except Exception as e:
            print(f"Could not parse tools: {e}")
            print(f"Debug - Raw response: {llm_response[:500]}...")
        
        return []

# Convenience functions for creating hosts
async def create_claude_host():
    """Create a Claude-based MCP host"""
    return MCPHost("claude")

async def create_chatgpt_host():
    """Create a ChatGPT-based MCP host"""
    return MCPHost("chatgpt")

async def create_ollama_host():
    """Create an Ollama-based MCP host"""
    return MCPHost("ollama")

def check_backend_availability():
    """Check which backends are available"""
    available_backends = []
    
    # Check Claude availability
    if ANTHROPIC_AVAILABLE and os.getenv("ANTHROPIC_API_KEY"):
        available_backends.append("claude")
    
    # Check ChatGPT availability
    if OPENAI_AVAILABLE and os.getenv("OPENAI_API_KEY"):
        available_backends.append("chatgpt")
    
    # Check Ollama availability
    if OLLAMA_AVAILABLE:
        available_backends.append("ollama")
    
    return available_backends