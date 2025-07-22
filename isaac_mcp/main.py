import os
import sys
import asyncio

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'agent'))

# Import both agents
try:
    from claude import ClaudeAgent
    CLAUDE_AVAILABLE = True
except ImportError as e:
    print(f"Claude agent not available: {e}")
    CLAUDE_AVAILABLE = False

try:
    from chatgpt import ChatGPTAgent
    CHATGPT_AVAILABLE = True
except ImportError as e:
    print(f"ChatGPT agent not available: {e}")
    CHATGPT_AVAILABLE = False

async def main():
    print("Isaac Sim Agent Interface")
    print("=" * 30)
    
    # Check available agents
    available_agents = []
    if CLAUDE_AVAILABLE:
        available_agents.append("Claude")
    if CHATGPT_AVAILABLE:
        available_agents.append("ChatGPT")
    
    if not available_agents:
        print("No agents available! Please check your API keys and dependencies.")
        return
    
    # Agent selection
    print("Available agents:")
    for i, agent_name in enumerate(available_agents, 1):
        print(f"{i}. {agent_name}")
    
    while True:
        try:
            choice = input(f"\nSelect agent (1-{len(available_agents)}) or press Enter for Claude: ").strip()
            if choice == "":
                if "Claude" in available_agents:
                    selected_agent = "Claude"
                    break
                else:
                    selected_agent = available_agents[0]
                    break
            elif choice.isdigit() and 1 <= int(choice) <= len(available_agents):
                selected_agent = available_agents[int(choice) - 1]
                break
            else:
                print("Invalid choice. Please try again.")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            return
    
    # Initialize the selected agent
    agent = None
    try:
        if selected_agent == "Claude" and CLAUDE_AVAILABLE:
            agent = ClaudeAgent()
        elif selected_agent == "ChatGPT" and CHATGPT_AVAILABLE:
            agent = ChatGPTAgent()
        else:
            print(f"Failed to initialize {selected_agent} agent")
            return
    except Exception as e:
        print(f"Error initializing {selected_agent} agent: {e}")
        print("Make sure you have set the required API keys in your .env file")
        return
    
    print(f"\n✓ {selected_agent} agent initialized successfully!")
    print("What would you like to do in Isaac Sim?")
    print("Type 'quit' to exit, 'switch' to change agents")
    print("=" * 30)
    
    while True:
        try:
            user_input = input(f"\nYou ({selected_agent}): ").strip()
            
            if user_input.lower() in ['quit', 'q', 'exit']:
                break
            elif user_input.lower() == 'switch':
                # Switch agent
                if len(available_agents) > 1:
                    current_index = available_agents.index(selected_agent)
                    next_index = (current_index + 1) % len(available_agents)
                    selected_agent = available_agents[next_index]
                    
                    # Initialize new agent
                    try:
                        if selected_agent == "Claude":
                            agent = ClaudeAgent()
                        elif selected_agent == "ChatGPT":
                            agent = ChatGPTAgent()
                        print(f"✓ Switched to {selected_agent} agent")
                    except Exception as e:
                        print(f"Error switching to {selected_agent}: {e}")
                else:
                    print("Only one agent available, cannot switch")
                continue
            elif not user_input:
                continue
            
            # Process the request
            print(f"\n{selected_agent} is thinking...")
            response = await agent.process_request(user_input)
            print(f"\n{selected_agent}: {response}")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("\nGoodbye!")

if __name__ == "__main__":
    asyncio.run(main())