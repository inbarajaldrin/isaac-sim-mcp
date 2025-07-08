import os
import sys
import asyncio
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'agent'))
from claude import ClaudeAgent

async def main():
    print("Isaac Sim Agent")
    print("=" * 30)
    
    agent = ClaudeAgent()
    
    print("What would you like to do in Isaac Sim?")
    print("Type 'quit' to exit")
    print("=" * 30)
    
    while True:
        try:
            user_input = input("\nYou: ").strip()
            
            if user_input.lower() in ['quit', 'q', 'exit']:
                break
                
            if not user_input:
                continue
            
            await agent.process_request(user_input)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Goodbye!")

if __name__ == "__main__":
    asyncio.run(main())