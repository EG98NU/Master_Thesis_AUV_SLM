#!/usr/bin/env python3
"""
Minimal chat interface for RoboticMissionSystem
- Always allows user to type
- Reads system status from status.json
- Sends commands to command.json
- Stop keywords interrupt missions: exit, quit, stop
"""

import json
import time
from pathlib import Path

COMMAND_FILE = Path("command.json")
STATUS_FILE = Path("status.json")
STOP_KEYWORDS = ["exit", "quit", "stop"]

def read_status():
    """Read system status.json safely"""
    try:
        if STATUS_FILE.exists():
            with open(STATUS_FILE, 'r') as f:
                return json.load(f)
    except json.JSONDecodeError:
        return {}
    return {}

def write_command(cmd):
    """Write user command to command.json"""
    data = {
        "command": cmd,
        "timestamp": time.time(),
        "processed": False
    }
    with open(COMMAND_FILE, 'w') as f:
        json.dump(data, f)

def main():
    print("=" * 60)
    print("                     💬 CHAT INTERFACE")
    print("=" * 60)
    print("Monitor development on system terminal.")
    print("Type 'exit', 'quit' or 'stop' to cancel ongoing mission ")
    print("Type 'shutdown' to terminate system")
    print("=" * 60)
    
    last_status = None

    try:
        while True:
            # Display latest system status
            status = read_status()
            if status != last_status:
                msg = status.get("message") or status.get("status") or ""
                # if msg:
                #     print(f"\n❗ System: {msg}")
                last_status = status

            # Get user input
            user_input = input("📝 Enter text: ").strip()
            if not user_input:
                continue

            # Send to system
            write_command(user_input)

            if user_input.lower() in STOP_KEYWORDS:
                print(f"🛑 Stop command sent to system: '{user_input}'")
                print("-" * 60)
            else:
                # print(f"✅ Text sent to system: '{user_input}'")
                print(f"✅ Text sent to system")
                print("-" * 60)
            
            # Short delay to avoid flooding
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n👋 Chat terminated by user")

if __name__ == "__main__":
    main()

