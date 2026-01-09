#!/usr/bin/env python3

"""
Enhanced chat interface for ROS Mission Manager

- Supports multi-line input and complex text formatting
- Preserves special characters (bullets, unicode)
- Reads system status from status.json
- Sends commands to command.json
- Stop keywords interrupt missions: exit, quit, stop
- NEW: Arrow left/right to move through text, arrow up for command history
"""

import json
import time
import readline
from pathlib import Path

COMMAND_FILE = Path("command.json")
STATUS_FILE = Path("status.json")
STOP_KEYWORDS = ["exit", "quit"]

def read_status():
    """Read system status.json safely"""
    try:
        if STATUS_FILE.exists():
            with open(STATUS_FILE, 'r', encoding='utf-8') as f:
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
    with open(COMMAND_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def setup_readline():
    """
    Configure readline for enhanced editing capabilities.
    Enables arrow keys for navigation and history.
    """
    # Enable history persistence (optional)
    history_file = Path.home() / ".chat_history"
    try:
        readline.read_history_file(str(history_file))
    except FileNotFoundError:
        pass
    
    # Set up history length
    readline.set_history_length(100)
    
    return history_file

def save_history(history_file):
    """Save command history to file"""
    try:
        readline.write_history_file(str(history_file))
    except Exception:
        pass

def get_multiline_input(history_file=None):
    """
    Get multi-line input from user with enhanced editing.
    - Arrow left/right: Move through current line
    - Arrow up/down: Navigate history and edit previous commands
    - Press Enter twice (blank line) to submit.
    - Type 'exit', 'quit' or 'shutdown' alone to stop.
    """
    print("\nEnter command (press Enter twice to submit):")
    print(" Or type: exit, quit, shutdown")
    print("-" * 60)
    print("Use arrow keys: ← → to navigate, ↑ ↓ for history")
    print("-" * 60)
    
    lines = []
    empty_line_count = 0
    
    while True:
        try:
            # Use input() which now respects readline settings
            line = input()
            
            # Check if single-word command on first line
            if not lines and line.strip().lower() in STOP_KEYWORDS + ["shutdown"]:
                if history_file:
                    save_history(history_file)
                return line.strip()
            
            # Empty line detection
            if line.strip() == "":
                empty_line_count += 1
                if empty_line_count >= 1 and lines:  # Submit on first empty line after content
                    break
            else:
                empty_line_count = 0
                lines.append(line)
        
        except EOFError:
            break
    
    result = '\n'.join(lines)
    if history_file:
        save_history(history_file)
    return result

def format_status_message(status):
    """Format status message with emoji indicators"""
    status_text = status.get("status", "unknown")
    processing = status.get("processing", False)
    message = status.get("message", "")
    
    # Status icons
    status_icons = {
        "ready": "✅",
        "processing": "⚙️",
        "waiting_clarification": "❓",
        "initializing": "🔧",
        "shutdown": "🛑",
        "error": "❌"
    }
    
    icon = status_icons.get(status_text, "ℹ️")
    output = f"{icon} Status: {status_text.upper()}"
    
    if processing:
        output += " [PROCESSING]"
    if message:
        output += f"\n 💬 {message}"
    if status.get("pending_command"):
        output += f"\n 📋 Pending: {status.get('pending_command')}"
    
    return output

def main():
    print("=" * 60)
    print(" 💬 CHAT INTERFACE - ROS Mission Manager")
    print("=" * 60)
    print("📡 Connected to manager.py")
    print("📝 Supports complex text with special characters")
    print()
    print("Commands:")
    print(" • exit/quit/stop - Interrupt current mission")
    print(" • SHUTDOWN - Terminate entire system")
    print(" • Multi-line - Paste text, press Enter twice")
    print("=" * 60)
    
    # Set up readline with history
    history_file = setup_readline()
    
    last_status = None
    
    try:
        while True:
            # Display latest system status
            status = read_status()
            
            # Show status changes
            if status and status != last_status:
                current_status = status.get("status", "")
                
                # Always show important status changes
                if current_status in ["ready", "processing", "waiting_clarification", "error"]:
                    print()
                    print(format_status_message(status))
                    print()
                
                last_status = status
            
            # Get user input with enhanced editing capabilities
            user_input = get_multiline_input(history_file)
            
            if not user_input or not user_input.strip():
                continue
            
            # Send to manager
            write_command(user_input)
            
            # Handle special commands
            cmd_lower = user_input.lower().strip()
            
            if cmd_lower == "shutdown":
                print("🛑 SHUTDOWN command sent to manager")
                print("⏳ Waiting for system to terminate...")
                time.sleep(2)
                break
            
            elif cmd_lower in STOP_KEYWORDS:
                print(f"⏹️ Stop command sent: '{user_input}'")
                print(" Current mission will be interrupted")
            
            else:
                # Show preview of sent text
                lines = user_input.split('\n')
                line_count = len(lines)
                
                if line_count == 1:
                    # Single line command
                    preview = user_input[:80] + "..." if len(user_input) > 80 else user_input
                    print(f"✅ Command sent: {preview}")
                else:
                    # Multi-line command
                    print(f"✅ Multi-line text sent ({line_count} lines)")
                    print(" First line:", lines[0][:60] + "..." if len(lines[0]) > 60 else lines[0])
                
                # Check if clarification mode
                if status.get("status") == "waiting_clarification":
                    print(" 📝 Sent as clarification response")
            
            print("-" * 60)
            
            # Short delay to avoid flooding
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n")
        print("=" * 60)
        print("👋 Chat interface terminated by user")
        print("⚠️ Manager is still running - send SHUTDOWN to stop it")
        print("=" * 60)
        if history_file:
            save_history(history_file)
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

#########################################################################
# #!/usr/bin/env python3

# """
# Enhanced chat interface for ROS Mission Manager

# - Supports multi-line input and complex text formatting
# - Preserves special characters (bullets, unicode)
# - Reads system status from status.json
# - Sends commands to command.json
# - Stop keywords interrupt missions: exit, quit, stop
# """

# import json
# import time
# from pathlib import Path

# COMMAND_FILE = Path("command.json")
# STATUS_FILE = Path("status.json")
# STOP_KEYWORDS = ["exit", "quit"]

# def read_status():
#     """Read system status.json safely"""
#     try:
#         if STATUS_FILE.exists():
#             with open(STATUS_FILE, 'r', encoding='utf-8') as f:
#                 return json.load(f)
#     except json.JSONDecodeError:
#         return {}
#     return {}

# def write_command(cmd):
#     """Write user command to command.json"""
#     data = {
#         "command": cmd,
#         "timestamp": time.time(),
#         "processed": False
#     }
    
#     with open(COMMAND_FILE, 'w', encoding='utf-8') as f:
#         json.dump(data, f, ensure_ascii=False, indent=2)

# def get_multiline_input():
#     """
#     Get multi-line input from user.
#     Press Enter twice (blank line) to submit.
#     Type 'exit', 'quit' or 'shutdown' alone to stop.
#     """
#     print("\n📝 Enter command (press Enter twice to submit):")
#     print("    Or type: exit, quit, shutdown")
#     print("-" * 60)
    
#     lines = []
#     empty_line_count = 0
    
#     while True:
#         try:
#             line = input()
            
#             # Check if single-word command on first line
#             if not lines and line.strip().lower() in STOP_KEYWORDS + ["shutdown"]:
#                 return line.strip()
            
#             # Empty line detection
#             if line.strip() == "":
#                 empty_line_count += 1
#                 if empty_line_count >= 1 and lines:  # Submit on first empty line after content
#                     break
#             else:
#                 empty_line_count = 0
#                 lines.append(line)
                
#         except EOFError:
#             break
    
#     return '\n'.join(lines)

# def format_status_message(status):
#     """Format status message with emoji indicators"""
#     status_text = status.get("status", "unknown")
#     processing = status.get("processing", False)
#     message = status.get("message", "")
    
#     # Status icons
#     status_icons = {
#         "ready": "✅",
#         "processing": "⚙️",
#         "waiting_clarification": "❓",
#         "initializing": "🔧",
#         "shutdown": "🛑",
#         "error": "❌"
#     }
    
#     icon = status_icons.get(status_text, "ℹ️")
#     output = f"{icon} Status: {status_text.upper()}"
    
#     if processing:
#         output += " [PROCESSING]"
    
#     if message:
#         output += f"\n   💬 {message}"
    
#     if status.get("pending_command"):
#         output += f"\n   📋 Pending: {status.get('pending_command')}"
    
#     return output

# def main():
#     print("=" * 60)
#     print(" 💬 CHAT INTERFACE - ROS Mission Manager")
#     print("=" * 60)
#     print("📡 Connected to manager.py")
#     print("📝 Supports complex text with special characters")
#     print()
#     print("Commands:")
#     print("  • exit/quit/stop - Interrupt current mission")
#     print("  • SHUTDOWN       - Terminate entire system")
#     print("  • Multi-line     - Paste text, press Enter twice")
#     print("=" * 60)
    
#     last_status = None
    
#     try:
#         while True:
#             # Display latest system status
#             status = read_status()
            
#             # Show status changes
#             if status and status != last_status:
#                 current_status = status.get("status", "")
                
#                 # Always show important status changes
#                 if current_status in ["ready", "processing", "waiting_clarification", "error"]:
#                     print()
#                     print(format_status_message(status))
#                     print()
                
#                 last_status = status
            
#             # Get user input (multi-line capable)
#             user_input = get_multiline_input()
            
#             if not user_input or not user_input.strip():
#                 continue
            
#             # Send to manager
#             write_command(user_input)
            
#             # Handle special commands
#             cmd_lower = user_input.lower().strip()
            
#             if cmd_lower == "shutdown":
#                 print("🛑 SHUTDOWN command sent to manager")
#                 print("⏳ Waiting for system to terminate...")
#                 time.sleep(2)
#                 break
            
#             elif cmd_lower in STOP_KEYWORDS:
#                 print(f"⏹️ Stop command sent: '{user_input}'")
#                 print("   Current mission will be interrupted")
            
#             else:
#                 # Show preview of sent text
#                 lines = user_input.split('\n')
#                 line_count = len(lines)
                
#                 if line_count == 1:
#                     # Single line command
#                     preview = user_input[:80] + "..." if len(user_input) > 80 else user_input
#                     print(f"✅ Command sent: {preview}")
#                 else:
#                     # Multi-line command
#                     print(f"✅ Multi-line text sent ({line_count} lines)")
#                     print("   First line:", lines[0][:60] + "..." if len(lines[0]) > 60 else lines[0])
                
#                 # Check if clarification mode
#                 if status.get("status") == "waiting_clarification":
#                     print("   📝 Sent as clarification response")
            
#             print("-" * 60)
            
#             # Short delay to avoid flooding
#             time.sleep(0.1)
            
#     except KeyboardInterrupt:
#         print("\n")
#         print("=" * 60)
#         print("👋 Chat interface terminated by user")
#         print("⚠️ Manager is still running - send SHUTDOWN to stop it")
#         print("=" * 60)
#     except Exception as e:
#         print(f"\n❌ Error: {e}")
#         import traceback
#         traceback.print_exc()

# if __name__ == "__main__":
#     main()

