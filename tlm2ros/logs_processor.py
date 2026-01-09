#!/usr/bin/env python3

"""
Mission Logs Processor

Provides log retrieval and transformation functions for mission event extraction.
Converts raw ROS log messages to natural language descriptions.

Functions:
- preprocess_message(msg: str) -> str: Convert single log message to natural language
- preprocess_logs(logs: List[Dict]) -> List[str]: Process list of logs into events
- extract_mission_events(mission_logs_file: Path, mission_counter: int) -> str: Extract events for specific mission
"""

import yaml
import re
from pathlib import Path
from typing import List, Dict, Any

# Configuration
MISSION_LOGS_FILE = Path("mission_logs.yaml")


def preprocess_message(msg: str) -> str:
    """
    Convert raw log messages from gamma_bt and perceive_world to natural language.
    Covers all ROS_INFO and ROS_WARN patterns from the C++ source code.
    """
    # ============ MISSION COUNTER & ACCEPTANCE ============
    if "[MISSION COUNTER]" in msg or "New mission received" in msg:
        counter = re.search(r'Counter:\s*(\d+)', msg)
        return f"Mission {counter.group(1)} started." if counter else "Mission started."
    
    if "MISSION" in msg and "ACCEPTED" in msg:
        tag = re.search(r'MISSION\s+(\d+)', msg)
        return f"Mission type {tag.group(1)} accepted." if tag else "Mission accepted."
    
    # ============ SYSTEM STATUS ============
    if "SYSTEM STATUS" in msg:
        if "INITIALIZING" in msg:
            return "System is initializing."
        if "SYSTEM READY" in msg or "NODES READY" in msg:
            return "System and guidance are ready."
        if "PERFORMING SURVEY" in msg:
            return "Survey operations are underway."
        if "PERFORMED SURVEY" in msg:
            return "Survey operations completed."
        if "PERFORMING..." in msg or "PERFORMING" in msg:
            return "System is performing mission operations."
        if "PERFORMED" in msg:
            return "Operations completed successfully."
        if "PERFORMING AP" in msg:
            return "Approaching the gate."
        if "PERFORMED AP" in msg:
            return "Approach completed."
        if "PERFORMING GG" in msg:
            return "Passing through the gate."
        if "PERFORMED GG" in msg:
            return "Gate passage completed."
        if "NODE NOT READY" in msg:
            return "System nodes are not yet ready."
    
    # ============ GUIDANCE STATUS ============
    if "GUIDANCE STATUS" in msg:
        if "WAITING" in msg:
            return "Guidance system is waiting."
        if "ACTIVE" in msg:
            return "Guidance system is active."
        if "READY" in msg:
            return "Guidance is ready."
        if "RUNNING" in msg:
            return "Guidance is running."
        if "COMPLETED" in msg:
            return "Guidance operation completed."
    
    if "Guidance:" in msg:
        if "READY" in msg:
            return "Guidance is ready."
        if "RUNNING" in msg:
            return "Guidance is running."
        if "COMPLETED" in msg:
            return "Guidance completed."

        # ============ GUIDANCE MANAGER - NODE STATE ============
    if "Initializing GuidanceNode" in msg:
        return "Guidance manager is initializing."

    if "Subscription completed" in msg and "guidance" in msg.lower():
        return "Guidance manager subscriptions completed."

    if "Activated" in msg and "guidance" in msg.lower():
        return "Guidance manager activated."

    if "Deactivated" in msg and "guidance" in msg.lower():
        return "Guidance manager deactivated."

    # ============ GUIDANCE MANAGER - SWITCHING MODES ============
    if "Switching guidance type" in msg:
        if "ABSOLUTE" in msg:
            return "Switching to absolute (path-based) guidance."
        elif "RELATIVE" in msg:
            return "Switching to relative (vision-based) guidance."
        return "Switching guidance mode."

    if "Switching to ABSOLUTE guidance" in msg:
        return "Switching to absolute (path-based) guidance."

    if "Switching to RELATIVE guidance" in msg:
        return "Switching to relative (vision-based) guidance."

    if "Guidance type is already set" in msg:
        return "Guidance mode already configured."

    if "Path Follower is ACTIVE and Relative Guidance is IDLE" in msg:
        return "Absolute guidance is now active."

    if "Relative Guidance is ACTIVE and Path Follower is IDLE" in msg:
        return "Relative guidance is now active."

    if "Waiting for Path Follower to be ACTIVE" in msg:
        return "Waiting for path follower to activate."

    if "Waiting for Relative Guidance to be ACTIVE" in msg:
        return "Waiting for relative guidance to activate."

    # ============ GUIDANCE MANAGER - MISSION PROCESSING ============
    if "Computed linear Path" in msg:
        return "Linear navigation path computed."

    if "Computing Gate Path" in msg:
        return "Computing gate passage path."

    if "Computed Gate Path" in msg:
        return "Gate passage path computed."

    if "Added Gate Path" in msg:
        return "Gate path loaded into system."

    if "Computing Rotation Path" in msg:
        return "Computing rotation maneuver."

    if "Computed Rotation Path" in msg:
        return "Rotation path computed."

    if "Added Rotation Path" in msg:
        return "Rotation maneuver loaded."

    if "Computing stop path" in msg:
        return "Computing stop position path."

    if "Computing path to fix GPS" in msg:
        return "Computing GPS fix path."

    if "Valid request to center the OPI" in msg:
        return "Object centering mission accepted."

    if "Valid request to rotate around the OPI" in msg:
        return "Object rotation mission accepted."

    if "Received Rotation Command" in msg:
        return "Rotation command received."

    if "Valid Rotation Command" in msg:
        return "Rotation command validated."

    # ============ GUIDANCE MANAGER - MISSION FEEDBACK ============
    if "PATH COMPLETED" in msg:
        return "Navigation path completed successfully."

    if "RELATIVE MISSION COMPLETED" in msg:
        return "Relative guidance mission completed."

    if "Recived Feedback from Path Follower" in msg or "Received Feedback from Path Follower" in msg:
        return "Path follower status update received."

    if "Recived Feedback from Relative Guidance" in msg or "Received Feedback from Relative Guidance" in msg:
        return "Relative guidance status update received."

    # ============ GUIDANCE MANAGER - WARNINGS ============
    if "Mission ID" in msg and "already processed" in msg:
        return "Duplicate mission ignored."

    if "Goal is outside the safe area" in msg:
        return "Goal rejected: outside safe area."

    if "Area center is outside the safe area" in msg:
        return "Survey area rejected: outside safe area."

    if "Area radius is negative" in msg or "Area radius close to 0" in msg:
        return "Survey area rejected: invalid radius."

    if "Stop timeout is not valid" in msg:
        return "Stop command rejected: invalid timeout."

    if "Stop position is outside the safe area" in msg:
        return "Stop position rejected: outside safe area."

    if "Rotation angle is too small" in msg:
        return "Rotation rejected: angle too small."

    if "Invalid omega value" in msg:
        return "Rotation rejected: invalid omega parameter."

    if "Gate is outside the safe area" in msg:
        return "Gate mission rejected: outside safe area."

    if "Invalid rotation direction" in msg:
        return "OPI rotation rejected: invalid direction."

    # ============ GUIDANCE MANAGER - ERRORS ============
    if "Zeno sysmode MANUAL" in msg:
        return "System in manual mode, guidance disabled."

    if "Switching guidance is taking too long" in msg:
        return "Guidance mode switch timeout error."

    if "Path Follower failed to follow the path" in msg:
        return "Path following failed."

    if "Relative Guidance failed" in msg:
        return "Relative guidance mission failed."

    if "Node is not active, ignoring callback" in msg:
        return "Guidance manager not active, command ignored."

    
    # ============ BUOY DETECTION (perceive_world.cpp) ============
    if "BUOY DISCOVERED AND SAVED TO MEMORY" in msg:
        buoy = re.search(r':\s*([a-zA-Z0-9_]+)', msg)
        buoy_name = buoy.group(1) if buoy else "buoy"
        # Extract color from buoy name
        color_match = re.search(r'boa_([a-z]+)_', buoy_name)
        color = color_match.group(1) if color_match else ""
        return f"A {color} buoy ({buoy_name}) was found and stored."
    
    if "Yellow buoy detected" in msg:
        if "stopping survey" in msg:
            return "A yellow buoy was detected, survey stopped."
        return "A yellow buoy was detected."
    
    if "Non-yellow buoy detected" in msg or "continuing survey" in msg:
        return "A non-target buoy was detected, continuing survey."
    
    if "NEW BUOY FOUND" in msg:
        return "A new buoy was found."
    
    if "SECOND BUOY FOUNDED" in msg:
        return "Second buoy found."
    
    if "Gate detected" in msg:
        if "stopping survey" in msg:
            return "Gate detected, survey stopped."
        return "Gate detected."
    
    # ============ SURVEY OPERATIONS ============
    if "SURVEY RUNNING" in msg:
        return "Survey operations are underway."
    
    if "SURVEY COMPLETED" in msg:
        return "Survey was completed successfully."
    
    if "SPIRAL SURVEY RUNNING" in msg:
        return "Spiral survey pattern is running."
    
    if "SPIRAL SURVEY COMPLETED" in msg:
        return "Spiral survey completed."
    
    if "[SURVEY_CHECK]" in msg:
        return "Survey status check performed."
    
    # ============ WAYPOINT NAVIGATION ============
    if "GOING TO WP" in msg:
        return "Navigating to waypoint."
    
    if "WP REACHED" in msg:
        return "Waypoint reached."
    
    # ============ GATE OPERATIONS ============
    if "APPROACHING GATE" in msg:
        return "Approaching the gate."
    
    if "READY TO GO OVER THE GATE" in msg:
        return "Ready to pass through the gate."
    
    if "PASSING GATE" in msg:
        return "Passing through the gate."
    
    if "GATE PASSED" in msg:
        return "Gate passage confirmed."
    
    if "GATE FINDER" in msg:
        return "Searching for gate."
    
    # ============ MOVE OPERATIONS (TAG 6) ============
    if "STARTING MOVE MISSION" in msg:
        return "Move mission started."
    
    if "Checking mission memory for buoy" in msg:
        return "Checking mission memory for buoy information."
    
    if "Loaded" in msg and "buoys from mission memory" in msg:
        count = re.search(r'Loaded\s+(\d+)', msg)
        return f"Loaded {count.group(1)} buoys from mission memory." if count else "Loaded buoys from mission memory."
    
    if "Buoy of color" in msg and "found in memory" in msg:
        color = re.search(r"color\s+'?([a-z]+)'?", msg)
        name = re.search(r'memory:\s+([a-zA-Z0-9_]+)', msg)
        if color and name:
            return f"Buoy {name.group(1)} ({color.group(1)}) position loaded from memory."
        return "Buoy position loaded from memory."
    
    if "Loaded buoy" in msg and "from memory" in msg:
        match = re.search(r"Loaded buoy\s+'?([a-zA-Z0-9_]+)'?", msg)
        return f"Buoy {match.group(1)} position loaded from memory." if match else "Buoy position loaded from memory."
    
    if "Executing" in msg and ("pattern" in msg or "move" in msg):
        pat = re.search(r"Executing\s+([a-zA-Z0-9_\s]+)", msg)
        return f"Executing maneuver '{pat.group(1).strip()}'." if pat else "Executing maneuver."
    
    if "Cardinal points calculated" in msg:
        return "Cardinal navigation points calculated."
    
    if "Cardinal point reached" in msg or "[TAG 6]" in msg and "point reached" in msg:
        pt = re.search(r'reached:\s*([0-9]+/4)', msg)
        return f"Cardinal navigation point {pt.group(1)} reached." if pt else "A cardinal navigation point reached."
    
    if "Moving to" in msg and ("NORTH" in msg or "EAST" in msg or "SOUTH" in msg or "WEST" in msg):
        direction = re.search(r'Moving to\s+([A-Z]+)', msg)
        return f"Moving to {direction.group(1).lower()} point." if direction else "Moving to cardinal point."
    
    if "waypoint reached" in msg and ("NORTH" in msg or "EAST" in msg or "SOUTH" in msg or "WEST" in msg):
        direction = re.search(r'([A-Z]+)\s+waypoint', msg)
        return f"The {direction.group(1).lower()} waypoint was reached." if direction else "Cardinal waypoint reached."
    
    if "All cardinal points completed" in msg:
        return "All cardinal navigation points completed."

    # ============ TAG 6 CARDINAL NAVIGATION ============

    if "Cardinal point reached" in msg:
        match = re.search(r'Cardinal point reached:\s*(\d+)/4', msg)
        if match:
            return f"Cardinal point reached: {match.group(1)}/4"

    if "All cardinal points completed" in msg:
        return "All cardinal points completed - mission success"

    if "Target buoy color" in msg and "proceeding to Move" in msg:
        color_match = re.search(r'Target buoy color\s+(\w+)\s+found', msg)
        if color_match:
            return f"Target buoy ({color_match.group(1)}) found - moving to coordinates"

    if "Stop message sent for Move mission" in msg:
        return "Stop command issued - Move guidance stopped"

    if "Reset completed_counter" in msg:
        return "Move mission counter reset for next operation"

    # ============ TAG 5 BUOY DETECTION REFINEMENT ============

    if "Target buoy (color:" in msg and "stopping spiral survey" in msg:
        color_match = re.search(r"color:\s*(\w+)", msg)
        if color_match:
            return f"Target buoy ({color_match.group(1)}) found - stopping spiral survey"

    if "Wrong color detected" in msg and "continuing spiral survey" in msg:
        wrong_color = re.search(r"Wrong color detected \((\w+)", msg)
        needed_color = re.search(r"needed\s+(\w+)", msg)
        if wrong_color and needed_color:
            return f"Wrong buoy color ({wrong_color.group(1)}) - continuing search for {needed_color.group(1)}"

    if "Stop message sent for FindBuoy spiral mission" in msg:
        return "Stop command issued - FindBuoy spiral survey completed"


    # ============================================================================
    # MAP AREA OPERATIONS (TAG 7)
    # ============================================================================
    if "SYSTEM SETTING MAP AREA" in msg:
        return "Configuring map area survey mission."
        
    if "GO LAWN MOWER SURVEY FOR MAP AREA" in msg:
        return "Starting lawn mower survey pattern for area mapping."
        
    if "Survey area received" in msg:
        match = re.search(r'lat=([\d.]+), lon=([\d.]+), radius=([\d.]+)', msg)
        if match:
            return f"Survey area confirmed at coordinates (lat: {match.group(1)}, lon: {match.group(2)}) with radius {match.group(3)}m."
        return "Survey area parameters received."

    if "MapArea SUCCESS" in msg or "All required buoys" in msg and "found in mission memory" in msg:
        return "Map area mission completed successfully - all required buoys found."
        
    if "MapArea INCOMPLETE" in msg or "Not all buoys found" in msg:
        return "Map area mission incomplete - some required buoys were not detected."
        
    if "MapArea:" in msg and "buoy found in memory" in msg:
        # Extract buoy details
        color_match = re.search(r'MapArea: (\w+) buoy found', msg)
        buoy_match = re.search(r'memory - ([\w_]+)', msg)
        if color_match and buoy_match:
            return f"{color_match.group(1).capitalize()} buoy ({buoy_match.group(1)}) position loaded from memory."
        return "Required buoy position loaded from memory."
        
    if "Loaded" in msg and "buoys from mission memory" in msg:
        count_match = re.search(r'Loaded (\d+) buoys', msg)
        if count_match:
            count = count_match.group(1)
            return f"Retrieved {count} buoy positions from mission memory."
        return "Buoy positions retrieved from mission memory."

    if "Updated discovered buoys" in msg:
        match = re.search(r'(\d+) -> (\d+)', msg)
        if match:
            return f"Buoy discovery count updated from {match.group(1)} to {match.group(2)}."
        return "Buoy discovery count updated."


    
    # ============ STRATEGY & PATTERNS ============
    if "STRATEGY" in msg or "LAWN MOVER" in msg:
        return "Using lawn mower search pattern."
    
    if "SPIRAL" in msg and ("strategy" in msg.lower() or "pattern" in msg.lower()):
        return "Using spiral search pattern."
    
    if "READY TO SEARCH BUOYS" in msg:
        return "Ready to search for buoys."
    
    if "SEARCHING 2nd BUOY" in msg or "2nd BUOY" in msg:
        return "Searching for second buoy."
    
    # ============ MISSION STATUS ============
    if "[MISSION STATUS]" in msg:
        if "COMPLETED" in msg:
            counter = re.search(r'Mission\s+#?(\d+)', msg)
            return f"Mission {counter.group(1)} was completed successfully." if counter else "The mission was completed successfully."
        if "FAILED" in msg:
            return "The mission failed."
        if "ABORTED" in msg:
            return "The mission was aborted."
    
    if "Mission completed" in msg or "COMPLETED" in msg and "mission" in msg.lower():
        return "The mission was completed successfully."
    
    if "Mission aborted" in msg or "ABORTED" in msg:
        return "The mission was aborted."
    
    # ============ STAND BY & WAITING ============
    if "STAND BY" in msg or "stand by" in msg.lower():
        return "System is standing by."
    
    if "WAITING MISSION TAG" in msg:
        return "Waiting for mission assignment."
    
    # ============ SYSTEM SETTING & PREPARATION ============
    if "SYSTEM SETTING" in msg:
        return "System is configuring parameters."
    
    if "GO SURVEY" in msg:
        return "Initiating survey operations."
    
    if "Go to WP" in msg or "GO IN FRONT OF GATE" in msg:
        return "Moving to waypoint."
    
    if "GO OVER GATE" in msg:
        return "Passing over the gate."
    
    # ============ ERRORS & FAILURES ============
    if "GUIDANCE MANAGER ERROR" in msg:
        return "Guidance system encountered an error."
    
    if "ERROR" in msg.upper():
        return "An error occurred in the system."
    
    # ============ STOP & RESET ============
    if "ZENO STOP" in msg or "Stop message sent" in msg:
        return "System stop command issued."
    
    if "Reset" in msg and "mission" in msg.lower():
        return "Mission reset initiated."
    
    # ============ GENERIC TAG-SPECIFIC ============
    if "[TAG 5]" in msg or "TAG 5" in msg:
        if "Target buoy" in msg and "found" in msg:
            return "Target buoy found during search."
        if "Wrong color detected" in msg:
            return "Non-target buoy detected, continuing search."
    
    if "[TAG 6]" in msg:
        if "Reset completed_counter" in msg:
            return "Move mission counter reset for new operation."
    
    # ============ POSITION & DEBUGGING ============
    if "Publishing" in msg:
        return "Command published to system."
    
    # ============ DEFAULT/FALLBACK ============
    # Clean up brackets and technical markers
    readable = re.sub(r'[\[\]\(\)]+', '', msg)
    readable = re.sub(r'[🔧🧪💤⏳]+', '', readable)  # Remove emojis
    readable = readable.strip()
    
    # If still too technical or empty, return generic
    if not readable or len(readable) < 3:
        return "Status update."
    
    return readable.capitalize() if readable[0].islower() else readable


def preprocess_logs(logs: List[Dict]) -> List[str]:
    """Preprocess list of log entries to natural language."""
    return [preprocess_message(log['message']) for log in logs]


def extract_mission_events(mission_logs_file: Path = MISSION_LOGS_FILE, 
                           mission_counter: int = None) -> str:
    """
    Extract formatted events for a specific mission from mission logs.
    
    Args:
        mission_logs_file: Path to mission_logs.yaml
        mission_counter: Mission counter to extract events for
        
    Returns:
        String with events separated by ' | ', or empty string if not found
    """
    if not mission_logs_file.exists():
        return ""
    
    try:
        with open(mission_logs_file, 'r') as f:
            logs_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading mission logs: {e}")
        return ""
    
    if not logs_data or 'missions' not in logs_data:
        return ""
    
    # Find mission by counter
    for mission in logs_data.get("missions", []):
        if mission.get("mission_counter") == mission_counter:
            logs = mission.get("logs", [])
            events = preprocess_logs(logs)
            return " | ".join(events)
    
    return ""


def get_all_mission_events(mission_logs_file: Path = MISSION_LOGS_FILE) -> Dict[int, str]:
    """
    Extract events for all missions in the log file.
    
    Args:
        mission_logs_file: Path to mission_logs.yaml
        
    Returns:
        Dictionary mapping mission_counter to events string
    """
    if not mission_logs_file.exists():
        return {}
    
    try:
        with open(mission_logs_file, 'r') as f:
            logs_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading mission logs: {e}")
        return {}
    
    if not logs_data or 'missions' not in logs_data:
        return {}
    
    all_events = {}
    for mission in logs_data.get("missions", []):
        mission_counter = mission.get("mission_counter")
        if mission_counter is not None:
            logs = mission.get("logs", [])
            events = preprocess_logs(logs)
            all_events[mission_counter] = " | ".join(events)
    
    return all_events


# For backwards compatibility with supervisor_large imports
def reload_supervisor_logs():
    """Deprecated: Kept for backwards compatibility"""
    pass


def get_supervisor_stats() -> dict:
    """Deprecated: Kept for backwards compatibility"""
    return {"status": "logs_processor - no stats available"}


if __name__ == "__main__":
    # Test the processor
    print("Mission Logs Processor - Test Mode")
    print("=" * 60)
    
    # Test with mission_logs.yaml if it exists
    if MISSION_LOGS_FILE.exists():
        print(f"\nLoading logs from: {MISSION_LOGS_FILE}")
        all_events = get_all_mission_events()
        
        if all_events:
            for counter, events in all_events.items():
                print(f"\n[Mission #{counter}]")
                print(events)
        else:
            print("No missions found in log file")
    else:
        print(f"\nLog file not found: {MISSION_LOGS_FILE}")

