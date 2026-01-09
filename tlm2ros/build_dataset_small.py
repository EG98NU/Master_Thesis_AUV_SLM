#!/usr/bin/env python3

"""
Build fine-tuning dataset for ROS mission planner + test
Generates training examples with buoy memory and mission history
"""

import json
import random
import yaml
from typing import List, Dict, Any, Optional
from collections import defaultdict, Counter
from pathlib import Path

# Load missions from missions.yaml
MISSIONS_FILE = Path("missions.yaml")

def load_missions():
    """Load mission names from missions.yaml"""
    try:
        with open(MISSIONS_FILE, 'r') as f:
            missions_data = yaml.safe_load(f)
        return [k for k in missions_data.keys() if k != 'default_mission' and k != 'quadrant_radius']
    except Exception as e:
        print(f"Error loading missions: {e}")
        return []

# Available missions - loaded dynamically
AVAILABLE_MISSIONS = load_missions()

# Buoys present in the map
AVAILABLE_BUOYS = ["yellow", "yellow", "red", "black", "white"]  # 2 yellows, 1 red, 1 black, 1 white
UNIQUE_BUOY_COLORS = list(set(AVAILABLE_BUOYS))  # ["yellow", "red", "black", "white"]

# Max number of unique buoys and missions that can appear
MAX_UNIQUE_BUOYS = len(UNIQUE_BUOY_COLORS)  # 4
MAX_MISSIONS_IN_HISTORY = len(AVAILABLE_MISSIONS)  # Total missions available

def memory(required_buoys: Optional[List[str]] = None,
           excluded_buoys: Optional[List[str]] = None,
           total_buoys: Optional[int] = None,
           required_missions: Optional[List[str]] = None,
           excluded_missions: Optional[List[str]] = None,
           total_missions: Optional[int] = None) -> str:
    """
    Generate memory field for the prompt, including required and excluded buoys/missions.
    
    Args:
        required_buoys: List of specific buoy colors that must be included (e.g., ["red", "yellow"])
        excluded_buoys: List of specific buoy colors that must NOT be included
        total_buoys: Total number of buoys in memory. If None, random between 1 and MAX_UNIQUE_BUOYS
        required_missions: List of specific missions that must be included
        excluded_missions: List of specific missions that must NOT be included
        total_missions: Total number of missions in history. If None, random between 1 and MAX_MISSIONS_IN_HISTORY
    
    Returns:
        Memory string formatted as: "Buoys found so far: ... | Previous missions: ..."
    """
    memory_parts = []

    # --- Determine total counts ---
    if total_buoys is None:
        total_buoys = random.randint(0, MAX_UNIQUE_BUOYS)

    if total_missions is None:
        # total_missions = random.randint(0, MAX_MISSIONS_IN_HISTORY)
        total_missions = random.randint(0, 5)

    # --- Part 1: Buoys found ---
    if total_buoys > 0:
        buoys_found = []

        # Start with required buoys if provided
        if required_buoys:
            # Filter out excluded buoys from required buoys
            valid_required = [
                b for b in required_buoys 
                if not excluded_buoys or b not in excluded_buoys
            ]
            buoys_found.extend(valid_required)
            remaining_count = total_buoys - len(valid_required)
        else:
            remaining_count = total_buoys

        # Build pool excluding required and excluded buoys
        available_pool = [
            b for b in UNIQUE_BUOY_COLORS
            if (not excluded_buoys or b not in excluded_buoys)
            and (not required_buoys or b not in required_buoys)
        ]

        # Fill remaining slots with random buoys from available pool
        if remaining_count > 0 and available_pool:
            random_buoys = random.sample(available_pool, min(remaining_count, len(available_pool)))
            buoys_found.extend(random_buoys)

        # Randomize order for natural variation
        random.shuffle(buoys_found)

        if buoys_found:
            memory_parts.append(f"Buoys found so far: {', '.join(buoys_found)}")

    # --- Part 2: Missions history ---
    if total_missions > 0:
        missions_history = []

        # Start with required missions if provided
        if required_missions:
            valid_required = [m for m in required_missions if not excluded_missions or m not in excluded_missions]
            missions_history.extend(valid_required)
            remaining_count = total_missions - len(valid_required)
        else:
            remaining_count = total_missions

        # Build pool excluding required + excluded missions
        available_pool = [
            m for m in AVAILABLE_MISSIONS
            if (not required_missions or m not in required_missions)
            and (not excluded_missions or m not in excluded_missions)
        ]

        # Fill remaining slots
        if remaining_count > 0 and available_pool:
            random_missions = random.sample(available_pool, min(remaining_count, len(available_pool)))
            missions_history.extend(random_missions)

        # Keep unique missions only
        seen = set()
        unique_missions = []
        for m in missions_history:
            if m not in seen:
                unique_missions.append(m)
                seen.add(m)

        if unique_missions:
            memory_parts.append(f"Previous missions: {', '.join(unique_missions)}")

    # --- Combine results ---
    if memory_parts:
        return " | ".join(memory_parts)
    return ""

# ========================================
# FAILURE CONTEXT TEMPLATES
# ========================================

# Template strings for different failure types
FAILURE_CONTEXT_TEMPLATES = {
    # "find": "Failure context: Mission 'map buoy area A' failed.\nMission 1 started. | Survey operations are underway. | System is configuring parameters. | Initiating survey operations. | Guidance is ready. | System and guidance are ready. | Survey operations are underway. | Survey was completed successfully. | Mission FAILED - target buoy NOT found | The mission failed.",
    "find": "Mission 'map buoy area A' failed.\nMission 1 started. | Survey operations are underway. | Configuring map area survey mission. | Starting lawn mower survey pattern for area mapping. | Guidance is ready. | System and guidance are ready. | Survey operations are underway. | Survey was completed successfully. | Loaded 0 buoys from mission memory. | Map area mission incomplete - some required buoys were not detected. | Mission FAILED - Not all required buoys found in memory | The mission failed.",
    "move": "Mission 'make move black A' failed.\nMission 1 started. | Mission reset initiated. | Move mission started. | Checking mission memory for buoy information. | Loaded 0 buoys from mission memory. | Buoy position loaded from memory. | Buoy not in memory - executing FindBuoy mission... | Survey operations are underway. | System is configuring parameters. | Initiating survey operations. | Guidance is ready. | System and guidance are ready. | Survey operations are underway. | Survey was completed successfully. | Mission FAILED - target buoy NOT found | The mission failed. Remaining missions were: make move red A, make move white A."
}


def get_failure_context(failure_type: str) -> str:
    """
    Get the failure context string for a given failure type.
    
    Args:
        failure_type (str): Type of failure - either "find" or "move"
    
    Returns:
        str: The corresponding failure context template string
    
    Raises:
        ValueError: If failure_type is not in the predefined templates
    """
    if failure_type not in FAILURE_CONTEXT_TEMPLATES:
        raise ValueError(
            f"Unknown failure type: '{failure_type}'. "
            f"Available types: {', '.join(FAILURE_CONTEXT_TEMPLATES.keys())}"
        )
    
    return FAILURE_CONTEXT_TEMPLATES[failure_type]



# ========================================
# GENERATION FUNCTIONS
# ========================================

def generate_command_variations(base_example: Dict[str, Any],
                               variations: List[str],
                               test_variations: List[str]) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Generate multiple examples with different commands - returns (train, test) examples"""
    train_examples = []
    test_examples = []
    
    for variant in variations:
        new_example = {
            "memory": base_example["memory"],
            "command": variant,
            "output": base_example["output"]
        }
        train_examples.append(new_example)
    
    for variant in test_variations:
        new_example = {
            "memory": base_example["memory"],
            "command": variant,
            "output": base_example["output"]
        }
        test_examples.append(new_example)
    
    return train_examples, test_examples


def generate_cmd_cla_variations(base_example: Dict[str, Any],
                               command_variations: List[str],
                               test_command_variations: List[str],
                               clarification_variations: List[str],
                               failure_context: Optional[str] = None) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Generate examples with variations for both commands and clarifications.
    Optionally includes failure_context if the clarification follows a mission failure.
    
    Args:
        base_example: Dict containing memory, previous_response, output
        command_variations: List of command strings for training examples
        test_command_variations: List of command strings for test examples
        clarification_variations: List of clarification strings for both datasets
        failure_context: Optional failure context string (included if clarification is after failure)
    
    Returns:
        Tuple of (train_examples, test_examples)
    """
    train_examples = []
    test_examples = []
    
    # Training examples
    for cmd in command_variations:
        for clar in clarification_variations:
            new_example = {
                "memory": base_example["memory"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "clarification": clar,
                "output": base_example["output"]
            }
            # Add failure_context if provided
            if failure_context is not None:
                new_example["failure_context"] = failure_context
            
            train_examples.append(new_example)
    
    # Test examples
    for cmd in test_command_variations:
        for clar in clarification_variations:
            new_example = {
                "memory": base_example["memory"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "clarification": clar,
                "output": base_example["output"]
            }
            # Add failure_context if provided
            if failure_context is not None:
                new_example["failure_context"] = failure_context
            
            test_examples.append(new_example)
    
    return train_examples, test_examples



def generate_failure_variations(base_example: Dict[str, Any],
                               command_variations: List[str],
                               test_command_variations: List[str]) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Generate examples with failure context - returns (train, test) examples.
    
    The failure_context is taken directly from base_example["failure_context"],
    which should be set using get_failure_context() when creating the base.
    
    Args:
        base_example: Dict containing memory, output, and failure_context
        command_variations: List of command strings for training examples
        test_command_variations: List of command strings for test examples
    
    Returns:
        Tuple of (train_examples, test_examples)
    """
    train_examples = []
    test_examples = []
    
    # Get failure context from base example
    failure_context = base_example["failure_context"]
    
    # Training examples
    for cmd in command_variations:
        new_example = {
            "memory": base_example["memory"],
            "command": cmd,
            "failure_context": failure_context,
            "output": base_example["output"]
        }
        train_examples.append(new_example)
    
    # Test examples
    for cmd in test_command_variations:
        new_example = {
            "memory": base_example["memory"],
            "command": cmd,
            "failure_context": failure_context,
            "output": base_example["output"]
        }
        test_examples.append(new_example)
    
    return train_examples, test_examples

def generate_cmd_dup_variations(base_example: Dict[str, Any], 
                                command_variations: List[str], 
                                test_command_variations: List[str],
                                duplication_variations: List[str],
                                failure_context: Optional[str] = None) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Generate examples with variations for both commands and duplications.
    Optionally includes failure_context if the duplication follows a mission failure.
    
    Args:
        base_example: Dict containing memory, previous_response, output
        command_variations: List of command strings for training examples
        test_command_variations: List of command strings for test examples
        duplication_variations: List of duplication strings for both datasets
        failure_context: Optional failure context string included if duplication is after failure
    
    Returns:
        Tuple of (train_examples, test_examples)
    """
    train_examples = []
    test_examples = []
    
    # Training examples
    for cmd in command_variations:
        for dup in duplication_variations:
            new_example = {
                "memory": base_example["memory"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "duplication": dup,
                "output": base_example["output"]
            }
            if failure_context is not None:
                new_example["failure_context"] = failure_context
            train_examples.append(new_example)
    
    # Test examples
    for cmd in test_command_variations:
        for dup in duplication_variations:
            new_example = {
                "memory": base_example["memory"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "duplication": dup,
                "output": base_example["output"]
            }
            if failure_context is not None:
                new_example["failure_context"] = failure_context
            test_examples.append(new_example)
    
    return train_examples, test_examples


# ========================================
# PROMPT LINES
# ========================================

MISSIONS_LINE = f"Available missions: skip, {', '.join(AVAILABLE_MISSIONS)}"

FINAL_LINE = "Instruction: Given the command and the mission memory, generate one or more missions from the available missions. If multiple missions are needed, separate them with commas."

FINAL_LINE_C = "Instruction: Given the command, mission memory, and clarification, generate one or more missions from the available missions. If multiple missions are needed, separate them with commas."

FINAL_LINE_F = FINAL_LINE

FINAL_LINE_D = FINAL_LINE
# ========================================
# DATASET TEMPLATES
# ========================================

# ============================================================================================
#                                   REGULAR EXAMPLES
# ============================================================================================

command_variations_data = [

    ###########################################################################################
    #                                   GOAL MISSIONS
    ###########################################################################################
    {
        "base": { # Goal North-West
            "memory": memory(excluded_missions="go to NW goal"),
            "command": "go to northwest area",
            "output": "go to NW goal"
        },
        "variations": [
            "go to northwest area", #1
            "move to northwest", #2
            "navigate to NW sector", #3
            "head to northwest zone", #4
            "reach the center of the NW area", #5
            "proceed to the center of the north-western area", #6
            "set route to the northwestern zone", #7
            "place a goal waypoint in the NW area", #8
            "set the target waypoint in the northwest sector", #9
            "move to the center of the NW zone", #10
            "navigate toward the middle of the northwest quadrant", #11
            "travel to the northwest region", #12
            "advance to the NW sector", #13
            "head toward the northwestern area", #14
            "reach northwest zone", #15
            "proceed to northwest", #16
            "set destination to the NW area", #17
            "place waypoint in the northwestern sector", #18
            "set goal in the northwest quadrant", #19
            "move toward NW", #20
            "navigate to the center of northwest", #21
            "go to the NW zone", #22
            "journey to the northwest area", #23
            "direct path to the northwestern sector", #24
            "aim for the NW region", #25
            "steer toward northwest zone", #26
            "move to middle of the NW area", #27
            "navigate to the northwestern quadrant", #28
            "head to the center NW sector", #29
            "reach toward the northwest area", #30
            "proceed northwest", #31
            "set route to northwest area", #32
            "place target in NW zone", #33
            "go toward the north-western sector", #34
            "move to the NW quadrant", #35
            "navigate northwest zone", #36
            "head to center of NW", #37
            "reach the northwest sector", #38
            "proceed to the northwestern area", #39
            "set waypoint in northwest quadrant", #40
            "advance to northwest area", #41
            "travel to the NW sector", #42
            "go to the northwestern region", #43
            "move to middle northwest", #44
            "navigate to center NW zone", #45
            "head toward the NW quadrant", #46
            "reach middle of northwest area", #47
            "proceed to the NW sector", #48
            "set goal waypoint northwest", #49
            "place objective in the NW area", #50
            "go to north-west zone", #51
            "move to the northwestern area", #52
            "navigate toward NW sector", #53
            "head to the northwest quadrant", #54
            "reach center northwest zone", #55
            "proceed to middle NW area", #56
            "set target to the northwestern sector", #57
            "place waypoint at northwest", #58
            "go toward northwest region", #59
            "move to center of northwestern area", #60
            "navigate to the NW region", #61
            "head to northwest sector", #62
            "reach the NW quadrant", #63
            "proceed to center of NW sector", #64
            "set route toward the northwest area", #65
            "place goal in northwest zone", #66
            "advance toward the NW area", #67
            "travel northwest sector", #68
            "go to the center northwest sector", #69
            "move toward the northwestern zone", #70
            "navigate to northwest area center", #71
            "head toward middle NW sector", #72
            "reach the northwestern quadrant", #73
            "proceed to the north-western zone", #74
            "set waypoint to NW area", #75
            "place target northwest", #76
            "go to middle of the NW sector", #77
            "move to the NW region", #78
            "navigate northwest quadrant", #79
            "head to the NW area", #80
            "reach northwest center", #81
            "proceed toward the northwestern area", #82
            "set destination northwest zone", #83
            "place waypoint in the NW quadrant", #84
            "advance to the northwestern zone", #85
            "travel to northwest", #86
            "go toward the NW zone", #87
            "move to northwestern sector", #88
            "navigate to the center northwest", #89
            "head to the center of the NW area", #90
            "reach toward NW sector", #91
            "proceed to northwest quadrant", #92
            "set goal in the NW zone", #93
            "place objective at the NW area", #94
            "go to the north-western area", #95
            "move toward center NW zone", #96
            "navigate toward the northwestern region", #97
            "head to northwest center", #98
            "reach the middle of NW sector", #99
            "proceed to center NW zone" #100
            ],
        "test_variations": [
            "navigate toward the middle of the north-western sector", #1
            "set waypoint to the center of northwest zone", #2
            "head to the middle NW area and proceed", #3
            "advance to the center of the north-western quadrant", #4
            "move toward middle of the NW sector region", #5
            "place a target in the center northwest zone", #6
            "reach the middle point of the north-western area", #7
            "navigate to center of the NW quadrant region", #8
            "go toward the middle northwestern sector area", #9
            "set destination to center of NW zone" #10
            ]


    },
    {
        "base": { # Goal North-East
            "memory": memory(excluded_missions="go to NE goal"),
            "command": "go to northeast area",
            "output": "go to NE goal"
        },
        "variations": [
            "go to northeast area", #1
            "move to northeast", #2
            "navigate to NE sector", #3
            "head to northeast zone", #4
            "reach the center of the NE area", #5
            "proceed to the center of the north-eastern area", #6
            "set route to the northeastern zone", #7
            "place a goal waypoint in the NE area", #8
            "set the target waypoint in the northeast sector", #9
            "move to the center of the NE zone", #10
            "navigate toward the middle of the northeast quadrant", #11
            "plot a waypoint at the northeast center", #12
            "head for the center of the northeast region", #13
            "assign waypoint to the northeast central area", #14
            "move to the central point of the NE section", #15
            "direct the vehicle to the northeast central point", #16
            "send the vehicle to the center of the northeast sector", #17
            "define a waypoint in the northeastern center", #18
            "mark the central coordinate of the NE zone", #19
            "set navigation point at the northeast midpoint", #20
            "guide the vehicle to the core of the northeast sector", #21
            "travel to the northeast area", #22
            "advance to the NE sector", #23
            "head toward the northeastern area", #24
            "reach northeast zone", #25
            "proceed to northeast", #26
            "set destination to the NE area", #27
            "place waypoint in the northeastern sector", #28
            "set goal in the northeast quadrant", #29
            "move toward NE", #30
            "navigate to the center of northeast", #31
            "go to the NE zone", #32
            "journey to the northeast area", #33
            "direct path to the northeastern sector", #34
            "aim for the NE region", #35
            "steer toward northeast zone", #36
            "move to middle of the NE area", #37
            "navigate to the northeastern quadrant", #38
            "head to the center NE sector", #39
            "reach toward the northeast area", #40
            "proceed northeast", #41
            "set route to northeast area", #42
            "place target in NE zone", #43
            "go toward the north-eastern sector", #44
            "move to the NE quadrant", #45
            "navigate northeast zone", #46
            "head to center of NE", #47
            "reach the northeast sector", #48
            "proceed to the northeastern area", #49
            "set waypoint in northeast quadrant", #50
            "advance to northeast area", #51
            "travel to the NE sector", #52
            "go to the northeastern region", #53
            "move to middle northeast", #54
            "navigate to center NE zone", #55
            "head toward the NE quadrant", #56
            "reach middle of northeast area", #57
            "proceed to the NE sector", #58
            "set goal waypoint northeast", #59
            "place objective in the NE area", #60
            "go to north-east zone", #61
            "move to the northeastern area", #62
            "navigate toward NE sector", #63
            "head to the northeast quadrant", #64
            "reach center northeast zone", #65
            "proceed to middle NE area", #66
            "set target to the northeastern sector", #67
            "place waypoint at northeast", #68
            "go toward northeast region", #69
            "move to center of northeastern area", #70
            "navigate to the NE region", #71
            "head to northeast sector", #72
            "reach the NE quadrant", #73
            "proceed to center of NE sector", #74
            "set route toward the northeast area", #75
            "place goal in northeast zone", #76
            "advance toward the NE area", #77
            "travel northeast sector", #78
            "go to the center northeast sector", #79
            "move toward the northeastern zone", #80
            "navigate to northeast area center", #81
            "head toward middle NE sector", #82
            "reach the northeastern quadrant", #83
            "proceed to the north-eastern zone", #84
            "set waypoint to NE area", #85
            "place target northeast", #86
            "go to middle of the NE sector", #87
            "move to the NE region", #88
            "navigate northeast quadrant", #89
            "head to the NE area", #90
            "reach northeast center", #91
            "proceed toward the northeastern area", #92
            "set destination northeast zone", #93
            "place waypoint in the NE quadrant", #94
            "advance to the northeastern zone", #95
            "travel to northeast", #96
            "go toward the NE zone", #97
            "move to northeastern sector", #98
            "navigate to the center northeast", #99
            "head to the center of the NE area" #100
            ],
        "test_variations": [
            "navigate toward the middle of the north-eastern sector", #1
            "set waypoint to the center of northeast zone", #2
            "head to the middle NE area and proceed", #3
            "advance to the center of the north-eastern quadrant", #4
            "move toward middle of the NE sector region", #5
            "place a target in the center northeast zone", #6
            "reach the middle point of the north-eastern area", #7
            "navigate to center of the NE quadrant region", #8
            "go toward the middle northeastern sector area", #9
            "set destination to center of NE zone" #10
            ]


    },
    {
        "base": { # Goal South-West
            "memory": memory(excluded_missions="go to SW goal"),
            "command": "go to southwest area",
            "output": "go to SW goal"
        },
        "variations": [
            "go to southwest area", #1
            "move to southwest", #2
            "navigate to SW sector", #3
            "head to southwest zone", #4
            "reach the center of the SW area", #5
            "proceed to the center of the south-western area", #6
            "set route to the southwestern zone", #7
            "place a goal waypoint in the SW area", #8
            "set the target waypoint in the southwest sector", #9
            "move to the center of the SW zone", #10
            "navigate toward the middle of the southwest quadrant", #11
            "plot a waypoint at the southwest center", #12
            "head for the center of the southwest region", #13
            "assign waypoint to the southwest central area", #14
            "move to the central point of the SW section", #15
            "direct the vehicle to the southwest central point", #16
            "send the vehicle to the center of the southwest sector", #17
            "define a waypoint in the southwestern center", #18
            "mark the central coordinate of the SW zone", #19
            "set navigation point at the southwest midpoint", #20
            "guide the vehicle to the core of the southwest sector", #21
            "travel to the southwest area", #22
            "advance to the SW sector", #23
            "head toward the southwestern area", #24
            "reach southwest zone", #25
            "proceed to southwest", #26
            "set destination to the SW area", #27
            "place waypoint in the southwestern sector", #28
            "set goal in the southwest quadrant", #29
            "move toward SW", #30
            "navigate to the center of southwest", #31
            "go to the SW zone", #32
            "journey to the southwest area", #33
            "direct path to the southwestern sector", #34
            "aim for the SW region", #35
            "steer toward southwest zone", #36
            "move to middle of the SW area", #37
            "navigate to the southwestern quadrant", #38
            "head to the center SW sector", #39
            "reach toward the southwest area", #40
            "proceed southwest", #41
            "set route to southwest area", #42
            "place target in SW zone", #43
            "go toward the south-western sector", #44
            "move to the SW quadrant", #45
            "navigate southwest zone", #46
            "head to center of SW", #47
            "reach the southwest sector", #48
            "proceed to the southwestern area", #49
            "set waypoint in southwest quadrant", #50
            "advance to southwest area", #51
            "travel to the SW sector", #52
            "go to the southwestern region", #53
            "move to middle southwest", #54
            "navigate to center SW zone", #55
            "head toward the SW quadrant", #56
            "reach middle of southwest area", #57
            "proceed to the SW sector", #58
            "set goal waypoint southwest", #59
            "place objective in the SW area", #60
            "go to south-west zone", #61
            "move to the southwestern area", #62
            "navigate toward SW sector", #63
            "head to the southwest quadrant", #64
            "reach center southwest zone", #65
            "proceed to middle SW area", #66
            "set target to the southwestern sector", #67
            "place waypoint at southwest", #68
            "go toward southwest region", #69
            "move to center of southwestern area", #70
            "navigate to the SW region", #71
            "head to southwest sector", #72
            "reach the SW quadrant", #73
            "proceed to center of SW sector", #74
            "set route toward the southwest area", #75
            "place goal in southwest zone", #76
            "advance toward the SW area", #77
            "travel southwest sector", #78
            "go to the center southwest sector", #79
            "move toward the southwestern zone", #80
            "navigate to southwest area center", #81
            "head toward middle SW sector", #82
            "reach the southwestern quadrant", #83
            "proceed to the south-western zone", #84
            "set waypoint to SW area", #85
            "place target southwest", #86
            "go to middle of the SW sector", #87
            "move to the SW region", #88
            "navigate southwest quadrant", #89
            "head to the SW area", #90
            "reach southwest center", #91
            "proceed toward the southwestern area", #92
            "set destination southwest zone", #93
            "place waypoint in the SW quadrant", #94
            "advance to the southwestern zone", #95
            "travel to southwest", #96
            "go toward the SW zone", #97
            "move to southwestern sector", #98
            "navigate to the center southwest", #99
            "head to the center of the SW area" #100
            ],
        "test_variations": [
            "navigate toward the middle of the south-western sector", #1
            "set waypoint to the center of southwest zone", #2
            "head to the middle SW area and proceed", #3
            "advance to the center of the south-western quadrant", #4
            "move toward middle of the SW sector region", #5
            "place a target in the center southwest zone", #6
            "reach the middle point of the south-western area", #7
            "navigate to center of the SW quadrant region", #8
            "go toward the middle southwestern sector area", #9
            "set destination to center of SW zone" #10
            ]


    },
    {
        "base": { # Goal South-East
            "memory": memory(excluded_missions="go to SE goal"),
            "command": "go to southeast area",
            "output": "go to SE goal"
        },
        "variations": [
            "go to southeast area", #1
            "move to southeast", #2
            "navigate to SE sector", #3
            "head to southeast zone", #4
            "reach the center of the SE area", #5
            "proceed to the center of the south-eastern area", #6
            "set route to the southeastern zone", #7
            "place a goal waypoint in the SE area", #8
            "set the target waypoint in the southeast sector", #9
            "move to the center of the SE zone", #10
            "navigate toward the middle of the southeast quadrant", #11
            "plot a waypoint at the southeast center", #12
            "head for the center of the southeast region", #13
            "assign waypoint to the southeast central area", #14
            "move to the central point of the SE section", #15
            "direct the vehicle to the southeast central point", #16
            "send the vehicle to the center of the southeast sector", #17
            "define a waypoint in the southeastern center", #18
            "mark the central coordinate of the SE zone", #19
            "set navigation point at the southeast midpoint", #20
            "guide the vehicle to the core of the southeast sector", #21
            "travel to the southeast area", #22
            "advance to the SE sector", #23
            "head toward the southeastern area", #24
            "reach southeast zone", #25
            "proceed to southeast", #26
            "set destination to the SE area", #27
            "place waypoint in the southeastern sector", #28
            "set goal in the southeast quadrant", #29
            "move toward SE", #30
            "navigate to the center of southeast", #31
            "go to the SE zone", #32
            "journey to the southeast area", #33
            "direct path to the southeastern sector", #34
            "aim for the SE region", #35
            "steer toward southeast zone", #36
            "move to middle of the SE area", #37
            "navigate to the southeastern quadrant", #38
            "head to the center SE sector", #39
            "reach toward the southeast area", #40
            "proceed southeast", #41
            "set route to southeast area", #42
            "place target in SE zone", #43
            "go toward the south-eastern sector", #44
            "move to the SE quadrant", #45
            "navigate southeast zone", #46
            "head to center of SE", #47
            "reach the southeast sector", #48
            "proceed to the southeastern area", #49
            "set waypoint in southeast quadrant", #50
            "advance to southeast area", #51
            "travel to the SE sector", #52
            "go to the southeastern region", #53
            "move to middle southeast", #54
            "navigate to center SE zone", #55
            "head toward the SE quadrant", #56
            "reach middle of southeast area", #57
            "proceed to the SE sector", #58
            "set goal waypoint southeast", #59
            "place objective in the SE area", #60
            "go to south-east zone", #61
            "move to the southeastern area", #62
            "navigate toward SE sector", #63
            "head to the southeast quadrant", #64
            "reach center southeast zone", #65
            "proceed to middle SE area", #66
            "set target to the southeastern sector", #67
            "place waypoint at southeast", #68
            "go toward southeast region", #69
            "move to center of southeastern area", #70
            "navigate to the SE region", #71
            "head to southeast sector", #72
            "reach the SE quadrant", #73
            "proceed to center of SE sector", #74
            "set route toward the southeast area", #75
            "place goal in southeast zone", #76
            "advance toward the SE area", #77
            "travel southeast sector", #78
            "go to the center southeast sector", #79
            "move toward the southeastern zone", #80
            "navigate to southeast area center", #81
            "head toward middle SE sector", #82
            "reach the southeastern quadrant", #83
            "proceed to the south-eastern zone", #84
            "set waypoint to SE area", #85
            "place target southeast", #86
            "go to middle of the SE sector", #87
            "move to the SE region", #88
            "navigate southeast quadrant", #89
            "head to the SE area", #90
            "reach southeast center", #91
            "proceed toward the southeastern area", #92
            "set destination southeast zone", #93
            "place waypoint in the SE quadrant", #94
            "advance to the southeastern zone", #95
            "travel to southeast", #96
            "go toward the SE zone", #97
            "move to southeastern sector", #98
            "navigate to the center southeast", #99
            "head to the center of the SE area" #100
            ],
        "test_variations": [
            "navigate toward the middle of the south-eastern sector", #1
            "set waypoint to the center of southeast zone", #2
            "head to the middle SE area and proceed", #3
            "advance to the center of the south-eastern quadrant", #4
            "move toward middle of the SE sector region", #5
            "place a target in the center southeast zone", #6
            "reach the middle point of the south-eastern area", #7
            "navigate to center of the SE quadrant region", #8
            "go toward the middle southeastern sector area", #9
            "set destination to center of SE zone" #10
            ]


    },
    # {
    #     "base": {
    #         "memory": memory(),
    #         "command": "",
    #         "output": ""
    #     },
    #     "variations": [
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #         "",
    #     ]
    # },
    
    ###########################################################################################
    #                                   SURVEY MISSIONS
    ###########################################################################################
    {
        "base": { # Survey North-East
            "memory": memory(excluded_missions="NE quadrant survey"),
            "command": "survey the northeast area",
            "output": "NE quadrant survey"
        },
        "variations": [
            "survey the northeast area", #1
            "scan northeast sector", #2
            "explore NE zone", #3
            "perform survey in northeast", #4
            "see what you can find in the NE area", #5
            "take a look around in the north-eastern sector", #6
            "search the northeast area", #7
            "perform a survey in the NE section", #8
            "survey the north-eastern zone", #9
            "explore the northeast area", #10
            "conduct a full survey of the NE zone", #11
            "begin scanning the entire northeast region", #12
            "execute a survey mission in the NE sector", #13
            "perform a complete scan of the north-eastern area", #14
            "carry out a survey in the northeast field", #15
            "start mapping the northeast sector", #16
            "run a scan across the northeast zone", #17
            "search and map the NE area", #18
            "inspect the northeast region thoroughly", #19
            "initiate a scan of the northeast sector", #20
            "survey the entire northeast section", #21
            "map the northeast part of the region", #22
            "examine the NE zone", #23
            "investigate the northeast area", #24
            "monitor the north-eastern sector", #25
            "analyze the NE region", #26
            "probe the northeast zone", #27
            "check the entire northeast sector", #28
            "explore and map the NE field", #29
            "scan the full northeast quadrant", #30
            "survey northeast", #31
            "scan NE", #32
            "explore northeast zone", #33
            "perform scan in the NE sector", #34
            "search northeast sector", #35
            "map the northeast zone", #36
            "inspect NE area", #37
            "execute scan in northeast", #38
            "conduct survey of the NE zone", #39
            "begin exploration of the northeast area", #40
            "start scanning northeast sector", #41
            "run survey of the north-eastern zone", #42
            "carry out mapping in northeast", #43
            "initiate exploration of the NE area", #44
            "navigate and survey the northeast sector", #45
            "perform complete survey northeast", #46
            "execute full scan of NE zone", #47
            "conduct thorough inspection of the northeast area", #48
            "begin comprehensive survey in northeast", #49
            "search the central northeast zone", #50
            "scan the middle of the NE sector", #51
            "explore the core northeast area", #52
            "map the center northeast zone", #53
            "inspect the northeast sector thoroughly", #54
            "examine the entire NE area", #55
            "investigate the north-eastern zone", #56
            "monitor the center of the northeast sector", #57
            "analyze the northeast region fully", #58
            "probe the NE zone systematically", #59
            "check the northeast area completely", #60
            "survey across the northeast sector", #61
            "scan throughout the NE zone", #62
            "explore all of northeast area", #63
            "perform survey across northeast", #64
            "search throughout the NE sector", #65
            "map across the northeast zone", #66
            "inspect the whole northeast area", #67
            "execute survey throughout northeast", #68
            "conduct scan of northeast sector", #69
            "begin thorough survey of the NE zone", #70
            "start comprehensive scan of northeast", #71
            "run full survey of the north-eastern area", #72
            "carry out complete mapping in northeast", #73
            "initiate comprehensive survey of NE", #74
            "survey the northeast region completely", #75
            "scan the full northeast zone", #76
            "explore the entire northeast sector", #77
            "map the entire NE area", #78
            "inspect the complete northeast zone", #79
            "examine the full northeast region", #80
            "investigate all of the northeast area", #81
            "monitor the entire northeast sector", #82
            "analyze the full northeast zone", #83
            "probe the entire NE area", #84
            "check the whole northeast region", #85
            "survey northeast field", #86
            "scan northeast region", #87
            "explore northeast sector", #88
            "perform northeast survey", #89
            "search northeast zone", #90
            "map northeast area", #91
            "inspect northeast zone", #92
            "execute northeast scan", #93
            "conduct northeast survey", #94
            "begin northeast exploration", #95
            "start northeast scanning", #96
            "run northeast survey", #97
            "carry out northeast mapping", #98
            "initiate northeast scan", #99
            "execute comprehensive northeast survey" #100
            ],
        "test_variations": [
            "conduct a thorough survey of the north-eastern sector region", #1
            "scan and map the entire northeast zone area", #2
            "perform a complete survey across the NE sector", #3
            "explore the northeast area with detailed scanning", #4
            "execute a comprehensive scan in the north-eastern zone", #5
            "survey the central northeast sector thoroughly", #6
            "begin full exploration and mapping of the NE area", #7
            "initiate complete scan of the northeastern region", #8
            "perform detailed survey across the northeast zone", #9
            "conduct extensive mapping of the north-eastern sector" #10
            ]


    },
    {
        "base": { # Survey North-West
            "memory": memory(excluded_missions="NW quadrant survey"),
            "command": "survey the northwest area",
            "output": "NW quadrant survey"
        },
        "variations": [
            "survey the northwest area", #1
            "scan northwest sector", #2
            "explore NW zone", #3
            "perform survey in northwest", #4
            "see what you can find in the NW area", #5
            "take a look around in the north-western sector", #6
            "search the northwest area", #7
            "perform a survey in the NW section", #8
            "survey the north-western zone", #9
            "explore the northwest area", #10
            "conduct a full survey of the NW zone", #11
            "begin scanning the entire northwest region", #12
            "execute a survey mission in the NW sector", #13
            "perform a complete scan of the north-western area", #14
            "carry out a survey in the northwest field", #15
            "start mapping the northwest sector", #16
            "run a scan across the northwest zone", #17
            "search and map the NW area", #18
            "inspect the northwest region thoroughly", #19
            "initiate a scan of the northwest sector", #20
            "survey the entire northwest section", #21
            "map the northwest part of the region", #22
            "examine the NW zone", #23
            "investigate the northwest area", #24
            "monitor the north-western sector", #25
            "analyze the NW region", #26
            "probe the northwest zone", #27
            "check the entire northwest sector", #28
            "explore and map the NW field", #29
            "scan the full northwest quadrant", #30
            "survey northwest", #31
            "scan NW", #32
            "explore northwest zone", #33
            "perform scan in the NW sector", #34
            "search northwest sector", #35
            "map the northwest zone", #36
            "inspect NW area", #37
            "execute scan in northwest", #38
            "conduct survey of the NW zone", #39
            "begin exploration of the northwest area", #40
            "start scanning northwest sector", #41
            "run survey of the north-western zone", #42
            "carry out mapping in northwest", #43
            "initiate exploration of the NW area", #44
            "navigate and survey the northwest sector", #45
            "perform complete survey northwest", #46
            "execute full scan of NW zone", #47
            "conduct thorough inspection of the northwest area", #48
            "begin comprehensive survey in northwest", #49
            "search the central northwest zone", #50
            "scan the middle of the NW sector", #51
            "explore the core northwest area", #52
            "map the center northwest zone", #53
            "inspect the northwest sector thoroughly", #54
            "examine the entire NW area", #55
            "investigate the north-western zone", #56
            "monitor the center of the northwest sector", #57
            "analyze the northwest region fully", #58
            "probe the NW zone systematically", #59
            "check the northwest area completely", #60
            "survey across the northwest sector", #61
            "scan throughout the NW zone", #62
            "explore all of northwest area", #63
            "perform survey across northwest", #64
            "search throughout the NW sector", #65
            "map across the northwest zone", #66
            "inspect the whole northwest area", #67
            "execute survey throughout northwest", #68
            "conduct scan of northwest sector", #69
            "begin thorough survey of the NW zone", #70
            "start comprehensive scan of northwest", #71
            "run full survey of the north-western area", #72
            "carry out complete mapping in northwest", #73
            "initiate comprehensive survey of NW", #74
            "survey the northwest region completely", #75
            "scan the full northwest zone", #76
            "explore the entire northwest sector", #77
            "map the entire NW area", #78
            "inspect the complete northwest zone", #79
            "examine the full northwest region", #80
            "investigate all of the northwest area", #81
            "monitor the entire northwest sector", #82
            "analyze the full northwest zone", #83
            "probe the entire NW area", #84
            "check the whole northwest region", #85
            "survey northwest field", #86
            "scan northwest region", #87
            "explore northwest sector", #88
            "perform northwest survey", #89
            "search northwest zone", #90
            "map northwest area", #91
            "inspect northwest zone", #92
            "execute northwest scan", #93
            "conduct northwest survey", #94
            "begin northwest exploration", #95
            "start northwest scanning", #96
            "run northwest survey", #97
            "carry out northwest mapping", #98
            "initiate northwest scan", #99
            "execute comprehensive northwest survey" #100
            ],
        "test_variations": [
            "conduct a thorough survey of the north-western sector region", #1
            "scan and map the entire northwest zone area", #2
            "perform a complete survey across the NW sector", #3
            "explore the northwest area with detailed scanning", #4
            "execute a comprehensive scan in the north-western zone", #5
            "survey the central northwest sector thoroughly", #6
            "begin full exploration and mapping of the NW area", #7
            "initiate complete scan of the northwestern region", #8
            "perform detailed survey across the northwest zone", #9
            "conduct extensive mapping of the north-western sector" #10
            ]


    },
    {
        "base": { # Survey South-East
            "memory": memory(excluded_missions="SE quadrant survey"),
            "command": "survey the southeast area",
            "output": "SE quadrant survey"
        },
        "variations": [
            "survey the southeast area", #1
            "scan southeast sector", #2
            "explore SE zone", #3
            "perform survey in southeast", #4
            "see what you can find in the SE area", #5
            "take a look around in the south-eastern sector", #6
            "search the southeast area", #7
            "perform a survey in the SE section", #8
            "survey the south-eastern zone", #9
            "explore the southeast area", #10
            "conduct a full survey of the SE zone", #11
            "begin scanning the entire southeast region", #12
            "execute a survey mission in the SE sector", #13
            "perform a complete scan of the south-eastern area", #14
            "carry out a survey in the southeast field", #15
            "start mapping the southeast sector", #16
            "run a scan across the southeast zone", #17
            "search and map the SE area", #18
            "inspect the southeast region thoroughly", #19
            "initiate a scan of the southeast sector", #20
            "survey the entire southeast section", #21
            "map the southeast part of the region", #22
            "examine the SE zone", #23
            "investigate the southeast area", #24
            "monitor the south-eastern sector", #25
            "analyze the SE region", #26
            "probe the southeast zone", #27
            "check the entire southeast sector", #28
            "explore and map the SE field", #29
            "scan the full southeast quadrant", #30
            "survey southeast", #31
            "scan SE", #32
            "explore southeast zone", #33
            "perform scan in the SE sector", #34
            "search southeast sector", #35
            "map the southeast zone", #36
            "inspect SE area", #37
            "execute scan in southeast", #38
            "conduct survey of the SE zone", #39
            "begin exploration of the southeast area", #40
            "start scanning southeast sector", #41
            "run survey of the south-eastern zone", #42
            "carry out mapping in southeast", #43
            "initiate exploration of the SE area", #44
            "navigate and survey the southeast sector", #45
            "perform complete survey southeast", #46
            "execute full scan of SE zone", #47
            "conduct thorough inspection of the southeast area", #48
            "begin comprehensive survey in southeast", #49
            "search the central southeast zone", #50
            "scan the middle of the SE sector", #51
            "explore the core southeast area", #52
            "map the center southeast zone", #53
            "inspect the southeast sector thoroughly", #54
            "examine the entire SE area", #55
            "investigate the south-eastern zone", #56
            "monitor the center of the southeast sector", #57
            "analyze the southeast region fully", #58
            "probe the SE zone systematically", #59
            "check the southeast area completely", #60
            "survey across the southeast sector", #61
            "scan throughout the SE zone", #62
            "explore all of southeast area", #63
            "perform survey across southeast", #64
            "search throughout the SE sector", #65
            "map across the southeast zone", #66
            "inspect the whole southeast area", #67
            "execute survey throughout southeast", #68
            "conduct scan of southeast sector", #69
            "begin thorough survey of the SE zone", #70
            "start comprehensive scan of southeast", #71
            "run full survey of the south-eastern area", #72
            "carry out complete mapping in southeast", #73
            "initiate comprehensive survey of SE", #74
            "survey the southeast region completely", #75
            "scan the full southeast zone", #76
            "explore the entire southeast sector", #77
            "map the entire SE area", #78
            "inspect the complete southeast zone", #79
            "examine the full southeast region", #80
            "investigate all of the southeast area", #81
            "monitor the entire southeast sector", #82
            "analyze the full southeast zone", #83
            "probe the entire SE area", #84
            "check the whole southeast region", #85
            "survey southeast field", #86
            "scan southeast region", #87
            "explore southeast sector", #88
            "perform southeast survey", #89
            "search southeast zone", #90
            "map southeast area", #91
            "inspect southeast zone", #92
            "execute southeast scan", #93
            "conduct southeast survey", #94
            "begin southeast exploration", #95
            "start southeast scanning", #96
            "run southeast survey", #97
            "carry out southeast mapping", #98
            "initiate southeast scan", #99
            "execute comprehensive southeast survey" #100
            ],
        "test_variations": [
            "conduct a thorough survey of the south-eastern sector region", #1
            "scan and map the entire southeast zone area", #2
            "perform a complete survey across the SE sector", #3
            "explore the southeast area with detailed scanning", #4
            "execute a comprehensive scan in the south-eastern zone", #5
            "survey the central southeast sector thoroughly", #6
            "begin full exploration and mapping of the SE area", #7
            "initiate complete scan of the southeastern region", #8
            "perform detailed survey across the southeast zone", #9
            "conduct extensive mapping of the south-eastern sector" #10
            ]


    },
    {
        "base": { # Survey South-West
            "memory": memory(excluded_missions="SW quadrant survey"),
            "command": "survey the southwest area",
            "output": "SW quadrant survey"
        },
        "variations": [
            "survey the southwest area", #1
            "scan southwest sector", #2
            "explore SW zone", #3
            "perform survey in southwest", #4
            "see what you can find in the SW area", #5
            "take a look around in the south-western sector", #6
            "search the southwest area", #7
            "perform a survey in the SW section", #8
            "survey the south-western zone", #9
            "explore the southwest area", #10
            "conduct a full survey of the SW zone", #11
            "begin scanning the entire southwest region", #12
            "execute a survey mission in the SW sector", #13
            "perform a complete scan of the south-western area", #14
            "carry out a survey in the southwest field", #15
            "start mapping the southwest sector", #16
            "run a scan across the southwest zone", #17
            "search and map the SW area", #18
            "inspect the southwest region thoroughly", #19
            "initiate a scan of the southwest sector", #20
            "survey the entire southwest section", #21
            "map the southwest part of the region", #22
            "examine the SW zone", #23
            "investigate the southwest area", #24
            "monitor the south-western sector", #25
            "analyze the SW region", #26
            "probe the southwest zone", #27
            "check the entire southwest sector", #28
            "explore and map the SW field", #29
            "scan the full southwest quadrant", #30
            "survey southwest", #31
            "scan SW", #32
            "explore southwest zone", #33
            "perform scan in the SW sector", #34
            "search southwest sector", #35
            "map the southwest zone", #36
            "inspect SW area", #37
            "execute scan in southwest", #38
            "conduct survey of the SW zone", #39
            "begin exploration of the southwest area", #40
            "start scanning southwest sector", #41
            "run survey of the south-western zone", #42
            "carry out mapping in southwest", #43
            "initiate exploration of the SW area", #44
            "navigate and survey the southwest sector", #45
            "perform complete survey southwest", #46
            "execute full scan of SW zone", #47
            "conduct thorough inspection of the southwest area", #48
            "begin comprehensive survey in southwest", #49
            "search the central southwest zone", #50
            "scan the middle of the SW sector", #51
            "explore the core southwest area", #52
            "map the center southwest zone", #53
            "inspect the southwest sector thoroughly", #54
            "examine the entire SW area", #55
            "investigate the south-western zone", #56
            "monitor the center of the southwest sector", #57
            "analyze the southwest region fully", #58
            "probe the SW zone systematically", #59
            "check the southwest area completely", #60
            "survey across the southwest sector", #61
            "scan throughout the SW zone", #62
            "explore all of southwest area", #63
            "perform survey across southwest", #64
            "search throughout the SW sector", #65
            "map across the southwest zone", #66
            "inspect the whole southwest area", #67
            "execute survey throughout southwest", #68
            "conduct scan of southwest sector", #69
            "begin thorough survey of the SW zone", #70
            "start comprehensive scan of southwest", #71
            "run full survey of the south-western area", #72
            "carry out complete mapping in southwest", #73
            "initiate comprehensive survey of SW", #74
            "survey the southwest region completely", #75
            "scan the full southwest zone", #76
            "explore the entire southwest sector", #77
            "map the entire SW area", #78
            "inspect the complete southwest zone", #79
            "examine the full southwest region", #80
            "investigate all of the southwest area", #81
            "monitor the entire southwest sector", #82
            "analyze the full southwest zone", #83
            "probe the entire SW area", #84
            "check the whole southwest region", #85
            "survey southwest field", #86
            "scan southwest region", #87
            "explore southwest sector", #88
            "perform southwest survey", #89
            "search southwest zone", #90
            "map southwest area", #91
            "inspect southwest zone", #92
            "execute southwest scan", #93
            "conduct southwest survey", #94
            "begin southwest exploration", #95
            "start southwest scanning", #96
            "run southwest survey", #97
            "carry out southwest mapping", #98
            "initiate southwest scan", #99
            "execute comprehensive southwest survey" #100
            ],
        "test_variations": [
            "conduct a thorough survey of the south-western sector region", #1
            "scan and map the entire southwest zone area", #2
            "perform a complete survey across the SW sector", #3
            "explore the southwest area with detailed scanning", #4
            "execute a comprehensive scan in the south-western zone", #5
            "survey the central southwest sector thoroughly", #6
            "begin full exploration and mapping of the SW area", #7
            "initiate complete scan of the southwestern region", #8
            "perform detailed survey across the southwest zone", #9
            "conduct extensive mapping of the south-western sector" #10
            ]


    },
    {
        "base": { # Survey Center
            "memory": memory(excluded_missions="central survey"),
            "command": "survey the central area",
            "output": "central survey"
        },
        "variations": [
            "survey the central area of the map", #1
            "explore the center of the map", #2
            "perform a survey in the central zone", #3
            "look around the central area", #4
            "scan the central region", #5
            "explore the central map area", #6
            "perform a central area reconnaissance", #7
            "survey the map center thoroughly", #8
            "look for points of interest in the central area", #9
            "scan the central zone", #10
            "explore and survey the center", #11
            "perform a complete sweep of the central area", #12
            "look around the center zone", #13
            "survey the central area carefully", #14
            "scan the central map section", #15
            "explore the central map", #16
            "perform a reconnaissance of the map center", #17
            "look around the central location", #18
            "survey the central region", #19
            "scan and explore the central area", #20
            "explore the center zone systematically", #21
            "examine the central area", #22
            "investigate the center of the map", #23
            "monitor the central zone", #24
            "analyze the central region", #25
            "probe the central area", #26
            "check the entire central zone", #27
            "explore and map the central field", #28
            "scan the full central quadrant", #29
            "survey central", #30
            "scan center", #31
            "explore central zone", #32
            "perform scan in the central area", #33
            "search central region", #34
            "map the central zone", #35
            "inspect central area", #36
            "execute scan in central", #37
            "conduct survey of the central zone", #38
            "begin exploration of the central area", #39
            "start scanning central region", #40
            "run survey of the central zone", #41
            "carry out mapping in central area", #42
            "initiate exploration of the center", #43
            "navigate and survey the central area", #44
            "perform complete survey central", #45
            "execute full scan of central zone", #46
            "conduct thorough inspection of the central area", #47
            "begin comprehensive survey in central", #48
            "search the middle central zone", #49
            "scan the heart of the map", #50
            "explore the core central area", #51
            "map the center zone", #52
            "inspect the central area thoroughly", #53
            "examine the entire central region", #54
            "investigate the central zone", #55
            "monitor the center of the map", #56
            "analyze the central area fully", #57
            "probe the central zone systematically", #58
            "check the central area completely", #59
            "survey across the central region", #60
            "scan throughout the central zone", #61
            "explore all of central area", #62
            "perform survey across central", #63
            "search throughout the central region", #64
            "map across the central zone", #65
            "inspect the whole central area", #66
            "execute survey throughout central", #67
            "conduct scan of central region", #68
            "begin thorough survey of the central zone", #69
            "start comprehensive scan of central", #70
            "run full survey of the central area", #71
            "carry out complete mapping in central", #72
            "initiate comprehensive survey of center", #73
            "survey the central region completely", #74
            "scan the full central zone", #75
            "explore the entire central area", #76
            "map the entire central region", #77
            "inspect the complete central zone", #78
            "examine the full central area", #79
            "investigate all of the central region", #80
            "monitor the entire central zone", #81
            "analyze the full central area", #82
            "probe the entire central region", #83
            "check the whole central area", #84
            "survey central field", #85
            "scan central region", #86
            "explore central area", #87
            "perform central survey", #88
            "search central zone", #89
            "map central area", #90
            "inspect central zone", #91
            "execute central scan", #92
            "conduct central survey", #93
            "begin central exploration", #94
            "start central scanning", #95
            "run central survey", #96
            "carry out central mapping", #97
            "initiate central scan", #98
            "execute comprehensive central survey", #99
            "perform detailed central area survey" #100
            ],
        "test_variations": [
            "conduct a thorough survey of the central area region", #1
            "scan and map the entire central zone area", #2
            "perform a complete survey across the center", #3
            "explore the central area with detailed scanning", #4
            "execute a comprehensive scan of the map center", #5
            "survey the central region thoroughly and systematically", #6
            "begin full exploration and mapping of the central area", #7
            "initiate complete scan of the center region", #8
            "perform detailed survey across the central zone", #9
            "conduct extensive mapping of the central area sector" #10
            ]


    },

    
    ###########################################################################################
    #                                   STOP MISSION
    ###########################################################################################
    {
        "base": {
            "memory": memory(),
            "command": "stop the mission",
            "output": "stop_mission"
        },
        "variations": [
            "stop the mission",
            "halt",
            "stop",
            "abort mission",
            "fermo",
            "stop right there",
            "enough",
            "stop thrusters",
            "cease movement",
            "hold position",
            "pause",
            "halt immediately",
            "stop all activity",
            "freeze",
            "halt thrusters",
            "stop operations",
            "abort",
            "stop now",
            "stop engine",
            "hold",
            "cease",
            "terminate mission",
            "pause mission",
            "halt maneuver",
            "stop navigation",
            "end movement",
            "stop propulsion",
            "halt progress",
            "stop actions",
            "cease operation",
            "stop the robot",
            "hold steady",
            "halt immediately",
            "stop driving",
            "halt immediately",
            "stop thruster power",
            "halt thruster activity",
            "stop moving",
            "hold position now",
            "cease all actions",
            "pause operations",
            "stop functioning",
            "freeze movement",
            "halt engines",
            "stop thruster output",
            "halt thruster output",
            "stop navigation now",
            "pause immediately",
            "stop propulsion now",
            "halt the vehicle",
            "stop right now",
            "hold thrusters",
            "cease propulsion",
            "stop and hold",
            "halt immediately thrusters",
            "pause thrusters",
            "stop motion",
            "halt movement",
            "freeze position",
            "stop the craft",
            "hold operations",
            "cease movement now",
            "halt the system",
            "pause the robot",
            "stop the AUV",
            "hold engine",
            "stop all thrusters",
            "halt all thrusters",
            "cease all thrusters",
            "stop right away",
            "halt right away",
            "pause right away",
            "freeze now",
            "stop the mission immediately",
            "halt mission immediately",
            "cease mission",
            "stop vehicle",
            "hold propulsion",
            "halt propulsion",
            "stop course",
            "stop action",
            "freeze thrusters",
            "stop immediately",
            "halt immediately now",
            "cease immediately",
            "hold position immediately",
            "stop operations immediately",
            "pause movement",
            "halt movement immediately",
            "stop navigation system",
            "halt navigation",
            "freeze the vehicle",
            "stop motors",
            "halt motors",
            "pause motors",
            "stop all engines",
            "halt all engines",
            "cease all engines",
            "stop thrust",
            "halt thrust",
            "pause thrust",
            "stop forward motion",
            "halt forward motion",
            "cease forward motion",
            "stop propulsion system"
            ],
    "test_variations": [
        "immediately halt and stop all mission operations and activities", #1
        "abort the current mission and cease all movement immediately", #2
        "stop all thrusters and freeze the vehicle in its current position", #3
        "halt immediately and discontinue all forward motion and mission execution", #4
        "stop the mission operations and hold position with all systems paused", #5
        "cease all movement and activity, then halt all thrusters completely", #6
        "freeze all operations and abort the mission with immediate effect", #7
        "stop right there and halt all mission-related activities and motion", #8
        "discontinue mission execution and hold at current position with all systems halted", #9
        "immediately cease all movement and stop the mission with complete halt of operations" #10
        ]


    },

    ###########################################################################################
    ###########################################################################################
    ###########################################################################################
                                # ██████╗  █████╗ ███╗   ███╗██╗
                                # ██╔══██╗██╔══██╗████╗ ████║██║
                                # ██████╔╝███████║██╔████╔██║██║
                                # ██╔║██║ ██╔══██║██║╚██╔╝██║██║
                                # ██║║██║ ██║  ██║██║ ╚═╝ ██║██║
                                # ╚═╝ ╚═╝ ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝
    ###########################################################################################
    ###########################################################################################
    ###########################################################################################

    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Receive and process the location of a waypoint",
            "output": "skip"
        },
        "variations": [
            "Receive and process the location of a waypoint", #1
            "Accept and process the location of a waypoint", #2
            "Obtain and process the location of a waypoint", #3
            "Acquire and process the location of a waypoint", #4
            "Retrieve and process the location of a waypoint", #5
            "Collect and process the location of a waypoint", #6
            "Gather and process the location of a waypoint", #7
            "Capture and process the location of a waypoint", #8
            "Get and process the location of a waypoint", #9
            "Take and process the location of a waypoint", #10
            "Receive and handle the location of a waypoint", #11
            "Accept and handle the location of a waypoint", #12
            "Obtain and handle the location of a waypoint", #13
            "Acquire and handle the location of a waypoint", #14
            "Retrieve and handle the location of a waypoint", #15
            "Collect and handle the location of a waypoint", #16
            "Gather and handle the location of a waypoint", #17
            "Capture and handle the location of a waypoint", #18
            "Get and handle the location of a waypoint", #19
            "Take and handle the location of a waypoint", #20
            "Receive and execute the location of a waypoint", #21
            "Accept and execute the location of a waypoint", #22
            "Obtain and execute the location of a waypoint", #23
            "Acquire and execute the location of a waypoint", #24
            "Retrieve and execute the location of a waypoint", #25
            "Collect and execute the location of a waypoint", #26
            "Gather and execute the location of a waypoint", #27
            "Capture and execute the location of a waypoint", #28
            "Get and execute the location of a waypoint", #29
            "Take and execute the location of a waypoint", #30
            "Receive and interpret the location of a waypoint", #31
            "Accept and interpret the location of a waypoint", #32
            "Obtain and interpret the location of a waypoint", #33
            "Acquire and interpret the location of a waypoint", #34
            "Retrieve and interpret the location of a waypoint", #35
            "Collect and interpret the location of a waypoint", #36
            "Gather and interpret the location of a waypoint", #37
            "Capture and interpret the location of a waypoint", #38
            "Get and interpret the location of a waypoint", #39
            "Take and interpret the location of a waypoint", #40
            "Receive and compute the location of a waypoint", #41
            "Accept and compute the location of a waypoint", #42
            "Obtain and compute the location of a waypoint", #43
            "Acquire and compute the location of a waypoint", #44
            "Retrieve and compute the location of a waypoint", #45
            "Collect and compute the location of a waypoint", #46
            "Gather and compute the location of a waypoint", #47
            "Capture and compute the location of a waypoint", #48
            "Get and compute the location of a waypoint", #49
            "Take and compute the location of a waypoint", #50
            "Receive and analyse the location of a waypoint", #51
            "Accept and analyse the location of a waypoint", #52
            "Obtain and analyse the location of a waypoint", #53
            "Acquire and analyse the location of a waypoint", #54
            "Retrieve and analyse the location of a waypoint", #55
            "Collect and analyse the location of a waypoint", #56
            "Gather and analyse the location of a waypoint", #57
            "Capture and analyse the location of a waypoint", #58
            "Get and analyse the location of a waypoint", #59
            "Take and analyse the location of a waypoint", #60
            "Receive and evaluate the location of a waypoint", #61
            "Accept and evaluate the location of a waypoint", #62
            "Obtain and evaluate the location of a waypoint", #63
            "Acquire and evaluate the location of a waypoint", #64
            "Retrieve and evaluate the location of a waypoint", #65
            "Collect and evaluate the location of a waypoint", #66
            "Gather and evaluate the location of a waypoint", #67
            "Capture and evaluate the location of a waypoint", #68
            "Get and evaluate the location of a waypoint", #69
            "Take and evaluate the location of a waypoint", #70
            "Receive and decode the location of a waypoint", #71
            "Accept and decode the location of a waypoint", #72
            "Obtain and decode the location of a waypoint", #73
            "Acquire and decode the location of a waypoint", #74
            "Retrieve and decode the location of a waypoint", #75
            "Collect and decode the location of a waypoint", #76
            "Gather and decode the location of a waypoint", #77
            "Capture and decode the location of a waypoint", #78
            "Get and decode the location of a waypoint", #79
            "Take and decode the location of a waypoint", #80
            "Receive and parse the location of a waypoint", #81
            "Accept and parse the location of a waypoint", #82
            "Obtain and parse the location of a waypoint", #83
            "Acquire and parse the location of a waypoint", #84
            "Retrieve and parse the location of a waypoint", #85
            "Collect and parse the location of a waypoint", #86
            "Gather and parse the location of a waypoint", #87
            "Capture and parse the location of a waypoint", #88
            "Get and parse the location of a waypoint", #89
            "Take and parse the location of a waypoint", #90
            "Receive and resolve the location of a waypoint", #91
            "Accept and resolve the location of a waypoint", #92
            "Obtain and resolve the location of a waypoint", #93
            "Acquire and resolve the location of a waypoint", #94
            "Retrieve and resolve the location of a waypoint", #95
            "Collect and resolve the location of a waypoint", #96
            "Gather and resolve the location of a waypoint", #97
            "Capture and resolve the location of a waypoint", #98
            "Get and resolve the location of a waypoint", #99
            "Take and resolve the location of a waypoint", #100
            ],
        "test_variations": [
            "receive and comprehensively process the specified waypoint location coordinates", #1
            "accept and properly handle the incoming waypoint location data", #2
            "obtain and accurately process the waypoint location information provided", #3
            "acquire and execute the waypoint location with appropriate handling", #4
            "retrieve and methodically process the waypoint location parameters", #5
            "collect and execute the waypoint location data in sequence", #6
            "gather and process the waypoint location coordinates systematically", #7
            "capture and handle the waypoint location with precision", #8
            "receive and execute the complete waypoint location information accurately", #9
            "accept and process the waypoint location data with full execution" #10
            ]

    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Reach the waypoint received in underwater autonomous navigation",
            "output": "go to received goal"
        },
        "variations": [
            "Reach the waypoint received", #1
            "Arrive at the waypoint received", #2
            "Navigate to the waypoint received", #3
            "Proceed to the waypoint received", #4
            "Travel to the waypoint received", #5
            "Move to the waypoint received", #6
            "Go to the waypoint received", #7
            "Advance to the waypoint received", #8
            "Progress to the waypoint received", #9
            "Head to the waypoint received", #10
            "Approach the waypoint received", #11
            "Access the waypoint received", #12
            "Attain the waypoint received", #13
            "Get to the waypoint received", #14
            "Make it to the waypoint received", #15
            "Reach the waypoint obtained", #16
            "Arrive at the waypoint obtained", #17
            "Navigate to the waypoint obtained", #18
            "Proceed to the waypoint obtained", #19
            "Travel to the waypoint obtained", #20
            "Move to the waypoint obtained", #21
            "Go to the waypoint obtained", #22
            "Advance to the waypoint obtained", #23
            "Progress to the waypoint obtained", #24
            "Head to the waypoint obtained", #25
            "Approach the waypoint obtained", #26
            "Access the waypoint obtained", #27
            "Attain the waypoint obtained", #28
            "Get to the waypoint obtained", #29
            "Make it to the waypoint obtained", #30
            "Reach the waypoint acquired", #31
            "Arrive at the waypoint acquired", #32
            "Navigate to the waypoint acquired", #33
            "Proceed to the waypoint acquired", #34
            "Travel to the waypoint acquired", #35
            "Move to the waypoint acquired", #36
            "Go to the waypoint acquired", #37
            "Advance to the waypoint acquired", #38
            "Progress to the waypoint acquired", #39
            "Head to the waypoint acquired", #40
            "Approach the waypoint acquired", #41
            "Access the waypoint acquired", #42
            "Attain the waypoint acquired", #43
            "Get to the waypoint acquired", #44
            "Make it to the waypoint acquired", #45
            "Reach the waypoint provided", #46
            "Arrive at the waypoint provided", #47
            "Navigate to the waypoint provided", #48
            "Proceed to the waypoint provided", #49
            "Travel to the waypoint provided", #50
            "Move to the waypoint provided", #51
            "Go to the waypoint provided", #52
            "Advance to the waypoint provided", #53
            "Progress to the waypoint provided", #54
            "Head to the waypoint provided", #55
            "Approach the waypoint provided", #56
            "Access the waypoint provided", #57
            "Attain the waypoint provided", #58
            "Get to the waypoint provided", #59
            "Make it to the waypoint provided", #60
            "Reach the waypoint transmitted", #61
            "Arrive at the waypoint transmitted", #62
            "Navigate to the waypoint transmitted", #63
            "Proceed to the waypoint transmitted", #64
            "Travel to the waypoint transmitted", #65
            "Move to the waypoint transmitted", #66
            "Go to the waypoint transmitted", #67
            "Advance to the waypoint transmitted", #68
            "Progress to the waypoint transmitted", #69
            "Head to the waypoint transmitted", #70
            "Approach the waypoint transmitted", #71
            "Access the waypoint transmitted", #72
            "Attain the waypoint transmitted", #73
            "Get to the waypoint transmitted", #74
            "Make it to the waypoint transmitted", #75
            "Reach the waypoint specified", #76
            "Arrive at the waypoint specified", #77
            "Navigate to the waypoint specified", #78
            "Proceed to the waypoint specified", #79
            "Travel to the waypoint specified", #80
            "Move to the waypoint specified", #81
            "Go to the waypoint specified", #82
            "Advance to the waypoint specified", #83
            "Progress to the waypoint specified", #84
            "Head to the waypoint specified", #85
            "Approach the waypoint specified", #86
            "Access the waypoint specified", #87
            "Attain the waypoint specified", #88
            "Get to the waypoint specified", #89
            "Make it to the waypoint specified", #90
            "Reach the waypoint designated", #91
            "Arrive at the waypoint designated", #92
            "Navigate to the waypoint designated", #93
            "Proceed to the waypoint designated", #94
            "Travel to the waypoint designated", #95
            "Move to the waypoint designated", #96
            "Go to the waypoint designated", #97
            "Advance to the waypoint designated", #98
            "Progress to the waypoint designated", #99
            "Head to the waypoint designated", #100
            ],
        "test_variations": [
            "reach and arrive at the designated waypoint that was received", #1
            "navigate to the waypoint obtained and proceed toward its location", #2
            "advance to the transmitted waypoint coordinates that were received", #3
            "travel to the specified waypoint location obtained from the system", #4
            "move toward and reach the waypoint data that was received", #5
            "go to the waypoint received and execute arrival at the coordinates", #6
            "progress toward the designated waypoint that was obtained and transmitted", #7
            "approach and attain the waypoint location received from the command", #8
            "navigate to the specified waypoint received with full execution to arrival", #9
            "reach the waypoint coordinates obtained and complete transit to the location" #10
            ]

    },
    {
        "base": {
            "memory": memory(total_buoys=0, required_missions="go to received goal", total_missions=1),
            "command": "-	Pass through the gate",
            "output": "cross gate"
        },
        "variations": [
            "pass through the gate", #1
            "transit through the gate", #2
            "navigate through the gate", #3
            "proceed through the gate", #4
            "move through the gate", #5
            "traverse through the gate", #6
            "go through the gate", #7
            "advance through the gate", #8
            "travel through the gate", #9
            "cross through the gate", #10
            "pass via the gate", #11
            "transit via the gate", #12
            "navigate via the gate", #13
            "proceed via the gate", #14
            "move via the gate", #15
            "pass between the yellow buoys", #16
            "pass between the gate buoys", #17
            "traverse between the yellow buoys", #18
            "navigate between the gate buoys", #19
            "go between the yellow buoys", #20
            "move between the gate buoys", #21
            "travel between the yellow buoys", #22
            "cross between the gate buoys", #23
            "advance between the yellow buoys", #24
            "transit between the gate buoys", #25
            "pass through the buoy gate", #26
            "navigate through the buoy gate", #27
            "traverse through the buoy gate", #28
            "proceed through the buoy gate", #29
            "travel through the buoy gate", #30
            "move through the buoy gate", #31
            "cross through the buoy gate", #32
            "advance through the buoy gate", #33
            "transit through the buoy gate", #34
            "go through the buoy gate", #35
            "pass the gate structure", #36
            "transit the gate structure", #37
            "navigate the gate structure", #38
            "proceed past the gate", #39
            "move past the gate", #40
            "travel past the gate", #41
            "cross past the gate", #42
            "advance past the gate", #43
            "transit past the gate", #44
            "go past the gate", #45
            "pass through the entrance", #46
            "navigate through the entrance", #47
            "proceed through the entrance", #48
            "travel through the entrance", #49
            "move through the entrance", #50
            "pass between the buoys", #51
            "navigate between the buoys", #52
            "traverse between the buoys", #53
            "go between the buoys", #54
            "move between the buoys", #55
            "travel between the buoys", #56
            "cross between the buoys", #57
            "advance between the buoys", #58
            "transit between the buoys", #59
            "proceed between the buoys", #60
            "pass through the opening", #61
            "navigate through the opening", #62
            "transit through the opening", #63
            "move through the opening", #64
            "travel through the opening", #65
            "cross the gate opening", #66
            "go through the marked gate", #67
            "traverse the marked gate", #68
            "navigate the marked gate", #69
            "proceed through the marked gate", #70
            "pass through the yellow gate", #71
            "navigate through the yellow gate", #72
            "transit through the yellow gate", #73
            "move through the yellow gate", #74
            "travel through the yellow gate", #75
            "pass by the gate", #76
            "transit by the gate", #77
            "move by the gate", #78
            "go by the gate", #79
            "advance by the gate", #80
            "pass through the checkpoint", #81
            "navigate through the checkpoint", #82
            "proceed through the checkpoint", #83
            "travel through the checkpoint", #84
            "move through the checkpoint", #85
            "pass the buoy markers", #86
            "navigate the buoy markers", #87
            "transit the buoy markers", #88
            "proceed past the buoy markers", #89
            "move past the buoy markers", #90
            "pass in front of the gate", #91
            "navigate in front of the gate", #92
            "traverse through the designated gate", #93
            "go through the designated passage", #94
            "cross the gate threshold", #95
            "advance through the gate corridor", #96
            "transit through the gate passage", #97
            "move through the gate corridor", #98
            "navigate the gate passage", #99
            "proceed through the gate entrance" #100
            ],
        "test_variations": [
            "pass through the gate and advance between the marked yellow buoys", #1
            "navigate through the gate structure while passing between the buoys", #2
            "traverse through the marked gate opening between the yellow buoys", #3
            "proceed through the gate passage and cross between the designated buoys", #4
            "transit through the buoy gate and pass between the marked markers", #5
            "move through the gate while navigating between the yellow buoys correctly", #6
            "go through the marked gate entrance and advance between the buoy markers", #7
            "cross through the gate passage situated between the yellow buoys", #8
            "navigate the gate structure by passing precisely between the marked buoys", #9
            "travel through the designated gate opening positioned between the yellow markers" #10
            ]

    },
        {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Pass through the gate",
            "output": "cross gate"
        },
        "variations": [
            "pass through the gate", #1
            "transit through the gate", #2
            "navigate through the gate", #3
            "proceed through the gate", #4
            "move through the gate", #5
            "traverse through the gate", #6
            "go through the gate", #7
            "advance through the gate", #8
            "travel through the gate", #9
            "cross through the gate", #10
            "pass via the gate", #11
            "transit via the gate", #12
            "navigate via the gate", #13
            "proceed via the gate", #14
            "move via the gate", #15
            "pass between the yellow buoys", #16
            "pass between the gate buoys", #17
            "traverse between the yellow buoys", #18
            "navigate between the gate buoys", #19
            "go between the yellow buoys", #20
            "move between the gate buoys", #21
            "travel between the yellow buoys", #22
            "cross between the gate buoys", #23
            "advance between the yellow buoys", #24
            "transit between the gate buoys", #25
            "pass through the buoy gate", #26
            "navigate through the buoy gate", #27
            "traverse through the buoy gate", #28
            "proceed through the buoy gate", #29
            "travel through the buoy gate", #30
            "move through the buoy gate", #31
            "cross through the buoy gate", #32
            "advance through the buoy gate", #33
            "transit through the buoy gate", #34
            "go through the buoy gate", #35
            "pass the gate structure", #36
            "transit the gate structure", #37
            "navigate the gate structure", #38
            "proceed past the gate", #39
            "move past the gate", #40
            "travel past the gate", #41
            "cross past the gate", #42
            "advance past the gate", #43
            "transit past the gate", #44
            "go past the gate", #45
            "pass through the entrance", #46
            "navigate through the entrance", #47
            "proceed through the entrance", #48
            "travel through the entrance", #49
            "move through the entrance", #50
            "pass between the buoys", #51
            "navigate between the buoys", #52
            "traverse between the buoys", #53
            "go between the buoys", #54
            "move between the buoys", #55
            "travel between the buoys", #56
            "cross between the buoys", #57
            "advance between the buoys", #58
            "transit between the buoys", #59
            "proceed between the buoys", #60
            "pass through the opening", #61
            "navigate through the opening", #62
            "transit through the opening", #63
            "move through the opening", #64
            "travel through the opening", #65
            "cross the gate opening", #66
            "go through the marked gate", #67
            "traverse the marked gate", #68
            "navigate the marked gate", #69
            "proceed through the marked gate", #70
            "pass through the yellow gate", #71
            "navigate through the yellow gate", #72
            "transit through the yellow gate", #73
            "move through the yellow gate", #74
            "travel through the yellow gate", #75
            "pass by the gate", #76
            "transit by the gate", #77
            "move by the gate", #78
            "go by the gate", #79
            "advance by the gate", #80
            "pass through the checkpoint", #81
            "navigate through the checkpoint", #82
            "proceed through the checkpoint", #83
            "travel through the checkpoint", #84
            "move through the checkpoint", #85
            "pass the buoy markers", #86
            "navigate the buoy markers", #87
            "transit the buoy markers", #88
            "proceed past the buoy markers", #89
            "move past the buoy markers", #90
            "pass in front of the gate", #91
            "navigate in front of the gate", #92
            "traverse through the designated gate", #93
            "go through the designated passage", #94
            "cross the gate threshold", #95
            "advance through the gate corridor", #96
            "transit through the gate passage", #97
            "move through the gate corridor", #98
            "navigate the gate passage", #99
            "proceed through the gate entrance" #100
            ],
        "test_variations": [
            "pass through the gate and advance between the marked yellow buoys", #1
            "navigate through the gate structure while passing between the buoys", #2
            "traverse through the marked gate opening between the yellow buoys", #3
            "proceed through the gate passage and cross between the designated buoys", #4
            "transit through the buoy gate and pass between the marked markers", #5
            "move through the gate while navigating between the yellow buoys correctly", #6
            "go through the marked gate entrance and advance between the buoy markers", #7
            "cross through the gate passage situated between the yellow buoys", #8
            "navigate the gate structure by passing precisely between the marked buoys", #9
            "travel through the designated gate opening positioned between the yellow markers" #10
            ]

    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow"], total_buoys=2, required_missions=["go to received goal", "cross gate"], total_missions=2),
            "command": "Map the coloured buoy area providing a geo-localized map with the buoy positions",
            "output": "map buoy area A"
        },
        "variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions", #10
            "Map the coloured buoy area delivering a geo-localised map with the buoy positions", #11
            "Chart the coloured buoy area delivering a geo-localised map with the buoy positions", #12
            "Survey the coloured buoy area delivering a geo-localised map with the buoy positions", #13
            "Document the coloured buoy area delivering a geo-localised map with the buoy positions", #14
            "Plot the coloured buoy area delivering a geo-localised map with the buoy positions", #15
            "Map the coloured buoy area generating a geo-localised map with the buoy positions", #16
            "Chart the coloured buoy area generating a geo-localised map with the buoy positions", #17
            "Survey the coloured buoy area generating a geo-localised map with the buoy positions", #18
            "Document the coloured buoy area generating a geo-localised map with the buoy positions", #19
            "Plot the coloured buoy area generating a geo-localised map with the buoy positions", #20
            "Map the coloured buoy area producing a geo-localised map with the buoy positions", #21
            "Chart the coloured buoy area producing a geo-localised map with the buoy positions", #22
            "Survey the coloured buoy area producing a geo-localised map with the buoy positions", #23
            "Document the coloured buoy area producing a geo-localised map with the buoy positions", #24
            "Plot the coloured buoy area producing a geo-localised map with the buoy positions", #25
            "Map the coloured buoy area creating a geo-localised map with the buoy positions", #26
            "Chart the coloured buoy area creating a geo-localised map with the buoy positions", #27
            "Survey the coloured buoy area creating a geo-localised map with the buoy positions", #28
            "Document the coloured buoy area creating a geo-localised map with the buoy positions", #29
            "Plot the coloured buoy area creating a geo-localised map with the buoy positions", #30
            "Map the coloured buoy zone providing a geo-localised map with the buoy positions", #31
            "Chart the coloured buoy zone providing a geo-localised map with the buoy positions", #32
            "Survey the coloured buoy zone providing a geo-localised map with the buoy positions", #33
            "Document the coloured buoy zone providing a geo-localised map with the buoy positions", #34
            "Plot the coloured buoy zone providing a geo-localised map with the buoy positions", #35
            "Map the coloured buoy region providing a geo-localised map with the buoy positions", #36
            "Chart the coloured buoy region providing a geo-localised map with the buoy positions", #37
            "Survey the coloured buoy region providing a geo-localised map with the buoy positions", #38
            "Document the coloured buoy region providing a geo-localised map with the buoy positions", #39
            "Plot the coloured buoy region providing a geo-localised map with the buoy positions", #40
            "Map the coloured buoy sector providing a geo-localised map with the buoy positions", #41
            "Chart the coloured buoy sector providing a geo-localised map with the buoy positions", #42
            "Survey the coloured buoy sector providing a geo-localised map with the buoy positions", #43
            "Document the coloured buoy sector providing a geo-localised map with the buoy positions", #44
            "Plot the coloured buoy sector providing a geo-localised map with the buoy positions", #45
            "Map the coloured buoy field providing a geo-localised map with the buoy positions", #46
            "Chart the coloured buoy field providing a geo-localised map with the buoy positions", #47
            "Survey the coloured buoy field providing a geo-localised map with the buoy positions", #48
            "Document the coloured buoy field providing a geo-localised map with the buoy positions", #49
            "Plot the coloured buoy field providing a geo-localised map with the buoy positions", #50
            "Map the coloured buoy area providing a geo-referenced map with the buoy positions", #51
            "Chart the coloured buoy area providing a geo-referenced map with the buoy positions", #52
            "Survey the coloured buoy area providing a geo-referenced map with the buoy positions", #53
            "Document the coloured buoy area providing a geo-referenced map with the buoy positions", #54
            "Plot the coloured buoy area providing a geo-referenced map with the buoy positions", #55
            "Map the coloured buoy area providing a geo-tagged map with the buoy positions", #56
            "Chart the coloured buoy area providing a geo-tagged map with the buoy positions", #57
            "Survey the coloured buoy area providing a geo-tagged map with the buoy positions", #58
            "Document the coloured buoy area providing a geo-tagged map with the buoy positions", #59
            "Plot the coloured buoy area providing a geo-tagged map with the buoy positions", #60
            "Map the coloured buoy area providing a spatially-referenced map with the buoy positions", #61
            "Chart the coloured buoy area providing a spatially-referenced map with the buoy positions", #62
            "Survey the coloured buoy area providing a spatially-referenced map with the buoy positions", #63
            "Document the coloured buoy area providing a spatially-referenced map with the buoy positions", #64
            "Plot the coloured buoy area providing a spatially-referenced map with the buoy positions", #65
            "Map the coloured buoy area providing a position-stamped map with the buoy positions", #66
            "Chart the coloured buoy area providing a position-stamped map with the buoy positions", #67
            "Survey the coloured buoy area providing a position-stamped map with the buoy positions", #68
            "Document the coloured buoy area providing a position-stamped map with the buoy positions", #69
            "Plot the coloured buoy area providing a position-stamped map with the buoy positions", #70
            "Map the coloured buoy area providing a location-stamped map with the buoy positions", #71
            "Chart the coloured buoy area providing a location-stamped map with the buoy positions", #72
            "Survey the coloured buoy area providing a location-stamped map with the buoy positions", #73
            "Document the coloured buoy area providing a location-stamped map with the buoy positions", #74
            "Plot the coloured buoy area providing a location-stamped map with the buoy positions", #75
            "Map the coloured buoy area providing a coordinate-tagged map with the buoy positions", #76
            "Chart the coloured buoy area providing a coordinate-tagged map with the buoy positions", #77
            "Survey the coloured buoy area providing a coordinate-tagged map with the buoy positions", #78
            "Document the coloured buoy area providing a coordinate-tagged map with the buoy positions", #79
            "Plot the coloured buoy area providing a coordinate-tagged map with the buoy positions", #80
            "Map the coloured buoy area and provide a geo-localised map with the buoy positions", #81
            "Chart the coloured buoy area and provide a geo-localised map with the buoy positions", #82
            "Survey the coloured buoy area and provide a geo-localised map with the buoy positions", #83
            "Document the coloured buoy area and provide a geo-localised map with the buoy positions", #84
            "Plot the coloured buoy area and provide a geo-localised map with the buoy positions", #85
            "Map the coloured buoy area and supply a geo-localised map with the buoy positions", #86
            "Chart the coloured buoy area and supply a geo-localised map with the buoy positions", #87
            "Survey the coloured buoy area and supply a geo-localised map with the buoy positions", #88
            "Document the coloured buoy area and supply a geo-localised map with the buoy positions", #89
            "Plot the coloured buoy area and supply a geo-localised map with the buoy positions", #90
            "Map the coloured buoy area and deliver a geo-localised map with the buoy positions", #91
            "Chart the coloured buoy area and deliver a geo-localised map with the buoy positions", #92
            "Survey the coloured buoy area and deliver a geo-localised map with the buoy positions", #93
            "Document the coloured buoy area and deliver a geo-localised map with the buoy positions", #94
            "Plot the coloured buoy area and deliver a geo-localised map with the buoy positions", #95
            "Map the coloured buoy area and generate a geo-localised map with the buoy positions", #96
            "Chart the coloured buoy area and generate a geo-localised map with the buoy positions", #97
            "Survey the coloured buoy area and generate a geo-localised map with the buoy positions", #98
            "Document the coloured buoy area and generate a geo-localised map with the buoy positions", #99
            "Plot the coloured buoy area and generate a geo-localised map with the buoy positions", #100
            ],
        "test_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Map the coloured buoy area providing a geo-localized map with the buoy positions",
            "output": "map buoy area A"
        },
        "variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions", #10
            "Map the coloured buoy area delivering a geo-localised map with the buoy positions", #11
            "Chart the coloured buoy area delivering a geo-localised map with the buoy positions", #12
            "Survey the coloured buoy area delivering a geo-localised map with the buoy positions", #13
            "Document the coloured buoy area delivering a geo-localised map with the buoy positions", #14
            "Plot the coloured buoy area delivering a geo-localised map with the buoy positions", #15
            "Map the coloured buoy area generating a geo-localised map with the buoy positions", #16
            "Chart the coloured buoy area generating a geo-localised map with the buoy positions", #17
            "Survey the coloured buoy area generating a geo-localised map with the buoy positions", #18
            "Document the coloured buoy area generating a geo-localised map with the buoy positions", #19
            "Plot the coloured buoy area generating a geo-localised map with the buoy positions", #20
            "Map the coloured buoy area producing a geo-localised map with the buoy positions", #21
            "Chart the coloured buoy area producing a geo-localised map with the buoy positions", #22
            "Survey the coloured buoy area producing a geo-localised map with the buoy positions", #23
            "Document the coloured buoy area producing a geo-localised map with the buoy positions", #24
            "Plot the coloured buoy area producing a geo-localised map with the buoy positions", #25
            "Map the coloured buoy area creating a geo-localised map with the buoy positions", #26
            "Chart the coloured buoy area creating a geo-localised map with the buoy positions", #27
            "Survey the coloured buoy area creating a geo-localised map with the buoy positions", #28
            "Document the coloured buoy area creating a geo-localised map with the buoy positions", #29
            "Plot the coloured buoy area creating a geo-localised map with the buoy positions", #30
            "Map the coloured buoy zone providing a geo-localised map with the buoy positions", #31
            "Chart the coloured buoy zone providing a geo-localised map with the buoy positions", #32
            "Survey the coloured buoy zone providing a geo-localised map with the buoy positions", #33
            "Document the coloured buoy zone providing a geo-localised map with the buoy positions", #34
            "Plot the coloured buoy zone providing a geo-localised map with the buoy positions", #35
            "Map the coloured buoy region providing a geo-localised map with the buoy positions", #36
            "Chart the coloured buoy region providing a geo-localised map with the buoy positions", #37
            "Survey the coloured buoy region providing a geo-localised map with the buoy positions", #38
            "Document the coloured buoy region providing a geo-localised map with the buoy positions", #39
            "Plot the coloured buoy region providing a geo-localised map with the buoy positions", #40
            "Map the coloured buoy sector providing a geo-localised map with the buoy positions", #41
            "Chart the coloured buoy sector providing a geo-localised map with the buoy positions", #42
            "Survey the coloured buoy sector providing a geo-localised map with the buoy positions", #43
            "Document the coloured buoy sector providing a geo-localised map with the buoy positions", #44
            "Plot the coloured buoy sector providing a geo-localised map with the buoy positions", #45
            "Map the coloured buoy field providing a geo-localised map with the buoy positions", #46
            "Chart the coloured buoy field providing a geo-localised map with the buoy positions", #47
            "Survey the coloured buoy field providing a geo-localised map with the buoy positions", #48
            "Document the coloured buoy field providing a geo-localised map with the buoy positions", #49
            "Plot the coloured buoy field providing a geo-localised map with the buoy positions", #50
            "Map the coloured buoy area providing a geo-referenced map with the buoy positions", #51
            "Chart the coloured buoy area providing a geo-referenced map with the buoy positions", #52
            "Survey the coloured buoy area providing a geo-referenced map with the buoy positions", #53
            "Document the coloured buoy area providing a geo-referenced map with the buoy positions", #54
            "Plot the coloured buoy area providing a geo-referenced map with the buoy positions", #55
            "Map the coloured buoy area providing a geo-tagged map with the buoy positions", #56
            "Chart the coloured buoy area providing a geo-tagged map with the buoy positions", #57
            "Survey the coloured buoy area providing a geo-tagged map with the buoy positions", #58
            "Document the coloured buoy area providing a geo-tagged map with the buoy positions", #59
            "Plot the coloured buoy area providing a geo-tagged map with the buoy positions", #60
            "Map the coloured buoy area providing a spatially-referenced map with the buoy positions", #61
            "Chart the coloured buoy area providing a spatially-referenced map with the buoy positions", #62
            "Survey the coloured buoy area providing a spatially-referenced map with the buoy positions", #63
            "Document the coloured buoy area providing a spatially-referenced map with the buoy positions", #64
            "Plot the coloured buoy area providing a spatially-referenced map with the buoy positions", #65
            "Map the coloured buoy area providing a position-stamped map with the buoy positions", #66
            "Chart the coloured buoy area providing a position-stamped map with the buoy positions", #67
            "Survey the coloured buoy area providing a position-stamped map with the buoy positions", #68
            "Document the coloured buoy area providing a position-stamped map with the buoy positions", #69
            "Plot the coloured buoy area providing a position-stamped map with the buoy positions", #70
            "Map the coloured buoy area providing a location-stamped map with the buoy positions", #71
            "Chart the coloured buoy area providing a location-stamped map with the buoy positions", #72
            "Survey the coloured buoy area providing a location-stamped map with the buoy positions", #73
            "Document the coloured buoy area providing a location-stamped map with the buoy positions", #74
            "Plot the coloured buoy area providing a location-stamped map with the buoy positions", #75
            "Map the coloured buoy area providing a coordinate-tagged map with the buoy positions", #76
            "Chart the coloured buoy area providing a coordinate-tagged map with the buoy positions", #77
            "Survey the coloured buoy area providing a coordinate-tagged map with the buoy positions", #78
            "Document the coloured buoy area providing a coordinate-tagged map with the buoy positions", #79
            "Plot the coloured buoy area providing a coordinate-tagged map with the buoy positions", #80
            "Map the coloured buoy area and provide a geo-localised map with the buoy positions", #81
            "Chart the coloured buoy area and provide a geo-localised map with the buoy positions", #82
            "Survey the coloured buoy area and provide a geo-localised map with the buoy positions", #83
            "Document the coloured buoy area and provide a geo-localised map with the buoy positions", #84
            "Plot the coloured buoy area and provide a geo-localised map with the buoy positions", #85
            "Map the coloured buoy area and supply a geo-localised map with the buoy positions", #86
            "Chart the coloured buoy area and supply a geo-localised map with the buoy positions", #87
            "Survey the coloured buoy area and supply a geo-localised map with the buoy positions", #88
            "Document the coloured buoy area and supply a geo-localised map with the buoy positions", #89
            "Plot the coloured buoy area and supply a geo-localised map with the buoy positions", #90
            "Map the coloured buoy area and deliver a geo-localised map with the buoy positions", #91
            "Chart the coloured buoy area and deliver a geo-localised map with the buoy positions", #92
            "Survey the coloured buoy area and deliver a geo-localised map with the buoy positions", #93
            "Document the coloured buoy area and deliver a geo-localised map with the buoy positions", #94
            "Plot the coloured buoy area and deliver a geo-localised map with the buoy positions", #95
            "Map the coloured buoy area and generate a geo-localised map with the buoy positions", #96
            "Chart the coloured buoy area and generate a geo-localised map with the buoy positions", #97
            "Survey the coloured buoy area and generate a geo-localised map with the buoy positions", #98
            "Document the coloured buoy area and generate a geo-localised map with the buoy positions", #99
            "Plot the coloured buoy area and generate a geo-localised map with the buoy positions", #100
            ],
        "test_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ]
    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow"], total_buoys=2, required_missions=["go to received goal", "cross gate"], total_missions=2),
            "command": "Detect the colors of each buoy and react in a specific mode",
            "output": "make move A"
        },
        "variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10
            "Detect the colours of each buoy and act in a specific mode", #11
            "Identify the colours of each buoy and act in a specific mode", #12
            "Recognise the colours of each buoy and act in a specific mode", #13
            "Determine the colours of each buoy and act in a specific mode", #14
            "Sense the colours of each buoy and act in a specific mode", #15
            "Detect the colours of each buoy and operate in a specific mode", #16
            "Identify the colours of each buoy and operate in a specific mode", #17
            "Recognise the colours of each buoy and operate in a specific mode", #18
            "Determine the colours of each buoy and operate in a specific mode", #19
            "Sense the colours of each buoy and operate in a specific mode", #20
            "Detect the colours of each buoy and function in a specific mode", #21
            "Identify the colours of each buoy and function in a specific mode", #22
            "Recognise the colours of each buoy and function in a specific mode", #23
            "Determine the colours of each buoy and function in a specific mode", #24
            "Sense the colours of each buoy and function in a specific mode", #25
            "Detect the colours of each buoy and perform in a specific mode", #26
            "Identify the colours of each buoy and perform in a specific mode", #27
            "Recognise the colours of each buoy and perform in a specific mode", #28
            "Determine the colours of each buoy and perform in a specific mode", #29
            "Sense the colours of each buoy and perform in a specific mode", #30
            "Detect the colours of each buoy and behave in a specific mode", #31
            "Identify the colours of each buoy and behave in a specific mode", #32
            "Recognise the colours of each buoy and behave in a specific mode", #33
            "Determine the colours of each buoy and behave in a specific mode", #34
            "Sense the colours of each buoy and behave in a specific mode", #35
            "Detect the colours of each buoy and execute in a specific mode", #36
            "Identify the colours of each buoy and execute in a specific mode", #37
            "Recognise the colours of each buoy and execute in a specific mode", #38
            "Determine the colours of each buoy and execute in a specific mode", #39
            "Sense the colours of each buoy and execute in a specific mode", #40
            "Detect the hues of each buoy and react in a specific mode", #41
            "Identify the hues of each buoy and react in a specific mode", #42
            "Recognise the hues of each buoy and react in a specific mode", #43
            "Determine the hues of each buoy and react in a specific mode", #44
            "Sense the hues of each buoy and react in a specific mode", #45
            "Detect the colouration of each buoy and react in a specific mode", #46
            "Identify the colouration of each buoy and react in a specific mode", #47
            "Recognise the colouration of each buoy and react in a specific mode", #48
            "Determine the colouration of each buoy and react in a specific mode", #49
            "Sense the colouration of each buoy and react in a specific mode", #50
            "Detect the colour coding of each buoy and react in a specific mode", #51
            "Identify the colour coding of each buoy and react in a specific mode", #52
            "Recognise the colour coding of each buoy and react in a specific mode", #53
            "Determine the colour coding of each buoy and react in a specific mode", #54
            "Sense the colour coding of each buoy and react in a specific mode", #55
            "Detect the colour markers of each buoy and react in a specific mode", #56
            "Identify the colour markers of each buoy and react in a specific mode", #57
            "Recognise the colour markers of each buoy and react in a specific mode", #58
            "Determine the colour markers of each buoy and react in a specific mode", #59
            "Sense the colour markers of each buoy and react in a specific mode", #60
            "Detect the colours of every buoy and react in a specific mode", #61
            "Identify the colours of every buoy and react in a specific mode", #62
            "Recognise the colours of every buoy and react in a specific mode", #63
            "Determine the colours of every buoy and react in a specific mode", #64
            "Sense the colours of every buoy and react in a specific mode", #65
            "Detect the colours of all buoys and react in a specific mode", #66
            "Identify the colours of all buoys and react in a specific mode", #67
            "Recognise the colours of all buoys and react in a specific mode", #68
            "Determine the colours of all buoys and react in a specific mode", #69
            "Sense the colours of all buoys and react in a specific mode", #70
            "Detect buoy colours and react in a specific mode", #71
            "Identify buoy colours and react in a specific mode", #72
            "Recognise buoy colours and react in a specific mode", #73
            "Determine buoy colours and react in a specific mode", #74
            "Sense buoy colours and react in a specific mode", #75
            "Detect the colour of each buoy and react in a specific mode", #76
            "Identify the colour of each buoy and react in a specific mode", #77
            "Recognise the colour of each buoy and react in a specific mode", #78
            "Determine the colour of each buoy and react in a specific mode", #79
            "Sense the colour of each buoy and react in a specific mode", #80
            "Detect the colours on each buoy and react in a specific mode", #81
            "Identify the colours on each buoy and react in a specific mode", #82
            "Recognise the colours on each buoy and react in a specific mode", #83
            "Determine the colours on each buoy and react in a specific mode", #84
            "Sense the colours on each buoy and react in a specific mode", #85
            "Detect each buoy colour and react in a specific mode", #86
            "Identify each buoy colour and react in a specific mode", #87
            "Recognise each buoy colour and react in a specific mode", #88
            "Determine each buoy colour and react in a specific mode", #89
            "Sense each buoy colour and react in a specific mode", #90
            "Detect the colours per buoy and react in a specific mode", #91
            "Identify the colours per buoy and react in a specific mode", #92
            "Recognise the colours per buoy and react in a specific mode", #93
            "Determine the colours per buoy and react in a specific mode", #94
            "Sense the colours per buoy and react in a specific mode", #95
            "Detect colour for each buoy and react in a specific mode", #96
            "Identify colour for each buoy and react in a specific mode", #97
            "Recognise colour for each buoy and react in a specific mode", #98
            "Determine colour for each buoy and react in a specific mode", #99
            "Sense colour for each buoy and react in a specific mode", #100

            "perform the specific move for each buoy",
            "for each buoy perform the corresponding move",
            "make the corresponding move for every buoy",
            "perform the adequate action for every buoy",
            "execute the specific move for each buoy",
            "for each buoy execute the corresponding move",
            "make the appropriate move for every buoy",
            "perform the proper action for every buoy",
            "carry out the specific move for each buoy",
            "for each buoy carry out the corresponding move",
            "make the suitable move for every buoy",
            "perform the correct action for every buoy",
            "conduct the specific move for each buoy",
            "for each buoy conduct the corresponding move",
            "make the right move for every buoy",
            "perform the necessary action for every buoy",
            "complete the specific move for each buoy",
            "for each buoy complete the corresponding move",
            "make the proper move for every buoy",
            "perform the required action for every buoy",
            "implement the specific move for each buoy",
            "for each buoy implement the corresponding move",
            "make the correct move for every buoy",
            "perform the appropriate action for every buoy",
            "execute the appropriate move for each buoy",
            "for each buoy execute the proper move",
            "make the adequate move for every buoy",
            "perform the suitable action for every buoy",
            "carry out the appropriate move for each buoy",
            "for each buoy carry out the proper move",
            "make the necessary move for every buoy",
            "perform the fitting action for every buoy",
            "conduct the appropriate move for each buoy",
            "for each buoy conduct the proper move",
            "make the required move for every buoy",
            "perform the matching action for every buoy",
            "complete the appropriate move for each buoy",
            "for each buoy complete the proper move",
            "make the fitting move for every buoy",
            "perform the designated action for every buoy",
            "implement the appropriate move for each buoy",
            "for each buoy implement the proper move",
            "make the matching move for every buoy",
            "perform the respective action for every buoy",
            "execute the correct move for each buoy",
            "for each buoy execute the suitable move",
            "make the designated move for every buoy",
            "perform the relevant action for every buoy",
            "carry out the correct move for each buoy",
            "for each buoy carry out the suitable move" #150
            ],
        "test_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses" #10
            ]
    },
        {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Detect the colors of each buoy and react in a specific mode",
            "output": "make move A"
        },
        "variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10
            "Detect the colours of each buoy and act in a specific mode", #11
            "Identify the colours of each buoy and act in a specific mode", #12
            "Recognise the colours of each buoy and act in a specific mode", #13
            "Determine the colours of each buoy and act in a specific mode", #14
            "Sense the colours of each buoy and act in a specific mode", #15
            "Detect the colours of each buoy and operate in a specific mode", #16
            "Identify the colours of each buoy and operate in a specific mode", #17
            "Recognise the colours of each buoy and operate in a specific mode", #18
            "Determine the colours of each buoy and operate in a specific mode", #19
            "Sense the colours of each buoy and operate in a specific mode", #20
            "Detect the colours of each buoy and function in a specific mode", #21
            "Identify the colours of each buoy and function in a specific mode", #22
            "Recognise the colours of each buoy and function in a specific mode", #23
            "Determine the colours of each buoy and function in a specific mode", #24
            "Sense the colours of each buoy and function in a specific mode", #25
            "Detect the colours of each buoy and perform in a specific mode", #26
            "Identify the colours of each buoy and perform in a specific mode", #27
            "Recognise the colours of each buoy and perform in a specific mode", #28
            "Determine the colours of each buoy and perform in a specific mode", #29
            "Sense the colours of each buoy and perform in a specific mode", #30
            "Detect the colours of each buoy and behave in a specific mode", #31
            "Identify the colours of each buoy and behave in a specific mode", #32
            "Recognise the colours of each buoy and behave in a specific mode", #33
            "Determine the colours of each buoy and behave in a specific mode", #34
            "Sense the colours of each buoy and behave in a specific mode", #35
            "Detect the colours of each buoy and execute in a specific mode", #36
            "Identify the colours of each buoy and execute in a specific mode", #37
            "Recognise the colours of each buoy and execute in a specific mode", #38
            "Determine the colours of each buoy and execute in a specific mode", #39
            "Sense the colours of each buoy and execute in a specific mode", #40
            "Detect the hues of each buoy and react in a specific mode", #41
            "Identify the hues of each buoy and react in a specific mode", #42
            "Recognise the hues of each buoy and react in a specific mode", #43
            "Determine the hues of each buoy and react in a specific mode", #44
            "Sense the hues of each buoy and react in a specific mode", #45
            "Detect the colouration of each buoy and react in a specific mode", #46
            "Identify the colouration of each buoy and react in a specific mode", #47
            "Recognise the colouration of each buoy and react in a specific mode", #48
            "Determine the colouration of each buoy and react in a specific mode", #49
            "Sense the colouration of each buoy and react in a specific mode", #50
            "Detect the colour coding of each buoy and react in a specific mode", #51
            "Identify the colour coding of each buoy and react in a specific mode", #52
            "Recognise the colour coding of each buoy and react in a specific mode", #53
            "Determine the colour coding of each buoy and react in a specific mode", #54
            "Sense the colour coding of each buoy and react in a specific mode", #55
            "Detect the colour markers of each buoy and react in a specific mode", #56
            "Identify the colour markers of each buoy and react in a specific mode", #57
            "Recognise the colour markers of each buoy and react in a specific mode", #58
            "Determine the colour markers of each buoy and react in a specific mode", #59
            "Sense the colour markers of each buoy and react in a specific mode", #60
            "Detect the colours of every buoy and react in a specific mode", #61
            "Identify the colours of every buoy and react in a specific mode", #62
            "Recognise the colours of every buoy and react in a specific mode", #63
            "Determine the colours of every buoy and react in a specific mode", #64
            "Sense the colours of every buoy and react in a specific mode", #65
            "Detect the colours of all buoys and react in a specific mode", #66
            "Identify the colours of all buoys and react in a specific mode", #67
            "Recognise the colours of all buoys and react in a specific mode", #68
            "Determine the colours of all buoys and react in a specific mode", #69
            "Sense the colours of all buoys and react in a specific mode", #70
            "Detect buoy colours and react in a specific mode", #71
            "Identify buoy colours and react in a specific mode", #72
            "Recognise buoy colours and react in a specific mode", #73
            "Determine buoy colours and react in a specific mode", #74
            "Sense buoy colours and react in a specific mode", #75
            "Detect the colour of each buoy and react in a specific mode", #76
            "Identify the colour of each buoy and react in a specific mode", #77
            "Recognise the colour of each buoy and react in a specific mode", #78
            "Determine the colour of each buoy and react in a specific mode", #79
            "Sense the colour of each buoy and react in a specific mode", #80
            "Detect the colours on each buoy and react in a specific mode", #81
            "Identify the colours on each buoy and react in a specific mode", #82
            "Recognise the colours on each buoy and react in a specific mode", #83
            "Determine the colours on each buoy and react in a specific mode", #84
            "Sense the colours on each buoy and react in a specific mode", #85
            "Detect each buoy colour and react in a specific mode", #86
            "Identify each buoy colour and react in a specific mode", #87
            "Recognise each buoy colour and react in a specific mode", #88
            "Determine each buoy colour and react in a specific mode", #89
            "Sense each buoy colour and react in a specific mode", #90
            "Detect the colours per buoy and react in a specific mode", #91
            "Identify the colours per buoy and react in a specific mode", #92
            "Recognise the colours per buoy and react in a specific mode", #93
            "Determine the colours per buoy and react in a specific mode", #94
            "Sense the colours per buoy and react in a specific mode", #95
            "Detect colour for each buoy and react in a specific mode", #96
            "Identify colour for each buoy and react in a specific mode", #97
            "Recognise colour for each buoy and react in a specific mode", #98
            "Determine colour for each buoy and react in a specific mode", #99
            "Sense colour for each buoy and react in a specific mode", #100

            # "perform the specific move for each buoy",
            # "for each buoy perform the corresponding move",
            # "make the corresponding move for every buoy",
            # "perform the adequate action for every buoy",
            # "execute the specific move for each buoy",
            # "for each buoy execute the corresponding move",
            # "make the appropriate move for every buoy",
            # "perform the proper action for every buoy",
            # "carry out the specific move for each buoy",
            # "for each buoy carry out the corresponding move",
            # "make the suitable move for every buoy",
            # "perform the correct action for every buoy",
            # "conduct the specific move for each buoy",
            # "for each buoy conduct the corresponding move",
            # "make the right move for every buoy",
            # "perform the necessary action for every buoy",
            # "complete the specific move for each buoy",
            # "for each buoy complete the corresponding move",
            # "make the proper move for every buoy",
            # "perform the required action for every buoy",
            # "implement the specific move for each buoy",
            # "for each buoy implement the corresponding move",
            # "make the correct move for every buoy",
            # "perform the appropriate action for every buoy",
            # "execute the appropriate move for each buoy",
            # "for each buoy execute the proper move",
            # "make the adequate move for every buoy",
            # "perform the suitable action for every buoy",
            # "carry out the appropriate move for each buoy",
            # "for each buoy carry out the proper move",
            # "make the necessary move for every buoy",
            # "perform the fitting action for every buoy",
            # "conduct the appropriate move for each buoy",
            # "for each buoy conduct the proper move",
            # "make the required move for every buoy",
            # "perform the matching action for every buoy",
            # "complete the appropriate move for each buoy",
            # "for each buoy complete the proper move",
            # "make the fitting move for every buoy",
            # "perform the designated action for every buoy",
            # "implement the appropriate move for each buoy",
            # "for each buoy implement the proper move",
            # "make the matching move for every buoy",
            # "perform the respective action for every buoy",
            # "execute the correct move for each buoy",
            # "for each buoy execute the suitable move",
            # "make the designated move for every buoy",
            # "perform the relevant action for every buoy",
            # "carry out the correct move for each buoy",
            # "for each buoy carry out the suitable move" #150
            ],
        "test_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses" #10
            ]
    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow", "black", "red", "white"], total_buoys=5, required_missions=["go to received goal", "cross gate", "map buoy area A"], total_missions=5),
            "command": "Detect the colors of each buoy and react in a specific mode",
            "output": "make move A"
        },
        "variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10
            "Detect the colours of each buoy and act in a specific mode", #11
            "Identify the colours of each buoy and act in a specific mode", #12
            "Recognise the colours of each buoy and act in a specific mode", #13
            "Determine the colours of each buoy and act in a specific mode", #14
            "Sense the colours of each buoy and act in a specific mode", #15
            "Detect the colours of each buoy and operate in a specific mode", #16
            "Identify the colours of each buoy and operate in a specific mode", #17
            "Recognise the colours of each buoy and operate in a specific mode", #18
            "Determine the colours of each buoy and operate in a specific mode", #19
            "Sense the colours of each buoy and operate in a specific mode", #20
            "Detect the colours of each buoy and function in a specific mode", #21
            "Identify the colours of each buoy and function in a specific mode", #22
            "Recognise the colours of each buoy and function in a specific mode", #23
            "Determine the colours of each buoy and function in a specific mode", #24
            "Sense the colours of each buoy and function in a specific mode", #25
            "Detect the colours of each buoy and perform in a specific mode", #26
            "Identify the colours of each buoy and perform in a specific mode", #27
            "Recognise the colours of each buoy and perform in a specific mode", #28
            "Determine the colours of each buoy and perform in a specific mode", #29
            "Sense the colours of each buoy and perform in a specific mode", #30
            "Detect the colours of each buoy and behave in a specific mode", #31
            "Identify the colours of each buoy and behave in a specific mode", #32
            "Recognise the colours of each buoy and behave in a specific mode", #33
            "Determine the colours of each buoy and behave in a specific mode", #34
            "Sense the colours of each buoy and behave in a specific mode", #35
            "Detect the colours of each buoy and execute in a specific mode", #36
            "Identify the colours of each buoy and execute in a specific mode", #37
            "Recognise the colours of each buoy and execute in a specific mode", #38
            "Determine the colours of each buoy and execute in a specific mode", #39
            "Sense the colours of each buoy and execute in a specific mode", #40
            "Detect the hues of each buoy and react in a specific mode", #41
            "Identify the hues of each buoy and react in a specific mode", #42
            "Recognise the hues of each buoy and react in a specific mode", #43
            "Determine the hues of each buoy and react in a specific mode", #44
            "Sense the hues of each buoy and react in a specific mode", #45
            "Detect the colouration of each buoy and react in a specific mode", #46
            "Identify the colouration of each buoy and react in a specific mode", #47
            "Recognise the colouration of each buoy and react in a specific mode", #48
            "Determine the colouration of each buoy and react in a specific mode", #49
            "Sense the colouration of each buoy and react in a specific mode", #50
            "Detect the colour coding of each buoy and react in a specific mode", #51
            "Identify the colour coding of each buoy and react in a specific mode", #52
            "Recognise the colour coding of each buoy and react in a specific mode", #53
            "Determine the colour coding of each buoy and react in a specific mode", #54
            "Sense the colour coding of each buoy and react in a specific mode", #55
            "Detect the colour markers of each buoy and react in a specific mode", #56
            "Identify the colour markers of each buoy and react in a specific mode", #57
            "Recognise the colour markers of each buoy and react in a specific mode", #58
            "Determine the colour markers of each buoy and react in a specific mode", #59
            "Sense the colour markers of each buoy and react in a specific mode", #60
            "Detect the colours of every buoy and react in a specific mode", #61
            "Identify the colours of every buoy and react in a specific mode", #62
            "Recognise the colours of every buoy and react in a specific mode", #63
            "Determine the colours of every buoy and react in a specific mode", #64
            "Sense the colours of every buoy and react in a specific mode", #65
            "Detect the colours of all buoys and react in a specific mode", #66
            "Identify the colours of all buoys and react in a specific mode", #67
            "Recognise the colours of all buoys and react in a specific mode", #68
            "Determine the colours of all buoys and react in a specific mode", #69
            "Sense the colours of all buoys and react in a specific mode", #70
            "Detect buoy colours and react in a specific mode", #71
            "Identify buoy colours and react in a specific mode", #72
            "Recognise buoy colours and react in a specific mode", #73
            "Determine buoy colours and react in a specific mode", #74
            "Sense buoy colours and react in a specific mode", #75
            "Detect the colour of each buoy and react in a specific mode", #76
            "Identify the colour of each buoy and react in a specific mode", #77
            "Recognise the colour of each buoy and react in a specific mode", #78
            "Determine the colour of each buoy and react in a specific mode", #79
            "Sense the colour of each buoy and react in a specific mode", #80
            "Detect the colours on each buoy and react in a specific mode", #81
            "Identify the colours on each buoy and react in a specific mode", #82
            "Recognise the colours on each buoy and react in a specific mode", #83
            "Determine the colours on each buoy and react in a specific mode", #84
            "Sense the colours on each buoy and react in a specific mode", #85
            "Detect each buoy colour and react in a specific mode", #86
            "Identify each buoy colour and react in a specific mode", #87
            "Recognise each buoy colour and react in a specific mode", #88
            "Determine each buoy colour and react in a specific mode", #89
            "Sense each buoy colour and react in a specific mode", #90
            "Detect the colours per buoy and react in a specific mode", #91
            "Identify the colours per buoy and react in a specific mode", #92
            "Recognise the colours per buoy and react in a specific mode", #93
            "Determine the colours per buoy and react in a specific mode", #94
            "Sense the colours per buoy and react in a specific mode", #95
            "Detect colour for each buoy and react in a specific mode", #96
            "Identify colour for each buoy and react in a specific mode", #97
            "Recognise colour for each buoy and react in a specific mode", #98
            "Determine colour for each buoy and react in a specific mode", #99
            "Sense colour for each buoy and react in a specific mode", #100

            # "perform the specific move for each buoy",
            # "for each buoy perform the corresponding move",
            # "make the corresponding move for every buoy",
            # "perform the adequate action for every buoy",
            # "execute the specific move for each buoy",
            # "for each buoy execute the corresponding move",
            # "make the appropriate move for every buoy",
            # "perform the proper action for every buoy",
            # "carry out the specific move for each buoy",
            # "for each buoy carry out the corresponding move",
            # "make the suitable move for every buoy",
            # "perform the correct action for every buoy",
            # "conduct the specific move for each buoy",
            # "for each buoy conduct the corresponding move",
            # "make the right move for every buoy",
            # "perform the necessary action for every buoy",
            # "complete the specific move for each buoy",
            # "for each buoy complete the corresponding move",
            # "make the proper move for every buoy",
            # "perform the required action for every buoy",
            # "implement the specific move for each buoy",
            # "for each buoy implement the corresponding move",
            # "make the correct move for every buoy",
            # "perform the appropriate action for every buoy",
            # "execute the appropriate move for each buoy",
            # "for each buoy execute the proper move",
            # "make the adequate move for every buoy",
            # "perform the suitable action for every buoy",
            # "carry out the appropriate move for each buoy",
            # "for each buoy carry out the proper move",
            # "make the necessary move for every buoy",
            # "perform the fitting action for every buoy",
            # "conduct the appropriate move for each buoy",
            # "for each buoy conduct the proper move",
            # "make the required move for every buoy",
            # "perform the matching action for every buoy",
            # "complete the appropriate move for each buoy",
            # "for each buoy complete the proper move",
            # "make the fitting move for every buoy",
            # "perform the designated action for every buoy",
            # "implement the appropriate move for each buoy",
            # "for each buoy implement the proper move",
            # "make the matching move for every buoy",
            # "perform the respective action for every buoy",
            # "execute the correct move for each buoy",
            # "for each buoy execute the suitable move",
            # "make the designated move for every buoy",
            # "perform the relevant action for every buoy",
            # "carry out the correct move for each buoy",
            # "for each buoy carry out the suitable move" #150
            ],
        "test_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses", #10

            # "perform the specific maneuver associated with each buoy in the sequence",
            # "for each buoy encountered, execute the corresponding predefined move",
            # "carry out the appropriate action designated for every individual buoy",
            # "perform the particular move that corresponds to each buoy's identifier",
            # "for every buoy in the mission, make the specific move assigned to it",
            # "execute the distinct action appropriate for each buoy along the path",
            # "perform the designated maneuver tailored to each buoy's requirements",
            # "for each buoy detected, carry out its corresponding operational move",
            # "make the specific move prescribed for every buoy in the mission plan",
            # "perform the action that matches each buoy's designated procedure" #20
            ]

    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow", "red", "white", "black"], total_buoys=5, required_missions=["go to NE goal", "cross gate", "map buoy area A", "make move black A", "make move red A", "make move white A"], total_missions=8),
            "command": "-	Survey the pipeline structure ",
            "output": "skip"
        },
        "variations": [
            "Survey the pipeline structure", #1
            "Inspect the pipeline structure", #2
            "Examine the pipeline structure", #3
            "Assess the pipeline structure", #4
            "Evaluate the pipeline structure", #5
            "Observe the pipeline structure", #6
            "Analyse the pipeline structure", #7
            "Review the pipeline structure", #8
            "Study the pipeline structure", #9
            "Investigate the pipeline structure", #10
            "Survey the pipeline infrastructure", #11
            "Inspect the pipeline infrastructure", #12
            "Examine the pipeline infrastructure", #13
            "Assess the pipeline infrastructure", #14
            "Evaluate the pipeline infrastructure", #15
            "Observe the pipeline infrastructure", #16
            "Analyse the pipeline infrastructure", #17
            "Review the pipeline infrastructure", #18
            "Study the pipeline infrastructure", #19
            "Investigate the pipeline infrastructure", #20
            "Survey the pipeline system", #21
            "Inspect the pipeline system", #22
            "Examine the pipeline system", #23
            "Assess the pipeline system", #24
            "Evaluate the pipeline system", #25
            "Observe the pipeline system", #26
            "Analyse the pipeline system", #27
            "Review the pipeline system", #28
            "Study the pipeline system", #29
            "Investigate the pipeline system", #30
            "Survey the pipeline assembly", #31
            "Inspect the pipeline assembly", #32
            "Examine the pipeline assembly", #33
            "Assess the pipeline assembly", #34
            "Evaluate the pipeline assembly", #35
            "Observe the pipeline assembly", #36
            "Analyse the pipeline assembly", #37
            "Review the pipeline assembly", #38
            "Study the pipeline assembly", #39
            "Investigate the pipeline assembly", #40
            "Survey the pipeline configuration", #41
            "Inspect the pipeline configuration", #42
            "Examine the pipeline configuration", #43
            "Assess the pipeline configuration", #44
            "Evaluate the pipeline configuration", #45
            "Observe the pipeline configuration", #46
            "Analyse the pipeline configuration", #47
            "Review the pipeline configuration", #48
            "Study the pipeline configuration", #49
            "Investigate the pipeline configuration", #50
            "Conduct survey of the pipeline structure", #51
            "Conduct inspection of the pipeline structure", #52
            "Conduct examination of the pipeline structure", #53
            "Conduct assessment of the pipeline structure", #54
            "Conduct evaluation of the pipeline structure", #55
            "Perform survey of the pipeline structure", #56
            "Perform inspection of the pipeline structure", #57
            "Perform examination of the pipeline structure", #58
            "Perform assessment of the pipeline structure", #59
            "Perform evaluation of the pipeline structure", #60
            "Carry out survey of the pipeline structure", #61
            "Carry out inspection of the pipeline structure", #62
            "Carry out examination of the pipeline structure", #63
            "Carry out assessment of the pipeline structure", #64
            "Carry out evaluation of the pipeline structure", #65
            "Execute survey of the pipeline structure", #66
            "Execute inspection of the pipeline structure", #67
            "Execute examination of the pipeline structure", #68
            "Execute assessment of the pipeline structure", #69
            "Execute evaluation of the pipeline structure", #70
            "Undertake survey of the pipeline structure", #71
            "Undertake inspection of the pipeline structure", #72
            "Undertake examination of the pipeline structure", #73
            "Undertake assessment of the pipeline structure", #74
            "Undertake evaluation of the pipeline structure", #75
            "Complete survey of the pipeline structure", #76
            "Complete inspection of the pipeline structure", #77
            "Complete examination of the pipeline structure", #78
            "Complete assessment of the pipeline structure", #79
            "Complete evaluation of the pipeline structure", #80
            "Survey the pipeline layout", #81
            "Inspect the pipeline layout", #82
            "Examine the pipeline layout", #83
            "Assess the pipeline layout", #84
            "Evaluate the pipeline layout", #85
            "Observe the pipeline layout", #86
            "Analyse the pipeline layout", #87
            "Review the pipeline layout", #88
            "Study the pipeline layout", #89
            "Investigate the pipeline layout", #90
            "Survey the pipeline framework", #91
            "Inspect the pipeline framework", #92
            "Examine the pipeline framework", #93
            "Assess the pipeline framework", #94
            "Evaluate the pipeline framework", #95
            "Observe the pipeline framework", #96
            "Analyse the pipeline framework", #97
            "Review the pipeline framework", #98
            "Study the pipeline framework", #99
            "Investigate the pipeline framework", #100
            ],
        "test_variations": [
            "survey and inspect the pipeline structure comprehensively to evaluate the overall condition and integrity", #1
            "examine the pipeline infrastructure thoroughly and assess any potential damage or structural defects present", #2
            "analyse the pipeline structure in detail and evaluate the condition of all critical pipeline components", #3
            "investigate the pipeline infrastructure systematically and review the findings to determine maintenance requirements", #4
            "observe the pipeline structure carefully and conduct a detailed assessment of corrosion or wear patterns", #5
            "study the pipeline infrastructure and assess its structural integrity using comprehensive inspection methodologies", #6
            "examine and analyse the pipeline structure to evaluate its operational safety and infrastructure compliance", #7
            "review the pipeline infrastructure thoroughly and investigate any areas showing signs of deterioration or damage", #8
            "assess the pipeline structure comprehensively and evaluate the condition with detailed observations and analysis", #9
            "inspect the pipeline infrastructure systematically and examine all structural components for potential issues or degradation" #10
            ]

    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Survey the pipeline structure ",
            "output": "skip"
        },
        "variations": [
            "Survey the pipeline structure", #1
            "Inspect the pipeline structure", #2
            "Examine the pipeline structure", #3
            "Assess the pipeline structure", #4
            "Evaluate the pipeline structure", #5
            "Observe the pipeline structure", #6
            "Analyse the pipeline structure", #7
            "Review the pipeline structure", #8
            "Study the pipeline structure", #9
            "Investigate the pipeline structure", #10
            "Survey the pipeline infrastructure", #11
            "Inspect the pipeline infrastructure", #12
            "Examine the pipeline infrastructure", #13
            "Assess the pipeline infrastructure", #14
            "Evaluate the pipeline infrastructure", #15
            "Observe the pipeline infrastructure", #16
            "Analyse the pipeline infrastructure", #17
            "Review the pipeline infrastructure", #18
            "Study the pipeline infrastructure", #19
            "Investigate the pipeline infrastructure", #20
            "Survey the pipeline system", #21
            "Inspect the pipeline system", #22
            "Examine the pipeline system", #23
            "Assess the pipeline system", #24
            "Evaluate the pipeline system", #25
            "Observe the pipeline system", #26
            "Analyse the pipeline system", #27
            "Review the pipeline system", #28
            "Study the pipeline system", #29
            "Investigate the pipeline system", #30
            "Survey the pipeline assembly", #31
            "Inspect the pipeline assembly", #32
            "Examine the pipeline assembly", #33
            "Assess the pipeline assembly", #34
            "Evaluate the pipeline assembly", #35
            "Observe the pipeline assembly", #36
            "Analyse the pipeline assembly", #37
            "Review the pipeline assembly", #38
            "Study the pipeline assembly", #39
            "Investigate the pipeline assembly", #40
            "Survey the pipeline configuration", #41
            "Inspect the pipeline configuration", #42
            "Examine the pipeline configuration", #43
            "Assess the pipeline configuration", #44
            "Evaluate the pipeline configuration", #45
            "Observe the pipeline configuration", #46
            "Analyse the pipeline configuration", #47
            "Review the pipeline configuration", #48
            "Study the pipeline configuration", #49
            "Investigate the pipeline configuration", #50
            "Conduct survey of the pipeline structure", #51
            "Conduct inspection of the pipeline structure", #52
            "Conduct examination of the pipeline structure", #53
            "Conduct assessment of the pipeline structure", #54
            "Conduct evaluation of the pipeline structure", #55
            "Perform survey of the pipeline structure", #56
            "Perform inspection of the pipeline structure", #57
            "Perform examination of the pipeline structure", #58
            "Perform assessment of the pipeline structure", #59
            "Perform evaluation of the pipeline structure", #60
            "Carry out survey of the pipeline structure", #61
            "Carry out inspection of the pipeline structure", #62
            "Carry out examination of the pipeline structure", #63
            "Carry out assessment of the pipeline structure", #64
            "Carry out evaluation of the pipeline structure", #65
            "Execute survey of the pipeline structure", #66
            "Execute inspection of the pipeline structure", #67
            "Execute examination of the pipeline structure", #68
            "Execute assessment of the pipeline structure", #69
            "Execute evaluation of the pipeline structure", #70
            "Undertake survey of the pipeline structure", #71
            "Undertake inspection of the pipeline structure", #72
            "Undertake examination of the pipeline structure", #73
            "Undertake assessment of the pipeline structure", #74
            "Undertake evaluation of the pipeline structure", #75
            "Complete survey of the pipeline structure", #76
            "Complete inspection of the pipeline structure", #77
            "Complete examination of the pipeline structure", #78
            "Complete assessment of the pipeline structure", #79
            "Complete evaluation of the pipeline structure", #80
            "Survey the pipeline layout", #81
            "Inspect the pipeline layout", #82
            "Examine the pipeline layout", #83
            "Assess the pipeline layout", #84
            "Evaluate the pipeline layout", #85
            "Observe the pipeline layout", #86
            "Analyse the pipeline layout", #87
            "Review the pipeline layout", #88
            "Study the pipeline layout", #89
            "Investigate the pipeline layout", #90
            "Survey the pipeline framework", #91
            "Inspect the pipeline framework", #92
            "Examine the pipeline framework", #93
            "Assess the pipeline framework", #94
            "Evaluate the pipeline framework", #95
            "Observe the pipeline framework", #96
            "Analyse the pipeline framework", #97
            "Review the pipeline framework", #98
            "Study the pipeline framework", #99
            "Investigate the pipeline framework", #100
            ],
        "test_variations": [
            "survey and inspect the pipeline structure comprehensively to evaluate the overall condition and integrity", #1
            "examine the pipeline infrastructure thoroughly and assess any potential damage or structural defects present", #2
            "analyse the pipeline structure in detail and evaluate the condition of all critical pipeline components", #3
            "investigate the pipeline infrastructure systematically and review the findings to determine maintenance requirements", #4
            "observe the pipeline structure carefully and conduct a detailed assessment of corrosion or wear patterns", #5
            "study the pipeline infrastructure and assess its structural integrity using comprehensive inspection methodologies", #6
            "examine and analyse the pipeline structure to evaluate its operational safety and infrastructure compliance", #7
            "review the pipeline infrastructure thoroughly and investigate any areas showing signs of deterioration or damage", #8
            "assess the pipeline structure comprehensively and evaluate the condition with detailed observations and analysis", #9
            "inspect the pipeline infrastructure systematically and examine all structural components for potential issues or degradation" #10
            ]

    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow", "red", "white", "black"], total_buoys=5, required_missions=["go to NE goal", "cross gate", "map buoy area A", "make move black A", "make move red A", "make move white A"], total_missions=8),
            "command": "Detect, localize and determining the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure",
            "output": "skip"
        },
        "variations": [
            "Detect, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #1
            "Identify, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #2
            "Recognise, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #3
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #4
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #5
            "Detect, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #6
            "Identify, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #7
            "Recognise, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #8
            "Locate, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #9
            "Find, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #10
            "Detect, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #11
            "Identify, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #12
            "Recognise, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #13
            "Locate, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #14
            "Find, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #15
            "Detect, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #16
            "Identify, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #17
            "Recognise, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #18
            "Locate, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #19
            "Find, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #20
            "Detect, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #21
            "Identify, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #22
            "Recognise, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #23
            "Locate, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #24
            "Find, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #25
            "Detect, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #26
            "Identify, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #27
            "Recognise, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #28
            "Locate, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #29
            "Find, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #30
            "Detect, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #31
            "Identify, locate and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #32
            "Recognise, locate and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #33
            "Locate, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #34
            "Find, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #35
            "Detect, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #36
            "Identify, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #37
            "Recognise, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #38
            "Locate, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #39
            "Find, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #40
            "Detect, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #41
            "Identify, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #42
            "Recognise, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #43
            "Locate, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #44
            "Find, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #45
            "Detect, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #46
            "Identify, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #47
            "Recognise, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #48
            "Locate, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #49
            "Find, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #50
            "Detect, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #51
            "Identify, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #52
            "Recognise, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #53
            "Locate, localise and determine the dimensions of the damage (a red marker) situated on one of the two pipes connected to one pipeline structure", #54
            "Find, localise and determine the dimensions of the damage (a red marker) situated on one of the two pipes connected to one pipeline structure", #55
            "Detect, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #56
            "Identify, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #57
            "Recognise, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #58
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes attached to one pipeline structure", #59
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes attached to one pipeline structure", #60
            "Detect, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #61
            "Identify, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #62
            "Recognise, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #63
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes joined to one pipeline structure", #64
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes joined to one pipeline structure", #65
            "Detect, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #66
            "Identify, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #67
            "Recognise, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #68
            "Locate, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #69
            "Find, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #70
            "Detect, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #71
            "Identify, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #72
            "Recognise, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #73
            "Locate, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #74
            "Find, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #75
            "Detect, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #76
            "Identify, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #77
            "Recognise, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #78
            "Locate, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #79
            "Find, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #80
            "Detect, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #81
            "Identify, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #82
            "Recognise, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #83
            "Locate, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #84
            "Find, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #85
            "Detect, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #86
            "Identify, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #87
            "Recognise, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #88
            "Locate, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #89
            "Find, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #90
            "Detect, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #91
            "Identify, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #92
            "Recognise, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #93
            "Locate, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #94
            "Find, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #95
            "Detect, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #96
            "Identify, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #97
            "Recognise, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #98
            "Locate, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #99
            "Find, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #100
            ],
        "test_variations": [
            "detect, localise and precisely determine the exact dimensions of the damage marked by a red marker positioned on one of the two pipes within the connected pipeline structure", #1
            "identify and localise the red marker damage, then establish and measure the dimensional characteristics on the interconnected pipeline structure", #2
            "recognise the red damage marker on either pipe of the pipeline structure, localise its position, and ascertain its dimensional parameters accurately", #3
            "locate the red marker indicating damage on one of the two connected pipes and determine the precise dimensions of the identified damage area", #4
            "find, localise and comprehensively measure the dimensions of the red-marked damage situated on one of the dual pipes in the pipeline infrastructure", #5
            "detect the red damage marker on the interconnected pipeline structure, establish its localisation point, and determine its dimensional extent", #6
            "identify the red marker damage position on either pipe, localise it precisely, and measure the exact dimensions of the damage structure", #7
            "recognise and localise the damage indicated by the red marker on one of the two connected pipes, then ascertain the complete dimensional information", #8
            "locate the red-marked damage on the pipeline structure's dual pipes, localise it systematically, and determine all relevant damage dimensions", #9
            "comprehensively detect the red marker damage on one of the connected pipes, localise it within the pipeline structure, and measure its dimensional characteristics" #10
            ]

    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Detect, localize and determining the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure",
            "output": "skip"
        },
        "variations": [
            "Detect, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #1
            "Identify, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #2
            "Recognise, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #3
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #4
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #5
            "Detect, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #6
            "Identify, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #7
            "Recognise, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #8
            "Locate, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #9
            "Find, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #10
            "Detect, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #11
            "Identify, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #12
            "Recognise, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #13
            "Locate, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #14
            "Find, localise and ascertain the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #15
            "Detect, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #16
            "Identify, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #17
            "Recognise, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #18
            "Locate, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #19
            "Find, localise and measure the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #20
            "Detect, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #21
            "Identify, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #22
            "Recognise, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #23
            "Locate, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #24
            "Find, localise and calculate the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #25
            "Detect, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #26
            "Identify, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #27
            "Recognise, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #28
            "Locate, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #29
            "Find, pinpoint and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #30
            "Detect, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #31
            "Identify, locate and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #32
            "Recognise, locate and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #33
            "Locate, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #34
            "Find, identify and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #35
            "Detect, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #36
            "Identify, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #37
            "Recognise, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #38
            "Locate, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #39
            "Find, localise and determine the size of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #40
            "Detect, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #41
            "Identify, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #42
            "Recognise, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #43
            "Locate, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #44
            "Find, localise and determine the extent of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #45
            "Detect, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #46
            "Identify, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #47
            "Recognise, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #48
            "Locate, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #49
            "Find, localise and determine the measurements of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #50
            "Detect, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #51
            "Identify, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #52
            "Recognise, localise and determine the dimensions of the damage (a red marker) located on one of the two pipes connected to one pipeline structure", #53
            "Locate, localise and determine the dimensions of the damage (a red marker) situated on one of the two pipes connected to one pipeline structure", #54
            "Find, localise and determine the dimensions of the damage (a red marker) situated on one of the two pipes connected to one pipeline structure", #55
            "Detect, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #56
            "Identify, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #57
            "Recognise, localise and determine the dimensions of the damage (a red marker) placed on one of the two pipes connected to one pipeline structure", #58
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes attached to one pipeline structure", #59
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes attached to one pipeline structure", #60
            "Detect, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #61
            "Identify, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #62
            "Recognise, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes linked to one pipeline structure", #63
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes joined to one pipeline structure", #64
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes joined to one pipeline structure", #65
            "Detect, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #66
            "Identify, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #67
            "Recognise, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #68
            "Locate, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #69
            "Find, localise and determine the dimensions of the defect (a red marker) positioned on one of the two pipes connected to one pipeline structure", #70
            "Detect, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #71
            "Identify, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #72
            "Recognise, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #73
            "Locate, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #74
            "Find, localise and determine the dimensions of the fault (a red marker) positioned on one of the two pipes connected to one pipeline structure", #75
            "Detect, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #76
            "Identify, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #77
            "Recognise, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #78
            "Locate, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #79
            "Find, localise and determine the dimensions of the impairment (a red marker) positioned on one of the two pipes connected to one pipeline structure", #80
            "Detect, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #81
            "Identify, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #82
            "Recognise, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #83
            "Locate, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #84
            "Find, localise and determine the dimensions of the anomaly (a red marker) positioned on one of the two pipes connected to one pipeline structure", #85
            "Detect, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #86
            "Identify, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #87
            "Recognise, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #88
            "Locate, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #89
            "Find, geo-localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #90
            "Detect, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #91
            "Identify, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #92
            "Recognise, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #93
            "Locate, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #94
            "Find, position and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #95
            "Detect, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #96
            "Identify, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #97
            "Recognise, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #98
            "Locate, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #99
            "Find, map and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #100
            ],
        "test_variations": [
            "detect, localise and precisely determine the exact dimensions of the damage marked by a red marker positioned on one of the two pipes within the connected pipeline structure", #1
            "identify and localise the red marker damage, then establish and measure the dimensional characteristics on the interconnected pipeline structure", #2
            "recognise the red damage marker on either pipe of the pipeline structure, localise its position, and ascertain its dimensional parameters accurately", #3
            "locate the red marker indicating damage on one of the two connected pipes and determine the precise dimensions of the identified damage area", #4
            "find, localise and comprehensively measure the dimensions of the red-marked damage situated on one of the dual pipes in the pipeline infrastructure", #5
            "detect the red damage marker on the interconnected pipeline structure, establish its localisation point, and determine its dimensional extent", #6
            "identify the red marker damage position on either pipe, localise it precisely, and measure the exact dimensions of the damage structure", #7
            "recognise and localise the damage indicated by the red marker on one of the two connected pipes, then ascertain the complete dimensional information", #8
            "locate the red-marked damage on the pipeline structure's dual pipes, localise it systematically, and determine all relevant damage dimensions", #9
            "comprehensively detect the red marker damage on one of the connected pipes, localise it within the pipeline structure, and measure its dimensional characteristics" #10
            ]

    }


]

# ============================================================================================
#                                   CLARIFICATION EXAMPLES
# ============================================================================================

cmdcla_variations_data = [
    # # Regular clarification (no failure)
    # {
    #     "base": {
    #         "memory": "...",
    #         "previous_response": "move_red",
    #         "output": "move_red"
    #     },
    #     "command_variations": [...],
    #     "test_command_variations": [...],
    #     "clarification_variations": [...]
    #     # No failure_context
    # },
    # # Clarification after failure
    # {
    #     "base": {
    #         "memory": "...",
    #         "previous_response": "findbuoy_red",
    #         "output": "move_red"  # Different output after failure
    #     },
    #     "command_variations": [...],
    #     "test_command_variations": [...],
    #     "clarification_variations": [...],
    #     "failure_context": get_failure_context("find")  # Include this
    # }
        {
        "base": { # confirm Goal North-West
            "memory": memory(excluded_missions="go to NW goal"),
            "command": "go to northwest area",
            "previous_response": "go to NW goal",
            "output": "go to NW goal"
        },
        "command_variations": [
            "go to north west area", #1
            "move to northwest", #2
            "navigate to NW sector", #3
            "head to north west zone", #4
            "reach the center of the NW area", #5
            "proceed to the center of the north-western area", #6
            "set route to the north western zone", #7
            "place a goal waypoint in the NW area", #8
            "set the target waypoint in the north west sector", #9
            "move to the center of the NW zone" #10
            ],
        "test_command_variations": [
            "navigate toward the middle of the north-western sector", #1
            "set waypoint to the center of northwest zone", #2
            "head to the middle NW area and proceed", #3
            "advance to the center of the north-western quadrant", #4
            "move toward middle of the NW sector region", #5
            "place a target in the center northwest zone", #6
            "reach the middle point of the north-western area", #7
            "navigate to center of the NW quadrant region", #8
            "go toward the middle northwestern sector area", #9
            "set destination to center of NW zone" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
        {
        "base": { # this is not go to NW goal but Goal North-East
            "memory": memory(excluded_missions="go to NE goal"),
            "command": "go to northeast area",
            "previous_response": "go to NW goal",
            "output": "go to NE goal"
        },
        "command_variations": [
            "go to north east area", #1
            "move to north east", #2
            "navigate to NE sector", #3
            "head to northeast zone", #4
            "reach the center of the NE area", #5
            "proceed to the center of the north-eastern area", #6
            "set route to the north eastern zone", #7
            "place a goal waypoint in the NE area", #8
            "set the target waypoint in the north east sector", #9
            "move to the center of the NE zone" #10
            ],
        "test_command_variations": [
            "navigate toward the middle of the north-eastern sector", #1
            "set waypoint to the center of northeast zone", #2
            "head to the middle NE area and proceed", #3
            "advance to the center of the north-eastern quadrant", #4
            "move toward middle of the NE sector region", #5
            "place a target in the center northeast zone", #6
            "reach the middle point of the north-eastern area", #7
            "navigate to center of the NE quadrant region", #8
            "go toward the middle northeastern sector area", #9
            "set destination to center of NE zone" #10
            ],
        "clarification_variations": [
            "east", #1
            "northeast", #2
            "eastern", #3
            "northeastern", #4
            "E", #5
            "NE", #6
            "I said eastern", #7
            "north eastern was the command", #8
            "no, east", #9
            "no, northeast" #10
            ]
    },
    {
        "base": { # confirm Survey South-East
            "memory": memory(excluded_missions="SE quadrant survey"),
            "command": "survey the southeast area",
            "previous_response": "SE quadrant survey",
            "output": "SE quadrant survey"
        },
        "command_variations": [
            "survey the south east area", #1
            "scan south east sector", #2
            "explore SE zone", #3
            "perform survey in south east", #4
            "see what you can find in the SE area", #5
            "take a look around in the south eastern sector", #6
            "search the south east area", #7
            "perform a survey in the SE section", #8
            "survey the south eastern zone", #9
            "explore the south east area" #10
            ],
        "test_command_variations": [
            "conduct a thorough survey of the south-eastern sector region", #1
            "scan and map the entire southeast zone area", #2
            "perform a complete survey across the SE sector", #3
            "explore the southeast area with detailed scanning", #4
            "execute a comprehensive scan in the south-eastern zone", #5
            "survey the central southeast sector thoroughly", #6
            "begin full exploration and mapping of the SE area", #7
            "initiate complete scan of the southeastern region", #8
            "perform detailed survey across the southeast zone", #9
            "conduct extensive mapping of the south-eastern sector" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    {
        "base": { # this is not SE quadrant survey but Survey South-West
            "memory": memory(excluded_missions="SW quadrant survey"),
            "command": "survey the southwest area",
            "previous_response": "SE quadrant survey",
            "output": "SW quadrant survey"
        },
        "command_variations": [
            "survey the south west area", #1
            "scan south west sector", #2
            "explore SW zone", #3
            "perform survey in south west", #4
            "see what you can find in the SW area", #5
            "take a look around in the south-western sector", #6
            "search the south west area", #7
            "perform a survey in the SW section", #8
            "survey the south-western zone", #9
            "explore the south west area" #10
            ],
        "test_command_variations": [
            "conduct a thorough survey of the south-western sector region", #1
            "scan and map the entire southwest zone area", #2
            "perform a complete survey across the SW sector", #3
            "explore the southwest area with detailed scanning", #4
            "execute a comprehensive scan in the south-western zone", #5
            "survey the central southwest sector thoroughly", #6
            "begin full exploration and mapping of the SW area", #7
            "initiate complete scan of the southwestern region", #8
            "perform detailed survey across the southwest zone", #9
            "conduct extensive mapping of the south-western sector" #10
            ],
        "clarification_variations": [
            "west", #1
            "southwest", #2
            "western", #3
            "southwestern", #4
            "W", #5
            "SW", #6
            "I said western", #7
            "south western was the command", #8
            "no, west", #9
            "no, southwest" #10
            ]

    },


    { # receive the waypoint is not go to received goal, but skip
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Receive and process the location of a waypoint",
            "previous_response": "go to received goal",
            "output": "skip"
        },
        "command_variations": [
            "Receive and process the location of a waypoint", #1
            "Accept and process the location of a waypoint", #2
            "Obtain and process the location of a waypoint", #3
            "Acquire and process the location of a waypoint", #4
            "Retrieve and process the location of a waypoint", #5
            "Collect and process the location of a waypoint", #6
            "Gather and process the location of a waypoint", #7
            "Capture and process the location of a waypoint", #8
            "Get and process the location of a waypoint", #9
            "Take and process the location of a waypoint" #10
            ],
        "test_command_variations": [
            "receive and comprehensively process the specified waypoint location coordinates", #1
            "accept and properly handle the incoming waypoint location data", #2
            "obtain and accurately process the waypoint location information provided", #3
            "acquire and execute the waypoint location with appropriate handling", #4
            "retrieve and methodically process the waypoint location parameters", #5
            "collect and execute the waypoint location data in sequence", #6
            "gather and process the waypoint location coordinates systematically", #7
            "capture and handle the waypoint location with precision", #8
            "receive and execute the complete waypoint location information accurately", #9
            "accept and process the waypoint location data with full execution" #10
            ],
        "clarification_variations": [
            "you can't do it", #1
            "the system is not equipped for this", #2
            "we don't have this option", #3
            "no mission can fulfill the request", #4
            "there is no such option", #5
            "none of the available missions is fit", #6
            "not feasible", #7
            "not an option", #8
            "no mission can do this" #9
            "the system has not the necessary means" #10
            ]
    },
    { # survey pipeline is not central survey but skip
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Survey the pipeline structure ",
            "previous_response": "central survey",
            "output": "skip"
        },
        "command_variations": [
            "Survey the pipeline structure", #1
            "Inspect the pipeline structure", #2
            "Examine the pipeline structure", #3
            "Assess the pipeline structure", #4
            "Evaluate the pipeline structure", #5
            "Observe the pipeline structure", #6
            "Analyse the pipeline structure", #7
            "Review the pipeline structure", #8
            "Study the pipeline structure", #9
            "Investigate the pipeline structure" #10
            ],
        "test_command_variations": [
            "survey and inspect the pipeline structure comprehensively to evaluate the overall condition and integrity", #1
            "examine the pipeline infrastructure thoroughly and assess any potential damage or structural defects present", #2
            "analyse the pipeline structure in detail and evaluate the condition of all critical pipeline components", #3
            "investigate the pipeline infrastructure systematically and review the findings to determine maintenance requirements", #4
            "observe the pipeline structure carefully and conduct a detailed assessment of corrosion or wear patterns", #5
            "study the pipeline infrastructure and assess its structural integrity using comprehensive inspection methodologies", #6
            "examine and analyse the pipeline structure to evaluate its operational safety and infrastructure compliance", #7
            "review the pipeline infrastructure thoroughly and investigate any areas showing signs of deterioration or damage", #8
            "assess the pipeline structure comprehensively and evaluate the condition with detailed observations and analysis", #9
            "inspect the pipeline infrastructure systematically and examine all structural components for potential issues or degradation" #10
            ],
        "clarification_variations": [
            "you can't do it", #1
            "the system is not equipped for this", #2
            "we don't have this option", #3
            "no mission can fulfill the request", #4
            "there is no such option", #5
            "none of the available missions is fit", #6
            "not feasible", #7
            "not an option", #8
            "no mission can do this" #9
            "the system has not the necessary means" #10
            ]
    },

    # CONFIRMATION FOR ALL RAMI

    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Receive and process the location of a waypoint",
            "previous_response": "skip",
            "output": "skip"
        },
        "command_variations": [
            "Receive and process the location of a waypoint", #1
            "Accept and process the location of a waypoint", #2
            "Obtain and process the location of a waypoint", #3
            "Acquire and process the location of a waypoint", #4
            "Retrieve and process the location of a waypoint", #5
            "Collect and process the location of a waypoint", #6
            "Gather and process the location of a waypoint", #7
            "Capture and process the location of a waypoint", #8
            "Get and process the location of a waypoint", #9
            "Take and process the location of a waypoint" #10
            ],
        "test_command_variations": [
            "receive and comprehensively process the specified waypoint location coordinates", #1
            "accept and properly handle the incoming waypoint location data", #2
            "obtain and accurately process the waypoint location information provided", #3
            "acquire and execute the waypoint location with appropriate handling", #4
            "retrieve and methodically process the waypoint location parameters", #5
            "collect and execute the waypoint location data in sequence", #6
            "gather and process the waypoint location coordinates systematically", #7
            "capture and handle the waypoint location with precision", #8
            "receive and execute the complete waypoint location information accurately", #9
            "accept and process the waypoint location data with full execution" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Reach the waypoint received in underwater autonomous navigation",
            "previous_response": "go to received goal",
            "output": "go to received goal"
        },
        "command_variations": [
            "Reach the waypoint received", #1
            "Arrive at the waypoint received", #2
            "Navigate to the waypoint received", #3
            "Proceed to the waypoint received", #4
            "Travel to the waypoint received", #5
            "Move to the waypoint received", #6
            "Go to the waypoint received", #7
            "Advance to the waypoint received", #8
            "Progress to the waypoint received", #9
            "Head to the waypoint received" #10
            ],
        "test_command_variations": [
            "reach and arrive at the designated waypoint that was received", #1
            "navigate to the waypoint obtained and proceed toward its location", #2
            "advance to the transmitted waypoint coordinates that were received", #3
            "travel to the specified waypoint location obtained from the system", #4
            "move toward and reach the waypoint data that was received", #5
            "go to the waypoint received and execute arrival at the coordinates", #6
            "progress toward the designated waypoint that was obtained and transmitted", #7
            "approach and attain the waypoint location received from the command", #8
            "navigate to the specified waypoint received with full execution to arrival", #9
            "reach the waypoint coordinates obtained and complete transit to the location" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, required_missions="go to received goal", total_missions=1),
            "command": "-	Pass through the gate",
            "previous_response": "cross gate",
            "output": "cross gate"
        },
        "command_variations": [
            "pass through the gate", #1
            "transit through the gate", #2
            "navigate through the gate", #3
            "proceed through the gate", #4
            "move through the gate", #5
            "traverse through the gate", #6
            "go through the gate", #7
            "advance through the gate", #8
            "travel through the gate", #9
            "cross through the gate" #10
            ],
        "test_command_variations": [
            "pass through the gate and advance between the marked yellow buoys", #1
            "navigate through the gate structure while passing between the buoys", #2
            "traverse through the marked gate opening between the yellow buoys", #3
            "proceed through the gate passage and cross between the designated buoys", #4
            "transit through the buoy gate and pass between the marked markers", #5
            "move through the gate while navigating between the yellow buoys correctly", #6
            "go through the marked gate entrance and advance between the buoy markers", #7
            "cross through the gate passage situated between the yellow buoys", #8
            "navigate the gate structure by passing precisely between the marked buoys", #9
            "travel through the designated gate opening positioned between the yellow markers" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    { # confirm map the buoy area A
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "previous_response": "map buoy area A",
            "output": "map buoy area A"
        },
        "command_variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions" #10
            ],
        "test_command_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    { # confirm perform the moves A
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "previous_response": "make move A",
            "output": "make move A"
        },
        "command_variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10

            # "perform the specific move for each buoy",
            # "for each buoy perform the corresponding move",
            # "make the corresponding move for every buoy",
            # "perform the adequate action for every buoy",
            # "execute the specific move for each buoy",
            # "for each buoy execute the corresponding move",
            # "make the appropriate move for every buoy",
            # "perform the proper action for every buoy",
            # "carry out the specific move for each buoy",
            # "for each buoy carry out the corresponding move",
            # "make the suitable move for every buoy" #20
            ],
        "test_command_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses", #10
            
            # "perform the specific maneuver associated with each buoy in the sequence",
            # "for each buoy encountered, execute the corresponding predefined move",
            # "carry out the appropriate action designated for every individual buoy",
            # "perform the particular move that corresponds to each buoy's identifier",
            # "for every buoy in the mission, make the specific move assigned to it",
            # "execute the distinct action appropriate for each buoy along the path",
            # "perform the designated maneuver tailored to each buoy's requirements",
            # "for each buoy detected, carry out its corresponding operational move",
            # "make the specific move prescribed for every buoy in the mission plan",
            # "perform the action that matches each buoy's designated procedure" #20

            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },

    { # confirm map the buoy area B
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "previous_response": "map buoy area B",
            "output": "map buoy area B"
        },
        "command_variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions" #10
            ],
        "test_command_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ],
        "failure_context": get_failure_context("find")
    },
    { # confirm perform the moves B
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "previous_response": "make move B",
            "output": "make move B"
        },
        "command_variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode" #10
            ],
        "test_command_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ],
        "failure_context": get_failure_context("move")
    },
    { 
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "-	Survey the pipeline structure ",
            "previous_response": "skip",
            "output": "skip"
        },
        "command_variations": [
            "Survey the pipeline structure", #1
            "Inspect the pipeline structure", #2
            "Examine the pipeline structure", #3
            "Assess the pipeline structure", #4
            "Evaluate the pipeline structure", #5
            "Observe the pipeline structure", #6
            "Analyse the pipeline structure", #7
            "Review the pipeline structure", #8
            "Study the pipeline structure", #9
            "Investigate the pipeline structure" #10
            ],
        "test_command_variations": [
            "survey and inspect the pipeline structure comprehensively to evaluate the overall condition and integrity", #1
            "examine the pipeline infrastructure thoroughly and assess any potential damage or structural defects present", #2
            "analyse the pipeline structure in detail and evaluate the condition of all critical pipeline components", #3
            "investigate the pipeline infrastructure systematically and review the findings to determine maintenance requirements", #4
            "observe the pipeline structure carefully and conduct a detailed assessment of corrosion or wear patterns", #5
            "study the pipeline infrastructure and assess its structural integrity using comprehensive inspection methodologies", #6
            "examine and analyse the pipeline structure to evaluate its operational safety and infrastructure compliance", #7
            "review the pipeline infrastructure thoroughly and investigate any areas showing signs of deterioration or damage", #8
            "assess the pipeline structure comprehensively and evaluate the condition with detailed observations and analysis", #9
            "inspect the pipeline infrastructure systematically and examine all structural components for potential issues or degradation" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "Detect, localize and determining the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure",
            "previous_response": "skip",
            "output": "skip"
        },
        "command_variations": [
            "Detect, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #1
            "Identify, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #2
            "Recognise, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #3
            "Locate, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #4
            "Find, localise and determine the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #5
            "Detect, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #6
            "Identify, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #7
            "Recognise, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #8
            "Locate, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #9
            "Find, localise and establish the dimensions of the damage (a red marker) positioned on one of the two pipes connected to one pipeline structure", #10
            ],
        "test_command_variations": [
            "detect, localise and precisely determine the exact dimensions of the damage marked by a red marker positioned on one of the two pipes within the connected pipeline structure", #1
            "identify and localise the red marker damage, then establish and measure the dimensional characteristics on the interconnected pipeline structure", #2
            "recognise the red damage marker on either pipe of the pipeline structure, localise its position, and ascertain its dimensional parameters accurately", #3
            "locate the red marker indicating damage on one of the two connected pipes and determine the precise dimensions of the identified damage area", #4
            "find, localise and comprehensively measure the dimensions of the red-marked damage situated on one of the dual pipes in the pipeline infrastructure", #5
            "detect the red damage marker on the interconnected pipeline structure, establish its localisation point, and determine its dimensional extent", #6
            "identify the red marker damage position on either pipe, localise it precisely, and measure the exact dimensions of the damage structure", #7
            "recognise and localise the damage indicated by the red marker on one of the two connected pipes, then ascertain the complete dimensional information", #8
            "locate the red-marked damage on the pipeline structure's dual pipes, localise it systematically, and determine all relevant damage dimensions", #9
            "comprehensively detect the red marker damage on one of the connected pipes, localise it within the pipeline structure, and measure its dimensional characteristics" #10
            ],
        "clarification_variations": [
            "ok", #1
            "yes", #2
            "correct", #3
            "right", #4
            "this is ok", #5
            "this is correct", #6
            "the previous response is ok", #7
            "the previous response is right", #8
            "the previous response is correct", #9
            "you did good" #10
            ]
    }
]

# ============================================================================================
#                                   FAILURE EXAMPLES
# ============================================================================================

failure_variations_data = [
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow"], total_buoys=2, required_missions=["go to received goal", "cross gate"], total_missions=2),
            "command": "",
            "failure_context": get_failure_context("find"),
            "output": "map buoy area B"
        },
        "command_variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions", #10
            "Map the coloured buoy area delivering a geo-localised map with the buoy positions", #11
            "Chart the coloured buoy area delivering a geo-localised map with the buoy positions", #12
            "Survey the coloured buoy area delivering a geo-localised map with the buoy positions", #13
            "Document the coloured buoy area delivering a geo-localised map with the buoy positions", #14
            "Plot the coloured buoy area delivering a geo-localised map with the buoy positions", #15
            "Map the coloured buoy area generating a geo-localised map with the buoy positions", #16
            "Chart the coloured buoy area generating a geo-localised map with the buoy positions", #17
            "Survey the coloured buoy area generating a geo-localised map with the buoy positions", #18
            "Document the coloured buoy area generating a geo-localised map with the buoy positions", #19
            "Plot the coloured buoy area generating a geo-localised map with the buoy positions", #20
            "Map the coloured buoy area producing a geo-localised map with the buoy positions", #21
            "Chart the coloured buoy area producing a geo-localised map with the buoy positions", #22
            "Survey the coloured buoy area producing a geo-localised map with the buoy positions", #23
            "Document the coloured buoy area producing a geo-localised map with the buoy positions", #24
            "Plot the coloured buoy area producing a geo-localised map with the buoy positions", #25
            "Map the coloured buoy area creating a geo-localised map with the buoy positions", #26
            "Chart the coloured buoy area creating a geo-localised map with the buoy positions", #27
            "Survey the coloured buoy area creating a geo-localised map with the buoy positions", #28
            "Document the coloured buoy area creating a geo-localised map with the buoy positions", #29
            "Plot the coloured buoy area creating a geo-localised map with the buoy positions", #30
            "Map the coloured buoy zone providing a geo-localised map with the buoy positions", #31
            "Chart the coloured buoy zone providing a geo-localised map with the buoy positions", #32
            "Survey the coloured buoy zone providing a geo-localised map with the buoy positions", #33
            "Document the coloured buoy zone providing a geo-localised map with the buoy positions", #34
            "Plot the coloured buoy zone providing a geo-localised map with the buoy positions", #35
            "Map the coloured buoy region providing a geo-localised map with the buoy positions", #36
            "Chart the coloured buoy region providing a geo-localised map with the buoy positions", #37
            "Survey the coloured buoy region providing a geo-localised map with the buoy positions", #38
            "Document the coloured buoy region providing a geo-localised map with the buoy positions", #39
            "Plot the coloured buoy region providing a geo-localised map with the buoy positions", #40
            "Map the coloured buoy sector providing a geo-localised map with the buoy positions", #41
            "Chart the coloured buoy sector providing a geo-localised map with the buoy positions", #42
            "Survey the coloured buoy sector providing a geo-localised map with the buoy positions", #43
            "Document the coloured buoy sector providing a geo-localised map with the buoy positions", #44
            "Plot the coloured buoy sector providing a geo-localised map with the buoy positions", #45
            "Map the coloured buoy field providing a geo-localised map with the buoy positions", #46
            "Chart the coloured buoy field providing a geo-localised map with the buoy positions", #47
            "Survey the coloured buoy field providing a geo-localised map with the buoy positions", #48
            "Document the coloured buoy field providing a geo-localised map with the buoy positions", #49
            "Plot the coloured buoy field providing a geo-localised map with the buoy positions", #50
            "Map the coloured buoy area providing a geo-referenced map with the buoy positions", #51
            "Chart the coloured buoy area providing a geo-referenced map with the buoy positions", #52
            "Survey the coloured buoy area providing a geo-referenced map with the buoy positions", #53
            "Document the coloured buoy area providing a geo-referenced map with the buoy positions", #54
            "Plot the coloured buoy area providing a geo-referenced map with the buoy positions", #55
            "Map the coloured buoy area providing a geo-tagged map with the buoy positions", #56
            "Chart the coloured buoy area providing a geo-tagged map with the buoy positions", #57
            "Survey the coloured buoy area providing a geo-tagged map with the buoy positions", #58
            "Document the coloured buoy area providing a geo-tagged map with the buoy positions", #59
            "Plot the coloured buoy area providing a geo-tagged map with the buoy positions", #60
            "Map the coloured buoy area providing a spatially-referenced map with the buoy positions", #61
            "Chart the coloured buoy area providing a spatially-referenced map with the buoy positions", #62
            "Survey the coloured buoy area providing a spatially-referenced map with the buoy positions", #63
            "Document the coloured buoy area providing a spatially-referenced map with the buoy positions", #64
            "Plot the coloured buoy area providing a spatially-referenced map with the buoy positions", #65
            "Map the coloured buoy area providing a position-stamped map with the buoy positions", #66
            "Chart the coloured buoy area providing a position-stamped map with the buoy positions", #67
            "Survey the coloured buoy area providing a position-stamped map with the buoy positions", #68
            "Document the coloured buoy area providing a position-stamped map with the buoy positions", #69
            "Plot the coloured buoy area providing a position-stamped map with the buoy positions", #70
            "Map the coloured buoy area providing a location-stamped map with the buoy positions", #71
            "Chart the coloured buoy area providing a location-stamped map with the buoy positions", #72
            "Survey the coloured buoy area providing a location-stamped map with the buoy positions", #73
            "Document the coloured buoy area providing a location-stamped map with the buoy positions", #74
            "Plot the coloured buoy area providing a location-stamped map with the buoy positions", #75
            "Map the coloured buoy area providing a coordinate-tagged map with the buoy positions", #76
            "Chart the coloured buoy area providing a coordinate-tagged map with the buoy positions", #77
            "Survey the coloured buoy area providing a coordinate-tagged map with the buoy positions", #78
            "Document the coloured buoy area providing a coordinate-tagged map with the buoy positions", #79
            "Plot the coloured buoy area providing a coordinate-tagged map with the buoy positions", #80
            "Map the coloured buoy area and provide a geo-localised map with the buoy positions", #81
            "Chart the coloured buoy area and provide a geo-localised map with the buoy positions", #82
            "Survey the coloured buoy area and provide a geo-localised map with the buoy positions", #83
            "Document the coloured buoy area and provide a geo-localised map with the buoy positions", #84
            "Plot the coloured buoy area and provide a geo-localised map with the buoy positions", #85
            "Map the coloured buoy area and supply a geo-localised map with the buoy positions", #86
            "Chart the coloured buoy area and supply a geo-localised map with the buoy positions", #87
            "Survey the coloured buoy area and supply a geo-localised map with the buoy positions", #88
            "Document the coloured buoy area and supply a geo-localised map with the buoy positions", #89
            "Plot the coloured buoy area and supply a geo-localised map with the buoy positions", #90
            "Map the coloured buoy area and deliver a geo-localised map with the buoy positions", #91
            "Chart the coloured buoy area and deliver a geo-localised map with the buoy positions", #92
            "Survey the coloured buoy area and deliver a geo-localised map with the buoy positions", #93
            "Document the coloured buoy area and deliver a geo-localised map with the buoy positions", #94
            "Plot the coloured buoy area and deliver a geo-localised map with the buoy positions", #95
            "Map the coloured buoy area and generate a geo-localised map with the buoy positions", #96
            "Chart the coloured buoy area and generate a geo-localised map with the buoy positions", #97
            "Survey the coloured buoy area and generate a geo-localised map with the buoy positions", #98
            "Document the coloured buoy area and generate a geo-localised map with the buoy positions", #99
            "Plot the coloured buoy area and generate a geo-localised map with the buoy positions", #100
            ],
        "test_command_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "",
            "failure_context": get_failure_context("find"),
            "output": "map buoy area B"
        },
        "command_variations": [
            "Map the coloured buoy area providing a geo-localised map with the buoy positions", #1
            "Chart the coloured buoy area providing a geo-localised map with the buoy positions", #2
            "Survey the coloured buoy area providing a geo-localised map with the buoy positions", #3
            "Document the coloured buoy area providing a geo-localised map with the buoy positions", #4
            "Plot the coloured buoy area providing a geo-localised map with the buoy positions", #5
            "Map the coloured buoy area supplying a geo-localised map with the buoy positions", #6
            "Chart the coloured buoy area supplying a geo-localised map with the buoy positions", #7
            "Survey the coloured buoy area supplying a geo-localised map with the buoy positions", #8
            "Document the coloured buoy area supplying a geo-localised map with the buoy positions", #9
            "Plot the coloured buoy area supplying a geo-localised map with the buoy positions", #10
            "Map the coloured buoy area delivering a geo-localised map with the buoy positions", #11
            "Chart the coloured buoy area delivering a geo-localised map with the buoy positions", #12
            "Survey the coloured buoy area delivering a geo-localised map with the buoy positions", #13
            "Document the coloured buoy area delivering a geo-localised map with the buoy positions", #14
            "Plot the coloured buoy area delivering a geo-localised map with the buoy positions", #15
            "Map the coloured buoy area generating a geo-localised map with the buoy positions", #16
            "Chart the coloured buoy area generating a geo-localised map with the buoy positions", #17
            "Survey the coloured buoy area generating a geo-localised map with the buoy positions", #18
            "Document the coloured buoy area generating a geo-localised map with the buoy positions", #19
            "Plot the coloured buoy area generating a geo-localised map with the buoy positions", #20
            "Map the coloured buoy area producing a geo-localised map with the buoy positions", #21
            "Chart the coloured buoy area producing a geo-localised map with the buoy positions", #22
            "Survey the coloured buoy area producing a geo-localised map with the buoy positions", #23
            "Document the coloured buoy area producing a geo-localised map with the buoy positions", #24
            "Plot the coloured buoy area producing a geo-localised map with the buoy positions", #25
            "Map the coloured buoy area creating a geo-localised map with the buoy positions", #26
            "Chart the coloured buoy area creating a geo-localised map with the buoy positions", #27
            "Survey the coloured buoy area creating a geo-localised map with the buoy positions", #28
            "Document the coloured buoy area creating a geo-localised map with the buoy positions", #29
            "Plot the coloured buoy area creating a geo-localised map with the buoy positions", #30
            "Map the coloured buoy zone providing a geo-localised map with the buoy positions", #31
            "Chart the coloured buoy zone providing a geo-localised map with the buoy positions", #32
            "Survey the coloured buoy zone providing a geo-localised map with the buoy positions", #33
            "Document the coloured buoy zone providing a geo-localised map with the buoy positions", #34
            "Plot the coloured buoy zone providing a geo-localised map with the buoy positions", #35
            "Map the coloured buoy region providing a geo-localised map with the buoy positions", #36
            "Chart the coloured buoy region providing a geo-localised map with the buoy positions", #37
            "Survey the coloured buoy region providing a geo-localised map with the buoy positions", #38
            "Document the coloured buoy region providing a geo-localised map with the buoy positions", #39
            "Plot the coloured buoy region providing a geo-localised map with the buoy positions", #40
            "Map the coloured buoy sector providing a geo-localised map with the buoy positions", #41
            "Chart the coloured buoy sector providing a geo-localised map with the buoy positions", #42
            "Survey the coloured buoy sector providing a geo-localised map with the buoy positions", #43
            "Document the coloured buoy sector providing a geo-localised map with the buoy positions", #44
            "Plot the coloured buoy sector providing a geo-localised map with the buoy positions", #45
            "Map the coloured buoy field providing a geo-localised map with the buoy positions", #46
            "Chart the coloured buoy field providing a geo-localised map with the buoy positions", #47
            "Survey the coloured buoy field providing a geo-localised map with the buoy positions", #48
            "Document the coloured buoy field providing a geo-localised map with the buoy positions", #49
            "Plot the coloured buoy field providing a geo-localised map with the buoy positions", #50
            "Map the coloured buoy area providing a geo-referenced map with the buoy positions", #51
            "Chart the coloured buoy area providing a geo-referenced map with the buoy positions", #52
            "Survey the coloured buoy area providing a geo-referenced map with the buoy positions", #53
            "Document the coloured buoy area providing a geo-referenced map with the buoy positions", #54
            "Plot the coloured buoy area providing a geo-referenced map with the buoy positions", #55
            "Map the coloured buoy area providing a geo-tagged map with the buoy positions", #56
            "Chart the coloured buoy area providing a geo-tagged map with the buoy positions", #57
            "Survey the coloured buoy area providing a geo-tagged map with the buoy positions", #58
            "Document the coloured buoy area providing a geo-tagged map with the buoy positions", #59
            "Plot the coloured buoy area providing a geo-tagged map with the buoy positions", #60
            "Map the coloured buoy area providing a spatially-referenced map with the buoy positions", #61
            "Chart the coloured buoy area providing a spatially-referenced map with the buoy positions", #62
            "Survey the coloured buoy area providing a spatially-referenced map with the buoy positions", #63
            "Document the coloured buoy area providing a spatially-referenced map with the buoy positions", #64
            "Plot the coloured buoy area providing a spatially-referenced map with the buoy positions", #65
            "Map the coloured buoy area providing a position-stamped map with the buoy positions", #66
            "Chart the coloured buoy area providing a position-stamped map with the buoy positions", #67
            "Survey the coloured buoy area providing a position-stamped map with the buoy positions", #68
            "Document the coloured buoy area providing a position-stamped map with the buoy positions", #69
            "Plot the coloured buoy area providing a position-stamped map with the buoy positions", #70
            "Map the coloured buoy area providing a location-stamped map with the buoy positions", #71
            "Chart the coloured buoy area providing a location-stamped map with the buoy positions", #72
            "Survey the coloured buoy area providing a location-stamped map with the buoy positions", #73
            "Document the coloured buoy area providing a location-stamped map with the buoy positions", #74
            "Plot the coloured buoy area providing a location-stamped map with the buoy positions", #75
            "Map the coloured buoy area providing a coordinate-tagged map with the buoy positions", #76
            "Chart the coloured buoy area providing a coordinate-tagged map with the buoy positions", #77
            "Survey the coloured buoy area providing a coordinate-tagged map with the buoy positions", #78
            "Document the coloured buoy area providing a coordinate-tagged map with the buoy positions", #79
            "Plot the coloured buoy area providing a coordinate-tagged map with the buoy positions", #80
            "Map the coloured buoy area and provide a geo-localised map with the buoy positions", #81
            "Chart the coloured buoy area and provide a geo-localised map with the buoy positions", #82
            "Survey the coloured buoy area and provide a geo-localised map with the buoy positions", #83
            "Document the coloured buoy area and provide a geo-localised map with the buoy positions", #84
            "Plot the coloured buoy area and provide a geo-localised map with the buoy positions", #85
            "Map the coloured buoy area and supply a geo-localised map with the buoy positions", #86
            "Chart the coloured buoy area and supply a geo-localised map with the buoy positions", #87
            "Survey the coloured buoy area and supply a geo-localised map with the buoy positions", #88
            "Document the coloured buoy area and supply a geo-localised map with the buoy positions", #89
            "Plot the coloured buoy area and supply a geo-localised map with the buoy positions", #90
            "Map the coloured buoy area and deliver a geo-localised map with the buoy positions", #91
            "Chart the coloured buoy area and deliver a geo-localised map with the buoy positions", #92
            "Survey the coloured buoy area and deliver a geo-localised map with the buoy positions", #93
            "Document the coloured buoy area and deliver a geo-localised map with the buoy positions", #94
            "Plot the coloured buoy area and deliver a geo-localised map with the buoy positions", #95
            "Map the coloured buoy area and generate a geo-localised map with the buoy positions", #96
            "Chart the coloured buoy area and generate a geo-localised map with the buoy positions", #97
            "Survey the coloured buoy area and generate a geo-localised map with the buoy positions", #98
            "Document the coloured buoy area and generate a geo-localised map with the buoy positions", #99
            "Plot the coloured buoy area and generate a geo-localised map with the buoy positions", #100
            ],
        "test_command_variations": [
            "map and chart the coloured buoy area while generating a comprehensive geo-localised map with all buoy positions", #1
            "survey the coloured buoy area and deliver a detailed geo-localised map showing the exact positions of all buoys", #2
            "document the coloured buoy area by providing a precise geo-localised map with marked buoy location coordinates", #3
            "plot the coloured buoy area and supply a complete geo-localised map including all buoy positions and spatial references", #4
            "chart the coloured buoy area delivering a geo-localised map that accurately represents all colored buoy positions", #5
            "map the coloured buoy area generating a comprehensive geo-localised map with precise buoy position data", #6
            "survey and document the coloured buoy area providing a geo-localised map with complete buoy position information", #7
            "plot the coloured buoy area while supplying a detailed geo-localised map with all buoy locations and coordinates", #8
            "chart and map the coloured buoy area delivering a geo-localised map showing precise positions of all marked buoys", #9
            "document the coloured buoy area by generating a comprehensive geo-localised map with accurate buoy position references" #10
            ]
    },
    {
        "base": {
            "memory": memory(required_buoys=["yellow", "yellow"], total_buoys=2, required_missions=["go to received goal", "cross gate"], total_missions=2),
            "command": "",
            "failure_context": get_failure_context("move"),
            "output": "make move B"
        },
        "command_variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10
            "Detect the colours of each buoy and act in a specific mode", #11
            "Identify the colours of each buoy and act in a specific mode", #12
            "Recognise the colours of each buoy and act in a specific mode", #13
            "Determine the colours of each buoy and act in a specific mode", #14
            "Sense the colours of each buoy and act in a specific mode", #15
            "Detect the colours of each buoy and operate in a specific mode", #16
            "Identify the colours of each buoy and operate in a specific mode", #17
            "Recognise the colours of each buoy and operate in a specific mode", #18
            "Determine the colours of each buoy and operate in a specific mode", #19
            "Sense the colours of each buoy and operate in a specific mode", #20
            "Detect the colours of each buoy and function in a specific mode", #21
            "Identify the colours of each buoy and function in a specific mode", #22
            "Recognise the colours of each buoy and function in a specific mode", #23
            "Determine the colours of each buoy and function in a specific mode", #24
            "Sense the colours of each buoy and function in a specific mode", #25
            "Detect the colours of each buoy and perform in a specific mode", #26
            "Identify the colours of each buoy and perform in a specific mode", #27
            "Recognise the colours of each buoy and perform in a specific mode", #28
            "Determine the colours of each buoy and perform in a specific mode", #29
            "Sense the colours of each buoy and perform in a specific mode", #30
            "Detect the colours of each buoy and behave in a specific mode", #31
            "Identify the colours of each buoy and behave in a specific mode", #32
            "Recognise the colours of each buoy and behave in a specific mode", #33
            "Determine the colours of each buoy and behave in a specific mode", #34
            "Sense the colours of each buoy and behave in a specific mode", #35
            "Detect the colours of each buoy and execute in a specific mode", #36
            "Identify the colours of each buoy and execute in a specific mode", #37
            "Recognise the colours of each buoy and execute in a specific mode", #38
            "Determine the colours of each buoy and execute in a specific mode", #39
            "Sense the colours of each buoy and execute in a specific mode", #40
            "Detect the hues of each buoy and react in a specific mode", #41
            "Identify the hues of each buoy and react in a specific mode", #42
            "Recognise the hues of each buoy and react in a specific mode", #43
            "Determine the hues of each buoy and react in a specific mode", #44
            "Sense the hues of each buoy and react in a specific mode", #45
            "Detect the colouration of each buoy and react in a specific mode", #46
            "Identify the colouration of each buoy and react in a specific mode", #47
            "Recognise the colouration of each buoy and react in a specific mode", #48
            "Determine the colouration of each buoy and react in a specific mode", #49
            "Sense the colouration of each buoy and react in a specific mode", #50
            "Detect the colour coding of each buoy and react in a specific mode", #51
            "Identify the colour coding of each buoy and react in a specific mode", #52
            "Recognise the colour coding of each buoy and react in a specific mode", #53
            "Determine the colour coding of each buoy and react in a specific mode", #54
            "Sense the colour coding of each buoy and react in a specific mode", #55
            "Detect the colour markers of each buoy and react in a specific mode", #56
            "Identify the colour markers of each buoy and react in a specific mode", #57
            "Recognise the colour markers of each buoy and react in a specific mode", #58
            "Determine the colour markers of each buoy and react in a specific mode", #59
            "Sense the colour markers of each buoy and react in a specific mode", #60
            "Detect the colours of every buoy and react in a specific mode", #61
            "Identify the colours of every buoy and react in a specific mode", #62
            "Recognise the colours of every buoy and react in a specific mode", #63
            "Determine the colours of every buoy and react in a specific mode", #64
            "Sense the colours of every buoy and react in a specific mode", #65
            "Detect the colours of all buoys and react in a specific mode", #66
            "Identify the colours of all buoys and react in a specific mode", #67
            "Recognise the colours of all buoys and react in a specific mode", #68
            "Determine the colours of all buoys and react in a specific mode", #69
            "Sense the colours of all buoys and react in a specific mode", #70
            "Detect buoy colours and react in a specific mode", #71
            "Identify buoy colours and react in a specific mode", #72
            "Recognise buoy colours and react in a specific mode", #73
            "Determine buoy colours and react in a specific mode", #74
            "Sense buoy colours and react in a specific mode", #75
            "Detect the colour of each buoy and react in a specific mode", #76
            "Identify the colour of each buoy and react in a specific mode", #77
            "Recognise the colour of each buoy and react in a specific mode", #78
            "Determine the colour of each buoy and react in a specific mode", #79
            "Sense the colour of each buoy and react in a specific mode", #80
            "Detect the colours on each buoy and react in a specific mode", #81
            "Identify the colours on each buoy and react in a specific mode", #82
            "Recognise the colours on each buoy and react in a specific mode", #83
            "Determine the colours on each buoy and react in a specific mode", #84
            "Sense the colours on each buoy and react in a specific mode", #85
            "Detect each buoy colour and react in a specific mode", #86
            "Identify each buoy colour and react in a specific mode", #87
            "Recognise each buoy colour and react in a specific mode", #88
            "Determine each buoy colour and react in a specific mode", #89
            "Sense each buoy colour and react in a specific mode", #90
            "Detect the colours per buoy and react in a specific mode", #91
            "Identify the colours per buoy and react in a specific mode", #92
            "Recognise the colours per buoy and react in a specific mode", #93
            "Determine the colours per buoy and react in a specific mode", #94
            "Sense the colours per buoy and react in a specific mode", #95
            "Detect colour for each buoy and react in a specific mode", #96
            "Identify colour for each buoy and react in a specific mode", #97
            "Recognise colour for each buoy and react in a specific mode", #98
            "Determine colour for each buoy and react in a specific mode", #99
            "Sense colour for each buoy and react in a specific mode", #100
            ],
        "test_command_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses" #10
            ]
    },
    {
        "base": {
            "memory": memory(total_buoys=0, total_missions=0),
            "command": "",
            "failure_context": get_failure_context("move"),
            "output": "make move B"
        },
        "command_variations": [
            "Detect the colours of each buoy and react in a specific mode", #1
            "Identify the colours of each buoy and react in a specific mode", #2
            "Recognise the colours of each buoy and react in a specific mode", #3
            "Determine the colours of each buoy and react in a specific mode", #4
            "Sense the colours of each buoy and react in a specific mode", #5
            "Detect the colours of each buoy and respond in a specific mode", #6
            "Identify the colours of each buoy and respond in a specific mode", #7
            "Recognise the colours of each buoy and respond in a specific mode", #8
            "Determine the colours of each buoy and respond in a specific mode", #9
            "Sense the colours of each buoy and respond in a specific mode", #10
            "Detect the colours of each buoy and act in a specific mode", #11
            "Identify the colours of each buoy and act in a specific mode", #12
            "Recognise the colours of each buoy and act in a specific mode", #13
            "Determine the colours of each buoy and act in a specific mode", #14
            "Sense the colours of each buoy and act in a specific mode", #15
            "Detect the colours of each buoy and operate in a specific mode", #16
            "Identify the colours of each buoy and operate in a specific mode", #17
            "Recognise the colours of each buoy and operate in a specific mode", #18
            "Determine the colours of each buoy and operate in a specific mode", #19
            "Sense the colours of each buoy and operate in a specific mode", #20
            "Detect the colours of each buoy and function in a specific mode", #21
            "Identify the colours of each buoy and function in a specific mode", #22
            "Recognise the colours of each buoy and function in a specific mode", #23
            "Determine the colours of each buoy and function in a specific mode", #24
            "Sense the colours of each buoy and function in a specific mode", #25
            "Detect the colours of each buoy and perform in a specific mode", #26
            "Identify the colours of each buoy and perform in a specific mode", #27
            "Recognise the colours of each buoy and perform in a specific mode", #28
            "Determine the colours of each buoy and perform in a specific mode", #29
            "Sense the colours of each buoy and perform in a specific mode", #30
            "Detect the colours of each buoy and behave in a specific mode", #31
            "Identify the colours of each buoy and behave in a specific mode", #32
            "Recognise the colours of each buoy and behave in a specific mode", #33
            "Determine the colours of each buoy and behave in a specific mode", #34
            "Sense the colours of each buoy and behave in a specific mode", #35
            "Detect the colours of each buoy and execute in a specific mode", #36
            "Identify the colours of each buoy and execute in a specific mode", #37
            "Recognise the colours of each buoy and execute in a specific mode", #38
            "Determine the colours of each buoy and execute in a specific mode", #39
            "Sense the colours of each buoy and execute in a specific mode", #40
            "Detect the hues of each buoy and react in a specific mode", #41
            "Identify the hues of each buoy and react in a specific mode", #42
            "Recognise the hues of each buoy and react in a specific mode", #43
            "Determine the hues of each buoy and react in a specific mode", #44
            "Sense the hues of each buoy and react in a specific mode", #45
            "Detect the colouration of each buoy and react in a specific mode", #46
            "Identify the colouration of each buoy and react in a specific mode", #47
            "Recognise the colouration of each buoy and react in a specific mode", #48
            "Determine the colouration of each buoy and react in a specific mode", #49
            "Sense the colouration of each buoy and react in a specific mode", #50
            "Detect the colour coding of each buoy and react in a specific mode", #51
            "Identify the colour coding of each buoy and react in a specific mode", #52
            "Recognise the colour coding of each buoy and react in a specific mode", #53
            "Determine the colour coding of each buoy and react in a specific mode", #54
            "Sense the colour coding of each buoy and react in a specific mode", #55
            "Detect the colour markers of each buoy and react in a specific mode", #56
            "Identify the colour markers of each buoy and react in a specific mode", #57
            "Recognise the colour markers of each buoy and react in a specific mode", #58
            "Determine the colour markers of each buoy and react in a specific mode", #59
            "Sense the colour markers of each buoy and react in a specific mode", #60
            "Detect the colours of every buoy and react in a specific mode", #61
            "Identify the colours of every buoy and react in a specific mode", #62
            "Recognise the colours of every buoy and react in a specific mode", #63
            "Determine the colours of every buoy and react in a specific mode", #64
            "Sense the colours of every buoy and react in a specific mode", #65
            "Detect the colours of all buoys and react in a specific mode", #66
            "Identify the colours of all buoys and react in a specific mode", #67
            "Recognise the colours of all buoys and react in a specific mode", #68
            "Determine the colours of all buoys and react in a specific mode", #69
            "Sense the colours of all buoys and react in a specific mode", #70
            "Detect buoy colours and react in a specific mode", #71
            "Identify buoy colours and react in a specific mode", #72
            "Recognise buoy colours and react in a specific mode", #73
            "Determine buoy colours and react in a specific mode", #74
            "Sense buoy colours and react in a specific mode", #75
            "Detect the colour of each buoy and react in a specific mode", #76
            "Identify the colour of each buoy and react in a specific mode", #77
            "Recognise the colour of each buoy and react in a specific mode", #78
            "Determine the colour of each buoy and react in a specific mode", #79
            "Sense the colour of each buoy and react in a specific mode", #80
            "Detect the colours on each buoy and react in a specific mode", #81
            "Identify the colours on each buoy and react in a specific mode", #82
            "Recognise the colours on each buoy and react in a specific mode", #83
            "Determine the colours on each buoy and react in a specific mode", #84
            "Sense the colours on each buoy and react in a specific mode", #85
            "Detect each buoy colour and react in a specific mode", #86
            "Identify each buoy colour and react in a specific mode", #87
            "Recognise each buoy colour and react in a specific mode", #88
            "Determine each buoy colour and react in a specific mode", #89
            "Sense each buoy colour and react in a specific mode", #90
            "Detect the colours per buoy and react in a specific mode", #91
            "Identify the colours per buoy and react in a specific mode", #92
            "Recognise the colours per buoy and react in a specific mode", #93
            "Determine the colours per buoy and react in a specific mode", #94
            "Sense the colours per buoy and react in a specific mode", #95
            "Detect colour for each buoy and react in a specific mode", #96
            "Identify colour for each buoy and react in a specific mode", #97
            "Recognise colour for each buoy and react in a specific mode", #98
            "Determine colour for each buoy and react in a specific mode", #99
            "Sense colour for each buoy and react in a specific mode", #100
            ],
        "test_command_variations": [
            "detect and identify the specific colours of each individual buoy and respond accordingly in the designated operational mode", #1
            "recognise the colours of each buoy and react dynamically in a specific predetermined mode", #2
            "determine the exact colours of each buoy present and execute the appropriate response in the specified mode", #3
            "sense the buoy colours accurately and operate in a specific mode based on the detected color information", #4
            "identify the colours of each buoy and activate the corresponding reaction protocol in the designated mode", #5
            "detect the colors of all buoys and respond intelligently in the specific operational mode selected", #6
            "recognise and process the buoy colours, then react appropriately in the specified operational mode", #7
            "determine the colours of each buoy within the area and execute actions in the designated response mode", #8
            "sense the buoy colors with precision and respond in the specific mode based on color detection results", #9
            "identify the exact colours of each buoy and operate in the designated mode with appropriate behavioral responses" #10
            ]
    }

]

# ============================================================================================
#                                   DUPLICATION EXAMPLES
# ============================================================================================

duplication_variations_data = [
    {
        "base": { # Goal North-West
            "memory": memory(excluded_missions="go to NW goal"),
            "command": "go to northwest area",
            "output": "go to NW goal",
            "previous_response": "go to NW goal"
        },
        "command_variations": [
            "go to northwest area", #1
            "move to northwest", #2
            "navigate to NW sector", #3
            "head to northwest zone", #4
            "reach the center of the NW area", #5
            "proceed to the center of the north-western area", #6
            "set route to the northwestern zone", #7
            "place a goal waypoint in the NW area", #8
            "set the target waypoint in the northwest sector", #9
            "move to the center of the NW zone" #10
            ],
        "test_command_variations": [
            "navigate toward the middle of the north-western sector", #1
            "set waypoint to the center of northwest zone", #2
            "head to the middle NW area and proceed", #3
            "advance to the center of the north-western quadrant", #4
            "move toward middle of the NW sector region", #5
            "place a target in the center northwest zone", #6
            "reach the middle point of the north-western area", #7
            "navigate to center of the NW quadrant region", #8
            "go toward the middle northwestern sector area", #9
            "set destination to center of NW zone" #10
            ],
        "duplication_variations": [
            "Please repeat the mission", #1
            "Do it again", #2
            "Execute the same mission", #3
            "Repeat that", #4
            "Once again", #5
            "yes", #6
            "affirmative" # 7
            "yes please" # 8
            "yes, again" # 9
            "start over" # 10
            ]
    },
        {
        "base": { # Goal North-West
            "memory": memory(excluded_missions="go to NW goal"),
            "command": "go to northwest area",
            "previous_response": "go to NW goal",
            "output": "skip"
        },
        "command_variations": [
            "go to northwest area", #1
            "move to northwest", #2
            "navigate to NW sector", #3
            "head to northwest zone", #4
            "reach the center of the NW area", #5
            "proceed to the center of the north-western area", #6
            "set route to the northwestern zone", #7
            "place a goal waypoint in the NW area", #8
            "set the target waypoint in the northwest sector", #9
            "move to the center of the NW zone" #10
            ],
        "test_command_variations": [
            "navigate toward the middle of the north-western sector", #1
            "set waypoint to the center of northwest zone", #2
            "head to the middle NW area and proceed", #3
            "advance to the center of the north-western quadrant", #4
            "move toward middle of the NW sector region", #5
            "place a target in the center northwest zone", #6
            "reach the middle point of the north-western area", #7
            "navigate to center of the NW quadrant region", #8
            "go toward the middle northwestern sector area", #9
            "set destination to center of NW zone" #10
            ],
        "duplication_variations": [
            "Do not repeat the mission", #1
            "Do not do it again", #2
            "Don't", #3
            "Move on", #4
            "No more", #5
            "no", #6
            "negative" # 7
            "no thanks" # 8
            "better not" # 9
            "wait" # 10
        ]
    },

    {
        "base": { # Survey South-East
            "memory": memory(excluded_missions="SE quadrant survey"),
            "command": "survey the southeast area",
            "previous_response": "SE quadrant survey",
            "output": "SE quadrant survey"
        },
        "command_variations": [
            "survey the southeast area", #1
            "scan southeast sector", #2
            "explore SE zone", #3
            "perform survey in southeast", #4
            "see what you can find in the SE area", #5
            "take a look around in the south-eastern sector", #6
            "search the southeast area", #7
            "perform a survey in the SE section", #8
            "survey the south-eastern zone", #9
            "explore the southeast area" #10
            ],
        "test_command_variations": [
            "conduct a thorough survey of the south-eastern sector region", #1
            "scan and map the entire southeast zone area", #2
            "perform a complete survey across the SE sector", #3
            "explore the southeast area with detailed scanning", #4
            "execute a comprehensive scan in the south-eastern zone", #5
            "survey the central southeast sector thoroughly", #6
            "begin full exploration and mapping of the SE area", #7
            "initiate complete scan of the southeastern region", #8
            "perform detailed survey across the southeast zone", #9
            "conduct extensive mapping of the south-eastern sector" #10
            ],
        "duplication_variations": [
            "Please repeat the mission", #1
            "Do it again", #2
            "Execute the same mission", #3
            "Repeat that", #4
            "Once again", #5
            "yes", #6
            "affirmative" # 7
            "yes please" # 8
            "yes, again" # 9
            "start over" # 10
            ]
    },
    {
        "base": { # Survey South-East
            "memory": memory(excluded_missions="SE quadrant survey"),
            "command": "survey the southeast area",
            "previous_response": "SE quadrant survey",
            "output": "skip"
        },
        "command_variations": [
            "survey the southeast area", #1
            "scan southeast sector", #2
            "explore SE zone", #3
            "perform survey in southeast", #4
            "see what you can find in the SE area", #5
            "take a look around in the south-eastern sector", #6
            "search the southeast area", #7
            "perform a survey in the SE section", #8
            "survey the south-eastern zone", #9
            "explore the southeast area" #10
            ],
        "test_command_variations": [
            "conduct a thorough survey of the south-eastern sector region", #1
            "scan and map the entire southeast zone area", #2
            "perform a complete survey across the SE sector", #3
            "explore the southeast area with detailed scanning", #4
            "execute a comprehensive scan in the south-eastern zone", #5
            "survey the central southeast sector thoroughly", #6
            "begin full exploration and mapping of the SE area", #7
            "initiate complete scan of the southeastern region", #8
            "perform detailed survey across the southeast zone", #9
            "conduct extensive mapping of the south-eastern sector" #10
            ],
        "duplication_variations": [
            "Do not repeat the mission", #1
            "Do not do it again", #2
            "Don't", #3
            "Move on", #4
            "No more", #5
            "no", #6
            "negative" # 7
            "no thanks" # 8
            "better not" # 9
            "wait" # 10
        ]
    },
]




# ========================================
# HELPER FUNCTION FOR BALANCING
# ========================================

def balance_dataset_by_pattern(examples_main: List[Dict[str, Any]],
                               examples_clarification: List[Dict[str, Any]],
                               examples_failure: List[Dict[str, Any]],
                               examples_duplication: List[Dict[str, Any]],
                               max_examples_per_pattern: int,
                               examples_ratio: float,
                               examples_c_ratio: float,
                               examples_f_ratio: float,
                               examples_d_ratio: float,
                               dataset_name: str) -> tuple[List[Dict[str, Any]], Counter]:
    """
    Balance and merge examples by output pattern using dynamic ratios.
    
    Args:
        examples_main: List of normal command examples
        examples_clarification: List of clarification examples
        examples_failure: List of failure examples
        max_examples_per_pattern: Maximum examples allowed per pattern
        examples_ratio: Target ratio for main examples (0.0-1.0)
        examples_c_ratio: Target ratio for clarification examples (0.0-1.0)
        examples_f_ratio: Target ratio for failure examples (0.0-1.0)
        examples_d_ratio: Target ratio for duplication examples (0.0-1.0)
        dataset_name: Name for logging ("train" or "test")
    
    Returns:
        Tuple of (balanced_examples_list, pattern_counts_dict)
    """
    # Group examples by output pattern
    pattern_groups = defaultdict(lambda: {
        "main": [],
        "clarification": [],
        "failure": [],
        "duplication": []
    })
    
    # Populate pattern groups
    for ex in examples_main:
        pattern_groups[ex["output"]]["main"].append(ex)
    
    for ex in examples_clarification:
        pattern_groups[ex["output"]]["clarification"].append(ex)
    
    for ex in examples_failure:
        pattern_groups[ex["output"]]["failure"].append(ex)

    for ex in examples_duplication:
        pattern_groups[ex["output"]]["duplication"].append(ex)
    
    balanced_examples = []
    pattern_counts = Counter()
    
    # Process each pattern
    for pattern, group_dict in pattern_groups.items():
        main_group = group_dict["main"]
        clarification_group = group_dict["clarification"]
        failure_group = group_dict["failure"]
        duplication_group = group_dict["duplication"]
        
        total_available = len(main_group) + len(clarification_group) + len(failure_group) + len(duplication_group)
        
        if total_available == 0:
            continue
        
        # Determine which types are available
        has_main = len(main_group) > 0
        has_clarification = len(clarification_group) > 0
        has_failure = len(failure_group) > 0
        has_duplication = len(duplication_group) > 0
        
        # Initialize all targets to 0
        main_target = 0
        clarification_target = 0
        failure_target = 0
        duplication_target = 0
        
        # Count how many types are available
        available_types = sum([has_main, has_clarification, has_failure, has_duplication])
        
        # Apply specific rules based on available combinations
        if available_types == 1:
            # Only one type - give it 100%
            if has_main:
                main_target = total_available
            elif has_clarification:
                clarification_target = total_available
            elif has_failure:
                failure_target = total_available
            elif has_duplication:
                duplication_target = total_available
        
        elif available_types == 2:
            # Two types - check specific combinations
            if has_main and has_clarification:
                # main-clarification: 60/40
                main_target = int(total_available * 0.6)
                clarification_target = total_available - main_target
            elif has_main and has_failure:
                # main-failure: 70/30
                main_target = int(total_available * 0.7)
                failure_target = total_available - main_target
            elif has_main and has_duplication:
                # main-duplication: 70/30
                main_target = int(total_available * 0.7)
                duplication_target = total_available - main_target
            elif has_failure and has_clarification:
                # failure-clarification: 70/30
                failure_target = int(total_available * 0.7)
                clarification_target = total_available - failure_target
            elif has_failure and has_duplication:
                # failure-duplication: 80/20
                failure_target = int(total_available * 0.8)
                duplication_target = total_available - failure_target
            else:
                # clarification-duplication: 60/40 (default equal split)
                clarification_target = int(total_available * 0.6)
                duplication_target = total_available - clarification_target
        
        elif available_types == 3:
            # Three types - check specific combinations
            if has_main and has_clarification and has_failure:
                # main-clarification-failure: 60/30/10
                main_target = int(total_available * 0.6)
                clarification_target = int(total_available * 0.3)
                failure_target = total_available - main_target - clarification_target
            elif has_main and has_clarification and has_duplication:
                # main-clarification-duplication: 60/30/10
                main_target = int(total_available * 0.6)
                clarification_target = int(total_available * 0.3)
                duplication_target = total_available - main_target - clarification_target
            else:
                # All other 3-type combinations: distribute equally (33/33/33)
                non_empty_types = []
                if has_main:
                    non_empty_types.append("main")
                if has_clarification:
                    non_empty_types.append("clarification")
                if has_failure:
                    non_empty_types.append("failure")
                if has_duplication:
                    non_empty_types.append("duplication")
                
                ratio_per_type = 1.0 / 3.0
                first_target = int(total_available * ratio_per_type)
                second_target = int(total_available * ratio_per_type)
                third_target = total_available - first_target - second_target
                
                targets = [first_target, second_target, third_target]
                for i, type_name in enumerate(non_empty_types):
                    if type_name == "main":
                        main_target = targets[i]
                    elif type_name == "clarification":
                        clarification_target = targets[i]
                    elif type_name == "failure":
                        failure_target = targets[i]
                    elif type_name == "duplication":
                        duplication_target = targets[i]
        
        else:
            # All four types available - use original ratios
            failure_target = int(total_available * examples_f_ratio)
            clarification_target = int(total_available * examples_c_ratio)
            duplication_target = int(total_available * examples_d_ratio)
            main_target = total_available - failure_target - clarification_target - duplication_target
        
        # Select examples randomly respecting targets
        selected_main = random.sample(
            main_group,
            min(len(main_group), main_target)
        )
        selected_clarification = random.sample(
            clarification_group,
            min(len(clarification_group), clarification_target)
        )
        selected_failure = random.sample(
            failure_group,
            min(len(failure_group), failure_target)
        )
        selected_duplication = random.sample(
            duplication_group,
            min(len(duplication_group), duplication_target)
        )
        
        # Build merged examples with prompts
        merged_group = []


        
        # Add main examples
        for ex in selected_main:
            prompt = (
                f"{MISSIONS_LINE}\n"
                # f"Memory: {ex['memory']}\n"
                f"Command: {ex['command']}\n"
                f"{FINAL_LINE}"
            )
            merged_group.append({
                "input": prompt,
                "output": ex["output"]
            })
        
        # Add clarification examples
        for ex in selected_clarification:
            prompt = (
                f"{MISSIONS_LINE}\n"
                # f"Memory: {ex['memory']}\n"
                f"Command: {ex['command']}\n"
                f"Previous response: {ex['previous_response']}\n"
            )
            # Include failure context if present
            if "failure_context" in ex:
                prompt += f"Failure context: {ex['failure_context']}\n"
            
            prompt += (
                f"Clarification: {ex['clarification']}\n"
                f"{FINAL_LINE_C}"
            )
            merged_group.append({
                "input": prompt,
                "output": ex["output"]
            })

        
        # Add failure examples
        for ex in selected_failure:
            prompt = (
                f"{MISSIONS_LINE}\n"
                # f"Memory: {ex['memory']}\n"
                f"Command: {ex['command']}\n"
                f"Failure context: {ex['failure_context']}\n"
                f"{FINAL_LINE_F}"
            )
            merged_group.append({
                "input": prompt,
                "output": ex["output"]
            })

        # Add duplication examples
        for ex in selected_duplication:
            prompt = f"{MISSIONS_LINE}\n"
            # prompt += f"Memory: {ex['memory']}\n"
            prompt += f"Command: {ex['command']}\n"
            prompt += f"Previous response: {ex['previous_response']}\n"
            if "failure_context" in ex:
                prompt += f"Failure context: {ex['failure_context']}\n"
            prompt += f"Repeat mission: {ex['duplication']}\n"
            prompt += f"{FINAL_LINE_D}"
            merged_group.append({"input": prompt, "output": ex["output"]})

        
        # Apply cap to merged group
        if len(merged_group) > max_examples_per_pattern:
            merged_group = random.sample(
                merged_group,
                max_examples_per_pattern
            )
        
        balanced_examples.extend(merged_group)
        pattern_counts[pattern] = len(merged_group)
    
    # Shuffle before returning
    random.shuffle(balanced_examples)
    return balanced_examples, pattern_counts


# ========================================
# MAIN FUNCTION - COMPLETE REWRITE
# ========================================

def main():
    """Generate separate training and test datasets with proper balancing"""
    
    # ===================================
    # STEP 1: GENERATE ALL EXAMPLES
    # ===================================
    
    # Initialize lists for training examples
    train_examples_main = []
    train_examples_clarification = []
    train_examples_failure = []
    train_examples_duplication = []
    
    # Initialize lists for test examples
    test_examples_main = []
    test_examples_clarification = []
    test_examples_failure = []
    test_examples_duplication = []
    
    # Generate regular command variations (train + test)
    for variation_set in command_variations_data:
        if "test_variations" not in variation_set:
            variation_set["test_variations"] = variation_set["variations"]
        
        train_ex, test_ex = generate_command_variations(
            variation_set["base"],
            variation_set["variations"],
            variation_set["test_variations"]
        )
        train_examples_main.extend(train_ex)
        test_examples_main.extend(test_ex)
    
    # Generate clarification variations (train + test)
    for cmdcla_set in cmdcla_variations_data:
        if "test_command_variations" not in cmdcla_set:
            cmdcla_set["test_command_variations"] = cmdcla_set["command_variations"]
        
        # Get failure_context if present, else None
        failure_ctx = cmdcla_set.get("failure_context", None)
        
        train_ex, test_ex = generate_cmd_cla_variations(
            cmdcla_set["base"],
            cmdcla_set["command_variations"],
            cmdcla_set["test_command_variations"],
            cmdcla_set["clarification_variations"],
            failure_context=failure_ctx  # Pass optional failure context
        )
        train_examples_clarification.extend(train_ex)
        test_examples_clarification.extend(test_ex)

    
    # Generate failure variations (train + test)
    for failure_set in failure_variations_data:
        if "test_command_variations" not in failure_set:
            failure_set["test_command_variations"] = failure_set["command_variations"]
        
        train_ex, test_ex = generate_failure_variations(
            failure_set["base"],
            failure_set["command_variations"],
            failure_set["test_command_variations"]
        )
        train_examples_failure.extend(train_ex)
        test_examples_failure.extend(test_ex)

    # Generate duplication examples
    for dup_set in duplication_variations_data:
        train_ex, test_ex = generate_cmd_dup_variations(
            dup_set["base"],
            dup_set["command_variations"],
            dup_set["test_command_variations"],
            dup_set["duplication_variations"]
        )
        train_examples_duplication.extend(train_ex)
        test_examples_duplication.extend(test_ex)
    
    # ===================================
    # STEP 2: BALANCING PARAMETERS
    # ===================================
    
    max_examples_per_pattern_train = 100
    max_examples_per_pattern_test = 10
    examples_ratio = 0.6       
    examples_c_ratio = 0.2      
    examples_f_ratio = 0.1      
    examples_d_ratio = 0.1  

    # ===================================
    # STEP 3: BALANCE TRAINING DATASET
    # ===================================
    
    balanced_train_examples, pattern_counts_train = balance_dataset_by_pattern(
        examples_main=train_examples_main,
        examples_clarification=train_examples_clarification,
        examples_failure=train_examples_failure,
        examples_duplication=train_examples_duplication,
        max_examples_per_pattern=max_examples_per_pattern_train,
        examples_ratio=examples_ratio,
        examples_c_ratio=examples_c_ratio,
        examples_f_ratio=examples_f_ratio,
        examples_d_ratio=examples_d_ratio,
        dataset_name="train"
    )
    
    # ===================================
    # STEP 4: BALANCE TEST DATASET
    # ===================================
    
    balanced_test_examples, pattern_counts_test = balance_dataset_by_pattern(
        examples_main=test_examples_main,
        examples_clarification=test_examples_clarification,
        examples_failure=test_examples_failure,
        examples_duplication=test_examples_duplication,
        max_examples_per_pattern=max_examples_per_pattern_test,
        examples_ratio=examples_ratio,
        examples_c_ratio=examples_c_ratio,
        examples_f_ratio=examples_f_ratio,
        examples_d_ratio=examples_d_ratio,
        dataset_name="test"
    )
    
    # ===================================
    # STEP 5: WRITE DATASETS TO FILES
    # ===================================
    
    train_output_file = "tlm2ros_train_dataset_small.jsonl"
    with open(train_output_file, "w", encoding="utf-8") as f:
        for ex in balanced_train_examples:
            f.write(json.dumps(ex, ensure_ascii=False) + "\n")
    
    test_output_file = "tlm2ros_test_dataset_small.jsonl"
    with open(test_output_file, "w", encoding="utf-8") as f:
        for ex in balanced_test_examples:
            f.write(json.dumps(ex, ensure_ascii=False) + "\n")
    
    # ===================================
    # STEP 6: PRINT STATISTICS
    # ===================================
    
    print("=" * 90)
    print("✅ DATASETS GENERATED SUCCESSFULLY")
    print("=" * 90)
    
    print(f"\n📁 TRAINING DATASET: '{train_output_file}'")
    print(f"   - Total examples: {len(balanced_train_examples)}")
    print(f"   - Unique output patterns: {len(pattern_counts_train)}")
    if len(pattern_counts_train) > 0:
        avg_per_pattern = len(balanced_train_examples) / len(pattern_counts_train)
        print(f"   - Average examples per pattern: {avg_per_pattern:.1f}")
    print(f"   - Max examples per pattern: {max_examples_per_pattern_train}")
    
    print(f"\n📁 TEST DATASET: '{test_output_file}'")
    print(f"   - Total examples: {len(balanced_test_examples)}")
    print(f"   - Unique output patterns: {len(pattern_counts_test)}")
    if len(pattern_counts_test) > 0:
        avg_per_pattern = len(balanced_test_examples) / len(pattern_counts_test)
        print(f"   - Average examples per pattern: {avg_per_pattern:.1f}")
    print(f"   - Max examples per pattern: {max_examples_per_pattern_test}")
    
    print(f"\n📊 COMPOSITION RATIOS (when all types available):")
    print(f"   - Main examples: {examples_ratio * 100:.0f}%")
    print(f"   - Clarification examples: {examples_c_ratio * 100:.0f}%")
    print(f"   - Failure examples: {examples_f_ratio * 100:.0f}%")
    print(f"   - Duplication examples: {examples_d_ratio * 100:.0f}%")
    print(f"   - Note: Ratios adjust dynamically when types are missing for a pattern")
    
    print(f"\n🔎 TRAINING: Examples per output pattern:")
    for output, count in sorted(pattern_counts_train.items()):
        print(f"   - {output}: {count}")
    
    print(f"\n🔎 TEST: Examples per output pattern:")
    for output, count in sorted(pattern_counts_test.items()):
        print(f"   - {output}: {count}")
    
    print(f"\n🎯 Available missions ({len(AVAILABLE_MISSIONS)}): {', '.join(AVAILABLE_MISSIONS)}")
    print(f"🟡 Available buoys ({MAX_UNIQUE_BUOYS} unique): {', '.join(UNIQUE_BUOY_COLORS)}")
    print(f"📈 Max buoys in memory: {MAX_UNIQUE_BUOYS}")
    print(f"📈 Max missions in history: {MAX_MISSIONS_IN_HISTORY}")
    print("=" * 90)


if __name__ == "__main__":
    main()
