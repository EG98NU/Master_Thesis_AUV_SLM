import json
import random
from typing import List, Dict, Any
from collections import defaultdict, Counter
from operations import OPERATIONS

#=================================================
version = 1
train_output_file = f'X PoC_train_dataset_v{version}.jsonl'
test_output_file = f'X PoC_test_dataset_v{version}.jsonl'
#=================================================

def aw0():
    return "Target not acquired"

def aw1():
    return "Target acquired"

def by0():
    return "0 of 2 yellow buoys: gate not found"

def by1():
    return "1 yellow buoy found, 1 missing: gate partial"


def by2():
    return "2 yellow buoys: gate found"

def nob():
    return "Other buoys not found"

def ob():
    return "Other buoys found"

def pl0():
    return "Pipeline not found"

def pl1():
    return "Pipeline found | Damaged pipe not found"

def plmf():
    return "Pipeline found | Damaged pipe found"

def vnc():
    return "Valve not closed"

def vc():
    return "Valve closed"

def rg():
    return "Ring grabbed"

def ru():
    return "Ring untouched"


def mp():
    return "Main pipe found"

def nmp():
    return "Main pipe not found"

def mpm():
    return "Main pipe markers found"

def nmpm():
    return "No main pipe markers found"

def mp_nm():
    return f"{mp()} | {nmpm()}"

def mp_m():
    return f"{mp()} | {mpm()}"

def nip():
    return "No potential buoys detected in the buoy area"

def ip():
    return "Potential buoys detected in the buoy area"

def ctx_(arg="default"):
    """Generate context based on argument - fixed to avoid eval()"""
    context_map = {
        'wp': aw1(),
        'nwp': aw0(),

        'y0': by0(),
        'y1': by1(),
        'y2': by2(),

        'nip' : nip(),
        'ip': ip(),

        'y0_nip' : f"{by0} | {nip()}",
        'y1_nip' : f"{by1} | {nip()}",
        'y2_nip' : f"{by2} | {nip()}",

        'y0_ip' : f"{by0} | {ip()}",
        'y1_ip' : f"{by1} | {ip()}",
        'y2_ip' : f"{by2} | {ip()}",

        'pl0' : pl0(),
        "npl" : pl0(),
        'pl1': pl1(),
        'plmf': plmf(),

        'nmp': nmp(),
        'mp_nm': mp_nm(),
        'mp_m': mp_m(),
        'npl_nmp': f"{pl0()} | {nmp()}",
        'npl_mp_nm': f"{pl0()} | {mp_nm()}",
        'npl_mp_m': f"{pl0()} | {mp_m()}",
        'pl1_nmp': f"{pl1()} | {nmp()}",
        'pl1_mp_nm': f"{pl1()} | {mp_nm()}",
        'pl1_mp_m': f"{pl1()} | {mp_m()}",
        'plmf_nmp': f"{plmf()} | {nmp()}",
        'plmf_mp_nm': f"{plmf()} | {mp_nm()}",
        'plmf_mp_m': f"{plmf()} | {mp_m()}",
        'vc': f"{plmf()} | {vc()}",
        'vnc': f"{plmf()} | {vnc()}",
        'rg': f"{plmf()} | {rg()}",
        'ru': f"{plmf()} | {ru()}",


        'nwp_y0' : f"{aw0()} | {by0()}",
        'y2_pl0' : f"{by2()} | {pl0()}",
        'y2_pl0_nip': f"{by2()} | {pl0()} | {nip()}",
        'pl0_nip' : f"{pl0()} | {nip()}",
        'pl1_nip' : f"{pl1()} | {nip()}",
        'y0_nip' : f"{by0()} | {nip()}",
        'y0_ip' : f"{by0()} | {ip()}",
        'y1_nip' : f"{by1()} | {nip()}",
        'y1_ip' : f"{by1()} | {ip()}",
        'y2_nip' : f"{by2()} | {nip()}",
        'y2_ip' : f"{by2()} | {ip()}",
        'default': ""
    }
    return context_map.get(arg)

#===============================================================================================================================

def generate_command_variations(base_example: Dict[str, Any], command_variations: List[str], test_variations: List[str]) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Generate multiple examples with different commands and contexts"""
    train_examples = []
    test_examples = []
    
    for variant in command_variations:
        new_example = {
            "context": base_example["context"],
            "command": variant,
            "output": base_example["output"]
        }
        train_examples.append(new_example)
    
    for variant in test_variations:
        new_example = {
            "context": base_example["context"],
            "command": variant,
            "output": base_example["output"]
        }
        test_examples.append(new_example)
    
    return train_examples, test_examples



def generate_cmdcla_variations(base_example: Dict[str, Any], command_variations: List[str], command_test_variations: List[str], clarification_variations: List[str]) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Generate examples with variations for both commands and clarifications"""
    train_examples = []
    test_examples = []
    
    for cmd in command_variations:
        for clar in clarification_variations:
            new_example = {
                "context": base_example["context"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "clarification": clar,
                "output": base_example["output"]
            }
            train_examples.append(new_example)
    
    for cmd in command_test_variations:
        for clar in clarification_variations:
            new_example = {
                "context": base_example["context"],
                "command": cmd,
                "previous_response": base_example["previous_response"],
                "clarification": clar,
                "output": base_example["output"]
            }
            test_examples.append(new_example)
    
    return train_examples, test_examples

#===============================================================================================================================

SEQ_LINE = f"Available operations: {', '.join(OPERATIONS.keys())}"
FINAL_LINE = "Instruction: Generate a numbered list of operations to accomplish this command, pay attention to the current knowledge of the environment:"
FINAL_LINE_C = "Instruction: Generate a numbered list of operations to accomplish this command, pay attention to the current knowledge of the environment, your previous response and the clarification for more info"

# EXAMPLES
examples = []
examples_c = []

# Add this code after the examples list definition and before the file writing section

# Define all base examples and their variations in a structured way
command_variations_data = [
    {
        "base": {
            "context": f"{ctx_('wp')}",
            "command": "move to the waypoint",
            "output": "Plan: 1. move to waypoint"
        },
        "variations": [
                    "move to the waypoint", #1
                    "go to the waypoint", #2
                    "reach the waypoint", #3
                    "move to point", #4
                    "go to wp", #5
                    "reach the target", #6
                    "move toward the target waypoint", #7
                    "go toward the target waypoint", #8
                    "move forward to the goal", #9
                    "go forward to the target", #10
                    "reach destination", #11
                    "go to destination", #12
                    "move to destination", #13
                    "head to the waypoint", #14
                    "head to the target waypoint", #15
                    "head to the destination", #16
                    "proceed to the waypoint", #17
                    "proceed to the target waypoint", #18
                    "proceed to the destination", #19
                    "navigate to the waypoint", #20
                    "navigate to the target point", #21
                    "advance to the waypoint", #22
                    "advance to the target", #23
                    "advance toward the waypoint", #24
                    "step forward to the waypoint", #25
                    "march to the waypoint", #26
                    "march toward the target", #27
                    "approach the waypoint", #28
                    "approach the destination", #29
                    "move closer to the waypoint", #30
                    "go in the direction of the waypoint", #31
                    "head forward to the waypoint", #32
                    "travel to the waypoint", #33
                    "travel toward the destination", #34
                    "make your way to the waypoint", #35
                    "make your way to the destination", #36
                    "continue to the waypoint", #37
                    "continue toward the target", #38
                    "continue forward to the goal", #39
                    "relocate to the waypoint", #40
                    "shift position to the waypoint", #41
                    "change position toward the waypoint", #42
                    "move along to the waypoint", #43
                    "progress to the waypoint", #44
                    "progress toward the target waypoint", #45
                    "set course to the waypoint", #46
                    "set course for the destination", #47
                    "set heading to the waypoint", #48
                    "set heading toward the target", #49
                    "direct movement to the waypoint", #50
                    "direct movement toward the goal", #51
                    "navigate forward to the waypoint", #52
                    "advance forward to the destination", #53
                    "go directly to the waypoint", #54
                    "move directly toward the destination", #55
                    "transition to the waypoint", #56
                    "transition toward the target", #57
                    "shift toward the waypoint", #58
                    "adjust course to the destination", #59
                    "navigate in the direction of the waypoint", #60
                    "head straight to the destination", #61
                    "move swiftly to the waypoint", #62
                    "advance quickly to the target", #63
                    "travel quickly to the waypoint", #64
                    "progress steadily to the destination", #65
                    "make progress toward the waypoint", #66
                    "keep moving to the waypoint", #67
                    "stay on course to the destination", #68
                    "maintain heading toward the waypoint", #69
                    "move at pace to the waypoint", #70
                    "push forward to the target", #71
                    "proceed directly to the destination", #72
                    "navigate smoothly to the waypoint", #73
                    "go straight to the target waypoint", #74
                    "move without delay to the waypoint", #75
                    "advance without hesitation toward the destination", #76
                    "relocate directly to the destination", #77
                    "transition smoothly to the waypoint", #78
                    "shift trajectory toward the target", #79
                    "change course to the waypoint", #80
                    "alter direction to the destination", #81
                    "go to the designated waypoint", #82
                    "move to the marked waypoint", #83
                    "reach the specified target", #84
                    "advance to the chosen waypoint", #85
                    "navigate to the assigned destination", #86
                    "proceed to the given waypoint", #87
                    "travel to the identified target", #88
                    "head to the designated location", #89
                    "move forward along the path to the waypoint", #90
                    "progress along the route to the destination", #91
                    "continue along the trajectory to the target", #92
                    "move steadily to the waypoint", #93
                    "advance methodically to the target", #94
                    "travel consistently to the destination", #95
                    "progress deliberately toward the waypoint", #96
                    "navigate purposefully to the target", #97
                    "execute movement to the waypoint", #98
                    "initiate movement toward the destination", #99
                    "begin transit to the waypoint" #100
        ],
        "test_variations": [
                    "your next destination is the waypoint, proceed there", #1
                    "ensure you reach the target position designated as waypoint", #2
                    "traverse the area in the direction of the destination waypoint", #3
                    "the waypoint is your objective, move accordingly", #4
                    "achieve positioning at the waypoint destination", #5
                    "direct your movement toward waypoint positioning", #6
                    "waypointbound trajectory is required", #7
                    "execute waypoint transit procedures", #8
                    "positioning at waypoint coordinates is the current objective", #9
                    "waypoint approach and arrival confirmation needed" #10
        ],


    },
    {
        "base": {
            "context": f"{ctx_('nwp')}",
            "command": "move to the waypoint",
            "output": "Plan: 1. acquire target 2. move to waypoint"
        },
        "variations": [
                    "move to the waypoint", #1
                    "go to the waypoint", #2
                    "reach the waypoint", #3
                    "move to point", #4
                    "go to wp", #5
                    "reach the target", #6
                    "move toward the target waypoint", #7
                    "go toward the target waypoint", #8
                    "move forward to the goal", #9
                    "go forward to the target", #10
                    "reach destination", #11
                    "go to destination", #12
                    "move to destination", #13
                    "head to the waypoint", #14
                    "head to the target waypoint", #15
                    "head to the destination", #16
                    "proceed to the waypoint", #17
                    "proceed to the target waypoint", #18
                    "proceed to the destination", #19
                    "navigate to the waypoint", #20
                    "navigate to the target point", #21
                    "advance to the waypoint", #22
                    "advance to the target", #23
                    "advance toward the waypoint", #24
                    "step forward to the waypoint", #25
                    "march to the waypoint", #26
                    "march toward the target", #27
                    "approach the waypoint", #28
                    "approach the destination", #29
                    "move closer to the waypoint", #30
                    "go in the direction of the waypoint", #31
                    "head forward to the waypoint", #32
                    "travel to the waypoint", #33
                    "travel toward the destination", #34
                    "make your way to the waypoint", #35
                    "make your way to the destination", #36
                    "continue to the waypoint", #37
                    "continue toward the target", #38
                    "continue forward to the goal", #39
                    "relocate to the waypoint", #40
                    "shift position to the waypoint", #41
                    "change position toward the waypoint", #42
                    "move along to the waypoint", #43
                    "progress to the waypoint", #44
                    "progress toward the target waypoint", #45
                    "set course to the waypoint", #46
                    "set course for the destination", #47
                    "set heading to the waypoint", #48
                    "set heading toward the target", #49
                    "direct movement to the waypoint", #50
                    "direct movement toward the goal", #51
                    "navigate forward to the waypoint", #52
                    "advance forward to the destination", #53
                    "go directly to the waypoint", #54
                    "move directly toward the destination", #55
                    "transition to the waypoint", #56
                    "transition toward the target", #57
                    "shift toward the waypoint", #58
                    "adjust course to the destination", #59
                    "navigate in the direction of the waypoint", #60
                    "head straight to the destination", #61
                    "move swiftly to the waypoint", #62
                    "advance quickly to the target", #63
                    "travel quickly to the waypoint", #64
                    "progress steadily to the destination", #65
                    "make progress toward the waypoint", #66
                    "keep moving to the waypoint", #67
                    "stay on course to the destination", #68
                    "maintain heading toward the waypoint", #69
                    "move at pace to the waypoint", #70
                    "push forward to the target", #71
                    "proceed directly to the destination", #72
                    "navigate smoothly to the waypoint", #73
                    "go straight to the target waypoint", #74
                    "move without delay to the waypoint", #75
                    "advance without hesitation toward the destination", #76
                    "relocate directly to the destination", #77
                    "transition smoothly to the waypoint", #78
                    "shift trajectory toward the target", #79
                    "change course to the waypoint", #80
                    "alter direction to the destination", #81
                    "go to the designated waypoint", #82
                    "move to the marked waypoint", #83
                    "reach the specified target", #84
                    "advance to the chosen waypoint", #85
                    "navigate to the assigned destination", #86
                    "proceed to the given waypoint", #87
                    "travel to the identified target", #88
                    "head to the designated location", #89
                    "move forward along the path to the waypoint", #90
                    "progress along the route to the destination", #91
                    "continue along the trajectory to the target", #92
                    "move steadily to the waypoint", #93
                    "advance methodically to the target", #94
                    "travel consistently to the destination", #95
                    "progress deliberately toward the waypoint", #96
                    "navigate purposefully to the target", #97
                    "execute movement to the waypoint", #98
                    "initiate movement toward the destination", #99
                    "begin transit to the waypoint" #100
        ],
        "test_variations": [
                    "your next destination is the waypoint, proceed there", #1
                    "ensure you reach the target position designated as waypoint", #2
                    "traverse the area in the direction of the destination waypoint", #3
                    "the waypoint is your objective, move accordingly", #4
                    "achieve positioning at the waypoint destination", #5
                    "direct your movement toward waypoint positioning", #6
                    "waypointbound trajectory is required", #7
                    "execute waypoint transit procedures", #8
                    "positioning at waypoint coordinates is the current objective", #9
                    "waypoint approach and arrival confirmation needed" #10
        ],

    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "find a yellow buoy",
            "output": "Plan: 1. find yellow buoy"
        },
        "variations": [
                    "find a yellow buoy", #1
                    "find a gate buoy", #2
                    "look for one of the yellow buoys", #3
                    "spot a yellow buoy", #4
                    "spot a gate buoy", #5
                    "spot one of the yellow buoys", #6
                    "look for a yellow buoy", #7
                    "spot a gate buoy", #8
                    "find one yellow buoy", #9
                    "look for a buoy of the gate", #10
                    "search for a yellow buoy", #11
                    "search for a gate buoy", #12
                    "search for one of the yellow buoys", #13
                    "locate a yellow buoy", #14
                    "locate a gate buoy", #15
                    "locate one of the yellow buoys", #16
                    "identify a yellow buoy", #17
                    "identify a gate buoy", #18
                    "identify one of the yellow buoys", #19
                    "detect a yellow buoy", #20
                    "detect a gate buoy", #21
                    "detect one of the yellow buoys", #22
                    "find and recognize a yellow buoy", #23
                    "find and recognize a gate buoy", #24
                    "look around for a yellow buoy", #25
                    "look around for a gate buoy", #26
                    "try to find a yellow buoy", #27
                    "try to find a gate buoy", #28
                    "try to spot a yellow buoy", #29
                    "try to spot a gate buoy", #30
                    "try to locate a yellow buoy", #31
                    "try to locate a gate buoy", #32
                    "scan for a yellow buoy", #33
                    "scan for a gate buoy", #34
                    "scan the area for a yellow buoy", #35
                    "scan the area for a gate buoy", #36
                    "look out for a yellow buoy", #37
                    "look out for a gate buoy", #38
                    "seek a yellow buoy", #39
                    "seek a gate buoy", #40
                    "seek one of the yellow buoys", #41
                    "attempt to detect a yellow buoy", #42
                    "attempt to detect a gate buoy", #43
                    "attempt to identify a yellow buoy", #44
                    "attempt to identify a gate buoy", #45
                    "check for a yellow buoy", #46
                    "check for a gate buoy", #47
                    "check the area for a yellow buoy", #48
                    "check the area for a gate buoy", #49
                    "search the area for a yellow buoy", #50
                    "search the area for a gate buoy", #51
                    "find the yellow buoy", #52
                    "find the gate buoy", #53
                    "locate the yellow buoy", #54
                    "locate the gate buoy", #55
                    "spot the yellow buoy", #56
                    "spot the gate buoy", #57
                    "identify the yellow buoy", #58
                    "identify the gate buoy", #59
                    "detect the yellow buoy", #60
                    "detect the gate buoy", #61
                    "look for yellow buoys", #62
                    "look for gate buoys", #63
                    "search for yellow buoys", #64
                    "search for gate buoys", #65
                    "locate yellow buoys", #66
                    "locate gate buoys", #67
                    "spot yellow buoys", #68
                    "spot gate buoys", #69
                    "find yellow buoys", #70
                    "find gate buoys", #71
                    "scan for yellow buoys", #72
                    "scan for gate buoys", #73
                    "identify yellow buoys", #74
                    "identify gate buoys", #75
                    "detect yellow buoys", #76
                    "detect gate buoys", #77
                    "observe a yellow buoy", #78
                    "observe a gate buoy", #79
                    "observe yellow buoys", #80
                    "watch for a yellow buoy", #81
                    "watch for a gate buoy", #82
                    "watch for yellow buoys", #83
                    "watch for gate buoys", #84
                    "search out a yellow buoy", #85
                    "search out a gate buoy", #86
                    "locate and identify a yellow buoy", #87
                    "locate and identify a gate buoy", #88
                    "find and locate a yellow buoy", #89
                    "find and locate a gate buoy", #90
                    "try to identify a yellow buoy", #91
                    "try to identify a gate buoy", #92
                    "attempt to locate a yellow buoy", #93
                    "attempt to locate a gate buoy", #94
                    "attempt to spot a yellow buoy", #95
                    "attempt to spot a gate buoy", #96
                    "check out a yellow buoy", #97
                    "check out a gate buoy", #98
                    "verify the yellow buoy", #99
                    "verify the gate buoy" #100
        ],
        "test_variations": [
                    "acquire visual confirmation of a yellow gate buoy", #1
                    "yellow buoy identification is required, prioritize detection", #2
                    "locate and confirm the presence of gate-marked yellow buoys", #3
                    "visual acquisition of buoys marked in yellow is the objective", #4
                    "gate buoys must be identified and reported", #5
                    "search operations should prioritize yellow marker identification", #6
                    "buoy markers that are gate-type require detection", #7
                    "yellow coloration indicates target buoy priority", #8
                    "maintain surveillance for buoy markers of gate designation", #9
                    "detection of colored buoys in the gate system is required" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('y0_ip')}",
            "command": "find a yellow buoy",
            "output": "Plan: 1. find yellow buoy"
        },
        "variations": [
                    "find a yellow buoy", #1
                    "find a gate buoy", #2
                    "look for one of the yellow buoys", #3
                    "spot a yellow buoy", #4
                    "spot a gate buoy", #5
                    "spot one of the yellow buoys", #6
                    "look for a yellow buoy", #7
                    "spot a gate buoy", #8
                    "find one yellow buoy", #9
                    "look for a buoy of the gate", #10
                    "search for a yellow buoy", #11
                    "search for a gate buoy", #12
                    "search for one of the yellow buoys", #13
                    "locate a yellow buoy", #14
                    "locate a gate buoy", #15
                    "locate one of the yellow buoys", #16
                    "identify a yellow buoy", #17
                    "identify a gate buoy", #18
                    "identify one of the yellow buoys", #19
                    "detect a yellow buoy", #20
                    "detect a gate buoy", #21
                    "detect one of the yellow buoys", #22
                    "find and recognize a yellow buoy", #23
                    "find and recognize a gate buoy", #24
                    "look around for a yellow buoy", #25
                    "look around for a gate buoy", #26
                    "try to find a yellow buoy", #27
                    "try to find a gate buoy", #28
                    "try to spot a yellow buoy", #29
                    "try to spot a gate buoy", #30
                    "try to locate a yellow buoy", #31
                    "try to locate a gate buoy", #32
                    "scan for a yellow buoy", #33
                    "scan for a gate buoy", #34
                    "scan the area for a yellow buoy", #35
                    "scan the area for a gate buoy", #36
                    "look out for a yellow buoy", #37
                    "look out for a gate buoy", #38
                    "seek a yellow buoy", #39
                    "seek a gate buoy", #40
                    "seek one of the yellow buoys", #41
                    "attempt to detect a yellow buoy", #42
                    "attempt to detect a gate buoy", #43
                    "attempt to identify a yellow buoy", #44
                    "attempt to identify a gate buoy", #45
                    "check for a yellow buoy", #46
                    "check for a gate buoy", #47
                    "check the area for a yellow buoy", #48
                    "check the area for a gate buoy", #49
                    "search the area for a yellow buoy", #50
                    "search the area for a gate buoy", #51
                    "find the yellow buoy", #52
                    "find the gate buoy", #53
                    "locate the yellow buoy", #54
                    "locate the gate buoy", #55
                    "spot the yellow buoy", #56
                    "spot the gate buoy", #57
                    "identify the yellow buoy", #58
                    "identify the gate buoy", #59
                    "detect the yellow buoy", #60
                    "detect the gate buoy", #61
                    "look for yellow buoys", #62
                    "look for gate buoys", #63
                    "search for yellow buoys", #64
                    "search for gate buoys", #65
                    "locate yellow buoys", #66
                    "locate gate buoys", #67
                    "spot yellow buoys", #68
                    "spot gate buoys", #69
                    "find yellow buoys", #70
                    "find gate buoys", #71
                    "scan for yellow buoys", #72
                    "scan for gate buoys", #73
                    "identify yellow buoys", #74
                    "identify gate buoys", #75
                    "detect yellow buoys", #76
                    "detect gate buoys", #77
                    "observe a yellow buoy", #78
                    "observe a gate buoy", #79
                    "observe yellow buoys", #80
                    "watch for a yellow buoy", #81
                    "watch for a gate buoy", #82
                    "watch for yellow buoys", #83
                    "watch for gate buoys", #84
                    "search out a yellow buoy", #85
                    "search out a gate buoy", #86
                    "locate and identify a yellow buoy", #87
                    "locate and identify a gate buoy", #88
                    "find and locate a yellow buoy", #89
                    "find and locate a gate buoy", #90
                    "try to identify a yellow buoy", #91
                    "try to identify a gate buoy", #92
                    "attempt to locate a yellow buoy", #93
                    "attempt to locate a gate buoy", #94
                    "attempt to spot a yellow buoy", #95
                    "attempt to spot a gate buoy", #96
                    "check out a yellow buoy", #97
                    "check out a gate buoy", #98
                    "verify the yellow buoy", #99
                    "verify the gate buoy" #100
        ],
        "test_variations": [
                    "acquire visual confirmation of a yellow gate buoy", #1
                    "yellow buoy identification is required, prioritize detection", #2
                    "locate and confirm the presence of gate-marked yellow buoys", #3
                    "visual acquisition of buoys marked in yellow is the objective", #4
                    "gate buoys must be identified and reported", #5
                    "search operations should prioritize yellow marker identification", #6
                    "buoy markers that are gate-type require detection", #7
                    "yellow coloration indicates target buoy priority", #8
                    "maintain surveillance for buoy markers of gate designation", #9
                    "detection of colored buoys in the gate system is required" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y0')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy"
        },
        "variations": [
                    "find the gate", #1
                    "locate the gate", #2
                    "look for the gate", #3
                    "search for the gate", #4
                    "scan for the gate", #5
                    "look around for the gate", #6
                    "try to find the gate", #7
                    "seek the gate", #8
                    "check for the gate", #9
                    "locate and recognize the gate", #10
                    "spot and confirm the gate", #11
                    "look out for the gate", #12
                    "locate the gate position", #13
                    "detect the gate position", #14
                    "approach the gate", #15
                    "recognize the gate formation", #16
                    "find and approach the gate", #17
                    "identify the gate", #18
                    "detect the gate", #19
                    "observe the gate", #20
                    "spot the gate", #21
                    "find the gate location", #22
                    "search the area for the gate", #23
                    "scan the area for the gate", #24
                    "look around the area for the gate", #25
                    "try to locate the gate", #26
                    "attempt to find the gate", #27
                    "attempt to locate the gate", #28
                    "attempt to identify the gate", #29
                    "attempt to detect the gate", #30
                    "seek and locate the gate", #31
                    "seek and identify the gate", #32
                    "seek and find the gate", #33
                    "check and locate the gate", #34
                    "check and identify the gate", #35
                    "check the area for the gate", #36
                    "verify the gate position", #37
                    "verify the gate location", #38
                    "watch for the gate", #39
                    "watch out for the gate", #40
                    "keep looking for the gate", #41
                    "keep searching for the gate", #42
                    "continue looking for the gate", #43
                    "continue searching for the gate", #44
                    "proceed to find the gate", #45
                    "proceed to locate the gate", #46
                    "move toward the gate", #47
                    "move in the direction of the gate", #48
                    "head toward the gate", #49
                    "head in the direction of the gate", #50
                    "navigate toward the gate", #51
                    "navigate in the direction of the gate", #52
                    "travel toward the gate", #53
                    "travel in the direction of the gate", #54
                    "advance toward the gate", #55
                    "advance in the direction of the gate", #56
                    "progress toward the gate", #57
                    "progress in the direction of the gate", #58
                    "move to find the gate", #59
                    "move to locate the gate", #60
                    "head to find the gate", #61
                    "head to locate the gate", #62
                    "navigate to find the gate", #63
                    "navigate to locate the gate", #64
                    "travel to find the gate", #65
                    "travel to locate the gate", #66
                    "advance to find the gate", #67
                    "advance to locate the gate", #68
                    "scan and identify the gate", #69
                    "scan and locate the gate", #70
                    "search and identify the gate", #71
                    "search and locate the gate", #72
                    "look and identify the gate", #73
                    "look and locate the gate", #74
                    "find the gate formation", #75
                    "locate the gate formation", #76
                    "identify the gate formation", #77
                    "detect the gate formation", #78
                    "recognize and locate the gate", #79
                    "recognize and identify the gate", #80
                    "find and identify the gate", #81
                    "find and locate the gate", #82
                    "find and detect the gate", #83
                    "find and observe the gate", #84
                    "locate and detect the gate", #85
                    "locate and observe the gate", #86
                    "spot and identify the gate", #87
                    "spot and locate the gate", #88
                    "observe and identify the gate", #89
                    "observe and locate the gate", #90
                    "search and find the gate", #91
                    "scan and find the gate", #92
                    "look and find the gate", #93
                    "seek and find the gate", #94
                    "check and find the gate", #95
                    "try and find the gate", #96
                    "attempt and locate the gate", #97
                    "verify and locate the gate", #98
                    "confirm the gate location", #99
                    "confirm the gate position" #100
        ],
        "test_variations": [
                    "gate acquisition is the primary objective requiring positive identification", #1
                    "visual confirmation of the gate structure must be established immediately", #2
                    "the gate perimeter should be located and monitored", #3
                    "gate positioning must be determined relative to current location", #4
                    "establish gate contact and report dimensional parameters", #5
                    "gate markers require systematic search and recognition procedures", #6
                    "initiate gate detection protocols and report findings", #7
                    "the target structure designated as gate requires location confirmation", #8
                    "gate-based reference point identification is operationally critical", #9
                    "execute gate reconnaissance and provide positional data" #10
        ]


    }, 
    {
        "base":    {
            "context": f"{ctx_('y0_nip')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy"
        },
        "variations": [
                    "find both the gate buoys", #1
                    "spot two gate buoys", #2
                    "find both buoys of the gate", #3
                    "locate the two yellow buoys", #4
                    "look for the pair of gate buoys", #5
                    "find the yellow buoys forming the gate", #6
                    "search for the yellow buoys forming the gate", #7
                    "locate the gate buoys", #8
                    "identify the gate buoys", #9
                    "detect the gate buoys", #10
                    "spot the two yellow buoys of the gate", #11
                    "find the two yellow buoys of the gate", #12
                    "scan for both gate buoys", #13
                    "look around for the two yellow buoys", #14
                    "try to locate the gate buoys", #15
                    "try to spot both gate buoys", #16
                    "seek the two yellow buoys of the gate", #17
                    "attempt to detect the gate buoys", #18
                    "attempt to identify both gate buoys", #19
                    "check for the two yellow buoys", #20
                    "find and identify the gate buoys", #21
                    "detect and mark the gate buoys", #22
                    "identify and log the two yellow buoys", #23
                    "observe the gate buoys carefully", #24
                    "search around for the pair of gate buoys", #25
                    "scan the gate buoys", #26
                    "track the gate buoys", #27
                    "head towards the gate buoys", #28
                    "confirm the gate buoys", #29
                    "detect both buoys forming the gate", #30
                    "locate both yellow buoys", #31
                    "find the pair of buoys", #32
                    "spot the paired gate buoys", #33
                    "identify both gate markers", #34
                    "search for the two buoys", #35
                    "look for both yellow markers", #36
                    "scan for the gate markers", #37
                    "detect the dual gate buoys", #38
                    "locate the buoy pair", #39
                    "observe both gate buoys", #40
                    "find and confirm both buoys", #41
                    "locate and track the gate buoys", #42
                    "identify and position the two buoys", #43
                    "spot and verify the gate buoys", #44
                    "search and locate both buoys", #45
                    "scan and identify the gate buoys", #46
                    "watch for the paired buoys", #47
                    "look out for both gate markers", #48
                    "check and confirm the gate buoys", #49
                    "verify both yellow buoys", #50
                    "find the two markers of the gate", #51
                    "locate the gate's buoy markers", #52
                    "identify the gate buoy pair", #53
                    "detect the paired yellow buoys", #54
                    "spot the dual yellow markers", #55
                    "search the area for both gate buoys", #56
                    "scan the area for the two yellow buoys", #57
                    "look around the area for both buoys", #58
                    "try to identify both gate buoys", #59
                    "attempt to spot the gate buoy pair", #60
                    "attempt to locate both yellow markers", #61
                    "seek both gate buoys", #62
                    "seek the paired buoy markers", #63
                    "check for both gate markers", #64
                    "verify the gate buoy positions", #65
                    "confirm both buoy locations", #66
                    "observe and identify both buoys", #67
                    "monitor the gate buoys", #68
                    "track both yellow buoys", #69
                    "follow the gate buoys", #70
                    "proceed to find both buoys", #71
                    "proceed to locate the gate buoys", #72
                    "move toward both gate buoys", #73
                    "head toward the gate buoy pair", #74
                    "navigate toward both yellow buoys", #75
                    "travel toward the gate buoys", #76
                    "advance toward both gate markers", #77
                    "progress toward the gate buoys", #78
                    "find and approach both gate buoys", #79
                    "locate and approach the gate buoys", #80
                    "identify and approach both buoys", #81
                    "detect and approach the gate buoys", #82
                    "spot and approach both markers", #83
                    "search and find both gate buoys", #84
                    "scan and find the gate buoys", #85
                    "look and find both yellow buoys", #86
                    "seek and find the paired buoys", #87
                    "check and locate both buoys", #88
                    "try and find the gate buoys", #89
                    "attempt and identify both buoys", #90
                    "verify and locate the gate markers", #91
                    "confirm and track the gate buoys", #92
                    "locate the paired yellow gate buoys", #93
                    "find the dual positioned gate buoys", #94
                    "identify the two markers forming the gate", #95
                    "detect the positioned gate buoy pair", #96
                    "spot the arranged gate buoys", #97
                    "observe the gate buoy alignment", #98
                    "recognize the gate buoy configuration", #99
                    "establish both gate buoy positions" #100
        ],
        "test_variations": [
                    "acquire and verify the exact coordinates of both gate buoys in sequence", #1
                    "the gate structure is defined by two yellow buoys requiring dual confirmation", #2
                    "establish the gate reference frame by identifying and logging both marker positions", #3
                    "both yellow markers must be located and their spatial relationship documented", #4
                    "gate definition requires simultaneous detection of the paired buoy markers", #5
                    "buoy pair designation indicates gate functionality requiring identification of both elements", #6
                    "execute identification protocol for gate buoys and confirm marker alignment parameters", #7
                    "dual yellow buoy detection is mandatory for gate transit authorization", #8
                    "the gateway is formed by two distinct buoy markers requiring positional confirmation", #9
                    "locate and correlate both buoy positions to establish gate reference data" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('y0_ip')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy"
        },
        "variations": [
                    "find both the gate buoys", #1
                    "spot two gate buoys", #2
                    "find both buoys of the gate", #3
                    "locate the two yellow buoys", #4
                    "look for the pair of gate buoys", #5
                    "find the yellow buoys forming the gate", #6
                    "search for the yellow buoys forming the gate", #7
                    "locate the gate buoys", #8
                    "identify the gate buoys", #9
                    "detect the gate buoys", #10
                    "spot the two yellow buoys of the gate", #11
                    "find the two yellow buoys of the gate", #12
                    "scan for both gate buoys", #13
                    "look around for the two yellow buoys", #14
                    "try to locate the gate buoys", #15
                    "try to spot both gate buoys", #16
                    "seek the two yellow buoys of the gate", #17
                    "attempt to detect the gate buoys", #18
                    "attempt to identify both gate buoys", #19
                    "check for the two yellow buoys", #20
                    "find and identify the gate buoys", #21
                    "detect and mark the gate buoys", #22
                    "identify and log the two yellow buoys", #23
                    "observe the gate buoys carefully", #24
                    "search around for the pair of gate buoys", #25
                    "scan the gate buoys", #26
                    "track the gate buoys", #27
                    "head towards the gate buoys", #28
                    "confirm the gate buoys", #29
                    "detect both buoys forming the gate", #30
                    "locate both yellow buoys", #31
                    "find the pair of buoys", #32
                    "spot the paired gate buoys", #33
                    "identify both gate markers", #34
                    "search for the two buoys", #35
                    "look for both yellow markers", #36
                    "scan for the gate markers", #37
                    "detect the dual gate buoys", #38
                    "locate the buoy pair", #39
                    "observe both gate buoys", #40
                    "find and confirm both buoys", #41
                    "locate and track the gate buoys", #42
                    "identify and position the two buoys", #43
                    "spot and verify the gate buoys", #44
                    "search and locate both buoys", #45
                    "scan and identify the gate buoys", #46
                    "watch for the paired buoys", #47
                    "look out for both gate markers", #48
                    "check and confirm the gate buoys", #49
                    "verify both yellow buoys", #50
                    "find the two markers of the gate", #51
                    "locate the gate's buoy markers", #52
                    "identify the gate buoy pair", #53
                    "detect the paired yellow buoys", #54
                    "spot the dual yellow markers", #55
                    "search the area for both gate buoys", #56
                    "scan the area for the two yellow buoys", #57
                    "look around the area for both buoys", #58
                    "try to identify both gate buoys", #59
                    "attempt to spot the gate buoy pair", #60
                    "attempt to locate both yellow markers", #61
                    "seek both gate buoys", #62
                    "seek the paired buoy markers", #63
                    "check for both gate markers", #64
                    "verify the gate buoy positions", #65
                    "confirm both buoy locations", #66
                    "observe and identify both buoys", #67
                    "monitor the gate buoys", #68
                    "track both yellow buoys", #69
                    "follow the gate buoys", #70
                    "proceed to find both buoys", #71
                    "proceed to locate the gate buoys", #72
                    "move toward both gate buoys", #73
                    "head toward the gate buoy pair", #74
                    "navigate toward both yellow buoys", #75
                    "travel toward the gate buoys", #76
                    "advance toward both gate markers", #77
                    "progress toward the gate buoys", #78
                    "find and approach both gate buoys", #79
                    "locate and approach the gate buoys", #80
                    "identify and approach both buoys", #81
                    "detect and approach the gate buoys", #82
                    "spot and approach both markers", #83
                    "search and find both gate buoys", #84
                    "scan and find the gate buoys", #85
                    "look and find both yellow buoys", #86
                    "seek and find the paired buoys", #87
                    "check and locate both buoys", #88
                    "try and find the gate buoys", #89
                    "attempt and identify both buoys", #90
                    "verify and locate the gate markers", #91
                    "confirm and track the gate buoys", #92
                    "locate the paired yellow gate buoys", #93
                    "find the dual positioned gate buoys", #94
                    "identify the two markers forming the gate", #95
                    "detect the positioned gate buoy pair", #96
                    "spot the arranged gate buoys", #97
                    "observe the gate buoy alignment", #98
                    "recognize the gate buoy configuration", #99
                    "establish both gate buoy positions" #100
        ],
        "test_variations": [
                    "acquire and verify the exact coordinates of both gate buoys in sequence", #1
                    "the gate structure is defined by two yellow buoys requiring dual confirmation", #2
                    "establish the gate reference frame by identifying and logging both marker positions", #3
                    "both yellow markers must be located and their spatial relationship documented", #4
                    "gate definition requires simultaneous detection of the paired buoy markers", #5
                    "buoy pair designation indicates gate functionality requiring identification of both elements", #6
                    "execute identification protocol for gate buoys and confirm marker alignment parameters", #7
                    "dual yellow buoy detection is mandatory for gate transit authorization", #8
                    "the gateway is formed by two distinct buoy markers requiring positional confirmation", #9
                    "locate and correlate both buoy positions to establish gate reference data" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y1')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy"
        },
        "variations": [
                    "find the gate", #1
                    "locate the gate", #2
                    "look for the gate", #3
                    "search for the gate", #4
                    "scan for the gate", #5
                    "look around for the gate", #6
                    "try to find the gate", #7
                    "seek the gate", #8
                    "check for the gate", #9
                    "locate and recognize the gate", #10
                    "spot and confirm the gate", #11
                    "look out for the gate", #12
                    "locate the gate position", #13
                    "detect the gate position", #14
                    "approach the gate", #15
                    "recognize the gate formation", #16
                    "find and approach the gate", #17
                    "identify the gate", #18
                    "detect the gate", #19
                    "observe the gate", #20
                    "spot the gate", #21
                    "find the gate location", #22
                    "search the area for the gate", #23
                    "scan the area for the gate", #24
                    "look around the area for the gate", #25
                    "try to locate the gate", #26
                    "attempt to find the gate", #27
                    "attempt to locate the gate", #28
                    "attempt to identify the gate", #29
                    "attempt to detect the gate", #30
                    "seek and locate the gate", #31
                    "seek and identify the gate", #32
                    "seek and find the gate", #33
                    "check and locate the gate", #34
                    "check and identify the gate", #35
                    "check the area for the gate", #36
                    "verify the gate position", #37
                    "verify the gate location", #38
                    "watch for the gate", #39
                    "watch out for the gate", #40
                    "keep looking for the gate", #41
                    "keep searching for the gate", #42
                    "continue looking for the gate", #43
                    "continue searching for the gate", #44
                    "proceed to find the gate", #45
                    "proceed to locate the gate", #46
                    "move toward the gate", #47
                    "move in the direction of the gate", #48
                    "head toward the gate", #49
                    "head in the direction of the gate", #50
                    "navigate toward the gate", #51
                    "navigate in the direction of the gate", #52
                    "travel toward the gate", #53
                    "travel in the direction of the gate", #54
                    "advance toward the gate", #55
                    "advance in the direction of the gate", #56
                    "progress toward the gate", #57
                    "progress in the direction of the gate", #58
                    "move to find the gate", #59
                    "move to locate the gate", #60
                    "head to find the gate", #61
                    "head to locate the gate", #62
                    "navigate to find the gate", #63
                    "navigate to locate the gate", #64
                    "travel to find the gate", #65
                    "travel to locate the gate", #66
                    "advance to find the gate", #67
                    "advance to locate the gate", #68
                    "scan and identify the gate", #69
                    "scan and locate the gate", #70
                    "search and identify the gate", #71
                    "search and locate the gate", #72
                    "look and identify the gate", #73
                    "look and locate the gate", #74
                    "find the gate formation", #75
                    "locate the gate formation", #76
                    "identify the gate formation", #77
                    "detect the gate formation", #78
                    "recognize and locate the gate", #79
                    "recognize and identify the gate", #80
                    "find and identify the gate", #81
                    "find and locate the gate", #82
                    "find and detect the gate", #83
                    "find and observe the gate", #84
                    "locate and detect the gate", #85
                    "locate and observe the gate", #86
                    "spot and identify the gate", #87
                    "spot and locate the gate", #88
                    "observe and identify the gate", #89
                    "observe and locate the gate", #90
                    "search and find the gate", #91
                    "scan and find the gate", #92
                    "look and find the gate", #93
                    "seek and find the gate", #94
                    "check and find the gate", #95
                    "try and find the gate", #96
                    "attempt and locate the gate", #97
                    "verify and locate the gate", #98
                    "confirm the gate location", #99
                    "confirm the gate position" #100
        ],
        "test_variations": [
                    "gate acquisition is the primary objective requiring positive identification", #1
                    "visual confirmation of the gate structure must be established immediately", #2
                    "the gate perimeter should be located and monitored", #3
                    "gate positioning must be determined relative to current location", #4
                    "establish gate contact and report dimensional parameters", #5
                    "gate markers require systematic search and recognition procedures", #6
                    "initiate gate detection protocols and report findings", #7
                    "the target structure designated as gate requires location confirmation", #8
                    "gate-based reference point identification is operationally critical", #9
                    "execute gate reconnaissance and provide positional data" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y1_nip')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy"
        },
        "variations": [
                    "find both the gate buoys", #1
                    "spot two gate buoys", #2
                    "find both buoys of the gate", #3
                    "locate the two yellow buoys", #4
                    "look for the pair of gate buoys", #5
                    "find the yellow buoys forming the gate", #6
                    "search for the yellow buoys forming the gate", #7
                    "locate the gate buoys", #8
                    "identify the gate buoys", #9
                    "detect the gate buoys", #10
                    "spot the two yellow buoys of the gate", #11
                    "find the two yellow buoys of the gate", #12
                    "scan for both gate buoys", #13
                    "look around for the two yellow buoys", #14
                    "try to locate the gate buoys", #15
                    "try to spot both gate buoys", #16
                    "seek the two yellow buoys of the gate", #17
                    "attempt to detect the gate buoys", #18
                    "attempt to identify both gate buoys", #19
                    "check for the two yellow buoys", #20
                    "find and identify the gate buoys", #21
                    "detect and mark the gate buoys", #22
                    "identify and log the two yellow buoys", #23
                    "observe the gate buoys carefully", #24
                    "search around for the pair of gate buoys", #25
                    "scan the gate buoys", #26
                    "track the gate buoys", #27
                    "head towards the gate buoys", #28
                    "confirm the gate buoys", #29
                    "detect both buoys forming the gate", #30
                    "locate both yellow buoys", #31
                    "find the pair of buoys", #32
                    "spot the paired gate buoys", #33
                    "identify both gate markers", #34
                    "search for the two buoys", #35
                    "look for both yellow markers", #36
                    "scan for the gate markers", #37
                    "detect the dual gate buoys", #38
                    "locate the buoy pair", #39
                    "observe both gate buoys", #40
                    "find and confirm both buoys", #41
                    "locate and track the gate buoys", #42
                    "identify and position the two buoys", #43
                    "spot and verify the gate buoys", #44
                    "search and locate both buoys", #45
                    "scan and identify the gate buoys", #46
                    "watch for the paired buoys", #47
                    "look out for both gate markers", #48
                    "check and confirm the gate buoys", #49
                    "verify both yellow buoys", #50
                    "find the two markers of the gate", #51
                    "locate the gate's buoy markers", #52
                    "identify the gate buoy pair", #53
                    "detect the paired yellow buoys", #54
                    "spot the dual yellow markers", #55
                    "search the area for both gate buoys", #56
                    "scan the area for the two yellow buoys", #57
                    "look around the area for both buoys", #58
                    "try to identify both gate buoys", #59
                    "attempt to spot the gate buoy pair", #60
                    "attempt to locate both yellow markers", #61
                    "seek both gate buoys", #62
                    "seek the paired buoy markers", #63
                    "check for both gate markers", #64
                    "verify the gate buoy positions", #65
                    "confirm both buoy locations", #66
                    "observe and identify both buoys", #67
                    "monitor the gate buoys", #68
                    "track both yellow buoys", #69
                    "follow the gate buoys", #70
                    "proceed to find both buoys", #71
                    "proceed to locate the gate buoys", #72
                    "move toward both gate buoys", #73
                    "head toward the gate buoy pair", #74
                    "navigate toward both yellow buoys", #75
                    "travel toward the gate buoys", #76
                    "advance toward both gate markers", #77
                    "progress toward the gate buoys", #78
                    "find and approach both gate buoys", #79
                    "locate and approach the gate buoys", #80
                    "identify and approach both buoys", #81
                    "detect and approach the gate buoys", #82
                    "spot and approach both markers", #83
                    "search and find both gate buoys", #84
                    "scan and find the gate buoys", #85
                    "look and find both yellow buoys", #86
                    "seek and find the paired buoys", #87
                    "check and locate both buoys", #88
                    "try and find the gate buoys", #89
                    "attempt and identify both buoys", #90
                    "verify and locate the gate markers", #91
                    "confirm and track the gate buoys", #92
                    "locate the paired yellow gate buoys", #93
                    "find the dual positioned gate buoys", #94
                    "identify the two markers forming the gate", #95
                    "detect the positioned gate buoy pair", #96
                    "spot the arranged gate buoys", #97
                    "observe the gate buoy alignment", #98
                    "recognize the gate buoy configuration", #99
                    "establish both gate buoy positions" #100
        ],
        "test_variations": [
                    "acquire and verify the exact coordinates of both gate buoys in sequence", #1
                    "the gate structure is defined by two yellow buoys requiring dual confirmation", #2
                    "establish the gate reference frame by identifying and logging both marker positions", #3
                    "both yellow markers must be located and their spatial relationship documented", #4
                    "gate definition requires simultaneous detection of the paired buoy markers", #5
                    "buoy pair designation indicates gate functionality requiring identification of both elements", #6
                    "execute identification protocol for gate buoys and confirm marker alignment parameters", #7
                    "dual yellow buoy detection is mandatory for gate transit authorization", #8
                    "the gateway is formed by two distinct buoy markers requiring positional confirmation", #9
                    "locate and correlate both buoy positions to establish gate reference data" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y1_ip')}",
            "command": "find the gate",
            "output": "Plan: 1. find yellow buoy"
        },
        "variations": [
                    "find both the gate buoys", #1
                    "spot two gate buoys", #2
                    "find both buoys of the gate", #3
                    "locate the two yellow buoys", #4
                    "look for the pair of gate buoys", #5
                    "find the yellow buoys forming the gate", #6
                    "search for the yellow buoys forming the gate", #7
                    "locate the gate buoys", #8
                    "identify the gate buoys", #9
                    "detect the gate buoys", #10
                    "spot the two yellow buoys of the gate", #11
                    "find the two yellow buoys of the gate", #12
                    "scan for both gate buoys", #13
                    "look around for the two yellow buoys", #14
                    "try to locate the gate buoys", #15
                    "try to spot both gate buoys", #16
                    "seek the two yellow buoys of the gate", #17
                    "attempt to detect the gate buoys", #18
                    "attempt to identify both gate buoys", #19
                    "check for the two yellow buoys", #20
                    "find and identify the gate buoys", #21
                    "detect and mark the gate buoys", #22
                    "identify and log the two yellow buoys", #23
                    "observe the gate buoys carefully", #24
                    "search around for the pair of gate buoys", #25
                    "scan the gate buoys", #26
                    "track the gate buoys", #27
                    "head towards the gate buoys", #28
                    "confirm the gate buoys", #29
                    "detect both buoys forming the gate", #30
                    "locate both yellow buoys", #31
                    "find the pair of buoys", #32
                    "spot the paired gate buoys", #33
                    "identify both gate markers", #34
                    "search for the two buoys", #35
                    "look for both yellow markers", #36
                    "scan for the gate markers", #37
                    "detect the dual gate buoys", #38
                    "locate the buoy pair", #39
                    "observe both gate buoys", #40
                    "find and confirm both buoys", #41
                    "locate and track the gate buoys", #42
                    "identify and position the two buoys", #43
                    "spot and verify the gate buoys", #44
                    "search and locate both buoys", #45
                    "scan and identify the gate buoys", #46
                    "watch for the paired buoys", #47
                    "look out for both gate markers", #48
                    "check and confirm the gate buoys", #49
                    "verify both yellow buoys", #50
                    "find the two markers of the gate", #51
                    "locate the gate's buoy markers", #52
                    "identify the gate buoy pair", #53
                    "detect the paired yellow buoys", #54
                    "spot the dual yellow markers", #55
                    "search the area for both gate buoys", #56
                    "scan the area for the two yellow buoys", #57
                    "look around the area for both buoys", #58
                    "try to identify both gate buoys", #59
                    "attempt to spot the gate buoy pair", #60
                    "attempt to locate both yellow markers", #61
                    "seek both gate buoys", #62
                    "seek the paired buoy markers", #63
                    "check for both gate markers", #64
                    "verify the gate buoy positions", #65
                    "confirm both buoy locations", #66
                    "observe and identify both buoys", #67
                    "monitor the gate buoys", #68
                    "track both yellow buoys", #69
                    "follow the gate buoys", #70
                    "proceed to find both buoys", #71
                    "proceed to locate the gate buoys", #72
                    "move toward both gate buoys", #73
                    "head toward the gate buoy pair", #74
                    "navigate toward both yellow buoys", #75
                    "travel toward the gate buoys", #76
                    "advance toward both gate markers", #77
                    "progress toward the gate buoys", #78
                    "find and approach both gate buoys", #79
                    "locate and approach the gate buoys", #80
                    "identify and approach both buoys", #81
                    "detect and approach the gate buoys", #82
                    "spot and approach both markers", #83
                    "search and find both gate buoys", #84
                    "scan and find the gate buoys", #85
                    "look and find both yellow buoys", #86
                    "seek and find the paired buoys", #87
                    "check and locate both buoys", #88
                    "try and find the gate buoys", #89
                    "attempt and identify both buoys", #90
                    "verify and locate the gate markers", #91
                    "confirm and track the gate buoys", #92
                    "locate the paired yellow gate buoys", #93
                    "find the dual positioned gate buoys", #94
                    "identify the two markers forming the gate", #95
                    "detect the positioned gate buoy pair", #96
                    "spot the arranged gate buoys", #97
                    "observe the gate buoy alignment", #98
                    "recognize the gate buoy configuration", #99
                    "establish both gate buoy positions" #100
        ],
        "test_variations": [
                    "acquire and verify the exact coordinates of both gate buoys in sequence", #1
                    "the gate structure is defined by two yellow buoys requiring dual confirmation", #2
                    "establish the gate reference frame by identifying and logging both marker positions", #3
                    "both yellow markers must be located and their spatial relationship documented", #4
                    "gate definition requires simultaneous detection of the paired buoy markers", #5
                    "buoy pair designation indicates gate functionality requiring identification of both elements", #6
                    "execute identification protocol for gate buoys and confirm marker alignment parameters", #7
                    "dual yellow buoy detection is mandatory for gate transit authorization", #8
                    "the gateway is formed by two distinct buoy markers requiring positional confirmation", #9
                    "locate and correlate both buoy positions to establish gate reference data" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y2')}",
            "command": "pass through the gate",
            "output": "Plan: 1. pass through"
        },
        "variations": [
            "pass through the gate", #1
            "move through the gate", #2
            "pass through", #3
            "cross the gate", #4
            "pass across the gate", #5
            "cross through the gate", #6
            "move across the gate", #7
            "navigate through the gate", #8
            "proceed through the gate", #9
            "advance through the gate", #10
            "go through the gate", #11
            "head through the gate", #12
            "make your way through the gate", #13
            "travel through the gate", #14
            "move forward through the gate", #15
            "cross the gate area", #16
            "pass the gate", #17
            "pass forward through the gate", #18
            "cross forward through the gate", #19
            "move ahead through the gate", #20
            "move onward through the gate", #21
            "go forward through the gate", #22
            "move through the yellow gate", #23
            "advance across the gate", #24
            "proceed across the gate", #25
            "head across the gate", #26
            "navigate across the gate", #27
            "pass onward through the gate", #28
            "proceed onward through the gate", #29
            "move directly through the gate", #30
            "cross directly through the gate", #31
            "head directly through the gate", #32
            "travel directly through the gate", #33
            "advance directly through the gate", #34
            "transit through the gate", #35
            "move through the gate opening", #36
            "pass through the gate opening", #37
            "cross through the gate opening", #38
            "navigate through the gate opening", #39
            "proceed through the gate opening", #40
            "go across the gate", #41
            "head across the gate area", #42
            "travel across the gate", #43
            "advance across the gate area", #44
            "move across the gate area", #45
            "pass through the gate structure", #46
            "cross through the gate structure", #47
            "navigate through the gate structure", #48
            "proceed through the gate structure", #49
            "advance through the gate structure", #50
            "transit across the gate", #51
            "transit through the gate area", #52
            "traverse the gate", #53
            "traverse through the gate", #54
            "traverse across the gate", #55
            "move through the center of the gate", #56
            "pass through the center of the gate", #57
            "cross through the center of the gate", #58
            "head through the center of the gate", #59
            "navigate through the center of the gate", #60
            "proceed directly across the gate", #61
            "advance directly across the gate", #62
            "move directly across the gate", #63
            "head directly across the gate", #64
            "travel directly across the gate", #65
            "pass directly across the gate", #66
            "cross directly across the gate", #67
            "navigate directly across the gate", #68
            "proceed through the gate passage", #69
            "move through the gate passage", #70
            "cross through the gate passage", #71
            "navigate through the gate passage", #72
            "advance through the gate passage", #73
            "go through the gate opening", #74
            "head through the gate opening", #75
            "pass forward across the gate", #76
            "cross forward across the gate", #77
            "move forward across the gate", #78
            "head forward across the gate", #79
            "navigate forward across the gate", #80
            "proceed forward across the gate", #81
            "advance forward across the gate", #82
            "travel forward across the gate", #83
            "move onward across the gate", #84
            "proceed onward across the gate", #85
            "advance onward across the gate", #86
            "head onward across the gate", #87
            "go onward through the gate", #88
            "pass between the gate posts", #89
            "move between the gate posts", #90
            "cross between the gate posts", #91
            "navigate between the gate posts", #92
            "proceed between the gate posts", #93
            "advance between the gate posts", #94
            "head between the gate posts", #95
            "travel between the gate posts", #96
            "go between the gate posts", #97
            "transit between the gate posts", #98
            "traverse between the gate posts", #99
            "move through the gate threshold" #100
        ],
        "test_variations": [
                    "execute gate transit with both reference positions maintained in visual acquisition", #1
                    "proceed through the gate aperture maintaining equidistant spacing from the flanking posts", #2
                    "gate passage requires navigation through the defined reference channel", #3
                    "cross the gate boundary defined by the positioned anchors", #4
                    "complete gate traversal by passing through the yellow signal channel", #5
                    "transit authorization requires passage through both gate reference positions", #6
                    "the gate defines a specific transit corridor that must be followed exactly", #7
                    "navigate the gate passage while maintaining proper clearance from the positioned guides", #8
                    "gate crossing protocol requires directional passage between both defined positions", #9
                    "establish gate clearance by transiting through the complete signal perimeter" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('y2_nip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]

    },
    {
        "base":    {
            "context": f"{ctx_('y2_ip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y1')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. pass through"
        },
        "variations": [
            "pass through the gate", #1
            "move through the gate", #2
            "pass through", #3
            "cross the gate", #4
            "pass across the gate", #5
            "cross through the gate", #6
            "move across the gate", #7
            "navigate through the gate", #8
            "proceed through the gate", #9
            "advance through the gate", #10
            "go through the gate", #11
            "head through the gate", #12
            "make your way through the gate", #13
            "travel through the gate", #14
            "move forward through the gate", #15
            "cross the gate area", #16
            "pass the gate", #17
            "pass forward through the gate", #18
            "cross forward through the gate", #19
            "move ahead through the gate", #20
            "move onward through the gate", #21
            "go forward through the gate", #22
            "move through the yellow gate", #23
            "advance across the gate", #24
            "proceed across the gate", #25
            "head across the gate", #26
            "navigate across the gate", #27
            "pass onward through the gate", #28
            "proceed onward through the gate", #29
            "move directly through the gate", #30
            "cross directly through the gate", #31
            "head directly through the gate", #32
            "travel directly through the gate", #33
            "advance directly through the gate", #34
            "transit through the gate", #35
            "move through the gate opening", #36
            "pass through the gate opening", #37
            "cross through the gate opening", #38
            "navigate through the gate opening", #39
            "proceed through the gate opening", #40
            "go across the gate", #41
            "head across the gate area", #42
            "travel across the gate", #43
            "advance across the gate area", #44
            "move across the gate area", #45
            "pass through the gate structure", #46
            "cross through the gate structure", #47
            "navigate through the gate structure", #48
            "proceed through the gate structure", #49
            "advance through the gate structure", #50
            "transit across the gate", #51
            "transit through the gate area", #52
            "traverse the gate", #53
            "traverse through the gate", #54
            "traverse across the gate", #55
            "move through the center of the gate", #56
            "pass through the center of the gate", #57
            "cross through the center of the gate", #58
            "head through the center of the gate", #59
            "navigate through the center of the gate", #60
            "proceed directly across the gate", #61
            "advance directly across the gate", #62
            "move directly across the gate", #63
            "head directly across the gate", #64
            "travel directly across the gate", #65
            "pass directly across the gate", #66
            "cross directly across the gate", #67
            "navigate directly across the gate", #68
            "proceed through the gate passage", #69
            "move through the gate passage", #70
            "cross through the gate passage", #71
            "navigate through the gate passage", #72
            "advance through the gate passage", #73
            "go through the gate opening", #74
            "head through the gate opening", #75
            "pass forward across the gate", #76
            "cross forward across the gate", #77
            "move forward across the gate", #78
            "head forward across the gate", #79
            "navigate forward across the gate", #80
            "proceed forward across the gate", #81
            "advance forward across the gate", #82
            "travel forward across the gate", #83
            "move onward across the gate", #84
            "proceed onward across the gate", #85
            "advance onward across the gate", #86
            "head onward across the gate", #87
            "go onward through the gate", #88
            "pass between the gate posts", #89
            "move between the gate posts", #90
            "cross between the gate posts", #91
            "navigate between the gate posts", #92
            "proceed between the gate posts", #93
            "advance between the gate posts", #94
            "head between the gate posts", #95
            "travel between the gate posts", #96
            "go between the gate posts", #97
            "transit between the gate posts", #98
            "traverse between the gate posts", #99
            "move through the gate threshold" #100
        ],
        "test_variations": [
                    "execute gate transit with both reference positions maintained in visual acquisition", #1
                    "proceed through the gate aperture maintaining equidistant spacing from the flanking posts", #2
                    "gate passage requires navigation through the defined reference channel", #3
                    "cross the gate boundary defined by the positioned anchors", #4
                    "complete gate traversal by passing through the yellow signal channel", #5
                    "transit authorization requires passage through both gate reference positions", #6
                    "the gate defines a specific transit corridor that must be followed exactly", #7
                    "navigate the gate passage while maintaining proper clearance from the positioned guides", #8
                    "gate crossing protocol requires directional passage between both defined positions", #9
                    "establish gate clearance by transiting through the complete signal perimeter" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y1_nip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y1_ip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y0')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy 3. pass through"
        },
        "variations": [
            "pass through the gate", #1
            "move through the gate", #2
            "pass through", #3
            "cross the gate", #4
            "pass across the gate", #5
            "cross through the gate", #6
            "move across the gate", #7
            "navigate through the gate", #8
            "proceed through the gate", #9
            "advance through the gate", #10
            "go through the gate", #11
            "head through the gate", #12
            "make your way through the gate", #13
            "travel through the gate", #14
            "move forward through the gate", #15
            "cross the gate area", #16
            "pass the gate", #17
            "pass forward through the gate", #18
            "cross forward through the gate", #19
            "move ahead through the gate", #20
            "move onward through the gate", #21
            "go forward through the gate", #22
            "move through the yellow gate", #23
            "advance across the gate", #24
            "proceed across the gate", #25
            "head across the gate", #26
            "navigate across the gate", #27
            "pass onward through the gate", #28
            "proceed onward through the gate", #29
            "move directly through the gate", #30
            "cross directly through the gate", #31
            "head directly through the gate", #32
            "travel directly through the gate", #33
            "advance directly through the gate", #34
            "transit through the gate", #35
            "move through the gate opening", #36
            "pass through the gate opening", #37
            "cross through the gate opening", #38
            "navigate through the gate opening", #39
            "proceed through the gate opening", #40
            "go across the gate", #41
            "head across the gate area", #42
            "travel across the gate", #43
            "advance across the gate area", #44
            "move across the gate area", #45
            "pass through the gate structure", #46
            "cross through the gate structure", #47
            "navigate through the gate structure", #48
            "proceed through the gate structure", #49
            "advance through the gate structure", #50
            "transit across the gate", #51
            "transit through the gate area", #52
            "traverse the gate", #53
            "traverse through the gate", #54
            "traverse across the gate", #55
            "move through the center of the gate", #56
            "pass through the center of the gate", #57
            "cross through the center of the gate", #58
            "head through the center of the gate", #59
            "navigate through the center of the gate", #60
            "proceed directly across the gate", #61
            "advance directly across the gate", #62
            "move directly across the gate", #63
            "head directly across the gate", #64
            "travel directly across the gate", #65
            "pass directly across the gate", #66
            "cross directly across the gate", #67
            "navigate directly across the gate", #68
            "proceed through the gate passage", #69
            "move through the gate passage", #70
            "cross through the gate passage", #71
            "navigate through the gate passage", #72
            "advance through the gate passage", #73
            "go through the gate opening", #74
            "head through the gate opening", #75
            "pass forward across the gate", #76
            "cross forward across the gate", #77
            "move forward across the gate", #78
            "head forward across the gate", #79
            "navigate forward across the gate", #80
            "proceed forward across the gate", #81
            "advance forward across the gate", #82
            "travel forward across the gate", #83
            "move onward across the gate", #84
            "proceed onward across the gate", #85
            "advance onward across the gate", #86
            "head onward across the gate", #87
            "go onward through the gate", #88
            "pass between the gate posts", #89
            "move between the gate posts", #90
            "cross between the gate posts", #91
            "navigate between the gate posts", #92
            "proceed between the gate posts", #93
            "advance between the gate posts", #94
            "head between the gate posts", #95
            "travel between the gate posts", #96
            "go between the gate posts", #97
            "transit between the gate posts", #98
            "traverse between the gate posts", #99
            "move through the gate threshold" #100
        ],
        "test_variations": [
                    "execute gate transit with both reference positions maintained in visual acquisition", #1
                    "proceed through the gate aperture maintaining equidistant spacing from the flanking posts", #2
                    "gate passage requires navigation through the defined reference channel", #3
                    "cross the gate boundary defined by the positioned anchors", #4
                    "complete gate traversal by passing through the yellow signal channel", #5
                    "transit authorization requires passage through both gate reference positions", #6
                    "the gate defines a specific transit corridor that must be followed exactly", #7
                    "navigate the gate passage while maintaining proper clearance from the positioned guides", #8
                    "gate crossing protocol requires directional passage between both defined positions", #9
                    "establish gate clearance by transiting through the complete signal perimeter" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y0_nip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy 3. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('y0_ip')}",
            "command": "pass through the gate",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy 3. pass through"
        },
        "variations": [
                    "pass between the gate buoys", #1
                    "cross between the gate buoys", #2
                    "move between the gate buoys", #3
                    "proceed between the gate buoys", #4
                    "advance between the gate buoys", #5
                    "head between the gate buoys", #6
                    "navigate between the gate buoys", #7
                    "cross the two gate buoys", #8
                    "pass the two yellow buoys", #9
                    "travel between the gate buoys", #10
                    "move across the two gate buoys", #11
                    "pass across the two yellow buoys", #12
                    "cross forward between the gate buoys", #13
                    "advance forward between the gate buoys", #14
                    "go between the gate buoys", #15
                    "navigate directly between the gate buoys", #16
                    "move through between the gate buoys", #17
                    "transit between the gate buoys", #18
                    "pass through between the gate buoys", #19
                    "cross through between the gate buoys", #20
                    "head through between the gate buoys", #21
                    "proceed through between the gate buoys", #22
                    "advance through between the gate buoys", #23
                    "navigate through between the gate buoys", #24
                    "traverse between the gate buoys", #25
                    "move forward between the gate buoys", #26
                    "pass forward between the gate buoys", #27
                    "head forward between the gate buoys", #28
                    "proceed forward between the gate buoys", #29
                    "advance forward between the gate buoys", #30
                    "navigate forward between the gate buoys", #31
                    "cross forward through the gate buoys", #32
                    "move forward through the gate buoys", #33
                    "pass forward through the gate buoys", #34
                    "head forward through the gate buoys", #35
                    "proceed forward through the gate buoys", #36
                    "advance forward through the gate buoys", #37
                    "navigate forward through the gate buoys", #38
                    "move across between the gate buoys", #39
                    "pass across between the gate buoys", #40
                    "cross across between the gate buoys", #41
                    "head across between the gate buoys", #42
                    "proceed across between the gate buoys", #43
                    "advance across between the gate buoys", #44
                    "navigate across between the gate buoys", #45
                    "move directly between the gate buoys", #46
                    "pass directly between the gate buoys", #47
                    "cross directly between the gate buoys", #48
                    "head directly between the gate buoys", #49
                    "proceed directly between the gate buoys", #50
                    "advance directly between the gate buoys", #51
                    "move through the gate buoys", #52
                    "pass through the gate buoys", #53
                    "cross through the gate buoys", #54
                    "head through the gate buoys", #55
                    "proceed through the gate buoys", #56
                    "advance through the gate buoys", #57
                    "navigate through the gate buoys", #58
                    "travel through the gate buoys", #59
                    "go through the gate buoys", #60
                    "move in between the gate buoys", #61
                    "pass in between the gate buoys", #62
                    "cross in between the gate buoys", #63
                    "head in between the gate buoys", #64
                    "proceed in between the gate buoys", #65
                    "advance in between the gate buoys", #66
                    "navigate in between the gate buoys", #67
                    "travel in between the gate buoys", #68
                    "go in between the gate buoys", #69
                    "move past the gate buoys", #70
                    "pass past the gate buoys", #71
                    "cross past the gate buoys", #72
                    "head past the gate buoys", #73
                    "proceed past the gate buoys", #74
                    "advance past the gate buoys", #75
                    "navigate past the gate buoys", #76
                    "travel past the gate buoys", #77
                    "go past the gate buoys", #78
                    "move alongside the gate buoys", #79
                    "pass alongside the gate buoys", #80
                    "cross alongside the gate buoys", #81
                    "head alongside the gate buoys", #82
                    "proceed alongside the gate buoys", #83
                    "advance alongside the gate buoys", #84
                    "navigate alongside the gate buoys", #85
                    "travel alongside the gate buoys", #86
                    "go alongside the gate buoys", #87
                    "move onward between the gate buoys", #88
                    "pass onward between the gate buoys", #89
                    "cross onward between the gate buoys", #90
                    "head onward between the gate buoys", #91
                    "proceed onward between the gate buoys", #92
                    "advance onward between the gate buoys", #93
                    "navigate onward between the gate buoys", #94
                    "travel onward between the gate buoys", #95
                    "transit through between the gate buoys", #96
                    "traverse through between the gate buoys", #97
                    "move steadily between the gate buoys", #98
                    "proceed steadily between the gate buoys", #99
                    "navigate steadily between the gate buoys" #100
        ],
        "test_variations": [
                    "position your trajectory to pass centrally between both yellow gate buoys", #1
                    "execute transit through the buoy channel while maintaining equidistant positioning", #2
                    "the gate buoys define a navigable corridor that requires precise passage", #3
                    "cross through the defined aperture established by the two yellow buoys", #4
                    "navigate the buoy passage ensuring neither buoy is contacted during transit", #5
                    "advance through the gap between the gate buoys with controlled approach", #6
                    "buoy clearance protocol requires passage between both established positions", #7
                    "complete the gate transit by moving through the yellow buoy alignment", #8
                    "proceed through the buoy corridor following the established reference line", #9
                    "establish gate passage confirmation by successfully transiting between both buoys" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('nip')}",
            "command": "perform a survey",
            "output": "Plan: 1. survey"
        },
        "variations": [
                    "perform a survey", #1
                    "get some interesting points", #2
                    "explore the surroundings", #3
                    "survey here", #4
                    "find some interesting points", #5
                    "survey this zone", #6
                    "scan around for interesting points", #7
                    "explore the environment", #8
                    "survey around here", #9
                    "scan this zone", #10
                    "do a survey", #11
                    "gather interesting points", #12
                    "explore nearby", #13
                    "survey", #14
                    "look for interesting points", #15
                    "survey this sector", #16
                    "scan for things of interest", #17
                    "explore environment", #18
                    "survey around", #19
                    "scan for discoveries", #20
                    "look around for notable spots", #21
                    "search the surroundings", #22
                    "check the vicinity for points", #23
                    "explore locally", #24
                    "survey in this location", #25
                    "look for details nearby", #26
                    "explore casually", #27
                    "survey the region quickly", #28
                    "check around here", #29
                    "explore the immediate surroundings", #30
                    "scan loosely for features", #31
                    "look for anything interesting", #32
                    "explore outward from here", #33
                    "survey without detail", #34
                    "scan lightly", #35
                    "explore in a broad way", #36
                    "survey generally", #37
                    "look for potential findings", #38
                    "explore freely", #39
                    "check casually for features", #40
                    "conduct a survey", #41
                    "scan the locality", #42
                    "explore this area", #43
                    "investigate the surroundings", #44
                    "look for points of interest", #45
                    "survey the vicinity", #46
                    "scan for notable features", #47
                    "explore broadly", #48
                    "check the area for interesting elements", #49
                    "survey casually", #50
                    "look around the surroundings", #51
                    "explore the space around here", #52
                    "scan this region", #53
                    "find interesting details", #54
                    "survey the local area", #55
                    "check for things of note", #56
                    "explore the perimeter", #57
                    "scan around the vicinity", #58
                    "look for notable discoveries", #59
                    "survey loosely", #60
                    "explore superficially", #61
                    "scan the environment", #62
                    "investigate points nearby", #63
                    "survey the terrain", #64
                    "look for features around here", #65
                    "explore the landscape", #66
                    "check the surroundings for interest", #67
                    "scan for areas of note", #68
                    "survey the expanse", #69
                    "explore this locale", #70
                    "look for interesting sectors", #71
                    "survey nonspecifically", #72
                    "scan for general features", #73
                    "explore the locality broadly", #74
                    "check around for discoveries", #75
                    "survey in a general manner", #76
                    "look around this zone", #77
                    "explore without specifics", #78
                    "scan this locality", #79
                    "investigate the general area", #80
                    "survey the zone quickly", #81
                    "explore with casual interest", #82
                    "check the region for points", #83
                    "look for things worth noting", #84
                    "survey the surroundings lightly", #85
                    "scan around loosely", #86
                    "explore the area freely", #87
                    "investigate lightly", #88
                    "survey the environment casually", #89
                    "look for any interesting aspects", #90
                    "explore generally around here", #91
                    "check casually around", #92
                    "scan the surroundings lightly", #93
                    "survey informally", #94
                    "explore in a relaxed manner", #95
                    "look for anything of note", #96
                    "investigate the locality", #97
                    "survey the space", #98
                    "scan for general areas of interest", #99
                    "explore the vicinity with curiosity" #100
        ],
        "test_variations": [
                    "conduct a comprehensive survey of this sector identifying all notable geographical and structural features present", #1
                    "systematically explore the area documenting interesting points and their spatial relationships to current position", #2
                    "perform an environmental scan to identify potential sites of interest with tactical significance assessment", #3
                    "survey the region using systematic observation techniques to locate and categorize points of potential interest", #4
                    "execute reconnaissance operations to identify interesting aspects within the surveyed area for mission analysis", #5
                    "conduct a detailed environmental survey while cataloging notable discoveries and their operational relevance", #6
                    "explore the surroundings with methodical attention to identify interesting points and feature documentation", #7
                    "perform area reconnaissance to establish survey points and identify interesting locations with positional data", #8
                    "investigate the zone systematically to locate and assess interesting areas based on established criteria", #9
                    "execute a comprehensive survey protocol to identify all points of interest and their tactical relevance parameters" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('y0_nip')}",
            "command": "perform a survey",
            "output": "Plan: 1. survey"
        },
        "variations": [
                    "look for potential buoys", #1
                    "find some interesting points that might be buoys", #2
                    "look for objects that could be buoys", #3
                    "find potential buoys", #4
                    "survey to find potential buoys", #5
                    "explore to find something that might be a buoy", #6
                    "look for things that could be buoys", #7
                    "scan the surroundings for potential buoys", #8
                    "survey for stuff that could be a buoy", #9
                    "get the location of a few potential buoys", #10
                    "explore for buoy candidates", #11
                    "survey for possible buoys", #12
                    "perform a survey to find buoys", #13
                    "get interesting points that may be buoys", #14
                    "find interesting points resembling buoys", #15
                    "find potential buoys nearby", #16
                    "find something that could be a buoy around here", #17
                    "look for potential buoys in sight", #18
                    "scan for interesting things that look like buoys", #19
                    "look around for potential buoys", #20
                    "search for buoy-like objects", #21
                    "look for buoy candidates in range", #22
                    "spot possible buoys in the distance", #23
                    "try to find potential buoys", #24
                    "seek out buoys that might be present", #25
                    "check the surroundings for buoys", #26
                    "explore to spot buoy-like shapes", #27
                    "look for potential buoys ahead", #28
                    "scan casually for buoy candidates", #29
                    "look around loosely for buoys", #30
                    "survey casually for buoy possibilities", #31
                    "explore freely for anything buoy-like", #32
                    "check around here for potential buoys", #33
                    "look for nearby buoys", #34
                    "search around for buoy-like items", #35
                    "look for possible buoys ahead", #36
                    "scan loosely for buoy points", #37
                    "seek potential buoys nearby", #38
                    "look outward for buoy-like objects", #39
                    "try spotting buoys in the vicinity", #40
                    "locate buoy positions", #41
                    "identify potential buoys in the area", #42
                    "search for buoys in range", #43
                    "look for buoy formations", #44
                    "survey the area for buoys", #45
                    "explore for visible buoys", #46
                    "scan for buoy shapes", #47
                    "find buoy-shaped objects", #48
                    "look for anything resembling a buoy", #49
                    "search for buoy signatures", #50
                    "explore to locate buoys", #51
                    "check for buoy presence", #52
                    "look for indicators of buoys", #53
                    "survey for buoy sightings", #54
                    "scan for buoy features", #55
                    "seek out any visible buoys", #56
                    "try to spot buoys", #57
                    "look for buoys that stand out", #58
                    "search the vicinity for buoys", #59
                    "explore to find buoy positions", #60
                    "check casually for buoys", #61
                    "look loosely for buoy markers", #62
                    "survey informally for buoys", #63
                    "scan freely for buoy objects", #64
                    "explore generally for buoys", #65
                    "seek potential buoy locations", #66
                    "look broadly for buoys", #67
                    "search casually for buoy-like forms", #68
                    "find possible buoy locations", #69
                    "investigate for buoys in the area", #70
                    "look for observable buoys", #71
                    "explore around for buoys", #72
                    "survey loosely for buoy points", #73
                    "scan the area for potential buoys", #74
                    "check around for buoy sightings", #75
                    "look for buoys or similar objects", #76
                    "search for anything that looks like a buoy", #77
                    "explore for buoy candidates in sight", #78
                    "survey to locate any buoys", #79
                    "scan for possible buoy locations", #80
                    "find things that might be buoys", #81
                    "look for objects in the distance that could be buoys", #82
                    "explore to identify buoy positions", #83
                    "check the surroundings for buoy-like features", #84
                    "survey for any visible buoys nearby", #85
                    "look for potential buoy sightings", #86
                    "scan casually around for buoys", #87
                    "seek to identify potential buoys", #88
                    "explore to spot any buoys", #89
                    "look for items that resemble buoys", #90
                    "survey the zone for possible buoys", #91
                    "search the area loosely for buoys", #92
                    "look around the vicinity for buoys", #93
                    "scan for any buoy-like markers", #94
                    "explore generally in search of buoys", #95
                    "check around casually for buoys", #96
                    "look for buoys or buoy candidates", #97
                    "survey informally for buoy locations", #98
                    "scan the surroundings for anything buoy-like", #99
                    "explore freely to locate potential buoys" #100
        ],
        "test_variations": [
                    "conduct systematic reconnaissance to identify and catalog all potential buoy locations within sensor range", #1
                    "survey the operational area documenting any objects with buoy-like characteristics for tactical assessment", #2
                    "execute buoy detection procedures scanning for shapes and formations consistent with buoy positioning", #3
                    "search the vicinity using methodical observation to identify possible buoy candidates and their locations", #4
                    "perform environmental analysis to locate potential buoys and classify their characteristics for mission planning", #5
                    "explore the area comprehensively to identify any objects that may constitute navigational buoys", #6
                    "conduct surveillance operations to detect potential buoys within the search perimeter with positional data", #7
                    "survey the surroundings systematically searching for buoy-like objects and establishing their coordinates", #8
                    "execute reconnaissance protocol to identify and map all potential buoy locations in the operational sector", #9
                    "perform detailed area analysis to locate, identify and document any potential buoys present in the zone" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('y2_nip')}",
            "command": "perform a survey",
            "output": "Plan: 1. survey"
        },
        "variations": [
                    "look for potential buoys", #1
                    "find some interesting points that might be buoys", #2
                    "look for objects that could be buoys", #3
                    "find potential buoys", #4
                    "survey to find potential buoys", #5
                    "explore to find something that might be a buoy", #6
                    "look for things that could be buoys", #7
                    "scan the surroundings for potential buoys", #8
                    "survey for stuff that could be a buoy", #9
                    "get the location of a few potential buoys", #10
                    "explore for buoy candidates", #11
                    "survey for possible buoys", #12
                    "perform a survey to find buoys", #13
                    "get interesting points that may be buoys", #14
                    "find interesting points resembling buoys", #15
                    "find potential buoys nearby", #16
                    "find something that could be a buoy around here", #17
                    "look for potential buoys in sight", #18
                    "scan for interesting things that look like buoys", #19
                    "look around for potential buoys", #20
                    "search for buoy-like objects", #21
                    "look for buoy candidates in range", #22
                    "spot possible buoys in the distance", #23
                    "try to find potential buoys", #24
                    "seek out buoys that might be present", #25
                    "check the surroundings for buoys", #26
                    "explore to spot buoy-like shapes", #27
                    "look for potential buoys ahead", #28
                    "scan casually for buoy candidates", #29
                    "look around loosely for buoys", #30
                    "survey casually for buoy possibilities", #31
                    "explore freely for anything buoy-like", #32
                    "check around here for potential buoys", #33
                    "look for nearby buoys", #34
                    "search around for buoy-like items", #35
                    "look for possible buoys ahead", #36
                    "scan loosely for buoy points", #37
                    "seek potential buoys nearby", #38
                    "look outward for buoy-like objects", #39
                    "try spotting buoys in the vicinity", #40
                    "locate buoy positions", #41
                    "identify potential buoys in the area", #42
                    "search for buoys in range", #43
                    "look for buoy formations", #44
                    "survey the area for buoys", #45
                    "explore for visible buoys", #46
                    "scan for buoy shapes", #47
                    "find buoy-shaped objects", #48
                    "look for anything resembling a buoy", #49
                    "search for buoy signatures", #50
                    "explore to locate buoys", #51
                    "check for buoy presence", #52
                    "look for indicators of buoys", #53
                    "survey for buoy sightings", #54
                    "scan for buoy features", #55
                    "seek out any visible buoys", #56
                    "try to spot buoys", #57
                    "look for buoys that stand out", #58
                    "search the vicinity for buoys", #59
                    "explore to find buoy positions", #60
                    "check casually for buoys", #61
                    "look loosely for buoy markers", #62
                    "survey informally for buoys", #63
                    "scan freely for buoy objects", #64
                    "explore generally for buoys", #65
                    "seek potential buoy locations", #66
                    "look broadly for buoys", #67
                    "search casually for buoy-like forms", #68
                    "find possible buoy locations", #69
                    "investigate for buoys in the area", #70
                    "look for observable buoys", #71
                    "explore around for buoys", #72
                    "survey loosely for buoy points", #73
                    "scan the area for potential buoys", #74
                    "check around for buoy sightings", #75
                    "look for buoys or similar objects", #76
                    "search for anything that looks like a buoy", #77
                    "explore for buoy candidates in sight", #78
                    "survey to locate any buoys", #79
                    "scan for possible buoy locations", #80
                    "find things that might be buoys", #81
                    "look for objects in the distance that could be buoys", #82
                    "explore to identify buoy positions", #83
                    "check the surroundings for buoy-like features", #84
                    "survey for any visible buoys nearby", #85
                    "look for potential buoy sightings", #86
                    "scan casually around for buoys", #87
                    "seek to identify potential buoys", #88
                    "explore to spot any buoys", #89
                    "look for items that resemble buoys", #90
                    "survey the zone for possible buoys", #91
                    "search the area loosely for buoys", #92
                    "look around the vicinity for buoys", #93
                    "scan for any buoy-like markers", #94
                    "explore generally in search of buoys", #95
                    "check around casually for buoys", #96
                    "look for buoys or buoy candidates", #97
                    "survey informally for buoy locations", #98
                    "scan the surroundings for anything buoy-like", #99
                    "explore freely to locate potential buoys" #100
        ],
        "test_variations": [
                    "conduct systematic reconnaissance to identify and catalog all potential buoy locations within sensor range", #1
                    "survey the operational area documenting any objects with buoy-like characteristics for tactical assessment", #2
                    "execute buoy detection procedures scanning for shapes and formations consistent with buoy positioning", #3
                    "search the vicinity using methodical observation to identify possible buoy candidates and their locations", #4
                    "perform environmental analysis to locate potential buoys and classify their characteristics for mission planning", #5
                    "explore the area comprehensively to identify any objects that may constitute navigational buoys", #6
                    "conduct surveillance operations to detect potential buoys within the search perimeter with positional data", #7
                    "survey the surroundings systematically searching for buoy-like objects and establishing their coordinates", #8
                    "execute reconnaissance protocol to identify and map all potential buoy locations in the operational sector", #9
                    "perform detailed area analysis to locate, identify and document any potential buoys present in the zone" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('ip')}",
            "command": "map the area",
            "output": "Plan: 1. map area"
        },
        "variations": [
                    "map the area", #1
                    "analyze the area", #2
                    "parse all points of interest", #3
                    "analyze the interesting points", #4
                    "parse each object of interest", #5
                    "examine every point closely", #6
                    "map the interesting points", #7
                    "analyze the surroundings", #8
                    "parse the region carefully", #9
                    "examine the region fully", #10
                    "map systematically", #11
                    "analyze systematically", #12
                    "parse the points of interest", #13
                    "analyze specific points", #14
                    "parse notable objects", #15
                    "examine identified points", #16
                    "map interesting points precisely", #17
                    "analyze surrounding details", #18
                    "parse with care", #19
                    "examine thoroughly", #20
                    "document every feature", #21
                    "record detailed findings", #22
                    "chart each point of interest", #23
                    "evaluate systematically", #24
                    "inspect with precision", #25
                    "note each object carefully", #26
                    "map with accuracy", #27
                    "analyze comprehensively", #28
                    "study each feature in depth", #29
                    "record observations in detail", #30
                    "track each identified point", #31
                    "assess methodically", #32
                    "examine with rigor", #33
                    "parse every listed item", #34
                    "analyze with structure", #35
                    "review all findings carefully", #36
                    "compile mapped points", #37
                    "verify every detail", #38
                    "break down observations", #39
                    "systematically log features", #40
                    "catalog the region", #41
                    "process the area data", #42
                    "dissect the findings", #43
                    "examine point by point", #44
                    "map the features", #45
                    "analyze methodically", #46
                    "parse the data carefully", #47
                    "document the analysis", #48
                    "record the mapping", #49
                    "evaluate the points", #50
                    "inspect the surroundings", #51
                    "note the observations", #52
                    "map the coordinates", #53
                    "analyze the layout", #54
                    "parse the structure", #55
                    "examine the details", #56
                    "document the details", #57
                    "record the analysis", #58
                    "chart the positions", #59
                    "evaluate the findings", #60
                    "inspect the details", #61
                    "note the findings", #62
                    "map the positions", #63
                    "analyze the structure", #64
                    "parse the observations", #65
                    "examine the findings", #66
                    "document the observations", #67
                    "record the findings", #68
                    "chart the area", #69
                    "evaluate the layout", #70
                    "inspect the area", #71
                    "note the layout", #72
                    "map the layout", #73
                    "analyze the observations", #74
                    "parse the area", #75
                    "examine the analysis", #76
                    "document the area", #77
                    "record the area", #78
                    "chart the observations", #79
                    "evaluate the structure", #80
                    "inspect the coordinates", #81
                    "note the structure", #82
                    "map the observations", #83
                    "analyze the data", #84
                    "parse the details", #85
                    "examine the coordinates", #86
                    "document the mapping", #87
                    "record the layout", #88
                    "chart the data", #89
                    "evaluate the analysis", #90
                    "inspect the observations", #91
                    "note the analysis", #92
                    "map the data", #93
                    "analyze the coordinates", #94
                    "parse the layout", #95
                    "examine the structure", #96
                    "document the coordinates", #97
                    "record the observations", #98
                    "chart the analysis", #99
                    "evaluate the observations" #100
        ],
        "test_variations": [
                    "execute comprehensive spatial analysis requiring point-by-point evaluation and detailed structural documentation", #1
                    "perform systematic area mapping with full data parsing and rigorous analysis of all identified features", #2
                    "conduct detailed region examination documenting each point of interest with methodical classification procedures", #3
                    "analyze the zone using structured protocols to map coordinates and parse all observable elements", #4
                    "execute thorough environmental analysis mapping features and parsing observations with complete documentation", #5
                    "perform detailed inspection parsing all points of interest with systematic evaluation and precise positioning", #6
                    "conduct comprehensive analysis examining the area structure mapping observations with rigorous data assessment", #7
                    "analyze systematically by parsing each feature recording coordinates with detailed structural examination protocols", #8
                    "execute detailed mapping procedures documenting observations parsing spatial data with comprehensive analysis standards", #9
                    "perform structured area analysis mapping all observations parsing features with methodical evaluation criteria" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y0_ip')}",
            "command": "map the area",
            "output": "Plan: 1. map area"
        },
        "variations": [
                    "map the buoy area", #1
                    "parse everything in the buoy area", #2
                    "analyze every object in the buoy area", #3
                    "analyze the buoy area", #4
                    "parse the buoy area", #5
                    "parse every potential buoy", #6
                    "analyze every potential buoy", #7
                    "scope every interesting point that could be a buoy", #8
                    "check each point that could be a buoy", #9
                    "parse everything that could be a buoy", #10
                    "analyze each object that may be a buoy", #11
                    "examine every potential buoy", #12
                    "examine each object that could be a buoy", #13
                    "parse every point of interest that is potentially a buoy", #14
                    "analyze the points that might be buoys", #15
                    "inspect every potential buoy", #16
                    "evaluate each potential buoy", #17
                    "assess every point that could be a buoy", #18
                    "investigate each potential buoy", #19
                    "review every buoy candidate", #20
                    "study each object that might be a buoy", #21
                    "check all potential buoys one by one", #22
                    "assess each buoy candidate thoroughly", #23
                    "inspect every object that could be a buoy", #24
                    "examine each point carefully for buoy features", #25
                    "chart all potential buoys precisely", #26
                    "evaluate objects that resemble buoys", #27
                    "study every object that might be buoy-like", #28
                    "log each potential buoy with detail", #29
                    "detail every buoy candidate clearly", #30
                    "assess all points of interest systematically", #31
                    "inspect each potential buoy thoroughly", #32
                    "evaluate points that might represent buoys", #33
                    "document each potential buoy", #34
                    "analyze and record each candidate buoy", #35
                    "map out all potential buoy locations", #36
                    "check each object for buoy characteristics", #37
                    "evaluate each potential buoy with rigor", #38
                    "assess potential buoy positions carefully", #39
                    "inspect all buoy candidates in detail", #40
                    "study and log each potential buoy", #41
                    "record every buoy candidate systematically", #42
                    "detail each object for buoy confirmation", #43
                    "review each buoy-like point carefully", #44
                    "document and analyze every potential buoy", #45
                    "parse all buoy-related data", #46
                    "analyze the buoy location data", #47
                    "map every buoy position", #48
                    "examine the buoy candidates closely", #49
                    "inspect and record potential buoys", #50
                    "assess buoy candidate characteristics", #51
                    "evaluate the buoy area in detail", #52
                    "study the potential buoy positions", #53
                    "check for buoy indicators", #54
                    "log buoy candidate positions", #55
                    "document the buoy analysis", #56
                    "parse buoy position data", #57
                    "analyze buoy candidate objects", #58
                    "map the buoy positions", #59
                    "examine buoy location candidates", #60
                    "inspect buoy area objects", #61
                    "evaluate buoy candidates", #62
                    "assess buoy locations", #63
                    "investigate buoy candidates", #64
                    "review buoy positions", #65
                    "study buoy area features", #66
                    "check buoy candidates", #67
                    "assess buoy features", #68
                    "inspect buoy candidates", #69
                    "examine buoy characteristics", #70
                    "document buoy candidates", #71
                    "analyze buoy characteristics", #72
                    "map buoy candidates", #73
                    "parse buoy candidates", #74
                    "evaluate buoy area", #75
                    "assess buoy area objects", #76
                    "investigate buoy area", #77
                    "review buoy area data", #78
                    "study buoy candidates", #79
                    "check buoy area", #80
                    "detail buoy positions", #81
                    "record buoy analysis", #82
                    "log buoy positions", #83
                    "examine each potential buoy object", #84
                    "parse and evaluate buoy candidates", #85
                    "analyze and map buoy positions", #86
                    "inspect and document buoy candidates", #87
                    "assess and record buoy positions", #88
                    "evaluate and analyze buoy area", #89
                    "map and analyze buoy candidates", #90
                    "examine and parse buoy data", #91
                    "detail and assess buoy candidates", #92
                    "inspect and evaluate buoy positions", #93
                    "review and analyze buoy candidates", #94
                    "study and document buoy area", #95
                    "check and assess buoy candidates", #96
                    "investigate and map buoy positions", #97
                    "document and evaluate buoy candidates", #98
                    "analyze and assess buoy area", #99
                    "parse and map buoy candidates" #100
        ],
        "test_variations": [
                    "execute comprehensive buoy area mapping protocol parsing all candidate objects with detailed feature analysis", #1
                    "conduct systematic evaluation of potential buoy candidates documenting positional data and characteristic parameters", #2
                    "perform detailed analysis of the buoy area examining each candidate object for confirmation of buoy designation", #3
                    "map all buoy candidates within the area sector parsing their characteristics and establishing reference coordinates", #4
                    "analyze the buoy location zone examining every potential candidate with rigorous classification procedures", #5
                    "execute buoy candidate assessment parsing all area objects and recording positional and characteristical data", #6
                    "conduct thorough analysis of potential buoy objects detailing positions and evaluating classification criteria", #7
                    "map the buoy candidates systematically analyzing the area with methodical examination of each object", #8
                    "perform detailed parsing of buoy area data evaluating all candidates and documenting their positions", #9
                    "assess all potential buoy candidates within the area mapping their locations with comprehensive feature evaluation" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y2_ip')}",
            "command": "map the area",
            "output": "Plan: 1. map area"
        },
        "variations": [
                    "map the buoy area", #1
                    "parse everything in the buoy area", #2
                    "analyze every object in the buoy area", #3
                    "analyze the buoy area", #4
                    "parse the buoy area", #5
                    "parse every potential buoy", #6
                    "analyze every potential buoy", #7
                    "scope every interesting point that could be a buoy", #8
                    "check each point that could be a buoy", #9
                    "parse everything that could be a buoy", #10
                    "analyze each object that may be a buoy", #11
                    "examine every potential buoy", #12
                    "examine each object that could be a buoy", #13
                    "parse every point of interest that is potentially a buoy", #14
                    "analyze the points that might be buoys", #15
                    "inspect every potential buoy", #16
                    "evaluate each potential buoy", #17
                    "assess every point that could be a buoy", #18
                    "investigate each potential buoy", #19
                    "review every buoy candidate", #20
                    "study each object that might be a buoy", #21
                    "check all potential buoys one by one", #22
                    "assess each buoy candidate thoroughly", #23
                    "inspect every object that could be a buoy", #24
                    "examine each point carefully for buoy features", #25
                    "chart all potential buoys precisely", #26
                    "evaluate objects that resemble buoys", #27
                    "study every object that might be buoy-like", #28
                    "log each potential buoy with detail", #29
                    "detail every buoy candidate clearly", #30
                    "assess all points of interest systematically", #31
                    "inspect each potential buoy thoroughly", #32
                    "evaluate points that might represent buoys", #33
                    "document each potential buoy", #34
                    "analyze and record each candidate buoy", #35
                    "map out all potential buoy locations", #36
                    "check each object for buoy characteristics", #37
                    "evaluate each potential buoy with rigor", #38
                    "assess potential buoy positions carefully", #39
                    "inspect all buoy candidates in detail", #40
                    "study and log each potential buoy", #41
                    "record every buoy candidate systematically", #42
                    "detail each object for buoy confirmation", #43
                    "review each buoy-like point carefully", #44
                    "document and analyze every potential buoy", #45
                    "parse all buoy-related data", #46
                    "analyze the buoy location data", #47
                    "map every buoy position", #48
                    "examine the buoy candidates closely", #49
                    "inspect and record potential buoys", #50
                    "assess buoy candidate characteristics", #51
                    "evaluate the buoy area in detail", #52
                    "study the potential buoy positions", #53
                    "check for buoy indicators", #54
                    "log buoy candidate positions", #55
                    "document the buoy analysis", #56
                    "parse buoy position data", #57
                    "analyze buoy candidate objects", #58
                    "map the buoy positions", #59
                    "examine buoy location candidates", #60
                    "inspect buoy area objects", #61
                    "evaluate buoy candidates", #62
                    "assess buoy locations", #63
                    "investigate buoy candidates", #64
                    "review buoy positions", #65
                    "study buoy area features", #66
                    "check buoy candidates", #67
                    "assess buoy features", #68
                    "inspect buoy candidates", #69
                    "examine buoy characteristics", #70
                    "document buoy candidates", #71
                    "analyze buoy characteristics", #72
                    "map buoy candidates", #73
                    "parse buoy candidates", #74
                    "evaluate buoy area", #75
                    "assess buoy area objects", #76
                    "investigate buoy area", #77
                    "review buoy area data", #78
                    "study buoy candidates", #79
                    "check buoy area", #80
                    "detail buoy positions", #81
                    "record buoy analysis", #82
                    "log buoy positions", #83
                    "examine each potential buoy object", #84
                    "parse and evaluate buoy candidates", #85
                    "analyze and map buoy positions", #86
                    "inspect and document buoy candidates", #87
                    "assess and record buoy positions", #88
                    "evaluate and analyze buoy area", #89
                    "map and analyze buoy candidates", #90
                    "examine and parse buoy data", #91
                    "detail and assess buoy candidates", #92
                    "inspect and evaluate buoy positions", #93
                    "review and analyze buoy candidates", #94
                    "study and document buoy area", #95
                    "check and assess buoy candidates", #96
                    "investigate and map buoy positions", #97
                    "document and evaluate buoy candidates", #98
                    "analyze and assess buoy area", #99
                    "parse and map buoy candidates" #100
        ],
        "test_variations": [
                    "execute comprehensive buoy area mapping protocol parsing all candidate objects with detailed feature analysis", #1
                    "conduct systematic evaluation of potential buoy candidates documenting positional data and characteristic parameters", #2
                    "perform detailed analysis of the buoy area examining each candidate object for confirmation of buoy designation", #3
                    "map all buoy candidates within the area sector parsing their characteristics and establishing reference coordinates", #4
                    "analyze the buoy location zone examining every potential candidate with rigorous classification procedures", #5
                    "execute buoy candidate assessment parsing all area objects and recording positional and characteristical data", #6
                    "conduct thorough analysis of potential buoy objects detailing positions and evaluating classification criteria", #7
                    "map the buoy candidates systematically analyzing the area with methodical examination of each object", #8
                    "perform detailed parsing of buoy area data evaluating all candidates and documenting their positions", #9
                    "assess all potential buoy candidates within the area mapping their locations with comprehensive feature evaluation" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('nip')}",
            "command": "map the area",
            "output": "Plan: 1. survey 2. map area"
        },
        "variations": [
                    "map the area", #1
                    "analyze the area", #2
                    "parse all points of interest", #3
                    "analyze the interesting points", #4
                    "parse each object of interest", #5
                    "examine every point closely", #6
                    "map the interesting points", #7
                    "analyze the surroundings", #8
                    "parse the region carefully", #9
                    "examine the region fully", #10
                    "map systematically", #11
                    "analyze systematically", #12
                    "parse the points of interest", #13
                    "analyze specific points", #14
                    "parse notable objects", #15
                    "examine identified points", #16
                    "map interesting points precisely", #17
                    "analyze surrounding details", #18
                    "parse with care", #19
                    "examine thoroughly", #20
                    "document every feature", #21
                    "record detailed findings", #22
                    "chart each point of interest", #23
                    "evaluate systematically", #24
                    "inspect with precision", #25
                    "note each object carefully", #26
                    "map with accuracy", #27
                    "analyze comprehensively", #28
                    "study each feature in depth", #29
                    "record observations in detail", #30
                    "track each identified point", #31
                    "assess methodically", #32
                    "examine with rigor", #33
                    "parse every listed item", #34
                    "analyze with structure", #35
                    "review all findings carefully", #36
                    "compile mapped points", #37
                    "verify every detail", #38
                    "break down observations", #39
                    "systematically log features", #40
                    "catalog the region", #41
                    "process the area data", #42
                    "dissect the findings", #43
                    "examine point by point", #44
                    "map the features", #45
                    "analyze methodically", #46
                    "parse the data carefully", #47
                    "document the analysis", #48
                    "record the mapping", #49
                    "evaluate the points", #50
                    "inspect the surroundings", #51
                    "note the observations", #52
                    "map the coordinates", #53
                    "analyze the layout", #54
                    "parse the structure", #55
                    "examine the details", #56
                    "document the details", #57
                    "record the analysis", #58
                    "chart the positions", #59
                    "evaluate the findings", #60
                    "inspect the details", #61
                    "note the findings", #62
                    "map the positions", #63
                    "analyze the structure", #64
                    "parse the observations", #65
                    "examine the findings", #66
                    "document the observations", #67
                    "record the findings", #68
                    "chart the area", #69
                    "evaluate the layout", #70
                    "inspect the area", #71
                    "note the layout", #72
                    "map the layout", #73
                    "analyze the observations", #74
                    "parse the area", #75
                    "examine the analysis", #76
                    "document the area", #77
                    "record the area", #78
                    "chart the observations", #79
                    "evaluate the structure", #80
                    "inspect the coordinates", #81
                    "note the structure", #82
                    "map the observations", #83
                    "analyze the data", #84
                    "parse the details", #85
                    "examine the coordinates", #86
                    "document the mapping", #87
                    "record the layout", #88
                    "chart the data", #89
                    "evaluate the analysis", #90
                    "inspect the observations", #91
                    "note the analysis", #92
                    "map the data", #93
                    "analyze the coordinates", #94
                    "parse the layout", #95
                    "examine the structure", #96
                    "document the coordinates", #97
                    "record the observations", #98
                    "chart the analysis", #99
                    "evaluate the observations" #100
        ],
        "test_variations": [
                    "execute comprehensive spatial analysis requiring point-by-point evaluation and detailed structural documentation", #1
                    "perform systematic area mapping with full data parsing and rigorous analysis of all identified features", #2
                    "conduct detailed region examination documenting each point of interest with methodical classification procedures", #3
                    "analyze the zone using structured protocols to map coordinates and parse all observable elements", #4
                    "execute thorough environmental analysis mapping features and parsing observations with complete documentation", #5
                    "perform detailed inspection parsing all points of interest with systematic evaluation and precise positioning", #6
                    "conduct comprehensive analysis examining the area structure mapping observations with rigorous data assessment", #7
                    "analyze systematically by parsing each feature recording coordinates with detailed structural examination protocols", #8
                    "execute detailed mapping procedures documenting observations parsing spatial data with comprehensive analysis standards", #9
                    "perform structured area analysis mapping all observations parsing features with methodical evaluation criteria" #10
        ]
    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "map the area",
            "output": "Plan: 1. survey 2. map area"
        },
        "variations": [
                    "map the buoy area", #1
                    "parse everything in the buoy area", #2
                    "analyze every object in the buoy area", #3
                    "analyze the buoy area", #4
                    "parse the buoy area", #5
                    "parse every potential buoy", #6
                    "analyze every potential buoy", #7
                    "scope every interesting point that could be a buoy", #8
                    "check each point that could be a buoy", #9
                    "parse everything that could be a buoy", #10
                    "analyze each object that may be a buoy", #11
                    "examine every potential buoy", #12
                    "examine each object that could be a buoy", #13
                    "parse every point of interest that is potentially a buoy", #14
                    "analyze the points that might be buoys", #15
                    "inspect every potential buoy", #16
                    "evaluate each potential buoy", #17
                    "assess every point that could be a buoy", #18
                    "investigate each potential buoy", #19
                    "review every buoy candidate", #20
                    "study each object that might be a buoy", #21
                    "check all potential buoys one by one", #22
                    "assess each buoy candidate thoroughly", #23
                    "inspect every object that could be a buoy", #24
                    "examine each point carefully for buoy features", #25
                    "chart all potential buoys precisely", #26
                    "evaluate objects that resemble buoys", #27
                    "study every object that might be buoy-like", #28
                    "log each potential buoy with detail", #29
                    "detail every buoy candidate clearly", #30
                    "assess all points of interest systematically", #31
                    "inspect each potential buoy thoroughly", #32
                    "evaluate points that might represent buoys", #33
                    "document each potential buoy", #34
                    "analyze and record each candidate buoy", #35
                    "map out all potential buoy locations", #36
                    "check each object for buoy characteristics", #37
                    "evaluate each potential buoy with rigor", #38
                    "assess potential buoy positions carefully", #39
                    "inspect all buoy candidates in detail", #40
                    "study and log each potential buoy", #41
                    "record every buoy candidate systematically", #42
                    "detail each object for buoy confirmation", #43
                    "review each buoy-like point carefully", #44
                    "document and analyze every potential buoy", #45
                    "parse all buoy-related data", #46
                    "analyze the buoy location data", #47
                    "map every buoy position", #48
                    "examine the buoy candidates closely", #49
                    "inspect and record potential buoys", #50
                    "assess buoy candidate characteristics", #51
                    "evaluate the buoy area in detail", #52
                    "study the potential buoy positions", #53
                    "check for buoy indicators", #54
                    "log buoy candidate positions", #55
                    "document the buoy analysis", #56
                    "parse buoy position data", #57
                    "analyze buoy candidate objects", #58
                    "map the buoy positions", #59
                    "examine buoy location candidates", #60
                    "inspect buoy area objects", #61
                    "evaluate buoy candidates", #62
                    "assess buoy locations", #63
                    "investigate buoy candidates", #64
                    "review buoy positions", #65
                    "study buoy area features", #66
                    "check buoy candidates", #67
                    "assess buoy features", #68
                    "inspect buoy candidates", #69
                    "examine buoy characteristics", #70
                    "document buoy candidates", #71
                    "analyze buoy characteristics", #72
                    "map buoy candidates", #73
                    "parse buoy candidates", #74
                    "evaluate buoy area", #75
                    "assess buoy area objects", #76
                    "investigate buoy area", #77
                    "review buoy area data", #78
                    "study buoy candidates", #79
                    "check buoy area", #80
                    "detail buoy positions", #81
                    "record buoy analysis", #82
                    "log buoy positions", #83
                    "examine each potential buoy object", #84
                    "parse and evaluate buoy candidates", #85
                    "analyze and map buoy positions", #86
                    "inspect and document buoy candidates", #87
                    "assess and record buoy positions", #88
                    "evaluate and analyze buoy area", #89
                    "map and analyze buoy candidates", #90
                    "examine and parse buoy data", #91
                    "detail and assess buoy candidates", #92
                    "inspect and evaluate buoy positions", #93
                    "review and analyze buoy candidates", #94
                    "study and document buoy area", #95
                    "check and assess buoy candidates", #96
                    "investigate and map buoy positions", #97
                    "document and evaluate buoy candidates", #98
                    "analyze and assess buoy area", #99
                    "parse and map buoy candidates" #100
        ],
        "test_variations": [
                    "execute comprehensive buoy area mapping protocol parsing all candidate objects with detailed feature analysis", #1
                    "conduct systematic evaluation of potential buoy candidates documenting positional data and characteristic parameters", #2
                    "perform detailed analysis of the buoy area examining each candidate object for confirmation of buoy designation", #3
                    "map all buoy candidates within the area sector parsing their characteristics and establishing reference coordinates", #4
                    "analyze the buoy location zone examining every potential candidate with rigorous classification procedures", #5
                    "execute buoy candidate assessment parsing all area objects and recording positional and characteristical data", #6
                    "conduct thorough analysis of potential buoy objects detailing positions and evaluating classification criteria", #7
                    "map the buoy candidates systematically analyzing the area with methodical examination of each object", #8
                    "perform detailed parsing of buoy area data evaluating all candidates and documenting their positions", #9
                    "assess all potential buoy candidates within the area mapping their locations with comprehensive feature evaluation" #10
        ]
    },
    {
        "base": {
            "context": f"{ctx_('y2_nip')}",
            "command": "map the area",
            "output": "Plan: 1. survey 2. map area"
        },
        "variations": [
                    "map the buoy area", #1
                    "parse everything in the buoy area", #2
                    "analyze every object in the buoy area", #3
                    "analyze the buoy area", #4
                    "parse the buoy area", #5
                    "parse every potential buoy", #6
                    "analyze every potential buoy", #7
                    "scope every interesting point that could be a buoy", #8
                    "check each point that could be a buoy", #9
                    "parse everything that could be a buoy", #10
                    "analyze each object that may be a buoy", #11
                    "examine every potential buoy", #12
                    "examine each object that could be a buoy", #13
                    "parse every point of interest that is potentially a buoy", #14
                    "analyze the points that might be buoys", #15
                    "inspect every potential buoy", #16
                    "evaluate each potential buoy", #17
                    "assess every point that could be a buoy", #18
                    "investigate each potential buoy", #19
                    "review every buoy candidate", #20
                    "study each object that might be a buoy", #21
                    "check all potential buoys one by one", #22
                    "assess each buoy candidate thoroughly", #23
                    "inspect every object that could be a buoy", #24
                    "examine each point carefully for buoy features", #25
                    "chart all potential buoys precisely", #26
                    "evaluate objects that resemble buoys", #27
                    "study every object that might be buoy-like", #28
                    "log each potential buoy with detail", #29
                    "detail every buoy candidate clearly", #30
                    "assess all points of interest systematically", #31
                    "inspect each potential buoy thoroughly", #32
                    "evaluate points that might represent buoys", #33
                    "document each potential buoy", #34
                    "analyze and record each candidate buoy", #35
                    "map out all potential buoy locations", #36
                    "check each object for buoy characteristics", #37
                    "evaluate each potential buoy with rigor", #38
                    "assess potential buoy positions carefully", #39
                    "inspect all buoy candidates in detail", #40
                    "study and log each potential buoy", #41
                    "record every buoy candidate systematically", #42
                    "detail each object for buoy confirmation", #43
                    "review each buoy-like point carefully", #44
                    "document and analyze every potential buoy", #45
                    "parse all buoy-related data", #46
                    "analyze the buoy location data", #47
                    "map every buoy position", #48
                    "examine the buoy candidates closely", #49
                    "inspect and record potential buoys", #50
                    "assess buoy candidate characteristics", #51
                    "evaluate the buoy area in detail", #52
                    "study the potential buoy positions", #53
                    "check for buoy indicators", #54
                    "log buoy candidate positions", #55
                    "document the buoy analysis", #56
                    "parse buoy position data", #57
                    "analyze buoy candidate objects", #58
                    "map the buoy positions", #59
                    "examine buoy location candidates", #60
                    "inspect buoy area objects", #61
                    "evaluate buoy candidates", #62
                    "assess buoy locations", #63
                    "investigate buoy candidates", #64
                    "review buoy positions", #65
                    "study buoy area features", #66
                    "check buoy candidates", #67
                    "assess buoy features", #68
                    "inspect buoy candidates", #69
                    "examine buoy characteristics", #70
                    "document buoy candidates", #71
                    "analyze buoy characteristics", #72
                    "map buoy candidates", #73
                    "parse buoy candidates", #74
                    "evaluate buoy area", #75
                    "assess buoy area objects", #76
                    "investigate buoy area", #77
                    "review buoy area data", #78
                    "study buoy candidates", #79
                    "check buoy area", #80
                    "detail buoy positions", #81
                    "record buoy analysis", #82
                    "log buoy positions", #83
                    "examine each potential buoy object", #84
                    "parse and evaluate buoy candidates", #85
                    "analyze and map buoy positions", #86
                    "inspect and document buoy candidates", #87
                    "assess and record buoy positions", #88
                    "evaluate and analyze buoy area", #89
                    "map and analyze buoy candidates", #90
                    "examine and parse buoy data", #91
                    "detail and assess buoy candidates", #92
                    "inspect and evaluate buoy positions", #93
                    "review and analyze buoy candidates", #94
                    "study and document buoy area", #95
                    "check and assess buoy candidates", #96
                    "investigate and map buoy positions", #97
                    "document and evaluate buoy candidates", #98
                    "analyze and assess buoy area", #99
                    "parse and map buoy candidates" #100
        ],
        "test_variations": [
                    "execute comprehensive buoy area mapping protocol parsing all candidate objects with detailed feature analysis", #1
                    "conduct systematic evaluation of potential buoy candidates documenting positional data and characteristic parameters", #2
                    "perform detailed analysis of the buoy area examining each candidate object for confirmation of buoy designation", #3
                    "map all buoy candidates within the area sector parsing their characteristics and establishing reference coordinates", #4
                    "analyze the buoy location zone examining every potential candidate with rigorous classification procedures", #5
                    "execute buoy candidate assessment parsing all area objects and recording positional and characteristical data", #6
                    "conduct thorough analysis of potential buoy objects detailing positions and evaluating classification criteria", #7
                    "map the buoy candidates systematically analyzing the area with methodical examination of each object", #8
                    "perform detailed parsing of buoy area data evaluating all candidates and documenting their positions", #9
                    "assess all potential buoy candidates within the area mapping their locations with comprehensive feature evaluation" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('pl0')}",
            "command": "find the assigned pipeline",
            "output": "Plan: 1. find given pipeline"
        },
        "variations": [
                    "find the assigned pipeline", #1
                    "look for the given pipeline", #2
                    "search for the pipeline with the given ID", #3
                    "find given pipeline", #4
                    "look for assigned pipeline", #5
                    "spot the pipeline with the given ID", #6
                    "identify the correct pipeline", #7
                    "track down the assigned pipeline", #8
                    "find the pipeline with the correct ID", #9
                    "look for the specified pipeline", #10
                    "search for the specified pipeline", #11
                    "search for the pipeline with the correct ID", #12
                    "find specified pipeline", #13
                    "look for pipeline with correct ID", #14
                    "spot the specified pipeline", #15
                    "identify the specified pipeline", #16
                    "track down the specified pipeline", #17
                    "find the pipeline with the specified ID", #18
                    "look for the pipeline with the specified ID", #19
                    "spot the pipeline with the correct ID", #20
                    "locate the assigned pipeline", #21
                    "locate the pipeline with the given ID", #22
                    "locate the specified pipeline", #23
                    "track the given pipeline", #24
                    "track the pipeline with the specified ID", #25
                    "identify the assigned pipeline", #26
                    "spot the correct pipeline", #27
                    "check for the pipeline with the given ID", #28
                    "check the assigned pipeline", #29
                    "verify the specified pipeline", #30
                    "verify the pipeline with the correct ID", #31
                    "pinpoint the assigned pipeline", #32
                    "pinpoint the pipeline with the given ID", #33
                    "search for the assigned pipeline", #34
                    "search for the pipeline with the assigned ID", #35
                    "look out for the specified pipeline", #36
                    "look out for the pipeline with the correct ID", #37
                    "detect the assigned pipeline", #38
                    "detect the pipeline with the given ID", #39
                    "confirm the specified pipeline", #40
                    "confirm the pipeline with the assigned ID", #41
                    "find and identify the assigned pipeline", #42
                    "find and track the pipeline with the given ID", #43
                    "spot and log the specified pipeline", #44
                    "track and locate the assigned pipeline", #45
                    "identify and mark the correct pipeline", #46
                    "locate and verify the given pipeline", #47
                    "detect and record the specified pipeline", #48
                    "confirm and track the assigned pipeline", #49
                    "pinpoint and identify the pipeline with the specified ID", #50
                    "look for and confirm the correct pipeline", #51
                    "spot and verify the pipeline with the assigned ID", #52
                    "track down and log the specified pipeline", #53
                    "find and confirm the pipeline with the correct ID", #54
                    "identify and track the given pipeline", #55
                    "locate and log the specified pipeline", #56
                    "detect and pinpoint the assigned pipeline", #57
                    "verify and identify the pipeline with the given ID", #58
                    "confirm and spot the specified pipeline", #59
                    "check and track the assigned pipeline", #60
                    "look for and pinpoint the pipeline with the specified ID", #61
                    "locate and detect the correct pipeline", #62
                    "search and identify the pipeline with the given ID", #63
                    "track and confirm the assigned pipeline", #64
                    "spot and log the correct pipeline", #65
                    "identify and verify the pipeline with the specified ID", #66
                    "pinpoint and track the assigned pipeline", #67
                    "detect and confirm the specified pipeline", #68
                    "check and locate the pipeline with the correct ID", #69
                    "find and log the assigned pipeline", #70
                    "look for and detect the given pipeline", #71
                    "search for the target pipeline", #72
                    "find the target pipeline", #73
                    "locate the target pipeline", #74
                    "identify the target pipeline", #75
                    "spot the target pipeline", #76
                    "detect the target pipeline", #77
                    "track the target pipeline", #78
                    "verify the target pipeline", #79
                    "confirm the target pipeline", #80
                    "pinpoint the target pipeline", #81
                    "find and locate the designated pipeline", #82
                    "search and verify the designated pipeline", #83
                    "identify and locate the designated pipeline", #84
                    "spot and confirm the designated pipeline", #85
                    "detect and track the designated pipeline", #86
                    "look for the designated pipeline", #87
                    "find the designated pipeline", #88
                    "locate the designated pipeline", #89
                    "identify the designated pipeline", #90
                    "spot the designated pipeline", #91
                    "track down the target pipeline", #92
                    "search for the target pipeline location", #93
                    "find the pipeline according to the given ID", #94
                    "locate the pipeline as specified", #95
                    "identify and log the assigned pipeline", #96
                    "detect and verify the target pipeline", #97
                    "confirm and locate the given pipeline", #98
                    "verify and track the assigned pipeline", #99
                    "pinpoint and log the designated pipeline" #100
        ],
        "test_variations": [
                    "locate and establish communication with the designated pipeline infrastructure asset", #1
                    "identify the pipeline system corresponding to the provided operational identification parameters", #2
                    "execute pipeline asset detection protocol locating the structure through assigned identifier confirmation", #3
                    "establish pipeline position and verify operational status according to provided system parameters", #4
                    "conduct pipeline facility reconnaissance acquiring location data and verifying assigned designation", #5
                    "identify and document the pipeline infrastructure matching the specified identification criteria", #6
                    "execute targeting procedures to locate the designated pipeline asset within operational zone", #7
                    "establish pipeline facility coordinates and confirm against provided identification parameters", #8
                    "locate the specified pipeline infrastructure and initiate status assessment procedures", #9
                    "identify the assigned pipeline asset and establish communication protocols with facility" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('pl1_nmp')}",
            "command": "",
            "output": "Plan: 1. check yellow pipe"
        },
        "variations": [
                    "find the red marker on the assigned pipeline", #1
                    "find the red marker on the damaged pipe", #2
                    "check the yellow pipe", #3
                    "find out which pipe of the pipeline is damaged", #4
                    "check both yellow pipes", #5
                    "find the red marker on the damaged pipe", #6
                    "control the yellow pipes", #7
                    "check the pipes of the pipeline", #8
                    "find the red marker on the yellow pipe", #9
                    "make a checkup of the pipeline pipes", #10
                    "find the damaged pipe of the pipeline", #11
                    "take a look at the yellow pipes", #12
                    "take a look at the pipes of the pipeline", #13
                    "look for the damaged pipe", #14
                    "look for the red marker on the given pipeline", #15
                    "find out on which pipe of the pipeline is the red marker", #16
                    "find the marker of the yellow pipe", #17
                    "spot the marker on the pipe of the pipeline", #18
                    "spot the marker on the pipeline pipe", #19
                    "control the pipes of the pipeline", #20
                    "inspect the red marker on the pipeline", #21
                    "inspect the red marker on the damaged pipe", #22
                    "review the yellow pipe", #23
                    "assess which pipe is damaged", #24
                    "inspect both yellow pipes", #25
                    "examine the red marker on the damaged pipe", #26
                    "check control of the yellow pipes", #27
                    "review the pipeline pipes", #28
                    "identify the red marker on the yellow pipe", #29
                    "examine all pipeline pipes", #30
                    "determine the damaged pipe in the pipeline", #31
                    "observe the yellow pipes", #32
                    "inspect all pipes of the pipeline", #33
                    "locate the damaged pipe", #34
                    "find the red marker location on the given pipeline", #35
                    "determine which pipe holds the red marker", #36
                    "identify the marker on the yellow pipe", #37
                    "spot the marker along the pipeline", #38
                    "verify the marker on the pipeline pipe", #39
                    "monitor the pipeline pipes", #40
                    "check for red markers on assigned pipelines", #41
                    "inspect which pipe is damaged", #42
                    "assess the yellow pipes", #43
                    "observe all pipeline pipes", #44
                    "find the marker on damaged pipes", #45
                    "review red marker positions on the pipeline", #46
                    "inspect yellow pipes for markers", #47
                    "check the pipeline for damaged sections", #48
                    "spot the marker on the correct pipe", #49
                    "identify damaged pipes in the pipeline", #50
                    "determine location of red marker on pipeline", #51
                    "examine yellow pipes for markers", #52
                    "inspect each pipe in the pipeline", #53
                    "locate damaged sections in the pipeline", #54
                    "monitor yellow pipes for issues", #55
                    "identify pipeline pipe with red marker", #56
                    "observe red marker on damaged pipeline", #57
                    "check marker positions on the pipeline", #58
                    "verify yellow pipes for markers", #59
                    "inspect and locate red markers on pipeline", #60
                    "find and review damaged pipeline pipes", #61
                    "assess pipeline pipes for markers", #62
                    "examine and identify the red marker", #63
                    "spot and verify damaged pipes", #64
                    "monitor pipeline for red markers", #65
                    "review each pipe in the pipeline", #66
                    "check yellow pipes for issues", #67
                    "inspect and document pipeline pipes", #68
                    "locate and examine red markers on pipes", #69
                    "track the red marker location", #70
                    "find and log the damaged pipe", #71
                    "assess the pipe damage severity", #72
                    "check the marker visibility", #73
                    "examine pipe condition systematically", #74
                    "document the yellow pipe location", #75
                    "identify the marker color and type", #76
                    "verify the pipe configuration", #77
                    "monitor the red marker status", #78
                    "inspect the pipe structural integrity", #79
                    "locate the damaged section precisely", #80
                    "assess yellow pipe functionality", #81
                    "check the marker alignment", #82
                    "examine the pipe connections", #83
                    "find the primary damage location", #84
                    "review the marker configuration", #85
                    "verify the pipeline system integrity", #86
                    "inspect the marked damage area", #87
                    "locate all yellow pipe segments", #88
                    "check the red marker distinctiveness", #89
                    "assess the pipeline operational status", #90
                    "examine the pipe material condition", #91
                    "identify secondary damage indicators", #92
                    "track all red markers in sequence", #93
                    "verify the marker placement accuracy", #94
                    "inspect the complete pipe assembly", #95
                    "locate the primary marker position", #96
                    "assess the damaged pipe extent", #97
                    "check all yellow pipe sections", #98
                    "examine the marker and damage correlation", #99
                    "identify the compromised pipeline section" #100
        ],
        "test_variations": [
                    "systematically examine the pipeline infrastructure identifying the red marker location on the designated yellow pipe segment", #1
                    "conduct damage assessment protocol locating the marked section while documenting pipe condition and marker status", #2
                    "inspect the pipeline structure verifying the red marker position and determining the extent of pipe damage", #3
                    "execute marker detection and pipe condition survey establishing the relationship between marker location and damage", #4
                    "assess the assigned pipeline identifying both marker placement and damaged pipe section with condition documentation", #5
                    "perform detailed inspection of pipeline components establishing red marker coordinates and pipe damage parameters", #6
                    "locate and verify the red marker on the damaged yellow pipe assessing structural integrity", #7
                    "conduct comprehensive pipeline assessment identifying the marked damage and documenting pipe operational status", #8
                    "execute inspection protocol to establish red marker position correlation with pipeline damage location", #9
                    "assess the pipeline system damage through marker identification and damaged pipe section documentation" #10
        ]


    },
    {
        "base":    {
            "context": f"{ctx_('pl1_mp_m')}",
            "command": "find the pipe of the pipeline with the red marker",
            "output": "Plan: 1. check yellow pipe"
        },
        "variations": [
                    "find the red marker on the assigned pipeline", #1
                    "find the red marker on the damaged pipe", #2
                    "check the yellow pipe", #3
                    "find out which pipe of the pipeline is damaged", #4
                    "check both yellow pipes", #5
                    "find the red marker on the damaged pipe", #6
                    "control the yellow pipes", #7
                    "check the pipes of the pipeline", #8
                    "find the red marker on the yellow pipe", #9
                    "make a checkup of the pipeline pipes", #10
                    "find the damaged pipe of the pipeline", #11
                    "take a look at the yellow pipes", #12
                    "take a look at the pipes of the pipeline", #13
                    "look for the damaged pipe", #14
                    "look for the red marker on the given pipeline", #15
                    "find out on which pipe of the pipeline is the red marker", #16
                    "find the marker of the yellow pipe", #17
                    "spot the marker on the pipe of the pipeline", #18
                    "spot the marker on the pipeline pipe", #19
                    "control the pipes of the pipeline", #20
                    "inspect the red marker on the pipeline", #21
                    "inspect the red marker on the damaged pipe", #22
                    "review the yellow pipe", #23
                    "assess which pipe is damaged", #24
                    "inspect both yellow pipes", #25
                    "examine the red marker on the damaged pipe", #26
                    "check control of the yellow pipes", #27
                    "review the pipeline pipes", #28
                    "identify the red marker on the yellow pipe", #29
                    "examine all pipeline pipes", #30
                    "determine the damaged pipe in the pipeline", #31
                    "observe the yellow pipes", #32
                    "inspect all pipes of the pipeline", #33
                    "locate the damaged pipe", #34
                    "find the red marker location on the given pipeline", #35
                    "determine which pipe holds the red marker", #36
                    "identify the marker on the yellow pipe", #37
                    "spot the marker along the pipeline", #38
                    "verify the marker on the pipeline pipe", #39
                    "monitor the pipeline pipes", #40
                    "check for red markers on assigned pipelines", #41
                    "inspect which pipe is damaged", #42
                    "assess the yellow pipes", #43
                    "observe all pipeline pipes", #44
                    "find the marker on damaged pipes", #45
                    "review red marker positions on the pipeline", #46
                    "inspect yellow pipes for markers", #47
                    "check the pipeline for damaged sections", #48
                    "spot the marker on the correct pipe", #49
                    "identify damaged pipes in the pipeline", #50
                    "determine location of red marker on pipeline", #51
                    "examine yellow pipes for markers", #52
                    "inspect each pipe in the pipeline", #53
                    "locate damaged sections in the pipeline", #54
                    "monitor yellow pipes for issues", #55
                    "identify pipeline pipe with red marker", #56
                    "observe red marker on damaged pipeline", #57
                    "check marker positions on the pipeline", #58
                    "verify yellow pipes for markers", #59
                    "inspect and locate red markers on pipeline", #60
                    "find and review damaged pipeline pipes", #61
                    "assess pipeline pipes for markers", #62
                    "examine and identify the red marker", #63
                    "spot and verify damaged pipes", #64
                    "monitor pipeline for red markers", #65
                    "review each pipe in the pipeline", #66
                    "check yellow pipes for issues", #67
                    "inspect and document pipeline pipes", #68
                    "locate and examine red markers on pipes", #69
                    "track the red marker location", #70
                    "find and log the damaged pipe", #71
                    "assess the pipe damage severity", #72
                    "check the marker visibility", #73
                    "examine pipe condition systematically", #74
                    "document the yellow pipe location", #75
                    "identify the marker color and type", #76
                    "verify the pipe configuration", #77
                    "monitor the red marker status", #78
                    "inspect the pipe structural integrity", #79
                    "locate the damaged section precisely", #80
                    "assess yellow pipe functionality", #81
                    "check the marker alignment", #82
                    "examine the pipe connections", #83
                    "find the primary damage location", #84
                    "review the marker configuration", #85
                    "verify the pipeline system integrity", #86
                    "inspect the marked damage area", #87
                    "locate all yellow pipe segments", #88
                    "check the red marker distinctiveness", #89
                    "assess the pipeline operational status", #90
                    "examine the pipe material condition", #91
                    "identify secondary damage indicators", #92
                    "track all red markers in sequence", #93
                    "verify the marker placement accuracy", #94
                    "inspect the complete pipe assembly", #95
                    "locate the primary marker position", #96
                    "assess the damaged pipe extent", #97
                    "check all yellow pipe sections", #98
                    "examine the marker and damage correlation", #99
                    "identify the compromised pipeline section" #100
        ],
        "test_variations": [
                    "systematically examine the pipeline infrastructure identifying the red marker location on the designated yellow pipe segment", #1
                    "conduct damage assessment protocol locating the marked section while documenting pipe condition and marker status", #2
                    "inspect the pipeline structure verifying the red marker position and determining the extent of pipe damage", #3
                    "execute marker detection and pipe condition survey establishing the relationship between marker location and damage", #4
                    "assess the assigned pipeline identifying both marker placement and damaged pipe section with condition documentation", #5
                    "perform detailed inspection of pipeline components establishing red marker coordinates and pipe damage parameters", #6
                    "locate and verify the red marker on the damaged yellow pipe assessing structural integrity", #7
                    "conduct comprehensive pipeline assessment identifying the marked damage and documenting pipe operational status", #8
                    "execute inspection protocol to establish red marker position correlation with pipeline damage location", #9
                    "assess the pipeline system damage through marker identification and damaged pipe section documentation" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('npl_nmp')}",
            "command": "find the pipe with the red marker",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "variations": [
                    "find the red marker on the assigned pipeline", #1
                    "find the red marker on the damaged pipe", #2
                    "check the yellow pipe", #3
                    "find out which pipe of the pipeline is damaged", #4
                    "check both yellow pipes", #5
                    "find the red marker on the damaged pipe", #6
                    "control the yellow pipes", #7
                    "check the pipes of the pipeline", #8
                    "find the red marker on the yellow pipe", #9
                    "make a checkup of the pipeline pipes", #10
                    "find the damaged pipe of the pipeline", #11
                    "take a look at the yellow pipes", #12
                    "take a look at the pipes of the pipeline", #13
                    "look for the damaged pipe", #14
                    "look for the red marker on the given pipeline", #15
                    "find out on which pipe of the pipeline is the red marker", #16
                    "find the marker of the yellow pipe", #17
                    "spot the marker on the pipe of the pipeline", #18
                    "spot the marker on the pipeline pipe", #19
                    "control the pipes of the pipeline", #20
                    "inspect the red marker on the pipeline", #21
                    "inspect the red marker on the damaged pipe", #22
                    "review the yellow pipe", #23
                    "assess which pipe is damaged", #24
                    "inspect both yellow pipes", #25
                    "examine the red marker on the damaged pipe", #26
                    "check control of the yellow pipes", #27
                    "review the pipeline pipes", #28
                    "identify the red marker on the yellow pipe", #29
                    "examine all pipeline pipes", #30
                    "determine the damaged pipe in the pipeline", #31
                    "observe the yellow pipes", #32
                    "inspect all pipes of the pipeline", #33
                    "locate the damaged pipe", #34
                    "find the red marker location on the given pipeline", #35
                    "determine which pipe holds the red marker", #36
                    "identify the marker on the yellow pipe", #37
                    "spot the marker along the pipeline", #38
                    "verify the marker on the pipeline pipe", #39
                    "monitor the pipeline pipes", #40
                    "check for red markers on assigned pipelines", #41
                    "inspect which pipe is damaged", #42
                    "assess the yellow pipes", #43
                    "observe all pipeline pipes", #44
                    "find the marker on damaged pipes", #45
                    "review red marker positions on the pipeline", #46
                    "inspect yellow pipes for markers", #47
                    "check the pipeline for damaged sections", #48
                    "spot the marker on the correct pipe", #49
                    "identify damaged pipes in the pipeline", #50
                    "determine location of red marker on pipeline", #51
                    "examine yellow pipes for markers", #52
                    "inspect each pipe in the pipeline", #53
                    "locate damaged sections in the pipeline", #54
                    "monitor yellow pipes for issues", #55
                    "identify pipeline pipe with red marker", #56
                    "observe red marker on damaged pipeline", #57
                    "check marker positions on the pipeline", #58
                    "verify yellow pipes for markers", #59
                    "inspect and locate red markers on pipeline", #60
                    "find and review damaged pipeline pipes", #61
                    "assess pipeline pipes for markers", #62
                    "examine and identify the red marker", #63
                    "spot and verify damaged pipes", #64
                    "monitor pipeline for red markers", #65
                    "review each pipe in the pipeline", #66
                    "check yellow pipes for issues", #67
                    "inspect and document pipeline pipes", #68
                    "locate and examine red markers on pipes", #69
                    "track the red marker location", #70
                    "find and log the damaged pipe", #71
                    "assess the pipe damage severity", #72
                    "check the marker visibility", #73
                    "examine pipe condition systematically", #74
                    "document the yellow pipe location", #75
                    "identify the marker color and type", #76
                    "verify the pipe configuration", #77
                    "monitor the red marker status", #78
                    "inspect the pipe structural integrity", #79
                    "locate the damaged section precisely", #80
                    "assess yellow pipe functionality", #81
                    "check the marker alignment", #82
                    "examine the pipe connections", #83
                    "find the primary damage location", #84
                    "review the marker configuration", #85
                    "verify the pipeline system integrity", #86
                    "inspect the marked damage area", #87
                    "locate all yellow pipe segments", #88
                    "check the red marker distinctiveness", #89
                    "assess the pipeline operational status", #90
                    "examine the pipe material condition", #91
                    "identify secondary damage indicators", #92
                    "track all red markers in sequence", #93
                    "verify the marker placement accuracy", #94
                    "inspect the complete pipe assembly", #95
                    "locate the primary marker position", #96
                    "assess the damaged pipe extent", #97
                    "check all yellow pipe sections", #98
                    "examine the marker and damage correlation", #99
                    "identify the compromised pipeline section" #100
        ],
        "test_variations": [
                    "systematically examine the pipeline infrastructure identifying the red marker location on the designated yellow pipe segment", #1
                    "conduct damage assessment protocol locating the marked section while documenting pipe condition and marker status", #2
                    "inspect the pipeline structure verifying the red marker position and determining the extent of pipe damage", #3
                    "execute marker detection and pipe condition survey establishing the relationship between marker location and damage", #4
                    "assess the assigned pipeline identifying both marker placement and damaged pipe section with condition documentation", #5
                    "perform detailed inspection of pipeline components establishing red marker coordinates and pipe damage parameters", #6
                    "locate and verify the red marker on the damaged yellow pipe assessing structural integrity", #7
                    "conduct comprehensive pipeline assessment identifying the marked damage and documenting pipe operational status", #8
                    "execute inspection protocol to establish red marker position correlation with pipeline damage location", #9
                    "assess the pipeline system damage through marker identification and damaged pipe section documentation" #10
        ]
    },
    {
        "base":    {
            "context": f"{ctx_('npl_mp_m')}",
            "command": "find the pipe with the red marker",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "variations": [
                    "find the red marker on the assigned pipeline", #1
                    "find the red marker on the damaged pipe", #2
                    "check the yellow pipe", #3
                    "find out which pipe of the pipeline is damaged", #4
                    "check both yellow pipes", #5
                    "find the red marker on the damaged pipe", #6
                    "control the yellow pipes", #7
                    "check the pipes of the pipeline", #8
                    "find the red marker on the yellow pipe", #9
                    "make a checkup of the pipeline pipes", #10
                    "find the damaged pipe of the pipeline", #11
                    "take a look at the yellow pipes", #12
                    "take a look at the pipes of the pipeline", #13
                    "look for the damaged pipe", #14
                    "look for the red marker on the given pipeline", #15
                    "find out on which pipe of the pipeline is the red marker", #16
                    "find the marker of the yellow pipe", #17
                    "spot the marker on the pipe of the pipeline", #18
                    "spot the marker on the pipeline pipe", #19
                    "control the pipes of the pipeline", #20
                    "inspect the red marker on the pipeline", #21
                    "inspect the red marker on the damaged pipe", #22
                    "review the yellow pipe", #23
                    "assess which pipe is damaged", #24
                    "inspect both yellow pipes", #25
                    "examine the red marker on the damaged pipe", #26
                    "check control of the yellow pipes", #27
                    "review the pipeline pipes", #28
                    "identify the red marker on the yellow pipe", #29
                    "examine all pipeline pipes", #30
                    "determine the damaged pipe in the pipeline", #31
                    "observe the yellow pipes", #32
                    "inspect all pipes of the pipeline", #33
                    "locate the damaged pipe", #34
                    "find the red marker location on the given pipeline", #35
                    "determine which pipe holds the red marker", #36
                    "identify the marker on the yellow pipe", #37
                    "spot the marker along the pipeline", #38
                    "verify the marker on the pipeline pipe", #39
                    "monitor the pipeline pipes", #40
                    "check for red markers on assigned pipelines", #41
                    "inspect which pipe is damaged", #42
                    "assess the yellow pipes", #43
                    "observe all pipeline pipes", #44
                    "find the marker on damaged pipes", #45
                    "review red marker positions on the pipeline", #46
                    "inspect yellow pipes for markers", #47
                    "check the pipeline for damaged sections", #48
                    "spot the marker on the correct pipe", #49
                    "identify damaged pipes in the pipeline", #50
                    "determine location of red marker on pipeline", #51
                    "examine yellow pipes for markers", #52
                    "inspect each pipe in the pipeline", #53
                    "locate damaged sections in the pipeline", #54
                    "monitor yellow pipes for issues", #55
                    "identify pipeline pipe with red marker", #56
                    "observe red marker on damaged pipeline", #57
                    "check marker positions on the pipeline", #58
                    "verify yellow pipes for markers", #59
                    "inspect and locate red markers on pipeline", #60
                    "find and review damaged pipeline pipes", #61
                    "assess pipeline pipes for markers", #62
                    "examine and identify the red marker", #63
                    "spot and verify damaged pipes", #64
                    "monitor pipeline for red markers", #65
                    "review each pipe in the pipeline", #66
                    "check yellow pipes for issues", #67
                    "inspect and document pipeline pipes", #68
                    "locate and examine red markers on pipes", #69
                    "track the red marker location", #70
                    "find and log the damaged pipe", #71
                    "assess the pipe damage severity", #72
                    "check the marker visibility", #73
                    "examine pipe condition systematically", #74
                    "document the yellow pipe location", #75
                    "identify the marker color and type", #76
                    "verify the pipe configuration", #77
                    "monitor the red marker status", #78
                    "inspect the pipe structural integrity", #79
                    "locate the damaged section precisely", #80
                    "assess yellow pipe functionality", #81
                    "check the marker alignment", #82
                    "examine the pipe connections", #83
                    "find the primary damage location", #84
                    "review the marker configuration", #85
                    "verify the pipeline system integrity", #86
                    "inspect the marked damage area", #87
                    "locate all yellow pipe segments", #88
                    "check the red marker distinctiveness", #89
                    "assess the pipeline operational status", #90
                    "examine the pipe material condition", #91
                    "identify secondary damage indicators", #92
                    "track all red markers in sequence", #93
                    "verify the marker placement accuracy", #94
                    "inspect the complete pipe assembly", #95
                    "locate the primary marker position", #96
                    "assess the damaged pipe extent", #97
                    "check all yellow pipe sections", #98
                    "examine the marker and damage correlation", #99
                    "identify the compromised pipeline section" #100
        ],
        "test_variations": [
                    "systematically examine the pipeline infrastructure identifying the red marker location on the designated yellow pipe segment", #1
                    "conduct damage assessment protocol locating the marked section while documenting pipe condition and marker status", #2
                    "inspect the pipeline structure verifying the red marker position and determining the extent of pipe damage", #3
                    "execute marker detection and pipe condition survey establishing the relationship between marker location and damage", #4
                    "assess the assigned pipeline identifying both marker placement and damaged pipe section with condition documentation", #5
                    "perform detailed inspection of pipeline components establishing red marker coordinates and pipe damage parameters", #6
                    "locate and verify the red marker on the damaged yellow pipe assessing structural integrity", #7
                    "conduct comprehensive pipeline assessment identifying the marked damage and documenting pipe operational status", #8
                    "execute inspection protocol to establish red marker position correlation with pipeline damage location", #9
                    "assess the pipeline system damage through marker identification and damaged pipe section documentation" #10
        ]
    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "check the main pipe",
            "output": "Plan: 1. check main pipe"
        },
        "variations": [
                    "check the main pipe", #1
                    "inspect the main pipe", #2
                    "examine the main pipe", #3
                    "look for the markers on the main pipe", #4
                    "detect markers on the main pipe", #5
                    "find markers on the main pipe", #6
                    "analyze the main pipe", #7
                    "scan the main pipe", #8
                    "find colored markers on the main pipe", #9
                    "look for colored markers on the main pipe", #10
                    "check main pipe", #11
                    "inspect main pipe", #12
                    "examine main pipe", #13
                    "look for markers on main pipe", #14
                    "detect markers on main pipe", #15
                    "find markers on main pipe", #16
                    "analyze main pipe", #17
                    "scan main pipe", #18
                    "find colored markers on main pipe", #19
                    "look for colored markers on main pipe", #20
                    "follow the main pipe and look for markers", #21
                    "find red and green markers on the main pipe", #22
                    "find a red marker on the main pipe", #23
                    "find a green marker on the main pipe", #24
                    "inspect the main pipe for markers", #25
                    "check the main pipe for colored markers", #26
                    "examine main pipe for markers", #27
                    "detect colored markers along the main pipe", #28
                    "locate markers on the main pipe", #29
                    "identify markers on main pipe", #30
                    "analyze the main pipe for markers", #31
                    "scan main pipe for markers", #32
                    "observe colored markers on the main pipe", #33
                    "track markers along the main pipe", #34
                    "look for red markers on the main pipe", #35
                    "look for green markers on the main pipe", #36
                    "follow the main pipe and detect markers", #37
                    "inspect main pipe for red and green markers", #38
                    "find marker locations on the main pipe", #39
                    "check main pipe for red markers", #40
                    "check main pipe for green markers", #41
                    "scan for markers along the main pipe", #42
                    "detect red markers on the main pipe", #43
                    "detect green markers on the main pipe", #44
                    "locate red and green markers on the main pipe", #45
                    "observe red markers on main pipe", #46
                    "observe green markers on main pipe", #47
                    "identify the red marker on the main pipe", #48
                    "identify the green marker on the main pipe", #49
                    "track red markers along the main pipe", #50
                    "track green markers along the main pipe", #51
                    "find and mark red markers on the main pipe", #52
                    "find and mark green markers on the main pipe", #53
                    "follow main pipe and log markers", #54
                    "inspect main pipe for all markers", #55
                    "scan the main pipe for red and green markers", #56
                    "locate all colored markers on main pipe", #57
                    "detect all markers on main pipe", #58
                    "observe all markers on the main pipe", #59
                    "track all markers along the main pipe", #60
                    "identify all markers on the main pipe", #61
                    "check main pipe thoroughly for markers", #62
                    "examine main pipe thoroughly for markers", #63
                    "find all marker positions on the main pipe", #64
                    "mark all colored markers on main pipe", #65
                    "inspect and locate red markers on main pipe", #66
                    "inspect and locate green markers on main pipe", #67
                    "observe and log markers on the main pipe", #68
                    "detect and record red markers on main pipe", #69
                    "detect and record green markers on main pipe", #70
                    "track and document all markers on the main pipe", #71
                    "identify and mark all colored markers on main pipe", #72
                    "scan thoroughly for markers on the main pipe", #73
                    "follow and inspect all markers on main pipe", #74
                    "verify the main pipe for markers", #75
                    "check the main pipe structure for markers", #76
                    "examine the main pipe configuration", #77
                    "look along the main pipe for markers", #78
                    "detect and locate all markers on main pipe", #79
                    "find every marker on the main pipe", #80
                    "inspect the entire main pipe", #81
                    "scan the complete main pipe", #82
                    "analyze marker positions on main pipe", #83
                    "observe the main pipe condition", #84
                    "track marker distribution on main pipe", #85
                    "identify marker patterns on main pipe", #86
                    "verify marker placement on main pipe", #87
                    "assess the main pipe for markers", #88
                    "document markers on the main pipe", #89
                    "record marker locations on main pipe", #90
                    "establish marker coordinates on main pipe", #91
                    "map marker positions on the main pipe", #92
                    "catalog markers on main pipe", #93
                    "classify markers on the main pipe", #94
                    "confirm marker presence on main pipe", #95
                    "validate marker locations on main pipe", #96
                    "audit the main pipe for markers", #97
                    "review markers along the main pipe", #98
                    "assess marker visibility on main pipe", #99
                    "determine marker significance on the main pipe" #100
        ],
        "test_variations": [
                    "conduct systematic inspection of the main pipe structure documenting all red and green marker locations with positional coordinates", #1
                    "execute marker detection protocol along the main pipe perimeter identifying and recording colored markers by type and position", #2
                    "perform comprehensive analysis of the main pipe including full marker inventory assessment and color classification", #3
                    "inspect the main pipeline system tracking and documenting all visible markers establishing their operational significance", #4
                    "analyze the complete main pipe configuration identifying marker patterns and recording detection data systematically", #5
                    "execute detailed marker survey along the main pipe structure with complete documentation of color types and positions", #6
                    "conduct thorough examination of the main pipe detecting all red and green markers and establishing coordinate data", #7
                    "perform methodical inspection protocol scanning the main pipe for all markers with detailed positional documentation", #8
                    "analyze the main pipe infrastructure identifying marker distributions and recording comprehensive marker assessment data", #9
                    "execute complete marker audit of the main pipe system documenting all colored markers with operational parameters" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('plmf_nmp')}",
            "command": "check the main pipe",
            "output": "Plan: 1. check main pipe"
        },
    "variations": [
                "check the main pipe", #1
                "inspect the main pipe", #2
                "examine the main pipe", #3
                "look for the markers on the main pipe", #4
                "detect markers on the main pipe", #5
                "find markers on the main pipe", #6
                "analyze the main pipe", #7
                "scan the main pipe", #8
                "find colored markers on the main pipe", #9
                "look for colored markers on the main pipe", #10
                "check main pipe", #11
                "inspect main pipe", #12
                "examine main pipe", #13
                "look for markers on main pipe", #14
                "detect markers on main pipe", #15
                "find markers on main pipe", #16
                "analyze main pipe", #17
                "scan main pipe", #18
                "find colored markers on main pipe", #19
                "look for colored markers on main pipe", #20
                "follow the main pipe and look for markers", #21
                "find red and green markers on the main pipe", #22
                "find a red marker on the main pipe", #23
                "find a green marker on the main pipe", #24
                "inspect the main pipe for markers", #25
                "check the main pipe for colored markers", #26
                "examine main pipe for markers", #27
                "detect colored markers along the main pipe", #28
                "locate markers on the main pipe", #29
                "identify markers on main pipe", #30
                "analyze the main pipe for markers", #31
                "scan main pipe for markers", #32
                "observe colored markers on the main pipe", #33
                "track markers along the main pipe", #34
                "look for red markers on the main pipe", #35
                "look for green markers on the main pipe", #36
                "follow the main pipe and detect markers", #37
                "inspect main pipe for red and green markers", #38
                "find marker locations on the main pipe", #39
                "check main pipe for red markers", #40
                "check main pipe for green markers", #41
                "scan for markers along the main pipe", #42
                "detect red markers on the main pipe", #43
                "detect green markers on the main pipe", #44
                "locate red and green markers on the main pipe", #45
                "observe red markers on main pipe", #46
                "observe green markers on main pipe", #47
                "identify the red marker on the main pipe", #48
                "identify the green marker on the main pipe", #49
                "track red markers along the main pipe", #50
                "track green markers along the main pipe", #51
                "find and mark red markers on the main pipe", #52
                "find and mark green markers on the main pipe", #53
                "follow main pipe and log markers", #54
                "inspect main pipe for all markers", #55
                "scan the main pipe for red and green markers", #56
                "locate all colored markers on main pipe", #57
                "detect all markers on main pipe", #58
                "observe all markers on the main pipe", #59
                "track all markers along the main pipe", #60
                "identify all markers on the main pipe", #61
                "check main pipe thoroughly for markers", #62
                "examine main pipe thoroughly for markers", #63
                "find all marker positions on the main pipe", #64
                "mark all colored markers on main pipe", #65
                "inspect and locate red markers on main pipe", #66
                "inspect and locate green markers on main pipe", #67
                "observe and log markers on the main pipe", #68
                "detect and record red markers on main pipe", #69
                "detect and record green markers on main pipe", #70
                "track and document all markers on the main pipe", #71
                "identify and mark all colored markers on main pipe", #72
                "scan thoroughly for markers on the main pipe", #73
                "follow and inspect all markers on main pipe", #74
                "verify the main pipe for markers", #75
                "check the main pipe structure for markers", #76
                "examine the main pipe configuration", #77
                "look along the main pipe for markers", #78
                "detect and locate all markers on main pipe", #79
                "find every marker on the main pipe", #80
                "inspect the entire main pipe", #81
                "scan the complete main pipe", #82
                "analyze marker positions on main pipe", #83
                "observe the main pipe condition", #84
                "track marker distribution on main pipe", #85
                "identify marker patterns on main pipe", #86
                "verify marker placement on main pipe", #87
                "assess the main pipe for markers", #88
                "document markers on the main pipe", #89
                "record marker locations on main pipe", #90
                "establish marker coordinates on main pipe", #91
                "map marker positions on the main pipe", #92
                "catalog markers on main pipe", #93
                "classify markers on the main pipe", #94
                "confirm marker presence on main pipe", #95
                "validate marker locations on main pipe", #96
                "audit the main pipe for markers", #97
                "review markers along the main pipe", #98
                "assess marker visibility on main pipe", #99
                "determine marker significance on the main pipe" #100
    ],
    "test_variations": [
                "conduct systematic inspection of the main pipe structure documenting all red and green marker locations with positional coordinates", #1
                "execute marker detection protocol along the main pipe perimeter identifying and recording colored markers by type and position", #2
                "perform comprehensive analysis of the main pipe including full marker inventory assessment and color classification", #3
                "inspect the main pipeline system tracking and documenting all visible markers establishing their operational significance", #4
                "analyze the complete main pipe configuration identifying marker patterns and recording detection data systematically", #5
                "execute detailed marker survey along the main pipe structure with complete documentation of color types and positions", #6
                "conduct thorough examination of the main pipe detecting all red and green markers and establishing coordinate data", #7
                "perform methodical inspection protocol scanning the main pipe for all markers with detailed positional documentation", #8
                "analyze the main pipe infrastructure identifying marker distributions and recording comprehensive marker assessment data", #9
                "execute complete marker audit of the main pipe system documenting all colored markers with operational parameters" #10
    ]


    },
    {
        "base": {
            "context": f"{ctx_('vnc')}",
            "command": "close the valve",
            "output": "Plan: 1. close valve"
        },
        "variations": [
                    "close the valve", #1
                    "shut the valve", #2
                    "turn off the valve", #3
                    "seal the valve", #4
                    "stop the valve", #5
                    "shut off the valve", #6
                    "turn the valve off", #7
                    "seal off the valve", #8
                    "stop flow with the valve", #9
                    "close off the valve", #10
                    "close valve", #11
                    "shut valve", #12
                    "turn off valve", #13
                    "seal valve", #14
                    "stop valve", #15
                    "shut off valve", #16
                    "turn valve off", #17
                    "seal off valve", #18
                    "stop flow with valve", #19
                    "close off valve", #20
                    "turn the valve to off", #21
                    "lock the valve closed", #22
                    "engage the valve shut", #23
                    "disable the valve", #24
                    "block the valve", #25
                    "cut off flow with the valve", #26
                    "turn valve to closed position", #27
                    "seal the valve completely", #28
                    "stop valve flow", #29
                    "close the valve fully", #30
                    "shut the valve completely", #31
                    "turn valve off fully", #32
                    "seal valve completely", #33
                    "stop the valve flow", #34
                    "engage valve closure", #35
                    "turn off the valve entirely", #36
                    "shut off the valve completely", #37
                    "lock valve off", #38
                    "block flow using the valve", #39
                    "engage valve fully", #40
                    "close valve completely", #41
                    "shut valve fully", #42
                    "turn valve fully off", #43
                    "seal off valve completely", #44
                    "stop flow fully with the valve", #45
                    "lock the valve", #46
                    "disable valve flow", #47
                    "engage full valve closure", #48
                    "block valve flow completely", #49
                    "turn valve completely off", #50
                    "shut off flow with valve", #51
                    "seal the valve tightly", #52
                    "stop all flow with valve", #53
                    "close and seal valve", #54
                    "shut and lock valve", #55
                    "turn off and seal valve", #56
                    "disable and close valve", #57
                    "engage and seal the valve", #58
                    "cut off flow completely with valve", #59
                    "lock and turn off valve", #60
                    "seal and stop valve", #61
                    "close valve and block flow", #62
                    "shut valve to block flow", #63
                    "turn off valve fully", #64
                    "engage valve lock", #65
                    "seal and lock valve", #66
                    "stop and close valve", #67
                    "turn valve to locked position", #68
                    "shut off flow entirely with valve", #69
                    "block and seal the valve", #70
                    "deactivate the valve", #71
                    "shut down the valve", #72
                    "power down the valve", #73
                    "isolate the valve", #74
                    "restrict valve flow", #75
                    "minimize valve operation", #76
                    "neutralize the valve", #77
                    "put valve in standby", #78
                    "position valve closed", #79
                    "manipulate valve to closed state", #80
                    "execute valve closure", #81
                    "perform valve shutdown", #82
                    "initiate valve closing", #83
                    "activate valve closure mechanism", #84
                    "complete valve sealing process", #85
                    "finalize valve closure", #86
                    "ensure valve is closed", #87
                    "verify valve closure", #88
                    "confirm valve shutdown", #89
                    "establish valve lock", #90
                    "maintain valve closed position", #91
                    "secure valve in closed state", #92
                    "hold valve closed", #93
                    "keep valve shut", #94
                    "maintain valve seal", #95
                    "prevent valve opening", #96
                    "restrict valve operation", #97
                    "limit valve function", #98
                    "control valve closure", #99
                    "regulate valve to off state" #100
        ],
        "test_variations": [
                    "execute valve closure protocol ensuring complete flow isolation with mechanical lock engagement", #1
                    "shut down the valve system establishing total blockage of the specified flow path", #2
                    "close the valve and verify sealed position with operational status confirmation", #3
                    "engage valve shutdown procedures restricting all flow through the designated valve", #4
                    "activate valve closure mechanisms achieving complete flow interruption and system isolation", #5
                    "perform valve shutdown sequence establishing locked closed position and confirming flow cessation", #6
                    "close and seal the valve preventing any residual flow through the system", #7
                    "execute valve control protocol terminating flow by complete mechanical valve closure", #8
                    "shut off the valve and establish permanent sealed condition for operational integrity", #9
                    "perform immediate valve deactivation and lock engagement establishing complete flow isolation" #10
        ]



    },
    {
        "base": {
            "context": f"{ctx_('pl0')}",
            "command": "close the valve",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. close valve"
        },
        "variations": [
                    "close the valve", #1
                    "shut the valve", #2
                    "turn off the valve", #3
                    "seal the valve", #4
                    "stop the valve", #5
                    "shut off the valve", #6
                    "turn the valve off", #7
                    "seal off the valve", #8
                    "stop flow with the valve", #9
                    "close off the valve", #10
                    "close valve", #11
                    "shut valve", #12
                    "turn off valve", #13
                    "seal valve", #14
                    "stop valve", #15
                    "shut off valve", #16
                    "turn valve off", #17
                    "seal off valve", #18
                    "stop flow with valve", #19
                    "close off valve", #20
                    "turn the valve to off", #21
                    "lock the valve closed", #22
                    "engage the valve shut", #23
                    "disable the valve", #24
                    "block the valve", #25
                    "cut off flow with the valve", #26
                    "turn valve to closed position", #27
                    "seal the valve completely", #28
                    "stop valve flow", #29
                    "close the valve fully", #30
                    "shut the valve completely", #31
                    "turn valve off fully", #32
                    "seal valve completely", #33
                    "stop the valve flow", #34
                    "engage valve closure", #35
                    "turn off the valve entirely", #36
                    "shut off the valve completely", #37
                    "lock valve off", #38
                    "block flow using the valve", #39
                    "engage valve fully", #40
                    "close valve completely", #41
                    "shut valve fully", #42
                    "turn valve fully off", #43
                    "seal off valve completely", #44
                    "stop flow fully with the valve", #45
                    "lock the valve", #46
                    "disable valve flow", #47
                    "engage full valve closure", #48
                    "block valve flow completely", #49
                    "turn valve completely off", #50
                    "shut off flow with valve", #51
                    "seal the valve tightly", #52
                    "stop all flow with valve", #53
                    "close and seal valve", #54
                    "shut and lock valve", #55
                    "turn off and seal valve", #56
                    "disable and close valve", #57
                    "engage and seal the valve", #58
                    "cut off flow completely with valve", #59
                    "lock and turn off valve", #60
                    "seal and stop valve", #61
                    "close valve and block flow", #62
                    "shut valve to block flow", #63
                    "turn off valve fully", #64
                    "engage valve lock", #65
                    "seal and lock valve", #66
                    "stop and close valve", #67
                    "turn valve to locked position", #68
                    "shut off flow entirely with valve", #69
                    "block and seal the valve", #70
                    "deactivate the valve", #71
                    "shut down the valve", #72
                    "power down the valve", #73
                    "isolate the valve", #74
                    "restrict valve flow", #75
                    "minimize valve operation", #76
                    "neutralize the valve", #77
                    "put valve in standby", #78
                    "position valve closed", #79
                    "manipulate valve to closed state", #80
                    "execute valve closure", #81
                    "perform valve shutdown", #82
                    "initiate valve closing", #83
                    "activate valve closure mechanism", #84
                    "complete valve sealing process", #85
                    "finalize valve closure", #86
                    "ensure valve is closed", #87
                    "verify valve closure", #88
                    "confirm valve shutdown", #89
                    "establish valve lock", #90
                    "maintain valve closed position", #91
                    "secure valve in closed state", #92
                    "hold valve closed", #93
                    "keep valve shut", #94
                    "maintain valve seal", #95
                    "prevent valve opening", #96
                    "restrict valve operation", #97
                    "limit valve function", #98
                    "control valve closure", #99
                    "regulate valve to off state" #100
        ],
        "test_variations": [
                    "execute valve closure protocol ensuring complete flow isolation with mechanical lock engagement", #1
                    "shut down the valve system establishing total blockage of the specified flow path", #2
                    "close the valve and verify sealed position with operational status confirmation", #3
                    "engage valve shutdown procedures restricting all flow through the designated valve", #4
                    "activate valve closure mechanisms achieving complete flow interruption and system isolation", #5
                    "perform valve shutdown sequence establishing locked closed position and confirming flow cessation", #6
                    "close and seal the valve preventing any residual flow through the system", #7
                    "execute valve control protocol terminating flow by complete mechanical valve closure", #8
                    "shut off the valve and establish permanent sealed condition for operational integrity", #9
                    "perform immediate valve deactivation and lock engagement establishing complete flow isolation" #10
        ]


    },
        {
        "base": {
            "context": f"{ctx_('ru')}",
            "command": "grab the ring",
            "output": "Plan: 1. catch ring"
        },
        "variations": [
                    "grab the ring", #1
                    "take the ring", #2
                    "seize the ring", #3
                    "capture the ring", #4
                    "collect the ring", #5
                    "pick up the ring", #6
                    "snatch the ring", #7
                    "get the ring", #8
                    "acquire the ring", #9
                    "obtain the ring", #10
                    "grab ring", #11
                    "take ring", #12
                    "seize ring", #13
                    "capture ring", #14
                    "collect ring", #15
                    "pick up ring", #16
                    "snatch ring", #17
                    "get ring", #18
                    "acquire ring", #19
                    "obtain ring", #20
                    "secure the ring", #21
                    "retrieve the ring", #22
                    "fetch the ring", #23
                    "claim the ring", #24
                    "lift the ring", #25
                    "grasp the ring", #26
                    "hold the ring", #27
                    "take hold of the ring", #28
                    "pick the ring", #29
                    "snag the ring", #30
                    "capture the ring carefully", #31
                    "collect the ring carefully", #32
                    "grab and secure the ring", #33
                    "take and hold the ring", #34
                    "seize and collect the ring", #35
                    "acquire and hold the ring", #36
                    "retrieve and secure the ring", #37
                    "fetch and acquire the ring", #38
                    "claim and pick up the ring", #39
                    "grasp and hold the ring", #40
                    "snatch and secure the ring", #41
                    "pick up and hold the ring", #42
                    "get and keep the ring", #43
                    "obtain and secure the ring", #44
                    "take possession of the ring", #45
                    "collect and grab the ring", #46
                    "retrieve and pick the ring", #47
                    "grab and acquire the ring", #48
                    "hold and secure the ring", #49
                    "capture and claim the ring", #50
                    "fetch and hold the ring", #51
                    "lift and take the ring", #52
                    "snag and hold the ring", #53
                    "secure and collect the ring", #54
                    "claim and hold the ring", #55
                    "get and grasp the ring", #56
                    "acquire and keep the ring", #57
                    "obtain and grasp the ring", #58
                    "pick and secure the ring", #59
                    "take and retrieve the ring", #60
                    "grab and claim the ring", #61
                    "fetch and secure the ring", #62
                    "lift and acquire the ring", #63
                    "hold and obtain the ring", #64
                    "grasp and pick up the ring", #65
                    "collect and retrieve the ring", #66
                    "snatch and obtain the ring", #67
                    "secure and claim the ring", #68
                    "get and retrieve the ring", #69
                    "take and claim the ring", #70
                    "seize and acquire the ring", #71
                    "capture and obtain the ring", #72
                    "collect and hold the ring", #73
                    "pick up and secure the ring", #74
                    "snatch and claim the ring", #75
                    "grasp and retrieve the ring", #76
                    "lift and secure the ring", #77
                    "claim and secure the ring", #78
                    "fetch and claim the ring", #79
                    "obtain and collect the ring", #80
                    "secure and retrieve the ring", #81
                    "grasp and acquire the ring", #82
                    "snag and secure the ring", #83
                    "take and secure the ring", #84
                    "grab and hold the ring", #85
                    "seize and hold the ring", #86
                    "capture and hold the ring", #87
                    "collect and claim the ring", #88
                    "pick up and claim the ring", #89
                    "snatch and hold the ring", #90
                    "get and secure the ring", #91
                    "acquire and claim the ring", #92
                    "obtain and hold the ring", #93
                    "retrieve and claim the ring", #94
                    "fetch and grab the ring", #95
                    "lift and claim the ring", #96
                    "hold and claim the ring", #97
                    "grasp and claim the ring", #98
                    "take full possession of the ring", #99
                    "successfully claim the ring" #100
        ],
        "test_variations": [
                    "locate and physically secure the ring object ensuring proper containment and custody establishment", #1
                    "execute ring acquisition protocol with careful handling and possession verification procedures", #2
                    "retrieve the ring object establishing operational custody with appropriate safeguarding measures", #3
                    "seize and secure the ring target completing the acquisition objective with retention confirmation", #4
                    "obtain the designated ring through systematic collection procedures with status documentation", #5
                    "capture and hold the ring ensuring continued possession and control throughout the operation", #6
                    "claim the ring target with proper acquisition documentation and custody transfer confirmation", #7
                    "secure the ring object preventing loss or compromise during the collection phase", #8
                    "acquire the ring completing the target collection objective with verified possession status", #9
                    "obtain full operational control of the ring through successful acquisition and containment procedures" #10
        ]
    },
            {
        "base": {
            "context": f"{ctx_('pl0')}",
            "command": "grab the ring",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. catch ring"
        },
        "variations": [
                    "grab the ring", #1
                    "take the ring", #2
                    "seize the ring", #3
                    "capture the ring", #4
                    "collect the ring", #5
                    "pick up the ring", #6
                    "snatch the ring", #7
                    "get the ring", #8
                    "acquire the ring", #9
                    "obtain the ring", #10
                    "grab ring", #11
                    "take ring", #12
                    "seize ring", #13
                    "capture ring", #14
                    "collect ring", #15
                    "pick up ring", #16
                    "snatch ring", #17
                    "get ring", #18
                    "acquire ring", #19
                    "obtain ring", #20
                    "secure the ring", #21
                    "retrieve the ring", #22
                    "fetch the ring", #23
                    "claim the ring", #24
                    "lift the ring", #25
                    "grasp the ring", #26
                    "hold the ring", #27
                    "take hold of the ring", #28
                    "pick the ring", #29
                    "snag the ring", #30
                    "capture the ring carefully", #31
                    "collect the ring carefully", #32
                    "grab and secure the ring", #33
                    "take and hold the ring", #34
                    "seize and collect the ring", #35
                    "acquire and hold the ring", #36
                    "retrieve and secure the ring", #37
                    "fetch and acquire the ring", #38
                    "claim and pick up the ring", #39
                    "grasp and hold the ring", #40
                    "snatch and secure the ring", #41
                    "pick up and hold the ring", #42
                    "get and keep the ring", #43
                    "obtain and secure the ring", #44
                    "take possession of the ring", #45
                    "collect and grab the ring", #46
                    "retrieve and pick the ring", #47
                    "grab and acquire the ring", #48
                    "hold and secure the ring", #49
                    "capture and claim the ring", #50
                    "fetch and hold the ring", #51
                    "lift and take the ring", #52
                    "snag and hold the ring", #53
                    "secure and collect the ring", #54
                    "claim and hold the ring", #55
                    "get and grasp the ring", #56
                    "acquire and keep the ring", #57
                    "obtain and grasp the ring", #58
                    "pick and secure the ring", #59
                    "take and retrieve the ring", #60
                    "grab and claim the ring", #61
                    "fetch and secure the ring", #62
                    "lift and acquire the ring", #63
                    "hold and obtain the ring", #64
                    "grasp and pick up the ring", #65
                    "collect and retrieve the ring", #66
                    "snatch and obtain the ring", #67
                    "secure and claim the ring", #68
                    "get and retrieve the ring", #69
                    "take and claim the ring", #70
                    "seize and acquire the ring", #71
                    "capture and obtain the ring", #72
                    "collect and hold the ring", #73
                    "pick up and secure the ring", #74
                    "snatch and claim the ring", #75
                    "grasp and retrieve the ring", #76
                    "lift and secure the ring", #77
                    "claim and secure the ring", #78
                    "fetch and claim the ring", #79
                    "obtain and collect the ring", #80
                    "secure and retrieve the ring", #81
                    "grasp and acquire the ring", #82
                    "snag and secure the ring", #83
                    "take and secure the ring", #84
                    "grab and hold the ring", #85
                    "seize and hold the ring", #86
                    "capture and hold the ring", #87
                    "collect and claim the ring", #88
                    "pick up and claim the ring", #89
                    "snatch and hold the ring", #90
                    "get and secure the ring", #91
                    "acquire and claim the ring", #92
                    "obtain and hold the ring", #93
                    "retrieve and claim the ring", #94
                    "fetch and grab the ring", #95
                    "lift and claim the ring", #96
                    "hold and claim the ring", #97
                    "grasp and claim the ring", #98
                    "take full possession of the ring", #99
                    "successfully claim the ring" #100
        ],
        "test_variations": [
                    "locate and physically secure the ring object ensuring proper containment and custody establishment", #1
                    "execute ring acquisition protocol with careful handling and possession verification procedures", #2
                    "retrieve the ring object establishing operational custody with appropriate safeguarding measures", #3
                    "seize and secure the ring target completing the acquisition objective with retention confirmation", #4
                    "obtain the designated ring through systematic collection procedures with status documentation", #5
                    "capture and hold the ring ensuring continued possession and control throughout the operation", #6
                    "claim the ring target with proper acquisition documentation and custody transfer confirmation", #7
                    "secure the ring object preventing loss or compromise during the collection phase", #8
                    "acquire the ring completing the target collection objective with verified possession status", #9
                    "obtain full operational control of the ring through successful acquisition and containment procedures" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('rg')}",
            "command": "surface with the ring",
            "output": "Plan: 1. surface"
        },
        "variations": [
                    "surface with the ring", #1
                    "ascend with the ring", #2
                    "rise to surface with the ring", #3
                    "go up with the ring", #4
                    "move to surface with the ring", #5
                    "come to surface with the ring", #6
                    "head to surface with the ring", #7
                    "travel to surface with the ring", #8
                    "get to surface with the ring", #9
                    "make it to surface with the ring", #10
                    "surface with ring", #11
                    "ascend with ring", #12
                    "rise to surface with ring", #13
                    "go up with ring", #14
                    "move to surface with ring", #15
                    "come to surface with ring", #16
                    "head to surface with ring", #17
                    "travel to surface with ring", #18
                    "get to surface with ring", #19
                    "make it to surface with ring", #20
                    "rise and bring the ring up", #21
                    "ascend carrying the ring", #22
                    "move upwards with the ring", #23
                    "go to the surface holding the ring", #24
                    "head upward with the ring", #25
                    "travel upwards with the ring", #26
                    "bring the ring to the surface", #27
                    "get to surface while holding the ring", #28
                    "make your way to surface with the ring", #29
                    "ascend safely with the ring", #30
                    "reach the surface with the ring", #31
                    "surface while carrying the ring", #32
                    "rise to top with the ring", #33
                    "head to top with the ring", #34
                    "travel to top with the ring", #35
                    "bring ring to surface", #36
                    "get to the surface with ring", #37
                    "make it to the top with the ring", #38
                    "ascend and deliver the ring to surface", #39
                    "move to the surface bringing the ring", #40
                    "come up with the ring", #41
                    "rise and secure the ring at surface", #42
                    "ascend and hold the ring", #43
                    "head upward and carry the ring", #44
                    "travel upward with the ring", #45
                    "bring the ring safely to surface", #46
                    "get to top with the ring", #47
                    "make it up with the ring", #48
                    "surface and hold the ring", #49
                    "ascend and transport the ring", #50
                    "rise while holding the ring", #51
                    "move to surface safely with ring", #52
                    "come to top with the ring", #53
                    "head to surface safely with ring", #54
                    "travel to surface safely with ring", #55
                    "get to surface safely with ring", #56
                    "make it to top safely with ring", #57
                    "ascend carefully with the ring", #58
                    "bring the ring up safely", #59
                    "reach surface holding the ring", #60
                    "move to top carrying the ring", #61
                    "come up safely with the ring", #62
                    "head to top safely with the ring", #63
                    "travel upwards safely with the ring", #64
                    "get to surface securely with the ring", #65
                    "make it to surface securely with the ring", #66
                    "ascend steadily with the ring", #67
                    "surface steadily with the ring", #68
                    "rise steadily with the ring", #69
                    "bring ring steadily to surface", #70
                    "move to top steadily with the ring", #71
                    "return to surface with the ring", #72
                    "bring the ring back to surface", #73
                    "come back up with the ring", #74
                    "escort the ring to surface", #75
                    "deliver the ring to surface", #76
                    "transport the ring to surface", #77
                    "convey the ring upward", #78
                    "carry the ring to the top", #79
                    "bring the ring to the top", #80
                    "exit with the ring", #81
                    "proceed to surface with the ring", #82
                    "advance to surface with the ring", #83
                    "progress to surface with the ring", #84
                    "move upward with the ring", #85
                    "go upward with the ring", #86
                    "travel upward with the ring", #87
                    "head upward to surface with the ring", #88
                    "reach the top with the ring", #89
                    "get to the top with the ring", #90
                    "make it to the top carrying the ring", #91
                    "ascend toward surface with the ring", #92
                    "rise toward surface with the ring", #93
                    "move toward surface with the ring", #94
                    "travel toward surface with the ring", #95
                    "head toward surface with the ring", #96
                    "go toward surface with the ring", #97
                    "proceed upward with the ring", #98
                    "advance upward with the ring", #99
                    "conclude ascent with the ring at surface" #100
        ],
        "test_variations": [
                    "establish ascending trajectory maintaining positive buoyancy while securing the ring during vertical transit to surface", #1
                    "execute controlled ascent protocol transporting the ring to surface operations zone with buoyancy management", #2
                    "initiate surfacing procedures with the ring ensuring steady vertical movement to atmospheric interface", #3
                    "conduct upward transit while maintaining secure custody of the ring throughout the ascension sequence", #4
                    "perform regulated rise to surface altitude while safely conveying the ring to completion zone", #5
                    "execute final ascent phase bringing the ring to surface position with operational integrity confirmation", #6
                    "manage buoyancy control during surfacing transit maintaining continuous ring possession and containment", #7
                    "conduct systematic elevation establishing surface arrival with the ring and mission completion verification", #8
                    "perform vertical transit with ring custody establishing surface recovery and objective completion", #9
                    "execute final surface approach transporting the ring to designated recovery zone with operation closure procedures" #10
        ]



    },
        {
        "base": {
            "context": f"{ctx_('ru')}",
            "command": "surface with the ring",
            "output": "Plan: 1. catch ring 2. surface"
        },
        "variations": [
                    "surface with the ring", #1
                    "ascend with the ring", #2
                    "rise to surface with the ring", #3
                    "go up with the ring", #4
                    "move to surface with the ring", #5
                    "come to surface with the ring", #6
                    "head to surface with the ring", #7
                    "travel to surface with the ring", #8
                    "get to surface with the ring", #9
                    "make it to surface with the ring", #10
                    "surface with ring", #11
                    "ascend with ring", #12
                    "rise to surface with ring", #13
                    "go up with ring", #14
                    "move to surface with ring", #15
                    "come to surface with ring", #16
                    "head to surface with ring", #17
                    "travel to surface with ring", #18
                    "get to surface with ring", #19
                    "make it to surface with ring", #20
                    "rise and bring the ring up", #21
                    "ascend carrying the ring", #22
                    "move upwards with the ring", #23
                    "go to the surface holding the ring", #24
                    "head upward with the ring", #25
                    "travel upwards with the ring", #26
                    "bring the ring to the surface", #27
                    "get to surface while holding the ring", #28
                    "make your way to surface with the ring", #29
                    "ascend safely with the ring", #30
                    "reach the surface with the ring", #31
                    "surface while carrying the ring", #32
                    "rise to top with the ring", #33
                    "head to top with the ring", #34
                    "travel to top with the ring", #35
                    "bring ring to surface", #36
                    "get to the surface with ring", #37
                    "make it to the top with the ring", #38
                    "ascend and deliver the ring to surface", #39
                    "move to the surface bringing the ring", #40
                    "come up with the ring", #41
                    "rise and secure the ring at surface", #42
                    "ascend and hold the ring", #43
                    "head upward and carry the ring", #44
                    "travel upward with the ring", #45
                    "bring the ring safely to surface", #46
                    "get to top with the ring", #47
                    "make it up with the ring", #48
                    "surface and hold the ring", #49
                    "ascend and transport the ring", #50
                    "rise while holding the ring", #51
                    "move to surface safely with ring", #52
                    "come to top with the ring", #53
                    "head to surface safely with ring", #54
                    "travel to surface safely with ring", #55
                    "get to surface safely with ring", #56
                    "make it to top safely with ring", #57
                    "ascend carefully with the ring", #58
                    "bring the ring up safely", #59
                    "reach surface holding the ring", #60
                    "move to top carrying the ring", #61
                    "come up safely with the ring", #62
                    "head to top safely with the ring", #63
                    "travel upwards safely with the ring", #64
                    "get to surface securely with the ring", #65
                    "make it to surface securely with the ring", #66
                    "ascend steadily with the ring", #67
                    "surface steadily with the ring", #68
                    "rise steadily with the ring", #69
                    "bring ring steadily to surface", #70
                    "move to top steadily with the ring", #71
                    "return to surface with the ring", #72
                    "bring the ring back to surface", #73
                    "come back up with the ring", #74
                    "escort the ring to surface", #75
                    "deliver the ring to surface", #76
                    "transport the ring to surface", #77
                    "convey the ring upward", #78
                    "carry the ring to the top", #79
                    "bring the ring to the top", #80
                    "exit with the ring", #81
                    "proceed to surface with the ring", #82
                    "advance to surface with the ring", #83
                    "progress to surface with the ring", #84
                    "move upward with the ring", #85
                    "go upward with the ring", #86
                    "travel upward with the ring", #87
                    "head upward to surface with the ring", #88
                    "reach the top with the ring", #89
                    "get to the top with the ring", #90
                    "make it to the top carrying the ring", #91
                    "ascend toward surface with the ring", #92
                    "rise toward surface with the ring", #93
                    "move toward surface with the ring", #94
                    "travel toward surface with the ring", #95
                    "head toward surface with the ring", #96
                    "go toward surface with the ring", #97
                    "proceed upward with the ring", #98
                    "advance upward with the ring", #99
                    "conclude ascent with the ring at surface" #100
        ],
        "test_variations": [
                    "establish ascending trajectory maintaining positive buoyancy while securing the ring during vertical transit to surface", #1
                    "execute controlled ascent protocol transporting the ring to surface operations zone with buoyancy management", #2
                    "initiate surfacing procedures with the ring ensuring steady vertical movement to atmospheric interface", #3
                    "conduct upward transit while maintaining secure custody of the ring throughout the ascension sequence", #4
                    "perform regulated rise to surface altitude while safely conveying the ring to completion zone", #5
                    "execute final ascent phase bringing the ring to surface position with operational integrity confirmation", #6
                    "manage buoyancy control during surfacing transit maintaining continuous ring possession and containment", #7
                    "conduct systematic elevation establishing surface arrival with the ring and mission completion verification", #8
                    "perform vertical transit with ring custody establishing surface recovery and objective completion", #9
                    "execute final surface approach transporting the ring to designated recovery zone with operation closure procedures" #10
        ]

    },
        {
            "base":{
            "context": f"{ctx_('default')}",
            "command": "clear the memory",
            "output": "Plan: 1. clear memory"
        },
        "variations":[
            "clear memory",
            "reset memory",
            "wipe memory",
            "tabula rasa",
            "forget everything",
            "clear all stored knowledge",
            "reset stored data",
            "erase all knowledge",
            "delete all stored information",
            "wipe all data",
            "delete all information",
            "delete everything",
            "reset",
            "restart",
            "start over"
        ]
    },


    # # COMBINATIONS ############################################################################################
    # {
    #     "base": {
    #         "context": f"{ctx_('nwp_y0')}",
    #         "command": "move to the waypoint and find the gate",
    #         "output": "Plan: 1. acquire target 2. move to waypoint 3. find yellow buoy 4. find yellow buoy"
    #     },
    #     "variations":[
    #         "go to the goal and find gate",
    #         "move to destination, then look for the gate",
    #         "navigate to waypoint and find gate buoys",
    #         "go and find gate",
    #         "move to goal and spot gate",
    #         "move to target and find gate",
    #         "go to the goal, spot the gate",
    #         "reach the target waypoint, then look for gate",
    #         "move to goal and find gate buoys",
    #         "go to waypoint, then spot gate",
    #         "navigate to destination and find gate",
    #         "go to goal and find gate",
    #         "move to destination, then look for the gate",
    #         "navigate to waypoint and find gate buoys",
    #         "go and find gate",
    #         "move to goal and spot gate",
    #         "move to target and find gate",
    #         "go to the goal, spot the gate",
    #         "reach the target waypoint, then look for gate",
    #         "move to goal and find gate buoys",
    #         "go to waypoint, then spot gate",

    #         "head to the goal and locate the gate",
    #         "navigate to the target and identify gate buoys",
    #         "move to waypoint, then locate the gate",
    #         "go to destination and spot the gate",
    #         "reach goal and identify gate buoys",
    #         "move to the target waypoint, then find gate",
    #         "proceed to goal and find gate",
    #         "navigate to goal and spot gate buoys",
    #         "go toward the target and locate the gate",
    #         "move to destination and find gate buoys",
    #         "head to waypoint and spot the gate",
    #         "reach goal and find gate",
    #         "navigate to destination and spot the gate",
    #         "move to the waypoint and identify gate buoys",
    #         "go to the goal and locate gate buoys",
    #         "proceed to the target and spot the gate",
    #         "reach the goal and identify gate",
    #         "move toward the goal and find gate",
    #         "navigate to waypoint and spot gate",
    #         "head to destination and locate gate",
    #         "proceed to waypoint and find gate buoys",
    #         "go to goal, then identify gate",
    #         "move to target and spot gate buoys",
    #         "reach waypoint and locate gate",
    #         "head to the goal and find gate buoys",
    #         "navigate toward target and spot gate",
    #         "move to goal and locate the gate",
    #         "go to waypoint and identify gate buoys",
    #         "proceed to goal and spot gate",
    #         "reach target and find gate buoys",
    #         "navigate to destination and locate gate"
    #     ]


    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('y2_pl0')}",
    #         "command": "pass through the gate and find the assigned pipeline",
    #         "output": "Plan: 1. pass through 2. find given pipeline"
    #     },
    #     "variations": [
    #         "cross the gate and find the given pipeline",
    #         "pass through and look for the pipeline with the right ID",
    #         "go through the gate, then find the assigned pipeline",
    #         "cross the gate, spot the right pipeline",
    #         "pass through gate, then look for given pipeline",
    #         "pass the gate, find correct ID pipeline",
    #         "go through gate and look for assigned pipeline",
    #         "make it through gate, spot given pipeline",
    #         "cross gate, then spot assigned pipeline",
    #         "navigate through the gate and locate the assigned pipeline",
    #         "go through gate and identify the pipeline with the correct ID",
    #         "pass through and detect the given pipeline",
    #         "cross the gate, then pinpoint the correct pipeline",
    #         "move through gate and find the assigned pipeline",
    #         "go past the gate and spot the right pipeline",
    #         "traverse the gate and identify the pipeline with the given ID",
    #         "cross gate and find the specified pipeline",
    #         "make it through the gate, then locate the pipeline",
    #         "proceed through the gate and find the pipeline with correct ID",
    #         "pass gate and locate the pipeline with the assigned ID",
    #         "move through gate and spot the specified pipeline",
    #         "traverse gate, then find the pipeline with correct ID",
    #         "go through the gate and locate the specified pipeline",
    #         "navigate gate, then spot the assigned pipeline",
    #         "move past gate and identify the correct pipeline",
    #         "cross through the gate and detect the assigned pipeline",
    #         "pass through and locate the pipeline with correct ID",
    #         "traverse the gate and pinpoint the assigned pipeline",
    #         "proceed through the gate, then identify the given pipeline",
    #         "cross the gate and locate the pipeline with assigned ID",
    #         "pass through and detect the specified pipeline",
    #         "move through and identify the pipeline with the right ID",
    #         "navigate through the gate and spot the assigned pipeline"
    #         ]


    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('y2_pl0_nip')}",
    #         "command": "pass through the gate and find the assigned pipeline",
    #         "output": "Plan: 1. pass through 2. find given pipeline"
    #     },
    #     "variations": [
    #         "pass between gate buoys, find given pipeline",
    #         "pass between gate buoys, find given pipeline",
    #         "navigate gate buoys and identify the given pipeline",
    #         "cross gate buoys and detect the assigned pipeline",
    #         "move through gate buoys and spot the pipeline",
    #         "navigate through the gate buoys and find the correct pipeline",
    #         "go through gate buoys, then spot the assigned pipeline",
    #         "traverse gate buoys, locate the correct pipeline",
    #         "cross the gate buoys and find the given pipeline"
    #     ]
    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('y2_pl0')}",
    #         "command": "pass through the gate and find the red marker on the pipeline",
    #         "output": "Plan: 1. pass through 2. find given pipeline 3. check yellow pipe"
    #     },
    #     "variations":[
    #         "cross the gate and find the red marker on the pipeline",
    #         "pass through and look for the damaged pipe",
    #         "go through the gate, then check the pipes of the pipeline",
    #         "cross gate, check yellow pipe",
    #         "go through gate, check pipes of pipeline",
    #         "pass through the gate, then look for damage",
    #         "pass through, find damage on pipeline",
    #         "cross the gate and control the pipes of the pipelines",
    #         "pass through and check the yellow pipes",
    #         "go through the gate, then check the pipes of the pipeline",
    #         "cross gate, find marker on pipeline",
    #         "go through gate, check pipeline pipe",
    #         "move through gate and spot the damage",
    #         "pass through the gate, then control the yellow pipe",
    #         "pass through, make a checkup of the yellow pipe",
    #         "navigate through the gate while locating the red marker on the pipeline",
    #         "upon passing the gate, identify the damaged pipe",
    #         "go through the gate, inspecting each pipe of the pipeline",
    #         "cross the gate and simultaneously spot the red marker on the pipeline",
    #         "after moving through the gate, determine which pipe holds the marker",
    #         "traverse the gate and examine the yellow pipe carefully",
    #         "go through the gate, observing all pipeline pipes",
    #         "while moving through the gate, inspect for damage",
    #         "after crossing the gate, find damage on the pipeline",
    #         "upon passing the gate, control the pipeline pipes",
    #         "move through the gate, checking the condition of the yellow pipes",
    #         "after going through the gate, inspect all pipeline pipes",
    #         "cross the gate while pinpointing the marker on the pipeline",
    #         "upon moving through the gate, identify the pipe with the marker",
    #         "traverse the gate, locating the marker on the pipeline",
    #         "while going through the gate, examine the pipeline pipe",
    #         "after crossing the gate, spot any damage present",
    #         "on moving through the gate, control the yellow pipe",
    #         "after passing the gate, make a full checkup of the yellow pipe",
    #         "navigate through the gate, noting positions of the red marker",
    #         "upon crossing the gate, assess the damaged pipeline sections",
    #         "while passing through the gate, verify the yellow pipes",
    #         "on moving through the gate, locate the marker on the pipeline",
    #         "after crossing the gate, examine each pipeline pipe carefully",
    #         "traverse the gate while spotting damaged sections of the pipeline",
    #         "upon moving through the gate, identify marker locations",
    #         "while passing the gate, check for pipeline damage",
    #         "after moving through the gate, inspect red marker positions",
    #         "on traversing the gate, assess overall pipeline condition",
    #         "while crossing the gate, evaluate the yellow pipe",
    #         "upon passing through, examine all pipeline pipes thoroughly",
    #         "after moving through the gate, identify damaged pipes",
    #         "while traversing the gate, locate the yellow pipe marker",
    #         "upon crossing the gate, track the red marker along the pipeline",
    #         "on moving through the gate, inspect for pipeline issues",
    #         "while passing through the gate, check the yellow pipe's condition",
    #         "after crossing the gate, control the pipeline sections",
    #         "upon traversing the gate, inspect each pipeline pipe",
    #         "while crossing the gate, verify the red marker position",
    #         "after passing through the gate, identify any pipe damage",
    #         "on moving through the gate, locate the damaged pipe",
    #         "while traversing the gate, check the marker positions",
    #         "after crossing the gate, inspect the pipeline thoroughly",
    #         "upon passing the gate, assess the red marker's location",
    #         "while moving through, examine the pipeline carefully",
    #         "after traversing the gate, monitor the yellow pipe",
    #         "upon passing through, record the red marker on the pipeline"
    #     ]


    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('y2_pl0')}",
    #         "command": "pass through the gate and find the red marker on the pipeline",
    #         "output": "Plan: 1. pass through 2. find given pipeline 3. check yellow pipe"
    #     },
    #     "variations":[
    #         "pass between gate buoys, locating the red marker along the pipeline",
    #         "navigate through gate buoys and check the yellow pipe",
    #         "cross the gate buoys and look for markers on the right ID pipeline",
    #         "move between the yellow buoys, then examine the yellow pipe"
    #     ]
    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('pl0_nip')}",
    #         "command": "find the red marker and perform a survey",
    #         "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. survey"
    #     },
    #     "variations": [
    #         "find the red marker on the assigned pipeline, then perform a survey",
    #         "find the red marker on th damaged pipe and survey the area",
    #         "check the yellow pipe, explore the area",
    #         "find out which pipe of the pipeline is damaged, survey",
    #         "check both yellow pipes, look for interesting points",
    #         "find the red marker on the damaged pipe, then spot potential buoys",
    #         "control the yellow pipes, find points of interest",
    #         "check the pipes of the pipeline and explore the surroundings",
    #         "find the red marker on the yellow pipe, perform a survey",
    #         "make a checkup of the pipeline pipes, find some potential buoys",
    #         "find the damaged pipe of the pipeline, survey the surroundings",
    #         "take a look a the yellow pipes and explore the area",
    #         "take a look at the pipes of the pipeline, then survey here",
    #         "look for the damaged pipe and for potential buoys",
    #         "look for the red marker on the given pipeline, after that perform a survey",
    #         "find out on which pipe of the pipeline is the red marker, then get some interesting points",
    #         "find the marker of the yellow pipe and some potential buoys",
    #         "spot the marker on the pipe of the pipeline, then explore the buoy area",
    #         "spot the marker on the pipeline pipe, survey the buoy area",
    #         "control the pipes of the pipeline and survey the buoy area",

    #         "inspect the red marker on the assigned pipeline and explore the area",
    #         "check the damaged pipe, then look for interesting points",
    #         "review the yellow pipe and spot potential buoys",
    #         "identify the damaged pipeline section, then survey the surroundings",
    #         "examine both yellow pipes and explore potential points",
    #         "locate the red marker on the damaged pipe, then find potential buoys",
    #         "control and inspect the yellow pipes, explore the area",
    #         "check the pipeline pipes thoroughly and survey the surroundings",
    #         "spot the red marker on the yellow pipe and explore potential points",
    #         "perform a checkup of the pipeline pipes, then find potential buoys",
    #         "identify damaged pipes and survey the surroundings",
    #         "inspect the yellow pipes, then explore the area",
    #         "review the pipeline pipes and perform a local survey",
    #         "look for the damaged pipe, then locate potential buoys",
    #         "find the red marker on the pipeline, afterwards survey the area",
    #         "determine which pipe has the red marker, then identify interesting points",
    #         "spot the yellow pipe marker, then locate potential buoys",
    #         "locate the marker on the pipeline pipe, then explore the buoy area",
    #         "identify the pipeline pipe marker and survey the buoy area",
    #         "control the pipeline pipes and explore the buoy area",
    #         "inspect red markers on the assigned pipeline, then explore interesting points",
    #         "check yellow pipes and survey for potential buoys",
    #         "review pipeline pipes and locate points of interest",
    #         "identify damaged pipe sections, then survey the area",
    #         "spot red markers on damaged pipes and locate potential buoys",
    #         "inspect the yellow pipes and explore the surroundings",
    #         "check all pipeline pipes and find potential buoys",
    #         "find the damaged pipeline section and survey the area",
    #         "examine yellow pipes, then spot interesting points",
    #         "locate red markers on the pipeline and explore the area",
    #         "check the assigned pipeline, then find potential buoys",
    #         "spot markers on the pipeline pipes, then survey the surroundings",
    #         "control the pipeline, then explore potential buoy locations",
    #         "inspect each pipeline pipe and find interesting points",
    #         "review damaged pipes and survey the surroundings",
    #         "identify yellow pipes and explore potential points",
    #         "locate red markers on the pipeline and survey the area",
    #         "check yellow pipe markers and explore the buoy area",
    #         "inspect the pipeline pipes, then find potential buoys",
    #         "spot damaged pipes and survey the surroundings",
    #         "review the assigned pipeline for markers, then explore interesting points",
    #         "control the yellow pipes, then locate potential buoys",
    #         "inspect markers on the pipeline and survey the area",
    #         "find red markers on the assigned pipeline, then explore potential points",
    #         "check damaged pipeline pipes, then locate potential buoys",
    #         "spot the yellow pipe markers, then survey the surroundings",
    #         "inspect each pipeline pipe marker and explore the area",
    #         "review the pipeline and locate potential buoy points",
    #         "check red markers on the pipeline, then explore the area"
    #     ]

    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('pl1_nip')}",
    #         "command": "find the red marker and perform a survey",
    #         "output": "Plan: 1. check yellow pipe 2. survey"
    #     },
    #     "variations": [
    #         "find the red marker on the assigned pipeline, then perform a survey",
    #         "find the red marker on the damaged pipe and survey the area",
    #         "check the yellow pipe, explore the area",
    #         "find out which pipe of the pipeline is damaged, survey",
    #         "check both yellow pipes, look for interesting points",
    #         "find the red marker on the damaged pipe, then spot potential buoys",
    #         "control the yellow pipes, find points of interest",
    #         "check the pipes of the pipeline and explore the surroundings",
    #         "find the red marker on the yellow pipe, perform a survey",
    #         "make a checkup of the pipeline pipes, find some potential buoys",
    #         "find the damaged pipe of the pipeline, survey the surroundings",
    #         "take a look at the yellow pipes and explore the area",
    #         "take a look at the pipes of the pipeline, then survey here",
    #         "look for the damaged pipe and for potential buoys",
    #         "look for the red marker on the given pipeline, after that perform a survey",
    #         "find out on which pipe of the pipeline is the red marker, then get some interesting points",
    #         "find the marker of the yellow pipe and some potential buoys",
    #         "spot the marker on the pipe of the pipeline, then explore the buoy area",
    #         "spot the marker on the pipeline pipe, survey the buoy area",
    #         "control the pipes of the pipeline and survey the buoy area",

    #         "inspect the red marker on the assigned pipeline, and explore the surroundings",
    #         "check the damaged pipe, then find interesting points in the area",
    #         "review the yellow pipe and survey nearby points of interest",
    #         "identify which pipeline pipe is damaged, and explore around",
    #         "examine both yellow pipes and locate interesting spots",
    #         "spot the red marker on the damaged pipe, then locate potential buoys",
    #         "control the yellow pipes and discover points of interest",
    #         "inspect the pipeline pipes and explore nearby surroundings",
    #         "locate the red marker on the yellow pipe, then survey the area",
    #         "perform a checkup of pipeline pipes, and find potential buoys",
    #         "determine the damaged pipeline pipe and survey the surroundings",
    #         "take a look at the yellow pipes and locate interesting points",
    #         "review the pipeline pipes, then explore the area",
    #         "search for the damaged pipe and identify potential buoys",
    #         "find the red marker on the pipeline, then survey the area",
    #         "determine which pipeline pipe has the red marker, and locate interesting points",
    #         "find the yellow pipe marker and identify potential buoys",
    #         "spot the pipeline pipe marker, then explore the buoy area",
    #         "locate the pipeline pipe marker and survey the buoy area",
    #         "inspect the pipeline pipes and explore the buoy area",
    #         "check for red markers on the assigned pipeline, then explore the surroundings",
    #         "review damaged pipes and locate interesting points",
    #         "examine yellow pipes and find potential buoys",
    #         "inspect each pipeline pipe and explore surrounding points",
    #         "spot damaged pipes and survey the nearby area",
    #         "locate red markers on damaged pipes and explore potential points",
    #         "check the yellow pipes and survey surrounding area",
    #         "inspect all pipeline pipes and find potential buoys",
    #         "determine damaged pipeline sections and explore nearby points",
    #         "examine yellow pipes, then spot interesting points",
    #         "locate red markers on the pipeline and survey the area",
    #         "check the assigned pipeline, then find potential buoys",
    #         "spot markers on pipeline pipes and explore the surroundings",
    #         "control the pipeline pipes, then explore potential buoy locations",
    #         "inspect each pipeline pipe and find interesting points",
    #         "review damaged pipes and survey the surrounding area",
    #         "identify yellow pipes and locate potential points",
    #         "locate red markers on the pipeline and explore the area",
    #         "check yellow pipe markers and survey the buoy area",
    #         "inspect the pipeline pipes, then find potential buoys",
    #         "spot damaged pipes and explore surrounding points",
    #         "review the assigned pipeline for markers, then explore interesting points",
    #         "control the yellow pipes, then find potential buoys",
    #         "inspect markers on the pipeline and survey the area",
    #         "find red markers on the assigned pipeline, then explore potential points",
    #         "check damaged pipeline pipes, then locate potential buoys",
    #         "spot yellow pipe markers, then survey the surrounding area",
    #         "inspect each pipeline pipe marker and explore the area",
    #         "review the pipeline and locate potential buoy points",
    #         "check red markers on the pipeline, then explore the area"
    #     ]


    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('pl0_nip')}",
    #         "command": "find the red marker and map the area",
    #         "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. survey, map area"
    #     },
    #     "variations": [
    #         "find the red marker on the assigned pipeline, then map the area",
    #         "find the red marker on the damaged pipe and map the area",
    #         "check the yellow pipe, map the buoy area",
    #         "find out which pipe of the pipeline is damaged, map buoy area",
    #         "check both yellow pipes, parse interesting points",
    #         "find the red marker on the damaged pipe, then parse potential buoys",
    #         "control the yellow pipes, analyze points of interest",
    #         "check the pipes of the pipeline and map the surroundings",
    #         "find the red marker on the yellow pipe, analyze everything in the buoy area",
    #         "make a checkup of the pipeline pipes, analyze the potential buoys",
    #         "find the damaged pipe of the pipeline, analyze every object in the surroundings",
    #         "take a look at the yellow pipes and parse every object in the area",
    #         "take a look at the pipes of the pipeline, then analyze every potential buoy",
    #         "look for the damaged pipe and examine potential buoys",
    #         "look for the red marker on the given pipeline, after that examine potential buoys",
    #         "find out on which pipe of the pipeline is the red marker, then analyze some interesting points",
    #         "find the marker of the yellow pipe and parse potential buoys",
    #         "spot the marker on the pipe of the pipeline, then map the buoy area",
    #         "spot the marker on the pipeline pipe, analyze everything in the buoy area",
    #         "control the pipes of the pipeline and parse everything in the buoy area",

    #         "inspect the red marker on the assigned pipeline, then map the surroundings",
    #         "check the damaged pipe, then map nearby area",
    #         "review the yellow pipe and map the buoy area",
    #         "identify the damaged pipeline section, then map the buoy area",
    #         "examine both yellow pipes and parse all interesting points",
    #         "spot the red marker on the damaged pipe, then parse potential buoy locations",
    #         "control the yellow pipes and analyze all points of interest",
    #         "inspect the pipeline pipes and map the surroundings",
    #         "locate the red marker on the yellow pipe, then analyze everything in the buoy area",
    #         "perform a checkup of pipeline pipes, then analyze potential buoys",
    #         "determine the damaged pipeline pipe, then analyze every object in surroundings",
    #         "take a look at the yellow pipes and parse all objects in the area",
    #         "review the pipeline pipes, then analyze every potential buoy",
    #         "look for the damaged pipe and parse potential buoys",
    #         "find the red marker on the pipeline, then examine potential buoys",
    #         "determine which pipeline pipe has the red marker, then analyze interesting points",
    #         "find the yellow pipe marker, then parse potential buoys",
    #         "spot the pipeline pipe marker, then map the buoy area",
    #         "locate the pipeline pipe marker, then analyze everything in the buoy area",
    #         "control the pipeline pipes and parse everything in the buoy area",
    #         "inspect red markers on the assigned pipeline, then map the area",
    #         "check yellow pipes and parse potential buoys",
    #         "review pipeline pipes and analyze points of interest",
    #         "identify damaged pipe sections, then map the area",
    #         "spot red markers on damaged pipes, then parse potential buoys",
    #         "inspect yellow pipes and analyze the surroundings",
    #         "check all pipeline pipes, then parse potential buoys",
    #         "find the damaged pipeline section, then analyze everything in area",
    #         "examine yellow pipes, then parse interesting points",
    #         "locate red markers on the pipeline, then map the buoy area",
    #         "check the assigned pipeline, then parse potential buoys",
    #         "spot markers on pipeline pipes, then analyze the surroundings",
    #         "control the pipeline pipes, then parse potential buoy objects",
    #         "inspect each pipeline pipe, then analyze interesting points",
    #         "review damaged pipes and map the surroundings",
    #         "identify yellow pipes and parse potential buoys",
    #         "locate red markers on the pipeline, then analyze everything",
    #         "check yellow pipe markers, then parse potential buoys",
    #         "inspect the pipeline pipes, then analyze potential buoys",
    #         "spot damaged pipes and parse the surroundings",
    #         "review the assigned pipeline for markers, then analyze interesting points",
    #         "control yellow pipes, then parse potential buoy points",
    #         "inspect markers on the pipeline, then map the area",
    #         "find red markers on the assigned pipeline, then parse potential points",
    #         "check damaged pipeline pipes, then analyze potential buoys",
    #         "spot yellow pipe markers, then map the surroundings",
    #         "inspect each pipeline pipe marker, then analyze potential buoys",
    #         "review the pipeline, then parse all potential buoy points",
    #         "check red markers on the pipeline, then analyze everything in the area"
    #     ]


    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('pl0')}",
    #         "command": "s",
    #         "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. close valve 4. catch ring 5. surface"
    #     },
    #     "variations":[
    #         "first close the valve, second catch the ring, third surface",
    #         "first close the valve, then catch the ring, finally surface",
    #         "shut the valve, capture the ring and go to surface",
    #         "close the valve, then grab the ring, then surface",
    #         "close valve, pick up the ring and surface",
    #         "seal the valve, catch the ring and move to surface",
    #         "close the valve, obtain the ring and then surface",
    #         "shut valve, collect the ring, head to surface",
    #         "turn off the valve, grab the ring and rise to surface",
    #         "close valve, snatch the ring, ascend to surface",
    #         "close the valve, get the ring and surface",
    #         "stop the valve, take the ring and go up",
    #         "close valve, seize the ring and surface",
    #         "close valve, acquire the ring and move upward",
    #         "shut off the valve, catch the ring and ascend",
    #         "seal off valve, capture the ring and surface",
    #         "close valve, then pick up the ring and surface",
    #         "turn the valve off, then collect the ring and surface",
    #         "stop flow with the valve, then grab the ring and rise",
    #         "close valve, obtain ring, make it to surface",

    #         "first shut the valve, next grab the ring, then ascend",
    #         "close the valve first, then seize the ring, finally go to surface",
    #         "turn off the valve, pick up the ring and make your way to surface",
    #         "seal the valve, collect the ring, then rise to surface",
    #         "shut off the valve, acquire the ring and move upward",
    #         "close valve, catch the ring and head toward the surface",
    #         "stop the valve, grab the ring, then surface",
    #         "turn the valve off, obtain the ring and ascend",
    #         "close the valve first, then get the ring and rise",
    #         "seal off the valve, pick up the ring, move to surface",
    #         "close valve, snatch the ring and make it to surface",
    #         "shut valve, seize the ring and rise to surface",
    #         "stop flow with the valve, catch the ring and go up",
    #         "close the valve, capture the ring and ascend",
    #         "first seal the valve, next collect the ring, then surface",
    #         "turn off the valve first, then acquire the ring and move to surface",
    #         "shut the valve, grab the ring and ascend to surface",
    #         "close valve, get the ring, then make it to surface",
    #         "seal the valve, seize the ring and head to surface",
    #         "stop the valve, pick up the ring and ascend",
    #         "close the valve, collect the ring, then rise",
    #         "first close the valve, then obtain the ring and ascend",
    #         "shut off the valve, catch the ring and go up",
    #         "turn the valve off, grab the ring and make it to surface",
    #         "seal off the valve, acquire the ring, then surface",
    #         "close the valve first, then pick up the ring and ascend",
    #         "stop flow with the valve, collect the ring and head to surface",
    #         "shut the valve, seize the ring and move to surface",
    #         "close valve, catch the ring, then ascend",
    #         "turn off the valve, pick up the ring and rise",
    #         "seal the valve, grab the ring and go up",
    #         "close valve, obtain the ring and move to surface",
    #         "first shut off the valve, then seize the ring and ascend",
    #         "stop the valve, snatch the ring and rise to surface",
    #         "turn the valve off, acquire the ring and move upward",
    #         "shut off the valve, pick up the ring and head to surface",
    #         "close valve, capture the ring, then ascend",
    #         "seal off valve, obtain the ring and go to surface",
    #         "first close the valve, then collect the ring and rise"
    #     ]

    # },
    #     {
    #     "base": {
    #         "context": f"{ctx_('pl1')}",
    #         "command": "close the valve, grab the ring and surface",
    #         "output": "Plan: 1. check yellow pipe 2. close valve 3. catch ring 4. surface"
    #     },
    #     "variations":[
    #         "first close the valve, second catch the ring, third surface",
    #         "first close the valve, then catch the ring, finally surface",
    #         "shut the valve, capture the ring and go to surface",
    #         "close the valve, then grab the ring, then surface",
    #         "close valve, pick up the ring and surface",
    #         "seal the valve, catch the ring and move to surface",
    #         "close the valve, obtain the ring and then surface",
    #         "shut valve, collect the ring, head to surface",
    #         "turn off the valve, grab the ring and rise to surface",
    #         "close valve, snatch the ring, ascend to surface",
    #         "close the valve, get the ring and surface",
    #         "stop the valve, take the ring and go up",
    #         "close valve, seize the ring and surface",
    #         "close valve, acquire the ring and move upward",
    #         "shut off the valve, catch the ring and ascend",
    #         "seal off valve, capture the ring and surface",
    #         "close valve, then pick up the ring and surface",
    #         "turn the valve off, then collect the ring and surface",
    #         "stop flow with the valve, then grab the ring and rise",
    #         "close valve, obtain ring, make it to surface",

    #         "first shut the valve, next grab the ring, then ascend",
    #         "close the valve first, then seize the ring, finally go to surface",
    #         "turn off the valve, pick up the ring and make your way to surface",
    #         "seal the valve, collect the ring, then rise to surface",
    #         "shut off the valve, acquire the ring and move upward",
    #         "close valve, catch the ring and head toward the surface",
    #         "stop the valve, grab the ring, then surface",
    #         "turn the valve off, obtain the ring and ascend",
    #         "close the valve first, then get the ring and rise",
    #         "seal off the valve, pick up the ring, move to surface",
    #         "close valve, snatch the ring and make it to surface",
    #         "shut valve, seize the ring and rise to surface",
    #         "stop flow with the valve, catch the ring and go up",
    #         "close the valve, capture the ring and ascend",
    #         "first seal the valve, next collect the ring, then surface",
    #         "turn off the valve first, then acquire the ring and move to surface",
    #         "shut the valve, grab the ring and ascend to surface",
    #         "close valve, get the ring, then make it to surface",
    #         "seal the valve, seize the ring and head to surface",
    #         "stop the valve, pick up the ring and ascend",
    #         "close the valve, collect the ring, then rise",
    #         "first close the valve, then obtain the ring and ascend",
    #         "shut off the valve, catch the ring and go up",
    #         "turn the valve off, grab the ring and make it to surface",
    #         "seal off the valve, acquire the ring, then surface",
    #         "close the valve first, then pick up the ring and ascend",
    #         "stop flow with the valve, collect the ring and head to surface",
    #         "shut the valve, seize the ring and move to surface",
    #         "close valve, catch the ring, then ascend",
    #         "turn off the valve, pick up the ring and rise",
    #         "seal the valve, grab the ring and go up",
    #         "close valve, obtain the ring and move to surface",
    #         "first shut off the valve, then seize the ring and ascend",
    #         "stop the valve, snatch the ring and rise to surface",
    #         "turn the valve off, acquire the ring and move upward",
    #         "shut off the valve, pick up the ring and head to surface",
    #         "close valve, capture the ring, then ascend",
    #         "seal off valve, obtain the ring and go to surface",
    #         "first close the valve, then collect the ring and rise"
    #     ]
    # },
    # {
    #     "base": {
    #         "context": f"{ctx_('npl_nmp')}",
    #         "command": "find the red marker on the pipeline and check the main pipe",
    #         "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. check main pipe"
    #     },
    #     "variations":[
    #         "find the red marker on the pipeline, then inspect the main pipe",
    #         "look for the marker on the pipeline and check the main pipe",
    #         "spot the marker on the pipe of the pipeline and examine the main pipe",
    #         "find out which pipeline pipe has the red marker, then scan the main pipe",
    #         "check the yellow pipe for the marker, then check the main pipe",
    #         "control the pipes of the pipeline, then analyze the main pipe",
    #         "look for the damaged pipe with marker, then inspect the main pipe",
    #         "find the marker of the yellow pipe, then examine the main pipe",
    #         "check the pipes of the pipeline for marker, then scan the main pipe",
    #         "spot the marker on the pipeline pipe, then detect markers on the main pipe",
    #         "look for the red marker on the given pipeline and check the main pipe",
    #         "find which pipe of the pipeline has the marker, then inspect the main pipe",
    #         "find the damaged pipeline pipe, then examine the main pipe",
    #         "make a checkup of the pipeline pipes, then analyze the main pipe",
    #         "take a look at the yellow pipes, then check the main pipe",
    #         "control the yellow pipes, then scan the main pipe",
    #         "find the red marker on the yellow pipe, then inspect the main pipe",
    #         "spot the marker on the pipeline pipe, then examine the main pipe",
    #         "look for the marker on the assigned pipeline and check the main pipe",
    #         "find out which pipe has the red marker, then check the main pipe",

    #         "inspect the red marker on the pipeline, then scan the main pipe",
    #         "check the pipeline for the marker, then examine the main pipe",
    #         "spot the marker on pipeline sections, then analyze the main pipe",
    #         "determine which pipeline pipe has the red marker, then inspect the main pipe",
    #         "check the yellow pipe for markers, then scan the main pipe",
    #         "control the pipeline pipes and examine the main pipe",
    #         "look for damaged pipes with marker, then analyze the main pipe",
    #         "find the yellow pipe marker and inspect the main pipe",
    #         "review the pipeline pipes for markers, then scan the main pipe",
    #         "spot the marker on the pipeline, then check the main pipe",
    #         "inspect the red marker on the assigned pipeline, then analyze the main pipe",
    #         "find which pipe contains the marker, then examine the main pipe",
    #         "determine the damaged pipeline pipe, then check the main pipe",
    #         "make a checkup of pipeline pipes, then inspect the main pipe",
    #         "review yellow pipes, then scan the main pipe",
    #         "control yellow pipes, then examine the main pipe",
    #         "spot the red marker on the yellow pipe, then analyze the main pipe",
    #         "inspect the pipeline pipe marker, then check the main pipe",
    #         "check the assigned pipeline for markers, then examine the main pipe",
    #         "identify which pipe has the red marker, then scan the main pipe",
    #         "look for the pipeline marker, then detect markers on the main pipe",
    #         "review pipeline pipes and spot the marker, then examine the main pipe",
    #         "check the yellow pipes for the marker, then analyze the main pipe",
    #         "inspect the red marker along the pipeline, then check the main pipe",
    #         "spot markers on the pipeline, then scan the main pipe",
    #         "determine marker location in pipeline, then examine the main pipe",
    #         "look for markers on the assigned pipeline, then inspect the main pipe",
    #         "check each pipeline pipe for the marker, then analyze the main pipe"
    #     ]

    # }
    # {
    #     "base": {
    #         "context": f"{ctx_('')}",
    #         "command": "",
    #         "output": ""
    #     },
    #     "variations":[

    #     ]

    # }

    

]



cmdcla_variations_data = [
    {
        "base": {
            "context": f"{ctx_('y0')}",
            "command": "",
            "previous_response": "Plan: 1. find yellow buoy 2. find yellow buoy 3. pass through",
            "clarification": "",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy 3. pass through"
        },
        "command_variations": [
                    "pass through the gate", #1
                    "cross the gate", #2
                    "pass the gate", #3
                    "pass between the yellow buoys", #4
                    "cross it", #5
                    "go through the gate", #6
                    "move through the gate opening", #7
                    "traverse the gate", #8
                    "navigate through the buoys", #9
                    "proceed forward through the passage" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "execute gate transit procedure maintaining proper alignment between the buoy references", #1
                    "proceed through the designated passage ensuring correct trajectory through the marked channel", #2
                    "navigate the gate aperture with positioning confirmation between the yellow buoys", #3
                    "cross the gate boundary maintaining the correct spatial relationship to both markers", #4
                    "accomplish gate passage by moving through the defined transit corridor", #5
                    "execute proper gate crossing protocol with vector alignment confirmation", #6
                    "establish gate clearance through correct buoy passage procedures", #7
                    "perform gate transit while maintaining required positional parameters", #8
                    "complete gate crossing with verified passage through the buoy channel", #9
                    "execute gate navigation achieving proper transit through the designated passage area" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y1')}",
            "command": "",
            "previous_response": "Plan: 1. find yellow buoy 2. pass through",
            "clarification": "",
            "output": "Plan: 1. find yellow buoy 2. pass through"
        },
        "command_variations": [
                    "pass through the gate", #1
                    "cross the gate", #2
                    "pass the gate", #3
                    "pass between the yellow buoys", #4
                    "cross it", #5
                    "go through the gate", #6
                    "move through the gate opening", #7
                    "traverse the gate", #8
                    "navigate through the buoys", #9
                    "proceed forward through the passage" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "execute gate transit procedure maintaining proper alignment between the buoy references", #1
                    "proceed through the designated passage ensuring correct trajectory through the marked channel", #2
                    "navigate the gate aperture with positioning confirmation between the yellow buoys", #3
                    "cross the gate boundary maintaining the correct spatial relationship to both markers", #4
                    "accomplish gate passage by moving through the defined transit corridor", #5
                    "execute proper gate crossing protocol with vector alignment confirmation", #6
                    "establish gate clearance through correct buoy passage procedures", #7
                    "perform gate transit while maintaining required positional parameters", #8
                    "complete gate crossing with verified passage through the buoy channel", #9
                    "execute gate navigation achieving proper transit through the designated passage area" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "command_variations": [
                    "find the red marker on the pipeline", #1
                    "find a red marker on the pipeline pipe", #2
                    "check the yellow pipes", #3
                    "check the yellow pipe", #4
                    "spot the red marker on the yellow pipe", #5
                    "find the red marker on the yellow pipe", #6
                    "find the damaged pipe", #7
                    "look for the damaged pipe", #8
                    "look for the yellow pipe with the red marker", #9
                    "look for the pipe of the pipeline with a red marker" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "locate the red marker positioned on the yellow pipe section of the assigned pipeline infrastructure", #1
                    "identify the damaged pipe location by finding the red marker indicator on the yellow pipeline segment", #2
                    "execute marker detection protocol identifying the red marker on the designated yellow pipe", #3
                    "scan the yellow pipes systematically to locate the red marker and confirm pipe damage status", #4
                    "assess the pipeline condition by locating the red marker on the yellow pipe section", #5
                    "perform visual inspection to identify the red marker on damaged yellow pipeline components", #6
                    "establish marker position data on the yellow pipe with damage confirmation procedures", #7
                    "conduct pipe damage assessment locating the red marker on the yellow pipeline segment", #8
                    "verify the red marker location on the yellow pipe determining damage extent and position", #9
                    "execute comprehensive pipeline inspection identifying red marker placement on yellow pipes with damage documentation" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. check yellow pipe",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "command_variations": [
                    "find the red marker on the pipeline", #1
                    "find a red marker on the pipeline pipe", #2
                    "check the yellow pipes", #3
                    "check the yellow pipe", #4
                    "spot the red marker on the yellow pipe", #5
                    "find the red marker on the yellow pipe", #6
                    "find the damaged pipe", #7
                    "look for the damaged pipe", #8
                    "look for the yellow pipe with the red marker", #9
                    "look for the pipe of the pipeline with a red marker" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "locate the red marker positioned on the yellow pipe section of the assigned pipeline infrastructure", #1
                    "identify the damaged pipe location by finding the red marker indicator on the yellow pipeline segment", #2
                    "execute marker detection protocol identifying the red marker on the designated yellow pipe", #3
                    "scan the yellow pipes systematically to locate the red marker and confirm pipe damage status", #4
                    "assess the pipeline condition by locating the red marker on the yellow pipe section", #5
                    "perform visual inspection to identify the red marker on damaged yellow pipeline components", #6
                    "establish marker position data on the yellow pipe with damage confirmation procedures", #7
                    "conduct pipe damage assessment locating the red marker on the yellow pipeline segment", #8
                    "verify the red marker location on the yellow pipe determining damage extent and position", #9
                    "execute comprehensive pipeline inspection identifying red marker placement on yellow pipes with damage documentation" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "command_variations": [
                    "find a marker", #1
                    "spot a marker", #2
                    "look for markers", #3
                    "find markers", #4
                    "find a red marker", #5
                    "look for a red marker", #6
                    "find some markers", #7
                    "look for some markers", #8
                    "locate a marker", #9
                    "search for markers" #10
        ],
        "clarification_variations": [
                    "on the pipeline", #1
                    "on the yellow pipes", #2
                    "on the yellow pipe", #3
                    "damage marker on pipeline", #4
                    "on the pipeline structure", #5
                    "on the pipeline pipes", #6
                    "on the infrastructure", #7
                    "on the system", #8
                    "in the area", #9
                    "in the structure" #10
        ],
        "command_test_variations": [
                    "identify marker locations without predetermined specification of target infrastructure or marker type", #1
                    "execute marker detection protocol with context-dependent location determination procedures", #2
                    "locate markers requiring clarification on infrastructure type and marker classification", #3
                    "perform marker search establishing identity and location through contextual analysis", #4
                    "conduct marker identification with ambiguous infrastructure reference requiring situational interpretation", #5
                    "scan for markers with unspecified location parameters and marker type determination", #6
                    "identify marker presence and characteristics based on provided clarification context", #7
                    "locate markers through contextual interpretation of location and infrastructure specifications", #8
                    "execute marker detection with deferred location specification pending clarification input", #9
                    "identify markers establishing location and type through contextual analysis procedures" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe"
        },
        "command_variations": [
                    "parse the pipe", #1
                    "examine the pipe", #2
                    "check the pipe", #3
                    "analyze the pipe", #4
                    "review the pipe", #5
                    "inspect the pipe", #6
                    "assess the pipe", #7
                    "study the pipe", #8
                    "evaluate the pipe", #9
                    "scan the pipe" #10
        ],
        "clarification_variations": [
                    "the pipe of the pipeline", #1
                    "the yellow pipe", #2
                    "the yellow one", #3
                    "the one on the pipeline", #4
                    "the pipeline one", #5
                    "the marked pipe", #6
                    "the pipe section", #7
                    "the designated pipe", #8
                    "the target pipe", #9
                    "the specified one" #10
        ],
        "command_test_variations": [
                    "analyze the designated pipe section establishing structural parameters and condition assessment", #1
                    "perform detailed examination of the specified pipe with comprehensive analysis procedures", #2
                    "execute pipe inspection protocol on the identified target with evaluation documentation", #3
                    "conduct systematic review of the pipe segment determining operational status and characteristics", #4
                    "assess the referenced pipe with detailed analysis establishing condition and specification data", #5
                    "examine the specific pipe section through structured evaluation procedures with documentation", #6
                    "perform pipe analysis on the designated target establishing parameters and assessment criteria", #7
                    "conduct comprehensive pipe review with detailed examination and characterization procedures", #8
                    "analyze the specified pipe infrastructure with systematic evaluation and status documentation", #9
                    "execute detailed pipe assessment protocol establishing comprehensive analysis and evaluation parameters" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe",
            "output": "Plan: 1. check main pipe"
        },
        "command_variations": [
                    "find a marker", #1
                    "spot a marker", #2
                    "look for markers", #3
                    "find markers", #4
                    "find a red marker", #5
                    "look for a red marker", #6
                    "find some markers", #7
                    "look for some markers", #8
                    "locate a marker", #9
                    "search for markers" #10
        ],
        "clarification_variations": [
                    "on the main pipe", #1
                    "the ones on the main pipe", #2
                    "those on the main pipe", #3
                    "along the main pipe", #4
                    "follow the main pipe", #5
                    "on the main pipeline", #6
                    "the markers on the main pipe", #7
                    "along the main structure", #8
                    "on the primary pipe", #9
                    "throughout the main pipe" #10
        ],
        "command_test_variations": [
                    "identify markers positioned along the main pipe structure with systematic location documentation", #1
                    "execute marker detection protocol on the main pipe establishing complete marker inventory", #2
                    "locate all markers on the main pipeline with comprehensive positional mapping", #3
                    "perform systematic marker search along the main pipe determining marker density and distribution", #4
                    "scan the main pipe structure for markers establishing detection and classification procedures", #5
                    "identify marker locations throughout the main pipe with detailed coordinate documentation", #6
                    "conduct marker survey along the primary pipe establishing presence and significance parameters", #7
                    "execute comprehensive marker analysis on the main pipe with complete documentation", #8
                    "locate and verify all markers on the main structure establishing operational data", #9
                    "perform detailed marker assessment throughout the main pipe with systematic mapping procedures" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('npl_nmp')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe",
            "output": "Plan: 1. check main pipe"
        },
        "command_variations": [
                    "find a marker", #1
                    "spot a marker", #2
                    "look for markers", #3
                    "find markers", #4
                    "find a red marker", #5
                    "look for a red marker", #6
                    "find some markers", #7
                    "look for some markers", #8
                    "locate a marker", #9
                    "search for markers" #10
        ],
        "clarification_variations": [
                    "on the main pipe", #1
                    "the ones on the main pipe", #2
                    "those on the main pipe", #3
                    "along the main pipe", #4
                    "follow the main pipe", #5
                    "on the main pipeline", #6
                    "the markers on the main pipe", #7
                    "along the main structure", #8
                    "on the primary pipe", #9
                    "throughout the main pipe" #10
        ],
        "command_test_variations": [
                    "identify markers positioned along the main pipe structure with systematic location documentation", #1
                    "execute marker detection protocol on the main pipe establishing complete marker inventory", #2
                    "locate all markers on the main pipeline with comprehensive positional mapping", #3
                    "perform systematic marker search along the main pipe determining marker density and distribution", #4
                    "scan the main pipe structure for markers establishing detection and classification procedures", #5
                    "identify marker locations throughout the main pipe with detailed coordinate documentation", #6
                    "conduct marker survey along the primary pipe establishing presence and significance parameters", #7
                    "execute comprehensive marker analysis on the main pipe with complete documentation", #8
                    "locate and verify all markers on the main structure establishing operational data", #9
                    "perform detailed marker assessment throughout the main pipe with systematic mapping procedures" #10
        ]
    },
    {
        "base": {
            "context": f"{ctx_('nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. map area",
            "output": "Plan: 1. survey 2. map area"
        },
        "command_variations": [
                    "map the area", #1
                    "analyze the area", #2
                    "parse all points of interest", #3
                    "analyze the interesting points", #4
                    "parse each object of interest", #5
                    "examine every point closely", #6
                    "map the interesting points", #7
                    "analyze the surroundings", #8
                    "parse the region carefully", #9
                    "examine the region fully" #10
        ],
        "clarification_variations": [
                    "find the interesting points first", #1
                    "perform a survey first", #2
                    "get something to parse", #3
                    "survey and then do it", #4
                    "first survey for interesting points", #5
                    "identify points before analysis", #6
                    "gather data first", #7
                    "locate targets before mapping", #8
                    "scan for points initially", #9
                    "establish points of interest first" #10
        ],
        "command_test_variations": [
                    "execute sequential procedures initiating area survey to identify points of interest prior to comprehensive analysis", #1
                    "perform prerequisite survey establishing target identification followed by detailed area mapping and analysis", #2
                    "conduct initial reconnaissance to locate interesting points then execute systematic region parsing procedures", #3
                    "establish area survey protocol first identifying points of interest before detailed examination", #4
                    "execute two-phase analysis beginning with point identification then proceeding to comprehensive area mapping", #5
                    "perform systematic survey to establish baseline data then conduct detailed analysis of identified interesting points", #6
                    "initiate area reconnaissance establishing point locations prior to executing detailed mapping and parsing operations", #7
                    "conduct preliminary survey phase identifying targets then execute complete area analysis procedures", #8
                    "execute phased approach performing initial point location survey followed by comprehensive region analysis", #9
                    "establish data collection through initial survey then proceed to detailed area examination and mapping procedures" #10
        ]


    },
        {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. map area",
            "output": "Plan: 1. survey 2. map area"
        },
        "command_variations": [
                    "map the buoy area", #1
                    "parse everything in the buoy area", #2
                    "analyze every object in the buoy area", #3
                    "analyze the buoy area", #4
                    "parse the buoy area", #5
                    "parse every potential buoy", #6
                    "analyze every potential buoy", #7
                    "scope every interesting point that could be a buoy", #8
                    "check each point that could be a buoy", #9
                    "parse everything that could be a buoy" #10
        ],
        "clarification_variations": [
                    "find potential buoys first", #1
                    "perform a survey first", #2
                    "get something to parse", #3
                    "survey and then do it", #4
                    "first survey for potential buoys", #5
                    "identify buoys before analysis", #6
                    "locate targets first", #7
                    "scan for buoys initially", #8
                    "establish buoy positions first", #9
                    "discover buoy candidates beforehand" #10
        ],
        "command_test_variations": [
                    "execute sequential procedures initiating buoy area survey to identify potential buoys prior to comprehensive analysis", #1
                    "perform prerequisite buoy detection establishing target identification followed by detailed area mapping and parsing", #2
                    "conduct initial reconnaissance to locate buoy candidates then execute systematic region analysis procedures", #3
                    "establish buoy survey protocol first identifying potential targets before detailed examination of the area", #4
                    "execute two-phase analysis beginning with buoy identification then proceeding to comprehensive area mapping", #5
                    "perform systematic buoy survey establishing baseline data then conduct detailed analysis of identified candidates", #6
                    "initiate area reconnaissance establishing buoy locations prior to executing detailed parsing and analysis operations", #7
                    "conduct preliminary buoy detection phase identifying targets then execute complete area analysis procedures", #8
                    "execute phased approach performing initial buoy location survey followed by comprehensive region parsing", #9
                    "establish data collection through initial buoy survey then proceed to detailed area examination and mapping procedures" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find yellow buoy",
            "output": "Plan: 1. survey 2. map area"
        },
        "command_variations": [
                    "find buoys", #1
                    "find some buoys", #2
                    "look for buoys", #3
                    "look for some buoys", #4
                    "find all buoys", #5
                    "look for all of the buoys", #6
                    "locate buoys", #7
                    "search for buoys", #8
                    "spot buoys", #9
                    "identify buoys" #10
        ],
        "clarification_variations": [
                    "in the buoy area", #1
                    "the buoys in the buoy area", #2
                    "the ones in the buoy area", #3
                    "those in the buoy area", #4
                    "colored buoys in the buoy area", #5
                    "in the designated buoy zone", #6
                    "within the buoy perimeter", #7
                    "the buoys present in the area", #8
                    "all buoys in the zone", #9
                    "buoys located in the region" #10
        ],
        "command_test_variations": [
                    "identify and locate all buoy objects within the designated buoy area sector", #1
                    "execute systematic buoy detection protocol scanning the entire buoy zone comprehensively", #2
                    "perform complete buoy survey establishing detection and cataloging of all buoys present", #3
                    "conduct thorough buoy identification establishing complete inventory within the operational area", #4
                    "locate all buoy objects with systematic area coverage and buoy confirmation procedures", #5
                    "execute comprehensive buoy detection throughout the designated zone with position documentation", #6
                    "perform systematic identification and positioning of all buoys within the area perimeter", #7
                    "conduct complete buoy survey with detailed documentation and area coverage verification", #8
                    "identify and establish coordinates for all buoys in the designated area sector", #9
                    "execute thorough buoy reconnaissance establishing comprehensive detection and inventory procedures" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find yellow buoy 2. find yellow buoy",
            "output": "Plan: 1. survey 2. map area"
        },
        "command_variations": [
                    "find buoys", #1
                    "find some buoys", #2
                    "look for buoys", #3
                    "look for some buoys", #4
                    "find all buoys", #5
                    "look for all of the buoys", #6
                    "locate buoys", #7
                    "search for buoys", #8
                    "spot buoys", #9
                    "identify buoys" #10
        ],
        "clarification_variations": [
                    "in the buoy area", #1
                    "the buoys in the buoy area", #2
                    "the ones in the buoy area", #3
                    "those in the buoy area", #4
                    "colored buoys in the buoy area", #5
                    "in the designated buoy zone", #6
                    "within the buoy perimeter", #7
                    "the buoys present in the area", #8
                    "all buoys in the zone", #9
                    "buoys located in the region" #10
        ],
        "command_test_variations": [
                    "identify and locate all buoy objects within the designated buoy area sector", #1
                    "execute systematic buoy detection protocol scanning the entire buoy zone comprehensively", #2
                    "perform complete buoy survey establishing detection and cataloging of all buoys present", #3
                    "conduct thorough buoy identification establishing complete inventory within the operational area", #4
                    "locate all buoy objects with systematic area coverage and buoy confirmation procedures", #5
                    "execute comprehensive buoy detection throughout the designated zone with position documentation", #6
                    "perform systematic identification and positioning of all buoys within the area perimeter", #7
                    "conduct complete buoy survey with detailed documentation and area coverage verification", #8
                    "identify and establish coordinates for all buoys in the designated area sector", #9
                    "execute thorough buoy reconnaissance establishing comprehensive detection and inventory procedures" #10
        ]

    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find yellow buoy",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy"
        },
        "command_variations": [
                    "find buoys", #1
                    "find some buoys", #2
                    "look for buoys", #3
                    "look for some buoys", #4
                    "find all buoys", #5
                    "look for all of the buoys", #6
                    "locate buoys", #7
                    "search for buoys", #8
                    "spot buoys", #9
                    "identify buoys" #10
        ],
        "clarification_variations": [
                    "the yellow ones", #1
                    "yellow buoys", #2
                    "gate buoys", #3
                    "the ones forming the gate", #4
                    "both yellow buoys", #5
                    "both gate buoys", #6
                    "the yellow gate buoys", #7
                    "the pair of yellow buoys", #8
                    "the yellow markers forming the gate", #9
                    "both buoys in the gate" #10
        ],
        "command_test_variations": [
                    "identify and locate the yellow gate buoys establishing their specific positions and spatial relationship", #1
                    "execute buoy detection protocol targeting the yellow buoy pair that defines the gate structure", #2
                    "locate both yellow buoys forming the gate with complete positional and confirmation data", #3
                    "perform systematic identification of the gate buoys establishing dual buoy detection and positioning", #4
                    "conduct comprehensive search for yellow gate buoys establishing complete pair location and documentation", #5
                    "identify the yellow buoy pair with gate designation establishing spatial parameters and confirmation", #6
                    "execute targeted buoy detection for the yellow gate markers with complete location documentation", #7
                    "locate and verify both yellow buoys forming the gate structure with positional accuracy", #8
                    "perform detailed identification of gate buoys establishing yellow buoy coordinates and relationship", #9
                    "conduct systematic detection of the yellow gate buoy pair establishing complete structural definition" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('y0_nip')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find yellow buoy 2. find yellow buoy",
            "output": "Plan: 1. find yellow buoy 2. find yellow buoy"
        },
        "command_variations": [
                    "find buoys", #1
                    "find some buoys", #2
                    "look for buoys", #3
                    "look for some buoys", #4
                    "find all buoys", #5
                    "look for all of the buoys", #6
                    "locate buoys", #7
                    "search for buoys", #8
                    "spot buoys", #9
                    "identify buoys" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "execute buoy detection procedures without predetermined specification requiring contextual interpretation", #1
                    "conduct buoy identification protocol with ambiguous target parameters pending operational clarification", #2
                    "perform systematic buoy search with deferred target specification requiring mission context", #3
                    "identify buoys through contextual understanding of operational environment and mission parameters", #4
                    "execute buoy detection with undefined buoy type classification requiring situational analysis", #5
                    "conduct buoy reconnaissance with open-ended target parameters based on operational context", #6
                    "locate buoys establishing identity and classification through contextual interpretation procedures", #7
                    "perform buoy identification with ambiguous scope requiring environmental and mission assessment", #8
                    "execute buoy detection protocol with context-dependent target determination procedures", #9
                    "conduct systematic buoy search with mission-dependent target specification and identification" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('npl')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe 3. close valve",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. close valve"
        },
        "command_variations": [
                    "close the valve", #1
                    "shut the valve", #2
                    "turn off the valve", #3
                    "seal the valve", #4
                    "stop the valve", #5
                    "shut off the valve", #6
                    "turn the valve off", #7
                    "seal off the valve", #8
                    "stop flow with the valve", #9
                    "close off the valve" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "execute valve closure protocol ensuring complete flow isolation with mechanical lock engagement", #1
                    "shut down the valve system establishing total blockage of the specified flow path", #2
                    "close the valve and verify sealed position with operational status confirmation", #3
                    "engage valve shutdown procedures restricting all flow through the designated valve", #4
                    "activate valve closure mechanisms achieving complete flow interruption and system isolation", #5
                    "perform valve shutdown sequence establishing locked closed position and confirming flow cessation", #6
                    "close and seal the valve preventing any residual flow through the system", #7
                    "execute valve control protocol terminating flow by complete mechanical valve closure", #8
                    "shut off the valve and establish permanent sealed condition for operational integrity", #9
                    "perform immediate valve deactivation and lock engagement establishing complete flow isolation" #10
        ]


    },
    {
        "base": {
            "context": f"{ctx_('npl')}",
            "command": "",
            "clarification": "",
            "previous_response": "Plan: 1. find given pipeline 2. check yellow pipe 3. catch ring",
            "output": "Plan: 1. find given pipeline 2. check yellow pipe 3. catch ring"
        },
        "command_variations": [
                    "grab the ring", #1
                    "take the ring", #2
                    "seize the ring", #3
                    "capture the ring", #4
                    "collect the ring", #5
                    "pick up the ring", #6
                    "snatch the ring", #7
                    "get the ring", #8
                    "acquire the ring", #9
                    "obtain the ring" #10
        ],
        "clarification_variations": [
                    "ok", #1
                    "fine", #2
                    "correct", #3
                    "this is ok", #4
                    "this is fine", #5
                    "understood", #6
                    "affirmative", #7
                    "that's right", #8
                    "all good", #9
                    "confirmed" #10
        ],
        "command_test_variations": [
                    "locate and physically secure the ring object ensuring proper containment and custody establishment", #1
                    "execute ring acquisition protocol with careful handling and possession verification procedures", #2
                    "retrieve the ring object establishing operational custody with appropriate safeguarding measures", #3
                    "seize and secure the ring target completing the acquisition objective with retention confirmation", #4
                    "obtain the designated ring through systematic collection procedures with status documentation", #5
                    "capture and hold the ring ensuring continued possession and control throughout the operation", #6
                    "claim the ring target with proper acquisition documentation and custody transfer confirmation", #7
                    "secure the ring object preventing loss or compromise during the collection phase", #8
                    "acquire the ring completing the target collection objective with verified possession status", #9
                    "obtain full operational control of the ring through successful acquisition and containment procedures" #10
        ]


    },
    # {
    #     "base": {
    #         "context": f"{ctx_('')}",
    #         "command": "",
    #         "clarification": "",
    #         "previous_response": 
    #         "output": ""
    #     },
    #     "command_variations":[
    #     ],
    #     "clarification_variations":[

    #     ]

    # },

]


from collections import defaultdict, Counter
import random
import json

def main():
    """Generate the training dataset"""
    train_examples = []
    test_examples = []
    train_examplesc = []
    test_examplesc = []

    # Generate all clarification variations...
    for cmdcla_set in cmdcla_variations_data:
        if "command_test_variations" not in cmdcla_set:
            cmdcla_set["command_test_variations"] = cmdcla_set["command_variations"]
        
        train_ex, test_ex = generate_cmdcla_variations(cmdcla_set["base"], cmdcla_set["command_variations"], cmdcla_set["command_test_variations"], cmdcla_set["clarification_variations"])
        train_examplesc.extend(train_ex)
        test_examplesc.extend(test_ex)

    # Generate all variations...
    for variation_set in command_variations_data:
        if "test_variations" not in variation_set:
            variation_set["test_variations"] = variation_set["variations"]
        
        train_ex, test_ex = generate_command_variations(variation_set["base"], variation_set["variations"], variation_set["test_variations"])
        train_examples.extend(train_ex)
        test_examples.extend(test_ex)


    # --- Group by output pattern ---
    train_pattern_groups = defaultdict(lambda: {"main": [], "clar": []})
    test_pattern_groups = defaultdict(lambda: {"main": [], "clar": []})

    for ex in train_examples:
        train_pattern_groups[ex["output"]]["main"].append(ex)

    for ex in train_examplesc:
        train_pattern_groups[ex["output"]]["clar"].append(ex)

    for ex in test_examples:
        test_pattern_groups[ex["output"]]["main"].append(ex)

    for ex in test_examplesc:
        test_pattern_groups[ex["output"]]["clar"].append(ex)


    train_max_per_pattern = 100  # or your desired training cap
    test_max_per_pattern = 10   # or your desired test cap

    examplesc_fraction = 0.3
    train_balanced_examples = []
    test_balanced_examples = []
    train_pattern_counts = Counter()
    test_pattern_counts = Counter()

    # --- Process each batch (TRAINING) ---
    for pattern, group_dict in train_pattern_groups.items():
        main_group = group_dict["main"]
        clar_group = group_dict["clar"]
        
        if len(main_group) + len(clar_group) == 0:
            continue
        
        # If we have BOTH types, honor the ratio
        # If we only have ONE type, take up to cap from that type
        if len(clar_group) > 0:
            # Has both: apply ratio to total
            total_available = len(main_group) + len(clar_group)
            clar_target = int(total_available * examplesc_fraction)
            main_target = total_available - clar_target
        else:
            # Only main exists: take up to cap from main
            main_target = min(len(main_group), train_max_per_pattern)
            clar_target = 0
        
        selected_main = random.sample(main_group, min(len(main_group), main_target))
        selected_clar = random.sample(clar_group, min(len(clar_group), clar_target))
        
        merged_group = []
        
        for ex in selected_main:
            prompt = f"{SEQ_LINE}\n\nCurrent knowledge: {ex['context']}\n\n"
            prompt += f"Command: {ex['command']}\n"
            prompt += f"\n{FINAL_LINE}"
            merged_group.append({"input": prompt, "output": ex["output"]})
        
        for ex in selected_clar:
            prompt = f"{SEQ_LINE}\n\nCurrent knowledge: {ex['context']}\n\n"
            prompt += f"Command: {ex['command']}\n"
            prompt += f"Previous response: {ex['previous_response']}\n"
            prompt += f"Clarification: {ex['clarification']}\n"
            prompt += f"\n{FINAL_LINE_C}"
            merged_group.append({"input": prompt, "output": ex["output"]})
        
        random.shuffle(merged_group)
        
        # Cap AFTER merging and shuffling
        if len(merged_group) > train_max_per_pattern:
            merged_group = random.sample(merged_group, train_max_per_pattern)
        
        train_balanced_examples.extend(merged_group)
        train_pattern_counts[pattern] = len(merged_group)



    # --- Process each batch (TEST) ---
    for pattern, group_dict in test_pattern_groups.items():
        main_group = group_dict["main"]
        clar_group = group_dict["clar"]
        
        if len(main_group) + len(clar_group) == 0:
            continue
        
        if len(clar_group) > 0:
            total_available = len(main_group) + len(clar_group)
            clar_target = int(total_available * examplesc_fraction)
            main_target = total_available - clar_target
        else:
            main_target = min(len(main_group), test_max_per_pattern)
            clar_target = 0
        
        selected_main = random.sample(main_group, min(len(main_group), main_target))
        selected_clar = random.sample(clar_group, min(len(clar_group), clar_target))
        
        merged_group = []
        
        for ex in selected_main:
            prompt = f"{SEQ_LINE}\n\nCurrent knowledge: {ex['context']}\n\n"
            prompt += f"Command: {ex['command']}\n"
            prompt += f"\n{FINAL_LINE}"
            merged_group.append({"input": prompt, "output": ex["output"]})
        
        for ex in selected_clar:
            prompt = f"{SEQ_LINE}\n\nCurrent knowledge: {ex['context']}\n\n"
            prompt += f"Command: {ex['command']}\n"
            prompt += f"Previous response: {ex['previous_response']}\n"
            prompt += f"Clarification: {ex['clarification']}\n"
            prompt += f"\n{FINAL_LINE_C}"
            merged_group.append({"input": prompt, "output": ex["output"]})
        
        random.shuffle(merged_group)
        
        # Cap AFTER merging and shuffling
        if len(merged_group) > test_max_per_pattern:
            merged_group = random.sample(merged_group, test_max_per_pattern)
        
        test_balanced_examples.extend(merged_group)
        test_pattern_counts[pattern] = len(merged_group)


    # --- Shuffle all examples ---
    random.shuffle(train_balanced_examples)
    random.shuffle(test_balanced_examples)

    # --- Write to JSONL ---

    with open(train_output_file, 'w', encoding='utf-8') as f:
        for ex in train_balanced_examples:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')

    with open(test_output_file, 'w', encoding='utf-8') as f:
        for ex in test_balanced_examples:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')

    print("\n" + "="*90)
    print("DATASETS GENERATED SUCCESSFULLY")
    print("="*90)

    print(f"\n{'TRAINING DATASET':^90}")
    print(f"{train_output_file}")
    print(f"  - Total examples: {len(train_balanced_examples)}")
    print(f"  - Unique output patterns: {len(train_pattern_counts)}")
    if len(train_pattern_counts) > 0:
        avg_per_pattern = len(train_balanced_examples) / len(train_pattern_counts)
        print(f"  - Average examples per pattern: {avg_per_pattern:.1f}")
        print(f"  - Max examples per pattern: {max(train_pattern_counts.values())}")

    print(f"\n{'TEST DATASET':^90}")
    print(f"{test_output_file}")
    print(f"  - Total examples: {len(test_balanced_examples)}")
    print(f"  - Unique output patterns: {len(test_pattern_counts)}")
    if len(test_pattern_counts) > 0:
        avg_per_pattern = len(test_balanced_examples) / len(test_pattern_counts)
        print(f"  - Average examples per pattern: {avg_per_pattern:.1f}")
        print(f"  - Max examples per pattern: {max(test_pattern_counts.values())}")

    print(f"\n{'COMPOSITION RATIOS':^90}")
    examples_ratio = (len([ex for ex in train_balanced_examples if "clarification" not in ex]) / len(train_balanced_examples) * 100) if train_balanced_examples else 0
    examplesc_ratio = (len([ex for ex in train_balanced_examples if "clarification" in ex]) / len(train_balanced_examples) * 100) if train_balanced_examples else 0
    print(f"  - Main examples: {examples_ratio:.1f}%")
    print(f"  - Clarification examples: {examplesc_ratio:.1f}%")
    print(f"  - Note: Ratios adjust dynamically when types are missing for a pattern")

    print(f"\n{'TRAINING EXAMPLES PER PATTERN':^90}")
    for output, count in sorted(train_pattern_counts.items()):
        print(f"  - {output}: {count}")

    print(f"\n{'TEST EXAMPLES PER PATTERN':^90}")
    for output, count in sorted(test_pattern_counts.items()):
        print(f"  - {output}: {count}")

    print(f"\n{'OPERATIONS AVAILABLE':^90}")
    print(f"Count: {len(OPERATIONS)}")
    print(f"Operations: {', '.join(OPERATIONS.keys())}")


if __name__ == "__main__":
    main()


# "", #1
# "", #2
# "",#3
# "",#4
# "",#5
# "",#6
# "",#7
# "",#8
# "",#9
# "",#10
# "",#11
# "",#12
# "",#13
# "",#14
# "",#15
# "",#16
# "",#17
# "",#18
# "",#19
# ""#20
