#!/usr/bin/env python3
"""
Action operations derived from behavior trees.
Each tree becomes a simple operation of actions to be executed in order.
"""

# Action operations mapping
OPERATIONS = {
    "acquire target": [
        "activate receiver",
        "process waypoint",
        "deactivate receiver"
    ],
    
    "move to waypoint": [
        "compute path",
        "reach waypoint"
    ],

    "find yellow buoy": [
        "activate camera",
        "gate buoy survey",
        "deactivate camera",
        "is there gate"
    ],
    
    "pass through": [
        "is there gate",
        "compute crossing path",
        "follow path"
    ],
    
    "survey": [
        "generic survey"
    ],
    
    "map area": [
        "activate camera",
        "map buoy area",
        "deactivate camera"
    ],

     "find given pipeline": [
        "activate camera",
        "given pipeline survey",
        "deactivate camera"
    ],
    
    "check yellow pipe": [
        "activate camera",
        "check pipes",
        "deactivate camera"
    ],

    "check main pipe": [
        "activate camera",
        "reach end",
        "follow pipe",
        "deactivate camera"
    ],

    "close valve": [
        "activate camera",
        "close valve",
        "deactivate camera"
    ],

    "catch ring": [
        "activate camera",
        "grab ring",
        "deactivate camera"
    ],

    "surface": [
        "surface"
    ],

    "clear memory": [
        "tabula rasa"
    ]
}

def get_operation(operation_name: str) -> list:
    """Get operation by name"""
    return OPERATIONS.get(operation_name.lower(), [])

def get_available_operation() -> list:
    """Get list of available operation names"""
    return list(OPERATIONS.keys())

def is_valid_operation(operation_name: str) -> bool:
    """Check if operation name is valid"""
    return operation_name.lower() in OPERATIONS