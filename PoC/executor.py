#!/usr/bin/env python3
"""
Action Executor for robotic mission system.
Receives action operations from LM planner and executes them,
updating the world state accordingly.
Based on behaviors.py actions.
"""

import time
import random
import numpy as np
from typing import List, Dict, Any
from state import World, Buoy, Pipeline, Pipe
from environment import real_world, external_waypoint, pipeline_id
from operations import get_operation, is_valid_operation, OPERATIONS
from actions import *


class ActionExecutor:
    """Executes operations and updates world state"""
    
    def __init__(self, world: World):
        self.world = world
        self.real_world = real_world
        self.action_map = {
            "activate receiver": lambda: activate_receiver(self.world),
            "activate camera": lambda: activate_camera(self.world),
            "activate sonar": lambda: activate_sonar(self.world),
            "deactivate receiver": lambda: deactivate_receiver(self.world),
            "deactivate camera": lambda: deactivate_camera(self.world),
            "deactivate sonar": lambda: deactivate_sonar(self.world),
            "process waypoint": lambda: process_waypoint(self.world, external_waypoint),
            "set target waypoint": lambda: set_target_waypoint(self.world),
            "reach waypoint": lambda: reach_waypoint(self.world),
            "compute path": lambda: compute_path(self.world),
            "is there gate": lambda: is_there_gate(self.world),
            "compute crossing path": lambda: compute_crossing_path(self.world),
            "follow path": lambda: follow_path(self.world),
            "generic survey": lambda: generic_survey(self.world, self.real_world),
            "buoy survey": lambda: buoy_survey(self.world, self.real_world),
            "gate buoy survey": lambda: gate_buoy_survey(self.world, self.real_world),
            "map buoy area": lambda: map_buoy_area(self.world, self.real_world),
            "given pipeline survey": lambda: given_pipeline_survey(self.world, self.real_world, pipeline_id),
            "pipeline survey": lambda: pipeline_survey(self.world, self.real_world),
            "check pipes": lambda: check_pipes(self.world, self.real_world),
            "reach end": lambda: reach_end(self.world, self.real_world),
            "follow pipe": lambda: follow_pipe(self.world, self.real_world),
            "close valve": lambda: close_valve(self.world),
            "grab ring": lambda: grab_ring(self.world),
            "surface": lambda: surface(self.world),
            "tabula rasa": lambda: tabula_rasa(self.world, external_waypoint)
        }
    
    def execute_operation_by_name(self, operation_name: str) -> bool:
        """Execute a single operation by its name."""
        return self.execute_operation([operation_name])


    def execute_operation(self, operation_names: List[str]) -> bool:
        """Execute one or more predefined operations by name. Stops on first failure."""

        for action in operation_names:
            if getattr(self, 'stop_current_mission', False):
                print("⏹️ Mission interrupted by stop keyword!")
                self.stop_current_mission = False  # reset flag
                return False
        
        if not operation_names:
            print("⚠️ Empty operation name list")
            return False

        for seq_index, operation_name in enumerate(operation_names, start=1):
            if operation_name not in OPERATIONS:
                print(f"❌ Unknown operation name: '{operation_name}'")
                return False  # fail-fast
            
            actions = OPERATIONS[operation_name]
            
            if not actions:
                print(f"⚠️ Empty action list for operation: '{operation_name}'")
                return False  # fail-fast

            print(f"\n🚀 [{seq_index}/{len(operation_names)}] Starting operation: '{operation_name}'")
            
            if not self._run_actions(actions):
                print(f"❌ Operation '{operation_name}' failed, requesting clarification")
                return False  # fail-fast

        print(f"\n🏁 All operations completed successfully")
        return True

    def _run_actions(self, actions: List[str]) -> bool:
        """Run a list of actions in order. Returns False immediately if any action fails."""
        for i, action in enumerate(actions, start=1):
            print(f"  [{i}/{len(actions)}] Executing: {action}")
            
            if action not in self.action_map:
                print(f"    ❌ Unknown action: {action}")
                return False  # fail-fast
            
            try:
                result = self.action_map[action]()
                if result is True:
                    print(f"    ✅ {action} completed successfully")
                else:
                    print(f"    ❌ {action} failed or returned False")
                    return False  # fail-fast on warnings/failures
                time.sleep(0.5)  # simulate execution time
            except Exception as e:
                print(f"    ❌ {action} failed with exception: {e}")
                return False  # fail-fast
        return True



def main():
    """Test the action executor with operation names"""
    from state import world
    from action_operations import get_available_operations
    
    # Create world and executor
    executor = ActionExecutor(world)

if __name__ == "__main__":
    main()
