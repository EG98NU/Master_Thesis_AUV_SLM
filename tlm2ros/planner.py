#!/usr/bin/env python3

"""
Language Model Planner for ROS mission system.
Uses FLAN-T5-Base to convert natural language commands and mission memory
into mission names from missions.yaml.
MODIFIED: Support for 'skip' output and failure context handling
"""

import json
import time
import yaml
from typing import List, Dict, Any, Optional
from pathlib import Path
import threading
import queue
import re
from collections import deque
import math
import torch.nn.functional as F
import statistics

# Transformers imports
from transformers import T5ForConditionalGeneration, T5Tokenizer
import torch

# Configuration
MODEL_NAME = "./flan-t5-base_planner"  # Language model to use
CONFIDENCE_THRESHOLD = 0.5
MISSIONS_FILE = Path("missions.yaml")
MISSION_MEMORY_FILE = Path("mission_memory.yaml")
MISSION_HISTORY_FILE = Path("mission_history.yaml")


class MissionMemory:
    """Handles reading and formatting mission memory"""
    
    def __init__(self, memory_file: Path):
        self.memory_file = memory_file
        self.buoys_found = []
    
    def get_mission_history(self) -> str:
        """Read mission history and format unique missions"""
        try:
            history_file = Path("mission_history.yaml")
            if not history_file.exists():
                return ""
            
            with open(history_file, 'r') as f:
                history_data = yaml.safe_load(f)
            
            if not history_data or 'missions' not in history_data:
                return ""
            
            # Extract mission names and keep only unique ones (preserving order)
            missions = history_data.get('missions', [])
            unique_missions = []
            seen = set()
            
            for mission_entry in missions:
                if isinstance(mission_entry, dict):
                    mission_name = mission_entry.get('mission_name', '')
                    if mission_name and mission_name not in seen:
                        unique_missions.append(mission_name)
                        seen.add(mission_name)
            
            if unique_missions:
                return f"Previous missions: {', '.join(unique_missions)}"
            
            return ""
        
        except Exception as e:
            print(f"⚠️ Error reading mission history: {e}")
            return ""
    
    def check_mission_in_history(self, mission_name: str) -> bool:
        """Check if a mission exists in the history"""
        try:
            history_file = Path("mission_history.yaml")
            if not history_file.exists():
                return False
            
            with open(history_file, 'r') as f:
                history_data = yaml.safe_load(f)
            
            if not history_data or 'missions' not in history_data:
                return False
            
            missions = history_data.get('missions', [])
            for mission_entry in missions:
                if isinstance(mission_entry, dict):
                    if mission_entry.get('mission_name', '') == mission_name:
                        return True
            
            return False
        
        except Exception as e:
            print(f"⚠️ Error checking mission history: {e}")
            return False
    
    def update_memory(self) -> str:
        """Read mission memory and history, format for the prompt"""
        try:
            memory_parts = []
            
            # Part 1: Buoys found
            if self.memory_file.exists():
                with open(self.memory_file, 'r') as f:
                    memory_data = yaml.safe_load(f)
                
                if memory_data and 'discovered_buoys' in memory_data:
                    # Extract buoy colors
                    self.buoys_found = []
                    for buoy_entry in memory_data.get('discovered_buoys', []):
                        if isinstance(buoy_entry, dict) and 'color' in buoy_entry:
                            self.buoys_found.append(buoy_entry['color'])
                    
                    if self.buoys_found:
                        memory_parts.append(f"Buoys found so far: {', '.join(self.buoys_found)}")
            
            # Part 2: Mission history
            history_str = self.get_mission_history()
            if history_str:
                memory_parts.append(history_str)
            
            # Combine with separator
            if memory_parts:
                return " | ".join(memory_parts)
            
            return ""
        
        except Exception as e:
            print(f"⚠️ Error reading mission memory: {e}")
            return ""


class ActionPlanner:
    """Language model planner using FLAN-T5"""
    
    def __init__(self, missions_file: Path):
        # Set the model name
        self.model_name = MODEL_NAME
        
        # Load the tokenizer and model
        print(f"🔧 Loading model: {self.model_name}")
        self.tokenizer = T5Tokenizer.from_pretrained(self.model_name)
        self.model = T5ForConditionalGeneration.from_pretrained(self.model_name)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        print(f"✅ Model loaded on {self.device}")
        
        # Load available missions
        self.available_missions, self.mission_aliases = self._load_missions(missions_file)

        
        self.last_command = None
        self.clarification = None
        self.failure_context = None
    
    def _load_missions(self, missions_file: Path) -> tuple:
        """Load mission names AND aliases from missions.yaml"""
        try:
            if not missions_file.exists():
                print(f"⚠️ Missions file not found: {missions_file}")
                return [], {}
            
            with open(missions_file, 'r') as f:
                missions_data = yaml.safe_load(f)
            
            # Extract mission names (exclude metadata keys)
            mission_names = [k for k in missions_data.keys() 
                            if k not in ['default_mission', 'quadrant_radius', 'mission_aliases']]
            
            # Extract aliases
            aliases = missions_data.get('mission_aliases', {})
            
            print(f"✅ Loaded {len(mission_names)} missions")
            if aliases:
                print(f"✅ Loaded {len(aliases)} mission aliases: {', '.join(aliases.keys())}")
            
            return mission_names, aliases
            
        except Exception as e:
            print(f"❌ Error loading missions: {e}")
            return [], {}

    def _expand_mission_item(self, item: str) -> list:
        """
        Expand a mission item using aliases from missions_n.yaml.
        Returns a list of mission names (single item if no alias found).
        """
        item_normalized = item.strip()
        
        # Check if this item is an alias
        for alias_name, mission_list in self.mission_aliases.items():
            if item_normalized.lower() == alias_name.lower():
                print(f"🔄 Expanding alias '{item_normalized}' -> {mission_list}")
                return mission_list
        
        # No alias found - return as single-item list
        return [item_normalized]

    def plan_actions(self, command: str, memory: str) -> List[str]:
        """Generate mission name(s) from command and memory - can return comma-separated list or 'skip'"""
        try:
            if self.last_command != command:
                # New command - reset clarification and failure context
                self.last_command = command
                self.clarification = None
                self.failure_context = None
            
            # Create prompt for FLAN-T5
            prompt = self._create_prompt(command, memory)
            print("-"*60)
            print(f"🖊️ Model prompt:\n{prompt}\n")
            print("-"*60)
            # Generate response
            inputs = self.tokenizer(prompt, return_tensors="pt", max_length=512, truncation=True)
            inputs = inputs.to(self.device)
            
            with torch.no_grad():
                outputs = self.model.generate(
                    inputs.input_ids,
                    max_length=100,
                    num_beams=4,
                    temperature=0.7,
                    do_sample=True,
                    pad_token_id=self.tokenizer.pad_token_id
                )
            
            raw_response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
            
            # Store previous output for potential clarification
            self.last_output = raw_response
            
            # Parse and score - now handles comma-separated missions and 'skip'
            missions = self._parse_and_score(raw_response, prompt, CONFIDENCE_THRESHOLD)
            
            return missions
        
        except Exception as e:
            print(f"❌ Error in plan_actions: {e}")
            return []
    
    def _create_prompt(self, command: str, memory: str) -> str:
        """Create prompt for FLAN-T5 with events context from mission logs"""
        available_missions_str = "skip, " + ", ".join(self.available_missions)
        prompt = f"""Available missions: {available_missions_str}
    Memory: {memory}
    Command: {command}"""
        
        # Add failure context (now includes events from logs_processor)
        if self.failure_context:
            prompt += f"\nFailure context: {self.failure_context}"
        
        # Add clarification or repeat mission section with previous response
        if self.clarification:
            # Check if this is a duplicate mission scenario
            if "Repeat mission:" in self.clarification:
                # Duplicate mission case: use "Repeat mission:" field instead of "Clarification:"
                if hasattr(self, 'last_output') and self.last_output:
                    prompt += f"\nPrevious response: {self.last_output}"
                # Extract the repeat mission message (everything after "Repeat mission:")
                repeat_msg = self.clarification.split("Repeat mission:", 1)[1].strip()
                prompt += f"\nRepeat mission: {repeat_msg}"
                prompt += "\nInstruction: The previous mission was already completed. Generate a new mission plan based on the user's input. If the user confirms repeating, return the same mission. If they provide new input, generate appropriate missions."
            else:
                # Regular clarification (not duplicate)
                if hasattr(self, 'last_output') and self.last_output:
                    prompt += f"\nPrevious response: {self.last_output}"
                prompt += f"\nClarification: {self.clarification}"
                prompt += "\nInstruction: Given the command, mission memory, failure context with events, and clarification, generate one or more missions from the available missions. If multiple missions are needed, separate them with commas."
        else:
            prompt += "\nInstruction: Given the command and the mission memory, generate one or more missions from the available missions. If multiple missions are needed, separate them with commas."
        
        return prompt



    
    def _parse_and_score(self, response: str, prompt: str, confidence_threshold: float) -> List[str]:
        """Parse response and compute confidence score - unified scoring for single and multiple items"""
        print(f"🧠 Model raw response: {response}\n")
        print("-"*60)
        
        response = response.strip()
        
        # Handle single 'skip' case (no scoring needed)
        if response.lower() == 'skip':
            print("⏭️ Model output: skip (single skip)")
            return ['skip']
        
        # Parse items (comma-separated or single)
        if ',' in response:
            print(f"📋 Detected multiple items in response")
            items = [item.strip() for item in response.split(',')]
        else:
            items = [response.strip()]
        
        # Validate all items (aliases are treated as valid if they exist)
        invalid_items = []
        valid_items = []
        for item in items:
            if item.lower() == 'skip':
                valid_items.append('skip')
            elif item in self.available_missions:
                valid_items.append(item)
            elif item in self.mission_aliases:  # ✅ Check if it's a valid alias
                valid_items.append(item)
            else:
                invalid_items.append(item)

        # # Validate all items
        # invalid_items = []
        # valid_items = []
        
        # for item in items:
        #     if item.lower() == 'skip':
        #         valid_items.append('skip')
        #     elif item in self.available_missions:
        #         valid_items.append(item)
        #     else:
        #         invalid_items.append(item)
        
        # if invalid_items:
        #     print(f"⚠️ Invalid items found: {invalid_items}")
        #     return ["CLARIFY:Cannot understand which missions to perform: " + ", ".join(invalid_items)]
        
        # Extract missions to score (exclude 'skip' items)
        mission_items = [item for item in valid_items if item.lower() != 'skip']
        
        # If only skips, return without scoring
        if not mission_items:
            print(f"  Item sequence: {', '.join(valid_items)} (all skips)")
            return [", ".join(valid_items)] if len(valid_items) > 1 else valid_items
        
        # ===== UNIFIED SCORING PHASE =====
        print("-"*60)
        print(f"📊 Mission confidence evaluation:")
        
        # Prepare text to score (comma-separated if multiple, single item otherwise)
        score_text = ", ".join(mission_items)
        
        # Self-Reflection Certainty
        self_ref_certainty = self.query_self_reflection(prompt, score_text, repeats=5)
        
        # Observed Consistency
        encoder_inputs = self.tokenizer(prompt, return_tensors="pt").to(self.device)
        alt_outputs = self.model.generate(
            encoder_inputs.input_ids,
            max_length=100,
            num_return_sequences=7,
            do_sample=True,
            temperature=0.9,
            pad_token_id=self.tokenizer.pad_token_id
        )
        
        alt_plans = [self.tokenizer.decode(seq, skip_special_tokens=True).strip() for seq in alt_outputs]
        
        # Check if all mission items appear in alternative outputs
        agreement = sum(1 for plan in alt_plans if all(mission in plan for mission in mission_items)) / len(alt_plans)
        observed_consistency = agreement
        
        # Combine into final confidence (SAME formula for all)
        beta = 0.8
        confidence = beta * observed_consistency + (1 - beta) * self_ref_certainty
        
        # Print results
        if len(mission_items) > 1:
            print(f" Item sequence: {', '.join(valid_items)}")
        else:
            print(f"  Mission: {mission_items[0]}")
        
        print(f"  🔹 Self-Reflection certainty = {(self_ref_certainty*100):.1f}%")
        print(f"  🔹 Observed Consistency = {(observed_consistency*100):.1f}%")
        print(f"  🔶 Confidence = {(confidence*100):.1f}%")
        print("-"*60)
        
        if confidence < confidence_threshold:
            print("⚠️ Low confidence detected, asking for clarification")
            return ["NEEDS_CLARIFICATION"]
        
        # ✅ EXPAND ALIASES AFTER CONFIDENCE CHECK
        expanded_final = []
        for item in valid_items:
            if item in self.mission_aliases:
                expanded = self.mission_aliases[item]
                print(f"🔄 Expanding validated alias '{item}' -> {expanded}")
                expanded_final.extend(expanded)
            else:
                expanded_final.append(item)
        
        # # Return as comma-separated string for multiple items, single item for single mission
        # return ", ".join(expanded_final) if len(expanded_final) > 1 else expanded_final
        
        # ALWAYS return a list containing a single comma-separated string
        if len(expanded_final) == 1:
            return [expanded_final[0]]  # Returns: ['mission name']
        else:
            return [", ".join(expanded_final)]  # Returns: ['mission1, mission2, mission3']


    
    
    def query_self_reflection(self, step_prompt: str, mission: str, repeats: int = 3) -> float:
        """Ask model follow-up questions about correctness of mission choice"""
        reflection_prompt = f"""Original Prompt:
{step_prompt}

You previously generated this mission: "{mission}".
Question: Was this mission correct, not sure, or incorrect
based on the available missions and command?
Answer only with:
- 1.0 if correct
- 0.5 if not sure
- 0.0 if incorrect
"""
        
        scores = []
        for _ in range(repeats):
            enc = self.tokenizer(reflection_prompt, return_tensors="pt", truncation=True, max_length=128).to(self.device)
            
            with torch.no_grad():
                out_ids = self.model.generate(
                    enc.input_ids,
                    max_length=10,
                    num_return_sequences=1,
                    do_sample=True,
                    temperature=0.3,
                    pad_token_id=self.tokenizer.pad_token_id
                )
            
            response = self.tokenizer.decode(out_ids[0], skip_special_tokens=True).strip()
            
            # Parse numeric value
            try:
                val = float(response)
                if val in (0.0, 0.5, 1.0):
                    scores.append(val)
            except:
                continue
        
        # Default to 0.5 if parsing failed
        if not scores:
            return 0.5
        
        return statistics.median(scores)


class LMPlanner:
    """Main planner coordinating mission memory and action planning"""
    
    def __init__(self, missions_file: Path, memory_file: Path):
        self.planner = ActionPlanner(missions_file)
        self.memory_manager = MissionMemory(memory_file)
        
        # Threading components
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.running = True
        self.planning_thread = None
        
        # Tracking state
        self.last_command = None
    
    def check_mission_in_history(self, mission_name: str) -> bool:
        """Check if a mission was already completed"""
        return self.memory_manager.check_mission_in_history(mission_name)
    
    def plan_command(self, command: str) -> List[str]:
        """Queue command and wait for result"""

        # Reload missions AND aliases from file before each command
        self.planner.available_missions, self.planner.mission_aliases = self.planner._load_missions(MISSIONS_FILE)


        # If this is a new top-level command, set it and reset clarifications
        if self.last_command is None or command != self.last_command:
            self.last_command = command
            self.planner.clarification = None
            self.planner.failure_context = None
        
        # Put the canonical command into the planning queue
        self.command_queue.put(self.last_command)
        
        try:
            result = self.result_queue.get(timeout=30.0)
            
            # Check for clarification mechanism
            if result and isinstance(result, list) and result and isinstance(result[0], str) and result[0].startswith("CLARIFY:"):
                print(f"❓ Model needs clarification: {result[0][8:]}")
                return ["NEEDS_CLARIFICATION"]
            
            return result
        
        except queue.Empty:
            print("⏰ Planning timeout, returning empty plan")
            return []
    
    def add_clarification(self, clarification: str):
        """Add clarification to the current command context"""
        self.planner.clarification = clarification
    
    def add_failure_context(self, failure_context: str):
        """Add failure context to help with replanning"""
        self.planner.failure_context = failure_context
    
    def start(self):
        """Start the planning thread"""
        def planning_loop():
            while self.running:
                try:
                    # Get next command from the queue
                    command = self.command_queue.get(timeout=1.0)
                    
                    # Update memory
                    memory = self.memory_manager.update_memory()
                    
                    # Build full query for context
                    full_query = command
                    if self.planner.clarification:
                        full_query += " Clarification: " + self.planner.clarification
                    if self.planner.failure_context:
                        full_query += " Failure: " + self.planner.failure_context
                    
                    # Generate mission(s)
                    mission_result = self.planner.plan_actions(command, memory)
                    
                    # Put result in queue
                    self.result_queue.put(mission_result)
                
                except queue.Empty:
                    continue
                
                except Exception as e:
                    print(f"❌ Error in planning loop: {e}")
                    self.result_queue.put([])
        
        self.planning_thread = threading.Thread(target=planning_loop, daemon=True)
        self.planning_thread.start()
        
        return self.planning_thread
    
    def stop(self):
        """Stop the planning thread"""
        self.running = False
        if self.planning_thread:
            self.planning_thread.join(timeout=2)


def main():
    """Test the planner standalone"""
    print("=== ROS LM Planner Test ===")
    
    planner = LMPlanner(MISSIONS_FILE, MISSION_MEMORY_FILE)
    planner_thread = planner.start()
    
    print("🚀 Planner started! Enter commands:")
    print("💡 You can use 'skip' in responses (e.g., 'goal_A, skip, goal_B')")
    
    try:
        while True:
            command = input("\n📝 Command: ").strip()
            
            if not command:
                continue
            
            if command.lower() in ['exit', 'quit']:
                break
            
            result = planner.plan_command(command)
            
            if result == ["NEEDS_CLARIFICATION"]:
                print("❓ Clarification needed - provide additional info:")
                clarification = input("📝 Clarification: ").strip()
                planner.add_clarification(clarification)
                result = planner.plan_command(command)
            
            if result:
                print(f"✅ Mission(s) generated: {result}")
                if 'skip' in result[0].lower():
                    print("⏭️ Note: Contains 'skip' instructions")
            else:
                print("❌ No valid mission generated")
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted")
    
    finally:
        planner.stop()


if __name__ == "__main__":
    main()