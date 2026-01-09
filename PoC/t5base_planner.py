#!/usr/bin/env python3
"""
Language Model Planner for robotic mission system.
Uses FLAN-T5-Small to convert natural language commands and mission state
into operations of action strings.
"""

import json
import time
from typing import List, Dict, Any, Optional
from dataclasses import asdict
import threading
import queue
import re
from collections import deque
import math
import torch.nn.functional as F
import statistics

# LangChain and transformers imports
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.vectorstores import FAISS
from langchain_huggingface import HuggingFaceEmbeddings
from langchain.schema import Document
from transformers import T5ForConditionalGeneration, T5Tokenizer
import torch

# Local imports
from state import World
from operations import OPERATIONS

confidence_threshold = 0.5
model_name = "./flan-t5-base_planner_PoC"


class MissionContext:
    """Mission state information"""
    
    def retrieve_context(self, query: str, world_state: World) -> str:
        """Retrieve relevant context for the query"""

        # Add current world state context
        world_context = self._extract_world_context(world_state, query)
        
        context_parts = [world_context]
        
        return "\n".join(context_parts)
    

    def _extract_world_context(self, world: World, query: str) -> str:
        """Extract relevant world state information based on query and trigger words"""

        context_parts = []
        q_lower = query.lower()

        # Extract clarification from query if it contains a clarification marker
        clarification = ""
        if "Clarification:" in query:
            parts = query.split("Clarification:", 1)
            q_lower = parts[0].lower()
            clarification = parts[1].lower()


        # Flags to control pipeline details
        wants_valve = ("valve" in q_lower or "valves" in q_lower)
        wants_ring = ("ring" in q_lower or "rings" in q_lower or "surface" in q_lower)

        # Mapping of trigger words → context appender
        triggers = {
            ("target", "waypoint", "coordinates", "goal", "destination"): lambda: (
                "Target acquired" if world.coordinates.waypoints else "Target not acquired"
            ),
            ("buoy", "buoys", "gate"): lambda: (
                self._get_buoy_context(world)
            ),
            # Pipeline context (also forced if valve/ring are mentioned)
            ("pipeline", "pipe", "pipes", "id", "red", "marker", "markers", "damage", "damaged"): lambda: (
                self._get_pipeline_context(world, include_valve=wants_valve, include_ring=wants_ring)
            ),
            ("main pipe", "marker", "markers", "red", "green", "damage", "damaged", "pipe"): lambda: (
                self._get_main_pipe_context(world)
            ),
            ("interest", "interesting", "area", "surroundings", "survey", "object", "thing", "buoy", "buoys"): lambda: (
                "Potential buoys detected in the buoy area" if world.interesting_points else "No potential buoys detected in the buoy area"
            ),
        }

        # Check triggers in both main query and clarification
        for keywords, context_fn in triggers.items():
            if (any(word in q_lower for word in keywords) or 
                (clarification and any(word in clarification for word in keywords))):
                context = context_fn()
                if context not in context_parts:  # Avoid duplicates
                    context_parts.append(context)

        # Force pipeline context if valve or ring mentioned
        if (wants_valve or wants_ring) and not any("Pipeline" in part for part in context_parts):
            context_parts.append(
                self._get_pipeline_context(world, include_valve=wants_valve, include_ring=wants_ring)
            )

        return " " + " | ".join(context_parts) if context_parts else ""


    def _get_pipeline_context(self, world: World, include_valve: bool = False, include_ring: bool = False) -> str:
        """Helper to format pipeline info with red marker, optionally valve and ring info."""
        parts = []

        if not world.pipelines:
            parts.append("Pipeline not found")
            return " | ".join(parts)

        parts.append("Pipeline found")

        red_marker_found = False
        for pipeline in world.pipelines:
            if not pipeline.pipes:  # Safety check
                continue

            for pipe in pipeline.pipes:
                if hasattr(pipe, 'markers'):  # Safety check
                    for marker in pipe.markers:
                        if marker.color.lower() == "red" and not red_marker_found:
                            red_marker_found = True
                            parts.append("Damaged pipe found")
                            if include_valve:
                                parts.append(
                                    f"Valve {'closed' if getattr(pipe, 'valve_closure', 0) else 'not closed'}"
                                )
                            if include_ring:
                                parts.append(
                                    f"Ring {getattr(pipe, 'ring', 'untouched')}"
                                )

        if not red_marker_found:
            parts.append("Damaged pipe not found")

        return " | ".join(parts)
    
    def _get_buoy_context(self, world: World) -> str:
        """Helper to format buoy info"""
        if world.buoys:
            yellow_buoys = [b for b in world.buoys if b.color.lower() == "yellow"]
            parts = []
            if len(yellow_buoys) == 2:
                parts.append("2 yellow buoys: gate found")
            elif len(yellow_buoys) == 1:
                parts.append("1 yellow buoy found, 1 missing: gate partial")
            else:
                parts.append("0 of 2 yellow buoys: gate not found")
            return " | ".join(parts)
        else:
            return "0 of 2 yellow buoys: gate not found"


    def _get_main_pipe_context(self, world: World) -> str:
        """Helper to format main pipe info with markers info."""
        parts = []

        # Check if main pipe exists and has end position
        if not hasattr(world, 'main_pipe') or not world.main_pipe.end_position:
            parts.append("Main pipe not found")
            return " | ".join(parts)

        parts.append("Main pipe found")
        
        # Check markers
        if hasattr(world.main_pipe, 'markers') and world.main_pipe.markers:
            parts.append(f"Main pipe markers found")
            # marker_colors = [m.color for m in world.main_pipe.markers]
            # parts.append(f"Markers found: {', '.join(marker_colors)}")
        else:
            parts.append("No main pipe markers found")

        return " | ".join(parts)


class ActionPlanner:
    def __init__(self):
        # Set the path to your local model directory
        self.model_name = model_name

        # Load the tokenizer and model from the local path
        self.tokenizer = T5Tokenizer.from_pretrained(self.model_name)
        self.model = T5ForConditionalGeneration.from_pretrained(self.model_name)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        
        # Available actions for validation
        self.available_operations = list(OPERATIONS.keys())

        self.last_command = None
        self.clarification = None
    
    def plan_actions(self, command: str, context: str) -> List[str]:
        """Generate action operation from command and context"""
        try:
            if self.last_command != command:
                # New command - reset clarification
                self.last_command = command
                self.clarification = None

            # Create prompt for FLAN-T5
            prompt = self._create_prompt(command, context)
            print(f"🖊️ Model prompt: {prompt}\n")
            
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

            # ---------- Parse and score ----------
            actions = self._parse_and_score(raw_response, prompt, confidence_threshold)

            return actions

        except Exception as e:
            print(f"❌ Error in plan_actions: {e}")
            return []


    def _create_prompt(self, command: str, context: str) -> str:
        """
        Create prompt for FLAN-T5 with proper clarification handling.
        If a clarification is in progress, append the previous output.
        """
        available_operations_str = ", ".join(self.available_operations)
        
        prompt = f"""Available operations: {available_operations_str}

Current knowledge: {context}

Command: {command}"""

        # Add clarification section if we have clarifications
        if self.clarification:
            # Include previous output if clarification is triggered
            if hasattr(self, 'last_output') and self.last_output:
                prompt += f"\n\nPrevious response: {self.last_output}"
                
            prompt += f"\n\nClarification: {self.clarification}"
            prompt += "\n\nInstruction: Generate a numbered list of operations to accomplish this command, pay attention to the current knowledge of the environment, your previous response and the clarification for more info"
        else:
            prompt += "\n\nInstruction: Generate a numbered list of operations to accomplish this command, pay attention to the current knowledge of the environment:"
        
        return prompt


    def _parse_response(self, response: str) -> List[str]:
        """Parse FLAN-T5 response to extract clean operations (no confidences)."""
        print(f"🧠 Model raw response: {response}\n")

        response = response.strip()

        # If the model didn’t produce a numbered plan, treat as clarification
        if not response.lower().startswith("plan:"):
            print("⚠️ Response appears to be a clarification request")
            return ["CLARIFY:" + response]

        # Remove "Plan:" prefix
        response = response[len("Plan:"):].strip()

        # Regex to capture lines like '1. some operation'
        step_pattern = re.compile(r"\d+\.\s*(.*?)(?=\d+\.|$)", re.DOTALL)
        actions = step_pattern.findall(response)

        if not actions:
            print("⚠️ Response not in proper numbered list format")
            return ["CLARIFY:" + response]

        # Clean up whitespace, keep original case
        actions = [a.strip() for a in actions if a.strip()]

        # Validate against available operations
        valid_actions = []
        for action in actions:
            if action in self.available_operations:
                valid_actions.append(action)
            else:
                print(f"⚠️ Invalid operation found: '{action}'")
                return ["CLARIFY:Cannot understand how to perform: " + action]

        return valid_actions


    def _parse_and_score(self, response: str, prompt: str, confidence_threshold: float) -> List[str]:
        """
        Parse the raw model response into clean operations,
        compute self-reflection certainty (via follow-up questions)
        and observed consistency (resampling),
        then combine them into a final confidence score for each step.
        """
        print(f"🧠 Model raw response: {response}\n")
        response = response.strip()

        # Remove "Plan:" prefix
        if response.lower().startswith("plan:"):
            response = response[len("Plan:"):].strip()

        # Parse numbered steps
        step_pattern = re.compile(r"\d+\.\s*(.*?)(?=\d+\.|$)", re.DOTALL)
        actions = step_pattern.findall(response)
        actions = [a.strip() for a in actions if a.strip()]

        # Validate against available operations
        for op in actions:
            if op not in self.available_operations:
                print(f"⚠️ Invalid operation found: '{op}'")
                return ["CLARIFY:Cannot understand how to perform: " + op]

        # --------- SCORING SECTION ----------
        print("📊 Step-wise confidence evaluation:")
        low_confidence_detected = False
        scored_ops = []
        prev_steps_text = ""

        for i, op in enumerate(actions):
            # Construct prompt including previous steps
            if i == 0:
                step_prompt = prompt
            else:
                step_prompt = f"{prompt}\nPlan: "
                for j in range(i):
                    step_prompt += f"{j+1}. {actions[j]} "

            # --- Method A: Self-Reflection Certainty (per paper) ---
            self_ref_certainty = self.query_self_reflection(step_prompt, op, repeats=5)

            # --- Method B: Observed Consistency ---
            encoder_inputs = self.tokenizer(step_prompt, return_tensors="pt").to(self.device)
            alt_outputs = self.model.generate(
                encoder_inputs.input_ids,
                max_length=50,
                num_return_sequences=7,
                do_sample=True,
                temperature=0.9,
                pad_token_id=self.tokenizer.pad_token_id
            )
            alt_plans = [self.tokenizer.decode(seq, skip_special_tokens=True) for seq in alt_outputs]
            agreement = sum(1 for plan in alt_plans if op in plan) / len(alt_plans)
            observed_consistency = agreement

            # --- Combine into final confidence (fixed β = 0.7 as in paper) ---
            beta = 0.7
            confidence = beta * observed_consistency + (1 - beta) * self_ref_certainty

            # Log step info
            print(f"  Step {i+1}: {op}")
            print(f"     🔹 Self-Reflection certainty = {(self_ref_certainty*100):.1f}%")
            print(f"     🔹 Observed Consistency      = {(observed_consistency*100):.1f}%")
            print(f"     🔶 Confidence = {(confidence*100):.1f}%")

            if confidence < confidence_threshold:
                low_confidence_detected = True

            scored_ops.append(op)
            prev_steps_text += f"{i+1}. {op} "

        if low_confidence_detected:
            print("❓ Low confidence detected, asking for clarification")
            return ["NEEDS_CLARIFICATION"]

        return scored_ops


    # --- Method A: Self-Reflection Certainty (per paper) ---
    def query_self_reflection(self, step_prompt: str, op: str, repeats: int = 3) -> float:
        """
        Ask the model follow-up questions about the correctness of an operation.
        Collect multiple responses and return the median score.
        """
        reflection_prompt = f"""
    You previously generated this plan step: "{op}".

    Question: Was this step correct, not sure, or incorrect
    based on the available operations and current knowledge?

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
                    temperature=0.7,
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
    def __init__(self, world: World):
        self.world = world
        self.planner = ActionPlanner()
        self.context_manager = MissionContext()
        
        # Threading components
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.running = True
        self.planning_thread = None
        
        # Tracking state
        self.last_command = None
        
    def plan_command(self, command: str) -> List[str]:
        """
        Queue the canonical (original) command and wait for a result.
        This method assumes the caller (main loop) decides whether an incoming
        chat message is a top-level command or a clarification.
        """
        # If this is a new top-level command, set it and reset clarifications.
        if self.last_command is None or command != self.last_command:
            self.last_command = command
            self.planner.clarification = None

        # Put the canonical command (original top-level) into the planning queue
        self.command_queue.put(self.last_command)

        try:
            result = self.result_queue.get(timeout=20.0)

            # ---------------- CONFIDENCE-BASED UNCERTAINTY ----------------
            import re
            if result and isinstance(result, list):
                confidences = []
                for step in result:
                    match = re.search(r"\((0\.\d+)\)", step)
                    if match:
                        confidences.append(float(match.group(1)))
                # If any per-step confidence below threshold (shouldn't happen because we used plan-level),
                # treat as needing clarification
                if confidences and any(c < confidence_threshold for c in confidences):
                    print("❓ Low confidence detected, asking for clarification")
                    return ["NEEDS_CLARIFICATION"]

            # ---------------- FORMAT CLARIFICATION MECHANISM ----------------
            if result and isinstance(result, list) and result and isinstance(result[0], str) and result[0].startswith("CLARIFY:"):
                print(f"❓ Model needs clarification: {result[0][8:]}")
                return ["NEEDS_CLARIFICATION"]

            return result

        except queue.Empty:
            print("⏰ Planning timeout, returning empty plan")
            return []


    def add_clarification(self, clarification: str):
        """Add clarification to the current command context"""
        # Clear old clarification before writing new one
        self.planner.clarification = clarification

    def start(self):
        """Start the planning thread (works with current chat system)."""
        def planning_loop():
            while self.running:
                try:
                    # Get next command from the queue
                    command = self.command_queue.get(timeout=1.0)
                    
                    # Build full query for context extraction (include clarification internally)
                    full_query = command
                    if self.planner.clarification:
                        full_query += " Clarification: " + self.planner.clarification
                    
                    context = self.context_manager.retrieve_context(full_query, self.world)
                    
                    # Generate actions
                    action_operation = self.planner.plan_actions(command, context)

                    # Put result in queue
                    self.result_queue.put(action_operation)
                    
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
    """Run the LM planner with lightweight chat integration."""

    COMMAND_FILE = Path("command.json")

    planner = LMPlanner(world)
    planner_thread = planner.start()

    print("🚀 LM Planner system started! Waiting for commands...")
    recent_msgs = deque(maxlen=200)

    pending_command = None
    waiting_for_clarification = False

    try:
        last_command_data = {}
        while True:
            # Read latest command.json
            try:
                if COMMAND_FILE.exists():
                    with open(COMMAND_FILE, 'r') as f:
                        cmd_data = json.load(f)
                else:
                    cmd_data = {}
            except json.JSONDecodeError:
                cmd_data = {}

            command = cmd_data.get("command", "").strip()
            processed = cmd_data.get("processed", True)

            # Skip if already processed or duplicate
            if not command or processed or command in recent_msgs:
                time.sleep(0.05)
                continue

            recent_msgs.append(command)

            # ---------- CASE: waiting for clarification ----------
            if waiting_for_clarification and pending_command:
                print(f"📝 Consuming as clarification for: '{pending_command}'")
                planner.add_clarification(command)
                action_operation = planner.plan_command(pending_command)

                waiting_for_clarification = False
                pending_command = None

                if action_operation == ["NEEDS_CLARIFICATION"]:
                    print("❓ Model still needs clarification. Waiting for next input.")
                    waiting_for_clarification = True
                    pending_command = planner.last_command
                elif action_operation:
                    print(f"✅ Action operation generated: {action_operation}")
                else:
                    print("❌ No valid action operation generated")

            # ---------- CASE: treat as new top-level command ----------
            else:
                pending_command = command
                action_operation = planner.plan_command(pending_command)

                if action_operation == ["NEEDS_CLARIFICATION"]:
                    print("❓ Clarification needed - type additional info (next message will be consumed as clarification).")
                    waiting_for_clarification = True
                    # pending_command stays set
                elif action_operation:
                    print(f"✅ Action operation generated: {action_operation}")
                    pending_command = None
                else:
                    print("❌ No valid action operation generated")
                    pending_command = None

            # Mark the command as processed
            cmd_data["processed"] = True
            cmd_data["timestamp"] = time.time()
            with open(COMMAND_FILE, 'w') as f:
                json.dump(cmd_data, f)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n🛑 System interrupted by user")
    finally:
        planner.stop()



if __name__ == "__main__":
    main()