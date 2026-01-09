#!/usr/bin/env python3

"""
ROS Mission Manager
"""

import time
import json
import yaml
import threading
from typing import Optional, List
from pathlib import Path
from flask import Flask, request, jsonify
from datetime import datetime

from planner import LMPlanner

from ros_connection import send_cmd_to_ros
from ros_connection import get_latency_tracker

from logs_processor import extract_mission_events

import argparse
from resource_monitor import ResourceMonitor


# File paths
MISSIONS_FILE = Path("missions.yaml")
MEMORY_FILE = Path("mission_memory.yaml")
MISSION_LOGS_FILE = Path("mission_logs.yaml")
COMMAND_FILE = Path("command.json")
STATUS_FILE = Path("status.json")
LOG_FILE = Path("mission_memory_updates.log")
MISSION_HISTORY_FILE = Path("mission_history.yaml")

# Flask app for receiving mission memory updates
app = Flask(__name__)
LISTEN_HOST = '0.0.0.0'
LISTEN_PORT = 5002

class ROSMissionManager:
    """Main manager coordinating planner and ROS communication"""
    
    def __init__(self, enable_monitoring=False):
        # File communication
        self.command_file = COMMAND_FILE
        self.status_file = STATUS_FILE
        self.memory_file = MEMORY_FILE
        self.mission_logs_file = MISSION_LOGS_FILE  # NEW
        self.missions_file = MISSIONS_FILE
        self.last_model_output = None  # Store last model output for duplicate handling
        self.duplicate_context = False  # Flag to track if we're in duplicate clarification mode
        self.bypass_duplicate_check = False  # Flag to bypass duplicate check after user confirms

        
        # Wipe memory from previous session
        if self.memory_file.exists():
            print(f"🧹 Wiping existing mission memory: {self.memory_file}")
            self.memory_file.unlink()
            print("✅ Mission memory cleared")
        
        # NEW: Wipe mission logs from previous session
        if self.mission_logs_file.exists():
            print(f"🧹 Wiping existing mission logs: {self.mission_logs_file}")
            self.mission_logs_file.unlink()
            print("✅ Mission logs cleared")
        
        # Wipe mission history file if it exists
        if MISSION_HISTORY_FILE.exists():
            print(f"🧹 Wiping existing mission history: {MISSION_HISTORY_FILE}")
            MISSION_HISTORY_FILE.unlink()
            print("✅ Mission history cleared")
        
        # Initialize planner
        print("🔧 Initializing LM Planner...")
        self.planner = LMPlanner(self.missions_file, self.memory_file)
        
        # System state
        self.running = True
        self.mission_counter = 0
        self.current_mission = None
        self.mission_thread = None
        
        # Clarification state
        self.waiting_for_clarification = False
        self.pending_command = None
        self.failed_mission_name = None
        self.remaining_missions = []
        
        # Duplicate mission confirmation state
        self.waiting_for_duplicate_confirmation = False
        self.duplicate_mission_name = None
        self.duplicate_mission_index = None
        
        # Sequential mission execution state
        self.mission_queue = []  # List of mission names to execute
        self.current_mission_index = 0  # Index in the queue
        self.waiting_for_mission_completion = False
        self.last_mission_status = None

        # self.mission_status_received = threading.Event()
        # self.mission_status_lock = threading.Lock()
        self.mission_status_condition = threading.Condition()
        self.mission_completed = False
        
        # Load missions
        self.missions = self._load_missions()
        
        # Initialize communication files
        self._initialize_files()
        
        # Store global reference for Flask routes
        global manager_instance
        manager_instance = self

        self.enable_monitoring = enable_monitoring
        self.resource_monitor = None
        if self.enable_monitoring:
            print("📊 Resource monitoring ENABLED")
            self.resource_monitor = ResourceMonitor(
                log_file="resource_usage.json",
                interval=1.0
            )
    
    def _log_mission_to_history(self, mission_name: str):
        """Log a mission to the mission history file (excludes stop_mission)"""
        try:
            # MODIFICATION: Don't log stop_mission to history
            if mission_name.lower() == "stop_mission":
                print(f"⏭️  Skipping history log for: {mission_name}")
                return
            
            # Read existing history
            if MISSION_HISTORY_FILE.exists():
                with open(MISSION_HISTORY_FILE, 'r') as f:
                    history_data = yaml.safe_load(f) or {}
            else:
                history_data = {}
            
            # Initialize missions list if not exists
            if 'missions' not in history_data:
                history_data['missions'] = []
            
            # Add new mission entry with timestamp
            mission_entry = {
                'mission_name': mission_name,
                'timestamp': time.time(),
                'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            history_data['missions'].append(mission_entry)
            
            # Write back to file
            with open(MISSION_HISTORY_FILE, 'w') as f:
                yaml.dump(history_data, f, default_flow_style=False)
            
            print(f"📝 Logged to history: {mission_name}")
            
        except Exception as e:
            print(f"⚠️  Error logging mission to history: {e}")
    
    def _check_duplicate_mission(self, mission_name: str) -> bool:
        """Check if mission was already completed"""
        return self.planner.check_mission_in_history(mission_name)
    
    def _load_missions(self):
        """Load missions from YAML file"""
        try:
            if self.missions_file.exists():
                with open(self.missions_file, 'r') as f:
                    return yaml.safe_load(f)
        except Exception as e:
            print(f"⚠️  Error loading missions.yaml: {e}")
            return {}
        return {}
    
    def _initialize_files(self):
        """Initialize communication files"""
        print("🔧 Initializing communication files...")
        
        initial_command = {
            "command": None,
            "timestamp": time.time(),
            "processed": True
        }

        self._write_command_file(initial_command)
        print(f"✅ Command file initialized: {self.command_file}")
        
        # Status file
        initial_status = {
            "status": "initializing",
            "processing": False,
            "timestamp": time.time()
        }
        self._write_status_file(initial_status)
        print(f"✅ Status file initialized: {self.status_file}")
        
        # Memory file (create if doesn't exist)
        if not self.memory_file.exists():
            initial_memory = {"buoys_found": []}
            with open(self.memory_file, 'w') as f:
                yaml.dump(initial_memory, f)
            print(f"✅ Memory file initialized: {self.memory_file}")
        
        # NEW: Mission logs file (create if doesn't exist)
        if not self.mission_logs_file.exists():
            initial_logs = {"missions": []}
            with open(self.mission_logs_file, 'w') as f:
                yaml.dump(initial_logs, f)
            print(f"✅ Mission logs file initialized: {self.mission_logs_file}")
    
    def _write_command_file(self, data):
        """Write command data to file"""
        try:
            with open(self.command_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            print(f"Error writing command file: {e}")
    
    def _write_status_file(self, data):
        """Write status data to file"""
        try:
            with open(self.status_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            print(f"Error writing status file: {e}")
    
    def _read_command_file(self):
        """Read command from file"""
        try:
            if self.command_file.exists():
                with open(self.command_file, 'r') as f:
                    return json.load(f)
        except (json.JSONDecodeError, FileNotFoundError) as e:
            print(f"⚠️  Warning: Error reading command file: {e}")
        except Exception as e:
            print(f"❌ Error reading command file: {e}")
        
        return {"command": None, "timestamp": time.time(), "processed": True}
    
    def handle_mission_status_update(self, mission_number, status_code, status_text):
        """Handle mission status update from bridge"""

        # print(f"[DEBUG MGR STATUS] Called: mission={mission_number}, status={status_code}, text={status_text}")
        # print(f"[DEBUG MGR STATUS] Current state: mission_counter={self.mission_counter}, "
        #     f"last_mission_status={self.last_mission_status}")

        # with self.mission_status_lock:
        with self.mission_status_condition:
            # Check if this is a duplicate status (same mission_number and status_code)
            if self.last_mission_status:
                if (self.last_mission_status["mission_number"] == mission_number and 
                    self.last_mission_status["status_code"] == status_code):

                    print(f"[DEBUG MGR DUP-CHECK] Duplicate detected - same mission & status, returning early")
                
                    # Duplicate status - ignore silently
                    return
            
            # Store the new status
            self.last_mission_status = {
                "mission_number": mission_number,
                "status_code": status_code,
                "status_text": status_text
            }
        
            # Print clear status information (only for first occurrence)
            print("=" * 60)
            print("📊 MISSION STATUS UPDATE")
            print("=" * 60)
            print(f"Mission Number: #{mission_number}")
            print(f"Status Code: {status_code}")
            print(f"Status: {status_text}")
            print("=" * 60)

            # print(f"[DEBUG MGR SIGNAL] Setting mission_status_received event for mission={mission_number}, status={status_code}")
            
            # Signal that status was received (only once)
            # self.mission_status_received.set()
            self.mission_completed = True  # Replace .set()
            self.mission_status_condition.notify_all()  # Replace .set()



    
    def start(self):
        """Start the complete system"""
        print("🚀 Starting ROS Mission Manager")
        print("=" * 60)
        
        # Start resource monitoring if enabled
        if self.enable_monitoring and self.resource_monitor:
            self.resource_monitor.start_monitoring()

        # Start planner thread
        planner_thread = self.planner.start()
        
        # Start Flask listener in separate thread
        listener_thread = threading.Thread(target=self._start_listener, daemon=True)
        listener_thread.start()
        
        # Signal ready
        self._write_status_file({
            "status": "ready",
            "processing": False,
            "timestamp": time.time()
        })
        
        print("✅ All subsystems started successfully")
        print("🎯 System ready for mission commands from chat interface")
        print("💬 Start the chat interface in another terminal with: python chat.py")
        print("🔄 Waiting for commands... (Press Ctrl+C to shutdown)")
        print("=" * 60)
        
        # Main system loop
        last_processed_timestamp = 0
        idle_counter = 0
        
        try:
            while self.running:
                # Read latest command
                command_data = self._read_command_file()
                command = command_data.get("command")
                processed = command_data.get("processed", True)
                command_timestamp = command_data.get("timestamp", 0)
                
                # Process new unprocessed commands
                if command and not processed and command_timestamp > last_processed_timestamp:
                    idle_counter = 0
                    last_processed_timestamp = command_timestamp
                    cmd_lower = command.strip().lower()
                    
                    # Shutdown command
                    if command.strip().upper() == "SHUTDOWN":
                        print("🛑 Shutdown command received")
                        self.running = False
                        break
                    
                    # Stop keywords interrupt mission
                    if cmd_lower in ["exit", "quit"]:
                        if self.current_mission and self.mission_thread and self.mission_thread.is_alive():
                            print(f"⏹️  Mission interrupted by stop command: '{command}'")
                            # Send stop command (mission_tag=4)
                            self._stop_current_mission()
                            self.current_mission = None
                            self.mission_queue = []
                            self.waiting_for_clarification = False
                            self.waiting_for_duplicate_confirmation = False
                            self.pending_command = None
                            self.failed_mission_name = None
                            self.remaining_missions = []
                            self.duplicate_mission_name = None
                            self.duplicate_mission_index = None
                            self._mark_command_processed()
                            continue
                
                    
                    # Mark as processing
                    self._mark_command_as_processing()
                    
                    # # Handle duplicate mission confirmation
                    # if self.waiting_for_duplicate_confirmation and self.pending_command:
                    #     cmd_lower = command.strip().lower()
                        
                    #     if cmd_lower in ["yes", "y", "repeat", "confirm"]:
                    #         print(f"✅ Operator confirmed: Repeating mission '{self.duplicate_mission_name}'")
                    #         self.waiting_for_duplicate_confirmation = False
                            
                    #         # Continue with mission execution from duplicate index
                    #         self._mark_command_processed()
                    #         self._write_status_file({
                    #             "status": "processing",
                    #             "processing": True,
                    #             "timestamp": time.time()
                    #         })
                            
                    #         # Continue executing from the duplicate mission
                    #         self._continue_mission_execution_from_index(self.duplicate_mission_index)
                    #         continue
                        
                    #     elif cmd_lower in ["no", "n", "skip", "cancel"]:
                    #         print(f"❌ Operator declined: Skipping mission '{self.duplicate_mission_name}'")
                    #         self.waiting_for_duplicate_confirmation = False
                            
                    #         # Skip this mission and continue with the next
                    #         self._mark_command_processed()
                    #         self._write_status_file({
                    #             "status": "processing",
                    #             "processing": True,
                    #             "timestamp": time.time()
                    #         })
                            
                    #         # Continue from next mission
                    #         self._continue_mission_execution_from_index(self.duplicate_mission_index + 1)
                    #         continue
                    #     else:
                    #         print(f"⚠️  Invalid response. Please respond with 'yes' or 'no'")
                    #         self._mark_command_processed()
                    #         continue
                    
                    # handles both normal and failure-based clarification
                    if self.waiting_for_clarification and self.pending_command:
                        cmd_lower = command.strip().lower()
                        
                        if cmd_lower in ["stop clarification", "cancel clarification"]:
                            print("🛑 Clarification request cancelled")
                            self.waiting_for_clarification = False
                            self.pending_command = None
                            self.failed_mission_name = None
                            self.remaining_missions = []
                            if hasattr(self, '_failure_replanning_context'):
                                delattr(self, '_failure_replanning_context')
                            self._mark_command_processed()
                            self._write_status_file({
                                "status": "ready",
                                "processing": False,
                                "timestamp": time.time()
                            })
                            continue
                        
                        # ✨ Check if this is failure-based clarification
                        if hasattr(self, '_failure_replanning_context'):
                            # This is clarification for a failed mission replanning
                            ctx = self._failure_replanning_context
                            failure_context = ctx["failure_context"]
                            mission_name = ctx["mission_name"]
                            
                            print(f"📝 Received clarification: '{command}'")
                            
                            # Add ONLY the user's clarification to the planner
                            self.planner.add_clarification(command)
                            
                            print(f"🔄 Retrying replanning with failure context + clarification...")
                            
                            # Replan with failure context + user clarification
                            result = self.planner.plan_command(self.pending_command)
                            
                            if not result or result == ["NEEDS_CLARIFICATION"]:
                                print("⚠️ Still uncertain after clarification. Please provide more specific guidance.")
                                self._mark_command_processed()
                                continue
                            
                            # Parse new plan
                            mission_output = result[0]
                            raw_missions = [m.strip() for m in mission_output.split(",")]
                            
                            if all(m.lower() == "skip" for m in raw_missions):
                                print("All replanned missions are 'skip' - cancelling remaining missions")
                                self.waiting_for_clarification = False
                                self.pending_command = None
                                self.failed_mission_name = None
                                self.remaining_missions = []
                                delattr(self, '_failure_replanning_context')
                                self._mark_command_processed()
                                self._write_status_file({
                                    "status": "ready",
                                    "processing": False,
                                    "timestamp": time.time()
                                })
                                continue
                            
                            # Update mission queue with clarified plan
                            self.mission_queue = [m for m in raw_missions if m.lower() != "skip"]
                            
                            print(f"✅ New plan after clarification: {self.mission_queue}")
                            
                            # Clear failure state
                            self.waiting_for_clarification = False
                            self.pending_command = None
                            self.failed_mission_name = None
                            self.remaining_missions = []
                            delattr(self, '_failure_replanning_context')
                            
                            self._mark_command_processed()
                            self._write_status_file({
                                "status": "processing",
                                "processing": True,
                                "timestamp": time.time()
                            })
                            
                            # Continue mission execution from start of new queue
                            self._continue_mission_execution_from_index(0)
                            continue
                        
                        else:
                            # Regular clarification (not failure-based) - normal planning path
                            print(f"📝 Received clarification: '{command}'")
                            
                            # Check if this was a duplicate mission clarification
                            if getattr(self, 'duplicate_context', False):
                                print(f"🔄 Handling duplicate mission clarification - will bypass duplicate check on next execution")
                                
                                # For duplicate mission clarification, append user input to the duplicate prompt
                                # instead of replacing it
                                # duplicate_msg = f"Repeat mission: The mission '{self.duplicate_mission_name}' was already completed. User response: {command}"
                                duplicate_msg = f"Repeat mission: {command}"
                                self.planner.add_clarification(duplicate_msg)
                                
                                self.bypass_duplicate_check = True
                                self.duplicate_context = False  # Clear the flag
                            else:
                                # Regular clarification - just add the user's input
                                self.planner.add_clarification(command)
                            
                            self._mark_command_processed()
                            
                            # Restart mission execution
                            self.current_mission = self.pending_command
                            self.mission_thread = threading.Thread(
                                target=self._execute_mission_thread,
                                args=(self.pending_command,),
                                daemon=True
                            )
                            self.mission_thread.start()
                            continue




                    # Clear only the duplicate/clarification states - don't stop current mission
                    self.waitingforclarification = False
                    self.waitingforduplicateconfirmation = False
                    self.pendingcommand = None
                    self.failedmissionname = None
                    self.remainingmissions = []
                    self.duplicatemissionname = None
                    self.duplicatemissionindex = None

                    # Command will go through planner even if mission is already running
                    print(f"New command received, sending through planner: {command}")
                    
                    # Start new mission
                    self.current_mission = command
                    self.mission_thread = threading.Thread(
                        target=self._execute_mission_thread,
                        args=(command,),
                        daemon=True
                    )
                    self.mission_thread.start()
                
                else:
                    # System idle
                    idle_counter += 1
                    if idle_counter % 600 == 0:  # Print every minute
                        if self.waiting_for_clarification:
                            print(f"⏳ Waiting for clarification for failed mission...")
                        elif self.waiting_for_duplicate_confirmation:
                            print(f"⏳ Waiting for operator confirmation on duplicate mission...")
                        else:
                            print("💤 System idle - waiting for commands...")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n🛑 System shutdown initiated")
        finally:
            self._shutdown_system(planner_thread)
    
    def _stop_current_mission(self):
        """Stop the currently executing mission by sending mission_tag=4"""
        print("🛑 Sending STOP command (mission_tag=4) to current mission...")
        
        # Create stop command with mission_tag=4
        stop_command = {
            "mission_name": "STOP",
            "mission_id": self.mission_counter,
            "mission_tag": 4,  # Tag 4 means STOP
            "latitude": 0.0,
            "longitude": 0.0,
            "depth": 0.0,
            "yaw": 0.0,
            "radius": 0.0,
            "use_yaw": False,
            "use_spiral": False,
            "color": ""
        }
        
        # Send to ROS bridge
        response = send_cmd_to_ros(stop_command, "/send_mission")
        if response.get("status") == "success":
            print("✅ STOP command sent to ROS successfully")
        else:
            print(f"❌ Failed to send STOP command: {response.get('message')}")
    
    def _execute_mission_thread(self, command: str):
        """Execute mission in separate thread"""
        try:
            self._write_status_file({
                "status": "processing",
                "processing": True,
                "timestamp": time.time()
            })
            
            success = self._execute_mission(command)
            
            if not self.waiting_for_clarification and not self.waiting_for_duplicate_confirmation:
                self._mark_command_processed()
        
        except Exception as e:
            print(f"❌ Mission execution error: {e}")
            if not self.waiting_for_clarification and not self.waiting_for_duplicate_confirmation:
                self._mark_command_processed()
        
        finally:
            if not self.waiting_for_clarification and not self.waiting_for_duplicate_confirmation:
                self._write_status_file({
                    "status": "ready",
                    "processing": False,
                    "timestamp": time.time()
                })
                self.current_mission = None
    
    def _execute_mission(self, command: str):
        """Execute complete mission with sequential handling"""
        # self.mission_counter += 1
        print(f"\n🎯 MISSION REQUEST #{self.mission_counter}")
        print(f"📝 Command: '{command}'")
        print("-" * 40)
        
        # Step 1: Plan mission using LM
        print("🧠 Step 1: Planning mission with LM...")
        mission_result = self.planner.plan_command(command)

        # Store the model's last output for potential duplicate handling
        if hasattr(self.planner.planner, 'last_output'):
            self.last_model_output = self.planner.planner.last_output
        else:
            self.last_model_output = None

        
        if mission_result == ["NEEDS_CLARIFICATION"]:
            print("❓ Please provide clarification through the chat interface")
            self.waiting_for_clarification = True
            self.pending_command = command
            
            self._write_status_file({
                "status": "waiting_clarification",
                "pending_command": command,
                "message": "Planner requires clarification to proceed",
                "processing": False,
                "timestamp": time.time()
            })
            return True
        
        if not mission_result or len(mission_result) == 0:
            print("❌ Failed to generate mission")
            return False
        
        # Parse mission result - can be single or comma-separated list
        mission_output = mission_result[0]
        print(f"✅ Planner output: {mission_output}")
        
        # Parse comma-separated missions and filter out 'skip'
        raw_missions = [m.strip() for m in mission_output.split(',')]
        
        # Check if all missions are 'skip'
        if all(m.lower() == 'skip' for m in raw_missions):
            print("⏭️  All missions are 'skip' - nothing to execute")
            return True
        
        # Filter out 'skip' missions
        self.mission_queue = [m for m in raw_missions if m.lower() != 'skip']
        
        if not self.mission_queue:
            print("⏭️  No valid missions to execute after filtering 'skip'")
            return True
        
        print(f"📋 Mission queue ({len(self.mission_queue)} missions): {self.mission_queue}")
        
        # Clear clarification state
        self.waiting_for_clarification = False
        self.pending_command = None
        self.failed_mission_name = None
        self.remaining_missions = []
        
        # Execute missions sequentially starting from index 0
        return self._continue_mission_execution_from_index(0)
    
    def _continue_mission_execution_from_index(self, start_index: int):
        """Continue mission execution from a specific index"""
        # Execute missions sequentially
        for idx in range(start_index, len(self.mission_queue)):
            mission_name = self.mission_queue[idx]
            self.current_mission_index = idx
            
            # Check if mission was interrupted
            if self.current_mission is None:
                print("⏹️  Mission sequence interrupted")
                return False
            
            print(f"\n{'='*60}")
            print(f"🚀 Executing Mission {idx+1}/{len(self.mission_queue)}: {mission_name}")
            print(f"{'='*60}")
            
            
            # MODIFICATION: Check if mission was already completed (unless bypassing)
            if not getattr(self, 'bypass_duplicate_check', False) and self._check_duplicate_mission(mission_name):
                print(f"⚠️ Mission '{mission_name}' was already completed!")
                print(f"🔄 Prompting model with duplicate notification...")
                
                # Set clarification state (reusing existing mechanism)
                self.waiting_for_clarification = True
                self.pending_command = self.current_mission
                self.duplicate_mission_name = mission_name
                self.duplicate_mission_index = idx
                
                # Build duplicate prompt with previous response
                duplicate_prompt = f"Repeat mission: The mission '{mission_name}' was already completed. Please provide a different command or confirm if you want to repeat this mission."
                
                # Add clarification to planner (this will include previous response)
                self.planner.add_clarification(duplicate_prompt)
                
                # Mark that we're handling a duplicate - next execution should bypass duplicate check
                self.duplicate_context = True
                
                self._write_status_file({
                    "status": "waiting_clarification",
                    "pending_command": self.current_mission,
                    "duplicate_mission": mission_name,
                    "message": f"Mission '{mission_name}' was already completed. Awaiting user input in chat.",
                    "processing": False,
                    "timestamp": time.time()
                })
                
                # Return and wait for user input
                return True

            # If we're bypassing duplicate check for this mission, reset the flag
            if getattr(self, 'bypass_duplicate_check', False):
                print(f"✅ Bypassing duplicate check - executing '{mission_name}' as requested")
                self.bypass_duplicate_check = False


            # print(f"[DEBUG MGR CLEAR] Clearing mission_status_received for next mission "
            #   f"(current mission_counter={self.mission_counter})")
            
            # Reset status event
            # self.mission_status_received.clear()
            # self.last_mission_status = None
            with self.mission_status_condition:
                self.mission_completed = False
                self.last_mission_status = None

            
            # Send mission to ROS
            success = self._send_mission_to_ros(mission_name)
            
            if not success:
                print(f"❌ Failed to send mission '{mission_name}' to ROS")
                return False
            
            print(f"⏳ Waiting for mission completion status...")

            # print(f"[DEBUG MGR WAIT] Calling .wait(timeout=1800s) for mission_counter={self.mission_counter}")
            
            # Wait for mission completion status (with timeout)
            # status_received = self.mission_status_received.wait(timeout=1800.0)  # 30 minutes timeout

            # print(f"[DEBUG MGR WAIT-DONE] .wait() returned={status_received}, "
            #   f"received_status={self.last_mission_status}")
            
            # if not status_received:
            #     print(f"⚠️  Timeout waiting for mission status")
            #     return False
            
            # # Check mission status
            # with self.mission_status_lock:
            #     if self.last_mission_status:
            #         status_code = self.last_mission_status['status_code']
            #         status_text = self.last_mission_status['status_text']
            with self.mission_status_condition:
                status_received = self.mission_status_condition.wait_for(
                    lambda: self.mission_completed,
                    timeout=1800.0
                )

            if not status_received:
                return False

            with self.mission_status_condition:
                if self.last_mission_status:
                    status_code = self.last_mission_status['status_code']
                    status_text = self.last_mission_status['status_text']
                    
                    if status_code == 1:  # COMPLETED
                        print(f"✅ Mission '{mission_name}' COMPLETED successfully")
                        
                        # Log mission to history only after successful completion
                        self._log_mission_to_history(mission_name)

                        # Wait for gamma_bt to reset before sending next mission
                        print("⏱️  Waiting 6 seconds for ROS behavior tree to reset...")
                        time.sleep(6.0)

                        # print(f"[DEBUG MGR SLEEP-DONE] 6-second sleep complete, "
                        #     f"advancing to mission_counter+1")
                        
                        # Reset status event for next mission
                        # self.mission_status_received.clear()
                        # self.last_mission_status = None
                        with self.mission_status_condition:
                            self.mission_completed = False
                            self.last_mission_status = None
                    
                    elif status_code == 2:  # FAILED
                        print(f"❌ Mission '{mission_name}' FAILED")

                        # WAIT for mission logs to be written
                        # Mission logs are updated asynchronously, so we need to give the system time
                        print("⏳ Waiting for mission logs to be written...")
                        time.sleep(1.0)
                        
                        # Extract events from failed mission logs
                        mission_number = self.last_mission_status.get("mission_number")
                        events_context = extract_mission_events(self.mission_logs_file, mission_number)
                        
                        # Build failure context with formatted events
                        failure_context = f"Mission '{mission_name}' failed."
                        if events_context:
                            failure_context += f"\n{events_context}"
                        
                        remaining_missions = list(self.mission_queue[idx+1:])
                        if remaining_missions:
                            failure_context += f" Remaining missions were: {', '.join(remaining_missions)}."
                        
                        print("="*60)
                        print("MISSION FAILURE - AUTOMATIC REPLANNING")
                        print("="*60)
                        print(f"Failed Mission: {mission_name}")
                        if remaining_missions:
                            print(f"Remaining Missions: {', '.join(remaining_missions)}")
                        print("="*60)
                        
                        # Add failure context to planner
                        self.planner.add_failure_context(failure_context)
                        
                        # Automatically replan WITHOUT clarification field
                        # Use the original pending command (before it was split into mission queue)
                        original_command = self.pending_command or self.current_mission
                        
                        print("Step: Replanning with failure context...")

                        # checks for NEEDS_CLARIFICATION first
                        mission_result = self.planner.plan_command(original_command)
                        if not mission_result or len(mission_result) == 0:
                            print("Failed to replan mission after failure")
                            return False

                        mission_output = mission_result[0]

                        # ✨ CRITICAL: Check if planner needs clarification BEFORE treating as mission
                        if mission_output == "NEEDS_CLARIFICATION":
                            print("❓ Planner uncertain about replanned missions after failure")
                            print("❓ Requesting user clarification to resolve replanning...")
                            
                            # Set clarification state for failure recovery
                            self.waiting_for_clarification = True
                            self.pending_command = original_command
                            self.failed_mission_name = mission_name
                            self.remaining_missions = list(self.mission_queue[idx+1:])
                            
                            # Store state for later recovery
                            self._failure_replanning_context = {
                                "mission_name": mission_name,
                                "failure_context": failure_context,
                                "remaining_missions": self.remaining_missions,
                                "queue_at_failure": self.mission_queue.copy(),
                                "failure_index": idx
                            }
                            
                            self._write_status_file({
                                "status": "waiting_clarification_after_failure",
                                "failed_mission": mission_name,
                                "failure_context": failure_context,
                                "message": f"Mission '{mission_name}' failed. Planner needs clarification: {failure_context}",
                                "processing": False,
                                "timestamp": time.time()
                            })
                            
                            # Return and wait for user clarification
                            return True

                        # Now safe to treat as normal mission output
                        print(f"Replanned output: {mission_output}")
                        raw_missions = [m.strip() for m in mission_output.split(",")]

                        if all(m.lower() == "skip" for m in raw_missions):
                            print("All replanned missions are 'skip' - cancelling remaining missions")
                            return True

                        # Update mission queue with new plan
                        self.mission_queue = [m for m in raw_missions if m.lower() != "skip"]


                        if not self.mission_queue:
                            print("No valid missions after replanning")
                            return True
                        
                        print(f"New mission queue ({len(self.mission_queue)} missions): {self.mission_queue}")
                        
                        # Continue from the beginning of the new queue
                        self._continue_mission_execution_from_index(0)
                        return True


                    
                    elif status_code == 3:  # STOPPED
                        print(f"⏹️  Mission '{mission_name}' was STOPPED")
                        return False
                    
                    else:
                        print(f"⚠️  Unknown status code: {status_code}")
                        return False
        
        # All missions completed
        print(f"\n{'='*60}")
        print(f"🏁 All missions completed successfully!")
        print(f"{'='*60}\n")
        return True
    
    def _send_mission_to_ros(self, mission_name: str) -> bool:
        """Send mission to ROS bridge"""
        if mission_name not in self.missions:
            print(f"❌ Mission '{mission_name}' not found in missions.yaml")
            return False
        
        mission_data = self.missions[mission_name]
        
        # Increment mission counter for each individual mission sent
        self.mission_counter += 1
        
        # Build command payload
        command = {
            "mission_name": mission_name,
            "mission_id": self.mission_counter,  # Use counter as mission number
            "mission_tag": mission_data.get("mission_tag", 0),
            "latitude": mission_data.get("latitude", 0.0),
            "longitude": mission_data.get("longitude", 0.0),
            "depth": mission_data.get("depth", 0.0),
            "yaw": mission_data.get("yaw", 0.0),
            "radius": mission_data.get("radius", 0.0),
            "use_yaw": mission_data.get("use_yaw", False),
            "use_spiral": mission_data.get("use_spiral", False),
            "color": mission_data.get("color", "")
        }
        
        print(f"📤 Sending mission #{self.mission_counter}: {mission_name}")
        
        # Send to ROS bridge
        response = send_cmd_to_ros(command, "/send_mission")
        
        if response.get("status") == "success":
            print(f"✅ Mission sent to ROS successfully")
            return True
        else:
            print(f"❌ ROS bridge error: {response.get('message')}")
            return False
    
    def _mark_command_as_processing(self):
        """Mark command as being processed"""
        command_data = self._read_command_file()
        command_data["processing_started"] = True
        command_data["processing_timestamp"] = time.time()
        self._write_command_file(command_data)
    
    def _mark_command_processed(self):
        """Mark command as processed"""
        command_data = self._read_command_file()
        command_data["processed"] = True
        command_data["completed_timestamp"] = time.time()
        self._write_command_file(command_data)
    
    def _start_listener(self):
        """Start Flask listener for mission memory updates"""
        app.run(host=LISTEN_HOST, port=LISTEN_PORT, threaded=True, use_reloader=False)
    
    def _shutdown_system(self, planner_thread):
        """Shutdown all subsystems cleanly"""
        print("\n🛑 Shutting down system...")
        
        if self.enable_monitoring and self.resource_monitor:
            print("📊 Stopping resource monitor...")
            self.resource_monitor.stop_monitoring()
        
        print("🧠 Stopping planner...")
        self.planner.stop()
        if planner_thread and planner_thread.is_alive():
            planner_thread.join(timeout=5)
        
        # Save latency statistics
        print("📊 Saving latency statistics...")
        latency_tracker = get_latency_tracker()
        latency_tracker.save_statistics()
        
        self._write_status_file({"status": "shutdown", "processing": False, "timestamp": time.time()})
        print("✅ System shutdown complete")



# Global manager instance for Flask routes
manager_instance = None


# Flask routes for mission memory listener
@app.route('/mission_memory_update', methods=['POST'])
def mission_memory_update():
    """Receive and save mission memory updates from ROS"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"status": "error", "message": "No data received"}), 400
        
        # Save as YAML file
        with open(MEMORY_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        # Log update
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] Mission memory updated - {len(data) if isinstance(data, dict) else 0} entries\n"
        
        with open(LOG_FILE, 'a') as f:
            f.write(log_message)
        
        # print(log_message.strip())
        # print(f"Saved to: {MEMORY_FILE.absolute()}")
        
        return jsonify({
            "status": "success",
            "message": "Mission memory updated",
            "file": str(MEMORY_FILE.absolute())
        }), 200
    
    except Exception as e:
        error_msg = f"Error processing mission memory: {str(e)}"
        print(error_msg)
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/mission_logs_update", methods=["POST"])
def mission_logs_update():
    """Receive mission logs update from ROS bridge"""
    global manager_instance
    try:
        logs_data = request.get_json()

        if not logs_data or not isinstance(logs_data, dict):
            return jsonify({"status": "error", "message": "Invalid or empty logs payload"}), 400

        # Write to mission_logs.yaml
        with open(MISSION_LOGS_FILE, 'w') as f:
            yaml.dump(logs_data, f, default_flow_style=False)

        # Extract the mission counter from the latest mission in the payload
        latest_mission = logs_data.get('missions', [])[-1] if logs_data.get('missions') else None
        mission_counter = latest_mission.get('mission_counter') if latest_mission else None

        # Print mission events NOW that logs are available
        if mission_counter is not None:
            events_str = extract_mission_events(MISSION_LOGS_FILE, mission_counter)
            if events_str:
                print("\n📋 MISSION EVENTS (from logs):")
                print("-" * 60)
                events_list = [e.strip() for e in events_str.split(" | ")]
                for i, event in enumerate(events_list, 1):
                    print(f"  {i}. {event}")
                print("-" * 60)
                print()

                # If we’re waiting for clarification, enrich the planner failure_context with mission events
                if manager_instance and getattr(manager_instance, "waiting_for_clarification", False):
                    try:
                        # Read current failure_context from the planner
                        current_ctx = getattr(manager_instance.planner, "planner", None)
                        current_ctx = getattr(current_ctx, "failure_context", None)

                        # Build formatted events block
                        formatted_events = "\n".join([f"{i}. {e}" for i, e in enumerate(events_list, 1)])
                        events_block = f"\nMission events:\n{formatted_events}"

                        # Append events if not already present
                        if current_ctx and "Mission events:" not in current_ctx:
                            updated_ctx = f"{current_ctx}{events_block}"
                            manager_instance.planner.add_failure_context(updated_ctx)
                    except Exception as e:
                        print(f"⚠️ Could not append mission events to failure context: {e}")

        return jsonify({"status": "success"}), 200

    except Exception as e:
        print(f"❌ Error updating mission logs: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500




@app.route('/mission_status_update', methods=['POST'])
def mission_status_update():
    """Receive mission status updates from ROS bridge"""
    global manager_instance

    # print(f"[DEBUG MGR ROUTE] /mission_status_update POST received: {request.get_json()}")
    
    try:
        data = request.get_json()

        # Track latency for incoming requests
        if '_send_time' in data:
            send_time = data['_send_time']
            receive_time = time.time()
            latency_tracker = get_latency_tracker()
            latency_tracker.record_measurement('WSL18->WSL22', '/mission_status_update', 
                                              send_time, receive_time)
            del data['_send_time']
        
        if not data:
            return jsonify({"status": "error", "message": "No data received"}), 400
        
        mission_number = data.get("mission_number")
        status_code = data.get("status_code")
        status_text = data.get("status_text", "UNKNOWN")
        
        if manager_instance:
            manager_instance.handle_mission_status_update(mission_number, status_code, status_text)
        
        return jsonify({
            "status": "success",
            "message": "Mission status received"
        }), 200
    
    except Exception as e:
        error_msg = f"Error processing mission status: {str(e)}"
        print(error_msg)
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        "status": "healthy",
        "service": "ros_mission_manager",
        "port": LISTEN_PORT
    }), 200


@app.route('/get_latest', methods=['GET'])
def get_latest():
    """Get latest mission memory data"""
    try:
        if not MEMORY_FILE.exists():
            return jsonify({"status": "error", "message": "No data available"}), 404
        
        with open(MEMORY_FILE, 'r') as f:
            data = yaml.safe_load(f)
        
        return jsonify(data), 200
    
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="ROS Mission Manager - Sequential Execution Mode"
    )
    parser.add_argument(
        '--monitor-resources',
        action='store_true',
        help='Enable resource usage monitoring (CPU, memory, I/O)'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print(" ROS MISSION MANAGER - Sequential Execution Mode")
    print("=" * 60)
    
    if args.monitor_resources:
        print("📊 Resource monitoring: ENABLED")
    else:
        print("📊 Resource monitoring: DISABLED (use --monitor-resources to enable)")
    
    print("This terminal will show mission execution.")
    print("Use chat interface to send commands.")
    print("=" * 60)
    
    manager = ROSMissionManager(enable_monitoring=args.monitor_resources)
    
    try:
        print("🚀 Initializing system components...")
        manager.start()
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ System error: {e}")
        import traceback
        traceback.print_exc()
    
    print("👋 Manager terminated!")



if __name__ == "__main__":
    main()
