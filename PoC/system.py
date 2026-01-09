#!/usr/bin/env python3
"""
Main system integrating LM planner and action executor.
Complete robotic mission system with world state feedback loop.
Receives commands from separate chat interface via file communication.
"""

import time
import json
import threading
from typing import List, Optional
from pathlib import Path

from state import World
from environment import real_world, external_waypoint, pipeline_id
from t5base_planner import LMPlanner
from executor import ActionExecutor

class RoboticMissionSystem:
    """Main system coordinating all components"""
    
    def __init__(self, command_file="command.json", status_file="status.json"):
        # File communication
        self.command_file = Path(command_file)
        self.status_file = Path(status_file)
        
        # Initialize world state
        self.world = World()
        self.real_world = real_world
        self.external_waypoint = external_waypoint
        self.pipeline_id = pipeline_id
        
        # Initialize components
        self.planner = LMPlanner(self.world)
        self.executor = ActionExecutor(self.world)
        
        # System state
        self.running = True
        self.mission_counter = 0
        self.current_mission = None
        self.mission_thread = None
        
        # Clarification state tracking
        self.waiting_for_clarification = False
        self.pending_command = None
        
        # Initialize communication files
        self._initialize_files()
    
    def _initialize_files(self):
        """Initialize communication files"""
        print("🔧 Initializing communication files...")
        
        # Always create/reset the command file to ensure clean state
        initial_command = {
            "command": None, 
            "timestamp": time.time(), 
            "processed": True
        }
        self._write_command_file(initial_command)
        print(f"✅ Command file initialized: {self.command_file}")
        
        # Initialize status file
        initial_status = {
            "status": "initializing", 
            "processing": False, 
            "timestamp": time.time()
        }
        self._write_status_file(initial_status)
        print(f"✅ Status file initialized: {self.status_file}")
        
        # Verify files were created
        if not self.command_file.exists() or not self.status_file.exists():
            raise RuntimeError("Failed to create communication files")
    
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
                    data = json.load(f)
                    return data
        except (json.JSONDecodeError, FileNotFoundError) as e:
            print(f"⚠️  Warning: Error reading command file: {e}")
            # Return safe default
            return {"command": None, "timestamp": time.time(), "processed": True}
        except Exception as e:
            print(f"❌ Error reading command file: {e}")
            return {"command": None, "timestamp": time.time(), "processed": True}
    
    def start(self):
        """Start the complete system"""
        print("🚀 Starting Mission System")
        print("=" * 60)
        
        # Start planner thread
        planner_thread = self.planner.start()
        
        # Signal ready status
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
        
        # Track the last processed command timestamp to avoid duplicates
        last_processed_timestamp = 0
        idle_counter = 0
        
        try:
            # Main system loop
            while self.running:
                # Read the latest command
                command_data = self._read_command_file()
                command = command_data.get("command")
                processed = command_data.get("processed", True)
                command_timestamp = command_data.get("timestamp", 0)

                # Process only new unprocessed commands
                if command and not processed and command_timestamp > last_processed_timestamp:
                    idle_counter = 0
                    last_processed_timestamp = command_timestamp

                    cmd_lower = command.strip().lower()

                    # Only "shutdown" actually stops the system
                    if command.strip().upper() == "SHUTDOWN":
                        print("🛑 Shutdown command received from chat interface")
                        self.running = False
                        break

                    # Stop keywords interrupt the current mission
                    if cmd_lower in ["exit", "quit", "stop"]:
                        if self.current_mission and self.mission_thread and self.mission_thread.is_alive():
                            print(f"⏹️ Mission interrupted by stop command: '{command}'")
                            self.current_mission = None
                            self.waiting_for_clarification = False
                            self.pending_command = None
                        self._mark_command_processed()
                        continue  # skip regular command processing


                    # Mark as processing immediately
                    self._mark_command_as_processing()

                    # --- Clarification handling ---
                    if self.waiting_for_clarification and self.pending_command:
                        cmd_lower = command.strip().lower()

                        # Cancel clarification
                        if cmd_lower in ["stop clarification", "cancel clarification"]:
                            print("🛑 Clarification request cancelled by user")

                            # Reset state
                            self.waiting_for_clarification = False
                            self.pending_command = None

                            # Mark command as processed
                            self._mark_command_processed()

                            # Update status.json to ready and clear previous message
                            self._write_status_file({
                                "status": "ready",
                                "processing": False,
                                "pending_command": None,
                                "message": None,
                                "timestamp": time.time()
                            })
                            continue  # skip to next loop iteration

                        # Treat as clarification input
                        print(f"📝 Received clarification: '{command}' for command: '{self.pending_command}'")
                        self.planner.add_clarification(command)
                        self._mark_command_processed()

                        # Retry original mission with clarification
                        self.current_mission = self.pending_command
                        self.mission_thread = threading.Thread(
                            target=self._execute_mission_thread,
                            args=(self.pending_command,),
                            daemon=True
                        )
                        self.mission_thread.start()
                        continue  # skip regular command processing

                    # --- Regular command processing ---
                    # Stop any current mission
                    if self.current_mission and self.mission_thread and self.mission_thread.is_alive():
                        print("⏹️ Stopping current mission for new command")
                        self.current_mission = None
                        self.waiting_for_clarification = False
                        self.pending_command = None

                    # Start new mission
                    self.current_mission = command
                    self.mission_thread = threading.Thread(
                        target=self._execute_mission_thread,
                        args=(command,),
                        daemon=True
                    )
                    self.mission_thread.start()

                else:
                    # No new command; system idle
                    idle_counter += 1
                    if idle_counter % 600 == 0:  # every ~60 seconds
                        if self.waiting_for_clarification:
                            print(f"⏳ Waiting for clarification for command: '{self.pending_command}'...")
                        else:
                            print("💤 System idle - waiting for commands...")

                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 System shutdown initiated")
        finally:
            self._shutdown_system(planner_thread)

    
    def _execute_mission_thread(self, command: str):
        """Execute a mission in a separate thread"""
        try:
            # Mark as processing
            self._write_status_file({
                "status": "processing", 
                "processing": True, 
                "timestamp": time.time()
            })
            
            # Execute the mission
            success = self._execute_mission(command)
            
            # Only mark command as processed if we're not waiting for clarification
            if not self.waiting_for_clarification:
                self._mark_command_processed()
            
        except Exception as e:
            print(f"❌ Mission execution error: {e}")
            if not self.waiting_for_clarification:
                self._mark_command_processed()
        finally:
            if not self.waiting_for_clarification:
                self._write_status_file({
                    "status": "ready",
                    "processing": False,
                    "timestamp": time.time()
                })
                self.current_mission = None

    def _execute_mission(self, command: str):
        """Execute a complete mission from command to world state update"""
        self.mission_counter += 1
        
        print(f"\n🎯 MISSION #{self.mission_counter}")
        print(f"📝 Command: '{command}'")
        print("-" * 40)
        
        # Step 1: Generate action operation using LM planner
        print("🧠 Step 1: Planning action operation...")
        action_operation = self.planner.plan_command(command)
        
        if action_operation == ["NEEDS_CLARIFICATION"]:
            print("❓ Please provide clarification through the chat interface")
            
            # Set clarification state
            self.waiting_for_clarification = True
            self.pending_command = command
            
            # Don't mark command as processed yet - we need clarification
            return True
        
        if not action_operation:
            print("❌ Failed to generate action operation")
            return False

        # Clear clarification state if we successfully got an action operation
        self.waiting_for_clarification = False
        self.pending_command = None

        # Check if mission was interrupted
        if self.current_mission != command:
            print("⏹️  Mission interrupted by new command")
            return False
        
        # Step 2: Execute action operation
        print("🚀 Step 2: Executing actions...")
        success = self.executor.execute_operation(action_operation)
        
        # 🔥 NEW LOGIC: If execution fails → ask for clarification
        if not success:
            print("❓ Execution failed, asking for clarification")
            self.waiting_for_clarification = True
            self.pending_command = command
            return True  # Don’t mark as processed, system will wait
        
        # Check if mission was interrupted
        if self.current_mission != command:
            print("⏹️  Mission interrupted during execution")
            return False
        
        # Step 3: Report world state changes
        print("📊 Step 3: World state updated")
        self._report_world_state()
        
        # Step 4: Mission summary
        status = "✅ SUCCESS" if success else "⚠️ PARTIAL SUCCESS"
        print(f"🏁 Mission #{self.mission_counter} completed - {status}")
        print("-" * 40)
        
        return success

    
    def _mark_command_as_processing(self):
        """Mark command as being processed to prevent duplicates"""
        command_data = self._read_command_file()
        command_data["processing_started"] = True
        command_data["processing_timestamp"] = time.time()
        self._write_command_file(command_data)
    
    def _mark_command_processed(self):
        """Mark the current command as processed"""
        command_data = self._read_command_file()
        command_data["processed"] = True
        command_data["completed_timestamp"] = time.time()
        self._write_command_file(command_data)

    def _execute_mission(self, command: str):
        """Execute a complete mission from command to world state update"""
        self.mission_counter += 1
        
        print(f"\n🎯 MISSION #{self.mission_counter}")
        print(f"📝 Command: '{command}'")
        print("-" * 40)
        
        # Step 1: Generate action operation using LM planner
        print("🧠 Step 1: Planning action operation...\n")
        action_operation = self.planner.plan_command(command)
        
        if action_operation == ["NEEDS_CLARIFICATION"]:
            print("❓ Please provide clarification through the chat interface")
            
            # Set clarification state
            self.waiting_for_clarification = True
            self.pending_command = command

            # Write status file so chat can detect clarification request
            self._write_status_file({
                "status": "waiting_clarification",
                "pending_command": command,
                "message": "Planner requires clarification to proceed",
                "processing": False,
                "timestamp": time.time()
            })
            
            # Don't mark command as processed yet - we need clarification
            return True
        
        if not action_operation:
            print("❌ Failed to generate action operation")
            return False

        # ✅ Do NOT clear clarification state here — only after mission success
        
        # Check if mission was interrupted
        if self.current_mission != command:
            print("⏹️  Mission interrupted by new command")
            return False
        
        # Step 2: Execute action operation
        print("🚀 Step 2: Executing actions...")
        success = self.executor.execute_operation(action_operation)

        if not success:
            print("❓ Execution failed, asking for clarification")
            self.waiting_for_clarification = True
            self.pending_command = command

            # Notify chat via status.json
            self._write_status_file({
                "status": "waiting_clarification",
                "pending_command": command,
                "message": "Execution failed, need clarification to retry",
                "processing": False,
                "timestamp": time.time()
            })

            return True  # Don’t mark as processed, system will wait
        
        # Check if mission was interrupted
        if self.current_mission != command:
            print("⏹️  Mission interrupted during execution")
            return False
        
        # Step 3: Report world state changes
        print("📊 Step 3: World state updated")
        self._report_world_state()
        
        # Step 4: Mission summary
        status = "✅ SUCCESS" if success else "⚠️ PARTIAL SUCCESS"
        print(f"🏁 Mission #{self.mission_counter} completed - {status}")
        print("-" * 40)
        
        # ✅ Only clear clarification state after a successful mission
        self.waiting_for_clarification = False
        self.pending_command = None
        
        return success


    
    def _report_world_state(self):
        """Report current world state"""
        print("  Current World State:")
        
        # Sensor status
        sensors = self.world.sensors
        print(f"    🔧 Sensors: Receiver:{sensors.receiver} Camera:{sensors.camera} Sonar:{sensors.sonar}")
        
        # Position
        pos = self.world.coordinates.current
        print(f"    📍 Position: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
        
        # Waypoints
        if self.world.coordinates.waypoints:
            print(f"    🎯 Waypoints: {len(self.world.coordinates.waypoints)} pending")
        
        # Buoys
        if self.world.buoys:
            print(f"    🚩 Buoys detected: {len(self.world.buoys)}")
            for i, buoy in enumerate(self.world.buoys[-3:]):  # Show last 3
                print(f"      - {buoy.color} buoy at [{buoy.position[0]:.1f}, {buoy.position[1]:.1f}, {buoy.position[2]:.1f}]")

        # Gate
        if self.world.gate.position1 and self.world.gate.position2:
            print(f"    🚪 Gate positions: {self.world.gate.position1}, {self.world.gate.position2}")

        # Pipelines
        if self.world.pipelines:
            print(f"    🔧 Pipelines detected: {len(self.world.pipelines)}")
            for pipeline in self.world.pipelines:
                # Find pipes with red markers - but only process each once
                red_marker_reported = False
                for pipe in pipeline.pipes:
                    if not hasattr(pipe, 'markers') or not pipe.markers:
                        continue
                        
                    if any(marker.color.lower() == "red" for marker in pipe.markers) and not red_marker_reported:
                        red_marker_reported = True
                        valve_status = "closed" if pipe.valve_closure else "not closed"
                        print(f"      • Pipe {pipe.number}: Red marker found, Valve {valve_status}, Ring {pipe.ring}")
                        
                if not red_marker_reported:
                    print(f"      - No red marker found yet")

        # Main pipe
        if hasattr(self.world, 'main_pipe'):
            if self.world.main_pipe.end_position:
                print(f"    🔧 Main pipe found at [{self.world.main_pipe.end_position[0]:.1f}, "
                    f"{self.world.main_pipe.end_position[1]:.1f}, "
                    f"{self.world.main_pipe.end_position[2]:.1f}]")
                if hasattr(self.world.main_pipe, 'markers') and self.world.main_pipe.markers:
                    for marker in self.world.main_pipe.markers:
                        color = marker.color.capitalize()
                        pos = marker.position
                        print(f"      • {color} marker at [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
                else:
                    print("      - No markers found")
        
        # Interesting points
        if self.world.interesting_points:
            print(f"    🔍 Interesting points mapped: {len(self.world.interesting_points)}")
    
    def _shutdown_system(self, planner_thread):
        """Gracefully shutdown the system"""
        print("🔥 Shutting down system components...")
        
        # Stop components
        self.planner.stop()
        self.running = False
        
        # Wait for planner thread to finish
        planner_thread.join(timeout=3)
        
        # Update status file
        self._write_status_file({
            "status": "shutdown", 
            "processing": False, 
            "timestamp": time.time()
        })
        
        print("🏁 Robotic Mission System shutdown complete")
    
    def get_world_state_summary(self) -> dict:
        """Get a summary of current world state"""
        return {
            "position": self.world.coordinates.current,
            "target": self.world.coordinates.waypoints[-1] if self.world.coordinates.waypoints else None,
            "sensors": {
                "receiver": self.world.sensors.receiver,
                "camera": self.world.sensors.camera,
                "sonar": self.world.sensors.sonar
            },
            "buoys_detected": len(self.world.buoys),
            "pipelines_detected": len(self.world.pipelines),
            "interesting_points": len(self.world.interesting_points),
            "mission_count": self.mission_counter
        }

def main():
    """Main entry point"""
    print("=" * 60)
    print("                     MISSION PLANNER")
    print("=" * 60)
    print("This terminal will show mission execution.")
    print("Use the chat interface in the chat terminal to send commands.")
    print("Wait for system initialization...")
    print("=" * 60)
    
    # Create and start the system
    system = RoboticMissionSystem()
    
    try:
        print("🚀 Initializing system components...")
        system.start()
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ System error: {e}")
        print("🔧 Please check all dependencies and model files")
        import traceback
        print("🔍 Full error trace:")
        traceback.print_exc()
    
    print("👋 Main system terminated!")

if __name__ == "__main__":
    main()