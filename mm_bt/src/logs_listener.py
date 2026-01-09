#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Mission Log Listener Node for gamma_bt
Subscribes to /rosout and logs mission-relevant messages to YAML
Logs from /gamma_bt, /perceive_world, and /guidance_manager nodes
Only logs between "MISSION COUNTER" and "MISSION COMPLETED/FAILED/ABORTED"
"""


import rospy
import yaml
import os
import re
from rosgraph_msgs.msg import Log
from datetime import datetime

class MissionLogListener:
    def __init__(self):
        rospy.init_node('mission_log_listener', anonymous=True)
        
        # Path configuration
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.log_dir = os.path.join(package_path, 'logs')
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        self.log_file = os.path.join(self.log_dir, 'mission_logs.yaml')
        
        # Wipe log file clean on startup
        with open(self.log_file, 'w') as f:
            f.write("# Mission logs - Started at {}\n".format(datetime.now().isoformat()))
        
        rospy.loginfo("[Mission Log Listener] Log file wiped clean: %s", self.log_file)
        
        # Mission state
        self.mission_active = False  # Only True between COUNTER and COMPLETED
        self.current_mission_counter = None  # From [MISSION COUNTER] message (1, 2, 3...)
        self.current_mission_tag = None  # Tag 1-6
        self.current_phase = None
        self.mission_logs = []
        self.all_missions = []
        self.phase_first_seen = {}  # Track first occurrence of messages per phase
        
        # Mission types by tag
        self.mission_types = {
            1: "GOTO_WAYPOINT",
            2: "GATE_PASS_MISSION",
            3: "SURVEY_MISSION",
            4: "STOP_RESET",
            5: "FIND_SPECIFIC_BUOY",
            6: "MOVE_TO_BUOY",
            7: 'MAP_AREA_MISSION'
        }
        
        # Messages to always skip (noise)
        self.skip_patterns = [
            r'^\[/gamma_bt\]: Running$',
            r'^\[/gamma_bt\]: Current status: ACTIVE$',
            r'^Mission: \d+$',  # Internal counter, not our mission boundary
            r'Mission \d+ \([a-zA-Z]+\) detected'
        ]
        
        # Subscribe to rosout
        rospy.Subscriber('/rosout', Log, self.rosout_callback, queue_size=100)
        
        rospy.loginfo("[Mission Log Listener] Started - Listening to /gamma_bt and /perceive_world on /rosout")
    
    def extract_mission_counter(self, message):
        """Extract mission counter from '[MISSION COUNTER] New mission received. Counter: X'"""
        match = re.search(r'\[MISSION COUNTER\].*Counter:\s*(\d+)', message)
        if match:
            return int(match.group(1))
        return None
    
    def extract_mission_tag(self, message):
        """Extract mission tag from 'Mission received: ID=X, TAG=Y' message"""
        match = re.search(r'Mission received:.*TAG=(\d+)', message)
        if match:
            return int(match.group(1))
        return None
    
    def is_mission_start(self, message):
        """Check if message is the MISSION COUNTER message (true mission start)"""
        return re.search(r'\[MISSION COUNTER\].*New mission received', message) is not None
    
    def is_mission_end(self, message):
        """Check if message indicates mission end"""
        end_patterns = [
            r'MISSION COMPLETED',
            r'MISSION STATUS.*COMPLETED',
            r'MISSION STATUS.*FAILED',
            r'MISSION STATUS.*ABORTED'
        ]
        for pattern in end_patterns:
            if re.search(pattern, message, re.IGNORECASE):
                return True
        return False
    
    def should_skip(self, message):
        """Check if message is noise"""
        msg_stripped = message.strip()
        for pattern in self.skip_patterns:
            if re.search(pattern, msg_stripped):
                return True
        return False
    
    def identify_phase(self, message):
        """Identify mission phase based on message content and mission tag"""
        msg_lower = message.lower()
        
        # Common phases for all mission types
        if 'mission counter' in msg_lower and 'new mission' in msg_lower:
            return 'MISSION_ACCEPTED'
        elif 'mission completed' in msg_lower or ('mission status' in msg_lower and 'completed' in msg_lower):
            return 'MISSION_COMPLETED'
        elif 'failed' in msg_lower:
            return 'MISSION_FAILED'
        elif 'aborted' in msg_lower:
            return 'MISSION_ABORTED'
        
        # Tag 1: GOTO_WAYPOINT - Simple navigation
        if self.current_mission_tag == 1:
            if 'publishing wp' in msg_lower or 'new goal received' in msg_lower:
                return 'NAVIGATION_START'
            elif 'going to wp' in msg_lower:
                return 'NAVIGATION_TO_WP'
            elif 'wp reached' in msg_lower or 'path completed' in msg_lower:
                return 'WP_REACHED'
            elif 'stand by' in msg_lower:
                return 'STANDBY'
        
        # Tag 2: GATE_PASS_MISSION - Navigate, find buoy, approach gate, pass gate
        elif self.current_mission_tag == 2:
            if 'publishing wp' in msg_lower and 'send_goal' in msg_lower:
                return 'NAVIGATION_START'
            elif 'going to wp' in msg_lower:
                return 'NAVIGATION_TO_WP'
            elif 'wp reached' in msg_lower or 'path completed' in msg_lower:
                return 'WP_REACHED'
            elif 'ready to search buoys' in msg_lower or ('strategy' in msg_lower and 'lawn' in msg_lower):
                return 'SEARCH_BUOY_LAWN_START'
            elif 'survey running' in msg_lower and 'spiral' not in msg_lower:
                return 'SEARCH_BUOY_LAWN'
            elif 'searching 2nd buoy' in msg_lower or 'spiral' in msg_lower:
                return 'SEARCH_BUOY_SPIRAL'
            elif 'buoy discovered' in msg_lower or 'buoy detected' in msg_lower:
                return 'BUOY_DETECTED'
            elif 'approaching gate' in msg_lower or 'ready to go over' in msg_lower:
                return 'APPROACH_GATE'
            elif 'passing gate' in msg_lower or 'go over gate' in msg_lower:
                return 'PASS_GATE'
            elif 'gate passed' in msg_lower:
                return 'GATE_PASSED'
        
        # Tag 3: SURVEY_MISSION
        elif self.current_mission_tag == 3:
            if 'publishing wp' in msg_lower and 'survey_area' in msg_lower:
                return 'SURVEY_START'
            elif 'survey area received' in msg_lower:
                return 'SURVEY_AREA_RECEIVED'
            elif 'survey running' in msg_lower:
                return 'SURVEY_RUNNING'
            elif 'buoy discovered' in msg_lower or 'buoy detected' in msg_lower:
                return 'BUOY_DETECTED'
            elif 'survey completed' in msg_lower or 'performed survey' in msg_lower:
                return 'SURVEY_COMPLETED'
        
        # Tag 4: STOP_RESET
        elif self.current_mission_tag == 4:
            if 'stop' in msg_lower and 'thrusters' in msg_lower:
                return 'STOPPING'
            elif 'reset' in msg_lower:
                return 'RESETTING'
        
        # Tag 5: FIND_SPECIFIC_BUOY
        elif self.current_mission_tag == 5:
            if 'survey running' in msg_lower or 'spiral survey' in msg_lower:
                return 'SEARCHING_BUOY'
            elif 'buoy discovered' in msg_lower or 'buoy detected' in msg_lower:
                return 'BUOY_FOUND'
        
        # Tag 6: MOVE_TO_BUOY
        elif self.current_mission_tag == 6:
            if 'going to wp' in msg_lower or 'new goal' in msg_lower:
                return 'MOVING_TO_BUOY'
            elif 'wp reached' in msg_lower or 'buoy reached' in msg_lower:
                return 'BUOY_REACHED'
            
        # Tag 7: MAP_AREA_MISSION - Survey area and find specific buoys
        elif self.current_mission_tag == 7:
            if 'system setting map area mission' in msg_lower:
                return 'MAP_AREA_START'
            elif 'go lawn mower survey for map area' in msg_lower:
                return 'MAP_AREA_SURVEY_START'
            elif 'survey area received' in msg_lower:
                return 'SURVEY_AREA_RECEIVED'
            elif 'survey running' in msg_lower:
                return 'SURVEY_RUNNING'
            elif 'buoy discovered' in msg_lower:
                return 'BUOY_DETECTED'
            elif 'survey completed' in msg_lower:
                return 'SURVEY_COMPLETED'
            elif 'loaded' in msg_lower and 'buoys from mission memory' in msg_lower:
                return 'CHECKING_MEMORY'
            elif 'maparea success' in msg_lower or 'all required buoys' in msg_lower:
                return 'MAP_AREA_SUCCESS'
            elif 'maparea incomplete' in msg_lower or 'not all buoys found' in msg_lower:
                return 'MAP_AREA_INCOMPLETE'

        
        # Generic phase identification for any status messages
        if 'system status' in msg_lower:
            if 'performing' in msg_lower:
                return 'SYSTEM_PERFORMING'
            elif 'ready' in msg_lower:
                return 'SYSTEM_READY'
        elif 'guidance status' in msg_lower:
            if 'waiting' in msg_lower:
                return 'GUIDANCE_WAITING'
            elif 'active' in msg_lower:
                return 'GUIDANCE_ACTIVE'
        
        # Keep current phase if no match
        return self.current_phase if self.current_phase else 'UNKNOWN'
    
    def should_save_message(self, message, new_phase):
        """Determine if message should be saved (deduplication)"""
        # Always save state transition and important messages
        important_patterns = [
            r'MISSION.*COUNTER',
            r'Mission received:',
            r'MISSION.*ACCEPTED',
            r'MISSION COMPLETED',
            r'MISSION STATUS',
            r'Publishing WP to topic',
            r'New goal received',
            r'path_follower.*Activated',
            r'path_follower.*Received path',
            r'Path completed',
            r'WP REACHED',
            r'STAND BY',
            r'SYSTEM STATUS',
            r'GUIDANCE STATUS',
            r'BUOY DISCOVERED',
            r'buoy detected',
            r'Updated discovered buoys',
            r'READY TO SEARCH',
            r'STRATEGY',
            r'APPROACHING GATE',
            r'PASSING GATE',
            r'GATE PASSED',
            r'Survey area received',
            r'ZENO.*MISSION'

            r'outside the safe area',
            r'Goal is outside',
            r'Area.*outside',
            r'Invalid.*value',
            r'Switching guidance'

            # TAG 6 Cardinal Navigation
            r'Cardinal point reached.*\d+/4',
            r'All cardinal points completed',
            r'Stop message sent for Move mission',
            r'Reset completed_counter',
            
            # TAG 5 & 6 Target Buoy Detection
            r'Target buoy.*color.*found.*survey',
            r'Target buoy color.*found.*proceeding to Move',
            # r'Wrong color detected.*continuing',
            
            # # Deduplication for non-target buoys
            # r'Non-yellow buoy detected.*continuing survey',
            # r'Non-target buoy detected.*continuing survey',

            # TAG 7 Map Area Mission
            r'SYSTEM SETTING MAP AREA',
            r'GO LAWN MOWER SURVEY FOR MAP AREA',
            r'Loaded.*buoys from mission memory',
            r'MapArea.*buoy found in memory',
            r'MapArea SUCCESS',
            r'MapArea INCOMPLETE',
            r'Not all buoys found in memory',
            r'All required buoys found',

        ]
        
        for pattern in important_patterns:
            if re.search(pattern, message, re.IGNORECASE):
                return True
        
        # For repetitive messages like "GOING TO WP", "SURVEY RUNNING"
        # Save only first occurrence per phase
        repetitive_patterns = [
            r'^GOING TO WP$',
            r'^SURVEY RUNNING$',
            r'^SPIRAL SURVEY RUNNING$',

            r'Non-yellow buoy detected.*continuing survey',     
            r'Non-target buoy detected.*continuing survey',     
            r'Wrong color detected.*continuing'                 
        ]
        
        for pattern in repetitive_patterns:
            if re.search(pattern, message.strip()):
                phase_msg_key = (self.current_mission_counter, new_phase, message.strip())
                if phase_msg_key in self.phase_first_seen:
                    return False  # Already saved in this phase
                else:
                    self.phase_first_seen[phase_msg_key] = True
                    return True
        
        # Save other unique messages
        return True
    
    def rosout_callback(self, msg):
        """Callback for /rosout messages"""
        # Filter: only gamma_bt and perceive_world nodes
        if msg.name not in ['/gamma_bt', '/perceive_world', '/guidance_manager']:
            return
        
        message_content = msg.msg
        
        # Skip noise
        if self.should_skip(message_content):
            return
        
        # Extract mission tag if present (from gamma_bt messages)
        mission_tag = self.extract_mission_tag(message_content)
        if mission_tag is not None:
            self.current_mission_tag = mission_tag
            rospy.loginfo("[Mission Log Listener] Detected mission tag: %d (%s)", 
                         mission_tag, self.mission_types.get(mission_tag, "UNKNOWN"))
        
        # Check for mission start (MISSION COUNTER message)
        if self.is_mission_start(message_content):
            if self.mission_active:
                # Previous mission didn't complete properly - save it anyway
                rospy.logwarn("[Mission Log Listener] Mission %d interrupted by new mission", 
                            self.current_mission_counter)
                self.finalize_current_mission()
            
            # Extract mission counter from "[MISSION COUNTER] ... Counter: X"
            counter = self.extract_mission_counter(message_content)
            if counter is not None:
                self.current_mission_counter = counter
            
            self.mission_active = True
            self.current_phase = 'MISSION_ACCEPTED'
            self.mission_logs = []
            self.phase_first_seen.clear()
            
            # rospy.logwarn("[Mission Log Listener] ===== MISSION %d STARTED (TAG %d) =====", 
            #              self.current_mission_counter, self.current_mission_tag or 0)
            rospy.logwarn("[Mission Log Listener] ===== MISSION %s STARTED =====",
              str(self.current_mission_counter) if self.current_mission_counter else "?")

        
        # Only save logs when mission is active
        if not self.mission_active:
            return
        
        # Identify phase
        new_phase = self.identify_phase(message_content)
        if new_phase != self.current_phase:
            rospy.loginfo("[Mission Log Listener] Phase: %s -> %s", 
                         self.current_phase, new_phase)
            self.current_phase = new_phase
        
        # Decide if we should save
        if self.should_save_message(message_content, new_phase):
            log_entry = {
                'mission_counter': self.current_mission_counter,
                'mission_tag': self.current_mission_tag,
                'mission_type': self.mission_types.get(self.current_mission_tag, "UNKNOWN"),
                'mission_phase': new_phase,
                'log_level': self.log_level_to_string(msg.level),
                'message': message_content,
                'ros_time_secs': msg.header.stamp.secs,
                'ros_time_nsecs': msg.header.stamp.nsecs
            }
            
            self.mission_logs.append(log_entry)
            rospy.logdebug("[Mission Log Listener] Saved: [M%d][%s] %s", 
                          self.current_mission_counter, new_phase, message_content[:60])
        
        # Check for mission end
        if self.is_mission_end(message_content):
            rospy.logwarn("[Mission Log Listener] ===== MISSION %d ENDED =====", 
                         self.current_mission_counter)
            self.finalize_current_mission()
            self.mission_active = False
            self.current_mission_tag = None
            self.current_phase = None
    
    def finalize_current_mission(self):
        """Finalize and save current mission"""
        if len(self.mission_logs) > 0:
            mission_data = {
                'mission_counter': self.current_mission_counter,
                'mission_tag': self.current_mission_tag,
                'mission_type': self.mission_types.get(self.current_mission_tag, "UNKNOWN"),
                'log_count': len(self.mission_logs),
                'logs': self.mission_logs
            }
            self.all_missions.append(mission_data)
            self.save_to_yaml()
            rospy.loginfo("[Mission Log Listener] Mission %d finalized with %d log entries", 
                         self.current_mission_counter, len(self.mission_logs))
    
    def log_level_to_string(self, level):
        """Convert ROS log level to string"""
        levels = {1: 'DEBUG', 2: 'INFO', 4: 'WARN', 8: 'ERROR', 16: 'FATAL'}
        return levels.get(level, 'UNKNOWN')
    
    def save_to_yaml(self):
        """Save all missions to YAML file"""
        try:
            output = {
                'missions': self.all_missions,
                'total_missions': len(self.all_missions),
                'total_log_entries': sum(m['log_count'] for m in self.all_missions),
                'last_updated': datetime.now().isoformat(),
                'mission_types_reference': self.mission_types
            }
            
            with open(self.log_file, 'w') as f:
                yaml.dump(output, f, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            rospy.logerr("[Mission Log Listener] Failed to save: %s", str(e))
    
    def run(self):
        """Keep node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = MissionLogListener()
        listener.run()
    except rospy.ROSInterruptException:
        pass




