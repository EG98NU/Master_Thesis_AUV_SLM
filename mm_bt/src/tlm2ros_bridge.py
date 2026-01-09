#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Enhanced ROS Bridge Node for Mission Management
- Receives HTTP requests and publishes ROS messages
- Monitors mission_memory.yaml for changes and sends to WSL 22
- Monitors mission_logs.yaml for changes and sends to WSL 22
Python 2.7 Compatible Version
"""

import threading
import time
import os
import yaml
import requests
import rospy
from flask import Flask, request, jsonify
from rami_msgs.msg import SetInfoMissionManager
from std_msgs.msg import Header
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from std_msgs.msg import Int32MultiArray
import csv



class FileWatcher(FileSystemEventHandler):
    """Generic file watcher with debouncing"""
    
    def __init__(self, file_path, callback, name="File"):
        self.file_path = file_path
        self.callback = callback
        self.name = name
        self.last_modified = 0
    
    def on_modified(self, event):
        """Called when file is modified"""
        if not event.is_directory and event.src_path == self.file_path:
            # Debounce: avoid multiple triggers for same modification
            current_time = time.time()
            if current_time - self.last_modified > 1.0:
                self.last_modified = current_time
                rospy.loginfo("[{} Watcher] Detected change in {}".format(
                    self.name, os.path.basename(self.file_path)))
                self.callback()

class LatencyTrackerBridge:
    """Track latency for ROS bridge (Python 2.7 compatible)"""
    
    def __init__(self, csv_file="/tmp/bridge_latency_measurements.csv"):
        self.csv_file = csv_file
        self.measurements = []
        self._initialize_csv()
    
    def _initialize_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'direction', 'endpoint', 'send_time', 'receive_time', 'latency_ms'])
    
    def record_measurement(self, direction, endpoint, send_time, receive_time):
        latency_ms = (receive_time - send_time) * 1000
        measurement = {'timestamp': time.time(), 'direction': direction, 'endpoint': endpoint,
                      'send_time': send_time, 'receive_time': receive_time, 'latency_ms': latency_ms}
        self.measurements.append(measurement)
        
        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([measurement['timestamp'], measurement['direction'], 
                           measurement['endpoint'], measurement['send_time'], 
                           measurement['receive_time'], measurement['latency_ms']])
        
        rospy.loginfo("[Latency] {} - {}: {:.2f} ms".format(direction, endpoint, latency_ms))
    
    def save_statistics(self, stats_file="/tmp/bridge_latency_statistics.txt"):
        if not self.measurements:
            return
        
        latencies = [m['latency_ms'] for m in self.measurements]
        import math
        mean_latency = sum(latencies) / len(latencies)
        variance = sum((x - mean_latency) ** 2 for x in latencies) / len(latencies)
        std_dev = math.sqrt(variance) if len(latencies) > 1 else 0
        
        with open(stats_file, 'w') as f:
            f.write("="*60 + "\n")
            f.write("BRIDGE LATENCY STATISTICS\n")
            f.write("="*60 + "\n")
            f.write("Total: {}\n".format(len(latencies)))
            f.write("Average: {:.2f} ms\n".format(mean_latency))
            f.write("Std Dev: {:.2f} ms\n".format(std_dev))
            f.write("Min: {:.2f} ms\n".format(min(latencies)))
            f.write("Max: {:.2f} ms\n".format(max(latencies)))
        
        rospy.loginfo("Latency stats saved to {}".format(stats_file))



class RosMissionBridge:
    def __init__(self):
        self.app = Flask(__name__)
        self.data_lock = threading.Lock()
        
        self.ROS_HOST = '0.0.0.0'  # Listen on all interfaces
        self.ROS_PORT = 5001
        
        # WSL 22 endpoint configuration
        self.WSL22_HOST = 'localhost'  # Change to WSL 22 IP if needed
        self.WSL22_PORT = 5002
        
        rospy.init_node('ros_mission_bridge', anonymous=True)
        
        # ROS Publisher for mission commands
        self.mission_pub = rospy.Publisher(
            '/gamma_bt/setInfo',
            SetInfoMissionManager,
            queue_size=10
        )
        
        # Mission memory file path
        self.mission_memory_path = rospy.get_param(
            '~mission_memory_file',
            '/root/btree_ws/src/mm_bt/config/mission_memory.yaml'
        )
        
        # Mission logs file path
        self.mission_logs_path = rospy.get_param(
            '~mission_logs_file',
            '/root/btree_ws/src/mm_bt/logs/mission_logs.yaml'
        )
        
        # Setup file watchers for both files
        self.setup_file_watchers()
        
        # Wait for subscribers
        rospy.sleep(1)
        
        # Define endpoint routes
        self.app.add_url_rule('/send_mission', 'send_mission',
                             self.send_mission, methods=['POST'])
        self.app.add_url_rule('/health', 'health',
                             self.health_check, methods=['GET'])
        self.app.add_url_rule('/get_mission_memory', 'get_mission_memory',
                             self.get_mission_memory, methods=['GET'])
        self.app.add_url_rule('/get_mission_logs', 'get_mission_logs',
                             self.get_mission_logs, methods=['GET'])
        
        # Start Flask in a separate thread
        flask_thread = threading.Thread(target=self.run_flask_server)
        flask_thread.daemon = True
        flask_thread.start()
        
        rospy.loginfo("[ROS Mission Bridge] Node initialized and ready")
        rospy.loginfo("[ROS Mission Bridge] Watching mission_memory: {}".format(
            self.mission_memory_path))
        rospy.loginfo("[ROS Mission Bridge] Watching mission_logs: {}".format(
            self.mission_logs_path))
        
        # Subscriber for mission status
        self.mission_status_sub = rospy.Subscriber(
            '/gamma_bt/mission_status',
            Int32MultiArray,
            self.mission_status_callback,
            queue_size=10
        )
        
        rospy.loginfo("[ROS Mission Bridge] Subscribed to /gamma_bt/mission_status")
        self.last_mission_status_time = 0
        self.mission_status_received = False
        self.mission_completion_delay = rospy.get_param('~mission_delay', 5.0)
        rospy.loginfo("[ROS Mission Bridge] Mission completion delay: {} seconds".format(
            self.mission_completion_delay))
        
        self.last_sent_mission_number = None
        self.last_sent_status_code = None

        self.latency_tracker = LatencyTrackerBridge()
        rospy.on_shutdown(self.shutdown_hook)

    
    def mission_status_callback(self, msg):
        """Callback when gamma_bt publishes mission status"""
        if len(msg.data) >= 2:
            mission_number = msg.data[0]
            status_code = msg.data[1]  # 1=COMPLETED, 2=FAILED, 3=ABORTED

            # rospy.loginfo("[DEBUG BRIDGE RECV] mission={}, status={} | Last: mission={}, status={}".format(
            #     mission_number, status_code, self.last_sent_mission_number, self.last_sent_status_code))

            
            # Check if this is a duplicate status update
            is_same_mission = (self.last_sent_mission_number == mission_number)
            is_same_status = (self.last_sent_status_code == status_code)
            
            # CRITICAL FIX: Allow sending if previous status was terminal (1=COMPLETED, 3=ABORTED)
            # Only block if previous status was FAILED (2) and trying to send FAILED again
            previous_was_terminal = (self.last_sent_status_code in [1, 3])
            
            if (is_same_mission and is_same_status and not previous_was_terminal):

                # rospy.logwarn("[DEBUG BRIDGE DUP-CHECK] is_same_mission={}, is_same_status={}, "
                #     "previous_was_terminal={} -> BLOCKING={}".format(
                #         is_same_mission, is_same_status, previous_was_terminal,
                #         is_same_mission and is_same_status and not previous_was_terminal))

                
                # Block duplicate: same mission, same status, and previous status wasn't terminal
                rospy.logwarn("[Mission Status] Blocking duplicate: Mission #%d already reported with status %d",
                    mission_number, status_code)
                return
            
            # Update tracking
            self.last_sent_mission_number = mission_number
            self.last_sent_status_code = status_code
            
            status_str = {1: "COMPLETED", 2: "FAILED", 3: "ABORTED"}.get(
                status_code, "UNKNOWN")
            
            rospy.loginfo("[Mission Status] Mission #%d - %s",
                mission_number, status_str)
            
            # rospy.loginfo("[DEBUG BRIDGE SEND] Sending to WSL 22 - mission={}, status={}".format(
            #     mission_number, status_str))

            
            with self.data_lock:
                self.last_mission_status_time = time.time()
                self.mission_status_received = True
            
            # Prepare data to send to WSL 22
            status_data = {
                "mission_number": int(mission_number),
                "status_code": int(status_code),
                "status_text": status_str,
                "timestamp": rospy.Time.now().to_sec()
            }
            
            # Send to WSL 22
            self.send_mission_status_to_wsl22(status_data)


    
    def send_mission_status_to_wsl22(self, data):
        """Send mission status to WSL 22 via HTTP"""

        # rospy.loginfo("[DEBUG BRIDGE HTTP] POST /mission_status_update with payload: {}".format(data))
    
        endpoint = "http://{}:{}/mission_status_update".format(
            self.WSL22_HOST, self.WSL22_PORT)
        
        try:
            response = requests.post(
                endpoint,
                json=data,
                headers={'Content-Type': 'application/json'},
                timeout=1800
            )
            
            # rospy.loginfo("[DEBUG BRIDGE HTTP-RESP] Status: {}, Body: {}".format(
            #     response.status_code, response.json()))

        
            if response.status_code == 200:
                rospy.loginfo("[Bridge] Mission status sent successfully")
            else:
                rospy.logwarn("[Bridge] Failed to send status: HTTP %d", 
                            response.status_code)
        except requests.RequestException as e:
            rospy.logerr("[Bridge] Error sending mission status: %s", str(e))
    
    def setup_file_watchers(self):
        """Setup watchdog to monitor both mission_memory.yaml and mission_logs.yaml"""
        self.observer = Observer()
        
        # Watch mission_memory.yaml
        if os.path.exists(self.mission_memory_path):
            memory_handler = FileWatcher(
                self.mission_memory_path,
                self.on_mission_memory_changed,
                "Mission Memory"
            )
            memory_watch_dir = os.path.dirname(self.mission_memory_path)
            self.observer.schedule(memory_handler, memory_watch_dir, recursive=False)
            rospy.loginfo("[ROS Mission Bridge] Watching: {}".format(
                self.mission_memory_path))
        else:
            rospy.logwarn("[ROS Mission Bridge] Mission memory file not found: {}".format(
                self.mission_memory_path))
        
        # Watch mission_logs.yaml
        if os.path.exists(self.mission_logs_path):
            logs_handler = FileWatcher(
                self.mission_logs_path,
                self.on_mission_logs_changed,
                "Mission Logs"
            )
            logs_watch_dir = os.path.dirname(self.mission_logs_path)
            self.observer.schedule(logs_handler, logs_watch_dir, recursive=False)
            rospy.loginfo("[ROS Mission Bridge] Watching: {}".format(
                self.mission_logs_path))
        else:
            rospy.logwarn("[ROS Mission Bridge] Mission logs file not found: {}".format(
                self.mission_logs_path))
        
        # Start the observer
        self.observer.start()
        rospy.loginfo("[ROS Mission Bridge] File watchers started")
    
    def on_mission_memory_changed(self):
        """Callback when mission_memory.yaml is modified"""
        try:
            # Read the updated file
            with open(self.mission_memory_path, 'r') as f:
                mission_data = yaml.safe_load(f)
            
            rospy.loginfo("[ROS Mission Bridge] Sending updated mission_memory.yaml to WSL 22")
            
            # Send to WSL 22
            response = self.send_to_wsl22(mission_data, "mission_memory_update")
            
            if response.get('status') == 'success':
                rospy.loginfo("[ROS Mission Bridge] Successfully sent mission memory update")
            else:
                rospy.logwarn("[ROS Mission Bridge] Failed to send memory update: {}".format(
                    response.get('message', 'Unknown error')))
        
        except Exception as e:
            rospy.logerr("[ROS Mission Bridge] Error reading/sending mission memory: {}".format(
                str(e)))
    
    def on_mission_logs_changed(self):
        """Callback when mission_logs.yaml is modified"""
        try:
            # Read the updated file
            with open(self.mission_logs_path, 'r') as f:
                logs_data = yaml.safe_load(f)
            
            rospy.loginfo("[ROS Mission Bridge] Sending updated mission_logs.yaml to WSL 22")
            
            # Send to WSL 22
            response = self.send_to_wsl22(logs_data, "mission_logs_update")
            
            if response.get('status') == 'success':
                rospy.loginfo("[ROS Mission Bridge] Successfully sent mission logs update")
            else:
                rospy.logwarn("[ROS Mission Bridge] Failed to send logs update: {}".format(
                    response.get('message', 'Unknown error')))
        
        except Exception as e:
            rospy.logerr("[ROS Mission Bridge] Error reading/sending mission logs: {}".format(
                str(e)))
    
    def send_to_wsl22(self, data, endpoint_name):
        """Send data to WSL 22"""
        endpoint = "http://{}:{}/{}".format(
            self.WSL22_HOST, self.WSL22_PORT, endpoint_name)
        
        try:
            response = requests.post(
                endpoint,
                json=data,
                headers={'Content-Type': 'application/json'},
                timeout=1800
            )
            
            return response.json()
        except requests.RequestException as e:
            return {'status': 'error', 'message': str(e)}
    
    def run_flask_server(self):
        """Run the Flask server in a separate thread"""
        rospy.loginfo("[ROS Mission Bridge] Starting HTTP server on {}:{}".format(
            self.ROS_HOST, self.ROS_PORT))
        self.app.run(host=self.ROS_HOST, port=self.ROS_PORT, threaded=True)
    
    def health_check(self):
        """Health check endpoint"""
        return jsonify({"status": "healthy", "node": "ros_mission_bridge"}), 200
    
    def get_mission_memory(self):
        """GET endpoint to retrieve current mission_memory.yaml"""
        try:
            with open(self.mission_memory_path, 'r') as f:
                mission_data = yaml.safe_load(f)
            return jsonify(mission_data), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    
    def get_mission_logs(self):
        """GET endpoint to retrieve current mission_logs.yaml"""
        try:
            with open(self.mission_logs_path, 'r') as f:
                logs_data = yaml.safe_load(f)
            return jsonify(logs_data), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    
    def send_mission(self):
        """
        Handle HTTP POST request to send mission
        Receives mission data and publishes to ROS topic
        """
        try:
            data = request.get_json()

            if '_send_time' in data:
                send_time = data['_send_time']
                receive_time = time.time()
                self.latency_tracker.record_measurement('WSL22->WSL18', '/send_mission', 
                                                    send_time, receive_time)
                del data['_send_time']
            
            with self.data_lock:
                if self.mission_status_received:
                    time_since_status = time.time() - self.last_mission_status_time
                    if time_since_status < self.mission_completion_delay:
                        wait_time = self.mission_completion_delay - time_since_status
                        rospy.loginfo(
                            "[Bridge] Waiting {:.2f}s after mission completion "
                            "before sending new mission".format(wait_time))
                        time.sleep(wait_time)
                    
                    # Reset flag after delay
                    self.mission_status_received = False
            
            rospy.loginfo("[ROS Mission Bridge] Received mission request: %s", str(data))
            
            # Extract mission parameters
            mission_name = data.get("mission_name", "unknown")
            mission_id = data.get("mission_id", 0)
            mission_tag = data.get("mission_tag", 0)
            latitude = data.get("latitude", 0.0)
            longitude = data.get("longitude", 0.0)
            depth = data.get("depth", 0.0)
            yaw = data.get("yaw", 0.0)
            radius = data.get("radius", 0.0)
            use_yaw = data.get("use_yaw", False)
            use_spiral = data.get("use_spiral", False)
            color = data.get("color", "")
            
            # Create ROS message
            msg = SetInfoMissionManager()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.mission_id = mission_id
            msg.mission_tag = mission_tag
            msg.latitude = latitude
            msg.longitude = longitude
            msg.depth = depth
            msg.yaw = yaw
            msg.radius = radius
            msg.use_yaw = use_yaw
            msg.use_spiral = use_spiral
            msg.color = color
            
            # Publish the message
            with self.data_lock:
                self.mission_pub.publish(msg)
            
            rospy.loginfo(
                "[ROS Mission Bridge] Published mission '{}' to /gamma_bt/setInfo".format(
                    mission_name))
            
            return jsonify({
                "status": "success",
                "message": "Mission '{}' sent to ROS".format(mission_name),
                "mission_id": mission_id,
                "mission_tag": mission_tag
            }), 200
        
        except Exception as e:
            rospy.logerr("[ROS Mission Bridge] Error processing mission: %s", str(e))
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500
    
    def run(self):
        """Main loop"""
        rospy.loginfo("[ROS Mission Bridge] Running. Waiting for HTTP requests...")
        rospy.spin()

    def shutdown_hook(self):
        rospy.loginfo("Saving latency statistics...")
        self.latency_tracker.save_statistics()



if __name__ == '__main__':
    try:
        bridge = RosMissionBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass