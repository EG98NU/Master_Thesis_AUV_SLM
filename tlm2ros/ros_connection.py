import requests
import json
import time
import csv
from pathlib import Path
from typing import List, Dict
import statistics

ROS_HOST = '192.168.45.222'
ROS_PORT = 5001
LATENCY_FILE = Path("latency_measurements.csv")

class LatencyTracker:
    """Track latency measurements for bidirectional communication"""
    
    def __init__(self, csv_file: Path = LATENCY_FILE):
        self.csv_file = csv_file
        self.measurements: List[Dict] = []
        self._initialize_csv()
    
    def _initialize_csv(self):
        """Initialize CSV file with headers"""
        if not self.csv_file.exists():
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'direction', 'endpoint', 'send_time', 'receive_time', 'latency_ms'])
    
    def record_measurement(self, direction: str, endpoint: str, send_time: float, receive_time: float):
        """Record a latency measurement"""
        latency_ms = (receive_time - send_time) * 1000
        
        measurement = {
            'timestamp': time.time(),
            'direction': direction,
            'endpoint': endpoint,
            'send_time': send_time,
            'receive_time': receive_time,
            'latency_ms': latency_ms
        }
        
        self.measurements.append(measurement)
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([measurement['timestamp'], measurement['direction'], 
                           measurement['endpoint'], measurement['send_time'], 
                           measurement['receive_time'], measurement['latency_ms']])
        
        print(f"[Latency] {direction} - {endpoint}: {latency_ms:.2f} ms")
    
    def compute_statistics(self) -> Dict:
        """Compute statistics for all measurements"""
        if not self.measurements:
            return {'total_measurements': 0, 'average_latency_ms': 0, 
                   'std_deviation_ms': 0, 'min_latency_ms': 0, 'max_latency_ms': 0}
        
        latencies = [m['latency_ms'] for m in self.measurements]
        stats = {
            'total_measurements': len(latencies),
            'average_latency_ms': statistics.mean(latencies),
            'std_deviation_ms': statistics.stdev(latencies) if len(latencies) > 1 else 0,
            'min_latency_ms': min(latencies),
            'max_latency_ms': max(latencies)
        }
        
        wsl22_to_wsl18 = [m['latency_ms'] for m in self.measurements if m['direction'] == 'WSL22->WSL18']
        wsl18_to_wsl22 = [m['latency_ms'] for m in self.measurements if m['direction'] == 'WSL18->WSL22']
        
        if wsl22_to_wsl18:
            stats['wsl22_to_wsl18_avg'] = statistics.mean(wsl22_to_wsl18)
            stats['wsl22_to_wsl18_std'] = statistics.stdev(wsl22_to_wsl18) if len(wsl22_to_wsl18) > 1 else 0
        
        if wsl18_to_wsl22:
            stats['wsl18_to_wsl22_avg'] = statistics.mean(wsl18_to_wsl22)
            stats['wsl18_to_wsl22_std'] = statistics.stdev(wsl18_to_wsl22) if len(wsl18_to_wsl22) > 1 else 0
        
        return stats
    
    def save_statistics(self, stats_file: Path = Path("latency_statistics.json")):
        """Save statistics to a JSON file"""
        stats = self.compute_statistics()
        
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"\n{'='*60}")
        print("LATENCY STATISTICS")
        print(f"{'='*60}")
        print(f"Total Measurements: {stats['total_measurements']}")
        print(f"Average Latency: {stats['average_latency_ms']:.2f} ms")
        print(f"Std Deviation: {stats['std_deviation_ms']:.2f} ms")
        print(f"Min Latency: {stats['min_latency_ms']:.2f} ms")
        print(f"Max Latency: {stats['max_latency_ms']:.2f} ms")
        
        if 'wsl22_to_wsl18_avg' in stats:
            print(f"\nWSL22 -> WSL18: Avg={stats['wsl22_to_wsl18_avg']:.2f} ms, StdDev={stats['wsl22_to_wsl18_std']:.2f} ms")
        if 'wsl18_to_wsl22_avg' in stats:
            print(f"WSL18 -> WSL22: Avg={stats['wsl18_to_wsl22_avg']:.2f} ms, StdDev={stats['wsl18_to_wsl22_std']:.2f} ms")
        print(f"{'='*60}")

latency_tracker = LatencyTracker()

def send_cmd_to_ros(command: dict, endpoint: str) -> dict:
    """Sends a command to the ROS bridge with latency tracking"""
    headers = {"Content-Type": "application/json"}
    ros_cmd_endpoint = f"http://{ROS_HOST}:{ROS_PORT}{endpoint}"
    
    send_time = time.time()
    command['_send_time'] = send_time
    
    try:
        response = requests.post(ros_cmd_endpoint, data=json.dumps(command), 
                               headers=headers, timeout=10)
        receive_time = time.time()
        response.raise_for_status()
        response_data = response.json()
        
        latency_tracker.record_measurement('WSL22->WSL18', endpoint, send_time, receive_time)
        return response_data
        
    except requests.Timeout as e:
        return {"status": "error", "message": f"Timeout connecting to ROS bridge"}
    except requests.ConnectionError as e:
        return {"status": "error", "message": "Cannot connect to ROS bridge"}
    except requests.RequestException as e:
        return {"status": "error", "message": str(e)}

def test_connection() -> bool:
    """Test if ROS bridge is reachable"""
    try:
        response = requests.get(f"http://{ROS_HOST}:{ROS_PORT}/health", timeout=5)
        return response.status_code == 200
    except:
        return False

def get_latency_tracker():
    """Get the global latency tracker instance"""
    return latency_tracker

