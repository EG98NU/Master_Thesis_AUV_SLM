#!/usr/bin/env python3

"""
Resource Monitor for ROS Mission Manager
Tracks CPU, memory, I/O usage, and session timing for each manager session
"""

import psutil
import time
import json
import numpy as np
from pathlib import Path
from datetime import datetime
from threading import Thread, Event

class ResourceMonitor:
    """Monitor computational resources for manager sessions"""

    def __init__(self, log_file="resource_usage.json", interval=1.0):
        """
        Initialize resource monitor

        Args:
            log_file: Path to JSON file storing resource usage data
            interval: Sampling interval in seconds
        """
        self.log_file = Path(log_file)
        self.interval = interval
        self.monitoring = False
        self.monitor_thread = None
        self.stop_event = Event()

        # Current session data
        self.session_data = {
            "start_time": None,
            "end_time": None,
            "start_timestamp": None,
            "end_timestamp": None,
            "duration_seconds": 0,
            "samples": [],
            "stats": {}
        }

        # Process tracking
        self.process = psutil.Process()
        self.start_cpu_times = None
        self.start_io_counters = None

    def start_monitoring(self):
        """Start resource monitoring in background thread"""
        if self.monitoring:
            print("⚠️  Resource monitor already running")
            return

        print("📊 Starting resource monitoring...")
        self.monitoring = True
        self.stop_event.clear()

        # Record session start with both human-readable and unix timestamp
        now = datetime.now()
        self.session_data["start_time"] = now.isoformat()
        self.session_data["start_timestamp"] = now.timestamp()

        self.start_cpu_times = self.process.cpu_times()

        try:
            self.start_io_counters = self.process.io_counters()
        except (AttributeError, OSError):
            self.start_io_counters = None

        # Start monitoring thread
        self.monitor_thread = Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("✅ Resource monitoring started")

    def stop_monitoring(self):
        """Stop resource monitoring and save session data"""
        if not self.monitoring:
            print("⚠️  Resource monitor not running")
            return

        print("📊 Stopping resource monitoring...")
        self.monitoring = False
        self.stop_event.set()

        if self.monitor_thread:
            self.monitor_thread.join(timeout=5.0)

        # Record session end with both human-readable and unix timestamp
        now = datetime.now()
        self.session_data["end_time"] = now.isoformat()
        self.session_data["end_timestamp"] = now.timestamp()

        # Calculate session duration
        if self.session_data["start_timestamp"] and self.session_data["end_timestamp"]:
            self.session_data["duration_seconds"] = (
                self.session_data["end_timestamp"] - self.session_data["start_timestamp"]
            )

        # Calculate statistics
        self._calculate_statistics()

        # Save session data
        self._save_session()

        print("✅ Resource monitoring stopped and saved")

        # Print session summary
        self._print_session_summary()

    def _monitor_loop(self):
        """Background monitoring loop"""
        while not self.stop_event.is_set():
            try:
                # Collect resource usage sample
                sample = self._collect_sample()
                self.session_data["samples"].append(sample)

            except Exception as e:
                print(f"⚠️  Error collecting resource sample: {e}")

            # Wait for next interval
            self.stop_event.wait(self.interval)

    def _collect_sample(self):
        """Collect single resource usage sample"""
        sample = {
            "timestamp": time.time(),
            "cpu_percent": self.process.cpu_percent(interval=None),
            "memory_mb": self.process.memory_info().rss / (1024 * 1024),
            "memory_percent": self.process.memory_percent(),
            "num_threads": self.process.num_threads()
        }

        # Add I/O counters if available
        try:
            io = self.process.io_counters()
            sample["io_read_mb"] = io.read_bytes / (1024 * 1024)
            sample["io_write_mb"] = io.write_bytes / (1024 * 1024)
        except (AttributeError, OSError):
            pass

        return sample

    def _calculate_statistics(self):
        """Calculate statistics from collected samples"""
        if not self.session_data["samples"]:
            return

        # Extract metrics
        cpu_values = [s["cpu_percent"] for s in self.session_data["samples"]]
        memory_mb_values = [s["memory_mb"] for s in self.session_data["samples"]]
        memory_percent_values = [s["memory_percent"] for s in self.session_data["samples"]]
        thread_values = [s["num_threads"] for s in self.session_data["samples"]]

        # Calculate CPU statistics
        self.session_data["stats"]["cpu"] = {
            "mean": float(np.mean(cpu_values)),
            "std": float(np.std(cpu_values)),
            "min": float(np.min(cpu_values)),
            "max": float(np.max(cpu_values)),
            "median": float(np.median(cpu_values))
        }

        # Calculate memory statistics (MB)
        self.session_data["stats"]["memory_mb"] = {
            "mean": float(np.mean(memory_mb_values)),
            "std": float(np.std(memory_mb_values)),
            "min": float(np.min(memory_mb_values)),
            "max": float(np.max(memory_mb_values)),
            "median": float(np.median(memory_mb_values))
        }

        # Calculate memory percentage statistics
        self.session_data["stats"]["memory_percent"] = {
            "mean": float(np.mean(memory_percent_values)),
            "std": float(np.std(memory_percent_values)),
            "min": float(np.min(memory_percent_values)),
            "max": float(np.max(memory_percent_values)),
            "median": float(np.median(memory_percent_values))
        }

        # Calculate thread statistics
        self.session_data["stats"]["threads"] = {
            "mean": float(np.mean(thread_values)),
            "std": float(np.std(thread_values)),
            "min": int(np.min(thread_values)),
            "max": int(np.max(thread_values)),
            "median": float(np.median(thread_values))
        }

        # Calculate I/O statistics if available
        io_read_values = [s.get("io_read_mb", 0) for s in self.session_data["samples"]]
        io_write_values = [s.get("io_write_mb", 0) for s in self.session_data["samples"]]

        if any(io_read_values):
            self.session_data["stats"]["io_read_mb"] = {
                "total": float(np.max(io_read_values)),
                "mean_rate": float(np.mean(np.diff(io_read_values))) if len(io_read_values) > 1 else 0
            }

            self.session_data["stats"]["io_write_mb"] = {
                "total": float(np.max(io_write_values)),
                "mean_rate": float(np.mean(np.diff(io_write_values))) if len(io_write_values) > 1 else 0
            }

        # Total samples collected
        self.session_data["stats"]["total_samples"] = len(self.session_data["samples"])

    def _save_session(self):
        """Save session data to log file"""
        # Load existing sessions
        if self.log_file.exists():
            try:
                with open(self.log_file, 'r') as f:
                    data = json.load(f)
            except (json.JSONDecodeError, FileNotFoundError):
                data = {"sessions": []}
        else:
            data = {"sessions": []}

        # Remove raw samples to save space (keep only statistics)
        session_to_save = {
            "start_time": self.session_data["start_time"],
            "end_time": self.session_data["end_time"],
            "start_timestamp": self.session_data["start_timestamp"],
            "end_timestamp": self.session_data["end_timestamp"],
            "duration_seconds": self.session_data["duration_seconds"],
            "stats": self.session_data["stats"]
        }

        # Append new session
        data["sessions"].append(session_to_save)

        # Write back to file
        with open(self.log_file, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"💾 Session data saved to: {self.log_file.absolute()}")

    def _print_session_summary(self):
        """Print summary of current session"""
        stats = self.session_data.get("stats", {})

        print("\n" + "="*60)
        print("📊 SESSION RESOURCE USAGE SUMMARY")
        print("="*60)

        # Session timing information
        if self.session_data["start_time"]:
            print(f"Start time: {self.session_data['start_time']}")
        if self.session_data["end_time"]:
            print(f"End time: {self.session_data['end_time']}")
        print(f"Duration: {self.session_data['duration_seconds']:.2f} seconds")
        print(f"Samples collected: {stats.get('total_samples', 0)}")

        if "cpu" in stats:
            print(f"\nCPU Usage (%): {stats['cpu']['mean']:.2f} ± {stats['cpu']['std']:.2f}")
            print(f"  Min: {stats['cpu']['min']:.2f}% | Max: {stats['cpu']['max']:.2f}%")

        if "memory_mb" in stats:
            print(f"\nMemory Usage (MB): {stats['memory_mb']['mean']:.2f} ± {stats['memory_mb']['std']:.2f}")
            print(f"  Min: {stats['memory_mb']['min']:.2f} MB | Max: {stats['memory_mb']['max']:.2f} MB")

        if "threads" in stats:
            print(f"\nThreads: {stats['threads']['mean']:.2f} ± {stats['threads']['std']:.2f}")
            print(f"  Min: {stats['threads']['min']} | Max: {stats['threads']['max']}")

        if "io_read_mb" in stats:
            print(f"\nI/O Read: {stats['io_read_mb']['total']:.2f} MB total")
            print(f"I/O Write: {stats['io_write_mb']['total']:.2f} MB total")

        print("="*60 + "\n")

def compute_aggregate_statistics(log_file="resource_usage.json"):
    """
    Compute aggregate statistics across all sessions

    Args:
        log_file: Path to resource usage log file

    Returns:
        Dictionary containing aggregate statistics
    """
    log_path = Path(log_file)

    if not log_path.exists():
        print(f"⚠️  No resource usage data found at: {log_path}")
        return None

    # Load session data
    try:
        with open(log_path, 'r') as f:
            data = json.load(f)
    except (json.JSONDecodeError, FileNotFoundError) as e:
        print(f"❌ Error loading resource data: {e}")
        return None

    sessions = data.get("sessions", [])

    if not sessions:
        print("⚠️  No sessions found in resource usage log")
        return None

    print(f"\n📈 Computing aggregate statistics from {len(sessions)} sessions...")

    # Collect metrics across all sessions
    metrics = {
        "cpu_mean": [],
        "cpu_max": [],
        "memory_mb_mean": [],
        "memory_mb_max": [],
        "memory_percent_mean": [],
        "duration_seconds": [],
        "threads_mean": []
    }

    for session in sessions:
        stats = session.get("stats", {})

        if "cpu" in stats:
            metrics["cpu_mean"].append(stats["cpu"]["mean"])
            metrics["cpu_max"].append(stats["cpu"]["max"])

        if "memory_mb" in stats:
            metrics["memory_mb_mean"].append(stats["memory_mb"]["mean"])
            metrics["memory_mb_max"].append(stats["memory_mb"]["max"])

        if "memory_percent" in stats:
            metrics["memory_percent_mean"].append(stats["memory_percent"]["mean"])

        if "threads" in stats:
            metrics["threads_mean"].append(stats["threads"]["mean"])

        if "duration_seconds" in session:
            metrics["duration_seconds"].append(session["duration_seconds"])

    # Calculate aggregate statistics
    aggregate = {
        "total_sessions": len(sessions),
        "metrics": {}
    }

    for metric_name, values in metrics.items():
        if values:
            aggregate["metrics"][metric_name] = {
                "mean": float(np.mean(values)),
                "std": float(np.std(values)),
                "min": float(np.min(values)),
                "max": float(np.max(values)),
                "median": float(np.median(values))
            }

    # Print aggregate statistics
    print("\n" + "="*60)
    print("📊 AGGREGATE STATISTICS ACROSS ALL SESSIONS")
    print("="*60)
    print(f"Total sessions: {aggregate['total_sessions']}")

    if "duration_seconds" in aggregate["metrics"]:
        dur = aggregate["metrics"]["duration_seconds"]
        print(f"\nSession Duration (seconds):")
        print(f"  Mean: {dur['mean']:.2f} ± {dur['std']:.2f}")
        print(f"  Min: {dur['min']:.2f} | Max: {dur['max']:.2f} | Median: {dur['median']:.2f}")

    if "cpu_mean" in aggregate["metrics"]:
        cpu = aggregate["metrics"]["cpu_mean"]
        print(f"\nAverage CPU Usage (%):")
        print(f"  Mean: {cpu['mean']:.2f} ± {cpu['std']:.2f}")
        print(f"  Min: {cpu['min']:.2f} | Max: {cpu['max']:.2f} | Median: {cpu['median']:.2f}")

    if "cpu_max" in aggregate["metrics"]:
        cpu_max = aggregate["metrics"]["cpu_max"]
        print(f"\nPeak CPU Usage (%):")
        print(f"  Mean: {cpu_max['mean']:.2f} ± {cpu_max['std']:.2f}")
        print(f"  Min: {cpu_max['min']:.2f} | Max: {cpu_max['max']:.2f} | Median: {cpu_max['median']:.2f}")

    if "memory_mb_mean" in aggregate["metrics"]:
        mem = aggregate["metrics"]["memory_mb_mean"]
        print(f"\nAverage Memory Usage (MB):")
        print(f"  Mean: {mem['mean']:.2f} ± {mem['std']:.2f}")
        print(f"  Min: {mem['min']:.2f} | Max: {mem['max']:.2f} | Median: {mem['median']:.2f}")

    if "memory_mb_max" in aggregate["metrics"]:
        mem_max = aggregate["metrics"]["memory_mb_max"]
        print(f"\nPeak Memory Usage (MB):")
        print(f"  Mean: {mem_max['mean']:.2f} ± {mem_max['std']:.2f}")
        print(f"  Min: {mem_max['min']:.2f} | Max: {mem_max['max']:.2f} | Median: {mem_max['median']:.2f}")

    if "threads_mean" in aggregate["metrics"]:
        threads = aggregate["metrics"]["threads_mean"]
        print(f"\nAverage Thread Count:")
        print(f"  Mean: {threads['mean']:.2f} ± {threads['std']:.2f}")
        print(f"  Min: {threads['min']:.2f} | Max: {threads['max']:.2f} | Median: {threads['median']:.2f}")

    print("="*60 + "\n")

    return aggregate

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        # Analyze existing data
        log_file = sys.argv[2] if len(sys.argv) > 2 else "resource_usage.json"
        compute_aggregate_statistics(log_file)
    else:
        print("ResourceMonitor - Usage:")
        print("  Import in your code: from resource_monitor import ResourceMonitor")
        print("  Or analyze data: python resource_monitor.py analyze [log_file]")