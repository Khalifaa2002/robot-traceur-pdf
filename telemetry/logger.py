"""
telemetry/logger.py
===================
Handles logging of mission data (pose, speed, error) for post-mission analysis.
"""

import os
import json
import csv
from datetime import datetime
from pathlib import Path
from utils.logger import logger

class MissionTelemetry:
    """
    Logs robot state and metrics to JSON/CSV for later analysis.
    """
    
    def __init__(self, mission_name: str = "mission"):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename_base = f"telemetry_{mission_name}_{self.timestamp}"
        self.log_dir = Path("data/telemetry")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.data = []
        logger.info(f"📊 Telemetry initialized: {self.log_dir}/{self.filename_base}")

    def record(self, pose: tuple, velocity: tuple, target: tuple, error: float):
        """Record a single state snapshot."""
        entry = {
            "time": datetime.now().isoformat(),
            "x": pose[0], "y": pose[1], "theta": pose[2],
            "v": velocity[0], "omega": velocity[1],
            "target_x": target[0], "target_y": target[1],
            "error": error
        }
        self.data.append(entry)

    def save(self):
        """Save all recorded data to disk."""
        if not self.data:
            return
            
        json_path = self.log_dir / f"{self.filename_base}.json"
        with open(json_path, 'w') as f:
            json.dump(self.data, f, indent=4)
            
        csv_path = self.log_dir / f"{self.filename_base}.csv"
        with open(csv_path, 'w', newline='') as f:
            if self.data:
                writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
                writer.writeheader()
                writer.writerows(self.data)
                
        logger.info(f"✅ Telemetry saved to {json_path}")
