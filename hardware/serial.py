"""
hardware/serial.py
==================
UART Serial interface for microcontroller-based robot control.

Handles JSON-encoded message exchange with Arduino/ESP32.

Status: Production (migrated from v3.1 core)
"""

import time
import json
import threading
from typing import Dict
from dataclasses import dataclass
from unittest import mock

try:
    import serial
except ImportError:
    serial = mock.Mock()

from .base import RobotBase
from utils.config import RobotConfig, logger

@dataclass
class RobotMessage:
    command: str
    params: Dict = None
    
    def to_json(self) -> str:
        msg = {'cmd': self.command}
        if self.params:
            msg.update(self.params)
        return json.dumps(msg)

class SerialRobotInterface(RobotBase):
    """
    Interface for robots controlled via UART (e.g. Arduino firmware).
    """
    
    def __init__(self, port: str = None, baudrate: int = 115200, timeout: float = 1.0):
        super().__init__()
        self.port = port or RobotConfig.SERIAL_PORT
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.reading_thread = None
        self.stop_reading = False
        logger.info(f"📡 SerialRobotInterface initialized (port={self.port})")
    
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(1) # Wait for bootloader
            self.is_connected = True
            self.stop_reading = False
            self.reading_thread = threading.Thread(target=self._reading_loop, daemon=True)
            self.reading_thread.start()
            logger.info(f"✅ Connected to {self.port}")
            return True
        except Exception as e:
            logger.error(f"❌ Serial connection error: {e}")
            logger.info("💡 Tip: Run 'python scripts/list_ports.py' to find the correct port.")
            self.is_connected = False
            return False

    
    def disconnect(self):
        if self.serial:
            self.stop_reading = True
            if self.reading_thread:
                self.reading_thread.join(timeout=2)
            self.serial.close()
            self.is_connected = False
            logger.info("✅ Serial disconnected")
    
    def send_command(self, command: str) -> bool:
        if not self.is_connected or not self.serial:
            return False
        try:
            self.serial.write((command + '\n').encode())
            return True
        except Exception as e:
            logger.error(f"Serial write error: {e}")
            return False
    
    def send_json_command(self, msg: RobotMessage) -> bool:
        return self.send_command(msg.to_json())
    
    def read_state(self) -> dict:
        return self.state.copy()
    
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        msg = RobotMessage("MOTOR", {
            'left': round(left_speed, 3),
            'right': round(right_speed, 3)
        })
        return self.send_json_command(msg)
    
    def set_draw(self, active: bool) -> bool:
        msg = RobotMessage("DRAW", {'active': active})
        return self.send_json_command(msg)
    
    def emergency_stop(self) -> bool:
        return self.send_command("STOP")
    
    def _reading_loop(self):
        buffer = ""
        while not self.stop_reading and self.is_connected:
            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode()
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._process_message(line.strip())
                time.sleep(0.01)
            except Exception as e:
                break
    
    def _process_message(self, line: str):
        if not line:
            return
        try:
            if line.startswith('{'):
                data = json.loads(line)
                self.state.update(data)
        except json.JSONDecodeError:
            pass
