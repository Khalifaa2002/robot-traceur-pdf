"""
hardware/simulator.py
=====================
Robot kinematic simulator for offline testing.

Simulates differential drive kinematics and encoder feedback.

Status: Production (migrated from v3.1 core)
"""

import numpy as np
from .base import RobotBase
from utils.config import RobotConfig, logger

class RobotSimulator(RobotBase):
    """
    Virtual robot for simulation testing.
    """
    
    def __init__(self):
        super().__init__()
        self.time = 0.0
        self.dt = 0.01
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.drawing = False
        
        self.wheel_diameter = RobotConfig.WHEEL_DIAMETER
        self.wheel_base = RobotConfig.WHEEL_BASE
        
        logger.info("🤖 Robot Simulator initialized")
    
    def connect(self) -> bool:
        self.is_connected = True
        self.state['battery'] = 12.0
        logger.info("✅ Simulator connected")
        return True
    
    def disconnect(self):
        self.is_connected = False
        logger.info("✅ Simulator disconnected")
    
    def send_command(self, command: str) -> bool:
        logger.debug(f"📤 Command: {command}")
        return True
    
    def read_state(self) -> dict:
        return self.state.copy()
    
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        self.motor_left = np.clip(left_speed, -1.0, 1.0)
        self.motor_right = np.clip(right_speed, -1.0, 1.0)
        
        # Simple velocity model (max speed 0.5 m/s)
        v = (self.motor_left + self.motor_right) / 2.0 * 0.5
        omega = (self.motor_right - self.motor_left) / self.wheel_base * 1.0
        
        perimeter = np.pi * self.wheel_diameter
        ticks_left = int((v - omega * self.wheel_base / 2) * self.dt / perimeter * RobotConfig.PPR)
        ticks_right = int((v + omega * self.wheel_base / 2) * self.dt / perimeter * RobotConfig.PPR)
        
        self.state['encoder_left'] += ticks_left
        self.state['encoder_right'] += ticks_right
        
        self.state['x'] += v * np.cos(self.state['theta']) * self.dt
        self.state['y'] += v * np.sin(self.state['theta']) * self.dt
        self.state['theta'] += omega * self.dt
        self.state['theta'] = np.arctan2(np.sin(self.state['theta']), np.cos(self.state['theta']))
        
        self.state['v'] = v
        self.state['omega'] = omega
        
        return True
    
    def set_draw(self, active: bool) -> bool:
        self.drawing = active
        logger.debug(f"✏️ Draw: {'ON' if active else 'OFF'}")
        return True
    
    def update(self):
        self.time += self.dt
        self.state['battery'] = 12.0 - 0.001 * self.time

    def get_current_speed(self) -> float:
        """Return current linear speed [m/s]."""
        return float(self.state.get('v', 0.0))

