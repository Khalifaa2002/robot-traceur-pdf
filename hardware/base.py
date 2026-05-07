"""
hardware/base.py
================
Abstract base classes and factory for robot hardware abstraction.

This module defines the standard interface for robot implementations (Simulation,
Serial, GPIO) allowing the high-level controllers to remain hardware-agnostic.

Status: Production (migrated from v3.1 core)
"""

from abc import ABC, abstractmethod
from typing import Tuple, Dict
from utils.logger import logger

class RobotBase(ABC):
    """
    Interface for any robot platform.
    """
    
    def __init__(self):
        self.is_connected = False
        self.state = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0,
            'v': 0.0, 'omega': 0.0,
            'battery': 0.0,
            'encoder_left': 0, 'encoder_right': 0
        }
    
    @abstractmethod
    def connect(self) -> bool:
        """Establish connection with the hardware."""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Safely shutdown and close connections."""
        pass
    
    @abstractmethod
    def send_command(self, command: str) -> bool:
        """Send a raw command string."""
        pass
    
    @abstractmethod
    def read_state(self) -> dict:
        """Return the current internal state dictionary."""
        pass
    
    @abstractmethod
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        """Set speeds in range [-1.0, 1.0]."""
        pass
    
    @abstractmethod
    def set_draw(self, active: bool) -> bool:
        """Activate/deactivate the drawing tool (pen/laser)."""
        pass
    
    def update(self):
        """Optional update step (e.g. for simulation physics)."""
        pass

    def get_pose(self) -> Tuple[float, float, float]:
        return (self.state['x'], self.state['y'], self.state['theta'])
    
    def get_velocity(self) -> Tuple[float, float]:
        return (self.state['v'], self.state['omega'])
    
    def get_current_speed(self) -> float:
        """Return the linear velocity magnitude."""
        return float(self.state.get('v', 0.0))

    
    def is_ready(self) -> bool:
        return self.is_connected and self.state.get('battery', 0) > 9.0
    
    def __repr__(self) -> str:
        status = "🟢 Connected" if self.is_connected else "🔴 Disconnected"
        return f"{self.__class__.__name__}({status})"


class RobotFactory:
    """
    Factory to instantiate the appropriate robot implementation.
    """
    @staticmethod
    def create_robot(mode: str = "simulation") -> RobotBase:
        # Late imports to avoid circular dependencies and unnecessary overhead
        if mode in ("simulator", "simulation"):
            from .simulator import RobotSimulator
            logger.info("🎮 Creating RobotSimulator")
            return RobotSimulator()
        elif mode == "serial":
            from .serial import SerialRobotInterface
            logger.info("🤖 Creating SerialRobotInterface")
            return SerialRobotInterface()
        elif mode == "gpio":
            from .rpi_gpio import RPiGPIOInterface
            logger.info("🍓 Creating RPiGPIOInterface")
            return RPiGPIOInterface()
        else:
            raise ValueError(f"Unknown robot mode: {mode}")
