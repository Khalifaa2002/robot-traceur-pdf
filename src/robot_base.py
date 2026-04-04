"""
Classe de base abstraite pour le robot
"""

from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np
from .config import logger

class RobotBase(ABC):
    """Interface abstraite pour le robot"""
    
    def __init__(self):
        self.is_connected = False
        self.state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'v': 0.0,
            'omega': 0.0,
            'battery': 0.0,
        }
    
    @abstractmethod
    def connect(self) -> bool:
        """Établit la connexion"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Ferme la connexion"""
        pass
    
    @abstractmethod
    def send_command(self, command: str) -> bool:
        """Envoie une commande"""
        pass
    
    @abstractmethod
    def read_state(self) -> dict:
        """Lit l'état"""
        pass
    
    @abstractmethod
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        """Commande les moteurs"""
        pass
    
    @abstractmethod
    def set_draw(self, active: bool) -> bool:
        """Active/désactive le traçage"""
        pass
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Retourne (x, y, theta)"""
        return (self.state['x'], self.state['y'], self.state['theta'])
    
    def get_velocity(self) -> Tuple[float, float]:
        """Retourne (v, omega)"""
        return (self.state['v'], self.state['omega'])
    
    def is_ready(self) -> bool:
        """Vérifie si prêt"""
        return (self.is_connected and self.state['battery'] > 9.0)
    
    def __repr__(self) -> str:
        status = "🟢 Connected" if self.is_connected else "🔴 Disconnected"
        return f"Robot({status})"


class RobotSimulator(RobotBase):
    """Simulateur du robot"""
    
    def __init__(self):
        super().__init__()
        self.time = 0.0
        self.dt = 0.01
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.drawing = False
        
        from .config import RobotConfig
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
        
        v = (self.motor_left + self.motor_right) / 2.0 * 0.5
        omega = (self.motor_right - self.motor_left) / self.wheel_base * 1.0
        
        self.state['x'] += v * np.cos(self.state['theta']) * self.dt
        self.state['y'] += v * np.sin(self.state['theta']) * self.dt
        self.state['theta'] += omega * self.dt
        self.state['theta'] = np.arctan2(np.sin(self.state['theta']), 
                                          np.cos(self.state['theta']))
        
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


if __name__ == "__main__":
    robot = RobotSimulator()
    if robot.connect():
        for i in range(100):
            robot.set_motor_speed(0.5, 0.5)
            robot.update()
            if i % 20 == 0:
                x, y, theta = robot.get_pose()
                print(f"Step {i}: x={x:.3f}, y={y:.3f}, θ={theta:.3f}")
        robot.disconnect()
        print("✅ Simulator test complete!")
