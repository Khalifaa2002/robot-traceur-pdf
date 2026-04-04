"""
Tests unitaires basiques
"""

import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.robot_base import RobotSimulator
from src.localizer import Localizer
from src.pid_controller import PIDController
from src.robot_interface import RobotFactory


class TestRobotSimulator:
    """Tests du simulateur"""
    
    def test_connect(self):
        robot = RobotSimulator()
        assert robot.connect() == True
        robot.disconnect()
    
    def test_motor_control(self):
        robot = RobotSimulator()
        robot.connect()
        assert robot.set_motor_speed(0.5, 0.5) == True
        robot.disconnect()
    
    def test_draw_control(self):
        robot = RobotSimulator()
        robot.connect()
        assert robot.set_draw(True) == True
        assert robot.set_draw(False) == True
        robot.disconnect()
    
    def test_movement(self):
        robot = RobotSimulator()
        robot.connect()
        
        # Avance
        for _ in range(100):
            robot.set_motor_speed(0.5, 0.5)
            robot.update()
        
        x, y, theta = robot.get_pose()
        assert x > 0  # Devrait s'être déplacé
        assert y == 0  # Pas de virage
        robot.disconnect()


class TestLocalizer:
    """Tests du localizer"""
    
    def test_creation(self):
        localizer = Localizer()
        x, y, theta = localizer.get_pose()
        assert x == 0.0
        assert y == 0.0
        assert theta == 0.0
    
    def test_reset(self):
        localizer = Localizer()
        localizer.reset(1.0, 2.0, 0.5)
        x, y, theta = localizer.get_pose()
        assert x == 1.0
        assert y == 2.0
        assert theta == 0.5
    
    def test_update(self):
        localizer = Localizer()
        
        # Simule des encodeurs
        for i in range(10):
            localizer.update(i*10, i*10)
        
        x, y, theta = localizer.get_pose()
        assert x > 0  # S'est déplacé
        assert y == 0  # Pas de virage


class TestPIDController:
    """Tests du PID"""
    
    def test_creation(self):
        pid = PIDController(1.0, 0.1, 0.05)
        assert pid.kp == 1.0
    
    def test_update(self):
        pid = PIDController(1.0, 0.0, 0.0)
        output = pid.update(1.0, dt=0.01)
        assert output == 1.0  # P=1.0, I=0, D=0
    
    def test_limits(self):
        pid = PIDController(10.0, 0.0, 0.0, output_max=1.0)
        output = pid.update(10.0, dt=0.01)
        assert output == 1.0  # Clippé à 1.0


class TestRobotFactory:
    """Tests de la factory"""
    
    def test_create_simulator(self):
        robot = RobotFactory.create_robot("simulator")
        assert isinstance(robot, RobotSimulator)
    
    def test_invalid_mode(self):
        with pytest.raises(ValueError):
            RobotFactory.create_robot("invalid")


if __name__ == "__main__":
    # Lance les tests
    pytest.main([__file__, "-v"])
