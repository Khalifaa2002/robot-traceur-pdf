"""
Contrôle du suivi de trajectoire
"""

import numpy as np
import time
from typing import List, Tuple, Optional
from .robot_base import RobotBase
from .localizer import Localizer
from .pid_controller import create_linear_controller, create_angular_controller
from .config import TrajectoryConfig, logger

class TrajectoryFollower:
    """Fait suivre une trajectoire au robot"""
    
    def __init__(self, robot: RobotBase, localizer: Localizer, simulation: bool = False):
        self.robot = robot
        self.localizer = localizer
        self.simulation = simulation
        self.trajectory = None
        self.current_waypoint_idx = 0
        self.config = TrajectoryConfig()
        self.pid_linear = create_linear_controller()
        self.pid_angular = create_angular_controller()
        self.is_executing = False
        self.trajectory_complete = False
        logger.info("🎯 TrajectoryFollower initialized")
    
    def load_trajectory(self, trajectory: np.ndarray) -> bool:
        if trajectory is None or len(trajectory) == 0:
            logger.error("Invalid trajectory")
            return False
        self.trajectory = trajectory
        self.current_waypoint_idx = 0
        self.trajectory_complete = False
        logger.info(f"📋 Trajectory loaded: {len(trajectory)} waypoints")
        return True
    
    def get_current_waypoint(self) -> Optional[Tuple[float, float, float]]:
        if self.trajectory is None or self.current_waypoint_idx >= len(self.trajectory):
            return None
        wp = self.trajectory[self.current_waypoint_idx]
        return (float(wp[0]), float(wp[1]), float(wp[2]))
    
    def advance_waypoint(self):
        self.current_waypoint_idx += 1
        self.pid_linear.reset()
        self.pid_angular.reset()
        if self.current_waypoint_idx >= len(self.trajectory):
            self.trajectory_complete = True
            logger.info("✅ Trajectory complete!")
    
    def follow(self, max_time: float = 300.0, max_velocity: float = 0.5) -> bool:
        if self.trajectory is None:
            logger.error("No trajectory loaded")
            return False
        
        logger.info(f"🚀 Starting trajectory following")
        self.is_executing = True
        start_time = time.time()
        errors = []
        
        try:
            while self.is_executing and time.time() - start_time < max_time:
                self.robot.update()
                
                if self.simulation:
                    # Sync localizer with robot simulator state to fix desync
                    x, y, theta = self.robot.get_pose()
                    self.localizer.set_pose(x, y, theta)
                    
                waypoint = self.get_current_waypoint()
                if waypoint is None:
                    break
                
                x_target, y_target, theta_target = waypoint
                x, y, theta = self.localizer.get_pose()
                
                dx = x_target - x
                dy = y_target - y
                dist_error = np.sqrt(dx**2 + dy**2)
                
                angle_target = np.arctan2(dy, dx)
                angle_error = angle_target - theta
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                
                errors.append(dist_error)
                
                v_cmd = self.pid_linear.update(dist_error, dt=0.01)
                omega_cmd = self.pid_angular.update(angle_error, dt=0.01)
                
                v_cmd = np.clip(v_cmd, -max_velocity, max_velocity)
                omega_cmd = np.clip(omega_cmd, -1.0, 1.0)
                
                v_left = v_cmd - (self.localizer.config.WHEEL_BASE / 2) * omega_cmd
                v_right = v_cmd + (self.localizer.config.WHEEL_BASE / 2) * omega_cmd
                
                max_v = max(abs(v_left), abs(v_right), 1e-6)
                if max_v > 1.0:
                    v_left /= max_v
                    v_right /= max_v
                
                self.robot.set_motor_speed(v_left, v_right)
                
                if (dist_error < self.config.LINEAR_TOLERANCE and 
                    abs(angle_error) < self.config.ANGULAR_TOLERANCE):
                    logger.info(f"✅ Waypoint {self.current_waypoint_idx} reached")
                    self.advance_waypoint()
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.warning("⚠️ Interrupted")
        except Exception as e:
            logger.error(f"❌ Error: {e}")
        
        finally:
            self.robot.set_motor_speed(0, 0)
            self.is_executing = False
        
        elapsed = time.time() - start_time
        if errors:
            errors = np.array(errors)
            logger.info(f"\n📊 Statistics:")
            logger.info(f"   Time: {elapsed:.1f}s")
            logger.info(f"   Error mean: {errors.mean():.4f}m")
            logger.info(f"   Error max: {errors.max():.4f}m")
        
        return self.trajectory_complete

# ✅ FIXED: [BUG 1: Robot/Localizer desynchronization, BUG 6: missing update() call, BUG 7: PID integral windup reset, BUG 8: Angle normalization verified]
