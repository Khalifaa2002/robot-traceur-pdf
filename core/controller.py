import numpy as np
import time
from typing import Optional, Tuple, List

from utils.config import PIDConfig, TrajectoryConfig
from utils.logger import logger
from .hardware import RobotBase
from .localization import Localizer

# ==================== PID CONTROLLER ====================

class PIDController:
    """Contrôleur PID"""
    
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_max: float = 1.0,
                 name: str = "PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self.name = name
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        
        logger.info(f"🔧 PIDController '{name}' initialized (Kp={kp}, Ki={ki}, Kd={kd})")
    
    def update(self, error: float, dt: float = 0.01) -> float:
        """Calcule la correction PID"""
        
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral
        
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)
        
        self.prev_error = error
        self.prev_time += dt
        
        return output
    
    def reset(self):
        """Réinitialise"""
        self.integral = 0.0
        self.prev_error = 0.0
        logger.debug(f"🔄 PIDController '{self.name}' reset")
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """Change les gains"""
        logger.info(f"📊 PIDController '{self.name}' gains updated:")
        logger.info(f"   Kp: {self.kp:.3f} → {kp:.3f}")
        logger.info(f"   Ki: {self.ki:.3f} → {ki:.3f}")
        logger.info(f"   Kd: {self.kd:.3f} → {kd:.3f}")
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def __repr__(self) -> str:
        return (f"PIDController({self.name}, "
                f"Kp={self.kp:.3f}, Ki={self.ki:.3f}, Kd={self.kd:.3f})")


def create_linear_controller(config: PIDConfig = None) -> PIDController:
    """Crée un PID pour le contrôle linéaire"""
    if config is None:
        config = PIDConfig()
    return PIDController(
        config.LINEAR_KP,
        config.LINEAR_KI,
        config.LINEAR_KD,
        integral_max=config.INTEGRAL_MAX,
        name="Linear PID"
    )


def create_angular_controller(config: PIDConfig = None) -> PIDController:
    """Crée un PID pour le contrôle angulaire"""
    if config is None:
        config = PIDConfig()
    return PIDController(
        config.ANGULAR_KP,
        config.ANGULAR_KI,
        config.ANGULAR_KD,
        integral_max=config.INTEGRAL_MAX,
        name="Angular PID"
    )

# ==================== TRAJECTORY FOLLOWER ====================

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
                
                # 1. Calcul des commandes PID de base
                v_cmd = self.pid_linear.update(dist_error, dt=0.01)
                omega_cmd = self.pid_angular.update(angle_error, dt=0.01)
                
                # 2. Adaptive Velocity Scaling (Stabilité accrue)
                # On réduit la vitesse linéaire si l'erreur angulaire est importante.
                # Si le robot n'est pas aligné, il doit d'abord pivoter avant de foncer.
                # Facteur d'atténuation : 1.0 si aligné, 0.0 si à 90° ou plus.
                v_scale = max(0.0, 1.0 - abs(angle_error) / (np.pi / 2))
                v_cmd *= v_scale
                
                # 3. Saturation des commandes
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
        
        self.metrics = {
            "success": self.trajectory_complete,
            "completion_rate": float(self.current_waypoint_idx / len(self.trajectory)) if self.trajectory is not None and len(self.trajectory) > 0 else 0.0,
            "time_elapsed_s": float(elapsed),
            "rms_error_m": 0.0,
            "max_error_m": 0.0
        }
        
        if errors:
            errors = np.array(errors)
            rms_error = np.sqrt(np.mean(errors**2))
            self.metrics["rms_error_m"] = float(rms_error)
            self.metrics["max_error_m"] = float(errors.max())
            
            logger.info(f"\n📊 Statistics:")
            logger.info(f"   Time: {elapsed:.1f}s")
            logger.info(f"   RMS Error: {rms_error:.4f}m")
            logger.info(f"   Max Error: {errors.max():.4f}m")
            logger.info(f"   Completion: {self.metrics['completion_rate']*100:.1f}%")
        
        return self.trajectory_complete
