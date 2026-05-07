"""
app/mission.py
==============
Mission orchestration and high-level trajectory following logic.

This module coordinates the robot, localizer, and controllers to execute
a path tracing mission.

Status: Production (migrated from v3.1 core)

FIXES (v2):
  - Fixed Pure Pursuit speed estimation: fallback to 0.2 m/s when v_prev ≈ 0
  - Fixed EKF circular update: removed update_odometry() call, predict-only mode
  - Fixed goal detection: now checks distance to final waypoint, not lookahead point
"""

import numpy as np
import time
from typing import Optional, Tuple, List

from utils.config import TrajectoryConfig, logger
from hardware.base import RobotBase
from localization.odometry import Localizer
from control.pid import create_linear_controller, create_angular_controller
from control.pure_pursuit import PurePursuitController, TargetCourse
from localization.ekf import EKF
from planning.dwa import DWAPlanner
from telemetry.logger import MissionTelemetry


class TrajectoryFollower:
    """
    Orchestrates the robot to follow a pre-calculated waypoint trajectory.
    Supports PID and Pure Pursuit control strategies.
    """
    
    def __init__(self, robot: RobotBase, localizer: Localizer, simulation: bool = False, controller_type: str = 'pid'):
        self.robot = robot
        self.localizer = localizer
        self.simulation = simulation
        self.controller_type = controller_type
        self.trajectory = None
        self.current_waypoint_idx = 0
        self.config = TrajectoryConfig()
        
        # Telemetry
        self.telemetry = MissionTelemetry(mission_name=controller_type)

        
        # Initialize Controllers
        self.pid_linear = create_linear_controller()
        self.pid_angular = create_angular_controller()
        self.pure_pursuit = PurePursuitController()
        self.target_course = None
        
        # Optional EKF for state estimation (dt matched to 100Hz loop)
        self.ekf = EKF(dt=0.01)
        self.use_ekf = False
        self.v_prev = 0.0
        self.omega_prev = 0.0
        self.dwa_planner = DWAPlanner()


        
        self.is_executing = False
        self.trajectory_complete = False
        self.metrics = {}
        logger.info(f"🎯 TrajectoryFollower initialized (controller: {controller_type})")
    
    def load_trajectory(self, trajectory: np.ndarray) -> bool:
        """Load a trajectory array of shape (N, 3) where columns are [x, y, theta]."""
        if trajectory is None or len(trajectory) == 0:
            logger.error("Invalid trajectory")
            return False
        self.trajectory = trajectory
        self.current_waypoint_idx = 0
        self.trajectory_complete = False
        
        if self.controller_type == 'pure_pursuit':
            self.target_course = TargetCourse(trajectory[:, 0], trajectory[:, 1])
            
        logger.info(f"📋 Trajectory loaded: {len(trajectory)} waypoints")
        return True
    
    def get_current_waypoint(self) -> Optional[Tuple[float, float, float]]:
        """Get the current target waypoint [x, y, theta]."""
        if self.trajectory is None or self.current_waypoint_idx >= len(self.trajectory):
            return None
        wp = self.trajectory[self.current_waypoint_idx]
        return (float(wp[0]), float(wp[1]), float(wp[2]))
    
    def advance_waypoint(self):
        """Move to the next waypoint and reset PID integrals."""
        self.current_waypoint_idx += 1
        self.pid_linear.reset()
        self.pid_angular.reset()
        if self.current_waypoint_idx >= len(self.trajectory):
            self.trajectory_complete = True
            logger.info("✅ Trajectory complete!")
    
    def follow(self, max_time: float = 300.0, max_velocity: float = 0.5) -> bool:
        """
        Execute the following loop. Blocks until complete or timeout.
        """
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
                    
                x, y, theta = self.localizer.get_pose()
                
                # --- EKF UPDATE (State Estimation) ---
                if self.use_ekf:
                    # Pure dead-reckoning EKF:
                    # Predict forward using PREVIOUS command (v, omega)
                    # ✅ FIX: No update() step — odometry IS the state estimate source
                    self.ekf.predict(self.v_prev, self.omega_prev)
                    x, y, theta = self.ekf.get_pose()



                # --- DWA OBSTACLE AVOIDANCE OVERRIDE ---
                obstacles = np.zeros((0, 2))  # placeholder: replace with real sensor data
                if len(obstacles) > 0 and hasattr(self, 'dwa_planner'):
                    robot_state = np.array([x, y, theta, self.v_prev, self.omega_prev])
                    if self.trajectory is not None and self.current_waypoint_idx < len(self.trajectory):
                        goal = self.trajectory[min(self.current_waypoint_idx + 5, len(self.trajectory)-1), :2]
                        v_cmd, omega_cmd = self.dwa_planner.compute(robot_state, goal, obstacles)
                        v_left, v_right = self.dwa_planner.to_wheel_speeds(v_cmd, omega_cmd)
                        self.robot.set_motor_speed(v_left, v_right)
                        self.v_prev = v_cmd
                        self.omega_prev = omega_cmd
                        continue

                # --- CONTROL LOGIC ---
                if self.controller_type == 'pure_pursuit':
                    # ✅ FIX: Use last known v_cmd as speed estimate with fallback
                    # Avoids get_current_speed() issue and handles startup (v=0)
                    current_speed = abs(self.v_prev) if abs(self.v_prev) > 0.01 else 0.2
                    
                    v_cmd, omega_cmd, look_ind = self.pure_pursuit.compute(
                        x, y, theta, current_speed, self.target_course
                    )
                    
                    # Error = distance to current lookahead point
                    tx = self.target_course.cx[look_ind]
                    ty = self.target_course.cy[look_ind]
                    dist_error = np.sqrt((tx - x)**2 + (ty - y)**2)
                    errors.append(dist_error)
                    
                    # --- WAYPOINT ADVANCEMENT ---
                    if look_ind > self.current_waypoint_idx:
                        self.current_waypoint_idx = look_ind
                        logger.debug(f"📍 PP progress: {self.current_waypoint_idx}/{len(self.trajectory)}")
                    
                    # ✅ FIX: Goal check: distance to FINAL waypoint (not lookahead)
                    final_x = self.target_course.cx[-1]
                    final_y = self.target_course.cy[-1]
                    dist_to_goal = np.sqrt((final_x - x)**2 + (final_y - y)**2)
                    
                    if dist_to_goal < self.config.LINEAR_TOLERANCE:
                        self.trajectory_complete = True
                        self.current_waypoint_idx = len(self.trajectory) - 1
                        logger.info("✅ Goal reached (Pure Pursuit)")
                        break

                        
                else:

                    # Legacy PID Logic
                    waypoint = self.get_current_waypoint()
                    if waypoint is None:
                        break
                    
                    x_target, y_target, theta_target = waypoint
                    
                    dx = x_target - x
                    dy = y_target - y
                    dist_error = np.sqrt(dx**2 + dy**2)
                    
                    angle_target = np.arctan2(dy, dx)
                    angle_error = angle_target - theta
                    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                    
                    errors.append(dist_error)
                    
                    # 1. PID Command Calculation
                    v_cmd = self.pid_linear.update(dist_error, dt=0.01)
                    omega_cmd = self.pid_angular.update(angle_error, dt=0.01)
                    
                    # 2. Adaptive Velocity Scaling
                    v_scale = max(0.0, 1.0 - abs(angle_error) / (np.pi / 2))
                    v_cmd *= v_scale
                    
                    if dist_error < self.config.LINEAR_TOLERANCE:
                        logger.info(f"✅ Waypoint {self.current_waypoint_idx} reached")
                        self.advance_waypoint()

                # --- SATURATION & OUTPUT ---
                v_cmd = np.clip(v_cmd, -max_velocity, max_velocity)
                omega_cmd = np.clip(omega_cmd, -1.0, 1.0)
                
                v_left = v_cmd - (self.localizer.config.WHEEL_BASE / 2) * omega_cmd
                v_right = v_cmd + (self.localizer.config.WHEEL_BASE / 2) * omega_cmd
                
                # Normalize wheel speeds to [-1.0, 1.0]
                max_v = max(abs(v_left), abs(v_right), 1e-6)
                if max_v > 1.0:
                    v_left /= max_v
                    v_right /= max_v
                
                self.robot.set_motor_speed(v_left, v_right)
                
                # Store commands for next EKF prediction step
                self.v_prev = v_cmd
                self.omega_prev = omega_cmd

                
                # --- TELEMETRY RECORDING ---
                if self.controller_type == 'pure_pursuit':
                    target = (tx, ty)
                else:
                    target = (x_target, y_target)
                self.telemetry.record((x, y, theta), (v_cmd, omega_cmd), target, dist_error)
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.warning("⚠️ Interrupted")
        except Exception as e:
            logger.error(f"❌ Error: {e}")
        
        finally:
            self.robot.set_motor_speed(0, 0)
            self.is_executing = False
            self.telemetry.save()

        
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
