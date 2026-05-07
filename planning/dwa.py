"""
planning/dwa.py
===============
Dynamic Window Approach (DWA) for local obstacle avoidance.

Adapted from PythonRobotics/PathPlanning/DynamicWindowApproach
Optimized for Raspberry Pi: reduced sampling resolution and prediction time.

FIXES (v2):
  - Fixed v_max saturation issue: now guarantees exploitable velocity window
  - Prevents np.arange(v_min, v_max, resolution) from returning single value
"""

import numpy as np
import math
from utils.config import RobotConfig, logger

class DWAConfig:
    """Configuration for DWA Planner."""
    max_speed: float = 0.4      # [m/s]
    min_speed: float = 0.0      # [m/s]
    max_yaw_rate: float = 1.5   # [rad/s]
    max_accel: float = 0.3      # [m/s²]
    max_delta_yaw_rate: float = 1.5 # [rad/s²]
    v_resolution: float = 0.05  # [m/s]
    yaw_rate_resolution: float = 0.1 # [rad/s]
    dt: float = 0.1             # [s]
    predict_time: float = 1.0   # [s]
    to_goal_cost_gain: float = 0.15
    speed_cost_gain: float = 1.0
    obstacle_cost_gain: float = 2.0
    robot_radius: float = 0.12  # [m]

class DWAPlanner:
    """Dynamic Window Approach local planner."""
    
    def __init__(self, config: DWAConfig = None):
        self.config = config if config is not None else DWAConfig()
        self.wheel_base = RobotConfig.WHEEL_BASE

    def compute(self,
                robot_state: np.ndarray,
                goal: np.ndarray,
                obstacles: np.ndarray) -> tuple[float, float]:
        """
        Compute best (v, omega) to reach goal while avoiding obstacles.
        
        Args:
            robot_state: [x, y, theta, v, omega]
            goal:        [gx, gy]
            obstacles:   np.ndarray shape (N, 2)
            
        Returns:
            (v_cmd, omega_cmd)
        """
        dw = self._calc_dynamic_window(robot_state)
        
        best_u = [0.0, 0.0]
        min_cost = float("inf")
        
        # Search window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                
                # Predict trajectory
                trajectory = self._predict_trajectory(
                    robot_state[0], robot_state[1], robot_state[2], v, omega)
                
                # Calculate costs
                to_goal_cost = self.config.to_goal_cost_gain * self._calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.config.obstacle_cost_gain * self._calc_obstacle_cost(trajectory, obstacles)
                
                final_cost = to_goal_cost + speed_cost + ob_cost
                
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, omega]
        
        return best_u[0], best_u[1]

    def to_wheel_speeds(self, v: float, omega: float) -> tuple[float, float]:
        """
        Convert (v, omega) to (v_left, v_right) normalized to [-1.0, 1.0].
        """
        L = self.wheel_base
        v_left = v - omega * L / 2.0
        v_right = v + omega * L / 2.0
        
        # Normalize if beyond max speed
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > 0.5: # Using a safe max speed threshold
            v_left = (v_left / max_v) * 1.0
            v_right = (v_right / max_v) * 1.0
        else:
            # Map [0, 0.5] to [0, 1.0] roughly
            v_left = v_left / 0.5
            v_right = v_right / 0.5
            
        return float(np.clip(v_left, -1.0, 1.0)), float(np.clip(v_right, -1.0, 1.0))

    def _calc_dynamic_window(self, state: np.ndarray) -> list:
        """Compute achievable velocities within hardware constraints."""
        # Hardware limits
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic constraints based on current state
        v_min_dynamic = state[3] - self.config.max_accel * self.config.dt
        v_max_dynamic = state[3] + self.config.max_accel * self.config.dt
        omega_min_dynamic = state[4] - self.config.max_delta_yaw_rate * self.config.dt
        omega_max_dynamic = state[4] + self.config.max_delta_yaw_rate * self.config.dt
        
        # Ensure v_max respects both hardware max speed and reachable acceleration
        v_max = min(self.config.max_speed, v_max_dynamic)
        
        # ✅ FIX: Garantir une plage exploitable
        # Assure that v_max > v_min + v_resolution so that np.arange generates multiple values
        v_max = max(v_max, self.config.min_speed + self.config.v_resolution)
        
        v_min = max(self.config.min_speed, v_min_dynamic)
        omega_min = max(-self.config.max_yaw_rate, omega_min_dynamic)
        omega_max = min(self.config.max_yaw_rate, omega_max_dynamic)

        # Assemble final dynamic window [v_min, v_max, omega_min, omega_max]
        return [v_min, v_max, omega_min, omega_max]

    def _predict_trajectory(self, x, y, theta, v, omega) -> np.ndarray:
        """Simulate trajectory over predict_time."""
        trajectory = np.array([x, y, theta, v, omega])
        time = 0.0
        while time <= self.config.predict_time:
            x += v * math.cos(theta) * self.config.dt
            y += v * math.sin(theta) * self.config.dt
            theta += omega * self.config.dt
            trajectory = np.vstack((trajectory, [x, y, theta, v, omega]))
            time += self.config.dt
        return trajectory

    def _calc_to_goal_cost(self, trajectory: np.ndarray, goal: np.ndarray) -> float:
        """Goal cost based on final heading towards target."""
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        return abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    def _calc_obstacle_cost(self, trajectory: np.ndarray, obstacles: np.ndarray) -> float:
        """Obstacle cost based on proximity along the path."""
        if len(obstacles) == 0:
            return 0.0
        
        # Vectorized distance calculation
        # trajectory[:, :2] is (M, 2), obstacles is (N, 2)
        # We need distances between all M trajectory points and all N obstacles
        # Use broadcasting: (M, 1, 2) - (1, N, 2) -> (M, N, 2)
        diff = trajectory[:, np.newaxis, :2] - obstacles[np.newaxis, :, :]
        dist_sq = np.sum(diff**2, axis=2)
        min_dist = np.sqrt(np.min(dist_sq))
        
        if min_dist <= self.config.robot_radius:
            return float("inf")
        
        return 1.0 / min_dist

def ultrasonic_to_obstacles(front_m: float, back_m: float,
                          left_m: float, right_m: float,
                          robot_x: float, robot_y: float,
                          robot_theta: float,
                          max_range: float = 0.6) -> np.ndarray:
    """
    Convert ultrasonic sensor readings to world-frame obstacle points.
    """
    obstacles = []
    sensors = [
        (front_m, 0),
        (back_m, math.pi),
        (left_m, math.pi/2),
        (right_m, -math.pi/2)
    ]
    
    for dist, angle_offset in sensors:
        if dist < max_range:
            angle = robot_theta + angle_offset
            ox = robot_x + dist * math.cos(angle)
            oy = robot_y + dist * math.sin(angle)
            obstacles.append([ox, oy])
            
    return np.array(obstacles) if obstacles else np.zeros((0, 2))
