"""
localization/odometry.py
========================
Differential drive odometry for robot pose estimation.

This module provides the standard wheel-encoder based odometry calculation.
It tracks the (x, y, theta) pose of the robot based on cumulative encoder ticks.

Status: Production (migrated from v3.1 core)
"""

import numpy as np
from typing import Tuple
from utils.config import RobotConfig, logger

class Localizer:
    """
    Tracks robot pose (x, y, theta) using differential drive odometry.
    
    Compatible with incremental quadrature encoders providing cumulative tick counts.
    Includes filtering for hardware jitter and overflow detection.
    """
    
    def __init__(self, config: RobotConfig = None):
        if config is None:
            config = RobotConfig()
        
        self.config = config
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        
        logger.info(f"Localizer initialized")
        logger.info(f"   Wheel diameter: {config.WHEEL_DIAMETER} m")
        logger.info(f"   Wheel base: {config.WHEEL_BASE} m")
        logger.info(f"   PPR: {config.PPR}")
    
    def update(self, encoder_left: int, encoder_right: int):
        """
        Update the pose with support for reversing and noise filtering.
        
        Args:
            encoder_left: CUMULATIVE (absolute) ticks for left wheel
            encoder_right: CUMULATIVE (absolute) ticks for right wheel
        """
        
        delta_left = encoder_left - self.last_encoder_left
        delta_right = encoder_right - self.last_encoder_right
        
        # Detection of hardware reset or overflow (massive jump > 50000 ticks)
        OVERFLOW_THRESHOLD = 50000
        if abs(delta_left) > OVERFLOW_THRESHOLD or abs(delta_right) > OVERFLOW_THRESHOLD:
            logger.warning(f"Encoder jump detected (L:{delta_left}, R:{delta_right}). Resetting counters.")
            self.last_encoder_left = encoder_left
            self.last_encoder_right = encoder_right
            return

        # Jitter filtering: ignore micro-oscillations
        NOISE_THRESHOLD = 0.1 
        if abs(delta_left) < NOISE_THRESHOLD: delta_left = 0
        if abs(delta_right) < NOISE_THRESHOLD: delta_right = 0
        
        if delta_left == 0 and delta_right == 0:
            return
            
        self.last_encoder_left = encoder_left
        self.last_encoder_right = encoder_right
        
        perimeter = np.pi * self.config.WHEEL_DIAMETER
        dist_left = (delta_left / self.config.PPR) * perimeter
        dist_right = (delta_right / self.config.PPR) * perimeter
        
        dist_avg = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.config.WHEEL_BASE
        
        self.x += dist_avg * np.cos(self.theta)
        self.y += dist_avg * np.sin(self.theta)
        self.theta += delta_theta
        # Normalize to [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Return (x, y, theta) pose."""
        return (self.x, self.y, self.theta)
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset the pose and zero the internal encoder counters."""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        logger.info(f"Localizer reset to ({x:.2f}, {y:.2f}, {theta:.2f})")
    
    def set_pose(self, x: float, y: float, theta: float):
        """Manually correct the pose (e.g. from EKF or external SLAM)."""
        self.x = x
        self.y = y
        self.theta = theta
        logger.info(f"Pose corrected to ({x:.2f}, {y:.2f}, {theta:.2f})")
    
    def __repr__(self) -> str:
        return f"Localizer(x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f})"
