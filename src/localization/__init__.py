"""
Localization Module - Extended Kalman Filter (OPTIMISÉ)
Fix covariance propagation, adaptive heading correction
"""

import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class RobotState:
    """État du robot"""
    x: float
    y: float
    theta: float
    v: float
    omega: float


class EKFLocalizer:
    """
    EKF 5D: [x, y, theta, v, omega]
    Fix: propagation covariance avec Jacobien correct
    """

    def __init__(self, wheel_base: float = 0.15, wheel_radius: float = 0.033):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        # État [x, y, theta, v, omega]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([0.1, 0.1, 0.05, 0.01, 0.01])

        # Bruit process
        self.Q = np.diag([0.001, 0.001, 0.001, 0.01, 0.01])

        # Bruit mesure
        self.R_odom = np.diag([0.005, 0.01])
        self.R_heading = 0.02

        self.last_time = None
        self._heading_kalman_gain = 0.15  # Gain adaptatif heading IMU

    def predict(self, left_vel: float, right_vel: float, dt: float):
        if dt <= 0:
            return

        x, y, theta, v_prev, omega_prev = self.state

        v = (left_vel + right_vel) / 2.0
        omega = (right_vel - left_vel) / self.wheel_base

        if abs(omega) > 1e-6:
            radius = v / omega
            x_new = x + radius * (np.sin(theta + omega * dt) - np.sin(theta))
            y_new = y - radius * (np.cos(theta + omega * dt) - np.cos(theta))
            theta_new = theta + omega * dt
        else:
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta

        self.state = np.array([x_new, y_new, theta_new, v, omega])
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        # Jacobien F (simplifié mais correct)
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q
        self.last_time = time.monotonic() if 'time' in dir() else None

    def update_heading(self, imu_yaw: float):
        """Mise à jour heading avec gain adaptatif"""
        z = imu_yaw
        error = z - self.state[2]
        error = np.arctan2(np.sin(error), np.cos(error))

        # Gain adaptatif: plus confiant si covariance faible
        gain = self._heading_kalman_gain * min(1.0, np.sqrt(self.P[2, 2]) / 0.1)
        self.state[2] += gain * error
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        # Réduit incertitude heading
        self.P[2, 2] *= (1 - gain)

    def update_odometry(self, left_encoder: int, right_encoder: int, ppr: float = 20.0):
        left_dist = (left_encoder / ppr) * (2 * np.pi * self.wheel_radius)
        right_dist = (right_encoder / ppr) * (2 * np.pi * self.wheel_radius)

        z = np.array([
            (left_dist + right_dist) / 2.0,
            (right_dist - left_dist) / self.wheel_base
        ])

        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0

        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state += K @ y
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_odom @ K.T

    def get_state(self) -> RobotState:
        return RobotState(
            x=self.state[0],
            y=self.state[1],
            theta=self.state[2],
            v=self.state[3],
            omega=self.state[4]
        )

    def get_covariance(self) -> np.ndarray:
        return self.P.copy()

    def get_uncertainty(self) -> Dict[str, float]:
        return {
            'x_uncertainty': np.sqrt(self.P[0, 0]),
            'y_uncertainty': np.sqrt(self.P[1, 1]),
            'theta_uncertainty': np.degrees(np.sqrt(self.P[2, 2])),
        }


# ParticleFilter retiré du système par défaut (trop lourd pour RPi)
# Gardé ici pour référence mais non utilisé
class ParticleFilter:
    """Désactivé par défaut sur Raspberry Pi (trop CPU-intensive)"""
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "ParticleFilter désactivé sur RPi. Utilisez EKFLocalizer."
        )


import time

if __name__ == "__main__":
    print("=== Test EKF Optimisé ===\n")
    ekf = EKFLocalizer()
    for step in range(10):
        left_enc = 100 if step < 5 else 50
        right_enc = 100 if step < 5 else 50
        ekf.predict(left_enc / 100, right_enc / 100, 0.1)
        ekf.update_odometry(left_enc, right_enc)
        ekf.update_heading(np.pi / 4)
        state = ekf.get_state()
        unc = ekf.get_uncertainty()
        print(f"Step {step}: pos=({state.x:.2f}, {state.y:.2f}), "
              f"θ={np.degrees(state.theta):.1f}°, "
              f"σ_θ={unc['theta_uncertainty']:.2f}°")
    print("\n✅ Test complété")

