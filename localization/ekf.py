"""
localization/ekf.py
====================
Extended Kalman Filter (EKF) for differential drive robot localization.

Adapted from PythonRobotics/Localization/extended_kalman_filter
Original author: Atsushi Sakai (@Atsushi_twi)
Adaptation:
  - State reduced to [x, y, theta]^T (3-DOF, no velocity state)
  - Input: [v, omega] — linear and angular velocity from encoder odometry
  - No GPS/landmark observation required (pure dead-reckoning with EKF covariance)
  - Extendable: add_observation(z, H, R) for future IMU / GPS fusion
  - Stripped of all matplotlib, animation, simulation harness
  - Pure numpy, no external dependencies beyond numpy

Physical model:
  x_{t+1} = x_t + v * cos(theta) * dt
  y_{t+1} = y_t + v * sin(theta) * dt
  theta_{t+1} = theta_t + omega * dt

State vector: x = [x, y, theta]^T
Input vector: u = [v, omega]^T

Noise parameters (tunable via constructor):
  Q — process noise covariance (3x3)    — reflects model uncertainty
  R — observation noise covariance (2x2) — used when GPS/landmarks available

Raspberry Pi compatible: numpy only, no matplotlib.

Classes:
    EKFLocalizer — full EKF state estimator
"""

import math
import numpy as np


# ── Default noise matrices (conservative) ────────────────────────────────────

# Process noise: position grows slowly, heading more uncertain
_DEFAULT_Q = np.diag([
    0.02,              # x [m]
    0.02,              # y [m]
    np.deg2rad(2.0),   # theta [rad]
]) ** 2

# Observation noise (x, y in metres) — used when GPS/landmark observation arrives
_DEFAULT_R = np.diag([0.3, 0.3]) ** 2


class EKF:

    """
    Extended Kalman Filter localizer for a differential drive robot.

    Uses encoder-derived (v, omega) inputs and propagates position uncertainty
    over time. Can fuse position observations (GPS, landmark) via update().

    Parameters
    ----------
    x0, y0, theta0 : float — initial pose [m, m, rad]
    dt             : float — control loop timestep [s]
    Q              : ndarray (3,3) — process noise covariance
    R              : ndarray (2,2) — observation noise covariance

    Attributes
    ----------
    x_est : ndarray (3,1) — current state estimate [x, y, theta]
    P_est : ndarray (3,3) — current covariance estimate
    """

    def __init__(self,
                 x0: float = 0.0,
                 y0: float = 0.0,
                 theta0: float = 0.0,
                 dt: float = 0.05,
                 Q: np.ndarray = None,
                 R: np.ndarray = None):
        self.dt = dt
        self.Q = Q if Q is not None else _DEFAULT_Q.copy()
        self.R = R if R is not None else _DEFAULT_R.copy()

        # State estimate: [x, y, theta]^T
        self.x_est = np.array([[x0], [y0], [theta0]], dtype=float)
        # Covariance: start with moderate uncertainty
        self.P_est = np.eye(3) * 0.01

    # ── Public API ──────────────────────────────────────────────────────────

    def predict(self, v: float, omega: float):
        """
        EKF Predict step: propagate state and covariance forward by one timestep.

        Parameters
        ----------
        v     : float — linear velocity [m/s]  (from encoder odometry)
        omega : float — angular velocity [rad/s]
        """
        u = np.array([[v], [omega]])

        # Predicted state via motion model
        x_pred = self._motion_model(self.x_est, u)

        # Jacobian of motion model wrt state
        jF = self._jacobian_F(self.x_est, u)

        # Predicted covariance
        P_pred = jF @ self.P_est @ jF.T + self.Q

        self.x_est = x_pred
        self.P_est = P_pred

    def update(self, z: np.ndarray):
        """
        EKF Update step: fuse a position observation [x_obs, y_obs].

        Call this when an external observation (GPS, ArUco landmark, etc.) is
        available. Skip this method if only dead-reckoning is used.

        Parameters
        ----------
        z : ndarray (2,1) — observed position [x_obs, y_obs]
        """
        z = np.array(z).reshape(2, 1)

        # Observation model: H maps state → observed [x, y]
        jH = self._jacobian_H()
        z_pred = jH @ self.x_est

        # Innovation
        y = z - z_pred

        # Kalman gain
        S = jH @ self.P_est @ jH.T + self.R
        K = self.P_est @ jH.T @ np.linalg.inv(S)

        # Updated state and covariance
        self.x_est = self.x_est + K @ y
        # Normalize theta to [-pi, pi]
        self.x_est[2, 0] = math.atan2(
            math.sin(self.x_est[2, 0]),
            math.cos(self.x_est[2, 0])
        )
        self.P_est = (np.eye(3) - K @ jH) @ self.P_est

    def update_odometry(self, x: float, y: float, theta: float):
        """
        Wrapper to update EKF with odometry observation.
        """
        z = np.array([[x], [y]])
        self.update(z)


    def get_pose(self):
        """
        Return current best-estimate pose.

        Returns
        -------
        x     : float — estimated x position [m]
        y     : float — estimated y position [m]
        theta : float — estimated heading [rad], normalized to [-pi, pi]
        """
        return (
            float(self.x_est[0, 0]),
            float(self.x_est[1, 0]),
            float(self.x_est[2, 0]),
        )

    def get_covariance(self) -> np.ndarray:
        """Return current 3x3 state covariance matrix."""
        return self.P_est.copy()

    def get_position_uncertainty(self) -> float:
        """
        Return scalar position uncertainty (sqrt of position covariance trace).
        Useful as a confidence metric in telemetry reports.
        """
        return float(math.sqrt(self.P_est[0, 0] + self.P_est[1, 1]))

    def reset(self, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0):
        """Reset the filter to a given pose with fresh covariance."""
        self.x_est = np.array([[x0], [y0], [theta0]], dtype=float)
        self.P_est = np.eye(3) * 0.01

    # ── Private: motion model and Jacobians ──────────────────────────────────

    def _motion_model(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Nonlinear differential drive motion model.

        x_{t+1} = x_t + v * cos(theta) * dt
        y_{t+1} = y_t + v * sin(theta) * dt
        theta_{t+1} = theta_t + omega * dt
        """
        theta = float(x[2, 0])
        v     = float(u[0, 0])
        omega = float(u[1, 0])
        dt    = self.dt

        x_new = np.array([
            [x[0, 0] + v * math.cos(theta) * dt],
            [x[1, 0] + v * math.sin(theta) * dt],
            [theta    + omega * dt              ],
        ])
        # Normalize heading
        x_new[2, 0] = math.atan2(math.sin(x_new[2, 0]),
                                  math.cos(x_new[2, 0]))
        return x_new

    def _jacobian_F(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Jacobian of motion model wrt state.

        dF/dx:
          [ 1  0  -v*sin(theta)*dt ]
          [ 0  1   v*cos(theta)*dt ]
          [ 0  0        1          ]
        """
        theta = float(x[2, 0])
        v     = float(u[0, 0])
        dt    = self.dt

        return np.array([
            [1.0,  0.0, -v * math.sin(theta) * dt],
            [0.0,  1.0,  v * math.cos(theta) * dt],
            [0.0,  0.0,  1.0                     ],
        ])

    def _jacobian_H(self) -> np.ndarray:
        """
        Jacobian of observation model wrt state.
        Observes [x, y], ignores theta:
          H = [ 1 0 0 ]
              [ 0 1 0 ]
        """
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
