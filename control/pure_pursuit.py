"""
control/pure_pursuit.py
========================
Pure Pursuit path tracking controller for differential drive robots.

Adapted from PythonRobotics/PathTracking/pure_pursuit
Original author: Atsushi Sakai (@Atsushi_twi)
Adaptation:
  - Rewritten for differential drive (v, omega) output instead of Ackermann steering.
  - Stripped of all matplotlib, animation, global state, and sys.path hacks.
  - Designed as a stateless, reusable controller class.

Algorithm overview:
  1. Find the nearest point on the path behind the robot.
  2. Advance a lookahead distance L_d along the path to the target point.
  3. Compute the angular rate omega needed to steer toward the target.
  4. Scale linear velocity v to avoid overshoot on tight curves.

Raspberry Pi compatible: math + numpy only, no matplotlib dependency.

Classes:
    TargetCourse      — wraps a path and efficiently finds the lookahead point
    PurePursuitController — stateful controller, call compute() each cycle
"""

import math
import numpy as np


# ── Tunable parameters (can be overridden via constructor) ───────────────────

_DEFAULT_LOOKAHEAD_GAIN = 0.1    # k: lookahead distance = k * speed + L_d_min
_DEFAULT_LOOKAHEAD_MIN  = 0.20   # L_d_min [m] — minimum lookahead distance
_DEFAULT_MAX_LINEAR_V   = 0.5    # [m/s] — max forward speed
_DEFAULT_MAX_OMEGA      = 2.0    # [rad/s] — max angular rate


class TargetCourse:
    """
    Wraps a 2D path (cx, cy) and provides efficient lookahead point search.

    The search is incremental: once the nearest index is found on the first
    call, subsequent calls only scan forward — O(1) amortized per cycle.

    Parameters
    ----------
    cx, cy : array-like — x and y coordinates of path waypoints (metres)
    """

    def __init__(self, cx, cy):
        self.cx = list(cx)
        self.cy = list(cy)
        self._nearest_idx: int | None = None

    def search_lookahead_index(self, robot_x: float, robot_y: float,
                               robot_speed: float, lookahead_gain: float,
                               lookahead_min: float):
        """
        Find the index of the lookahead target point.

        Parameters
        ----------
        robot_x, robot_y : float — current robot position
        robot_speed       : float — current forward speed [m/s]
        lookahead_gain    : float — adaptive lookahead gain k
        lookahead_min     : float — minimum lookahead distance [m]

        Returns
        -------
        ind : int   — index of the lookahead target point
        L_d : float — actual lookahead distance used [m]
        """
        # Adaptive lookahead: L_d grows with speed to prevent oscillation
        L_d = lookahead_gain * abs(robot_speed) + lookahead_min

        # First call: brute-force nearest point
        if self._nearest_idx is None:
            d_sq = [(robot_x - cx)**2 + (robot_y - cy)**2
                    for cx, cy in zip(self.cx, self.cy)]
            self._nearest_idx = int(np.argmin(d_sq))

        ind = self._nearest_idx

        # Advance nearest index forward as robot progresses
        while ind + 1 < len(self.cx):
            d_next = math.hypot(robot_x - self.cx[ind + 1],
                                robot_y - self.cy[ind + 1])
            d_curr = math.hypot(robot_x - self.cx[ind],
                                robot_y - self.cy[ind])
            if d_next < d_curr:
                ind += 1
            else:
                break
        self._nearest_idx = ind

        # Advance to lookahead point
        look_ind = ind
        while look_ind + 1 < len(self.cx):
            d = math.hypot(robot_x - self.cx[look_ind],
                           robot_y - self.cy[look_ind])
            if d >= L_d:
                break
            look_ind += 1

        return look_ind, L_d

    def reset(self):
        """Reset index search state (call when switching trajectories)."""
        self._nearest_idx = None


class PurePursuitController:
    """
    Pure Pursuit controller for differential drive robots.

    Computes (v_cmd, omega_cmd) to steer a differential drive robot
    along a given path using the pure pursuit geometric algorithm.

    Usage
    -----
    controller = PurePursuitController(max_v=0.3)
    course = TargetCourse(path_x, path_y)

    # In control loop:
    v, omega = controller.compute(robot_x, robot_y, robot_yaw,
                                  robot_speed, course)

    Parameters
    ----------
    max_v          : float — max linear velocity [m/s]
    max_omega      : float — max angular velocity [rad/s]
    lookahead_gain : float — adaptive lookahead gain (speed multiplier)
    lookahead_min  : float — minimum lookahead distance [m]
    """

    def __init__(self,
                 max_v: float = _DEFAULT_MAX_LINEAR_V,
                 max_omega: float = _DEFAULT_MAX_OMEGA,
                 lookahead_gain: float = _DEFAULT_LOOKAHEAD_GAIN,
                 lookahead_min: float = _DEFAULT_LOOKAHEAD_MIN):
        self.max_v = max_v
        self.max_omega = max_omega
        self.lookahead_gain = lookahead_gain
        self.lookahead_min = lookahead_min

    def compute(self, robot_x: float, robot_y: float, robot_yaw: float,
                robot_speed: float, course: TargetCourse,
                target_speed: float = None):
        """
        Compute velocity commands to follow the given course.

        Parameters
        ----------
        robot_x, robot_y : float — current position [m]
        robot_yaw        : float — current heading [rad]
        robot_speed      : float — current forward speed [m/s]
        course           : TargetCourse — the path to follow
        target_speed     : float | None — desired cruise speed [m/s]
                           If None, uses self.max_v

        Returns
        -------
        v_cmd    : float — linear velocity command [m/s]
        omega_cmd: float — angular velocity command [rad/s]
        look_ind : int   — current lookahead index (for telemetry / completion check)
        """
        if target_speed is None:
            target_speed = self.max_v

        look_ind, L_d = course.search_lookahead_index(
            robot_x, robot_y, robot_speed,
            self.lookahead_gain, self.lookahead_min
        )

        # Target point
        tx = course.cx[look_ind]
        ty = course.cy[look_ind]

        # Angle from robot heading to target point
        alpha = math.atan2(ty - robot_y, tx - robot_x) - robot_yaw
        # Normalize to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Linear velocity: attenuate on sharp turns (same as adaptive scaling in PID)
        v_scale = max(0.1, 1.0 - abs(alpha) / (math.pi / 2))
        v_cmd = target_speed * v_scale

        # Pure pursuit curvature: kappa = 2 * sin(alpha) / L_d
        # For differential drive: omega = v * kappa
        if L_d < 1e-6:
            omega_cmd = 0.0
        else:
            kappa = 2.0 * math.sin(alpha) / L_d
            omega_cmd = v_cmd * kappa

        # Saturate
        v_cmd = float(np.clip(v_cmd, 0.0, self.max_v))
        omega_cmd = float(np.clip(omega_cmd, -self.max_omega, self.max_omega))

        return v_cmd, omega_cmd, look_ind

    def is_goal_reached(self, robot_x: float, robot_y: float,
                        course: TargetCourse, tolerance: float = 0.05) -> bool:
        """
        True when the robot is within `tolerance` metres of the last waypoint.

        Parameters
        ----------
        tolerance : float — acceptance radius [m] (default 5 cm)
        """
        last_x = course.cx[-1]
        last_y = course.cy[-1]
        dist = math.hypot(robot_x - last_x, robot_y - last_y)
        return dist <= tolerance
