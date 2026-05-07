"""
planning/cubic_spline.py
========================
Cubic Spline path planning for smooth 2D trajectory generation.

Adapted from PythonRobotics/PathPlanning/CubicSpline
Original author: Atsushi Sakai (@Atsushi_twi)
Adaptation: stripped of matplotlib, demo code, and sys.path hacks.
Designed for use as a production library module.

Classes:
    CubicSpline1D  — 1D cubic spline interpolation
    CubicSpline2D  — 2D arc-length parameterized cubic spline

Functions:
    calc_spline_course(x, y, ds) → (rx, ry, ryaw, rk, s)
        High-level helper: waypoints → dense smooth path with yaw/curvature.

Raspberry Pi compatible: numpy + math only, no matplotlib dependency.
"""

import math
import numpy as np
import bisect


class CubicSpline1D:
    """
    1D Cubic Spline interpolation.

    Computes natural cubic spline coefficients from (x, y) data points.
    x must be sorted in ascending order.

    Parameters
    ----------
    x : array-like  — x coordinates (must be strictly ascending)
    y : array-like  — y coordinates

    Methods
    -------
    calc_position(x)        → float | None
    calc_first_derivative(x) → float | None
    calc_second_derivative(x)→ float | None
    """

    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h <= 0):
            raise ValueError(
                "CubicSpline1D: x must be strictly ascending. "
                f"Got diff min={h.min():.6f}"
            )

        self.x = list(x)
        self.y = list(y)
        self.nx = len(x)

        self.a = list(y)
        A = self._calc_A(h)
        B = self._calc_B(h, self.a)
        self.c = list(np.linalg.solve(A, B))

        self.b, self.d = [], []
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = (self.a[i + 1] - self.a[i]) / h[i] \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.b.append(b)
            self.d.append(d)

    def calc_position(self, x: float):
        """Interpolated y at given x. Returns None if x is out of range."""
        if x < self.x[0] or x > self.x[-1]:
            return None
        i = self._search_index(x)
        # Clamp: bisect on x[-1] returns nx, but last segment index is nx-2
        i = min(i, self.nx - 2)
        dx = x - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3

    def calc_first_derivative(self, x: float):
        """First derivative (slope) at given x. Returns None if out of range."""
        if x < self.x[0] or x > self.x[-1]:
            return None
        i = self._search_index(x)
        i = min(i, self.nx - 2)
        dx = x - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2

    def calc_second_derivative(self, x: float):
        """Second derivative at given x. Returns None if out of range."""
        if x < self.x[0] or x > self.x[-1]:
            return None
        i = self._search_index(x)
        i = min(i, self.nx - 2)
        dx = x - self.x[i]
        return 2.0 * self.c[i] + 6.0 * self.d[i] * dx

    def _search_index(self, x: float) -> int:
        return bisect.bisect(self.x, x) - 1

    def _calc_A(self, h):
        n = self.nx
        A = np.zeros((n, n))
        A[0, 0] = 1.0
        A[n - 1, n - 1] = 1.0
        for i in range(n - 1):
            if i != n - 2:
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]
        A[0, 1] = 0.0
        A[n - 1, n - 2] = 0.0
        return A

    def _calc_B(self, h, a):
        n = self.nx
        B = np.zeros(n)
        for i in range(n - 2):
            B[i + 1] = (
                3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]
                - 3.0 * (a[i + 1] - a[i]) / h[i]
            )
        return B


class CubicSpline2D:
    """
    2D Cubic Spline parameterized by arc length.

    Takes a list of (x, y) waypoints and fits smooth cubic splines to both
    x(s) and y(s), where s is the cumulative arc-length parameter.

    This enables computing smooth positions, yaw angles, and curvatures at
    any arc-length position — essential for trajectory smoothing.

    Parameters
    ----------
    x, y : array-like — waypoint coordinates

    Methods
    -------
    calc_position(s)  → (x, y)   — position at arc-length s
    calc_yaw(s)       → float    — tangent angle (radians) at s
    calc_curvature(s) → float    — signed curvature at s

    Attributes
    ----------
    s : list[float]  — arc-length values at each waypoint
    """

    def __init__(self, x, y):
        self.s = self._calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def _calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        s = [0.0]
        s.extend(np.cumsum(ds))
        return s

    def calc_position(self, s: float):
        """
        Position (x, y) at arc-length s.
        Returns (None, None) if s is out of range.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)
        return x, y

    def calc_yaw(self, s: float):
        """Tangent heading angle (rad) at arc-length s."""
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        if dx is None or dy is None:
            return None
        return math.atan2(dy, dx)

    def calc_curvature(self, s: float):
        """Signed curvature (1/m) at arc-length s."""
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        if None in (dx, ddx, dy, ddy):
            return None
        denom = (dx**2 + dy**2) ** 1.5
        if abs(denom) < 1e-9:
            return 0.0
        return (ddy * dx - ddx * dy) / denom


def calc_spline_course(x, y, ds: float = 0.05):
    """
    High-level helper: convert sparse waypoints into a dense smooth path.

    Parameters
    ----------
    x, y : array-like — input waypoints (metres)
    ds   : float      — arc-length step between output points (metres)
                        Default 0.05 m is suitable for most robot speeds.

    Returns
    -------
    rx   : list[float] — x positions
    ry   : list[float] — y positions
    ryaw : list[float] — yaw angles (rad)
    rk   : list[float] — curvatures (1/m)
    s    : list[float] — arc-length values
    """
    sp = CubicSpline2D(x, y)
    s_vals = list(np.arange(0.0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for s_i in s_vals:
        ix, iy = sp.calc_position(s_i)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(s_i))
        rk.append(sp.calc_curvature(s_i))

    return rx, ry, ryaw, rk, s_vals
