"""
tests/test_planning.py
======================
Unit tests for planning/cubic_spline.py
"""

import sys
import os
import math
import numpy as np

# Ensure project root is in path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planning.cubic_spline import CubicSpline1D, CubicSpline2D, calc_spline_course


def test_cubic_spline_1d_basic():
    """CubicSpline1D: interpolates through given data points."""
    x = [0.0, 1.0, 2.0, 3.0, 4.0]
    y = [0.0, 1.0, 0.0, 1.0, 0.0]
    sp = CubicSpline1D(x, y)

    # Must pass through all control points
    for xi, yi in zip(x, y):
        val = sp.calc_position(xi)
        assert val is not None, f"calc_position({xi}) returned None"
        assert abs(val - yi) < 1e-6, f"Interpolation error at x={xi}: got {val}, expected {yi}"


def test_cubic_spline_1d_out_of_range():
    """CubicSpline1D: returns None outside data range."""
    sp = CubicSpline1D([0.0, 1.0, 2.0], [0.0, 1.0, 0.0])
    assert sp.calc_position(-0.1) is None
    assert sp.calc_position(2.1) is None


def test_cubic_spline_1d_derivative():
    """CubicSpline1D: first derivative is consistent with finite difference."""
    x = [0.0, 1.0, 2.0, 3.0]
    y = [0.0, 1.0, 4.0, 9.0]  # approximates y ~ x^2 locally
    sp = CubicSpline1D(x, y)

    # Derivative at x=1.5 via finite difference
    dx = 1e-5
    fd = (sp.calc_position(1.5 + dx) - sp.calc_position(1.5 - dx)) / (2 * dx)
    d = sp.calc_first_derivative(1.5)
    assert d is not None
    assert abs(d - fd) < 1e-4, f"Derivative mismatch: analytical={d:.6f}, FD={fd:.6f}"


def test_cubic_spline_2d_straight_line():
    """CubicSpline2D: straight line gives constant yaw and near-zero curvature."""
    x = [0.0, 1.0, 2.0, 3.0, 4.0]
    y = [0.0, 0.0, 0.0, 0.0, 0.0]  # horizontal line
    sp = CubicSpline2D(x, y)

    mid_s = sp.s[-1] / 2.0
    px, py = sp.calc_position(mid_s)
    yaw = sp.calc_yaw(mid_s)
    k = sp.calc_curvature(mid_s)

    assert px is not None and py is not None
    assert abs(py) < 1e-6, f"Expected py≈0, got {py}"
    assert abs(yaw) < 1e-6, f"Expected yaw≈0 on straight line, got {yaw}"
    assert abs(k) < 1e-4, f"Expected zero curvature on straight line, got {k}"


def test_cubic_spline_2d_arc_length():
    """CubicSpline2D: s values are monotonically increasing."""
    x = [0.0, 1.0, 0.0, -1.0]
    y = [0.0, 1.0, 2.0,  1.0]
    sp = CubicSpline2D(x, y)

    diffs = np.diff(sp.s)
    assert np.all(diffs > 0), "Arc-length values must be strictly increasing"


def test_calc_spline_course_shape():
    """calc_spline_course: returns 5 consistent lists of the same length."""
    x = [0.0, 1.0, 2.0, 1.0, 0.0]
    y = [0.0, 0.5, 1.5, 2.0, 2.0]
    rx, ry, ryaw, rk, s = calc_spline_course(x, y, ds=0.1)

    assert len(rx) == len(ry) == len(ryaw) == len(rk) == len(s)
    assert len(rx) > 5, "Should produce many interpolated points"


def test_calc_spline_course_yaw_direction():
    """calc_spline_course: first point yaw should point roughly toward second waypoint."""
    x = [0.0, 3.0, 6.0]
    y = [0.0, 0.0, 0.0]  # pointing east
    rx, ry, ryaw, rk, s = calc_spline_course(x, y, ds=0.1)

    assert abs(ryaw[0]) < 0.1, f"Expected yaw≈0 (east), got {math.degrees(ryaw[0]):.1f}°"


def test_cubic_spline_1d_non_ascending_raises():
    """CubicSpline1D: raises ValueError if x is not strictly ascending."""
    try:
        sp = CubicSpline1D([0.0, 1.0, 1.0, 2.0], [0.0, 1.0, 2.0, 3.0])
        assert False, "Should have raised ValueError"
    except ValueError:
        pass  # Expected


if __name__ == "__main__":
    tests = [
        test_cubic_spline_1d_basic,
        test_cubic_spline_1d_out_of_range,
        test_cubic_spline_1d_derivative,
        test_cubic_spline_2d_straight_line,
        test_cubic_spline_2d_arc_length,
        test_calc_spline_course_shape,
        test_calc_spline_course_yaw_direction,
        test_cubic_spline_1d_non_ascending_raises,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {t.__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passed")
