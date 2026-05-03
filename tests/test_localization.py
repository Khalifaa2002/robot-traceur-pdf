"""
tests/test_localization.py
==========================
Unit tests for localization/ekf.py
"""

import sys
import os
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from localization.ekf import EKFLocalizer


def test_ekf_import():
    """EKFLocalizer imports cleanly."""
    from localization.ekf import EKFLocalizer
    assert EKFLocalizer is not None


def test_ekf_initial_pose():
    """EKF initializes to given pose."""
    ekf = EKFLocalizer(x0=1.0, y0=2.0, theta0=0.5)
    x, y, theta = ekf.get_pose()
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 2.0) < 1e-9
    assert abs(theta - 0.5) < 1e-9


def test_ekf_predict_straight():
    """EKF predict: straight forward motion accumulates x correctly."""
    ekf = EKFLocalizer(x0=0.0, y0=0.0, theta0=0.0, dt=0.1)
    # Move forward at 0.5 m/s for 10 steps = 0.5 m
    for _ in range(10):
        ekf.predict(v=0.5, omega=0.0)

    x, y, theta = ekf.get_pose()
    assert abs(x - 0.5) < 0.01, f"Expected x≈0.5, got {x:.4f}"
    assert abs(y) < 1e-6, f"Expected y≈0, got {y:.6f}"
    assert abs(theta) < 1e-6, f"Expected theta≈0, got {theta:.6f}"


def test_ekf_predict_rotation():
    """EKF predict: pure rotation (v=0) accumulates theta correctly."""
    ekf = EKFLocalizer(x0=0.0, y0=0.0, theta0=0.0, dt=0.1)
    # Rotate at pi/2 rad/s for 10 steps = pi/2 rad total
    for _ in range(10):
        ekf.predict(v=0.0, omega=math.pi / 2)

    x, y, theta = ekf.get_pose()
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    assert abs(theta - math.pi / 2) < 0.01, f"Expected theta≈π/2, got {theta:.4f}"


def test_ekf_predict_circular_arc():
    """EKF predict: circular arc — heading changes, position moves."""
    ekf = EKFLocalizer(x0=0.0, y0=0.0, theta0=0.0, dt=0.1)
    # Half circle: v=1 m/s, omega=pi rad/s → semicircle, radius=1/pi
    steps = 10  # pi seconds / dt
    for _ in range(steps):
        ekf.predict(v=0.5, omega=math.pi / 2)

    x, y, theta = ekf.get_pose()
    # After pi/2 rad turn, robot should have moved non-trivially
    total_dist = math.hypot(x, y)
    assert total_dist > 0.1, f"Position should have changed, got ({x:.3f}, {y:.3f})"


def test_ekf_covariance_grows_without_update():
    """EKF covariance grows during prediction (no update = uncertainty grows)."""
    ekf = EKFLocalizer(dt=0.1)
    P_initial = ekf.get_covariance().copy()

    for _ in range(20):
        ekf.predict(v=0.3, omega=0.1)

    P_final = ekf.get_covariance()
    trace_initial = np.trace(P_initial)
    trace_final = np.trace(P_final)
    assert trace_final > trace_initial, \
        f"Covariance should grow: initial trace={trace_initial:.6f}, final={trace_final:.6f}"


def test_ekf_update_reduces_uncertainty():
    """EKF update: fusing a GPS observation reduces position uncertainty."""
    ekf = EKFLocalizer(dt=0.1)
    # Accumulate uncertainty
    for _ in range(20):
        ekf.predict(v=0.3, omega=0.0)

    uncertainty_before = ekf.get_position_uncertainty()

    # Fuse a perfect GPS observation at true position
    x, y, _ = ekf.get_pose()
    ekf.update(np.array([[x], [y]]))

    uncertainty_after = ekf.get_position_uncertainty()
    assert uncertainty_after < uncertainty_before, \
        f"Update should reduce uncertainty: {uncertainty_before:.4f} → {uncertainty_after:.4f}"


def test_ekf_theta_normalization():
    """EKF predict: theta stays in [-pi, pi] after accumulation."""
    ekf = EKFLocalizer(dt=0.1)
    # Spin many full rotations
    for _ in range(100):
        ekf.predict(v=0.0, omega=math.pi)

    _, _, theta = ekf.get_pose()
    assert -math.pi <= theta <= math.pi, f"Theta out of range: {theta:.4f}"


def test_ekf_reset():
    """EKF reset() correctly restores pose and covariance."""
    ekf = EKFLocalizer(dt=0.1)
    for _ in range(20):
        ekf.predict(v=0.5, omega=0.2)

    ekf.reset(x0=1.0, y0=2.0, theta0=0.3)
    x, y, theta = ekf.get_pose()
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 2.0) < 1e-9
    assert abs(theta - 0.3) < 1e-9
    # Covariance should be small again
    assert np.trace(ekf.get_covariance()) < 1.0


def test_ekf_get_position_uncertainty_nonneg():
    """get_position_uncertainty returns a non-negative float."""
    ekf = EKFLocalizer(dt=0.1)
    unc = ekf.get_position_uncertainty()
    assert isinstance(unc, float)
    assert unc >= 0.0


if __name__ == "__main__":
    tests = [
        test_ekf_import,
        test_ekf_initial_pose,
        test_ekf_predict_straight,
        test_ekf_predict_rotation,
        test_ekf_predict_circular_arc,
        test_ekf_covariance_grows_without_update,
        test_ekf_update_reduces_uncertainty,
        test_ekf_theta_normalization,
        test_ekf_reset,
        test_ekf_get_position_uncertainty_nonneg,
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
