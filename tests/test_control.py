"""
tests/test_control.py
=====================
Unit tests for control/pure_pursuit.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.pure_pursuit import PurePursuitController, TargetCourse


def _make_straight_course(length=5.0, step=0.1):
    """Helper: horizontal straight path from (0,0) to (length, 0)."""
    cx = [i * step for i in range(int(length / step) + 1)]
    cy = [0.0] * len(cx)
    return TargetCourse(cx, cy)


def test_pure_pursuit_import():
    """Module imports cleanly without matplotlib or sys.path side effects."""
    from control.pure_pursuit import PurePursuitController, TargetCourse
    assert PurePursuitController is not None
    assert TargetCourse is not None


def test_target_course_straight_lookahead():
    """TargetCourse: lookahead index advances along straight path."""
    course = _make_straight_course()
    # Robot at start, facing east
    ind1, L_d1 = course.search_lookahead_index(0.0, 0.0, 0.2, 0.1, 0.2)
    # Robot further along the path
    ind2, L_d2 = course.search_lookahead_index(2.0, 0.0, 0.2, 0.1, 0.2)
    assert ind2 > ind1, "Lookahead index must advance as robot moves forward"


def test_pure_pursuit_straight_low_omega():
    """Pure pursuit on straight path: angular command should be near zero."""
    course = _make_straight_course()
    ctrl = PurePursuitController(max_v=0.3, max_omega=2.0)

    # Robot perfectly on path, facing east (yaw=0)
    v, omega, idx = ctrl.compute(0.5, 0.0, 0.0, 0.2, course)

    assert v > 0, "Linear velocity must be positive on straight path"
    assert abs(omega) < 0.5, f"Omega should be small on straight path, got {omega:.3f}"


def test_pure_pursuit_turn_omega_direction():
    """Pure pursuit on left turn: positive omega (left = CCW for diff-drive)."""
    # Path curves left: all points have positive y
    cx = [float(i) for i in range(6)]
    cy = [0.0, 0.1, 0.3, 0.6, 1.0, 1.5]
    course = TargetCourse(cx, cy)
    ctrl = PurePursuitController(max_v=0.3, max_omega=3.0)

    # Robot at origin facing east
    v, omega, idx = ctrl.compute(0.0, 0.0, 0.0, 0.2, course)

    assert omega > 0, f"Left-curving path should yield positive omega, got {omega:.3f}"


def test_pure_pursuit_velocity_saturation():
    """Pure pursuit: v_cmd must never exceed max_v."""
    course = _make_straight_course()
    ctrl = PurePursuitController(max_v=0.2, max_omega=2.0)

    v, omega, _ = ctrl.compute(0.0, 0.0, 0.0, 0.5, course, target_speed=1.0)
    assert v <= ctrl.max_v, f"v_cmd={v:.3f} exceeds max_v={ctrl.max_v}"


def test_pure_pursuit_omega_saturation():
    """Pure pursuit: omega_cmd must never exceed max_omega."""
    # Extreme lateral offset: robot is far off path
    cx = [0.0, 0.1, 0.2]
    cy = [0.0, 0.0, 0.0]
    course = TargetCourse(cx, cy)
    ctrl = PurePursuitController(max_v=0.3, max_omega=1.0)

    # Robot far to the right, facing straight — huge angular error
    v, omega, _ = ctrl.compute(0.0, -2.0, 0.0, 0.2, course)
    assert abs(omega) <= ctrl.max_omega + 1e-9, \
        f"|omega|={abs(omega):.3f} exceeds max_omega={ctrl.max_omega}"


def test_pure_pursuit_goal_reached():
    """is_goal_reached returns True when robot is within tolerance."""
    cx = [0.0, 1.0, 2.0]
    cy = [0.0, 0.0, 0.0]
    course = TargetCourse(cx, cy)
    ctrl = PurePursuitController()

    assert ctrl.is_goal_reached(2.0, 0.0, course, tolerance=0.05)
    assert not ctrl.is_goal_reached(0.5, 0.0, course, tolerance=0.05)


def test_pure_pursuit_goal_not_reached_far():
    """is_goal_reached returns False when robot is far from goal."""
    cx = [0.0, 1.0, 2.0]
    cy = [0.0, 0.0, 0.0]
    course = TargetCourse(cx, cy)
    ctrl = PurePursuitController()
    assert not ctrl.is_goal_reached(0.0, 0.0, course, tolerance=0.05)


def test_target_course_reset():
    """TargetCourse.reset() restarts the search index."""
    course = _make_straight_course()
    course.search_lookahead_index(3.0, 0.0, 0.2, 0.1, 0.2)
    course.reset()
    assert course._nearest_idx is None


if __name__ == "__main__":
    tests = [
        test_pure_pursuit_import,
        test_target_course_straight_lookahead,
        test_pure_pursuit_straight_low_omega,
        test_pure_pursuit_turn_omega_direction,
        test_pure_pursuit_velocity_saturation,
        test_pure_pursuit_omega_saturation,
        test_pure_pursuit_goal_reached,
        test_pure_pursuit_goal_not_reached_far,
        test_target_course_reset,
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
