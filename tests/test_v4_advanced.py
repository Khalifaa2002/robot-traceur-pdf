"""
tests/test_v4_advanced.py
=========================
Validation of Phase E algorithms: Cubic Spline and Pure Pursuit.
"""

import unittest
import numpy as np
from planning.trajectory_generator import smooth_trajectory, add_orientation
from hardware.simulator import RobotSimulator
from localization.odometry import Localizer
from app.mission import TrajectoryFollower

class TestAdvancedAlgorithms(unittest.TestCase):
    
    def test_spline_smoothing(self):
        """Test that Cubic Spline produces a smooth path from sparse points."""
        sparse_points = np.array([
            [0, 0],
            [1, 0],
            [1, 1],
            [0, 1]
        ])
        
        # Linear smoothing
        linear_path = smooth_trajectory(sparse_points, num_points=20, method='linear')
        self.assertEqual(len(linear_path), 20)
        
        # Spline smoothing
        spline_path = smooth_trajectory(sparse_points, num_points=20, method='spline')
        self.assertEqual(len(spline_path), 20)
        
        # Spline should have different points than linear
        self.assertFalse(np.allclose(linear_path[5], spline_path[5]))
        
    def test_spline_orientation(self):
        """Test heading calculation using spline yaw."""
        points = np.array([[0, 0], [1, 0], [2, 1]])
        path_with_angle = add_orientation(points, method='spline')
        
        self.assertEqual(path_with_angle.shape, (3, 3))
        # Initial heading should be roughly 0 (along X axis)
        # Note: CubicSpline interpolates, so it might not be exactly 0 if second point is offset
        self.assertAlmostEqual(path_with_angle[0, 2], 0.0, delta=0.2)

        
    def test_pure_pursuit_follower(self):
        """Test TrajectoryFollower with Pure Pursuit controller."""
        robot = RobotSimulator()
        localizer = Localizer()
        follower = TrajectoryFollower(robot, localizer, simulation=True, controller_type='pure_pursuit')
        
        # Create a simple square trajectory
        trajectory = np.array([
            [0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [0.2, 0.2, np.pi/2],
            [0.0, 0.2, np.pi]
        ])
        
        self.assertTrue(follower.load_trajectory(trajectory))
        
        # Execute briefly in simulation
        # Note: follow() blocks, so we just run it for a very short time or mock it
        # Here we just verify it starts and computes at least one step
        robot.connect()
        success = follower.follow(max_time=0.5) # Run for 0.5s
        robot.disconnect()
        
        self.assertIn("completion_rate", follower.metrics)

if __name__ == "__main__":
    unittest.main()
