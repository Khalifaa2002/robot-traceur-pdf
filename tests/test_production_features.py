import unittest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.trajectory_generator import apply_tool_offset

class TestProductionFeatures(unittest.TestCase):
    def test_tool_offset_x(self):
        points_with_angle = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ])
        
        offset_points = apply_tool_offset(points_with_angle, offset_x=0.1, offset_y=0.0)
        
        self.assertAlmostEqual(offset_points[0, 0], -0.1)
        self.assertAlmostEqual(offset_points[0, 1], 0.0)
        self.assertAlmostEqual(offset_points[1, 0], 0.9)
        self.assertAlmostEqual(offset_points[1, 1], 0.0)

    def test_tool_offset_y(self):
        points_with_angle = np.array([
            [0.0, 0.0, 0.0]
        ])
        
        offset_points = apply_tool_offset(points_with_angle, offset_x=0.0, offset_y=0.1)
        
        self.assertAlmostEqual(offset_points[0, 0], 0.0)
        self.assertAlmostEqual(offset_points[0, 1], -0.1)

    def test_tool_offset_angle(self):
        points_with_angle = np.array([
            [0.0, 0.0, np.pi/2]
        ])
        
        offset_points = apply_tool_offset(points_with_angle, offset_x=0.1, offset_y=0.0)
        
        self.assertAlmostEqual(offset_points[0, 0], 0.0)
        self.assertAlmostEqual(offset_points[0, 1], -0.1)

if __name__ == "__main__":
    unittest.main()
