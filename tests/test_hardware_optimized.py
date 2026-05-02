"""
Tests hardware optimisé - Validation modules critiques
Exécutez sur Raspberry Pi: python tests/test_hardware_optimized.py
"""

import unittest
import numpy as np
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.sensors.ultrasonic_sensor import MedianFilter, UltrasonicSensor
from src.sensors.imu_sensor import IMUSensor, ComplementaryFilter
from src.sensors.sensor_fusion import OneDKalman, SensorFusion
from src.mapping import OccupancyGrid
from src.localization import EKFLocalizer
from src.planning import AStarPlanner, SimpleObstacleAvoidance, ConditionalReplanning
from src.control import PIDController, VelocityRamp
from src.pdf_map_fusion import PDFMapFusion


class TestMedianFilter(unittest.TestCase):
    def test_rejects_outliers(self):
        f = MedianFilter(size=5)
        for v in [1.0, 1.1, 1.2, 1.1, 1.0]:
            f.update(v)
        # Outlier
        result = f.update(5.0)
        self.assertLess(result, 2.0, "Doit rejeter outlier 5.0m")

    def test_median_value(self):
        f = MedianFilter(size=3)
        f.update(1.0)
        f.update(2.0)
        r = f.update(3.0)
        self.assertAlmostEqual(r, 2.0, places=2)


class TestOneDKalman(unittest.TestCase):
    def test_convergence(self):
        k = OneDKalman(q=0.01, r=0.05)
        for _ in range(50):
            k.update(1.0 + np.random.normal(0, 0.05))
        self.assertAlmostEqual(k.x, 1.0, delta=0.1)


class TestComplementaryFilter(unittest.TestCase):
    def test_drift_compensation(self):
        cf = ComplementaryFilter(alpha=0.98)
        # Simule gyro avec dérive
        for _ in range(200):
            cf.update((0, 0, 0.01), (0, 0, 9.81), is_stationary=True)
        # La dérive doit être partiellement compensée (tolérance large)
        self.assertLess(abs(cf.yaw_drift_rate - 0.01), 0.015)


class TestEKF(unittest.TestCase):
    def test_covariance_decreases_with_updates(self):
        ekf = EKFLocalizer()
        init_unc = ekf.get_uncertainty()['theta_uncertainty']
        for _ in range(5):
            ekf.predict(0.1, 0.1, 0.1)
            ekf.update_heading(0.0)
        final_unc = ekf.get_uncertainty()['theta_uncertainty']
        self.assertLess(final_unc, init_unc, "Covariance doit décroître")

    def test_straight_line_motion(self):
        ekf = EKFLocalizer()
        for _ in range(10):
            ekf.predict(0.2, 0.2, 0.1)
            ekf.update_odometry(10, 10)
        s = ekf.get_state()
        self.assertGreater(s.x, 0.1, "Doit avancer en ligne droite")


class TestAStarPlanner(unittest.TestCase):
    def test_path_found_no_obstacles(self):
        grid = OccupancyGrid(width=3.0, height=3.0, resolution=0.2)
        planner = AStarPlanner(grid)
        path = planner.plan(0, 0, 1.0, 1.0)
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)

    def test_path_blocked(self):
        grid = OccupancyGrid(width=3.0, height=3.0, resolution=0.2)
        # Bloque le centre (toute la ligne médiane)
        for i in range(grid.grid_width):
            grid.log_odds[grid.grid_height // 2, i] = 2.0
        planner = AStarPlanner(grid)
        path = planner.plan(-1, 0, 1, 0)
        # Le centre bloqué empêche le passage direct
        self.assertIsNone(path)


class TestSimpleObstacleAvoidance(unittest.TestCase):
    def test_emergency_stop(self):
        avoid = SimpleObstacleAvoidance()
        v, w, active = avoid.compute_avoidance(
            {'front': 0.10, 'left': 2.0, 'right': 2.0, 'back': 2.0},
            0.3, 0.0, 0.0
        )
        self.assertEqual(v, 0.0, "STOP d'urgence")
        self.assertTrue(active)

    def test_side_bias(self):
        avoid = SimpleObstacleAvoidance()
        v, w, active = avoid.compute_avoidance(
            {'front': 1.0, 'left': 0.15, 'right': 2.0, 'back': 2.0},
            0.3, 0.0, 0.0
        )
        self.assertLess(w, 0, "Doit tourner à droite si obstacle gauche")


class TestVelocityRamp(unittest.TestCase):
    def test_acceleration_limit(self):
        ramp = VelocityRamp(max_accel=1.0)
        v1, _ = ramp.limit(1.0, 0.0)
        time.sleep(0.1)
        v2, _ = ramp.limit(1.0, 0.0)
        self.assertLess(v2 - v1, 0.15, "Accélération limitée")


class TestPDFMapFusion(unittest.TestCase):
    def test_corridor_marked_free(self):
        grid = OccupancyGrid(width=3.0, height=3.0, resolution=0.2)
        fusion = PDFMapFusion(corridor_width=0.3)
        path = [(0, 0), (0.5, 0), (1.0, 0)]
        fusion.integrate_path(grid, path)
        occ = grid.get_occupancy_map()
        center = grid.world_to_grid(0.5, 0)
        self.assertLess(occ[center[1], center[0]], 0.3, "Corridor doit être libre")


class TestIntegration(unittest.TestCase):
    def test_full_sensor_chain(self):
        ultra = UltrasonicSensor(use_gpio=False)
        imu = IMUSensor(use_i2c=False)
        fusion = SensorFusion(ultrasonic=ultra, imu=imu)
        ultra.start()
        imu.start()
        time.sleep(0.2)
        reading = fusion.get_reading()
        ultra.stop()
        imu.stop()
        self.assertIsNotNone(reading)
        self.assertIn('front', reading.distances)


def run_tests():
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestMedianFilter))
    suite.addTests(loader.loadTestsFromTestCase(TestOneDKalman))
    suite.addTests(loader.loadTestsFromTestCase(TestComplementaryFilter))
    suite.addTests(loader.loadTestsFromTestCase(TestEKF))
    suite.addTests(loader.loadTestsFromTestCase(TestAStarPlanner))
    suite.addTests(loader.loadTestsFromTestCase(TestSimpleObstacleAvoidance))
    suite.addTests(loader.loadTestsFromTestCase(TestVelocityRamp))
    suite.addTests(loader.loadTestsFromTestCase(TestPDFMapFusion))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    print("=" * 60)
    print("  🤖 TESTS HARDWARE OPTIMISÉ")
    print("=" * 60)
    success = run_tests()
    sys.exit(0 if success else 1)

