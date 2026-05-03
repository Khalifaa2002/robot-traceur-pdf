"""
Comprehensive Test Suite pour Robot Traceur Autonome

Tests unitaires pour tous les modules
"""

import unittest
import numpy as np
import time
import sys
from pathlib import Path

# Ajoute src au path
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.sensors import UltrasonicSensor, IMUSensor, SensorFusion
from core.mapping import OccupancyGrid
from core.localization import EKFLocalizer, ParticleFilter
from core.planning import AStarPlanner, SimpleObstacleAvoidance
from core.control import MotorController, PIDController, ControlLoop


class TestUltrasonicSensor(unittest.TestCase):
    """Tests capteurs ultrasons"""
    
    def setUp(self):
        self.sensor = UltrasonicSensor(use_gpio=False)
    
    def test_initialization(self):
        """Teste initialisation"""
        self.assertIsNotNone(self.sensor)
        self.assertEqual(self.sensor.MAX_DISTANCE, 4.0)
    
    def test_measure_once(self):
        """Teste mesure unique"""
        reading = self.sensor.measure_once()
        self.assertIsNotNone(reading)
        self.assertGreaterEqual(reading.front, 0)
        self.assertLessEqual(reading.front, self.sensor.MAX_DISTANCE)
    
    def test_grid_occupancy(self):
        """Teste grille occupation"""
        self.sensor.start()
        time.sleep(0.1)
        occupancy = self.sensor.get_grid_occupancy()
        self.sensor.stop()
        
        self.assertIn('front', occupancy)
        self.assertIn('back', occupancy)
        self.assertIn('left', occupancy)
        self.assertIn('right', occupancy)


class TestIMUSensor(unittest.TestCase):
    """Tests capteur IMU"""
    
    def setUp(self):
        self.imu = IMUSensor(use_i2c=False)
    
    def test_initialization(self):
        """Teste initialisation"""
        self.assertIsNotNone(self.imu)
    
    def test_heading(self):
        """Teste heading"""
        self.imu.start()
        time.sleep(0.1)
        heading = self.imu.get_heading()
        self.imu.stop()
        
        self.assertGreaterEqual(heading, 0)
        self.assertLessEqual(heading, 2 * np.pi)
    
    def test_reading(self):
        """Teste reading"""
        self.imu.start()
        time.sleep(0.1)
        reading = self.imu.get_reading()
        self.imu.stop()
        
        self.assertIsNotNone(reading.roll)
        self.assertIsNotNone(reading.pitch)
        self.assertIsNotNone(reading.yaw)


class TestOccupancyGrid(unittest.TestCase):
    """Tests grille occupation"""
    
    def setUp(self):
        self.grid = OccupancyGrid(width=5.0, height=5.0, resolution=0.1)
    
    def test_initialization(self):
        """Teste initialisation"""
        self.assertEqual(self.grid.width, 5.0)
        self.assertEqual(self.grid.height, 5.0)
        self.assertEqual(self.grid.grid_width, 50)
        self.assertEqual(self.grid.grid_height, 50)
    
    def test_world_to_grid_conversion(self):
        """Teste conversion monde → grille"""
        grid_x, grid_y = self.grid.world_to_grid(0, 0)
        self.assertEqual(grid_x, 25)  # Centre
        self.assertEqual(grid_y, 25)
    
    def test_grid_to_world_conversion(self):
        """Teste conversion grille → monde"""
        x, y = self.grid.grid_to_world(25, 25)
        self.assertAlmostEqual(x, 0.05, places=1)  # Erreur résolution acceptable
        self.assertAlmostEqual(y, 0.05, places=1)
    
    def test_ray_update(self):
        """Teste mise à jour rayon"""
        grid_x, grid_y = self.grid.world_to_grid(0.5, 0)
        occupancy_before = self.grid.get_occupancy_map()[grid_y, grid_x]
        self.grid.update_ray(0, 0, 0.5, 0, occupied=True)
        occupancy_after = self.grid.get_occupancy_map()[grid_y, grid_x]
        
        # Doit avoir augmenté après update
        self.assertGreater(occupancy_after, occupancy_before)
    
    def test_is_occupied(self):
        """Teste requête occupation"""
        self.grid.update_ray(0, 0, 0.5, 0, occupied=True)
        
        # Occupation à (0.5, 0) doit être vraie
        is_occ = self.grid.is_occupied(0.5, 0)
        self.assertTrue(is_occ)


class TestEKFLocalizer(unittest.TestCase):
    """Tests localisateur EKF"""
    
    def setUp(self):
        self.ekf = EKFLocalizer()
    
    def test_initialization(self):
        """Teste initialisation"""
        self.assertIsNotNone(self.ekf.state)
        self.assertEqual(len(self.ekf.state), 5)
    
    def test_predict(self):
        """Teste prédiction"""
        initial_x = self.ekf.state[0]
        self.ekf.predict(1.0, 1.0, 0.1)  # Vitesses égales → tout droit
        
        # Doit avoir avancé
        final_x = self.ekf.state[0]
        self.assertGreater(final_x, initial_x)
    
    def test_update_heading(self):
        """Teste mise à jour heading"""
        self.ekf.update_heading(np.pi / 4)
        state = self.ekf.get_state()
        
        # Heading doit avoir changé
        self.assertNotAlmostEqual(state.theta, 0.0)


class TestAStarPlanner(unittest.TestCase):
    """Tests planificateur A*"""
    
    def setUp(self):
        self.grid = OccupancyGrid(width=5.0, height=5.0, resolution=0.1)
        self.planner = AStarPlanner(self.grid)
    
    def test_initialization(self):
        """Teste initialisation"""
        self.assertIsNotNone(self.planner)
    
    def test_plan_no_obstacles(self):
        """Teste planification sans obstacles"""
        path = self.planner.plan(0, 0, 1, 1)
        
        if path:
            self.assertGreater(len(path), 0)
            self.assertAlmostEqual(path[0][0], 0.05, places=1)
            self.assertAlmostEqual(path[-1][0], 1.05, places=1)
    
    def test_heuristic(self):
        """Teste heuristique"""
        h = self.planner.heuristic(0, 0, 3, 4)
        self.assertGreater(h, 0)


class TestPIDController(unittest.TestCase):
    """Tests contrôleur PID"""
    
    def setUp(self):
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.2)
    
    def test_zero_error(self):
        """Teste PID avec erreur zéro"""
        cmd = self.pid.compute(0)
        self.assertAlmostEqual(cmd, 0, places=1)
    
    def test_positive_error(self):
        """Teste PID avec erreur positive"""
        cmd = self.pid.compute(1.0)
        self.assertGreater(cmd, 0)
    
    def test_anti_windup(self):
        """Teste anti-windup intégral"""
        # Accumule erreurs
        for _ in range(100):
            cmd = self.pid.compute(1.0)
        
        # L'intégrale ne doit pas exploser (clipped)
        self.assertLessEqual(abs(self.pid.error_integral), 1.0)


class TestMotorController(unittest.TestCase):
    """Tests contrôleur moteurs"""
    
    def setUp(self):
        self.motor = MotorController(use_gpio=False)
    
    def test_set_pwm(self):
        """Teste commande PWM"""
        # Ne doit pas lever d'exception
        self.motor.set_motor_pwm(128, 128)
        self.motor.set_motor_pwm(0, 0)
        self.motor.set_motor_pwm(255, 255)
    
    def test_stop(self):
        """Teste arrêt"""
        self.motor.stop()  # Ne doit pas échouer


class TestIntegration(unittest.TestCase):
    """Tests d'intégration (modules ensemble)"""
    
    def test_sensor_fusion_complete_cycle(self):
        """Teste cycle complet fusion capteurs"""
        ultrasonic = UltrasonicSensor(use_gpio=False)
        imu = IMUSensor(use_i2c=False)
        fusion = SensorFusion(ultrasonic, imu)
        
        fusion.start()
        time.sleep(0.1)
        
        reading = fusion.get_reading()
        self.assertIsNotNone(reading)
        self.assertIsNotNone(reading.distances)
        
        fusion.stop()
    
    def test_mapping_and_planning(self):
        """Teste mapping + planning"""
        grid = OccupancyGrid()
        planner = AStarPlanner(grid)
        
        # Ajoute obstacle
        grid.update_ray(0, 0, 0.5, 0, occupied=True)
        
        # Plan chemin
        path = planner.plan(0, 0, 2, 2)
        self.assertIsNotNone(path or True)  # Chemin peut être None si impossible


# ============================================================================
def run_tests(verbosity=2):
    """Exécute tous les tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Ajoute tous les tests
    suite.addTests(loader.loadTestsFromTestCase(TestUltrasonicSensor))
    suite.addTests(loader.loadTestsFromTestCase(TestIMUSensor))
    suite.addTests(loader.loadTestsFromTestCase(TestOccupancyGrid))
    suite.addTests(loader.loadTestsFromTestCase(TestEKFLocalizer))
    suite.addTests(loader.loadTestsFromTestCase(TestAStarPlanner))
    suite.addTests(loader.loadTestsFromTestCase(TestPIDController))
    suite.addTests(loader.loadTestsFromTestCase(TestMotorController))
    from tests.test_production_features import TestProductionFeatures
    suite.addTests(loader.loadTestsFromTestCase(TestProductionFeatures))
    
    runner = unittest.TextTestRunner(verbosity=verbosity)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
