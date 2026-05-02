"""
Script de démonstration - Robot Traceur Autonome
Démontre les capacités du système
"""

import numpy as np
import time
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def demo_sensor_reading():
    """Démo: Lecture des capteurs"""
    logger.info("=" * 60)
    logger.info("DÉMO 1: Lecture Capteurs")
    logger.info("=" * 60)
    
    from src.sensors import UltrasonicSensor, IMUSensor, SensorFusion
    
    logger.info("\n📡 Initialisation capteurs...")
    ultrasonic = UltrasonicSensor(use_gpio=False)
    imu = IMUSensor(use_i2c=False)
    fusion = SensorFusion(ultrasonic, imu)
    
    logger.info("▶️  Démarrage lecture...")
    fusion.start()
    
    for i in range(5):
        reading = fusion.get_reading()
        state = fusion.get_estimated_state()
        
        logger.info(f"\n  Mesure {i+1}:")
        logger.info(f"    Position: ({state['x']:.3f}, {state['y']:.3f}) m")
        logger.info(f"    Cap: {state['heading_degrees']:.1f}°")
        logger.info(f"    Distances: F={reading.distances['front']:.2f}m, "
                   f"B={reading.distances['back']:.2f}m, "
                   f"L={reading.distances['left']:.2f}m, "
                   f"R={reading.distances['right']:.2f}m")
        time.sleep(0.2)
    
    fusion.stop()
    logger.info("\n✅ Démo 1 terminée")


def demo_occupancy_grid():
    """Démo: Mapping avec grille d'occupation"""
    logger.info("\n" + "=" * 60)
    logger.info("DÉMO 2: Occupancy Grid Mapping")
    logger.info("=" * 60)
    
    from src.mapping import OccupancyGrid
    
    logger.info("\n🗺️  Création grille (5m x 5m, res=0.1m)...")
    grid = OccupancyGrid(width=5.0, height=5.0, resolution=0.1)
    
    logger.info("▶️  Simulation rayons obstacles...")
    
    # Simule des obstacles
    obstacles = [
        (0, 0, 1.0, 0.5, "Obstacle avant"),
        (0, 0, -0.8, -0.3, "Obstacle arrière"),
        (0, 0, 0.3, -0.8, "Obstacle gauche"),
    ]
    
    for ox, oy, ex, ey, desc in obstacles:
        logger.info(f"  {desc}: ({ex:.2f}, {ey:.2f})")
        grid.update_ray(ox, oy, ex, ey, occupied=True)
    
    occupancy = grid.get_occupancy_map()
    logger.info(f"\n  Statistiques grille:")
    logger.info(f"    Occupation moyenne: {occupancy.mean():.2%}")
    logger.info(f"    Max occupancy: {occupancy.max():.2%}")
    logger.info(f"    Min occupancy: {occupancy.min():.2%}")
    
    logger.info("\n✅ Démo 2 terminée")


def demo_ekf_localization():
    """Démo: Localisation EKF"""
    logger.info("\n" + "=" * 60)
    logger.info("DÉMO 3: Extended Kalman Filter Localization")
    logger.info("=" * 60)
    
    from src.localization import EKFLocalizer
    
    logger.info("\n🎯 Initialisation EKF...")
    ekf = EKFLocalizer()
    
    logger.info("▶️  Simulation trajectoire: tout droit puis virage")
    
    for step in range(15):
        if step < 10:
            # Tout droit
            left_vel = 0.5
            right_vel = 0.5
        else:
            # Virage
            left_vel = 0.3
            right_vel = 0.5
        
        ekf.predict(left_vel, right_vel, 0.1)
        ekf.update_odometry(100, 100)
        ekf.update_heading(np.pi / 6 if step < 10 else np.pi / 3)
        
        state = ekf.get_state()
        uncertainty = ekf.get_uncertainty()
        
        logger.info(f"\n  Step {step}:")
        logger.info(f"    Position: ({state.x:.3f}, {state.y:.3f}) m")
        logger.info(f"    Heading: {np.degrees(state.theta):.1f}°")
        logger.info(f"    Uncertainty X: {uncertainty['x_uncertainty']:.4f} m")
    
    logger.info("\n✅ Démo 3 terminée")


def demo_astar_planning():
    """Démo: Planification A*"""
    logger.info("\n" + "=" * 60)
    logger.info("DÉMO 4: A* Path Planning")
    logger.info("=" * 60)
    
    from src.mapping import OccupancyGrid
    from src.planning import AStarPlanner
    
    logger.info("\n🗺️  Création grille avec obstacles...")
    grid = OccupancyGrid(width=10.0, height=10.0, resolution=0.2)
    
    # Ajoute obstacles
    for i in range(5):
        grid.update_ray(0, 0, 2.0 + i*0.3, 0, occupied=True)
    
    logger.info("▶️  Planification chemin...")
    planner = AStarPlanner(grid)
    
    path = planner.plan(-3, -3, 3, 3)
    
    if path:
        logger.info(f"\n  ✅ Chemin trouvé: {len(path)} points")
        logger.info(f"     Départ: ({path[0][0]:.2f}, {path[0][1]:.2f})")
        logger.info(f"     But: ({path[-1][0]:.2f}, {path[-1][1]:.2f})")
        
        # Calcule longueur
        path_length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            path_length += np.sqrt(dx**2 + dy**2)
        logger.info(f"     Longueur: {path_length:.2f} m")
    else:
        logger.info("  ❌ Pas de chemin trouvé")
    
    logger.info("\n✅ Démo 4 terminée")


def demo_dwa_collision_avoidance():
    """Démo: Evitement obstacles temps réel"""
    logger.info("\n" + "=" * 60)
    logger.info("DÉMO 5: Dynamic Window Approach")
    logger.info("=" * 60)
    
    from src.planning import DynamicWindowApproach
    from src.mapping import OccupancyGrid
    
    logger.info("\n⚠️  Initialisation DWA...")
    dwa = DynamicWindowApproach(max_v=0.5, max_omega=1.0)
    grid = OccupancyGrid()
    
    logger.info("▶️  Calcul commande évitement (10 cycles)...")
    
    for i in range(10):
        # Position, orientation
        x, y, theta = 0, 0, 0
        goal_x, goal_y = 2, 0
        
        v_cmd, w_cmd = dwa.plan_command(x, y, theta, goal_x, goal_y, grid)
        
        logger.info(f"\n  Cycle {i}:")
        logger.info(f"    Vitesse linéaire: {v_cmd:.3f} m/s")
        logger.info(f"    Vitesse angulaire: {w_cmd:.3f} rad/s")
    
    logger.info("\n✅ Démo 5 terminée")


def demo_motor_control():
    """Démo: Contrôle moteurs"""
    logger.info("\n" + "=" * 60)
    logger.info("DÉMO 6: Motor Control")
    logger.info("=" * 60)
    
    from src.control import MotorController, PIDController
    
    logger.info("\n⚙️  Initialisation contrôleur moteurs...")
    motor = MotorController(use_gpio=False)
    
    logger.info("▶️  Test rampe PWM (0 → 255 → 0)...")
    
    for pwm in list(range(0, 256, 50)) + list(range(255, -1, -50)):
        motor.set_motor_pwm(pwm, pwm)
        logger.info(f"  PWM: {pwm:3d}/255")
        time.sleep(0.1)
    
    motor.stop()
    
    logger.info("\n  Test PID...")
    pid = PIDController(kp=1.0, ki=0.1, kd=0.2)
    
    for error in [1.0, 0.8, 0.5, 0.2, 0.0]:
        cmd = pid.compute(error)
        logger.info(f"    Error: {error:.1f} → Command: {cmd:.3f}")
    
    logger.info("\n✅ Démo 6 terminée")

def main():
    """Exécute toutes les démos"""
    logger.info("\n" + "🚀 " * 20)
    logger.info("DÉMONSTRATION COMPLÈTE - ROBOT TRACEUR AUTONOME")
    logger.info("🚀 " * 20)
    
    try:
        demo_sensor_reading()
        demo_occupancy_grid()
        demo_ekf_localization()
        demo_astar_planning()
        demo_dwa_collision_avoidance()
        demo_motor_control()
        
        logger.info("\n" + "=" * 60)
        logger.info("✅ TOUTES LES DÉMOS COMPLÉTÉES AVEC SUCCÈS")
        logger.info("=" * 60)
    
    except Exception as e:
        logger.error(f"\n❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
