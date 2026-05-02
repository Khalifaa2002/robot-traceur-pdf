"""
main_hardware_optimized.py - Point d'entrée Raspberry Pi OPTIMISÉ

Pas de matplotlib, pas de DWA, pas de ParticleFilter.
Boucle temps réel 10 Hz, monitoring, emergency stop.

Usage:
    python main_hardware_optimized.py --mode gpio --pdf data/plans/plan.pdf
    python main_hardware_optimized.py --mode simulation --pdf data/plans/plan.pdf --duration 60
"""

import argparse
import sys
import signal
import time
import numpy as np
from pathlib import Path

# Configuration logging minimal (pas de matplotlib)
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

from src.sensors import UltrasonicSensor, IMUSensor, SensorFusion
from src.mapping import OccupancyGrid
from src.localization import EKFLocalizer
from src.planning import AStarPlanner, SimpleObstacleAvoidance, ConditionalReplanning
from src.control import MotorController, ControlLoop
from src.pdf_extractor import extract_path_from_pdf
from src.trajectory_generator import smooth_trajectory, pixel_to_world, add_orientation
from src.realtime_monitor import RealTimeMonitor
from src.pdf_map_fusion import PDFMapFusion


class HardwareRobotSystem:
    """Système robot optimisé pour Raspberry Pi"""

    def __init__(self, mode: str = 'simulation', pdf_path: str = None,
                 use_gpio: bool = False, duration: int = 60):
        self.mode = mode
        self.pdf_path = pdf_path
        self.use_gpio = use_gpio and mode in ['gpio', 'serial']
        self.duration = duration
        self.running = False

        logger.info(f"🤖 HardwareRobotSystem init (mode={mode})")

        # Capteurs
        self.ultrasonic = UltrasonicSensor(use_gpio=self.use_gpio, filter_size=5)
        self.imu = IMUSensor(use_i2c=self.use_gpio)
        self.sensor_fusion = SensorFusion(self.ultrasonic, self.imu)

        # Mapping & Localisation
        self.grid = OccupancyGrid(width=4.0, height=4.0, resolution=0.2)
        self.localizer = EKFLocalizer()

        # Planning
        self.planner = AStarPlanner(self.grid)
        self.obstacle_avoidance = SimpleObstacleAvoidance()
        self.conditional_replanning = ConditionalReplanning(self.planner)

        # PDF fusion
        self.pdf_fusion = PDFMapFusion(corridor_width=0.25)
        self.global_path = None

        # Contrôle
        self.motor = MotorController(use_gpio=self.use_gpio)
        self.control_loop = ControlLoop(
            self.motor,
            self.sensor_fusion,
            self.localizer,
            self.planner,
            self.grid,
            self.obstacle_avoidance,
            max_linear_v=0.3,
            max_angular_w=1.0
        )

        # Monitoring
        self.monitor = RealTimeMonitor(log_interval=5.0)

        # Signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        logger.info("\n⚠️ Signal d'arrêt reçu")
        self.stop()
        sys.exit(0)

    def load_pdf(self) -> bool:
        if not self.pdf_path:
            logger.warning("Pas de PDF spécifié")
            return False
        if not Path(self.pdf_path).exists():
            logger.error(f"PDF non trouvé: {self.pdf_path}")
            return False

        try:
            logger.info(f"📄 Extraction {self.pdf_path}...")
            points = extract_path_from_pdf(self.pdf_path)
            if points is None:
                return False

            smooth = smooth_trajectory(points, num_points=50)  # Réduit pour RPi
            world = pixel_to_world(smooth, pixel_to_meter=0.001)
            trajectory = add_orientation(world)

            self.global_path = trajectory
            self.control_loop.set_global_path(trajectory)

            # Intègre PDF dans la carte
            self.pdf_fusion.integrate_path(self.grid, [(p[0], p[1]) for p in trajectory])

            logger.info(f"✅ Trajectoire: {len(trajectory)} points")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur PDF: {e}")
            return False

    def start(self):
        logger.info("🚀 Démarrage système...")
        self.running = True
        self.sensor_fusion.start()
        self.control_loop.start()
        self.monitor.start()
        logger.info("✅ Système en exécution")

    def stop(self):
        logger.info("⛔ Arrêt système...")
        self.running = False
        self.control_loop.stop()
        self.sensor_fusion.stop()
        self.monitor.stop()
        logger.info("✅ Système arrêté")

    def run(self):
        if self.pdf_path and not self.load_pdf():
            logger.warning("Mode exploration sans PDF")

        self.start()

        end_time = time.monotonic() + self.duration
        try:
            while self.running and time.monotonic() < end_time:
                stats = self.control_loop.get_stats()
                self.monitor.update(stats, sensor_health=True)

                # Log télémétrie toutes les 5s
                if int(time.monotonic()) % 5 == 0:
                    state = self.localizer.get_state()
                    logger.info(f"📍 x={state.x:.2f} y={state.y:.2f} "
                               f"θ={np.degrees(state.theta):5.1f}° "
                               f"iter={stats['iterations']} "
                               f"misses={stats['deadline_misses']}")

                time.sleep(1.0)

        except KeyboardInterrupt:
            logger.info("\n⚠️ Interruption utilisateur")
        finally:
            self.stop()
            stats = self.control_loop.get_stats()
            logger.info(f"\n📊 Statistiques finales:")
            logger.info(f"   Itérations: {stats['iterations']}")
            logger.info(f"   Deadline misses: {stats['deadline_misses']}")
            logger.info(f"   Emergency stops: {stats['emergency_stops']}")
            logger.info(f"   Loop min/avg/max: {stats['min_loop_time']*1000:.1f}/"
                       f"{stats['avg_loop_time']*1000:.1f}/"
                       f"{stats['max_loop_time']*1000:.1f} ms")
            logger.info(f"   Log CSV: {self.monitor.csv_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="🤖 Robot Traceur - Hardware Optimisé")
    parser.add_argument('--mode', choices=['simulation', 'gpio', 'serial'],
                        default='simulation', help='Mode exécution')
    parser.add_argument('--pdf', type=str, help='Chemin fichier PDF')
    parser.add_argument('--gpio', action='store_true', help='Utiliser GPIO réel')
    parser.add_argument('--duration', type=int, default=60, help='Durée (s)')
    return parser.parse_args()


def main():
    args = parse_args()
    logger.info("=" * 60)
    logger.info("  🤖 ROBOT TRACEUR - Hardware Optimisé v2.0")
    logger.info("=" * 60)

    system = HardwareRobotSystem(
        mode=args.mode,
        pdf_path=args.pdf,
        use_gpio=args.gpio,
        duration=args.duration
    )
    system.run()
    logger.info("✅ Exécution terminée")


if __name__ == "__main__":
    main()

