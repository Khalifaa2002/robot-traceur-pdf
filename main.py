"""
Point d'entrée principal pour le robot traceur de plan PDF.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

from src.config import RobotConfig, logger
from src.pdf_extractor import extract_path_from_pdf, visualize_path
from src.trajectory_generator import smooth_trajectory, pixel_to_world, add_orientation, apply_tool_offset, save_trajectory
from src.robot_interface import RobotFactory
from src.localizer import Localizer
from src.trajectory_follower import TrajectoryFollower
import json
from datetime import datetime

def parse_args():
    parser = argparse.ArgumentParser(description="Robot Traceur de Plan PDF")
    
    parser.add_argument('--mode', type=str, choices=['simulation', 'serial', 'gpio'], 
                        default='simulation', help='Mode de fonctionnement du robot')
    
    parser.add_argument('--pdf', type=str, 
                        help='Chemin vers le fichier PDF à tracer')
    
    parser.add_argument('--port', type=str, default=RobotConfig.SERIAL_PORT,
                        help='Port série (ex: COM3 ou /dev/ttyACM0)')
    
    parser.add_argument('--no-plot', action='store_true',
                        help='Désactive la visualisation (utile sur Raspberry Pi headless)')
    
    parser.add_argument('--controller', type=str, default='pid',
                        help='Contrôleur à utiliser (ex: pid, pure_pursuit)')
    parser.add_argument('--validate', action='store_true',
                        help='Générer un rapport de validation JSON à la fin')
    parser.add_argument('--scanned', action='store_true',
                        help='Forcer le mode PDF scanné (fallback Hough lines)')
    parser.add_argument('--dpi', type=int, default=300,
                        help='DPI pour la rastérisation du PDF')
    parser.add_argument('--tool-offset-x', type=float, default=0.0,
                        help='Décalage de l\'outil en X (mètres)')
    parser.add_argument('--tool-offset-y', type=float, default=0.0,
                        help='Décalage de l\'outil en Y (mètres)')
    parser.add_argument('--dashboard', action='store_true',
                        help='Activer le tableau de bord')
    
    return parser.parse_args()

def main():
    args = parse_args()
    logger.info("🤖 Démarrage du Robot Traceur PDF")
    logger.info(f"   Mode: {args.mode}")
    
    # 1. Traitement du PDF
    if not args.pdf:
        logger.error("❌ Aucun fichier PDF spécifié. Utilisez --pdf <chemin>")
        sys.exit(1)
        
    pdf_path = Path(args.pdf)
    if not pdf_path.exists():
        logger.error(f"❌ Le fichier PDF {pdf_path.resolve()} n'existe pas.")
        sys.exit(1)
        
    logger.info(f"📄 Extraction depuis {pdf_path.name}...")
    original_points = extract_path_from_pdf(str(pdf_path), scanned=args.scanned, dpi=args.dpi)
    
    if original_points is None:
        logger.error("❌ Échec de l'extraction des points du PDF.")
        sys.exit(1)
        
    smooth_points = smooth_trajectory(original_points, num_points=100)
    world_points = pixel_to_world(smooth_points, pixel_to_meter=0.001)
    trajectory_base = add_orientation(world_points)
    
    # Application du tool offset
    trajectory = apply_tool_offset(trajectory_base, args.tool_offset_x, args.tool_offset_y)
    
    # Sauvegarde
    save_trajectory(trajectory, str(Path(args.pdf).parent / "trajectory.npy"))
    
    # 2. Initialisation du Robot
    robot = RobotFactory.create_robot(args.mode)
    if args.mode == "serial" and args.port:
        robot.port = args.port
        logger.info(f"🔌 Port série forcé à {args.port}")
        
    if not robot.connect():
        logger.error("❌ Impossible de se connecter au robot. Arrêt.")
        sys.exit(1)
        
    # 3. Localisation et Suivi
    localizer = Localizer()
    follower = TrajectoryFollower(robot, localizer, simulation=(args.mode == 'simulation'))
    
    if not follower.load_trajectory(trajectory):
        logger.error("❌ Impossible de charger la trajectoire. Arrêt.")
        robot.disconnect()
        sys.exit(1)
        
    # Thread de localisation continue
    stop_event = threading.Event()
    
    def odometry_loop():
        # Pour le mode simulation, on met à jour dans la boucle principale
        # car on veut conserver la synchronisation
        if args.mode == 'simulation':
            return
            
        while not stop_event.is_set():
            state = robot.read_state()
            if 'encoder_left' in state and 'encoder_right' in state:
                localizer.update(state['encoder_left'], state['encoder_right'])
            time.sleep(0.01)

    odo_thread = threading.Thread(target=odometry_loop, daemon=True)
    odo_thread.start()
    
    try:
        # 4. Exécution
        logger.info("🚀 Lancement du suivi de trajectoire...")
        success = follower.follow(max_time=300.0, max_velocity=0.3)
        
        if success:
            logger.info("🎉 Traçage terminé avec succès!")
        else:
            logger.warning("⚠️ Le traçage s'est arrêté avant la fin.")
            
        if args.validate and hasattr(follower, 'metrics'):
            report = {
                "timestamp": datetime.now().isoformat(),
                "file": str(pdf_path.name),
                "metrics": follower.metrics,
                "config": {
                    "mode": args.mode,
                    "controller": args.controller,
                    "scanned": args.scanned,
                    "tool_offset": [args.tool_offset_x, args.tool_offset_y]
                }
            }
            report_path = pdf_path.parent / f"report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(report_path, 'w') as f:
                json.dump(report, f, indent=4)
            logger.info(f"📄 Rapport de validation généré : {report_path}")
            
    except KeyboardInterrupt:
        logger.warning("\nArrêt manuel détecté.")
    finally:
        stop_event.set()
        odo_thread.join(timeout=1.0)
        robot.set_motor_speed(0, 0)
        robot.disconnect()
        logger.info("👋 Robot déconnecté. Au revoir!")

if __name__ == "__main__":
    main()

# ✅ FIXED: [BUG RPI-7: Created main.py CLI entry point with mode, pdf, and port parameters]
