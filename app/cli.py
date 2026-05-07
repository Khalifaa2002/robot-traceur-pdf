"""
app/cli.py
==========
Command Line Interface for the Robot Traceur PDF.

Handles argument parsing, system initialization, and mission execution.

Status: Production (migrated from v3.1 main.py)
"""

import argparse
import sys
import threading
import time
import json
from pathlib import Path
from datetime import datetime

from utils.config import RobotConfig
from utils.logger import logger
from perception.pdf_extractor import extract_path_from_pdf
from planning.trajectory_generator import (
    smooth_trajectory, pixel_to_world, add_orientation, 
    apply_tool_offset, save_trajectory
)
from hardware.base import RobotFactory
from localization.odometry import Localizer
from app.mission import TrajectoryFollower

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
    
    parser.add_argument('--ekf', action='store_true',
                        help='Activer le filtre de Kalman étendu (EKF) pour la localisation')
    
    parser.add_argument('--smooth-method', type=str, choices=['linear', 'spline'], 
                        default='spline', help='Méthode de lissage de trajectoire')
    
    return parser.parse_args()

def run_cli():
    args = parse_args()
    logger.info("🤖 Starting Robot Traceur PDF CLI")
    logger.info(f"   Mode: {args.mode}")
    
    # 1. PDF Processing
    if not args.pdf:
        logger.error("❌ No PDF file specified. Use --pdf <path>")
        sys.exit(1)
        
    pdf_path = Path(args.pdf)
    if not pdf_path.exists():
        logger.error(f"❌ PDF file {pdf_path.resolve()} does not exist.")
        sys.exit(1)
        
    logger.info(f"📄 Extracting from {pdf_path.name}...")
    original_points = extract_path_from_pdf(str(pdf_path), scanned=args.scanned, dpi=args.dpi)
    
    if original_points is None:
        logger.error("❌ Failed to extract points from PDF.")
        sys.exit(1)
        
    # Trajectory Generation Pipeline
    smooth_points = smooth_trajectory(original_points, num_points=100, method=args.smooth_method)
    world_points = pixel_to_world(smooth_points, pixel_to_meter=0.001)
    trajectory_base = add_orientation(world_points, method=args.smooth_method)
    
    # Tool Offset Compensation
    trajectory = apply_tool_offset(trajectory_base, args.tool_offset_x, args.tool_offset_y)
    
    # Save for reference
    save_trajectory(trajectory, str(pdf_path.parent / "trajectory.npy"))
    
    # 2. Hardware Initialization
    robot = RobotFactory.create_robot(args.mode)
    if args.mode == "serial" and args.port:
        robot.port = args.port
        
    if not robot.connect():
        logger.error("❌ Failed to connect to robot hardware.")
        sys.exit(1)
        
    # 3. Mission Orchestration
    localizer = Localizer()
    follower = TrajectoryFollower(
        robot, 
        localizer, 
        simulation=(args.mode == 'simulation'),
        controller_type=args.controller
    )
    follower.use_ekf = args.ekf
    
    if not follower.load_trajectory(trajectory):
        logger.error("❌ Failed to load trajectory.")
        robot.disconnect()
        sys.exit(1)
        
    # Odometry Background Thread
    stop_event = threading.Event()
    
    def odometry_loop():
        if args.mode == 'simulation':
            return # Simulation sync is handled inside follower.follow()
            
        while not stop_event.is_set():
            state = robot.read_state()
            if 'encoder_left' in state and 'encoder_right' in state:
                localizer.update(state['encoder_left'], state['encoder_right'])
            time.sleep(0.01)

    odo_thread = threading.Thread(target=odometry_loop, daemon=True)
    odo_thread.start()
    
    try:
        # 4. Execution
        logger.info("🚀 Starting mission execution...")
        success = follower.follow(max_time=300.0, max_velocity=0.3)
        
        if success:
            logger.info("🎉 Tracing mission completed successfully!")
        else:
            logger.warning("⚠️ Tracing stopped before completion.")
            
        # Validation Report
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
            logger.info(f"📄 Validation report generated: {report_path}")
            
    except KeyboardInterrupt:
        logger.warning("\nManual stop detected.")
    finally:
        stop_event.set()
        odo_thread.join(timeout=1.0)
        robot.set_motor_speed(0, 0)
        robot.disconnect()
        logger.info("👋 Robot disconnected. CLI exiting.")

if __name__ == "__main__":
    run_cli()
