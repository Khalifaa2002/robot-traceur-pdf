"""
Configuration globale du projet
"""

import os
import platform
import numpy as np
from pathlib import Path

# ==================== ENVIRONMENT FLAGS ====================
IS_RASPBERRY_PI = platform.machine() in ['armv7l', 'aarch64']
IS_HEADLESS = os.environ.get('DISPLAY') is None

# ==================== PATHS ====================
PROJECT_ROOT = Path(__file__).parent.parent
DATA_DIR = PROJECT_ROOT / "data"
PLANS_DIR = DATA_DIR / "plans"
TRAJECTORIES_DIR = DATA_DIR / "trajectories"
TESTS_DIR = PROJECT_ROOT / "tests"

for directory in [PLANS_DIR, TRAJECTORIES_DIR]:
    directory.mkdir(parents=True, exist_ok=True)

# ==================== ROBOT PARAMETERS ====================
class RobotConfig:
    WHEEL_DIAMETER = 0.065
    WHEEL_BASE = 0.150
    PPR = 20.0
    MAX_LINEAR_VELOCITY = 0.5
    MAX_ANGULAR_VELOCITY = 1.0
    SERIAL_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"
    BAUDRATE = 115200
    TIMEOUT = 1.0
    ODOMETRY_UPDATE_RATE = 100

# ==================== TRAJECTORY PARAMETERS ====================
class TrajectoryConfig:
    PIXEL_TO_METER = 0.001
    NUM_SMOOTH_POINTS = 100
    INTERPOLATION_KIND = 'cubic'
    LINEAR_TOLERANCE = 0.03
    ANGULAR_TOLERANCE = 0.1
    DRAW_ACTIVATION_DISTANCE = 0.05
    DRAW_OFFSET_X = 0.0
    DRAW_OFFSET_Y = 0.0

# ==================== PID PARAMETERS ====================
class PIDConfig:
    LINEAR_KP = 0.8
    LINEAR_KI = 0.15
    LINEAR_KD = 0.1
    ANGULAR_KP = 0.6
    ANGULAR_KI = 0.1
    ANGULAR_KD = 0.08
    INTEGRAL_MAX = 1.0
    OUTPUT_MIN = -1.0
    OUTPUT_MAX = 1.0

# ==================== PDF EXTRACTION ====================
class PDFConfig:
    MIN_CONTOUR_AREA = 500
    CONTOUR_EPSILON = 0.02
    BLUR_KERNEL = (5, 5)
    DILATE_ITERATIONS = 2
    ERODE_ITERATIONS = 1

# ==================== LOGGING ====================
import logging

def setup_logger(name: str, level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger

logger = setup_logger("robot-traceur")

# ==================== TESTS ====================
class TestConfig:
    COMMUNICATION_TIMEOUT = 5.0
    TRAJECTORY_TIMEOUT = 300.0
    ERROR_TOLERANCE = 0.1
    SIMULATION_FPS = 30
    SIMULATION_TIMESTEP = 1.0 / SIMULATION_FPS

# ==================== CONSTANTS ====================
PI = np.pi
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI

STATE_IDLE = 0
STATE_MOVING = 1
STATE_DRAWING = 2
STATE_ERROR = -1

if __name__ == "__main__":
    print("📋 Configuration:")
    print(f"   Project root: {PROJECT_ROOT}")
    print(f"   Data dir: {DATA_DIR}")
    print(f"   Wheel diameter: {RobotConfig.WHEEL_DIAMETER} m")
    print(f"   Wheel base: {RobotConfig.WHEEL_BASE} m")
    print(f"   PPR: {RobotConfig.PPR}")
    print(f"   Platform: {'Raspberry Pi' if IS_RASPBERRY_PI else platform.system()}")
    print(f"   Headless: {IS_HEADLESS}")
    print(f"   Serial: {RobotConfig.SERIAL_PORT}")
    print("✅ Configuration loaded!")
    
    # ✅ FIXED: [RPI-1: Serial port path, RPI-5: Auto-detect running environment]
