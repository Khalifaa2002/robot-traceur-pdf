"""
scripts/hardware_test.py
========================
Low-level hardware validation for motors and encoders.
Use this to verify wiring and basic motion before running missions.
"""

import time
import sys
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).parent.parent))

from hardware.base import RobotFactory
from utils.logger import logger

def test_hardware(mode='simulation', port=None):
    logger.info(f"🔍 Starting Hardware Test (Mode: {mode})")
    
    robot = RobotFactory.create_robot(mode)
    if mode == 'serial' and port:
        robot.port = port
        
    if not robot.connect():
        logger.error("❌ Failed to connect to hardware")
        return

    try:
        # 1. Encoder Test
        logger.info("📡 Testing Encoders (Rotate wheels manually)...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            state = robot.read_state()
            print(f"\rEncoder L: {state.get('encoder_left', 0)} | R: {state.get('encoder_right', 0)}", end="")
            time.sleep(0.1)
        print("\n")

        # 2. Motor Forward Test
        logger.info("🚀 Testing Motors: Forward (2 seconds @ 30%)")
        robot.set_motor_speed(0.3, 0.3)
        time.sleep(2.0)
        robot.set_motor_speed(0.0, 0.0)
        
        # 3. Motor Backward Test
        logger.info("🔙 Testing Motors: Backward (2 seconds @ 30%)")
        robot.set_motor_speed(-0.3, -0.3)
        time.sleep(2.0)
        robot.set_motor_speed(0.0, 0.0)

        # 4. Turn Test
        logger.info("🔄 Testing Motors: Turn (1 second)")
        robot.set_motor_speed(0.3, -0.3)
        time.sleep(1.0)
        robot.set_motor_speed(0.0, 0.0)

        # 5. Drawing Tool Test
        logger.info("✏️ Testing Drawing Tool")
        robot.set_draw(True)
        time.sleep(1.0)
        robot.set_draw(False)
        
        logger.info("✅ Hardware test complete!")

    except KeyboardInterrupt:
        logger.warning("\n⚠️ Test aborted by user")
    finally:
        robot.set_motor_speed(0, 0)
        robot.disconnect()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['simulation', 'serial', 'gpio'], default='simulation')
    parser.add_argument('--port', type=str)
    args = parser.parse_args()
    
    test_hardware(args.mode, args.port)
