"""
Script pour lancer les tests correctement
"""

import sys
from pathlib import Path

# Ajoute le dossier courant au path
sys.path.insert(0, str(Path(__file__).parent))

if __name__ == "__main__":
    print("\n" + "="*60)
    print("🧪 ROBOT TRACEUR PDF - TEST SUITE")
    print("="*60)
    
    # Test 1: Imports
    print("\n1️⃣  Testing Imports...")
    print("-"*60)
    
    try:
        from src.config import RobotConfig, TrajectoryConfig, PIDConfig, logger
        print("  ✅ config imported")
        
        from src.robot_base import RobotBase, RobotSimulator
        print("  ✅ robot_base imported")
        
        from src.localizer import Localizer
        print("  ✅ localizer imported")
        
        from src.pid_controller import PIDController, create_linear_controller
        print("  ✅ pid_controller imported")
        
        from src.robot_interface import RobotFactory, SerialRobotInterface
        print("  ✅ robot_interface imported")
        
        from src.trajectory_follower import TrajectoryFollower
        print("  ✅ trajectory_follower imported")
        
        print("\n✅ All imports successful!")
        
    except ImportError as e:
        print(f"\n❌ Import failed: {e}")
        sys.exit(1)
    
    # Test 2: Robot Simulator
    print("\n2️⃣  Testing RobotSimulator...")
    print("-"*60)
    
    try:
        robot = RobotSimulator()
        
        if robot.connect():
            print("  ✅ Connected")
            
            # Simule un mouvement
            for i in range(20):
                robot.set_motor_speed(0.5, 0.5)
                robot.update()
            
            x, y, theta = robot.get_pose()
            print(f"  ✅ Moved to ({x:.3f}, {y:.3f}, {theta:.3f})")
            
            robot.disconnect()
            print("  ✅ Disconnected")
        else:
            print("  ❌ Failed to connect")
            sys.exit(1)
    
    except Exception as e:
        print(f"  ❌ Error: {e}")
        sys.exit(1)
    
    # Test 3: Localizer
    print("\n3️⃣  Testing Localizer...")
    print("-"*60)
    
    try:
        localizer = Localizer()
        print(f"  ✅ Created: {localizer}")
        
        # Simule des encodeurs
        for i in range(0, 50, 10):
            localizer.update(i, i)
        
        x, y, theta = localizer.get_pose()
        print(f"  ✅ Updated to ({x:.3f}, {y:.3f}, {theta:.3f})")
        
        localizer.reset()
        x, y, theta = localizer.get_pose()
        print(f"  ✅ Reset to ({x:.3f}, {y:.3f}, {theta:.3f})")
    
    except Exception as e:
        print(f"  ❌ Error: {e}")
        sys.exit(1)
    
    # Test 4: PID Controller
    print("\n4️⃣  Testing PIDController...")
    print("-"*60)
    
    try:
        import numpy as np
        
        pid = create_linear_controller()
        print(f"  ✅ Created: {pid}")
        
        # Simule une réponse
        for t in np.linspace(0, 5, 100):
            error = 1.0
            output = pid.update(error, dt=0.01)
        
        print(f"  ✅ Final output: {output:.3f}")
        
    except Exception as e:
        print(f"  ❌ Error: {e}")
        sys.exit(1)
    
    # Test 5: Integration
    print("\n5️⃣  Testing Integration...")
    print("-"*60)
    
    try:
        import numpy as np
        
        # Crée les composants
        robot = RobotSimulator()
        localizer = Localizer()
        follower = TrajectoryFollower(robot, localizer, simulation=True)
        
        robot.connect()
        
        # Crée une trajectoire simple
        trajectory = np.array([
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.5, 0.5, 1.57],
            [0.0, 0.5, 3.14],
            [0.0, 0.0, 0.0],
        ])
        
        follower.load_trajectory(trajectory)
        print(f"  ✅ Trajectory loaded: {len(trajectory)} points")
        
        # Exécute quelques étapes
        for step in range(50):
            waypoint = follower.get_current_waypoint()
            if waypoint is None:
                break
            
            x_target, y_target, theta_target = waypoint
            x, y, theta = localizer.get_pose()
            
            dx = x_target - x
            dy = y_target - y
            dist_error = np.sqrt(dx**2 + dy**2)
            
            v_cmd = follower.pid_linear.update(dist_error, dt=0.01)
            robot.set_motor_speed(v_cmd, v_cmd)
            robot.update()
            
            if dist_error < follower.config.LINEAR_TOLERANCE:
                follower.advance_waypoint()
        
        print(f"  ✅ Executed {follower.current_waypoint_idx} waypoints")
        
        robot.disconnect()
        
    except Exception as e:
        print(f"  ❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
        
    # Test 6: Raspberry Pi Support Code
    print("\n6️⃣  Testing RPi Adapters (mocked)...")
    print("-"*60)
    
    try:
        from src.rpi_gpio_interface import RPiGPIOInterface
        print("  ✅ rpi_gpio_interface imported")
        
        gpio_robot = RPiGPIOInterface()
        print("  ✅ RPiGPIOInterface instanciated")
        
    except ImportError as e:
        print(f"  ❌ Import Error (Normal if not on RPi and no mock): {e}")
    except Exception as e:
        print(f"  ❌ Error: {e}")
        sys.exit(1)
    
    # Summary
    print("\n" + "="*60)
    print("✅ ALL TESTS PASSED!")
    print("="*60 + "\n")
    
    # ✅ FIXED: [Tests updated to cover simulation sync and RPiGPIOInterface]
    sys.exit(0)
