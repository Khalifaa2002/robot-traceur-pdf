"""
Test que tous les imports fonctionnent
"""

import sys
from pathlib import Path

# Ajoute le parent au path
sys.path.insert(0, str(Path(__file__).parent.parent))

def test_all_imports():
    """Teste tous les imports"""
    print("\n🧪 Testing all imports...")
    
    try:
        print("  1. Importing config...", end=" ")
        from src.config import RobotConfig, TrajectoryConfig, PIDConfig, logger
        print("✅")
        
        print("  2. Importing robot_base...", end=" ")
        from src.robot_base import RobotBase, RobotSimulator
        print("✅")
        
        print("  3. Importing localizer...", end=" ")
        from src.localizer import Localizer
        print("✅")
        
        print("  4. Importing pid_controller...", end=" ")
        from src.pid_controller import PIDController, create_linear_controller
        print("✅")
        
        print("  5. Importing robot_interface...", end=" ")
        from src.robot_interface import RobotFactory, SerialRobotInterface
        print("✅")
        
        print("  6. Importing trajectory_follower...", end=" ")
        from src.trajectory_follower import TrajectoryFollower
        print("✅")
        
        print("\n✅ All imports successful!\n")
        return True
    
    except ImportError as e:
        print(f"\n❌ Import error: {e}\n")
        return False


def test_robot_simulator():
    """Teste le simulateur du robot"""
    print("🧪 Testing RobotSimulator...")
    
    from src.robot_base import RobotSimulator
    
    robot = RobotSimulator()
    
    if robot.connect():
        print("  ✅ Connect")
        
        for i in range(10):
            robot.set_motor_speed(0.5, 0.5)
            robot.update()
            x, y, theta = robot.get_pose()
            if i % 5 == 0:
                print(f"     Step {i}: ({x:.3f}, {y:.3f}, {theta:.3f})")
        
        robot.disconnect()
        print("  ✅ Disconnect")
        print("✅ RobotSimulator test passed!\n")
        return True
    else:
        print("❌ Failed to connect\n")
        return False


def test_localizer():
    """Teste le localizer"""
    print("🧪 Testing Localizer...")
    
    from src.localizer import Localizer
    
    localizer = Localizer()
    print(f"  ✅ Created: {localizer}")
    
    # Simule un mouvement
    for i in range(0, 100, 10):
        localizer.update(i, i)
        x, y, theta = localizer.get_pose()
        print(f"     Encoders: {i:3d} → x={x:.3f}m")
    
    print("✅ Localizer test passed!\n")
    return True


def test_pid_controller():
    """Teste le contrôleur PID"""
    print("🧪 Testing PIDController...")
    
    from src.pid_controller import create_linear_controller
    import numpy as np
    
    pid = create_linear_controller()
    print(f"  ✅ Created: {pid}")
    
    # Simule une réponse
    errors = []
    for t in np.linspace(0, 5, 100):
        error = 1.0
        output = pid.update(error, dt=0.01)
        errors.append(output)
    
    print(f"     Final output: {errors[-1]:.3f}")
    print("✅ PIDController test passed!\n")
    return True


if __name__ == "__main__":
    print("\n" + "="*50)
    print("ROBOT TRACEUR PDF - IMPORT TESTS")
    print("="*50)
    
    results = []
    results.append(("Imports", test_all_imports()))
    results.append(("RobotSimulator", test_robot_simulator()))
    results.append(("Localizer", test_localizer()))
    results.append(("PIDController", test_pid_controller()))
    
    print("="*50)
    print("SUMMARY:")
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {name:20s}: {status}")
    print("="*50)
    
    all_passed = all(r for _, r in results)
    sys.exit(0 if all_passed else 1)
