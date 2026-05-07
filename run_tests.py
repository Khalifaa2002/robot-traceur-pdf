"""
Script pour lancer les tests correctement
"""

import sys
from pathlib import Path

# Ajoute le dossier courant au path
sys.path.insert(0, str(Path(__file__).parent))

if __name__ == "__main__":
    print("\n" + "="*60)
    print("ROBOT TRACEUR PDF - TEST SUITE")
    print("="*60)
    
    # Test 1: Imports
    print("\n1: Testing Imports...")
    print("-"*60)
    
    try:
        from utils.config import RobotConfig, TrajectoryConfig, PIDConfig, logger
        print("  OK: config imported")
        
        from hardware.base import RobotBase, RobotFactory
        from hardware.simulator import RobotSimulator
        from hardware.serial import SerialRobotInterface
        from hardware.rpi_gpio import RPiGPIOInterface
        print("  OK: hardware imported")
        
        from localization.odometry import Localizer
        print("  OK: localizer imported")
        
        from control.pid import PIDController, create_linear_controller
        from app.mission import TrajectoryFollower
        print("  OK: controller & mission imported")
        
        print("\nAll imports successful!")
        
    except ImportError as e:
        print(f"\nImport failed: {e}")
        sys.exit(1)
    
    # Test 2: Robot Simulator
    print("\n2: Testing RobotSimulator...")
    print("-"*60)
    
    try:
        robot = RobotSimulator()
        
        if robot.connect():
            print("  OK: Connected")
            
            # Simule un mouvement
            for i in range(20):
                robot.set_motor_speed(0.5, 0.5)
                robot.update()
            
            x, y, theta = robot.get_pose()
            print(f"  OK: Moved to ({x:.3f}, {y:.3f}, {theta:.3f})")
            
            robot.disconnect()
            print("  OK: Disconnected")
        else:
            print("  Error: Failed to connect")
            sys.exit(1)
    
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)
    
    # Test 3: Localizer
    print("\n3: Testing Localizer...")
    print("-"*60)
    
    try:
        localizer = Localizer()
        print(f"  OK: Created: {localizer}")
        
        # Simule des encodeurs
        for i in range(0, 50, 10):
            localizer.update(i, i)
        
        x, y, theta = localizer.get_pose()
        print(f"  OK: Updated to ({x:.3f}, {y:.3f}, {theta:.3f})")
        
        localizer.reset()
        x, y, theta = localizer.get_pose()
        print(f"  OK: Reset to ({x:.3f}, {y:.3f}, {theta:.3f})")
    
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)
    
    # Test 4: PID Controller
    print("\n4: Testing PIDController...")
    print("-"*60)
    
    try:
        import numpy as np
        
        pid = create_linear_controller()
        print(f"  OK: Created: {pid}")
        
        # Simule une réponse
        for t in np.linspace(0, 5, 100):
            error = 1.0
            output = pid.update(error, dt=0.01)
        
        print(f"  OK: Final output: {output:.3f}")
        
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)
    
    # Test 5: Integration
    print("\n5: Testing Integration...")
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
        print(f"  OK: Trajectory loaded: {len(trajectory)} points")
        
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
        
        print(f"  OK: Executed {follower.current_waypoint_idx} waypoints")
        
        robot.disconnect()
        
    except Exception as e:
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
        
    # Test 6: Raspberry Pi Support Code
    print("\n6: Testing RPi Adapters (mocked)...")
    print("-"*60)
    
    try:
        from hardware.rpi_gpio import RPiGPIOInterface
        print("  OK: RPiGPIOInterface imported")
        
        gpio_robot = RPiGPIOInterface()
        print("  OK: RPiGPIOInterface instanciated")
        
    except ImportError as e:
        print(f"  Error (Normal if not on RPi and no mock): {e}")
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)
    
    # Test 7: Pure Pursuit correctness
    print("\n7: Testing Pure Pursuit Controller...")
    print("-"*60)

    try:
        from control.pure_pursuit import PurePursuitController, TargetCourse
        import numpy as np
        
        # Build a simple straight-line course
        cx = np.linspace(0, 2, 50).tolist()
        cy = [0.0] * 50
        course = TargetCourse(cx, cy)
        pp = PurePursuitController(max_v=0.4, lookahead_min=0.15)
        
        # Robot at start, heading forward
        v, omega, idx = pp.compute(0.0, 0.0, 0.0, 0.2, course)
        assert abs(v) > 0, "v should be positive (moving forward)"
        assert abs(omega) < 0.5, f"omega should be small on straight line, got {omega:.3f}"
        assert 0 <= idx < len(cx), "look_ind must be valid index"
        print(f"  OK: Straight line: v={v:.3f}, omega={omega:.3f}, idx={idx}")
        
        # Test goal detection — robot at last point
        last_x, last_y = cx[-1], cy[-1]
        reached = pp.is_goal_reached(last_x, last_y, course, tolerance=0.10)
        assert reached, "is_goal_reached must be True when at final point"
        print("  OK: Goal detection works at final point")
        
        # Test speed=0 doesn't crash (uses min lookahead)
        v2, omega2, idx2 = pp.compute(0.0, 0.0, 0.0, 0.0, course)
        assert np.isfinite(v2) and np.isfinite(omega2), "Must not produce NaN/inf"
        print(f"  OK: Speed=0: v={v2:.3f}, omega={omega2:.3f} (no crash)")
        
        # Test wheel speed bounds from simulator
        from hardware.simulator import RobotSimulator
        sim = RobotSimulator()
        sim.connect()
        speed = sim.get_current_speed()
        assert isinstance(speed, float), "get_current_speed() must return float"
        print(f"  OK: get_current_speed() returns: {speed}")
        sim.disconnect()
        
        print("\n  OK: Pure Pursuit tests PASSED!")

    except Exception as e:
        print(f"  Error: {e}")
        import traceback; traceback.print_exc()
        sys.exit(1)

    # Test 8: EKF Localizer
    print("\n8: Testing EKF Localizer...")
    print("-"*60)

    try:
        from localization.ekf import EKF
        import numpy as np
        
        ekf = EKF(x0=0.0, y0=0.0, theta0=0.0, dt=0.01)
        print("  OK: EKF created")
        
        # Initial state
        x, y, theta = ekf.get_pose()
        assert x == 0.0 and y == 0.0, f"Initial pose wrong: ({x}, {y})"
        print(f"  OK: Initial pose: ({x}, {y}, {theta:.3f})")
        
        # Predict forward (straight line, v=0.3 m/s, omega=0)
        for _ in range(10):
            ekf.predict(v=0.3, omega=0.0)
        
        x2, y2, theta2 = ekf.get_pose()
        assert x2 > 0.0, f"After forward motion, x should > 0, got {x2:.4f}"
        print(f"  OK: After 10 predict steps: x={x2:.4f}m")
        
        # Covariance grows with dead-reckoning
        cov = ekf.get_covariance()
        assert cov.shape == (3, 3), f"Covariance must be 3x3, got {cov.shape}"
        assert cov[0, 0] > 0, "P[0,0] must be positive"
        print(f"  OK: Covariance shape: {cov.shape}, P[0,0]={cov[0,0]:.6f}")
        
        # Uncertainty metric
        unc = ekf.get_position_uncertainty()
        assert unc >= 0, "Uncertainty must be non-negative"
        print(f"  OK: Position uncertainty: {unc*100:.2f}cm")
        
        # Reset
        ekf.reset(1.0, 2.0, 0.5)
        x3, y3, theta3 = ekf.get_pose()
        assert abs(x3 - 1.0) < 1e-9 and abs(y3 - 2.0) < 1e-9
        print(f"  OK: Reset to (1.0, 2.0, 0.5): ({x3}, {y3}, {theta3:.3f})")
        
        print("\n  OK: EKF Localizer tests PASSED!")

    except Exception as e:
        print(f"  Error: {e}")
        import traceback; traceback.print_exc()
        sys.exit(1)

    # Test 9: DWA Planner
    print("\n9: Testing DWA Planner...")
    print("-"*60)

    try:
        from planning.dwa import DWAPlanner, DWAConfig, ultrasonic_to_obstacles
        import numpy as np
        
        dwa = DWAPlanner()
        print("  OK: DWAPlanner created")
        
        # No obstacles — robot should move toward goal
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        goal = np.array([2.0, 0.0])
        obstacles = np.zeros((0, 2))
        
        v, omega = dwa.compute(state, goal, obstacles)
        assert np.isfinite(v) and np.isfinite(omega), "Outputs must be finite"
        assert v >= 0, f"Speed must be >= 0, got {v:.3f}"
        assert v <= DWAConfig.max_speed, f"Speed must be <= max_speed={DWAConfig.max_speed}"
        print(f"  OK: No obstacles: v={v:.3f}, omega={omega:.3f}")
        
        # Wheel speeds in bounds
        vl, vr = dwa.to_wheel_speeds(0.3, 0.5)
        assert abs(vl) <= 1.0 and abs(vr) <= 1.0, f"Wheel speeds must be <=1.0, got vL={vl:.3f}, vR={vr:.3f}"
        print(f"  OK: to_wheel_speeds(0.3, 0.5): vL={vl:.3f}, vR={vr:.3f}")
        
        # Ultrasonic conversion
        obs = ultrasonic_to_obstacles(0.3, 2.0, 2.0, 2.0, 0.0, 0.0, 0.0, max_range=0.6)
        assert obs.shape == (1, 2), f"Expected 1 obstacle (front), got {obs.shape}"
        print(f"  OK: ultrasonic_to_obstacles: {len(obs)} obstacle(s) detected")
        
        # All clear
        obs_clear = ultrasonic_to_obstacles(1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, max_range=0.6)
        assert len(obs_clear) == 0, f"Expected 0 obstacles (all > max_range), got {len(obs_clear)}"
        print(f"  OK: All clear: 0 obstacles (all readings > max_range)")
        
        print("\n  OK: DWA Planner tests PASSED!")

    except Exception as e:
        print(f"  Error: {e}")
        import traceback; traceback.print_exc()
        sys.exit(1)

    # Summary
    print("\n" + "="*60)
    print("OK: ALL 9 SECTIONS PASSED - Robot Traceur PDF v4.1 READY")
    print("="*60 + "\n")
    sys.exit(0)

