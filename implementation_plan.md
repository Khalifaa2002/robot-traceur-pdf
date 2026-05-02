# Upgrade Navigation and Control System

We will upgrade the robot's navigation and control systems by integrating five new modules based on algorithms from PythonRobotics. These will be added as lightweight, optimized implementations that fit directly into the existing `robot-traceur-pdf` architecture without breaking existing features.

## Proposed Changes

### Configuration Updates
#### [MODIFY] src/config.py
- Add configuration classes for the new algorithms: `PurePursuitConfig`, `DWAConfig`, `EKFConfig`, and `SplineConfig` to allow easy tuning.

---

### Control Module
#### [NEW] src/control/pure_pursuit.py
- Implement a `PurePursuitController` class.
- Compute the steering angle and velocity based on a lookahead distance.
- Include methods to find the closest point and the target lookahead point on the path.

#### [MODIFY] src/control/__init__.py
- Import and expose `PurePursuitController`.

---

### Planning Module
#### [NEW] src/planning/spline_smoother.py
- Implement a `CubicSplineSmoother` utilizing scipy or numpy to generate a smooth, continuous path without relying on `matplotlib`.

#### [NEW] src/planning/dwa_avoidance.py
- Implement a lightweight `DynamicWindowApproach` class optimized for Raspberry Pi.
- Evaluate trajectories based on heading, clearance, and velocity to avoid obstacles.

#### [NEW] src/planning/astar_replanner.py
- Implement `AStarReplanner` that wraps the existing `AStarPlanner` logic to provide replanning capabilities seamlessly.

#### [MODIFY] src/planning/__init__.py
- Import and expose `CubicSplineSmoother`, `DynamicWindowApproach`, and `AStarReplanner`.
- Fix the broken import of `DynamicWindowApproach` that caused the tests to fail.

---

### Localization Module
#### [NEW] src/localization/ekf_localizer.py
- Implement an `EKFLocalizer` class that acts as a drop-in replacement for `Localizer`.
- Use an Extended Kalman Filter to fuse odometry and IMU data (or just odometry with a motion model for now) while keeping the same API (`update(encoder_left, encoder_right)`, `get_pose()`).

#### [MODIFY] src/localization/__init__.py
- Import and expose `EKFLocalizer`.

---

### Core Integration
#### [MODIFY] src/trajectory_generator.py
- Replace the simple linear smoothing logic with the new `CubicSplineSmoother` from `src/planning/spline_smoother.py`.

#### [MODIFY] src/trajectory_follower.py
- Integrate `PurePursuitController` as an option or default for following the trajectory.
- Ensure the fallback to `PIDController` remains available.

#### [MODIFY] main.py
- Optionally instantiate the new `EKFLocalizer` and `PurePursuitController` depending on the configuration or defaults.
- Ensure the simulation mode works perfectly with the new stack.

## Verification Plan

### Automated Tests
- Run `python run_tests.py` to ensure all existing and any newly added tests pass successfully.
- Check `test_autonomous_system.py` which currently fails due to the missing `DynamicWindowApproach` import.

### Manual Verification
- Run `python main.py --mode simulation --pdf data/plan_square.pdf` and verify that the robot successfully navigates the path using the pure pursuit controller and smoothed splines without crashing.
