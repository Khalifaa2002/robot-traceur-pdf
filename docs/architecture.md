# 🏗️ Robot Traceur PDF: v4.0 Architecture Detail

This document describes the modular architecture implemented in v4.0, which decouples hardware, algorithms, and application logic.

## 📁 Package Structure

### 1. `app/` (Orchestration)
- `main.py`: Entry point for all operations.
- `cli.py`: Command-line interface logic and pipeline orchestration.
- `mission.py`: The `TrajectoryFollower` class which coordinates robot, localizer, and controllers.

### 2. `planning/` (Trajectory)
- `trajectory_generator.py`: Converts PDF contours into smoothed robot paths.
- `cubic_spline.py`: High-performance $C^2$ spline interpolation (adapted from PythonRobotics).

### 3. `control/` (Motion Control)
- `pid.py`: Standard PID control logic for linear and angular errors.
- `pure_pursuit.py`: Geometric path-tracking controller (adapted from PythonRobotics).

### 4. `localization/` (State Estimation)
- `odometry.py`: Standard differential-drive encoder integration.
- `ekf.py`: Extended Kalman Filter for state estimation and future sensor fusion.

### 5. `perception/` (Input Processing)
- `pdf_extractor.py`: PDF rendering and contour extraction using OpenCV.

### 6. `hardware/` (Hardware Abstraction Layer)
- `base.py`: Abstract base class `RobotBase` and `RobotFactory`.
- `simulator.py`: Virtual robot for testing without physical hardware.
- `rpi_gpio.py`: Direct Raspberry Pi GPIO/L298N control.
- `serial.py`: UART/Serial interface for Arduino-controlled robots.

### 7. `telemetry/` (Analysis)
- `logger.py`: Automated mission data logging (JSON/CSV).

---

## 🔄 Data Flow (Mission Execution)

1.  **Input**: `cli.py` triggers `PDFExtractor` to generate a raw waypoint set.
2.  **Planning**: `TrajectoryGenerator` applies `CubicSpline2D` to smooth the path.
3.  **Execution**: `TrajectoryFollower` enters a high-speed loop (100Hz):
    - `Localizer` updates the robot pose (via Odometry or EKF).
    - `Controller` (PID or Pure Pursuit) calculates `v` and `omega`.
    - `RobotInterface` sends commands to hardware.
    - `MissionTelemetry` records the state.
4.  **Completion**: Telemetry is saved to `data/telemetry/` for post-mission audit.
