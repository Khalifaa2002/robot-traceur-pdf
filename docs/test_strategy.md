# 🧪 Automated Test Strategy — Robot Traceur PDF v4.0

To ensure the long-term stability of the modernized modular architecture, we implement a multi-layered testing strategy.

## 1. Unit Tests (Core Algorithms)
**Tool**: `pytest`
**Scope**: Individual math and logic modules without hardware dependencies.
- **Planning**: Validate `CubicSpline2D` continuity and heading generation.
- **Control**: Validate `PurePursuitController` lookahead logic and speed scaling.
- **Localization**: Validate `EKF` prediction/update cycles with synthetic observations.
- **Perception**: Mock PDF rendering and verify contour detection logic.

**Command**: `pytest tests/ -v`

## 2. Integration Tests (System Pipeline)
**Tool**: `run_tests.py`
**Scope**: Interaction between modules (e.g., Trajectory Generation -> Mission Loop).
- Verify that `TrajectoryFollower` correctly loads paths.
- Verify that `RobotFactory` returns the correct interface for each mode.
- Verify that `Localizer` stays synced with `RobotSimulator`.

**Command**: `python run_tests.py`

## 3. Simulation Regression (End-to-End)
**Tool**: `main.py --mode simulation`
**Scope**: Full system execution with virtual hardware.
- Run a "Square Test" using a reference PDF.
- Verify that `--ekf` and `--controller pure_pursuit` flags don't cause crashes.
- Assert that `RMS Error` in the final report is below a threshold (e.g., 0.5m).

**Command**: `python main.py --mode simulation --pdf data/plan_square.pdf --controller pure_pursuit --ekf --validate`

## 4. Hardware Mock Testing (HAL Stability)
**Scope**: Verify that low-level interfaces handle missing hardware gracefully.
- **Serial**: Mock `pyserial` to verify JSON command formatting.
- **GPIO**: Mock `RPi.GPIO` to verify PWM and interrupt registration.

**Command**: `python scripts/hardware_test.py --mode gpio` (on non-RPi systems, this triggers mock fallback)

## 5. CI/CD Integration (Future)
- Run unit and integration tests on every commit.
- Use `pytest-cov` to maintain > 80% code coverage in `control/` and `localization/`.
