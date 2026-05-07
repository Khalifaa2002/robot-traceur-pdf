# 🛠️ Robot Traceur PDF: Calibration & Tuning Guide

This guide explains how to calibrate the physical robot for optimal performance using the new v4.0 algorithms.

## 1. Encoder Calibration (ODOMETRY)
Correct odometry is the foundation of all higher-level algorithms.

1.  **Measurement**: Verify `WHEEL_DIAMETER` and `WHEEL_BASE` in `utils/config.py`.
2.  **Verification**: 
    - Move the robot forward exactly 1 metre.
    - Check the distance reported by the localizer.
    - Adjust `PPR` (Pulses Per Revolution) or `WHEEL_DIAMETER` until the error is < 1%.

## 2. Pure Pursuit Tuning (PATH TRACKING)
Pure Pursuit relies on the "Lookahead Distance" ($L_d$).

*   **Lookahead Min (`PurePursuitConfig.LFC`)**: 
    - **Too small**: The robot will oscillate around the path (unstable).
    - **Too large**: The robot will "cut corners" and lag behind the path.
    - *Recommended start*: 0.15m - 0.25m.
*   **Lookahead Gain (`PurePursuitConfig.KP`)**: 
    - Scales $L_d$ with speed ($L_d = k \cdot v + L_{d\_min}$).
    - Helps maintain stability at higher velocities.

## 3. EKF Calibration (STATE ESTIMATION)
The Extended Kalman Filter fuses model predictions with sensor measurements.

*   **Process Noise (`EKFConfig.Q`)**: 
    - Represents how much you "trust" the kinematic model.
    - Increase if the robot has significant wheel slip.
*   **Observation Noise (`EKFConfig.R`)**: 
    - Represents how much you "trust" the odometry sensors.
    - Increase if the encoders are noisy or prone to skipped pulses.

## 4. PID Tuning (LEGACY MODE)
If using `--controller pid`:
1.  **Kp**: Increase until the robot responds quickly, but doesn't oscillate.
2.  **Ki**: Increase slowly to eliminate steady-state error (e.g., if the robot stops 2cm short of the target).
3.  **Kd**: Increase to "dampen" oscillations if Kp is high.

---

### 🧪 Tuning Workflow
1. Run a simple square trajectory in simulation:
   `python main.py --mode simulation --pdf data/plan_square.pdf --controller pure_pursuit`
2. Once satisfied in simulation, run a physical test at low speed:
   `python main.py --mode serial --pdf data/plan_square.pdf --controller pure_pursuit`
3. Analyze the `report_*.json` for RMS Error.
