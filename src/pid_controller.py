"""
Contrôleur PID générique
"""

import numpy as np
from typing import Optional
from .config import PIDConfig, logger

class PIDController:
    """Contrôleur PID"""
    
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_max: float = 1.0,
                 name: str = "PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self.name = name
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        
        logger.info(f"🔧 PIDController '{name}' initialized (Kp={kp}, Ki={ki}, Kd={kd})")
    
    def update(self, error: float, dt: float = 0.01) -> float:
        """Calcule la correction PID"""
        
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral
        
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)
        
        self.prev_error = error
        self.prev_time += dt
        
        return output
    
    def reset(self):
        """Réinitialise"""
        self.integral = 0.0
        self.prev_error = 0.0
        logger.debug(f"🔄 PIDController '{self.name}' reset")
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """Change les gains"""
        logger.info(f"📊 PIDController '{self.name}' gains updated:")
        logger.info(f"   Kp: {self.kp:.3f} → {kp:.3f}")
        logger.info(f"   Ki: {self.ki:.3f} → {ki:.3f}")
        logger.info(f"   Kd: {self.kd:.3f} → {kd:.3f}")
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def __repr__(self) -> str:
        return (f"PIDController({self.name}, "
                f"Kp={self.kp:.3f}, Ki={self.ki:.3f}, Kd={self.kd:.3f})")


def create_linear_controller(config: PIDConfig = None) -> PIDController:
    """Crée un PID pour le contrôle linéaire"""
    if config is None:
        config = PIDConfig()
    return PIDController(
        config.LINEAR_KP,
        config.LINEAR_KI,
        config.LINEAR_KD,
        integral_max=config.INTEGRAL_MAX,
        name="Linear PID"
    )


def create_angular_controller(config: PIDConfig = None) -> PIDController:
    """Crée un PID pour le contrôle angulaire"""
    if config is None:
        config = PIDConfig()
    return PIDController(
        config.ANGULAR_KP,
        config.ANGULAR_KI,
        config.ANGULAR_KD,
        integral_max=config.INTEGRAL_MAX,
        name="Angular PID"
    )


if __name__ == "__main__":
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05, name="Test PID")
    print("\n🧪 Test PID: Step Response")
    for t in np.linspace(0, 5, 500):
        error = 1.0
        output = pid.update(error, dt=0.01)
    print(f"Final output: {output:.3f}")
    print("✅ PID test complete!")
    
    # ✅ FIXED: [BUG 7: PID integral windup division-by-zero risk, safe clip added]
