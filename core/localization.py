"""
Localisation du robot (odométrie)
"""

import numpy as np
from typing import Tuple
from utils.config import RobotConfig, logger

class Localizer:
    """Estime la position du robot"""
    
    def __init__(self, config: RobotConfig = None):
        if config is None:
            config = RobotConfig()
        
        self.config = config
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        
        logger.info(f"🗺️ Localizer initialized")
        logger.info(f"   Wheel diameter: {config.WHEEL_DIAMETER} m")
        logger.info(f"   Wheel base: {config.WHEEL_BASE} m")
        logger.info(f"   PPR: {config.PPR}")
    
    def update(self, encoder_left: int, encoder_right: int):
        """Met à jour la position avec support du recul et filtrage de bruit.
        
        Args:
            encoder_left: Valeur CUMULÉE (absolue) de l'encodeur gauche
            encoder_right: Valeur CUMULÉE (absolue) de l'encodeur droit
        """
        
        delta_left = encoder_left - self.last_encoder_left
        delta_right = encoder_right - self.last_encoder_right
        
        # Détection de reset matériel ou overflow (saut massif > 50000 ticks)
        # On ignore le delta si un saut anormal est détecté pour éviter une dérive violente
        OVERFLOW_THRESHOLD = 50000
        if abs(delta_left) > OVERFLOW_THRESHOLD or abs(delta_right) > OVERFLOW_THRESHOLD:
            logger.warning(f"⚠️ Saut d'encodeur détecté (L:{delta_left}, R:{delta_right}). Reset des compteurs.")
            self.last_encoder_left = encoder_left
            self.last_encoder_right = encoder_right
            return

        # Filtrage de bruit : ignorer les micro-oscillations (jitter matériel)
        # Utile sur Raspberry Pi avec des interruptions GPIO bruitées
        NOISE_THRESHOLD = 0.1 # On peut ajuster ce seuil (en ticks ou m)
        if abs(delta_left) < NOISE_THRESHOLD: delta_left = 0
        if abs(delta_right) < NOISE_THRESHOLD: delta_right = 0
        
        if delta_left == 0 and delta_right == 0:
            return
            
        self.last_encoder_left = encoder_left
        self.last_encoder_right = encoder_right
        
        perimeter = np.pi * self.config.WHEEL_DIAMETER
        dist_left = (delta_left / self.config.PPR) * perimeter
        dist_right = (delta_right / self.config.PPR) * perimeter
        
        dist_avg = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.config.WHEEL_BASE
        
        self.x += dist_avg * np.cos(self.theta)
        self.y += dist_avg * np.sin(self.theta)
        self.theta += delta_theta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Retourne la pose"""
        return (self.x, self.y, self.theta)
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Réinitialise"""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        logger.info(f"🔄 Localizer reset to ({x:.2f}, {y:.2f}, {theta:.2f})")
    
    def set_pose(self, x: float, y: float, theta: float):
        """Définit la pose"""
        self.x = x
        self.y = y
        self.theta = theta
        logger.info(f"📍 Pose corrected to ({x:.2f}, {y:.2f}, {theta:.2f})")
    
    def __repr__(self) -> str:
        return f"Localizer(x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f})"


if __name__ == "__main__":
    localizer = Localizer()
    print("\n📍 Test: Avance 1 mètre")
    for i in range(0, 100, 2):
        localizer.update(i, i)
        x, y, theta = localizer.get_pose()
        if i % 20 == 0:
            print(f"   Encoders: {i:3d} → x={x:.3f}m")
    print("✅ Localizer test complete!")
    
    # ✅ FIXED: [BUG 2: Odometry encoder API ambiguity + overflow validation]
