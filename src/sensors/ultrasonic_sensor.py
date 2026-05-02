"""
Ultrasonic Sensor Module - HC-SR04 sur GPIO Raspberry Pi
Capteurs de distance par ultrason (4 capteurs: avant, arrière, gauche, droite)
OPTIMISÉ pour Raspberry Pi: filtrage médian, rejet outliers, 10 Hz
"""

import time
import numpy as np
import threading
from typing import Dict, Optional, Deque
from collections import deque
from dataclasses import dataclass


@dataclass
class UltrasonicReading:
    """Résultat d'une mesure ultrason"""
    front: float      # Distance avant (m)
    back: float       # Distance arrière (m)
    left: float       # Distance gauche (m)
    right: float      # Distance droite (m)
    timestamp: float  # Timestamp de la mesure


class MedianFilter:
    """Filtre médian pour rejet d'outliers capteurs"""
    
    def __init__(self, size: int = 5):
        self.size = size
        self.buffer: Deque[float] = deque(maxlen=size)
        self.last_valid = 2.0  # Valeur par défaut sécurisée
    
    def update(self, value: float) -> float:
        """
        Met à jour le filtre avec une nouvelle valeur.
        Rejette les outliers > 3× écart-type de la médiane.
        """
        if value < 0 or value > 4.0 or np.isnan(value):
            return self.last_valid
        
        self.buffer.append(value)
        
        if len(self.buffer) < 3:
            self.last_valid = value
            return value
        
        arr = np.array(self.buffer)
        median = np.median(arr)
        mad = np.median(np.abs(arr - median))  # Median Absolute Deviation
        
        # Rejette si outlier (MAD-based)
        if mad > 0.01 and abs(value - median) > 3.0 * mad:
            return self.last_valid
        
        self.last_valid = median
        return median
    
    def reset(self):
        self.buffer.clear()
        self.last_valid = 2.0


class UltrasonicSensor:
    """
    Gestion des 4 capteurs ultrasons HC-SR04
    
    Configuration GPIO (BCM):
    - Front: TRIG=17, ECHO=27
    - Back:  TRIG=22, ECHO=23
    - Left:  TRIG=24, ECHO=25
    - Right: TRIG=12, ECHO=16
    """
    
    # Configuration GPIO par défaut (BCM mode)
    DEFAULT_CONFIG = {
        'front': {'trig': 17, 'echo': 27},
        'back': {'trig': 22, 'echo': 23},
        'left': {'trig': 24, 'echo': 25},
        'right': {'trig': 12, 'echo': 16},
    }
    
    # Paramètres physiques
    SPEED_OF_SOUND = 343.0  # m/s (température ~20°C)
    MAX_DISTANCE = 4.0      # Distance max mesurable (m)
    MIN_DISTANCE = 0.02     # Distance min mesurable (m)
    TIMEOUT = 0.04          # Timeout mesure (secondes)
    
    # Fréquence optimisée pour Raspberry Pi (10 Hz = synchro avec boucle contrôle)
    UPDATE_RATE_HZ = 10.0
    
    def __init__(self, config: Dict = None, use_gpio: bool = False,
                 filter_size: int = 5):
        """
        Initialise les capteurs ultrasons
        
        Args:
            config: Configuration GPIO custom (sinon utilise DEFAULT_CONFIG)
            use_gpio: Si True, utilise RPi.GPIO (sinon simulation)
            filter_size: Taille du filtre médian (défaut 5)
        """
        self.config = config or self.DEFAULT_CONFIG.copy()
        self.use_gpio = use_gpio
        self.gpio = None
        self.running = False
        self._lock = threading.Lock()
        self._last_reading = UltrasonicReading(
            front=self.MAX_DISTANCE,
            back=self.MAX_DISTANCE,
            left=self.MAX_DISTANCE,
            right=self.MAX_DISTANCE,
            timestamp=time.time()
        )
        
        # Filtres médian par direction
        self._filters = {
            'front': MedianFilter(size=filter_size),
            'back': MedianFilter(size=filter_size),
            'left': MedianFilter(size=filter_size),
            'right': MedianFilter(size=filter_size),
        }
        
        if self.use_gpio:
            self._init_gpio()
    
    def _init_gpio(self):
        """Initialise RPi.GPIO (simulation si pas disponible)"""
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
            self.gpio.setmode(GPIO.BCM)
            
            # Configure tous les pins
            for direction in ['front', 'back', 'left', 'right']:
                trig = self.config[direction]['trig']
                echo = self.config[direction]['echo']
                self.gpio.setup(trig, GPIO.OUT)
                self.gpio.setup(echo, GPIO.IN)
                # Éteint les pins TRIG initialement
                self.gpio.output(trig, GPIO.LOW)
            
            time.sleep(0.5)  # Stabilisation
        except ImportError:
            print("⚠️  RPi.GPIO non disponible. Utilisation du mode simulation.")
            self.use_gpio = False
    
    def start(self):
        """Démarre la lecture continue des capteurs"""
        if self.use_gpio and self.gpio is None:
            self._init_gpio()
        
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Arrête la lecture des capteurs"""
        self.running = False
        if hasattr(self, '_thread'):
            self._thread.join(timeout=1.0)
        
        if self.use_gpio and self.gpio:
            self.gpio.cleanup()
    
    def _read_loop(self):
        """Boucle de lecture continue optimisée (10 Hz)"""
        period = 1.0 / self.UPDATE_RATE_HZ
        while self.running:
            loop_start = time.monotonic()
            distances = self._measure_all()
            with self._lock:
                self._last_reading = UltrasonicReading(
                    front=distances['front'],
                    back=distances['back'],
                    left=distances['left'],
                    right=distances['right'],
                    timestamp=time.time()
                )
            # Sommeil précis pour maintenir 10 Hz
            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _measure_distance(self, direction: str) -> float:
        """
        Mesure la distance pour une direction donnée (non-blocking, timeout strict)
        
        Args:
            direction: 'front', 'back', 'left', ou 'right'
            
        Returns:
            Distance en mètres filtrée (ou MAX_DISTANCE si timeout/erreur)
        """
        raw = self._measure_distance_raw(direction)
        return self._filters[direction].update(raw)
    
    def _measure_distance_raw(self, direction: str) -> float:
        """Mesure brute (non filtrée)"""
        if not self.use_gpio:
            # Mode simulation: bruit réaliste
            base = np.random.uniform(0.3, 1.5)
            noise = np.random.normal(0, 0.03)  # 3cm sigma
            outlier = np.random.random() < 0.05  # 5% outliers
            if outlier:
                noise += np.random.choice([-1, 1]) * np.random.uniform(0.2, 0.5)
            return np.clip(base + noise, self.MIN_DISTANCE, self.MAX_DISTANCE)
        
        try:
            trig = self.config[direction]['trig']
            echo = self.config[direction]['echo']
            
            # Envoie une impulsion
            self.gpio.output(trig, self.gpio.HIGH)
            time.sleep(0.00001)  # 10 µs
            self.gpio.output(trig, self.gpio.LOW)
            
            # Attend le front montant (timeout strict non-bloquant)
            timeout = time.monotonic() + self.TIMEOUT
            pulse_start = None
            while time.monotonic() < timeout:
                if self.gpio.input(echo) == 1:
                    pulse_start = time.monotonic()
                    break
            
            if pulse_start is None:
                return self.MAX_DISTANCE
            
            # Attend le front descendant
            pulse_end = None
            while time.monotonic() < timeout:
                if self.gpio.input(echo) == 0:
                    pulse_end = time.monotonic()
                    break
            
            if pulse_end is None:
                return self.MAX_DISTANCE
            
            # Calcule la distance
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * self.SPEED_OF_SOUND) / 2
            
            # Clamp à [MIN_DISTANCE, MAX_DISTANCE]
            distance = max(self.MIN_DISTANCE, min(distance, self.MAX_DISTANCE))
            return distance
            
        except Exception as e:
            print(f"❌ Erreur mesure {direction}: {e}")
            return self.MAX_DISTANCE
    
    def _measure_all(self) -> Dict[str, float]:
        """Mesure tous les capteurs avec filtrage"""
        return {
            'front': self._measure_distance('front'),
            'back': self._measure_distance('back'),
            'left': self._measure_distance('left'),
            'right': self._measure_distance('right'),
        }
    
    def get_reading(self) -> UltrasonicReading:
        """Retourne la dernière lecture"""
        with self._lock:
            return self._last_reading
    
    def measure_once(self) -> UltrasonicReading:
        """Effectue une mesure unique (mode synchrone)"""
        distances = self._measure_all()
        return UltrasonicReading(
            front=distances['front'],
            back=distances['back'],
            left=distances['left'],
            right=distances['right'],
            timestamp=time.time()
        )
    
    def get_grid_occupancy(self) -> Dict[str, bool]:
        """
        Retourne l'état d'occupation selon les capteurs
        Utile pour le mapping et l'évitement
        
        Returns:
            Dict avec clés 'front', 'back', 'left', 'right'
            True si obstacle proche (< 0.5 m)
        """
        reading = self.get_reading()
        threshold = 0.5  # Distance critique (m)
        
        return {
            'front': reading.front < threshold,
            'back': reading.back < threshold,
            'left': reading.left < threshold,
            'right': reading.right < threshold,
        }


# Tests unitaires
if __name__ == "__main__":
    print("=== Test Capteurs Ultrasons ===\n")
    
    # Test mode simulation
    sensor = UltrasonicSensor(use_gpio=False)
    sensor.start()
    
    print("Lecture continue (mode simulation)...")
    for i in range(5):
        reading = sensor.get_reading()
        occupancy = sensor.get_grid_occupancy()
        print(f"\nMesure {i+1}:")
        print(f"  Front: {reading.front:.3f}m, Back: {reading.back:.3f}m")
        print(f"  Left:  {reading.left:.3f}m, Right: {reading.right:.3f}m")
        print(f"  Occupancy: {occupancy}")
        time.sleep(0.1)
    
    sensor.stop()
    print("\n✅ Test complété")
