"""
Interface Raspberry Pi GPIO direct (sans Arduino)
"""

import time
import threading
from typing import Dict, Tuple

try:
    import RPi.GPIO as GPIO
except ImportError:
    # Fallback/mock pour développement sur Windows/Mac
    import unittest.mock as mock
    GPIO = mock.Mock()
    GPIO.HIGH = 1
    GPIO.LOW = 0
    GPIO.OUT = 0
    GPIO.IN = 1
    GPIO.BCM = 11

from .robot_base import RobotBase
from .config import logger, RobotConfig

class RPiGPIOInterface(RobotBase):
    """
    Contrôle direct des moteurs via GPIO (L298N) et 
    lecture des encodeurs via interruptions matérielles.
    """
    
    # Motor L298N Pins
    PIN_ENA = 12  # PWM Gauche
    PIN_IN1 = 5   # Dir Gauche A
    PIN_IN2 = 6   # Dir Gauche B
    
    PIN_ENB = 13  # PWM Droit
    PIN_IN3 = 19  # Dir Droit A
    PIN_IN4 = 26  # Dir Droit B
    
    # Encoder Pins
    PIN_ENC_L_A = 17
    PIN_ENC_L_B = 27
    PIN_ENC_R_A = 22
    PIN_ENC_R_B = 23

    def __init__(self):
        super().__init__()
        self.pwm_l = None
        self.pwm_r = None
        
        # Encoders
        self.encoder_left = 0
        self.encoder_right = 0
        self._lock = threading.Lock()
        
        # Odométrie
        self.last_update_time = time.time()
        
        logger.info("🍓 RPiGPIOInterface initialized")

    def connect(self) -> bool:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup Motor Pins
            for pin in [self.PIN_ENA, self.PIN_IN1, self.PIN_IN2, 
                        self.PIN_ENB, self.PIN_IN3, self.PIN_IN4]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                
            # Setup PWM (1kHz)
            self.pwm_l = GPIO.PWM(self.PIN_ENA, 1000)
            self.pwm_r = GPIO.PWM(self.PIN_ENB, 1000)
            self.pwm_l.start(0)
            self.pwm_r.start(0)
            
            # Setup Encoder Pins
            for pin in [self.PIN_ENC_L_A, self.PIN_ENC_L_B, 
                        self.PIN_ENC_R_A, self.PIN_ENC_R_B]:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                
            # Interruption sur canal A
            GPIO.add_event_detect(self.PIN_ENC_L_A, GPIO.BOTH, callback=self._isr_left)
            GPIO.add_event_detect(self.PIN_ENC_R_A, GPIO.BOTH, callback=self._isr_right)
            
            self.is_connected = True
            logger.info("✅ GPIO Pins configured and enabled")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to setup GPIO: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        if self.is_connected:
            self.set_motor_speed(0, 0)
            if self.pwm_l:
                self.pwm_l.stop()
            if self.pwm_r:
                self.pwm_r.stop()
            GPIO.cleanup()
            self.is_connected = False
            logger.info("✅ GPIO disconnected and cleaned up")

    def _isr_left(self, channel):
        """Interrupt Service Routine for Left Encoder"""
        val_a = GPIO.input(self.PIN_ENC_L_A)
        val_b = GPIO.input(self.PIN_ENC_L_B)
        with self._lock:
            if val_a == val_b:
                self.encoder_left -= 1
            else:
                self.encoder_left += 1

    def _isr_right(self, channel):
        """Interrupt Service Routine for Right Encoder"""
        val_a = GPIO.input(self.PIN_ENC_R_A)
        val_b = GPIO.input(self.PIN_ENC_R_B)
        with self._lock:
            if val_a == val_b:
                self.encoder_right -= 1
            else:
                self.encoder_right += 1

    def send_command(self, command: str) -> bool:
        # Not applicable for direct GPIO, but implement for compatibility
        return True

    def read_state(self) -> dict:
        """Retourne l'état du robot et met à jour l'odométrie"""
        with self._lock:
            el = self.encoder_left
            er = self.encoder_right
            
        self.state.update({
            'encoder_left': el,
            'encoder_right': er,
            'battery': 12.0 # ADC if connected
        })
        return self.state.copy()

    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        if not self.is_connected:
            return False
            
        # Limite -1.0 à 1.0
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Moteur Gauche
        if left_speed >= 0:
            GPIO.output(self.PIN_IN1, GPIO.HIGH)
            GPIO.output(self.PIN_IN2, GPIO.LOW)
        else:
            GPIO.output(self.PIN_IN1, GPIO.LOW)
            GPIO.output(self.PIN_IN2, GPIO.HIGH)
            
        # Moteur Droit
        if right_speed >= 0:
            GPIO.output(self.PIN_IN3, GPIO.HIGH)
            GPIO.output(self.PIN_IN4, GPIO.LOW)
        else:
            GPIO.output(self.PIN_IN3, GPIO.LOW)
            GPIO.output(self.PIN_IN4, GPIO.HIGH)
            
        # Set PWM duty cycle (0-100)
        self.pwm_l.ChangeDutyCycle(abs(left_speed) * 100)
        self.pwm_r.ChangeDutyCycle(abs(right_speed) * 100)
        
        return True

    def set_draw(self, active: bool) -> bool:
        logger.info(f"✏️  Drawing mechanism (Servo) set to: {active}")
        # Could add servo PWM control here
        return True

# ✅ FIXED: [BUG RPI-2: Added GPIO support for direct Raspberry Pi control without Arduino]
