"""
hardware/rpi_gpio.py
====================
Direct Raspberry Pi GPIO control for motor drivers (L298N) and encoders.

This module uses RPi.GPIO to drive PWM and handle encoder interrupts.

Status: Production (migrated from v3.1 core)
"""

import threading
from unittest import mock
from .base import RobotBase
from utils.config import logger

try:
    import RPi.GPIO as GPIO
except ImportError:
    # Development fallback
    GPIO = mock.Mock()
    GPIO.HIGH, GPIO.LOW = 1, 0
    GPIO.OUT, GPIO.IN = 0, 1
    GPIO.BCM, GPIO.PUD_UP, GPIO.BOTH = 11, 22, 33

class RPiGPIOInterface(RobotBase):
    """
    Hardware interface for direct GPIO control on Raspberry Pi.
    """
    
    # Standard Pinout (can be moved to config if needed)
    PIN_ENA, PIN_IN1, PIN_IN2 = 12, 5, 6
    PIN_ENB, PIN_IN3, PIN_IN4 = 13, 19, 26
    PIN_ENC_L_A, PIN_ENC_L_B = 17, 27
    PIN_ENC_R_A, PIN_ENC_R_B = 22, 23

    def __init__(self):
        super().__init__()
        self.pwm_l = None
        self.pwm_r = None
        self.encoder_left = 0
        self.encoder_right = 0
        self._lock = threading.Lock()
        logger.info("🍓 RPiGPIOInterface initialized")

    def connect(self) -> bool:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor output pins
            for pin in [self.PIN_ENA, self.PIN_IN1, self.PIN_IN2, self.PIN_ENB, self.PIN_IN3, self.PIN_IN4]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                
            self.pwm_l = GPIO.PWM(self.PIN_ENA, 1000)
            self.pwm_r = GPIO.PWM(self.PIN_ENB, 1000)
            self.pwm_l.start(0)
            self.pwm_r.start(0)
            
            # Setup encoder input pins
            for pin in [self.PIN_ENC_L_A, self.PIN_ENC_L_B, self.PIN_ENC_R_A, self.PIN_ENC_R_B]:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                
            # Hardware interrupts for high-speed tracking
            GPIO.add_event_detect(self.PIN_ENC_L_A, GPIO.BOTH, callback=self._isr_left)
            GPIO.add_event_detect(self.PIN_ENC_R_A, GPIO.BOTH, callback=self._isr_right)
            
            self.is_connected = True
            logger.info("✅ GPIO Pins configured")
            return True
        except Exception as e:
            logger.error(f"❌ Failed to setup GPIO: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        if self.is_connected:
            self.set_motor_speed(0, 0)
            if self.pwm_l: self.pwm_l.stop()
            if self.pwm_r: self.pwm_r.stop()
            GPIO.cleanup()
            self.is_connected = False
            logger.info("✅ GPIO cleanup complete")

    def _isr_left(self, channel):
        """Interrupt Service Routine for left encoder."""
        val_a, val_b = GPIO.input(self.PIN_ENC_L_A), GPIO.input(self.PIN_ENC_L_B)
        with self._lock:
            if val_a == val_b: self.encoder_left -= 1
            else: self.encoder_left += 1

    def _isr_right(self, channel):
        """Interrupt Service Routine for right encoder."""
        val_a, val_b = GPIO.input(self.PIN_ENC_R_A), GPIO.input(self.PIN_ENC_R_B)
        with self._lock:
            if val_a == val_b: self.encoder_right -= 1
            else: self.encoder_right += 1

    def send_command(self, command: str) -> bool: return True

    def read_state(self) -> dict:
        """Sync internal counters with the state dictionary."""
        with self._lock:
            self.state.update({
                'encoder_left': self.encoder_left, 
                'encoder_right': self.encoder_right, 
                'battery': 12.0
            })
        return self.state.copy()

    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        if not self.is_connected: return False
        
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Left Motor Direction
        GPIO.output(self.PIN_IN1, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        GPIO.output(self.PIN_IN2, GPIO.LOW if left_speed >= 0 else GPIO.HIGH)
        # Right Motor Direction
        GPIO.output(self.PIN_IN3, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
        GPIO.output(self.PIN_IN4, GPIO.LOW if right_speed >= 0 else GPIO.HIGH)
            
        self.pwm_l.ChangeDutyCycle(abs(left_speed) * 100)
        self.pwm_r.ChangeDutyCycle(abs(right_speed) * 100)
        return True

    def set_draw(self, active: bool) -> bool:
        # Implementation for pen servo / laser can be added here
        return True
