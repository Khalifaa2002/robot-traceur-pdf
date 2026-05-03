import time
import math
import numpy as np
from typing import Tuple, Optional, Dict
import platform
import json
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from unittest import mock

try:
    import RPi.GPIO as GPIO
except ImportError:
    # Fallback/mock pour développement sur Windows/Mac
    GPIO = mock.Mock()
    GPIO.HIGH = 1
    GPIO.LOW = 0
    GPIO.OUT = 0
    GPIO.IN = 1
    GPIO.BCM = 11

try:
    import serial
except ImportError:
    serial = mock.Mock()

from utils.config import RobotConfig
from utils.logger import logger

# ==================== ABSTRACT BASE ====================

class RobotBase(ABC):
    """Interface abstraite pour le robot"""
    
    def __init__(self):
        self.is_connected = False
        self.state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'v': 0.0,
            'omega': 0.0,
            'battery': 0.0,
            'encoder_left': 0,
            'encoder_right': 0
        }
    
    @abstractmethod
    def connect(self) -> bool:
        pass
    
    @abstractmethod
    def disconnect(self):
        pass
    
    @abstractmethod
    def send_command(self, command: str) -> bool:
        pass
    
    @abstractmethod
    def read_state(self) -> dict:
        pass
    
    @abstractmethod
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        pass
    
    @abstractmethod
    def set_draw(self, active: bool) -> bool:
        pass
    
    def get_pose(self) -> Tuple[float, float, float]:
        return (self.state['x'], self.state['y'], self.state['theta'])
    
    def get_velocity(self) -> Tuple[float, float]:
        return (self.state['v'], self.state['omega'])
    
    def is_ready(self) -> bool:
        return (self.is_connected and self.state['battery'] > 9.0)
    
    def __repr__(self) -> str:
        status = "🟢 Connected" if self.is_connected else "🔴 Disconnected"
        return f"Robot({status})"

# ==================== SIMULATOR ====================

class RobotSimulator(RobotBase):
    """Simulateur du robot"""
    
    def __init__(self):
        super().__init__()
        self.time = 0.0
        self.dt = 0.01
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.drawing = False
        
        self.wheel_diameter = RobotConfig.WHEEL_DIAMETER
        self.wheel_base = RobotConfig.WHEEL_BASE
        
        logger.info("🤖 Robot Simulator initialized")
    
    def connect(self) -> bool:
        self.is_connected = True
        self.state['battery'] = 12.0
        logger.info("✅ Simulator connected")
        return True
    
    def disconnect(self):
        self.is_connected = False
        logger.info("✅ Simulator disconnected")
    
    def send_command(self, command: str) -> bool:
        logger.debug(f"📤 Command: {command}")
        return True
    
    def read_state(self) -> dict:
        return self.state.copy()
    
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        self.motor_left = np.clip(left_speed, -1.0, 1.0)
        self.motor_right = np.clip(right_speed, -1.0, 1.0)
        
        v = (self.motor_left + self.motor_right) / 2.0 * 0.5
        omega = (self.motor_right - self.motor_left) / self.wheel_base * 1.0
        
        perimeter = np.pi * self.wheel_diameter
        ticks_left = int((v - omega * self.wheel_base / 2) * self.dt / perimeter * RobotConfig.PPR)
        ticks_right = int((v + omega * self.wheel_base / 2) * self.dt / perimeter * RobotConfig.PPR)
        self.state['encoder_left'] += ticks_left
        self.state['encoder_right'] += ticks_right
        
        self.state['x'] += v * np.cos(self.state['theta']) * self.dt
        self.state['y'] += v * np.sin(self.state['theta']) * self.dt
        self.state['theta'] += omega * self.dt
        self.state['theta'] = np.arctan2(np.sin(self.state['theta']), np.cos(self.state['theta']))
        
        self.state['v'] = v
        self.state['omega'] = omega
        
        return True
    
    def set_draw(self, active: bool) -> bool:
        self.drawing = active
        logger.debug(f"✏️ Draw: {'ON' if active else 'OFF'}")
        return True
    
    def update(self):
        self.time += self.dt
        self.state['battery'] = 12.0 - 0.001 * self.time

# ==================== SERIAL INTERFACE ====================

@dataclass
class RobotMessage:
    command: str
    params: Dict = None
    
    def to_json(self) -> str:
        msg = {'cmd': self.command}
        if self.params:
            msg.update(self.params)
        return json.dumps(msg)

class SerialRobotInterface(RobotBase):
    """Interface UART avec robot réel"""
    
    def __init__(self, port: str = None, baudrate: int = 115200, timeout: float = 1.0):
        super().__init__()
        self.port = port or RobotConfig.SERIAL_PORT
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.reading_thread = None
        self.stop_reading = False
        logger.info(f"📡 SerialRobotInterface initialized (port={self.port})")
    
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(1)
            self.is_connected = True
            self.stop_reading = False
            self.reading_thread = threading.Thread(target=self._reading_loop, daemon=True)
            self.reading_thread.start()
            logger.info(f"✅ Connected to {self.port}")
            return True
        except Exception as e:
            logger.error(f"❌ Connection error: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        if self.serial:
            self.stop_reading = True
            if self.reading_thread:
                self.reading_thread.join(timeout=2)
            self.serial.close()
            self.is_connected = False
            logger.info("✅ Disconnected")
    
    def send_command(self, command: str) -> bool:
        if not self.is_connected or not self.serial:
            return False
        try:
            self.serial.write((command + '\n').encode())
            return True
        except Exception as e:
            logger.error(f"Send error: {e}")
            return False
    
    def send_json_command(self, msg: RobotMessage) -> bool:
        return self.send_command(msg.to_json())
    
    def read_state(self) -> dict:
        return self.state.copy()
    
    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        msg = RobotMessage("MOTOR", {
            'left': round(left_speed, 3),
            'right': round(right_speed, 3)
        })
        return self.send_json_command(msg)
    
    def set_draw(self, active: bool) -> bool:
        msg = RobotMessage("DRAW", {'active': active})
        return self.send_json_command(msg)
    
    def emergency_stop(self) -> bool:
        return self.send_command("STOP")
    
    def _reading_loop(self):
        buffer = ""
        while not self.stop_reading and self.is_connected:
            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode()
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._process_message(line.strip())
                time.sleep(0.01)
            except Exception as e:
                break
    
    def _process_message(self, line: str):
        if not line:
            return
        try:
            if line.startswith('{'):
                data = json.loads(line)
                self.state.update(data)
        except json.JSONDecodeError:
            pass

# ==================== GPIO INTERFACE ====================

class RPiGPIOInterface(RobotBase):
    """Contrôle direct des moteurs via GPIO"""
    
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
            
            for pin in [self.PIN_ENA, self.PIN_IN1, self.PIN_IN2, self.PIN_ENB, self.PIN_IN3, self.PIN_IN4]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                
            self.pwm_l = GPIO.PWM(self.PIN_ENA, 1000)
            self.pwm_r = GPIO.PWM(self.PIN_ENB, 1000)
            self.pwm_l.start(0)
            self.pwm_r.start(0)
            
            for pin in [self.PIN_ENC_L_A, self.PIN_ENC_L_B, self.PIN_ENC_R_A, self.PIN_ENC_R_B]:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                
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

    def _isr_left(self, channel):
        val_a, val_b = GPIO.input(self.PIN_ENC_L_A), GPIO.input(self.PIN_ENC_L_B)
        with self._lock:
            if val_a == val_b: self.encoder_left -= 1
            else: self.encoder_left += 1

    def _isr_right(self, channel):
        val_a, val_b = GPIO.input(self.PIN_ENC_R_A), GPIO.input(self.PIN_ENC_R_B)
        with self._lock:
            if val_a == val_b: self.encoder_right -= 1
            else: self.encoder_right += 1

    def send_command(self, command: str) -> bool: return True

    def read_state(self) -> dict:
        with self._lock:
            self.state.update({'encoder_left': self.encoder_left, 'encoder_right': self.encoder_right, 'battery': 12.0})
        return self.state.copy()

    def set_motor_speed(self, left_speed: float, right_speed: float) -> bool:
        if not self.is_connected: return False
        
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        GPIO.output(self.PIN_IN1, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        GPIO.output(self.PIN_IN2, GPIO.LOW if left_speed >= 0 else GPIO.HIGH)
        GPIO.output(self.PIN_IN3, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
        GPIO.output(self.PIN_IN4, GPIO.LOW if right_speed >= 0 else GPIO.HIGH)
            
        self.pwm_l.ChangeDutyCycle(abs(left_speed) * 100)
        self.pwm_r.ChangeDutyCycle(abs(right_speed) * 100)
        return True

    def set_draw(self, active: bool) -> bool:
        return True

# ==================== FACTORY ====================

class RobotFactory:
    @staticmethod
    def create_robot(mode: str = "simulation") -> RobotBase:
        if mode in ("simulator", "simulation"):
            logger.info("🎮 Creating RobotSimulator")
            return RobotSimulator()
        elif mode == "serial":
            logger.info("🤖 Creating SerialRobotInterface")
            return SerialRobotInterface()
        elif mode == "gpio":
            logger.info("🍓 Creating RPiGPIOInterface")
            return RPiGPIOInterface()
        else:
            raise ValueError(f"Unknown mode: {mode}")
