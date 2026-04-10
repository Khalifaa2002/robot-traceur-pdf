"""
Interface de communication avec le robot
"""

import serial
import json
import time
import threading
from typing import Optional, Dict
from dataclasses import dataclass
from .config import RobotConfig, logger
from .robot_base import RobotBase, RobotSimulator

@dataclass
class RobotMessage:
    """Message vers le robot"""
    command: str
    params: Dict = None
    
    def to_json(self) -> str:
        msg = {'cmd': self.command}
        if self.params:
            msg.update(self.params)
        return json.dumps(msg)
    
    def __repr__(self) -> str:
        return f"RobotMessage({self.command}, {self.params})"


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
        except serial.SerialException as e:
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
            logger.debug(f"📤 Sent: {command}")
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
        logger.warning("⚠️ EMERGENCY STOP!")
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
                logger.error(f"Reading error: {e}")
                break
    
    def _process_message(self, line: str):
        if not line:
            return
        try:
            if line.startswith('{'):
                data = json.loads(line)
                self.state.update(data)
                logger.debug(f"📥 State: {data}")
            else:
                logger.debug(f"📥 Raw: {line}")
        except json.JSONDecodeError:
            logger.warning(f"JSON decode error: {line}")


class RobotFactory:
    """Crée le bon type de robot"""
    
    @staticmethod
    def create_robot(mode: str = "simulator") -> RobotBase:
        if mode == "simulator":
            logger.info("🎮 Creating RobotSimulator")
            return RobotSimulator()
        elif mode == "serial":
            logger.info("🤖 Creating SerialRobotInterface")
            return SerialRobotInterface()
        elif mode == "gpio":
            logger.info("🍓 Creating RPiGPIOInterface (Direct Raspberry Pi Control)")
            try:
                from .rpi_gpio_interface import RPiGPIOInterface
                return RPiGPIOInterface()
            except ImportError as e:
                logger.error(f"❌ Impossible de charger RPiGPIOInterface: {e}")
                raise
        else:
            raise ValueError(f"Unknown mode: {mode}")

# ✅ FIXED: [BUG RPI-1: Verified serial port uses config platform logic, RPI-2: Registered GPIO mode in factory]


if __name__ == "__main__":
    print("\n🧪 Test: Robot Communication")
    robot = RobotFactory.create_robot("simulator")
    
    if robot.connect():
        print("✅ Connected")
        for i in range(10):
            robot.set_motor_speed(0.5, 0.5)
            robot.update()
            pose = robot.get_pose()
            print(f"   Step {i}: {pose}")
        robot.disconnect()
        print("✅ Test complete!")
