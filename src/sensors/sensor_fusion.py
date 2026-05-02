"""
Sensor Fusion - Fusion des données de capteurs (OPTIMISÉ)
Kalman 1D par canal ultrason, fusion IMU/encodeur
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Dict
from .ultrasonic_sensor import UltrasonicSensor, UltrasonicReading
from .imu_sensor import IMUSensor, IMUReading


@dataclass
class FusedSensorReading:
    """Fusion de tous les capteurs"""
    position_x: float
    position_y: float
    heading: float
    velocities: Dict
    distances: Dict
    imu_data: IMUReading
    ultrasonic_data: UltrasonicReading
    timestamp: float


class OneDKalman:
    """Kalman 1D simple pour filtrage distance ultrason"""

    def __init__(self, q: float = 0.01, r: float = 0.05):
        self.x = 2.0  # estimate
        self.P = 1.0  # error covariance
        self.Q = q    # process noise
        self.R = r    # measurement noise

    def update(self, z: float) -> float:
        # Prediction
        self.P += self.Q
        # Update
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P *= (1 - K)
        return self.x

    def reset(self):
        self.x = 2.0
        self.P = 1.0


class SensorFusion:
    """Fusion capteurs optimisée pour Raspberry Pi"""

    def __init__(self, ultrasonic: UltrasonicSensor, imu: IMUSensor,
                 wheel_base: float = 0.15, wheel_radius: float = 0.033):
        self.ultrasonic = ultrasonic
        self.imu = imu
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.heading = 0.0

        # Kalman 1D par direction ultrason
        self._distance_kalman = {
            'front': OneDKalman(q=0.005, r=0.02),
            'back': OneDKalman(q=0.005, r=0.02),
            'left': OneDKalman(q=0.005, r=0.02),
            'right': OneDKalman(q=0.005, r=0.02),
        }

        self._last_encoder_left = 0
        self._last_encoder_right = 0
        self._last_update_time = time.monotonic()

    def start(self):
        self.ultrasonic.start()
        self.imu.start()
        self._last_update_time = time.monotonic()

    def stop(self):
        self.ultrasonic.stop()
        self.imu.stop()

    def get_reading(self) -> FusedSensorReading:
        ultra = self.ultrasonic.get_reading()
        imu = self.imu.get_reading()

        # Filtre Kalman 1D sur distances
        filtered_distances = {
            'front': self._distance_kalman['front'].update(ultra.front),
            'back': self._distance_kalman['back'].update(ultra.back),
            'left': self._distance_kalman['left'].update(ultra.left),
            'right': self._distance_kalman['right'].update(ultra.right),
        }

        self.heading = imu.yaw

        return FusedSensorReading(
            position_x=self.position[0],
            position_y=self.position[1],
            heading=self.heading,
            velocities={
                'longitudinal': 0.0,
                'angular': imu.angular_vel_z,
            },
            distances=filtered_distances,
            imu_data=imu,
            ultrasonic_data=ultra,
            timestamp=time.monotonic()
        )

    def update_position_from_odometry(self, delta_left: float, delta_right: float):
        delta_dist = (delta_left + delta_right) / 2.0
        delta_heading = (delta_right - delta_left) / self.wheel_base

        self.position[0] += delta_dist * np.cos(self.heading + delta_heading / 2)
        self.position[1] += delta_dist * np.sin(self.heading + delta_heading / 2)
        self.heading += delta_heading
        self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))

    def get_estimated_state(self) -> Dict:
        reading = self.get_reading()
        return {
            'x': reading.position_x,
            'y': reading.position_y,
            'theta': reading.heading,
            'heading_degrees': np.degrees(reading.heading),
            'distances': reading.distances,
            'imu_yaw': np.degrees(reading.imu_data.yaw),
        }

    def reset_position(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.position = np.array([x, y])
        self.heading = theta
        for k in self._distance_kalman.values():
            k.reset()


if __name__ == "__main__":
    print("=== Test SensorFusion Optimisé ===\n")
    ultra = UltrasonicSensor(use_gpio=False)
    imu = IMUSensor(use_i2c=False)
    fusion = SensorFusion(ultra, imu)
    fusion.start()

    for i in range(10):
        reading = fusion.get_reading()
        print(f"[{i+1:02d}] F={reading.distances['front']:.2f}m  "
              f"θ={np.degrees(reading.heading):5.1f}°")
        time.sleep(0.1)

    fusion.stop()
    print("\n✅ Test complété")

