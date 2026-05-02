"""
IMU Sensor Module - Capteur d'orientation (MPU6050)
OPTIMISÉ pour Raspberry Pi:
  - Burst I2C read (1 transaction vs 6)
  - Zero-rate calibration gyro au démarrage
  - Compensation dérive yaw
  - 50 Hz (suffisant pour boucle 10 Hz)
"""

import time
import numpy as np
import threading
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class IMUReading:
    """Lecture IMU"""
    roll: float
    pitch: float
    yaw: float
    angular_vel_x: float
    angular_vel_y: float
    angular_vel_z: float
    accel_x: float
    accel_y: float
    accel_z: float
    timestamp: float


class ComplementaryFilter:
    """Filtre complémentaire avec compensation dérive"""

    def __init__(self, alpha: float = 0.98, yaw_drift_rate: float = 0.0):
        self.alpha = alpha
        self.yaw_drift_rate = yaw_drift_rate
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()

    def update(self, gyro: Tuple[float, float, float],
               accel: Tuple[float, float, float],
               is_stationary: bool = False) -> Tuple[float, float, float]:
        current_time = time.monotonic()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt > 0.1 or dt <= 0:
            dt = 0.02

        wx, wy, wz = gyro
        ax, ay, az = accel

        # Compense dérive yaw
        wz_compensated = wz - self.yaw_drift_rate

        # Intègre le gyro
        self.roll += wx * dt
        self.pitch += wy * dt
        self.yaw += wz_compensated * dt

        # Correction accel (roll/pitch seulement)
        g = np.sqrt(ax**2 + ay**2 + az**2)
        if 0.5 < g < 20.0:
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

            # Recalibre drift si immobile
            if is_stationary and abs(wz) > 0.001:
                self.yaw_drift_rate = 0.995 * self.yaw_drift_rate + 0.005 * wz

        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))
        return (self.roll, self.pitch, self.yaw)


class IMUSensor:
    """IMU optimisée: burst I2C, calibration auto, 50 Hz"""

    MPU6050_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    WHO_AM_I = 0x75
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    UPDATE_RATE_HZ = 50.0

    def __init__(self, use_i2c: bool = False, address: int = MPU6050_ADDR):
        self.use_i2c = use_i2c
        self.address = address
        self.i2c = None
        self.running = False
        self._lock = threading.Lock()

        self.complementary_filter = ComplementaryFilter(alpha=0.97)
        self._calibrated = False

        self._last_reading = IMUReading(
            roll=0.0, pitch=0.0, yaw=0.0,
            angular_vel_x=0.0, angular_vel_y=0.0, angular_vel_z=0.0,
            accel_x=0.0, accel_y=0.0, accel_z=9.81,
            timestamp=time.monotonic()
        )

        if self.use_i2c:
            self._init_i2c()

    def _init_i2c(self):
        try:
            import smbus2
            self.i2c = smbus2.SMBus(1)
            self.i2c.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            time.sleep(0.1)
            chip_id = self.i2c.read_byte_data(self.address, self.WHO_AM_I)
            print(f"✅ MPU6050 détecté (ID: 0x{chip_id:02x})")
            self._calibrate_gyro()
        except ImportError:
            print("⚠️  smbus2 non installé. Mode simulation.")
            self.use_i2c = False
        except Exception as e:
            print(f"⚠️  IMU I2C erreur: {e}. Mode simulation.")
            self.use_i2c = False

    def _calibrate_gyro(self, samples: int = 200):
        print("🔧 Calibration gyro (restez immobile)...")
        offsets = {'x': [], 'y': [], 'z': []}
        for _ in range(samples):
            data = self._read_burst_i2c()
            offsets['x'].append(data['gx'])
            offsets['y'].append(data['gy'])
            offsets['z'].append(data['gz'])
            time.sleep(0.01)
        self.gyro_offset_x = np.median(offsets['x'])
        self.gyro_offset_y = np.median(offsets['y'])
        self.gyro_offset_z = np.median(offsets['z'])
        self._calibrated = True
        print(f"   Offsets: gx={self.gyro_offset_x:.4f}, gy={self.gyro_offset_y:.4f}, gz={self.gyro_offset_z:.4f}")

    def _read_burst_i2c(self) -> dict:
        if not self.i2c:
            return self._simulate_imu()
        try:
            data = self.i2c.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)

            def to_signed(high, low):
                val = (high << 8) | low
                return val - 65536 if val > 32767 else val

            ax = to_signed(data[0], data[1]) / self.ACCEL_SCALE * 9.81
            ay = to_signed(data[2], data[3]) / self.ACCEL_SCALE * 9.81
            az = to_signed(data[4], data[5]) / self.ACCEL_SCALE * 9.81
            gx = (to_signed(data[8], data[9]) / self.GYRO_SCALE) * np.pi / 180
            gy = (to_signed(data[10], data[11]) / self.GYRO_SCALE) * np.pi / 180
            gz = (to_signed(data[12], data[13]) / self.GYRO_SCALE) * np.pi / 180

            if self._calibrated:
                gx -= self.gyro_offset_x
                gy -= self.gyro_offset_y
                gz -= self.gyro_offset_z

            return {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        except Exception as e:
            print(f"❌ Erreur burst I2C: {e}")
            return self._simulate_imu()

    def _simulate_imu(self) -> dict:
        return {
            'ax': np.random.normal(0, 0.05),
            'ay': np.random.normal(0, 0.05),
            'az': 9.81 + np.random.normal(0, 0.1),
            'gx': np.random.normal(0, 0.01),
            'gy': np.random.normal(0, 0.01),
            'gz': 0.005 + np.random.normal(0, 0.01),
        }

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, '_thread'):
            self._thread.join(timeout=1.0)

    def _read_loop(self):
        period = 1.0 / self.UPDATE_RATE_HZ
        while self.running:
            loop_start = time.monotonic()
            imu_data = self._read_burst_i2c()
            gyro = (imu_data['gx'], imu_data['gy'], imu_data['gz'])
            accel = (imu_data['ax'], imu_data['ay'], imu_data['az'])
            roll, pitch, yaw = self.complementary_filter.update(gyro, accel)
            with self._lock:
                self._last_reading = IMUReading(
                    roll=roll, pitch=pitch, yaw=yaw,
                    angular_vel_x=gyro[0], angular_vel_y=gyro[1], angular_vel_z=gyro[2],
                    accel_x=accel[0], accel_y=accel[1], accel_z=accel[2],
                    timestamp=time.monotonic()
                )
            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_reading(self) -> IMUReading:
        with self._lock:
            return self._last_reading

    def get_heading(self) -> float:
        with self._lock:
            yaw = self._last_reading.yaw
        return yaw % (2 * np.pi)

    def get_yaw_rate(self) -> float:
        with self._lock:
            return self._last_reading.angular_vel_z

    def reset_yaw(self, new_yaw: float = 0.0):
        with self._lock:
            self.complementary_filter.yaw = new_yaw


if __name__ == "__main__":
    print("=== Test IMU Optimisé ===\n")
    imu = IMUSensor(use_i2c=False)
    imu.start()
    for i in range(20):
        r = imu.get_reading()
        print(f"[{i+1:02d}] yaw={np.degrees(r.yaw):6.2f}°  "
              f"rate={r.angular_vel_z:7.4f} rad/s  "
              f"drift_est={np.degrees(imu.complementary_filter.yaw_drift_rate):.4f}°/s")
        time.sleep(0.1)
    imu.stop()
    print("\n✅ Test complété")

