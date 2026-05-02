"""
Control Module - Contrôle moteurs et boucle temps réel (OPTIMISÉ)
- Real-time enforcement dur
- Velocity ramping
- Emergency stop intégré
- PWM L298N correct (signe géré)
"""

import time
import threading
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class MotorCommand:
    left_pwm: float
    right_pwm: float
    left_dir: int
    right_dir: int


class MotorController:
    """Contrôle moteurs L298N via GPIO ou Serial"""

    def __init__(self, use_gpio: bool = False, serial_port: str = "COM3"):
        self.use_gpio = use_gpio
        self.serial_port = serial_port
        self.gpio = None
        self.serial = None

        self.LEFT_PWM = 18
        self.LEFT_DIR = 27
        self.RIGHT_PWM = 12
        self.RIGHT_DIR = 17

        self._emergency_stop = False

        if self.use_gpio:
            self._init_gpio()
        else:
            self._init_serial()

    def _init_gpio(self):
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
            self.gpio.setmode(GPIO.BCM)
            for pin in [self.LEFT_PWM, self.LEFT_DIR, self.RIGHT_PWM, self.RIGHT_DIR]:
                self.gpio.setup(pin, GPIO.OUT)
            self.pwm_left = self.gpio.PWM(self.LEFT_PWM, 1000)
            self.pwm_right = self.gpio.PWM(self.RIGHT_PWM, 1000)
            self.pwm_left.start(0)
            self.pwm_right.start(0)
        except Exception as e:
            print(f"⚠️ GPIO init failed: {e}")
            self.use_gpio = False

    def _init_serial(self):
        try:
            import serial
            self.serial = serial.Serial(port=self.serial_port, baudrate=115200, timeout=1.0)
        except Exception as e:
            self.serial = None

    def set_motor_pwm(self, left_speed: float, right_speed: float):
        """
        Commande moteur en [-1.0, 1.0].
        Signe négatif = marche arrière (L298N direction pin)
        """
        if self._emergency_stop:
            left_speed = 0.0
            right_speed = 0.0

        left_speed = np.clip(left_speed, -1.0, 1.0)
        right_speed = np.clip(right_speed, -1.0, 1.0)

        left_pwm = int(abs(left_speed) * 255)
        right_pwm = int(abs(right_speed) * 255)
        left_dir = 1 if left_speed >= 0 else -1
        right_dir = 1 if right_speed >= 0 else -1

        if self.use_gpio and self.gpio:
            self.gpio.output(self.LEFT_DIR, left_dir > 0)
            self.gpio.output(self.RIGHT_DIR, right_dir > 0)
            self.pwm_left.ChangeDutyCycle(left_pwm / 255.0 * 100)
            self.pwm_right.ChangeDutyCycle(right_pwm / 255.0 * 100)
        elif self.serial and self.serial.is_open:
            cmd = f"L{int(left_speed * 255)} R{int(right_speed * 255)}\n"
            try:
                self.serial.write(cmd.encode())
            except Exception:
                pass

    def emergency_stop(self):
        """STOP immédiat"""
        self._emergency_stop = True
        self.set_motor_pwm(0, 0)

    def reset_emergency(self):
        self._emergency_stop = False

    def stop(self):
        self.set_motor_pwm(0, 0)
        if self.use_gpio and self.gpio:
            self.pwm_left.stop()
            self.pwm_right.stop()
            self.gpio.cleanup()
        if self.serial and self.serial.is_open:
            self.serial.close()


class PIDController:
    """PID avec anti-windup"""

    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.2,
                 integral_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.error_integral = 0.0
        self.error_last = 0.0
        self.last_time = time.monotonic()

    def compute(self, error: float) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            dt = 0.01

        self.error_integral += error * dt
        self.error_integral = np.clip(self.error_integral, -self.integral_max, self.integral_max)

        derivative = (error - self.error_last) / dt
        self.error_last = error

        return self.kp * error + self.ki * self.error_integral + self.kd * derivative

    def reset(self):
        self.error_integral = 0.0
        self.error_last = 0.0
        self.last_time = time.monotonic()


class VelocityRamp:
    """Ramping pour protéger moteurs et réduire glissement"""

    def __init__(self, max_accel: float = 0.5, max_decel: float = 1.0):
        self.max_accel = max_accel  # m/s²
        self.max_decel = max_decel
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_time = time.monotonic()

    def limit(self, target_v: float, target_w: float) -> Tuple[float, float]:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            dt = 0.01

        # Ramping linéaire
        dv = target_v - self.current_v
        max_dv = self.max_accel * dt if dv > 0 else self.max_decel * dt
        dv = np.clip(dv, -max_dv, max_dv)
        self.current_v += dv

        # Ramping angulaire (plus permissif)
        dw = target_w - self.current_w
        max_dw = 2.0 * dt
        dw = np.clip(dw, -max_dw, max_dw)
        self.current_w += dw

        return (self.current_v, self.current_w)


class ControlLoop:
    """Boucle contrôle temps réel optimisée pour Raspberry Pi"""

    TARGET_FREQUENCY = 10  # Hz
    LOOP_TIME = 1.0 / TARGET_FREQUENCY

    def __init__(self, motor_controller: MotorController,
                 sensor_fusion, localizer, planner, occupancy_grid,
                 obstacle_avoidance, max_linear_v: float = 0.5,
                 max_angular_w: float = 1.0):
        self.motor = motor_controller
        self.sensor_fusion = sensor_fusion
        self.localizer = localizer
        self.planner = planner
        self.grid = occupancy_grid
        self.obstacle_avoidance = obstacle_avoidance

        self.max_linear_v = max_linear_v
        self.max_angular_w = max_angular_w

        self.global_path = None
        self.current_waypoint_idx = 0

        self.pid_linear = PIDController(kp=0.8, ki=0.05, kd=0.1)
        self.pid_angular = PIDController(kp=1.2, ki=0.08, kd=0.15)
        self.velocity_ramp = VelocityRamp(max_accel=0.5, max_decel=1.0)

        self.running = False
        self.loop_stats = {
            'iterations': 0,
            'deadline_misses': 0,
            'min_loop_time': float('inf'),
            'max_loop_time': 0.0,
            'avg_loop_time': 0.0,
            'emergency_stops': 0,
        }
        self._stats_lock = threading.Lock()

    def set_global_path(self, path: list):
        self.global_path = path
        self.current_waypoint_idx = 0

    def start(self):
        self.running = True
        self._start_time = time.monotonic()
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        self.motor.stop()

    def _control_loop(self):
        while self.running:
            iteration_start = time.monotonic()

            try:
                sensor_read = self.sensor_fusion.get_reading()
                self.localizer.update_heading(sensor_read.imu_data.yaw)
                state = self.localizer.get_state()

                # Mapping throttlé
                if self.grid.should_update(state.x, state.y, state.theta):
                    self._update_grid(sensor_read)

                # Commandes de base
                v_cmd, w_cmd = self._compute_motor_commands(state)

                # Évitement d'obstacles
                v_avoid, w_avoid, avoiding = self.obstacle_avoidance.compute_avoidance(
                    sensor_read.distances, v_cmd, w_cmd, state.theta
                )

                # Emergency stop si évitement très proche
                if avoiding and sensor_read.distances.get('front', 4.0) < 0.15:
                    self.motor.emergency_stop()
                    with self._stats_lock:
                        self.loop_stats['emergency_stops'] += 1
                else:
                    self.motor.reset_emergency()

                # Ramping + saturation
                v_ramped, w_ramped = self.velocity_ramp.limit(v_avoid, w_avoid)
                v_ramped = np.clip(v_ramped, -self.max_linear_v, self.max_linear_v)
                w_ramped = np.clip(w_ramped, -self.max_angular_w, self.max_angular_w)

                # Diff drive → moteurs
                self._send_motor_commands(v_ramped, w_ramped)

            except Exception as e:
                print(f"❌ Erreur boucle: {e}")
                self.motor.emergency_stop()

            # Real-time enforcement
            iteration_time = time.monotonic() - iteration_start
            sleep_time = self.LOOP_TIME - iteration_time

            with self._stats_lock:
                self.loop_stats['iterations'] += 1
                self.loop_stats['min_loop_time'] = min(self.loop_stats['min_loop_time'], iteration_time)
                self.loop_stats['max_loop_time'] = max(self.loop_stats['max_loop_time'], iteration_time)
                if sleep_time < 0:
                    self.loop_stats['deadline_misses'] += 1
                else:
                    time.sleep(sleep_time)

                # MAJ moyenne mobile
                n = self.loop_stats['iterations']
                self.loop_stats['avg_loop_time'] = (
                    (n - 1) * self.loop_stats['avg_loop_time'] + iteration_time
                ) / n

    def _update_grid(self, sensor_read):
        x, y = sensor_read.position_x, sensor_read.position_y
        theta = sensor_read.heading
        for direction, dist in sensor_read.distances.items():
            if direction == 'front':
                ox = x + dist * np.cos(theta)
                oy = y + dist * np.sin(theta)
            elif direction == 'back':
                ox = x - dist * np.cos(theta)
                oy = y - dist * np.sin(theta)
            elif direction == 'left':
                ox = x + dist * np.cos(theta + np.pi / 2)
                oy = y + dist * np.sin(theta + np.pi / 2)
            elif direction == 'right':
                ox = x + dist * np.cos(theta - np.pi / 2)
                oy = y + dist * np.sin(theta - np.pi / 2)
            else:
                continue
            self.grid.update_ray(x, y, ox, oy, occupied=(dist < 2.0))

    def _compute_motor_commands(self, state) -> Tuple[float, float]:
        if not self.global_path:
            return (0.0, 0.0)

        if self.current_waypoint_idx >= len(self.global_path):
            return (0.0, 0.0)

        waypoint = self.global_path[self.current_waypoint_idx]
        dx = waypoint[0] - state.x
        dy = waypoint[1] - state.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < 0.1:
            self.current_waypoint_idx += 1
            self.pid_linear.reset()
            self.pid_angular.reset()
            return (0.0, 0.0)

        angle_target = np.arctan2(dy, dx)
        angle_error = np.arctan2(np.sin(angle_target - state.theta), np.cos(angle_target - state.theta))

        v = self.pid_linear.compute(dist)
        w = self.pid_angular.compute(angle_error)

        return (v, w)

    def _send_motor_commands(self, v: float, w: float):
        wheel_base = 0.15
        v_left = v - w * wheel_base / 2
        v_right = v + w * wheel_base / 2

        # Normalise si saturation
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > self.max_linear_v:
            scale = self.max_linear_v / max_v
            v_left *= scale
            v_right *= scale

        self.motor.set_motor_pwm(v_left, v_right)

    def get_stats(self) -> dict:
        with self._stats_lock:
            return self.loop_stats.copy()


if __name__ == "__main__":
    print("=== Test Control Optimisé ===\n")
    motor = MotorController(use_gpio=False)
    pid = PIDController(kp=1.0, ki=0.1, kd=0.2)
    ramp = VelocityRamp()

    print("Test PID:")
    for e in [0.5, 0.3, 0.1, 0.0]:
        print(f"  error={e:.1f} → cmd={pid.compute(e):.3f}")

    print("\nTest VelocityRamp:")
    for target in [0.0, 0.5, 0.5, 0.0, -0.3]:
        v, w = ramp.limit(target, 0.0)
        print(f"  target={target:.1f} → v={v:.2f}")

    motor.stop()
    print("\n✅ Test complété")

