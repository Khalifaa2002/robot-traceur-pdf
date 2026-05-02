"""
Real-Time Monitoring - Logs CPU, température, timing, santé capteurs
Sortie console + CSV pour analyse post-mortem
"""

import time
import os
import threading
import csv
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional


class RealTimeMonitor:
    """Moniteur temps réel pour Raspberry Pi"""

    def __init__(self, log_interval: float = 5.0, csv_path: Optional[str] = None):
        self.log_interval = log_interval
        self.running = False
        self._data = {
            'cpu_percent': 0.0,
            'cpu_temp': 0.0,
            'memory_mb': 0.0,
            'loop_jitter_ms': 0.0,
            'deadline_misses': 0,
            'sensor_health': True,
        }
        self._lock = threading.Lock()

        if csv_path is None:
            csv_path = f"logs/robot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_path = Path(csv_path)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

        self._csv_file = None
        self._csv_writer = None
        self._start_time = time.monotonic()

    def _read_cpu_temp(self) -> float:
        """Lit température CPU Raspberry Pi"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def _read_cpu_percent(self) -> float:
        """CPU usage (simple)"""
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
            fields = list(map(int, line.split()[1:]))
            idle = fields[3]
            total = sum(fields)
            return 100.0 * (1 - idle / total) if total > 0 else 0.0
        except Exception:
            return 0.0

    def _read_memory(self) -> float:
        """Memory usage en MB"""
        try:
            import psutil
            return psutil.Process().memory_info().rss / (1024 * 1024)
        except Exception:
            return 0.0

    def update(self, control_stats: dict, sensor_health: bool = True):
        """Appelé depuis la boucle principale pour mettre à jour les métriques"""
        with self._lock:
            self._data['loop_jitter_ms'] = control_stats.get('avg_loop_time', 0) * 1000
            self._data['deadline_misses'] = control_stats.get('deadline_misses', 0)
            self._data['sensor_health'] = sensor_health

    def start(self):
        self.running = True
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'elapsed_s', 'cpu_pct', 'cpu_temp_c',
            'mem_mb', 'loop_jitter_ms', 'deadline_misses', 'sensor_ok'
        ])
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, '_thread'):
            self._thread.join(timeout=1.0)
        if self._csv_file:
            self._csv_file.close()

    def _monitor_loop(self):
        while self.running:
            loop_start = time.monotonic()

            cpu = self._read_cpu_percent()
            temp = self._read_cpu_temp()
            mem = self._read_memory()

            with self._lock:
                data = self._data.copy()

            elapsed = time.monotonic() - self._start_time
            row = [
                datetime.now().isoformat(),
                f"{elapsed:.1f}",
                f"{cpu:.1f}",
                f"{temp:.1f}",
                f"{mem:.1f}",
                f"{data['loop_jitter_ms']:.1f}",
                data['deadline_misses'],
                "OK" if data['sensor_health'] else "FAIL"
            ]

            if self._csv_writer:
                self._csv_writer.writerow(row)
                self._csv_file.flush()

            # Affichage console
            print(f"\r[MON] CPU:{cpu:5.1f}% TEMP:{temp:4.1f}°C MEM:{mem:6.1f}MB "
                  f"JITTER:{data['loop_jitter_ms']:5.1f}ms MISSES:{data['deadline_misses']:3d} "
                  f"SENSORS:{('OK' if data['sensor_health'] else 'FAIL')}", end="", flush=True)

            sleep_time = self.log_interval - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    print("=== Test RealTimeMonitor ===\n")
    mon = RealTimeMonitor(log_interval=2.0)
    mon.start()
    time.sleep(6)
    mon.stop()
    print(f"\n\n✅ Log sauvegardé: {mon.csv_path}")

