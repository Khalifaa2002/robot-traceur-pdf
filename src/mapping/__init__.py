"""
Mapping Module - Occupancy Grid Mapping (OPTIMISÉ)
Grille réduite pour Raspberry Pi, mises à jour throttlées
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class GridCell:
    """Cellule de la grille"""
    occupancy: float
    log_odds: float
    visits: int


class OccupancyGrid:
    """
    Grille d'occupation 2D optimisée pour Raspberry Pi
    - Résolution 0.2m (au lieu de 0.1m) = 4× moins de cellules
    - Mises à jour throttlées (min distance/rotation entre updates)
    - Log-odds clampés pour éviter overflow
    """

    def __init__(self, width: float = 4.0, height: float = 4.0,
                 resolution: float = 0.2):
        self.width = width
        self.height = height
        self.resolution = resolution

        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)

        self.log_odds = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        self.visits = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        self.log_occ = np.log(0.7 / 0.3)
        self.log_free = np.log(0.3 / 0.7)
        self.max_log_odds = 2.0
        self.min_log_odds = -2.0

        # Throttling
        self._last_update_x = 0.0
        self._last_update_y = 0.0
        self._last_update_theta = 0.0
        self._min_update_dist = 0.15  # m
        self._min_update_angle = np.radians(15)  # rad

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        grid_x = int((x + self.width / 2) / self.resolution)
        grid_y = int((y + self.height / 2) / self.resolution)
        grid_x = max(0, min(grid_x, self.grid_width - 1))
        grid_y = max(0, min(grid_y, self.grid_height - 1))
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        x = (grid_x + 0.5) * self.resolution - self.width / 2
        y = (grid_y + 0.5) * self.resolution - self.height / 2
        return (x, y)

    def should_update(self, x: float, y: float, theta: float) -> bool:
        """Vérifie si le robot a suffisamment bougé pour mettre à jour"""
        dx = x - self._last_update_x
        dy = y - self._last_update_y
        dtheta = abs(theta - self._last_update_theta)
        dist = np.sqrt(dx**2 + dy**2)
        if dist >= self._min_update_dist or dtheta >= self._min_update_angle:
            self._last_update_x = x
            self._last_update_y = y
            self._last_update_theta = theta
            return True
        return False

    def update_ray(self, start_x: float, start_y: float,
                   end_x: float, end_y: float, occupied: bool = True):
        start_grid = self.world_to_grid(start_x, start_y)
        end_grid = self.world_to_grid(end_x, end_y)
        points = self._bresenham_line(start_grid[0], start_grid[1],
                                      end_grid[0], end_grid[1])

        if occupied and len(points) > 0:
            for px, py in points[:-1]:
                self._update_cell(px, py, self.log_free)
            px, py = points[-1]
            self._update_cell(px, py, self.log_occ)
        else:
            for px, py in points:
                self._update_cell(px, py, self.log_free)

    def _update_cell(self, grid_x: int, grid_y: int, log_odds_increment: float):
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            self.log_odds[grid_y, grid_x] += log_odds_increment
            self.log_odds[grid_y, grid_x] = np.clip(
                self.log_odds[grid_y, grid_x],
                self.min_log_odds,
                self.max_log_odds
            )
            self.visits[grid_y, grid_x] += 1

    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> list:
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return points

    def get_occupancy_map(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def get_obstacle_grid(self, threshold: float = 0.5) -> np.ndarray:
        occupancy = self.get_occupancy_map()
        return (occupancy > threshold).astype(np.uint8)

    def is_occupied(self, x: float, y: float, threshold: float = 0.5) -> bool:
        grid_x, grid_y = self.world_to_grid(x, y)
        occupancy = 1.0 / (1.0 + np.exp(-self.log_odds[grid_y, grid_x]))
        return occupancy > threshold

    def get_free_cells(self) -> list:
        occupancy = self.get_occupancy_map()
        free_indices = np.argwhere(occupancy < 0.3)
        return [(int(row), int(col)) for row, col in free_indices]


if __name__ == "__main__":
    print("=== Test Occupancy Grid Optimisé ===\n")
    grid = OccupancyGrid(width=4.0, height=4.0, resolution=0.2)
    print(f"Grille: {grid.grid_width}x{grid.grid_height} = {grid.grid_width * grid.grid_height} cellules")
    grid.update_ray(0, 0, 0.5, 0, occupied=True)
    grid.update_ray(0, 0, 0, 0.3, occupied=False)
    print(f"Occupé à (0.5, 0): {grid.is_occupied(0.5, 0)}")
    print(f"Libre à (0, 0.2): {grid.is_occupied(0, 0.2)}")
    print("\n✅ Test complété")

