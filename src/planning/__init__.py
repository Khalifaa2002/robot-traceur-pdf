"""
Planning Module - Pathfinding et Obstacle Avoidance (OPTIMISÉ)
- A* avec reconstruction correcte du chemin
- Replanification conditionnelle (throttle 2s)
- SimpleObstacleAvoidance remplace DWA (trop lourd pour RPi)
"""

import time
import numpy as np
from typing import List, Tuple, Optional, Set, Dict
from dataclasses import dataclass
import heapq


@dataclass
class PathNode:
    """Nœud A*"""
    x: int
    y: int
    g: float = float('inf')
    h: float = 0.0
    parent: Optional['PathNode'] = None

    def f(self) -> float:
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()


class AStarPlanner:
    """A* avec reconstruction correcte du chemin"""

    MOVEMENTS = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]
    MOVEMENT_COSTS = [1, 1, 1, 1, np.sqrt(2), np.sqrt(2), np.sqrt(2), np.sqrt(2)]

    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.grid_width = occupancy_grid.grid_width
        self.grid_height = occupancy_grid.grid_height
        self.last_path = []

    def heuristic(self, gx: int, gy: int, goal_x: int, goal_y: int) -> float:
        dx = abs(goal_x - gx)
        dy = abs(goal_y - gy)
        return (dx + dy) + (np.sqrt(2) - 2) * min(dx, dy)

    def is_walkable(self, gx: int, gy: int, threshold: float = 0.55) -> bool:
        """Cellule franchissable si occupancy < threshold (inconnu=0.5 = libre)"""
        if not (0 <= gx < self.grid_width and 0 <= gy < self.grid_height):
            return False
        occupancy = 1.0 / (1.0 + np.exp(-self.grid.log_odds[gy, gx]))
        return occupancy < threshold

    def plan(self, start_x: float, start_y: float,
             goal_x: float, goal_y: float,
             max_iterations: int = 5000) -> Optional[List[Tuple[float, float]]]:
        """A* avec reconstruction correcte via dictionnaire parent"""
        start_g = self.grid.world_to_grid(start_x, start_y)
        goal_g = self.grid.world_to_grid(goal_x, goal_y)

        if not self.is_walkable(goal_g[0], goal_g[1]):
            return None

        open_set = []
        closed_set: Set[Tuple[int, int]] = set()
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {}

        start_key = start_g
        g_score[start_key] = 0.0
        start_h = self.heuristic(start_g[0], start_g[1], goal_g[0], goal_g[1])
        heapq.heappush(open_set, (start_h, id(start_key), start_key))

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_set)
            cx, cy = current

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal_g:
                # Reconstruction correcte
                path = []
                node = current
                while node is not None:
                    wx, wy = self.grid.grid_to_world(node[0], node[1])
                    path.append((wx, wy))
                    node = came_from.get(node)
                self.last_path = list(reversed(path))
                return self.last_path

            for idx, (dx, dy) in enumerate(self.MOVEMENTS):
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)

                if neighbor in closed_set or not self.is_walkable(nx, ny):
                    continue

                tentative_g = g_score[current] + self.MOVEMENT_COSTS[idx]

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = self.heuristic(nx, ny, goal_g[0], goal_g[1])
                    heapq.heappush(open_set, (tentative_g + h, id(neighbor), neighbor))

        return None

    def get_last_path(self) -> List[Tuple[float, float]]:
        return self.last_path


class SimpleObstacleAvoidance:
    """
    Évitement d'obstacles simple et robuste pour Raspberry Pi.
    Remplace DWA (trop CPU-intensive: 25 trajectoires × Python loops)
    Règles:
      - Front < 0.15m: STOP d'urgence
      - Front < 0.35m: Ralentir + tourner vers côté libre
      - Side < 0.20m: Bias latéral opposé
      - Sinon: Suivre trajectoire PDF avec pure pursuit
    """

    EMERGENCY_DIST = 0.15
    SLOW_DIST = 0.35
    SIDE_DIST = 0.20
    ROTATION_SPEED = 0.8  # rad/s

    def __init__(self):
        self.last_avoidance_time = 0.0
        self.avoidance_cooldown = 0.5  # s

    def compute_avoidance(self, distances: dict, target_v: float, target_w: float,
                          current_heading: float) -> Tuple[float, float, bool]:
        """
        Retourne (v, w, is_avoiding)
        """
        now = time.monotonic()
        front = distances.get('front', 4.0)
        left = distances.get('left', 4.0)
        right = distances.get('right', 4.0)
        back = distances.get('back', 4.0)

        # 1. STOP d'urgence
        if front < self.EMERGENCY_DIST:
            return (0.0, 0.0, True)

        # 2. Obstacle proche devant
        if front < self.SLOW_DIST:
            v = target_v * 0.3
            # Tourne vers côté le plus libre
            if left > right:
                w = self.ROTATION_SPEED
            else:
                w = -self.ROTATION_SPEED
            self.last_avoidance_time = now
            return (v, w, True)

        # 3. Obstacle latéral
        if left < self.SIDE_DIST:
            # Bias à droite
            target_w -= 0.3
        if right < self.SIDE_DIST:
            # Bias à gauche
            target_w += 0.3
        if back < self.EMERGENCY_DIST:
            target_v = max(0.05, target_v)

        return (target_v, target_w, False)


class ConditionalReplanning:
    """Décide quand replanifier pour économiser CPU"""

    def __init__(self, planner: AStarPlanner):
        self.planner = planner
        self.last_replan_time = 0.0
        self.min_replan_interval = 2.0  # s
        self.deviation_threshold = 0.25  # m
        self.blocked_threshold = 0.6

    def should_replan(self, state: 'RobotState', global_path: list,
                      grid) -> bool:
        now = time.monotonic()
        if now - self.last_replan_time < self.min_replan_interval:
            return False

        # Vérifie si chemin bloqué
        if global_path and len(global_path) > 0:
            for i in range(0, min(10, len(global_path)), 2):
                px, py = global_path[i]
                if grid.is_occupied(px, py, self.blocked_threshold):
                    self.last_replan_time = now
                    return True

        # Vérifie déviation importante
        if global_path and len(global_path) > 0:
            nearest = min(global_path, key=lambda p: np.hypot(p[0] - state.x, p[1] - state.y))
            dist = np.hypot(nearest[0] - state.x, nearest[1] - state.y)
            if dist > self.deviation_threshold:
                self.last_replan_time = now
                return True

        return False


if __name__ == "__main__":
    print("=== Test Planning Optimisé ===\n")
    from mapping import OccupancyGrid

    grid = OccupancyGrid(width=4.0, height=4.0, resolution=0.2)
    planner = AStarPlanner(grid)

    path = planner.plan(0, 0, 2, 2)
    if path:
        print(f"A* OK: {len(path)} points")
    else:
        print("A* échec")

    avoid = SimpleObstacleAvoidance()
    v, w, flag = avoid.compute_avoidance(
        {'front': 0.25, 'left': 1.0, 'right': 1.0, 'back': 2.0},
        0.3, 0.0, 0.0
    )
    print(f"Avoidance: v={v:.2f}, w={w:.2f}, active={flag}")

    print("\n✅ Test complété")

