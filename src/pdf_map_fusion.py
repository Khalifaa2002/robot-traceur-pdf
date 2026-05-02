"""
PDF Map Fusion - Intègre trajectoire PDF dans la carte d'occupation
Marque le corridor PDF comme "espace préféré" pour éviter
que le robot ne dévie à cause de faux obstacles (sol irrégulier, etc.)
"""

import numpy as np
from typing import List, Tuple


class PDFMapFusion:
    """
    Fusionne trajectoire PDF avec carte réelle.
    Le corridor PDF est marqué comme libre (log-odds négatif)
    pour guider le robot même si ultrasons créent des faux obstacles.
    """

    def __init__(self, corridor_width: float = 0.25):
        """
        Args:
            corridor_width: Largeur du corridor autour de la trajectoire (m)
        """
        self.corridor_width = corridor_width

    def integrate_path(self, grid, path: List[Tuple[float, float]]):
        """
        Marque le corridor autour du chemin PDF comme espace libre préféré.
        Réduit log-odds pour ces cellules (encourage le robot à suivre le PDF).
        """
        if not path or len(path) == 0:
            return

        log_free_preferred = np.log(0.2 / 0.8)  # Forte croyance "libre"

        for px, py in path:
            # Marque cellule et voisins dans corridor
            grid_x, grid_y = grid.world_to_grid(px, py)
            radius_cells = int(self.corridor_width / grid.resolution)

            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    cx = grid_x + dx
                    cy = grid_y + dy
                    if 0 <= cx < grid.grid_width and 0 <= cy < grid.grid_height:
                        # Distance euclidienne depuis le centre du corridor
                        wx, wy = grid.grid_to_world(cx, cy)
                        dist = np.hypot(wx - px, wy - py)
                        if dist <= self.corridor_width:
                            # Plus proche du centre = plus forte croyance libre
                            weight = 1.0 - (dist / self.corridor_width)
                            grid.log_odds[cy, cx] += weight * log_free_preferred
                            grid.log_odds[cy, cx] = np.clip(
                                grid.log_odds[cy, cx], grid.min_log_odds, grid.max_log_odds
                            )

    def get_path_deviation(self, path: List[Tuple[float, float]],
                           current_x: float, current_y: float) -> float:
        """Retourne distance du robot au chemin PDF le plus proche"""
        if not path:
            return float('inf')
        distances = [np.hypot(p[0] - current_x, p[1] - current_y) for p in path]
        return min(distances)


if __name__ == "__main__":
    print("=== Test PDFMapFusion ===\n")
    from mapping import OccupancyGrid

    grid = OccupancyGrid(width=4.0, height=4.0, resolution=0.2)
    fusion = PDFMapFusion(corridor_width=0.3)

    path = [(0, 0), (0.2, 0), (0.4, 0), (0.6, 0), (0.8, 0), (1.0, 0)]
    fusion.integrate_path(grid, path)

    print(f"Déviation au chemin: {fusion.get_path_deviation(path, 0.1, 0.05):.2f}m")
    print(f"Occupancy centre corridor: {grid.get_occupancy_map()[10, 10]:.2f}")
    print("\n✅ Test complété")

