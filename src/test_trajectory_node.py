"""
test_trajectory_node.py
Test du nœud ROS 2 sans avoir besoin de ROS 2 réel
"""

import numpy as np
import os
from matplotlib import pyplot as plt

def load_trajectory():
    """Charge la trajectoire"""
    trajectory_file = os.path.join(
        os.path.dirname(__file__),
        '../data/trajectory.npy'
    )
    
    if os.path.exists(trajectory_file):
        trajectory = np.load(trajectory_file)
        print(f"✅ Trajectoire chargée: {len(trajectory)} points")
        return trajectory
    else:
        print(f"❌ Fichier non trouvé: {trajectory_file}")
        return None

def visualize_path_with_poses(trajectory):
    """Visualise le chemin avec les poses (orientation)"""
    plt.figure(figsize=(12, 10))
    
    # Extrait x, y, theta
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    theta = trajectory[:, 2]
    
    # Affiche le chemin
    plt.plot(x, y, 'b-', linewidth=2, label='Trajectoire')
    plt.plot(x, y, 'bo', markersize=4, alpha=0.5)
    
    # Affiche les orientations tous les 10 points
    step = max(1, len(trajectory) // 15)
    for i in range(0, len(trajectory), step):
        xi, yi, ti = x[i], y[i], theta[i]
        
        # Dessine un vecteur d'orientation
        scale = 0.05
        dx = scale * np.cos(ti)
        dy = scale * np.sin(ti)
        plt.arrow(xi, yi, dx, dy, head_width=0.01, head_length=0.01, 
                 fc='red', ec='red', alpha=0.7)
    
    # Marque le début et la fin
    plt.plot(x[0], y[0], 'go', markersize=12, label='Début', zorder=5)
    plt.plot(x[-1], y[-1], 'r*', markersize=15, label='Fin', zorder=5)
    
    plt.title("Trajectoire avec Orientations (ROS 2 Path)")
    plt.xlabel("X (mètres)")
    plt.ylabel("Y (mètres)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

def print_trajectory_stats(trajectory):
    """Affiche les statistiques de la trajectoire"""
    print("\n📊 Statistiques de la trajectoire:")
    print("=" * 50)
    print(f"  Nombre de points: {len(trajectory)}")
    print(f"  X: [{trajectory[:, 0].min():.3f}, {trajectory[:, 0].max():.3f}] mètres")
    print(f"  Y: [{trajectory[:, 1].min():.3f}, {trajectory[:, 1].max():.3f}] mètres")
    print(f"  Theta: [{trajectory[:, 2].min():.3f}, {trajectory[:, 2].max():.3f}] radians")
    
    # Calcule la distance totale
    distances = np.sqrt(np.sum(np.diff(trajectory[:, :2], axis=0)**2, axis=1))
    total_distance = np.sum(distances)
    print(f"  Distance totale: {total_distance:.3f} mètres")
    print("=" * 50)

if __name__ == "__main__":
    print("🤖 Test de la Trajectoire ROS 2")
    print("=" * 50)
    
    # Charge la trajectoire
    trajectory = load_trajectory()
    
    if trajectory is not None:
        # Affiche les stats
        print_trajectory_stats(trajectory)
        
        # Affiche les premiers points
        print("\n📋 Premiers points de la trajectoire:")
        for i in range(min(5, len(trajectory))):
            print(f"  Point {i}: x={trajectory[i, 0]:.4f}, y={trajectory[i, 1]:.4f}, theta={trajectory[i, 2]:.4f}")
        
        print("  ...")
        
        # Affiche les derniers points
        print("\n📋 Derniers points de la trajectoire:")
        for i in range(max(0, len(trajectory) - 5), len(trajectory)):
            print(f"  Point {i}: x={trajectory[i, 0]:.4f}, y={trajectory[i, 1]:.4f}, theta={trajectory[i, 2]:.4f}")
        
        # Visualise
        print("\n📊 Génération de la visualisation...")
        visualize_path_with_poses(trajectory)
    else:
        print("❌ Impossible de charger la trajectoire")