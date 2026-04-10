"""
trajectory_generator.py
Génère une trajectoire à partir des points extraits du PDF
"""

import numpy as np
from matplotlib import pyplot as plt
import matplotlib
import sys
import os
from pathlib import Path

# Headless check for Raspberry Pi (RPI-4)
if os.environ.get('DISPLAY') is None:
    matplotlib.use('Agg')

# Ajoute le parent au path pour accéder à PythonRobotics
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

def pixel_to_world(points, pixel_to_meter=0.001, origin=(0, 0)):
    """
    Convertit les coordonnées pixels en coordonnées du monde réel (mètres)
    """
    # Normalise par rapport au min
    min_x = points[:, 0].min()
    min_y = points[:, 1].min()
    
    # Convertit en coordonnées de monde
    world_points = points.copy()
    world_points[:, 0] = (points[:, 0] - min_x) * pixel_to_meter + origin[0]
    world_points[:, 1] = (points[:, 1] - min_y) * pixel_to_meter + origin[1]
    
    return world_points

def smooth_trajectory(points, num_points=100):
    """Lisse la trajectoire en interpolant les points"""
    # Paramétrise par distance cumulative
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    cumulative_distance = np.concatenate(([0], np.cumsum(distances)))
    
    # Crée des nouvelles distances uniformément espacées
    total_distance = cumulative_distance[-1]
    new_distances = np.linspace(0, total_distance, num_points)
    
    # Interpole x et y
    smooth_x = np.interp(new_distances, cumulative_distance, points[:, 0])
    smooth_y = np.interp(new_distances, cumulative_distance, points[:, 1])
    
    smooth_points = np.column_stack((smooth_x, smooth_y))
    
    return smooth_points

def add_orientation(points):
    """Ajoute l'orientation (angle) à chaque point de la trajectoire"""
    points_with_angle = np.zeros((len(points), 3))
    points_with_angle[:, :2] = points
    
    # Calcule l'angle de chaque segment
    for i in range(len(points) - 1):
        dx = points[i+1, 0] - points[i, 0]
        dy = points[i+1, 1] - points[i, 1]
        angle = np.arctan2(dy, dx)
        points_with_angle[i, 2] = angle
    
    # Dernier point garde l'angle du précédent
    points_with_angle[-1, 2] = points_with_angle[-2, 2]
    
    return points_with_angle

def visualize_trajectory(original_points, smooth_points, world_points, points_with_angle):
    """Visualise les trajectoires"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # 1. Points extraits
    ax = axes[0, 0]
    ax.plot(original_points[:, 0], original_points[:, 1], 'ro-', linewidth=2, markersize=8, label='Points extraits')
    ax.set_title("1️⃣ Points extraits du PDF")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.grid(True)
    ax.legend()
    ax.axis('equal')
    
    # 2. Trajectoire lissée (pixels)
    ax = axes[0, 1]
    ax.plot(original_points[:, 0], original_points[:, 1], 'ro', markersize=8, alpha=0.5, label='Points bruts')
    ax.plot(smooth_points[:, 0], smooth_points[:, 1], 'b-', linewidth=2, label='Trajectoire lissée')
    ax.set_title("2️⃣ Trajectoire lissée (pixels)")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.grid(True)
    ax.legend()
    ax.axis('equal')
    
    # 3. Coordonnées monde (mètres)
    ax = axes[1, 0]
    ax.plot(world_points[:, 0], world_points[:, 1], 'g-', linewidth=2)
    ax.plot(world_points[:, 0], world_points[:, 1], 'go', markersize=4)
    ax.plot(world_points[0, 0], world_points[0, 1], 'go', markersize=10, label='Début')
    ax.plot(world_points[-1, 0], world_points[-1, 1], 'r*', markersize=15, label='Fin')
    ax.set_title("3️⃣ Trajectoire en mètres")
    ax.set_xlabel("X (mètres)")
    ax.set_ylabel("Y (mètres)")
    ax.grid(True)
    ax.legend()
    ax.axis('equal')
    
    # 4. Orientation (angle)
    ax = axes[1, 1]
    ax.plot(world_points[:, 0], world_points[:, 1], 'g-', linewidth=2, alpha=0.5, label='Chemin')
    # Affiche les vecteurs d'orientation tous les 10 points
    step = max(1, len(points_with_angle) // 10)
    for i in range(0, len(points_with_angle), step):
        x, y, theta = points_with_angle[i]
        # Vecteur de direction
        scale = 0.05
        dx = scale * np.cos(theta)
        dy = scale * np.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.01, head_length=0.01, fc='blue', ec='blue')
    ax.set_title("4️⃣ Orientations (angles)")
    ax.set_xlabel("X (mètres)")
    ax.set_ylabel("Y (mètres)")
    ax.grid(True)
    ax.legend()
    ax.axis('equal')
    
    plt.tight_layout()
    plt.show()

def save_trajectory(world_points, filename="../data/trajectory.npy"):
    """Sauvegarde la trajectoire en fichier"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    np.save(filename, world_points)
    print(f"✅ Trajectoire sauvegardée: {filename}")
    
    # Affiche aussi en CSV pour faciliter l'inspection
    csv_filename = filename.replace('.npy', '.csv')
    header = "x (m),y (m),theta (rad)\n"
    with open(csv_filename, 'w') as f:
        np.savetxt(f, world_points, delimiter=',', fmt='%.6f')
    print(f"✅ CSV sauvegardé: {csv_filename}")

# ✅ FIXED: [BUG 5: Hardcoded paths -> Pathlib + error msg, RPI-4: Headless matplotlib backend configuration]

if __name__ == "__main__":
    from pdf_extractor import extract_path_from_pdf
    
    print("🤖 Génération de trajectoire")
    print("=" * 50)
    
    # 1. Extrait les points du PDF
    project_root = Path(__file__).parent.parent
    pdf_file = project_root / "data" / "plan_square.pdf"
    
    if not pdf_file.exists():
        print(f"❌ Impossible de trouver le fichier PDF !")
        print(f"   Chemin recherché: {pdf_file.resolve()}")
        sys.exit(1)
        
    # Assume extract_path_from_pdf supports string paths
    original_points = extract_path_from_pdf(str(pdf_file))
    
    if original_points is None:
        print("❌ Impossible d'extraire les points")
        sys.exit(1)
    
    print(f"\n✅ Points extraits: {len(original_points)}")
    
    # 2. Lisse la trajectoire
    smooth_points = smooth_trajectory(original_points, num_points=100)
    print(f"✅ Trajectoire lissée: {len(smooth_points)} points")
    
    # 3. Convertit en coordonnées monde (mètres)
    # Assume: 1 pixel = 1 mm (pixel_to_meter=0.001)
    world_points = pixel_to_world(smooth_points, pixel_to_meter=0.001)
    print(f"✅ Conversion en mètres effectuée")
    print(f"   - Dimensions: {world_points[:, 0].max() - world_points[:, 0].min():.3f}m x {world_points[:, 1].max() - world_points[:, 1].min():.3f}m")
    
    # 4. Ajoute l'orientation
    points_with_angle = add_orientation(world_points)
    print(f"✅ Orientations calculées")
    
    # 5. Visualise
    print("\n📊 Génération du graphique de visualisation...")
    visualize_trajectory(original_points, smooth_points, world_points, points_with_angle)
    
    # 6. Sauvegarde
    save_trajectory(points_with_angle)
    
    print("\n" + "=" * 50)
    print("✅ Trajectoire générée avec succès!")
    print("=" * 50)