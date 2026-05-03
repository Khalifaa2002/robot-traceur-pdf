"""
pdf_extractor.py
Extrait une trajectoire d'un plan PDF
"""

import fitz  # PyMuPDF
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
import os
import sys
from pathlib import Path

# Headless check for Raspberry Pi
if os.environ.get('DISPLAY') is None:
    matplotlib.use('Agg')

def extract_path_from_pdf(pdf_path, scanned=False, dpi=300):
    """Extrait les points de trajectoire d'un PDF simple ou scanné"""
    if not os.path.exists(pdf_path):
        print(f"❌ Le fichier {pdf_path} n'existe pas!")
        return None
    
    print(f"📖 Ouverture du PDF: {pdf_path}")
    
    try:
        # Ouvrir le PDF
        doc = fitz.open(pdf_path)
        page = doc[0]
        
        # Convertir en image haute résolution
        # Matrix scale depends on DPI (72 default)
        scale = dpi / 72.0
        pix = page.get_pixmap(matrix=fitz.Matrix(scale, scale))
        img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
        
        print(f"✅ Image extraite: {img.shape}")
        
        # Convertir en gris
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # Binarisation
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Dilater pour joindre les petites brèches
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.dilate(binary, kernel, iterations=2)
        
        points = []
        
        if scanned:
            print("🔍 Mode scanné activé (Hough Lines fallback)")
            edges = cv2.Canny(binary, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    points.append([x1, y1])
                    points.append([x2, y2])
        else:
            # Détecter contours
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            print(f"🔍 {len(contours)} contours trouvés")
            
            # Extraire points des plus gros contours
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Filtrer le bruit
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    
                    for point in approx:
                        points.append(point[0])
                        
            # Hough fallback if no contours
            if len(points) == 0:
                print("⚠️ Aucun contour trouvé, tentative avec Hough Lines...")
                edges = cv2.Canny(binary, 50, 150, apertureSize=3)
                lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        points.append([x1, y1])
                        points.append([x2, y2])
        
        if points:
            points = np.array(points, dtype=np.float32)
            print(f"✅ {len(points)} points extraits!")
            return points
        else:
            print("⚠️ Aucun point trouvé dans le PDF")
            return None
            
    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
        return None

def visualize_path(points, title="Trajectoire extraite"):
    """Visualise les points extraits"""
    if points is None or len(points) == 0:
        print("❌ Pas de points à afficher")
        return
    
    plt.figure(figsize=(10, 8))
    plt.plot(points[:, 0], points[:, 1], 'b-', linewidth=2, label='Trajectoire')
    plt.plot(points[:, 0], points[:, 1], 'ro', markersize=4, alpha=0.5)
    plt.plot(points[0, 0], points[0, 1], 'go', markersize=10, label='Début')
    plt.plot(points[-1, 0], points[-1, 1], 'r*', markersize=15, label='Fin')
    
    plt.title(title)
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Test avec le PDF exemple
    project_root = Path(__file__).parent.parent
    pdf_file = project_root / "data" / "plan_square.pdf"
    
    if not pdf_file.exists():
        print(f"❌ Impossible de trouver le fichier PDF !")
        print(f"   Chemin recherché: {pdf_file.resolve()}")
        sys.exit(1)
    
    print("🤖 Robot Traceur de Plan PDF - Extraction")
    print("=" * 50)
    
    points = extract_path_from_pdf(str(pdf_file))
    
    if points is not None:
        visualize_path(points, "Plan extrait du PDF")
        print(f"\n📊 Statistiques:")
        print(f"   - Nombre de points: {len(points)}")
        print(f"   - Min X,Y: ({points[:, 0].min():.0f}, {points[:, 1].min():.0f})")
        print(f"   - Max X,Y: ({points[:, 0].max():.0f}, {points[:, 1].max():.0f})")

# ✅ FIXED: [BUG 5: Hardcoded PDF path, RPI-4: Headless matplotlib constraint]
