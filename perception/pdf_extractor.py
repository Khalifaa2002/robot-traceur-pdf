"""
perception/pdf_extractor.py
==========================
Extracts tracing trajectories from PDF floor plans.

This module uses computer vision (OpenCV) and PDF parsing (PyMuPDF) to 
detect lines and contours in PDF files, converting them into a set 
of raw pixel coordinates.

Status: Production (migrated from v3.1 core)
"""

import fitz  # PyMuPDF
import cv2
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import os
from pathlib import Path

# Headless backend configuration for Raspberry Pi (no display)
if os.environ.get('DISPLAY') is None:
    matplotlib.use('Agg')

def extract_path_from_pdf(pdf_path, scanned=False, dpi=300):
    """
    Extract trajectory points from a simple or scanned PDF.
    
    Args:
        pdf_path: Path to the PDF file
        scanned: If True, uses probabilistic Hough transform for hand-drawn/scanned plans
        dpi: Resolution for rasterization (higher is more precise but slower)
        
    Returns:
        numpy.ndarray: Array of (x, y) coordinates in pixels, or None if failed
    """
    if not os.path.exists(pdf_path):
        print(f"❌ File {pdf_path} not found!")
        return None
    
    print(f"📖 Opening PDF: {pdf_path}")
    
    try:
        doc = fitz.open(pdf_path)
        page = doc[0]
        
        # Scale factor from standard 72 DPI to target DPI
        scale = dpi / 72.0
        pix = page.get_pixmap(matrix=fitz.Matrix(scale, scale))
        img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
        
        print(f"✅ Image extracted: {img.shape}")
        
        # Pre-processing
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological operations to close small gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.dilate(binary, kernel, iterations=2)
        
        points = []
        
        if scanned:
            # Scanned mode: prioritize lines
            edges = cv2.Canny(binary, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    points.append([x1, y1])
                    points.append([x2, y2])
        else:
            # Digital mode: prioritize vector contours
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            print(f"🔍 {len(contours)} contours found")
            
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Noise filter
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    for point in approx:
                        points.append(point[0])
                        
            # Fallback to Hough if no contours detected
            if len(points) == 0:
                edges = cv2.Canny(binary, 50, 150, apertureSize=3)
                lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        points.append([x1, y1])
                        points.append([x2, y2])
        
        if points:
            points_arr = np.array(points, dtype=np.float32)
            print(f"✅ {len(points_arr)} points extracted")
            return points_arr
        else:
            print("⚠️ No points found in PDF")
            return None
            
    except Exception as e:
        print(f"❌ PDF Extraction error: {e}")
        return None

def visualize_path(points, title="Extracted Trajectory"):
    """Plot the extracted points using matplotlib (requires DISPLAY or GUI)."""
    if points is None or len(points) == 0:
        print("❌ No points to display")
        return
    
    plt.figure(figsize=(10, 8))
    plt.plot(points[:, 0], points[:, 1], 'b-', linewidth=2, label='Trajectory')
    plt.plot(points[:, 0], points[:, 1], 'ro', markersize=4, alpha=0.5)
    plt.plot(points[0, 0], points[0, 1], 'go', markersize=10, label='Start')
    plt.plot(points[-1, 0], points[-1, 1], 'r*', markersize=15, label='End')
    
    plt.title(title)
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()
