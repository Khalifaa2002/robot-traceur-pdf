# 🤖 Robot Traceur de Plan PDF

> **Version active :** v3.1 → Migration v4.0 en cours  
> **Backup disponible :** `git checkout v3.1-stable`  
> **Dernière milestone :** Phase A (Cleanup) ✅ COMPLÈTE

Un robot autonome de précision conçu pour tracer des plans PDF sur le terrain avec un système avancé de correction en temps réel et un firmware optimisé pour Raspberry Pi.

---

## 📊 État Actuel du Système (Live System State)

| Paramètre | Statut |
|-----------|--------|
| **Étape de Développement** | v3.1 - Phase de Stabilisation |
| **Statut Matériel** | 💻 SIMULATION (Mocked GPIO/Serial) |
| **Tests Unitaires** | ✅ PASS (run_tests.py) |
| **Précision Trajectoire** | 📈 Améliorée (Adaptive Velocity Scaling) |
| **Fiabilité Odométrie** | 🛠️ Corrigée (Support du recul & Filtrage) |

---

## 🏗️ Architecture du Système

Le système repose sur une architecture modulaire stricte garantissant la portabilité entre simulation et matériel réel.

- **`core/`** : Logique métier (Contrôleur PID adaptatif, HAL matériel, Extracteur PDF OpenCV).
- **`utils/`** : Gestion de la configuration globale et logging industriel.
- **`data/`** : Stockage des plans PDF et des trajectoires générées.

---

## 🚀 Fonctionnalités Implémentées

- [x] **Extraction PDF Vectorielle & Scannée** (OpenCV/Hough fallback).
- [x] **Lissage de Trajectoire Spline** (Cubic interpolation).
- [x] **Odométrie Différentielle Robuste** : Gestion des overflows, resets et support de la marche arrière.
- [x] **Contrôle PID Adaptatif** : Réduction automatique de la vitesse en virage pour minimiser le glissement.
- [x] **Compensation d'Outil (Tool Offset)** : Correction vectorielle de la position de la pointe.
- [x] **Rapports de Validation JSON** : Métriques RMS et Max Error post-mission.

---

## 🛠️ Historique des Versions (Versioning)

- **v3.0** : Unification de l'architecture (`core/` & `utils/`). Nettoyage complet du dépôt.
- **v3.1 (Actuelle)** : **Phase de Stabilisation**.
    - Amélioration de l'odométrie (support des encodeurs quadrature en recul).
    - Ajout de l'Adaptive Velocity Scaling dans le contrôleur.
    - Mise à jour des paramètres hardware par défaut (PPR=32).
    - Intégration du filtrage de bruit sur les signaux d'encodeurs.

---

## ⚠️ Limitations Connues & Prochaines Améliorations

### Limitations
- Absence de fusion de capteurs (IMU/GPS) pour corriger la dérive odométrique sur longue distance.
- Sensibilité au glissement excessif sur surfaces lisses (compensation logicielle en cours).

### Améliorations Prévues
- [ ] Intégration d'un filtre de Kalman (EKF) pour la fusion IMU/Odométrie.
- [ ] Optimisation de la performance OpenCV pour les gros fichiers PDF sur Raspberry Pi Zero.
- [ ] Implémentation d'un évitement d'obstacles dynamique (DWA).

---

## ▶️ Utilisation du CLI (Latest Usage)

```bash
# Lancer la simulation avec validation des performances
python -X utf8 main.py --mode simulation --pdf data/plan_square.pdf --validate
```

**Options disponibles :**
- `--mode [simulation|serial|gpio]` : Interface robot.
- `--pdf <path>` : Fichier source.
- `--tool-offset-x/y <meters>` : Décalage physique de l'outil.
- `--scanned` : Mode extraction OpenCV pour PDF non-vectoriels.
- `--no-plot` : Mode headless pour Raspberry Pi.

---

## 🧪 Statut des Tests

| Test | Résultat | Date |
|------|----------|------|
| `run_tests.py` | ✅ PASS | 2026-05-02 |
| `test_production_features.py` | ✅ PASS | 2026-05-02 |
| Simulation E2E (Square) | 🔄 EN COURS | 2026-05-02 |

---

## 📄 Licence
MIT
