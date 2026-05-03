# 🤖 Robot Traceur de Plan PDF

> **Version active :** v3.1 → Migration v4.0 en cours
> **Backup disponible :** `git checkout v3.1-stable`
> **Migration v4.0 :** Phase B ✅ COMPLÈTE — **Progression : 28%**

---

## 📊 État Actuel du Système (Live System State)

| Paramètre | Statut |
|-----------|--------|
| **Version** | v3.1 stable (migration v4.0 en cours) |
| **Étape de Développement** | Phase B/G — Scaffolding ✅ |
| **Statut Matériel** | 💻 SIMULATION (Mocked GPIO/Serial) |
| **Tests Unitaires** | ✅ PASS (run_tests.py — 6/6) |

---

## 🏗️ Architecture v4.0 (En cours de migration)

```
robot-traceur-pdf/
│
├── app/              ← [SCAFFOLD] Orchestration (CLI, mission)
├── control/          ← [SCAFFOLD] Contrôleurs (PID, Pure Pursuit)
├── planning/         ← [SCAFFOLD] Planification (Spline, DWA)
├── localization/     ← [SCAFFOLD] Localisation (Odometry, EKF)
├── perception/       ← [SCAFFOLD] Perception (PDF extraction)
├── telemetry/        ← [SCAFFOLD] Télémétrie (Métriques, rapports)
│
├── core/             ← [STABLE v3.1] Logique actuelle (EN ATTENTE phase D)
│   ├── controller.py
│   ├── hardware.py
│   ├── localization.py
│   ├── pdf_extractor.py
│   └── trajectory_generator.py
│
├── utils/            ← [STABLE] Config + Logger (inchangé)
├── hardware/         ← [EXISTANT] placeholder
├── tests/            ← [STABLE] Suite de tests complète
├── scripts/          ← [STABLE] Déploiement Raspberry Pi
├── data/             ← [STABLE] Plans PDF et trajectoires
│
├── main.py           ← [STABLE] Point d'entrée CLI
├── run_tests.py      ← [STABLE] Lanceur de tests
└── microcontroller_code.ino  ← [NE PAS TOUCHER]
```

---

## 📈 Progression de la Migration v4.0

| Phase | Description | Statut |
|-------|-------------|--------|
| **A** | Nettoyage des fichiers morts | ✅ COMPLÈTE |
| **B** | Création du squelette de répertoires | ✅ COMPLÈTE |
| **C** | Import des algorithmes PythonRobotics | ⏳ EN ATTENTE |
| **D** | Migration `core/` → packages spécialisés | ⏳ EN ATTENTE |
| **E** | Remplacement des implémentations faibles | ⏳ EN ATTENTE |
| **F** | Validation matérielle | ⏳ EN ATTENTE |
| **G** | Documentation finale | ⏳ EN ATTENTE |

> **Progression globale : 28% (2/7 phases)**

---

## 🚀 Fonctionnalités Actuelles (v3.1 stable)

- [x] Extraction PDF vectorielle & scannée (OpenCV/Hough fallback)
- [x] Lissage de trajectoire (interpolation)
- [x] Odométrie différentielle robuste (encodeurs quadrature, support recul)
- [x] Contrôle PID adaptatif (Adaptive Velocity Scaling)
- [x] Compensation d'outil (Tool Offset vectoriel)
- [x] HAL multi-plateforme (Simulation / Serial / GPIO)
- [x] Rapports de validation JSON (RMS, Max Error)

---

## 🔜 Prochaines fonctionnalités (Phase C)

- [ ] **Pure Pursuit** — Meilleur suivi en virage (adapté depuis PythonRobotics)
- [ ] **Cubic Spline** — Lissage C² supérieur à l'interpolation linéaire actuelle
- [ ] **EKF** — Réduction de la dérive odométrique (sans GPS)

---

## ▶️ Utilisation du CLI

```bash
# Simulation avec validation
python -X utf8 main.py --mode simulation --pdf data/plan_square.pdf --validate

# Raspberry Pi (GPIO)
python main.py --mode gpio --pdf data/plan.pdf --no-plot

# Arduino (Serial)
python main.py --mode serial --port /dev/ttyACM0 --pdf data/plan.pdf
```

**Options CLI disponibles :**
| Flag | Description |
|------|-------------|
| `--mode [simulation\|serial\|gpio]` | Interface robot |
| `--pdf <path>` | Fichier source PDF |
| `--controller [pid\|pure_pursuit]` | Algorithme de contrôle *(pure_pursuit en Phase C)* |
| `--tool-offset-x/y <m>` | Décalage physique de l'outil |
| `--scanned` | Mode extraction OpenCV pour PDF non-vectoriels |
| `--no-plot` | Mode headless pour Raspberry Pi |
| `--validate` | Génère un rapport JSON de précision |
| `--dpi <int>` | DPI de rastérisation PDF (défaut: 300) |

---

## 🧪 Commandes de Test

```bash
# Suite principale (recommandé)
python -X utf8 run_tests.py

# Via pytest avec couverture
pytest tests/ -v --tb=short
```

---

## 🔁 Rollback

```bash
# Revenir à l'état v3.1 stable à tout moment
git checkout v3.1-stable
```

---

## 🗺️ Roadmap

- **v4.0** *(en cours)* — Architecture modulaire + Pure Pursuit + Cubic Spline + EKF
- **v4.1** — Wizard de calibration, évitement DWA, watchdog
- **v4.2** — Dashboard web temps réel, API REST, mobile
- **v5.0** — SLAM, flotte multi-robots, optimisation IA, cloud

---

## 📄 Licence
MIT
