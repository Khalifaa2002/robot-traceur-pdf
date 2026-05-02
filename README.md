# 🤖 Robot Traceur de Plan PDF v3.0 (Production Ready)

Un robot autonome de précision conçu pour tracer des plans PDF sur le terrain avec un système avancé de correction en temps réel et un firmware optimisé pour Raspberry Pi.

## 🚀 Fonctionnalités Avancées (v3.0)

- **📄 Support PDF Scanné** : Utilisation d'OpenCV et algorithme de Hough (fallback) pour digitaliser et traiter des plans issus de scanners.
- **🎯 Compensation d'Outil (Tool Offset)** : Calcul vectoriel pour corriger la position de la pointe du traceur par rapport au centre de rotation du robot.
- **📊 Mesures de Précision** : Rapports JSON générés post-traçage incluant le taux de complétion, l'erreur quadratique moyenne (RMS) et l'erreur maximale transversale.
- **⚙️ CLI Robuste** : Ligne de commande unifiée via `main.py` supportant des environnements headless (RPI).

## 🛠️ Utilisation en Ligne de Commande

Le système propose un CLI complet pour l'opération :

```bash
python main.py --mode simulation --pdf data/plan_square.pdf [OPTIONS]
```

### Options Principales
* `--mode` : `simulation`, `serial`, `gpio` (Défaut: `simulation`)
* `--pdf` : Le chemin vers le fichier PDF à analyser
* `--controller` : Type de contrôleur (ex: `pid`)
* `--validate` : Génère un rapport de validation JSON détaillé en fin de session
* `--scanned` : Active l'analyse optimisée pour les PDF scannés via Hough lines
* `--dpi` : Définir la résolution d'extraction (Défaut: `300`)
* `--tool-offset-x` / `--tool-offset-y` : Décalage en mètres de l'outil par rapport au centre (ex: `--tool-offset-x 0.1`)
* `--no-plot` : Désactive la visualisation matplotlib (recommandé sur Raspberry Pi)

## ✅ Status du Projet
- [x] Logiciel préparé & Architecture validée
- [x] Tests unitaires & intégration réussis
- [x] Mode Headless (RPI) validé
- [ ] Hardware assemblé
- [ ] Déploiement sur site

## 📦 Installation

```bash
python -m venv venv
.\venv\Scripts\Activate
pip install -r requirements.txt
python run_tests.py
```

## 👤 Auteur
Khalifaa2002

## 📄 Licence
MIT
