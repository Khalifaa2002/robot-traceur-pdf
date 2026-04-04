\# 🤖 Robot Traceur de Plan PDF - Guide d'Intégration



\## Architecture



```

PDF Plan

&#x20;   ↓

\[Raspberry Pi 4: Python + ROS 2]

&#x20;   ├── pdf\_extractor.py      : Extraction des points

&#x20;   ├── trajectory\_generator.py: Génération trajectoire

&#x20;   └── ros\_trajectory\_node.py : Nœud ROS 2

&#x20;   ↓ (UART/USB)

\[Microcontrôleur: STM32/Arduino]

&#x20;   ├── Encodeurs (lecture position)

&#x20;   ├── IMU 9-DOF (orientation)

&#x20;   └── Commande moteurs PWM

&#x20;   ↓

\[Robot Physique]

&#x20;   ├── Moteurs DC + réducteur

&#x20;   ├── Encodeurs optiques

&#x20;   └── Outil traçage (stylo/craie)

```



\## Installation sur Raspberry Pi



\### 1. Système d'exploitation

```bash

\# Installez Ubuntu 24.04 ou Raspberry Pi OS

\# Ensuite installez ROS 2 Jazzy

sudo apt install ros-jazzy-desktop

```



\### 2. Dépendances Python

```bash

source venv/bin/activate

pip install pyserial PyMuPDF opencv-python numpy matplotlib

```



\### 3. Configuration microcontrôleur



Téléversez `microcontroller\_code.ino` sur votre STM32 ou Arduino avec :

\- \*\*Port:\*\* COM3 (Windows) ou /dev/ttyUSB0 (Linux)

\- \*\*Baudrate:\*\* 115200



\### Schéma de branchement (Arduino/STM32)



```

Encodeur Gauche  → Pin 2 (INT0)

Encodeur Droit   → Pin 3 (INT1)

Motor Left PWM   → Pin 9

Motor Left Dir   → Pin 8

Motor Right PWM  → Pin 10

Motor Right Dir  → Pin 7

IMU (I2C)        → SDA/SCL

```



\## Utilisation



\### 1. Extraction PDF

```bash

cd robot-traceur-pdf/src

python pdf\_extractor.py

```



\### 2. Génération trajectoire

```bash

python trajectory\_generator.py

```



\### 3. Test sans robot (mode simulation)

```bash

python test\_trajectory\_node.py

```



\### 4. Suivi de trajectoire réelle

```bash

python microcontroller\_interface.py

```



\## Paramètres à ajuster



\### Calibration encodeurs

Dans `microcontroller\_code.ino`:

```cpp

const float WHEEL\_DIAMETER = 0.065;  // À mesurer réellement

const float PPR = 20.0;              // À adapter selon l'encodeur

```



\### Conversion pixel→mètre

Dans `trajectory\_generator.py`:

```python

pixel\_to\_meter=0.001  # 1 pixel = 1 mm (à ajuster selon votre plan)

```



\### Vitesse de suivi

```python

follower.follow\_trajectory(velocity=0.5)  # m/s

```



\## Troubleshooting



\### Connexion microcontrôleur échoue

```python

\# Vérifiez le port COM

import serial

ports = serial.tools.list\_ports.comports()

for p in ports:

&#x20;   print(f"{p.device}: {p.description}")

```



\### Trajectoire décalée

\- Vérifiez l'étalonnage des encodeurs

\- Vérifiez la conversion pixel→mètre



\## Prochaines étapes



1\. \*\*Localisation avancée\*\*: Fusion encodeurs + IMU (Filtre de Kalman)

2\. \*\*Vision\*\*: Recalage avec caméra + ArUco tags

3\. \*\*IA\*\*: Détection automatique de plans scannés

4\. \*\*Multi-robot\*\*: Coordination de plusieurs robots



\## Ressources



\- \[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

\- \[ROS 2 Documentation](https://docs.ros.org)

\- \[micro-ROS](https://micro.ros.org/)

