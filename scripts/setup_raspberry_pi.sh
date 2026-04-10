#!/bin/bash
# =========================================================
# Setup Raspberry Pi - Robot Traceur PDF
# =========================================================

echo "🔄 Démarrage de l'installation..."

sudo apt update && sudo apt upgrade -y

echo "📦 Installation des dépendances système..."
sudo apt install -y python3-pip python3-venv python3-opencv
sudo apt install -y libatlas-base-dev  # for numpy
sudo apt install -y pigpio python3-pigpio

echo "⚙️ Activation du démon pigpio..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo "🐍 Installation des dépendances Python..."
# L'utilisation de --break-system-packages est parfois nécessaire sur Bookworm si on utilise pas de venv.
# On utilise pip install avec les bibliothèques demandées.
pip3 install PyMuPDF numpy matplotlib scipy pyserial RPi.GPIO pigpio

echo "🔌 Test du module GPIO..."
python3 -c "import RPi.GPIO; print('✅ RPi.GPIO OK')"
python3 -c "import pigpio; print('✅ pigpio OK')"

echo "🏁 Installation terminée. Vous pouvez lancer le robot avec:"
echo "   python3 main.py --mode gpio --pdf data/plan_square.pdf"
