#!/bin/bash
# Deploy script for Raspberry Pi

echo "🚀 Deploying to Raspberry Pi..."

# 1. Clone le repo
git clone https://github.com/yourusername/robot-traceur-pdf.git
cd robot-traceur-pdf

# 2. Install ROS 2
sudo apt update
sudo curl -sSL https://repo.ros2.org/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-jazzy-desktop

# 3. Setup Python env
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 4. Run tests
python tests/test_imports.py

echo "✅ Deployment complete!"
