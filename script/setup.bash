#!/bin/bash
echo "Init submodules..."
git submodule init
git submodule update
echo "Submodules initialized."

echo "Setting up apt libraries..."
sudo apt update
sudo apt install -y libsdl1.2-dev ros-jazzy-moveit ros-jazzy-moveit-servo ros-jazzy-serial-driver ros-jazzy-asio-cmake-module ros-jazzy-foxglove-bridge
echo "Apt libraries setup complete."

git clone https://github.com/orbbec/OrbbecSDK_v2.git
cd OrbbecSDK_v2
cd scripts/env_setup
  sudo chmod +x ./install_udev_rules.sh
  sudo ./install_udev_rules.sh
  sudo udevadm control --reload && sudo udevadm trigger






