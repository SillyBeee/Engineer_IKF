#!/bin/bash
echo "Init submodules..."
git submodule init
git submodule update
echo "Submodules initialized."

echo "Setting up apt libraries..."
sudo apt update
sudo apt install -y libsdl1.2-dev ros-jazzy-moveit ros-jazzy-moveit-servo ros-jazzy-serial-driver ros-jazzy-asio-cmake-module ros-jazzy-foxglove-bridge
echo "Apt libraries setup complete."


echo "Setting up Orbbec SDK v2..."
git clone https://github.com/orbbec/OrbbecSDK_v2.git
cd OrbbecSDK_v2
cd scripts/env_setup
  sudo chmod +x ./install_udev_rules.sh
  sudo ./install_udev_rules.sh
  sudo udevadm control --reload && sudo udevadm trigger
cd ../../../
rm -rf OrbbecSDK_v2
echo "Orbbec SDK v2 setup complete."


echo "Setting up ONNX Runtime..."
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-aarch64-1.20.0.tgz
tar -xvzf onnxruntime-linux-aarch64-1.20.0.tgz
sudo rm -rf /opt/onnxruntime
sudo mkdir -p /opt/onnxruntime
sudo cp -r onnxruntime-linux-aarch64-1.20.0/* /opt/onnxruntime
echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
sudo rm -rf onnxruntime-linux-aarch64-1.20.0 onnxruntime-linux-aarch64-1.20.0.tgz
echo "ONNX Runtime setup complete."








