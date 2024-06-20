#!/bin/bash

# -----------------------------------------------
# Install Jetson-Inference
# -----------------------------------------------

echo "Installing Jetson-Inference"

sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference

# Configure and build
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig

cd ../..

echo "Jetson-Inference installed, SUCCESS!"
