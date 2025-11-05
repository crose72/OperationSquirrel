#!/bin/bash

# -----------------------------------------------
# Install ORB_SLAM3 dependencies
# -----------------------------------------------

echo "Installing Pangolin"

sudo apt install ninja-build

# Building Pangolin https://github.com/stevenlovegrove/Pangolin
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install Pangolin dependencies
./scripts/install_prerequisites.sh -m brew all

# Would like to find a way to install it if possible though
echo "If catch2 isn't found it's OKAY"

# Configure and build
mkdir build
cd build
cmake -gNinja ..
make

cd ../..

echo "Pangolin installed, SUCCESS!"
