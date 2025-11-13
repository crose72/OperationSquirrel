#!/bin/bash

# -----------------------------------------------
# Preparation
# -----------------------------------------------
echo "Checking for updates"
sudo apt-get update
sudo apt-get upgrade

# -----------------------------------------------
# Install ORB_SLAM3 dependencies
# -----------------------------------------------

echo "Installing Pangolin"

sudo apt install ninja-build

# Building Pangolin https://github.com/stevenlovegrove/Pangolin
git clone --recursive https://github.com/crose72/Pangolin.git
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

# Additional libraries (I had to do this manually because of compile errors)
sudo apt-get install libssl-dev
sudo apt-get install libboost-all-dev

# -----------------------------------------------
# Check memory
# -----------------------------------------------

echo "Checking if you have enough  memory for ORB_SLAM3 build"

# Get memory information
memory_info=$(free -m)

# Extract total memory using awk
mem=$(echo "$memory_info" | awk 'NR==2 {print $2}')

# Extract swap information using awk
swap=$(echo "$memory_info" | awk '/Swap:/ {print $2}')

# Perform addition
total_memory=$((mem + swap))

# Display the total memory and swap
echo "Total Memory: $total_memory MB"

if [ "$total_memory" -lt 12288 ]; then
	echo "Not enough memory, need at least 12288 MB, you have $total_memory MB"
	exit 1
else
	echo "Yay! You have enough memory for ORB SLAM3, proceed."
fi

# -----------------------------------------------
# Install ORB_SLAM3
# -----------------------------------------------

echo "Installing ORB_SLAM3"

# ORB_SLAM3 full build https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
git clone https://github.com/crose72/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3

# Run build script
chmod +x build.sh
./build.sh

cd ..

echo "ORB_SLAM3 installed, SUCCESS!"