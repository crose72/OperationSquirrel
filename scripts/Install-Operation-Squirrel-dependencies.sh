#!/bin/bash

# -----------------------------------------------
# Preparation
# -----------------------------------------------
echo "Checking for updates"
sudo apt-get update
sudo apt-get upgrade

# -----------------------------------------------
# Check memory
# -----------------------------------------------

echo "Checking if you have enough  memory for OpenCV build"

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

if [ "$total_memory" -lt 10000 ]; then
	echo "Not enough memory, need at least 10000 MB, you have $total_memory MB"
	exit 1
else
	echo "Yay! You have enough memory for OpenCV-4-5-0, proceed."
fi

# -----------------------------------------------
# Install OpenCV-4-5-0
# -----------------------------------------------

# You need at least a total of 6.5 GB!
# if not, enlarge your swap space as explained in the guide
# https://qengineering.eu/install-opencv-on-jetson-nano.html
echo "Installing OpenCV-4-5-0"
echo "You need at least a total of 6.5 GB of memory for OpenCV-4-5-0"

wget https://github.com/Qengineering/Install-OpenCV-Jetson-Nano/raw/main/OpenCV-4-5-0.sh
sudo chmod 755 ./OpenCV-4-5-0.sh
./OpenCV-4-5-0.sh

rm OpenCV-4-5-0.sh

echo "OpenCV-4-5-0 installed, SUCCESS!"

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
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3

# Run build script
chmod +x build.sh
./build.sh

cd ..

echo "ORB_SLAM3 installed, SUCCESS!"

# -----------------------------------------------
# Install Jetson-Inference
# -----------------------------------------------

echo "Installing Jetson-Inference"

sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig

echo "Jetson-Inference installed, SUCCESS!"

cd ../..

# -----------------------------------------------
# Install JetsonGPIO
# -----------------------------------------------

echo "Installing JetsonGPIO"

git clone https://github.com/pjueon/JetsonGPIO.git

cd JetsonGPIO
mkdir build
cd build
cmake ..
make
sudo make install

echo "JetsonGPIO installed, SUCCESS!"

cd ../..

# -----------------------------------------------
# Install JsonCpp
# -----------------------------------------------

sudo apt-get install libjsoncpp-dev

echo "JsonCpp installed, SUCCESS!"