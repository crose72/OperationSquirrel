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
	echo "Yay! You have enough memory for OpenCV-4-6-0, proceed."
fi

# -----------------------------------------------
# Install OpenCV-4-10-0
# -----------------------------------------------

# You need at least a total of 6.5 GB!
# if not, enlarge your swap space as explained in the guide
# https://qengineering.eu/install-opencv-on-jetson-nano.html
echo "Installing OpenCV-4-10-0"
echo "You need at least a total of 6.5 GB of memory for OpenCV-4-10-0"

wget https://github.com/Qengineering/Install-OpenCV-Jetson-Nano/raw/main/OpenCV-4-10-0-tracking.sh
sudo chmod 755 ./OpenCV-4-10-0-tracking.sh
./OpenCV-4-10-0-tracking.sh

rm OpenCV-4-10-0-tracking.sh

echo "OpenCV-4-10-0 installed, SUCCESS!"

# -----------------------------------------------
# Install JsonCpp
# -----------------------------------------------

sudo apt-get install libjsoncpp-dev

echo "JsonCpp installed, SUCCESS!"

# -----------------------------------------------
# Install armadillo and other dependencies for
# kalman filter submodule
# -----------------------------------------------
sudo apt-get update && apt install -y \
    libspdlog-dev \
    libfmt-dev \
    libjsoncpp-dev \
    libarmadillo-dev \
    liblapack-dev \
    libblas-dev