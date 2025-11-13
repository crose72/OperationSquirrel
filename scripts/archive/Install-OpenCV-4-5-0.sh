#!/bin/bash

# -----------------------------------------------
# Preparation
# -----------------------------------------------
echo "Checking for updates"
$ sudo apt-get update
$ sudo apt-get upgrade

echo "Adding ZRAM and file swap"

# Enable default zram swap space
sudo /etc/systemd/nvzramconfig.sh

# Add swap file
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon -a

# Check if swap space was added
swapon --show

# Check memory (another way to check that swap was correctly added)
free -m

echo "Preparation complete"

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

