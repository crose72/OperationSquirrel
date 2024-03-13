#!/bin/bash

# -----------------------------------------------
# Preparation
# -----------------------------------------------
echo "Checking for updates"
sudo apt-get update
sudo apt-get upgrade

echo "Adding ZRAM and file swap"

# Enable default zram swap space
sudo /etc/systemd/nvzramconfig.sh

# Add swap file
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon -a

# install dphys-swapfile (for extra swap)
sudo apt-get install dphys-swapfile

# enlarge the boundary (OpenCV 4.5.2 and higher)
sudo vim /sbin/dphys-swapfile

# give the required memory size
sudo vim /etc/dphys-swapfile

# reboot afterwards
sudo reboot

# Check if swap space was added
swapon --show

# Check memory (another way to check that swap was correctly added)
free -m

echo "Preparation complete"

