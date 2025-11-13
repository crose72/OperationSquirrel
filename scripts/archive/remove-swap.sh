#!/bin/bash

# Remove existing swap files if any
sudo swapoff /swapfile
sudo rm -f /swapfile
sudo dphys-swapfile swapoff
sudo dphys-swapfile uninstall

# Verify if swap was removed
swapon --show

echo "Swap removed successfully."

