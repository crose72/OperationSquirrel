# Installing software on the Jetson-Nano

## Description

This folder contains scripts to automatically install certain dependencies needed for the project, including OpenCV, ORB SLAM3, and Jetson Inference.

## Installation instructions

1. Copy `./Install-Operation-Squirrel-dependencies.sh` to your favorite code folder (if you run it in here then everything will download in here) 
2. Run ` sudo ./setup-swap.sh` to install the necessary swap needed for all builds
    - OpenCV 4.5.0 needs about 10 GB (takes about 2 hours) and ORB SLAM3 needs about 14 GB (I went to sleep, but at least 2 hours)
3. Edit the `/sbin/dphys-swapfile` so that `CONF_MAXSWAP=4096` and `CONF_SWAPFACTOR=2`
4. Edit the `/etc/dphys-swapfile` so that `CONF_SWAPSIZE=4096` and `CONF_SWAPFACTOR=2`
5. Reboot the Jetson Nano (included in the script)
6. Run `sudo ./Install-Operation-Squirrel-dependencies.sh`
7. Run `sudo ./remove-swap.sh` to stop using swap