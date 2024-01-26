# Installing ORB-SLAM3 on the Jetson Nano

## Summary

This document will guide you through installing ORB-SLAM3 on the Jetson Nano.  The reason for using SLAM is to provide the distance of objects in the video frame to do obstacle avoidance, and to provide a vector for the drone to follow a moving target.  These instructions are stand alone.

ORB-SLAM3 repo: https://github.com/UZ-SLAMLab/ORB_SLAM3

## Required hardware

1. NVIDIA Jetson Nano
2. Camera

## Installation

1. Follow the instructions at https://qengineering.eu/install-opencv-on-jetson-nano.html to install OpenCV 4.5.0 (the ORB-SLAM3 repo was tested with 4.4.0 but I had a successful install with the slightly newer version)
    - Keep the swap memory enabled because you'll need it later
2. Install boost and ssl (I had to do this because either the one of the prerequisites failed or the full ORB-SLAM3 build failed due to not being able to find files contained in these libraries, and perhaps you won't need to, but just in case)
    ```
    sudo apt-get install libboost-all-dev
    sudo apt-get install libssl-dev
    ```
3. Install the prerequisites for ORB-SLAM3 which can be found at https://github.com/UZ-SLAMLab/ORB_SLAM3
4. Add an additional 4GB of swap for the ORB-SLAM3 build to avoid running out of memory while compiling.  This will bring you to a total of 8GB of swap (you should have added 4GB when you installed OpenCV).  This method adds swap in a different way
    ```
    sudo systemctl disable nvzramconfig
    sudo fallocate -l 4G /mnt/4GB.swap
    sudo mkswap /mnt/4GB.swap
    sudo swapon /mnt/4GB.swap
    ```
    
    Then add the following line to the end of /etc/fstab to make the change persistent:
    
    ```
    /mnt/4GB.swap  none  swap  sw 0  0
    ```
5. Clone the ORB-SLAM3 repository and build ORB-SLAM3 and its examples
    ```
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
    cd ORB_SLAM3
    chmod +x build.sh
    ./build.sh
    ```