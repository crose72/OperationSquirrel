#!/bin/bash

# -----------------------------------------------
# Install ORB_SLAM3
# -----------------------------------------------

echo "Installing ORB_SLAM3"

sudo ./Check-memory.sh

# ORB_SLAM3 full build https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3

# Run build script
chmod +x build.sh
./build.sh

cd ..

echo "ORB_SLAM3 installed, SUCCESS!"

